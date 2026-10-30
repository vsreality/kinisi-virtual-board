"""Standalone executable tests: production protocol and real SDK serial traffic."""
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import threading
import time
import unittest

from kinisi_virtual_board import FirmwareBoard, Schema
from test_board import COMMANDS, frames
from test_native import NativeSDKMixin

ROOT = Path(__file__).resolve().parents[1]
DEFAULT = ROOT / ("build/native/kinisi-sim.exe" if os.name == "nt" else "build/native-linux/kinisi-sim")
BINARY = Path(os.environ.get("KINISI_NATIVE_BINARY", DEFAULT)).resolve()


def require_binary():
    if not BINARY.is_file():
        if os.environ.get("KINISI_REQUIRE_NATIVE") == "1":
            raise AssertionError("Build standalone executable first: tools/build_native.py")
        raise unittest.SkipTest("Build standalone executable or set KINISI_NATIVE_BINARY")


class StandaloneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        require_binary()
        cls.schema = Schema(COMMANDS)

    def request(self, name, **values):
        return self.schema.request(name, 42, **values)

    def run_operations(self, operations):
        data = b"".join(op.encode() + struct.pack("<I", len(payload)) + payload for op, payload in operations)
        result = subprocess.run([str(BINARY), "--stdio-test"], input=data, capture_output=True, timeout=20, cwd=ROOT.parent)
        self.assertEqual(result.returncode, 0, result.stderr.decode(errors="replace"))
        responses, remaining = [], result.stdout
        while remaining:
            self.assertGreaterEqual(len(remaining), 4)
            length = struct.unpack("<I", remaining[:4])[0]
            self.assertGreaterEqual(len(remaining), length + 4)
            responses.append(remaining[4:4+length])
            remaining = remaining[4+length:]
        self.assertEqual(len(responses), len(operations))
        return responses

    def init(self):
        return ("F", self.request("INIT", sdk_type=1, protocol_major=2,
                                  protocol_minor=self.schema.version[1], client_capabilities=2))

    def motor_operations(self, reverse=False):
        return [self.init(), ("F", self.request("SET_CONTROLLER_FREQUENCY", frequency=100)),
                ("F", self.request("INITIALIZE_MOTOR_CONTROLLER", motor_index=0, encoder_index=0,
                                   is_reversed=reverse, is_encoder_reversed=reverse, encoder_resolution=10000,
                                   kp=10, ki=20, kd=0, integral_limit=100))]

    def test_fragmentation_and_native_identity(self):
        init = self.init()[1]
        results = self.run_operations([*(('F', bytes([byte])) for byte in init),
                                       ('F', self.request('PING'))])
        self.assertTrue(all(not r for r in results[:-2]))
        identity = frames(results[-2])[0]
        self.assertEqual(struct.unpack('<7BII', identity[2])[:7], (1, 0, 4, 0, *self.schema.version))
        self.assertEqual(frames(results[-1])[0][:2], (self.schema.code('PING'), 42))

    def test_cpp_and_python_plants_match_real_firmware(self):
        for reverse in (False, True):
            with self.subTest(reverse=reverse), FirmwareBoard(COMMANDS) as board:
                operations = self.motor_operations(reverse)
                operations += [('F', self.request('SET_MOTOR_TARGET_SPEED', speed=2)),
                               ('A', struct.pack('<Q', 10000)),
                               ('F', self.request('GET_MOTOR_CONTROLLER_STATE')),
                               ('A', struct.pack('<Q', 3990000)),
                               ('F', self.request('GET_MOTOR_CONTROLLER_STATE'))]
                expected = []
                for op, data in operations:
                    if op == 'F': board.feed(data)
                    else: board.advance(struct.unpack('<Q', data)[0] / 1e6)
                    expected.append(board.drain())
                self.assertEqual(self.run_operations(operations), expected)

    def test_position_control_reaches_target(self):
        operations = self.motor_operations() + [
            ('F', self.request('INITIALIZE_MOTOR_POSITION_CONTROLLER', kp=2, max_speed=2, tolerance=.01)),
            ('F', self.request('SET_MOTOR_POSITION', position=1)),
            ('A', struct.pack('<Q', 6000000)), ('F', self.request('GET_MOTOR_POSITION'))]
        position = struct.unpack('<d', frames(self.run_operations(operations)[-1])[0][2])[0]
        self.assertAlmostEqual(position, 1, delta=.015)

    def test_watchdog_and_disconnect_require_new_session(self):
        for expire in (True, False):
            operations = self.motor_operations() + [('F', self.request('SET_HEARTBEAT_CONFIG', enabled=1, timeout_ms=100))]
            operations += [('A', struct.pack('<Q', 100000))] if expire else [('D', b'')]
            operations += [('F', self.request('SET_MOTOR_TARGET_SPEED', speed=2)), self.init(), ('F', self.request('PING'))]
            results = self.run_operations(operations)
            error = frames(results[-3])[-1]
            self.assertEqual(error, (self.schema.code('ERROR'), 42,
                                    bytes([self.schema.code('SET_MOTOR_TARGET_SPEED'), self.schema.errors['INIT_REQUIRED']])))
            self.assertEqual(frames(results[-1])[0][0], self.schema.code('PING'))

    def test_pwm_frequency_uses_board_catalog(self):
        if 'SET_PWM_FREQUENCY' not in self.schema.commands:
            self.skipTest('Selected firmware predates configurable PWM frequency')
        # Command fields come from the selected schema.
        operations = [self.init(), ('F', self.request('GET_PWM_FREQUENCY')),
                      ('F', self.request('SET_PWM_FREQUENCY', frequency_hz=25000)),
                      ('D', b''), self.init(), ('F', self.request('GET_PWM_FREQUENCY')),
                      ('F', self.request('SET_PWM_FREQUENCY', frequency_hz=12345))]
        results = self.run_operations(operations)
        self.assertEqual(struct.unpack('<I', frames(results[1])[0][2])[0], 100000)
        self.assertEqual(struct.unpack('<I', frames(results[-2])[0][2])[0], 25000)
        self.assertEqual(frames(results[-1])[0][0], self.schema.code('ERROR'))

    def test_malformed_harness_input_is_rejected(self):
        for data in (b'F\xff\xff\xff\xff', b'A\x08\0\0\0\x01', b'Z\0\0\0\0'):
            result = subprocess.run([str(BINARY), '--stdio-test'], input=data, capture_output=True, timeout=3)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn(b'Kinisi simulator:', result.stderr)


class StandaloneSerialTests(NativeSDKMixin, unittest.TestCase):
    def windows_serial_contract(self, port):
        import ctypes as ct
        import serial
        from serial import win32
        from serial.tools import list_ports
        self.assertIn(port, [p.device for p in list_ports.comports()])
        with serial.Serial(port, timeout=.05) as stream:
            with self.assertRaises(serial.SerialException):
                with serial.Serial(port):
                    pass
            # An incomplete manual-queue read must survive timer polls until timeout.
            start = time.monotonic()
            self.assertEqual(stream.read(16), b'')
            self.assertGreaterEqual(time.monotonic()-start, .03)
            stream.timeout = None
            results, errors = [], []
            def blocking_read():
                try: results.append(stream.read(16))
                except Exception as error: errors.append(error)
            reader = threading.Thread(target=blocking_read, daemon=True)
            reader.start();time.sleep(.05)
            self.assertTrue(reader.is_alive(), errors)
            stream.cancel_read();reader.join(2)
            self.assertFalse(reader.is_alive(), 'CancelIoEx must complete queued reads')
            self.assertEqual(errors, [])
            self.assertEqual(results, [b''])
            handle = stream._port_handle
            self.assertTrue(win32.SetCommMask(handle, win32.EV_TXEMPTY))
            overlap = win32.OVERLAPPED()
            overlap.hEvent = win32.CreateEvent(None, True, False, None)
            self.assertTrue(overlap.hEvent)
            mask, transferred = win32.DWORD(), win32.DWORD()
            try:
                for transmit in (True, False):
                    win32.ResetEvent(overlap.hEvent)
                    completed = win32.WaitCommEvent(handle, ct.byref(mask), ct.byref(overlap))
                    self.assertFalse(completed, 'An idle board must not report a transmission')
                    self.assertEqual(win32.GetLastError(), win32.ERROR_IO_PENDING)
                    self.assertEqual(win32.WaitForSingleObject(overlap.hEvent, 100), 258)
                    if transmit:
                        stream.write(Schema(COMMANDS).request('PING', 7))
                        self.assertEqual(win32.WaitForSingleObject(overlap.hEvent, 2000), 0)
                        self.assertTrue(win32.GetOverlappedResult(handle, ct.byref(overlap), ct.byref(transferred), True))
                        self.assertEqual(mask.value, win32.EV_TXEMPTY)
                    else:
                        win32.CancelIoEx(handle, ct.byref(overlap))
                        win32.GetOverlappedResult(handle, ct.byref(overlap), ct.byref(transferred), True)
            finally:
                win32.CancelIoEx(handle, ct.byref(overlap))
                win32.GetOverlappedResult(handle, ct.byref(overlap), ct.byref(transferred), True)
                win32.CloseHandle(overlap.hEvent)

    def test_real_sdk_on_single_native_port(self):
        require_binary()
        if os.name == 'nt' and os.environ.get('KINISI_TEST_WINDOWS_DRIVER') != '1':
            self.skipTest('Install signed Kinisi driver and set KINISI_TEST_WINDOWS_DRIVER=1')
        process = subprocess.Popen([str(BINARY)], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        lines = []
        reader = threading.Thread(target=lambda: lines.append(process.stdout.readline()), daemon=True)
        reader.start()
        port = None
        try:
            reader.join(35)
            self.assertFalse(reader.is_alive(), 'Native port startup timed out')
            self.assertTrue(lines and lines[0], 'Native port startup failed')
            report = json.loads(lines[0]);port = report['serial_port']
            self.assertEqual(report['backend'], 'native-cpp')
            self.exercise(port, None)
            if os.name == 'nt':self.windows_serial_contract(port)
        finally:
            process.terminate()
            _, errors = process.communicate(timeout=5)
        if os.name != 'nt':
            self.assertEqual(process.returncode, 0, errors)
            self.assertFalse(Path(port).exists(), 'PTY must disappear on simulator exit')
