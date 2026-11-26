"""Native endpoints: real Linux PTYs and opt-in Windows virtual COM pairs."""
import importlib
import json
import os
from pathlib import Path
import select
import subprocess
import sys
import threading
import time
import unittest
from unittest.mock import patch

from kinisi_virtual_board import VirtualBoard, FirmwareBoard
from kinisi_virtual_board.native import PTYEndpoint, SerialEndpoint, serve_native
from test_board import COMMANDS, frames
from test_firmware import FIRMWARE


class NativeLoopTests(unittest.TestCase):
    def test_partial_writes_preserve_framing_and_stop_cleans_board(self):
        board = VirtualBoard(COMMANDS)
        stop = threading.Event()
        request = board.schema.request("INIT", 0x1234, protocol_major=2)
        class Endpoint:
            output = bytearray()
            def read(self):
                nonlocal request
                data, request = request[:1], request[1:]
                if len(self.output) == 24:
                    stop.set()
                return data
            def write(self, data):
                self.output.extend(data[:1])
                return 1
        endpoint = Endpoint()
        serve_native(board, endpoint, stop)
        self.assertEqual([(c, i) for c, i, _ in frames(endpoint.output)], [(0x70, 0x1234), (0x73, 0x1234)])
        self.assertFalse(board.ready)

    def test_write_failure_stops_board(self):
        board = VirtualBoard(COMMANDS)
        board.motors[0].speed = 3
        class Endpoint:
            def read(self):
                return board.schema.request("INIT", protocol_major=2)
            def write(self, _):
                raise OSError("port removed")
        with self.assertRaisesRegex(OSError, "port removed"):
            serve_native(board, Endpoint())
        self.assertTrue(board.lost)
        self.assertEqual(board.motors[0].speed, 0)

    def test_watchdog_runs_during_idle_serial_reads(self):
        board = VirtualBoard(COMMANDS)
        requests = b"".join([
            board.schema.request("INIT", 1, protocol_major=2),
            board.schema.request("SET_HEARTBEAT_CONFIG", 2, enabled=1, timeout_ms=100),
            board.schema.request("INITIALIZE_MOTOR", 3),
            board.schema.request("SET_MOTOR_SPEED", 4, pwm=50),
        ])
        stop = threading.Event()
        observed = []
        class Endpoint:
            count = 0
            def read(self):
                self.count += 1
                observed.append(board.motors[0].speed)
                if self.count == 4:
                    stop.set()
                return requests if self.count == 1 else b""
            def write(self, data):
                return len(data)
        with patch("kinisi_virtual_board.native.time.perf_counter_ns", side_effect=[0, 0, 60_000_000, 120_000_000, 180_000_000]):
            serve_native(board, Endpoint(), stop)
        self.assertEqual(observed, [0, 5, 5, 0])

    def test_stalled_writer_times_out(self):
        board = VirtualBoard(COMMANDS)
        class Endpoint:
            def read(self):
                return board.schema.request("INIT", protocol_major=2)
            def write(self, _):
                return 0
        with patch("kinisi_virtual_board.native.time.perf_counter_ns", side_effect=[0, 0, 2_000_000_000]):
            with self.assertRaisesRegex(TimeoutError, "one second"):
                serve_native(board, Endpoint())

    def test_serial_rejects_network_urls(self):
        with self.assertRaises(ValueError):
            SerialEndpoint("socket://localhost:8766")

    @unittest.skipUnless(os.name == "nt", "Windows-only diagnostic")
    def test_pty_on_windows_explains_serial_alternative(self):
        with self.assertRaisesRegex(RuntimeError, "--serial COMn"):
            with PTYEndpoint():
                self.fail("Windows has no POSIX PTY")


def load_sdk():
    sdk = Path(os.environ.get("KINISI_PYSDK", Path(__file__).resolve().parents[2] / "pykinisi"))
    if not (sdk / "pykinisi" / "KinisiController.py").is_file():
        raise unittest.SkipTest("Python SDK checkout required for native integration")
    sys.path.insert(0, str(sdk))
    return importlib.import_module("pykinisi")


class NativeSDKMixin:
    def exercise(self, client_port, board):
        sdk = load_sdk()
        # Normal SDK connect: no serial factory patch, injected transport, or URL.
        client = sdk.KinisiController(wall_clock=True, init_timeout=2)
        try:
            self.assertTrue(client.connect(client_port), client.last_error)
            client.initialize_motor_controller(0, False, 0, False, 1000, 1, 0, 0)
            client.set_motor_target_speed(0, 2)
            threading.Event().wait(.12)  # The real PID publishes state on its 100 ms task period.
            self.assertEqual(client.get_motor_controller_state(0).target_speed, 2)
            samples, arrived = [], threading.Event()
            def receive(sample):
                samples.append(sample)
                arrived.set()
            sub = client.subscribe(sdk.Topic.TEMPERATURE, receive, interval_ms=250)
            self.assertTrue(arrived.wait(2), client.last_telemetry_error)
            self.assertEqual(samples[-1].celsius, 25.)
            sub.unsubscribe()
            client.stop_motor(0)
            client.disconnect()
            self.assertTrue(client.connect(client_port), client.last_error)
            client.ping()
            self.assertTrue(client.ready)
        finally:
            client.disconnect()

    def run_endpoint(self, endpoint, client_port):
        board = FirmwareBoard(COMMANDS, firmware=FIRMWARE)
        self.addCleanup(board.close)
        stop = threading.Event()
        failures = []
        def run():
            try:
                serve_native(board, endpoint, stop)
            except Exception as error:
                failures.append(error)
        worker = threading.Thread(target=run, daemon=True)
        worker.start()
        try:
            self.exercise(client_port, board)
        finally:
            stop.set()
            worker.join(2)
        self.assertFalse(worker.is_alive(), "Native endpoint did not stop")
        self.assertEqual(failures, [])


@unittest.skipUnless(os.name == "posix", "Real PTY integration requires Linux/POSIX")
class LinuxPTYTests(NativeSDKMixin, unittest.TestCase):
    def test_unmodified_sdk_through_real_pty(self):
        with PTYEndpoint() as endpoint:
            path = endpoint.port
            self.assertTrue(os.path.exists(path))
            self.run_endpoint(endpoint, path)
        self.assertFalse(os.path.exists(path), "PTY should disappear when the emulator closes")

    def test_native_serial_endpoint_uses_real_pyserial(self):
        # Exercise the same SerialEndpoint used for Windows COM pairs, via Linux tty.
        with PTYEndpoint() as peer:
            with SerialEndpoint(peer.port) as endpoint:
                request = b"\x00\x0a\x0d\x11\x13\x7f\xff"
                os.write(peer.master, request)
                deadline = time.monotonic() + 1
                received = b""
                while received != request and time.monotonic() < deadline:
                    received += endpoint.read()
                self.assertEqual(received, request)
                self.assertEqual(endpoint.write(request), len(request))
                self.assertTrue(select.select([peer.master], [], [], 1)[0])
                self.assertEqual(os.read(peer.master, 4096), request)

    def test_cli_creates_native_port_without_socket(self):
        process = subprocess.Popen([sys.executable, "-m", "kinisi_virtual_board", "--pty",
                                    "--commands", str(COMMANDS), "--firmware", str(FIRMWARE)],
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            self.assertTrue(select.select([process.stdout], [], [], 30)[0], "CLI build/startup timed out")
            line = process.stdout.readline()
            if not line:
                self.fail(process.communicate(timeout=3)[1])
            report = json.loads(line)
            self.assertEqual(report["backend"], "firmware")
            self.assertEqual(report["transport"], "pty")
            self.exercise(report["serial_port"], None)
        finally:
            process.terminate()
            process.communicate(timeout=3)


@unittest.skipUnless(os.name == "nt", "Windows COM-pair integration")
class WindowsCOMTests(NativeSDKMixin, unittest.TestCase):
    def test_unmodified_sdk_through_virtual_com_pair(self):
        emulator = os.environ.get("KINISI_EMULATOR_PORT")
        client = os.environ.get("KINISI_CLIENT_PORT")
        if not emulator or not client:
            if os.environ.get("KINISI_REQUIRE_COM") == "1":
                self.fail("Set KINISI_EMULATOR_PORT and KINISI_CLIENT_PORT to an installed virtual COM pair")
            self.skipTest("No virtual COM pair configured; set KINISI_EMULATOR_PORT and KINISI_CLIENT_PORT")
        if emulator.casefold() == client.casefold():
            self.fail("The emulator and SDK must open different ends of the COM pair")
        with SerialEndpoint(emulator) as endpoint:
            self.run_endpoint(endpoint, client)


if __name__ == "__main__":
    unittest.main()
