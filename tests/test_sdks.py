"""Run actual sibling SDK sessions against the Python virtual board.

Optional external SDKs skip when absent; set KINISI_REQUIRE_SDKS=1 in integration
CI to make missing checkouts/runtimes fail. No production SDK files are edited.
"""
import importlib
import os
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import unittest
from unittest.mock import patch

from kinisi_virtual_board import TCPServer, VirtualBoard, FirmwareBoard, VirtualSerial
from test_board import COMMANDS
from test_firmware import FIRMWARE

WORKSPACE = Path(__file__).resolve().parents[2]
PYSDK = Path(os.environ.get("KINISI_PYSDK", WORKSPACE / "pykinisi"))
JSSDK = Path(os.environ.get("KINISI_JSSDK", WORKSPACE / "jskinisi"))


def missing(reason):
    if os.environ.get("KINISI_REQUIRE_SDKS") == "1":
        raise AssertionError(reason)
    raise unittest.SkipTest(reason)


class PythonSDKTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not (PYSDK / "pykinisi" / "KinisiController.py").is_file():
            missing("Set KINISI_PYSDK to the Python SDK checkout")
        sys.path.insert(0, str(PYSDK))
        cls.sdk = importlib.import_module("pykinisi")
        cls.module = importlib.import_module("pykinisi.KinisiController")

    def setUp(self):
        self.board = VirtualBoard(COMMANDS)
        self.port = VirtualSerial(self.board, realtime=False, fragment_size=1)
        self.client = self.sdk.KinisiController(wall_clock=False, request_timeout=.15, init_timeout=1)
        self.addCleanup(self.client.disconnect)
        with patch.object(self.module.serial, "Serial", return_value=self.port):
            self.assertTrue(self.client.connect("virtual"), self.client.last_error)

    def test_real_codec_motor_encoder_position_and_errors(self):
        c = self.client
        self.assertTrue(c.ready)
        self.assertEqual(c.board_info.protocol_minor, self.board.schema.version[1])
        with self.assertRaises(self.sdk.ControllerError) as caught:
            c.set_motor_target_speed(0, 1)
        self.assertEqual(caught.exception.error_code, self.sdk.ErrorCode.CONTROLLER_NOT_INITIALIZED)
        c.initialize_motor_controller(0, False, 0, False, 1000, 1, 0, 0)
        c.initialize_motor_position_controller(0, 2, 2, .001)
        c.set_motor_position(0, 1)
        # Turn off heartbeat during deliberate long manual advances.
        c.set_heartbeat_config(False, 500)
        self.port.advance(4)
        self.assertAlmostEqual(c.get_motor_position(0), 1, delta=.002)
        c.start_encoder_odometry(0)
        c.set_motor_target_speed(0, 1)
        self.port.advance(.2)
        self.assertAlmostEqual(c.get_encoder_odometry(0).angle, .2)
        c.stop_motor(0)
        with self.assertRaises(self.sdk.ControllerError):
            c.set_motor_target_speed(0, 1)

    def test_real_typed_telemetry_callback(self):
        samples, arrived = [], threading.Event()
        def receive(sample):
            samples.append(sample)
            arrived.set()
        subscription = self.client.subscribe(self.sdk.Topic.TEMPERATURE, receive, interval_ms=250)
        self.port.advance(.3)
        self.assertTrue(arrived.wait(1), self.client.last_telemetry_error)
        self.assertEqual(samples[-1].celsius, 25.)
        self.assertEqual(samples[-1].timestamp_us, 300000)
        subscription.unsubscribe()
        self.assertFalse(subscription.active)
        arrived.clear()
        self.port.advance(.3)
        self.assertFalse(arrived.wait(.03))

    def test_dropped_ack_times_out_then_session_still_works(self):
        self.board.drop_next_reply("PING")
        with self.assertRaises(self.sdk.RequestTimeoutError):
            self.client.ping()
        self.client.ping()
        self.assertTrue(self.client.ready)

    def test_watchdog_surfaces_controller_error(self):
        self.client.initialize_motor(0, False)
        self.client.set_motor_speed(0, 25)
        self.port.advance(.6)
        with self.assertRaises(self.sdk.ControllerError) as caught:
            self.client.set_motor_speed(0, 50)
        self.assertEqual(caught.exception.error_code, self.sdk.ErrorCode.INIT_REQUIRED)
        self.assertEqual(self.board.motors[0].speed, 0)

    def test_wall_clock_handshake_and_periodic_sync(self):
        self.client.disconnect()
        self.board = VirtualBoard(COMMANDS)
        self.port = VirtualSerial(self.board, realtime=False)
        self.client = self.sdk.KinisiController(wall_clock=True, heartbeat_timeout_ms=None, init_timeout=1)
        self.addCleanup(self.client.disconnect)
        with patch.object(self.module.serial, "Serial", return_value=self.port):
            self.assertTrue(self.client.connect("virtual"), self.client.last_error)
        self.assertEqual(self.client.clock_mode, 1)
        self.client.set_time_sync_interval(1000)
        self.port.advance(1.1)
        self.client.ping()  # Reader services the preceding sync request before ACK.
        self.assertIsNone(self.client.last_sync_error)
        self.assertIsNone(self.board.sync_pending)

    def test_python_sdk_over_real_tcp(self):
        import serial
        with TCPServer(("127.0.0.1", 0), lambda: FirmwareBoard(COMMANDS, firmware=FIRMWARE)) as server:
            worker = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": .01})
            worker.start()
            try:
                client = self.sdk.KinisiController(wall_clock=True)
                address = f"socket://127.0.0.1:{server.server_address[1]}"
                def open_socket(_port, baudrate, **kwargs):
                    return serial.serial_for_url(address, baudrate=baudrate, **kwargs)
                try:
                    with patch.object(self.module.serial, "Serial", side_effect=open_socket):
                        self.assertTrue(client.connect("virtual-tcp"), client.last_error)
                    client.initialize_gpio_pin(1, 3)
                    client.set_gpio_pin_state(1, 1)
                    self.assertEqual(client.get_gpio_pin_state(1), 1)
                    client.ping()
                finally:
                    client.disconnect()
            finally:
                server.shutdown()
                worker.join(2)
                self.assertFalse(worker.is_alive())


class JavaScriptSDKTests(unittest.TestCase):
    def test_real_javascript_session_over_tcp(self):
        node = shutil.which("node")
        if not node or not (JSSDK / "src/commands/kinisi_session.js").is_file():
            missing("Node and KINISI_JSSDK checkout are required")
        with TCPServer(("127.0.0.1", 0), lambda: FirmwareBoard(COMMANDS, firmware=FIRMWARE)) as server:
            worker = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": .01})
            worker.start()
            try:
                result = subprocess.run([node, str(Path(__file__).with_name("sdk_integration.mjs")),
                                         str(JSSDK), str(server.server_address[1])],
                                        capture_output=True, text=True, timeout=15)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn("JavaScript SDK integration passed", result.stdout)
            finally:
                server.shutdown()
                worker.join(2)
                self.assertFalse(worker.is_alive())


if __name__ == "__main__":
    unittest.main()
