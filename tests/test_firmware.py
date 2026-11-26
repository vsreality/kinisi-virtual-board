"""Exercise production C dispatch, controls and sessions with simulated hardware."""
import importlib
import json
import os
from pathlib import Path
import struct
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

from kinisi_virtual_board import FirmwareBoard, Schema, VirtualSerial
from kinisi_virtual_board.firmware_build import DEFAULT_FIRMWARE
from test_board import frames

FIRMWARE = Path(os.environ.get("KINISI_FIRMWARE", DEFAULT_FIRMWARE))
COMMANDS = FIRMWARE / "commands.json"


class FirmwareTests(unittest.TestCase):
    def setUp(self):
        self.board = FirmwareBoard(COMMANDS, firmware=FIRMWARE)
        self.addCleanup(self.board.close)
        self.schema = self.board.schema
        self.message_id = 0

    def send(self, name, **values):
        self.message_id += 1
        self.board.feed(self.schema.request(name, self.message_id, **values))
        return frames(self.board.drain())

    def ok(self, name, **values):
        replies = self.send(name, **values)
        response = next((r for r in replies if r[1] == self.message_id), None)
        self.assertIsNotNone(response, (name, replies))
        self.assertEqual(response[:2], (self.schema.code(name), self.message_id), (name, response))
        return response[2]

    def error(self, name, expected, **values):
        replies = self.send(name, **values)
        self.assertIn((self.schema.code("ERROR"), self.message_id,
                       bytes([self.schema.code(name), self.schema.errors[expected]])), replies)

    def initialize(self):
        return self.send("INIT", sdk_type=1, protocol_major=2, protocol_minor=self.schema.version[1], client_capabilities=2)

    def motor(self, reverse=False):
        self.ok("SET_CONTROLLER_FREQUENCY", frequency=100)
        self.ok("INITIALIZE_MOTOR_CONTROLLER", motor_index=0, encoder_index=0,
                is_reversed=reverse, is_encoder_reversed=reverse, encoder_resolution=10000,
                kp=10, ki=20, kd=0, integral_limit=100)

    def test_current_firmware_identity_and_fragmented_frames(self):
        data = self.schema.request("INIT", 42, sdk_type=1, protocol_major=2, client_capabilities=2)
        for byte in data[:-1]:
            self.board.feed(bytes([byte]))
            self.assertEqual(self.board.drain(), b"")
        self.board.feed(data[-1:] + self.schema.request("PING", 43))
        replies = frames(self.board.drain())
        self.assertEqual([r[:2] for r in replies], [(0x70, 42), (0x73, 42), (0x76, 43)])
        self.assertEqual(struct.unpack("<7BII", replies[0][2])[:7], (1, 0, 4, 0, *self.schema.version))
        self.assertIn("src/command_handler.c", self.board.build["sources"])
        # Existing firmware gap: keep it visible instead of supplying a fake reply.
        self.assertEqual(self.board.unsupported_commands, ["GET_PLATFORM_CURRENT_VELOCITY"])
        self.assertTrue(self.board.ready)
        self.error("GET_PLATFORM_CURRENT_VELOCITY", "UNKNOWN_COMMAND")

    def test_real_pid_updates_only_on_task_period_then_closes_loop(self):
        self.initialize()
        self.motor()
        self.ok("SET_MOTOR_TARGET_SPEED", speed=2)
        self.board.advance(.009)
        self.assertEqual(self.board.motor_pwm[0], 0)
        self.board.advance(.001)
        # Actual first C PID iteration: P=10*2, I=20*.01*2, D=0.
        expected = 20.4
        if "lib/hardware/hw_motor.c" in self.board.build["sources"]:
            expected = int(expected * 840 / 100) * 100 / 840
        self.assertAlmostEqual(self.board.motor_pwm[0], expected)
        self.assertEqual(self.board.motor_velocity[0], 0)
        self.board.advance(4)
        self.assertAlmostEqual(self.board.motor_velocity[0], 2, delta=.04)
        self.assertAlmostEqual(self.board.motor_pwm[0], 20, delta=.7)

    def test_position_controller_and_quantized_encoder_feedback(self):
        self.initialize()
        self.motor()
        self.ok("INITIALIZE_MOTOR_POSITION_CONTROLLER", kp=2, max_speed=2, tolerance=.01)
        self.ok("SET_MOTOR_POSITION", position=1)
        self.board.advance(6)
        position = struct.unpack("<d", self.ok("GET_MOTOR_POSITION"))[0]
        self.assertAlmostEqual(position, 1, delta=.015)
        self.assertAlmostEqual(position, self.board.motor_angle[0], delta=.002)
        self.ok("STOP_MOTOR")
        self.error("SET_MOTOR_POSITION", "CONTROLLER_NOT_INITIALIZED", position=2)

    def test_reverse_flags_provide_negative_feedback(self):
        self.initialize()
        self.motor(reverse=True)
        self.ok("SET_MOTOR_TARGET_SPEED", speed=2)
        self.board.advance(4)
        self.assertAlmostEqual(self.board.motor_velocity[0], -2, delta=.04)

    def test_differential_platform_world_odometry(self):
        self.initialize()
        self.ok("SET_CONTROLLER_FREQUENCY", frequency=100)
        self.ok("INITIALIZE_DIFFERENTIAL_PLATFORM", wheel_diameter=.1, wheel_base=.3, encoder_resolution=10000)
        self.ok("START_PLATFORM_CONTROLLER", kp=10, ki=20, kd=0, integral_limit=100)
        self.ok("START_PLATFORM_ODOMETRY")
        self.ok("SET_PLATFORM_TARGET_VELOCITY", x=.1, y=0, t=0)
        self.board.advance(3)
        timestamp, _, _, x, y, heading = struct.unpack("<QBB3d", self.ok("GET_PLATFORM_ODOMETRY"))
        self.assertEqual(timestamp, 3000000)
        self.assertGreater(x, .20)
        self.assertLess(x, .32)
        self.assertAlmostEqual(y, 0, places=5)
        self.assertAlmostEqual(heading, 0, places=5)
        self.error("SET_MOTOR_SPEED", "MOTOR_OWNED", pwm=10)

    def test_watchdog_stops_real_controller_and_requires_reinitialization(self):
        self.initialize()
        self.motor()
        self.ok("SET_HEARTBEAT_CONFIG", enabled=1, timeout_ms=100)
        self.ok("SET_MOTOR_TARGET_SPEED", speed=2)
        self.board.advance(.09)
        self.assertGreater(self.board.motor_pwm[0], 0)
        self.board.advance(.01)
        self.assertEqual(self.board.motor_pwm, (0., 0., 0., 0.))
        self.assertTrue(self.board.lost)
        self.board.drain()
        self.error("SET_MOTOR_TARGET_SPEED", "INIT_REQUIRED", speed=2)
        self.initialize()
        self.error("SET_MOTOR_TARGET_SPEED", "CONTROLLER_NOT_INITIALIZED", speed=2)

    def test_platform_position_runs_production_outer_and_velocity_loops(self):
        self.initialize()
        self.ok("SET_CONTROLLER_FREQUENCY", frequency=100)
        self.ok("INITIALIZE_DIFFERENTIAL_PLATFORM", wheel_diameter=.1, wheel_base=.3, encoder_resolution=10000)
        self.ok("START_PLATFORM_CONTROLLER", kp=10, ki=20, kd=0, integral_limit=100)
        self.ok("INITIALIZE_PLATFORM_POSITION_CONTROLLER", linear_kp=2, angular_kp=2,
                max_linear_speed=.15, max_angular_speed=1, position_tolerance=.005, heading_tolerance=.01)
        self.error("SET_PLATFORM_POSITION", "SAMPLE_NOT_AVAILABLE", x=.1, y=0, t=0)
        self.board.advance(.02)  # Production odometry must acquire its first sample.
        self.ok("SET_PLATFORM_POSITION", x=.1, y=0, t=0)
        self.board.advance(6)
        *_, x, y, heading = struct.unpack("<QBB3d", self.ok("GET_PLATFORM_ODOMETRY"))
        self.assertAlmostEqual(x, .1, delta=.007)
        self.assertAlmostEqual(y, 0, places=5)
        self.assertAlmostEqual(heading, 0, places=5)

    def test_encoder_counter_wrap_is_integrated_by_actual_odometry(self):
        self.initialize()
        self.ok("INITIALIZE_MOTOR")
        self.ok("INITIALIZE_ENCODER", encoder_resolution=10000)
        self.ok("START_ENCODER_ODOMETRY")
        self.ok("SET_MOTOR_SPEED", pwm=100)
        self.board.advance(5)
        stamp, _, _, angle = struct.unpack("<QBBd", self.ok("GET_ENCODER_ODOMETRY"))
        self.assertEqual(stamp, 5000000)
        self.assertGreater(angle, 42)  # More than 65536 counts at this resolution.
        self.assertAlmostEqual(angle, self.board.motor_angle[0], delta=.001)

    def test_driver_initialization_failure_reaches_command_handler(self):
        self.initialize()
        fail = (self.board.set_wake_readback_failure if "lib/hardware/hw_motor.c" in self.board.build["sources"]
                else self.board.set_fault)
        fail(0)
        self.error("INITIALIZE_MOTOR", "INTERNAL_ERROR")
        self.assertEqual(self.board.motor_pwm[0], 0)
        fail(0, False)
        self.ok("INITIALIZE_MOTOR")
        self.ok("SET_MOTOR_SPEED", pwm=25)
        self.assertEqual(self.board.motor_pwm[0], 25)

    def test_hardware_faults_and_current_limits_use_board_capabilities(self):
        self.initialize()
        self.ok("INITIALIZE_MOTOR")
        self.ok("SET_MOTOR_CURRENT_LIMIT", enabled=1, threshold_ma=5400)
        self.assertEqual(struct.unpack("<BH", self.ok("GET_MOTOR_CURRENT_LIMIT")), (1, 5400))
        self.error("SET_MOTOR_CURRENT_LIMIT", "INVALID_ARGUMENT", enabled=1, threshold_ma=10000)
        self.board.set_current(0, 1.25)
        self.assertEqual(struct.unpack("<d", self.ok("GET_MOTOR_CURRENT"))[0], 1.25)
        self.board.set_fault(0)
        self.error("SET_MOTOR_SPEED", "INTERNAL_ERROR", pwm=30)
        fault = self.ok("GET_MOTOR_FAULTS")
        self.assertEqual(fault[2], 1)
        self.ok("CLEAR_MOTOR_FAULTS")
        self.ok("SET_MOTOR_SPEED", pwm=30)
        self.assertEqual(self.board.motor_pwm[0], 30)

    def test_sensor_settling_injection_and_revision_absence(self):
        self.initialize()
        self.error("GET_TEMPERATURE", "SAMPLE_NOT_AVAILABLE")
        self.board.set_temperature(31.25)
        self.board.advance(.3)
        self.assertEqual(struct.unpack("<d", self.ok("GET_TEMPERATURE"))[0], 31.25)
        self.board.set_imu((1, 2, 3), (4, 5, 6))
        self.assertEqual(struct.unpack("<6d", self.ok("GET_IMU")), (1, 2, 3, 4, 5, 6))
        with FirmwareBoard(COMMANDS, firmware=FIRMWARE, revision="0.3.0") as older:
            older.feed(self.schema.request("INIT", protocol_major=2))
            older.drain()
            older.feed(self.schema.request("GET_TEMPERATURE", 2))
            self.assertEqual(frames(older.drain())[0][2][-1], self.schema.errors["SENSOR_NOT_AVAILABLE"])

    def test_instances_isolate_c_globals_and_shutdown_is_terminal(self):
        self.initialize()
        self.ok("INITIALIZE_MOTOR")
        self.ok("SET_MOTOR_SPEED", pwm=70)
        with FirmwareBoard(COMMANDS, firmware=FIRMWARE) as other:
            self.assertFalse(other.ready)
            self.assertEqual(other.motor_pwm, (0., 0., 0., 0.))
            other.advance(1)
            self.assertEqual(self.board.now_us, 0)
            self.assertEqual(self.board.motor_pwm[0], 70)
        with self.assertRaisesRegex(RuntimeError, "closed"):
            other.feed(b"")
        self.ok("PING")

    def test_malformed_frames_are_rejected_by_c_protocol(self):
        self.initialize()
        for raw, error in [(b"\0", "INVALID_LENGTH"), (b"\x03\xfe\x01\x00", "UNKNOWN_COMMAND"),
                           (b"\x03\x76\0\0", "INVALID_ARGUMENT"), (b"\x04\x76\x01\0\0", "INVALID_LENGTH")]:
            self.board.feed(raw)
            reply = frames(self.board.drain())[0]
            self.assertEqual(reply[0], self.schema.code("ERROR"))
            self.assertEqual(reply[2][-1], self.schema.errors[error])
        self.ok("PING")

    def test_schema_mismatch_fails_before_loading_firmware(self):
        document = dict(self.schema.document, version="2.255.0")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "commands.json"
            path.write_text(json.dumps(document))
            with self.assertRaisesRegex(ValueError, "must match"):
                FirmwareBoard(path, firmware=FIRMWARE)


class FirmwareSDKTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(os.environ.get("KINISI_PYSDK", FIRMWARE.parent / "pykinisi"))
        if not (path / "pykinisi/KinisiController.py").is_file():
            if os.environ.get("KINISI_REQUIRE_SDKS") == "1":
                raise AssertionError("Python SDK checkout required")
            raise unittest.SkipTest("Python SDK checkout required")
        sys.path.insert(0, str(path))
        cls.sdk = importlib.import_module("pykinisi")
        cls.module = importlib.import_module("pykinisi.KinisiController")

    def setUp(self):
        self.board = FirmwareBoard(COMMANDS, firmware=FIRMWARE)
        self.addCleanup(self.board.close)
        self.port = VirtualSerial(self.board, realtime=False, fragment_size=1)
        self.client = self.sdk.KinisiController(wall_clock=False, request_timeout=.2, init_timeout=2, heartbeat_timeout_ms=None)
        self.addCleanup(self.client.disconnect)
        with patch.object(self.module.serial, "Serial", return_value=self.port):
            self.assertTrue(self.client.connect("native-firmware"), self.client.last_error)

    def test_real_sdk_position_errors_and_timeout(self):
        c = self.client
        self.assertEqual(c.board_info.protocol_minor, self.board.schema.version[1])
        with self.assertRaises(self.sdk.ControllerError):
            c.set_motor_target_speed(0, 1)
        c.set_controller_frequency(100)
        c.initialize_motor_controller(0, False, 0, False, 10000, 10, 20, 0)
        c.initialize_motor_position_controller(0, 2, 2, .01)
        c.set_motor_position(0, 1)
        self.port.advance(6)
        self.assertAlmostEqual(c.get_motor_position(0), 1, delta=.015)
        self.board.drop_next_reply("PING")
        with self.assertRaises(self.sdk.RequestTimeoutError):
            c.ping()
        c.ping()

    def test_wall_clock_sync_with_realtime_transport(self):
        # A frozen board clock cannot measure a real host's round-trip interval.
        self.client.disconnect()
        self.port = VirtualSerial(self.board, realtime=True)
        c = self.sdk.KinisiController(wall_clock=True, init_timeout=2)
        self.addCleanup(c.disconnect)
        with patch.object(self.module.serial, "Serial", return_value=self.port):
            self.assertTrue(c.connect("native-realtime"), c.last_error)
        c.set_time_sync_interval(1000)
        threading.Event().wait(1.15)
        c.ping()
        self.assertEqual(c.clock_mode, 1)
        self.assertIsNone(c.last_sync_error)

    def test_configured_telemetry_uses_real_firmware_event_source(self):
        c, sdk = self.client, self.sdk
        c.set_heartbeat_config(True, 500)
        c.initialize_motor(0, False)
        samples, arrived = [], threading.Event()
        def receive(sample):
            samples.append(sample)
            arrived.set()
        sub = c.subscribe(sdk.Topic.MOTOR_FAULTS, receive, trigger=sdk.SubscriptionTrigger.ON_EVENT,
                          delivery=sdk.SubscriptionDelivery.ON_CHANGE)
        self.assertTrue(arrived.wait(1), c.last_telemetry_error)
        arrived.clear()
        self.board.set_fault(0)
        self.port.advance(.001)
        self.assertTrue(arrived.wait(1), c.last_telemetry_error)
        self.assertTrue(samples[-1].fault_active)
        sub.unsubscribe()
        self.assertFalse(sub.active)


if __name__ == "__main__":
    unittest.main()
