"""Protocol and state-machine tests with a deterministic clock."""
import json
import math
import os
from pathlib import Path
import struct
import tempfile
import unittest

from kinisi_virtual_board import Schema, VirtualBoard

COMMANDS = Path(os.environ.get("KINISI_COMMANDS", Path(__file__).resolve().parents[2] / "kinisi-motor-controller-firmware" / "commands.json"))


def frames(data):
    result = []
    while data:
        size = data[0] + 1
        if len(data) < size:
            raise AssertionError("Truncated output")
        result.append((data[1], int.from_bytes(data[2:4], "little"), data[4:size]))
        data = data[size:]
    return result


class BoardTests(unittest.TestCase):
    def setUp(self):
        self.board = VirtualBoard(COMMANDS)
        self.schema = self.board.schema
        self.message_id = 0

    def send(self, name, **args):
        self.message_id += 1
        self.board.feed(self.schema.request(name, self.message_id, **args))
        return frames(self.board.drain())

    def ok(self, name, **args):
        reply = self.send(name, **args)[0]
        self.assertEqual(reply[:2], (self.schema.code(name), self.message_id))
        return reply[2]

    def error(self, name, error, **args):
        reply = self.send(name, **args)[0]
        self.assertEqual(reply, (self.schema.code("ERROR"), self.message_id,
                                 bytes([self.schema.code(name), self.schema.errors[error]])))

    def initialize(self, wall=False):
        return self.send("INIT", sdk_type=1, protocol_major=2, protocol_minor=1, client_capabilities=3 if wall else 2)

    def motor(self):
        self.ok("INITIALIZE_MOTOR_CONTROLLER", motor_index=0, encoder_index=0, encoder_resolution=1000, kp=1, ki=0, kd=0, integral_limit=100)

    def differential(self):
        self.ok("INITIALIZE_DIFFERENTIAL_PLATFORM", wheel_diameter=.1, wheel_base=.3, encoder_resolution=1000)

    def test_identity_ready_and_raw_ping_vector(self):
        result = self.initialize()
        self.assertEqual(result, [(0x70, 1, struct.pack("<7BII", 1, 0, 4, 0, *self.schema.version, 0, 0)), (0x73, 1, b"\0")])
        self.board.feed(bytes.fromhex("03 76 34 12"))
        self.assertEqual(self.board.drain(), bytes.fromhex("03 76 34 12"))

    def test_wall_clock_and_periodic_sync_are_not_acknowledged(self):
        result = self.initialize(True)
        self.assertEqual([r[0] for r in result], [0x70, 0x71])
        self.board.feed(self.schema.request("TIME_SYNC_RESPONSE", result[1][1], host_receive_us=1000000, host_send_us=1000000))
        self.assertEqual(frames(self.board.drain()), [(0x73, 1, b"\1")])
        self.board.advance(30)
        request = frames(self.board.drain())[0]
        self.assertEqual(request[0], 0x71)
        self.board.feed(self.schema.request("TIME_SYNC_RESPONSE", request[1], host_receive_us=31000000, host_send_us=31000000))
        self.assertEqual(self.board.drain(), b"")
        self.assertTrue(self.board.ready)

    def test_wrong_sync_id_is_rejected(self):
        self.initialize(True)
        self.error("TIME_SYNC_RESPONSE", "INVALID_ARGUMENT", host_receive_us=1, host_send_us=1)
        self.assertFalse(self.board.ready)

    def test_fragmented_and_coalesced_frames(self):
        request = self.schema.request("INIT", 500, protocol_major=2)
        for byte in request[:-1]:
            self.board.feed(bytes([byte]))
            self.assertEqual(self.board.drain(), b"")
        self.board.feed(request[-1:] + self.schema.request("PING", 501))
        self.assertEqual([(c, i) for c, i, _ in frames(self.board.drain())], [(0x70, 500), (0x73, 500), (0x76, 501)])

    def test_invalid_frames_and_direction(self):
        cases = [(b"\0", 0, 0, "INVALID_LENGTH"), (b"\x03\xfe\x01\x00", 0xfe, 1, "UNKNOWN_COMMAND"),
                 (b"\x03\x76\0\0", 0x76, 0, "INVALID_ARGUMENT"),
                 (b"\x04\x76\x01\0\0", 0x76, 1, "INVALID_LENGTH"),
                 (b"\x03\x73\x01\0", 0x73, 1, "UNKNOWN_COMMAND")]
        for raw, command, mid, error in cases:
            with self.subTest(error=error, raw=raw):
                self.board.feed(raw)
                self.assertEqual(frames(self.board.drain()), [(0x7f, mid, bytes([command, self.schema.errors[error]]))])

    def test_invalid_values_do_not_mutate_state(self):
        self.error("INITIALIZE_MOTOR", "INVALID_ARGUMENT", motor_index=4)
        self.error("INITIALIZE_MOTOR", "INVALID_ARGUMENT", is_reversed=2)
        self.error("SET_MOTOR_SPEED", "INVALID_ARGUMENT", pwm=math.nan)
        self.error("SET_MOTOR_SPEED", "INVALID_ARGUMENT", pwm=101)
        self.error("INITIALIZE_MOTOR_CONTROLLER", "INVALID_ARGUMENT", encoder_resolution=0)
        self.assertFalse(any(m.initialized for m in self.board.motors))

    def test_incompatible_init(self):
        self.error("INIT", "INCOMPATIBLE_PROTOCOL", protocol_major=1)
        self.error("INIT", "INCOMPATIBLE_PROTOCOL", protocol_major=2, protocol_minor=255)
        self.error("INIT", "INVALID_ARGUMENT", protocol_major=2, client_capabilities=4)

    def test_motor_prerequisites_and_position_lifecycle(self):
        self.error("SET_MOTOR_SPEED", "MOTOR_NOT_INITIALIZED", pwm=10)
        self.error("SET_MOTOR_POSITION", "CONTROLLER_NOT_INITIALIZED", position=1)
        self.motor()
        self.error("INITIALIZE_MOTOR_POSITION_CONTROLLER", "INVALID_ARGUMENT", kp=-1, max_speed=2, tolerance=.01)
        self.ok("INITIALIZE_MOTOR_POSITION_CONTROLLER", kp=2, max_speed=2, tolerance=.001)
        self.ok("SET_MOTOR_POSITION", position=1)
        self.board.advance(4)
        self.assertAlmostEqual(struct.unpack("<d", self.ok("GET_MOTOR_POSITION"))[0], 1, delta=.002)
        self.ok("RESET_MOTOR_POSITION")
        self.assertEqual(struct.unpack("<d", self.ok("GET_MOTOR_POSITION"))[0], 0)
        self.ok("SET_MOTOR_POSITION", position=2)
        self.ok("SET_MOTOR_TARGET_SPEED", speed=-1)
        self.board.advance(.5)
        self.assertAlmostEqual(struct.unpack("<d", self.ok("GET_MOTOR_POSITION"))[0], -.5)
        self.ok("STOP_MOTOR")
        self.error("SET_MOTOR_TARGET_SPEED", "CONTROLLER_NOT_INITIALIZED", speed=1)
        self.assertEqual(self.board.motors[0].mode, "coast")

    def test_watchdog_stops_at_deadline_and_requires_new_init(self):
        self.initialize()
        self.ok("SET_HEARTBEAT_CONFIG", enabled=True, timeout_ms=100)
        self.motor()
        self.ok("SET_MOTOR_TARGET_SPEED", speed=2)
        self.board.advance(1)
        self.assertAlmostEqual(self.board.motors[0].angle, .2)
        self.assertEqual(self.board.motors[0].speed, 0)
        self.error("SET_MOTOR_SPEED", "INIT_REQUIRED", pwm=50)
        self.error("PING", "INIT_REQUIRED")
        self.ok("STOP_MOTOR")
        self.initialize()
        self.ok("PING")

    def test_encoder_and_odometry(self):
        self.initialize()
        self.error("GET_ENCODER_VALUE", "ENCODER_NOT_INITIALIZED")
        self.error("GET_ENCODER_ODOMETRY", "ODOMETRY_NOT_INITIALIZED")
        self.motor()
        self.ok("START_ENCODER_ODOMETRY")
        self.error("GET_ENCODER_ODOMETRY", "SAMPLE_NOT_AVAILABLE")
        self.ok("SET_MOTOR_TARGET_SPEED", speed=math.tau)
        self.board.advance(.25)
        self.assertEqual(struct.unpack("<H", self.ok("GET_ENCODER_VALUE"))[0], 250)
        timestamp, mode, quality, angle = struct.unpack("<QBBd", self.ok("GET_ENCODER_ODOMETRY"))
        self.assertEqual((timestamp, mode, quality), (250000, 0, 1))
        self.assertAlmostEqual(angle, math.pi / 2)
        self.ok("INITIALIZE_ENCODER", encoder_resolution=2000)
        self.error("SET_MOTOR_TARGET_SPEED", "CONTROLLER_NOT_INITIALIZED", speed=1)

    def test_platform_ownership_and_world_odometry(self):
        self.initialize()
        self.error("SET_PLATFORM_TARGET_VELOCITY", "PLATFORM_NOT_INITIALIZED", x=1)
        self.differential()
        self.error("INITIALIZE_MOTOR", "MOTOR_OWNED")
        self.error("SET_PLATFORM_TARGET_VELOCITY", "CONTROLLER_NOT_INITIALIZED", x=1)
        self.ok("START_PLATFORM_CONTROLLER", kp=1, ki=0, kd=0, integral_limit=100)
        self.ok("START_PLATFORM_ODOMETRY")
        self.ok("SET_PLATFORM_TARGET_VELOCITY", x=.2, y=9, t=0)
        self.board.advance(1)
        result = struct.unpack("<QBB3d", self.ok("GET_PLATFORM_ODOMETRY"))
        self.assertAlmostEqual(result[3], .2)
        self.assertEqual(result[4:], (0, 0))
        self.ok("STOP_PLATFORM_CONTROLLER")
        self.error("SET_PLATFORM_TARGET_VELOCITY", "CONTROLLER_NOT_INITIALIZED", x=1)

    def test_platform_position_converges(self):
        self.initialize()
        self.differential()
        self.ok("START_PLATFORM_CONTROLLER", kp=1, integral_limit=100)
        self.ok("START_PLATFORM_ODOMETRY")
        self.board.advance(.2)
        self.ok("INITIALIZE_PLATFORM_POSITION_CONTROLLER", linear_kp=2, angular_kp=3, max_linear_speed=.3,
                max_angular_speed=1, position_tolerance=.005, heading_tolerance=.005)
        self.ok("SET_PLATFORM_POSITION", x=.3, y=.2, t=.5)
        self.board.advance(20)
        for actual, expected in zip(self.board.pose, (.3, .2, .5)):
            self.assertAlmostEqual(actual, expected, delta=.01)

    def test_platform_position_auto_odometry_and_cancellation(self):
        self.initialize()
        self.differential()
        self.ok("START_PLATFORM_CONTROLLER", kp=1, integral_limit=100)
        self.ok("INITIALIZE_PLATFORM_POSITION_CONTROLLER", linear_kp=2, angular_kp=3, max_linear_speed=.3,
                max_angular_speed=1, position_tolerance=.005, heading_tolerance=.005)
        self.assertTrue(self.board.platform_odometry)
        self.error("SET_PLATFORM_POSITION", "SAMPLE_NOT_AVAILABLE", x=1)
        self.board.advance(.1)
        self.ok("SET_PLATFORM_POSITION", x=1)
        self.ok("SET_PLATFORM_TARGET_VELOCITY", x=.1)
        self.error("SET_PLATFORM_POSITION", "CONTROLLER_NOT_INITIALIZED", x=1)
        self.ok("SET_PLATFORM_VELOCITY", x=50)
        self.error("SET_PLATFORM_TARGET_VELOCITY", "CONTROLLER_NOT_INITIALIZED", x=.1)
        self.assertAlmostEqual(self.board.velocity[0], .25)

    def test_platform_wheel_kinematics(self):
        self.initialize()
        cases = [
            ("INITIALIZE_DIFFERENTIAL_PLATFORM", dict(wheel_diameter=.1, wheel_base=.3), (4., 4.)),
            ("INITIALIZE_OMNI_PLATFORM", dict(wheels_diameter=.1, robot_radius=.2), (2 * math.sqrt(3), -2 * math.sqrt(3), 0.)),
            ("INITIALIZE_MECANUM_PLATFORM", dict(wheels_diameter=.1, length=.2, width=.3), (4., 4., 4., 4.)),
        ]
        for command, dimensions, expected in cases:
            with self.subTest(platform=command):
                self.ok(command, encoder_resolution=1000, **dimensions)
                self.ok("START_PLATFORM_CONTROLLER", kp=1, integral_limit=100)
                self.ok("START_PLATFORM_ODOMETRY")
                self.ok("SET_PLATFORM_TARGET_VELOCITY", x=.2)
                self.board.advance(.1)
                for index, speed in enumerate(expected):
                    self.assertAlmostEqual(self.board.motors[index].speed, speed)
                    sample = struct.unpack("<QBBd", self.ok("GET_ENCODER_ODOMETRY", encoder_index=index))
                    self.assertAlmostEqual(sample[-1], speed * .1)

    def test_hardware_profiles(self):
        for revision in ("v0.3", "v0.4"):
            profile = COMMANDS.parent / "boards" / (revision + ".json")
            if not profile.is_file():
                self.skipTest("Optional firmware board profiles unavailable")
            board = VirtualBoard(COMMANDS, profile)
            self.assertEqual(board.revision, (0, 3 if revision == "v0.3" else 4, 0))
            self.assertEqual(board.sensor_counts["temperature"], int(revision == "v0.4"))

    def test_sensor_presence_settling_and_fault_injection(self):
        self.initialize()
        self.error("GET_TEMPERATURE", "SAMPLE_NOT_AVAILABLE")
        self.board.advance(.3)
        self.assertEqual(struct.unpack("<d", self.ok("GET_TEMPERATURE"))[0], 25.)
        self.assertEqual(struct.unpack("<6d", self.ok("GET_IMU"))[2], 9.80665)
        self.error("GET_IMU", "SENSOR_NOT_AVAILABLE", sensor_index=1)
        self.board.sensor_errors["temperature", 0] = "INTERNAL_ERROR"
        self.error("GET_TEMPERATURE", "INTERNAL_ERROR")
        self.board = VirtualBoard(COMMANDS, revision="0.3.0")
        self.initialize()
        self.error("GET_TEMPERATURE", "SENSOR_NOT_AVAILABLE")

    def test_topic_subscription_and_unsubscribe(self):
        self.initialize()
        self.error("SUBSCRIBE", "INVALID_ARGUMENT", topic_id=3, interval_ms=250)
        self.ok("SET_HEARTBEAT_CONFIG", enabled=True, timeout_ms=500)
        self.error("SUBSCRIBE", "INVALID_ARGUMENT", topic_id=3, interval_ms=249)
        self.ok("SUBSCRIBE", topic_id=3, interval_ms=250)
        self.board.advance(.3)
        event = frames(self.board.drain())[0]
        self.assertEqual(event[:2], (0x82, 0))
        self.assertEqual(event[2][:2], b"\3\0")
        self.assertEqual(struct.unpack("<QBBd", event[2][2:]), (300000, 0, 1, 25.))
        self.ok("UNSUBSCRIBE", topic_id=3)
        self.board.advance(.3)
        self.assertEqual(self.board.drain(), b"")

    def test_legacy_odometry_and_frequency_guard(self):
        self.initialize()
        self.motor()
        self.ok("START_ENCODER_ODOMETRY")
        self.ok("SET_HEARTBEAT_CONFIG", enabled=True, timeout_ms=500)
        self.error("SUBSCRIBE_ODOMETRY", "INVALID_ARGUMENT", source=0, interval_ms=100)
        self.ok("SUBSCRIBE_ODOMETRY", source=0, interval_ms=200)
        self.error("SET_ODOMETRY_FREQUENCY", "INVALID_ARGUMENT", frequency=5)
        self.board.advance(.2)
        event = frames(self.board.drain())[0]
        self.assertEqual(event[:2], (0x7b, 0))
        self.assertEqual(event[2][0], 0)
        self.ok("UNSUBSCRIBE_ODOMETRY", source=0)
        self.ok("SET_ODOMETRY_FREQUENCY", frequency=5)

    def test_faults_and_dropped_ack_have_different_side_effects(self):
        self.initialize()
        self.ok("INITIALIZE_MOTOR")
        self.board.fail_next("SET_MOTOR_SPEED")
        self.error("SET_MOTOR_SPEED", "INTERNAL_ERROR", pwm=50)
        self.assertEqual(self.board.motors[0].speed, 0)
        self.board.drop_next_reply("SET_MOTOR_SPEED")
        self.assertEqual(self.send("SET_MOTOR_SPEED", pwm=50), [])
        self.assertEqual(self.board.motors[0].speed, 5)

    def test_gpio_and_quantized_frequencies(self):
        self.ok("INITIALIZE_GPIO_PIN", pin_number=1, mode=3)
        self.ok("SET_GPIO_PIN_STATE", pin_number=1, state=1)
        self.assertEqual(self.ok("GET_GPIO_PIN_STATE", pin_number=1), b"\1")
        self.ok("TOGGLE_GPIO_PIN_STATE", pin_number=1)
        self.assertEqual(self.ok("GET_GPIO_PIN_STATE", pin_number=1), b"\0")
        self.ok("SET_CONTROLLER_FREQUENCY", frequency=600)
        self.assertEqual(struct.unpack("<H", self.ok("GET_CONTROLLER_FREQUENCY"))[0], 1000)

    def test_schema_is_loaded_not_copied_or_ids_hardcoded(self):
        document = json.loads(COMMANDS.read_text())
        next(c for c in document["commands"] if c["command"] == "PING")["code"] = "0xEE"
        document["commands"].append({"command": "FUTURE_COMMAND", "code": "0xEF", "direction": "client_to_controller", "properties": []})
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "commands.json"
            path.write_text(json.dumps(document))
            board = VirtualBoard(path)
            board.feed(board.schema.request("INIT", protocol_major=2))
            board.drain()
            board.feed(board.schema.request("PING", 20))
            self.assertEqual(frames(board.drain()), [(0xee, 20, b"")])
            board.feed(board.schema.request("FUTURE_COMMAND", 21))
            self.assertEqual(frames(board.drain()), [(0x7f, 21, b"\xef\3")])
            self.assertEqual(board.unsupported_commands, sorted(self.board.unsupported_commands + ["FUTURE_COMMAND"]))

    def test_legacy_model_reports_new_firmware_commands_as_unsupported(self):
        # These features are now tested through actual firmware, not duplicated
        # into the older behavioral model. --backend python --check fails here.
        expected = [
            "CLEAR_MOTOR_FAULTS", "GET_MOTOR_CURRENT", "GET_MOTOR_CURRENT_LIMIT",
            "GET_MOTOR_FAULTS", "SET_MOTOR_CURRENT_LIMIT", "SUBSCRIBE_CONFIGURED", "SUBSCRIBE_WITH_MODE"]
        expected += [name for name in ("GET_PWM_FREQUENCY", "SET_PWM_FREQUENCY",
                                      "SAVE_CONFIGURATION", "GET_CONFIGURATION_FIELD", "SET_DRIVER_SLEEP", "GET_DRIVER_SLEEP",
                                      "SET_BOARD_IDLE", "GET_BOARD_IDLE", "SET_FUSION_EXPERIMENT",
                                      "GET_FUSION_STATUS", "GET_FUSION_VECTOR", "GET_FUSION_CONFIGURATION",
                                      "SET_IMU_TRACKING", "GET_IMU_TRACKING_STATUS", "GET_IMU_TRACKING_VECTOR",
                                      "GET_IMU_TRACKING_CONFIGURATION")
                     if name in self.board.schema.commands]
        self.assertEqual(self.board.unsupported_commands, sorted(expected))


if __name__ == "__main__":
    unittest.main()
