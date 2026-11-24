"""Production save/journal/restore through wire commands and fresh C runtimes."""
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
import zlib

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from kinisi_virtual_board import FirmwareBoard
from kinisi_virtual_board.firmware_build import DEFAULT_FIRMWARE
from test_board import frames

FIRMWARE = Path(os.environ.get("KINISI_FIRMWARE", DEFAULT_FIRMWARE))
LEGACY_IMAGES = Path(__file__).parent / "fixtures" / "configuration-v1"


class Session:
    def __init__(self, board):
        self.board = board
        self.id = 0

    def send(self, name, **values):
        self.id += 1
        self.board.feed(self.board.schema.request(name, self.id, **values))
        replies = frames(self.board.drain())
        return next((r for r in replies if r[1] == self.id), None)

    def ok(self, name, **values):
        response = self.send(name, **values)
        assert response and response[:2] == (self.board.schema.code(name), self.id), (name, response)
        return response[2]

    def error(self, name, error, **values):
        response = self.send(name, **values)
        assert response == (self.board.schema.code("ERROR"), self.id,
                            bytes([self.board.schema.code(name), self.board.schema.errors[error]])), (name, response)

    def initialize(self):
        self.ok("INIT", sdk_type=1, protocol_major=2,
                protocol_minor=self.board.schema.version[1], client_capabilities=2)
        assert self.board.ready

    def setup(self, kind=1, diameter=.12, frequency=25000):
        common = dict(is_reversed_0=1, is_reversed_1=0,
                      is_encoder_reversed_0=0, is_encoder_reversed_1=1,
                      encoder_resolution=2000)
        if kind == 1:
            self.ok("INITIALIZE_DIFFERENTIAL_PLATFORM", wheel_diameter=diameter,
                    wheel_base=.36, **common)
        elif kind == 2:
            self.ok("INITIALIZE_OMNI_PLATFORM", wheels_diameter=diameter, robot_radius=.23,
                    is_reversed_2=1, is_encoder_reversed_2=1, **common)
        else:
            self.ok("INITIALIZE_MECANUM_PLATFORM", wheels_diameter=diameter, length=.48, width=.31,
                    is_reversed_2=1, is_reversed_3=0,
                    is_encoder_reversed_2=1, is_encoder_reversed_3=0, **common)
        # Independent encoder scales must survive, not merely the common initial scale.
        self.ok("INITIALIZE_ENCODER", encoder_index=0, encoder_resolution=4096, is_reversed=1)
        self.ok("SET_PWM_FREQUENCY", frequency_hz=frequency)

    def field(self, field, index=0):
        return struct.unpack("<d", self.ok("GET_CONFIGURATION_FIELD", field=field, index=index))[0]

    def snapshot(self):
        return [self.field(i) for i in range(4)] + [
            self.field(field, i) for field in (4, 5, 6) for i in range(4)
        ] + [struct.unpack("<I", self.ok("GET_PWM_FREQUENCY"))[0]]


class ConfigurationTests(unittest.TestCase):
    def test_legacy_factory_and_saved_setup(self):
        expected = [3, .12, .48, .31, 4096, 2000, 2000, 2000, 1, 1, 1, 0, 1, 0, 1, 0, 25000]
        images = {
            "factory": "7d2c7ac4888bfd75cd5f56e8d61f69595121183afc81556c876732fd3782c62f",
            "mecanum": "d492e8f032c427407fb232d459b65d0fa23d13ffebef2c7026764bc4de66231e",
        }
        for preset, digest in images.items():
            image = (LEGACY_IMAGES / f"{preset}.fram").read_bytes()
            self.assertEqual(hashlib.sha256(image).hexdigest(), digest)
            with self.subTest(preset=preset), FirmwareBoard(
                firmware=FIRMWARE, fram_image=image
            ) as board:
                self.assertFalse(board.ready)
                self.assertEqual(board.motor_pwm, (0.,) * 4)
                for connection in ("boot", "reconnect"):
                    if connection == "reconnect":
                        board.disconnect()
                        self.assertFalse(board.ready)
                    session = Session(board)
                    session.initialize()
                    if board.schema.version >= (2, 19, 0):
                        self.assertEqual(struct.unpack("<Bd", session.ok("GET_FUSION_CONFIGURATION")), (0, 0.))
                    if board.schema.version >= (2, 20, 0):
                        self.assertEqual(session.ok("GET_IMU_TRACKING_STATUS")[0], 0)  # STOPPED
                    if preset == "factory":
                        self.assertEqual(session.field(0), 0)
                        self.assertEqual(struct.unpack("<I", session.ok("GET_PWM_FREQUENCY"))[0], 100000)
                        session.error("GET_CONFIGURATION_FIELD", "PLATFORM_NOT_INITIALIZED", field=1)
                        session.error("SET_PLATFORM_TARGET_VELOCITY", "PLATFORM_NOT_INITIALIZED", x=.1)
                    else:
                        self.assertEqual(session.snapshot(), expected)
                        session.error("SET_PLATFORM_TARGET_VELOCITY", "CONTROLLER_NOT_INITIALIZED", x=.1)
                    session.error("GET_PLATFORM_ODOMETRY", "ODOMETRY_NOT_INITIALIZED")
                    board.advance(.1)
                    self.assertEqual(board.motor_pwm, (0.,) * 4)

    def test_legacy_mecanum_tracks_encoders_and_disconnect_stops_pwm(self):
        board, session = self.board(fram_image=(LEGACY_IMAGES / "mecanum.fram").read_bytes())
        board.set_imu(status=1)
        session.error("GET_IMU", "SENSOR_NOT_AVAILABLE")
        session.ok("SET_CONTROLLER_FREQUENCY", frequency=100)
        session.ok("START_PLATFORM_CONTROLLER", kp=10, ki=20, kd=0, integral_limit=100)
        session.ok("SET_ODOMETRY_FREQUENCY", frequency=50)
        session.ok("START_PLATFORM_ODOMETRY")
        session.ok("SET_PLATFORM_TARGET_VELOCITY", x=.1)
        board.advance(.24)
        self.assertTrue(any(board.motor_pwm))
        # Yaw is additive across odometry ticks. Apply the saved channel directions
        # and scales to independently observed, wrapping encoder registers.
        ticks = [n if n < 32768 else n - 65536 for n in board.encoder_counts]
        wheel = [n * math.tau * sign / scale for n, sign, scale in
                 zip(ticks, (-1, -1, -1, 1), (4096, 2000, 2000, 2000))]
        yaw = .06 * (wheel[0] + wheel[1] - wheel[2] - wheel[3]) / (2 * (.48 + .31))
        pose = session.ok("GET_PLATFORM_ODOMETRY")
        stamp, _, _, x, y, heading = struct.unpack("<QBB3d", pose)
        self.assertEqual(stamp, board.now_us)
        self.assertGreater(math.hypot(x, y), .001)
        self.assertGreater(abs(yaw), .001)
        self.assertAlmostEqual(heading, yaw, places=10)
        board.disconnect()
        self.assertFalse(board.ready)
        self.assertEqual(board.motor_pwm, (0.,) * 4)
        session = Session(board)
        session.initialize()
        self.assertEqual(session.ok("GET_PLATFORM_ODOMETRY"), pose)
        board.advance(.2)
        self.assertEqual(board.motor_pwm, (0.,) * 4)

    def test_legacy_image_on_unsupported_boards(self):
        image = (LEGACY_IMAGES / "mecanum.fram").read_bytes()
        for revision in ("0.3.0", "0.3.1"):
            with self.subTest(revision=revision):
                board, session = self.board(revision=revision, fram_image=image)
                session.error("GET_CONFIGURATION_FIELD", "SENSOR_NOT_AVAILABLE", field=0)
                session.error("SAVE_CONFIGURATION", "SENSOR_NOT_AVAILABLE")
                if board.schema.version >= (2, 19, 0):
                    session.error("GET_FUSION_CONFIGURATION", "SENSOR_NOT_AVAILABLE")
                if board.schema.version >= (2, 20, 0):
                    session.error("GET_IMU_TRACKING_STATUS", "SENSOR_NOT_AVAILABLE")
                self.assertEqual(board.fram_transactions, (0, 0))
                self.assertEqual(board.fram_image, image)
                self.assertEqual(board.motor_pwm, (0.,) * 4)

    def board(self, **kwargs):
        board = FirmwareBoard(firmware=FIRMWARE, **kwargs)
        self.addCleanup(board.close)
        session = Session(board)
        session.initialize()
        return board, session

    def saved(self, kind=1):
        board, session = self.board()
        session.setup(kind)
        expected = session.snapshot()
        session.ok("SAVE_CONFIGURATION")
        return board.fram_image, expected

    def assert_stopped(self, board, session):
        self.assertEqual(board.motor_pwm, (0.,) * 4)
        self.assertEqual(board.motor_velocity, [0.] * 4)
        session.error("SET_PLATFORM_TARGET_VELOCITY", "CONTROLLER_NOT_INITIALIZED", x=.1)
        session.error("GET_PLATFORM_ODOMETRY", "ODOMETRY_NOT_INITIALIZED")
        board.advance(.1)
        self.assertEqual(board.drain(), b"")  # No persisted subscriptions/events.
        self.assertEqual(board.motor_pwm, (0.,) * 4)

    def test_platform_types_round_trip_in_new_runtime_stopped(self):
        for kind in (1, 2, 3):
            with self.subTest(platform=kind):
                image, expected = self.saved(kind)
                boot, session = self.board(fram_image=image)
                self.assertEqual(session.snapshot(), expected)
                self.assert_stopped(boot, session)
                self.assertIn("src/controller_configuration.c", boot.build["sources"])
                self.assertIn("lib/config_store/config_store.c", boot.build["sources"])

    def test_unsaved_and_transient_changes_do_not_replace_saved_setup(self):
        board, session = self.board()
        session.setup()
        expected = session.snapshot()
        session.ok("SAVE_CONFIGURATION")
        saved = board.fram_image
        session.setup(kind=3, diameter=.19, frequency=50000)
        session.ok("SET_CONTROLLER_FREQUENCY", frequency=100)
        session.ok("START_PLATFORM_CONTROLLER", kp=10, ki=20, kd=0, integral_limit=100)
        session.ok("SET_ODOMETRY_FREQUENCY", frequency=100)
        session.ok("START_PLATFORM_ODOMETRY")
        session.ok("SET_HEARTBEAT_CONFIG", enabled=1, timeout_ms=500)
        session.ok("SUBSCRIBE", topic_id=board.schema.topic_names["PLATFORM_ODOMETRY"]["id"],
                   index=0, interval_ms=20)
        session.ok("SET_PLATFORM_TARGET_VELOCITY", x=.1)
        board.advance(.05)
        self.assertTrue(any(board.motor_pwm))
        self.assertTrue(any(reply[1] == 0 for reply in frames(board.drain())))
        self.assertEqual(board.fram_image, saved)
        board.disconnect()
        session.initialize()
        self.assertEqual(session.field(0), 3)  # Reconnect is not boot/restore.
        self.assertEqual(session.field(1), .19)
        self.assertEqual(board.fram_image, saved)
        boot, restored = self.board(fram_image=board.fram_image)
        self.assertEqual(restored.snapshot(), expected)
        self.assert_stopped(boot, restored)

    def test_setup_without_save_is_not_persisted(self):
        board, session = self.board()
        empty = board.fram_image
        session.setup()
        self.assertEqual(board.fram_image, empty)
        boot, restored = self.board(fram_image=board.fram_image)
        self.assertEqual(restored.field(0), 0)
        self.assertEqual(boot.motor_pwm, (0.,) * 4)
        restored.error("GET_CONFIGURATION_FIELD", "PLATFORM_NOT_INITIALIZED", field=1)

    def test_torn_replacement_preserves_prior_committed_record(self):
        saved, expected = self.saved()
        # Journal writes: marker invalidation (1), body (212), commit marker (1).
        # Cover every cut, including the byte immediately before commit.
        for budget in range(214):
            with self.subTest(bytes_written=budget):
                with FirmwareBoard(firmware=FIRMWARE, fram_image=saved) as board:
                    session = Session(board)
                    session.initialize()
                    session.setup(kind=3, diameter=.19, frequency=50000)
                    board.set_fram_fault(write_after_bytes=budget)
                    session.error("SAVE_CONFIGURATION", "INTERNAL_ERROR")
                    torn = board.fram_image
                    self.assertEqual(torn[:256], saved[:256])
                with FirmwareBoard(firmware=FIRMWARE, fram_image=torn) as boot:
                    restored = Session(boot)
                    restored.initialize()
                    self.assertEqual(restored.snapshot(), expected)
                    self.assertEqual(boot.motor_pwm, (0.,) * 4)
        board, session = self.board(fram_image=saved)
        session.setup(kind=3, diameter=.19, frequency=50000)
        replacement = session.snapshot()
        board.set_fram_fault(write_after_bytes=214)
        session.ok("SAVE_CONFIGURATION")
        _, restored = self.board(fram_image=board.fram_image)
        self.assertEqual(restored.snapshot(), replacement)

    def test_missing_corrupt_wrong_version_and_board_are_safe(self):
        saved, _ = self.saved()
        variants = {"missing": bytes([255]) * FirmwareBoard.FRAM_SIZE}
        corrupted = bytearray(saved)
        corrupted[24] ^= 1
        variants["crc"] = bytes(corrupted)
        for label, offset, value in (("version", 8, 2), ("board", 14, 3)):
            image = bytearray(saved)
            image[offset] = value
            # Recalculate CRC so version/board checks, not just CRC, reject it.
            struct.pack_into("<I", image, 208, zlib.crc32(image[:208]))
            variants[label] = bytes(image)
        malformed = bytearray(saved)
        struct.pack_into("<d", malformed, 16, float("nan"))
        struct.pack_into("<I", malformed, 208, zlib.crc32(malformed[:208]))
        variants["invalid_geometry"] = bytes(malformed)
        for label, image in variants.items():
            with self.subTest(record=label):
                board, session = self.board(fram_image=image)
                self.assertEqual(session.field(0), 0)
                self.assertEqual(board.motor_pwm, (0.,) * 4)
                self.assertEqual(board.fram_image, image)
                self.assertEqual(board.fram_transactions[1], 0)

    def test_storage_read_failure_does_not_overwrite_or_mutate(self):
        saved, _ = self.saved()
        board, session = self.board(fram_image=saved, fram_read_failure=True)
        self.assertEqual(session.field(0), 0)
        session.setup()
        session.error("SAVE_CONFIGURATION", "INTERNAL_ERROR")
        self.assertEqual(board.fram_image, saved)
        self.assertEqual(board.fram_transactions[1], 0)

    def test_unsupported_profile_never_accesses_or_mutates_fram(self):
        image, _ = self.saved()
        board, session = self.board(revision="0.3.0", fram_image=image)
        session.ok("INITIALIZE_MOTOR", motor_index=0)
        session.ok("SET_MOTOR_SPEED", motor_index=0, pwm=25)
        session.error("SAVE_CONFIGURATION", "SENSOR_NOT_AVAILABLE")
        session.error("GET_CONFIGURATION_FIELD", "SENSOR_NOT_AVAILABLE", field=0)
        self.assertEqual(board.fram_image, image)
        self.assertEqual(board.fram_transactions, (0, 0))
        self.assertEqual(board.motor_pwm, (25., 0., 0., 0.))

    def test_saved_image_survives_separate_process_boot(self):
        with tempfile.TemporaryDirectory(prefix="kinisi-fram-test-") as directory:
            path = Path(directory) / "fram.bin"
            results = []
            for action in ("save", "boot"):
                result = subprocess.run([sys.executable, str(Path(__file__).resolve()),
                                         "--cold-worker", action, str(path)],
                                        cwd=ROOT, capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                results.append(json.loads(result.stdout))
            self.assertNotEqual(results[0]["pid"], results[1]["pid"])
            self.assertEqual(results[0]["configuration"], results[1]["configuration"])
            self.assertEqual(results[1]["pwm"], [0.] * 4)
            self.assertEqual(results[1]["time_us"], 0)
            self.assertEqual(results[1]["writes"], 0)


def cold_worker(action, path):
    image = Path(path).read_bytes() if action == "boot" else None
    with FirmwareBoard(firmware=FIRMWARE, fram_image=image) as board:
        session = Session(board)
        session.initialize()
        if action == "save":
            session.setup(kind=3)
            session.ok("SAVE_CONFIGURATION")
            Path(path).write_bytes(board.fram_image)
            session.ok("SET_CONTROLLER_FREQUENCY", frequency=100)
            session.ok("START_PLATFORM_CONTROLLER", kp=10, ki=20, kd=0, integral_limit=100)
            session.ok("SET_PLATFORM_TARGET_VELOCITY", x=.1)
            board.advance(.05)
            assert any(board.motor_pwm)
            board.drain()
        print(json.dumps(dict(pid=os.getpid(), configuration=session.snapshot(),
                              pwm=board.motor_pwm, time_us=board.now_us,
                              writes=board.fram_transactions[1])))


if __name__ == "__main__":
    if len(sys.argv) == 4 and sys.argv[1] == "--cold-worker":
        cold_worker(sys.argv[2], sys.argv[3])
    else:
        unittest.main()
