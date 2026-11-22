"""Wire-level driver sleep tests on the production command path and virtual motors."""
import struct
import unittest

from test_configuration import FIRMWARE, FirmwareBoard, Session, frames


class DriverSleepTests(unittest.TestCase):
    def board(self, revision="0.4.0", **kwargs):
        board = FirmwareBoard(firmware=FIRMWARE, revision=revision, **kwargs)
        self.addCleanup(board.close)
        if "SET_DRIVER_SLEEP" not in board.schema.commands:
            self.skipTest("Selected firmware predates driver sleep")
        session = Session(board)
        session.initialize()
        return board, session

    def raw(self, session, name, payload, error=None):
        board = session.board
        session.id += 1
        board.feed(board.schema.frame(name, session.id, payload))
        reply = frames(board.drain())
        code = board.schema.code(name)
        expected = ((code, session.id, b"") if error is None else
                    (board.schema.code("ERROR"), session.id, bytes((code, board.schema.errors[error]))))
        self.assertIn(expected, reply)

    def sleep(self, session, value, error=None):
        self.raw(session, "SET_DRIVER_SLEEP", bytes([value]), error)

    def assert_latch(self, session, value):
        self.assertEqual(session.ok("GET_DRIVER_SLEEP"), bytes([value]))

    def assert_stopped(self, board):
        self.assertEqual(board.motor_pwm, (0.,) * 4)
        board.advance(.12)
        self.assertEqual(board.motor_pwm, (0.,) * 4)

    def test_stopped_repeated_transitions_and_malformed_requests(self):
        board, session = self.board()
        self.assert_latch(session, 0)
        for name, payload, err in (("SET_DRIVER_SLEEP", b"", "INVALID_LENGTH"),
                                   ("SET_DRIVER_SLEEP", b"\x01\x00", "INVALID_LENGTH"),
                                   ("SET_DRIVER_SLEEP", b"\x02", "INVALID_ARGUMENT"),
                                   ("GET_DRIVER_SLEEP", b"\x01", "INVALID_LENGTH")):
            with self.subTest(name=name, payload=payload):
                self.raw(session, name, payload, err)
                self.assert_latch(session, 0)
        for value in (1, 1, 0, 0, 1, 0):
            self.sleep(session, value)
            self.assert_latch(session, value)
            self.assert_stopped(board)
        session.error("SET_MOTOR_SPEED", "MOTOR_NOT_INITIALIZED", motor_index=2, pwm=40)

    def test_moving_platform_reconnect_and_cold_restore(self):
        board, session = self.board()
        session.setup()
        expected = session.snapshot()
        session.ok("SAVE_CONFIGURATION")
        image = board.fram_image
        saved_write_count = board.fram_transactions[1]
        session.ok("SET_CONTROLLER_FREQUENCY", frequency=100)
        session.ok("START_PLATFORM_CONTROLLER", kp=10, ki=20, kd=0, integral_limit=100)
        session.ok("SET_PLATFORM_TARGET_VELOCITY", x=.1)
        board.advance(.05)
        self.assertTrue(any(board.motor_pwm))
        self.sleep(session, 1)
        self.assert_stopped(board)
        board.disconnect()
        session.initialize()
        self.assert_latch(session, 1)
        session.error("SET_PLATFORM_TARGET_VELOCITY", "DRIVERS_ASLEEP", x=.1)
        self.assertEqual(board.fram_image, image)
        self.sleep(session, 0)
        self.assert_stopped(board)
        self.assertEqual(session.snapshot(), expected)
        session.error("SET_PLATFORM_TARGET_VELOCITY", "CONTROLLER_NOT_INITIALIZED", x=.1)
        self.sleep(session, 1)
        self.assertEqual(board.fram_transactions[1], saved_write_count)
        boot, restored = self.board(fram_image=board.fram_image)
        self.assert_latch(restored, 0)
        self.assertEqual(restored.snapshot(), expected)
        self.assert_stopped(boot)

    def test_partial_wake_rolls_back_then_restores_effective_limits(self):
        board, session = self.board()
        settings = {0: (1, 6500), 2: (1, 4000)}
        for motor, (enabled, ma) in settings.items():
            session.ok("INITIALIZE_MOTOR", motor_index=motor)
            session.ok("SET_MOTOR_CURRENT_LIMIT", motor_index=motor, enabled=enabled, threshold_ma=ma)
            session.ok("SET_MOTOR_SPEED", motor_index=motor, pwm=30)
        self.assertEqual(board.motor_pwm, (30., 0., 30., 0.))
        self.sleep(session, 1)
        board.set_wake_readback_failure(2, True)
        self.sleep(session, 0, "INTERNAL_ERROR")
        self.assert_latch(session, 1)
        self.assert_stopped(board)
        for motor in range(4):
            self.assertEqual(session.ok("GET_MOTOR_FAULTS", motor_index=motor)[:2], b"\x01\x00")
        session.error("SET_MOTOR_SPEED", "DRIVERS_ASLEEP", motor_index=0, pwm=40)
        board.set_wake_readback_failure(2, False)
        self.sleep(session, 0)
        self.assert_latch(session, 0)
        self.assert_stopped(board)
        for motor, expected in settings.items():
            self.assertEqual(struct.unpack("<BH", session.ok("GET_MOTOR_CURRENT_LIMIT", motor_index=motor)), expected)
            session.ok("SET_MOTOR_SPEED", motor_index=motor, pwm=25)
        self.sleep(session, 0)  # Already awake: preserve active outputs.
        self.assertEqual(board.motor_pwm, (25., 0., 25., 0.))
        for motor in (1, 3):
            self.assertEqual(session.ok("GET_MOTOR_FAULTS", motor_index=motor)[:2], b"\x01\x00")
        session.error("SET_MOTOR_SPEED", "MOTOR_NOT_INITIALIZED", motor_index=1, pwm=25)

    def test_motion_buffered_at_wake_requires_fresh_request(self):
        for split in (1, 3, None):
            with self.subTest(split=split):
                board, session = self.board()
                session.ok("INITIALIZE_MOTOR", motor_index=0)
                self.sleep(session, 1)
                wake = board.schema.frame("SET_DRIVER_SLEEP", 90, bytes([0]))
                motion = board.schema.request("SET_MOTOR_SPEED", 91, motor_index=0, pwm=25)
                board.feed(wake + motion[:split])
                if split is not None:
                    board.feed(motion[split:])
                self.assertEqual(frames(board.drain()), [
                    (board.schema.code("SET_DRIVER_SLEEP"), 90, b""),
                    (board.schema.code("ERROR"), 91, bytes([board.schema.code("SET_MOTOR_SPEED"),
                                                            board.schema.errors["DRIVERS_ASLEEP"]]))])
                self.assert_stopped(board)
                session.ok("SET_MOTOR_SPEED", motor_index=0, pwm=25)
                self.assertEqual(board.motor_pwm[0], 25.)

    def test_v03_capability_matches_selected_firmware(self):
        board, session = self.board("0.3.0")
        session.ok("INITIALIZE_MOTOR", motor_index=0)
        session.ok("SET_MOTOR_SPEED", motor_index=0, pwm=15)
        if board.has_drv8870:
            self.sleep(session, 1)
            self.assert_latch(session, 1)
            self.assert_stopped(board)
            return
        self.sleep(session, 1, "SENSOR_NOT_AVAILABLE")
        session.error("GET_DRIVER_SLEEP", "SENSOR_NOT_AVAILABLE")
        self.assertEqual(board.motor_pwm, (15., 0., 0., 0.))
