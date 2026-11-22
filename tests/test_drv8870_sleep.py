"""Protocol 2.17 DRV8870 release cases on the native production motor and transport paths."""
import json
import unittest

from test_configuration import FIRMWARE, FirmwareBoard, frames


REVISIONS = ("0.3.0", "0.3.1")
BUSES = ("usb", "i2c")
GPIO_LOW = ("gpio", 0)
COAST = ("timer", 0)
HIGH = ("timer", 840)


class Peer:
    def __init__(self, board, bus):
        self.board = board
        self.bus = bus
        self.sequence = 0

    def frame(self, command, **fields):
        self.sequence += 1
        return self.board.schema.request(command, self.sequence, **fields)

    def write(self, data, *, start=True):
        if self.bus == "usb":
            self.board.feed(data)
        else:
            self.board.i2c_write(data, start=start)

    def replies(self):
        if self.bus == "usb":
            return frames(self.board.drain())
        replies = []
        for _ in range(32):
            packet = self.board.i2c_read()
            if not packet:
                return replies
            end = packet[0] + 1
            assert len(packet) == 256 and not any(packet[end:]), "invalid I2C reply padding"
            replies.extend(frames(packet[:end]))
        raise AssertionError("I2C reply queue failed to drain")

    def issue(self, command, error=None, **fields):
        self.write(self.frame(command, **fields))
        code = self.board.schema.code(command)
        reply_codes = (code, self.board.schema.code("ERROR"))
        result, = [r for r in self.replies() if r[1] == self.sequence and r[0] in reply_codes]
        if error:
            assert result == (self.board.schema.code("ERROR"), self.sequence,
                              bytes([code, self.board.schema.errors[error]])), result
        else:
            assert result[:2] == (code, self.sequence), result
        return result[2]

    def initialize(self):
        identity = self.issue("INIT", sdk_type=1, protocol_major=2,
                              protocol_minor=self.board.schema.version[1], client_capabilities=2)
        assert identity[:4] == bytes((1, *self.board.revision)), identity


class Drv8870Fixture(unittest.TestCase):
    minimum_version = (2, 17, 0)

    def setUp(self):
        version = json.loads((FIRMWARE / "commands.json").read_text())["version"]
        if tuple(map(int, version.split("."))) < self.minimum_version:
            self.skipTest(f"Requires firmware protocol {self.minimum_version}")

    def board(self, revision):
        board = FirmwareBoard(firmware=FIRMWARE, revision=revision)
        self.addCleanup(board.close)
        return board

    def pairs(self, board):
        last = 9 if board.revision[-1] == 0 else 10
        pins = (("B", 14), ("B", 15), ("C", 6), ("C", 7),
                ("C", 8), ("C", 9), ("A", 8), ("A", last))
        return [tuple(board.input_pad(*pin) for pin in pins[i:i+2]) for i in range(0, 8, 2)]

    def held(self, board):
        self.assertEqual(self.pairs(board), [(GPIO_LOW, GPIO_LOW)] * 4)
        self.assertEqual(board.motor_pwm, (0.,) * 4)


class Drv8870Tests(Drv8870Fixture):
    def test_production_timer_and_pad_assignments(self):
        for revision in REVISIONS:
            with self.subTest(revision=revision):
                b = self.board(revision)
                p = Peer(b, "usb")
                p.initialize()
                nano = revision == "0.3.1"
                self.assertEqual(b.build["build_define"], "MP_V3_1" if nano else "MP_V3")
                # Chip pad mux and timer channels are independent expectations.
                routes = [(12, 1, 2, "B", 14, 15, 9), (8, 1, 2, "C", 6, 7, 3),
                          (8, 3, 4, "C", 8, 9, 3), (1, 1, 3 if nano else 2, "A", 8, 10 if nano else 9, 1)]
                for i, (timer, ch1, ch2, bank, bit1, bit2, af) in enumerate(routes):
                    p.issue("INITIALIZE_MOTOR", motor_index=i)
                    p.issue("SET_MOTOR_SPEED", motor_index=i, pwm=-25)
                    self.assertEqual((b.timer_ticks(timer, ch1), b.timer_ticks(timer, ch2)), (840, 630))
                    self.assertEqual((b.pad_alternate_function(bank, bit1),
                                      b.pad_alternate_function(bank, bit2)), (af, af))
                    self.assertEqual(self.pairs(b)[i], (HIGH, ("timer", 630)))
                    p.issue("STOP_MOTOR", motor_index=i)
                    self.assertEqual(self.pairs(b)[i], (COAST, COAST))
                self.assertEqual(b.timer_ticks(1, 2 if nano else 3), -1)

    def test_unconfirmed_low_hold_requires_retry(self):
        for revision in REVISIONS:
            with self.subTest(revision=revision):
                b = self.board(revision)
                p = Peer(b, "usb")
                p.initialize()
                b.set_sleep_readback_failure(2)
                p.issue("SET_DRIVER_SLEEP", error="INTERNAL_ERROR", sleeping=1)
                self.assertEqual(b.low_dwell_ms, 0)
                self.assertEqual(self.pairs(b)[2], (("gpio", 840), GPIO_LOW))
                self.assertEqual(p.issue("GET_DRIVER_SLEEP"), bytes([1]))
                p.issue("SET_DRIVER_SLEEP", error="INTERNAL_ERROR", sleeping=0)
                b.set_sleep_readback_failure(2, False)
                p.issue("SET_DRIVER_SLEEP", sleeping=0)
                self.assertGreaterEqual(b.low_dwell_ms, 2)
                self.held(b)

    def test_motion_coast_sleep_reconnect_and_release(self):
        for revision in REVISIONS:
            for bus in BUSES:
                with self.subTest(revision=revision, bus=bus):
                    b = self.board(revision)
                    peer = Peer(b, bus)
                    peer.initialize()
                    self.held(b)
                    if b.schema.version < (2, 18, 0):
                        peer.issue("GET_BOARD_IDLE", error="SENSOR_NOT_AVAILABLE")
                    else:
                        self.assertEqual(peer.issue("GET_BOARD_IDLE"), bytes([0]))
                    # Leave M1/M2 uninitialized; exercise M3's revision-specific second pin.
                    for i in (0, 3):
                        peer.issue("INITIALIZE_MOTOR", motor_index=i)
                        peer.issue("SET_MOTOR_SPEED", motor_index=i, pwm=50)
                    self.assertEqual(self.pairs(b)[3], (("timer", 420), HIGH))
                    unused = 10 if revision == "0.3.0" else 9
                    self.assertEqual(b.input_pad("A", unused), (None, None))
                    peer.issue("STOP_MOTOR", motor_index=3)
                    self.assertEqual(self.pairs(b)[3], (COAST, COAST))
                    self.assertEqual(peer.issue("GET_DRIVER_SLEEP"), bytes([0]))
                    peer.issue("SET_MOTOR_SPEED", motor_index=3, pwm=-25)
                    self.assertEqual(self.pairs(b)[3], (HIGH, ("timer", 630)))
                    b.advance(.02)
                    self.assertLess(b.motor_velocity[3], 0)
                    before = b.low_dwell_ms
                    for _ in range(2):
                        peer.issue("SET_DRIVER_SLEEP", sleeping=1)
                        self.assertGreaterEqual(b.low_dwell_ms - before, 2)
                        self.assertEqual(peer.issue("GET_DRIVER_SLEEP"), bytes([1]))
                        self.held(b)
                    b.disconnect()  # physical USB loss invalidates both sessions
                    peer.initialize()
                    self.assertEqual(peer.issue("GET_DRIVER_SLEEP"), bytes([1]))
                    for command in ("SET_MOTOR_SPEED", "BRAKE_MOTOR"):
                        peer.issue(command, error="DRIVERS_ASLEEP", motor_index=3,
                                   **({"pwm": 50} if command == "SET_MOTOR_SPEED" else {}))
                    for _ in range(2):
                        peer.issue("SET_DRIVER_SLEEP", sleeping=0)
                        self.assertEqual(peer.issue("GET_DRIVER_SLEEP"), bytes([0]))
                        b.advance(.02)
                        self.held(b)
                    peer.issue("BRAKE_MOTOR", motor_index=3)
                    self.assertEqual(self.pairs(b), [(GPIO_LOW, GPIO_LOW)] * 3 + [(HIGH, HIGH)])
                    peer.issue("STOP_MOTOR", motor_index=3)
                    self.assertEqual(self.pairs(b)[3], (COAST, COAST))
                    peer.issue("SET_MOTOR_SPEED", motor_index=3, pwm=25)
                    self.assertEqual(b.motor_pwm, (0., 0., 0., 25.))

    def test_cross_transport_wake_fences_queued_and_partial_requests(self):
        for revision in REVISIONS:
            for bus in BUSES:
                for split in (1, 3):
                    with self.subTest(revision=revision, pending=bus, split=split):
                        b = self.board(revision)
                        pending = Peer(b, bus)
                        other = Peer(b, "i2c" if bus == "usb" else "usb")
                        pending.initialize()
                        other.initialize()
                        pending.issue("INITIALIZE_MOTOR", motor_index=3)
                        other.issue("SET_DRIVER_SLEEP", sleeping=1)
                        if bus == "usb":
                            b.set_usb_busy(True)
                        # The first unread reply blocks dispatch on this session.
                        pending.write(pending.frame("GET_DRIVER_SLEEP"))
                        drive = ("SET_MOTOR_SPEED", {"pwm": 50})
                        brake = ("BRAKE_MOTOR", {})
                        complete, partial = (drive, brake) if split == 1 else (brake, drive)
                        queued = pending.frame(complete[0], motor_index=3, **complete[1])
                        pending.write(queued)
                        fragment = pending.frame(partial[0], motor_index=3, **partial[1])
                        pending.write(fragment[:split])
                        other.issue("SET_DRIVER_SLEEP", sleeping=0)
                        self.held(b)
                        pending.write(fragment[split:], start=False)
                        if bus == "usb":
                            b.set_usb_busy(False)
                        replies = pending.replies()
                        for name, offset in ((complete[0], 1), (partial[0], 0)):
                            self.assertIn((b.schema.code("ERROR"), pending.sequence - offset,
                                           bytes((b.schema.code(name), 19))), replies)
                        self.held(b)
                        pending.issue("BRAKE_MOTOR", motor_index=3)
                        self.assertEqual(self.pairs(b)[3], (HIGH, HIGH))

    def test_platform_controller_and_brake_after_release(self):
        for revision in REVISIONS:
            for bus in BUSES:
                with self.subTest(revision=revision, bus=bus):
                    b = self.board(revision)
                    p = Peer(b, bus)
                    p.initialize()
                    p.issue("INITIALIZE_DIFFERENTIAL_PLATFORM", wheel_diameter=.12,
                            wheel_base=.36, encoder_resolution=2000)
                    p.issue("SET_CONTROLLER_FREQUENCY", frequency=100)
                    p.issue("START_PLATFORM_CONTROLLER", kp=10, ki=20, kd=0, integral_limit=100)
                    p.issue("SET_PLATFORM_TARGET_VELOCITY", x=.1)
                    b.advance(.05)
                    self.assertTrue(any(b.motor_pwm))
                    p.issue("SET_DRIVER_SLEEP", sleeping=1)
                    p.issue("BRAKE_PLATFORM", error="DRIVERS_ASLEEP")
                    # A platform brake begun during sleep must also be rejected across release.
                    other = Peer(b, "i2c" if bus == "usb" else "usb")
                    other.initialize()
                    if bus == "usb":
                        b.set_usb_busy(True)
                    p.write(p.frame("GET_DRIVER_SLEEP"))
                    p.write(p.frame("BRAKE_PLATFORM"))
                    request = p.frame("BRAKE_PLATFORM")
                    p.write(request[:1])
                    other.issue("SET_DRIVER_SLEEP", sleeping=0)
                    p.write(request[1:], start=False)
                    if bus == "usb":
                        b.set_usb_busy(False)
                    replies = p.replies()
                    for seq in (p.sequence - 1, p.sequence):
                        self.assertIn((b.schema.code("ERROR"), seq,
                                       bytes([b.schema.code("BRAKE_PLATFORM"), 19])), replies)
                    b.advance(.15)
                    self.held(b)
                    p.issue("SET_PLATFORM_TARGET_VELOCITY", x=.1, error="CONTROLLER_NOT_INITIALIZED")
                    p.issue("BRAKE_PLATFORM")
                    self.assertEqual(self.pairs(b), [(HIGH, HIGH)]*2 + [(GPIO_LOW, GPIO_LOW)]*2)
                    p.issue("COAST_PLATFORM")
                    self.assertEqual(self.pairs(b), [(COAST, COAST)]*2 + [(GPIO_LOW, GPIO_LOW)]*2)
