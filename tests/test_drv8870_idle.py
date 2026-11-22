"""Original v0.3 and Nano shallow-idle release cases on native firmware."""
import json
import struct

from test_configuration import FIRMWARE
from test_drv8870_sleep import BUSES, GPIO_LOW, HIGH, REVISIONS, Drv8870Fixture, Peer


class Drv8870IdleTests(Drv8870Fixture):
    minimum_version = (2, 18, 0)

    def state(self, peer, idle, sleeping):
        self.assertEqual(peer.issue("GET_BOARD_IDLE"), bytes([idle]))
        self.assertEqual(peer.issue("GET_DRIVER_SLEEP"), bytes([sleeping]))

    def test_established_usb_enters_from_motion_and_exits_without_init(self):
        profiles = json.loads((FIRMWARE / "include/board_profiles.json").read_text())["boards"]
        for revision in REVISIONS:
            with self.subTest(revision=revision):
                b = self.board(revision)
                usb = Peer(b, "usb")
                usb.initialize()  # Only INIT on this board.
                profile, = [p for p in profiles if p["board"] == {"model": 1, "revision": revision}]
                self.assertIs(profile["power"]["board_idle"], True)
                self.assertEqual(b.build["build_define"], "MP_V3_1" if revision == "0.3.1" else "MP_V3")
                usb.issue("SET_TIME_SYNC_INTERVAL", interval_ms=3500)

                def check_session():
                    self.assertTrue(b.ready)
                    self.assertEqual(usb.issue("PING"), b"")
                    mode, _, interval, _ = struct.unpack("<BBIQ", usb.issue("GET_TIME_STATUS"))
                    self.assertEqual((mode, interval), (0, 3500))

                usb.issue("INITIALIZE_MOTOR", motor_index=3)
                for pwm in (50, -25):
                    usb.issue("SET_MOTOR_SPEED", motor_index=3, pwm=pwm)
                    b.advance(.1)
                    self.assertGreater(b.motor_velocity[3] * pwm, 0)
                    for _ in range(2):
                        usb.issue("SET_BOARD_IDLE", idle=1)
                        b.advance(.05)
                        check_session()
                        self.state(usb, 1, 1)
                        self.held(b)
                        usb.issue("SET_MOTOR_SPEED", error="DRIVERS_ASLEEP", motor_index=3, pwm=50)
                        usb.issue("BRAKE_MOTOR", error="DRIVERS_ASLEEP", motor_index=3)
                    for _ in range(2):
                        usb.issue("SET_BOARD_IDLE", idle=0)
                        b.advance(.05)
                        check_session()
                        self.state(usb, 0, 0)
                        self.held(b)
                    usb.issue("BRAKE_MOTOR", motor_index=3)
                    self.assertEqual(self.pairs(b), [(GPIO_LOW, GPIO_LOW)] * 3 + [(HIGH, HIGH)])
                    # PA9 is an original-board motor pin, reserved for VBUS on Nano.
                    self.assertEqual(b.motor_pad_touched("A", 9), revision == "0.3.0")
                    self.assertEqual(b.motor_pad_touched("A", 10), revision == "0.3.1")

    def test_idle_stops_platform_controller_and_allows_fresh_platform_brake(self):
        for revision in REVISIONS:
            for bus in BUSES:
                with self.subTest(revision=revision, bus=bus):
                    b = self.board(revision)
                    peer = Peer(b, bus)
                    peer.initialize()
                    peer.issue("INITIALIZE_DIFFERENTIAL_PLATFORM", wheel_diameter=.12,
                               wheel_base=.36, encoder_resolution=2000)
                    peer.issue("SET_CONTROLLER_FREQUENCY", frequency=100)
                    peer.issue("START_PLATFORM_CONTROLLER", kp=10, ki=20, kd=0, integral_limit=100)
                    peer.issue("SET_PLATFORM_TARGET_VELOCITY", x=.1)
                    b.advance(.05)
                    self.assertTrue(any(b.motor_pwm))
                    peer.issue("SET_BOARD_IDLE", idle=1)
                    peer.issue("BRAKE_PLATFORM", error="DRIVERS_ASLEEP")
                    peer.issue("SET_PLATFORM_TARGET_VELOCITY", x=.1, error="DRIVERS_ASLEEP")
                    self.held(b)
                    peer.issue("SET_BOARD_IDLE", idle=0)
                    b.advance(.15)
                    self.held(b)
                    peer.issue("SET_PLATFORM_TARGET_VELOCITY", x=.1, error="CONTROLLER_NOT_INITIALIZED")
                    peer.issue("BRAKE_PLATFORM")
                    self.assertEqual(self.pairs(b), [(HIGH, HIGH)] * 2 + [(GPIO_LOW, GPIO_LOW)] * 2)

    def test_entry_and_wake_readback_failures_restore_prior_policy_on_retry(self):
        for revision in REVISIONS:
            for prior_sleep in (0, 1):
                with self.subTest(revision=revision, prior_sleep=prior_sleep):
                    b = self.board(revision)
                    usb = Peer(b, "usb")
                    usb.initialize()
                    usb.issue("INITIALIZE_MOTOR", motor_index=3)
                    usb.issue("SET_MOTOR_SPEED", motor_index=3, pwm=30)
                    if prior_sleep:
                        usb.issue("SET_DRIVER_SLEEP", sleeping=1)
                    for entering in (True, False):
                        b.set_sleep_readback_failure(1)
                        usb.issue("SET_BOARD_IDLE", idle=int(entering), error="INTERNAL_ERROR")
                        self.state(usb, 1, 1)
                        self.assertEqual(usb.issue("PING"), b"")
                        usb.issue("SET_DRIVER_SLEEP", sleeping=0, error="BOARD_IDLE")
                        usb.issue("BRAKE_MOTOR", motor_index=3, error="DRIVERS_ASLEEP")
                        usb.issue("SET_MOTOR_SPEED", motor_index=3, pwm=30, error="DRIVERS_ASLEEP")
                        # The injected IN1 really reads high. Other pads remain low.
                        self.assertEqual(self.pairs(b), [(GPIO_LOW, GPIO_LOW),
                            (("gpio", 840), GPIO_LOW), (GPIO_LOW, GPIO_LOW), (GPIO_LOW, GPIO_LOW)])
                        b.set_sleep_readback_failure(1, False)
                        usb.issue("SET_BOARD_IDLE", idle=int(entering))
                        self.state(usb, int(entering), 1 if entering else prior_sleep)
                        self.held(b)
                    if prior_sleep:
                        usb.issue("BRAKE_MOTOR", motor_index=3, error="DRIVERS_ASLEEP")
                        usb.issue("SET_DRIVER_SLEEP", sleeping=0)
                        self.held(b)
                    usb.issue("SET_MOTOR_SPEED", motor_index=3, pwm=25)
                    self.assertEqual(b.motor_pwm, (0., 0., 0., 25.))

    def test_cross_transport_wake_rejects_complete_and_fragmented_work(self):
        for revision in REVISIONS:
            for bus in BUSES:
                for split in (1, 3):
                    with self.subTest(revision=revision, pending=bus, split=split):
                        b = self.board(revision)
                        pending = Peer(b, bus)
                        control = Peer(b, "i2c" if bus == "usb" else "usb")
                        pending.initialize()
                        control.initialize()
                        pending.issue("INITIALIZE_MOTOR", motor_index=3)
                        control.issue("SET_BOARD_IDLE", idle=1)
                        for name, fields in (("SET_MOTOR_SPEED", {"pwm": 50}), ("BRAKE_MOTOR", {})):
                            pending.issue(name, error="DRIVERS_ASLEEP", motor_index=3, **fields)
                        if bus == "usb":
                            b.set_usb_busy(True)
                        pending.write(pending.frame("GET_BOARD_IDLE"))
                        # The first query holds the transmit slot. The second must run after wake.
                        pending.write(pending.frame("GET_BOARD_IDLE"))
                        queued_query_id = pending.sequence
                        # This request tests the board-wake fence independently of driver wake.
                        pending.write(pending.frame("SET_DRIVER_SLEEP", sleeping=1))
                        drive = ("SET_MOTOR_SPEED", {"motor_index": 3, "pwm": 50})
                        brake = ("BRAKE_MOTOR", {"motor_index": 3})
                        whole, partial = (drive, brake) if split == 1 else (brake, drive)
                        pending.write(pending.frame(whole[0], **whole[1]))
                        frame = pending.frame(partial[0], **partial[1])
                        pending.write(frame[:split])
                        control.issue("SET_BOARD_IDLE", idle=0)
                        self.held(b)
                        pending.write(frame[split:], start=False)
                        if bus == "usb":
                            b.set_usb_busy(False)
                        replies = pending.replies()
                        query = b.schema.code("GET_BOARD_IDLE")
                        self.assertIn((query, queued_query_id - 1, bytes([1])), replies)
                        self.assertIn((query, queued_query_id, bytes([0])), replies)
                        expected = [("SET_DRIVER_SLEEP", "BOARD_IDLE"), (whole[0], "DRIVERS_ASLEEP"),
                                    (partial[0], "DRIVERS_ASLEEP")]
                        for seq, (command, error) in enumerate(expected, pending.sequence - 2):
                            self.assertIn((b.schema.code("ERROR"), seq,
                                           bytes([b.schema.code(command), b.schema.errors[error]])), replies)
                        self.state(pending, 0, 0)
                        self.held(b)
                        pending.issue("BRAKE_MOTOR", motor_index=3)
                        self.assertEqual(self.pairs(b)[3], (HIGH, HIGH))
                        pending.issue("SET_MOTOR_SPEED", motor_index=3, pwm=25)
                        self.assertEqual(b.motor_pwm, (0., 0., 0., 25.))
