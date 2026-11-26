"""Release cases for board idle on production firmware and virtual peripherals."""
import json
import struct
import sys
import unittest
from collections import Counter

from test_configuration import FIRMWARE, FirmwareBoard, Session, frames


class IdleSession(Session):
    """Keep unsolicited frames even when an event and a command ACK arrive together."""
    def __init__(self, board):
        super().__init__(board)
        self.events = []

    def capture(self):
        replies = frames(self.board.drain())
        self.events.extend(body for code, _, body in replies
                           if code == self.board.schema.code("TELEMETRY_EVENT"))
        return replies

    def send(self, name, **values):
        self.id += 1
        self.board.feed(self.board.schema.request(name, self.id, **values))
        return next((reply for reply in self.capture() if reply[1] == self.id), None)

    def idle(self, enabled):
        self.ok("SET_BOARD_IDLE", idle=int(enabled))

    def require_state(self, idle, sleeping):
        assert self.ok("GET_BOARD_IDLE") == bytes([idle])
        assert self.ok("GET_DRIVER_SLEEP") == bytes([sleeping])

    def subscribe_sensor(self, name):
        topic = self.board.schema.topic_names[name]["id"]
        self.ok("SUBSCRIBE", topic_id=topic, interval_ms=250)
        return topic


class BoardIdleTests(unittest.TestCase):
    def board(self, revision="0.4.0", initialize=True):
        board = FirmwareBoard(firmware=FIRMWARE, revision=revision)
        self.addCleanup(board.close)
        if "SET_BOARD_IDLE" not in board.schema.commands:
            self.skipTest("Selected firmware predates board idle")
        session = IdleSession(board)
        if initialize:
            session.initialize()
        return board, session

    def workload(self, idle):
        board, session = self.board()
        for motor in range(4):
            session.ok("INITIALIZE_MOTOR", motor_index=motor)
        session.ok("SET_DRIVER_SLEEP", sleeping=1)
        board.advance(.4)
        session.ok("SET_HEARTBEAT_CONFIG", enabled=1, timeout_ms=200)
        topics = [session.subscribe_sensor(name) for name in ("TEMPERATURE", "IMU")]
        session.idle(idle)
        session.require_state(idle, 1)
        session.events.clear()
        counts = board.sensor_read_counts
        start = board.now_us
        commands = session.id
        for _ in range(20):
            board.advance(.05)
            session.ok("PING")
            session.ok("POLL_TELEMETRY")
        totals = board.sensor_read_counts
        event_counts = Counter(body[0] for body in session.events)
        result = dict(duration_us=board.now_us - start, command_acks=session.id - commands,
                      temperature_reads=totals["temperature"] - counts["temperature"],
                      imu_reads=totals["imu"] - counts["imu"],
                      temperature_events=event_counts[topics[0]], imu_events=event_counts[topics[1]])
        session.require_state(idle, 1)
        self.assertEqual(board.motor_pwm, (0.,) * 4)
        return result

    def test_matched_workload(self):
        self.measurements = {name: self.workload(value) for name, value in (("baseline", 0), ("idle", 1))}
        for result in self.measurements.values():
            self.assertEqual((result["duration_us"], result["command_acks"]), (1000000, 40))
        for metric in ("temperature_reads", "imu_reads", "temperature_events", "imu_events"):
            self.assertGreater(self.measurements["baseline"][metric], 0, metric)
            self.assertEqual(self.measurements["idle"][metric], 0, metric)

    def test_subscriptions_resume_with_fresh_samples_and_unsubscribe(self):
        board, session = self.board()
        board.advance(.4)
        session.ok("SET_HEARTBEAT_CONFIG", enabled=1, timeout_ms=2000)
        temp = session.subscribe_sensor("TEMPERATURE")
        imu = session.subscribe_sensor("IMU")
        board.advance(.25)
        session.capture()
        self.assertEqual({body[0] for body in session.events}, {temp, imu})
        session.idle(1)
        session.events.clear()
        session.ok("UNSUBSCRIBE", topic_id=imu)
        for command in ("SUBSCRIBE", "SUBSCRIBE_WITH_MODE", "SUBSCRIBE_CONFIGURED", "SUBSCRIBE_ODOMETRY"):
            session.error(command, "BOARD_IDLE", topic_id=imu, interval_ms=250, poll_interval_ms=250)
        board.set_temperature(41.25)
        board.advance(.5)
        self.assertEqual(struct.unpack("<d", session.ok("GET_TEMPERATURE"))[0], 41.25)
        self.assertEqual(len(session.ok("GET_IMU")), 48)
        session.ok("POLL_TELEMETRY")
        self.assertFalse(session.events)
        wake_us = board.now_us
        session.idle(0)
        board.advance(.25)
        session.capture()
        self.assertTrue(session.events)
        for body in session.events:
            self.assertEqual(body[0], temp)
            self.assertGreaterEqual(struct.unpack_from("<Q", body, 2)[0], wake_us)
            self.assertEqual(struct.unpack_from("<d", body, 12)[0], 41.25)
        session.subscribe_sensor("IMU")
        board.advance(.25)
        session.capture()
        self.assertIn(imu, [body[0] for body in session.events])

    def test_moving_entry_coasts_with_continuous_encoder_odometry(self):
        board, session = self.board()
        session.ok("INITIALIZE_ENCODER", encoder_resolution=4096)
        session.ok("SET_ODOMETRY_FREQUENCY", frequency=100)
        session.ok("START_ENCODER_ODOMETRY")
        session.ok("INITIALIZE_MOTOR")
        session.ok("SET_MOTOR_SPEED", pwm=50)
        board.advance(.1)
        def odometry():
            return struct.unpack("<QBBd", session.ok("GET_ENCODER_ODOMETRY"))
        previous = odometry()
        self.assertGreater(previous[3], 0)
        session.idle(1)
        for _ in range(2):
            session.idle(1)
            session.require_state(1, 1)
            for sleeping in (0, 1):
                session.error("SET_DRIVER_SLEEP", "BOARD_IDLE", sleeping=sleeping)
            session.error("SET_MOTOR_SPEED", "DRIVERS_ASLEEP", pwm=50)
        self.assertEqual(board.motor_pwm, (0.,) * 4)
        board.advance(.1)
        coast = odometry()
        self.assertGreater(coast[0], previous[0])
        self.assertGreater(coast[3], previous[3])
        session.idle(0)
        session.require_state(0, 0)
        board.advance(.1)
        self.assertGreaterEqual(odometry()[3], coast[3])
        self.assertEqual(board.motor_pwm, (0.,) * 4)
        session.ok("SET_MOTOR_SPEED", pwm=25)
        session.idle(0)
        self.assertEqual(board.motor_pwm[0], 25.)

    def test_resume_drops_odometry_from_before_the_wake(self):
        board, session = self.board()
        session.ok("INITIALIZE_ENCODER", encoder_resolution=4096)
        session.ok("SET_ODOMETRY_FREQUENCY", frequency=100)
        session.ok("START_ENCODER_ODOMETRY")
        session.ok("SET_HEARTBEAT_CONFIG", enabled=1, timeout_ms=1000)
        board.advance(.02)
        topic = board.schema.topic_names["ENCODER_ODOMETRY"]["id"]
        session.ok("SUBSCRIBE", topic_id=topic, interval_ms=20)
        board.advance(.02)
        session.capture()
        self.assertTrue(session.events)
        session.idle(1)
        session.events.clear()
        board.advance(.005)  # Between two 10 ms odometry updates.
        wake_us = board.now_us
        session.idle(0)
        session.ok("POLL_TELEMETRY")
        self.assertFalse(session.events)
        board.advance(.03)
        session.capture()
        self.assertTrue(session.events)
        for body in session.events:
            self.assertEqual(body[0], topic)
            self.assertGreater(struct.unpack_from("<Q", body, 2)[0], wake_us)

    def test_failed_wake_reconnect_timeout_and_retry(self):
        board, session = self.board()
        for motor in (0, 2):
            session.ok("INITIALIZE_MOTOR", motor_index=motor)
        session.idle(1)
        board.set_wake_readback_failure(2)
        session.error("SET_BOARD_IDLE", "INTERNAL_ERROR", idle=0)
        session.require_state(1, 1)
        for motor in range(4):
            self.assertEqual(session.ok("GET_MOTOR_FAULTS", motor_index=motor)[:2], bytes([1, 0]))
        board.disconnect()
        session.initialize()
        session.require_state(1, 1)
        session.ok("SET_HEARTBEAT_CONFIG", enabled=1, timeout_ms=200)
        board.advance(.201)
        self.assertFalse(board.ready)
        session.initialize()
        session.require_state(1, 1)
        board.set_wake_readback_failure(2, False)
        session.idle(0)
        session.require_state(0, 0)
        self.assertEqual(board.motor_pwm, (0.,) * 4)
        session.ok("SET_MOTOR_SPEED", motor_index=2, pwm=25)

    def test_prior_sleep_is_retained_on_exit(self):
        board, session = self.board()
        session.ok("INITIALIZE_MOTOR")
        session.ok("SET_DRIVER_SLEEP", sleeping=1)
        session.idle(1)
        session.idle(1)
        session.idle(0)
        session.idle(0)
        session.require_state(0, 1)
        session.error("SET_MOTOR_SPEED", "DRIVERS_ASLEEP", pwm=25)
        session.ok("SET_DRIVER_SLEEP", sleeping=0)
        self.assertEqual(board.motor_pwm, (0.,) * 4)
        session.ok("SET_MOTOR_SPEED", pwm=25)
        self.assertEqual(board.motor_pwm[0], 25.)

    def test_unverified_entry_keeps_inhibition_and_prior_policy(self):
        board, session = self.board()
        session.ok("INITIALIZE_MOTOR")
        session.ok("SET_MOTOR_SPEED", pwm=25)
        board.set_sleep_readback_failure(2)
        session.error("SET_BOARD_IDLE", "INTERNAL_ERROR", idle=1)
        session.require_state(1, 1)
        session.error("SET_MOTOR_SPEED", "DRIVERS_ASLEEP", pwm=25)
        self.assertEqual(board.motor_pwm, (0.,) * 4)
        board.set_sleep_readback_failure(2, False)
        session.idle(1)
        session.idle(0)
        session.require_state(0, 0)
        self.assertEqual(board.motor_pwm, (0.,) * 4)

    def test_queued_frames_require_fresh_requests_after_wake(self):
        for sleeping in (False, True):
            for split in (1, None):
                with self.subTest(sleeping=sleeping, split=split):
                    board, session = self.board()
                    session.ok("INITIALIZE_MOTOR")
                    if sleeping:
                        session.ok("SET_DRIVER_SLEEP", sleeping=1)
                    session.idle(1)
                    wake = board.schema.request("SET_BOARD_IDLE", 80, idle=0)
                    cmd = "SET_DRIVER_SLEEP" if sleeping else "SET_MOTOR_SPEED"
                    pending = board.schema.request(cmd, 81, **({"sleeping": 0} if sleeping else {"pwm": 25}))
                    board.feed(wake + pending[:split])
                    if split is not None:
                        board.feed(pending[split:])
                    error = "BOARD_IDLE" if sleeping else "DRIVERS_ASLEEP"
                    self.assertEqual(frames(board.drain()), [
                        (board.schema.code("SET_BOARD_IDLE"), 80, b""),
                        (board.schema.code("ERROR"), 81, bytes([board.schema.code(cmd), board.schema.errors[error]]))])
                    session.require_state(0, sleeping)
                    if sleeping:
                        session.ok("SET_DRIVER_SLEEP", sleeping=0)
                    self.assertEqual(board.motor_pwm, (0.,) * 4)
                    session.ok("SET_MOTOR_SPEED", pwm=25)
                    self.assertEqual(board.motor_pwm[0], 25.)

    def test_wall_clock_resynchronizes_while_idle(self):
        board, session = self.board(initialize=False)
        board.feed(board.schema.request("INIT", 90, protocol_major=2, client_capabilities=3))
        def answer_burst():
            requests = 0
            replies = frames(board.drain())
            while True:
                sync = [i for cmd, i, _ in replies if cmd == board.schema.code("TIME_SYNC_REQUEST")]
                if not sync:
                    return requests
                self.assertEqual(len(sync), 1)
                requests += 1
                self.assertLessEqual(requests, 3)
                host_time = 10000000 + board.now_us
                board.feed(board.schema.request("TIME_SYNC_RESPONSE", sync[0],
                                                host_receive_us=host_time, host_send_us=host_time))
                replies = frames(board.drain())
                self.assertFalse(any(cmd == board.schema.code("ERROR") for cmd, _, _ in replies))
        self.assertEqual(answer_burst(), 3)
        self.assertTrue(board.ready)
        session.ok("SET_TIME_SYNC_INTERVAL", interval_ms=1000)
        session.idle(1)
        board.advance(1.)
        self.assertEqual(answer_burst(), 3)
        session.require_state(1, 1)
        self.assertTrue(board.ready)

    def test_v03_board_idle_contract_by_protocol(self):
        board, session = self.board("0.3.0")
        session.ok("INITIALIZE_MOTOR")
        session.ok("SET_MOTOR_SPEED", pwm=25)
        if board.schema.version >= (2, 18, 0):
            session.idle(1)
            session.require_state(1, 1)
            session.idle(0)
            session.require_state(0, 0)
            self.assertEqual(board.motor_pwm[0], 0.)
            return
        for idle in (0, 1):
            session.error("SET_BOARD_IDLE", "SENSOR_NOT_AVAILABLE", idle=idle)
        session.error("GET_BOARD_IDLE", "SENSOR_NOT_AVAILABLE")
        self.assertEqual(board.motor_pwm[0], 25.)


if __name__ == "__main__":
    if sys.argv[1:] == ["--measure"]:
        case = BoardIdleTests("test_matched_workload")
        result = case.run()
        if not result.wasSuccessful() or result.skipped:
            raise SystemExit(str(result.errors + result.failures + result.skipped))
        print(json.dumps(case.measurements, indent=2))
    else:
        unittest.main()
