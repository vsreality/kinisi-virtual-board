"""Native selector demonstrations; all reference estimates come from wire reads.

BMI088 seed 131 is previously disclosed public development input. Counter changes
represent external wheel rotation on an unpowered test stand: only the simulator
encoder boundary is driven, never firmware pose/estimator state. Passive fusion
has no raw producer getter, so a matched independent native instance runs the
release fused producer for comparison. That shadow is not control qualification.
"""
import json
import math

from kinisi_virtual_board.bmi088 import BMI088, MODEL_ID


class Stimulus:
    def __init__(self, *sessions):
        self.sessions = sessions
        self.model = BMI088(131)
        self.pending = []
        self.values = [0., 0., 9.80665, 0., 0., 0.]
        self.held = [0, 0]
        self.last = (0, 0)
        assert len({s.board.now_us for s in sessions}) == 1
        for s in sessions:
            s.board.set_imu(status=2, accel_us=0, gyro_us=0)
            s.mark("stimulus_setup", model=MODEL_ID, seed=131, delay_us=10000,
                   encoders="external unpowered wheel rotation, counter boundary only")

    def step(self, *, accel=(0., 0., 9.80665), gyro=(0., 0., 0.),
             counts=(0., 0., 0., 0.), deliver=True, require_coast=True):
        at = self.sessions[0].board.now_us + 2500
        self.model.advance(at, accel, gyro)
        stamps = (self.model.accel_us, self.model.gyro_us)
        for die in range(2):
            if stamps[die] != self.last[die]:
                self.pending.append((stamps[die]+10000, die, stamps[die], self.model.values[die*3:die*3+3]))
        self.last = stamps
        for s in self.sessions:
            # Public simulator counter storage is the input to the native timer
            # registers on advance; no production estimator state is modified.
            for i, increment in enumerate(counts):
                s.board._counts[i] += increment
            s.board.advance(.0025)
            assert s.board.now_us == at
            if require_coast:
                assert not any(s.board.motor_pwm), "Telemetry unexpectedly drove a motor"
        due = [row for row in self.pending if row[0] <= at]
        self.pending = [row for row in self.pending if row[0] > at]
        for _, die, stamp, values in due:
            self.values[3*die:3*die+3] = values
            self.held[die] = stamp
        if due and deliver:
            for s in self.sessions:
                s.board.set_imu(self.values[:3], self.values[3:], accel_us=self.held[0], gyro_us=self.held[1])
                s.mark("sensor_delivery", accel_us=self.held[0], gyro_us=self.held[1],
                       acceleration=self.values[:3], angular_velocity=self.values[3:],
                       external_count_increment=counts)

    def run(self, duration_us, **stimulus):
        assert duration_us % 2500 == 0
        for s in self.sessions:
            s.mark("stimulus_interval", duration_us=duration_us, step_us=2500,
                   body_specific_force=stimulus.get("accel", (0., 0., 9.80665)),
                   body_angular_velocity=stimulus.get("gyro", (0., 0., 0.)),
                   external_count_increment_per_step=stimulus.get("counts", (0., 0., 0., 0.)),
                   deliver=stimulus.get("deliver", True))
        for _ in range(duration_us//2500):
            self.step(**stimulus)


def _invalid(s, source, reason=None, cached=None):
    status = s.status()
    assert status["requested"] == source and status["active"] == 255 and not status["valid"], status
    assert status["sequence"] == status["timestamp_us"] == 0 and status["age_us"] == 0xffffffff
    if reason is not None:
        assert status["reason"] == reason, status
    for kind in (0, 1):
        s.error("GET_FEEDBACK_SOURCE_VECTOR", "SAMPLE_NOT_AVAILABLE", kind=kind,
                origin_id=status["origin_id"], sequence=status["sequence"])
        if cached is not None:
            s.error("GET_FEEDBACK_SOURCE_VECTOR", "SAMPLE_NOT_AVAILABLE", kind=kind,
                    origin_id=cached["origin_id"], sequence=cached["sequence"])
    assert not any(s.board.motor_pwm)
    s.mark("invalid_no_fallback", selector=status, rejected_capture=cached)
    return status


def _selected(s):
    before = s.status()
    assert before["valid"] and before["active"] == before["requested"]
    keys = dict(origin_id=before["origin_id"], sequence=before["sequence"])
    pose = s.ask("GET_FEEDBACK_SOURCE_VECTOR", kind=0, **keys)
    twist = s.ask("GET_FEEDBACK_SOURCE_VECTOR", kind=1, **keys)
    after = s.status()
    assert before == after and pose["sequence"] == twist["sequence"] == before["sequence"]
    assert before["age_us"] == s.board.now_us-before["timestamp_us"]
    assert not any(s.board.motor_pwm)
    return before, pose, twist


def _producer(s, fused=False):
    prefix = "FUSION" if fused else "IMU_TRACKING"
    before = s.ask("GET_"+prefix+"_STATUS")
    pose = s.ask("GET_"+prefix+"_VECTOR", kind=2 if fused else 0)
    twist = s.ask("GET_"+prefix+"_VECTOR", kind=3 if fused else 1)
    after = s.ask("GET_"+prefix+"_STATUS")
    assert before == after and before["flags"] & 1
    assert pose["sequence"] == twist["sequence"] == before["sequence"]
    return before, pose, twist


def _compare(s, producer, origin, *, label, same_sequence=True):
    selected = _selected(s)
    status, pose, twist = selected
    raw_status, raw_pose, raw_twist = producer
    assert status["timestamp_us"] == raw_status["timestamp_us"], (status, raw_status)
    if same_sequence:
        assert status["sequence"] == raw_status["sequence"]
    dx, dy = raw_pose["x"]-origin["x"], raw_pose["y"]-origin["y"]
    c, sn = math.cos(origin["z"]), math.sin(origin["z"])
    expected = (c*dx+sn*dy, -sn*dx+c*dy, raw_pose["z"]-origin["z"])
    for key, value in zip(("x", "y", "z"), expected):
        assert math.isclose(pose[key], value, rel_tol=1e-8, abs_tol=1e-8), (key, pose, expected)
        assert math.isclose(twist[key], raw_twist[key], rel_tol=1e-8, abs_tol=1e-8), (key, twist, raw_twist)
    s.mark(label, selector=status, pose=pose, twist=twist, producer_status=raw_status,
           producer_pose=raw_pose, producer_twist=raw_twist, origin=origin, expected_pose=expected)
    return selected


def _wait(s, stimulus, source, timeout_us=1000000):
    start = s.board.now_us
    while s.board.now_us-start < timeout_us:
        stimulus.step()
        status = s.status()
        if status["valid"]:
            assert status["active"] == source
            return status
    raise AssertionError(f"Source {source} never became valid: {status}")


def imu(s):
    stimulus = Stimulus(s)
    # Restored encoders have not yet been observed. The first attempt starts
    # the passive safety sampler; it does not silently start wheel odometry.
    s.error("SET_FEEDBACK_SOURCE", "SOURCE_SWITCH_UNSAFE", source=1)
    stimulus.run(150000)
    s.ask("SET_FEEDBACK_SOURCE", source=1)
    s.ask("START_PLATFORM_ODOMETRY")
    _invalid(s, 1, 1)
    s.error("SET_IMU_TRACKING", "INVALID_ARGUMENT", action=0, stationary_mount_confirmed=0,
            mounting_yaw_rad=math.pi)
    _invalid(s, 1, 1)
    s.ask("SET_IMU_TRACKING", action=0, stationary_mount_confirmed=1, mounting_yaw_rad=math.pi)
    _invalid(s, 1, 2)
    first = _wait(s, stimulus, 1)
    origin = _producer(s)[1]
    _compare(s, _producer(s), origin, label="imu_first_origin")
    stimulus.run(100000, gyro=(0., 0., .6))
    _compare(s, _producer(s), origin, label="imu_rotation")
    # Wheel counters remain stationary: changing IMU yaw must not be wheel fallback.
    assert abs(_producer(s)[1]["z"]-origin["z"]) > .02
    stimulus.run(200000)
    s.ask("SET_FEEDBACK_SOURCE", source=0)
    s.ask("SET_FEEDBACK_SOURCE", source=1)
    switched_at = s.board.now_us
    _invalid(s, 1, 8)
    anchored = _wait(s, stimulus, 1, 30000)
    s.mark("source_time_fence", switched_at=switched_at, selector=anchored)
    assert anchored["timestamp_us"] > switched_at, (
        f"IMU origin acquisition {anchored['timestamp_us']} must follow source switch {switched_at}")
    origin = _producer(s)[1]
    assert anchored["origin_id"] != first["origin_id"] and abs(origin["z"]) > .02
    _compare(s, _producer(s), origin, label="imu_nonzero_heading_origin")
    stimulus.run(120000, accel=(.4, .2, 9.80665), gyro=(0., 0., -.3))
    moving = _compare(s, _producer(s), origin, label="imu_rotated_translation_body_twist")
    assert moving[1]["x"] > .001 and moving[1]["y"] > .0001
    assert moving[2]["x"] > .02 and moving[2]["y"] > .005 and moving[2]["z"] < -.1
    # Equal opposite body-frame acceleration after a turning phase is not exact
    # world-frame cancellation. Use the same opposite yaw-rate interval so the
    # acceleration history retraces its world directions and returns to rest.
    stimulus.run(120000, accel=(-.4, -.2, 9.80665), gyro=(0., 0., .3))
    # Retain the latest valid capture up to the actual expiry boundary, so the
    # rejection cannot be mistaken for an ancient host-side capture alone.
    latest = s.status()
    deadline = s.board.now_us+1000000
    while s.board.now_us < deadline:
        stimulus.step()
        observed = s.status()
        if not observed["valid"]:
            break
        latest = observed
    _invalid(s, 1, 5, cached=latest)
    stimulus.run(100000, counts=(1., 2., 3., 4.))
    wheel = s.ask("GET_PLATFORM_ODOMETRY")
    assert wheel["clock_quality"] == 1 and wheel["clock_mode"] == 0
    assert s.board.now_us-wheel["timestamp_us"] <= 20000
    assert any(abs(wheel[k]) > .0001 for k in ("x", "y", "t"))
    _invalid(s, 1, 5)
    s.mark("imu_expired_healthy_encoder_no_fallback", wheel_producer=wheel)
    stimulus.run(150000)
    s.ask("SET_IMU_TRACKING", action=2, stationary_mount_confirmed=1, mounting_yaw_rad=math.pi)
    _invalid(s, 1, 2)
    recovered = _wait(s, stimulus, 1)
    assert recovered["origin_id"] != anchored["origin_id"]
    _compare(s, _producer(s), _producer(s)[1], label="imu_expiry_deliberate_recovery")
    cached = s.status()
    stimulus.run(120000, deliver=False)
    fault = _invalid(s, 1, cached=cached)
    assert fault["reason"] in (4, 6), fault
    stimulus.run(100000)
    _invalid(s, 1)  # Healthy input alone cannot clear a latched IMU fault.
    s.ask("SET_IMU_TRACKING", action=3, stationary_mount_confirmed=1, mounting_yaw_rad=math.pi)
    final = _wait(s, stimulus, 1)
    assert final["origin_id"] != recovered["origin_id"]
    _compare(s, _producer(s), _producer(s)[1], label="imu_fault_deliberate_recovery")


class _ShadowLog:
    def __init__(self, stream):
        self.stream = stream
    def write(self, value):
        row = json.loads(value)
        row["role"] = "independent_release_fused_shadow"
        self.stream.write(json.dumps(row, allow_nan=False)+"\n")
    def flush(self):
        self.stream.flush()


def fused(s):
    shadow = type(s)(s.firmware, s.revision, _ShadowLog(s.stream), compiler=s.compiler)
    try:
        shadow.reboot(s.board.fram_image)
        assert shadow.board.now_us == s.board.now_us
        stimulus = Stimulus(s, shadow)
        for target in (s, shadow):
            target.ask("START_PLATFORM_ODOMETRY")
        stimulus.run(150000)
        s.ask("SET_FEEDBACK_SOURCE", source=2)
        _invalid(s, 2, 9)
        assert s.ask("GET_FEEDBACK_FUSION_MOUNT")["confirmed"] == 0
        s.ask("SET_FEEDBACK_FUSION_MOUNT", mounting_yaw_rad=0.)
        mount = s.ask("GET_FEEDBACK_FUSION_MOUNT")
        assert mount["confirmed"] == 1 and mount["mounting_yaw_rad"] == 0.
        shadow.ask("SET_FUSION_EXPERIMENT", mode=2, mounting_yaw_rad=0.)
        assert s.ask("GET_FUSION_CONFIGURATION")["mode"] == 0
        _invalid(s, 2, 2)
        first = _wait(s, stimulus, 2)
        origin = _producer(shadow, True)[1]
        _compare(s, _producer(shadow, True), origin, label="passive_first_origin", same_sequence=False)
        # An externally rotated mecanum wheel set is deliberately distinguishable
        # from IMU-only motion. No controller or motor target is ever started.
        stimulus.run(200000, counts=(1., 2., 3., 4.), gyro=(0., 0., .1))
        moved = _compare(s, _producer(shadow, True), origin, label="passive_external_wheel_motion", same_sequence=False)
        assert math.hypot(moved[1]["x"], moved[1]["y"]) > .001
        stimulus.run(150000)
        cached = s.status()
        stimulus.run(100000, deliver=False)
        failed = _invalid(s, 2, cached=cached)
        assert failed["reason"] in (3, 4, 6), failed
        recovered = _wait(s, stimulus, 2)
        assert recovered["origin_id"] != first["origin_id"]
        # Shadow release recovery preserves its world transform; subtract its
        # same-time pose to test the passive selector's fresh relative origin.
        origin = _producer(shadow, True)[1]
        assert abs(origin["z"]) > .01, "Recovery must exercise a nonzero heading transform"
        _compare(s, _producer(shadow, True), origin, label="passive_fault_recovery_origin", same_sequence=False)
        stimulus.run(100000, counts=(2., 1., 4., 3.), gyro=(0., 0., -.1))
        _compare(s, _producer(shadow, True), origin, label="passive_recovery_motion", same_sequence=False)
        assert s.ask("GET_FUSION_CONFIGURATION")["mode"] == 0
        s.mark("passive_control_unchanged", mode=0, shadow_build=shadow.board.build,
               limitation="Matched release producer comparison, not independent fusion accuracy qualification")
    finally:
        shadow.close()
