"""Wheel telemetry and the boundary between a stopped and a disarmed controller."""
import math


def arm(s):
    s.ask("START_PLATFORM_CONTROLLER", kp=10., ki=20., kd=0., integral_limit=100.)


def observe(s):
    before = s.status()
    keys = {k:before[k] for k in ("origin_id","sequence")}
    pose = s.ask("GET_FEEDBACK_SOURCE_VECTOR", kind=0, **keys)
    twist = s.ask("GET_FEEDBACK_SOURCE_VECTOR", kind=1, **keys)
    wheel = s.ask("GET_PLATFORM_ODOMETRY")
    after = s.status()
    assert before == after and before["requested"] == before["active"] == 0 and before["valid"] == 1
    assert before["sequence"] == pose["sequence"] == twist["sequence"] > 0
    assert before["timestamp_us"] == wheel["timestamp_us"] > 0
    assert before["age_us"] == s.board.now_us - wheel["timestamp_us"]
    return before, tuple(wheel[k] for k in ("x","y","t")), tuple(pose[k] for k in "xyz"), tuple(twist[k] for k in "xyz")


def relative(p, anchor):
    dx, dy = p[0] - anchor[0], p[1] - anchor[1]
    c, sn = math.cos(anchor[2]), math.sin(anchor[2])
    return c*dx + sn*dy, -sn*dx + c*dy, p[2] - anchor[2]


def near(observed, expected):
    assert all(math.isclose(a,b, abs_tol=1e-10) for a,b in zip(observed,expected,strict=True)), (observed,expected)


def encoder(s):
    off = s.status()
    assert (off["requested"],off["active"],off["valid"],off["reason"]) == (0,255,0,1), off
    s.error("GET_FEEDBACK_SOURCE_VECTOR","SAMPLE_NOT_AVAILABLE",kind=0, origin_id=off["origin_id"], sequence=off["sequence"])
    s.ask("SET_ODOMETRY_FREQUENCY", frequency=50)
    s.ask("START_PLATFORM_ODOMETRY")
    s.board.advance(.20)
    initial, _, _, _ = observe(s)
    arm(s)
    s.ask("SET_PLATFORM_TARGET_VELOCITY",x=.10)
    s.board.advance(.32)
    s.ask("STOP_PLATFORM_CONTROLLER")
    s.board.advance(.8)
    assert not any(s.board.motor_pwm)
    s.ask("SET_FEEDBACK_SOURCE",source=1)
    s.ask("SET_FEEDBACK_SOURCE",source=0)
    s.board.advance(.02)
    status, anchor, p, _ = observe(s)
    assert initial["origin_id"] != status["origin_id"] and abs(anchor[2]) > .01, (status,anchor)
    near(p,(0,0,0))
    s.mark("encoder_anchor", underlying=anchor, selector=status)
    arm(s)
    s.ask("SET_PLATFORM_TARGET_VELOCITY",x=.1,y=.04,t=.2)
    previous = anchor
    for _ in range(12):
        s.board.advance(.02)
        current, wheel, p, v = observe(s)
        near(p, relative(wheel,anchor))
        # The release producer integrates body increments at midpoint heading.
        mid = (previous[2]+wheel[2]) / 2
        dx,dy,_ = relative(wheel,(previous[0],previous[1],mid))
        dt = (current["timestamp_us"] - status["timestamp_us"]) * 1e-6
        assert dt > 0 and current["sequence"] == status["sequence"] + 1
        near(v,(dx/dt,dy/dt,(wheel[2]-previous[2])/dt))
        previous, status = wheel,current
    assert math.hypot(*p[:2]) > .001 and any(s.board.motor_pwm)
    s.board.advance(.003)
    assert observe(s)[0]["age_us"] == 3000
    s.mark("moving_encoder", relative_pose=p, body_twist=v)
    keys = {k:status[k] for k in ("origin_id","sequence")}
    s.board.advance(.020)
    fresh = s.status()
    assert fresh["sequence"] != status["sequence"] and fresh["origin_id"] == status["origin_id"]
    for kind, expected in enumerate((p,v)):
        cached = s.ask("GET_FEEDBACK_SOURCE_VECTOR",kind=kind,**keys)
        assert cached["sequence"] == keys["sequence"]
        near(tuple(cached[k] for k in "xyz"), expected)
    s.error("GET_FEEDBACK_SOURCE_VECTOR","SAMPLE_NOT_AVAILABLE",kind=0,
            origin_id=status["origin_id"]+1, sequence=status["sequence"])
    s.board.advance(.051)
    s.error("GET_FEEDBACK_SOURCE_VECTOR","SAMPLE_NOT_AVAILABLE",kind=0,**keys)
    s.board.disconnect()
    assert not any(s.board.motor_pwm)
    s.initialize()
    assert s.status()["requested"] == 0
    s.board.advance(.25)
    assert not any(s.board.motor_pwm)


def switch_safety(s):
    image = s.board.fram_image
    s.ask("SET_ODOMETRY_FREQUENCY",frequency=50)
    s.ask("SET_CONTROLLER_FREQUENCY",frequency=100)
    s.ask("START_PLATFORM_ODOMETRY")
    s.board.advance(.3)
    initial = s.status()
    arm(s)
    assert not any(s.board.motor_pwm)
    s.error("SET_FEEDBACK_SOURCE","SOURCE_SWITCH_UNSAFE",source=2)
    s.error("SET_FEEDBACK_FUSION_MOUNT","SOURCE_SWITCH_UNSAFE",mounting_yaw_rad=0.)
    assert initial == s.status()
    s.ask("SET_PLATFORM_TARGET_VELOCITY",x=.1)
    s.board.advance(.2)
    assert any(s.board.motor_pwm)
    s.error("SET_FEEDBACK_SOURCE","SOURCE_SWITCH_UNSAFE",source=1)
    s.ask("STOP_PLATFORM_CONTROLLER")
    assert not any(s.board.motor_pwm)
    counts = s.board.encoder_counts
    s.board.advance(.02)
    assert counts != s.board.encoder_counts and not any(s.board.motor_pwm)
    s.error("SET_FEEDBACK_SOURCE","SOURCE_SWITCH_UNSAFE",source=1)
    s.mark("coasting_disarmed_rejection", previous_counts=counts)
    s.board.advance(.8)
    s.ask("SET_FEEDBACK_SOURCE",source=1)
    status = s.status()
    assert (status["requested"],status["active"],status["valid"]) == (1,255,0)
    assert status["origin_id"] != initial["origin_id"] and not any(s.board.motor_pwm)
    s.error("GET_FEEDBACK_SOURCE_VECTOR","SAMPLE_NOT_AVAILABLE",kind=0,origin_id=status["origin_id"],sequence=status["sequence"])
    s.board.disconnect()
    s.mark("disconnect")
    s.initialize()
    after = s.status()
    assert (after["requested"],after["active"],after["valid"]) == (1,255,0)
    s.board.advance(.2)
    assert not any(s.board.motor_pwm)
    for frequency in (1, 7):
        s.reboot(image)
        _slow_switch_safety(s, frequency)


def _slow_switch_safety(s, frequency):
    """Real shaft motion must gate selection independently of telemetry cadence."""
    period_us = (1000 // frequency) * 1000
    s.ask("SET_ODOMETRY_FREQUENCY", frequency=frequency)
    s.ask("SET_CONTROLLER_FREQUENCY", frequency=100)
    s.ask("START_PLATFORM_ODOMETRY")
    s.error("SET_FEEDBACK_SOURCE", "SOURCE_SWITCH_UNSAFE", source=1)
    s.board.advance(.16)
    assert not any(s.board.motor_pwm)
    s.mark("low_rate_quiet_selection_attempt", frequency_hz=frequency,
           quiet_us=160000, period_us=period_us)
    s.ask("SET_FEEDBACK_SOURCE", source=1)
    s.ask("SET_FEEDBACK_SOURCE", source=0)
    s.mark("low_rate_quiet_selection_succeeded", frequency_hz=frequency)

    arm(s)
    s.ask("SET_PLATFORM_TARGET_VELOCITY", x=.1)
    s.board.advance(.2)
    assert any(s.board.motor_pwm)
    s.error("SET_FEEDBACK_SOURCE", "SOURCE_SWITCH_UNSAFE", source=1)
    s.ask("STOP_PLATFORM_CONTROLLER")
    counts = s.board.encoder_counts
    s.board.advance(.02)
    assert counts != s.board.encoder_counts and not any(s.board.motor_pwm)
    s.error("SET_FEEDBACK_SOURCE", "SOURCE_SWITCH_UNSAFE", source=1)
    s.mark("low_rate_coasting_rejected", frequency_hz=frequency, previous_counts=counts)
    s.board.advance(.8)
    settled = s.board.encoder_counts
    s.board.advance(.12)
    assert settled == s.board.encoder_counts and not any(s.board.motor_pwm)
    s.ask("SET_FEEDBACK_SOURCE", source=1)
    s.ask("SET_FEEDBACK_SOURCE", source=0)
    s.mark("low_rate_stopped_selection_succeeded", frequency_hz=frequency)

    # Passive scans must not publish extra odometry or encoder-selector samples.
    # Poll every 2 ms: this also observes the 142 ms period at nominal 7 Hz.
    prior = None
    publications = []
    for _ in range((4 * period_us) // 2000 + 1):
        s.board.advance(.002)
        status = s.status()
        odometry = s.ask("GET_PLATFORM_ODOMETRY")
        if prior is not None and odometry["timestamp_us"] != prior:
            assert odometry["timestamp_us"] - prior == period_us, (frequency, prior, odometry)
        prior = odometry["timestamp_us"]
        if status["valid"]:
            assert status["max_age_ms"] == period_us * 3 // 1000
            assert status["timestamp_us"] == odometry["timestamp_us"]
            assert status["age_us"] == s.board.now_us - status["timestamp_us"]
            if not publications or status["sequence"] != publications[-1]["sequence"]:
                if publications:
                    assert status["sequence"] == publications[-1]["sequence"] + 1
                    assert status["timestamp_us"] - publications[-1]["timestamp_us"] == period_us
                publications.append(status)
    assert len(publications) >= 3, (frequency, publications)
    assert not any(s.board.motor_pwm)
    s.mark("low_rate_publication_cadence", frequency_hz=frequency,
           period_us=period_us, publications=publications)

    # A larger age allowance cannot renew an acquisition from the old period.
    captured = s.status()
    keys = {k: captured[k] for k in ("origin_id", "sequence")}
    assert captured["valid"]
    s.ask("SET_ODOMETRY_FREQUENCY", frequency=2 if frequency == 1 else 1)
    invalid = s.status()
    assert not invalid["valid"] and invalid["active"] == 255, invalid
    s.error("GET_FEEDBACK_SOURCE_VECTOR", "SAMPLE_NOT_AVAILABLE", kind=0, **keys)
    s.error("GET_FEEDBACK_SOURCE_VECTOR", "SAMPLE_NOT_AVAILABLE", kind=1, **keys)
    s.mark("rate_change_invalidates_old_capture", frequency_hz=frequency,
           captured=captured, invalid=invalid)
