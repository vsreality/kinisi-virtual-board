"""Public v0.4 dual-source tracking checks using native firmware and the BMI088 plant."""

import hashlib
import json
import math

from .inertial import Stimulus, _invalid, _selected, _wait
from ..imu_regression_capture import CommandError
from .runner import HERE, SIM, Session, pin
from .wheel import arm
from .health_wire import (
    available,
    sources as source_health,
    require,
    Flag,
    HEALTHY,
    SUSPECT,
    unattributed_conflict,
    UNAVAILABLE,
)

GEOMETRY = {
    "differential": dict(wheel_diameter=0.1, wheel_base=0.30),
    "omni": dict(wheels_diameter=0.1, robot_radius=0.17),
    "mecanum": dict(wheels_diameter=0.1, width=0.26, length=0.34),
}
STEP = 0.0025
CPR = 4096
CASES = (
    *GEOMETRY,
    "imu_disconnect",
    "imu_stale",
    "imu_invalid",
    "encoder_off",
    "encoder_timer",
    "gyro_stale",
    "frozen_counts",
    "constant_speed",
    "reconnect",
    "live_target_fault",
    "live_encoder_fault",
    "finite_imu",
    "v03",
    "nano",
    "health_protocol",
)


def setup(session, preset):
    session.ask(
        "INITIALIZE_" + preset.upper() + "_PLATFORM",
        encoder_resolution=CPR,
        **GEOMETRY[preset],
    )
    stim = Stimulus(session)
    session.ask("START_PLATFORM_ODOMETRY")
    stim.run(150000)
    session.ask("SET_FEEDBACK_SOURCE", source=2)
    session.ask("SET_FEEDBACK_FUSION_MOUNT", mounting_yaw_rad=0.0)
    require(
        session.ask("GET_FEEDBACK_FUSION_MOUNT")["confirmed"] == 1, "Mount unconfirmed"
    )
    _wait(session, stim, 2)
    return stim


def reading(session, label):
    status, pose, velocity = _selected(session)
    require(
        status["reason"] == status["source_detail"] == 0 and status["active"] == 2,
        f"{label}: unexpected selector {status}",
    )
    require(
        status["sequence"] > 0
        and 0 < status["timestamp_us"] <= session.board.now_us
        and status["age_us"] <= 1000 * status["max_age_ms"],
        f"Expired {label}: {status}",
    )
    for vector in (pose, velocity):
        require(
            all(math.isfinite(vector[k]) for k in "xyz"), f"Nonfinite {label}: {vector}"
        )
    wheel = session.ask("GET_PLATFORM_ODOMETRY")
    row = dict(
        selector=status,
        pose=pose,
        velocity=velocity,
        wheel=wheel,
        sources=source_health(session),
    )
    session.mark(label, **row)
    if row["sources"]:
        available(row["sources"])
    return row


def counts(preset, vx, yaw):
    # Independent rolling-wheel geometry for the imposed body twist (vy=0).
    if preset == "differential":
        linear = (vx - 0.15 * yaw, vx + 0.15 * yaw, 0, 0)
    elif preset == "omni":
        x = math.sqrt(3) * vx / 2
        linear = (x + 0.17 * yaw, -x + 0.17 * yaw, 0.17 * yaw, 0)
    else:
        linear = (vx + 0.30 * yaw, vx + 0.30 * yaw, vx - 0.30 * yaw, vx - 0.30 * yaw)
    return tuple(v * STEP * CPR / (math.pi * 0.1) for v in linear)


def motion(session, stim, preset):
    # Straight-line acceleration followed by constant velocity. Truth is used only
    # to construct independent sensor inputs and compare the returned estimates.
    start = reading(session, "rest")
    origin_us = session.board.now_us
    distance = 0.0
    peak_error = 0.0
    observations = []
    for index in range(240):
        a = 0.24 if index < 100 else 0.0
        end_v = 0.24 * min(index + 1, 100) * STEP
        mid_v = end_v - 0.5 * a * STEP
        distance += mid_v * STEP
        stim.step(accel=(a, 0.0, 9.80665), counts=counts(preset, mid_v, 0.0))
        if (index + 1) % 8 == 0:
            row = reading(session, "straight_motion")
            stamp = row["selector"]["timestamp_us"]
            elapsed = max(0.0, (stamp - origin_us) * 1e-6)
            ramp = min(elapsed, 0.25)
            reference_v = 0.24 * ramp
            reference_x = 0.5 * 0.24 * ramp * ramp + reference_v * (elapsed - ramp)
            error = row["pose"]["x"] - start["pose"]["x"] - reference_x
            peak_error = max(peak_error, abs(error))
            observations.append(
                dict(
                    time_us=session.board.now_us,
                    source_us=stamp,
                    reference_x_m=reference_x,
                    reference_vx_m_s=reference_v,
                    estimate_x_m=row["pose"]["x"],
                    estimate_vx_m_s=row["velocity"]["x"],
                )
            )
    require(
        abs(row["wheel"]["x"] - distance) < 0.003,
        f"Wheel fixture mismatch: {row['wheel']} vs {distance}",
    )
    require(
        row["pose"]["x"] > start["pose"]["x"] and row["velocity"]["x"] > 0,
        "Fused estimate did not observe forward translation",
    )
    return dict(
        positions_and_velocities=observations,
        peak_position_error_m=peak_error,
        endpoint=row,
        case_class=preset,
    )


def inject_registers(session, accel, gyro, accel_us, gyro_us, status=0):
    session.board.set_imu(
        accel, gyro, accel_us=accel_us, gyro_us=gyro_us, status=status
    )
    session.mark(
        "sensor_delivery",
        acceleration=accel,
        angular_velocity=gyro,
        accel_us=accel_us,
        gyro_us=gyro_us,
        status=status,
    )


def outage(session, stim, case):
    cached = reading(session, "before_fault")["selector"]
    injected = session.board.now_us
    reason, detail = {
        "imu_disconnect": (3, 3),
        "imu_stale": (4, 4),
        "gyro_stale": (4, 4),
        "imu_invalid": (6, 6),
        "encoder_off": (1, 2),
        "encoder_timer": (3, 2),
    }[case]
    encoder_fault = case.startswith("encoder_")
    bad, good = ("encoder", "imu") if encoder_fault else ("imu", "encoder")
    session.mark("inject", kind=case)
    held_gyro = stim.held[1]
    held_rate = tuple(stim.values[3:])
    if case == "imu_disconnect":
        inject_registers(
            session, stim.values[:3], held_rate, stim.held[0], held_gyro, status=3
        )
    elif case == "encoder_off":
        session.ask("STOP_PLATFORM_ODOMETRY")
    elif case == "encoder_timer":
        session.board.set_encoder_sampling(0, False)

    def step_fault():
        stim.step(
            deliver=encoder_fault or case == "imu_invalid",
            gyro=(0.0, 0.0, 8.65 if case == "imu_invalid" else 0.0),
        )
        if case == "gyro_stale":
            inject_registers(
                session, stim.values[:3], held_rate, stim.held[0], held_gyro
            )

    first_source = None
    for _ in range(80):
        state = session.status()
        rows = source_health(session)
        if rows and rows[bad]["health"] != HEALTHY and first_source is None:
            first_source = session.board.now_us
            session.mark("first_input_diagnosis", sources=rows, selector=state)
        if not state["valid"]:
            break
        step_fault()
    fault = _invalid(session, 2, reason, cached)
    require(
        fault["source_detail"] == detail, f"Expected selector detail {detail}: {fault}"
    )
    first = session.board.now_us
    sources = source_health(session)
    session.mark("diagnosed", selector=fault, sources=sources)
    if sources:
        available(sources, (good,))
        expected_health = SUSPECT if case == "imu_invalid" else UNAVAILABLE
        expected_detail = 0 if case == "encoder_off" else detail
        require(
            (sources[bad]["health"], sources[bad]["reason"], sources[bad]["detail"])
            == (expected_health, reason, expected_detail),
            f"Wrong {bad} diagnosis: {sources}",
        )
        require(
            sources["fused"]["health"] != HEALTHY,
            f"Fused output called healthy: {sources}",
        )
        if case == "imu_invalid":
            require(
                sources[bad]["flags"] & Flag.SATURATION,
                f"Missing saturation flag: {sources}",
            )
    for _ in range(64):
        step_fault()
    historical = source_health(session)
    session.mark("late_fault_read", sources=historical, selector=session.status())
    if historical:
        for name in (bad, "fused"):
            require(
                not historical[name]["flags"] & Flag.FRESH,
                f"Stale {name} marked fresh: {historical}",
            )
        if case == "gyro_stale":
            require(
                historical["dies"]["gyro_timestamp_us"] == held_gyro
                and historical["dies"]["accel_age_us"] <= 35000,
                f"Lost die provenance: {historical}",
            )
    if not encoder_fault:
        wheel = session.ask("GET_PLATFORM_ODOMETRY")
        require(
            0 <= session.board.now_us - wheel["timestamp_us"] <= 20000,
            f"Wheel observation lost: {wheel}",
        )
    elif case == "encoder_off":
        session.error("GET_PLATFORM_ODOMETRY", "ODOMETRY_NOT_INITIALIZED")
        session.ask("START_PLATFORM_ODOMETRY")
    else:
        session.board.set_encoder_sampling(0, True)
    recovered = _wait(session, stim, 2)
    require(
        recovered["origin_id"] != cached["origin_id"],
        "Fault did not reset relative origin",
    )
    state = reading(session, "recovered")
    return dict(
        injection_us=injected,
        first_observed_us=first,
        first_input_diagnosis_us=first_source,
        historical=historical,
        diagnosis=fault,
        sources=sources,
        recovery_us=session.board.now_us,
        recovered=state,
    )


def turn(session, stim, preset):
    start = reading(session, "turn_start")
    for _ in range(30):
        stim.run(20000, counts=counts(preset, 0.0, 0.5), gyro=(0.0, 0.0, 0.5))
        current = reading(session, "turn")
    actual = current["wheel"]["t"] - start["wheel"]["t"]
    require(
        abs(actual - 0.30) < 0.012,
        f"Turn fixture mismatch: expected .30 rad, measured {actual}",
    )
    return dict(reference_yaw_rad=0.30, reference_wz_rad_s=0.5, final=current)


def conflict(session, stim):
    before = reading(session, "before_conflict")
    # Brief external rotation, then agreement again. All counters remain current.
    stim.run(20000, gyro=(0.0, 0.0, 0.8))
    transient = source_health(session)
    if transient:
        require(
            not transient["fused"]["flags"] & Flag.DISAGREEMENT,
            f"Transient diagnosed: {transient}",
        )
    stim.run(300000)
    started = session.board.now_us
    session.mark(
        "inject", kind="stationary_counters_external_yaw", actual_yaw_rate_rad_s=0.8
    )
    first = None
    for _ in range(60):
        stim.run(20000, gyro=(0.0, 0.0, 0.8))
        state = session.status()
        rows = source_health(session)
        wheel = session.ask("GET_PLATFORM_ODOMETRY")
        session.mark("conflict_poll", selector=state, sources=rows, wheel=wheel)
        if rows and rows["fused"]["flags"] & Flag.DISAGREEMENT:
            if first is None:
                first = session.board.now_us
            unattributed_conflict(rows)
            require(
                state["requested"] == 2
                and state["active"] == (2 if state["valid"] else 255),
                f"Fallback during disagreement: {state}",
            )
        if state["valid"]:
            _selected(session)
    require(
        abs(wheel["t"] - before["wheel"]["t"]) < 0.001,
        f"Expected stationary counters: {wheel}",
    )
    require(
        first is not None and rows["fused"]["flags"] & Flag.DISAGREEMENT,
        f"No persistent disagreement for .96 rad of actual rotation: selector={state}, sources={rows}",
    )
    for _ in range(75):
        stim.run(20000)
        rows = source_health(session)
        if not rows["fused"]["flags"] & Flag.DISAGREEMENT:
            break
    return dict(
        injection_us=started,
        first_observed_us=first,
        reference_yaw_rad=0.96,
        transient=transient,
        recovery_us=session.board.now_us,
        recovered=reading(session, "recovered"),
    )


def active_fault(session, stim, case):
    # This separate experiment is the production correction path. The passive
    # selector alone never arms a velocity controller.
    session.ask("SET_FEEDBACK_SOURCE", source=0)
    arm(session)
    session.ask("SET_FUSION_EXPERIMENT", mode=2, mounting_yaw_rad=0.0)
    stim.run(850000)
    target = dict(x=0.12, y=0.0, t=0.0)
    healthy = session.ask("GET_FUSION_STATUS")
    require(
        healthy["health"] == 2 and healthy["flags"] & 1, f"Controller setup: {healthy}"
    )
    session.ask("SET_PLATFORM_TARGET_VELOCITY", **target)
    for _ in range(64):
        stim.step(require_coast=False)
    require(any(session.board.motor_pwm), "Prior target did not actuate")
    live = session.ask("GET_FUSION_STATUS")
    require(
        live["mode"] == 2 and live["flags"] & 8, f"No live fused correction: {live}"
    )
    started = session.board.now_us
    timer_failure = case == "live_encoder_fault"
    session.mark("live_target_fault", fault=case)
    if timer_failure:
        session.board.set_encoder_sampling(0, False)
    else:
        inject_registers(
            session, stim.values[:3], stim.values[3:], *stim.held, status=3
        )
    for _ in range(48):
        stim.step(deliver=timer_failure, require_coast=False)
        if not any(session.board.motor_pwm):
            break
    require(
        not any(session.board.motor_pwm),
        "Firmware failed to coast without a status read",
    )
    stopped = session.board.now_us
    failed = session.ask("GET_FUSION_STATUS")
    require(
        failed["mode"] == 2
        and failed["health"] == 3
        and failed["fault"] == (2 if timer_failure else 3)
        and failed["flags"] & 13 == 4,
        f"Fault/target fence: {failed}",
    )
    if timer_failure:
        session.error(
            "SET_PLATFORM_TARGET_VELOCITY", "CONTROLLER_NOT_INITIALIZED", **target
        )
        # A partial START received during loss cannot rearm after timer repair.
        packet = session.schema.request(
            "START_PLATFORM_CONTROLLER",
            60000,
            kp=10.0,
            ki=20.0,
            kd=0.0,
            integral_limit=100.0,
        )
        session.board.feed(packet[:3])
        session.mark("partial_start_before_repair", tx=packet[:3].hex())
        session.board.set_encoder_sampling(0, True)
        stim.run(120000)
        session.board.feed(packet[3:])
        reply = session.board.drain()
        session.mark("partial_start_after_repair", tx=packet[3:].hex(), rx=reply.hex())
        expected = session.schema.frame(
            "ERROR",
            60000,
            bytes(
                [
                    session.schema.code("START_PLATFORM_CONTROLLER"),
                    session.schema.errors["SAMPLE_NOT_AVAILABLE"],
                ]
            ),
        )
        require(reply == expected, f"Command spanning encoder repair: {reply.hex()}")
    else:
        session.error("SET_PLATFORM_TARGET_VELOCITY", "SAMPLE_NOT_AVAILABLE", **target)
    stim.run(1800000)
    recovered = session.ask("GET_FUSION_STATUS")
    require(
        recovered["health"] == 2
        and recovered["fault"] == 0
        and recovered["flags"] & 13 == 5,
        f"Target fence during recovery: {recovered}",
    )
    if timer_failure:
        session.error(
            "SET_PLATFORM_TARGET_VELOCITY", "CONTROLLER_NOT_INITIALIZED", **target
        )
        arm(session)
        stim.run(120000)
    session.ask("SET_PLATFORM_TARGET_VELOCITY", **target)
    for _ in range(80):
        stim.step(require_coast=False)
    require(any(session.board.motor_pwm), "New target did not actuate")
    session.board.disconnect()
    require(not any(session.board.motor_pwm), "Disconnect failed to disarm")
    session.initialize()
    stim.run(300000)
    return dict(
        injection_us=started,
        zero_pwm_before_read_us=stopped,
        live=live,
        failed=failed,
        recovered=recovered,
        reconnected=session.status(),
    )


def scenario(session, case):
    if case in ("health_protocol", "v03", "nano"):
        require(
            "GET_FEEDBACK_SOURCE_HEALTH" in session.schema.commands
            and "GET_FEEDBACK_IMU_CHANNELS" in session.schema.commands,
            "Firmware schema predates per-source health (protocol 2.22)",
        )
        if case == "health_protocol":
            session.error("GET_FEEDBACK_SOURCE_HEALTH", "INVALID_ARGUMENT", source=255)
            return dict(stopped_sources=source_health(session))
        for source in range(3):
            session.error(
                "GET_FEEDBACK_SOURCE_HEALTH", "SENSOR_NOT_AVAILABLE", source=source
            )
        session.error("GET_FEEDBACK_IMU_CHANNELS", "SENSOR_NOT_AVAILABLE")
        require(not any(session.board.motor_pwm), "Unsupported diagnostic actuated")
        return dict(unsupported=session.revision)
    preset = case if case in GEOMETRY else "differential"
    stim = setup(session, preset)
    if case == "constant_speed":
        # Compare two external reference histories on the identical observed interval.
        # At rest, or on a carriage already coasting at constant speed with the
        # shafts fixed, both a level accelerometer and gyro see the same inputs.
        start = reading(session, "ambiguous_start")
        for _ in range(30):
            stim.run(20000)
            current = reading(session, "ambiguous_rest_or_translation")
        require(
            current["wheel"]["x"] == start["wheel"]["x"],
            "Stationary shafts changed pose",
        )
        return dict(
            interval_s=0.6,
            rest_reference_travel_m=0.0,
            coasting_reference_travel_m=0.036,
            coasting_reference_speed_m_s=0.06,
            current=current,
        )
    if case in GEOMETRY:
        result = motion(session, stim, preset)

        # Decelerate over the same ramp before imposing pure rotation.
        for i in range(100):
            stim.step(
                accel=(-0.24, 0.0, 9.80665),
                counts=counts(preset, (99.5 - i) * 0.24 * STEP, 0.0),
            )
        stim.run(100000)
        result["turn"] = turn(session, stim, preset)
        return result
    if case == "finite_imu":
        session.ask("SET_FEEDBACK_SOURCE", source=1)
        session.ask(
            "SET_IMU_TRACKING",
            action=0,
            stationary_mount_confirmed=1,
            mounting_yaw_rad=math.pi,
        )
        initial = _wait(session, stim, 1)
        stim.run(1100000)
        expired = _invalid(session, 1, 5, initial)
        physical = source_health(session)
        if physical:
            available(physical, ("encoder", "imu"))
        session.mark(
            "expired_diagnostic_current_physical_imu",
            selector=expired,
            sources=physical,
        )
        session.ask(
            "SET_IMU_TRACKING",
            action=2,
            stationary_mount_confirmed=1,
            mounting_yaw_rad=math.pi,
        )
        restarted = _wait(session, stim, 1)
        require(
            restarted["origin_id"] != initial["origin_id"],
            "Restart reused expired origin",
        )
        return dict(
            initial=initial, expired=expired, physical=physical, restarted=restarted
        )
    if case in ("live_target_fault", "live_encoder_fault"):
        return active_fault(session, stim, case)
    if case == "frozen_counts":
        return conflict(session, stim)
    if case == "reconnect":
        before = reading(session, "before_disconnect")
        session.board.disconnect()
        require(not any(session.board.motor_pwm), "Disconnect actuated")
        session.initialize()
        after = session.status()
        session.mark("reconnected", selector=after, sources=source_health(session))
        require(after["requested"] == 2, f"Reconnect lost preference: {after}")
        stim.run(180000)
        require(not any(session.board.motor_pwm), "Reconnect resumed target")
        current = reading(session, "reconnected_live")
        require(
            current["selector"]["timestamp_us"] > before["selector"]["timestamp_us"],
            "Reconnect returned only a historical snapshot",
        )
        return dict(before=before, after=after, current=current)
    return outage(session, stim, case)


def run(args):
    if not __debug__:
        raise ValueError("Use Python with assertions enabled")
    firmware, out = args.firmware.resolve(), args.out.resolve()
    if out.exists():
        raise FileExistsError(out)
    fw = pin(firmware, output=out)
    sim = pin(SIM, development=True, output=out)
    image = (HERE / "fixtures" / "factory.fram").read_bytes()
    out.mkdir(parents=True, exist_ok=False)
    report = dict(
        firmware=fw,
        simulator=sim,
        seed=131,
        planned=list(CASES),
        cases={},
        outcome="INCOMPLETE",
    )

    def checkpoint():
        temp = out / "summary.tmp"
        temp.write_text(
            json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf8"
        )
        temp.replace(out / "summary.json")

    checkpoint()
    for case in CASES:
        log = out / (case + ".jsonl")
        with log.open("x", encoding="utf8") as stream:
            revision = {"v03": "0.3.0", "nano": "0.3.1"}.get(case, "0.4.0")
            session = Session(firmware, revision, stream, args.compiler)
            try:
                session.reboot(image)
                details = scenario(session, case)
                result = dict(outcome="PASS", evidence=details)
            except (AssertionError, CommandError) as exc:
                result = dict(
                    outcome="FAIL",
                    reason=str(exc) or "Native observation violated case expectation",
                )
            except Exception as exc:
                result = dict(
                    outcome="INCOMPLETE", reason=f"{type(exc).__name__}: {exc}"
                )
            finally:
                session.close()
        result["wire_sha256"] = hashlib.sha256(log.read_bytes()).hexdigest()
        report["cases"][case] = result
        print(case, result["outcome"], result.get("reason", ""), flush=True)
        checkpoint()
    states = [row["outcome"] for row in report["cases"].values()]
    report["counts"] = {
        key: states.count(key) for key in ("PASS", "FAIL", "INCOMPLETE")
    }
    report["outcome"] = (
        "FAIL"
        if "FAIL" in states
        else "INCOMPLETE"
        if "INCOMPLETE" in states
        else "PASS"
    )
    if fw != pin(firmware, output=out) or sim != pin(SIM, development=True, output=out):
        report.update(outcome="INCOMPLETE", reason="Inputs changed during capture")
    checkpoint()
    return {"PASS": 0, "FAIL": 1, "INCOMPLETE": 2}[report["outcome"]]
