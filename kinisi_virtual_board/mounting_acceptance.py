"""Public native mounting acceptance: physical inputs in, wire observations out.

Fixed public seed and maneuver matrix, never a private qualification draw.
Every output directory is new, and exceptions retain partial wire evidence.
"""

import hashlib
import json
import math
import os
from pathlib import Path
import time
import traceback

from .feedback_selector.runner import HERE, SIM, Session, pin
from .feedback_selector.health_wire import sources, available, unattributed_conflict
from .imu_regression_capture import CommandError
from .mounting_acceptance_score import LIMITS, score

STEP_US = 2500
DELAY_US = 10000
SEED = 131
MOUNTS = {
    "centered": ((0, 0, 0), (0, 0, 0)),
    "left": ((0, 0.1, 0), (0, 0, 0)),
    "right": ((0, -0.1, 0), (0, 0, 0)),
    "roll_positive": ((0, 0, 0), (90, 0, 0)),
    "roll_negative": ((0, 0, 0), (-90, 0, 0)),
    "pitch_positive": ((0, 0, 0), (0, 90, 0)),
    "pitch_negative": ((0, 0, 0), (0, -90, 0)),
    "inverted": ((0, 0, 0), (180, 0, 0)),
    "combined": ((0.08, -0.1, 0.04), (30, -20, 35)),
}
PROFILES = ("rest", "straight", "steady_yaw", "changing_yaw")


def rotation(angles):
    """Active Rz(yaw) Ry(pitch) Rx(roll), board columns in chassis axes."""
    roll, pitch, yaw = map(math.radians, angles)
    cr, sr, cp, sp, cy, sy = (
        math.cos(roll),
        math.sin(roll),
        math.cos(pitch),
        math.sin(pitch),
        math.cos(yaw),
        math.sin(yaw),
    )
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _installations(reference):
    if reference == "chip":
        return MOUNTS
    if reference != "board-center":
        raise ValueError("Reference must be chip or board-center")
    return dict(
        MOUNTS,
        right=((0, -0.1, 0), (90, 0, 0)),
        bottom_reorientation=((0, 0, 0), (90, 0, 0)),
    )


def _cad_vector_um():
    # Independent mechanical oracle: outline center, U601 placement, PCB and
    # package height. Do not substitute firmware constants into this check.
    return (
        round((148.14 - (123 + 204.025126) / 2) * 1000),
        round(((55.5 + 121.525126) / 2 - 61.56) * 1000),
        round((1.6 + 0.95 / 2) * 1000),
    )


def _profile_vector(firmware):
    from .imu_mount import board_center_to_imu_m

    profile = json.loads((firmware / "boards" / "v0.4.json").read_bytes())
    vector = board_center_to_imu_m(profile)
    if tuple(round(v * 1e6) for v in vector) != _cad_vector_um():
        raise AssertionError("Board IMU geometry differs from the independent Mini CAD datum")
    return vector


def plan(reference="chip"):
    installations = _installations(reference)
    cases = [
        dict(case=f"{mount}-{profile}-{mode}", mount=mount, profile=profile, mode=mode)
        for mount in MOUNTS
        for profile in PROFILES
        for mode in ("imu", "fused")
    ]
    cases += [
        dict(
            case="right-fast-steady-fused",
            mount="right",
            profile="fast_steady_yaw",
            mode="fused",
        ),
        dict(
            case="combined-disagreement-fused",
            mount="combined",
            profile="fault",
            mode="fused",
        ),
    ]
    if reference == "board-center":
        cases += [
            dict(
                case=f"bottom-reorientation-changing_yaw-{mode}",
                mount="bottom_reorientation",
                profile="changing_yaw",
                mode=mode,
            )
            for mode in ("imu", "fused")
        ]
    for case in cases:
        case["reference"] = reference
    return dict(
        design="public-mounting-v1" if reference == "chip" else "public-board-center-mounting-v1",
        reference=reference,
        board_center_oracle_um=_cad_vector_um() if reference == "board-center" else None,
        seed=SEED,
        delay_us=DELAY_US,
        step_us=STEP_US,
        limits=LIMITS,
        mounts={
            key: dict(
                offset_m=value[0],
                rpy_degrees=value[1],
                board_to_chassis=rotation(value[1]),
            )
            for key, value in installations.items()
        },
        timing=dict(
            settle_us=600000,
            calibration_timeout_us=1500000,
            motion_onset_after_anchor_us=50000,
            scored_window_us=1000000,
            normal_source_period_us=dict(imu=10000, fused=20000),
            normal_motion="Straight +0.24 m/s²; turn +1.6 rad/s² for .5s then hold .8 rad/s or brake at -1.6 rad/s².",
            fast_motion="Right-offset fused: +1 rad/s² for 2s, hold 2 rad/s; score [A+2.05s,A+3.05s), retaining origin A.",
            fault="Combined mount: after calibration, gyro .8 rad/s for 1.2s with fixed wheels, then rest 1.5s.",
            reorientation="Board-center only: flat to roll+90 at fixed underside center (0,0,0), same stopped boot; setup-only change, motion scored after save/reboot and fresh calibration.",
        ),
        cases=cases,
    )


def _dump(path, value):
    def json_value(item):
        if isinstance(item, float) and not math.isfinite(item):
            # JSON has no nonfinite number. Retain an explicit tagged value;
            # the associated raw wire frame remains the authoritative encoding.
            return {"nonfinite_float": repr(item), "authority": "raw wire"}
        if isinstance(item, dict):
            return {key: json_value(value) for key, value in item.items()}
        if isinstance(item, (list, tuple)):
            return [json_value(value) for value in item]
        return item

    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(json_value(value), indent=2, allow_nan=False) + "\n", encoding="utf8")
    for attempt in range(5):
        try:
            temporary.replace(path)
            break
        except OSError as error:
            # A concurrent Windows reader can briefly deny replacing an
            # existing summary. Retry that atomic step only, for at most375ms.
            if (
                os.name != "nt"
                or getattr(error, "winerror", None) not in (5, 32, 33)
                or not path.exists()
                or attempt == 4
            ):
                raise
            time.sleep(0.025 * 2**attempt)


class PhysicalInput:
    """Only timer counts and BMI088 register observations cross into firmware."""

    def __init__(self, session, mount, reference="chip", *, angles=None):
        from .bmi088 import BMI088
        from .imu_mount import Mount

        offset, declared_angles = _installations(reference)[mount]
        angles = declared_angles if angles is None else angles
        self.center_to_imu = _profile_vector(session.firmware) if reference == "board-center" else None
        physical = (
            Mount.from_board_center(offset, rotation(angles), center_to_imu_m=self.center_to_imu)
            if reference == "board-center"
            else Mount(offset, rotation(angles))
        )
        self.sensor = BMI088(SEED, mount=physical)
        self.session = session
        self.pending, self.last = [], (0, 0)
        self.stamps, self.values = [0, 0], [0.0, 0.0, 9.80665, 0.0, 0.0, 0.0]
        session.board.set_imu(status=2, accel_us=0, gyro_us=0)

    def step(self, *, accel=0.0, velocity=0.0, gyro=0.0, alpha=0.0, wheel_gyro=None):
        s = self.session
        at = s.board.now_us + STEP_US
        self.sensor.advance(at, (accel, 0.0, 9.80665), (0.0, 0.0, gyro), yaw_acceleration=alpha)
        stamps = self.sensor.accel_us, self.sensor.gyro_us
        for channel, stamp in enumerate(stamps):
            if stamp != self.last[channel]:
                self.pending.append(
                    (
                        stamp + DELAY_US,
                        channel,
                        stamp,
                        self.sensor.values[3 * channel : 3 * channel + 3],
                    )
                )
        self.last = stamps
        # Independent mecanum rolling constraint, .1m wheels, .34m x .26m body.
        # External shaft counts only; no target/PWM or estimator-state injection.
        rate = gyro if wheel_gyro is None else wheel_gyro
        travel = [
            velocity + 0.30 * rate,
            velocity + 0.30 * rate,
            velocity - 0.30 * rate,
            velocity - 0.30 * rate,
        ]
        increments = [v * STEP_US / 1e6 * 4096 / (math.pi * 0.1) for v in travel]
        for i, increment in enumerate(increments):
            s.board._counts[i] += increment
        s.board.advance(STEP_US / 1e6)
        if any(s.board.motor_pwm):
            raise AssertionError("Unexpected PWM during passive mounting test")
        due = [entry for entry in self.pending if entry[0] <= at]
        self.pending = [entry for entry in self.pending if entry[0] > at]
        for _, channel, stamp, values in due:
            self.stamps[channel] = stamp
            self.values[3 * channel : 3 * channel + 3] = values
        if due:
            s.board.set_imu(
                self.values[:3],
                self.values[3:],
                accel_us=self.stamps[0],
                gyro_us=self.stamps[1],
            )
        s.mark(
            "physical_input",
            specific_force=(accel, 0.0, 9.80665),
            yaw_rate=gyro,
            yaw_acceleration=alpha,
            encoder_increment=increments,
            delivered=bool(due),
            accel_us=self.stamps[0],
            gyro_us=self.stamps[1],
            sensor_values=self.values,
        )


def _geometry(s, name, stimulus, reference="chip"):
    # Validate positional contract before relying on its field order. Names are
    # recorded from the actual canonical schema, so no SDK aliases are assumed.
    setter, getter = (
        ("SET_BOARD_MOUNT", "GET_BOARD_MOUNT")
        if reference == "board-center"
        else ("SET_IMU_MOUNT", "GET_IMU_MOUNT")
    )
    for command in (setter, getter, "SET_IMU_MOUNT", "GET_IMU_MOUNT"):
        if command not in s.schema.commands:
            raise LookupError(f"Firmware protocol {s.schema.version} lacks {command}")
    props = s.schema.commands[setter]["properties"]
    contract = list(
        zip(
            ("roll_cdeg", "pitch_cdeg", "yaw_cdeg", "x_um", "y_um", "z_um"),
            ("int16_t",) * 3 + ("int32_t",) * 3,
        )
    )
    if [(p["name"], p["type"]) for p in props] != contract:
        raise ValueError("Mount request differs from the canonical six-field contract")
    offset, angles = _installations(reference)[name]
    fields = dict(
        zip(
            (p["name"] for p in props),
            [round(v * 100) for v in angles] + [round(v * 1e6) for v in offset],
            strict=True,
        )
    )

    def write(command, values):
        try:
            s.ask(command, **values)
        except CommandError as exc:
            if exc.code != s.schema.errors["SOURCE_SWITCH_UNSAFE"]:
                raise
            for _ in range(150000 // STEP_US):
                stimulus.step()
            s.ask(command, **values)

    def read(values):
        observed = s.ask(getter)
        if (
            any(observed.get(key) != value for key, value in values.items())
            or observed.get("configured") != 1
        ):
            raise AssertionError(f"Mount readback mismatch: {values} -> {observed}")
        chip = s.ask("GET_IMU_MOUNT") if reference == "board-center" else observed
        if reference == "board-center":
            matrix = rotation(tuple(values[k] / 100 for k in ("roll_cdeg", "pitch_cdeg", "yaw_cdeg")))
            datum = _cad_vector_um()
            for index, key in enumerate(("x_um", "y_um", "z_um")):
                expected = values[key] + sum(matrix[index][j] * datum[j] for j in range(3))
                if abs(chip[key] - expected) > 0.500001:
                    raise AssertionError(f"Effective chip offset {key}: {chip[key]} != rounded {expected}")
            if (
                any(chip[key] != values[key] for key in ("roll_cdeg", "pitch_cdeg", "yaw_cdeg"))
                or chip["configured"] != 1
            ):
                raise AssertionError("Effective chip orientation or configuration differs")
        return observed, chip

    reorientation = []
    if name == "bottom_reorientation":
        flat = dict(fields, roll_cdeg=0, pitch_cdeg=0, yaw_cdeg=0)
        write(setter, flat)
        center, chip = read(flat)
        reorientation.append(dict(phase="flat", boot=s.boot, center=center, chip=chip))
        s.mark(
            "board_center_reorientation",
            phase="flat",
            setup_only=True,
            center=center,
            chip=chip,
        )
        for _ in range(150000 // STEP_US):
            stimulus.step()
        from .imu_mount import Mount

        stimulus.sensor.mount = Mount.from_board_center(
            offset, rotation(angles), center_to_imu_m=stimulus.center_to_imu
        )
    write(setter, fields)
    observed, chip = read(fields)
    if reorientation:
        reorientation.append(dict(phase="vertical", boot=s.boot, center=observed, chip=chip))
        s.mark(
            "board_center_reorientation",
            phase="vertical",
            setup_only=True,
            center=observed,
            chip=chip,
        )
        for _ in range(150000 // STEP_US):
            stimulus.step()
    legacy_roundtrip = None
    if reference == "board-center":
        # The legacy setter must continue to interpret XYZ as chip coordinates.
        write("SET_IMU_MOUNT", {key: chip[key] for key in fields})
        legacy_chip, legacy_center = s.ask("GET_IMU_MOUNT"), s.ask(getter)
        if legacy_chip != chip or legacy_center.get("configured") != 1:
            raise AssertionError("Legacy chip setter changed effective geometry")
        for key in fields:
            tolerance = 1 if key in ("x_um", "y_um", "z_um") else 0
            if abs(legacy_center[key] - fields[key]) > tolerance:
                raise AssertionError("Legacy chip-to-board readback changed the operator pose")
        legacy_roundtrip = dict(chip=legacy_chip, board_center=legacy_center)
        write(setter, fields)  # Restore exact operator coordinates before saving.
        observed, chip = read(fields)
    s.ask("SAVE_CONFIGURATION")
    image = s.board.fram_image
    s.reboot(image)
    restored, restored_chip = read(fields)
    if restored != observed:
        raise AssertionError(f"Saved mounting geometry changed on reboot: {restored}")
    if s.ask("GET_IMU_TRACKING_STATUS")["state"] != 0 or s.status()["valid"]:
        raise AssertionError("Reboot restored a live tracking estimate")
    s.mark("mount_round_trip", configured=fields, before=observed, after=restored)
    return dict(
        request=fields,
        readback=observed,
        reboot_readback=restored,
        fram_sha256=hashlib.sha256(image).hexdigest(),
        reference=reference,
        effective_chip=chip,
        reboot_effective_chip=restored_chip,
        legacy_roundtrip=legacy_roundtrip,
        reorientation=reorientation,
        physical_center_to_imu_m=stimulus.center_to_imu,
    )


def _reading(s):
    status = s.status()
    row = dict(public_us=s.board.now_us, status=status, pwm=s.board.motor_pwm)
    if status["valid"]:
        keys = dict(origin_id=status["origin_id"], sequence=status["sequence"])
        row["pose"] = s.ask("GET_FEEDBACK_SOURCE_VECTOR", kind=0, **keys)
        row["twist"] = s.ask("GET_FEEDBACK_SOURCE_VECTOR", kind=1, **keys)
        if s.status() != status:
            raise AssertionError("Selector changed within a synchronous vector read")
    return row


def _drive(profile, elapsed_us):
    # Held sensor input uses the midpoint of each 2.5ms physical interval.
    # The scorer separately evaluates analytic pose at reported source time.
    t = max(0.0, elapsed_us / 1e6 - 0.05)
    if elapsed_us <= 50000 or profile == "rest":
        return {}
    if profile == "straight":
        return dict(accel=0.24, velocity=0.24 * t)
    if profile == "fast_steady_yaw":
        return dict(gyro=min(t, 2.0), alpha=1.0 if t < 2 else 0.0)
    if t < 0.5:
        return dict(gyro=1.6 * t, alpha=1.6)
    if profile == "steady_yaw":
        return dict(gyro=0.8)
    return dict(gyro=max(0.0, 0.8 - 1.6 * (t - 0.5)), alpha=-1.6 if t < 1.0 else 0.0)


def capture(s, case, result):
    s.reboot((HERE / "fixtures" / "factory.fram").read_bytes())
    result["build"] = s.board.build
    # Boot/INIT occur even on an old release: its unsupported interface is
    # observed and preserved, rather than treating absent coverage as a pass.
    reference = case.get("reference", "chip")
    setter = "SET_BOARD_MOUNT" if reference == "board-center" else "SET_IMU_MOUNT"
    if setter not in s.schema.commands:
        raise LookupError(f"Firmware protocol {s.schema.version} lacks {setter}")
    s.ask(
        "INITIALIZE_MECANUM_PLATFORM",
        wheels_diameter=0.1,
        length=0.34,
        width=0.26,
        encoder_resolution=4096,
    )
    before_reboot = PhysicalInput(
        s,
        case["mount"],
        reference,
        angles=(0, 0, 0) if case["mount"] == "bottom_reorientation" else None,
    )
    result["geometry"] = _geometry(s, case["mount"], before_reboot, reference)
    stimulus = PhysicalInput(s, case["mount"], reference)
    s.ask("START_PLATFORM_ODOMETRY")
    for _ in range(600000 // STEP_US):
        stimulus.step()
    source = 1 if case["mode"] == "imu" else 2
    s.ask("SET_FEEDBACK_SOURCE", source=source)
    if source == 1:
        s.ask(
            "SET_IMU_TRACKING",
            action=0,
            stationary_mount_confirmed=1,
            mounting_yaw_rad=0.0,
        )
    else:
        s.ask("SET_FEEDBACK_FUSION_MOUNT", mounting_yaw_rad=0.0)
    deadline = s.board.now_us + 1500000
    while s.board.now_us < deadline:
        stimulus.step()
        row = _reading(s)
        result["rows"].append(row)
        if row["status"]["valid"]:
            break
        if row["status"]["reason"] == 6:
            raise AssertionError(f"Explicit producer fault during initialization: {row['status']}")
    else:
        raise TimeoutError("Tracking failed stationary initialization within 1.5s")
    anchor = row["status"]["timestamp_us"]
    result["anchor_us"] = anchor
    if s.board.now_us >= anchor + 50000:
        raise AssertionError("First publication arrived too late to schedule motion causally")
    if case["profile"] == "fault":
        available(sources(s))
        health = []
        result["health"] = health
        for phase, duration in (("conflict", 1200000), ("recover", 1500000)):
            for index in range(duration // STEP_US):
                stimulus.step(gyro=0.8 if phase == "conflict" else 0.0, wheel_gyro=0.0)
                if index % 8 == 7:
                    selected = _reading(s)
                    observed = sources(s)
                    health.append(
                        dict(
                            phase=phase,
                            public_us=s.board.now_us,
                            sources=observed,
                            selected=selected,
                        )
                    )
                    status = selected["status"]
                    if status["requested"] != 2 or status["active"] != (2 if status["valid"] else 255):
                        raise AssertionError("Fallback during physical disagreement")
                    for name in ("encoder", "imu", "fused"):
                        if observed[name]["reason"] != 0 or observed[name]["detail"] != 0:
                            raise AssertionError(f"False failed-sensor attribution: {name}: {observed[name]}")
                    if any(observed[name]["flags"] & 4 for name in ("encoder", "imu", "fused")):
                        unattributed_conflict(observed)
                    else:
                        available(observed)
        conflict = False
        for row in health:
            if row["phase"] == "conflict":
                try:
                    unattributed_conflict(row["sources"])
                    conflict = True
                except AssertionError:
                    pass
        try:
            available(health[-1]["sources"])
            recovered = True
        except AssertionError:
            recovered = False
        result["fault_checks"] = dict(conflict_reported=conflict, recovered=recovered)
        if not conflict or not recovered:
            raise AssertionError("Physical disagreement was hidden or did not recover")
        return
    offset = 2050000 if case["profile"] == "fast_steady_yaw" else 0
    result["window_offset_us"] = offset
    while s.board.now_us < anchor + offset + 1070000:
        stimulus.step(**_drive(case["profile"], s.board.now_us + STEP_US // 2 - anchor))
        observed = _reading(s)
        if s.board.now_us % 20000 == 0:
            observed["health"] = sources(s)
        result["rows"].append(observed)
    if source == 1:
        last = result["rows"][-1]["status"]
        if last["valid"] or last["reason"] != 5:
            raise AssertionError(f"IMU duration/expiry not preserved: {last}")


def run(args):
    if not __debug__:
        raise ValueError("Run with Python assertions enabled")
    firmware, output = args.firmware.resolve(), args.out.resolve()
    if output.exists():
        raise FileExistsError(f"Output must be new: {output}")
    identity = dict(
        firmware=pin(firmware, args.development, output),
        simulator=pin(SIM, args.development, output),
    )
    frozen = plan(getattr(args, "reference", "chip"))
    chosen = args.cases or [case["case"] for case in frozen["cases"]]
    known = {case["case"]: case for case in frozen["cases"]}
    if len(set(chosen)) != len(chosen) or set(chosen) - known.keys():
        raise ValueError("Unknown or repeated case")
    output.mkdir(parents=True, exist_ok=False)
    scripts = {
        name: hashlib.sha256((Path(__file__).parent / name).read_bytes()).hexdigest()
        for name in (
            "mounting_acceptance.py",
            "mounting_acceptance_score.py",
            "__main__.py",
        )
    }
    _dump(
        output / "plan.json",
        dict(**frozen, selected_cases=chosen, scripts=scripts, **identity),
    )
    summary = dict(
        **identity,
        reference=frozen["reference"],
        plan_sha256=hashlib.sha256((output / "plan.json").read_bytes()).hexdigest(),
        cases={},
    )

    def save_summary():
        outcomes = [value["outcome"] for value in summary["cases"].values()]
        summary["counts"] = dict(
            declared=len(known),
            planned=len(chosen),
            passed=outcomes.count("PASS"),
            failed=outcomes.count("FAIL"),
            incomplete=outcomes.count("INCOMPLETE"),
            pending=len(chosen) - len(outcomes),
        )
        if summary.get("source_changed") or summary["counts"]["pending"]:
            summary["outcome"] = "INCOMPLETE"
        elif "FAIL" in outcomes:
            summary["outcome"] = "FAIL"
        elif "INCOMPLETE" in outcomes:
            summary["outcome"] = "INCOMPLETE"
        else:
            summary["outcome"] = "PASS"
        _dump(output / "summary.json", summary)

    save_summary()
    for name in chosen:
        case = known[name]
        result = dict(**case, **identity, rows=[], anchor_us=None)
        with (output / (name + ".wire.jsonl")).open("x", encoding="utf8") as stream:
            session = Session(firmware, "0.4.0", stream, args.compiler)
            try:
                capture(session, case, result)
            except Exception as exc:
                result["error"] = f"{type(exc).__name__}: {exc}"
                if isinstance(exc, (AssertionError, CommandError)):
                    result["product_error"] = result["error"]
                result["trace"] = traceback.format_exc()
            finally:
                session.close()
        if case["profile"] == "fault":
            if result.get("fault_checks") == dict(conflict_reported=True, recovered=True) and not result.get(
                "error"
            ):
                outcome = "PASS"
            elif result.get("product_error") or result.get("fault_checks"):
                outcome = "FAIL"
            else:
                outcome = "INCOMPLETE"
            result["score"] = dict(outcome=outcome)
        else:
            try:
                result["score"] = score(
                    result["rows"],
                    profile=case["profile"],
                    mode=case["mode"],
                    anchor_us=result["anchor_us"],
                    window_offset_us=result.get("window_offset_us", 0),
                    error=result.get("error"),
                    product_error=result.get("product_error"),
                )
            except (KeyError, TypeError, ValueError, OverflowError) as exc:
                result["score"] = dict(
                    outcome="FAIL" if result.get("product_error") else "INCOMPLETE",
                    error=f"Malformed score input: {type(exc).__name__}: {exc}",
                    product_error=result.get("product_error"),
                )
        result["wire_sha256"] = hashlib.sha256((output / (name + ".wire.jsonl")).read_bytes()).hexdigest()
        _dump(output / (name + ".json"), result)
        summary["cases"][name] = dict(
            outcome=result["score"]["outcome"],
            error=result.get("error"),
            result_sha256=hashlib.sha256((output / (name + ".json")).read_bytes()).hexdigest(),
        )
        save_summary()
        reasons = result["score"].get("failures", []) + result["score"].get("incomplete", [])
        reason = result.get("error") or "; ".join(reasons)
        print(name, result["score"]["outcome"], reason, flush=True)
    for key, repo in (("firmware", firmware), ("simulator", SIM)):
        if pin(repo, args.development, output) != identity[key]:
            summary["source_changed"] = True
    save_summary()
    return {"PASS": 0, "FAIL": 1, "INCOMPLETE": 2}[summary["outcome"]]
