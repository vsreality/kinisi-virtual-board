"""External analytic scorer for the predeclared public mounting maneuvers.

Only observed public vectors enter here. None of these pose/reference values are
passed to the native firmware or its estimators.
"""

import math

LIMITS = dict(position_m=0.050, body_speed_m_s=0.020, yaw_rad=math.radians(2), yaw_rate_rad_s=0.020)
WINDOW_US = 1_000_000


def _check_shape(row, mode):
    def integer(value):
        if type(value) is not int or value < 0:
            raise ValueError("Expected nonnegative integer clock or status")

    integer(row["public_us"])
    status = row["status"]
    for key in (
        "valid",
        "requested",
        "active",
        "reason",
        "source_detail",
        "origin_id",
        "timestamp_us",
        "sequence",
        "age_us",
        "max_age_ms",
    ):
        integer(status[key])
    if status["valid"] not in (0, 1):
        raise ValueError("Invalid boolean validity")
    if len(row["pwm"]) != 4 or any(type(v) not in (int, float) for v in row["pwm"]):
        raise ValueError("Expected four numeric motor outputs")
    if status["valid"]:
        for name in ("pose", "twist"):
            integer(row[name]["sequence"])
            if any(type(row[name][key]) not in (int, float) for key in "xyz"):
                raise ValueError("Expected numeric public vector")
    if "health" in row:
        for name in ("encoder", "imu", "fused") if mode == "fused" else ("encoder", "imu"):
            for key in ("health", "reason", "detail", "flags"):
                integer(row["health"][name][key])


def reference(profile, elapsed_us):
    """Chassis origin from rest; motion begins 50 ms after the public origin."""
    t = max(0.0, elapsed_us / 1e6 - 0.05)
    x = vx = yaw = wz = 0.0
    if profile == "straight":
        x, vx = 0.12 * t * t, 0.24 * t
    elif profile in ("steady_yaw", "changing_yaw"):
        ramp = min(t, 0.5)
        yaw, wz = 0.8 * ramp * ramp, 1.6 * ramp
        if t > 0.5:
            tail = t - 0.5
            if profile == "steady_yaw":
                yaw += 0.8 * tail
            else:
                brake = min(tail, 0.5)
                yaw += 0.8 * brake - 0.8 * brake * brake
                wz = max(0.0, 0.8 - 1.6 * brake)
    elif profile == "fast_steady_yaw":
        ramp = min(t, 2.0)
        yaw, wz = 0.5 * ramp * ramp, ramp
        if t > 2.0:
            yaw += 2 * (t - 2.0)
    elif profile != "rest":
        raise ValueError(f"Unknown maneuver: {profile}")
    return dict(x=x, y=0.0, yaw=yaw, vx=vx, vy=0.0, wz=wz)


def score(rows, *, profile, mode, anchor_us, window_offset_us=0, error=None, product_error=None):
    """Require every source slot, coherent origin, finite vectors and fixed gates."""
    failures, incomplete, measured = ([product_error] if product_error else []), [], []
    peaks = {name: dict(value=0.0, limit=limit, source_us=None) for name, limit in LIMITS.items()}
    period = 10000 if mode == "imu" else 20000
    if anchor_us is None:
        return dict(
            outcome="FAIL" if failures else "INCOMPLETE",
            failures=failures,
            incomplete=[error or "No valid origin"],
            anchor_us=None,
            measured_samples=0,
            peaks=peaks,
            comparisons=[],
        )
    start = anchor_us + window_offset_us
    # The high-rate plateau starts between grid points; cover every source stamp
    # in its half-open one-second interval without inventing a new origin.
    expected = list(range(anchor_us, start + WINDOW_US, period))
    expected = [stamp for stamp in expected if stamp >= start]
    seen, origin, sequences, previous = {}, None, {}, None
    previous_public = None
    health_times = set()
    first_valid_public = None
    for index, row in enumerate(rows):
        try:
            _check_shape(row, mode)
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            incomplete.append(f"Malformed observation {index}: {exc}")
            continue
        status = row["status"]
        if previous_public is not None and row["public_us"] <= previous_public:
            failures.append("Public poll time did not increase")
        previous_public = row["public_us"]
        if any(row.get("pwm", [])):
            failures.append("Unexpected motor output")
        for name in ("encoder", "imu", "fused") if mode == "fused" else ("encoder", "imu"):
            if "health" in row:
                health_times.add(row["public_us"])
                health = row["health"][name]
                if (
                    health["health"] != 0
                    or health["reason"] != 0
                    or health["detail"] != 0
                    or not health["flags"] & 2
                    or health["flags"] & 4
                ):
                    failures.append(f"Unexpected unhealthy {name} during nominal maneuver")
        if not status["valid"]:
            if status["reason"] == 6:
                failures.append("Explicit producer fault")
            elif first_valid_public is not None:
                expired = (
                    mode == "imu" and row["public_us"] >= anchor_us + WINDOW_US and status["reason"] == 5
                )
                if not expired:
                    failures.append("Tracking became unavailable inside its observation interval")
            elif status["reason"] not in (1, 2, 8):
                failures.append("Unexpected unavailable state during initialization")
            continue
        if first_valid_public is None:
            first_valid_public = row["public_us"]
        stamp = status["timestamp_us"]
        if origin is None:
            origin = status["origin_id"]
        if status["origin_id"] != origin:
            failures.append("Origin changed during maneuver")
        if status["active"] != (1 if mode == "imu" else 2):
            failures.append("Selected source changed or fell back")
        if status["requested"] != status["active"] or status["reason"] != 0 or status["source_detail"] != 0:
            failures.append("Valid source has an inconsistent request or reason")
        if stamp > row["public_us"] or row["public_us"] - stamp != status["age_us"]:
            failures.append("Invalid public source clock")
        budget = status["max_age_ms"] * 1000
        if not 0 < budget <= (50000 if mode == "imu" else 60000) or not 0 <= status["age_us"] <= budget:
            failures.append("Valid selected read exceeded its freshness budget")
        if status["sequence"] in sequences and sequences[status["sequence"]] != stamp:
            failures.append("Sequence reused for another source timestamp")
        sequences[status["sequence"]] = stamp
        pose, twist = row["pose"], row["twist"]
        if pose["sequence"] != status["sequence"] or twist["sequence"] != status["sequence"]:
            failures.append("Incoherent public vectors")
            continue
        if not all(math.isfinite(v[k]) for v in (pose, twist) for k in "xyz"):
            failures.append("Nonfinite public vector")
            continue
        if previous is not None:
            old_stamp, old_sequence, old_pose, old_twist = previous
            if stamp < old_stamp or status["sequence"] < old_sequence:
                failures.append("Source time or sequence regressed")
            if stamp == old_stamp and (
                status["sequence"] != old_sequence or pose != old_pose or twist != old_twist
            ):
                failures.append("Held source changed its identity or vectors")
            if stamp > old_stamp and status["sequence"] <= old_sequence:
                failures.append("New source did not advance its sequence")
        previous = stamp, status["sequence"], pose, twist
        if not start <= stamp < start + WINDOW_US:
            continue
        if stamp in seen:
            if (pose, twist) != seen[stamp]:
                failures.append("Held source changed its vectors")
            continue
        seen[stamp] = (pose, twist)
        if row["public_us"] - stamp > 50000:
            failures.append("First publication latency exceeds 50 ms")
        ref = reference(profile, stamp - anchor_us)
        errors = dict(
            position_m=math.hypot(pose["x"] - ref["x"], pose["y"] - ref["y"]),
            body_speed_m_s=math.hypot(twist["x"] - ref["vx"], twist["y"] - ref["vy"]),
            yaw_rad=abs(pose["z"] - ref["yaw"]),
            yaw_rate_rad_s=abs(twist["z"] - ref["wz"]),
        )
        measured.append(dict(source_us=stamp, public_us=row["public_us"], reference=ref, errors=errors))
        for name, value in errors.items():
            if value > peaks[name]["value"]:
                peaks[name].update(value=value, source_us=stamp)
    missing = sorted(set(expected) - seen.keys())
    extra = sorted(seen.keys() - set(expected))
    if missing:
        incomplete.append(f"Missing {len(missing)} expected source slots")
    if not health_times:
        incomplete.append("No per-input health observations")
    if first_valid_public is not None and previous_public is not None:
        expected_health = set(range((first_valid_public // 20000 + 1) * 20000, previous_public + 1, 20000))
        missing_health = sorted(expected_health - health_times)
        if missing_health:
            incomplete.append(f"Missing {len(missing_health)} scheduled 20ms health observations")
    if extra:
        failures.append("Unexpected source phase or period")
    if error:
        incomplete.append(error)
    for name, peak in peaks.items():
        if peak["value"] > peak["limit"]:
            failures.append(f"{name} exceeded fixed limit")
    return dict(
        outcome="FAIL" if failures else "INCOMPLETE" if incomplete else "PASS",
        failures=sorted(set(failures)),
        incomplete=incomplete,
        anchor_us=anchor_us,
        window_start_us=start,
        window_end_us=start + WINDOW_US,
        source_period_us=period,
        expected_samples=len(expected),
        measured_samples=len(seen),
        missing_sources=missing,
        first_valid_public_us=first_valid_public,
        health_observations=len(health_times),
        peaks=peaks,
        comparisons=measured,
    )
