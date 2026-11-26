"""Offline scoring of encoder estimates and tracking against simulated chassis motion."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics as stats
from collections import defaultdict
from itertools import product


HEADERS = {
    "firmware.csv": "at_us odom_us clock_mode clock_quality x y yaw controller_dispatches pwm0 pwm1 pwm2 pwm3",
    "sensors.csv": "at_us accel_us gyro_us enc0 enc1 enc2 enc3 ax ay az gx gy gz",
    "reference.csv": "at_us x y yaw vx vy wz",
    "targets.csv": "at_us x y yaw",
}


def read_csv(path):
    with path.open(encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f, strict=True)
        rows = []
        try:
            fields = reader.fieldnames or []
            required = HEADERS[path.name].split()
            if path.name == "sensors.csv" and "imu_acquired_us" in fields and "accel_us" not in fields:
                required = [k for k in required if k not in ("accel_us", "gyro_us")] + ["imu_acquired_us"]
            if len(fields) != len(set(fields)) or not set(required).issubset(fields):
                raise ValueError("Missing or duplicate columns")
            for row in reader:
                if None in row or None in row.values():
                    raise ValueError("Incorrect column count")
                rows.append({k: float(v) for k, v in row.items()})
        except (ValueError, csv.Error) as exc:
            raise ValueError(f"{path}, line {reader.line_num}: {exc}") from exc
    if not rows or not all(math.isfinite(v) for row in rows for v in row.values()):
        raise ValueError(f"Empty or nonfinite trace: {path}")
    return rows


def pose(row):
    return row["x"], row["y"], row["yaw"]


def angle_error(a, b):
    return math.remainder(a - b, math.tau)


def turn(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return c*x - s*y, s*x + c*y


def body_rate(a, b, dt):
    dx, dy = turn(b[0]-a[0], b[1]-a[1], -(a[2]+b[2])/2)
    return dx/dt, dy/dt, (b[2]-a[2])/dt


def pose_error(estimate, ref):
    return math.hypot(estimate[0]-ref[0], estimate[1]-ref[1]), abs(angle_error(estimate[2], ref[2]))


def rms(values):
    return math.sqrt(stats.mean(v*v for v in values))


def advance_target(p, velocity, dt):
    # Exact constant body-twist integration for the velocity-command reference.
    vx, vy, wz = velocity
    dtheta = wz * dt
    if wz:
        sn = math.sin(dtheta)/wz
        cs = 2 * math.sin(dtheta/2)**2 / wz
        dx, dy = vx*sn - vy*cs, vx*cs + vy*sn
    else:
        dx, dy = vx*dt, vy*dt
    dx, dy = turn(dx, dy, p[2])
    return p[0]+dx, p[1]+dy, p[2]+dtheta


def score_case(directory, spec):
    case = json.loads((directory/"case.json").read_text())
    firmware = read_csv(directory/"firmware.csv")
    sensors = read_csv(directory/"sensors.csv")
    reference = read_csv(directory/"reference.csv")
    targets = read_csv(directory/"targets.csv")
    dt_us = spec["step_ms"] * 1000
    dt = dt_us/1e6
    origin = case["origin_us"]
    duration = case["duration_ms"]/1000
    times = list(range(origin, origin + case["duration_ms"]*1000 + 1, dt_us))
    for trace in (firmware, sensors):
        if [r["at_us"] for r in trace] != times:
            raise ValueError("Missing, duplicate, or shifted acquisition")
    model = case.get("imu_model")
    reference_times = list(range(case["reference_start_us"], times[-1]+1, model["integration_us"])) if model else times
    if [r["at_us"] for r in reference] != reference_times:
        raise ValueError("Missing, duplicate, or shifted reference")
    if model:
        for s in sensors:
            for channel in ("accel", "gyro"):
                latest = s["at_us"] - model["transport_delay_us"]
                expected = latest - (latest - model[channel + "_phase_us"]) % model["source_period_us"]
                if s[channel + "_us"] != expected:
                    raise ValueError("Independent IMU provenance mismatch")
    elif any(s["at_us"] - s["imu_acquired_us"] != dt_us for s in sensors):
        raise ValueError("IMU provenance mismatch")
    truth_at = {r["at_us"]: pose(r) for r in reference}
    by_time = {r["at_us"]: r for r in reference}
    reference = [by_time[t] for t in times]
    command_at = {r["at_us"]: pose(r) for r in targets}
    course = spec["courses"][case["mode"]]
    schedule = {origin+t*1000: (v[0], 0 if case["preset"]=="differential" and case["mode"]=="velocity"
                                else v[1], v[2]) for t, v in course["targets"]}
    if command_at != schedule or len(targets) != len(schedule):
        raise ValueError("Executed targets differ from frozen course")
    onset = next(t for t, v in command_at.items() if any(v))
    ages, pos, heading, speed, yaw_rate, stamps = [], [], [], [], [], []
    track_pos, track_heading, track_speed, track_wz = [], [], [], []
    desired = (0., 0., 0.)
    target = desired
    pwm_on = odom_on = body_on = None
    path_m = 0.
    previous_stamp = None
    previous_pose = None
    for i, (f, r) in enumerate(zip(firmware, reference)):
        now, stamp = f["at_us"], f["odom_us"]
        if stamp not in truth_at or stamp > now or (previous_stamp is not None and stamp < previous_stamp):
            raise ValueError("Invalid odometry timestamp")
        if (f["clock_mode"], f["clock_quality"]) != (0, 1):
            raise ValueError("Odometry timestamp failed uptime/quality check")
        estimate = pose(f)
        ages.append((now-stamp)/1000)
        if stamp == previous_stamp and estimate != previous_pose:
            raise ValueError("Odometry pose changed without a new sample timestamp")
        if stamp != previous_stamp:
            stamps.append(stamp)
            pe, he = pose_error(estimate, truth_at[stamp])
            pos.append(pe)
            heading.append(he)
            if previous_stamp is not None:
                period = (stamp-previous_stamp)/1e6
                est_rate = body_rate(previous_pose, estimate, period)
                ref_rate = body_rate(truth_at[previous_stamp], truth_at[stamp], period)
                speed.append(math.hypot(est_rate[0]-ref_rate[0], est_rate[1]-ref_rate[1]))
                yaw_rate.append(abs(est_rate[2]-ref_rate[2]))
            previous_stamp, previous_pose = stamp, estimate
        if now >= onset:
            if pwm_on is None and any(f[f"pwm{k}"] for k in range(4)):
                pwm_on = now
            if odom_on is None and any(estimate):
                odom_on = now
            if body_on is None and any(pose(r)):
                body_on = now
        if i == 0:
            continue
        target = command_at.get(times[i-1], target)
        last = reference[i-1]
        path_m += math.hypot(r["x"]-last["x"], r["y"]-last["y"])
        if case["mode"] == "velocity":
            desired = advance_target(desired, target, dt)
            # Target applies on [previous time, now); sample actual twist at the end.
            track_speed.append(math.hypot(r["vx"]-target[0], r["vy"]-target[1]))
            track_wz.append(abs(r["wz"]-target[2]))
        else:
            desired = target
        pe, he = pose_error(pose(r), desired)
        track_pos.append(pe)
        track_heading.append(he)
    if len(stamps) < 2:
        raise ValueError("Insufficient odometry history for velocity scoring")
    def response_ms(observed):
        return None if observed is None else (observed-onset)/1000
    periods = [b-a for a,b in zip(stamps, stamps[1:])]
    values = dict(est_position_rmse_m=rms(pos), est_heading_rmse_rad=rms(heading),
                  est_linear_velocity_rmse_m_s=rms(speed), est_yaw_rate_rmse_rad_s=rms(yaw_rate),
                  est_end_position_m=pos[-1], est_end_heading_rad=heading[-1],
                  est_end_drift_m_per_m=pos[-1]/path_m if path_m else None, ground_path_m=path_m,
                  tracking_position_rmse_m=rms(track_pos), tracking_heading_rmse_rad=rms(track_heading),
                  tracking_end_position_m=track_pos[-1], tracking_end_heading_rad=track_heading[-1],
                  pwm_response_ms=response_ms(pwm_on), odometry_response_ms=response_ms(odom_on),
                  ground_motion_response_ms=response_ms(body_on), held_sample_age_max_ms=max(ages),
                  held_sample_age_mean_ms=stats.mean(ages), odometry_hz=1e6/stats.mean(periods),
                  odometry_period_max_ms=max(periods)/1000,
                  imu_hz=1e6/model["source_period_us"] if model else 1/dt,
                  control_hz=(firmware[-1]["controller_dispatches"]-firmware[0]["controller_dispatches"])/duration)
    if model:
        for channel in ("accel", "gyro"):
            age = [(s["at_us"]-s[channel+"_us"])/1000 for s in sensors]
            values[channel+"_delivery_age_max_ms"] = max(age)
            values[channel+"_delivery_age_mean_ms"] = stats.mean(age)
    if track_speed:
        values.update(tracking_linear_velocity_rmse_m_s=rms(track_speed), tracking_yaw_rate_rmse_rad_s=rms(track_wz))
    return values


def fingerprint(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    root = parser.parse_args().directory
    spec = json.loads((root/"cases.json").read_text())
    manifest = json.loads((root/"manifest.json").read_text())
    if fingerprint(root/"cases.json") != manifest["cases_sha256"]:
        raise ValueError("Schedule fingerprint mismatch")
    build_id = json.loads((root/"firmware-build.json").read_text())["build_id"]
    runs = list(product(spec["presets"], spec["courses"], spec["surfaces"], spec["seeds"]))
    rows, invalid, groups, paired = [], [], defaultdict(list), {}
    for preset, mode, surface, seed in runs:
        name = f"{preset}-{mode}-{surface}-{seed}"
        path = root/name
        try:
            case = json.loads((path/"case.json").read_text())
            if (case["preset"], case["mode"], case["surface"], case["seed"]) != (preset,mode,surface,seed):
                raise ValueError("Case identity mismatch")
            if case["firmware_build_id"] != build_id:
                raise ValueError("Firmware build identity mismatch")
            if (path/"invalid.json").exists():
                raise ValueError((path/"invalid.json").read_text())
            result = score_case(path, spec)
            # Under encoder-only control the external slip model cannot change firmware responses.
            key = preset,mode,seed
            trace_hash = fingerprint(path/"firmware.csv")
            if paired.setdefault(key,trace_hash) != trace_hash:
                raise ValueError("Firmware trajectory changed across matched contact conditions")
            rows.append(dict(preset=preset, mode=mode, surface=surface, seed=seed, **result))
            groups[preset,mode,surface].append(result)
        except (OSError, KeyError, ValueError, ZeroDivisionError, StopIteration) as exc:
            invalid.append({"run": name, "error": str(exc)})
    first = "-".join(map(str, runs[0]))
    repeat = root/(first+"-repeat")
    repeat_errors = []
    for file in ("case.json", "reference.csv", "firmware.csv", "sensors.csv", "targets.csv"):
        source, other = root/first/file, repeat/file
        if not source.exists() or not other.exists() or fingerprint(source) != fingerprint(other):
            repeat_errors.append(file)
    if repeat_errors or (repeat/"invalid.json").exists():
        invalid.append({"run":first+"-repeat","error":f"Determinism check failed: {repeat_errors}"})
    summary = []
    for key, results in groups.items():
        metrics = {}
        for metric in results[0]:
            values = [r[metric] for r in results if r[metric] is not None]
            metrics[metric] = dict(n_measured=len(values), mean=stats.mean(values) if values else None,
                                   sd=stats.stdev(values) if len(values)>1 else None,
                                   minimum=min(values, default=None), maximum=max(values, default=None))
        summary.append(dict(preset=key[0], mode=key[1], surface=key[2], n=len(results), metrics=metrics))
    output = dict(valid=len(rows), planned=len(runs), repeat_identical=not repeat_errors and
                  not (repeat/"invalid.json").exists(), invalid=invalid, groups=summary)
    (root/"summary.json").write_text(json.dumps(output, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    if rows:
        with (root/"scores.csv").open("w", newline="", encoding="utf-8") as f:
            fields = list(dict.fromkeys(k for r in rows for k in r))
            w = csv.DictWriter(f, fields)
            w.writeheader()
            w.writerows(rows)
    print(f"Scored {len(rows)}/{len(runs)}. Identical repeat: {output['repeat_identical']}. Invalid: {len(invalid)}")
    return int(bool(invalid))


if __name__ == "__main__":
    raise SystemExit(main())
