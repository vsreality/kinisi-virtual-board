"""Record the three long IMU-only development courses on the native Kinisi board."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import tempfile

if __package__:
    from . import encoder_baseline as baseline
else:
    import encoder_baseline as baseline

PRESETS = ("differential", "omni", "mecanum")
SEEDS = (131, 277, 419, 587, 743)
COURSES = ("stationary", "straight", "mixed")
CHANNELS = ("ax", "ay", "az", "gx", "gy", "gz")
ZERO = (0., 0., 0.)
STEP_MS = 10


def schedule(course):
    if course == "stationary":
        return 120_000, [(0, ZERO)]
    targets = [(0, ZERO)]
    if course == "straight":
        cycle = [(0, (.18, 0., 0.)), (2500, ZERO), (4500, (-.14, 0., 0.)), (7000, ZERO)]
    else:
        cycle = [(0, (.12, .05, .55)), (2500, (0., 0., .50)),
                 (4500, (-.10, -.05, -.60)), (7500, ZERO)]
    for start in range(2000, 62_000, 10_000):
        targets.extend((start + offset, twist) for offset, twist in cycle)
    return 62_000, targets


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def clean_revision(repo, paths):
    changes = subprocess.check_output(["git", "-C", str(repo), "status", "--porcelain",
                                       "--untracked-files=all", "--", *paths], text=True)
    if changes:
        raise ValueError(f"Recording inputs differ from the source commit in {repo}:\n{changes}")
    return baseline.revision(repo)


def export_recording(raw, out, case, course, preset, duration_ms):
    meta = json.loads((raw / "case.json").read_bytes())
    start = meta["origin_us"]
    imu_file, reference_file, targets_file = [out / folder / (case + ".csv")
                                               for folder in ("imu", "reference", "targets")]
    samples = duration_ms // STEP_MS + 1
    probes = {}
    path_m = 0.
    previous = (0., 0.)
    model = meta["imu_model"]
    with (raw / "sensors.csv").open(newline="") as source, (raw / "reference.csv").open(newline="") as truth:
        sensors = list(csv.DictReader(source))
        reader = csv.DictReader(truth)
        reference_fields = reader.fieldnames
        references = list(reader)
    end_delivery = start + (duration_ms + STEP_MS) * 1000
    expected_reference = list(range(meta["reference_start_us"], end_delivery + 1, model["integration_us"]))
    if [int(r["at_us"]) for r in references] != expected_reference:
        raise ValueError("Missing, duplicate or shifted independent reference")
    if len(sensors) != samples + 1:
        raise ValueError("Unexpected sensor recording length")
    by_time = {int(r["at_us"]): r for r in references}
    for i, s in enumerate(sensors):
        delivered = start + i * STEP_MS * 1000
        if int(s["at_us"]) != delivered:
            raise ValueError("Missing, duplicate or shifted sensor delivery")
        for channel in ("accel", "gyro"):
            latest = delivered - model["transport_delay_us"]
            expected = latest - (latest - model[channel + "_phase_us"]) % model["source_period_us"]
            if int(s[channel + "_us"]) != expected or expected not in by_time:
                raise ValueError("Independent channel timestamp has no matching reference")
        if not all(math.isfinite(float(s[k])) for k in CHANNELS):
            raise ValueError("Nonfinite sensor observation")
    # Retain the first course accel acquisition and the earlier independently phased gyro.
    # Reference rows cover both channels exactly, as well as every delivery including the tail.
    exported = sensors[1:]
    first_source = min(int(exported[0][k]) for k in ("accel_us", "gyro_us"))
    with imu_file.open("x", newline="") as imu_out, reference_file.open("x", newline="") as ref_out:
        imu = csv.writer(imu_out)
        ref = csv.DictWriter(ref_out, fieldnames=reference_fields)
        imu.writerow(("delivered_us", "accel_us", "gyro_us", *CHANNELS))
        ref.writeheader()
        for s in exported:
            imu.writerow((s["at_us"], s["accel_us"], s["gyro_us"], *(s[k] for k in CHANNELS)))
        for r in references:
            if not all(math.isfinite(float(v)) for v in r.values()):
                raise ValueError("Nonfinite independent reference")
            at_us = int(r["at_us"])
            if at_us >= first_source:
                ref.writerow(r)
            elapsed_us = at_us - start
            pose = float(r["x"]), float(r["y"])
            twist = tuple(float(r[k]) for k in ("vx", "vy", "wz"))
            if 0 <= elapsed_us <= duration_ms * 1000:
                path_m += math.dist(previous, pose)
                previous = pose
            if course == "stationary" or elapsed_us <= 2_000_000:
                if any(twist) or any(pose) or float(r["yaw"]):
                    raise ValueError(f"Stationary interval moved at {elapsed_us} us")
            elif elapsed_us % 1000 == 0 and (elapsed_us // 1000 - 2000) % 10_000 in (2000, 4200, 6600, 9800):
                probes[elapsed_us // 1000] = twist
    for cycle in range(6) if course != "stationary" else ():
        base = 2000 + cycle * 10_000
        forward, midway, reverse, stopped = [probes[base + offset] for offset in (2000, 4200, 6600, 9800)]
        if forward[0] <= .05 or reverse[0] >= -.05 or any(abs(v) > .04 for v in stopped):
            raise ValueError(f"Missing forward motion, reversal or stop in cycle {cycle + 1}")
        if course == "straight" and (any(abs(v) > .04 for v in midway) or abs(forward[2]) > .04 or abs(reverse[2]) > .04):
            raise ValueError(f"Incorrect straight motion in cycle {cycle + 1}")
        if course == "mixed":
            if forward[2] <= .1 or midway[2] <= .1 or reverse[2] >= -.1:
                raise ValueError(f"Missing turns in cycle {cycle + 1}")
            if preset != "differential" and (forward[1] <= .01 or reverse[1] >= -.01):
                raise ValueError(f"Missing lateral motion in cycle {cycle + 1}")
    shutil.copyfile(raw / "targets.csv", targets_file)
    return dict(samples=samples, source_origin_us=start, start_acquired_us=first_source,
                end_acquired_us=int(exported[-1]["accel_us"]), end_delivered_us=end_delivery,
                imu_model=model, duration_ms=duration_ms,
                ground_path_m=path_m, firmware_build_id=meta["firmware_build_id"],
                imu_file=imu_file.relative_to(out).as_posix(), imu_sha256=digest(imu_file),
                reference_path=reference_file.relative_to(out).as_posix(), reference_sha256=digest(reference_file),
                targets_path=targets_file.relative_to(out).as_posix(), targets_sha256=digest(targets_file))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--firmware", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path, help="New development recording directory")
    args = parser.parse_args()
    fw = args.firmware.resolve()
    sim_revision = clean_revision(baseline.ROOT, ["kinisi_virtual_board", "tools/imu_development.py",
                                                  "tools/encoder_baseline.py", "tools/encoder_baseline_cases.json"])
    fw_revision = clean_revision(fw, ["src", "include", "lib", "test", "tools", "boards", "commands.json"])
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    for folder in ("imu", "reference", "targets"):
        (out / folder).mkdir()
    settings = json.loads(baseline.CASES.read_bytes())
    spec = {key: settings[key] for key in ("revision", "presets", "encoder_counts_per_rev", "wheel_pid")}
    spec.update(step_ms=STEP_MS, surfaces={"nominal": []})
    manifest = dict(development_only=True, reserved_accessed=False, simulator_revision=sim_revision,
                    firmware_revision=fw_revision, schema_sha256=digest(fw / "commands.json"),
                    generator_sha256=digest(Path(__file__)), geometry_source_sha256=digest(baseline.CASES),
                    seeds=SEEDS, step_ms=STEP_MS, cases=[], invalid=[])
    for preset in PRESETS:
        for course in COURSES:
            duration_ms, targets = schedule(course)
            spec["courses"] = {"velocity": dict(duration_ms=duration_ms + STEP_MS, targets=targets)}
            for seed in SEEDS:
                name = f"{preset}-{course}-nominal-{seed}"
                with tempfile.TemporaryDirectory(dir=out) as temp:
                    raw = Path(temp) / name
                    try:
                        baseline.run_case(raw, fw, spec, preset, "velocity", "nominal", seed, stationary_warmup=True)
                        build = out / "firmware-build.json"
                        if not build.exists():
                            shutil.copyfile(raw.parent / build.name, build)
                        record = export_recording(raw, out, name, course, preset, duration_ms)
                        manifest["cases"].append(dict(case=name, dataset_id="imu-development-bmi088-v1/" + name,
                                                      preset=preset, course=course, surface="nominal",
                                                      seed=seed, **record))
                    except Exception as exc:
                        invalid = out / "invalid" / name
                        invalid.parent.mkdir(exist_ok=True)
                        shutil.copytree(temp, invalid)
                        manifest["invalid"].append(dict(case=name, error=repr(exc)))
                        print(f"INVALID {name}: {exc}", flush=True)
                baseline.dump(out / "manifest.json", manifest)
    print(f"{len(manifest['cases'])} valid, {len(manifest['invalid'])} invalid", flush=True)
    return int(bool(manifest["invalid"]))


if __name__ == "__main__":
    raise SystemExit(main())
