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
    with (raw / "sensors.csv").open(newline="") as source, (raw / "reference.csv").open(newline="") as truth, \
            imu_file.open("x", newline="") as imu_out, reference_file.open("x", newline="") as ref_out:
        sensors, references = csv.DictReader(source), csv.DictReader(truth)
        imu = csv.writer(imu_out)
        ref = csv.DictWriter(ref_out, fieldnames=references.fieldnames)
        imu.writerow(("delivered_us", "accel_us", "gyro_us", *CHANNELS))
        ref.writeheader()
        # Drop the pre-origin acquisition. One extra simulation step delivers the final observation.
        first = next(sensors)
        if int(first["at_us"]) != start or int(first["imu_acquired_us"]) != start - STEP_MS * 1000:
            raise ValueError("Unexpected pre-origin observation")
        for i in range(samples):
            s, r = next(sensors), next(references)
            elapsed_ms = i * STEP_MS
            stamp = start + elapsed_ms * 1000
            if (int(s["imu_acquired_us"]), int(r["at_us"]), int(s["at_us"])) != (stamp, stamp, stamp + STEP_MS * 1000):
                raise ValueError(f"Sensor or reference timing mismatch at row {i}")
            if not all(math.isfinite(float(s[k])) for k in CHANNELS) or not all(math.isfinite(float(v)) for v in r.values()):
                raise ValueError(f"Nonfinite observation at row {i}")
            imu.writerow((s["at_us"], stamp, stamp, *(s[k] for k in CHANNELS)))
            ref.writerow(r)
            pose = float(r["x"]), float(r["y"])
            path_m += math.dist(previous, pose)
            previous = pose
            twist = tuple(float(r[k]) for k in ("vx", "vy", "wz"))
            if course == "stationary" or elapsed_ms <= 2000:
                if any(twist) or any(pose) or float(r["yaw"]):
                    raise ValueError(f"Stationary interval moved at {elapsed_ms} ms")
            elif (elapsed_ms - 2000) % 10_000 in (2000, 4200, 6600, 9800):
                probes[elapsed_ms] = twist
        tail = next(references)
        if int(tail["at_us"]) != stamp + STEP_MS * 1000 or next(references, None) or next(sensors, None):
            raise ValueError("Unexpected recording length")
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
    return dict(samples=samples, start_acquired_us=start, end_acquired_us=stamp, duration_ms=duration_ms,
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
    manifest = dict(development_only=True, simulator_revision=sim_revision,
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
                        manifest["cases"].append(dict(case=name, preset=preset, course=course, surface="nominal",
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
