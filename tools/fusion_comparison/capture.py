"""Public-wire fusion mode adapter; native motor control stays in firmware."""
import csv
import struct
from unittest.mock import patch
import encoder_baseline as collector
import score_encoder_baseline as scorer

STATUS = struct.Struct("<BBBBIQIII")
VECTOR = struct.Struct("<Iddd")
STATUS_COLUMNS = ("mode","health","fault","flags","sequence","sample_us","accel_age_us","gyro_age_us","encoder_age_us")
MODES = {"release":0,"encoder":1,"fused":2}

def experiment_wire(mode, status_log):
    """A test client that adds only public configuration and diagnostic commands."""

    class ExperimentWire(collector.Wire):
        def ask(self, command, **fields):
            answer = super().ask(command, **fields)
            if command == "GET_PLATFORM_ODOMETRY":
                status = STATUS.unpack(super().ask("GET_FUSION_STATUS"))
                if status[0] != mode:
                    raise ValueError(f"Requested mode {mode}, active {status[0]}")
                vectors = [""] * 6
                if status[3] & 1:
                    pose_kind = 2 if mode == MODES["fused"] else 0
                    pose = VECTOR.unpack(
                        super().ask("GET_FUSION_VECTOR", kind=pose_kind)
                    )
                    twist = VECTOR.unpack(
                        super().ask("GET_FUSION_VECTOR", kind=pose_kind + 1)
                    )
                    if pose[0] != status[4] or twist[0] != status[4]:
                        raise ValueError("Snapshot changed during diagnostic reads")
                    vectors = [*pose[1:], *twist[1:]]
                status_log.writerow([self.board.now_us, *status, *vectors])
            return answer

    return ExperimentWire


def collect(directory, fw, spec, preset, course, surface, seed, variant):
    if variant == "release":
        collector.run_case(
            directory, fw, spec, preset, course, surface, seed, stationary_warmup=True
        )
    else:
        mode = MODES[variant]

        def setup(ask):
            ask("SET_FUSION_EXPERIMENT", mode=mode, mounting_yaw_rad=0.0)
            actual = struct.unpack("<Bd", ask("GET_FUSION_CONFIGURATION"))
            if actual != (mode, 0.0):
                raise ValueError(f"Experiment configuration mismatch: {actual}")

        status_path = directory.with_suffix(".status.csv")
        try:
            with status_path.open("x", encoding="utf-8", newline="") as stream:
                status = csv.writer(stream)
                status.writerow(
                    ("at_us", *STATUS_COLUMNS, "x", "y", "yaw", "vx", "vy", "wz")
                )
                with patch.object(collector, "Wire", experiment_wire(mode, status)):
                    collector.run_case(
                        directory,
                        fw,
                        spec,
                        preset,
                        course,
                        surface,
                        seed,
                        setup=setup,
                        stationary_warmup=True,
                    )
        finally:
            if status_path.exists():
                directory.mkdir(exist_ok=True)
                status_path.rename(directory / "status.csv")
    scores = scorer.score_case(directory, spec)
    frames = scorer.read_csv(directory / "firmware.csv")
    dispatches = []
    for before, after in zip(frames, frames[1:]):
        change = after["controller_dispatches"] - before["controller_dispatches"]
        if change not in (0, 1):
            raise ValueError(
                "Missing or duplicate controller dispatch on observation grid"
            )
        if change:
            dispatches.append(after["at_us"])
    intervals = [(b - a) / 1000 for a, b in zip(dispatches, dispatches[1:])]
    scores.update(
        control_min_interval_ms=min(intervals), control_max_interval_ms=max(intervals)
    )
    if variant != "release":
        with (directory / "status.csv").open(encoding="utf-8", newline="") as stream:
            diagnostics = list(csv.DictReader(stream, strict=True))
        scores["diagnostics"] = diagnostic_summary(diagnostics)
    return scores


def diagnostic_summary(rows):
    """Keep fault/flag distributions and unavailable ages, never filter intervals."""

    def counts(field):
        values = [int(row[field]) for row in rows]
        return {str(value): values.count(value) for value in sorted(set(values))}

    return dict(
        samples=len(rows),
        invalid_estimate_samples=sum(not (int(r["flags"]) & 1) for r in rows),
        fault_counts=counts("fault"),
        health_counts=counts("health"),
        flag_counts=counts("flags"),
        ages={
            field: dict(
                unavailable_samples=sum(int(r[field]) == 0xFFFFFFFF for r in rows),
                maximum_us=max(
                    (int(r[field]) for r in rows if int(r[field]) != 0xFFFFFFFF),
                    default=None,
                ),
            )
            for field in ("accel_age_us", "gyro_age_us", "encoder_age_us")
        },
    )

