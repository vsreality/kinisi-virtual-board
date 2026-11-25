"""Synthetic acceptance negatives; no firmware, schedules or qualification inputs."""

import copy
import math
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from kinisi_virtual_board.mounting_acceptance_score import reference, score


def fixture(profile="rest", mode="imu", offset=0):
    period = 10000 if mode == "imu" else 20000
    rows = []
    for at in range(0, offset + 1000000, period):
        if at < offset:
            continue
        truth = reference(profile, at)
        sequence = at // period + 1
        rows.append(
            dict(
                public_us=1000000 + at + 20000,
                pwm=[0] * 4,
                status=dict(
                    valid=1,
                    requested=1 if mode == "imu" else 2,
                    active=1 if mode == "imu" else 2,
                    reason=0,
                    source_detail=0,
                    origin_id=1,
                    timestamp_us=1000000 + at,
                    sequence=sequence,
                    age_us=20000,
                    max_age_ms=50 if mode == "imu" else 60,
                ),
                pose=dict(sequence=sequence, x=truth["x"], y=truth["y"], z=truth["yaw"]),
                twist=dict(sequence=sequence, x=truth["vx"], y=truth["vy"], z=truth["wz"]),
                health={
                    name: dict(health=0, reason=0, detail=0, flags=11) for name in ("encoder", "imu", "fused")
                },
            )
        )
    return rows


class MountingScoreTests(unittest.TestCase):
    def check(self, rows, outcome="FAIL", **kwargs):
        result = score(rows, profile="rest", mode="imu", anchor_us=1000000, **kwargs)
        self.assertEqual(result["outcome"], outcome, result)
        return result

    def test_full_analytic_profiles(self):
        for profile in ("rest", "straight", "steady_yaw", "changing_yaw"):
            for mode in ("imu", "fused"):
                with self.subTest(profile=profile, mode=mode):
                    result = score(fixture(profile, mode), profile=profile, mode=mode, anchor_us=1000000)
                    self.assertEqual(result["outcome"], "PASS", result)

    def test_unwrapped_fast_plateau(self):
        rows = fixture("fast_steady_yaw", "fused", 2050000)
        result = score(
            rows, profile="fast_steady_yaw", mode="fused", anchor_us=1000000, window_offset_us=2050000
        )
        self.assertEqual(result["outcome"], "PASS")
        self.assertEqual(result["measured_samples"], 50)
        self.assertGreater(rows[-1]["pose"]["z"], math.pi)

    def test_wrapped_or_extra_revolution_is_wrong(self):
        rows = fixture()
        rows[-1]["pose"]["z"] += math.tau
        self.check(rows)

    def test_missing_source(self):
        self.check(fixture()[:-1], "INCOMPLETE")

    def test_missing_health(self):
        rows = fixture()
        for row in rows:
            row.pop("health")
        self.check(rows, "INCOMPLETE")

    def test_numeric_limit(self):
        rows = fixture()
        rows[0]["pose"]["x"] = 0.051
        self.check(rows)

    def test_vector_coherence(self):
        rows = fixture()
        rows[0]["twist"]["sequence"] = 999
        self.check(rows)

    def test_stale_held_valid(self):
        rows = fixture()
        held = copy.deepcopy(rows[-1])
        held["public_us"] += 100000
        held["status"]["age_us"] += 100000
        rows.append(held)
        self.check(rows)

    def test_reversed_sequence(self):
        rows = fixture()
        for key in ("status", "pose", "twist"):
            rows[5][key]["sequence"] = 1
        self.check(rows)

    def test_changed_hold(self):
        rows = fixture()
        held = copy.deepcopy(rows[-1])
        held["public_us"] += 2500
        held["status"]["age_us"] += 2500
        held["pose"]["x"] += 0.001
        rows.append(held)
        self.check(rows)

    def test_explicit_fault_between_complete_slots(self):
        rows = fixture()
        row = copy.deepcopy(rows[5])
        row["public_us"] += 2500
        row["status"].update(valid=0, reason=6, active=255)
        rows.insert(6, row)
        self.check(rows)

    def test_expiry_allowed_only_after_horizon(self):
        rows = fixture()
        for public_us in (2020000, 2040000, 2060000):
            row = copy.deepcopy(rows[-1])
            row["public_us"] = public_us
            row["status"].update(valid=0, reason=5, active=255)
            rows.append(row)
        self.check(rows, "PASS")
        rows = fixture()
        row = copy.deepcopy(rows[5])
        row["public_us"] += 2500
        row["status"].update(valid=0, reason=5, active=255)
        rows.insert(6, row)
        self.check(rows)

    def test_fault_before_origin(self):
        result = score([], profile="rest", mode="imu", anchor_us=None, product_error="Mount invalid")
        self.assertEqual(result["outcome"], "FAIL")

    def test_pwm(self):
        rows = fixture()
        rows[0]["pwm"] = [1, 0, 0, 0]
        self.check(rows)

    def test_health_fault(self):
        rows = fixture()
        rows[0]["health"]["imu"]["reason"] = 3
        self.check(rows)

    def test_health_gap(self):
        rows = fixture()
        rows[2].pop("health")
        self.check(rows, "INCOMPLETE")

    def test_calibration_until_first_public_read(self):
        rows = fixture()
        row = copy.deepcopy(rows[0])
        row["public_us"] -= 10000
        row["status"].update(valid=0, reason=2, active=255)
        rows.insert(0, row)
        self.check(rows, "PASS")

    def test_malformed_row_preserves_other_failures(self):
        rows = fixture()
        rows[0]["pose"]["x"] = 0.051
        rows[10].pop("pose")
        result = self.check(rows)
        self.assertTrue(result["incomplete"])
        self.assertIn("position_m exceeded fixed limit", result["failures"])

    def test_malformed_row_is_incomplete(self):
        rows = fixture()
        rows[10].pop("pose")
        self.check(rows, "INCOMPLETE")

    def test_nonfinite_capture_is_saved_and_later_case_runs(self):
        from kinisi_virtual_board import mounting_acceptance as runner

        def capture(session, case, result):
            result.update(anchor_us=1000000, rows=fixture())
            if case["mount"] == "centered":
                result["rows"][0]["pose"]["x"] = float("nan")

        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "new-run"
            args = SimpleNamespace(
                firmware=Path(temporary),
                out=output,
                development=True,
                compiler=None,
                cases=["centered-rest-imu", "left-rest-imu"],
            )
            with (
                patch.object(runner, "pin", return_value="test-source"),
                patch.object(runner, "capture", side_effect=capture),
            ):
                self.assertEqual(runner.run(args), 1)
            summary = json.loads((output / "summary.json").read_text())
            self.assertEqual(summary["counts"]["failed"], 1)
            self.assertEqual(summary["counts"]["passed"], 1)
            self.assertEqual(summary["counts"]["pending"], 0)
            failed = json.loads((output / "centered-rest-imu.json").read_text())
            self.assertEqual(failed["score"]["outcome"], "FAIL")
            self.assertEqual(
                failed["rows"][0]["pose"]["x"], dict(nonfinite_float="nan", authority="raw wire")
            )


if __name__ == "__main__":
    unittest.main()
