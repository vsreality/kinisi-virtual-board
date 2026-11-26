"""Synthetic qualification coverage, binding and fault checks; no native runs."""
import copy
import math
import unittest

from kinisi_virtual_board import imu_qualification_score as q
from kinisi_virtual_board import imu_regression_score as base
from kinisi_virtual_board.imu_qualification_plan import make_plan
from test_imu_regression import capture


def fixture(role="stationary", delay=10000):
    sample = capture("stale" if role in q.FAULTS else "stationary", delay)
    nominal = capture("stationary")["sensor_rows"]
    sample["nominal_sensor_rows"] = nominal
    sample["metadata"]["profile"] = role
    sample["targets"] = [dict(at_us=600000, x=0., y=0., yaw=0.)]
    case = dict(case="toy", preset="differential", role=role, seed=1, delay_us=delay,
                targets_ms=[[0, [0., 0., 0.]]], direction_body_xy=None,
                source_id="differential-stationary-00000001", control_case="control",
                **{"class": "fault" if role in q.FAULTS else "healthy"})
    if role in ("accel_freeze", "gyro_reorder"):
        actual = []
        for index, original in enumerate(nominal):
            row = dict(original, delivered_us=original["delivered_us"]+delay-10000)
            k, die = divmod(index, 2)
            if role == "accel_freeze" and 80 <= k <= 109:
                row.update({key: nominal[158][key] for key in ("accel_us", "ax", "ay", "az")})
            if role == "gyro_reorder" and (index == 161 or index == 162):
                row.update({key: nominal[153][key] for key in ("gyro_us", "gx", "gy", "gz")})
            actual.append(row)
        sample["sensor_rows"] = actual
    if role == "gyro_reorder":
        # Its g79 healthy publication is eligible before the g80 anomaly.
        for row in sample["public_rows"]:
            if row["state"] == 2 and row["timestamp_us"] == 1_382_500 and row["public_us"] >= 1_402_000+delay:
                row.update(timestamp_us=1_392_500, consumed_accel_us=1_400_000,
                           consumed_gyro_us=1_392_500, accel_age_us=row["public_us"]-1_400_000,
                           gyro_age_us=row["public_us"]-1_392_500, sequence=29, pose_sequence=29, twist_sequence=29)
            if row["state"] == 3: row["fault"] = 4
    return sample, case


def motion_rows(role, anchor=1000000):
    rows, x, yaw = [], 0., 0.
    for i in range(100):
        vx = 0. if role == "stationary" or (role == "accelerate" and i < 5) or (role == "brake" and i >= 30) else .1
        if role == "reverse" and i >= 35: vx = -.1
        wz = .3 if role == "turn" else 0.
        rows.append(dict(at_us=anchor+i*10000, x=x, y=0., yaw=yaw, vx=vx, vy=0., wz=wz))
        x += vx*.01
        yaw += wz*.01
    return rows


class QualificationScoreTests(unittest.TestCase):
    def test_stationary_and_all_faults_all_phases(self):
        for role in ("stationary", *q.FAULTS):
            for delay in q.PHASES:
                with self.subTest(role=role, delay=delay):
                    sample, case = fixture(role, delay)
                    result = q.score_qualification(sample, case)
                    self.assertEqual(result["outcome"], "PASS", (result["failure_counts"], result["incomplete"]))
                    self.assertTrue(result["coverage"]["passed"])
                    self.assertEqual(len(result["coverage"]["planned"]), 8)
                    self.assertEqual(result["epochs"][1]["observed_samples"], 100)
                    if role in q.FAULTS:
                        self.assertEqual(result["epochs"][0]["observed_samples"], 29 if role == "gyro_reorder" else 28)
                        self.assertEqual(result["epochs"][0]["expected_fault"], 4 if role == "gyro_reorder" else 3)

    def test_each_measured_motion_event(self):
        anchor = 1000000
        targets = [dict(at_us=anchor-100000, x=.1, y=0, yaw=0), dict(at_us=anchor+300000, x=0, y=0, yaw=0)]
        for role in q.ROLES:
            with self.subTest(role=role):
                rows = motion_rows(role, anchor)
                result = q.coverage(rows, targets, role, [1., 0.], anchor)
                self.assertTrue(result["passed"], result)

    def test_turn_requires_simultaneous_translation(self):
        rows = motion_rows("turn")
        for i, row in enumerate(rows):
            row.update(vx=0. if i < 50 else .1, wz=.3 if i < 50 else 0., yaw=min(i, 50)*.003)
        result = q.coverage(rows, [], "turn", [1., 0.], 1000000)
        self.assertGreaterEqual(result["path_m"], .02)
        self.assertGreaterEqual(result["yaw_excursion_rad"], .10)
        self.assertFalse(result["passed"])

    def test_brake_requires_executed_stop_after_forward_before_rest(self):
        rows = motion_rows("brake")
        targets = [dict(at_us=900000, x=.1, y=0, yaw=0), dict(at_us=1000000, x=0, y=0, yaw=0)]
        self.assertFalse(q.coverage(rows, targets, "brake", [1., 0.], 1000000)["passed"])
        targets[1]["at_us"] = 1990001
        self.assertFalse(q.coverage(rows, targets, "brake", [1., 0.], 1000000)["passed"])
        targets[1]["at_us"] = 1300000
        self.assertTrue(q.coverage(rows, targets, "brake", [1., 0.], 1000000)["passed"])

    def test_coverage_uses_all_exact_slots_and_direction(self):
        rows = motion_rows("reverse")
        self.assertFalse(q.coverage(rows, [], "reverse", [-1., 0.], 1000000)["passed"])
        rows.pop(10)
        self.assertFalse(q.coverage(rows, [], "reverse", [1., 0.], 1000000)["passed"])
        rows = motion_rows("accelerate")
        rows[0]["vx"] = .0121
        self.assertFalse(q.coverage(rows, [], "accelerate", [1., 0.], 1000000)["passed"])
        rows = motion_rows("stationary")
        rows[-1]["yaw"] = .0051
        self.assertFalse(q.coverage(rows, [], "stationary", None, 1000000)["passed"])

    def test_fault_boundary_held_payload_and_unaffected_channel_are_bound(self):
        for role, index, key in (("producer_freeze", 220, "gz"), ("accel_freeze", 161, "gx"), ("gyro_reorder", 162, "gz")):
            sample, case = fixture(role)
            sample["sensor_rows"][index][key] += .01
            result = q.score_qualification(sample, case)
            self.assertNotEqual(result["outcome"], "PASS")
            self.assertGreater(result["counts"]["incomplete"], 0)

    def test_actual_targets_and_source_error_prevent_pass(self):
        sample, case = fixture()
        sample["targets"][0]["at_us"] += 1
        self.assertEqual(q.score_qualification(sample, case)["outcome"], "INCOMPLETE")
        sample, case = fixture()
        sample["metadata"]["source_error"] = "motion configuration rejected"
        result = q.score_qualification(sample, case)
        self.assertEqual(result["outcome"], "INCOMPLETE")
        self.assertTrue(any("configuration rejected" in s for s in result["incomplete"]))

    def test_age_and_cached_timing_are_reported_without_cache_age_gate(self):
        sample, case = fixture("stationary", 17000)
        result = q.score_qualification(sample, case)
        timing = result["epochs"][0]["timing"]
        self.assertLessEqual(timing["first_applied_accel_age_us"], 35000)
        self.assertGreater(timing["cached_applied_accel_age_us"], 35000)
        self.assertEqual(result["outcome"], "PASS")

    def test_aggregate_requires_exact_matrix_order_and_reference_pairing(self):
        plan = make_plan([1, 2, 3])
        results = []
        for case in plan:
            results.append(dict(case=case["case"], preset=case["preset"], role=case["role"],
                                category=case["class"], delay_us=case["delay_us"], outcome="PASS",
                                counts=dict(incomplete=0, samples=100), peaks={}, binding={"nominal": case["source_id"]}))
        result = q.summarize_qualification(results, plan)
        self.assertEqual(result["outcome"], "PASS", result["pairing_issues"])
        self.assertEqual(result["healthy"]["cases"], 135)
        self.assertEqual(result["fault"]["cases"], 9)
        self.assertTrue(all(group["cases"] == 48 for group in result["presets"].values()))
        self.assertEqual(q.summarize_qualification(results[:-1], plan)["outcome"], "INCOMPLETE")
        reordered = list(reversed(results))
        self.assertEqual(q.summarize_qualification(reordered, plan)["outcome"], "INCOMPLETE")
        phase = next(r for r in results if r["delay_us"] == 13000 and r["category"] == "healthy")
        phase["binding"] = {"nominal": "substituted"}
        self.assertEqual(q.summarize_qualification(results, plan)["outcome"], "INCOMPLETE")
        phase["outcome"] = "FAIL"
        self.assertEqual(q.summarize_qualification(results, plan)["outcome"], "FAIL")


if __name__ == "__main__":
    unittest.main()
