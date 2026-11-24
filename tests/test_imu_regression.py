"""Synthetic external-comparator checks; no firmware, private data or release imports."""
import math
import struct
import unittest
from types import SimpleNamespace

from kinisi_virtual_board import imu_regression_score as score


def capture(profile="translate", delay=10_000):
    restart = profile in ("stationary", "stale")
    metadata = dict(profile=profile, seed=1, delay_us=delay, start_us=600_000,
                    restart_us=3_100_000 if restart else None, end_us=5_120_000,
                    poll_us=1000, start_command_us=600_000,
                    restart_command_us=3_100_000 if restart else None)
    reference = [dict(at_us=t, x=0., y=0., yaw=0., vx=0., vy=0., wz=0.)
                 for t in range(580_000, 5_120_001, 2500)]
    sensors = []
    a, g = -1, -1
    for k in range(451):
        for die in (0, 1):
            index = 79 if profile == "stale" and 80 <= k <= 109 else k
            if die == 0: a = index
            else: g = index
            sensors.append(dict(delivered_us=600_000+k*10_000+die*2500+delay,
                                accel_us=600_000+a*10_000, gyro_us=602_500+g*10_000,
                                ax=float(a), ay=0., az=9.8, gx=0., gy=0., gz=float(g)))
    rows = []
    for p in range(600_000, 5_120_001, 1000):
        second = restart and p >= 3_100_000
        action, base_sequence = (3_100_000, 200) if second else (600_000, 0)
        anchor = action+512_500
        first = ((anchor+7500+delay+999)//1000)*1000+2000
        expiry = ((anchor+1_025_000+999)//1000)*1000
        fault_at = 1_420_000+delay
        row = dict(public_us=p, state=1, fault=0, flags=8, sequence=base_sequence,
                   timestamp_us=0, elapsed_us=0, accel_age_us=0xffffffff, gyro_age_us=0xffffffff,
                   pose_error=10, twist_error=10, motor_pwm_max=0)
        row.update({key: None for key in score.OPTIONAL})
        if p >= first:
            slot = min(99, (p-first)//10_000)
            if profile == "stale" and not second:
                slot = min(slot, (1_382_500-anchor)//10_000)
            source, sequence = anchor+slot*10_000, base_sequence+slot+1
            row.update(state=2, flags=11, sequence=sequence, timestamp_us=source,
                       elapsed_us=min(p-anchor, 1_000_000), accel_age_us=p-source-7500,
                       gyro_age_us=p-source, consumed_accel_us=source+7500, consumed_gyro_us=source,
                       pose_sequence=sequence, twist_sequence=sequence, pose_error=0, twist_error=0)
            row.update({key: 0. for key in score.VECTORS})
        fault = profile == "stale" and not second and p >= fault_at
        if fault or p >= expiry:
            row.update(state=3 if fault else 4, fault=3 if fault else 7, flags=12,
                       elapsed_us=fault_at-anchor if fault else 1_000_000,
                       sequence=base_sequence+100, pose_error=10, twist_error=10)
            row.update({key: None for key in (*score.VECTORS, "pose_sequence", "twist_sequence")})
        rows.append(row)
    return dict(metadata=metadata, sensor_rows=sensors, reference_rows=reference, public_rows=rows)


def kinds(result):
    return {key.split(".", 1)[0] for key in result["failure_counts"]}


class IMURegressionTests(unittest.TestCase):
    def test_complete_profiles_all_delivery_phases(self):
        for profile in score.PROFILES:
            for phase in (10_000, 13_000, 17_000):
                with self.subTest(profile=profile, phase=phase):
                    result = score.score_case(capture(profile, phase))
                    self.assertEqual(result["outcome"], "PASS", result["failures"][:2])
                    self.assertEqual(result["counts"]["polls"], 4521)
                    self.assertEqual(result["counts"]["reference_rows"], 1817)
                    self.assertEqual(result["counts"]["sensor_rows"], 902)
                    self.assertEqual(result["counts"]["samples"], 200 if profile == "stationary" else 128 if profile == "stale" else 100)

    def test_true_limits_are_inclusive_and_peaks_measured(self):
        for field, metric in (("x", "position_m"), ("body_vx", "body_speed_m_s"),
                              ("yaw", "yaw_rad"), ("wz", "yaw_rate_rad_s")):
            for factor, outcome in ((1., "PASS"), (1.001, "FAIL")):
                sample = capture()
                for row in sample["public_rows"]:
                    if row["state"] == 2: row[field] = score.LIMITS[metric]*factor
                result = score.score_case(sample)
                self.assertEqual(result["outcome"], outcome)
                self.assertAlmostEqual(result["peaks"][metric]["value"], score.LIMITS[metric]*factor)
                self.assertEqual(result["peaks"][metric]["source_us"], 1_112_500)

    def test_reference_rebase_rotation_and_body_velocity(self):
        sample, anchor = capture(), 1_112_500
        for row in sample["reference_rows"]:
            t = max(row["at_us"]-anchor, 0)/1e6
            row.update(x=10., y=20.+.1*t, yaw=math.pi/2, vx=.1 if t else 0.)
        for row in sample["public_rows"]:
            if row["state"] == 2:
                t = (row["timestamp_us"]-anchor)/1e6
                row.update(x=.1*t, body_vx=.1 if t else 0.)
        result = score.score_case(sample)
        self.assertEqual(result["outcome"], "PASS", result["failures"][:2])
        self.assertLess(result["peaks"]["position_m"]["value"], 1e-12)

    def test_missing_poll_reference_or_sensor_never_passes(self):
        for field, index in (("public_rows", 700), ("reference_rows", 800), ("sensor_rows", 800)):
            sample = capture()
            sample[field].pop(index)
            result = score.score_case(sample)
            self.assertEqual(result["outcome"], "INCOMPLETE")
            self.assertTrue(result["incomplete"])
        sample = capture()
        sample["reference_rows"] = [r for r in sample["reference_rows"] if r["at_us"] != 1_312_500]
        result = score.score_case(sample)
        self.assertEqual(result["outcome"], "INCOMPLETE")
        self.assertEqual(result["counts"]["samples"], 99)

    def test_skipped_source_with_complete_polls_is_product_failure(self):
        sample = capture()
        for row in sample["public_rows"]:
            if row["state"] == 2 and row["timestamp_us"] == 1_312_500:
                row["timestamp_us"] -= 10_000
                row["consumed_accel_us"] -= 10_000
                row["consumed_gyro_us"] -= 10_000
                row["accel_age_us"] += 10_000
                row["gyro_age_us"] += 10_000
                row["sequence"] -= 1
                row["pose_sequence"] -= 1
                row["twist_sequence"] -= 1
        result = score.score_case(sample)
        self.assertIn("missing_source_slot", kinds(result))
        self.assertIn(1_312_500, result["epochs"][0]["missing_source_examples"])

    def test_truncated_prefix_does_not_invent_future_failures(self):
        sample = capture()
        sample["public_rows"] = [r for r in sample["public_rows"] if r["public_us"] <= 1_350_000]
        result = score.score_case(sample)
        self.assertEqual(result["outcome"], "INCOMPLETE")
        self.assertFalse(result["failures"])

    def test_gap_hiding_anchor_skips_relative_errors_not_absolute_errors(self):
        sample = capture()
        sample["public_rows"] = [r for r in sample["public_rows"] if not 1_100_000 <= r["public_us"] <= 1_500_000]
        result = score.score_case(sample)
        self.assertEqual(result["outcome"], "INCOMPLETE")
        self.assertFalse(result["epochs"][0]["anchor_verified"])
        self.assertNotIn("position_m", result["peaks"])
        for row in sample["public_rows"]:
            if row["state"] == 2: row["body_vx"] = .03
        result = score.score_case(sample)
        self.assertEqual(result["outcome"], "FAIL")
        self.assertTrue(result["incomplete"])
        self.assertGreater(result["peaks"]["body_speed_m_s"]["value"], .02)

    def test_lifecycle_expiry_and_fault_latches(self):
        sample = capture()
        row = sample["public_rows"][3000]
        row.update(state=1, fault=0, flags=8, elapsed_us=0)
        self.assertIn("terminal_not_latched", kinds(score.score_case(sample)))
        sample = capture("stale")
        for row in sample["public_rows"]:
            if row["state"] == 3: row["fault"] = 4
        self.assertIn("fault_deadline_or_code", kinds(score.score_case(sample)))
        sample = capture()
        for row in sample["public_rows"]:
            if row["state"] == 4: row.update(state=1, fault=0, flags=8, elapsed_us=0)
        self.assertIn("expiry_deadline", kinds(score.score_case(sample)))

    def test_restart_and_command_confirmation(self):
        sample = capture("stationary")
        row = next(r for r in sample["public_rows"] if r["public_us"] > 3_100_000 and r["state"] == 2)
        row["sequence"] = row["pose_sequence"] = row["twist_sequence"] = 1
        self.assertIn("restart_reused_sequence", kinds(score.score_case(sample)))
        sample = capture()
        sample["metadata"]["start_command_us"] = None
        sample["public_rows"] = []
        result = score.score_case(sample)
        self.assertEqual(result["outcome"], "INCOMPLETE")
        self.assertFalse(result["failures"])

    def test_causality_age_and_cached_identity(self):
        sample = capture()
        row = sample["public_rows"][700]
        row["consumed_gyro_us"] += 10_000
        self.assertIn("source_pair_identity", kinds(score.score_case(sample)))
        sample = capture()
        sample["public_rows"][701]["x"] = .001
        self.assertIn("changed_cached_snapshot", kinds(score.score_case(sample)))
        sample = capture()
        row = next(r for r in sample["public_rows"] if r["state"] == 2)
        row["timestamp_us"] -= 40_000
        self.assertIn("first_source_age", kinds(score.score_case(sample)))

    def test_malformed_and_incomplete_evidence_preserves_pwm_violation(self):
        sample = capture()
        sample["public_rows"][700]["motor_pwm_max"] = .1
        sample["public_rows"][800]["flags"] = "bad"
        sample["reference_rows"].pop(600)
        result = score.score_case(sample)
        self.assertEqual(result["outcome"], "FAIL")
        self.assertTrue(result["incomplete"])
        self.assertIn("diagnostic_motor_output", kinds(result))

    def test_bounded_examples_retain_full_counts_and_capture_error(self):
        sample = capture()
        for row in sample["public_rows"]: row["motor_pwm_max"] = 1
        sample["metadata"]["error"] = "INIT unexpectedly emitted duplicate reply"
        result = score.score_case(sample)
        self.assertEqual(result["outcome"], "FAIL")
        self.assertEqual(result["failure_counts"]["diagnostic_motor_output"], 4521)
        self.assertEqual(result["counts"]["failures"], 4521)
        self.assertLessEqual(len(result["failures"]), score.MAX_EXAMPLES)
        self.assertTrue(any("INIT unexpectedly" in issue for issue in result["incomplete"]))
        self.assertGreater(result["counts"]["incomplete"], 0)

    def test_fault_elapsed_is_bounded_at_entry_and_then_frozen(self):
        sample = capture("stale")
        for row in sample["public_rows"]:
            if row["state"] == 3: row["elapsed_us"] = row["public_us"] % 100_000
        self.assertIn("fault_elapsed_latch", kinds(score.score_case(sample)))
        sample = capture("stale")
        for row in sample["public_rows"]:
            if row["state"] == 3: row["elapsed_us"] = 0
        self.assertIn("fault_elapsed_entry", kinds(score.score_case(sample)))

    def test_wire_accepts_init_with_same_request_ready_notification(self):
        from kinisi_virtual_board.imu_regression_capture import Wire
        codes = {"INIT": 1, "READY": 2, "ERROR": 255}
        schema = SimpleNamespace(commands={"INIT": {"properties": []}}, code=codes.__getitem__,
                                 request=lambda command, sequence: bytes([sequence]))
        payload = b"identity"
        def frame(code, body=b""):
            return bytes([len(body)+3])+struct.pack("<BH", code, 1)+body
        board = SimpleNamespace(schema=schema, feed=lambda request: None,
                                drain=lambda: frame(1, payload)+frame(2))
        self.assertEqual(Wire(board).ask("INIT"), payload)
        board.drain = lambda: frame(1, payload)+frame(1, payload)
        with self.assertRaises(ValueError): Wire(board).ask("INIT")

    def test_unknown_anchor_preserves_later_proven_missing_slot(self):
        sample = capture()
        sample["public_rows"] = [r for r in sample["public_rows"] if not 1_100_000 <= r["public_us"] <= 1_300_000]
        for row in sample["public_rows"]:
            if row["state"] == 2 and row["timestamp_us"] == 1_512_500:
                row["timestamp_us"] -= 10_000
                row["consumed_accel_us"] -= 10_000
                row["consumed_gyro_us"] -= 10_000
                row["accel_age_us"] += 10_000
                row["gyro_age_us"] += 10_000
                row["sequence"] -= 1
                row["pose_sequence"] -= 1
                row["twist_sequence"] -= 1
        result = score.score_case(sample)
        self.assertEqual(result["outcome"], "FAIL")
        self.assertTrue(result["incomplete"])
        self.assertFalse(result["epochs"][0]["anchor_verified"])
        self.assertIn("missing_source_slot", kinds(result))
        self.assertEqual(score.score_case({})["outcome"], "INCOMPLETE")


if __name__ == "__main__":
    unittest.main()
