"""Development-only sensor delivery, export alignment and scoring regressions."""
import csv
import json
from pathlib import Path
import tempfile
import unittest

from tools import encoder_baseline as baseline, imu_development as export, score_encoder_baseline as scorer


def write(path, header, rows):
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(header.split())
        writer.writerows(rows)


class DevelopmentTraceTests(unittest.TestCase):
    def test_independent_delivery_does_not_refresh_or_anticipate_samples(self):
        sensor = baseline.DelayedIMU(131)
        observed = {}
        for t in range(2500, 50001, 2500):
            sensor.advance(t, (0., 0., 9.80665), (0., 0., .2))
            observed[t] = tuple(sensor.stamps), tuple(sensor.values)
        self.assertEqual(observed[17500][0], (0, 0))
        self.assertEqual(observed[20000][0], (10000, 0))
        self.assertEqual(observed[22500][0], (10000, 12500))
        self.assertEqual(observed[27500], observed[22500])
        self.assertEqual(observed[50000][0], (40000, 32500))
        self.assertNotEqual(observed[20000][1][:3], observed[30000][1][:3])

    def fixture(self, root):
        raw = root/"raw"
        raw.mkdir()
        model = dict(id="bmi088-v04-1", integration_us=2500, transport_delay_us=10000,
                     source_period_us=10000, accel_phase_us=0, gyro_phase_us=2500)
        case = dict(origin_us=600000, reference_start_us=580000, imu_model=model,
                    duration_ms=50, preset="differential", mode="velocity", firmware_build_id="fixture")
        (raw/"case.json").write_text(json.dumps(case))
        write(raw/"sensors.csv", scorer.HEADERS["sensors.csv"],
              [(t,t-10000,t-17500,0,0,0,0,0,0,9.8,0,0,0) for t in range(600000,650001,10000)])
        write(raw/"reference.csv", scorer.HEADERS["reference.csv"],
              [(t,0,0,0,0,0,0) for t in range(580000,650001,2500)])
        write(raw/"firmware.csv", scorer.HEADERS["firmware.csv"],
              [(t,t,0,1,0,0,0,0,0,0,0,0) for t in range(600000,650001,10000)])
        write(raw/"targets.csv", scorer.HEADERS["targets.csv"], [(600000,0,0,0),(610000,.2,0,0)])
        for name in ("imu", "reference", "targets"):
            (root/name).mkdir()
        return raw

    def test_export_keeps_independent_source_references_and_final_delivery(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            raw = self.fixture(root)
            result = export.export_recording(raw, root, "stationary-test", "stationary", "differential", 40)
            with (root/result["imu_file"]).open() as stream:
                imu = list(csv.DictReader(stream))
            with (root/result["reference_path"]).open() as stream:
                reference = {int(r["at_us"]): r for r in csv.DictReader(stream)}
            self.assertEqual(len(imu), 5)
            self.assertEqual(int(imu[0]["accel_us"]), 600000)
            self.assertEqual(int(imu[0]["gyro_us"]), 592500)
            self.assertEqual(max(reference), 650000)
            for row in imu:
                for field in ("accel_us", "gyro_us", "delivered_us"):
                    self.assertIn(int(row[field]), reference)
            self.assertEqual(tuple(imu[0]), ("delivered_us","accel_us","gyro_us","ax","ay","az","gx","gy","gz"))

    def test_scorer_uses_dense_reference_without_zipping_wrong_times(self):
        with tempfile.TemporaryDirectory() as directory:
            raw = self.fixture(Path(directory))
            spec = dict(step_ms=10, courses={"velocity":dict(targets=[(0,(0,0,0)),(10,(.2,0,0))])})
            result = scorer.score_case(raw, spec)
            self.assertEqual(result["est_position_rmse_m"], 0)
            self.assertGreater(result["tracking_position_rmse_m"], 0)
            self.assertEqual(result["accel_delivery_age_max_ms"], 10)
            self.assertEqual(result["gyro_delivery_age_max_ms"], 17.5)
            rows = (raw/"sensors.csv").read_text()
            (raw/"sensors.csv").write_text(rows.replace("582500", "590000", 1))
            with self.assertRaisesRegex(ValueError, "provenance"):
                scorer.score_case(raw, spec)
