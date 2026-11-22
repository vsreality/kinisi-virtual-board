import csv
import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from contextlib import redirect_stdout
from io import StringIO
from unittest.mock import patch

from tools import score_encoder_baseline as scorer


def write_rows(path, header, rows):
    with path.open("w", newline="", encoding="utf-8") as f:
        output = csv.writer(f)
        output.writerow(header.split())
        output.writerows(rows)


class BaselineScoreTests(unittest.TestCase):
    def test_malformed_run_retains_other_runs_and_unresponsive_tracking(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            spec = {"presets": ["differential"], "courses": {"velocity": {"targets":
                    [[0, [0, 0, 0]], [10, [.2, 0, 0]]]}}, "surfaces": ["nominal"],
                    "seeds": [1, 2, 3, 4], "step_ms": 10}
            (root/"cases.json").write_text(json.dumps(spec))
            digest = hashlib.sha256((root/"cases.json").read_bytes()).hexdigest()
            (root/"manifest.json").write_text(json.dumps({"cases_sha256": digest}))
            (root/"firmware-build.json").write_text('{"build_id": "fixture"}')
            for seed in spec["seeds"]:
                folder = root/f"differential-velocity-nominal-{seed}"
                folder.mkdir()
                case = dict(preset="differential", mode="velocity", surface="nominal", seed=seed,
                            origin_us=10000, duration_ms=40, firmware_build_id="fixture")
                (folder/"case.json").write_text(json.dumps(case))
                times = range(10000, 60000, 10000)
                write_rows(folder/"sensors.csv", scorer.HEADERS["sensors.csv"],
                           [(t,t-10000,0,0,0,0,0,0,9.8,0,0,0) for t in times])
                write_rows(folder/"targets.csv", "at_us x y yaw", [(10000,0,0,0), (20000,.2,0,0)])
                write_rows(folder/"reference.csv", "at_us x y yaw vx vy wz", [(t,0,0,0,0,0,0) for t in times])
                write_rows(folder/"firmware.csv", "at_us odom_us clock_mode clock_quality x y yaw "
                           "controller_dispatches pwm0 pwm1 pwm2 pwm3",
                           [(t,t,0,1,0,0,0,0,0,0,0,0) for t in times])
                if seed == 1:
                    shutil.copytree(folder, root/(folder.name+"-repeat"))
                elif seed == 4:
                    (folder/"reference.csv").write_text('"at_us')
                else:
                    with (folder/"reference.csv").open("a") as f:
                        f.write("1,2\n" if seed == 2 else "1,2,3,4,5,6,7,8\n")
            with patch("sys.argv", ["score", str(root)]), redirect_stdout(StringIO()):
                self.assertEqual(scorer.main(), 1)
            summary = json.loads((root/"summary.json").read_text())
            self.assertEqual(summary["valid"], 1)
            self.assertEqual(len(summary["invalid"]), 3)
            self.assertTrue(all("column count" in r["error"] for r in summary["invalid"][:2]))
            self.assertIn("reference.csv, line", summary["invalid"][2]["error"])
            self.assertTrue(summary["repeat_identical"])
            metrics = summary["groups"][0]["metrics"]
            self.assertGreater(metrics["tracking_linear_velocity_rmse_m_s"]["mean"], 0.)
            self.assertEqual(metrics["pwm_response_ms"]["n_measured"], 0)
