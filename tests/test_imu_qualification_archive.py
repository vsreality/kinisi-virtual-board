"""Exercise the public archive and replay without running a qualification."""
import argparse
import contextlib
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from kinisi_virtual_board import imu_qualification as run
from kinisi_virtual_board.imu_qualification_plan import make_plan


class QualificationArchiveTests(unittest.TestCase):
    def test_source_replay_checks_saved_bytes(self):
        case = make_plan([1, 2, 3])[0]
        producer = {"build_id": "test"}
        source = dict(metadata=dict(producer, source_id=case["source_id"]), targets=[],
                      sensor_rows=[dict.fromkeys(run.IMU_FIELDS, 0)],
                      reference_rows=[dict.fromkeys(run.REFERENCE_FIELDS, 0)])
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            folder = root / "sources" / case["source_id"]
            folder.parent.mkdir()
            with patch("kinisi_virtual_board.imu_qualification_motion.source_for_case", return_value=source):
                run.create_source(folder, None, root, producer, case, None)
            with patch("kinisi_virtual_board.imu_qualification_motion.source_for_case") as acquire:
                receipt = run.digest(folder / "source.json")
                replay = run.create_source(root / "copy", root, None, producer, case, None, receipt)
                self.assertEqual(replay, source)
                (folder / "targets.json").write_text("[1]")
                with self.assertRaisesRegex(ValueError, "changed"):
                    run.create_source(root / "changed", root, None, producer, case, None, receipt)
                acquire.assert_not_called()

    def test_publishes_plan_before_source_and_retains_all_failures(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            dest = root / "first"
            def unavailable(*args, **kwargs):
                self.assertEqual(len(run.read_json(dest / "plan.json")["cases"]), 144)
                raise OSError("injected motion source unavailable")
            args = argparse.Namespace(firmware=root, compiler=None, motion_firmware=None, replay=None, out=dest)
            with patch.object(run, "firmware_identity", return_value={"build_id": "toy"}), \
                 patch.object(run.secrets, "SystemRandom") as rng, \
                 patch("kinisi_virtual_board.imu_qualification_motion.source_for_case", side_effect=unavailable) as acquire, \
                 contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                rng.return_value.sample.return_value = [1, 2, 3]
                self.assertEqual(run.execute(args), 2)
                self.assertEqual(acquire.call_count, 45)
                report = run.read_json(dest / "report.json")
                self.assertEqual(report["counts"], dict(planned=144, attempted=144, pending=0,
                                                        passed=0, product_failed=0, incomplete=144))
                self.assertIn("injected motion source unavailable", report["cases"][0]["reason"])
                with self.assertRaises(FileExistsError):
                    run.execute(args)
