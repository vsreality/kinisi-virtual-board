"""Portable runner boundaries; native scenario evidence is produced by the CLI."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch, MagicMock

from kinisi_virtual_board.feedback_selector import runner, packets, persistence


class SelectorRunnerTests(unittest.TestCase):
    def args(self, root, cases):
        return SimpleNamespace(firmware=root / "firmware", out=root / "capture",
                               development=False, compiler="chosen-cc", cases=cases)

    def test_cli_help_exposes_explicit_locations(self):
        result = subprocess.run([sys.executable, "-B", "-m", "kinisi_virtual_board",
                                 "selector-regression", "--help"], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        for option in ("--firmware", "--out", "--compiler", "--case", "--development"):
            self.assertIn(option, result.stdout)

    def test_packaged_v1_images_match_public_release_fixtures(self):
        original = Path(__file__).parent / "fixtures" / "configuration-v1"
        for name, expected in (("factory", persistence.FACTORY_SHA),
                               ("mecanum", persistence.PRESET_SHA)):
            raw = (runner.HERE / "fixtures" / (name + ".fram")).read_bytes()
            self.assertEqual(hashlib.sha256(raw).hexdigest(), expected)
            self.assertEqual(raw, (original / (name + ".fram")).read_bytes())

    def test_failure_preserves_raw_log_and_continues(self):
        def failure(session):
            session.stream.write('{"event":"before_failure"}\n')
            session.stream.flush()
            raise RuntimeError("observed failure")

        class Session:
            def __init__(self, firmware, revision, stream, compiler):
                self.stream = stream
                self.compiler = compiler
            def reboot(self, image):
                assert self.compiler == "chosen-cc"
                assert image
            def close(self):
                pass

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            args = self.args(root, ["factory", "v03"])
            module = SimpleNamespace(factory=failure, unsupported=lambda session: None)
            with patch.object(runner, "pin", return_value="clean-pin"), \
                 patch.object(runner, "Session", Session), \
                 patch.object(runner.importlib, "import_module", return_value=module):
                self.assertEqual(runner.run(args), 1)
            result = json.loads((args.out / "summary.json").read_text())
            self.assertEqual(result["cases"]["factory"]["result"], "FAIL_OR_INCOMPLETE")
            self.assertIn("observed failure", result["cases"]["factory"]["error"])
            self.assertEqual(result["cases"]["v03"]["result"], "PASS")
            raw = (args.out / "factory.jsonl").read_bytes()
            self.assertIn(b"before_failure", raw)
            self.assertEqual(result["cases"]["factory"]["sha256"], hashlib.sha256(raw).hexdigest())
            self.assertEqual(result["packet_export"]["result"], "NOT_RUN")

    def test_existing_output_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            args = self.args(Path(directory), ["factory"])
            args.out.mkdir()
            (args.out / "summary.json").write_text("retained")
            with patch.object(runner, "pin", return_value="clean-pin"), \
                 self.assertRaises(FileExistsError):
                runner.run(args)
            self.assertEqual((args.out / "summary.json").read_text(), "retained")

    def test_repeated_case_rejected_before_output_creation(self):
        with tempfile.TemporaryDirectory() as directory:
            args = self.args(Path(directory), ["factory", "factory"])
            with patch.object(runner, "pin", return_value="clean-pin"), \
                 self.assertRaisesRegex(ValueError, "only once"):
                runner.run(args)
            self.assertFalse(args.out.exists())

    def test_packet_export_rejects_changed_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "summary.json").write_text(json.dumps({"cases": {
                c: {"result": "PASS", "sha256": "wrong"} for c in ("encoder", "imu", "fused")}}))
            (root / "encoder.jsonl").write_text('{"event":"tampered"}\n')
            with patch.object(packets, "Schema", return_value=MagicMock()), self.assertRaises(AssertionError):
                packets.export(root, root)
            self.assertFalse((root / "sdk-records.json").exists())

    def test_only_new_output_is_excluded_from_source_identity(self):
        with tempfile.TemporaryDirectory(prefix=".selector-test-", dir=runner.SIM) as directory:
            root = Path(directory)
            output = root / "results"
            before = runner.pin(runner.SIM, development=True, output=output)
            output.mkdir()
            (output / "raw.jsonl").write_text("retained packet")
            self.assertEqual(before, runner.pin(runner.SIM, development=True, output=output))
            (root / "other-source.py").write_text("changed source")
            self.assertNotEqual(before, runner.pin(runner.SIM, development=True, output=output))


if __name__ == "__main__":
    unittest.main()
