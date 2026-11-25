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
from kinisi_virtual_board import mounting_acceptance as runner


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
                    result = score(
                        fixture(profile, mode),
                        profile=profile,
                        mode=mode,
                        anchor_us=1000000,
                    )
                    self.assertEqual(result["outcome"], "PASS", result)

    def test_unwrapped_fast_plateau(self):
        rows = fixture("fast_steady_yaw", "fused", 2050000)
        result = score(
            rows,
            profile="fast_steady_yaw",
            mode="fused",
            anchor_us=1000000,
            window_offset_us=2050000,
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
        result = score(
            [],
            profile="rest",
            mode="imu",
            anchor_us=None,
            product_error="Mount invalid",
        )
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
                failed["rows"][0]["pose"]["x"],
                dict(nonfinite_float="nan", authority="raw wire"),
            )


class BoardCenterRunnerTests(unittest.TestCase):
    @staticmethod
    def sharing_error(code=32):
        error = PermissionError("simulated Windows sharing denial")
        error.winerror = code
        return error

    def test_atomic_summary_replace_retries_transient_windows_reader(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "summary.json"
            target.write_text("old")
            original = Path.replace
            calls = []

            def replace(source, destination):
                calls.append((source, destination))
                if len(calls) == 1:
                    raise self.sharing_error()
                return original(source, destination)

            with (
                patch.object(runner, "os", SimpleNamespace(name="nt")),
                patch.object(Path, "replace", replace),
                patch.object(runner.time, "sleep") as sleep,
            ):
                runner._dump(target, dict(outcome="PASS"))
            self.assertEqual(json.loads(target.read_text()), dict(outcome="PASS"))
            self.assertEqual(len(calls), 2)
            sleep.assert_called_once_with(0.025)

    def test_atomic_summary_replace_exhaustion_preserves_error(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "summary.json"
            target.write_text("old")
            error = self.sharing_error(5)
            with (
                patch.object(runner, "os", SimpleNamespace(name="nt")),
                patch.object(Path, "replace", side_effect=error) as replace,
                patch.object(runner.time, "sleep") as sleep,
            ):
                with self.assertRaises(PermissionError) as caught:
                    runner._dump(target, dict(outcome="PASS"))
            self.assertIs(caught.exception, error)
            self.assertEqual(replace.call_count, 5)
            self.assertEqual(sleep.call_count, 4)
            self.assertEqual(target.read_text(), "old")
            self.assertTrue(target.with_name("summary.json.tmp").exists())

    def test_atomic_replace_does_not_retry_unrelated_or_initial_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "summary.json"
            for existing, platform, code in ((False, "nt", 5), (True, "nt", 87), (True, "posix", 5)):
                if existing:
                    target.write_text("old")
                with (
                    self.subTest(existing=existing, platform=platform, code=code),
                    patch.object(runner, "os", SimpleNamespace(name=platform)),
                    patch.object(Path, "replace", side_effect=self.sharing_error(code)) as replace,
                    patch.object(runner.time, "sleep") as sleep,
                ):
                    with self.assertRaises(PermissionError):
                        runner._dump(target, {})
                    self.assertEqual(replace.call_count, 1)
                    sleep.assert_not_called()

    def test_default_chip_plan_is_unchanged(self):
        original = runner.plan()
        self.assertEqual(original, runner.plan("chip"))
        self.assertEqual(len(original["cases"]), 74)
        self.assertEqual(original["mounts"]["right"]["offset_m"], (0, -0.1, 0))
        self.assertEqual(original["mounts"]["right"]["rpy_degrees"], (0, 0, 0))

    def test_board_center_plan_declares_vertical_right_and_fixed_pivot(self):
        board = runner.plan("board-center")
        self.assertEqual(len(board["cases"]), 76)
        self.assertEqual(board["limits"], runner.plan()["limits"])
        self.assertEqual(board["mounts"]["right"]["rpy_degrees"], (90, 0, 0))
        self.assertEqual(board["mounts"]["bottom_reorientation"]["offset_m"], (0, 0, 0))
        self.assertEqual(
            {c["mode"] for c in board["cases"] if c["mount"] == "bottom_reorientation"},
            {"imu", "fused"},
        )
        with self.assertRaises(ValueError):
            runner.plan("unknown")

    def test_profile_checked_against_independent_geometry(self):
        # Separate explicit layout/package calculation, not the sensor helper.
        expected = (
            round((148.14 - 163.512563) * 1000),
            round((88.512563 - 61.56) * 1000),
            2075,
        )
        self.assertEqual(expected, (-15373, 26953, 2075))
        with tempfile.TemporaryDirectory() as temporary:
            base = Path(temporary)
            (base / "boards").mkdir()
            profile = base / "boards" / "v0.4.json"
            profile.write_text(json.dumps(dict(imu_geometry=dict(underside_center_to_imu_um=list(expected)))))
            self.assertEqual(runner._profile_vector(base), tuple(v / 1e6 for v in expected))
            for wrong in (
                (15373, 26953, 2075),
                (-15373, -26953, 2075),
                (-15373, 26953, 0),
            ):
                profile.write_text(
                    json.dumps(dict(imu_geometry=dict(underside_center_to_imu_um=list(wrong))))
                )
                with self.assertRaises(AssertionError):
                    runner._profile_vector(base)

    def test_chip_input_does_not_require_new_board_profile(self):
        session = SimpleNamespace(
            firmware=Path("does-not-exist"),
            board=SimpleNamespace(set_imu=lambda **kwargs: None),
        )
        physical = runner.PhysicalInput(session, "right")
        self.assertEqual(physical.sensor.mount.offset_m, (0.0, -0.1, 0.0))

    @staticmethod
    def session(*, bad_effective=False, board_commands=True):
        names = ("roll_cdeg", "pitch_cdeg", "yaw_cdeg", "x_um", "y_um", "z_um")
        props = [dict(name=n, type=t) for n, t in zip(names, ("int16_t",) * 3 + ("int32_t",) * 3)]
        commands = {
            name: dict(properties=props if name.startswith("SET") else [])
            for name in (
                "SET_IMU_MOUNT",
                "GET_IMU_MOUNT",
                "SET_BOARD_MOUNT",
                "GET_BOARD_MOUNT",
            )
        }
        if not board_commands:
            commands.pop("SET_BOARD_MOUNT")
            commands.pop("GET_BOARD_MOUNT")

        class Session:
            def __init__(self):
                self.boot = 1
                self.calls = []
                self.schema = SimpleNamespace(commands=commands, version=(2, 24, 0))
                self.board = SimpleNamespace(fram_image=b"saved geometry")
                self.center = None
                self.chip = None

            def ask(self, command, **fields):
                self.calls.append((command, dict(fields), self.boot))
                if command.startswith("SET_"):
                    roll = math.radians(fields["roll_cdeg"] / 100)
                    local = (
                        -15373,
                        26953 * math.cos(roll) - 2075 * math.sin(roll),
                        26953 * math.sin(roll) + 2075 * math.cos(roll),
                    )
                    if command == "SET_BOARD_MOUNT":
                        self.center = dict(fields, configured=1)
                        self.chip = dict(self.center)
                        for key, value in zip(("x_um", "y_um", "z_um"), local):
                            self.chip[key] += round(value)
                    else:
                        self.chip = dict(fields, configured=1)
                        self.center = dict(self.chip)
                        for key, value in zip(("x_um", "y_um", "z_um"), local):
                            self.center[key] -= round(value)
                    return {}
                if command == "GET_BOARD_MOUNT":
                    return dict(self.center)
                if command == "GET_IMU_MOUNT":
                    return dict(self.chip, x_um=self.chip["x_um"] + (10 if bad_effective else 0))
                if command == "GET_IMU_TRACKING_STATUS":
                    return dict(state=0)
                if command == "SAVE_CONFIGURATION":
                    return {}
                raise AssertionError(command)

            def reboot(self, image):
                self.calls.append(("reboot", {}, self.boot))
                self.boot += 1

            def status(self):
                return dict(valid=0)

            def mark(self, *args, **kwargs):
                pass

        return Session()

    @staticmethod
    def stimulus():
        return SimpleNamespace(
            step=lambda: None,
            sensor=SimpleNamespace(mount=None),
            center_to_imu=(-0.015373, 0.026953, 0.002075),
        )

    def test_fixed_pivot_reads_both_orientations_before_reboot(self):
        s = self.session()
        result = runner._geometry(s, "bottom_reorientation", self.stimulus(), "board-center")
        flat, vertical = result["reorientation"]
        self.assertEqual(flat["boot"], vertical["boot"])
        for phase in (flat, vertical):
            self.assertEqual(tuple(phase["center"][k] for k in ("x_um", "y_um", "z_um")), (0, 0, 0))
        self.assertEqual(
            tuple(flat["chip"][k] for k in ("x_um", "y_um", "z_um")),
            (-15373, 26953, 2075),
        )
        self.assertEqual(
            tuple(vertical["chip"][k] for k in ("x_um", "y_um", "z_um")),
            (-15373, -2075, 26953),
        )
        self.assertEqual(result["readback"], result["reboot_readback"])
        self.assertEqual(result["effective_chip"], result["reboot_effective_chip"])
        self.assertIsNotNone(result["legacy_roundtrip"])

    def test_vertical_right_effective_offset(self):
        result = runner._geometry(self.session(), "right", self.stimulus(), "board-center")
        self.assertEqual(
            tuple(result["effective_chip"][k] for k in ("x_um", "y_um", "z_um")),
            (-15373, -102075, 26953),
        )

    def test_wrong_effective_chip_is_rejected(self):
        with self.assertRaisesRegex(AssertionError, "Effective chip offset"):
            runner._geometry(
                self.session(bad_effective=True),
                "right",
                self.stimulus(),
                "board-center",
            )

    def test_old_protocol_has_no_board_center_substitution(self):
        s = self.session(board_commands=False)
        with self.assertRaisesRegex(LookupError, "SET_BOARD_MOUNT"):
            runner._geometry(s, "right", self.stimulus(), "board-center")
        self.assertEqual(s.calls, [])
        result = runner._geometry(s, "right", self.stimulus())
        self.assertEqual(result["readback"]["y_um"], -100000)
        self.assertEqual(result["readback"]["roll_cdeg"], 0)


if __name__ == "__main__":
    unittest.main()
