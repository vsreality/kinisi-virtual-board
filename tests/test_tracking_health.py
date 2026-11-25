"""Regression checks for source-time and quadrature measurement boundaries."""

import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from kinisi_virtual_board.feedback_selector import health, health_wire as wire


class HealthSourceTests(unittest.TestCase):
    @staticmethod
    def replies():
        now = 250000
        replies = [
            dict(
                source=i,
                health=0,
                flags=3,
                timestamp_us=230000,
                age_us=20000,
                max_age_us=35000,
                reason=0,
                detail=0,
            )
            for i in range(3)
        ]
        dies = dict(
            accel_timestamp_us=230000,
            gyro_timestamp_us=232500,
            accel_age_us=20000,
            gyro_age_us=17500,
        )

        def ask(command, source=None):
            return dies if source is None else replies[source]

        session = SimpleNamespace(
            board=SimpleNamespace(now_us=now),
            ask=ask,
            schema=SimpleNamespace(commands={"GET_FEEDBACK_SOURCE_HEALTH"}),
        )
        return session, replies, dies

    def test_health_uses_older_die_and_actual_clock(self):
        session, replies, _ = self.replies()
        wire.available(wire.sources(session))
        replies[1].update(timestamp_us=232500, age_us=17500)
        with self.assertRaisesRegex(AssertionError, "IMU pair outside"):
            wire.sources(session)

    def test_old_acquisition_cannot_be_fresh(self):
        session, replies, _ = self.replies()
        replies[0].update(timestamp_us=10000, age_us=240000)
        with self.assertRaisesRegex(AssertionError, "False freshness"):
            wire.sources(session)

    def test_absent_die_has_unavailable_age(self):
        session, _, dies = self.replies()
        dies.update(gyro_timestamp_us=0, gyro_age_us=0)
        with self.assertRaisesRegex(AssertionError, "gyro age"):
            wire.sources(session)

    def test_disagreement_cannot_assign_an_input_fault(self):
        rows = {
            name: dict(health=3, reason=0, detail=0, flags=15)
            for name in ("encoder", "imu", "fused")
        }
        wire.unattributed_conflict(rows)
        for field, value in (("health", 1), ("reason", 3), ("flags", 3)):
            broken = dict(rows, imu=dict(rows["imu"], **{field: value}))
            with self.subTest(field=field), self.assertRaises(AssertionError):
                wire.unattributed_conflict(broken)

    def test_logged_single_die_freeze_matches_hardware_boundary(self):
        class FrozenGyroSession:
            def __init__(self):
                self.board = Mock(now_us=0)
                self.mark = Mock()

            def status(self):
                return {"valid": self.board.now_us < 5000}

        session = FrozenGyroSession()
        stim = SimpleNamespace(held=[50, 70], values=[1, 2, 3, 4, 5, 6])

        def step(**_):
            stim.values[:] = [v + 1 for v in stim.values]
            stim.held[:] = [t + 2500 for t in stim.held]
            session.board.now_us += 2500

        stim.step = step
        from unittest.mock import patch

        with (
            patch.object(health, "reading", return_value={"selector": {}}),
            patch.object(health, "source_health", return_value={}),
            patch.object(health, "_invalid", side_effect=StopIteration),
        ):
            with self.assertRaises(StopIteration):
                health.outage(session, stim, "gyro_stale")
        deliveries = [
            c.kwargs
            for c in session.mark.call_args_list
            if c.args == ("sensor_delivery",)
        ]
        self.assertEqual([r["accel_us"] for r in deliveries], [2550, 5050])
        self.assertEqual(
            [r["acceleration"] for r in deliveries], [[2, 3, 4], [3, 4, 5]]
        )
        for call, row in zip(
            session.board.set_imu.call_args_list, deliveries, strict=True
        ):
            self.assertEqual((row["angular_velocity"], row["gyro_us"]), ((4, 5, 6), 70))
            self.assertEqual(call.args, (row["acceleration"], row["angular_velocity"]))
            self.assertEqual(
                call.kwargs, {k: row[k] for k in ("accel_us", "gyro_us", "status")}
            )

    def test_preexisting_output_is_preserved(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder)
            with self.assertRaises(FileExistsError):
                health.run(SimpleNamespace(out=out, firmware=out))
