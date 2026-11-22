"""Exercise the firmware-facing C observation ABI on an isolated native board."""
import ctypes as ct
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from kinisi_virtual_board import FirmwareBoard
from test_firmware import FIRMWARE
from tools import encoder_baseline


def read_observation(board, index=0):
    read = board._lib.hw_imu_read_observation
    read.argtypes = (ct.c_uint8, ct.POINTER(ct.c_double), ct.POINTER(ct.c_uint64), ct.POINTER(ct.c_uint64))
    read.restype = ct.c_int
    sample = (ct.c_double * 6)(*([-8.] * 6))
    accel, gyro = ct.c_uint64(27), ct.c_uint64(31)
    status = read(index, sample, ct.byref(accel), ct.byref(gyro))
    return status, tuple(sample), accel.value, gyro.value


class IMUObservationTests(unittest.TestCase):
    values = (1., 2., 3., 4., 5., 6.)
    unchanged = ((-8.,) * 6, 27, 31)

    def test_frozen_identical_and_independently_reordered_samples(self):
        with FirmwareBoard(firmware=FIRMWARE) as board:
            board.advance(.1)
            self.assertEqual(read_observation(board), (2, *self.unchanged))

            def inject(**kwargs):
                board.set_imu(self.values[:3], self.values[3:], **kwargs)

            inject(accel_us=0, gyro_us=70000)
            board.advance(.1)
            self.assertEqual(read_observation(board), (2, *self.unchanged))
            inject(accel_us=60000, gyro_us=0)
            self.assertEqual(read_observation(board), (2, *self.unchanged))
            inject(accel_us=60000, gyro_us=70000)
            self.assertEqual(read_observation(board), (0, self.values, 60000, 70000))
            board.advance(.25)
            self.assertEqual(read_observation(board), (0, self.values, 60000, 70000))
            inject()
            self.assertEqual(read_observation(board), (0, self.values, 450000, 450000))
            board.advance(.01)
            for accel_us, gyro_us in [(440000, 450001), (450002, 439999)]:
                inject(accel_us=accel_us, gyro_us=gyro_us)
                self.assertEqual(read_observation(board), (0, self.values, accel_us, gyro_us))
            inject(status=3)
            board.advance(.01)
            self.assertEqual(read_observation(board), (3, *self.unchanged))
            inject(accel_us=455001, gyro_us=456002)
            self.assertEqual(read_observation(board), (0, self.values, 455001, 456002))
            self.assertEqual(read_observation(board, 1), (1, *self.unchanged))
            inject(accel_us=2**32 + 5)
            self.assertEqual(read_observation(board), (0, self.values, 2**32 + 5, board.now_us))

    def test_legacy_injection_and_sensorless_board(self):
        with FirmwareBoard(firmware=FIRMWARE) as board:
            board.advance(.08)
            inject = board._lib.sim_imu
            inject.argtypes = ct.POINTER(ct.c_double), ct.c_int
            inject.restype = None
            inject((ct.c_double * 6)(*self.values), 0)
            board.advance(.01)
            self.assertEqual(read_observation(board), (0, self.values, 80000, 80000))
            for invalid in [-1, 2**64, 1.5, True]:
                with self.assertRaises(ValueError):
                    board.set_imu(gyro_us=invalid)
            self.assertEqual(read_observation(board), (0, self.values, 80000, 80000))
        with FirmwareBoard(firmware=FIRMWARE, revision="0.3.0") as board:
            board.advance(.08)
            board.set_imu()
            self.assertEqual(read_observation(board), (1, *self.unchanged))

    def test_baseline_delivers_previous_acquisition_to_hal(self):
        observations = []

        class RecordingBoard(FirmwareBoard):
            def set_imu(self, *args, **kwargs):
                super().set_imu(*args, **kwargs)
                status, _, accel, gyro = read_observation(self)
                observations.append((self.now_us, status, accel, gyro))

        cases = json.loads(encoder_baseline.CASES.read_text(encoding="utf-8"))
        cases["courses"]["velocity"]["duration_ms"] = 30
        with tempfile.TemporaryDirectory() as folder, patch.object(encoder_baseline, "FirmwareBoard", RecordingBoard):
            encoder_baseline.run_case(Path(folder) / "case", FIRMWARE, cases, "differential", "velocity", "nominal", 131)
        self.assertEqual(observations, [(t, 0, t - 10000, t - 10000) for t in range(600000, 640000, 10000)])

    def test_stationary_warmup_is_delayed_and_preserves_course(self):
        deliveries = []

        class RecordingBoard(FirmwareBoard):
            def set_imu(self, acceleration, angular_velocity, **times):
                super().set_imu(acceleration, angular_velocity, **times)
                deliveries.append((self.now_us, times, (*acceleration, *angular_velocity)))

        def setup(ask):
            self.assertFalse(deliveries)
            self.assertEqual(ask("GET_ODOMETRY_FREQUENCY"), bytes([50, 0]))
            setup_calls.append(True)

        setup_calls = []
        cases = json.loads(encoder_baseline.CASES.read_text(encoding="utf-8"))
        cases["courses"]["position"]["duration_ms"] = 30
        args = FIRMWARE, cases, "omni", "position", "nominal", 131
        with tempfile.TemporaryDirectory() as folder:
            plain, warm = Path(folder) / "plain", Path(folder) / "warm"
            encoder_baseline.run_case(plain, *args)
            with patch.object(encoder_baseline, "FirmwareBoard", RecordingBoard):
                encoder_baseline.run_case(warm, *args, setup=setup, stationary_warmup=True)
            for name in ("sensors.csv", "firmware.csv", "reference.csv", "targets.csv"):
                self.assertEqual((plain / name).read_bytes(), (warm / name).read_bytes(), name)
        self.assertEqual(setup_calls, [True])
        self.assertEqual([t for t, _, _ in deliveries], list(range(10000, 640000, 10000)))
        for t, times, _ in deliveries:
            self.assertEqual(times, dict(accel_us=t - 10000, gyro_us=t - 10000))
        self.assertGreater(len({values for t, _, values in deliveries if t < 600000}), 1)
