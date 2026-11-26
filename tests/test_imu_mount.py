import math
import unittest

from kinisi_virtual_board.bmi088 import BMI088, G, LSB
from kinisi_virtual_board.imu_mount import IDENTITY, Mount, board_center_to_imu_m


def register_delta(mount, acceleration=(0.0, 0.0, G), rate=0.0, alpha=0.0):
    """Common die noise and bias cancel against the same unit at centered rest."""
    reference = BMI088(131)
    sensor = BMI088(131, mount=mount)
    reference.advance(500000)
    sensor.advance(500000, acceleration, (0.0, 0.0, rate), yaw_acceleration=alpha)
    return [value - base for value, base in zip(sensor.values, reference.values)]


class PhysicalMountTests(unittest.TestCase):
    def check_registers(self, actual, expected):
        for axis, (got, want) in enumerate(zip(actual, expected, strict=True)):
            self.assertAlmostEqual(got, want, delta=2 * LSB[axis // 3], msg=f"chip axis {axis}")

    def test_bottom_center_fixed_while_chip_rotates_and_accelerates(self):
        profile = {"imu_geometry": {"underside_center_to_imu_um": [-15373, 26953, 2075]}}
        vector = board_center_to_imu_m(profile)
        # Independent dimensions in board axes, before micrometre encoding.
        dx = (25.14 - 81.025126 / 2) * 0.001
        dy = (66.025126 / 2 - 6.06) * 0.001
        dz = (1.6 + 0.95 / 2) * 0.001
        roll90 = ((1, 0, 0), (0, 0, -1), (0, 1, 0))
        for orientation, y_center in ((IDENTITY, 0.0), (roll90, 0.0), (roll90, -0.100)):
            with self.subTest(orientation=orientation, y_center=y_center):
                chip = Mount.from_board_center((0.0, y_center, 0.0), orientation, center_to_imu_m=vector)
                expected = (dx, y_center + dy, dz) if orientation == IDENTITY else (dx, y_center - dz, dy)
                for actual, target in zip(chip.offset_m, expected):
                    self.assertAlmostEqual(actual, target, delta=0.5e-6)
                for alpha in (0.0, 1.0, -1.0):
                    # Chassis-origin rotation at 2 rad/s. Specific force of U601
                    # relative to an identically seeded centered, level sensor.
                    x, y, _ = expected
                    ax, ay = -4 * x - alpha * y, -4 * y + alpha * x
                    if orientation == IDENTITY:
                        expected_delta = (-ax, -ay, 0.0, 0.0, 0.0, 2.0)
                    else:
                        expected_delta = (-ax, -G, -ay - G, 0.0, -2.0, 0.0)
                    self.check_registers(register_delta(chip, rate=2.0, alpha=alpha), expected_delta)

    def test_board_center_uses_selected_profile_and_rejects_invalid_geometry(self):
        vector = board_center_to_imu_m({"imu_geometry": {"underside_center_to_imu_um": [1000, -2000, 3000]}})
        mount = Mount.from_board_center(
            (0.1, 0.2, 0.3), ((0, -1, 0), (1, 0, 0), (0, 0, 1)), center_to_imu_m=vector
        )
        for actual, target in zip(mount.offset_m, (0.102, 0.201, 0.303)):
            self.assertAlmostEqual(actual, target)
        with self.assertRaisesRegex(ValueError, "lacks"):
            board_center_to_imu_m({})
        for invalid in ([True, 2, 3], [1, 2], [1.0, 2.0, 3.0], [0, 0, 2**31]):
            with self.assertRaises(ValueError):
                board_center_to_imu_m({"imu_geometry": {"underside_center_to_imu_um": invalid}})

    def test_centered_identity_preserves_every_legacy_sample(self):
        legacy, explicit = BMI088(131), BMI088(131, mount=Mount())
        for i in range(1, 501):
            for sensor in (legacy, explicit):
                sensor.advance(i * 2500, (0.2, -0.1, G), (0, 0, math.sin(i * 0.01)))
            self.assertEqual(
                (legacy.values, legacy.accel_us, legacy.gyro_us),
                (explicit.values, explicit.accel_us, explicit.gyro_us),
            )

    def test_lever_force_changes_sign_on_opposite_sides_and_on_braking(self):
        for ry in (-0.1, 0.1):
            for alpha in (-1.0, 0.0, 1.0):
                # Positive left/forward chassis forces read negative on chip Y/X.
                self.check_registers(
                    register_delta(Mount((0.0, ry, 0.0)), rate=2.0, alpha=alpha),
                    (alpha * ry, 4 * ry, 0.0, 0.0, 0.0, 2.0),
                )
        self.check_registers(
            register_delta(Mount((0.0, 0.0, 0.1)), rate=-2.0, alpha=1.0), (0.0, 0.0, 0.0, 0.0, 0.0, -2.0)
        )

    def test_each_vertical_face_and_inverted_board(self):
        # Expected chip axes determined directly by which board edge points up.
        for matrix, up in [
            (((1, 0, 0), (0, 0, -1), (0, 1, 0)), (0, -1, 0)),
            (((1, 0, 0), (0, 0, 1), (0, -1, 0)), (0, 1, 0)),
            (((0, 0, 1), (0, 1, 0), (-1, 0, 0)), (1, 0, 0)),
            (((0, 0, -1), (0, 1, 0), (1, 0, 0)), (-1, 0, 0)),
            (((1, 0, 0), (0, -1, 0), (0, 0, -1)), (0, 0, -1)),
        ]:
            self.check_registers(
                register_delta(Mount(board_to_chassis=matrix), rate=0.5),
                (up[0] * G, up[1] * G, (up[2] - 1) * G, *(v * 0.5 for v in up)),
            )

    def test_combined_angles_offset_and_translation(self):
        # Columns found by rotating physical basis vectors successively about
        # X, Y and Z; expected chip measurements computed with inverse rotations.
        r, p, y = map(math.radians, (30, -20, 35))

        def rotate_pair(a, b, angle):
            return a * math.cos(angle) - b * math.sin(angle), a * math.sin(angle) + b * math.cos(angle)

        def rotate_forward(v):
            x, y0, z = v
            y0, z = rotate_pair(y0, z, r)
            z, x = rotate_pair(z, x, p)
            x, y0 = rotate_pair(x, y0, y)
            return x, y0, z

        def to_chip(v):
            x, y0, z = v
            x, y0 = rotate_pair(x, y0, -y)
            z, x = rotate_pair(z, x, -p)
            y0, z = rotate_pair(y0, z, -r)
            return -x, -y0, z

        columns = [rotate_forward(v) for v in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
        mount = Mount((0.08, -0.1, 0.04), tuple(zip(*columns)))
        # At w=2, alpha=1, chip adds (-.22, +.48, 0) in chassis axes.
        x, y0, z = to_chip((0.3 - 0.22, -0.15 + 0.48, G))
        self.check_registers(
            register_delta(mount, (0.3, -0.15, G), 2.0, 1.0), (x, y0, z - G, *to_chip((0, 0, 2)))
        )

    def test_rate_differences_follow_simulation_time(self):
        mount = Mount((0.08, -0.1, 0.0))
        analytic, derived = BMI088(131, mount=mount), BMI088(131, mount=mount)
        for i in range(1, 101):
            rate = i * 0.005
            analytic.advance(i * 2500, body_gyro=(0.0, 0.0, rate), yaw_acceleration=2.0)
            derived.advance(i * 2500, body_gyro=(0.0, 0.0, rate))
            derived.advance(i * 2500, body_gyro=(0.0, 0.0, 99.0))
            self.assertEqual(analytic.values, derived.values)

    def test_mount_validation(self):
        for matrix in [
            ((1, 0, 0), (0, 1, 0), (0, 0, -1)),
            ((1, 0, 0), (1, 0, 0), (0, 0, 1)),
            ((1, 0, 0), (0, 1, 0), (0, 0, float("nan"))),
            ((1, 0, 0),),
        ]:
            with self.assertRaises(ValueError):
                Mount(board_to_chassis=matrix)
        with self.assertRaises(ValueError):
            Mount((0, 0, float("inf")))
