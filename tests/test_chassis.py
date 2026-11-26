import math
import unittest

from kinisi_virtual_board.chassis import Chassis, VirtualIMU


class ChassisTests(unittest.TestCase):
    def test_contact_loss_separates_shaft_travel_from_chassis_motion(self):
        geometry = {"wheel_diameter": .1, "wheel_base": .3}
        free = Chassis("differential", geometry, (.1, .1, .1))
        slipping = Chassis("differential", geometry, (.1, .1, .1))
        zero_grip = Chassis("differential", geometry, (.1, .1, .1))
        for _ in range(100):
            for body, grip in ((free, 1.), (slipping, .5), (zero_grip, 0.)):
                body.step((.02, .02, 0, 0), (grip,)*4, .01)
        expected = .1 * (1 - .1 * (1 - math.exp(-10)))
        self.assertAlmostEqual(free.pose[0], expected, places=10)
        self.assertAlmostEqual(slipping.pose[0], expected/2, places=10)
        self.assertEqual(zero_grip.pose, (0., 0., 0.))
        self.assertEqual(free.pose[1:], (0., 0.))

    def test_mounted_wheel_directions(self):
        cases = [
            ("differential", {"wheel_base": .3}, (-.02, .02, 0, 0), 2),
            ("omni", {"robot_radius": .17}, (.01, .01, -.02, 0), 1),
            ("mecanum", {"width": .26, "length": .34}, (.02, -.02, .02, -.02), 1),
            ("mecanum", {"width": .26, "length": .34}, (.02, .02, -.02, -.02), 2),
        ]
        for preset, geometry, delta, axis in cases:
            with self.subTest(preset=preset, axis=axis):
                chassis = Chassis(preset, dict(geometry, wheel_diameter=.1), (.1, .1, .1))
                chassis.step(delta, (1.,)*4, .01)
                self.assertGreater(chassis.twist[axis], 0)
                for other in {0, 1, 2} - {axis}:
                    self.assertAlmostEqual(chassis.twist[other], 0.)

    def test_imu_mount_sign_and_determinism(self):
        units = [VirtualIMU(123), VirtualIMU(123)]
        for _ in range(10):
            a, b = [u.sample((2., -1., 9.80665), .5) for u in units]
        self.assertEqual(a, b)
        for actual, expected, offset in zip(a, (-2., 1., 9.80665, 0., 0., .5), units[0].bias):
            self.assertAlmostEqual(actual, expected + offset, delta=.05)
