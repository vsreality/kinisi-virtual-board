import math
import statistics
import unittest

from kinisi_virtual_board.bmi088 import BMI088, G, RAD


class BMI088Tests(unittest.TestCase):
    def test_each_die_holds_until_its_next_event(self):
        m = BMI088(0)
        for t, stamps in [(9999, (0, 0)), (10000, (10000, 0)), (12499, (10000, 0)),
                          (12500, (10000, 12500)), (20000, (20000, 12500))]:
            m.advance(t)
            self.assertEqual((m.accel_us, m.gyro_us), stamps)
        held = list(m.values)
        m.advance(22499)
        self.assertEqual(m.values, held)
        m.advance(22500)
        self.assertEqual(m.values[:3], held[:3])
        self.assertEqual(m.gyro_us, 22500)
        with self.assertRaises(ValueError):
            m.advance(22499)

    def test_chunking_and_signed_quantization(self):
        a, b = BMI088(123), BMI088(123)
        inputs = ((2., -1., G), (0., 0., .5))
        a.advance(20000, *inputs)
        for t in range(200, 20001, 200):
            b.advance(t, *inputs)
        self.assertEqual(a.values, b.values)
        q = (6*G/32768, 500*RAD/32768)
        self.assertEqual([round(v/q[i//3]) for i, v in enumerate(a.values)],
                         [-1045, 492, 5549, -12, -37, 1708])
        for v, count, i in zip(a.values, [-1045, 492, 5549, -12, -37, 1708], range(6)):
            self.assertEqual(v, count*q[i//3])
        a.advance(200000, (200., -200., 200.), (-50., 50., -50.))
        self.assertEqual(a.values, [v*q[i//3] for i, v in enumerate(
            [-32768, 32767, 32767, 32767, -32768, -32768])])

    def test_known_steps_and_independent_response_times(self):
        rest, motion = BMI088(7), BMI088(7)
        rest.advance(50000)
        motion.advance(50000)
        for t in (52500, 60000, 62500, 100000):
            rest.advance(t)
            motion.advance(t, (1., -2., G+3.), (1., -2., 3.))
            for i, delta in enumerate((-1, 2, 3, -1, 2, 3)):
                cutoff = 40 if i < 3 else 32
                stamp = motion.accel_us if i < 3 else motion.gyro_us
                elapsed = max(0., stamp - 50000)/1e6
                expected = delta * (1 - math.exp(-2*math.pi*cutoff*elapsed))
                self.assertAlmostEqual(motion.values[i] - rest.values[i], expected,
                                       delta=(6*G if i < 3 else 500*RAD)/32768)

    def test_stationary_noise_mean_rms_and_lag(self):
        m = BMI088(502)
        axes = [[] for _ in range(6)]
        for k in range(1, 20001):
            m.advance(k*10000 + 2500)
            for axis, value in zip(axes, m.values):
                axis.append(value)
        for i, values in enumerate(axes):
            cutoff = 40 if i < 3 else 32
            density = [160e-6*G, 160e-6*G, 190e-6*G, .014*RAD, .014*RAD, .014*RAD][i]
            sigma = density * math.sqrt(cutoff*math.pi/2)
            mean = statistics.mean(values)
            self.assertAlmostEqual(mean, m.bias[i] + (G if i == 2 else 0.), delta=.04*sigma)
            self.assertAlmostEqual(statistics.pstdev(values), sigma, delta=.025*sigma)
            self.assertAlmostEqual(statistics.correlation(values[:-1], values[1:]),
                                   math.exp(-.01*2*math.pi*cutoff), delta=.025)

    def test_seeded_offsets_temperature_and_sensitivity(self):
        cold, room, hot = [BMI088(132, temp) for temp in (5, 25, 45)]
        for i, offset in enumerate(room.bias):
            self.assertLess(abs(offset), .020*G if i < 3 else RAD)
            self.assertAlmostEqual(cold.bias[i] + hot.bias[i], 2*offset)
            self.assertLessEqual(abs(hot.bias[i] - offset), 20*(.0002*G if i < 3 else .015*RAD))
        self.assertNotEqual(cold.bias, hot.bias)
        # Remove offset/noise with a paired stationary unit, measure temperature gain.
        responses = []
        for t in (5., 25., 45.):
            base, driven = BMI088(132, t), BMI088(132, t)
            base.advance(250000)
            driven.advance(250000, (30., 0., G), (0., 0., 6.))
            responses.append([driven.values[i] - base.values[i] for i in (0, 5)])
        for column, demand in enumerate((-30., 6.)):
            lo, nominal, hi = (x[column] for x in responses)
            q = (6*G if column == 0 else 500*RAD)/32768
            self.assertAlmostEqual(nominal, demand, delta=q)
            self.assertAlmostEqual((lo + hi)/2, nominal, delta=2*q)
            self.assertNotEqual(lo, hi)
        for bad in (-41, 86, float("nan")):
            with self.assertRaises(ValueError):
                BMI088(1, bad)
