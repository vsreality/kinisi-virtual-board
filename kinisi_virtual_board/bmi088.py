"""Kinisi v0.4 BMI088, nominal rails. Bosch BST-BMI088-DS000-19 pp. 9–11, 16, 29–30, 39.

The continuous single-pole response and fixed independent sampling phases are
model assumptions. Input is a held body-frame specific force and angular rate
over the interval ending at at_us. Output is the most recent native-axis SI
register value of each die. Digital delivery belongs to the caller.
"""
import hashlib
import math
import operator
from .imu_mount import Mount

MODEL_ID = "bmi088-v04-1"
G = 9.80665
RAD = math.pi / 180
PERIOD_US = 10_000
CUTOFF = (40., 32.)
LSB = (6 * G / 32768, 500 * RAD / 32768)
DENSITY = (160e-6 * G, 160e-6 * G, 190e-6 * G, .014 * RAD, .014 * RAD, .014 * RAD)
RMS = tuple(n * math.sqrt(math.pi * CUTOFF[i // 3] / 2) for i, n in enumerate(DENSITY))
MOUNT = (-1, -1, 1)


class _Random:
    """Mulberry32, then Box-Muller. The native backend uses the same stream."""
    def __init__(self, seed):
        self.state = seed & 0xffffffff

    def uniform(self):
        mask = 0xffffffff
        self.state = (self.state + 0x6d2b79f5) & mask
        x = self.state
        x = ((x ^ (x >> 15)) * (x | 1)) & mask
        x ^= (x + (((x ^ (x >> 7)) * (x | 61)) & mask)) & mask
        return ((x ^ (x >> 14)) + .5) / 4294967296.

    def normal(self):
        return math.sqrt(-2 * math.log(self.uniform())) * math.cos(math.tau * self.uniform())


class BMI088:
    def __init__(self, seed=1, temperature_c=25., *, noise_seed=None, mount=None):
        if not -40 <= temperature_c <= 85:
            raise ValueError("BMI088 operating temperature must be -40 to 85 C")
        seed = operator.index(seed)
        if not 0 <= seed <= 0xffffffff:
            raise ValueError("BMI088 seed must be an unsigned 32-bit integer")
        unit_rng = _Random(seed)
        if mount is not None and not isinstance(mount, Mount):
            raise TypeError("mount must be a Mount")
        self.mount = mount
        self._previous_yaw_rate = 0.
        gravity = mount.to_board((0., 0., G)) if mount is not None else (0., 0., G)
        self._signal = [sign * v for sign, v in zip(MOUNT, gravity)] + [0.] * 3
        self._noise = [0.] * 6
        self.values = [0.] * 6
        self.bias = []
        self._gain = []
        for i in range(6):
            accel = i < 3
            offset = (.020 * G if accel else RAD) * (2 * unit_rng.uniform() - 1)
            tco = (.0002 * G if accel else .015 * RAD) * (2 * unit_rng.uniform() - 1)
            tcs = (.00002 if accel else .0003) * (2 * unit_rng.uniform() - 1)
            self.bias.append(offset + (temperature_c - 25) * tco)
            self._gain.append(1 + (temperature_c - 25) * tcs)
        if noise_seed is None:
            noise_seed = seed
        elif isinstance(noise_seed, str):
            noise_seed = int.from_bytes(hashlib.sha256(noise_seed.encode()).digest()[:4], 'little')
        self._streams = [_Random(noise_seed ^ 0x9e3779b9), _Random(noise_seed ^ 0x85ebca6b)]
        self.accel_us = self.gyro_us = 0
        self._now = 0
        self._next = [PERIOD_US, PERIOD_US + 2500]

    def advance(self, at_us, body_accel=(0., 0., G), body_gyro=(0., 0., 0.), *, yaw_acceleration=None):
        at_us = operator.index(at_us)
        if at_us < self._now:
            raise ValueError("BMI088 time must be monotonic")
        if len(body_accel) != 3 or len(body_gyro) != 3:
            raise ValueError("BMI088 inputs require three axes per die")
        if not all(math.isfinite(v) for v in (*body_accel, *body_gyro)):
            raise ValueError("BMI088 inputs must be finite")
        if yaw_acceleration is not None and not math.isfinite(yaw_acceleration):
            raise ValueError("Yaw acceleration must be finite")
        if self.mount is not None:
            if at_us == self._now:
                return
            if yaw_acceleration is None:
                yaw_acceleration = (body_gyro[2] - self._previous_yaw_rate) * 1e6 / (at_us - self._now)
            yaw_rate = body_gyro[2]
            body_accel, body_gyro = self.mount.chip_inputs(body_accel, body_gyro, yaw_acceleration)
            self._previous_yaw_rate = yaw_rate
        for die, inputs in enumerate((body_accel, body_gyro)):
            target = tuple(sign * value for sign, value in zip(MOUNT, inputs))
            start = self._now
            while start < at_us:
                end = min(at_us, self._next[die])
                retain = math.exp(-math.tau * CUTOFF[die] * (end - start) / 1e6)
                for axis, level in enumerate(target):
                    i = die * 3 + axis
                    self._signal[i] = level + (self._signal[i] - level) * retain
                if end == self._next[die]:
                    self._publish(die, end)
                    self._next[die] += PERIOD_US
                start = end
        self._now = at_us

    def _publish(self, die, stamp):
        # Sampled stationary single-pole white noise. Preserve its covariance
        # between events and do not consume random draws on intermediate polls.
        prior = self.gyro_us if die else self.accel_us
        rho = math.exp(-math.tau * CUTOFF[die] * PERIOD_US / 1e6) if prior else 0.
        for i in range(3 * die, 3 * die + 3):
            self._noise[i] = rho * self._noise[i] + math.sqrt(1 - rho*rho) * self._streams[die].normal()
            reading = self._signal[i] * self._gain[i] + self.bias[i] + RMS[i] * self._noise[i]
            code = math.floor(reading / LSB[die] + .5)
            self.values[i] = min(32767, max(-32768, code)) * LSB[die]
        if die:
            self.gyro_us = stamp
        else:
            self.accel_us = stamp
