"""External planar plant for encoder/IMU evaluations. No firmware dependencies."""
import math
from .bmi088 import BMI088, PERIOD_US


def rotate(x, y, angle):
    c, s = math.cos(angle), math.sin(angle)
    return c * x - s * y, s * x + c * y


class Chassis:
    """Wheel-ground constraints, finite body response, and world-frame reference pose."""

    def __init__(self, preset, geometry, response_s):
        radius = geometry["wheel_diameter"] / 2
        if preset == "differential":
            arm = geometry["wheel_base"] / 2
            rows = [(1, 0, -arm), (1, 0, arm)]
        elif preset == "omni":
            arm = geometry["robot_radius"]
            rows = [(math.sqrt(3)/2, .5, arm), (-math.sqrt(3)/2, .5, arm), (0, -1, arm)]
        elif preset == "mecanum":
            arm = (geometry["length"] + geometry["width"]) / 2
            rows = [(1, 1, arm), (1, -1, arm), (1, 1, -arm), (1, -1, -arm)]
        else:
            raise ValueError(f"Unknown wheel arrangement {preset}")
        # Columns of all three mounting matrices are orthogonal. Differential
        # wheels have a lateral no-slip constraint in this planar plant.
        self.projection = []
        for column in zip(*rows):
            norm = sum(entry**2 for entry in column)
            self.projection.append(tuple(radius * entry / norm if norm else 0. for entry in column))
        self.response_s = response_s
        self.pose = (0., 0., 0.)
        self.twist = (0., 0., 0.)

    def step(self, shaft_delta, traction, dt):
        """Advance from unquantized shaft rotation and per-wheel contact transmission."""
        previous = self.twist
        velocity, travel = [], []
        for weights, old, tau in zip(self.projection, previous, self.response_s):
            target = sum(k * delta * grip for k, delta, grip in zip(weights, shaft_delta, traction)) / dt
            approach = -math.expm1(-dt / tau)
            velocity.append(old + (target - old) * approach)
            travel.append(target * dt + (old - target) * tau * approach)
        x, y, yaw = self.pose
        dx, dy = rotate(travel[0], travel[1], yaw + travel[2] / 2)
        self.pose = x + dx, y + dy, yaw + travel[2]
        self.twist = tuple(velocity)
        # Specific force at the body origin for a level mount (+g when stationary).
        vx, vy, wz = [(a+b)/2 for a, b in zip(previous, velocity)]
        return ((velocity[0]-previous[0])/dt - wz*vy,
                (velocity[1]-previous[1])/dt + wz*vx, 9.80665)


class VirtualIMU:
    """Compatibility sampler for callers supplying one input per 10 ms frame.

    For sample identity or delivery timing use BMI088.advance directly.
    """
    def __init__(self, seed, *, noise_seed=None):
        self.sensor = BMI088(seed, noise_seed=noise_seed)
        self.bias = self.sensor.bias
        self._at_us = 2500

    def sample(self, specific_force, yaw_rate):
        self._at_us += PERIOD_US
        self.sensor.advance(self._at_us, specific_force, (0., 0., yaw_rate))
        return list(self.sensor.values)
