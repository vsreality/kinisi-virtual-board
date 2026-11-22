"""External planar plant for encoder/IMU evaluations. No firmware dependencies."""
import math
import random


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
    """Fixed board-aligned mount at chassis origin; output is in BMI088 native axes."""
    def __init__(self, seed):
        self.rng = random.Random(seed)
        self.bias = [self.rng.uniform(-.035, .035) for _ in range(3)]
        self.bias += [self.rng.uniform(-.004, .004) for _ in range(3)]

    def sample(self, specific_force, yaw_rate):
        values = (-specific_force[0], -specific_force[1], specific_force[2], 0., 0., yaw_rate)
        sample = []
        for index, value in enumerate(values):
            accel = index < 3
            full_scale = 6 * 9.80665 if accel else math.radians(500)
            quantum = full_scale / 32768
            noisy = value + self.bias[index] + self.rng.gauss(0, .025 if accel else .002)
            sample.append(min(32767, max(-32768, round(noisy / quantum))) * quantum)
        return sample
