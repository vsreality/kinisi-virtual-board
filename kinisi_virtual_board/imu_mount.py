"""Rigid board installation on a planar chassis with forward, left and up axes."""

from dataclasses import dataclass
import math

IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
ROTATION_TOLERANCE = 1e-6


def _vector(values):
    values = tuple(float(v) for v in values)
    if len(values) != 3 or not all(math.isfinite(v) for v in values):
        raise ValueError("Expected three finite coordinates")
    return values


@dataclass(frozen=True)
class Mount:
    """Chip offset in chassis metres and active board-to-chassis rotation."""

    offset_m: tuple = (0.0, 0.0, 0.0)
    board_to_chassis: tuple = IDENTITY

    def __post_init__(self):
        offset = _vector(self.offset_m)
        rows = tuple(_vector(row) for row in self.board_to_chassis)
        if len(rows) != 3:
            raise ValueError("Expected a 3x3 board-to-chassis rotation")
        for i, a in enumerate(rows):
            for j, b in enumerate(rows[: i + 1]):
                if abs(sum(x * y for x, y in zip(a, b)) - int(i == j)) > ROTATION_TOLERANCE:
                    raise ValueError("Board-to-chassis rotation must be orthonormal")
        a, b, c = rows
        det = (
            a[0] * (b[1] * c[2] - b[2] * c[1])
            - a[1] * (b[0] * c[2] - b[2] * c[0])
            + a[2] * (b[0] * c[1] - b[1] * c[0])
        )
        if abs(det - 1.0) > ROTATION_TOLERANCE:
            raise ValueError("Board-to-chassis rotation must be right-handed")
        object.__setattr__(self, "offset_m", offset)
        object.__setattr__(self, "board_to_chassis", rows)

    def to_board(self, chassis_vector):
        """Resolve a chassis-frame vector along the installed board axes."""
        return tuple(
            sum(a * b for a, b in zip(column, chassis_vector)) for column in zip(*self.board_to_chassis)
        )

    def chip_inputs(self, origin_force, gyro, yaw_acceleration):
        """Specific force at the displaced chip and angular rate, in board axes."""
        if self.offset_m != (0.0, 0.0, 0.0) and (gyro[0] != 0.0 or gyro[1] != 0.0):
            raise ValueError("Offset chip model requires planar chassis rotation")
        rx, ry, _ = self.offset_m
        wz = gyro[2]
        force = (
            origin_force[0] - wz * wz * rx - yaw_acceleration * ry,
            origin_force[1] - wz * wz * ry + yaw_acceleration * rx,
            origin_force[2],
        )
        return self.to_board(force), self.to_board(gyro)
