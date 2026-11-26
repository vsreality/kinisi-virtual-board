"""Physical mount agreement through real C firmware and the standalone host."""

import ctypes as ct
import math
import json
import os
from pathlib import Path
import struct
import subprocess
import unittest

from kinisi_virtual_board import FirmwareBoard
from kinisi_virtual_board.bmi088 import BMI088, G
from kinisi_virtual_board.imu_mount import Mount, IDENTITY, board_center_to_imu_m
from tools.encoder_baseline import Wire
from test_firmware import FIRMWARE
from test_imu_observation import read_observation


def configure(board):
    wire = Wire(board)
    wire.ask(
        "INIT",
        sdk_type=1,
        protocol_major=board.schema.version[0],
        protocol_minor=board.schema.version[1],
        client_capabilities=2,
    )
    wire.ask(
        "INITIALIZE_MECANUM_PLATFORM",
        wheels_diameter=0.1,
        length=0.34,
        width=0.26,
        encoder_resolution=4096,
    )
    return wire


class NativeMountTests(unittest.TestCase):
    def test_constructor_and_sensorless_profiles(self):
        mount = Mount((0.0, -0.1, 0.0))
        with FirmwareBoard(firmware=FIRMWARE, imu_mount=mount) as board:
            self.assertEqual(board.imu_mount, mount)
        for revision in ("0.3.0", "0.3.1"):
            with self.assertRaises(ValueError):
                FirmwareBoard(firmware=FIRMWARE, revision=revision, imu_mount=mount)
            with FirmwareBoard(firmware=FIRMWARE, revision=revision) as board:
                with self.assertRaises(ValueError):
                    board.set_imu_mount()

    def test_stationary_tilt_starts_at_projected_gravity(self):
        rotation = ((1.0, 0.0, 0.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0))
        mount = Mount((0.1, -0.1, 0.2), rotation)
        model = BMI088(1, mount=mount)
        with FirmwareBoard(firmware=FIRMWARE) as board:
            board.set_imu_mount(mount.offset_m, mount.board_to_chassis)
            for at in range(1000, 100001, 1000):
                board.advance(0.001)
                model.advance(at)
                if at >= 50000:
                    observed = read_observation(board)
                    self.assertEqual(observed[2:], (model.accel_us, model.gyro_us))
                    self.assertEqual(observed[1], tuple(model.values))
            # Gravity is already projected at initialization, not a synthetic step.
            self.assertGreater(abs(model.values[1]), 9.0)
            self.assertLess(abs(model.values[2]), 0.3)

    def test_actual_native_shaft_motion_matches_python_mount_model(self):
        angle = 0.4
        c, s = math.cos(angle), math.sin(angle)
        installations = (
            Mount(),
            Mount((0.0, -0.1, 0.0)),
            Mount((0.08, -0.1, 0.04), ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))),
        )
        for mount in installations:
            with self.subTest(mount=mount), FirmwareBoard(firmware=FIRMWARE) as board:
                board.set_imu_mount(mount.offset_m, mount.board_to_chassis)
                model = BMI088(1, mount=mount)
                previous = (0.0, 0.0, 0.0)
                previous_us = 0
                original_poll = board._poll

                def sample_and_poll():
                    nonlocal previous, previous_us
                    if board.now_us > previous_us:
                        w1, w2, w3, w4 = board.motor_velocity
                        body = (
                            0.1 / 2 / 4 * (w1 + w2 + w3 + w4),
                            0.1 / 2 / 4 * (w1 - w2 + w3 - w4),
                            0.1 / 2 / (2 * (0.34 + 0.26)) * (w1 + w2 - w3 - w4),
                        )
                        dt = (board.now_us - previous_us) / 1e6
                        force = (
                            (body[0] - previous[0]) / dt - body[2] * body[1],
                            (body[1] - previous[1]) / dt + body[2] * body[0],
                            G,
                        )
                        alpha = (body[2] - previous[2]) / dt
                        model.advance(
                            board.now_us,
                            force,
                            (0.0, 0.0, body[2]),
                            yaw_acceleration=alpha,
                        )
                        previous, previous_us = body, board.now_us
                    original_poll()

                board._poll = sample_and_poll
                wire = configure(board)
                for step in range(300):
                    if step == 20:
                        wire.ask("SET_PLATFORM_VELOCITY", x=20.0, y=5.0, t=12.0)
                    if step == 140:
                        wire.ask("SET_PLATFORM_VELOCITY", x=-10.0, y=3.0, t=-20.0)
                    board.advance(0.0025)
                    if board.now_us >= 50000:
                        observed = read_observation(board)
                        self.assertEqual(observed[2:], (model.accel_us, model.gyro_us))
                        for actual, expected in zip(observed[1], model.values):
                            self.assertAlmostEqual(actual, expected, delta=1e-12)
                self.assertTrue(any(board.motor_velocity))

    def test_explicit_identity_is_byte_identical_and_resettable_at_zero(self):
        def capture(setup):
            with FirmwareBoard(firmware=FIRMWARE) as board:
                setup(board)
                wire = configure(board)
                out = []
                wire.ask("SET_PLATFORM_VELOCITY", x=25.0, y=-5.0, t=15.0)
                for _ in range(50):
                    board.advance(0.01)
                    if board.now_us >= 50000:
                        out.append(wire.ask("GET_IMU"))
                return out

        legacy = capture(lambda b: None)
        self.assertEqual(legacy, capture(lambda b: b.set_imu_mount()))

        def reset(board):
            board.set_imu_mount(
                board_to_chassis=((1.0, 0.0, 0.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0))
            )
            board.set_imu_mount()

        self.assertEqual(legacy, capture(reset))

    def test_invalid_mount_and_running_reconfiguration_rejected(self):
        with FirmwareBoard(firmware=FIRMWARE) as board:
            invalid = [
                ((math.nan, 0, 0), IDENTITY),
                ((0, 0, 0), ((1, 0, 0), (0, 1, 0), (0, 0, -1))),
                ((0, 0, 0), ((2, 0, 0), (0, 1, 0), (0, 0, 1))),
                ((0, 0, 0), ((1, 0, 0), (1, 0, 0), (0, 0, 1))),
                ((0, 0), IDENTITY),
                ((0, 0, 0), ((1, 0), (0, 1))),
            ]
            for offset, rotation in invalid:
                with self.assertRaises(ValueError):
                    board.set_imu_mount(offset, rotation)
            # The native boundary validates independently of the Python wrapper.
            matrix = (ct.c_double * 9)(1, 0, 0, 0, 1, 0, 0, 0, -1)
            self.assertEqual(
                board._lib.sim_imu_mount((ct.c_double * 3)(0, 0, 0), matrix), 0
            )
            board.advance(0.001)
            with self.assertRaises(ValueError):
                board.set_imu_mount()

    def test_exact_injection_retains_chip_axes_and_owns_input(self):
        values = (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
        with FirmwareBoard(firmware=FIRMWARE) as board:
            board.set_imu_mount(
                (0.2, -0.1, 0.3), ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
            )
            board.set_imu(values[:3], values[3:], accel_us=60000, gyro_us=62500)
            with self.assertRaises(ValueError):
                board.set_imu_mount()
            board.advance(0.1)
            self.assertEqual(read_observation(board), (0, values, 60000, 62500))


class StandaloneMountTests(unittest.TestCase):
    def test_sensorless_native_defaults_and_explicit_mount_error(self):
        binary = os.environ.get("KINISI_NATIVE_BINARY")
        if not binary or not Path(binary).is_file():
            self.skipTest(
                "Set KINISI_NATIVE_BINARY to a matching newly built native host"
            )
        from kinisi_virtual_board import Schema
        from kinisi_virtual_board.firmware_build import build_firmware

        schema = Schema(FIRMWARE / "commands.json")
        request = schema.request(
            "INIT",
            42,
            sdk_type=1,
            protocol_major=schema.version[0],
            protocol_minor=schema.version[1],
            client_capabilities=2,
        )
        raw = b"F" + struct.pack("<I", len(request)) + request
        for revision in ("0.3.0", "0.3.1"):
            library, _ = build_firmware(schema, FIRMWARE, revision)
            args = [binary, "--library", str(library), "--stdio-test"]
            good = subprocess.run(args, input=raw, capture_output=True, timeout=15)
            self.assertEqual(good.returncode, 0, good.stderr.decode(errors="replace"))
            # Outer stdio length then public frame header, then model/board tuple.
            self.assertEqual(
                good.stdout[8:12], bytes([1, *map(int, revision.split("."))])
            )
            for flag in ("--imu-offset-m", "--imu-board-center-m"):
                bad = subprocess.run(
                    [*args, flag, "0,0,0"],
                    input=b"",
                    capture_output=True,
                    timeout=15,
                )
                self.assertNotEqual(bad.returncode, 0)

    def test_board_geometry_header_validates_profile(self):
        from kinisi_virtual_board.firmware_build import _imu_geometry_header

        self.assertEqual(_imu_geometry_header({}), "")
        for vector in ([1, 2], [True, 2, 3], [1.0, 2, 3], [2**31, 2, 3], None):
            with self.subTest(vector=vector), self.assertRaises(ValueError):
                _imu_geometry_header(
                    {"imu_geometry": {"underside_center_to_imu_um": vector}}
                )
        header = _imu_geometry_header(
            {"imu_geometry": {"underside_center_to_imu_um": [-15373, 26953, 2075]}}
        )
        self.assertIn("{-0.015373,0.026953,0.002075}", header)

    def test_cli_board_center_and_fixed_center_rotations(self):
        binary = os.environ.get("KINISI_NATIVE_BINARY")
        if not binary or not Path(binary).is_file():
            self.skipTest(
                "Set KINISI_NATIVE_BINARY to a matching newly built native host"
            )
        profile = json.loads((FIRMWARE / "boards/v0.4.json").read_text())
        if "imu_geometry" not in profile:
            result = subprocess.run(
                [binary, "--stdio-test", "--imu-board-center-m", "0,0,0"],
                input=b"",
                capture_output=True,
                timeout=15,
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn(b"geometry", result.stderr)
            return
        local = board_center_to_imu_m(profile)
        center = (0.03, -0.08, 0.04)
        rotations = (
            IDENTITY,
            ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
            ((1.0, 0.0, 0.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0)),
        )
        streams = []
        for rotation in rotations:
            mount = Mount.from_board_center(center, rotation, center_to_imu_m=local)
            with (
                self.subTest(rotation=rotation),
                FirmwareBoard(firmware=FIRMWARE, imu_mount=mount) as board,
            ):
                operations, expected = [], []

                def request(name, **fields):
                    data = board.schema.request(name, 42, **fields)
                    operations.append(b"F" + struct.pack("<I", len(data)) + data)
                    board.feed(data)
                    expected.append(board.drain())

                request(
                    "INIT",
                    sdk_type=1,
                    protocol_major=board.schema.version[0],
                    protocol_minor=board.schema.version[1],
                    client_capabilities=2,
                )
                request(
                    "INITIALIZE_MECANUM_PLATFORM",
                    wheels_diameter=0.1,
                    length=0.34,
                    width=0.26,
                    encoder_resolution=4096,
                )
                request("SET_PLATFORM_VELOCITY", x=20.0, y=-5.0, t=10.0)
                for _ in range(25):
                    operations.append(b"A" + struct.pack("<IQ", 8, 10000))
                    board.advance(0.01)
                    expected.append(board.drain())
                    request("GET_IMU")
            wire = b"".join(operations)
            expected_wire = b"".join(
                struct.pack("<I", len(value)) + value for value in expected
            )
            matrix = ",".join(str(v) for row in rotation for v in row)
            for flag, vector in (
                ("--imu-board-center-m", center),
                ("--imu-offset-m", mount.offset_m),
            ):
                result = subprocess.run(
                    [
                        binary,
                        "--stdio-test",
                        flag,
                        ",".join(map(str, vector)),
                        "--imu-board-to-chassis",
                        matrix,
                    ],
                    input=wire,
                    capture_output=True,
                    timeout=30,
                )
                self.assertEqual(
                    result.returncode, 0, result.stderr.decode(errors="replace")
                )
                self.assertEqual(result.stdout, expected_wire)
            streams.append(expected_wire)
        self.assertEqual(len(set(streams)), len(rotations))

    def test_cli_board_center_rejects_conflicts_and_bad_values(self):
        binary = os.environ.get("KINISI_NATIVE_BINARY")
        if not binary or not Path(binary).is_file():
            self.skipTest(
                "Set KINISI_NATIVE_BINARY to a matching newly built native host"
            )
        for flags in (
            ["--imu-board-center-m", "0,0,0", "--imu-offset-m", "0,0,0"],
            ["--imu-offset-m", "0,0,0", "--imu-board-center-m", "0,0,0"],
            ["--imu-board-center-m", "0,nan,0"],
            ["--imu-board-center-m", "0,0"],
            [
                "--imu-board-center-m",
                "0,0,0",
                "--imu-board-to-chassis",
                "1,0,0,0,1,0,0,0,-1",
            ],
        ):
            with self.subTest(flags=flags):
                result = subprocess.run(
                    [binary, "--stdio-test", *flags],
                    input=b"",
                    capture_output=True,
                    timeout=15,
                )
                self.assertNotEqual(result.returncode, 0)
                if "--imu-offset-m" in flags:
                    self.assertIn(b"mutually exclusive", result.stderr)

    def test_cli_mount_matches_python_host_wire_bytes(self):
        binary = os.environ.get("KINISI_NATIVE_BINARY")
        if not binary or not Path(binary).is_file():
            self.skipTest(
                "Set KINISI_NATIVE_BINARY to a matching newly built native host"
            )
        rotation = ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
        with FirmwareBoard(firmware=FIRMWARE) as board:
            board.set_imu_mount((0.0, -0.1, 0.05), rotation)
            schema = board.schema
            operations = []
            expected = []

            def request(name, **fields):
                data = schema.request(name, 42, **fields)
                operations.append(("F", data))
                board.feed(data)
                expected.append(board.drain())

            def advance(us):
                operations.append(("A", struct.pack("<Q", us)))
                board.advance(us / 1e6)
                expected.append(board.drain())

            request(
                "INIT",
                sdk_type=1,
                protocol_major=schema.version[0],
                protocol_minor=schema.version[1],
                client_capabilities=2,
            )
            request(
                "INITIALIZE_MECANUM_PLATFORM",
                wheels_diameter=0.1,
                length=0.34,
                width=0.26,
                encoder_resolution=4096,
            )
            advance(100000)
            request("GET_IMU")
            request("SET_PLATFORM_VELOCITY", x=20.0, y=-5.0, t=10.0)
            for _ in range(20):
                advance(10000)
                request("GET_IMU")
        raw = b"".join(
            op.encode() + struct.pack("<I", len(data)) + data for op, data in operations
        )
        result = subprocess.run(
            [
                binary,
                "--stdio-test",
                "--imu-offset-m",
                "0,-0.1,0.05",
                "--imu-board-to-chassis",
                "0,-1,0,1,0,0,0,0,1",
            ],
            input=raw,
            capture_output=True,
            timeout=30,
        )
        self.assertEqual(result.returncode, 0, result.stderr.decode(errors="replace"))
        actual = []
        stream = result.stdout
        while stream:
            (size,) = struct.unpack("<I", stream[:4])
            actual.append(stream[4 : 4 + size])
            stream = stream[4 + size :]
        self.assertEqual(actual, expected)
        for flag, value in (
            ("--imu-offset-m", "0,nan,0"),
            ("--imu-offset-m", "0,0"),
            ("--imu-board-to-chassis", "1,0,0,0,1,0,0,0,-1"),
        ):
            bad = subprocess.run(
                [binary, "--stdio-test", flag, value],
                input=b"",
                capture_output=True,
                timeout=10,
            )
            self.assertNotEqual(bad.returncode, 0)


if __name__ == "__main__":
    unittest.main()
