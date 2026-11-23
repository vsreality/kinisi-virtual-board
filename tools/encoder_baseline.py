"""Collect encoder-only firmware traces. Score them offline with score_encoder_baseline.py."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import random
import shutil
import struct
import subprocess
import sys
from collections import deque
from contextlib import ExitStack
from itertools import product

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from kinisi_virtual_board import FirmwareBoard
from kinisi_virtual_board.chassis import Chassis
from kinisi_virtual_board.bmi088 import BMI088, MODEL_ID
from kinisi_virtual_board.firmware_build import DEFAULT_FIRMWARE

CASES = Path(__file__).with_name("encoder_baseline_cases.json")
AXES = ("x", "y", "yaw")
IMU_DELAY_US = 10000
INTEGRATION_US = 2500


class DelayedIMU:
    """Independent register events delivered causally after fixed transport delay."""
    def __init__(self, seed, temperature_c=25.0):
        self.model = BMI088(seed, temperature_c=temperature_c)
        self.pending = deque()
        self.source_stamps = [0, 0]
        self.stamps = [0, 0]
        self.values = list(self.model.values)

    def advance(self, at_us, acceleration, angular_velocity):
        self.model.advance(at_us, body_accel=acceleration, body_gyro=angular_velocity)
        for channel, stamp in enumerate((self.model.accel_us, self.model.gyro_us)):
            if stamp != self.source_stamps[channel]:
                self.pending.append((stamp + IMU_DELAY_US, channel, stamp,
                                     tuple(self.model.values[channel*3:channel*3+3])))
                self.source_stamps[channel] = stamp
        # Integration stops at each independent source event, so queue order is causal.
        changed = False
        while self.pending and self.pending[0][0] <= at_us:
            _, channel, stamp, values = self.pending.popleft()
            self.stamps[channel] = stamp
            self.values[channel*3:channel*3+3] = values
            changed = True
        return changed

    def deliver(self, board):
        board.set_imu(self.values[:3], self.values[3:], accel_us=self.stamps[0], gyro_us=self.stamps[1])


class Wire:
    def __init__(self, board):
        self.board = board
        self.seq = 0

    def ask(self, command, **fields):
        self.seq += 1
        board = self.board
        unknown = fields.keys() - {f["name"] for f in board.schema.commands[command].get("properties", [])}
        if unknown:
            raise ValueError(f"Unknown {command} fields: {sorted(unknown)}")
        board.feed(board.schema.request(command, self.seq, **fields))
        stream = board.drain()
        while stream:
            size = stream[0] + 1
            if size < 4 or size > len(stream):
                raise ValueError("Truncated firmware response")
            code, seq = struct.unpack_from("<BH", stream, 1)
            payload, stream = stream[4:size], stream[size:]
            if seq != self.seq:
                continue
            if code == board.schema.code(command):
                return payload
            if code == board.schema.code("ERROR"):
                raise ValueError(f"{command} rejected at {board.now_us} us: {payload.hex()}")
        raise ValueError(f"No reply to {command}")


def dump(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def revision(path):
    return subprocess.check_output(["git", "-C", str(path), "rev-parse", "HEAD"], text=True).strip()


def snapshot_changes(repo, paths, destination):
    git = ["git", "-C", str(repo)]
    untracked = subprocess.check_output([*git, "ls-files", "--others", "--exclude-standard", "--", *paths])
    untracked_source = [p for p in untracked.decode().splitlines() if Path(p).suffix in (".c", ".h", ".py", ".json")]
    if untracked_source:
        raise ValueError(f"Stage untracked source for reproducible capture: {untracked_source}")
    patch = subprocess.check_output([*git, "diff", "--binary", "HEAD", "--", *paths])
    destination.write_bytes(patch)
    return hashlib.sha256(patch).hexdigest()


def writer(stack, directory, name, header):
    file = stack.enter_context((directory / name).open("w", newline="", encoding="utf-8"))
    output = csv.writer(file)
    output.writerow(header)
    return output


def configure(wire, spec, preset, mode):
    identity = wire.ask("INIT", sdk_type=1, protocol_major=2,
                        protocol_minor=wire.board.schema.version[1], client_capabilities=2)
    if identity[:4] != bytes([1, 0, 4, 0]):
        raise ValueError(f"Unexpected board identity {identity.hex()}")
    geometry = spec["presets"][preset]
    request = dict(geometry)
    if preset != "differential":
        request["wheels_diameter"] = request.pop("wheel_diameter")
    wire.ask(f"INITIALIZE_{preset.upper()}_PLATFORM", **request,
             encoder_resolution=spec["encoder_counts_per_rev"])
    # Read effective release configuration back across the wire. A missing field
    # can otherwise select a valid default for a physically different chassis.
    wheels = {"differential": 2, "omni": 3, "mecanum": 4}[preset]
    expected = [(0, 0, wheels-1), (1, 0, geometry["wheel_diameter"]),
                (2, 0, geometry.get("wheel_base", geometry.get("length", 0.))),
                (3, 0, geometry.get("robot_radius", geometry.get("width", 0.)))]
    for index in range(wheels):
        expected.extend([(4, index, spec["encoder_counts_per_rev"]), (5, index, 0), (6, index, 0)])
    readback = []
    for field, index, value in expected:
        observed, = struct.unpack("<d", wire.ask("GET_CONFIGURATION_FIELD", field=field, index=index))
        if observed != value:
            raise ValueError(f"{preset} field {field}[{index}] returned {observed}, expected {value}")
        readback.append([field, index, observed])
    wire.ask("START_PLATFORM_CONTROLLER", **spec["wheel_pid"])
    if mode == "position":
        wire.ask("INITIALIZE_PLATFORM_POSITION_CONTROLLER", **spec["position_pid"])
    else:
        wire.ask("START_PLATFORM_ODOMETRY")
    rates = {}
    for subsystem in ("CONTROLLER", "ODOMETRY"):
        rates[subsystem.lower() + "_hz"], = struct.unpack("<H", wire.ask(f"GET_{subsystem}_FREQUENCY"))
    return rates, readback


def run_case(directory, firmware, spec, preset, mode, surface, seed, *, setup=None, stationary_warmup=False,
             temperature_c=25.0):
    """Call setup(ask) after configuration, then optionally feed stationary IMU samples before the course."""
    directory.mkdir()
    rng = random.Random(seed)
    speed = rng.uniform(9.4, 10.6)
    motor_tau = rng.uniform(.045, .055)
    body_tau = rng.uniform(.075, .105)
    chassis = Chassis(preset, spec["presets"][preset], (body_tau, body_tau, body_tau * .8))
    imu = DelayedIMU(seed + 10000, temperature_c)
    dt_ms = spec["step_ms"]
    if type(dt_ms) is not int or dt_ms * 1000 % INTEGRATION_US or dt_ms <= 0:
        raise ValueError("Recorder period must be a positive whole multiple of 2.5 ms")
    recipe = spec["courses"][mode]
    events = dict(recipe["targets"])
    slip_intervals = spec["surfaces"][surface]
    with FirmwareBoard(firmware=firmware, revision=spec["revision"], max_speed=speed,
                       time_constant=motor_tau) as board, ExitStack() as files:
        imu.deliver(board)  # Select explicit producer ownership before any firmware task runs.
        wire = Wire(board)
        rates, readback = configure(wire, spec, preset, mode)
        if setup is not None:
            setup(wire.ask)
        # Advance the same producer from boot in both modes. Only early delivery is optional.
        # Keep pre-origin reference rows for the gyro's independent acquisition phase.
        warm_reference = []
        while board.now_us < 600000:
            board.advance(INTEGRATION_US / 1e6)
            changed = imu.advance(board.now_us, (0., 0., 9.80665), (0., 0., 0.))
            if changed and stationary_warmup:
                imu.deliver(board)
            if board.now_us >= 580000:
                warm_reference.append([board.now_us, *chassis.pose, *chassis.twist])
        origin = board.now_us
        metadata = dict(preset=preset, mode=mode, surface=surface, seed=seed, origin_us=origin,
                        max_speed_rad_s=speed, motor_tau_s=motor_tau, body_tau_s=chassis.response_s,
                        imu_bias=imu.model.bias, rates=rates, configuration_readback=readback, firmware_build_id=board.build["build_id"],
                        schema_sha256=board.schema.sha256, duration_ms=recipe["duration_ms"])
        metadata["imu_model"] = dict(id=MODEL_ID, seed=seed + 10000, temperature_c=temperature_c,
            source_sha256=hashlib.sha256((ROOT/"kinisi_virtual_board/bmi088.py").read_bytes()).hexdigest(),
            source_period_us=10000, accel_phase_us=0, gyro_phase_us=2500,
            transport_delay_us=IMU_DELAY_US, integration_us=INTEGRATION_US)
        metadata["reference_start_us"] = warm_reference[0][0]
        if stationary_warmup:
            metadata["stationary_warmup_ms"] = 600
        if setup is not None:
            metadata["client_setup"] = True
        dump(directory/"case.json", metadata)
        if not (directory.parent/"firmware-build.json").exists():
            dump(directory.parent/"firmware-build.json", board.build)
        obs = writer(files, directory, "sensors.csv",
                     ["at_us", "accel_us", "gyro_us", *[f"enc{i}" for i in range(4)],
                      "ax", "ay", "az", "gx", "gy", "gz"])
        est = writer(files, directory, "firmware.csv",
                     ["at_us", "odom_us", "clock_mode", "clock_quality", *AXES, "controller_dispatches",
                      *[f"pwm{i}" for i in range(4)]])
        truth = writer(files, directory, "reference.csv", ["at_us", *AXES, "vx", "vy", "wz"])
        commands = writer(files, directory, "targets.csv", ["at_us", *AXES])
        truth.writerows(warm_reference)

        def record():
            # Expose only delayed, biased, noisy and quantized sensor measurements to the board.
            imu.deliver(board)
            measured = struct.unpack("<6d", wire.ask("GET_IMU"))
            obs.writerow([board.now_us, *imu.stamps, *board.encoder_counts, *measured])
            pose = struct.unpack("<QBB3d", wire.ask("GET_PLATFORM_ODOMETRY"))
            est.writerow([board.now_us, *pose, board.controller_dispatches, *board.motor_pwm])

        record()
        for elapsed in range(0, recipe["duration_ms"], dt_ms):
            if elapsed in events:
                x, y, yaw = events[elapsed]
                if preset == "differential" and mode == "velocity":
                    y = 0.  # No direct lateral velocity command on this chassis.
                command = "SET_PLATFORM_POSITION" if mode == "position" else "SET_PLATFORM_TARGET_VELOCITY"
                wire.ask(command, x=x, y=y, t=yaw)
                commands.writerow([board.now_us, x, y, yaw])
            traction = next((v for start, stop, v in slip_intervals if start <= elapsed < stop), (1.,)*4)
            for _ in range(dt_ms * 1000 // INTEGRATION_US):
                previous_angle = tuple(board.motor_angle)
                board.advance(INTEGRATION_US / 1e6)
                delta = [end-start for end, start in zip(board.motor_angle, previous_angle)]
                specific_force = chassis.step(delta, traction, INTEGRATION_US / 1e6)
                changed = imu.advance(board.now_us, specific_force, (0., 0., chassis.twist[2]))
                if changed:
                    imu.deliver(board)
                truth.writerow([board.now_us, *chassis.pose, *chassis.twist])
            record()
        if board.now_us != origin + recipe["duration_ms"] * 1000:
            raise ValueError("Incomplete simulated duration")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True, type=Path, help="New directory for immutable run traces")
    parser.add_argument("--firmware", type=Path, default=DEFAULT_FIRMWARE)
    parser.add_argument("--cases", type=Path, default=CASES)
    args = parser.parse_args()
    spec = json.loads(args.cases.read_text(encoding="utf-8"))
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    fw = args.firmware.resolve()
    shutil.copyfile(args.cases, out/"cases.json")
    simulator_paths = ["kinisi_virtual_board", "tools/encoder_baseline.py", "tools/score_encoder_baseline.py",
                       "tools/encoder_baseline_cases.json"]
    firmware_paths = ["include", "src", "lib", "boards", "test", "tools", "commands.json"]
    sim_patch = snapshot_changes(ROOT, simulator_paths, out/"simulator.patch")
    fw_patch = snapshot_changes(fw, firmware_paths, out/"firmware.patch")
    meta = dict(firmware_revision=revision(fw), simulator_revision=revision(ROOT),
                firmware_path=str(fw), python=sys.version, platform=sys.platform,
                simulator_patch_sha256=sim_patch, firmware_patch_sha256=fw_patch,
                cases_sha256=hashlib.sha256(args.cases.read_bytes()).hexdigest())
    dump(out/"manifest.json", meta)
    runs = list(product(spec["presets"], spec["courses"], spec["surfaces"], spec["seeds"]))
    # Reexecute a fixed case in a fresh firmware instance to check deterministic replay.
    failures = []
    for index, (preset, mode, surface, seed) in enumerate([*runs, runs[0]]):
        name = f"{preset}-{mode}-{surface}-{seed}"
        if index == len(runs):
            name += "-repeat"
        try:
            run_case(out/name, fw, spec, preset, mode, surface, seed)
        except Exception as exc:
            failures.append((name, repr(exc)))
            dump(out/name/"invalid.json", {"error": repr(exc)})
    dump(out/"collection.json", {"planned": len(runs), "repeat": 1, "failures": failures})
    print(f"Collected {len(runs)} planned cases and one repeat; {len(failures)} invalid.")
    return int(bool(failures))


if __name__ == "__main__":
    raise SystemExit(main())
