"""Capture public IMU regression observations through the actual firmware wire.

The native board receives only BMI088 observations and tracking commands. The
analytic reference remains outside this module's sensor replay function.
"""
import math
from pathlib import Path
import struct

from .firmware import FirmwareBoard
from .imu_regression_motion import generate_case, IMU_FIELDS, PROFILES

STATUS_FIELDS = tuple("state fault flags reserved sequence timestamp_us elapsed_us accel_age_us gyro_age_us".split())
VECTOR_FIELDS = tuple("x y yaw body_vx body_vy wz".split())
PUBLIC_FIELDS = ("public_us", *[f for f in STATUS_FIELDS if f != "reserved"], *VECTOR_FIELDS,
                 "pose_sequence", "twist_sequence", "pose_error", "twist_error", "motor_pwm_max",
                 "consumed_accel_us", "consumed_gyro_us")


class CommandError(ValueError):
    def __init__(self, command, code):
        self.command, self.code = command, code
        super().__init__(f"{command} returned firmware error {code}")


class Wire:
    """Synchronous framed requests, with strict field and reply validation."""
    def __init__(self, board):
        self.board, self.sequence = board, 0

    def ask(self, command, **fields):
        board = self.board
        unknown = fields.keys()-{f["name"] for f in board.schema.commands[command].get("properties", [])}
        if unknown:
            raise ValueError(f"Unknown request fields for {command}: {sorted(unknown)}")
        self.sequence = self.sequence % 65535+1
        board.feed(board.schema.request(command, self.sequence, **fields))
        stream, response = board.drain(), None
        while stream:
            size = stream[0]+1
            if size < 4 or size > len(stream):
                raise ValueError("Truncated firmware response frame")
            code, sequence = struct.unpack_from("<BH", stream, 1)
            payload, stream = stream[4:size], stream[size:]
            if sequence != self.sequence:
                continue
            # INIT also emits READY carrying the same request id. It is a board
            # notification, not a second INIT response (the SDK ignores it too).
            if code == board.schema.code("READY"):
                continue
            if code == board.schema.code("ERROR"):
                if len(payload) != 2 or payload[0] != board.schema.code(command):
                    raise ValueError("Unassociated firmware error")
                raise CommandError(command, payload[1])
            if code != board.schema.code(command) or response is not None:
                raise ValueError("Unexpected or duplicate firmware reply")
            response = payload
        if response is None:
            raise ValueError(f"No firmware reply to {command}")
        return response


def read_snapshot(board, wire):
    status = struct.unpack("<BBBBIQIII", wire.ask("GET_IMU_TRACKING_STATUS"))
    row = {field: None for field in PUBLIC_FIELDS}
    decoded = dict(zip(STATUS_FIELDS, status, strict=True))
    if decoded.pop("reserved"):
        raise ValueError("Nonzero reserved tracking status field")
    row.update(decoded, public_us=board.now_us, motor_pwm_max=max(map(abs, board.motor_pwm)))
    for channel in ("accel", "gyro"):
        age = row[channel+"_age_us"]
        if age != 0xffffffff:
            row["consumed_"+channel+"_us"] = board.now_us-age
    for kind, name, fields in ((0, "pose", VECTOR_FIELDS[:3]), (1, "twist", VECTOR_FIELDS[3:])):
        try:
            payload = wire.ask("GET_IMU_TRACKING_VECTOR", kind=kind)
        except CommandError as exc:
            row[name+"_error"] = exc.code
        else:
            sequence, *values = struct.unpack("<Iddd", payload)
            row[name+"_sequence"], row[name+"_error"] = sequence, 0
            row.update(zip(fields, values, strict=True))
    return row


def capture_sensor_stream(firmware, sensor_rows, metadata, *, compiler=None):
    """Replay sensor-only rows; return partial observations on native errors.

    start_us/restart_us are intentions. *_command_us are recorded only after a
    successful wire command, preventing infrastructure errors becoming assumed
    lifecycle successes. Caller-supplied metadata is copied, never mutated.
    """
    progress = dict(metadata, start_command_us=None, restart_command_us=None, deliveries=0, polls=0)
    rows = []
    try:
        for key, expected in (("start_us", 600000), ("end_us", 5120000), ("poll_us", 1000)):
            if type(metadata.get(key)) is not int or metadata[key] != expected:
                raise ValueError(f"Public replay requires {key}={expected}")
        if metadata.get("restart_us") is not None and (type(metadata["restart_us"]) is not int or metadata["restart_us"] != 3100000):
            raise ValueError("Unexpected public replay restart")
        previous = -1
        for row in sensor_rows:
            if set(row) != set(IMU_FIELDS):
                raise ValueError("Replay accepts only the nine sensor fields")
            if any(type(row[k]) is not int or row[k] < 0 for k in IMU_FIELDS[:3]):
                raise ValueError("Sensor timestamps must be unsigned integers")
            if row["delivered_us"] <= previous or max(row["accel_us"], row["gyro_us"]) > row["delivered_us"]:
                raise ValueError("Noncausal/unordered sensor delivery")
            if any(type(row[k]) not in (int,float) or not math.isfinite(row[k]) for k in IMU_FIELDS[3:]):
                raise ValueError("Nonfinite sensor measurement")
            previous = row["delivered_us"]
        if not sensor_rows or sensor_rows[0]["delivered_us"] < metadata["start_us"] or previous > metadata["end_us"]:
            raise ValueError("Sensor stream outside the observation window")
        with FirmwareBoard(firmware=Path(firmware), compiler=compiler) as board:
            progress["build_id"] = board.build["build_id"]
            # Own the sensor before any scheduler advance; do not calibrate on
            # the board's unrelated default stationary sensor model.
            board.set_imu(status=2, accel_us=0, gyro_us=0)
            board.advance(metadata["start_us"]/1e6)
            wire = Wire(board)
            major, minor, patch = board.schema.version
            init = wire.ask("INIT", sdk_type=1, protocol_major=major, protocol_minor=minor, client_capabilities=2)
            expected_identity = [1, 0, 4, 0, major, minor, patch]
            if list(init[:7]) != expected_identity:
                raise ValueError(f"Board/protocol identity {list(init[:7])} differs from {expected_identity}")
            wire.ask("SET_IMU_TRACKING", action=0, stationary_mount_confirmed=1, mounting_yaw_rad=math.pi)
            progress["start_command_us"] = board.now_us
            yaw, window = struct.unpack("<dI", wire.ask("GET_IMU_TRACKING_CONFIGURATION"))
            if yaw != math.pi or window != 1000000:
                raise ValueError("Applied tracking configuration differs")
            poll, index = metadata["start_us"], 0
            while poll <= metadata["end_us"]:
                due = sensor_rows[index]["delivered_us"] if index < len(sensor_rows) else poll
                moment = min(poll, due)
                board.advance((moment-board.now_us)/1e6)
                if board.now_us != moment:
                    raise ValueError("Native scheduler differs from replay time")
                if moment == metadata["restart_us"] and progress["restart_command_us"] is None:
                    wire.ask("SET_IMU_TRACKING", action=3, stationary_mount_confirmed=1, mounting_yaw_rad=math.pi)
                    progress["restart_command_us"] = moment
                if index < len(sensor_rows) and moment == due:
                    sample = sensor_rows[index]
                    board.set_imu(tuple(sample[k] for k in ("ax","ay","az")),
                                  tuple(sample[k] for k in ("gx","gy","gz")),
                                  accel_us=sample["accel_us"], gyro_us=sample["gyro_us"])
                    index += 1
                    progress["deliveries"] = index
                if moment == poll:
                    rows.append(read_snapshot(board, wire))
                    progress["polls"] = len(rows)
                    poll += metadata["poll_us"]
            if index != len(sensor_rows):
                raise ValueError("Undelivered sensor events after observation tail")
    except Exception as exc:
        progress["error"] = f"{type(exc).__name__}: {exc}"
    return dict(metadata=progress, public_rows=rows)


def capture_case(firmware, *, profile, seed, delay_us=10000, compiler=None):
    """Create a disclosed analytic stimulus and run an explicit native checkout."""
    case = generate_case(profile=profile, seed=seed, delay_us=delay_us)
    replay = capture_sensor_stream(firmware, case["sensor_rows"], case["metadata"], compiler=compiler)
    return dict(case, **replay)
