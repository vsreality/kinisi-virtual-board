"""Actual firmware-controlled chassis sources for the disclosed public matrix.

Forty-five source groups feed all144 scheduled cases. Shaft travel drives an
independent Chassis; its specific force drives BMI088(seed+10000). Diagnostic
capture receives only delayed sensor observations, never this reference.
"""
from collections import deque
import math
from pathlib import Path
import random
import struct

from .bmi088 import BMI088, G
from .chassis import Chassis
from .firmware import FirmwareBoard
from .imu_regression_capture import Wire
from .imu_regression_motion import IMU_FIELDS, REFERENCE_FIELDS
from .imu_qualification_plan import (make_plan, source_key, schedule, GEOMETRY,
    WHEEL_PID, ENCODER_RESOLUTION, FAULTS, DELAYS, DESIGN_VERSION)

START_US, END_US, STEP_US = 600000, 5120000, 2500


def _case(case):
    preset, role, seed = source_key(case)
    program, direction = schedule(preset, role, seed)
    if case["role"] not in (role, *FAULTS) or case["delay_us"] not in DELAYS:
        raise ValueError("Unknown public source/fault variant")
    if case["targets_ms"] != program or case["direction_body_xy"] != direction:
        raise ValueError("Changed public target recipe")
    if case["source_id"] != f"{preset}-{role}-{seed:08x}":
        raise ValueError("Source identity differs from planned group")
    return preset, role, seed


def _configure(board, wire, preset):
    major, minor, patch = board.schema.version
    identity = wire.ask("INIT",sdk_type=1,protocol_major=major,protocol_minor=minor,client_capabilities=2)
    if list(identity[:7]) != [1,0,4,0,major,minor,patch]:
        raise ValueError("Motion board/protocol identity mismatch")
    geometry = dict(GEOMETRY[preset])
    request = dict(geometry)
    if preset != "differential":
        request["wheels_diameter"] = request.pop("wheel_diameter")
    wire.ask(f"INITIALIZE_{preset.upper()}_PLATFORM",**request,encoder_resolution=ENCODER_RESOLUTION)
    wheels = {"differential":2,"omni":3,"mecanum":4}[preset]
    expected = [(0,0,wheels-1),(1,0,geometry["wheel_diameter"]),
                (2,0,geometry.get("wheel_base",geometry.get("length",0.))),
                (3,0,geometry.get("robot_radius",geometry.get("width",0.)))]
    for wheel in range(wheels):
        expected.extend(((4,wheel,ENCODER_RESOLUTION),(5,wheel,0),(6,wheel,0)))
    for field,index,value in expected:
        actual, = struct.unpack("<d",wire.ask("GET_CONFIGURATION_FIELD",field=field,index=index))
        if actual != value:
            raise ValueError(f"Motion geometry field {field}[{index}] differs: {actual} != {value}")
    wire.ask("START_PLATFORM_CONTROLLER",**WHEEL_PID)
    wire.ask("START_PLATFORM_ODOMETRY")
    return expected


class _Sensor:
    def __init__(self, seed):
        self.model = BMI088(seed+10000, temperature_c=25.)
        self.pending = deque()
        self.acquired = (0,0)
        self.stamps = [0,0]
        self.values = list(self.model.values)

    def step(self, at_us, force, rate):
        self.model.advance(at_us, body_accel=force, body_gyro=(0.,0.,rate))
        stamps = (self.model.accel_us,self.model.gyro_us)
        for channel,stamp in enumerate(stamps):
            if stamp != self.acquired[channel]:
                self.pending.append((stamp+10000,channel,stamp,tuple(self.model.values[3*channel:3*channel+3])))
        self.acquired = stamps
        changed = False
        while self.pending and self.pending[0][0] <= at_us:
            _,channel,stamp,values = self.pending.popleft()
            self.stamps[channel] = stamp
            self.values[3*channel:3*channel+3] = values
            changed = True
        return changed

    def deliver(self, board):
        board.set_imu(self.values[:3],self.values[3:],accel_us=self.stamps[0],gyro_us=self.stamps[1])


def source_for_case(firmware, case, *, compiler=None):
    """Acquire one shared source; preserve partial readings/targets on failure.

    firmware is the explicit motion-producer checkout, independent from the
    diagnostic candidate used by capture_sensor_stream. There are no file writes.
    """
    preset, role, seed = _case(case)
    rng = random.Random(seed)
    speed, motor_tau, body_tau = rng.uniform(9.4,10.6), rng.uniform(.045,.055), rng.uniform(.075,.105)
    chassis = Chassis(preset,GEOMETRY[preset],(body_tau,body_tau,body_tau*.8))
    sensor = _Sensor(seed)
    reference, events, targets = [], [], []
    metadata = dict(source_id=case["source_id"],preset=preset,role=role,seed=seed,sensor_seed=seed+10000,
                    design_version=DESIGN_VERSION,max_speed_rad_s=speed,motor_tau_s=motor_tau,
                    body_tau_s=list(chassis.response_s))
    try:
        with FirmwareBoard(firmware=Path(firmware),compiler=compiler,max_speed=speed,time_constant=motor_tau) as board:
            metadata["build_id"] = board.build["build_id"]
            sensor.deliver(board)  # Explicit ownership before the first task poll.
            wire = Wire(board)
            metadata["configuration_readback"] = _configure(board,wire,preset)
            program = {START_US+1000*t:values for t,values in case["targets_ms"]}
            while board.now_us < END_US:
                if board.now_us in program:
                    x,y,yaw = program[board.now_us]
                    wire.ask("SET_PLATFORM_TARGET_VELOCITY",x=x,y=y,t=yaw)
                    targets.append(dict(at_us=board.now_us,x=x,y=y,yaw=yaw))
                previous = tuple(board.motor_angle)
                board.advance(STEP_US/1e6)
                delta = [end-start for end,start in zip(board.motor_angle,previous,strict=True)]
                force = chassis.step(delta,(1.,)*4,STEP_US/1e6)
                changed = sensor.step(board.now_us,force,chassis.twist[2])
                if changed:
                    sensor.deliver(board)
                    if START_US+10000 <= board.now_us <= 5112500:
                        events.append(dict(zip(IMU_FIELDS,(board.now_us,*sensor.stamps,*sensor.values),strict=True)))
                if board.now_us >= START_US-20000:
                    reference.append(dict(zip(REFERENCE_FIELDS,(board.now_us,*chassis.pose,*chassis.twist),strict=True)))
                # Exercise the production sensor read on the recorder's10ms
                # grid, as the motion source does, independently of reference.
                if board.now_us >= START_US and (board.now_us-START_US)%10000 == 0:
                    measured = struct.unpack("<6d",wire.ask("GET_IMU"))
                    if measured != tuple(sensor.values):
                        raise ValueError("Firmware IMU read differs from delivered modeled register")
            if len(events) != 902 or len(reference) != 1817:
                raise ValueError("Incomplete motion source clocks")
            expected = [dict(at_us=START_US+1000*t,x=v[0],y=v[1],yaw=v[2]) for t,v in case["targets_ms"]]
            if targets != expected:
                raise ValueError("Executed targets differ from the declared schedule")
    except Exception as exc:
        metadata["error"] = f"{type(exc).__name__}: {exc}"
    return dict(metadata=metadata,reference_rows=reference,sensor_rows=events,targets=targets)


def capture_inputs(source, case):
    """Derive an independently copied phase/fault stream from the shared source."""
    preset,role,seed = _case(case)
    if any(source["metadata"].get(key) != value for key,value in
           (("source_id",case["source_id"]),("preset",preset),("role",role),("seed",seed))):
        raise ValueError("Wrong shared source for case")
    nominal = [dict(row) for row in source["sensor_rows"]]
    actual = [dict(row,delivered_us=row["delivered_us"]+case["delay_us"]-10000) for row in nominal]
    for index,row in enumerate(actual):
        slot,gyro = divmod(index,2)
        if case["role"] in ("producer_freeze","accel_freeze") and 80 <= slot < 110:
            for key in ("accel_us","ax","ay","az"):
                row[key] = nominal[158][key]
        if case["role"] == "producer_freeze" and 80 <= slot-(1-gyro) < 110:
            for key in ("gyro_us","gx","gy","gz"):
                row[key] = nominal[159][key]
        if case["role"] == "gyro_reorder" and slot-(1-gyro) == 80:
            for key in ("gyro_us","gx","gy","gz"):
                row[key] = nominal[153][key]
    metadata = dict(profile=case["role"],preset=preset,seed=seed,delay_us=case["delay_us"],
                    start_us=START_US,restart_us=3100000 if role=="stationary" else None,
                    end_us=END_US,poll_us=1000,source_id=case["source_id"],
                    source_build_id=source["metadata"].get("build_id"))
    if source["metadata"].get("error"):
        metadata["source_error"] = source["metadata"]["error"]
    return dict(metadata=metadata,sensor_rows=actual,nominal_sensor_rows=nominal,
                reference_rows=[dict(row) for row in source["reference_rows"]],
                targets=[dict(row) for row in source["targets"]])
