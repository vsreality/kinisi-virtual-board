"""Deterministic stateful protocol model, not STM32 firmware or electrical simulation.

All time comes from advance(). Transports serialize calls and supply real elapsed
time; tests can advance manually. Unknown commands never receive a fake ACK.
"""
from dataclasses import dataclass, field
import json
import math
from pathlib import Path

from .bmi088 import BMI088
from .schema import Rejected, Schema

MOTOR_COMMANDS = set("""INITIALIZE_MOTOR SET_MOTOR_SPEED STOP_MOTOR BRAKE_MOTOR
INITIALIZE_MOTOR_CONTROLLER SET_MOTOR_TARGET_SPEED RESET_MOTOR_CONTROLLER
GET_MOTOR_CONTROLLER_STATE DELETE_MOTOR_CONTROLLER INITIALIZE_MOTOR_POSITION_CONTROLLER
INITIALIZE_MOTOR_POSITION_PID_CONTROLLER RESET_MOTOR_POSITION SET_MOTOR_POSITION GET_MOTOR_POSITION""".split())
ENCODER_COMMANDS = set("""INITIALIZE_ENCODER GET_ENCODER_VALUE START_ENCODER_ODOMETRY
RESET_ENCODER_ODOMETRY STOP_ENCODER_ODOMETRY GET_ENCODER_ODOMETRY""".split())
PLATFORM_COMMANDS = set("""INITIALIZE_MECANUM_PLATFORM INITIALIZE_OMNI_PLATFORM
INITIALIZE_DIFFERENTIAL_PLATFORM SET_PLATFORM_VELOCITY START_PLATFORM_CONTROLLER
SET_PLATFORM_TARGET_VELOCITY GET_PLATFORM_CURRENT_VELOCITY STOP_PLATFORM_CONTROLLER
START_PLATFORM_ODOMETRY RESET_PLATFORM_ODOMETRY STOP_PLATFORM_ODOMETRY GET_PLATFORM_ODOMETRY
BRAKE_PLATFORM COAST_PLATFORM INITIALIZE_PLATFORM_POSITION_CONTROLLER
INITIALIZE_PLATFORM_POSITION_PID_CONTROLLER RESET_PLATFORM_POSITION SET_PLATFORM_POSITION""".split())
OTHER_COMMANDS = set("""INIT TIME_SYNC_RESPONSE SET_TIME_SYNC_INTERVAL GET_TIME_STATUS PING
SET_HEARTBEAT_CONFIG GET_HEARTBEAT_CONFIG SUBSCRIBE_ODOMETRY UNSUBSCRIBE_ODOMETRY
SUBSCRIBE UNSUBSCRIBE POLL_TELEMETRY GET_TEMPERATURE GET_IMU SET_CONTROLLER_FREQUENCY
GET_CONTROLLER_FREQUENCY SET_ODOMETRY_FREQUENCY GET_ODOMETRY_FREQUENCY INITIALIZE_GPIO_PIN
SET_GPIO_PIN_STATE GET_GPIO_PIN_STATE TOGGLE_GPIO_PIN_STATE SET_STATUS_LED_STATE
TOGGLE_STATUS_LED_STATE""".split())
SUPPORTED = MOTOR_COMMANDS | ENCODER_COMMANDS | PLATFORM_COMMANDS | OTHER_COMMANDS
SAFE = {"INIT", "STOP_MOTOR", "BRAKE_MOTOR", "COAST_PLATFORM", "BRAKE_PLATFORM", "STOP_PLATFORM_CONTROLLER"}


@dataclass
class Motor:
    initialized: bool = False
    controller: bool = False
    encoder: int = 0
    speed: float = 0.0
    angle: float = 0.0
    origin: float = 0.0
    pwm: float = 0.0
    gains: dict = field(default_factory=dict)
    position: dict | None = None
    target: float | None = None
    mode: str = "coast"

    def stop(self, mode="coast"):
        self.speed = self.pwm = 0.0
        self.controller = False
        self.position = self.target = None
        self.mode = mode


@dataclass
class Encoder:
    initialized: bool = False
    resolution: float = 1000.0
    angle: float = 0.0
    origin: float = 0.0
    running: bool = False
    sample_after: int = 0


class VirtualBoard:
    def __init__(self, commands, board_profile=None, *, revision="0.4.0", pwm_speed=10.0, imu_seed=1,
                 temperature_c=25.0, imu_mount=None):
        self.schema = commands if isinstance(commands, Schema) else Schema(commands)
        self.profile = json.loads(Path(board_profile).read_text()) if board_profile else None
        if self.profile:
            revision = self.profile["board"]["revision"]
        self.revision = tuple(map(int, revision.split(".")))
        if self.revision not in ((0, 3, 0), (0, 4, 0)) and not self.profile:
            raise ValueError("Supply a board profile for an unknown hardware revision")
        self.sensor_counts = {
            "temperature": len(self.profile.get("temperature_sensors", [])) if self.profile else int(self.revision == (0, 4, 0)),
            "imu": len(self.profile.get("imu_sensors", [])) if self.profile else int(self.revision == (0, 4, 0)),
        }
        self.pwm_speed = pwm_speed
        self.motors = [Motor(encoder=i) for i in range(4)]
        self.encoders = [Encoder() for _ in range(4)]
        self.gpio = [0] * 10
        self.gpio_modes = [None] * 10
        self.led = 0
        self.controller_frequency = 10
        self.odometry_frequency = 10
        self.platform = None
        self.platform_controller = False
        self.platform_odometry = False
        self.platform_sample_after = 0
        self.pose = [0.0] * 3
        self.velocity = [0.0] * 3
        self.platform_position = None
        self.platform_target = None
        self.temperature = 25.0
        self.imu = dict(zip((p["name"] for p in self.schema.objects["imu_sample"]), (0., 0., 9.80665, 0., 0., 0.)))
        has_imu = self.revision == (0, 4, 0) and self.sensor_counts["imu"]
        if imu_mount is not None and not has_imu:
            raise ValueError("A physical IMU mount requires a board with a BMI088")
        self.imu_model = BMI088(imu_seed, temperature_c=temperature_c, mount=imu_mount) if has_imu else None
        self._imu_generated = dict(self.imu)
        self._imu_override = False
        self._imu_velocity = (0., 0., 0.)
        self.sensor_errors = {}
        self.now_us = 0
        self.initialized = self.ready = self.lost = False
        self.heartbeat_enabled = False
        self.heartbeat_timeout_ms = 500
        self.last_activity_us = 0
        self.clock_mode = 0
        self.clock_offset = 0
        self.sync_interval_ms = 30000
        self.last_sync_us = 0
        self.sync_pending = None
        self.sync_sequence = 0
        self.init_id = 0
        self.subscriptions = {}
        self._input = bytearray()
        self._output = bytearray()
        self._drop = {}
        self._errors = {}

    @property
    def unsupported_commands(self):
        return sorted(c["command"] for c in self.schema.commands.values()
                      if c["direction"] == "client_to_controller" and c["command"] not in SUPPORTED)

    @property
    def imu(self):
        return self._imu

    @imu.setter
    def imu(self, values):
        self._imu = values
        self._imu_override = True

    def drop_next_reply(self, command, count=1):
        """Execute requests normally but lose their next correlated responses."""
        self._drop[command] = count

    def fail_next(self, command, error="INTERNAL_ERROR"):
        """Reject the next command before it changes simulated state."""
        if error not in self.schema.errors:
            raise ValueError(error)
        self._errors[command] = error

    def set_imu(self, acceleration=(0., 0., 9.80665), angular_velocity=(0., 0., 0.)):
        """Override sensor output in native axes and SI units for protocol tests.

        Direct changes to the historical public ``imu`` dictionary also latch an
        override at the next advance, retaining existing fixture behavior.
        """
        acceleration, angular_velocity = tuple(acceleration), tuple(angular_velocity)
        values = acceleration + angular_velocity
        if len(acceleration) != 3 or len(angular_velocity) != 3 or not all(math.isfinite(v) for v in values):
            raise ValueError("IMU acceleration and angular velocity need three finite values each")
        self.imu = dict(zip(self.imu, values))
        self._imu_override = True

    def _advance_imu(self, at_us, dt):
        if self.imu_model is None:
            return
        if self.imu != self._imu_generated:
            self._imu_override = True
        if self._imu_override:
            return
        vx, vy, omega = self.velocity
        old_vx, old_vy, _ = self._imu_velocity
        # Planar chassis motion only: body velocity derivative plus rotation.
        # This behavioral model has no suspension, tilt, vibration, or wheel slip.
        accel = ((vx-old_vx)/dt - omega*vy, (vy-old_vy)/dt + omega*vx, 9.80665)
        self.imu_model.advance(at_us, body_accel=accel, body_gyro=(0., 0., omega))
        self.imu.update(zip(self.imu, self.imu_model.values))
        self._imu_generated = dict(self.imu)
        self._imu_velocity = tuple(self.velocity)

    def drain(self, size=None):
        size = len(self._output) if size is None else size
        data = bytes(self._output[:size])
        del self._output[:size]
        return data

    def _emit(self, name, message_id, payload=b""):
        self._output.extend(self.schema.frame(name, message_id, payload))

    def feed(self, data):
        """Consume fragmented or concatenated length-prefixed frames."""
        self._input.extend(data)
        while self._input and len(self._input) >= self._input[0] + 1:
            size = self._input[0]
            request = bytes(self._input[1:size + 1])
            del self._input[:size + 1]
            self._dispatch(request)

    def _dispatch(self, request):
        code = request[0] if request else 0
        message_id = int.from_bytes(request[1:3], "little") if len(request) >= 3 else 0
        command = self.schema.by_code.get(code)
        name = command["command"] if command else None
        drop = self._drop.get(name, 0) > 0
        if drop:
            self._drop[name] -= 1
        start = len(self._output)
        try:
            if len(request) < 3 or len(request) > 254:
                raise Rejected("INVALID_LENGTH")
            if not command or command["direction"] != "client_to_controller" or name not in SUPPORTED:
                raise Rejected("UNKNOWN_COMMAND")
            args = self.schema.unpack(command, request[3:])
            if not message_id:
                raise Rejected("INVALID_ARGUMENT")
            if self.lost and name not in SAFE:
                raise Rejected("INIT_REQUIRED")
            if name in self._errors:
                raise Rejected(self._errors.pop(name))
            self.last_activity_us = self.now_us
            if name == "TIME_SYNC_RESPONSE":
                self._sync_response(message_id, args)
                return
            result = self._execute(name, args)
            fields = self.schema.response_fields(command)
            if fields and not isinstance(result, dict):
                result = {fields[0]["name"]: result}
            self._emit(name, message_id, self.schema.pack(fields, result or {}))
            if name == "INIT":
                self.init_id = message_id
                if self.clock_mode:
                    self._request_sync()
                else:
                    self.ready = True
                    self._emit("READY", message_id, bytes([0]))
        except Rejected as error:
            self._emit("ERROR", message_id, bytes([code, self.schema.errors[str(error)]]))
        if drop:
            del self._output[start:]

    def _require_ready(self):
        if not self.initialized:
            raise Rejected("INIT_REQUIRED")
        if not self.ready:
            raise Rejected("CLOCK_NOT_READY")

    def _request_sync(self):
        self.sync_sequence = self.sync_sequence % 65535 + 1
        self.sync_pending = (self.sync_sequence, self.now_us)
        self._emit("TIME_SYNC_REQUEST", self.sync_sequence)

    def _sync_response(self, message_id, args):
        receive, send = args["host_receive_us"], args["host_send_us"]
        if not self.sync_pending or message_id != self.sync_pending[0] or not receive or send < receive:
            raise Rejected("INVALID_ARGUMENT")
        # Ideal link: no drift, one exchange. Firmware uses a multi-sample RTT filter.
        self.clock_offset = (receive + send) // 2 - (self.sync_pending[1] + self.now_us) // 2
        self.last_sync_us = self.now_us
        self.sync_pending = None
        if not self.ready:
            self.ready = True
            self._emit("READY", self.init_id, bytes([1]))

    def _metadata(self):
        self._require_ready()
        return {"timestamp_us": max(0, self.now_us + self.clock_offset),
                "clock_mode": self.clock_mode, "clock_quality": 1}

    def _execute(self, name, a):
        if name in MOTOR_COMMANDS:
            return self._motor(name, a)
        if name in ENCODER_COMMANDS:
            return self._encoder(name, a)
        if name in PLATFORM_COMMANDS:
            return self._platform(name, a)
        if name == "INIT":
            if a["protocol_major"] != self.schema.version[0] or a["protocol_minor"] > self.schema.version[1]:
                raise Rejected("INCOMPATIBLE_PROTOCOL")
            if a["sdk_type"] > 3 or a["client_capabilities"] & ~3:
                raise Rejected("INVALID_ARGUMENT")
            if self.heartbeat_enabled:
                self._stop_motion()
            self.initialized, self.ready, self.lost = True, False, False
            self.heartbeat_enabled = False
            self.heartbeat_timeout_ms = 500
            self.subscriptions.clear()
            self.clock_mode = a["client_capabilities"] & 1
            self.clock_offset = 0
            self.sync_pending = None
            self.sync_interval_ms = 30000
            return dict(zip((p["name"] for p in self.schema.objects["init_response"]),
                            (self.profile["board"]["model"] if self.profile else 1, *self.revision, *self.schema.version, 0, 0)))
        if name == "PING":
            if not self.initialized:
                raise Rejected("INIT_REQUIRED")
        elif name == "SET_HEARTBEAT_CONFIG":
            self._require_ready()
            self.heartbeat_enabled = bool(a["enabled"])
            self.heartbeat_timeout_ms = a["timeout_ms"]
            if not self.heartbeat_enabled:
                self.subscriptions.clear()
        elif name == "GET_HEARTBEAT_CONFIG":
            return {"enabled": self.heartbeat_enabled, "timeout_ms": self.heartbeat_timeout_ms}
        elif name == "SET_TIME_SYNC_INTERVAL":
            if not self.initialized:
                raise Rejected("INIT_REQUIRED")
            self.sync_interval_ms = a["interval_ms"]
            self.last_sync_us = self.now_us
        elif name == "GET_TIME_STATUS":
            return {"clock_mode": self.clock_mode, "clock_quality": int(self.ready),
                    "interval_ms": self.sync_interval_ms,
                    "last_sync_age_us": self.now_us - self.last_sync_us if self.clock_mode and self.ready else 0}
        elif name in {"SUBSCRIBE", "UNSUBSCRIBE", "SUBSCRIBE_ODOMETRY", "UNSUBSCRIBE_ODOMETRY"}:
            self._require_ready()
            legacy = name.endswith("_ODOMETRY")
            if legacy:
                source = a["source"]
                topic = self.schema.topic_names["PLATFORM_ODOMETRY" if source == 4 else "ENCODER_ODOMETRY"]
                index = 0 if source == 4 else source
            else:
                topic, index = self.schema.topics[a["topic_id"]], a["index"]
            if index > topic["index_max"]:
                raise Rejected("INVALID_ARGUMENT")
            key = (topic["id"], index)
            if name.startswith("UNSUBSCRIBE"):
                self.subscriptions.pop(key, None)
                return
            minimum = topic["min_interval_ms"]
            if topic["name"].endswith("ODOMETRY"):
                minimum = max(minimum, 2 * (1000 // self.odometry_frequency))
            if not self.heartbeat_enabled or a["interval_ms"] < minimum:
                raise Rejected("INVALID_ARGUMENT")
            if key not in self.subscriptions and len(self.subscriptions) >= 16:
                raise Rejected("SUBSCRIPTION_LIMIT")
            try:
                self._sample(topic, index)
            except Rejected as error:
                if str(error) != "SAMPLE_NOT_AVAILABLE":
                    raise
            self.subscriptions[key] = {"interval": a["interval_ms"] * 1000,
                                       "due": self.now_us + a["interval_ms"] * 1000, "legacy": legacy}
        elif name == "POLL_TELEMETRY":
            self._require_ready()
            self._publish()
        elif name == "GET_TEMPERATURE":
            return self._sensor("temperature", a["sensor_index"])
        elif name == "GET_IMU":
            return self._sensor("imu", a["sensor_index"])
        elif name in {"SET_CONTROLLER_FREQUENCY", "SET_ODOMETRY_FREQUENCY"}:
            period_ms = 1000 // a["frequency"]
            if name == "SET_ODOMETRY_FREQUENCY":
                if any(self.schema.topics[k[0]]["name"].endswith("ODOMETRY") and s["interval"] < 2 * period_ms * 1000
                       for k, s in self.subscriptions.items()):
                    raise Rejected("INVALID_ARGUMENT")
                self.odometry_frequency = 1000 // period_ms
            else:
                self.controller_frequency = 1000 // period_ms
        elif name == "GET_CONTROLLER_FREQUENCY":
            return self.controller_frequency
        elif name == "GET_ODOMETRY_FREQUENCY":
            return self.odometry_frequency
        elif name == "SET_STATUS_LED_STATE":
            self.led = a["state"]
        elif name == "TOGGLE_STATUS_LED_STATE":
            self.led ^= 1
        elif name == "INITIALIZE_GPIO_PIN":
            self.gpio_modes[a["pin_number"]] = a["mode"]
        elif name == "SET_GPIO_PIN_STATE":
            self.gpio[a["pin_number"]] = a["state"]
        elif name == "GET_GPIO_PIN_STATE":
            return self.gpio[a["pin_number"]]
        elif name == "TOGGLE_GPIO_PIN_STATE":
            self.gpio[a["pin_number"]] ^= 1

    def _sensor(self, kind, index):
        if index >= self.sensor_counts[kind]:
            raise Rejected("SENSOR_NOT_AVAILABLE")
        if (kind, index) in self.sensor_errors:
            raise Rejected(self.sensor_errors[kind, index])
        if self.now_us < (300000 if kind == "temperature" else 50000):
            raise Rejected("SAMPLE_NOT_AVAILABLE")
        return self.temperature if kind == "temperature" else dict(self.imu)

    def _owned(self, index):
        return self.platform is not None and index < self.platform["wheels"]

    @staticmethod
    def _positive(a, *names):
        if any(a[n] <= 0 for n in names):
            raise Rejected("INVALID_ARGUMENT")

    @staticmethod
    def _gains(a, position=False):
        for name, value in a.items():
            if any(token in name for token in ("kp", "ki", "kd", "tolerance", "integral_limit")) and value < 0:
                raise Rejected("INVALID_ARGUMENT")
        if not position and a.get("integral_limit", 100) > 100:
            raise Rejected("INVALID_ARGUMENT")

    def _motor(self, name, a):
        index = a["motor_index"]
        m = self.motors[index]
        if name != "GET_MOTOR_CONTROLLER_STATE" and self._owned(index):
            raise Rejected("MOTOR_OWNED")
        if name == "SET_MOTOR_SPEED" and not m.initialized:
            raise Rejected("MOTOR_NOT_INITIALIZED")
        if ("POSITION" in name or name == "SET_MOTOR_TARGET_SPEED") and not m.controller:
            raise Rejected("CONTROLLER_NOT_INITIALIZED")
        if name == "INITIALIZE_MOTOR":
            m.stop()
            m.initialized = True
        elif name in {"STOP_MOTOR", "BRAKE_MOTOR", "DELETE_MOTOR_CONTROLLER"}:
            m.stop("brake" if name == "BRAKE_MOTOR" else "coast")
        elif name == "SET_MOTOR_SPEED":
            m.stop("pwm")
            m.pwm = a["pwm"]
            m.speed = self.pwm_speed * m.pwm / 100
        elif name == "INITIALIZE_MOTOR_CONTROLLER":
            self._positive(a, "encoder_resolution")
            self._gains(a)
            self._invalidate_encoder(a["encoder_index"])
            m.stop("velocity")
            m.initialized = m.controller = True
            m.encoder, m.gains = a["encoder_index"], dict(a)
            self.encoders[m.encoder] = Encoder(True, a["encoder_resolution"])
        elif name == "SET_MOTOR_TARGET_SPEED":
            m.target = None
            m.speed = a["speed"]
        elif name == "RESET_MOTOR_CONTROLLER":
            m.speed = 0
            m.target = None
        elif name == "GET_MOTOR_CONTROLLER_STATE":
            values = {p["name"]: 0 for p in self.schema.objects["motor_controller_state"]}
            if m.controller:
                values.update(motor_index=index, kp=m.gains["kp"], ki=m.gains["ki"], kd=m.gains["kd"],
                              target_speed=m.speed, current_speed=m.speed, output=m.pwm)
            return values
        elif name.startswith("INITIALIZE_MOTOR_POSITION"):
            self._positive(a, "kp", "max_speed")
            self._gains(a, True)
            m.position, m.origin, m.target, m.speed = dict(a), m.angle, 0., 0.
        elif name == "RESET_MOTOR_POSITION":
            if m.position is None:
                raise Rejected("CONTROLLER_NOT_INITIALIZED")
            m.origin, m.target, m.speed = m.angle, 0., 0.
        elif name == "SET_MOTOR_POSITION":
            if m.position is None:
                raise Rejected("CONTROLLER_NOT_INITIALIZED")
            m.target = a["position"]
        elif name == "GET_MOTOR_POSITION":
            if m.position is None:
                raise Rejected("CONTROLLER_NOT_INITIALIZED")
            return m.angle - m.origin

    def _invalidate_encoder(self, index):
        for motor in self.motors:
            if motor.encoder == index and motor.controller:
                motor.stop()
        if self._owned(index):
            self._stop_platform()
            self.platform_odometry = False

    def _encoder(self, name, a):
        index = a["encoder_index"]
        e = self.encoders[index]
        if name in {"GET_ENCODER_VALUE", "START_ENCODER_ODOMETRY"} and not e.initialized:
            raise Rejected("ENCODER_NOT_INITIALIZED")
        if name == "INITIALIZE_ENCODER":
            self._positive(a, "encoder_resolution")
            self._invalidate_encoder(index)
            self.encoders[index] = Encoder(True, a["encoder_resolution"])
        elif name == "GET_ENCODER_VALUE":
            return int(e.angle * e.resolution / math.tau) % 65536
        elif name == "START_ENCODER_ODOMETRY":
            self._require_ready()
            if self._owned(index):
                self._cancel_platform_position()
            e.running = True
            e.origin = e.angle
            e.sample_after = self.now_us + 1000000 // self.odometry_frequency
        elif name == "RESET_ENCODER_ODOMETRY":
            e.origin = e.angle
            e.sample_after = self.now_us + 1000000 // self.odometry_frequency
        elif name == "STOP_ENCODER_ODOMETRY":
            e.running = False
            if self._owned(index):
                self.platform_odometry = False
                self._cancel_platform_position()
        elif name == "GET_ENCODER_ODOMETRY":
            return self._encoder_sample(index)

    def _encoder_sample(self, index):
        metadata = self._metadata()
        e = self.encoders[index]
        if not e.running:
            raise Rejected("ODOMETRY_NOT_INITIALIZED")
        if self.now_us < e.sample_after:
            raise Rejected("SAMPLE_NOT_AVAILABLE")
        return dict(metadata, angle=e.angle - e.origin)

    def _cancel_platform_position(self):
        if self.platform_position is not None:
            self.velocity = [0.] * 3
        self.platform_target = self.platform_position = None

    def _stop_platform(self, mode="coast"):
        self.platform_controller = False
        self.velocity = [0.] * 3
        self.platform_target = self.platform_position = None
        for index, motor in enumerate(self.motors):
            if self._owned(index):
                motor.stop(mode)

    def _platform(self, name, a):
        if name.startswith("INITIALIZE_") and name.endswith("_PLATFORM"):
            dimensions = [k for k in a if k in {"wheel_diameter", "wheels_diameter", "wheel_base", "length", "width", "robot_radius", "encoder_resolution"}]
            self._positive(a, *dimensions)
            self._stop_platform()
            wheels = {"INITIALIZE_DIFFERENTIAL_PLATFORM": 2, "INITIALIZE_OMNI_PLATFORM": 3,
                      "INITIALIZE_MECANUM_PLATFORM": 4}[name]
            self.platform = dict(a, wheels=wheels, kind=name)
            self.platform_odometry = False
            for index in range(wheels):
                self.motors[index].stop()
                self.motors[index].initialized = True
                self.motors[index].encoder = index
                self.encoders[index] = Encoder(True, a["encoder_resolution"])
            return
        needs_platform = {"SET_PLATFORM_VELOCITY", "START_PLATFORM_CONTROLLER", "START_PLATFORM_ODOMETRY"}
        closed_loop = "POSITION" in name or name == "SET_PLATFORM_TARGET_VELOCITY"
        if (name in needs_platform or closed_loop) and not self.platform:
            raise Rejected("PLATFORM_NOT_INITIALIZED")
        if closed_loop and not self.platform_controller:
            raise Rejected("CONTROLLER_NOT_INITIALIZED")
        if name == "START_PLATFORM_CONTROLLER":
            self._gains(a)
            self._stop_platform()
            self.platform_controller = True
            for index in range(self.platform["wheels"]):
                self.motors[index].controller = True
                self.motors[index].gains = dict(a)
        elif name in {"STOP_PLATFORM_CONTROLLER", "BRAKE_PLATFORM", "COAST_PLATFORM"}:
            self._stop_platform("brake" if name == "BRAKE_PLATFORM" else "coast")
        elif name in {"SET_PLATFORM_VELOCITY", "SET_PLATFORM_TARGET_VELOCITY"}:
            self._cancel_platform_position()
            if name == "SET_PLATFORM_VELOCITY":
                self._stop_platform()
                self._open_loop_platform(a)
                return
            self.velocity = [a[k] for k in ("x", "y", "t")]
            if self.platform["wheels"] == 2:
                self.velocity[1] = 0.
        elif name == "GET_PLATFORM_CURRENT_VELOCITY":
            return dict(zip(("x", "y", "t"), self.velocity))
        elif name == "START_PLATFORM_ODOMETRY":
            self._require_ready()
            self.platform_odometry = True
            self._cancel_platform_position()
            for index in range(self.platform["wheels"]):
                self._encoder("START_ENCODER_ODOMETRY", {"encoder_index": index})
            self.platform_sample_after = self.now_us + 1000000 // self.odometry_frequency
        elif name in {"RESET_PLATFORM_ODOMETRY", "RESET_PLATFORM_POSITION"}:
            if name == "RESET_PLATFORM_POSITION" and not self.platform_position:
                raise Rejected("CONTROLLER_NOT_INITIALIZED")
            self.pose = [0.] * 3
            self.platform_target = None
            if name == "RESET_PLATFORM_POSITION":
                self.velocity = [0.] * 3
            else:
                self._cancel_platform_position()
            self.platform_sample_after = self.now_us + 1000000 // self.odometry_frequency
        elif name == "STOP_PLATFORM_ODOMETRY":
            self.platform_odometry = False
            self._cancel_platform_position()
        elif name == "GET_PLATFORM_ODOMETRY":
            return self._platform_sample()
        elif name.startswith("INITIALIZE_PLATFORM_POSITION"):
            self._positive(a, "linear_kp", "angular_kp", "max_linear_speed", "max_angular_speed")
            self._gains(a, True)
            if not self.platform_odometry:
                self._platform("START_PLATFORM_ODOMETRY", {})
            self.platform_position = dict(a)
            self.platform_target = None
            self.velocity = [0.] * 3
        elif name == "SET_PLATFORM_POSITION":
            if not self.platform_position:
                raise Rejected("CONTROLLER_NOT_INITIALIZED")
            self._platform_sample()
            self.platform_target = [a[k] for k in ("x", "y", "t")]

    def _platform_sample(self):
        metadata = self._metadata()
        if not self.platform_odometry:
            raise Rejected("ODOMETRY_NOT_INITIALIZED")
        if self.now_us < self.platform_sample_after:
            raise Rejected("SAMPLE_NOT_AVAILABLE")
        return dict(metadata, **dict(zip(("x", "y", "t"), self.pose)))

    def _wheel_targets(self):
        x, y, t = self.velocity
        p = self.platform
        radius = p.get("wheel_diameter", p.get("wheels_diameter")) / 2
        if p["wheels"] == 2:
            return [(x - t * p["wheel_base"] / 2) / radius, (x + t * p["wheel_base"] / 2) / radius]
        if p["wheels"] == 3:
            return [(math.sqrt(3) / 2 * x + .5 * y + p["robot_radius"] * t) / radius,
                    (-math.sqrt(3) / 2 * x + .5 * y + p["robot_radius"] * t) / radius,
                    (-y + p["robot_radius"] * t) / radius]
        rotation = (p["length"] + p["width"]) / 2 * t
        return [(x + y + rotation) / radius, (x - y + rotation) / radius,
                (x + y - rotation) / radius, (x - y - rotation) / radius]

    def _open_loop_platform(self, a):
        x, y, t = (max(-100., min(100., a[k])) for k in ("x", "y", "t"))
        p = self.platform
        if p["wheels"] == 2:
            pwm = [x - t, x + t]
        elif p["wheels"] == 3:
            pwm = [math.sqrt(3) / 2 * x + .5 * y + t, -math.sqrt(3) / 2 * x + .5 * y + t, -y + t]
            scale = max(1., max(map(abs, pwm)) / 100)
            pwm = [value / scale for value in pwm]
        else:
            # The current C open-loop mixer uses integer abs() in its denominator.
            denominator = abs(int(x)) + abs(int(y)) + abs(int(t))
            x, y, t = [v * abs(v) / denominator if denominator else 0. for v in (x, y, t)]
            pwm = [x + y + t, x - y + t, x + y - t, x - y - t]
        speeds = []
        for index, value in enumerate(pwm):
            motor = self.motors[index]
            motor.pwm = max(-100., min(100., value))
            motor.speed = motor.pwm * self.pwm_speed / 100
            motor.mode = "pwm"
            speeds.append(motor.speed)
        radius = p.get("wheel_diameter", p.get("wheels_diameter")) / 2
        if p["wheels"] == 2:
            self.velocity = [radius / 2 * sum(speeds), 0., radius / p["wheel_base"] * (speeds[1] - speeds[0])]
        elif p["wheels"] == 3:
            v0, v1, v2 = speeds
            self.velocity = [radius / math.sqrt(3) * (v0 - v1), radius / 3 * (v0 + v1 - 2 * v2),
                             radius / (3 * p["robot_radius"]) * sum(speeds)]
        else:
            v0, v1, v2, v3 = speeds
            self.velocity = [radius / 4 * sum(speeds), radius / 4 * (v0 - v1 + v2 - v3),
                             radius / (2 * (p["length"] + p["width"])) * (v0 + v1 - v2 - v3)]

    def _sample(self, topic, index):
        name = topic["name"]
        if name == "ENCODER_ODOMETRY":
            return self._encoder_sample(index)
        if name == "PLATFORM_ODOMETRY":
            return self._platform_sample()
        if name == "TEMPERATURE":
            return dict(self._metadata(), celsius=self._sensor("temperature", index))
        if name == "IMU":
            return dict(self._metadata(), **self._sensor("imu", index))
        raise Rejected("UNKNOWN_COMMAND")

    def _publish(self):
        for (topic_id, index), sub in list(self.subscriptions.items()):
            if self.now_us < sub["due"]:
                continue
            sub["due"] = self.now_us + sub["interval"]
            topic = self.schema.topics[topic_id]
            try:
                sample = self._sample(topic, index)
            except Rejected:
                continue
            payload = self.schema.pack(self.schema.objects[topic["payload_type"]], sample)
            if sub["legacy"]:
                encoder = topic["name"] == "ENCODER_ODOMETRY"
                self._emit("ENCODER_ODOMETRY_EVENT" if encoder else "PLATFORM_ODOMETRY_EVENT", 0,
                           (bytes([index]) if encoder else b"") + payload)
            else:
                self._emit("TELEMETRY_EVENT", 0, bytes([topic_id, index]) + payload)

    def _stop_motion(self):
        self._stop_platform()
        for motor in self.motors:
            motor.stop()

    def disconnect(self):
        self._stop_motion()
        self.lost = True
        self.ready = self.initialized = self.heartbeat_enabled = False
        self.sync_pending = None
        self.subscriptions.clear()
        self._input.clear()
        self._output.clear()

    @staticmethod
    def _bounded(error, gain, maximum, tolerance, dt):
        if abs(error) <= tolerance:
            return 0.
        return math.copysign(min(abs(error) * gain, maximum, abs(error) / dt), error)

    def _integrate(self, dt):
        if self.platform_target is not None and self.platform_position:
            p = self.platform_position
            dx, dy = self.platform_target[0] - self.pose[0], self.platform_target[1] - self.pose[1]
            heading = self.pose[2]
            distance = math.hypot(dx, dy)
            angle_error = math.remainder(self.platform_target[2] - heading, math.tau)
            speed = self._bounded(distance, p["linear_kp"], p["max_linear_speed"], p["position_tolerance"], dt)
            if self.platform["wheels"] == 2:
                if distance > p["position_tolerance"]:
                    angle_error = math.remainder(math.atan2(dy, dx) - heading, math.tau)
                    speed *= max(0., math.cos(angle_error))
                vx, vy = speed, 0.
            else:
                vx = speed * (dx * math.cos(heading) + dy * math.sin(heading)) / distance if distance else 0.
                vy = speed * (-dx * math.sin(heading) + dy * math.cos(heading)) / distance if distance else 0.
            self.velocity = [vx, vy, self._bounded(angle_error, p["angular_kp"], p["max_angular_speed"], p["heading_tolerance"], dt)]
        if self.platform_controller:
            for index, speed in enumerate(self._wheel_targets()):
                self.motors[index].speed = speed
        for m in self.motors:
            if m.position is not None and m.target is not None:
                p = m.position
                m.speed = self._bounded(m.target - (m.angle - m.origin), p["kp"], p["max_speed"], p["tolerance"], dt)
            delta = m.speed * dt
            m.angle += delta
            if self.encoders[m.encoder].initialized:
                self.encoders[m.encoder].angle += delta
        if self.platform_odometry:
            vx, vy, omega = self.velocity
            heading = self.pose[2]
            self.pose[0] += (vx * math.cos(heading) - vy * math.sin(heading)) * dt
            self.pose[1] += (vx * math.sin(heading) + vy * math.cos(heading)) * dt
            self.pose[2] += omega * dt

    def advance(self, seconds):
        """Advance simulation time; watchdog cuts motion at its exact deadline."""
        if not math.isfinite(seconds) or seconds < 0:
            raise ValueError("Time step must be finite and nonnegative")
        target = self.now_us + round(seconds * 1000000)
        expiry = self.last_activity_us + self.heartbeat_timeout_ms * 1000
        motion_end = min(target, expiry) if self.heartbeat_enabled else target
        while self.now_us < motion_end:
            step = min(10000, motion_end - self.now_us)
            self._integrate(step / 1000000)
            self.now_us += step
            self._advance_imu(self.now_us, step / 1000000)
        if self.heartbeat_enabled and target >= expiry:
            self.disconnect()
        # Keep sensor input cadence after watchdog stop. A single long remainder
        # would spread the stopping impulse over that entire interval.
        while self.now_us < target:
            step = min(10000, target - self.now_us)
            self.now_us += step
            self._advance_imu(self.now_us, step / 1000000)
        if self.ready and self.clock_mode and not self.sync_pending and self.now_us - self.last_sync_us >= self.sync_interval_ms * 1000:
            self._request_sync()
        if self.ready:
            self._publish()
