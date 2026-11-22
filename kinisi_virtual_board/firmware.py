"""Real C firmware with a Python motor plant and deterministic virtual time."""
import ctypes as ct
import math
import os
from pathlib import Path
import shutil
import tempfile
import threading
import weakref

from .firmware_build import DEFAULT_FIRMWARE, build_firmware
from .schema import Schema


def _release(library, folder, dll_directory):
    # No native worker threads exist. Calls are serialized and close is terminal.
    import _ctypes
    if os.name == "nt":
        _ctypes.FreeLibrary(library._handle)
    else:
        _ctypes.dlclose(library._handle)
    folder.cleanup()
    if dll_directory:
        dll_directory.close()


class FirmwareBoard:
    """Native firmware backend compatible with all virtual-board transports.

    Motor output is PWM percent. A first-order plant returns quantized, wrapping
    16-bit encoder counts to the production controller/odometry tasks. This runs
    the firmware's C application, not the STM32 binary, HAL or real RTOS.
    Instances own independent shared libraries and must be closed after use.
    """
    FRAM_SIZE = 8192

    def __init__(self, commands=None, *, firmware=DEFAULT_FIRMWARE, revision="0.4.0",
                 compiler=None, max_speed=10.0, time_constant=0.05, encoder_motors=(0, 1, 2, 3),
                 fram_image=None, fram_read_failure=False):
        self.schema = commands if isinstance(commands, Schema) else Schema(commands or Path(firmware) / "commands.json")
        if not math.isfinite(max_speed) or max_speed <= 0 or not math.isfinite(time_constant) or time_constant <= 0:
            raise ValueError("Plant max_speed and time_constant must be finite and positive")
        if len(encoder_motors) != 4 or any(type(i) is not int or not 0 <= i < 4 for i in encoder_motors):
            raise ValueError("encoder_motors must map each of the four encoders to a motor index")
        if fram_image is not None:
            fram_image = bytes(fram_image)
            if len(fram_image) != self.FRAM_SIZE:
                raise ValueError(f"FRAM image must contain exactly {self.FRAM_SIZE} bytes")
        self.revision = tuple(map(int, revision.split(".")))
        self.max_speed, self.time_constant = max_speed, time_constant
        self.encoder_motors = tuple(encoder_motors)
        self.motor_velocity = [0.] * 4
        self.motor_angle = [0.] * 4
        self._counts = [0.] * 4
        self.now_us = 0
        self._fraction_us = 0.
        self._lock = threading.RLock()
        library, self.build = build_firmware(self.schema, firmware, revision, compiler)
        self.unsupported_commands = self.build["unsupported_commands"]
        folder = tempfile.TemporaryDirectory(prefix="kinisi-board-")
        dll_directory = None
        try:
            instance = Path(folder.name) / library.name
            shutil.copyfile(library, instance)
            if os.name == "nt":
                dll_directory = os.add_dll_directory(str(Path(self.build["compiler_path"]).parent))
            self._lib = ct.CDLL(str(instance))
        except BaseException:
            folder.cleanup()
            if dll_directory:
                dll_directory.close()
            raise
        self._finalizer = weakref.finalize(self, _release, self._lib, folder, dll_directory)
        signatures = {
            "init": (None, ()), "feed": (ct.c_int, (ct.c_void_p, ct.c_size_t)),
            "poll": (ct.c_int, (ct.c_uint64,)), "deadline": (ct.c_uint64, ()),
            "read": (ct.c_size_t, (ct.c_void_p, ct.c_size_t)),
            "disconnect": (None, ()), "lost": (ct.c_int, ()), "ready": (ct.c_int, ()),
            "drop": (None, (ct.c_uint8, ct.c_uint)), "encoder": (None, (ct.c_uint8, ct.c_uint16)),
            "pwm": (ct.c_double, (ct.c_uint8,)), "resolution": (ct.c_double, (ct.c_uint8,)),
            "reverse": (ct.c_int, (ct.c_uint8,)), "mode": (ct.c_int, (ct.c_uint8,)),
            "temperature": (None, (ct.c_double, ct.c_int)),
            "imu": (None, (ct.POINTER(ct.c_double), ct.c_int)),
            "current": (None, (ct.c_uint8, ct.c_double, ct.c_int)),
            "fault": (None, (ct.c_uint8, ct.c_int)),
            "sleep_readback_failure": (ct.c_int, (ct.c_uint8, ct.c_int)),
            "wake_readback_failure": (ct.c_int, (ct.c_uint8, ct.c_int)),
            "fram_load": (ct.c_int, (ct.c_void_p, ct.c_size_t)),
            "fram_read": (ct.c_size_t, (ct.c_void_p, ct.c_size_t)),
            "fram_fault": (None, (ct.c_int64, ct.c_int)),
            "temperature_reads": (ct.c_uint64, ()), "imu_reads": (ct.c_uint64, ()),
            "fram_reads": (ct.c_uint32, ()), "fram_writes": (ct.c_uint32, ()),
        }
        self.has_drv8870 = hasattr(self._lib, "sim_pad")
        if self.has_drv8870:
            signatures.update({
                "usb_busy": (None, (ct.c_int,)),
                "timer_ticks": (ct.c_int, (ct.c_uint, ct.c_uint)),
                "motor_pad_touched": (ct.c_int, (ct.c_uint, ct.c_uint)),
                "pad_af": (ct.c_int, (ct.c_uint, ct.c_uint)),
                "pad": (ct.c_int, (ct.c_uint, ct.c_uint, ct.POINTER(ct.c_int))),
                "low_dwell_ms": (ct.c_uint64, ()),
                "i2c_write": (ct.c_int, (ct.c_void_p, ct.c_size_t, ct.c_int)),
                "i2c_begin_read": (None, ()),
                "i2c_end_read": (ct.c_size_t, (ct.c_void_p, ct.c_size_t)),
            })
        for name, (result, arguments) in signatures.items():
            function = getattr(self._lib, "sim_" + name)
            function.restype, function.argtypes = result, arguments
        if fram_image is not None:
            self._lib.sim_fram_load(fram_image, len(fram_image))
        self._lib.sim_fram_fault(-1, bool(fram_read_failure))
        self._lib.sim_init()

    @property
    def fram_image(self):
        """Raw nonvolatile bytes; pass to a new board to simulate a cold boot.

        A new board loads a separate library with fresh runtime globals. Do not
        call sim_init again on the old instance: that is not a cold restart.
        """
        with self._lock:
            self._check_open()
            data = ct.create_string_buffer(self.FRAM_SIZE)
            if self._lib.sim_fram_read(data, self.FRAM_SIZE) != self.FRAM_SIZE:
                raise RuntimeError("Unable to read virtual FRAM image")
            return data.raw

    @property
    def sensor_read_counts(self):
        """Count calls into the virtual sensor hardware, including explicit reads."""
        with self._lock:
            self._check_open()
            return {"temperature": self._lib.sim_temperature_reads(), "imu": self._lib.sim_imu_reads()}

    @property
    def fram_transactions(self):
        with self._lock:
            self._check_open()
            return (self._lib.sim_fram_reads(), self._lib.sim_fram_writes())

    def set_fram_fault(self, *, write_after_bytes=None, read_failure=False):
        """Fail writes after a byte budget, retaining already-written bytes.

        The budget spans transactions until reset. None disables write failure;
        zero fails the next write before changing storage. Read failure never
        mutates bytes. Faults are intentionally below the production journal.
        """
        if write_after_bytes is not None and (type(write_after_bytes) is not int or
                                               not 0 <= write_after_bytes < 2**63):
            raise ValueError("Write budget must be a nonnegative int64 or None")
        with self._lock:
            self._check_open()
            self._lib.sim_fram_fault(-1 if write_after_bytes is None else write_after_bytes,
                                    bool(read_failure))

    def _check_open(self):
        if not self._finalizer.alive:
            raise RuntimeError("Native board is closed")

    def _poll(self):
        error = self._lib.sim_poll(self.now_us)
        if error:
            raise RuntimeError(f"Native scheduler invariant failed: {error}")

    @property
    def ready(self):
        with self._lock:
            self._check_open()
            return bool(self._lib.sim_ready())

    @property
    def lost(self):
        with self._lock:
            self._check_open()
            return bool(self._lib.sim_lost())

    @property
    def motor_pwm(self):
        with self._lock:
            self._check_open()
            return tuple(self._lib.sim_pwm(i) for i in range(4))

    def _check_drv8870(self):
        self._check_open()
        if not self.has_drv8870:
            raise ValueError("This observation requires a native DRV8870 sleep profile")

    def input_pad(self, bank, bit):
        """Return (source, high ticks per 840) for a motor input, or (None, None)."""
        if bank not in ("A", "B", "C", "D", "E") or not isinstance(bit, int) or not 0 <= bit < 16:
            raise ValueError("Expected GPIO bank A..E and bit 0..15")
        with self._lock:
            self._check_drv8870()
            ticks = ct.c_int()
            source = self._lib.sim_pad(ord(bank) - ord("A"), bit, ct.byref(ticks))
            if source < 0:
                return None, None
            return ("timer" if source else "gpio"), ticks.value

    def timer_ticks(self, timer, channel):
        """Raw compare on a started regular channel, or -1 if it has not started."""
        if timer not in range(1, 13) or channel not in range(1, 5):
            raise ValueError("Expected TIM1..12, channel 1..4")
        with self._lock:
            self._check_drv8870()
            return self._lib.sim_timer_ticks(timer, channel)

    def motor_pad_touched(self, bank, bit):
        """Whether motor firmware has ever configured or written this pad since boot."""
        if bank not in ("A", "B", "C", "D", "E") or type(bit) is not int or not 0 <= bit <= 15:
            raise ValueError("Expected GPIO bank A..E and bit 0..15")
        with self._lock:
            self._check_drv8870()
            return bool(self._lib.sim_motor_pad_touched(ord(bank) - ord("A"), bit))

    def pad_alternate_function(self, bank, bit):
        """Alternate function selected by HAL_GPIO_Init, or -1 in GPIO mode."""
        if bank not in ("A", "B", "C", "D", "E") or bit not in range(16):
            raise ValueError("Expected GPIO bank A..E and bit 0..15")
        with self._lock:
            self._check_drv8870()
            return self._lib.sim_pad_af(ord(bank) - ord("A"), bit)

    @property
    def low_dwell_ms(self):
        """Total HAL delay requested while all eight pads were GPIO outputs reading low."""
        with self._lock:
            self._check_drv8870()
            return self._lib.sim_low_dwell_ms()

    def set_usb_busy(self, busy):
        """Hold USB replies pending to model an occupied CDC transmit endpoint."""
        with self._lock:
            self._check_drv8870()
            self._lib.sim_usb_busy(bool(busy))
            self._poll()

    def i2c_write(self, data, *, start=True):
        """Start a master write, or continue the same transaction with start=False."""
        data = bytes(data)
        with self._lock:
            self._check_drv8870()
            if not self._lib.sim_i2c_write(data, len(data), start):
                raise ValueError("I2C write exceeds the active length-prefixed transaction")
            self._poll()

    def i2c_read(self, size=256):
        """One fixed-size master read. An empty result means no reply was ready."""
        if type(size) is not int or not 1 <= size <= 256:
            raise ValueError("I2C master read size must be 1..256")
        with self._lock:
            self._check_drv8870()
            buffer = ct.create_string_buffer(size)
            self._lib.sim_i2c_begin_read()
            self._poll()
            length = self._lib.sim_i2c_end_read(buffer, size)
            return buffer.raw[:length]

    def feed(self, data):
        data = bytes(data)
        with self._lock:
            self._check_open()
            if not self._lib.sim_feed(data, len(data)):
                raise BufferError("Native firmware RX buffer full")
            self._poll()

    def drain(self, size=None):
        size = 65536 if size is None else min(max(int(size), 0), 65536)
        with self._lock:
            self._check_open()
            buffer = ct.create_string_buffer(size)
            length = self._lib.sim_read(buffer, size)
            return buffer.raw[:length]

    def advance(self, seconds):
        if not math.isfinite(seconds) or seconds < 0:
            raise ValueError("Time advance must be finite and nonnegative")
        with self._lock:
            self._check_open()
            elapsed = seconds * 1e6 + self._fraction_us
            whole = int(elapsed)
            self._fraction_us = elapsed - whole
            end = self.now_us + whole
            while self.now_us < end:
                step = min(1000, end - self.now_us, self._lib.sim_deadline() - self.now_us)
                if step <= 0:
                    raise RuntimeError("Native scheduler failed to advance its deadline")
                dt = step / 1e6
                delta = []
                for i in range(4):
                    target = self._lib.sim_pwm(i) * self.max_speed / 100
                    if self._lib.sim_reverse(i):
                        target = -target
                    # Braking decelerates faster than coasting; this is a test plant.
                    tau = self.time_constant / (5 if self._lib.sim_mode(i) == 1 else 1)
                    decay = math.exp(-dt / tau)
                    prior = self.motor_velocity[i]
                    self.motor_velocity[i] = target + (prior - target) * decay
                    distance = target * dt + (prior - target) * tau * (1 - decay)
                    self.motor_angle[i] += distance
                    delta.append(distance)
                for encoder, motor in enumerate(self.encoder_motors):
                    self._counts[encoder] += delta[motor] * self._lib.sim_resolution(encoder) / math.tau
                    self._lib.sim_encoder(encoder, int(self._counts[encoder]) & 0xffff)
                self.now_us += step
                self._poll()
            self._poll()

    def drop_next_reply(self, command, count=1):
        if type(count) is not int or not 0 <= count <= 0xffffffff:
            raise ValueError("Drop count must be an unsigned 32-bit integer")
        with self._lock:
            self._check_open()
            self._lib.sim_drop(self.schema.code(command), count)

    def set_temperature(self, celsius=25., *, status=0):
        with self._lock:
            self._check_open()
            self._lib.sim_temperature(celsius, status)

    def set_imu(self, acceleration=(0., 0., 9.80665), angular_velocity=(0., 0., 0.), *, status=0):
        if len(acceleration) != 3 or len(angular_velocity) != 3:
            raise ValueError("IMU vectors must have three components")
        with self._lock:
            self._check_open()
            self._lib.sim_imu((ct.c_double * 6)(*acceleration, *angular_velocity), status)

    def set_current(self, index, amperes=0., *, status=0):
        self._index(index)
        with self._lock:
            self._check_open()
            self._lib.sim_current(index, amperes, status)

    @staticmethod
    def _index(index):
        if type(index) is not int or not 0 <= index < 4:
            raise ValueError("Hardware index must be 0..3")

    def set_fault(self, index, active=True):
        self._index(index)
        with self._lock:
            self._check_open()
            self._lib.sim_fault(index, active)

    def set_sleep_readback_failure(self, index, active=True):
        """Force nSLEEP (DRV8873) or IN1 (DRV8870) readback high until cleared."""
        self._index(index)
        with self._lock:
            self._check_open()
            if not self._lib.sim_sleep_readback_failure(index, active):
                raise ValueError("Selected firmware has no simulated driver sleep pins")

    def set_wake_readback_failure(self, index, active=True):
        """Corrupt the channel's IC4 readback after writes until cleared."""
        self._index(index)
        with self._lock:
            self._check_open()
            if not self._lib.sim_wake_readback_failure(index, active):
                raise ValueError("Selected firmware has no simulated driver SPI interface")

    def disconnect(self):
        with self._lock:
            self._check_open()
            self._lib.sim_disconnect()

    def close(self):
        with self._lock:
            if self._finalizer.alive:
                self._lib.sim_disconnect()
                self._finalizer()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
