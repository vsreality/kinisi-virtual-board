# Native firmware simulation

The default `firmware` backend compiles production C application sources into a
host DLL/shared library. `FirmwareBoard` loads one private copy per board and
connects it to a Python motor plant. The SDK sends its ordinary protocol bytes.
The native backend uses firmware implementations for command execution, PID,
platform kinematics and subscriptions.

## What runs

| Component | Implementation |
| --- | --- |
| Frame validation and responses | Firmware `lib/protocol` and generated `commands.h` |
| Command execution and prerequisites | Firmware `src/command_handler.c` and `lib/command_requirements` |
| INIT/READY, time sync, watchdog, telemetry | Firmware `lib/initialization`, `lib/time_sync`, `lib/connection` |
| Motor velocity/position PID | Firmware `controllers_manager.c` and `lib/libcontrollers` |
| Platform kinematics and position control | Firmware differential/omni/mecanum/platform-position sources |
| Encoder and world odometry | Firmware odometry manager, encoder odometry and integrator |
| Configuration save and boot restore | Firmware `controller_configuration.c` and `lib/config_store` |
| Motor, encoder, GPIO and sensor interfaces | Native shim in `firmware_support/runtime.c` |
| Motor response and clock advancement | Python `firmware.py` |
| COM/TTY, PTY and TCP | Python endpoint adapters |

The exact source list is in `firmware_build.py` and startup JSON's
`native_build.sources`. The shim replaces the command task's USB/I2C queue wiring
with one stream and connects real connection services to real managers.

This host build does not execute the flashed ARM image, boot sequence, STM32 HAL,
interrupts, DMA, USB enumeration, SPI/I2C drivers or FreeRTOS scheduler. It cannot
prove MCU timing, stack use, scheduling races, driver configuration or electrical
behavior. Continue running firmware host suites, MCU builds and hardware tests.

## Build and schema

```bash
python3 -m kinisi_virtual_board --firmware ../kinisi-motor-controller-firmware --revision 0.4.0 --pty
```

The PTY example requires Linux. On Windows use `--serial COM11` after creating a
virtual pair. The firmware directory can be an independent checkout anywhere.

- `--commands` defaults to `--firmware/commands.json`. An SDK copy is accepted
  only when its parsed JSON equals that firmware schema. Mismatches fail early.
- The firmware's command generator produces `commands.h` in temporary staging.
  The selected `boards/v0.3.json` or `boards/v0.4.json` supplies resource counts,
  driver availability and current-limit capabilities. The MCU-specific
  `hw_config.h` is replaced only in staging.
- `--compiler` / `CC` names a host compiler executable, not an ARM compiler or
  shell command with extra flags. GCC was verified on Windows/MSYS2 and Ubuntu.
  Clang is also detected, but was not validated here.
- Libraries are cached in the OS temporary directory under
  `kinisi-native-firmware/<build-id>`. The key hashes application sources,
  relevant headers/generator code, schema, board file, shim, build script and
  compiler identity. Changes trigger a new build. The checkout stays untouched.
- Each board loads a separate library copy, isolating firmware C globals even
  with several sessions in one process. `close()` unloads it and removes the
  instance directory. Use a context manager or close in `finally`.
- `native_build.build_id` identifies host build inputs. The on-wire MCU build ID
  remains zero; it must not be mistaken for a flashed release.

`--check` reports commands missing from C dispatch source and returns nonzero
when any are found. This static coverage check is an aid, not a behavioral proof.
For the current protocol 2.12.0 checkout, `GET_PLATFORM_CURRENT_VELOCITY` has no
handler and returns `UNKNOWN_COMMAND`. A regression test confirms that actual
wire response. The simulator does not supply a replacement implementation.

The firmware also emits an existing compiler warning for integer `abs()` applied
to doubles in the open-loop mecanum path. The host build follows the firmware
host tests in allowing this warning and preserves the source.

## Virtual time and motor feedback

```python
from kinisi_virtual_board import FirmwareBoard, VirtualSerial

with FirmwareBoard(firmware="../kinisi-motor-controller-firmware") as board:
    port = VirtualSerial(board, realtime=False, fragment_size=1)
    # Supply port as the SDK's serial transport; see examples/python_sdk.py.
    # Use KinisiController(wall_clock=False) for manually stepped tests.
    port.advance(0.1)
    print(board.motor_pwm, board.motor_velocity, board.motor_angle)
```

`feed(bytes)`, `drain(size=None)`, `advance(seconds)` and `disconnect()` provide the
same stream interface as the legacy model. `ready`, `lost`, `now_us`, `schema`,
`revision`, `build` and `unsupported_commands` expose diagnostic state. Native
board calls are serialized with a reentrant lock. Use `port.advance()` while an
SDK reader thread is active. `VirtualSerial.close()` disconnects the session;
the caller retains ownership of the board and closes it separately.

## Saved configuration and virtual cold boots

With firmware protocol 2.14, the native backend runs the production configuration
capture, FRAM journal, CRC/board/version checks and restore path. Only the FRAM
hardware boundary is replaced with an 8192-byte array. Restore runs before the
native connection is initialized, matching the command task's ordering.

`board.fram_image` returns the complete storage image. Pass those bytes to a new
`FirmwareBoard(fram_image=image)` to simulate a cold restart. The new board loads
an independent shared library with fresh C runtime state; only FRAM is retained.
`disconnect()` is a connection change and does not perform a boot or restore.
Without an image, FRAM starts erased. To retain it between processes, write the
image to a binary file and read it into the next constructor. Standalone serial
processes do not automatically retain FRAM across launches.

`set_fram_fault(write_after_bytes=N)` interrupts writes after N bytes across
transactions, preserving partial writes for the subsequent cold boot.
`set_fram_fault(read_failure=True)` makes storage reads fail without mutation;
the constructor also accepts `fram_read_failure=True` to test boot failure.
Calling `set_fram_fault()` clears both faults. `fram_transactions` returns read
and write transaction counts, allowing unsupported profiles to prove they never
access storage. Storage snapshots bypass injected bus failures for inspection.

Run the production-path regressions with:

```powershell
python -m unittest discover -s tests -p test_configuration.py -v
```

The tests exercise differential, omni and mecanum setup, independent encoder
scales/directions, PWM selection, unsaved edits, reconnect, stopped restoration,
every byte boundary of interrupted replacement saves, invalid records, read
failures and unsupported boards. A separate-process test saves a binary image
and starts another Python process with a fresh firmware library, checking that
configuration survives but motor outputs, time and controller activity do not.
This verifies application logic, not physical FRAM retention, I²C behavior,
STM32 startup, electrical timing or MCU stack/resource use.

Controller and odometry tasks run cooperatively. The shim yields at their
`vTaskDelayUntil` boundaries using the approach in the firmware's host tests.
It primes each task at creation, then runs it at its requested virtual deadline.
This relies on the two current task loops keeping persistent state in their
argument structures; changes to task structure require simulator review. This
is not a general FreeRTOS port. Task capacity and mutex invariants fail loudly.

The plant advances in steps of at most 1 ms, split at task deadlines. Defaults:

- 100% PWM corresponds to a 10 rad/s unloaded target speed.
- First-order response has a 50 ms time constant; braking decelerates five times
  faster. These are test parameters, not measured Kinisi hardware.
- Encoders accumulate counts using configured counts/revolution and wrap at
  16 bits. Actual firmware computes speed, position and rollover deltas.
- Encoders connect to the same-index motor by default. Constructor
  `encoder_motors=(...)` maps each encoder to a motor; `max_speed` and
  `time_constant` configure the plant. Motor/encoder reversal is independent.
- Load, slip, collision, noise, battery, thermal and electrical effects are absent.
  Current sensing is injected independently. Current limits configure simulated
  driver state but do not simulate current regulation.

The real PID needs time to run: setting a target does not immediately update the
PID state returned by GET. Platform position needs velocity control and fresh
odometry; advance one odometry period after initialization before sending its
first target. The example uses 100 Hz control, gains chosen for the test plant,
and a 0.01 rad position tolerance.

For long deterministic motion tests disable heartbeat explicitly or arrange
virtual heartbeats. For watchdog tests leave it enabled. Real endpoints advance
from `perf_counter_ns()`, which provides sufficient resolution on Windows for
microsecond RTT checks. Wall-clock synchronization requires a real-time transport
or an SDK clock tied to virtual time: a frozen board clock cannot measure real
host processing time.

## Fault and sensor injection

```python
board.set_temperature(32.5)                       # Celsius
board.set_imu((0, 0, 9.80665), (0, 0, 0))         # m/s² and rad/s
board.set_current(0, 1.25)                        # Amperes
board.set_fault(0, True)                          # Active overcurrent fault
board.set_fault(0, False)                         # Clear injected condition
board.drop_next_reply("PING")                    # Executes; reply is lost
```

Sensor methods accept optional hardware `status` integers: temperature/IMU use
0=OK, 1=unavailable, 2=not-ready, 3=I/O-error; current uses 0=OK, 1=unavailable,
2=out-of-range, 3=I/O-error. The shim models initial temperature and IMU settling
at 300 ms and 50 ms. On v0.4 the automatic BMI088 producer samples accel and gyro
independently. `set_imu` selects exact injected readings for that board instance.
Temperature conversion cadence and bus recovery are outside the shim.
v0.3 reports sensors/driver functions absent per its board definition.
[BMI088 settings and timing](bmi088.md)

Faults feed real command/controller checks and event subscription sequences.
The driver itself is simulated: clearing faults succeeds for an initialized
simulated driver. GPIO models values and resource bounds, without electrical
behavior. Use byte corruption to test wire validation; the native backend has no
`fail_next()` override for command execution.

## Tests and transport limits

```powershell
python -m unittest discover -s tests -p test_firmware.py -v
python examples/python_sdk.py
python -m unittest discover -s tests -v
```

`KINISI_FIRMWARE` selects the native test checkout. `KINISI_COMMANDS` selects the
schema for legacy/shared endpoint tests. `KINISI_PYSDK` and `KINISI_JSSDK` select
SDK directories. Set `KINISI_REQUIRE_SDKS=1` for jobs requiring both SDKs and Node;
otherwise absent external SDK/runtime integrations explicitly skip.

Tests cover real PID timing/convergence, encoder wrap, independent reverse flags,
motor/platform position, world odometry, initial-sample gating, malformed frames,
watchdog loss, sensor/driver injection, current limits, event telemetry, time sync,
isolated instances and schema mismatch. Python SDK tests use fragmented
in-process reads and TCP; JavaScript runs its production session/codec over Node
TCP. Linux tests use actual PTYs with the ordinary SDK serial factory, including
CLI startup and reconnect.

TCP is available with `--host 127.0.0.1 --port 8766` (port 0 selects automatically).
It serves raw bytes and creates a separate board per client; it is not the web
client's WebSocket protocol. COM/PTY mode owns one board across reconnects, with
bounded queues and watchdog cleanup. See [serial setup](serial.md). Concurrent
USB/I2C access, baud timing, WebSerial enumeration and Arduino Wire are outside
this simulation.

Validation uses Windows and Ubuntu under WSL. Windows DLL and both SDK
integrations are exercised; Linux shared-library and direct PTY integrations are
exercised. The Windows COM-pair test remains opt-in because no virtual pair is
installed. Node is absent in Linux here, so its JavaScript integration skips.
Firmware/SDK schemas and generated artifacts are unchanged; the synchronization
audit covers all three SDKs, including Arduino.


## Driver sleep on v0.4

With a firmware checkout that exposes `SET_DRIVER_SLEEP`, the v0.4 build links
production `hw_motor.c` and `hw_drv8873.c`. The plant reads the resulting GPIO
and timer states, including the 840-step PWM duty. Each simulated driver has
its own SPI register bank. Asserting sleep clears its registers. The host uses
the firmware's motor-control test HAL header for the peripheral declarations.
HAL delays consume no wall time, so this tests command sequencing and readback;
wake timing, electrical behavior and current draw require hardware.

Run `python -m unittest discover -s tests -p test_driver_sleep.py -v` against
protocol 2.15 firmware. In the Python harness,
`board.set_wake_readback_failure(2)` corrupts channel 2's current-limit register
on write. Pass `False` as the second argument to allow verified writes again.
The injection applies during initialization as well as wake.

## DRV8870 sleep on original v0.3 and Nano

With protocol 2.17 firmware, select `--revision 0.3.0` for the original board or
`--revision 0.3.1` for Nano. Both execute the firmware motor driver. The 0.3.0
build uses PA9/TIM1_CH2 for M3 IN2. Nano uses PA10/TIM1_CH3 and reserves PA9
for USB sensing. The native host supplies USB attachment explicitly.

`FirmwareBoard.feed()` feeds the firmware USB command queue. On these profiles,
`i2c_write(frame)` starts a transaction on the same board. To pause after a
prefix, write the first bytes and then call `i2c_write(tail, start=False)`.
Each `i2c_read()` requests one padded 256-byte reply through the firmware slave
callbacks. An empty result means the simulated master received no frame.
`set_usb_busy(True)` holds USB replies to exercise pending requests while the
other session runs. The C++ serial endpoint uses the USB command session.

`input_pad("A", 10)` returns the output source and high ticks per 840-count
period, or `(None, None)` for an undriven or unmapped motor input.
`timer_ticks(1, 3)` reads a regular timer compare and
`pad_alternate_function("A", 10)` reports the selected pin mux.
`low_dwell_ms` counts requested HAL delay while every motor input is a GPIO
output reading low. After explicit release, a fresh brake produces high/high on
the addressed channels. Ordinary coast produces low/low and permits immediate
drive. These observations describe settled simulated outputs. Oscilloscope
waveforms, transient timing, first-drive latency and current draw require hardware.

## Shallow board idle on original v0.3 and Nano

Protocol 2.18 enables the existing `SET_BOARD_IDLE` and `GET_BOARD_IDLE` commands
on both DRV8870 revisions. Within an established USB session, a PING or time read
is serviced while logical idle remains set. An interrupt may wake the physical CPU
to handle work. An explicit `SET_BOARD_IDLE(0)` exits the protocol mode and
restores the prior driver-inhibit policy. Outputs stay low until a fresh drive
or brake command is accepted.

The native tests run the production command and motor code. The motor HAL's
`motor_pad_touched("A", 9)` records any GPIO configuration or write since boot,
including a temporary one. Nano uses PA9 for VBUS and PA10 for M3 IN2. Its USB
attach/detach policy is exercised in the firmware host fixtures, including the
Nano command task with the production USB lifecycle during an idle session. The simulator models settled pin outputs and protocol state. Embedded
instruction selection, time in WFI, physical current and wake latency require
separate evidence.
