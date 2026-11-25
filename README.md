# Kinisi virtual board

Run production Kinisi C firmware in a standalone C++ simulator for SDK testing.
It models four motors and encoders while executing the actual protocol,
controllers, odometry, watchdog and telemetry code.

The SDK connects to one serial device: a Linux PTY or a Windows COM port.
The Windows endpoint uses the included UMDF driver. No COM pair or proxy process
is needed. Python builds the standalone executable and also provides an
in-process native firmware harness for regression tests.

## Build and run

Keep the firmware checkout at `../kinisi-motor-controller-firmware`, or pass
`--firmware PATH`. The build consumes that checkout's canonical `commands.json`.

Linux requires Python 3.10+, GCC and a C++17 compiler:

```bash
python3 tools/build_native.py --out build/native-linux
./build/native-linux/kinisi-sim
```

Connect the SDK to the `serial_port` printed at startup. Ctrl+C closes the board.

Windows requires MSVC, the Windows SDK and GCC for the production C sources:

```powershell
python tools/build_native.py
.\build\native\kinisi-sim.exe
```

**Windows COM support requires the Kinisi driver to be signed and installed
first, and the simulator must run from an Administrator terminal.** The driver
passes Microsoft's INF/catalog validation and live Python SDK serial tests.
See [standalone setup](docs/standalone.md).
The current driver targets Windows 11 x64.

## Test

From this checkout, run the public v0.4 IMU motion regression with Python 3.10+
and a host GCC or Clang compiler:

```sh
python -m kinisi_virtual_board imu-regression --firmware ../kinisi-motor-controller-firmware
```

The command builds the selected firmware, drives its BMI088 input and compares
reported pose and velocity to known motion. It prints a JSON report with the
build, seed and per-case results. It runs locally without a serial driver or
administrator access. See the [cases, limits and exit codes](docs/release-test-plan.md#public-imu-regression).

For the full 144-case chassis qualification run
`python -m kinisi_virtual_board imu-qualify --firmware ../kinisi-motor-controller-firmware`.
It saves its public plan, sensor inputs and results automatically.
[Qualification matrix and replay](docs/release-test-plan.md#full-public-imu-release-qualification)

Run the eight native feedback source selector scenarios and export SDK packet
examples with a clean candidate checkout:

```sh
python -m kinisi_virtual_board selector-regression --firmware ../kinisi-motor-controller-firmware --out build/selector-run
```

This covers migration, all three sources, switching, stale data and unsupported
boards. [Selector cases, receipts and optional browser checks](docs/release-test-plan.md#feedback-source-selector-regression)

Use the [controller firmware release test plan](docs/release-test-plan.md) for
candidate builds, virtual-board regression, story checks and release evidence.

```powershell
python -m unittest discover -s tests -v
```

The suite includes production firmware tests, native executable tests and SDK
integration. Standalone tests require a built executable; set
`KINISI_REQUIRE_NATIVE=1` to make a missing build fail. Linux serial integration
uses the unmodified Python SDK. Windows single-COM integration is enabled with
`KINISI_TEST_WINDOWS_DRIVER=1` after driver installation.

The simulator runs firmware application code, not an STM32 binary or electrical
hardware. Missing firmware handlers remain visible to tests.

- [Standalone architecture, builds, driver setup and limits](docs/standalone.md)
- [Python firmware harness and deterministic fault injection](docs/native-firmware.md)
- [v0.4 BMI088 measurements, timing and model limits](docs/bmi088.md)
- [Legacy Python serial endpoints](docs/serial.md)
- [Legacy Python behavioral model](docs/python-model.md)

This folder can live in a separate repository; it does not modify firmware or SDK
sources. `--revision 0.3.0` selects v0.3; v0.4 is the default.
