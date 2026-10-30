# Kinisi virtual board

Run production Kinisi C firmware in a standalone C++ simulator for SDK testing.
It models four motors and encoders while executing the actual protocol,
controllers, odometry, watchdog and telemetry code.

The SDK connects to one serial device: a Linux PTY or a Windows COM port.
The Windows endpoint uses the included UMDF driver. No COM pair or proxy process
is needed. Python is used to build the simulator, not to run it.

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
- [Legacy Python serial endpoints](docs/serial.md)
- [Legacy Python behavioral model](docs/python-model.md)

This folder can live in a separate repository; it does not modify firmware or SDK
sources. `--revision 0.3.0` selects v0.3; v0.4 is the default.
