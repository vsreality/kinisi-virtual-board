# Standalone firmware simulator

`native/` is a C++17 host, motor plant and OS serial endpoint. It loads
`firmware.dll` or `firmware.so` from beside the executable. That library contains
production C firmware plus the hardware/scheduler boundary in
`kinisi_virtual_board/firmware_support/`. There is no Python dependency at runtime.

The build stages generated headers from the selected firmware checkout and its
`commands.json`, and saves the schema and source/build identity beside the app.
Rebuild after changing firmware or commands; editing the packaged JSON alone does
not change the compiled firmware. No SDK or firmware sources are rewritten.

## Physical IMU installation

The normal executable accepts the chip position relative to the chassis origin
or the PCB underside center, and a rigid board-to-chassis rotation. Offsets use metres in forward/left/up
coordinates. The rotation is row-major, maps board coordinates to chassis
coordinates, and must be finite, orthonormal and right-handed (tolerance 1e-6).

```powershell
.\build\native\kinisi-sim.exe --imu-offset-m "0,-0.1,0" --imu-board-to-chassis "1,0,0,0,1,0,0,0,1"
```

The command above places the **chip** 100 mm right of the chassis origin.
Use `--imu-board-center-m "0,-0.1,0"` to place the PCB underside center
100 mm right of the chassis origin. This option and `--imu-offset-m` are mutually
exclusive. The selected library embeds `imu_geometry.underside_center_to_imu_um`
from its firmware checkout's hardware board profile at build time. It computes
`chip = center + board_to_chassis * board_geometry`, using the profile's integer
micrometre precision. Rotating a board around a fixed center therefore moves the
chip. `--library` selects that library's geometry; there is no host fallback.
Older profiles without this field and older libraries without this optional API
reject board-center input explicitly while retaining the chip-offset interface.

The native no-slip motor/shaft plant adds tangential and centripetal acceleration
at the chip, rotates the specific force and angular rate into board axes, then uses
the existing v0.4 chip convention `(-board X, -board Y, +board Z)` inside the
BMI088 model. A static tilted installation starts at its projected gravity.
An offset in height does not add rotational acceleration for planar yaw alone.
These options also apply to `--stdio-test`. The centered identity default keeps
the previous sensor stream unchanged. Explicit physical mount options require
an IMU-equipped board; v0.3/Nano still start normally without them.

The Python native adapter has the same physical boundary:

```python
from kinisi_virtual_board import FirmwareBoard
from kinisi_virtual_board.imu_mount import Mount

with FirmwareBoard(imu_mount=Mount(offset_m=(0, -0.1, 0))) as board:
    board.advance(0.1)
```

Alternatively call `board.set_imu_mount(offset_m, board_to_chassis)` before
advancing the clock. The matrix is a nested 3x3 tuple. Reconfiguration after
time starts or after exact input injection is rejected. `set_imu` continues to
accept exact native chip-axis samples, bypassing the physical mount model.
Neither physical interface declares a mount to a firmware estimator or changes
its configuration. Firmware/SDK mount commands remain a separate declaration.
The native `sim_imu_mount` ABI still accepts chip offsets. The optional
`sim_imu_board_center(center_m, row_major_rotation)` API applies the selected
profile geometry and returns 1 on success, 0 for invalid setup, or -1 when the
profile has no geometry. Both setters require clock zero and modeled IMU input.

## Linux

```bash
python3 tools/build_native.py --out build/native-linux
./build/native-linux/kinisi-sim
```

A startup line such as `{"backend":"native-cpp","serial_port":"/dev/pts/3"}`
identifies the SDK port. Use the exact printed path, in the same Linux environment
and user account. A WSL PTY is not accessible as a Windows COM port. No driver
installation or socat process is needed. PTYs may not appear in browser serial
pickers; clients that accept a device path can open them directly.

## Windows build

Install Visual Studio C++ tools, Windows SDK, and GCC (MSYS2 UCRT64 is detected at
`C:/msys64/ucrt64/bin/gcc.exe`). Python 3.10+ runs the build and schema generator.
Set `KINISI_MSVC` to the MSVC version directory containing `bin/Hostx64/x64/cl.exe`
if it is not under Visual Studio 2022 Community. Set `KINISI_WINDOWS_SDK` for a
nonstandard Windows SDK root.

```powershell
python tools/build_native.py
python tools/build_driver.py --wdk C:\path\to\WDK
```

The WDK root must contain `Include/wdf/umdf/2.33` and
`Lib/wdf/umdf/x64/2.33`. With Microsoft's `Microsoft.Windows.WDK.x64` NuGet package,
this is its `c` directory. Build tools run InfVerif and Inf2Cat; success produces
`build/driver/kinisiport.dll`, `.inf` and `.cat`. The package is initially unsigned.

The driver targets Windows 11 x64. Other Windows versions/architectures are not
validated. The C firmware uses GCC; MSVC builds the host and UMDF driver.

## Windows signing and installation

Local development signing is separate from system trust and installation:

```powershell
.\tools\sign_driver.ps1 -Wdk C:\path\to\WDK
```

This creates a three-month code-signing certificate in CurrentUser/My, signs the
DLL and catalog, and exports only the public certificate to `kinisi-test.cer`.
It does not trust the certificate, install the driver, or change boot/security
settings. Reuse a certificate by passing `-Thumbprint` on later builds. Rebuilding
the driver invalidates its previous signature; sign again after the final build.
A distributable release needs an appropriate production signing process.

On a development test machine, review the certificate thumbprint and package
before explicitly authorizing trust. The administrative installation steps are:

```powershell
Import-Certificate -FilePath .\build\driver\kinisi-test.cer -CertStoreLocation Cert:\LocalMachine\Root
Import-Certificate -FilePath .\build\driver\kinisi-test.cer -CertStoreLocation Cert:\LocalMachine\TrustedPublisher
pnputil /add-driver .\build\driver\kinisiport.inf /install
```

Trust is machine-wide and outlives the simulator. Do not run these steps on
someone else's machine without permission. This project does not disable Secure
Boot, signature enforcement or enable test mode. Acceptance of a development
certificate depends on the test machine's policy; an install or load failure
must be investigated, not worked around by silently weakening policy.

From an **Administrator terminal**:

```powershell
.\build\native\kinisi-sim.exe
```

Windows requires administrator access for
[SwDeviceCreate](https://learn.microsoft.com/en-us/windows/win32/api/swdevice/nf-swdevice-swdevicecreate).
The app creates a software device with one COM interface. The SDK opens the
printed `COMn`. The simulator uses a private driver interface to exchange bytes;
there is no second COM endpoint. Windows hosts the UMDF DLL in its normal driver
host process. Closing the simulator removes its active software device.

The device grants access to its creator, SYSTEM and administrators. Keep the SDK
under the same account as the simulator. Only one SDK connection and one host
connection are accepted per board. Multiple simulator processes create separate
boards. The system assigns COM numbers; do not hard-code them.

For removal, first close simulators and SDK connections. Find this package's
published `oemNN.inf` using `pnputil /enum-drivers`, verify its original name is
`kinisiport.inf`, then run `pnputil /delete-driver oemNN.inf /uninstall` as admin.
Remove only this test certificate's exact thumbprint from LocalMachine Root and
TrustedPublisher when no longer needed. Never delete other driver packages or
certificates. Remove its CurrentUser/My certificate when done signing builds.

## Validation

The build runs native bounded-buffer and serial-timeout tests. Then run:

```powershell
$env:KINISI_REQUIRE_NATIVE = '1'
python -m unittest discover -s tests -v
```

After installing the Windows driver, use an Administrator terminal:

```powershell
$env:KINISI_TEST_WINDOWS_DRIVER = '1'
python -m unittest discover -s tests -p test_standalone.py -v
```

The single-port test starts its own simulator and uses the ordinary Python SDK
serial connection, telemetry and reconnect behavior. It never selects physical
hardware. Set `KINISI_NATIVE_BINARY` for another build directory. SDK tests need
the sibling `pykinisi` checkout and pyserial. The wider suite uses Node.js and
`jskinisi`; unavailable optional integrations are reported as skips.

Current evidence: Windows and Linux executables build; Linux SDK/PTY integration
passes; INF validation and catalog generation pass. Windows driver 0.2.0.1 was
installed and loaded successfully. The ordinary Python SDK passed connection,
motor-state, telemetry and reconnect tests through its single COM port. Live
checks also passed port discovery, exclusive access, read timeouts, cancellation
and transmit-empty event behavior. Browser WebSerial and cross-account access
denial remain untested.

## Behavioral limits

The production protocol, command dispatch, PID/position loops, platform math,
odometry, time synchronization, watchdog and telemetry execute as C code. A
cooperative scheduler supplies deterministic task timing; this is not FreeRTOS
or instruction-level STM32 emulation. Hardware is simulated: first-order motors,
quantized 16-bit encoders, GPIO state, default sensor values and driver state.
ADC noise, electrical loads, current regulation, USB enumeration, MCU interrupts
and physical fault protection are not reproduced. The Python firmware harness
also provides programmable sensor/fault injection.

The default plant is 10 rad/s at 100% duty and a 50 ms time constant, with a
shorter braking constant. Encoders map one-to-one to motors. Controller periods
and timeouts come from firmware. Real-time execution uses a monotonic clock;
`--stdio-test` accepts deterministic binary operations for automated tests.

The Windows driver buffers 64 KiB in each direction, implements serial read
interval/total timeouts, cancellation through WDF queues, wait masks, purge and
exclusive SDK ownership. Writes are accepted immediately when space exists and
fail when full; baud rate does not throttle bytes. Hardware/software flow control
is not simulated. Generation IDs discard replies across SDK reconnects. Linux
holds the PTY open and relies on protocol heartbeat to detect an absent SDK;
keep heartbeats enabled. Stalled output terminates the simulator with an error.

The existing firmware declares `GET_PLATFORM_CURRENT_VELOCITY` without a handler;
the simulator preserves `UNKNOWN_COMMAND`. Use the Python CLI `--check` to audit
coverage against the selected checkout.
