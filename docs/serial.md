# Legacy Python serial endpoints

For the standalone C++ app with one Windows COM port and no pair, see [standalone setup](standalone.md). The paired-port instructions below apply only to the older Python serial adapter.

The default native firmware backend needs the host compiler and firmware checkout described in [native firmware simulation](native-firmware.md).

These modes run the board's protocol directly on the OS serial endpoint. They
do not open a TCP listener or use com2tcp, hub4com, socat, or a proxy process.
Existing SDK serial connection code can open the client device normally.

## Windows: virtual COM pair

Install a virtual COM-pair driver compatible with your Windows security settings
and create two available ports, for example `COM10` (SDK) and `COM11` (emulator).
Windows needs this OS driver; Python alone cannot register a real COM device.
The driver is a one-time prerequisite, not a separate bridge application.
[com0com's documentation](https://com0com.sourceforge.net/) describes this exact
device-emulator topology. Its [published installation notes](https://com0com.sourceforge.net/com0com/ReadMe.txt)
describe test-signed builds; compatibility with current Windows driver-signing
requirements must be checked for the driver you install. This project does not
install drivers or change Windows signing/security settings.

After creating the pair:

```powershell
python -m pip install -e ".[serial]"
python -m kinisi_virtual_board --serial COM11
```

The SDK opens **COM10**, not COM11:

```python
from pykinisi import KinisiController

client = KinisiController()
try:
    if not client.connect("COM10"):
        raise RuntimeError(client.last_error)
    client.ping()
finally:
    client.disconnect()
```

Use your pair's actual names. The emulator never guesses a port or opens a
physical controller automatically. For browser WebSerial, the virtual driver
must register the SDK-facing endpoint as a discoverable COM port; the current
JavaScript client has no USB-only filter.

To run the real Windows COM integration test after driver setup:

```powershell
$env:KINISI_EMULATOR_PORT = 'COM11'
$env:KINISI_CLIENT_PORT = 'COM10'
$env:KINISI_REQUIRE_COM = '1'
python -m unittest discover -s tests -p test_native.py -v
```

`KINISI_REQUIRE_COM=1` makes missing pair configuration a failure instead of a
skip. These variables must refer to a **virtual pair reserved for testing**.
The test opens both endpoints and sends simulated motor commands.

## Linux: emulator-created PTY

```bash
python3 -m kinisi_virtual_board --pty
```

The JSON startup record contains `"transport": "pty"` and
`"serial_port": "/dev/pts/N"`. Pass that exact path to the SDK:

```python
client.connect("/dev/pts/3")  # Replace with the path printed by your emulator.
```

The emulator creates the PTY directly using the OS, sets raw binary mode, and
removes it when it exits. No root access, driver installation, or socat process
is required. The PTY server itself uses only Python's standard library; the Python
SDK still needs pyserial. Run both processes in the same Linux environment and
as the same user unless you explicitly configure device permissions. A PTY
created in WSL is a Linux device, not a COM port available to Windows apps.
PTYs may not appear in browser WebSerial discovery; select the device path in
clients that accept native TTY paths. This is not virtual USB emulation.

Linux can also use `--serial /dev/...` to open the emulator side of an existing
serial pair. That mode uses the same pyserial adapter as Windows and requires
the `[serial]` extra. `--serial` and `--pty` are mutually exclusive. `--baudrate`
defaults to 115200 for native serial; PTYs do not model transmission baud timing.

```bash
python3 -m unittest discover -s tests -p test_native.py -v
```

Linux tests use actual PTYs and the unmodified Python SDK, including binary
control bytes, handshake, motor state, telemetry, reconnect, and CLI startup.
Platform-specific tests explicitly skip on the other OS. Windows COM integration
cannot be verified without an installed and configured virtual pair.

Native endpoints stay open across client reconnects. Keep SDK heartbeats enabled:
peer closure is detected by the protocol watchdog, not DTR/DSR. With heartbeat
disabled, the model cannot promptly detect a closed client. Ctrl+C stops the
emulator and closes its endpoint. Pending writes are bounded; port removal or a
stalled output produces an error and stops the simulated board.
