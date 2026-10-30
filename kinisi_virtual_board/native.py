"""Native serial endpoints. No TCP, forwarding service, or proxy process."""
import os
import threading
import time


class SerialEndpoint:
    """Open the emulator side of an existing virtual COM/TTY pair with pyserial."""
    def __init__(self, port, baudrate=115200):
        if "://" in port:
            raise ValueError("--serial requires a native COM/TTY device, not a transport URL")
        if baudrate <= 0:
            raise ValueError("Baud rate must be positive")
        self.port = port
        self.baudrate = baudrate
        self.serial = None

    def __enter__(self):
        try:
            import serial
        except ImportError as error:
            raise RuntimeError('Serial mode requires: python -m pip install "kinisi-virtual-board[serial]"') from error
        self.serial = serial.Serial(self.port, self.baudrate, timeout=.01, write_timeout=.1,
                                    xonxoff=False, rtscts=False, dsrdtr=False)
        return self

    def read(self):
        return self.serial.read(max(1, min(self.serial.in_waiting, 4096)))

    def write(self, data):
        return self.serial.write(data)

    def close(self):
        if self.serial is not None:
            self.serial.close()
            self.serial = None

    def __exit__(self, *_):
        self.close()


class PTYEndpoint:
    """Create a raw POSIX PTY; the SDK opens port (e.g. /dev/pts/3) normally.

    Keep a slave handle open so SDK disconnect/reconnect does not destroy the
    endpoint. Session loss is detected by the protocol heartbeat, not modem lines.
    """
    def __init__(self):
        self.master = self.slave = None
        self.port = None

    def __enter__(self):
        if os.name != "posix":
            raise RuntimeError("--pty requires Linux/POSIX; on Windows use --serial COMn with a virtual COM pair")
        import tty
        try:
            self.master, self.slave = os.openpty()
            tty.setraw(self.slave)
            os.set_blocking(self.master, False)
            self.port = os.ttyname(self.slave)
        except BaseException:
            self.close()
            raise
        return self

    def read(self):
        import select
        if not select.select([self.master], [], [], .01)[0]:
            return b""
        try:
            return os.read(self.master, 4096)
        except BlockingIOError:
            return b""

    def write(self, data):
        try:
            return os.write(self.master, data)
        except BlockingIOError:
            return 0

    def close(self):
        for name in ("master", "slave"):
            descriptor = getattr(self, name)
            if descriptor is not None:
                setattr(self, name, None)
                os.close(descriptor)

    def __exit__(self, *_):
        self.close()


def serve_native(board, endpoint, stop=None):
    """Service an already-open native endpoint until stopped or I/O fails.

    Bound pending output and retain partial writes. A stalled reader cannot stop
    watchdog processing or cause unlimited telemetry buffering. Fatal I/O errors
    propagate to the CLI after stopping the simulated board.
    """
    stop = stop if stop is not None else threading.Event()
    previous = time.perf_counter_ns()
    pending = bytearray()
    pending_since = None
    try:
        while not stop.is_set():
            data = endpoint.read()
            now = time.perf_counter_ns()
            board.advance((now - previous) / 1e9)
            previous = now
            if board.lost:
                pending.clear()
                pending_since = None
            board.feed(data)
            pending.extend(board.drain())
            if len(pending) > 65536:
                raise OSError("Native serial client is not consuming output (64 KiB limit)")
            if pending:
                pending_since = now if pending_since is None else pending_since
                written = endpoint.write(pending)
                if not isinstance(written, int) or not 0 <= written <= len(pending):
                    raise OSError("Invalid native serial write result")
                del pending[:written]
                if written:
                    pending_since = now if pending else None
                elif now - pending_since > 1_000_000_000:
                    raise TimeoutError("Native serial client did not accept output for one second")
    finally:
        board.disconnect()
