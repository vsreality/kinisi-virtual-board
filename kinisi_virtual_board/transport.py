"""Serial-like in-process transport and loopback TCP byte stream."""
import socket
import socketserver
import threading
import time


class VirtualSerial:
    """pyserial-compatible boundary; the real SDK still encodes/decodes all bytes.

    Use realtime=False with advance() for deterministic tests. A read timeout is
    wall-clock time in either mode. Each instance represents one board connection.
    """
    def __init__(self, board, *, timeout=0.05, write_timeout=1.0, fragment_size=3, realtime=True):
        if fragment_size < 1:
            raise ValueError("fragment_size must be positive")
        self.board = board
        self.timeout = timeout
        self.write_timeout = write_timeout
        self.fragment_size = fragment_size
        self.realtime = realtime
        self.is_open = True
        self._lock = threading.RLock()
        self._cancelled = threading.Event()
        self._last_ns = time.perf_counter_ns()

    def _tick(self):
        now = time.perf_counter_ns()
        if self.realtime:
            self.board.advance((now - self._last_ns) / 1e9)
        self._last_ns = now

    def write(self, data):
        with self._lock:
            if not self.is_open:
                raise OSError("Virtual serial port is closed")
            self._tick()
            self.board.feed(bytes(data))
            return len(data)

    def read(self, size=1):
        if size <= 0:
            return b""
        deadline = time.monotonic() + (self.timeout or 0)
        while True:
            with self._lock:
                if not self.is_open or self._cancelled.is_set():
                    return b""
                self._tick()
                data = self.board.drain(min(size, self.fragment_size))
                if data:
                    return data
            if self.timeout is not None and time.monotonic() >= deadline:
                return b""
            self._cancelled.wait(0.001)

    def advance(self, seconds):
        with self._lock:
            self._tick()
            self.board.advance(seconds)

    def reset_input_buffer(self):
        with self._lock:
            self.board.drain()

    def cancel_read(self):
        self._cancelled.set()

    def close(self):
        with self._lock:
            if self.is_open:
                self.board.disconnect()
            self.is_open = False
        self.cancel_read()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


class _Handler(socketserver.BaseRequestHandler):
    def handle(self):
        board = self.server.board_factory()
        self.request.settimeout(0.01)
        self.request.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        last = time.perf_counter_ns()
        try:
            while not self.server.stopping.is_set():
                try:
                    data = self.request.recv(4096)
                    if not data:
                        return
                except socket.timeout:
                    data = b""
                now = time.perf_counter_ns()
                board.advance((now - last) / 1e9)
                last = now
                board.feed(data)
                output = board.drain()
                if output:
                    self.request.sendall(output)
        except (OSError, ConnectionError):
            pass
        finally:
            board.disconnect()
            if hasattr(board, "close"):
                board.close()


class TCPServer(socketserver.ThreadingTCPServer):
    """One independent virtual board per connection; never touches physical ports."""
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address, board_factory):
        self.board_factory = board_factory
        self.stopping = threading.Event()
        super().__init__(address, _Handler)

    def server_close(self):
        self.stopping.set()
        super().server_close()
