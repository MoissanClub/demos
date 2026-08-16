"""Non-blocking terminal exit-key handling for the handshake controller."""

import os
import select
import sys
import termios
import threading
import tty
from typing import Any, Optional


def is_exit_key(data: bytes) -> bool:
    return data in (b"q", b"Q")


class KeyboardExitMonitor:
    """Watch a TTY for q/Q without requiring Enter and restore it on exit."""

    def __init__(self, stream: Any = None) -> None:
        self.stream = sys.stdin if stream is None else stream
        self.exit_requested = threading.Event()
        self.enabled = False
        self._stop_requested = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._fd: Optional[int] = None
        self._original_settings: Any = None

    def start(self) -> bool:
        try:
            fd = self.stream.fileno()
            if not os.isatty(fd):
                return False
            original = termios.tcgetattr(fd)
            tty.setcbreak(fd)
        except (AttributeError, OSError, termios.error):
            return False

        self._fd = fd
        self._original_settings = original
        self.enabled = True
        self._thread = threading.Thread(target=self._read_loop, name="keyboard_exit", daemon=True)
        self._thread.start()
        return True

    def stop(self) -> None:
        self._stop_requested.set()
        if self._thread is not None:
            self._thread.join(timeout=0.3)
        if self._fd is not None and self._original_settings is not None:
            try:
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._original_settings)
            except (OSError, termios.error):
                pass
        self.enabled = False

    def _read_loop(self) -> None:
        assert self._fd is not None
        while not self._stop_requested.is_set():
            try:
                readable, _, _ = select.select([self._fd], [], [], 0.1)
                if readable and is_exit_key(os.read(self._fd, 1)):
                    self.exit_requested.set()
                    return
            except OSError:
                return
