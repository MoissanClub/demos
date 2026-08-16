import unittest
import os
import pty
import termios

from handshake.keyboard import KeyboardExitMonitor, is_exit_key


class NoFileDescriptor:
    pass


class KeyboardExitTests(unittest.TestCase):
    def test_q_and_uppercase_q_are_exit_keys(self):
        self.assertTrue(is_exit_key(b"q"))
        self.assertTrue(is_exit_key(b"Q"))

    def test_other_keys_are_not_exit_keys(self):
        self.assertFalse(is_exit_key(b"x"))
        self.assertFalse(is_exit_key(b"\n"))

    def test_non_terminal_input_disables_monitor_cleanly(self):
        monitor = KeyboardExitMonitor(NoFileDescriptor())
        self.assertFalse(monitor.start())
        monitor.stop()
        self.assertFalse(monitor.enabled)

    def test_terminal_q_requests_exit_and_settings_are_restored(self):
        master_fd, slave_fd = pty.openpty()
        stream = os.fdopen(slave_fd, "rb", buffering=0)
        original = termios.tcgetattr(slave_fd)
        monitor = KeyboardExitMonitor(stream)
        try:
            self.assertTrue(monitor.start())
            os.write(master_fd, b"Q")
            self.assertTrue(monitor.exit_requested.wait(timeout=1.0))
        finally:
            monitor.stop()
            restored = termios.tcgetattr(slave_fd)
            stream.close()
            os.close(master_fd)
        self.assertEqual(restored, original)


if __name__ == "__main__":
    unittest.main()
