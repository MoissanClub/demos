import asyncio
import unittest
from types import SimpleNamespace

from handshake.brainco_replay import BrainCoHandReplay


class FakeContext:
    def __init__(self):
        self.positions = [0] * 6
        self.closed = False
        self.read_failures = 0
        self.unit_mode = None

    async def set_finger_unit_mode(self, _slave_id, mode):
        self.unit_mode = mode

    async def get_finger_unit_mode(self, _slave_id):
        return self.unit_mode

    async def set_finger_positions_and_speeds(self, _slave_id, positions, _speeds):
        self.positions = list(positions)

    async def get_motor_status(self, _slave_id):
        if self.read_failures:
            self.read_failures -= 1
            raise RuntimeError("transient read")
        return SimpleNamespace(positions=list(self.positions))

    async def close(self):
        self.closed = True


class FakeSdk:
    class FingerUnitMode:
        Normalized = "normalized"

    def __init__(self):
        self.context = FakeContext()

    async def modbus_open(self, _port, _baud):
        await asyncio.sleep(0)
        return self.context


class BrainCoHandReplayTests(unittest.TestCase):
    def test_commands_are_measured_and_shutdown_reopens_hand(self):
        sdk = FakeSdk()
        events = []
        replay = BrainCoHandReplay(
            sdk, "/dev/fake", object(), 0x7F,
            lambda name, details: events.append((name, details)),
            timeout_seconds=1.0,
        )
        replay.start()
        replay.command((100,) * 6, "test_close", wait=True)
        self.assertEqual(sdk.context.positions, [100] * 6)
        replay.close()
        self.assertEqual(sdk.context.positions, [0] * 6)
        self.assertTrue(sdk.context.closed)
        self.assertTrue(any(
            details["reason"] == "test_close" and details["measured_positions"] == [100] * 6
            for name, details in events if name == "brainco_hand_command"
        ))

    def test_out_of_bounds_command_fails_closed(self):
        replay = BrainCoHandReplay(
            FakeSdk(), "/dev/fake", object(), 0x7F, lambda *_: None,
        )
        with self.assertRaisesRegex(ValueError, "positions"):
            replay.command((10001,) * 6, "too_far")

    def test_initial_open_retries_a_transient_measurement_failure(self):
        sdk = FakeSdk()
        sdk.context.read_failures = 1
        events = []
        replay = BrainCoHandReplay(
            sdk, "/dev/fake", object(), 0x7F,
            lambda name, details: events.append((name, details)),
            timeout_seconds=1.0,
        )
        replay.start()
        replay.close()
        self.assertTrue(any(name == "brainco_hand_open_reference_retry" for name, _ in events))
        self.assertEqual(sdk.context.positions, [0] * 6)


if __name__ == "__main__":
    unittest.main()
