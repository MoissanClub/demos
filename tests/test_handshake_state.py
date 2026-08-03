import unittest

from handshake_state import HandshakeConfig, HandshakeState, HandshakeStateMachine


def config(**overrides):
    values = {
        "start_threshold": 50.0,
        "stop_threshold": 250.0,
        "release_threshold": 20.0,
        "release_seconds": 0.7,
        "hold_duration": 5.0,
        "max_close": 500,
        "step": 50,
    }
    values.update(overrides)
    return HandshakeConfig(**values)


class HandshakeStateMachineTests(unittest.TestCase):
    def test_normal_contact_closes_holds_and_releases(self):
        machine = HandshakeStateMachine(config())

        decision = machine.update(60.0, 1.0)
        self.assertEqual(decision.state, HandshakeState.CLOSING)
        self.assertTrue(decision.trigger_arm)

        decision = machine.update(100.0, 1.1)
        self.assertEqual(decision.command_close, 50)

        decision = machine.update(260.0, 1.2)
        self.assertEqual(decision.state, HandshakeState.HOLD)
        self.assertEqual(decision.event, "pressure_limit_reached")

        machine.update(0.0, 1.3)
        decision = machine.update(0.0, 2.0)
        self.assertEqual(decision.state, HandshakeState.OPEN_WAIT)
        self.assertEqual(decision.command_close, 0)
        self.assertTrue(decision.release_arm)

    def test_release_during_closing_opens(self):
        machine = HandshakeStateMachine(config())
        machine.update(60.0, 0.0)
        machine.update(0.0, 0.1)

        decision = machine.update(0.0, 0.8)

        self.assertEqual(decision.event, "release_during_closing")
        self.assertEqual(decision.command_close, 0)
        self.assertTrue(decision.release_arm)

    def test_hold_timeout_opens_even_with_contact(self):
        machine = HandshakeStateMachine(config(hold_duration=2.0))
        machine.update(60.0, 0.0)
        machine.update(300.0, 0.1)

        decision = machine.update(300.0, 2.1)

        self.assertEqual(decision.event, "hold_timeout")
        self.assertEqual(decision.state, HandshakeState.OPEN_WAIT)
        self.assertTrue(decision.release_arm)

    def test_max_close_enters_hold(self):
        machine = HandshakeStateMachine(config(max_close=100, step=50))
        machine.update(60.0, 0.0)
        machine.update(60.0, 0.1)
        machine.update(60.0, 0.2)

        decision = machine.update(60.0, 0.3)

        self.assertEqual(decision.state, HandshakeState.HOLD)
        self.assertEqual(decision.close_value, 100)
        self.assertEqual(decision.event, "max_close_reached")

    def test_requires_confirmed_release_before_rearming(self):
        machine = HandshakeStateMachine(config())
        machine.update(60.0, 0.0)
        machine.update(300.0, 0.1)
        machine.update(0.0, 0.2)
        machine.update(0.0, 0.9)

        decision = machine.update(60.0, 1.0)
        self.assertEqual(decision.state, HandshakeState.OPEN_WAIT)
        self.assertFalse(decision.trigger_arm)

        machine.update(0.0, 1.1)
        machine.update(0.0, 1.8)
        decision = machine.update(60.0, 1.9)
        self.assertEqual(decision.state, HandshakeState.CLOSING)
        self.assertTrue(decision.trigger_arm)

    def test_release_timer_resets_if_contact_returns(self):
        machine = HandshakeStateMachine(config())
        machine.update(60.0, 0.0)
        machine.update(0.0, 0.1)
        machine.update(60.0, 0.5)
        machine.update(0.0, 0.6)

        decision = machine.update(0.0, 1.0)

        self.assertEqual(decision.state, HandshakeState.CLOSING)
        self.assertFalse(decision.release_arm)


if __name__ == "__main__":
    unittest.main()
