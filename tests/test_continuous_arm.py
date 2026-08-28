import contextlib
import io
import unittest
from dataclasses import replace
from types import SimpleNamespace

from g1_arm_sdk_raise import CANDIDATE_CAPTURE_SCALE_010, parse_args
from handshake.continuous_arm import ContinuousArmConfig, ContinuousArmController
from handshake.standalone_arm import ARM_JOINT_INDICES


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def clock(self):
        return self.now

    def monotonic_ns(self):
        return int(self.now * 1e9)

    def sleep(self, duration):
        self.now += duration


class Plant:
    def __init__(self, clock):
        self.clock = clock
        self.positions = [0.0] * 30
        self.velocities = [0.0] * 30
        self.torques = [0.0] * 30
        self.commands = []
        self.fsm_id = 501
        self.fsm_mode = 0

    def state(self):
        motors = [
            SimpleNamespace(q=q, dq=dq, tau_est=tau)
            for q, dq, tau in zip(self.positions, self.velocities, self.torques)
        ]
        return SimpleNamespace(motor_state=motors), self.clock.monotonic_ns()

    def sport(self):
        return SimpleNamespace(fsm_id=self.fsm_id, fsm_mode=self.fsm_mode), self.clock.monotonic_ns()

    def command(self, positions, velocities, torques, weight):
        self.commands.append((list(positions), list(velocities), list(torques), weight))
        for index in ARM_JOINT_INDICES:
            self.positions[index] = positions[index]
            self.velocities[index] = 0.0


def fast_config():
    return ContinuousArmConfig(
        sample_rate_hz=50.0,
        acquire_seconds=0.5,
        raise_seconds=0.5,
        return_seconds=0.5,
        release_seconds=0.5,
        settle_seconds=0.1,
        settle_timeout_seconds=0.5,
    )


class ContinuousArmCliTests(unittest.TestCase):
    def test_execute_requires_all_safety_confirmations(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args(["--execute-cycle", "--raise-offset-rad", *(["0"] * 14)])

    def test_plan_requires_explicit_or_named_candidate_target(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args(["--print-plan"])
        args = parse_args(["--print-plan", "--raise-offset-rad", *(["0"] * 14)])
        self.assertTrue(args.print_plan)
        args = parse_args(["--print-plan", "--candidate-capture-scale-010"])
        self.assertTrue(args.candidate_capture_scale_010)

    def test_authority_test_has_no_raise_target(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args([
                "--execute-authority-test",
                "--confirm-gantry-attached",
                "--confirm-estop-ready",
                "--confirm-regular-mode-501-0",
            ])

    def test_candidate_moves_only_right_arm_and_is_capped(self):
        self.assertTrue(all(CANDIDATE_CAPTURE_SCALE_010[i] == 0.0 for i in range(15, 22)))
        self.assertEqual(max(abs(v) for v in CANDIDATE_CAPTURE_SCALE_010.values()), 0.08)

    def test_raise_execution_remains_paused(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args([
                "--execute-cycle", "--candidate-capture-scale-010",
                "--confirm-gantry-attached", "--confirm-estop-ready",
                "--confirm-regular-mode-501-0",
            ])

    def test_authority_execution_is_suspended_after_wrist_abort(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args([
                "--execute-authority-test", "--confirm-gantry-attached",
                "--confirm-estop-ready", "--confirm-regular-mode-501-0",
            ])

    def test_xr_pattern_execution_requires_specific_review(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args([
                "--execute-xr-pattern-authority-test", "--confirm-gantry-attached",
                "--confirm-estop-ready", "--confirm-regular-mode-501-0",
            ])
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args([
                "--execute-xr-pattern-authority-test", "--confirm-gantry-attached",
                "--confirm-estop-ready", "--confirm-regular-mode-501-0",
                "--confirm-xr-message-pattern-reviewed",
            ])

    def test_default_output_is_unique_per_run(self):
        args = parse_args(["--print-plan", "--candidate-capture-scale-010"])
        self.assertIsNone(args.output)


class ContinuousArmControllerTests(unittest.TestCase):
    def make_controller(self, plant, clock, events, publish=True):
        controller = ContinuousArmController(
            plant.state,
            plant.sport,
            None,
            lambda name, details: events.append((name, details)),
            lambda q, dq: {i: 1.0 for i in ARM_JOINT_INDICES},
            publish_commands=publish,
            config=fast_config(),
            clock=clock.clock,
            monotonic_ns=clock.monotonic_ns,
            sleep=clock.sleep,
        )
        controller.observe_initial_pose()
        controller.attach_command_sink(plant.command)
        return controller

    def test_full_raise_and_release_uses_continuous_arm_sdk_phases(self):
        clock, events = FakeClock(), []
        plant = Plant(clock)
        controller = self.make_controller(plant, clock, events)
        offsets = {index: (0.1 if index == 25 else 0.0) for index in ARM_JOINT_INDICES}

        controller.raise_arm(offsets)
        self.assertEqual(controller.phase, "raised_hold")
        self.assertAlmostEqual(plant.positions[25], 0.1)
        controller.hold_once()
        controller.release_arm()

        self.assertEqual(controller.phase, "released")
        self.assertEqual(controller.authority_weight, 0.0)
        self.assertAlmostEqual(plant.positions[25], 0.0)
        phases = [details["phase"] for name, details in events if name == "phase_started"]
        self.assertEqual(
            phases,
            ["initial_observe", "authority_acquire", "raise", "raised_settle", "return", "return_settle", "authority_release", "internal_control_return"],
        )
        weights = [weight for _, _, _, weight in plant.commands]
        self.assertEqual(weights[0], 0.0)
        self.assertIn(1.0, weights)
        self.assertEqual(weights[-1], 0.0)
        self.assertTrue(all(t == 0.0 for t in plant.commands[0][2]))
        full_authority = next(command for command in plant.commands if command[3] == 1.0)
        self.assertTrue(all(full_authority[2][i] == 1.0 for i in ARM_JOINT_INDICES))

    def test_release_waits_for_internal_mode_zero(self):
        clock, events = FakeClock(), []
        plant = Plant(clock)
        zero_weight_commands = 0

        def command(positions, velocities, torques, weight):
            nonlocal zero_weight_commands
            plant.command(positions, velocities, torques, weight)
            if weight > 0.0:
                plant.fsm_mode = 1
            elif plant.fsm_mode == 1:
                zero_weight_commands += 1
                if zero_weight_commands >= 3:
                    plant.fsm_mode = 0

        controller = ContinuousArmController(
            plant.state, plant.sport, None,
            lambda name, details: events.append((name, details)),
            lambda q, dq: {i: 1.0 for i in ARM_JOINT_INDICES},
            publish_commands=True, config=fast_config(), clock=clock.clock,
            monotonic_ns=clock.monotonic_ns, sleep=clock.sleep,
        )
        controller.observe_initial_pose()
        controller.attach_command_sink(command)
        offsets = {index: 0.0 for index in ARM_JOINT_INDICES}
        controller.raise_arm(offsets)
        controller.release_arm()
        self.assertEqual(plant.fsm_mode, 0)
        self.assertGreaterEqual(zero_weight_commands, 3)
        released = [details for name, details in events if name == "arm_released"][-1]
        self.assertEqual(released["internal_controller"], (501, 0))

    def test_authority_acquisition_holds_one_fixed_initial_pose(self):
        clock, events = FakeClock(), []
        plant = Plant(clock)
        plant.positions[15] = 0.2
        controller = self.make_controller(plant, clock, events)
        offsets = {index: 0.0 for index in ARM_JOINT_INDICES}
        controller.raise_arm(offsets)
        acquire = [
            details for name, details in events
            if name == "arm_sdk_command" and details["phase"] == "authority_acquire"
        ]
        self.assertTrue(acquire)
        self.assertEqual({row["positions"]["15"] for row in acquire}, {0.2})

    def test_exact_xr_semantics_step_authority_and_do_not_scale_torque(self):
        clock, events = FakeClock(), []
        plant = Plant(clock)
        controller = ContinuousArmController(
            plant.state, plant.sport, None,
            lambda name, details: events.append((name, details)),
            lambda q, dq: {i: 0.5 for i in ARM_JOINT_INDICES},
            publish_commands=True,
            config=replace(fast_config(), scale_feedforward_by_authority=False,
                           step_to_full_authority=True),
            clock=clock.clock, monotonic_ns=clock.monotonic_ns, sleep=clock.sleep,
        )
        controller.observe_initial_pose()
        controller.attach_command_sink(plant.command)
        controller.raise_arm({i: 0.0 for i in ARM_JOINT_INDICES})
        first = plant.commands[0]
        self.assertEqual(first[3], 1.0)
        self.assertTrue(all(first[2][i] == 0.5 for i in ARM_JOINT_INDICES))

    def test_unsupported_fsm_mode_aborts_before_next_command(self):
        clock, events = FakeClock(), []
        plant = Plant(clock)
        controller = self.make_controller(plant, clock, events)
        controller.phase = "raised_hold"
        controller.raised_pose = {index: 0.0 for index in ARM_JOINT_INDICES}
        plant.fsm_mode = 2
        with self.assertRaisesRegex(RuntimeError, "controller state changed"):
            controller.hold_once()
        self.assertEqual(plant.commands, [])

    def test_release_allows_native_controller_pose_blending(self):
        clock, events = FakeClock(), []
        plant = Plant(clock)
        controller = self.make_controller(plant, clock, events)
        controller.phase = "authority_release"
        controller.last_target = {i: 0.0 for i in ARM_JOINT_INDICES}
        controller.authority_weight = 0.7
        plant.positions[20] = 0.02
        controller._check_state()

    def test_mode_one_is_allowed_only_after_initial_observation(self):
        clock, events = FakeClock(), []
        plant = Plant(clock)
        plant.fsm_mode = 1
        controller = ContinuousArmController(
            plant.state, plant.sport, None, lambda *args: events.append(args),
            lambda q, dq: {i: 1.0 for i in ARM_JOINT_INDICES},
            publish_commands=True, config=fast_config(), clock=clock.clock,
            monotonic_ns=clock.monotonic_ns, sleep=clock.sleep,
        )
        with self.assertRaisesRegex(RuntimeError, "mode in \\[0\\]"):
            controller.observe_initial_pose()

        plant.fsm_mode = 0
        controller = self.make_controller(plant, clock, events)
        controller.phase = "raised_hold"
        controller.raised_pose = {index: 0.0 for index in ARM_JOINT_INDICES}
        plant.fsm_mode = 1
        controller.hold_once()
        self.assertEqual(len(plant.commands), 1)

    def test_publisher_cannot_be_attached_before_initial_observation(self):
        clock, events = FakeClock(), []
        plant = Plant(clock)
        controller = ContinuousArmController(
            plant.state, plant.sport, None, lambda *args: events.append(args),
            lambda q, dq: {i: 1.0 for i in ARM_JOINT_INDICES},
            publish_commands=True, config=fast_config(), clock=clock.clock,
            monotonic_ns=clock.monotonic_ns, sleep=clock.sleep,
        )
        with self.assertRaisesRegex(RuntimeError, "after initial observation"):
            controller.attach_command_sink(plant.command)

    def test_initial_observation_waits_for_subscriber_samples(self):
        clock, events = FakeClock(), []
        plant = Plant(clock)

        def delayed_sport():
            if clock.now < 0.1:
                return None, None
            return plant.sport()

        controller = ContinuousArmController(
            plant.state, delayed_sport, None,
            lambda name, details: events.append((name, details)),
            lambda q, dq: {i: 1.0 for i in ARM_JOINT_INDICES},
            publish_commands=True, config=fast_config(), clock=clock.clock,
            monotonic_ns=clock.monotonic_ns, sleep=clock.sleep,
        )
        controller.observe_initial_pose()
        self.assertEqual(controller.phase, "prepared")
        self.assertGreaterEqual(clock.now, 0.2)
        self.assertIn("initial_telemetry_ready", [name for name, _ in events])

    def test_sport_stream_allows_normal_14hz_jitter(self):
        clock, events = FakeClock(), []
        plant = Plant(clock)
        controller = self.make_controller(plant, clock, events)
        controller.phase = "raised_hold"
        controller.raised_pose = {index: 0.0 for index in ARM_JOINT_INDICES}
        sport_timestamp = clock.monotonic_ns()
        clock.sleep(0.15)
        controller.sport_supplier = lambda: (
            SimpleNamespace(fsm_id=501, fsm_mode=1), sport_timestamp
        )
        controller.state_supplier = lambda: plant.state()
        controller.hold_once()
        self.assertEqual(len(plant.commands), 1)

    def test_offsets_are_complete_and_bounded(self):
        clock, events = FakeClock(), []
        plant = Plant(clock)
        controller = self.make_controller(plant, clock, events)
        with self.assertRaisesRegex(ValueError, "exactly joints"):
            controller.raise_arm({25: 0.1})
        offsets = {index: 0.0 for index in ARM_JOINT_INDICES}
        offsets[25] = 0.36
        with self.assertRaisesRegex(ValueError, "exceeds"):
            controller.raise_arm(offsets)


if __name__ == "__main__":
    unittest.main()
