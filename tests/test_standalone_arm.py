import contextlib
import io
import unittest
from unittest import mock
from types import SimpleNamespace

from g1_standalone_arm_sequence import (
    ArmSdkCommandSink,
    LegacySportArmActions,
    LowStateMonitor,
    parse_args,
    require_rpc_success,
    require_rpc_success_or_defer_timeout,
    run_preflight,
    select_high_level_backend,
    validate_arm_action_fsm,
    xr_motion_mode_initialization,
)
from handshake.standalone_arm import (
    ARM_SDK_CONTROL_RATE_HZ,
    ARM_JOINT_INDICES,
    BoundedArmExecutor,
    BoundedArmPlan,
    arm_sdk_gains,
    capture_pose_centers,
    pose_failures,
    require_arm_displacement,
    wait_for_settled_state,
)


def low_state(position_overrides=None, velocity_overrides=None, torque_overrides=None):
    position_overrides = position_overrides or {}
    velocity_overrides = velocity_overrides or {}
    torque_overrides = torque_overrides or {}
    motors = [
        SimpleNamespace(
            q=position_overrides.get(index, 0.0),
            dq=velocity_overrides.get(index, 0.0),
            tau_est=torque_overrides.get(index, 0.0),
        )
        for index in range(30)
    ]
    return SimpleNamespace(motor_state=motors)


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def clock(self):
        return self.now

    def monotonic_ns(self):
        return int(self.now * 1e9)

    def sleep(self, duration):
        self.now += duration

    def consume(self, duration):
        self.now += duration


class StandaloneArmCliTests(unittest.TestCase):
    def assert_parse_error(self, argv):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args(argv)

    def test_offline_plan_requires_no_hardware_confirmation(self):
        args = parse_args(["--offline-plan-only"])
        self.assertTrue(args.offline_plan_only)

    def test_read_only_preflight_requires_no_physical_confirmation(self):
        args = parse_args(["--probe-preflight"])
        self.assertTrue(args.probe_preflight)
        self.assertEqual(args.high_level_backend, "arm-action")

    def test_action_list_timeout_does_not_block_valid_preflight(self):
        class ValidLoco:
            def get_fsm(self):
                return 501, 0

        class FailingArm:
            def get_action_list(self):
                raise RuntimeError("3104 timeout")

        events = []
        details = run_preflight(
            SimpleNamespace(latest_sport=lambda: (None, None)),
            ValidLoco(),
            FailingArm(),
            lambda event, data: events.append((event, data)),
            sport_timeout_seconds=0.0,
            probe_action_list=True,
        )
        self.assertEqual(details["failures"], [])
        self.assertIn("3104 timeout", details["advisories"][0])
        self.assertEqual(events[-1][0], "preflight_passed")

    def test_only_documented_arm_action_fsms_are_accepted(self):
        for fsm_id, fsm_mode in ((500, 0), (501, 99), (801, 0), (801, 3)):
            validate_arm_action_fsm(fsm_id, fsm_mode)
        with self.assertRaisesRegex(RuntimeError, "unsupported in FSM 5"):
            validate_arm_action_fsm(5, 0)
        with self.assertRaisesRegex(RuntimeError, "require FSM mode 0 or 3"):
            validate_arm_action_fsm(801, 2)

    def test_preflight_requires_locomotion_but_action_list_is_advisory(self):
        class FailingLoco:
            def get_fsm(self):
                raise RuntimeError("sport unavailable")

        class FailingArm:
            def get_action_list(self):
                raise RuntimeError("arm unavailable")

        events = []
        monitor = SimpleNamespace(latest_sport=lambda: (None, None))
        with self.assertRaisesRegex(RuntimeError, "locomotion FSM probe"):
            run_preflight(
                monitor,
                FailingLoco(),
                FailingArm(),
                lambda event, details: events.append((event, details)),
                sport_timeout_seconds=0.0,
                probe_action_list=True,
            )
        self.assertEqual(events[0][0], "preflight_observed")
        self.assertEqual(len(events[0][1]["failures"]), 1)
        self.assertEqual(len(events[0][1]["advisories"]), 1)

    def test_legacy_backend_requires_explicit_selection(self):
        calls = []
        loco = SimpleNamespace(
            set_task_id=lambda task_id: calls.append(task_id) or 0,
        )
        modern = SimpleNamespace()
        backend, evidence = select_high_level_backend(loco, modern, "legacy-sport")
        self.assertIsInstance(backend, LegacySportArmActions)
        self.assertEqual(evidence["backend"], "legacy_sport_task")
        self.assertEqual(evidence["selection_basis"], "explicit_operator_override")
        self.assertEqual(backend.execute_raise(), 0)
        self.assertEqual(backend.execute_release(), 0)
        self.assertEqual(calls, [2, 3])

    def test_modern_backend_is_default_without_introspection(self):
        loco = SimpleNamespace()
        modern = SimpleNamespace()
        backend, evidence = select_high_level_backend(loco, modern)
        self.assertIs(backend, modern)
        self.assertEqual(evidence["backend"], "arm_action")
        self.assertEqual(
            evidence["selection_basis"],
            "configured_default_and_physical_raise_evidence",
        )

    def test_unknown_backend_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "unsupported high-level backend"):
            select_high_level_backend(SimpleNamespace(), SimpleNamespace(), "auto")

    def test_legacy_task_ids_cannot_be_remapped(self):
        self.assert_parse_error(["--offline-plan-only", "--legacy-release-task-id", "99"])

    def test_nonzero_high_level_rpc_result_is_failure(self):
        self.assertEqual(require_rpc_success("shake hand", 0), 0)
        with self.assertRaisesRegex(RuntimeError, "3104: RPC API timeout"):
            require_rpc_success("shake hand", 3104)
        with self.assertRaisesRegex(RuntimeError, "7400: rt/armsdk is occupied"):
            require_rpc_success("release arm", 7400)

    def test_timeout_can_only_be_deferred_when_explicitly_enabled(self):
        events = []
        with self.assertRaisesRegex(RuntimeError, "3104: RPC API timeout"):
            require_rpc_success_or_defer_timeout(
                "shake hand", 3104, False, lambda *item: events.append(item)
            )
        self.assertEqual(events, [])
        self.assertEqual(
            require_rpc_success_or_defer_timeout(
                "shake hand", 3104, True, lambda *item: events.append(item)
            ),
            3104,
        )
        self.assertEqual(
            events[0][0], "high_level_rpc_timeout_pending_telemetry_verification"
        )

    def test_physical_modes_require_gantry_and_estop(self):
        self.assert_parse_error(["--capture-post-action-pose"])

    def test_dry_arm_sdk_requires_reviewed_pose(self):
        self.assert_parse_error(
            [
                "--dry-run-arm-sdk",
                "--confirm-gantry-attached",
                "--confirm-estop-ready",
            ]
        )

    def test_publication_is_hard_disabled_even_with_all_old_acknowledgements(self):
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr), self.assertRaises(SystemExit):
            parse_args(
                [
                    "--execute-arm-sdk",
                    "--confirm-gantry-attached",
                    "--confirm-estop-ready",
                    "--confirm-arm-ownership-reviewed",
                    "--post-action-pose-rad",
                    *(["0"] * 14),
                    "--safe-return-pose-rad",
                    *(["0"] * 14),
                ]
            )
        self.assertIn("--execute-arm-sdk is disabled", stderr.getvalue())


class StandaloneArmCoreTests(unittest.TestCase):
    def test_low_state_safety_updates_are_not_limited_by_recording_rate(self):
        class Recorder:
            def __init__(self):
                self.rows = []

            def record(self, *args, **kwargs):
                self.rows.append((args, kwargs))

        recorder = Recorder()
        monitor = LowStateMonitor(recorder, telemetry_record_rate_hz=100.0)
        states = [low_state({25: value}) for value in (0.0, 0.1, 0.2)]
        with mock.patch(
            "g1_standalone_arm_sequence.time.monotonic_ns",
            side_effect=(1_000_000_000, 1_004_000_000, 1_011_000_000),
        ), mock.patch(
            "g1_standalone_arm_sequence.unitree_lowstate_record",
            return_value={"synthetic": True},
        ):
            for state in states:
                monitor._receive(state)
        latest, received_ns = monitor.latest()
        self.assertIs(latest, states[-1])
        self.assertEqual(received_ns, 1_011_000_000)
        self.assertEqual(len(recorder.rows), 2)

    def test_action_requires_measured_arm_displacement(self):
        baseline = {index: 0.0 for index in ARM_JOINT_INDICES}
        evidence = require_arm_displacement(low_state({25: 0.2}), baseline, 0.1)
        self.assertEqual(evidence["joint_index"], 25)
        self.assertAlmostEqual(evidence["maximum_displacement_rad"], 0.2)
        with self.assertRaisesRegex(RuntimeError, "not verified by telemetry"):
            require_arm_displacement(low_state({25: 0.05}), baseline, 0.1)

    def test_plan_is_bounded_out_and_back(self):
        samples = BoundedArmPlan().samples()
        self.assertEqual(samples[0]["relative_position_rad"], 0.0)
        self.assertAlmostEqual(samples[-1]["relative_position_rad"], 0.0)
        self.assertAlmostEqual(max(row["relative_position_rad"] for row in samples), 0.02)
        self.assertEqual(BoundedArmPlan().sample_rate_hz, ARM_SDK_CONTROL_RATE_HZ)

    def test_command_targets_are_velocity_clipped_from_measured_state(self):
        fake = FakeClock()
        executor = BoundedArmExecutor(
            BoundedArmPlan(
                amplitude_rad=0.01,
                duration_seconds=0.5,
                sample_rate_hz=100.0,
                blend_seconds=0.25,
            ),
            lambda: (low_state(), fake.monotonic_ns()),
            lambda *_: None,
            lambda *_: None,
            publish_commands=False,
            max_command_velocity_rad_s=0.5,
            clock=fake.clock,
            monotonic_ns=fake.monotonic_ns,
            sleep=fake.sleep,
        )
        targets = [0.0] * 30
        targets[25] = 0.02
        clipped = executor._clip_targets(targets, [0.0] * 30)
        self.assertAlmostEqual(clipped[25], 0.005)

    def test_arm_sdk_cli_defaults_to_unitree_motion_control_rate(self):
        args = parse_args(["--offline-plan-only"])
        self.assertEqual(args.sample_rate_hz, ARM_SDK_CONTROL_RATE_HZ)

    def test_unitree_motion_mode_gain_map_is_used(self):
        self.assertEqual(arm_sdk_gains(15), (80.0, 3.0))
        self.assertEqual(arm_sdk_gains(25), (80.0, 3.0))
        self.assertEqual(arm_sdk_gains(19), (40.0, 1.5))
        self.assertEqual(arm_sdk_gains(28), (40.0, 1.5))
        with self.assertRaisesRegex(ValueError, "not a G1_29 arm joint"):
            arm_sdk_gains(14)

    def test_arm_sdk_configuration_matches_unitree_motion_mode(self):
        configuration = ArmSdkCommandSink.configuration()
        self.assertEqual(configuration["topic"], "rt/arm_sdk")
        self.assertEqual(configuration["motor_mode"], 1)
        self.assertEqual(configuration["authority_weight_joint"], 29)
        self.assertEqual(configuration["arm_joint_indices"], list(range(15, 29)))
        self.assertEqual(configuration["gains"]["25"], {"kp": 80.0, "kd": 3.0})
        self.assertEqual(configuration["gains"]["28"], {"kp": 40.0, "kd": 1.5})
        self.assertEqual(configuration["message_initialization"], "xr_teleoperate_g1_29_motion_mode")
        self.assertEqual(configuration["all_motor_indices_initialized_from_lowstate"], list(range(29)))

    def test_xr_motion_message_copies_mode_machine_and_all_measured_positions(self):
        state = low_state()
        state.mode_machine = 5
        for index, motor in enumerate(state.motor_state):
            motor.q = index / 100.0
        plan = xr_motion_mode_initialization(state)
        self.assertEqual((plan["mode_pr"], plan["mode_machine"]), (0, 5))
        self.assertEqual([row["q"] for row in plan["motors"]], [i / 100.0 for i in range(29)])
        self.assertEqual((plan["motors"][0]["kp"], plan["motors"][0]["kd"]), (300.0, 3.0))
        self.assertEqual((plan["motors"][4]["kp"], plan["motors"][4]["kd"]), (80.0, 3.0))
        self.assertEqual((plan["motors"][26]["kp"], plan["motors"][26]["kd"]), (40.0, 1.5))

    def test_pose_failures_report_every_joint_not_only_first(self):
        state = low_state({20: 0.02, 25: 0.50})
        expected = {index: 0.0 for index in ARM_JOINT_INDICES}
        failures = pose_failures(state, expected, 0.01, 0.10)
        self.assertEqual({failure["joint_index"] for failure in failures}, {20, 25})

    def test_settling_requires_sustained_valid_state(self):
        fake = FakeClock()
        state = low_state()
        events = []
        settled = wait_for_settled_state(
            lambda: (state, fake.monotonic_ns()),
            {index: 0.0 for index in ARM_JOINT_INDICES},
            lambda event, details: events.append((event, details)),
            required_duration_seconds=0.10,
            timeout_seconds=0.50,
            clock=fake.clock,
            monotonic_ns=fake.monotonic_ns,
            sleep=fake.sleep,
        )
        self.assertIs(settled, state)
        self.assertEqual([event for event, _ in events], ["settling_started", "settled"])

    def test_capture_uses_median_arm_pose(self):
        fake = FakeClock()
        calls = 0

        def supplier():
            nonlocal calls
            calls += 1
            value = 1.0 if calls == 2 else 0.5
            return low_state({25: value}), fake.monotonic_ns()

        centers = capture_pose_centers(
            supplier,
            0.06,
            clock=fake.clock,
            monotonic_ns=fake.monotonic_ns,
            sleep=fake.sleep,
        )
        self.assertEqual(centers[25], 0.5)

    def test_dry_run_computes_all_phases_without_sink_calls(self):
        fake = FakeClock()
        state = low_state({25: 0.2})
        events = []
        sink_calls = []
        executor = BoundedArmExecutor(
            BoundedArmPlan(
                amplitude_rad=0.01,
                duration_seconds=0.5,
                sample_rate_hz=50.0,
                blend_seconds=0.25,
            ),
            lambda: (state, fake.monotonic_ns()),
            lambda *args: sink_calls.append(args),
            lambda event, details: events.append((event, details)),
            publish_commands=False,
            clock=fake.clock,
            monotonic_ns=fake.monotonic_ns,
            sleep=fake.sleep,
        )
        outcome, _ = executor.run(state)
        commands = [details for event, details in events if event == "arm_sdk_command"]
        self.assertEqual(outcome, "completed")
        self.assertEqual(sink_calls, [])
        self.assertEqual({row["phase"] for row in commands}, {"blend_in", "trajectory", "release"})
        self.assertTrue(all(row["published"] is False for row in commands))
        self.assertEqual([row["sequence"] for row in commands], list(range(1, len(commands) + 1)))

    def test_deadline_scheduler_does_not_accumulate_logging_overhead(self):
        fake = FakeClock()
        state = low_state()
        events = []

        def event(name, details):
            fake.consume(0.001)
            events.append((name, details))

        executor = BoundedArmExecutor(
            BoundedArmPlan(
                amplitude_rad=0.01,
                duration_seconds=0.5,
                sample_rate_hz=250.0,
                blend_seconds=0.25,
            ),
            lambda: (state, fake.monotonic_ns()),
            lambda *_: None,
            event,
            publish_commands=False,
            clock=fake.clock,
            monotonic_ns=fake.monotonic_ns,
            sleep=fake.sleep,
        )
        outcome, _ = executor.run(state)
        commands = [d for e, d in events if e == "arm_sdk_command"]
        finished = [d for e, d in events if e == "movement_finished"][0]
        self.assertEqual(outcome, "completed")
        self.assertGreaterEqual(len(commands), 249)
        self.assertLessEqual(len(commands), 252)
        self.assertLessEqual(finished["maximum_schedule_lag_seconds"], 0.001)

    def test_state_limit_abort_after_authority_uses_controlled_release(self):
        fake = FakeClock()
        calls = 0
        events = []
        sink_calls = []

        def supplier():
            nonlocal calls
            calls += 1
            velocity = 2.0 if calls >= 3 else 0.0
            return low_state(velocity_overrides={25: velocity}), fake.monotonic_ns()

        state = low_state()
        executor = BoundedArmExecutor(
            BoundedArmPlan(duration_seconds=0.5, sample_rate_hz=50.0, blend_seconds=0.25),
            supplier,
            lambda positions, velocities, weight: sink_calls.append(weight),
            lambda event, details: events.append((event, details)),
            publish_commands=True,
            clock=fake.clock,
            monotonic_ns=fake.monotonic_ns,
            sleep=fake.sleep,
        )
        outcome, _ = executor.run(state)
        self.assertEqual(outcome, "aborted")
        self.assertTrue(any(weight > 0.0 for weight in sink_calls))
        self.assertEqual(sink_calls[-1], 0.0)
        self.assertIn("authority_release_finished", [event for event, _ in events])


if __name__ == "__main__":
    unittest.main()
