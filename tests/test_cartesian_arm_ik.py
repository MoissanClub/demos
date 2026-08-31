import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from handshake.cartesian_arm_ik import G1CartesianArmIK
from handshake.cartesian_command import (
    CartesianDeltaCommand,
    CartesianPositionCommand,
    CartesianWorkspace,
    CoordinateMoveSafety,
    G1CoordinateMover,
    G1CartesianCommandInterface,
)
from handshake.standalone_arm import ARM_JOINT_INDICES

URDF = Path("/home/dwei/xr_teleoperate/assets/g1/g1_body29_hand14.urdf")


@unittest.skipUnless(URDF.is_file(), "checked-out G1 model unavailable")
class G1CartesianArmIKTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ik = G1CartesianArmIK(URDF)
        values = (.2936, .2151, -.0052, .9792, .0882, .0610, .0111,
                  .2926, -.2193, .0158, .9858, -.0499, .0222, -.0529)
        cls.initial = dict(zip(ARM_JOINT_INDICES, values))

    def test_zero_displacement_preserves_joint_pose(self):
        left, right = self.ik.forward_kinematics(self.initial)
        result = self.ik.solve(left, right, self.initial)
        self.assertEqual(result["maximum_joint_step_rad"], 0.0)
        self.assertLess(result["translation_error_m"]["right"], 1e-9)

    def test_one_millimeter_target_is_bounded(self):
        left, right = self.ik.forward_kinematics(self.initial)
        right[0, 3] += 0.001
        result = self.ik.solve(left, right, self.initial)
        self.assertLess(result["maximum_joint_step_rad"], 0.01)
        self.assertLess(result["translation_error_m"]["right"], 0.001)

    def test_one_millimeter_trajectory_is_smooth_and_bounded(self):
        left, right = self.ik.forward_kinematics(self.initial)
        right[0, 3] += 0.001
        result = self.ik.plan_trajectory(left, right, self.initial, 2.0, 50.0)
        self.assertEqual(result["sample_count"], 101)
        self.assertLess(result["maximum_joint_velocity_rad_s"], 0.01)
        self.assertTrue(all(v == 0.0 for v in result["samples"][0]["velocities_rad_s"].values()))
        self.assertTrue(all(v == 0.0 for v in result["samples"][-1]["velocities_rad_s"].values()))
        self.assertEqual(result["samples"][0]["positions_rad"], self.initial)
        self.assertEqual(result["samples"][-1]["positions_rad"], result["endpoint"]["positions_rad"])

    def test_invalid_transform_fails_closed(self):
        left, right = self.ik.forward_kinematics(self.initial)
        right[3, 3] = 2.0
        with self.assertRaisesRegex(ValueError, "homogeneous"):
            self.ik.solve(left, right, self.initial)

    def test_parameterized_command_preserves_intent_and_bounds(self):
        left, right = self.ik.forward_kinematics(self.initial)
        margin = np.array([0.02, 0.02, 0.02])
        interface = G1CartesianCommandInterface(self.ik)
        result = interface.plan(
            CartesianDeltaCommand(
                right_delta_m=(0.001, 0.0, 0.0),
                duration_seconds=2.0,
                sample_rate_hz=50.0,
                maximum_displacement_m=0.01,
                maximum_joint_offset_rad=0.01,
                maximum_joint_velocity_rad_s=0.01,
            ),
            self.initial,
            CartesianWorkspace(tuple(left[:3, 3] - margin), tuple(left[:3, 3] + margin)),
            CartesianWorkspace(tuple(right[:3, 3] - margin), tuple(right[:3, 3] + margin)),
        )
        self.assertEqual(result["command"]["right_delta_m"], [0.001, 0.0, 0.0])
        self.assertEqual(result["command"]["frame"], "world")
        self.assertEqual(result["joint_step_limit_rad"], 0.01)
        self.assertEqual(result["joint_velocity_limit_rad_s"], 0.01)

    def test_parameterized_command_rejects_vector_norm_over_limit(self):
        with self.assertRaisesRegex(ValueError, "right Cartesian displacement"):
            CartesianDeltaCommand(
                right_delta_m=(0.008, 0.008, 0.0),
                maximum_displacement_m=0.01,
            )

    def test_parameterized_command_rejects_target_outside_workspace(self):
        left, right = self.ik.forward_kinematics(self.initial)
        margin = np.array([0.002, 0.002, 0.002])
        interface = G1CartesianCommandInterface(self.ik)
        with self.assertRaisesRegex(RuntimeError, "target right hand.*outside"):
            interface.plan(
                CartesianDeltaCommand(
                    right_delta_m=(0.003, 0.0, 0.0),
                    maximum_displacement_m=0.01,
                ),
                self.initial,
                CartesianWorkspace(tuple(left[:3, 3] - margin), tuple(left[:3, 3] + margin)),
                CartesianWorkspace(tuple(right[:3, 3] - margin), tuple(right[:3, 3] + margin)),
            )

    def test_absolute_world_target_preserves_orientation_and_left_hand(self):
        left, right = self.ik.forward_kinematics(self.initial)
        target = tuple(right[:3, 3] + np.array([0.001, 0.0, 0.0]))
        margin = np.array([0.02, 0.02, 0.02])
        result = G1CartesianCommandInterface(self.ik).plan_position(
            CartesianPositionCommand(
                right_target_m=target,
                duration_seconds=2.0,
                sample_rate_hz=50.0,
                maximum_displacement_m=0.01,
                maximum_joint_offset_rad=0.01,
                maximum_joint_velocity_rad_s=0.01,
            ),
            self.initial,
            CartesianWorkspace(tuple(left[:3, 3] - margin), tuple(left[:3, 3] + margin)),
            CartesianWorkspace(tuple(right[:3, 3] - margin), tuple(right[:3, 3] + margin)),
        )
        self.assertEqual(result["command"]["type"], "absolute_position")
        self.assertTrue(np.allclose(result["command"]["right_target_m"], target))
        self.assertLess(result["command"]["left_displacement_m"], 1e-12)
        self.assertAlmostEqual(result["command"]["right_displacement_m"], 0.001)

    def test_absolute_target_rejects_displacement_over_limit(self):
        left, right = self.ik.forward_kinematics(self.initial)
        margin = np.array([0.05, 0.05, 0.05])
        with self.assertRaisesRegex(RuntimeError, "target displacement exceeds"):
            G1CartesianCommandInterface(self.ik).plan_position(
                CartesianPositionCommand(
                    right_target_m=tuple(right[:3, 3] + np.array([0.011, 0.0, 0.0])),
                    maximum_displacement_m=0.01,
                ),
                self.initial,
                CartesianWorkspace(tuple(left[:3, 3] - margin), tuple(left[:3, 3] + margin)),
                CartesianWorkspace(tuple(right[:3, 3] - margin), tuple(right[:3, 3] + margin)),
            )

    def test_absolute_target_requires_at_least_one_hand(self):
        with self.assertRaisesRegex(ValueError, "at least one"):
            CartesianPositionCommand()


class MinimumPeakSpeedSelectionTests(unittest.TestCase):
    def test_multistart_selects_smallest_maximum_joint_delta(self):
        ik = object.__new__(G1CartesianArmIK)
        ik.np = np
        ik.robot = SimpleNamespace(model=SimpleNamespace(
            lowerPositionLimit=np.full(14, -2.0),
            upperPositionLimit=np.full(14, 2.0),
        ))
        initial = dict.fromkeys(ARM_JOINT_INDICES, 0.0)

        def solve(_left, _right, _previous, max_joint_step_rad, initial_guess):
            seed = np.asarray([initial_guess[i] for i in ARM_JOINT_INDICES])
            # Candidate 2 (negative elbow seed) has the smallest max delta.
            delta = 0.10 if seed[10] > 0 else 0.04 if seed[10] < 0 else 0.08
            positions = dict(initial)
            positions[25] = delta
            return {
                "positions_rad": positions,
                "feedforward_torques_nm": dict(initial),
                "maximum_joint_step_rad": delta,
                "translation_error_m": {"left": 0.0, "right": 0.0},
                "rotation_error_rad": {"left": 0.0, "right": 0.0},
            }

        ik.solve = solve
        result = ik.solve_minimum_peak_speed(
            np.eye(4), np.eye(4), initial, 10.0, max_candidates=3,
        )
        selection = result["ik_selection"]
        self.assertEqual(selection["selected_candidate_index"], 2)
        self.assertAlmostEqual(selection["predicted_peak_joint_speed_rad_s"], 0.0075)

    def test_two_input_mover_plans_then_executes_selected_trajectory(self):
        initial = dict.fromkeys(ARM_JOINT_INDICES, 0.0)

        class FakePlanner:
            def forward_kinematics(self, _positions):
                left = np.eye(4)
                right = np.eye(4)
                left[:3, 3] = (0.0, 0.2, 0.0)
                right[:3, 3] = (0.0, -0.2, 0.0)
                return left, right

            def plan_minimum_peak_speed_trajectory(
                self, _left, _right, _initial, maximum_time, sample_rate,
                **kwargs,
            ):
                return {
                    "duration_seconds": maximum_time,
                    "sample_rate_hz": sample_rate,
                    "endpoint": {
                        "translation_error_m": {"left": 0.0, "right": 0.0},
                        "rotation_error_rad": {"left": 0.0, "right": 0.0},
                    },
                    "ik_selection": {"selected_candidate_index": 1},
                }

        executed = []
        safety = CoordinateMoveSafety(
            left_workspace=CartesianWorkspace((-0.1, 0.1, -0.1), (0.1, 0.3, 0.1)),
            right_workspace=CartesianWorkspace((-0.1, -0.3, -0.1), (0.1, -0.1, 0.1)),
            maximum_displacement_m=0.05,
        )
        mover = G1CoordinateMover(FakePlanner(), lambda: initial, executed.append, safety)
        plan = mover.move((0.01, -0.2, 0.0), 8.0)
        self.assertEqual(plan["duration_seconds"], 8.0)
        self.assertEqual(executed, [plan])


if __name__ == "__main__":
    unittest.main()
