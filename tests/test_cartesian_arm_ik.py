import unittest
from pathlib import Path

import numpy as np

from handshake.cartesian_arm_ik import G1CartesianArmIK
from handshake.cartesian_command import (
    CartesianDeltaCommand,
    CartesianWorkspace,
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


if __name__ == "__main__":
    unittest.main()
