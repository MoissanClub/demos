import math
import unittest
from pathlib import Path

from handshake.arm_feedforward import G1ArmGravityFeedforward, G1_ARM_JOINT_NAMES
from handshake.standalone_arm import ARM_JOINT_INDICES


URDF = Path("/home/dwei/xr_teleoperate/assets/g1/g1_body29_hand14.urdf")


@unittest.skipUnless(URDF.is_file(), "checked-out xr_teleoperate G1 model unavailable")
class G1ArmGravityFeedforwardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = G1ArmGravityFeedforward(URDF)

    def test_model_mapping_and_hash_are_explicit(self):
        config = self.model.configuration()
        self.assertEqual(config["sdk_joint_indices"], list(range(15, 29)))
        self.assertEqual(config["model_joint_names"], list(G1_ARM_JOINT_NAMES))
        self.assertEqual(len(config["urdf_sha256"]), 64)

    def test_known_authority_pose_produces_finite_bounded_torque(self):
        values = (.292, .210, -.005, .980, .145, .061, .012,
                  .292, -.218, .017, .982, -.126, .082, -.013)
        q = dict(zip(ARM_JOINT_INDICES, values))
        tau = self.model(q, {i: 0.0 for i in ARM_JOINT_INDICES})
        self.assertEqual(set(tau), set(ARM_JOINT_INDICES))
        self.assertTrue(all(math.isfinite(v) for v in tau.values()))
        self.assertAlmostEqual(tau[16], 1.9237, places=3)
        self.assertAlmostEqual(tau[23], -1.9675, places=3)

    def test_nonfinite_and_incomplete_inputs_fail_closed(self):
        zeros = {i: 0.0 for i in ARM_JOINT_INDICES}
        with self.assertRaisesRegex(ValueError, "exactly joints"):
            self.model({15: 0.0}, zeros)
        bad = dict(zeros)
        bad[15] = float("nan")
        with self.assertRaisesRegex(ValueError, "non-finite"):
            self.model(bad, zeros)

    def test_torque_bound_violation_fails_closed(self):
        limits = {i: 0.01 for i in ARM_JOINT_INDICES}
        strict = G1ArmGravityFeedforward(URDF, limits)
        q = {i: 0.2 for i in ARM_JOINT_INDICES}
        with self.assertRaisesRegex(RuntimeError, "exceeds"):
            strict(q, {i: 0.0 for i in ARM_JOINT_INDICES})


if __name__ == "__main__":
    unittest.main()
