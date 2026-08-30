import contextlib
import io
import unittest
from unittest.mock import patch

import run_g1_reviewed_cartesian_test as reviewed


class ReviewedCartesianPhysicalTests(unittest.TestCase):
    def test_only_exact_reviewed_attempt_is_enabled(self):
        args = reviewed.parse_args([
            "--execute-reviewed-attempt", reviewed.ATTEMPT_ID,
            "--confirm-area-clear", "--confirm-estop-ready",
            "--confirm-regular-mode-501-0", "--confirm-plan-reviewed",
        ])
        self.assertEqual(args.execute_reviewed_attempt, reviewed.ATTEMPT_ID)

    def test_enabled_attempt_requires_every_confirmation(self):
        with patch.object(reviewed, "PHYSICAL_EXECUTION_ENABLED", True):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                reviewed.parse_args([
                    "--execute-reviewed-attempt", reviewed.ATTEMPT_ID,
                    "--confirm-area-clear", "--confirm-regular-mode-501-0",
                    "--confirm-plan-reviewed",
                ])

    def test_reviewed_command_is_exact_and_bounded(self):
        self.assertEqual(reviewed.RIGHT_DELTA_M, (0.01, 0.0, 0.0))
        self.assertEqual(reviewed.COMMAND.maximum_displacement_m, 0.01)
        self.assertEqual(reviewed.COMMAND.maximum_joint_offset_rad, 0.05)
        self.assertEqual(reviewed.COMMAND.maximum_joint_velocity_rad_s, 0.02)
        self.assertEqual(reviewed.COMMAND.duration_seconds, 8.0)


if __name__ == "__main__":
    unittest.main()
