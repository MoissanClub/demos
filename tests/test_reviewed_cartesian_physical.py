import contextlib
import io
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import run_g1_reviewed_cartesian_test as reviewed
from handshake.cartesian_command import CartesianPositionCommand, CartesianWorkspace
from handshake.cartesian_request import CartesianMoveRequest


class ReviewedCartesianPhysicalTests(unittest.TestCase):
    def request_file(self, root):
        request = CartesianMoveRequest(
            attempt_id="reviewed-coordinate-001",
            command=CartesianPositionCommand(
                right_target_m=(0.02, -0.23, -0.10),
                duration_seconds=8.0,
                maximum_displacement_m=0.01,
                maximum_joint_offset_rad=0.05,
                maximum_joint_velocity_rad_s=0.02,
            ),
            left_workspace=CartesianWorkspace(
                (-0.01, 0.21, -0.13), (0.04, 0.26, -0.08),
            ),
            right_workspace=CartesianWorkspace(
                (-0.01, -0.26, -0.13), (0.05, -0.21, -0.08),
            ),
        )
        path = Path(root) / "request.json"
        request.write_new(path)
        return request, path

    def test_retry_is_hard_disabled_after_execution(self):
        with tempfile.TemporaryDirectory() as temporary:
            request, path = self.request_file(temporary)
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                reviewed.parse_args([
                    "--execute-reviewed-request", str(path),
                    "--expect-request-sha256", request.sha256,
                    "--confirm-area-clear", "--confirm-estop-ready",
                    "--confirm-regular-mode-501-0", "--confirm-plan-reviewed",
                ])

    def test_enabled_attempt_requires_every_confirmation(self):
        with tempfile.TemporaryDirectory() as temporary:
            request, path = self.request_file(temporary)
            with patch.object(reviewed, "PHYSICAL_EXECUTION_ENABLED", True), \
                    patch.object(reviewed, "AUTHORIZED_REQUEST_SHA256", request.sha256):
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    reviewed.parse_args([
                        "--execute-reviewed-request", str(path),
                        "--expect-request-sha256", request.sha256,
                        "--confirm-area-clear", "--confirm-regular-mode-501-0",
                        "--confirm-plan-reviewed",
                    ])

    def test_request_hash_mismatch_fails_before_hardware_setup(self):
        with tempfile.TemporaryDirectory() as temporary:
            _, path = self.request_file(temporary)
            with patch.object(reviewed, "PHYSICAL_EXECUTION_ENABLED", True), \
                    patch.object(reviewed, "AUTHORIZED_REQUEST_SHA256", "0" * 64):
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    reviewed.parse_args([
                        "--execute-reviewed-request", str(path),
                        "--expect-request-sha256", "0" * 64,
                        "--confirm-area-clear", "--confirm-estop-ready",
                        "--confirm-regular-mode-501-0", "--confirm-plan-reviewed",
                    ])

    def test_enabled_runner_rejects_a_different_request_hash(self):
        with tempfile.TemporaryDirectory() as temporary:
            request, path = self.request_file(temporary)
            with patch.object(reviewed, "PHYSICAL_EXECUTION_ENABLED", True), \
                    patch.object(reviewed, "AUTHORIZED_REQUEST_SHA256", "f" * 64):
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    reviewed.parse_args([
                        "--execute-reviewed-request", str(path),
                        "--expect-request-sha256", request.sha256,
                        "--confirm-area-clear", "--confirm-estop-ready",
                        "--confirm-regular-mode-501-0", "--confirm-plan-reviewed",
                    ])

    def test_exact_hash_and_all_confirmations_pass_argument_gate(self):
        with tempfile.TemporaryDirectory() as temporary:
            request, path = self.request_file(temporary)
            with patch.object(reviewed, "PHYSICAL_EXECUTION_ENABLED", True), \
                    patch.object(reviewed, "AUTHORIZED_REQUEST_SHA256", request.sha256):
                args = reviewed.parse_args([
                    "--execute-reviewed-request", str(path),
                    "--expect-request-sha256", request.sha256,
                    "--confirm-area-clear", "--confirm-estop-ready",
                    "--confirm-regular-mode-501-0", "--confirm-plan-reviewed",
                ])
            self.assertEqual(args.move_request, request)


if __name__ == "__main__":
    unittest.main()
