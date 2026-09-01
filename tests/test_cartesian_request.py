import json
import tempfile
import unittest
from pathlib import Path

from handshake.cartesian_command import (
    CartesianOscillation, CartesianPositionCommand, CartesianWorkspace,
)
from handshake.cartesian_request import CartesianMoveRequest


class CartesianMoveRequestTests(unittest.TestCase):
    def request(self):
        return CartesianMoveRequest(
            attempt_id="right-world-target-001",
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

    def test_round_trip_has_stable_canonical_hash(self):
        request = self.request()
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "request.json"
            request.write_new(path)
            loaded = CartesianMoveRequest.load(path)
        self.assertEqual(loaded, request)
        self.assertEqual(loaded.sha256, request.sha256)
        self.assertEqual(len(request.sha256), 64)

    def test_right_orientation_round_trips_and_is_hash_addressed(self):
        request = self.request()
        oriented = CartesianMoveRequest(
            attempt_id=request.attempt_id,
            command=CartesianPositionCommand(
                right_target_m=request.command.right_target_m,
                right_orientation=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
                duration_seconds=request.command.duration_seconds,
                maximum_displacement_m=request.command.maximum_displacement_m,
                maximum_joint_offset_rad=request.command.maximum_joint_offset_rad,
                maximum_joint_velocity_rad_s=request.command.maximum_joint_velocity_rad_s,
            ),
            left_workspace=request.left_workspace,
            right_workspace=request.right_workspace,
        )
        loaded = CartesianMoveRequest.from_mapping(oriented.as_dict())
        self.assertEqual(loaded, oriented)
        self.assertNotEqual(loaded.sha256, request.sha256)

    def test_right_orientation_must_be_a_rotation(self):
        with self.assertRaisesRegex(ValueError, "orthonormal"):
            CartesianPositionCommand(
                right_target_m=(0.02, -0.23, -0.10),
                right_orientation=((1.0, 0.0, 0.0), (0.0, 2.0, 0.0), (0.0, 0.0, 1.0)),
            )

    def test_oscillation_round_trips_and_changes_hash(self):
        request = self.request()
        oscillating = CartesianMoveRequest(
            attempt_id=request.attempt_id,
            command=CartesianPositionCommand(
                right_target_m=request.command.right_target_m,
                oscillation=CartesianOscillation(),
                duration_seconds=request.command.duration_seconds,
                maximum_displacement_m=request.command.maximum_displacement_m,
                maximum_joint_offset_rad=request.command.maximum_joint_offset_rad,
                maximum_joint_velocity_rad_s=request.command.maximum_joint_velocity_rad_s,
            ),
            left_workspace=request.left_workspace,
            right_workspace=request.right_workspace,
        )
        loaded = CartesianMoveRequest.from_mapping(oscillating.as_dict())
        self.assertEqual(loaded, oscillating)
        self.assertNotEqual(loaded.sha256, request.sha256)

    def test_oscillation_axis_must_be_unit_length(self):
        with self.assertRaisesRegex(ValueError, "unit vector"):
            CartesianOscillation(axis=(0.0, 0.0, 2.0))

    def test_reviewed_handshake_scale_bounds_are_allowed(self):
        command = CartesianPositionCommand(
            right_target_m=(0.265, -0.138, 0.110),
            duration_seconds=10.0,
            maximum_displacement_m=0.38,
            maximum_joint_offset_rad=0.9,
            maximum_joint_velocity_rad_s=0.16,
        )
        self.assertEqual(command.maximum_joint_offset_rad, 0.9)

    def test_unknown_fields_fail_closed(self):
        value = self.request().as_dict()
        value["command"]["unreviewed_override"] = True
        with self.assertRaisesRegex(ValueError, "missing or unknown fields"):
            CartesianMoveRequest.from_mapping(value)

    def test_existing_request_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "request.json"
            self.request().write_new(path)
            original = json.loads(path.read_text())
            with self.assertRaises(FileExistsError):
                self.request().write_new(path)
            self.assertEqual(json.loads(path.read_text()), original)

    def test_nonfinite_limit_fails_closed(self):
        with self.assertRaisesRegex(ValueError, "must be finite"):
            CartesianPositionCommand(
                right_target_m=(0.02, -0.23, -0.10),
                duration_seconds=float("nan"),
            )

    def test_twenty_second_coordinate_trajectory_is_allowed(self):
        command = CartesianPositionCommand(
            right_target_m=(0.04, -0.20, -0.10),
            duration_seconds=20.0,
        )
        self.assertEqual(command.duration_seconds, 20.0)

    def test_coordinate_trajectory_over_thirty_seconds_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "between 1 and 30"):
            CartesianPositionCommand(
                right_target_m=(0.04, -0.20, -0.10),
                duration_seconds=30.1,
            )


if __name__ == "__main__":
    unittest.main()
