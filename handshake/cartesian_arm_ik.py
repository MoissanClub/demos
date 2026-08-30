"""Offline G1-29 Cartesian arm IK following xr_teleoperate's solver pattern."""
from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, Mapping, Tuple

from handshake.arm_feedforward import (
    G1_ARM_JOINT_NAMES, G1_LOCKED_JOINT_NAMES, G1ArmGravityFeedforward,
)
from handshake.standalone_arm import ARM_JOINT_INDICES


class G1CartesianArmIK:
    """Constrained dual-arm SE(3) IK with continuity and gravity RNEA output.

    This is deliberately offline-only: the module has no Unitree SDK or DDS
    imports and exposes no publisher.
    """

    def __init__(self, urdf_path: Path):
        import numpy as np
        import pinocchio as pin

        self.np, self.pin = np, pin
        self.feedforward = G1ArmGravityFeedforward(urdf_path)
        self.robot = self.feedforward.robot
        for frame_name, joint_name in (
            ("L_ee", "left_wrist_yaw_joint"),
            ("R_ee", "right_wrist_yaw_joint"),
        ):
            self.robot.model.addFrame(pin.Frame(
                frame_name, self.robot.model.getJointId(joint_name),
                pin.SE3(np.eye(3), np.array([0.05, 0.0, 0.0])), pin.FrameType.OP_FRAME,
            ))
        # The feedforward wrapper created data before these operational frames.
        self.robot.data = self.robot.model.createData()
        self.left_frame = self.robot.model.getFrameId("L_ee")
        self.right_frame = self.robot.model.getFrameId("R_ee")


    def forward_kinematics(self, positions: Mapping[int, float]) -> Tuple[object, object]:
        q = self._vector(positions, "positions")
        self.pin.framesForwardKinematics(self.robot.model, self.robot.data, q)
        return (self.robot.data.oMf[self.left_frame].homogeneous.copy(),
                self.robot.data.oMf[self.right_frame].homogeneous.copy())

    def solve(self, left_target, right_target, previous: Mapping[int, float],
              max_joint_step_rad: float = 0.10) -> Dict[str, object]:
        np = self.np
        q0 = self._vector(previous, "previous positions")
        left = self._transform(left_target, "left target")
        right = self._transform(right_target, "right target")
        if not 0.001 <= max_joint_step_rad <= 0.40:
            raise ValueError("maximum joint step must be between 0.001 and 0.40 rad")
        q = q0.copy()
        translation_weight = math.sqrt(50.0)
        weights = np.diag([translation_weight] * 3 + [1.0] * 3 +
                          [translation_weight] * 3 + [1.0] * 3)
        damping = 1e-4
        continuity = 0.045
        converged = False
        for iteration in range(80):
            error = self._pose_error(q, left, right)
            if np.linalg.norm(error[:3]) < 1e-5 and np.linalg.norm(error[6:9]) < 1e-5 and np.linalg.norm(error[[3,4,5,9,10,11]]) < 1e-4:
                converged = True
                break
            # Numerical differentiation is intentional here. It avoids relying
            # on frame-Jacobian row/reference conventions while retaining the
            # same Pinocchio kinematics and optimization objective offline.
            epsilon = 1e-6
            jacobian = np.column_stack([
                (self._pose_error(q + np.eye(14)[joint] * epsilon, left, right) - error) / epsilon
                for joint in range(14)
            ])
            weighted_j = weights @ jacobian
            weighted_e = weights @ error
            # Damped least squares plus the upstream solver's continuity bias.
            lhs = weighted_j.T @ weighted_j + (damping + continuity) * np.eye(14)
            rhs = -weighted_j.T @ weighted_e + continuity * (q0 - q)
            delta = np.linalg.solve(lhs, rhs)
            delta = np.clip(delta, -0.02, 0.02)
            q = np.clip(q + delta, self.robot.model.lowerPositionLimit,
                        self.robot.model.upperPositionLimit)
        if q.shape != (14,) or not np.all(np.isfinite(q)):
            raise RuntimeError("Cartesian IK returned invalid joint output")
        step = float(np.max(np.abs(q - q0)))
        if step > max_joint_step_rad:
            raise RuntimeError(f"IK joint step {step:.4f} rad exceeds {max_joint_step_rad:.4f} rad")
        positions = dict(zip(ARM_JOINT_INDICES, map(float, q)))
        actual_left, actual_right = self.forward_kinematics(positions)
        left_error = float(np.linalg.norm(actual_left[:3, 3] - left[:3, 3]))
        right_error = float(np.linalg.norm(actual_right[:3, 3] - right[:3, 3]))
        left_rotation_error = float(np.linalg.norm(self.pin.log3(actual_left[:3, :3].T @ left[:3, :3])))
        right_rotation_error = float(np.linalg.norm(self.pin.log3(actual_right[:3, :3].T @ right[:3, :3])))
        if max(left_error, right_error) > 0.02:
            raise RuntimeError(f"IK translation residual too large: left={left_error:.4f}m right={right_error:.4f}m")
        if max(left_rotation_error, right_rotation_error) > 0.02:
            raise RuntimeError(
                f"IK rotation residual too large: left={left_rotation_error:.4f}rad "
                f"right={right_rotation_error:.4f}rad"
            )
        torques = self.feedforward(positions, {i: 0.0 for i in ARM_JOINT_INDICES})
        return {"positions_rad": positions, "feedforward_torques_nm": torques,
                "maximum_joint_step_rad": step,
                "strict_convergence": converged,
                "translation_error_m": {"left": left_error, "right": right_error},
                "rotation_error_rad": {"left": left_rotation_error, "right": right_rotation_error}}

    def plan_trajectory(self, left_target, right_target, initial: Mapping[int, float],
                        duration_seconds: float = 2.0, sample_rate_hz: float = 250.0,
                        max_joint_step_rad: float = 0.40,
                        max_joint_velocity_rad_s: float = 0.075):
        """Solve one Cartesian endpoint, then time-parameterize it in joint space."""
        if not 1.0 <= duration_seconds <= 10.0:
            raise ValueError("trajectory duration must be between 1 and 10 seconds")
        if not 50.0 <= sample_rate_hz <= 250.0:
            raise ValueError("sample rate must be between 50 and 250 Hz")
        if not 0.001 <= max_joint_step_rad <= 0.40:
            raise ValueError("maximum joint step must be between 0.001 and 0.40 rad")
        if not 0.001 <= max_joint_velocity_rad_s <= 0.075:
            raise ValueError("maximum joint velocity must be between 0.001 and 0.075 rad/s")
        endpoint = self.solve(
            left_target, right_target, initial,
            max_joint_step_rad=max_joint_step_rad,
        )
        q0 = self._vector(initial, "initial positions")
        q1 = self._vector(endpoint["positions_rad"], "endpoint positions")
        count = int(round(duration_seconds * sample_rate_hz)) + 1
        samples = []
        maximum_velocity = 0.0
        for index in range(count):
            fraction = index / (count - 1)
            blend = 10 * fraction**3 - 15 * fraction**4 + 6 * fraction**5
            derivative = 30 * fraction**2 - 60 * fraction**3 + 30 * fraction**4
            q = q0 + (q1 - q0) * blend
            dq = (q1 - q0) * derivative / duration_seconds
            maximum_velocity = max(maximum_velocity, float(self.np.max(self.np.abs(dq))))
            positions = dict(zip(ARM_JOINT_INDICES, map(float, q)))
            velocities = dict(zip(ARM_JOINT_INDICES, map(float, dq)))
            torques = self.feedforward(positions, velocities)
            samples.append({"time_seconds": index / sample_rate_hz,
                            "positions_rad": positions,
                            "velocities_rad_s": velocities,
                            "feedforward_torques_nm": torques})
        if maximum_velocity > max_joint_velocity_rad_s:
            raise RuntimeError(
                f"Cartesian trajectory velocity {maximum_velocity:.4f} rad/s exceeds "
                f"{max_joint_velocity_rad_s:.4f} rad/s"
            )
        return {"duration_seconds": duration_seconds, "sample_rate_hz": sample_rate_hz,
                "sample_count": count, "maximum_joint_velocity_rad_s": maximum_velocity,
                "joint_step_limit_rad": max_joint_step_rad,
                "joint_velocity_limit_rad_s": max_joint_velocity_rad_s,
                "endpoint": endpoint, "samples": samples}

    def _pose_error(self, q, left, right):
        np = self.np
        self.pin.framesForwardKinematics(self.robot.model, self.robot.data, q)
        errors = []
        for frame, target in ((self.left_frame, left), (self.right_frame, right)):
            actual = self.robot.data.oMf[frame]
            errors.extend((target[:3, 3] - actual.translation).tolist())
            errors.extend(self.pin.log3(actual.rotation.T @ target[:3, :3]).tolist())
        return np.asarray(errors)

    def _vector(self, values: Mapping[int, float], label: str):
        np = self.np
        if set(values) != set(ARM_JOINT_INDICES):
            raise ValueError(f"{label} must specify exactly joints 15 through 28")
        vector = np.asarray([values[i] for i in ARM_JOINT_INDICES], dtype=float)
        if not np.all(np.isfinite(vector)):
            raise ValueError(f"{label} contains a non-finite value")
        return vector

    def _transform(self, value, label: str):
        np = self.np
        transform = np.asarray(value, dtype=float)
        if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
            raise ValueError(f"{label} must be a finite 4x4 transform")
        if not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-8):
            raise ValueError(f"{label} has an invalid homogeneous row")
        rotation = transform[:3, :3]
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5) or not math.isclose(
            float(np.linalg.det(rotation)), 1.0, abs_tol=1e-5
        ):
            raise ValueError(f"{label} rotation is not in SO(3)")
        return transform
