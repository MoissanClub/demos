"""Offline G1-29 Cartesian arm IK following xr_teleoperate's solver pattern."""
from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, Mapping, Optional, Tuple

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
              max_joint_step_rad: float = 0.10,
              initial_guess: Optional[Mapping[int, float]] = None) -> Dict[str, object]:
        np = self.np
        q0 = self._vector(previous, "previous positions")
        left = self._transform(left_target, "left target")
        right = self._transform(right_target, "right target")
        if not 0.001 <= max_joint_step_rad <= 0.90:
            raise ValueError("maximum joint step must be between 0.001 and 0.90 rad")
        q = (
            self._vector(initial_guess, "IK initial guess").copy()
            if initial_guess is not None else q0.copy()
        )
        translation_weight = math.sqrt(50.0)
        weights = np.diag([translation_weight] * 3 + [1.0] * 3 +
                          [translation_weight] * 3 + [1.0] * 3)
        damping = 1e-4
        continuity = 0.045
        converged = False
        for iteration in range(160):
            # The posture-continuity bias gives a stable nearby solution for
            # small moves. For larger reachable moves, finish with a lower
            # bias so endpoint accuracy is not traded away indefinitely.
            iteration_continuity = continuity if iteration < 80 else 0.035
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
            lhs = weighted_j.T @ weighted_j + (damping + iteration_continuity) * np.eye(14)
            rhs = -weighted_j.T @ weighted_e + iteration_continuity * (q0 - q)
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

    def solve_minimum_peak_speed(
        self,
        left_target,
        right_target,
        previous: Mapping[int, float],
        maximum_time_seconds: float,
        max_joint_step_rad: float = 0.40,
        max_candidates: int = 5,
    ) -> Dict[str, object]:
        """Choose the valid multi-start IK result with the lowest peak speed.

        A quintic point-to-point trajectory has peak blend derivative 1.875,
        so minimizing ``max(abs(q1-q0))`` also minimizes its peak joint speed
        for a fixed time budget. Candidate zero is the normal continuity solve;
        the remaining deterministic seeds explore alternate right-arm elbow and
        shoulder-yaw branches without changing the requested hand poses.
        """
        if not 1.0 <= maximum_time_seconds <= 30.0:
            raise ValueError("maximum time must be between 1 and 30 seconds")
        if not 1 <= max_candidates <= 9:
            raise ValueError("IK candidate count must be between 1 and 9")
        q0 = self._vector(previous, "previous positions")
        seed_specs = (
            (),
            ((10, 0.08),), ((10, -0.08),),
            ((9, 0.08),), ((9, -0.08),),
            ((11, 0.08),), ((11, -0.08),),
            ((13, 0.08),), ((13, -0.08),),
        )
        candidates = []
        failures = []
        for candidate_index, changes in enumerate(seed_specs[:max_candidates]):
            seed = q0.copy()
            for local_index, offset in changes:
                seed[local_index] += offset
            seed = self.np.clip(
                seed, self.robot.model.lowerPositionLimit,
                self.robot.model.upperPositionLimit,
            )
            try:
                endpoint = self.solve(
                    left_target, right_target, previous,
                    max_joint_step_rad=max_joint_step_rad,
                    initial_guess=dict(zip(ARM_JOINT_INDICES, map(float, seed))),
                )
            except RuntimeError as exc:
                failures.append({"candidate_index": candidate_index, "reason": str(exc)})
                continue
            delta = max(
                abs(float(endpoint["positions_rad"][joint]) - float(previous[joint]))
                for joint in ARM_JOINT_INDICES
            )
            peak_speed = 1.875 * delta / maximum_time_seconds
            candidates.append((peak_speed, delta, candidate_index, endpoint))
        if not candidates:
            raise RuntimeError(f"no valid IK candidates: {failures}")
        peak_speed, delta, candidate_index, endpoint = min(
            candidates, key=lambda item: (item[0], item[1], item[2])
        )
        return {
            **endpoint,
            "ik_selection": {
                "method": "deterministic_multistart_minimum_peak_joint_speed",
                "attempted_candidates": min(max_candidates, len(seed_specs)),
                "valid_candidates": len(candidates),
                "selected_candidate_index": candidate_index,
                "maximum_time_seconds": maximum_time_seconds,
                "predicted_peak_joint_speed_rad_s": peak_speed,
                "maximum_joint_delta_rad": delta,
                "failures": failures,
            },
        }

    def plan_minimum_peak_speed_trajectory(
        self, left_target, right_target, initial: Mapping[int, float],
        maximum_time_seconds: float, sample_rate_hz: float = 250.0,
        max_joint_step_rad: float = 0.40,
        max_joint_velocity_rad_s: float = 0.075,
        max_candidates: int = 5,
    ):
        """Plan at the time budget using the valid IK with least peak speed."""
        if not 50.0 <= sample_rate_hz <= 250.0:
            raise ValueError("sample rate must be between 50 and 250 Hz")
        if not 0.001 <= max_joint_step_rad <= 0.90:
            raise ValueError("maximum joint step must be between 0.001 and 0.90 rad")
        if not 0.001 <= max_joint_velocity_rad_s <= 0.20:
            raise ValueError("maximum joint velocity must be between 0.001 and 0.20 rad/s")
        endpoint = self.solve_minimum_peak_speed(
            left_target, right_target, initial, maximum_time_seconds,
            max_joint_step_rad=max_joint_step_rad,
            max_candidates=max_candidates,
        )
        return self._time_parameterize(
            endpoint, initial, maximum_time_seconds, sample_rate_hz,
            max_joint_step_rad, max_joint_velocity_rad_s,
        )

    def plan_trajectory(self, left_target, right_target, initial: Mapping[int, float],
                        duration_seconds: float = 2.0, sample_rate_hz: float = 250.0,
                        max_joint_step_rad: float = 0.40,
                        max_joint_velocity_rad_s: float = 0.075):
        """Solve one Cartesian endpoint, then time-parameterize it in joint space."""
        if not 1.0 <= duration_seconds <= 30.0:
            raise ValueError("trajectory duration must be between 1 and 30 seconds")
        if not 50.0 <= sample_rate_hz <= 250.0:
            raise ValueError("sample rate must be between 50 and 250 Hz")
        if not 0.001 <= max_joint_step_rad <= 0.90:
            raise ValueError("maximum joint step must be between 0.001 and 0.90 rad")
        if not 0.001 <= max_joint_velocity_rad_s <= 0.20:
            raise ValueError("maximum joint velocity must be between 0.001 and 0.20 rad/s")
        endpoint = self.solve(
            left_target, right_target, initial,
            max_joint_step_rad=max_joint_step_rad,
        )
        return self._time_parameterize(
            endpoint, initial, duration_seconds, sample_rate_hz,
            max_joint_step_rad, max_joint_velocity_rad_s,
        )

    def plan_cartesian_oscillation(
        self, left_target, right_center, raised: Mapping[int, float], *,
        axis, amplitude_m: float, frequency_hz: float, duration_seconds: float,
        waypoint_rate_hz: float, sample_rate_hz: float,
        max_joint_velocity_rad_s: float, max_joint_acceleration_rad_s2: float,
        waveform: str = "enveloped_sine",
    ):
        """Warm-start Cartesian waypoints, then spline a closed joint cycle."""
        from scipy.interpolate import CubicSpline

        np = self.np
        axis = np.asarray(axis, dtype=float)
        left = self._transform(left_target, "oscillation left target")
        center = self._transform(right_center, "oscillation right center")
        q_center = self._vector(raised, "raised positions")
        waypoint_count = int(round(duration_seconds * waypoint_rate_hz)) + 1
        waypoint_times = np.linspace(0.0, duration_seconds, waypoint_count)

        def offset_at(time_seconds):
            if waveform == "raised_cosine_squared":
                rise = (1.0 - math.cos(2.0 * math.pi * frequency_hz * time_seconds)) / 2.0
                return amplitude_m * rise**2
            envelope = math.sin(math.pi * time_seconds / duration_seconds) ** 2
            return amplitude_m * envelope * math.sin(2.0 * math.pi * frequency_hz * time_seconds)

        extrema = {}
        waypoint_errors = []
        for sign in (-1.0, 1.0):
            right = center.copy()
            right[:3, 3] += axis * amplitude_m * sign
            solved = self.solve(
                left, right, raised, max_joint_step_rad=0.10,
                initial_guess=raised,
            )
            if max(solved["translation_error_m"].values()) > 0.005:
                raise RuntimeError(
                    f"oscillation {sign:+.0f} amplitude translation residual exceeds 0.005 m"
                )
            if max(solved["rotation_error_rad"].values()) > 0.015:
                raise RuntimeError(
                    f"oscillation {sign:+.0f} amplitude rotation residual exceeds 0.015 rad"
                )
            extrema[sign] = self._vector(solved["positions_rad"], "oscillation extremum")
            waypoint_errors.append({
                "normalized_amplitude": sign,
                "translation_error_m": solved["translation_error_m"],
                "rotation_error_rad": solved["rotation_error_rad"],
            })
        linear = (extrema[1.0] - extrema[-1.0]) / (2.0 * amplitude_m)
        quadratic = (
            extrema[1.0] + extrema[-1.0] - 2.0 * q_center
        ) / (2.0 * amplitude_m**2)
        waypoint_q = np.asarray([
            q_center + linear * offset_at(float(timestamp))
            + quadratic * offset_at(float(timestamp)) ** 2
            for timestamp in waypoint_times
        ])
        spline = CubicSpline(
            waypoint_times, waypoint_q, axis=0,
            bc_type=((1, np.zeros(14)), (1, np.zeros(14))),
        )
        sample_count = int(round(duration_seconds * sample_rate_hz)) + 1
        sample_times = np.linspace(0.0, duration_seconds, sample_count)
        q_samples = spline(sample_times)
        dq_samples = spline(sample_times, 1)
        ddq_samples = spline(sample_times, 2)
        maximum_velocity = float(np.max(np.abs(dq_samples)))
        maximum_acceleration = float(np.max(np.abs(ddq_samples)))
        maximum_center_offset = float(np.max(np.abs(q_samples - q_center)))
        if maximum_velocity > max_joint_velocity_rad_s:
            raise RuntimeError(
                f"oscillation joint velocity {maximum_velocity:.4f} rad/s exceeds "
                f"{max_joint_velocity_rad_s:.4f} rad/s"
            )
        if maximum_acceleration > max_joint_acceleration_rad_s2:
            raise RuntimeError(
                f"oscillation joint acceleration {maximum_acceleration:.4f} rad/s^2 exceeds "
                f"{max_joint_acceleration_rad_s2:.4f} rad/s^2"
            )
        if np.any(q_samples < self.robot.model.lowerPositionLimit - 1e-9) or np.any(
            q_samples > self.robot.model.upperPositionLimit + 1e-9
        ):
            raise RuntimeError("oscillation spline exceeds a model joint limit")
        samples = []
        maximum_translation_error = 0.0
        maximum_rotation_error = 0.0
        for index, timestamp in enumerate(sample_times):
            positions = dict(zip(ARM_JOINT_INDICES, map(float, q_samples[index])))
            velocities = dict(zip(ARM_JOINT_INDICES, map(float, dq_samples[index])))
            actual_left, actual_right = self.forward_kinematics(positions)
            desired_right = center.copy()
            desired_right[:3, 3] += axis * offset_at(float(timestamp))
            translation_error = max(
                float(np.linalg.norm(actual_left[:3, 3] - left[:3, 3])),
                float(np.linalg.norm(actual_right[:3, 3] - desired_right[:3, 3])),
            )
            rotation_error = max(
                float(np.linalg.norm(self.pin.log3(actual_left[:3, :3].T @ left[:3, :3]))),
                float(np.linalg.norm(self.pin.log3(actual_right[:3, :3].T @ desired_right[:3, :3]))),
            )
            maximum_translation_error = max(maximum_translation_error, translation_error)
            maximum_rotation_error = max(maximum_rotation_error, rotation_error)
            samples.append({
                "time_seconds": float(timestamp),
                "positions_rad": positions,
                "velocities_rad_s": velocities,
                "feedforward_torques_nm": self.feedforward(positions, velocities),
            })
        if maximum_translation_error > 0.006:
            raise RuntimeError(
                f"oscillation spline translation residual {maximum_translation_error:.4f} m exceeds 0.006 m"
            )
        if maximum_rotation_error > 0.016:
            raise RuntimeError(
                f"oscillation spline rotation residual {maximum_rotation_error:.4f} rad exceeds 0.016 rad"
            )
        return {
            "duration_seconds": duration_seconds,
            "sample_rate_hz": sample_rate_hz,
            "sample_count": sample_count,
            "waypoint_rate_hz": waypoint_rate_hz,
            "waypoint_count": waypoint_count,
            "waveform": waveform,
            "maximum_joint_velocity_rad_s": maximum_velocity,
            "maximum_joint_acceleration_rad_s2": maximum_acceleration,
            "maximum_joint_offset_from_center_rad": maximum_center_offset,
            "maximum_translation_error_m": maximum_translation_error,
            "maximum_rotation_error_rad": maximum_rotation_error,
            "waypoint_errors": waypoint_errors,
            "samples": samples,
        }

    def _time_parameterize(
        self, endpoint, initial, duration_seconds, sample_rate_hz,
        max_joint_step_rad, max_joint_velocity_rad_s,
    ):
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
