"""Bounded, offline Cartesian command interface for the G1 arms.

This module deliberately exposes planning only.  It has no Unitree SDK, DDS,
or publisher imports; a reviewed runtime must separately authorize execution.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Mapping, Optional, Sequence, Tuple

from handshake.cartesian_arm_ik import G1CartesianArmIK


Vector3 = Tuple[float, float, float]
Matrix3 = Tuple[Vector3, Vector3, Vector3]


@dataclass(frozen=True)
class CartesianOscillation:
    """Smooth Cartesian oscillation about a reviewed raised hand pose."""

    axis: Vector3 = (0.0, 0.0, 1.0)
    amplitude_m: float = 0.005
    frequency_hz: float = 0.25
    duration_seconds: float = 8.0
    waypoint_rate_hz: float = 8.0
    maximum_joint_velocity_rad_s: float = 0.08
    maximum_joint_acceleration_rad_s2: float = 0.20
    waveform: str = "enveloped_sine"

    def __post_init__(self) -> None:
        axis = _vector3(self.axis, "oscillation axis")
        norm = math.sqrt(sum(value * value for value in axis))
        if not math.isclose(norm, 1.0, abs_tol=1e-6):
            raise ValueError("oscillation axis must be a unit vector")
        object.__setattr__(self, "axis", axis)
        values = (
            self.amplitude_m, self.frequency_hz, self.duration_seconds,
            self.waypoint_rate_hz, self.maximum_joint_velocity_rad_s,
            self.maximum_joint_acceleration_rad_s2,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("oscillation parameters must be finite")
        if not 0.001 <= self.amplitude_m <= 0.02:
            raise ValueError("oscillation amplitude must be between 0.001 and 0.02 m")
        if not 0.1 <= self.frequency_hz <= 1.0:
            raise ValueError("oscillation frequency must be between 0.1 and 1.0 Hz")
        if not 2.0 <= self.duration_seconds <= 20.0:
            raise ValueError("oscillation duration must be between 2 and 20 seconds")
        if not 4.0 <= self.waypoint_rate_hz <= 20.0:
            raise ValueError("oscillation waypoint rate must be between 4 and 20 Hz")
        if not 0.01 <= self.maximum_joint_velocity_rad_s <= 0.20:
            raise ValueError("oscillation joint velocity limit must be between 0.01 and 0.20 rad/s")
        if not 0.02 <= self.maximum_joint_acceleration_rad_s2 <= 0.50:
            raise ValueError("oscillation joint acceleration limit must be between 0.02 and 0.50 rad/s^2")
        if self.waveform not in {"enveloped_sine", "raised_cosine_squared"}:
            raise ValueError("unsupported oscillation waveform")
        if self.waveform == "raised_cosine_squared" and not math.isclose(
            self.duration_seconds * self.frequency_hz,
            round(self.duration_seconds * self.frequency_hz), abs_tol=1e-9,
        ):
            raise ValueError("raised-cosine oscillation duration must contain whole cycles")


def _vector3(values: Sequence[float], label: str) -> Vector3:
    if len(values) != 3:
        raise ValueError(f"{label} must contain exactly three values")
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} contains a non-finite value")
    return result  # type: ignore[return-value]


def _rotation3(values: Sequence[Sequence[float]], label: str) -> Matrix3:
    if len(values) != 3:
        raise ValueError(f"{label} must contain exactly three rows")
    rows = tuple(_vector3(row, f"{label} row") for row in values)
    for i in range(3):
        for j in range(3):
            dot = sum(rows[k][i] * rows[k][j] for k in range(3))
            if not math.isclose(dot, 1.0 if i == j else 0.0, abs_tol=1e-5):
                raise ValueError(f"{label} must be orthonormal")
    determinant = (
        rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
        - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
        + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0])
    )
    if not math.isclose(determinant, 1.0, abs_tol=1e-5):
        raise ValueError(f"{label} determinant must be one")
    return rows  # type: ignore[return-value]


@dataclass(frozen=True)
class CartesianWorkspace:
    """Reviewed world-frame axis-aligned bounds for one hand endpoint."""

    minimum_m: Vector3
    maximum_m: Vector3

    def __post_init__(self) -> None:
        minimum = _vector3(self.minimum_m, "workspace minimum")
        maximum = _vector3(self.maximum_m, "workspace maximum")
        if any(low >= high for low, high in zip(minimum, maximum)):
            raise ValueError("each workspace minimum must be less than its maximum")
        object.__setattr__(self, "minimum_m", minimum)
        object.__setattr__(self, "maximum_m", maximum)

    def require_contains(self, position: Sequence[float], label: str) -> None:
        point = _vector3(position, label)
        if any(value < low or value > high for value, low, high in zip(
            point, self.minimum_m, self.maximum_m
        )):
            raise RuntimeError(
                f"{label} {point} is outside reviewed workspace "
                f"[{self.minimum_m}, {self.maximum_m}]"
            )


@dataclass(frozen=True)
class CartesianDeltaCommand:
    """A relative dual-hand translation with explicit reviewed safety bounds."""

    right_delta_m: Vector3
    left_delta_m: Vector3 = (0.0, 0.0, 0.0)
    duration_seconds: float = 2.0
    sample_rate_hz: float = 250.0
    maximum_displacement_m: float = 0.10
    maximum_joint_offset_rad: float = 0.40
    maximum_joint_velocity_rad_s: float = 0.075

    def __post_init__(self) -> None:
        right = _vector3(self.right_delta_m, "right Cartesian delta")
        left = _vector3(self.left_delta_m, "left Cartesian delta")
        object.__setattr__(self, "right_delta_m", right)
        object.__setattr__(self, "left_delta_m", left)
        _validate_motion_bounds(self)
        for side, delta in (("left", left), ("right", right)):
            norm = math.sqrt(sum(value * value for value in delta))
            if norm > self.maximum_displacement_m + 1e-12:
                raise ValueError(
                    f"{side} Cartesian displacement {norm:.4f} m exceeds "
                    f"{self.maximum_displacement_m:.4f} m"
                )


@dataclass(frozen=True)
class CartesianPositionCommand:
    """Absolute world-frame hand positions with the current orientations held."""

    right_target_m: Optional[Vector3] = None
    left_target_m: Optional[Vector3] = None
    right_orientation: Optional[Matrix3] = None
    oscillation: Optional[CartesianOscillation] = None
    duration_seconds: float = 2.0
    sample_rate_hz: float = 250.0
    maximum_displacement_m: float = 0.02
    maximum_joint_offset_rad: float = 0.40
    maximum_joint_velocity_rad_s: float = 0.075

    def __post_init__(self) -> None:
        if self.right_target_m is None and self.left_target_m is None:
            raise ValueError("at least one absolute Cartesian target is required")
        if self.right_target_m is not None:
            object.__setattr__(
                self, "right_target_m",
                _vector3(self.right_target_m, "right Cartesian target"),
            )
        if self.left_target_m is not None:
            object.__setattr__(
                self, "left_target_m",
                _vector3(self.left_target_m, "left Cartesian target"),
            )
        if self.right_orientation is not None:
            if self.right_target_m is None:
                raise ValueError("right orientation requires a right Cartesian target")
            object.__setattr__(
                self, "right_orientation",
                _rotation3(self.right_orientation, "right target orientation"),
            )
        if self.oscillation is not None and self.right_target_m is None:
            raise ValueError("Cartesian oscillation requires a right Cartesian target")
        _validate_motion_bounds(self)


def _validate_motion_bounds(command) -> None:
    values = (
        command.maximum_displacement_m, command.maximum_joint_offset_rad,
        command.maximum_joint_velocity_rad_s, command.duration_seconds,
        command.sample_rate_hz,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("Cartesian motion bounds must be finite")
    if not 0.001 <= command.maximum_displacement_m <= 0.40:
        raise ValueError("maximum Cartesian displacement must be between 0.001 and 0.40 m")
    if not 0.001 <= command.maximum_joint_offset_rad <= 0.90:
        raise ValueError("maximum joint offset must be between 0.001 and 0.90 rad")
    if not 0.001 <= command.maximum_joint_velocity_rad_s <= 0.20:
        raise ValueError("maximum joint velocity must be between 0.001 and 0.20 rad/s")
    if not 1.0 <= command.duration_seconds <= 30.0:
        raise ValueError("trajectory duration must be between 1 and 30 seconds")
    if not 50.0 <= command.sample_rate_hz <= 250.0:
        raise ValueError("sample rate must be between 50 and 250 Hz")


class G1CartesianCommandInterface:
    """Turn a bounded relative Cartesian intent into an offline joint plan."""

    def __init__(self, planner: G1CartesianArmIK):
        self.planner = planner

    def plan(
        self,
        command: CartesianDeltaCommand,
        initial: Mapping[int, float],
        left_workspace: CartesianWorkspace,
        right_workspace: CartesianWorkspace,
    ):
        left_initial, right_initial = self.planner.forward_kinematics(initial)
        left_target, right_target = left_initial.copy(), right_initial.copy()
        left_target[:3, 3] += command.left_delta_m
        right_target[:3, 3] += command.right_delta_m

        left_workspace.require_contains(left_initial[:3, 3], "initial left hand")
        right_workspace.require_contains(right_initial[:3, 3], "initial right hand")
        left_workspace.require_contains(left_target[:3, 3], "target left hand")
        right_workspace.require_contains(right_target[:3, 3], "target right hand")

        result = self.planner.plan_trajectory(
            left_target,
            right_target,
            initial,
            command.duration_seconds,
            command.sample_rate_hz,
            max_joint_step_rad=command.maximum_joint_offset_rad,
            max_joint_velocity_rad_s=command.maximum_joint_velocity_rad_s,
        )
        result["command"] = {
            "frame": "world",
            "left_delta_m": list(command.left_delta_m),
            "right_delta_m": list(command.right_delta_m),
            "maximum_displacement_m": command.maximum_displacement_m,
            "maximum_joint_offset_rad": command.maximum_joint_offset_rad,
            "maximum_joint_velocity_rad_s": command.maximum_joint_velocity_rad_s,
        }
        result["workspace_m"] = {
            "left": {"minimum": list(left_workspace.minimum_m),
                     "maximum": list(left_workspace.maximum_m)},
            "right": {"minimum": list(right_workspace.minimum_m),
                      "maximum": list(right_workspace.maximum_m)},
        }
        return result

    def plan_position(
        self,
        command: CartesianPositionCommand,
        initial: Mapping[int, float],
        left_workspace: CartesianWorkspace,
        right_workspace: CartesianWorkspace,
        minimum_peak_speed: bool = False,
        max_ik_candidates: int = 5,
    ):
        left_initial, right_initial = self.planner.forward_kinematics(initial)
        left_target, right_target = left_initial.copy(), right_initial.copy()
        if command.left_target_m is not None:
            left_target[:3, 3] = command.left_target_m
        if command.right_target_m is not None:
            right_target[:3, 3] = command.right_target_m
        if command.right_orientation is not None:
            right_target[:3, :3] = command.right_orientation

        left_workspace.require_contains(left_initial[:3, 3], "initial left hand")
        right_workspace.require_contains(right_initial[:3, 3], "initial right hand")
        left_workspace.require_contains(left_target[:3, 3], "target left hand")
        right_workspace.require_contains(right_target[:3, 3], "target right hand")
        displacements = {
            "left": math.sqrt(sum(float(a - b) ** 2 for a, b in zip(
                left_target[:3, 3], left_initial[:3, 3]
            ))),
            "right": math.sqrt(sum(float(a - b) ** 2 for a, b in zip(
                right_target[:3, 3], right_initial[:3, 3]
            ))),
        }
        violations = [
            side for side, distance in displacements.items()
            if distance > command.maximum_displacement_m + 1e-12
        ]
        if violations:
            raise RuntimeError(
                f"absolute Cartesian target displacement exceeds "
                f"{command.maximum_displacement_m:.4f} m for {violations}"
            )

        plan_method = (
            self.planner.plan_minimum_peak_speed_trajectory
            if minimum_peak_speed else self.planner.plan_trajectory
        )
        extra = {"max_candidates": max_ik_candidates} if minimum_peak_speed else {}
        result = plan_method(
            left_target, right_target, initial, command.duration_seconds,
            command.sample_rate_hz,
            max_joint_step_rad=command.maximum_joint_offset_rad,
            max_joint_velocity_rad_s=command.maximum_joint_velocity_rad_s,
            **extra,
        )
        translation_error = result["endpoint"]["translation_error_m"]
        rotation_error = result["endpoint"]["rotation_error_rad"]
        if max(translation_error.values()) > 0.005:
            raise RuntimeError(
                "absolute Cartesian endpoint translation residual exceeds 0.005 m"
            )
        if max(rotation_error.values()) > 0.015:
            raise RuntimeError(
                "absolute Cartesian endpoint rotation residual exceeds 0.015 rad"
            )
        if command.oscillation is not None:
            oscillation = command.oscillation
            for sign in (-1.0, 1.0):
                extreme = [
                    float(right_target[i, 3]) + sign * oscillation.amplitude_m * oscillation.axis[i]
                    for i in range(3)
                ]
                right_workspace.require_contains(extreme, "oscillation right hand extreme")
            result["oscillation"] = self.planner.plan_cartesian_oscillation(
                left_target, right_target, result["endpoint"]["positions_rad"],
                axis=oscillation.axis,
                amplitude_m=oscillation.amplitude_m,
                frequency_hz=oscillation.frequency_hz,
                duration_seconds=oscillation.duration_seconds,
                waypoint_rate_hz=oscillation.waypoint_rate_hz,
                sample_rate_hz=command.sample_rate_hz,
                max_joint_velocity_rad_s=oscillation.maximum_joint_velocity_rad_s,
                max_joint_acceleration_rad_s2=oscillation.maximum_joint_acceleration_rad_s2,
                waveform=oscillation.waveform,
            )
        result["command"] = {
            "frame": "world",
            "type": "absolute_position",
            "left_target_m": list(map(float, left_target[:3, 3])),
            "right_target_m": list(map(float, right_target[:3, 3])),
            "right_orientation": (
                [list(map(float, row)) for row in command.right_orientation]
                if command.right_orientation is not None else None
            ),
            "left_displacement_m": displacements["left"],
            "right_displacement_m": displacements["right"],
            "maximum_displacement_m": command.maximum_displacement_m,
            "maximum_joint_offset_rad": command.maximum_joint_offset_rad,
            "maximum_joint_velocity_rad_s": command.maximum_joint_velocity_rad_s,
            "oscillation": (
                {
                    "axis": list(command.oscillation.axis),
                    "amplitude_m": command.oscillation.amplitude_m,
                    "frequency_hz": command.oscillation.frequency_hz,
                    "duration_seconds": command.oscillation.duration_seconds,
                    "waypoint_rate_hz": command.oscillation.waypoint_rate_hz,
                    "maximum_joint_velocity_rad_s": command.oscillation.maximum_joint_velocity_rad_s,
                    "maximum_joint_acceleration_rad_s2": command.oscillation.maximum_joint_acceleration_rad_s2,
                    "waveform": command.oscillation.waveform,
                } if command.oscillation is not None else None
            ),
        }
        result["workspace_m"] = {
            "left": {"minimum": list(left_workspace.minimum_m),
                     "maximum": list(left_workspace.maximum_m)},
            "right": {"minimum": list(right_workspace.minimum_m),
                      "maximum": list(right_workspace.maximum_m)},
        }
        return result


@dataclass(frozen=True)
class CoordinateMoveSafety:
    """Fixed reviewed bounds configured around the two-input move function."""

    left_workspace: CartesianWorkspace
    right_workspace: CartesianWorkspace
    maximum_displacement_m: float = 0.05
    maximum_joint_offset_rad: float = 0.22
    maximum_joint_velocity_rad_s: float = 0.075
    sample_rate_hz: float = 250.0
    max_ik_candidates: int = 5


class G1CoordinateMover:
    """Plan and execute a right-hand coordinate with a maximum time budget.

    Hardware-specific state reading and guarded execution are injected at
    construction. Consequently the movement call itself has the requested
    two-input shape while retaining explicit, reviewed safety configuration.
    The executor receives the complete evidence-ready trajectory and is the
    only component allowed to publish it.
    """

    def __init__(
        self,
        planner: G1CartesianArmIK,
        read_arm_positions: Callable[[], Mapping[int, float]],
        execute_trajectory: Callable[[Mapping[str, object]], None],
        safety: CoordinateMoveSafety,
    ):
        self.planner = planner
        self.read_arm_positions = read_arm_positions
        self.execute_trajectory = execute_trajectory
        self.safety = safety

    def move(self, coordinate_m: Sequence[float], maximum_time_seconds: float):
        """Move the right hand, choosing the IK with least peak joint speed."""
        coordinate = _vector3(coordinate_m, "right Cartesian coordinate")
        if not math.isfinite(float(maximum_time_seconds)):
            raise ValueError("maximum time must be finite")
        initial = dict(self.read_arm_positions())
        command = CartesianPositionCommand(
            right_target_m=coordinate,
            duration_seconds=float(maximum_time_seconds),
            sample_rate_hz=self.safety.sample_rate_hz,
            maximum_displacement_m=self.safety.maximum_displacement_m,
            maximum_joint_offset_rad=self.safety.maximum_joint_offset_rad,
            maximum_joint_velocity_rad_s=self.safety.maximum_joint_velocity_rad_s,
        )
        plan = G1CartesianCommandInterface(self.planner).plan_position(
            command,
            initial,
            self.safety.left_workspace,
            self.safety.right_workspace,
            minimum_peak_speed=True,
            max_ik_candidates=self.safety.max_ik_candidates,
        )
        self.execute_trajectory(plan)
        return plan
