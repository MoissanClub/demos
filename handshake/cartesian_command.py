"""Bounded, offline Cartesian command interface for the G1 arms.

This module deliberately exposes planning only.  It has no Unitree SDK, DDS,
or publisher imports; a reviewed runtime must separately authorize execution.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Sequence, Tuple

from handshake.cartesian_arm_ik import G1CartesianArmIK


Vector3 = Tuple[float, float, float]


def _vector3(values: Sequence[float], label: str) -> Vector3:
    if len(values) != 3:
        raise ValueError(f"{label} must contain exactly three values")
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} contains a non-finite value")
    return result  # type: ignore[return-value]


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
        if not 0.001 <= self.maximum_displacement_m <= 0.10:
            raise ValueError("maximum Cartesian displacement must be between 0.001 and 0.10 m")
        for side, delta in (("left", left), ("right", right)):
            norm = math.sqrt(sum(value * value for value in delta))
            if norm > self.maximum_displacement_m + 1e-12:
                raise ValueError(
                    f"{side} Cartesian displacement {norm:.4f} m exceeds "
                    f"{self.maximum_displacement_m:.4f} m"
                )
        if not 0.001 <= self.maximum_joint_offset_rad <= 0.40:
            raise ValueError("maximum joint offset must be between 0.001 and 0.40 rad")
        if not 0.001 <= self.maximum_joint_velocity_rad_s <= 0.075:
            raise ValueError("maximum joint velocity must be between 0.001 and 0.075 rad/s")
        if not 1.0 <= self.duration_seconds <= 10.0:
            raise ValueError("trajectory duration must be between 1 and 10 seconds")
        if not 50.0 <= self.sample_rate_hz <= 250.0:
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
