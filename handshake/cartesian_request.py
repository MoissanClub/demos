"""Strict, hash-addressed intent for one reviewed Cartesian arm move."""
from __future__ import annotations

import hashlib
import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Mapping

from handshake.cartesian_command import (
    CartesianOscillation, CartesianPositionCommand, CartesianWorkspace,
)


SCHEMA_VERSION = "1.0"
_ATTEMPT_ID = re.compile(r"^[a-z0-9][a-z0-9_.-]*$")


@dataclass(frozen=True)
class CartesianMoveRequest:
    attempt_id: str
    command: CartesianPositionCommand
    left_workspace: CartesianWorkspace
    right_workspace: CartesianWorkspace

    def __post_init__(self) -> None:
        if not _ATTEMPT_ID.fullmatch(self.attempt_id) or ".." in self.attempt_id:
            raise ValueError("attempt_id must be a safe lowercase identifier")

    def as_dict(self) -> Dict[str, Any]:
        command = {
            "type": "absolute_position",
            "frame": "world",
            "right_target_m": self.command.right_target_m,
            "left_target_m": self.command.left_target_m,
            "duration_seconds": self.command.duration_seconds,
            "sample_rate_hz": self.command.sample_rate_hz,
            "maximum_displacement_m": self.command.maximum_displacement_m,
            "maximum_joint_offset_rad": self.command.maximum_joint_offset_rad,
            "maximum_joint_velocity_rad_s": self.command.maximum_joint_velocity_rad_s,
        }
        if self.command.right_orientation is not None:
            command["right_orientation"] = self.command.right_orientation
        if self.command.oscillation is not None:
            oscillation = self.command.oscillation
            command["oscillation"] = {
                "axis": oscillation.axis,
                "amplitude_m": oscillation.amplitude_m,
                "frequency_hz": oscillation.frequency_hz,
                "duration_seconds": oscillation.duration_seconds,
                "waypoint_rate_hz": oscillation.waypoint_rate_hz,
                "maximum_joint_velocity_rad_s": oscillation.maximum_joint_velocity_rad_s,
                "maximum_joint_acceleration_rad_s2": oscillation.maximum_joint_acceleration_rad_s2,
            }
            if oscillation.waveform != "enveloped_sine":
                command["oscillation"]["waveform"] = oscillation.waveform
        return {
            "schema_version": SCHEMA_VERSION,
            "attempt_id": self.attempt_id,
            "command": command,
            "workspace_m": {
                "left": {
                    "minimum": self.left_workspace.minimum_m,
                    "maximum": self.left_workspace.maximum_m,
                },
                "right": {
                    "minimum": self.right_workspace.minimum_m,
                    "maximum": self.right_workspace.maximum_m,
                },
            },
        }

    def canonical_json(self) -> str:
        return json.dumps(
            self.as_dict(), sort_keys=True, separators=(",", ":"),
            ensure_ascii=False,
        )

    @property
    def sha256(self) -> str:
        return hashlib.sha256(self.canonical_json().encode("utf-8")).hexdigest()

    def write_new(self, path: Path) -> None:
        with path.open("x", encoding="utf-8") as output:
            output.write(json.dumps(self.as_dict(), indent=2, sort_keys=True) + "\n")

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "CartesianMoveRequest":
        if value.get("schema_version") != SCHEMA_VERSION:
            raise ValueError("unsupported Cartesian request schema_version")
        if set(value) != {"schema_version", "attempt_id", "command", "workspace_m"}:
            raise ValueError("Cartesian request has missing or unknown top-level fields")
        command = value["command"]
        if not isinstance(command, Mapping) or command.get("type") != "absolute_position" \
                or command.get("frame") != "world":
            raise ValueError("Cartesian request must be an absolute world-frame position")
        expected_command = {
            "type", "frame", "right_target_m", "left_target_m", "duration_seconds",
            "sample_rate_hz", "maximum_displacement_m", "maximum_joint_offset_rad",
            "maximum_joint_velocity_rad_s",
        }
        optional_command = {"right_orientation", "oscillation"}
        if not expected_command <= set(command) or not set(command) <= expected_command | optional_command:
            raise ValueError("Cartesian request command has missing or unknown fields")
        oscillation = command.get("oscillation")
        if oscillation is not None:
            expected_oscillation = {
                "axis", "amplitude_m", "frequency_hz", "duration_seconds",
                "waypoint_rate_hz", "maximum_joint_velocity_rad_s",
                "maximum_joint_acceleration_rad_s2",
            }
            if not isinstance(oscillation, Mapping) or set(oscillation) not in (
                expected_oscillation, expected_oscillation | {"waveform"},
            ):
                raise ValueError("Cartesian request has invalid oscillation fields")
        workspaces = value["workspace_m"]
        if not isinstance(workspaces, Mapping) or set(workspaces) != {"left", "right"}:
            raise ValueError("Cartesian request must define both hand workspaces")
        for side in ("left", "right"):
            if not isinstance(workspaces[side], Mapping) \
                    or set(workspaces[side]) != {"minimum", "maximum"}:
                raise ValueError(f"Cartesian request has invalid {side} workspace")
        return cls(
            attempt_id=str(value["attempt_id"]),
            command=CartesianPositionCommand(
                right_target_m=command["right_target_m"],
                left_target_m=command["left_target_m"],
                right_orientation=command.get("right_orientation"),
                oscillation=(
                    CartesianOscillation(
                        axis=oscillation["axis"],
                        amplitude_m=float(oscillation["amplitude_m"]),
                        frequency_hz=float(oscillation["frequency_hz"]),
                        duration_seconds=float(oscillation["duration_seconds"]),
                        waypoint_rate_hz=float(oscillation["waypoint_rate_hz"]),
                        maximum_joint_velocity_rad_s=float(oscillation["maximum_joint_velocity_rad_s"]),
                        maximum_joint_acceleration_rad_s2=float(oscillation["maximum_joint_acceleration_rad_s2"]),
                        waveform=str(oscillation.get("waveform", "enveloped_sine")),
                    ) if oscillation is not None else None
                ),
                duration_seconds=float(command["duration_seconds"]),
                sample_rate_hz=float(command["sample_rate_hz"]),
                maximum_displacement_m=float(command["maximum_displacement_m"]),
                maximum_joint_offset_rad=float(command["maximum_joint_offset_rad"]),
                maximum_joint_velocity_rad_s=float(command["maximum_joint_velocity_rad_s"]),
            ),
            left_workspace=CartesianWorkspace(
                workspaces["left"]["minimum"], workspaces["left"]["maximum"],
            ),
            right_workspace=CartesianWorkspace(
                workspaces["right"]["minimum"], workspaces["right"]["maximum"],
            ),
        )

    @classmethod
    def load(cls, path: Path) -> "CartesianMoveRequest":
        value = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(value, Mapping):
            raise ValueError("Cartesian request root must be an object")
        return cls.from_mapping(value)
