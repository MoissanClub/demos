"""Fail-closed Pinocchio gravity feedforward for the G1-29 arm joints."""
from __future__ import annotations

import hashlib
import math
from pathlib import Path
from typing import Dict, Mapping

from handshake.standalone_arm import ARM_JOINT_INDICES

G1_ARM_JOINT_NAMES = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint",
    "left_wrist_yaw_joint", "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
    "right_wrist_pitch_joint", "right_wrist_yaw_joint",
)
G1_LOCKED_JOINT_NAMES = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
    "left_ankle_pitch_joint", "left_ankle_roll_joint", "right_hip_pitch_joint",
    "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint",
    "right_ankle_pitch_joint", "right_ankle_roll_joint", "waist_yaw_joint",
    "waist_roll_joint", "waist_pitch_joint", "left_hand_thumb_0_joint",
    "left_hand_thumb_1_joint", "left_hand_thumb_2_joint", "left_hand_middle_0_joint",
    "left_hand_middle_1_joint", "left_hand_index_0_joint", "left_hand_index_1_joint",
    "right_hand_thumb_0_joint", "right_hand_thumb_1_joint", "right_hand_thumb_2_joint",
    "right_hand_index_0_joint", "right_hand_index_1_joint", "right_hand_middle_0_joint",
    "right_hand_middle_1_joint",
)
DEFAULT_TORQUE_LIMITS_NM = {
    i: (5.0 if i in (15, 16, 17, 18, 22, 23, 24, 25) else 1.5)
    for i in ARM_JOINT_INDICES
}


class G1ArmGravityFeedforward:
    """Compute zero-acceleration RNEA torque in rt/arm_sdk joint order."""
    def __init__(self, urdf_path: Path, torque_limits_nm: Mapping[int, float] = DEFAULT_TORQUE_LIMITS_NM):
        import numpy as np
        import pinocchio as pin

        self.urdf_path = Path(urdf_path).resolve()
        if not self.urdf_path.is_file():
            raise FileNotFoundError(f"G1 URDF not found: {self.urdf_path}")
        if set(torque_limits_nm) != set(ARM_JOINT_INDICES):
            raise ValueError("torque limits must specify exactly joints 15 through 28")
        self.torque_limits_nm = {i: float(torque_limits_nm[i]) for i in ARM_JOINT_INDICES}
        if any(not math.isfinite(v) or v <= 0.0 for v in self.torque_limits_nm.values()):
            raise ValueError("torque limits must be positive finite values")
        full = pin.RobotWrapper.BuildFromURDF(str(self.urdf_path), str(self.urdf_path.parent))
        self.robot = full.buildReducedRobot(list(G1_LOCKED_JOINT_NAMES), np.zeros(full.model.nq))
        observed = tuple(self.robot.model.names[1:])
        if self.robot.model.nq != 14 or self.robot.model.nv != 14 or observed != G1_ARM_JOINT_NAMES:
            raise RuntimeError(f"unexpected reduced G1 model: nq={self.robot.model.nq}, nv={self.robot.model.nv}, joints={observed}")
        self.urdf_sha256 = hashlib.sha256(self.urdf_path.read_bytes()).hexdigest()

    def configuration(self) -> Dict[str, object]:
        return {"method": "pinocchio_rnea_zero_acceleration", "urdf_path": str(self.urdf_path),
                "urdf_sha256": self.urdf_sha256, "sdk_joint_indices": list(ARM_JOINT_INDICES),
                "model_joint_names": list(G1_ARM_JOINT_NAMES), "torque_limits_nm": dict(self.torque_limits_nm)}

    def __call__(self, positions: Mapping[int, float], velocities: Mapping[int, float]) -> Dict[int, float]:
        import numpy as np
        import pinocchio as pin

        if set(positions) != set(ARM_JOINT_INDICES) or set(velocities) != set(ARM_JOINT_INDICES):
            raise ValueError("feedforward input must specify exactly joints 15 through 28")
        q = np.asarray([positions[i] for i in ARM_JOINT_INDICES], dtype=float)
        v = np.asarray([velocities[i] for i in ARM_JOINT_INDICES], dtype=float)
        if not np.all(np.isfinite(q)) or not np.all(np.isfinite(v)):
            raise ValueError("feedforward input contains a non-finite value")
        # Match xr_teleoperate's G1 path: validate desired velocity, but use
        # gravity-only RNEA (zero model velocity and acceleration). This avoids
        # injecting unreviewed Coriolis terms into the first compensated test.
        tau = np.asarray(pin.rnea(
            self.robot.model, self.robot.data, q, np.zeros(14), np.zeros(14)
        ), dtype=float)
        if tau.shape != (14,) or not np.all(np.isfinite(tau)):
            raise RuntimeError("RNEA returned invalid torque output")
        result = {}
        for offset, index in enumerate(ARM_JOINT_INDICES):
            value, limit = float(tau[offset]), self.torque_limits_nm[index]
            if abs(value) > limit:
                raise RuntimeError(f"RNEA torque {value:.3f} Nm exceeds {limit:.3f} Nm bound at joint {index}")
            result[index] = value
        return result
