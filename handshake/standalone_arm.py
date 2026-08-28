"""Reusable safety core for the standalone G1 arm handoff sequence."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from statistics import median
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence, Tuple


ARM_JOINT_INDICES = tuple(range(15, 29))
RIGHT_ELBOW_INDEX = 25
WRIST_JOINT_INDICES = (19, 20, 21, 26, 27, 28)
ARM_SDK_CONTROL_RATE_HZ = 250.0
ARM_SDK_MAX_COMMAND_VELOCITY_RAD_S = 0.50


def arm_sdk_gains(joint_index: int) -> Tuple[float, float]:
    """Return Unitree G1_29 motion-mode gains for an arm joint."""
    if joint_index not in ARM_JOINT_INDICES:
        raise ValueError(f"joint {joint_index} is not a G1_29 arm joint")
    if joint_index in WRIST_JOINT_INDICES:
        return 40.0, 1.5
    return 80.0, 3.0


@dataclass(frozen=True)
class BoundedArmPlan:
    amplitude_rad: float = 0.02
    duration_seconds: float = 1.0
    sample_rate_hz: float = ARM_SDK_CONTROL_RATE_HZ
    blend_seconds: float = 0.5

    def validate(self) -> None:
        if not 0.0 < self.amplitude_rad <= 0.05:
            raise ValueError("amplitude must be greater than zero and at most 0.05 rad")
        if not 0.5 <= self.duration_seconds <= 2.0:
            raise ValueError("duration must be between 0.5 and 2.0 seconds")
        if not 50.0 <= self.sample_rate_hz <= ARM_SDK_CONTROL_RATE_HZ:
            raise ValueError("sample rate must be between 50 and 250 Hz")
        if not 0.25 <= self.blend_seconds <= 2.0:
            raise ValueError("blend duration must be between 0.25 and 2.0 seconds")

    def samples(self) -> List[Dict[str, float]]:
        self.validate()
        intervals = max(2, int(round(self.duration_seconds * self.sample_rate_hz)))
        return [
            {
                "time_seconds": self.duration_seconds * index / intervals,
                "relative_position_rad": self.amplitude_rad
                * 0.5
                * (1.0 - math.cos(2.0 * math.pi * index / intervals)),
            }
            for index in range(intervals + 1)
        ]


def arm_vectors(state: Any) -> Tuple[List[float], List[float], List[float]]:
    motors = state.motor_state
    positions = [float(motor.q) for motor in motors]
    velocities = [float(motor.dq) for motor in motors]
    torques = [float(motor.tau_est) for motor in motors]
    if len(positions) <= max(ARM_JOINT_INDICES):
        raise RuntimeError("low-state message does not contain every arm joint")
    return positions, velocities, torques


def pose_failures(
    state: Any,
    expected: Optional[Mapping[int, float]],
    pose_tolerance_rad: float,
    velocity_limit_rad_s: float,
) -> List[Dict[str, Any]]:
    positions, velocities, _ = arm_vectors(state)
    failures = []
    for index in ARM_JOINT_INDICES:
        pose_error = None if expected is None else abs(positions[index] - expected[index])
        velocity = abs(velocities[index])
        if velocity > velocity_limit_rad_s or (
            pose_error is not None and pose_error > pose_tolerance_rad
        ):
            failures.append(
                {
                    "joint_index": index,
                    "pose_error_rad": pose_error,
                    "absolute_velocity_rad_s": velocity,
                }
            )
    return failures


def wait_for_settled_state(
    state_supplier: Callable[[], Tuple[Any, Optional[int]]],
    expected: Optional[Mapping[int, float]],
    event_callback: Callable[[str, Dict[str, Any]], None],
    *,
    pose_tolerance_rad: float = 0.01,
    velocity_limit_rad_s: float = 0.10,
    required_duration_seconds: float = 0.50,
    timeout_seconds: float = 8.0,
    telemetry_timeout_seconds: float = 0.10,
    clock: Callable[[], float] = time.monotonic,
    monotonic_ns: Callable[[], int] = time.monotonic_ns,
    sleep: Callable[[float], None] = time.sleep,
) -> Any:
    deadline = clock() + timeout_seconds
    settled_since = None
    worst_failures: List[Dict[str, Any]] = []
    event_callback(
        "settling_started",
        {
            "timeout_seconds": timeout_seconds,
            "required_duration_seconds": required_duration_seconds,
            "pose_envelope_enabled": expected is not None,
        },
    )
    while clock() < deadline:
        state, received_ns = state_supplier()
        if state is None or received_ns is None:
            raise RuntimeError("unitree telemetry unavailable")
        age_seconds = (monotonic_ns() - received_ns) / 1e9
        if age_seconds > telemetry_timeout_seconds:
            raise RuntimeError(f"unitree telemetry stale ({age_seconds:.3f}s)")
        failures = pose_failures(
            state, expected, pose_tolerance_rad, velocity_limit_rad_s
        )
        now = clock()
        if failures:
            settled_since = None
            worst_failures = failures
        else:
            if settled_since is None:
                settled_since = now
            if now - settled_since >= required_duration_seconds:
                event_callback("settled", {"duration_seconds": now - settled_since})
                return state
        sleep(0.02)
    worst_failures.sort(
        key=lambda item: max(
            item["pose_error_rad"] or 0.0,
            item["absolute_velocity_rad_s"],
        ),
        reverse=True,
    )
    raise RuntimeError(f"arm settling timeout; failing joints: {worst_failures}")


def capture_pose_centers(
    state_supplier: Callable[[], Tuple[Any, Optional[int]]],
    duration_seconds: float,
    *,
    telemetry_timeout_seconds: float = 0.10,
    clock: Callable[[], float] = time.monotonic,
    monotonic_ns: Callable[[], int] = time.monotonic_ns,
    sleep: Callable[[float], None] = time.sleep,
) -> Dict[int, float]:
    deadline = clock() + duration_seconds
    samples: Dict[int, List[float]] = {index: [] for index in ARM_JOINT_INDICES}
    while clock() < deadline:
        state, received_ns = state_supplier()
        if state is None or received_ns is None:
            raise RuntimeError("unitree telemetry unavailable during pose capture")
        age_seconds = (monotonic_ns() - received_ns) / 1e9
        if age_seconds > telemetry_timeout_seconds:
            raise RuntimeError(f"unitree telemetry stale during capture ({age_seconds:.3f}s)")
        positions, _, _ = arm_vectors(state)
        for index in ARM_JOINT_INDICES:
            samples[index].append(positions[index])
        sleep(0.02)
    if not all(samples.values()):
        raise RuntimeError("pose capture produced no samples")
    return {index: median(samples[index]) for index in ARM_JOINT_INDICES}


def require_arm_displacement(
    state: Any,
    baseline: Mapping[int, float],
    minimum_displacement_rad: float,
) -> Dict[str, float]:
    """Require measured evidence that a requested arm action actually occurred."""
    positions, _, _ = arm_vectors(state)
    displacement = {
        index: abs(positions[index] - baseline[index]) for index in ARM_JOINT_INDICES
    }
    joint_index = max(displacement, key=displacement.get)
    maximum = displacement[joint_index]
    if maximum < minimum_displacement_rad:
        raise RuntimeError(
            "arm action not verified by telemetry; maximum displacement "
            f"{maximum:.6f} rad is below {minimum_displacement_rad:.6f} rad"
        )
    return {"joint_index": joint_index, "maximum_displacement_rad": maximum}


class BoundedArmExecutor:
    """Compute or publish one bumpless out-and-back arm-SDK trajectory."""

    def __init__(
        self,
        plan: BoundedArmPlan,
        state_supplier: Callable[[], Tuple[Any, Optional[int]]],
        command_sink: Callable[[Sequence[float], Sequence[float], float], None],
        event_callback: Callable[[str, Dict[str, Any]], None],
        *,
        publish_commands: bool,
        telemetry_timeout_seconds: float = 0.10,
        max_velocity_rad_s: float = 1.0,
        max_torque_nm: float = 10.0,
        max_command_velocity_rad_s: float = ARM_SDK_MAX_COMMAND_VELOCITY_RAD_S,
        clock: Callable[[], float] = time.monotonic,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        plan.validate()
        self.plan = plan
        self.state_supplier = state_supplier
        self.command_sink = command_sink
        self.event_callback = event_callback
        self.publish_commands = publish_commands
        self.telemetry_timeout_seconds = telemetry_timeout_seconds
        self.max_velocity_rad_s = max_velocity_rad_s
        self.max_torque_nm = max_torque_nm
        if not 0.0 < max_command_velocity_rad_s <= 1.0:
            raise ValueError("command velocity limit must be greater than zero and at most 1 rad/s")
        self.max_command_velocity_rad_s = max_command_velocity_rad_s
        self.clock = clock
        self.monotonic_ns = monotonic_ns
        self.sleep = sleep
        self.sequence = 0
        self.authority_weight = 0.0
        self.last_positions: Optional[List[float]] = None
        self.last_velocities: Optional[List[float]] = None
        self.maximum_schedule_lag_seconds = 0.0

    def _clip_targets(
        self,
        targets: Sequence[float],
        measured_positions: Sequence[float],
    ) -> List[float]:
        """Apply Unitree's measured-state velocity clipping to every arm target."""
        maximum_step = self.max_command_velocity_rad_s / self.plan.sample_rate_hz
        maximum_error = max(
            abs(targets[index] - measured_positions[index])
            for index in ARM_JOINT_INDICES
        )
        scale = max(1.0, maximum_error / maximum_step)
        clipped = list(measured_positions)
        for index in ARM_JOINT_INDICES:
            clipped[index] = measured_positions[index] + (
                targets[index] - measured_positions[index]
            ) / scale
        return clipped

    def _fresh_vectors(self) -> Tuple[List[float], List[float], List[float]]:
        state, received_ns = self.state_supplier()
        if state is None or received_ns is None:
            raise RuntimeError("unitree telemetry unavailable")
        age_seconds = (self.monotonic_ns() - received_ns) / 1e9
        if age_seconds > self.telemetry_timeout_seconds:
            raise RuntimeError(f"unitree telemetry stale ({age_seconds:.3f}s)")
        positions, velocities, torques = arm_vectors(state)
        violations = [
            {
                "joint_index": index,
                "velocity_rad_s": velocities[index],
                "torque_nm": torques[index],
            }
            for index in ARM_JOINT_INDICES
            if abs(velocities[index]) > self.max_velocity_rad_s
            or abs(torques[index]) > self.max_torque_nm
        ]
        if violations:
            raise RuntimeError(f"arm state limit exceeded: {violations}")
        return positions, velocities, torques

    def _command(
        self,
        positions: Sequence[float],
        velocities: Sequence[float],
        weight: float,
        phase: str,
    ) -> None:
        weight = max(0.0, min(1.0, weight))
        if self.publish_commands:
            self.command_sink(positions, velocities, weight)
        self.sequence += 1
        self.authority_weight = weight
        self.last_positions = list(positions)
        self.last_velocities = list(velocities)
        self.event_callback(
            "arm_sdk_command",
            {
                "sequence": self.sequence,
                "phase": phase,
                "authority_weight": weight,
                "positions": {str(i): positions[i] for i in ARM_JOINT_INDICES},
                "velocities": {str(i): velocities[i] for i in ARM_JOINT_INDICES},
                "max_command_velocity_rad_s": self.max_command_velocity_rad_s,
                "published": self.publish_commands,
            },
        )

    def _controlled_release(self) -> None:
        if self.authority_weight <= 0.0 or self.last_positions is None:
            return
        initial_weight = self.authority_weight
        started = self.clock()
        period = 1.0 / self.plan.sample_rate_hz
        next_tick = started
        self.event_callback("authority_release_started", {"initial_weight": initial_weight})
        while True:
            fraction = min(1.0, (self.clock() - started) / self.plan.blend_seconds)
            try:
                positions, _, _ = self._fresh_vectors()
            except Exception:
                positions = list(self.last_positions)
            targets = self._clip_targets(self.last_positions, positions)
            velocities = [0.0] * len(targets)
            self._command(
                targets,
                velocities,
                initial_weight * (1.0 - fraction),
                "abort_release",
            )
            if fraction >= 1.0:
                break
            next_tick += period
            now = self.clock()
            self.maximum_schedule_lag_seconds = max(
                self.maximum_schedule_lag_seconds, max(0.0, now - next_tick)
            )
            self.sleep(max(0.0, next_tick - now))
        self.event_callback("authority_release_finished", {"final_weight": 0.0})

    def run(self, settled_state: Any) -> Tuple[str, str]:
        base, _, _ = arm_vectors(settled_state)
        started = self.clock()
        period = 1.0 / self.plan.sample_rate_hz
        next_tick = started
        outcome, reason = "completed", "trajectory_complete"
        self.event_callback("movement_started", {"published": self.publish_commands})
        try:
            while True:
                elapsed = self.clock() - started
                positions, _, _ = self._fresh_vectors()
                if elapsed < self.plan.blend_seconds:
                    phase = "blend_in"
                    weight = elapsed / self.plan.blend_seconds
                    targets = list(positions)
                elif elapsed < self.plan.blend_seconds + self.plan.duration_seconds:
                    phase = "trajectory"
                    movement_time = elapsed - self.plan.blend_seconds
                    fraction = movement_time / self.plan.duration_seconds
                    offset = self.plan.amplitude_rad * 0.5 * (
                        1.0 - math.cos(2.0 * math.pi * fraction)
                    )
                    targets = list(base)
                    targets[RIGHT_ELBOW_INDEX] += offset
                    weight = 1.0
                elif elapsed < 2.0 * self.plan.blend_seconds + self.plan.duration_seconds:
                    phase = "release"
                    release_time = elapsed - self.plan.blend_seconds - self.plan.duration_seconds
                    weight = 1.0 - release_time / self.plan.blend_seconds
                    targets = list(base)
                else:
                    targets = self._clip_targets(base, positions)
                    self._command(targets, [0.0] * len(targets), 0.0, "release")
                    break
                targets = self._clip_targets(targets, positions)
                target_velocities = [0.0] * len(targets)
                self._command(targets, target_velocities, weight, phase)
                next_tick += period
                now = self.clock()
                self.maximum_schedule_lag_seconds = max(
                    self.maximum_schedule_lag_seconds, max(0.0, now - next_tick)
                )
                self.sleep(max(0.0, next_tick - now))
        except BaseException as exc:
            outcome, reason = "aborted", f"{type(exc).__name__}: {exc}"
            self._controlled_release()
        self.event_callback(
            "movement_finished",
            {
                "outcome": outcome,
                "reason": reason,
                "command_count": self.sequence,
                "target_rate_hz": self.plan.sample_rate_hz,
                "maximum_schedule_lag_seconds": self.maximum_schedule_lag_seconds,
            },
        )
        return outcome, reason
