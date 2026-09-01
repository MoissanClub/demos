"""Continuous rt/arm_sdk ownership for a bounded G1 arm raise and return."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from statistics import median
from typing import Any, Callable, Dict, Mapping, Optional, Sequence, Tuple

from handshake.standalone_arm import ARM_JOINT_INDICES, arm_vectors


CommandSink = Callable[[Sequence[float], Sequence[float], Sequence[float], float], None]
Feedforward = Callable[[Mapping[int, float], Mapping[int, float]], Mapping[int, float]]
StateSupplier = Callable[[], Tuple[Any, Optional[int]]]
SportSupplier = Callable[[], Tuple[Any, Optional[int]]]
EventCallback = Callable[[str, Dict[str, Any]], None]


@dataclass(frozen=True)
class ContinuousArmConfig:
    sample_rate_hz: float = 250.0
    acquire_seconds: float = 1.0
    raise_seconds: float = 2.0
    return_seconds: float = 2.0
    release_seconds: float = 1.0
    internal_return_timeout_seconds: float = 3.0
    settle_seconds: float = 0.5
    settle_timeout_seconds: float = 5.0
    initial_telemetry_timeout_seconds: float = 3.0
    telemetry_timeout_seconds: float = 0.1
    sport_telemetry_timeout_seconds: float = 0.25
    pose_tolerance_rad: float = 0.03
    settle_velocity_rad_s: float = 0.1
    max_measured_velocity_rad_s: float = 1.0
    max_release_velocity_rad_s: float = 1.0
    max_measured_torque_nm: float = 10.0
    max_tracking_error_rad: float = 0.03
    max_command_lead_rad: Optional[float] = None
    scale_feedforward_by_authority: bool = True
    step_to_full_authority: bool = False
    max_offset_rad: float = 0.35
    required_fsm_id: int = 501
    required_fsm_mode: int = 0

    def validate(self) -> None:
        if not 50.0 <= self.sample_rate_hz <= 250.0:
            raise ValueError("sample rate must be between 50 and 250 Hz")
        for name in ("acquire_seconds", "release_seconds"):
            if not 0.5 <= getattr(self, name) <= 10.0:
                raise ValueError(f"{name} must be between 0.5 and 10 seconds")
        for name in ("raise_seconds", "return_seconds"):
            if not 0.5 <= getattr(self, name) <= 30.0:
                raise ValueError(f"{name} must be between 0.5 and 30 seconds")
        if not 0.1 <= self.settle_seconds <= 2.0:
            raise ValueError("settle_seconds must be between 0.1 and 2 seconds")
        if self.settle_timeout_seconds < self.settle_seconds:
            raise ValueError("settle timeout must be at least the settle duration")
        if not 0.5 <= self.internal_return_timeout_seconds <= 10.0:
            raise ValueError("internal return timeout must be between 0.5 and 10 seconds")
        if not 0.5 <= self.initial_telemetry_timeout_seconds <= 10.0:
            raise ValueError("initial telemetry timeout must be between 0.5 and 10 seconds")
        if not 0.1 <= self.sport_telemetry_timeout_seconds <= 1.0:
            raise ValueError("sport telemetry timeout must be between 0.1 and 1 second")
        if not 0.005 <= self.pose_tolerance_rad <= 0.05:
            raise ValueError("pose tolerance must be between 0.005 and 0.05 rad")
        if not 0.01 <= self.settle_velocity_rad_s <= 0.2:
            raise ValueError("settle velocity must be between 0.01 and 0.2 rad/s")
        if not 0.01 <= self.max_offset_rad <= 0.9:
            raise ValueError("maximum offset must be between 0.01 and 0.9 rad")
        if not self.max_measured_velocity_rad_s <= self.max_release_velocity_rad_s <= 2.0:
            raise ValueError("release velocity limit must be at least the motion limit and at most 2 rad/s")
        if not 0.005 <= self.max_tracking_error_rad <= 0.05:
            raise ValueError("maximum tracking error must be between 0.005 and 0.05 rad")
        if self.max_command_lead_rad is not None and not 0.005 <= self.max_command_lead_rad <= 0.05:
            raise ValueError("maximum command lead must be between 0.005 and 0.05 rad")


def _smoothstep5(fraction: float) -> Tuple[float, float]:
    """Return quintic position fraction and derivative with respect to fraction."""
    x = max(0.0, min(1.0, fraction))
    return 10 * x**3 - 15 * x**4 + 6 * x**5, 30 * x**2 - 60 * x**3 + 30 * x**4


class ContinuousArmController:
    """Own the arm continuously between :meth:`raise_arm` and :meth:`release_arm`.

    The caller must keep this object alive while the arm is raised. No Unitree
    high-level action client is used or accepted by this class.
    """

    def __init__(
        self,
        state_supplier: StateSupplier,
        sport_supplier: SportSupplier,
        command_sink: Optional[CommandSink],
        event_callback: EventCallback,
        feedforward: Feedforward,
        *,
        publish_commands: bool,
        config: ContinuousArmConfig = ContinuousArmConfig(),
        clock: Callable[[], float] = time.monotonic,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        config.validate()
        self.state_supplier = state_supplier
        self.sport_supplier = sport_supplier
        self.command_sink = command_sink
        self.event = event_callback
        self.feedforward = feedforward
        self.publish_commands = publish_commands
        self.config = config
        self.clock = clock
        self.monotonic_ns = monotonic_ns
        self.sleep = sleep
        self.initial_pose: Optional[Dict[int, float]] = None
        self.raised_pose: Optional[Dict[int, float]] = None
        self.last_target: Optional[Dict[int, float]] = None
        self.authority_weight = 0.0
        self.phase = "idle"
        self.sequence = 0
        self.last_controller_state: Optional[Tuple[int, int]] = None

    def observe_initial_pose(self) -> Dict[int, float]:
        """Require a sustained stationary Regular-mode pose without publishing."""
        if self.phase != "idle" or self.command_sink is not None:
            raise RuntimeError("initial observation requires idle state before publisher construction")
        self.phase = "initial_observe"
        self.event("phase_started", {"phase": self.phase})
        readiness_deadline = self.clock() + self.config.initial_telemetry_timeout_seconds
        while True:
            state, state_ns = self.state_supplier()
            sport, sport_ns = self.sport_supplier()
            if state is not None and state_ns is not None and sport is not None and sport_ns is not None:
                break
            if self.clock() >= readiness_deadline:
                missing = []
                if state is None or state_ns is None:
                    missing.append("low-state")
                if sport is None or sport_ns is None:
                    missing.append("sport-mode")
                self.phase = "idle"
                raise RuntimeError(
                    "initial telemetry timeout waiting for " + " and ".join(missing)
                )
            self.sleep(0.02)
        self.event("initial_telemetry_ready", {})
        deadline = self.clock() + self.config.settle_seconds
        samples = {index: [] for index in ARM_JOINT_INDICES}
        while self.clock() < deadline:
            positions, velocities, _ = self._check_state()
            if any(abs(velocities[i]) > self.config.settle_velocity_rad_s for i in ARM_JOINT_INDICES):
                self.phase = "idle"
                raise RuntimeError("initial arm pose is not stationary")
            for index in ARM_JOINT_INDICES:
                samples[index].append(float(positions[index]))
            self.sleep(0.02)
        pose = {index: median(samples[index]) for index in ARM_JOINT_INDICES}
        if any(
            max(samples[i]) - min(samples[i]) > self.config.pose_tolerance_rad
            for i in ARM_JOINT_INDICES
        ):
            self.phase = "idle"
            raise RuntimeError("initial arm pose varied outside the capture envelope")
        self.initial_pose = pose
        self.phase = "prepared"
        self.event("phase_finished", {"phase": "initial_observe", "initial_pose": pose})
        return dict(pose)

    def attach_command_sink(self, command_sink: CommandSink) -> None:
        if self.phase != "prepared" or self.initial_pose is None:
            raise RuntimeError("command sink can only be attached after initial observation")
        if self.command_sink is not None:
            raise RuntimeError("command sink is already attached")
        self.command_sink = command_sink

    def _check_state(self) -> Tuple[Sequence[float], Sequence[float], Sequence[float]]:
        state, received_ns = self.state_supplier()
        if state is None or received_ns is None:
            raise RuntimeError("low-state telemetry unavailable")
        age = (self.monotonic_ns() - received_ns) / 1e9
        if age > self.config.telemetry_timeout_seconds:
            raise RuntimeError(f"low-state telemetry stale ({age:.3f}s)")
        sport, sport_ns = self.sport_supplier()
        if sport is None or sport_ns is None:
            raise RuntimeError("sport-mode telemetry unavailable")
        sport_age = (self.monotonic_ns() - sport_ns) / 1e9
        if sport_age > self.config.sport_telemetry_timeout_seconds:
            raise RuntimeError(f"sport-mode telemetry stale ({sport_age:.3f}s)")
        observed = (int(sport.fsm_id), int(sport.fsm_mode))
        allowed_modes = (
            {self.config.required_fsm_mode}
            if self.phase in {"idle", "initial_observe", "prepared"}
            else {self.config.required_fsm_mode, 1}
        )
        if observed[0] != self.config.required_fsm_id or observed[1] not in allowed_modes:
            raise RuntimeError(
                "controller state changed: expected FSM "
                f"{self.config.required_fsm_id} with mode in {sorted(allowed_modes)}, "
                f"observed {observed}"
            )
        if self.last_controller_state is not None and observed != self.last_controller_state:
            self.event(
                "controller_state_transition",
                {"previous": self.last_controller_state, "observed": observed, "phase": self.phase},
            )
        self.last_controller_state = observed
        positions, velocities, torques = arm_vectors(state)
        velocity_limit = (
            self.config.max_release_velocity_rad_s
            if self.phase in {"authority_release", "internal_control_return"}
            else self.config.max_measured_velocity_rad_s
        )
        violations = [
            index
            for index in ARM_JOINT_INDICES
            if abs(velocities[index]) > velocity_limit
            or abs(torques[index]) > self.config.max_measured_torque_nm
        ]
        if violations:
            raise RuntimeError(f"measured arm limit exceeded at joints {violations}")
        # During authority release the native controller increasingly owns the
        # arm and may legitimately move toward its own target. Continue
        # enforcing velocity/torque/FSM limits, but do not require tracking of
        # the outgoing arm-SDK target.
        tracking_phases = {
            "authority_acquire", "raise", "raised_settle", "raised_hold",
            "return", "return_settle",
        }
        if (
            self.last_target is not None
            and self.authority_weight > 0.0
            and self.phase in tracking_phases
        ):
            tracking = [
                index for index in ARM_JOINT_INDICES
                if abs(positions[index] - self.last_target[index]) > self.config.max_tracking_error_rad
            ]
            if tracking:
                raise RuntimeError(f"arm tracking error exceeded at joints {tracking}")
        return positions, velocities, torques

    def _send(self, target: Mapping[int, float], velocity: Mapping[int, float], weight: float) -> None:
        measured, _, _ = self._check_state()
        if self.config.max_command_lead_rad is not None and self.phase in {
            "raise", "raised_settle", "raised_hold", "return", "return_settle",
        }:
            maximum_error = max(abs(float(target[i]) - measured[i]) for i in ARM_JOINT_INDICES)
            scale = max(1.0, maximum_error / self.config.max_command_lead_rad)
            target = {
                i: measured[i] + (float(target[i]) - measured[i]) / scale
                for i in ARM_JOINT_INDICES
            }
            velocity = {i: float(velocity[i]) / scale for i in ARM_JOINT_INDICES}
        positions = list(measured)
        velocities = [0.0] * len(positions)
        torques = [0.0] * len(positions)
        for index in ARM_JOINT_INDICES:
            positions[index] = target[index]
            velocities[index] = velocity[index]
        weight = max(0.0, min(1.0, weight))
        raw_torques = self.feedforward(target, velocity)
        if set(raw_torques) != set(ARM_JOINT_INDICES):
            raise RuntimeError("feedforward returned an incomplete joint mapping")
        for index in ARM_JOINT_INDICES:
            value = float(raw_torques[index])
            if not math.isfinite(value):
                raise RuntimeError(f"feedforward returned non-finite torque at joint {index}")
            torques[index] = value * weight if self.config.scale_feedforward_by_authority else value
        if self.publish_commands:
            if self.command_sink is None:
                raise RuntimeError("arm-SDK command sink is not attached")
            self.command_sink(positions, velocities, torques, weight)
        self.sequence += 1
        self.authority_weight = weight
        self.last_target = dict(target)
        self.event("arm_sdk_command", {
            "sequence": self.sequence, "phase": self.phase,
            "authority_weight": weight, "published": self.publish_commands,
            "positions": {str(i): target[i] for i in ARM_JOINT_INDICES},
            "velocities": {str(i): velocity[i] for i in ARM_JOINT_INDICES},
            "feedforward_torques_nm": {str(i): torques[i] for i in ARM_JOINT_INDICES},
        })

    def _run_timed(self, phase: str, duration: float, sample: Callable[[float], Tuple[Mapping[int, float], Mapping[int, float], float]]) -> None:
        self.phase = phase
        self.event("phase_started", {"phase": phase, "duration_seconds": duration})
        started = self.clock()
        period = 1.0 / self.config.sample_rate_hz
        tick = 0
        while True:
            elapsed = self.clock() - started
            fraction = min(1.0, elapsed / duration)
            target, velocity, weight = sample(fraction)
            self._send(target, velocity, weight)
            if fraction >= 1.0:
                break
            tick += 1
            deadline = started + tick * period
            self.sleep(max(0.0, deadline - self.clock()))
        self.event("phase_finished", {"phase": phase})

    def _hold_until_settled(self, phase: str, target: Mapping[int, float]) -> None:
        self.phase = phase
        self.event("phase_started", {"phase": phase})
        started = self.clock()
        settled_since: Optional[float] = None
        zeros = {index: 0.0 for index in ARM_JOINT_INDICES}
        period = 1.0 / self.config.sample_rate_hz
        tick = 0
        while self.clock() - started <= self.config.settle_timeout_seconds:
            positions, velocities, _ = self._check_state()
            self._send(target, zeros, 1.0)
            valid = all(
                abs(positions[i] - target[i]) <= self.config.pose_tolerance_rad
                and abs(velocities[i]) <= self.config.settle_velocity_rad_s
                for i in ARM_JOINT_INDICES
            )
            now = self.clock()
            settled_since = now if valid and settled_since is None else settled_since
            if not valid:
                settled_since = None
            if settled_since is not None and now - settled_since >= self.config.settle_seconds:
                self.event("phase_finished", {"phase": phase, "settled_seconds": now - settled_since})
                return
            tick += 1
            deadline = started + tick * period
            self.sleep(max(0.0, deadline - self.clock()))
        raise RuntimeError(f"{phase} failed to settle")

    def raise_arm(self, offsets_rad: Mapping[int, float]) -> None:
        if self.phase != "prepared" or self.initial_pose is None:
            raise RuntimeError(f"raise_arm requires prepared state, observed {self.phase}")
        if set(offsets_rad) != set(ARM_JOINT_INDICES):
            raise ValueError("raise offsets must specify exactly joints 15 through 28")
        if max(abs(float(offsets_rad[i])) for i in ARM_JOINT_INDICES) > self.config.max_offset_rad:
            raise ValueError("raise offset exceeds configured maximum")
        peak_factor = 1.875
        peak_velocity = max(abs(float(offsets_rad[i])) for i in ARM_JOINT_INDICES) * peak_factor / min(
            self.config.raise_seconds, self.config.return_seconds
        )
        if peak_velocity > 0.5:
            raise ValueError(f"planned peak command velocity {peak_velocity:.3f} rad/s exceeds 0.5 rad/s")
        positions, velocities, _ = self._check_state()
        if any(abs(velocities[i]) > self.config.settle_velocity_rad_s for i in ARM_JOINT_INDICES):
            raise RuntimeError("initial arm pose is not stationary")
        self.raised_pose = {i: self.initial_pose[i] + float(offsets_rad[i]) for i in ARM_JOINT_INDICES}
        zeros = {i: 0.0 for i in ARM_JOINT_INDICES}
        fixed = dict(self.initial_pose)
        self._run_timed(
            "authority_acquire", self.config.acquire_seconds,
            lambda f: (fixed, zeros, 1.0 if self.config.step_to_full_authority else f),
        )

        def raising(fraction: float) -> Tuple[Mapping[int, float], Mapping[int, float], float]:
            blend, derivative = _smoothstep5(fraction)
            target = {i: fixed[i] + offsets_rad[i] * blend for i in ARM_JOINT_INDICES}
            velocity = {i: offsets_rad[i] * derivative / self.config.raise_seconds for i in ARM_JOINT_INDICES}
            return target, velocity, 1.0

        self._run_timed("raise", self.config.raise_seconds, raising)
        self._hold_until_settled("raised_settle", self.raised_pose)
        self.phase = "raised_hold"
        self.event("arm_raised", {"initial_pose": self.initial_pose, "raised_pose": self.raised_pose})

    def hold_once(self) -> None:
        if self.phase != "raised_hold" or self.raised_pose is None:
            raise RuntimeError("hold_once requires a raised arm")
        self._send(self.raised_pose, {i: 0.0 for i in ARM_JOINT_INDICES}, 1.0)

    def abort_release(self) -> None:
        """Release arm-SDK authority from the measured pose after a motion fault."""
        if self.command_sink is None or self.authority_weight <= 0.0:
            return
        state, received_ns = self.state_supplier()
        if state is None or received_ns is None:
            raise RuntimeError("cannot release after abort without low-state telemetry")
        measured, _, _ = arm_vectors(state)
        target = {i: float(measured[i]) for i in ARM_JOINT_INDICES}
        zeros = {i: 0.0 for i in ARM_JOINT_INDICES}
        initial_weight = self.authority_weight
        self.phase = "authority_release"
        self._run_timed(
            "authority_release", self.config.release_seconds,
            lambda f: (target, zeros, initial_weight * (1.0 - f)),
        )
        self.phase = "internal_control_return"
        deadline = self.clock() + self.config.internal_return_timeout_seconds
        while self.clock() < deadline:
            sport, sport_ns = self.sport_supplier()
            if sport is not None and sport_ns is not None and int(sport.fsm_id) == self.config.required_fsm_id and int(sport.fsm_mode) == self.config.required_fsm_mode:
                self.authority_weight = 0.0
                self.phase = "released"
                self.event("abort_release_finished", {"observed": (int(sport.fsm_id), int(sport.fsm_mode))})
                return
            self.sleep(1.0 / self.config.sample_rate_hz)
        raise RuntimeError("abort release did not return the native controller to Regular mode")

    def release_arm(self) -> None:
        if self.phase != "raised_hold" or self.initial_pose is None or self.raised_pose is None:
            raise RuntimeError("release_arm requires a successfully raised arm")
        initial, raised = dict(self.initial_pose), dict(self.raised_pose)

        def returning(fraction: float) -> Tuple[Mapping[int, float], Mapping[int, float], float]:
            blend, derivative = _smoothstep5(fraction)
            target = {i: raised[i] + (initial[i] - raised[i]) * blend for i in ARM_JOINT_INDICES}
            velocity = {i: (initial[i] - raised[i]) * derivative / self.config.return_seconds for i in ARM_JOINT_INDICES}
            return target, velocity, 1.0

        self._run_timed("return", self.config.return_seconds, returning)
        self._hold_until_settled("return_settle", initial)
        zeros = {i: 0.0 for i in ARM_JOINT_INDICES}
        self._run_timed("authority_release", self.config.release_seconds, lambda f: (initial, zeros, 1.0 - f))
        self.phase = "internal_control_return"
        self.event("phase_started", {"phase": self.phase})
        deadline = self.clock() + self.config.internal_return_timeout_seconds
        period = 1.0 / self.config.sample_rate_hz
        polling_started = self.clock()
        tick = 0
        while True:
            self._send(initial, zeros, 0.0)
            sport, sport_ns = self.sport_supplier()
            if sport is not None and sport_ns is not None:
                age = (self.monotonic_ns() - sport_ns) / 1e9
                if (
                    age <= self.config.sport_telemetry_timeout_seconds
                    and int(sport.fsm_id) == self.config.required_fsm_id
                    and int(sport.fsm_mode) == self.config.required_fsm_mode
                ):
                    self.event(
                        "phase_finished",
                        {"phase": self.phase, "observed": (int(sport.fsm_id), int(sport.fsm_mode))},
                    )
                    break
            if self.clock() >= deadline:
                raise RuntimeError(
                    "authority weight reached zero but internal controller did not return "
                    f"to ({self.config.required_fsm_id}, {self.config.required_fsm_mode})"
                )
            tick += 1
            polling_deadline = polling_started + tick * period
            self.sleep(max(0.0, polling_deadline - self.clock()))
        self.phase = "released"
        self.event(
            "arm_released",
            {
                "final_weight": self.authority_weight,
                "internal_controller": (
                    self.config.required_fsm_id,
                    self.config.required_fsm_mode,
                ),
            },
        )
