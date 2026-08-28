#!/usr/bin/env python3
"""Stage the G1 high-level raise -> bounded arm SDK -> high-level return sequence.

Vision, tactile sensing, and BrainCo hand commands are intentionally absent.
Physical modes require a gantry and a dedicated emergency-stop operator.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional, Sequence, Tuple

from handshake.recording import TelemetryRecorder
from handshake.standalone_arm import (
    ARM_SDK_CONTROL_RATE_HZ,
    ARM_JOINT_INDICES,
    BoundedArmExecutor,
    BoundedArmPlan,
    arm_sdk_gains,
    capture_pose_centers,
    require_arm_displacement,
    wait_for_settled_state,
)
from handshake.unitree_cleanup import close_rpc_client
from telemetry_probe import unitree_lowstate_record


RPC_ERROR_DESCRIPTIONS = {
    3102: "RPC client send failure",
    3103: "RPC API not registered",
    3104: "RPC API timeout",
    3105: "RPC API version mismatch",
    3106: "RPC API data error",
    3107: "RPC lease invalid",
    7400: "rt/armsdk is occupied",
    7401: "arm is holding; expected release action or the previous action",
    7402: "invalid arm action id",
    7404: "invalid robot FSM id for arm action",
}


def require_rpc_success(action: str, code: Any) -> Any:
    if code != 0:
        description = RPC_ERROR_DESCRIPTIONS.get(code, "unknown Unitree RPC error")
        raise RuntimeError(f"{action} failed with code {code}: {description}")
    return code


def require_rpc_success_or_defer_timeout(
    action: str,
    code: Any,
    allow_timeout_verification: bool,
    event: Any,
) -> Any:
    if code == 3104 and allow_timeout_verification:
        event(
            "high_level_rpc_timeout_pending_telemetry_verification",
            {"action": action, "return_value": code},
        )
        return code
    return require_rpc_success(action, code)


def validate_arm_action_fsm(fsm_id: Any, fsm_mode: Any) -> None:
    if fsm_id not in {500, 501, 801}:
        raise RuntimeError(
            f"arm actions are unsupported in FSM {fsm_id}; expected 500, 501, or 801"
        )
    if fsm_id == 801 and fsm_mode not in {0, 3}:
        raise RuntimeError(
            f"arm actions in FSM 801 require FSM mode 0 or 3, observed {fsm_mode}"
        )


def sport_mode_state_type() -> Any:
    """Define the humanoid sport-state IDL omitted by older Python SDK builds."""
    from handshake.sport_mode_state import SportModeState_

    return SportModeState_


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--offline-plan-only", action="store_true")
    modes.add_argument("--probe-preflight", action="store_true")
    modes.add_argument("--capture-post-action-pose", action="store_true")
    modes.add_argument("--dry-run-arm-sdk", action="store_true")
    modes.add_argument("--execute-arm-sdk", action="store_true")
    parser.add_argument("--network-interface", default="eth0")
    parser.add_argument("--raise-action", default="shake hand")
    parser.add_argument("--release-action", default="release arm")
    parser.add_argument(
        "--high-level-backend",
        choices=("arm-action", "legacy-sport"),
        default="arm-action",
        help=(
            "high-level raise/return backend; arm-action is the default because "
            "its shake-hand action has been physically verified on this robot"
        ),
    )
    parser.add_argument("--legacy-raise-task-id", type=int, default=2)
    parser.add_argument("--legacy-release-task-id", type=int, default=3)
    parser.add_argument("--post-action-pose-rad", type=float, nargs=14)
    parser.add_argument("--safe-return-pose-rad", type=float, nargs=14)
    parser.add_argument("--pose-tolerance-rad", type=float, default=0.01)
    parser.add_argument("--settle-velocity-rad-s", type=float, default=0.10)
    parser.add_argument("--settle-duration", type=float, default=0.50)
    parser.add_argument("--settle-timeout", type=float, default=8.0)
    parser.add_argument("--capture-duration", type=float, default=1.0)
    parser.add_argument("--minimum-action-displacement-rad", type=float, default=0.10)
    parser.add_argument(
        "--allow-action-timeout-with-telemetry-verification",
        action="store_true",
        help=(
            "Accept RPC code 3104 only after measured raise displacement and "
            "measured return to the pre-action pose"
        ),
    )
    parser.add_argument("--amplitude-rad", type=float, default=0.02)
    parser.add_argument("--movement-duration", type=float, default=1.0)
    parser.add_argument("--blend-duration", type=float, default=0.5)
    parser.add_argument(
        "--sample-rate-hz", type=float, default=ARM_SDK_CONTROL_RATE_HZ
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--confirm-gantry-attached", action="store_true")
    parser.add_argument("--confirm-estop-ready", action="store_true")
    parser.add_argument("--confirm-arm-ownership-reviewed", action="store_true")
    args = parser.parse_args(argv)

    if args.execute_arm_sdk:
        parser.error(
            "--execute-arm-sdk is disabled after the 2026-08-27 controller-handoff "
            "incidents; this executable is restricted to non-publishing modes"
        )

    physical = not (args.offline_plan_only or args.probe_preflight)
    if physical and not (args.confirm_gantry_attached and args.confirm_estop_ready):
        parser.error(
            "physical modes require --confirm-gantry-attached and --confirm-estop-ready"
        )
    if (args.dry_run_arm_sdk or args.execute_arm_sdk) and args.post_action_pose_rad is None:
        parser.error("arm-SDK modes require 14 values in --post-action-pose-rad")
    if not 0.001 <= args.pose_tolerance_rad <= 0.02:
        parser.error("pose tolerance must be between 0.001 and 0.02 rad")
    if not 0.01 <= args.settle_velocity_rad_s <= 0.10:
        parser.error("settling velocity must be between 0.01 and 0.10 rad/s")
    if args.capture_duration < 0.5:
        parser.error("capture duration must be at least 0.5 seconds")
    if not 0.05 <= args.minimum_action_displacement_rad <= 0.50:
        parser.error("minimum action displacement must be between 0.05 and 0.50 rad")
    if (args.legacy_raise_task_id, args.legacy_release_task_id) != (2, 3):
        parser.error("legacy backend is restricted to explicit task IDs 2 then 3")
    return args


class LowStateMonitor:
    def __init__(
        self,
        recorder: TelemetryRecorder,
        telemetry_record_rate_hz: float = 100.0,
    ) -> None:
        if not 10.0 <= telemetry_record_rate_hz <= 250.0:
            raise ValueError("low-state recording rate must be between 10 and 250 Hz")
        self.recorder = recorder
        self._record_interval_ns = int(1e9 / telemetry_record_rate_hz)
        self._last_recorded_ns: Optional[int] = None
        self.subscriber: Any = None
        self.sport_subscriber: Any = None
        self._lock = threading.Lock()
        self._state: Any = None
        self._received_ns: Optional[int] = None
        self._sport_state: Any = None
        self._sport_received_ns: Optional[int] = None

    def start(self) -> None:
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self._receive, 10)
        self.sport_subscriber = ChannelSubscriber(
            "rt/sportmodestate", sport_mode_state_type()
        )
        self.sport_subscriber.Init(self._receive_sport, 10)

    def _receive(self, state: Any) -> None:
        received_ns = time.monotonic_ns()
        with self._lock:
            self._state = state
            self._received_ns = received_ns
        if (
            self._last_recorded_ns is None
            or received_ns - self._last_recorded_ns >= self._record_interval_ns
        ):
            self._last_recorded_ns = received_ns
            self.recorder.record(
                "unitree.lowstate",
                unitree_lowstate_record(state),
                timestamp_ns=received_ns,
                topic="rt/lowstate",
            )

    def latest(self) -> Tuple[Any, Optional[int]]:
        with self._lock:
            return self._state, self._received_ns

    def _receive_sport(self, state: Any) -> None:
        received_ns = time.monotonic_ns()
        with self._lock:
            self._sport_state = state
            self._sport_received_ns = received_ns
        self.recorder.record(
            "unitree.sportmodestate",
            {
                "fsm_id": state.fsm_id,
                "fsm_mode": state.fsm_mode,
                "task_id": state.task_id,
                "task_time": state.task_time,
            },
            timestamp_ns=received_ns,
            topic="rt/sportmodestate",
        )

    def latest_sport(self) -> Tuple[Any, Optional[int]]:
        with self._lock:
            return self._sport_state, self._sport_received_ns

    def close(self) -> None:
        if self.subscriber is not None:
            self.subscriber.Close()
            self.subscriber = None
        if self.sport_subscriber is not None:
            self.sport_subscriber.Close()
            self.sport_subscriber = None


class HighLevelArmActions:
    def __init__(self, raise_action: str, release_action: str) -> None:
        from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient, action_map

        if raise_action not in action_map or release_action not in action_map:
            raise RuntimeError(f"unknown arm action; available actions: {sorted(action_map)}")
        self.client = G1ArmActionClient()
        self.client.SetTimeout(15.0)
        self.client.Init()
        self.action_map = action_map
        self.raise_action = raise_action
        self.release_action = release_action

    def execute_raise(self) -> Any:
        return self.client.ExecuteAction(self.action_map[self.raise_action])

    def execute_release(self) -> Any:
        return self.client.ExecuteAction(self.action_map[self.release_action])

    def get_action_list(self) -> Any:
        code, actions = self.client.GetActionList()
        require_rpc_success("get arm action list", code)
        return actions

    def get_server_api_version(self) -> Tuple[int, Optional[str]]:
        return self.client.GetServerApiVersion()

    def close(self) -> None:
        close_rpc_client(self.client)


class LocoStateProbe:
    """Read FSM ID and mode without requesting a locomotion transition."""

    def __init__(self) -> None:
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

        self.client = LocoClient()
        self.client.SetTimeout(5.0)
        self.client.Init()

    def get_fsm(self) -> Tuple[int, int]:
        from unitree_sdk2py.g1.loco.g1_loco_api import ROBOT_API_ID_LOCO_GET_FSM_MODE

        code, fsm_id = self.client.GetFsmId()
        require_rpc_success("get locomotion FSM ID", code)
        code, data = self.client._Call(ROBOT_API_ID_LOCO_GET_FSM_MODE, json.dumps({}))
        require_rpc_success("get locomotion FSM mode", code)
        payload = json.loads(data)
        if not isinstance(payload, dict) or not isinstance(payload.get("data"), int):
            raise RuntimeError(f"invalid FSM mode response: {payload!r}")
        return int(fsm_id), int(payload["data"])

    def get_server_api_version(self) -> Tuple[int, Optional[str]]:
        return self.client.GetServerApiVersion()

    def set_task_id(self, task_id: int) -> Any:
        return self.client.SetTaskId(task_id)

    def close(self) -> None:
        close_rpc_client(self.client)


class LegacySportArmActions:
    """Explicit legacy task-2/task-3 backend using sport API 7106."""

    def __init__(self, loco: LocoStateProbe, raise_task_id: int = 2, release_task_id: int = 3) -> None:
        if (raise_task_id, release_task_id) != (2, 3):
            raise ValueError("legacy arm actions are restricted to task IDs 2 then 3")
        self.loco = loco
        self.raise_task_id = raise_task_id
        self.release_task_id = release_task_id
        self.raise_action = f"sport task {raise_task_id} (ShakeHand stage 0)"
        self.release_action = f"sport task {release_task_id} (ShakeHand stage 1)"

    def execute_raise(self) -> Any:
        return self.loco.set_task_id(self.raise_task_id)

    def execute_release(self) -> Any:
        return self.loco.set_task_id(self.release_task_id)

    def close(self) -> None:
        pass


def select_high_level_backend(
    loco: Any,
    modern: Any,
    requested_backend: str = "arm-action",
    legacy_raise_task_id: int = 2,
    legacy_release_task_id: int = 3,
) -> Tuple[Any, Dict[str, Any]]:
    """Select explicitly; firmware introspection RPCs are not reliable evidence."""
    if requested_backend == "arm-action":
        return modern, {
            "backend": "arm_action",
            "selection_basis": "configured_default_and_physical_raise_evidence",
        }
    if requested_backend == "legacy-sport":
        return LegacySportArmActions(
            loco, legacy_raise_task_id, legacy_release_task_id
        ), {
            "backend": "legacy_sport_task",
            "selection_basis": "explicit_operator_override",
            "warning": "legacy task 3 safe-return semantics are not verified",
        }
    raise ValueError(f"unsupported high-level backend: {requested_backend!r}")


def xr_motion_mode_initialization(initial_state: Any) -> Dict[str, Any]:
    """Return the exact non-publishing LowCmd initialization used by xr_teleoperate."""
    if initial_state is None or len(initial_state.motor_state) < 29:
        raise RuntimeError("fresh 29-motor low state is required")
    weak = {4, 10, 15, 16, 17, 18, 22, 23, 24, 25}
    wrists = {19, 20, 21, 26, 27, 28}
    motors = []
    for index in range(29):
        if index in wrists:
            kp, kd = 40.0, 1.5
        elif index in weak:
            kp, kd = 80.0, 3.0
        else:
            kp, kd = 300.0, 3.0
        motors.append({"index": index, "mode": 1,
                       "q": float(initial_state.motor_state[index].q), "kp": kp, "kd": kd})
    return {"mode_pr": 0, "mode_machine": int(initial_state.mode_machine), "motors": motors}


class ArmSdkCommandSink:
    def __init__(self, initial_state: Any) -> None:
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
        from unitree_sdk2py.utils.crc import CRC

        self.command = unitree_hg_msg_dds__LowCmd_()
        self.crc = CRC()
        initialization = xr_motion_mode_initialization(initial_state)
        self.command.mode_pr = initialization["mode_pr"]
        self.command.mode_machine = initialization["mode_machine"]
        for spec in initialization["motors"]:
            motor = self.command.motor_cmd[spec["index"]]
            motor.mode, motor.q = spec["mode"], spec["q"]
            motor.kp, motor.kd = spec["kp"], spec["kd"]
        self.publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.publisher.Init()
        self._mode_machine = initialization["mode_machine"]

    @staticmethod
    def configuration(mode_machine: Optional[int] = None) -> Dict[str, Any]:
        result = {
            "topic": "rt/arm_sdk",
            "motor_mode": 1,
            "authority_weight_joint": 29,
            "arm_joint_indices": list(ARM_JOINT_INDICES),
            "gains": {
                str(index): {"kp": arm_sdk_gains(index)[0], "kd": arm_sdk_gains(index)[1]}
                for index in ARM_JOINT_INDICES
            },
            "feedforward_torque_nm": "per_joint_bounded_rnea",
            "message_initialization": "xr_teleoperate_g1_29_motion_mode",
            "all_motor_indices_initialized_from_lowstate": list(range(29)),
        }
        if mode_machine is not None:
            result["mode_pr"] = 0
            result["mode_machine"] = mode_machine
        return result

    def runtime_configuration(self) -> Dict[str, Any]:
        return self.configuration(self._mode_machine)

    def __call__(self, positions: Sequence[float], velocities: Sequence[float], torques: Sequence[float], weight: float) -> None:
        self.command.motor_cmd[29].q = weight
        for index in ARM_JOINT_INDICES:
            motor = self.command.motor_cmd[index]
            motor.mode = 1
            motor.tau = torques[index]
            motor.q = positions[index]
            motor.dq = velocities[index]
            motor.kp, motor.kd = arm_sdk_gains(index)
        self.command.crc = self.crc.Crc(self.command)
        self.publisher.Write(self.command)


def _load_sdk_path() -> None:
    sdk_path = os.environ.get("UNITREE_SDK2_PYTHON", os.path.expanduser("~/unitree_sdk2_python"))
    if os.path.isdir(sdk_path) and sdk_path not in sys.path:
        sys.path.insert(0, sdk_path)


def run_preflight(
    monitor: Any,
    loco: LocoStateProbe,
    actions: HighLevelArmActions,
    event: Any,
    sport_timeout_seconds: float = 3.0,
    probe_action_list: bool = False,
) -> Dict[str, Any]:
    failures = []
    advisories = []
    fsm_id = fsm_mode = None
    action_list = None
    deadline = time.monotonic() + sport_timeout_seconds
    while time.monotonic() < deadline:
        sport_state, received_ns = monitor.latest_sport()
        if sport_state is not None and received_ns is not None:
            age_seconds = (time.monotonic_ns() - received_ns) / 1e9
            if age_seconds <= 0.10:
                fsm_id, fsm_mode = int(sport_state.fsm_id), int(sport_state.fsm_mode)
                break
        time.sleep(0.02)
    if fsm_id is None:
        try:
            fsm_id, fsm_mode = loco.get_fsm()
        except Exception as exc:
            failures.append(f"locomotion FSM probe: {type(exc).__name__}: {exc}")
    if probe_action_list and hasattr(actions, "get_action_list"):
        try:
            action_list = actions.get_action_list()
        except Exception as exc:
            advisories.append(f"arm action-list probe: {type(exc).__name__}: {exc}")
    elif hasattr(actions, "get_action_list"):
        advisories.append(
            "arm action-list discovery skipped because it is unreliable on this firmware"
        )
    details = {
        "fsm_id": fsm_id,
        "fsm_mode": fsm_mode,
        "arm_action_list": action_list,
        "failures": failures,
        "advisories": advisories,
    }
    event("preflight_observed", details)
    if failures:
        raise RuntimeError("preflight service failures: " + "; ".join(failures))
    validate_arm_action_fsm(details["fsm_id"], details["fsm_mode"])
    event("preflight_passed", {"fsm_id": details["fsm_id"], "fsm_mode": details["fsm_mode"]})
    return details


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    plan = BoundedArmPlan(
        amplitude_rad=args.amplitude_rad,
        duration_seconds=args.movement_duration,
        sample_rate_hz=args.sample_rate_hz,
        blend_seconds=args.blend_duration,
    )
    try:
        plan.validate()
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    if args.offline_plan_only:
        print(json.dumps({"mode": "offline_plan_only", "samples": plan.samples()}, indent=2))
        return 0

    _load_sdk_path()
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize

    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output = args.output or Path("telemetry") / "standalone_arm" / f"sequence_{run_id}.jsonl"
    recorder = TelemetryRecorder(output, queue_size=8192)
    monitor: Optional[LowStateMonitor] = None
    actions: Optional[Any] = None
    modern_actions: Optional[HighLevelArmActions] = None
    loco: Optional[LocoStateProbe] = None
    raise_requested = False
    pre_action_pose: Optional[Dict[int, float]] = None
    result, reason = "aborted", "initialization_failed"

    def event(name: str, details: Dict[str, Any]) -> None:
        if name != "arm_sdk_command":
            print(f"{name}: {details}")
        recorder.record("standalone_arm.event", {"event": name, **details})

    recorder.start()
    recorder.record(
        "standalone_arm.metadata",
        {
            "run_id": run_id,
            "mode": (
                "probe_preflight"
                if args.probe_preflight
                else "capture_post_action_pose"
                if args.capture_post_action_pose
                else "execute_arm_sdk"
                if args.execute_arm_sdk
                else "dry_run_arm_sdk"
            ),
            "network_interface": args.network_interface,
            "raise_action": args.raise_action,
            "release_action": args.release_action,
            "high_level_backend": args.high_level_backend,
            "gantry_confirmed": args.confirm_gantry_attached,
            "estop_confirmed": args.confirm_estop_ready,
            "arm_ownership_reviewed": args.confirm_arm_ownership_reviewed,
            "safe_return_pose_rad": args.safe_return_pose_rad,
            "allow_action_timeout_with_telemetry_verification": (
                args.allow_action_timeout_with_telemetry_verification
            ),
            "minimum_action_displacement_rad": args.minimum_action_displacement_rad,
            "plan": plan.__dict__,
        },
    )
    try:
        ChannelFactoryInitialize(0, args.network_interface)
        monitor = LowStateMonitor(recorder)
        monitor.start()
        loco = LocoStateProbe()
        modern_actions = HighLevelArmActions(args.raise_action, args.release_action)
        actions, capability = select_high_level_backend(
            loco,
            modern_actions,
            args.high_level_backend,
            args.legacy_raise_task_id,
            args.legacy_release_task_id,
        )
        event("high_level_backend_selected", capability)
        run_preflight(monitor, loco, actions, event)
        if args.probe_preflight:
            result, reason = "success", "preflight_passed"
            return_code = 0
            print(f"result={result}; reason={reason}; telemetry={output}")
            return return_code
        pre_action_pose = capture_pose_centers(monitor.latest, 0.5)
        event(
            "pre_action_pose_captured",
            {"centers_rad": [pre_action_pose[index] for index in ARM_JOINT_INDICES]},
        )
        event("high_level_raise_requested", {"action": actions.raise_action})
        raise_requested = True
        ret = actions.execute_raise()
        event("high_level_raise_returned", {"return_value": ret})
        require_rpc_success_or_defer_timeout(
            actions.raise_action,
            ret,
            args.allow_action_timeout_with_telemetry_verification,
            event,
        )

        expected = None
        if args.post_action_pose_rad is not None:
            expected = {
                index: args.post_action_pose_rad[index - ARM_JOINT_INDICES[0]]
                for index in ARM_JOINT_INDICES
            }
        settled_state = wait_for_settled_state(
            monitor.latest,
            expected,
            event,
            pose_tolerance_rad=args.pose_tolerance_rad,
            velocity_limit_rad_s=args.settle_velocity_rad_s,
            required_duration_seconds=args.settle_duration,
            timeout_seconds=args.settle_timeout,
        )
        displacement = require_arm_displacement(
            settled_state,
            pre_action_pose,
            args.minimum_action_displacement_rad,
        )
        event("high_level_raise_verified_by_telemetry", displacement)
        if args.capture_post_action_pose:
            centers = capture_pose_centers(monitor.latest, args.capture_duration)
            ordered = [centers[index] for index in ARM_JOINT_INDICES]
            event("post_action_pose_captured", {"centers_rad": ordered})
            print("Suggested reviewed argument (repeat captures before approval):")
            print("--post-action-pose-rad " + " ".join(f"{value:.6f}" for value in ordered))
            result, reason = "success", "post_action_pose_captured"
        else:
            if args.execute_arm_sdk:
                current_state, current_state_ns = monitor.latest()
                if current_state is None or current_state_ns is None:
                    raise RuntimeError("low state unavailable before publisher construction")
                sink = ArmSdkCommandSink(current_state)
                event("arm_sdk_publisher_initialized", sink.runtime_configuration())
            else:
                sink = lambda *_: None
            executor = BoundedArmExecutor(
                plan,
                monitor.latest,
                sink,
                event,
                publish_commands=args.execute_arm_sdk,
            )
            outcome, arm_reason = executor.run(settled_state)
            result = "success" if outcome == "completed" else "aborted"
            reason = arm_reason
    except KeyboardInterrupt:
        result, reason = "aborted", "operator_cancelled"
        event("operator_cancelled", {})
    except BaseException as exc:
        result, reason = "aborted", f"{type(exc).__name__}: {exc}"
        event("sequence_failed", {"reason": reason})
    finally:
        if raise_requested and actions is not None:
            try:
                event("high_level_release_requested", {"action": actions.release_action})
                ret = actions.execute_release()
                event("high_level_release_returned", {"return_value": ret})
                require_rpc_success_or_defer_timeout(
                    actions.release_action,
                    ret,
                    args.allow_action_timeout_with_telemetry_verification,
                    event,
                )
                return_expected = pre_action_pose
                if args.safe_return_pose_rad is not None:
                    return_expected = {
                        index: args.safe_return_pose_rad[index - ARM_JOINT_INDICES[0]]
                        for index in ARM_JOINT_INDICES
                    }

                def return_event(name: str, details: Dict[str, Any]) -> None:
                    event(f"safe_return_{name}", details)

                wait_for_settled_state(
                    monitor.latest,
                    return_expected,
                    return_event,
                    pose_tolerance_rad=args.pose_tolerance_rad,
                    velocity_limit_rad_s=args.settle_velocity_rad_s,
                    required_duration_seconds=args.settle_duration,
                    timeout_seconds=args.settle_timeout,
                )
                return_centers = capture_pose_centers(monitor.latest, 0.5)
                ordered_return = [return_centers[index] for index in ARM_JOINT_INDICES]
                event(
                    "safe_return_pose_observed",
                    {
                        "centers_rad": ordered_return,
                        "reviewed_envelope_applied": return_expected is not None,
                    },
                )
                if args.capture_post_action_pose:
                    print("Suggested safe-return argument (repeat captures before approval):")
                    print(
                        "--safe-return-pose-rad "
                        + " ".join(f"{value:.6f}" for value in ordered_return)
                    )
            except BaseException as exc:
                result = "aborted"
                reason = f"high_level_release_or_return_failed: {type(exc).__name__}: {exc}"
                event("high_level_release_or_return_failed", {"reason": reason})
        if monitor is not None:
            monitor.close()
        if actions is not None:
            actions.close()
        if modern_actions is not None and actions is not modern_actions:
            modern_actions.close()
        if loco is not None:
            loco.close()
        recorder.record("standalone_arm.summary", {"result": result, "reason": reason})
        recorder.close()
    print(f"result={result}; reason={reason}; telemetry={output}")
    return 0 if result == "success" else 1


if __name__ == "__main__":
    raise SystemExit(main())
