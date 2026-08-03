#!/usr/bin/env python3
"""
BrainCo Revo2 tactile handshake demo for Unitree G1 PC2.

Version note:
  This version avoids common_imports.has_touch() and
  common_imports.is_array_pressure_touch(), because some bc-stark-sdk versions
  do not expose StarkHardwareType.Revo2TouchForce3D, which can cause:
      AttributeError: type object 'builtins.StarkHardwareType'
      has no attribute 'Revo2TouchForce3D'

Behavior:
  1. Keep the selected hand fully open while no touch is detected.
  2. When touch/contact is detected, close slowly.
  3. Stop closing when either:
       - tactile value reaches --stop-threshold, or
       - commanded close reaches --max-close, default 500 = 1/2 closed.
  4. Reopen when touch is released for --release-seconds or the hold timeout expires.

Important:
  - Do NOT run this while launch_robot.sh / stark_node is running.
    Only one process can own the BrainCo hand serial port.
  - Start with --dry-run first.
  - Start with a conservative --max-close such as 500 or 600 before using 750.
"""

import argparse
import asyncio
import os
import sys
import threading
import time
from typing import Any, Iterable, List, Optional, Sequence, Tuple

from handshake_state import HandshakeConfig, HandshakeStateMachine


DEFAULT_LEFT_PORT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA1LW3T-if02-port0"
DEFAULT_RIGHT_PORT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA1LW3T-if01-port0"
DEFAULT_LEFT_ID = 0x7E
DEFAULT_RIGHT_ID = 0x7F
DEFAULT_ARM_ACTION = "shake hand"
DEFAULT_ARM_RELEASE_ACTION = "release arm"


def add_brainco_sdk_path() -> None:
    candidates = [
        os.environ.get("BRAINCO_SDK_PYTHON", ""),
        os.path.expanduser("~/brainco-hand-sdk/python"),
        os.path.expanduser("~/stark-serialport-example/python"),
    ]
    for path in candidates:
        if path and os.path.isdir(path) and path not in sys.path:
            sys.path.insert(0, path)


add_brainco_sdk_path()


def add_unitree_sdk_path() -> None:
    candidates = [
        os.environ.get("UNITREE_SDK2_PYTHON", ""),
        os.path.expanduser("~/unitree_sdk2_python"),
    ]
    for path in candidates:
        if path and os.path.isdir(path) and path not in sys.path:
            sys.path.insert(0, path)


add_unitree_sdk_path()

try:
    # Do NOT import has_touch or is_array_pressure_touch from common_imports.
    # Those helpers may reference enum names that are absent in older SDK wheels.
    from common_imports import (
        sdk,
        check_sdk,
        int_to_baudrate,
        get_hw_type_name,
    )
except Exception as exc:  # pragma: no cover - depends on user's PC2 environment
    print("Failed to import BrainCo SDK helper modules.", file=sys.stderr)
    print("Expected one of these directories to exist:", file=sys.stderr)
    print("  ~/brainco-hand-sdk/python", file=sys.stderr)
    print("  ~/stark-serialport-example/python", file=sys.stderr)
    print(f"Import error: {exc}", file=sys.stderr)
    sys.exit(1)


def hw_int_value(hw_type: Any) -> Optional[int]:
    """Best-effort conversion of StarkHardwareType enum to its integer code."""
    for attr in ("int_value", "value"):
        if hasattr(hw_type, attr):
            try:
                v = getattr(hw_type, attr)
                return int(v() if callable(v) else v)
            except Exception:
                pass
    try:
        return int(hw_type)
    except Exception:
        return None


def hw_name(hw_type: Any) -> str:
    """Stable-ish string name for hardware enum across SDK versions."""
    try:
        return str(hw_type)
    except Exception:
        return repr(hw_type)


def safe_hw_type_name(hw_type: Any) -> str:
    """Friendly hardware name without crashing on SDK enum mismatches."""
    try:
        return get_hw_type_name(hw_type)
    except Exception:
        name = hw_name(hw_type)
        val = hw_int_value(hw_type)
        return f"{name}" + (f" ({val})" if val is not None else "")


def safe_has_touch(hw_type: Any) -> bool:
    """
    Avoid common_imports.has_touch() because it can reference missing enum names.

    Detection order:
      1. SDK enum method has_touch(), if available and safe.
      2. Hardware type name contains "Touch".
      3. Known numeric touch-capable Revo hardware IDs.
    """
    if hasattr(hw_type, "has_touch"):
        try:
            return bool(hw_type.has_touch())
        except Exception:
            pass

    name = hw_name(hw_type).lower()
    if "touch" in name or "pressure" in name:
        return True

    value = hw_int_value(hw_type)
    known_touch_ids = {
        2,   # Revo1Touch
        4,   # Revo1AdvancedTouch
        11,  # Revo2Touch
        12,  # Revo2TouchPressure
        13,  # Revo2TouchForce3D, name may not exist in older SDK
        14,  # Revo2TouchArrayPressure
        21,  # Revo3UltraTouch
        22,  # Revo3UltraVisionTouch
        24,  # Revo3ProTouch
        27,  # Revo3BasicTouch
    }
    return value in known_touch_ids


def safe_is_array_pressure(hw_type: Any) -> bool:
    """Avoid common_imports.is_array_pressure_touch() for SDK compatibility."""
    if hasattr(hw_type, "is_array_pressure_touch"):
        try:
            return bool(hw_type.is_array_pressure_touch())
        except Exception:
            pass

    name = hw_name(hw_type).lower()
    if "arraypressure" in name or "array_pressure" in name:
        return True

    return hw_int_value(hw_type) == 14


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Tactile handshake demo: open when idle, close slowly until a touch or close limit."
    )

    hand = parser.add_mutually_exclusive_group()
    hand.add_argument("--left", action="store_true", help="Use left hand, slave 0x7e on FTDI if02.")
    hand.add_argument("--right", action="store_true", help="Use right hand, slave 0x7f on FTDI if01.")

    parser.add_argument("--port", default=None, help="Override serial port.")
    parser.add_argument("--slave-id", type=lambda x: int(x, 0), default=None, help="Override slave ID, e.g. 126 or 0x7e.")
    parser.add_argument("--baud", type=int, default=460800, help="Modbus baudrate. Default: 460800.")

    parser.add_argument("--start-threshold", type=float, default=50.0,
                        help="Touch value that starts closing from open. Default: 50.")
    parser.add_argument("--stop-threshold", type=float, default=250.0,
                        help="Touch value that stops closing. Default: 250.")
    parser.add_argument("--release-threshold", type=float, default=20.0,
                        help="Below this value counts as released. Default: 20.")
    parser.add_argument("--release-seconds", type=float, default=0.7,
                        help="Seconds below release threshold before reopening. Default: 0.7.")
    parser.add_argument("--hold-duration", type=float, default=5.0,
                        help="Seconds to remain in hold before reopening. Default: 5.0.")

    parser.add_argument("--max-close", type=int, default=500,
                        help="Maximum close command, 0=open, 1000=fully closed. Default: 500.")
    parser.add_argument("--step", type=int, default=50,
                        help="Close command increment per step. Default: 50.")
    parser.add_argument("--period", type=float, default=0.10,
                        help="Control loop period in seconds. Default: 0.10.")
    parser.add_argument("--open-repeat", type=float, default=1.0,
                        help="Repeat open command every N seconds while idle. Default: 1.0.")
    parser.add_argument("--sensor-timeout", type=float, default=1.0,
                        help="Maximum seconds allowed for one tactile read. Default: 1.0.")

    parser.add_argument("--thumb-scale", type=float, default=1.0,
                        help="Scale thumb/thumb_aux close target relative to fingers. Default: 1.0.")
    parser.add_argument("--dry-run", action="store_true",
                        help="Read tactile values and print decisions, but do not move the hand.")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Optional max runtime in seconds. 0 means run until Ctrl+C.")
    parser.add_argument("--quiet", action="store_true", help="Print less often.")
    parser.add_argument("--ignore-touch-type-check", action="store_true",
                        help="Proceed even if hardware type does not report touch support.")

    parser.add_argument("--enable-arm", action="store_true",
                        help="Also trigger a Unitree high-level arm action when touch starts closing.")
    parser.add_argument("--arm-network-interface", default=None,
                        help="DDS network interface for Unitree arm action service, e.g. eth0. Default: auto.")
    parser.add_argument("--arm-action", default=DEFAULT_ARM_ACTION,
                        help=f"Unitree arm action to run on touch. Default: {DEFAULT_ARM_ACTION!r}.")
    parser.add_argument("--arm-release-action", default=DEFAULT_ARM_RELEASE_ACTION,
                        help=f"Unitree arm action to run after the shake. Default: {DEFAULT_ARM_RELEASE_ACTION!r}.")
    parser.add_argument("--arm-release-delay", type=float, default=4.0,
                        help="Seconds after arm action before release action. Default: 4.0.")

    args = parser.parse_args(argv)

    if not args.left and not args.right:
        args.right = True

    if args.port is None:
        args.port = DEFAULT_RIGHT_PORT if args.right else DEFAULT_LEFT_PORT

    if args.slave_id is None:
        args.slave_id = DEFAULT_RIGHT_ID if args.right else DEFAULT_LEFT_ID

    if not args.release_threshold < args.start_threshold < args.stop_threshold:
        parser.error("thresholds must satisfy release-threshold < start-threshold < stop-threshold")
    if not 0 <= args.max_close <= 1000:
        parser.error("max-close must be between 0 and 1000")
    if not 1 <= args.step <= 1000:
        parser.error("step must be between 1 and 1000")
    if args.period < 0.02:
        parser.error("period must be at least 0.02 seconds")
    if args.release_seconds < 0 or args.hold_duration < 0 or args.arm_release_delay < 0:
        parser.error("release-seconds, hold-duration, and arm-release-delay must be nonnegative")
    if args.duration < 0:
        parser.error("duration must be nonnegative")
    if args.open_repeat <= 0 or args.sensor_timeout <= 0:
        parser.error("open-repeat and sensor-timeout must be greater than zero")
    if not 0.0 <= args.thumb_scale <= 1.0:
        parser.error("thumb-scale must be between 0.0 and 1.0")
    if not 0 <= args.slave_id <= 0xFF:
        parser.error("slave-id must be between 0 and 255")
    if args.baud <= 0:
        parser.error("baud must be greater than zero")

    return args


def as_signed_u16(value: int) -> int:
    value = int(value)
    return value if value < 32768 else value - 65536


def collect_attr_values(obj: Any, names: Iterable[str]) -> List[float]:
    values: List[float] = []
    for name in names:
        if hasattr(obj, name):
            try:
                values.append(float(getattr(obj, name)))
            except Exception:
                pass
    return values


def touch_metric_from_status_items(items: Sequence[Any]) -> Tuple[float, str]:
    """
    Return one scalar contact metric from capacitive TouchFingerItem list.

    Uses normal force primarily. Tangential force is included because a handshake
    may create shear before large normal force.
    """
    best = 0.0
    parts: List[str] = []

    finger_names = ["thumb", "index", "middle", "ring", "pinky"]
    force_fields = [
        "normal_force1", "normal_force2", "normal_force3",
        "tangential_force1", "tangential_force2", "tangential_force3",
    ]

    for idx, item in enumerate(items):
        vals = [abs(v) for v in collect_attr_values(item, force_fields)]
        metric = max(vals) if vals else 0.0
        best = max(best, metric)

        status = getattr(item, "status", None)
        name = finger_names[idx] if idx < len(finger_names) else f"f{idx}"
        parts.append(f"{name}={metric:.0f}" + (f"/s{status}" if status is not None else ""))

    return best, " ".join(parts)


def touch_metric_from_array_pressure(ap_data: Any) -> Tuple[float, str]:
    """
    Return metric from Revo2TouchArrayPressure data.

    The SDK demo describes 5 values per finger: Fx, Fy, Fz, Mx, My, raw scaled by 100.
    Fz is the normal force. We use max(abs(Fx), abs(Fy), abs(Fz)) in N.
    """
    if isinstance(ap_data, dict):
        raw = list(ap_data.get("data", []))
    else:
        raw = list(getattr(ap_data, "data", []))

    if len(raw) < 25:
        return 0.0, f"array_pressure_incomplete({len(raw)})"

    finger_names = ["thumb", "index", "middle", "ring", "pinky"]
    best = 0.0
    parts: List[str] = []
    for finger_idx, name in enumerate(finger_names):
        base = finger_idx * 5
        fx = as_signed_u16(raw[base + 0]) / 100.0
        fy = as_signed_u16(raw[base + 1]) / 100.0
        fz = as_signed_u16(raw[base + 2]) / 100.0
        metric = max(abs(fx), abs(fy), abs(fz))
        best = max(best, metric)
        parts.append(f"{name}={metric:.2f}N")

    return best, " ".join(parts)


async def get_touch_metric(ctx: Any, slave_id: int, hw_type: Any) -> Tuple[float, str]:
    """
    Read tactile metric. Try the ArrayPressure API only when hardware type says
    it is array-pressure. Otherwise use the capacitive touch API.
    """
    if safe_is_array_pressure(hw_type):
        data = await ctx.get_array_pressure_touch_data(slave_id)
        return touch_metric_from_array_pressure(data)

    items = await ctx.get_touch_sensor_status(slave_id)
    return touch_metric_from_status_items(items)


def make_positions(close_value: int, thumb_scale: float) -> List[int]:
    close_value = int(max(0, min(1000, close_value)))
    thumb = int(max(0, min(1000, close_value * thumb_scale)))
    # BrainCo SDK order: thumb, thumb_aux, index, middle, ring, pinky.
    return [thumb, thumb, close_value, close_value, close_value, close_value]


async def command_positions(
    ctx: Any,
    slave_id: int,
    positions: Sequence[int],
    dry_run: bool,
    timeout: Optional[float] = None,
) -> None:
    if dry_run:
        return
    operation = ctx.set_finger_positions(slave_id, [int(x) for x in positions])
    if timeout is None:
        await operation
    else:
        await asyncio.wait_for(operation, timeout=timeout)


class ArmActionRunner:
    """Nonblocking wrapper around Unitree's high-level G1 arm action service."""

    def __init__(
        self,
        enabled: bool,
        dry_run: bool,
        network_interface: Optional[str],
        action_name: str,
        release_action_name: str,
        release_delay: float,
    ) -> None:
        self.enabled = enabled
        self.dry_run = dry_run
        self.network_interface = network_interface
        self.action_name = action_name
        self.release_action_name = release_action_name
        self.release_delay = release_delay
        self.client: Any = None
        self.action_map: Any = None
        self._thread_lock = threading.Lock()
        self._client_lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._cancel_delayed_release = threading.Event()
        self._release_sent = threading.Event()

    def init(self) -> None:
        if not self.enabled:
            return

        if self.dry_run:
            print("arm: dry-run enabled; arm actions will be logged but not executed.")
            return

        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            from unitree_sdk2py.g1.arm.g1_arm_action_client import (
                G1ArmActionClient,
                action_map,
            )
        except Exception as exc:
            raise RuntimeError(
                "failed to import Unitree arm action client; set UNITREE_SDK2_PYTHON "
                "or install unitree_sdk2_python; underlying error: "
                f"{type(exc).__name__}: {exc}"
            ) from exc

        if self.action_name not in action_map:
            raise RuntimeError(f"unknown arm action {self.action_name!r}; known: {sorted(action_map)}")
        if self.release_action_name and self.release_action_name not in action_map:
            raise RuntimeError(
                f"unknown arm release action {self.release_action_name!r}; known: {sorted(action_map)}"
            )

        if self.network_interface:
            ChannelFactoryInitialize(0, self.network_interface)
        else:
            ChannelFactoryInitialize(0)

        self.action_map = action_map
        self.client = G1ArmActionClient()
        self.client.SetTimeout(10.0)
        self.client.Init()
        print(
            "arm: enabled; "
            f"action={self.action_name!r}, release={self.release_action_name!r}, "
            f"interface={self.network_interface or 'auto'}"
        )

    def trigger(self) -> None:
        if not self.enabled:
            return

        with self._thread_lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._cancel_delayed_release.clear()
            self._release_sent.clear()
            self._thread = threading.Thread(target=self._run_sequence, name="arm_action", daemon=True)
            self._thread.start()

    def _run_sequence(self) -> None:
        if self.dry_run:
            print(f"arm: would execute {self.action_name!r}")
            if self.release_action_name:
                print(f"arm: would execute {self.release_action_name!r} after {self.release_delay:.1f}s")
            return

        try:
            action_id = self.action_map[self.action_name]
            with self._client_lock:
                ret = self.client.ExecuteAction(action_id)
            print(f"arm: executed {self.action_name!r}, ret={ret}")

            if self.release_action_name:
                if not self._cancel_delayed_release.wait(self.release_delay):
                    self._execute_release_once()
        except Exception as exc:
            print(f"WARNING: arm action failed: {exc}", file=sys.stderr)

    def release_now(self) -> None:
        if not self.enabled or self.dry_run or not self.release_action_name:
            return

        self._cancel_delayed_release.set()
        try:
            self._execute_release_once()
        except Exception as exc:
            print(f"WARNING: arm release failed: {exc}", file=sys.stderr)

    def _execute_release_once(self) -> None:
        with self._client_lock:
            if self._release_sent.is_set():
                return
            release_id = self.action_map[self.release_action_name]
            ret = self.client.ExecuteAction(release_id)
            self._release_sent.set()
        print(f"arm: executed {self.release_action_name!r}, ret={ret}")


async def maybe_get_motor_positions(
    ctx: Any, slave_id: int, timeout: float
) -> Optional[List[int]]:
    try:
        status = await asyncio.wait_for(ctx.get_motor_status(slave_id), timeout=timeout)
        return [int(x) for x in list(status.positions)]
    except Exception:
        return None


async def main() -> int:
    args = parse_args()
    check_sdk()

    if not os.path.exists(args.port):
        print(f"ERROR: port does not exist: {args.port}", file=sys.stderr)
        print("Run: ls -l /dev/serial/by-id/", file=sys.stderr)
        return 2

    baud_enum = int_to_baudrate(args.baud)

    print("=== BrainCo tactile handshake demo ===")
    print(f"port:              {args.port}")
    print(f"slave_id:          {args.slave_id} / 0x{args.slave_id:02x}")
    print(f"baud:              {args.baud}")
    print(f"start_threshold:   {args.start_threshold}")
    print(f"stop_threshold:    {args.stop_threshold}")
    print(f"release_threshold: {args.release_threshold}")
    print(f"max_close:         {args.max_close} / 1000")
    print(f"sensor_timeout:    {args.sensor_timeout}s")
    print(f"dry_run:           {args.dry_run}")
    print(f"enable_arm:        {args.enable_arm}")
    if args.enable_arm:
        print()
        print("ALERT: get the robot to regular mode")
        print("       Use the controller sequence: Damping L2+B -> Ready L2+Up -> Regular R1+X")
    print()

    ctx = None
    arm = ArmActionRunner(
        enabled=args.enable_arm,
        dry_run=args.dry_run,
        network_interface=args.arm_network_interface,
        action_name=args.arm_action,
        release_action_name=args.arm_release_action,
        release_delay=args.arm_release_delay,
    )
    try:
        arm.init()
        ctx = await sdk.modbus_open(args.port, baud_enum)

        info = await ctx.get_device_info(args.slave_id)
        hw_type = info.hardware_type
        print(f"device: {safe_hw_type_name(hw_type)}")
        print(f"raw hardware type: {hw_name(hw_type)}; int={hw_int_value(hw_type)}")

        if not safe_has_touch(hw_type) and not args.ignore_touch_type_check:
            print("ERROR: this hand does not report tactile support.", file=sys.stderr)
            print("If hand_monitor.py touch works anyway, rerun with --ignore-touch-type-check.", file=sys.stderr)
            return 3

        # Revo2 capacitive touch demo enables all five tactile sensors with 0x1F.
        # Some hardware/SDK variants do not need this; warning is nonfatal.
        try:
            await ctx.touch_sensor_setup(args.slave_id, 0x1F)
            await asyncio.sleep(0.5)
        except Exception as exc:
            print(f"WARNING: touch_sensor_setup failed, trying to continue: {exc}")

        machine = HandshakeStateMachine(
            HandshakeConfig(
                start_threshold=args.start_threshold,
                stop_threshold=args.stop_threshold,
                release_threshold=args.release_threshold,
                release_seconds=args.release_seconds,
                hold_duration=args.hold_duration,
                max_close=args.max_close,
                step=args.step,
            )
        )
        last_open_command = 0.0
        started_at = time.monotonic()
        tick = 0

        print("Starting loop. Press Ctrl+C to stop.")
        print("State legend: open_wait -> closing -> hold -> open_wait")
        print("Release behavior: confirmed release opens hand, releases arm, and waits for rearm.")
        print()

        # Start fully open.
        await command_positions(
            ctx,
            args.slave_id,
            make_positions(0, args.thumb_scale),
            args.dry_run,
            args.sensor_timeout,
        )

        while True:
            now = time.monotonic()
            if args.duration > 0 and now - started_at >= args.duration:
                print("Duration reached; exiting through safe cleanup.")
                return 0

            try:
                metric, detail = await asyncio.wait_for(
                    get_touch_metric(ctx, args.slave_id, hw_type),
                    timeout=args.sensor_timeout,
                )
            except asyncio.TimeoutError as exc:
                raise RuntimeError(
                    f"tactile read exceeded sensor timeout of {args.sensor_timeout:.2f}s"
                ) from exc

            decision = machine.update(metric, now)

            if decision.state.value == "open_wait" and now - last_open_command >= args.open_repeat:
                await command_positions(
                    ctx,
                    args.slave_id,
                    make_positions(0, args.thumb_scale),
                    args.dry_run,
                    args.sensor_timeout,
                )
                last_open_command = now

            if decision.command_close is not None:
                await command_positions(
                    ctx,
                    args.slave_id,
                    make_positions(decision.command_close, args.thumb_scale),
                    args.dry_run,
                    args.sensor_timeout,
                )
            if decision.trigger_arm:
                arm.trigger()
            if decision.release_arm:
                print(f"{decision.event}; opening hand and releasing arm.")
                arm.release_now()

            positions = None
            if tick % max(1, int(1.0 / args.period)) == 0:
                positions = await maybe_get_motor_positions(ctx, args.slave_id, args.sensor_timeout)

            if (not args.quiet) or tick % max(1, int(1.0 / args.period)) == 0:
                pos_str = f" motor={positions}" if positions is not None else ""
                armed = " armed" if machine.ready_for_contact else " disarmed"
                print(
                    f"{decision.state.value:10s}{armed:9s} touch={metric:7.2f} "
                    f"close_cmd={decision.close_value:4d}{pos_str} | {detail}"
                )

            tick += 1
            await asyncio.sleep(args.period)

    except PermissionError:
        print(f"ERROR: permission denied opening {args.port}", file=sys.stderr)
        print("Run your permission script first:", file=sys.stderr)
        print("  ~/bin/g1_fix_serial_permissions.sh", file=sys.stderr)
        return 13
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 15
    except AttributeError as exc:
        print("ERROR: SDK AttributeError:", exc, file=sys.stderr)
        print("This is usually a bc-stark-sdk / brainco-hand-sdk version mismatch.", file=sys.stderr)
        print("Try updating the SDK examples and wheel, or send this full output.", file=sys.stderr)
        return 14
    except asyncio.CancelledError:
        print("\nCancellation received; running safe cleanup.", file=sys.stderr)
        raise
    except Exception as exc:
        print(f"ERROR: unexpected {type(exc).__name__}: {exc}", file=sys.stderr)
        return 16
    finally:
        if ctx is not None:
            try:
                await asyncio.wait_for(
                    command_positions(ctx, args.slave_id, make_positions(0, args.thumb_scale), args.dry_run),
                    timeout=2.0,
                )
                print("cleanup: hand open command sent.")
            except BaseException as exc:
                print(f"WARNING: cleanup could not open hand: {exc}", file=sys.stderr)
            try:
                arm.release_now()
            except BaseException as exc:
                print(f"WARNING: cleanup could not release arm: {exc}", file=sys.stderr)
            try:
                sdk.modbus_close(ctx)
            except Exception:
                pass

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(asyncio.run(main()))
    except KeyboardInterrupt:
        raise SystemExit(130)
