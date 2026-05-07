#!/usr/bin/env python3
"""
BrainCo Revo2 tactile handshake demo for Unitree G1 PC2.

Behavior:
  1. Keep the selected hand fully open while no touch is detected.
  2. When touch/contact is detected, close slowly.
  3. Stop closing when either:
       - tactile value reaches --stop-threshold, or
       - commanded close reaches --max-close, default 750 = 3/4 closed.
  4. Reopen when touch is released for --release-seconds.

Important:
  - Do NOT run this while launch_robot.sh / stark_node is running.
    Only one process can own the BrainCo hand serial port.
  - Start with --dry-run first.
  - Start with a conservative --max-close such as 500 or 600 before using 750.
  - Thresholds are raw tactile values from the BrainCo SDK and should be tuned
    using hand_monitor.py or this script's live printout.

Known G1 mapping from this setup:
  left  hand = FTDI if02 = /dev/serial/by-id/...if02-port0 = slave 126 / 0x7e
  right hand = FTDI if01 = /dev/serial/by-id/...if01-port0 = slave 127 / 0x7f
"""

import argparse
import asyncio
import os
import sys
import time
from typing import Any, Iterable, List, Optional, Sequence, Tuple


DEFAULT_LEFT_PORT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA1LW3T-if02-port0"
DEFAULT_RIGHT_PORT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA1LW3T-if01-port0"
DEFAULT_LEFT_ID = 0x7E
DEFAULT_RIGHT_ID = 0x7F


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

try:
    from common_imports import (
        sdk,
        check_sdk,
        int_to_baudrate,
        get_hw_type_name,
        has_touch,
        is_array_pressure_touch,
    )
except Exception as exc:  # pragma: no cover - depends on user's PC2 environment
    print("Failed to import BrainCo SDK helper modules.", file=sys.stderr)
    print("Expected one of these directories to exist:", file=sys.stderr)
    print("  ~/brainco-hand-sdk/python", file=sys.stderr)
    print("  ~/stark-serialport-example/python", file=sys.stderr)
    print(f"Import error: {exc}", file=sys.stderr)
    sys.exit(1)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Tactile handshake demo: open when no touch, close slowly until touch threshold or 3/4 close."
    )

    hand = parser.add_mutually_exclusive_group()
    hand.add_argument("--left", action="store_true", help="Use left hand, slave 0x7e on FTDI if02.")
    hand.add_argument("--right", action="store_true", help="Use right hand, slave 0x7f on FTDI if01.")

    parser.add_argument("--port", default=None, help="Override serial port.")
    parser.add_argument("--slave-id", type=lambda x: int(x, 0), default=None, help="Override slave ID, e.g. 126 or 0x7e.")
    parser.add_argument("--baud", type=int, default=460800, help="Modbus baudrate. Default: 460800.")

    parser.add_argument("--start-threshold", type=float, default=20.0,
                        help="Touch value that starts closing from open. Default: 20.")
    parser.add_argument("--stop-threshold", type=float, default=80.0,
                        help="Touch value that stops closing. Default: 80.")
    parser.add_argument("--release-threshold", type=float, default=10.0,
                        help="Below this value counts as released. Default: 10.")
    parser.add_argument("--release-seconds", type=float, default=0.7,
                        help="Seconds below release threshold before reopening. Default: 0.7.")

    parser.add_argument("--max-close", type=int, default=750,
                        help="Maximum close command, 0=open, 1000=fully closed. Default 750 = 3/4 closed.")
    parser.add_argument("--step", type=int, default=25,
                        help="Close command increment per step. Default: 25.")
    parser.add_argument("--period", type=float, default=0.15,
                        help="Control loop period in seconds. Default: 0.15.")
    parser.add_argument("--open-repeat", type=float, default=1.0,
                        help="Repeat open command every N seconds while idle. Default: 1.0.")

    parser.add_argument("--thumb-scale", type=float, default=1.0,
                        help="Scale thumb/thumb_aux close target relative to fingers. Default: 1.0.")
    parser.add_argument("--dry-run", action="store_true",
                        help="Read tactile values and print decisions, but do not move the hand.")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Optional max runtime in seconds. 0 means run until Ctrl+C.")
    parser.add_argument("--quiet", action="store_true", help="Print less often.")

    args = parser.parse_args()

    if not args.left and not args.right and args.port is None and args.slave_id is None:
        # Default to left hand because that is what the user tested first.
        args.left = True

    if args.port is None:
        args.port = DEFAULT_RIGHT_PORT if args.right else DEFAULT_LEFT_PORT

    if args.slave_id is None:
        args.slave_id = DEFAULT_RIGHT_ID if args.right else DEFAULT_LEFT_ID

    args.max_close = max(0, min(1000, args.max_close))
    args.step = max(1, min(1000, args.step))
    args.period = max(0.02, args.period)
    args.release_seconds = max(0.0, args.release_seconds)

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
    if is_array_pressure_touch(hw_type):
        data = await ctx.get_array_pressure_touch_data(slave_id)
        return touch_metric_from_array_pressure(data)

    # Capacitive Revo2 Touch / Revo1 Touch path.
    items = await ctx.get_touch_sensor_status(slave_id)
    return touch_metric_from_status_items(items)


def make_positions(close_value: int, thumb_scale: float) -> List[int]:
    close_value = int(max(0, min(1000, close_value)))
    thumb = int(max(0, min(1000, close_value * thumb_scale)))
    # BrainCo SDK order: thumb, thumb_aux, index, middle, ring, pinky.
    return [thumb, thumb, close_value, close_value, close_value, close_value]


async def command_positions(ctx: Any, slave_id: int, positions: Sequence[int], dry_run: bool) -> None:
    if dry_run:
        return
    await ctx.set_finger_positions(slave_id, [int(x) for x in positions])


async def maybe_get_motor_positions(ctx: Any, slave_id: int) -> Optional[List[int]]:
    try:
        status = await ctx.get_motor_status(slave_id)
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
    print(f"dry_run:           {args.dry_run}")
    print()

    ctx = None
    try:
        ctx = await sdk.modbus_open(args.port, baud_enum)

        info = await ctx.get_device_info(args.slave_id)
        hw_type = info.hardware_type
        print(f"device: {get_hw_type_name(hw_type)}")
        if not has_touch(hw_type):
            print("ERROR: this hand does not report tactile support.", file=sys.stderr)
            return 3

        # Revo2 capacitive touch demo enables all five tactile sensors with 0x1F.
        try:
            await ctx.touch_sensor_setup(args.slave_id, 0x1F)
            await asyncio.sleep(0.5)
        except Exception as exc:
            print(f"WARNING: touch_sensor_setup failed, trying to continue: {exc}")

        state = "open_wait"
        close_value = 0
        last_open_command = 0.0
        release_started: Optional[float] = None
        started_at = time.monotonic()
        tick = 0

        print("Starting loop. Press Ctrl+C to stop.")
        print("State legend: open_wait -> closing -> hold -> open_wait")
        print()

        # Start fully open.
        await command_positions(ctx, args.slave_id, make_positions(0, args.thumb_scale), args.dry_run)

        while True:
            now = time.monotonic()
            if args.duration > 0 and now - started_at >= args.duration:
                print("Duration reached; opening hand and exiting.")
                await command_positions(ctx, args.slave_id, make_positions(0, args.thumb_scale), args.dry_run)
                return 0

            metric, detail = await get_touch_metric(ctx, args.slave_id, hw_type)

            if state == "open_wait":
                close_value = 0
                release_started = None

                if now - last_open_command >= args.open_repeat:
                    await command_positions(ctx, args.slave_id, make_positions(0, args.thumb_scale), args.dry_run)
                    last_open_command = now

                if metric >= args.start_threshold:
                    state = "closing"
                    close_value = 0

            elif state == "closing":
                if metric >= args.stop_threshold or close_value >= args.max_close:
                    state = "hold"
                    close_value = min(close_value, args.max_close)
                    await command_positions(ctx, args.slave_id, make_positions(close_value, args.thumb_scale), args.dry_run)
                else:
                    close_value = min(args.max_close, close_value + args.step)
                    await command_positions(ctx, args.slave_id, make_positions(close_value, args.thumb_scale), args.dry_run)

                if metric < args.release_threshold:
                    if release_started is None:
                        release_started = now
                    elif now - release_started >= args.release_seconds:
                        state = "open_wait"
                        close_value = 0
                        await command_positions(ctx, args.slave_id, make_positions(0, args.thumb_scale), args.dry_run)
                else:
                    release_started = None

            elif state == "hold":
                # Hold current command while contact remains.
                if metric < args.release_threshold:
                    if release_started is None:
                        release_started = now
                    elif now - release_started >= args.release_seconds:
                        state = "open_wait"
                        close_value = 0
                        await command_positions(ctx, args.slave_id, make_positions(0, args.thumb_scale), args.dry_run)
                else:
                    release_started = None

            positions = None
            if tick % max(1, int(1.0 / args.period)) == 0:
                positions = await maybe_get_motor_positions(ctx, args.slave_id)

            if (not args.quiet) or tick % max(1, int(1.0 / args.period)) == 0:
                pos_str = f" motor={positions}" if positions is not None else ""
                print(f"{state:10s} touch={metric:7.2f} close_cmd={close_value:4d}{pos_str} | {detail}")

            tick += 1
            await asyncio.sleep(args.period)

    except PermissionError:
        print(f"ERROR: permission denied opening {args.port}", file=sys.stderr)
        print("Run your permission script first:", file=sys.stderr)
        print("  ~/bin/g1_fix_serial_permissions.sh", file=sys.stderr)
        return 13
    except KeyboardInterrupt:
        print("\nInterrupted; opening hand before exit.")
        if ctx is not None:
            try:
                await command_positions(ctx, args.slave_id, make_positions(0, args.thumb_scale), args.dry_run)
            except Exception:
                pass
        return 130
    finally:
        if ctx is not None:
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
