#!/usr/bin/env python3
"""Read-only BrainCo hand and Unitree G1 telemetry discovery probe."""

import argparse
import asyncio
import dataclasses
import json
import os
import statistics
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, Optional, Sequence


DEFAULT_RIGHT_PORT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA1LW3T-if01-port0"
DEFAULT_LEFT_PORT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA1LW3T-if02-port0"
G1_JOINT_NAMES = [
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee",
    "left_ankle_pitch", "left_ankle_roll", "right_hip_pitch", "right_hip_roll",
    "right_hip_yaw", "right_knee", "right_ankle_pitch", "right_ankle_roll",
    "waist_yaw", "waist_roll", "waist_pitch", "left_shoulder_pitch",
    "left_shoulder_roll", "left_shoulder_yaw", "left_elbow", "left_wrist_roll",
    "left_wrist_pitch", "left_wrist_yaw", "right_shoulder_pitch",
    "right_shoulder_roll", "right_shoulder_yaw", "right_elbow",
    "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
]


def json_value(value: Any, _seen: Optional[set] = None, _depth: int = 0) -> Any:
    """Convert SDK objects to JSON values without following recursive fields."""
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, os.PathLike):
        return os.fspath(value)
    # BrainCo's Rust enums expose every possible enum member as an attribute.
    # Serializing dir(enum) therefore expands a tiny status into megabytes.
    try:
        enum_number = getattr(value, "int_value")
        enum_number = enum_number() if callable(enum_number) else enum_number
        if isinstance(enum_number, int) and not isinstance(enum_number, bool):
            return {"name": str(value), "int_value": enum_number}
    except Exception:
        pass
    if _depth >= 20:
        return repr(value)
    if _seen is None:
        _seen = set()
    identity = id(value)
    if identity in _seen:
        return "<recursive-reference>"
    _seen.add(identity)
    try:
        return _json_container_or_object(value, _seen, _depth)
    finally:
        _seen.remove(identity)


def _json_container_or_object(value: Any, seen: set, depth: int) -> Any:
    if isinstance(value, dict):
        return {str(key): json_value(item, seen, depth + 1) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_value(item, seen, depth + 1) for item in value]
    if dataclasses.is_dataclass(value):
        return {
            field.name: json_value(getattr(value, field.name), seen, depth + 1)
            for field in dataclasses.fields(value)
        }
    # Rust/Python SDK objects often expose enum types and class metadata through
    # dir(). Prefer actual instance attributes when the binding provides them.
    instance_fields = getattr(value, "__dict__", None)
    if isinstance(instance_fields, dict) and instance_fields:
        return {
            str(name): json_value(item, seen, depth + 1)
            for name, item in instance_fields.items()
            if not str(name).startswith("_")
        }
    fields: Dict[str, Any] = {}
    for name in dir(value):
        if name.startswith("_"):
            continue
        try:
            item = getattr(value, name)
        except Exception:
            continue
        if not callable(item):
            fields[name] = json_value(item, seen, depth + 1)
    return fields or repr(value)


class JsonlWriter:
    def __init__(self, path: Path) -> None:
        self.path = path
        self._file = path.open("w", encoding="utf-8")
        self._lock = threading.Lock()

    def write(self, stream: str, data: Any, **metadata: Any) -> None:
        record = {
            "timestamp_monotonic_ns": time.monotonic_ns(),
            "stream": stream,
            "data": json_value(data),
            **metadata,
        }
        line = json.dumps(record, separators=(",", ":"), ensure_ascii=False)
        with self._lock:
            self._file.write(line + "\n")
            self._file.flush()

    def close(self) -> None:
        self._file.close()


class StreamStats:
    def __init__(self) -> None:
        self.timestamps_ns = []
        self.latencies_ms = []
        self.errors = 0

    def success(self, timestamp_ns: int, latency_ms: Optional[float] = None) -> None:
        self.timestamps_ns.append(timestamp_ns)
        if latency_ms is not None:
            self.latencies_ms.append(latency_ms)

    def summary(self) -> Dict[str, Any]:
        elapsed = 0.0
        if len(self.timestamps_ns) > 1:
            elapsed = (self.timestamps_ns[-1] - self.timestamps_ns[0]) / 1e9
        result = {
            "samples": len(self.timestamps_ns),
            "errors": self.errors,
            "observed_rate_hz": ((len(self.timestamps_ns) - 1) / elapsed) if elapsed > 0 else None,
        }
        if self.latencies_ms:
            ordered = sorted(self.latencies_ms)
            result["latency_ms"] = {
                "min": ordered[0],
                "median": statistics.median(ordered),
                "p95": ordered[min(len(ordered) - 1, int(0.95 * len(ordered)))],
                "max": ordered[-1],
            }
        return result


def unitree_lowstate_record(msg: Any) -> Dict[str, Any]:
    motors = []
    for index, motor in enumerate(msg.motor_state):
        motors.append({
            "index": index,
            "joint_name": G1_JOINT_NAMES[index] if index < len(G1_JOINT_NAMES) else f"reserved_{index}",
            "mode_raw": motor.mode,
            "position_rad": motor.q,
            "velocity_rad_s": motor.dq,
            "acceleration_raw": motor.ddq,
            "estimated_torque_nm": motor.tau_est,
            "temperature_raw": list(motor.temperature),
            "voltage_raw": motor.vol,
            "motor_state_raw": motor.motorstate,
        })
    return {
        "version_raw": list(msg.version),
        "mode_pr_raw": msg.mode_pr,
        "mode_machine_raw": msg.mode_machine,
        "tick_raw": msg.tick,
        "imu": json_value(msg.imu_state),
        "motors": motors,
    }


async def probe_brainco(args: argparse.Namespace, writer: JsonlWriter, stats: Dict[str, StreamStats]) -> None:
    for candidate in (os.environ.get("BRAINCO_SDK_PYTHON"), os.path.expanduser("~/brainco-hand-sdk/python")):
        if candidate and os.path.isdir(candidate) and candidate not in sys.path:
            sys.path.insert(0, candidate)
    from common_imports import check_sdk, int_to_baudrate, sdk

    check_sdk()
    ctx = await sdk.modbus_open(args.port, int_to_baudrate(args.baud))
    try:
        info = await asyncio.wait_for(ctx.get_device_info(args.slave_id), args.timeout)
        writer.write("brainco.device_info", info)
        hardware_name = str(info.hardware_type).lower()
        array_pressure = "arraypressure" in hardware_name or "array_pressure" in hardware_name
        if args.enable_touch_sensors:
            await asyncio.wait_for(ctx.touch_sensor_setup(args.slave_id, 0x1F), args.timeout)
            await asyncio.sleep(0.5)

        deadline = time.monotonic() + args.duration
        next_sample = time.monotonic()
        while time.monotonic() < deadline:
            for stream, operation in (
                ("brainco.touch", ctx.get_array_pressure_touch_data(args.slave_id) if array_pressure else ctx.get_touch_sensor_status(args.slave_id)),
                ("brainco.motor", ctx.get_motor_status(args.slave_id)),
            ):
                started = time.monotonic_ns()
                try:
                    data = await asyncio.wait_for(operation, args.timeout)
                    received = time.monotonic_ns()
                    latency = (received - started) / 1e6
                    stats[stream].success(received, latency)
                    writer.write(stream, data, read_latency_ms=latency)
                except Exception as exc:
                    stats[stream].errors += 1
                    writer.write(stream + ".error", {"type": type(exc).__name__, "message": str(exc)})
            next_sample += args.period
            await asyncio.sleep(max(0.0, next_sample - time.monotonic()))
    finally:
        sdk.modbus_close(ctx)


def start_unitree(args: argparse.Namespace, writer: JsonlWriter, stats: StreamStats) -> Any:
    sdk_path = os.environ.get("UNITREE_SDK2_PYTHON", os.path.expanduser("~/unitree_sdk2_python"))
    if os.path.isdir(sdk_path) and sdk_path not in sys.path:
        sys.path.insert(0, sdk_path)
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

    ChannelFactoryInitialize(0, args.network_interface) if args.network_interface else ChannelFactoryInitialize(0)
    subscriber = ChannelSubscriber(args.unitree_topic, LowState_)

    def receive(msg: Any) -> None:
        timestamp = time.monotonic_ns()
        stats.success(timestamp)
        writer.write("unitree.lowstate", unitree_lowstate_record(msg), topic=args.unitree_topic)

    subscriber.Init(receive, 10)
    return subscriber


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    sources = parser.add_argument_group("sources")
    sources.add_argument("--brainco", action="store_true", help="Probe BrainCo tactile and motor telemetry.")
    sources.add_argument("--unitree", action="store_true", help="Subscribe to Unitree G1 low state.")
    hand = parser.add_mutually_exclusive_group()
    hand.add_argument("--left", action="store_true")
    hand.add_argument("--right", action="store_true")
    parser.add_argument("--port")
    parser.add_argument("--slave-id", type=lambda value: int(value, 0))
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--period", type=float, default=0.1, help="BrainCo paired-read period in seconds.")
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--enable-touch-sensors", action="store_true", help="Opt in to touch_sensor_setup (device write).")
    parser.add_argument("--network-interface")
    parser.add_argument("--unitree-topic", default="rt/lowstate")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    if not args.brainco and not args.unitree:
        parser.error("select --brainco, --unitree, or both")
    if args.duration <= 0 or args.period <= 0 or args.timeout <= 0:
        parser.error("duration, period, and timeout must be positive")
    if not args.left and not args.right:
        args.right = True
    args.port = args.port or (DEFAULT_LEFT_PORT if args.left else DEFAULT_RIGHT_PORT)
    args.slave_id = args.slave_id if args.slave_id is not None else (0x7E if args.left else 0x7F)
    return args


async def run(args: argparse.Namespace) -> int:
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output = args.output or Path("telemetry") / f"probe_{timestamp}.jsonl"
    output.parent.mkdir(parents=True, exist_ok=True)
    writer = JsonlWriter(output)
    stats = {name: StreamStats() for name in ("brainco.touch", "brainco.motor", "unitree.lowstate")}
    started_utc = datetime.now(timezone.utc).isoformat()
    subscriber = None
    try:
        writer.write("probe.metadata", {"started_at_utc": started_utc, "arguments": vars(args)})
        if args.unitree:
            subscriber = start_unitree(args, writer, stats["unitree.lowstate"])
        if args.brainco:
            await probe_brainco(args, writer, stats)
        else:
            await asyncio.sleep(args.duration)
        summary = {name: value.summary() for name, value in stats.items() if getattr(args, name.split('.')[0], False)}
        writer.write("probe.summary", summary, finished_at_utc=datetime.now(timezone.utc).isoformat())
        print(json.dumps({"output": str(output), "streams": summary}, indent=2))
        return 0
    finally:
        del subscriber
        writer.close()


if __name__ == "__main__":
    raise SystemExit(asyncio.run(run(parse_args())))
