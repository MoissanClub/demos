#!/usr/bin/env python3
"""Capture a read-only G1/PC diagnostic snapshot for pre/post reboot comparison."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import socket
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path


def run(command: list[str], timeout: int = 20) -> dict:
    try:
        result = subprocess.run(
            command, text=True, capture_output=True, timeout=timeout, check=False
        )
        return {
            "command": command,
            "returncode": result.returncode,
            "stdout": result.stdout,
            "stderr": result.stderr,
        }
    except Exception as exc:
        return {"command": command, "error": f"{type(exc).__name__}: {exc}"}


def service_list(interface: str) -> dict:
    sdk_path = os.environ.get(
        "UNITREE_SDK2_PYTHON", os.path.expanduser("~/unitree_sdk2_python")
    )
    if sdk_path not in sys.path:
        sys.path.insert(0, sdk_path)
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient

        ChannelFactoryInitialize(0, interface)
        client = RobotStateClient()
        client.SetTimeout(3.0)
        client.Init()
        code, services = client.ServiceList()
        return {
            "code": code,
            "services": None
            if services is None
            else [
                {"name": item.name, "status": item.status, "protect": item.protect}
                for item in services
            ],
        }
    except Exception as exc:
        return {"error": f"{type(exc).__name__}: {exc}"}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--network-interface", default="eth0")
    parser.add_argument("--label", choices=("pre_reboot", "post_reboot"), required=True)
    parser.add_argument("--output-root", type=Path, default=Path("telemetry/diagnostics"))
    args = parser.parse_args()

    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output = args.output_root / f"g1_{args.label}_{timestamp}"
    output.mkdir(parents=True, exist_ok=False)
    python = sys.executable

    host = {
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "label": args.label,
        "hostname": socket.gethostname(),
        "platform": platform.platform(),
        "python": sys.version,
        "environment": {
            key: os.environ.get(key)
            for key in ("CYCLONEDDS_URI", "UNITREE_SDK2_PYTHON")
        },
        "commands": {
            "ip_address": run(["ip", "-details", "address", "show", args.network_interface]),
            "ip_route": run(["ip", "route", "show"]),
            "ip_neigh": run(["ip", "neigh", "show", "dev", args.network_interface]),
            "multicast": run(["ip", "maddress", "show", "dev", args.network_interface]),
            "udp_sockets": run(["ss", "-uapn"]),
            "processes": run(["ps", "-eo", "pid,lstart,stat,cmd"]),
            "kernel_tail": run(["dmesg", "--ctime", "--level=err,warn"], timeout=10),
        },
    }
    (output / "host.json").write_text(json.dumps(host, indent=2) + "\n")

    services = service_list(args.network_interface)
    (output / "robot_service_list.json").write_text(json.dumps(services, indent=2) + "\n")

    telemetry_path = output / "lowstate_5s.jsonl"
    telemetry = run(
        [
            python,
            "telemetry_probe.py",
            "--unitree",
            "--network-interface",
            args.network_interface,
            "--duration",
            "5",
            "--output",
            str(telemetry_path),
        ],
        timeout=15,
    )
    (output / "telemetry_probe_result.json").write_text(json.dumps(telemetry, indent=2) + "\n")

    preflight_path = output / "preflight.jsonl"
    preflight = run(
        [
            python,
            "g1_standalone_arm_sequence.py",
            "--probe-preflight",
            "--network-interface",
            args.network_interface,
            "--output",
            str(preflight_path),
        ],
        timeout=40,
    )
    (output / "preflight_result.json").write_text(json.dumps(preflight, indent=2) + "\n")

    checksums = []
    for path in sorted(output.iterdir()):
        if path.is_file():
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            checksums.append(f"{digest}  {path.name}")
    (output / "SHA256SUMS").write_text("\n".join(checksums) + "\n")
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
