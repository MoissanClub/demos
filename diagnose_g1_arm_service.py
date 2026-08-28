#!/usr/bin/env python3
"""Read-only discovery checks for the Unitree G1 arm action RPC service."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

from handshake.unitree_cleanup import close_rpc_client


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--network-interface", default="eth0")
    parser.add_argument("--observe-seconds", type=float, default=5.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    sdk_path = os.environ.get(
        "UNITREE_SDK2_PYTHON", os.path.expanduser("~/unitree_sdk2_python")
    )
    if sdk_path not in sys.path:
        sys.path.insert(0, sdk_path)

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient
    from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

    ChannelFactoryInitialize(0, args.network_interface)
    observed = []
    subscriber = ChannelSubscriber("rt/arm/action/state", String_)
    subscriber.Init(lambda message: observed.append(message.data), 10)
    clients = []
    result = {
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "network_interface": args.network_interface,
    }
    try:
        state = RobotStateClient()
        state.SetTimeout(3.0)
        state.Init()
        clients.append(state)
        code, services = state.ServiceList()
        result["service_list"] = {
            "code": code,
            "services": None
            if services is None
            else [
                {"name": item.name, "status": item.status, "protect": item.protect}
                for item in services
            ],
        }

        for key, client_type in (("sport", LocoClient), ("arm", G1ArmActionClient)):
            client = client_type()
            client.SetTimeout(3.0)
            client.Init()
            clients.append(client)
            code, version = client.GetServerApiVersion()
            result[key + "_server_api"] = {
                "client_api_version": client.GetApiVersion(),
                "code": code,
                "server_api_version": version,
            }
        time.sleep(args.observe_seconds)
        result["arm_action_state"] = {
            "topic": "rt/arm/action/state",
            "observe_seconds": args.observe_seconds,
            "sample_count": len(observed),
            "samples": observed,
        }
    finally:
        subscriber.Close()
        for client in clients:
            close_rpc_client(client)

    payload = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload)
    print(payload, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
