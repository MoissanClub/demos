"""Compatibility cleanup for Unitree SDK2 Python RPC clients."""

from typing import Any


def close_rpc_client(client: Any) -> None:
    """Close SDK2 RPC DDS channels despite the SDK lacking a public Close()."""
    if client is None:
        return
    stub = getattr(client, "_ClientBase__stub", None)
    if stub is None:
        return
    recv_channel = getattr(stub, "_ClientStub__recvChannel", None)
    send_channel = getattr(stub, "_ClientStub__sendChannel", None)
    if recv_channel is not None:
        recv_channel.CloseReader()
    if send_channel is not None:
        send_channel.CloseWriter()
