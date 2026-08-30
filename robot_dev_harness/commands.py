"""Evidence-first wrapper for separately reviewed robot command transports."""
from __future__ import annotations

from typing import Any, Callable, Mapping

from robot_dev_harness.session import EvidenceSession


class EvidenceBackedCommandTransport:
    """Record a command successfully before invoking its transport once.

    This wrapper adds no retry, timing, control, or safety policy. The supplied
    transport remains project-specific and must be reviewed independently.
    """

    def __init__(
        self,
        session: EvidenceSession,
        source: str,
        transport: Callable[[Mapping[str, Any]], Any],
    ) -> None:
        self.session = session
        self.source = source
        self.transport = transport

    def send(self, command: Mapping[str, Any]) -> Any:
        # Use the session's single pre-send snapshot for transport too, avoiding
        # a second mapping copy in every command tick.
        payload = self.session.command(self.source, command)
        try:
            return self.transport(payload)
        except BaseException as exc:
            self.session.event("command_transport_failed", {
                "source": self.source,
                "error": f"{type(exc).__name__}: {exc}",
            })
            raise
