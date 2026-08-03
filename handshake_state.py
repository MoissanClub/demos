"""Hardware-independent state machine for the tactile handshake demo."""

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class HandshakeState(str, Enum):
    OPEN_WAIT = "open_wait"
    CLOSING = "closing"
    HOLD = "hold"


@dataclass(frozen=True)
class HandshakeConfig:
    start_threshold: float
    stop_threshold: float
    release_threshold: float
    release_seconds: float
    hold_duration: float
    max_close: int
    step: int


@dataclass(frozen=True)
class HandshakeDecision:
    state: HandshakeState
    close_value: int
    command_close: Optional[int] = None
    trigger_arm: bool = False
    release_arm: bool = False
    entered_hold: bool = False
    event: Optional[str] = None


class HandshakeStateMachine:
    """Advance handshake behavior from timestamped tactile samples."""

    def __init__(self, config: HandshakeConfig) -> None:
        self.config = config
        self.state = HandshakeState.OPEN_WAIT
        self.close_value = 0
        self.ready_for_contact = True
        self.release_started: Optional[float] = None
        self.rearm_started: Optional[float] = None
        self.hold_started: Optional[float] = None

    def update(self, metric: float, now: float) -> HandshakeDecision:
        if self.state == HandshakeState.OPEN_WAIT:
            return self._update_open_wait(metric, now)
        if self.state == HandshakeState.CLOSING:
            return self._update_closing(metric, now)
        return self._update_hold(metric, now)

    def _update_open_wait(self, metric: float, now: float) -> HandshakeDecision:
        self.close_value = 0
        self.hold_started = None

        if not self.ready_for_contact:
            if metric < self.config.release_threshold:
                if self.rearm_started is None:
                    self.rearm_started = now
                elif now - self.rearm_started >= self.config.release_seconds:
                    self.ready_for_contact = True
                    self.rearm_started = None
            else:
                self.rearm_started = None

        if self.ready_for_contact and metric >= self.config.start_threshold:
            self.state = HandshakeState.CLOSING
            self.release_started = None
            self.rearm_started = None
            self.ready_for_contact = False
            return self._decision(trigger_arm=True, event="contact_started")

        return self._decision()

    def _update_closing(self, metric: float, now: float) -> HandshakeDecision:
        event = None
        if metric >= self.config.stop_threshold or self.close_value >= self.config.max_close:
            self.state = HandshakeState.HOLD
            self.hold_started = now
            self.close_value = min(self.close_value, self.config.max_close)
            event = "pressure_limit_reached" if metric >= self.config.stop_threshold else "max_close_reached"
        else:
            self.close_value = min(self.config.max_close, self.close_value + self.config.step)

        if self._release_confirmed(metric, now):
            return self._open_after_handshake("release_during_closing")

        return self._decision(
            command_close=self.close_value,
            entered_hold=self.state == HandshakeState.HOLD,
            event=event,
        )

    def _update_hold(self, metric: float, now: float) -> HandshakeDecision:
        if self.hold_started is not None and now - self.hold_started >= self.config.hold_duration:
            return self._open_after_handshake("hold_timeout")

        if self._release_confirmed(metric, now):
            return self._open_after_handshake("release_during_hold")

        return self._decision()

    def _release_confirmed(self, metric: float, now: float) -> bool:
        if metric < self.config.release_threshold:
            if self.release_started is None:
                self.release_started = now
            return now - self.release_started >= self.config.release_seconds

        self.release_started = None
        return False

    def _open_after_handshake(self, event: str) -> HandshakeDecision:
        self.state = HandshakeState.OPEN_WAIT
        self.close_value = 0
        self.hold_started = None
        self.release_started = None
        self.rearm_started = None
        return self._decision(command_close=0, release_arm=True, event=event)

    def _decision(
        self,
        *,
        command_close: Optional[int] = None,
        trigger_arm: bool = False,
        release_arm: bool = False,
        entered_hold: bool = False,
        event: Optional[str] = None,
    ) -> HandshakeDecision:
        return HandshakeDecision(
            state=self.state,
            close_value=self.close_value,
            command_close=command_close,
            trigger_arm=trigger_arm,
            release_arm=release_arm,
            entered_hold=entered_hold,
            event=event,
        )
