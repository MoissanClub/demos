"""Coordination policy between independent tactile and vision states."""

from dataclasses import dataclass
from typing import Optional

from .state import HandshakeState
from .vision import VisionState


@dataclass(frozen=True)
class ArmDecision:
    raise_arm: bool = False
    lower_arm: bool = False


class ArmPolicy:
    """Coordinate invitation, safe post-handshake lowering, and vision rearming."""

    def __init__(self, post_handshake_lower_delay: float = 1.0) -> None:
        self._raised: Optional[bool] = None
        self._release_started: Optional[float] = None
        self._hand_open_complete = False
        self._suppress_vision = False
        self.post_handshake_lower_delay = post_handshake_lower_delay

    def update(
        self,
        vision: VisionState,
        handshake: HandshakeState,
        now: float = 0.0,
        hand_open_complete: bool = False,
    ) -> ArmDecision:
        if handshake == HandshakeState.RELEASING and self._release_started is None:
            self._release_started = now
            self._hand_open_complete = False
        if self._release_started is not None:
            self._hand_open_complete = self._hand_open_complete or hand_open_complete
            delay_elapsed = now - self._release_started >= self.post_handshake_lower_delay
            if self._hand_open_complete and delay_elapsed:
                self._release_started = None
                self._hand_open_complete = False
                # A stale hand_present result must not immediately raise the
                # arm again. A subsequent no_hand observation rearms vision.
                self._suppress_vision = vision == VisionState.HAND_PRESENT
                if self._raised is not False:
                    self._raised = False
                    return ArmDecision(lower_arm=True)
                return ArmDecision()
            # Hold the raised posture while opening and during the delay.
            if self._raised is not True:
                self._raised = True
                return ArmDecision(raise_arm=True)
            return ArmDecision()

        if self._suppress_vision and vision == VisionState.NO_HAND:
            self._suppress_vision = False
        vision_requests_raise = (
            not self._suppress_vision and vision == VisionState.HAND_PRESENT
        )
        should_raise = vision_requests_raise or handshake != HandshakeState.OPEN_WAIT
        if should_raise and self._raised is not True:
            self._raised = True
            return ArmDecision(raise_arm=True)
        if not should_raise and self._raised is not False:
            self._raised = False
            return ArmDecision(lower_arm=True)
        return ArmDecision()
