import unittest

from handshake.arm_policy import ArmPolicy
from handshake.state import HandshakeState
from handshake.vision import (
    HandPresenceDetector,
    VisionConfig,
    VisionState,
    VisionStateMachine,
    parse_camera_source,
)


class VisionStateMachineTests(unittest.TestCase):
    def test_presence_and_absence_are_debounced(self):
        machine = VisionStateMachine(VisionConfig(present_seconds=0.2, absent_seconds=0.5))
        self.assertFalse(machine.update(True, 1.0))
        self.assertFalse(machine.update(False, 1.1))
        self.assertFalse(machine.update(True, 2.0))
        self.assertTrue(machine.update(True, 2.2))
        self.assertEqual(machine.state, VisionState.HAND_PRESENT)
        self.assertFalse(machine.update(False, 3.0))
        self.assertTrue(machine.update(False, 3.5))
        self.assertEqual(machine.state, VisionState.NO_HAND)

    def test_camera_source_parsing(self):
        self.assertEqual(parse_camera_source("0"), 0)
        self.assertEqual(parse_camera_source("/dev/video2"), "/dev/video2")

    def test_object_entering_initial_scene_is_detected(self):
        import cv2
        import numpy as np

        detector = HandPresenceDetector(0, VisionConfig(), min_area_ratio=0.005)
        empty = np.full((240, 320, 3), 70, dtype=np.uint8)
        detector._warmup_frames = 30
        for _ in range(8):
            detector._detect(empty, cv2)
        entered = empty.copy()
        cv2.rectangle(entered, (120, 70), (200, 190), (255, 0, 0), thickness=-1)

        detected, score = detector._detect(entered, cv2)

        self.assertTrue(detected)
        self.assertGreater(score, 0.005)


class ArmPolicyTests(unittest.TestCase):
    def test_vision_raises_and_absence_lowers_while_idle(self):
        policy = ArmPolicy()
        self.assertTrue(policy.update(VisionState.NO_HAND, HandshakeState.OPEN_WAIT).lower_arm)
        self.assertTrue(policy.update(VisionState.HAND_PRESENT, HandshakeState.OPEN_WAIT).raise_arm)
        self.assertTrue(policy.update(VisionState.NO_HAND, HandshakeState.OPEN_WAIT).lower_arm)

    def test_active_handshake_prevents_lowering(self):
        policy = ArmPolicy()
        policy.update(VisionState.HAND_PRESENT, HandshakeState.OPEN_WAIT)
        decision = policy.update(VisionState.NO_HAND, HandshakeState.HOLD)
        self.assertFalse(decision.lower_arm)
        decision = policy.update(VisionState.NO_HAND, HandshakeState.OPEN_WAIT)
        self.assertTrue(decision.lower_arm)

    def test_lowers_one_second_after_release_even_if_vision_stays_present(self):
        policy = ArmPolicy(post_handshake_lower_delay=1.0)
        policy.update(VisionState.HAND_PRESENT, HandshakeState.OPEN_WAIT, now=0.0)
        policy.update(VisionState.HAND_PRESENT, HandshakeState.RELEASING, now=2.0)
        decision = policy.update(
            VisionState.HAND_PRESENT,
            HandshakeState.OPEN_WAIT,
            now=2.2,
            hand_open_complete=True,
        )
        self.assertFalse(decision.lower_arm)

        decision = policy.update(
            VisionState.HAND_PRESENT, HandshakeState.OPEN_WAIT, now=3.0
        )
        self.assertTrue(decision.lower_arm)

        decision = policy.update(
            VisionState.HAND_PRESENT, HandshakeState.OPEN_WAIT, now=3.1
        )
        self.assertFalse(decision.raise_arm)

    def test_no_hand_rearms_vision_after_forced_lower(self):
        policy = ArmPolicy(post_handshake_lower_delay=0.0)
        policy.update(VisionState.HAND_PRESENT, HandshakeState.OPEN_WAIT, now=0.0)
        policy.update(VisionState.HAND_PRESENT, HandshakeState.RELEASING, now=1.0)
        policy.update(
            VisionState.HAND_PRESENT,
            HandshakeState.OPEN_WAIT,
            now=1.1,
            hand_open_complete=True,
        )
        policy.update(VisionState.NO_HAND, HandshakeState.OPEN_WAIT, now=1.2)

        decision = policy.update(
            VisionState.HAND_PRESENT, HandshakeState.OPEN_WAIT, now=1.3
        )
        self.assertTrue(decision.raise_arm)


if __name__ == "__main__":
    unittest.main()
