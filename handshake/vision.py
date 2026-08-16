"""Simple hand-presence detection and a debounced vision state machine."""

from dataclasses import dataclass
from enum import Enum
import threading
import time
from typing import Any, Optional, Tuple, Union


class VisionState(str, Enum):
    NO_HAND = "no_hand"
    HAND_PRESENT = "hand_present"


@dataclass(frozen=True)
class VisionConfig:
    present_seconds: float = 0.25
    absent_seconds: float = 0.75


class VisionStateMachine:
    """Debounce raw detections so a single bad frame cannot move the arm."""

    def __init__(self, config: VisionConfig) -> None:
        self.config = config
        self.state = VisionState.NO_HAND
        self._candidate: Optional[VisionState] = None
        self._candidate_since: Optional[float] = None

    def update(self, detected: bool, now: float) -> bool:
        candidate = VisionState.HAND_PRESENT if detected else VisionState.NO_HAND
        if candidate == self.state:
            self._candidate = None
            self._candidate_since = None
            return False
        if candidate != self._candidate:
            self._candidate = candidate
            self._candidate_since = now
            return False
        required = self.config.present_seconds if detected else self.config.absent_seconds
        if self._candidate_since is not None and now - self._candidate_since >= required:
            self.state = candidate
            self._candidate = None
            self._candidate_since = None
            return True
        return False


def parse_camera_source(value: str) -> Union[int, str]:
    """Treat an integer as a camera index; otherwise retain a path/stream URL."""
    try:
        return int(value)
    except ValueError:
        return value


class HandPresenceDetector:
    """Background detector exposing only presence and an area score.

    This simple first version finds sufficiently large skin-colored regions in
    a central region of interest. It does not estimate landmarks or coordinates.
    """

    def __init__(self, source: Union[int, str], config: VisionConfig,
                 min_area_ratio: float = 0.005, roi_scale: float = 0.9,
                 fps: float = 10.0, network_interface: Optional[str] = None,
                 initialize_channel: bool = False,
                 realsense_serial: Optional[str] = None) -> None:
        self.source = source
        self.machine = VisionStateMachine(config)
        self.min_area_ratio = min_area_ratio
        self.roi_scale = roi_scale
        self.fps = fps
        self.network_interface = network_interface
        self.initialize_channel = initialize_channel
        self.realsense_serial = realsense_serial
        self._lock = threading.Lock()
        self._state = VisionState.NO_HAND
        self._score = 0.0
        self._error: Optional[str] = None
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._capture: Any = None
        self._unitree_client: Any = None
        self._realsense_pipeline: Any = None
        self._background_gray: Any = None
        self._background_frames = 0
        self._warmup_frames = 0

    def start(self, timeout: float = 10.0) -> None:
        self._thread = threading.Thread(target=self._run, name="hand_vision", daemon=True)
        self._thread.start()
        if not self._ready.wait(timeout):
            raise RuntimeError("camera initialization timed out")
        if self.error:
            self.close()
            raise RuntimeError(self.error)

    def snapshot(self) -> Tuple[VisionState, float]:
        with self._lock:
            return self._state, self._score

    def _run(self) -> None:
        try:
            import cv2

            if self.source == "realsense":
                import pyrealsense2 as rs

                pipeline = rs.pipeline()
                config = rs.config()
                if self.realsense_serial:
                    config.enable_device(self.realsense_serial)
                config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                pipeline.start(config)
                self._realsense_pipeline = pipeline
            elif self.source == "unitree":
                if self.initialize_channel:
                    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
                    if self.network_interface:
                        ChannelFactoryInitialize(0, self.network_interface)
                    else:
                        ChannelFactoryInitialize(0)
                from unitree_sdk2py.go2.video.video_client import VideoClient
                self._unitree_client = VideoClient()
                self._unitree_client.SetTimeout(3.0)
                self._unitree_client.Init()
            else:
                capture = cv2.VideoCapture(self.source)
                self._capture = capture
                if not capture.isOpened():
                    raise RuntimeError(f"cannot open camera source {self.source!r}")
            period = 1.0 / self.fps
            while not self._stop.is_set():
                started = time.monotonic()
                if self._realsense_pipeline is not None:
                    import numpy as np
                    frames = self._realsense_pipeline.wait_for_frames(3000)
                    color_frame = frames.get_color_frame()
                    frame = np.asanyarray(color_frame.get_data()) if color_frame else None
                    ok = frame is not None
                elif self._unitree_client is not None:
                    import numpy as np
                    code, data = self._unitree_client.GetImageSample()
                    frame = cv2.imdecode(
                        np.frombuffer(bytes(data), dtype=np.uint8), cv2.IMREAD_COLOR
                    ) if code == 0 else None
                    ok = code == 0 and frame is not None
                else:
                    ok, frame = self._capture.read()
                if not ok or frame is None:
                    raise RuntimeError(f"camera source {self.source!r} stopped producing frames")
                detected, score = self._detect(frame, cv2)
                self.machine.update(detected, time.monotonic())
                with self._lock:
                    self._state = self.machine.state
                    self._score = score
                # Initialization is complete only after camera exposure has
                # settled and the empty-scene reference has been collected.
                if self._background_frames >= 8:
                    self._ready.set()
                self._stop.wait(max(0.0, period - (time.monotonic() - started)))
        except Exception as exc:
            with self._lock:
                self._error = f"vision detector failed: {exc}"
            self._ready.set()
        finally:
            if self._capture is not None:
                self._capture.release()
            if self._realsense_pipeline is not None:
                self._realsense_pipeline.stop()
                self._realsense_pipeline = None
            if self._unitree_client is not None:
                from .unitree_cleanup import close_rpc_client
                close_rpc_client(self._unitree_client)
                self._unitree_client = None

    def _detect(self, frame: Any, cv2: Any) -> Tuple[bool, float]:
        height, width = frame.shape[:2]
        roi_width = max(1, int(width * self.roi_scale))
        roi_height = max(1, int(height * self.roi_scale))
        left = (width - roi_width) // 2
        top = (height - roi_height) // 2
        roi = frame[top:top + roi_height, left:left + roi_width]
        blurred = cv2.GaussianBlur(roi, (7, 7), 0)
        gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
        # RealSense auto-exposure changes strongly during its first frames.
        # Discard that settling period and normalize later global illumination.
        if self._warmup_frames < 30:
            self._warmup_frames += 1
            return False, 0.0
        gray = cv2.equalizeHist(gray)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        motion_mask = None
        if self._background_gray is None:
            self._background_gray = gray.astype("float")
            self._background_frames = 1
        elif self._background_frames < 8:
            cv2.accumulateWeighted(gray, self._background_gray, 0.2)
            self._background_frames += 1
        else:
            background = cv2.convertScaleAbs(self._background_gray)
            difference = cv2.absdiff(gray, background)
            motion_mask = cv2.threshold(difference, 28, 255, cv2.THRESH_BINARY)[1]
            motion_mask = cv2.morphologyEx(motion_mask, cv2.MORPH_OPEN, kernel)
            motion_mask = cv2.morphologyEx(motion_mask, cv2.MORPH_CLOSE, kernel)
        motion_largest = 0.0
        if motion_mask is not None:
            motion_contours, _ = cv2.findContours(
                motion_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            motion_largest = max(
                (cv2.contourArea(item) for item in motion_contours), default=0.0
            )

        score = float(motion_largest) / float(roi_width * roi_height)
        return score >= self.min_area_ratio, score

    @property
    def error(self) -> Optional[str]:
        with self._lock:
            return self._error

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            # Unitree image RPC timeout is 3 seconds; allow it to return and
            # let the reader thread release its own camera/RPC resources. This
            # avoids closing a V4L2 descriptor during an active read().
            self._thread.join(timeout=5.0)
