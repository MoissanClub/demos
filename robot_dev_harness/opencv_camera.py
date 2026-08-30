"""Direct V4L2/OpenCV camera recording with per-frame host timestamps."""
from __future__ import annotations

import json
import re
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

from robot_dev_harness.run_artifacts import RunArtifacts, SCHEMA_VERSION, utc_text


_VIDEO_DEVICE = re.compile(r"^/dev/video[0-9]+$")


class OpenCVMjpegCamera:
    """Record MJPEG AVI and timestamp each acquired frame on the host clock."""

    def __init__(
        self,
        run: RunArtifacts,
        device: str = "/dev/video6",
        width: int = 640,
        height: int = 480,
        fps: float = 30.0,
        utc_now: Callable[[], datetime] = lambda: datetime.now(timezone.utc),
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        if not _VIDEO_DEVICE.fullmatch(device):
            raise ValueError("camera device must be an explicit /dev/videoN path")
        if not 1 <= width <= 4096 or not 1 <= height <= 2160:
            raise ValueError("camera dimensions are outside the supported harness range")
        if not 1.0 <= fps <= 120.0:
            raise ValueError("camera frame rate must be between 1 and 120 Hz")
        self.run = run
        self.device = device
        self.width = width
        self.height = height
        self.requested_fps = fps
        self.utc_now = utc_now
        self.monotonic_ns = monotonic_ns
        self.video_path = run.path(
            f"video/camera_opencv_{Path(device).name}_{run.run_id}.avi"
        )
        self._capture: Any = None
        self._writer: Any = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._first_frame = threading.Event()
        self._timestamps: List[Dict[str, Any]] = []
        self._error: Optional[str] = None
        self._started_utc: Optional[datetime] = None
        self._started_ns: Optional[int] = None
        self._actual_fps: Optional[float] = None
        self._actual_width: Optional[int] = None
        self._actual_height: Optional[int] = None

    @property
    def active(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    @property
    def error(self) -> Optional[str]:
        return self._error

    def start(self, startup_timeout_seconds: float = 5.0) -> None:
        if self._thread is not None:
            raise RuntimeError("camera capture was already started")
        import cv2

        capture = cv2.VideoCapture(int(Path(self.device).name[5:]), cv2.CAP_V4L2)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(f"cannot open camera {self.device}")
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_FPS, self.requested_fps)
        self._actual_width = int(round(capture.get(cv2.CAP_PROP_FRAME_WIDTH)))
        self._actual_height = int(round(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        self._actual_fps = float(capture.get(cv2.CAP_PROP_FPS))
        if self._actual_width <= 0 or self._actual_height <= 0 or self._actual_fps <= 0:
            capture.release()
            raise RuntimeError("camera returned an invalid negotiated format")
        writer = cv2.VideoWriter(
            str(self.video_path), cv2.VideoWriter_fourcc(*"MJPG"),
            self._actual_fps, (self._actual_width, self._actual_height),
        )
        if not writer.isOpened():
            capture.release()
            writer.release()
            raise RuntimeError(f"cannot create camera video {self.video_path}")
        self._capture, self._writer = capture, writer
        self._started_utc = self.utc_now()
        self._started_ns = self.monotonic_ns()
        self.run.register_device("camera", {
            "backend": "opencv_v4l2",
            "device": self.device,
            "codec": "MJPG",
            "container": "AVI",
            "requested_format": {
                "width": self.width, "height": self.height, "fps": self.requested_fps,
            },
            "negotiated_format": {
                "width": self._actual_width, "height": self._actual_height,
                "fps": self._actual_fps,
            },
            "timestamp_method": "host_immediately_after_frame_read",
        })
        self._thread = threading.Thread(
            target=self._capture_loop, name=f"camera_{Path(self.device).name}", daemon=True,
        )
        self._thread.start()
        if not self._first_frame.wait(startup_timeout_seconds):
            self._stop.set()
            self._thread.join(timeout=2.0)
            self._release()
            raise RuntimeError(self._error or "camera produced no frame before startup timeout")
        if self._error:
            self._stop.set()
            self._thread.join(timeout=2.0)
            self._release()
            raise RuntimeError(self._error)

    def stop(self, timeout_seconds: float = 5.0) -> Dict[str, Any]:
        if self._thread is None or self._started_ns is None or self._started_utc is None:
            raise RuntimeError("camera capture is not active")
        self._stop.set()
        self._thread.join(timeout=timeout_seconds)
        if self._thread.is_alive():
            self._release()
            self._thread.join(timeout=1.0)
        self._release()
        if self._thread.is_alive():
            raise RuntimeError("camera thread did not stop")
        if self._timestamps:
            self._write_frame_timestamps()
        if self._error:
            raise RuntimeError(self._error)
        if not self._timestamps or not self.video_path.is_file() or self.video_path.stat().st_size == 0:
            raise RuntimeError("camera capture produced no complete video frames")
        stopped_utc, stopped_ns = self.utc_now(), self.monotonic_ns()
        return {
            "path": str(self.video_path.relative_to(self.run.directory.resolve())),
            "frame_count": len(self._timestamps),
            "size_bytes": self.video_path.stat().st_size,
            "started_at_utc": utc_text(self._started_utc),
            "started_monotonic_ns": self._started_ns,
            "stopped_at_utc": utc_text(stopped_utc),
            "stopped_monotonic_ns": stopped_ns,
            "negotiated_fps": self._actual_fps,
            "measured_fps": self._measured_fps(),
        }

    def _capture_loop(self) -> None:
        try:
            while not self._stop.is_set():
                ok, frame = self._capture.read()
                timestamp_ns = self.monotonic_ns()
                timestamp_utc = self.utc_now()
                if not ok:
                    self._error = "camera frame read failed"
                    return
                self._writer.write(frame)
                self._timestamps.append({
                    "timestamp_utc": utc_text(timestamp_utc),
                    "monotonic_ns": timestamp_ns,
                })
                self._first_frame.set()
        except BaseException as exc:
            self._error = f"{type(exc).__name__}: {exc}"
        finally:
            self._first_frame.set()

    def _release(self) -> None:
        if self._capture is not None:
            self._capture.release()
            self._capture = None
        if self._writer is not None:
            self._writer.release()
            self._writer = None

    def _measured_fps(self) -> Optional[float]:
        if len(self._timestamps) < 2:
            return None
        duration = (
            self._timestamps[-1]["monotonic_ns"] - self._timestamps[0]["monotonic_ns"]
        ) / 1e9
        return (len(self._timestamps) - 1) / duration if duration > 0 else None

    def _write_frame_timestamps(self) -> None:
        path = self.run.path("video/frame_timestamps.jsonl")
        cadence_ns = int(1e9 / self._actual_fps) if self._actual_fps else None
        with path.open("x", encoding="utf-8") as output:
            for frame_index, timestamp in enumerate(self._timestamps):
                row = {
                    "schema_version": SCHEMA_VERSION,
                    "run_id": self.run.run_id,
                    "frame_index": frame_index,
                    **timestamp,
                    "timestamp_method": "measured_host_after_frame_read",
                    "estimated_uncertainty_ns": cadence_ns,
                }
                output.write(json.dumps(row, separators=(",", ":")) + "\n")
