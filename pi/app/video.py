"""Video: libcamera (IMX708) on the robot, or a synthetic pipe-interior renderer driven by live
telemetry in simulation. Frames go to WebRTC (aiortc), the MJPEG fallback and the recorder.

On the robot the H.264 stream comes from the Pi's hardware encoder (Picamera2 + H264Encoder);
the synthetic source exists so the whole stack can be exercised against the SIL simulator.
"""

from __future__ import annotations

import io
import logging
import threading
import time
from collections.abc import Callable
from fractions import Fraction
from typing import Any

import numpy as np
from PIL import Image

log = logging.getLogger("pipebot.video")


class SyntheticCamera:
    """Perspective view down a pipe: weld seams every metre, scale texture, lights, pan/tilt."""

    def __init__(self, w: int, h: int, telemetry: Callable[[], dict[str, Any]]) -> None:
        """w x h frames rendered from the current telemetry snapshot."""
        self.w, self.h = w, h
        self.telemetry = telemetry
        yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
        self.xx = (xx - w / 2) / (h / 2)
        self.yy = (yy - h / 2) / (h / 2)
        rng = np.random.default_rng(7)
        self.tex = rng.random((64, 256)).astype(np.float32)

    def frame(self) -> np.ndarray:
        """RGB uint8 frame."""
        t = self.telemetry() or {}
        dist = float(t.get("distance_m") or 0.0)
        gim = t.get("gimbal") or {}
        light = t.get("light") or {}
        pan, tilt = float(gim.get("pan_deg") or 0.0), float(gim.get("tilt_deg") or 0.0)
        white, ir = float(light.get("white_pct") or 0) / 100.0, float(light.get("ir_pct") or 0) / 100.0
        stale = bool(t.get("stale", True))
        # vanishing point moves with pan/tilt; at |tilt| -> 90 deg it leaves the frame (wall view)
        vx = np.tan(np.radians(np.clip(pan, -80, 80))) * 0.9
        vy = -np.tan(np.radians(np.clip(tilt, -80, 80))) * 0.9
        dx, dy = self.xx - vx, self.yy - vy
        r = np.sqrt(dx * dx + dy * dy) + 1e-3
        phi = np.arctan2(dy, dx)
        depth = 0.35 / r  # metres down the pipe for a wall point at image radius r
        along = depth + dist
        seam = (np.mod(along, 1.0) < 0.012).astype(np.float32)
        u = (np.mod(along * 8.0, 1.0) * 63).astype(np.int32)
        v = (np.mod(phi / (2 * np.pi) + 0.5, 1.0) * 255).astype(np.int32)
        tex = self.tex[u, v]
        illum = (0.15 + 0.85 * max(white, ir)) / (1.0 + 1.6 * depth * depth)
        base = (0.45 + 0.35 * tex - 0.6 * seam) * illum
        if ir > white:
            rgb = np.stack([base, base, base], axis=-1)
        else:
            rgb = np.stack([base * 0.95, base * 0.78, base * 0.58], axis=-1)  # rusty steel tint
        img = np.clip(rgb * 255.0, 0, 255).astype(np.uint8)
        if stale:
            img = (img * 0.35).astype(np.uint8)  # dark when the robot link is stale
        return img


class LibcameraCamera:
    """Picamera2 source (robot only)."""

    def __init__(self, w: int, h: int, fps: int) -> None:
        """Open the IMX708."""
        from picamera2 import Picamera2  # type: ignore[import-not-found]

        self.cam = Picamera2()
        cfg = self.cam.create_video_configuration(main={"size": (w, h), "format": "RGB888"}, controls={"FrameRate": fps})
        self.cam.configure(cfg)
        self.cam.start()

    def frame(self) -> np.ndarray:
        """RGB frame."""
        return self.cam.capture_array("main")


class VideoHub:
    """Captures frames on a thread; consumers read the latest frame."""

    def __init__(self, source: Any, fps: int) -> None:
        """Wrap a source with frame()."""
        self.source = source
        self.fps = fps
        self.latest: np.ndarray | None = None
        self.frame_no = 0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        """Start capturing."""
        self._thread.start()

    def stop(self) -> None:
        """Stop capturing."""
        self._stop.set()

    def _run(self) -> None:
        period = 1.0 / self.fps
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                f = self.source.frame()
                with self._lock:
                    self.latest = f
                    self.frame_no += 1
            except Exception:  # noqa: BLE001 - a camera glitch must not kill the server
                log.exception("frame capture failed")
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    def get(self) -> np.ndarray | None:
        """Latest frame."""
        with self._lock:
            return self.latest

    def jpeg(self, quality: int = 70) -> bytes | None:
        """Latest frame as JPEG."""
        f = self.get()
        if f is None:
            return None
        buf = io.BytesIO()
        Image.fromarray(f).save(buf, format="JPEG", quality=quality)
        return buf.getvalue()


def make_source(kind: str, w: int, h: int, fps: int, telemetry: Callable[[], dict[str, Any]]) -> Any:
    """libcamera when available (auto), else the synthetic renderer."""
    if kind in ("auto", "libcamera"):
        try:
            return LibcameraCamera(w, h, fps)
        except Exception as e:  # noqa: BLE001
            if kind == "libcamera":
                raise
            log.info("libcamera unavailable (%s): using the synthetic pipe camera", e)
    return SyntheticCamera(min(w, 640), min(h, 360), telemetry)


def webrtc_track(hub: VideoHub) -> Any:
    """aiortc video track fed from the hub."""
    import av
    from aiortc import VideoStreamTrack

    class HubTrack(VideoStreamTrack):
        kind = "video"

        def __init__(self) -> None:
            super().__init__()
            self._pts = 0

        async def recv(self) -> av.VideoFrame:
            import asyncio

            await asyncio.sleep(1.0 / hub.fps)
            f = hub.get()
            if f is None:
                f = np.zeros((360, 640, 3), np.uint8)
            frame = av.VideoFrame.from_ndarray(f, format="rgb24")
            self._pts += int(90000 / hub.fps)
            frame.pts = self._pts
            frame.time_base = Fraction(1, 90000)
            return frame

    return HubTrack()
