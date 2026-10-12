"""PIPEBOT-X companion server (Raspberry Pi): REST + WebSocket API, WebRTC/MJPEG video,
recording, and the operator dashboard (served from app/dist).

    PIPEBOT_LINK=tcp://127.0.0.1:7000 uvicorn pi.app.main:app --host 0.0.0.0 --port 8000
"""

from __future__ import annotations

import asyncio
import contextlib
import csv
import io
import json
import time
from contextlib import asynccontextmanager
from typing import Any

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from . import protocol_gen as P
from .config import Settings
from .control import CommandService
from .recorder import Recorder
from .telemetry import RobotHub
from .video import VideoHub, make_source, webrtc_track

VERSION = "3.0.0"


class DriveBody(BaseModel):
    """POST /drive."""

    linear_mm_s: float = Field(..., description="+ forward / up the pipe")
    turn_deg_s: float = 0.0
    bend_dir_deg: float = 0.0
    ttl_ms: int | None = None


class ExpandBody(BaseModel):
    """POST /expand."""

    mode: str = Field("FORCE", description="DIAMETER (mm) | FORCE (N per arm) | RETRACT")
    target: float | None = None


class GimbalBody(BaseModel):
    """POST /gimbal."""

    pan_deg: float = 0.0
    tilt_deg: float = 0.0


class LightBody(BaseModel):
    """POST /light."""

    white_pct: int = 0
    ir_pct: int = 0


class ParamBody(BaseModel):
    """POST /param."""

    name: str
    value: float


class RecordBody(BaseModel):
    """POST /recording/start."""

    operator: str = ""
    pipe_id_range: str = ""
    note: str = ""


class MarkBody(BaseModel):
    """POST /mark."""

    label: str = "event"


class OfferBody(BaseModel):
    """WebRTC offer."""

    sdp: str
    type: str


def create_app(settings: Settings | None = None) -> FastAPI:
    """Application factory (tests build their own)."""
    s = settings or Settings()
    hub = RobotHub(s)
    cmd = CommandService(hub)
    rec = Recorder(s.recordings)
    hub.add_listener(rec.on_message)
    video = VideoHub(make_source(s.camera, s.video_w, s.video_h, s.video_fps, hub.snapshot), s.video_fps)
    pcs: set[Any] = set()

    async def record_frames() -> None:
        last = -1
        while True:
            await asyncio.sleep(1.0 / s.video_fps)
            if rec.active and video.frame_no != last:
                f = video.get()
                if f is not None:
                    last = video.frame_no
                    await asyncio.to_thread(rec.add_frame, f)

    @asynccontextmanager
    async def lifespan(_: FastAPI):  # noqa: ANN202
        await hub.start()
        video.start()
        task = asyncio.create_task(record_frames())
        yield
        task.cancel()
        video.stop()
        rec.stop()
        for pc in list(pcs):
            await pc.close()
        await hub.stop()

    app = FastAPI(title="PIPEBOT-X companion API", version=VERSION, lifespan=lifespan)
    app.state.hub, app.state.cmd, app.state.rec, app.state.video = hub, cmd, rec, video

    # ------------------------------------------------------------------ status
    @app.get("/api/v1/health")
    async def health() -> dict[str, Any]:
        snap = hub.snapshot()
        return {
            "status": "ok" if snap["link"] and not snap["stale"] else "degraded",
            "version": VERSION,
            "size": s.size,
            "link": snap["link"],
            "telemetry_age_ms": snap["age_ms"],
            "telemetry_packets": hub.tel_count,
            "rtt_ms": snap["rtt_ms"],
            "video_frames": video.frame_no,
            "recording": rec.run_id,
            "tether_m": s.tether_length_m,
            "protocol_version": P.VERSION,
            "link_reconnects": hub.link.reconnects,
        }

    @app.get("/api/v1/config")
    async def config() -> dict[str, Any]:
        p = s.params
        return {
            "size": s.size,
            "pipe_id_min_mm": p["PIPE_ID_MIN_MM"],
            "pipe_id_max_mm": p["PIPE_ID_MAX_MM"],
            "max_speed_mm_s": p["MAX_SPEED_MM_S"],
            "max_grip_n": p["MAX_GRIP_N"],
            "working_grip_n": p["WORKING_GRIP_N"],
            "default_grip_n": p["DEFAULT_GRIP_TARGET_N"],
            "grip_low_n": p["GRIP_LOW_N"],
            "telemetry_hz": p["TELEMETRY_HZ"],
            "drive_ttl_ms": s.drive_ttl_ms,
            "tether_length_m": s.tether_length_m,
            "battery_series": p["BATTERY_SERIES"],
            "gimbal_tilt_range": [p["GIMBAL_TILT_MIN_DEG"], p["GIMBAL_TILT_MAX_DEG"]],
            "params": [x.name for x in P.Param],
        }

    @app.get("/api/v1/telemetry")
    async def telemetry() -> dict[str, Any]:
        return hub.snapshot()

    @app.websocket("/api/v1/telemetry")
    async def telemetry_ws(ws: WebSocket) -> None:
        await ws.accept()
        q = hub.subscribe()
        try:
            while True:
                await ws.send_text(json.dumps(await q.get()))
        except (WebSocketDisconnect, RuntimeError):
            pass
        finally:
            hub.unsubscribe(q)

    @app.get("/api/v1/events")
    async def events(limit: int = 100) -> list[dict[str, Any]]:
        return list(hub.events)[-limit:]

    @app.get("/api/v1/preflight")
    async def preflight() -> dict[str, Any]:
        snap = hub.snapshot()
        faults = snap.get("faults") or []
        imu_ok = "IMU_FAIL" not in faults
        checks = [
            {"name": "MCU link", "ok": bool(snap["link"]), "detail": f"{hub.link.url}"},
            {"name": "Telemetry fresh (20 Hz)", "ok": not snap["stale"], "detail": f"age {snap['age_ms']} ms"},
            {"name": "IMU", "ok": imu_ok and snap.get("imu") is not None, "detail": f"pitch {(snap.get('imu') or {}).get('pitch')}"},
            {"name": "Camera frames", "ok": video.frame_no > 0, "detail": f"{video.frame_no} frames"},
            {"name": "Lever / wheel encoders", "ok": "ENCODER_FAIL" not in faults, "detail": ""},
            {
                "name": "No latched faults",
                "ok": not any(f in faults for f in ("ESTOP", "OVERCURRENT", "OVERTEMP", "EXPANSION_STALL")),
                "detail": ",".join(faults),
            },
            {
                "name": "Supply",
                "ok": not any(f in faults for f in ("CRITICAL_BATTERY", "SUPPLY_UNDERVOLT")),
                "detail": f"{(snap.get('battery') or {}).get('v')} V {(snap.get('battery') or {}).get('supply')}",
            },
            {"name": "Grip test (arms loaded)", "ok": snap.get("state") in ("GRIPPED", "HOLDING", "CRAWLING"), "detail": f"grip {snap.get('grip_force_n')} N"},
        ]
        return {"ok": all(c["ok"] for c in checks), "checks": checks}

    # ------------------------------------------------------------------ commands
    def _resp(r: dict[str, Any]) -> JSONResponse:
        code = 200 if r.get("status") in ("OK", "ESTOPPED", "SENT") else (503 if r.get("status") in ("NO_LINK", "TIMEOUT") else 409)
        if r.get("status") in ("RATE_LIMITED",):
            code = 429
        if r.get("status") == "BAD_ARG":
            code = 422
        return JSONResponse(r, status_code=code)

    @app.post("/api/v1/drive")
    async def drive(b: DriveBody) -> JSONResponse:
        return _resp(await cmd.drive(b.linear_mm_s, b.turn_deg_s, b.bend_dir_deg, b.ttl_ms))

    @app.post("/api/v1/stop")
    async def stop() -> JSONResponse:
        return _resp(await cmd.stop())

    @app.post("/api/v1/expand")
    async def expand(b: ExpandBody) -> JSONResponse:
        return _resp(await cmd.expand(b.mode, b.target))

    @app.get("/api/v1/expand")
    async def expand_job() -> dict[str, Any]:
        return cmd.expand_job

    @app.delete("/api/v1/expand")
    async def expand_cancel() -> dict[str, Any]:
        await cmd.cancel_expand()
        return cmd.expand_job

    @app.post("/api/v1/gimbal")
    async def gimbal(b: GimbalBody) -> JSONResponse:
        return _resp(await cmd.gimbal(b.pan_deg, b.tilt_deg))

    @app.post("/api/v1/light")
    async def light(b: LightBody) -> JSONResponse:
        return _resp(await cmd.light(b.white_pct, b.ir_pct))

    @app.post("/api/v1/estop")
    async def estop() -> JSONResponse:
        if rec.active:
            rec.mark("ESTOP")
        return _resp(await cmd.estop())

    @app.post("/api/v1/clear_fault")
    async def clear_fault() -> JSONResponse:
        return _resp(await cmd.clear_fault())

    @app.post("/api/v1/param")
    async def param(b: ParamBody) -> JSONResponse:
        return _resp(await cmd.set_param(b.name, b.value))

    @app.websocket("/api/v1/control")
    async def control_ws(ws: WebSocket) -> None:
        """Low-latency control channel: {"type":"drive","linear_mm_s":..} at ~10 Hz while the stick is held."""
        await ws.accept()
        try:
            while True:
                m = json.loads(await ws.receive_text())
                t = m.get("type")
                if t == "drive":
                    r = await cmd.drive(m.get("linear_mm_s", 0), m.get("turn_deg_s", 0), m.get("bend_dir_deg", 0), m.get("ttl_ms"))
                elif t == "gimbal":
                    r = await cmd.gimbal(m.get("pan_deg", 0), m.get("tilt_deg", 0))
                elif t == "estop":
                    r = await cmd.estop()
                else:
                    r = {"status": "BAD_ARG"}
                await ws.send_text(json.dumps({"id": m.get("id"), **r}))
        except (WebSocketDisconnect, RuntimeError, json.JSONDecodeError):
            # client gone: stop driving now instead of waiting for the TTL
            with contextlib.suppress(Exception):
                await cmd.stop()

    # ------------------------------------------------------------------ video
    @app.get("/stream.mjpeg")
    async def mjpeg() -> StreamingResponse:
        async def gen():  # noqa: ANN202
            while True:
                jpg = await asyncio.to_thread(video.jpeg)
                if jpg:
                    yield b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n"
                await asyncio.sleep(1.0 / 15)

        return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")

    @app.get("/api/v1/snapshot.jpg")
    async def snapshot_jpg() -> Response:
        jpg = await asyncio.to_thread(video.jpeg, 90)
        if not jpg:
            raise HTTPException(503, "no frame yet")
        return Response(jpg, media_type="image/jpeg")

    @app.post("/api/v1/webrtc/offer")
    async def webrtc_offer(b: OfferBody) -> dict[str, str]:
        from aiortc import RTCPeerConnection, RTCSessionDescription

        pc = RTCPeerConnection()
        pcs.add(pc)

        @pc.on("connectionstatechange")
        async def _state() -> None:
            if pc.connectionState in ("failed", "closed"):
                await pc.close()
                pcs.discard(pc)

        pc.addTrack(webrtc_track(video))
        await pc.setRemoteDescription(RTCSessionDescription(sdp=b.sdp, type=b.type))
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)
        return {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}

    # ------------------------------------------------------------------ recording
    @app.post("/api/v1/recording/start")
    async def rec_start(b: RecordBody) -> dict[str, Any]:
        f = video.get()
        h, w = (f.shape[0], f.shape[1]) if f is not None else (360, 640)
        rid = rec.start(
            {"operator": b.operator, "pipe_id_range": b.pipe_id_range, "note": b.note, "tether_m": s.tether_length_m, "size": s.size, "version": VERSION},
            w,
            h,
            s.video_fps,
        )
        return {"id": rid}

    @app.post("/api/v1/recording/stop")
    async def rec_stop() -> dict[str, Any]:
        return {"id": await asyncio.to_thread(rec.stop)}

    @app.post("/api/v1/mark")
    async def mark(b: MarkBody) -> dict[str, Any]:
        rec.mark(b.label, {"distance_m": (hub.latest or {}).get("distance_m")})
        return {"ok": rec.active}

    @app.post("/api/v1/snapshot")
    async def snapshot() -> dict[str, Any]:
        jpg = await asyncio.to_thread(video.jpeg, 92)
        if not jpg:
            raise HTTPException(503, "no frame yet")
        name = rec.snapshot(jpg, hub.snapshot())
        return {"file": name, "recording": rec.active}

    @app.get("/api/v1/recordings")
    async def recordings() -> list[dict[str, Any]]:
        return rec.list()

    def _run_path(rid: str):  # noqa: ANN202
        p = rec.path(rid)
        if p is None:
            raise HTTPException(404, "no such recording")
        return p

    @app.get("/api/v1/recordings/{rid}")
    async def recording(rid: str) -> dict[str, Any]:
        p = _run_path(rid)
        events = [json.loads(line) for line in (p / "events.jsonl").read_text().splitlines() if line.strip()] if (p / "events.jsonl").exists() else []
        snaps = sorted(x.name for x in (p / "snapshots").glob("*.jpg"))
        return {"meta": json.loads((p / "meta.json").read_text()), "events": events, "snapshots": snaps}

    @app.get("/api/v1/recordings/{rid}/export.csv")
    async def export_csv(rid: str, start_s: float | None = None, end_s: float | None = None) -> Response:
        p = _run_path(rid) / "telemetry.csv"
        if not p.exists():
            raise HTTPException(404, "no telemetry")
        if start_s is None and end_s is None:
            return FileResponse(p, media_type="text/csv", filename=f"{rid}_telemetry.csv")
        rows = list(csv.DictReader(p.open()))
        t0 = float(rows[0]["t_wall"]) if rows else 0
        out = io.StringIO()
        w = csv.DictWriter(out, fieldnames=list(rows[0].keys()) if rows else [])
        w.writeheader()
        for r in rows:
            t = float(r["t_wall"]) - t0
            if (start_s is None or t >= start_s) and (end_s is None or t <= end_s):
                w.writerow(r)
        return Response(out.getvalue(), media_type="text/csv")

    @app.get("/api/v1/recordings/{rid}/telemetry.json")
    async def rec_telemetry(rid: str, step: int = 5) -> list[dict[str, Any]]:
        p = _run_path(rid) / "telemetry.csv"
        if not p.exists():
            return []
        rows = list(csv.DictReader(p.open()))
        t0 = float(rows[0]["t_wall"]) if rows else 0
        keep = ("distance_m", "pipe_orientation_deg", "grip0", "grip1", "grip2", "cur0", "cur3", "est_diameter_mm", "linear_mm_s", "state")
        return [{"t": round(float(r["t_wall"]) - t0, 2), **{k: r[k] for k in keep}} for r in rows[:: max(1, step)]]

    @app.get("/api/v1/recordings/{rid}/video.mp4")
    async def rec_video(rid: str) -> FileResponse:
        return FileResponse(_run_path(rid) / "video.mp4", media_type="video/mp4")

    @app.get("/api/v1/recordings/{rid}/snapshots/{name}")
    async def rec_snapshot(rid: str, name: str) -> FileResponse:
        if "/" in name or ".." in name:
            raise HTTPException(400, "bad name")
        return FileResponse(_run_path(rid) / "snapshots" / name)

    @app.get("/api/v1/recordings/{rid}/clip.mp4")
    async def rec_clip(rid: str, start_s: float, end_s: float) -> Response:
        """Re-encode a segment of the run video."""
        import av

        src = _run_path(rid) / "video.mp4"

        def cut() -> bytes:
            buf = io.BytesIO()
            with av.open(str(src)) as inp, av.open(buf, "w", format="mp4") as out:
                vin = inp.streams.video[0]
                vout = out.add_stream("libx264", rate=vin.average_rate or 30)
                vout.width, vout.height, vout.pix_fmt = vin.width, vin.height, "yuv420p"
                for fr in inp.decode(vin):
                    t = float(fr.pts * vin.time_base) if fr.pts is not None else 0
                    if start_s <= t <= end_s:
                        fr.pts = None
                        for pkt in vout.encode(fr):
                            out.mux(pkt)
                for pkt in vout.encode():
                    out.mux(pkt)
            return buf.getvalue()

        return Response(await asyncio.to_thread(cut), media_type="video/mp4")

    # ------------------------------------------------------------------ dashboard
    if s.static_dir.exists():
        app.mount("/", StaticFiles(directory=s.static_dir, html=True), name="app")
    else:

        @app.get("/")
        async def no_app() -> dict[str, Any]:
            return {"msg": "dashboard not built: cd app && npm ci && npm run build", "time": time.time()}

    return app


app = create_app()
