"""REST API against a fake MCU: ACK pass-through, clamping, stale detection, rate limit, recording."""

from __future__ import annotations

import asyncio

import httpx
import pytest

from pi.app import protocol_gen as P
from pi.app.config import Settings
from pi.app.main import create_app


def make(port: int, tmp_path):  # noqa: ANN201
    """App bound to a link port."""
    s = Settings(link=f"tcp://127.0.0.1:{port}", camera="synthetic", recordings=tmp_path / "rec", static_dir=tmp_path / "none")
    return create_app(s)


async def wait_for(cond, timeout: float = 3.0) -> bool:  # noqa: ANN001
    """Poll a condition."""
    t = 0.0
    while t < timeout:
        if cond():
            return True
        await asyncio.sleep(0.05)
        t += 0.05
    return False


async def test_commands_are_acked_and_validated(fake_mcu, tmp_path) -> None:
    """Clamps, wraps, passes MCU status through with the right HTTP code."""
    app = make(fake_mcu.port, tmp_path)
    async with app.router.lifespan_context(app):
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://t") as c:
            assert await wait_for(lambda: app.state.hub.latest is not None)
            r = await c.post("/api/v1/drive", json={"linear_mm_s": 9999, "ttl_ms": 5000})
            assert r.status_code == 200 and r.json()["status"] == "OK"
            drive = [m for m in fake_mcu.received if isinstance(m, P.DriveMsg)][-1]
            assert drive.linear_mm_s == int(Settings().params["MAX_SPEED_MM_S"])
            assert drive.ttl_ms == 1000
            await c.post("/api/v1/gimbal", json={"pan_deg": 270, "tilt_deg": 120})
            g = [m for m in fake_mcu.received if isinstance(m, P.GimbalMsg)][-1]
            assert abs(g.pan_deg + 90) < 1e-4 and g.tilt_deg == 90
            fake_mcu.status = int(P.AckStatus.REJECTED)
            r = await c.post("/api/v1/drive", json={"linear_mm_s": 10})
            assert r.status_code == 409 and r.json()["status"] == "REJECTED"
            assert (await c.post("/api/v1/estop")).json()["status"] == "ESTOPPED"
            assert (await c.post("/api/v1/param", json={"name": "NOPE", "value": 1})).status_code == 422


async def test_stale_telemetry_is_flagged(fake_mcu, tmp_path) -> None:
    """Fail-visible: a silent MCU makes the snapshot stale and health degraded."""
    app = make(fake_mcu.port, tmp_path)
    async with app.router.lifespan_context(app):
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://t") as c:
            assert await wait_for(lambda: app.state.hub.latest is not None)
            assert (await c.get("/api/v1/telemetry")).json()["stale"] is False
            fake_mcu.silent = True
            await asyncio.sleep(0.5)
            snap = (await c.get("/api/v1/telemetry")).json()
            assert snap["stale"] is True and snap["age_ms"] > 300
            assert (await c.get("/api/v1/health")).json()["status"] == "degraded"


async def test_no_link_reports_503(tmp_path) -> None:
    """No MCU: commands fail loudly, telemetry is stale."""
    app = make(1, tmp_path)
    async with app.router.lifespan_context(app):
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://t") as c:
            r = await c.post("/api/v1/drive", json={"linear_mm_s": 10})
            assert r.status_code == 503 and r.json()["status"] == "NO_LINK"
            assert (await c.get("/api/v1/telemetry")).json()["stale"] is True


def test_rate_limiter() -> None:
    """Token bucket."""
    from pi.app.control import RateLimiter

    rl = RateLimiter(5)
    assert sum(rl.allow() for _ in range(20)) == 5


async def test_recording_round_trip(fake_mcu, tmp_path) -> None:
    """Start, mark, snapshot, stop, list, export, path traversal refused."""
    app = make(fake_mcu.port, tmp_path)
    async with app.router.lifespan_context(app):
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://t") as c:
            assert await wait_for(lambda: app.state.video.frame_no > 2)
            rid = (await c.post("/api/v1/recording/start", json={"operator": "test"})).json()["id"]
            await asyncio.sleep(0.6)
            await c.post("/api/v1/mark", json={"label": "crack"})
            assert (await c.post("/api/v1/snapshot")).status_code == 200
            await c.post("/api/v1/recording/stop")
            runs = (await c.get("/api/v1/recordings")).json()
            assert runs[0]["id"] == rid and runs[0]["frames"] > 3
            detail = (await c.get(f"/api/v1/recordings/{rid}")).json()
            assert any(e.get("label") == "crack" for e in detail["events"])
            assert len(detail["snapshots"]) == 1
            csv_text = (await c.get(f"/api/v1/recordings/{rid}/export.csv")).text
            assert csv_text.startswith("t_wall,") and csv_text.count("\n") > 5
            assert (await c.get("/api/v1/recordings/..%2F..%2Fetc/export.csv")).status_code == 404


@pytest.mark.parametrize("pan,expect", [(190, -170), (-190, 170), (0, 0)])
async def test_pan_wraps(fake_mcu, tmp_path, pan, expect) -> None:
    """Continuous pan is sent wrapped to +/-180."""
    app = make(fake_mcu.port, tmp_path)
    async with app.router.lifespan_context(app):
        assert await wait_for(lambda: app.state.hub.link.connected.is_set())
        await app.state.cmd.gimbal(pan, 0)
        g = [m for m in fake_mcu.received if isinstance(m, P.GimbalMsg)][-1]
        assert abs(g.pan_deg - expect) < 1e-4
