"""Full stack: FastAPI server -> TCP -> firmware core running in the SIL simulator."""

from __future__ import annotations

import asyncio

import httpx

from pi.app.config import Settings
from pi.app.main import create_app


async def test_expand_climb_hold_via_api(sil_server, tmp_path) -> None:
    """Grip, climb 1.5 s up a vertical pipe with the joystick, release, robot holds."""
    s = Settings(link=f"tcp://127.0.0.1:{sil_server}", camera="synthetic", recordings=tmp_path / "rec", static_dir=tmp_path / "x")
    app = create_app(s)
    async with app.router.lifespan_context(app):
        async with httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://t") as c:
            for _ in range(60):
                if app.state.hub.latest:
                    break
                await asyncio.sleep(0.05)
            assert (await c.post("/api/v1/expand", json={"mode": "FORCE"})).json()["status"] == "OK"
            for _ in range(80):
                if not (await c.get("/api/v1/expand")).json()["active"]:
                    break
                await asyncio.sleep(0.1)
            t = (await c.get("/api/v1/telemetry")).json()
            assert t["state"] in ("GRIPPED", "HOLDING")
            d0 = t["distance_m"]
            for _ in range(15):
                assert (await c.post("/api/v1/drive", json={"linear_mm_s": 60})).json()["status"] == "OK"
                await asyncio.sleep(0.1)
            await asyncio.sleep(0.6)  # joystick released: the MCU's TTL stops it
            t = (await c.get("/api/v1/telemetry")).json()
            assert t["state"] == "HOLDING"
            assert 0.05 < t["distance_m"] - d0 < 0.15
            pre = (await c.get("/api/v1/preflight")).json()
            assert pre["ok"], pre
