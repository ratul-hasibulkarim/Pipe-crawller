"""Command API: validation, TTL enforcement, rate limiting, dead-man refresh for motion.

Motion commands always carry a short TTL. A drive command keeps the robot moving only while
the operator keeps sending it (joystick held); the MCU stops on its own when they stop.
EXPAND is refreshed by the Pi until the MCU reports the expansion finished, the job is
cancelled, or it times out.
"""

from __future__ import annotations

import asyncio
import contextlib
import time
from typing import Any

from . import protocol_gen as P
from .telemetry import RobotHub


class RateLimiter:
    """Token bucket per client."""

    def __init__(self, per_s: int) -> None:
        """Bucket of `per_s` tokens refilled per second."""
        self.cap = float(per_s)
        self.tokens = float(per_s)
        self.t = time.monotonic()

    def allow(self) -> bool:
        """Take one token if available."""
        now = time.monotonic()
        self.tokens = min(self.cap, self.tokens + (now - self.t) * self.cap)
        self.t = now
        if self.tokens >= 1:
            self.tokens -= 1
            return True
        return False


class CommandService:
    """Validated commands to the robot."""

    def __init__(self, hub: RobotHub) -> None:
        """Bind to the hub."""
        self.hub = hub
        self.s = hub.s
        self.limiter = RateLimiter(self.s.max_commands_per_s)
        self._expand_task: asyncio.Task | None = None
        self.expand_job: dict[str, Any] = {"active": False}

    def _p(self, key: str) -> float:
        return self.s.params[key]

    async def _send(self, msg: Any) -> dict[str, Any]:
        if not self.limiter.allow():
            return {"status": "RATE_LIMITED", "seq": getattr(msg, "seq", 0)}
        return await self.hub.send(msg)

    async def drive(self, linear_mm_s: float, turn_deg_s: float = 0, bend_dir_deg: float = 0, ttl_ms: int | None = None) -> dict[str, Any]:
        """One drive command (TTL clamped to 50..1000 ms)."""
        vmax = self._p("MAX_SPEED_MM_S")
        v = max(-vmax, min(vmax, float(linear_mm_s)))
        ttl = int(max(50, min(1000, ttl_ms or self.s.drive_ttl_ms)))
        msg = P.DriveMsg(
            seq=self.hub.next_seq(), linear_mm_s=int(round(v)), turn_deg_s=int(max(-90, min(90, turn_deg_s))), bend_dir_deg=int(bend_dir_deg) % 360, ttl_ms=ttl
        )
        return await self._send(msg)

    async def stop(self) -> dict[str, Any]:
        """Zero-speed drive (always accepted)."""
        return await self.hub.send(P.DriveMsg(seq=self.hub.next_seq(), ttl_ms=200))

    async def expand_once(self, mode: str, target: float) -> dict[str, Any]:
        """Single EXPAND (the job below refreshes it)."""
        m = {"DIAMETER": P.ExpandMode.DIAMETER, "FORCE": P.ExpandMode.FORCE, "RETRACT": P.ExpandMode.RETRACT}[mode]
        return await self._send(P.ExpandMsg(seq=self.hub.next_seq(), mode=int(m), target=float(target), ttl_ms=self.s.expand_ttl_ms))

    async def expand(self, mode: str, target: float | None, timeout_s: float = 40.0) -> dict[str, Any]:
        """Start an expansion job; returns the first ACK. Replaces any running job."""
        mode = mode.upper()
        if mode not in ("DIAMETER", "FORCE", "RETRACT"):
            return {"status": "BAD_ARG"}
        if target is None:
            target = self._p("DEFAULT_GRIP_TARGET_N") if mode == "FORCE" else 0.0
        await self.cancel_expand()
        first = await self.expand_once(mode, target)
        if first.get("status") != "OK":
            return first
        self.expand_job = {"active": True, "mode": mode, "target": target, "started": time.time(), "result": None}
        self._expand_task = asyncio.create_task(self._expand_refresh(mode, target, timeout_s))
        return first

    async def _expand_refresh(self, mode: str, target: float, timeout_s: float) -> None:
        t0 = time.monotonic()
        seen_busy = False
        try:
            while time.monotonic() - t0 < timeout_s:
                await asyncio.sleep(0.15)
                tel = self.hub.latest or {}
                st = tel.get("state")
                busy = st in ("EXPANDING", "RETRACTING")
                seen_busy = seen_busy or busy
                if (seen_busy or time.monotonic() - t0 > 1.0) and not busy:
                    self.expand_job.update(active=False, result=st)
                    return
                r = await self.expand_once(mode, target)
                if r.get("status") not in ("OK", "TIMEOUT"):
                    self.expand_job.update(active=False, result=r.get("status"))
                    return
            self.expand_job.update(active=False, result="TIMEOUT")
        except asyncio.CancelledError:
            self.expand_job.update(active=False, result="CANCELLED")
            raise

    async def cancel_expand(self) -> None:
        """Stop refreshing EXPAND: the MCU halts the arms when the TTL lapses (worms lock)."""
        if self._expand_task and not self._expand_task.done():
            self._expand_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._expand_task

    async def gimbal(self, pan_deg: float, tilt_deg: float) -> dict[str, Any]:
        """Camera pan (continuous, wrapped to +/-180) and tilt (+/-90)."""
        pan = ((float(pan_deg) + 180.0) % 360.0) - 180.0
        tilt = max(-90.0, min(90.0, float(tilt_deg)))
        return await self._send(P.GimbalMsg(seq=self.hub.next_seq(), pan_deg=pan, tilt_deg=tilt))

    async def light(self, white_pct: int, ir_pct: int) -> dict[str, Any]:
        """LED levels 0-100 %."""
        return await self._send(P.LightMsg(seq=self.hub.next_seq(), white_pct=max(0, min(100, int(white_pct))), ir_pct=max(0, min(100, int(ir_pct)))))

    async def estop(self) -> dict[str, Any]:
        """E-stop: never rate limited, sent three times (lossy links)."""
        await self.cancel_expand()
        res: dict[str, Any] = {}
        for _ in range(3):
            res = await self.hub.send(P.EstopMsg(seq=self.hub.next_seq()))
            if res.get("status") == "ESTOPPED":
                break
        return res

    async def clear_fault(self) -> dict[str, Any]:
        """Acknowledge faults."""
        return await self.hub.send(P.ClearFaultMsg(seq=self.hub.next_seq()))

    async def set_param(self, name: str, value: float) -> dict[str, Any]:
        """Tuning parameter by name (Param enum)."""
        try:
            pid = P.Param[name.upper()]
        except KeyError:
            return {"status": "BAD_ARG"}
        return await self._send(P.SetParamMsg(seq=self.hub.next_seq(), param=int(pid), value=float(value)))
