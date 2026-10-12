"""MCU link hub: decodes frames, keeps the latest telemetry, fans it out at 20 Hz, matches ACKs.

Fail-visible rule: every telemetry message carries `age_ms` and `stale`; nothing the UI shows
can look live when the link is down.
"""

from __future__ import annotations

import asyncio
import contextlib
import time
from collections import deque
from collections.abc import Callable
from typing import Any

from . import protocol_gen as P
from .config import Settings
from .link import Link


class RobotHub:
    """One per process: owns the link to the MCU."""

    def __init__(self, settings: Settings) -> None:
        """Create the decoder and link (not started)."""
        self.s = settings
        self.decoder = P.FrameDecoder()
        self.link = Link(settings.link, self._on_bytes)
        self.latest: dict[str, Any] | None = None
        self.latest_rx = 0.0
        self.events: deque[dict[str, Any]] = deque(maxlen=500)
        self.params: dict[int, float] = {}
        self.tel_count = 0
        self.rtt_ms: float | None = None
        self._acks: dict[int, asyncio.Future] = {}
        self._subs: set[asyncio.Queue] = set()
        self._listeners: list[Callable[[dict[str, Any]], None]] = []
        self._tasks: list[asyncio.Task] = []
        self._seq = 1

    # ---------------------------------------------------------------- lifecycle
    async def start(self) -> None:
        """Connect and start the ping + broadcast loops."""
        await self.link.start()
        self._tasks = [asyncio.create_task(self._ping_loop()), asyncio.create_task(self._broadcast_loop())]

    async def stop(self) -> None:
        """Stop everything."""
        for t in self._tasks:
            t.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await t
        await self.link.stop()

    def next_seq(self) -> int:
        """16-bit command sequence number."""
        self._seq = (self._seq + 1) & 0xFFFF or 1
        return self._seq

    def add_listener(self, fn: Callable[[dict[str, Any]], None]) -> None:
        """Called for every decoded message (recorder)."""
        self._listeners.append(fn)

    # ---------------------------------------------------------------- receive
    def _on_bytes(self, data: bytes) -> None:
        now = time.monotonic()
        for msg in self.decoder.feed(data):
            j = msg.to_json()
            if isinstance(msg, P.TelemetryMsg):
                self.latest = j
                self.latest_rx = now
                self.tel_count += 1
            elif isinstance(msg, P.AckMsg):
                fut = self._acks.pop(msg.seq, None)
                if fut is not None and not fut.done():
                    fut.set_result(j)
            elif isinstance(msg, P.PongMsg):
                self.rtt_ms = (time.monotonic() * 1000.0) % (2**32) - msg.t_ms
            elif isinstance(msg, P.EventMsg):
                j["t_wall"] = time.time()
                self.events.append(j)
            elif isinstance(msg, P.ParamValueMsg):
                self.params[msg.param] = msg.value
            for fn in self._listeners:
                fn(j)

    # ---------------------------------------------------------------- send
    async def send(self, msg: Any, wait_ack: bool = True) -> dict[str, Any]:
        """Send a command; wait for its ACK (status OK/REJECTED/...) or report NO_LINK/TIMEOUT."""
        loop = asyncio.get_running_loop()
        fut: asyncio.Future | None = None
        if wait_ack and hasattr(msg, "seq"):
            fut = loop.create_future()
            self._acks[msg.seq] = fut
        if not await self.link.write(P.encode(msg)):
            if fut is not None:
                self._acks.pop(msg.seq, None)
            return {"status": "NO_LINK", "seq": getattr(msg, "seq", 0)}
        if fut is None:
            return {"status": "SENT", "seq": getattr(msg, "seq", 0)}
        try:
            return await asyncio.wait_for(fut, self.s.ack_timeout_s)
        except asyncio.TimeoutError:
            self._acks.pop(msg.seq, None)
            return {"status": "TIMEOUT", "seq": msg.seq}

    async def _ping_loop(self) -> None:
        while True:
            await asyncio.sleep(1.0 / self.s.ping_hz)
            ping = P.PingMsg(seq=self.next_seq(), t_ms=int(time.monotonic() * 1000.0) % (2**32))
            await self.link.write(P.encode(ping))

    # ---------------------------------------------------------------- fan-out
    def snapshot(self) -> dict[str, Any]:
        """Latest telemetry with freshness fields (what the UI may show)."""
        age = (time.monotonic() - self.latest_rx) * 1000.0 if self.latest else None
        base = dict(self.latest) if self.latest else {"type": "TELEMETRY"}
        base["age_ms"] = round(age, 1) if age is not None else None
        base["stale"] = age is None or age > self.s.stale_after_s * 1000.0
        base["link"] = self.link.connected.is_set()
        base["rtt_ms"] = round(self.rtt_ms, 1) if self.rtt_ms is not None else None
        return base

    def subscribe(self) -> asyncio.Queue:
        """Queue receiving snapshots at the telemetry rate."""
        q: asyncio.Queue = asyncio.Queue(maxsize=4)
        self._subs.add(q)
        return q

    def unsubscribe(self, q: asyncio.Queue) -> None:
        """Remove a subscriber."""
        self._subs.discard(q)

    async def _broadcast_loop(self) -> None:
        period = 1.0 / self.s.params["TELEMETRY_HZ"]
        while True:
            await asyncio.sleep(period)
            snap = self.snapshot()
            for q in list(self._subs):
                if q.full():
                    with contextlib.suppress(asyncio.QueueEmpty):
                        q.get_nowait()  # drop the oldest: a slow client never sees old data
                q.put_nowait(snap)
