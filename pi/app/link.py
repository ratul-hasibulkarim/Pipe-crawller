"""Byte transport to the MCU: USB/UART serial on the robot, TCP to the SIL simulator."""

from __future__ import annotations

import asyncio
import contextlib
import logging
from collections.abc import Awaitable, Callable
from urllib.parse import parse_qs, urlparse

log = logging.getLogger("pipebot.link")


class Link:
    """Reconnecting byte stream. `on_bytes` is called for everything received."""

    def __init__(self, url: str, on_bytes: Callable[[bytes], Awaitable[None] | None]) -> None:
        """Remember the URL; call start() to connect."""
        self.url = url
        self.on_bytes = on_bytes
        self.connected = asyncio.Event()
        self._writer: asyncio.StreamWriter | None = None
        self._serial = None
        self._task: asyncio.Task | None = None
        self.bytes_rx = 0
        self.bytes_tx = 0
        self.reconnects = 0

    async def start(self) -> None:
        """Start the connect/read loop."""
        self._task = asyncio.create_task(self._run())

    async def stop(self) -> None:
        """Stop and close."""
        if self._task:
            self._task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._task
        self._close()

    def _close(self) -> None:
        self.connected.clear()
        if self._writer:
            self._writer.close()
        self._writer = None
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    async def write(self, data: bytes) -> bool:
        """Send bytes; False if not connected."""
        if not self.connected.is_set():
            return False
        try:
            if self._writer is not None:
                self._writer.write(data)
                await self._writer.drain()
            elif self._serial is not None:
                await asyncio.to_thread(self._serial.write, data)
            self.bytes_tx += len(data)
            return True
        except (OSError, ConnectionError):
            self._close()
            return False

    async def _deliver(self, data: bytes) -> None:
        self.bytes_rx += len(data)
        r = self.on_bytes(data)
        if asyncio.iscoroutine(r):
            await r

    async def _run(self) -> None:
        u = urlparse(self.url)
        while True:
            try:
                if u.scheme == "tcp":
                    reader, self._writer = await asyncio.open_connection(u.hostname, u.port)
                    self.connected.set()
                    log.info("connected to %s", self.url)
                    while True:
                        data = await reader.read(4096)
                        if not data:
                            raise ConnectionError("closed")
                        await self._deliver(data)
                elif u.scheme == "serial":
                    import serial  # pyserial

                    baud = int(parse_qs(u.query).get("baud", ["921600"])[0])
                    self._serial = serial.Serial(u.path, baud, timeout=0.02)
                    self.connected.set()
                    log.info("opened %s", self.url)
                    while True:
                        data = await asyncio.to_thread(self._serial.read, 4096)
                        if data:
                            await self._deliver(data)
                else:
                    raise ValueError(f"unsupported link url {self.url}")
            except asyncio.CancelledError:
                raise
            except (OSError, ConnectionError, Exception) as e:  # noqa: BLE001 - keep reconnecting whatever happened
                log.warning("link %s down: %s", self.url, e)
            self._close()
            self.reconnects += 1
            await asyncio.sleep(0.5)
