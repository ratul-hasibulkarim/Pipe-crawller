"""Shared fixtures: a fake MCU (in-process TCP server) and the real SIL binary when built."""

from __future__ import annotations

import asyncio
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from pi.app import protocol_gen as P  # noqa: E402


def free_port() -> int:
    """Unused TCP port."""
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class FakeMcu:
    """Minimal MCU: ACKs every command with `status`, answers PING, sends telemetry at 20 Hz."""

    def __init__(self) -> None:
        """Pick a port."""
        self.received: list = []
        self.status = 0
        self.state = P.State.HOLDING
        self.server: asyncio.base_events.Server | None = None
        self.port = free_port()
        self.silent = False

    async def start(self) -> None:
        """Listen."""
        self.server = await asyncio.start_server(self._client, "127.0.0.1", self.port)

    async def stop(self) -> None:
        """Close."""
        if self.server:
            self.server.close()

    async def _client(self, r: asyncio.StreamReader, w: asyncio.StreamWriter) -> None:
        dec = P.FrameDecoder()

        async def tel() -> None:
            while True:
                await asyncio.sleep(0.05)
                if not self.silent:
                    w.write(P.encode(P.TelemetryMsg(state=int(self.state), est_diameter_mm=210.0, distance_m=1.5)))

        task = asyncio.create_task(tel())
        try:
            while data := await r.read(1024):
                for m in dec.feed(data):
                    self.received.append(m)
                    if isinstance(m, P.PingMsg):
                        w.write(P.encode(P.PongMsg(seq=m.seq, t_ms=m.t_ms)))
                    elif hasattr(m, "seq"):
                        st = P.AckStatus.ESTOPPED if isinstance(m, P.EstopMsg) else self.status
                        w.write(P.encode(P.AckMsg(seq=m.seq, cmd=m.TYPE, status=int(st))))
        except ConnectionError:
            pass
        finally:
            task.cancel()


@pytest.fixture
async def fake_mcu():
    """Running fake MCU."""
    m = FakeMcu()
    await m.start()
    yield m
    await m.stop()


@pytest.fixture
def sil_binary() -> Path:
    """Path to the size-M SIL binary (skips when not built)."""
    b = ROOT / "build" / "sil" / "pipebot_sil_M"
    if not b.exists():
        pytest.skip("SIL not built (make sim)")
    return b


@pytest.fixture
def sil_server(sil_binary: Path, tmp_path: Path):
    """SIL in real-time TCP mode: gripped in a vertical 200 mm pipe."""
    port = free_port()
    scen = tmp_path / "scen.json"
    scen.write_text('{"plant":{"sections":[{"start_m":0,"end_m":50,"id_mm":200,"incline_deg":90}],"x0_m":1.0,"start_gripped":true}}')
    p = subprocess.Popen([str(sil_binary), "--scenario", str(scen), "--tcp", str(port)], stdout=subprocess.DEVNULL)
    time.sleep(0.4)
    yield port
    p.terminate()
    p.wait(5)
