"""Run recorder: every run gets a folder with video.mp4, telemetry.csv, events.jsonl, meta.json,
snapshots/, and a row in recordings.db (SQLite index)."""

from __future__ import annotations

import csv
import json
import sqlite3
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np

CSV_FIELDS = [
    "t_wall",
    "t_ms",
    "state",
    "faults",
    "est_diameter_mm",
    "lever_angle_deg",
    "grip0",
    "grip1",
    "grip2",
    "axis_offset_mm",
    "linear_mm_s",
    "distance_m",
    "odom_confidence",
    "pitch",
    "roll",
    "yaw",
    "pipe_orientation_deg",
    "cur0",
    "cur1",
    "cur2",
    "cur3",
    "driver_temp_c",
    "battery_v",
    "battery_pct",
    "battery_current_a",
    "supply",
    "temp_c",
    "humidity_pct",
    "pressure_kpa",
    "gas_ppm",
    "tof_front_mm",
    "tof_rear_mm",
    "pan_deg",
    "tilt_deg",
    "white_pct",
    "ir_pct",
]


def flat(t: dict[str, Any]) -> dict[str, Any]:
    """Telemetry JSON -> one CSV row."""
    imu, bat, env, gim, li = (t.get(k) or {} for k in ("imu", "battery", "env", "gimbal", "light"))
    g, c = t.get("grip_force_n") or [0, 0, 0], t.get("motor_current_a") or [0, 0, 0, 0]
    return {
        "t_wall": round(time.time(), 3),
        "t_ms": t.get("t_ms"),
        "state": t.get("state"),
        "faults": "|".join(t.get("faults") or []),
        "est_diameter_mm": t.get("est_diameter_mm"),
        "lever_angle_deg": t.get("lever_angle_deg"),
        "grip0": g[0],
        "grip1": g[1],
        "grip2": g[2],
        "axis_offset_mm": t.get("axis_offset_mm"),
        "linear_mm_s": t.get("linear_mm_s"),
        "distance_m": t.get("distance_m"),
        "odom_confidence": t.get("odom_confidence"),
        "pitch": imu.get("pitch"),
        "roll": imu.get("roll"),
        "yaw": imu.get("yaw"),
        "pipe_orientation_deg": t.get("pipe_orientation_deg"),
        "cur0": c[0],
        "cur1": c[1],
        "cur2": c[2],
        "cur3": c[3],
        "driver_temp_c": t.get("driver_temp_c"),
        "battery_v": bat.get("v"),
        "battery_pct": bat.get("pct"),
        "battery_current_a": bat.get("current_a"),
        "supply": bat.get("supply"),
        "temp_c": env.get("temp_c"),
        "humidity_pct": env.get("humidity_pct"),
        "pressure_kpa": env.get("pressure_kpa"),
        "gas_ppm": env.get("gas_ppm"),
        "tof_front_mm": t.get("tof_front_mm"),
        "tof_rear_mm": t.get("tof_rear_mm"),
        "pan_deg": gim.get("pan_deg"),
        "tilt_deg": gim.get("tilt_deg"),
        "white_pct": li.get("white_pct"),
        "ir_pct": li.get("ir_pct"),
    }


class Recorder:
    """At most one active run."""

    def __init__(self, root: Path) -> None:
        """Index lives in root/recordings.db."""
        self.root = root
        root.mkdir(parents=True, exist_ok=True)
        self.db_path = root / "recordings.db"
        with sqlite3.connect(self.db_path) as db:
            db.execute(
                "CREATE TABLE IF NOT EXISTS runs (id TEXT PRIMARY KEY, started REAL, ended REAL, operator TEXT, pipe_id_range TEXT,"
                " tether_m REAL, distance_m REAL, frames INTEGER, events INTEGER, note TEXT)"
            )
        self.run_id: str | None = None
        self._lock = threading.Lock()
        self._csv = self._csvw = self._ev = None
        self._video = self._vstream = None
        self.frames = 0
        self.events = 0
        self.last_distance = 0.0

    @property
    def active(self) -> bool:
        """Recording?"""
        return self.run_id is not None

    def start(self, meta: dict[str, Any], w: int, h: int, fps: int) -> str:
        """Open a run folder and writers."""
        import av

        with self._lock:
            if self.run_id:
                return self.run_id
            rid = time.strftime("%Y%m%d-%H%M%S")
            d = self.root / rid
            (d / "snapshots").mkdir(parents=True, exist_ok=True)
            meta = {**meta, "id": rid, "started": time.time()}
            (d / "meta.json").write_text(json.dumps(meta, indent=2))
            self._csv = open(d / "telemetry.csv", "w", newline="")  # noqa: SIM115 - closed in stop()
            self._csvw = csv.DictWriter(self._csv, fieldnames=CSV_FIELDS)
            self._csvw.writeheader()
            self._ev = open(d / "events.jsonl", "w")  # noqa: SIM115
            self._video = av.open(str(d / "video.mp4"), "w")
            self._vstream = self._video.add_stream("libx264", rate=fps)
            self._vstream.width, self._vstream.height = w, h
            self._vstream.pix_fmt = "yuv420p"
            self._vstream.options = {"preset": "veryfast", "tune": "zerolatency"}
            self.frames = self.events = 0
            self.run_id = rid
            with sqlite3.connect(self.db_path) as db:
                db.execute(
                    "INSERT INTO runs (id, started, operator, pipe_id_range, tether_m, frames, events) VALUES (?,?,?,?,?,0,0)",
                    (rid, meta["started"], meta.get("operator", ""), meta.get("pipe_id_range", ""), meta.get("tether_m", 0)),
                )
            return rid

    def on_message(self, j: dict[str, Any]) -> None:
        """Hub listener: telemetry rows + events."""
        if not self.run_id:
            return
        with self._lock:
            if j.get("type") == "TELEMETRY" and self._csvw:
                self._csvw.writerow(flat(j))
                self.last_distance = float(j.get("distance_m") or 0.0)
            elif j.get("type") in ("EVENT", "ACK") and self._ev:
                self._ev.write(json.dumps({**j, "t_wall": time.time()}) + "\n")
                self.events += 1

    def mark(self, label: str, data: dict[str, Any] | None = None) -> None:
        """Operator event marker."""
        if not self.run_id or not self._ev:
            return
        with self._lock:
            self._ev.write(json.dumps({"type": "MARK", "label": label, "t_wall": time.time(), **(data or {})}) + "\n")
            self.events += 1

    def add_frame(self, frame: np.ndarray) -> None:
        """Encode one frame."""
        import av

        if not self.run_id or self._video is None:
            return
        with self._lock:
            vf = av.VideoFrame.from_ndarray(frame, format="rgb24")
            for pkt in self._vstream.encode(vf):
                self._video.mux(pkt)
            self.frames += 1

    def snapshot(self, jpeg: bytes, telemetry: dict[str, Any]) -> str | None:
        """Save a JPEG + its telemetry next to the run."""
        if not self.run_id:
            return None
        name = f"snap_{int(time.time() * 1000)}.jpg"
        d = self.root / self.run_id / "snapshots"
        (d / name).write_bytes(jpeg)
        (d / (name + ".json")).write_text(json.dumps(telemetry))
        self.mark("snapshot", {"file": name, "distance_m": telemetry.get("distance_m")})
        return name

    def stop(self) -> str | None:
        """Close writers and update the index."""
        with self._lock:
            rid = self.run_id
            if not rid:
                return None
            if self._video is not None:
                for pkt in self._vstream.encode():
                    self._video.mux(pkt)
                self._video.close()
            for f in (self._csv, self._ev):
                if f:
                    f.close()
            with sqlite3.connect(self.db_path) as db:
                db.execute(
                    "UPDATE runs SET ended=?, distance_m=?, frames=?, events=? WHERE id=?", (time.time(), self.last_distance, self.frames, self.events, rid)
                )
            self.run_id = None
            self._video = self._vstream = self._csv = self._csvw = self._ev = None
            return rid

    def list(self) -> list[dict[str, Any]]:
        """All runs, newest first."""
        with sqlite3.connect(self.db_path) as db:
            db.row_factory = sqlite3.Row
            return [dict(r) for r in db.execute("SELECT * FROM runs ORDER BY started DESC")]

    def path(self, rid: str) -> Path | None:
        """Run folder (validated id)."""
        if not rid.replace("-", "").isdigit():
            return None
        p = self.root / rid
        return p if p.is_dir() else None
