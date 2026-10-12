#!/usr/bin/env python3
"""Plot a simulator run (the CSV written by pipebot_sil_<size> --out run.csv).

    python tools/plot_run.py tests/acceptance/out/M/T9.csv -o t9.png

Panels: position and speed, arm forces (measured vs true), lever angle and diameter,
axis offset, bus voltage / battery, motor-driver temperature, state and fault timeline.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

STATES = ["IDLE", "EXPANDING", "GRIPPED", "CRAWLING", "HOLDING", "RETRACTING", "FAULT", "LOW_POWER_SAFE"]


def load(path: Path) -> dict[str, list[float | str]]:
    """CSV -> column lists (numbers where they parse)."""
    with open(path) as f:
        rows = list(csv.DictReader(f))
    cols: dict[str, list[float | str]] = {k: [] for k in rows[0]} if rows else {}
    for r in rows:
        for k, v in r.items():
            try:
                cols[k].append(float(v))
            except ValueError:
                cols[k].append(v)
    return cols


def plot(path: Path, out: Path) -> None:
    """Write a multi-panel PNG for one run."""
    c = load(path)
    t = c["t"]
    fig, ax = plt.subplots(7, 1, figsize=(11, 15), sharex=True)
    ax[0].plot(t, c["x_m"], label="true position x (m)")
    ax[0].plot(t, c["distance_m"], "--", label="odometry (m)")
    a0 = ax[0].twinx()
    a0.plot(t, [v * 1000 for v in c["v_m_s"]], color="tab:gray", lw=0.7, label="true speed (mm/s)")
    a0.set_ylabel("mm/s")
    ax[0].legend(loc="upper left")
    for i, col in enumerate(("tab:red", "tab:green", "tab:blue")):
        ax[1].plot(t, c[f"grip{i}"], color=col, label=f"arm {'ABC'[i]} measured")
        ax[1].plot(t, c[f"f{i}_true"], color=col, ls=":", label=f"arm {'ABC'[i]} true")
    ax[1].set_ylabel("wall force (N)")
    ax[1].legend(ncol=3, fontsize=8)
    ax[2].plot(t, c["lever_deg"], label="lever angle (deg)")
    a2 = ax[2].twinx()
    a2.plot(t, c["est_diameter_mm"], color="tab:orange", label="estimated diameter (mm)")
    a2.plot(t, c["pipe_id_mm"], color="k", ls="--", lw=0.8, label="true pipe ID (mm)")
    a2.legend(loc="lower right", fontsize=8)
    ax[2].legend(loc="upper left", fontsize=8)
    ax[3].plot(t, c["axis_offset_mm"], label="estimated axis offset")
    ax[3].plot(t, [(x * x + y * y) ** 0.5 for x, y in zip(c["off_x_mm"], c["off_y_mm"], strict=True)], ":", label="true axis offset")
    ax[3].set_ylabel("mm")
    ax[3].legend(fontsize=8)
    ax[4].plot(t, c["bus_v"], label="bus V")
    ax[4].plot(t, [v * 100 for v in c["soc"]], label="battery SOC %")
    ax[4].legend(fontsize=8)
    ax[5].plot(t, c["driver_temp_true"], color="tab:red", label="hottest motor/driver (C)")
    ax[5].axhline(85, color="k", ls="--", lw=0.8)
    ax[5].legend(fontsize=8)
    ax[6].step(t, [STATES.index(s) if s in STATES else -1 for s in c["state"]], where="post", label="state")
    ax[6].set_yticks(range(len(STATES)), STATES, fontsize=7)
    ax6b = ax[6].twinx()
    ax6b.step(t, c["faults"], where="post", color="tab:red", lw=0.7)
    ax6b.set_ylabel("fault bits", color="tab:red")
    ax[6].set_xlabel("time (s)")
    fig.suptitle(path.name)
    fig.tight_layout()
    fig.savefig(out, dpi=90)
    plt.close(fig)


def main() -> None:
    """CLI."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", type=Path)
    ap.add_argument("-o", "--out", type=Path)
    a = ap.parse_args()
    out = a.out or a.csv.with_suffix(".png")
    plot(a.csv, out)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
