"""Headless matplotlib renderer for parts and assemblies (PNG thumbnails, views, exploded)."""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

import cadquery as cq
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

LIGHT = np.array([0.4, -0.5, 0.8]) / np.linalg.norm([0.4, -0.5, 0.8])


def tris(shape: cq.Shape, tol: float) -> np.ndarray:
    """Triangles (n, 3, 3) of a shape."""
    verts, faces = shape.tessellate(tol, 0.5)
    if not faces:
        return np.zeros((0, 3, 3))
    v = np.array([(p.x, p.y, p.z) for p in verts])
    return v[np.array(faces)]


def shade(t: np.ndarray, rgb: Sequence[float], alpha: float) -> np.ndarray:
    """Per-triangle Lambert shading."""
    n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    ln[ln == 0] = 1
    k = 0.35 + 0.65 * np.abs((n / ln) @ LIGHT)
    c = np.clip(np.outer(k, rgb), 0, 1)
    return np.hstack([c, np.full((len(c), 1), alpha)])


def render(items: Sequence[tuple[cq.Shape, Sequence[float], float]], path: Path, title: str = "",
           elev: float = 22, azim: float = -60, tol: float = 0.4, size: tuple[float, float] = (8, 6)) -> None:
    """Render (shape, rgb, alpha) items to a PNG."""
    fig = plt.figure(figsize=size, dpi=110)
    ax = fig.add_subplot(111, projection="3d")
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for shp, rgb, alpha in items:
        t = tris(shp, tol)
        if len(t) == 0:
            continue
        pc = Poly3DCollection(t, facecolors=shade(t, rgb, alpha), edgecolors="none", linewidths=0)
        ax.add_collection3d(pc)
        lo = np.minimum(lo, t.reshape(-1, 3).min(0))
        hi = np.maximum(hi, t.reshape(-1, 3).max(0))
    c, r = (lo + hi) / 2, (hi - lo).max() / 2
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()
    if title:
        ax.set_title(title, fontsize=10)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, facecolor="white")
    plt.close(fig)
