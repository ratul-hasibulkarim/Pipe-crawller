"""Part record: geometry + manufacturing metadata used by BOM, mass report and checks."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import cadquery as cq


@dataclass
class Part:
    """A generated part.

    solid      geometry in the part's own frame (exported as-is to STEP/STL)
    module     A_body ... E_accessories, or 'purchased'
    material   key into params['materials'] (made parts) or free text (purchased)
    qty        quantity per robot
    fasteners  {"M3x8 SHCS": n, "M3 heat-set insert": n, ...} per part instance
    bearing_bores  bore axes (part frame) that hold bearings; must print vertical (FDM rule)
    print_rot  (axis, deg) rotation that puts the part on the build plate (FDM parts only)
    mass_kg    fixed mass for purchased parts (else volume x density)
    cost       unit cost override (purchased)
    """

    name: str
    module: str
    solid: cq.Workplane
    material: str
    qty: int = 1
    process: str = ""
    fasteners: dict[str, int] = field(default_factory=dict)
    bearing_bores: list[tuple[float, float, float]] = field(default_factory=list)
    print_rot: tuple[tuple[float, float, float], float] | None = None
    mass_kg: float | None = None
    cost: float | None = None
    purchased: bool = False
    color: tuple[float, float, float] = (0.7, 0.7, 0.72)
    note: str = ""
    meta: dict[str, Any] = field(default_factory=dict)

    @property
    def shape(self) -> cq.Shape:
        """Single OCC shape of the part."""
        vals = self.solid.vals()
        if len(vals) == 1:
            return vals[0]
        return cq.Compound.makeCompound(vals)

    def volume_mm3(self) -> float:
        """Solid volume."""
        return self.shape.Volume()

    def unit_mass(self, materials: dict[str, Any]) -> float:
        """Mass of one instance in kg."""
        if self.mass_kg is not None:
            return self.mass_kg
        return self.volume_mm3() * 1e-3 * materials[self.material]["density"] / 1000


# Colour palette by role (used by renders and the STEP assembly).
COLORS = {
    "shell": (0.20, 0.36, 0.55),
    "alu": (0.75, 0.76, 0.78),
    "bronze": (0.80, 0.55, 0.25),
    "steel": (0.55, 0.56, 0.58),
    "print": (0.95, 0.55, 0.15),
    "rubber": (0.12, 0.12, 0.12),
    "motor": (0.25, 0.25, 0.27),
    "pcb": (0.10, 0.50, 0.25),
    "clear": (0.70, 0.85, 0.95),
    "led": (0.98, 0.95, 0.70),
    "belt": (0.15, 0.15, 0.15),
    "boot": (0.30, 0.30, 0.30),
}
