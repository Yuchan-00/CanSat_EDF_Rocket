"""Barrowman centre-of-pressure estimates (nose, conical transition, trapezoid fins).

Invariant: pure functions of geometry.  Nothing in this module reads
``params.json``; every length comes in as an argument.  Axial positions follow
the Barrowman convention - measured *aft from the nose tip* - and the caller
converts them to vehicle Z (``z = z_nose_tip - x``).  Normal-force slopes are
per radian, referenced to the area ``pi d_ref^2 / 4`` of the body reference
diameter ``d_ref``.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Iterable


@dataclass(frozen=True)
class CpComponent:
    """One Barrowman component: normal-force slope and CP position from the tip."""

    name: str
    cna_per_rad: float
    x_cp_from_tip_mm: float


def nose_from_volume(length_mm: float, base_radius_mm: float, volume_mm3: float) -> CpComponent:
    """Nose of any revolved profile: C_Na = 2, X = L - V / (pi R^2) (Barrowman exact).

    Cone -> 2L/3, LD-Haack (von Karman) -> L/2, ogive -> ~0.466 L.
    """
    base_area = math.pi * base_radius_mm**2
    return CpComponent("nose", 2.0, length_mm - volume_mm3 / base_area)


def nose_volume(r_of_x: Callable[[float], float], length_mm: float, n: int = 4000) -> float:
    """Volume (mm^3) of a revolved profile r(x), x from the base (0) to the tip (L)."""
    dx = length_mm / n
    vol = 0.0
    for i in range(n):
        r = r_of_x((i + 0.5) * dx)
        vol += math.pi * r * r * dx
    return vol


def transition(d_fore_mm: float, d_aft_mm: float, length_mm: float, d_ref_mm: float,
               x_front_from_tip_mm: float, name: str = "transition") -> CpComponent:
    """Conical shoulder (d_aft > d_fore) or boat-tail (d_aft < d_fore).

    C_Na = 2 [(d_aft/d_ref)^2 - (d_fore/d_ref)^2];
    X = X_front + (L/3) [1 + (1 - d_fore/d_aft) / (1 - (d_fore/d_aft)^2)].
    """
    cna = 2.0 * ((d_aft_mm / d_ref_mm) ** 2 - (d_fore_mm / d_ref_mm) ** 2)
    ratio = d_fore_mm / d_aft_mm
    if abs(1.0 - ratio * ratio) < 1e-12:
        x_local = length_mm / 2.0
    else:
        x_local = (length_mm / 3.0) * (1.0 + (1.0 - ratio) / (1.0 - ratio * ratio))
    return CpComponent(name, cna, x_front_from_tip_mm + x_local)


def trapezoid_fins(n_fins: int, semispan_mm: float, root_chord_mm: float, tip_chord_mm: float,
                   le_sweep_mm: float, body_radius_at_root_mm: float, d_ref_mm: float,
                   x_root_le_from_tip_mm: float, name: str = "fins") -> CpComponent:
    """N trapezoid fins (Barrowman, with the body-interference factor 1 + R/(s+R)).

    ``le_sweep_mm`` is the axial distance from the root leading edge to the tip
    leading edge (positive aft).  Mid-chord line length
    l = sqrt(s^2 + (x_r + b/2 - a/2)^2).
    """
    a, b, s = root_chord_mm, tip_chord_mm, semispan_mm
    mid_chord_dx = le_sweep_mm + b / 2.0 - a / 2.0
    l_mid = math.hypot(s, mid_chord_dx)
    interference = 1.0 + body_radius_at_root_mm / (s + body_radius_at_root_mm)
    cna = interference * (4.0 * n_fins * (s / d_ref_mm) ** 2) / (1.0 + math.sqrt(1.0 + (2.0 * l_mid / (a + b)) ** 2))
    x_local = le_sweep_mm * (a + 2.0 * b) / (3.0 * (a + b)) + (a + b - a * b / (a + b)) / 6.0
    return CpComponent(name, cna, x_root_le_from_tip_mm + x_local)


def combine(components: Iterable[CpComponent]) -> tuple[float, float]:
    """Total C_Na and the moment-weighted CP position (from the tip)."""
    comps = list(components)
    cna = sum(c.cna_per_rad for c in comps)
    if abs(cna) < 1e-12:
        raise ValueError("total C_Na is zero - cannot locate a centre of pressure")
    x = sum(c.cna_per_rad * c.x_cp_from_tip_mm for c in comps) / cna
    return cna, x
