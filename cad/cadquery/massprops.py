"""Mass properties of CadQuery/OCC solids in the SPEC section 6 report units.

Invariant: every number here is measured from the solid with OCC's GProp (volume
properties), never estimated.  Units: volume mm^3, density g/cm^3, mass g, lengths mm,
inertia kg*m^2 about the part's own centre of mass in the standard tensor convention
(I_xy = -sum(m x y)); the OCC matrix of inertia is translation-invariant (about the COM)
and uses the same sign convention (verified on an asymmetric test body).
"""
from __future__ import annotations

import math
from typing import Sequence

import cadquery as cq
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

Matrix = list[list[float]]

# unit-density OCC inertia is in mm^5; x rho[g/cm^3] / 1000 -> g*mm^2; x 1e-3 x 1e-6 -> kg*m^2
_MM5_RHO_TO_KG_M2 = 1e-12


def _zeros() -> Matrix:
    return [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]


GPROP_EPS = 1e-7   # v0.9 fix round: adaptive (precise) integration for precise_volume - the default call under-reports the
                   # VTOL propulsion module by 2154 mm^3 (0.7 %, geometry reviewer; its STL and a 1e-7 GProp agree), other
                   # parts by < 1 mm^3.  volume_properties keeps the default call so every historical report entry stays
                   # bit-identical (the pinned v0.4 / v0.6.2 entries); the STL export check and the savings use precise_volume


def precise_volume(shape: cq.Shape) -> float:
    """Volume (mm^3) by adaptive OCC integration to GPROP_EPS (v0.9 fix round)."""
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape.wrapped, props, GPROP_EPS, False)
    return float(props.Mass())


def volume_properties(shape: cq.Shape) -> tuple[float, tuple[float, float, float], Matrix]:
    """(volume mm^3, centre of mass mm, unit-density inertia about the COM in mm^5)."""
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape.wrapped, props)
    vol = float(props.Mass())
    c = props.CentreOfMass()
    m = props.MatrixOfInertia()
    inertia = [[float(m.Value(i, j)) for j in (1, 2, 3)] for i in (1, 2, 3)]
    return vol, (float(c.X()), float(c.Y()), float(c.Z())), inertia


def bbox(shape: cq.Shape) -> list[list[float]]:
    """Exact geometric bounding box (never the triangulation's, which OCC enlarges by the mesh
    deflection once a shape has been meshed for STL export)."""
    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape.wrapped, box, False, False)
    xmin, ymin, zmin, xmax, ymax, zmax = box.Get()
    return [[float(xmin), float(ymin), float(zmin)], [float(xmax), float(ymax), float(zmax)]]


def part_properties(shape: cq.Shape, density_g_cm3: float | None = None, mass_g: float | None = None) -> dict:
    """Report entry for a solid: either a density (printed) or a fixed mass (placeholder)."""
    if (density_g_cm3 is None) == (mass_g is None):
        raise ValueError("give exactly one of density_g_cm3 / mass_g")
    vol, com, inertia_unit = volume_properties(shape)
    if vol <= 0.0:
        raise ValueError("solid has no volume")
    if density_g_cm3 is None:
        density_g_cm3 = mass_g / (vol / 1000.0)
    else:
        mass_g = density_g_cm3 * vol / 1000.0
    k = density_g_cm3 * _MM5_RHO_TO_KG_M2
    inertia = [[inertia_unit[i][j] * k for j in range(3)] for i in range(3)]
    return {
        "volume_mm3": vol,
        "density_g_cm3": float(density_g_cm3),
        "mass_g": float(mass_g),
        "bbox_mm": bbox(shape),
        "com_mm": [com[0], com[1], com[2]],
        "inertia_about_com_kg_m2": inertia,
    }


def lumped_properties(mass_g: float, point_mm: Sequence[float]) -> dict:
    p = [float(v) for v in point_mm]
    return {
        "volume_mm3": 0.0,
        "density_g_cm3": 0.0,
        "mass_g": float(mass_g),
        "bbox_mm": [list(p), list(p)],
        "com_mm": list(p),
        "inertia_about_com_kg_m2": _zeros(),
    }


def _parallel_axis(mass_g: float, d_mm: Sequence[float]) -> Matrix:
    """m (|d|^2 I - d d^T) in kg*m^2 for a mass in g and an offset in mm."""
    m = mass_g * 1e-3
    d = [v * 1e-3 for v in d_mm]
    d2 = sum(v * v for v in d)
    return [[m * ((d2 if i == j else 0.0) - d[i] * d[j]) for j in range(3)] for i in range(3)]


def combine(entries: Sequence[dict]) -> dict:
    """Total mass, COM and inertia about the total COM of several report entries."""
    mass = sum(e["mass_g"] for e in entries)
    if mass <= 0.0:
        return {"mass_g": 0.0, "com_mm": [0.0, 0.0, 0.0], "inertia_about_com_kg_m2": _zeros()}
    com = [sum(e["mass_g"] * e["com_mm"][k] for e in entries) / mass for k in range(3)]
    total = _zeros()
    for e in entries:
        d = [e["com_mm"][k] - com[k] for k in range(3)]
        shift = _parallel_axis(e["mass_g"], d)
        for i in range(3):
            for j in range(3):
                total[i][j] += e["inertia_about_com_kg_m2"][i][j] + shift[i][j]
    return {"mass_g": float(mass), "com_mm": com, "inertia_about_com_kg_m2": total}


def radius_of_gyration_mm(entry: dict, axis: int) -> float:
    m = entry["mass_g"] * 1e-3
    if m <= 0.0:
        return 0.0
    return math.sqrt(entry["inertia_about_com_kg_m2"][axis][axis] / m) * 1e3
