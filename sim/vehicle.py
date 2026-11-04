"""Vehicle mass properties and aerodynamic reference data for the simulation.

Mass-source priority (SPEC §9): ``cad/exports/cadquery_report.json`` ->
``cad/exports/fusion360_report.json`` -> ``sizing/sizing_report.json`` -> an
internal estimate built from ``cad/layout.py`` + ``params.json``.  An alt preset
reads the ``_<preset>``-suffixed files first; a report built for another preset
is skipped and one built from another params.json is flagged (``load_vehicle``).
A CAD report that lists failed components, or that is missing any component of
the internal estimate (the SPEC §6 list; ``launch_pad`` optional), is skipped: a
report that silently dropped a part is no evidence.  Every ``Vehicle`` carries
``mass_source`` so each report can state where its numbers came from.

Invariant: no dimension, mass or density is typed here.  Every number comes
from ``Layout`` (geometry), ``params.json`` (component masses, aero
coefficients) or one of the report files.  Units: SI internally (kg, m,
kg m^2); the CAD-report schema of SPEC §6 (g, mm) is used only at the file
boundary.  The v0.5 tab-mount features of the internal estimate are gated on
``Layout.tabs``; the ``lip_clamp`` estimate is the v0.4 one except the v0.5d
tray stop (two blocks in the avionics module, both presets).  No joint-screw
hole is modelled, so the v0.5d blind propulsion joint holes change nothing here.
v0.6.1 (both presets): the intake's windowed band is ``r_band_in`` .. body_r (``window_wall_t``) and its
struts are exact radial-sided sectors (``strut_angle_deg``).  v0.6.2: the window gussets under the truncated-gable
lower ends (Layout's exact volume per window at its centroid) and the ``radio_log`` placeholder (Layout box, params
mass) join the estimate.  v0.6.3: the tray placeholder ``fc`` is ``nav`` (``electronics.nav``, same box) and the ``fcu``
placeholder (Layout box on the intake bulkhead, ``electronics.fcu`` mass) joins the estimate; the servo-lead lump follows
Layout (its bay legs now end on the FCU).

v0.9 (VTOL hardware round): ``load_vehicle(config='vtol', power=...)`` loads the VTOL hardware configuration
(``configs.vtol``, SPEC 13) from ``cadquery_report<_preset>_vtol[_tether|_hybrid].json`` only (``load_vtol_vehicle``: the
internal estimate does not model the top cap / carrier / tether kit; the report must carry ``VTOL_REQUIRED_PARTS`` + the
power source's parts).  ``Vehicle`` carries ``hardware_config``, ``power_source`` and the component names; ``has_chute``,
``tether_attach_body_m`` (the top-cap eye bolt), ``gps_antenna_mm`` (rocket pod or top-cap patch) and
``footprint_radius_m`` (the solid-fin footprint: the square through the fin tips) come from the Layout.  ``compute_aero``
accepts the noseless VTOL layout (nose term 0 at the top cap).

simfix (2026-11-04): with ``propulsion_module.tab_mount`` 'clock_keyways' (``Layout.clock_mount``, wire09c) the estimate
models the clock mount the solid has (``_clock_mount_solids``: keyways round the circle, floor pads, servo-lead column
guide walls, the lead-passage blister + keel and the passage cut as exact plan-polygon / annulus intersections in Z
slabs; no lugs, tab screws or 22 mm slot).  Invariant: the estimate's printed mass is its own volume x the slicer-table
density of the SAME solid, so every mount the CAD builds must be the mount the estimate models (the screw-mount estimate
on the clock solid was +8 % volume, +17.7 g, and cost test_sim_assist_off_limiter_cuts its I4 cut).
"""
from __future__ import annotations

import datetime as _dt
import hashlib
import json
import math
import os
import sys
from dataclasses import dataclass, field
from typing import Any, Callable

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.normpath(os.path.join(_HERE, ".."))
CAD_DIR = os.path.join(PROJECT_ROOT, "cad")
if CAD_DIR not in sys.path:
    sys.path.insert(0, CAD_DIR)
import layout as layout_mod  # noqa: E402  (cad/layout.py)

Layout = layout_mod.Layout

G0 = 9.80665  # m/s^2, standard gravity

CAD_REPORT_CANDIDATES = (
    os.path.join("cad", "exports", "cadquery_report.json"),
    os.path.join("cad", "exports", "fusion360_report.json"),
)
SIZING_REPORT = os.path.join("sizing", "sizing_report.json")


def preset_suffix(preset: str, default_preset: str) -> str:
    """File-name suffix of a preset's reports: '' for the default preset, '_<preset>' otherwise (the naming of
    cad/cadquery/build_rocket.py and sizing/sizing.py)."""
    return "" if preset == default_preset else f"_{preset}"


def report_candidates(suffix: str) -> list[tuple[str, str]]:
    """(kind, path relative to the project root) in priority order: each CAD generator's preset-suffixed
    report, then its plain one (used only when its ``preset`` matches), then the sizing report."""
    out: list[tuple[str, str]] = []
    for rel in CAD_REPORT_CANDIDATES:
        stem, ext = os.path.splitext(rel)
        out += [("cad", f"{stem}{s}{ext}") for s in dict.fromkeys((suffix, ""))]
    stem, ext = os.path.splitext(SIZING_REPORT)
    out.append(("sizing", f"{stem}{suffix}{ext}"))
    return out
EJECTED_PARTS = ("cansat_payload", "nose_cone", "chute_pack")
NON_VEHICLE_PARTS = ("launch_pad",)
# v0.9 VTOL hardware configuration (SPEC 13): the internal estimate does not model the top cap / carrier / tether kit, so
# a VTOL vehicle comes from its CAD report only; the report must carry at least these components (+ the power source's)
VTOL_REQUIRED_PARTS = ("propulsion_module", "intake_module", "avionics_module", "avionics_tray", "top_cap", "edf_unit",
                       "esc", "carrier", "motor_leads", "servo_leads", "vane_1", "vane_2", "vane_3", "vane_4",
                       "servo_vane_1", "servo_vane_2", "servo_vane_3", "servo_vane_4")
VTOL_POWER_PARTS = {"battery": ("battery",),
                    "tether": ("tether_power_lead", "tether_cap_bank", "tether_bus_clamp", "tether_xt90", "tether_eyebolt"),
                    "hybrid": ("tether_power_lead", "tether_cap_bank", "tether_bus_clamp", "tether_xt90", "tether_eyebolt",
                               "buffer_pack", "hybrid_diode_board")}
EDF_HUB_LEN_MM = 70.0   # SPEC §6: edf_unit hub cylinder from z_edf_housing_bottom to +70 (no params key)


# --------------------------------------------------------------------------- #
# data classes
# --------------------------------------------------------------------------- #
@dataclass
class MassProps:
    """Rigid-body mass properties: mass [kg], COM [m, vehicle frame], inertia
    tensor about the COM [kg m^2, vehicle axes]."""

    mass_kg: float
    com_m: np.ndarray
    inertia_kg_m2: np.ndarray

    def as_report(self) -> dict[str, Any]:
        return {
            "mass_g": self.mass_kg * 1e3,
            "com_mm": (self.com_m * 1e3).tolist(),
            "inertia_about_com_kg_m2": self.inertia_kg_m2.tolist(),
        }


@dataclass
class AeroRef:
    """Barrowman reference data.  ``z_*`` are heights above the exit plane [m];
    ``cn_alpha_*`` are per radian, referenced to ``a_ref_m2``."""

    a_ref_m2: float
    cd: float
    cn_alpha_nose: float
    z_cp_nose_m: float
    cn_alpha_fins: float
    z_cp_fins_m: float
    cn_alpha_body: float
    z_cp_body_m: float
    cn_alpha_total: float
    z_cp_m: float
    z_cp_nose_fins_m: float
    cn_alpha_nose_fins: float
    static_margin_cal: float          # (z_cg - z_cp_nose_fins) / body_od, full stack
    static_margin_total_cal: float    # same with body lift included

    def as_dict(self) -> dict[str, float]:
        return {k: float(v) for k, v in self.__dict__.items()}


@dataclass
class _Solid:
    """One simple solid in SI: mass (may be negative for a cavity), COM, inertia
    about its COM, volume, bbox in mm."""

    m: float
    c: np.ndarray
    I: np.ndarray
    V: float
    bbox: np.ndarray


@dataclass
class Part:
    name: str
    kind: str                      # printed | placeholder | lumped
    mass_g: float
    com_mm: np.ndarray
    inertia_about_com_kg_m2: np.ndarray
    volume_mm3: float
    density_g_cm3: float
    bbox_mm: np.ndarray

    def as_report(self) -> dict[str, Any]:
        return {
            "kind": self.kind,
            "volume_mm3": float(self.volume_mm3),
            "density_g_cm3": float(self.density_g_cm3),
            "mass_g": float(self.mass_g),
            "bbox_mm": self.bbox_mm.tolist(),
            "com_mm": self.com_mm.tolist(),
            "inertia_about_com_kg_m2": self.inertia_about_com_kg_m2.tolist(),
        }


# --------------------------------------------------------------------------- #
# inertia helpers (SI)
# --------------------------------------------------------------------------- #
def _parallel_axis(I_com: np.ndarray, m: float, d: np.ndarray) -> np.ndarray:
    """Inertia about a point displaced by ``d`` from the COM (Steiner)."""
    return I_com + m * (float(d @ d) * np.eye(3) - np.outer(d, d))


def _compose(solids: list[_Solid]) -> _Solid:
    """Sum solids (negative masses = cavities) into one rigid body."""
    m = sum(s.m for s in solids)
    if abs(m) < 1e-15:
        raise ValueError("composite solid has zero mass")
    c = sum(s.m * s.c for s in solids) / m
    I = np.zeros((3, 3))
    for s in solids:
        I += _parallel_axis(s.I, s.m, s.c - c)
    V = sum(s.V for s in solids)
    lo = np.min([s.bbox[0] for s in solids if s.m > 0], axis=0)
    hi = np.max([s.bbox[1] for s in solids if s.m > 0], axis=0)
    return _Solid(m, c, I, V, np.array([lo, hi]))


def _rot_z(theta_rad: float) -> np.ndarray:
    c, s = math.cos(theta_rad), math.sin(theta_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _tube(ro_mm: float, ri_mm: float, z0_mm: float, z1_mm: float, rho_kg_m3: float,
          cx_mm: float = 0.0, cy_mm: float = 0.0) -> _Solid:
    """Hollow (or solid, ``ri=0``) cylinder along +Z."""
    ro, ri, h = ro_mm * 1e-3, ri_mm * 1e-3, (z1_mm - z0_mm) * 1e-3
    V = math.pi * (ro * ro - ri * ri) * h
    m = rho_kg_m3 * V
    izz = 0.5 * m * (ro * ro + ri * ri)
    ixx = m / 12.0 * (3.0 * (ro * ro + ri * ri) + h * h)
    c = np.array([cx_mm, cy_mm, 0.5 * (z0_mm + z1_mm)]) * 1e-3
    bbox = np.array([[cx_mm - ro_mm, cy_mm - ro_mm, z0_mm], [cx_mm + ro_mm, cy_mm + ro_mm, z1_mm]])
    return _Solid(m, c, np.diag([ixx, ixx, izz]), V, bbox)


def _box_xyz(x: tuple[float, float], y: tuple[float, float], z: tuple[float, float],
             rho_kg_m3: float) -> _Solid:
    """Axis-aligned box given by mm ranges."""
    lx, ly, lz = (x[1] - x[0]) * 1e-3, (y[1] - y[0]) * 1e-3, (z[1] - z[0]) * 1e-3
    V = lx * ly * lz
    m = rho_kg_m3 * V
    I = m / 12.0 * np.diag([ly * ly + lz * lz, lx * lx + lz * lz, lx * lx + ly * ly])
    c = np.array([x[0] + x[1], y[0] + y[1], z[0] + z[1]]) * 0.5e-3
    bbox = np.array([[x[0], y[0], z[0]], [x[1], y[1], z[1]]])
    return _Solid(m, c, I, V, bbox)


def _box_polar(r_range: tuple[float, float], t_width: float, z_range: tuple[float, float],
               theta_deg: float, rho_kg_m3: float, v_offset: float = 0.0) -> _Solid:
    """Box aligned with the local (radial u, tangential v, Z) frame at azimuth
    ``theta_deg``; ``r_range`` along u, width along v (centred at ``v_offset``)."""
    lr, lt, lz = (r_range[1] - r_range[0]) * 1e-3, t_width * 1e-3, (z_range[1] - z_range[0]) * 1e-3
    V = lr * lt * lz
    m = rho_kg_m3 * V
    I_loc = m / 12.0 * np.diag([lt * lt + lz * lz, lr * lr + lz * lz, lr * lr + lt * lt])
    Q = _rot_z(math.radians(theta_deg))
    I = Q @ I_loc @ Q.T
    rc = 0.5 * (r_range[0] + r_range[1])
    c_loc = np.array([rc, v_offset, 0.5 * (z_range[0] + z_range[1])]) * 1e-3
    c = Q @ c_loc
    corners = np.array([[u, v, zz] for u in r_range for v in (v_offset - t_width / 2, v_offset + t_width / 2)
                        for zz in z_range])
    cw = (Q @ corners.T).T
    bbox = np.array([cw.min(axis=0), cw.max(axis=0)])
    return _Solid(m, c, I, V, bbox)


def _radial_cylinder(d_mm: float, r0_mm: float, r1_mm: float, z_mm: float, theta_deg: float,
                     rho_kg_m3: float, bore_d_mm: float = 0.0) -> _Solid:
    """Cylinder whose axis is radial (along u) at height ``z_mm``."""
    ro, ri, L = 0.5 * d_mm * 1e-3, 0.5 * bore_d_mm * 1e-3, (r1_mm - r0_mm) * 1e-3
    V = math.pi * (ro * ro - ri * ri) * L
    m = rho_kg_m3 * V
    i_axis = 0.5 * m * (ro * ro + ri * ri)
    i_perp = m / 12.0 * (3.0 * (ro * ro + ri * ri) + L * L)
    Q = _rot_z(math.radians(theta_deg))
    I = Q @ np.diag([i_axis, i_perp, i_perp]) @ Q.T
    c = Q @ (np.array([0.5 * (r0_mm + r1_mm), 0.0, z_mm]) * 1e-3)
    corners = np.array([[u, v, z_mm + w] for u in (r0_mm, r1_mm) for v in (-d_mm / 2, d_mm / 2)
                        for w in (-d_mm / 2, d_mm / 2)])
    cw = (Q @ corners.T).T
    return _Solid(m, c, I, V, np.array([cw.min(axis=0), cw.max(axis=0)]))


def _polygon_props(pts: np.ndarray) -> tuple[float, float, float, float, float, float]:
    """Area, centroid and centroidal second moments of a planar polygon
    (shoelace formulas).  ``pts`` = (N,2) vertices (x, y) in any orientation.
    Returns ``A, cx, cy, Ixx_c (int y^2 dA), Iyy_c (int x^2 dA), Ixy_c``."""
    x, y = pts[:, 0], pts[:, 1]
    x1, y1 = np.roll(x, -1), np.roll(y, -1)
    cross = x * y1 - x1 * y
    A = 0.5 * cross.sum()
    sgn = 1.0 if A >= 0 else -1.0
    A = abs(A)
    cx = sgn * ((x + x1) * cross).sum() / (6.0 * A)
    cy = sgn * ((y + y1) * cross).sum() / (6.0 * A)
    ixx = sgn * ((y * y + y * y1 + y1 * y1) * cross).sum() / 12.0
    iyy = sgn * ((x * x + x * x1 + x1 * x1) * cross).sum() / 12.0
    ixy = sgn * ((x * y1 + 2 * x * y + 2 * x1 * y1 + x1 * y) * cross).sum() / 24.0
    return A, cx, cy, ixx - A * cy * cy, iyy - A * cx * cx, ixy - A * cx * cy


def _plate_rz(poly_rz_mm: np.ndarray, t_mm: float, theta_deg: float, rho_kg_m3: float,
              v_offset: float = 0.0) -> _Solid:
    """Thin flat plate lying in the plane containing the vehicle axis at azimuth
    ``theta_deg`` (fins, webs).  ``poly_rz_mm`` = (N,2) planform vertices (r, z).  ``v_offset`` (mm, simfix
    2026-11-04): the plate's mid plane is shifted tangentially by that much (parallel to the axial plane)."""
    pts = poly_rz_mm * 1e-3
    t = t_mm * 1e-3
    A, rc, zc, Izz_a, Irr_a, Irz_a = _polygon_props(pts)  # Izz_a = int (z-zc)^2 dA etc.
    m = rho_kg_m3 * A * t
    sigma = rho_kg_m3 * t  # areal density
    # local frame: u (radial), v (tangential = plate normal), z
    I_uu = sigma * Izz_a + m * t * t / 12.0            # int (z^2 + v^2) dm
    I_vv = sigma * (Izz_a + Irr_a)                      # int (u^2 + z^2) dm
    I_zz = sigma * Irr_a + m * t * t / 12.0             # int (u^2 + v^2) dm
    I_uz = -sigma * Irz_a                               # product term
    I_loc = np.array([[I_uu, 0.0, I_uz], [0.0, I_vv, 0.0], [I_uz, 0.0, I_zz]])
    Q = _rot_z(math.radians(theta_deg))
    I = Q @ I_loc @ Q.T
    c = Q @ np.array([rc, v_offset * 1e-3, zc])
    corners = np.array([[r, v, z] for (r, z) in poly_rz_mm for v in (v_offset - t_mm / 2, v_offset + t_mm / 2)])
    cw = (Q @ corners.T).T
    return _Solid(m, c, I, A * t, np.array([cw.min(axis=0), cw.max(axis=0)]))


def _revolved_shell(z_edges_mm: np.ndarray, r_out_fn: Callable[[np.ndarray], np.ndarray],
                    r_in_fn: Callable[[np.ndarray], np.ndarray], rho_kg_m3: float) -> _Solid:
    """Body of revolution about Z integrated as a stack of thin tubes
    (vectorised).  ``r_*_fn`` take the slice-centre Z array (mm) -> radii (mm)."""
    z0, z1 = z_edges_mm[:-1], z_edges_mm[1:]
    zc = 0.5 * (z0 + z1)
    dz = (z1 - z0) * 1e-3
    ro = np.maximum(r_out_fn(zc), 0.0) * 1e-3
    ri = np.clip(r_in_fn(zc), 0.0, None) * 1e-3
    ri = np.minimum(ri, ro)
    dV = math.pi * (ro * ro - ri * ri) * dz
    dm = rho_kg_m3 * dV
    m = dm.sum()
    if m <= 0:
        raise ValueError("revolved shell has no volume")
    zcm = (dm * zc * 1e-3).sum() / m
    izz = (0.5 * dm * (ro * ro + ri * ri)).sum()
    ixx = (dm / 12.0 * (3.0 * (ro * ro + ri * ri) + dz * dz) + dm * (zc * 1e-3 - zcm) ** 2).sum()
    rmax = float(np.max(r_out_fn(zc)))
    bbox = np.array([[-rmax, -rmax, z_edges_mm[0]], [rmax, rmax, z_edges_mm[-1]]])
    return _Solid(float(m), np.array([0.0, 0.0, zcm]), np.diag([ixx, ixx, izz]), float(dV.sum()), bbox)


def _box_annulus_area(u0: float, u1: float, v_half: float, a: float, b: float = math.inf,
                      n: int = 400) -> tuple[float, float]:
    """Plan area [mm^2] and radial centroid [mm] of the radial box u in [u0, u1], |v| <= v_half (u >= 0)
    intersected with the annulus a <= r <= b (midpoint strips in v, each strip exact)."""
    v = -v_half + (np.arange(n) + 0.5) * (2.0 * v_half / n)
    dv = 2.0 * v_half / n
    lo = np.where(np.abs(v) < a, np.maximum(u0, np.sqrt(np.maximum(a * a - v * v, 0.0))), u0)
    hi = np.full(n, u1) if math.isinf(b) else np.minimum(u1, np.sqrt(np.maximum(b * b - v * v, 0.0)))
    ok = (hi > lo) & (np.abs(v) <= b)
    area = float(((hi - lo) * dv)[ok].sum())
    mom = float((0.5 * (hi * hi - lo * lo) * dv)[ok].sum())
    return area, (mom / area if area > 0.0 else 0.5 * (u0 + u1))


def _signed(s: _Solid) -> _Solid:
    """A cut (negative mass) with a negative volume.  The v0.4 solids of ``estimate_parts`` keep their cavity
    volumes positive (only ``volume_mm3`` is affected, never mass / COM / inertia); the v0.5 cuts are signed."""
    return _Solid(s.m, s.c, s.I, -abs(s.V), s.bbox) if s.m < 0 else s


def _clipped_box_polar(u0: float, u1: float, v_half: float, a: float, b: float, z_range: tuple[float, float],
                       theta_deg: float, rho_kg_m3: float) -> _Solid | None:
    """Radial box clipped to the annulus a <= r <= b: exact mass and radial centroid, box-like inertia
    (negative ``rho`` = cut, signed volume).  None when they do not overlap."""
    area, uc = _box_annulus_area(u0, u1, v_half, a, b)
    if area <= 0.0 or z_range[1] <= z_range[0]:
        return None
    du = area / (2.0 * v_half)
    return _signed(_box_polar((uc - du / 2.0, uc + du / 2.0), 2.0 * v_half, z_range, theta_deg, rho_kg_m3))


def _sector(ri_mm: float, ro_mm: float, a0_deg: float, a1_deg: float, z_range: tuple[float, float],
            rho_kg_m3: float) -> _Solid:
    """Annular sector ri..ro, a0..a1 (a1 > a0), Z range: exact mass, COM and full inertia tensor."""
    ri, ro = ri_mm * 1e-3, ro_mm * 1e-3
    a0, a1 = math.radians(a0_deg), math.radians(a1_deg)
    h = (z_range[1] - z_range[0]) * 1e-3
    area = 0.5 * (a1 - a0) * (ro * ro - ri * ri)
    m = rho_kg_m3 * area * h
    r3, r4 = (ro**3 - ri**3) / 3.0, (ro**4 - ri**4) / 4.0
    sx = r3 * (math.sin(a1) - math.sin(a0))                      # int x dA
    sy = r3 * (math.cos(a0) - math.cos(a1))                      # int y dA
    s2 = (math.sin(2 * a1) - math.sin(2 * a0)) / 4.0
    sxx = r4 * ((a1 - a0) / 2.0 + s2)                            # int x^2 dA
    syy = r4 * ((a1 - a0) / 2.0 - s2)
    sxy = r4 * (math.sin(a1) ** 2 - math.sin(a0) ** 2) / 2.0     # int x y dA
    sig = rho_kg_m3 * h
    zc = 0.5 * (z_range[0] + z_range[1]) * 1e-3
    c = np.array([sx / area, sy / area, zc])
    izz_h = m * h * h / 12.0
    I_o = np.array([[sig * syy + izz_h, -sig * sxy, 0.0],          # about (0, 0, zc)
                    [-sig * sxy, sig * sxx + izz_h, 0.0],
                    [0.0, 0.0, sig * (sxx + syy)]])
    d = c - np.array([0.0, 0.0, zc])
    I = I_o - m * (float(d @ d) * np.eye(3) - np.outer(d, d))     # move to the COM
    cardinal = [90.0 * k for k in range(math.ceil(a0_deg / 90.0), math.floor(a1_deg / 90.0) + 1)]
    angs = np.radians(np.array([a0_deg, a1_deg] + cardinal))      # arc extremes: ends + axis crossings
    xy = np.concatenate([np.c_[r * np.cos(angs), r * np.sin(angs)] for r in (ri_mm, ro_mm)])
    bbox = np.array([[*xy.min(axis=0), z_range[0]], [*xy.max(axis=0), z_range[1]]])
    return _Solid(m, c, I, area * h, bbox)


def _tangential_bore_in_annulus(r_s: float, rho: float, t0: float, t1: float, a: float, b: float,
                                n: int = 400) -> tuple[float, float, float]:
    """Volume [mm^3] and centroid (u_c, t_c) [mm] of a cylinder of radius ``rho`` on the tangential line u = r_s
    (Z constant), t in [t0, t1], intersected with the annulus a <= sqrt(u^2 + t^2) <= b.  Strips across u: the
    (u, Z) slice of the disc is exact, the t-interval exact at the strip's mid u."""
    edges = np.clip(-rho + np.arange(n + 1) * (2.0 * rho / n), -rho, rho)
    disc = edges * np.sqrt(np.maximum(rho * rho - edges * edges, 0.0)) + rho * rho * np.arcsin(edges / rho)
    area = np.diff(disc)
    u = r_s + 0.5 * (edges[:-1] + edges[1:])
    ok = u < b
    hi = np.sqrt(np.maximum(b * b - u * u, 0.0))
    lo = np.where(u < a, np.sqrt(np.maximum(a * a - u * u, 0.0)), 0.0)
    vol = mu = mt = 0.0
    for s0, s1 in ((lo, hi), (-hi, -lo)):
        x0, x1 = np.maximum(s0, t0), np.minimum(s1, t1)
        ln = np.where(ok & (x1 > x0), x1 - x0, 0.0)
        vol += float((area * ln).sum())
        mu += float((area * ln * u).sum())
        mt += float((area * np.where(ln > 0.0, 0.5 * (x1 * x1 - x0 * x0), 0.0)).sum())
    if vol <= 0.0:
        return 0.0, r_s, 0.5 * (t0 + t1)
    return vol, mu / vol, mt / vol


def _cylinder_between(d_mm: float, p0_mm: np.ndarray, p1_mm: np.ndarray, rho_kg_m3: float) -> _Solid:
    """Solid cylinder of diameter d on the segment p0 -> p1 (any direction)."""
    p0, p1 = np.asarray(p0_mm, dtype=float), np.asarray(p1_mm, dtype=float)
    axis = p1 - p0
    length = float(np.linalg.norm(axis)) * 1e-3
    a = axis / np.linalg.norm(axis)
    r = 0.5 * d_mm * 1e-3
    V = math.pi * r * r * length
    m = rho_kg_m3 * V
    i_ax, i_perp = 0.5 * m * r * r, m / 12.0 * (3.0 * r * r + length * length)
    I = i_perp * (np.eye(3) - np.outer(a, a)) + i_ax * np.outer(a, a)
    pts = np.array([p0, p1])
    bbox = np.array([pts.min(axis=0) - d_mm / 2, pts.max(axis=0) + d_mm / 2])
    return _signed(_Solid(m, 0.5 * (p0 + p1) * 1e-3, I, V, bbox))


def _point(mass_kg: float, xyz_mm: tuple[float, float, float]) -> _Solid:
    c = np.asarray(xyz_mm, dtype=float) * 1e-3
    return _Solid(mass_kg, c, np.zeros((3, 3)), 0.0, np.array([list(xyz_mm), list(xyz_mm)], dtype=float))


def _polar_xyz(r_mm: float, theta_deg: float, z_mm: float, v_mm: float = 0.0) -> np.ndarray:
    th = math.radians(theta_deg)
    return np.array([r_mm * math.cos(th) - v_mm * math.sin(th), r_mm * math.sin(th) + v_mm * math.cos(th), z_mm])


# --------------------------------------------------------------------------- #
# v0.5 tab mount (SPEC §3.7, §5.1): only called when Layout.tabs
# --------------------------------------------------------------------------- #
def _lug_sectors(L: Layout) -> list[tuple[float, float]]:
    """(a0, a1) deg (a1 > a0) of the screw lugs of each tab: [th - hs*block_theta_far, th + hs*block_theta_head]."""
    hs = L.tab_screw_head_side
    return [tuple(sorted((th - hs * L.block_theta_far, th + hs * L.block_theta_head))) for th in L.tab_thetas]


def _liner_layers(L: Layout) -> list[tuple[float, float, float, float]]:
    """Thin liner (SPEC §3.7): duct tube r [duct_r, liner_r_out] over [z_edf_shoulder, z_liner_top] and the part
    of the top ring r [duct_r, sleeve_r] over [z_liner_ring_bottom, z_liner_top] that neither the tube nor the
    sleeve holds (r [liner_r_out, edf_bore_r]); the cavity behind the tube stays empty."""
    return [(L.duct_r, L.liner_r_out, L.z_edf_shoulder, L.z_liner_top),
            (L.liner_r_out, L.edf_bore_r, L.z_liner_ring_bottom, L.z_liner_top)]


def _prop_layers(L: Layout, theta: float, lugs: bool = True) -> list[tuple[float, float, float, float]]:
    """Solid annular layers (r_in, r_out, z0, z1) of the EDF section at azimuth theta (webs: Layout keeps
    every cut clear of them); one lug layer per entry of ``boss_z_ranges`` when theta lies in a lug sector
    (``lugs`` False: none - the wire09c clock mount has no lugs)."""
    r_skin = L.body_r - L.cone_wall
    layers = [(L.edf_bore_r, L.sleeve_r, L.z_sleeve_bottom, L.z_prop_spigot_bottom),
              (L.edf_bore_r, L.spigot_od / 2, L.z_prop_spigot_bottom, L.z_prop_spigot_top),
              (r_skin, L.body_r, L.cone_z1, L.z_prop_spigot_bottom)] + _liner_layers(L)
    for a0, a1 in (_lug_sectors(L) if lugs else []):
        if (theta - a0) % 360.0 <= (a1 - a0):
            layers += [(L.block_r_in, r_skin, z0, z1) for z0, z1 in L.boss_z_ranges]
    return layers


def _radial_cut(layers: list[tuple[float, float, float, float]], u0: float, u1: float, v_half: float,
                z0: float, z1: float, theta: float, rho_kg_m3: float) -> list[_Solid]:
    """Negative solids of a radial box cut through every layer it overlaps."""
    out = []
    for a, b, za, zb in layers:
        s = _clipped_box_polar(u0, u1, v_half, a, b, (max(z0, za), min(z1, zb)), theta, -rho_kg_m3)
        if s is not None:
            out.append(s)
    return out


def _tab_screw_hole_cuts(L: Layout, rho: float) -> list[_Solid]:
    """Material each tab-screw hole segment (clearance [t_keyway, t_head], counterbore [t_head, t_cbore_end],
    tap [t_tap_end, -t_keyway]) removes from the sleeve / lug / skin layers at its tab; the counterbore removes
    nothing past the outer skin (``Layout.tab_screw_point`` applies the head side).  A hole that straddles a
    layer's Z edge is a contract change (ValueError)."""
    out: list[_Solid] = []
    for th in L.tab_thetas:
        layers = _prop_layers(L, th)
        e_r = np.array([math.cos(math.radians(th)), math.sin(math.radians(th)), 0.0])
        for z in L.tab_screw_zs:
            for d, t0, t1 in ((L.tab_screw_d_clear, L.tab_screw_t_keyway, L.tab_screw_t_head),
                              (L.tab_screw_d_cbore, L.tab_screw_t_head, L.tab_screw_t_cbore_end),
                              (L.tab_screw_d_tap, L.tab_screw_t_tap_end, -L.tab_screw_t_keyway)):
                r = d / 2
                for a, b, za, zb in layers:
                    if zb <= z - r or za >= z + r:
                        continue
                    if za > z - r + 1e-9 or zb < z + r - 1e-9:
                        raise ValueError(f"tab screw hole O{d} at z={z:.2f} crosses a layer's Z edge ({za:.2f}..{zb:.2f})")
                    vol, uc, tc = _tangential_bore_in_annulus(L.tab_screw_r, r, t0, t1, a, b)
                    if vol <= 0.0:
                        continue
                    le = vol / (math.pi * r * r)            # equivalent length: mass = rho pi r^2 le = rho vol
                    shift = (uc - L.tab_screw_r) * e_r       # centroid radius uc instead of the axis radius
                    out.append(_cylinder_between(d, np.array(L.tab_screw_point(th, z, tc - le / 2)) + shift,
                                                 np.array(L.tab_screw_point(th, z, tc + le / 2)) + shift, -rho))
    return out


def _tab_mount_solids(L: Layout, rho: float) -> list[_Solid]:
    """Cylindrical skin cone_z1..z_prop_spigot_bottom, thin liner (tube + top ring), screw lugs (one sector per
    tab and per ``boss_z_ranges`` entry, between sleeve and skin), keyways, tab-screw holes (exact material
    removed) and the two-box wire slot (SPEC §3.7).  wire09c clock mount (``Layout.clock_mount``): cylindrical skin,
    thin liner and ``_clock_mount_solids`` instead (no lugs, no tab screws, no 22 mm slot)."""
    r_skin = L.body_r - L.cone_wall
    sol = [_tube(L.body_r, r_skin, L.cone_z1, L.z_prop_spigot_bottom, rho)]
    sol += [_tube(b, a, za, zb, rho) for a, b, za, zb in _liner_layers(L)]
    if getattr(L, "clock_mount", False):
        return sol + _clock_mount_solids(L, rho)
    for a0, a1 in _lug_sectors(L):
        for z0, z1 in L.boss_z_ranges:
            sol.append(_sector(L.block_r_in, r_skin, a0, a1, (z0, z1), rho))
    for th in L.tab_thetas:
        sol += _radial_cut(_prop_layers(L, th), L.keyway_r_in, L.keyway_r_out, L.keyway_w / 2,
                           L.keyway_z0, L.keyway_z1, th, rho)
    sol += _tab_screw_hole_cuts(L, rho)
    layers = _prop_layers(L, L.wire_theta)
    sol += _radial_cut(layers, L.wire_slot_r_in, L.wire_slot_lower_r_out, L.wire_slot_w / 2,   # (a) short of the skin
                       L.wire_slot_z0, L.z_prop_spigot_bottom, L.wire_theta, rho)
    sol += _radial_cut(layers, L.wire_slot_r_in, L.wire_slot_r_out, L.wire_slot_w / 2,         # (b) through the spigot ring
                       L.z_prop_spigot_bottom, L.wire_slot_z1, L.wire_theta, rho)
    if getattr(L, "ml_low_cut_uz", None):
        # v0.9 lead-exit round (2026-10-29): lead notch under the housing rim + lower slot (Layout.ml_low_cut_uz) as two
        # radial boxes of the polygon's area (the 45 deg chamfer split at its mid height), up to the 22 mm box
        (u_in, z_b), (u_b, _), (u_o, z_ct), _, _ = L.ml_low_cut_uz
        hv, z_mid = L.ml_low_cut_w / 2, 0.5 * (L.ml_low_cut_uz[0][1] + L.ml_low_cut_uz[2][1])
        sol += _radial_cut(layers, u_in, u_b, hv, z_b, L.wire_slot_z0, L.wire_theta, rho)
        sol += _radial_cut(layers, u_b, u_o, hv, z_mid, L.wire_slot_z0, L.wire_theta, rho)
    return sol


# --------------------------------------------------------------------------- #
# wire09c clock mount (propulsion_module.tab_mount 'clock_keyways', Layout.clock_mount) - simfix 2026-11-04.
# Until this round the estimate kept the SCREW mount here (lugs, two keyways, tab screws, the 22 mm slot) while the solid
# had moved to the clock mount: the slicer-table density (part grams / CLOCK solid volume) was applied to a screw-mount
# volume ~8 % too big, +17.7 g on the propulsion module (E), and test_sim_assist_off_limiter_cuts lost its I4 cut.
# Plan regions are the Layout's own polygons (vehicle XY) and the axis-centred annuli of this estimate's solids: their
# intersections are exact (polygon edges split at the circle + circular sectors, Green's theorem), integrated in Z slabs.
# --------------------------------------------------------------------------- #
def _seg_moments(x0: float, y0: float, x1: float, y1: float) -> np.ndarray:
    """[A, Sx, Sy, Sxx, Syy, Sxy] (int 1, x, y, x^2, y^2, xy dA) of the signed triangle (origin, p0, p1): the
    contribution of the directed boundary segment p0 -> p1 (shoelace terms)."""
    c = x0 * y1 - x1 * y0
    return np.array([0.5 * c, (x0 + x1) * c / 6.0, (y0 + y1) * c / 6.0, (x0 * x0 + x0 * x1 + x1 * x1) * c / 12.0,
                     (y0 * y0 + y0 * y1 + y1 * y1) * c / 12.0, (x0 * y1 + 2 * x0 * y0 + 2 * x1 * y1 + x1 * y0) * c / 24.0])


def _arc_moments(R: float, a0: float, a1: float) -> np.ndarray:
    """The same moments of the circular sector (origin, radius R, angles a0 -> a1 rad): the contribution of a CCW arc."""
    d, s2 = a1 - a0, (math.sin(2 * a1) - math.sin(2 * a0)) / 4.0
    r3, r4 = R ** 3 / 3.0, R ** 4 / 4.0
    return np.array([0.5 * R * R * d, r3 * (math.sin(a1) - math.sin(a0)), r3 * (math.cos(a0) - math.cos(a1)),
                     r4 * (d / 2.0 + s2), r4 * (d / 2.0 - s2), r4 * (math.sin(a1) ** 2 - math.sin(a0) ** 2) / 2.0])


def _ccw(poly: Any) -> list[tuple[float, float]]:
    pts = [(float(x), float(y)) for x, y in poly]
    a = sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]))
    return pts if a > 0 else pts[::-1]


def _inside(x: float, y: float, poly: list[tuple[float, float]]) -> bool:
    """Even-odd point-in-polygon test."""
    ins = False
    for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            ins = not ins
    return ins


def _poly_disc_moments(poly: list[tuple[float, float]], R: float) -> np.ndarray:
    """Exact moments of (CCW polygon) INTERSECT the disc r <= R centred on the axis (R = inf: the whole polygon).
    A polygon with vertices ON the circle (the blister's inner edge follows the EDF bore) is taken against R + 1e-6 mm,
    so every crossing is a proper one (changes the area by < 1e-4 mm^2)."""
    if not math.isinf(R) and min(abs(math.hypot(x, y) - R) for x, y in poly) < 1e-7:
        R += 1e-6
    m, angs = np.zeros(6), []
    for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
        if math.isinf(R):
            m += _seg_moments(x0, y0, x1, y1)
            continue
        dx, dy = x1 - x0, y1 - y0
        a, b, c = dx * dx + dy * dy, 2.0 * (x0 * dx + y0 * dy), x0 * x0 + y0 * y0 - R * R
        ts, disc = [0.0], b * b - 4.0 * a * c
        if a > 0.0 and disc > 0.0:
            sq = math.sqrt(disc)
            for t in sorted(((-b - sq) / (2.0 * a), (-b + sq) / (2.0 * a))):
                if 1e-12 < t < 1.0 - 1e-12:
                    ts.append(t)
                    angs.append(math.atan2(y0 + t * dy, x0 + t * dx))
        ts.append(1.0)
        for ta, tb in zip(ts[:-1], ts[1:]):
            tm = 0.5 * (ta + tb)
            if (x0 + tm * dx) ** 2 + (y0 + tm * dy) ** 2 < R * R:
                m += _seg_moments(x0 + ta * dx, y0 + ta * dy, x0 + tb * dx, y0 + tb * dy)
    if math.isinf(R):
        return m
    if not angs:                                    # no crossing: polygon inside, disc inside or disjoint
        if m[0] > 0.0:
            return m
        return _arc_moments(R, 0.0, 2.0 * math.pi) if _inside(R, 0.0, poly) else m
    angs.sort()
    for k, a0 in enumerate(angs):
        a1 = angs[k + 1] if k + 1 < len(angs) else angs[0] + 2.0 * math.pi
        am = 0.5 * (a0 + a1)
        if a1 - a0 > 1e-12 and _inside(R * math.cos(am), R * math.sin(am), poly):
            m += _arc_moments(R, a0, a1)
    return m


def _clock_annuli(L: Layout, z: float) -> list[tuple[float, float]]:
    """(r_in, r_out) of every axisymmetric solid of this estimate's propulsion module at height z (duct tube, shoulder
    ring, cone shell, cylindrical skin, sleeve, spigot ring, liner tube and ring) - what the lead passage / blister /
    keel plan regions meet."""
    out = []
    for a, b, za, zb in ((L.duct_r, L.vane_section_r, L.z_exit, L.z_vane_section_top),
                         (L.duct_r, None, L.z_vane_section_top, L.z_edf_shoulder),
                         (-L.cone_wall, None, L.z_sleeve_bottom, L.cone_z1),
                         (L.body_r - L.cone_wall, L.body_r, L.cone_z1, L.z_prop_spigot_bottom),
                         (L.edf_bore_r, L.sleeve_r, L.z_sleeve_bottom, L.z_prop_spigot_bottom),
                         (L.edf_bore_r, L.spigot_od / 2, L.z_prop_spigot_bottom, L.z_prop_spigot_top),
                         (L.duct_r, L.liner_r_out, L.z_edf_shoulder, L.z_liner_top),
                         (L.liner_r_out, L.edf_bore_r, L.z_liner_ring_bottom, L.z_liner_top)):
        if not za <= z < zb:
            continue
        if b is None:                               # the shoulder ring / cone shell follow the outer skin
            b = float(L.cone_outer_radius(z))
            a = b + a if a < 0 else a
        if b > a:
            out.append((a, b))
    return out


def _clock_plan_terms(L: Layout) -> list[tuple[float, float, np.ndarray, np.ndarray]]:
    """Z slabs (z0, z1, added moments, removed moments) of the blister (Layout.lp_blister_xy, z lp_z_blister_bottom ..
    lp_z_blister_top), its 45 deg V keel under the floor (Layout.lp_keel steps; the part above the floor lies in the
    blister) and the lead passage cut (Layout.lp_fan_xy, z lp_z_fan_floor .. z_prop_top), against the annuli of
    ``_clock_annuli``: added = blister / keel minus what the annuli already hold; removed = the passage's share of the
    blister (its part outside the EDF bore, which the blister fills) plus the annuli inside the bore radius, or above
    the blister top the annuli.  Slabs are 0.5 mm where a radius or the keel width changes with z, one slab elsewhere."""
    B, F = _ccw(L.lp_blister_xy), _ccw(L.lp_fan_xy)
    zbb, zbt, zff, ztop = L.lp_z_blister_bottom, L.lp_z_blister_top, L.lp_z_fan_floor, L.z_prop_top
    if not (zbb <= zff < zbt <= ztop):
        raise ValueError("lead passage / blister Z order changed: model the clock mount's overlap explicitly")
    ov, th = L.lp_keel_overlap, math.radians(L.wire_theta)
    cs, sn = math.cos(th), math.sin(th)
    keel = [(u0, u1, hk, zbb - hk) for u0, u1, hk in L.lp_keel]
    z_lo = min([k[3] for k in keel] + [zbb])
    cuts = {z_lo, zbb, zff, zbt, ztop} | {k[3] for k in keel}
    cuts |= {z for z in (L.z_vane_section_top, L.z_edf_shoulder, L.z_sleeve_bottom, L.cone_z0, L.cone_z1,
                         L.z_liner_ring_bottom, L.z_liner_top, L.z_prop_spigot_bottom, L.z_prop_spigot_top)
             if z_lo < z < ztop}
    zs = sorted(cuts)
    cache: dict[tuple[int, float], np.ndarray] = {}

    def dm(key: int, poly: list[tuple[float, float]], R: float) -> np.ndarray:
        if key < 0:
            return _poly_disc_moments(poly, R)
        k = (key, R)
        if k not in cache:
            cache[k] = _poly_disc_moments(poly, R)
        return cache[k]

    def ann(key: int, poly: list[tuple[float, float]], a: float, b: float) -> np.ndarray:
        return dm(key, poly, b) - (dm(key, poly, a) if a > 0.0 else 0.0)

    out = []
    for za, zb in zip(zs[:-1], zs[1:]):
        zm0 = 0.5 * (za + zb)
        varies = zm0 < zbb or L.z_vane_section_top <= zm0 < L.cone_z1
        n = max(1, math.ceil((zb - za) / 0.5)) if varies else 1
        for i in range(n):
            z0, z1 = za + i * (zb - za) / n, za + (i + 1) * (zb - za) / n
            zm = 0.5 * (z0 + z1)
            annuli = _clock_annuli(L, zm)
            adds: list[tuple[int, list[tuple[float, float]]]] = []
            if zbb <= zm < zbt:
                adds.append((0, B))
            elif zm < zbb:
                for u0, u1, hk, zk in keel:
                    if zm > zk:
                        w = hk * (zm - zk) / (hk + ov)
                        adds.append((-1, _ccw([(u * cs - v * sn, u * sn + v * cs)
                                               for u, v in ((u0, -w), (u1, -w), (u1, w), (u0, w))])))
            add, rem = np.zeros(6), np.zeros(6)
            for key, P in adds:
                add += dm(key, P, math.inf)
                for a, b in annuli:
                    rem += ann(key, P, a, b)
            if zff <= zm < ztop:
                if zbb <= zm < zbt:
                    rem += dm(1, F, math.inf) - dm(1, F, L.edf_bore_r)
                    for a, b in annuli:
                        if a < L.edf_bore_r:
                            rem += ann(1, F, a, min(b, L.edf_bore_r))
                else:
                    for a, b in annuli:
                        rem += ann(1, F, a, b)
            out.append((z0, z1, add, rem))
    return out


def _slab_solid(mom: np.ndarray, z0: float, z1: float, rho: float, bbox_xy: np.ndarray) -> _Solid | None:
    """Prism z0..z1 over a plan region of moments ``mom`` (mm), density rho (negative = cut, signed volume)."""
    A = float(mom[0])
    if A <= 1e-9:
        return None
    h = z1 - z0
    cx, cy = mom[1] / A, mom[2] / A
    m = rho * A * h * 1e-9
    s = rho * h * 1e-15
    ih = m * h * h * 1e-6 / 12.0
    ixy = -s * (mom[5] - A * cx * cy)
    I = np.array([[s * (mom[4] - A * cy * cy) + ih, ixy, 0.0], [ixy, s * (mom[3] - A * cx * cx) + ih, 0.0],
                  [0.0, 0.0, s * (mom[3] + mom[4] - A * (cx * cx + cy * cy))]])
    bbox = np.array([[bbox_xy[0, 0], bbox_xy[0, 1], z0], [bbox_xy[1, 0], bbox_xy[1, 1], z1]])
    return _signed(_Solid(m, np.array([cx, cy, 0.5 * (z0 + z1)]) * 1e-3, I, A * h * 1e-9, bbox))


def _clock_mount_solids(L: Layout, rho: float) -> list[_Solid]:
    """wire09c clock mount as the CadQuery solid builds it (rocket_geometry.propulsion_module, clock branch):
    * the keyways round the circle (Layout.ck_keyways, kw wide, r keyway_r_in .. keyway_r_out, floor lp_z_floor, open at
      the top) cut through the sleeve / spigot ring / skin / liner layers (no lugs);
    * one floor pad per keyway (Layout.ck_pad_rz, ck_pad_w wide) clamped to the cavity (u from sleeve_r to the skin's inner
      face - the 0.3 mm the CAD polygon laps into both is already counted);
    * the servo-lead column guide walls (Layout.ck_col_walls: (column, v0, v1) per side) over the sleeve -> skin cavity,
      shoulder top .. deck bottom (z_prop_spigot_bottom - body_wall, the deck CAD ties the spigot with); the channel
      between them is that cavity, so it removes nothing here;
    * the lead-passage blister, its keel and the passage cut (``_clock_plan_terms``).
    Not modelled (E): the deck the passage / keyways also cut, the fillet-free blister wall's lap onto the column
    guide wall next to it, the keel's touch on the fin root below it, the servo-lead cutters (as before)."""
    sol: list[_Solid] = []
    h = L.ck_kw / 2
    for th in L.ck_keyways:
        sol += _radial_cut(_prop_layers(L, th, lugs=False), L.keyway_r_in, L.keyway_r_out, h, L.lp_z_floor,
                           L.keyway_z1, th, rho)

    def rin(z: float) -> float:
        return float(L.cone_outer_radius(z)) - L.cone_wall

    pad = np.array([(min(max(u, L.sleeve_r), rin(z)), z) for u, z in L.ck_pad_rz])
    sol += [_plate_rz(pad, L.ck_pad_w, th, rho) for th in L.ck_keyways]
    z0, z1 = L.z_edf_shoulder, L.z_prop_spigot_bottom - L.body_wall
    skin = ([(float(L.cone_outer_radius(z0)), z0)] + [(r, z) for r, z in L.outer_skin_profile() if z0 < z < z1]
            + [(float(L.cone_outer_radius(z1)), z1)])
    chan = np.array([(L.sleeve_r, z0)] + [(r - L.cone_wall, z) for r, z in skin] + [(L.sleeve_r, z1)])
    sol += [_plate_rz(chan, v1 - v0, c, rho, v_offset=0.5 * (v0 + v1)) for c, v0, v1 in L.ck_col_walls]
    bb_b = np.array(L.lp_blister_xy, dtype=float)
    bb_b = np.array([bb_b.min(axis=0), bb_b.max(axis=0)])
    for za, zb, add, rem in _clock_plan_terms(L):
        for mom, r in ((add, rho), (rem, -rho)):
            s = _slab_solid(mom, za, zb, r, bb_b)
            if s is not None:
                sol.append(s)
    return sol


def _wire_notch_z(L: Layout) -> tuple[float, float]:
    """Shelf-notch Z range [wire_notch_z0, wire_ch_z1] (SPEC §5.1).  Its outer 0.5 mm (r to wire_ch_r0 + 0.5) is
    channel void already because the notch's Z range lies inside the channel's, so only r [wire_notch_r0,
    wire_ch_r0] is cut; a Layout with the notch starting below the channel is refused."""
    if L.wire_notch_z0 < L.wire_ch_z0:
        raise ValueError("wire shelf notch starts below the wire channel: model its overlap explicitly")
    return L.wire_notch_z0, L.wire_ch_z1


def _wire_floor_wedge_rz(L: Layout) -> np.ndarray | None:
    """(r, z) polygon of the new material the v0.5d floor wedge adds (SPEC §5.1): the triangle ``wire_ch_floor_rz``
    clipped to u <= wire_ch_r1, i.e. its part inside the channel void (the wedge is unioned AFTER the cuts); the
    part beyond wire_ch_r1 lies in the bulge's outer wall, solid already.  A triangle leaving the void / bulge wall
    anywhere else is a contract change (ValueError)."""
    tri = [(float(u), float(z)) for u, z in L.wire_ch_floor_rz]
    us, zs = [u for u, _ in tri], [z for _, z in tri]
    if (min(us) < L.wire_ch_r0 - 1e-9 or min(zs) < L.wire_ch_z0 - 1e-9 or max(zs) > L.wire_ch_z1 + 1e-9
            or min(zs) < L.wire_bulge_z0 or max(zs) > L.wire_bulge_z1 or max(us) > L.wire_bulge_r_out + 1e-9):
        raise ValueError("wire channel floor wedge leaves the channel void / bulge wall: model its overlap explicitly")
    out: list[tuple[float, float]] = []
    for i, a in enumerate(tri):                       # Sutherland-Hodgman against u <= wire_ch_r1
        b = tri[(i + 1) % len(tri)]
        a_in, b_in = a[0] <= L.wire_ch_r1, b[0] <= L.wire_ch_r1
        if a_in:
            out.append(a)
        if a_in != b_in:
            t = (L.wire_ch_r1 - a[0]) / (b[0] - a[0])
            out.append((L.wire_ch_r1, a[1] + t * (b[1] - a[1])))
    return np.array(out) if len(out) >= 3 else None


def _wire_channel_solids(L: Layout, rho: float) -> list[_Solid]:
    """Intake wire bulge (its part outside body_r, unioned before the cuts so it closes the channel's bottom
    and top), channel void through skirt / lip pocket ring / shelf / ring below the windows / bulge, the shelf
    notch and the bulkhead wire hole, then (v0.5d, unioned after the cuts) the floor wedge as an exact
    triangular prism over the channel width (SPEC §5.1)."""
    ri, ro = L.body_id / 2, L.body_r
    if getattr(L, "wire_ch_open", False):
        # v0.9 open groove: two side walls outside body_r, the roof wedge, the groove cut outside wire_ch_r0 up to the
        # shelf top, the gate (wire_gate_w) through shelf / ring, the bulkhead wire hole
        hw, sw = L.wire_ch_w / 2, L.wire_side_wall
        sol = [_box_polar((ro, L.wire_ch_r1), sw, (L.z_intake_bottom, L.wire_ch_z1), L.wire_theta, rho, v_offset=s * (hw + sw / 2))
               for s in (1.0, -1.0)]
        sol.append(_plate_rz(np.array(L.wire_roof_rz, dtype=float), 2 * (hw + sw), L.wire_theta, rho))
        layers = [(ri, ro, L.z_intake_bottom, L.z_intake_skirt_top),
                  (L.lip_pocket_id / 2, ro, L.z_intake_skirt_top, L.z_lip_pocket_top),
                  (L.shelf_id / 2, ro, L.z_lip_pocket_top, L.z_shelf_top),
                  (L.r_band_in, ro, L.z_shelf_top, L.z_window_bottom)]
        sol += _radial_cut(layers, L.wire_ch_r0, ro, hw, L.z_intake_bottom, L.wire_groove_z1, L.wire_theta, rho)
        # the gate: below the groove top only from the bore to wire_ch_r0 (outside it the groove removed the material
        # already), above it through the ring out to body_r
        zg = min(max(L.wire_groove_z1, L.wire_notch_z0), L.wire_ch_z1)
        sol += _radial_cut(layers, L.wire_notch_r0, L.wire_ch_r0, L.wire_gate_w / 2, L.wire_notch_z0, zg, L.wire_theta, rho)
        sol += _radial_cut(layers, L.wire_notch_r0, ro, L.wire_gate_w / 2, zg, L.wire_ch_z1, L.wire_theta, rho)
        hole = _clipped_box_polar(L.wire_hole_r0, L.wire_hole_r1, hw, 0.0, ro,
                                  (L.z_intake_bulkhead_bottom, L.z_intake_bulkhead_top), L.wire_theta, -rho)
        if hole is not None:
            sol.append(hole)
        return sol
    hb = L.wire_bulge_w / 2
    sol: list[_Solid] = []
    bulge = _clipped_box_polar(math.sqrt(ro * ro - hb * hb), L.wire_bulge_r_out, hb, ro, math.inf,
                               (L.wire_bulge_z0, L.wire_bulge_z1), L.wire_theta, rho)
    if bulge is not None:
        sol.append(bulge)
    layers = [(ri, ro, L.z_intake_bottom, L.z_intake_skirt_top),
              (L.lip_pocket_id / 2, ro, L.z_intake_skirt_top, L.z_lip_pocket_top),
              (L.shelf_id / 2, ro, L.z_lip_pocket_top, L.z_shelf_top),
              (L.r_band_in, ro, L.z_shelf_top, L.z_window_bottom),            # v0.6.1: window_wall_t band
              (ro, L.wire_bulge_r_out, L.wire_bulge_z0, L.wire_bulge_z1)]
    sol += _radial_cut(layers, L.wire_ch_r0, L.wire_ch_r1, L.wire_ch_w / 2, L.wire_ch_z0, L.wire_ch_z1,
                       L.wire_theta, rho)
    sol += _radial_cut(layers, L.wire_notch_r0, L.wire_ch_r0, L.wire_ch_w / 2, *_wire_notch_z(L), L.wire_theta, rho)
    hole = _clipped_box_polar(L.wire_hole_r0, L.wire_hole_r1, L.wire_ch_w / 2, 0.0, ro,
                              (L.z_intake_bulkhead_bottom, L.z_intake_bulkhead_top), L.wire_theta, -rho)
    if hole is not None:
        sol.append(hole)
    wedge = _wire_floor_wedge_rz(L)
    if wedge is not None:
        sol.append(_plate_rz(wedge, L.wire_ch_w, L.wire_theta, rho))
    return sol


def _window_gussets(L: Layout, rho: float) -> list[_Solid]:
    """v0.6.2 (``intake_module.window_bottom_end``): the band material under each window's truncated gable - one solid
    per window with Layout's exact gusset volume at its centroid (gusset_centroid_u along the window centre,
    gusset_centroid_z); own inertia neglected (~2.7 g each, ~30 mm across).  Empty without a gable."""
    if not getattr(L, "has_window_gable", False):
        return []
    v_mm3, out = L.gusset_volume_per_window_mm3, []
    for th in L.window_thetas:
        xyz = _polar_xyz(L.gusset_centroid_u, th, L.gusset_centroid_z)
        out.append(_Solid(rho * v_mm3 * 1e-9, xyz * 1e-3, np.zeros((3, 3)), v_mm3 * 1e-9, np.array([xyz, xyz])))
    return out


def _tray_stop_solids(L: Layout, rho: float) -> list[_Solid]:
    """v0.5d tray stop (SPEC §5.2, both presets): one block per rail slot between the two ribs (x = tray_x +-
    rail_gap/2), the ribs' radial extent (their box convention: |y| from rail_y - rail_depth to rail_y),
    Z [z_tray_stop_bottom, z_tray_bottom].  Empty when params carry no ``tray_stop_h``."""
    if not getattr(L, "tray_stop_h", None):
        return []
    out = []
    for sy in (-1.0, 1.0):
        y = sorted((sy * L.rail_y, sy * (L.rail_y - L.rail_depth)))
        out.append(_box_xyz((L.tray_x - L.rail_gap / 2, L.tray_x + L.rail_gap / 2), (y[0], y[1]),
                            (L.z_tray_stop_bottom, L.z_tray_bottom), rho))
    return out


# --------------------------------------------------------------------------- #
# internal estimate (SPEC §3-6 geometry, simplified solids)
# --------------------------------------------------------------------------- #
def _fairing_solids(L: Layout, rho: float) -> list[_Solid]:
    """v0.9 (vane_drive.mode 'direct'): fin-root fairing per fin - box from the duct surface, minus the pocket, the
    service opening and the ear slot, plus the keel (as a box of its volume), minus the fin plate inside them; the lead
    pipe (tube O sl_pipe_d - O sl_tunnel_d) and half the web pipe along P0 -> Pe as a cylinder of that volume."""
    out: list[_Solid] = []
    u0, u1, hv, pv = L.vane_section_r, L.fair_u[1], L.fair_hv, L.dd_pocket_hv
    kd = L.fair_z[0] - L.fair_keel_z0
    for k, th in enumerate(L.fin_thetas):
        out += [_box_polar((u0, u1), 2 * hv, (L.fair_z[0], L.fair_z[1]), th, rho),
                _box_polar(L.dd_pocket_u, 2 * pv, L.dd_pocket_z, th, -rho),
                _box_polar(L.dd_pocket_u, hv - pv, (L.dd_pocket_z[0], L.dd_open_z1), th, -rho, v_offset=0.5 * (hv + pv)),
                _box_polar(L.dd_slot_u, hv + pv, L.dd_slot_z, th, -rho, v_offset=0.5 * (hv - pv)),
                _box_polar((u0, u1), (2 * hv - L.fin_t) / 2 + L.fin_t, (L.fair_keel_z0, L.fair_z[0]), th, rho),
                _box_polar((u0, u1), L.fin_t, (L.fair_keel_z0, L.fair_z[1]), th, -rho)]
        if getattr(L, "servo_leads", False):
            a, b = L.sl_pipe_points(k)
            d_eq = math.sqrt(L.sl_pipe_d ** 2 - L.sl_tunnel_d ** 2 + 0.5 * L.sl_pipe_d ** 2)
            out.append(_cylinder_between(d_eq, np.asarray(a), np.asarray(b), rho))
    return out


def estimate_parts(L: Layout, p: dict) -> dict[str, Part]:
    """Approximate every printed part / placeholder of SPEC §6 with simple
    solids.  Returns the ``parts`` dict in the CAD-report schema units."""
    rho_print = p["printer"]["density_g_cm3"] * 1e3  # kg/m^3
    parts: dict[str, Part] = {}

    def add(name: str, kind: str, solids: list[_Solid], density_g_cm3: float | None = None,
            mass_g: float | None = None) -> None:
        s = _compose(solids)
        if mass_g is not None:  # placeholder: density = mass / volume
            scale = (mass_g * 1e-3) / s.m if s.m > 0 else 0.0
            s = _Solid(mass_g * 1e-3, s.c, s.I * scale, s.V, s.bbox)
            density_g_cm3 = (mass_g / (s.V * 1e6)) if s.V > 0 else 0.0
        parts[name] = Part(name, kind, s.m * 1e3, s.c * 1e3, s.I, s.V * 1e9,
                           density_g_cm3 if density_g_cm3 is not None else rho_print * 1e-3, s.bbox)

    def cone_r(z: np.ndarray | float) -> np.ndarray | float:
        return np.vectorize(L.cone_outer_radius, otypes=[float])(np.asarray(z, dtype=float))

    pod_w = L.pod_w
    cav_w = L.pod_cavity_w

    def pod_solids(r_wall: float, r_in_cav: float, r_out_cav: float, r_out: float, z0: float, z_floor: float,
                   z_rim: float, theta: float) -> list[_Solid]:
        """Servo pod cup outside the wall it sits on + 45 deg support wedge (SPEC §3.4)."""
        sol = [_box_polar((r_wall, r_out), pod_w, (z0, z_rim), theta, rho_print),
               _box_polar((r_in_cav, r_out_cav), cav_w, (z_floor, z_rim), theta, -rho_print)]
        drop = r_out - r_wall
        z_low = max(z0 - drop, 0.0)
        if drop > 0 and z0 > z_low:  # triangular wedge below the box, hypotenuse at the chamfer angle
            tri = np.array([[r_wall, z0], [r_out, z0], [r_wall, z_low]])
            # a tangential prism: use plate helper with "thickness" = pod width (exact for a prism)
            sol.append(_plate_rz(tri, pod_w, theta, rho_print))
        return sol

    # ---- propulsion module -------------------------------------------------
    prop: list[_Solid] = [
        _tube(L.vane_section_r, L.duct_r, L.z_exit, L.z_vane_section_top, rho_print),
        _revolved_shell(np.linspace(L.z_vane_section_top, L.z_edf_shoulder, 20),
                        cone_r, lambda z: np.full_like(np.asarray(z, dtype=float), L.duct_r), rho_print),
        _revolved_shell(np.linspace(L.z_sleeve_bottom, L.cone_z1, 60),
                        cone_r, lambda z: cone_r(z) - L.cone_wall, rho_print),
        _tube(L.sleeve_r, L.edf_bore_r, L.z_sleeve_bottom, L.z_prop_spigot_bottom, rho_print),
        _tube(L.spigot_od / 2, L.edf_bore_r, L.z_prop_spigot_bottom, L.z_prop_spigot_top, rho_print),
    ]
    z0w, z1w = L.z_sleeve_bottom, L.z_prop_spigot_bottom
    for th in L.web_thetas:
        r_in0 = max(float(cone_r(z0w)) - L.cone_wall, L.sleeve_r)
        r_in1 = max(float(cone_r(z1w)) - L.cone_wall, L.sleeve_r)
        if L.tabs:   # v0.5: the webs follow the outer skin, which turns cylindrical at cone_z1
            r_k = max(float(cone_r(L.cone_z1)) - L.cone_wall, L.sleeve_r)
            poly = np.array([[L.sleeve_r, z0w], [r_in0, z0w], [r_k, L.cone_z1], [r_in1, z1w], [L.sleeve_r, z1w]])
        else:
            poly = np.array([[L.sleeve_r, z0w], [r_in0, z0w], [r_in1, z1w], [L.sleeve_r, z1w]])
        if abs(_polygon_props(poly * 1e-3)[0]) > 1e-12:
            prop.append(_plate_rz(poly, L.web_t, th, rho_print))
    for th in L.vane_thetas:
        prop.append(_radial_cylinder(L.hinge_boss_od, L.vane_section_r, L.hinge_boss_r_out, L.vane_hinge_z, th,
                                     rho_print, bore_d_mm=L.hinge_bore_d))
    if getattr(L, "direct_drive", False):      # v0.9: fin-root fairings (box - pocket - opening - slot + keel), pipes
        prop += _fairing_solids(L, rho_print)
    else:
        for th in L.pod_thetas:
            prop += pod_solids(L.vane_section_r, L.pod_cavity_r_in, L.pod_cavity_r_out, L.pod_r_out,
                               L.pod_z0, L.pod_floor_top_z, L.pod_rim_z, th)
    fin_poly = np.array([[L.vane_section_r, L.fin_te_z], [L.fin_tip_r, L.fin_te_z],
                         [L.fin_tip_r, L.fin_tip_le_z], [L.vane_section_r, L.fin_root_le_z]])
    for th in L.fin_thetas:
        prop.append(_plate_rz(fin_poly, L.fin_t, th, rho_print))
    if L.tabs:
        prop += _tab_mount_solids(L, rho_print)
    add("propulsion_module", "printed", prop)

    # ---- intake module -----------------------------------------------------
    # v0.6.1 (SPEC §5.1): the windowed band z_shelf_top .. bulkhead is r_band_in .. body_r (window_wall_t), the struts
    # are radial-sided sectors of strut_angle_deg centred on strut_thetas (exact annular sectors)
    intake: list[_Solid] = [
        _tube(L.body_r, L.body_id / 2, L.z_intake_bottom, L.z_intake_skirt_top, rho_print),
        _tube(L.body_r, L.lip_pocket_id / 2, L.z_intake_skirt_top, L.z_lip_pocket_top, rho_print),
        _tube(L.body_r, L.shelf_id / 2, L.z_lip_pocket_top, L.z_shelf_top, rho_print),
        _tube(L.body_r, L.r_band_in, L.z_shelf_top, L.z_window_bottom, rho_print),
        _tube(L.body_r, L.r_band_in, L.z_window_top, L.z_intake_bulkhead_bottom, rho_print),
        _tube(L.body_r, 0.0, L.z_intake_bulkhead_bottom, L.z_intake_bulkhead_top, rho_print),
        _tube(L.cable_hole_d / 2, 0.0, L.z_intake_bulkhead_bottom, L.z_intake_bulkhead_top, -rho_print),
        _tube(L.spigot_od / 2, L.spigot_id / 2, L.z_intake_spigot_bottom, L.z_intake_spigot_top, rho_print),
    ]
    for th in L.strut_thetas:
        intake.append(_sector(L.r_band_in, L.body_r, th - L.strut_angle_deg / 2, th + L.strut_angle_deg / 2,
                              (L.z_window_bottom, L.z_window_top), rho_print))
    for th in L.vent_hole_thetas:
        xy = _polar_xyz(L.vent_hole_r, th, 0.0)
        intake.append(_tube(L.vent_hole_d / 2, 0.0, L.z_intake_bulkhead_bottom, L.z_intake_bulkhead_top,
                            -rho_print, xy[0], xy[1]))
    intake += _window_gussets(L, rho_print)
    if L.tabs:
        intake += _wire_channel_solids(L, rho_print)
    add("intake_module", "printed", intake)

    # ---- avionics module ---------------------------------------------------
    avi: list[_Solid] = [
        _tube(L.body_r, L.body_id / 2, L.z_avionics_bottom, L.z_avionics_spigot_bottom, rho_print),
        _tube(L.spigot_od / 2, L.spigot_id / 2, L.z_avionics_spigot_bottom, L.z_avionics_spigot_top, rho_print),
    ]
    for th in L.vent_slot_thetas:
        avi.append(_box_polar((L.body_id / 2, L.body_r), L.vent_slot_w,
                              (L.z_vent_slot_bottom, L.z_vent_slot_bottom + L.vent_slot_h), th, -rho_print))
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            xc = L.tray_x + sx * (L.rail_gap / 2 + L.rail_w / 2)
            y_outer = sy * L.rail_y
            y_inner = y_outer - sy * L.rail_depth
            avi.append(_box_xyz((xc - L.rail_w / 2, xc + L.rail_w / 2), (min(y_outer, y_inner), max(y_outer, y_inner)),
                                (L.z_rail_bottom, L.z_rail_top), rho_print))
    avi += _tray_stop_solids(L, rho_print)
    if getattr(L, "has_livery", False):
        # livery (2026-11-04): the orange inlay of each copy as a thin radial box over the logo's bounding box, at the
        # density that gives logo area x depth (ESTIMATE); the same removed from the module (its pocket)
        (s0_, s1_), (lz0, lz1) = L.livery_s_range, L.livery_z_range
        fill_ = L.livery_area_mm2 / ((s1_ - s0_) * (lz1 - lz0))
        logo = [_box_polar((L.livery_r_in, L.livery_r), s1_ - s0_, (lz0, lz1), th_, rho_print * fill_,
                           v_offset=0.5 * (s0_ + s1_)) for th_ in L.livery_thetas]
        avi += [_box_polar((L.livery_r_in, L.livery_r), s1_ - s0_, (lz0, lz1), th_, -rho_print * fill_,
                           v_offset=0.5 * (s0_ + s1_)) for th_ in L.livery_thetas]
    add("avionics_module", "printed", avi)
    if getattr(L, "has_livery", False):
        add(L.livery_inlay_part, "printed", logo)

    # ---- avionics tray -----------------------------------------------------
    am = p["avionics_module"]
    tray: list[_Solid] = [_box_xyz((L.tray_x - L.tray_t / 2, L.tray_x + L.tray_t / 2), (-L.tray_w / 2, L.tray_w / 2),
                                   (L.z_tray_bottom, L.z_tray_top), rho_print)]
    slot_w, slot_h = am["strap_slot_w"], am["strap_slot_h"]
    esc_mid = 0.5 * (L.esc_z[0] + L.esc_z[1])
    for (yc, zc) in ((40.0, L.battery_z[0] + 25.0), (-40.0, L.battery_z[0] + 25.0),
                     (40.0, L.battery_z[1] - 25.0), (-40.0, L.battery_z[1] - 25.0),
                     (35.0, esc_mid), (-35.0, esc_mid)):
        tray.append(_box_xyz((L.tray_x - L.tray_t / 2, L.tray_x + L.tray_t / 2),
                             (yc - slot_w / 2, yc + slot_w / 2), (zc - slot_h / 2, zc + slot_h / 2), -rho_print))
    add("avionics_tray", "printed", tray)

    # ---- payload module ----------------------------------------------------
    pl = p["payload_module"]
    pay: list[_Solid] = [
        _tube(L.body_r, L.body_id / 2, L.z_payload_bottom, L.z_payload_spigot_bottom, rho_print),
        _tube(L.body_id / 2, 0.0, L.z_payload_bulkhead_bottom, L.z_payload_bulkhead_top, rho_print),
        _tube(L.spring_seat_boss_d / 2, 0.0, L.z_payload_bulkhead_top, L.z_payload_bulkhead_top + pl["spring_seat_boss_h"], rho_print),
        _tube(L.spigot_od / 2, L.spigot_id / 2, L.z_payload_spigot_bottom, L.z_payload_spigot_top, rho_print),
    ]
    for k in range(pl["bulkhead_hole_count"]):
        xy = _polar_xyz(pl["bulkhead_hole_r"], 45.0 + k * 360.0 / pl["bulkhead_hole_count"], 0.0)
        pay.append(_tube(pl["bulkhead_hole_d"] / 2, 0.0, L.z_payload_bulkhead_bottom, L.z_payload_bulkhead_top,
                         -rho_print, xy[0], xy[1]))
    pay += pod_solids(L.body_r, L.release_pod_cavity_r_in, L.release_pod_cavity_r_out, L.release_pod_r_out,
                      L.release_pod_z0, L.release_pod_floor_top_z, L.release_pod_rim_z, L.release_pod_theta)
    add("payload_module", "printed", pay)

    # ---- piston ------------------------------------------------------------
    add("piston", "printed", [
        _tube(L.piston_d / 2, 0.0, L.z_piston_bottom, L.z_piston_top, rho_print),
        _tube(pl["piston_recess_d"] / 2, 0.0, L.z_piston_bottom, L.z_piston_bottom + pl["piston_recess_depth"], -rho_print),
    ])

    # ---- nose cone ---------------------------------------------------------
    no = p["nose"]
    solid_tip_len = 8.0  # SPEC §5.4: shell approximation is acceptable with a solid tip region

    def nose_r_out(z: np.ndarray) -> np.ndarray:
        return np.array([L.nose_outer_radius(zz - L.z_nose_base) for zz in np.atleast_1d(z)])

    def nose_r_in(z: np.ndarray) -> np.ndarray:
        z = np.atleast_1d(z)
        r = nose_r_out(z) - L.nose_wall
        r[(z - L.z_nose_base) > L.nose_len - solid_tip_len] = 0.0
        return r

    r_in_bar = max(L.nose_outer_radius(L.z_anchor_bar - L.z_nose_base) - L.nose_wall, 0.0)
    nose: list[_Solid] = [
        _tube(L.body_r, L.nose_skirt_id / 2, L.z_nose_skirt_bottom, L.z_nose_skirt_top, rho_print),
        _revolved_shell(np.linspace(L.z_nose_base, L.z_nose_tip, 400), nose_r_out, nose_r_in, rho_print),
        _box_xyz((-r_in_bar, r_in_bar), (-no["anchor_bar_w"] / 2, no["anchor_bar_w"] / 2),
                 (L.z_anchor_bar, L.z_anchor_bar + no["anchor_bar_h"]), rho_print),
        _box_polar((L.lug_r_in, L.lug_r_out), L.lug_w, (L.z_lug_bottom, L.z_lug_top), L.latch_lug_theta, rho_print),
    ]
    add("nose_cone", "printed", nose)

    # ---- vanes, horns, latch arm -------------------------------------------
    v = p["vane"]
    for i, th in enumerate(L.vane_thetas, start=1):
        add(f"vane_{i}", "printed", [
            _box_polar((L.vane_r_in, L.vane_r_out), L.vane_t, (L.vane_te_z, L.vane_le_z), th, rho_print),
            _radial_cylinder(L.vane_shaft_d, L.vane_shaft_r_in, L.vane_shaft_r_out, L.vane_hinge_z, th, rho_print),
        ])
        if getattr(L, "direct_drive", False):     # v0.9: coupler (hub + flange - bore), servo cover plate
            # v0.9 fix round: the flange is the horn socket (hub relief bored out; the arm slot's share is left in -
            # < 0.1 g per coupler at this density, the CAD report has the exact solid)
            add(f"vane_coupler_{i}", "printed", [
                _radial_cylinder(L.dd_hub_d, L.dd_u_hub0, L.dd_u_hub1, L.vane_hinge_z, th, rho_print, bore_d_mm=L.dd_bore_d),
                _radial_cylinder(L.dd_flange_d, L.dd_u_hub1, L.dd_u_flange1, L.vane_hinge_z, th, rho_print,
                                 bore_d_mm=getattr(L, "dd_collar_relief_d", 0.0))])
            add(f"servo_cover_{i}", "printed", [
                _box_polar((L.cov_u[0], L.cov_u[1]), L.cov_t, (L.cov_z[0], L.cov_z[1]), th, rho_print,
                           v_offset=0.5 * (L.cov_v[0] + L.cov_v[1]))])
        else:
            add(f"vane_horn_{i}", "printed", [
                _box_polar((L.horn_r_in, L.horn_r_out), v["horn_w"], (L.horn_tail_z, L.horn_tip_z), th, rho_print)])
    if L.tabs and getattr(L, "wire_ch_open", False):     # v0.9: the motor-lead groove cover
        add("wire_cover", "printed", [_box_polar((L.wire_cov_u[0], L.wire_cov_u[1]), 2 * L.wire_cov_hv,
                                                 (L.wire_cov_z[0], L.wire_cov_z[1]), L.wire_theta, rho_print)])
    if getattr(L, "has_gps_pod", False):    # v0.8: the GPS pod lid (flat plate + posts, as a plate over the pod's plan)
        add("gps_pod_lid", "printed", [_box_polar((L.gps_lid_u_back, L.gps_u_out), 2 * L.gps_w_h,
                                                  (L.gps_z_rim, L.gps_z_lid_top), L.gps_theta, rho_print)])
    lug_c = _polar_xyz(0.5 * (L.lug_r_in + L.lug_r_out), L.latch_lug_theta, 0.0)
    srv_c = _polar_xyz(L.release_servo_axis_r, L.release_pod_theta, 0.0)
    arm_c = 0.5 * (lug_c + srv_c)
    arm_r = math.hypot(arm_c[0], arm_c[1])
    arm_th = math.degrees(math.atan2(arm_c[1], arm_c[0]))
    add("latch_arm", "printed", [
        _box_polar((arm_r - L.latch_arm_len / 2, arm_r + L.latch_arm_len / 2), L.latch_arm_w,
                   (L.z_latch_arm_bottom, L.z_latch_arm_top), arm_th, rho_print)])

    # ---- placeholders (density = mass / volume) ----------------------------
    e = p["edf"]
    edf = [
        _tube(e["lip_od"] / 2, e["housing_id"] / 2 if L.tabs else 0.0, L.z_edf_lip_bottom, L.z_edf_lip_top, 1.0),
        _tube(e["housing_od"] / 2, e["housing_id"] / 2, L.z_edf_housing_bottom, L.z_edf_lip_bottom, 1.0),
        _tube(e["hub_d_ASSUMED"] / 2, 0.0, L.z_edf_housing_bottom, L.z_edf_housing_bottom + EDF_HUB_LEN_MM, 1.0),
    ]
    if L.tabs:   # v0.5 SPEC §6: bell = ring (above), motor below the housing, two tab plates with three slots
        edf.append(_tube(e["motor_d"] / 2, 0.0, L.z_edf_housing_bottom - e["motor_protrusion_rear"],
                         L.z_edf_housing_bottom, 1.0))
        for th in L.tab_thetas:
            edf.append(_box_polar((L.tab_r_in, L.tab_r_out), L.tab_t, (L.tab_z0, L.tab_z1), th, 1.0))
            for z in L.tab_slot_zs:   # tab_slot_len_radial along r x tab_slot_w along Z
                edf.append(_signed(_box_polar((L.tab_slot_r - L.tab_slot_len_radial / 2,
                                               L.tab_slot_r + L.tab_slot_len_radial / 2),
                                              L.tab_t, (z - L.tab_slot_w / 2, z + L.tab_slot_w / 2), th, -1.0)))
    add("edf_unit", "placeholder", edf, mass_g=e["mass_g"])
    add("battery", "placeholder", [_box_xyz(L.battery_x, L.battery_y, L.battery_z, 1.0)], mass_g=p["battery"]["mass_g"])
    add("esc", "placeholder", [_box_xyz(L.esc_x, L.esc_y, L.esc_z, 1.0)], mass_g=p["electronics"]["esc"]["mass_g"])
    add("nav", "placeholder", [_box_xyz(L.nav_x, L.nav_y, L.nav_z, 1.0)], mass_g=p["electronics"]["nav"]["mass_g"])
    add("misc_electronics", "placeholder", [
        _box_xyz(L.misc_x, L.misc_y, L.misc_z, 1.0)], mass_g=p["electronics"]["misc"]["mass_g"])
    if getattr(L, "has_com_board", False):   # v0.6: COM board (2nd ATmega128A) hanging under the payload bulkhead
        add("com", "placeholder", [_box_xyz(L.com_board_x, L.com_board_y, L.com_board_z, 1.0)],
            mass_g=p["electronics"]["com"]["mass_g"])
    if getattr(L, "has_radio_log", False):   # v0.6.2: LoRa / OpenLog / level shifter / LDO box beside the COM board
        add("radio_log", "placeholder", [_box_xyz(L.radio_log_x, L.radio_log_y, L.radio_log_z, 1.0)],
            mass_g=p["electronics"]["radio_log"]["mass_g"])
    if getattr(L, "has_fcu", False):         # v0.6.3: FCU board flat on the intake bulkhead (foam tape)
        add("fcu", "placeholder", [_box_xyz(L.fcu_x, L.fcu_y, L.fcu_z, 1.0)], mass_g=p["electronics"]["fcu"]["mass_g"])
    s = p["servo"]
    for i, th in enumerate(L.fin_thetas if getattr(L, "direct_drive", False) else [], start=1):   # v0.9
        add(f"servo_vane_{i}", "placeholder", [
            _box_polar((L.dd_u_case_top, L.dd_u_case_bot), s["body_w"], (L.dd_z_case[0], L.dd_z_case[1]), th, 1.0)],
            mass_g=s["mass_g"])
        add(f"servo_horn_{i}", "placeholder", [     # v0.9 fix round: the cut double-arm horn (arm plane .. its face)
            _radial_cylinder(L.dd_horn_d, getattr(L, "dd_u_arm0", L.dd_u_flange1), L.dd_u_horn1, L.vane_hinge_z, th, 1.0)],
            mass_g=p["vane_drive"]["horn"]["mass_g"])
    for i, th in enumerate(L.vane_thetas if getattr(L, "dd_bushing", False) else [], start=1):   # v0.9 fix round
        add(f"hinge_bushing_{i}", "placeholder", [
            _radial_cylinder(L.dd_bush_od, L.dd_bush_u[0], L.dd_bush_u[1], L.vane_hinge_z, th, 1.0, bore_d_mm=L.dd_bush_id)],
            mass_g=L.dd_bush_mass_g)
    for i, th in enumerate([] if getattr(L, "direct_drive", False) else L.pod_thetas, start=1):
        add(f"servo_vane_{i}", "placeholder", [
            _box_polar((L.servo_axis_r - s["body_w"] / 2, L.servo_axis_r + s["body_w"] / 2), s["body_l"],
                       (L.servo_bottom_z, L.servo_bottom_z + s["case_top_from_bottom"]), th, 1.0)], mass_g=s["mass_g"])
        add(f"servo_arm_{i}", "placeholder", [_point(1e-3, tuple(_polar_xyz(
            0.5 * (L.servo_axis_r + L.servo_arm_tip_r), th, 0.5 * (L.servo_arm_z_bottom + L.servo_arm_z_top),
            L.servo_shaft_tangential_offset)))], density_g_cm3=0.0)
        arm_tip = _polar_xyz(L.servo_arm_tip_r, th, 0.5 * (L.servo_arm_z_bottom + L.servo_arm_z_top),
                             L.servo_shaft_tangential_offset)
        horn_tip = _polar_xyz(0.5 * (L.horn_r_in + L.horn_r_out), L.vane_thetas[i - 1], L.horn_tip_z - 2.0)
        add(f"pushrod_{i}", "placeholder", [_point(1e-3, tuple(0.5 * (arm_tip + horn_tip)))], density_g_cm3=0.0)
    add("servo_release", "placeholder", [
        _box_polar((L.release_servo_axis_r - s["body_w"] / 2, L.release_servo_axis_r + s["body_w"] / 2), s["body_l"],
                   (L.release_servo_bottom_z, L.release_servo_bottom_z + s["case_top_from_bottom"]),
                   L.release_pod_theta, 1.0)], mass_g=s["mass_g"])
    z_cansat_chute_top = L.z_can_top + p["payload"]["chute_pack_h"]  # Layout.z_chute_pack_top is the NOSE pack
    add("cansat_payload", "placeholder", [
        _tube(L.can_d / 2, 0.0, L.z_can_bottom, L.z_can_top, 1.0),
        _tube(L.can_d / 2, 0.0, L.z_can_top, max(z_cansat_chute_top, L.z_can_top + 1e-6), 1.0),
    ], mass_g=p["payload"]["mass_g"])
    add("spring", "placeholder", [
        _tube(pl["spring_od"] / 2, pl["spring_od"] / 2 - pl["spring_wire_d"], L.z_spring_bottom, L.z_spring_top, 1.0)],
        mass_g=0.5 * p["hardware_mass_g"]["spring_piston_hardware"])
    rec = p["recovery"]
    add("chute_pack", "placeholder", [_tube(L.chute_pack_d / 2, 0.0, L.z_chute_pack_bottom, L.z_chute_pack_top, 1.0)],
        mass_g=rec["chute_mass_g"] + rec["lines_swivel_mass_g"] + rec["shock_cord_mass_g"])

    # ---- lumped ------------------------------------------------------------
    hw = p["hardware_mass_g"]
    z_joints = [L.z_prop_spigot_bottom, L.z_intake_spigot_bottom, L.z_avionics_spigot_bottom, L.z_payload_spigot_bottom]
    z_joint_mean = sum(z_joints) / len(z_joints) + L.joint_screw_dz
    add("joint_screws_inserts", "lumped", [_point(hw["joint_screws_inserts"] * 1e-3, (0.0, 0.0, z_joint_mean))],
        density_g_cm3=0.0)
    if getattr(L, "direct_drive", False):
        add("vane_drive_hardware", "lumped", [_point(p["vane_drive"]["hardware_g_ASSUMED"] * 1e-3,
                                                     (0.0, 0.0, 0.5 * (L.fair_z[0] + L.fair_z[1])))], density_g_cm3=0.0)
    else:
        add("pushrods_horns_clevises", "lumped", [_point(hw["pushrods_horns_clevises"] * 1e-3, (0.0, 0.0, L.vane_hinge_z))],
            density_g_cm3=0.0)
    if getattr(L, "has_motor_leads", False):   # v0.9: 3 x 12 AWG motor leads + bullets (ASSUMED) at their centroid
        add("motor_leads", "lumped", [_point(L.ml_mass_g * 1e-3, tuple(L.ml_com))], density_g_cm3=0.0)
    if getattr(L, "has_gps_pod", False):       # v0.8: GPS board + patch, ground plane, lead (Layout masses / centroids)
        add("gps", "placeholder", [_point(L.gps_board_mass_g * 1e-3, tuple(L.gps_antenna_xyz))], density_g_cm3=0.0)
        add("gps_ground_plane", "placeholder", [_point(L.gps_gp_mass_g * 1e-3, tuple(L.gps_antenna_xyz))], density_g_cm3=0.0)
        add("gps_lead", "lumped", [_point(L.gps_lead_mass_g * 1e-3, tuple(L.gps_lead_com))], density_g_cm3=0.0)
    if getattr(L, "has_gns", False):           # v0.8.1: the GNS board (+ screws) on the -x tube wall
        add("gns", "placeholder", [_box_xyz(L.gns_x, L.gns_y, L.gns_z, 1.0)], mass_g=L.gns_mass_g)
    if getattr(L, "servo_leads", False):     # SPEC §3.8 / §6 (v0.6): vane-servo extension leads, tab-mount preset only
        add("servo_leads", "lumped", [_point(L.sl_lead_mass_g * 1e-3, tuple(L.sl_lead_com))], density_g_cm3=0.0)
    _apply_print_mass_model(L, p, parts)
    return parts


def _printed_base(name: str) -> str:
    stem = name.rsplit("_", 1)
    return stem[0] if len(stem) == 2 and stem[1].isdigit() else name


def _apply_print_mass_model(L: Layout, p: dict, parts: dict[str, Part]) -> None:
    """v0.9 (printer.mass_model 'slicer_part_g'): scale each estimated printed part to the effective density of its part
    in the slicer table (part grams / solid volume of the CAD part it was sliced from), else its print class's
    effective_density_fallback_g_cm3; the solid density (printer.density_g_cm3) stays for mode 'solid'."""
    pr = p["printer"]
    mm = pr.get("mass_model") or {}
    if mm.get("mode") != "slicer_part_g":
        return
    table = {}
    path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), mm.get("table", ""))
    if mm.get("table") and os.path.exists(path):
        with open(path, "r", encoding="utf-8") as f:
            table = json.load(f).get("parts", {})
    for name, pt in list(parts.items()):
        if pt.kind != "printed" or pt.volume_mm3 <= 0:
            continue
        base = _printed_base(name)
        ent = table.get(base)
        if ent and ent.get("part_g") and ent.get("solid_volume_mm3"):
            rho_eff = ent["part_g"] * 1000.0 / ent["solid_volume_mm3"]
        else:
            cls = (pr.get("part_class") or {}).get(base)
            if cls is None:
                continue
            rho_eff = pr["print_classes"][cls]["effective_density_fallback_g_cm3"]
        k = rho_eff / pt.density_g_cm3 if pt.density_g_cm3 > 0 else 1.0
        parts[name] = Part(pt.name, pt.kind, pt.mass_g * k, pt.com_mm, np.asarray(pt.inertia_about_com_kg_m2) * k,
                           pt.volume_mm3, rho_eff, pt.bbox_mm)


def _compose_parts(parts: dict[str, Part], exclude: tuple[str, ...] = ()) -> MassProps:
    """Sum report parts (g, mm, kg m^2) into one ``MassProps`` (SI)."""
    solids = [_Solid(pt.mass_g * 1e-3, np.asarray(pt.com_mm, dtype=float) * 1e-3,
                     np.asarray(pt.inertia_about_com_kg_m2, dtype=float), pt.volume_mm3 * 1e-9, np.asarray(pt.bbox_mm))
              for name, pt in parts.items()
              if not any(name == ex or name.startswith(ex + "_") for ex in exclude) and pt.mass_g != 0.0]
    s = _compose(solids)
    return MassProps(s.m, s.c, s.I)


def estimate_report(L: Layout, p: dict, params_path: str | None = None) -> dict[str, Any]:
    """Internal estimate in the exact CAD-report schema of SPEC §6."""
    parts = estimate_parts(L, p)
    full = _compose_parts(parts, exclude=NON_VEHICLE_PARTS)
    post = _compose_parts(parts, exclude=NON_VEHICLE_PARTS + EJECTED_PARTS)
    chute_mass = parts["chute_pack"].mass_g
    printed_mass = sum(pt.mass_g for pt in parts.values() if pt.kind == "printed")
    sha = ""
    if params_path and os.path.exists(params_path):
        with open(params_path, "rb") as f:
            sha = hashlib.sha256(f.read()).hexdigest()
    i = 1
    if getattr(L, "direct_drive", False):    # v0.9: no pushrod
        arm_tip = horn_tip = _polar_xyz(L.hinge_boss_r_out, L.vane_thetas[0], L.vane_hinge_z)
    else:
        arm_tip = _polar_xyz(L.servo_arm_tip_r, L.pod_thetas[0], 0.5 * (L.servo_arm_z_bottom + L.servo_arm_z_top),
                             L.servo_shaft_tangential_offset)
        horn_tip = _polar_xyz(0.5 * (L.horn_r_in + L.horn_r_out), L.vane_thetas[0], L.horn_tip_z - 2.0)
    return {
        "generator": "sim_internal_estimate",
        "generator_version": "0.1",
        "preset": p["meta"].get("preset"),
        "params_sha256": sha,
        "parts": {name: pt.as_report() for name, pt in parts.items()},
        "totals": {
            **full.as_report(),
            "printed_mass_g": printed_mass,
            "post_ejection": post.as_report(),
            "descent_mass_g": post.mass_kg * 1e3 + chute_mass,
        },
        "derived": {
            "pushrod_len_mm": float(np.linalg.norm(arm_tip - horn_tip)),
            "nose_cavity_volume_L": L.nose_cavity_volume_L,
            "footprint_d_mm": L.footprint_d,
            "vehicle_h_mm": L.vehicle_h,
            "inlet_area_ratio": L.inlet_area_ratio,
        },
        "warnings": _estimate_warnings(L),
    }


def _estimate_warnings(L: Layout) -> list[str]:
    """What the simplified-solid estimate leaves out.  lip_clamp keeps the v0.4 text verbatim; with the v0.5 tab
    mount the tab-screw holes ARE modelled, so the base line names only the holes that are not."""
    if not L.tabs:
        return ["simplified-solid estimate: pods include the 45 deg support wedge; "
                "screw holes, fillets and the vane D-flat are ignored"]
    if getattr(L, "clock_mount", False):
        return ["simplified-solid estimate: joint-screw and servo-ear holes, the servo-lead cutters, fillets and the "
                "vane D-flat are ignored",
                "wire09c clock tab mount modelled (simfix 2026-11-04): cylindrical skin, thin liner, keyways round the "
                "circle, floor pads, servo-lead column guide walls, lead-passage blister + keel and the passage cut (exact "
                "plan intersections in Z slabs); no lugs / tab screws / 22 mm slot; intake groove as before; edf_unit "
                "with bell ring, motor and tabs; the propulsion deck is not modelled (~+2.3 % volume vs the solid)"]
    return ["simplified-solid estimate: pods include the 45 deg support wedge; joint-screw and servo-ear holes, "
            "fillets and the vane D-flat are ignored (the v0.5 tab-screw holes are modelled)",
            "v0.5 tab mount modelled: cylindrical skin, thin liner (tube + top ring), one screw lug per tab and "
            "boss_z_ranges entry, keyways, tab-screw holes (clearance / counterbore to t_cbore_end / tap, exact "
            "material removed), two-box wire slot; intake bulge, wire channel, shelf notch, bulkhead wire hole and "
            "the v0.5d channel floor wedge; edf_unit with bell ring, motor and tabs; the propulsion deck is not modelled"]


# --------------------------------------------------------------------------- #
# report readers
# --------------------------------------------------------------------------- #
def _parts_from_report(rep: dict) -> dict[str, Part] | None:
    parts = rep.get("parts")
    if not isinstance(parts, dict) or not parts:
        return None
    out: dict[str, Part] = {}
    for name, d in parts.items():
        try:
            out[name] = Part(name, d.get("kind", "printed"), float(d["mass_g"]), np.asarray(d["com_mm"], dtype=float),
                             np.asarray(d["inertia_about_com_kg_m2"], dtype=float), float(d.get("volume_mm3", 0.0)),
                             float(d.get("density_g_cm3", 0.0)),
                             np.asarray(d.get("bbox_mm", [[0, 0, 0], [0, 0, 0]]), dtype=float))
        except (KeyError, TypeError, ValueError):
            return None
    return out


def _massprops_from_totals(d: dict) -> MassProps:
    return MassProps(float(d["mass_g"]) * 1e-3, np.asarray(d["com_mm"], dtype=float) * 1e-3,
                     np.asarray(d["inertia_about_com_kg_m2"], dtype=float))


def expected_report_parts(L: Layout, p: dict) -> list[str]:
    """Component names a complete SPEC §6 CAD report must carry: every part of the internal estimate (printed
    flight parts, placeholders, lumped items - exactly the SPEC §6 list).  ``launch_pad`` (ground equipment,
    excluded from every total) is optional.  Exact names: the parts are composed as listed, so a report that
    gives one ``vane`` for four would undercount."""
    return list(estimate_parts(L, p))


def missing_report_parts(rep: dict, expected: list[str] | tuple[str, ...] | set[str]) -> list[str]:
    """Expected components absent from ``rep['parts']`` (all of them when the report has no parts dict)."""
    parts = rep.get("parts")
    if not isinstance(parts, dict):
        return sorted(expected)
    return sorted(n for n in expected if n not in parts)


def load_cad_report(path: str, expected: list[str] | tuple[str, ...] | set[str] | None = None
                    ) -> tuple[MassProps, MassProps, float, list[str]]:
    """Read a SPEC §6 CAD report by recomposing its ``parts`` (so the launch pad can be excluded and the
    post-ejection state recomputed).  ``expected``: the components it must carry (default
    ``expected_report_parts`` of the report's own preset from this project's params.json); a report missing
    one, or without usable per-part data, raises ValueError - a report that dropped a part is no evidence, and
    its totals could not be checked either.  Returns (full, post_ejection, descent_mass_kg, warnings)."""
    with open(path, "r", encoding="utf-8") as f:
        rep = json.load(f)
    if expected is None:
        p_rep = layout_mod.load_params(os.path.join(PROJECT_ROOT, "params.json"), rep.get("preset"))
        expected = expected_report_parts(Layout(p_rep), p_rep)
    warnings: list[str] = []
    missing = missing_report_parts(rep, expected)
    if missing:
        raise ValueError(f"{os.path.basename(path)} is missing components {missing}")
    parts = _parts_from_report(rep)
    if parts is None:
        raise ValueError(f"{os.path.basename(path)}: per-part data unreadable (missing mass_g / com_mm / inertia)")
    totals = rep.get("totals", {})
    full = _compose_parts(parts, exclude=NON_VEHICLE_PARTS)
    post = _compose_parts(parts, exclude=NON_VEHICLE_PARTS + EJECTED_PARTS)
    chute = sum(pt.mass_g for n, pt in parts.items() if n == "chute_pack" or n.startswith("chute_pack_")) * 1e-3
    if "mass_g" in totals:
        dm = abs(float(totals["mass_g"]) * 1e-3 - full.mass_kg)
        if dm > 0.02 * full.mass_kg:
            warnings.append(f"{os.path.basename(path)}: totals.mass_g={totals['mass_g']:.0f} differs from the "
                            f"sum of parts (excluding launch_pad) {full.mass_kg * 1e3:.0f} g; using the parts sum")
    descent = float(totals.get("descent_mass_g", (post.mass_kg + chute) * 1e3)) * 1e-3
    return full, post, descent, warnings


def _find_key(obj: Any, names: tuple[str, ...], depth: int = 0) -> Any:
    """Depth-first search for the first key in ``names`` inside nested dicts."""
    if depth > 6 or not isinstance(obj, dict):
        return None
    for n in names:
        if n in obj:
            return obj[n]
    for v in obj.values():
        if isinstance(v, dict):
            hit = _find_key(v, names, depth + 1)
            if hit is not None:
                return hit
    return None


def load_sizing_report(path: str, est_full: MassProps, est_post: MassProps, est_descent: float
                       ) -> tuple[MassProps, MassProps, float, str, list[str]]:
    """Tolerant reader for ``sizing/sizing_report.json`` (schema owned by the
    sizing subsystem).  Takes mass / CG when it finds them and scales the
    internal inertia estimate; everything it could not find is labelled."""
    with open(path, "r", encoding="utf-8") as f:
        rep = json.load(f)
    warnings: list[str] = []
    used: list[str] = []
    totals = rep.get("totals") if isinstance(rep.get("totals"), dict) else None
    if totals and {"mass_g", "com_mm", "inertia_about_com_kg_m2"} <= set(totals):
        full = _massprops_from_totals(totals)
        pe = totals.get("post_ejection")
        post = _massprops_from_totals(pe) if pe else MassProps(est_post.mass_kg * full.mass_kg / est_full.mass_kg,
                                                                est_post.com_m, est_post.inertia_kg_m2)
        descent = float(totals.get("descent_mass_g", (post.mass_kg * 1e3 + (est_descent - est_post.mass_kg) * 1e3))) * 1e-3
        return full, post, descent, "sizing_report.json (CAD-schema totals)", warnings

    mass_g = _find_key(rep, ("total_mass_g", "mass_total_g", "takeoff_mass_g", "liftoff_mass_g", "mtow_g",
                             "vehicle_mass_g", "mass_g"))
    cg = _find_key(rep, ("cg_z_mm", "z_cg_mm", "com_z_mm", "cg_mm", "com_mm"))
    inertia = _find_key(rep, ("inertia_about_com_kg_m2", "inertia_kg_m2", "inertia"))
    post_mass_g = _find_key(rep, ("post_ejection_mass_g", "mass_post_ejection_g", "post_ejection"))
    if isinstance(post_mass_g, dict):
        post_mass_g = post_mass_g.get("mass_g")

    full_mass = est_full.mass_kg
    if isinstance(mass_g, (int, float)) and mass_g > 0:
        full_mass = float(mass_g) * 1e-3
        used.append("mass")
    com = est_full.com_m.copy()
    if isinstance(cg, (int, float)):
        com = np.array([0.0, 0.0, float(cg) * 1e-3])
        used.append("cg_z")
    elif isinstance(cg, (list, tuple)) and len(cg) == 3:
        com = np.asarray(cg, dtype=float) * 1e-3
        used.append("com")
    ratio = full_mass / est_full.mass_kg
    if isinstance(inertia, (list, tuple)) and np.asarray(inertia, dtype=float).shape == (3, 3):
        I = np.asarray(inertia, dtype=float)
        used.append("inertia")
    else:
        I = est_full.inertia_kg_m2 * ratio
        warnings.append("sizing_report.json has no inertia tensor; internal estimate scaled by the mass ratio")
    full = MassProps(full_mass, com, I)
    if isinstance(post_mass_g, (int, float)) and post_mass_g > 0:
        post_mass = float(post_mass_g) * 1e-3
        used.append("post_ejection_mass")
    else:
        post_mass = est_post.mass_kg * ratio
    post = MassProps(post_mass, est_post.com_m + (com - est_full.com_m), est_post.inertia_kg_m2 * (post_mass / est_post.mass_kg))
    descent = post_mass + (est_descent - est_post.mass_kg)
    if not used:
        warnings.append("sizing_report.json contained no recognisable mass keys; internal estimate used")
        return est_full, est_post, est_descent, "internal estimate (sizing_report.json unreadable)", warnings
    return full, post, descent, f"sizing_report.json ({', '.join(used)}) + internal estimate for the rest", warnings


# --------------------------------------------------------------------------- #
# aerodynamics (Barrowman)
# --------------------------------------------------------------------------- #
def body_top_mm(L: Layout) -> float:
    """Top of the vehicle above the exit plane [mm]: the nose tip (rocket) or the top cap (v0.9 VTOL configuration)."""
    zt = getattr(L, "z_nose_tip", None)
    return float(zt) if zt is not None else float(L.z_cap_top)


def body_shoulder_mm(L: Layout) -> float:
    """Top of the cylindrical body [mm]: the nose base (rocket) or the top cap (VTOL)."""
    zb = getattr(L, "z_nose_base", None)
    return float(zb) if zb is not None else float(L.z_cap_top)


def compute_aero(L: Layout, p: dict, z_cg_m: float) -> AeroRef:
    """Barrowman normal-force derivatives and centre of pressure (SPEC §9).

    * nose: C_Na = 2 (any nose), X_cp from the tip = L - V_nose / (pi R^2)
      (exact Barrowman body-of-revolution result, V_nose integrated from
      ``Layout.nose_outer_radius``);
    * fins (TIR-33 form, N = 4): C_Na = K 4N (s/d)^2 / (1 + sqrt(1 + (2 l/(Cr+Ct))^2)),
      K = 1 + R_f/(s+R_f) with R_f the radius of the duct section the fins sit on,
      d = body OD (reference), l = mid-chord sweep; X_f = X_B + X_R(Cr+2Ct)/(3(Cr+Ct)) + ((Cr+Ct) - CrCt/(Cr+Ct))/6;
    * body lift: ``aero.body_cn_alpha_per_rad`` at the planform centroid of
      the cylinder (an engineering allowance, not Barrowman).
    """
    a = p["aero"]
    body_r = L.body_r * 1e-3
    a_ref = math.pi * body_r * body_r
    d = L.body_od
    z_top = body_top_mm(L)
    if getattr(L, "nose_len", None):
        # nose
        n = 400
        x = (np.arange(n) + 0.5) / n * L.nose_len
        r = np.array([L.nose_outer_radius(xx) for xx in x])
        v_nose = float((math.pi * r * r).sum() * (L.nose_len / n))
        x_n = L.nose_len - v_nose / (math.pi * L.body_r ** 2)      # from the tip, mm
        z_cp_nose = L.z_nose_tip - x_n
        cn_nose = 2.0
    else:
        # v0.9 VTOL configuration: a flat top cap, no nose (Barrowman nose term 0 at the top; the VTOL 6-DOF uses the
        # cross-flow / ram-drag model of sim.sim6dof.vtol_aero_geometry, this AeroRef only feeds the axial drag)
        z_cp_nose, cn_nose = z_top, 0.0
    # fins
    s_span = L.fin_tip_r - L.vane_section_r
    cr, ct = L.fin_root_chord, L.fin_tip_chord
    root_mid = np.array([L.fin_root_r, 0.5 * (L.fin_te_z + L.fin_root_le_z)])
    tip_mid = np.array([L.fin_tip_r, 0.5 * (L.fin_te_z + L.fin_tip_le_z)])
    l_mid = float(np.linalg.norm(tip_mid - root_mid))
    n_fins = p["fin"]["count"]
    k_int = 1.0 + L.vane_section_r / (s_span + L.vane_section_r)
    cn_fins = k_int * 4.0 * n_fins * (s_span / d) ** 2 / (1.0 + math.sqrt(1.0 + (2.0 * l_mid / (cr + ct)) ** 2))
    x_b = z_top - L.fin_root_le_z
    x_r = L.fin_root_le_z - L.fin_tip_le_z   # tip LE aft of root LE (positive aft)
    x_f = x_b + x_r / 3.0 * (cr + 2.0 * ct) / (cr + ct) + (1.0 / 6.0) * ((cr + ct) - cr * ct / (cr + ct))
    z_cp_fins = z_top - x_f
    # body
    cn_body = float(a["body_cn_alpha_per_rad"])
    z_cp_body = 0.5 * (L.z_exit + body_shoulder_mm(L))
    cn_nf = cn_nose + cn_fins
    z_cp_nf = (cn_nose * z_cp_nose + cn_fins * z_cp_fins) / cn_nf
    cn_tot = cn_nf + cn_body
    z_cp = (cn_nose * z_cp_nose + cn_fins * z_cp_fins + cn_body * z_cp_body) / cn_tot
    return AeroRef(
        a_ref_m2=a_ref, cd=float(a["body_cd"]),
        cn_alpha_nose=cn_nose, z_cp_nose_m=z_cp_nose * 1e-3,
        cn_alpha_fins=cn_fins, z_cp_fins_m=z_cp_fins * 1e-3,
        cn_alpha_body=cn_body, z_cp_body_m=z_cp_body * 1e-3,
        cn_alpha_total=cn_tot, z_cp_m=z_cp * 1e-3,
        z_cp_nose_fins_m=z_cp_nf * 1e-3, cn_alpha_nose_fins=cn_nf,
        static_margin_cal=(z_cg_m - z_cp_nf * 1e-3) / (L.body_od * 1e-3),
        static_margin_total_cal=(z_cg_m - z_cp * 1e-3) / (L.body_od * 1e-3),
    )


# --------------------------------------------------------------------------- #
# the vehicle
# --------------------------------------------------------------------------- #
@dataclass
class Vehicle:
    """Everything the simulation and controller need, in SI, with provenance."""

    params: dict
    layout: Layout
    mass_source: str
    source_path: str | None
    full: MassProps
    post_ejection: MassProps
    descent_mass_kg: float
    aero: AeroRef
    warnings: list[str] = field(default_factory=list)
    loaded_at: str = field(default_factory=lambda: _dt.datetime.now().isoformat(timespec="seconds"))
    hardware_config: str = "rocket"     # v0.9: params mission.config of the loaded vehicle ('rocket' | 'vtol')
    power_source: str = "battery"       # v0.9: power.source of the loaded vehicle ('battery' | 'tether' | 'hybrid')
    part_names: tuple = ()              # the components the mass properties were composed from (empty: an estimate)

    # ---- v0.9 configuration geometry -----------------------------------------
    @property
    def has_chute(self) -> bool:
        """The vehicle carries a parachute (the rocket configuration); the VTOL configuration has none (SPEC 13)."""
        if self.part_names:
            return any(n == "chute_pack" or n.startswith("chute_pack_") for n in self.part_names)
        return self.hardware_config != "vtol"

    @property
    def tether_attach_body_m(self) -> np.ndarray | None:
        """The tether hardpoint (top of the M5 eye bolt on the top cap axis), vehicle frame [m]; None without a top
        cap (the rocket).  Layout: z_cap_top + cap_eye_h (eye height ASSUMED, params top_cap.tether.eye_h_ASSUMED)."""
        L = self.layout
        if getattr(L, "z_cap_top", None) is None:
            return None
        return np.array([0.0, 0.0, (float(L.z_cap_top) + float(getattr(L, "cap_eye_h", 0.0))) * 1e-3])

    @property
    def gps_antenna_mm(self) -> tuple[float, float, float] | None:
        """GPS antenna phase centre, vehicle frame [mm]: the rocket pod (Layout.gps_antenna_xyz) or the VTOL top-cap patch
        (centre of cap_patch_xy at the patch top cap_patch_z[1], ASSUMED phase centre at the patch face)."""
        L = self.layout
        a = getattr(L, "gps_antenna_xyz", None)
        if a is not None:
            return tuple(float(v) for v in a)
        px, pz = getattr(L, "cap_patch_xy", None), getattr(L, "cap_patch_z", None)
        if px is None or pz is None:
            return None
        return (0.5 * (px[0][0] + px[0][1]), 0.5 * (px[1][0] + px[1][1]), float(pz[1]))

    def footprint_radius_m(self, direction_body_rad: float) -> float:
        """Distance from the vehicle axis to the edge of the solid-fin footprint (the square through the four fin tips,
        Layout fin_thetas / fin_tip_r) along a body-frame direction [m] (a fin: fin_tip_r; between two fins: fin_tip_r
        cos 45 deg)."""
        L = self.layout
        r = float(L.fin_tip_r) * 1e-3
        n = len(L.fin_thetas)
        if n < 3:
            return r
        half = math.pi / n
        rel = (direction_body_rad - math.radians(float(L.fin_thetas[0]))) % (2.0 * half)
        return r * math.cos(half) / math.cos(rel - half)

    # ---- propulsion / control geometry -------------------------------------
    @property
    def z_cg_m(self) -> float:
        return float(self.full.com_m[2])

    @property
    def l_v_m(self) -> float:
        """Vane lever arm l_v = z_CG - vane_hinge_z (SPEC §8.4)."""
        return self.z_cg_m - self.layout.vane_hinge_z * 1e-3

    @property
    def r_v_m(self) -> float:
        return self.params["vane"]["force_centroid_r"] * 1e-3

    @property
    def s_v_m2(self) -> float:
        return self.layout.vane_area_mm2 * 1e-6

    @property
    def a_e_m2(self) -> float:
        """Exhaust annulus area A_e = pi/4 (duct_id^2 - (hub_frac duct_id)^2)."""
        d = self.layout.duct_id * 1e-3
        hub = self.params["edf"]["exit_hub_fraction"] * d
        return math.pi / 4.0 * (d * d - hub * hub)

    @property
    def t_max_n(self) -> float:
        return float(self.params["edf"]["thrust_nominal_N"])

    @property
    def weight_n(self) -> float:
        return self.full.mass_kg * G0

    @property
    def thrust_to_weight(self) -> float:
        return self.t_max_n / self.weight_n

    @property
    def hover_throttle(self) -> float:
        """u with T_max u^exp = m g."""
        return (self.weight_n / self.t_max_n) ** (1.0 / self.params["edf"]["throttle_exponent"])

    def vane_moment_gains(self, thrust_n: float) -> tuple[float, float, float]:
        """(dM_x/d delta_x, dM_y/d delta_y, dM_z/d delta_z) in N m/rad at a given
        thrust - the §8.4 formulas with q_e = T / (2 A_e)."""
        q_e = thrust_n / (2.0 * self.a_e_m2)
        k = q_e * self.s_v_m2 * self.params["vane"]["cl_alpha_per_rad"]
        return 2.0 * self.l_v_m * k, 2.0 * self.l_v_m * k, 4.0 * self.r_v_m * k

    def summary(self) -> dict[str, Any]:
        I = self.full.inertia_kg_m2
        mx, my, mz = self.vane_moment_gains(self.t_max_n)
        dmax = math.radians(self.params["vane"]["max_deflection_cmd_deg"])
        return {
            "mass_source": self.mass_source,
            "source_path": self.source_path,
            "loaded_at": self.loaded_at,
            "mass_kg": self.full.mass_kg,
            "post_ejection_mass_kg": self.post_ejection.mass_kg,
            "descent_mass_kg": self.descent_mass_kg,
            "z_cg_mm": self.z_cg_m * 1e3,
            "z_cg_post_ejection_mm": float(self.post_ejection.com_m[2]) * 1e3,
            "inertia_diag_kg_m2": [float(I[0, 0]), float(I[1, 1]), float(I[2, 2])],
            "l_v_mm": self.l_v_m * 1e3,
            "thrust_to_weight": self.thrust_to_weight,
            "hover_throttle": self.hover_throttle,
            "z_cp_mm": self.aero.z_cp_m * 1e3,
            "z_cp_nose_fins_mm": self.aero.z_cp_nose_fins_m * 1e3,
            "static_margin_cal_nose_fins": self.aero.static_margin_cal,
            "cn_alpha_total": self.aero.cn_alpha_total,
            "pitch_authority_Nm_at_Tmax_full_delta": mx * dmax,
            "roll_authority_Nm_at_Tmax_full_delta": mz * dmax,
            "pitch_accel_rad_s2_at_Tmax_full_delta": mx * dmax / float(I[0, 0]),
            "warnings": list(self.warnings),
        }


def load_vtol_vehicle(root: str, p: dict, params_path: str, power: str) -> Vehicle:
    """v0.9 VTOL hardware configuration (SPEC 13): the mass properties of ``cad/exports/cadquery_report<_preset>_vtol
    <_power>.json`` (the CAD numbers - the internal estimate does not model the top cap / carrier / tether kit).  The
    report must be for this preset and configuration / power source, list no failed components and carry
    ``VTOL_REQUIRED_PARTS`` + ``VTOL_POWER_PARTS[power]``; a params sha256 differing from this params.json is used and
    flagged (as for the rocket).  No usable report -> ValueError (no estimate fallback for this configuration)."""
    L = Layout(p)
    preset_name = p["meta"].get("preset")
    with open(params_path, "rb") as f:
        sha = hashlib.sha256(f.read()).hexdigest()
    suffix = preset_suffix(preset_name, layout_mod.load_params(params_path)["meta"].get("preset"))
    csuf = layout_mod.config_suffix("vtol", power)
    need = list(VTOL_REQUIRED_PARTS) + list(VTOL_POWER_PARTS.get(power, ()))
    warnings: list[str] = []
    for rel in CAD_REPORT_CANDIDATES:
        stem, ext = os.path.splitext(rel)
        path = os.path.join(root, f"{stem}{suffix}{csuf}{ext}")
        if not os.path.exists(path):
            continue
        name = os.path.basename(path)
        with open(path, "r", encoding="utf-8") as f:
            rep = json.load(f)
        if (rep.get("preset") != preset_name or rep.get("config", "vtol") != "vtol"
                or (rep.get("power_source") or "battery") != power):
            warnings.append(f"{name} is for preset {rep.get('preset')!r} / config {rep.get('config')!r} / power "
                            f"{rep.get('power_source')!r}, not {preset_name!r} / 'vtol' / {power!r} - ignored")
            continue
        if rep.get("failed_components"):
            warnings.append(f"{name} lists failed components {rep['failed_components']} - ignored")
            continue
        miss = missing_report_parts(rep, need)
        if miss:
            warnings.append(f"{name} is missing components {miss} - ignored")
            continue
        if rep.get("params_sha256") != sha:
            warnings.append(f"{name} was generated from a different params.json "
                            f"(sha {str(rep.get('params_sha256'))[:12]}... vs {sha[:12]}...)")
        names = tuple(n for n in rep["parts"] if n not in NON_VEHICLE_PARTS)
        full, post, descent, w = load_cad_report(path, list(names))
        aero = compute_aero(L, p, float(full.com_m[2]))
        return Vehicle(p, L, f"{name} (generator={rep.get('generator', '?')})", path, full, post, descent, aero,
                       warnings + w, hardware_config="vtol", power_source=power, part_names=names)
    raise ValueError(f"no usable CAD report for the VTOL configuration ({power}): {warnings or 'none found'}")


def load_vehicle(root: str = PROJECT_ROOT, preset: str | None = None, prefer: str | None = None,
                 params_path: str | None = None, config: str | None = None, power: str | None = None) -> Vehicle:
    """Build the ``Vehicle``.  ``prefer``: ``None`` (priority order), ``"cad"``,
    ``"sizing"`` or ``"estimate"`` (skip all files).

    Report files are looked up per preset (``report_candidates``): an alt preset reads
    ``cadquery_report_<preset>.json`` / ``sizing_report_<preset>.json``.  As in ``sizing.find_cad_report``, a
    report whose ``preset`` is not the requested one is skipped with a warning (a CAD report must carry the
    key; a sizing report is skipped only when it names another preset) and a ``params_sha256`` that differs
    from this params.json is used but flagged.  A CAD report that lists failed components or misses a component
    of the internal estimate (``expected_report_parts``; launch_pad optional) is skipped with a warning."""
    params_path = params_path or os.path.join(root, "params.json")
    if config == "vtol":
        p = layout_mod.load_params(params_path, preset, "vtol", power or "battery")
        return load_vtol_vehicle(root, p, params_path, p["power"]["source"])
    p = layout_mod.load_params(params_path, preset, config, power)
    L = Layout(p)
    est = estimate_report(L, p, params_path)
    est_full = _massprops_from_totals(est["totals"])
    est_post = _massprops_from_totals(est["totals"]["post_ejection"])
    est_descent = est["totals"]["descent_mass_g"] * 1e-3
    expected = [n for n in est["parts"] if n not in NON_VEHICLE_PARTS]   # = expected_report_parts(L, p)
    warnings: list[str] = []
    full, post, descent = est_full, est_post, est_descent
    source, source_path = "internal estimate (Layout + params.json simple solids)", None
    preset_name, sha = p["meta"].get("preset"), est["params_sha256"]
    suffix = preset_suffix(preset_name, layout_mod.load_params(params_path)["meta"].get("preset"))

    candidates = [(kind, os.path.join(root, rel)) for kind, rel in report_candidates(suffix)
                  if prefer is None or prefer == kind]
    for kind, path in candidates:
        if not os.path.exists(path):
            continue
        name = os.path.basename(path)
        try:
            with open(path, "r", encoding="utf-8") as f:
                rep = json.load(f)
            if not isinstance(rep, dict):
                raise ValueError("not a JSON object")
            if (kind == "cad" or "preset" in rep) and rep.get("preset") != preset_name:
                warnings.append(f"{name} is for preset {rep.get('preset')!r}, not {preset_name!r} - ignored")
                continue
            if kind == "cad" and rep.get("failed_components"):
                warnings.append(f"{name} lists failed components {rep['failed_components']} - ignored")
                continue
            if kind == "cad" and missing_report_parts(rep, expected):
                warnings.append(f"{name} is missing components {missing_report_parts(rep, expected)} "
                                f"(expected every SPEC §6 part; launch_pad optional) - ignored")
                continue
            if (kind == "cad" or "params_sha256" in rep) and rep.get("params_sha256") != sha:
                warnings.append(f"{name} was generated from a different params.json "
                                f"(sha {str(rep.get('params_sha256'))[:12]}... vs {sha[:12]}...)")
            if kind == "cad":
                full, post, descent, w = load_cad_report(path, expected)
                source = f"{name} (generator={rep.get('generator', '?')})"
            else:
                full, post, descent, source, w = load_sizing_report(path, est_full, est_post, est_descent)
            warnings += w
            source_path = path
            break
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
            warnings.append(f"could not use {path}: {exc!r}; trying the next source")
    if source_path is None and prefer not in (None, "estimate"):
        warnings.append(f"preferred source {prefer!r} not found; internal estimate used")
    aero = compute_aero(L, p, float(full.com_m[2]))
    names: tuple = ()
    if source_path is not None and "generator=" in source:            # a CAD report: its component list
        with open(source_path, "r", encoding="utf-8") as f:
            names = tuple(n for n in (json.load(f).get("parts") or {}) if n not in NON_VEHICLE_PARTS)
    return Vehicle(p, L, source, source_path, full, post, descent, aero, warnings,
                   hardware_config=str((p.get("mission") or {}).get("config") or "rocket"), part_names=names)


if __name__ == "__main__":
    veh = load_vehicle()
    print(json.dumps(veh.summary(), indent=2, ensure_ascii=False))
    rep = estimate_report(veh.layout, veh.params, os.path.join(PROJECT_ROOT, "params.json"))
    print(f"\ninternal estimate: total {rep['totals']['mass_g']:.0f} g, printed {rep['totals']['printed_mass_g']:.0f} g, "
          f"CG z {rep['totals']['com_mm'][2]:.1f} mm, post-ejection {rep['totals']['post_ejection']['mass_g']:.0f} g")
    for name, pt in sorted(rep["parts"].items(), key=lambda kv: -kv[1]["mass_g"]):
        print(f"  {name:<26} {pt['kind']:<11} {pt['mass_g']:8.1f} g  z={pt['com_mm'][2]:7.1f}")
