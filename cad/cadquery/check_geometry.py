"""Geometry checks C1-C27 of SPEC section 7 against the CadQuery twin (C24: v0.6.2 radio/log box, C25: v0.6.3 FCU board,
C26: v0.8 GPS pod, C27: v0.8.1 GNS board; C36: the 2026-11-04 livery - YUCHAN inlay, check_livery).

Invariant: every PASS is backed by a measurement on the solids (boolean intersection volume,
bounding box, OCC mass properties) or on Layout/params numbers - never by inspection of the
code.  "No intersection" means boolean common volume < INTERSECT_TOL (1 mm^3).  Rows X1..X3
are extra informational checks (WARN at most, never FAIL).  C18-C21 check the v0.5 EDF tab
mount and are reported ``N/A`` (never FAIL) when ``edf.mount`` is not ``tabs``.  v0.5d: C7 also needs
positive evidence of the tray stops, C16 of the blind propulsion joint holes, C19 that the head tunnel crosses the
skin, C15's latched overlap is positive evidence (uncomputable -> 0), and C20's step-out window and floor closure
follow the 45 deg floor wedge.  v0.6: C22 checks the COM board (second ATmega128A) under the payload bulkhead on the
solids - BRep clearances to the tray / placeholders / rails / spigot bore (uncomputable -> 0 -> FAIL), the board
seated on the boss tips, solid bosses around clear blind holes with a floor, open bulkhead holes - and is N/A (never
FAIL) for params without electronics.com / payload_module.com_mount.  C14 includes the ``com`` placeholder.
v0.6 servo leads: C23 checks the vane-servo lead route (SPEC 3.8 / 5.1) with ribbon proxies and solid evidence (servo
bulge closed, deck seat and sleeve intact) and is N/A without ``Layout.servo_leads``; C20's skin-opening test allows
the servo channel behind the servo bulge, C21's keyway probes stop at the rebate floor, C10 lists the route's walls.
v0.6.1 intake band (SPEC 5.1, window_wall_t thick, radial-sector windows): C9 judges the MINIMUM opening of the built
band (probe shells across it) and the Layout minimum at r_band_in, and adds solid evidence - every window clear through,
every strut's mid-height section >= 0.95 x the design sector and window_wall_t deep, the band window_wall_t thick and
solid in the margin rings; C10 lists window_wall_t, the strut inner face and the shelf ledge past the band; C20's upper
gate is taken to the band's inner face, and C20 / C23 measure the BRep gap from the leg that runs up beside the strut
to the band (> 0 / >= servo_leads.min_clearance).
v0.6.2 (window gable, radio/log box): C9 probes the windows clear ABOVE the lower boundary only, the gussets solid under
it, the bridge flat at z_window_bottom, and every lower-boundary edge on the outer and inner band surface (BRep edges)
flat within the bridge or >= GABLE_MIN_EDGE_SLOPE_DEG from horizontal, spanning the full gable height on both sides of
every window (printability evidence); C10 notes the gussets (no new minimum feature); C14 includes ``radio_log``; the new
row C24 checks the radio/log box under the payload bulkhead on the solids (N/A without electronics.radio_log).
v0.6.3 (three ATmega128A): the tray placeholder ``fc`` is ``nav`` now (C7 / C14 / C22 / C24 test it under that name); C14
includes the new ``fcu`` placeholder and the new row C25 checks the FCU board on the intake bulkhead on the solids
(N/A without electronics.fcu) - a separate row, gated on its own params key like C24.
v0.8 (GPS pod, payload_module.gps_pod): the new row C26 checks the pod, its lid, the antenna stack, the raceway / connector
pocket and the avionics entry hole on the solids (N/A without the pod); C14 includes the ``gps`` / ``gps_ground_plane``
placeholders (zone: footprint radius over the payload module, clear of the payload module and the lid); C10 lists the pod's
walls (designed) and its hole / post walls (derived, Layout.gps_walls); C1 / C2 include the new printed part ``gps_pod_lid``.
v0.8.1 (GNS board, electronics.gns / avionics_module.gns_mount): the new row C27 checks the GNS board on its four bosses on
the -x avionics tube wall on the solids (clearances, seat on the boss tips, blind tap holes, 45 deg boss faces, the GPS /
glink lead tubes; N/A without electronics.gns); C7 / C14 include the ``gns`` placeholder (avionics zone); C10 lists the
GNS boss walls (derived, Layout.gns_walls).

Writes cad/exports/cadquery_checks.md + .json (``_<preset>`` variants for non-default presets)
and exits 1 when any C-row FAILs.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass, field

import cadquery as cq

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import build_rocket as br  # noqa: E402
import check_clock as ckc  # noqa: E402  (wire09c clock mount: C18 / C21 / C29 clock versions, C34, C35)
import check_livery as ckl  # noqa: E402  (livery 2026-11-04: C36)
import rocket_geometry as rg  # noqa: E402
from layout import GABLE_MIN_EDGE_SLOPE_DEG, PLACEHOLDER_BORE_CLEAR_MM, Layout, load_params  # noqa: E402

INTERSECT_TOL_MM3 = 1.0
PASS, WARN, FAIL, NA = "PASS", "WARN", "FAIL", "N/A"
G0 = 9.80665

# SPEC 7 literals of the v0.5 tab-mount checks
C18_FLOOR_GAP_MAX = 0.05        # C18: keyway floor touches the tab (gap < 0.05 mm)
C19_CLEAR_PROBE_D = 3.0         # C19: clearance-hole probe O3.0 over t in [t_keyway, t_head]
C19_TUNNEL_PROBE_D = 6.0        # C19: head tunnel probe O6.0 over t in [t_head + 0.2, t_cbore_end + 2]
C19_TUNNEL_PAST_END = 2.0       # C19: the tunnel probe runs to t_cbore_end + 2
C19_TAP_FREE_PROBE_D = 2.0      # C19: tap hole is open: O2.0 probe over [t_tap_end + 0.2, -t_keyway] x module = 0
C19_TAP_MAT_PROBE_D = 3.2       # C19: material to self-tap: O3.2 probe there x module >= 0.95 x pi/4 (3.2^2 - 2.5^2) x len
C19_TAP_MAT_FRACTION = 0.95     # C19: (the 2.5 of that formula is the tap hole, tab_screw_d_tap)
C19_PROBE_INSET = 0.2           # C19: probes start 0.2 mm past the counterbore floor / tap-hole bottom
SHANK_PROBE_D = 3.0             # C19: O3.0 probe for |t| <= tab_t/2 x edf_unit = 0 (the screw passes the tab slot)
C19_SLOT_CONTROL_EXTRA_D = 1.0  # C19: positive control, a O(tab_slot_w + 1.0) probe there x edf_unit > 0
C21_MIN_SKIN_WALL = 0.8         # C21: material between each keyway (centre and corners) and the outer surface >= 0.8 mm
C18_FLOOR_AREA_FRACTION = 0.95  # C18: floor area under the tab >= 0.95 x tab_t x (tab_r_out - edf_bore_r)
C19_TUNNEL_TOL_MM3 = 0.01       # C19: the head tunnel must be clear (a clearance, not the generic 1 mm^3 tolerance)
C20_CLOSED_TOL_MM3 = 0.5        # C20: bulge walls / floor / roof and the intake skin have no unintended opening
C21_SLOT_VOID_TOL_MM3 = 1.0     # C21: the skin sector beside the wire slot is intact
# modelling helpers of the checks
FLOOR_PROBE_H = 2.0             # C18: height of the box under the tab that finds the keyway floor
FLOOR_AREA_SLAB = 0.1           # C18: thin slab under the tab whose material / height is the floor bearing area
FEATURE_EPS = 1e-6              # C10: float round-off allowance (134.05 - 132.85 = 1.1999999999999886 is 1.2)
C21_Z_STATIONS = 48             # C21: radial probes at the midpoints of this many equal Z intervals
C21_LOCAL_PAD = 3.0             # C21: the module is first clipped to a box this much larger than the probed region
TRAY_STOP_FILL_MIN = 0.95       # C7 (v0.5d tray stop evidence, SPEC 5.2): the FLOOR_AREA_SLAB-thin slab under the
                                # tray footprint in each rail slot is >= 95 % avionics_module material (as C18's floor)
TRAY_STOP_TOP_TOL = 0.05        # C7: the stop's top face is the tray's bottom face (|gap| < 0.05 mm, as C18_FLOOR_GAP_MAX)
JOINT_FLOOR_FILL_MIN = 0.95     # C16 (v0.5d blind propulsion joint holes, SPEC 3.2): the probe between the EDF bore
                                # and the hole bottom is >= 95 % module material (the hole is blind)
# v0.6 C22 (COM board, SPEC 5.3 / 6)
COM_FILL_MIN = 0.95             # C22: boss ring and the floor above each blind hole are >= 95 % payload_module material
COM_HOLE_CLEAR_TOL_MM3 = 0.01   # C22: a probe Ø(hole_d - 2 x COM_PROBE_INSET) in the blind hole meets no material (a
                                # clearance, as C19's tunnel; the 1 mm^3 tolerance would hide a 2.0 hole cut for 2.5)
COM_PROBE_INSET = 0.05          # C22: probes keep 0.05 mm off the faces they pin (hole wall, hole end, boss tip, ...)
COM_SEAT_TOL = 0.05             # C22: the lowest payload_module material over the board is the boss tip (|gap| < 0.05)
# v0.6.1 C9 (intake band, SPEC 5.1)
C9_STRUT_SECTION_MIN = 0.95     # C9: each strut's mid-height section >= 0.95 x the design sector (strut_w at body_r,
                                # radial sides, window_wall_t)
C9_WINDOW_CLEAR_TOL_MM3 = 0.01  # C9: a window probe (inset C9_PROBE_INSET) meets no material - a clearance, as C19's
                                # tunnel (the 1 mm^3 tolerance would hide a 0.1 mm skin left across a window)
C9_PROBE_INSET = 0.05           # C9: window probes stay 0.05 mm inside the window faces, ring probes inside the band
C9_WINDOW_PROBE_PAST = 0.5      # C9: window / strut probes run 0.5 mm past the band faces radially
C9_RADIAL_TOL = 0.01            # C9: radial probes find the band at r_band_in .. body_r within 0.01 mm
# v0.6.2 C9 (window gable, SPEC 5.1 / intake_module.window_bottom_end)
C9_GUSSET_VOID_TOL_MM3 = 0.01   # C9: the inset gusset region (below the boundary) is solid - void < 0.01 mm^3 (as the
                                # window clearance: a square-ended window leaves ~2000 mm^3 per window void there)
C9_EDGE_TOL = 1e-3              # C9: an edge lies on a band surface / at a Z station within 1e-3 mm
C9_EDGE_SAMPLES = 21            # C9: tangent / position samples per lower-boundary edge
# v0.6 C23 (servo leads, SPEC 3.8 / 5.1)
C23_CLOSED_TOL_MM3 = 0.5        # C23: servo bulge walls / floor / roof, the deck seat and the sleeve are solid (as C20)
C23_PROBE_INSET = 0.05          # C23: solid-evidence probes stay 0.05 mm inside the faces they test
# v0.6.3 C25 (FCU board on the intake bulkhead)
FCU_SEAT_FILL_MIN = 0.5         # C25: more than half of the board footprint is bulkhead material for the foam tape (a
                                # board over a hole has nothing to stick to)
FCU_SEAT_FILL_TOL = 1e-3        # C25: that measured fill equals Layout.fcu_seat_fraction within 1e-3 (when it is exact)
FCU_TIE_PROBE_D = 1.0           # C25: a O1 rod from one vent-hole centre to another at the board's mid height meets the
                                # board when the cable tie between them runs across it
# v0.8 C26 (GPS pod)
GPS_PROBE_INSET = 0.05          # C26: probes keep 0.05 mm off the faces they pin (hole walls, raceway faces, patch top)
GPS_CLEAR_TOL_MM3 = 0.01        # C26: a clearance probe (hole, raceway, sky column, bore) meets no material - as C22's
                                # hole probe (the 1 mm^3 tolerance would hide a 0.1 mm skin across the raceway)
# v0.9 C5 (direct drive: servo in the fin-root fairing) and C28 (coupler alignment)
C5_WALL_PROBE = 1.0             # C5: probes 1 mm deep beside / under the lower ear tab's slot find its walls and floor
C5_PROBE_INSET = 0.05           # C5: probes keep 0.05 mm off the faces they pin
C5_FILL_MIN = 0.95              # C5: slot walls / floor and the ring around each tap hole are >= 95 % material
C5_SWEEP_PAST = 5.0             # C5: the slide-out sweep runs the servo's width + 5 mm past the fairing face
C5_CLEAR_TOL_MM3 = 0.01         # C5: tap / clearance-hole probes meet no material (a clearance, as C19's tunnel)
C5_TAP_RING = 1.0               # C5: material ring 1 mm thick around each cover-screw tap hole
C28_AXIS_TOL = 0.01             # C28: the horn (servo output) centre lies on the hinge axis within 0.01 mm
C28_TORQUE_MARGIN_MIN = 2.0     # C28: servo torque >= 2 x the worst hinge moment
# v0.9 C20 / C29 (open motor-lead groove, bullets)
C29_CLEAR_TOL_MM3 = 0.01        # C29: a bullet / lead proxy (already grown by its clearance) meets no material
C33_TRIM_PASS = 0.05            # C33 (v0.9 fix round): standing trim from the lateral CG <= 5 % of the hover pitch authority
C33_TRIM_WARN = 0.10            # ... WARN up to 10 %
# v0.8.1 C27 (GNS board)
GNS_SEAT_SLAB = 0.1             # C27: slab behind the board's back face whose avionics material / thickness is the
                                # bearing area of the four boss tips (as C18's FLOOR_AREA_SLAB)
GNS_SEAT_AREA_TOL = 0.01        # C27: that area equals 4 x (diamond - hole) within 1 %
GNS_FACE_PROBE_W = 0.1          # C27: thin columns that find a boss's lowest material (bottom vertex / mid-face)
GNS_FACE_TOL = 0.1              # C27: ... at the 45-deg design height within 0.1 mm (the column is 0.1 wide)


@dataclass
class Check:
    id: str
    title: str
    status: str
    measured: str
    detail: str = ""
    values: dict = field(default_factory=dict)


def _v(a: cq.Shape, b: cq.Shape) -> float:
    """Intersection volume for "no intersection" tests: an uncomputable intersection counts as +inf (FAIL)."""
    v = rg.intersection_volume(a, b)
    return math.inf if math.isnan(v) else v


def _vpos(a: cq.Shape, b: cq.Shape) -> float:
    """Intersection volume used as positive evidence ("material is there"): uncomputable counts as 0 (FAIL)."""
    v = rg.intersection_volume(a, b)
    return 0.0 if math.isnan(v) else v


def _void(region: cq.Shape, solid: cq.Shape) -> float:
    """Volume of region not covered by solid (region minus solid); uncomputable counts as +inf (FAIL)."""
    try:
        rest = region.cut(solid)
        return 0.0 if rest is None or rest.wrapped.IsNull() else float(rest.Volume())
    except Exception:  # noqa: BLE001
        return math.inf


def _status_no_intersection(vmax: float) -> str:
    return PASS if vmax < INTERSECT_TOL_MM3 else FAIL


def _box(x0, x1, y0, y1, z0, z1) -> cq.Shape:
    return rg._box(x0, x1, y0, y1, z0, z1)


def _top_z(shape: cq.Shape, region: cq.Shape) -> float:
    """Highest Z of shape's material inside region; -inf when there is none (or OCC cannot compute it)."""
    try:
        common = shape.intersect(region)
        if common is None or common.wrapped.IsNull() or common.Volume() <= 1e-9:
            return -math.inf
        return common.BoundingBox().zmax
    except Exception:  # noqa: BLE001
        return -math.inf


def _bottom_z(shape: cq.Shape, region: cq.Shape) -> float:
    """Lowest Z of shape's material inside region; +inf when there is none (or OCC cannot compute it)."""
    try:
        common = shape.intersect(region)
        if common is None or common.wrapped.IsNull() or common.Volume() <= 1e-9:
            return math.inf
        return common.BoundingBox().zmin
    except Exception:  # noqa: BLE001
        return math.inf


def _clearance(a: cq.Shape, b: cq.Shape) -> float:
    """Minimum BRep distance (mm) between two solids, used as a clearance (>= limit): 0 when they touch or overlap,
    and 0 when OCC cannot compute it (no evidence is never PASS)."""
    try:
        d = float(a.distance(b))
    except Exception:  # noqa: BLE001
        return 0.0
    return d if math.isfinite(d) else 0.0


def _off_axis_ring(x: float, y: float, r_in: float, r_out: float, z0: float, z1: float) -> cq.Shape:
    """Vertical annulus r_in..r_out about the vertical line through (x, y), Z [z0, z1] (C22 boss probe)."""
    return rg._cyl_z(r_out, z0, z1, x, y).cut(rg._cyl_z(r_in, z0 - 1.0, z1 + 1.0, x, y))


def _pt_seg_dist(p: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
    """Distance from point p to the segment a-b in a plane."""
    (px, py), (ax, ay), (bx, by) = p, a, b
    dx, dy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


class Checker:
    def __init__(self, L: Layout, parts: dict[str, rg.Part], report: dict):
        self.L = L
        self.parts = parts
        self.report = report
        self.shape = {n: p.shape for n, p in parts.items() if p.shape is not None}
        self.prop = self.shape["propulsion_module"]
        self._band_part: cq.Shape | None = None

    def _intake_band_part(self) -> cq.Shape:
        """v0.6.1: the intake module's windowed band above the lead notches / channels (rocket_geometry.
        intake_opening_z_range), r from 1 mm inside r_band_in to 1 mm outside body_r - what the C20 / C23 legs that run
        up beside the struts must keep clear of (BRep distance)."""
        if self._band_part is None:
            z0, z1 = rg.intake_opening_z_range(self.L)
            self._band_part = self.shape["intake_module"].intersect(
                rg._tube(self.L.body_r + 1.0, self.L.r_band_in - 1.0, z0, z1))
        return self._band_part

    # ------------------------------------------------------------------ #
    def c1_bed_fit(self) -> Check:
        pr = self.L.p["printer"]
        bed_xy = pr["bed_x"] - 2 * pr["usable_margin"]
        bed_z = pr["bed_z"] - pr["usable_margin"]
        rows, worst, fails = [], 0.0, []
        seen = set()
        for p in self.parts.values():
            if p.kind != "printed" or p.base in seen:
                continue
            seen.add(p.base)
            rz = rg.plate_rot_z(self.L, p.base)          # v0.9: the propulsion module turns 45 deg on the plate
            bb = (p.shape.rotate(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), rz) if rz else p.shape).BoundingBox()
            ext = sorted([bb.xlen, bb.ylen, bb.zlen])
            if p.orientation == "flat":
                h, xy = ext[0], (ext[1], ext[2])
            else:  # upright / plate_down: vehicle Z is the print Z
                h, xy = bb.zlen, (bb.xlen, bb.ylen)
            ratio = max(xy[0] / bed_xy, xy[1] / bed_xy, h / bed_z)
            worst = max(worst, ratio)
            ok = xy[0] <= bed_xy and xy[1] <= bed_xy and h <= bed_z
            if not ok:
                fails.append(p.base)
            rows.append(f"{p.base}({p.orientation}) {xy[0]:.1f}x{xy[1]:.1f}x{h:.1f}")
        return Check("C1", "printed parts fit the bed in print orientation",
                     FAIL if fails else PASS, f"worst extent/limit = {worst:.3f} (limits {bed_xy:.0f}x{bed_xy:.0f}x{bed_z:.0f})",
                     "; ".join(rows) + (f"; FAILS: {fails}" if fails else ""), {"worst_ratio": worst, "fails": fails})

    def c2_envelope(self) -> Check:
        env = rg.payload_envelope(self.L)
        vols = {n: _v(env, p.shape) for n, p in self.parts.items() if p.kind == "printed" and p.in_vehicle}
        worst = max(vols, key=vols.get)
        return Check("C2", "payload envelope does not intersect any printed body", _status_no_intersection(vols[worst]),
                     f"max common volume {vols[worst]:.3f} mm^3 ({worst})",
                     f"envelope D{self.L.envelope_d:.0f} Z {self.L.z_envelope_bottom:.0f}..{self.L.z_envelope_top:.0f}", vols)

    def c3_edf(self) -> Check:
        L = self.L
        edf = self.shape["edf_unit"]
        v1, v2 = _v(edf, self.prop), _v(edf, self.shape["intake_module"])
        lip_ok = L.shelf_id < L.p["edf"]["lip_od"]
        st = _status_no_intersection(max(v1, v2))
        if not lip_ok:
            st = FAIL
        return Check("C3", "EDF clears propulsion/intake modules; shelf retains the lip", st,
                     f"edf x prop {v1:.3f}, edf x intake {v2:.3f} mm^3; shelf_id {L.shelf_id:.1f} < lip_od {L.p['edf']['lip_od']:.1f} -> {lip_ok}",
                     "", {"edf_prop": v1, "edf_intake": v2, "shelf_id": L.shelf_id, "lip_od": L.p["edf"]["lip_od"]})

    def c4_vanes(self) -> Check:
        L = self.L
        dmax = L.p["vane"]["max_deflection_mech_deg"]
        neutral = [self.shape[f"vane_{i + 1}"] for i in range(L.vane_count)]
        worst_prop, worst_vane, worst_shaft = (0.0, ""), (0.0, ""), (0.0, "")
        for i in range(L.vane_count):
            for d in (0.0, +dmax, -dmax):
                vs = neutral[i] if d == 0.0 else rg.vane(L, i, d)
                v = _v(vs, self.prop)
                if v > worst_prop[0]:
                    worst_prop = (v, f"vane_{i + 1} d={d:+.0f}")
                for j in range(L.vane_count):
                    if j == i:
                        continue
                    vv = _v(vs, neutral[j])
                    if vv > worst_vane[0]:
                        worst_vane = (vv, f"vane_{i + 1} d={d:+.0f} vs vane_{j + 1}")
            sh = _v(rg.vane_shaft(L, i), self.prop)
            if sh > worst_shaft[0]:
                worst_shaft = (sh, f"shaft_{i + 1}")
        vmax = max(worst_prop[0], worst_vane[0], worst_shaft[0])
        return Check("C4", f"vanes at d=0/+-{dmax:.0f}deg clear the module and each other; shafts pass the bore",
                     _status_no_intersection(vmax),
                     f"max vane x module {worst_prop[0]:.3f} ({worst_prop[1]}), vane x vane {worst_vane[0]:.3f} ({worst_vane[1]}), shaft x module {worst_shaft[0]:.3f} mm^3",
                     f"vane_r_out {L.vane_r_out:.1f} vs duct_r {L.duct_r:.1f}; TE offset {L.vane_hinge_z - L.vane_te_z:.1f} mm",
                     {"vane_module": worst_prop[0], "vane_vane": worst_vane[0], "shaft_module": worst_shaft[0]})

    def _c5_direct(self) -> Check:
        """v0.9 (vane_drive.mode 'direct'): each servo in its fin-root fairing, on the solids - (a) the servo (case + ears +
        output boss), the horn and the coupler clear the propulsion module and the cover (< INTERSECT_TOL) and the servo
        keeps >= servo_float - 0.01 from the module (it floats); (b) the lower ear tab is captured in its floor slot:
        material on both radial sides of the tab over the slot height and under it (>= C5_FILL_MIN); (c) slide-out: the
        servo + horn + coupler envelope swept along the service direction past the fairing face meets no module material;
        (d) the cover is seated on the service face (cover x module < tol, BRep gap < 0.01) and covers the pocket opening;
        (e) each cover screw: the clearance hole in the cover clear, the tap hole clear, tap material around it >=
        C5_FILL_MIN, and a screwdriver (cov_driver_d x cov_driver_len from the cover face outward) clear of every other
        component; (f) the connector bay holds the inline connector clear of the module and the servo."""
        L = self.L
        n, ss, f = L.vane_count, L.dd_service_side, L.dd_float
        prop = self.prop
        worst_x, worst_gap, slot_fill, sweep_x, cov = (0.0, ""), (math.inf, ""), (math.inf, ""), (0.0, ""), []
        screw_rows, driver_x, conn_x = [], (0.0, ""), (0.0, "")
        removal: list[dict] = []
        others = {nm: s for nm, s in self.shape.items() if nm != "propulsion_module" and self.parts[nm].in_vehicle}
        for k in range(n):
            th = L.fin_thetas[k]
            servo, horn, coupler, cover = (self.shape[f"servo_vane_{k + 1}"], self.shape[f"servo_horn_{k + 1}"],
                                           self.shape[f"vane_coupler_{k + 1}"], self.shape[f"servo_cover_{k + 1}"])
            for nm, s in ((f"servo_vane_{k + 1}", servo), (f"servo_horn_{k + 1}", horn), (f"vane_coupler_{k + 1}", coupler)):
                for tgt_nm, tgt in (("propulsion_module", prop), (f"servo_cover_{k + 1}", cover)):
                    v = _v(s, tgt)
                    if v > worst_x[0]:
                        worst_x = (v, f"{nm} x {tgt_nm}")
            g = _clearance(servo, prop)
            if g < worst_gap[0]:
                worst_gap = (g, f"servo_vane_{k + 1}")
            # (b) ear tab captured: inner / outer radial walls of the slot and its floor
            hw = L.dd_case_w / 2
            probes = {"inner": rg._box_polar(L.dd_slot_u[0] - C5_WALL_PROBE, L.dd_slot_u[0] - C5_PROBE_INSET, -hw, hw,
                                             L.dd_slot_z[0] + C5_PROBE_INSET, L.dd_slot_z[1] - C5_PROBE_INSET, th),
                      "outer": rg._box_polar(L.dd_slot_u[1] + C5_PROBE_INSET, L.dd_slot_u[1] + C5_WALL_PROBE, -hw, hw,
                                             L.dd_slot_z[0] + C5_PROBE_INSET, L.dd_slot_z[1] - C5_PROBE_INSET, th),
                      "floor": rg._box_polar(L.dd_slot_u[0], L.dd_slot_u[1], -hw, hw, L.dd_slot_z[0] - C5_WALL_PROBE,
                                             L.dd_slot_z[0] - C5_PROBE_INSET, th)}
            for pn, pb in probes.items():
                fr = _vpos(pb, prop) / pb.Volume()
                if fr < slot_fill[0]:
                    slot_fill = (fr, f"fin {k + 1} slot {pn}")
            # (c) slide-out along the service direction: the servo / horn / coupler boxes extended past the fairing face
            far = ss * (L.fair_hv + L.dd_case_w + C5_SWEEP_PAST)
            def swept(u0, u1, v0, v1, z0, z1):
                a, b = (min(v0, far), max(v1, far)) if ss > 0 else (min(v0, far), max(v1, far))
                return rg._box_polar(u0, u1, a, b, z0, z1, th)
            rr = max(L.dd_flange_d, L.dd_horn_d) / 2
            env = [swept(L.dd_u_case_top, L.dd_u_case_bot, -hw, hw, L.dd_z_case[0], L.dd_z_case[1]),
                   swept(L.dd_u_ear[0], L.dd_u_ear[1], -hw, hw, L.dd_z_ear[0], L.dd_z_ear[1]),
                   swept(L.dd_u_hub0, L.dd_u_case_top, -rr, rr, L.vane_hinge_z - rr, L.vane_hinge_z + rr),
                   # the servo's own lead and its connector half come out with it (under the lintel)
                   swept(L.dd_conn_u[0], L.dd_conn_u[1], -L.dd_conn_v, L.dd_conn_v, L.dd_conn_z[0], L.dd_conn_z[1])]
            for e in env:
                v = _v(e, prop)
                if v > sweep_x[0]:
                    sweep_x = (v, f"fin {k + 1}")
            # v0.9 fix round (reviewer: 'the removal procedure is blocked by the vane shaft'): the same sweep against the
            # vane installed (blocked - documented) and pushed inward by the shaft engagement + 1 mm (must be clear); the
            # pushed vane must clear the module, its bushing and the other vanes; the set screw's hex-key path (key_d x
            # key_len along +v from the hub with the cover off) must be clear
            if hasattr(L, "dd_set_screw_u"):
                push = L.dd_engagement + 1.0
                t = math.radians(L.vane_thetas[k])
                vane_in = self.shape[f"vane_{k + 1}"]
                vane_pushed = vane_in.translate(cq.Vector(-push * math.cos(t), -push * math.sin(t), 0.0))
                envu = rg._fuse(env)
                rm = {"installed": _v(envu, vane_in), "pushed": _v(envu, vane_pushed)}
                others_v = [prop] + [self.shape[f"vane_{j + 1}"] for j in range(n) if j != k]
                if f"hinge_bushing_{k + 1}" in self.shape:
                    others_v.append(self.shape[f"hinge_bushing_{k + 1}"])
                rm["pushed_vane_x"] = max(_v(vane_pushed, s) for s in others_v)
                kd, kl = L.dd_set_screw_key
                a = L.fin_xyz(k, L.dd_set_screw_u, ss * (L.dd_hub_d / 2 + 0.1), L.vane_hinge_z)
                b = L.fin_xyz(k, L.dd_set_screw_u, ss * (L.fair_hv + kl), L.vane_hinge_z)
                key = rg._cyl_between(kd, a, b)
                kx = max((_v(key, s), nm) for nm, s in list(others.items()) + [("propulsion_module", prop)]
                         if nm != f"servo_cover_{k + 1}")
                rm["key"], rm["key_hit"] = kx
                removal.append({"fin": k + 1, **rm})
            # (d) cover seated
            cov.append({"fin": k + 1, "x_module": _v(cover, prop), "gap": _clearance(cover, prop)})
            # (e) screws
            for j, (a, b) in enumerate(rg.cover_screw_axes(L, k)):
                va, vb = cq.Vector(*a), cq.Vector(*b)
                ax = (vb - va).normalized()
                tap_len = (vb - va).Length
                tap_clear = rg._cyl_between(L.cov_d_tap - 2 * C5_PROBE_INSET, a, (va + ax * (tap_len - C5_PROBE_INSET)).toTuple())
                ring = rg._cyl_between(L.cov_d_tap + 2 * C5_TAP_RING, (va + ax * C5_PROBE_INSET).toTuple(), b).cut(
                    rg._cyl_between(L.cov_d_tap + 2 * C5_PROBE_INSET, a, b, ext0=1.0, ext1=1.0))
                hole_clear = rg._cyl_between(L.cov_d_clear - 2 * C5_PROBE_INSET, (va - ax * (L.cov_t + 0.5)).toTuple(), a)
                driver = rg._cyl_between(L.cov_driver_d, (va - ax * (L.cov_t + 0.2)).toTuple(),
                                         (va - ax * (L.cov_t + 0.2 + L.cov_driver_len)).toTuple())
                vt = _v(tap_clear, prop)
                vr = _vpos(ring, prop) / ring.Volume()
                vh = _v(hole_clear, cover)
                dx = max((_v(driver, s), nm) for nm, s in list(others.items()) + [("propulsion_module", prop)])
                screw_rows.append({"fin": k + 1, "screw": j + 1, "tap_clear": vt, "tap_material": vr, "hole_clear": vh,
                                   "driver": dx[0], "driver_hit": dx[1]})
                if dx[0] > driver_x[0]:
                    driver_x = (dx[0], f"fin {k + 1} screw {j + 1} x {dx[1]}")
            # (f) connector bay
            conn = rg.servo_connector(L, k)
            v = max(_v(conn, prop), _v(conn, servo))
            if v > conn_x[0]:
                conn_x = (v, f"fin {k + 1}")
        ok_x = worst_x[0] < INTERSECT_TOL_MM3
        ok_gap = worst_gap[0] >= f - 0.01
        ok_slot = slot_fill[0] >= C5_FILL_MIN
        ok_sweep = sweep_x[0] < INTERSECT_TOL_MM3
        ok_cov = all(c["x_module"] < INTERSECT_TOL_MM3 and c["gap"] < 0.01 for c in cov)
        ok_screw = all(r["tap_clear"] < C5_CLEAR_TOL_MM3 and r["tap_material"] >= C5_FILL_MIN and r["hole_clear"] < C5_CLEAR_TOL_MM3
                       and r["driver"] < INTERSECT_TOL_MM3 for r in screw_rows)
        ok_conn = conn_x[0] < INTERSECT_TOL_MM3
        # v0.9 fix round: removal order (vane pushed in first) and the set-screw key path
        ok_rm = all(r["pushed"] < INTERSECT_TOL_MM3 and r["pushed_vane_x"] < INTERSECT_TOL_MM3 and r["key"] < INTERSECT_TOL_MM3
                    for r in removal)
        allok = ok_x and ok_gap and ok_slot and ok_sweep and ok_cov and ok_screw and ok_conn and ok_rm
        rm_txt = ""
        if removal:
            rm_txt = (f"; removal: sweep x vane installed {max(r['installed'] for r in removal):.1f} mm^3 (blocked: push the "
                      f"vane in first), x vane pushed {L.dd_engagement + 1.0:.1f} mm in {max(r['pushed'] for r in removal):.3f} "
                      f"mm^3, pushed vane x module / bushing / other vanes {max(r['pushed_vane_x'] for r in removal):.3f} mm^3; "
                      f"set-screw key O{L.dd_set_screw_key[0]:g} x {L.dd_set_screw_key[1]:g} x anything (cover off) "
                      f"{max(r['key'] for r in removal):.3f} mm^3")
        return Check("C5", "(v0.9 direct drive) servo + ears + horn + coupler in the fin-root fairing: clear, floating, ear tab "
                     "captured, slides out to the service side (after the vane is pushed in), cover seated, cover screws (tap, "
                     "material, screwdriver path), set-screw key path, connector bay", PASS if allok else FAIL,
                     f"max x (module / cover) {worst_x[0]:.3f} mm^3 ({worst_x[1]}); servo -> module min gap {worst_gap[0]:.3f} "
                     f"mm (float {f:g}); ear-slot walls / floor min fill {slot_fill[0]:.3f} ({slot_fill[1]}); slide-out sweep "
                     f"x module {sweep_x[0]:.3f} mm^3; cover x module max {max(c['x_module'] for c in cov):.3f}, gap max "
                     f"{max(c['gap'] for c in cov):.3f} mm; screws: tap clear max {max(r['tap_clear'] for r in screw_rows):.4f}, "
                     f"tap material min {min(r['tap_material'] for r in screw_rows):.3f}, cover hole clear max "
                     f"{max(r['hole_clear'] for r in screw_rows):.4f}, screwdriver x anything max {driver_x[0]:.3f} mm^3 "
                     f"({driver_x[1] or '-'}); connector x (module, servo) {conn_x[0]:.3f} mm^3" + rm_txt,
                     f"pocket u {L.dd_pocket_u[0]:.2f}..{L.dd_pocket_u[1]:.2f}, |v| {L.dd_pocket_hv:.2f} (+ open to the service side "
                     f"{'+' if ss > 0 else '-'}v), z {L.dd_pocket_z[0]:.2f}..{L.dd_pocket_z[1]:.2f}; ear slot u {L.dd_slot_u[0]:.2f}.."
                     f"{L.dd_slot_u[1]:.2f}, z {L.dd_slot_z[0]:.2f}..{L.dd_slot_z[1]:.2f}; cover screws (u, z) "
                     + ", ".join(f"({u:.2f}, {z:.2f})" for u, z in L.cov_screws_uz)
                     + f", tap O{L.cov_d_tap:g} x {L.cov_tap_depth:g}, clear O{L.cov_d_clear:g}; screwdriver O{L.cov_driver_d:g} x "
                       f"{L.cov_driver_len:g}",
                     {"intersection": worst_x[0], "servo_gap": worst_gap[0], "slot_fill": slot_fill[0], "sweep": sweep_x[0],
                      "covers": cov, "screws": screw_rows, "connector": conn_x[0], "removal": removal})

    def c28_coupler(self) -> Check:
        """v0.9 (direct drive): coupler alignment and torque - (a) the servo output axis (the horn disc's COM on the
        solid) lies on the vane hinge axis (offset < C28_AXIS_TOL); (b) servo_float >= print_tolerance_ASSUMED (the
        servo floats, so a pocket printed off the axis does not side-load the coupler); (c) the coupler and horn rotated
        with the vane at 0 / +-max_deflection_mech_deg clear the propulsion module and the servo case by >= servo_float -
        0.01 (BRep), and the coupler clears the vane shaft at every deflection (its D-bore clearance); (d) the shaft
        engages the hub over >= half the hub and ends short of the flange; (e) the worst hinge moment (Layout,
        ASSUMED CP offset and bearing friction) is below the servo torque by >= C28_TORQUE_MARGIN_MIN."""
        L = self.L
        if not L.direct_drive:
            return Check("C28", "(v0.9 direct drive) coupler alignment and hinge moment", NA,
                         "not applicable: vane_drive.mode 'pushrod'", "", {"applicable": False})
        dmax = L.p["vane"]["max_deflection_mech_deg"]
        axis_off, gap_min, shaft_x = 0.0, (math.inf, ""), (0.0, "")
        for k in range(L.vane_count):
            c = self.shape[f"servo_horn_{k + 1}"].Center()
            t = math.radians(L.vane_thetas[k])
            # distance of the horn's centre from the hinge axis line (radial at theta_k, Z hinge)
            along = c.x * math.cos(t) + c.y * math.sin(t)
            perp = -c.x * math.sin(t) + c.y * math.cos(t)
            axis_off = max(axis_off, math.hypot(perp, c.z - L.vane_hinge_z))
            for d in (0.0, +dmax, -dmax):
                cp = rg.vane_coupler(L, k, d)
                g = _clearance(cp, self.prop)
                if g < gap_min[0]:
                    gap_min = (g, f"coupler_{k + 1} d={d:+.0f} x propulsion_module")
                v = _v(cp, rg.vane(L, k, d))
                if v > shaft_x[0]:
                    shaft_x = (v, f"coupler_{k + 1} x vane_{k + 1} d={d:+.0f}")
        ok_axis = axis_off < C28_AXIS_TOL
        ok_float = L.dd_float >= L.dd_tol - 1e-9
        ok_gap = gap_min[0] >= L.dd_float - 0.01
        ok_shaft = shaft_x[0] < INTERSECT_TOL_MM3
        hub = L.dd_u_hub1 - L.dd_u_hub0
        ok_eng = L.dd_engagement >= 0.5 * hub and L.vane_shaft_r_out < L.dd_u_hub1
        ok_torque = L.vane_hinge_margin >= C28_TORQUE_MARGIN_MIN
        allok = ok_axis and ok_float and ok_gap and ok_shaft and ok_eng and ok_torque
        # v0.9 fix round: (f) axial retention by the set screw - the vane + coupler move as one: the inward stop (hub on
        # the boss face, boss_gap) < half the engagement; the outward free travel (vane root to the duct wall, measured
        # by translating vane_1 + coupler_1 radially in 0.05 mm steps) leaves the horn (arm face, screw head) untouched;
        # (g) foam pad compression range; (h) bushing (bore = its OD, shaft tilt); (i) set-screw thread length; (j)
        # socket: arm engagement over the slot at both stops, estimated lost motion
        extra = {}
        fix_txt = ""
        if hasattr(L, "dd_set_screw_u"):
            k = 0
            t = math.radians(L.vane_thetas[k])
            e = cq.Vector(math.cos(t), math.sin(t), 0.0)
            vane0, cp0 = self.shape[f"vane_{k + 1}"], self.shape[f"vane_coupler_{k + 1}"]
            horn0 = self.shape[f"servo_horn_{k + 1}"]
            stops = [self.prop] + ([self.shape[f"hinge_bushing_{k + 1}"]] if f"hinge_bushing_{k + 1}" in self.shape else [])
            def hits(d: float) -> float:
                return max(_v(vane0.translate(e * d), s) for s in stops)
            lo, hi = 0.0, 6.0          # bisection to 0.02 mm (hits(0) = 0: the vane clears the module as built)
            if hits(hi) < INTERSECT_TOL_MM3:
                lo = hi
            while hi - lo > 0.02:
                mid = 0.5 * (lo + hi)
                lo, hi = (mid, hi) if hits(mid) < INTERSECT_TOL_MM3 else (lo, mid)
            t_out = lo
            horn_at_out = max(_v(cp0.translate(e * t_out), horn0), _v(vane0.translate(e * t_out), horn0))
            inward_ok = (max(_v(cp0.translate(e * -(L.dd_inward_stop - 0.05)), s) for s in stops) < INTERSECT_TOL_MM3
                         and max(_v(cp0.translate(e * -(L.dd_inward_stop + 0.2)), s) for s in stops) > INTERSECT_TOL_MM3)
            arm_eng_out = min(L.dd_u_flange1 + t_out, L.dd_u_horn1) - max(L.dd_u_hub1 + t_out, L.dd_u_arm0)
            arm_eng_in = min(L.dd_u_flange1 - L.dd_inward_stop, L.dd_u_horn1) - max(L.dd_u_hub1 - L.dd_inward_stop, L.dd_u_arm0)
            ok_ret = (L.dd_inward_stop < 0.5 * L.dd_engagement and inward_ok and horn_at_out < INTERSECT_TOL_MM3
                      and min(arm_eng_out, arm_eng_in) >= L.dd_horn_arm_t - 1e-6)
            ok_foam = (not getattr(L, "dd_foam", False)) or (
                L.dd_foam_limits[0] - 1e-9 <= L.dd_foam_compression[0] and L.dd_foam_compression[1] <= L.dd_foam_limits[1] + 1e-9)
            ok_bush = (not getattr(L, "dd_bushing", False)) or abs(L.hinge_bore_d - L.dd_bush_od) < 1e-9
            ok_ss = L.dd_set_screw_wall >= 2 * L.p["printer"]["min_feature"] - 1e-9
            allok = allok and ok_ret and ok_foam and ok_bush and ok_ss
            extra = {"inward_stop_mm": L.dd_inward_stop, "inward_stop_found": inward_ok, "outward_free_mm": t_out,
                     "horn_x_at_outward_stop": horn_at_out, "arm_engagement_mm": [arm_eng_in, arm_eng_out],
                     "foam_compression": list(getattr(L, "dd_foam_compression", ())),
                     "bushing_tilt_deg": getattr(L, "dd_bush_tilt_deg", None), "set_screw_thread_mm": L.dd_set_screw_wall,
                     "deadband_pp_deg": {"socket": L.dd_deadband_slot_deg, "gear_backlash_ASSUMED": L.dd_gear_backlash_deg,
                                         "total": L.dd_deadband_pp_deg}}
            fix_txt = (f"; retention (set screw): inward stop {L.dd_inward_stop:g} mm (< {0.5 * L.dd_engagement:.2f} = half the "
                       f"engagement; stop found {inward_ok}), outward free {t_out:.2f} mm (vane root -> duct wall), coupler / shaft "
                       f"x horn at the outward stop {horn_at_out:.3f} mm^3, horn arms engaged {min(arm_eng_in, arm_eng_out):.2f} of "
                       f"{L.dd_horn_arm_t:g} mm at both stops; foam {L.dd_foam_t:g} mm compressed "
                       f"{L.dd_foam_compression[0]:.2f}..{L.dd_foam_compression[1]:.2f} (limits {L.dd_foam_limits[0]:g}.."
                       f"{L.dd_foam_limits[1]:g}); bushing O{L.dd_bush_od:g}/{L.dd_bush_id:g} in a O{L.hinge_bore_d:g} bore, tilt "
                       f"{L.dd_bush_tilt_deg:.2f} deg; set-screw thread {L.dd_set_screw_wall:.2f} mm; lost motion est. "
                       f"{L.dd_deadband_slot_deg:.2f} deg (socket) + {L.dd_gear_backlash_deg:g} deg gear (ASSUMED) p-p")
        return Check("C28", "(v0.9 direct drive) coupler alignment and hinge moment: servo axis on the hinge axis, servo floats "
                     ">= print tolerance, coupler clears the fairing at 0/+-max deflection, D-bore clears the shaft, shaft "
                     "engagement, hinge moment << servo torque; (fix round) set-screw axial retention, horn socket, foam pad, "
                     "hinge bushing", PASS if allok else FAIL,
                     f"axis offset {axis_off:.4f} mm (< {C28_AXIS_TOL:g}); float {L.dd_float:g} >= tolerance {L.dd_tol:g}; coupler -> "
                     f"module min gap {gap_min[0]:.3f} mm ({gap_min[1]}); coupler x shaft {shaft_x[0]:.3f} mm^3; engagement "
                     f"{L.dd_engagement:.2f} of {hub:.1f} mm; hinge moment {L.vane_hinge_moment_kgcm:.3f} kgf cm vs servo "
                     f"{L.servo_torque_kgcm:g} kgf cm (x{L.vane_hinge_margin:.1f}, >= {C28_TORQUE_MARGIN_MIN:g})" + fix_txt,
                     f"worst vane side load {L.vane_side_load_N:.2f} N (thrust_peak {L.p['edf']['thrust_peak_N']:.1f} N, exhaust "
                     f"q {L.vane_exit_q_pa:.0f} Pa, {L.vane_exit_v_mps:.1f} m/s, d_cmd {L.p['vane']['max_deflection_cmd_deg']:g} deg); "
                     f"CP offset {L.p['vane_drive']['hinge_moment']['cp_offset_chord_max_ASSUMED']:g} chord and bearing mu "
                     f"{L.p['vane_drive']['hinge_moment']['bearing_mu_ASSUMED']:g} ASSUMED",
                     {"axis_offset": axis_off, "coupler_gap": gap_min[0], "coupler_shaft": shaft_x[0],
                      "engagement": L.dd_engagement, "hinge_moment_kgcm": L.vane_hinge_moment_kgcm,
                      "servo_torque_kgcm": L.servo_torque_kgcm, "margin": L.vane_hinge_margin, **extra})

    def c5_linkage(self) -> Check:
        L = self.L
        if L.direct_drive:
            return self._c5_direct()
        n = L.vane_count
        group = {}
        for i in range(n):
            group[f"vane_horn_{i + 1}"] = self.shape[f"vane_horn_{i + 1}"]
            group[f"servo_vane_{i + 1}"] = self.shape[f"servo_vane_{i + 1}"]
            group[f"servo_arm_{i + 1}"] = self.shape[f"servo_arm_{i + 1}"]
            group[f"pushrod_{i + 1}"] = self.shape[f"pushrod_{i + 1}"]
        connected = set()
        for i in range(n):
            connected.add(frozenset({f"pushrod_{i + 1}", f"servo_arm_{i + 1}"}))
            connected.add(frozenset({f"pushrod_{i + 1}", f"vane_horn_{i + 1}"}))
        worst_mod = (0.0, "")
        for name, s in group.items():
            v = _v(s, self.prop)
            if v > worst_mod[0]:
                worst_mod = (v, name)
        worst_pair, skipped = (0.0, ""), []
        names = list(group)
        for a in range(len(names)):
            for b in range(a + 1, len(names)):
                pair = frozenset({names[a], names[b]})
                if pair in connected:
                    skipped.append(f"{names[a]}-{names[b]}")
                    continue
                v = _v(group[names[a]], group[names[b]])
                if v > worst_pair[0]:
                    worst_pair = (v, f"{names[a]} x {names[b]}")
        worst_pod = (0.0, "")
        for i in range(n):
            v = _v(self.shape[f"pushrod_{i + 1}"], rg.servo_pod(L, i))
            if v > worst_pod[0]:
                worst_pod = (v, f"pushrod_{i + 1} x pod_{i + 1}")
        vmax = max(worst_mod[0], worst_pair[0], worst_pod[0])
        return Check("C5", "horns, servos, arms, pushrods clear the module (incl. fins) and each other",
                     _status_no_intersection(vmax),
                     f"max  x module {worst_mod[0]:.3f} ({worst_mod[1]}), pairwise {worst_pair[0]:.3f} ({worst_pair[1]}), pushrod x pod {worst_pod[0]:.3f} mm^3",
                     f"connected pairs excluded from the pairwise test (joined by a clevis pin): {len(skipped)}",
                     {"module": worst_mod[0], "pairwise": worst_pair[0], "pushrod_pod": worst_pod[0]})

    def c6_modules(self) -> Check:
        L = self.L
        vtol = getattr(L, "vtol", False)
        pairs = [("propulsion_module", "intake_module"), ("intake_module", "avionics_module")] + (
            [("avionics_module", "top_cap")] if vtol else       # v0.9 CAD-2 VTOL: the top cap closes the stack
            [("avionics_module", "payload_module"), ("payload_module", "nose_cone")])
        vols = {f"{a} x {b}": _v(self.shape[a], self.shape[b]) for a, b in pairs}
        worst = max(vols, key=vols.get)
        return Check("C6", "adjacent modules do not intersect (spigot/skirt laps, nose skirt vs payload)",
                     _status_no_intersection(vols[worst]), f"max common volume {vols[worst]:.3f} mm^3 ({worst})",
                     f"spigot OD {L.spigot_od:.1f} vs tube ID {L.body_id:.1f}; " + ("top cap skirt over the avionics spigot" if vtol
                     else f"nose skirt ID {L.nose_skirt_id:.1f}"), vols)

    def c7_avionics(self) -> Check:
        L = self.L
        tube, tray = self.shape["avionics_module"], self.shape["avionics_tray"]
        vols = {}
        for n in [n for n in ("battery", "buffer_pack", "esc", "nav", "misc_electronics", "gns") if n in self.shape]:
            vols[f"{n} x tube"] = _v(self.shape[n], tube)
            vols[f"{n} x tray"] = _v(self.shape[n], tray)
        vols["tray x tube"] = _v(tray, tube)
        bb = tray.BoundingBox()
        lo, hi = L.tray_x - L.rail_gap / 2, L.tray_x + L.rail_gap / 2
        in_slot = bb.xmin >= lo - 1e-6 and bb.xmax <= hi + 1e-6
        worst = max(vols, key=vols.get)
        st = _status_no_intersection(vols[worst])
        if not in_slot:
            st = FAIL
        # v0.5d tray stop (SPEC 5.2, both presets), positive evidence: in each rail slot, right under the tray
        # footprint (x = the tray thickness, y from the rails' inner face to the tray edge), a FLOOR_AREA_SLAB-thin slab
        # is avionics_module material, and that material's top is the tray's bottom face (the tray rests on it)
        stop: dict[str, dict] = {}
        stop_txt = "no tray stop in params (tray_stop_h) - SPEC 5.2 requires one: FAIL"
        if not L.tray_stop_h:
            st = FAIL
        else:
            x0, x1 = L.tray_x - L.tray_t / 2, L.tray_x + L.tray_t / 2
            y_in, y_edge = L.rail_y - L.rail_depth, L.tray_w / 2
            for sy, nm in ((+1.0, "+y"), (-1.0, "-y")):
                if y_edge <= y_in:          # the tray does not reach into the slot: nothing to rest on
                    stop[nm] = {"filled": 0.0, "top_z": -math.inf}
                    continue
                ya, yb = (y_in, y_edge) if sy > 0 else (-y_edge, -y_in)
                slab = _box(x0, x1, ya, yb, L.z_tray_bottom - FLOOR_AREA_SLAB, L.z_tray_bottom)
                around = _box(x0, x1, ya, yb, L.z_tray_bottom - FLOOR_PROBE_H, L.z_tray_bottom + FLOOR_PROBE_H)
                stop[nm] = {"filled": _vpos(slab, tube) / slab.Volume(), "top_z": _top_z(tube, around)}
            stop_ok = all(s["filled"] >= TRAY_STOP_FILL_MIN and abs(s["top_z"] - L.z_tray_bottom) < TRAY_STOP_TOP_TOL
                          for s in stop.values())
            if not stop_ok:
                st = FAIL
            stop_txt = ("tray stop under the tray footprint: slab filled " +
                        ", ".join(f"{k} {s['filled']:.3f}" for k, s in stop.items()) +
                        f" (>= {TRAY_STOP_FILL_MIN:g}), top z " + ", ".join(f"{k} {s['top_z']:.3f}" for k, s in stop.items()) +
                        f" vs z_tray_bottom {L.z_tray_bottom:.3f} -> {stop_ok}")
        return Check("C7", "electronics clear tube and tray; tray clears tube and sits in the rail slot (v0.5d: on the tray stops)", st,
                     f"max common volume {vols[worst]:.3f} mm^3 ({worst}); tray x [{bb.xmin:.2f},{bb.xmax:.2f}] in slot [{lo:.2f},{hi:.2f}] -> {in_slot}; "
                     + stop_txt,
                     f"tray stop (SPEC 5.2): block x tray_x +- rail_gap/2, Z {L.z_tray_stop_bottom if L.tray_stop_h else float('nan'):.2f}.."
                     f"{L.z_tray_bottom:.2f}; probes x {L.tray_x - L.tray_t / 2:.2f}..{L.tray_x + L.tray_t / 2:.2f}, |y| "
                     f"{L.rail_y - L.rail_depth:.3f}..{L.tray_w / 2:.2f}, slab {FLOOR_AREA_SLAB:g} mm under the tray",
                     {**vols, "tray_x": [bb.xmin, bb.xmax], "slot": [lo, hi], "tray_stop": stop})

    def c8_exhaust(self) -> Check:
        """SPEC C8: the path cylinder intersects nothing but vanes/shafts and (tabs) the edf_unit motor/hub;
        path x propulsion_module = 0 and, measured, path x every other component = 0."""
        L = self.L
        path = rg.exhaust_path(L)
        pb = path.BoundingBox()
        v = _v(path, self.prop)
        vane_v = sum(_v(path, self.shape[f"vane_{i + 1}"]) for i in range(L.vane_count))
        allowed = {f"vane_{i + 1}" for i in range(L.vane_count)} | {"propulsion_module"}
        if L.tabs:
            allowed.add("edf_unit")
        others = {}
        for n, p in self.parts.items():          # every other component whose bbox reaches the path
            if n in allowed or p.shape is None or not p.in_vehicle:
                continue
            bb = p.shape.BoundingBox()
            if bb.zmax < pb.zmin or bb.zmin > pb.zmax or bb.xmax < pb.xmin or bb.xmin > pb.xmax \
                    or bb.ymax < pb.ymin or bb.ymin > pb.ymax:
                continue
            others[n] = _v(path, p.shape)
        worst_other = max(others.items(), key=lambda kv: kv[1]) if others else ("", 0.0)
        detail = (f"D{L.duct_id - 2:.0f} Z 0..{L.z_edf_housing_bottom:.1f}; other components tested (bbox overlap): "
                  f"{', '.join(f'{n} {x:.3f}' for n, x in others.items()) or 'none'}")
        values = {"module": v, "vanes": vane_v, "others": others}
        if L.tabs:   # v0.5: the probe passes the liner bore (duct_r) up to z_liner_top; the motor sits in the jet
            motor_v = _v(path, self.shape["edf_unit"])
            detail += (f"; liner bore r {L.duct_r:.2f} Z {L.z_edf_shoulder:.1f}..{L.z_liner_top:.1f}; "
                       f"EDF motor/hub inside the path {motor_v:.0f} mm^3 (the EDF's own tail, allowed)")
            values["edf_unit"] = motor_v
        lead_txt = ""
        if getattr(L, "clock_mount", False):
            # wire09c clock mount: the factory bundle / leads of the WORST drag corner (grown) inside the path - allowed,
            # quantified (as the EDF motor / hub); the frontal blockage of that corner; the duct-wall window the passage
            # opens under the rim (E)
            wc = L.ml_blockage["worst_corner"]
            cw = {"S": wc["S"], "sec": wc["sec"], "h": L.gb_sections[wc["sec"]][0], "w": L.gb_sections[wc["sec"]][1],
                  "od": wc["od"], "R": min(L.gb_R), "gap": min(L.gb_gap), "inset": max(L.gb_inset), "early": 0.0}
            lv = {"worst-drag corner envelope (grown)": ckc._v_env(rg.clock_lead_env(L, cw, 0.0, True), path)}
            bl_ = L.ml_blockage
            values["motor_lead_legs"] = lv
            values["motor_lead_blockage"] = {k: v for k, v in bl_.items()}
            values["duct_window_mm2_E"] = L.lp_duct_window_mm2
            lead_txt = (f", motor-lead envelope inside the path {sum(lv.values()):.0f} mm^3 (allowed, quantified: worst corner "
                        f"{bl_['projected_mm2']:.1f} mm^2 = {bl_['projected_pct']:.1f} % of the jet annulus, E; duct-wall "
                        f"window under the rim {L.lp_duct_window_mm2:.0f} mm^2, E)")
            detail += (f"; motor leads (clock mount, worst drag corner {wc['sec']} S{wc['S']:g} od{wc['od']:g} "
                       f"{wc['singles']}): bundle {bl_['bundle_len_in_jet_mm']:.1f} mm + single leads "
                       f"{bl_['single_leads_len_in_jet_mm']:.1f} mm in the jet (drag in C33)")
        elif getattr(L, "ml_lower_uz", None):
            # fix round (review 2026-10-29 G6): the motor-lead bundle / leads behind the stator vane lie in the exhaust
            # path - an allowed, quantified obstruction (as the EDF motor / hub): their envelope (grown by wire_clearance)
            # inside the path and Layout's frontal blockage of the jet annulus; the drag goes to C33 / vtol_sizing
            legs = rg.motor_lead_lower_legs(L)
            lv = {k: _v(path, s_) for k, s_ in legs.items()}
            bl_ = L.ml_blockage
            values["motor_lead_legs"] = lv
            values["motor_lead_blockage"] = dict(bl_)
            lead_txt = (f", motor-lead envelope inside the path {sum(lv.values()):.0f} mm^3 (allowed, quantified: "
                        f"{bl_['projected_mm2']:.0f} mm^2 = {bl_['projected_pct']:.1f} % of the jet annulus)")
            detail += (f"; motor leads (photo round): envelope x path " + ", ".join(f"{k} {x:.0f}" for k, x in lv.items())
                       + f" mm^3, bundle {bl_['bundle_len_in_jet_mm']:.1f} mm + single leads "
                       f"{bl_['single_leads_len_in_jet_mm']:.1f} mm in the jet (drag in C33)")
        return Check("C8", "exhaust path D(duct_id-2), exit -> EDF housing bottom, meets nothing but vanes/shafts "
                     "(and, tabs, the EDF motor/hub; v0.9 the motor-lead bundle, quantified); x propulsion_module = 0",
                     _status_no_intersection(max(v, worst_other[1])),
                     f"path x module {v:.3f} mm^3, x other components max {worst_other[1]:.3f} ({worst_other[0] or '-'}); "
                     f"vanes inside the path {vane_v:.0f} mm^3" + (", EDF motor/hub allowed" if L.tabs else "") + lead_txt
                     + " (allowed)",
                     detail, values)

    def c9_inlet(self) -> Check:
        """SPEC C9 (v0.6.1): inlet area ratio >= minimum, from the MINIMUM opening of the built band (cylindrical probe
        shells across it, build_rocket / rocket_geometry.intake_opening_areas_mm2) and from Layout (at r_band_in); plus
        solid evidence of the v0.6.1 band: every window clear through (no leftover skin), every strut's mid-height
        horizontal section >= C9_STRUT_SECTION_MIN x the design sector and window_wall_t deep, and the band
        window_wall_t thick (radial probes) and solid in the margin rings under / over every window."""
        L = self.L
        intake = self.shape["intake_module"]
        ex = self.report["extra"]
        meas, meas_out = ex["inlet_area_ratio_measured"], ex["inlet_area_ratio_measured_outer"]
        req = L.p["intake_module"]["min_inlet_area_ratio"]
        ratio_ok = L.inlet_area_ratio >= req and meas >= req        # NaN (no evidence) compares False
        e, t = C9_PROBE_INSET, L.window_wall_t
        zm = 0.5 * (L.z_window_bottom + L.z_window_top)
        d_in = math.degrees(e / L.r_band_in)                        # angular inset of the window probes
        gable = L.has_window_gable
        # (a) every window clear through, r from inside the band to outside the body; v0.6.2: above the lower boundary
        # (the gable prism raised by the inset), so the gussets under it are not counted as a blocked window
        win_v = {}
        for (lo, hi), th in zip(L.window_sectors, L.window_thetas):
            probe = rg._sector(L.r_band_in - C9_WINDOW_PROBE_PAST, L.body_r + C9_WINDOW_PROBE_PAST, lo + d_in, hi - d_in,
                               L.z_window_bottom + e, L.z_window_top - e)
            if gable:
                probe = probe.intersect(rg.intake_window_gable_prism(L, th).translate(rg.V(0.0, 0.0, e)))
            win_v[f"{th:g}"] = _v(probe, intake)
        win_ok = max(win_v.values()) < C9_WINDOW_CLEAR_TOL_MM3
        gab = self._c9_gable(intake) if gable else {"ok": True}
        # (b) every strut: mid-height horizontal section (FLOOR_AREA_SLAB slab over the strut and a quarter of each
        # neighbouring window) and its radial extent at the strut centre
        half = 0.5 * L.strut_angle_deg + 0.25 * L.window_angle_deg
        struts, sect_min, depth_err = {}, math.inf, 0.0
        # v0.9 fix round: the motor-lead tie anchors stand on the 0 deg strut's inner face - section it in the free span
        # nearest mid-height (between two anchors) so the probe sees the strut itself
        zs_ = zm
        anc = list(getattr(L, "ml_anchor_z", []) or [])
        if anc:
            stops_ = [L.z_shelf_top] + anc + [L.z_intake_bulkhead_bottom]
            mids = [0.5 * (a + b) for a, b in zip(stops_[:-1], stops_[1:])]
            zs_ = min(mids, key=lambda z: abs(z - zm))
        for s in L.strut_thetas:
            z_p = zs_ if (anc and abs(((s - L.wire_theta + 180.0) % 360.0) - 180.0) < 1e-6) else zm
            slab = rg._sector(L.r_band_in - C9_WINDOW_PROBE_PAST, L.body_r + C9_WINDOW_PROBE_PAST, s - half, s + half,
                              z_p, z_p + FLOOR_AREA_SLAB)
            area = _vpos(slab, intake) / FLOOR_AREA_SLAB
            segs = rg.radial_material(intake, s, z_p, L.r_band_in - C9_WINDOW_PROBE_PAST, L.body_r + C9_WINDOW_PROBE_PAST)
            if len(segs) == 1:
                err = max(abs(segs[0][0] - L.r_band_in), abs(segs[0][1] - L.body_r))
            else:
                err = math.inf                                      # split or missing: not the design section
            struts[f"{s:g}"] = {"section_mm2": area, "radial": segs}
            sect_min, depth_err = min(sect_min, area), max(depth_err, err)
        sect_req = C9_STRUT_SECTION_MIN * L.strut_section_mm2
        strut_ok = sect_min >= sect_req and depth_err < C9_RADIAL_TOL
        # (c) band in the margin rings: radial probes at every window centre, and the ring under / over every window
        # span solid over the band's whole thickness
        rings = {"below": (L.z_shelf_top, L.z_window_bottom), "above": (L.z_window_top, L.z_intake_bulkhead_bottom)}
        ring_err, ring_void = 0.0, 0.0
        for z0, z1 in rings.values():
            for (lo, hi), th in zip(L.window_sectors, L.window_thetas):
                segs = rg.radial_material(intake, th, 0.5 * (z0 + z1), L.r_band_in - C9_WINDOW_PROBE_PAST,
                                          L.body_r + C9_WINDOW_PROBE_PAST)
                ring_err = max(ring_err, max(abs(segs[0][0] - L.r_band_in), abs(segs[0][1] - L.body_r))
                               if len(segs) == 1 else math.inf)
                ring_void = max(ring_void, _void(rg._sector(L.r_band_in + e, L.body_r - e, lo, hi, z0 + e, z1 - e), intake))
        band_ok = ring_err < C9_RADIAL_TOL and ring_void < C20_CLOSED_TOL_MM3
        ok = ratio_ok and win_ok and strut_ok and band_ok and gab["ok"]
        probes = ex.get("inlet_opening_probes_mm2", [])
        gab_txt = ""
        if gable:
            gab_txt = (f"; v0.6.2 gable: gusset void max {gab['gusset_void_max']:.4f} mm^3 (< {C9_GUSSET_VOID_TOL_MM3:g}), "
                       f"bridge above / below z_window_bottom {gab['bridge_above_max']:.4f} / {gab['bridge_below_void_max']:.4f} "
                       f"mm^3, lower edges {gab['n_sloped']} sloped min {gab['slope_min_deg']:.2f} deg (>= "
                       f"{GABLE_MIN_EDGE_SLOPE_DEG:g}), flat edges off the bridge {gab['flat_off_bridge']}, missing "
                       f"{len(gab['missing'])}")
        return Check("C9", "inlet area ratio (minimum opening of the built band) >= minimum; v0.6.1 band: windows clear "
                     "through, struts >= 0.95 x design section and window_wall_t deep, band window_wall_t thick; v0.6.2: "
                     "gussets solid under the gable, bridge flat at z_window_bottom, lower edges >= 35 deg",
                     PASS if ok else FAIL,
                     f"ratio layout {L.inlet_area_ratio:.3f} (min, r {L.inlet_area_r_min:.2f}) / {L.inlet_area_ratio_outer:.3f} "
                     f"(outer), measured min {meas:.3f} / outer {meas_out:.3f} >= {req:.2f}; windows clear max "
                     f"{max(win_v.values()):.4f} mm^3 (< {C9_WINDOW_CLEAR_TOL_MM3:g}); strut section min {sect_min:.2f} >= "
                     f"{sect_req:.2f} mm^2, depth err {depth_err:.4f}; band ring err {ring_err:.4f} mm, ring void "
                     f"{ring_void:.3f} mm^3" + gab_txt,
                     f"band r {L.r_band_in:.2f}..{L.body_r:.2f} (window_wall_t {t:g}), Z {L.z_shelf_top:.2f}.."
                     f"{L.z_intake_bulkhead_bottom:.2f}; windows (radial sectors) "
                     + ", ".join(f"{a:.2f}..{b:.2f}" for a, b in L.window_sectors)
                     + f" deg x Z {L.z_window_bottom:.2f}..{L.z_window_top:.2f}; struts {L.strut_w:g} at body_r / "
                     f"{L.strut_w_in:.2f} inner face, design section {L.strut_section_mm2:.2f} mm^2; opening probes "
                     f"(r_mean, mm^2) over Z {ex.get('inlet_opening_z_mm')}: "
                     + ", ".join(f"({r:.3f}, {a:.1f})" for r, a in probes)
                     + f"; layout area min {L.inlet_area_mm2:.1f} / outer {L.inlet_area_outer_mm2:.1f} mm^2 (square windows "
                       f"{L.inlet_area_square_mm2:.1f}), fan annulus {L.fan_annulus_area_mm2:.1f} mm^2; window probes x intake "
                       f"(mm^3, above the lower boundary): "
                     + ", ".join(f"{k} {v:.4f}" for k, v in win_v.items())
                     + "; strut sections (mm^2) / radial material: "
                     + "; ".join(f"{k} {v['section_mm2']:.2f} / " + ",".join(f"{a:.3f}..{b:.3f}" for a, b in v["radial"])
                                 for k, v in struts.items())
                     + (f"; gable (slope {L.gable_slope_deg:g} deg, bridge {L.gable_bridge_w:g} mm, h {L.gable_h_inner:.2f} / "
                        f"{L.gable_h_outer:.2f} at r_band_in / body_r, layout edge slope min {L.gable_edge_slope_min_deg:.2f} "
                        f"deg): gusset void per window " + ", ".join(f"{k} {v:.4f}" for k, v in gab["gusset_void"].items())
                        + "; lower edges per window (surface/side: z span, min slope) " + "; ".join(
                            f"{k} " + ", ".join(f"{s} {a:.2f}..{b:.2f} {m:.2f}" for s, (a, b, m) in v.items())
                            for k, v in gab["edges"].items())
                        + (f"; MISSING {gab['missing']}" if gab["missing"] else "") if gable else "; square windows (no gable in params)"),
                     {"layout": L.inlet_area_ratio, "layout_outer": L.inlet_area_ratio_outer, "measured": meas,
                      "measured_outer": meas_out, "required": req, "window_probe_mm3": win_v, "struts": struts,
                      "strut_section_min_mm2": sect_min, "strut_section_required_mm2": sect_req,
                      "strut_depth_err_mm": depth_err, "ring_radial_err_mm": ring_err, "ring_void_mm3": ring_void,
                      "gable": gab})

    def _c9_gable(self, intake: cq.Shape) -> dict:
        """v0.6.2 C9 evidence of the window gable (Layout.has_window_gable), all on the solid:
        * gussets solid: per window, the design gusset region inset C9_PROBE_INSET from the band faces, the window sides
          and the gable faces (the prism lowered by the inset) is material - void < C9_GUSSET_VOID_TOL_MM3;
        * bridge flat at z_window_bottom: over |y'| <= bridge_w/2 - inset (sector half angle asin((hf - e) / body_r)) a
          slab right above z_window_bottom is clear and one right below it is solid;
        * printability: every lower-boundary edge on the outer AND inner band surface (BRep edges lying on r = body_r or
          r_band_in inside a window sector, Z [z_window_bottom, z_gable_top]) is either flat at z_window_bottom within the
          bridge or rises at >= GABLE_MIN_EDGE_SLOPE_DEG from horizontal at every tangent sample; on each surface and
          each side of every window the sloped edges run from z_window_bottom to the gable height of that surface
          (missing edges are no evidence -> FAIL)."""
        L = self.L
        e = C9_PROBE_INSET
        res: dict = {"gusset_void": {}, "edges": {}, "missing": [], "flat_off_bridge": 0, "n_sloped": 0,
                     "slope_min_deg": math.inf}
        hf, d_in = L.gable_half_flat, math.degrees(e / L.r_band_in)
        bridge_half = math.degrees(math.asin(max(0.0, hf - e) / L.body_r))
        above, below = 0.0, 0.0
        for (lo, hi), th in zip(L.window_sectors, L.window_thetas):
            region = rg._sector(L.r_band_in + e, L.body_r - e, lo + d_in, hi - d_in, L.z_window_bottom + e,
                                L.z_gable_top + 1.0).cut(rg.intake_window_gable_prism(L, th).translate(rg.V(0.0, 0.0, -e)))
            res["gusset_void"][f"{th:g}"] = _void(region, intake)
            if bridge_half > 0.0:
                above = max(above, _v(rg._sector(L.r_band_in + e, L.body_r - e, th - bridge_half, th + bridge_half,
                                                 L.z_window_bottom + e, L.z_window_bottom + 2 * e), intake))
                below = max(below, _void(rg._sector(L.r_band_in + e, L.body_r - e, th - bridge_half, th + bridge_half,
                                                    L.z_window_bottom - 2 * e, L.z_window_bottom - e), intake))
        res["gusset_void_max"] = max(res["gusset_void"].values())
        res["bridge_above_max"], res["bridge_below_void_max"] = above, below
        # lower-boundary edges on the curved band surfaces
        tol, zb = C9_EDGE_TOL, L.z_window_bottom
        h_surf = {"outer": L.gable_h_outer, "inner": L.gable_h_inner}
        spans: dict[tuple, list] = {}
        for edge in intake.Edges():
            bb = edge.BoundingBox()
            if bb.zmin < zb - tol or bb.zmax > L.z_gable_top + tol:
                continue
            ts = [k / (C9_EDGE_SAMPLES - 1) for k in range(C9_EDGE_SAMPLES)]
            pts = [edge.positionAt(t) for t in ts]
            rs = [math.hypot(p.x, p.y) for p in pts]
            surf = ("outer" if all(abs(r - L.body_r) < tol for r in rs) else
                    "inner" if all(abs(r - L.r_band_in) < tol for r in rs) else None)
            if surf is None:
                continue
            mid = pts[len(pts) // 2]
            th_mid = math.degrees(math.atan2(mid.y, mid.x))
            win = [k for k, (lo, hi) in enumerate(L.window_sectors) if (th_mid - lo) % 360.0 <= hi - lo]
            if len(win) != 1:
                continue
            th = math.radians(L.window_thetas[win[0]])
            yps = [-p.x * math.sin(th) + p.y * math.cos(th) for p in pts]
            if bb.zmax - bb.zmin <= tol:                                   # flat: must be the bridge at z_window_bottom
                if abs(bb.zmax - zb) > tol or max(abs(y) for y in yps) > hf + tol:
                    res["flat_off_bridge"] += 1
                continue
            slopes = []
            for t in ts:
                tg = edge.tangentAt(t)
                slopes.append(math.degrees(math.atan2(abs(tg.z), math.hypot(tg.x, tg.y))))
            side = "+" if yps[len(yps) // 2] > 0 else "-"
            spans.setdefault((win[0], surf, side), []).append((bb.zmin, bb.zmax, min(slopes)))
            res["n_sloped"] += 1
            res["slope_min_deg"] = min(res["slope_min_deg"], min(slopes))
        for k, th_w in enumerate(L.window_thetas):
            rec = {}
            for surf in ("outer", "inner"):
                for side in ("+", "-"):
                    sp = spans.get((k, surf, side))
                    if not sp:
                        res["missing"].append(f"{th_w:g} {surf} {side}")
                        continue
                    z0, z1, m = min(s[0] for s in sp), max(s[1] for s in sp), min(s[2] for s in sp)
                    rec[f"{surf}{side}"] = (z0, z1, m)
                    if abs(z0 - zb) > tol or abs(z1 - (zb + h_surf[surf])) > tol:
                        res["missing"].append(f"{th_w:g} {surf} {side} spans {z0:.3f}..{z1:.3f}")
            res["edges"][f"{th_w:g}"] = rec
        res["ok"] = (res["gusset_void_max"] < C9_GUSSET_VOID_TOL_MM3 and above < C9_WINDOW_CLEAR_TOL_MM3
                     and below < C9_GUSSET_VOID_TOL_MM3 and not res["missing"] and res["flat_off_bridge"] == 0
                     and res["n_sloped"] > 0 and res["slope_min_deg"] >= GABLE_MIN_EDGE_SLOPE_DEG - 1e-6)
        return res

    def c10_min_feature(self) -> Check:
        p = self.L.p
        L = self.L
        mf = p["printer"]["min_feature"]
        designed = {
            "body.wall": p["body"]["wall"], "body.spigot_wall": p["body"]["spigot_wall"],
            "propulsion_module.cone_wall": p["propulsion_module"]["cone_wall"],
            "propulsion_module.sleeve_wall": p["propulsion_module"]["sleeve_wall"],
            "propulsion_module.web_t": p["propulsion_module"]["web_t"],
            "pod.wall_tangential": p["pod"]["wall_tangential"], "pod.wall_radial": p["pod"]["wall_radial"],
            "pod.floor_t": p["pod"]["floor_t"], "fin.t": p["fin"]["t"], "vane.t": p["vane"]["t"],
            "vane.shaft_d": p["vane"]["shaft_d"],
            **({"vane.horn_t": p["vane"]["horn_t"], "vane.horn_w": p["vane"]["horn_w"]} if not L.direct_drive else
               {"vane_drive.fairing.wall": p["vane_drive"]["fairing"]["wall"],
                "vane_drive.fairing.floor_t": p["vane_drive"]["fairing"]["floor_t"],
                "vane_drive.fairing.top_t": p["vane_drive"]["fairing"]["top_t"],
                "vane_drive.fairing.outer_block": p["vane_drive"]["fairing"]["outer_block"],
                "vane_drive.cover.t": p["vane_drive"]["cover"]["t"],
                **({"vane_drive.coupler.flange_t": p["vane_drive"]["coupler"]["flange_t"]}
                   if "flange_t" in p["vane_drive"]["coupler"] else
                   # v0.9 fix round: the horn socket's jaws (radial, outside the hub relief) and the set-screw wall
                   {"coupler socket jaw ((flange_d - collar relief)/2)": (L.dd_flange_d - L.dd_collar_relief_d) / 2,
                    "coupler hub wall at the flat (set-screw thread)": L.dd_set_screw_wall}),
                "servo_leads.fin_route.pipe.wall": p["servo_leads"]["fin_route"]["pipe"]["wall"]}),
            "intake_module.shelf_t": p["intake_module"]["shelf_t"], "intake_module.bulkhead_t": p["intake_module"]["bulkhead_t"],
            "intake_module.strut_w": p["intake_module"]["strut_w"],
            "intake_module.window_wall_t (band / strut thickness)": p["intake_module"]["window_wall_t"],
            "avionics_module.tray_t": p["avionics_module"]["tray_t"], "avionics_module.rail_w": p["avionics_module"]["rail_w"],
            "payload_module.bulkhead_t": p["payload_module"]["bulkhead_t"], "payload_module.piston_t": p["payload_module"]["piston_t"],
            "payload_module.latch_arm_t": p["payload_module"]["latch_arm_t"], "payload_module.latch_arm_w": p["payload_module"]["latch_arm_w"],
            "payload_module.latch_lug_h": p["payload_module"]["latch_lug_h"], "payload_module.latch_lug_w": p["payload_module"]["latch_lug_w"],
            "nose.wall": p["nose"]["wall"], "nose.anchor_bar_w": p["nose"]["anchor_bar_w"], "nose.anchor_bar_h": p["nose"]["anchor_bar_h"],
            "launch_pad.plate_t": p["launch_pad"]["plate_t"], "launch_pad.fin_locator_rib_w": p["launch_pad"]["fin_locator_rib_w"],
            "launch_pad.fin_locator_rib_h": p["launch_pad"]["fin_locator_rib_h"], "launch_pad.foot_t": p["launch_pad"]["foot_t"],
            "spigot shoulder ring (=body.wall)": rg.spigot_shoulder_t(L),
        }
        derived = {
            "hinge boss wall (boss_od-bore_d)/2": (L.hinge_boss_od - L.hinge_bore_d) / 2,
            "piston under recess (piston_t-recess_depth)": p["payload_module"]["piston_t"] - p["payload_module"]["piston_recess_depth"],
            # (the release pod keeps the pod cup and its ear holes in both vane_drive modes)
            "pod wall outside ear hole (pod_w/2-(pitch/2+d/2))": L.pod_w / 2 - (p["servo"]["ear_hole_pitch"] / 2 + p["servo"]["ear_hole_d"] / 2),
            "pod wall inside ear hole ((pitch/2-d/2)-cavity_w/2)": (p["servo"]["ear_hole_pitch"] / 2 - p["servo"]["ear_hole_d"] / 2) - L.pod_cavity_w / 2,
            "shaft D-flat remaining (shaft_d-d_flat_depth)": p["vane"]["shaft_d"] - p["vane"]["d_flat_depth"],
            "spring seat boss wall beside bulkhead holes": p["payload_module"]["bulkhead_hole_r"] - p["payload_module"]["bulkhead_hole_d"] / 2 - p["payload_module"]["spring_seat_boss_d"] / 2,
            "anchor bar wall beside tether eye ((bar_w-eye_d)/2)": (p["nose"]["anchor_bar_w"] - p["nose"]["tether_eye_d"]) / 2,
            # v0.6.1 intake band (SPEC 5.1): radial-sided struts keep strut_w_in at the band's inner face; the shelf
            # stands out past the band by shelf_ledge (Layout raises below min_feature for both)
            "intake strut inner face (strut_w x r_band_in / body_r)": L.strut_w_in,
            "intake shelf ledge past the band (r_band_in - shelf_id/2)": L.shelf_ledge,
        }
        if not L.direct_drive:
            derived["horn wall beside D-bore (horn_w-bore_d)/2"] = (p["vane"]["horn_w"] - L.hinge_bore_d) / 2
            derived["horn material above pin hole"] = rg.HORN_PIN_BELOW_TIP - p["vane"]["horn_pin_hole_d"] / 2
        else:            # v0.9 direct drive (Layout raises below min_feature for all of these)
            derived.update({f"fairing {k}": w for k, w in L.fair_walls.items() if k not in ("side wall", "top", "outer block")})
            derived.update({f"servo cover screw {k}": w for k, w in L.cov_tap_walls.items()})
            derived["coupler hub wall around the D-bore ((hub_d - bore_d)/2)"] = (L.dd_hub_d - L.dd_bore_d) / 2
            derived.update({f"lead pipe {k}": w for k, w in getattr(L, "sl_pipe_walls", {}).items()})
        if L.tabs and getattr(L, "wire_ch_open", False):    # v0.9 open motor-lead groove
            derived.update({f"motor-lead groove {k}": w for k, w in L.wire_cov_walls.items()})
            if getattr(L, "wire_lip_slit_w", 0.0):           # v0.9 fix round: lip slit, tie anchors
                derived["lip-pocket wall beside the slit, inside the gate ((gate_w - slit_w)/2)"] = (
                    (L.wire_gate_w - L.wire_lip_slit_w) / 2)
            if getattr(L, "ml_anchor_z", None):
                derived["tie anchor wall inside its tunnel (inner face)"] = L.ml_anchor_wall
                derived["tie anchor tunnel -> strut face"] = L.r_band_in - (L.ml_anchor_u + L.ml_anchor_wall + L.ml_anchor_tunnel[0])
                derived["tie anchor wall above / below its tunnel ((h - tunnel_h)/2)"] = (L.ml_anchor_h - L.ml_anchor_tunnel[1]) / 2
        if L.direct_drive and hasattr(L, "dd_slot_hw"):      # v0.9 fix round: coupler socket jaw beside the arm slot
            derived["coupler socket jaw beside the arm slot (flange_d/2 - slot half width)"] = L.dd_flange_d / 2 - L.dd_slot_hw
            derived["coupler socket jaw at the horn arm tips"] = L.dd_jaw_at_tip
        if L.has_com_board:   # v0.6 COM-board bosses (SPEC 5.3)
            derived["COM boss wall beside its blind hole ((boss_d-hole_d)/2)"] = (L.com_boss_d - L.com_hole_d) / 2
            derived["payload bulkhead skin over the COM blind hole (boss_h+bulkhead_t-hole_depth)"] = L.com_hole_skin
        if L.tabs:   # v0.5 tab mount (SPEC 3.7 / 5.1)
            wc = p["intake_module"]["wire_channel"]
            pm = p["propulsion_module"]
            designed["intake_module.wire_channel.wall"] = wc["wall"]
            designed["propulsion_module.liner_wall"] = pm["liner_wall"]
            designed["propulsion_module.tab_screw.floor_t"] = pm["tab_screw"]["floor_t"]
            if getattr(L, "clock_mount", False):
                # wire09c clock mount: no tab screws / lugs / wire slot; the new walls and features instead
                ck, lp = pm["clock_keyways"], pm["lead_passage"]
                designed["propulsion_module.clock_keyways.pad_t"] = ck["pad_t"]
                designed["propulsion_module.clock_keyways.col_wall"] = ck["col_wall"]
                designed["propulsion_module.lead_passage.wall"] = lp["wall"]
                derived.update({
                    "sleeve tooth between two keyways (min, at the bore)": L.ck_min_tooth,
                    "spigot ring outside the keyway (spigot_od/2 - keyway corner r)":
                        L.spigot_od / 2 - math.hypot(L.keyway_r_out, L.keyway_w / 2),
                    "lip-pocket wall beside the wire channel (wire_ch_r0 - lip_pocket_id/2)":
                        L.wire_ch_r0 - L.lip_pocket_id / 2,
                })
            def hole_to_bore(d: float, t_near: float) -> float:
                # a hole along t (|t| >= t_near) of diameter d: its point nearest the vehicle axis is
                # (r_s - d/2, t_near), so the wall to the EDF bore is hypot(r_s - d/2, t_near) - bore_r
                return math.hypot(L.tab_screw_r - d / 2, t_near) - L.edf_bore_r
            derived.update({} if getattr(L, "clock_mount", False) else {
                # (Layout.tab_cbore_wall_to_bore = r_seat - d_cbore/2 - bore_r is the conservative disc bound)
                "tab counterbore -> EDF bore (at t_head)": hole_to_bore(L.tab_screw_d_cbore, L.tab_screw_t_head),
                "tab clearance hole -> EDF bore (at the keyway face)": hole_to_bore(L.tab_screw_d_clear, L.tab_screw_t_keyway),
                "tab tap hole -> EDF bore (at the keyway face)": hole_to_bore(L.tab_screw_d_tap, L.tab_screw_t_keyway),
                "keyway floor (keyway_z0 - block_z0)": L.keyway_z0 - L.block_z0,
                "lug margin beyond the counterbore in Z (min over screws)":
                    min(min(z - L.tab_screw_d_cbore / 2 - z0, z1 - (z + L.tab_screw_d_cbore / 2))
                        for z in L.tab_screw_zs for z0, z1 in L.boss_z_ranges if z0 <= z <= z1),
                "spigot ring outside the keyway (spigot_od/2 - keyway corner r)":
                    L.spigot_od / 2 - math.hypot(L.keyway_r_out, L.keyway_w / 2),
                "deck outside the lower wire-slot corners (body_r - hypot(lower_r_out, slot_w/2))":
                    L.body_r - math.hypot(L.wire_slot_lower_r_out, L.wire_slot_w / 2),
                "lip-pocket wall beside the wire channel (wire_ch_r0 - lip_pocket_id/2)":
                    L.wire_ch_r0 - L.lip_pocket_id / 2,
            })
            if not getattr(L, "wire_ch_open", False):   # the v0.5d-v0.8 closed channel (v0.9: open groove + cover)
                derived.update({
                    "bulge outer wall (wire_bulge_r_out - wire_ch_r1)": L.wire_bulge_r_out - L.wire_ch_r1,
                    "bulge floor (wire_ch_z0 - wire_bulge_z0)": L.wire_ch_z0 - L.wire_bulge_z0,
                    "bulge roof (wire_bulge_z1 - wire_ch_z1)": L.wire_bulge_z1 - L.wire_ch_z1,
                })
        if L.servo_leads:   # v0.6 SPEC 3.8 / 5.1 (layout raises below min_feature too)

            def _ang(a: float, b: float) -> float:
                return math.radians(abs(((a - b) + 180.0) % 360.0 - 180.0))
            if not L.direct_drive:
                derived.update({
                    "pod lead window roof under the ear hole ((rim - ear hole depth) - sl_win_z1)":
                        (L.pod_rim_z - rg.EAR_HOLE_DEPTH) - L.sl_win_z1,
                    "pod end wall beside the lead window": min(L.sl_win_u0 - L.pod_cavity_r_in, L.pod_cavity_r_out - L.sl_win_u1)})
            derived.update({
                "entry slot -> web (min over columns)": min(
                    L.sl_entry_u0 * math.sin(_ang(t, w)) - L.sl_entry_w / 2 - L.web_t / 2
                    for t in L.sl_col_thetas for w in L.web_thetas),
                "deck notch -> joint hole (min over columns)": min(
                    L.sl_notch_u0 * math.sin(_ang(t, j)) - L.sl_notch_w / 2 - p["body"]["joint_hole_spigot_d"] / 2
                    for t in L.sl_col_thetas for j in L.joint_thetas),
                "deck (skirt seat) outside the servo-lead notch corners": L.sl_deck_seat,
                "servo-lead rebate floor over the joint holes": L.sl_rebate_floor_over_joint,
                "spigot ring inside the servo-lead rebate": L.sl_rebate_ring,
                "lip-pocket wall beside the servo channel (sl_ch_r0 - lip_pocket_id/2)": L.sl_ch_r0 - L.lip_pocket_id / 2,
                "servo bulge outer wall (sl_bulge_r_out - sl_ch_r1)": L.sl_bulge_r_out - L.sl_ch_r1,
                "servo bulge floor / roof": min(L.sl_ch_z0 - L.sl_bulge_z0, L.sl_bulge_z1 - L.sl_ch_z1),
            })
        if getattr(L, "has_gps_pod", False):   # v0.8 GPS pod (Layout raises below min_feature too)
            gq = p["payload_module"]["gps_pod"]
            designed.update({"payload_module.gps_pod.wall_t": gq["wall_t"], "payload_module.gps_pod.floor_t": gq["floor_t"],
                             "payload_module.gps_pod.lid.t": gq["lid"]["t"], "payload_module.gps_pod.raceway.wall": gq["raceway"]["wall"],
                             "payload_module.gps_pod.connector_pocket.wall": gq["connector_pocket"]["wall"]})
            derived.update({f"GPS {k}": w for k, w in L.gps_walls.items()
                            if k not in ("pod wall_t", "pod floor_t", "lid t", "raceway wall", "connector pocket wall",
                                         "raceway skin over the tube")})
        if getattr(L, "has_gns", False):   # v0.8.1 GNS bosses (Layout raises below min_feature too); the boss length at its
            # outer vertex is a length, not a wall
            derived.update({f"GNS {k}": w for k, w in L.gns_walls.items() if not k.startswith("shortest boss")})
        bad_designed = {k: v for k, v in designed.items() if v < mf - FEATURE_EPS}
        bad_derived = {k: v for k, v in derived.items() if v < mf - FEATURE_EPS}
        st = FAIL if bad_designed else (WARN if bad_derived else PASS)
        return Check("C10", f"minimum designed wall/feature >= printer.min_feature ({mf})", st,
                     f"min designed {min(designed.values()):.2f} ({min(designed, key=designed.get)}); min derived {min(derived.values()):.2f} ({min(derived, key=derived.get)})",
                     "designed: " + ", ".join(f"{k}={v:g}" for k, v in designed.items()) +
                     f"; intake band (v0.6.1): band thickness = strut thickness window_wall_t {L.window_wall_t:g} (body.wall "
                     f"{L.body_wall:g}), strut_w {L.strut_w:g} at body_r, strut inner face {L.strut_w_in:.2f}, shelf ledge past "
                     f"the band {L.shelf_ledge:.2f} mm"
                     + (f"; v0.6.2 window gussets: wedges standing on the ring below each window and joined to the struts, "
                        f"the full band thickness {L.window_wall_t:g} deep everywhere (their thin edge is the bridge end, "
                        f"which lies on the ring: no free feature thinner than the band) - no new minimum feature"
                        if L.has_window_gable else "") +
                     ("; DERIVED BELOW MIN: " + ", ".join(f"{k}={v:.2f}" for k, v in bad_derived.items()) if bad_derived else ""),
                     {"designed": designed, "derived": derived, "min_feature": mf})

    def c11_nose(self) -> Check:
        L = self.L
        cav = self.report["derived"]["nose_cavity_volume_L"]
        req = L.p["recovery"]["chute_pack_volume_L_required"]
        v = _v(self.shape["chute_pack"], self.shape["nose_cone"])
        ok = cav >= req and v < INTERSECT_TOL_MM3
        return Check("C11", "nose cavity above the anchor bar >= required chute volume; chute pack clears the nose",
                     PASS if ok else FAIL, f"cavity {cav:.3f} L >= {req:.2f} L; chute_pack x nose {v:.3f} mm^3",
                     f"layout integral {L.nose_cavity_volume_L:.3f} L", {"cavity_L": cav, "required_L": req, "chute_nose": v})

    def c12_thrust_weight(self) -> Check:
        L = self.L
        m = self.report["totals"]["mass_g"]
        tw = L.p["edf"]["thrust_nominal_N"] / (m * 1e-3 * G0)
        st = PASS if tw >= 1.30 else (WARN if tw >= 1.15 else FAIL)
        return Check("C12", "thrust-to-weight at nominal thrust (>=1.30 PASS, 1.15-1.30 WARN)", st,
                     f"T/W = {tw:.3f} ({L.p['edf']['thrust_nominal_N']:.1f} N / {m:.0f} g)",
                     f"printed {self.report['totals']['printed_mass_g']:.0f} g of {m:.0f} g", {"tw": tw, "mass_g": m})

    def barrowman(self) -> dict:
        L = self.L
        d, R = L.body_od, L.body_r
        n = 2000
        vol = 0.0
        for k in range(n):
            x = L.nose_len * (k + 0.5) / n
            vol += math.pi * L.nose_outer_radius(x) ** 2 * (L.nose_len / n)
        x_n = L.nose_len - vol / (math.pi * R * R)
        z_n = L.z_nose_tip - x_n
        cn_n = 2.0
        s = L.fin_tip_r - L.vane_section_r
        cr, ct = L.fin_root_chord, L.fin_tip_chord
        x_r = L.fin_root_le_z - L.fin_tip_le_z
        lm = math.hypot(s, (x_r + ct / 2) - cr / 2)
        nf = L.p["fin"]["count"]
        rt = L.vane_section_r
        cn_f = (1 + rt / (s + rt)) * (4 * nf * (s / d) ** 2) / (1 + math.sqrt(1 + (2 * lm / (cr + ct)) ** 2))
        x_f = x_r * (cr + 2 * ct) / (3 * (cr + ct)) + (1 / 6) * ((cr + ct) - cr * ct / (cr + ct))
        z_f = L.fin_root_le_z - x_f
        cn = cn_n + cn_f
        z_cp = (cn_n * z_n + cn_f * z_f) / cn
        return {"cn_nose": cn_n, "z_cp_nose": z_n, "cn_fins": cn_f, "z_cp_fins": z_f, "cn_total": cn, "z_cp": z_cp,
                "nose_volume_mm3": vol, "fin_span": s, "fin_mid_chord_len": lm}

    def c13_stability(self) -> Check:
        L = self.L
        z_cg = self.report["totals"]["com_mm"][2]
        h = z_cg - L.vane_hinge_z
        b = self.barrowman()
        margin = (z_cg - b["z_cp"]) / L.body_od
        st = PASS if (h >= 250.0 and margin >= 1.0) else (WARN if (h >= 250.0 and margin >= 0.5) else FAIL)
        return Check("C13", "CG >= 250 mm above the vane hinge; Barrowman CP (nose+fins) below CG (>=1.0 cal PASS, >=0.5 WARN)", st,
                     f"CG z {z_cg:.1f} (hinge+{h:.1f} mm); CP z {b['z_cp']:.1f}; static margin {margin:.2f} cal",
                     f"nose Cn {b['cn_nose']:.2f} @z{b['z_cp_nose']:.0f}, fins Cn {b['cn_fins']:.2f} @z{b['z_cp_fins']:.0f} (coast config = full vehicle)",
                     {"z_cg": z_cg, "cg_above_hinge": h, "margin_cal": margin, **b})

    def c14_placeholders(self) -> Check:
        L = self.L
        big_r = L.footprint_d / 2
        zone: dict[str, tuple[cq.Shape, list[str]]] = {}
        z_edf0 = L.z_edf_housing_bottom - (L.p["edf"]["motor_protrusion_rear"] if L.tabs else 0.0)   # v0.5: motor below
        zone["edf_unit"] = (rg._cyl_z(L.body_r, z_edf0 - 1.0, L.z_edf_lip_top), ["propulsion_module", "intake_module"])
        vtol = getattr(L, "vtol", False)
        for n in [n for n in ("battery", "buffer_pack", "esc", "nav", "misc_electronics") if n in self.shape]:
            zone[n] = (rg._cyl_z(L.body_r, L.z_avionics_bottom, L.z_avionics_top), ["avionics_module", "avionics_tray"]
                       + (["top_cap"] if vtol else []))
        fcu_n = getattr(L, "fcu_part", "fcu")
        if fcu_n in self.shape:   # v0.6.3: avionics zone, lies on the intake bulkhead inside the intake spigot (VTOL: carrier)
            zone[fcu_n] = (rg._cyl_z(L.body_r, L.z_avionics_bottom, L.z_avionics_top),
                           ["intake_module", "avionics_module", "avionics_tray"])
        if "com" in self.shape:   # v0.6: avionics zone, hangs from the payload bulkhead's bosses
            zone["com"] = (rg._cyl_z(L.body_r, L.z_avionics_bottom, L.z_avionics_top),
                           ["avionics_module", "avionics_tray", "payload_module"])
        if "gns" in self.shape:   # v0.8.1: avionics zone, on its bosses on the -x tube wall
            zone["gns"] = (rg._cyl_z(L.body_r, L.z_avionics_bottom, L.z_avionics_top), ["avionics_module", "avionics_tray"])
        if "radio_log" in self.shape:   # v0.6.2: avionics zone, hangs from the payload bulkhead's underside
            zone["radio_log"] = (rg._cyl_z(L.body_r, L.z_avionics_bottom, L.z_avionics_top),
                                 ["avionics_module", "avionics_tray", "payload_module"])
        for n in ("gps", "gps_ground_plane"):   # v0.8: in the GPS pod outside the payload tube, under its lid
            if n in self.shape and vtol:    # v0.9 CAD-2 VTOL: on the cap top, inside the body radius
                zone[n] = (rg._cyl_z(L.body_r, L.z_cap_top, L.z_cap_top + 30.0), ["top_cap"])
            elif n in self.shape:
                zone[n] = (rg._cyl_z(big_r, L.z_payload_bottom, L.z_payload_top), ["payload_module", "gps_pod_lid"])
        for i in range(L.vane_count):
            z = rg._cyl_z(big_r, L.z_exit, L.z_prop_top)
            names = ((f"servo_vane_{i + 1}", f"servo_horn_{i + 1}") if L.direct_drive else
                     (f"servo_vane_{i + 1}", f"servo_arm_{i + 1}", f"pushrod_{i + 1}"))
            for n in names:      # v0.9 direct drive: the servo and horn sit in the fairing (they must clear the cover too)
                zone[n] = (z, ["propulsion_module"] + ([f"servo_cover_{i + 1}", f"vane_coupler_{i + 1}"] if L.direct_drive
                                                       and n.startswith("servo_vane") else
                                                       [f"servo_cover_{i + 1}"] if L.direct_drive else []))
        for i in range(L.vane_count):         # v0.9 fix round: brass hinge bushings in the bosses (clear of the vane shaft)
            n = f"hinge_bushing_{i + 1}"
            if n in self.shape:
                zone[n] = (rg._cyl_z(big_r, L.z_exit, L.z_prop_top), ["propulsion_module", f"vane_{i + 1}",
                                                                       f"vane_coupler_{i + 1}"])
        for n in getattr(L, "tether_boxes", {}):   # v0.9 fix round: tether kit boxes in the avionics bay
            zone[n] = (rg._cyl_z(L.body_r, L.z_avionics_bottom, L.z_avionics_top),
                       ["avionics_module", "avionics_tray", "top_cap", "intake_module"])
        for n in ("tether_eyebolt", "tether_load_washer", "tether_nut"):   # v0.9 fix round: hardpoint stack
            if n in self.shape:
                zone[n] = (rg._cyl_z(L.body_r, L.z_avionics_spigot_bottom, L.z_cap_top + 40.0),
                           ["avionics_module", "avionics_tray", "top_cap"])
        if not vtol:
            zone["servo_release"] = (rg._cyl_z(big_r, L.z_payload_bottom, L.z_payload_top), ["payload_module"])
            zone["cansat_payload"] = (rg.payload_envelope(L), ["payload_module", "piston"])
            zone["spring"] = (rg._cyl_z(L.body_id / 2, L.z_payload_bottom, L.z_payload_top), ["payload_module", "piston"])
            zone["chute_pack"] = (rg.nose_inner_solid(L), ["nose_cone"])
        worst_out, worst_in, rows = (0.0, ""), (0.0, ""), []
        for name, (zs, mods) in zone.items():
            s = self.shape[name]
            outside = s.cut(zs).Volume()
            inter = max(_v(s, self.shape[m]) for m in mods)
            rows.append(f"{name}: out {outside:.2f},  x struct {inter:.2f}")
            if outside > worst_out[0]:
                worst_out = (outside, name)
            if inter > worst_in[0]:
                worst_in = (inter, name)
        vmax = max(worst_out[0], worst_in[0])
        return Check("C14", "every placeholder lies inside its module zone and clears the structure",
                     _status_no_intersection(vmax),
                     f"max outside-zone {worst_out[0]:.3f} mm^3 ({worst_out[1]}), max  x structure {worst_in[0]:.3f} mm^3 ({worst_in[1]})",
                     "zones: bore cylinder of the module (EDF: body_r over housing" + (" (tabs: motor)" if L.tabs else "") +
                     "..lip; servos/arms/pushrods: footprint radius over the module; CanSat: payload envelope; chute: nose inner void); " + "; ".join(rows),
                     {"worst_outside": worst_out, "worst_intersect": worst_in})

    def c15_latch(self) -> Check:
        L = self.L
        arm = self.shape["latch_arm"]
        rel = rg.latch_arm(L, released=True)
        lug = rg.latch_lug(L, overlap=0.0)
        lug_tall = rg._box_polar(L.lug_r_in, L.lug_r_out, -L.lug_w / 2, L.lug_w / 2, L.z_latch_arm_bottom, L.z_latch_arm_top, L.latch_lug_theta)
        t = L.p["payload_module"]["latch_arm_t"]
        area_latched = _vpos(arm, lug_tall) / t       # positive evidence: an uncomputable overlap counts 0 (FAIL)
        area_released = _v(rel, lug_tall) / t         # no-intersection test: uncomputable counts +inf (FAIL)
        ov_len = area_latched / L.latch_arm_w
        v_lug, v_nose = _v(arm, lug), _v(arm, self.shape["nose_cone"])
        v_rel_nose, v_rel_pay = _v(rel, self.shape["nose_cone"]), _v(rel, self.shape["payload_module"])
        ok = ov_len >= 4.0 and max(v_lug, v_nose, v_rel_nose, v_rel_pay) < INTERSECT_TOL_MM3 and area_released < INTERSECT_TOL_MM3
        tip_l, tip_r = rg.latch_arm_tip_xy(L), rg.latch_arm_tip_xy(L, released=True)
        return Check("C15", "latch arm overlaps the lug >= 4 mm in XY when latched, clears lug/skirt; zero overlap released",
                     PASS if ok else FAIL,
                     f"latched XY overlap {area_latched:.1f} mm^2 = {ov_len:.2f} mm along the arm; arm x lug {v_lug:.3f}, arm x nose {v_nose:.3f}; released overlap {area_released:.2f} mm^2, released x nose {v_rel_nose:.3f},  x payload {v_rel_pay:.3f} mm^3",
                     f"axis ({rg.latch_axis_xy(L)[0]:.2f},{rg.latch_axis_xy(L)[1]:.2f}) r={math.hypot(*rg.latch_axis_xy(L)):.2f}; latched tip ({tip_l[0]:.2f},{tip_l[1]:.2f}) r={math.hypot(*tip_l):.2f}; released tip r={math.hypot(*tip_r):.2f}; arm len {L.latch_arm_len:g}",
                     {"overlap_mm": ov_len, "area_latched": area_latched, "area_released": area_released, "arm_lug": v_lug, "arm_nose": v_nose})

    def c16_screw_holes(self) -> Check:
        """SPEC C16; v0.5d (SPEC 3.2): the propulsion spigot holes are blind, so their probe starts just past the hole
        bottom, and a probe from the EDF bore to just short of the bottom must be solid (positive evidence that the
        hole really is blind - a joint screw cannot reach the EDF housing)."""
        L = self.L
        worst, rows = (0.0, ""), []
        floor_min, floor_rows = math.inf, []
        for j in rg.joints(L):
            if not j.screwed:
                continue
            for th in L.joint_thetas:
                probe = rg.joint_probe(L, j, th)
                v_lo, v_up = _v(probe, self.shape[j.lower]), _v(probe, self.shape[j.upper])
                rows.append(f"{j.lower}/{j.upper} th{th:.0f} z{j.z_spigot_bottom + L.joint_screw_dz:.1f}: {v_lo:.3f}/{v_up:.3f}")
                for v, who in ((v_lo, j.lower), (v_up, j.upper)):
                    if v > worst[0]:
                        worst = (v, f"{who} th{th:.0f}")
                fp = rg.joint_hole_floor_probe(L, j, th)
                if fp is not None:
                    fill = _vpos(fp, self.shape[j.lower]) / fp.Volume()
                    floor_min = min(floor_min, fill)
                    floor_rows.append(f"{j.lower} th{th:.0f} {fill:.4f}")
        blind = [j for j in rg.joints(L) if j.screwed and j.hole_bottom_r is not None]
        # SPEC 3.2: with a prop_joint_hole_depth the propulsion holes MUST be blind and their floor must be measured
        need_blind = bool(L.prop_joint_hole_depth)
        floor_ok = (not need_blind or (blind and math.isfinite(floor_min))) and (not blind or floor_min >= JOINT_FLOOR_FILL_MIN)
        st = _status_no_intersection(worst[0]) if floor_ok else FAIL
        blind_txt = (f"; blind holes ({', '.join(j.lower for j in blind)}): floor r {blind[0].spigot_inner_r:.2f}.."
                     f"{blind[0].hole_bottom_r:.2f} (bore -> hole bottom, depth {L.prop_joint_hole_depth:g} from the "
                     f"spigot OD) filled min {floor_min:.4f} >= {JOINT_FLOOR_FILL_MIN:g}" if blind else "; no blind holes")
        return Check("C16", "D2 probes along every joint screw axis pass both the spigot and the outer tube "
                     "(v0.5d: propulsion holes blind)",
                     st, f"max probe x wall {worst[0]:.3f} mm^3 ({worst[1]}) over {len(rows)} probes" + blind_txt,
                     "; ".join(rows) + (f"; blind-hole floor fill: {'; '.join(floor_rows)}" if floor_rows else "")
                     + f"; blind-hole probes start {rg.JOINT_PROBE_INSET:g} mm past the hole bottom",
                     {"worst": worst, "blind_floor_fill_min": floor_min if blind else None})

    def c17_presets(self) -> Check:
        p = load_params()
        presets = [p["meta"]["preset"]] + list(p.get("alt_presets", {}))
        results = {}
        for name in presets:
            if name == self.L.p["meta"]["preset"]:
                results[name] = f"ok ({len(self.parts)} parts)"
                continue
            try:
                t = time.time()
                parts = rg.build_all(rg.layout_for(name))
                bad = [n for n, q in parts.items() if q.shape is not None and not q.shape.isValid()]
                results[name] = f"ok ({len(parts)} parts, {time.time() - t:.1f} s)" if not bad else f"INVALID solids: {bad}"
            except Exception as ex:  # noqa: BLE001 - the check reports any exception
                results[name] = f"EXCEPTION {type(ex).__name__}: {ex}"
        ok = all(r.startswith("ok") for r in results.values())
        return Check("C17", "all presets build without exceptions", PASS if ok else FAIL,
                     "; ".join(f"{k}: {v}" for k, v in results.items()), "", results)

    # ---------------- v0.5 EDF tab mount (SPEC 3.7 / 5.1), N/A for edf.mount = lip_clamp ----------------
    def _na(self, cid: str, title: str) -> Check:
        return Check(cid, title, NA, f"not applicable: edf.mount = {self.L.edf_mount!r} (tab-mount check)", "",
                     {"applicable": False})

    def c18_tab_slide(self) -> Check:
        title = "(tabs) EDF tabs slide down the keyways, seat without interference, keyway floor carries the tab"
        L = self.L
        if not L.tabs:
            return self._na("C18", title)
        if getattr(L, "clock_mount", False):
            st, meas, det, vals = ckc.c18_clock(L, self.prop, self.shape["intake_module"])
            return Check("C18", "(clock mount) EDF tabs slide down EVERY keyway, seat without interference, floor + floor pad "
                         "carry the tab (pad bending factor >= 5 PASS, 3-5 WARN, < 3 FAIL, E)", st, meas, det, vals)
        intake = self.shape["intake_module"]
        rows, worst_sweep, worst_seat, worst_gap, areas = [], 0.0, 0.0, 0.0, []
        for th in L.tab_thetas:
            v_sweep = _v(rg.tab_sweep(L, th), self.prop)
            tab = rg.edf_tab(L, th)
            v_seat = max(_v(tab, self.prop), _v(tab, intake))
            # keyway floor = module material directly under the tab footprint
            under = rg._box_polar(L.tab_r_in - rg.EDF_TAB_INTO_HOUSING, L.tab_r_out, -L.tab_t / 2, L.tab_t / 2,
                                  L.tab_z0 - FLOOR_PROBE_H, L.tab_z0, th)
            floor = self.prop.intersect(under)
            v_floor = floor.Volume() if not floor.wrapped.IsNull() else 0.0
            gap = (L.tab_z0 - floor.BoundingBox().zmax) if v_floor > 1e-9 else math.inf
            # bearing area: material in a FLOOR_AREA_SLAB-thin slab right under the tab footprint / slab height
            slab = rg._box_polar(L.tab_r_in - rg.EDF_TAB_INTO_HOUSING, L.tab_r_out, -L.tab_t / 2, L.tab_t / 2,
                                 L.tab_z0 - FLOOR_AREA_SLAB, L.tab_z0, th)
            area = _vpos(slab, self.prop) / FLOOR_AREA_SLAB
            rows.append(f"tab {th:.0f}deg: sweep x prop {v_sweep:.3f}, seated x modules {v_seat:.3f} mm^3, "
                        f"floor gap {gap:.4f} mm, floor area ~{area:.1f} mm^2")
            worst_sweep, worst_seat, worst_gap = max(worst_sweep, v_sweep), max(worst_seat, v_seat), max(worst_gap, gap)
            areas.append(area)
        area_req = C18_FLOOR_AREA_FRACTION * L.tab_t * (L.tab_r_out - L.edf_bore_r)
        ok = (max(worst_sweep, worst_seat) < INTERSECT_TOL_MM3 and worst_gap < C18_FLOOR_GAP_MAX
              and min(areas) >= area_req)
        return Check("C18", title, PASS if ok else FAIL,
                     f"max sweep x prop {worst_sweep:.3f}, seated tab x (prop|intake) {worst_seat:.3f} mm^3; "
                     f"max floor gap {worst_gap:.4f} mm (< {C18_FLOOR_GAP_MAX}); min floor area {min(areas):.2f} mm^2 "
                     f">= {area_req:.2f} ({C18_FLOOR_AREA_FRACTION:g} x tab_t x (tab_r_out - edf_bore_r))",
                     f"sweep Z {L.tab_z0:.1f}..{L.keyway_z1 + L.tab_z1 - L.tab_z0:.1f} (tab bottom up to keyway_z1 "
                     f"{L.keyway_z1:.1f}); keyway w {L.keyway_w:.1f} r {L.keyway_r_in:.2f}..{L.keyway_r_out:.2f} vs tab "
                     f"t {L.tab_t:.1f} r ..{L.tab_r_out:.1f}; " + "; ".join(rows),
                     {"sweep": worst_sweep, "seat": worst_seat, "floor_gap": worst_gap, "floor_area_mm2": areas})

    def c19_tab_screws(self) -> Check:
        title = "(tabs) tab screws: clear hole, open head tunnel to the outside, tap hole with material, axis through the tab slot"
        L = self.L
        if not L.tabs:
            return self._na("C19", title)
        if getattr(L, "clock_mount", False):
            return Check("C19", title, NA, "not applicable: propulsion_module.tab_mount 'clock_keyways' - no tab screws; the "
                         "EDF is held axially by the intake shelf / foam pads (C35) and turned by the keyway walls (C35 "
                         "bearing)", "", {"applicable": False, "reason": "no tab screws, see C35"})
        edf = self.shape["edf_unit"]
        skin = rg._tube(L.body_r, L.body_r - L.cone_wall, L.z_cyl0, L.z_prop_spigot_bottom)
        t_tun0, t_tun1 = L.tab_screw_t_head + C19_PROBE_INSET, L.tab_screw_t_cbore_end + C19_TUNNEL_PAST_END
        t_tap0, t_tap1 = L.tab_screw_t_tap_end + C19_PROBE_INSET, -L.tab_screw_t_keyway
        tap_len = t_tap1 - t_tap0
        tap_req = C19_TAP_MAT_FRACTION * math.pi / 4 * (C19_TAP_MAT_PROBE_D ** 2 - L.tab_screw_d_tap ** 2) * tap_len
        d_ctrl = L.tab_slot_w + C19_SLOT_CONTROL_EXTRA_D
        t_slot = L.tab_t / 2
        worst = {"clear": 0.0, "tunnel": 0.0, "tap_free": 0.0, "edf": 0.0}
        min_tap_mat, min_ctrl, min_skin, rows = math.inf, math.inf, math.inf, []
        n = 0
        for th in L.tab_thetas:
            for z in L.tab_screw_zs:
                n += 1
                probe = lambda d, t0, t1: rg.tab_screw_probe(L, th, z, d, t0, t1)  # noqa: E731
                tunnel = probe(C19_TUNNEL_PROBE_D, t_tun0, t_tun1)
                v = {
                    "clear": _v(probe(C19_CLEAR_PROBE_D, L.tab_screw_t_keyway, L.tab_screw_t_head), self.prop),
                    "tunnel": _v(tunnel, self.prop),
                    "tap_free": _v(probe(C19_TAP_FREE_PROBE_D, t_tap0, t_tap1), self.prop),
                    "edf": _v(probe(SHANK_PROBE_D, -t_slot, t_slot), edf),
                }
                tap_mat = _vpos(probe(C19_TAP_MAT_PROBE_D, t_tap0, t_tap1), self.prop)
                ctrl = _vpos(probe(d_ctrl, -t_slot, t_slot), edf)  # positive control: the tab is really there
                through_skin = _vpos(tunnel, skin)         # > 1 mm^3: the open tunnel crosses where the skin is
                for k in worst:
                    worst[k] = max(worst[k], v[k])
                min_tap_mat, min_ctrl = min(min_tap_mat, tap_mat), min(min_ctrl, ctrl)
                min_skin = min(min_skin, through_skin)
                rows.append(f"{th:.0f}deg z{z:.1f}: clear {v['clear']:.3f} tunnel {v['tunnel']:.3f} (skin crossed "
                            f"{through_skin:.1f}) tap {v['tap_free']:.3f} / O{C19_TAP_MAT_PROBE_D:g} {tap_mat:.2f} "
                            f"slot O{SHANK_PROBE_D:g} x edf {v['edf']:.3f} / O{d_ctrl:g} x edf {ctrl:.2f}")
        ok = (max(worst["clear"], worst["tap_free"], worst["edf"]) < INTERSECT_TOL_MM3
              and worst["tunnel"] < C19_TUNNEL_TOL_MM3 and min_tap_mat >= tap_req and min_ctrl > INTERSECT_TOL_MM3
              and min_skin > INTERSECT_TOL_MM3)
        return Check("C19", title, PASS if ok else FAIL,
                     f"{n} screws: max clear-hole {worst['clear']:.3f}, tunnel {worst['tunnel']:.4f} (< {C19_TUNNEL_TOL_MM3:g}), tap-hole "
                     f"{worst['tap_free']:.3f}, O{SHANK_PROBE_D:g} slot x edf {worst['edf']:.3f} mm^3; tunnel x skin annulus min "
                     f"{min_skin:.2f} mm^3 (> {INTERSECT_TOL_MM3:g}: it reaches the outside); min tap material "
                     f"{min_tap_mat:.2f} >= {tap_req:.2f} mm^3; slot control O{d_ctrl:g} x edf min {min_ctrl:.2f} mm^3 "
                     f"(> 0, i.e. above the {INTERSECT_TOL_MM3:g} mm^3 tolerance)",
                     f"axis r {L.tab_screw_r:.1f}, t keyway {L.tab_screw_t_keyway:.1f} / head {L.tab_screw_t_head:.1f} / tip "
                     f"{L.tab_screw_t_tip:.1f} / tap end {L.tab_screw_t_tap_end:.1f} / out {L.tab_screw_t_out:.2f} / cbore end "
                     f"{L.tab_screw_t_cbore_end:.2f}; tunnel probe t {t_tun0:.2f}..{t_tun1:.2f}; tap probes t {t_tap0:.2f}.."
                     f"{t_tap1:.2f} (len {tap_len:.2f}, required {C19_TAP_MAT_FRACTION:g} x pi/4 x ({C19_TAP_MAT_PROBE_D:g}^2 - "
                     f"{L.tab_screw_d_tap:g}^2) x len = {tap_req:.2f}); slot probes |t| <= {t_slot:.2f}; " + "; ".join(rows),
                     {**worst, "min_tap_material": min_tap_mat, "tap_material_required": tap_req,
                      "slot_control_min": min_ctrl, "slot_control_d": d_ctrl, "tunnel_skin_min": min_skin})

    def _screw_probes(self, axes, d_tap: float, d_clear: float, t_cover: float, target: cq.Shape, cover: cq.Shape) -> list[dict]:
        """Tap hole clear, material ring around it, cover clearance hole clear - for screw axes (head point, tap end)."""
        rows = []
        for a, b in axes:
            va, vb = cq.Vector(*a), cq.Vector(*b)
            ax = (vb - va).normalized()
            n = (vb - va).Length
            tap_clear = rg._cyl_between(d_tap - 2 * C5_PROBE_INSET, a, (va + ax * (n - C5_PROBE_INSET)).toTuple())
            ring = rg._cyl_between(d_tap + 2 * C5_TAP_RING, (va + ax * C5_PROBE_INSET).toTuple(), b).cut(
                rg._cyl_between(d_tap + 2 * C5_PROBE_INSET, a, b, ext0=1.0, ext1=1.0))
            hole = rg._cyl_between(d_clear - 2 * C5_PROBE_INSET, (va - ax * (t_cover + 0.5)).toTuple(), a)
            rows.append({"tap_clear": _v(tap_clear, target), "tap_material": _vpos(ring, target) / ring.Volume(),
                         "hole_clear": _v(hole, cover)})
        return rows

    def _c20_open(self, title: str) -> Check:
        """v0.9 (intake wire_channel.open): the three bare motor leads side by side (rocket_geometry.motor_lead_legs, grown
        by wire_clearance) clear the propulsion / intake modules, the EDF and the cover along the whole path; the groove
        is OPEN with the cover off (no material across its outer face), its side walls and roof wedge are solid, the
        wire_cover seats on it (no overlap, gap < 0.01) and its four screws have clear tap holes with material and clear
        cover holes; leg E keeps a BRep gap > 0 to the windowed band; the intake's outer skin has no other opening."""
        L = self.L
        intake, cover = self.shape["intake_module"], self.shape["wire_cover"]
        mods = {"prop": self.prop, "intake": intake, "edf": self.shape["edf_unit"], "cover": cover}
        legs = rg.motor_lead_legs(L)
        per_leg = {k: {m: _v(leg, s) for m, s in mods.items()} for k, leg in legs.items()}
        v_max = max(max(d.values()) for d in per_leg.values())
        th, hw, sw, i_ = L.wire_theta, L.wire_ch_w / 2, L.wire_side_wall, C5_PROBE_INSET
        open_probe = rg._box_polar(L.wire_ch_r1 - 0.5, L.wire_ch_r1 + 2.0, -hw + i_, hw - i_, L.z_intake_bottom + i_,
                                   L.wire_ch_z1 - i_, th)
        v_open = _v(open_probe, intake)
        walls = {f"side wall {'+' if s > 0 else '-'}": rg._box_polar(L.body_r + i_, L.wire_ch_r1 - i_,
                                                                      (hw + i_) if s > 0 else (-hw - sw + i_),
                                                                      (hw + sw - i_) if s > 0 else (-hw - i_),
                                                                      L.z_intake_bottom + i_, L.wire_ch_z1 - i_, th)
                 for s in (1.0, -1.0)}
        walls["roof wedge"] = rg._prism_uz(L.wire_roof_rz, -hw - sw + i_, hw + sw - i_, th).intersect(
            rg._tube(L.wire_ch_r1 - i_, L.body_r + i_, L.wire_ch_z1 + i_, L.wire_bulge_z1))
        taps = rg._fuse([rg._cyl_between(L.wire_cov_d_tap + 2 * i_, a, b, ext0=1.0, ext1=i_)
                         for a, b in rg.wire_cover_screw_axes(L)])
        missing = {k: _void(b.cut(taps), intake) for k, b in walls.items()}     # the tap holes are C20's screw rows
        seat = {"cover x intake": _v(cover, intake), "cover gap": _clearance(cover, intake)}
        covers_opening = (L.wire_cov_hv >= hw + sw - 1e-9 and L.wire_cov_z[0] <= L.z_intake_bottom + 1e-9
                          and L.wire_cov_z[1] >= L.wire_ch_z1 - 1e-9)
        screws = self._screw_probes(rg.wire_cover_screw_axes(L), L.wire_cov_d_tap, L.wire_cov_d_clear, L.wire_cov_t,
                                    intake, cover)
        # skin: the intake's outer 0.3 mm over [z_intake_bottom, z_window_bottom] has no opening except the joint holes,
        # the groove (with its side walls) and the servo channel
        shell = rg._tube(L.body_r, L.body_r - 0.3, L.z_intake_bottom, L.z_window_bottom)
        # (u from body_r - 3: at |v| = 15 the cylinder lies 1.9 mm inside the tangent plane's body_r)
        allowed = rg._box_polar(L.body_r - 3.0, L.body_r + 1.0, -hw - sw, hw + sw, L.z_intake_bottom - 1.0, L.wire_ch_z1 + 1.0, th)
        for jth in L.joint_thetas:
            jz = L.z_intake_bottom + L.joint_screw_dz
            allowed = allowed.fuse(rg._box_polar(L.body_r - 1.0, L.body_r + 1.0, -2.0, 2.0, jz - 2.0, jz + 2.0, jth))
        if L.servo_leads:
            allowed = allowed.fuse(rg._box_polar(L.body_r - 1.0, L.body_r + 1.0, -L.sl_ch_w / 2, L.sl_ch_w / 2,
                                                 L.sl_ch_z0, L.sl_ch_z1, L.sl_gather_theta))
        try:
            stray = _void(shell.cut(allowed), intake)
        except Exception:  # noqa: BLE001
            stray = math.inf
        band_gap = _clearance(legs["E"], self._intake_band_part())
        ok = (v_max < INTERSECT_TOL_MM3 and v_open < C29_CLEAR_TOL_MM3 and max(missing.values()) < C20_CLOSED_TOL_MM3
              and seat["cover x intake"] < INTERSECT_TOL_MM3 and seat["cover gap"] < 0.01 and covers_opening
              and all(r["tap_clear"] < C5_CLEAR_TOL_MM3 and r["tap_material"] >= C5_FILL_MIN and r["hole_clear"] < C5_CLEAR_TOL_MM3
                      for r in screws) and stray < C20_CLOSED_TOL_MM3 and band_gap > 0.0)
        sec = L.ml_sections
        return Check("C20", title, PASS if ok else FAIL,
                     f"3 leads (O{L.ml_od:g} + {L.ml_wire_clear:g}) x (prop/intake/edf/cover) max {v_max:.3f} mm^3; groove open "
                     f"(cover off) x intake {v_open:.4f} mm^3; side walls / roof missing max {max(missing.values()):.3f} mm^3; "
                     f"cover x intake {seat['cover x intake']:.3f} mm^3, gap {seat['cover gap']:.3f} mm, covers the opening "
                     f"{covers_opening}; screws tap clear max {max(r['tap_clear'] for r in screws):.4f}, material min "
                     f"{min(r['tap_material'] for r in screws):.3f}, cover holes clear max {max(r['hole_clear'] for r in screws):.4f}; "
                     f"skin stray {stray:.3f} mm^3; leg E -> band {band_gap:.2f} mm",
                     f"groove {L.wire_ch_w:g} x {L.wire_ch_r1 - L.wire_ch_r0:g} (r {L.wire_ch_r0:.2f}..{L.wire_ch_r1:.2f}), Z "
                     f"{L.z_intake_bottom:.1f}..{L.wire_ch_z1:.1f}, gate {L.wire_ch_gate_h:g} mm over the lip, bulkhead hole "
                     f"r {L.wire_hole_r0:g}..{L.wire_hole_r1:g} x {L.wire_ch_w:g}; sections (have vs need, mm): "
                     + "; ".join(f"{k} {d['have_w']:.1f}x{d['have_h']:.1f} vs {d['need_w']:.1f}x{d['need_h']:.1f}"
                                 for k, d in sec.items())
                     + "; per leg x (prop/intake/edf/cover): "
                     + "; ".join(f"{k} " + "/".join(f"{per_leg[k][m]:.3f}" for m in mods) for k in legs),
                     {"per_leg": per_leg, "open": v_open, "walls_missing": missing, "seat": seat,
                      "covers_opening": covers_opening, "screws": screws, "skin_stray": stray, "leg_e_band_gap_mm": band_gap,
                      "sections": sec})

    def _c29_clock(self) -> dict:
        """wire09c clock mount: the generosity-box corner grid (check_clock.clock_grid: every corner x e in {-E, 0, +E},
        grown envelope x module / intake, drop-in sweep x module, raw envelope x EDF, hovering tab gap, clearances,
        negative controls that must collide) + the bond-gap rule (TE inset + rim gap <= bond_gap_max: WARN 'thicker RTV
        bead or a shim', the limit is not raised) + the jet free span without the optional RTV bond."""
        L = self.L
        cache = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "exports", "c29_clock_grid")
        os.makedirs(cache, exist_ok=True)
        g = ckc.clock_grid(L, self.shape, cache, log=lambda m: print(m, flush=True))
        tol = ckc.CLOCK_CLEAR_TOL_MM3
        neg = min(g["negative_control_e_plus6_mm3"], g["negative_control_wall_plus3_mm3"],
                  g["negative_control_dropin_e_plus6_mm3"])
        hg = g["hover_tab_gap_min_mm"]
        cl = g["clearances_mm"]
        ok = (g["grown_x_module_max"][0] < tol and g["grown_x_intake_max"][0] < tol and g["dropin_x_module_max"][0] < tol
              and g["raw_x_edf_max"][0] < tol and neg > INTERSECT_TOL_MM3 and (hg is None or hg >= ckc.HOVER_TAB_GAP_MIN)
              and min(cl.values()) > 0.0 and max(L.ml_lower_free_spans) <= L.ml_lower_max_free_span + 1e-9)
        warns = []
        if L.ml_bond_gap_worst > L.ml_bond_gap_max + 1e-9:
            warns.append(f"bond gap TE inset {max(L.gb_inset):g} + rim gap {max(L.gb_gap):g} = {L.ml_bond_gap_worst:g} mm > "
                         f"bond_gap_max_ASSUMED {L.ml_bond_gap_max:g} (the optional RTV bond of the bundle to the stator TE "
                         f"needs a thicker RTV bead or a shim there; the limit is not raised)")
        txt = (f"; CLOCK MOUNT lead passage (every generosity-box corner x e in "
               f"{{{', '.join(f'{e:+.2f}' for e in g['e_values_deg'])}}} deg{', ' + g['from_cache'] if g.get('from_cache') else ''}): "
               f"{g['module_envelopes']} grown envelopes x module max {g['grown_x_module_max'][0]:.4f} mm^3 "
               f"({g['grown_x_module_max'][1]}), x intake max {g['grown_x_intake_max'][0]:.4f}; {g['dropin_sweeps']} drop-in "
               f"sweeps x module max {g['dropin_x_module_max'][0]:.4f}; {g['raw_x_edf_tests']} raw envelope x EDF tests max "
               f"{g['raw_x_edf_max'][0]:.4f} (TE inset 0 / 3, hub ends at the TE - A; protrusion "
               f"{min(L.gb_protrusion):g} / {max(L.gb_protrusion):g}; nominal + {len(L.ck_hover)} hover seats); negative "
               f"controls (must collide) e + 6 deg {g['negative_control_e_plus6_mm3']:.0f}, wall + 3 mm "
               f"{g['negative_control_wall_plus3_mm3']:.0f}, drop-in e + 6 {g['negative_control_dropin_e_plus6_mm3']:.0f} mm^3; "
               f"hovering tab -> leads min {hg if hg is None else round(hg, 2)} mm (>= {ckc.HOVER_TAB_GAP_MIN:g}); "
               f"clearances " + ", ".join(f"{k} {v:.1f}" for k, v in cl.items())
               + f" mm; jet free span without the RTV bond {max(L.ml_lower_free_spans):.1f} mm (<= "
               f"{L.ml_lower_max_free_span:g} ASSUMED); exit azimuth not an input (coverage C34)")
        return {"ok": ok, "warns": warns, "txt": txt, "grid": g}

    def c29_bullets(self) -> Check:
        """v0.9 (motor_leads): 5.5 mm bullets single file - a bullet grown by bullet_clearance (O bullet.d + 2 x 1 mm) with
        the other two leads beside it (rocket_geometry.bullet_group) passes (i) STRAIGHT UP through the intake module held
        alone (skirt bore, lip pocket, shelf notch, plenum beside the 0 deg strut, bulkhead hole) at the bulkhead hole's
        mid radius and (ii) up the open groove with the cover ON, beside the EDF and the propulsion spigot - every proxy
        x material < C29_CLEAR_TOL_MM3 (the clearance is in the proxy); (iii) the bullet joint zone in the avionics bay
        clears the avionics module, the intake and every bay component (>= com_mount.min_clear_to_tray, BRep) and the
        bore (Layout); the cross-sections and the route length / mass are Layout.ml_* numbers."""
        L = self.L
        title = ("(v0.9) 5.5 mm bullets single file: straight up through the intake + bulkhead hole, up the groove with the "
                 "cover on, joint zone in the avionics bay; (fix round) the leads move interior -> groove through the "
                 "lip-pocket slit, plenum leads tied off the EDF throat; (2026-10-29 photo round) leads from the motor's front "
                 "ring behind a stator vane, under the housing rim, up the lead notch")
        if not getattr(L, "has_motor_leads", False) or not getattr(L, "wire_ch_open", False):
            return Check("C29", title, NA, "not applicable: params carry no motor_leads / open groove", "", {"applicable": False})
        intake, cover = self.shape["intake_module"], self.shape["wire_cover"]
        u_in = 0.5 * (L.wire_hole_r0 + L.wire_hole_r1)
        u_g = 0.5 * (L.wire_ch_r0 + L.wire_ch_r1)
        bl = L.ml_bullet_len
        g1 = rg.bullet_group(L, u_in, L.z_intake_bottom - bl, L.z_intake_bulkhead_top + bl)
        r1 = {k: _v(s, intake) for k, s in g1.items()}
        g2 = rg.bullet_group(L, u_g, L.z_intake_bottom, L.z_lip_pocket_top)
        tg = {"prop": self.prop, "intake": intake, "edf": self.shape["edf_unit"], "cover": cover}
        r2 = {f"{k} x {m}": _v(s, t) for k, s in g2.items() for m, t in tg.items()}
        zone = rg.motor_joint_zone(L)
        bay = {n: s for n, s in self.shape.items() if n in ("avionics_tray", "battery", "esc", "nav", "misc_electronics",
                                                            "fcu", "gns", "com", "radio_log", "carrier", "buffer_pack")}
        x_mod = max(_v(zone, self.shape["avionics_module"]), _v(zone, intake))
        gaps = {n: _clearance(zone, s) for n, s in bay.items()}
        lim = L.ml_joint_min_clear
        g_mod = _clearance(zone, self.shape["avionics_module"])
        ok = (max(r1.values()) < C29_CLEAR_TOL_MM3 and max(r2.values()) < C29_CLEAR_TOL_MM3 and x_mod < INTERSECT_TOL_MM3
              and min(gaps.values()) >= lim - 1e-6 and g_mod >= PLACEHOLDER_BORE_CLEAR_MM - 1e-6)
        # v0.9 fix round: (iv) assembly sequence - the lip-pocket wall is open at wire_theta for a bare lead (O od + 2 x
        # wire_clearance) from the interior to the groove over the whole wall height (else the lead threaded up the interior
        # cannot reach the groove / gate: the wall would be a closed ring); (v) plenum - the leads' envelope stays >=
        # throat_clear_min outside the EDF throat, no free span > max_free_span, every tie anchor present with an open tunnel
        seq, ple = {}, {}
        if getattr(L, "wire_lip_slit_w", 0.0):
            hl = 0.5 * L.ml_od + L.ml_wire_clear
            probe = rg._box_polar(L.lip_pocket_id / 2 - 1.0, L.wire_ch_r0 + 1.0, -hl, hl, L.z_intake_skirt_top + 0.2,
                                  L.z_lip_pocket_top - 0.2, L.wire_theta)
            seq["slit x intake"] = _v(probe, intake)
            ring = rg._tube(L.wire_ch_r0 - 0.1, L.lip_pocket_id / 2 + 0.1, L.z_intake_skirt_top + 0.5, L.z_lip_pocket_top - 0.5)
            seq["wall ring fraction"] = _vpos(ring, intake) / ring.Volume()
            ok = ok and seq["slit x intake"] < C29_CLEAR_TOL_MM3 and seq["wall ring fraction"] < 1.0 - 1e-4
        if getattr(L, "ml_anchor_z", None):
            pl_ = L.p["motor_leads"]["plenum"]
            ple["throat clear mm"] = L.ml_throat_clear
            ple["max free span mm"] = max(L.ml_free_spans)
            t_, h_ = L.ml_anchor_tunnel
            u0 = L.ml_anchor_u + L.ml_anchor_wall
            hw = L.ml_anchor_w / 2
            i_ = C5_PROBE_INSET
            tun = [rg._box_polar(u0 + i_, u0 + t_ - i_, -hw - 0.5, hw + 0.5, zc - h_ / 2 + i_, zc + h_ / 2 - i_, L.wire_theta)
                   for zc in L.ml_anchor_z]
            mat = [rg._box_polar(L.ml_anchor_u + i_, u0 - i_, -hw + i_, hw - i_, zc - L.ml_anchor_h / 2 + i_,
                                 zc + L.ml_anchor_h / 2 - i_, L.wire_theta) for zc in L.ml_anchor_z]
            ple["tunnels open (max x intake)"] = max(_v(s, intake) for s in tun)
            ple["anchor inner wall fill (min)"] = min(_vpos(s, intake) / s.Volume() for s in mat)
            ple["anchors"] = len(L.ml_anchor_z)
            ok = (ok and L.ml_throat_clear >= pl_["throat_clear_min"] - 1e-9 and ple["max free span mm"] <= pl_["max_free_span"] + 1e-9
                  and ple["tunnels open (max x intake)"] < C29_CLEAR_TOL_MM3 and ple["anchor inner wall fill (min)"] >= C5_FILL_MIN)
        # v0.9 lead-exit, photo round (user photos 2026-10-29): (vi) the EDF-internal route (motor side at its front ring ->
        # heat-shrink bundle behind the stator vane's trailing edge -> under the housing rim -> single leads bend up the
        # slot; round 1's climb along the can is superseded): every leg / bend envelope
        # (grown by wire_clearance) x propulsion module / EDF / intake < C29_CLEAR_TOL_MM3, the drop-in sweep (EDF lowered
        # into its bore with the leads formed) x propulsion module < C29_CLEAR_TOL_MM3, BRep gaps to the thrust vane at
        # wire_theta (neutral, +- max mechanical deflection) and its coupler / horn / servo > 0, Layout's clearances (rim
        # corner, housing, skin, fins, fairing, rotor) met, free spans <= lower_route.max_free_span_jet_ASSUMED with the RTV
        # bond to the stator vane's trailing edge as the only restraint inside the EDF (review 2026-10-29 G2: no tie round
        # the vane - it would loop over the leading edge next to the rotor)
        low = {}
        clk: dict = {}
        if getattr(L, "clock_mount", False):
            clk = self._c29_clock()
            ok = ok and clk["ok"]
        if getattr(L, "ml_lower_uz", None) and not getattr(L, "clock_mount", False):
            legs_l = rg.motor_lead_lower_legs(L)
            tgl = {"prop": self.prop, "edf": self.shape["edf_unit"], "intake": intake}
            low["legs x material"] = {f"{k} x {m}": _v(s, t) for k, s in legs_l.items() for m, t in tgl.items()}
            low["drop-in sweep x prop"] = {k: _v(s, self.prop) for k, s in rg.motor_lead_dropin_sweep(L).items()}
            lower = rg._fuse(list(legs_l.values()))
            k0 = min(range(L.vane_count), key=lambda i: abs(((L.vane_thetas[i] - L.wire_theta) + 180.0) % 360.0 - 180.0))
            dmax = float(L.ml_max_deflection_deg or 0.0)
            gaps_l = {f"vane_{k0 + 1} d={d:+.0f}": _clearance(lower, rg.vane(L, k0, d)) for d in (0.0, dmax, -dmax)}
            for nm in (f"vane_coupler_{k0 + 1}", f"servo_horn_{k0 + 1}", f"servo_vane_{k0 + 1}"):
                if nm in self.shape:
                    gaps_l[nm] = _clearance(lower, self.shape[nm])
            low["gaps"] = gaps_l
            low["layout clearances"] = dict(L.ml_lower_clear)
            low["free spans"] = list(L.ml_lower_free_spans)
            low["RTV bond u"] = list(L.ml_bond_u)
            low["bend r (centre) / inner"] = (L.ml_bend_r, L.ml_bend_r - L.ml_od / 2)
            low["stator TE gap"] = L.ml_stator_te_gap
            low["bundle od / shrink end u / bend start - shrink end"] = (L.ml_bundle_d, L.ml_u_shrink_end,
                                                                         L.ml_shrink_end_margin)
            low["transition (triangle -> row) len / bend start - (shrink end + transition)"] = (
                L.ml_transition_len, L.ml_shrink_bend_margin)
            low["bundle bend angle deg"] = L.ml_shrink_bend_phi_deg
            low["exit angle tolerance deg (E) / uncertainty deg (ASSUMED) / measured"] = (
                L.ml_exit_angle_tol_deg, L.ml_theta_uncert_deg, L.ml_theta_measured)
            low["bundle envelope end u / inside duct_r"] = (L.ml_u_bundle_env_end, L.ml_bundle_env_in_duct)
            low["segments mm"] = dict(L.ml_lower_segments)
            low["blockage"] = dict(L.ml_blockage)
            ok = (ok and max(low["legs x material"].values()) < C29_CLEAR_TOL_MM3
                  and max(low["drop-in sweep x prop"].values()) < C29_CLEAR_TOL_MM3 and min(gaps_l.values()) > 0.0
                  and max(L.ml_lower_free_spans) <= L.ml_lower_max_free_span + 1e-9)
        # fix round (review 2026-10-29): WARN (never PASS silently) when (G2) the shrink + triangle -> row transition runs
        # into the bend (Layout.ml_shrink_bend_margin < 0: the stiff bundle / unformed leads would take the first part of
        # the bend), or (G1) the exit angle is not measured and the route's tolerance (E) is below its uncertainty
        warns = []
        if low:
            if L.ml_shrink_bend_margin < 0.0:
                warns.append(f"shrink + transition end {-L.ml_shrink_bend_margin:.2f} mm inside the bend "
                             f"({L.ml_shrink_bend_phi_deg:.1f} deg of it)")
            if not L.ml_theta_measured and L.ml_exit_angle_tol_deg < L.ml_theta_uncert_deg:
                warns.append(f"exit angle NOT MEASURED: route tolerates +-{L.ml_exit_angle_tol_deg:.1f} deg (E) vs "
                             f"+-{L.ml_theta_uncert_deg:g} deg uncertainty (ASSUMED) - measure before printing the "
                             f"propulsion module")
            if not L.ml_bundle_env_in_duct:
                warns.append("the bundle envelope reaches the duct wall: the exit-angle tolerance is the rigid bundle's")
        fix_txt = ""
        if low:
            wl = max(low["legs x material"], key=low["legs x material"].get)
            ws_ = max(low["drop-in sweep x prop"], key=low["drop-in sweep x prop"].get)
            gl = min(low["gaps"], key=low["gaps"].get)
            lc = low["layout clearances"]
            fix_txt += (f"; lower route (photo round 2026-10-29: bundle O{L.ml_bundle_d:g} from the motor's front ring at z "
                        f"{L.ml_z_exit:.2f} ({L.ml_stator_te_gap:.2f} under the stator TE / rim), re-sleeved shrink end u "
                        f"{L.ml_u_shrink_end:.2f} + triangle -> row {L.ml_transition_len:.2f} (E), bend start - (shrink end + "
                        f"transition) {L.ml_shrink_bend_margin:+.2f} mm, exit-angle tolerance +-{L.ml_exit_angle_tol_deg:.1f} "
                        f"deg (E; {'measured' if L.ml_theta_measured else 'NOT measured'}), slot u "
                        f"{L.ml_u_slot:.3f}, 1 bend R {L.ml_bend_r:g}): legs x (prop/edf/intake) max {low['legs x material'][wl]:.4f} mm^3 ({wl}), "
                        f"drop-in sweep x prop max {low['drop-in sweep x prop'][ws_]:.4f} mm^3 ({ws_}), min gap {low['gaps'][gl]:.2f} "
                        f"mm ({gl}), rim corner {lc['housing rim corner -> leads / bundle']:.2f}, housing OD "
                        f"{lc['housing OD -> leads in the slot']:.2f}, notch skin {lc['notch outer corner -> cone skin']:.2f}, "
                        f"thrust-vane sweep {lc['thrust-vane sweep (any deflection) -> leads']:.1f}, fin {lc['fin (root LE) -> leads']:.1f}, "
                        f"fairing {lc['servo fairing / pocket / coupler -> leads']:.1f}, rotor/stator LE "
                        f"{lc['rotor (ahead of the stator LE, ASSUMED) -> bundle top']:.1f} mm; free spans "
                        + ", ".join(f"{s:.1f}" for s in L.ml_lower_free_spans)
                        + f" mm (<= {L.ml_lower_max_free_span:g} ASSUMED; RTV bond to the stator TE at u {L.ml_bond_u[0]:.1f}.."
                        f"{L.ml_bond_u[1]:.1f}, gap {L.ml_stator_te_gap:.2f} mm); jet blockage (E) "
                        f"{L.ml_blockage['projected_pct']:.1f} % projected / {L.ml_blockage['drag_area_pct']:.1f} % drag area")
        if seq:
            fix_txt += (f"; lip slit: bare-lead probe x intake {seq['slit x intake']:.4f} mm^3, lip-pocket wall ring "
                        f"{100 * seq['wall ring fraction']:.1f} % material (< 100 %: open)")
        if ple:
            fix_txt += (f"; plenum: leads {ple['throat clear mm']:.2f} mm outside the EDF throat (>= "
                        f"{L.p['motor_leads']['plenum']['throat_clear_min']:g}), {ple['anchors']} tie anchors, max free span "
                        f"{ple['max free span mm']:.1f} mm (<= {L.p['motor_leads']['plenum']['max_free_span']:g}), tunnels x intake "
                        f"{ple['tunnels open (max x intake)']:.4f} mm^3, anchor wall fill {ple['anchor inner wall fill (min)']:.3f}")
        if clk:
            warns += clk["warns"]
            fix_txt += clk["txt"]
        if warns:
            fix_txt += "; WARN: " + "; ".join(warns)
        return Check("C29", title, (WARN if warns else PASS) if ok else FAIL,
                     f"straight up through the intake x intake max {max(r1.values()):.4f} mm^3; groove (cover on) max "
                     f"{max(r2.values()):.4f} mm^3; joint zone x (avionics / intake) {x_mod:.3f} mm^3, gap to the avionics module "
                     f"{g_mod:.2f} mm (>= {PLACEHOLDER_BORE_CLEAR_MM:g}), min gap to the bay {min(gaps.values()):.2f} mm "
                     f"({min(gaps, key=gaps.get)}, >= {lim:g})" + fix_txt,
                     f"bullet O{L.ml_bullet_d:g} + 2 x {L.ml_bullet_clear:g} = O{L.ml_bullet_d + 2 * L.ml_bullet_clear:g} x "
                     f"{bl:g} (ASSUMED) beside {L.ml_count - 1} leads O{L.ml_od:g}; pass (i) at r {u_in:.2f} Z "
                     f"{L.z_intake_bottom - bl:.1f}..{L.z_intake_bulkhead_top + bl:.1f}; pass (ii) at r {u_g:.2f} Z "
                     f"{L.z_intake_bottom:.1f}..{L.z_lip_pocket_top:.1f}; joint zone "
                     + " x ".join(f"{a:.1f}..{b:.1f}" for a, b in L.ml_joint_box)
                     + f"; lead route {L.ml_route_len:.0f} mm + slack -> {L.ml_lead_len:.0f} mm per lead, {L.ml_mass_g:.1f} g "
                       f"(ASSUMED g/m and bullets); gaps " + ", ".join(f"{n} {g:.2f}" for n, g in gaps.items()),
                     {"interior_pass": r1, "groove_pass": r2, "zone_x_module": x_mod, "zone_gap_module": g_mod, "zone_gaps": gaps,
                      "route_mm": L.ml_route_len, "lead_mm": L.ml_lead_len, "mass_g": L.ml_mass_g,
                      "assembly_sequence": seq, "plenum": ple, "free_spans": getattr(L, "ml_free_spans", None),
                      "anchor_z": getattr(L, "ml_anchor_z", None), "lower_route": low,
                      "clock_grid": clk.get("grid") if clk else None})

    def c20_wire_path(self) -> Check:
        title = "(tabs) 12x5 mm motor-lead bundle: slot -> steps out below the lip -> channel -> steps in through the shelf notch -> bulkhead hole"
        L = self.L
        if not L.tabs:
            return self._na("C20", title)
        if getattr(L, "wire_ch_open", False):
            return self._c20_open("(tabs, v0.9) three 12 AWG motor leads: slot -> open groove past the lip (cover) -> gate -> "
                                  "bulkhead hole; groove open, walls / roof solid, cover seated and screwed")
        mods = {"prop": self.prop, "intake": self.shape["intake_module"], "edf": self.shape["edf_unit"]}
        legs = rg.wire_bundle_legs(L)
        steps = rg.wire_bundle_steps(L)
        per_leg = {k: {m: _v(leg, s) for m, s in mods.items()} for k, leg in legs.items()}
        proxy = rg.wire_bundle_proxy(L)
        total = {m: _v(proxy, s) for m, s in mods.items()}
        v_tot = sum(total.values())
        # the step legs lie inside the Z windows SPEC 7 gives them (v0.5d: step out within [wire_gate_lo_z0,
        # z_intake_skirt_top], above the floor wedge), and the channel leg C is outside wire_ch_r0
        T = rg.WIRE_BUNDLE_T
        c_ch = (L.wire_ch_r0 + L.wire_ch_r1) / 2
        win_ok = (L.wire_gate_lo_z0 - 1e-9 <= steps["out"][0] and steps["out"][1] <= L.z_intake_skirt_top + 1e-9
                  and L.wire_notch_z0 - 1e-9 <= steps["in"][0] and steps["in"][1] <= L.wire_ch_z1 + 1e-9
                  and c_ch - T / 2 >= L.wire_ch_r0 - 1e-9)
        # gates (informational).  Below the lip: from the pocket wall's bottom corner (u wire_ch_r0, Z skirt top) to
        # the nearest point of the channel floor - the floor wedge (triangle wire_ch_floor_rz, the same at every v) or,
        # at the bundle's side faces |v| = W/2, the flat channel bottom where the tube's inner face is at
        # u = sqrt((body_id/2)^2 - (W/2)^2).  Above the lip: pocket-wall top corner to the tube / bulge-roof corner.
        hv = rg.WIRE_BUNDLE_W / 2
        u_tube_in = math.sqrt((L.body_id / 2) ** 2 - hv ** 2)
        p_lo = (L.wire_ch_r0, L.z_intake_skirt_top)
        tri = list(L.wire_ch_floor_rz)
        gate_wedge = min(_pt_seg_dist(p_lo, tri[k], tri[(k + 1) % len(tri)]) for k in range(len(tri)))
        gate_flat = math.hypot(u_tube_in - L.wire_ch_r0, L.z_intake_skirt_top - L.wire_ch_z0)
        gate_lo = min(gate_wedge, gate_flat)
        # above the lip (v0.6.1): the roof of the step-in is the windowed band's bottom face over the notch (Z
        # wire_ch_z1) from its inner face u_band_in outward; when that face lies inside the pocket wall's top corner
        # (u wire_ch_r0), the gate is the vertical gap, else the corner-to-corner diagonal
        u_band_in = math.sqrt(L.r_band_in ** 2 - hv ** 2)
        gate_hi = math.hypot(max(0.0, u_band_in - L.wire_ch_r0), L.wire_ch_z1 - L.wire_notch_z0)
        leg_txt = "; ".join(f"{k} " + "/".join(f"{per_leg[k][m]:.3f}" for m in mods) for k in legs)
        # the channel is closed outside: bulge outer wall, side walls, floor wedge (v0.5d: the triangle
        # wire_ch_floor_rz over the channel width, unioned after the cuts) and roof are solid, and the intake's
        # outer 0.3 mm over [z_intake_bottom, z_window_bottom] has no opening except the joint holes and the
        # channel behind the bulge
        intake = mods["intake"]
        th_w, hw, wall = L.wire_theta, L.wire_ch_w / 2, L.wire_ch_wall
        closure = {
            "outer wall": rg._box_polar(L.wire_ch_r1, L.wire_bulge_r_out, -hw, hw, L.wire_ch_z0, L.wire_ch_z1, th_w),
            "side walls": rg._box_polar(L.body_r, L.wire_ch_r1, hw, hw + wall, L.wire_ch_z0, L.wire_ch_z1, th_w).fuse(
                rg._box_polar(L.body_r, L.wire_ch_r1, -hw - wall, -hw, L.wire_ch_z0, L.wire_ch_z1, th_w)),
            "floor wedge": rg.intake_wire_floor_wedge(L),
            "roof": rg._box_polar(L.body_r, L.wire_bulge_r_out, -hw - wall, hw + wall, L.wire_ch_z1, L.wire_bulge_z1, th_w),
        }
        holes = {k: _void(b, intake) for k, b in closure.items()}
        shell = rg._tube(L.body_r, L.body_r - 0.3, L.z_intake_bottom, L.z_window_bottom)
        allowed = rg._box_polar(L.body_r - 1.0, L.body_r + 1.0, -hw, hw, L.wire_ch_z0, L.wire_ch_z1, th_w)
        for jth in L.joint_thetas:
            jz = L.z_intake_bottom + L.joint_screw_dz
            allowed = allowed.fuse(rg._box_polar(L.body_r - 1.0, L.body_r + 1.0, -2.0, 2.0, jz - 2.0, jz + 2.0, jth))
        if L.servo_leads:   # v0.6: the servo channel behind the servo bulge (SPEC 5.1; its closure is C23's)
            allowed = allowed.fuse(rg._box_polar(L.body_r - 1.0, L.body_r + 1.0, -L.sl_ch_w / 2, L.sl_ch_w / 2,
                                                 L.sl_ch_z0, L.sl_ch_z1, L.sl_gather_theta))
        try:
            stray = _void(shell.cut(allowed), intake)
        except Exception:  # noqa: BLE001
            stray = math.inf
        closed_ok = max(holes.values()) < C20_CLOSED_TOL_MM3 and stray < C20_CLOSED_TOL_MM3
        # v0.6.1: leg E runs up beside the strut inside the thicker band - measured BRep gap, it must not touch it
        band_gap = _clearance(legs["E"], self._intake_band_part())
        st = (_status_no_intersection(v_tot) if win_ok and band_gap > 0.0 else FAIL) if closed_ok else FAIL
        return Check("C20", title, st,
                     f"bundle x (prop/intake/edf) {total['prop']:.3f}/{total['intake']:.3f}/{total['edf']:.3f} mm^3 "
                     f"(sum {v_tot:.3f}); step out Z {steps['out'][0]:.1f}..{steps['out'][1]:.1f}, in Z {steps['in'][0]:.1f}.."
                     f"{steps['in'][1]:.1f} (in their windows: {win_ok}); gates below/above the lip "
                     f"{gate_lo:.2f}/{gate_hi:.2f} mm vs bundle {T:g} mm; closure missing max {max(holes.values()):.3f}, "
                     f"skin stray {stray:.3f} mm^3 (< {C20_CLOSED_TOL_MM3:g}); leg E -> band {band_gap:.2f} mm (> 0)",
                     f"per leg x (prop/intake/edf) mm^3: {leg_txt}; legs: A up the slot r {(L.edf_bore_r + L.sleeve_r) / 2:.2f}.. "
                     f"from Z {L.wire_slot_z0:.1f}; B steps out inside [{L.wire_gate_lo_z0:.1f}, {L.z_intake_skirt_top:.1f}]; C up the "
                     f"channel r {c_ch - T / 2:.2f}..{c_ch + T / 2:.2f} (channel {L.wire_ch_r0:.2f}..{L.wire_ch_r1:.2f}); D steps in "
                     f"through the shelf notch inside [{L.wire_notch_z0:.1f}, {L.wire_ch_z1:.1f}]; E up through the bulkhead hole "
                     f"r {L.wire_hole_r0:g}..{L.wire_hole_r1:g}; lower gate = pocket-wall corner (u {L.wire_ch_r0:.2f}, Z "
                     f"{L.z_intake_skirt_top:.1f}) to the floor wedge {gate_wedge:.3f} (triangle "
                     + ", ".join(f"({r:.2f}, {z:.2f})" for r, z in tri)
                     + f") / to the flat channel bottom at |v| {hv:g} (u {u_tube_in:.3f}, Z {L.wire_ch_z0:.2f}) {gate_flat:.3f}; "
                       f"upper gate = pocket-wall top corner (u {L.wire_ch_r0:.2f}, Z {L.wire_notch_z0:.1f}) to the band's "
                       f"bottom face over the notch (inner face u {u_band_in:.3f}, Z {L.wire_ch_z1:.1f}); leg E (up beside the "
                       f"strut) -> windowed band {band_gap:.3f} mm (BRep); channel closure missing material mm^3: "
                     + ", ".join(f"{k} {v:.3f}" for k, v in holes.items())
                     + f"; other openings in the intake's outer 0.3 mm (Z {L.z_intake_bottom:.1f}..{L.z_window_bottom:.1f}, "
                       f"joint holes and channel excluded) {stray:.3f} (< {C20_CLOSED_TOL_MM3:g})",
                     {"total": total, "per_leg": per_leg, "steps": steps, "steps_in_windows": win_ok,
                      "gate_below_lip": gate_lo, "gate_below_lip_wedge": gate_wedge, "gate_below_lip_flat": gate_flat,
                      "gate_above_lip": gate_hi, "bulge_holes": holes, "skin_stray": stray, "leg_e_band_gap_mm": band_gap})

    def _c21_clock(self, title: str, probe_set, z_kw1: float) -> Check:
        """wire09c clock mount: every keyway (centre + corners, C21_Z_STATIONS stations) keeps >= C21_MIN_SKIN_WALL of
        material to the outer surface and the skin annulus over it is intact; the blister walls (sides, outer, floor) are
        >= lead_passage.wall on the solid (ckc.blister_walls)."""
        L, prop = self.L, self.prop
        rows, kmin, worst_void = [], math.inf, 0.0
        for th in L.ck_keyways:
            r = probe_set(th, L.keyway_r_out, L.keyway_w / 2, L.keyway_z0, z_kw1, f"keyway {th:.2f}deg")
            m = min(r["centre"][0], r["corner"][0])
            if m < kmin:
                kmin = m
            half = math.degrees(math.atan2(L.keyway_w / 2, L.keyway_r_out)) + 1.0
            win = rg._sector(L.body_r - L.cone_wall, L.body_r, th - half, th + half, L.keyway_z0, L.z_prop_spigot_bottom)
            worst_void = max(worst_void, _void(win, prop))
            rows.append(f"{th:.2f}: {m:.3f}")
        bw = ckc.blister_walls(L, prop)
        bmin = min(bw.values())
        ok = (kmin >= C21_MIN_SKIN_WALL and worst_void < INTERSECT_TOL_MM3
              and bmin >= L.lp_wall - ckc.BLISTER_WALL_TOL)
        return Check("C21", "(clock mount) outer skin on the solid: every keyway (centre/corners) -> outer surface >= 0.8 "
                     "mm, skin over the keyways intact, blister walls >= lead_passage.wall", PASS if ok else FAIL,
                     f"{len(L.ck_keyways)} keyways -> outer surface min {kmin:.3f} mm >= {C21_MIN_SKIN_WALL}; skin void over the "
                     f"keyways max {worst_void:.3f} mm^3; blister walls min {bmin:.3f} mm >= {L.lp_wall:g} "
                     f"({min(bw, key=bw.get)})",
                     f"keyway Z {L.keyway_z0:.2f}..{z_kw1:.1f}; per keyway min: " + "; ".join(rows)
                     + "; blister walls: " + "; ".join(f"{k} {v:.3f}" for k, v in bw.items()),
                     {"keyway_min_mm": kmin, "skin_void_mm3": worst_void, "blister_walls_mm": bw})

    def c21_outer_skin(self) -> Check:
        title = "(tabs) outer skin on the solid: keyway (centre/corners) -> outer surface >= 0.8 mm, skin beside the wire slot >= skin_min_wall, lugs in the cylinder"
        L = self.L
        if not L.tabs:
            return self._na("C21", title)
        prop = self.prop
        n = C21_Z_STATIONS
        pad = C21_LOCAL_PAD

        def stations(z0: float, z1: float) -> list[float]:
            return [z0 + (z1 - z0) * (k + 0.5) / n for k in range(n)]

        def r_outer(z: float) -> float:
            """Outer surface of the propulsion module at z: the skin below the cone-shell top, the spigot ring above."""
            return L.cone_outer_radius(z) if z < L.z_prop_spigot_bottom else L.spigot_od / 2

        def outer_wall(local: cq.Shape, theta: float, z: float, r_start: float) -> tuple[float, float]:
            """(material from r_start out to the outer surface along the radial line at theta, start r of that
            outermost segment); 0 when the outermost segment does not reach the outer surface (breach)."""
            ro = r_outer(z)
            segs = rg.radial_material(local, theta, z, r_start - 0.5, ro + 1.0)
            if not segs or segs[-1][1] < ro - 0.01:
                return 0.0, math.nan
            a, b = segs[-1]
            return b - max(a, r_start), a

        def probe_set(theta: float, r_face: float, half_w: float, z0: float, z1: float, label: str) -> dict:
            """Radial probes at the cut's centre (r_start = r_face) and both corners (radial lines through
            (r_face, +-half_w), r_start = their radius), at the midpoints of n Z intervals of [z0, z1]."""
            corner_r = math.hypot(r_face, half_w)
            d_corner = math.degrees(math.atan2(half_w, r_face))
            local = prop.intersect(rg._box_polar(r_face - pad, L.body_r + pad, -half_w - pad, half_w + pad,
                                                 z0 - pad, z1 + pad, theta))
            res = {"centre": (math.inf, None), "corner": (math.inf, None), "gap": math.inf}
            for z in stations(z0, z1):
                for key, th_p, r0 in (("centre", theta, r_face), ("corner", theta - d_corner, corner_r),
                                      ("corner", theta + d_corner, corner_r)):
                    w, a = outer_wall(local, th_p, z, r0)
                    if w < res[key][0]:
                        res[key] = (w, z)
                    if key == "corner" and not math.isnan(a) and a > r0 + 1e-3:
                        res["gap"] = min(res["gap"], a - r0)   # corner -> skin, where the corner lies in a cavity
            res["label"] = label
            return res

        rows, keyway_min, worst_void = [], math.inf, 0.0
        z_kw1 = min(L.keyway_z1, L.z_prop_spigot_top)               # the keyway runs out through the spigot top
        if L.servo_leads:   # v0.6 (SPEC 3.8): above the rebate floor the keyway is open to the servo-lead rebate on purpose
            z_kw1 = min(z_kw1, L.sl_rebate_z0)
        if getattr(L, "clock_mount", False):
            return self._c21_clock(title, probe_set, z_kw1)
        for th in L.tab_thetas:
            r = probe_set(th, L.keyway_r_out, L.keyway_w / 2, L.keyway_z0, z_kw1, f"keyway {th:.0f}deg")
            keyway_min = min(keyway_min, r["centre"][0], r["corner"][0])
            rows.append(f"keyway {th:.0f}deg: min material to the outer surface centre {r['centre'][0]:.3f} (Z "
                        f"{r['centre'][1]:.1f}), corners {r['corner'][0]:.3f} (Z {r['corner'][1]:.1f})")
            # extra solid evidence: the skin annulus over the keyway window (Z keyway_z0..spigot bottom) is intact
            half = math.degrees(math.atan2(L.keyway_w / 2, L.keyway_r_out)) + 1.0
            win = rg._sector(L.body_r - L.cone_wall, L.body_r, th - half, th + half, L.keyway_z0, L.z_prop_spigot_bottom)
            worst_void = max(worst_void, _void(win, prop))
        ws = L.p["propulsion_module"]["wire_slot"]
        r = probe_set(L.wire_theta, L.wire_slot_lower_r_out, L.wire_slot_w / 2, L.wire_slot_z0, L.z_prop_spigot_bottom,
                      "wire slot")
        slot_min = min(r["centre"][0], r["corner"][0])
        # solid evidence: the skin sector across the whole slot width (plus 1 deg) is intact over the slot height
        half_s = math.degrees(math.atan2(L.wire_slot_w / 2, L.wire_slot_lower_r_out)) + 1.0
        slot_void = _void(rg._sector(L.body_r - L.cone_wall, L.body_r, L.wire_theta - half_s, L.wire_theta + half_s,
                                     L.wire_slot_z0, L.z_prop_spigot_bottom), prop)
        rows.append(f"wire slot {L.wire_theta:.0f}deg Z {L.wire_slot_z0:.1f}..{L.z_prop_spigot_bottom:.1f}: min skin/deck "
                    f"outside the slot centre {r['centre'][0]:.3f} (Z {r['centre'][1]:.1f}), corners {r['corner'][0]:.3f} "
                    f"(Z {r['corner'][1]:.1f}); below the deck the slot corners stop {r['gap']:.3f} mm short of the skin "
                    f"(design skin_min_wall {ws['skin_min_wall']:g})")
        low_min = math.inf
        if getattr(L, "ml_low_cut_uz", None):
            # v0.9 lead-exit round: the lead notch / lower slot below the 22 mm box (its vertical outer face, from the
            # chamfer top up to the 22 mm box) keeps the same skin rule
            (_, _), (_, _), (u_o, z_ct), (_, _), (_, _) = L.ml_low_cut_uz
            rl = probe_set(L.wire_theta, u_o, L.ml_low_cut_w / 2, z_ct, L.wire_slot_z0, "lead notch / lower slot")
            low_min = min(rl["centre"][0], rl["corner"][0])
            slot_min = min(slot_min, low_min)
            rows.append(f"lead notch / lower slot (2026-10-29) {L.wire_theta:.0f}deg Z {z_ct:.2f}..{L.wire_slot_z0:.1f}: min "
                        f"skin outside it centre {rl['centre'][0]:.3f} (Z {rl['centre'][1]:.1f}), corners {rl['corner'][0]:.3f} "
                        f"(Z {rl['corner'][1]:.1f}); corners stop {rl['gap']:.3f} mm short of the skin")
        # lugs: measured bboxes of the lug solids, inside the cylindrical section
        lug_z = [(s.BoundingBox().zmin, s.BoundingBox().zmax) for th in L.tab_thetas for s in rg.tab_screw_lugs(L, th)]
        lug_ok = (L.block_z0 >= L.z_cyl0 and all(z0 >= L.z_cyl0 - 1e-6 and z1 <= L.z_prop_spigot_bottom + 1e-6
                                                for z0, z1 in lug_z)
                  and all(abs(L.cone_outer_radius(z) - L.body_r) < 1e-9 for zz in lug_z for z in zz))
        ok = (keyway_min >= C21_MIN_SKIN_WALL and slot_min >= ws["skin_min_wall"] and worst_void < INTERSECT_TOL_MM3
              and slot_void < C21_SLOT_VOID_TOL_MM3 and lug_ok)
        return Check("C21", title, PASS if ok else FAIL,
                     f"keyway -> outer surface min {keyway_min:.3f} mm >= {C21_MIN_SKIN_WALL}; skin outside the wire slot min "
                     f"{slot_min:.3f} mm >= skin_min_wall {ws['skin_min_wall']:g}; skin void over the keyways "
                     f"{worst_void:.3f} mm^3, beside the wire slot {slot_void:.3f} mm^3; lugs Z {min(z for z, _ in lug_z):.2f}..{max(z for _, z in lug_z):.2f} in the "
                     f"cylinder (z_cyl0 {L.z_cyl0:.2f}) -> {lug_ok}",
                     f"radial probes (0.1 x 0.1 mm) at {n} Z stations, centre + both corners; keyway Z {L.keyway_z0:.2f}..{z_kw1:.1f} "
                     f"(outer surface = skin r {L.body_r:g} below Z {L.z_prop_spigot_bottom:.1f}, spigot ring r "
                     f"{L.spigot_od / 2:.2f} above), r_out {L.keyway_r_out:.2f}, corner r {math.hypot(L.keyway_r_out, L.keyway_w / 2):.3f}; "
                     f"wire slot r_out {L.wire_slot_lower_r_out:.3f}, corner r "
                     f"{math.hypot(L.wire_slot_lower_r_out, L.wire_slot_w / 2):.3f}; " + "; ".join(rows) +
                     f"; lug Z ranges {', '.join(f'{a:.2f}..{b:.2f}' for a, b in lug_z)}",
                     {"keyway_min": keyway_min, "slot_min": slot_min, "slot_corner_gap": r["gap"], "skin_void": worst_void,
                      "slot_skin_void": slot_void, "lead_notch_skin_min": low_min,
                      "lug_z": lug_z})

    # ---------------- v0.6 COM board (SPEC 5.3 / 6) ----------------
    def c22_com_board(self) -> Check:
        """SPEC C22 (v0.6): the COM-board placeholder hangs inside the avionics spigot bore and clears everything below
        it; it is seated on the payload bulkhead's bosses, which are solid around clear blind holes.  All measured on
        the solids: BRep distances (uncomputable -> 0 -> FAIL), intersection volumes (uncomputable -> inf), fill ratios
        of probes (uncomputable -> 0)."""
        title = ("(v0.6) COM board under the payload bulkhead: inside the spigot bore, clear of tray / battery / ESC / NAV / "
                 "misc / rails; seated on 4 solid bosses with clear blind holes")
        L = self.L
        if not L.has_com_board:
            return Check("C22", title, NA, "not applicable: params carry no electronics.com / payload_module.com_mount", "",
                         {"applicable": False})
        com, pay, av = self.shape["com"], self.shape["payload_module"], self.shape["avionics_module"]
        e = COM_PROBE_INSET
        mc = L.com_min_clear
        # (1) clearances: the parts below it >= min_clear_to_tray, the avionics module (spigot bore, shoulder, tube)
        # >= the placeholder bore clearance and its rail ribs >= min_clear_to_tray
        below = {n: _clearance(com, self.shape[n]) for n in ("avionics_tray", "battery", "esc", "nav", "misc_electronics")}
        x_r0, x_r1 = L.tray_x - L.rail_gap / 2 - L.rail_w, L.tray_x + L.rail_gap / 2 + L.rail_w
        y_in = L.rail_y - L.rail_depth
        rails = {}
        for sy, nm in ((+1.0, "+y"), (-1.0, "-y")):
            ya, yb = (y_in - e, L.body_r + 1.0) if sy > 0 else (-L.body_r - 1.0, -y_in + e)
            region = _box(x_r0 - e, x_r1 + e, ya, yb, L.z_rail_bottom - e, L.z_rail_top + e)
            try:
                rib = av.intersect(region)
                rails[nm] = _clearance(com, rib) if rib.Volume() > 1e-6 else 0.0   # no rib found: no evidence
            except Exception:  # noqa: BLE001
                rails[nm] = 0.0
        d_av = _clearance(com, av)
        r_max = max(math.hypot(v.X, v.Y) for v in com.Vertices())
        bore_meas = L.spigot_id / 2 - r_max
        clear_ok = (all(v >= mc - 1e-6 for v in below.values()) and all(v >= mc - 1e-6 for v in rails.values())
                    and d_av >= PLACEHOLDER_BORE_CLEAR_MM - 1e-6)
        # (2) seated on the bosses: no overlap with the payload module, and its lowest material over the board footprint
        # is the boss tip = the board's top face
        v_seat = _v(com, pay)
        over = _box(*L.com_board_x, *L.com_board_y, L.com_board_z[0] - 1.0, L.z_payload_bulkhead_top)
        z_low = _bottom_z(pay, over)
        seat_ok = v_seat < INTERSECT_TOL_MM3 and abs(z_low - L.com_board_z[1]) < COM_SEAT_TOL
        # (3) every boss: solid ring around the hole, hole clear up to its end, floor of bulkhead material above it
        boss_rows, ring_min, hole_max, floor_min = [], math.inf, 0.0, math.inf
        for x, y in L.com_boss_xy:
            ring = _off_axis_ring(x, y, L.com_hole_d / 2 + e, L.com_boss_d / 2 - e, L.z_com_boss_tip + e, L.z_com_boss_top - e)
            fill = _vpos(ring, pay) / ring.Volume()
            hole = rg._cyl_z(L.com_hole_d / 2 - e, L.z_com_boss_tip - 0.5, L.z_com_hole_end - e, x, y)
            v_hole = _v(hole, pay)
            if L.com_hole_skin > 2 * e:
                fl = rg._cyl_z(L.com_hole_d / 2, L.z_com_hole_end + e, L.z_payload_bulkhead_top - e, x, y)
                f_fill = _vpos(fl, pay) / fl.Volume()
            else:
                f_fill = 0.0                               # no measurable floor: not blind
            ring_min, hole_max, floor_min = min(ring_min, fill), max(hole_max, v_hole), min(floor_min, f_fill)
            boss_rows.append(f"({x:+.0f},{y:+.0f}) ring {fill:.4f} hole {v_hole:.4f} floor {f_fill:.4f}")
        boss_ok = (len(L.com_boss_xy) == 4 and ring_min >= COM_FILL_MIN and hole_max < COM_HOLE_CLEAR_TOL_MM3
                   and floor_min >= COM_FILL_MIN)
        # (4) the bulkhead holes stay open from the board's bottom up through the bulkhead (no boss, no board in them)
        col_max = 0.0
        for th in L.bulkhead_hole_thetas:
            c = rg._polar(L.bulkhead_hole_r, 0.0, th, 0.0)
            col = rg._cyl_z(L.bulkhead_hole_d / 2 - e, L.com_board_z[0], L.z_payload_bulkhead_top + e, c.x, c.y)
            col_max = max(col_max, _v(col, pay), _v(col, com))
        holes_ok = col_max < COM_HOLE_CLEAR_TOL_MM3
        ok = clear_ok and seat_ok and boss_ok and holes_ok
        return Check("C22", title, PASS if ok else FAIL,
                     "clearance " + ", ".join(f"{k} {v:.2f}" for k, v in below.items())
                     + f" (>= {mc:g}); rails " + "/".join(f"{v:.2f}" for v in rails.values())
                     + f" (>= {mc:g}); avionics_module {d_av:.2f} (>= {PLACEHOLDER_BORE_CLEAR_MM:g}), radial to spigot ID {bore_meas:.2f}; "
                       f"seat: com x payload {v_seat:.3f} mm^3, lowest payload material over the board z {z_low:.3f} vs board "
                       f"top {L.com_board_z[1]:.3f}; bosses: ring fill min {ring_min:.4f} (>= {COM_FILL_MIN:g}), hole probe max "
                       f"{hole_max:.4f} mm^3 (< {COM_HOLE_CLEAR_TOL_MM3:g}), floor fill min {floor_min:.4f}; bulkhead hole "
                       f"columns max {col_max:.4f} mm^3",
                     f"board x {L.com_board_x[0]:g}..{L.com_board_x[1]:g} y {L.com_board_y[0]:g}..{L.com_board_y[1]:g} z "
                     f"{L.com_board_z[0]:.2f}..{L.com_board_z[1]:.2f}; bosses Ø{L.com_boss_d:g} x {L.com_boss_h:g} at "
                     f"+-{L.com_pattern / 2:g}, blind Ø{L.com_hole_d:g} x {L.com_hole_depth:g} (end z {L.z_com_hole_end:.2f}, "
                     f"skin {L.com_hole_skin:.2f}); probes inset {e:g} mm; " + "; ".join(boss_rows)
                     + f"; bulkhead holes Ø{L.bulkhead_hole_d:g} r {L.bulkhead_hole_r:g} at "
                     + ", ".join(f"{t:g}" for t in L.bulkhead_hole_thetas) + " deg, columns from the board bottom up; "
                     f"Layout box clearances: " + ", ".join(f"{k} {v:.2f}" for k, v in L.com_clearance.items()),
                     {"below": below, "rails": rails, "avionics_module": d_av, "bore_radial": bore_meas, "seat_volume": v_seat,
                      "seat_z": z_low, "ring_fill_min": ring_min, "hole_probe_max": hole_max, "floor_fill_min": floor_min,
                      "bulkhead_hole_columns_max": col_max})

    # ---------------- v0.6.2 radio / logger box (electronics.radio_log) ----------------
    def c24_radio_log(self) -> Check:
        """v0.6.2 C24: the radio / logger box (LoRa + antenna, OpenLog, 2nd level shifter, 2nd LDO) hangs from the payload
        bulkhead's underside inside the avionics spigot bore, beside the COM board.  All on the solids: BRep clearances
        (uncomputable -> 0 -> FAIL) >= payload_module.com_mount.min_clear_to_tray to the tray / battery / ESC / NAV / misc
        / COM board / rail ribs / payload material below the bulkhead (COM bosses, skirt) / every joint-screw axis proxy
        (Ø joint_hole_outer_d from the vehicle axis to body_r: a screw of any length), >= PLACEHOLDER_BORE_CLEAR_MM to
        the avionics module; seated (no overlap with the payload module, whose lowest material over the box footprint is
        the bulkhead underside = the box top); the payload bulkhead holes stay open past it.  N/A without the box.
        A separate row (not C22): it is gated on its own params key, and C22's COM-board evidence stays as it was."""
        title = ("(v0.6.2) radio/log box under the payload bulkhead: inside the spigot bore, clear of COM board / bosses / "
                 "tray / battery / ESC / NAV / misc / rails / joint screws, hanging from the bulkhead, bulkhead holes open")
        L = self.L
        if not L.has_radio_log or "radio_log" not in self.shape:
            return Check("C24", title, NA, "not applicable: params carry no electronics.radio_log", "", {"applicable": False})
        rl, pay, av = self.shape["radio_log"], self.shape["payload_module"], self.shape["avionics_module"]
        e, mc = COM_PROBE_INSET, L.radio_log_min_clear
        near = {n: _clearance(rl, self.shape[n]) for n in ("avionics_tray", "battery", "esc", "nav", "misc_electronics", "com")
                if n in self.shape}
        x_r0, x_r1 = L.tray_x - L.rail_gap / 2 - L.rail_w, L.tray_x + L.rail_gap / 2 + L.rail_w
        y_in = L.rail_y - L.rail_depth
        for sy, nm in ((+1.0, "rails +y"), (-1.0, "rails -y")):
            ya, yb = (y_in - e, L.body_r + 1.0) if sy > 0 else (-L.body_r - 1.0, -y_in + e)
            try:
                rib = av.intersect(_box(x_r0 - e, x_r1 + e, ya, yb, L.z_rail_bottom - e, L.z_rail_top + e))
                near[nm] = _clearance(rl, rib) if rib.Volume() > 1e-6 else 0.0      # no rib found: no evidence
            except Exception:  # noqa: BLE001
                near[nm] = 0.0
        # payload material below the bulkhead underside (COM bosses, the skirt over the avionics spigot)
        try:
            low = pay.intersect(_box(-L.body_r - 1.0, L.body_r + 1.0, -L.body_r - 1.0, L.body_r + 1.0,
                                     L.radio_log_z[0] - 1.0, L.z_payload_bulkhead_bottom - e))
            near["payload below the bulkhead"] = _clearance(rl, low) if low.Volume() > 1e-6 else 0.0
        except Exception:  # noqa: BLE001
            near["payload below the bulkhead"] = 0.0
        near["joint screw axes"] = min(_clearance(rl, s) for s in rg.joint_screw_axis_proxies(L))
        d_av = _clearance(rl, av)
        bore = L.spigot_id / 2 - max(math.hypot(v.X, v.Y) for v in rl.Vertices())
        clear_ok = all(v >= mc - 1e-6 for v in near.values()) and d_av >= PLACEHOLDER_BORE_CLEAR_MM - 1e-6
        # seated: measured on the box SOLID - the lowest payload material over its footprint is its top face
        v_seat = _v(rl, pay)
        bb = rl.BoundingBox()
        z_low = _bottom_z(pay, _box(bb.xmin, bb.xmax, bb.ymin, bb.ymax, bb.zmin - 1.0, L.z_payload_bulkhead_top))
        seat_ok = (v_seat < INTERSECT_TOL_MM3 and abs(z_low - bb.zmax) < COM_SEAT_TOL
                   and abs(bb.zmax - L.radio_log_z[1]) < COM_SEAT_TOL)
        col_max = 0.0
        for th in L.bulkhead_hole_thetas:
            c = rg._polar(L.bulkhead_hole_r, 0.0, th, 0.0)
            col = rg._cyl_z(L.bulkhead_hole_d / 2 - e, L.radio_log_z[0], L.z_payload_bulkhead_top + e, c.x, c.y)
            col_max = max(col_max, _v(col, rl), _v(col, pay))
        holes_ok = col_max < COM_HOLE_CLEAR_TOL_MM3
        ok = clear_ok and seat_ok and holes_ok
        return Check("C24", title, PASS if ok else FAIL,
                     "clearance " + ", ".join(f"{k} {v:.2f}" for k, v in near.items())
                     + f" (>= {mc:g}); avionics_module {d_av:.2f} (>= {PLACEHOLDER_BORE_CLEAR_MM:g}), radial to spigot ID "
                       f"{bore:.2f}; seat: radio_log x payload {v_seat:.3f} mm^3, lowest payload material over the box z "
                       f"{z_low:.3f} vs box top {bb.zmax:.3f} (Layout {L.radio_log_z[1]:.3f}); bulkhead hole columns max "
                       f"{col_max:.4f} mm^3",
                     f"box x {L.radio_log_x[0]:.2f}..{L.radio_log_x[1]:.2f} y {L.radio_log_y[0]:.2f}..{L.radio_log_y[1]:.2f} z "
                     f"{L.radio_log_z[0]:.2f}..{L.radio_log_z[1]:.2f} ({L.p['electronics']['radio_log']['mass_g']:g} g, centre r "
                     f"{L.radio_log_r_center:g} at {L.radio_log_theta_deg:g} deg); joint-screw proxies Ø"
                     f"{L.p['body']['joint_hole_outer_d']:g} from the axis to body_r at "
                     + ", ".join(f"{a[2]:.1f}" for _, a, _ in L.joint_screw_axes()[::len(L.joint_thetas)])
                     + f"; probes inset {e:g} mm; Layout box clearances: "
                     + ", ".join(f"{k} {v:.2f}" for k, v in L.radio_log_clearance.items()),
                     {"clearance": near, "avionics_module": d_av, "bore_radial": bore, "seat_volume": v_seat, "seat_z": z_low,
                      "bulkhead_hole_columns_max": col_max, "min_clear": mc})

    # ---------------- v0.6.3 FCU board (electronics.fcu) ----------------
    def c25_fcu_board(self) -> Check:
        """v0.6.3 C25: the FCU board lies flat on the intake bulkhead's top face on foam tape, inside the intake spigot
        bore, below the avionics tray.  All on the solids:

        * clearances (BRep, uncomputable -> 0 -> FAIL) >= payload_module.com_mount.min_clear_to_tray to the tray, the tray
          placeholders, the rail ribs + tray stops (avionics_module material in the rail-slot region, each side) and the
          lead bundles that rise through the bulkhead wire / servo holes (rocket_geometry.lead_riser_proxies: C20's and
          C23's leg E continued up to the tray bottom); >= PLACEHOLDER_BORE_CLEAR_MM to the avionics module and to the
          intake spigot (intake material above the bulkhead), and radially to the spigot ID;
        * seated: no overlap with the intake module; the highest intake material under the footprint is the bulkhead top
          face, foam_t below the board; the bulkhead under the footprint is >= FCU_SEAT_FILL_MIN solid (and equals
          Layout.fcu_seat_fraction);
        * the motor-lead and servo-lead holes are open through the bulkhead and free of the board up to the tray bottom
          (columns inset COM_PROBE_INSET from the hole walls);
        * a vent hole is reachable for the cable tie when it is open through the bulkhead and free of the board up to the
          board's top; at least one pair of reachable holes has its tie run across the board (a FCU_TIE_PROBE_D rod between
          the hole centres at the board's mid height meets the board);
        * the centre cable hole is reported (covered by the board or not).

        N/A without electronics.fcu.  A separate row, gated on its own params key like C24."""
        title = ("(v0.6.3) FCU board on the intake bulkhead: inside the spigot bore, clear of tray / stops / rails / ESC / "
                 "misc / rising lead bundles, seated on the foam tape, lead holes uncovered, a cable-tie pair of vent holes open")
        L = self.L
        fcu_n = getattr(L, "fcu_part", "fcu")      # v0.9 CAD-2 VTOL: the carrier PCB takes the FCU spot and rules
        if getattr(L, "vtol", False):
            title = "(v0.9 CAD-2) carrier PCB on the intake bulkhead (the FCU rules)" + title[len("(v0.6.3) FCU board on the intake bulkhead"):]
        if not getattr(L, "has_fcu", False) or fcu_n not in self.shape:
            return Check("C25", title, NA, "not applicable: params carry no electronics.fcu", "", {"applicable": False})
        fcu, intake, av = self.shape[fcu_n], self.shape["intake_module"], self.shape["avionics_module"]
        e, mc, big = COM_PROBE_INSET, L.fcu_min_clear, L.body_r + 5.0
        # (1) clearances
        near = {n: _clearance(fcu, self.shape[n]) for n in ("avionics_tray", "battery", "buffer_pack", "esc", "nav",
                                                            "misc_electronics") if n in self.shape}
        x_r0, x_r1 = L.tray_x - L.rail_gap / 2 - L.rail_w, L.tray_x + L.rail_gap / 2 + L.rail_w
        y_in = L.rail_y - L.rail_depth
        z_lo = min(L.z_rail_bottom, L.z_tray_stop_bottom if L.tray_stop_h else L.z_rail_bottom)
        for sy, nm in ((+1.0, "rails + tray stop +y"), (-1.0, "rails + tray stop -y")):
            ya, yb = (y_in - e, L.body_r + 1.0) if sy > 0 else (-L.body_r - 1.0, -y_in + e)
            try:
                rib = av.intersect(_box(x_r0 - e, x_r1 + e, ya, yb, z_lo - e, L.z_rail_top + e))
                near[nm] = _clearance(fcu, rib) if rib.Volume() > 1e-6 else 0.0      # no rib found: no evidence
            except Exception:  # noqa: BLE001
                near[nm] = 0.0
        risers = rg.lead_riser_proxies(L, L.z_tray_bottom)
        for nm, shp in risers.items():
            near[nm] = _clearance(fcu, shp)
        d_av = _clearance(fcu, av)
        try:
            spig = intake.intersect(_box(-big, big, -big, big, L.z_intake_bulkhead_top + e, L.z_intake_top + 1.0))
            d_spig = _clearance(fcu, spig) if spig.Volume() > 1e-6 else 0.0          # no spigot found: no evidence
        except Exception:  # noqa: BLE001
            d_spig = 0.0
        bore = min(L.spigot_id / 2, L.body_id / 2) - max(math.hypot(v.X, v.Y) for v in fcu.Vertices())
        clear_ok = (all(v >= mc - 1e-6 for v in near.values()) and min(d_av, d_spig) >= PLACEHOLDER_BORE_CLEAR_MM - 1e-6
                    and bore >= PLACEHOLDER_BORE_CLEAR_MM - 1e-6)
        # (2) seated on the foam tape on the bulkhead top
        v_seat = _v(fcu, intake)
        bb = fcu.BoundingBox()
        z_under = _top_z(intake, _box(bb.xmin, bb.xmax, bb.ymin, bb.ymax, L.z_intake_bulkhead_bottom - 1.0, bb.zmin))
        foam = bb.zmin - z_under
        slab = _box(bb.xmin, bb.xmax, bb.ymin, bb.ymax, L.z_intake_bulkhead_top - FLOOR_AREA_SLAB, L.z_intake_bulkhead_top)
        fill = _vpos(slab, intake) / slab.Volume()
        fill_match = (not L.fcu_seat_area_exact) or abs(fill - L.fcu_seat_fraction) < FCU_SEAT_FILL_TOL
        seat_ok = (v_seat < INTERSECT_TOL_MM3 and abs(z_under - L.z_intake_bulkhead_top) < COM_SEAT_TOL
                   and abs(foam - L.fcu_foam_t) < COM_SEAT_TOL and fill >= FCU_SEAT_FILL_MIN and fill_match)
        # (3) lead holes open through the bulkhead and free of the board up to the tray bottom; vent holes up to its top
        z_h0 = L.z_intake_bulkhead_bottom + e
        cols = rg.bulkhead_hole_columns(L, z_h0, L.z_tray_bottom - e, e)
        lead = {nm: max(_v(c, intake), _v(c, fcu)) for nm, c in cols.items() if nm in ("motor-lead hole", "servo-lead hole")}
        leads_ok = all(v < COM_HOLE_CLEAR_TOL_MM3 for v in lead.values())
        vent_cols = rg.bulkhead_hole_columns(L, z_h0, bb.zmax + e, e)
        vents = {th: {"intake": _v(vent_cols[f"vent hole {th:g} deg"], intake), "fcu": _v(vent_cols[f"vent hole {th:g} deg"], fcu)}
                 for th in L.vent_hole_thetas}
        reachable = [th for th, v in vents.items() if v["intake"] < COM_HOLE_CLEAR_TOL_MM3 and v["fcu"] < COM_HOLE_CLEAR_TOL_MM3]
        z_mid = 0.5 * (bb.zmin + bb.zmax)
        ties = []
        for i, a in enumerate(reachable):
            for b in reachable[i + 1:]:
                pa, pb = rg._polar(L.vent_hole_r, 0.0, a, z_mid), rg._polar(L.vent_hole_r, 0.0, b, z_mid)
                rod = cq.Solid.makeCylinder(FCU_TIE_PROBE_D / 2, (pb - pa).Length, pa, (pb - pa).normalized())
                if _vpos(rod, fcu) > 0.0:
                    ties.append((a, b))
        tie_ok = len(reachable) >= 2 and bool(ties)
        cable_on_board = _vpos(rg._cyl_z(L.cable_hole_d / 2 - e, bb.zmin + e, bb.zmax - e), fcu)
        cable_open = _v(cols["cable hole"].intersect(_box(-big, big, -big, big, z_h0, L.z_intake_bulkhead_top - e)), intake)
        ok = clear_ok and seat_ok and leads_ok and tie_ok
        return Check("C25", title, PASS if ok else FAIL,
                     "clearance " + ", ".join(f"{k} {v:.2f}" for k, v in near.items())
                     + f" (>= {mc:g}); avionics_module {d_av:.2f}, intake spigot {d_spig:.2f} (>= {PLACEHOLDER_BORE_CLEAR_MM:g}), "
                       f"radial to spigot ID {bore:.2f}; seat: fcu x intake {v_seat:.3f} mm^3, bulkhead top under the board z "
                       f"{z_under:.3f} (Layout {L.z_intake_bulkhead_top:.3f}), foam gap {foam:.3f} (foam_t {L.fcu_foam_t:g}), "
                       f"bulkhead under the footprint {fill:.4f} solid (>= {FCU_SEAT_FILL_MIN:g}; Layout {L.fcu_seat_fraction:.4f}); "
                       "lead-hole columns " + (", ".join(f"{k} {v:.4f}" for k, v in lead.items()) + " mm^3" if lead else
                                               "none (no lead holes in this preset)")
                     + f"; vent holes reachable {', '.join(f'{t:g}' for t in reachable) or 'none'} deg, tie runs across "
                       f"the board " + (", ".join(f"{a:g}/{b:g}" for a, b in ties) or "none")
                     + f"; cable hole {'covered by the board' if cable_on_board > 1.0 else 'not covered'}",
                     f"board x {L.fcu_x[0]:.2f}..{L.fcu_x[1]:.2f} y {L.fcu_y[0]:.2f}..{L.fcu_y[1]:.2f} z {L.fcu_z[0]:.2f}.."
                     f"{L.fcu_z[1]:.2f} ({L.p['electronics']['fcu']['mass_g']:g} g) on {L.fcu_foam_t:g} mm foam on the intake "
                     f"bulkhead (top z {L.z_intake_bulkhead_top:.2f}); rising bundles = C20 / C23 leg E footprints from the "
                     f"bulkhead up to the tray bottom z {L.z_tray_bottom:.2f}; probes inset {e:g} mm; vent holes Ø{L.vent_hole_d:g} "
                     f"at r {L.vent_hole_r:g}: " + "; ".join(f"{t:g} deg x intake {v['intake']:.4f} x fcu {v['fcu']:.4f}"
                                                            for t, v in vents.items())
                     + f"; cable hole Ø{L.cable_hole_d:g}: board over it {cable_on_board:.1f} mm^3, open through the bulkhead "
                       f"{cable_open:.4f} mm^3 of material; Layout box clearances: "
                     + ", ".join(f"{k} {v:.2f}" for k, v in L.fcu_clearance.items()),
                     {"clearance": near, "avionics_module": d_av, "intake_spigot": d_spig, "bore_radial": bore,
                      "seat_volume": v_seat, "seat_z": z_under, "foam_gap": foam, "seat_fill": fill, "lead_hole_columns": lead,
                      "vent_holes": {f"{t:g}": v for t, v in vents.items()}, "vent_reachable": reachable, "tie_pairs": ties,
                      "cable_hole_covered": cable_on_board > 1.0, "min_clear": mc})

    def c23_servo_leads(self) -> Check:
        """SPEC 7 C23 (v0.6, SPEC 3.8 / 5.1): ribbon proxies of every lead and of the gathered bundle clear the modules
        and the EDF (the outside run also the servos, arms, pushrods and horns); the servo bulge is closed, the deck
        outside every notch (the intake skirt's seat) and the sleeve at every column are solid.  N/A without the route."""
        title = ("(tabs) vane-servo leads: pod window -> outside -> entry slot -> straight column -> deck notch -> rebate "
                 "-> servo channel past the lip -> shelf notch -> bulkhead hole; proxies clear, bulge closed, deck seat and "
                 "sleeve intact")
        L = self.L
        if not L.servo_leads:
            return self._na("C23", title)
        mods = {"prop": self.prop, "intake": self.shape["intake_module"], "edf": self.shape["edf_unit"]}
        linkage = {n: s for n, s in self.shape.items() if n.startswith(("servo_vane_", "servo_arm_", "pushrod_", "vane_horn_",
                                                                         "servo_horn_", "vane_coupler_", "servo_cover_"))}
        near = ("K", "P") if L.direct_drive else ("W", "O", "X")
        worst, rows = (0.0, ""), []
        pipe_void = 0.0
        for k in range(L.vane_count):
            per = {}
            for nm, leg in rg.servo_lead_legs(L, k).items():
                hits = {m: _v(leg, s) for m, s in mods.items()}
                if nm in near:      # the outside run / the pocket run also clears the servos, arms, pushrods, horns, couplers
                    hits.update({m: _v(leg, s) for m, s in linkage.items()})
                m_worst = max(hits, key=hits.get)
                per[nm] = hits[m_worst]
                if hits[m_worst] > worst[0]:
                    worst = (hits[m_worst], f"lead {k + 1} leg {nm} x {m_worst}")
            if L.direct_drive:      # v0.9: the lead pipe's wall is solid around the middle half of the tunnel
                a, b = L.sl_pipe_points(k)
                va, vb = cq.Vector(*a), cq.Vector(*b)
                m0, m1 = (va + (vb - va) * 0.25).toTuple(), (va + (vb - va) * 0.75).toTuple()
                ring = rg._cyl_between(L.sl_pipe_d - 2 * C23_PROBE_INSET, m0, m1).cut(
                    rg._cyl_between(L.sl_tunnel_d + 2 * C23_PROBE_INSET, m0, m1, ext0=1.0, ext1=1.0))
                pipe_void = max(pipe_void, _void(ring, self.prop))
            rows.append(f"lead {k + 1} ({'fin' if L.direct_drive else 'pod'} "
                        f"{(L.fin_thetas if L.direct_drive else L.pod_thetas)[k]:g}, column {L.sl_col_thetas[k]:g} deg): "
                        + "/".join(f"{nm} {v:.3f}" for nm, v in per.items()))
        bundle = {}
        bundle_legs = rg.servo_bundle_legs(L)
        for nm, leg in bundle_legs.items():
            hits = {m: _v(leg, s) for m, s in mods.items()}
            bundle[nm] = max(hits.values())
            if bundle[nm] > worst[0]:
                worst = (bundle[nm], f"bundle leg {nm} x {max(hits, key=hits.get)}")
        # v0.6.1: the bundle's leg E runs up beside the strut inside the thicker band: measured BRep gap >= the route's
        # min_clearance (Layout.band_clear_servo_bundle is the same number from the stations)
        band_gap = _clearance(bundle_legs["E"], self._intake_band_part())
        band_ok = band_gap >= L.p["servo_leads"]["min_clearance"] - 1e-6
        # solid evidence: servo bulge closed (outer wall, side walls, floor wedge, roof), the deck outside every notch,
        # the sleeve at every column
        intake, e = mods["intake"], C23_PROBE_INSET
        tg, hw, wall = L.sl_gather_theta, L.sl_ch_w / 2, L.p["intake_module"]["wire_channel"]["wall"]
        closure = {
            "outer wall": rg._box_polar(L.sl_ch_r1, L.sl_bulge_r_out, -hw, hw, L.sl_ch_z0, L.sl_ch_z1, tg),
            "side walls": rg._box_polar(L.body_r, L.sl_ch_r1, hw, hw + wall, L.sl_ch_z0, L.sl_ch_z1, tg).fuse(
                rg._box_polar(L.body_r, L.sl_ch_r1, -hw - wall, -hw, L.sl_ch_z0, L.sl_ch_z1, tg)),
            "floor wedge": rg.intake_servo_floor_wedge(L),
            "roof": rg._box_polar(L.body_r, L.sl_bulge_r_out, -hw - wall, hw + wall, L.sl_ch_z1, L.sl_bulge_z1, tg),
        }
        holes = {k: _void(b, intake) for k, b in closure.items()}
        z_deck, so = L.z_prop_spigot_bottom - L.body_wall, L.spigot_od / 2
        seat, sleeve = 0.0, 0.0
        for th in L.sl_col_thetas:
            half_n = math.degrees(math.atan2(L.sl_notch_w / 2, L.sl_notch_u0)) + 1.0
            seat = max(seat, _void(rg._sector(math.hypot(so, L.sl_notch_w / 2) + e, L.body_r - e, th - half_n, th + half_n,
                                              z_deck + e, L.z_prop_spigot_bottom - e), self.prop))
            half_e = math.degrees(math.atan2(L.sl_entry_w / 2, L.sl_entry_u0)) + 1.0
            sleeve = max(sleeve, _void(rg._sector(L.edf_bore_r + e, L.sleeve_r - e, th - half_e, th + half_e,
                                                  L.z_sleeve_bottom + e, z_deck - e), self.prop))
        closed_ok = (max(holes.values()) < C23_CLOSED_TOL_MM3 and seat < C23_CLOSED_TOL_MM3 and sleeve < C23_CLOSED_TOL_MM3
                     and pipe_void < C23_CLOSED_TOL_MM3)
        st = _status_no_intersection(worst[0]) if closed_ok and band_ok else FAIL
        W, T = L.sl_ribbon_w, L.sl_ribbon_t
        if L.direct_drive:
            title = ("(tabs, v0.9 direct drive) vane-servo leads: connector bay in the fairing -> pocket -> closed lead pipe on "
                     "the duct -> entry slot -> straight column -> deck notch -> rebate -> servo channel past the lip -> shelf "
                     "notch -> bulkhead hole; proxies clear, pipe wall / bulge closed, deck seat and sleeve intact")
        return Check("C23", title, st,
                     f"max proxy x (prop/intake/edf; outside run also linkage) {worst[0]:.3f} mm^3 ({worst[1]}); bundle legs "
                     + "/".join(f"{k} {v:.3f}" for k, v in bundle.items())
                     + f"; bulge missing max {max(holes.values()):.3f}, deck seat void {seat:.3f}, sleeve void {sleeve:.3f}"
                     + (f", lead-pipe wall void {pipe_void:.3f}" if L.direct_drive else "")
                     + f" mm^3 (< {C23_CLOSED_TOL_MM3:g}); bundle leg E -> band {band_gap:.2f} mm (>= "
                       f"{L.p['servo_leads']['min_clearance']:g}); leads {L.sl_lead_mass_g:.1f} g at z {L.sl_lead_com[2]:.1f}",
                     f"ribbon {W:g} x {T:g} mm (ASSUMED), bundle 2 x 2 = {2 * W:g} x {2 * T:g}, proxies {rg.SERVO_PROXY_GAP:g} mm "
                     f"off the walls; columns {', '.join(f'{t:g}' for t in L.sl_col_thetas)} deg; entry slot r "
                     f"{L.sl_entry_u0:.2f}..{L.sl_entry_u1:.2f} open into the annulus above Z {L.sl_entry_z_open:.2f}; "
                     + (f"lead pipe O{L.sl_pipe_d:g} (tunnel O{L.sl_tunnel_d:g}) from the pocket's inner top corner to the slot "
                        f"bottom; " if L.direct_drive else f"crossing Z {L.sl_cross_z0:.2f}..{L.sl_cross_z1:.2f}; ")
                     + f"deck notch r {L.sl_notch_u0:.2f}..{so:.2f}; rebate r "
                     f"{L.sl_rebate_r0:.2f}.., Z {L.sl_rebate_z0:.1f}.., theta {L.sl_rebate_th0:.2f}..{L.sl_rebate_th1:.2f}; channel "
                     f"r {L.sl_ch_r0:.2f}..{L.sl_ch_r1:.2f} at {tg:g} deg, Z {L.sl_ch_z0:.2f}..{L.sl_ch_z1:.2f}; bulkhead hole r "
                     f"{L.sl_hole_r0:g}..{L.sl_hole_r1:g}; per lead (W window, O outside, X crossing, S slot, C column, N notch) "
                     "mm^3: " + "; ".join(rows)
                     + "; bulge closure missing mm^3: " + ", ".join(f"{k} {v:.3f}" for k, v in holes.items())
                     + "; route length per lead (to the " + ("FCU" if L.has_fcu else "flight computer")
                     + ", bay legs estimated) " + ", ".join(f"{x:.0f}" for x in L.sl_route_len)
                     + " mm, extension " + ", ".join(f"{x:.0f}" for x in L.sl_ext_len) + " mm",
                     {"worst": worst, "bundle": bundle, "bulge_holes": holes, "deck_seat_void": seat, "sleeve_void": sleeve,
                      "lead_mass_g": L.sl_lead_mass_g, "lead_com_mm": list(L.sl_lead_com), "leg_e_band_gap_mm": band_gap,
                      "route_len_mm": list(L.sl_route_len), "extension_len_mm": list(L.sl_ext_len)})

    # ---------------- v0.8 GPS pod (payload_module.gps_pod, electronics.gps) ----------------
    def c26_gps_pod(self) -> Check:
        """v0.8 C26: the GPS pod on the payload tube, its lid, the antenna stack, the raceway / connector pocket and the
        avionics entry hole, all on the solids (uncomputable -> FAIL):

        * nose: the highest pod / lid material is gap_below_nose_skirt under the nose-separation edge; lid x nose = 0;
        * lid: lid x payload = 0, it rests on the rim (highest pod material under its footprint = its underside), its posts
          reach the ground plane (lowest lid material over each post = the plate top);
        * antenna: patch -> lid = patch_air_gap; nothing but the lid above the patch (payload / nose material over the
          patch footprint = 0, lid material there = footprint x lid t within 1 %);
        * stack: gps / gps_ground_plane x payload / lid = 0, BRep clearance to the pod >= cavity_clear;
        * lid-screw bosses: each blind hole clear (probe inset GPS_PROBE_INSET) with material around it and under its end;
        * raceway: a cable probe from the pocket's open bottom up through the raceway into the cavity meets no material, the
          raceway walls are solid, and the payload bore over the pod is clear (the piston passes);
        * avionics entry hole: a probe through the wall at the hole meets no material and the hole is under the pocket;
        * latch arm (latched / released) and release servo clear the lid.
        Noise-source distances and the antenna lever arm are reported.  N/A without payload_module.gps_pod."""
        title = ("(v0.8) GPS pod: under the nose edge, lid seated with the air gap, open sky over the patch, stack clear in "
                 "the cavity, blind lid-screw holes, raceway through to the pocket, entry hole open, bore clear")
        L = self.L
        if not getattr(L, "has_gps_pod", False) or "gps" not in self.shape:
            return Check("C26", title, NA, "not applicable: params carry no payload_module.gps_pod", "", {"applicable": False})
        pay, lid, nose = self.shape["payload_module"], self.shape["gps_pod_lid"], self.shape["nose_cone"]
        gps, gp, av = self.shape["gps"], self.shape["gps_ground_plane"], self.shape["avionics_module"]
        e, th = GPS_PROBE_INSET, L.gps_theta
        g = L.p["payload_module"]["gps_pod"]
        v: dict[str, float] = {}
        ok: dict[str, bool] = {}
        # (1) nose edge
        pod_region = rg._box_polar(L.body_r + 0.1, L.gps_u_out + 2.0, -L.gps_w_h - 2.0, L.gps_w_h + 2.0,
                                   L.gps_z_foot - 2.0, L.z_nose_tip, th)
        v["pod top z"] = _top_z(pay, pod_region)
        v["lid top z"] = lid.BoundingBox().zmax
        z_lim = L.z_payload_spigot_bottom - g["lid"]["gap_below_nose_skirt"]
        v["lid x nose"], v["payload x nose"] = _v(lid, nose), _v(pay, nose)
        ok["nose edge"] = (abs(v["lid top z"] - z_lim) < COM_SEAT_TOL and v["pod top z"] <= z_lim + 1e-6
                           and max(v["lid x nose"], v["payload x nose"]) < INTERSECT_TOL_MM3)
        # (2) lid seated on the rim, posts on the plate
        v["lid x payload"] = _v(lid, pay)
        # under the lid's footprint (outside its inner arc r = gps_lid_r_in: the tube itself runs on up to the spigot)
        under_lid = rg._box_polar(L.gps_lid_u_back, L.gps_u_out, -L.gps_w_h, L.gps_w_h, L.gps_z_rim - 2.0,
                                  L.gps_z_lid_top + 1.0, th).cut(rg._cyl_z(L.gps_lid_r_in, L.gps_z_rim - 3.0, L.gps_z_lid_top + 2.0))
        v["pod material under the lid, top z"] = _top_z(pay, under_lid)
        post_low = []
        for u, w in L.gps_post_uv:
            x, y, _ = L.gps_xyz(u, w, 0.0)
            post_low.append(_bottom_z(lid, rg._cyl_z(L.gps_post_d / 2 - e, L.gps_z_gp_top - 2.0, L.gps_z_rim, x, y)))
        v["lowest lid post z (max over posts)"] = max(post_low)
        v["lowest lid post z (min over posts)"] = min(post_low)
        ok["lid seat"] = (v["lid x payload"] < INTERSECT_TOL_MM3 and abs(v["pod material under the lid, top z"] - L.gps_z_rim) < COM_SEAT_TOL
                          and all(abs(z - L.gps_z_gp_top) < COM_SEAT_TOL for z in post_low))
        # (3) antenna: air gap and open sky over the patch
        (pu, pv, pz) = L.gps_patch_box
        column = rg._box_polar(pu[0], pu[1], pv[0], pv[1], pz[1] + e, L.z_nose_tip + 10.0, th)
        v["patch -> lid (BRep)"] = _clearance(gps, lid)
        v["payload over the patch"] = _v(column, pay)
        v["nose over the patch"] = _v(column, nose)
        lid_col = _vpos(column, lid)
        lid_col_expect = (pu[1] - pu[0]) * (pv[1] - pv[0]) * L.gps_lid_t
        v["lid over the patch / footprint x t"] = lid_col / lid_col_expect
        ok["antenna"] = (abs(v["patch -> lid (BRep)"] - L.gps_patch_air_gap) < COM_SEAT_TOL
                         and max(v["payload over the patch"], v["nose over the patch"]) < GPS_CLEAR_TOL_MM3
                         and abs(v["lid over the patch / footprint x t"] - 1.0) < 0.01)
        # (4) stack in the cavity
        v["gps x payload"], v["plate x payload"] = _v(gps, pay), _v(gp, pay)
        v["gps x lid"], v["plate x lid"] = _v(gps, lid), _v(gp, lid)
        v["gps -> pod (BRep)"], v["plate -> pod (BRep)"] = _clearance(gps, pay), _clearance(gp, pay)
        ok["stack"] = (max(v["gps x payload"], v["plate x payload"], v["gps x lid"], v["plate x lid"]) < INTERSECT_TOL_MM3
                       and min(v["gps -> pod (BRep)"], v["plate -> pod (BRep)"]) >= L.gps_cavity_clear - 1e-3)
        # (5) blind lid-screw holes in the bosses
        hole_clear, ring_fill, floor_fill = [], [], []
        r_h = L.gps_boss_hole_d / 2
        for u, w in L.gps_boss_hole_uv:
            x, y, _ = L.gps_xyz(u, w, 0.0)
            hole_clear.append(_v(rg._cyl_z(r_h - e, L.gps_boss_hole_z0 + e, L.gps_z_rim - e, x, y), pay))
            ring = _off_axis_ring(x, y, r_h + e, r_h + 1.0, L.gps_boss_hole_z0 + e, L.gps_z_rim - e)
            ring_fill.append(_vpos(ring, pay) / ring.Volume())
            under = rg._cyl_z(r_h, L.gps_boss_hole_z0 - 1.0, L.gps_boss_hole_z0 - e, x, y)
            floor_fill.append(_vpos(under, pay) / under.Volume())
        v["boss hole clear (max mm^3)"], v["boss ring fill (min)"], v["boss hole floor fill (min)"] = (
            max(hole_clear), min(ring_fill), min(floor_fill))
        ok["bosses"] = (max(hole_clear) < GPS_CLEAR_TOL_MM3 and min(ring_fill) >= COM_FILL_MIN
                        and min(floor_fill) >= COM_FILL_MIN)
        # (6) raceway: cable probe through, walls solid, opens into the cavity; the bore over the pod is clear
        (vu0, vu1), hv = L.gps_rw_void_u, L.gps_rw_void_hv
        probe = rg._box_polar(vu0 + e, vu1 - e, -hv + e, hv - e, L.z_payload_bottom + e, L.gps_rw_void_z[1] - e, th)
        v["raceway cable probe x payload"] = _v(probe, pay)
        z_w0, z_w1 = L.gps_pk_solid_z[1] + 1.0, L.gps_floor_under_z(L.gps_rw_solid_u[1]) - 1.0
        (su0, su1), shv = L.gps_rw_solid_u, L.gps_rw_solid_hv
        walls = [rg._box_polar(vu1 + e, su1 - e, -shv + e, shv - e, z_w0, z_w1, th),            # outer wall
                 rg._box_polar(L.body_r + e, su1 - e, hv + e, shv - e, z_w0, z_w1, th),           # side walls
                 rg._box_polar(L.body_r + e, su1 - e, -shv + e, -hv - e, z_w0, z_w1, th)]
        v["raceway wall void (mm^3)"] = sum(_void(wl, pay) for wl in walls)
        above = rg._box_polar(vu0 + e, vu1 - e, -hv + e, hv - e, L.gps_floor_top_z(vu1) + e, L.gps_floor_top_z(vu1) + 3.0, th)
        v["raceway mouth in the cavity x payload"] = _v(above, pay)
        bore = rg._cyl_z(L.body_id / 2 - e, *L.gps_cut_z)
        v["payload bore over the pod x payload"] = _v(bore, pay)
        ok["raceway"] = (v["raceway cable probe x payload"] < GPS_CLEAR_TOL_MM3 and v["raceway wall void (mm^3)"] < C20_CLOSED_TOL_MM3
                         and v["raceway mouth in the cavity x payload"] < GPS_CLEAR_TOL_MM3
                         and v["payload bore over the pod x payload"] < GPS_CLEAR_TOL_MM3)
        # (7) avionics entry hole: open through the wall, under the pocket's open bottom
        hole = rg._radial_cyl(L.gps_entry_d - 2 * e, L.body_id / 2 - 1.0, L.body_r + 1.0, L.gps_entry_theta, L.gps_entry_z)
        v["entry hole probe x avionics"] = _v(hole, av)
        ring = rg._radial_cyl(L.gps_entry_d + 2.0, L.body_id / 2 + e, L.body_r - e, L.gps_entry_theta, L.gps_entry_z).cut(
            rg._radial_cyl(L.gps_entry_d + 2 * e, L.body_id / 2 - 1.0, L.body_r + 1.0, L.gps_entry_theta, L.gps_entry_z))
        v["entry hole wall ring fill"] = _vpos(ring, av) / ring.Volume()
        ok["entry hole"] = (v["entry hole probe x avionics"] < GPS_CLEAR_TOL_MM3 and v["entry hole wall ring fill"] >= COM_FILL_MIN
                            and L.gps_entry_z < L.gps_pk_z[0])
        # (8) latch arm / release servo vs the lid and the pod
        v["latch arm x lid"] = max(_v(self.shape["latch_arm"], lid), _v(rg.latch_arm(L, released=True), lid))
        v["release servo x lid"] = _v(self.shape["servo_release"], lid)
        ok["latch / release servo"] = max(v["latch arm x lid"], v["release servo x lid"]) < INTERSECT_TOL_MM3
        # antenna lever arm from the vehicle CG (report)
        cg = self.report["totals"]["com_mm"]
        arm = [a - c for a, c in zip(L.gps_antenna_xyz, cg)]
        allok = all(ok.values())
        bad = [k for k, s in ok.items() if not s]
        return Check("C26", title, PASS if allok else FAIL,
                     "; ".join(f"{k} {'ok' if s else 'FAIL'}" for k, s in ok.items())
                     + f"; lid top z {v['lid top z']:.3f} (limit {z_lim:.3f}), patch -> lid {v['patch -> lid (BRep)']:.3f} "
                       f"(air gap {L.gps_patch_air_gap:g}), stack -> pod {min(v['gps -> pod (BRep)'], v['plate -> pod (BRep)']):.3f} "
                       f"(>= {L.gps_cavity_clear:g}); antenna lever arm from the CG ({', '.join(f'{a:.1f}' for a in arm)}) mm, "
                       f"|r| {math.sqrt(sum(a * a for a in arm)):.1f} mm" + (f"; FAILED: {', '.join(bad)}" if bad else ""),
                     "measured: " + ", ".join(f"{k} {val:.4f}" for k, val in v.items())
                     + f"; probes inset {e:g} mm; noise sources from the phase centre (target >= {L.gps_noise_target_mm:g} mm, "
                       "reported only): " + ", ".join(f"{k} {d:.0f}" for k, d in sorted(L.gps_noise_mm.items(), key=lambda kv: kv[1]))
                     + f"; lead {L.gps_lead_mass_g:.2f} g ({L.gps_lead_len_mm:.0f} mm incl. slack)",
                     {"ok": ok, "measured": v, "antenna_xyz_mm": list(L.gps_antenna_xyz), "lever_arm_from_cg_mm": arm,
                      "noise_mm": L.gps_noise_mm, "lead_mass_g": L.gps_lead_mass_g, "applicable": True})

    # ---------------- v0.8.1 GNS board (electronics.gns, avionics_module.gns_mount) ----------------
    def c27_gns_board(self) -> Check:
        """v0.8.1 C27: the GNS board (ATmega128A #4: GPS + position hold) on its four bosses on the -x avionics tube wall,
        on the solids (uncomputable -> FAIL):

        * clearances (BRep) >= avionics_module.gns_mount.min_clear to the tray, its placeholders, the COM board, the
          radio box, the FCU and the rail ribs (avionics material in each rail-slot region); radially to the bore >=
          PLACEHOLDER_BORE_CLEAR_MM; no overlap with the avionics module;
        * seated: the avionics material in a GNS_SEAT_SLAB slab just behind the board's back face is the four boss tips
          (4 x (diamond - hole) within GNS_SEAT_AREA_TOL), and nothing of the module is in front of the back face over the
          board's footprint;
        * bosses: each blind hole is clear (probe inset COM_PROBE_INSET), has material around it (ring) and beyond its
          end; the lowest material of each boss is its bottom vertex and the material at v_h +- hd/2 starts hd/2 higher
          (45 deg faces: support-free when the module prints upright);
        * leads: the GPS lead's bay leg and the glink lead (bundle-diameter tubes along Layout's polylines) meet no
          structure or placeholder except the boards they plug into (gns, and nav for the glink).
        The board's distances (ESC, GPS antenna, LoRa antenna, ...) are reported.  The GPS entry hole beside the top bosses
        stays open: C26's hole probe runs through the module that carries them.  N/A without electronics.gns."""
        title = ("(v0.8.1) GNS board on the -x tube wall: clear of tray / placeholders / COM / radio box / rails, seated on "
                 "four 45-deg diamond bosses, blind tap holes, GPS and glink leads clear")
        L = self.L
        if not getattr(L, "has_gns", False) or "gns" not in self.shape:
            return Check("C27", title, NA, "not applicable: params carry no electronics.gns", "", {"applicable": False})
        gns, av = self.shape["gns"], self.shape["avionics_module"]
        e, mc, th = COM_PROBE_INSET, L.gns_min_clear, L.gns_theta
        hd, bw = L.gns_boss_hd, L.gns_dims[1]
        v: dict[str, float] = {}
        ok: dict[str, bool] = {}
        # (1) clearances
        near = {n: _clearance(gns, self.shape[n]) for n in ("avionics_tray", "battery", "esc", "nav", "misc_electronics", "com",
                                                            "radio_log", "fcu") if n in self.shape}
        x_r0, x_r1 = L.tray_x - L.rail_gap / 2 - L.rail_w, L.tray_x + L.rail_gap / 2 + L.rail_w
        y_in = L.rail_y - L.rail_depth
        for sy, nm in ((+1.0, "rails +y"), (-1.0, "rails -y")):
            ya, yb = (y_in - e, L.body_r + 1.0) if sy > 0 else (-L.body_r - 1.0, -y_in + e)
            try:
                rib = av.intersect(_box(x_r0 - e, x_r1 + e, ya, yb, L.z_rail_bottom - e, L.z_rail_top + e))
                near[nm] = _clearance(gns, rib) if rib.Volume() > 1e-6 else 0.0      # no rib found: no evidence
            except Exception:  # noqa: BLE001
                near[nm] = 0.0
        v["gns x avionics_module"] = _v(gns, av)
        v["bore radial (board corners)"] = L.body_id / 2 - max(math.hypot(q.X, q.Y) for q in gns.Vertices())
        ok["clearances"] = (all(d >= mc - 1e-6 for d in near.values()) and v["gns x avionics_module"] < INTERSECT_TOL_MM3
                            and v["bore radial (board corners)"] >= PLACEHOLDER_BORE_CLEAR_MM - 1e-6)
        # (2) seat on the four boss tips
        z0, z1 = L.gns_z
        # over the bosses' tangential extent only (a thicker tube wall - KAIST - reaches u_back + slab beyond it)
        v_b = L.gns_pattern / 2 + hd + e
        slab = rg._box_polar(L.gns_u_back, L.gns_u_back + GNS_SEAT_SLAB, -v_b, v_b,
                             L.gns_boss_z[0] - 1.0, L.gns_boss_z[1] + 1.0, th)
        area = _vpos(slab, av) / GNS_SEAT_SLAB
        area_design = 4 * (2 * hd * hd - math.pi * (L.gns_hole_d / 2) ** 2)
        v["boss tip area behind the back face (mm^2)"] = area
        v["boss tip area / design"] = area / area_design
        front = rg._box_polar(L.gns_u_front - 1.0, L.gns_u_back - e, -bw / 2, bw / 2, z0, z1, th)
        v["avionics material in front of the back face"] = _v(front, av)
        v["board -> avionics_module (BRep, on the boss tips)"] = _clearance(gns, av)
        ok["seat"] = (abs(v["boss tip area / design"] - 1.0) < GNS_SEAT_AREA_TOL
                      and v["avionics material in front of the back face"] < INTERSECT_TOL_MM3
                      and v["board -> avionics_module (BRep, on the boss tips)"] < COM_SEAT_TOL)
        # (3) bosses: blind holes, rings, material beyond, 45 deg faces
        hole_clear, ring_fill, end_fill, faces = [], [], [], []
        u0h, u1h, dh = L.gns_hole_u[0], L.gns_hole_u[1], L.gns_hole_d
        w = GNS_FACE_PROBE_W
        for (vh, zh) in L.gns_hole_vz:
            hole_clear.append(_v(rg._radial_cyl(dh - 2 * e, u0h + e, u1h - e, th, zh, vh), av))
            ring = rg._radial_cyl(dh + 2.0, u0h + e, u1h - e, th, zh, vh).cut(
                rg._radial_cyl(dh + 2 * e, u0h - 1.0, u1h + 1.0, th, zh, vh))
            ring_fill.append(_vpos(ring, av) / ring.Volume())
            beyond = rg._radial_cyl(dh, u1h + e, u1h + 1.0, th, zh, vh)
            end_fill.append(_vpos(beyond, av) / beyond.Volume())
            for dv, z_exp in ((0.0, zh - hd), (+hd / 2, zh - hd / 2), (-hd / 2, zh - hd / 2)):
                vc = vh + dv
                u_wall = L.gns_u_wall(abs(vc) + w)
                col = rg._box_polar(L.gns_u_back + e, u_wall - e, vc - w / 2, vc + w / 2, zh - hd - 2.0, zh, th)
                faces.append(_bottom_z(av, col) - z_exp)
        v["hole clear (max mm^3)"], v["hole ring fill (min)"], v["beyond the hole end fill (min)"] = (
            max(hole_clear), min(ring_fill), min(end_fill))
        v["boss underside vs 45 deg design (max |dz|)"] = max(abs(f) for f in faces)
        ok["bosses"] = (max(hole_clear) < COM_HOLE_CLEAR_TOL_MM3 and min(ring_fill) >= COM_FILL_MIN
                        and min(end_fill) >= COM_FILL_MIN and v["boss underside vs 45 deg design (max |dz|)"] < GNS_FACE_TOL)
        # (4) leads as bundle-diameter tubes
        bd = L.gps_bundle_d

        def tube_along(pts: list) -> cq.Shape:
            segs = []
            for a, b in zip(pts[:-1], pts[1:]):
                pa, pb = cq.Vector(*a), cq.Vector(*b)
                if (pb - pa).Length > 1e-6:
                    segs.append(cq.Solid.makeCylinder(bd / 2, (pb - pa).Length, pa, (pb - pa).normalized()))
            return rg._fuse(segs, clean=False) if len(segs) > 1 else segs[0]
        leads = {"GPS lead (bay leg)": (tube_along(L.gps_lead_polyline[L.gps_lead_bay_start:]), ("gns",)),
                 "glink lead": (tube_along(L.glink_lead_polyline), ("gns", "nav"))}
        others = [n for n in ("avionics_module", "avionics_tray", "battery", "esc", "nav", "misc_electronics", "com",
                              "radio_log", "fcu", "payload_module", "gns") if n in self.shape]
        lead_hits = {}
        for nm, (shp, plugged) in leads.items():
            hits = {o: _v(shp, self.shape[o]) for o in others if o not in plugged}
            worst = max(hits, key=hits.get)
            lead_hits[nm] = (hits[worst], worst)
        ok["leads"] = all(h < INTERSECT_TOL_MM3 for h, _ in lead_hits.values())
        allok = all(ok.values())
        bad = [k for k, s_ in ok.items() if not s_]
        d = L.gns_distance_mm
        return Check("C27", title, PASS if allok else FAIL,
                     "; ".join(f"{k} {'ok' if s_ else 'FAIL'}" for k, s_ in ok.items())
                     + "; clearance " + ", ".join(f"{k} {x:.2f}" for k, x in near.items())
                     + f" (>= {mc:g}), radial {v['bore radial (board corners)']:.2f}; seat area {area:.2f} mm^2 "
                       f"({v['boss tip area / design']:.4f} of design); leads "
                     + ", ".join(f"{k} max {h:.3f} mm^3 ({o})" for k, (h, o) in lead_hits.items())
                     + f"; ESC {d['ESC (box)']:.1f} mm (target {L.gns_esc_target_mm:g} where possible; best on this wall "
                       f"{L.gns_esc_best_on_wall_mm:.1f})" + (f"; FAILED: {', '.join(bad)}" if bad else ""),
                     "measured: " + ", ".join(f"{k} {x:.4f}" for k, x in v.items())
                     + f"; board x {L.gns_x[0]:.2f}..{L.gns_x[1]:.2f} y {L.gns_y[0]:.2f}..{L.gns_y[1]:.2f} z {z0:.2f}..{z1:.2f} "
                       f"({L.gns_mass_g:g} g: {L.gns_board_mass_g:g} ASSUMED + {L.gns_screws_g:g} screws); bosses diamond hd "
                       f"{hd:g} at (v, z) " + ", ".join(f"({a:+.1f}, {b:.1f})" for a, b in L.gns_hole_vz)
                     + f"; holes {dh:g} mm u {u0h:g}..{u1h:g}; probes inset {e:g} mm; distances (reported only): "
                     + ", ".join(f"{k} {x:.0f}" for k, x in sorted(d.items(), key=lambda kv: kv[1]))
                     + f"; GPS lead {L.gps_lead_route_len:.0f} mm route, glink {L.glink_route_len:.0f} mm route "
                       f"({L.glink_mass_g:.2f} g estimate inside the board mass)",
                     {"ok": ok, "measured": v, "clearance": near, "lead_hits": {k: list(h) for k, h in lead_hits.items()},
                      "distance_mm": d, "min_clear": mc, "applicable": True})

    # ---------------- extra informational rows (WARN at most) ----------------
    def x1_vane_pair(self) -> Check:
        L = self.L
        dmax = L.p["vane"]["max_deflection_mech_deg"]
        dcmd = L.p["vane"]["max_deflection_cmd_deg"]
        worst = {dmax: (0.0, ""), dcmd: (0.0, "")}
        for d in (dmax, dcmd):
            for i in range(L.vane_count):
                j = (i + 1) % L.vane_count
                for si, sj in ((+1, -1), (-1, +1), (+1, +1), (-1, -1)):
                    v = _v(rg.vane(L, i, si * d), rg.vane(L, j, sj * d))
                    if v > worst[d][0]:
                        worst[d] = (v, f"vane_{i + 1} d={si * d:+.0f} vs vane_{j + 1} d={sj * d:+.0f}")
        vmax = max(w[0] for w in worst.values())
        return Check("X1", "(extra) adjacent vanes both deflected (mechanical and commanded limits)",
                     WARN if vmax >= INTERSECT_TOL_MM3 else PASS,
                     f"at +-{dmax:.0f}deg {worst[dmax][0]:.3f} mm^3 ({worst[dmax][1]}); at +-{dcmd:.0f}deg {worst[dcmd][0]:.3f} mm^3 ({worst[dcmd][1]})",
                     f"inner corners near vane_r_in {L.vane_r_in:g}; not required by SPEC C4 (which deflects one vane at a time)",
                     {"mech": worst[dmax], "cmd": worst[dcmd]})

    def x2_pad(self) -> Check:
        v = _v(self.shape["launch_pad"], self.prop)
        return Check("X2", "(extra) launch pad vs propulsion module (fins rest on the plate)", WARN if v >= INTERSECT_TOL_MM3 else PASS,
                     f"pad x propulsion_module {v:.3f} mm^3",
                     ("v0.9: fairing keels (lowest Z %.2f) vs fin-locator ribs" % self.L.fair_keel_z0) if self.L.direct_drive
                     else "pods' chamfer gussets vs fin-locator ribs", {"volume": v})

    def run(self) -> list[Check]:
        if getattr(self.L, "vtol", False):
            return self.run_vtol()
        return [self.c1_bed_fit(), self.c2_envelope(), self.c3_edf(), self.c4_vanes(), self.c5_linkage(),
                self.c6_modules(), self.c7_avionics(), self.c8_exhaust(), self.c9_inlet(), self.c10_min_feature(),
                self.c11_nose(), self.c12_thrust_weight(), self.c13_stability(), self.c14_placeholders(),
                self.c15_latch(), self.c16_screw_holes(), self.c17_presets(), self.c18_tab_slide(),
                self.c19_tab_screws(), self.c20_wire_path(), self.c21_outer_skin(), self.c22_com_board(),
                self.c23_servo_leads(), self.c24_radio_log(), self.c25_fcu_board(), self.c26_gps_pod(),
                self.c27_gns_board(), self.c28_coupler(), self.c29_bullets(), self._guard("C33", self.c33_lateral_cg),
                self._guard("C34", self.c34_clock_coverage), self._guard("C35", self.c35_axial_retention),
                self._guard("C36", self.c36_livery), self.x1_vane_pair(), self.x2_pad()]

    # ---- v0.9 CAD-2 VTOL configuration ----
    VTOL_NA = {"C2": ("payload envelope does not intersect any printed body", "no CanSat payload in the VTOL configuration"),
               "C11": ("nose cone / chute pack", "no nose cone or chute in the VTOL configuration (top cap: C30)"),
               "C13": ("static stability margin", "hover vehicle: no ballistic coast; the CG / tip-over / vane authority are "
                                                  "in C32 and the sizing report"),
               "C15": ("release latch", "no payload release in the VTOL configuration"),
               "C22": ("COM board", "no COM board: the carrier PCB (C25) holds the COM ATmega128A"),
               "C24": ("radio / logger box", "the LoRa module and OpenLog sit on the carrier PCB (C25)"),
               "C26": ("GPS pod", "no GPS pod: the GPS sits on the top cap (C30)"),
               "C27": ("GNS board", "the GNS ATmega128A is on the carrier PCB (C25)"),
               "X2": ("launch pad", "the VTOL vehicle stands on its fins (no launch pad)")}

    def _guard(self, cid: str, fn) -> Check:
        """Run one check; an exception becomes a FAIL row (never a silent pass, never a crashed table)."""
        try:
            return fn()
        except Exception as exc:  # noqa: BLE001 - reported as the check's FAIL
            return Check(cid, "check raised", FAIL, f"{type(exc).__name__}: {exc}"[:200], repr(exc), {"error": repr(exc)})

    def run_vtol(self) -> list[Check]:
        """v0.9 CAD-2: the rocket rows that apply (each guarded), N/A rows for the rocket-only ones (VTOL_NA), and the VTOL
        rows C30 (top cap, tether path, GPS), C31 (avionics windows, tray holes), C32 (fin legs: holes, tip-over, vane
        authority inputs)."""
        out = []
        order = [("C1", self.c1_bed_fit), ("C2", None), ("C3", self.c3_edf), ("C4", self.c4_vanes), ("C5", self.c5_linkage),
                 ("C6", self.c6_modules), ("C7", self.c7_avionics), ("C8", self.c8_exhaust), ("C9", self.c9_inlet),
                 ("C10", self.c10_min_feature), ("C11", None), ("C12", self.c12_thrust_weight), ("C13", None),
                 ("C14", self.c14_placeholders), ("C15", None), ("C16", self.c16_screw_holes), ("C17", self.c17_presets),
                 ("C18", self.c18_tab_slide), ("C19", self.c19_tab_screws), ("C20", self.c20_wire_path),
                 ("C21", self.c21_outer_skin), ("C22", None), ("C23", self.c23_servo_leads), ("C24", None),
                 ("C25", self.c25_fcu_board), ("C26", None), ("C27", None), ("C28", self.c28_coupler), ("C29", self.c29_bullets),
                 ("C30", self.c30_top_cap), ("C31", self.c31_av_windows), ("C32", self.c32_fin_legs), ("C33", self.c33_lateral_cg),
                 ("C34", self.c34_clock_coverage), ("C35", self.c35_axial_retention), ("C36", self.c36_livery),
                 ("X1", self.x1_vane_pair), ("X2", None)]
        for cid, fn in order:
            if fn is None:
                title, why = self.VTOL_NA[cid]
                out.append(Check(cid, title, NA, f"not applicable: mission.config 'vtol' - {why}", "", {"applicable": False}))
            else:
                out.append(self._guard(cid, fn))
        return out

    def c30_top_cap(self) -> Check:
        """v0.9 CAD-2 VTOL: the top cap on the avionics spigot - (a) the cap does not intersect the avionics module, the
        carrier / bay placeholders or the GPS placeholders (it touches the spigot top face only); (b) every plate hole and
        the hardpoint hole is open (a thin probe through it meets no cap material); (c) the ribs / hub / rim ring are
        material (probes inside them); (d) the Layout clearances (cap_clearance) and the tether-path distances (cap_tether:
        the free cable from the boot top >= min_window_clear from every intake window) - the boot base distance is
        reported; (e) the GPS patch top is the highest point of the vehicle except the tether boot (open sky)."""
        L, sh = self.L, self.shape
        cap = sh["top_cap"]
        vols = {n: _v(cap, sh[n]) for n in ("avionics_module", "avionics_tray", "battery", "buffer_pack", "esc",
                                              "misc_electronics", L.fcu_part, "gps", "gps_ground_plane") if n in sh}
        worst = max(vols, key=vols.get)
        holes = {}
        zp, zt = L.z_cap_plate_bottom, L.z_cap_top
        for nm, (x, y, d) in list(L.cap_holes.items()) + [("hardpoint", (0.0, 0.0, L.cap_hardpoint_d))]:
            z0 = L.z_cap_under - 0.5 if nm == "hardpoint" else zp - 0.5
            holes[nm] = _v(cap, rg._cyl_z(0.25 * d, z0, zt + 0.5, x, y))
        probes = {}
        for th in L.cap_rib_thetas:
            pt = rg._box_polar(0.5 * (L.cap_rib_r[0] + L.cap_rib_r[1]) - 1.0, 0.5 * (L.cap_rib_r[0] + L.cap_rib_r[1]) + 1.0,
                               -0.4, 0.4, L.z_cap_under + 1.0, L.z_cap_under + 3.0, th)
            probes[f"rib {th:g}"] = _v(cap, pt) / pt.Volume()
        zh0 = L.z_cap_under + (L.cap_washer_recess[1] if getattr(L, "cap_hardpoint", False) else 0.0)   # over the washer recess
        hub = rg._tube(0.5 * L.cap_hub_d - 0.3, 0.5 * L.cap_hardpoint_d + 0.3, zh0 + 1.0, zh0 + 3.0)
        probes["hub"] = _v(cap, hub) / hub.Volume()
        ring = rg._tube(L.cap_ring_r[1] - 0.3, L.cap_ring_r[0] + 0.3, L.z_cap_ring_bottom + 0.5, L.z_cap_ring_bottom + 2.0)
        probes["rim ring"] = _v(cap, ring) / ring.Volume()
        short = {k: v for k, v in L.cap_clearance.items() if L.cap_limits.get(k) is not None and v < L.cap_limits[k] - 1e-9}
        tdist = L.cap_tether["free cable start (boot top) -> nearest intake window"]
        treq = L.cap_tether["required (free cable)"]
        # (the tether hardware - eye bolt - stands over the patch by design: the boot shadow is reported below)
        zmax = max(e["bbox_mm"][1][2] for n, e in self.report["parts"].items()
                   if e["kind"] != "lumped" and not n.startswith("tether_"))
        gps_top = L.cap_patch_z[1]
        fails = []
        # v0.9 fix round: the hardpoint stack (load washer flush in the hub, jam nut, bolt end) keeps
        # top_cap.hardpoint.min_clear_below over every bay box in every VTOL source (the flight pack too: catch-line tests)
        stack_txt = ""
        if getattr(L, "cap_hardpoint", False):
            sc = L.cap_stack_clearance
            worst_s = min(sc, key=sc.get)
            if sc[worst_s] < L.cap_stack_min_clear - 1e-9:
                fails.append(f"hardpoint stack {sc[worst_s]:.2f} mm from {worst_s}")
            recess = rg._tube(L.cap_washer_d / 2 - 0.3, L.cap_hardpoint_d / 2 + 0.3, L.z_cap_under + 0.2,
                              L.z_cap_under + L.cap_washer_t - 0.2)
            rv = _v(cap, recess)
            if rv > INTERSECT_TOL_MM3:
                fails.append(f"load-washer recess not open ({rv:.3f} mm^3)")
            stack_txt = (f"; hardpoint stack: D{L.cap_washer_d:g} x {L.cap_washer_t:g} washer recess open ({rv:.4f} mm^3), "
                         f"M5 x {L.cap_bolt_len:g} end z {L.z_bolt_end:.1f} / jam nut bottom {L.z_cap_nut[0]:.1f}, min "
                         f"{sc[worst_s]:.2f} mm to {worst_s} (>= {L.cap_stack_min_clear:g})")
        if vols[worst] > INTERSECT_TOL_MM3:
            fails.append(f"cap intersects {worst} ({vols[worst]:.3f} mm^3)")
        blocked = {k: v for k, v in holes.items() if v > INTERSECT_TOL_MM3}
        if blocked:
            fails.append(f"holes blocked {blocked}")
        thin = {k: v for k, v in probes.items() if v < 0.95}
        if thin:
            fails.append(f"rib / hub / ring probes not material {thin}")
        if short:
            fails.append(f"Layout clearances short {short}")
        if tdist < treq - 1e-9:
            fails.append(f"free tether cable {tdist:.1f} < {treq:g} mm from an intake window")
        if gps_top < zmax - 1e-6:
            fails.append(f"GPS patch top {gps_top:.2f} below the vehicle top {zmax:.2f}")
        return Check("C30", "VTOL top cap: fit, open holes, ribs / hub / ring, clearances, tether path, GPS sky",
                     FAIL if fails else PASS,
                     f"max cap overlap {vols[worst]:.4f} mm^3 ({worst}); holes open; min rib/hub/ring fill "
                     f"{min(probes.values()):.3f}; min clearance margin "
                     f"{min(v - L.cap_limits[k] for k, v in L.cap_clearance.items() if L.cap_limits.get(k) is not None):.2f} mm; "
                     f"free tether cable {tdist:.1f} mm (>= {treq:g}), boot base "
                     f"{L.cap_tether['boot base (cap top) -> nearest intake window']:.1f} mm (reported)" + stack_txt,
                     "; ".join(fails) if fails else
                     f"GPS patch top z {gps_top:.2f} = vehicle top; boot shadow on the patch {L.cap_gps_boot_az_deg:.1f} deg "
                     f"azimuth up to {L.cap_gps_boot_el_deg:.1f} deg elevation (tether variants only)",
                     {"overlap_mm3": vols, "hole_probe_mm3": holes, "fill": probes, "clearance_mm": dict(L.cap_clearance),
                      "tether_mm": dict(L.cap_tether), "gps_patch_top_z": gps_top, "vehicle_top_z": zmax})

    def c31_av_windows(self) -> Check:
        """v0.9 CAD-2 VTOL: (a) each avionics window is open through the wall (a radial probe at its centre meets no
        avionics_module material) and the struts between them are material (probe at the mid angle); (b) no window opens
        onto a rail rib / tray stop (the rib probes stay material; Layout raised on the angle already); (c) the tray holes
        are open and the tray keeps its strap slots; (d) the windows' gable is >= 45 deg (Layout)."""
        L, sh = self.L, self.shape
        av = sh["avionics_module"]
        zc = 0.5 * (L.avw_z0 + L.avw_z1)
        r_in, r_out = L.body_id / 2, L.body_r
        open_ = {th: _v(av, rg._radial_cyl(2.0, r_in - 0.5, r_out + 0.5, th, zc)) for th in L.avw_thetas}
        th_sorted = sorted(t % 360.0 for t in L.avw_thetas)
        mids = []
        for a, b in zip(th_sorted, th_sorted[1:] + [th_sorted[0] + 360.0]):
            if b - a < 30.0:
                mids.append(0.5 * (a + b))
        struts = {m: _v(av, rg._radial_cyl(1.0, r_in + 0.1, r_out - 0.1, m, zc)) /
                  rg._radial_cyl(1.0, r_in + 0.1, r_out - 0.1, m, zc).Volume() for m in mids}
        rib = {}
        for lo, hi in L.avw_rail_thetas:
            rib[f"{0.5 * (lo + hi):.1f}"] = _v(av, rg._radial_cyl(1.0, r_in - 3.0, r_in - 0.5, 0.5 * (lo + hi), zc))
        tray = sh["avionics_tray"]
        th_open = {}
        if getattr(L, "has_tray_holes", False):
            (yi, yo), (hz0, hz1) = L.tray_hole_y, L.tray_hole_z
            for s in (1.0, -1.0):
                yc = s * 0.5 * (yi + yo)
                th_open[f"y{yc:+.0f}"] = _v(tray, rg._box(L.tray_x - 2.0, L.tray_x + 2.0, yc - 1.0, yc + 1.0, hz0 + 2.0, hz1 - 2.0))
        fails = []
        if max(open_.values()) > INTERSECT_TOL_MM3:
            fails.append(f"windows not open {[(k, round(v, 3)) for k, v in open_.items() if v > INTERSECT_TOL_MM3]}")
        if struts and min(struts.values()) < 0.95:
            fails.append(f"struts not material {struts}")
        if min(rib.values()) <= 0.0:
            fails.append(f"rail ribs missing at the window height {rib}")
        if th_open and max(th_open.values()) > INTERSECT_TOL_MM3:
            fails.append(f"tray holes not open {th_open}")
        return Check("C31", "VTOL avionics windows open, struts / rail ribs material, tray lightening holes open",
                     FAIL if fails else PASS,
                     f"{len(open_)} windows, max wall in a window {max(open_.values()):.4f} mm^3; min strut fill "
                     f"{(min(struts.values()) if struts else float('nan')):.3f} ({L.avw_strut_w:.2f} mm at the bore); rail "
                     f"clearance {L.avw_rail_clear_deg:.1f} deg; tray holes {len(th_open)} open",
                     "; ".join(fails), {"window_probe_mm3": open_, "strut_fill": struts, "rib_probe_mm3": rib,
                                        "tray_hole_probe_mm3": th_open, "open_area_mm2": L.avw_open_area_mm2})

    def c32_fin_legs(self) -> Check:
        """v0.9 CAD-2 VTOL: the fins are the landing legs - (a) each fin hole is open (probe at its centroid) and the frame
        around it is material (probes in the bottom strip, the tip strip and above the apex); (b) the footprint (fin tips)
        and the tip-over angles at the CAD CG (toward a fin / between two fins); the strength hand calc is in the CAD-2
        report.  WARN when the between-fins tip-over angle is below 15 deg."""
        L, sh = self.L, self.shape
        prop = self.prop
        if not getattr(L, "has_fin_hole", False):
            # v0.9 fix round: solid fins (the hole cut the leg's out-of-plane margin to ~1.5x - mass reviewer): the fin plate
            # outboard of the fairing is material over its whole height; footprint and tip-over (with the CG's offset)
            t = self.report["extra"]["vtol"]["tip_over"]
            fill = {}
            u0 = (L.fair_u[1] if L.direct_drive else L.fin_root_r) + 2.0
            u1 = L.fin_tip_r - 2.0
            for th in L.fin_thetas:
                zt = L.fin_le_z(u1) - 1.0
                pb = rg._box_polar(u0, u1, -0.3, 0.3, L.fin_te_z + 1.0, zt, th)
                fill[f"{th:g}"] = _v(prop, pb) / pb.Volume()
            ang_b = t.get("angle_between_fins_with_offset_deg", t["angle_between_fins_deg"])
            ang_f = t.get("angle_toward_a_fin_with_offset_deg", t["angle_toward_a_fin_deg"])
            fails = [] if min(fill.values()) >= 0.95 else [f"fin plate not solid {fill}"]
            st = FAIL if fails else (WARN if ang_b < 15.0 else PASS)
            return Check("C32", "VTOL fin legs: solid fin plates (no lightening hole), footprint / tip-over with the CG offset", st,
                         f"fin plate fill min {min(fill.values()):.3f}; footprint D{2 * L.fin_tip_r:.1f}; tip-over "
                         f"{ang_f:.1f} deg toward a fin, {ang_b:.1f} deg between fins with the CG {t.get('cg_lateral_offset_mm', 0.0):.2f} "
                         f"mm off the axis (centred: {t['angle_toward_a_fin_deg']:.1f} / {t['angle_between_fins_deg']:.1f}; CG "
                         f"{t['cg_height_mm']:.1f} mm)", "; ".join(fails), {"fin_fill": fill, "tip_over": t})
        (u0, z0), (u1, _), (_, z1), (um, za), _ = L.fin_hole_uz
        res, frame = {}, {}
        for th in L.fin_thetas:
            res[th] = _v(prop, rg._radial_cyl(2.0, u0 + 2.0, u1 - 2.0, th, 0.5 * (z0 + z1)))
            for nm, (ua, ub, za_, zb_) in {"bottom strip": (u0 + 1.0, u1 - 1.0, L.fin_te_z + 1.0, z0 - 1.0),
                                           "tip strip": (u1 + 0.5, L.fin_tip_r - 0.5, z0, z1),
                                           "LE chord": (um - 1.0, um + 1.0, za + 0.5, L.fin_le_z(um) - 0.5)}.items():
                pb = rg._box_polar(ua, ub, -0.3, 0.3, za_, zb_, th)
                frame[f"{th:g} {nm}"] = _v(prop, pb) / pb.Volume()
        t = self.report["extra"]["vtol"]["tip_over"]
        fails = []
        if max(res.values()) > INTERSECT_TOL_MM3:
            fails.append(f"fin holes not open {res}")
        if min(frame.values()) < 0.95:
            fails.append(f"fin frame not material {[(k, round(v, 3)) for k, v in frame.items() if v < 0.95]}")
        st = FAIL if fails else (WARN if t["angle_between_fins_deg"] < 15.0 else PASS)
        return Check("C32", "VTOL fin legs: holes open, frame material, footprint / tip-over", st,  # (CAD-2 hole variant)
                     f"holes open (max {max(res.values()):.4f} mm^3), min frame fill {min(frame.values()):.3f}; footprint "
                     f"D{2 * L.fin_tip_r:.1f}; tip-over {t['angle_toward_a_fin_deg']:.1f} deg toward a fin, "
                     f"{t['angle_between_fins_deg']:.1f} deg between fins (CG {t['cg_height_mm']:.1f} mm)",
                     "; ".join(fails), {"hole_probe_mm3": res, "frame_fill": frame, "tip_over": t})

    def c33_lateral_cg(self) -> Check:
        """v0.9 fix round (mass reviewer: 'lateral CG offset uses 9-20 % of the hover pitch authority as standing trim'):
        the CAD CG's lateral offset r and the standing vane trim it needs in a hover, as a fraction of the vane pitch
        authority at hover thrust: W r / (2 N lever), N = q S cl_alpha d_cmd with q = T / (2 A_exit), T = W (the model of
        Layout._direct_drive / sizing/vtol_sizing.py).  PASS <= C33_TRIM_PASS, WARN <= C33_TRIM_WARN, else FAIL."""
        L, rep = self.L, self.report
        tot = rep["totals"]
        cx, cy, cz = tot["com_mm"]
        r_off = math.hypot(cx, cy)
        w_n = tot["mass_g"] * 1e-3 * 9.80665
        e, v = L.p["edf"], L.p["vane"]
        a_exit = math.pi / 4 * (L.duct_id ** 2 - (e["exit_hub_fraction"] * L.duct_id) ** 2) * 1e-6
        lever = (cz - L.vane_hinge_z) * 1e-3
        # review 2026-10-29 (FIND-1): the motor leads cross the jet (Layout.ml_jet_drag, E): drag D = Cd q A_f = k T with
        # q = T / (2 A_exit), so a hover needs T = W / (1 - k); about the CG the thrust (on the axis) and the lead drag (at
        # p, along the jet) leave the moment W c + D p (c, p lateral vectors) that the vanes trim
        jd = getattr(L, "ml_jet_drag", None)

        # wire09c clock mount: the leads cross the jet anywhere in the passage's azimuth range (Layout.ml_jet_drag
        # azimuth_range_deg, theta_P +- E_design): the drag point is swept over it and the status comes from the WORST
        # azimuth and the worst end of the Cd band
        az = jd.get("azimuth_range_deg") if jd else None
        xy_list = ([(jd["centroid_u_mm"] * math.cos(math.radians(az[0] + (az[1] - az[0]) * k / 40)),
                     jd["centroid_u_mm"] * math.sin(math.radians(az[0] + (az[1] - az[0]) * k / 40))) for k in range(41)]
                   if az else [jd["xy_mm"] if jd else (0.0, 0.0)])
        xy_cur = [xy_list[0]]

        def trim_at(cd: float) -> tuple[float, float, float, float, float]:
            k = cd * jd["area_mm2"] * 1e-6 / (2 * a_exit) if jd else 0.0
            t_n = w_n / (1.0 - k)
            q_ = 0.5 * t_n / a_exit
            n_ = q_ * L.vane_area_mm2 * 1e-6 * v["cl_alpha_per_rad"] * math.radians(v["max_deflection_cmd_deg"])
            m_a = 2 * n_ * lever
            px, py = xy_cur[0]
            d_n = k * t_n
            m_need = math.hypot(w_n * cx + d_n * px, w_n * cy + d_n * py) * 1e-3
            return (m_need / m_a if m_a > 0 else math.inf), q_, n_, m_a, d_n

        trim0 = trim_at(0.0)[0]
        cd0 = jd["cd_ASSUMED"] if jd else 0.0
        best = None
        for xy in xy_list:                 # the worst azimuth (by the worst end of the Cd band)
            xy_cur[0] = xy
            b_ = [trim_at(c)[0] for c in jd["cd_band_ASSUMED"]] if jd else [trim_at(cd0)[0]] * 2
            w_ = max([trim_at(cd0)[0]] + b_)
            if best is None or w_ > best[0]:
                best = (w_, xy)
        xy_cur[0] = best[1]
        az_worst = math.degrees(math.atan2(best[1][1], best[1][0]))
        trim, q, n_side, m_auth, d_n = trim_at(cd0)
        band = [trim_at(c)[0] for c in jd["cd_band_ASSUMED"]] if jd else [trim, trim]
        trim_deg = trim * v["max_deflection_cmd_deg"]
        # fix round (review 2026-10-29 G4): the lead drag is an estimate (E, Cd ASSUMED) - the status comes from the WORST
        # end of the stated Cd band (never from the central value alone)
        worst = max([trim] + list(band))
        st = PASS if worst <= C33_TRIM_PASS else WARN if worst <= C33_TRIM_WARN else FAIL
        drag_txt = (f"; with the motor-lead drag in the jet (E, Cd {cd0:g}: D {d_n:.3f} N at u {jd['centroid_u_mm']:.1f} mm"
                    + (f", worst azimuth {az_worst:+.2f} deg of {az[0]:+.2f}..{az[1]:+.2f}" if az else "") + ") "
                    f"trim {100 * trim:.1f} % (Cd {jd['cd_band_ASSUMED'][0]:g}-{jd['cd_band_ASSUMED'][1]:g}: "
                    f"{100 * band[0]:.1f}-{100 * band[1]:.1f} %; CG alone {100 * trim0:.1f} %; status from the worst end "
                    f"{100 * worst:.1f} %)") if jd else ""
        return Check("C33", "(fix round) lateral CG (+ lead drag in the jet, E): standing vane trim in a hover <= 5 % of the "
                     "pitch authority (WARN <= 10 %), worst end of the Cd band",
                     st, f"CG ({cx:+.2f}, {cy:+.2f}) mm -> {r_off:.2f} mm off the axis; trim {100 * trim:.1f} % of the hover "
                     f"pitch authority {m_auth:.3f} N m ({trim_deg:.2f} of {v['max_deflection_cmd_deg']:g} deg){drag_txt}",
                     f"W {w_n:.2f} N, q_exit {q:.0f} Pa at hover, vane force {n_side:.3f} N each, lever {1e3 * lever:.1f} mm "
                     f"(CG z {cz:.1f} - hinge {L.vane_hinge_z:g})",
                     {"cg_xy_mm": [cx, cy], "offset_mm": r_off, "trim_fraction": trim, "trim_deg": trim_deg,
                      "authority_Nm": m_auth, "trim_fraction_cg_only": trim0, "trim_fraction_cd_band": band,
                      "trim_fraction_status": worst,
                      "lead_drag_N": d_n, "lead_drag": dict(jd) if jd else None,
                      "worst_azimuth_deg": az_worst if az else None})

    def c34_clock_coverage(self) -> Check:
        """wire09c (user 2026-10-30): clock coverage - with the keyways (Layout._clock_seat) and the hover seats, EVERY exit
        azimuth (0-360 deg, recomputed here independently of Layout at 0.01 deg) is within E_cov of the lead passage for
        some seat (worst residual <= E_cov, no gaps); the EDF placeholder at every seated configuration x module = 0;
        the minimum sleeve tooth >= sleeve_min.  N/A without the clock mount."""
        L = self.L
        title = ("(wire09c clock mount) clock coverage: every exit azimuth 0-360 deg within E_cov of the lead passage, every "
                 "seat clear, sleeve teeth >= sleeve_min")
        if not getattr(L, "clock_mount", False):
            return Check("C34", title, NA, "not applicable: no clock mount (propulsion_module.tab_mount)", "",
                         {"applicable": False})
        rc = ckc.coverage_recheck(L)
        seat_v = 0.0
        for c in L.ck_configs:
            seat_v = max(seat_v, _v(rg.edf_unit_variant(L, [c["a"], c["a"] + 180.0]), self.prop))
        smin = float(L.p["propulsion_module"]["clock_keyways"]["sleeve_min"])
        ok = (rc["coverage"] >= 1.0 - 1e-12 and not rc["gaps"] and rc["worst_residual_deg"] <= L.lp_E_cov + 1e-9
              and seat_v < INTERSECT_TOL_MM3 and L.ck_min_tooth >= smin - 1e-9)
        return Check("C34", title, PASS if ok else FAIL,
                     f"coverage {100 * rc['coverage']:.3f} % of 0-360 deg (gaps {rc['gaps'] or 'none'}), worst residual "
                     f"{rc['worst_residual_deg']:.2f} deg <= E_cov {L.lp_E_cov:g} (passage takes E_design "
                     f"{L.lp_E_design:.2f} = E_cov + tab play {L.lp_tab_play_deg:.2f}); {len(L.ck_keyways)} keyways, "
                     f"{len(L.ck_configs)} seats ({len(L.ck_hover)} with one tab hovering in the passage), {rc['seats']} "
                     f"seat positions, largest gap {L.ck_largest_gap_deg:.2f} deg; EDF at every seat x module max "
                     f"{seat_v:.3f} mm^3; min sleeve tooth {L.ck_min_tooth:.2f} mm >= {smin:g}",
                     "keyways " + ", ".join(f"{a:.2f}" for a in L.ck_keyways) + "; hover seats "
                     + ", ".join(f"{c['a']:.2f}/{c['b']:.2f}" for c in L.ck_hover)
                     + f"; nominal (drawn) seat {L.tab_thetas[0]:.2f}/{L.tab_thetas[1]:.2f}",
                     {"recheck": rc, "layout_coverage": L.ck_coverage, "layout_worst_residual_deg": L.ck_worst_residual_deg,
                      "keyways": list(L.ck_keyways), "configs": L.ck_configs, "edf_seat_x_module_max": seat_v,
                      "min_tooth_mm": L.ck_min_tooth})

    def c35_axial_retention(self) -> Check:
        """wire09c: axial retention without tab screws (edf.axial_retention, A / E): foam compression inside its band for
        every stack case, keyway engagement after the largest lift >= RETENTION_ENGAGED_MIN, the shelf over the lip on the
        intake solid with the lip_stop_gap clear, shelf bending and keyway bearing factors >= RETENTION_FACTOR_MIN (full
        thrust / full-power torque on one tab); fix round (review F1): every sleeve tooth beside a keyway, measured on the
        solid, carries the design torque on ONE tab in tangential bending (fixed-fixed, uniform) with a factor >=
        RETENTION_FACTOR_MIN; the static-weight foam sag is reported (F4, A).  N/A without the clock mount."""
        L = self.L
        title = ("(wire09c clock mount) axial retention: lip on the intake shelf (up), foam pads under the tabs (down), "
                 "keyway walls and sleeve teeth (torque)")
        if not getattr(L, "clock_mount", False):
            return Check("C35", title, NA, "not applicable: no clock mount (the tab screws hold the EDF, C19)", "",
                         {"applicable": False})
        r = ckc.retention(L, self.shape["intake_module"])
        tb = ckc.teeth_bending(L, self.prop)
        r = dict(r, teeth_bending=tb)
        comp = r["foam_compression"]
        tol = L.p["edf"]["axial_retention"]["stack_tol_ASSUMED"]
        st = r["static_weight_A"]
        s2, s1 = st["2 pads (two-tab seat)"], st["1 pad (hover seat)"]
        wt = tb["worst"] or {}

        def _c(x: dict) -> str:
            return "bottoms (> 50 %)" if x["beyond_curve_bottoms"] else f"{100 * x['compression']:.0f} %"
        return Check("C35", title, PASS if (r["ok"] and tb["ok"]) else FAIL,
                     f"foam compression {100 * min(comp.values()):.0f}..{100 * max(comp.values()):.0f} % (band "
                     f"{100 * r['compression_band'][0]:.0f}..{100 * r['compression_band'][1]:.0f} %, stack +-{tol:g} mm A); "
                     f"largest lift {r['lift_max_mm']:.2f} mm, keyway engagement left {r['engaged_after_lift_mm']:.1f} mm "
                     f"(>= {ckc.RETENTION_ENGAGED_MIN:g}); shelf over the lip {100 * r['shelf_fill_over_lip']:.1f} % material, "
                     f"lip -> shelf gap clear ({r['lip_to_shelf_gap_material_mm3']:.3f} mm^3); shelf bending at "
                     f"{r['thrust_peak_N']:.1f} N thrust factor {r['shelf_factor']:.1f}, keyway bearing at "
                     f"{r['torque_design_Nm']:.2f} N m on one tab factor {r['bearing_factor']:.0f} (>= "
                     f"{ckc.RETENTION_FACTOR_MIN:g}); sleeve teeth (measured, {len(tb['teeth'])} pieces, min width "
                     f"{tb['w_min_mm']:.2f} mm): tangential bending at {tb['torque_design_Nm']:.2f} N m on ONE tab, "
                     f"fixed-fixed uniform, worst factor {tb['factor_min']:.1f} (tooth at {wt.get('mid_deg', math.nan):.2f} "
                     f"deg, w {wt.get('w_mm', math.nan):.2f} x t {wt.get('t_mm', math.nan):.2f}, span "
                     f"{wt.get('span_mm', math.nan):.1f} mm; >= {ckc.RETENTION_FACTOR_MIN:g}); foam preload "
                     f"{r['foam_preload_N_A']:.2f} N vs EDF weight {r['edf_weight_N']:.2f} N (A); at rest under the "
                     f"weight (A, reported): 2 pads {_c(s2)}, sag {s2['sag_below_drawn_mm']:.2f}, spin-up travel "
                     f"{s2['travel_to_shelf_mm']:.2f} mm; 1 pad (hover seat) {_c(s1)}, travel <= "
                     f"{s1['travel_to_shelf_mm']:.2f} mm",
                     "; ".join(f"{k} {100 * v:.0f} %" for k, v in comp.items())
                     + "; teeth " + ", ".join(f"{t_['mid_deg']:.1f}: {t_['w_mm']:.2f} mm f {t_['factor']:.1f}"
                                              for t_ in sorted(tb["teeth"], key=lambda q: q["mid_deg"])),
                     r)

    def c36_livery(self) -> Check:
        """livery (user decisions 2026-11-04): the orange YUCHAN inlay on the avionics module (check_livery): (a) the logo
        footprint grown by the clearance rule (3 mm) lies in the plain module's wall on every copy - no window, vent slot,
        hole, joint-screw hole or part end within 3 mm of any orange point - and (reviewfix F6) the BRep distance from
        each copy to every void of its skin band >= 3 mm with no volume tolerance, plus the BRep distance to each known
        feature; per-layer chords / white gaps reported (F7, information); (b) min stroke / white gap / island on the placed polygons >= livery.check; (c) inlay depth <= one
        outer line and wall - depth >= the 2 white lines; (d) white body + inlay = the plain module, inlay outer area =
        the polygons'.  N/A without a livery (params livery.applies_to)."""
        L = self.L
        title = "(livery) YUCHAN inlay: zone and 3 mm clearance, strokes / gaps / islands, depth, union = plain module"
        if not getattr(L, "has_livery", False):
            why = ("no params livery block" if not L.p.get("livery") else
                   f"livery.applies_to excludes config {L.config!r} / preset {L.p['meta']['preset']!r}")
            return Check("C36", title, NA, f"not applicable: {why}", "", {"applicable": False})
        r = ckl.livery_check(L, self.shape[L.livery_part], self.shape[L.livery_inlay_part])
        u, m, d = r["union"], r["polygons"], r["depth"]
        ck = L.livery_check
        dist = ", ".join(f"{k} {v:.2f}" for k, v in r["distances_mm"].items() if v is not None)
        cont = ", ".join(f"{k} deg {v['outside_material_mm3']:.4f} mm^3" for k, v in r["clearance_containment"].items())
        vb = r.get("clearance_void_band") or {}
        void = ", ".join(f"{t:g} deg {vb[f'{t:g}']['min_mm']:.3f}" for t in L.livery_thetas if f"{t:g}" in vb)
        lc = r.get("layer_chords") or {}
        return Check("C36", title, PASS if r["ok"] else FAIL,
                     f"{len(L.livery_thetas)} copies at {', '.join(f'{t:g}' for t in L.livery_thetas)} deg; footprint + "
                     f"{r['clearance_required_mm']:g} mm outside the wall material: {cont}; nearest void of the skin band "
                     f"(BRep, no volume tolerance) {void} mm (>= {r['clearance_required_mm']:g}); nearest feature "
                     f"{r['clearance_min_reported_mm']:.2f} mm; stroke {m['min_stroke_mm']:.2f} (>= {ck.get('min_stroke_mm')}), "
                     f"white gap {m['min_white_gap_mm']:.2f} (>= {ck.get('min_white_gap_mm')}), smallest island "
                     f"{m['smallest_island_mm2']:.2f} mm^2 (>= {ck.get('min_island_mm2')}); depth {d['design_mm']:g} "
                     f"(measured {d['measured_mm']:.3f}) <= line {d['line_w_mm']:g}, white behind {d['white_behind_mm']:.2f} "
                     f">= {d['white_behind_min_mm']:g}; white + inlay - plain {u['white_plus_inlay_minus_plain_mm3']:+.4f} "
                     f"mm^3, white x inlay {u['white_x_inlay_mm3']:.4f}, inlay outer area {100 * r['outer_area']['rel']:+.3f} %"
                     + (f"; FAILS: {', '.join(r['fails'])}" if r["fails"] else ""),
                     f"distances (BRep, inlay -> feature) {dist}; inlay {u['inlay_mm3']:.2f} mm^3, white body "
                     f"{u['white_mm3']:.2f}, plain {u['plain_mm3']:.2f}; logo {L.livery_file} ({L.livery_poly_set}, sha256 "
                     f"{L.livery_file_sha256[:12]}), {m['islands']} islands / copy, area {m['area_mm2_per_copy']:.1f} mm^2"
                     + (f"; per layer (information, layer {lc['layer_h_mm']:g} mm): {lc['layers_with_orange']} layers with "
                        f"orange, chords < one line {lc['chords_below_line_w']} (min {lc['min_chord_mm']:.3f} mm at print z "
                        f"{lc['min_chord_print_z_mm']}), white gaps < one line {lc['white_gaps_below_line_w']} (min "
                        f"{lc['min_white_gap_mm']} mm) - single-layer specks / bridged wedge tips expected there" if lc else ""),
                     r)


# --------------------------------------------------------------------------- #
def format_table(checks: list[Check]) -> str:
    rows = [f"{'id':<4} {'status':<6} {'measured':<110} check", "-" * 160]
    for c in checks:
        rows.append(f"{c.id:<4} {c.status:<6} {c.measured[:110]:<110} {c.title}")
    return "\n".join(rows)


def write_outputs(checks: list[Check], L: Layout, report: dict, paths: dict[str, str], runtime_s: float) -> None:
    summary = {s: sum(1 for c in checks if c.status == s and c.id.startswith("C")) for s in (PASS, WARN, FAIL, NA)}
    lines = [f"# CadQuery geometry checks - preset {L.p['meta']['preset']}"
             + (f" - config {L.config} (power {L.power_source})" if getattr(L, "vtol", False) else ""), "",
             f"params_sha256 `{report['params_sha256']}` - generator {report['generator_version']} - runtime {runtime_s:.1f} s", "",
             f"Summary (C-rows): {summary[PASS]} PASS, {summary[WARN]} WARN, {summary[FAIL]} FAIL, {summary[NA]} N/A "
             f"(edf.mount = {L.edf_mount}). No-intersection tolerance {INTERSECT_TOL_MM3:g} mm^3.", "",
             "| id | status | check | measured | detail |", "|---|---|---|---|---|"]
    for c in checks:
        esc = lambda s: s.replace("|", "\\|")  # noqa: E731
        lines.append(f"| {c.id} | {c.status} | {esc(c.title)} | {esc(c.measured)} | {esc(c.detail)} |")
    t = report["totals"]
    lines += ["", "## Key numbers", "",
              f"* total mass {t['mass_g']:.1f} g (printed {t['printed_mass_g']:.1f} g), COM z {t['com_mm'][2]:.1f} mm",
              f"* Ixx {t['inertia_about_com_kg_m2'][0][0]:.5f}, Iyy {t['inertia_about_com_kg_m2'][1][1]:.5f}, Izz {t['inertia_about_com_kg_m2'][2][2]:.5f} kg m^2",
              f"* post-ejection mass {t['post_ejection']['mass_g']:.1f} g, COM z {t['post_ejection']['com_mm'][2]:.1f} mm",
              f"* pushrod {report['derived']['pushrod_len_mm']:.2f} mm, nose cavity {report['derived']['nose_cavity_volume_L']:.3f} L, "
              f"footprint {report['derived']['footprint_d_mm']:.1f} mm, height {report['derived']['vehicle_h_mm']:.1f} mm", ""]
    with open(paths["checks_md"], "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines))
    br.write_json({
        "generator": "cadquery", "preset": L.p["meta"]["preset"], "params_sha256": report["params_sha256"],
        "tolerance_mm3": INTERSECT_TOL_MM3, "runtime_s": runtime_s, "summary": summary,
        "checks": [c.__dict__ for c in checks],
    }, paths["checks_json"])


def run(preset: str | None, export: bool = False, config: str | None = None, power: str | None = None
        ) -> tuple[list[Check], dict]:
    t0 = time.time()
    L, parts, report = br.build(preset, export=export, verbose=False, config=config, power=power)
    checks = Checker(L, parts, report).run()
    runtime = time.time() - t0
    paths = br.output_paths(preset, L.config, L.power_source)
    write_outputs(checks, L, report, paths, runtime)
    return checks, report


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--preset", default=None)
    ap.add_argument("--export", action="store_true", help="also export STL/STEP while building")
    ap.add_argument("--config", default=None, help="v0.9 CAD-2 hardware configuration (rocket | vtol)")
    ap.add_argument("--power", default=None, help="v0.9 CAD-2 VTOL power source (battery | tether | hybrid)")
    args = ap.parse_args(argv)
    t0 = time.time()
    checks, report = run(args.preset, export=args.export, config=args.config, power=args.power)
    print(format_table(checks))
    fails = [c for c in checks if c.status == FAIL and c.id.startswith("C")]
    warns = [c for c in checks if c.status == WARN]
    print(f"\n{len(checks)} rows: {len(fails)} FAIL, {len(warns)} WARN; runtime {time.time() - t0:.1f} s; "
          f"outputs {br.output_paths(args.preset, args.config, args.power)['checks_md']}")
    for c in warns + fails:
        print(f"  {c.id} {c.status}: {c.detail[:400]}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
