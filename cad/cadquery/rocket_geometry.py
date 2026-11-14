"""CadQuery twin of the CanSat EDF Rocket: every printed part and placeholder of SPEC sections 3-6.

Invariant enforced here: every dimension comes from ``Layout`` (cad/layout.py) and, through
it, from ``params.json``.  The only numeric literals in this module are the ones SPEC.md
states as literals itself (collected in the ``SPEC_LITERALS`` block below with their
section) plus pure modelling helpers (overlaps of +-1 mm so that unioned solids share
volume instead of a face, cutter over-travel).  Solids are built in global vehicle
coordinates (mm, +Z up along the thrust axis, Z = 0 at the exhaust exit plane, +X toward
vane 1, theta from +X toward +Y) and are never translated as occurrences (SPEC section 1).

Pod frame (SPEC 3.4): radial axis u, tangential axis v = +e_theta, vertical Z.  A pod-frame
solid is built with u along +X and v along +Y and then rotated about Z by theta.

Every public builder returns a ``cadquery.Shape`` (Solid, or Compound when a placeholder
consists of disjoint solids such as the EDF hub) - one function per component, named as
in SPEC.  ``build_all`` returns the ordered part registry used by build_rocket.py and
check_geometry.py.

v0.5 EDF tab mount (SPEC 3.7, 5.1, 6): every tab-mount feature (outer-skin cylinder, thin liner,
screw lugs, keyways, tab-screw holes, two-box wire slot, intake wire channel / shelf notch / bulge /
bulkhead hole, EDF tab plates and motor) is gated on ``Layout.tabs``.  With ``edf.mount = 'lip_clamp'``
(KAIST preset) the builders take exactly the v0.4 code path - same primitives, same order -
so that preset's solids are unchanged.

v0.5d (SPEC 3.2, 5.1, 5.2): the intake wire channel's void starts at wire_ch_z0 and the 45 deg floor wedge
(triangle wire_ch_floor_rz) is unioned back after the cuts (tabs only).  Both presets: the avionics module gets a
tray stop block in each rail slot, and the propulsion spigot ring's joint holes are blind (prop_joint_hole_depth
from the spigot OD).  Those two are the only v0.5d changes to the lip_clamp (KAIST) solids.

v0.6 (SPEC 5.3 / 6, two ATmega128A): when ``Layout.has_com_board``, payload_module gets four COM-board bosses under
its bulkhead (fused after the v0.5 primitives, so the v0.5 solid is the same with them removed) and a blind tap hole
in each (cut after the v0.5 cutters); the new placeholder ``com`` (the COM board box, module avionics_module,
mass electronics.com.mass_g) hangs from the boss tips.  Those are the only v0.6 changes to the solids (both presets).

v0.6 servo leads (SPEC 3.8 / 5.1 / 6, ``Layout.servo_leads``: tab-mount preset only, the KAIST solids do not change):
propulsion_module gets the pod lead windows, entry slots, deck notches and the spigot-ring rebate in its cut list;
intake_module gets the servo bulge (unioned before the cuts), the servo channel / shelf notch / bulkhead servo hole
(cut) and the servo floor wedge (unioned after the cuts); the lumped part ``servo_leads`` sits at Layout.sl_lead_com.
``servo_lead_legs`` / ``servo_bundle_legs`` are the C23 ribbon proxies.

v0.6.1 (SPEC 5.1, both presets - a printability fix like the v0.5d tray stop): the intake module's windowed band
(``intake_band``) is ``window_wall_t`` thick, thickened inward to ``Layout.r_band_in`` (outer surface = body OD), and the
windows are radial sectors (``intake_window_cutters``, ``Layout.window_sectors``) instead of two orthogonal through-slab
cuts, so every strut keeps strut_w at body_r with radial side faces.  That band and its windows are the only change to
the intake solid (the KAIST intake equals its v0.4 solid outside the band).  ``intake_window_area_mm2`` now measures the
minimum opening of the built solid with thin cylindrical probe shells across the band (``intake_opening_areas_mm2``).

v0.6.2 (both presets, user 2026-10-15): each window cutter is its v0.6.1 radial sector INTERSECT the gable prism
(``intake_window_gable_prism``: Layout.window_gable_profile_yz extruded along the window centre direction), so the
lower end of every window is the truncated gable - flat bridge at z_window_bottom, slope_deg faces to the struts - and
the band keeps the gussets (``intake_window_gussets``, the design region, for checks); the upper end stays square.  New
placeholder ``radio_log`` (Layout.radio_log_x/y/z, params mass, module avionics_module) hangs from the payload-bulkhead
underside beside the COM board; ``joint_screw_axis_proxies`` are C24's screw proxies.  Those are the only v0.6.2 changes.

v0.6.3 (both presets, three ATmega128A; user 2026-10-15): the placeholder ``fc`` is renamed ``nav`` (``electronics.nav``,
the NAV board: same Layout box, same tray holes - the tray solid is unchanged) and the new placeholder ``fcu``
(Layout.fcu_x/y/z, ``electronics.fcu.mass_g``, module avionics_module) lies flat on the intake bulkhead top on foam tape;
``lead_riser_proxies`` / ``bulkhead_hole_columns`` are C25's proxies.  No printed solid changes.

v0.8.1 (both presets, a fourth ATmega128A - the GNS board; architect 2026-10-21): when ``Layout.has_gns``, avionics_module
gets four bosses on the -x tube wall (``gns_bosses``: diamond (v, z) prisms, faces at 45 deg, from the board's back face
out to the OD, clipped there; unioned after the rails / tray stops) and a blind tap hole in each (``gns_hole_cutters``,
cut after the GPS entry hole), so the v0.8 solid is the same with them removed (``with_gns=False``); the new placeholder
``gns`` (Layout.gns_x/y/z, Layout.gns_mass_g = electronics.gns.mass_g_ASSUMED + the four screws) sits on the boss tips.
The GPS lead lump follows Layout (it now ends at the GNS board).  Those are the only v0.8.1 changes.

livery (user decisions 2026-11-04; ``Layout.has_livery``, params ``livery``: BUDGET rocket and VTOL, not KAIST): every printed
part's STEP colour is white (PRINTED_COLOR, Bambu PLA Matte Ivory White), the launch pad keeps the black (LAUNCH_PAD_COLOR);
the avionics module is the WHITE body - the plain module minus the 0.42 mm logo pocket (``livery_inlay_solids`` out past
the OD, cut last; ``avionics_module(with_livery=False)`` is the plain module) - and the new printed part ``avionics_logo``
(LIVERY_COLOR, Mandarin Orange) is the inlay that fills the pocket: per copy and island the pre-warped (v, z) outline
(``Layout.livery_polys_vz``) extruded along the copy's radial direction INTERSECT the annular sector r [body_r - depth,
body_r].  White + inlay = the plain module (check C36).  No other solid changes.
2026-11-05 logic power (Layout.has_logic_power, VTOL battery build): ``top_cap`` cuts the logic switch hole (a plate hole of
Layout.cap_holes; ``with_logic_hole=False`` gives the hole-less cap, the flight-mass model's ``Part.mass_shape`` while
logic_power.in_flight_mass_model is false); ``logic_power_parts`` holds the placeholders ``logic_pack`` / ``logic_switch``
and the lumped ``logic_switch_lead`` / ``logic_retention`` - in ``build_all`` only once the mass switch is true;
``logic_lead_keepouts`` / ``carrier_plug_boxes`` are check helpers (C37-C39).
Switch-collar round (user decision 2026-11-05, Layout.has_lp_collar): ``top_cap`` also fuses the protective collar
(``logic_collar``: tube round the switch with two finger notches on the throw axis) - one piece with the cap; it is part
of the logic-power kit, so ``with_logic_hole=False`` leaves out the collar too (``with_logic_collar`` overrides) and the
flight-mass shape stays the cap of the previous round, bit for bit.  ``logic_lever_sweep`` is a check helper (C39 / C41).
"""
from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass
from typing import Callable, Iterable, Sequence

import cadquery as cq

_HERE = os.path.dirname(os.path.abspath(__file__))
_CAD_DIR = os.path.normpath(os.path.join(_HERE, ".."))
if _CAD_DIR not in sys.path:
    sys.path.insert(0, _CAD_DIR)

from layout import Layout, build as build_layout  # noqa: E402

V = cq.Vector

# --------------------------------------------------------------------------- #
# SPEC_LITERALS - numbers that SPEC.md states verbatim (not in params.json)
# --------------------------------------------------------------------------- #
EAR_HOLE_DEPTH = 6.0            # SPEC 3.4 / 5.3: servo ear screw holes "6 mm deep" from the rim top
HORN_PIN_BELOW_TIP = 2.0        # SPEC 4.2 / 4.5: pushrod pin hole at Z = horn_tip_z - 2
NOSE_SOLID_TIP_LEN = 2.0        # SPEC 5.4: inner (normal-offset) profile stops 2 mm short of the tip (matches Fusion shell)
EDF_HUB_LEN = 70.0              # SPEC 6: hub cylinder from z_edf_housing_bottom to +70
MISC_BOX = (40.0, 40.0, 20.0)   # SPEC 6: misc-electronics box 40 x 40 x 20 (x, y, z)
MISC_Z_ABOVE_FC = (4.0, 24.0)   # SPEC 6: misc box Z = fc_z[1] + 4 .. + 24
LATCH_HUB_D = 6.0               # SPEC 5.3: latch arm "with a O6 hub at the servo axis"
BULKHEAD_HOLE_THETA0 = 45.0     # SPEC 5.3: payload bulkhead holes at theta = 45 + k*90
VANE_PLACEHOLDER_MASS_G = 1.0   # SPEC 6: servo arms and pushrods "1 g each"
SPRING_MASS_FRACTION = 0.5      # SPEC 6: spring = 0.5 x hardware_mass_g.spring_piston_hardware
WIRE_BULGE_INTO_WALL = 0.6      # SPEC 5.1: wire bulge r in [body_r - 0.6, wire_bulge_r_out]
WIRE_NOTCH_PAST_CH_R0 = 0.5     # SPEC 5.1: shelf notch r in [wire_notch_r0, wire_ch_r0 + 0.5]
WIRE_BUNDLE_W, WIRE_BUNDLE_T = 12.0, 5.0   # SPEC 7 C20: 12 x 5 mm proxy bundle (tangential x in the r-z plane)
OVERLAP = 1.0                   # modelling helper: union overlap / cutter over-travel (mm)
EDF_TAB_INTO_HOUSING = 0.5      # modelling helper: EDF tab plate starts 0.5 mm inside the housing wall (r tab_r_in - 0.5)
                                # so the placeholder fuses into one solid (a face contact would leave two)
NOSE_TIP_FLAT_R = 0.1           # modelling helper: the Haack apex ends on a O0.2 mm flat instead of a degenerate
                                # point (a degenerate revolve apex meshes into a non-watertight STL); tip Z unchanged
JOINT_PROBE_INSET = 0.2         # check helper (C16): the probe of a blind joint hole starts 0.2 mm past its flat bottom
INLET_PROBE_T = 0.1             # check helper (C9, v0.6.1): cylindrical probe shells 0.1 mm thick measure the opening ...
INLET_PROBE_GAP = 0.01          # ... starting 0.01 mm off the band faces (no coincident cylinder faces in the booleans)
INLET_PROBE_COUNT = 5           # ... at 5 radii across the band (the innermost gives the minimum for radial sides)
SERVO_PROXY_GAP = 0.2           # SPEC 7 C23: the lead proxies sit 0.2 mm off the walls they follow (pod face, rebate)


# --------------------------------------------------------------------------- #
# primitive helpers (all in global coordinates)
# --------------------------------------------------------------------------- #
def _revolve(pts_rz: Sequence[tuple[float, float]]) -> cq.Shape:
    """Revolve a closed (r, z) polygon 360 deg about the vehicle axis."""
    wp = cq.Workplane("XZ").polyline(list(pts_rz)).close().revolve(360.0, (0, 0, 0), (0, 1, 0))
    return wp.val()


def _tube(r_out: float, r_in: float, z0: float, z1: float) -> cq.Shape:
    if r_in <= 0.0:
        return _cyl_z(r_out, z0, z1)
    return _revolve([(r_in, z0), (r_out, z0), (r_out, z1), (r_in, z1)])


def _cyl_z(r: float, z0: float, z1: float, x: float = 0.0, y: float = 0.0) -> cq.Shape:
    return cq.Solid.makeCylinder(r, z1 - z0, V(x, y, z0), V(0, 0, 1))


def spigot_shoulder_t(L: Layout) -> float:
    """Thickness of the internal ring that ties a male spigot to its tube.

    SPEC 2 makes the spigot OD = body ID - 0.4, so a spigot ring standing on a plain tube
    shares no face with it (0.2 mm radial gap).  SPEC only names the resulting shoulder
    (the annulus spigot_od..body_od at the spigot bottom); the connecting ring's thickness
    is not dimensioned, so the body wall thickness is used (recorded as an ambiguity).
    """
    return L.body_wall


def spigot_shoulder_r_in(L: Layout) -> float:
    """Inner radius of the tube-side shoulder ring: it overlaps the outer half of the spigot
    wall (spigot_od/2 - spigot_wall/2), which connects the two solids by a face while
    narrowing the tube bore as little as possible below the spigot."""
    return L.spigot_od / 2 - L.p["body"]["spigot_wall"] / 2


def _spigot_shoulder(L: Layout, z_spigot_bottom: float, r_in: float) -> cq.Shape:
    """Ring r in [r_in, body_r], Z in [spigot bottom - shoulder_t, spigot bottom]."""
    t = spigot_shoulder_t(L)
    return _tube(L.body_r, r_in, z_spigot_bottom - t, z_spigot_bottom)


def _box(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float) -> cq.Shape:
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def _rot_z(shape: cq.Shape, theta_deg: float) -> cq.Shape:
    if theta_deg == 0.0:
        return shape
    return shape.rotate(V(0, 0, 0), V(0, 0, 1), theta_deg)


def _box_polar(u0: float, u1: float, v0: float, v1: float, z0: float, z1: float, theta: float) -> cq.Shape:
    """Box in the pod frame (u radial, v tangential) at angle theta."""
    return _rot_z(_box(u0, u1, v0, v1, z0, z1), theta)


def _radial_cyl(d: float, r0: float, r1: float, theta: float, z: float, v: float = 0.0) -> cq.Shape:
    """Cylinder of diameter d whose axis is the radial direction at theta, from r0 to r1."""
    c = cq.Solid.makeCylinder(d / 2.0, r1 - r0, V(r0, v, z), V(1, 0, 0))
    return _rot_z(c, theta)


def _prism_uz(pts_uz: Sequence[tuple[float, float]], v0: float, v1: float, theta: float = 0.0) -> cq.Shape:
    """Polygon in the (u, z) plane extruded over v in [v0, v1], then rotated to theta."""
    wp = cq.Workplane("XZ").workplane(offset=-v1).polyline(list(pts_uz)).close().extrude(v1 - v0)
    return _rot_z(wp.val(), theta)


def _polar(u: float, v: float, theta_deg: float, z: float) -> V:
    t = math.radians(theta_deg)
    return V(u * math.cos(t) - v * math.sin(t), u * math.sin(t) + v * math.cos(t), z)


def _sector(r0: float, r1: float, th0: float, th1: float, z0: float, z1: float) -> cq.Shape:
    """Annular sector r in [r0, r1], theta in [th0, th1] deg (th0 < th1), Z in [z0, z1].

    A positive revolve of the (r, z) rectangle on the XZ workplane sweeps from +X toward +Y
    (measured: a 30 deg revolve spans y 0..30), so the sector is revolved by th1 - th0 and
    rotated to start at th0."""
    wp = cq.Workplane("XZ").polyline([(r0, z0), (r1, z0), (r1, z1), (r0, z1)]).close()
    return _rot_z(wp.revolve(th1 - th0, (0, 0, 0), (0, 1, 0)).val(), th0)


def _screw_axis_cyl(L: Layout, tab_theta: float, z: float, d: float, t0: float, t1: float) -> cq.Shape:
    """Cylinder of diameter d on the tangential tab-screw axis (Layout.tab_screw_point) for t in [t0, t1]."""
    p0 = V(*L.tab_screw_point(tab_theta, z, t0))
    p1 = V(*L.tab_screw_point(tab_theta, z, t1))
    return cq.Solid.makeCylinder(d / 2.0, t1 - t0, p0, (p1 - p0).normalized())


def _fuse(shapes: Sequence[cq.Shape], clean: bool = True) -> cq.Shape:
    shapes = list(shapes)
    if len(shapes) == 1:
        return shapes[0]
    out = shapes[0].fuse(*shapes[1:])
    return out.clean() if clean else out


def _cut(shape: cq.Shape, cutters: Sequence[cq.Shape]) -> cq.Shape:
    cutters = list(cutters)
    if not cutters:
        return shape
    return shape.cut(*cutters)


def intersection_volume(a: cq.Shape, b: cq.Shape) -> float:
    """Boolean-intersection volume in mm^3 (0.0 when the solids are disjoint).

    Returns NaN when OCC cannot compute it: no evidence is never a PASS, so callers must treat NaN as a
    violation (check_geometry._v maps it to +inf for "no intersection" tests and to 0 for "material present")."""
    try:
        common = a.intersect(b)
    except Exception:  # OCC raises on some degenerate touching cases
        return math.nan
    if common is None or common.wrapped.IsNull():
        return 0.0
    try:
        return float(common.Volume())
    except Exception:
        return math.nan


# --------------------------------------------------------------------------- #
# SPEC 3 - propulsion module
# --------------------------------------------------------------------------- #
def _cone_r_out(L: Layout, z: float) -> float:
    """Outer radius of the EDF section (Layout.cone_outer_radius: 45 deg chamfer + linear cone)."""
    return L.cone_outer_radius(z)


def _ear_hole_cutters(L: Layout, theta: float, axis_r: float, rim_z: float) -> list[cq.Shape]:
    s = L.p["servo"]
    out = []
    for sign in (+1.0, -1.0):
        c = _cyl_z(s["ear_hole_d"] / 2.0, rim_z - EAR_HOLE_DEPTH, rim_z + OVERLAP,
                   axis_r, sign * s["ear_hole_pitch"] / 2.0)
        out.append(_rot_z(c, theta))
    return out


def _pod_cup(L: Layout, *, theta: float, wall_r: float, r_in: float, r_out: float,
             cav_r_in: float, cav_r_out: float, z0: float, floor_top_z: float, rim_z: float) -> cq.Shape:
    """Rectangular servo cup of SPEC 3.4 (also the release pod of 5.3), open at the top.

    The 45 deg underside chamfer is the wedge cut SPEC describes: the cup is filled down to
    the point where the chamfer line meets the tube wall (clamped at Z = 0) and the region
    below the line through (wall_r, z0 - (r_out - wall_r)) and (r_out, z0) is removed.
    Returned before the union with its tube and before the bore re-cut.
    """
    pod_w, cav_w = L.pod_w, L.pod_cavity_w
    slope = math.tan(math.radians(L.p["pod"]["underside_chamfer_deg"]))
    z_a = z0 - (r_out - wall_r) * slope
    z_floor = max(L.z_exit, z_a)
    box = _box(r_in, r_out, -pod_w / 2, pod_w / 2, z_floor, rim_z)
    u_a = r_in - OVERLAP
    z_ua = z_a - (wall_r - u_a) * slope
    u_b = r_out + 2 * OVERLAP
    z_ub = z0 + 2 * OVERLAP * slope
    z_low = min(z_ua, z_floor) - OVERLAP
    wedge = _prism_uz([(u_a, z_ua), (u_b, z_ub), (u_b, z_low), (u_a, z_low)],
                      -pod_w / 2 - OVERLAP, pod_w / 2 + OVERLAP)
    cup = box.cut(wedge)
    cavity = _box(cav_r_in, cav_r_out, -cav_w / 2, cav_w / 2, floor_top_z, rim_z + OVERLAP)
    cup = cup.cut(cavity)
    return _rot_z(cup, theta)


def pod_z_floor(L: Layout, z0: float, r_out: float, wall_r: float) -> float:
    """Lowest Z of a pod cup (chamfer start, clamped at the exit plane)."""
    slope = math.tan(math.radians(L.p["pod"]["underside_chamfer_deg"]))
    return max(L.z_exit, z0 - (r_out - wall_r) * slope)


def servo_pod(L: Layout, i: int = 0) -> cq.Shape:
    """Vane-servo cup i (SPEC 3.4) alone, in global coordinates."""
    return _pod_cup(L, theta=L.pod_thetas[i], wall_r=L.vane_section_r, r_in=L.pod_r_in, r_out=L.pod_r_out,
                    cav_r_in=L.pod_cavity_r_in, cav_r_out=L.pod_cavity_r_out, z0=L.pod_z0,
                    floor_top_z=L.pod_floor_top_z, rim_z=L.pod_rim_z)


def fin(L: Layout, theta: float) -> cq.Shape:
    """One fin plate (SPEC 3.5) in the plane containing the axis at theta."""
    planform = [(L.fin_root_r, L.fin_te_z), (L.fin_root_r, L.fin_root_le_z),
                (L.fin_tip_r, L.fin_tip_le_z), (L.fin_tip_r, L.fin_te_z)]
    return _prism_uz(planform, -L.fin_t / 2, L.fin_t / 2, theta)


def _skin_outer_points(L: Layout) -> list[tuple[float, float]]:
    """Outer-skin (r, z) polyline from the cone start up: Layout.outer_skin_profile() without the
    chamfer's lower vertex (the chamfer lies inside the solid shoulder ring, SPEC 3.2)."""
    return [(r, z) for r, z in L.outer_skin_profile() if z >= L.cone_z0 - 1e-9]


def outer_skin(L: Layout) -> cq.Shape:
    """SPEC 3.2 / 3.7 outer skin: linear cone cone_z0..cone_z1 and (tabs) the cylinder r body_r
    cone_z1..z_prop_spigot_bottom, wall cone_wall measured radially inward.  For lip_clamp the
    polygon is exactly the v0.4 trapezoid (the profile has no cylinder point)."""
    outer = _skin_outer_points(L)
    inner = [(r - L.cone_wall, z) for r, z in reversed(outer)]
    return _revolve(outer + inner)


def skin_filled(L: Layout) -> cq.Shape:
    """Filled solid bounded by the outer skin (cone_z0 .. top of the skin), used to clip the webs."""
    if not L.tabs:          # v0.4 path (kept literally so the KAIST solids do not change)
        return cq.Solid.makeCone(L.cone_r_bottom, L.cone_r_top, L.cone_z1 - L.cone_z0,
                                 V(0, 0, L.cone_z0), V(0, 0, 1))
    outer = _skin_outer_points(L)
    return _revolve([(0.0, outer[0][1])] + outer + [(0.0, outer[-1][1])])


def liner(L: Layout) -> cq.Shape:
    """SPEC 3.7 thin liner (tabs only): duct tube r [duct_r, liner_r_out] over [z_edf_shoulder, z_liner_top]
    plus the top ring r [duct_r, sleeve_r] over [z_liner_ring_bottom, z_liner_top], revolved as one L-shaped
    profile.  The ring overlaps the sleeve wall, so the cavity behind the tube is closed."""
    return _revolve([(L.duct_r, L.z_edf_shoulder), (L.liner_r_out, L.z_edf_shoulder),
                     (L.liner_r_out, L.z_liner_ring_bottom), (L.sleeve_r, L.z_liner_ring_bottom),
                     (L.sleeve_r, L.z_liner_top), (L.duct_r, L.z_liner_top)])


def tab_boss_thetas(L: Layout, tab_theta: float) -> tuple[float, float]:
    """Angular extent (lo, hi) of the screw lugs: [th - hs*block_theta_far, th + hs*block_theta_head]
    sorted (SPEC 3.7; hs = tab_screw_head_side, +1 -> head side at +theta)."""
    a = tab_theta - L.tab_screw_head_side * L.block_theta_far
    b = tab_theta + L.tab_screw_head_side * L.block_theta_head
    return min(a, b), max(a, b)


def tab_screw_lugs(L: Layout, tab_theta: float) -> list[cq.Shape]:
    """SPEC 3.7 screw lugs of one tab: one angular sector per entry (z0, z1) of boss_z_ranges, r
    [block_r_in - OVERLAP, skin inner + OVERLAP] (the overlaps make each lug share volume with the sleeve and
    the skin).  The lowest lug reaches floor_t below keyway_z0 (keyway floor), the highest merges with the deck."""
    lo, hi = tab_boss_thetas(L, tab_theta)
    out = []
    for z0, z1 in L.boss_z_ranges:
        r_skin_in = L.cone_outer_radius(z0) - L.cone_wall      # cylindrical section (C21: block_z0 >= z_cyl0)
        out.append(_sector(L.block_r_in - OVERLAP, r_skin_in + OVERLAP, lo, hi, z0, z1))
    return out


def tab_keyway(L: Layout, tab_theta: float) -> cq.Shape:
    """SPEC 3.7 keyway cutter: tangential +-keyway_w/2, r [keyway_r_in, keyway_r_out],
    Z [keyway_z0, keyway_z1] (keyway_z1 is above the spigot top: open at the top)."""
    return _box_polar(L.keyway_r_in, L.keyway_r_out, -L.keyway_w / 2, L.keyway_w / 2,
                      L.keyway_z0, L.keyway_z1, tab_theta)


def tab_screw_cutters(L: Layout, tab_theta: float, z: float) -> list[cq.Shape]:
    """SPEC 3.7 cutters of one tab screw along its tangential axis: clearance hole O d_clear for
    t in [t_keyway, t_head], counterbore O d_cbore for t in [t_head, t_cbore_end] (its edge nearest the axis
    leaves the skin before t_cbore_end, so the head enters straight from outside), tap hole O d_tap for
    t in [t_tap_end, -t_keyway].  The clearance and tap holes over-travel OVERLAP into the counterbore /
    keyway voids (no material there), so no cutter ends on a face."""
    return [
        _screw_axis_cyl(L, tab_theta, z, L.tab_screw_d_clear, L.tab_screw_t_keyway - OVERLAP,
                        L.tab_screw_t_head + OVERLAP),
        _screw_axis_cyl(L, tab_theta, z, L.tab_screw_d_cbore, L.tab_screw_t_head, L.tab_screw_t_cbore_end),
        _screw_axis_cyl(L, tab_theta, z, L.tab_screw_d_tap, L.tab_screw_t_tap_end,
                        -L.tab_screw_t_keyway + OVERLAP),
    ]


def wire_slot_boxes(L: Layout) -> list[cq.Shape]:
    """SPEC 3.7 wire-slot cutters at wire_theta, tangential +-wire_slot_w/2, r from wire_slot_r_in:
    (a) r <= wire_slot_lower_r_out for Z [wire_slot_z0, z_prop_spigot_bottom] (sleeve and deck; the corners
    keep skin_min_wall to the skin), (b) r <= wire_slot_r_out for Z [z_prop_spigot_bottom, wire_slot_z1]
    (through the spigot ring, open at the top and to the skirt)."""
    hw = L.wire_slot_w / 2
    out = [_box_polar(L.wire_slot_r_in, L.wire_slot_lower_r_out, -hw, hw, L.wire_slot_z0, L.z_prop_spigot_bottom,
                      L.wire_theta),
           _box_polar(L.wire_slot_r_in, L.wire_slot_r_out, -hw, hw, L.z_prop_spigot_bottom, L.wire_slot_z1,
                      L.wire_theta)]
    if getattr(L, "ml_low_cut_uz", None):       # v0.9 lead-exit round: notch under the housing rim + lower slot
        out.append(wire_low_cut(L))
    return out


def wire_low_cut(L: Layout) -> cq.Shape:
    """v0.9 lead-exit round (user 2026-10-29): the motor-lead notch under the housing's rear rim and the lower slot beside
    the housing - the (u, z) polygon Layout.ml_low_cut_uz at wire_theta, +-ml_low_cut_w / 2 tangential (from duct_r - 0.5,
    which opens the exhaust bore, out to the leads' envelope + margin with a 45 deg chamfer along the outer bend; its top
    over-travels 0.5 into the 22 mm box)."""
    hw = L.ml_low_cut_w / 2
    return _prism_uz(L.ml_low_cut_uz, -hw, hw, L.wire_theta)


def wire_slot(L: Layout) -> cq.Shape:
    """Union of the two wire-slot boxes (check / display helper; the module cuts the boxes separately)."""
    return _fuse(wire_slot_boxes(L))


def prop_joint_hole_r0(L: Layout) -> float | None:
    """SPEC 3.2 v0.5d: radius of the flat bottom of the blind propulsion spigot joint holes,
    spigot_od/2 - prop_joint_hole_depth; None when the hole goes through to the EDF bore (no depth given, or a
    depth that reaches the bore)."""
    if not L.prop_joint_hole_depth:
        return None
    r0 = L.spigot_od / 2 - L.prop_joint_hole_depth
    if r0 <= L.edf_bore_r:
        # SPEC 3.2 requires blind holes when a depth is given; never silently fall back to a through hole
        raise ValueError(f"prop_joint_hole_depth {L.prop_joint_hole_depth} reaches the EDF bore (r0 {r0:.2f} <= "
                         f"{L.edf_bore_r:.2f})")
    return r0


def prop_joint_hole_cutter(L: Layout, theta: float) -> cq.Shape:
    """SPEC 3.2 joint hole of the propulsion spigot ring: O joint_hole_spigot_d radial cylinder at Z
    z_prop_spigot_bottom + joint_screw_dz from prop_joint_hole_r0 (blind; v0.5d) - or, through, from inside the
    EDF bore - out past the spigot OD."""
    r0 = prop_joint_hole_r0(L)
    return _radial_cyl(L.p["body"]["joint_hole_spigot_d"], L.edf_bore_r - 2 * OVERLAP if r0 is None else r0,
                       L.spigot_od / 2 + 2 * OVERLAP, theta, L.z_prop_spigot_bottom + L.joint_screw_dz)


def servo_lead_window(L: Layout, i: int) -> cq.Shape:
    """SPEC 3.8 lead window through the far end wall of vane-servo pod i: u [sl_win_u0, sl_win_u1], v from the cavity
    side of the wall (over-travel OVERLAP into the cavity) out OVERLAP past the pod face, Z [sl_win_z0, sl_win_z1]."""
    a, b = L.pod_cavity_w / 2 - OVERLAP, L.pod_w / 2 + OVERLAP
    v0, v1 = (a, b) if L.sl_side > 0 else (-b, -a)
    return _box_polar(L.sl_win_u0, L.sl_win_u1, v0, v1, L.sl_win_z0, L.sl_win_z1, L.pod_thetas[i])


def servo_lead_entry_slot(L: Layout, k: int) -> cq.Shape:
    """SPEC 3.8 entry slot of lead k: vertical box at sl_col_thetas[k], r [sl_entry_u0, sl_entry_u1], tangential
    +-sl_entry_w/2, Z [sl_entry_z0, sl_entry_z_open + OVERLAP] (through the shoulder ring and the cone skin; above
    sl_entry_z_open it runs in the annulus void)."""
    return _box_polar(L.sl_entry_u0, L.sl_entry_u1, -L.sl_entry_w / 2, L.sl_entry_w / 2, L.sl_entry_z0,
                      L.sl_entry_z_open + OVERLAP, L.sl_col_thetas[k])


def servo_lead_deck_notch(L: Layout, k: int) -> list[cq.Shape]:
    """SPEC 3.8 deck notch of lead k at sl_col_thetas[k], tangential +-sl_notch_w/2: (a) r [sl_notch_u0, spigot_od/2]
    through the deck (from OVERLAP below it in the annulus to OVERLAP up into (b)) - the deck outside it stays the
    intake skirt's seat; (b) r [sl_notch_u0, spigot_od/2 + OVERLAP] up the spigot ring from z_prop_spigot_bottom to
    OVERLAP into the rebate (open to the skirt)."""
    th, hw = L.sl_col_thetas[k], L.sl_notch_w / 2
    zsb, so = L.z_prop_spigot_bottom, L.spigot_od / 2
    return [_box_polar(L.sl_notch_u0, so, -hw, hw, zsb - L.body_wall - OVERLAP, zsb + OVERLAP, th),
            _box_polar(L.sl_notch_u0, so + OVERLAP, -hw, hw, zsb, L.sl_rebate_z0 + OVERLAP, th)]


def servo_lead_rebate(L: Layout) -> cq.Shape:
    """SPEC 3.8 spigot-ring rebate: annular sector r [sl_rebate_r0, spigot_od/2 + OVERLAP], theta [sl_rebate_th0,
    sl_rebate_th1] (through the gather theta, never the motor wire slot), Z [sl_rebate_z0, z_prop_spigot_top + OVERLAP]."""
    return _sector(L.sl_rebate_r0, L.spigot_od / 2 + OVERLAP, L.sl_rebate_th0, L.sl_rebate_th1, L.sl_rebate_z0,
                   L.z_prop_spigot_top + OVERLAP)


def _cyl_between(d: float, a: Sequence[float], b: Sequence[float], ext0: float = 0.0, ext1: float = 0.0) -> cq.Shape:
    """Cylinder of diameter d on the segment a -> b, extended ext0 before a and ext1 past b along the axis."""
    pa, pb = V(*a), V(*b)
    u = (pb - pa).normalized()
    p0 = pa - u * ext0
    return cq.Solid.makeCylinder(d / 2.0, (pb - pa).Length + ext0 + ext1, p0, u)


# ---- v0.9 direct-drive vanes (vane_drive.mode 'direct', SPEC 3.4 v0.9): fin-root fairing, pocket, cover, lead pipe ----
def fairing_box(L: Layout, k: int) -> cq.Shape:
    """Fairing body of fin k: box in the fin frame u fair_u, v +-fair_hv, Z fair_z (the inner end overlaps the duct
    wall: fair_u[0] = fin_root_r)."""
    return _box_polar(L.fair_u[0], L.fair_u[1], -L.fair_hv, L.fair_hv, L.fair_z[0], L.fair_z[1], L.fin_thetas[k])


def fairing_keel(L: Layout, k: int) -> cq.Shape:
    """45 deg keel under fairing k: the (v, z) polygon Layout.fair_keel_vz extruded radially over fair_u (the fin plate
    is its bottom edge), so the fairing prints upright without support underneath."""
    return _prism_vz(L.fair_keel_vz, L.fair_u[0], L.fair_u[1], L.fin_thetas[k])


def servo_pocket_cutters(L: Layout, k: int) -> list[cq.Shape]:
    """Pocket of fairing k (void): u dd_pocket_u, v from -dd_pocket_hv out through the service face (open side),
    Z dd_pocket_z; plus the lower ear tab's floor slot (u dd_slot_u, Z dd_slot_z), also open to the service side."""
    th, ss, hv = L.fin_thetas[k], L.dd_service_side, L.dd_pocket_hv
    v0, v1 = (-hv, L.fair_hv + OVERLAP) if ss > 0 else (-L.fair_hv - OVERLAP, hv)
    # pocket (closed at the top), the service opening under the lintel, the lower ear tab's slot (open to the service side)
    return [_box_polar(L.dd_pocket_u[0], L.dd_pocket_u[1], -hv, hv, L.dd_pocket_z[0], L.dd_pocket_z[1], th),
            _box_polar(L.dd_pocket_u[0], L.dd_pocket_u[1], v0, v1, L.dd_pocket_z[0], L.dd_open_z1, th),
            _box_polar(L.dd_slot_u[0], L.dd_slot_u[1], v0, v1, L.dd_slot_z[0], L.dd_pocket_z[0] + OVERLAP, th)]


def cover_screw_axes(L: Layout, k: int) -> list[tuple[tuple[float, float, float], tuple[float, float, float]]]:
    """Axis of each servo-cover screw of fin k, from the service face (head side) into the fairing (tap end)."""
    ss = L.dd_service_side
    return [(L.fin_xyz(k, u, ss * L.fair_hv, z), L.fin_xyz(k, u, ss * (L.fair_hv - L.cov_tap_depth), z))
            for u, z in L.cov_screws_uz]


def cover_tap_cutters(L: Layout, k: int) -> list[cq.Shape]:
    """Blind tap holes (O cov_d_tap, cov_tap_depth deep) of the two cover screws of fairing k, along -v from the
    service face (over-travel OVERLAP out of it)."""
    return [_cyl_between(L.cov_d_tap, a, b, ext0=OVERLAP) for a, b in cover_screw_axes(L, k)]


def lead_pipe(L: Layout, k: int) -> list[cq.Shape]:
    """Lead pipe of fin k (solid): O sl_pipe_d on Layout.sl_pipe_points plus the web pipe (sl_web_points) that ties it
    to the duct wall (unioned with the module before the pocket / tunnel cuts)."""
    a, b = L.sl_pipe_points(k)
    wa, wb = L.sl_web_points(k)
    return [_cyl_between(L.sl_pipe_d, a, b), _cyl_between(L.sl_pipe_d, wa, wb)]


def lead_tunnel(L: Layout, k: int) -> cq.Shape:
    """Lead tunnel of fin k (void): O sl_tunnel_d on sl_pipe_points, from 3 mm inside the pocket to past Pe (the bottom
    of the entry slot)."""
    a, b = L.sl_pipe_points(k)
    return _cyl_between(L.sl_tunnel_d, a, b, ext0=3.0, ext1=OVERLAP)


def servo_cover(L: Layout, k: int = 0) -> cq.Shape:
    """Printed servo_cover of fin k (v0.9): plate cov_u x cov_v x cov_z on the fairing's service face with the two
    O cov_d_clear screw holes on the cover-screw axes."""
    th = L.fin_thetas[k]
    plate = _box_polar(L.cov_u[0], L.cov_u[1], L.cov_v[0], L.cov_v[1], L.cov_z[0], L.cov_z[1], th)
    holes = []
    for u, z in L.cov_screws_uz:
        a = L.fin_xyz(k, u, L.cov_v[0] - OVERLAP, z)
        b = L.fin_xyz(k, u, L.cov_v[1] + OVERLAP, z)
        holes.append(_cyl_between(L.cov_d_clear, a, b))
    return plate.cut(*holes)


def vane_coupler(L: Layout, k: int = 0, deflection_deg: float = 0.0) -> cq.Shape:
    """Printed vane_coupler of vane k (v0.9): hub O dd_hub_d (dd_u_hub0 .. dd_u_hub1) + flange O dd_flange_d (..
    dd_u_flange1) on the hinge axis, with the D-bore (O dd_bore_d, flat dd_bore_flat below the axis on the -Z side, as
    the shaft's flat) through the hub; rotated with the vane by ``deflection_deg``."""
    zh = L.vane_hinge_z
    hub = cq.Solid.makeCylinder(L.dd_hub_d / 2, L.dd_u_hub1 - L.dd_u_hub0, V(L.dd_u_hub0, 0, zh), V(1, 0, 0))
    fl = cq.Solid.makeCylinder(L.dd_flange_d / 2, L.dd_u_flange1 - L.dd_u_hub1, V(L.dd_u_hub1, 0, zh), V(1, 0, 0))
    body = hub.fuse(fl).clean()
    # the D-bore runs through hub and flange (as the Fusion twin's through_circle)
    bore = cq.Solid.makeCylinder(L.dd_bore_d / 2, L.dd_u_flange1 - L.dd_u_hub0 + 2 * OVERLAP, V(L.dd_u_hub0 - OVERLAP, 0, zh),
                                 V(1, 0, 0))
    if getattr(L, "dd_flat_side", "-z") == "+v":       # v0.9 fix round: the flat on the +v side, only as far as the shaft
        u_f = L.dd_u_flat_end                            # reaches (+0.3) - beyond it the bore is round (horn screw head)
        keep = _box(L.dd_u_hub0 - 2 * OVERLAP, u_f, -L.dd_bore_d, L.dd_bore_flat, zh - L.dd_bore_d, zh + L.dd_bore_d)
        bore_round = cq.Solid.makeCylinder(L.dd_bore_d / 2, L.dd_u_flange1 - u_f + OVERLAP, V(u_f, 0, zh), V(1, 0, 0))
        cut = [bore.intersect(keep), bore_round]
    else:
        keep = _box(L.dd_u_hub0 - 2 * OVERLAP, L.dd_u_flange1 + 2 * OVERLAP, -L.dd_bore_d, L.dd_bore_d, zh - L.dd_bore_flat,
                    zh + L.dd_bore_d)
        cut = [bore.intersect(keep)]
    if hasattr(L, "dd_slot_hw"):
        # v0.9 fix round: horn socket - the slot across the flange for the horn arms (along v at neutral, open at both
        # ends), from the hub's end face to the socket end; the relief for the horn's spline collar beyond the arm plane;
        # the set screw's tap hole along -v from the hub surface onto the flat
        cut.append(_box(L.dd_u_hub1, L.dd_u_flange1 + OVERLAP, -L.dd_flange_d, L.dd_flange_d,
                        zh - L.dd_slot_hw, zh + L.dd_slot_hw))
        cut.append(cq.Solid.makeCylinder(L.dd_collar_relief_d / 2, L.dd_u_flange1 - L.dd_u_hub1 + OVERLAP,
                                         V(L.dd_u_hub1, 0, zh), V(1, 0, 0)))
        cut.append(cq.Solid.makeCylinder(L.dd_set_screw_d_tap / 2, L.dd_hub_d / 2 + OVERLAP,
                                         V(L.dd_set_screw_u, 0, zh), V(0, 1, 0)))
    body = body.cut(*cut)
    if deflection_deg:
        body = body.rotate(V(0, 0, zh), V(1, 0, zh), deflection_deg)
    return _rot_z(body, L.vane_thetas[k])


def servo_horn(L: Layout, k: int = 0) -> cq.Shape:
    """Placeholder servo_horn_k.  v0.9 fix round: the stock double-arm horn cut to dd_horn_arm_len - the arm bar (along v
    at neutral, dd_horn_arm_w across, dd_u_arm0 .. dd_u_horn1), the spline collar (O dd_horn_collar_d, dd_u_horn1 ..
    dd_u_collar1) and the horn screw head (O dd_horn_screw_d, dd_u_screw0 .. dd_u_arm0) - one solid.  (v0.9 CAD-1: a disc
    O dd_horn_d, dd_u_flange1 .. dd_u_horn1.)"""
    th, zh = L.vane_thetas[k], L.vane_hinge_z
    if not hasattr(L, "dd_u_arm0"):
        return _radial_cyl(L.dd_horn_d, L.dd_u_flange1, L.dd_u_horn1, th, zh)
    hl, hw = L.dd_horn_arm_len / 2, L.dd_horn_arm_w / 2
    bar = _box_polar(L.dd_u_arm0, L.dd_u_horn1, -hl, hl, zh - hw, zh + hw, th)
    collar = _radial_cyl(L.dd_horn_collar_d, L.dd_u_arm0, L.dd_u_collar1, th, zh)     # round hub through the arm level
    head = _radial_cyl(L.dd_horn_screw_d, L.dd_u_screw0, L.dd_u_arm0 + OVERLAP, th, zh)
    return _fuse([bar, collar, head])


def hinge_bushing(L: Layout, k: int = 0) -> cq.Shape:
    """v0.9 fix round placeholder hinge_bushing_k: the brass tube O dd_bush_od / dd_bush_id over dd_bush_u on the hinge
    axis of vane k (pressed into the bore, flush with the duct's inner wall and the boss face)."""
    th, zh = L.vane_thetas[k], L.vane_hinge_z
    return _radial_cyl(L.dd_bush_od, L.dd_bush_u[0], L.dd_bush_u[1], th, zh).cut(
        _radial_cyl(L.dd_bush_id, L.dd_bush_u[0] - OVERLAP, L.dd_bush_u[1] + OVERLAP, th, zh))


def servo_direct(L: Layout, k: int = 0) -> cq.Shape:
    """Placeholder servo_vane_k in direct drive (SPEC 4.3 v0.9): case u [dd_u_case_top, dd_u_case_bot] x v +-body_w/2 x
    Z dd_z_case, ears u dd_u_ear x Z dd_z_ear, output boss O servo.boss_d from the case top in to dd_u_horn1."""
    th = L.fin_thetas[k]
    w = L.dd_case_w / 2
    case = _box_polar(L.dd_u_case_top, L.dd_u_case_bot, -w, w, L.dd_z_case[0], L.dd_z_case[1], th)
    ears = _box_polar(L.dd_u_ear[0], L.dd_u_ear[1], -w, w, L.dd_z_ear[0], L.dd_z_ear[1], th)
    # v0.9 fix round: the output boss starts where the horn's spline collar ends (the spline itself is inside the collar)
    boss = _radial_cyl(L.dd_boss_d, getattr(L, "dd_u_collar1", L.dd_u_horn1), L.dd_u_case_top + OVERLAP, th, L.vane_hinge_z)
    return _fuse([case, ears, boss])


def servo_connector(L: Layout, k: int = 0) -> cq.Shape:
    """Check volume (C23 v0.9): the mated inline lead connector in the connector bay of fairing k."""
    return _box_polar(L.dd_conn_u[0], L.dd_conn_u[1], -L.dd_conn_v, L.dd_conn_v, L.dd_conn_z[0], L.dd_conn_z[1],
                      L.fin_thetas[k])


def servo_lead_cutters(L: Layout) -> list[cq.Shape]:
    """Every SPEC 3.8 cutter of the propulsion module: pod windows (pushrod mode) / lead tunnels (v0.9 direct mode),
    entry slots, deck notches, rebate."""
    if L.direct_drive:
        out = [lead_tunnel(L, k) for k in range(L.vane_count)]
    else:
        out = [servo_lead_window(L, i) for i in range(L.vane_count)]
    out += [servo_lead_entry_slot(L, k) for k in range(L.vane_count)]
    for k in range(L.vane_count):
        out += servo_lead_deck_notch(L, k)
    return out + [servo_lead_rebate(L)]


def propulsion_module(L: Layout) -> cq.Shape:
    """SPEC 3: duct, outer skin, shoulder ring, (tabs: thin liner), sleeve, spigot, deck, webs,
    (tabs: screw lugs), hinge bosses, pods, fins; then the bore re-cut and the cutters
    (hinge bores, blind joint holes, ear holes; tabs: keyways, tab-screw holes, the two wire-slot boxes;
    v0.6 servo leads: pod windows, entry slots, deck notches, rebate)."""
    parts: list[cq.Shape] = []
    # 3.1 exhaust duct / vane section
    parts.append(_tube(L.vane_section_r, L.duct_r, L.z_exit, L.z_vane_section_top))
    # 3.2 / 3.7 outer skin (cone, + cylinder above z_cyl0 for tabs), wall measured radially inward
    parts.append(outer_skin(L))
    # 3.2 shoulder ring: duct bore -> cone outer surface, Z vane-section top .. EDF shoulder
    parts.append(_revolve(L.shoulder_ring_profile()))
    if L.tabs:              # 3.7 liner: duct bore continued under the housing
        parts.append(liner(L))
    # 3.2 sleeve
    parts.append(_tube(L.sleeve_r, L.edf_bore_r, L.z_sleeve_bottom, L.z_prop_spigot_bottom))
    # 3.2 spigot ring (solid annulus, the EDF lip rests on its top face) + the shoulder ring
    # that ties it to the sleeve / cone top (also closes the cavity between sleeve and cone)
    parts.append(_tube(L.spigot_od / 2, L.edf_bore_r, L.z_prop_spigot_bottom, L.z_prop_spigot_top))
    # deck under the spigot ring: EDF bore -> cone OUTER surface (follows the cone, no lip)
    t_deck = spigot_shoulder_t(L)
    zsb = L.z_prop_spigot_bottom
    parts.append(_revolve([(L.edf_bore_r, zsb - t_deck), (_cone_r_out(L, zsb - t_deck), zsb - t_deck),
                           (_cone_r_out(L, zsb), zsb), (L.edf_bore_r, zsb)]))
    # 3.2 webs: sleeve OD -> skin inner surface (box clipped by the filled skin profile)
    cone_filled = skin_filled(L)
    for th in L.web_thetas:
        web = _box_polar(L.sleeve_r - OVERLAP, L.cone_r_top + OVERLAP, -L.web_t / 2, L.web_t / 2,
                         L.z_sleeve_bottom, L.z_prop_spigot_bottom, th)
        parts.append(web.intersect(cone_filled))
    clock = getattr(L, "clock_mount", False)
    if L.tabs and not clock:  # 3.7 screw lugs (unioned before the keyway / screw cuts)
        for th in L.tab_thetas:
            parts += tab_screw_lugs(L, th)
    if clock:               # wire09c clock mount: floor pads, servo-column guide walls, the lead-passage blister + keel
        parts += clock_floor_pads(L) + clock_column_walls(L) + lead_blister_parts(L)
    # 3.3 hinge bosses
    for th in L.vane_thetas:
        parts.append(_radial_cyl(L.hinge_boss_od, L.duct_r, L.hinge_boss_r_out, th, L.vane_hinge_z))
    if L.direct_drive:      # v0.9: fin-root fairings with their keels and the lead pipes (no pods)
        for k in range(L.vane_count):
            parts += [fairing_box(L, k), fairing_keel(L, k)]
            if L.servo_leads:
                parts += lead_pipe(L, k)
    else:                   # 3.4 servo pods
        for i in range(L.vane_count):
            parts.append(servo_pod(L, i))
    # 3.5 fins
    for th in L.fin_thetas:
        parts.append(fin(L, th))
    body = _fuse(parts)
    # re-cut the exhaust bore so pods / bosses never intrude (SPEC 3.4)
    body = body.cut(_cyl_z(L.duct_r, L.z_exit - OVERLAP, L.z_edf_shoulder))
    cutters: list[cq.Shape] = []
    for th in L.vane_thetas:                                   # 3.3 hinge bores through boss and wall
        cutters.append(_radial_cyl(L.hinge_bore_d, L.duct_r - 5 * OVERLAP,
                                   (L.dd_pocket_u[0] + OVERLAP) if L.direct_drive else (L.hinge_boss_r_out + OVERLAP),
                                   th, L.vane_hinge_z))
    for th in L.joint_thetas:                                  # 3.2 spigot screw holes (self-tapping; v0.5d blind)
        cutters.append(prop_joint_hole_cutter(L, th))
    if L.direct_drive:                                         # v0.9 servo pockets, ear slots, cover tap holes
        for k in range(L.vane_count):
            cutters += servo_pocket_cutters(L, k) + cover_tap_cutters(L, k)
    else:
        for th in L.pod_thetas:                                # 3.4 servo ear screw holes
            cutters += _ear_hole_cutters(L, th, L.servo_axis_r, L.pod_rim_z)
    if L.tabs and clock:                                       # wire09c: keyways round the circle + the lead passage
        cutters += clock_column_channels(L) + [clock_keyway(L, a) for a in L.ck_keyways]
        cutters.append(lead_passage_cutter(L))
    elif L.tabs:                                               # 3.7 keyways, tab screws, wire slot
        for th in L.tab_thetas:
            cutters.append(tab_keyway(L, th))
            for z in L.tab_screw_zs:
                cutters += tab_screw_cutters(L, th, z)
        cutters += wire_slot_boxes(L)
    if L.servo_leads:                                          # 3.8 v0.6 vane-servo lead route
        cutters += servo_lead_cutters(L)
    if getattr(L, "has_fin_hole", False):                      # v0.9 CAD-2 VTOL: fin lightening holes
        cutters += fin_hole_cutters(L)
    return _cut(body, cutters)


def fin_hole_cutters(L: Layout) -> list[cq.Shape]:
    """v0.9 CAD-2 VTOL: one pentagon prism per fin (Layout.fin_hole_uz in the fin plane), +-(fin_t/2 + OVERLAP) thick."""
    return [_prism_uz(L.fin_hole_uz, -L.fin_t / 2 - OVERLAP, L.fin_t / 2 + OVERLAP, th) for th in L.fin_thetas]


# --------------------------------------------------------------------------- #
# wire09c clock mount (propulsion_module.tab_mount 'clock_keyways'): keyways round the circle, floor pads, servo-column
# guide walls, the lead passage cut and its blister + V keel; the lead envelopes of every generosity-box corner
# --------------------------------------------------------------------------- #
def _plan_prism(pts_xy: Sequence[tuple[float, float]], z0: float, z1: float) -> cq.Shape:
    """Plan polygon (vehicle XY) extruded from z0 to z1 (the Fusion poly_pod at theta 0)."""
    lo, hi = min(z0, z1), max(z0, z1)
    return cq.Workplane("XY").workplane(offset=lo).polyline(list(pts_xy)).close().extrude(hi - lo).val()


def clock_keyway(L: Layout, theta: float) -> cq.Shape:
    """Clock keyway at theta: kw wide, r keyway_r_in .. keyway_r_out, from the floor lp_z_floor up past the top."""
    h = L.ck_kw / 2
    return _box_polar(L.keyway_r_in, L.keyway_r_out, -h, h, L.lp_z_floor, L.keyway_z1, theta)


def clock_floor_pads(L: Layout) -> list[cq.Shape]:
    """Floor pad under every keyway: the (r, z) polygon Layout.ck_pad_rz (bridge sleeve -> skin, pad_t thick, 45 deg
    corbel underneath), kw + 2 pad_margin wide."""
    h = L.ck_pad_w / 2
    return [_prism_uz(L.ck_pad_rz, -h, h, a) for a in L.ck_keyways]


def clock_col_wall_rz(L: Layout) -> list[tuple[float, float]]:
    """(r, z) polygon of a servo-lead column guide wall: the web rule (sleeve_r - OVERLAP .. outer skin - 1.0, following
    the cone / cylinder kink) from the shoulder-ring top to the deck bottom."""
    z0 = L.z_edf_shoulder - OVERLAP
    z1 = L.z_prop_spigot_bottom - spigot_shoulder_t(L) + OVERLAP
    skin = [(L.cone_outer_radius(z0), z0)] + [(r, z) for r, z in L.outer_skin_profile() if z0 < z < z1] + \
        [(L.cone_outer_radius(z1), z1)]
    return [(L.sleeve_r - OVERLAP, z0)] + [(r - 1.0, z) for r, z in skin] + [(L.sleeve_r - OVERLAP, z1)]


def clock_col_channel_rz(L: Layout) -> list[tuple[float, float]]:
    """(r, z) polygon of the channel between a column's guide walls: the sleeve -> skin cavity (sleeve_r .. outer skin -
    cone_wall), shoulder-ring top .. deck bottom (its planar faces leave / take a lens <= 0.05 mm against the sleeve /
    skin cylinders - the Fusion script builds the same U prism + channel cut, X3)."""
    z0, z1 = L.z_edf_shoulder, L.z_prop_spigot_bottom - spigot_shoulder_t(L)
    skin = [(L.cone_outer_radius(z0), z0)] + [(r, z) for r, z in L.outer_skin_profile() if z0 < z < z1] +         [(L.cone_outer_radius(z1), z1)]
    return [(L.sleeve_r, z0)] + [(r - L.cone_wall, z) for r, z in skin] + [(L.sleeve_r, z1)]


def _col_widths(L: Layout) -> dict[float, tuple[float, float]]:
    out: dict[float, list] = {}
    for c, v0, v1 in getattr(L, "ck_col_walls", []):
        out.setdefault(c, []).extend([abs(v0), abs(v1)])
    return {c: (max(v), min(v)) for c, v in out.items()}


def clock_column_walls(L: Layout) -> list[cq.Shape]:
    """Servo-lead column guide walls (Layout.ck_col_walls, U channel): per column the U prism (outer half width), whose
    channel clock_column_channels cuts back to the cavity."""
    rz = clock_col_wall_rz(L)
    return [_prism_uz(rz, -vo, vo, c) for c, (vo, _) in _col_widths(L).items()]


def clock_column_channels(L: Layout) -> list[cq.Shape]:
    rz = clock_col_channel_rz(L)
    return [_prism_uz(rz, -vi, vi, c) for c, (_, vi) in _col_widths(L).items()]


def lead_passage_cutter(L: Layout) -> cq.Shape:
    """The lead passage (Layout.lp_fan_xy) from the fan floor up through the module top."""
    return _plan_prism(L.lp_fan_xy, L.lp_z_fan_floor, L.z_prop_top + OVERLAP)


def lead_blister_parts(L: Layout) -> list[cq.Shape]:
    """The blister (Layout.lp_blister_xy: the fan grown by wall, minus the EDF bore disc) from the blister bottom to
    under the intake, and the 45 deg V keel under its floor in keel_step steps (tprisms, each no wider than the
    blister outline over its step)."""
    out = [_plan_prism(L.lp_blister_xy, L.lp_z_blister_bottom, L.lp_z_blister_top)]
    zb, ov = L.lp_z_blister_bottom, L.lp_keel_overlap
    for u0, u1, hk in L.lp_keel:
        out.append(_prism_vz([(-hk, zb + ov), (hk, zb + ov), (0.0, zb - hk)], u0, u1, L.wire_theta))
    return out


def edf_unit_variant(L: Layout, tab_thetas: Sequence[float] | None = None, protrusion: float | None = None,
                     hub_lift: float = 0.0) -> cq.Shape:
    """edf_unit with other tab angles / motor protrusion; hub_lift raises the hub's bottom (C29: with the stator TE inset
    into the housing the hub ends at the TE - ASSUMED)."""
    old_t, old_p = list(L.tab_thetas), L.p["edf"]["motor_protrusion_rear"]
    try:
        if tab_thetas is not None:
            L.tab_thetas = list(tab_thetas)
        if protrusion is not None:
            L.p["edf"]["motor_protrusion_rear"] = protrusion
        e = L.p["edf"]
        housing = _tube(e["housing_od"] / 2, e["housing_id"] / 2, L.z_edf_housing_bottom, L.z_edf_lip_bottom)
        hub = _cyl_z(e["hub_d_ASSUMED"] / 2, L.z_edf_housing_bottom + hub_lift, L.z_edf_housing_bottom + EDF_HUB_LEN)
        lip = _tube(e["lip_od"] / 2, e["housing_id"] / 2, L.z_edf_lip_bottom, L.z_edf_lip_top)
        front = _fuse([lip, housing] + [edf_tab(L, th) for th in L.tab_thetas])
        motor = _cyl_z(e["motor_d"] / 2, L.z_edf_housing_bottom - e["motor_protrusion_rear"],
                       L.z_edf_housing_bottom + hub_lift)
        return front.fuse(_fuse([motor, hub]))
    finally:
        L.tab_thetas, L.p["edf"]["motor_protrusion_rear"] = old_t, old_p


def _pt_th(u: float, v: float, z: float, th: float) -> V:
    t = math.radians(th)
    return V(u * math.cos(t) - v * math.sin(t), u * math.sin(t) + v * math.cos(t), z)


def _sausage(p0: V, p1: V, r: float) -> list[cq.Shape]:
    d = p1 - p0
    out = [cq.Solid.makeSphere(r, p0, angleDegrees1=-90, angleDegrees2=90)]
    if d.Length > 1e-6:
        out.append(cq.Solid.makeCylinder(r, d.Length, p0, d.normalized()))
    return out


def clock_lead_env(L: Layout, c: dict, e: float, grown: bool = True, z_top: float | None = None) -> cq.Shape:
    """Envelope (compound of solids) of one generosity-box corner at azimuth theta_P + e (Layout.clock_lead_path):
    bundle box (+-wb tangential, +-hb along z; inside the duct up to the TE-inset height), single leads under the rim
    (+-rho), the bend and the vertical leg and the steer (sausages of radius rho), the flat row in the groove at u_top
    (theta_P).  grown: + wire_clearance (the path's halves already carry it); raw: - (wire_clearance + 0.05), so a
    touching contact reads 0."""
    pth = L.clock_lead_path(c)
    th_l, th0 = L.wire_theta + e, L.wire_theta
    d = 0.0 if grown else -(L.lp_wcl + 0.05)
    rm, r_e, r_s, r_v, R, z_c = pth["rm"], pth["r_e"], pth["r_s"], pth["r_v"], pth["R"], pth["z_c"]
    hb, wb, rho = pth["hb"] + d, pth["wb"] + d, pth["rho"] + d
    u_a = rm + (0.0 if grown else 0.1)
    sh: list[cq.Shape] = [_box_polar(u_a, r_e, -wb, wb, z_c - hb, z_c + hb, th_l)]
    zt = pth["z_in_duct_top"] + (L.lp_wcl if grown else -0.05)
    if zt > z_c + hb:
        sh.append(_box_polar(u_a, min(r_e, L.duct_r - 1.0), -wb, wb, z_c - hb, zt, th_l))
    if r_s > r_e + 1e-6:
        sh.append(_box_polar(r_e - 0.01, r_s, -rho, rho, pth["z_s"] - rho, pth["z_s"] + rho, th_l))
    n = 10
    pts = [(r_s + R * math.sin(0.5 * math.pi * j / n), pth["z_s"] + R - R * math.cos(0.5 * math.pi * j / n))
           for j in range(n + 1)]
    for (u0, z0), (u1, z1) in zip(pts[:-1], pts[1:]):
        sh += _sausage(_pt_th(u0, 0, z0, th_l), _pt_th(u1, 0, z1, th_l), rho)
    p_v1 = _pt_th(r_v, 0, pth["z_r0"], th_l)
    sh += _sausage(_pt_th(r_v, 0, pth["z_v0"], th_l), p_v1, rho)
    p_s1 = _pt_th(L.lp_u_top, 0, pth["z_r1"], th0)
    sh += _sausage(p_v1, p_s1, rho)
    sh.append(cq.Solid.makeSphere(rho, p_s1, angleDegrees1=-90, angleDegrees2=90))
    fh, fw = pth["flat"][0] + d, pth["flat"][1] + d
    zt2 = (L.z_intake_bottom + 20.0) if z_top is None else z_top
    sh.append(_box_polar(L.lp_u_top - fh, L.lp_u_top + fh, -fw, fw, pth["z_r1"], zt2, th0))
    return cq.Compound.makeCompound(sh)


def clock_lead_dropin(L: Layout, c: dict, e: float) -> cq.Shape:
    """Drop-in sweep of one corner (the EDF lowered into its bore with the leads formed): the grown envelope's plan
    swept from its lowest seated z up past the module top - bundle / single leads / bend / vertical part as a box u
    [rm, r_v + rho] x v +-max(wb, rho), the steer as the hull of its two end discs from z_r0 - rho."""
    pth = L.clock_lead_path(c)
    th_l = L.wire_theta + e
    top = L.z_prop_top + 5.0
    hv = max(pth["wb"], pth["rho"])
    z0 = min(pth["z_c"] - pth["hb"], pth["z_s"] - pth["rho"])
    out = [_box_polar(pth["rm"] + 0.1, pth["r_v"] + pth["rho"], -hv, hv, z0, top, th_l)]
    a, b = _pt_th(pth["r_v"], 0, 0, th_l), _pt_th(L.lp_u_top, 0, 0, L.wire_theta)
    d = b - a
    ln = d.Length
    rho = pth["rho"]
    if ln > 1e-6:
        nx, ny = -d.y / ln, d.x / ln
        hull = [(a.x + nx * rho, a.y + ny * rho), (b.x + nx * rho, b.y + ny * rho), (b.x - nx * rho, b.y - ny * rho),
                (a.x - nx * rho, a.y - ny * rho)]
        out.append(_plan_prism(hull, pth["z_r0"] - rho, top))
    out.append(_cyl_z(rho, pth["z_r0"] - rho, top, a.x, a.y))
    out.append(_cyl_z(rho, pth["z_r0"] - rho, top, b.x, b.y))
    return cq.Compound.makeCompound(out)


# --------------------------------------------------------------------------- #
# SPEC 4 - vanes, horns, linkage
# --------------------------------------------------------------------------- #
def vane_shaft(L: Layout, i: int = 0, deflection_deg: float = 0.0) -> cq.Shape:
    """Integral vane shaft alone (O vane_shaft_d, r shaft_r_in..shaft_r_out, with the D-flat)."""
    v = L.p["vane"]
    shaft = cq.Solid.makeCylinder(L.vane_shaft_d / 2, L.vane_shaft_r_out - L.vane_shaft_r_in,
                                  V(L.vane_shaft_r_in, 0, L.vane_hinge_z), V(1, 0, 0))
    flat_off = L.vane_shaft_d / 2 - v["d_flat_depth"]           # flat plane's distance from the axis
    if getattr(L, "dd_flat_side", "-z") == "+v":
        # v0.9 fix round: the flat faces the service side (+v = +y in the vane frame) at neutral (set screw access)
        slab = _box(L.vane_d_flat_r_in, L.vane_shaft_r_out + OVERLAP, flat_off, L.vane_shaft_d,
                    L.vane_hinge_z - L.vane_shaft_d, L.vane_hinge_z + L.vane_shaft_d)
    else:
        slab = _box(L.vane_d_flat_r_in, L.vane_shaft_r_out + OVERLAP, -L.vane_shaft_d, L.vane_shaft_d,
                    L.vane_hinge_z - L.vane_shaft_d, L.vane_hinge_z - flat_off)
    shaft = shaft.cut(slab)
    if deflection_deg:
        shaft = shaft.rotate(V(0, 0, L.vane_hinge_z), V(1, 0, L.vane_hinge_z), deflection_deg)
    return _rot_z(shaft, L.vane_thetas[i])


def vane(L: Layout, i: int = 0, deflection_deg: float = 0.0) -> cq.Shape:
    """SPEC 4.1: flat plate with rounded LE/TE and the integral D-flatted shaft.

    ``deflection_deg`` rotates the vane about its hinge axis (+e_r at theta_i); positive
    moves the trailing edge toward +e_theta (SPEC 8.2 sign convention).
    """
    v = L.p["vane"]
    plate = _box(L.vane_r_in, L.vane_r_out, -L.vane_t / 2, L.vane_t / 2, L.vane_te_z, L.vane_le_z)
    r_edge = min(v["edge_radius"], L.vane_t / 2 - 0.05)
    plate = cq.Workplane("XY").add(plate).edges("|X").fillet(r_edge).val()
    body = plate.fuse(vane_shaft(L, 0)).clean()
    if deflection_deg:
        body = body.rotate(V(0, 0, L.vane_hinge_z), V(1, 0, L.vane_hinge_z), deflection_deg)
    return _rot_z(body, L.vane_thetas[i])


def horn_d_bore_flat_distance(L: Layout) -> float:
    """Distance from the hinge axis to the flat of the horn's D-bore.

    Shaft flat sits (shaft_d/2 - d_flat_depth) from the axis; the bore adds the same radial
    clearance as the round part ((hinge_bore_d - shaft_d)/2).
    """
    v = L.p["vane"]
    return (L.vane_shaft_d / 2 - v["d_flat_depth"]) + (L.hinge_bore_d - L.vane_shaft_d) / 2


def vane_horn(L: Layout, i: int = 0) -> cq.Shape:
    """SPEC 4.2: radial plate with D-bore on the hinge axis and the pushrod pin hole."""
    v = L.p["vane"]
    plate = _box(L.horn_r_in, L.horn_r_out, -v["horn_w"] / 2, v["horn_w"] / 2, L.horn_tail_z, L.horn_tip_z)
    bore = cq.Solid.makeCylinder(L.hinge_bore_d / 2, (L.horn_r_out - L.horn_r_in) + 2 * OVERLAP,
                                 V(L.horn_r_in - OVERLAP, 0, L.vane_hinge_z), V(1, 0, 0))
    keep = _box(L.horn_r_in - 2 * OVERLAP, L.horn_r_out + 2 * OVERLAP, -L.hinge_bore_d, L.hinge_bore_d,
                L.vane_hinge_z - horn_d_bore_flat_distance(L), L.vane_hinge_z + L.hinge_bore_d)
    d_bore = bore.intersect(keep)
    pin = _radial_cyl(v["horn_pin_hole_d"], L.horn_r_in - OVERLAP, L.horn_r_out + OVERLAP, 0.0,
                      L.horn_tip_z - HORN_PIN_BELOW_TIP)
    return _rot_z(plate.cut(d_bore, pin), L.vane_thetas[i])


def _servo(L: Layout, theta: float, axis_r: float, bottom_z: float, rim_z: float, arm_z_bottom: float) -> cq.Shape:
    """SPEC 4.3 servo placeholder: body + ears + output boss, long side tangential."""
    s = L.p["servo"]
    u0, u1 = axis_r - s["body_w"] / 2, axis_r + s["body_w"] / 2
    case_top = bottom_z + s["case_top_from_bottom"]
    body = _box(u0, u1, -s["body_l"] / 2, s["body_l"] / 2, bottom_z, case_top)
    ears = _box(u0, u1, -s["ear_span"] / 2, s["ear_span"] / 2, rim_z, rim_z + s["ear_t"])
    boss = _cyl_z(s["boss_d"] / 2, case_top, arm_z_bottom, axis_r, L.servo_shaft_tangential_offset)
    return _rot_z(_fuse([body, ears, boss]), theta)


def servo_vane(L: Layout, i: int = 0) -> cq.Shape:
    return _servo(L, L.pod_thetas[i], L.servo_axis_r, L.servo_bottom_z, L.pod_rim_z, L.servo_arm_z_bottom)


def servo_release(L: Layout) -> cq.Shape:
    return _servo(L, L.release_pod_theta, L.release_servo_axis_r, L.release_servo_bottom_z,
                  L.release_pod_rim_z, L.z_latch_arm_bottom)


def servo_arm(L: Layout, i: int = 0) -> cq.Shape:
    """SPEC 4.4: arm from the servo axis radially outward in the pod frame."""
    pod = L.p["pod"]
    off = L.servo_shaft_tangential_offset
    return _box_polar(L.servo_axis_r, L.servo_arm_tip_r, off - pod["arm_w"] / 2, off + pod["arm_w"] / 2,
                      L.servo_arm_z_bottom, L.servo_arm_z_top, L.pod_thetas[i])


def pushrod_endpoints(L: Layout, i: int = 0) -> tuple[V, V]:
    """Arm tip (mid-thickness of the arm) -> horn pin hole, SPEC 4.5."""
    p1 = _polar(L.servo_arm_tip_r, L.servo_shaft_tangential_offset, L.pod_thetas[i],
                (L.servo_arm_z_bottom + L.servo_arm_z_top) / 2)
    p2 = _polar((L.horn_r_in + L.horn_r_out) / 2, 0.0, L.vane_thetas[i], L.horn_tip_z - HORN_PIN_BELOW_TIP)
    return p1, p2


def pushrod_length(L: Layout, i: int = 0) -> float:
    p1, p2 = pushrod_endpoints(L, i)
    return (p2 - p1).Length


def pushrod(L: Layout, i: int = 0) -> cq.Shape:
    """SPEC 4.5 pushrod placeholder cylinder between arm tip and horn pin hole."""
    p1, p2 = pushrod_endpoints(L, i)
    d = p2 - p1
    return cq.Solid.makeCylinder(L.p["pod"]["pushrod_d"] / 2, d.Length, p1, d.normalized())


# --------------------------------------------------------------------------- #
# SPEC 5.1 - intake module
# --------------------------------------------------------------------------- #
def intake_band(L: Layout) -> cq.Shape:
    """SPEC 5.1 v0.6.1 windowed band: r [r_band_in, body_r] (window_wall_t thick, thickened inward from the body OD),
    Z [z_shelf_top, z_intake_bulkhead_bottom], before the windows are cut."""
    return _tube(L.body_r, L.r_band_in, L.z_shelf_top, L.z_intake_bulkhead_bottom)


def intake_window_sectors(L: Layout) -> list[cq.Shape]:
    """SPEC 5.1 v0.6.1 window sectors: one radial sector per window (Layout.window_sectors, centred on window_thetas,
    angle window_angle_deg), r [r_band_in - OVERLAP, body_r + OVERLAP], Z [z_window_bottom, z_window_top].  Radial side
    faces: every strut keeps strut_angle - strut_w at body_r, strut_w_in at r_band_in.  (The v0.6.1 cutters; v0.6.2
    intersects them with the gable prism.)"""
    return [_sector(L.r_band_in - OVERLAP, L.body_r + OVERLAP, lo, hi, L.z_window_bottom, L.z_window_top)
            for lo, hi in L.window_sectors]


def intake_window_gable_prism(L: Layout, theta: float) -> cq.Shape:
    """v0.6.2 gable prism of the window centred on theta: Layout.window_gable_profile_yz (the region above the window's
    lower boundary in the window frame: y' tangential, z) extruded along the window centre direction u over
    +-window_gable_u_half.  Planar faces: the bridge at z_window_bottom and the two slope_deg faces."""
    U = L.window_gable_u_half
    wp = cq.Workplane("YZ").workplane(offset=-U).polyline(list(L.window_gable_profile_yz)).close().extrude(2.0 * U)
    return _rot_z(wp.val(), theta)


def intake_window_cutters(L: Layout) -> list[cq.Shape]:
    """SPEC 5.1 windows: v0.6.1 radial sector (intake_window_sectors) per window; v0.6.2 (Layout.has_window_gable) that
    sector INTERSECT the gable prism (intake_window_gable_prism), so the lower end is the truncated gable and the band
    keeps the gussets below it; the upper end stays square."""
    sectors = intake_window_sectors(L)
    if not L.has_window_gable:
        return sectors
    return [s.intersect(intake_window_gable_prism(L, th)) for s, th in zip(sectors, L.window_thetas)]


def intake_window_gussets(L: Layout) -> list[cq.Shape]:
    """v0.6.2 design gussets (check / test helper): per window, the band r [r_band_in, body_r] over the window sector,
    Z [z_window_bottom, z_gable_top + OVERLAP], minus the gable prism - what the band keeps under the lower boundary.
    Empty without a gable."""
    if not L.has_window_gable:
        return []
    return [_sector(L.r_band_in, L.body_r, lo, hi, L.z_window_bottom, L.z_gable_top + OVERLAP).cut(
        intake_window_gable_prism(L, th)) for (lo, hi), th in zip(L.window_sectors, L.window_thetas)]


def intake_opening_z_range(L: Layout) -> tuple[float, float]:
    """Z range over which the inlet opening is measured: the band from above the lead notches / channels that open it
    on purpose at the struts (tabs: wire_ch_z1, servo leads: sl_ch_z1) - else from z_shelf_top - to the bulkhead.  It
    contains both margin rings' window side, so a window cut taller than designed would count."""
    z0 = L.z_shelf_top
    if L.tabs:
        z0 = max(z0, L.wire_ch_z1)
    if L.servo_leads:
        z0 = max(z0, L.sl_ch_z1)
    return z0, L.z_intake_bulkhead_bottom


def intake_opening_probe_radii(L: Layout, n: int = INLET_PROBE_COUNT) -> list[tuple[float, float]]:
    """(r_a, r_b) of n cylindrical probe shells INLET_PROBE_T thick spread across the band, the first starting
    INLET_PROBE_GAP outside r_band_in, the last ending INLET_PROBE_GAP inside body_r (no probe face coincides with a
    band face)."""
    a0, a1 = L.r_band_in + INLET_PROBE_GAP, L.body_r - INLET_PROBE_GAP - INLET_PROBE_T
    return [(a0 + (a1 - a0) * k / (n - 1), a0 + (a1 - a0) * k / (n - 1) + INLET_PROBE_T) for k in range(n)]


def intake_opening_areas_mm2(L: Layout, intake: cq.Shape) -> list[tuple[float, float]]:
    """Measured opening of the intake band on the SOLID (C9, report): for every probe shell r [r_a, r_b] over
    intake_opening_z_range, (shell volume not covered by ``intake``) / (r_b - r_a) = the open area of the vertical
    cylinder at the shell's mean radius (exact for openings whose width is linear in r, a mean otherwise).  Returns
    [(r_mean, area_mm2)] from the inside out; the minimum is the inlet area that counts.  NaN when OCC cannot compute a
    probe (no evidence)."""
    z0, z1 = intake_opening_z_range(L)
    out = []
    for ra, rb in intake_opening_probe_radii(L):
        shell = _tube(rb, ra, z0, z1)
        try:
            rest = shell.cut(intake)
            v = 0.0 if rest is None or rest.wrapped.IsNull() else float(rest.Volume())
        except Exception:  # noqa: BLE001 - no evidence
            v = math.nan
        out.append((0.5 * (ra + rb), v / (rb - ra)))
    return out


def intake_window_area_mm2(L: Layout, intake: cq.Shape | None = None) -> float:
    """Measured inlet area (mm^2): the minimum of ``intake_opening_areas_mm2`` on ``intake`` (default: the intake
    module built from L).  NaN propagates (min with NaN -> NaN), so an unmeasurable opening is never a PASS."""
    areas = [a for _, a in intake_opening_areas_mm2(L, intake if intake is not None else intake_module(L))]
    return math.nan if any(math.isnan(a) for a in areas) else min(areas)


def intake_wire_bulge_parts(L: Layout) -> list[cq.Shape]:
    """SPEC 5.1 (tabs) solids that close the motor-lead channel, unioned before the cuts.  Closed channel (v0.5d-v0.8):
    one bulge box tangential +-wire_bulge_w/2, r [body_r - 0.6, wire_bulge_r_out], Z [wire_bulge_z0, wire_bulge_z1].
    v0.9 open groove (``wire_ch_open``): the two side walls (|v| in [wire_ch_w/2, + wire_side_wall], r [body_r - 0.6,
    wire_ch_r1], Z [z_intake_bottom, wire_ch_z1]), the roof wedge (Layout.wire_roof_rz over the bulge width) and four
    cover-screw pads (wire_pad_v on both sides at wire_pad_zs, pad_h tall) - no outer wall: the wire_cover closes it."""
    th, r_in = L.wire_theta, L.body_r - WIRE_BULGE_INTO_WALL
    if not getattr(L, "wire_ch_open", False):
        return [_box_polar(r_in, L.wire_bulge_r_out, -L.wire_bulge_w / 2, L.wire_bulge_w / 2,
                           L.wire_bulge_z0, L.wire_bulge_z1, th)]
    hw, sw = L.wire_ch_w / 2, L.wire_side_wall
    out = [_box_polar(r_in, L.wire_ch_r1, s * hw if s > 0 else -hw - sw, hw + sw if s > 0 else -hw,
                      L.z_intake_bottom, L.wire_ch_z1, th) for s in (1.0, -1.0)]
    out.append(_prism_uz(L.wire_roof_rz, -hw - sw, hw + sw, th))
    for s in (1.0, -1.0):
        for z in L.wire_pad_zs:
            v0, v1 = L.wire_pad_v
            out.append(_box_polar(r_in, L.wire_ch_r1, v0 if s > 0 else -v1, v1 if s > 0 else -v0,
                                  z - L.wire_pad_h / 2, z + L.wire_pad_h / 2, th))
    return out


def intake_wire_bulge(L: Layout) -> cq.Shape:
    """SPEC 5.1 (tabs) union of ``intake_wire_bulge_parts`` (check / display helper)."""
    return _fuse(intake_wire_bulge_parts(L))


def intake_wire_channel(L: Layout) -> cq.Shape:
    """SPEC 5.1 (tabs) wire-channel main void: tangential +-wire_ch_w/2, r [wire_ch_r0, wire_ch_r1],
    Z [wire_ch_z0, wire_ch_z1] (cut after the bulge is unioned; the floor wedge is unioned back after the cuts).
    v0.9 open groove: r out past the side walls (open outward) and Z from OVERLAP under the skirt's bottom edge."""
    if getattr(L, "wire_ch_open", False):
        return _box_polar(L.wire_ch_r0, L.wire_ch_r1 + OVERLAP, -L.wire_ch_w / 2, L.wire_ch_w / 2,
                          L.wire_ch_z0 - OVERLAP, L.wire_groove_z1, L.wire_theta)
    return _box_polar(L.wire_ch_r0, L.wire_ch_r1, -L.wire_ch_w / 2, L.wire_ch_w / 2,
                      L.wire_ch_z0, L.wire_ch_z1, L.wire_theta)


def wire_cover_screw_axes(L: Layout) -> list[tuple[tuple[float, float, float], tuple[float, float, float]]]:
    """v0.9: axis of every wire_cover screw, from the side wall's outer face (under the cover) radially into its pad (tap
    end) - as cover_screw_axes starts at the fairing face."""
    out = []
    for s in (1.0, -1.0):
        for z in L.wire_pad_zs:
            out.append((L.ml_xyz(L.wire_ch_r1, s * L.wire_screw_v, z),
                        L.ml_xyz(L.wire_ch_r1 - L.wire_cov_tap_depth, s * L.wire_screw_v, z)))
    return out


def intake_wire_cover_taps(L: Layout) -> list[cq.Shape]:
    """v0.9: blind tap holes (O wire_cov_d_tap, wire_cov_tap_depth into the pads from the side walls' outer face)."""
    return [_radial_cyl(L.wire_cov_d_tap, L.wire_ch_r1 - L.wire_cov_tap_depth, L.wire_ch_r1 + OVERLAP, L.wire_theta, z, s * L.wire_screw_v)
            for s in (1.0, -1.0) for z in L.wire_pad_zs]


def wire_cover(L: Layout) -> cq.Shape:
    """Printed wire_cover (v0.9): plate u wire_cov_u (on the side walls' outer ends), v +-wire_cov_hv, Z wire_cov_z at
    wire_theta, with four O wire_cov_d_clear holes on the screw axes."""
    th = L.wire_theta
    plate = _box_polar(L.wire_cov_u[0], L.wire_cov_u[1], -L.wire_cov_hv, L.wire_cov_hv, L.wire_cov_z[0], L.wire_cov_z[1], th)
    holes = [_radial_cyl(L.wire_cov_d_clear, L.wire_cov_u[0] - OVERLAP, L.wire_cov_u[1] + OVERLAP, th, z, s * L.wire_screw_v)
             for s in (1.0, -1.0) for z in L.wire_pad_zs]
    return plate.cut(*holes)


def intake_wire_floor_wedge(L: Layout) -> cq.Shape:
    """SPEC 5.1 v0.5d (tabs) channel floor wedge: the (r, z) triangle wire_ch_floor_rz in the pod frame at
    wire_theta, extruded tangentially over +-wire_ch_w/2.  Unioned after the channel / notch cuts: its bottom face
    lies on the bulge floor (Z wire_ch_z0) and its outer 0.5 mm overlaps the bulge's outer wall, so the floor rises
    at 45 deg from the skirt's inner face to wire_gate_lo_z0 at wire_ch_r1 (no support in the closed channel
    when the intake prints flipped)."""
    return _prism_uz(L.wire_ch_floor_rz, -L.wire_ch_w / 2, L.wire_ch_w / 2, L.wire_theta)


def intake_wire_notch(L: Layout) -> cq.Shape:
    """SPEC 5.1 (tabs) shelf-notch void: tangential +-wire_ch_w/2, r [wire_notch_r0, wire_ch_r0 + 0.5],
    Z [wire_notch_z0, wire_ch_z1] - the leads step back in over the lip face through the shelf."""
    gw = getattr(L, "wire_gate_w", L.wire_ch_w)          # v0.9: the gate may be narrower than the groove
    if getattr(L, "wire_ch_open", False):                # v0.9: the gate runs out through the band to the open groove
        return _box_polar(L.wire_notch_r0, L.wire_ch_r1 + OVERLAP, -gw / 2, gw / 2, L.wire_notch_z0, L.wire_ch_z1,
                          L.wire_theta)
    return _box_polar(L.wire_notch_r0, L.wire_ch_r0 + WIRE_NOTCH_PAST_CH_R0, -gw / 2, gw / 2,
                      L.wire_notch_z0, L.wire_ch_z1, L.wire_theta)


def intake_bulkhead_wire_hole(L: Layout) -> cq.Shape:
    """SPEC 5.1 (tabs) bulkhead wire hole: theta wire_theta, tangential +-wire_hole_w/2 (v0.9 fix round: its own width;
    before = wire_ch_w), r [wire_hole_r0, wire_hole_r1], through the intake bulkhead."""
    hw = getattr(L, "wire_hole_w", L.wire_ch_w) / 2
    return _box_polar(L.wire_hole_r0, L.wire_hole_r1, -hw, hw,
                      L.z_intake_bulkhead_bottom - OVERLAP, L.z_intake_bulkhead_top + OVERLAP, L.wire_theta)


def intake_wire_skirt_cut(L: Layout) -> cq.Shape:
    """v0.9 fix round (open groove): the groove's part below the skirt top, from the skirt bore's chord at the groove edge
    (Layout.wire_skirt_cut_r0) instead of wire_ch_r0, so no sliver of the skirt is left beside the groove."""
    return _box_polar(L.wire_skirt_cut_r0, L.wire_ch_r1 + OVERLAP, -L.wire_ch_w / 2, L.wire_ch_w / 2,
                      L.wire_ch_z0 - OVERLAP, L.z_intake_skirt_top, L.wire_theta)


def intake_lip_slit(L: Layout) -> cq.Shape:
    """v0.9 fix round: vertical slit (wire_lip_slit_w) through the 1.2 mm lip-pocket wall at wire_theta, r from 0.5 inside
    the pocket bore to 0.5 past wire_ch_r0, Z from 1 mm under the skirt top to OVERLAP over the lip-pocket top (into the
    gate) - the bare leads slide from the interior into the groove while the intake is held above the EDF."""
    hw = L.wire_lip_slit_w / 2
    return _box_polar(L.lip_pocket_id / 2 - 0.5, L.wire_ch_r0 + 0.5, -hw, hw, L.z_intake_skirt_top - 1.0,
                      L.z_lip_pocket_top + OVERLAP, L.wire_theta)


def tie_anchor_rz(L: Layout, zc: float) -> list[tuple[float, float]]:
    """(u, z) profile of the tie anchor centred (block part) at zc: the block ml_anchor_h tall from the anchors' inner face
    ml_anchor_u out ml_anchor_into into the strut, and a wedge_deg roof toward the bulkhead (+z: the intake prints
    flipped, so the roof is the anchor's underside in print)."""
    u0, u1, h = L.ml_anchor_u, L.ml_anchor_u1, L.ml_anchor_h
    rise = (u1 - u0) * math.tan(math.radians(L.ml_anchor_wedge))
    z0, z1 = zc - h / 2, zc + h / 2
    return [(u0, z0), (u1, z0), (u1, z1 + rise), (u0, z1)]


def tie_anchors(L: Layout) -> list[cq.Shape]:
    """v0.9 fix round: the strut tie anchors (solid, unioned with the intake before the cuts) at Layout.ml_anchor_z on the
    0 deg strut's inner face, +-ml_anchor_w/2 tangential."""
    hw = L.ml_anchor_w / 2
    return [_prism_uz(tie_anchor_rz(L, zc), -hw, hw, L.wire_theta) for zc in getattr(L, "ml_anchor_z", [])]


def tie_anchor_tunnels(L: Layout) -> list[cq.Shape]:
    """v0.9 fix round: the cable-tie tunnel through each anchor (void): u from the anchor's inner face + wall, tunnel_t
    radial x tunnel_h tall, tangentially through (OVERLAP past both sides)."""
    t, h = L.ml_anchor_tunnel
    u0 = L.ml_anchor_u + L.ml_anchor_wall
    hw = L.ml_anchor_w / 2 + OVERLAP
    return [_box_polar(u0, u0 + t, -hw, hw, zc - h / 2, zc + h / 2, L.wire_theta) for zc in getattr(L, "ml_anchor_z", [])]


def intake_servo_bulge(L: Layout) -> cq.Shape:
    """SPEC 5.1 v0.6 servo bulge at sl_gather_theta: tangential +-sl_bulge_w/2, r [body_r - 0.6, sl_bulge_r_out],
    Z [sl_bulge_z0, sl_bulge_z1] (unioned before the cuts, so its bottom, top and outer walls stay closed)."""
    return _box_polar(L.body_r - WIRE_BULGE_INTO_WALL, L.sl_bulge_r_out, -L.sl_bulge_w / 2, L.sl_bulge_w / 2,
                      L.sl_bulge_z0, L.sl_bulge_z1, L.sl_gather_theta)


def intake_servo_channel(L: Layout) -> cq.Shape:
    """SPEC 5.1 v0.6 servo channel void: tangential +-sl_ch_w/2, r [sl_ch_r0, sl_ch_r1], Z [sl_ch_z0, sl_ch_z1]."""
    return _box_polar(L.sl_ch_r0, L.sl_ch_r1, -L.sl_ch_w / 2, L.sl_ch_w / 2, L.sl_ch_z0, L.sl_ch_z1, L.sl_gather_theta)


def intake_servo_notch(L: Layout) -> cq.Shape:
    """SPEC 5.1 v0.6 servo shelf notch: tangential +-sl_ch_w/2, r [sl_notch_r0, sl_ch_r0 + 0.5], Z [z_lip_pocket_top,
    sl_ch_z1] - the leads step back in over the lip face."""
    return _box_polar(L.sl_notch_r0, L.sl_ch_r0 + WIRE_NOTCH_PAST_CH_R0, -L.sl_ch_w / 2, L.sl_ch_w / 2,
                      L.z_lip_pocket_top, L.sl_ch_z1, L.sl_gather_theta)


def intake_servo_floor_wedge(L: Layout) -> cq.Shape:
    """SPEC 5.1 v0.6 servo channel floor wedge: the (r, z) triangle sl_ch_floor_rz over +-sl_ch_w/2 at
    sl_gather_theta (unioned after the cuts; it reaches sl_gate_lo_z0 at sl_ch_r1, its outer 0.5 mm in the bulge wall)."""
    return _prism_uz(L.sl_ch_floor_rz, -L.sl_ch_w / 2, L.sl_ch_w / 2, L.sl_gather_theta)


def intake_bulkhead_servo_hole(L: Layout) -> cq.Shape:
    """SPEC 5.1 v0.6 bulkhead servo hole: tangential +-sl_ch_w/2, r [sl_hole_r0, sl_hole_r1] through the bulkhead."""
    return _box_polar(L.sl_hole_r0, L.sl_hole_r1, -L.sl_ch_w / 2, L.sl_ch_w / 2, L.z_intake_bulkhead_bottom - OVERLAP,
                      L.z_intake_bulkhead_top + OVERLAP, L.sl_gather_theta)


def intake_module(L: Layout) -> cq.Shape:
    """SPEC 5.1: skirt, lip pocket, shelf, windowed band (v0.6.1: window_wall_t thick, r_band_in..body_r, radial-sector
    windows, struts at strut_thetas), bulkhead, spigot;
    tabs: + wire bulge (union), - wire channel, shelf notch and bulkhead wire hole (cut), + floor wedge (union);
    v0.6 servo leads: the same pattern at sl_gather_theta (servo bulge, channel, notch, hole, floor wedge)."""
    im = L.p["intake_module"]
    parts = [
        _tube(L.body_r, L.body_id / 2, L.z_intake_bottom, L.z_intake_skirt_top),
        _tube(L.body_r, L.lip_pocket_id / 2, L.z_intake_skirt_top, L.z_lip_pocket_top),
        _tube(L.body_r, L.shelf_id / 2, L.z_lip_pocket_top, L.z_shelf_top),
        intake_band(L),
        _cyl_z(L.body_r, L.z_intake_bulkhead_bottom, L.z_intake_bulkhead_top),
        _tube(L.spigot_od / 2, L.spigot_id / 2, L.z_intake_spigot_bottom, L.z_intake_spigot_top),
    ]
    if L.tabs:
        parts += intake_wire_bulge_parts(L)
        parts += tie_anchors(L)                      # v0.9 fix round: motor-lead tie anchors on the 0 deg strut
    if L.servo_leads:
        parts.append(intake_servo_bulge(L))
    body = _fuse(parts)
    cutters = list(intake_window_cutters(L))
    zb0, zb1 = L.z_intake_bulkhead_bottom - OVERLAP, L.z_intake_bulkhead_top + OVERLAP
    cutters.append(_cyl_z(L.cable_hole_d / 2, zb0, zb1))
    for th in L.vent_hole_thetas:
        c = _polar(L.vent_hole_r, 0.0, th, 0.0)
        cutters.append(_cyl_z(L.vent_hole_d / 2, zb0, zb1, c.x, c.y))
    for th in L.joint_thetas:
        cutters.append(_radial_cyl(L.p["body"]["joint_hole_outer_d"], L.body_id / 2 - 2 * OVERLAP,
                                   L.body_r + 2 * OVERLAP, th, L.z_intake_bottom + L.joint_screw_dz))
        cutters.append(_radial_cyl(L.p["body"]["joint_hole_spigot_d"], L.spigot_id / 2 - 2 * OVERLAP,
                                   L.spigot_od / 2 + 2 * OVERLAP, th, L.z_intake_spigot_bottom + L.joint_screw_dz))
    if L.tabs:
        cutters += [intake_wire_channel(L), intake_wire_notch(L), intake_bulkhead_wire_hole(L)]
        if getattr(L, "wire_ch_open", False):        # v0.9: the cover screws' tap holes in the pads
            cutters += intake_wire_cover_taps(L)
            if L.wire_skirt_cut_r0 < L.wire_ch_r0 - 1e-9:     # v0.9 fix round: no skirt sliver at the groove edges
                cutters.append(intake_wire_skirt_cut(L))
            if getattr(L, "wire_lip_slit_w", 0.0):            # v0.9 fix round: lip-pocket wall slit
                cutters.append(intake_lip_slit(L))
            cutters += tie_anchor_tunnels(L)                  # v0.9 fix round: cable-tie tunnels
    if L.servo_leads:
        cutters += [intake_servo_channel(L), intake_servo_notch(L), intake_bulkhead_servo_hole(L)]
    del im
    body = _cut(body, cutters)
    if L.tabs and not getattr(L, "wire_ch_open", False):   # v0.5d: 45 deg channel floor, unioned AFTER the cuts
        body = _fuse([body, intake_wire_floor_wedge(L)])
    if L.servo_leads:       # v0.6: the servo channel's floor wedge, likewise
        body = _fuse([body, intake_servo_floor_wedge(L)])
    return body


# --------------------------------------------------------------------------- #
# SPEC 5.2 - avionics module, tray, electronics placeholders
# --------------------------------------------------------------------------- #
def _rail_y_range(L: Layout, sy: float) -> tuple[float, float]:
    """y extent of the rail ribs on the +y (sy > 0) or -y wall: rail_depth inward from rail_y, out to body_r
    (the solids are clipped at body_r)."""
    y_in = L.rail_y - L.rail_depth
    return (y_in, L.body_r) if sy > 0 else (-L.body_r, -y_in)


def tray_stop_blocks(L: Layout) -> list[cq.Shape]:
    """SPEC 5.2 v0.5d tray stop (both presets), unclipped: in each rail slot a block between the inner faces of the
    two ribs (x in tray_x +- rail_gap/2), the ribs' y / radial extent, Z [z_tray_stop_bottom, z_tray_bottom].
    Empty when params give no tray_stop_h.  avionics_module clips the blocks at body_r like the ribs."""
    if not L.tray_stop_h:
        return []
    return [_box(L.tray_x - L.rail_gap / 2, L.tray_x + L.rail_gap / 2, *_rail_y_range(L, sy),
                 L.z_tray_stop_bottom, L.z_tray_bottom) for sy in (+1.0, -1.0)]


def _prism_vz(pts_vz: Sequence[tuple[float, float]], u0: float, u1: float, theta: float) -> cq.Shape:
    """v0.8.1: polygon in the (v, z) plane (tangential, vertical) extruded radially over u in [u0, u1], then rotated to
    theta (the pod frame: u along +X, v along +Y before the rotation)."""
    wp = cq.Workplane("YZ").workplane(offset=u0).polyline(list(pts_vz)).close().extrude(u1 - u0)
    return _rot_z(wp.val(), theta)


def gns_boss_prisms(L: Layout) -> list[cq.Shape]:
    """v0.8.1 (Layout._gns): the four GNS-board bosses before the OD clip - each a diamond (Layout.gns_diamond_vz, faces at
    45 deg) from the board's back face u gns_u_back out to body_r + OVERLAP at gns_theta."""
    return [_prism_vz(L.gns_diamond_vz(k), L.gns_u_back, L.body_r + OVERLAP, L.gns_theta) for k in range(4)]


def gns_boss_clip(L: Layout) -> cq.Shape:
    """v0.8.1: the cylinder r <= body_r over the bosses' Z range (+- OVERLAP) - the boss prisms are clipped to it, so they
    end flush with the tube's outer surface (as the rail ribs)."""
    return _cyl_z(L.body_r, L.gns_boss_z[0] - OVERLAP, L.gns_boss_z[1] + OVERLAP)


def gns_bosses(L: Layout) -> list[cq.Shape]:
    """v0.8.1: the four GNS bosses clipped at the tube OD (gns_boss_prisms INTERSECT gns_boss_clip)."""
    clip = gns_boss_clip(L)
    return [b.intersect(clip) for b in gns_boss_prisms(L)]


def gns_hole_cutters(L: Layout) -> list[cq.Shape]:
    """v0.8.1: blind Ø gns_hole_d tap hole per boss along u, from OVERLAP inside the bay (air) to gns_hole_u[1]
    (gns_hole_depth past the tip), at the boss's (v, z)."""
    return [_radial_cyl(L.gns_hole_d, L.gns_u_back - OVERLAP, L.gns_hole_u[1], L.gns_theta, z, v) for v, z in L.gns_hole_vz]


def gns_board(L: Layout) -> cq.Shape:
    """v0.8.1 placeholder ``gns``: the GNS board box (Layout.gns_x/y/z) on the boss tips at the -x wall."""
    return _box(*L.gns_x, *L.gns_y, *L.gns_z)


def avionics_module(L: Layout, with_gps: bool = True, with_gns: bool = True, with_livery: bool = True) -> cq.Shape:
    """SPEC 5.2: tube + spigot, two pairs of tray rails, (v0.5d) a tray stop block in each rail slot, vent slots,
    joint holes; v0.8 (``Layout.has_gps_pod`` and ``with_gps``): the GPS lead entry hole, cut last; v0.8.1
    (``Layout.has_gns``, ``with_gps`` and ``with_gns``): the four GNS-board bosses (clipped at the OD, unioned after the
    rails / stops) and their blind tap holes (cut after the entry hole), so the v0.8 solid is the same without them.
    livery (``Layout.has_livery`` and ``with_livery``, user 2026-11-04): the WHITE body - the logo pocket
    (livery_inlay_solids out past the OD) is cut after everything else, so ``with_livery=False`` is the plain
    module exactly and plain = white body + avionics_logo (C36)."""
    parts = [
        _tube(L.body_r, L.body_id / 2, L.z_avionics_bottom, L.z_avionics_spigot_bottom),
        _tube(L.spigot_od / 2, L.spigot_id / 2, L.z_avionics_spigot_bottom, L.z_avionics_spigot_top),
        _spigot_shoulder(L, L.z_avionics_spigot_bottom, spigot_shoulder_r_in(L)),
    ]
    clip = _cyl_z(L.body_r, L.z_rail_bottom - OVERLAP, L.z_rail_top + OVERLAP)
    for sy in (+1.0, -1.0):
        for sx in (+1.0, -1.0):
            xc = L.tray_x + sx * (L.rail_gap / 2 + L.rail_w / 2)
            y0, y1 = _rail_y_range(L, sy)
            rib = _box(xc - L.rail_w / 2, xc + L.rail_w / 2, y0, y1, L.z_rail_bottom, L.z_rail_top)
            parts.append(rib.intersect(clip))
    # tray stops: they share their x faces with the ribs and their volume with the tube wall (one solid)
    parts += [blk.intersect(clip) for blk in tray_stop_blocks(L)]
    gns = with_gps and with_gns and getattr(L, "has_gns", False)
    if gns:
        parts += gns_bosses(L)
    body = _fuse(parts)
    cutters: list[cq.Shape] = []
    for th in L.joint_thetas:
        cutters.append(_radial_cyl(L.p["body"]["joint_hole_outer_d"], L.body_id / 2 - 2 * OVERLAP,
                                   L.body_r + 2 * OVERLAP, th, L.z_avionics_bottom + L.joint_screw_dz))
        cutters.append(_radial_cyl(L.p["body"]["joint_hole_spigot_d"], L.spigot_id / 2 - 2 * OVERLAP,
                                   L.spigot_od / 2 + 2 * OVERLAP, th, L.z_avionics_spigot_bottom + L.joint_screw_dz))
    for th in L.vent_slot_thetas:
        # SPEC 5.2: the vent slot cuts the WALL only.  The cutter starts 0.5 mm inside the wall's inner face (at
        # |v| = vent_slot_w/2 that face is at u = 58.66 for the 117.6 bore); before v0.5d it started 2 mm inside and
        # bit 2 mm off the tray-rail rib ends at +-61 deg over the slot height (found by the X3 centroid check).
        cutters.append(_box_polar(L.body_id / 2 - 0.5, L.body_r + 2 * OVERLAP,
                                  -L.vent_slot_w / 2, L.vent_slot_w / 2,
                                  L.z_vent_slot_bottom, L.z_vent_slot_bottom + L.vent_slot_h, th))
    body = _cut(body, cutters)
    if with_gps and getattr(L, "has_gps_pod", False):
        body = body.cut(gps_entry_cutter(L))
    if gns:
        body = _cut(body, gns_hole_cutters(L))
    if getattr(L, "lp_rocket", False) or getattr(L, "has_release_lead", False):   # rocket-carrier round: wall holes
        body = _cut(body, wall_hole_cutters(L))
    if getattr(L, "has_av_windows", False):        # v0.9 CAD-2 VTOL: hexagonal lightening / cooling windows, cut last
        body = _cut(body, avionics_window_cutters(L))
    if with_livery and getattr(L, "has_livery", False):   # livery: the 0.42 mm logo pocket (the orange part fills it)
        body = _cut(body, livery_inlay_solids(L, L.livery_r + OVERLAP))
    return body


def _face_vz(outer: Sequence[tuple[float, float]], holes: Sequence[Sequence[tuple[float, float]]], u: float) -> cq.Face:
    """livery: planar face in the plane x = u of the theta = 0 frame, (v, z) -> (u, v, z), with hole rings."""
    def wire(ring):
        pts = [V(u, v, z) for v, z in ring]
        return cq.Wire.makePolygon(pts + [pts[0]])
    return cq.Face.makeFromWires(wire(outer), [wire(h) for h in holes])


def livery_inlay_solids(L: Layout, r_out: float | None = None) -> list[cq.Shape]:
    """livery (user decisions 2026-11-04): one solid per logo island and copy - the (v, z) polygon of
    ``Layout.livery_polys_vz`` (pre-warped, hole rings kept) extruded radially at the copy azimuth over
    ``Layout.livery_u`` (it covers the band), INTERSECT the annular sector r [livery_r_in, r_out] over
    ``Layout.livery_sector`` - i.e. the logo's exact outline on the outer skin, ``livery_depth`` deep; the sides are
    parallel to the copy's radial direction (within asin(9.2 / 60) = 8.8 deg of radial at the logo's edges, so the
    hidden inner outline is up to 0.07 mm wider - the slicer sees one outer line).
    r_out = body_r (default): the orange inlay itself; body_r + OVERLAP: the pocket cutter of the white body."""
    r_out = L.livery_r if r_out is None else r_out
    u0, u1 = L.livery_u
    out = []
    for th in L.livery_thetas:
        lo, hi, z0, z1 = L.livery_sector[th]
        band = _sector(L.livery_r_in, r_out, lo, hi, z0, z1)
        for outer, holes in L.livery_polys_vz:
            prism = cq.Solid.extrudeLinear(_face_vz(outer, holes, u0), V(u1 - u0, 0, 0))
            out.append(_rot_z(prism, th).intersect(band))
    return out


def avionics_logo(L: Layout) -> cq.Shape:
    """livery: the orange inlay part (``livery.inlay_part``) - every island of every copy, one Compound of disjoint
    solids; each lies inside the plain module's wall (C36 measures it) and fills the white body's pocket exactly."""
    return cq.Compound.makeCompound(livery_inlay_solids(L))


def avionics_window_cutters(L: Layout) -> list[cq.Shape]:
    """v0.9 CAD-2 VTOL: one hexagon prism per window (Layout.avw_vz in the (v, z) plane) extruded radially over
    [body_id/2 - OVERLAP, body_r + OVERLAP] at avw_thetas."""
    return [_prism_vz(L.avw_vz, L.body_id / 2 - OVERLAP, L.body_r + OVERLAP, th) for th in L.avw_thetas]


def avionics_tray(L: Layout) -> cq.Shape:
    """SPEC 5.2 tray plate with the board holes (v0.6.3: the NAV board at the v0.5 FC spot, same holes) and battery / ESC
    strap slots (printed flat)."""
    am = L.p["avionics_module"]
    x0, x1 = L.tray_x - L.tray_t / 2, L.tray_x + L.tray_t / 2
    plate = _box(x0, x1, -L.tray_w / 2, L.tray_w / 2, L.z_tray_bottom, L.z_tray_top)
    cutters: list[cq.Shape] = []
    half = L.fc_hole_pattern / 2
    if getattr(L, "tray_nav_holes", True):       # v0.9 CAD-2: no NAV board (no holes) in the VTOL configuration
        for sy in (+1.0, -1.0):
            for sz in (+1.0, -1.0):
                cutters.append(cq.Solid.makeCylinder(L.fc_hole_d / 2, L.tray_t + 2 * OVERLAP,
                                                     V(x0 - OVERLAP, sy * half, L.nav_center_z + sz * half), V(1, 0, 0)))
    if getattr(L, "has_tray_holes", False):      # v0.9 CAD-2 VTOL: two tall lightening holes beside the centre spine
        (yi, yo), (hz0, hz1) = L.tray_hole_y, L.tray_hole_z
        cutters += [_box(x0 - OVERLAP, x1 + OVERLAP, yi, yo, hz0, hz1), _box(x0 - OVERLAP, x1 + OVERLAP, -yo, -yi, hz0, hz1)]
    sw, sh = am["strap_slot_w"], am["strap_slot_h"]     # strap width runs along Z, slot thickness along y

    def slot(yc: float, zc: float) -> cq.Shape:
        return _box(x0 - OVERLAP, x1 + OVERLAP, yc - sh / 2, yc + sh / 2, zc - sw / 2, zc + sw / 2)

    bz0, bz1 = L.battery_z
    for yc in (L.tray_w / 2 - 10.0, -(L.tray_w / 2 - 10.0)):   # y = +-40 for tray_w 100 (SPEC: "y = +-40")
        cutters.append(slot(yc, bz0 + 25.0))
        cutters.append(slot(yc, bz1 - 25.0))
    ez = (L.esc_z[0] + L.esc_z[1]) / 2
    for yc in (L.tray_w / 2 - 15.0, -(L.tray_w / 2 - 15.0)):   # y = +-35 for tray_w 100 (SPEC: "y = +-35")
        cutters.append(slot(yc, ez))
    return _cut(plate, cutters)


def battery(L: Layout) -> cq.Shape:
    return _box(*L.battery_x, *L.battery_y, *L.battery_z)


def esc(L: Layout) -> cq.Shape:
    return _box(*L.esc_x, *L.esc_y, *L.esc_z)


def nav(L: Layout) -> cq.Shape:
    """v0.6.3 SPEC 6: NAV board box (Layout.nav_x/y/z, the v0.5 flight-computer spot on the tray's +x face)."""
    return _box(*L.nav_x, *L.nav_y, *L.nav_z)


def fcu_board(L: Layout) -> cq.Shape:
    """v0.6.3 SPEC 6: FCU board box (Layout.fcu_x/y/z) flat on the intake bulkhead top, foam_t above it."""
    return _box(*L.fcu_x, *L.fcu_y, *L.fcu_z)


def misc_electronics(L: Layout) -> cq.Shape:
    """SPEC 6: misc-electronics box (Layout.misc_x/y/z: tray -x face above the battery)."""
    return _box(*L.misc_x, *L.misc_y, *L.misc_z)


def com_board(L: Layout) -> cq.Shape:
    """v0.6 SPEC 6: COM-board box (Layout.com_board_x/y/z) hanging from the boss tips under the payload bulkhead."""
    return _box(*L.com_board_x, *L.com_board_y, *L.com_board_z)


def radio_log_box(L: Layout) -> cq.Shape:
    """v0.6.2 SPEC 6: radio / logger box (Layout.radio_log_x/y/z) hanging from the payload-bulkhead underside beside the
    COM board."""
    return _box(*L.radio_log_x, *L.radio_log_y, *L.radio_log_z)


def joint_screw_axis_proxies(L: Layout) -> list[cq.Shape]:
    """v0.6.2 check helper (C24): one radial cylinder Ø joint_hole_outer_d per Layout.joint_screw_axes entry, from the
    vehicle axis out to body_r at the screw station - every screw of any length lies inside it."""
    d = L.p["body"]["joint_hole_outer_d"]
    return [_radial_cyl(d, 0.0, L.body_r, th, a[2]) for th, a, _ in L.joint_screw_axes()]


def com_bosses(L: Layout) -> list[cq.Shape]:
    """v0.6 SPEC 5.3: one boss Ø com_boss_d per Layout.com_boss_xy from the boss tip up to the bulkhead underside, run
    OVERLAP on into the (solid) bulkhead so the union shares volume instead of a face."""
    return [_cyl_z(L.com_boss_d / 2, L.z_com_boss_tip, L.z_com_boss_top + OVERLAP, x, y) for x, y in L.com_boss_xy]


def com_hole_cutters(L: Layout) -> list[cq.Shape]:
    """v0.6 SPEC 5.3: blind Ø com_hole_d tap hole per boss, from OVERLAP below the boss tip (over-travel into air) up
    to its flat end z_com_hole_end (com_hole_depth above the tip, below the bulkhead top)."""
    return [_cyl_z(L.com_hole_d / 2, L.z_com_boss_tip - OVERLAP, L.z_com_hole_end, x, y) for x, y in L.com_boss_xy]


# --------------------------------------------------------------------------- #
# SPEC 5.3 - payload module, piston, spring, CanSat, release pod, latch arm
# --------------------------------------------------------------------------- #
def release_pod(L: Layout) -> cq.Shape:
    """Release-servo cup on the payload tube (SPEC 5.3), alone."""
    return _pod_cup(L, theta=L.release_pod_theta, wall_r=L.body_r, r_in=L.release_pod_r_in,
                    r_out=L.release_pod_r_out, cav_r_in=L.release_pod_cavity_r_in,
                    cav_r_out=L.release_pod_cavity_r_out, z0=L.release_pod_z0,
                    floor_top_z=L.release_pod_floor_top_z, rim_z=L.release_pod_rim_z)


# --------------------------------------------------------------------------- #
# v0.8 GPS pod (Layout._gps_pod; payload_module.gps_pod, avionics_module.gps_entry)
# --------------------------------------------------------------------------- #
def _poly_pod(pts_uv: Sequence[tuple[float, float]], z0: float, z1: float, theta: float) -> cq.Shape:
    """Plan polygon in the pod frame (u along X, v along Y) extruded from z0 to z1, then rotated to theta."""
    wp = cq.Workplane("XY").workplane(offset=z0).polyline(list(pts_uv)).close().extrude(z1 - z0)
    return _rot_z(wp.val(), theta)


def gps_pod_outer(L: Layout) -> cq.Shape:
    """v0.8: the pod's outer box, u gps_u_back (its corners on the bore circle) .. gps_u_out, |v| <= gps_w_h, Z gps_z_lo ..
    gps_z_rim (JOIN; everything below the floor underside, inside the bore and inside the cavity is cut afterwards)."""
    return _box_polar(L.gps_u_back, L.gps_u_out, -L.gps_w_h, L.gps_w_h, L.gps_z_lo, L.gps_z_rim, L.gps_theta)


def gps_cavity_cutter(L: Layout) -> cq.Shape:
    """v0.8: the cavity - the (u, z) polygon above the floor's top line, u from 1 mm inside the outer box's back to the
    outer wall's inner face, up to gps_cut_z[1], over |v| <= gps_cav_hw (CUT; it opens the tube wall, which
    gps_tube_restore joins back)."""
    u0 = L.gps_u_back - 1.0
    pts = [(u0, L.gps_floor_top_z(u0)), (L.gps_u_cav_out, L.gps_z_floor_top_out),
           (L.gps_u_cav_out, L.gps_cut_z[1]), (u0, L.gps_cut_z[1])]
    return _prism_uz(pts, -L.gps_cav_hw, L.gps_cav_hw, L.gps_theta)


def gps_bosses(L: Layout) -> list[cq.Shape]:
    """v0.8: the two lid-screw bosses filling the outer cavity corners, from the floor underside at their inner face up
    to the rim (JOIN; the underside cut trims what they put below the floor)."""
    (u0, u1), (v0, v1) = L.gps_boss_u, L.gps_boss_v
    return [_box_polar(u0, u1, a, b, L.gps_boss_z0, L.gps_z_rim, L.gps_theta) for a, b in ((v0, v1), (-v1, -v0))]


def gps_underside_cutter(L: Layout) -> cq.Shape:
    """v0.8: everything below the floor's underside line (floor_slope_deg), u from 1 mm inside the outer box's back to 1 mm
    past the outer wall, |v| <= gps_w_h + 1 (CUT; it also cuts the tube wall under the pod, joined back by
    gps_tube_restore)."""
    ua, ub = L.gps_u_back - 1.0, L.gps_u_out + 1.0
    pts = [(ua, L.gps_floor_under_z(ua)), (ub, L.gps_floor_under_z(ub)), (ub, L.gps_cut_z[0]), (ua, L.gps_cut_z[0])]
    return _prism_uz(pts, -(L.gps_w_h + 1.0), L.gps_w_h + 1.0, L.gps_theta)


def gps_corner_pts(L: Layout, sign: float) -> list[tuple[float, float]]:
    """v0.8: plan triangle (pod frame) outside the 45 deg chamfer of the outer vertical edge on the side ``sign`` (+1 /
    -1): its hypotenuse is the chamfer line u + |v| = gps_u_out + gps_w_h - gps_corner_c, extended 1 mm past both faces."""
    u1, w, c = L.gps_u_out, L.gps_w_h, L.gps_corner_c
    return [(u1 - c - 1.0, sign * (w + 1.0)), (u1 + 1.0, sign * (w + 1.0)), (u1 + 1.0, sign * (w - c - 1.0))]


def gps_corner_cutters(L: Layout, z0: float, z1: float) -> list[cq.Shape]:
    """v0.8: the two corner-chamfer cutters (CUT) over Z [z0, z1]."""
    return [_poly_pod(gps_corner_pts(L, s), z0, z1, L.gps_theta) for s in (1.0, -1.0)]


def gps_tube_restore(L: Layout) -> cq.Shape:
    """v0.8: the payload tube wall (r body_id/2 .. body_r) over the pod sector +- gps_restore_half_deg and gps_cut_z,
    joined back after the cavity and underside cuts opened it (JOIN)."""
    a = L.gps_restore_half_deg
    return _sector(L.body_id / 2, L.body_r, L.gps_theta - a, L.gps_theta + a, *L.gps_cut_z)


def gps_bore_cutter(L: Layout) -> cq.Shape:
    """v0.8: the payload bore over gps_cut_z, re-cut (CUT) - the outer box and the floor reach into it."""
    return _cyl_z(L.body_id / 2, *L.gps_cut_z)


def gps_boss_hole_cutters(L: Layout) -> list[cq.Shape]:
    """v0.8: blind lid-screw hole Ø gps_boss_hole_d per boss, from its end gps_boss_hole_z0 up through the rim (CUT)."""
    out = []
    for u, v in L.gps_boss_hole_uv:
        x, y, _ = L.gps_xyz(u, v, 0.0)
        out.append(_cyl_z(L.gps_boss_hole_d / 2, L.gps_boss_hole_z0, L.gps_z_rim + OVERLAP, x, y))
    return out


def gps_raceway_solids(L: Layout) -> list[cq.Shape]:
    """v0.8: the raceway tunnel's outer box (from the payload module's bottom edge up to the floor top at its outer face)
    and the connector pocket's outer box at the bottom (JOIN); their back faces sit in the tube wall."""
    t = L.gps_theta
    return [_box_polar(*L.gps_rw_solid_u, -L.gps_rw_solid_hv, L.gps_rw_solid_hv, *L.gps_rw_z, t),
            _box_polar(*L.gps_pk_solid_u, -L.gps_pk_solid_hv, L.gps_pk_solid_hv, *L.gps_pk_solid_z, t)]


def gps_raceway_cutters(L: Layout) -> list[cq.Shape]:
    """v0.8: the raceway void (opens into the cavity through the floor) and the connector pocket void (open at the payload
    module's bottom edge), both starting gps_pod.raceway.skin_over_tube outside the tube (CUT)."""
    t = L.gps_theta
    return [_box_polar(*L.gps_rw_void_u, -L.gps_rw_void_hv, L.gps_rw_void_hv, *L.gps_rw_void_z, t),
            _box_polar(*L.gps_pk_void_u, -L.gps_pk_void_hv, L.gps_pk_void_hv, L.gps_pk_z[0] - OVERLAP, L.gps_pk_z[1], t)]


def gps_pod_features(L: Layout, body: cq.Shape) -> cq.Shape:
    """v0.8: the GPS pod, raceway and connector pocket applied to the finished v0.6 payload module, in the Fusion script's
    order (build_gps_pod): JOIN outer box, CUT cavity, JOIN bosses, CUT underside, CUT corner chamfers, JOIN tube wall,
    CUT bore, CUT boss holes, JOIN raceway + pocket, CUT their voids."""
    body = body.fuse(gps_pod_outer(L)).clean()
    body = body.cut(gps_cavity_cutter(L))
    body = _fuse([body] + gps_bosses(L))
    body = body.cut(gps_underside_cutter(L))
    body = _cut(body, gps_corner_cutters(L, L.gps_cut_z[0], L.gps_cut_z[1]))
    body = body.fuse(gps_tube_restore(L)).clean()
    body = body.cut(gps_bore_cutter(L))
    body = _cut(body, gps_boss_hole_cutters(L))
    body = _fuse([body] + gps_raceway_solids(L))
    return _cut(body, gps_raceway_cutters(L))


def gps_entry_cutter(L: Layout) -> cq.Shape:
    """v0.8: the avionics tube's lead entry hole Ø gps_entry_d, radial at gps_entry_theta / gps_entry_z (CUT)."""
    return _radial_cyl(L.gps_entry_d, L.body_id / 2 - 2 * OVERLAP, L.body_r + 2 * OVERLAP, L.gps_entry_theta, L.gps_entry_z)


def gps_pod_lid(L: Layout) -> cq.Shape:
    """v0.8 printed part: the pod lid (flush with the pod walls, inner edge an arc gps_lid_r_in around the tube, the same
    corner chamfers) with the three posts that carry the ground plane (blind Ø post_hole_d holes from below) and the two
    lid-screw clearance holes.  Order as the Fusion script: plate, CUT tube arc, CUT chamfers, JOIN posts, CUT post
    holes, CUT screw holes."""
    t = L.gps_theta
    lid = _box_polar(L.gps_lid_u_back - 1.0, L.gps_u_out, -L.gps_w_h, L.gps_w_h, L.gps_z_rim, L.gps_z_lid_top, t)
    lid = lid.cut(_cyl_z(L.gps_lid_r_in, L.gps_z_rim - OVERLAP, L.gps_z_lid_top + OVERLAP))
    lid = _cut(lid, gps_corner_cutters(L, L.gps_z_rim - OVERLAP, L.gps_z_lid_top + OVERLAP))
    posts = []
    for u, v in L.gps_post_uv:
        x, y, _ = L.gps_xyz(u, v, 0.0)
        posts.append(_cyl_z(L.gps_post_d / 2, L.gps_post_z[0], L.gps_post_z[1] + 0.5 * L.gps_lid_t, x, y))
    lid = _fuse([lid] + posts)
    cutters = []
    for u, v in L.gps_post_uv:
        x, y, _ = L.gps_xyz(u, v, 0.0)
        cutters.append(_cyl_z(L.gps_post_hole_d / 2, L.gps_post_z[0] - OVERLAP, L.gps_post_hole_z1, x, y))
    for u, v in L.gps_boss_hole_uv:
        x, y, _ = L.gps_xyz(u, v, 0.0)
        cutters.append(_cyl_z(L.gps_lid_screw_d / 2, L.gps_z_rim - OVERLAP, L.gps_z_lid_top + OVERLAP, x, y))
    return _cut(lid, cutters)


def gps_module(L: Layout) -> cq.Shape:
    """v0.8 placeholder ``gps``: the ceramic patch on the ground plane and the GPS board (GY-NEO6MV2 dims; the bought NEO-7M board is not measured, A) under it (two boxes, one
    compound; electronics.gps.board_mass_g spread over both by volume)."""
    t = L.gps_theta
    (pu, pv, pz), (bu, bv, bz) = L.gps_patch_box, L.gps_board_box
    return cq.Compound.makeCompound([_box_polar(*pu, *pv, *pz, t), _box_polar(*bu, *bv, *bz, t)])


def gps_ground_plane(L: Layout) -> cq.Shape:
    """v0.8 placeholder ``gps_ground_plane``: the FR4 plate with the two outer corner notches."""
    t = L.gps_theta
    (u0, u1), (v0, v1), (z0, z1) = L.gps_gp_box
    n = L.gps_gp_notch
    plate = _box_polar(u0, u1, v0, v1, z0, z1, t)
    notches = [_box_polar(u1 - n, u1 + OVERLAP, a, b, z0 - OVERLAP, z1 + OVERLAP, t)
               for a, b in ((v1 - n, v1 + OVERLAP), (v0 - OVERLAP, v0 + n))]
    return _cut(plate, notches)


def payload_module(L: Layout, with_gps: bool = True) -> cq.Shape:
    """SPEC 5.3: tube, top spigot (no holes), bulkhead with spring-seat boss and holes, release pod; v0.6: the four
    COM-board bosses under the bulkhead with their blind tap holes (appended after the v0.5 primitives / cutters); v0.8
    (``Layout.has_gps_pod`` and ``with_gps``): the GPS pod, raceway and connector pocket applied to that finished solid
    (gps_pod_features), so the v0.6 solid is the same with them removed."""
    pl = L.p["payload_module"]
    parts = [
        _tube(L.body_r, L.body_id / 2, L.z_payload_bottom, L.z_payload_spigot_bottom),
        _tube(L.spigot_od / 2, L.spigot_id / 2, L.z_payload_spigot_bottom, L.z_payload_spigot_top),
        _spigot_shoulder(L, L.z_payload_spigot_bottom, spigot_shoulder_r_in(L)),
        _cyl_z(L.body_id / 2 + OVERLAP / 2, L.z_payload_bulkhead_bottom, L.z_payload_bulkhead_top),
        _cyl_z(L.spring_seat_boss_d / 2, L.z_payload_bulkhead_top, L.z_payload_bulkhead_top + pl["spring_seat_boss_h"]),
        release_pod(L),
    ]
    if L.has_com_board:
        parts += com_bosses(L)
    body = _fuse(parts)
    z_floor = pod_z_floor(L, L.release_pod_z0, L.release_pod_r_out, L.body_r)
    body = body.cut(_cyl_z(L.body_id / 2, z_floor - OVERLAP, L.release_pod_rim_z + OVERLAP))  # bore re-cut
    cutters: list[cq.Shape] = []
    for th in L.joint_thetas:
        cutters.append(_radial_cyl(L.p["body"]["joint_hole_outer_d"], L.body_id / 2 - 2 * OVERLAP,
                                   L.body_r + 2 * OVERLAP, th, L.z_payload_bottom + L.joint_screw_dz))
    n = pl["bulkhead_hole_count"]
    for k in range(n):
        c = _polar(pl["bulkhead_hole_r"], 0.0, BULKHEAD_HOLE_THETA0 + k * 360.0 / n, 0.0)
        cutters.append(_cyl_z(pl["bulkhead_hole_d"] / 2, L.z_payload_bulkhead_bottom - OVERLAP,
                              L.z_payload_bulkhead_top + OVERLAP, c.x, c.y))
    cutters += _ear_hole_cutters(L, L.release_pod_theta, L.release_servo_axis_r, L.release_pod_rim_z)
    if L.has_com_board:
        cutters += com_hole_cutters(L)
    body = _cut(body, cutters)
    if with_gps and getattr(L, "has_gps_pod", False):
        body = gps_pod_features(L, body)
    return body


def piston(L: Layout) -> cq.Shape:
    pl = L.p["payload_module"]
    disc = _cyl_z(L.piston_d / 2, L.z_piston_bottom, L.z_piston_top)
    recess = _cyl_z(pl["piston_recess_d"] / 2, L.z_piston_bottom - OVERLAP, L.z_piston_bottom + pl["piston_recess_depth"])
    return disc.cut(recess)


def spring(L: Layout) -> cq.Shape:
    pl = L.p["payload_module"]
    return _tube(pl["spring_od"] / 2, pl["spring_od"] / 2 - pl["spring_wire_d"], L.z_spring_bottom, L.z_spring_top)


def cansat_chute_block_top(L: Layout) -> float:
    """Top of the CanSat's own chute block = z_can_top + payload.chute_pack_h.

    layout.py assigns ``z_chute_pack_top`` twice (payload chute block, then the nose chute
    pack), so the payload value is re-derived here from params.
    """
    return L.z_can_top + L.p["payload"]["chute_pack_h"]


def cansat_payload(L: Layout) -> cq.Shape:
    can = _cyl_z(L.can_d / 2, L.z_can_bottom, L.z_can_top)
    top = cansat_chute_block_top(L)
    if top > L.z_can_top + 1e-9:
        can = can.fuse(_cyl_z(L.can_d / 2, L.z_can_top, top)).clean()
    return can


def payload_envelope(L: Layout) -> cq.Shape:
    """Check volume (not a part): the competition payload envelope cylinder."""
    return _cyl_z(L.envelope_d / 2, L.z_envelope_bottom, L.z_envelope_top)


def latch_axis_xy(L: Layout) -> tuple[float, float]:
    a = _polar(L.release_servo_axis_r, L.servo_shaft_tangential_offset, L.release_pod_theta, 0.0)
    return a.x, a.y


def latch_lug_centre_xy(L: Layout) -> tuple[float, float]:
    c = _polar((L.lug_r_in + L.lug_r_out) / 2, 0.0, L.latch_lug_theta, 0.0)
    return c.x, c.y


def latch_arm_tip_xy(L: Layout, released: bool = False) -> tuple[float, float]:
    ax, ay = latch_axis_xy(L)
    cx, cy = latch_lug_centre_xy(L)
    ang = math.atan2(cy - ay, cx - ax)
    if released:
        ang += math.radians(_release_rotation_deg(L))
    return ax + L.latch_arm_len * math.cos(ang), ay + L.latch_arm_len * math.sin(ang)


def _release_rotation_deg(L: Layout) -> float:
    """Signed release rotation: the sense that swings the arm tip outward (away from the lug/skirt)."""
    travel = L.p["mission"]["deploy_servo_travel_deg"]
    ax, ay = latch_axis_xy(L)
    cx, cy = latch_lug_centre_xy(L)
    ang = math.atan2(cy - ay, cx - ax)
    best, best_r = travel, -1.0
    for s in (+1.0, -1.0):
        a = ang + math.radians(s * travel)
        tx, ty = ax + L.latch_arm_len * math.cos(a), ay + L.latch_arm_len * math.sin(a)
        r = math.hypot(tx, ty)
        if r > best_r:
            best, best_r = s * travel, r
    return best


def latch_arm(L: Layout, released: bool = False) -> cq.Shape:
    """SPEC 5.3 latch arm: bar from the servo axis toward the lug centre, O6 hub at the axis."""
    pl = L.p["payload_module"]
    ax, ay = latch_axis_xy(L)
    cx, cy = latch_lug_centre_xy(L)
    ang = math.degrees(math.atan2(cy - ay, cx - ax))
    z0, z1 = L.z_latch_arm_bottom, L.z_latch_arm_top
    bar = _box(0.0, L.latch_arm_len, -L.latch_arm_w / 2, L.latch_arm_w / 2, z0, z1)
    hub = _cyl_z(LATCH_HUB_D / 2, z0, z1)
    arm = _fuse([bar, hub]).rotate(V(0, 0, 0), V(0, 0, 1), ang).translate(V(ax, ay, 0))
    if released:
        arm = arm.rotate(V(ax, ay, 0), V(ax, ay, 1), _release_rotation_deg(L))
    del pl
    return arm


# --------------------------------------------------------------------------- #
# SPEC 5.4 - nose cone, lug, chute pack
# --------------------------------------------------------------------------- #
def latch_lug(L: Layout, overlap: float = OVERLAP) -> cq.Shape:
    """Lug block r in [lug_r_in, lug_r_out]; ``overlap`` extends it into the skirt wall so the
    union shares volume (a box face against the cylinder would only touch along a line)."""
    return _box_polar(L.lug_r_in - overlap, L.lug_r_out, -L.lug_w / 2, L.lug_w / 2,
                      L.z_lug_bottom, L.z_lug_top, L.latch_lug_theta)


def nose_outer_solid(L: Layout, n: int = 60) -> cq.Shape:
    """Filled Haack body: spline through Layout.nose_profile_points revolved about the axis.

    The last profile point (0, z_tip) is replaced by (NOSE_TIP_FLAT_R, z_tip) and closed with a
    radial line, so the tip is a O0.2 mm flat rather than a degenerate apex (SPEC 5.4 allows a
    far larger optional 3 mm blunt; that one is not applied).
    """
    pts = L.nose_profile_points(n)
    pts[-1] = (NOSE_TIP_FLAT_R, L.z_nose_tip)
    wp = (cq.Workplane("XZ").moveTo(0.0, L.z_nose_base).lineTo(pts[0][0], pts[0][1])
          .spline([(r, z) for r, z in pts[1:]], includeCurrent=True)
          .lineTo(0.0, L.z_nose_tip).close())
    return wp.revolve(360.0, (0, 0, 0), (0, 1, 0)).val()


def nose_inner_points(L: Layout, n: int = 240) -> list[tuple[float, float]]:
    """Inner wall of the nose: the outer Haack profile offset by nose.wall along its NORMAL.

    This is what Fusion's ShellFeature produces (a true constant wall).  The offset curve is
    followed from the base to x = nose_len - NOSE_SOLID_TIP_LEN (or until it would reach the
    axis / fold back) and then closed straight to the axis, so the tip stays solid.
    """
    w = L.nose_wall
    Ln = L.nose_len
    h = 1e-4 * Ln
    pts: list[tuple[float, float]] = []
    x_end = Ln - NOSE_SOLID_TIP_LEN               # SPEC 5.4: the last 8 mm of the tip stay solid
    for k in range(n + 1):
        x = x_end * k / n
        x0, x1 = max(0.0, x - h), min(Ln, x + h)
        drdx = (L.nose_outer_radius(x1) - L.nose_outer_radius(x0)) / (x1 - x0)
        nrm = math.sqrt(1.0 + drdx * drdx)
        ri = L.nose_outer_radius(x) - w / nrm
        zi = L.z_nose_base + x + w * drdx / nrm
        if k == 0:                      # Haack slope is exactly 0 at the base: no numeric sliver vs the skirt bore
            ri, zi = L.nose_outer_radius(0.0) - w, L.z_nose_base
        if pts and (zi <= pts[-1][1] + 1e-6 or ri >= pts[-1][0] - 1e-9):
            break                                   # the offset curve folds back near the tip
        if ri <= 0.05:
            if pts:                                 # end on the axis by linear interpolation
                r_p, z_p = pts[-1]
                t = r_p / (r_p - ri) if r_p != ri else 1.0
                pts.append((0.0, z_p + t * (zi - z_p)))
            break
        pts.append((ri, zi))
    return pts


def nose_inner_solid(L: Layout, n: int = 240) -> cq.Shape:
    """Revolved inner void (normal-offset wall), extended below the base so the cut is clean."""
    pts = nose_inner_points(L, n)
    r0 = pts[0][0]
    if pts[-1][0] > 0.0:
        pts.append((0.0, pts[-1][1]))
    spline_pts = [(r, z) for r, z in pts[1:-1]]
    wp = (cq.Workplane("XZ").moveTo(0.0, L.z_nose_base - OVERLAP).lineTo(r0, L.z_nose_base - OVERLAP)
          .lineTo(r0, L.z_nose_base).spline(spline_pts, includeCurrent=True)
          .lineTo(0.0, pts[-1][1]).close())
    return wp.revolve(360.0, (0, 0, 0), (0, 1, 0)).val()


def nose_cone(L: Layout) -> cq.Shape:
    """SPEC 5.4: skirt + latch lug + shelled Haack body + anchor bar with tether eye."""
    no = L.p["nose"]
    outer = nose_outer_solid(L)
    shell = outer.cut(nose_inner_solid(L))
    skirt = _tube(L.body_r, L.nose_skirt_id / 2, L.z_nose_skirt_bottom, L.z_nose_skirt_top)
    # anchor bar ends inside the wall (mid-wall radius at the bar top), so the union never has
    # to merge a face lying on the spline-revolved outer surface
    z_bar_top = L.z_anchor_bar + no["anchor_bar_h"]
    r_clip = L.nose_outer_radius(z_bar_top - L.z_nose_base) - L.nose_wall / 2
    bar = _box(-L.body_r, L.body_r, -no["anchor_bar_w"] / 2, no["anchor_bar_w"] / 2,
               L.z_anchor_bar, z_bar_top).intersect(_cyl_z(r_clip, L.z_anchor_bar - OVERLAP, z_bar_top + OVERLAP))
    nose = _fuse([skirt, latch_lug(L), shell, bar])
    eye = _cyl_z(no["tether_eye_d"] / 2, L.z_anchor_bar - OVERLAP, L.z_anchor_bar + no["anchor_bar_h"] + OVERLAP)
    return nose.cut(eye)


def nose_cavity_volume_mm3(L: Layout, nose: cq.Shape | None = None) -> float:
    """Void above the anchor bar: filled outer body minus the shelled part, both clipped to Z >= bar top."""
    no = L.p["nose"]
    z_from = L.z_anchor_bar + no["anchor_bar_h"]
    outer = nose_outer_solid(L)
    nose = nose if nose is not None else nose_cone(L)
    big = L.body_r + 2 * OVERLAP
    clip = _box(-big, big, -big, big, z_from, L.z_nose_tip + OVERLAP)   # a box: a coaxial cylinder clip fails on the revolve
    v_outer = outer.intersect(clip).Volume()
    v_nose = nose.intersect(clip).Volume()
    if v_outer <= 0.0:
        raise RuntimeError("nose cavity measurement: boolean clip of the outer body failed")
    return v_outer - v_nose


def chute_pack(L: Layout) -> cq.Shape:
    return _cyl_z(L.chute_pack_d / 2, L.z_chute_pack_bottom, L.z_chute_pack_top)


# --------------------------------------------------------------------------- #
# v0.9 CAD-2 VTOL configuration: top cap, GPS on the cap, hybrid buffer pack
# --------------------------------------------------------------------------- #
def logic_collar(L: Layout) -> cq.Shape:
    """Switch-collar round (user decision 2026-11-05, '상단 캡과 한 몸으로 출력하는 보호 칼라를 달아줘'; Layout._logic_collar):
    the protective collar printed with the top cap - Ø 2 lp_col_ro from half the plate thickness (union overlap) to the
    rim lp_col_z[1], minus the bore Ø 2 lp_col_ri from the plate top up, minus the two finger notches on the throw axis
    (vehicle y).  Review fix (2026-11-05, F1 / F2): each notch is a U slot (``logic_collar_slot``): the slab |x - sx| <
    lp_col_notch_w / 2 from the U's centre lp_col_notch_zc up plus a round bottom (a cylinder Ø lp_col_notch_w along y,
    axis at lp_col_notch_zc), lowest point lp_col_notch_z0.  Fused into ``top_cap``."""
    sx, sy = L.lp_sw_xy
    zin = L.z_cap_plate_bottom + 0.5 * L.cap_plate_t
    z1 = L.lp_col_z[1]
    outer = _cyl_z(L.lp_col_ro, zin, z1, sx, sy)
    return _cut(outer, [_cyl_z(L.lp_col_ri, L.z_cap_top, z1 + OVERLAP, sx, sy),
                        logic_collar_slot(L, sy - L.lp_col_ro - OVERLAP, sy + L.lp_col_ro + OVERLAP)])


def logic_collar_slot(L: Layout, y0: float, y1: float, shrink: float = 0.0) -> cq.Shape:
    """Review fix (2026-11-05, F1 / F2): the collar's U notch slot between y0 and y1 (vehicle y, the throw axis) - the
    slab |x - sx| < lp_col_notch_w / 2 - shrink from lp_col_notch_zc to the rim + OVERLAP plus the round bottom (a
    cylinder of radius lp_col_notch_w / 2 - shrink along y, axis at (sx, lp_col_notch_zc)).  shrink > 0 gives the
    check probes (C41, the tests: the slot must be free of material)."""
    sx, _ = L.lp_sw_xy
    r = 0.5 * L.lp_col_notch_w - shrink
    zc = L.lp_col_notch_zc
    slab = _box(sx - r, sx + r, y0, y1, zc, L.lp_col_z[1] + OVERLAP)
    rnd = cq.Solid.makeCylinder(r, y1 - y0, V(sx, y0, zc), V(0.0, 1.0, 0.0))
    return slab.fuse(rnd).clean()


def top_cap(L: Layout, with_logic_hole: bool = True, with_logic_collar: bool | None = None) -> cq.Shape:
    """v0.9 CAD-2 VTOL top cap (Layout._top_cap): skirt r body_id/2..body_r over the avionics spigot, plate (full disc,
    flat top) z_cap_plate_bottom..z_cap_top; under the plate the hub (O cap_hub_d), cap_rib_n radial ribs (cap_rib_t, from
    cap_rib_r[0] inside the hub to cap_rib_r[1]) down to z_cap_under and the rim ring (cap_ring_r) down to
    z_cap_ring_bottom - the hub / ribs / ring reach half the plate thickness into it (union overlap); then the hardpoint
    hole (axis, through hub and plate), the plate holes (Layout.cap_holes) and the skirt joint holes (O joint_hole_outer_d at
    z_cap_skirt_bottom + joint_screw_dz, joint_thetas).  2026-11-05: ``with_logic_hole`` False leaves out the logic power
    switch hole (the flight-mass model while logic_power.in_flight_mass_model is false; the printed part has the hole).
    Switch-collar round (2026-11-05): the battery build's cap also carries the protective collar (``logic_collar``,
    Layout.has_lp_collar) - part of the logic-power kit, so ``with_logic_collar`` follows ``with_logic_hole`` unless
    given (the flight-mass model is the cap without the hole AND without the collar)."""
    if with_logic_collar is None:
        with_logic_collar = with_logic_hole
    zp = L.z_cap_plate_bottom
    zin = zp + 0.5 * L.cap_plate_t
    parts = [_tube(L.body_r, L.body_id / 2, L.z_cap_skirt_bottom, zp + OVERLAP),
             _cyl_z(L.body_r, zp, L.z_cap_top),
             _cyl_z(L.cap_hub_d / 2, L.z_cap_under, zin),
             _tube(L.cap_ring_r[1], L.cap_ring_r[0], L.z_cap_ring_bottom, zin)]
    for th in L.cap_rib_thetas:
        parts.append(_box_polar(L.cap_rib_r[0], L.cap_rib_r[1], -L.cap_rib_t / 2, L.cap_rib_t / 2, L.z_cap_under, zin, th))
    if with_logic_collar and getattr(L, "has_lp_collar", False):
        parts.append(logic_collar(L))
    body = _fuse(parts)
    cutters = [_cyl_z(L.cap_hardpoint_d / 2, L.z_cap_under - OVERLAP, L.z_cap_top + OVERLAP)]
    if getattr(L, "cap_hardpoint", False):     # v0.9 fix round: the load washer's flush recess in the hub / rib bottoms
        cutters.append(_cyl_z(L.cap_washer_recess[0] / 2, L.z_cap_under - OVERLAP, L.z_cap_under + L.cap_washer_recess[1]))
    for nm, (x, y, d) in L.cap_holes.items():
        if nm == "logic power switch" and not with_logic_hole:
            continue
        cutters.append(_cyl_z(d / 2, zp - OVERLAP, L.z_cap_top + OVERLAP, x, y))
    for th in L.joint_thetas:
        cutters.append(_radial_cyl(L.p["body"]["joint_hole_outer_d"], L.body_id / 2 - 2 * OVERLAP, L.body_r + 2 * OVERLAP,
                                   th, L.z_cap_skirt_bottom + L.joint_screw_dz))
    return _cut(body, cutters)


def cap_gps_module(L: Layout) -> cq.Shape:
    """v0.9 CAD-2 VTOL placeholder ``gps``: the patch on the ground plane and the GPS board (GY-NEO6MV2 dims; the bought NEO-7M board is not measured, A) taped on the cap top
    (two boxes, one compound)."""
    (px, py), (bx, by) = L.cap_patch_xy, L.cap_board_xy
    return cq.Compound.makeCompound([_box(*px, *py, *L.cap_patch_z), _box(*bx, *by, *L.cap_board_z)])


def cap_gps_ground_plane(L: Layout) -> cq.Shape:
    """v0.9 CAD-2 VTOL placeholder ``gps_ground_plane``: the square copper-clad plate taped on the cap top."""
    return _box(*L.cap_gp_xy[0], *L.cap_gp_xy[1], *L.cap_gp_z)


def buffer_pack(L: Layout) -> cq.Shape:
    """v0.9 CAD-2 VTOL 'hybrid' placeholder ``buffer_pack``: the 6S 650 mAh buffer in the battery slot."""
    return _box(*L.buffer_x, *L.buffer_y, *L.buffer_z)


def tether_box(L: Layout, name: str) -> cq.Shape:
    """v0.9 fix round placeholder: one tether-kit box (Layout.tether_boxes[name]) on the tray's -x side."""
    (x, y, z, _m) = L.tether_boxes[name]
    return _box(*x, *y, *z)


def tether_eyebolt(L: Layout) -> cq.Shape:
    """v0.9 fix round placeholder ``tether_eyebolt``: the M5 shank (O cap_bolt_d) from the cap top down cap_bolt_len and
    the eye above the cap (a ring O cap_eye_od, 4 mm thick in y, cap_eye_h tall, modelled as its bounding plate with
    the hole) - one solid."""
    zt = L.z_cap_top
    shank = _cyl_z(L.cap_bolt_d / 2, L.z_bolt_end, zt + OVERLAP)
    r = L.cap_eye_od / 2
    eye = cq.Solid.makeCylinder(r, 4.0, V(0, -2.0, zt + L.cap_eye_h - r), V(0, 1, 0)).fuse(
        _box(-0.5 * L.cap_bolt_d - 1.0, 0.5 * L.cap_bolt_d + 1.0, -2.0, 2.0, zt, zt + L.cap_eye_h - r)).clean()
    eye = eye.cut(cq.Solid.makeCylinder(r - 4.0, 4.0 + 2 * OVERLAP, V(0, -2.0 - OVERLAP, zt + L.cap_eye_h - r), V(0, 1, 0)))
    return _fuse([shank, eye])


def tether_load_washer(L: Layout) -> cq.Shape:
    """v0.9 fix round placeholder ``tether_load_washer``: D cap_washer_d x cap_washer_t aluminium in the flush recess."""
    return _tube(L.cap_washer_d / 2, L.cap_hardpoint_d / 2, L.z_cap_washer[0], L.z_cap_washer[1])


def tether_nut(L: Layout) -> cq.Shape:
    """v0.9 fix round placeholder ``tether_nut``: the M5 jam nut (hexagon cap_nut_af across flats, cap_nut_h) under the
    washer, with the M5 hole."""
    hexagon = cq.Workplane("XY").workplane(offset=L.z_cap_nut[0]).polygon(6, 2 * L.cap_nut_af / math.sqrt(3)) \
        .extrude(L.cap_nut_h).val()
    return hexagon.cut(_cyl_z(L.cap_bolt_d / 2 + 0.05, L.z_cap_nut[0] - OVERLAP, L.z_cap_nut[1] + OVERLAP))


# --------------------------------------------------------------------------- #
# v0.9 logic-power round (user decisions 2026-11-05, configs.vtol.logic_power; Layout._logic_power): the 1S logic pack at
# spot A, the MTS-102 switch through the top cap and the lead keep-outs - VTOL battery build only
# --------------------------------------------------------------------------- #
def logic_pack(L: Layout) -> cq.Shape:
    """Placeholder ``logic_pack``: the 1S pack box on the 6S pack's -x face (Layout.lp_pack_x / y / z)."""
    return _box(*L.lp_pack_x, *L.lp_pack_y, *L.lp_pack_z)


def logic_switch_below(L: Layout) -> cq.Shape:
    """Check helper: the switch under the plate - the inner nut (Ø lp_sw_nut_d, lp_sw_inner_nut_z), the body
    (lp_sw_body) and the solder lugs / heat shrink (lp_sw_lugs)."""
    sx, sy = L.lp_sw_xy
    return _fuse([_cyl_z(L.lp_sw_nut_d / 2, *L.lp_sw_inner_nut_z, sx, sy), _box(*L.lp_sw_body[0], *L.lp_sw_body[1],
                  *L.lp_sw_body[2]), _box(*L.lp_sw_lugs[0], *L.lp_sw_lugs[1], *L.lp_sw_lugs[2])])


def logic_switch_above(L: Layout) -> cq.Shape:
    """Check helper: the switch over the plate - the outer nut + washer (Ø lp_sw_nut_d, lp_sw_outer_nut_z), the
    bushing top (Ø lp_sw_bushing_d to lp_sw_bushing_top_z) and the lever (Ø lp_sw_lever_d to lp_sw_top_z)."""
    sx, sy = L.lp_sw_xy
    return _fuse([_cyl_z(L.lp_sw_nut_d / 2, *L.lp_sw_outer_nut_z, sx, sy),
                  _cyl_z(L.lp_sw_bushing_d / 2, L.lp_sw_outer_nut_z[1] - OVERLAP, L.lp_sw_bushing_top_z, sx, sy),
                  _cyl_z(L.lp_sw_lever_d / 2, L.lp_sw_bushing_top_z - OVERLAP, L.lp_sw_top_z, sx, sy)])


WALL_SW_BUSHING_D, WALL_SW_ABOVE, WALL_SW_LEVER_D = 6.0, 5.0, 3.0   # as configs.vtol.logic_power.switch (ASSUMED)


def wall_switch(L: Layout, name: str) -> cq.Shape:
    """rocket-carrier round (2026-11-14, Layout.lp_wall_sw): one avionics-tube wall switch as mounted - inner nut (Ø nut_d
    on the bore face, its rim on the curved bore), body and solder lugs inside the bore, the bushing (Ø 6.0) through the
    hole (no contact), the outer nut on the skin and the lever (Ø 3) out to lever_r[1]; one solid."""
    w = L.lp_wall_sw[name]
    th, z, ro = w["theta"], w["z"], L.body_r
    return _fuse([_radial_cyl(w["nut_d"], *w["r_nut"], th, z),
                  _box_polar(*w["r_body"], -w["body_t"] / 2, w["body_t"] / 2, z - w["body_a"] / 2, z + w["body_a"] / 2, th),
                  _box_polar(*w["r_lug"], -w["lug_t"] / 2, w["lug_t"] / 2, z - w["lug_a"] / 2, z + w["lug_a"] / 2, th),
                  _radial_cyl(WALL_SW_BUSHING_D, w["r_nut"][0] + OVERLAP, ro + WALL_SW_ABOVE, th, z),
                  _radial_cyl(w["nut_d"], ro, ro + 2.0, th, z),
                  _radial_cyl(WALL_SW_LEVER_D, ro + WALL_SW_ABOVE - OVERLAP, w["lever_r"][1], th, z)])


def wall_switch_inside(L: Layout, name: str) -> cq.Shape:
    """Check helper: the part of a wall switch inside the bore (inner nut, body, lugs)."""
    w = L.lp_wall_sw[name]
    th, z = w["theta"], w["z"]
    return _fuse([_radial_cyl(w["nut_d"], *w["r_nut"], th, z),
                  _box_polar(*w["r_body"], -w["body_t"] / 2, w["body_t"] / 2, z - w["body_a"] / 2, z + w["body_a"] / 2, th),
                  _box_polar(*w["r_lug"], -w["lug_t"] / 2, w["lug_t"] / 2, z - w["lug_a"] / 2, z + w["lug_a"] / 2, th)])


def wall_hole_cutters(L: Layout) -> list[cq.Shape]:
    """rocket-carrier round: the avionics-tube wall holes - the two switch holes (Layout.lp_wall_sw) and the release-lead
    entry hole (Layout.rl_entry_*), radial, through the wall only (CUT)."""
    out = [_radial_cyl(w["hole_d"], L.body_id / 2 - 2 * OVERLAP - 0.5, L.body_r + 2 * OVERLAP, w["theta"], w["z"])
           for w in (getattr(L, "lp_wall_sw", None) or {}).values()]
    if getattr(L, "has_release_lead", False):
        out.append(_radial_cyl(L.rl_entry_d, L.body_id / 2 - 2 * OVERLAP, L.body_r + 2 * OVERLAP, L.rl_entry_theta, L.rl_entry_z))
    return out


def release_lead_keepouts(L: Layout, bay_only: bool = True) -> dict[str, cq.Shape]:
    """Check helpers (C42): one cylinder Ø rl_od per segment of Layout.release_lead_polyline (from the bay entry when
    bay_only), extended by its radius at both ends; names 'release lead <k>' (the last enters the carrier envelope)."""
    r = L.rl_od / 2
    pts = L.release_lead_polyline[L.rl_bay_start if bay_only else 0:]
    return {f"release lead {k}": _cyl_between(L.rl_od, a, b, r, r) for k, (a, b) in enumerate(zip(pts[:-1], pts[1:]))}


def logic_switch(L: Layout) -> cq.Shape:
    """Placeholder ``logic_switch``: the MTS-102 as mounted - below the plate (inner nut, body, lugs), the bushing through
    the lp_sw_hole_d hole (Ø lp_sw_bushing_d, no contact with the hole wall) and above the plate (outer nut, bushing top,
    lever); one solid.  rocket-carrier round: in the rocket (Layout.lp_rocket) the switch in the avionics-tube wall."""
    if getattr(L, "lp_rocket", False):
        return wall_switch(L, "logic")
    sx, sy = L.lp_sw_xy
    bush = _cyl_z(L.lp_sw_bushing_d / 2, L.lp_sw_inner_nut_z[0] + OVERLAP, L.lp_sw_outer_nut_z[1], sx, sy)
    return _fuse([logic_switch_below(L), bush, logic_switch_above(L)])


def logic_lever_positions(L: Layout, n: int = 9) -> dict[float, cq.Shape]:
    """Switch-collar round check helper (C39 / C41): the lever (Ø lp_sw_lever_d, flat end) at ``n`` throw angles evenly
    from -lp_lever_throw_deg to +lp_lever_throw_deg (0 included for odd n), turned about the pivot axis (vehicle x
    through (sx, sy, lp_lever_pivot_z)) - each a cylinder from the bushing top's arm lp_lever_s0 to the tip lp_lever_len
    along (0, sin a, cos a).  Their union is the lever's swept envelope up to the sampling (the extreme corners sit at the
    end positions and near vertical, both sampled)."""
    sx, sy = L.lp_sw_xy
    out: dict[float, cq.Shape] = {}
    for k in range(n):
        a = -L.lp_lever_throw_deg + 2.0 * L.lp_lever_throw_deg * k / (n - 1)
        ar = math.radians(a)
        d = V(0.0, math.sin(ar), math.cos(ar))
        base = V(sx, sy + L.lp_lever_s0 * d.y, L.lp_lever_pivot_z + L.lp_lever_s0 * d.z)
        out[round(a, 6)] = cq.Solid.makeCylinder(0.5 * L.lp_sw_lever_d, L.lp_lever_len - L.lp_lever_s0, base, d)
    # the near-vertical angle where the tip end's upper corner peaks (Layout lp_lever_env_top_z)
    ap = math.degrees(math.atan2(0.5 * L.lp_sw_lever_d, L.lp_lever_len))
    if ap <= L.lp_lever_throw_deg:
        for a in (ap, -ap):
            ar = math.radians(a)
            d = V(0.0, math.sin(ar), math.cos(ar))
            base = V(sx, sy + L.lp_lever_s0 * d.y, L.lp_lever_pivot_z + L.lp_lever_s0 * d.z)
            out[round(a, 6)] = cq.Solid.makeCylinder(0.5 * L.lp_sw_lever_d, L.lp_lever_len - L.lp_lever_s0, base, d)
    return out


def logic_finger_probes(L: Layout, angle_deg: float | None = None) -> dict[str, cq.Shape]:
    """Switch-collar round check helper (C41): at each end position the fingertip sphere (Layout.lp_col_finger: Ø
    finger_tip_d, level with the outer corner of the lever's tip end, its radius outboard along the throw axis) fused
    with the finger behind it (a cylinder of the same Ø, body_len_mm further out along the throw axis, horizontal).
    Review fix (2026-11-05, F1): ``angle_deg`` given = the probe of the PUSH at that lever angle (positive = the lever
    tilted toward the probe's own side, the end position = lp_lever_throw_deg): the sphere on the lever's outboard tip
    corner (Layout lp_col_finger_path), one probe per side ('+y' pushes the lever tilted to +y back toward -y)."""
    fg = L.lp_col_finger
    r, ln = fg["sphere_r_mm"], fg["body_len_mm"]
    if angle_deg is None:
        cs = list(fg["centres"])
    else:
        sx, sy = L.lp_sw_xy
        ar = math.radians(angle_deg)
        yc = L.lp_lever_len * math.sin(ar) + 0.5 * L.lp_sw_lever_d * math.cos(ar) + r
        zc = L.lp_lever_pivot_z + L.lp_lever_len * math.cos(ar) - 0.5 * L.lp_sw_lever_d * math.sin(ar)
        cs = [(sx, sy + yc, zc), (sx, sy - yc, zc)]
    out = {}
    for (x, y, z), s, nm in zip(cs, (1.0, -1.0), ("+y", "-y")):
        sph = cq.Solid.makeSphere(r, V(x, y, z), angleDegrees1=-90, angleDegrees2=90)
        body = cq.Solid.makeCylinder(r, ln, V(x, y, z), V(0.0, s, 0.0))
        out[nm] = sph.fuse(body).clean()
    return out


def logic_lead_keepouts(L: Layout) -> dict[str, cq.Shape]:
    """Check helpers: one cylinder Ø lp_wire_od per segment of the two lead routes (Layout.lp_switch_lead_polyline,
    lp_pack_lead_polyline), each extended by its radius at both ends so the corners are covered.  Names
    '<lead> <k>' (k = segment index; the last segment of each enters its plug box inside the carrier envelope)."""
    r = L.lp_wire_od / 2
    out: dict[str, cq.Shape] = {}
    leads = [("switch lead", L.lp_switch_lead_polyline), ("pack lead", L.lp_pack_lead_polyline)]
    if getattr(L, "lp_rocket", False):     # rocket-carrier round: the arm switch lead to J16
        leads.append(("arm lead", L.lp_arm_lead_polyline))
    for nm, pts in leads:
        for k, (a, b) in enumerate(zip(pts[:-1], pts[1:])):
            out[f"{nm} {k}"] = _cyl_between(L.lp_wire_od, a, b, r, r)
    return out


def carrier_plug_boxes(L: Layout) -> dict[str, cq.Shape]:
    """Check helpers: the carrier PCB rev 2c's mated plug boxes (Layout.lp_plugs, from the PCB round's board metrics)."""
    return {nm: _box(*bx[0], *bx[1], *bx[2]) for nm, bx in L.lp_plugs.items()}


def logic_power_parts(L: Layout) -> dict[str, "Part"]:
    """The logic-power report components (VTOL battery build, Layout.has_logic_power): placeholders ``logic_pack`` (box,
    lp_pack_mass_g) and ``logic_switch`` (lp_sw_mass_g); lumped ``logic_switch_lead`` (wire + connector at the route
    centroid) and ``logic_retention`` (longer strap + tie).  The pack's own lead is part of its mass (params ASSUMED).
    With logic_power.in_flight_mass_model false they are NOT in build_all (pending-mass report only)."""
    if not getattr(L, "has_logic_power", False):
        return {}
    return {
        "logic_pack": Part("logic_pack", "placeholder", "logic_pack", logic_pack(L), "avionics_module", "",
                           COLORS["logic_pack"], mass_g=L.lp_pack_mass_g),
        "logic_switch": Part("logic_switch", "placeholder", "logic_switch", logic_switch(L),
                             "avionics_module" if getattr(L, "lp_rocket", False) else "top_cap", "",
                             COLORS["logic_switch"], mass_g=L.lp_sw_mass_g),
        "logic_switch_lead": Part("logic_switch_lead", "lumped", "logic_switch_lead", None, "avionics_module",
                                  mass_g=L.lp_switch_lead_mass_g, point_mm=tuple(L.lp_switch_lead_com)),
        "logic_retention": Part("logic_retention", "lumped", "logic_retention", None, "avionics_module",
                                mass_g=L.lp_retention_mass_g, point_mm=tuple(L.lp_retention_com)),
    }


def edf_tab(L: Layout, tab_theta: float) -> cq.Shape:
    """SPEC 3.7 / 6 EDF mounting tab: plate r [tab_r_in - 0.5, tab_r_out], tangential +-tab_t/2,
    Z [tab_z0, tab_z1]; three slots tab_slot_w (axial) x tab_slot_len_radial (radial) centred at
    r tab_slot_r, Z tab_slot_zs, through the plate thickness."""
    plate = _box(L.tab_r_in - EDF_TAB_INTO_HOUSING, L.tab_r_out, -L.tab_t / 2, L.tab_t / 2, L.tab_z0, L.tab_z1)
    h = L.tab_slot_len_radial / 2
    slots = [_box(L.tab_slot_r - h, L.tab_slot_r + h, -L.tab_t / 2 - OVERLAP, L.tab_t / 2 + OVERLAP,
                  zs - L.tab_slot_w / 2, zs + L.tab_slot_w / 2) for zs in L.tab_slot_zs]
    return _rot_z(plate.cut(*slots), tab_theta)


def edf_motor(L: Layout) -> cq.Shape:
    """SPEC 6 (tabs) motor cylinder O edf.motor_d for edf.motor_protrusion_rear below the housing."""
    e = L.p["edf"]
    return _cyl_z(e["motor_d"] / 2, L.z_edf_housing_bottom - e["motor_protrusion_rear"], L.z_edf_housing_bottom)


def edf_unit(L: Layout) -> cq.Shape:
    """SPEC 6: lip + housing ring + hub cylinder (hub is a separate solid -> Compound).

    lip_clamp (v0.4): lip disc O lip_od x edf_lip_t.  tabs (v0.5): the lip is a ring ID housing_id
    (envelope of the bell), the two tab plates are fused to the housing, and the motor cylinder
    is fused to the hub (face contact at z_edf_housing_bottom) -> still two solids."""
    e = L.p["edf"]
    housing = _tube(e["housing_od"] / 2, e["housing_id"] / 2, L.z_edf_housing_bottom, L.z_edf_lip_bottom)
    hub = _cyl_z(e["hub_d_ASSUMED"] / 2, L.z_edf_housing_bottom, L.z_edf_housing_bottom + EDF_HUB_LEN)
    if not L.tabs:
        lip = _cyl_z(e["lip_od"] / 2, L.z_edf_lip_bottom, L.z_edf_lip_top)
        return lip.fuse(housing).clean().fuse(hub)
    lip = _tube(e["lip_od"] / 2, e["housing_id"] / 2, L.z_edf_lip_bottom, L.z_edf_lip_top)
    front = _fuse([lip, housing] + [edf_tab(L, th) for th in L.tab_thetas])
    rear = _fuse([edf_motor(L), hub])
    return front.fuse(rear)


def exhaust_path(L: Layout) -> cq.Shape:
    """Check volume (C8): cylinder O(duct_id - 2) from the exit plane to the EDF housing bottom."""
    return _cyl_z((L.duct_id - 2.0) / 2, L.z_exit, L.z_edf_housing_bottom)


def tab_sweep(L: Layout, tab_theta: float) -> cq.Shape:
    """Check volume (C18): the tab plate (no slots) swept from its seat (bottom at tab_z0) up until
    its bottom face reaches keyway_z1, i.e. Z [tab_z0, keyway_z1 + tab length]."""
    return _box_polar(L.tab_r_in - EDF_TAB_INTO_HOUSING, L.tab_r_out, -L.tab_t / 2, L.tab_t / 2,
                      L.tab_z0, L.keyway_z1 + (L.tab_z1 - L.tab_z0), tab_theta)


def tab_screw_probe(L: Layout, tab_theta: float, z: float, d: float, t0: float, t1: float) -> cq.Shape:
    """Check volume (C19): cylinder O d on the tab-screw axis for t in [t0, t1]."""
    return _screw_axis_cyl(L, tab_theta, z, d, t0, t1)


def wire_bundle_steps(L: Layout) -> dict[str, tuple[float, float]]:
    """Z ranges of the two horizontal legs of the C20 bundle: each is WIRE_BUNDLE_T tall and centred in the
    window SPEC 7 allows for it - step out within [wire_gate_lo_z0, z_intake_skirt_top] (v0.5d: above the floor
    wedge, not from wire_ch_z0), step in within [wire_notch_z0, wire_ch_z1]."""
    T = WIRE_BUNDLE_T
    out_c = 0.5 * (L.wire_gate_lo_z0 + L.z_intake_skirt_top)
    in_c = 0.5 * (L.wire_notch_z0 + L.wire_ch_z1)
    return {"out": (out_c - T / 2, out_c + T / 2), "in": (in_c - T / 2, in_c + T / 2)}


def wire_bundle_legs(L: Layout) -> dict[str, cq.Shape]:
    """Check volumes (C20): the 12 x 5 mm proxy bundle as five straight legs (union = the sweep with
    square corners), in the pod frame at wire_theta, tangential +-WIRE_BUNDLE_W/2:

    A  up the propulsion wire slot: r [r_a, r_a + 5] with r_a = (edf_bore_r + sleeve_r)/2 (SPEC: the
       bundle's inner face), from wire_slot_z0 to the top of the step-out leg B;
    B  step out (wire_bundle_steps 'out', inside [wire_gate_lo_z0, z_intake_skirt_top]): r_a .. outer face of C;
    C  up the intake channel past the lip, centred between wire_ch_r0 and wire_ch_r1 (inner face >=
       wire_ch_r0), from B's bottom to D's top;
    D  step in (wire_bundle_steps 'in', inside [wire_notch_z0, wire_ch_z1]) through the shelf notch, from
       C to E;
    E  up through the bulkhead wire hole, centred between wire_hole_r0 and wire_hole_r1, from D's bottom
       to z_intake_bulkhead_top + OVERLAP."""
    T, hw = WIRE_BUNDLE_T, WIRE_BUNDLE_W / 2
    r_a = (L.edf_bore_r + L.sleeve_r) / 2
    c_ch = (L.wire_ch_r0 + L.wire_ch_r1) / 2
    c_hole = (L.wire_hole_r0 + L.wire_hole_r1) / 2
    (zb0, zb1), (zd0, zd1) = wire_bundle_steps(L)["out"], wire_bundle_steps(L)["in"]

    def leg(u0: float, u1: float, z0: float, z1: float) -> cq.Shape:
        return _box_polar(u0, u1, -hw, hw, z0, z1, L.wire_theta)

    return {
        "A": leg(r_a, r_a + T, L.wire_slot_z0, zb1),
        "B": leg(r_a, c_ch + T / 2, zb0, zb1),
        "C": leg(c_ch - T / 2, c_ch + T / 2, zb0, zd1),
        "D": leg(c_hole - T / 2, c_ch + T / 2, zd0, zd1),
        "E": leg(c_hole - T / 2, c_hole + T / 2, zd0, L.z_intake_bulkhead_top + OVERLAP),
    }


def motor_lead_legs(L: Layout) -> dict[str, cq.Shape]:
    """v0.9 check volumes (C20, open groove): the motor_leads.count bare leads side by side, each O od_ASSUMED, as a box
    (count x od + 2 x wire_clearance tangential, od + 2 x wire_clearance thick) along Layout.ml_route_uz in the wire_theta
    frame: A up the propulsion slot from the exit height, B steps out mid-skirt, C up the open groove, D steps in through
    the gate over the lip, E up through the bulkhead hole to OVERLAP above the bulkhead."""
    wcl = L.ml_wire_clear
    hw, h = 0.5 * L.ml_count * L.ml_od + wcl, 0.5 * L.ml_od + wcl
    k = L.ml_key
    ze, us, zo, ug, zg, ui = k["z_a0"], k["u_slot"], k["z_out"], k["u_g"], k["z_gate"], k["u_in"]
    th = L.wire_theta

    def box(u0, u1, z0, z1):
        return _box_polar(u0, u1, -hw, hw, z0, z1, th)

    out = {"A": box(us - h, us + h, ze, zo + h), "B": box(us - h, ug + h, zo - h, zo + h),
           "C": box(ug - h, ug + h, zo - h, zg + h), "D": box(ui - h, ug + h, zg - h, zg + h),
           "E": box(ui - h, ui + h, zg - h, L.z_intake_bulkhead_top + OVERLAP)}
    if getattr(L, "ml_lower_uz", None) and not getattr(L, "clock_mount", False):   # v0.9 lead-exit round: EDF-internal
        out.update(motor_lead_lower_legs(L))     # legs and bends (clock mount: every corner, C29 / clock_lead_env)
    return out


def _lead_arc(L: Layout, cu: float, cz: float, quadrant: str, inner: float, outer: float, hw: float) -> cq.Shape:
    """Quarter annulus (radii inner .. outer about (cu, cz) in the (u, z) plane at wire_theta), +-hw tangential:
    'lower_right' = u >= cu, z <= cz; 'upper_left' = u <= cu, z >= cz."""
    ring = cq.Solid.makeCylinder(outer, 2 * hw, V(cu, -hw, cz), V(0, 1, 0)).cut(
        cq.Solid.makeCylinder(inner, 2 * hw, V(cu, -hw, cz), V(0, 1, 0)))
    if quadrant == "lower_right":
        q = _box(cu, cu + outer + 1.0, -hw - 1.0, hw + 1.0, cz - outer - 1.0, cz)
    else:
        q = _box(cu - outer - 1.0, cu, -hw - 1.0, hw + 1.0, cz, cz + outer + 1.0)
    return _rot_z(ring.intersect(q), L.wire_theta)


def _lead_arc_span(L: Layout, cu: float, cz: float, a0: float, a1: float, inner: float, outer: float,
                   hw: float) -> cq.Shape:
    """Annular sector (radii inner .. outer about (cu, cz) in the (u, z) plane at wire_theta) for phi in [a0, a1] rad
    (phi measured from +u toward +z; the lead bend runs -pi/2 -> 0), +-hw tangential."""
    ring = cq.Solid.makeCylinder(outer, 2 * hw, V(cu, -hw, cz), V(0, 1, 0)).cut(
        cq.Solid.makeCylinder(inner, 2 * hw, V(cu, -hw, cz), V(0, 1, 0)))
    big = outer + 2.0
    k = max(2, int(math.ceil((a1 - a0) / math.radians(5.0))))
    wedge = [(cu, cz)] + [(cu + big / math.cos(0.5 * (a1 - a0) / k) * math.cos(a0 + (a1 - a0) * j / k),
                           cz + big / math.cos(0.5 * (a1 - a0) / k) * math.sin(a0 + (a1 - a0) * j / k)) for j in range(k + 1)]
    return _rot_z(ring.intersect(_prism_uz(wedge, -hw - 1.0, hw + 1.0)), L.wire_theta)


def motor_lead_lower_legs(L: Layout) -> dict[str, cq.Shape]:
    """v0.9 lead-exit, photo round check volumes (C20 / C29): the EDF-internal route of Layout._motor_lead_lower - S the
    heat-shrink bundle (O ml_bundle_d + 2 x wire_clearance, square in section: +-ml_hb both ways) radially from the motor
    side behind the stator vane and under the rim to the shrink end + the triangle -> row transition (Layout.
    ml_u_bundle_env_end; or the bend start), F the single leads side by side (count x od + 2 x wire_clearance tangential,
    od + 2 x wire_clearance thick) from the shrink end to the bend start (S and F overlap over the transition: its
    envelope is their union), b2
    the bend up into the slot (quarter annulus of the centre-line radius ml_bend_r +- h; where the shrink runs into the
    bend - ml_shrink_bend_margin < 0 - its first part is the bundle's sector R +- hb, +-hb)."""
    h, hw, hb, R = L.ml_h, L.ml_hw, L.ml_hb, L.ml_bend_r
    th = L.wire_theta
    zr, us, rm, use = L.ml_z_run, L.ml_u_slot, L.ml_r_motor, L.ml_u_shrink_end
    ube = getattr(L, "ml_u_bundle_env_end", use)     # fix round: shrink end + triangle -> row transition (G3)
    cu, cz = us - R, zr + R
    out = {"S": _box_polar(rm, min(ube, cu), -hb, hb, zr - hb, zr + hb, th)}
    if use < cu - 1e-9:                              # the row from the shrink end (over the transition: union with S)
        out["F"] = _box_polar(use, cu, -hw, hw, zr - h, zr + h, th)
    dphi = math.radians(L.ml_shrink_bend_phi_deg)
    if dphi > 1e-9:
        out["b2 bundle"] = _lead_arc_span(L, cu, cz, -0.5 * math.pi, -0.5 * math.pi + dphi, R - hb, R + hb, hb)
        out["b2"] = _lead_arc_span(L, cu, cz, -0.5 * math.pi + dphi, 0.0, R - h, R + h, hw)
    else:
        out["b2"] = _lead_arc(L, cu, cz, "lower_right", R - h, R + h, hw)
    return out


def motor_lead_dropin_sweep(L: Layout) -> dict[str, cq.Shape]:
    """v0.9 lead-exit, photo round (C29 assembly): the EDF goes into its bore from above with the leads already formed
    (bundle out of the motor's front ring behind the stator vane, under the rim, single leads bent up beside the housing):
    the volume each lower leg (and leg A) sweeps on the way down = the leg extended up to the module top; b2 without its
    lower-right corner outside the bend's outer edge (the corner the leads never occupy); where the heat-shrink runs into the
    bend, its part as a box (cu .. its outer end) +-hb from the bundle's bottom.  Every sweep x propulsion module must be
    empty."""
    h, hw, hb, R = L.ml_h, L.ml_hw, L.ml_hb, L.ml_bend_r
    th = L.wire_theta
    zr, us, rm, use = L.ml_z_run, L.ml_u_slot, L.ml_r_motor, L.ml_u_shrink_end
    top = L.z_prop_top + 2.0

    def box(u0, u1, z0, half=hw):
        return _box_polar(u0, u1, -half, half, z0, top, th)

    cu, cz = us - R, zr + R
    corner = _box(cu, us + h + 1.0, -hw - 1.0, hw + 1.0, zr - h - 1.0, cz).cut(
        cq.Solid.makeCylinder(R + h, 2 * hw + 4.0, V(cu, -hw - 2.0, cz), V(0, 1, 0)))
    b2 = _box(cu, us + h, -hw, hw, zr - h, top).cut(corner)
    ube = getattr(L, "ml_u_bundle_env_end", use)     # fix round: shrink end + triangle -> row transition (G3)
    out = {"S": box(rm, min(ube, cu), zr - hb, hb), "b2": _rot_z(b2, th), "A": box(us - h, us + h, L.ml_key["z_a0"])}
    if use < cu - 1e-9:
        out["F"] = box(use, cu, zr - h)
    dphi = math.radians(L.ml_shrink_bend_phi_deg)
    if dphi > 1e-9:
        out["b2 bundle"] = box(cu, cu + (R + hb) * math.sin(dphi), zr - hb, hb)
    # review 2026-10-29 (G4): the step-out B (formed before the drop, below the EDF lip) and the groove leg C go down
    # with the EDF too - their sweeps where they start below the module top
    k = L.ml_key
    if k["z_out"] - h < top:
        out["B"] = box(us - h, k["u_g"] + h, k["z_out"] - h)
        out["C"] = box(k["u_g"] - h, k["u_g"] + h, k["z_out"] - h)
    return out


def bullet_group(L: Layout, u: float, z0: float, z1: float) -> dict[str, cq.Shape]:
    """v0.9 check volumes (C29): one 5.5 mm bullet (O bullet.d + 2 x bullet_clearance) and the other count - 1 leads
    (O od) beside it, vertical at radius u in the wire_theta frame over Z [z0, z1] (the group centred tangentially)."""
    bd = L.ml_bullet_d + 2 * L.ml_bullet_clear
    w = bd + (L.ml_count - 1) * L.ml_od
    out = {"bullet": cq.Solid.makeCylinder(bd / 2, z1 - z0, V(*L.ml_xyz(u, -w / 2 + bd / 2, z0)), V(0, 0, 1))}
    for j in range(L.ml_count - 1):
        v = -w / 2 + bd + (j + 0.5) * L.ml_od
        out[f"lead {j + 1}"] = cq.Solid.makeCylinder(L.ml_od / 2, z1 - z0, V(*L.ml_xyz(u, v, z0)), V(0, 0, 1))
    return out


def motor_joint_zone(L: Layout) -> cq.Shape:
    """v0.9 check volume (C29): the three mated bullet pairs in the avionics bay (Layout.ml_joint_box)."""
    (x0, x1), (y0, y1), (z0, z1) = L.ml_joint_box
    return _box(x0, x1, y0, y1, z0, z1)


def wire_bundle_proxy(L: Layout) -> cq.Shape:
    """C20 proxy bundle: union of ``wire_bundle_legs``."""
    return _fuse(list(wire_bundle_legs(L).values()))


def servo_rebate_band(L: Layout) -> tuple[float, float]:
    """Z band of the leads in the rebate and of the step-out leg (C23): ribbon width sl_ribbon_w (the ribbons lie
    with their width along Z), SERVO_PROXY_GAP above the rebate floor."""
    z0 = L.sl_rebate_z0 + SERVO_PROXY_GAP
    return z0, z0 + L.sl_ribbon_w


def servo_lead_legs(L: Layout, k: int) -> dict[str, cq.Shape]:
    """Check volumes (C23) of lead k: the ribbon envelope sl_ribbon_w x sl_ribbon_t along its route as straight legs
    (union = the route with square corners):

    W  through the pod window, flat (width along u, thickness along Z), centred in the window, from the case end
       face (v = body_l/2) out to the outside leg;
    O  up outside the pod's far face (SERVO_PROXY_GAP off it, thickness tangential) from W to the crossing band top;
    X  crossing corridor: annular sector at Z [sl_cross_z0, sl_cross_z1] from the outside leg's corner angles to the
       column's slot edge, r from the slot proxy's inner face to the outside leg's outer corner;
    S  up the entry slot (centred, thickness radial) from sl_cross_z0 to sl_entry_z_open;
    C  straight column: parallelogram in (r, Z) from the slot centre at sl_entry_z_open to the deck-notch centre at
       the deck bottom (the straight line a pull wire follows);
    N  up the deck notch (centred) from the deck bottom to the top of the rebate band."""
    W, T, g = L.sl_ribbon_w, L.sl_ribbon_t, SERVO_PROXY_GAP
    if L.direct_drive:
        return _servo_lead_legs_direct(L, k)
    thp, thc, s = L.pod_thetas[k], L.sl_col_thetas[k], L.sl_side
    ua = L.servo_axis_r
    zw = 0.5 * (L.sl_win_z0 + L.sl_win_z1)
    v_case, v_face = L.p["servo"]["body_l"] / 2, L.pod_w / 2 + g

    def vr(a: float, b: float) -> tuple[float, float]:
        return (a, b) if s > 0 else (-b, -a)

    legs = {"W": _box_polar(ua - W / 2, ua + W / 2, *vr(v_case, v_face + T), zw - T / 2, zw + T / 2, thp),
            "O": _box_polar(ua - W / 2, ua + W / 2, *vr(v_face, v_face + T), zw - T / 2, L.sl_cross_z1, thp)}
    corners = [(u, v) for u in (ua - W / 2, ua + W / 2) for v in (s * v_face, s * (v_face + T))]
    r_out = max(math.hypot(u, v) for u, v in corners)
    th_o = [thp + math.degrees(math.atan2(v, u)) for u, v in corners]
    half_e = math.degrees(math.atan2(W / 2, L.sl_entry_u0))
    th_a = min(th_o) if s > 0 else thc - half_e
    th_b = thc + half_e if s > 0 else max(th_o)
    legs["X"] = _sector(L.sl_entry_u0 + 0.5 * (L.sl_entry_u1 - L.sl_entry_u0 - T), r_out, th_a, th_b,
                        L.sl_cross_z0, L.sl_cross_z1)
    ue = 0.5 * (L.sl_entry_u0 + L.sl_entry_u1)
    un = 0.5 * (L.sl_notch_u0 + L.spigot_od / 2)
    z_deck = L.z_prop_spigot_bottom - L.body_wall
    legs["S"] = _box_polar(ue - T / 2, ue + T / 2, -W / 2, W / 2, L.sl_cross_z0, L.sl_entry_z_open, thc)
    legs["C"] = _prism_uz([(ue - T / 2, L.sl_entry_z_open), (ue + T / 2, L.sl_entry_z_open),
                           (un + T / 2, z_deck), (un - T / 2, z_deck)], -W / 2, W / 2, thc)
    legs["N"] = _box_polar(un - T / 2, un + T / 2, -W / 2, W / 2, z_deck, servo_rebate_band(L)[1], thc)
    return legs


def _servo_lead_legs_direct(L: Layout, k: int) -> dict[str, cq.Shape]:
    """v0.9 direct drive (C23) ribbon proxies of lead k: K the inline connector in the fairing's connector bay (shrunk
    SERVO_PROXY_GAP), P the extension along the pocket top from the connector to the tunnel (flat ribbon, width along Z),
    T the tunnel run (a cylinder of the ribbon's diagonal + 2 x min_clearance... less SERVO_PROXY_GAP on the radius, on
    sl_pipe_points), S up the entry slot from the pipe's end to sl_entry_z_open (as v0.6), C the straight column and N
    up the deck notch (as v0.6)."""
    W, T, g = L.sl_ribbon_w, L.sl_ribbon_t, SERVO_PROXY_GAP
    thf, thc = L.fin_thetas[k], L.sl_col_thetas[k]
    zc = 0.5 * (L.dd_conn_z[0] + L.dd_conn_z[1])
    p0, pe = L.sl_pipe_points(k)
    legs = {"K": _box_polar(L.dd_conn_u[0] + g, L.dd_conn_u[1] - g, -L.dd_conn_v + g, L.dd_conn_v - g,
                            L.dd_conn_z[0] + g, L.dd_conn_z[1] - g, thf),
            "P": _box_polar(L.sl_p0_uz[0], L.dd_conn_u[0] + g, -T / 2, T / 2, min(zc, L.sl_p0_uz[1]) - W / 2,
                            max(zc, L.sl_p0_uz[1]) + W / 2, thf),
            "T": _cyl_between(math.hypot(W, T) + 2 * L.p["servo_leads"]["min_clearance"] - 2 * g, p0, pe)}
    ue = 0.5 * (L.sl_entry_u0 + L.sl_entry_u1)
    un = 0.5 * (L.sl_notch_u0 + L.spigot_od / 2)
    z_deck = L.z_prop_spigot_bottom - L.body_wall
    legs["S"] = _box_polar(ue - T / 2, ue + T / 2, -W / 2, W / 2, pe[2], L.sl_entry_z_open, thc)
    legs["C"] = _prism_uz([(ue - T / 2, L.sl_entry_z_open), (ue + T / 2, L.sl_entry_z_open),
                           (un + T / 2, z_deck), (un - T / 2, z_deck)], -W / 2, W / 2, thc)
    legs["N"] = _box_polar(un - T / 2, un + T / 2, -W / 2, W / 2, z_deck, servo_rebate_band(L)[1], thc)
    return legs


def servo_bundle_legs(L: Layout) -> dict[str, cq.Shape]:
    """Check volumes (C23) of the gathered leads: R- / R+ two-ribbon layers (2T thick, SERVO_PROXY_GAP off the rebate's
    inner wall, in the rebate band) from the farthest column on each side to the bundle at the gather theta; then the
    2 x 2 bundle (2W tangential x 2T radial): B steps out of the rebate (rebate band) to the channel, U goes up the
    channel (centred) past the lip, D steps in through the shelf notch (centred in [z_lip_pocket_top, sl_ch_z1], 2T
    tall), E goes up along the strut through the bulkhead servo hole (centred) to OVERLAP above the bulkhead."""
    W, T, tg = L.sl_ribbon_w, L.sl_ribbon_t, L.sl_gather_theta
    zr0, zr1 = servo_rebate_band(L)
    r0 = L.sl_rebate_r0 + SERVO_PROXY_GAP
    half = math.degrees(math.atan2(W, L.sl_rebate_r0))          # the bundle's half width at the gather theta
    offs = [((t - tg + 180.0) % 360.0) - 180.0 for t in L.sl_col_thetas]
    legs = {}
    if min(offs) < -half:
        legs["R-"] = _sector(r0, r0 + 2 * T, tg + min(offs), tg - half, zr0, zr1)
    if max(offs) > half:
        legs["R+"] = _sector(r0, r0 + 2 * T, tg + half, tg + max(offs), zr0, zr1)
    cch = 0.5 * (L.sl_ch_r0 + L.sl_ch_r1)
    chole = 0.5 * (L.sl_hole_r0 + L.sl_hole_r1)
    zin = 0.5 * (L.z_lip_pocket_top + L.sl_ch_z1)
    legs["B"] = _box_polar(r0, cch + T, -W, W, zr0, zr1, tg)
    legs["U"] = _box_polar(cch - T, cch + T, -W, W, zr0, zin + T, tg)
    legs["D"] = _box_polar(chole - T, cch + T, -W, W, zin - T, zin + T, tg)
    legs["E"] = _box_polar(chole - T, chole + T, -W, W, zin - T, L.z_intake_bulkhead_top + OVERLAP, tg)
    return legs


def lead_riser_proxies(L: Layout, z_top: float) -> dict[str, cq.Shape]:
    """v0.6.3 check helper (C25): the legs that rise through the intake bulkhead - C20's leg E (motor-lead bundle,
    tangential +-WIRE_BUNDLE_W/2, r c_hole +- WIRE_BUNDLE_T/2 at wire_theta) and C23's bundle leg E (2 x 2 servo ribbons,
    tangential +-sl_ribbon_w, r chole +- sl_ribbon_t at sl_gather_theta) - with their footprints unchanged but run from
    inside the hole (z_intake_bulkhead_bottom) straight on up to z_top: the leads keep rising into the bay beside the
    FCU board.  Empty for a preset without those routes."""
    out: dict[str, cq.Shape] = {}
    z0 = L.z_intake_bulkhead_bottom
    if L.tabs:
        c = (L.wire_hole_r0 + L.wire_hole_r1) / 2
        out["motor-lead bundle (C20 leg E)"] = _box_polar(c - WIRE_BUNDLE_T / 2, c + WIRE_BUNDLE_T / 2, -WIRE_BUNDLE_W / 2,
                                                          WIRE_BUNDLE_W / 2, z0, z_top, L.wire_theta)
    if L.servo_leads:
        c, W, T = (L.sl_hole_r0 + L.sl_hole_r1) / 2, L.sl_ribbon_w, L.sl_ribbon_t
        out["servo-lead bundle (C23 leg E)"] = _box_polar(c - T, c + T, -W, W, z0, z_top, L.sl_gather_theta)
    return out


def bulkhead_hole_columns(L: Layout, z0: float, z1: float, inset: float) -> dict[str, cq.Shape]:
    """v0.6.3 check helper (C25): the cross-section of every intake-bulkhead hole, inset by ``inset`` from its walls, as
    a vertical column Z [z0, z1] - the motor-lead hole (tabs), the servo-lead hole (servo leads), each vent hole and the
    centre cable hole."""
    out: dict[str, cq.Shape] = {}
    if L.tabs:
        hw = getattr(L, "wire_hole_w", L.wire_ch_w) / 2          # v0.9 fix round: the hole's own width
        out["motor-lead hole"] = _box_polar(L.wire_hole_r0 + inset, L.wire_hole_r1 - inset, -hw + inset,
                                            hw - inset, z0, z1, L.wire_theta)
    if L.servo_leads:
        out["servo-lead hole"] = _box_polar(L.sl_hole_r0 + inset, L.sl_hole_r1 - inset, -L.sl_ch_w / 2 + inset,
                                            L.sl_ch_w / 2 - inset, z0, z1, L.sl_gather_theta)
    for th in L.vent_hole_thetas:
        c = _polar(L.vent_hole_r, 0.0, th, 0.0)
        out[f"vent hole {th:g} deg"] = _cyl_z(L.vent_hole_d / 2 - inset, z0, z1, c.x, c.y)
    out["cable hole"] = _cyl_z(L.cable_hole_d / 2 - inset, z0, z1)
    return out


def radial_material(shape: cq.Shape, theta: float, z: float, r0: float, r1: float,
                    w: float = 0.1, h: float = 0.1) -> list[tuple[float, float]]:
    """Radial probe (check helper, C21 and tests): the (r_min, r_max) of every solid of ``shape`` inside a
    thin box along the true radial direction theta (width w, Z [z, z + h], r [r0, r1]), sorted by r.  The
    vertices of the common solids lie on the probe's side faces, whose radial sag is w^2 / (8 r) ~ 2e-5 mm."""
    common = shape.intersect(_box_polar(r0, r1, -w / 2, w / 2, z, z + h, theta))
    out = []
    for s in common.Solids():
        rs = [math.hypot(p.X, p.Y) for p in s.Vertices()]
        out.append((min(rs), max(rs)))
    return sorted(out)


# --------------------------------------------------------------------------- #
# SPEC 5.5 - launch pad
# --------------------------------------------------------------------------- #
def launch_pad(L: Layout) -> cq.Shape:
    lp = L.p["launch_pad"]
    parts = [_tube(L.pad_plate_od / 2, L.pad_plate_id / 2, -L.pad_plate_t, 0.0)]
    s, f = lp["leg_size"], lp["foot_size"]
    z_leg_top, z_leg_bot = -L.pad_plate_t, -L.pad_plate_t - L.pad_leg_h
    for th in L.pad_leg_thetas:
        parts.append(_box_polar(lp["leg_r"] - s / 2, lp["leg_r"] + s / 2, -s / 2, s / 2, z_leg_bot, z_leg_top, th))
        parts.append(_box_polar(lp["leg_r"] - f / 2, lp["leg_r"] + f / 2, -f / 2, f / 2,
                                z_leg_bot - lp["foot_t"], z_leg_bot, th))
    rw, rh, rl, r_in, gap = (lp["fin_locator_rib_w"], lp["fin_locator_rib_h"], lp["fin_locator_rib_len"],
                             lp["fin_locator_rib_r_in"], lp["fin_locator_gap"])
    for th in L.fin_thetas:
        for sign in (+1.0, -1.0):
            v0 = sign * gap / 2
            v1 = sign * (gap / 2 + rw)
            parts.append(_box_polar(r_in, r_in + rl, min(v0, v1), max(v0, v1), 0.0, rh, th))
    return _fuse(parts)


def pad_foot_bottom_z(L: Layout) -> float:
    return -L.pad_plate_t - L.pad_leg_h - L.p["launch_pad"]["foot_t"]


# --------------------------------------------------------------------------- #
# joints (for lumped masses and check C16)
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class Joint:
    lower: str
    upper: str
    z_spigot_bottom: float
    spigot_inner_r: float
    screwed: bool
    hole_bottom_r: float | None = None   # v0.5d: flat bottom of a blind spigot hole (propulsion); None = through

    @property
    def z_screw(self) -> float:
        return self.z_spigot_bottom


def joints(L: Layout) -> list[Joint]:
    """Module joints bottom -> top; z_spigot_bottom + joint_screw_dz is the screw station.  v0.9 CAD-2 VTOL: the top cap
    is screwed to the avionics spigot (the stack's last joint)."""
    out = [
        Joint("propulsion_module", "intake_module", L.z_prop_spigot_bottom, L.edf_bore_r, True, prop_joint_hole_r0(L)),
        Joint("intake_module", "avionics_module", L.z_intake_spigot_bottom, L.spigot_id / 2, True),
    ]
    if getattr(L, "vtol", False):
        return out + [Joint("avionics_module", "top_cap", L.z_avionics_spigot_bottom, L.spigot_id / 2, True)]
    return out + [
        Joint("avionics_module", "payload_module", L.z_avionics_spigot_bottom, L.spigot_id / 2, True),
        Joint("payload_module", "nose_cone", L.z_payload_spigot_bottom, L.spigot_id / 2, False),
    ]


def joint_probe(L: Layout, j: Joint, theta: float, probe_d: float = 2.0) -> cq.Shape:
    """Thin radial probe along a joint screw-hole axis (C16): from inside the spigot bore - or, for a blind hole,
    JOINT_PROBE_INSET past its flat bottom - out past the outer tube."""
    r0 = j.spigot_inner_r - OVERLAP if j.hole_bottom_r is None else j.hole_bottom_r + JOINT_PROBE_INSET
    return _radial_cyl(probe_d, r0, L.body_r + 2 * OVERLAP, theta, j.z_spigot_bottom + L.joint_screw_dz)


def joint_hole_floor_probe(L: Layout, j: Joint, theta: float, probe_d: float = 2.0) -> cq.Shape | None:
    """C16 positive evidence that a blind hole is blind: a radial probe from the spigot bore to JOINT_PROBE_INSET
    short of the hole bottom (it must be solid); None for a through hole."""
    if j.hole_bottom_r is None:
        return None
    return _radial_cyl(probe_d, j.spigot_inner_r, j.hole_bottom_r - JOINT_PROBE_INSET, theta,
                       j.z_spigot_bottom + L.joint_screw_dz)


# --------------------------------------------------------------------------- #
# part registry
# --------------------------------------------------------------------------- #
# printed parts: Bambu Lab PLA Matte Ivory White #FFFFFF (params.json printer.color / color_hex; user 2026-11-04, was
# PLA Matte Black 2026-10-20) - STEP display colour; the YUCHAN inlay (livery.colours.logo, Mandarin Orange #F99963) and
# the launch pad (printer.color_exceptions: stays Black / Charcoal #000000, not part of the 2026-11-04 request)
PRINTED_COLOR = (1.0, 1.0, 1.0)
LIVERY_COLOR = (0xF9 / 255.0, 0x99 / 255.0, 0x63 / 255.0)
LAUNCH_PAD_COLOR = (0.08, 0.08, 0.085)    # unchanged (the 2026-10-20 black)
COLORS = {
    "edf_unit": (0.25, 0.25, 0.28), "battery": (0.95, 0.60, 0.10), "esc": (0.85, 0.20, 0.20),
    "nav": (0.20, 0.65, 0.25), "misc_electronics": (0.60, 0.40, 0.80), "servo": (0.15, 0.35, 0.90),
    "servo_arm": (0.35, 0.55, 0.95), "pushrod": (0.95, 0.90, 0.10), "cansat_payload": (0.90, 0.30, 0.60),
    "spring": (0.65, 0.65, 0.15), "chute_pack": (0.95, 0.50, 0.05), "com": (0.10, 0.75, 0.80),
    "radio_log": (0.55, 0.85, 0.20), "fcu": (0.05, 0.45, 0.35),
    "gps": (0.10, 0.30, 0.75), "gps_ground_plane": (0.80, 0.50, 0.20),   # v0.8: blue GPS board (GY-NEO6MV2 then, NEO-7M now), copper plate
    "gns": (0.15, 0.55, 0.55),     # v0.8.1: the GNS board (ATmega128A #4)
    "carrier": (0.10, 0.50, 0.30),     # v0.9 CAD-2 VTOL: the one avionics carrier PCB (green)
    "buffer_pack": (0.95, 0.80, 0.15),  # v0.9 CAD-2 VTOL hybrid: the 6S 650 mAh buffer
    "logic_pack": (0.95, 0.75, 0.30), "logic_switch": (0.20, 0.20, 0.22),   # 2026-11-05 logic power (battery build)
}


@dataclass
class Part:
    """One report component (SPEC 1 / 6). ``base`` is the printed part name (STL file name)."""

    name: str
    kind: str                      # printed | placeholder | lumped
    base: str
    shape: cq.Shape | None
    module: str                    # owning module / zone (C14)
    orientation: str = ""          # upright | flat | plate_down (printed only)
    color: tuple[float, float, float] = PRINTED_COLOR
    mass_g: float | None = None    # placeholders / lumped (fixed mass)
    density_g_cm3: float | None = None  # printed
    point_mm: tuple[float, float, float] | None = None  # lumped
    in_vehicle: bool = True        # launch_pad is ground equipment
    # 2026-11-05 (logic_power.in_flight_mass_model false): the solid the report's mass entry is measured on when it is not
    # ``shape`` (the top cap without the logic switch hole); ``shape`` stays the real part (STL / STEP / checks)
    mass_shape: cq.Shape | None = None


PRINT_ORIENTATION = {
    "propulsion_module": "upright", "intake_module": "upright", "avionics_module": "upright",
    "payload_module": "upright", "nose_cone": "upright", "avionics_tray": "flat", "piston": "flat",
    "vane": "flat", "vane_horn": "flat", "latch_arm": "flat", "launch_pad": "plate_down",
    "gps_pod_lid": "flat",          # v0.8: top face on the bed, posts up
    "avionics_logo": "upright",     # livery 2026-11-04: printed WITH the avionics module (2-filament project)
    "vane_coupler": "upright", "servo_cover": "flat", "wire_cover": "flat",     # v0.9
    "top_cap": "flat",              # v0.9 CAD-2 VTOL: top face on the bed, skirt / ribs / hub up
}

# v0.9: rotation about Z on the print plate (deg).  With the fins on the axes (direct drive) the propulsion module spans
# footprint_d (246.8) along X and Y; the P2S plate refuses that axis-aligned (its wrap-detection zone), turned 45 deg the
# fins lie on the diagonals (176.2 x 176.2) - make_print_files / make_v09_print_files rotate it the same way
def plate_rot_z(L: Layout, base: str) -> float:
    """Rotation about Z on the print plate (deg) that puts the propulsion module's fins on the plate diagonals."""
    return ((45.0 - L.fin_thetas[0]) % 90.0) if base == "propulsion_module" else 0.0

PRINTED_BUILDERS: dict[str, Callable[[Layout], cq.Shape]] = {
    "propulsion_module": propulsion_module, "intake_module": intake_module,
    "avionics_module": avionics_module, "avionics_tray": avionics_tray,
    "payload_module": payload_module, "piston": piston, "nose_cone": nose_cone,
    "vane": vane, "vane_horn": vane_horn, "latch_arm": latch_arm, "launch_pad": launch_pad,
    "gps_pod_lid": gps_pod_lid,     # v0.8
    "vane_coupler": vane_coupler, "servo_cover": servo_cover, "wire_cover": wire_cover,   # v0.9
    "top_cap": top_cap,             # v0.9 CAD-2 VTOL
    "avionics_logo": avionics_logo,  # livery 2026-11-04
}

PRINTED_MODULE = {
    "propulsion_module": "propulsion_module", "intake_module": "intake_module",
    "avionics_module": "avionics_module", "avionics_tray": "avionics_module",
    "payload_module": "payload_module", "piston": "payload_module", "nose_cone": "nose_cone",
    "vane": "propulsion_module", "vane_horn": "propulsion_module", "latch_arm": "payload_module",
    "launch_pad": "ground", "gps_pod_lid": "payload_module",
    "vane_coupler": "propulsion_module", "servo_cover": "propulsion_module", "wire_cover": "intake_module",   # v0.9
    "top_cap": "top_cap",           # v0.9 CAD-2 VTOL
    "avionics_logo": "avionics_module",     # livery 2026-11-04: the orange inlay of the avionics module
}


def lumped_masses(L: Layout) -> list[Part]:
    hw = L.p["hardware_mass_g"]
    z_joints = [j.z_spigot_bottom + L.joint_screw_dz for j in joints(L)]
    z_mean = sum(z_joints) / len(z_joints)
    if L.direct_drive:       # v0.9: cover / coupler / horn screws in the fairings (vane_drive.hardware_g_ASSUMED)
        z_hw = 0.5 * (L.fair_z[0] + L.fair_z[1])
        linkage = [Part("vane_drive_hardware", "lumped", "vane_drive_hardware", None, "propulsion_module",
                        mass_g=L.p["vane_drive"]["hardware_g_ASSUMED"], point_mm=(0.0, 0.0, z_hw))]
    else:
        linkage = [Part("pushrods_horns_clevises", "lumped", "pushrods_horns_clevises", None, "propulsion_module",
                        mass_g=hw["pushrods_horns_clevises"], point_mm=(0.0, 0.0, L.vane_hinge_z))]
    return [
        Part("joint_screws_inserts", "lumped", "joint_screws_inserts", None, "body",
             mass_g=hw["joint_screws_inserts"], point_mm=(0.0, 0.0, z_mean)),
    ] + linkage + ([Part("servo_leads", "lumped", "servo_leads", None, "body", mass_g=L.sl_lead_mass_g,    # SPEC 3.8 / 6 v0.6
                         point_mm=tuple(L.sl_lead_com))] if L.servo_leads else []) + (
        # v0.9: the motor leads (3 x 12 AWG + bullet pairs) along Layout.motor_lead_polyline
        [Part("motor_leads", "lumped", "motor_leads", None, "body", mass_g=L.ml_mass_g, point_mm=tuple(L.ml_com))]
        if getattr(L, "has_motor_leads", False) else []) + (
        # v0.8: GPS lead (wire along Layout.gps_lead_polyline + slack), connector pair, screws / standoffs / tape
        [Part("gps_lead", "lumped", "gps_lead", None, "payload_module", mass_g=L.gps_lead_mass_g,
              point_mm=tuple(L.gps_lead_com))] if getattr(L, "has_gps_pod", False) else []) + (
        # v0.9 CAD-2 VTOL: the GPS lead from the cap top to the carrier, and (power.source tether / hybrid) the tether kit
        [Part("gps_lead", "lumped", "gps_lead", None, "top_cap", mass_g=L.gps_lead_mass_g, point_mm=tuple(L.gps_lead_com))]
        + [Part(nm, "lumped", nm, None, "top_cap" if nm in ("tether_hardpoint", "tether_boot") else "avionics_module", mass_g=m,
                point_mm=tuple(pt)) for nm, (m, pt) in L.tether_items.items()]
        if getattr(L, "vtol", False) else [])


def build_all(L: Layout) -> dict[str, Part]:
    """Every component of SPEC sections 3-6 in report order (printed, placeholders, lumped)."""
    rho = L.p["printer"]["density_g_cm3"]
    e, el, rec, hw = L.p["edf"], L.p["electronics"], L.p["recovery"], L.p["hardware_mass_g"]
    parts: list[Part] = []

    def printed(name: str, base: str, shape: cq.Shape, in_vehicle: bool = True) -> None:
        color = LIVERY_COLOR if base == "avionics_logo" else LAUNCH_PAD_COLOR if base == "launch_pad" else PRINTED_COLOR
        parts.append(Part(name, "printed", base, shape, PRINTED_MODULE[base], PRINT_ORIENTATION[base],
                          color, density_g_cm3=rho, in_vehicle=in_vehicle))

    def placeholder(name: str, shape: cq.Shape, module: str, mass_g: float, color_key: str) -> None:
        parts.append(Part(name, "placeholder", name, shape, module, "", COLORS[color_key], mass_g=mass_g))

    vtol = getattr(L, "vtol", False)
    if vtol:     # v0.9 CAD-2: the VTOL configuration (Layout._top_cap / carrier / power source)
        return _build_all_vtol(L, parts, printed, placeholder)
    for base in ("propulsion_module", "intake_module", "avionics_module", "avionics_tray",
                 "payload_module", "piston", "nose_cone"):
        printed(base, base, PRINTED_BUILDERS[base](L))
        if base == "avionics_module" and getattr(L, "has_livery", False):   # livery: the orange inlay, own part
            printed("avionics_logo", "avionics_logo", avionics_logo(L))
    for i in range(L.vane_count):
        printed(f"vane_{i + 1}", "vane", vane(L, i))
    if L.direct_drive:     # v0.9: couplers and servo covers instead of horns
        for i in range(L.vane_count):
            printed(f"vane_coupler_{i + 1}", "vane_coupler", vane_coupler(L, i))
        for i in range(L.vane_count):
            printed(f"servo_cover_{i + 1}", "servo_cover", servo_cover(L, i))
    else:
        for i in range(L.vane_count):
            printed(f"vane_horn_{i + 1}", "vane_horn", vane_horn(L, i))
    if L.tabs and getattr(L, "wire_ch_open", False):   # v0.9: the motor-lead groove cover
        printed("wire_cover", "wire_cover", wire_cover(L))
    printed("latch_arm", "latch_arm", latch_arm(L))
    if getattr(L, "has_gps_pod", False):   # v0.8: the GPS pod lid (the pod itself is part of payload_module)
        printed("gps_pod_lid", "gps_pod_lid", gps_pod_lid(L))
    printed("launch_pad", "launch_pad", launch_pad(L), in_vehicle=False)

    placeholder("edf_unit", edf_unit(L), "propulsion_module", e["mass_g"], "edf_unit")
    placeholder("battery", battery(L), "avionics_module", L.p["battery"]["mass_g"], "battery")
    placeholder("esc", esc(L), "avionics_module", el["esc"]["mass_g"], "esc")
    if L.has_nav:          # rocket-carrier round (2026-11-14): no NAV board with the carrier
        placeholder("nav", nav(L), "avionics_module", el["nav"]["mass_g"], "nav")
    placeholder("misc_electronics", misc_electronics(L), "avionics_module", el["misc"]["mass_g"], "misc_electronics")
    if L.has_com_board:   # v0.6: hangs under the payload bulkhead, inside the avionics spigot bore
        placeholder("com", com_board(L), "avionics_module", el["com"]["mass_g"], "com")
    if L.has_radio_log:   # v0.6.2: LoRa / OpenLog / level shifter / LDO box beside the COM board
        placeholder("radio_log", radio_log_box(L), "avionics_module", el["radio_log"]["mass_g"], "radio_log")
    if L.has_fcu:         # v0.6.3: FCU board flat on the intake bulkhead top (inside the intake spigot, below the tray)
        # rocket-carrier round (2026-11-14): the carrier PCB rev 3.1 takes the spot (Layout.fcu_part 'carrier')
        placeholder(L.fcu_part, fcu_board(L), "avionics_module", L.fcu_mass_g,
                    "carrier" if L.fcu_part == "carrier" else "fcu")
    if getattr(L, "has_gns", False):   # v0.8.1: the GNS board on its bosses on the -x tube wall (board + 4 screws)
        placeholder("gns", gns_board(L), "avionics_module", L.gns_mass_g, "gns")
    if getattr(L, "has_gps_pod", False):   # v0.8: GPS board + patch and the ground plane in the pod
        placeholder("gps", gps_module(L), "payload_module", L.gps_board_mass_g, "gps")
        placeholder("gps_ground_plane", gps_ground_plane(L), "payload_module", L.gps_gp_mass_g, "gps_ground_plane")
    for i in range(L.vane_count):
        placeholder(f"servo_vane_{i + 1}", servo_direct(L, i) if L.direct_drive else servo_vane(L, i), "propulsion_module",
                    L.p["servo"]["mass_g"], "servo")
    placeholder("servo_release", servo_release(L), "payload_module", L.p["servo"]["mass_g"], "servo")
    if L.direct_drive:     # v0.9: the stock horn (fix round: double-arm, cut) on each output spline
        for i in range(L.vane_count):
            placeholder(f"servo_horn_{i + 1}", servo_horn(L, i), "propulsion_module", L.p["vane_drive"]["horn"]["mass_g"],
                        "servo_arm")
        if getattr(L, "dd_bushing", False):      # v0.9 fix round: brass hinge bushings
            for i in range(L.vane_count):
                placeholder(f"hinge_bushing_{i + 1}", hinge_bushing(L, i), "propulsion_module", L.dd_bush_mass_g,
                            "servo_arm")
    else:
        for i in range(L.vane_count):
            placeholder(f"servo_arm_{i + 1}", servo_arm(L, i), "propulsion_module", VANE_PLACEHOLDER_MASS_G, "servo_arm")
        for i in range(L.vane_count):
            placeholder(f"pushrod_{i + 1}", pushrod(L, i), "propulsion_module", VANE_PLACEHOLDER_MASS_G, "pushrod")
    placeholder("cansat_payload", cansat_payload(L), "payload_module", L.p["payload"]["mass_g"], "cansat_payload")
    placeholder("spring", spring(L), "payload_module", SPRING_MASS_FRACTION * hw["spring_piston_hardware"], "spring")
    placeholder("chute_pack", chute_pack(L), "nose_cone",
                rec["chute_mass_g"] + rec["lines_swivel_mass_g"] + rec["shock_cord_mass_g"], "chute_pack")
    parts += lumped_masses(L)
    if getattr(L, "has_logic_power", False) and getattr(L, "logic_in_flight", False):
        # rocket-carrier round (2026-11-14): the 1S logic pack, switch, switch lead and retention fly (in_flight_mass_model)
        parts += list(logic_power_parts(L).values())
    return {p.name: p for p in parts}


def _build_all_vtol(L: Layout, parts: list, printed, placeholder) -> dict[str, Part]:
    """v0.9 CAD-2 VTOL part list (report order): printed propulsion / intake / avionics / tray / top cap, vanes, couplers,
    servo covers, wire cover; placeholders EDF, battery (power.source battery) or the hybrid buffer pack, ESC, misc, the
    carrier PCB, GPS + ground plane on the cap, the four vane servos + horns; lumped joint screws, vane-drive hardware,
    servo / motor / GPS leads and the tether kit (tether / hybrid).  No CanSat, chute, payload module, piston, spring,
    release servo, latch, nose, NAV / COM / GNS / radio box, launch pad."""
    e, el = L.p["edf"], L.p["electronics"]
    for base in ("propulsion_module", "intake_module", "avionics_module", "avionics_tray", "top_cap"):
        printed(base, base, PRINTED_BUILDERS[base](L))
        if base == "avionics_module" and getattr(L, "has_livery", False):   # livery: the orange inlay, own part
            printed("avionics_logo", "avionics_logo", avionics_logo(L))
    if getattr(L, "has_logic_power", False) and not getattr(L, "logic_in_flight", False):
        # 2026-11-05 user decision 4 (mass deferred, logic_power.in_flight_mass_model false): the printed cap HAS the switch
        # hole (STL / STEP / checks); its mass entry is measured on the cap without it, so the flight mass model is unchanged
        # (switch-collar round 2026-11-05: without the collar too - with_logic_hole=False leaves both out)
        parts[-1].mass_shape = top_cap(L, with_logic_hole=False)
    for i in range(L.vane_count):
        printed(f"vane_{i + 1}", "vane", vane(L, i))
    for i in range(L.vane_count):
        printed(f"vane_coupler_{i + 1}", "vane_coupler", vane_coupler(L, i))
    for i in range(L.vane_count):
        printed(f"servo_cover_{i + 1}", "servo_cover", servo_cover(L, i))
    printed("wire_cover", "wire_cover", wire_cover(L))
    placeholder("edf_unit", edf_unit(L), "propulsion_module", e["mass_g"], "edf_unit")
    if L.has_battery and not getattr(L, "buffer_z", None):
        placeholder("battery", battery(L), "avionics_module", L.p["battery"]["mass_g"], "battery")
    if getattr(L, "buffer_z", None):
        placeholder("buffer_pack", buffer_pack(L), "avionics_module",
                    L.p["power"]["hybrid_extra_ASSUMED"]["buffer_pack"]["mass_g"], "buffer_pack")
    placeholder("esc", esc(L), "avionics_module", el["esc"]["mass_g"], "esc")
    placeholder("misc_electronics", misc_electronics(L), "avionics_module", el["misc"]["mass_g"], "misc_electronics")
    placeholder(L.fcu_part, fcu_board(L), "avionics_module", L.fcu_mass_g, "carrier")
    placeholder("gps", cap_gps_module(L), "top_cap", L.gps_board_mass_g, "gps")
    placeholder("gps_ground_plane", cap_gps_ground_plane(L), "top_cap", L.gps_gp_mass_g, "gps_ground_plane")
    for i in range(L.vane_count):
        placeholder(f"servo_vane_{i + 1}", servo_direct(L, i), "propulsion_module", L.p["servo"]["mass_g"], "servo")
    for i in range(L.vane_count):
        placeholder(f"servo_horn_{i + 1}", servo_horn(L, i), "propulsion_module", L.p["vane_drive"]["horn"]["mass_g"],
                    "servo_arm")
    if getattr(L, "dd_bushing", False):      # v0.9 fix round: brass hinge bushings
        for i in range(L.vane_count):
            placeholder(f"hinge_bushing_{i + 1}", hinge_bushing(L, i), "propulsion_module", L.dd_bush_mass_g, "servo_arm")
    # v0.9 fix round: the tether kit's boxes and the hardpoint stack (power.source tether / hybrid)
    for nm, (_x, _y, _z, m) in getattr(L, "tether_boxes", {}).items():
        placeholder(nm, tether_box(L, nm), "avionics_module", m, "misc_electronics")
    if L.power_source in ("tether", "hybrid") and getattr(L, "cap_hardpoint", False):
        placeholder("tether_eyebolt", tether_eyebolt(L), "top_cap", L.cap_bolt_mass_g, "servo_arm")
        placeholder("tether_load_washer", tether_load_washer(L), "top_cap", L.cap_washer_mass_g, "servo_arm")
        placeholder("tether_nut", tether_nut(L), "top_cap", L.cap_nut_mass_g, "servo_arm")
    parts += lumped_masses(L)
    if getattr(L, "has_logic_power", False) and getattr(L, "logic_in_flight", False):
        parts += list(logic_power_parts(L).values())     # only once the user decides (logic_power.in_flight_mass_model)
    return {p.name: p for p in parts}


POST_EJECTION_EXCLUDED = ("cansat_payload", "nose_cone", "chute_pack")   # SPEC 6


def layout_for(preset: str | None = None, config: str | None = None, power: str | None = None) -> Layout:
    """Layout of a preset; v0.9 CAD-2: of a hardware configuration (mission.config) and VTOL power source."""
    if config is None and power is None:
        return build_layout(preset)
    return build_layout(preset, config=config, power=power)


__all__ = [
    "Layout", "Part", "Joint", "build_all", "layout_for", "joints", "joint_probe", "intersection_volume",
    "propulsion_module", "intake_module", "avionics_module", "avionics_tray", "payload_module", "piston",
    "nose_cone", "vane", "vane_shaft", "vane_horn", "latch_arm", "launch_pad", "edf_unit", "battery", "esc",
    "nav", "misc_electronics", "servo_vane", "servo_release", "servo_arm", "pushrod", "pushrod_endpoints",
    "pushrod_length", "cansat_payload", "spring", "chute_pack", "servo_pod", "release_pod", "fin",
    "latch_lug", "payload_envelope", "exhaust_path", "nose_outer_solid", "nose_inner_solid",
    "nose_cavity_volume_mm3", "intake_window_area_mm2", "intake_band", "intake_window_cutters",
    "intake_opening_z_range", "intake_opening_probe_radii", "intake_opening_areas_mm2", "latch_axis_xy", "latch_lug_centre_xy",
    "latch_arm_tip_xy", "horn_d_bore_flat_distance", "cansat_chute_block_top", "pod_z_floor",
    "pad_foot_bottom_z", "POST_EJECTION_EXCLUDED", "PRINT_ORIENTATION", "PRINTED_BUILDERS",
    # v0.5 tab mount
    "outer_skin", "skin_filled", "liner", "tab_boss_thetas", "tab_screw_lugs", "tab_keyway", "tab_screw_cutters",
    "wire_slot_boxes", "wire_slot", "wire_low_cut", "motor_lead_lower_legs", "motor_lead_dropin_sweep", "intake_wire_bulge", "intake_wire_channel", "intake_wire_notch",
    "intake_bulkhead_wire_hole", "edf_tab", "edf_motor", "tab_sweep", "tab_screw_probe", "wire_bundle_steps",
    "wire_bundle_legs", "wire_bundle_proxy", "radial_material",
    # v0.5d
    "intake_wire_floor_wedge", "tray_stop_blocks", "prop_joint_hole_r0", "prop_joint_hole_cutter",
    "joint_hole_floor_probe",
    # v0.6
    "com_board", "com_bosses", "com_hole_cutters",
    # v0.6 servo leads
    "servo_lead_window", "servo_lead_entry_slot", "servo_lead_deck_notch", "servo_lead_rebate", "servo_lead_cutters",
    "intake_servo_bulge", "intake_servo_channel", "intake_servo_notch", "intake_servo_floor_wedge",
    "intake_bulkhead_servo_hole", "servo_lead_legs", "servo_rebate_band", "servo_bundle_legs",
    # v0.6.2
    "intake_window_sectors", "intake_window_gable_prism", "intake_window_gussets", "radio_log_box",
    "joint_screw_axis_proxies",
    # v0.6.3
    "fcu_board", "lead_riser_proxies", "bulkhead_hole_columns",
    # v0.8
    "gps_pod_outer", "gps_cavity_cutter", "gps_bosses", "gps_underside_cutter", "gps_corner_pts", "gps_corner_cutters",
    "gps_tube_restore", "gps_bore_cutter", "gps_boss_hole_cutters", "gps_raceway_solids", "gps_raceway_cutters",
    "gps_pod_features", "gps_entry_cutter", "gps_pod_lid", "gps_module", "gps_ground_plane",
    # v0.8.1
    "gns_boss_prisms", "gns_boss_clip", "gns_bosses", "gns_hole_cutters", "gns_board",
    # livery 2026-11-04
    "livery_inlay_solids", "avionics_logo", "LIVERY_COLOR", "PRINTED_COLOR",
    # logic power 2026-11-05
    "logic_pack", "logic_switch", "logic_switch_below", "logic_switch_above", "logic_lead_keepouts", "carrier_plug_boxes",
    "logic_power_parts",
]
