"""Single source of truth for the vehicle layout.

Loads ``params.json`` (optionally applying an ``alt_presets`` entry) and derives
every Z station and radius that the Fusion 360 script, the CadQuery twin, the
sizing tool and the simulation share.  Pure Python 3 (no third-party imports)
so it also runs inside Fusion 360's embedded interpreter.

Invariant: geometry code never hard-codes a dimension that is derivable from
``params.json``.  If two CAD implementations disagree, the bug is in one of
them, not in this module - fix it here only when the spec changes.

Coordinate system (see docs/SPEC.md): +Z up along the thrust axis, Z = 0 at the
exhaust duct exit plane, +X toward vane 1 (theta = 0 deg).  Lengths in mm.

Run ``python cad/layout.py [preset]`` to print the station table, the v0.6.1 intake-band report, the v0.6.2 window-gable
report, the v0.6 COM-board, the v0.6.2 radio/log-box and the v0.6.3 FCU-board clearance reports.

v0.8.1 (architect 2026-10-21: a fourth ATmega128A, the GNS board - GPS + position hold; ``electronics.gns``,
``avionics_module.gns_mount``): ``_gns`` places the board (``gns_x/y/z``, ``gns_center``, ``gns_mass_g`` = board ASSUMED + screws)
on the -x avionics tube wall outboard of the battery, its top just under the GPS entry hole, on four diamond bosses with
blind tap holes (``gns_hole_vz``, ``gns_diamond_vz``, ``gns_boss_boxes``; walls ``gns_walls``) and raises on every shortfall
(``gns_clearance``); ``avionics_bay_boxes`` carries the board and its bosses, so the FCU / COM / radio-box / lead tables
include them.  The GPS lead now ends at the GNS header (``gps_lead_polyline``), and ``_gns_leads`` routes the glink lead
GNS -> NAV over the tray top (``glink_lead_polyline``, ``glink_mass_g`` reported, inside the board's ASSUMED mass) and
reports the board's distances (``gns_distance_mm``: ESC, GPS antenna, LoRa antenna, ...).  ``gns_report`` prints it all.

v0.6.3 (user 2026-10-15: a third ATmega128A; params flight_computer.architecture): ``electronics.fc`` is now
``electronics.nav`` - the NAV board (sensors + attitude estimate) at the v0.5 flight-computer spot on the tray, box
``nav_x/y/z`` (``nav_center_z``; the tray holes keep the params key names ``fc_hole_pattern`` / ``fc_hole_d``, printed
geometry unchanged) - and the new ``electronics.fcu`` board lies flat on the intake bulkhead's top face on foam tape
(``fcu_x/y/z``, ``fcu_center``; clearances ``fcu_clearance``, vent holes / cable-tie runs ``fcu_vent_reachable`` /
``fcu_tie_pairs``, raised on when short: ``_fcu``).  The FCU drives the servos, so the vane-servo leads' three estimated
bay legs now end on it (``servo_lead_polyline``), which shortens the leads and moves their slack / connectors down.
No printed part changes.

v0.6.2 (user 2026-10-15, both presets): (1) ``intake_module.window_bottom_end`` - the LOWER end of every window (the
ceiling when the intake prints flipped) is a truncated gable, flat over ``bridge_w`` at z_window_bottom and rising at
``slope_deg`` to the struts in the window's own frame (``_window_gable``: profile ``window_gable_profile_yz``, gable heights,
the lower edge's slope on the curved band surfaces ``gable_edge_slope_min_deg`` >= GABLE_MIN_EDGE_SLOPE_DEG, gusset volume /
centroid); the inlet area is the minimum over the band of ``window_open_area_mm2`` (gussets included).  (2)
``electronics.radio_log`` - the LoRa / OpenLog / level-shifter / LDO box moves from the tray's misc box to the payload-bulkhead
underside beside the COM board (``radio_log_x/y/z``; clearances ``radio_log_clearance``, raised on when short).

v0.6.1 intake band (SPEC 5.1, user 2026-10-15: the 1.2 mm struts broke when the print supports came off): the windowed
band Z [z_shelf_top, z_intake_bulkhead_bottom] is ``intake_module.window_wall_t`` thick, thickened inward (``r_band_in``
= body_r - window_wall_t, outer surface = body OD); the four windows are radial sectors (``window_sectors``, angle
``window_angle_deg`` = 360/strut_count - strut_w/body_r) so every strut keeps strut_w at body_r with radial side faces
(inner face ``strut_w_in``).  The inlet area that counts (C9) is the minimum opening, at r_band_in (``inlet_area_mm2`` /
``inlet_area_ratio``); the outer-surface value is ``inlet_area_outer_mm2``.  The constructor raises when the band is
thinner than the body wall, the shelf no longer stands out past the band by printer.min_feature, a strut's inner face is
below min_feature, the inlet ratio is below the minimum, a lead notch / channel no longer opens the step-in through the
band, or a lead's bulkhead hole / bundle reaches the band (``band_step_in``, ``band_clear_*``).

v0.6 (two ATmega128A): the COM board (``electronics.com``) hangs horizontally under the payload bulkhead on four
``payload_module.com_mount`` bosses with blind M3 holes, inside the avionics spigot bore.  Its box, the bosses and
every clearance it needs (spigot bore, tray / battery / ESC / FC / misc / rails below it, bulkhead holes, spring-seat
boss) are derived here and raised on when short (``com_clearance``), so a params change that does not fit fails
before any CAD runs.

v0.6 servo leads (SPEC 3.8 / 5.1, ``servo_leads`` in params and ``edf.mount`` = tabs; ``Layout.servo_leads``): the
vane-servo lead route - pod windows, entry slots, straight annulus columns, deck notches, spigot-ring rebate, intake
servo channel / bulge / notch / bulkhead hole (attributes ``sl_*``) - with every passage sized from the ribbon envelope
and every wall it leaves checked against printer.min_feature, plus the lead lengths and the lumped lead mass / COM
(``sl_lead_mass_g``, ``sl_lead_com``) along ``servo_lead_polyline``.

2026-11-05 logic power (user decisions 1-4, ``logic_power`` = params configs.vtol.logic_power; ``has_logic_power`` only
for the power sources it applies to - the battery build): ``_top_cap`` adds the MTS-102 switch hole to ``cap_holes`` (every
hole rule applies to it) and ``_logic_power`` places the carrier PCB's 1S logic pack at spot A (``lp_pack_*``, on the 6S
pack's -x face), the switch body / nuts / lever (``lp_sw_*``), the lead routes to the carrier's J15 / J14 (``lp_*_lead_*``)
and the carrier's mated plug boxes (``lp_plugs``), with every clearance raised on when short (``lp_clearance``).  Nothing
else in the layout changes; ``logic_in_flight`` mirrors logic_power.in_flight_mass_model (the named mass switch).
"""
from __future__ import annotations

import copy
import json
import math
import os
import sys
from typing import Any

HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS_PATH = os.path.normpath(os.path.join(HERE, "..", "params.json"))

# SPEC literals (not in params.json)
BULKHEAD_HOLE_THETA0_DEG = 45.0   # SPEC 5.3: payload bulkhead holes at theta = 45 + k * 360 / count
PLACEHOLDER_BORE_CLEAR_MM = 1.0   # every electronics placeholder keeps >= 1 mm to the bore it sits in (as the tray
                                  # placeholders' corner test below: body_id / 2 - 1.0)
# v0.6.2 window gable (intake_module.window_bottom_end, user decision 2026-10-15)
GABLE_SLOPE_RANGE_DEG = (30.0, 60.0)   # slope_deg outside 30..60 is rejected (flatter needs support, steeper eats inlet)
GABLE_MIN_EDGE_SLOPE_DEG = 35.0        # the window's lower edge on the curved band surfaces (inner and outer) must stay
                                       # >= 35 deg from horizontal everywhere except the bridged flat (no support in the
                                       # windows when the intake prints flipped; the slicer's support threshold is 30)
GABLE_PROFILE_PAST_MM = 2.0            # modelling helper: the gable cutter profile runs 2 mm past body_r and past the
                                       # window top (the generators' window cutters over-travel 1 mm)
GABLE_SIMPSON_N = 200                  # Simpson intervals over the band radius for the gusset volume / moments
INLET_AREA_SAMPLES = 31                # radii sampled across the band for the minimum inlet area (gussets make the
                                       # opening non-linear in r)
JOINT_SCREW_JOINTS = 3                 # SPEC 2: the three lower joints are screwed (the nose skirt is latched); the
                                       # radio/log box keeps clear of their screw axes (rocket_geometry.joints)


def _box_gap(a: tuple, b: tuple) -> float:
    """Euclidean distance between two axis-aligned boxes ((x0, x1), (y0, y1), (z0, z1)); 0 when they touch or
    overlap."""
    d2 = 0.0
    for (a0, a1), (b0, b1) in zip(a, b):
        g = max(b0 - a1, a0 - b1, 0.0)
        d2 += g * g
    return math.sqrt(d2)


def _box_point_dist(box: tuple, pt: tuple) -> float:
    """Distance from a point to an axis-aligned box ((x0, x1), (y0, y1), (z0, z1)); 0 inside."""
    return math.sqrt(sum(max(lo - c, c - hi, 0.0) ** 2 for (lo, hi), c in zip(box, pt)))


def _box_segment_dist(box: tuple, a: tuple, b: tuple) -> float:
    """Distance from a segment a-b to an axis-aligned box: the point-box distance is convex along the segment, so a
    ternary search finds its minimum (to well below 1e-9 mm after 200 steps)."""
    def f(t: float) -> float:
        return _box_point_dist(box, tuple(a[i] + t * (b[i] - a[i]) for i in range(3)))
    lo, hi = 0.0, 1.0
    for _ in range(200):
        m1, m2 = lo + (hi - lo) / 3.0, hi - (hi - lo) / 3.0
        if f(m1) <= f(m2):
            hi = m2
        else:
            lo = m1
    return min(f(0.0), f(1.0), f(0.5 * (lo + hi)))


def _pt_seg_dist_3d(p: tuple, a: tuple, b: tuple) -> float:
    """Distance from point p to the segment a-b in space (v0.8 GPS noise-source distances)."""
    d = [b[i] - a[i] for i in range(3)]
    dd = sum(x * x for x in d)
    t = 0.0 if dd == 0.0 else max(0.0, min(1.0, sum((p[i] - a[i]) * d[i] for i in range(3)) / dd))
    return math.dist(p, tuple(a[i] + t * d[i] for i in range(3)))


def _seg_seg_dist(a: tuple, b: tuple, c: tuple, d: tuple) -> float:
    """Distance between the segments a-b and c-d in space: the distance from a point of a-b to c-d is convex along a-b,
    so a ternary search on it finds the minimum (v0.8 GPS lead route vs the joint-screw axes)."""
    def f(t: float) -> float:
        return _pt_seg_dist_3d(tuple(a[i] + t * (b[i] - a[i]) for i in range(3)), c, d)
    lo, hi = 0.0, 1.0
    for _ in range(200):
        m1, m2 = lo + (hi - lo) / 3.0, hi - (hi - lo) / 3.0
        if f(m1) <= f(m2):
            hi = m2
        else:
            lo = m1
    return min(f(0.0), f(1.0), f(0.5 * (lo + hi)))


def _pt_seg_dist_2d(p: tuple, a: tuple, b: tuple) -> float:
    """Distance from point p to the segment a-b in the plane."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    dd = dx * dx + dy * dy
    t = 0.0 if dd == 0.0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / dd))
    return math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy))


def _convex_hull_2d(pts: list) -> list:
    """Convex hull of plane points (monotone chain), counter-clockwise, no repeated end point (2026-11-05 logic-power
    strap / tie loops)."""
    ps = sorted(set((float(x), float(y)) for x, y in pts))
    if len(ps) < 3:
        return ps
    def cross(o: tuple, a: tuple, b: tuple) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo: list = []
    for q in ps:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], q) <= 0.0:
            lo.pop()
        lo.append(q)
    up: list = []
    for q in reversed(ps):
        while len(up) >= 2 and cross(up[-2], up[-1], q) <= 0.0:
            up.pop()
        up.append(q)
    return lo[:-1] + up[:-1]


def _signed_dist_convex_2d(poly: list, p: tuple) -> float:
    """Signed distance from a plane point to the boundary of a convex polygon: negative inside, positive outside."""
    d = min(_pt_seg_dist_2d(p, poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly)))
    inside = all((b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) >= 0.0
                 for a, b in zip(poly, poly[1:] + poly[:1]))
    return -d if inside else d


def _convex_gap_2d(pa: list, pb: list) -> float:
    """Gap between two convex polygons in the plane (vertex lists, either winding): 0 when they touch or overlap
    (no separating edge normal), else the smallest vertex-to-edge distance either way (exact for convex polygons)."""
    def edges(poly: list) -> list:
        return [(poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly))]
    separated = False
    for a, b in edges(pa) + edges(pb):
        n = (a[1] - b[1], b[0] - a[0])
        sa = [n[0] * x + n[1] * y for x, y in pa]
        sb = [n[0] * x + n[1] * y for x, y in pb]
        if max(sa) < min(sb) or max(sb) < min(sa):
            separated = True
            break
    if not separated:
        return 0.0
    return min(min(_pt_seg_dist_2d(p, a, b) for p in pa for a, b in edges(pb)),
               min(_pt_seg_dist_2d(p, a, b) for p in pb for a, b in edges(pa)))


def _disc_strip_area(r: float, a: float) -> float:
    """Area of the disc of radius r inside the strip |x| < a (2026-11-05 switch collar: the notch slab)."""
    a = min(max(a, 0.0), r)
    return 2.0 * (a * math.sqrt(r * r - a * a) + r * r * math.asin(a / r))


def _polygon_section(pts: list) -> dict:
    """Plane section properties of a simple polygon (Green's theorem; either winding): area, centroid, centroidal
    Ixx / Iyy / Ixy, principal I_min and the section modulus about the minor principal axis
    (I_min / the farthest vertex measured perpendicular to that axis).  2026-11-05 switch collar side-load hand calc."""
    a = cx = cy = ixx = iyy = ixy = 0.0
    for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]):
        c = x0 * y1 - x1 * y0
        a += c / 2.0
        cx += (x0 + x1) * c / 6.0
        cy += (y0 + y1) * c / 6.0
        ixx += (y0 * y0 + y0 * y1 + y1 * y1) * c / 12.0
        iyy += (x0 * x0 + x0 * x1 + x1 * x1) * c / 12.0
        ixy += (x0 * y1 + 2.0 * x0 * y0 + 2.0 * x1 * y1 + x1 * y0) * c / 24.0
    s = 1.0 if a > 0 else -1.0
    a, cx, cy, ixx, iyy, ixy = a * s, cx * s, cy * s, ixx * s, iyy * s, ixy * s
    cx, cy = cx / a, cy / a
    ixx, iyy, ixy = ixx - a * cy * cy, iyy - a * cx * cx, ixy - a * cx * cy
    m, r = 0.5 * (ixx + iyy), math.hypot(0.5 * (ixx - iyy), ixy)
    i_min = m - r
    # the axis (through the centroid, direction angle t) with the smallest second moment: I(t) = sum (n . p)^2 dA with
    # n = (-sin t, cos t) the in-plane normal of the axis; the bending stress grows with the distance n . p
    def i_about(t: float) -> float:
        nx, ny = -math.sin(t), math.cos(t)
        return iyy * nx * nx + ixx * ny * ny + 2.0 * ixy * nx * ny
    t_min = min((k * math.pi / 3600.0 for k in range(3600)), key=i_about)
    nx, ny = -math.sin(t_min), math.cos(t_min)
    c_max = max(abs((x - cx) * nx + (y - cy) * ny) for x, y in pts)
    return {"area": a, "centroid": (cx, cy), "ixx": ixx, "iyy": iyy, "ixy": ixy, "i_min": i_min,
            "minor_axis_deg": math.degrees(t_min), "c_max_minor": c_max, "z_min": i_about(t_min) / c_max}


def _solve_linear(A: list, b: list) -> list:
    """Small dense linear solve (Gauss elimination with partial pivoting) - no numpy in the Layout."""
    n = len(b)
    M = [list(map(float, row)) + [float(b[i])] for i, row in enumerate(A)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(M[r][c]))
        if abs(M[p][c]) < 1e-300:
            raise ValueError("singular system")
        M[c], M[p] = M[p], M[c]
        for r in range(n):
            if r != c:
                f = M[r][c] / M[c][c]
                for k in range(c, n + 1):
                    M[r][k] -= f * M[c][k]
    return [M[i][n] / M[i][i] for i in range(n)]


def _annular_plate_boss(a: float, b: float, t: float, nu: float) -> dict:
    """2026-11-05 switch collar, review F3 (plate root): Kirchhoff annular plate of thickness t, clamped at the outer
    radius b, its inner edge welded to a RIGID boss of radius a (the collar).  Returns the largest |radial bending stress|
    (at r = a or r = b) per unit load, MPa per N mm of moment M on the boss (tilting, the cos(theta) harmonic
    w = f(r) cos(theta), f = A r^3 + B r ln r + C r + E / r with B = M / (4 pi D) from the concentrated-couple solution,
    boundary f(a) = a phi, f'(a) = phi, f(b) = f'(b) = 0) and MPa per N of axial force P through the boss (axisymmetric
    w = A + B r^2 + C ln r + Dc r^2 ln r with Dc = P / (8 pi D), w'(a) = 0, w(b) = w'(b) = 0).  The stress does not
    depend on the modulus (D cancels).  Roark's rigid-boss cases; checked against the reviewer's numbers (M = 294.8 N mm,
    a 12.9, t 1.2: 6.25 / 8.78 / 10.92 / 13.26 / 15.03 MPa for b = 20 / 25 / 32 / 50 / 200)."""
    D = 1.0     # cancels
    B = 1.0 / (4.0 * math.pi * D)
    A_, C_, E_, _phi = _solve_linear(
        [[a ** 3, a, 1.0 / a, -a], [3.0 * a * a, 1.0, -1.0 / (a * a), -1.0], [b ** 3, b, 1.0 / b, 0.0],
         [3.0 * b * b, 1.0, -1.0 / (b * b), 0.0]],
        [-B * a * math.log(a), -B * (math.log(a) + 1.0), -B * b * math.log(b), -B * (math.log(b) + 1.0)])

    def mr_m(r: float) -> float:
        f = A_ * r ** 3 + B * r * math.log(r) + C_ * r + E_ / r
        fp = 3.0 * A_ * r * r + B * (math.log(r) + 1.0) + C_ - E_ / (r * r)
        fpp = 6.0 * A_ * r + B / r + 2.0 * E_ / r ** 3
        return -D * (fpp + nu * (fp / r - f / (r * r)))
    Dc = 1.0 / (8.0 * math.pi * D)
    A2, B2, C2 = _solve_linear(
        [[0.0, 2.0 * a, 1.0 / a], [1.0, b * b, math.log(b)], [0.0, 2.0 * b, 1.0 / b]],
        [-Dc * (2.0 * a * math.log(a) + a), -Dc * b * b * math.log(b), -Dc * (2.0 * b * math.log(b) + b)])

    def mr_p(r: float) -> float:
        wpp = 2.0 * B2 - C2 / (r * r) + Dc * (2.0 * math.log(r) + 3.0)
        wp = 2.0 * B2 * r + C2 / r + Dc * (2.0 * r * math.log(r) + r)
        return -D * (wpp + nu * wp / r)
    k = 6.0 / (t * t)
    return {"sigma_per_Nmm_moment": k * max(abs(mr_m(a)), abs(mr_m(b))),
            "sigma_per_N_axial": k * max(abs(mr_p(a)), abs(mr_p(b)))}


def _ray_exit(px: float, py: float, dx: float, dy: float, circles_out: list, circles_in: list, rects: list) -> float:
    """2026-11-05 switch collar plate root: distance from (px, py) along the unit direction (dx, dy) to the first support
    - a circle the ray leaves the inside of (the rim ring's inner radius), a circle it enters from outside (the hub) or a
    rib rectangle (theta, u0, u1, half width).  inf when nothing is met."""
    best = math.inf
    for r in circles_out:                        # the point is inside: s^2 + 2 s (p.d) + |p|^2 - r^2 = 0, the + root
        pd, pp = px * dx + py * dy, px * px + py * py
        disc = pd * pd - (pp - r * r)
        if disc >= 0.0:
            s = -pd + math.sqrt(disc)
            if s > 0.0:
                best = min(best, s)
    for r in circles_in:                         # the point is outside: the - root, when in front
        pd, pp = px * dx + py * dy, px * px + py * py
        disc = pd * pd - (pp - r * r)
        if disc >= 0.0:
            s = -pd - math.sqrt(disc)
            if s > 0.0:
                best = min(best, s)
    for th, u0, u1, hw in rects:                 # slab method in the rib's (u, v) frame
        t = math.radians(th)
        c, s_ = math.cos(t), math.sin(t)
        pu, pv = px * c + py * s_, -px * s_ + py * c
        du, dv = dx * c + dy * s_, -dx * s_ + dy * c
        lo, hi = 0.0, math.inf
        for p0, d0, a0, a1 in ((pu, du, u0, u1), (pv, dv, -hw, hw)):
            if abs(d0) < 1e-15:
                if not (a0 <= p0 <= a1):
                    lo, hi = 1.0, 0.0
                continue
            t0, t1 = (a0 - p0) / d0, (a1 - p0) / d0
            lo, hi = max(lo, min(t0, t1)), min(hi, max(t0, t1))
        if lo <= hi and lo > 0.0:
            best = min(best, lo)
    return best


def _polar_rect_2d(theta_deg: float, u0: float, u1: float, v0: float, v1: float) -> list:
    """Plan corners of the box u in [u0, u1] (radial along theta), v in [v0, v1] (tangential)."""
    t = math.radians(theta_deg)
    c, s = math.cos(t), math.sin(t)
    return [(u * c - v * s, u * s + v * c) for u, v in ((u0, v0), (u1, v0), (u1, v1), (u0, v1))]


def _wrap180(a: float) -> float:
    """Angle a (deg) wrapped to [-180, 180)."""
    return (a + 180.0) % 360.0 - 180.0


def _convex_overlap(pa: list, pb: list) -> bool:
    """True when two convex polygons (vertex lists) overlap or touch (no separating axis, SAT)."""
    for poly in (pa, pb):
        n_ = len(poly)
        for i in range(n_):
            a, b = poly[i], poly[(i + 1) % n_]
            nx, ny = a[1] - b[1], b[0] - a[0]
            sa = [nx * x + ny * y for x, y in pa]
            sb = [nx * x + ny * y for x, y in pb]
            if max(sa) < min(sb) - 1e-12 or max(sb) < min(sa) - 1e-12:
                return False
    return True


def _in_convex(pt: tuple, poly: list) -> bool:
    """Point inside (or on) a convex polygon of either winding."""
    sgn = 0
    n_ = len(poly)
    for i in range(n_):
        a, b = poly[i], poly[(i + 1) % n_]
        c = (b[0] - a[0]) * (pt[1] - a[1]) - (b[1] - a[1]) * (pt[0] - a[0])
        if abs(c) < 1e-12:
            continue
        s = 1 if c > 0 else -1
        if sgn == 0:
            sgn = s
        elif s != sgn:
            return False
    return True


def _rot_xy(pt: tuple, theta_deg: float) -> tuple:
    t = math.radians(theta_deg)
    c, s = math.cos(t), math.sin(t)
    return (pt[0] * c - pt[1] * s, pt[0] * s + pt[1] * c)


def clock_fan_outline(theta_deg: float, e_deg: float, u_in: float, r_w: float, hw: float, inner: str = "rect",
                      arc_step_deg: float = 1.0) -> list[tuple[float, float]]:
    """v0.9 clock mount (wire09c): plan outline (CCW, vehicle XY) of the union of the radial rectangles u in [u_in, r_w],
    |v| <= hw turned to theta + e for every |e| <= e_deg - the lead passage ('fan').  Exact: two straight sides (the
    rectangles at theta +- e_deg), the outer corner arc r = hypot(r_w, hw) between them (sampled INSIDE the arc every
    arc_step_deg - chords, never outside), and the inner end: 'rect' = the inner edges of the two extreme rectangles and
    the arc r = u_in between them (non-convex; u_in inside the jet / void); 'circle' = the circle r = u_in between the two
    side lines (the outline minus the disc r < u_in, used to keep the blister out of the EDF bore); 'chord' = the convex
    hull (the inner end closed by a chord - only for overlap tests on shapes that stay outside r = hypot(u_in, hw))."""
    a = e_deg
    rc = math.hypot(r_w, hw)
    al = math.degrees(math.atan2(hw, r_w))
    pts: list[tuple[float, float]] = []
    if inner == "circle":
        if u_in <= hw:
            raise ValueError("clock_fan_outline: the inner circle lies inside the side lines' offset")
        u0 = math.sqrt(u_in * u_in - hw * hw)
    else:
        u0 = u_in
    pts.append(_rot_xy((u0, -hw), -a))
    pts.append(_rot_xy((r_w, -hw), -a))
    a0, a1 = -a - al, a + al
    n = max(2, int(math.ceil((a1 - a0) / arc_step_deg)))
    for k in range(1, n):
        ph = math.radians(a0 + (a1 - a0) * k / n)
        pts.append((rc * math.cos(ph), rc * math.sin(ph)))
    pts.append(_rot_xy((r_w, hw), a))
    pts.append(_rot_xy((u0, hw), a))
    if inner == "rect":
        pts.append(_rot_xy((u_in, 0.0), a))
        m = max(2, int(math.ceil(2 * a / arc_step_deg)))
        for k in range(1, m):
            ph = math.radians(a - 2 * a * k / m)
            pts.append((u_in * math.cos(ph), u_in * math.sin(ph)))
        pts.append(_rot_xy((u_in, 0.0), -a))
    elif inner == "circle":
        b = a + math.degrees(math.atan2(hw, u0))
        m = max(2, int(math.ceil(2 * b / arc_step_deg)))
        for k in range(1, m):
            ph = math.radians(b - 2 * b * k / m)
            pts.append((u_in * math.cos(ph), u_in * math.sin(ph)))
    elif inner != "chord":
        raise ValueError(inner)
    return [_rot_xy(p_, theta_deg) for p_ in pts]


def clock_fan_half_width(u: float, e_deg: float, r_w: float, hw: float) -> float:
    """Half width (along v, in the frame of the passage axis) of the clock_fan_outline at distance u along the axis:
    min(side line (hw + u sin e) / cos e, outer arc sqrt(hypot(r_w, hw)^2 - u^2))."""
    t = math.radians(e_deg)
    side = (hw + u * math.sin(t)) / math.cos(t)
    rc2 = r_w * r_w + hw * hw
    arc = math.sqrt(rc2 - u * u) if u * u < rc2 else 0.0
    return min(side, arc)


def _simpson(f, a: float, b: float, n: int) -> float:
    """Composite Simpson rule of f over [a, b] with n (even) intervals."""
    n += n % 2
    h = (b - a) / n
    s = f(a) + f(b)
    for k in range(1, n):
        s += (4.0 if k % 2 else 2.0) * f(a + k * h)
    return s * h / 3.0


# --------------------------------------------------------------------------- #
# params loading
# --------------------------------------------------------------------------- #
def _deep_update(base: dict, override: dict) -> dict:
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _deep_update(base[k], v)
        else:
            base[k] = copy.deepcopy(v)
    return base


CONFIGS = ("rocket", "vtol")                 # v0.9 CAD-2: mission.config (hardware configuration)
POWER_SOURCES = ("battery", "tether", "hybrid")   # v0.9 CAD-2: power.source (VTOL only)
POWER_ALIASES = {"wired": "tether", "wired_buffer": "hybrid"}   # the tether report's names


def load_params(path: str = PARAMS_PATH, preset: str | None = None, config: str | None = None,
                power: str | None = None) -> dict:
    """Load params.json; apply ``alt_presets[preset]`` on top when given; then (v0.9 CAD-2) the hardware configuration
    ``config`` (default ``mission.config``, 'rocket' when absent): 'vtol' deep-merges ``configs.vtol`` over the result and
    sets ``mission.config``; ``power`` (default ``power.source``) sets ``power.source`` ('battery' | 'tether' | 'hybrid',
    the tether report's 'wired' / 'wired_buffer' accepted).  The rocket configuration loads exactly the v0.9 CAD-1 dict
    (configs / power blocks untouched), so every rocket artifact stays bit-identical."""
    with open(path, "r", encoding="utf-8") as f:
        p = json.load(f)
    if preset and preset != p["meta"].get("preset"):
        presets = p.get("alt_presets", {})
        if preset not in presets:
            raise KeyError(f"unknown preset {preset!r}; have {list(presets)}")
        over = copy.deepcopy(presets[preset])
        over.pop("note", None)
        _deep_update(p, over)
        p["meta"]["preset"] = preset
    cfg = config or (p.get("mission") or {}).get("config") or "rocket"
    if cfg not in CONFIGS:
        raise KeyError(f"unknown mission.config {cfg!r}; have {list(CONFIGS)}")
    if cfg != "rocket":
        cfgs = p.get("configs") or {}
        if cfg not in cfgs:
            raise KeyError(f"params.json has no configs.{cfg}")
        over = copy.deepcopy(cfgs[cfg])
        over.pop("note", None)
        _deep_update(p, over)
    if "mission" in p and (cfg != "rocket" or "config" in p["mission"]):
        p["mission"]["config"] = cfg
    if power is not None or (cfg != "rocket" and isinstance(p.get("power"), dict)):
        src = POWER_ALIASES.get(power or p["power"].get("source", "battery"), power or p["power"].get("source", "battery"))
        if src not in POWER_SOURCES:
            raise KeyError(f"unknown power.source {src!r}; have {list(POWER_SOURCES)}")
        if src != "battery" and cfg != "vtol":
            raise ValueError(f"power.source {src!r} applies to mission.config 'vtol' only (the rocket is battery-powered)")
        p["power"]["source"] = src
    return p


def config_suffix(config: str | None, power: str | None = None) -> str:
    """v0.9 CAD-2: report-file suffix of a hardware configuration / power source: '' for the rocket (and for the default
    battery source), '_vtol', '_vtol_tether', '_vtol_hybrid'."""
    if not config or config == "rocket":
        return ""
    src = POWER_ALIASES.get(power or "battery", power or "battery")
    return f"_{config}" + ("" if src == "battery" else f"_{src}")


# --------------------------------------------------------------------------- #
# layout
# --------------------------------------------------------------------------- #
class Layout:
    """All derived dimensions.  Attributes are plain floats in mm / deg.

    Every attribute name below is referenced by name in docs/SPEC.md.
    """

    def __init__(self, p: dict):
        self.p = p
        b, e, pm, v, s, pod, fin = (p["body"], p["edf"], p["propulsion_module"],
                                     p["vane"], p["servo"], p["pod"], p["fin"])
        im, am, pl, no, lp = (p["intake_module"], p["avionics_module"],
                              p["payload_module"], p["nose"], p["launch_pad"])
        pay = p["payload"]
        # v0.9 CAD-2: hardware configuration (mission.config, applied by load_params) and the VTOL power source
        self.config = (p.get("mission") or {}).get("config") or "rocket"
        if self.config not in CONFIGS:
            raise ValueError(f"mission.config must be one of {CONFIGS}, not {self.config!r}")
        self.vtol = self.config == "vtol"
        self.power_source = ((p.get("power") or {}).get("source") or "battery") if self.vtol else "battery"
        if self.power_source not in POWER_SOURCES:
            raise ValueError(f"power.source must be one of {POWER_SOURCES}, not {self.power_source!r}")
        self.has_battery = self.power_source != "tether"
        # v0.9 logic-power round (user decisions 2026-11-05): the carrier's 1S logic pack + top-cap switch, battery build
        # only (logic_power.applies_to_power_sources); set in _top_cap / _logic_power
        lpw = p.get("logic_power") if self.vtol else None
        self.has_logic_power = bool(lpw) and self.power_source in (lpw.get("applies_to_power_sources") or [])
        self.logic_in_flight = bool(lpw.get("in_flight_mass_model")) if self.has_logic_power else False
        self.has_lp_collar = False       # switch-collar round (2026-11-05): set by _logic_power (logic_power.collar)

        # ---------------- body tube ----------------
        self.body_od = b["od"]
        self.body_wall = b["wall"]
        self.body_id = self.body_od - 2 * self.body_wall
        self.body_r = self.body_od / 2
        self.spigot_h = b["spigot_h"]
        self.spigot_od = self.body_id - b["spigot_clearance_dia"]
        self.spigot_id = self.spigot_od - 2 * b["spigot_wall"]
        self.joint_screw_dz = b["joint_screw_z_from_spigot_bottom"]
        self.joint_thetas = [b["joint_screw_theta0_deg"] + i * 360.0 / b["joint_screw_count"]
                             for i in range(b["joint_screw_count"])]
        # v0.5d: the propulsion spigot ring is solid down to the EDF bore; its joint holes are blind
        # (prop_joint_hole_depth from the spigot OD) so a long screw bottoms in plastic, never on the EDF housing
        self.prop_joint_hole_depth = b.get("prop_joint_hole_depth")

        # ---------------- propulsion module ----------------
        self.duct_id = pm["duct_id"]
        self.duct_r = self.duct_id / 2
        self.vane_section_h = pm["vane_section_h"]
        self.vane_section_od = pm["vane_section_od"]
        self.vane_section_r = self.vane_section_od / 2

        self.edf_bore_d = e["housing_od"] + e["bore_clearance_dia"]
        self.edf_bore_r = self.edf_bore_d / 2
        # v0.5: edf.mount = "tabs" -> the EDF hangs on its two moulded tabs (screws), the bell lip floats in a
        # pocket; "lip_clamp" (v0.2-v0.4) -> the flat lip is clamped between the spigot top and the intake shelf.
        self.edf_mount = e.get("mount", "lip_clamp")
        if self.edf_mount not in ("lip_clamp", "tabs"):
            raise ValueError(f"edf.mount must be 'lip_clamp' or 'tabs', not {self.edf_mount!r}")
        self.tabs = self.edf_mount == "tabs"
        if self.tabs:
            self.edf_lip_t = e["lip_depth"]                       # axial depth of the bell lip (envelope)
            self.edf_len = e["housing_len_from_lip_front"]        # lip front face -> housing rear end
            self.lip_gap_below = im["lip_gap_below"]              # spigot top -> bell rear
        else:
            self.edf_lip_t = e["lip_t_ASSUMED"]
            self.edf_len = e["len_lip_top_to_housing_bottom_ASSUMED"]
            self.lip_gap_below = 0.0
        self.edf_housing_len = self.edf_len - self.edf_lip_t

        # Z stations of the propulsion module
        self.z_exit = 0.0
        self.z_vane_section_top = self.vane_section_h                       # 90
        self.z_prop_spigot_top = None  # set below once module height known
        # The EDF lip rests on the propulsion module top face (spigot top).
        # Module height = vane section + EDF section; EDF section must be tall
        # enough for the housing + shoulder + sleeve ring.
        self.sleeve_wall = pm["sleeve_wall"]
        self.cone_wall = pm["cone_wall"]
        self.shoulder_ring_min_h = float(pm.get("shoulder_ring_min_h", 18.0))
        min_edf_section = self.edf_housing_len - self.lip_gap_below + e["bottom_clearance"] + self.shoulder_ring_min_h
        self.edf_section_h = max(float(pm.get("edf_section_min_h", 125.0)), math.ceil(min_edf_section))
        self.z_prop_cone_top = self.z_vane_section_top + self.edf_section_h  # 215 nominal -> see spec
        # spigot sits on top of the cone shell
        self.z_prop_spigot_bottom = self.z_prop_cone_top - self.spigot_h     # cone shell top face
        self.z_prop_spigot_top = self.z_prop_cone_top                        # EDF lip bottom face
        self.z_edf_lip_bottom = self.z_prop_spigot_top + self.lip_gap_below
        self.z_edf_lip_top = self.z_edf_lip_bottom + self.edf_lip_t                # tabs: bell front face
        self.z_edf_housing_bottom = self.z_edf_lip_top - self.edf_len
        if self.tabs:
            # the tabs locate the EDF axially; the bore keeps the minimum shoulder ring and a liner tube
            # (duct bore -> sleeve OD) fills the bore under the housing so the jet sees no step
            self.z_edf_shoulder = self.z_vane_section_top + self.shoulder_ring_min_h
            self.z_liner_top = self.z_edf_housing_bottom - e["bottom_clearance"]
            # thin liner: duct-bore tube of liner_wall plus a top ring out to the sleeve (closed cavity behind it)
            self.liner_wall = pm["liner_wall"]
            self.liner_r_out = self.duct_r + self.liner_wall
            self.z_liner_ring_bottom = self.z_liner_top - self.liner_wall
            if self.z_liner_ring_bottom < self.z_edf_shoulder + self.liner_wall:
                raise ValueError("EDF housing ends too close to the shoulder ring: raise edf_section_min_h")
        else:
            self.z_edf_shoulder = self.z_edf_housing_bottom - e["bottom_clearance"]  # bore -> duct step
            self.z_liner_top = None
        if self.z_edf_shoulder < self.z_vane_section_top + 10.0:
            raise ValueError("EDF too long for the propulsion module: raise edf_section_h")
        self.z_prop_top = self.z_prop_spigot_top
        self.prop_module_h = self.z_prop_top - self.z_exit

        # outer cone: OD vane_section_od at z_vane_section_top -> body_od at cone top face.
        # If the EDF sleeve is wider than the vane section (a fat housing on a thin duct), the outer
        # surface first widens at 45 deg (prints without support) to sleeve_r + cone_wall, and the
        # linear cone starts there: cone_z0 / cone_r_bottom are the chamfer top.  The chamfer lies
        # inside the solid shoulder ring, so every consumer must take the profile from
        # cone_outer_radius() / shoulder_ring_profile() (single source).
        self.sleeve_od = self.edf_bore_d + 2 * self.sleeve_wall
        self.sleeve_r = self.sleeve_od / 2
        self.z_sleeve_bottom = self.z_edf_shoulder          # solid ring below this down to z_vane_section_top
        self.cone_r_top = self.body_r
        self.cone_r_bottom = max(self.vane_section_r, self.sleeve_r + self.cone_wall)
        self.cone_chamfer_h = self.cone_r_bottom - self.vane_section_r     # 45 deg
        self.cone_z0 = self.z_vane_section_top + self.cone_chamfer_h
        # ---- v0.5 tab mount: keyways, screw bosses ("saddle blocks"), cylindrical outer section ----
        # wire09c (user 2026-10-30): propulsion_module.tab_mount 'screws' (v0.5: two keyways at the tabs + tangential tab
        # screws in lugs) or 'clock_keyways' (keyways round the circle, no screws; the EDF is clocked to the lead passage)
        self.tab_mount = pm.get("tab_mount", "screws")
        if self.tab_mount not in ("screws", "clock_keyways"):
            raise ValueError(f"propulsion_module.tab_mount must be 'screws' or 'clock_keyways', not {self.tab_mount!r}")
        self.clock_mount = bool(self.tabs and self.tab_mount == "clock_keyways")
        if self.tabs:
            t, sc = e["tab"], pm["tab_screw"]
            self.tab_count = int(t["count"])
            self.tab_thetas = [t["theta0_deg"] + i * 360.0 / self.tab_count for i in range(self.tab_count)]
            self.tab_t = t["t_ASSUMED"]
            self.tab_r_in = e["housing_od"] / 2
            self.tab_r_out = t["outer_r"]
            self.tab_len = t["len"]
            self.tab_z_center = self.z_edf_lip_top - t["center_from_lip_front"]
            self.tab_z0 = self.tab_z_center - self.tab_len / 2
            self.tab_z1 = self.tab_z_center + self.tab_len / 2
            self.tab_slot_r = t["slot_r"]
            self.tab_slot_w = t["slot_w"]
            self.tab_slot_len_radial = t["slot_len_radial"]
            self.tab_slot_zs = [self.tab_z_center + k * t["slot_pitch"] for k in (-1, 0, 1)]
            self.tab_screw_zs = [self.tab_z_center + k * t["slot_pitch"] for k in t["screw_slot_index"]]
            c = pm["tab_keyway_clearance"]
            self.keyway_w = self.tab_t + 2 * c                      # tangential
            self.keyway_r_in = self.edf_bore_r - 0.5                # opens into the bore
            self.keyway_r_out = self.tab_r_out + c
            self.keyway_z0 = self.tab_z0                            # keyway floor = axial seat of the tab
            self.keyway_z1 = self.z_prop_spigot_top + 1.0           # open at the top (EDF drops in)
            # screw axis: tangential line through the slot centre, P(t) = r_s e_r(th) + head_side t e_th(th)
            self.tab_screw_size = sc["size"]
            self.tab_screw_len = sc["len"]
            self.tab_screw_r = self.tab_slot_r
            if sc["head_side"] not in (1, -1) or isinstance(sc["head_side"], bool):
                raise ValueError("propulsion_module.tab_screw.head_side must be +1 or -1")
            self.tab_screw_head_side = int(sc["head_side"])
            self.tab_screw_d_clear, self.tab_screw_d_tap, self.tab_screw_d_cbore = sc["d_clear"], sc["d_tap"], sc["d_cbore"]
            self.tab_screw_t_out = math.sqrt(self.body_r ** 2 - self.tab_screw_r ** 2)   # leaves the outer skin
            self.tab_screw_t_head = sc["t_head"]                    # head seat (counterbore floor)
            self.tab_screw_t_tip = self.tab_screw_t_head - self.tab_screw_len
            self.tab_screw_t_keyway = self.keyway_w / 2             # keyway faces at +-t_keyway
            self.tab_screw_t_tap_end = self.tab_screw_t_tip - 1.0
            self.tab_screw_engagement = -self.tab_screw_t_keyway - self.tab_screw_t_tip
            if self.tab_screw_engagement < sc["min_engagement"]:
                raise ValueError(f"tab screw thread engagement {self.tab_screw_engagement:.1f} mm < {sc['min_engagement']}")
            r_seat = math.hypot(self.tab_screw_r, self.tab_screw_t_head)
            self.tab_cbore_wall_to_bore = r_seat - self.tab_screw_d_cbore / 2 - self.edf_bore_r
            # every hole segment, at its innermost point (the keyway face or the head seat), keeps min_wall to the bore
            walls = {"counterbore": self.tab_cbore_wall_to_bore,
                     "clearance hole": math.hypot(self.tab_screw_r - self.tab_screw_d_clear / 2, self.tab_screw_t_keyway) - self.edf_bore_r,
                     "tap hole": math.hypot(self.tab_screw_r - self.tab_screw_d_tap / 2, self.tab_screw_t_keyway) - self.edf_bore_r}
            self.tab_hole_wall_to_bore = min(walls.values())
            for nm, wv in walls.items():
                if wv < sc["min_wall_to_bore"]:
                    raise ValueError(f"tab screw {nm} leaves {wv:.2f} mm to the EDF bore")
            if self.tab_screw_t_head > self.tab_screw_t_out - 2.0:
                raise ValueError("tab screw head seat outside the body")
            # the counterbore must clear the skin along its whole circle: its edge nearest the axis (r_s - d/2)
            # leaves r = body_r last, at t = sqrt(body_r^2 - (r_s - d/2)^2)
            r_cb_in = self.tab_screw_r - self.tab_screw_d_cbore / 2
            self.tab_screw_t_cbore_end = math.sqrt(self.body_r ** 2 - r_cb_in ** 2) + sc["cbore_exit_margin"]
            mdeg = sc["block_margin_deg"]
            self.block_theta_head = math.degrees(math.atan2(self.tab_screw_t_cbore_end, r_cb_in)) + mdeg
            self.block_theta_far = math.degrees(math.atan2(-self.tab_screw_t_tap_end + self.tab_screw_d_tap / 2 + 1.0,
                                                           self.tab_screw_r)) + mdeg
            # screw lugs: one per screw (merged when they touch); the lowest also forms the keyway floor
            zs = sorted(self.tab_screw_zs)
            half = self.tab_screw_d_cbore / 2 + sc["block_margin_z"]
            ranges = [[z - half, z + half] for z in zs]
            ranges[0][0] = min(ranges[0][0], self.keyway_z0 - sc["floor_t"])
            ranges[-1][1] = self.z_prop_spigot_bottom                 # top lug merges with the deck
            merged: list[list[float]] = []
            for r0, r1 in ranges:
                if merged and r0 <= merged[-1][1]:
                    merged[-1][1] = max(merged[-1][1], r1)
                else:
                    merged.append([r0, r1])
            self.boss_z_ranges = [tuple(r) for r in merged]
            self.block_z0 = self.boss_z_ranges[0][0]
            self.block_z1 = self.boss_z_ranges[-1][1]
            self.block_r_in = self.sleeve_r
            self.z_cyl0 = self.keyway_z0 - pm["cyl_margin_below_tab"]  # outer skin is a cylinder above this
            if self.z_cyl0 < self.cone_z0 + 5.0:
                raise ValueError("tab keyway too low: no room for the cone below the cylindrical section")
            if self.block_z0 < self.z_cyl0 + 1.0:
                raise ValueError("tab screw boss starts below the cylindrical section")
            if self.keyway_r_out > self.body_r - self.cone_wall - 0.8:
                raise ValueError("tab keyway too close to the outer skin")
            # wire passage: housing exit slot -> slot through sleeve / deck / spigot ring (open at the top)
            we, ws = e["wire_exit"], pm["wire_slot"]
            # v0.9 lead-exit round (user photos 2026-10-29): the leads leave the motor's side at its front ring (stator /
            # rim level) and follow a stator vane under the housing's rear rim (we["motor_exit"], Layout._motor_lead_lower;
            # round 1's climb along the can is superseded); params without it are the
            # pre-2026-10-29 housing-side slot (v0.9: read from the vendor drawing, keys *_ASSUMED; the old names still load)
            self.wire_exit_motor = isinstance(we.get("motor_exit"), dict)
            if self.wire_exit_motor:
                self.wire_exit_theta_from_tab = we["motor_exit"]["theta_from_tab_deg_ASSUMED"]
                self.wire_exit_z_from_lip_front = None
            else:
                self.wire_exit_theta_from_tab = we.get("theta_from_tab_deg_ASSUMED", we.get("theta_from_tab_deg"))
                self.wire_exit_z_from_lip_front = list(we.get("z_from_lip_front_ASSUMED", we.get("z_from_lip_front")))
            self.wire_theta = self.tab_thetas[0] + self.wire_exit_theta_from_tab
            if self.clock_mount:
                # wire09c clock mount (user 2026-10-30 'be generous, no measuring'): the EDF is turned until its factory
                # lead bundle points into ONE fixed lead passage at lead_passage.theta_deg; the tabs drop into the
                # nearest of many keyways (Layout._clock_seat).  The exit angle is not an input any more.
                self.wire_theta = float(pm["lead_passage"]["theta_deg"])
                self._clock_fan(p)
            self.wire_slot_w = ws["w"]
            # v0.9 fix round: the inner face sits 0.5 inside the EDF bore at the slot's CORNERS (|v| = w/2), not at its
            # centre - with the 22 mm slot the old edf_bore_r - 0.5 left a 0-0.73 mm crescent of the sleeve beside it
            self.wire_slot_r_in = min(self.edf_bore_r,
                                      math.sqrt(max(self.edf_bore_r ** 2 - (self.wire_slot_w / 2) ** 2, 0.0))) - 0.5
            # two boxes: below the cone-shell top the slot stops short of the skin (its corners at |v| = w/2 keep
            # skin_min_wall); in the spigot ring (above z_prop_spigot_bottom) it breaks through to the skirt
            self.wire_slot_r_out = self.spigot_od / 2 + 0.1
            self.wire_slot_lower_r_out = math.sqrt((self.body_r - self.cone_wall - ws["skin_min_wall"]) ** 2
                                                   - (ws["w"] / 2) ** 2)
            if self.wire_exit_motor:
                # the 22 mm box keeps its pre-2026-10-29 start; the lead notch / lower slot below it is ml_low_cut_uz
                self.wire_slot_z0 = self.z_edf_lip_top - ws["wide_z0_from_lip_front"]
            else:
                self.wire_slot_z0 = self.z_edf_lip_top - max(self.wire_exit_z_from_lip_front) - ws["below_exit"]
            self.wire_slot_z1 = self.z_prop_spigot_top + 1.0
            if self.wire_slot_lower_r_out < self.sleeve_r + 1.0:
                raise ValueError("wire slot cannot reach past the sleeve")
            if self.wire_slot_z0 < self.z_cyl0:
                raise ValueError("wire slot reaches below the cylindrical section")
        else:
            self.z_cyl0 = self.z_prop_spigot_bottom
        self.cone_z1 = self.z_cyl0
        if self.clock_mount:
            self._clock_pads(p)
        if self.cone_z0 > self.z_edf_shoulder - 1.0:
            raise ValueError(f"cone chamfer top z={self.cone_z0:.1f} is above the EDF shoulder z={self.z_edf_shoulder:.1f}")
        if self.cone_r_bottom > self.cone_r_top:
            raise ValueError("EDF sleeve wider than the body")
        housing_r = e["housing_od"] / 2
        r_in_at_housing = self.cone_outer_radius(self.z_edf_housing_bottom) - self.cone_wall
        if r_in_at_housing < housing_r + 0.3:
            raise ValueError(f"EDF housing r {housing_r} too close to the cone inner wall r {r_in_at_housing:.2f}")
        self.web_thetas = [pm["web_theta0_deg"] + i * 360.0 / pm["web_count"] for i in range(pm["web_count"])]
        self.web_t = pm["web_t"]
        if self.tabs:
            def _in_arc(x: float, lo: float, span: float) -> bool:
                """x (deg) inside the arc [lo, lo + span] modulo 360."""
                return (x - lo) % 360.0 <= span
            ww = math.degrees(self.web_t / 2 / self.sleeve_r) + 1.0             # half web + 1 deg margin
            hw_slot = math.degrees(math.atan2(self.wire_slot_w / 2, self.wire_slot_r_in)) + 1.0
            if self.clock_mount:
                # clock mount: no lugs / screws; the lead passage (fan) must keep every web whole (its plan rectangle
                # web_t + 2 web_margin wide from the bore to the skin)
                ck = pm["clock_keyways"]
                for w in self.web_thetas:
                    web = _polar_rect_2d(w, self.edf_bore_r - 1.0, self.body_r, -(self.web_t / 2 + ck["web_margin"]),
                                         self.web_t / 2 + ck["web_margin"])
                    if _convex_overlap(self.lp_fan_hull, web):
                        raise ValueError(f"lead passage (fan) at {self.wire_theta:g} deg cuts the web at {w:g} deg")
            for th in ([] if self.clock_mount else self.tab_thetas):
                hs = self.tab_screw_head_side
                lo, hi = sorted((th - hs * self.block_theta_far, th + hs * self.block_theta_head))
                for w in self.web_thetas:
                    if _in_arc(w, lo - ww, (hi - lo) + 2 * ww):
                        raise ValueError(f"tab screw lug at {th} deg overlaps the web at {w} deg")
                # the wire slot (with its angular half-width) must not enter any lug sector
                if _in_arc(self.wire_theta, lo - hw_slot, (hi - lo) + 2 * hw_slot):
                    raise ValueError(f"wire slot at {self.wire_theta} deg overlaps the tab lug at {th} deg")
            for w in ([] if self.clock_mount else self.web_thetas):
                if _in_arc(self.wire_theta, w - ww - hw_slot, 2 * (ww + hw_slot)):
                    raise ValueError("wire slot overlaps a web")

        # ---------------- vanes ----------------
        self.vane_count = v["count"]
        self.vane_thetas = [v["theta0_deg"] + i * 360.0 / v["count"] for i in range(v["count"])]
        self.vane_chord = v["chord"]
        self.vane_t = v["t"]
        self.vane_hinge_z = v["hinge_z"]
        self.vane_le_z = self.vane_hinge_z + v["hinge_chord_fraction_from_le"] * v["chord"]
        self.vane_te_z = self.vane_le_z - v["chord"]
        self.vane_r_in = v["span_r_in"]
        self.vane_r_out = v["span_r_out"]
        self.vane_span = self.vane_r_out - self.vane_r_in
        self.vane_area_mm2 = self.vane_span * self.vane_chord
        self.vane_shaft_d = v["shaft_d"]
        self.vane_shaft_r_in = v["shaft_r_in"]
        self.vane_shaft_r_out = v["shaft_r_out"]
        self.hinge_bore_d = pm["hinge_bore_d"]
        self.hinge_boss_od = pm["hinge_boss_od"]
        self.hinge_boss_r_out = pm["hinge_boss_r_out"]
        self.vane_d_flat_r_in = v["d_flat_r_in"]
        if self.vane_te_z < self.z_exit + 5.0:
            raise ValueError("vane trailing edge too close to exit plane")
        if self.vane_le_z > self.z_edf_shoulder - 5.0:
            raise ValueError("vane leading edge collides with EDF shoulder")
        # v0.9 (user 2026-10-27, decision 1): vane_drive.mode 'direct' = servo coaxial in a fin-root fairing (no pods,
        # horns, arms, pushrods); 'pushrod' = the v0.2-v0.8 linkage.  Params without the block are the pushrod design.
        self.vane_drive_mode = (p.get("vane_drive") or {}).get("mode", "pushrod")
        if self.vane_drive_mode not in ("direct", "pushrod"):
            raise ValueError(f"vane_drive.mode must be 'direct' or 'pushrod', not {self.vane_drive_mode!r}")
        self.direct_drive = self.vane_drive_mode == "direct"

        # ---------------- servo pods (vane; pushrod mode) / pod cup widths (release pod, both modes) ----------------
        self.pod_cavity_w = pod["cavity_tangential"]
        self.pod_w = self.pod_cavity_w + 2 * pod["wall_tangential"]
        self.pod_wall_t = pod["wall_tangential"]
        self.pod_floor_t = pod["floor_t"]
        # servo output shaft is offset along the body length; put it toward the vane horn (release pod: toward the arm)
        self.servo_shaft_tangential_offset = -s["shaft_offset_along_l"]
        if not self.direct_drive:
            self.horn_r_in = v["horn_r_in"]
            self.horn_r_out = v["horn_r_in"] + v["horn_t"]
            self.horn_tip_z = self.vane_hinge_z + v["horn_arm_len"]
            self.horn_tail_z = self.vane_hinge_z - v["horn_tail_len"]
            self.pod_thetas = [t + pod["theta_offset_from_vane_deg"] for t in self.vane_thetas]
            self.pod_cavity_r_in = self.vane_section_r + 1.0         # flat inner face of cavity
            self.pod_cavity_r_out = self.pod_cavity_r_in + pod["cavity_radial"]
            self.pod_r_out = self.pod_cavity_r_out + pod["wall_radial"]
            self.pod_r_in = self.vane_section_r - 3.0                # box overlaps into duct wall; bore re-cut
            # servo arm plane = horn tip height so the pushrod runs horizontally
            self.servo_arm_z_bottom = self.horn_tip_z - 1.0
            self.servo_arm_z_top = self.servo_arm_z_bottom + pod["arm_t"]
            self.servo_bottom_z = self.servo_arm_z_bottom - s["arm_plane_bottom_from_bottom"]
            self.pod_floor_top_z = self.servo_bottom_z
            self.pod_z0 = self.pod_floor_top_z - pod["floor_t"]
            self.pod_rim_z = self.servo_bottom_z + s["ear_underside_from_bottom"]
            self.servo_axis_r = (self.pod_cavity_r_in + self.pod_cavity_r_out) / 2
            self.servo_arm_tip_r = self.servo_axis_r + pod["arm_len"]
            if self.pod_z0 < self.z_exit + 2.0:
                raise ValueError("servo pod bottom below exit plane")
            if self.pod_rim_z + 10.0 > self.z_vane_section_top:
                raise ValueError("servo pod rim too high for the vane section")

        # ---------------- fins ----------------
        self.fin_thetas = [fin["theta0_deg"] + i * 360.0 / fin["count"] for i in range(fin["count"])]
        self.fin_t = fin["t"]
        self.fin_root_r = self.vane_section_r - fin["root_overlap"]
        self.fin_tip_r = self.vane_section_r + fin["span"]
        self.fin_root_chord = fin["root_chord"]
        self.fin_tip_chord = fin["tip_chord"]
        self.fin_te_z = fin["te_z"]
        self.fin_root_le_z = self.fin_te_z + self.fin_root_chord
        self.fin_tip_le_z = self.fin_te_z + self.fin_tip_chord
        self.footprint_d = 2 * self.fin_tip_r
        if self.fin_root_le_z > self.z_vane_section_top + 1e-6:
            raise ValueError("fin root chord exceeds vane section height")
        # ---------------- v0.9 direct-drive vanes: servo in a fin-root fairing (vane_drive) ----------------
        if self.direct_drive:
            self._direct_drive(p)
        if self.vtol and not (self.direct_drive and self.tabs):
            raise ValueError("mission.config 'vtol' is laid out for the direct-drive, tab-mount (BUDGET) preset only")
        # ---------------- v0.9 CAD-2 VTOL fin lightening hole (fin.hole) ----------------
        self.has_fin_hole = bool(fin.get("hole"))
        if self.has_fin_hole:
            self._fin_hole(p)

        # ---------------- intake module ----------------
        self.z_intake_bottom = self.z_prop_spigot_bottom           # sits on cone shell top face
        self.z_intake_skirt_top = self.z_prop_spigot_top           # lip_clamp: = lip bottom
        # tabs: the bell floats; the shelf is only a back-up thrust stop lip_stop_gap above the lip front face
        self.z_lip_pocket_top = self.z_edf_lip_top + (im["lip_stop_gap"] if self.tabs else 0.0)
        self.lip_pocket_id = e["lip_od"] + im["lip_pocket_clearance_dia"]
        self.z_shelf_top = self.z_lip_pocket_top + im["shelf_t"]
        self.shelf_id = im["shelf_id"]
        self.z_window_bottom = self.z_shelf_top + im["window_margin_below"]
        self.z_window_top = self.z_window_bottom + im["window_h"]
        self.z_intake_bulkhead_bottom = self.z_window_top + im["window_margin_above"]
        self.z_intake_bulkhead_top = self.z_intake_bulkhead_bottom + im["bulkhead_t"]
        self.z_intake_spigot_bottom = self.z_intake_bulkhead_top
        self.z_intake_spigot_top = self.z_intake_spigot_bottom + self.spigot_h
        self.z_intake_top = self.z_intake_spigot_top
        self.intake_module_h = self.z_intake_top - self.z_intake_bottom
        self.strut_count = int(im["strut_count"])
        self.strut_thetas = [im["strut_theta0_deg"] + i * 360.0 / self.strut_count for i in range(self.strut_count)]
        self.strut_w = im["strut_w"]
        # v0.6.1 windowed band (SPEC 5.1): Z [z_shelf_top, z_intake_bulkhead_bottom], window_wall_t thick, thickened
        # INWARD (outer surface = body OD).  The windows are radial sectors centred on window_thetas, so each strut keeps
        # strut_w (arc at body_r) with radial side faces: strut_angle at every radius, inner face strut_w_in wide.
        mf = p["printer"]["min_feature"]
        self.window_wall_t = im["window_wall_t"]
        self.r_band_in = self.body_r - self.window_wall_t
        if self.window_wall_t < self.body_wall - 1e-9:
            raise ValueError(f"intake_module.window_wall_t {self.window_wall_t:g} < body.wall {self.body_wall:g}: the "
                             f"windowed band must not be thinner than the tube it continues")
        strut_angle = self.strut_w / self.body_r                                  # rad, the same at every radius
        window_angle = (2 * math.pi / self.strut_count) - strut_angle            # rad, per window
        if self.strut_count < 1 or window_angle <= 0.0:
            raise ValueError(f"{self.strut_count} struts of {self.strut_w:g} mm leave no window at r {self.body_r:g}")
        self.strut_angle_deg = math.degrees(strut_angle)
        self.window_angle_deg = math.degrees(window_angle)
        self.window_thetas = [t + 180.0 / self.strut_count for t in self.strut_thetas]  # window centres
        self.window_sectors = [(t - self.window_angle_deg / 2, t + self.window_angle_deg / 2) for t in self.window_thetas]
        self.window_h = im["window_h"]
        self.strut_w_in = self.strut_w * self.r_band_in / self.body_r            # strut inner face (arc at r_band_in)
        # design horizontal section of one strut: annular sector strut_angle x [r_band_in, body_r]
        self.strut_section_mm2 = 0.5 * strut_angle * (self.body_r ** 2 - self.r_band_in ** 2)
        if self.strut_w_in < mf - 1e-9:
            raise ValueError(f"intake strut inner face {self.strut_w_in:.2f} mm < printer.min_feature {mf:g}")
        # the shelf must stick out past the band's inner face to stop the EDF lip
        self.shelf_ledge = self.r_band_in - self.shelf_id / 2                    # radial shelf left inside the band
        self.shelf_ledge_margin = self.shelf_ledge - mf
        if self.shelf_ledge_margin < -1e-9:
            raise ValueError(f"intake band inner radius r_band_in {self.r_band_in:.2f} leaves the shelf (id "
                             f"{self.shelf_id:g}) only {self.shelf_ledge:.2f} mm past the band (< printer.min_feature "
                             f"{mf:g}): the shelf would not stand out to stop the EDF lip")
        # v0.6.2 window lower end (intake_module.window_bottom_end): truncated gable, see _window_gable
        self.window_half_angle_deg = 0.5 * self.window_angle_deg
        self._window_gable(im)
        # inlet area vs fan annulus area: the open area of the windows on the cylinder of radius r
        # (window_open_area_mm2: radial sides -> r x window_angle x window_h, minus the v0.6.2 gussets); the inlet area
        # that counts (C9) is its minimum over the band (sampled across it; without gussets the minimum is at r_band_in);
        # the outer-surface value and the v0.6.1 square-window value at r_band_in are reported too
        radii = [self.r_band_in + (self.body_r - self.r_band_in) * k / (INLET_AREA_SAMPLES - 1)
                 for k in range(INLET_AREA_SAMPLES)]
        areas = [(self.strut_count * self.window_open_area_mm2(r), r) for r in radii]
        self.inlet_area_mm2, self.inlet_area_r_min = min(areas)
        self.inlet_area_outer_mm2 = self.strut_count * self.window_open_area_mm2(self.body_r)
        self.inlet_area_square_mm2 = self.strut_count * window_angle * self.r_band_in * self.window_h
        hub_d = e["hub_d_ASSUMED"]
        self.fan_annulus_area_mm2 = math.pi / 4 * (e["housing_id"] ** 2 - hub_d ** 2)
        self.inlet_area_ratio = self.inlet_area_mm2 / self.fan_annulus_area_mm2
        self.inlet_area_ratio_outer = self.inlet_area_outer_mm2 / self.fan_annulus_area_mm2
        self.inlet_area_ratio_square = self.inlet_area_square_mm2 / self.fan_annulus_area_mm2
        if self.inlet_area_ratio < im["min_inlet_area_ratio"] - 1e-9:
            raise ValueError(f"inlet area ratio {self.inlet_area_ratio:.3f} (minimum opening over the band, at r "
                             f"{self.inlet_area_r_min:.2f}, gussets included) < intake_module.min_inlet_area_ratio "
                             f"{im['min_inlet_area_ratio']:g}")
        self.cable_hole_d = im["cable_hole_d"]
        if self.tabs:
            # wire channel past the lip: void outside a 1.2 mm pocket wall, closed by an external bulge;
            # below the lip the leads run in the propulsion module's wire slot, above the shelf inside the tube
            wc = im["wire_channel"]
            self.wire_ch_w = wc["w"]
            self.wire_ch_wall = wc["wall"]
            self.wire_ch_r0 = self.lip_pocket_id / 2 + wc["wall"]
            self.wire_ch_r1 = self.wire_ch_r0 + wc["depth"]
            self.wire_notch_r0 = self.shelf_id / 2 - 0.5
            self.wire_notch_z0 = self.z_lip_pocket_top
            # v0.9: the gate (shelf notch over the lip) may be narrower than the groove (wire_channel.gate_w_motor): it must
            # stay inside the 0 deg strut, else the lip-pocket wall's top edge under it hangs over the window sectors
            self.wire_gate_w = wc.get("gate_w_motor", wc["w"])
            # v0.9 (user 2026-10-27, decision 3): an OPEN groove (no outer wall, open at the skirt's bottom edge) closed by
            # the printed wire_cover, side walls side_wall thick, a roof_slope_deg wedge on top, two cover-screw pads per
            # side wall, a gate_h_motor gate over the lip; params without wire_channel.open keep the v0.5d closed channel
            self.wire_ch_open = bool(wc.get("open", False))
            if self.wire_ch_open:
                sw, mfp = wc["side_wall"], p["printer"]["min_feature"]
                self.wire_side_wall = sw
                self.wire_bulge_w = self.wire_ch_w + 2 * sw
                self.wire_bulge_r_out = self.wire_ch_r1                   # side walls end here: the cover sits on them
                self.wire_ch_gate_h = wc["gate_h_motor"]
                self.wire_gate_lo_z0 = self.z_intake_bottom                # the skirt is open over the whole groove width
                self.wire_ch_z1 = self.z_lip_pocket_top + wc["gate_h_motor"]
                self.wire_ch_z0 = self.z_intake_bottom
                # the full-width groove ends at the shelf top (under the band); only the gate (gate_w_motor, inside the
                # strut) continues up to wire_ch_z1 - a 22 mm cut through the band's outer part beside the strut would
                # leave a 0.95 mm sliver of band (r_band_in .. wire_ch_r0)
                self.wire_groove_z1 = self.z_shelf_top
                self.wire_ch_floor_rz = None
                land = wc["wall"]
                r_in = self.body_r - 0.6                                   # bulge solids start 0.6 inside the skin (SPEC 5.1)
                h = (self.wire_ch_r1 - r_in) * math.tan(math.radians(wc["roof_slope_deg"]))
                self.wire_roof_rz = [(r_in, self.wire_ch_z1), (self.wire_ch_r1, self.wire_ch_z1),
                                     (self.wire_ch_r1, self.wire_ch_z1 + land), (r_in, self.wire_ch_z1 + land + h)]
                if wc["roof_slope_deg"] < 45.0 - 1e-9:
                    raise ValueError("intake_module.wire_channel.roof_slope_deg < 45: the groove roof needs support")
                self.wire_bulge_z0 = self.z_intake_bottom
                self.wire_bulge_z1 = self.wire_ch_z1 + land + h
                cv = wc["cover"]
                self.wire_cov_t, self.wire_cov_gap = cv["t"], cv["gap"]
                self.wire_cov_d_clear, self.wire_cov_d_tap, self.wire_cov_tap_depth = cv["d_clear"], cv["d_tap"], cv["tap_depth"]
                self.wire_pad_v = (self.wire_ch_w / 2, self.wire_ch_w / 2 + cv["pad_t"])
                self.wire_pad_h = cv["pad_h"]
                self.wire_screw_v = self.wire_ch_w / 2 + cv["pad_t"] / 2
                self.wire_pad_zs = [self.z_intake_bottom + cv["pad_h"] / 2 + 1.0, self.wire_ch_z1 - cv["pad_h"] / 2 - 1.0]
                self.wire_cov_u = (self.wire_ch_r1, self.wire_ch_r1 + cv["t"])
                self.wire_cov_hv = self.wire_screw_v + cv["d_clear"] / 2 + mfp
                self.wire_cov_z = (self.z_intake_bottom, self.wire_ch_z1 + land)
                self.wire_cov_walls = {"tap hole -> groove": self.wire_screw_v - cv["d_tap"] / 2 - self.wire_ch_w / 2,
                                       "tap hole -> pad outer face": self.wire_pad_v[1] - self.wire_screw_v - cv["d_tap"] / 2,
                                       "tap end -> skin": self.wire_ch_r1 - cv["tap_depth"] - r_in,
                                       "side wall": sw, "roof land": land,
                                       "clear hole -> cover edge": self.wire_cov_hv - self.wire_screw_v - cv["d_clear"] / 2}
                for nm, wv in self.wire_cov_walls.items():
                    if wv < mfp - 1e-9:
                        raise ValueError(f"motor-lead groove: {nm} {wv:.2f} mm < printer.min_feature {mfp:g}")
                if self.wire_pad_zs[1] - self.wire_pad_zs[0] < cv["pad_h"]:
                    raise ValueError("motor-lead groove too short for two cover-screw pads")
                # the wedge / walls stay below the window openings at the bulge's angular half-width
                if self.has_window_gable:
                    phi = math.degrees(math.atan2(self.wire_bulge_w / 2 + cv["pad_t"] - sw, r_in))
                    ang = [abs(((self.wire_theta - wt + 180.0) % 360.0) - 180.0) for wt in self.window_thetas]
                    d_c = min(ang) - phi                      # angle from the nearest window centre to the bulge edge
                    zb = min(self.gable_boundary_z(r * math.sin(math.radians(d_c))) for r in (self.r_band_in, self.body_r))
                    self.wire_bulge_window_clear = zb - self.wire_bulge_z1
                else:
                    self.wire_bulge_window_clear = self.z_window_bottom - self.wire_bulge_z1
                if self.wire_bulge_window_clear < 0.0:
                    raise ValueError(f"motor-lead groove roof (top z {self.wire_bulge_z1:.2f}) reaches a window opening")
            else:
                self.wire_bulge_r_out = self.wire_ch_r1 + wc["wall"]
                self.wire_bulge_w = self.wire_ch_w + 2 * wc["wall"]
                # gates: below the lip the channel reaches gate_h down into the skirt (the bundle steps out of the
                # propulsion wire slot there); above the lip face a shelf notch (r from shelf_id/2) lets it step back in
                self.wire_ch_gate_h = wc["gate_h"]
                self.wire_gate_lo_z0 = self.z_intake_skirt_top - wc["gate_h"]   # the leads step out within [this, skirt top]
                self.wire_ch_z1 = self.z_lip_pocket_top + wc["gate_h"]
                # v0.5d: the channel floor rises at 45 deg from the skirt's inner face (r body_id/2) to the bulge's
                # outer wall, reaching wire_gate_lo_z0 at r wire_ch_r1, so the intake (printed flipped) needs no support
                # inside the closed channel; the void is cut from wire_ch_z0 and the floor wedge is unioned back
                self.wire_ch_z0 = self.wire_gate_lo_z0 - (self.wire_ch_r1 - self.body_id / 2)
                self.wire_ch_floor_rz = [(self.body_id / 2, self.wire_ch_z0), (self.wire_ch_r1 + 0.5, self.wire_ch_z0),
                                         (self.wire_ch_r1 + 0.5, self.wire_ch_z0 + (self.wire_ch_r1 + 0.5 - self.body_id / 2))]
                self.wire_bulge_z0 = self.wire_ch_z0 - wc["wall"]
                self.wire_bulge_z1 = self.wire_ch_z1 + wc["wall"]
                if self.wire_bulge_z0 < self.z_intake_bottom + 2.0:
                    raise ValueError("wire bulge runs off the bottom of the intake skirt")
                if self.wire_bulge_z1 > self.z_window_bottom:
                    raise ValueError("wire bulge reaches the windows")
            self.wire_hole_r0, self.wire_hole_r1 = wc["bulkhead_hole_r"]
            # v0.9 fix round: the bulkhead hole has its own width (20; before = the groove width)
            self.wire_hole_w = float(wc.get("bulkhead_hole_w", self.wire_ch_w))
            # v0.9 fix round: below the skirt top the open groove is cut from the skirt bore's chord at the groove edge
            # (no sliver of the skirt beside the groove); above it from wire_ch_r0 (the lip-pocket wall stays)
            self.wire_skirt_cut_r0 = min(self.wire_ch_r0, math.sqrt((self.body_id / 2) ** 2 - (self.wire_ch_w / 2) ** 2) - 0.3)
            # v0.9 fix round: slit through the lip-pocket wall at wire_theta (the bare leads move from the interior into
            # the groove while the intake is held above the EDF)
            sl_ = wc.get("lip_slit")
            self.wire_lip_slit_w = float(sl_["w"]) if (sl_ and getattr(self, "wire_ch_open", False)) else 0.0
            if self.wire_lip_slit_w and self.wire_lip_slit_w > self.wire_gate_w - 2 * p["printer"]["min_feature"] + 1e-9:
                raise ValueError("intake_module.wire_channel.lip_slit.w must stay inside the gate with min_feature each side")
            if self.wire_ch_r0 > self.wire_slot_r_out - 0.5:
                raise ValueError("intake wire channel does not overlap the propulsion wire slot radially")
            if self.wire_hole_r1 > self.spigot_id / 2 - 0.5:
                raise ValueError("bulkhead wire hole cuts the intake spigot")
            if min(abs((self.wire_theta - s + 180.0) % 360.0 - 180.0) for s in self.strut_thetas) > 1e-6:
                raise ValueError("wire channel should sit on a strut (intake_module.strut_theta0_deg)")
            # v0.6.1 band: the shelf notch + channel must still open the step-in through the thicker band (the notch
            # starts inside the band's inner face, the channel ends outside the body), and the leads run up beside the
            # strut to the bulkhead hole inside the band's inner face
            self._band_step_in_check("motor-lead", self.wire_notch_r0, self.wire_ch_r0, self.wire_ch_r1,
                                     self.wire_gate_w, self.wire_ch_z1)
            # the gate stays inside the strut's inner face (with printer.min_feature each side)
            self.wire_gate_in_strut = self.strut_w_in - self.wire_gate_w
            if self.wire_ch_open and self.wire_gate_in_strut < 2 * p["printer"]["min_feature"] - 1e-9:
                raise ValueError(f"motor-lead gate {self.wire_gate_w:g} mm is not inside the 0 deg strut's inner face "
                                 f"{self.strut_w_in:.2f} mm with min_feature each side")
            self.band_clear_wire_hole = self.r_band_in - self.wire_hole_r1
            if self.band_clear_wire_hole < 0.0:
                raise ValueError(f"bulkhead wire hole (r to {self.wire_hole_r1:g}) reaches past the intake band's inner "
                                 f"face r {self.r_band_in:.2f}: the motor leads would run up inside the band")
            # v0.9: the hole's OUTER CORNERS (|v| = w/2) are farther out than its outer face; with the 22 mm hole they
            # decide (r 56 put them 0.07 mm into the band and the slicer added support there): they keep min_feature
            self.band_clear_wire_hole_corner = self.r_band_in - math.hypot(self.wire_hole_r1, self.wire_hole_w / 2)
            if self.band_clear_wire_hole_corner < p["printer"]["min_feature"] - 1e-9:
                raise ValueError(f"bulkhead wire hole's outer corners (r {math.hypot(self.wire_hole_r1, self.wire_hole_w / 2):.2f}) "
                                 f"leave {self.band_clear_wire_hole_corner:.2f} mm to the intake band's inner face")
        self.vent_hole_d = im["vent_hole_d"]
        self.vent_hole_r = im["vent_hole_r"]
        self.vent_hole_thetas = [im.get("vent_hole_theta0_deg", 45.0) + i * 360.0 / im["vent_hole_count"]
                                 for i in range(im["vent_hole_count"])]

        # ---------------- avionics module ----------------
        self.z_avionics_bottom = self.z_intake_spigot_bottom
        self.avionics_h = am["tube_h"]
        self.z_avionics_top = self.z_avionics_bottom + self.avionics_h
        self.z_avionics_spigot_bottom = self.z_avionics_top - self.spigot_h
        self.z_avionics_spigot_top = self.z_avionics_top
        self.tray_x = am["tray_plane_x"]
        self.tray_t = am["tray_t"]
        self.tray_w = am["tray_w"]
        self.tray_h = am["tray_h"]
        self.z_tray_bottom = self.z_avionics_bottom + am["tray_z_from_module_bottom"]
        self.z_tray_top = self.z_tray_bottom + self.tray_h
        if self.z_tray_top > self.z_avionics_spigot_bottom - 2.0:
            raise ValueError("avionics tray too tall")
        self.rail_y = math.sqrt((self.body_id / 2) ** 2 - self.tray_x ** 2)   # rails on the wall at y = +-rail_y
        self.rail_gap = am["rail_gap"]
        self.rail_w = am["rail_w"]
        self.rail_depth = am["rail_depth"]
        self.z_rail_bottom = self.z_tray_bottom - 2.0
        # v0.5d: tray stop - each rail slot is closed by a block from z_tray_stop_bottom to z_tray_bottom, so the tray
        # rests at its design height instead of sliding down onto the intake bulkhead
        self.tray_stop_h = am.get("tray_stop_h")
        self.z_tray_stop_bottom = self.z_tray_bottom - self.tray_stop_h if self.tray_stop_h else None
        if self.tray_stop_h and self.z_tray_stop_bottom < self.z_rail_bottom - 1e-9:
            raise ValueError("tray stop taller than the rail run-in below the tray")
        self.z_rail_top = self.z_avionics_spigot_bottom - 2.0
        self.vent_slot_thetas = list(am["vent_slot_thetas_deg"])
        self.vent_slot_w = am["vent_slot_w"]
        self.vent_slot_h = am["vent_slot_h"]
        self.z_vent_slot_bottom = self.z_avionics_bottom + am["vent_slot_z_from_module_bottom"]
        # placeholders (battery on -x side of tray, ESC / NAV on +x side)
        # v0.9 CAD-2: the battery / ESC offsets over the tray bottom (10 mm before) and the misc box side are params
        # (avionics_module.battery_z_from_tray_bottom / esc_z_from_tray_bottom / misc_side / misc_gap_above_esc; the VTOL
        # bay packs them at the tray bottom); the defaults keep the rocket numbers
        bat_dz = float(am.get("battery_z_from_tray_bottom", 10.0))
        esc_dz = float(am.get("esc_z_from_tray_bottom", 10.0))
        bl, bw, bh = p["battery"]["dims_lwh"]
        self.battery_x = (self.tray_x - self.tray_t / 2 - bh, self.tray_x - self.tray_t / 2)
        self.battery_y = (-bw / 2, bw / 2)
        self.battery_z = (self.z_tray_bottom + bat_dz, self.z_tray_bottom + bat_dz + bl)
        el, ew, eh = p["electronics"]["esc"]["dims_lwh"]
        self.esc_x = (self.tray_x + self.tray_t / 2, self.tray_x + self.tray_t / 2 + eh)
        self.esc_y = (-ew / 2, ew / 2)
        self.esc_z = (self.z_tray_bottom + esc_dz, self.z_tray_bottom + esc_dz + el)
        # v0.6.3: the NAV board (electronics.nav, sensor computer) takes the v0.5 flight-computer spot on the tray's +x
        # face above the ESC - same box, same tray holes.  The tray-hole attributes keep the names of the params keys they
        # mirror (avionics_module.fc_hole_pattern / fc_hole_d, unchanged in params v0.6.3) because they are printed tray
        # geometry; the board box itself is nav_x / nav_y / nav_z.  v0.9 CAD-2: no NAV board in the VTOL configuration
        # (electronics.nav null: the carrier PCB has it) and no tray holes for it (avionics_module.nav_holes false).
        self.has_nav = bool(p["electronics"].get("nav"))
        self.tray_nav_holes = bool(am.get("nav_holes", True)) and self.has_nav
        if self.has_nav:
            nl, nw, nh = p["electronics"]["nav"]["dims_lwh"]
            self.nav_x = (self.tray_x + self.tray_t / 2, self.tray_x + self.tray_t / 2 + nh)
            self.nav_y = (-nw / 2, nw / 2)
            self.nav_z = (self.esc_z[1] + 10.0, self.esc_z[1] + 10.0 + nl)
            if self.nav_z[1] > self.z_tray_top:
                raise ValueError("NAV placeholder above tray top")
            self.nav_center_z = (self.nav_z[0] + self.nav_z[1]) / 2
        self.fc_hole_pattern = am["fc_hole_pattern"]
        self.fc_hole_d = am["fc_hole_d"]
        # misc electronics (receiver, logger, UBECs, ...): box on the tray's -x face above the battery
        ml, mw, mh = p["electronics"]["misc"].get("dims_lwh", [40.0, 40.0, 20.0])
        self.misc_side = am.get("misc_side", "-x")
        if self.misc_side not in ("-x", "+x"):
            raise ValueError("avionics_module.misc_side must be '-x' or '+x'")
        if self.misc_side == "+x":
            # v0.9 CAD-2 VTOL: on the tray's +x face above the ESC (the battery alone on the -x face, at the tray bottom)
            gap = float(am.get("misc_gap_above_esc", 8.0))
            self.misc_x = (self.tray_x + self.tray_t / 2, self.tray_x + self.tray_t / 2 + mh)
            self.misc_y = (-mw / 2, mw / 2)
            self.misc_z = (self.esc_z[1] + gap, self.esc_z[1] + gap + ml)
            if self.has_nav and self.misc_z[1] > self.nav_z[0] - 2.0:
                raise ValueError("misc box above the ESC runs into the NAV board")
        else:
            self.misc_x = (self.tray_x - self.tray_t / 2 - mh, self.tray_x - self.tray_t / 2)
            self.misc_y = (-mw / 2, mw / 2)
            self.misc_z = (self.battery_z[1] + 8.0, self.battery_z[1] + 8.0 + ml)
            if am.get("battery_above_misc", False):
                # heavy battery on top (raises the CG for static margin), misc electronics below it
                self.misc_z = (self.z_tray_bottom + 10.0, self.z_tray_bottom + 10.0 + ml)
                self.battery_z = (self.misc_z[1] + 8.0, self.misc_z[1] + 8.0 + bl)
        if self.battery_z[1] > self.z_tray_top:
            raise ValueError("battery placeholder above tray top")
        if self.esc_z[1] > self.z_tray_top:
            raise ValueError("ESC placeholder above tray top")
        if self.misc_z[1] > self.z_tray_top:
            raise ValueError("misc electronics placeholder above tray top")
        # v0.9 CAD-2: the hybrid buffer pack sits in the battery slot (bottom at battery_z[0]); no pack for 'tether'
        if self.vtol and self.power_source == "hybrid":
            hb = p["power"]["hybrid_extra_ASSUMED"]["buffer_pack"]
            ql, qw, qh = hb["dims_lwh"]
            self.buffer_x = (self.tray_x - self.tray_t / 2 - qh, self.tray_x - self.tray_t / 2)
            self.buffer_y = (-qw / 2, qw / 2)
            self.buffer_z = (self.battery_z[0], self.battery_z[0] + ql)
        # v0.9 fix round: the tether kit's boxes on the tray's -x side (power.tether_kit_ASSUMED.boxes[source]): x_gap from
        # the tray's -x face to the box's +x face, z0 from the tray bottom; dims_xyz; masses from the kit / hybrid keys
        self.tether_boxes: dict[str, tuple] = {}
        if self.vtol and self.power_source in ("tether", "hybrid"):
            kit = p["power"]["tether_kit_ASSUMED"]
            bx_ = (kit.get("boxes") or {}).get(self.power_source) or {}
            masses = {**kit, **p["power"].get("hybrid_extra_ASSUMED", {})}
            xf = self.tray_x - self.tray_t / 2
            for nm, b in bx_.items():
                dx, dy, dz = b["dims_xyz"]
                x1 = xf - b["x_gap"]
                z0 = self.z_tray_bottom + b["z0_from_tray_bottom"]
                self.tether_boxes[nm] = ((x1 - dx, x1), (-dy / 2, dy / 2), (z0, z0 + dz), float(masses[b["mass_key"]]))
            names = list(self.tether_boxes)
            others = {"esc": (self.esc_x, self.esc_y, self.esc_z), "misc": (self.misc_x, self.misc_y, self.misc_z),
                      "avionics_tray": ((self.tray_x - self.tray_t / 2, self.tray_x + self.tray_t / 2),
                                        (-self.tray_w / 2, self.tray_w / 2), (self.z_tray_bottom, self.z_tray_top))}
            if getattr(self, "buffer_z", None):
                others["buffer_pack"] = (self.buffer_x, self.buffer_y, self.buffer_z)
            self.tether_box_clearance: dict[str, float] = {}
            for i, a in enumerate(names):
                (ax, ay, az, _) = self.tether_boxes[a]
                for b in names[i + 1:]:
                    (bx2, by2, bz2, _) = self.tether_boxes[b]
                    self.tether_box_clearance[f"{a} -> {b}"] = _box_gap((ax, ay, az), (bx2, by2, bz2))
                for b, bb in others.items():
                    self.tether_box_clearance[f"{a} -> {b}"] = _box_gap((ax, ay, az), bb)
                rc = max(math.hypot(x, y) for x in ax for y in ay)
                self.tether_box_clearance[f"{a} -> bore (radial)"] = self.body_id / 2 - rc
            # the cap bank's centre -> the ESC's top (input) end centre (tether report: <= 50 mm of lead to the pads)
            if "tether_cap_bank" in self.tether_boxes:
                (cx_, cy_, cz_, _) = self.tether_boxes["tether_cap_bank"]
                self.tether_cap_to_esc_mm = math.dist((0.5 * (cx_[0] + cx_[1]), 0.0, 0.5 * (cz_[0] + cz_[1])),
                                                      (0.5 * (self.esc_x[0] + self.esc_x[1]), 0.0, self.esc_z[1]))
                if self.tether_cap_to_esc_mm > 50.0:
                    raise ValueError(f"tether cap bank {self.tether_cap_to_esc_mm:.1f} mm from the ESC's input end (> 50)")
            for k_, gv in self.tether_box_clearance.items():
                lim_ = 1.0 if k_.endswith("(radial)") else 0.5 if "avionics_tray" in k_ else 1.0
                if "avionics_tray" in k_ or "buffer_pack" in k_:
                    lim_ = 0.0       # strapped against the tray face / the buffer pack
                if gv < lim_ - 1e-9:
                    raise ValueError(f"tether kit box: {k_} {gv:.2f} mm < {lim_:g}")
        # check placeholders stay inside the tube bore
        for name, (x0, x1), (y0, y1) in (("battery", self.battery_x, self.battery_y),
                                         ("esc", self.esc_x, self.esc_y),
                                         *((("nav", self.nav_x, self.nav_y),) if self.has_nav else ()),
                                         ("misc", self.misc_x, self.misc_y)):
            for x in (x0, x1):
                for y in (y0, y1):
                    if math.hypot(x, y) > self.body_id / 2 - 1.0:
                        raise ValueError(f"{name} placeholder corner ({x:.1f},{y:.1f}) outside tube bore")
        # ---------------- v0.9 CAD-2 VTOL: avionics-tube windows and tray lightening holes ----------------
        self.has_av_windows = bool(am.get("windows"))
        if self.has_av_windows:
            self._avionics_windows(p)
        # ---------------- livery (user decisions 2026-11-04): white body + orange YUCHAN inlay ----------------
        self._livery(p)
        self.has_tray_holes = bool(am.get("tray_holes"))
        if self.has_tray_holes:
            th_ = am["tray_holes"]
            self.tray_hole_y = (float(th_["y_inner"]), float(th_["y_outer"]))
            self.tray_hole_z = (self.z_tray_bottom + th_["z_margin"], self.z_tray_top - th_["z_margin"])
            slot_y = min(self.tray_w / 2 - 15.0 - am["strap_slot_h"] / 2, self.tray_w / 2 - 10.0 - am["strap_slot_h"] / 2)
            self.tray_hole_walls = {"centre spine": 2 * self.tray_hole_y[0], "hole -> strap slot": slot_y - self.tray_hole_y[1],
                                    "end margin": float(th_["z_margin"])}
            mf_ = p["printer"]["min_feature"]
            for nm, wv in self.tray_hole_walls.items():
                if wv < mf_ - 1e-9:
                    raise ValueError(f"tray lightening hole: {nm} {wv:.2f} mm < printer.min_feature {mf_:g}")
            if self.tray_hole_z[1] - self.tray_hole_z[0] < 2 * mf_:
                raise ValueError("tray too short for the lightening holes")

        # ---------------- v0.8.1 GNS board (electronics.gns, avionics_module.gns_mount) ----------------
        # Fourth ATmega128A (GPS + position hold) on four bosses on the -x tube wall outboard of the battery, under the GPS
        # lead's entry hole.  Placed here (before the FCU / COM / radio box, whose clearance tables then include it through
        # avionics_bay_boxes); its leads are routed after the GPS pod (``_gns_leads``).
        self.has_gns = bool(p["electronics"].get("gns")) and bool(am.get("gns_mount"))
        self.gns_clearance: dict[str, float] = {}
        if self.has_gns:
            self._gns(p)

        # ---------------- v0.6.3 FCU board (electronics.fcu) ----------------
        # Flat on the intake bulkhead's top face on foam_t foam tape, centred, edges along x / y (dims_lwh = x length,
        # y width, z height): Z [z_intake_bulkhead_top + foam_t, + height], inside the intake spigot bore below the tray.
        # The box is placed here (the servo-lead route below ends at it); its clearances are derived and raised on after
        # the route (``_fcu``), because the bulkhead servo hole is part of the route.
        # v0.9 CAD-2: in the VTOL configuration the one avionics carrier PCB (electronics.carrier) takes this spot and every
        # FCU clearance rule; its report part is "carrier" (fcu_part / fcu_mass_g name the part for every consumer)
        self.fcu_key = "carrier" if self.vtol else "fcu"
        self.fcu_part = self.fcu_key
        self.has_fcu = bool(p["electronics"].get(self.fcu_key))
        self.fcu_clearance: dict[str, float] = {}
        if self.has_fcu:
            fe = p["electronics"][self.fcu_key]
            self.fcu_mass_g = float(fe["mass_g"])
            ul, uw, uh = fe["dims_lwh"]
            self.fcu_foam_t = float(fe["foam_t"])
            if self.fcu_foam_t < 0.0 or min(ul, uw, uh) <= 0.0:
                raise ValueError("electronics.fcu: dims_lwh must be positive and foam_t >= 0")
            self.fcu_x = (-ul / 2, ul / 2)
            self.fcu_y = (-uw / 2, uw / 2)
            self.fcu_z = (self.z_intake_bulkhead_top + self.fcu_foam_t, self.z_intake_bulkhead_top + self.fcu_foam_t + uh)
            self.fcu_center = (0.0, 0.0, 0.5 * (self.fcu_z[0] + self.fcu_z[1]))

        # ---------------- v0.6 vane-servo lead route (SPEC 3.8 / 5.1, tabs only) ----------------
        # needs the propulsion, intake and avionics stations above; KAIST (lip_clamp) keeps no route
        self.servo_leads = bool(self.tabs and p.get("servo_leads"))
        if self.servo_leads:
            self._servo_lead_route()
        if self.has_fcu:
            self._fcu(p)
        # ---------------- wire09c clock mount: keyways round the circle (needs the servo-lead columns) ----------------
        if self.clock_mount:
            self._clock_seat(p)
        # ---------------- v0.9 motor leads (3 x 12 AWG, 5.5 mm bullets; motor_leads) ----------------
        self.has_motor_leads = bool(self.tabs and p.get("motor_leads"))
        if self.has_motor_leads:
            self._motor_leads(p)

        # ---------------- payload module ----------------
        # v0.9 CAD-2: rocket configuration only - the VTOL stack ends with the top cap on the avionics module
        if not self.vtol:
            self.z_payload_bottom = self.z_avionics_spigot_bottom
            self.z_payload_bulkhead_bottom = self.z_avionics_spigot_top
            self.z_payload_bulkhead_top = self.z_payload_bulkhead_bottom + pl["bulkhead_t"]
            self.spring_seat_boss_d = pl["spring_seat_boss_d"]
            self.z_spring_bottom = self.z_payload_bulkhead_top
            self.z_spring_top = self.z_spring_bottom + pl["spring_compressed_len"]
            self.z_piston_bottom = self.z_spring_top
            self.z_piston_top = self.z_piston_bottom + pl["piston_t"]
            self.piston_d = self.body_id - pl["piston_clearance_dia"]
            self.z_envelope_bottom = self.z_piston_top
            self.z_envelope_top = self.z_envelope_bottom + pay["envelope_h"]
            self.envelope_d = pay["envelope_d"]
            if self.envelope_d > self.body_id - 2.0:
                raise ValueError("payload envelope too large for body ID")
            self.z_payload_spigot_bottom = self.z_envelope_top
            self.z_payload_spigot_top = self.z_payload_spigot_bottom + self.spigot_h
            self.z_payload_top = self.z_payload_spigot_top
            self.payload_module_h = self.z_payload_top - self.z_payload_bottom
            if self.spigot_id < self.envelope_d:
                # envelope never enters the spigot zone, but warn if it would
                pass
            self.can_d = pay["can_d"]
            self.can_h = pay["can_h"]
            self.z_can_bottom = self.z_envelope_bottom
            self.z_can_top = self.z_can_bottom + self.can_h
            self.z_chute_pack_top = self.z_can_top + pay["chute_pack_h"]
            # release latch
            self.release_pod_theta = pl["release_pod_theta_deg"]
            self.latch_lug_theta = pl["latch_lug_theta_deg"]
            self.z_lug_bottom = self.z_payload_spigot_bottom              # skirt bottom edge
            self.z_lug_top = self.z_lug_bottom + pl["latch_lug_h"]
            self.lug_r_in = self.body_r
            self.lug_r_out = pl["latch_lug_r_out"]
            self.lug_w = pl["latch_lug_w"]
            self.z_latch_arm_bottom = self.z_lug_top + pl["latch_arm_clearance_z"]
            self.z_latch_arm_top = self.z_latch_arm_bottom + pl["latch_arm_t"]
            self.release_servo_bottom_z = self.z_latch_arm_bottom - s["arm_plane_bottom_from_bottom"]
            self.release_pod_floor_top_z = self.release_servo_bottom_z
            self.release_pod_z0 = self.release_pod_floor_top_z - pod["floor_t"]
            self.release_pod_rim_z = self.release_servo_bottom_z + s["ear_underside_from_bottom"]
            self.release_pod_cavity_r_in = self.body_r + 1.0
            self.release_pod_cavity_r_out = self.release_pod_cavity_r_in + pod["cavity_radial"]
            self.release_pod_r_out = self.release_pod_cavity_r_out + pod["wall_radial"]
            self.release_pod_r_in = self.body_r - 1.5
            self.release_servo_axis_r = (self.release_pod_cavity_r_in + self.release_pod_cavity_r_out) / 2
            self.latch_arm_len = pl["latch_arm_len"]
            self.latch_arm_w = pl["latch_arm_w"]
            if self.release_pod_rim_z > self.z_payload_spigot_bottom - 1.0:
                raise ValueError("release pod rim collides with nose skirt")
            n_bh = int(pl["bulkhead_hole_count"])
            self.bulkhead_hole_thetas = [BULKHEAD_HOLE_THETA0_DEG + k * 360.0 / n_bh for k in range(n_bh)]
            self.bulkhead_hole_r = pl["bulkhead_hole_r"]
            self.bulkhead_hole_d = pl["bulkhead_hole_d"]

        # ---------------- v0.6 COM board (second ATmega128A: LoRa link) ----------------
        # Horizontal board under the payload bulkhead.  com_mount: four bosses Ø boss_d x boss_h on a square
        # `pattern` centred on the axis (edges along x / y) grow DOWN from the bulkhead underside; each has a blind
        # Ø hole_d hole hole_depth deep from its tip (M3 self-tapping from below through the board).  The board box
        # (electronics.com.dims_lwh = x length, y width, z height; components hang down) hangs from the boss tips
        # into the avionics spigot bore.  Only when params carry both electronics.com and payload_module.com_mount.
        self.has_com_board = bool(pl.get("com_mount")) and bool(p["electronics"].get("com"))
        self.com_clearance: dict[str, float] = {}
        if self.has_com_board:
            cmt, cel = pl["com_mount"], p["electronics"]["com"]
            self.com_boss_d, self.com_boss_h = cmt["boss_d"], cmt["boss_h"]
            self.com_hole_d, self.com_hole_depth = cmt["hole_d"], cmt["hole_depth"]
            self.com_pattern = cmt["pattern"]
            self.com_min_clear = cmt["min_clear_to_tray"]
            half = self.com_pattern / 2
            self.com_boss_xy = [(sx * half, sy * half) for sx in (1.0, -1.0) for sy in (1.0, -1.0)]
            self.z_com_boss_top = self.z_payload_bulkhead_bottom              # bosses hang from the bulkhead underside
            self.z_com_boss_tip = self.z_payload_bulkhead_bottom - self.com_boss_h
            self.z_com_hole_end = self.z_com_boss_tip + self.com_hole_depth     # flat bottom of the blind hole
            self.com_hole_skin = self.z_payload_bulkhead_top - self.z_com_hole_end   # bulkhead left above the hole
            if not 0.0 < self.com_hole_d < self.com_boss_d:
                raise ValueError("com_mount.hole_d must be > 0 and smaller than boss_d")
            if self.com_hole_depth <= 0.0 or self.com_hole_skin <= 0.0:
                raise ValueError(f"COM boss hole is not blind: hole_depth {self.com_hole_depth:g} must be < boss_h + "
                                 f"bulkhead_t = {self.com_boss_h + pl['bulkhead_t']:g} (it would open on the spring side)")
            cl, cw, ch = cel["dims_lwh"]
            self.com_board_x = (-cl / 2, cl / 2)
            self.com_board_y = (-cw / 2, cw / 2)
            self.com_board_z = (self.z_com_boss_tip - ch, self.z_com_boss_tip)
            self.com_board_center_z = (self.com_board_z[0] + self.com_board_z[1]) / 2
            if half + self.com_hole_d / 2 > min(cl, cw) / 2:
                raise ValueError(f"COM mount pattern {self.com_pattern:g} puts the holes off the {cl:g} x {cw:g} board")
            # (a) inside the bore it hangs in: the spigot bore (spigot_id) over its whole height (the tube bore below
            # the spigot is wider), board corners and bosses
            r_corner = math.hypot(max(abs(v) for v in self.com_board_x), max(abs(v) for v in self.com_board_y))
            r_boss = math.hypot(half, half) + self.com_boss_d / 2
            r_bore = min(self.spigot_id / 2, self.body_id / 2)
            self.com_clearance["spigot bore (radial)"] = r_bore - max(r_corner, r_boss)
            # (b) everything below it in the avionics module: tray top edge, placeholders, rail ribs
            com_box = (self.com_board_x, self.com_board_y, self.com_board_z)
            below = self.avionics_bay_boxes()
            for nm, bx in below.items():
                self.com_clearance[nm] = _box_gap(com_box, bx)
            # (c) bulkhead holes (Ø bulkhead_hole_d at bulkhead_hole_r, bulkhead_hole_thetas): boss edge -> hole edge,
            # and the board footprint stays off the holes' projection, so the holes are reachable from below
            holes = [(self.bulkhead_hole_r * math.cos(math.radians(t)), self.bulkhead_hole_r * math.sin(math.radians(t)))
                     for t in self.bulkhead_hole_thetas]
            self.com_clearance["bulkhead holes (boss edge, plan)"] = min(
                math.dist(b, h) - self.com_boss_d / 2 - self.bulkhead_hole_d / 2 for b in self.com_boss_xy for h in holes)
            self.com_clearance["bulkhead holes (board footprint, plan)"] = min(
                math.hypot(max(self.com_board_x[0] - hx, hx - self.com_board_x[1], 0.0),
                           max(self.com_board_y[0] - hy, hy - self.com_board_y[1], 0.0)) - self.bulkhead_hole_d / 2
                for hx, hy in holes)
            # (d) the spring-seat boss sits on the OTHER face (bulkhead top); plan gap, informational
            self.com_clearance["spring-seat boss (plan, other face)"] = (
                math.hypot(half, half) - self.com_boss_d / 2 - self.spring_seat_boss_d / 2)
            mf = p["printer"]["min_feature"]
            if self.com_clearance["spigot bore (radial)"] < PLACEHOLDER_BORE_CLEAR_MM - 1e-9:
                raise ValueError(f"COM board leaves {self.com_clearance['spigot bore (radial)']:.2f} mm to the avionics "
                                 f"spigot bore (< {PLACEHOLDER_BORE_CLEAR_MM:g})")
            for nm in below:
                if self.com_clearance[nm] < self.com_min_clear - 1e-9:
                    raise ValueError(f"COM board is {self.com_clearance[nm]:.2f} mm from {nm} "
                                     f"(< com_mount.min_clear_to_tray {self.com_min_clear:g})")
            if self.com_clearance["bulkhead holes (boss edge, plan)"] < mf - 1e-9:
                raise ValueError("COM boss too close to a payload bulkhead hole "
                                 f"({self.com_clearance['bulkhead holes (boss edge, plan)']:.2f} mm < {mf:g})")
            if self.com_clearance["bulkhead holes (board footprint, plan)"] < 0.0:
                raise ValueError("COM board covers a payload bulkhead hole in plan")

        # ---------------- v0.6.2 radio / logger box (electronics.radio_log) ----------------
        # LoRa module + antenna, OpenLog + microSD, 2nd level shifter, 2nd AMS1117 in one axis-aligned box (dims_lwh =
        # x length, y width, z height) centred at r_center on theta_deg, hanging from the payload-bulkhead underside
        # (top = z_payload_bulkhead_bottom) beside the COM board, inside the avionics spigot bore.  Every clearance is
        # derived here and raised on when short (``radio_log_clearance``).
        self.has_radio_log = bool(p["electronics"].get("radio_log"))
        self.radio_log_clearance: dict[str, float] = {}
        if self.has_radio_log:
            self._radio_log(p)

        # ---------------- v0.8 GPS pod (payload_module.gps_pod, electronics.gps) ----------------
        # External pod on the payload-module tube at theta gps_pod.theta_deg, lid just under the nose-separation edge,
        # raceway + connector pocket down the tube, entry hole in the avionics tube; every station / clearance derived
        # and raised on in _gps_pod.  Only when params carry payload_module.gps_pod and electronics.gps.
        self.has_gps_pod = bool(pl.get("gps_pod")) and bool(p["electronics"].get("gps"))
        self.gps_clearance: dict[str, float] = {}
        if self.has_gps_pod:
            self._gps_pod(p)
        if self.has_gns:      # v0.8.1: the glink lead GNS -> NAV, the GNS distances (needs the GPS lead / COM / radio box)
            self._gns_leads(p)

        # ---------------- nose ----------------
        if self.vtol:
            # v0.9 CAD-2 VTOL: top cap on the avionics spigot (GPS, tether hardpoint, gland, arming switch)
            self._top_cap(p)
            self.vehicle_h = self.z_cap_top - self.z_exit
            if self.has_logic_power:     # 2026-11-05: 1S logic pack at spot A, top-cap switch, leads (battery build)
                self._logic_power(p)
        else:
            self.z_nose_skirt_bottom = self.z_payload_spigot_bottom
            self.z_nose_skirt_top = self.z_payload_spigot_top
            self.nose_skirt_id = self.spigot_od + no["skirt_clearance_dia"]
            self.nose_len = no["length"]
            self.z_nose_base = self.z_nose_skirt_top
            self.z_nose_tip = self.z_nose_base + self.nose_len
            self.nose_wall = no["wall"]
            self.z_anchor_bar = self.z_nose_base + no["anchor_bar_z_above_skirt"]
            rec = p["recovery"]
            self.chute_pack_d = rec["chute_pack_placeholder_d"]
            self.chute_pack_h = rec["chute_pack_placeholder_h"]
            self.z_chute_pack_bottom = self.z_anchor_bar + no["anchor_bar_h"] + 6.0
            self.z_chute_pack_top = self.z_chute_pack_bottom + self.chute_pack_h
            # check the pack fits inside the inner nose profile
            for z in (self.z_chute_pack_bottom, self.z_chute_pack_top):
                r_in = self.nose_outer_radius(z - self.z_nose_base) - self.nose_wall
                if r_in < self.chute_pack_d / 2 + 1.0:
                    raise ValueError(f"chute pack does not fit in nose at z={z:.0f} (r_in={r_in:.1f})")
            # usable cavity volume above the anchor bar (numerical integration of the inner profile)
            self.nose_cavity_volume_L = self._nose_cavity_volume(self.z_anchor_bar + no["anchor_bar_h"]) / 1e6
            if self.nose_cavity_volume_L < rec["chute_pack_volume_L_required"]:
                raise ValueError(f"nose cavity {self.nose_cavity_volume_L:.2f} L < required "
                                 f"{rec['chute_pack_volume_L_required']} L")

            self.vehicle_h = self.z_nose_tip - self.z_exit

        # ---------------- launch pad ----------------
        self.pad_plate_od = lp["plate_od"]
        self.pad_plate_id = lp["plate_id"]
        self.pad_plate_t = lp["plate_t"]
        self.pad_leg_h = lp["leg_h"]
        self.pad_leg_thetas = [lp["leg_theta0_deg"] + i * 360.0 / lp["leg_count"] for i in range(lp["leg_count"])]
        if self.pad_plate_id < self.vane_section_od + 2.0:
            raise ValueError("launch pad hole too small for the vane section")
        if self.pad_plate_od > p["printer"]["bed_x"] - 2 * p["printer"]["usable_margin"]:
            raise ValueError("launch pad plate exceeds bed")

        # ---------------- printed part list with print heights ----------------
        self.printed_parts = {
            # name: (print height along Z when printed upright, max XY extent)
            "propulsion_module": (self.prop_module_h, self.footprint_d),
            "intake_module": (self.intake_module_h, self.intake_xy_extent()),   # tube + wire / servo bulges
            "avionics_module": (self.avionics_h, self.body_od),
            "avionics_tray": (self.tray_h, self.tray_w),
        }
        if getattr(self, "has_livery", False):   # livery 2026-11-04: the orange inlay, printed WITH the avionics module
            self.printed_parts[self.livery_inlay_part] = (self.livery_z_print[1], self.body_od)
        if self.vtol:      # v0.9 CAD-2: the top cap prints top face down (skirt / ribs / hub up)
            self.printed_parts["top_cap"] = (self.cap_print_h, self.body_od)
        else:
            self.printed_parts.update({
                # v0.8: the GPS pod sits opposite the release pod, so the span is release pod + GPS pod corner radius
                "payload_module": (self.payload_module_h, (self.release_pod_r_out + self.gps_r_corner) if self.has_gps_pod
                                   else 2 * self.release_pod_r_out),
                "piston": (p["payload_module"]["piston_t"], self.piston_d),
                "nose_cone": (self.nose_len + no["skirt_h"], self.body_od),
            })
        self.printed_parts["vane"] = (self.vane_chord, self.vane_shaft_r_out - self.vane_r_in)
        if not self.vtol:   # v0.9 CAD-2: the VTOL vehicle stands / lands on its fins (no latch, no launch pad)
            self.printed_parts["latch_arm"] = (pl["latch_arm_t"], pl["latch_arm_len"])
            self.printed_parts["launch_pad"] = (self.pad_leg_h + self.pad_plate_t, self.pad_plate_od)
        if self.direct_drive:     # v0.9: no horns; the coupler (prints upright, flange down), the servo cover (flat)
            self.printed_parts["vane_coupler"] = (self.dd_u_flange1 - self.dd_u_hub0, self.dd_flange_d)
            self.printed_parts["servo_cover"] = (self.cov_t, max(self.cov_u[1] - self.cov_u[0], self.cov_z[1] - self.cov_z[0]))
        else:
            self.printed_parts["vane_horn"] = (v["horn_arm_len"] + v["horn_tail_len"], v["horn_w"])
        if self.tabs and getattr(self, "wire_ch_open", False):   # v0.9: the motor-lead groove cover prints flat
            self.printed_parts["wire_cover"] = (self.wire_cov_t, max(2 * self.wire_cov_hv,
                                                                     self.wire_cov_z[1] - self.wire_cov_z[0]))
        if self.has_gps_pod:   # v0.8: the pod lid (with its three posts) prints flat, top face on the bed
            self.printed_parts["gps_pod_lid"] = (self.gps_lid_print_h, self.gps_lid_xy_extent)
        bed = p["printer"]["bed_z"] - p["printer"]["usable_margin"]
        bedxy = p["printer"]["bed_x"] - 2 * p["printer"]["usable_margin"]
        for name, (h, xy) in self.printed_parts.items():
            if h > bed:
                raise ValueError(f"{name} print height {h:.1f} exceeds bed Z {bed}")
            if xy > bedxy:
                raise ValueError(f"{name} footprint {xy:.1f} exceeds bed XY {bedxy}")

    # ------------------------------------------------------------------ #
    def nose_outer_radius(self, x: float) -> float:
        """Outer radius of the nose at distance ``x`` from the base (0..nose_len).

        Haack series: r = R/sqrt(pi) * sqrt(theta - sin(2 theta)/2 + C sin^3 theta),
        theta = acos(1 - 2 x'/L) where x' is measured from the TIP.
        """
        L = self.nose_len
        R = self.body_r
        c = self.p["nose"]["haack_c"]
        xp = max(0.0, min(L, L - x))            # distance from tip
        th = math.acos(1.0 - 2.0 * xp / L)
        val = th - math.sin(2 * th) / 2 + c * math.sin(th) ** 3
        return R / math.sqrt(math.pi) * math.sqrt(max(val, 0.0))

    def cone_outer_radius(self, z: float) -> float:
        """Outer radius of the propulsion module's EDF section at height z (vane section top .. cone top).

        45 deg chamfer from (vane_section_r, z_vane_section_top) to (cone_r_bottom, cone_z0), then the
        linear cone to (cone_r_top, cone_z1).  Below the vane-section top it returns vane_section_r."""
        if z <= self.z_vane_section_top:
            return self.vane_section_r
        if z <= self.cone_z0:
            return self.vane_section_r + (z - self.z_vane_section_top)
        if z >= self.cone_z1:
            return self.cone_r_top              # v0.5 tabs: cylindrical section cone_z1 .. z_prop_spigot_bottom
        f = (z - self.cone_z0) / (self.cone_z1 - self.cone_z0)
        return self.cone_r_bottom + (self.cone_r_top - self.cone_r_bottom) * f

    def outer_skin_profile(self) -> list[tuple[float, float]]:
        """(r, z) polyline of the EDF section's outer surface from the vane-section top to the cone shell top:
        chamfer vertex, linear cone, and (tabs) the cylinder cone_z1 .. z_prop_spigot_bottom."""
        pts = [(self.vane_section_r, self.z_vane_section_top)]
        if self.cone_chamfer_h > 1e-6:
            pts.append((self.cone_r_bottom, self.cone_z0))
        pts.append((self.cone_r_top, self.cone_z1))
        if self.z_prop_spigot_bottom > self.cone_z1 + 1e-9:
            pts.append((self.cone_r_top, self.z_prop_spigot_bottom))
        return pts

    def outer_skin_z_at_radius(self, r: float) -> float:
        """Inverse of cone_outer_radius on the chamfer / linear cone: the Z where the EDF section's outer surface
        reaches radius r (vane_section_r < r <= cone_r_top; r = cone_r_top -> cone_z1)."""
        if not self.vane_section_r < r <= self.cone_r_top + 1e-9:
            raise ValueError(f"radius {r:.2f} is not on the EDF section's outer skin")
        if r <= self.cone_r_bottom:
            return self.z_vane_section_top + (r - self.vane_section_r)        # 45 deg chamfer
        return self.cone_z0 + (r - self.cone_r_bottom) / (self.cone_r_top - self.cone_r_bottom) * (self.cone_z1 - self.cone_z0)

    def fin_le_z(self, r: float) -> float:
        """Z of the fin leading edge at radius r (straight line from the root LE to the tip LE, SPEC 3.5)."""
        return self.fin_root_le_z - (r - self.fin_root_r) * (self.fin_root_le_z - self.fin_tip_le_z) / (
            self.fin_tip_r - self.fin_root_r)

    def fin_xyz(self, k: int, u: float, v: float, z: float) -> tuple[float, float, float]:
        """Vehicle coordinates of a point in the frame of fin / vane k (u radial at its theta, v = +e_theta, Z)."""
        t = math.radians(self.fin_thetas[k])
        return (u * math.cos(t) - v * math.sin(t), u * math.sin(t) + v * math.cos(t), z)

    def _direct_drive(self, p: dict) -> None:
        """v0.9 (user 2026-10-27, decision 1) direct-drive vanes, ``vane_drive.mode`` = 'direct'.  In the frame of each fin
        (u radial along the vane hinge axis, v = +e_theta, Z): the MG90S output axis lies ON the hinge axis (u, Z =
        vane.hinge_z); radially outward from the hinge boss: boss_gap, the printed coupler hub (D-bore on the vane shaft's
        D-flat) and flange, the stock horn trimmed to a disc, the spline (servo.arm_plane_bottom_from_bottom -
        case_top_from_bottom), the case (case_top_from_bottom) with its ears (ear_underside_from_bottom, ear_t); the case
        length runs along Z, its centre servo_body_side x shaft_offset_along_l from the hinge axis.  The fin-root fairing
        (``fair_*``) wraps a pocket (``dd_pocket_*``: case + float, coupler zone, a floor slot for the lower ear tab, a
        connector bay over the case), open on the service side (+v) and closed by the servo_cover; a 45 deg keel joins its
        floor to the fin plate.  Raises when anything does not fit: the coupler / horn in the pocket, the fairing under the
        fin leading edge and over the exit plane, the cover screws' tap material, the shaft engagement."""
        vd, s, v, mf = p["vane_drive"], p["servo"], p["vane"], p["printer"]["min_feature"]
        fa, co, cp, hn = vd["fairing"], vd["coupler"], vd["cover"], vd["horn"]
        if len(self.fin_thetas) != len(self.vane_thetas) or any(
                abs(((f - t + 180.0) % 360.0) - 180.0) > 1e-9 for f, t in zip(self.fin_thetas, self.vane_thetas)):
            raise ValueError("vane_drive.mode 'direct' needs one fin on every vane angle (fin.theta0_deg / count = vane's)")
        if vd["servo_body_side"] != 1:
            raise ValueError("vane_drive.servo_body_side: only +1 is laid out (the lead leaves the case's upper end, the "
                             "connector bay sits over the case)")
        if fa["service_side"] not in (1, -1):
            raise ValueError("vane_drive.fairing.service_side must be +1 or -1")
        self.dd_service_side = float(fa["service_side"])
        self.dd_float, self.dd_tol = vd["servo_float"], vd["print_tolerance_ASSUMED"]
        if self.dd_float < self.dd_tol - 1e-9:
            raise ValueError(f"vane_drive.servo_float {self.dd_float:g} < print_tolerance_ASSUMED {self.dd_tol:g}: a pocket "
                             f"printed off the hinge axis would side-load the coupler")
        f = self.dd_float
        # (1) radial stack on the hinge axis.  v0.9 fix round: the coupler = hub (D-bore + set screw) + a SOCKET flange
        # for the stock double-arm horn (vane_drive.coupler.socket): hub0..hub1, then horn_axial_gap to the horn arms' vane-
        # side face (dd_u_arm0), the arms (arm_t) to the arm plane (dd_u_horn1), the spline collar beyond it; the socket
        # (flange) runs from hub1 to over_arm_plane past the arm plane.  The case stays where CAD-1 put it.
        sk, ssc = co["socket"], co["set_screw"]
        self.dd_u_hub0 = self.hinge_boss_r_out + vd["boss_gap"]
        self.dd_u_hub1 = self.dd_u_hub0 + co["hub_len"]
        self.dd_horn_arm_len, self.dd_horn_arm_w, self.dd_horn_arm_t = hn["arm_len"], hn["arm_w_ASSUMED"], hn["arm_t_ASSUMED"]
        self.dd_horn_collar_d, self.dd_horn_collar_h = hn["collar_d_ASSUMED"], hn["collar_h_ASSUMED"]
        self.dd_horn_screw_d, self.dd_horn_screw_h = hn["screw_head_d_ASSUMED"], hn["screw_head_h_ASSUMED"]
        self.dd_horn_axial_gap = co["horn_axial_gap"]
        self.dd_u_arm0 = self.dd_u_hub1 + self.dd_horn_axial_gap
        self.dd_u_horn1 = self.dd_u_arm0 + self.dd_horn_arm_t                 # the horn's arm plane (servo side)
        self.dd_u_collar1 = self.dd_u_horn1 + self.dd_horn_collar_h
        self.dd_u_screw0 = self.dd_u_arm0 - self.dd_horn_screw_h               # horn screw head on the arm face
        self.dd_u_flange1 = self.dd_u_horn1 + sk["over_arm_plane"]             # socket end (servo side)
        self.dd_slot_hw = 0.5 * (self.dd_horn_arm_w + sk["slot_fit"])          # socket slot half width (across the arms)
        self.dd_slot_fit = sk["slot_fit"]
        self.dd_collar_relief_d = self.dd_horn_collar_d + sk["collar_relief_dia"]
        self.dd_spline_len = s["arm_plane_bottom_from_bottom"] - s["case_top_from_bottom"]
        self.dd_u_case_top = self.dd_u_horn1 + self.dd_spline_len
        self.dd_u_case_bot = self.dd_u_case_top + s["case_top_from_bottom"]
        self.dd_u_ear = (self.dd_u_case_bot - s["ear_underside_from_bottom"] - s["ear_t"],
                         self.dd_u_case_bot - s["ear_underside_from_bottom"])
        self.dd_hub_d, self.dd_flange_d = co["hub_d"], co["flange_d"]
        self.dd_horn_d = self.dd_horn_arm_len                                  # swept diameter of the cut arms
        self.dd_bore_d = self.vane_shaft_d + co["bore_clearance_dia"]
        self.dd_bore_flat = (self.vane_shaft_d / 2 - v["d_flat_depth"]) + co["bore_clearance_dia"] / 2
        # v0.9 fix round: the flat faces the service side (+v) at neutral (vane_drive.flat_side), so the set screw is
        # driven through the open pocket; 'Z-' keeps the CAD-1 orientation (flat on the -Z side)
        self.dd_flat_side = vd.get("flat_side", "-z")
        if self.dd_flat_side not in ("+v", "-z"):
            raise ValueError("vane_drive.flat_side must be '+v' or '-z'")
        # the vane shaft ends inside the hub, its D-flat starts just before it (replace vane.shaft_r_out / d_flat_r_in)
        self.vane_shaft_r_out = self.dd_u_hub1 - vd["shaft_end_gap"]
        self.vane_d_flat_r_in = self.dd_u_hub0 - vd["d_flat_lead_in"]
        self.dd_engagement = self.vane_shaft_r_out - self.dd_u_hub0
        if self.dd_engagement < 0.5 * co["hub_len"] - 1e-9:
            raise ValueError(f"vane shaft engages the coupler over {self.dd_engagement:.2f} mm < half the hub")
        if self.dd_bore_d >= self.dd_hub_d - 2 * mf:
            raise ValueError("coupler hub wall around the D-bore < printer.min_feature")
        # the bore's flat ends 0.3 mm past the shaft end; from there the bore is round (the horn screw head may enter it
        # when the vane + coupler sit at their outward stop)
        self.dd_u_flat_end = self.vane_shaft_r_out + 0.3
        if self.dd_horn_arm_len > self.dd_flange_d + 1e-9:
            raise ValueError("vane_drive.horn.arm_len longer than the coupler socket diameter")
        if self.dd_slot_hw >= self.dd_flange_d / 2 - mf:
            raise ValueError("coupler socket slot leaves no jaw (flange_d vs horn arm_w)")
        # v0.9 fix round: the jaw (beside the slot, inside the D flange_d disc) where the arm tips bear on it
        self.dd_jaw_at_tip = math.sqrt(max((self.dd_flange_d / 2) ** 2 - (self.dd_horn_arm_len / 2) ** 2, 0.0)) - self.dd_slot_hw
        self.dd_arm_contact = self.dd_horn_arm_len / 2 - self.dd_collar_relief_d / 2      # arm flank length on the jaw
        if self.dd_jaw_at_tip < mf - 1e-9:
            raise ValueError(f"coupler socket jaw {self.dd_jaw_at_tip:.2f} mm thick at the horn arm tips < printer.min_feature "
                             f"(shorten vane_drive.horn.arm_len)")
        if self.dd_arm_contact <= 0.0:
            raise ValueError("horn arms end inside the socket's hub relief (no torque flank)")
        if self.dd_u_collar1 > self.dd_u_case_top + 1e-9:
            raise ValueError("horn collar longer than the spline (servo.arm_plane_bottom_from_bottom - case_top_from_bottom)")
        if self.dd_u_flange1 > self.dd_u_case_top - f - mf:
            raise ValueError("coupler socket runs into the servo case")
        # set screw: radial along -v from the +v side, at mid shaft engagement, onto the flat
        self.dd_set_screw_u = 0.5 * (self.dd_u_hub0 + self.vane_shaft_r_out)
        self.dd_set_screw_d_tap = ssc["d_tap"]
        self.dd_set_screw_key = (ssc["key_d"], ssc["key_len"])
        self.dd_set_screw_wall = self.dd_hub_d / 2 - self.dd_bore_flat         # thread length in the hub wall
        if self.dd_set_screw_wall < 2 * mf - 1e-9:
            raise ValueError("coupler hub wall at the flat too thin for the set screw")
        # axial retention (set screw): inward stop = hub on the boss face; the horn screw head / socket floor stay clear
        # of the shaft end / arm face by more than the vane's outward free travel (measured in C28)
        # (the hub meets the pocket's inner wall servo_float inside its design position - the boss face lies inside that
        # wall - or the boss face when the pocket starts outside it)
        self.dd_inward_stop = self.dd_u_hub0 - max(self.hinge_boss_r_out, self.dd_u_hub0 - f)
        self.dd_shaft_to_screw = self.dd_u_screw0 - self.vane_shaft_r_out     # shaft end -> horn screw head (axial)
        if self.dd_inward_stop > 0.5 * self.dd_engagement - 1e-9:
            raise ValueError("vane_drive.boss_gap >= half the shaft engagement: the set-screw retention is too loose")
        # (1b) hinge bushing (vane_drive.bushing): the bore opens to its OD; from the duct's inner wall to the boss face
        bu = vd.get("bushing")
        self.dd_bushing = bool(bu)
        if bu:
            self.hinge_bore_d = bu["od"]
            self.dd_bush_od, self.dd_bush_id = bu["od"], bu["id_ASSUMED"]
            self.dd_bush_u = (self.duct_r, self.hinge_boss_r_out)
            self.dd_bush_mass_g = (math.pi / 4 * (bu["od"] ** 2 - bu["id_ASSUMED"] ** 2) * (self.dd_bush_u[1] - self.dd_bush_u[0])
                                   * bu["density_g_cm3"] * 1e-3)
            if self.dd_bush_id < self.vane_shaft_d:
                raise ValueError("vane_drive.bushing.id_ASSUMED smaller than the vane shaft")
            if (self.hinge_boss_od - self.dd_bush_od) / 2 < 2 * mf - 1e-9:
                raise ValueError("hinge boss wall around the bushing < 2 x printer.min_feature")
            # free tilt of the shaft in the bushing (diametral clearance over the bearing length)
            self.dd_bush_tilt_deg = math.degrees(math.atan2(self.dd_bush_id - self.vane_shaft_d,
                                                            self.dd_bush_u[1] - self.dd_bush_u[0]))
        # (1c) estimated lost motion for the sim round: the horn arms in the socket slot (+ gear backlash ASSUMED)
        # the arm tips (half length arm_len/2) cross the slot's fit each way: peak to peak 2 atan(fit / arm_len)
        self.dd_deadband_slot_deg = 2 * math.degrees(math.atan2(sk["slot_fit"], self.dd_horn_arm_len))
        self.dd_gear_backlash_deg = float(vd.get("gear_backlash_deg_ASSUMED", 0.0))
        self.dd_deadband_pp_deg = self.dd_deadband_slot_deg + self.dd_gear_backlash_deg
        # (2) the case along Z: centre servo_body_side x shaft_offset_along_l above the hinge axis
        zc = self.vane_hinge_z + s["shaft_offset_along_l"]
        if s["shaft_offset_along_l"] >= s["body_l"] / 2:
            raise ValueError("servo.shaft_offset_along_l puts the output shaft off the case")
        self.dd_z_case = (zc - s["body_l"] / 2, zc + s["body_l"] / 2)
        self.dd_z_ear = (zc - s["ear_span"] / 2, zc + s["ear_span"] / 2)
        self.dd_case_w = s["body_w"]
        self.dd_boss_d = s["boss_d"]
        # (3) pocket (void, fin frame): case + float, coupler zone (same width), lower-ear-tab floor slot, connector bay
        cl, cw, ch = fa["connector_lwh_ASSUMED"]
        self.dd_conn_u = (self.dd_u_case_bot - fa["connector_from_case_bottom"] - cl,
                          self.dd_u_case_bot - fa["connector_from_case_bottom"])
        self.dd_conn_v = cw / 2
        self.dd_conn_z = (self.dd_z_ear[1] + fa["connector_gap"], self.dd_z_ear[1] + fa["connector_gap"] + ch)
        self.dd_pocket_u = (self.dd_u_hub0 - f, self.dd_u_case_bot + f)
        self.dd_pocket_hv = self.dd_case_w / 2 + f                       # half width (v)
        # the service opening ends lintel_h under the pocket ceiling (the lintel carries the ceiling in print)
        self.dd_lintel_h = fa.get("lintel_h", 0.0)
        self.dd_open_z1 = self.dd_conn_z[1] + fa["connector_gap"]
        self.dd_pocket_z = (self.dd_z_case[0] - f, self.dd_open_z1 + self.dd_lintel_h)
        self.dd_slot_u = (self.dd_u_ear[0] - f, self.dd_u_ear[1] + f)
        self.dd_slot_z = (self.dd_z_ear[0] - f, self.dd_pocket_z[0])
        if self.dd_conn_u[0] < self.dd_pocket_u[0] or self.dd_conn_v > self.dd_pocket_hv:
            raise ValueError("servo lead connector bay does not fit over the case in the pocket")
        rr = max(self.dd_flange_d, self.dd_horn_d) / 2 + f                 # coupler / horn with float
        if rr > self.dd_pocket_hv + 1e-9 or self.vane_hinge_z - rr < self.dd_pocket_z[0] - 1e-9 or \
                self.vane_hinge_z + rr > self.dd_pocket_z[1] + 1e-9:
            raise ValueError("coupler flange / horn disc (with servo_float) does not fit in the pocket")
        if self.dd_pocket_u[0] < self.hinge_boss_r_out - 1e-9:
            raise ValueError("servo pocket reaches into the hinge boss")
        # (4) fairing (solid, fin frame) + keel
        self.fair_wall, self.fair_floor_t, self.fair_top_t = fa["wall"], fa["floor_t"], fa["top_t"]
        self.fair_u = (self.fin_root_r, self.dd_pocket_u[1] + fa["outer_block"])
        self.fair_hv = self.dd_pocket_hv + fa["wall"]
        self.fair_z = (self.dd_slot_z[0] - fa["floor_t"], self.dd_pocket_z[1] + fa["top_t"])
        kd = (self.fair_hv - self.fin_t / 2) * math.tan(math.radians(fa["keel_deg"]))
        self.fair_keel_vz = [(-self.fair_hv, self.fair_z[0]), (self.fair_hv, self.fair_z[0]),
                             (self.fin_t / 2, self.fair_z[0] - kd), (-self.fin_t / 2, self.fair_z[0] - kd)]
        self.fair_keel_z0 = self.fair_z[0] - kd
        if fa["keel_deg"] < 45.0 - 1e-9:
            raise ValueError("vane_drive.fairing.keel_deg < 45: the fairing underside would need support")
        if self.fair_keel_z0 < self.z_exit + 2.0:
            raise ValueError("fairing keel reaches the exit plane")
        self.fair_le_clear = self.fin_le_z(self.fair_u[1]) - self.fair_z[1]
        if self.fair_le_clear < mf:
            raise ValueError(f"fairing top {self.fair_z[1]:.2f} is not under the fin leading edge at its outer face "
                             f"(LE z {self.fin_le_z(self.fair_u[1]):.2f})")
        if self.fair_u[1] > self.fin_tip_r - 5.0:
            raise ValueError("fairing reaches the fin tip")
        self.fair_walls = {"pocket -> duct (inner wall)": self.dd_pocket_u[0] - self.vane_section_r,
                           "side wall": fa["wall"], "floor under the ear slot": fa["floor_t"], "top": fa["top_t"],
                           "outer block": fa["outer_block"]}
        if self.dd_lintel_h:
            self.fair_walls["lintel (service-side strip over the opening)"] = min(self.dd_lintel_h, fa["wall"])
        self.dd_open_bridge = self.dd_pocket_u[1] - self.dd_pocket_u[0]      # the lintel's bridge length in print
        for nm, wv in self.fair_walls.items():
            if wv < mf - 1e-9:
                raise ValueError(f"fairing {nm} {wv:.2f} mm < printer.min_feature {mf:g}")
        # (5) servo cover on the service face + two screws along -v (into the inner wall and the outer block)
        ss = self.dd_service_side
        self.cov_t = cp["t"]
        self.cov_v = (self.fair_hv, self.fair_hv + cp["t"]) if ss > 0 else (-self.fair_hv - cp["t"], -self.fair_hv)
        self.cov_u = (self.vane_section_r, self.fair_u[1])
        self.cov_z = self.fair_z
        self.cov_d_clear, self.cov_d_tap, self.cov_tap_depth = cp["d_clear"], cp["d_tap"], cp["tap_depth"]
        self.cov_head_d, self.cov_driver_d, self.cov_driver_len = cp["head_d"], cp["driver_d"], cp["driver_len"]
        u_si = 0.5 * (self.vane_section_r + self.dd_pocket_u[0])
        u_so = self.dd_pocket_u[1] + 0.5 * fa["outer_block"]
        # the lead pipe's tunnel starts at the pocket's inner top corner (servo_leads.fin_route.pipe; without a servo-lead
        # route in params the inner screw only has to clear the pocket top)
        pipe = ((p.get("servo_leads") or {}).get("fin_route") or {}).get("pipe")
        if pipe:
            self.sl_tunnel_d, self.sl_pipe_wall, self.sl_pipe_web = pipe["tunnel_d"], pipe["wall"], pipe["web_shift"]
            self.sl_pipe_d = self.sl_tunnel_d + 2 * pipe["wall"]
            self.sl_p0_uz = (self.dd_pocket_u[0], self.dd_pocket_z[1] - self.sl_tunnel_d / 2 - fa["connector_gap"])
            z_top_free = self.sl_p0_uz[1] - self.sl_tunnel_d / 2
        else:
            z_top_free = self.dd_pocket_z[1]
        z_lo = self.vane_hinge_z + self.hinge_boss_od / 2 + mf + cp["d_tap"] / 2
        z_hi = z_top_free - mf - cp["d_tap"] / 2
        if z_hi < z_lo:
            raise ValueError("no room for the inner cover screw between the hinge boss and the lead pipe")
        self.cov_screws_uz = [(u_si, 0.5 * (z_lo + z_hi)), (u_so, 0.5 * (self.dd_pocket_z[0] + self.dd_pocket_z[1]))]
        self.cov_tap_walls = {"inner screw -> pocket": self.dd_pocket_u[0] - u_si - cp["d_tap"] / 2,
                              "inner screw -> duct": u_si - cp["d_tap"] / 2 - self.vane_section_r,
                              "outer screw -> pocket / outer face": 0.5 * (fa["outer_block"] - cp["d_tap"]),
                              "tap end -> far side": 2 * self.fair_hv - cp["tap_depth"],
                              "clear hole -> cover edge": min(self.cov_u[1] - u_so, u_si - self.cov_u[0]) - cp["d_clear"] / 2}
        for nm, wv in self.cov_tap_walls.items():
            if wv < mf - 1e-9:
                raise ValueError(f"servo cover screw: {nm} {wv:.2f} mm < printer.min_feature {mf:g}")
        # (5b) v0.9 fix round: foam pad between the case's service face and the cover (vane_drive.foam_pad); its
        # compression with the case centred in its float and pushed to the far wall
        fp = vd.get("foam_pad")
        self.dd_foam = bool(fp)
        if fp:
            gap = self.fair_hv - self.dd_case_w / 2                     # case service face -> cover inner face (centred)
            self.dd_foam_t = fp["t_free"]
            self.dd_foam_gap = (gap, gap + f)
            self.dd_foam_compression = ((fp["t_free"] - gap - f) / fp["t_free"], (fp["t_free"] - gap) / fp["t_free"])
            self.dd_foam_limits = (fp["min_compression"], fp["max_compression"])
            if self.dd_foam_compression[0] < fp["min_compression"] - 1e-9 or \
                    self.dd_foam_compression[1] > fp["max_compression"] + 1e-9:
                raise ValueError(f"vane_drive.foam_pad.t_free {fp['t_free']:g}: compression "
                                 f"{self.dd_foam_compression[0]:.2f}..{self.dd_foam_compression[1]:.2f} outside "
                                 f"{fp['min_compression']:g}..{fp['max_compression']:g} (gap {gap:.2f} + float {f:g})")
        # (6) hinge moment vs servo torque (worst case: thrust_peak, commanded limit, CP offset and friction ASSUMED)
        e, hm = p["edf"], vd["hinge_moment"]
        a_exit = math.pi / 4 * (self.duct_id ** 2 - (e["exit_hub_fraction"] * self.duct_id) ** 2) * 1e-6
        rho = p["aero"]["rho_kg_m3"]
        q = 0.5 * e["thrust_peak_N"] / a_exit                                  # 1/2 rho V^2 with rho V^2 = T / A
        n_side = q * self.vane_area_mm2 * 1e-6 * v["cl_alpha_per_rad"] * math.radians(v["max_deflection_cmd_deg"])
        m_aero = n_side * hm["cp_offset_chord_max_ASSUMED"] * self.vane_chord        # N mm
        m_fric = hm["bearing_mu_ASSUMED"] * n_side * self.vane_shaft_d / 2
        self.vane_side_load_N = n_side
        self.vane_exit_q_pa = q
        self.vane_exit_v_mps = math.sqrt(2 * q / rho)
        self.vane_hinge_moment_Nmm = m_aero + m_fric
        self.vane_hinge_moment_kgcm = (m_aero + m_fric) / 98.0665
        self.servo_torque_kgcm = s["torque_kgcm_4v8"]
        self.vane_hinge_margin = self.servo_torque_kgcm / self.vane_hinge_moment_kgcm

    def _fin_hole(self, p: dict) -> None:
        """v0.9 CAD-2 VTOL fin lightening hole (``fin.hole``): one pentagon per fin in the fin plane (u radial, z), outboard
        of the servo fairing - u from the fairing's outer face (``fair_u[1]``; the fin root without a fairing) + root_gap to
        fin_tip_r - tip_strip, z from fin_te_z + bottom_strip up the vertical sides to ``fin_hole_z1``, closed by a gable at
        gable_deg (>= 45: the fin prints upright without support) whose apex and upper corners stay le_margin under the
        leading edge.  The fin keeps a closed frame: the bottom strip (its ground-contact edge), the tip strip, the root gap
        to the fairing and the leading-edge chord.  Raises when a frame member is below printer.min_feature or the hole
        collapses."""
        h, mf = p["fin"]["hole"], p["printer"]["min_feature"]
        gd = float(h["gable_deg"])
        if gd < 45.0 - 1e-9:
            raise ValueError("fin.hole.gable_deg < 45: the hole's top would need support")
        u0 = (self.fair_u[1] if self.direct_drive else self.fin_root_r) + h["root_gap"]
        u1 = self.fin_tip_r - h["tip_strip"]
        z0 = self.fin_te_z + h["bottom_strip"]
        hw = 0.5 * (u1 - u0)
        um = 0.5 * (u0 + u1)
        gh = hw * math.tan(math.radians(gd))
        z1 = min(self.fin_le_z(um) - h["le_margin"] - gh, self.fin_le_z(u0) - h["le_margin"],
                 self.fin_le_z(u1) - h["le_margin"])
        if hw < mf or z1 - z0 < 2 * mf:
            raise ValueError(f"fin hole collapses (width {u1 - u0:.2f}, side height {z1 - z0:.2f})")
        self.fin_hole_uz = [(u0, z0), (u1, z0), (u1, z1), (um, z1 + gh), (u0, z1)]
        self.fin_hole_area_mm2 = 2 * hw * (z1 - z0) + hw * gh
        self.fin_hole_frame = {"bottom strip (ground-contact edge)": z0 - self.fin_te_z, "tip strip": self.fin_tip_r - u1,
                               "root gap to the fairing": float(h["root_gap"]),
                               "leading-edge chord over the apex": self.fin_le_z(um) - (z1 + gh),
                               "leading-edge chord over the tip corner": self.fin_le_z(u1) - z1}
        for nm, wv in self.fin_hole_frame.items():
            if wv < mf - 1e-9:
                raise ValueError(f"fin hole: {nm} {wv:.2f} mm < printer.min_feature {mf:g}")

    def _livery(self, p: dict) -> None:
        """livery (user decisions 2026-11-04): the orange YUCHAN inlay on the avionics_module (``params.livery``).

        Gated on ``livery.applies_to`` (configs and presets); ``has_livery`` False builds the plain module as before.  The
        logo polygons are design data (``livery.logo_file``, made by cad/livery/make_logo.py) in the unrolled "s_z" frame of
        the module's outer skin: s = arc length on r = body_r from the copy azimuth, z = height above the part's lower end.
        Here they are only PLACED, never redrawn:
          * ``livery_polys_sz``  [(outer ring, [hole rings])] as read (closing point dropped), s_z frame;
          * ``livery_polys_vz``  the same rings pre-warped for a RADIAL prism at each copy azimuth: v = r sin(s / r)
            (tangential coordinate in the copy's pod frame), z = vehicle z.  A straight line of constant v parallel to
            the copy's radial direction meets the cylinder r = body_r at theta = theta_c + s / r exactly, so a (v, z)
            polygon extruded radially (rocket_geometry._prism_vz / Fusion CB.tprism) has the logo's exact outline on
            the visible outer surface;
          * ``livery_r_in`` = body_r - inlay_depth: the inlay is that prism INTERSECT the annulus r [r_in, body_r];
          * ``livery_sector`` per copy: (theta_lo, theta_hi) deg and z range of an annular sector that contains it.
        Raises when the file is missing / not the generator's frame, the depth is not 0 < depth < wall, a copy azimuth
        repeats, or a polygon leaves the part's z range.  Clearances, strokes and the union with the plain module are
        check C36's job (check_geometry), measured on the solids and on these polygons."""
        lv = p.get("livery") or {}
        app = lv.get("applies_to") or {}
        self.has_livery = bool(lv) and lv.get("part") == "avionics_module" and \
            self.config in (app.get("configs") or []) and p["meta"].get("preset") in (app.get("presets") or [])
        if not self.has_livery:
            return
        path = os.path.join(os.path.dirname(PARAMS_PATH), lv["logo_file"])
        with open(path, "rb") as f:
            raw = f.read()
        import hashlib
        self.livery_file = lv["logo_file"]
        self.livery_file_sha256 = hashlib.sha256(raw).hexdigest()
        doc = json.loads(raw.decode("utf-8"))
        if not str(doc.get("frame", "")).startswith("s_z"):
            raise ValueError(f"{lv['logo_file']}: not an s_z-frame logo file")
        self.livery_part, self.livery_inlay_part = lv["part"], lv["inlay_part"]
        self.livery_thetas = [float(t) for t in lv["copies_theta_deg"]]
        if len({round(t % 360.0, 6) for t in self.livery_thetas}) != len(self.livery_thetas):
            raise ValueError("livery.copies_theta_deg repeats an azimuth")
        self.livery_depth = float(lv["inlay_depth_mm"])
        if not 0.0 < self.livery_depth < self.body_wall:
            raise ValueError(f"livery.inlay_depth_mm {self.livery_depth:g} must lie in (0, body.wall {self.body_wall:g})")
        self.livery_r = self.body_r
        self.livery_r_in = self.body_r - self.livery_depth
        self.livery_white_behind = self.body_wall - self.livery_depth
        if lv.get("z_ref", "part_bottom") != "part_bottom":
            raise ValueError("livery.z_ref: only 'part_bottom' (the module's lower end) is implemented")
        self.livery_z0 = self.z_avionics_bottom + float(lv.get("z_offset_mm", 0.0))
        R = self.livery_r
        self.livery_polys_sz, self.livery_polys_vz = [], []
        # the CAD set (make_logo.CAD_SIMPLIFY, Hausdorff recorded in the file) when present, else the design set
        self.livery_poly_set = "polygons_s_z_cad" if doc.get("polygons_s_z_cad") else "polygons_s_z"
        for g in doc[self.livery_poly_set]:
            if g.get("type") != "Polygon":
                raise ValueError(f"{lv['logo_file']}: {g.get('type')} (only Polygon entries)")
            rings = [[(float(s), float(z)) for s, z in ring[:-1]] for ring in g["coordinates"]]
            self.livery_polys_sz.append((rings[0], rings[1:]))
            warp = [[(R * math.sin(s / R), z + self.livery_z0) for s, z in ring] for ring in rings]
            self.livery_polys_vz.append((warp[0], warp[1:]))
        def _ring_area(ring) -> float:          # shoelace (s_z frame = area on the outer skin)
            return 0.5 * abs(sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(ring, ring[1:] + ring[:1])))
        self.livery_area_mm2 = sum(_ring_area(o) - sum(_ring_area(h) for h in hs) for o, hs in self.livery_polys_sz)
        ss = [s for o, _ in self.livery_polys_sz for s, _z in o]
        zs = [z for o, _ in self.livery_polys_sz for _s, z in o]
        self.livery_s_range = (min(ss), max(ss))
        self.livery_z_print = (min(zs), max(zs))
        self.livery_z_range = (min(zs) + self.livery_z0, max(zs) + self.livery_z0)
        part_h = self.z_avionics_spigot_bottom - self.z_avionics_bottom
        if self.livery_z_print[0] <= 0.0 or self.livery_z_print[1] >= part_h:
            raise ValueError(f"livery logo print z {self.livery_z_print} leaves the module's skin (0 .. {part_h:g} mm)")
        pad_deg = 1.0                                   # sector margin around the logo (the prism trims it)
        self.livery_sector = {t: (t + math.degrees(self.livery_s_range[0] / R) - pad_deg,
                                  t + math.degrees(self.livery_s_range[1] / R) + pad_deg,
                                  self.livery_z_range[0] - 1.0, self.livery_z_range[1] + 1.0)
                              for t in self.livery_thetas}
        # radial extent of the outline prism (both builders): from 1 mm inside r_in at the logo's widest |v| to 2 mm past
        # the OD - it only has to cover the band r [r_in, body_r + OVERLAP]; the sector sets the inlay's radial faces
        vmax = max(abs(v) for o, _ in self.livery_polys_vz for v, _z in o)
        self.livery_u = (math.sqrt(self.livery_r_in ** 2 - vmax ** 2) - 1.0, self.body_r + 2.0)
        self.livery_colours = lv["colours"]
        self.livery_check = dict(lv.get("check") or {})
        self.livery_white_behind_min = float(lv.get("white_behind_min_mm", 0.0))

    def _avionics_windows(self, p: dict) -> None:
        """v0.9 CAD-2 VTOL avionics-tube windows (``avionics_module.windows``): per_side hexagons (|v| <= w/2, vertical
        sides, gable_deg pointed top and bottom - support-free when the module prints upright) at pitch_deg around each
        centre theta, cut radially through the tube wall.  Z from the intake spigot top + z_above_lower_joint (the lower joint
        overlap and its screws stay solid) to the avionics spigot bottom - z_below_spigot.  Raises when a window reaches a
        rail rib / tray stop (theta), the struts between neighbours drop below 2 x printer.min_feature at the bore, or the
        windows collapse.  ``avw_open_area_mm2`` (all windows), ``avw_strut_w`` (narrowest strut at the bore),
        ``avw_rail_clear_deg``."""
        w, mf = p["avionics_module"]["windows"], p["printer"]["min_feature"]
        gd = float(w["gable_deg"])
        if gd < 45.0 - 1e-9:
            raise ValueError("avionics_module.windows.gable_deg < 45: the window tops would need support")
        self.avw_w = float(w["w"])
        hw = self.avw_w / 2
        gh = hw * math.tan(math.radians(gd))
        n, pitch = int(w["per_side"]), float(w["pitch_deg"])
        self.avw_thetas = ([c + (k - 0.5 * (n - 1)) * pitch for c in w["center_thetas_deg"] for k in range(n)]
                           + [float(t) for t in w.get("extra_thetas_deg", [])])
        self.avw_z0 = self.z_intake_spigot_top + w["z_above_lower_joint"]
        self.avw_z1 = self.z_avionics_spigot_bottom - w["z_below_spigot"]
        if self.avw_z1 - self.avw_z0 < 2 * gh + 2 * mf:
            raise ValueError("avionics tube too short for its windows")
        z0, z1 = self.avw_z0, self.avw_z1
        self.avw_vz = [(0.0, z0), (hw, z0 + gh), (hw, z1 - gh), (0.0, z1), (-hw, z1 - gh), (-hw, z0 + gh)]
        self.avw_gable_h = gh
        r_in = self.body_id / 2
        half_in = math.degrees(math.asin(hw / r_in))          # angular half width at the bore (widest)
        self.avw_strut_w = r_in * math.radians(pitch - 2 * half_in) if n > 1 else math.inf
        if self.avw_strut_w < 2 * mf - 1e-9:
            raise ValueError(f"avionics window struts {self.avw_strut_w:.2f} mm < 2 x printer.min_feature")
        # rail ribs / tray stops: x in tray_x +- (rail_gap/2 + rail_w) at the bore -> their angular extent
        xs = (self.tray_x - self.rail_gap / 2 - self.rail_w, self.tray_x + self.rail_gap / 2 + self.rail_w)
        rib_th = []
        for sy in (1.0, -1.0):
            a = [math.degrees(math.atan2(sy * math.sqrt(r_in ** 2 - x * x), x)) for x in xs]
            rib_th.append((min(a), max(a)))
        def ang(a: float, b: float) -> float:
            return abs(((a - b + 180.0) % 360.0) - 180.0)
        clear = math.inf
        for t in self.avw_thetas:
            for lo, hi in rib_th:
                if ang(t, 0.5 * (lo + hi)) <= 0.5 * (hi - lo) + half_in:
                    raise ValueError(f"avionics window at {t:g} deg reaches the rail rib {lo:.1f}..{hi:.1f} deg")
                clear = min(clear, ang(t, 0.5 * (lo + hi)) - 0.5 * (hi - lo) - half_in)
        self.avw_rail_clear_deg = clear
        self.avw_rail_thetas = rib_th
        # overlapping neighbours (pitch smaller than the width) would merge two windows: refuse
        for i, a in enumerate(self.avw_thetas):
            for b in self.avw_thetas[i + 1:]:
                if ang(a, b) < 2 * half_in + 2 * mf:
                    raise ValueError(f"avionics windows at {a:g} / {b:g} deg overlap")
        self.avw_open_area_mm2 = len(self.avw_thetas) * 2 * hw * (z1 - z0 - gh)

    def _top_cap(self, p: dict) -> None:
        """v0.9 CAD-2 VTOL top cap (``top_cap``) on the avionics spigot, replacing the payload module and the nose: a skirt
        (tube r body_id/2..body_r over the spigot, screwed at the joint thetas like the payload joint), a plate_t plate
        (flat top), rib_count radial ribs (rib_t x rib_h, from the hub to the spigot bore - rib_bore_gap) and a hub
        (hub_d, through hole hardpoint_hole_d for the M5 eye bolt) under it.  Holes: the conductor gland (two holes),
        the GPS lead hole, the arming-switch hole.  On top: the GPS ground plane + patch and the GPS board (taped), the
        tether eye on the axis.  Stations ``z_cap_*``; clearances ``cap_clearance`` (ribs / hub / switch body over every bay
        box, holes vs ribs / hub / spigot wall, GPS parts vs each other / the eye / the plate edge / the holes) raise when
        short; ``cap_tether`` holds the tether-path distances to the intake windows.  The GPS lead (top -> carrier) and,
        for power.source 'tether' / 'hybrid', the tether kit are placed here too."""
        tc, mf = p["top_cap"], p["printer"]["min_feature"]
        mc = float(p["payload_module"]["com_mount"]["min_clear_to_tray"])
        self.cap_plate_t = float(tc["plate_t"])
        self.z_cap_skirt_bottom = self.z_avionics_spigot_bottom
        self.z_cap_plate_bottom = self.z_avionics_top
        self.z_cap_top = self.z_cap_plate_bottom + self.cap_plate_t
        self.cap_rib_n = int(tc["rib_count"])
        self.cap_rib_thetas = [tc["rib_theta0_deg"] + k * 360.0 / self.cap_rib_n for k in range(self.cap_rib_n)]
        self.cap_rib_t, self.cap_rib_h = float(tc["rib_t"]), float(tc["rib_h"])
        self.cap_hub_d = float(tc["hub_d"])
        self.cap_rib_r = (self.cap_hub_d / 2 - 1.0, self.spigot_id / 2 - tc["rib_bore_gap"])
        self.z_cap_under = self.z_cap_plate_bottom - self.cap_rib_h
        if self.z_cap_under < self.z_cap_skirt_bottom:
            raise ValueError("top_cap.rib_h reaches below the cap skirt")
        # rim ring joining the rib ends (under the plate, inside the spigot bore)
        self.cap_ring_r = (self.cap_rib_r[1] - tc["ring_w"], self.cap_rib_r[1])
        self.z_cap_ring_bottom = self.z_cap_plate_bottom - tc["ring_h"]
        z_js = self.z_cap_skirt_bottom + self.joint_screw_dz
        self.cap_ring_joint_screw_gap = self.z_cap_ring_bottom - (z_js + 0.5 * p["body"]["joint_hole_outer_d"])
        self.cap_hardpoint_d = float(tc["hardpoint_hole_d"])
        self.cap_print_h = (self.z_cap_top - min(self.z_cap_skirt_bottom, self.z_cap_under))
        if self.cap_hardpoint_d > self.cap_hub_d - 2 * mf:
            raise ValueError("top_cap hub wall around the hardpoint hole < printer.min_feature")
        # holes through the plate (x, y, d)
        g = tc["gland"]
        gt = math.radians(g["theta_deg"])
        gx, gy = g["r"] * math.cos(gt), g["r"] * math.sin(gt)
        tx, ty = -math.sin(gt), math.cos(gt)
        self.cap_holes = {f"gland {i + 1}": (gx + s * 0.5 * g["pitch"] * tx, gy + s * 0.5 * g["pitch"] * ty, float(g["hole_d"]))
                          for i, s in enumerate((-1.0, 1.0))}
        gp = tc["gps"]
        lh = gp["lead_hole"]
        self.cap_holes["GPS lead"] = (lh["r"] * math.cos(math.radians(lh["theta_deg"])),
                                      lh["r"] * math.sin(math.radians(lh["theta_deg"])), float(lh["d"]))
        sw = tc["switch_hole"]
        self.cap_holes["arming switch"] = (sw["r"] * math.cos(math.radians(sw["theta_deg"])),
                                           sw["r"] * math.sin(math.radians(sw["theta_deg"])), float(sw["d"]))
        if self.has_logic_power:
            # 2026-11-05 user decision 2 ('상단 캡에 구멍 내기'): the logic power switch (MTS-102) through the plate; every
            # hole rule below applies to it (rim ring, spigot bore, hub, ribs, the other holes, the GPS parts)
            lsw = p["logic_power"]["switch"]
            self.cap_holes["logic power switch"] = (lsw["r"] * math.cos(math.radians(lsw["theta_deg"])),
                                                    lsw["r"] * math.sin(math.radians(lsw["theta_deg"])), float(lsw["hole_d"]))
        cl: dict[str, float] = {}
        lim: dict[str, float | None] = {}
        # (1) every hole: inside the spigot bore (the plate rests on the spigot wall outside it), clear of the hub and the
        # ribs, and of the other holes
        r_bore = self.spigot_id / 2
        def seg_dist(px: float, py: float, th: float) -> float:   # point -> rib centre segment
            c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
            return _pt_seg_dist_2d((px, py), (self.cap_rib_r[0] * c, self.cap_rib_r[0] * s),
                                   (self.cap_rib_r[1] * c, self.cap_rib_r[1] * s))
        names = list(self.cap_holes)
        cl["rim ring bottom -> skirt joint screw (z)"], lim["rim ring bottom -> skirt joint screw (z)"] = (
            self.cap_ring_joint_screw_gap, mf)
        for nm in names:
            x, y, d = self.cap_holes[nm]
            cl[f"{nm} hole -> rim ring"] = self.cap_ring_r[0] - math.hypot(x, y) - d / 2
            lim[f"{nm} hole -> rim ring"] = mf
            cl[f"{nm} hole -> spigot bore"] = r_bore - math.hypot(x, y) - d / 2
            cl[f"{nm} hole -> hub"] = math.hypot(x, y) - d / 2 - self.cap_hub_d / 2
            cl[f"{nm} hole -> ribs"] = min(seg_dist(x, y, th) for th in self.cap_rib_thetas) - d / 2 - self.cap_rib_t / 2
            for k in (f"{nm} hole -> spigot bore", f"{nm} hole -> hub", f"{nm} hole -> ribs"):
                lim[k] = mf
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                (xa, ya, da), (xb, yb, db) = self.cap_holes[a], self.cap_holes[b]
                k = f"{a} hole -> {b} hole"
                cl[k], lim[k] = math.hypot(xa - xb, ya - yb) - da / 2 - db / 2, mf
        # (2) under the plate: ribs / hub bottom and the switch body over every bay box (z, plan conservative)
        bay = self.avionics_bay_boxes()
        top_in_bay = max(bx[2][1] for bx in bay.values())
        cl["ribs / hub bottom -> highest bay part"], lim["ribs / hub bottom -> highest bay part"] = self.z_cap_under - top_in_bay, mc
        sx, sy, sd = self.cap_holes["arming switch"]
        sb = 0.5 * sd + 3.0
        self.cap_switch_box = ((sx - sb, sx + sb), (sy - sb, sy + sb),
                               (self.z_cap_plate_bottom - sw["body_depth_ASSUMED"], self.z_cap_plate_bottom))
        k = "arming-switch body -> bay parts"
        cl[k], lim[k] = min(_box_gap(self.cap_switch_box, bx) for bx in bay.values()), mc
        k = "arming-switch body -> ribs"
        cl[k], lim[k] = min(seg_dist(sx, sy, th) for th in self.cap_rib_thetas) - sb * math.sqrt(2) - self.cap_rib_t / 2, mf
        # (3) GPS on top: ground plane (square, centred on the patch), board (36 x 25.8, long side tangential)
        side = float(gp["ground_plane_side"])
        pt = math.radians(gp["patch_center_theta_deg"])
        pcx, pcy = gp["patch_center_r"] * math.cos(pt), gp["patch_center_r"] * math.sin(pt)
        self.cap_gp_xy = ((pcx - side / 2, pcx + side / 2), (pcy - side / 2, pcy + side / 2))
        self.cap_gp_t, self.cap_tape_t = float(gp["ground_plane_t"]), float(gp["tape_t_ASSUMED"])
        pl_, pw_, ph_ = gp["patch_lwh"]
        bl_, bw_, bh_ = gp["board_lwh"]
        self.cap_patch_xy = ((pcx - pl_ / 2, pcx + pl_ / 2), (pcy - pw_ / 2, pcy + pw_ / 2))
        self.cap_gp_z = (self.z_cap_top + self.cap_tape_t, self.z_cap_top + self.cap_tape_t + self.cap_gp_t)
        self.cap_patch_z = (self.cap_gp_z[1], self.cap_gp_z[1] + ph_)
        bt = math.radians(gp["board_center_theta_deg"])
        bcx, bcy = gp["board_center_r"] * math.cos(bt), gp["board_center_r"] * math.sin(bt)
        # long side tangential: at theta the tangential direction is (-sin, cos); boards are placed on the axes only
        if abs(math.sin(bt)) > abs(math.cos(bt)):
            self.cap_board_xy = ((bcx - bl_ / 2, bcx + bl_ / 2), (bcy - bw_ / 2, bcy + bw_ / 2))
        else:
            self.cap_board_xy = ((bcx - bw_ / 2, bcx + bw_ / 2), (bcy - bl_ / 2, bcy + bl_ / 2))
        self.cap_board_z = (self.z_cap_top + self.cap_tape_t, self.z_cap_top + self.cap_tape_t + bh_)
        te = tc["tether"]
        r_eye = 0.5 * te["eye_od_ASSUMED"]
        def rect_r_max(xy: tuple) -> float:
            return max(math.hypot(x, y) for x in xy[0] for y in xy[1])
        def rect_pt(xy: tuple, x: float, y: float) -> float:
            return math.hypot(max(xy[0][0] - x, x - xy[0][1], 0.0), max(xy[1][0] - y, y - xy[1][1], 0.0))
        for nm, xy in (("GPS ground plane", self.cap_gp_xy), ("GPS board", self.cap_board_xy)):
            k = f"{nm} -> cap edge"
            cl[k], lim[k] = self.body_r - rect_r_max(xy), 0.0
            k = f"{nm} -> tether eye"
            cl[k], lim[k] = rect_pt(xy, 0.0, 0.0) - r_eye, 1.0
            for hn, (hx, hy, hd) in self.cap_holes.items():
                k = f"{nm} -> {hn} hole"
                cl[k], lim[k] = rect_pt(xy, hx, hy) - hd / 2, 0.0
        gpb = self.cap_gp_xy
        bxy = self.cap_board_xy
        k = "GPS ground plane -> GPS board"
        cl[k] = math.hypot(max(bxy[0][0] - gpb[0][1], gpb[0][0] - bxy[0][1], 0.0),
                           max(bxy[1][0] - gpb[1][1], gpb[1][0] - bxy[1][1], 0.0))
        lim[k] = 1.0
        self.cap_clearance, self.cap_limits = cl, lim
        for k, v in cl.items():
            if lim.get(k) is not None and v < lim[k] - 1e-9:
                raise ValueError(f"top cap: {k} {v:.2f} mm < {lim[k]:g}")
        # (4) tether path vs the intake windows (tether report keep_out_checks): the boot base (cap top, axis), the free
        # cable from the boot top (within cone_half_angle_deg of the axis, going up - every later point is farther)
        def window_dist(z: float) -> float:          # point on the axis -> nearest window opening (top edge at body_r)
            dz = max(0.0, z - self.z_window_top) if z >= self.z_window_bottom else self.z_window_bottom - z
            return math.hypot(self.body_r, dz)
        zb = self.z_cap_top + te["boot_len"]
        self.cap_tether = {"boot base (cap top) -> nearest intake window": window_dist(self.z_cap_top),
                           "free cable start (boot top) -> nearest intake window": window_dist(zb),
                           "required (free cable)": float(te["min_window_clear"]),
                           "boot top z": zb, "cone half angle deg": float(te["cone_half_angle_deg"])}
        if self.cap_tether["free cable start (boot top) -> nearest intake window"] < te["min_window_clear"] - 1e-9:
            raise ValueError("tether: the free cable starts closer than top_cap.tether.min_window_clear to an intake window")
        # sky view of the patch: the boot (when fitted) seen from the patch centre
        d_p = math.hypot(pcx, pcy)
        self.cap_gps_boot_az_deg = 2 * math.degrees(math.atan2(0.5 * te["boot_d"], d_p))
        self.cap_gps_boot_el_deg = math.degrees(math.atan2(zb - self.cap_patch_z[1], d_p - 0.5 * te["boot_d"]))
        # (5) GPS lead (top -> the carrier's GNS): board -> lead hole -> straight down -> carrier top centre
        ld = gp["lead"]
        hx, hy, _ = self.cap_holes["GPS lead"]
        z_c = self.fcu_z[1] if self.has_fcu else self.z_intake_bulkhead_top
        self.gps_lead_polyline = [(0.5 * (bxy[0][0] + bxy[0][1]), 0.5 * (bxy[1][0] + bxy[1][1]), self.z_cap_top + 1.0),
                                  (hx, hy, self.z_cap_top + 1.0), (hx, hy, z_c + 2.0), (0.0, 0.0, z_c + 2.0)]
        segs = list(zip(self.gps_lead_polyline[:-1], self.gps_lead_polyline[1:]))
        route = sum(math.dist(a, b) for a, b in segs)
        self.gps_lead_len = route + ld["slack_mm"]
        self.gps_lead_mass_g = self.gps_lead_len * 1e-3 * ld["mass_g_per_m_ASSUMED"] + ld["connector_pair_g_ASSUMED"]
        mids = [(tuple(0.5 * (a[i] + b[i]) for i in range(3)), math.dist(a, b)) for a, b in segs]
        self.gps_lead_com = tuple(sum(m[i] * w for m, w in mids) / route for i in range(3))
        self.gps_board_mass_g = float(p["electronics"]["gps"]["board_mass_g"])
        self.gps_gp_mass_g = float(gp["ground_plane_mass_g_ASSUMED"])
        # (6) v0.9 fix round: hardpoint stack (top_cap.hardpoint) - the eye bolt's collar on the cap top, its shank through
        # the plate and the hub, a load washer in a flush recess in the hub / rib bottoms, a jam nut under it.  Solids
        # (tether / hybrid builds); the clearance below is checked in every VTOL source (catch-line tests with the pack).
        hp = tc.get("hardpoint")
        self.cap_hardpoint = bool(hp)
        if hp:
            self.cap_washer_d, self.cap_washer_t = float(hp["washer_d"]), float(hp["washer_t"])
            self.cap_washer_recess = (self.cap_washer_d + 2 * hp["washer_recess_clear"], self.cap_washer_t)
            self.cap_nut_af, self.cap_nut_h = float(hp["nut_af"]), float(hp["nut_h"])
            self.cap_bolt_d, self.cap_bolt_len = float(hp["shank_d"]), float(hp["shank_len"])
            self.z_cap_washer = (self.z_cap_under, self.z_cap_under + self.cap_washer_t)
            self.z_cap_nut = (self.z_cap_under - self.cap_nut_h, self.z_cap_under)
            self.z_bolt_end = self.z_cap_top - self.cap_bolt_len
            self.cap_eye_od, self.cap_eye_h = float(te["eye_od_ASSUMED"]), float(te["eye_h_ASSUMED"])
            if self.z_bolt_end > self.z_cap_nut[0] + 1e-9:
                raise ValueError(f"top_cap.hardpoint: the {self.cap_bolt_len:g} mm shank ends {self.z_bolt_end - self.z_cap_nut[0]:.2f} "
                                 f"mm short of the jam nut's bottom (no full thread engagement)")
            if self.cap_washer_d / 2 > self.cap_ring_r[0] - mf:
                raise ValueError("top_cap.hardpoint.washer_d reaches the rim ring")
            self.cap_washer_mass_g = (math.pi / 4 * (self.cap_washer_d ** 2 - self.cap_hardpoint_d ** 2) * self.cap_washer_t
                                      * hp["washer_density_g_cm3"] * 1e-3)
            self.cap_bolt_mass_g = float(hp["bolt_mass_g_ASSUMED"])
            self.cap_nut_mass_g = float(hp["nut_mass_g_ASSUMED"])
            # lowest point of the stack and the clearance to every bay box under it (flight pack: catch-line tests)
            z_low = min(self.z_bolt_end, self.z_cap_nut[0])
            self.cap_stack_low_z = z_low
            r_st = max(self.cap_nut_af / math.sqrt(3), self.cap_bolt_d / 2)
            st_box = ((-r_st, r_st), (-r_st, r_st), (z_low, self.z_cap_under))
            self.cap_stack_clearance = {nm: _box_gap(st_box, bx) for nm, bx in bay.items()
                                        if not nm.startswith("rail")}
            self.cap_stack_min_clear = float(hp["min_clear_below"])
            for nm, gv in self.cap_stack_clearance.items():
                if gv < self.cap_stack_min_clear - 1e-9:
                    raise ValueError(f"top cap hardpoint stack (bolt end / jam nut) is {gv:.2f} mm from {nm} "
                                     f"(< top_cap.hardpoint.min_clear_below {self.cap_stack_min_clear:g})")
        # (7) tether kit (power.source 'tether' / 'hybrid'): the boxes are placeholders (Layout.tether_boxes); lumped: the
        # cable boot (hardpoint total - bolt - washer - nut), the 10 AWG pair along gland -> XT90-S -> cap bank -> ESC
        # top, the hybrid buffer lead (ASSUMED masses)
        self.tether_items: dict[str, tuple[float, tuple[float, float, float]]] = {}
        if self.power_source in ("tether", "hybrid"):
            kit = p["power"]["tether_kit_ASSUMED"]
            esc_top = (0.5 * (self.esc_x[0] + self.esc_x[1]), 0.0, self.esc_z[1])
            if hp:
                boot = kit["hardpoint_eyebolt_plate_boot_g"] - self.cap_bolt_mass_g - self.cap_washer_mass_g - self.cap_nut_mass_g
                if boot < 0.0:
                    raise ValueError("tether kit: hardpoint_eyebolt_plate_boot_g smaller than the bolt + washer + nut")
                self.tether_items["tether_boot"] = (boot, (0.0, 0.0, self.z_cap_top + self.cap_eye_h + 0.5 * te["boot_len"]))
            else:
                self.tether_items["tether_hardpoint"] = (kit["hardpoint_eyebolt_plate_boot_g"],
                                                         (0.0, 0.0, self.z_cap_top + 0.2 * te["boot_len"]))
            bxs = self.tether_boxes
            pts = [(gx, gy, self.z_cap_top)]
            if "tether_xt90" in bxs:
                (ax, ay, az, _) = bxs["tether_xt90"]
                pts += [(0.5 * (ax[0] + ax[1]), 0.0, az[1]), (0.5 * (ax[0] + ax[1]), 0.0, az[0])]
            if "tether_cap_bank" in bxs:
                (ax, ay, az, _) = bxs["tether_cap_bank"]
                pts.append((0.5 * (ax[0] + ax[1]), 0.0, 0.5 * (az[0] + az[1])))
            pts.append(esc_top)
            self.tether_lead_polyline = pts
            segs_ = list(zip(pts[:-1], pts[1:]))
            ln_ = sum(math.dist(a, b) for a, b in segs_)
            self.tether_lead_route_mm = ln_
            com_ = tuple(sum(0.5 * (a[i] + b[i]) * math.dist(a, b) for a, b in segs_) / ln_ for i in range(3))
            self.tether_items["tether_power_lead"] = (kit["power_lead_10awg_pair_g"], com_)
            if self.power_source == "hybrid":
                hx_ = p["power"]["hybrid_extra_ASSUMED"]
                bz = self.buffer_z
                self.tether_items["hybrid_buffer_lead"] = (hx_["buffer_lead_xt60_g"],
                                                           (0.5 * (self.buffer_x[0] + self.buffer_x[1]), 0.0, bz[1] + 1.0))
            if not bxs:     # params without boxes: the CAD-2 lumped stations
                esc_c = (0.5 * (self.esc_x[0] + self.esc_x[1]), 0.0, 0.5 * (self.esc_z[0] + self.esc_z[1]))
                self.tether_items["tether_power_lead"] = (kit["xt90s_plug_g"] + kit["power_lead_10awg_pair_g"],
                                                          (gx, gy, 0.5 * (self.z_cap_top + self.esc_z[1])))
                self.tether_items["tether_bus_parts"] = (kit["esc_cap_bank_g"] + kit["bus_clamp_g"], esc_c)
                if self.power_source == "hybrid":
                    hx_ = p["power"]["hybrid_extra_ASSUMED"]
                    self.tether_items["hybrid_diode_board"] = (hx_["ideal_diode_board_g"] + hx_["buffer_lead_xt60_g"],
                                                               (esc_c[0], 0.0, self.esc_z[1] + 2.0))

    def _logic_power(self, p: dict) -> None:
        """v0.9 logic-power round (user decisions 2026-11-05, ``logic_power``; VTOL battery build only): the carrier PCB rev
        2c's 1S logic pack at spot A (on the 6S pack's -x face, ``lp_pack_*``), the MTS-102 switch through the top cap
        (hole ``cap_holes['logic power switch']`` placed by _top_cap; body / nuts / lever ``lp_sw_*``), the lead routes
        switch -> J15 and pack -> J14 (``lp_*_lead_polyline``, keep-out Ø ``lp_wire_od``) and the carrier's mated plug
        boxes (``lp_plugs``, from the PCB round).  Clearances ``lp_clearance`` / ``lp_limits`` (box / segment, raised
        when short; C37-C39 re-measure them on the solids): pack vs the 6S pack (strapped against it, >= 0), the bore
        (>= PLACEHOLDER_BORE_CLEAR_MM), the carrier and every bay box (>= min_clear_to_tray), the plug boxes and the GPS
        lead (>= logic_power.clearance.min_to_leads_plugs, PCB review MF2-10); switch body / lugs vs the ribs, the rim
        ring, the arming-switch body and the hardpoint stack (>= min_feature / min_clear_to_tray) and every bay box and the
        pack (>= min_clear_to_tray); the leads vs the 6S pack / bay boxes (no overlap) and the bore (>= 1 mm) and the
        switch lead vs the pack (>= min_to_leads_plugs).  Masses (``lp_*_mass_g``, ``lp_*_com``) feed the
        pending-mass report (logic_power.in_flight_mass_model false) or the flight model (true)."""
        lp, mf = p["logic_power"], p["printer"]["min_feature"]
        mc = float(p["payload_module"]["com_mount"]["min_clear_to_tray"])
        mlp = float(lp["clearance"]["min_to_leads_plugs"])
        cl: dict[str, float] = {}
        lim: dict[str, float] = {}
        # ---- pack at spot A ----
        pk = lp["pack"]
        dx, dy, dz = pk["dims_xyz"]
        x1 = self.battery_x[0] - pk["gap_to_flight_pack"]
        z0 = self.battery_z[0] + pk["z0_from_flight_pack_bottom"]
        self.lp_pack_x, self.lp_pack_y, self.lp_pack_z = (x1 - dx, x1), (-dy / 2, dy / 2), (z0, z0 + dz)
        self.lp_pack_mass_g = float(pk["mass_g"])
        self.lp_pack_center = (0.5 * (x1 - dx + x1), 0.0, z0 + dz / 2)
        pack = (self.lp_pack_x, self.lp_pack_y, self.lp_pack_z)
        # the carrier's mated plug boxes (vehicle x / y; z above the board's top face = carrier foam top + board_t)
        self.lp_board_top_z = self.fcu_z[0] + float(p["electronics"]["carrier"]["board_t"])
        self.lp_plugs = {nm: (tuple(b["x"]), tuple(b["y"]), (self.lp_board_top_z + b["z"][0], self.lp_board_top_z + b["z"][1]))
                         for nm, b in lp["carrier_plugs"].items() if nm.startswith("J")}
        bay = self.avionics_bay_boxes()
        k = "pack -> 6S pack (strapped against its -x face)"
        # signed (an overlap is negative; _box_gap would give 0 for it): the face-to-face gap along x
        cl[k], lim[k] = bay["battery"][0][0] - self.lp_pack_x[1], 0.0
        k = "pack -> bore (radial)"
        cl[k] = self.body_id / 2 - max(math.hypot(x, y) for x in self.lp_pack_x for y in self.lp_pack_y)
        lim[k] = PLACEHOLDER_BORE_CLEAR_MM
        if self.z_intake_spigot_top > self.lp_pack_z[0]:
            raise ValueError("logic pack reaches down into the intake spigot (its bore is the tighter one)")
        for nm, bx in bay.items():
            if nm != "battery":
                cl[f"pack -> {nm}"], lim[f"pack -> {nm}"] = _box_gap(pack, bx), mc
        for nm, bx in self.lp_plugs.items():
            cl[f"pack -> {nm} plug"], lim[f"pack -> {nm} plug"] = _box_gap(pack, bx), mlp
        gl = self.gps_lead_polyline
        k = "pack -> GPS lead (centre line - OD/2)"
        cl[k] = (min(_box_segment_dist(pack, a, b) for a, b in zip(gl[:-1], gl[1:]))
                 - 0.5 * float(lp["clearance"]["gps_lead_od_ASSUMED"]))
        lim[k] = mlp
        # retention: the 6S pack's lower strap (rocket_geometry.avionics_tray slot at battery_z[0] + 25, strap_slot_w wide,
        # y +-(tray_w / 2 - 10)) wraps both packs; one tie lp_tie_z below the pack top
        rt = lp["retention"]
        sw_w = float(p["avionics_module"]["strap_slot_w"])
        zs = self.battery_z[0] + 25.0
        self.lp_strap_z = (zs - sw_w / 2, zs + sw_w / 2)
        self.lp_strap_slot_y = self.tray_w / 2 - 10.0
        self.lp_tie_z = self.lp_pack_z[1] - float(rt["tie_z_below_pack_top"])
        self.lp_strap_wraps_pack = (self.lp_pack_z[0] <= self.lp_strap_z[0] and self.lp_strap_z[1] <= self.lp_pack_z[1]
                                    and self.lp_strap_slot_y > self.lp_pack_y[1])
        if not self.lp_strap_wraps_pack:
            raise ValueError("logic pack: the 6S pack's lower strap does not wrap it (strap band / slot span)")
        self.lp_retention_mass_g = float(rt["strap_tie_mass_g_ASSUMED"])
        self.lp_retention_com = (self.lp_pack_x[0], 0.0, 0.5 * (zs + self.lp_tie_z))
        f_n = self.lp_pack_mass_g * 1e-3 * 9.80665 * float(rt["touchdown_g_ASSUMED"])
        self.lp_retention_E = {"touchdown_load_N": f_n, "friction_mu_ASSUMED": float(rt["friction_mu_ASSUMED"]),
                               "normal_force_needed_N (2 faces)": f_n / (2.0 * float(rt["friction_mu_ASSUMED"]))}
        # ---- switch through the top cap (hole placed by _top_cap) ----
        sw = lp["switch"]
        sx, sy, sd = self.cap_holes["logic power switch"]
        self.lp_sw_xy, self.lp_sw_hole_d = (sx, sy), sd
        self.lp_sw_r, self.lp_sw_theta = math.hypot(sx, sy), float(sw["theta_deg"])
        bx_, by_, bd_ = sw["body_xyz_ASSUMED"]
        self.lp_sw_nut_d, self.lp_sw_bushing_d = float(sw["nut_d_ASSUMED"]), float(sw["bushing_d"])
        self.lp_sw_lever_d = float(sw["lever_d_ASSUMED"])
        if self.lp_sw_bushing_d >= sd:
            raise ValueError("logic_power.switch: the bushing does not pass the hole")
        # review fix (2026-11-05, G5): the nuts must bear on the plate round the hole (flats - hole >= 2 x min bearing) and
        # the nut stack must fit on the bushing (outer nut + lock washer within the bushing above the plate)
        self.lp_sw_nut_af = float(sw["nut_af_ASSUMED"])
        self.lp_sw_nut_bearing_w = 0.5 * (self.lp_sw_nut_af - sd)
        k = "switch nut bearing on the plate (flats - hole) / 2"
        cl[k], lim[k] = self.lp_sw_nut_bearing_w, float(sw["nut_min_bearing_w_ASSUMED"])
        self.lp_sw_thread_spare = (float(sw["above_plate_h_ASSUMED"]) - float(sw["inner_nut_h_ASSUMED"])
                                   - float(sw["lock_washer_t_ASSUMED"]))
        k = "switch bushing above the plate - (outer nut + lock washer)"
        cl[k], lim[k] = self.lp_sw_thread_spare, 0.0
        z_bt = self.z_cap_plate_bottom - float(sw["inner_nut_h_ASSUMED"])
        self.lp_sw_inner_nut_z = (z_bt, self.z_cap_plate_bottom)
        self.lp_sw_body = ((sx - bx_ / 2, sx + bx_ / 2), (sy - by_ / 2, sy + by_ / 2), (z_bt - bd_, z_bt))
        zl = z_bt - bd_ - float(sw["lugs_h_ASSUMED"])
        self.lp_sw_lugs = ((sx - bx_ / 2 + 1.0, sx + bx_ / 2 - 1.0), (sy - by_ / 2 + 1.5, sy + by_ / 2 - 1.5), (zl, z_bt - bd_))
        self.lp_sw_outer_nut_z = (self.z_cap_top, self.z_cap_top + float(sw["inner_nut_h_ASSUMED"]))
        self.lp_sw_bushing_top_z = self.z_cap_top + float(sw["above_plate_h_ASSUMED"])
        self.lp_sw_top_z = self.lp_sw_bushing_top_z + float(sw["lever_h_ASSUMED"])
        self.lp_sw_mass_g = float(sw["mass_g_ASSUMED"])
        self.lp_sw_center = (sx, sy, 0.5 * (zl + self.lp_sw_top_z))
        nut = ((sx - self.lp_sw_nut_d / 2, sx + self.lp_sw_nut_d / 2), (sy - self.lp_sw_nut_d / 2, sy + self.lp_sw_nut_d / 2),
               self.lp_sw_inner_nut_z)
        below = [self.lp_sw_body, self.lp_sw_lugs, nut]
        corners = [(x, y) for bb in below for x in bb[0] for y in bb[1]]
        k = "switch body / nut -> rim ring (radial)"
        cl[k], lim[k] = self.cap_ring_r[0] - max(math.hypot(x, y) for x, y in corners), mf
        def rect(bb: tuple) -> list:
            return [(bb[0][0], bb[1][0]), (bb[0][1], bb[1][0]), (bb[0][1], bb[1][1]), (bb[0][0], bb[1][1])]
        ribs = []
        for th in self.cap_rib_thetas:
            c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
            ribs.append([(self.cap_rib_r[0] * c, self.cap_rib_r[0] * s), (self.cap_rib_r[1] * c, self.cap_rib_r[1] * s)])
        k = "switch body / nut -> ribs"
        cl[k] = min(_convex_gap_2d(rect(bb), rb) for bb in below for rb in ribs) - self.cap_rib_t / 2
        lim[k] = mf
        k = "switch body / nut -> hub"
        cl[k] = min(math.hypot(max(bb[0][0], min(0.0, bb[0][1])), max(bb[1][0], min(0.0, bb[1][1]))) for bb in below) \
            - self.cap_hub_d / 2
        lim[k] = mf
        k = "switch body -> arming-switch body"
        cl[k], lim[k] = min(_box_gap(bb, self.cap_switch_box) for bb in below), mf
        if getattr(self, "cap_hardpoint", False):
            r_st = max(self.cap_nut_af / math.sqrt(3), self.cap_bolt_d / 2)
            st = ((-r_st, r_st), (-r_st, r_st), (self.cap_stack_low_z, self.z_cap_under))
            k = "switch body -> hardpoint stack"
            cl[k], lim[k] = min(_box_gap(bb, st) for bb in below), mc
        # review fix (2026-11-05, G1): the cap's joint screws (M3 at joint_thetas, z skirt bottom + joint_screw_dz, head on
        # the skirt at body_r, driven radially in).  The project proxy for a screw of ANY length (joint_screw_axes, Ø
        # joint_hole_outer_d to the axis) crosses the switch body at 135 deg, so the cap screws are limited to
        # switch.cap_joint_screw_max_len (tip at body_r - len): that finite screw keeps >= min_clear_to_tray from the
        # switch below the plate; the any-length proxy distance and the longest screw that still keeps it are reported
        z_js = self.z_cap_skirt_bottom + self.joint_screw_dz
        r_js = 0.5 * float(p["body"]["joint_hole_outer_d"])
        self.lp_cap_screw_max_len = float(sw["cap_joint_screw_max_len"])
        def screw_gap(th: float, ln: float) -> float:
            c, s = math.cos(math.radians(th)), math.sin(math.radians(th))
            r0 = max(self.body_r - ln, 0.0)
            return min(_box_segment_dist(bb, (r0 * c, r0 * s, z_js), (self.body_r * c, self.body_r * s, z_js))
                       for bb in below) - r_js
        self.lp_cap_screw_z = z_js
        self.lp_cap_screw_gap = {th: screw_gap(th, self.lp_cap_screw_max_len) for th in self.joint_thetas}
        self.lp_cap_screw_any_len_gap = {th: screw_gap(th, self.body_r) for th in self.joint_thetas}
        th_c = min(self.lp_cap_screw_gap, key=self.lp_cap_screw_gap.get)
        lo_, hi_ = 0.0, self.body_r                # longest screw at th_c that keeps min_clear_to_tray (bisection)
        for _ in range(60):
            mid = 0.5 * (lo_ + hi_)
            lo_, hi_ = (mid, hi_) if screw_gap(th_c, mid) >= mc else (lo_, mid)
        self.lp_cap_screw_critical = (th_c, lo_)
        k = f"switch body / lugs -> cap joint screw M3 x {self.lp_cap_screw_max_len:g} (max, {th_c:g} deg)"
        cl[k], lim[k] = self.lp_cap_screw_gap[th_c], mc
        for nm, bx in list(bay.items()) + [("logic pack", pack)]:
            cl[f"switch body / lugs -> {nm}"] = min(_box_gap(bb, bx) for bb in below)
            lim[f"switch body / lugs -> {nm}"] = mc
        k = "switch lever / nut -> body edge (radial)"
        cl[k], lim[k] = self.body_r - (self.lp_sw_r + self.lp_sw_nut_d / 2), 0.0
        # ---- leads (keep-out Ø lp_wire_od): switch lugs -> J15, pack top (lead exit) -> J14 ----
        ld = lp["leads"]
        self.lp_wire_od = float(ld["wire_od_ASSUMED"])
        rw = self.lp_wire_od / 2
        def plug_top(nm: str) -> tuple:
            b = self.lp_plugs[nm]
            return (0.5 * (b[0][0] + b[0][1]), 0.5 * (b[1][0] + b[1][1]), b[2][1])
        j15, j14 = plug_top("J15"), plug_top("J14")
        z_drop = self.fcu_z[1] + rw + 0.5         # the vertical legs end just over the carrier envelope's top
        self.lp_switch_lead_polyline = [(sx, sy, zl), (sx, sy, z_drop), j15]
        pl = ld["pack_to_J14"]
        ex = pk["lead_exit_ASSUMED"]
        if ex["end"] != "top":
            raise ValueError("logic_power.pack.lead_exit_ASSUMED.end: only 'top' is routed (the bottom end sits over the "
                             "servo bundle with no room for the lead bend)")
        xc = self.lp_pack_center[0]
        zt = self.lp_pack_z[1] + float(pl["over_pack_top"])
        vx, vy = float(pl["vertical_x"]), float(pl["vertical_y"])
        self.lp_pack_lead_polyline = [(xc, float(ex["y"]), self.lp_pack_z[1]), (xc, float(ex["y"]), zt), (vx, vy, zt),
                                      (vx, vy, z_drop), j14]
        def route_len(pts: list) -> float:
            return sum(math.dist(a, b) for a, b in zip(pts[:-1], pts[1:]))
        def route_com(pts: list) -> tuple:
            segs = list(zip(pts[:-1], pts[1:]))
            n_ = route_len(pts)
            return tuple(sum(0.5 * (a[i] + b[i]) * math.dist(a, b) for a, b in segs) / n_ for i in range(3))
        sl = ld["switch_to_J15"]
        self.lp_switch_lead_route_mm = route_len(self.lp_switch_lead_polyline)
        self.lp_switch_lead_len_mm = self.lp_switch_lead_route_mm + float(sl["slack_mm"])
        self.lp_switch_lead_mass_g = (self.lp_switch_lead_len_mm * 1e-3 * float(sl["mass_g_per_m_ASSUMED"])
                                      + float(sl["connector_g_ASSUMED"]))
        self.lp_switch_lead_com = route_com(self.lp_switch_lead_polyline)
        self.lp_pack_lead_route_mm = route_len(self.lp_pack_lead_polyline)
        self.lp_pack_lead_len_mm = self.lp_pack_lead_route_mm + float(pl["slack_mm"])
        self.lp_cap_skirt_engagement_mm = self.z_cap_plate_bottom - self.z_cap_skirt_bottom
        k = "switch lead slack -> cap skirt engagement (lift the cap before unplugging J15)"
        cl[k], lim[k] = float(sl["slack_mm"]) - self.lp_cap_skirt_engagement_mm, 0.0
        carrier = bay.get(self.fcu_part)
        for lead, pts in (("switch lead", self.lp_switch_lead_polyline), ("pack lead", self.lp_pack_lead_polyline)):
            segs = list(zip(pts[:-1], pts[1:]))
            above = segs[:-1]                       # the last segment enters its plug box inside the carrier envelope
            k = f"{lead} -> bore (radial)"
            cl[k] = self.body_id / 2 - max(math.hypot(q[0], q[1]) for q in pts) - rw
            lim[k] = PLACEHOLDER_BORE_CLEAR_MM
            for nm, bx in bay.items():
                if nm == self.fcu_part:
                    continue
                k = f"{lead} -> {nm}"
                cl[k], lim[k] = min(_box_segment_dist(bx, a, b) for a, b in segs) - rw, 0.0
            if carrier is not None:
                k = f"{lead} -> {self.fcu_part} (above its envelope)"
                cl[k], lim[k] = min(_box_segment_dist(carrier, a, b) for a, b in above) - rw, 0.0
        k = "switch lead -> logic pack"
        cl[k] = min(_box_segment_dist(pack, a, b) for a, b in zip(self.lp_switch_lead_polyline[:-1],
                                                                   self.lp_switch_lead_polyline[1:])) - rw
        lim[k] = mlp
        k = "pack lead -> logic pack (its own lead, beside it)"
        cl[k] = min(_box_segment_dist(pack, a, b) for a, b in zip(self.lp_pack_lead_polyline[1:-1],
                                                                   self.lp_pack_lead_polyline[2:])) - rw
        lim[k] = 0.0
        # ---- review fix (2026-11-05, G3 / DP-10): strap / tie loops in plan vs the leads (E, information for C37) ----
        # the lengthened lower 6S strap: hull of the tray slots (tray_x, +-slot y), both packs' corners; the tie: hull of
        # the tray lightening holes' outer edges (tray_x, +-tray_hole_y[1], retention.tie_path_ASSUMED) and both packs.
        # Each lead's vertical leg (plan point) -> signed band gap: inside the loop = (distance to the band's inner face) -
        # lead radius (negative = the band presses on the lead: it is held); outside = distance - band thickness - lead
        # radius (negative = the lead lies over the band)
        bx6 = bay["battery"]
        corners = ([(x, y) for x in self.lp_pack_x for y in self.lp_pack_y]
                   + [(x, y) for x in bx6[0] for y in bx6[1]])
        self.lp_strap_loop = _convex_hull_2d(corners + [(self.tray_x, s * self.lp_strap_slot_y) for s in (1.0, -1.0)])
        y_tie = float(self.tray_hole_y[1]) if getattr(self, "tray_hole_y", None) else self.lp_strap_slot_y
        self.lp_tie_loop = _convex_hull_2d(corners + [(self.tray_x, s * y_tie) for s in (1.0, -1.0)])
        self.lp_strap_wraps_flight_pack_corners = [c for c in ((bx6[0][0], bx6[1][0]), (bx6[0][0], bx6[1][1]))
                                                   if c in self.lp_strap_loop]
        t_band = {"strap": float(rt["strap_t_ASSUMED"]), "tie": float(rt["tie_t_ASSUMED"])}
        legs = {"switch lead": (sx, sy), "pack lead": (vx, vy)}
        self.lp_band_leads = {}
        for band, poly in (("strap", self.lp_strap_loop), ("tie", self.lp_tie_loop)):
            for lead, xy in legs.items():
                sd_ = _signed_dist_convex_2d(poly, xy)
                inside = sd_ < 0.0
                gap = (-sd_ - rw) if inside else (sd_ - t_band[band] - rw)
                self.lp_band_leads[f"{lead} vs {band}"] = {"inside_loop": inside, "centre_to_band_mm": abs(sd_),
                                                           "band_gap_mm": gap}
        # ---- reach / exposure (information for C39) ----
        te = p["top_cap"]["tether"]
        r_eye, h_eye = 0.5 * float(te["eye_od_ASSUMED"]), float(te["eye_h_ASSUMED"])
        self.lp_sw_lever_h = self.lp_sw_top_z - self.z_cap_top
        hull = h_eye * (self.body_r - self.lp_sw_r) / (self.body_r - r_eye)
        self.lp_exposure = {"lever top above the cap top": self.lp_sw_lever_h,
                            "eye-bolt top above the cap top (when fitted)": h_eye,
                            "lever top below the eye-bolt top": h_eye - self.lp_sw_lever_h,
                            "rim -> eye-top line height at the switch r (tip-over onto the top corner)": hull,
                            "lever top over that line (information)": self.lp_sw_lever_h - hull,
                            "lever axis -> eye axis (plan)": self.lp_sw_r,
                            "lever axis -> body edge (plan)": self.body_r - self.lp_sw_r}
        pcx, pcy = 0.5 * sum(self.cap_patch_xy[0]), 0.5 * sum(self.cap_patch_xy[1])
        dp = math.hypot(sx - pcx, sy - pcy)
        self.lp_gps_lever_shadow_deg = {"azimuth_width": 2 * math.degrees(math.atan2(0.5 * self.lp_sw_nut_d, dp)),
                                        "elevation_top": math.degrees(math.atan2(self.lp_sw_top_z - self.cap_patch_z[1],
                                                                                 dp - 0.5 * self.lp_sw_nut_d))}
        fr = 0.5 * float(sw["finger_clear_d_ASSUMED"])
        self.lp_finger_r = fr
        def rect_pt(xy: tuple, x: float, y: float) -> float:
            return math.hypot(max(xy[0][0] - x, x - xy[0][1], 0.0), max(xy[1][0] - y, y - xy[1][1], 0.0))
        tops = {"GPS ground plane": rect_pt(self.cap_gp_xy, sx, sy), "GPS board": rect_pt(self.cap_board_xy, sx, sy),
                "tether eye (when fitted)": math.hypot(sx, sy) - r_eye,
                "arming switch (its nut, Ø as this one)": math.hypot(sx - self.cap_holes["arming switch"][0],
                                                                      sy - self.cap_holes["arming switch"][1])
                - 0.5 * self.lp_sw_nut_d}
        for nm, dv in tops.items():
            k = f"finger circle (Ø {2 * fr:g}) round the lever -> {nm}"
            cl[k], lim[k] = dv - fr, 0.0
        k = "finger circle round the lever -> cap edge"
        cl[k], lim[k] = self.body_r - self.lp_sw_r - fr, 0.0
        # ---- switch-collar round (user decision 2026-11-05, '상단 캡과 한 몸으로 출력하는 보호 칼라를 달아줘') ----
        self.has_lp_collar = bool(lp.get("collar"))
        if self.has_lp_collar:
            self._logic_collar(p, cl, lim, tops_xy={"GPS ground plane": self.cap_gp_xy, "GPS board": self.cap_board_xy},
                               r_eye=r_eye)
        # J15 reach with the cap off: the nearest avionics-window bottom tip (tweezers through the window)
        if getattr(self, "has_av_windows", False):
            self.lp_j15_window = min(
                ((math.dist((self.body_id / 2 * math.cos(math.radians(t)), self.body_id / 2 * math.sin(math.radians(t)),
                             self.avw_z0), j15), t) for t in self.avw_thetas))
        self.lp_clearance, self.lp_limits = cl, lim
        for k_, v_ in cl.items():
            if v_ < lim[k_] - 1e-9:
                raise ValueError(f"logic power: {k_} {v_:.2f} mm < {lim[k_]:g}")
        # the slicer table keeps the hole-less cap's slice as 'top_cap@no_logic_switch_hole' (make_v09_print_files): the
        # tether / hybrid builds and, while in_flight_mass_model is false, the flight-mass model use that geometry
        # (switch-collar round: that geometry is the cap without the hole AND without the collar)
        self.slicer_variants = {"top_cap": "no_logic_switch_hole"}

    def _logic_collar(self, p: dict, cl: dict, lim: dict, tops_xy: dict, r_eye: float) -> None:
        """Switch-collar round (user decision 2026-11-05, '상단 캡과 한 몸으로 출력하는 보호 칼라를 달아줘';
        params configs.vtol.logic_power.collar): the protective collar round the logic switch lever, one piece with the
        top cap - a tube (bore Ø inner_d, wall wall_t) on the plate top coaxial with the switch, rim h over the plate
        top, two finger notches on the lever's throw axis (vehicle y).  Review fix (2026-11-05, F1 / F2): each notch is
        a U slot - width notch_w, a round bottom of radius notch_w / 2 whose lowest point is notch_bottom_h over the
        plate top (centre ``lp_col_notch_zc``), straight sides from the centre up through the rim; the round bottom
        replaces the box slot's sharp re-entrant corners.  Derives ``lp_col_*`` and the lever sweep ``lp_lever_*``
        (pivot at the bushing's body end, +- lever_throw_deg_ASSUMED about it) and adds to ``cl`` / ``lim`` (raised by
        _logic_power when short): the FDM wall rule (a multiple of printer.nozzle_d, >= min_feature, all perimeters with
        the cap's class), the rim over the sweep's top (>= rim_margin_min), the sweep in plan inside the rim tops' convex
        hull (>= plan_margin_min), the sweep -> collar (>= lever_clear_min), the fingertip sphere at each end position ->
        the notch bottom (>= printer.layer_h) / the arcs / the plate (>= 0), the FINGER PATH (review F1: the fingertip
        probe kept on the lever's outboard tip corner while it pushes the lever from the end position to
        finger_path_past_centre_deg past centre) -> the collar >= finger_path_margin_min, the bore -> the existing Ø
        finger circle (>= 0), the collar -> every other cap-top feature (holes, GPS parts, cap edge >= min_feature; the
        eye bolt when fitted >= 1.0 as the GPS parts; the arming switch's Ø finger circle >= 0).  Reported (E): the GPS
        sky shading seen from the patch centre (``lp_col_gps``), the side-load sections along the notch height
        (``lp_col_section``: the arc's largest arm / Z, review F2) and the plate root (``lp_col_plate_root``: the
        rigid-boss plate model, review F3); the collar volume / centroid (``lp_col_volume_mm3``, ``lp_col_centroid``)
        for the Fusion twin's flight-model totals.  Nothing here moves another part."""
        lp, pr = p["logic_power"], p["printer"]
        mf, nz, lh = float(pr["min_feature"]), float(pr["nozzle_d"]), float(pr["layer_h"])
        sw, co = lp["switch"], lp["collar"]
        sx, sy = self.lp_sw_xy
        ri, t = 0.5 * float(co["inner_d"]), float(co["wall_t"])
        ro = ri + t
        h, hn, w = float(co["h"]), float(co["notch_bottom_h"]), float(co["notch_w"])
        ru = 0.5 * w
        if not (0.0 < hn and hn + ru <= h) or w <= 0.0 or w >= 2.0 * ri:
            raise ValueError("logic_power.collar: the U notch (lowest point notch_bottom_h, round bottom of radius "
                             "notch_w / 2) must lie between the plate and the rim and notch_w inside the bore")
        self.lp_col_ri, self.lp_col_ro, self.lp_col_t = ri, ro, t
        self.lp_col_z = (self.z_cap_top, self.z_cap_top + h)
        self.lp_col_notch_w, self.lp_col_notch_z0 = w, self.z_cap_top + hn
        self.lp_col_notch_zc = self.z_cap_top + hn + ru          # the U's centre: straight sides above, round below
        self.lp_col_axis = "y"            # the throw axis (switch.body_note: the lever throws along vehicle y)
        # (1) FDM wall rule: whole 0.4 mm lines, >= min_feature, all perimeters with the cap's print class
        n_lines = t / nz
        cls = (pr.get("part_class") or {}).get("top_cap")
        loops = int(pr["print_classes"][cls]["wall_loops"]) if cls else 0
        self.lp_col_wall = {"t_mm": t, "lines_of_nozzle": n_lines, "class": cls, "wall_loops_per_side": loops,
                            "perimeter_width_mm": 2 * loops * nz}
        if abs(n_lines - round(n_lines)) > 1e-9:
            raise ValueError(f"logic_power.collar.wall_t {t:g} is not a whole number of {nz:g} mm lines")
        k = "collar wall - printer.min_feature"
        cl[k], lim[k] = t - mf, 0.0
        k = "collar wall: perimeters (2 x class wall loops x nozzle) - wall_t"
        cl[k], lim[k] = 2 * loops * nz - t, 0.0
        # (2) the lever sweep (A): a Ø lever_d cylinder from the bushing top to lp_sw_top_z (vertical), turned +- throw
        # about the pivot at the bushing's body end (the inner nut's bottom)
        if sw.get("lever_pivot_ASSUMED") != "body_end_of_bushing":
            raise ValueError("logic_power.switch.lever_pivot_ASSUMED: only 'body_end_of_bushing' is modelled")
        th_deg = float(sw["lever_throw_deg_ASSUMED"])
        th = math.radians(th_deg)
        zpv = self.lp_sw_inner_nut_z[0]
        Lt = self.lp_sw_top_z - zpv
        rl = 0.5 * self.lp_sw_lever_d
        self.lp_lever_pivot_z, self.lp_lever_len, self.lp_lever_throw_deg = zpv, Lt, th_deg
        self.lp_lever_s0 = self.lp_sw_bushing_top_z - zpv
        a_top = min(math.atan2(rl, Lt), th)            # the tip end's upper corner peaks slightly off vertical
        self.lp_lever_env_top_z = zpv + Lt * math.cos(a_top) + rl * math.sin(a_top)
        self.lp_lever_tip_y = Lt * math.sin(th) + rl * math.cos(th)          # the outer corner at an end position
        self.lp_lever_corner_z = zpv + Lt * math.cos(th) - rl * math.sin(th)
        z_rim = self.lp_col_z[1]
        k = "collar rim - lever sweep top (every position)"
        cl[k], lim[k] = z_rim - self.lp_lever_env_top_z, float(co["rim_margin_min"])
        y_o = math.sqrt(ro * ro - 0.25 * w * w)          # the notch chord of the rim tops' convex hull
        self.lp_col_hull_chord_y = y_o
        k = "lever sweep inside the rim hull (plan, along the throw axis)"
        cl[k], lim[k] = y_o - self.lp_lever_tip_y, float(co["plan_margin_min"])
        k = "lever sweep -> collar bore (radial)"
        cl[k], lim[k] = ri - self.lp_lever_tip_y, float(co["lever_clear_min"])
        # (3) finger access at each end position: a fingertip sphere (Ø finger_tip_d) level with the outer corner of the
        # lever's tip end, finger_tip_d / 2 outboard of it along the throw axis (a horizontal push over centre)
        rf = 0.5 * float(co["finger_tip_d_ASSUMED"])
        yc, zc = self.lp_lever_tip_y + rf, self.lp_lever_corner_z
        self.lp_col_finger = {"sphere_r_mm": rf, "centre_local_y_mm": yc, "centre_z": zc,
                              "centres": [(sx, sy + s * yc, zc) for s in (1.0, -1.0)],
                              "body_len_mm": 30.0, "contact_end_positions_deg": [th_deg, -th_deg]}
        dpl = yc - ro
        z_low = zc - (math.sqrt(rf * rf - dpl * dpl) if 0.0 < dpl < rf else (rf if dpl <= 0.0 else -math.inf))
        k = "finger tip sphere -> collar notch bottom (z, over the wall band)"
        cl[k], lim[k] = z_low - self.lp_col_notch_z0, lh
        d_arc = math.hypot(0.5 * w, yc - y_o) if yc > y_o else 0.5 * w
        k = "finger tip sphere -> collar arcs (plan)"
        cl[k], lim[k] = d_arc - rf, 0.0
        k = "finger tip sphere -> plate top (z)"
        cl[k], lim[k] = zc - rf - self.z_cap_top, 0.0
        # (3b) review fix (2026-11-05, F1): the finger PATH, not only the end positions.  The fingertip probe (sphere Ø
        # finger_tip_d + the finger behind it, a Ø finger_tip_d cylinder outward along the throw axis) stays on the
        # lever's outboard tip corner (horizontal push, the sphere centre level with the corner) while it pushes the lever
        # from the end position (+throw) to finger_path_past_centre_deg past centre, every finger_path_step_deg; the
        # distance to the collar material (the U slot's surface within the wall, the plate) must stay >=
        # finger_path_margin_min at every step (A: room for a real, soft fingertip).  Both notches are mirror images
        # (the throw axis is the collar's symmetry plane), so one side is computed.  C41 measures it on the solids.
        zc_u, z_top_ = self.lp_col_notch_zc, self.z_cap_top

        def probe_dist(ycl: float, zcl: float) -> float:
            def d_pt(x_: float, z_: float) -> float:
                ax_ = abs(x_)
                yi_ = math.sqrt(max(ri * ri - ax_ * ax_, 0.0))
                yo_ = math.sqrt(max(ro * ro - ax_ * ax_, 0.0))
                del yi_                          # the finger body covers y >= ycl: the wall's inner edge never governs
                if ycl <= yo_:
                    return math.hypot(x_, z_ - zcl) - rf
                return math.sqrt(x_ * x_ + (yo_ - ycl) ** 2 + (z_ - zcl) ** 2) - rf
            ds = [d_pt(ru * math.cos(math.radians(-90.0 + 0.5 * k_)), zc_u + ru * math.sin(math.radians(-90.0 + 0.5 * k_)))
                  for k_ in range(181)]                                     # the round bottom (one side; symmetric in x)
            ds.append(d_pt(ru, min(max(zcl, zc_u), z_rim)))                 # the straight side, nearest height
            ds.append(zcl - rf - z_top_)                                    # the plate top inside the bore
            return min(ds)
        step = float(co["finger_path_step_deg"])
        past = float(co["finger_path_past_centre_deg"])
        n_st = int(math.ceil((th_deg + past) / step - 1e-9))
        path = []
        for j in range(n_st + 1):
            a_deg = max(th_deg - j * step, -past)
            ar = math.radians(a_deg)
            ycn = Lt * math.sin(ar) + rl * math.cos(ar) + rf
            zcn = zpv + Lt * math.cos(ar) - rl * math.sin(ar)
            path.append((a_deg, ycn, zcn, probe_dist(ycn, zcn)))
        worst_p = min(path, key=lambda q: q[3])
        self.lp_col_finger_path = {"angles_deg": [q[0] for q in path], "centre_local_y_mm": [q[1] for q in path],
                                   "centre_z": [q[2] for q in path], "dist_mm": [q[3] for q in path],
                                   "min_mm": worst_p[3], "at_deg": worst_p[0],
                                   "margin_min_mm": float(co["finger_path_margin_min"])}
        k = "finger path (end position -> past centre) -> collar (3D, min over the push)"
        cl[k], lim[k] = worst_p[3], float(co["finger_path_margin_min"])
        k = "collar bore -> lever finger circle (radial)"
        cl[k], lim[k] = ri - self.lp_finger_r, 0.0
        # (4) the collar's outer circle against every other cap-top feature (plan)
        def rect_pt(xy: tuple, x: float, y: float) -> float:
            return math.hypot(max(xy[0][0] - x, x - xy[0][1], 0.0), max(xy[1][0] - y, y - xy[1][1], 0.0))
        for hn_, (hx, hy, hd) in self.cap_holes.items():
            if hn_ == "logic power switch":
                continue
            k = f"collar -> {hn_} hole"
            cl[k], lim[k] = math.hypot(hx - sx, hy - sy) - ro - 0.5 * hd, mf
        for nm, xy in tops_xy.items():
            k = f"collar -> {nm}"
            cl[k], lim[k] = rect_pt(xy, sx, sy) - ro, mf
        k = "collar -> tether eye (when fitted)"
        cl[k], lim[k] = math.hypot(sx, sy) - ro - r_eye, 1.0
        k = "collar -> cap edge"
        cl[k], lim[k] = self.body_r - (self.lp_sw_r + ro), mf
        ax, ay, _ = self.cap_holes["arming switch"]
        k = "collar -> arming switch finger circle"
        cl[k], lim[k] = math.hypot(ax - sx, ay - sy) - ro - self.lp_finger_r, 0.0
        k = "outer nut -> collar bore (nut driver room, radial)"
        cl[k], lim[k] = ri - 0.5 * self.lp_sw_nut_d, mf
        # (5) GPS sky seen from the patch centre (E): the collar top at its nearest point, its azimuth width, the share
        # of the sky above the receiver's elevation mask it shades (azimuth x elevation band, conservative)
        gs = co["gps_sky"]
        pcx, pcy = 0.5 * sum(self.cap_patch_xy[0]), 0.5 * sum(self.cap_patch_xy[1])
        dp = math.hypot(sx - pcx, sy - pcy)
        el = math.degrees(math.atan2(z_rim - self.cap_patch_z[1], dp - ro))
        az = 2.0 * math.degrees(math.asin(min(1.0, ro / dp)))
        mask = float(gs["receiver_elev_mask_deg_ASSUMED"])
        sm = math.sin(math.radians(mask))
        frac = (0.0 if el <= mask else
                math.radians(az) * (math.sin(math.radians(el)) - sm) / (2.0 * math.pi * (1.0 - sm)))
        self.lp_col_gps = {"patch_centre_to_collar_axis_mm": dp, "nearest_plan_distance_mm": dp - ro,
                           "rim_over_patch_top_mm": z_rim - self.cap_patch_z[1], "top_elevation_deg": el,
                           "azimuth_width_deg": az, "receiver_mask_deg_A": mask,
                           "blocked_sky_above_mask_pct": 100.0 * frac,
                           "limit_pct": float(gs["max_blocked_sky_above_mask_pct"])}
        # (6) side-load sections (E, judged in C41 with the report mass).  Review fix (2026-11-05, F2): with the U notch
        # the arc's section grows below the U's centre, so the governing section is searched along the notch height:
        # at height z over the plate the arc is the annulus part with x_local >= the slot half-width x(z) (notch_w / 2 on
        # the straight part, sqrt(r_u^2 - (z_u - z)^2) on the round bottom), loaded on its rim (arm h - z); the largest
        # arm / Z_min is kept (``arc_k_per_mm2``; C41: stress = F x it x kt_notch).  Below the notch the full ring.
        def arc_sec(xh: float) -> dict:
            n_ = 48
            ao, ai = math.acos(xh / ro), math.acos(xh / ri)
            arc = ([(ro * math.cos(-ao + 2 * ao * j / n_), ro * math.sin(-ao + 2 * ao * j / n_)) for j in range(n_ + 1)]
                   + [(ri * math.cos(ai - 2 * ai * j / n_), ri * math.sin(ai - 2 * ai * j / n_)) for j in range(n_ + 1)])
            return _polygon_section(arc)
        zu_rel = hn + ru

        def half_w(zr: float) -> float:
            return ru if zr >= zu_rel else math.sqrt(max(ru * ru - (zu_rel - zr) ** 2, 0.0))

        def k_of(zr: float) -> float:
            xh = half_w(zr)
            return (h - zr) / arc_sec(xh)["z_min"] if xh > 0.05 else 0.0
        n_z = 60
        zs = [hn + (h - hn) * j / n_z for j in range(n_z + 1)]
        j_best = max(range(n_z + 1), key=lambda j: k_of(zs[j]))
        lo_, hi_ = zs[max(j_best - 1, 0)], zs[min(j_best + 1, n_z)]
        for _ in range(40):                                    # golden-section refinement of the peak
            m1, m2 = lo_ + 0.382 * (hi_ - lo_), lo_ + 0.618 * (hi_ - lo_)
            if k_of(m1) < k_of(m2):
                lo_ = m1
            else:
                hi_ = m2
        z_g = 0.5 * (lo_ + hi_)
        z_g = max((z_g, zs[j_best]), key=k_of)
        sec = arc_sec(half_w(z_g))
        self.lp_col_section = {"arc": sec, "arc_z_over_plate_mm": z_g, "arc_half_width_mm": half_w(z_g),
                               "arc_arm_mm": h - z_g, "arc_k_per_mm2": (h - z_g) / sec["z_min"],
                               "ring_z_mm3": math.pi * (ro ** 4 - ri ** 4) / (4.0 * ro), "ring_arm_mm": h}
        # (6b) review fix (2026-11-05, F3): the plate root.  The collar is a rigid boss (radius ro) in the 1.2 mm plate;
        # the plate is modelled as an annular Kirchhoff plate clamped at b = the FARTHEST first support seen from the
        # collar axis (rays every 1 deg to the rim ring's inner face, the hub, the ribs - conservative: the real panel is
        # held closer on most sides), unit stresses per N mm of boss moment and per N of axial force (_annular_plate_boss);
        # C41 multiplies them by the load cases (side_load.plate_root)
        pl = co["side_load"]["plate_root"]
        rects = [(th_, self.cap_rib_r[0], self.cap_rib_r[1], 0.5 * self.cap_rib_t) for th_ in self.cap_rib_thetas]
        rays = []
        for k_ in range(360):
            a_ = math.radians(k_)
            rays.append(_ray_exit(sx, sy, math.cos(a_), math.sin(a_), [self.cap_ring_r[0]], [0.5 * self.cap_hub_d], rects))
        b_far = max(rays)
        if not math.isfinite(b_far) or b_far <= ro:
            raise ValueError("logic_power.collar: no support round the collar for the plate-root model")
        unit = _annular_plate_boss(ro, b_far, self.cap_plate_t, float(pl["nu_ASSUMED"]))
        self.lp_col_plate_root = {"boss_r_mm": ro, "b_farthest_support_mm": b_far, "b_nearest_support_mm": min(rays),
                                  "plate_t_mm": self.cap_plate_t, "nu_A": float(pl["nu_ASSUMED"]), **unit,
                                  "lever_rule_share": self.body_r / (self.body_r + self.lp_sw_r)}
        # (7) volume / centroid of the material the collar adds over the plate top (the tube minus the two U slots, both
        # across the wall: per x across the slot, wall length in y 2 (sqrt(ro^2 - x^2) - sqrt(ri^2 - x^2)) times the slot
        # height from its bottom zb(x) = hn + ru - sqrt(ru^2 - x^2) to the rim; x = ru sin(u) makes the integrand smooth,
        # composite Simpson, 2000 panels - converged far below the 1e-4 C41 rule)
        v_ring = math.pi * (ro * ro - ri * ri) * h
        n_s = 2000
        v_cut = m_cut = 0.0
        for j in range(n_s + 1):
            u_ = -0.5 * math.pi + math.pi * j / n_s
            wgt = (1 if j in (0, n_s) else (4 if j % 2 else 2)) * (math.pi / n_s) / 3.0
            x_ = ru * math.sin(u_)
            ly = 2.0 * (math.sqrt(ro * ro - x_ * x_) - math.sqrt(ri * ri - x_ * x_))
            zb = hn + ru - ru * math.cos(u_)
            dx_ = ru * math.cos(u_)
            v_cut += wgt * ly * (h - zb) * dx_
            m_cut += wgt * ly * 0.5 * (h * h - zb * zb) * dx_
        self.lp_col_volume_mm3 = v_ring - v_cut
        zbar = (v_ring * 0.5 * h - m_cut) / self.lp_col_volume_mm3
        self.lp_col_centroid = (sx, sy, self.z_cap_top + zbar)

    def _servo_lead_route(self) -> None:
        """SPEC 3.8 / 5.1 vane-servo lead route (edf.mount = tabs).  Every passage is sized from the ribbon envelope
        (ribbon_w x ribbon_t, ASSUMED) plus min_clearance per side and raises when it is not; walls that the route
        leaves are checked against printer.min_feature (also listed in C10)."""
        p = self.p
        sl, wc, mf = p["servo_leads"], p["intake_module"]["wire_channel"], p["printer"]["min_feature"]
        W, T, c = sl["ribbon_w_ASSUMED"], sl["ribbon_t_ASSUMED"], sl["min_clearance"]
        self.sl_ribbon_w, self.sl_ribbon_t = W, T

        def need(name: str, have: float, want: float) -> None:
            if have < want - 1e-9:
                raise ValueError(f"servo lead route: {name} {have:.2f} mm < {want:.2f} mm")

        if sl["lead_exit_ASSUMED"] != "far_end":
            raise ValueError("servo_leads.lead_exit_ASSUMED: only 'far_end' (lead opposite the output shaft) is routed; a "
                             "lead at the shaft end would leave toward the vane horn and pushrod")
        en = sl["entry"]
        self.sl_entry_w = en["w"]
        self.sl_entry_u0 = self.sleeve_r + en["gap_to_sleeve"]
        self.sl_entry_u1 = self.sl_entry_u0 + en["radial"]
        if self.direct_drive:
            # v0.9 (servo_leads.fin_route): the lead leaves the servo pocket through a closed pipe that rises on the duct
            # surface from the fin to the v0.6 entry slot at column theta = fin theta + column_theta_from_fin_deg
            fr = sl["fin_route"]
            self.sl_fin_thetas = list(self.fin_thetas)
            self.sl_col_thetas = [t + fr["column_theta_from_fin_deg"] for t in self.fin_thetas]
            self.sl_side = 1.0 if fr["column_theta_from_fin_deg"] >= 0 else -1.0
            need("lead pipe tunnel (ribbon diagonal + 2 x clearance)", self.sl_tunnel_d, math.hypot(W, T) + 2 * c)
            need("lead pipe wall", self.sl_pipe_wall, mf)
        else:
            # the far end of the case is the pod side away from the output shaft (servo_shaft_tangential_offset)
            self.sl_side = 1.0 if self.servo_shaft_tangential_offset <= 0 else -1.0
            # (1) pod window: through the far end wall, from the cavity floor up, centred on the servo axis
            pw = sl["pod_window"]
            self.sl_win_u0, self.sl_win_u1 = self.servo_axis_r - pw["w"] / 2, self.servo_axis_r + pw["w"] / 2
            self.sl_win_z0, self.sl_win_z1 = self.pod_floor_top_z, self.pod_floor_top_z + pw["h"]
            need("pod window width", pw["w"], W + 2 * c)
            need("pod window height", pw["h"], T + 2 * c)
            if self.sl_win_u0 < self.pod_cavity_r_in + mf or self.sl_win_u1 > self.pod_cavity_r_out - mf:
                raise ValueError("servo lead pod window leaves less than min_feature beside it")
            # (2) outside: up beside the pod's far face to the crossing band, then over the fin line (above its leading
            # edge, below the shoulder-ring chamfer) to the column theta = fin theta + side x column_theta_from_fin
            self.sl_fin_thetas, self.sl_col_thetas = [], []
            for th in self.pod_thetas:
                ahead = min(((f - th) * self.sl_side) % 360.0 for f in self.fin_thetas)
                thf = th + self.sl_side * ahead
                self.sl_fin_thetas.append(thf)
                self.sl_col_thetas.append(thf + self.sl_side * sl["column_theta_from_fin_deg"])
            z_le = self.fin_root_le_z - (self.sl_entry_u0 - self.fin_root_r) * (self.fin_root_le_z - self.fin_tip_le_z) / (
                self.fin_tip_r - self.fin_root_r)                                 # fin leading edge at the slot's inner face
            self.sl_cross_z0 = z_le + sl["crossing_above_fin_le"]
            self.sl_cross_z1 = self.sl_cross_z0 + W
        # (3) entry slot: vertical box r [sl_entry_u0, sl_entry_u1], tangential +-w/2, from the vane-section top (no
        # material at r >= u0 there) through the shoulder ring and the cone skin; open into the annulus above
        # sl_entry_z_open, where the skin's inner surface passes u1
        need("entry slot width", en["w"], W + 2 * c)
        need("entry slot radial", en["radial"], T + 2 * c)
        if self.sl_entry_u0 <= self.vane_section_r or self.sl_entry_u1 + self.cone_wall > self.body_r:
            raise ValueError("servo lead entry slot must lie between the vane section and the skin")
        self.sl_entry_z0 = self.z_vane_section_top
        self.sl_entry_z_mouth = self.outer_skin_z_at_radius(self.sl_entry_u0)
        self.sl_entry_z_open = self.outer_skin_z_at_radius(self.sl_entry_u1 + self.cone_wall)
        if not self.direct_drive and self.sl_cross_z1 > self.sl_entry_z_mouth:
            raise ValueError("servo lead crossing band reaches the shoulder-ring chamfer")
        if self.direct_drive:
            self._fin_lead_pipes(mf)
        # wire09c clock mount: no lugs - the lowest new solid above the annulus is the lowest keyway floor-pad corbel
        z_lug0 = self.lp_pad_z_bottom if self.clock_mount else self.block_z0
        if self.sl_entry_z_open < self.z_edf_shoulder or self.sl_entry_z_open > z_lug0 - mf:
            raise ValueError("servo lead entry must open into the annulus between the shoulder ring and the screw lugs")
        # (4) deck notch at the column: (a) through the deck, r [u0, spigot_od/2] (the skirt seat outside it stays),
        # (b) up the spigot ring, r [u0, spigot_od/2 + overtravel], open to the skirt, up to the rebate
        dn, rb = sl["deck_notch"], sl["rebate"]
        so = self.spigot_od / 2
        self.sl_notch_w = dn["w"]
        self.sl_notch_u0 = so - dn["radial"]
        need("deck notch width", dn["w"], W + 2 * c)
        need("deck notch radial", dn["radial"], T + 2 * c)
        self.sl_deck_seat = self.body_r - math.hypot(so, dn["w"] / 2)       # deck left outside the notch corners
        need("deck (skirt seat) outside the notch", self.sl_deck_seat, mf)
        if self.sl_notch_u0 < self.sleeve_r + mf:
            raise ValueError("servo lead deck notch reaches the sleeve")
        # (5) rebate: annular sector r [sl_rebate_r0, spigot OD], Z [sl_rebate_z0, spigot top], from the outermost
        # column on each side to gather_theta_deg; it opens the keyways above the tabs, never the motor wire slot
        self.sl_rebate_r0 = so - rb["depth"]
        self.sl_rebate_z0 = self.z_prop_spigot_top - rb["h"]
        need("rebate depth (two ribbons)", rb["depth"], 2 * T + 2 * c)
        need("rebate height", rb["h"], W + 2 * c)
        z_joint_top = self.z_prop_spigot_bottom + self.joint_screw_dz + p["body"]["joint_hole_spigot_d"] / 2
        self.sl_rebate_floor_over_joint = self.sl_rebate_z0 - z_joint_top
        need("rebate floor over the joint holes", self.sl_rebate_floor_over_joint, mf)
        self.sl_rebate_ring = self.sl_rebate_r0 - self.edf_bore_r
        need("spigot ring inside the rebate", self.sl_rebate_ring, mf)
        self.sl_rebate_over_tab = self.sl_rebate_z0 - self.tab_z1
        need("rebate floor above the tab tops", self.sl_rebate_over_tab, mf)
        tg = float(sl["gather_theta_deg"])
        self.sl_gather_theta = tg
        half_n = math.degrees(math.atan2(dn["w"] / 2, self.sl_notch_u0))
        offs = [((t - tg + 180.0) % 360.0) - 180.0 for t in self.sl_col_thetas]
        self.sl_rebate_th0 = tg + min(offs) - half_n
        self.sl_rebate_th1 = tg + max(offs) + half_n
        hw_slot = math.degrees(math.atan2(self.wire_slot_w / 2, self.wire_slot_r_in)) + 1.0
        if self.clock_mount:     # wire09c: the lead passage's angular half width at the rebate's inner radius
            hw_slot = self.lp_E_design + math.degrees(math.atan2(self.lp_hw, self.sl_rebate_r0)) + 1.0
        if (self.wire_theta - self.sl_rebate_th0 + hw_slot) % 360.0 <= (self.sl_rebate_th1 - self.sl_rebate_th0) + 2 * hw_slot:
            raise ValueError("servo lead rebate would cross the motor wire slot")
        # a spigot-ring point outside the rebate between the motor-lead passage and the rebate start (Fusion's
        # cut_verified 'kept' point; wire09c: clear of every clock keyway too)
        r_reb = 0.5 * (self.sl_rebate_r0 + so)
        self.sl_rebate_kept_theta = 0.5 * (self.sl_rebate_th0 + self.wire_theta
                                           + math.degrees(math.atan2(self.wire_slot_w / 2.0, r_reb)))
        for th in self.sl_col_thetas:                 # columns clear the webs (entry) and the joint holes (notch)
            for w in self.web_thetas:
                dth = math.radians(abs(((th - w) + 180.0) % 360.0 - 180.0))
                need(f"entry slot {th:g} deg -> web {w:g} deg", self.sl_entry_u0 * math.sin(dth) - en["w"] / 2 - self.web_t / 2, mf)
            for j in self.joint_thetas:
                dth = math.radians(abs(((th - j) + 180.0) % 360.0 - 180.0))
                need(f"deck notch {th:g} deg -> joint hole {j:g} deg",
                     self.sl_notch_u0 * math.sin(dth) - dn["w"] / 2 - p["body"]["joint_hole_spigot_d"] / 2, mf)
            for t in ([] if self.clock_mount else self.tab_thetas):   # and the screw-lug sectors (clock mount: none)
                lo, hi = t - self.tab_screw_head_side * self.block_theta_far, t + self.tab_screw_head_side * self.block_theta_head
                lo, hi = min(lo, hi), max(lo, hi)
                if (th - lo + half_n) % 360.0 <= (hi - lo) + 2 * half_n:
                    raise ValueError(f"servo lead column {th:g} deg runs into the screw lug of the tab at {t:g} deg")
        # (6) intake: servo channel + bulge at gather_theta (pocket-wall rule, wall and gate_h of the motor channel)
        ch = sl["channel"]
        self.sl_ch_w = ch["w"]
        self.sl_ch_r0 = self.lip_pocket_id / 2 + wc["wall"]
        self.sl_ch_r1 = self.sl_ch_r0 + ch["depth"]
        self.sl_bulge_r_out = self.sl_ch_r1 + wc["wall"]
        self.sl_bulge_w = ch["w"] + 2 * wc["wall"]
        need("servo channel width (2 x 2 ribbons)", ch["w"], 2 * W + 2 * c)
        need("servo channel depth (2 x 2 ribbons)", ch["depth"], 2 * T + 2 * c)
        self.sl_gate_lo_z0 = self.sl_rebate_z0            # the leads step out of the rebate within [this, skirt top]
        need("step-out window", self.z_intake_skirt_top - self.sl_gate_lo_z0, W + 2 * c)
        need("step-in gate", wc["gate_h"], 2 * T + 2 * c)
        self.sl_ch_z0 = self.sl_gate_lo_z0 - (self.sl_ch_r1 - self.body_id / 2)   # 45 deg floor wedge, as wire_ch_z0
        self.sl_ch_floor_rz = [(self.body_id / 2, self.sl_ch_z0), (self.sl_ch_r1 + 0.5, self.sl_ch_z0),
                               (self.sl_ch_r1 + 0.5, self.sl_ch_z0 + (self.sl_ch_r1 + 0.5 - self.body_id / 2))]
        self.sl_ch_z1 = self.z_lip_pocket_top + wc["gate_h"]
        self.sl_bulge_z0 = self.sl_ch_z0 - wc["wall"]
        self.sl_bulge_z1 = self.sl_ch_z1 + wc["wall"]
        self.sl_notch_r0 = self.shelf_id / 2 - 0.5
        if self.sl_ch_r0 > so - 0.5:
            raise ValueError("servo channel does not overlap the rebate radially")
        if self.sl_bulge_z0 < self.z_intake_bottom + 2.0 or self.sl_bulge_z1 > self.z_window_bottom:
            raise ValueError("servo bulge runs off the intake skirt or into the windows")
        if min(abs((tg - s + 180.0) % 360.0 - 180.0) for s in self.strut_thetas) > 1e-6:
            raise ValueError("servo channel should sit on a strut (servo_leads.gather_theta_deg)")
        sep = math.radians(abs((tg - self.wire_theta + 180.0) % 360.0 - 180.0))
        if self.body_r * sep < (self.wire_bulge_w + self.sl_bulge_w) / 2 + 10.0:
            raise ValueError("servo bulge too close to the motor-lead bulge")
        self.sl_hole_r0, self.sl_hole_r1 = sl["bulkhead_hole_r"]
        need("bulkhead servo hole radial", self.sl_hole_r1 - self.sl_hole_r0, 2 * T + 2 * c)
        if self.sl_hole_r1 > self.spigot_id / 2 - 0.5:
            raise ValueError("bulkhead servo hole cuts the intake spigot")
        # v0.6.1 band: the servo notch + channel open the step-in through the thicker band, and the 2 x 2 bundle runs
        # up beside the strut (centred in the bulkhead servo hole: 2T radial x 2W tangential) inside the band's inner
        # face - its outer corners at (centre + T, +-W) are the nearest points to the band
        self._band_step_in_check("servo-lead", self.sl_notch_r0, self.sl_ch_r0, self.sl_ch_r1, ch["w"], self.sl_ch_z1)
        self.band_clear_servo_hole = self.r_band_in - self.sl_hole_r1
        need("bulkhead servo hole outer edge -> intake band inner face", self.band_clear_servo_hole, 0.0)
        self.band_clear_servo_bundle = self.r_band_in - math.hypot(0.5 * (self.sl_hole_r0 + self.sl_hole_r1) + T, W)
        need("servo bundle beside the strut -> intake band inner face", self.band_clear_servo_bundle, c)
        # (7) lead lengths and the mass the route adds (wire beyond the splice + slack + FC connector per lead)
        # v0.9 direct drive: the MG90S lead runs to its inline connector in the fairing's connector bay; the extension (the
        # mass the route adds) starts at the connector's middle, which is the 3rd polyline point (servo_lead_polyline)
        self.sl_route_len, self.sl_route_len_modelled, self.sl_ext_len = [], [], []
        m_tot, mom = 0.0, [0.0, 0.0, 0.0]
        g_m = sl["mass_g_per_m_ASSUMED"] / 1000.0
        for k in range(len(self.sl_col_thetas)):
            pts = self.servo_lead_polyline(k)
            seg = [(pts[i], pts[i + 1], math.dist(pts[i], pts[i + 1])) for i in range(len(pts) - 1)]
            total = sum(s[2] for s in seg)
            kept = sum(s[2] for s in seg[:2]) if self.direct_drive else sl["servo_lead_kept"]
            self.sl_route_len.append(total)
            self.sl_route_len_modelled.append(total - sum(s[2] for s in seg[-3:]))   # last 3 legs: bay estimate
            self.sl_ext_len.append(total - kept + sl["slack"])
            s_acc = 0.0
            for a, b, ln in seg:                                # wire beyond the splice, mass at each piece's middle
                lo = max(s_acc, kept)
                if lo < s_acc + ln:
                    f = 0.5 * ((lo - s_acc) / ln + 1.0)
                    m = (s_acc + ln - lo) * g_m
                    m_tot += m
                    for i in range(3):
                        mom[i] += m * (a[i] + (b[i] - a[i]) * f)
                s_acc += ln
            m = sl["slack"] * g_m + sl["fc_connector_mass_g"]     # slack and connector at the board end (v0.6.3: FCU)
            m_tot += m
            for i in range(3):
                mom[i] += m * pts[-1][i]
            if self.direct_drive:                              # the inline connector pair in the fairing
                m = sl["fin_route"]["connector_pair_g_ASSUMED"]
                m_tot += m
                for i in range(3):
                    mom[i] += m * pts[2][i]
                need("MG90S lead kept to the connector (servo_leads.fin_route.servo_lead_kept_ASSUMED)",
                     sl["fin_route"]["servo_lead_kept_ASSUMED"], kept)
            else:
                # the splice (servo_lead_kept from the case, splice_len long) lies outside, before the crossing
                need("route length before the crossing (splice outside)", sum(s[2] for s in seg[:2]),
                     sl["servo_lead_kept"] + sl["splice_len"] / 2)
        self.sl_lead_mass_g = m_tot
        self.sl_lead_com = tuple(m / m_tot for m in mom)

    # ------------------------------------------------------------------------------------------------------------
    # wire09c clock mount (user 2026-10-30: 'make the motor-lead passage generous, I do not want to measure').  The
    # EDF is turned (clocked) until its factory lead bundle points into ONE fixed lead passage at theta_P =
    # lead_passage.theta_deg; its two tabs drop into the nearest pair of the keyways cut round the circle.  The passage
    # takes every corner of motor_leads.generosity_box (factory heat-shrink kept, any exit azimuth), leaves the body
    # through an external blister and is checked by C29 / C34 / C35 (check_geometry).  M / E / A as in the params.
    # ------------------------------------------------------------------------------------------------------------
    def clock_rho_c(self, od: float) -> float:
        """Round envelope half of the count single leads in any arrangement (the stacked row, count x od / 2, is the
        largest; a triangle (1 + 2 / sqrt 3) od / 2) + wire_clearance."""
        n = int(self.p["motor_leads"]["count"])
        return max(0.5 * n * od, 0.5 * (1.0 + 2.0 / math.sqrt(3.0)) * od) + self.lp_wcl

    def _clock_fan(self, p: dict) -> None:
        """Plan of the lead passage (Layout.lp_*), from the generosity box: half width lp_hw = the largest tangential
        envelope half of anything in it (bundle round / ribbon flat or on edge, the single leads stacked) + fan_margin;
        outer wall lp_r_wall = motor_d / 2 + the longest factory sleeve + the largest bend radius + the stacked thickest
        leads + bend_tol (the stiff sleeve never enters the bend); fanned over +-lp_E_design = E_cov_deg (half the
        largest keyway gap) + the tab's play in its keyway.  The fan floor lies floor_margin under the lowest envelope
        of every corner.  Raises when the passage would not leave the bore / reach the skin."""
        pm, ml, e = p["propulsion_module"], p["motor_leads"], p["edf"]
        ck, lp, gb = pm["clock_keyways"], pm["lead_passage"], ml["generosity_box"]
        self.lp_wcl = float(ml["wire_clearance"])
        self.ck_kw = float(ck["kw"])
        if self.ck_kw < self.tab_t + 0.4 - 1e-9:
            raise ValueError("clock_keyways.kw must take the tab (tab t + 2 x 0.2)")
        self.ck_tab_t_max = self.ck_kw - 0.4
        self.keyway_w = self.ck_kw
        r_mid = 0.5 * (self.tab_r_in + self.tab_r_out)
        self.lp_tab_play_deg = math.degrees((self.ck_kw - self.tab_t) / 2 / r_mid)
        self.lp_E_cov = float(ck["E_cov_deg"])
        self.lp_E_design = self.lp_E_cov + self.lp_tab_play_deg
        self.gb_S = sorted(float(x) for x in gb["shrink_len_from_motor"])
        self.gb_sections = {k: (float(v[0]), float(v[1])) for k, v in gb["sections"].items()}
        self.gb_od = sorted(float(x) for x in gb["od"])
        self.gb_R = sorted(float(x) for x in gb["bend_r_centre"])
        self.gb_inset = sorted(float(x) for x in gb["te_inset"])
        self.gb_gap = sorted(float(x) for x in gb["rim_gap"])
        self.gb_protrusion = sorted(float(x) for x in gb["motor_protrusion"])
        self.lp_bend_tol = float(lp["bend_tol"])
        self.lp_early = [0.0, self.lp_bend_tol]
        od_max = max(self.gb_od)
        rib = [math.hypot(h, w) for h, w in self.gb_sections.values() if abs(h - w) > 1e-9]   # a ribbon may turn
        self.lp_rho_t_max = max(max(w for _, w in self.gb_sections.values()) + self.lp_wcl, self.clock_rho_c(od_max),
                                (max(rib) + self.lp_wcl) if rib else 0.0)
        self.lp_hw = self.lp_rho_t_max + float(lp["fan_margin"])
        self.lp_rm = e["motor_d"] / 2
        self.lp_r_wall = self.lp_rm + max(self.gb_S) + max(self.gb_R) + self.clock_rho_c(od_max) + self.lp_bend_tol
        self.lp_wall = float(lp["wall"])
        self.lp_u_a = self.duct_r - float(lp["u_start_below_duct"])
        self.lp_u_top = float(lp["u_top"])
        self.lp_steer_len = float(lp["steer_len"])
        self.lp_z_top_gap = float(lp["z_top_gap"])
        st = float(lp["arc_step_deg"])
        th, E = self.wire_theta, self.lp_E_design
        self.lp_fan_xy = clock_fan_outline(th, E, self.lp_u_a, self.lp_r_wall, self.lp_hw, "rect", st)
        self.lp_fan_hull = clock_fan_outline(th, E, self.lp_u_a, self.lp_r_wall, self.lp_hw, "chord", st)
        self.lp_blister_r_out = self.lp_r_wall + self.lp_wall
        self.lp_blister_hw = self.lp_hw + self.lp_wall
        self.lp_blister_xy = clock_fan_outline(th, E, self.edf_bore_r, self.lp_blister_r_out, self.lp_blister_hw,
                                               "circle", st)
        self.lp_blister_hull = clock_fan_outline(th, E, self.lp_u_a, self.lp_blister_r_out, self.lp_blister_hw, "chord",
                                                 st)
        self.lp_blister_r_max = math.hypot(self.lp_blister_r_out, self.lp_blister_hw)
        if self.lp_r_wall <= self.body_r + 1.0:
            raise ValueError("lead passage: the outer wall lies inside the body (no blister needed - check the box)")
        # lowest envelope of every corner (u >= duct_r - 0.5): bundle box / single-lead circle
        z_low = math.inf
        for c in self.clock_corners():
            pth = self.clock_lead_path(c)
            z_low = min(z_low, pth["z_c"] - pth["hb"], pth["z_s"] - pth["rho"])
        self.lp_z_env_low = z_low
        self.lp_z_fan_floor = z_low - float(lp["floor_margin"])
        self.lp_z_blister_bottom = self.lp_z_fan_floor - self.lp_wall
        if self.lp_z_blister_bottom < self.z_vane_section_top + 2.0:
            raise ValueError("lead passage floor reaches the vane section (the blister would sit on the duct tube)")

    def clock_corners(self, module_only: bool = False) -> list[dict]:
        """Every corner of motor_leads.generosity_box (shrink length x section x lead OD x bend radius x rim gap x TE
        inset x hug (bend starts 0 / bend_tol before the wall)).  module_only drops the TE inset: it only lifts the
        bundle INSIDE the duct (u < duct_r - 1), where the propulsion module has no material - exact, not a monotonic
        argument; the inset corners are checked against the EDF (C29)."""
        out = []
        for S in self.gb_S:
            for sec, (h, w) in self.gb_sections.items():
                for od in self.gb_od:
                    for R in self.gb_R:
                        for gap in self.gb_gap:
                            for ins in ([0.0] if module_only else self.gb_inset):
                                for early in self.lp_early:
                                    out.append({"S": S, "sec": sec, "h": h, "w": w, "od": od, "R": R, "gap": gap,
                                                "inset": ins, "early": early})
        return out

    def clock_lead_path(self, c: dict) -> dict:
        """Centre line of one corner in its own vertical plane (azimuth theta_P + e), (u, z): the factory bundle
        straight from the motor side (r motor_d / 2) out to rm + S at z_c (its top gap under the rim; with a TE inset
        the part inside the duct sits up to inset higher), the single leads straight to r_s (pressed down to the same
        gap under the rim), bend R up to the vertical leg at r_v = r_wall - rho_c - early (the leads hug the blister's
        outer wall), vertical to z_r0, steer to (u_top, v = 0) at z_r1 = z_intake_bottom - z_top_gap, then flat side by
        side in the intake groove."""
        rm, wcl = self.lp_rm, self.lp_wcl
        z_hb = self.z_edf_housing_bottom
        rc = self.clock_rho_c(c["od"])
        r_v = self.lp_r_wall - rc - c["early"]
        r_s = r_v - c["R"]
        r_e = rm + c["S"]
        z_c = z_hb - c["gap"] - c["h"]
        z_s = min(z_c, z_hb - c["gap"] - (rc - wcl))
        z_r1 = getattr(self, "lp_z_r1", None)
        z_r0 = (z_r1 - self.lp_steer_len) if z_r1 is not None else None
        out = {"rm": rm, "r_e": r_e, "r_s": r_s, "r_v": r_v, "R": c["R"], "z_c": z_c, "z_s": z_s, "z_v0": z_s + c["R"],
               "z_r0": z_r0, "z_r1": z_r1, "rho": rc, "hb": c["h"] + wcl, "wb": c["w"] + wcl,
               "z_in_duct_top": z_hb + c["inset"] - c["gap"], "bundle_in_bend": r_e > r_s + 1e-9,
               "flat": (0.5 * c["od"] + wcl, 0.5 * int(self.p["motor_leads"]["count"]) * c["od"] + wcl)}
        if z_r1 is not None:
            out["len_to_z_r1"] = ((r_s - rm) + (z_c - z_s) + 0.5 * math.pi * c["R"] + (z_r0 - (z_s + c["R"]))
                                  + math.hypot(self.lp_steer_len, r_v - self.lp_u_top))
        return out

    def _clock_pads(self, p: dict) -> None:
        """Keyway floor (z lp_z_floor = tab_z0 - axial_retention.floor_drop: room for the foam pad under each tab) and
        the floor pad under every keyway: an (r, z) polygon in the keyway's radial plane, kw + 2 pad_margin wide,
        pad_t thick (bridges the sleeve and the skin; the JUDGE FIX, 1.2 -> 3.0) with a 45 deg corbel underneath from
        both walls (meets mid span, prints without support)."""
        pm, ar = p["propulsion_module"], p["edf"]["axial_retention"]
        ck = pm["clock_keyways"]
        self.ar_floor_drop = float(ar["floor_drop"])
        self.lp_z_floor = self.tab_z0 - self.ar_floor_drop
        self.keyway_z0 = self.lp_z_floor
        if self.lp_z_floor < self.z_cyl0 - 1e-9:
            raise ValueError("clock keyway floor below the cylindrical section (keyway -> skin rule)")
        self.ck_pad_t = float(ck["pad_t"])
        self.ck_pad_margin = float(ck["pad_margin"])
        self.ck_pad_w = self.ck_kw + 2 * self.ck_pad_margin
        ov = 0.3
        z1 = self.lp_z_floor
        z0 = z1 - self.ck_pad_t

        def rin(z: float) -> float:
            return self.cone_outer_radius(z) - self.cone_wall

        r_m = 0.5 * (self.sleeve_r + rin(z0))
        self.ck_pad_span = rin(z0) - self.sleeve_r
        # skin-side corbel end: the 45 deg line z = z0 - (r - r_m) meets the skin's inner face (bisection; the cone
        # narrows downward)
        lo, hi = z0 - 2.0 * self.ck_pad_span, z0
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            if z0 - (rin(mid) - r_m) > mid:
                lo = mid
            else:
                hi = mid
        z_k = 0.5 * (lo + hi)
        z_sl = z0 - (r_m - self.sleeve_r)
        pts = [(self.sleeve_r - ov, z1), (rin(z1) + ov, z1)]
        if z_k < self.cone_z1 < z1:
            pts.append((rin(self.cone_z1) + ov, self.cone_z1))
        pts += [(rin(z_k) + ov, z_k), (r_m, z0), (self.sleeve_r - ov, z_sl)]
        self.ck_pad_rz = pts
        self.lp_pad_z_bottom = min(z_k, z_sl)
        if math.degrees(math.atan2(1.0, 1.0)) < float(ck["pad_underside_chamfer_deg"]) - 1e-9:
            raise ValueError("clock_keyways.pad_underside_chamfer_deg: only the 45 deg corbel is modelled")

    def _clock_seat(self, p: dict) -> None:
        """Keyways round the circle (plan_keyways of wire09c concept C, computed - never typed in): a keyway (kw wide,
        r keyway_r_in .. keyway_r_out, floor lp_z_floor, open at the top) is placed where its sleeve teeth (sleeve_min
        each side) stay out of the lead passage, its walls off every web (+ web_margin), servo-lead column guide wall
        (+ col_clear) and joint hole, and its twin at +180 deg is also a keyway site or a tab can hover in the passage.
        Gaps <= 2 E_cov and >= the tooth minimum.  Coverage: the exit azimuth phi (from either tab) is covered when a
        seated configuration puts it within E_cov of theta_P; the passage takes E_cov + the tab play.  Raises on a
        keyway that hits a fence / the passage, a tooth < sleeve_min or coverage < 100 %."""
        pm = p["propulsion_module"]
        ck, lp = pm["clock_keyways"], pm["lead_passage"]
        smin, step = float(ck["sleeve_min"]), float(ck["plan_step_deg"])
        th_p = self.wire_theta
        h_kw = self.ck_kw / 2
        fen: list[tuple[float, list]] = []
        for w in self.web_thetas:
            hw_ = self.web_t / 2 + float(ck["web_margin"])
            fen.append((w, _polar_rect_2d(w, self.edf_bore_r - 1.0, self.body_r, -hw_, hw_)))
        self.ck_col_walls = []
        if self.servo_leads:
            t_, cl_ = float(ck["col_wall"]), float(ck["col_clear"])
            hw_ = self.sl_entry_w / 2 + t_ + cl_
            for c_ in self.sl_col_thetas:
                fen.append((c_, _polar_rect_2d(c_, self.sl_entry_u0 - t_, self.sl_entry_u1 + t_, -hw_, hw_)))
                for s_ in (-1.0, 1.0):
                    v0, v1 = sorted((s_ * self.sl_entry_w / 2, s_ * (self.sl_entry_w / 2 + t_)))
                    self.ck_col_walls.append((c_, v0, v1))
                    # (the wall's part in the sleeve - r < sleeve_r - is sleeve material the passage cuts anyway)
                    wall = _polar_rect_2d(c_, self.sleeve_r, self.body_r, v0, v1)
                    if _convex_overlap(self.lp_fan_hull, wall):
                        raise ValueError(f"lead passage cuts the servo-lead column guide wall at {c_:g} deg")
        jd = p["body"]["joint_hole_spigot_d"]
        for j in self.joint_thetas:
            hw_ = jd / 2 + smin
            # (from the bore whether the hole is blind or through: the keyways do not depend on prop_joint_hole_depth)
            fen.append((j, _polar_rect_2d(j, self.edf_bore_r - 1.0, self.spigot_od / 2 + 1.0, -hw_, hw_)))
        # the passage must also keep the joint holes whole
        for j in self.joint_thetas:
            hole = _polar_rect_2d(j, self.spigot_od / 2 - (self.prop_joint_hole_depth or 0.0), self.spigot_od / 2 + 1.0,
                                  -jd / 2, jd / 2)
            if _convex_overlap(self.lp_fan_hull, hole):
                raise ValueError(f"lead passage cuts the joint hole at {j:g} deg")
        # wire09c fix round (review F1): the sleeve tooth beside a keyway is a free pillar (sleeve_r - edf_bore_r deep
        # radially) between the keyway floor and the deck under the spigot ring.  It must carry the full design motor
        # torque on ONE tab (a hover seat; single contact in a two-tab seat; either turning direction) - fixed-fixed,
        # load uniform over the span (the tab bears over its whole length), E: w_req = sqrt(6 M f / (t sigma_b)),
        # M = F span / 12, F = torque / tab mid radius, torque = max(1, 1.5 x P / omega) (C35's design torque).
        # Every tooth (also the one between a keyway and the lead passage) is planned >= w_req + tooth_w_margin; webs and
        # column walls are NOT credited (conservative).  C35 measures the teeth on the solid.
        e_ = p["edf"]
        omega_ = e_["kv"] * e_["voltage_nominal_V"] * 2 * math.pi / 60.0
        t_des = max(1.0, 1.5 * e_["power_nominal_W"] / omega_)
        r_tab = 0.5 * (self.tab_r_in + self.tab_r_out)
        f_tab = t_des / (r_tab * 1e-3)
        t_rad = self.sleeve_r - self.edf_bore_r
        span_t = (self.z_prop_spigot_bottom - self.body_wall) - self.lp_z_floor
        sb_ = float(p["printer"]["bending_mpa"])
        f_min_t = float(ck["tooth_factor_min"])
        w_req = math.sqrt(6.0 * (f_tab * span_t / 12.0) * f_min_t / (t_rad * sb_))
        w_tooth = max(smin, w_req + float(ck["tooth_w_margin"]))
        self.ck_tooth = {"torque_design_Nm": t_des, "tab_force_N": f_tab, "tab_r_mid_mm": r_tab, "radial_t_mm": t_rad,
                         "span_mm": span_t, "z_floor": self.lp_z_floor, "z_deck_bottom": self.z_prop_spigot_bottom
                         - self.body_wall, "bending_mpa": sb_, "factor_min": f_min_t, "w_req_mm": w_req,
                         "w_plan_mm": w_tooth}
        fan = self.lp_fan_hull
        fan_half = self.lp_E_design + math.degrees(math.atan2(self.lp_hw, self.lp_u_a)) + 1.0
        kw_half = math.degrees(math.atan2(h_kw + w_tooth, self.keyway_r_in)) + 0.5

        def keyway_rect(a: float, extra: float = 0.0) -> list:
            return _polar_rect_2d(a, self.keyway_r_in, self.keyway_r_out, -(h_kw + extra), h_kw + extra)

        def keyway_ok(a: float) -> bool:
            # the tooth between the keyway and the passage keeps w_tooth (F1; it was sleeve_min before)
            if abs(_wrap180(a - th_p)) < fan_half + kw_half and _convex_overlap(keyway_rect(a, w_tooth), fan):
                return False
            kr = None
            for f_th, f_poly in fen:
                if abs(_wrap180(a - f_th)) < kw_half + 8.0:
                    kr = kr or keyway_rect(a)
                    if _convex_overlap(kr, f_poly):
                        return False
            return True

        hc = float(ck["hover_clear"])

        def hover_ok(a: float) -> bool:
            if abs(_wrap180(a - th_p)) > fan_half:
                return False
            tab = _polar_rect_2d(a, self.tab_r_in, self.tab_r_out + hc, -(self.tab_t / 2 + hc), self.tab_t / 2 + hc)
            return all(_in_convex(q, fan) for q in tab)

        # seat chain (wire09c fix round, F1; replaces the interval planner whose twins left a 1.73 mm tooth): on the
        # mod-180 circle a seat at alpha is a keyway pair (alpha, alpha + 180), or one keyway with the other tab hovering
        # in the passage.  Consecutive seats are >= s_req apart (every tooth >= w_tooth at the bore: keyways of
        # consecutive seats are the only neighbours on either side) and <= g_cap = 2 E_cov - plan_gap_margin_deg apart
        # (coverage with margin).  Fewest seats (fewest keyways / floor pads), then the smallest largest gap.
        n = int(round(360.0 / step))
        if abs(n * step - 360.0) > 1e-9 or n % 2:
            raise ValueError("clock_keyways.plan_step_deg must divide 180 deg")
        k_ok = [keyway_ok(k * step) for k in range(n)]
        h_ok = [hover_ok(k * step) for k in range(n)]
        half = n // 2
        opt: list = []
        for i in range(half):
            j2 = i + half
            if k_ok[i] and k_ok[j2]:
                opt.append(("pair", (i * step, j2 * step)))
            elif k_ok[i] and h_ok[j2]:
                opt.append(("hover", (i * step,)))
            elif h_ok[i] and k_ok[j2]:
                opt.append(("hover", (j2 * step,)))
            else:
                opt.append(None)
        s_req = math.degrees(w_tooth / self.edf_bore_r + 2.0 * math.asin(h_kw / self.edf_bore_r))
        g_cap = 2.0 * self.lp_E_cov - float(ck["plan_gap_margin_deg"])
        lo_i = int(math.ceil(s_req / step - 1e-9))
        hi_cap = int(math.floor(g_cap / step + 1e-9))
        if lo_i > hi_cap:
            raise ValueError(f"clock keyways: tooth spacing {s_req:.2f} deg > the coverage gap {g_cap:.2f} deg")

        def chain(hi: int):
            """Fewest-seat cycle (start = the first seat, in [0, hi)) with gaps in [lo_i, hi]; None if there is none."""
            best_ = None
            for s0 in range(min(hi, half)):
                if opt[s0] is None:
                    continue
                inf = 1 << 30
                cnt = [inf] * (half + 1)
                par = [-1] * (half + 1)
                cnt[0] = 1
                dq: list[int] = []
                qh = 0
                for k in range(1, half + 1):
                    a_ = k - lo_i
                    if a_ >= 0 and cnt[a_] < inf:
                        while len(dq) > qh and cnt[dq[-1]] >= cnt[a_]:
                            dq.pop()
                        dq.append(a_)
                    while len(dq) > qh and dq[qh] < k - hi:
                        qh += 1
                    if len(dq) == qh:
                        continue
                    if k == half or opt[(s0 + k) % half] is not None:
                        cnt[k], par[k] = cnt[dq[qh]] + (0 if k == half else 1), dq[qh]
                if cnt[half] < inf and (best_ is None or cnt[half] < best_[0]):
                    path, k = [], par[half]
                    while k > 0:
                        path.append(s0 + k)
                        k = par[k]
                    best_ = (cnt[half], [s0] + path[::-1])
            return best_

        sol = chain(hi_cap)
        if sol is None:
            raise ValueError(f"clock keyways: no seat chain with teeth >= {w_tooth:.2f} mm and gaps <= {g_cap:.2f} deg")
        lo_h, hi_h = lo_i, hi_cap
        while lo_h < hi_h:                       # smallest largest gap at the fewest seats
            mid = (lo_h + hi_h) // 2
            r_ = chain(mid)
            if r_ is not None and r_[0] == sol[0]:
                sol, hi_h = r_, mid
            else:
                lo_h = mid + 1
        plan = [(round((i % half) * step, 6), opt[i % half][0]) for i in sol[1]]
        out = sorted({round(a_ % 360.0, 3) for i in sol[1] for a_ in opt[i % half][1]})
        self.ck_keyways = out
        self.ck_seat_plan = plan
        self.ck_seat_intervals = plan               # (kept for reports: the planned seats, mod 180)
        self.ck_s_min_deg = s_req
        self.ck_plan_gap_cap_deg = g_cap
        # verify every keyway (fences, passage, teeth)
        for a_ in out:
            if not keyway_ok(a_):
                raise ValueError(f"clock keyway at {a_:.2f} deg hits a web / column / joint hole / the lead passage")
        teeth = []
        for i, a_ in enumerate(out):
            b_ = out[(i + 1) % len(out)]
            gap_deg = (b_ - a_) % 360.0
            if abs(_wrap180(0.5 * (a_ + (a_ + gap_deg)) - th_p)) < gap_deg / 2:
                continue                        # this gap holds the passage: its teeth are kept by keyway_ok (w_tooth)
            teeth.append(self.edf_bore_r * (math.radians(gap_deg) - 2.0 * math.asin(h_kw / self.edf_bore_r)))
        self.ck_min_tooth = min(teeth)
        if self.ck_min_tooth < w_tooth - 1e-3:
            raise ValueError(f"clock keyways: a sleeve tooth {self.ck_min_tooth:.2f} mm < {w_tooth:.2f} (F1 bending)")
        # seated configurations (tab A at a keyway; tab B at +180 in a keyway or hovering in the passage)
        cf = []
        for k in out:
            b_ = (k + 180.0) % 360.0
            if any(abs(_wrap180(b_ - x)) < 0.01 for x in out):
                cf.append({"a": k, "b": b_, "b_state": "keyway"})
            elif hover_ok(b_):
                cf.append({"a": k, "b": b_, "b_state": "hover"})
        self.ck_configs = cf
        self.ck_hover = [c_ for c_ in cf if c_["b_state"] == "hover"]
        # coverage of the exit azimuth phi (measured from a tab, phi and phi + 180 are the same unit)
        E = self.lp_E_cov
        angs = [c_["a"] for c_ in cf] + [c_["a"] + 180.0 for c_ in cf]
        cstep = float(ck["coverage_step_deg"])
        n_c = int(round(360.0 / cstep))
        worst, cov_n, gaps, s0 = 0.0, 0, [], None
        for i in range(n_c):
            phi = -180.0 + cstep * i
            best = min(abs(_wrap180(a_ + phi - th_p)) for a_ in angs)
            worst = max(worst, best)
            ok_ = best <= E + 1e-9
            cov_n += ok_
            if not ok_ and s0 is None:
                s0 = phi
            if ok_ and s0 is not None:
                gaps.append((s0, phi - cstep))
                s0 = None
        if s0 is not None:
            gaps.append((s0, 180.0 - cstep))
        self.ck_coverage = cov_n / n_c
        self.ck_worst_residual_deg = worst
        self.ck_coverage_gaps = gaps
        seats = sorted({round(a_ % 360.0, 3) for a_ in angs})
        self.ck_seat_positions = seats
        self.ck_largest_gap_deg = max((seats[(i + 1) % len(seats)] - seats[i]) % 360.0 for i in range(len(seats)))
        if self.ck_coverage < 1.0 - 1e-12:
            raise ValueError(f"clock keyways cover {100 * self.ck_coverage:.2f} % of the exit azimuth (gaps {gaps})")
        # nominal assembly drawn in the CAD: the configuration with both tabs in keyways nearest the params' tab angle
        t0 = float(p["edf"]["tab"]["theta0_deg"])
        both = [c_ for c_ in cf if c_["b_state"] == "keyway"]
        nom = min(both, key=lambda c_: abs(_wrap180(c_["a"] - t0)))
        self.tab_thetas = [nom["a"], nom["a"] + 180.0]
        self.ck_nominal = nom
        # blister (vertical extent), V keel (2 mm steps under the floor, 45 deg), the passage top
        self.lp_z_r1 = self.z_intake_bottom - self.lp_z_top_gap
        self.lp_z_r0 = self.lp_z_r1 - self.lp_steer_len
        self.lp_z_blister_top = self.z_intake_bottom - float(lp["blister_top_gap"])
        kd = float(lp["keel_deg"])
        if abs(kd - 45.0) > 1e-9:
            raise ValueError("lead_passage.keel_deg: only the 45 deg keel is modelled")
        u0, u1 = self.edf_bore_r, self.lp_blister_r_out
        stp = float(lp["keel_step"])
        segs = []
        u = u0
        while u < u1 - 1e-9:
            ue = min(u + stp, u1)
            hk = min(clock_fan_half_width(u, self.lp_E_design, self.lp_blister_r_out, self.lp_blister_hw),
                     clock_fan_half_width(ue, self.lp_E_design, self.lp_blister_r_out, self.lp_blister_hw))
            if hk > 0.2:
                segs.append((u, ue, hk))
            u = ue
        self.lp_keel = segs
        # frontal (axial) projection of the blister outside the body circle (E, for the rocket drag reference):
        # 1/2 int (r_out(phi)^2 - body_r^2) dphi over the blister's angular span, r_out = the far ray / outline crossing
        hull = self.lp_blister_hull
        ang = [_wrap180(math.degrees(math.atan2(y, x)) - th_p) for x, y in hull]
        a_lo, a_hi = min(ang), max(ang)
        n_f, area = 2000, 0.0
        for k in range(n_f):
            ph = math.radians(th_p + a_lo + (a_hi - a_lo) * (k + 0.5) / n_f)
            dx, dy = math.cos(ph), math.sin(ph)
            t_far = 0.0
            for i in range(len(hull)):
                (x0, y0), (x1, y1) = hull[i], hull[(i + 1) % len(hull)]
                ex, ey = x1 - x0, y1 - y0
                den = dx * ey - dy * ex
                if abs(den) < 1e-12:
                    continue
                t_ = (x0 * ey - y0 * ex) / den
                s_ = (x0 * dy - y0 * dx) / den
                if -1e-9 <= s_ <= 1.0 + 1e-9 and t_ > t_far:
                    t_far = t_
            if t_far > self.body_r:
                area += 0.5 * (t_far ** 2 - self.body_r ** 2) * math.radians(a_hi - a_lo) / n_f
        self.lp_blister_frontal_mm2 = area
        self.lp_keel_overlap = 0.5
        self.lp_keel_z_low = self.lp_z_blister_bottom - max(s_[2] for s_ in segs)
        if self.lp_keel_z_low < self.z_exit + 5.0:
            raise ValueError("lead-passage keel reaches the exhaust exit")
        # Fusion's 'kept' point of the servo-rebate cut: outside the rebate, between the passage and the rebate start,
        # >= 3 deg from every keyway
        if self.servo_leads:
            so = self.spigot_od / 2
            r_reb = 0.5 * (self.sl_rebate_r0 + so)
            lo_ = th_p + self.lp_E_design + math.degrees(math.atan2(self.lp_hw, r_reb)) + 1.0
            hi_ = self.sl_rebate_th0 - 1.0
            cands = [lo_ + (hi_ - lo_) * k / 200 for k in range(201)]
            good = [a_ for a_ in cands if min(abs(_wrap180(a_ - x)) for x in out) >= 3.0]
            if not good:
                raise ValueError("no spigot-ring point outside the rebate clear of the keyways (Fusion kept point)")
            self.sl_rebate_kept_theta = max(good, key=lambda a_: min(abs(_wrap180(a_ - x)) for x in out))
        # axial retention (A): up = the EDF lip on the intake shelf (lip_stop_gap above it at rest); down = the tabs
        # on foam pads on the keyway floors (floor_drop below the tabs), the pads on the floor-pad bridges
        ar = p["edf"]["axial_retention"]
        im = p["intake_module"]
        self.ar_type = ar["type"]
        if self.ar_type != "shelf_stop_foam_under_tabs":
            raise ValueError("edf.axial_retention.type: only 'shelf_stop_foam_under_tabs' is modelled")
        self.ar_lip_gap = float(im["lip_stop_gap"])
        self.ar_foam_t = float(ar["foam_t_free"])
        tol = float(ar["stack_tol_ASSUMED"])
        # foam thickness when the lip is up against the shelf (thrust) and at rest (lip_stop_gap below it), +- tolerance
        t_up = self.ar_floor_drop + self.ar_lip_gap
        self.ar_foam_compression = {
            "lip on the shelf (thrust), nominal": 1.0 - t_up / self.ar_foam_t,
            "lip on the shelf, stack + tol": 1.0 - (t_up - tol) / self.ar_foam_t,
            "lip on the shelf, stack - tol": 1.0 - (t_up + tol) / self.ar_foam_t,
            "drawn position (lip_stop_gap under the shelf)": 1.0 - self.ar_floor_drop / self.ar_foam_t,
            "drawn position, stack + tol": 1.0 - (self.ar_floor_drop - tol) / self.ar_foam_t}
        self.ar_lift_max = self.ar_lip_gap + tol
        self.ar_engaged_after_lift = min(self.tab_len, self.z_prop_spigot_top - self.tab_z0) - self.ar_lift_max

    def _motor_lead_clock(self, p: dict, n: int, od: float, wcl: float) -> None:
        """The leads' route for the mass, the report and the drawings (the checks use every corner): the LONGEST corner
        (ml_lower_uz in the theta_P plane, motor -> bend -> wall -> steer -> (u_top, z_r1)); jet blockage and drag
        from the WORST corner's frontal area (bundle 2 w x (its end - hub radius) + single leads (row 3 od or stack od)
        to duct_r, E); the jet free span without the optional RTV bond (motor side -> duct wall); the lumped extras
        (A): RTV bond (optional, counted) + chafe tape + factory shrink, the foam plug at the passage top, the foam pads
        under the tabs."""
        e, ml = p["edf"], p["motor_leads"]
        lr = ml["lower_route"]
        lp = p["propulsion_module"]["lead_passage"]
        ar = e["axial_retention"]
        rm = self.lp_rm
        self.ml_r_motor = rm
        self.ml_z_motor_bottom = self.z_edf_housing_bottom - max(self.gb_protrusion)
        best, best_len = None, -1.0
        for c in self.clock_corners(module_only=True):
            pth = self.clock_lead_path(c)
            if pth["z_r0"] < pth["z_v0"] + 1.0:
                raise ValueError("lead passage: no vertical leg between the bend and the steer (steer_len)")
            if pth["bundle_in_bend"]:
                raise ValueError("lead passage: the factory sleeve reaches into the bend (r_wall / bend_tol)")
            if min(pth["z_c"] - pth["hb"], pth["z_s"] - pth["rho"]) < self.ml_z_motor_bottom:
                raise ValueError("lead passage: a lead envelope reaches below the motor's rear end")
            if pth["len_to_z_r1"] > best_len:
                best, best_len = (c, pth), pth["len_to_z_r1"]
        c, pth = best
        self.ml_clock_longest = dict(c)
        self.ml_u_slot = self.lp_u_top
        self.ml_bend_r = c["R"]
        self.ml_z_run = self.ml_z_exit = pth["z_c"]
        arc = self._quarter_arc(pth["r_s"], pth["z_s"] + c["R"], c["R"], "lower_right")
        self.ml_lower_uz = ([(rm, pth["z_c"]), (pth["r_e"], pth["z_c"]), (pth["r_e"], pth["z_s"])] + arc
                            + [(pth["r_v"], self.lp_z_r0), (self.lp_u_top, self.lp_z_r1)])
        self.ml_lower_len = best_len
        self.ml_lower_segments = {"longest corner, motor -> (u_top, z_r1)": best_len}
        # jet: frontal area of the bundle and the single leads between the hub radius and duct_r, every corner (E)
        r_hub = max(e["hub_d_ASSUMED"] / 2, rm)
        a_duct = math.pi * (self.duct_r ** 2 - rm ** 2)
        worst = None
        for S in self.gb_S:
            for sec, (h, w) in self.gb_sections.items():
                for od_ in self.gb_od:
                    for row in ("row", "stack"):
                        u_e = min(max(rm + S, r_hub), self.duct_r)
                        a_b = 2 * w * (u_e - r_hub)
                        a_s = (n * od_ if row == "row" else od_) * (self.duct_r - u_e)
                        a_ = a_b + a_s
                        u_c = (a_b * 0.5 * (r_hub + u_e) + a_s * 0.5 * (u_e + self.duct_r)) / a_
                        if worst is None or a_ > worst["area_mm2"]:
                            worst = {"S": S, "sec": sec, "od": od_, "singles": row, "area_mm2": a_, "u_c": u_c,
                                     "bundle_len_in_jet_mm": u_e - r_hub, "single_leads_len_in_jet_mm": self.duct_r - u_e}
        self.ml_blockage = {"annulus_mm2": a_duct, "projected_mm2": worst["area_mm2"],
                            "projected_pct": 100 * worst["area_mm2"] / a_duct, "drag_area_mm2": worst["area_mm2"],
                            "drag_area_pct": 100 * worst["area_mm2"] / a_duct,
                            "bundle_len_in_jet_mm": worst["bundle_len_in_jet_mm"],
                            "single_leads_len_in_jet_mm": worst["single_leads_len_in_jet_mm"], "worst_corner": worst}
        jd = lr["jet_drag"]
        cx_, cy_, _ = self.ml_xyz(worst["u_c"], 0.0, pth["z_c"])
        self.ml_jet_drag = {"cd_ASSUMED": float(jd["cd_ASSUMED"]), "cd_band_ASSUMED": list(jd["cd_band_ASSUMED"]),
                            "area_mm2": worst["area_mm2"], "centroid_u_mm": worst["u_c"], "xy_mm": (cx_, cy_),
                            "azimuth_range_deg": [self.wire_theta - self.lp_E_design, self.wire_theta + self.lp_E_design]}
        # duct-wall window under the rim (E): the passage's chord at r = duct_r x (rim - fan floor)
        self.lp_duct_window_mm2 = (2 * clock_fan_half_width(self.duct_r, self.lp_E_design, self.lp_r_wall, self.lp_hw)
                                   * (self.z_edf_housing_bottom - self.lp_z_fan_floor))
        # restraints: no tie; the optional RTV bond to the stator TE; without it the jet free span is motor -> duct
        self.ml_lower_max_free_span = float(lr["max_free_span_jet_ASSUMED"])
        self.ml_lower_free_spans = [self.duct_r - rm]
        self.ml_bond_u = (r_hub, self.duct_r)
        self.ml_bond_gap_max = float(lr["bond_gap_max_ASSUMED"])
        self.ml_bond_gap_worst = max(self.gb_inset) + max(self.gb_gap)
        self.ml_stator_te_gap = self.ml_bond_gap_worst
        # clearances (E, in the theta_P plane; the CAD checks measure the 3D gaps)
        v_ = p["vane"]
        sweep_top = self.vane_hinge_z + math.hypot(self.vane_le_z - self.vane_hinge_z, self.vane_t / 2)
        stator_le = self.z_edf_housing_bottom + min(self.gb_inset) + float(e["wire_exit"]["stator_vane"]["chord_ASSUMED"])
        self.ml_lower_clear = {
            "thrust-vane sweep (any deflection) -> passage floor": self.lp_z_blister_bottom - sweep_top,
            "fin root LE -> passage floor": self.lp_z_blister_bottom - self.fin_root_le_z,
            "rotor (ahead of the stator LE, ASSUMED) -> bundle top (inset 0, gap 0)": stator_le - self.z_edf_housing_bottom,
            "motor rear end (shortest protrusion) -> lowest envelope": (self.lp_z_env_low
                                                                      - (self.z_edf_housing_bottom - min(self.gb_protrusion))),
            "blister outer r (max) - body r": self.lp_blister_r_max - self.body_r}
        self.ml_max_deflection_deg = v_.get("max_deflection_mech_deg")
        self.ml_theta_measured = True            # not an input in the clock mount (coverage C34)
        self.ml_theta_uncert_deg = 0.0
        self.ml_exit_angle_tol_deg = self.lp_E_design
        self.ml_bundle_env_in_duct = False
        self.ml_bundle_d = 2 * max(max(h, w) for h, w in self.gb_sections.values())
        self.ml_u_shrink_end = rm + max(self.gb_S)
        self.ml_shrink_end_margin = min(self.clock_lead_path(c_)["r_s"] - (rm + c_["S"])
                                        for c_ in self.clock_corners(module_only=True))
        self.ml_shrink_bend_margin = self.ml_shrink_end_margin
        self.ml_transition_len = 0.0
        self.ml_shrink_bend_phi_deg = 0.0
        # lumped extras (A)
        rtv = lr["rtv_g_ASSUMED"]
        th = math.radians(self.wire_theta)
        lumps = [(float(rtv["bond"]) + float(rtv["chafe_tape"]) + float(lr["bundle"]["shrink_g_ASSUMED"]),
                  self.ml_xyz(0.5 * (r_hub + self.duct_r), 0.0, pth["z_c"])),
                 (float(lp["top_plug_g_ASSUMED"]), self.ml_xyz(self.lp_u_top, 0.0, self.lp_z_blister_top)),
                 (float(ar["foam_pads_g_ASSUMED"]), (0.0, 0.0, self.lp_z_floor + 0.5 * self.ar_floor_drop))]
        self.ml_clock_lumps = lumps
        self.ml_rtv_g = sum(m_ for m_, _ in lumps)
        del th

    def ml_xyz(self, u: float, v: float, z: float) -> tuple[float, float, float]:
        """Vehicle coordinates of a point in the motor-lead frame (u radial at wire_theta, v = +e_theta, Z)."""
        t = math.radians(self.wire_theta)
        return (u * math.cos(t) - v * math.sin(t), u * math.sin(t) + v * math.cos(t), z)

    @staticmethod
    def _quarter_arc(cu: float, cz: float, r: float, quadrant: str, n: int = 12) -> list[tuple[float, float]]:
        """n + 1 points of a 90 deg arc of radius r about (cu, cz) in the (u, z) plane, in route order: 'lower_right' runs
        bottom -> right (phi 0..90: (cu + r sin phi, cz - r cos phi)), 'upper_left' runs left -> top."""
        out = []
        for k in range(n + 1):
            ph = 0.5 * math.pi * k / n
            if quadrant == "lower_right":
                out.append((cu + r * math.sin(ph), cz - r * math.cos(ph)))
            elif quadrant == "upper_left":
                out.append((cu - r * math.cos(ph), cz + r * math.sin(ph)))
            else:
                raise ValueError(quadrant)
        return out

    def _motor_lead_lower(self, p: dict, n: int, od: float, wcl: float) -> None:
        """v0.9 lead-exit, photo round (user photos 2026-10-29, docs/실물사진/EDF_모터선_출구_2026-10-29_1/2.jpg; edf.wire_exit,
        motor_leads.lower_route, propulsion_module.wire_slot.low_*).  The photos show the stator vanes at the housing's rear
        rim, the motor can protruding behind it, and the three leads leaving the motor SIDE at its FRONT ring (next to the
        stator hub, i.e. at the rim level), bundled in heat-shrink, radially out along / behind ONE stator vane and under the
        rear rim, then bending as single leads.  No climb along the can (round 1, superseded).  In the wire_theta plane (u
        radial, z), bend radius R (centre line), the heat-shrink bundle O Db (envelope half hb = Db/2 + wire_clearance both
        ways), the single leads side by side (half thickness h = od/2 + wire_clearance, half width hw = count od/2 +
        wire_clearance):

        * exit ``ml_z_exit`` = ``ml_z_run`` = the rim z (z_edf_housing_bottom, drawing data) - bundle.gap_to_te_ASSUMED -
          Db/2: the bundle's top lies gap under the stator trailing edge / rim, its centre inside the motor's front ring band
          (``ml_ring_z`` = rim .. rim - front_ring_w_ASSUMED, +- Db/2) - else Layout raises;
        * straight run S radially from the motor side (r motor_d / 2) behind the stator vane, under the rim, to the bend start
          ``ml_u_slot`` - R; the heat-shrink covers the first shrink_len_from_motor_ASSUMED of it (``ml_u_shrink_end``; fix
          round: an 18 mm re-sleeve, assembly spec), then the leads go from the triangle to a row side by side across the
          slot over ``ml_transition_len`` (E) and run as single leads to the bend;
        * bend b2 up (centre (u_slot - R, z_run + R)) into the slot beside the housing at ``ml_u_slot``.

        ``ml_u_slot`` is the largest radius at which the printed lead notch / lower slot (``ml_low_cut_uz``: a (u, z) polygon
        prism low_w wide from duct_r - 0.5 up into the 22 mm box, its bottom low_cut_margin under the bundle's envelope, a 45
        deg chamfer low_cut_margin outside every envelope - bundle and single leads - of the route) keeps cone_wall +
        skin_min_wall to the cone skin at its chamfer top (the 22 mm box's rule).  ``ml_shrink_bend_margin`` = bend start -
        (heat-shrink end + transition) (< 0: the stiff bundle / unformed leads take the first part of the bend; the
        envelopes then carry the bundle's size into the bend; C29 WARNs).  ``ml_exit_angle_tol_deg`` (E) is the exit-angle
        error the single leads can steer out before the bend (C29 WARNs while the angle is not measured and the tolerance
        is below edf.wire_exit.motor_exit.theta_uncertainty_deg_ASSUMED).  ``ml_lower_clear`` holds the clearances (thrust-vane sweep, hinge boss, servo fairing,
        fin root, rotor / stator LE, rim corner, rim -> bundle top, housing OD, notch skin) and raises when one is short; the
        frontal blockage of the bundle in the jet is ``ml_blockage`` and its drag model (area, centroid, Cd) ``ml_jet_drag``
        (used by C33 and sizing/vtol_sizing.py).  Raises on every infeasible geometry."""
        e, ml, pm = p["edf"], p["motor_leads"], p["propulsion_module"]
        we, lr, ws = e["wire_exit"], ml["lower_route"], pm["wire_slot"]
        me, sv = we["motor_exit"], we["stator_vane"]
        if we.get("housing_exit", "rear_rim") != "rear_rim":
            raise ValueError("edf.wire_exit.housing_exit: only 'rear_rim' is modelled")
        if me.get("level") != "front_ring":
            raise ValueError("edf.wire_exit.motor_exit.level: only the photo-based 'front_ring' exit is modelled (round 1's "
                             "climb along the motor can, z_above_motor_bottom_ASSUMED, is superseded - history/v0.9_wire_round1)")
        bu = lr["bundle"]
        db, gap, l_sh = float(bu["od_ASSUMED"]), float(bu["gap_to_te_ASSUMED"]), float(bu["shrink_len_from_motor_ASSUMED"])
        if db < (1.0 + 2.0 / math.sqrt(3.0)) * od - 1e-9:
            raise ValueError("motor_leads.lower_route.bundle.od_ASSUMED is smaller than the three leads in a triangle")
        if gap < 0.0 or l_sh < 0.0:
            raise ValueError("motor_leads.lower_route.bundle: gap_to_te / shrink_len must be >= 0")
        h, hw = od / 2 + wcl, 0.5 * n * od + wcl
        hb = db / 2 + wcl
        R = float(lr["bend_r_centre_ASSUMED"])
        if R < h + 1e-9:
            raise ValueError("motor_leads.lower_route.bend_r_centre_ASSUMED must exceed od / 2 + wire_clearance")
        # fix round (review 2026-10-29 G2 / G3): after the heat-shrink the three leads leave as a triangle (or stacked)
        # and must lie SIDE BY SIDE across the slot (tangential row) before the bend - in the bend each lead needs the
        # centre-line radius R itself.  Triangle -> row moves every lead by d = od / sqrt 3 (E: the apex to the middle, the
        # base leads out to +-od); an S-bend of offset d with radius R needs the straight length 2 sqrt(R d - d^2 / 4)
        # (``ml_transition_len``, E).  Over that transition the envelope is the union of the bundle's (+-hb) and the
        # row's (+-hw x +-h): the bundle envelope runs to ``ml_u_bundle_env_end`` = shrink end + transition
        d_t = od / math.sqrt(3.0)
        l_t = 2.0 * math.sqrt(max(0.0, R * d_t - 0.25 * d_t * d_t))
        self.ml_transition_offset, self.ml_transition_len = d_t, l_t
        l_be = l_sh + l_t
        self.ml_bend_r, self.ml_h, self.ml_hw, self.ml_bundle_d, self.ml_hb = R, h, hw, db, hb
        self.ml_r_motor = rm = e["motor_d"] / 2
        r_ho, r_hi = e["housing_od"] / 2, e["housing_id"] / 2
        z_hb = self.z_edf_housing_bottom
        self.ml_z_motor_bottom = z_hb - e["motor_protrusion_rear"]
        ring_w = float(me["front_ring_w_ASSUMED"])
        self.ml_ring_z = (z_hb - ring_w, z_hb)
        # stator vane (EDF's own): in line with the exit, trailing edge at the rear rim + te_inset (photos: at the rim)
        self.ml_stator_theta = self.tab_thetas[0] + sv["theta_from_tab_deg_ASSUMED"]
        if abs(((self.ml_stator_theta - self.wire_theta) + 180.0) % 360.0 - 180.0) > 1e-9:
            raise ValueError("motor leads: the route model needs the stator vane in line with the lead exit (wire_theta)")
        inset = float(sv["te_inset_from_housing_rear_ASSUMED"])
        if inset < 0.0:
            raise ValueError("stator vane trailing edge below the housing rim: a different route (not modelled)")
        self.ml_stator_te_z = z_hb + inset
        self.ml_stator_le_z = self.ml_stator_te_z + sv["chord_ASSUMED"]
        self.ml_stator_t = sv["t_ASSUMED"]
        self.ml_stator_r = (e["hub_d_ASSUMED"] / 2, r_hi)
        # the bundle passes UNDER the rim (top gap below it) behind the vane's trailing edge; its centre is the exit height
        zr = z_hb - gap - db / 2
        self.ml_z_run = self.ml_z_exit = zr
        if not (self.ml_ring_z[0] - db / 2 - 1e-9 <= zr <= self.ml_ring_z[1] + 1e-9):
            raise ValueError(f"motor leads: the bundle centre z {zr:.2f} is not at the motor's front ring "
                             f"({self.ml_ring_z[0]:.2f}..{self.ml_ring_z[1]:.2f} +- Db/2)")
        if zr - hb < self.ml_z_motor_bottom:
            raise ValueError("motor leads: the bundle reaches below the motor's rear end")
        self.ml_u_shrink_end = rm + l_sh
        self.ml_u_bundle_env_end = rm + l_be
        # notch / lower slot cutter and the slot radius
        w, m = float(ws["low_w"]), float(ws["low_cut_margin"])
        if w / 2 < max(hw, hb) - 1e-9:
            raise ValueError("propulsion_module.wire_slot.low_w narrower than the bundle / the three leads side by side")
        skin = self.cone_wall + ws["skin_min_wall"]
        u_in = self.duct_r - 0.5
        z_t = self.wire_slot_z0 + 0.5                   # over-travels into the 22 mm box (void there)
        s2 = math.sqrt(2.0)

        def arc_max(cu: float, cz: float, a0: float, a1: float, r: float) -> float:
            """max of u - z on the arc of radius r about (cu, cz) for phi in [a0, a1] (rad, -pi/2 = bend start)."""
            ph = min(max(-0.25 * math.pi, a0), a1)
            return cu - cz + r * (math.cos(ph) - math.sin(ph))

        def geom(u_v: float) -> dict:
            cu, cz = u_v - R, zr + R
            ls = cu - rm                                 # straight length motor side -> bend start
            if l_be <= ls:                               # shrink + transition end on the straight: the bundle envelope's
                kb = (rm + l_be) - (zr - hb)             # lower outer corner (the single-lead row continues from it)
                kw = arc_max(cu, cz, -0.5 * math.pi, 0.0, R + h)
                phs = -0.5 * math.pi
            else:                                        # shrink / transition runs into the bend up to phs
                phs = -0.5 * math.pi + (l_be - ls) / R
                if phs >= 0.0:
                    return {"u_v": u_v, "g": -math.inf, "bad": "the heat-shrink bundle reaches the vertical slot"}
                # the bundle's outer edge (radius R + hb) up to its square-cut end at phs (the end's lower outer corner)
                kb = max(cu - (zr - hb), arc_max(cu, cz, -0.5 * math.pi, phs, R + hb))
                kw = arc_max(cu, cz, phs, 0.0, R + h)
            k_ = max(kb, kw) + m * s2                    # 45 deg chamfer u - z = k_ outside every envelope + m
            u_o = u_v + h + m
            z_ct = u_o - k_
            z_b = zr - hb - m
            u_b = k_ + z_b
            g = self.cone_outer_radius(z_ct) - skin - math.hypot(u_o, w / 2)
            return {"u_v": u_v, "c": (cu, cz), "phs": phs, "u_o": u_o, "z_ct": z_ct, "z_b": z_b, "u_b": u_b, "g": g,
                    "k": k_}

        u_lo, u_hi = r_ho + h, self.wire_slot_lower_r_out - h
        best = None
        k = 0
        while u_hi - 0.001 * k >= u_lo - 1e-12:          # largest feasible radius, 1 um steps (deterministic)
            gm = geom(round(u_hi - 0.001 * k, 3))
            if gm["g"] >= 0.0:
                best = gm
                break
            k += 1
        if best is None:
            raise ValueError("motor leads: no slot radius beside the housing keeps cone_wall + skin_min_wall to the skin")
        g_ = best
        us = self.ml_u_slot = g_["u_v"]
        cu, cz = g_["c"]
        if cu < rm + 2 * od - 1e-9:
            raise ValueError("motor leads: no straight run behind the stator vane before the bend")
        if g_["u_b"] <= u_in + 1e-9 or not (g_["z_b"] < g_["z_ct"] < z_t):
            raise ValueError("motor-lead notch polygon degenerate")
        if g_["z_b"] < self.z_vane_section_top + 2.0:
            raise ValueError("motor-lead notch reaches the vane section")
        # bend start - end of the shrink + transition (< 0: the bundle / unformed leads take the first part of the bend -
        # C29 WARNs); ml_shrink_end_margin = bend start - shrink end (the straight single-lead run before the bend)
        self.ml_shrink_bend_margin = cu - self.ml_u_bundle_env_end
        self.ml_shrink_end_margin = cu - self.ml_u_shrink_end
        self.ml_shrink_bend_phi_deg = math.degrees(g_["phs"]) + 90.0     # bend angle the bundle envelope takes (0 = none)
        # fix round (review 2026-10-29 G1): exit-angle tolerance (E).  The bundle (bonded to the stator vane, rigid) turns
        # with the EDF; if the exit is dtheta off the notch plane the single leads must steer sideways by cu sin(dtheta) at
        # the bend start within the straight run after the shrink (length l_f = cu - shrink end), together with the
        # triangle -> row move (the outermost base lead: od/2 across + od/(2 sqrt 3) along z), S-bend radius >= R: largest
        # offset d_max = 2R - 2 sqrt(R^2 - (l_f/2)^2) (l_f < 2R); plus the row's slack in the notch (low_w/2 - hw).  The
        # bundle's envelope end (``ml_u_bundle_env_end``) lies inside duct_r, so the rigid part never meets the module
        l_f = max(0.0, cu - self.ml_u_shrink_end)
        d_max = (2 * R - 2 * math.sqrt(R * R - 0.25 * l_f * l_f)) if l_f < 2 * R else 2 * R + (l_f - 2 * R)
        zz = od / (2.0 * math.sqrt(3.0))
        steer = max(0.0, math.sqrt(max(0.0, d_max * d_max - zz * zz)) - od / 2) + max(0.0, w / 2 - hw)
        self.ml_exit_angle_tol_deg = math.degrees(math.asin(min(1.0, steer / cu)))
        self.ml_exit_steer_mm = steer
        self.ml_bundle_env_in_duct = self.ml_u_bundle_env_end < self.duct_r - 1e-9
        self.ml_theta_measured = bool(me.get("theta_measured", False))
        self.ml_theta_uncert_deg = float(me.get("theta_uncertainty_deg_ASSUMED", 0.0))
        self.ml_low_cut_w = w
        self.ml_low_cut_uz = [(u_in, g_["z_b"]), (g_["u_b"], g_["z_b"]), (g_["u_o"], g_["z_ct"]), (g_["u_o"], z_t),
                              (u_in, z_t)]
        self.ml_low_cut_skin = self.cone_outer_radius(g_["z_ct"]) - math.hypot(g_["u_o"], w / 2)
        self.ml_bends = [("b2 outer bend (under the rim -> up the slot)", cu, cz, "lower_right")]
        pts = [(rm, zr)] + self._quarter_arc(cu, cz, R, "lower_right")
        self.ml_lower_uz = pts                          # (r_motor, z_run) ... (u_slot, z_run + R)
        arc = 0.5 * math.pi * R
        run = cu - rm
        self.ml_lower_len = run + arc
        self.ml_lower_segments = {"S run behind the stator vane (heat-shrink bundle, then single leads)": run, "b2": arc}
        # restraints: the motor exit (s = 0), the RTV bond of the bundle to the stator vane's trailing edge (lower_route.
        # restraint 'rtv_bond_stator_te': no tie round the vane - it would loop over the leading edge next to the rotor),
        # the notch entry.  The bond covers the straight run inside the vane's span (hub radius .. housing bore) up to the
        # notch entry; free spans = exit -> bond start, bond end -> notch entry
        if lr.get("restraint", "rtv_bond_stator_te") != "rtv_bond_stator_te":
            raise ValueError("motor_leads.lower_route.restraint: only 'rtv_bond_stator_te' is modelled (a tie round the "
                             "stator vane needs the rotor-stator gap, not in params)")
        span = float(lr["max_free_span_jet_ASSUMED"])
        u_b0 = max(rm, self.ml_stator_r[0])
        u_b1 = min(cu, self.ml_stator_r[1], u_in)
        if u_b1 - u_b0 < 2 * db - 1e-9:
            raise ValueError("motor leads: the straight run along the stator vane is too short to bond (< 2 bundle OD)")
        s_notch = (u_in - rm) if u_in <= cu else (run + R * math.asin(min(1.0, (u_in - cu) / R)))
        self.ml_bond_u, self.ml_bond_s = (u_b0, u_b1), (u_b0 - rm, u_b1 - rm)
        self.ml_lower_max_free_span = span
        self.ml_lower_free_spans = [u_b0 - rm, s_notch - (u_b1 - rm)]
        if max(self.ml_lower_free_spans) > span + 1e-9:
            raise ValueError(f"motor leads: free span along the stator route {max(self.ml_lower_free_spans):.1f} mm > "
                             f"max_free_span_jet_ASSUMED {span:g}")
        # clearances (mm, wire_theta plane; the vane / fin / fairing at wire_theta lie in the same plane - vertical
        # distances are lower bounds of the true 3D gaps).  Dense centre-line samples carry their envelope half: hb on the
        # heat-shrink (arc length <= shrink_len), h after it
        v_ = p["vane"]
        d_le = self.vane_le_z - self.vane_hinge_z
        sweep_top = self.vane_hinge_z + math.hypot(d_le, self.vane_t / 2)     # max over any deflection (LE corner)
        fair_top = self.fair_z[1] if getattr(self, "fair_z", None) else self.vane_hinge_z
        fair_u0 = self.fair_u[0] if getattr(self, "fair_u", None) else self.duct_r
        dense: list[tuple[float, float, float, float]] = []      # (u, z, envelope half incl. wcl, half without wcl)
        k_s = max(1, int(math.ceil(run / 0.25)))
        for j in range(k_s + 1):
            s_ = run * j / k_s
            big_ = s_ <= l_be + 1e-9
            dense.append((rm + s_, zr, hb if big_ else h, db / 2 if big_ else od / 2))
        k_a = max(1, int(math.ceil(arc / 0.25)))
        for j in range(1, k_a + 1):
            ph = -0.5 * math.pi + 0.5 * math.pi * j / k_a
            big_ = run + R * (ph + 0.5 * math.pi) <= l_be + 1e-9
            dense.append((cu + R * math.cos(ph), cz + R * math.sin(ph), hb if big_ else h, db / 2 if big_ else od / 2))
        self.ml_lower_dense = dense

        def rect_gap(u0: float, u1: float, z0: float, z1: float) -> float:
            """Smallest distance from the route envelope (centre line +- its half) to the (u, z) rectangle."""
            return min(math.hypot(max(u0 - u, 0.0, u - u1), max(z0 - z, 0.0, z - z1)) - hh for u, z, hh, _ in dense)

        big = 1e3
        corner = min(math.dist((r_ho, z_hb), (u, z)) - h0 for u, z, _, h0 in dense)
        self.ml_lower_clear = {
            "thrust-vane sweep (any deflection) -> leads": rect_gap(self.vane_r_in, self.vane_r_out, -big, sweep_top),
            "hinge boss -> leads": rect_gap(self.duct_r, self.hinge_boss_r_out, -big,
                                            self.vane_hinge_z + self.hinge_boss_od / 2),
            "servo fairing / pocket / coupler -> leads": rect_gap(fair_u0, big, -big, fair_top),
            "fin (root LE) -> leads": rect_gap(self.fin_root_r, big, -big, self.fin_root_le_z),
            "fin root LE -> notch bottom": g_["z_b"] - self.fin_root_le_z,
            "duct wall below the notch -> leads": rect_gap(self.duct_r, big, -big, g_["z_b"]),
            "rotor (ahead of the stator LE, ASSUMED) -> bundle top": self.ml_stator_le_z - (zr + hb),
            "housing rim corner -> leads / bundle": corner,
            "housing rim -> bundle top (under the rim)": z_hb - (zr + db / 2),
            "housing OD -> leads in the slot": us - od / 2 - r_ho,
            "motor rear end -> bundle bottom": (zr - db / 2) - self.ml_z_motor_bottom,
            "notch outer corner -> cone skin": self.ml_low_cut_skin,
        }
        self.ml_stator_te_gap = self.ml_stator_te_z - (zr + db / 2)
        self.ml_max_deflection_deg = v_.get("max_deflection_mech_deg")
        need = {"housing rim corner -> leads / bundle": wcl, "housing OD -> leads in the slot": wcl,
                "notch outer corner -> cone skin": skin}
        for kk, vv in self.ml_lower_clear.items():
            if vv < need.get(kk, 0.0) - 1e-6:
                raise ValueError(f"motor leads (lower route): {kk} {vv:.2f} mm < {need.get(kk, 0.0):g}")
        if self.ml_stator_te_gap > lr["bond_gap_max_ASSUMED"] + 1e-9:
            raise ValueError(f"motor leads: the bundle lies {self.ml_stator_te_gap:.2f} mm behind the vane's trailing "
                             f"edge > bond_gap_max_ASSUMED {lr['bond_gap_max_ASSUMED']:g} (not 'along the vane'; the RTV "
                             f"bond cannot bridge it)")
        # frontal area in the jet (photo round, E): ONE bundle O db radially behind the stator vane from the hub radius
        # (inside it: the hub's shadow, hub_d ASSUMED) to duct_r; single leads side by side (count x od) only where the
        # heat-shrink ends inside duct_r.  No wake credit for the 2 mm vane ahead (conservative).  D = Cd q A
        r_hub = max(self.ml_stator_r[0], rm)
        a_duct = math.pi * (self.duct_r ** 2 - rm ** 2)
        u_e = min(max(self.ml_u_shrink_end, r_hub), self.duct_r)
        l_b, l_w = u_e - r_hub, self.duct_r - u_e
        a_b, a_w = db * l_b, n * od * l_w
        a_drag = a_b + a_w
        u_c = (a_b * 0.5 * (r_hub + u_e) + a_w * 0.5 * (u_e + self.duct_r)) / a_drag
        self.ml_blockage = {"annulus_mm2": a_duct, "projected_mm2": a_drag, "projected_pct": 100 * a_drag / a_duct,
                            "drag_area_mm2": a_drag, "drag_area_pct": 100 * a_drag / a_duct,
                            "bundle_len_in_jet_mm": l_b, "single_leads_len_in_jet_mm": l_w}
        jd = lr["jet_drag"]
        cx_, cy_, _ = self.ml_xyz(u_c, 0.0, zr)
        self.ml_jet_drag = {"cd_ASSUMED": float(jd["cd_ASSUMED"]), "cd_band_ASSUMED": list(jd["cd_band_ASSUMED"]),
                            "area_mm2": a_drag, "centroid_u_mm": u_c, "xy_mm": (cx_, cy_)}
        # sections (have vs need) of the printed passages the route uses
        bundle = (db + 2 * wcl, db + 2 * wcl)
        three = (n * od + 2 * wcl, od + 2 * wcl)
        self.ml_sections["lead notch under the housing rim: heat-shrink bundle"] = {
            "have_w": w, "have_h": z_hb - g_["z_b"], "need_w": bundle[0], "need_h": bundle[1],
            "margin_w": w - bundle[0], "margin_h": z_hb - g_["z_b"] - bundle[1]}
        self.ml_sections["lower slot beside the housing: leads side by side"] = {
            "have_w": w, "have_h": g_["u_o"] - r_ho, "need_w": three[0], "need_h": three[1],
            "margin_w": w - three[0], "margin_h": g_["u_o"] - r_ho - three[1]}

    def motor_lead_lower_polyline(self) -> list[tuple[float, float, float]]:
        """v0.9 lead-exit round: the EDF-internal centre line (exit -> slot), vehicle frame."""
        return [self.ml_xyz(u, 0.0, z) for u, z in getattr(self, "ml_lower_uz", [])]

    def _motor_leads(self, p: dict) -> None:
        """v0.9 (user 2026-10-27, decision 3): the motor-lead path for ``motor_leads.count`` x 12 AWG (OD od_ASSUMED) with
        5.5 mm bullets (OD bullet.d_ASSUMED, bullet.len_ASSUMED per side).  Cross-sections (``ml_sections``): the three
        bare leads side by side with wire_clearance in the propulsion slot (laid in from its open top) and the groove; a
        bullet + the other leads beside it with bullet_clearance in every passage a bullet goes through - the groove with
        the cover on (vertical), the gate over the lip (radial), the bulkhead hole (vertical).  Route polyline
        (``motor_lead_polyline``: housing exit -> slot -> step out in the skirt -> groove -> gate -> along the strut ->
        bulkhead hole -> the joint zone), its length and the lumped mass (wire + slack + bullet pairs).  The bullet joint
        zone (``ml_joint_*``) in the avionics bay clears every bay box by com_mount.min_clear_to_tray and the bore by
        PLACEHOLDER_BORE_CLEAR_MM.  Raises on any shortfall."""
        ml = p["motor_leads"]
        n, od = int(ml["count"]), ml["od_ASSUMED"]
        self.ml_od_nominal = od
        if self.clock_mount:
            # wire09c: every passage the single leads use is checked with the THICKEST lead of the generosity box
            # (motor_leads.generosity_box.od, user 2026-10-30: 12 AWG up to 4.3 mm, not measured)
            od = max(float(x) for x in ml["generosity_box"]["od"])
        bd, bl, bc, wcl = ml["bullet"]["d_ASSUMED"], ml["bullet"]["len_ASSUMED"], ml["bullet_clearance"], ml["wire_clearance"]
        self.ml_count, self.ml_od, self.ml_bullet_d, self.ml_bullet_len = n, od, bd, bl
        self.ml_bullet_clear, self.ml_wire_clear = bc, wcl
        r_h = p["edf"]["housing_od"] / 2
        three = (n * od + 2 * wcl, od + 2 * wcl)                       # bare leads side by side (w, h)
        bullet = (bd + 2 * bc + (n - 1) * od, bd + 2 * bc)             # bullet + the other leads beside it (w, h)
        # the bullets are threaded STRAIGHT UP through the intake (held above the propulsion module) at the bulkhead hole's
        # mid radius: the tightest radial room on that line is the lip-pocket wall outside it and the bulkhead hole's
        # inner edge inside it; tangentially the shelf notch (groove width) lets the bullet past the shelf
        u_in = 0.5 * (self.wire_hole_r0 + self.wire_hole_r1)
        r_pass = 2 * min(self.lip_pocket_id / 2 - u_in, u_in - self.wire_hole_r0)
        sec0 = (("lead passage (fan, clock mount): widest bundle / lead stack, any orientation",
                 ((2 * self.lp_hw, self.lp_r_wall - r_h), (2 * self.lp_rho_t_max, 2 * self.lp_rho_t_max)))
                if self.clock_mount else
                ("propulsion slot: leads side by side", ((self.wire_slot_w, self.wire_slot_lower_r_out - r_h), three)))
        sec = {sec0[0]: sec0[1],
               "intake groove: leads side by side": ((self.wire_ch_w, self.wire_ch_r1 - self.wire_ch_r0), three),
               "gate over the lip: leads side by side (laid in)": ((self.wire_gate_w, self.wire_ch_gate_h), three),
               "intake groove, cover on: bullet + leads": ((self.wire_ch_w, self.wire_ch_r1 - self.wire_ch_r0), bullet),
               "intake interior (straight up at the hole radius): bullet + leads": ((self.wire_ch_w, r_pass), bullet),
               "bulkhead hole: bullet + leads": ((self.wire_hole_w, self.wire_hole_r1 - self.wire_hole_r0), bullet)}
        self.ml_sections = {k: {"have_w": h[0], "have_h": h[1], "need_w": nd[0], "need_h": nd[1],
                                "margin_w": h[0] - nd[0], "margin_h": h[1] - nd[1]} for k, (h, nd) in sec.items()}
        for k, d in self.ml_sections.items():
            if min(d["margin_w"], d["margin_h"]) < -1e-9:
                raise ValueError(f"motor leads: {k} {d['have_w']:.2f} x {d['have_h']:.2f} mm < {d['need_w']:.2f} x "
                                 f"{d['need_h']:.2f}")
        # route (motor-lead frame u, v = 0, Z)
        if self.clock_mount:
            # wire09c clock mount: the factory bundle from the motor's front ring under the rim, out through the lead
            # passage to its outer wall, up along it, steered to u_top into the intake groove - every corner of
            # motor_leads.generosity_box (Layout._motor_lead_clock); u_slot = u_top (the leads enter the groove there)
            self._motor_lead_clock(p, n, od, wcl)
            u_slot = self.ml_u_slot
        elif getattr(self, "wire_exit_motor", False):
            # v0.9 lead-exit, photo round (user photos 2026-10-29): motor side at its front ring -> heat-shrink bundle
            # behind the stator vane's trailing edge -> under the housing's rear rim -> single leads bend up the slot at
            # ml_u_slot
            self._motor_lead_lower(p, n, od, wcl)
            u_slot = self.ml_u_slot
        else:
            zs = self.wire_exit_z_from_lip_front
            self.ml_z_exit = self.z_edf_lip_top - 0.5 * (zs[0] + zs[1])
            u_slot = 0.5 * (r_h + self.wire_slot_lower_r_out)
        z_out = 0.5 * (self.z_intake_bottom + self.z_intake_skirt_top)
        u_g = 0.5 * (self.wire_ch_r0 + self.wire_ch_r1)
        z_gate = 0.5 * (self.z_lip_pocket_top + self.wire_ch_z1)
        u_in = 0.5 * (self.wire_hole_r0 + self.wire_hole_r1)
        # v0.9 fix round (motor_leads.plenum, wire_channel.tie_anchors): between the gate and the bulkhead the bare leads
        # run on the outer side of the bulkhead hole, tied to anchors on the 0 deg strut, off the EDF throat radius
        pl_ = ml.get("plenum")
        ta_ = (p["intake_module"]["wire_channel"] or {}).get("tie_anchors")
        self.ml_anchor_z: list[float] = []
        if pl_ and ta_ and getattr(self, "wire_ch_open", False):
            u_in = self.wire_hole_r1 - wcl - od / 2
            self.ml_anchor_u = u_in + od / 2 + wcl                  # anchors' inner face = the leads' outer line
            self.ml_anchor_w, self.ml_anchor_h = ta_["w"], ta_["h"]
            self.ml_anchor_wedge = ta_["wedge_deg"]
            self.ml_anchor_tunnel = (ta_["tunnel_t"], ta_["tunnel_h"])
            self.ml_anchor_wall, self.ml_anchor_into = ta_["wall"], ta_["into_band"]
            self.ml_anchor_u1 = self.r_band_in + ta_["into_band"]
            mf_ = p["printer"]["min_feature"]
            if self.ml_anchor_wedge < 45.0 - 1e-9:
                raise ValueError("tie_anchors.wedge_deg < 45: the anchor roof would need support (intake prints flipped)")
            if self.ml_anchor_u > self.r_band_in - (ta_["wall"] + ta_["tunnel_t"] + mf_) + 1e-9:
                raise ValueError("tie anchor too shallow for its tunnel (leads' outer line too close to the strut)")
            if self.ml_anchor_w / 2 > self.strut_w_in / 2 - 1e-9:
                raise ValueError("tie anchor wider than the 0 deg strut's inner face")
            # the anchor's tangential half width must stay inside the strut sector at the anchor's inner face too
            half_sector = self.ml_anchor_u * math.tan(0.5 * self.strut_w / self.body_r)
            self.ml_anchor_sector_margin = half_sector - self.ml_anchor_w / 2
            if self.ml_anchor_sector_margin < 0.0:
                raise ValueError("tie anchor reaches past the strut sector into a window")
            self.ml_throat_clear = (u_in - od / 2) - p["edf"]["housing_id"] / 2
            if self.ml_throat_clear < pl_["throat_clear_min"] - 1e-9:
                raise ValueError(f"motor leads pass {self.ml_throat_clear:.2f} mm outside the EDF throat radius "
                                 f"(< motor_leads.plenum.throat_clear_min {pl_['throat_clear_min']:g})")
            # restraints: the gate, the anchors (tie centre), the bulkhead hole; equal spans no longer than max_free_span
            z_top = self.z_intake_bulkhead_bottom
            span = z_top - z_gate
            n_a = max(0, math.ceil(span / pl_["max_free_span"] - 1e-9) - 1)
            self.ml_anchor_z = [z_gate + span * (k + 1) / (n_a + 1) for k in range(n_a)]
            stops = [z_gate] + self.ml_anchor_z + [z_top]
            self.ml_free_spans = [b - a for a, b in zip(stops[:-1], stops[1:])]
            self.ml_max_free_span = pl_["max_free_span"]
            # the anchors (roof included) stay inside the windowed band and clear of each other
            tot_h = ta_["h"] + (self.ml_anchor_u1 - self.ml_anchor_u) * math.tan(math.radians(self.ml_anchor_wedge))
            self.ml_anchor_total_h = tot_h
            for zc in self.ml_anchor_z:
                if zc - ta_["h"] / 2 < self.z_shelf_top or zc + ta_["h"] / 2 + tot_h - ta_["h"] > z_top:
                    raise ValueError("tie anchor outside the windowed band")
            if n_a > 1 and min(self.ml_free_spans[1:-1]) < tot_h + mf_:
                raise ValueError("tie anchors overlap")
            # the bullets still go straight up at the hole's mid radius: the anchors' inner face bounds that pass too
            u_mid = 0.5 * (self.wire_hole_r0 + self.wire_hole_r1)
            r_pass = 2 * min(self.lip_pocket_id / 2 - u_mid, u_mid - self.wire_hole_r0, self.ml_anchor_u - u_mid)
            d = self.ml_sections["intake interior (straight up at the hole radius): bullet + leads"]
            d["have_h"], d["margin_h"] = r_pass, r_pass - d["need_h"]
            if d["margin_h"] < -1e-9:
                raise ValueError(f"motor leads: the bullets' straight pass up the intake ({r_pass:.2f} mm between the "
                                 f"pocket wall / hole edge / tie anchors) < {d['need_h']:.2f}")
        jz = ml["joint_zone"]
        self.ml_joint_u = tuple(jz["u"])
        self.ml_joint_hv = 0.5 * (n * bd + (n - 1) * 0.5)
        self.ml_joint_z = (self.z_intake_bulkhead_top + jz["z_above_bulkhead"],
                           self.z_intake_bulkhead_top + jz["z_above_bulkhead"] + 2 * bl)
        if self.clock_mount:
            # wire09c: the LONGEST corner's centre line (motor -> bend -> wall -> steer -> u_top at z_r1), then up the
            # groove at u_top to the step-out height and the v0.9 legs (mass / length from the longest corner, E)
            self.ml_route_uz = list(self.ml_lower_uz) + [(u_slot, z_out), (u_g, z_out), (u_g, z_gate), (u_in, z_gate),
                                                         (u_in, self.ml_joint_z[0])]
            z_a0 = self.lp_z_r1                                   # leg A starts where the steer ends (u_top, z_r1)
        elif getattr(self, "wire_exit_motor", False):
            # sampled centre line of the EDF-internal part (bends as arcs) up to the slot, then the v0.9 legs
            self.ml_route_uz = list(self.ml_lower_uz) + [(u_slot, z_out), (u_g, z_out), (u_g, z_gate), (u_in, z_gate),
                                                         (u_in, self.ml_joint_z[0])]
            z_a0 = self.ml_z_run + self.ml_bend_r                 # leg A starts where the outer bend ends
        else:
            self.ml_route_uz = [(r_h, self.ml_z_exit), (u_slot, self.ml_z_exit), (u_slot, z_out), (u_g, z_out),
                                (u_g, z_gate), (u_in, z_gate), (u_in, self.ml_joint_z[0])]
            z_a0 = self.ml_z_exit
        # named stations of the printed-passage legs (rocket_geometry.motor_lead_legs)
        self.ml_key = {"u_slot": u_slot, "z_a0": z_a0, "z_out": z_out, "u_g": u_g, "z_gate": z_gate, "u_in": u_in}
        pts = [self.ml_xyz(u, 0.0, z) for u, z in self.ml_route_uz]
        segs = [(pts[i], pts[i + 1], math.dist(pts[i], pts[i + 1])) for i in range(len(pts) - 1)]
        self.ml_route_len = sum(s[2] for s in segs)
        self.ml_lead_len = self.ml_route_len + ml["slack"]
        g_m = ml["mass_g_per_m_ASSUMED"] / 1000.0
        extra = max(0.0, self.ml_lead_len - ml["stock_len_in_edf_mass_ASSUMED"])
        m_w = n * extra * g_m
        mom = [0.0, 0.0, 0.0]
        for a, b, ln in segs:
            for i in range(3):
                mom[i] += n * ln * g_m * 0.5 * (a[i] + b[i]) * (extra / self.ml_lead_len)
        jc = self.ml_xyz(0.5 * sum(self.ml_joint_u), 0.0, 0.5 * sum(self.ml_joint_z))
        m_b = n * ml["bullet"]["pair_g_ASSUMED"]
        for i in range(3):
            mom[i] += (m_b + n * ml["slack"] * g_m * (extra / self.ml_lead_len)) * jc[i]
        # review 2026-10-29 (FIND-5 / G7): the RTV bond to the stator vane, the notch seal and the chafe tape (ASSUMED),
        # lumped at the notch entry; photo round: + the stock heat-shrink over the bundle (bundle.shrink_g_ASSUMED)
        if self.clock_mount:
            # wire09c: RTV bond (optional, counted) + chafe tape + factory shrink at the bond end, the foam plug at the
            # passage top (theta_P) and the foam pads under the tabs (on the axis line of the two tabs: lateral 0)
            m_r = 0.0
            for m_, xyz_ in self.ml_clock_lumps:
                m_r += m_
                for i in range(3):
                    mom[i] += m_ * xyz_[i]
        else:
            m_r = (sum(ml["lower_route"]["rtv_g_ASSUMED"].values()) + float(ml["lower_route"]["bundle"]["shrink_g_ASSUMED"])
                   if getattr(self, "wire_exit_motor", False) else 0.0)
            if m_r > 0.0:
                rc = self.ml_xyz(self.ml_bond_u[1], 0.0, self.ml_z_run)
                for i in range(3):
                    mom[i] += m_r * rc[i]
        self.ml_rtv_g = m_r
        self.ml_mass_g = m_w + m_b + m_r
        self.ml_com = tuple(mom[i] / self.ml_mass_g for i in range(3))
        # joint zone clearances in the bay (axis-aligned box in the wire_theta frame: wire_theta is a multiple of 90 deg)
        if abs(((self.wire_theta + 45.0) % 90.0) - 45.0) > 1e-9:
            raise ValueError("motor-lead joint zone box needs wire_theta on an axis")
        c0 = self.ml_xyz(self.ml_joint_u[0], -self.ml_joint_hv, 0.0)
        c1 = self.ml_xyz(self.ml_joint_u[1], self.ml_joint_hv, 0.0)
        self.ml_joint_box = ((min(c0[0], c1[0]), max(c0[0], c1[0])), (min(c0[1], c1[1]), max(c0[1], c1[1])),
                             self.ml_joint_z)
        lim = jz["min_clear"]
        self.ml_joint_min_clear = lim
        self.ml_joint_clearance = {nm: _box_gap(self.ml_joint_box, bx) for nm, bx in self.avionics_bay_boxes().items()}
        # the zone starts in the intake spigot (its bore spigot_id is the tighter one)
        self.ml_joint_clearance["bore (radial)"] = (min(self.spigot_id, self.body_id) / 2
                                                    - math.hypot(self.ml_joint_u[1], self.ml_joint_hv))
        for nm, gv in self.ml_joint_clearance.items():
            want = PLACEHOLDER_BORE_CLEAR_MM if nm == "bore (radial)" else lim
            if gv < want - 1e-9:
                raise ValueError(f"motor-lead bullet joint zone is {gv:.2f} mm from {nm} (< {want:g})")

    def motor_lead_polyline(self) -> list[tuple[float, float, float]]:
        """v0.9 motor-lead centre line (vehicle frame): Layout.ml_route_uz in the wire_theta plane."""
        return [self.ml_xyz(u, 0.0, z) for u, z in self.ml_route_uz]

    def sl_pipe_points(self, k: int) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        """v0.9 lead pipe of fin k: (P0, Pe) - P0 at the servo pocket's inner top corner (fin theta, u = pocket inner face),
        Pe at the bottom of the v0.6 entry slot (column theta, u = the slot's mid radius, Z = vane-section top + tunnel
        radius).  The tunnel (O sl_tunnel_d) and the pipe (O sl_pipe_d) run along P0 -> Pe."""
        ue = 0.5 * (self.sl_entry_u0 + self.sl_entry_u1)
        t0, t1 = math.radians(self.fin_thetas[k]), math.radians(self.sl_col_thetas[k])
        u0, z0 = self.sl_p0_uz
        return ((u0 * math.cos(t0), u0 * math.sin(t0), z0),
                (ue * math.cos(t1), ue * math.sin(t1), self.z_vane_section_top + self.sl_tunnel_d / 2))

    def sl_web_points(self, k: int) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        """v0.9 web pipe of fin k: the lead pipe shifted sl_pipe_web toward the axis at both ends (ties it to the duct)."""
        a, b = self.sl_pipe_points(k)
        out = []
        for q in (a, b):
            r = math.hypot(q[0], q[1])
            out.append((q[0] * (r - self.sl_pipe_web) / r, q[1] * (r - self.sl_pipe_web) / r, q[2]))
        return out[0], out[1]

    def _fin_lead_pipes(self, mf: float) -> None:
        """v0.9: walls of each lead pipe's tunnel, measured on the axis segment (sampled): to the duct bore (radius duct_r),
        to the hinge bore (radial segment at the vane theta, Z hinge), to the inner cover screw's tap hole (segment along v)
        and the top of the pipe below the EDF shoulder (no pipe near the EDF bore).  Records ``sl_pipe_walls``; raises
        below printer.min_feature."""
        self.sl_pipe_walls: dict[str, float] = {}
        for k in range(len(self.fin_thetas)):
            a, b = self.sl_pipe_points(k)
            pts = [tuple(a[i] + (b[i] - a[i]) * j / 200.0 for i in range(3)) for j in range(201)]
            w_bore = min(math.hypot(q[0], q[1]) for q in pts) - self.sl_tunnel_d / 2 - self.duct_r
            hb0, hb1 = self.fin_xyz(k, self.duct_r, 0.0, self.vane_hinge_z), self.fin_xyz(k, self.dd_pocket_u[0], 0.0,
                                                                                            self.vane_hinge_z)
            w_hinge = _seg_seg_dist(a, b, hb0, hb1) - self.sl_tunnel_d / 2 - self.hinge_bore_d / 2
            us, zs = self.cov_screws_uz[0]
            ss = self.dd_service_side
            t0 = self.fin_xyz(k, us, ss * self.fair_hv, zs)
            t1 = self.fin_xyz(k, us, ss * (self.fair_hv - self.cov_tap_depth), zs)
            w_tap = _seg_seg_dist(a, b, t0, t1) - self.sl_tunnel_d / 2 - self.cov_d_tap / 2
            top = b[2] + self.sl_pipe_d / 2
            for nm, wv in ((f"fin {k + 1}: tunnel -> duct bore", w_bore), (f"fin {k + 1}: tunnel -> hinge bore", w_hinge),
                           (f"fin {k + 1}: tunnel -> inner cover-screw tap hole", w_tap),
                           (f"fin {k + 1}: pipe top -> EDF shoulder", self.z_edf_shoulder - top)):
                self.sl_pipe_walls[nm] = wv
                if wv < mf - 1e-9:
                    raise ValueError(f"servo lead pipe: {nm} {wv:.2f} mm < printer.min_feature {mf:g}")

    def servo_lead_polyline(self, k: int) -> list[tuple[float, float, float]]:
        """3D route of lead k (mm) along the ribbon's centre line, from the servo case end face to the board with the
        PWM outputs: window, up beside the pod, crossing, entry slot, straight column, deck notch, rebate arc (1 deg
        steps), step out, channel, step in, strut / bulkhead hole - then three legs ESTIMATED in the avionics bay.
        v0.6.3 (``has_fcu``: the FCU drives the servos and lies on the bulkhead): up from the hole to the FCU's top face,
        across to the point of its footprint nearest the hole, on to the middle of its top face (the servo header's
        position on the board is not known).  Without an FCU (params before v0.6.3) the v0.6 legs to the flight
        computer on the tray: to the gap between the bulkhead and the tray, across under the tray to the board's outer
        face, up to its bottom."""
        sl = self.p["servo_leads"]

        def pol(u: float, v: float, th: float, z: float) -> tuple[float, float, float]:
            t = math.radians(th)
            return (u * math.cos(t) - v * math.sin(t), u * math.sin(t) + v * math.cos(t), z)

        thc, tg = self.sl_col_thetas[k], self.sl_gather_theta
        ue = 0.5 * (self.sl_entry_u0 + self.sl_entry_u1)
        un = 0.5 * (self.sl_notch_u0 + self.spigot_od / 2)
        zr = 0.5 * (self.sl_rebate_z0 + self.z_prop_spigot_top)
        rr = 0.5 * (self.sl_rebate_r0 + self.spigot_od / 2)
        cch = 0.5 * (self.sl_ch_r0 + self.sl_ch_r1)
        zin = 0.5 * (self.z_lip_pocket_top + self.sl_ch_z1)
        chole = 0.5 * (self.sl_hole_r0 + self.sl_hole_r1)
        if self.direct_drive:
            # v0.9: case upper end face (near the case bottom) -> up into the connector bay -> along the connector to its
            # middle (the MG90S lead ends here; the extension starts) -> to its inner end -> the pocket's inner top corner
            # (pipe P0) -> pipe -> bottom of the entry slot (Pe) -> the v0.6 route from the slot
            thf = self.fin_thetas[k]
            zcn = 0.5 * (self.dd_conn_z[0] + self.dd_conn_z[1])
            u_exit = self.dd_conn_u[1]
            p0, pe = self.sl_pipe_points(k)
            pts = [pol(u_exit, 0.0, thf, self.dd_z_case[1]), pol(u_exit, 0.0, thf, zcn),
                   pol(0.5 * (self.dd_conn_u[0] + self.dd_conn_u[1]), 0.0, thf, zcn), pol(self.dd_conn_u[0], 0.0, thf, zcn),
                   p0, pe, pol(ue, 0.0, thc, self.sl_entry_z_open),
                   pol(un, 0.0, thc, self.z_prop_spigot_bottom - self.body_wall), pol(un, 0.0, thc, zr)]
        else:
            thp = self.pod_thetas[k]
            zw = 0.5 * (self.sl_win_z0 + self.sl_win_z1)
            v_out = self.sl_side * (self.pod_w / 2 + sl["outside_gap"] + self.sl_ribbon_t / 2)
            zx = 0.5 * (self.sl_cross_z0 + self.sl_cross_z1)
            pts = [pol(self.servo_axis_r, self.sl_side * self.p["servo"]["body_l"] / 2, thp, zw),
                   pol(self.servo_axis_r, v_out, thp, zw), pol(self.servo_axis_r, v_out, thp, zx),
                   pol(ue, 0.0, thc, zx), pol(ue, 0.0, thc, self.sl_entry_z_open),
                   pol(un, 0.0, thc, self.z_prop_spigot_bottom - self.body_wall), pol(un, 0.0, thc, zr)]
        off = ((thc - tg + 180.0) % 360.0) - 180.0
        n = max(1, int(math.ceil(abs(off))))
        pts += [pol(rr, 0.0, tg + off * (1.0 - i / n), zr) for i in range(1, n + 1)]
        pts += [pol(cch, 0.0, tg, zr), pol(cch, 0.0, tg, zin), pol(chole, 0.0, tg, zin),
                pol(chole, 0.0, tg, self.z_intake_bulkhead_top)]
        x_h, y_h = pts[-1][0], pts[-1][1]
        if self.has_fcu:
            z_top = self.fcu_z[1]
            x_e = min(max(x_h, self.fcu_x[0]), self.fcu_x[1])
            y_e = min(max(y_h, self.fcu_y[0]), self.fcu_y[1])
            pts += [(x_h, y_h, z_top), (x_e, y_e, z_top), (self.fcu_center[0], self.fcu_center[1], z_top)]
            return pts
        z_under = 0.5 * (self.z_intake_bulkhead_top + self.z_tray_bottom)
        pts += [(x_h, y_h, z_under), (self.nav_x[1], 0.0, z_under), (self.nav_x[1], 0.0, self.nav_z[0])]
        return pts

    def _band_step_in_check(self, name: str, notch_r0: float, ch_r0: float, ch_r1: float, w: float, z1: float) -> None:
        """v0.6.1: a lead channel's shelf notch (r from notch_r0 to past ch_r0, Z up to z1) and channel (r ch_r0..ch_r1)
        must open the whole tangential width w through the windowed band: the notch starts inside
        the band's inner face at the cut's side faces (u = sqrt(r_band_in^2 - (w/2)^2)) and the channel ends outside the
        body OD.  Records the margins in ``band_step_in`` (raises when either is not positive)."""
        if not hasattr(self, "band_step_in"):
            self.band_step_in: dict[str, dict[str, float]] = {}
        u_face = math.sqrt(self.r_band_in ** 2 - (w / 2) ** 2)
        rec = {"notch_inside_band_face": u_face - notch_r0, "channel_past_body_od": ch_r1 - self.body_r,
               "band_opened_h": max(0.0, z1 - self.z_shelf_top), "gate_h": z1 - self.z_lip_pocket_top}
        self.band_step_in[name] = rec
        if rec["notch_inside_band_face"] <= 0.0:
            raise ValueError(f"{name} shelf notch (r from {notch_r0:.2f}) starts outside the intake band's inner face "
                             f"(r {u_face:.2f} at |v| {w / 2:g}): the step-in would be blocked by the band")
        if rec["channel_past_body_od"] <= 0.0:
            raise ValueError(f"{name} channel (r {ch_r0:.2f}..{ch_r1:.2f}) does not open past the body OD {self.body_r:g}")

    def avionics_bay_boxes(self) -> dict[str, tuple]:
        """Axis-aligned boxes ((x0, x1), (y0, y1), (z0, z1)) of everything in the avionics bay that a box hanging under
        the payload bulkhead must clear: the tray, the tray placeholders, (v0.6.3) the FCU board on the intake bulkhead
        and the four rail ribs (out to the bore)."""
        boxes = {
            "avionics_tray": ((self.tray_x - self.tray_t / 2, self.tray_x + self.tray_t / 2),
                              (-self.tray_w / 2, self.tray_w / 2), (self.z_tray_bottom, self.z_tray_top)),
            "battery": (self.battery_x, self.battery_y, self.battery_z),
            "esc": (self.esc_x, self.esc_y, self.esc_z),
            **({"nav": (self.nav_x, self.nav_y, self.nav_z)} if self.has_nav else {}),
            "misc_electronics": (self.misc_x, self.misc_y, self.misc_z),
        }
        if not self.has_battery:          # v0.9 CAD-2 VTOL 'tether': the pack is removed
            del boxes["battery"]
        if getattr(self, "buffer_z", None):   # v0.9 CAD-2 VTOL 'hybrid': the buffer pack in the battery slot
            del boxes["battery"]
            boxes["buffer_pack"] = (self.buffer_x, self.buffer_y, self.buffer_z)
        if getattr(self, "has_fcu", False):
            boxes[self.fcu_part] = (self.fcu_x, self.fcu_y, self.fcu_z)
        for nm, (bx_, by_, bz_, _m) in getattr(self, "tether_boxes", {}).items():   # v0.9 fix round: the tether kit
            boxes[nm] = (bx_, by_, bz_)
        if getattr(self, "has_gns", False) and hasattr(self, "gns_boss_boxes"):   # v0.8.1: the GNS board and its bosses
            boxes["gns"] = (self.gns_x, self.gns_y, self.gns_z)
            for k, bb in enumerate(self.gns_boss_boxes):
                boxes[f"gns boss {k + 1}"] = bb
        for sy in (1.0, -1.0):
            y_in, y_wall = sy * (self.rail_y - self.rail_depth), sy * self.body_id / 2
            for sx in (1.0, -1.0):
                xc = self.tray_x + sx * (self.rail_gap / 2 + self.rail_w / 2)
                boxes[f"rail x{xc:g} y{sy:+.0f}"] = ((xc - self.rail_w / 2, xc + self.rail_w / 2),
                                                     (min(y_in, y_wall), max(y_in, y_wall)),
                                                     (self.z_rail_bottom, self.z_rail_top))
        return boxes

    def joint_screw_axes(self) -> list[tuple[float, tuple[float, float, float], tuple[float, float, float]]]:
        """(theta, axis start, axis end) of every joint screw of the screwed joints (JOINT_SCREW_JOINTS lower spigots):
        the radial axis from the vehicle axis out to body_r at z_spigot_bottom + joint_screw_dz - a screw of ANY length
        lies on it (the conservative proxy the radio/log box keeps clear of)."""
        zs = [self.z_prop_spigot_bottom, self.z_intake_spigot_bottom, self.z_avionics_spigot_bottom][:JOINT_SCREW_JOINTS]
        out = []
        for z in zs:
            for th in self.joint_thetas:
                t = math.radians(th)
                out.append((th, (0.0, 0.0, z + self.joint_screw_dz),
                            (self.body_r * math.cos(t), self.body_r * math.sin(t), z + self.joint_screw_dz)))
        return out

    def _radio_log(self, p: dict) -> None:
        """v0.6.2 radio / logger box (``electronics.radio_log``, user 2026-10-15: raises the CG for static margin and
        keeps the antenna away from the ESC / motor leads).  Box and clearances, raised on when short:
        spigot bore (radial) >= PLACEHOLDER_BORE_CLEAR_MM; the COM board, the COM bosses (plan: they reach the same Z),
        every joint-screw axis (Ø joint_hole_outer_d, any length), the tray and its placeholders and the rail ribs >=
        payload_module.com_mount.min_clear_to_tray; the payload bulkhead holes stay uncovered in plan (>= 0)."""
        rl, pl = p["electronics"]["radio_log"], p["payload_module"]
        cmt = pl.get("com_mount") or {}
        if "min_clear_to_tray" not in cmt:
            raise ValueError("electronics.radio_log needs payload_module.com_mount.min_clear_to_tray (its clearance)")
        mc = self.radio_log_min_clear = float(cmt["min_clear_to_tray"])
        l, w, h = rl["dims_lwh"]
        th = math.radians(rl["theta_deg"])
        cx, cy = rl["r_center"] * math.cos(th), rl["r_center"] * math.sin(th)
        cx, cy = (0.0 if abs(cx) < 1e-9 else cx), (0.0 if abs(cy) < 1e-9 else cy)    # cos(-90 deg) = 6e-17
        self.radio_log_theta_deg, self.radio_log_r_center = float(rl["theta_deg"]), float(rl["r_center"])
        self.radio_log_x = (cx - l / 2, cx + l / 2)
        self.radio_log_y = (cy - w / 2, cy + w / 2)
        self.radio_log_z = (self.z_payload_bulkhead_bottom - h, self.z_payload_bulkhead_bottom)
        self.radio_log_center = (cx, cy, 0.5 * (self.radio_log_z[0] + self.radio_log_z[1]))
        box = (self.radio_log_x, self.radio_log_y, self.radio_log_z)
        cl = self.radio_log_clearance
        r_corner = max(math.hypot(x, y) for x in self.radio_log_x for y in self.radio_log_y)
        cl["spigot bore (radial)"] = min(self.spigot_id / 2, self.body_id / 2) - r_corner
        if self.has_com_board:
            cl["COM board"] = _box_gap(box, (self.com_board_x, self.com_board_y, self.com_board_z))
            cl["COM bosses (plan)"] = min(_box_point_dist(box[:2], (bx, by)) - self.com_boss_d / 2
                                          for bx, by in self.com_boss_xy)
        cl["joint screw axes (Ø joint_hole_outer_d)"] = min(
            _box_segment_dist(box, a, b) for _, a, b in self.joint_screw_axes()) - p["body"]["joint_hole_outer_d"] / 2
        for nm, bx in self.avionics_bay_boxes().items():
            cl[nm] = _box_gap(box, bx)
        holes = [(self.bulkhead_hole_r * math.cos(math.radians(t)), self.bulkhead_hole_r * math.sin(math.radians(t)))
                 for t in self.bulkhead_hole_thetas]
        cl["bulkhead holes (footprint, plan)"] = min(_box_point_dist(box[:2], hxy) for hxy in holes) - self.bulkhead_hole_d / 2
        if cl["spigot bore (radial)"] < PLACEHOLDER_BORE_CLEAR_MM - 1e-9:
            raise ValueError(f"radio/log box leaves {cl['spigot bore (radial)']:.2f} mm to the avionics spigot bore "
                             f"(< {PLACEHOLDER_BORE_CLEAR_MM:g})")
        if cl["bulkhead holes (footprint, plan)"] < 0.0:
            raise ValueError("radio/log box covers a payload bulkhead hole in plan")
        for nm, v in cl.items():
            if nm in ("spigot bore (radial)", "bulkhead holes (footprint, plan)"):
                continue
            if v < mc - 1e-9:
                raise ValueError(f"radio/log box is {v:.2f} mm from {nm} (< com_mount.min_clear_to_tray {mc:g})")

    def radio_log_report(self) -> str:
        """v0.6.2 radio/log box report: box, mass position and every clearance of ``radio_log_clearance`` with the limit
        it was held to (the constructor already raised on any shortfall)."""
        if not self.has_radio_log:
            return "radio/log box: not in params (electronics.radio_log absent)"
        el = self.p["electronics"]["radio_log"]
        lines = [f"radio/log box ({el['mass_g']:g} g, {' x '.join(f'{d:g}' for d in el['dims_lwh'])} mm, centre r "
                 f"{self.radio_log_r_center:g} at theta {self.radio_log_theta_deg:g} deg): x {self.radio_log_x[0]:.1f}.."
                 f"{self.radio_log_x[1]:.1f}, y {self.radio_log_y[0]:.1f}..{self.radio_log_y[1]:.1f}, z "
                 f"{self.radio_log_z[0]:.2f}..{self.radio_log_z[1]:.2f} (top = payload bulkhead underside); centre z "
                 f"{self.radio_log_center[2]:.2f} (vane hinge + {self.radio_log_center[2] - self.vane_hinge_z:.1f} mm)",
                 "  clearances (mm, limit):"]
        limits = {"spigot bore (radial)": PLACEHOLDER_BORE_CLEAR_MM, "bulkhead holes (footprint, plan)": 0.0}
        for nm, v in self.radio_log_clearance.items():
            lines.append(f"    {nm:<40} {v:8.2f}   >= {limits.get(nm, self.radio_log_min_clear):g}")
        return "\n".join(lines)

    # ------------------------------------------------------------------ #
    # v0.8 GPS pod
    # ------------------------------------------------------------------ #
    def gps_xyz(self, u: float, v: float, z: float) -> tuple[float, float, float]:
        """Vehicle coordinates of the GPS-pod frame point (u radial along gps_theta, v tangential, z)."""
        t = math.radians(self.gps_theta)
        c, s = math.cos(t), math.sin(t)
        return (u * c - v * s, u * s + v * c, z)

    def gps_floor_top_z(self, u: float) -> float:
        """Z of the pod floor's top (cavity) surface at radial position u: z_floor_top_out at the outer wall's inner
        face, falling inward at floor_slope_deg."""
        return self.gps_z_floor_top_out - (self.gps_u_cav_out - u) * self.gps_floor_tan

    def gps_floor_under_z(self, u: float) -> float:
        """Z of the pod floor's underside at u (floor_t thick normal to the slope -> floor_t / cos vertically)."""
        return self.gps_floor_top_z(u) - self.gps_floor_dz

    def _gps_pod(self, p: dict) -> None:
        """v0.8 GPS pod (``payload_module.gps_pod``, ``avionics_module.gps_entry``, ``electronics.gps``; option B of the v0.8
        CAD study, architect 2026-10-21).  Pod frame: u radial along gps_theta, v tangential, Z vertical.

        Z stack from the nose-separation edge (z_payload_spigot_bottom) down: lid top gap_below_nose_skirt under it, lid t,
        patch_air_gap, patch (face up) on the ground-plane plate, board_standoff_h, board.  Plan: the plate's inner edge
        cavity_clear outside the tube, the cavity cavity_clear around the plate, wall_t walls; the tube is the pod's back
        wall.  Floor: floor_t thick, rising outward at floor_slope_deg from the tube, its top floor_below_plate under the
        plate at the outer wall (support-free when the payload module prints upright).  Corner bosses (lid screws) in the
        two outer cavity corners, the plate notched around them; three lid posts carry the plate (lid + antenna + board
        lift out as one unit).  Raceway (closed tunnel) from the pod floor down the tube to the connector pocket at the
        payload module's bottom edge; the lead enters the avionics bay through gps_entry and runs over the tray top to the
        NAV (``gps_lead_polyline``).

        Raises when: the floor is flatter than 45 deg; the board or the plate is too close to the floor; a wall / boss /
        post wall is thinner than printer.min_feature; the notch does not clear a boss by cavity_clear; a post is off the
        plate, over the patch keep-out (patch + patch_air_gap) or too close to a boss; the bore re-cut would reach the
        spigot shoulder; the pod overlaps the release pod / latch lug sector; the raceway or pocket is too small for the
        lead / connector or does not sit in the tube wall; the pocket or the entry hole comes within min_feature of a joint
        screw hole, the shoulder ring, a vent slot or a tray rail; the lead route in the bay comes closer than half the
        bundle to the tray, a placeholder, the COM board / radio box, a rail or a joint-screw axis.  Noise-source
        distances (from the antenna phase centre) are reported, not raised on (u-blox guidance, not a geometry rule)."""
        g, ge = p["payload_module"]["gps_pod"], p["avionics_module"].get("gps_entry")
        if not ge:
            raise ValueError("payload_module.gps_pod needs avionics_module.gps_entry (the lead's way into the bay)")
        st, gp, lid, bo = g["stack"], g["ground_plane"], g["lid"], g["corner_boss"]
        rw, pk, ld = g["raceway"], g["connector_pocket"], g["lead"]
        mf = p["printer"]["min_feature"]
        cl = self.gps_clearance
        self.gps_theta = float(g["theta_deg"])
        # ---- Z stack ----
        self.gps_lid_t = float(lid["t"])
        self.gps_z_lid_top = self.z_payload_spigot_bottom - lid["gap_below_nose_skirt"]
        self.gps_z_rim = self.gps_z_lid_top - self.gps_lid_t                      # wall tops = lid underside
        pl_, pw_, ph_ = st["patch_lwh"]
        bl_, bw_, bh_ = st["board_lwh"]
        self.gps_patch_air_gap = float(st["patch_air_gap"])
        self.gps_z_patch_top = self.gps_z_rim - self.gps_patch_air_gap
        self.gps_z_gp_top = self.gps_z_patch_top - ph_
        self.gps_gp_t = float(gp["t"])
        self.gps_z_gp_bottom = self.gps_z_gp_top - self.gps_gp_t
        self.gps_z_board_top = self.gps_z_gp_bottom - st["board_standoff_h"]
        self.gps_z_board_bottom = self.gps_z_board_top - bh_
        # ---- plan ----
        side, clr = float(gp["side"]), float(g["cavity_clear"])
        b_u, b_v = (bw_, bl_) if st.get("board_long_side_tangential", True) else (bl_, bw_)
        if side < max(pl_, pw_, b_u, b_v):
            raise ValueError(f"gps_pod.ground_plane.side {side:g} is smaller than the patch / board it carries")
        self.gps_gp_side, self.gps_cavity_clear = side, clr
        self.gps_wall_t, self.gps_floor_t = float(g["wall_t"]), float(g["floor_t"])
        self.gps_u_gp_in = self.body_r + clr                       # the plate's inner edge clears the tube by clr at v = 0
        self.gps_u_c = self.gps_u_gp_in + side / 2
        self.gps_u_gp_out = self.gps_u_gp_in + side
        self.gps_u_cav_out = self.gps_u_gp_out + clr
        self.gps_u_out = self.gps_u_cav_out + self.gps_wall_t
        self.gps_cav_hw = side / 2 + clr
        self.gps_w_h = self.gps_cav_hw + self.gps_wall_t
        r_in = self.body_id / 2
        if self.gps_w_h >= r_in:
            raise ValueError("GPS pod wider than the payload bore")
        # the outer box's back face: its corners touch the bore circle, so the pod back merges into the tube wall over
        # the full width; what it puts inside the bore is re-cut (gps_bore_cut_z)
        self.gps_u_back = math.sqrt(r_in ** 2 - self.gps_w_h ** 2)
        self.gps_corner_c = float(g["corner_chamfer"])
        self.gps_r_corner = max(math.hypot(self.gps_u_out - self.gps_corner_c, self.gps_w_h),
                                math.hypot(self.gps_u_out, self.gps_w_h - self.gps_corner_c))
        # ---- floor ----
        slope = float(g["floor_slope_deg"])
        if slope < 45.0 - 1e-9 or slope >= 90.0:
            raise ValueError(f"gps_pod.floor_slope_deg {slope:g}: the floor must rise at >= 45 deg (support-free when the "
                             f"payload module prints upright)")
        self.gps_floor_slope_deg = slope
        self.gps_floor_tan = math.tan(math.radians(slope))
        self.gps_floor_dz = self.gps_floor_t / math.cos(math.radians(slope))
        self.gps_z_floor_top_out = self.gps_z_gp_bottom - g["floor_below_plate"]
        self.gps_z_lo = self.gps_floor_under_z(self.gps_u_back)                  # outer box bottom (cut below the floor)
        self.gps_z_foot = self.gps_floor_under_z(math.sqrt(self.body_r ** 2 - self.gps_w_h ** 2))   # on the tube skin
        u_board_out = self.gps_u_c + b_u / 2
        cl["board above the floor (vertical)"] = self.gps_z_board_bottom - self.gps_floor_top_z(u_board_out)
        cl["plate above the floor (vertical)"] = self.gps_z_gp_bottom - self.gps_floor_top_z(self.gps_u_gp_out)
        if cl["board above the floor (vertical)"] < st["board_bottom_clear"] - 1e-9:
            raise ValueError(f"GPS board bottom only {cl['board above the floor (vertical)']:.2f} mm above the pod floor "
                             f"(< stack.board_bottom_clear {st['board_bottom_clear']:g})")
        if g["floor_below_plate"] < 0.0 or cl["plate above the floor (vertical)"] < clr - 1e-9:
            raise ValueError("GPS ground plane closer to the pod floor than cavity_clear")
        # ---- corner bosses (lid screws) ----
        bs = float(bo["size"])
        self.gps_boss_u = (self.gps_u_cav_out - bs, self.gps_u_cav_out)
        self.gps_boss_v = (self.gps_cav_hw - bs, self.gps_cav_hw)
        # the boss bottom is the floor underside at the boss's inner face: the boss merges with the floor over its whole
        # footprint (no void under it), and the underside cut trims what it puts below the floor
        self.gps_boss_z0 = self.gps_floor_under_z(self.gps_u_cav_out - bs)
        self.gps_boss_hole_d, self.gps_boss_hole_depth = float(bo["hole_d"]), float(bo["hole_depth"])
        self.gps_boss_hole_uv = [(self.gps_u_cav_out - bs / 2, s * (self.gps_cav_hw - bs / 2)) for s in (1.0, -1.0)]
        self.gps_boss_hole_z0 = self.gps_z_rim - self.gps_boss_hole_depth
        hu, hv = self.gps_boss_hole_uv[0]
        chamfer_k = self.gps_u_out + self.gps_w_h - self.gps_corner_c         # chamfer plane: u + |v| = chamfer_k
        walls = {"boss hole -> corner chamfer": (chamfer_k - hu - hv) / math.sqrt(2.0) - self.gps_boss_hole_d / 2,
                 "boss hole -> outer wall face": self.gps_u_out - hu - self.gps_boss_hole_d / 2,
                 "boss hole -> side wall face": self.gps_w_h - hv - self.gps_boss_hole_d / 2,
                 "boss hole -> boss inner faces": bs / 2 - self.gps_boss_hole_d / 2,
                 "pod wall_t": self.gps_wall_t, "pod floor_t": self.gps_floor_t,
                 "corner chamfer -> cavity (outside the boss)": (chamfer_k - self.gps_boss_u[0] - self.gps_cav_hw) / math.sqrt(2.0)}
        if self.gps_boss_hole_z0 < self.gps_boss_z0 + mf:
            raise ValueError("GPS lid-screw hole runs out of the corner boss")
        # ---- ground plane notches vs bosses ----
        self.gps_gp_notch = float(gp["corner_notch"])
        cl["plate notch -> corner boss"] = min(self.gps_boss_u[0] - (self.gps_u_gp_out - self.gps_gp_notch),
                                               self.gps_boss_v[0] - (side / 2 - self.gps_gp_notch))
        if cl["plate notch -> corner boss"] < clr - 1e-9:
            raise ValueError(f"GPS ground-plane notch clears the corner boss by {cl['plate notch -> corner boss']:.2f} mm "
                             f"(< cavity_clear {clr:g})")
        # ---- lid ----
        self.gps_lid_r_in = self.body_r + lid["tube_gap"]
        self.gps_lid_u_back = math.sqrt(self.gps_lid_r_in ** 2 - self.gps_w_h ** 2)
        self.gps_lid_screw_d = float(lid["screw_clear_d"])
        walls["lid screw hole -> corner chamfer"] = (chamfer_k - hu - hv) / math.sqrt(2.0) - self.gps_lid_screw_d / 2
        walls["lid t"] = self.gps_lid_t
        pi_, pd_ = float(lid["post_inset"]), float(lid["post_d"])
        self.gps_post_d, self.gps_post_hole_d = pd_, float(lid["post_hole_d"])
        self.gps_post_uv = [(self.gps_u_gp_in + pi_, side / 2 - pi_), (self.gps_u_gp_in + pi_, -(side / 2 - pi_)),
                            (self.gps_u_gp_out - pi_, 0.0)]
        self.gps_post_z = (self.gps_z_gp_top, self.gps_z_rim)
        self.gps_post_hole_z1 = self.gps_z_gp_top + lid["post_hole_depth"]
        walls["lid post wall"] = (pd_ - self.gps_post_hole_d) / 2
        if self.gps_post_hole_z1 > self.gps_z_lid_top - mf:
            raise ValueError("GPS lid-post hole runs into the lid top")
        patch_u = (self.gps_u_c - pl_ / 2, self.gps_u_c + pl_ / 2)
        patch_v = (-pw_ / 2, pw_ / 2)
        keep = self.gps_patch_air_gap
        cl["post -> patch keep-out (plan)"] = min(
            _box_point_dist((patch_u, patch_v), (u, v)) - pd_ / 2 - keep for u, v in self.gps_post_uv)
        cl["post -> plate edge (plan)"] = min(min(u - self.gps_u_gp_in, self.gps_u_gp_out - u, side / 2 - abs(v)) - pd_ / 2
                                              for u, v in self.gps_post_uv)
        boss_plan = [(self.gps_boss_u, self.gps_boss_v), (self.gps_boss_u, (-self.gps_boss_v[1], -self.gps_boss_v[0]))]
        cl["post -> corner boss (plan)"] = min(_box_point_dist(bx, (u, v)) - pd_ / 2
                                               for u, v in self.gps_post_uv for bx in boss_plan)
        if cl["post -> patch keep-out (plan)"] < 0.0:
            raise ValueError("a GPS lid post stands inside the patch keep-out (patch + patch_air_gap)")
        if cl["post -> plate edge (plan)"] < clr - 1e-9:
            raise ValueError("a GPS lid post is off the ground plane (closer than cavity_clear to its edge)")
        if cl["post -> corner boss (plan)"] < mf - 1e-9:
            raise ValueError("a GPS lid post is too close to a corner boss")
        notch_lo_u, notch_lo_v = self.gps_u_gp_out - self.gps_gp_notch, side / 2 - self.gps_gp_notch
        if any(u > notch_lo_u - pd_ / 2 and abs(v) > notch_lo_v - pd_ / 2 for u, v in self.gps_post_uv):
            raise ValueError("a GPS lid post stands over a ground-plane notch")
        self.gps_lid_print_h = (self.gps_z_rim - self.gps_z_gp_top) + self.gps_lid_t
        self.gps_lid_xy_extent = max(self.gps_u_out - self.gps_lid_u_back, 2 * self.gps_w_h)
        # ---- bore re-cut / tube restore (the cavity cutter opens the tube wall; the wall sector is joined back) ----
        # The cutters run over_z past the rim (to open the cavity cleanly) but stop short of the spigot shoulder ring
        # (z_payload_spigot_bottom - spigot_shoulder_t, spigot_shoulder_t = body wall, SPEC 2), which reaches into the bore
        z_shoulder = self.z_payload_spigot_bottom - self.body_wall
        over_z = min(0.5, 0.5 * (z_shoulder - self.gps_z_rim))
        if over_z < 0.05:
            raise ValueError("the GPS pod rim is at the payload spigot shoulder: lower the lid (gap_below_nose_skirt)")
        self.gps_cut_z = (self.gps_z_lo - 2.0, self.gps_z_rim + over_z)       # cavity / wedge / bore re-cut / tube restore
        # the tube-wall sector joined back after the cavity and underside cuts: it covers the pod width + 1 mm at the bore
        self.gps_restore_half_deg = math.degrees(math.asin(min(1.0, (self.gps_w_h + 1.0) / r_in)))
        # ---- placeholders (pod frame boxes) ----
        self.gps_patch_box = (patch_u, patch_v, (self.gps_z_gp_top, self.gps_z_patch_top))
        self.gps_board_box = ((self.gps_u_c - b_u / 2, self.gps_u_c + b_u / 2), (-b_v / 2, b_v / 2),
                              (self.gps_z_board_bottom, self.gps_z_board_top))
        self.gps_gp_box = ((self.gps_u_gp_in, self.gps_u_gp_out), (-side / 2, side / 2), (self.gps_z_gp_bottom, self.gps_z_gp_top))
        self.gps_gp_mass_g = float(gp["mass_g_ASSUMED"])
        self.gps_board_mass_g = float(p["electronics"]["gps"]["board_mass_g"])
        self.gps_antenna_uvz = (self.gps_u_c, 0.0, self.gps_z_patch_top - st["phase_centre_below_patch_top_ASSUMED"])
        self.gps_antenna_xyz = self.gps_xyz(*self.gps_antenna_uvz)
        # ---- sector overlap with the release pod / latch lug (plan angles at the pod's outer corners) ----
        half = math.degrees(math.atan2(self.gps_w_h, self.gps_u_back))           # widest angular half-width of the pod

        def dth(a: float, b: float) -> float:
            return abs((a - b + 180.0) % 360.0 - 180.0)
        rel_half = math.degrees(math.atan2(self.pod_w / 2, self.release_pod_r_in))
        cl["angle to the release pod sector (deg)"] = dth(self.gps_theta, self.release_pod_theta) - half - rel_half
        cl["angle to the latch lug (deg)"] = dth(self.gps_theta, self.latch_lug_theta) - half - math.degrees(
            math.atan2(self.lug_w / 2, self.lug_r_in))
        if min(cl["angle to the release pod sector (deg)"], cl["angle to the latch lug (deg)"]) <= 0.0:
            raise ValueError("the GPS pod overlaps the release pod / latch lug sector")
        # ---- raceway + connector pocket on the payload tube ----
        self.gps_rw_w, self.gps_rw_depth, self.gps_rw_wall = float(rw["w"]), float(rw["depth"]), float(rw["wall"])
        u0 = self.body_r + rw["skin_over_tube"]
        self.gps_rw_void_u = (u0, u0 + self.gps_rw_depth)
        self.gps_rw_void_hv = self.gps_rw_w / 2
        self.gps_rw_solid_u = (self.body_r - self.body_wall / 2, u0 + self.gps_rw_depth + self.gps_rw_wall)
        self.gps_rw_solid_hv = self.gps_rw_void_hv + self.gps_rw_wall
        self.gps_pk_w, self.gps_pk_depth, self.gps_pk_h, self.gps_pk_wall = (float(pk["w"]), float(pk["depth"]),
                                                                             float(pk["h"]), float(pk["wall"]))
        self.gps_pk_void_u = (u0, u0 + self.gps_pk_depth)
        self.gps_pk_void_hv = self.gps_pk_w / 2
        self.gps_pk_solid_u = (self.gps_rw_solid_u[0], u0 + self.gps_pk_depth + self.gps_pk_wall)
        self.gps_pk_solid_hv = self.gps_pk_void_hv + self.gps_pk_wall
        self.gps_pk_z = (self.z_payload_bottom, self.z_payload_bottom + self.gps_pk_h)          # void (open at the bottom)
        self.gps_pk_solid_z = (self.z_payload_bottom, self.gps_pk_z[1] + self.gps_pk_wall)
        self.gps_rw_z = (self.z_payload_bottom, self.gps_floor_top_z(self.gps_rw_solid_u[1]))   # solid: up to the floor top
        self.gps_rw_void_z = (self.z_payload_bottom - 1.0, self.gps_rw_z[1] + 1.0)                # void: opens into the cavity
        walls.update({"raceway wall": self.gps_rw_wall, "connector pocket wall": self.gps_pk_wall,
                      "raceway skin over the tube": rw["skin_over_tube"]})
        for nm, hv in (("raceway", self.gps_rw_solid_hv), ("connector pocket", self.gps_pk_solid_hv)):
            if math.sqrt(self.body_r ** 2 - hv ** 2) <= self.gps_rw_solid_u[0] + 1e-9:
                raise ValueError(f"the GPS {nm} back face is not inside the tube wall at its edges (too wide)")
            if math.sqrt(r_in ** 2 - hv ** 2) >= self.gps_rw_solid_u[0] - 1e-9:
                raise ValueError(f"the GPS {nm} would reach into the payload bore")
        bd = float(ld["bundle_d_ASSUMED"])
        self.gps_bundle_d = bd
        cl["raceway void - lead bundle (min of w, depth)"] = min(self.gps_rw_w, self.gps_rw_depth) - bd
        ck_l, ck_w, ck_d = pk["connector_lwh_ASSUMED"]
        cl["pocket - connector (min of h, w, depth)"] = min(self.gps_pk_h - ck_l, self.gps_pk_w - ck_w, self.gps_pk_depth - ck_d)
        if cl["raceway void - lead bundle (min of w, depth)"] < 0.0 or cl["pocket - connector (min of h, w, depth)"] < 0.0:
            raise ValueError("the GPS raceway / connector pocket is smaller than the lead bundle / connector")
        if self.gps_rw_z[1] <= self.gps_pk_solid_z[1]:
            raise ValueError("the GPS raceway does not reach above its connector pocket")
        # joint screws of the avionics/payload joint (z = z_payload_bottom + joint_screw_dz) beside the pocket
        z_js = self.z_payload_bottom + self.joint_screw_dz
        jd = p["body"]["joint_hole_outer_d"]
        arc = min(dth(self.gps_theta, j) for j in self.joint_thetas) * math.pi / 180.0 * self.body_r
        cl["connector pocket -> joint screw hole (arc at body_r)"] = arc - self.gps_pk_solid_hv - jd / 2
        if (self.gps_pk_solid_z[0] - jd / 2 <= z_js <= self.gps_pk_solid_z[1] + jd / 2
                and cl["connector pocket -> joint screw hole (arc at body_r)"] < mf - 1e-9):
            raise ValueError("the GPS connector pocket runs over an avionics/payload joint screw hole")
        # ---- avionics entry hole ----
        self.gps_entry_theta = float(ge["theta_deg"])
        if dth(self.gps_entry_theta, self.gps_theta) > 1e-9:
            raise ValueError("avionics_module.gps_entry.theta_deg must equal payload_module.gps_pod.theta_deg (the lead "
                             "drops straight from the connector pocket into the hole)")
        self.gps_entry_d = float(ge["d"])
        self.gps_entry_z = self.z_avionics_spigot_bottom - ge["z_below_payload_skirt"]
        z_sh_av = self.z_avionics_spigot_bottom - self.body_wall               # avionics spigot shoulder ring bottom
        cl["entry hole -> spigot shoulder (z)"] = z_sh_av - (self.gps_entry_z + self.gps_entry_d / 2)
        if cl["entry hole -> spigot shoulder (z)"] < mf - 1e-9:
            raise ValueError("the GPS entry hole reaches the avionics spigot shoulder")
        half_e = math.degrees(math.atan2(self.gps_entry_d / 2, self.body_id / 2))
        for th in self.vent_slot_thetas:
            v_half = math.degrees(math.atan2(self.vent_slot_w / 2, self.body_id / 2))
            zs = (self.z_vent_slot_bottom, self.z_vent_slot_bottom + self.vent_slot_h)
            if dth(th, self.gps_entry_theta) < v_half + half_e and zs[0] - mf < self.gps_entry_z < zs[1] + mf:
                raise ValueError("the GPS entry hole cuts into a vent slot")
        for sy in (1.0, -1.0):        # tray rails (x tray_x +- ..., y +-rail_y)
            th_r = math.degrees(math.atan2(sy * self.rail_y, self.tray_x)) % 360.0
            if dth(th_r, self.gps_entry_theta) < half_e + 5.0:
                raise ValueError("the GPS entry hole cuts into a tray rail")
        # ---- lead route (pod -> raceway -> pocket -> entry hole -> bay -> NAV top) ----
        rr = self.body_id / 2 - bd / 2 - 1.0            # along the inner wall, 1 mm off it
        z_route = self.z_tray_top + ld["bay_route_z_above_tray_top"]
        uv_rw = 0.5 * (self.gps_rw_void_u[0] + self.gps_rw_void_u[1])
        uv_pk = 0.5 * (self.gps_pk_void_u[0] + self.gps_pk_void_u[1])
        pod_pts = [(self.gps_u_c, -b_v / 2, 0.5 * (self.gps_z_board_bottom + self.gps_z_board_top)),   # header (ASSUMED side)
                   (uv_rw, -b_v / 2, self.gps_floor_top_z(uv_rw) + bd),
                   (uv_rw, 0.0, self.gps_floor_top_z(uv_rw) + bd / 2),
                   (uv_pk, 0.0, self.gps_pk_z[0] + 0.5 * self.gps_pk_h),                    # connector in the pocket
                   (uv_pk, 0.0, self.gps_pk_z[0]),
                   (self.body_r + 0.5, 0.0, self.gps_entry_z),
                   (rr, 0.0, self.gps_entry_z)]
        cross_y = float(ld["bay_cross_y"])
        if abs(cross_y) >= rr:
            raise ValueError("gps_pod.lead.bay_cross_y is outside the bay")
        if self.has_gns:
            # v0.8.1: the lead ends at the GNS board right under the entry hole (header at gns_mount.gps_header_v, ASSUMED)
            pts = [self.gps_xyz(*q) for q in pod_pts]
            u_mid = 0.5 * (self.gns_u_front + self.gns_u_back)
            pts.append(self.gns_xyz(u_mid, float(p["avionics_module"]["gns_mount"]["gps_header_v"]), self.gns_z[1] + bd / 2))
            self.gps_lead_bay_start = len(pod_pts) - 1           # index of the first point inside the bay
        else:
            # v0.8: along the wall over the tray top, across at bay_cross_y, down to the NAV top
            pod_pts.append((rr, 0.0, z_route))
            pts = [self.gps_xyz(*q) for q in pod_pts]
            th_c = math.degrees(math.atan2(cross_y, -math.sqrt(rr * rr - cross_y * cross_y))) % 360.0   # on the -x wall
            th0 = self.gps_theta % 360.0
            n_arc = max(2, int(math.ceil(abs(th0 - th_c) / 5.0)))
            for k in range(1, n_arc + 1):
                th = math.radians(th0 + (th_c - th0) * k / n_arc)
                pts.append((rr * math.cos(th), rr * math.sin(th), z_route))
            x_nav = 0.5 * (self.nav_x[0] + self.nav_x[1])
            pts += [(x_nav, cross_y, z_route), (x_nav, 0.0, self.nav_z[1] + bd / 2)]
            self.gps_lead_bay_start = len(pod_pts) - 2           # index of the first point inside the bay
        self.gps_lead_polyline = pts
        segs = list(zip(pts[:-1], pts[1:]))
        self.gps_lead_route_len = sum(math.dist(a, b) for a, b in segs)
        # clearance of the bay part (entry hole inner end onwards) to every bay box and the joint-screw axes
        bay_boxes = dict(self.avionics_bay_boxes())
        if self.has_com_board:
            bay_boxes["COM board"] = (self.com_board_x, self.com_board_y, self.com_board_z)
        if self.has_radio_log:
            bay_boxes["radio/log box"] = (self.radio_log_x, self.radio_log_y, self.radio_log_z)
        bay_segs = segs[self.gps_lead_bay_start:]
        for nm, bx in bay_boxes.items():
            cl[f"lead (bay) -> {nm}"] = min(_box_segment_dist(bx, a, b) for a, b in bay_segs) - bd / 2
        cl["lead (bay) -> joint screw axes"] = (min(_seg_seg_dist(a, b, ja, jb) for a, b in bay_segs
                                                    for _, ja, jb in self.joint_screw_axes())
                                                - bd / 2 - p["body"]["joint_hole_outer_d"] / 2)
        bad = {k: v for k, v in cl.items() if k.startswith("lead (bay)") and v < -1e-9}
        # the last segment ends ON the NAV top (v0.8.1: the GNS top) - the lead plugs into it: that clearance is >= 0 there
        # by construction
        if bad:
            raise ValueError("the GPS lead route in the avionics bay hits " + ", ".join(f"{k} ({v:.2f})" for k, v in bad.items()))
        # ---- lead mass and COM: wire along the route (+ slack at its centroid), connector in the pocket, hardware at
        # the plate (screws, standoffs, tape) ----
        L_w = self.gps_lead_route_len + ld["slack_mm"]
        m_w = L_w * 1e-3 * ld["mass_g_per_m_ASSUMED"]
        c_w = [sum(math.dist(a, b) * 0.5 * (a[i] + b[i]) for a, b in segs) / self.gps_lead_route_len for i in range(3)]
        c_k = self.gps_xyz(uv_pk, 0.0, self.gps_pk_z[0] + 0.5 * self.gps_pk_h)
        c_h = self.gps_xyz(self.gps_u_c, 0.0, self.gps_z_gp_bottom)
        m_k, m_h = float(ld["connector_pair_g_ASSUMED"]), float(ld["hardware_g_ASSUMED"])
        self.gps_lead_len_mm = L_w
        self.gps_lead_parts_g = {"wire": m_w, "connector pair": m_k, "hardware": m_h}
        self.gps_lead_mass_g = m_w + m_k + m_h
        self.gps_lead_com = tuple((m_w * c_w[i] + m_k * c_k[i] + m_h * c_h[i]) / self.gps_lead_mass_g for i in range(3))
        # ---- separation interfaces: the route stays below the nose edge, outside the payload bore above the bulkhead
        top = max(q[2] for q in pts)
        cl["lead route below the nose-separation edge (z)"] = self.z_payload_spigot_bottom - top
        in_bore = [q for q in pts[:self.gps_lead_bay_start] if math.hypot(q[0], q[1]) < self.body_r]
        if cl["lead route below the nose-separation edge (z)"] <= 0.0 or in_bore:
            raise ValueError("the GPS lead route crosses a separation interface (nose edge / payload bore)")
        # ---- walls ----
        self.gps_walls = walls
        thin = {k: v for k, v in walls.items() if k != "raceway skin over the tube" and v < mf - 1e-9}
        if thin:
            raise ValueError("GPS pod walls below printer.min_feature: " + ", ".join(f"{k} {v:.2f}" for k, v in thin.items()))
        # ---- noise-source distances from the antenna phase centre (reported, not raised on) ----
        a = self.gps_antenna_xyz
        e_ = p["edf"]
        noise = {"ESC (box)": _box_point_dist((self.esc_x, self.esc_y, self.esc_z), a),
                 "logic UBEC (misc box)": _box_point_dist((self.misc_x, self.misc_y, self.misc_z), a),
                 "NAV board": _box_point_dist((self.nav_x, self.nav_y, self.nav_z), a),
                 "battery": _box_point_dist((self.battery_x, self.battery_y, self.battery_z), a),
                 "EDF motor": _box_point_dist(((-e_["motor_d"] / 2, e_["motor_d"] / 2), (-e_["motor_d"] / 2, e_["motor_d"] / 2),
                                               (self.z_edf_housing_bottom - e_.get("motor_protrusion_rear", 0.0),
                                                self.z_edf_housing_bottom)), a)}
        if self.has_com_board:
            noise["COM board"] = _box_point_dist((self.com_board_x, self.com_board_y, self.com_board_z), a)
        if self.has_radio_log:
            noise["LoRa / log box"] = _box_point_dist((self.radio_log_x, self.radio_log_y, self.radio_log_z), a)
            # LoRa antenna: hangs vertically along the -y wall under the radio box (build guide 8), 100 mm ASSUMED
            ya = -(self.body_id / 2 - 3.0)
            noise["LoRa antenna (100 mm ASSUMED, -y wall)"] = _pt_seg_dist_3d(a, (0.0, ya, self.radio_log_z[0]),
                                                                               (0.0, ya, self.radio_log_z[0] - 100.0))
        if self.has_fcu:
            noise["FCU board"] = _box_point_dist((self.fcu_x, self.fcu_y, self.fcu_z), a)
        if self.has_gns:      # v0.8.1: the GNS board (16 MHz crystal: harmonics 1568 / 1584 MHz)
            noise["GNS board"] = _box_point_dist((self.gns_x, self.gns_y, self.gns_z), a)
        if self.tabs:
            wh =((self.wire_hole_r0 + self.wire_hole_r1) / 2 * math.cos(math.radians(self.wire_theta)),
                  (self.wire_hole_r0 + self.wire_hole_r1) / 2 * math.sin(math.radians(self.wire_theta)), self.z_intake_bulkhead_top)
            noise["motor leads (bulkhead hole -> ESC)"] = _pt_seg_dist_3d(a, wh, (self.esc_x[0], 0.0, self.esc_z[0]))
        noise["release servo (MG90S)"] = math.dist(a, (self.release_servo_axis_r * math.cos(math.radians(self.release_pod_theta)),
                                                       self.release_servo_axis_r * math.sin(math.radians(self.release_pod_theta)),
                                                       self.release_servo_bottom_z))
        self.gps_noise_mm = noise
        self.gps_noise_target_mm = float(g.get("min_noise_distance_target_mm", 100.0))

    def gps_report(self) -> str:
        """v0.8 GPS-pod report: stations, the stack, clearances / walls with their limits (the constructor already raised
        on any shortfall), the lead route and mass, the antenna position and the noise-source distances."""
        if not self.has_gps_pod:
            return "GPS pod: not in params (payload_module.gps_pod / electronics.gps absent)"
        g = self.p["payload_module"]["gps_pod"]
        mf = self.p["printer"]["min_feature"]
        ax, ay, az = self.gps_antenna_xyz
        lines = [f"GPS pod (v0.8) at theta {self.gps_theta:g} deg: u {self.gps_u_back:.2f} (back, re-cut at the bore) .. "
                 f"{self.gps_u_out:.2f}, |v| <= {self.gps_w_h:.2f} (cavity u <= {self.gps_u_cav_out:.2f}, |v| <= "
                 f"{self.gps_cav_hw:.2f}); corner chamfers {self.gps_corner_c:g} mm; corner radius {self.gps_r_corner:.2f}",
                 f"  Z: lid top {self.gps_z_lid_top:.2f} (nose edge {self.z_payload_spigot_bottom:.2f} - "
                 f"{g['lid']['gap_below_nose_skirt']:g}), rim / lid underside {self.gps_z_rim:.2f}, patch top "
                 f"{self.gps_z_patch_top:.2f}, ground plane {self.gps_z_gp_bottom:.2f}..{self.gps_z_gp_top:.2f}, board "
                 f"{self.gps_z_board_bottom:.2f}..{self.gps_z_board_top:.2f}; floor top at the outer wall "
                 f"{self.gps_z_floor_top_out:.2f}, rising {self.gps_floor_slope_deg:g} deg; lowest pod material on the "
                 f"tube {self.gps_z_foot:.2f}",
                 f"  ground plane {self.gps_gp_side:g} x {self.gps_gp_side:g} x {self.gps_gp_t:g} mm (u {self.gps_u_gp_in:.2f}.."
                 f"{self.gps_u_gp_out:.2f}), notches {self.gps_gp_notch:g}; lid posts at (u, v) "
                 + ", ".join(f"({u:.1f}, {v:+.1f})" for u, v in self.gps_post_uv)
                 + f"; lid screws at " + ", ".join(f"({u:.1f}, {v:+.1f})" for u, v in self.gps_boss_hole_uv),
                 f"  antenna phase centre (ASSUMED {g['stack']['phase_centre_below_patch_top_ASSUMED']:g} mm under the "
                 f"patch top) xyz ({ax:.2f}, {ay:.2f}, {az:.2f}) mm",
                 f"  raceway void u {self.gps_rw_void_u[0]:.2f}..{self.gps_rw_void_u[1]:.2f} x {2 * self.gps_rw_void_hv:g} wide, "
                 f"Z {self.gps_rw_z[0]:.2f}..{self.gps_rw_z[1]:.2f}; connector pocket u {self.gps_pk_void_u[0]:.2f}.."
                 f"{self.gps_pk_void_u[1]:.2f} x {self.gps_pk_w:g}, Z {self.gps_pk_z[0]:.2f}..{self.gps_pk_z[1]:.2f} (open "
                 f"at the bottom); avionics entry hole D{self.gps_entry_d:g} at z {self.gps_entry_z:.2f}",
                 f"  lead: route {self.gps_lead_route_len:.0f} mm + slack -> {self.gps_lead_len_mm:.0f} mm, "
                 + ", ".join(f"{k} {v:.2f} g" for k, v in self.gps_lead_parts_g.items())
                 + f" = {self.gps_lead_mass_g:.2f} g at ({', '.join(f'{c:.1f}' for c in self.gps_lead_com)})",
                 "  clearances (mm):"]
        for k, v in self.gps_clearance.items():
            lines.append(f"    {k:<52} {v:8.2f}")
        lines.append(f"  walls (mm, >= printer.min_feature {mf:g}):")
        for k, v in self.gps_walls.items():
            lines.append(f"    {k:<52} {v:8.2f}")
        lines.append(f"  noise sources from the phase centre (target >= {self.gps_noise_target_mm:g} mm, reported only):")
        for k, v in sorted(self.gps_noise_mm.items(), key=lambda kv: kv[1]):
            lines.append(f"    {k:<52} {v:8.1f}" + ("" if v >= self.gps_noise_target_mm else "   < target"))
        return "\n".join(lines)

    # ------------------------------------------------------------------ #
    # v0.8.1 GNS board
    # ------------------------------------------------------------------ #
    def gns_xyz(self, u: float, v: float, z: float) -> tuple[float, float, float]:
        """Vehicle coordinates of the GNS-frame point (u radial along gns_theta, v tangential = +theta, z).  gns_theta is a
        multiple of 90 deg, so the frame's axes are exact (no 1e-16 residue from cos / sin) and the board box is
        axis-aligned."""
        c, s = self.gns_cs
        return (u * c - v * s, u * s + v * c, z)

    def gns_diamond_vz(self, k: int) -> list[tuple[float, float]]:
        """(v, z) corners of boss k's cross-section: a square turned 45 deg (half diagonal gns_boss_hd) centred on its hole,
        so every face is at 45 deg (support-free when the avionics module prints upright).  Bottom vertex first, then
        counter-clockwise in (v, z)."""
        v, z = self.gns_hole_vz[k]
        h = self.gns_boss_hd
        return [(v, z - h), (v + h, z), (v, z + h), (v - h, z)]

    def gns_u_wall(self, v: float) -> float:
        """Radial position u of the tube's inner face at tangential offset v in the GNS frame."""
        return math.sqrt((self.body_id / 2) ** 2 - v * v)

    def _gns(self, p: dict) -> None:
        """v0.8.1 GNS board (``electronics.gns``, ``avionics_module.gns_mount``; architect 2026-10-21: a fourth ATmega128A
        runs the GPS receiver, UBX, the GPS filter and the position controller).  The board stands on the avionics tube's
        inner wall at gns_theta, outboard of the battery: PCB back face u_back from the axis on four printed bosses, the
        components (dims radial height) facing the axis, centred tangentially; its top is top_below_entry_hole under the
        GPS lead's entry hole (avionics_module.gps_entry), so the GPS lead runs straight down to it.

        Bosses: diamond cross-section (square turned 45 deg, half diagonal boss_half_diag) on the board's hole pattern,
        from the PCB back face out into the tube wall (clipped at the tube OD), each with a blind Ø hole_d tap hole
        hole_depth deep from the tip (M3 self-tapping from the component side, as the COM board).

        Raises when: gns_theta is not a multiple of 90 deg; the board corners come within PLACEHOLDER_BORE_CLEAR_MM of the
        bore, or the board reaches the avionics spigot shoulder; the PCB back is closer than back_clear to the wall at the
        board edges; the hole pattern is off the board; a boss's outer vertex does not reach past the PCB plane; the boss
        wall around a hole, or the skin between a hole's end and the tube's outer surface, is thinner than
        printer.min_feature; a boss comes within min_feature of the GPS entry hole, a vent slot or a joint-screw axis; the
        board comes closer than gns_mount.min_clear (or a boss closer than min_feature) to the tray,
        its placeholders or a rail rib.  The COM board, radio box, FCU and the leads are checked against it in their own
        sections (``avionics_bay_boxes`` carries it once this method returns)."""
        am, gm, el = p["avionics_module"], p["avionics_module"]["gns_mount"], p["electronics"]["gns"]
        ge = am.get("gps_entry")
        if not ge or not p["payload_module"].get("gps_pod") or not p["electronics"].get("gps"):
            raise ValueError("electronics.gns needs the GPS pod (payload_module.gps_pod, electronics.gps) and its entry hole "
                             "(avionics_module.gps_entry): the GNS board sits under that hole")
        mf = p["printer"]["min_feature"]
        mc = self.gns_min_clear = float(gm["min_clear"])
        th = float(gm["theta_deg"])
        q = th / 90.0
        if abs(q - round(q)) > 1e-9:
            raise ValueError(f"avionics_module.gns_mount.theta_deg {th:g} must be a multiple of 90 (axis-aligned board box)")
        self.gns_theta = th
        self.gns_cs = ((1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0))[int(round(q)) % 4]
        bl, bw, bh = (float(d) for d in el["dims_lwh"])       # z length, tangential width, radial height (as electronics.nav)
        if min(bl, bw, bh) <= 0.0:
            raise ValueError("electronics.gns.dims_lwh must be positive")
        self.gns_dims = (bl, bw, bh)
        self.gns_board_mass_g = float(el["mass_g_ASSUMED"])
        self.gns_screws_g = float(gm["screws_g_ASSUMED"])
        self.gns_mass_g = self.gns_board_mass_g + self.gns_screws_g      # placeholder: board (incl. its leads) + 4 screws
        self.gns_u_back = float(gm["u_back"])
        self.gns_u_front = self.gns_u_back - bh
        if self.gns_u_front <= 0.0:
            raise ValueError("GNS board deeper than u_back: it would cross the vehicle axis")
        z_e = self.z_avionics_spigot_bottom - ge["z_below_payload_skirt"]
        z_top = z_e - ge["d"] / 2 - gm["top_below_entry_hole"]
        self.gns_z = (z_top - bl, z_top)
        self.gns_v = (-bw / 2, bw / 2)
        corners = [self.gns_xyz(u, v, 0.0) for u in (self.gns_u_front, self.gns_u_back) for v in self.gns_v]
        self.gns_x = (min(c[0] for c in corners), max(c[0] for c in corners))
        self.gns_y = (min(c[1] for c in corners), max(c[1] for c in corners))
        self.gns_center = (0.5 * (self.gns_x[0] + self.gns_x[1]), 0.5 * (self.gns_y[0] + self.gns_y[1]),
                           0.5 * (self.gns_z[0] + self.gns_z[1]))
        box = (self.gns_x, self.gns_y, self.gns_z)
        cl = self.gns_clearance
        r_in = self.body_id / 2
        cl["bore (radial, board corners)"] = r_in - max(math.hypot(u, v) for u in (self.gns_u_front, self.gns_u_back)
                                                        for v in self.gns_v)
        if cl["bore (radial, board corners)"] < PLACEHOLDER_BORE_CLEAR_MM - 1e-9:
            raise ValueError(f"GNS board corners {cl['bore (radial, board corners)']:.2f} mm from the avionics bore "
                             f"(< {PLACEHOLDER_BORE_CLEAR_MM:g})")
        cl["board top -> avionics spigot shoulder (z)"] = (self.z_avionics_spigot_bottom - self.body_wall) - self.gns_z[1]
        if cl["board top -> avionics spigot shoulder (z)"] < 0.0:
            raise ValueError("GNS board reaches the avionics spigot shoulder ring")
        if self.gns_z[0] < self.z_avionics_bottom:
            raise ValueError("GNS board runs below the avionics module")
        cl["PCB back -> wall at the board edges"] = self.gns_u_wall(bw / 2) - self.gns_u_back
        if cl["PCB back -> wall at the board edges"] < gm["back_clear"] - 1e-9:
            raise ValueError(f"GNS PCB back only {cl['PCB back -> wall at the board edges']:.2f} mm from the wall at the board "
                             f"edges (< gns_mount.back_clear {gm['back_clear']:g})")
        # ---- bosses and their blind tap holes ----
        half = float(gm["pattern"]) / 2
        self.gns_pattern = 2 * half
        zc = self.gns_center[2]
        self.gns_hole_vz = [(sv * half, zc + sz * half) for sz in (1.0, -1.0) for sv in (1.0, -1.0)]
        self.gns_hole_d, self.gns_hole_depth = float(gm["hole_d"]), float(gm["hole_depth"])
        self.gns_boss_hd = hd = float(gm["boss_half_diag"])
        if not 0.0 < self.gns_hole_d < hd * math.sqrt(2.0):
            raise ValueError("gns_mount.hole_d must be > 0 and smaller than the boss")
        if half + self.gns_hole_d / 2 > min(bl, bw) / 2:
            raise ValueError(f"GNS hole pattern {self.gns_pattern:g} puts the holes off the {bw:g} x {bl:g} board")
        self.gns_hole_u = (self.gns_u_back, self.gns_u_back + self.gns_hole_depth)
        v_out = half + hd
        if v_out >= r_in:
            raise ValueError("GNS bosses outside the bore")
        self.gns_boss_z = (zc - half - hd, zc + half + hd)            # all four bosses (bottom / top vertices)
        walls = {"boss wall around the hole (diamond face)": hd / math.sqrt(2.0) - self.gns_hole_d / 2,
                 "hole end -> tube outer surface (skin, hole's outer edge)":
                     math.sqrt(self.body_r ** 2 - (half + self.gns_hole_d / 2) ** 2) - self.gns_hole_u[1],
                 "shortest boss (outer vertex, tip -> bore)": self.gns_u_wall(v_out) - self.gns_u_back}
        if walls["shortest boss (outer vertex, tip -> bore)"] <= 0.0:
            raise ValueError("a GNS boss's outer vertex does not reach past the PCB plane (the board would sit on the wall)")
        thin = {k: v for k, v in walls.items() if k.startswith(("boss wall", "hole end")) and v < mf - 1e-9}
        if thin:
            raise ValueError("GNS boss walls below printer.min_feature: " + ", ".join(f"{k} {v:.2f}" for k, v in thin.items()))
        self.gns_walls = walls
        # boss bounding boxes (vehicle frame; u from the tip to the OD)
        boss_boxes = []
        for v, z in self.gns_hole_vz:
            a, b = self.gns_xyz(self.gns_u_back, v - hd, 0.0), self.gns_xyz(self.body_r, v + hd, 0.0)
            boss_boxes.append(((min(a[0], b[0]), max(a[0], b[0])), (min(a[1], b[1]), max(a[1], b[1])), (z - hd, z + hd)))
        # ---- bosses vs the GPS entry hole (in the wall's (v, z) plan, at the bore and at the OD) ----
        n_c = 24
        r_c = 0.5 * ge["d"] / math.cos(math.pi / n_c)                 # circumscribed polygon of the hole
        gaps = []
        for r in (r_in, self.body_r):
            v_e = r * math.sin(math.radians(float(ge["theta_deg"]) - th))
            circ = [(v_e + r_c * math.cos(2 * math.pi * k / n_c), z_e + r_c * math.sin(2 * math.pi * k / n_c)) for k in range(n_c)]
            gaps += [_convex_gap_2d(circ, self.gns_diamond_vz(k)) for k in range(4)]
        cl["bosses -> GPS entry hole (wall plan)"] = min(gaps)
        if cl["bosses -> GPS entry hole (wall plan)"] < mf - 1e-9:
            raise ValueError("a GNS boss comes within printer.min_feature of the GPS entry hole")
        # ---- bosses vs the vent slots (plan gap of the boss / slot rectangles when their Z ranges meet) ----
        z_v = (self.z_vent_slot_bottom, self.z_vent_slot_bottom + self.vent_slot_h)
        best = math.inf
        for tv in self.vent_slot_thetas:
            slot = _polar_rect_2d(tv, r_in - 0.5, self.body_r + 2.0, -self.vent_slot_w / 2, self.vent_slot_w / 2)
            for (v, z), bb in zip(self.gns_hole_vz, boss_boxes):
                rect = _polar_rect_2d(th, self.gns_u_back, self.body_r, v - hd, v + hd)
                dz = max(z_v[0] - bb[2][1], bb[2][0] - z_v[1], 0.0)
                best = min(best, max(_convex_gap_2d(slot, rect), dz))
        cl["bosses -> vent slots"] = best
        if best < mf - 1e-9:
            raise ValueError("a GNS boss comes within printer.min_feature of a vent slot")
        cl["bosses -> joint screw axes (Ø joint_hole_outer_d)"] = min(
            _box_segment_dist(bb, a, b) for bb in boss_boxes for _, a, b in self.joint_screw_axes()) - p["body"]["joint_hole_outer_d"] / 2
        if cl["bosses -> joint screw axes (Ø joint_hole_outer_d)"] < mf - 1e-9:
            raise ValueError("a GNS boss comes within printer.min_feature of a joint screw")
        # ---- board / bosses vs the tray, its placeholders and the rail ribs ----
        for nm, bx in self.avionics_bay_boxes().items():
            if nm.startswith("gns"):
                continue
            cl[nm] = _box_gap(box, bx)
            if cl[nm] < mc - 1e-9:
                raise ValueError(f"GNS board is {cl[nm]:.2f} mm from {nm} (< gns_mount.min_clear {mc:g})")
            gb = min(_box_gap(bb, bx) for bb in boss_boxes)
            if gb < mf - 1e-9:
                raise ValueError(f"a GNS boss is {gb:.2f} mm from {nm} (< printer.min_feature {mf:g})")
        self.gns_boss_boxes = boss_boxes          # last: avionics_bay_boxes carries the board and its bosses from now on

    def _gns_leads(self, p: dict) -> None:
        """v0.8.1 GNS leads and distances (after the GPS pod, COM board and radio box are placed).

        glink lead (NAV USART1 <-> GNS USART0 + 5 V + GND, gns_mount.glink_lead): from the GNS top at glink_header_v up to
        the v0.8 GPS lead's corridor over the tray top (payload_module.gps_pod.lead.bay_route_z_above_tray_top), across at
        bay_cross_y (kept inside the bore by the bundle + 1 mm, as the GPS lead), down to the NAV top.  Raises when it comes
        closer than half the bundle to the tray, a placeholder, a boss / rail, the COM board, the radio box or a joint-screw
        axis (it plugs into the GNS and the NAV: those two are >= 0 by construction).  Its mass (route + slack, connector
        pair) is reported against the leads share inside electronics.gns.mass_g_ASSUMED, not added again.

        Distances (reported, not raised on): the board to the ESC (gns_mount.esc_distance_target_mm 'where possible'), to
        the GPS antenna phase centre, the LoRa antenna, the motor leads, the logic UBEC, the battery and the NAV."""
        gm, gl = p["avionics_module"]["gns_mount"], p["avionics_module"]["gns_mount"]["glink_lead"]
        ld = p["payload_module"]["gps_pod"]["lead"]
        bd = self.gps_bundle_d
        rr = self.body_id / 2 - bd / 2 - 1.0
        z_route = self.z_tray_top + ld["bay_route_z_above_tray_top"]
        cross_y = float(ld["bay_cross_y"])
        u_mid = 0.5 * (self.gns_u_front + self.gns_u_back)
        p0 = self.gns_xyz(u_mid, float(gm["glink_header_v"]), self.gns_z[1] + bd / 2)
        x_c = math.copysign(min(abs(p0[0]), math.sqrt(rr * rr - cross_y * cross_y)), p0[0])
        x_nav = 0.5 * (self.nav_x[0] + self.nav_x[1])
        pts = [p0, (p0[0], p0[1], z_route), (x_c, cross_y, z_route), (x_nav, cross_y, z_route),
               (x_nav, 0.0, self.nav_z[1] + bd / 2)]
        if max(math.hypot(q[0], q[1]) for q in pts) > rr + 1e-9:
            raise ValueError("the glink lead route leaves the bay (closer than the bundle + 1 mm to the wall)")
        self.glink_lead_polyline = pts
        segs = list(zip(pts[:-1], pts[1:]))
        self.glink_route_len = sum(math.dist(a, b) for a, b in segs)
        boxes = dict(self.avionics_bay_boxes())
        if self.has_com_board:
            boxes["COM board"] = (self.com_board_x, self.com_board_y, self.com_board_z)
        if self.has_radio_log:
            boxes["radio/log box"] = (self.radio_log_x, self.radio_log_y, self.radio_log_z)
        cl = self.gns_clearance
        for nm, bx in boxes.items():
            cl[f"glink lead -> {nm}"] = min(_box_segment_dist(bx, a, b) for a, b in segs) - bd / 2
        cl["glink lead -> joint screw axes"] = (min(_seg_seg_dist(a, b, ja, jb) for a, b in segs for _, ja, jb in self.joint_screw_axes())
                                                - bd / 2 - p["body"]["joint_hole_outer_d"] / 2)
        bad = {k: v for k, v in cl.items() if k.startswith("glink lead") and v < -1e-9}
        if bad:
            raise ValueError("the glink lead route hits " + ", ".join(f"{k} ({v:.2f})" for k, v in bad.items()))
        self.glink_len_mm = self.glink_route_len + gl["slack_mm"]
        self.glink_mass_g = self.glink_len_mm * 1e-3 * gl["mass_g_per_m_ASSUMED"] + gl["connector_pair_g_ASSUMED"]
        # ---- distances (reported only) ----
        box = (self.gns_x, self.gns_y, self.gns_z)
        d: dict[str, float] = {"ESC (box)": _box_gap(box, (self.esc_x, self.esc_y, self.esc_z)),
                               "logic UBEC (misc box)": _box_gap(box, (self.misc_x, self.misc_y, self.misc_z)),
                               "battery": _box_gap(box, (self.battery_x, self.battery_y, self.battery_z)),
                               "NAV board": _box_gap(box, (self.nav_x, self.nav_y, self.nav_z)),
                               "GPS antenna phase centre": _box_point_dist(box, self.gps_antenna_xyz)}
        if self.has_radio_log:
            ya = -(self.body_id / 2 - 3.0)
            d["LoRa antenna (100 mm ASSUMED, -y wall)"] = _box_segment_dist(box, (0.0, ya, self.radio_log_z[0]),
                                                                            (0.0, ya, self.radio_log_z[0] - 100.0))
        if self.tabs:
            wh = ((self.wire_hole_r0 + self.wire_hole_r1) / 2 * math.cos(math.radians(self.wire_theta)),
                  (self.wire_hole_r0 + self.wire_hole_r1) / 2 * math.sin(math.radians(self.wire_theta)), self.z_intake_bulkhead_top)
            d["motor leads (bulkhead hole -> ESC)"] = _box_segment_dist(box, wh, (self.esc_x[0], 0.0, self.esc_z[0]))
        self.gns_distance_mm = d
        self.gns_esc_target_mm = float(gm.get("esc_distance_target_mm", 100.0))
        # the farthest any board of this size gets from the ESC box on the bay wall at gns_theta (same u_back, slid in Z
        # from the tray bottom to the chosen top): evidence for "where possible"
        esc = (self.esc_x, self.esc_y, self.esc_z)
        bl = self.gns_dims[0]
        zs = [self.z_tray_bottom + k * (self.gns_z[1] - bl - self.z_tray_bottom) / 40.0 for k in range(41)]
        self.gns_esc_best_on_wall_mm = max(_box_gap((self.gns_x, self.gns_y, (z, z + bl)), esc) for z in zs)

    def gns_report(self) -> str:
        """v0.8.1 GNS-board report: board box, bosses / holes, clearances (the constructor already raised on any
        shortfall), walls, the two leads and the distances."""
        if not self.has_gns:
            return "GNS board: not in params (electronics.gns / avionics_module.gns_mount absent)"
        mf = self.p["printer"]["min_feature"]
        bl, bw, bh = self.gns_dims
        cx, cy, cz = self.gns_center
        lines = [f"GNS board (v0.8.1, {self.gns_board_mass_g:g} g ASSUMED + screws {self.gns_screws_g:g} g, {bl:g} x {bw:g} x {bh:g} mm) "
                 f"on the tube wall at theta {self.gns_theta:g} deg: x {self.gns_x[0]:.2f}..{self.gns_x[1]:.2f}, y {self.gns_y[0]:.2f}.."
                 f"{self.gns_y[1]:.2f}, z {self.gns_z[0]:.2f}..{self.gns_z[1]:.2f}; centre ({cx:.2f}, {cy:.2f}, {cz:.2f}) "
                 f"(vane hinge + {cz - self.vane_hinge_z:.1f} mm); PCB back u {self.gns_u_back:g}, components to u {self.gns_u_front:g}",
                 f"  bosses: diamond half-diagonal {self.gns_boss_hd:g} (45 deg faces) at (v, z) "
                 + ", ".join(f"({v:+.1f}, {z:.1f})" for v, z in self.gns_hole_vz)
                 + f", tip u {self.gns_u_back:g} -> tube OD; blind Ø{self.gns_hole_d:g} holes u {self.gns_hole_u[0]:g}.."
                   f"{self.gns_hole_u[1]:g}",
                 f"  GPS lead: route {self.gps_lead_route_len:.0f} mm + slack -> {self.gps_lead_len_mm:.0f} mm, bay part "
                 f"{sum(math.dist(a, b) for a, b in zip(self.gps_lead_polyline[self.gps_lead_bay_start:-1], self.gps_lead_polyline[self.gps_lead_bay_start + 1:])):.0f} mm "
                 f"(entry hole -> GNS header), {self.gps_lead_mass_g:.2f} g",
                 f"  glink lead (GNS -> NAV): route {self.glink_route_len:.0f} mm + slack -> {self.glink_len_mm:.0f} mm, "
                 f"{self.glink_mass_g:.2f} g estimate (inside electronics.gns.mass_g_ASSUMED, not added again)",
                 "  clearances (mm; board >= gns_mount.min_clear "
                 f"{self.gns_min_clear:g}, bosses >= min_feature {mf:g}, leads >= 0 after half the bundle):"]
        for k, v in self.gns_clearance.items():
            lines.append(f"    {k:<52} {v:8.2f}")
        lines.append(f"  walls (mm, >= printer.min_feature {mf:g} except the boss length):")
        for k, v in self.gns_walls.items():
            lines.append(f"    {k:<52} {v:8.2f}")
        lines.append(f"  distances from the board (reported only; ESC target >= {self.gns_esc_target_mm:g} mm where possible, the best "
                     f"spot on this wall gives {self.gns_esc_best_on_wall_mm:.1f} mm):")
        for k, v in sorted(self.gns_distance_mm.items(), key=lambda kv: kv[1]):
            lines.append(f"    {k:<52} {v:8.1f}" + ("   < target" if k == "ESC (box)" and v < self.gns_esc_target_mm else ""))
        return "\n".join(lines)

    def _fcu(self, p: dict) -> None:
        """v0.6.3 FCU board (``electronics.fcu``, user 2026-10-15: a third ATmega128A; the NAV takes the sensors): flat on
        the intake bulkhead's top face on ``foam_t`` foam tape, centred, edges along x / y, inside the intake spigot bore
        below the avionics tray.  Clearances (``fcu_clearance``), raised on when short:

        * intake spigot bore (radial, board corners) >= PLACEHOLDER_BORE_CLEAR_MM;
        * the tray, the tray placeholders, the rail ribs (``avionics_bay_boxes``) and the two tray stops >=
          payload_module.com_mount.min_clear_to_tray (the COM board's limit: the same kind of board);
        * the bulkhead motor-lead hole (theta wire_theta) and servo-lead hole (sl_gather_theta), in plan, >= the same limit
          - the C20 / C23 bundles rise through them, so the board keeps that clearance from the bundles too;
        * vent holes: a hole is reachable for the cable tie when the board footprint does not cover it (plan gap >= 0,
          ``fcu_vent_reachable``); at least one pair of reachable holes must have its straight tie run across the board
          footprint (``fcu_tie_pairs``) - otherwise the tie cannot hold the board.

        Reported, not limits: the centre cable hole (``fcu_cable_hole``: covered / partly covered / clear - nothing is routed
        through it, the leads use the wire / servo holes), the radial room a joint screw of the intake-avionics joint has in
        the spigot bore before it reaches the board when that joint's Z lies within the board's span
        (``fcu_joint_screw_room``), and the bulkhead area under the board for the foam tape (``fcu_seat_area_mm2``,
        ``fcu_seat_fraction``: footprint minus the holes under it)."""
        pl = p["payload_module"]
        cmt = pl.get("com_mount") or {}
        if "min_clear_to_tray" not in cmt:
            raise ValueError("electronics.fcu needs payload_module.com_mount.min_clear_to_tray (its clearance)")
        mc = self.fcu_min_clear = float(cmt["min_clear_to_tray"])
        box = (self.fcu_x, self.fcu_y, self.fcu_z)
        foot = [(self.fcu_x[0], self.fcu_y[0]), (self.fcu_x[1], self.fcu_y[0]),
                (self.fcu_x[1], self.fcu_y[1]), (self.fcu_x[0], self.fcu_y[1])]
        cl = self.fcu_clearance
        r_corner = max(math.hypot(x, y) for x in self.fcu_x for y in self.fcu_y)
        r_bore = min(self.spigot_id / 2, self.body_id / 2)
        cl["intake spigot bore (radial)"] = r_bore - r_corner
        limits = {"intake spigot bore (radial)": PLACEHOLDER_BORE_CLEAR_MM}
        for nm, bx in self.avionics_bay_boxes().items():
            if nm != self.fcu_part:
                cl[nm] = _box_gap(box, bx)
                limits[nm] = mc
        if self.tray_stop_h:
            for sy, tag in ((1.0, "+y"), (-1.0, "-y")):
                y_in, y_wall = sy * (self.rail_y - self.rail_depth), sy * self.body_id / 2
                stop = ((self.tray_x - self.rail_gap / 2, self.tray_x + self.rail_gap / 2),
                        (min(y_in, y_wall), max(y_in, y_wall)), (self.z_tray_stop_bottom, self.z_tray_bottom))
                cl[f"tray stop {tag}"] = _box_gap(box, stop)
                limits[f"tray stop {tag}"] = mc
        # bulkhead lead holes, in plan (the bundles rise through them)
        self.fcu_lead_holes: dict[str, list] = {}
        if self.tabs:
            self.fcu_lead_holes["motor-lead hole (plan)"] = _polar_rect_2d(
                self.wire_theta, self.wire_hole_r0, self.wire_hole_r1, -self.wire_hole_w / 2, self.wire_hole_w / 2)
        if self.servo_leads:
            self.fcu_lead_holes["servo-lead hole (plan)"] = _polar_rect_2d(
                self.sl_gather_theta, self.sl_hole_r0, self.sl_hole_r1, -self.sl_ch_w / 2, self.sl_ch_w / 2)
        for nm, poly in self.fcu_lead_holes.items():
            cl[nm] = _convex_gap_2d(foot, poly)
            limits[nm] = mc
        # vent holes: uncovered in plan -> reachable; tie runs between two reachable holes that cross the board
        self.fcu_vent_gap: dict[float, float] = {}
        centres = {}
        for th in self.vent_hole_thetas:
            c = (self.vent_hole_r * math.cos(math.radians(th)), self.vent_hole_r * math.sin(math.radians(th)))
            centres[th] = c
            self.fcu_vent_gap[th] = _box_point_dist((self.fcu_x, self.fcu_y), c) - self.vent_hole_d / 2
            cl[f"vent hole {th:g} deg (plan)"] = self.fcu_vent_gap[th]
            limits[f"vent hole {th:g} deg (plan)"] = None
        self.fcu_vent_reachable = [th for th, g in self.fcu_vent_gap.items() if g >= 0.0]
        z_mid = 0.5 * (self.fcu_z[0] + self.fcu_z[1])
        self.fcu_tie_pairs = [(a, b) for i, a in enumerate(self.fcu_vent_reachable) for b in self.fcu_vent_reachable[i + 1:]
                              if _box_segment_dist(box, (*centres[a], z_mid), (*centres[b], z_mid)) <= 1e-9]
        # centre cable hole (reported), bulkhead seat area under the board (reported)
        def inside(c: tuple, r: float) -> bool:         # circle (centre c, radius r) wholly inside the footprint
            return (self.fcu_x[0] + r <= c[0] <= self.fcu_x[1] - r) and (self.fcu_y[0] + r <= c[1] <= self.fcu_y[1] - r)
        rc = self.cable_hole_d / 2
        gap_c = _box_point_dist((self.fcu_x, self.fcu_y), (0.0, 0.0))
        self.fcu_cable_hole = "covered" if inside((0.0, 0.0), rc) else ("clear" if gap_c >= rc else "partly covered")
        foot_area = (self.fcu_x[1] - self.fcu_x[0]) * (self.fcu_y[1] - self.fcu_y[0])
        under = math.pi * rc * rc if self.fcu_cable_hole == "covered" else 0.0
        under += sum(math.pi * (self.vent_hole_d / 2) ** 2 for c in centres.values() if inside(c, self.vent_hole_d / 2))
        partial = (self.fcu_cable_hole == "partly covered"
                   or any(g < 0.0 and not inside(centres[t], self.vent_hole_d / 2) for t, g in self.fcu_vent_gap.items()))
        self.fcu_seat_area_mm2 = foot_area - under          # exact unless a hole is only partly under the board
        self.fcu_seat_area_exact = not partial
        self.fcu_seat_fraction = self.fcu_seat_area_mm2 / foot_area
        # intake-avionics joint screws: radial room in the spigot bore before the board, at the joint thetas
        z_joint = self.z_intake_spigot_bottom + self.joint_screw_dz
        self.fcu_joint_screw_room = None
        if self.fcu_z[0] <= z_joint <= self.fcu_z[1]:
            def r_exit(th: float) -> float:          # where the radial ray at th leaves the footprint (centred box)
                c, s = abs(math.cos(math.radians(th))), abs(math.sin(math.radians(th)))
                return min(self.fcu_x[1] / c if c > 1e-12 else math.inf, self.fcu_y[1] / s if s > 1e-12 else math.inf)
            self.fcu_joint_screw_room = min(r_bore - r_exit(th) for th in self.joint_thetas)
            cl["joint screw room in the spigot bore (radial)"] = self.fcu_joint_screw_room
            limits["joint screw room in the spigot bore (radial)"] = None
        self.fcu_limits = limits
        for nm, lim in limits.items():
            if lim is not None and cl[nm] < lim - 1e-9:
                raise ValueError(f"FCU board is {cl[nm]:.2f} mm from {nm} (< {lim:g})")
        if len(self.fcu_vent_reachable) < 2 or not self.fcu_tie_pairs:
            raise ValueError(f"FCU board leaves no cable-tie run: reachable vent holes {self.fcu_vent_reachable} deg, none "
                             f"of their pairs runs across the board")

    def fcu_report(self) -> str:
        """v0.6.3 FCU board report: box, mass position and every clearance of ``fcu_clearance`` with the limit it was
        held to (the constructor already raised on any shortfall), the vent holes / tie runs, the cable hole."""
        if not self.has_fcu:
            return "FCU board: not in params (electronics.fcu absent)"
        el = self.p["electronics"][self.fcu_key]
        lines = [f"{'carrier PCB' if self.vtol else 'FCU board'} ({el['mass_g']:g} g, {' x '.join(f'{d:g}' for d in el['dims_lwh'])} mm, on {self.fcu_foam_t:g} mm "
                 f"foam on the intake bulkhead top z {self.z_intake_bulkhead_top:.2f}): x {self.fcu_x[0]:.1f}..{self.fcu_x[1]:.1f}, "
                 f"y {self.fcu_y[0]:.1f}..{self.fcu_y[1]:.1f}, z {self.fcu_z[0]:.2f}..{self.fcu_z[1]:.2f}; centre z "
                 f"{self.fcu_center[2]:.2f} (vane hinge + {self.fcu_center[2] - self.vane_hinge_z:.1f} mm)",
                 "  clearances (mm, limit):"]
        for nm, v in self.fcu_clearance.items():
            lim = self.fcu_limits.get(nm)
            lines.append(f"    {nm:<44} {v:8.2f}   " + ("(reported)" if lim is None else f">= {lim:g}"))
        lines.append(f"  vent holes reachable for the cable tie (uncovered): {', '.join(f'{t:g}' for t in self.fcu_vent_reachable)} "
                     f"deg; tie runs across the board: " + ", ".join(f"{a:g}/{b:g}" for a, b in self.fcu_tie_pairs) + " deg")
        routed = ("nothing is routed through it: the motor leads use the bulkhead wire hole"
                  + (", the servo leads the servo hole" if self.servo_leads else "") if self.tabs else
                  "lip_clamp preset: SPEC routes no lead through the intake bulkhead")
        lines.append(f"  centre cable hole Ø{self.cable_hole_d:g}: {self.fcu_cable_hole} by the board ({routed}); bulkhead "
                     f"under the board {self.fcu_seat_area_mm2:.0f} mm^2 ({100 * self.fcu_seat_fraction:.1f} % of the "
                     f"footprint{'' if self.fcu_seat_area_exact else ', upper bound: a hole is partly under it'}) for the foam tape")
        if self.fcu_joint_screw_room is not None:
            lines.append(f"  intake-avionics joint screws (Z {self.z_intake_spigot_bottom + self.joint_screw_dz:.2f}, within the "
                         f"board's span): a screw tip may protrude {self.fcu_joint_screw_room:.2f} mm into the spigot bore before "
                         f"it reaches the board (at {', '.join(f'{t:g}' for t in self.joint_thetas)} deg)")
        return "\n".join(lines)

    def window_gable_report(self) -> str:
        """v0.6.2 window lower-end report: gable geometry, overhang, gussets and the inlet area they leave."""
        if not self.has_window_gable:
            return "window lower end: square (intake_module.window_bottom_end absent)"
        im = self.p["intake_module"]
        return "\n".join([
            f"window lower end (v0.6.2): truncated gable, slope {self.gable_slope_deg:g} deg, bridge {self.gable_bridge_w:g} mm "
            f"(flat |y'| <= {self.gable_half_flat:g} at z {self.z_window_bottom:.2f}); upper end square",
            f"  window half angle {self.window_half_angle_deg:.3f} deg: reaches |y'| {self.window_half_chord_in:.2f} (r_band_in) / "
            f"{self.window_half_chord_out:.2f} (body_r) at the struts",
            f"  gable height at the struts: {self.gable_h_inner:.2f} mm at r_band_in, {self.gable_h_outer:.2f} mm at body_r "
            f"(top z {self.z_gable_top:.2f}; window_h {self.window_h:g})",
            f"  lower edge on the curved band surfaces: {self.gable_edge_slope_min_deg:.2f} deg from horizontal at the struts "
            f"(worst; inner and outer alike), {self.gable_edge_slope_bridge_end_deg['inner']:.2f} / "
            f"{self.gable_edge_slope_bridge_end_deg['outer']:.2f} deg at the bridge ends (inner / outer); limit >= "
            f"{GABLE_MIN_EDGE_SLOPE_DEG:g}",
            f"  gussets: {self.gusset_volume_per_window_mm3:.1f} mm^3 per window, {self.gusset_volume_mm3:.1f} mm^3 total "
            f"({self.gusset_volume_mm3 * self.p['printer']['density_g_cm3'] / 1000.0:.2f} g solid), centroid u "
            f"{self.gusset_centroid_u:.2f} mm, z {self.gusset_centroid_z:.2f}",
            f"  inlet area with the gussets: min {self.inlet_area_mm2:.0f} mm^2 at r {self.inlet_area_r_min:.2f} (ratio "
            f"{self.inlet_area_ratio:.3f}), outer {self.inlet_area_outer_mm2:.0f} mm^2 (ratio {self.inlet_area_ratio_outer:.3f}); "
            f"square windows {self.inlet_area_square_mm2:.0f} mm^2 (ratio {self.inlet_area_ratio_square:.3f}); required >= "
            f"{im['min_inlet_area_ratio']:g}",
            "  cutter profile (y', z): " + ", ".join(f"({y:.2f}, {z:.2f})" for y, z in self.window_gable_profile_yz)
            + f", along u +-{self.window_gable_u_half:g}"])

    def _window_gable(self, im: dict) -> None:
        """v0.6.2 (``intake_module.window_bottom_end``, user 2026-10-15: no support inside the windows, so nothing to rip
        off the struts): only the LOWER end of each window - its ceiling when the intake prints flipped - is a truncated
        gable.  In the window's own frame (u along window_thetas, y' tangential = -x sin(theta) + y cos(theta), z) the
        lower boundary is z = z_window_bottom + max(0, |y'| - bridge_w/2) tan(slope_deg): flat over the bridge, then two
        planar faces rising to the struts; the upper end stays square and the radial strut sides bound it.  Both
        generators cut sector INTERSECT the prism ``window_gable_profile_yz`` (region above the boundary, extruded along u
        over +-window_gable_u_half); what the band keeps under the boundary is the gusset.

        Raises for nonsense params: slope outside GABLE_SLOPE_RANGE_DEG, a negative bridge, a bridge as wide as the window
        at the band's inner face (window_half_chord_in, the narrowest), a gable as tall as the window (at body_r, the
        tallest) and a lower edge flatter than GABLE_MIN_EDGE_SLOPE_DEG on the curved surfaces.  On the cylinder of radius
        r the boundary runs z(phi) = z_window_bottom + (r sin(phi) - bridge_w/2) tan(slope), so its slope along the arc
        is cos(phi) tan(slope): steepest at the bridge end, flattest at the strut side phi = window_half_angle, the same
        on the inner and the outer surface (``gable_edge_slope_min_deg``; < slope_deg because the wall turns away from
        the gable plane).  Records the gusset volume / centroid (exact integrals in phi, Simpson in r)."""
        wb = im.get("window_bottom_end")
        self.has_window_gable = bool(wb)
        a = math.radians(self.window_half_angle_deg)
        s_a = math.sin(min(a, math.pi / 2))
        self.window_half_chord_in = self.r_band_in * s_a       # |y'| the window reaches at the strut side, inner face
        self.window_half_chord_out = self.body_r * s_a         # ... outer face
        self.gable_slope_deg = self.gable_bridge_w = None
        self.gable_tan = self.gable_half_flat = 0.0
        self.gable_h_inner = self.gable_h_outer = 0.0
        self.z_gable_top = self.z_window_bottom
        self.gable_edge_slope_min_deg = None
        self.gable_edge_slope_bridge_end_deg: dict[str, float] = {}
        self.window_gable_profile_yz: list[tuple[float, float]] | None = None
        self.window_gable_u_half = None
        self.gusset_volume_per_window_mm3 = self.gusset_volume_mm3 = 0.0
        self.gusset_centroid_u = self.gusset_centroid_z = None
        if not self.has_window_gable:
            return
        slope, bw = float(wb["slope_deg"]), float(wb["bridge_w"])
        lo, hi = GABLE_SLOPE_RANGE_DEG
        if not lo <= slope <= hi:
            raise ValueError(f"intake_module.window_bottom_end.slope_deg {slope:g} is outside {lo:g}..{hi:g} deg")
        if bw < 0.0:
            raise ValueError(f"intake_module.window_bottom_end.bridge_w {bw:g} must be >= 0")
        t, hf = math.tan(math.radians(slope)), 0.5 * bw
        if hf >= self.window_half_chord_in - 1e-9:
            raise ValueError(f"intake_module.window_bottom_end.bridge_w {bw:g} is wider than the window: at the band's "
                             f"inner face (r {self.r_band_in:.2f}) the window reaches only |y'| "
                             f"{self.window_half_chord_in:.2f} from its centre line")
        self.gable_slope_deg, self.gable_bridge_w, self.gable_tan, self.gable_half_flat = slope, bw, t, hf
        self.gable_h_inner = (self.window_half_chord_in - hf) * t
        self.gable_h_outer = (self.window_half_chord_out - hf) * t
        if self.gable_h_outer >= self.window_h - 1e-9:
            raise ValueError(f"window gable {self.gable_h_outer:.2f} mm tall at the struts (body_r) is taller than the "
                             f"window (intake_module.window_h {self.window_h:g})")
        self.z_gable_top = self.z_window_bottom + self.gable_h_outer
        self.gable_edge_slope_min_deg = math.degrees(math.atan(math.cos(min(a, math.pi / 2)) * t))
        self.gable_edge_slope_bridge_end_deg = {
            nm: math.degrees(math.atan(math.cos(math.asin(hf / r)) * t)) for nm, r in (("inner", self.r_band_in),
                                                                                   ("outer", self.body_r))}
        if self.gable_edge_slope_min_deg < GABLE_MIN_EDGE_SLOPE_DEG - 1e-9:
            raise ValueError(f"window gable edge on the curved band surface is {self.gable_edge_slope_min_deg:.1f} deg from "
                             f"horizontal at the struts (< {GABLE_MIN_EDGE_SLOPE_DEG:g}): raise slope_deg or narrow the window")
        # cutter profile (y', z): the region above the boundary out to |y'| = Y (past every point of the sector at
        # r <= body_r + over-travel) and up to z_top (above the window top), counter-clockwise from the bridge
        Y = self.body_r + GABLE_PROFILE_PAST_MM
        zb = self.z_window_bottom
        z_side = zb + (Y - hf) * t
        z_top = max(self.z_window_top, z_side) + GABLE_PROFILE_PAST_MM
        prof = [(-hf, zb)] + ([(hf, zb)] if hf > 0.0 else []) + [(Y, z_side), (Y, z_top), (-Y, z_top), (-Y, z_side)]
        self.window_gable_profile_yz = prof
        self.window_gable_u_half = self.body_r + GABLE_PROFILE_PAST_MM
        # gusset integrals per window (phi exact, r by Simpson): V = int r W dr, Mz = int r Q dr (about z_window_bottom),
        # Mu = int r^2 C dr (along the window centre direction)
        n = GABLE_SIMPSON_N
        rb, R = self.r_band_in, self.body_r
        v = _simpson(lambda r: r * self._gable_phi_integrals(r)[0], rb, R, n)
        mz = _simpson(lambda r: r * self._gable_phi_integrals(r)[1], rb, R, n)
        mu = _simpson(lambda r: r * r * self._gable_phi_integrals(r)[2], rb, R, n)
        self.gusset_volume_per_window_mm3 = v
        self.gusset_volume_mm3 = self.strut_count * v
        self.gusset_centroid_u = mu / v
        self.gusset_centroid_z = zb + mz / v

    def _gable_phi_integrals(self, r: float) -> tuple[float, float, float]:
        """Over one window at radius r, both sides of the bridge (phi from the bridge end p0 = asin(hf / r) to the strut
        side a): (W, Q, C) = (int h dphi, int h^2 / 2 dphi, int cos(phi) h dphi) with h = (r sin(phi) - hf) tan(slope)
        the gusset height; (0, 0, 0) where the bridge covers the whole window or without a gable."""
        if not self.has_window_gable:
            return 0.0, 0.0, 0.0
        a, hf, t = math.radians(self.window_half_angle_deg), self.gable_half_flat, self.gable_tan
        if r * math.sin(a) <= hf:
            return 0.0, 0.0, 0.0
        p0 = math.asin(hf / r)
        w = 2.0 * t * (r * (math.cos(p0) - math.cos(a)) - hf * (a - p0))
        q = t * t * (r * r * (0.5 * (a - p0) - 0.25 * (math.sin(2 * a) - math.sin(2 * p0)))
                     - 2.0 * r * hf * (math.cos(p0) - math.cos(a)) + hf * hf * (a - p0))
        c = 2.0 * t * (0.5 * r * (math.sin(a) ** 2 - math.sin(p0) ** 2) - hf * (math.sin(a) - math.sin(p0)))
        return w, q, c

    def window_open_area_mm2(self, r: float) -> float:
        """Open area (mm^2) of ONE window on the vertical cylinder of radius r: r x window_angle x window_h for the
        radial sector, minus the v0.6.2 gussets r x int h dphi."""
        return r * (math.radians(self.window_angle_deg) * self.window_h - self._gable_phi_integrals(r)[0])

    def gable_boundary_z(self, y_t: float) -> float:
        """Z of the window's lower boundary at tangential coordinate y' (window frame): z_window_bottom without a gable."""
        return self.z_window_bottom + max(0.0, abs(y_t) - self.gable_half_flat) * self.gable_tan

    def intake_xy_extent(self) -> float:
        """Largest XY extent of the intake module: the tube plus every bulge (motor leads, servo leads) as boxes."""
        xs, ys = [-self.body_r, self.body_r], [-self.body_r, self.body_r]
        bulges = []
        if self.tabs:
            bulges.append((self.wire_theta, self.wire_bulge_r_out, self.wire_bulge_w))
        if self.servo_leads:
            bulges.append((self.sl_gather_theta, self.sl_bulge_r_out, self.sl_bulge_w))
        for th, r, w in bulges:
            t = math.radians(th)
            for v in (-w / 2, w / 2):
                xs.append(r * math.cos(t) - v * math.sin(t))
                ys.append(r * math.sin(t) + v * math.cos(t))
        return max(max(xs) - min(xs), max(ys) - min(ys))

    def tab_screw_point(self, tab_theta_deg: float, z: float, t: float) -> tuple[float, float, float]:
        """Point at parameter t (mm, + toward the head) on the tangential screw axis of the tab at tab_theta."""
        th = math.radians(tab_theta_deg)
        r, s = self.tab_screw_r, self.tab_screw_head_side * t
        return (r * math.cos(th) - s * math.sin(th), r * math.sin(th) + s * math.cos(th), z)

    def shoulder_ring_profile(self) -> list[tuple[float, float]]:
        """Closed (r, z) polygon of the solid shoulder ring: duct bore -> outer surface, vane-section
        top -> EDF shoulder, including the chamfer vertex when there is one."""
        pts = [(self.duct_r, self.z_vane_section_top), (self.vane_section_r, self.z_vane_section_top)]
        if self.cone_chamfer_h > 1e-6:
            pts.append((self.cone_r_bottom, self.cone_z0))
        pts += [(self.cone_outer_radius(self.z_edf_shoulder), self.z_edf_shoulder), (self.duct_r, self.z_edf_shoulder)]
        return pts

    def _nose_cavity_volume(self, z_from: float, n: int = 400) -> float:
        """Inner cavity volume (mm^3) from global z_from to the tip, wall subtracted."""
        z0, z1 = z_from, self.z_nose_tip
        dz = (z1 - z0) / n
        vol = 0.0
        for i in range(n):
            z = z0 + (i + 0.5) * dz
            r_in = max(0.0, self.nose_outer_radius(z - self.z_nose_base) - self.nose_wall)
            vol += math.pi * r_in * r_in * dz
        return vol

    def nose_profile_points(self, n: int = 24) -> list[tuple[float, float]]:
        """(r, z_global) points from base to tip (tip point r=0)."""
        pts = []
        for i in range(n + 1):
            x = self.nose_len * i / n
            pts.append((self.nose_outer_radius(x), self.z_nose_base + x))
        pts[-1] = (0.0, self.z_nose_tip)
        return pts

    # ------------------------------------------------------------------ #
    def module_z_ranges(self) -> dict[str, tuple[float, float]]:
        out = {
            "propulsion_module": (self.z_exit, self.z_prop_top),
            "intake_module": (self.z_intake_bottom, self.z_intake_top),
            "avionics_module": (self.z_avionics_bottom, self.z_avionics_top),
        }
        if self.vtol:     # v0.9 CAD-2: the top cap closes the VTOL stack
            out["top_cap"] = (self.z_cap_skirt_bottom, self.z_cap_top)
        else:
            out["payload_module"] = (self.z_payload_bottom, self.z_payload_top)
            out["nose_cone"] = (self.z_nose_skirt_bottom, self.z_nose_tip)
        return out

    def as_dict(self) -> dict[str, Any]:
        out = {}
        for k, v in vars(self).items():
            if k == "p":
                continue
            if isinstance(v, (int, float, str, bool)) or v is None:
                out[k] = v
            elif isinstance(v, (list, tuple)):
                out[k] = list(v)
            elif isinstance(v, dict):
                out[k] = v
        out["module_z_ranges"] = {k: list(v) for k, v in self.module_z_ranges().items()}
        return out

    def table(self) -> str:
        rows = [
            ("body OD / ID / wall", f"{self.body_od:.1f} / {self.body_id:.1f} / {self.body_wall:.1f}"),
            ("spigot OD / ID / h", f"{self.spigot_od:.1f} / {self.spigot_id:.1f} / {self.spigot_h:.1f}"),
            ("duct ID, vane section OD/h", f"{self.duct_id:.1f}, {self.vane_section_od:.1f}/{self.vane_section_h:.1f}"),
            ("EDF bore / shoulder z / housing bottom z", f"{self.edf_bore_d:.1f} / {self.z_edf_shoulder:.1f} / {self.z_edf_housing_bottom:.1f}"),
            ("EDF lip bottom / top z", f"{self.z_edf_lip_bottom:.1f} / {self.z_edf_lip_top:.1f}"),
            ("propulsion module z", f"{self.z_exit:.1f} .. {self.z_prop_top:.1f} (h={self.prop_module_h:.1f})"),
            ("  cone z0..z1, r0..r1", f"{self.cone_z0:.1f}..{self.cone_z1:.1f}, {self.cone_r_bottom:.1f}..{self.cone_r_top:.1f}"),
            *((("  EDF mount", "tabs (screwed), bell lip floats"),
               ("  tabs theta / z / r", f"{', '.join(f'{t:.0f}' for t in self.tab_thetas)} deg, {self.tab_z0:.1f}..{self.tab_z1:.1f}, "
                                       f"{self.tab_r_in:.1f}..{self.tab_r_out:.1f} (t {self.tab_t:.1f})"),
               ("  keyway w / r / z", f"{self.keyway_w:.1f}, {self.keyway_r_in:.2f}..{self.keyway_r_out:.1f}, {self.keyway_z0:.1f}..{self.keyway_z1:.1f}"),
               ("  tab screws z, r, t head/tip/out", f"{', '.join(f'{z:.1f}' for z in self.tab_screw_zs)}, {self.tab_screw_r:.1f}, "
                                                     f"{self.tab_screw_t_head:.1f}/{self.tab_screw_t_tip:.1f}/{self.tab_screw_t_out:.2f} "
                                                     f"({self.tab_screw_size}x{self.tab_screw_len:.0f}, engage {self.tab_screw_engagement:.1f}, "
                                                     f"min hole wall {self.tab_hole_wall_to_bore:.2f})"),
               ("  screw lugs theta far/head, z", f"-{self.block_theta_far:.1f}/+{self.block_theta_head:.1f} deg, "
                                                   f"{', '.join(f'{a:.1f}..{b:.1f}' for a, b in self.boss_z_ranges)}; cbore to t {self.tab_screw_t_cbore_end:.2f}"),
               ("  liner z (wall), wire slot theta / z / r", f"{self.z_edf_shoulder:.1f}..{self.z_liner_top:.1f} ({self.liner_wall:.1f}), "
                                                            f"{self.wire_theta:.0f} deg / {self.wire_slot_z0:.1f}..{self.wire_slot_z1:.1f} / "
                                                            f"{self.wire_slot_lower_r_out:.2f} | {self.wire_slot_r_out:.2f}"),
               ("  intake wire channel r / z", f"{self.wire_ch_r0:.2f}..{self.wire_ch_r1:.2f} (bulge {self.wire_bulge_r_out:.2f}), "
                                              f"{self.wire_ch_z0:.1f}..{self.wire_ch_z1:.1f}"),
               ) if self.tabs else ()),
            *((("  servo leads: columns / entry r, open z", f"{', '.join(f'{t:g}' for t in self.sl_col_thetas)} deg / "
                                                         f"{self.sl_entry_u0:.2f}..{self.sl_entry_u1:.2f}, {self.sl_entry_z_open:.1f}"),
               ("  servo leads: rebate r / z / theta", f"{self.sl_rebate_r0:.2f}.. / {self.sl_rebate_z0:.1f}.. / "
                                                     f"{self.sl_rebate_th0:.2f}..{self.sl_rebate_th1:.2f}"),
               ("  servo leads: channel theta / r / z", f"{self.sl_gather_theta:g} deg / {self.sl_ch_r0:.2f}..{self.sl_ch_r1:.2f} "
                                                      f"(bulge {self.sl_bulge_r_out:.2f}) / {self.sl_ch_z0:.2f}..{self.sl_ch_z1:.1f}"),
               ("  servo leads: mass @ COM, route / ext", f"{self.sl_lead_mass_g:.1f} g @ ({', '.join(f'{c:.1f}' for c in self.sl_lead_com)}), "
                                                        f"{' / '.join(f'{a:.0f}' for a in self.sl_route_len)} | "
                                                        f"{' / '.join(f'{b:.0f}' for b in self.sl_ext_len)} mm"),
               ) if self.servo_leads else ()),
            ("  vane LE/hinge/TE z", f"{self.vane_le_z:.1f} / {self.vane_hinge_z:.1f} / {self.vane_te_z:.1f}"),
            *((("  pod z0 / floor top / rim", f"{self.pod_z0:.1f} / {self.pod_floor_top_z:.1f} / {self.pod_rim_z:.1f}"),
               ("  servo arm z / axis r / tip r", f"{self.servo_arm_z_bottom:.1f} / {self.servo_axis_r:.2f} / {self.servo_arm_tip_r:.2f}"),
               ("  horn tip z", f"{self.horn_tip_z:.1f}"),
               ) if not self.direct_drive else
              (("  direct drive: hub / flange / horn / case u", f"{self.dd_u_hub0:.1f}..{self.dd_u_hub1:.1f} / ..{self.dd_u_flange1:.1f} / "
                                                              f"..{self.dd_u_horn1:.1f} / {self.dd_u_case_top:.1f}..{self.dd_u_case_bot:.1f} "
                                                              f"(ears {self.dd_u_ear[0]:.1f}..{self.dd_u_ear[1]:.1f})"),
               ("  direct drive: case z / ears z, shaft r_out / flat", f"{self.dd_z_case[0]:.2f}..{self.dd_z_case[1]:.2f} / "
                                                                     f"{self.dd_z_ear[0]:.2f}..{self.dd_z_ear[1]:.2f}, {self.vane_shaft_r_out:.1f} / "
                                                                     f"{self.vane_d_flat_r_in:.1f} (engage {self.dd_engagement:.1f})"),
               ("  fairing u / |v| / z / keel z, pocket u / z", f"{self.fair_u[0]:.1f}..{self.fair_u[1]:.1f} / {self.fair_hv:.2f} / "
                                                              f"{self.fair_z[0]:.2f}..{self.fair_z[1]:.2f} / {self.fair_keel_z0:.2f}, "
                                                              f"{self.dd_pocket_u[0]:.1f}..{self.dd_pocket_u[1]:.1f} / "
                                                              f"{self.dd_pocket_z[0]:.2f}..{self.dd_pocket_z[1]:.2f}"),
               ("  hinge moment / servo torque", f"{self.vane_hinge_moment_kgcm:.3f} kgf cm (side load {self.vane_side_load_N:.2f} N) / "
                                                 f"{self.servo_torque_kgcm:g} kgf cm (x{self.vane_hinge_margin:.1f})"),
               )),
            ("  fin theta / root/tip r, LE z root/tip", f"{', '.join(f'{t:g}' for t in self.fin_thetas)} deg / "
                                                         f"{self.fin_root_r:.1f}/{self.fin_tip_r:.1f}, {self.fin_root_le_z:.1f}/{self.fin_tip_le_z:.1f}"),
            *((("  motor leads: route / lead length, mass", f"{self.ml_route_len:.0f} / {self.ml_lead_len:.0f} mm, {self.ml_mass_g:.1f} g "
                                                          f"@ z {self.ml_com[2]:.1f}; joint zone z {self.ml_joint_z[0]:.1f}..{self.ml_joint_z[1]:.1f}"),
               ) if getattr(self, "has_motor_leads", False) else ()),
            *((("  motor leads (photo round 2026-10-29): exit = run z (front ring) / shrink end u / slot u",
                f"motor rear end {self.ml_z_motor_bottom:.1f}, front ring {self.ml_ring_z[0]:.1f}..{self.ml_ring_z[1]:.1f}, "
                f"bundle O{self.ml_bundle_d:g} at z {self.ml_z_run:.2f} (stator TE {self.ml_stator_te_z:.1f}, gap "
                f"{self.ml_stator_te_gap:.2f}) / {self.ml_u_shrink_end:.2f} (+ transition {self.ml_transition_len:.2f}; bend "
                f"start - both {self.ml_shrink_bend_margin:+.2f}) / {self.ml_u_slot:.3f}; R {self.ml_bend_r:g}, exit-angle "
                f"tol +-{self.ml_exit_angle_tol_deg:.1f} deg (E), "
                f"RTV bond u {self.ml_bond_u[0]:.1f}..{self.ml_bond_u[1]:.1f}, spans "
                f"{', '.join(f'{s:.1f}' for s in self.ml_lower_free_spans)}; notch z {self.ml_low_cut_uz[0][1]:.2f}.."
                f"{self.ml_low_cut_uz[-1][1]:.2f}, w {self.ml_low_cut_w:g}"),
               ) if getattr(self, "has_motor_leads", False) and getattr(self, "wire_exit_motor", False) else ()),
            ("intake module z", f"{self.z_intake_bottom:.1f} .. {self.z_intake_top:.1f} (h={self.intake_module_h:.1f})"),
            ("  lip pocket ID, shelf top z", f"{self.lip_pocket_id:.1f}, {self.z_shelf_top:.1f}"),
            ("  band r / t, windows z, sector angle", f"{self.r_band_in:.2f}..{self.body_r:.2f} / {self.window_wall_t:g}, "
                                                    f"{self.z_window_bottom:.1f}..{self.z_window_top:.1f}, {self.window_angle_deg:.2f} deg"),
            ("  struts w at body_r / inner face", f"{self.strut_w:g} / {self.strut_w_in:.2f} ({self.strut_angle_deg:.2f} deg)"),
            ("  inlet area ratio min / outer", f"{self.inlet_area_ratio:.2f} / {self.inlet_area_ratio_outer:.2f}"),
            *((("  window gable slope / bridge, h in/out", f"{self.gable_slope_deg:g} deg / {self.gable_bridge_w:g} mm, "
                                                         f"{self.gable_h_inner:.2f} / {self.gable_h_outer:.2f}"),
               ) if self.has_window_gable else ()),
            ("  bulkhead z", f"{self.z_intake_bulkhead_bottom:.1f}..{self.z_intake_bulkhead_top:.1f}"),
            ("avionics module z", f"{self.z_avionics_bottom:.1f} .. {self.z_avionics_top:.1f}"),
            ("  tray z, rail y", f"{self.z_tray_bottom:.1f}..{self.z_tray_top:.1f}, +-{self.rail_y:.2f}"),
            ("  battery z", f"{self.battery_z[0]:.1f}..{self.battery_z[1]:.1f}"
                            + ("" if self.has_battery else " (slot empty: power.source 'tether')")
                            + (f"; hybrid buffer pack z {self.buffer_z[0]:.1f}..{self.buffer_z[1]:.1f}"
                               if getattr(self, "buffer_z", None) else "")),
            *((("  NAV board x / z", f"{self.nav_x[0]:.1f}..{self.nav_x[1]:.1f} / {self.nav_z[0]:.1f}..{self.nav_z[1]:.1f}"),)
              if self.has_nav else ()),
            *(((f"  {'carrier PCB' if self.vtol else 'FCU board'} x / y / z",
                f"{self.fcu_x[0]:.1f}..{self.fcu_x[1]:.1f} / {self.fcu_y[0]:.1f}..{self.fcu_y[1]:.1f} / "
                f"{self.fcu_z[0]:.1f}..{self.fcu_z[1]:.1f} (on the intake bulkhead)"),
               ) if self.has_fcu else ()),
            *((("  ESC / misc z", f"{self.esc_z[0]:.1f}..{self.esc_z[1]:.1f} / {self.misc_z[0]:.1f}..{self.misc_z[1]:.1f} "
                                  f"(misc on the {self.misc_side} face)"),) if self.vtol else ()),
            *((("  windows theta / w / z, strut", f"{', '.join(f'{t:g}' for t in self.avw_thetas)} deg / {self.avw_w:g} / "
                                                  f"{self.avw_z0:.1f}..{self.avw_z1:.1f}, strut {self.avw_strut_w:.2f} mm, "
                                                  f"open {self.avw_open_area_mm2:.0f} mm^2"),)
              if self.has_av_windows else ()),
            *((("  GNS board x / y / z", f"{self.gns_x[0]:.1f}..{self.gns_x[1]:.1f} / {self.gns_y[0]:.1f}..{self.gns_y[1]:.1f} / "
                                         f"{self.gns_z[0]:.1f}..{self.gns_z[1]:.1f} (on the tube wall, theta {self.gns_theta:g} deg)"),
               ) if self.has_gns else ()),
            *((("top cap z (skirt / plate / under ribs)", f"{self.z_cap_skirt_bottom:.1f} / {self.z_cap_plate_bottom:.1f}.."
                                                           f"{self.z_cap_top:.1f} / {self.z_cap_under:.1f}"),
               ("  ribs theta / t x h, hub d, hardpoint d", f"{', '.join(f'{t:g}' for t in self.cap_rib_thetas)} deg / "
                                                           f"{self.cap_rib_t:g} x {self.cap_rib_h:g}, {self.cap_hub_d:g}, "
                                                           f"{self.cap_hardpoint_d:g}"),
               ("  holes (x, y, d)", "; ".join(f"{k} ({x:.1f}, {y:.1f}, {d:g})" for k, (x, y, d) in self.cap_holes.items())),
               ("  GPS plane / patch / board xy", f"x {self.cap_gp_xy[0][0]:.1f}..{self.cap_gp_xy[0][1]:.1f} y "
                                                 f"{self.cap_gp_xy[1][0]:.1f}..{self.cap_gp_xy[1][1]:.1f} / patch top z "
                                                 f"{self.cap_patch_z[1]:.1f} / board x {self.cap_board_xy[0][0]:.1f}.."
                                                 f"{self.cap_board_xy[0][1]:.1f} y {self.cap_board_xy[1][0]:.1f}.."
                                                 f"{self.cap_board_xy[1][1]:.1f}"),
               ("  tether: boot base / free cable -> window", f"{self.cap_tether['boot base (cap top) -> nearest intake window']:.1f} / "
                                                             f"{self.cap_tether['free cable start (boot top) -> nearest intake window']:.1f} mm "
                                                             f"(free cable >= {self.cap_tether['required (free cable)']:g})"),
               ("  GPS lead / power source", f"{self.gps_lead_len:.0f} mm {self.gps_lead_mass_g:.1f} g / {self.power_source}"),
               ) if self.vtol else ()),
            *((("payload module z", f"{self.z_payload_bottom:.1f} .. {self.z_payload_top:.1f} (h={self.payload_module_h:.1f})"),
               ("  bulkhead / piston top / envelope", f"{self.z_payload_bulkhead_bottom:.1f} / {self.z_piston_top:.1f} / {self.z_envelope_bottom:.1f}..{self.z_envelope_top:.1f}"),
               ("  release pod z0/rim, lug z, arm z", f"{self.release_pod_z0:.1f}/{self.release_pod_rim_z:.1f}, {self.z_lug_bottom:.1f}..{self.z_lug_top:.1f}, {self.z_latch_arm_bottom:.1f}"),
               ) if not self.vtol else ()),
            *((("  COM board x / y / z", f"{self.com_board_x[0]:.1f}..{self.com_board_x[1]:.1f} / {self.com_board_y[0]:.1f}.."
                                         f"{self.com_board_y[1]:.1f} / {self.com_board_z[0]:.1f}..{self.com_board_z[1]:.1f}"),
               ("  COM bosses z / blind hole end, skin", f"{self.z_com_boss_tip:.1f}..{self.z_com_boss_top:.1f} / "
                                                        f"{self.z_com_hole_end:.1f}, {self.com_hole_skin:.1f}"),
               ) if self.has_com_board else ()),
            *((("  radio/log box x / y / z", f"{self.radio_log_x[0]:.1f}..{self.radio_log_x[1]:.1f} / {self.radio_log_y[0]:.1f}.."
                                             f"{self.radio_log_y[1]:.1f} / {self.radio_log_z[0]:.1f}..{self.radio_log_z[1]:.1f}"),
               ) if self.has_radio_log else ()),
            *((("  GPS pod theta / u / |v| / z", f"{self.gps_theta:g} deg / {self.gps_u_back:.1f}..{self.gps_u_out:.1f} / "
                                                 f"{self.gps_w_h:.1f} / {self.gps_z_foot:.1f}..{self.gps_z_lid_top:.1f}"),
               ("  GPS patch top / plate / board z", f"{self.gps_z_patch_top:.1f} / {self.gps_z_gp_bottom:.1f}..{self.gps_z_gp_top:.1f}"
                                                     f" / {self.gps_z_board_bottom:.1f}..{self.gps_z_board_top:.1f}"),
               ("  GPS raceway / pocket / entry z", f"{self.gps_rw_z[0]:.1f}..{self.gps_rw_z[1]:.1f} / {self.gps_pk_z[0]:.1f}.."
                                                    f"{self.gps_pk_z[1]:.1f} / {self.gps_entry_z:.1f}"),
               ) if self.has_gps_pod else ()),
            *((("nose skirt z, base z, tip z", f"{self.z_nose_skirt_bottom:.1f}..{self.z_nose_skirt_top:.1f}, {self.z_nose_base:.1f}, {self.z_nose_tip:.1f}"),
               ("  chute pack z, cavity volume", f"{self.z_chute_pack_bottom:.1f}..{self.z_chute_pack_top:.1f}, {self.nose_cavity_volume_L:.2f} L"),
               ) if not self.vtol else ()),
            *((("  fin hole (u, z)", ", ".join(f"({u:.1f}, {z:.1f})" for u, z in self.fin_hole_uz)
                + f"; {self.fin_hole_area_mm2:.0f} mm^2, frame min {min(self.fin_hole_frame.values()):.1f} mm"),)
              if self.has_fin_hole else ()),
            ("vehicle height / footprint", f"{self.vehicle_h:.1f} / {self.footprint_d:.1f}"),
        ]
        w = max(len(r[0]) for r in rows)
        return "\n".join(f"{k:<{w}}  {v}" for k, v in rows)

    def intake_band_report(self) -> str:
        """v0.6.1 windowed-band report (SPEC 5.1): band, struts, windows, inlet area and every margin the constructor
        held (it raised on any shortfall)."""
        im, mf = self.p["intake_module"], self.p["printer"]["min_feature"]
        lines = [f"intake windowed band (v0.6.1): r {self.r_band_in:.2f}..{self.body_r:.2f} (window_wall_t "
                 f"{self.window_wall_t:g}, body.wall {self.body_wall:g}), Z {self.z_shelf_top:.2f}..{self.z_intake_bulkhead_bottom:.2f}",
                 f"  struts at {', '.join(f'{t:g}' for t in self.strut_thetas)} deg: {self.strut_w:g} mm at body_r, inner "
                 f"face {self.strut_w_in:.2f} mm (>= min_feature {mf:g}), angle {self.strut_angle_deg:.3f} deg, radial sides; "
                 f"section {self.strut_section_mm2:.2f} mm^2 ({self.strut_w:g} x {self.window_wall_t:g} at the mean radius)",
                 "  windows (radial sectors, deg): " + ", ".join(f"{a:.2f}..{b:.2f}" for a, b in self.window_sectors)
                 + f"; Z {self.z_window_bottom:.2f}..{self.z_window_top:.2f} (h {self.window_h:g})",
                 f"  inlet area{' (v0.6.2 gussets included)' if self.has_window_gable else ''}: min "
                 f"{self.inlet_area_mm2:.0f} mm^2 at r {self.inlet_area_r_min:.2f} (ratio {self.inlet_area_ratio:.3f}), outer "
                 f"{self.inlet_area_outer_mm2:.0f} mm^2 at body_r (ratio {self.inlet_area_ratio_outer:.3f}); fan annulus "
                 f"{self.fan_annulus_area_mm2:.0f} mm^2; required >= {im['min_inlet_area_ratio']:g}",
                 f"  shelf ledge past the band (r_band_in - shelf_id/2) {self.shelf_ledge:.2f} mm, margin over min_feature "
                 f"{self.shelf_ledge_margin:.2f} mm"]
        for nm, rec in getattr(self, "band_step_in", {}).items():
            lines.append(f"  {nm} step-in through the band: notch starts {rec['notch_inside_band_face']:.2f} mm inside the "
                         f"band face, channel {rec['channel_past_body_od']:.2f} mm past the body OD, band opened "
                         f"{rec['band_opened_h']:.2f} mm above the shelf (gate {rec['gate_h']:.2f} mm)")
        if self.tabs:
            lines.append(f"  motor-lead bulkhead hole outer edge -> band inner face {self.band_clear_wire_hole:.2f} mm")
        if self.servo_leads:
            lines.append(f"  servo-lead bulkhead hole outer edge -> band inner face {self.band_clear_servo_hole:.2f} mm; "
                         f"servo bundle (2 x 2 ribbons, outer corners) -> band {self.band_clear_servo_bundle:.2f} mm "
                         f"(>= servo_leads.min_clearance {self.p['servo_leads']['min_clearance']:g})")
        return "\n".join(lines)

    def com_report(self) -> str:
        """v0.6 COM-board placement report: every clearance of ``com_clearance`` with the limit it was held to
        (the constructor already raised on any shortfall), plus the board's mass position."""
        if not self.has_com_board:
            return "COM board: not in params (electronics.com / payload_module.com_mount absent)"
        mf = self.p["printer"]["min_feature"]
        el = self.p["electronics"]["com"]
        lines = [f"COM board ({el['mass_g']:g} g, {' x '.join(f'{d:g}' for d in el['dims_lwh'])} mm): "
                 f"x {self.com_board_x[0]:.1f}..{self.com_board_x[1]:.1f}, y {self.com_board_y[0]:.1f}..{self.com_board_y[1]:.1f}, "
                 f"z {self.com_board_z[0]:.2f}..{self.com_board_z[1]:.2f}; centre z {self.com_board_center_z:.2f} "
                 f"(vane hinge + {self.com_board_center_z - self.vane_hinge_z:.1f} mm)",
                 f"  bosses Ø{self.com_boss_d:g} x {self.com_boss_h:g} at (+-{self.com_pattern / 2:g}, +-{self.com_pattern / 2:g}), "
                 f"z {self.z_com_boss_tip:.2f}..{self.z_com_boss_top:.2f}; blind Ø{self.com_hole_d:g} holes {self.com_hole_depth:g} "
                 f"deep from the tip, end z {self.z_com_hole_end:.2f}, bulkhead skin above {self.com_hole_skin:.2f} mm "
                 f"(printer.min_feature {mf:g})",
                 f"  bulkhead holes Ø{self.bulkhead_hole_d:g} at r {self.bulkhead_hole_r:g}, theta "
                 + ", ".join(f"{t:g}" for t in self.bulkhead_hole_thetas) + " deg",
                 "  clearances (mm, limit):"]
        limits = {"spigot bore (radial)": PLACEHOLDER_BORE_CLEAR_MM, "bulkhead holes (boss edge, plan)": mf,
                  "bulkhead holes (board footprint, plan)": 0.0, "spring-seat boss (plan, other face)": None}
        for nm, v in self.com_clearance.items():
            lim = limits.get(nm, self.com_min_clear)
            lines.append(f"    {nm:<40} {v:8.2f}   " + ("(informational)" if lim is None else f">= {lim:g}"))
        return "\n".join(lines)


def build(preset: str | None = None, path: str = PARAMS_PATH, config: str | None = None, power: str | None = None) -> Layout:
    return Layout(load_params(path, preset, config, power))


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    opts = dict(a[2:].split("=", 1) for a in sys.argv[1:] if a.startswith("--") and "=" in a)
    preset = args[0] if args else None
    lay = build(preset, config=opts.get("config"), power=opts.get("power"))
    print(f"preset: {lay.p['meta']['preset']}" + (f"  config: {lay.config} ({lay.power_source})" if lay.vtol else ""))
    print(lay.table())
    print("\nprinted parts (height, xy extent):")
    for n, (h, xy) in lay.printed_parts.items():
        print(f"  {n:<20} {h:7.1f} {xy:7.1f}")
    print()
    print(lay.intake_band_report())
    print(lay.window_gable_report())
    print()
    print(lay.com_report())
    print()
    print(lay.radio_log_report())
    print()
    print(lay.fcu_report())
    print()
    print(lay.gps_report())
    print()
    print(lay.gns_report())
