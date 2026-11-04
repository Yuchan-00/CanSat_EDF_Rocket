"""Analytic mass / COM / inertia estimate of the printed structure and the placeholders.

Invariant: every dimension used here comes from ``cad/layout.py`` (``Layout``) or
``params.json``; the only literals are the few SPEC section-6 placeholder shapes
that neither source provides (named constants below, each cited).  The printed
structure estimate is labelled ESTIMATE and is superseded by a CAD report
(``cad/exports/*_report.json``) whenever one exists.

Units: volumes mm^3, masses g, COM mm in the vehicle frame (+Z up, Z = 0 at the
duct exit), inertia tensors kg*m^2 about the body's own COM (standard tensor
convention: off-diagonals are minus the products of inertia).

v0.5 tab mount (``Layout.tabs``, SPEC s3.7 / s5.1 / s6): the cylindrical outer skin
above ``cone_z1``; the thin liner (duct tube r [duct_r, liner_r_out] + top ring out
to the sleeve, closed cavity behind it); one screw lug per tab and per entry of
``boss_z_ranges`` (exact annular sectors; the lowest one forms the keyway floor)
minus the keyways; the tab-screw holes (clearance, counterbore to
``tab_screw_t_cbore_end``, tap hole), each the exact material it removes from the
sleeve / lug / skin annuli (``tangential_bore_in_annulus``: a counterbore that
runs past the outer skin removes nothing out there); the two-box wire slot.  In
the intake module: the wire bulge, the channel void and the shelf notch, the
bulkhead wire hole.  The ``edf_unit`` placeholder gets the bell ring, motor
cylinder and tab plates.  Radial-box cuts through annular layers use
``box_annulus_area`` (exact plan area by strips).  Every one of these is gated on
``Layout.tabs``.

v0.5d (SPEC s5.1, s5.2, s3.2): the intake channel void runs from ``wire_ch_z0`` and the
floor wedge (triangle ``wire_ch_floor_rz`` extruded over the channel width) is unioned
back AFTER the cuts (``wire_floor_wedge``: only its part inside the channel void is new
material, the rest must lie in the bulge wall); bulge Z, channel depth and pocket ID
come from Layout unchanged.  Both presets: the tray stop - one block per rail slot
between the two ribs, Z [``z_tray_stop_bottom``, ``z_tray_bottom``].  The blind
propulsion joint holes change nothing here because no joint-screw hole is modelled.
So ``lip_clamp`` (KAIST) gives the v0.4 numbers bit for bit except the avionics
module's two tray-stop blocks.  Not modelled in either mode (as in v0.4): the deck
under the propulsion spigot ring (the top lug merges with it in CAD), joint-screw and
servo-ear holes, fillets.

v0.6 (SPEC s5.3 / s6, both presets, when ``Layout.has_com_board``): the payload module
gets the four COM-board bosses under its bulkhead (boss ring minus the tap hole over
``com_boss_h``) and loses the bulkhead material the blind holes remove above them
(``com_hole_depth`` - ``com_boss_h`` each); the ``com`` placeholder (the COM-board box,
``electronics.com.mass_g``, group avionics) joins the SPEC s6 list.  Those are the only
v0.6 changes here.

v0.6.1 (SPEC s5.1, both presets): the intake module's windowed band Z [``z_shelf_top``,
``z_intake_bulkhead_bottom``] is ``r_band_in`` .. body_r (``window_wall_t`` thick, at its fill factor) and the
struts are radial-sided sectors (``strut_angle_deg`` each, exact as a strut fraction of the ring); the wire
channel / shelf notch cuts run through that thicker ring below the windows.  Nothing else changes.

v0.8 (both presets, when ``Layout.has_gps_pod``): the payload module gains the GPS pod shell, raceway and connector
pocket (``gps_pod_solids``, strips along the pod's radial axis), the new printed part ``gps_pod_lid`` is estimated
(``gps_pod_lid``), and the placeholders ``gps`` / ``gps_ground_plane`` (pod-frame boxes, params masses) and the lump
``gps_lead`` (Layout.gps_lead_mass_g at Layout.gps_lead_com) join the SPEC s6 list.
v0.8.1 (both presets, when ``Layout.has_gns``): the avionics module gains the four GNS-board bosses minus their tap holes
(``gns_boss_solids``), the placeholder ``gns`` (box Layout.gns_x/y/z, Layout.gns_mass_g = electronics.gns.mass_g_ASSUMED +
the screws, group avionics) joins the SPEC s6 list, and the ``gps_lead`` lump follows Layout (it now ends at the GNS).

v0.6.2 (both presets, when ``Layout.has_window_gable`` / ``has_radio_log``): the intake module gains the window
gussets (``window_gussets``: Layout's exact volume at the windows' common centroid, band fill); the ``radio_log``
placeholder (box Layout.radio_log_x/y/z, ``electronics.radio_log.mass_g``, group avionics) joins the SPEC s6 list
(misc_electronics keeps its tray box with the lighter params mass).  Nothing else changes.

v0.6.3 (both presets, three ATmega128A): the tray placeholder ``fc`` is ``nav`` (box Layout.nav_x/y/z,
``electronics.nav.mass_g``) and the ``fcu`` placeholder (box Layout.fcu_x/y/z on the intake bulkhead,
``electronics.fcu.mass_g``, group avionics) joins the SPEC s6 list; the servo-lead lump follows Layout (bay legs to the
FCU).  No printed solid changes.

simfix (2026-11-04, wire09c clock mount, ``Layout.clock_mount``): ``tab_mount_solids`` models the mount the solid has -
no lugs, tab screws or 22 mm slot; the keyways round the circle, one floor pad per keyway, the servo-lead column guide
walls and the lead-passage blister + keel minus the passage (``clock_plan_solids``: the Layout's plan polygons against
this estimate's annuli, exact polygon / disc intersections in Z slabs, each cut at its annulus fill).  The sim estimate
(sim/vehicle.py) carries the same model; sim/test_sim.py holds the two to 1e-6 in volume.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable

Vec3 = tuple[float, float, float]
Mat3 = list[list[float]]

G_MM2_TO_KG_M2 = 1e-9  # g*mm^2 -> kg*m^2

# --- SPEC literals that params.json / layout.py do not provide ---------------
EDF_HUB_LEN_MM = 70.0                 # SPEC s6: hub cylinder from housing bottom, +70 mm
MISC_BOX_MM = (20.0, 40.0, 40.0)      # SPEC s6: misc_electronics box 40 x 40 x 20 (x depth, y, z)
MISC_Z_ABOVE_FC_MM = (4.0, 24.0)      # SPEC s6: Z fc_z[1] + 4 .. + 24
SMALL_PLACEHOLDER_MASS_G = 1.0        # SPEC s6: servo arms / pushrods 1 g each
SPRING_FRACTION_OF_HARDWARE = 0.5     # SPEC s6: spring = 0.5 x spring_piston_hardware
HORN_PIN_HOLE_BELOW_TIP_MM = 2.0      # SPEC s4.2 / s4.5: pin hole at horn_tip_z - 2


# --------------------------------------------------------------------------- #
# primitive solids
# --------------------------------------------------------------------------- #
@dataclass
class Solid:
    """A simple solid: volume, centroid, local inertia/mass (diag, mm^2), print fill."""

    volume_mm3: float
    com_mm: Vec3
    k_mm2: Vec3
    fill: float = 1.0
    label: str = ""


def _box_k(dx: float, dy: float, dz: float) -> Vec3:
    return ((dy * dy + dz * dz) / 12.0, (dx * dx + dz * dz) / 12.0, (dx * dx + dy * dy) / 12.0)


def _tube_k(ri: float, ro: float, length: float) -> Vec3:
    a = ri * ri + ro * ro
    return (a / 4.0 + length * length / 12.0, a / 4.0 + length * length / 12.0, a / 2.0)


def box(center: Vec3, dims: Vec3, fill: float = 1.0, label: str = "", volume: float | None = None) -> Solid:
    dx, dy, dz = dims
    return Solid(volume if volume is not None else dx * dy * dz, center, _box_k(dx, dy, dz), fill, label)


def tube_z(ri: float, ro: float, z0: float, z1: float, fill: float = 1.0, label: str = "",
           frac: float = 1.0) -> Solid:
    """Tube/disc/annulus along Z; ``frac`` scales the volume (e.g. windowed section)."""
    length = z1 - z0
    vol = math.pi * (ro * ro - ri * ri) * length * frac
    return Solid(vol, (0.0, 0.0, (z0 + z1) / 2.0), _tube_k(ri, ro, length), fill, label)


def pod_frame(u: float, v: float, theta_deg: float) -> tuple[float, float]:
    """(u radial, v tangential toward +e_theta) at angle theta -> vehicle (x, y)."""
    th = math.radians(theta_deg)
    return u * math.cos(th) - v * math.sin(th), u * math.sin(th) + v * math.cos(th)


def radial_box(u0: float, u1: float, v0: float, v1: float, z0: float, z1: float, theta_deg: float,
               fill: float = 1.0, label: str = "", volume: float | None = None) -> Solid:
    """Box aligned with the radial frame at ``theta`` (u radial, v tangential, z)."""
    du, dv, dz = u1 - u0, v1 - v0, z1 - z0
    x, y = pod_frame((u0 + u1) / 2.0, (v0 + v1) / 2.0, theta_deg)
    ku, kv, kz = _box_k(du, dv, dz)
    th = math.radians(theta_deg)
    c2, s2 = math.cos(th) ** 2, math.sin(th) ** 2
    k = (ku * c2 + kv * s2, ku * s2 + kv * c2, kz)
    return Solid(volume if volume is not None else du * dv * dz, (x, y, (z0 + z1) / 2.0), k, fill, label)


def box_annulus_area(u0: float, u1: float, v_half: float, a: float, b: float = math.inf,
                     n: int = 400) -> tuple[float, float]:
    """Plan area and radial centroid of the radial box u in [u0, u1], |v| <= v_half (u >= 0 side)
    intersected with the annulus a <= r <= b.  Midpoint strips in v; each strip is exact."""
    dv = 2.0 * v_half / n
    area = mom = 0.0
    for i in range(n):
        v = -v_half + (i + 0.5) * dv
        if abs(v) > b:
            continue
        lo = max(u0, math.sqrt(a * a - v * v)) if abs(v) < a else u0
        hi = u1 if math.isinf(b) else min(u1, math.sqrt(b * b - v * v))
        if hi > lo:
            area += (hi - lo) * dv
            mom += 0.5 * (hi * hi - lo * lo) * dv
    return area, (mom / area if area > 0.0 else 0.5 * (u0 + u1))


def clipped_radial_box(u0: float, u1: float, v_half: float, a: float, b: float, z0: float, z1: float,
                       theta_deg: float, sign: float = 1.0, fill: float = 1.0, label: str = "") -> Solid | None:
    """Radial box clipped to the annulus a <= r <= b: exact volume and radial centroid, box-like inertia.
    ``sign`` = -1 for a cut.  None when the two do not overlap."""
    area, uc = box_annulus_area(u0, u1, v_half, a, b)
    if area <= 0.0 or z1 <= z0:
        return None
    du = area / (2.0 * v_half)                    # equivalent radial depth
    return radial_box(uc - du / 2.0, uc + du / 2.0, -v_half, v_half, z0, z1, theta_deg, fill=fill, label=label,
                      volume=sign * area * (z1 - z0))


def tangential_bore_in_annulus(r_s: float, rho: float, t0: float, t1: float, a: float, b: float,
                               n: int = 400) -> tuple[float, float, float]:
    """Volume and centroid (u_c, t_c) of a cylinder of radius ``rho`` whose axis is the tangential line
    u = r_s (Z constant), t in [t0, t1], intersected with the annulus a <= sqrt(u^2 + t^2) <= b.

    Strips across u: each strip's (u, Z) area is the exact slice of the disc, its t-interval is exact at the
    strip's mid u (the annulus seen along t is |t| in [sqrt(a^2 - u^2), sqrt(b^2 - u^2)])."""
    def disc(w: float) -> float:                    # disc area with u - r_s <= w
        w = max(-rho, min(rho, w))
        return w * math.sqrt(max(rho * rho - w * w, 0.0)) + rho * rho * math.asin(w / rho)

    du = 2.0 * rho / n
    vol = mu = mt = 0.0
    for i in range(n):
        wa = -rho + i * du
        u = r_s + wa + 0.5 * du
        if u >= b:
            continue
        hi = math.sqrt(b * b - u * u)
        lo = math.sqrt(a * a - u * u) if u < a else 0.0
        area = disc(wa + du) - disc(wa)
        for s0, s1 in ((lo, hi), (-hi, -lo)):
            x0, x1 = max(s0, t0), min(s1, t1)
            if x1 > x0:
                vol += area * (x1 - x0)
                mu += area * (x1 - x0) * u
                mt += area * 0.5 * (x1 * x1 - x0 * x0)
    if vol <= 0.0:
        return 0.0, r_s, 0.5 * (t0 + t1)
    return vol, mu / vol, mt / vol


def clip_polygon(poly: list[tuple[float, float]], axis: int, bound: float, keep_above: bool) -> list[tuple[float, float]]:
    """Sutherland-Hodgman clip of a convex polygon [(u, z), ...] to the half plane p[axis] >= bound
    (``keep_above``) or p[axis] <= bound."""
    def inside(pt: tuple[float, float]) -> bool:
        return pt[axis] >= bound if keep_above else pt[axis] <= bound

    out: list[tuple[float, float]] = []
    for i, a in enumerate(poly):
        b = poly[(i + 1) % len(poly)]
        if inside(a):
            out.append(a)
        if inside(a) != inside(b):
            t = (bound - a[axis]) / (b[axis] - a[axis])
            out.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
    return out


def polygon_props(poly: list[tuple[float, float]]) -> tuple[float, float, float, float, float]:
    """Area, centroid (u_c, z_c) and central second moments (int (u-u_c)^2 dA, int (z-z_c)^2 dA) of a simple
    polygon [(u, z), ...] in either orientation (shoelace formulas)."""
    n = len(poly)
    a = cu = cz = suu = szz = 0.0
    for i in range(n):
        (u0, z0), (u1, z1) = poly[i], poly[(i + 1) % n]
        cr = u0 * z1 - u1 * z0
        a += cr
        cu += (u0 + u1) * cr
        cz += (z0 + z1) * cr
        suu += (u0 * u0 + u0 * u1 + u1 * u1) * cr
        szz += (z0 * z0 + z0 * z1 + z1 * z1) * cr
    a *= 0.5
    if abs(a) < 1e-15:
        return 0.0, 0.0, 0.0, 0.0, 0.0
    uc, zc = cu / (6.0 * a), cz / (6.0 * a)
    area = abs(a)
    return area, uc, zc, abs(suu / 12.0) - area * uc * uc, abs(szz / 12.0) - area * zc * zc


# --- wire09c clock mount (simfix 2026-11-04): plan polygon INTERSECT axis-centred disc, exact -----------------------
def _seg_moments(x0: float, y0: float, x1: float, y1: float) -> list[float]:
    """[A, Sx, Sy, Sxx, Syy, Sxy] (int 1, x, y, x^2, y^2, xy dA) of the signed triangle (origin, p0, p1)."""
    c = x0 * y1 - x1 * y0
    return [0.5 * c, (x0 + x1) * c / 6.0, (y0 + y1) * c / 6.0, (x0 * x0 + x0 * x1 + x1 * x1) * c / 12.0,
            (y0 * y0 + y0 * y1 + y1 * y1) * c / 12.0, (x0 * y1 + 2 * x0 * y0 + 2 * x1 * y1 + x1 * y0) * c / 24.0]


def _arc_moments(R: float, a0: float, a1: float) -> list[float]:
    """The same moments of the circular sector (origin, radius R, angles a0 -> a1 rad)."""
    d, s2 = a1 - a0, (math.sin(2 * a1) - math.sin(2 * a0)) / 4.0
    r3, r4 = R ** 3 / 3.0, R ** 4 / 4.0
    return [0.5 * R * R * d, r3 * (math.sin(a1) - math.sin(a0)), r3 * (math.cos(a0) - math.cos(a1)),
            r4 * (d / 2.0 + s2), r4 * (d / 2.0 - s2), r4 * (math.sin(a1) ** 2 - math.sin(a0) ** 2) / 2.0]


def _vadd(a: list[float], b: list[float], s: float = 1.0) -> list[float]:
    return [x + s * y for x, y in zip(a, b)]


def ccw_polygon(poly) -> list[tuple[float, float]]:
    pts = [(float(x), float(y)) for x, y in poly]
    a = sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]))
    return pts if a > 0 else pts[::-1]


def _point_in_polygon(x: float, y: float, poly: list[tuple[float, float]]) -> bool:
    ins = False
    for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            ins = not ins
    return ins


def polygon_disc_moments(poly: list[tuple[float, float]], R: float) -> list[float]:
    """Moments [A, Sx, Sy, Sxx, Syy, Sxy] of (CCW plan polygon) INTERSECT the disc r <= R on the axis, exact: the
    polygon's edges split where they cross the circle (the parts inside, as origin triangles) plus the circle's arcs that
    lie inside the polygon (as origin sectors) - Green's theorem.  R = inf: the polygon.  Vertices ON the circle (the
    blister's inner edge is the EDF bore) are taken against R + 1e-6 mm so every crossing is a proper one."""
    if not math.isinf(R) and min(abs(math.hypot(x, y) - R) for x, y in poly) < 1e-7:
        R += 1e-6
    m, angs = [0.0] * 6, []
    for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
        if math.isinf(R):
            m = _vadd(m, _seg_moments(x0, y0, x1, y1))
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
                m = _vadd(m, _seg_moments(x0 + ta * dx, y0 + ta * dy, x0 + tb * dx, y0 + tb * dy))
    if math.isinf(R):
        return m
    if not angs:
        if m[0] > 0.0:
            return m
        return _arc_moments(R, 0.0, 2.0 * math.pi) if _point_in_polygon(R, 0.0, poly) else m
    angs.sort()
    for k, a0 in enumerate(angs):
        a1 = angs[k + 1] if k + 1 < len(angs) else angs[0] + 2.0 * math.pi
        am = 0.5 * (a0 + a1)
        if a1 - a0 > 1e-12 and _point_in_polygon(R * math.cos(am), R * math.sin(am), poly):
            m = _vadd(m, _arc_moments(R, a0, a1))
    return m


def plan_slab(mom: list[float], z0: float, z1: float, sign: float, fill: float, label: str) -> Solid | None:
    """Prism z0..z1 over a plan region of moments ``mom``: volume sign x A x h, centroid, diagonal k about it."""
    A = mom[0]
    if A <= 1e-9:
        return None
    h = z1 - z0
    cx, cy = mom[1] / A, mom[2] / A
    k = ((mom[4] - A * cy * cy) / A + h * h / 12.0, (mom[3] - A * cx * cx) / A + h * h / 12.0,
         (mom[3] + mom[4] - A * (cx * cx + cy * cy)) / A)
    return Solid(sign * A * h, (cx, cy, 0.5 * (z0 + z1)), k, fill, label)


def sector_z(ri: float, ro: float, a0_deg: float, a1_deg: float, z0: float, z1: float, fill: float = 1.0,
             label: str = "") -> Solid:
    """Annular sector ri..ro, a0..a1 (deg, a1 > a0), Z z0..z1: exact volume, centroid and diagonal inertia."""
    a0, a1 = math.radians(a0_deg), math.radians(a1_deg)
    d = a1 - a0
    h = z1 - z0
    area = 0.5 * d * (ro * ro - ri * ri)
    r3 = (ro**3 - ri**3) / 3.0
    r4 = (ro**4 - ri**4) / 4.0
    xbar = r3 * (math.sin(a1) - math.sin(a0)) / area
    ybar = r3 * (math.cos(a0) - math.cos(a1)) / area
    s2 = (math.sin(2 * a1) - math.sin(2 * a0)) / 4.0
    xx = r4 * (d / 2.0 + s2) / area                 # mean x^2
    yy = r4 * (d / 2.0 - s2) / area                 # mean y^2
    k = (yy - ybar**2 + h * h / 12.0, xx - xbar**2 + h * h / 12.0, xx + yy - xbar**2 - ybar**2)
    return Solid(area * h, (xbar, ybar, (z0 + z1) / 2.0), k, fill, label)


def revolved_shell(r_out: Callable[[float], float], r_in: Callable[[float], float], z0: float, z1: float,
                   n: int, fill_of_thickness: Callable[[float], float] | None = None,
                   label: str = "") -> list[Solid]:
    """Slice a revolved shell (r_in(z) .. r_out(z)) into n thin rings along Z."""
    out: list[Solid] = []
    dz = (z1 - z0) / n
    for i in range(n):
        za, zb = z0 + i * dz, z0 + (i + 1) * dz
        z = (za + zb) / 2.0
        ro, ri = r_out(z), max(0.0, r_in(z))
        if ro <= ri:
            continue
        fill = fill_of_thickness(ro - ri) if fill_of_thickness else 1.0
        out.append(tube_z(ri, ro, za, zb, fill=fill, label=label))
    return out


# --------------------------------------------------------------------------- #
# aggregation
# --------------------------------------------------------------------------- #
def inertia_about(items: list[tuple[float, Vec3, Mat3]], point: Vec3) -> Mat3:
    """Sum inertia tensors (kg*m^2 about their own COMs) about ``point`` (parallel axis)."""
    I = [[0.0] * 3 for _ in range(3)]
    for m_g, com, I_own in items:
        m = m_g * 1e-3
        d = [(com[i] - point[i]) * 1e-3 for i in range(3)]
        d2 = d[0] ** 2 + d[1] ** 2 + d[2] ** 2
        for r in range(3):
            for c in range(3):
                I[r][c] += I_own[r][c] + m * ((d2 if r == c else 0.0) - d[r] * d[c])
    return I


def _diag(kx: float, ky: float, kz: float) -> Mat3:
    return [[kx, 0.0, 0.0], [0.0, ky, 0.0], [0.0, 0.0, kz]]


@dataclass
class PartEstimate:
    """Mass properties of one component (printed, placeholder or lumped)."""

    name: str
    kind: str                      # printed | placeholder | lumped
    mass_g: float
    com_mm: Vec3
    inertia_about_com_kg_m2: Mat3
    volume_mm3: float = 0.0
    density_g_cm3: float = 0.0
    mass_solid_g: float | None = None   # printed only: fully-solid mass (what CAD reports)
    notes: list[str] = field(default_factory=list)


def aggregate(name: str, kind: str, solids: list[Solid], rho_g_cm3: float | None = None,
              mass_g: float | None = None, notes: list[str] | None = None) -> PartEstimate:
    """Combine sub-solids.  Give ``rho_g_cm3`` (printed) or ``mass_g`` (placeholder)."""
    vol = sum(s.volume_mm3 for s in solids)
    if rho_g_cm3 is not None:
        masses = [rho_g_cm3 * 1e-3 * s.volume_mm3 * s.fill for s in solids]
        mass_solid = rho_g_cm3 * 1e-3 * vol
        rho = rho_g_cm3
    else:
        assert mass_g is not None and vol > 0
        rho = mass_g / (vol * 1e-3)
        masses = [mass_g * s.volume_mm3 / vol for s in solids]   # sums exactly to the params mass
        mass_solid = None
    m_tot = sum(masses) if mass_g is None else mass_g
    com = tuple(sum(m * s.com_mm[i] for m, s in zip(masses, solids)) / m_tot for i in range(3))
    items = [(m, s.com_mm, _diag(*(m * 1e-3 * k * 1e-6 for k in s.k_mm2))) for m, s in zip(masses, solids)]
    I = inertia_about(items, com)  # type: ignore[arg-type]
    return PartEstimate(name, kind, m_tot, com, I, vol, rho, mass_solid, list(notes or []))  # type: ignore[arg-type]


def point_mass(name: str, mass_g: float, com_mm: Vec3, notes: list[str] | None = None) -> PartEstimate:
    return PartEstimate(name, "lumped", mass_g, com_mm, _diag(0.0, 0.0, 0.0), 0.0, 0.0, None, list(notes or []))


# --------------------------------------------------------------------------- #
# printed structure
# --------------------------------------------------------------------------- #
class StructureEstimator:
    """Analytic volumes of every printed part in ``Layout.printed_parts``.

    ``fill_factor(t)`` models the slicer (SPEC s12: ``wall_loops`` loops of
    ``line_w`` plus ``infill_pct`` gyroid): a region thicker than two shells is
    only partly solid.  ``mass_solid_g`` keeps the fully-solid figure that a CAD
    mass-property report will show.
    """

    def __init__(self, lay, p: dict):
        self.lay = lay
        self.p = p
        self.rho = p["printer"]["density_g_cm3"]
        pr = p["printer"]
        self.shell = pr["wall_loops"] * pr["line_w"]
        self.infill = pr["infill_pct"] / 100.0

    # -- helpers -----------------------------------------------------------
    def fill_factor(self, thickness_mm: float) -> float:
        t = max(thickness_mm, 1e-9)
        if t <= 2.0 * self.shell:
            return 1.0
        return (2.0 * self.shell + (t - 2.0 * self.shell) * self.infill) / t

    def inner_fill(self, total_t: float, counted_t: float) -> float:
        """Fill of the part of a ``total_t`` thick solid region whose other ``counted_t`` mm are already
        modelled at fill 1 (e.g. the liner inside the sleeve): makes the whole region match fill_factor(total_t)."""
        rest = total_t - counted_t
        if rest <= 1e-9:
            return 1.0
        return min(1.0, max(0.0, (self.fill_factor(total_t) * total_t - counted_t) / rest))

    def cone_r_out(self, z: float) -> float:
        L = self.lay
        return L.cone_outer_radius(z)

    def _pod(self, theta: float, u_wall: float, u_out: float, w: float, z0: float, z_rim: float,
             cav_u0: float, cav_u1: float, cav_w: float, z_floor_top: float, z_clamp: float,
             label: str) -> list[Solid]:
        """Servo cup: box (outside the wall) - cavity + 45 deg underside filler wedge (SPEC s3.4)."""
        sol = [radial_box(u_wall, u_out, -w / 2, w / 2, z0, z_rim, theta, label=f"{label} box"),
               radial_box(cav_u0, cav_u1, -cav_w / 2, cav_w / 2, z_floor_top, z_rim, theta,
                          volume=-(cav_u1 - cav_u0) * cav_w * (z_rim - z_floor_top), label=f"{label} cavity")]
        n = 40
        du = (u_out - u_wall) / n
        for i in range(n):
            ua, ub = u_wall + i * du, u_wall + (i + 1) * du
            u = (ua + ub) / 2
            z_line = z0 - (u_out - u)
            zb = max(z_clamp, z_line)
            h = z0 - zb
            if h <= 0:
                continue
            sol.append(radial_box(ua, ub, -w / 2, w / 2, zb, z0, theta,
                                  fill=self.fill_factor(min(w, h)), label=f"{label} chamfer wedge"))
        return sol

    # -- parts -------------------------------------------------------------
    def propulsion_module(self) -> list[Solid]:
        L = self.lay
        sol: list[Solid] = [tube_z(L.duct_r, L.vane_section_r, L.z_exit, L.z_vane_section_top, label="duct wall")]
        sol += revolved_shell(self.cone_r_out, lambda z: L.duct_r, L.z_vane_section_top, L.z_edf_shoulder, 40,
                              self.fill_factor, "shoulder ring")
        sol += revolved_shell(self.cone_r_out, lambda z: max(self.cone_r_out(z) - L.cone_wall, L.sleeve_r),
                              L.z_edf_shoulder, L.cone_z1, 120, None, "cone shell")
        sol.append(tube_z(L.edf_bore_r, L.sleeve_r, L.z_sleeve_bottom, L.z_prop_spigot_bottom, label="sleeve"))
        spigot_r = L.spigot_od / 2
        sol.append(tube_z(L.edf_bore_r, spigot_r, L.z_prop_spigot_bottom, L.z_prop_spigot_top,
                          fill=self.fill_factor(spigot_r - L.edf_bore_r), label="spigot ring"))
        # webs: sleeve OD -> cone inner surface (v0.5 tabs: cone_outer_radius = body_r above cone_z1, so the
        # webs follow the cylindrical skin there - SPEC s3.7 "Webs follow it")
        n = 60
        dz = (L.z_prop_spigot_bottom - L.z_sleeve_bottom) / n
        for th in L.web_thetas:
            for i in range(n):
                za = L.z_sleeve_bottom + i * dz
                z = za + dz / 2
                dr = (self.cone_r_out(z) - L.cone_wall) - L.sleeve_r
                if dr <= 0:
                    continue
                sol.append(radial_box(L.sleeve_r, L.sleeve_r + dr, -L.web_t / 2, L.web_t / 2, za, za + dz, th, label="web"))
        # hinge bosses (outside the duct wall) minus the bore through boss + wall
        for th in L.vane_thetas:
            u0, u1 = L.vane_section_r, L.hinge_boss_r_out
            vol = (math.pi / 4 * L.hinge_boss_od**2 * (u1 - u0)
                   - math.pi / 4 * L.hinge_bore_d**2 * (L.hinge_boss_r_out - L.duct_r))
            sol.append(radial_box(u0, u1, -L.hinge_boss_od / 2, L.hinge_boss_od / 2,
                                  L.vane_hinge_z - L.hinge_boss_od / 2, L.vane_hinge_z + L.hinge_boss_od / 2, th,
                                  volume=vol, label="hinge boss"))
        # servo pods (pushrod mode) / v0.9 fin-root fairings with keels and lead pipes (direct drive)
        if getattr(L, "direct_drive", False):
            sol += self.fairing_solids()
        else:
            for th in L.pod_thetas:
                sol += self._pod(th, L.vane_section_r, L.pod_r_out, L.pod_w, L.pod_z0, L.pod_rim_z,
                                 L.pod_cavity_r_in, L.pod_cavity_r_out, L.pod_cavity_w, L.pod_floor_top_z,
                                 L.z_exit, "servo pod")
        # fins: exposed trapezoid from the duct outer wall to the tip
        a_exp, b, s, z_le_root_exp = self.fin_exposed_planform()
        area = (a_exp + b) / 2 * s
        r_bar = L.vane_section_r + s * (a_exp + 2 * b) / (3 * (a_exp + b))
        z_bar = L.fin_te_z + (a_exp**2 + a_exp * b + b**2) / (3 * (a_exp + b))
        c_mean = (a_exp + b) / 2
        for th in L.fin_thetas:
            sol.append(radial_box(r_bar - s / 2, r_bar + s / 2, -L.fin_t / 2, L.fin_t / 2,
                                  z_bar - c_mean / 2, z_bar + c_mean / 2, th, volume=area * L.fin_t, label="fin"))
        if L.tabs:
            sol += self.tab_mount_solids()
        return sol

    # -- v0.9 direct drive (vane_drive.mode 'direct') ------------------------
    def fairing_solids(self) -> list[Solid]:
        """Fin-root fairings (Layout.fair_*): box from the duct surface out - pocket (closed top) - service opening under
        the lintel - lower-ear-tab slot + 45 deg keel; minus the fin plate counted inside them (the fin estimate covers
        the whole planform); plus each lead pipe (tube O sl_pipe_d - O sl_tunnel_d along P0 -> Pe) and half its web pipe
        (the other half lies in the duct wall / shoulder ring)."""
        L = self.lay
        sol: list[Solid] = []
        u0, u1 = L.vane_section_r, L.fair_u[1]
        hv, pv = L.fair_hv, L.dd_pocket_hv
        kd = L.fair_z[0] - L.fair_keel_z0
        for k, th in enumerate(L.fin_thetas):
            pu0, pu1 = L.dd_pocket_u
            sol.append(radial_box(u0, u1, -hv, hv, L.fair_z[0], L.fair_z[1], th, label="fairing box"))
            sol.append(radial_box(pu0, pu1, -pv, pv, L.dd_pocket_z[0], L.dd_pocket_z[1], th,
                                  volume=-(pu1 - pu0) * 2 * pv * (L.dd_pocket_z[1] - L.dd_pocket_z[0]), label="pocket"))
            sol.append(radial_box(pu0, pu1, pv, hv, L.dd_pocket_z[0], L.dd_open_z1, th,
                                  volume=-(pu1 - pu0) * (hv - pv) * (L.dd_open_z1 - L.dd_pocket_z[0]), label="opening"))
            su0, su1 = L.dd_slot_u
            sol.append(radial_box(su0, su1, -pv, hv, L.dd_slot_z[0], L.dd_slot_z[1], th,
                                  volume=-(su1 - su0) * (hv + pv) * (L.dd_slot_z[1] - L.dd_slot_z[0]), label="ear slot"))
            sol.append(radial_box(u0, u1, -hv, hv, L.fair_keel_z0, L.fair_z[0], th,
                                  volume=(hv - L.fin_t / 2) * kd * (u1 - u0) + L.fin_t * kd * (u1 - u0), label="keel"))
            sol.append(radial_box(u0, u1, -L.fin_t / 2, L.fin_t / 2, L.fair_keel_z0, L.fair_z[1], th,
                                  volume=-L.fin_t * (u1 - u0) * (L.fair_z[1] - L.fair_keel_z0), label="fin inside the fairing"))
            if L.servo_leads:
                a, b = L.sl_pipe_points(k)
                ln = math.dist(a, b)
                c = tuple(0.5 * (a[i] + b[i]) for i in range(3))
                vol = math.pi / 4 * (L.sl_pipe_d ** 2 - L.sl_tunnel_d ** 2) * ln + 0.5 * math.pi / 4 * L.sl_pipe_d ** 2 * ln
                sol.append(Solid(vol, c, (ln * ln / 12.0, ln * ln / 12.0, L.sl_pipe_d ** 2 / 8.0), 1.0, "lead pipe + web"))
        return sol

    def vane_coupler(self, theta: float) -> list[Solid]:
        L = self.lay
        zh = L.vane_hinge_z
        hub = math.pi / 4 * (L.dd_hub_d ** 2 - L.dd_bore_d ** 2) * (L.dd_u_hub1 - L.dd_u_hub0)
        # v0.9 fix round: the flange is the horn socket - hub relief bored out, the arm slot across it
        rel = getattr(L, "dd_collar_relief_d", 0.0)
        slot = 2 * getattr(L, "dd_slot_hw", 0.0) * (L.dd_flange_d - rel)
        fl = (math.pi / 4 * (L.dd_flange_d ** 2 - rel ** 2) - slot) * (L.dd_u_flange1 - L.dd_u_hub1)
        r1, r2 = L.dd_hub_d / 2, L.dd_flange_d / 2
        return [radial_box(L.dd_u_hub0, L.dd_u_hub1, -r1, r1, zh - r1, zh + r1, theta, volume=hub, label="coupler hub"),
                radial_box(L.dd_u_hub1, L.dd_u_flange1, -r2, r2, zh - r2, zh + r2, theta, volume=fl, label="coupler flange")]

    def servo_cover(self, theta: float) -> list[Solid]:
        L = self.lay
        holes = len(L.cov_screws_uz) * math.pi / 4 * L.cov_d_clear ** 2 * L.cov_t
        return [radial_box(L.cov_u[0], L.cov_u[1], L.cov_v[0], L.cov_v[1], L.cov_z[0], L.cov_z[1], theta,
                           volume=(L.cov_u[1] - L.cov_u[0]) * L.cov_t * (L.cov_z[1] - L.cov_z[0]) - holes, label="servo cover")]

    def wire_cover(self) -> list[Solid]:
        L = self.lay
        holes = 2 * len(L.wire_pad_zs) * math.pi / 4 * L.wire_cov_d_clear ** 2 * L.wire_cov_t
        return [radial_box(L.wire_cov_u[0], L.wire_cov_u[1], -L.wire_cov_hv, L.wire_cov_hv, L.wire_cov_z[0], L.wire_cov_z[1],
                           L.wire_theta, volume=L.wire_cov_t * 2 * L.wire_cov_hv * (L.wire_cov_z[1] - L.wire_cov_z[0]) - holes,
                           label="wire cover")]

    # -- v0.5 tab mount (SPEC s3.7) ------------------------------------------
    def lug_sectors(self) -> list[tuple[float, float]]:
        """(a0, a1) deg (a1 > a0) of the screw lugs of each tab: [th - hs*block_theta_far, th + hs*block_theta_head]."""
        L = self.lay
        hs = L.tab_screw_head_side
        return [tuple(sorted((th - hs * L.block_theta_far, th + hs * L.block_theta_head))) for th in L.tab_thetas]

    def lug_fill(self) -> float:
        """Lug between sleeve and skin: one solid region edf_bore_r..body_r with the sleeve and skin counted."""
        L = self.lay
        return self.inner_fill(L.body_r - L.edf_bore_r, (L.sleeve_r - L.edf_bore_r) + L.cone_wall)

    def liner_layers(self) -> list[tuple[float, float, float, float, float, str]]:
        """Thin liner (SPEC s3.7): duct tube r [duct_r, liner_r_out] over [z_edf_shoulder, z_liner_top], plus the
        part of the top ring r [duct_r, sleeve_r] over [z_liner_ring_bottom, z_liner_top] that the tube and the
        sleeve do not already hold (r [liner_r_out, edf_bore_r]).  Each prints solid when it is thinner than two
        shells (the ring with the tube and sleeve is one plate z_liner_top - z_liner_ring_bottom thick)."""
        L = self.lay
        ring_h = L.z_liner_top - L.z_liner_ring_bottom
        return [(L.duct_r, L.liner_r_out, L.z_edf_shoulder, L.z_liner_top, self.fill_factor(L.liner_wall), "liner tube"),
                (L.liner_r_out, L.edf_bore_r, L.z_liner_ring_bottom, L.z_liner_top,
                 self.fill_factor(min(ring_h, L.sleeve_r - L.duct_r)), "liner ring")]

    def prop_layers(self, theta: float, lugs: bool = True) -> list[tuple[float, float, float, float, float, str]]:
        """Solid annular layers (r_in, r_out, z0, z1, fill, name) of the EDF section at azimuth theta,
        as modelled above (the webs are kept clear of every cut by Layout); ``lugs`` False: no screw lugs (the
        wire09c clock mount has none)."""
        L = self.lay
        spigot_r = L.spigot_od / 2
        r_skin = L.body_r - L.cone_wall
        layers = [(L.edf_bore_r, L.sleeve_r, L.z_sleeve_bottom, L.z_prop_spigot_bottom, 1.0, "sleeve"),
                  (L.edf_bore_r, spigot_r, L.z_prop_spigot_bottom, L.z_prop_spigot_top,
                   self.fill_factor(spigot_r - L.edf_bore_r), "spigot ring"),
                  (r_skin, L.body_r, L.cone_z1, L.z_prop_spigot_bottom, 1.0, "cylindrical skin")]
        layers += self.liner_layers()
        for a0, a1 in (self.lug_sectors() if lugs else []):
            if (theta - a0) % 360.0 <= (a1 - a0):
                layers += [(L.block_r_in, r_skin, z0, z1, self.lug_fill(), "screw lug") for z0, z1 in L.boss_z_ranges]
        return layers

    def radial_cut(self, layers, u0: float, u1: float, v_half: float, z0: float, z1: float, theta: float,
                   label: str) -> list[Solid]:
        """Negative solids of a radial box cut through every layer it overlaps."""
        out = []
        for a, b, za, zb, fill, name in layers:
            s = clipped_radial_box(u0, u1, v_half, a, b, max(z0, za), min(z1, zb), theta, sign=-1.0, fill=fill,
                                   label=f"{label} ({name})")
            if s is not None:
                out.append(s)
        return out

    def tab_screw_holes(self) -> list[tuple[float, float, float, float, float, str]]:
        """(theta_tab, z, d, t0, t1, name) of every tab-screw hole segment along its tangential axis (SPEC s3.7):
        clearance hole [t_keyway, t_head], counterbore [t_head, t_cbore_end], tap hole [t_tap_end, -t_keyway]."""
        L = self.lay
        segs = []
        for th in L.tab_thetas:
            for z in L.tab_screw_zs:
                segs += [(th, z, L.tab_screw_d_clear, L.tab_screw_t_keyway, L.tab_screw_t_head, "clearance hole"),
                         (th, z, L.tab_screw_d_cbore, L.tab_screw_t_head, L.tab_screw_t_cbore_end, "counterbore"),
                         (th, z, L.tab_screw_d_tap, L.tab_screw_t_tap_end, -L.tab_screw_t_keyway, "tap hole")]
        return segs

    def tab_screw_hole_cuts(self) -> list[Solid]:
        """Material each hole segment removes from the layers at its tab: the sleeve, the lug and the skin
        (``tangential_bore_in_annulus``; the counterbore removes nothing beyond the outer skin).  The keyway
        (|t| <= t_keyway) is already void and no segment enters it.  Layout centres the lugs on the screws, so a
        hole lies wholly inside or wholly outside each layer's Z range; anything else is a contract change."""
        L = self.lay
        hs = L.tab_screw_head_side
        out: list[Solid] = []
        for th, z, d, t0, t1, name in self.tab_screw_holes():
            rho = d / 2
            for a, b, za, zb, fill, lname in self.prop_layers(th):
                if zb <= z - rho or za >= z + rho:
                    continue
                if za > z - rho + 1e-9 or zb < z + rho - 1e-9:
                    raise ValueError(f"tab screw {name} at z={z:.2f} crosses the Z edge of the {lname}")
                vol, uc, tc = tangential_bore_in_annulus(L.tab_screw_r, rho, t0, t1, a, b)
                if vol <= 0.0:
                    continue
                le = vol / (math.pi * rho * rho)          # equivalent length (inertia of a box d x le x d)
                out.append(radial_box(uc - rho, uc + rho, hs * tc - le / 2, hs * tc + le / 2, z - rho, z + rho, th,
                                      fill=fill, volume=-vol, label=f"tab screw {name} ({lname})"))
        return out

    def tab_mount_solids(self) -> list[Solid]:
        """Cylindrical skin, thin liner, screw lugs, keyways, tab-screw holes and the two-box wire slot (SPEC s3.7).
        wire09c clock mount (``Layout.clock_mount``): skin, liner and ``clock_mount_solids`` instead."""
        L = self.lay
        r_skin = L.body_r - L.cone_wall
        sol = [tube_z(r_skin, L.body_r, L.cone_z1, L.z_prop_spigot_bottom, label="cylindrical skin")]
        sol += [tube_z(a, b, za, zb, fill=fill, label=name) for a, b, za, zb, fill, name in self.liner_layers()]
        if getattr(L, "clock_mount", False):
            return sol + self.clock_mount_solids()
        # lugs: Layout keeps them inside the cylindrical section (block_z0 >= z_cyl0 + 1), so the skin inner
        # radius is constant; sleeve and skin are already counted.  The lowest one reaches floor_t below keyway_z0
        # (the keyway floor), the highest merges with the (unmodelled) deck.
        for a0, a1 in self.lug_sectors():
            for z0, z1 in L.boss_z_ranges:
                sol.append(sector_z(L.block_r_in, r_skin, a0, a1, z0, z1, fill=self.lug_fill(), label="screw lug"))
        for th in L.tab_thetas:
            sol += self.radial_cut(self.prop_layers(th), L.keyway_r_in, L.keyway_r_out, L.keyway_w / 2,
                                   L.keyway_z0, L.keyway_z1, th, "keyway")
        sol += self.tab_screw_hole_cuts()
        # wire slot: (a) stops short of the skin below the cone-shell top, (b) breaks through the spigot ring
        layers = self.prop_layers(L.wire_theta)
        sol += self.radial_cut(layers, L.wire_slot_r_in, L.wire_slot_lower_r_out, L.wire_slot_w / 2,
                               L.wire_slot_z0, L.z_prop_spigot_bottom, L.wire_theta, "wire slot lower")
        sol += self.radial_cut(layers, L.wire_slot_r_in, L.wire_slot_r_out, L.wire_slot_w / 2,
                               L.z_prop_spigot_bottom, L.wire_slot_z1, L.wire_theta, "wire slot upper")
        if getattr(L, "ml_low_cut_uz", None):
            # v0.9 lead-exit round (2026-10-29): lead notch under the housing rim + lower slot (Layout.ml_low_cut_uz) as
            # two radial boxes of the polygon's area (the 45 deg chamfer split at its mid height), up to the 22 mm box
            (u_in, z_b), (u_b, _), (u_o, z_ct), _, _ = L.ml_low_cut_uz
            hv, z_mid = L.ml_low_cut_w / 2, 0.5 * (z_b + z_ct)
            sol += self.radial_cut(layers, u_in, u_b, hv, z_b, L.wire_slot_z0, L.wire_theta, "lead notch")
            sol += self.radial_cut(layers, u_b, u_o, hv, z_mid, L.wire_slot_z0, L.wire_theta, "lower slot")
        return sol

    # -- wire09c clock mount (simfix 2026-11-04) -------------------------------
    def clock_annuli(self, z: float) -> list[tuple[float, float, float]]:
        """(r_in, r_out, fill) of every axisymmetric solid of propulsion_module() at height z (duct wall, shoulder
        ring, cone shell, cylindrical skin, sleeve, spigot ring, liner tube / ring)."""
        L = self.lay
        cr = self.cone_r_out
        spigot_r = L.spigot_od / 2
        (_, _, _, _, f_lt, _), (_, _, _, _, f_lr, _) = self.liner_layers()
        rows = []
        if L.z_exit <= z < L.z_vane_section_top:
            rows.append((L.duct_r, L.vane_section_r, 1.0))
        if L.z_vane_section_top <= z < L.z_edf_shoulder:
            rows.append((L.duct_r, cr(z), self.fill_factor(cr(z) - L.duct_r)))
        if L.z_edf_shoulder <= z < L.cone_z1:
            rows.append((max(cr(z) - L.cone_wall, L.sleeve_r), cr(z), 1.0))
        if L.cone_z1 <= z < L.z_prop_spigot_bottom:
            rows.append((L.body_r - L.cone_wall, L.body_r, 1.0))
        if L.z_sleeve_bottom <= z < L.z_prop_spigot_bottom:
            rows.append((L.edf_bore_r, L.sleeve_r, 1.0))
        if L.z_prop_spigot_bottom <= z < L.z_prop_spigot_top:
            rows.append((L.edf_bore_r, spigot_r, self.fill_factor(spigot_r - L.edf_bore_r)))
        if L.z_edf_shoulder <= z < L.z_liner_top:
            rows.append((L.duct_r, L.liner_r_out, f_lt))
        if L.z_liner_ring_bottom <= z < L.z_liner_top:
            rows.append((L.liner_r_out, L.edf_bore_r, f_lr))
        return [r for r in rows if r[1] > r[0]]

    def clock_plan_solids(self) -> list[Solid]:
        """The lead-passage blister (Layout.lp_blister_xy, lp_z_blister_bottom .. lp_z_blister_top, solid), its 45 deg
        V keel under the floor (Layout.lp_keel; the part above the floor lies in the blister) and the passage cut
        (Layout.lp_fan_xy, lp_z_fan_floor .. z_prop_top), in Z slabs (0.5 mm where a radius or the keel width varies,
        else one): added = blister / keel; removed = the annuli they cover (at the annulus fill), then the passage's part
        of the blister (outside the EDF bore) and of the annuli inside the bore radius, or above the blister top the
        annuli it crosses."""
        L = self.lay
        B, F = ccw_polygon(L.lp_blister_xy), ccw_polygon(L.lp_fan_xy)
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
        cache: dict = {}

        def dm(key, poly, R):
            if key is None:
                return polygon_disc_moments(poly, R)
            if (key, R) not in cache:
                cache[(key, R)] = polygon_disc_moments(poly, R)
            return cache[(key, R)]

        def ann(key, poly, a, b):
            return _vadd(dm(key, poly, b), dm(key, poly, a), -1.0) if a > 0.0 else dm(key, poly, b)

        out: list[Solid] = []
        for za, zb in zip(zs[:-1], zs[1:]):
            zm0 = 0.5 * (za + zb)
            varies = zm0 < zbb or L.z_vane_section_top <= zm0 < L.cone_z1
            n = max(1, math.ceil((zb - za) / 0.5)) if varies else 1
            for i in range(n):
                z0, z1 = za + i * (zb - za) / n, za + (i + 1) * (zb - za) / n
                zm = 0.5 * (z0 + z1)
                annuli = self.clock_annuli(zm)
                adds = []
                if zbb <= zm < zbt:
                    adds.append(("B", B, "blister"))
                elif zm < zbb:
                    for u0, u1, hk, zk in keel:
                        if zm > zk:
                            w = hk * (zm - zk) / (hk + ov)
                            adds.append((None, ccw_polygon([(u * cs - v * sn, u * sn + v * cs)
                                                            for u, v in ((u0, -w), (u1, -w), (u1, w), (u0, w))]), "keel"))
                rem: dict[float, list[float]] = {}

                def take(fill: float, mom: list[float]) -> None:
                    rem[fill] = _vadd(rem.get(fill, [0.0] * 6), mom)

                for key, P, label in adds:
                    s_ = plan_slab(dm(key, P, math.inf), z0, z1, 1.0, 1.0, label)
                    if s_ is not None:
                        out.append(s_)
                    for a, b, f in annuli:
                        take(f, ann(key, P, a, b))
                if zff <= zm < ztop:
                    if zbb <= zm < zbt:
                        take(1.0, _vadd(dm("F", F, math.inf), dm("F", F, L.edf_bore_r), -1.0))
                        for a, b, f in annuli:
                            if a < L.edf_bore_r:
                                take(f, ann("F", F, a, min(b, L.edf_bore_r)))
                    else:
                        for a, b, f in annuli:
                            take(f, ann("F", F, a, b))
                for f, mom in rem.items():
                    s_ = plan_slab(mom, z0, z1, -1.0, f, "clock plan cut")
                    if s_ is not None:
                        out.append(s_)
        return out

    def clock_mount_solids(self) -> list[Solid]:
        """wire09c clock mount as rocket_geometry.propulsion_module builds it (no lugs, tab screws or 22 mm slot): the
        keyways round the circle (Layout.ck_keyways) cut through the lug-free layers; one floor pad per keyway
        (Layout.ck_pad_rz clamped to the sleeve -> skin cavity, ck_pad_w wide); the servo-lead column guide walls
        (Layout.ck_col_walls, over the cavity from the shoulder top to the deck bottom - their channel is that cavity);
        the blister, keel and lead passage (``clock_plan_solids``).  Not modelled (E): the deck they also cut, the
        blister wall's lap onto the next column guide wall, the keel's touch on the fin root, the servo-lead cutters."""
        L = self.lay
        sol: list[Solid] = []
        for th in L.ck_keyways:
            sol += self.radial_cut(self.prop_layers(th, lugs=False), L.keyway_r_in, L.keyway_r_out, L.ck_kw / 2,
                                   L.lp_z_floor, L.keyway_z1, th, "clock keyway")

        def rin(z: float) -> float:
            return self.cone_r_out(z) - L.cone_wall

        def plate(poly: list[tuple[float, float]], t: float, theta: float, v_off: float, label: str) -> Solid:
            """Prism of the (u, z) polygon, t thick along v, mid plane at v = v_off (diagonal k, plate axes)."""
            area, uc, zc, suu, szz = polygon_props(poly)
            x, y = pod_frame(uc, v_off, theta)
            ku, kv, kz = suu / area, t * t / 12.0, szz / area
            c2, s2 = math.cos(math.radians(theta)) ** 2, math.sin(math.radians(theta)) ** 2
            k = ((kv + kz) * c2 + (ku + kz) * s2, (kv + kz) * s2 + (ku + kz) * c2, ku + kv)
            return Solid(area * t, (x, y, zc), k, self.fill_factor(t), label)

        pad = [(min(max(u, L.sleeve_r), rin(z)), z) for u, z in L.ck_pad_rz]
        sol += [plate(pad, L.ck_pad_w, th, 0.0, "clock floor pad") for th in L.ck_keyways]
        z0, z1 = L.z_edf_shoulder, L.z_prop_spigot_bottom - L.body_wall
        skin = ([(L.cone_outer_radius(z0), z0)] + [(r, z) for r, z in L.outer_skin_profile() if z0 < z < z1]
                + [(L.cone_outer_radius(z1), z1)])
        chan = [(L.sleeve_r, z0)] + [(r - L.cone_wall, z) for r, z in skin] + [(L.sleeve_r, z1)]
        sol += [plate(chan, v1 - v0, c, 0.5 * (v0 + v1), "column guide wall") for c, v0, v1 in L.ck_col_walls]
        return sol + self.clock_plan_solids()

    def fin_exposed_planform(self) -> tuple[float, float, float, float]:
        """(root chord at the duct outer wall, tip chord, exposed semispan, LE z at the exposed root)."""
        L = self.lay
        frac = (L.vane_section_r - L.fin_root_r) / (L.fin_tip_r - L.fin_root_r)
        z_le_root_exp = L.fin_root_le_z + (L.fin_tip_le_z - L.fin_root_le_z) * frac
        a_exp = z_le_root_exp - L.fin_te_z
        return a_exp, L.fin_tip_chord, L.fin_tip_r - L.vane_section_r, z_le_root_exp

    def intake_module(self) -> list[Solid]:
        """SPEC s5.1.  v0.6.1: the windowed band Z [z_shelf_top, z_intake_bulkhead_bottom] is r_band_in..body_r
        (window_wall_t, at its fill); the struts are the radial-sided sectors strut_angle each, so the band over the
        windows is exactly the strut fraction strut_count x strut_angle / 360 of that ring."""
        L = self.lay
        im = self.p["intake_module"]
        ri, ro = L.body_id / 2, L.body_r
        rb, fb = L.r_band_in, self.fill_factor(L.window_wall_t)
        sol = [tube_z(ri, ro, L.z_intake_bottom, L.z_intake_skirt_top, label="skirt"),
               tube_z(L.lip_pocket_id / 2, ro, L.z_intake_skirt_top, L.z_lip_pocket_top,
                      fill=self.fill_factor(ro - L.lip_pocket_id / 2), label="lip pocket ring"),
               tube_z(L.shelf_id / 2, ro, L.z_lip_pocket_top, L.z_shelf_top,
                      fill=self.fill_factor(ro - L.shelf_id / 2), label="shelf"),
               tube_z(rb, ro, L.z_shelf_top, L.z_window_bottom, fill=fb, label="ring below windows"),
               tube_z(rb, ro, L.z_window_bottom, L.z_window_top, fill=fb,
                      frac=L.strut_count * L.strut_angle_deg / 360.0, label="struts"),
               tube_z(rb, ro, L.z_window_top, L.z_intake_bulkhead_bottom, fill=fb, label="ring above windows"),
               tube_z(0.0, ro, L.z_intake_bulkhead_bottom, L.z_intake_bulkhead_top, label="bulkhead"),
               tube_z(0.0, L.cable_hole_d / 2, L.z_intake_bulkhead_bottom, L.z_intake_bulkhead_top,
                      frac=-1.0, label="cable hole")]
        for th in L.vent_hole_thetas:
            x, y = pod_frame(L.vent_hole_r, 0.0, th)
            sol.append(box((x, y, (L.z_intake_bulkhead_bottom + L.z_intake_bulkhead_top) / 2),
                           (L.vent_hole_d, L.vent_hole_d, im["bulkhead_t"]),
                           volume=-math.pi / 4 * L.vent_hole_d**2 * im["bulkhead_t"], label="vent hole"))
        sol.append(tube_z(L.spigot_id / 2, L.spigot_od / 2, L.z_intake_spigot_bottom, L.z_intake_spigot_top, label="spigot"))
        if getattr(L, "has_window_gable", False):
            sol.append(self.window_gussets())
        if L.tabs:
            sol += self.wire_channel_solids()
        return sol

    def window_gussets(self) -> Solid:
        """v0.6.2 window gussets (intake_module.window_bottom_end): the band material left under every window's truncated
        gable - Layout's exact volume (gusset_volume_mm3, all windows) at the windows' common centroid (gusset_centroid_u
        along each window centre, averaged over window_thetas: on the axis for equally spaced windows; gusset_centroid_z),
        at the band's fill; own inertia that of the band ring over the gable height (an estimate, ~11 g)."""
        L = self.lay
        n = len(L.window_thetas)
        cx = L.gusset_centroid_u * sum(math.cos(math.radians(t)) for t in L.window_thetas) / n
        cy = L.gusset_centroid_u * sum(math.sin(math.radians(t)) for t in L.window_thetas) / n
        return Solid(L.gusset_volume_mm3, (cx, cy, L.gusset_centroid_z), _tube_k(L.r_band_in, L.body_r, L.gable_h_outer),
                     self.fill_factor(L.window_wall_t), "window gussets")

    def wire_notch_z(self) -> tuple[float, float]:
        """Z range of the shelf notch, [wire_notch_z0, wire_ch_z1] (SPEC s5.1).  The SPEC runs it out to
        r = wire_ch_r0 + 0.5 so it fuses with the channel; that outer 0.5 mm is channel void already because the
        notch's Z range lies inside the channel's, so the estimate cuts r [wire_notch_r0, wire_ch_r0] only (and
        refuses a Layout where the notch starts below the channel)."""
        L = self.lay
        if L.wire_notch_z0 < L.wire_ch_z0:
            raise ValueError("wire shelf notch starts below the wire channel: model its overlap explicitly")
        return L.wire_notch_z0, L.wire_ch_z1

    def intake_layers(self) -> list[tuple[float, float, float, float, float, str]]:
        """Solid annular layers (r_in, r_out, z0, z1, fill, name) of the intake module below the windows,
        plus the wire bulge outside body_r (tabs; it spans the whole channel width)."""
        L = self.lay
        ri, ro = L.body_id / 2, L.body_r
        layers = [(ri, ro, L.z_intake_bottom, L.z_intake_skirt_top, 1.0, "skirt"),
                  (L.lip_pocket_id / 2, ro, L.z_intake_skirt_top, L.z_lip_pocket_top,
                   self.fill_factor(ro - L.lip_pocket_id / 2), "lip pocket ring"),
                  (L.shelf_id / 2, ro, L.z_lip_pocket_top, L.z_shelf_top, self.fill_factor(ro - L.shelf_id / 2), "shelf"),
                  (L.r_band_in, ro, L.z_shelf_top, L.z_window_bottom, self.fill_factor(L.window_wall_t),
                   "ring below windows")]   # v0.6.1: window_wall_t band
        if L.tabs and not getattr(L, "wire_ch_open", False):
            layers.append((ro, L.wire_bulge_r_out, L.wire_bulge_z0, L.wire_bulge_z1, 1.0, "wire bulge"))
        return layers

    def wire_floor_wedge(self) -> Solid | None:
        """v0.5d floor wedge (SPEC s5.1): the (r, z) triangle ``wire_ch_floor_rz`` extruded tangentially over
        +-wire_ch_w/2 at wire_theta, unioned AFTER the channel / notch cuts.  New material is only its part inside
        the channel void (u <= wire_ch_r1; Layout puts the whole triangle above wire_ch_z0 and outside wire_ch_r0);
        the rest (u in (wire_ch_r1, wire_ch_r1 + 0.5]) lies in the bulge's outer wall, which is solid already.  A
        triangle that leaves the void anywhere else would overlap material this estimate does not track, so it is
        refused (contract change)."""
        L = self.lay
        tri = [(float(u), float(z)) for u, z in L.wire_ch_floor_rz]
        us, zs = [u for u, _ in tri], [z for _, z in tri]
        if (min(us) < L.wire_ch_r0 - 1e-9 or min(zs) < L.wire_ch_z0 - 1e-9 or max(zs) > L.wire_ch_z1 + 1e-9
                or min(zs) < L.wire_bulge_z0 or max(zs) > L.wire_bulge_z1 or max(us) > L.wire_bulge_r_out + 1e-9):
            raise ValueError("wire channel floor wedge leaves the channel void / bulge wall: model its overlap explicitly")
        poly = clip_polygon(tri, 0, L.wire_ch_r1, keep_above=False)
        area, uc, zc, suu, szz = polygon_props(poly)
        if area <= 0.0:
            return None
        w = L.wire_ch_w
        du, dz = math.sqrt(12.0 * suu / area), math.sqrt(12.0 * szz / area)   # box with the same second moments
        longest = max(math.dist(poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly)))   # wedge thickness
        # normal to its sloped face = 2 A / hypotenuse (the slicer's wall loops fill it when < 2 shells)
        return radial_box(uc - du / 2, uc + du / 2, -w / 2, w / 2, zc - dz / 2, zc + dz / 2, L.wire_theta,
                          fill=self.fill_factor(2.0 * area / longest), volume=area * w, label="wire channel floor wedge")

    def wire_channel_solids(self) -> list[Solid]:
        """Wire bulge (outside body_r), channel void through skirt / pocket ring / shelf / ring / bulge, shelf
        notch, bulkhead wire hole (SPEC s5.1).  The bulge is unioned before the cuts, so it closes the channel
        below wire_ch_z0 and above wire_ch_z1; v0.5d: the floor wedge is unioned after them."""
        L = self.lay
        if getattr(L, "wire_ch_open", False):
            return self._open_groove_solids()
        hb = L.wire_bulge_w / 2
        # the bulge box starts inside the wall (SPEC: body_r - 0.6); only its part outside body_r is new material
        u_chord = math.sqrt(L.body_r ** 2 - hb ** 2)
        sol = [s for s in (clipped_radial_box(u_chord, L.wire_bulge_r_out, hb, L.body_r, math.inf, L.wire_bulge_z0,
                                              L.wire_bulge_z1, L.wire_theta, label="wire bulge"),) if s is not None]
        layers = self.intake_layers()
        sol += self.radial_cut(layers, L.wire_ch_r0, L.wire_ch_r1, L.wire_ch_w / 2, L.wire_ch_z0,
                               L.wire_ch_z1, L.wire_theta, "wire channel")
        sol += self.radial_cut(layers, L.wire_notch_r0, L.wire_ch_r0, L.wire_ch_w / 2, *self.wire_notch_z(),
                               L.wire_theta, "wire shelf notch")
        hole = clipped_radial_box(L.wire_hole_r0, L.wire_hole_r1, L.wire_ch_w / 2, 0.0, L.body_r,
                                  L.z_intake_bulkhead_bottom, L.z_intake_bulkhead_top, L.wire_theta, sign=-1.0,
                                  label="bulkhead wire hole")
        if hole is not None:
            sol.append(hole)
        wedge = self.wire_floor_wedge()
        if wedge is not None:
            sol.append(wedge)
        return sol

    def _open_groove_solids(self) -> list[Solid]:
        """v0.9 open motor-lead groove (intake wire_channel.open): two side walls outside body_r, the roof wedge
        (Layout.wire_roof_rz over the bulge width), the groove cut through the skirt / pocket ring / shelf outside
        wire_ch_r0 (to the shelf top), the gate (gate_w) through the shelf / ring, the bulkhead wire hole."""
        L = self.lay
        hw, sw = L.wire_ch_w / 2, L.wire_side_wall
        sol: list[Solid] = []
        for s in (1.0, -1.0):
            v0, v1 = (hw, hw + sw) if s > 0 else (-hw - sw, -hw)
            sol.append(radial_box(L.body_r, L.wire_ch_r1, v0, v1, L.z_intake_bottom, L.wire_ch_z1, L.wire_theta,
                                  fill=self.fill_factor(sw), label="groove side wall"))
        area, uc, zc, suu, szz = polygon_props([(float(u), float(z)) for u, z in L.wire_roof_rz])
        du, dz = math.sqrt(12.0 * suu / area), math.sqrt(12.0 * szz / area)
        sol.append(radial_box(uc - du / 2, uc + du / 2, -hw - sw, hw + sw, zc - dz / 2, zc + dz / 2, L.wire_theta,
                              volume=area * (2 * (hw + sw)), label="groove roof wedge"))
        layers = self.intake_layers()
        sol += self.radial_cut(layers, L.wire_ch_r0, L.body_r, hw, L.z_intake_bottom, L.wire_groove_z1, L.wire_theta,
                               "open groove")
        # the gate: below the groove top only from the bore to wire_ch_r0 (outside it the groove already removed the
        # material), above it through the whole ring out to body_r
        zg = min(max(L.wire_groove_z1, L.wire_notch_z0), L.wire_ch_z1)
        sol += self.radial_cut(layers, L.wire_notch_r0, L.wire_ch_r0, L.wire_gate_w / 2, L.wire_notch_z0, zg,
                               L.wire_theta, "gate")
        sol += self.radial_cut(layers, L.wire_notch_r0, L.body_r, L.wire_gate_w / 2, zg, L.wire_ch_z1, L.wire_theta, "gate")
        hole = clipped_radial_box(L.wire_hole_r0, L.wire_hole_r1, hw, 0.0, L.body_r, L.z_intake_bulkhead_bottom,
                                  L.z_intake_bulkhead_top, L.wire_theta, sign=-1.0, label="bulkhead wire hole")
        if hole is not None:
            sol.append(hole)
        return sol

    def tray_stop_blocks(self) -> list[Solid]:
        """v0.5d tray stop (SPEC s5.2, both presets): in each rail slot a block between the two ribs - x across the
        slot gap tray_x +- rail_gap/2, the ribs' radial extent (same box convention as the ribs: y from
        rail_y - rail_depth to rail_y), Z [z_tray_stop_bottom, z_tray_bottom].  Empty when params carry no stop."""
        L = self.lay
        if not getattr(L, "tray_stop_h", None):
            return []
        return [box((L.tray_x, sy * (L.rail_y - L.rail_depth / 2), (L.z_tray_stop_bottom + L.z_tray_bottom) / 2),
                    (L.rail_gap, L.rail_depth, L.z_tray_bottom - L.z_tray_stop_bottom), label="tray stop")
                for sy in (-1, 1)]

    def livery_inlay(self, sign: float = 1.0) -> list[Solid]:
        """livery (2026-11-04, ``Layout.has_livery``): the orange inlay of each copy as a thin radial box over the logo's
        bounding box at the copy azimuth, volume = logo area x depth (exact for the parallel-sided inlay to second order);
        sign -1: the same volumes removed from the avionics module (its pocket)."""
        L = self.lay
        (s0, s1), (z0, z1) = L.livery_s_range, L.livery_z_range
        out = []
        for th in L.livery_thetas:
            out.append(radial_box(L.livery_r_in, L.livery_r, s0, s1, z0, z1, th,
                                  volume=sign * L.livery_area_mm2 * L.livery_depth, label="logo inlay"))
        return out

    def avionics_module(self) -> list[Solid]:
        L = self.lay
        ri, ro = L.body_id / 2, L.body_r
        sol = [tube_z(ri, ro, L.z_avionics_bottom, L.z_avionics_spigot_bottom, label="tube"),
               tube_z(L.spigot_id / 2, L.spigot_od / 2, L.z_avionics_spigot_bottom, L.z_avionics_spigot_top, label="spigot")]
        if getattr(L, "has_livery", False):        # livery 2026-11-04: the logo pocket (the inlay is its own part)
            sol += self.livery_inlay(-1.0)
        for th in L.vent_slot_thetas:
            sol.append(radial_box(ri, ro, -L.vent_slot_w / 2, L.vent_slot_w / 2, L.z_vent_slot_bottom,
                                  L.z_vent_slot_bottom + L.vent_slot_h, th,
                                  volume=-L.body_wall * L.vent_slot_w * L.vent_slot_h, label="vent slot"))
        for sx in (-1, 1):
            for sy in (-1, 1):
                x = L.tray_x + sx * (L.rail_gap / 2 + L.rail_w / 2)
                y = sy * (L.rail_y - L.rail_depth / 2)
                sol.append(box((x, y, (L.z_rail_bottom + L.z_rail_top) / 2),
                               (L.rail_w, L.rail_depth, L.z_rail_top - L.z_rail_bottom), label="tray rail"))
        sol += self.tray_stop_blocks()
        sol += self.gns_boss_solids()
        return sol

    def gns_boss_solids(self) -> list[Solid]:
        """v0.8.1 (``Layout.has_gns``): the four GNS-board bosses on the tube wall - per boss the diamond (half diagonal
        gns_boss_hd) from the tip u gns_u_back to the bore, its volume the integral over v of the diamond's height
        2 (hd - |v - v_h|) times the length u_wall(v) - u_back (Simpson), minus the blind tap hole (inside the boss).  Thin
        boss walls print solid (fill 1).  Empty without the GNS board."""
        L = self.lay
        if not getattr(L, "has_gns", False):
            return []
        hd, sol = L.gns_boss_hd, []
        for v_h, z_h in L.gns_hole_vz:
            n = 40
            h = 2 * hd / n
            acc = 0.0
            for k in range(n + 1):
                v = v_h - hd + k * h
                f = 2 * (hd - abs(v - v_h)) * max(L.gns_u_wall(v) - L.gns_u_back, 0.0)
                acc += f * (1 if k in (0, n) else (4 if k % 2 else 2))
            vol = acc * h / 3.0
            u1 = L.gns_u_wall(v_h)
            sol.append(radial_box(L.gns_u_back, u1, v_h - hd, v_h + hd, z_h - hd, z_h + hd, L.gns_theta, volume=vol,
                                  label="GNS boss"))
            r = L.gns_hole_d / 2
            sol.append(radial_box(L.gns_hole_u[0], L.gns_hole_u[1], v_h - r, v_h + r, z_h - r, z_h + r, L.gns_theta,
                                  volume=-math.pi * r * r * L.gns_hole_depth, label="GNS boss tap hole"))
        return sol

    def avionics_tray(self) -> list[Solid]:
        L = self.lay
        am = self.p["avionics_module"]
        zc = (L.z_tray_bottom + L.z_tray_top) / 2
        sol = [box((L.tray_x, 0.0, zc), (L.tray_t, L.tray_w, L.tray_h), label="plate")]
        hole_v = -math.pi / 4 * L.fc_hole_d**2 * L.tray_t
        for sy in (-1, 1):
            for sz in (-1, 1):
                sol.append(box((L.tray_x, sy * L.fc_hole_pattern / 2, L.nav_center_z + sz * L.fc_hole_pattern / 2),
                               (L.tray_t, L.fc_hole_d, L.fc_hole_d), volume=hole_v, label="FC hole"))
        slot_v = -am["strap_slot_w"] * am["strap_slot_h"] * L.tray_t
        slot_dims = (L.tray_t, am["strap_slot_w"], am["strap_slot_h"])
        for y in (-40.0, 40.0):  # SPEC s5.2: battery strap slots at y = +-40
            for z in (L.battery_z[0] + 25.0, L.battery_z[1] - 25.0):
                sol.append(box((L.tray_x, y, z), slot_dims, volume=slot_v, label="battery strap slot"))
        for y in (-35.0, 35.0):  # SPEC s5.2: ESC strap slots at y = +-35
            sol.append(box((L.tray_x, y, (L.esc_z[0] + L.esc_z[1]) / 2), slot_dims, volume=slot_v, label="ESC strap slot"))
        return sol

    def payload_module(self) -> list[Solid]:
        L = self.lay
        pl = self.p["payload_module"]
        ri, ro = L.body_id / 2, L.body_r
        sol = [tube_z(ri, ro, L.z_payload_bottom, L.z_payload_spigot_bottom, label="tube"),
               tube_z(L.spigot_id / 2, L.spigot_od / 2, L.z_payload_spigot_bottom, L.z_payload_spigot_top, label="spigot"),
               tube_z(0.0, ri, L.z_payload_bulkhead_bottom, L.z_payload_bulkhead_top, label="bulkhead"),
               tube_z(0.0, L.spring_seat_boss_d / 2, L.z_payload_bulkhead_top,
                      L.z_payload_bulkhead_top + pl["spring_seat_boss_h"], label="spring seat boss")]
        for i in range(pl["bulkhead_hole_count"]):
            th = 45.0 + i * 360.0 / pl["bulkhead_hole_count"]   # SPEC s5.3: theta = 45 + k*90
            x, y = pod_frame(pl["bulkhead_hole_r"], 0.0, th)
            sol.append(box((x, y, (L.z_payload_bulkhead_bottom + L.z_payload_bulkhead_top) / 2),
                           (pl["bulkhead_hole_d"], pl["bulkhead_hole_d"], pl["bulkhead_t"]),
                           volume=-math.pi / 4 * pl["bulkhead_hole_d"]**2 * pl["bulkhead_t"], label="bulkhead hole"))
        sol += self._pod(L.release_pod_theta, L.body_r, L.release_pod_r_out, L.pod_w, L.release_pod_z0,
                         L.release_pod_rim_z, L.release_pod_cavity_r_in, L.release_pod_cavity_r_out, L.pod_cavity_w,
                         L.release_pod_floor_top_z, L.z_payload_bottom, "release pod")
        sol += self.com_mount_solids()
        sol += self.gps_pod_solids()
        return sol

    def gps_pod_solids(self) -> list[Solid]:
        """v0.8 (``Layout.has_gps_pod``): the GPS pod shell, raceway and connector pocket added to the payload module,
        analytic (strips along u): the 45 deg floor (floor_t normal, z_floor_under .. z_floor_top, from the tube skin to
        the outer wall, width 2 cav_hw clipped by the tube), the two side walls (wall_t, from the floor underside to the
        rim), the outer wall, the corner bosses above the floor minus their holes and the corner chamfers, the raceway and
        pocket walls outside the tube.  All thin walls print solid (fill 1).  Empty without the pod."""
        L = self.lay
        if not getattr(L, "has_gps_pod", False):
            return []
        th, R, n = L.gps_theta, L.body_r, 60
        sol: list[Solid] = []
        # floor: strips from where the cavity meets the tube skin (|v| = cav_hw) to the outer wall's inner face
        u0, u1 = math.sqrt(R * R - L.gps_cav_hw ** 2), L.gps_u_cav_out
        du = (u1 - u0) / n
        for i in range(n):
            u = u0 + (i + 0.5) * du
            w = 2 * L.gps_cav_hw if u >= R else 2 * (L.gps_cav_hw - math.sqrt(R * R - u * u))
            zb, zt = L.gps_floor_under_z(u), L.gps_floor_top_z(u)
            sol.append(radial_box(u - du / 2, u + du / 2, -w / 2, w / 2, zb, zt, th, volume=du * w * (zt - zb),
                                  label="GPS pod floor"))
        # side walls (|v| in [cav_hw, w_h]) and the outer wall, floor underside -> rim
        v_mid = 0.5 * (L.gps_cav_hw + L.gps_w_h)
        u0w = math.sqrt(R * R - v_mid ** 2)
        duw = (L.gps_u_out - u0w) / n
        for i in range(n):
            u = u0w + (i + 0.5) * duw
            zb = L.gps_floor_under_z(u)
            for s in (1.0, -1.0):
                sol.append(radial_box(u - duw / 2, u + duw / 2, s * L.gps_cav_hw, s * L.gps_w_h, zb, L.gps_z_rim, th,
                                      volume=duw * L.gps_wall_t * (L.gps_z_rim - zb), label="GPS pod side wall"))
        uo = 0.5 * (L.gps_u_cav_out + L.gps_u_out)
        zo = L.gps_floor_under_z(uo)
        sol.append(radial_box(L.gps_u_cav_out, L.gps_u_out, -L.gps_cav_hw, L.gps_cav_hw, zo, L.gps_z_rim, th,
                              label="GPS pod outer wall"))
        # corner bosses above the floor top, their blind holes, the corner chamfers (triangles c x c / 2 over the height)
        (bu0, bu1), (bv0, bv1) = L.gps_boss_u, L.gps_boss_v
        zb_b = L.gps_floor_top_z(0.5 * (bu0 + bu1))
        c = L.gps_corner_c
        zc = L.gps_floor_under_z(L.gps_u_out - c / 3)
        for s in (1.0, -1.0):
            va, vb = (bv0, bv1) if s > 0 else (-bv1, -bv0)
            sol.append(radial_box(bu0, bu1, va, vb, zb_b, L.gps_z_rim, th, label="GPS pod corner boss"))
            hu, hv = L.gps_boss_hole_uv[0 if s > 0 else 1]
            r_h = L.gps_boss_hole_d / 2
            sol.append(radial_box(hu - r_h, hu + r_h, hv - r_h, hv + r_h, L.gps_boss_hole_z0, L.gps_z_rim, th,
                                  volume=-math.pi * r_h * r_h * L.gps_boss_hole_depth, label="GPS lid-screw hole"))
            sol.append(radial_box(L.gps_u_out - c, L.gps_u_out, s * (L.gps_w_h - c) if s > 0 else -L.gps_w_h,
                                  L.gps_w_h if s > 0 else -(L.gps_w_h - c), zc, L.gps_z_rim, th,
                                  volume=-0.5 * c * c * (L.gps_z_rim - zc), label="GPS pod corner chamfer"))
        # raceway (above the pocket, up to the floor underside) and the connector pocket: walls outside the tube skin
        (su0, su1), shv = L.gps_rw_solid_u, L.gps_rw_solid_hv
        (vu0, vu1), vhv = L.gps_rw_void_u, L.gps_rw_void_hv
        z_rw0, z_rw1 = L.gps_pk_solid_z[1], L.gps_floor_under_z(su1)
        a_rw = (su1 - R) * 2 * shv - (vu1 - vu0) * 2 * vhv
        sol.append(radial_box(R, su1, -shv, shv, z_rw0, z_rw1, th, volume=a_rw * (z_rw1 - z_rw0), label="GPS raceway"))
        (pu0, pu1), phv = L.gps_pk_solid_u, L.gps_pk_solid_hv
        (ku0, ku1), khv = L.gps_pk_void_u, L.gps_pk_void_hv
        a_pk = (pu1 - R) * 2 * phv
        v_pk = a_pk * (L.gps_pk_solid_z[1] - L.gps_pk_solid_z[0]) - (ku1 - ku0) * 2 * khv * (L.gps_pk_z[1] - L.gps_pk_z[0])
        sol.append(radial_box(R, pu1, -phv, phv, *L.gps_pk_solid_z, th, volume=v_pk, label="GPS connector pocket"))
        return sol

    def gps_pod_lid(self) -> list[Solid]:
        """v0.8 printed part gps_pod_lid: the plate (strips along u: inner edge the arc r = gps_lid_r_in, outer corners
        chamfered) x lid t, the three posts under it, minus the post holes and the two lid-screw holes."""
        L = self.lay
        th, Rl, w, c, n = L.gps_theta, L.gps_lid_r_in, L.gps_w_h, L.gps_corner_c, 80
        z0, z1 = L.gps_z_rim, L.gps_z_lid_top
        u0 = math.sqrt(Rl * Rl - w * w)
        du = (L.gps_u_out - u0) / n
        sol: list[Solid] = []
        for i in range(n):
            u = u0 + (i + 0.5) * du
            half = w - (math.sqrt(Rl * Rl - u * u) if u < Rl else 0.0)     # the arc takes the middle for u < Rl
            width = 2 * half if u >= Rl else 2 * half
            if u > L.gps_u_out - c:
                width -= 2 * (u - (L.gps_u_out - c))
            sol.append(radial_box(u - du / 2, u + du / 2, -width / 2, width / 2, z0, z1, th, volume=du * width * (z1 - z0),
                                  label="GPS lid plate"))
        r_p, r_ph, r_s = L.gps_post_d / 2, L.gps_post_hole_d / 2, L.gps_lid_screw_d / 2
        for u, v in L.gps_post_uv:
            sol.append(radial_box(u - r_p, u + r_p, v - r_p, v + r_p, L.gps_post_z[0], L.gps_post_z[1], th,
                                  volume=math.pi * r_p * r_p * (L.gps_post_z[1] - L.gps_post_z[0]), label="GPS lid post"))
            sol.append(radial_box(u - r_ph, u + r_ph, v - r_ph, v + r_ph, L.gps_post_z[0], L.gps_post_hole_z1, th,
                                  volume=-math.pi * r_ph * r_ph * (L.gps_post_hole_z1 - L.gps_post_z[0]), label="GPS lid post hole"))
        for u, v in L.gps_boss_hole_uv:
            sol.append(radial_box(u - r_s, u + r_s, v - r_s, v + r_s, z0, z1, th, volume=-math.pi * r_s * r_s * (z1 - z0),
                                  label="GPS lid screw hole"))
        return sol

    def com_mount_solids(self) -> list[Solid]:
        """v0.6 SPEC s5.3: per COM-board boss the ring Ø com_boss_d minus the Ø com_hole_d tap hole over com_boss_h
        (printed at the fill of its (boss_d - hole_d)/2 wall), and the bulkhead material the blind hole removes above
        the boss top (com_hole_depth - com_boss_h, at the bulkhead's fill 1).  Empty without a COM board."""
        L = self.lay
        if not getattr(L, "has_com_board", False):
            return []
        sol = []
        ring_a = math.pi / 4 * (L.com_boss_d ** 2 - L.com_hole_d ** 2)
        hole_a = math.pi / 4 * L.com_hole_d ** 2
        h_in_bulkhead = L.com_hole_depth - L.com_boss_h
        for x, y in L.com_boss_xy:
            sol.append(box((x, y, L.z_com_boss_tip + L.com_boss_h / 2), (L.com_boss_d, L.com_boss_d, L.com_boss_h),
                           fill=self.fill_factor((L.com_boss_d - L.com_hole_d) / 2), volume=ring_a * L.com_boss_h,
                           label="COM boss"))
            if h_in_bulkhead > 0.0:
                sol.append(box((x, y, L.z_com_boss_top + h_in_bulkhead / 2), (L.com_hole_d, L.com_hole_d, h_in_bulkhead),
                               volume=-hole_a * h_in_bulkhead, label="COM blind hole (in the bulkhead)"))
        return sol

    def piston(self) -> list[Solid]:
        L = self.lay
        pl = self.p["payload_module"]
        return [tube_z(0.0, L.piston_d / 2, L.z_piston_bottom, L.z_piston_top, label="disc"),
                tube_z(0.0, pl["piston_recess_d"] / 2, L.z_piston_bottom, L.z_piston_bottom + pl["piston_recess_depth"],
                       frac=-1.0, label="recess")]

    def nose_cone(self) -> list[Solid]:
        L = self.lay
        no = self.p["nose"]
        sol = [tube_z(L.nose_skirt_id / 2, L.body_r, L.z_nose_skirt_bottom, L.z_nose_skirt_top, label="skirt"),
               radial_box(L.lug_r_in, L.lug_r_out, -L.lug_w / 2, L.lug_w / 2, L.z_lug_bottom, L.z_lug_top,
                          L.latch_lug_theta, label="latch lug")]
        wall = L.nose_wall
        h = 1e-3

        def r_in(z: float) -> float:
            x = z - L.z_nose_base
            r = L.nose_outer_radius(x)
            slope = (L.nose_outer_radius(min(L.nose_len, x + h)) - L.nose_outer_radius(max(0.0, x - h))) / (2 * h)
            return r - wall * math.sqrt(1.0 + slope * slope)   # normal-offset shell, solid near the tip

        sol += revolved_shell(lambda z: L.nose_outer_radius(z - L.z_nose_base), r_in, L.z_nose_base, L.z_nose_tip,
                              2000, None, "Haack shell")
        chord = 2.0 * (L.nose_outer_radius(L.z_anchor_bar - L.z_nose_base) - wall)
        sol.append(box((0.0, 0.0, L.z_anchor_bar + no["anchor_bar_h"] / 2),
                       (chord, no["anchor_bar_w"], no["anchor_bar_h"]), label="anchor bar"))
        sol.append(box((0.0, 0.0, L.z_anchor_bar + no["anchor_bar_h"] / 2),
                       (no["tether_eye_d"], no["tether_eye_d"], no["anchor_bar_h"]),
                       volume=-math.pi / 4 * no["tether_eye_d"]**2 * no["anchor_bar_h"], label="tether eye"))
        return sol

    def vane(self, theta: float) -> list[Solid]:
        L = self.lay
        v = self.p["vane"]
        r_sh = L.vane_shaft_d / 2
        sol = [radial_box(L.vane_r_in, L.vane_r_out, -L.vane_t / 2, L.vane_t / 2, L.vane_te_z, L.vane_le_z, theta, label="plate")]
        # full rounds on LE and TE: each removes (1/2 - pi/8) t^2 per unit span
        er = v["edge_radius"]
        sol.append(radial_box(L.vane_r_in, L.vane_r_out, -er, er, L.vane_te_z, L.vane_le_z, theta,
                              volume=-2.0 * (0.5 - math.pi / 8) * (2 * er) ** 2 * L.vane_span, label="edge rounds"))
        shaft_len = L.vane_shaft_r_out - L.vane_shaft_r_in
        sol.append(radial_box(L.vane_shaft_r_in, L.vane_shaft_r_out, -r_sh, r_sh, L.vane_hinge_z - r_sh,
                              L.vane_hinge_z + r_sh, theta, volume=math.pi * r_sh**2 * shaft_len, label="shaft"))
        # shaft volume already inside the plate (circle r_sh clipped to |v| <= t/2)
        hh = min(L.vane_t / 2, r_sh)
        seg = 2.0 * (hh * math.sqrt(max(r_sh**2 - hh**2, 0.0)) + r_sh**2 * math.asin(hh / r_sh))
        overlap_len = max(0.0, min(L.vane_shaft_r_out, L.vane_r_out) - max(L.vane_shaft_r_in, L.vane_r_in))
        u0, u1 = max(L.vane_shaft_r_in, L.vane_r_in), min(L.vane_shaft_r_out, L.vane_r_out)
        sol.append(radial_box(u0, u1, -hh, hh, L.vane_hinge_z - hh, L.vane_hinge_z + hh, theta,
                              volume=-seg * overlap_len, label="shaft/plate overlap"))
        # D-flat on the -Z side for r >= d_flat_r_in
        d = v["d_flat_depth"]
        seg_d = r_sh**2 * math.acos((r_sh - d) / r_sh) - (r_sh - d) * math.sqrt(2 * r_sh * d - d * d)
        fr = getattr(L, "vane_d_flat_r_in", v["d_flat_r_in"])       # v0.9: Layout derives it in direct drive
        sol.append(radial_box(fr, L.vane_shaft_r_out, -r_sh, r_sh, L.vane_hinge_z - r_sh,
                              L.vane_hinge_z - r_sh + d, theta, volume=-seg_d * (L.vane_shaft_r_out - fr),
                              label="D-flat"))
        return sol

    def vane_horn(self, theta: float) -> list[Solid]:
        L = self.lay
        v = self.p["vane"]
        t = v["horn_t"]
        sol = [radial_box(L.horn_r_in, L.horn_r_out, -v["horn_w"] / 2, v["horn_w"] / 2, L.horn_tail_z, L.horn_tip_z,
                          theta, label="plate"),
               radial_box(L.horn_r_in, L.horn_r_out, -L.hinge_bore_d / 2, L.hinge_bore_d / 2,
                          L.vane_hinge_z - L.hinge_bore_d / 2, L.vane_hinge_z + L.hinge_bore_d / 2, theta,
                          volume=-math.pi / 4 * L.hinge_bore_d**2 * t, label="D-bore"),
               radial_box(L.horn_r_in, L.horn_r_out, -v["horn_pin_hole_d"] / 2, v["horn_pin_hole_d"] / 2,
                          L.horn_tip_z - HORN_PIN_HOLE_BELOW_TIP_MM - v["horn_pin_hole_d"] / 2,
                          L.horn_tip_z - HORN_PIN_HOLE_BELOW_TIP_MM + v["horn_pin_hole_d"] / 2, theta,
                          volume=-math.pi / 4 * v["horn_pin_hole_d"]**2 * t, label="pin hole")]
        return sol

    def latch_arm(self) -> list[Solid]:
        L = self.lay
        pl = self.p["payload_module"]
        ax, ay = pod_frame(L.release_servo_axis_r, L.servo_shaft_tangential_offset, L.release_pod_theta)
        lx, ly = pod_frame((L.lug_r_in + L.lug_r_out) / 2, 0.0, L.latch_lug_theta)
        d = math.hypot(lx - ax, ly - ay)
        ux, uy = (lx - ax) / d, (ly - ay) / d
        zc = (L.z_latch_arm_bottom + L.z_latch_arm_top) / 2
        half = L.latch_arm_len / 2
        hub_d = L.latch_arm_w   # SPEC s5.3: "O6 hub" = arm width (6 mm) in the default preset
        return [box((ax + ux * half, ay + uy * half, zc), (L.latch_arm_len, L.latch_arm_w, pl["latch_arm_t"]), label="arm"),
                box((ax, ay, zc), (hub_d, hub_d, pl["latch_arm_t"]),
                    volume=0.5 * math.pi / 4 * hub_d**2 * pl["latch_arm_t"], label="hub (half outside arm)")]

    def launch_pad(self) -> list[Solid]:
        L = self.lay
        lp = self.p["launch_pad"]
        t = L.pad_plate_t
        sol = [tube_z(L.pad_plate_id / 2, L.pad_plate_od / 2, -t, 0.0, fill=self.fill_factor(t), label="plate")]
        leg = lp["leg_size"]
        for th in L.pad_leg_thetas:
            x, y = pod_frame(lp["leg_r"], 0.0, th)
            sol.append(box((x, y, -t - L.pad_leg_h / 2), (leg, leg, L.pad_leg_h), fill=self.fill_factor(leg), label="leg"))
            sol.append(box((x, y, -t - L.pad_leg_h - lp["foot_t"] / 2), (lp["foot_size"], lp["foot_size"], lp["foot_t"]),
                           fill=self.fill_factor(lp["foot_t"]), label="foot"))
        rib_w, rib_h, rib_len = lp["fin_locator_rib_w"], lp["fin_locator_rib_h"], lp["fin_locator_rib_len"]
        for th in L.fin_thetas:
            for sgn in (-1, 1):
                v = sgn * (lp["fin_locator_gap"] / 2 + rib_w / 2)
                sol.append(radial_box(lp["fin_locator_rib_r_in"], lp["fin_locator_rib_r_in"] + rib_len,
                                      v - rib_w / 2, v + rib_w / 2, 0.0, rib_h, th, label="fin locator rib"))
        return sol

    # -- all parts ---------------------------------------------------------
    def estimate_all(self) -> dict[str, PartEstimate]:
        """Every printed instance (multi-instance parts are suffixed _1.._4)."""
        L = self.lay
        parts: dict[str, PartEstimate] = {}
        single = {
            "propulsion_module": self.propulsion_module,
            "intake_module": self.intake_module,
            "avionics_module": self.avionics_module,
            "avionics_tray": self.avionics_tray,
            "payload_module": self.payload_module,
            "piston": self.piston,
            "nose_cone": self.nose_cone,
            "latch_arm": self.latch_arm,
        }
        if getattr(L, "has_gps_pod", False):   # v0.8: the GPS pod lid (the pod itself is part of payload_module)
            single["gps_pod_lid"] = self.gps_pod_lid
        if getattr(L, "has_livery", False):    # livery 2026-11-04: the orange inlay of the avionics module
            single[L.livery_inlay_part] = self.livery_inlay
        for name, fn in single.items():
            parts[name] = aggregate(name, "printed", fn(), rho_g_cm3=self.rho, notes=["ESTIMATE (analytic)"])
        for i, th in enumerate(L.vane_thetas, start=1):
            parts[f"vane_{i}"] = aggregate(f"vane_{i}", "printed", self.vane(th), rho_g_cm3=self.rho, notes=["ESTIMATE (analytic)"])
            if getattr(L, "direct_drive", False):     # v0.9: couplers and servo covers instead of horns
                parts[f"vane_coupler_{i}"] = aggregate(f"vane_coupler_{i}", "printed", self.vane_coupler(th), rho_g_cm3=self.rho,
                                                       notes=["ESTIMATE (analytic)"])
                parts[f"servo_cover_{i}"] = aggregate(f"servo_cover_{i}", "printed", self.servo_cover(th), rho_g_cm3=self.rho,
                                                      notes=["ESTIMATE (analytic)"])
            else:
                parts[f"vane_horn_{i}"] = aggregate(f"vane_horn_{i}", "printed", self.vane_horn(th), rho_g_cm3=self.rho,
                                                    notes=["ESTIMATE (analytic)"])
        if L.tabs and getattr(L, "wire_ch_open", False):   # v0.9: the motor-lead groove cover
            parts["wire_cover"] = aggregate("wire_cover", "printed", self.wire_cover(), rho_g_cm3=self.rho,
                                            notes=["ESTIMATE (analytic)"])
        return parts

    def launch_pad_estimate(self) -> PartEstimate:
        return aggregate("launch_pad", "printed", self.launch_pad(), rho_g_cm3=self.rho,
                         notes=["ESTIMATE (analytic); ground equipment, not flight mass"])


# --------------------------------------------------------------------------- #
# placeholders (SPEC s6 table) - masses from params, geometry from Layout
# --------------------------------------------------------------------------- #
def pushrod_endpoints(lay) -> tuple[Vec3, Vec3, float]:
    """Arm tip -> horn tip for vane 1 (SPEC s4.5); all four are congruent.  v0.9 direct drive: no pushrod (length 0,
    both ends at the hinge point of vane 1)."""
    if getattr(lay, "direct_drive", False):
        h = (*pod_frame(lay.hinge_boss_r_out, 0.0, lay.vane_thetas[0]), lay.vane_hinge_z)
        return h, h, 0.0
    th_pod, th_vane = lay.pod_thetas[0], lay.vane_thetas[0]
    ax, ay = pod_frame(lay.servo_arm_tip_r, lay.servo_shaft_tangential_offset, th_pod)
    hx, hy = pod_frame((lay.horn_r_in + lay.horn_r_out) / 2, 0.0, th_vane)
    za = (lay.servo_arm_z_bottom + lay.servo_arm_z_top) / 2
    zh = lay.horn_tip_z - HORN_PIN_HOLE_BELOW_TIP_MM
    a, b = (ax, ay, za), (hx, hy, zh)
    return a, b, math.dist(a, b)


def edf_unit_solids(L, e: dict) -> list[Solid]:
    """SPEC s6 edf_unit: lip disc + housing ring + hub; v0.5 tabs: the lip is a ring (bell envelope, ID
    housing_id), plus the motor cylinder below the housing and the two tab plates minus their three slots."""
    hub_len = min(EDF_HUB_LEN_MM, L.edf_housing_len)
    lip_ri = e["housing_id"] / 2 if L.tabs else 0.0
    sol = [tube_z(lip_ri, e["lip_od"] / 2, L.z_edf_lip_bottom, L.z_edf_lip_top, label="lip"),
           tube_z(e["housing_id"] / 2, e["housing_od"] / 2, L.z_edf_housing_bottom, L.z_edf_lip_bottom, label="housing"),
           tube_z(0.0, e["hub_d_ASSUMED"] / 2, L.z_edf_housing_bottom, L.z_edf_housing_bottom + hub_len, label="hub")]
    if L.tabs:
        sol.append(tube_z(0.0, e["motor_d"] / 2, L.z_edf_housing_bottom - e["motor_protrusion_rear"],
                          L.z_edf_housing_bottom, label="motor"))
        t, sl, sw = L.tab_t, L.tab_slot_len_radial, L.tab_slot_w
        for th in L.tab_thetas:
            sol.append(radial_box(L.tab_r_in, L.tab_r_out, -t / 2, t / 2, L.tab_z0, L.tab_z1, th, label="tab"))
            for z in L.tab_slot_zs:   # slot: tab_slot_len_radial along r x tab_slot_w along Z, through the plate
                sol.append(radial_box(L.tab_slot_r - sl / 2, L.tab_slot_r + sl / 2, -t / 2, t / 2, z - sw / 2, z + sw / 2,
                                      th, volume=-sl * sw * t, label="tab slot"))
    return sol


def placeholder_parts(lay, p: dict) -> dict[str, PartEstimate]:
    """SPEC s6 placeholders and lumped masses (every mass is a params value)."""
    L = lay
    e, s, pod = p["edf"], p["servo"], p["pod"]
    hw = p["hardware_mass_g"]
    parts: dict[str, PartEstimate] = {}

    parts["edf_unit"] = aggregate("edf_unit", "placeholder", edf_unit_solids(L, e), mass_g=e["mass_g"],
                                  notes=["mass = edf.mass_g (vendor)"])

    def box_part(name: str, xr: tuple, yr: tuple, zr: tuple, mass: float, note: str) -> None:
        c = ((xr[0] + xr[1]) / 2, (yr[0] + yr[1]) / 2, (zr[0] + zr[1]) / 2)
        d = (xr[1] - xr[0], yr[1] - yr[0], zr[1] - zr[0])
        parts[name] = aggregate(name, "placeholder", [box(c, d)], mass_g=mass, notes=[note])

    box_part("battery", L.battery_x, L.battery_y, L.battery_z, p["battery"]["mass_g"], "mass = battery.mass_g")
    box_part("esc", L.esc_x, L.esc_y, L.esc_z, p["electronics"]["esc"]["mass_g"], "mass = electronics.esc.mass_g")
    box_part("nav", L.nav_x, L.nav_y, L.nav_z, p["electronics"]["nav"]["mass_g"], "mass = electronics.nav.mass_g")
    box_part("misc_electronics", L.misc_x, L.misc_y, L.misc_z,
             p["electronics"]["misc"]["mass_g"], "mass = electronics.misc.mass_g")
    if getattr(L, "has_com_board", False):   # v0.6: COM board (2nd ATmega128A) under the payload bulkhead
        box_part("com", L.com_board_x, L.com_board_y, L.com_board_z, p["electronics"]["com"]["mass_g"],
                 "mass = electronics.com.mass_g (ASSUMED in params; weigh it)")
    if getattr(L, "has_radio_log", False):   # v0.6.2: LoRa / OpenLog / level shifter / LDO box under the payload bulkhead
        box_part("radio_log", L.radio_log_x, L.radio_log_y, L.radio_log_z, p["electronics"]["radio_log"]["mass_g"],
                 "mass = electronics.radio_log.mass_g (ASSUMED in params; weigh it)")
    if getattr(L, "has_fcu", False):         # v0.6.3: FCU board flat on the intake bulkhead (foam tape)
        box_part("fcu", L.fcu_x, L.fcu_y, L.fcu_z, p["electronics"]["fcu"]["mass_g"],
                 "mass = electronics.fcu.mass_g (ASSUMED in params; weigh it)")
    if getattr(L, "has_gns", False):         # v0.8.1: GNS board (4th ATmega128A) on its bosses on the -x tube wall
        box_part("gns", L.gns_x, L.gns_y, L.gns_z, L.gns_mass_g,
                 "mass = electronics.gns.mass_g_ASSUMED (board + shifter + leads) + avionics_module.gns_mount.screws_g_ASSUMED "
                 "(Layout.gns_mass_g; weigh it)")
    if getattr(L, "has_gps_pod", False):     # v0.8: GPS board + patch, ground plane (pod frame boxes), lead lump
        th = L.gps_theta
        (pu, pv, pz), (bu, bv, bz) = L.gps_patch_box, L.gps_board_box
        parts["gps"] = aggregate("gps", "placeholder", [radial_box(*pu, *pv, *pz, th), radial_box(*bu, *bv, *bz, th)],
                                 mass_g=L.gps_board_mass_g,
                                 notes=["mass = electronics.gps.board_mass_g (vendor 17 g with the antenna; weigh it), over "
                                        "the patch and board boxes by volume"])
        (gu, gv, gz), n_ = L.gps_gp_box, L.gps_gp_notch
        plate = [radial_box(*gu, *gv, *gz, th)]
        for va, vb in ((gv[1] - n_, gv[1]), (gv[0], gv[0] + n_)):
            plate.append(radial_box(gu[1] - n_, gu[1], va, vb, *gz, th, volume=-n_ * n_ * (gz[1] - gz[0])))
        parts["gps_ground_plane"] = aggregate("gps_ground_plane", "placeholder", plate, mass_g=L.gps_gp_mass_g,
                                              notes=["mass = payload_module.gps_pod.ground_plane.mass_g_ASSUMED"])
        parts["gps_lead"] = point_mass("gps_lead", L.gps_lead_mass_g, tuple(L.gps_lead_com),
                                       ["v0.8: GPS lead along Layout.gps_lead_polyline + slack, connector pair, screws / "
                                        "standoffs / tape (payload_module.gps_pod.lead, ASSUMED) at their mass centroid"])

    def servo_solid(theta: float, axis_r: float, z_bottom: float) -> Solid:
        return radial_box(axis_r - s["body_w"] / 2, axis_r + s["body_w"] / 2, -s["body_l"] / 2, s["body_l"] / 2,
                          z_bottom, z_bottom + s["case_top_from_bottom"], theta)

    if getattr(L, "direct_drive", False):      # v0.9: servo case in the fin-root fairing, horn disc on its spline
        for i, th in enumerate(L.fin_thetas, start=1):
            w = s["body_w"] / 2
            parts[f"servo_vane_{i}"] = aggregate(f"servo_vane_{i}", "placeholder",
                                                 [radial_box(L.dd_u_case_top, L.dd_u_case_bot, -w, w, L.dd_z_case[0],
                                                             L.dd_z_case[1], th)], mass_g=s["mass_g"], notes=["mass = servo.mass_g"])
            r = L.dd_horn_d / 2
            parts[f"servo_horn_{i}"] = aggregate(f"servo_horn_{i}", "placeholder",
                                                 [radial_box(getattr(L, "dd_u_arm0", L.dd_u_flange1), L.dd_u_horn1, -r, r,
                                                             L.vane_hinge_z - r,
                                                             L.vane_hinge_z + r, th)],
                                                 mass_g=p["vane_drive"]["horn"]["mass_g"], notes=["vane_drive.horn.mass_g"])
        if getattr(L, "dd_bushing", False):    # v0.9 fix round: brass hinge bushings (Layout.dd_bush_*)
            for i, th in enumerate(L.vane_thetas, start=1):
                rb = L.dd_bush_od / 2
                parts[f"hinge_bushing_{i}"] = aggregate(f"hinge_bushing_{i}", "placeholder",
                                                        [radial_box(L.dd_bush_u[0], L.dd_bush_u[1], -rb, rb,
                                                                    L.vane_hinge_z - rb, L.vane_hinge_z + rb, th)],
                                                        mass_g=L.dd_bush_mass_g, notes=["brass tube, Layout.dd_bush_mass_g"])
    else:
        for i, th in enumerate(L.pod_thetas, start=1):
            parts[f"servo_vane_{i}"] = aggregate(f"servo_vane_{i}", "placeholder", [servo_solid(th, L.servo_axis_r, L.servo_bottom_z)],
                                                 mass_g=s["mass_g"], notes=["mass = servo.mass_g"])
            arm = radial_box(L.servo_axis_r, L.servo_arm_tip_r, L.servo_shaft_tangential_offset - pod["arm_w"] / 2,
                             L.servo_shaft_tangential_offset + pod["arm_w"] / 2, L.servo_arm_z_bottom, L.servo_arm_z_top, th)
            parts[f"servo_arm_{i}"] = aggregate(f"servo_arm_{i}", "placeholder", [arm], mass_g=SMALL_PLACEHOLDER_MASS_G,
                                                notes=["SPEC s6: 1 g, negligible"])
        a, b, rod_len = pushrod_endpoints(L)
        for i in range(1, L.vane_count + 1):
            th_pod, th_vane = L.pod_thetas[i - 1], L.vane_thetas[i - 1]
            ax, ay = pod_frame(L.servo_arm_tip_r, L.servo_shaft_tangential_offset, th_pod)
            hx, hy = pod_frame((L.horn_r_in + L.horn_r_out) / 2, 0.0, th_vane)
            c = ((ax + hx) / 2, (ay + hy) / 2, (a[2] + b[2]) / 2)
            rod = Solid(math.pi / 4 * pod["pushrod_d"] ** 2 * rod_len, c, (rod_len**2 / 12, rod_len**2 / 12, 0.0))
            parts[f"pushrod_{i}"] = aggregate(f"pushrod_{i}", "placeholder", [rod], mass_g=SMALL_PLACEHOLDER_MASS_G,
                                              notes=[f"SPEC s6: 1 g; length {rod_len:.1f} mm"])
    parts["servo_release"] = aggregate("servo_release", "placeholder",
                                       [servo_solid(L.release_pod_theta, L.release_servo_axis_r, L.release_servo_bottom_z)],
                                       mass_g=s["mass_g"], notes=["mass = servo.mass_g"])

    z_chute_block_top = L.z_can_top + p["payload"]["chute_pack_h"]   # SPEC s5.3 (Layout overwrites z_chute_pack_top)
    parts["cansat_payload"] = aggregate("cansat_payload", "placeholder",
                                        [tube_z(0.0, L.can_d / 2, L.z_can_bottom, max(z_chute_block_top, L.z_can_top))],
                                        mass_g=p["payload"]["mass_g"], notes=["mass = payload.mass_g (competition max)"])
    pl = p["payload_module"]
    parts["spring"] = aggregate("spring", "placeholder",
                                [tube_z(pl["spring_od"] / 2 - pl["spring_wire_d"], pl["spring_od"] / 2, L.z_spring_bottom, L.z_spring_top)],
                                mass_g=SPRING_FRACTION_OF_HARDWARE * hw["spring_piston_hardware"],
                                notes=["mass = 0.5 x hardware_mass_g.spring_piston_hardware"])
    rec = p["recovery"]
    parts["chute_pack"] = aggregate("chute_pack", "placeholder",
                                    [tube_z(0.0, L.chute_pack_d / 2, L.z_chute_pack_bottom, L.z_chute_pack_top)],
                                    mass_g=rec["chute_mass_g"] + rec["lines_swivel_mass_g"] + rec["shock_cord_mass_g"],
                                    notes=["chute + lines/swivel + shock cord"])
    joint_zs = [L.z_prop_spigot_bottom + L.joint_screw_dz, L.z_intake_spigot_bottom + L.joint_screw_dz,
                L.z_avionics_spigot_bottom + L.joint_screw_dz, L.z_payload_spigot_bottom + L.joint_screw_dz]
    parts["joint_screws_inserts"] = point_mass("joint_screws_inserts", hw["joint_screws_inserts"],
                                               (0.0, 0.0, sum(joint_zs) / len(joint_zs)),
                                               ["SPEC s6: point mass at the mean of the four joint Z"])
    if getattr(L, "direct_drive", False):      # v0.9: cover / coupler / horn screws in the fairings
        parts["vane_drive_hardware"] = point_mass("vane_drive_hardware", p["vane_drive"]["hardware_g_ASSUMED"],
                                                  (0.0, 0.0, 0.5 * (L.fair_z[0] + L.fair_z[1])),
                                                  ["vane_drive.hardware_g_ASSUMED at the fairings' mid height"])
    else:
        parts["pushrods_horns_clevises"] = point_mass("pushrods_horns_clevises", hw["pushrods_horns_clevises"],
                                                      (0.0, 0.0, L.vane_hinge_z), ["SPEC s6: point at vane_hinge_z"])
    if getattr(L, "has_motor_leads", False):   # v0.9: 3 x 12 AWG motor leads + bullet pairs along their route
        parts["motor_leads"] = point_mass("motor_leads", L.ml_mass_g, tuple(L.ml_com),
                                          ["v0.9: motor_leads.* (ASSUMED g/m, bullets) along Layout.motor_lead_polyline"])
    if getattr(L, "servo_leads", False):     # SPEC s3.8 / s6 (v0.6): vane-servo extension leads, tab-mount preset only
        parts["servo_leads"] = point_mass("servo_leads", L.sl_lead_mass_g, tuple(L.sl_lead_com),
                                          ["SPEC s3.8: extension wire beyond the splice + slack + connectors at the FCU "
                                           "(servo_leads.mass_g_per_m_ASSUMED) at the route centroid Layout.sl_lead_com"])
    parts["spring_piston_hardware_balance"] = point_mass(
        "spring_piston_hardware_balance", (1.0 - SPRING_FRACTION_OF_HARDWARE) * hw["spring_piston_hardware"],
        (0.0, 0.0, (L.z_piston_bottom + L.z_piston_top) / 2),
        ["NOT in the SPEC s6 table: the other half of hardware_mass_g.spring_piston_hardware, lumped at the piston"])
    return parts


# budget-only items: in the sizing budget, never in a CAD report (not in the SPEC s6 table)
BUDGET_ONLY_ITEMS = ("spring_piston_hardware_balance",)
# ground equipment: a CAD report may carry it (both generators do), the flight budget never uses it
OPTIONAL_REPORT_PARTS = ("launch_pad",)


def expected_report_parts(lay, p: dict) -> list[str]:
    """Component names a complete SPEC s6 CAD report must carry: every printed flight part of
    ``StructureEstimator.estimate_all`` (multi-instance parts as name_1..name_n) and every SPEC s6 placeholder /
    lumped item of ``placeholder_parts``.  ``launch_pad`` is optional; the budget-only hardware balance is not
    expected.  A report missing any of these silently dropped a part and is no evidence (sizing.find_cad_report)."""
    printed = []
    for name in lay.printed_parts:
        if name in OPTIONAL_REPORT_PARTS:
            continue
        if name in ("vane", "vane_horn", "vane_coupler", "servo_cover"):
            printed += [f"{name}_{i}" for i in range(1, len(lay.vane_thetas) + 1)]
        else:
            printed.append(name)
    return printed + [n for n in placeholder_parts(lay, p) if n not in BUDGET_ONLY_ITEMS]
