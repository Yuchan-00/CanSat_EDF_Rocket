"""Livery check C36 (user decisions 2026-11-04: white body + orange YUCHAN inlay on the avionics_module).

Invariant enforced here: every PASS of the logo is backed by booleans / BRep distances on the solids or by shapely
measurements on the PLACED polygons (Layout.livery_polys_vz mapped back to the outer skin) - nothing is taken from the
design notes.  The rows (all must hold for PASS; any miss is FAIL):

(a) zone / clearance - the logo footprint grown by ``livery.check.clearance_min_mm`` (3 mm, user rule) in the s_z frame,
    built as a thin shell just under the outer skin (r [body_r - depth, body_r - 0.01]), lies entirely in the PLAIN module
    (the module without the pocket): every hole, window, vent slot, joint-screw hole, the part's lower end and the end of
    the visible skin (spigot step) is >= 3 mm from every orange point, by one boolean per copy that sees every feature
    of the solid (also ones nobody listed).  Reported beside it: the BRep distance from the inlay to each known feature
    (windows, vent slots, joint holes, GPS entry hole, both part ends) and from the logo to each joint-screw axis.
    reviewfix F6 (2026-11-04, independent review): the containment boolean has a volume tolerance (LIVERY_VOL_TOL_MM3)
    and let an encroachment of up to ~0.15 mm pass (0.037 mm^3 < 0.05); so the clearance verdict also needs, per copy,
    the BRep distance from that copy's inlay to every VOID of the outer skin band (the copy's sector of the annulus
    r [r_in, body_r], logo half-width + clearance + 2 mm each side, over the module and 6 mm past both ends, minus the
    plain module): every window, slot, hole, part end and the spigot step near the logo, listed or not, with NO volume
    tolerance - it must be >= the clearance rule.
(e) per-layer information (reviewfix F7, not a verdict): the orange chords and the white gaps between them that the
    slicer sees at every layer mid-plane (printer.layer_h; the inlay is a prism along each copy's radial direction, so
    its chords in v = r sin(s / r) are the same at every radius) - counts below one line width (single-layer specks).
(b) strokes / gaps / islands on the placed polygons (mapped back to s = r asin(v / r)): min orange stroke and min white gap
    = the largest disc whose opening / closing removes only corner slivers (inscribed radius <= k d / 2, k =
    livery.check.corner_sliver_k, the judge-print criterion) and keeps every island; smallest island area.
(c) depth - livery.inlay_depth_mm <= printer.line_w (one outer wall line) and body.wall - depth >= livery.white_behind_min_mm
    (the 2 white lines behind it, AMS probe); measured on the solid: inlay volume / outer area.
(d) union - V(white body) + V(inlay) = V(plain module), V(white x inlay) = 0, V(plain x inlay) = V(inlay), and the inlay's
    outer area = the polygons' area (the solid IS the drawn logo), within LIVERY_VOL_TOL_MM3 / LIVERY_AREA_REL_TOL.
"""
from __future__ import annotations

import math

import cadquery as cq
import shapely
from shapely.geometry import Polygon
from shapely.ops import unary_union

import massprops as mp
import rocket_geometry as rg
from layout import Layout

LIVERY_VOL_TOL_MM3 = 0.05        # (d) union / containment volumes (OCC boolean precision on ~7000 small faces)
LIVERY_AREA_REL_TOL = 0.005      # (d) inlay outer area vs polygon area
GROW_QUAD_SEGS = 8               # (a) round joins of the grown footprint
SKIN_GAP = 0.01                  # (a) the grown shell stops 0.01 mm under the outer skin (no coincident faces)


def placed_polygons_sz(L: Layout) -> list[Polygon]:
    """The PLACED islands back in the s_z frame: v = r sin(s / r) inverted (s = r asin(v / r)), z - livery_z0."""
    R = L.livery_r
    out = []
    for outer, holes in L.livery_polys_vz:
        def back(ring):
            return [(R * math.asin(v / R), z - L.livery_z0) for v, z in ring]
        out.append(Polygon(back(outer), [back(h) for h in holes]))
    return out


def _polys(g) -> list[Polygon]:
    return [q for q in (g.geoms if hasattr(g, "geoms") else [g]) if isinstance(q, Polygon) and not q.is_empty]


def _only_corner_slivers(region, d: float, k: float) -> bool:
    r = d / 2
    op = region.buffer(-r, quad_segs=32).buffer(r, quad_segs=32)
    if len(_polys(op)) < len(_polys(region)):
        return False
    for q in _polys(region.difference(op)):
        if q.area < 1e-6:
            continue
        if shapely.maximum_inscribed_circle(q, 0.002).length > k * d / 2:
            return False
    return True


def _bisect(fn, lo: float = 0.4, hi: float = 8.0) -> float:
    if not fn(lo):
        return lo
    while hi - lo > 0.005:
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if fn(mid) else (lo, mid)
    return lo


def polygon_metrics(L: Layout) -> dict:
    """(b) on the placed polygons: strokes, white gaps, islands, area, extents."""
    k = float(L.livery_check.get("corner_sliver_k", 0.25))
    ps = placed_polygons_sz(L)
    g = unary_union(ps)
    x0, y0, x1, y1 = g.bounds
    frame = shapely.box(x0 - 2, y0 - 2, x1 + 2, y1 + 2).buffer(4.0, quad_segs=32)
    white = frame.difference(g)
    pair = min(ps[i].distance(ps[j]) for i in range(len(ps)) for j in range(i + 1, len(ps)))
    return {"min_stroke_mm": round(_bisect(lambda d: _only_corner_slivers(g, d, k)), 3),
            "min_white_gap_mm": round(_bisect(lambda d: _only_corner_slivers(white, d, k)), 3),
            "min_island_gap_mm": round(pair, 3), "islands": len(ps),
            "smallest_island_mm2": round(min(q.area for q in ps), 3),
            "counters": sum(len(q.interiors) for q in ps),
            "smallest_counter_mm2": round(min((Polygon(i).area for q in ps for i in q.interiors), default=math.nan), 3),
            "area_mm2_per_copy": round(g.area, 3), "s_mm": [round(x0, 3), round(x1, 3)],
            "print_z_mm": [round(y0, 3), round(y1, 3)], "corner_sliver_k": k}


def _shell_from_sz(L: Layout, polys_sz: list[Polygon], theta: float, r0: float, r1: float) -> cq.Shape:
    """Thin radial shell r [r0, r1] under the given s_z polygons at copy azimuth theta (the inlay construction:
    pre-warped (v, z) prism INTERSECT an annular sector)."""
    R = L.livery_r
    s_lo = min(q.bounds[0] for q in polys_sz)
    s_hi = max(q.bounds[2] for q in polys_sz)
    z_lo = min(q.bounds[1] for q in polys_sz) + L.livery_z0
    z_hi = max(q.bounds[3] for q in polys_sz) + L.livery_z0
    vmax = max(abs(R * math.sin(s / R)) for s in (s_lo, s_hi))
    u0, u1 = math.sqrt(r0 ** 2 - vmax ** 2) - 1.0, R + 2.0
    band = rg._sector(r0, r1, theta + math.degrees(s_lo / R) - 1.0, theta + math.degrees(s_hi / R) + 1.0,
                      z_lo - 1.0, z_hi + 1.0)
    out = []
    for q in polys_sz:
        def warp(ring):
            return [(R * math.sin(s / R), z + L.livery_z0) for s, z in list(ring.coords)[:-1]]
        face = rg._face_vz(warp(q.exterior), [warp(h) for h in q.interiors], u0)
        prism = cq.Solid.extrudeLinear(face, cq.Vector(u1 - u0, 0, 0))
        out.append(rg._rot_z(prism, theta).intersect(band))
    return out[0] if len(out) == 1 else rg._fuse(out, clean=False)


def known_features(L: Layout) -> dict[str, list[cq.Shape]]:
    """The avionics module's wall features near the logo, built as rocket_geometry.avionics_module cuts them (for the
    reported distances; the clearance PASS is the containment boolean, which sees every feature)."""
    am = L.p["body"]
    feats: dict[str, list[cq.Shape]] = {}
    feats["joint screw hole (lower skirt)"] = [rg._radial_cyl(am["joint_hole_outer_d"], L.body_id / 2 - 2.0, L.body_r + 2.0, th,
                                                              L.z_avionics_bottom + L.joint_screw_dz) for th in L.joint_thetas]
    feats["vent slot"] = [rg._box_polar(L.body_id / 2 - 0.5, L.body_r + 2.0, -L.vent_slot_w / 2, L.vent_slot_w / 2,
                                        L.z_vent_slot_bottom, L.z_vent_slot_bottom + L.vent_slot_h, th)
                          for th in L.vent_slot_thetas]
    if getattr(L, "has_av_windows", False):
        feats["lightening window"] = rg.avionics_window_cutters(L)
    if getattr(L, "has_gps_pod", False):
        feats["GPS entry hole"] = [rg.gps_entry_cutter(L)]
    return feats


VOID_BAND_END_MM = 6.0           # (a, F6) the skin band runs this far past both ends of the module (the end voids)
VOID_BAND_IN_EPS = 0.001         # (a, F6) the band's inner face 1 um under r_in (no face coincident with the pocket floor)
VOID_BAND_SIDE_MARGIN_MM = 2.0   # (a, F6) the copy's band sector reaches this far past (logo half-width + clearance)


def _copy_islands(L: Layout, inlay: cq.Shape) -> dict[float, cq.Shape]:
    """The inlay's islands grouped by copy (nearest copy azimuth of each solid's centre)."""
    groups: dict[float, list] = {th: [] for th in L.livery_thetas}
    for s in inlay.Solids():
        c = s.Center()
        az = math.degrees(math.atan2(c.y, c.x)) % 360.0
        th = min(L.livery_thetas, key=lambda t: abs(((az - t + 180.0) % 360.0) - 180.0))
        groups[th].append(s)
    return {th: cq.Compound.makeCompound(v) for th, v in groups.items() if v}


def void_band_distances(L: Layout, inlay: cq.Shape, plain: cq.Shape) -> dict:
    """(a, F6) per copy: the BRep distance from the copy's inlay to the nearest void of the outer skin band (every
    window / slot / hole / part end, listed or not; no volume tolerance), and where that void is."""
    R = L.livery_r
    clear = float(L.livery_check.get("clearance_min_mm", 3.0))
    half_v = max(abs(v) for o, _ in L.livery_polys_vz for v, _z in o)
    w_deg = math.degrees((half_v + clear + VOID_BAND_SIDE_MARGIN_MM) / R)
    out: dict = {"sector_half_width_deg": w_deg}
    for th, cop in _copy_islands(L, inlay).items():
        # the copy's sector of the skin band (logo half-width + clearance + margin each side: any void within the
        # clearance of an orange point lies in it); its side faces are inside the wall material, so every void is a
        # feature of the module or one of its ends
        band = rg._sector(L.livery_r_in - VOID_BAND_IN_EPS, R, th - w_deg, th + w_deg,
                          L.z_avionics_bottom - VOID_BAND_END_MM, L.z_avionics_spigot_bottom + VOID_BAND_END_MM)
        voids = [v for v in band.cut(plain).Solids() if v.Volume() > 1e-6]
        out[f"n_voids_{th:g}"] = len(voids)
        best = None
        for v in voids:
            d = float(cop.distance(v))
            if best is None or d < best[0]:
                bb = v.BoundingBox()
                best = (d, [round(bb.zmin, 2), round(bb.zmax, 2)],
                        round(math.degrees(math.atan2(bb.center.y, bb.center.x)) % 360.0, 1), round(v.Volume(), 2))
        out[f"{th:g}"] = ({"min_mm": best[0], "void_z_mm": best[1], "void_azimuth_deg": best[2], "void_mm3": best[3]}
                          if best else {"min_mm": math.inf})
    return out


def layer_chords(L: Layout) -> dict:
    """(e, F7) information: the orange chords and the white gaps between them at every layer mid-plane (print z from
    the part's lower end, printer.layer_h), in v = r sin(s / r) (the same at every radius: radial prism)."""
    from shapely.geometry import LineString
    lh = float(L.p["printer"].get("layer_h", 0.2))
    lw = float(L.p["printer"]["line_w"])
    polys = [Polygon(o, h) for o, h in L.livery_polys_vz]
    g = unary_union(polys)
    x0, z0, x1, z1 = g.bounds
    n_lay, chords_lt, gaps_lt = 0, 0, 0
    min_chord, min_gap = (math.inf, None), (math.inf, None)
    k = 0
    while True:
        zp = (k + 0.5) * lh                   # print z of layer k's mid-plane
        zv = L.z_avionics_bottom + zp
        k += 1
        if zv > z1:
            break
        if zv < z0:
            continue
        sec = g.intersection(LineString([(x0 - 1.0, zv), (x1 + 1.0, zv)]))
        segs = sorted(([min(c[0] for c in s.coords), max(c[0] for c in s.coords)]
                       for s in (sec.geoms if hasattr(sec, "geoms") else [sec]) if s.length > 0), key=lambda a: a[0])
        if not segs:
            continue
        n_lay += 1
        for a, b in segs:
            if b - a < lw:
                chords_lt += 1
            if b - a < min_chord[0]:
                min_chord = (b - a, round(zp, 2))
        for (a0, b0), (a1, b1) in zip(segs, segs[1:]):
            gp = a1 - b0
            if gp < lw:
                gaps_lt += 1
            if gp < min_gap[0]:
                min_gap = (gp, round(zp, 2))
    return {"layer_h_mm": lh, "line_w_mm": lw, "layers_with_orange": n_lay,
            "chords_below_line_w": chords_lt, "white_gaps_below_line_w": gaps_lt,
            "min_chord_mm": round(min_chord[0], 3), "min_chord_print_z_mm": min_chord[1],
            "min_white_gap_mm": round(min_gap[0], 3) if math.isfinite(min_gap[0]) else None,
            "min_white_gap_print_z_mm": min_gap[1],
            "note": "information only (no threshold): single-layer orange specks or a bridged white wedge tip are "
                    "expected where a chord / gap is below one line width; the slicer drops or merges them"}


def livery_check(L: Layout, white: cq.Shape, inlay: cq.Shape, plain: cq.Shape | None = None) -> dict:
    """C36 measurements (see the module docstring); ``ok`` is the PASS verdict."""
    ck = L.livery_check
    clear_req = float(ck.get("clearance_min_mm", 3.0))
    plain = rg.avionics_module(L, with_livery=False) if plain is None else plain
    out: dict = {"copies_deg": list(L.livery_thetas), "logo_file": L.livery_file, "logo_file_sha256": L.livery_file_sha256}
    fails: list[str] = []
    # (d) union
    vp, vw, vi = mp.precise_volume(plain), mp.precise_volume(white), mp.precise_volume(inlay)
    v_wi = rg.intersection_volume(white, inlay)
    v_pi = rg.intersection_volume(plain, inlay)
    out["union"] = {"plain_mm3": vp, "white_mm3": vw, "inlay_mm3": vi, "white_plus_inlay_minus_plain_mm3": vw + vi - vp,
                    "white_x_inlay_mm3": v_wi, "plain_x_inlay_minus_inlay_mm3": v_pi - vi}
    if not (abs(vw + vi - vp) <= LIVERY_VOL_TOL_MM3 and math.isfinite(v_wi) and v_wi <= LIVERY_VOL_TOL_MM3
            and math.isfinite(v_pi) and abs(v_pi - vi) <= LIVERY_VOL_TOL_MM3):
        fails.append("union / containment")
    # outer area of the inlay (faces on the cylinder r = body_r) vs the polygons
    R = L.livery_r
    a_out = 0.0
    for f in inlay.Faces():
        if f.geomType() == "CYLINDER":
            # outer faces: every vertex at r = body_r (inner faces at r_in)
            if all(abs(math.hypot(v.X, v.Y) - R) < 1e-3 for v in f.Vertices()):
                a_out += f.Area()
    met = polygon_metrics(L)
    a_poly = met["area_mm2_per_copy"] * len(L.livery_thetas)
    out["outer_area"] = {"inlay_outer_faces_mm2": a_out, "polygons_mm2": a_poly, "rel": a_out / a_poly - 1.0}
    if not abs(a_out / a_poly - 1.0) <= LIVERY_AREA_REL_TOL:
        fails.append("inlay outer area != polygon area")
    # (c) depth
    lw = float(L.p["printer"]["line_w"])
    # the inlay's sides are parallel to the copy's radial direction (the outline prism), so V = A_out * t to second order
    # in v / r (a truly radial shell would carry (1 - t / 2r)): measured depth = inlay volume / outer area
    depth_meas = vi / a_out if a_out > 0 else math.nan
    out["depth"] = {"design_mm": L.livery_depth, "measured_mm": depth_meas, "line_w_mm": lw, "wall_mm": L.body_wall,
                    "white_behind_mm": L.livery_white_behind, "white_behind_min_mm": L.livery_white_behind_min}
    if not (L.livery_depth <= lw + 1e-9 and L.livery_white_behind >= L.livery_white_behind_min - 1e-9
            and abs(depth_meas - L.livery_depth) < 0.01):
        fails.append("inlay depth")
    # (b) polygons
    out["polygons"] = met
    if met["min_stroke_mm"] < float(ck.get("min_stroke_mm", 1.6)) - 1e-9:
        fails.append("min stroke")
    if met["min_white_gap_mm"] < float(ck.get("min_white_gap_mm", 1.2)) - 1e-9:
        fails.append("min white gap")
    if met["smallest_island_mm2"] < float(ck.get("min_island_mm2", 3.0)) - 1e-9:
        fails.append("smallest island")
    # (a) clearance: grown footprint shell inside the plain module, per copy
    ps = placed_polygons_sz(L)
    grown = _polys(unary_union(ps).buffer(clear_req, quad_segs=GROW_QUAD_SEGS))
    cl = {}
    for th in L.livery_thetas:
        shell = _shell_from_sz(L, grown, th, L.livery_r_in, R - SKIN_GAP)
        vs = mp.precise_volume(shell)
        missing = vs - rg.intersection_volume(shell, plain)
        cl[f"{th:g}"] = {"grown_shell_mm3": vs, "outside_material_mm3": missing}
        if not (math.isfinite(missing) and missing <= LIVERY_VOL_TOL_MM3):
            fails.append(f"clearance < {clear_req:g} mm at {th:g} deg")
    out["clearance_containment"] = cl
    # (a, F6) the void-band distance per copy - every void of the skin band, no volume tolerance
    vb = void_band_distances(L, inlay, plain)
    out["clearance_void_band"] = vb
    for th in L.livery_thetas:
        d_ = (vb.get(f"{th:g}") or {}).get("min_mm", math.nan)
        if not (math.isfinite(d_) and d_ >= clear_req - 1e-6):
            fails.append(f"clearance < {clear_req:g} mm at {th:g} deg (void band)")
    # (e, F7) per-layer chords / white gaps, information
    out["layer_chords"] = layer_chords(L)
    # reported distances (BRep, M) from the inlay to the known features and the part ends
    dist = {}
    for name, shapes in known_features(L).items():
        ds = [float(inlay.distance(s)) for s in shapes]
        dist[name] = round(min(ds), 3) if ds else None
    zlo, zhi = L.livery_z_range
    dist["part lower end (z_avionics_bottom)"] = round(zlo - L.z_avionics_bottom, 3)
    dist["visible skin upper end (spigot step)"] = round(L.z_avionics_spigot_bottom - zhi, 3)
    axes = []
    for th in L.livery_thetas:
        for jt in L.joint_thetas:
            dth = math.radians(((jt - th + 180.0) % 360.0) - 180.0)
            pt = shapely.Point(R * dth, L.joint_screw_dz)
            axes.append(min(q.distance(pt) for q in ps))
    dist["joint screw axis (on the skin)"] = round(min(axes), 3)
    out["distances_mm"] = dist
    out["clearance_min_reported_mm"] = min(v for v in dist.values() if v is not None)
    out["clearance_required_mm"] = clear_req
    if out["clearance_min_reported_mm"] < clear_req - 1e-6:
        fails.append("a reported distance < the clearance rule")
    out["fails"] = fails
    out["ok"] = not fails
    return out
