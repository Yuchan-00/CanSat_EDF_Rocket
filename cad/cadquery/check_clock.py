"""wire09c clock-mount checks (propulsion_module.tab_mount 'clock_keyways'; user 2026-10-30: 'make the motor-lead passage
generous - I do not want to measure').  Used by check_geometry.Checker for C18 / C21 / C29 (clock versions), the new rows
C34 (clock coverage) and C35 (axial retention), and the C8 lead envelopes.

Invariant enforced here: every PASS of the generous lead passage is backed by booleans / BRep distances on the solids over
the WHOLE motor_leads.generosity_box (every corner x exit error {-E_design, 0, +E_design}) - nothing is sampled away by a
monotonic argument - or by stated hand calculations (E) on Layout / params numbers (A).  No threshold of the screw-mount
rows is changed; the clock rows have their own thresholds, stated in the constants below.

Speed: one corner envelope x the local module is ~0.5 s, so the grid (648 module + 324 drop-in + the EDF set) takes
minutes.  The tether / hybrid configurations share the VTOL propulsion module and lead geometry: the grid result is
stored in cad/exports/c29_clock_grid/c29_clock_grid_<fingerprint>.json; the fingerprint is the module volume / area /
centre to 1e-3, the intake / vane / coupler / horn / servo volumes and areas, the clock / passage / generosity /
wire-clearance params, the EDF params, the stations, the nominal tab angles and this file's CHECK_VERSION; a result is
reused only when the fingerprint matches exactly (the row says so)."""
from __future__ import annotations

import hashlib
import json
import math
import os
import time

import cadquery as cq

import rocket_geometry as rg
from layout import Layout, clock_fan_outline

CHECK_VERSION = "wire09c-2"   # fix round: ribbon12x5_turned section, seat-chain keyways
PAD_FACTOR_PASS, PAD_FACTOR_WARN = 5.0, 3.0      # C18 clock: floor-pad bending factor vs printer.bending_mpa (judge)
CLOCK_CLEAR_TOL_MM3 = 0.01                       # C29 clock: a grown lead envelope / drop-in sweep meets no material
HOVER_TAB_GAP_MIN = 0.5                          # C29 clock: a hovering tab -> grown lead envelopes (BRep gap)
RETENTION_ENGAGED_MIN = 30.0                     # C35: keyway engagement left after the largest lift (mm)
RETENTION_FACTOR_MIN = 5.0                       # C35: shelf bending / keyway bearing factors
KEYWAY_BEARING_MPA = 30.0                        # C35: allowable bearing on the printed sleeve tooth (PETG/PLA, A)
BLISTER_WALL_TOL = 0.01                          # C21 clock: measured blister walls >= lead_passage.wall - 0.01
C21_MIN_SKIN_WALL = 0.8                          # as check_geometry.C21_MIN_SKIN_WALL (keyway -> outer surface)


# --------------------------------------------------------------------------------------------------------- helpers
def _v(a: cq.Shape, b: cq.Shape) -> float:
    v = rg.intersection_volume(a, b)
    return math.inf if math.isnan(v) else v


def _vpos(a: cq.Shape, b: cq.Shape) -> float:
    v = rg.intersection_volume(a, b)
    return 0.0 if math.isnan(v) else v


def _v_env(env: cq.Shape, target: cq.Shape) -> float:
    """Intersection of a compound envelope with target as the sum over its solids (an upper bound of the true overlap;
    0 means no contact).  Uncomputable -> +inf."""
    tot = 0.0
    tb = target.BoundingBox()
    for s in env.Solids():
        bb = s.BoundingBox()
        if bb.xmax < tb.xmin or bb.xmin > tb.xmax or bb.ymax < tb.ymin or bb.ymin > tb.ymax or bb.zmax < tb.zmin \
                or bb.zmin > tb.zmax:
            continue
        tot += _v(s, target)
    return tot


def _gap_env(env: cq.Shape, target: cq.Shape) -> float:
    d = math.inf
    for s in env.Solids():
        try:
            d = min(d, float(s.distance(target)))
        except Exception:  # noqa: BLE001
            return 0.0
    return d if math.isfinite(d) else 0.0


def local_module(L: Layout, prop: cq.Shape) -> cq.Shape:
    """The propulsion module around the lead passage: its plan widened by 8 deg and 3 mm, from 4 mm under the fan floor
    up past the top (a lead envelope cannot reach module material elsewhere without crossing this region)."""
    reg = clock_fan_outline(L.wire_theta, L.lp_E_design + 8.0, 5.0, L.lp_blister_r_out + 3.0, L.lp_blister_hw + 3.0,
                            "chord")
    return prop.intersect(rg._plan_prism(reg, L.lp_z_fan_floor - 4.0, L.z_prop_top + 30.0))


def fingerprint(L: Layout, prop: cq.Shape, shape: dict | None = None) -> str:
    p = L.p
    c = prop.Center()
    others = {}
    for nm in sorted(shape or {}):
        if nm == "intake_module" or nm.startswith(("vane_", "servo_horn_", "servo_vane_", "vane_coupler_")):
            others[nm] = [round(shape[nm].Volume(), 3), round(shape[nm].Area(), 3)]
    key = {"v": round(prop.Volume(), 3), "a": round(prop.Area(), 3), "c": [round(c.x, 3), round(c.y, 3), round(c.z, 3)],
           "clock": p["propulsion_module"]["clock_keyways"], "lp": p["propulsion_module"]["lead_passage"],
           "gbox": p["motor_leads"]["generosity_box"], "wcl": p["motor_leads"]["wire_clearance"],
           "count": p["motor_leads"]["count"], "edf": {k: v for k, v in p["edf"].items() if not isinstance(v, str)},
           "z": [L.z_edf_housing_bottom, L.z_intake_bottom, L.z_prop_top], "tabs": list(L.tab_thetas),
           "others": others, "version": CHECK_VERSION}
    return hashlib.sha256(json.dumps(key, sort_keys=True, default=str).encode()).hexdigest()


def corner_tag(c: dict, e: float) -> str:
    return (f"S{c['S']:g} {c['sec']} od{c['od']:g} R{c['R']:g} gap{c['gap']:g} inset{c['inset']:g} early{c['early']:g} "
            f"e{e:+.2f}")


# --------------------------------------------------------------------------------------------------------- the grid
def clock_grid(L: Layout, shape: dict, cache_path: str | None = None, log=print) -> dict:
    """Every generosity-box corner x e in {-E, 0, +E}: grown envelope x local module, grown envelope x intake, drop-in
    sweep x local module, raw envelope x EDF (TE inset 0 / 3 with the hub ending at the TE (A); motor protrusion 36 at
    every e, 30 at e 0; the nominal seat and every hover seat at e 0), the hovering tabs -> grown envelopes (BRep gap),
    negative controls (the widest corner turned 6 deg past E_design; the R15 corner pushed 3 mm past the wall)."""
    prop = shape["propulsion_module"]
    fp = fingerprint(L, prop, shape)
    if cache_path and os.path.isdir(cache_path):
        cache_path = os.path.join(cache_path, f"c29_clock_grid_{fp[:16]}.json")
    if cache_path and os.path.exists(cache_path):
        try:
            with open(cache_path, encoding="utf-8") as f:
                old = json.load(f)
            if old.get("fingerprint") == fp:
                old["from_cache"] = os.path.basename(cache_path)
                return old
        except (OSError, ValueError):
            pass
    t0 = time.time()
    E = L.lp_E_design
    es = (-E, 0.0, E)
    loc = local_module(L, prop)
    intake = shape["intake_module"]
    res: dict = {"fingerprint": fp, "E_design_deg": E, "e_values_deg": list(es)}
    worst_mod, worst_int, worst_drop = (0.0, ""), (0.0, ""), (0.0, "")
    n_mod = n_drop = 0
    for c in L.clock_corners(module_only=True):
        for e in es:
            env = rg.clock_lead_env(L, c, e, True)
            x = _v_env(env, loc)
            n_mod += 1
            if x >= worst_mod[0]:
                worst_mod = (x, corner_tag(c, e))
            xi = _v_env(env, intake)
            if xi >= worst_int[0]:
                worst_int = (xi, corner_tag(c, e))
            if c["early"] == 0.0:          # the drop-in box spans u rm .. r_v + rho: the early 0 corner contains early 1
                x = _v_env(rg.clock_lead_dropin(L, c, e), loc)
                n_drop += 1
                if x >= worst_drop[0]:
                    worst_drop = (x, corner_tag(c, e))
        log(f"  [C29 grid] {n_mod} envelopes, {time.time() - t0:.0f} s")
    res.update({"module_envelopes": n_mod, "dropin_sweeps": n_drop, "grown_x_module_max": list(worst_mod),
                "grown_x_intake_max": list(worst_int), "dropin_x_module_max": list(worst_drop)})
    # raw envelopes x EDF
    nom = list(L.tab_thetas)
    hov = [[c["a"], c["b"]] for c in L.ck_hover]
    edf_cache: dict = {}

    def edf(tabs, prot, lift):
        k = (tuple(round(t, 3) for t in tabs), prot, lift)
        if k not in edf_cache:
            edf_cache[k] = rg.edf_unit_variant(L, tabs, prot, lift)
        return edf_cache[k]

    worst_edf, n_edf = (0.0, ""), 0
    for c in L.clock_corners():
        if c["gap"] != min(L.gb_gap) or c["early"] != 0.0:
            continue                       # gap 0 = the bundle highest (nearest the rim / hub); early only moves the leg out
        for e in es:
            raw = rg.clock_lead_env(L, c, e, False)
            variants = [(nom, max(L.gb_protrusion))]
            if e == 0.0:
                variants += [(nom, min(L.gb_protrusion))] + [(h, max(L.gb_protrusion)) for h in hov]
            for tabs, prot in variants:
                x = _v_env(raw, edf(tabs, prot, c["inset"]))
                n_edf += 1
                if x >= worst_edf[0]:
                    worst_edf = (x, corner_tag(c, e) + f" tabs {tabs[0]:.1f} prot {prot:g}")
    res.update({"raw_x_edf_tests": n_edf, "raw_x_edf_max": list(worst_edf),
                "edf_assumption": "with the stator TE inset into the housing the hub ends at the TE (hub bottom raised by the "
                                  "inset) - ASSUMED"})
    log(f"  [C29 grid] EDF done {time.time() - t0:.0f} s")
    # hovering tabs -> grown envelopes (the thickest leads hugging 0 / bend_tol inside the wall)
    hg = math.inf
    for h in hov:
        for a in h:
            if any(abs(((a - c["b"]) + 180.0) % 360.0 - 180.0) < 0.01 for c in L.ck_hover):
                tab = rg.edf_tab(L, a)
                for c in L.clock_corners(module_only=True):
                    if c["od"] != max(L.gb_od) or c["S"] != max(L.gb_S) or c["sec"] != "round12" or c["gap"] != max(L.gb_gap):
                        continue
                    for e in es:
                        hg = min(hg, _gap_env(rg.clock_lead_env(L, c, e, True), tab))
    res["hover_tab_gap_min_mm"] = hg if math.isfinite(hg) else None
    # negative controls (the check must see a collision)
    cw = {"S": max(L.gb_S), "sec": "round12", "h": L.gb_sections["round12"][0], "w": L.gb_sections["round12"][1],
          "od": max(L.gb_od), "R": max(L.gb_R), "gap": max(L.gb_gap), "inset": 0.0, "early": 0.0}
    res["negative_control_e_plus6_mm3"] = _v_env(rg.clock_lead_env(L, cw, E + 6.0, True), loc)
    res["negative_control_wall_plus3_mm3"] = _v_env(rg.clock_lead_env(L, dict(cw, early=-3.0), 0.0, True), loc)
    res["negative_control_dropin_e_plus6_mm3"] = _v_env(rg.clock_lead_dropin(L, cw, E + 6.0), loc)
    # clearances of the widest envelopes (S max / min, R max / min, both e ends, round12 + od max, gap max)
    sel = [c for c in L.clock_corners(module_only=True) if c["od"] == max(L.gb_od) and c["sec"] == "round12"
           and c["gap"] == max(L.gb_gap) and c["early"] == 0.0 and c["S"] in (min(L.gb_S), max(L.gb_S))
           and c["R"] in (min(L.gb_R), max(L.gb_R))]
    envs = cq.Compound.makeCompound([s for c in sel for e in (-E, E) for s in rg.clock_lead_env(L, c, e, True).Solids()])
    k0 = min(range(L.vane_count), key=lambda i: abs(((L.vane_thetas[i] - L.wire_theta) + 180.0) % 360.0 - 180.0))
    dmax = float(L.p["vane"].get("max_deflection_mech_deg", 25.0))
    g = {f"vane_{k0 + 1} d={d:+.0f}": _gap_env(envs, rg.vane(L, k0, d)) for d in (0.0, dmax, -dmax)}
    for nm in (f"vane_coupler_{k0 + 1}", f"servo_horn_{k0 + 1}", f"servo_vane_{k0 + 1}"):
        if nm in shape:
            g[nm] = _gap_env(envs, shape[nm])
    kf = min(range(len(L.fin_thetas)), key=lambda i: abs(((L.fin_thetas[i] - L.wire_theta) + 180.0) % 360.0 - 180.0))
    g[f"fin_{kf + 1} plate"] = _gap_env(envs, rg.fin(L, L.fin_thetas[kf]))
    g["intake module"] = _gap_env(envs, intake)
    res["clearances_mm"] = g
    res["runtime_s"] = time.time() - t0
    if cache_path:
        try:
            with open(cache_path, "w", encoding="utf-8", newline="\n") as f:
                json.dump(res, f, indent=1, default=str)
        except OSError:
            pass
    return res


# --------------------------------------------------------------------------------------------------------- C18
def pad_hand_calc(L: Layout) -> dict:
    """Floor-pad bending (E): one tab carries the EDF at the touchdown (edf.mass_g x g x touchdown_g from
    docs/vtol_strength_checks.json, 20 g), the pad a fixed-fixed beam over the sleeve -> skin span, kw + 2 pad_margin wide,
    pad_t deep (the corbel is not credited): point load mid span M = F l / 8, uniform M = F l / 12; factor = bending /
    sigma.  Two tabs: half the load."""
    root = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
    with open(os.path.join(root, "docs", "vtol_strength_checks.json"), encoding="utf-8") as f:
        vs = json.load(f)
    g_td = float(vs["assumed"]["touchdown_g"])
    sb = float(L.p["printer"]["bending_mpa"])
    F = L.p["edf"]["mass_g"] * 1e-3 * 9.80665 * g_td
    b, t, l_ = L.ck_pad_w, L.ck_pad_t, L.ck_pad_span
    Z = b * t * t / 6.0
    out = {"touchdown_g": g_td, "F_N": F, "pad_w_mm": b, "pad_t_mm": t, "span_mm": l_, "sb_mpa": sb}
    for n_tab in (1, 2):
        for nm, M in (("point", F / n_tab * l_ / 8.0), ("uniform", F / n_tab * l_ / 12.0)):
            out[f"{n_tab}tab_{nm}_factor"] = sb / (M / Z)
    out["factor_status"] = out["1tab_point_factor"]
    return out


def c18_clock(L: Layout, prop: cq.Shape, intake: cq.Shape) -> tuple[str, str, str, dict]:
    worst_sweep, worst_seat, fills, walls = 0.0, 0.0, [], []
    h = L.ck_kw / 2
    for a in L.ck_keyways:
        sweep = rg._box_polar(L.tab_r_in - rg.EDF_TAB_INTO_HOUSING, L.tab_r_out, -L.tab_t / 2, L.tab_t / 2, L.lp_z_floor,
                              L.keyway_z1 + (L.tab_z1 - L.tab_z0), a)
        worst_sweep = max(worst_sweep, _v(sweep, prop))
        tab = rg.edf_tab(L, a)
        worst_seat = max(worst_seat, _v(tab, prop), _v(tab, intake))
        pr = rg._box_polar(L.edf_bore_r + 0.1, L.tab_r_out - 0.1, -L.tab_t / 2, L.tab_t / 2, L.lp_z_floor - 0.3,
                           L.lp_z_floor - 0.05, a)
        fills.append((a, _vpos(pr, prop) / pr.Volume()))
        for s in (-1, 1):
            v0, v1 = sorted((s * (h + 0.05), s * (h + 1.0)))
            pr = rg._box_polar(L.edf_bore_r + 0.1, L.sleeve_r - 0.1, v0, v1, L.tab_z0 + 1.0, L.tab_z1 - 1.0, a)
            walls.append((a, _vpos(pr, prop) / pr.Volume()))
    pad = pad_hand_calc(L)
    f = pad["factor_status"]
    fmin, wmin = min(fills, key=lambda t: t[1]), min(walls, key=lambda t: t[1])
    geo_ok = worst_sweep < 1.0 and worst_seat < 1.0 and fmin[1] >= 1.0 - 1e-4 and wmin[1] >= 1.0 - 1e-4
    st = ("FAIL" if (not geo_ok or f < PAD_FACTOR_WARN) else "WARN" if f < PAD_FACTOR_PASS else "PASS")
    meas = (f"{len(L.ck_keyways)} keyways: tab sweep (floor -> out of the top) x module max {worst_sweep:.3f}, seated tab x "
            f"(module|intake) {worst_seat:.3f} mm^3; floor fill under the tab min {fmin[1]:.4f} ({fmin[0]:.2f} deg), sleeve "
            f"teeth fill min {wmin[1]:.4f}; floor pad (E, 1 tab, point load {pad['F_N']:.1f} N at {pad['touchdown_g']:g} g) "
            f"bending factor {f:.2f} (PASS >= {PAD_FACTOR_PASS:g}, WARN >= {PAD_FACTOR_WARN:g}; uniform "
            f"{pad['1tab_uniform_factor']:.2f}, 2 tabs {pad['2tab_point_factor']:.2f} / {pad['2tab_uniform_factor']:.2f})")
    det = (f"clock mount: keyways kw {L.ck_kw:g} (tabs <= {L.ck_tab_t_max:g} thick) r {L.keyway_r_in:.2f}..{L.keyway_r_out:.2f}, "
           f"floor z {L.lp_z_floor:.2f} (tab bottom {L.tab_z0:.2f} - floor_drop {L.ar_floor_drop:g}: the foam pad), pad "
           f"{L.ck_pad_t:g} x {L.ck_pad_w:g} over span {L.ck_pad_span:.2f} mm (sleeve -> skin), 45 deg corbel; keyways at "
           + ", ".join(f"{a:.2f}" for a in L.ck_keyways))
    return st, meas, det, {"sweep": worst_sweep, "seat": worst_seat, "floor_fill": fills, "teeth_fill": walls, "pad": pad}


# --------------------------------------------------------------------------------------------------------- C21
def blister_walls(L: Layout, prop: cq.Shape) -> dict:
    """Measured blister wall thicknesses: both side walls (perpendicular to the side lines at theta_P +- E_design), the
    outer wall (radial at theta_P) and the floor (vertical at three radii), at three heights."""
    th, E, hw, rw, w = L.wire_theta, L.lp_E_design, L.lp_hw, L.lp_r_wall, L.lp_wall
    zs = [L.lp_z_fan_floor + 5.0, 0.5 * (L.lp_z_fan_floor + L.lp_z_blister_top), L.lp_z_blister_top - 5.0]
    out: dict[str, float] = {}
    for s in (-1, 1):
        for u in (0.5 * (L.body_r + rw), rw - 4.0):
            for z in zs:
                pr = rg._box_polar(u - 0.05, u + 0.05, min(s * (hw - 1.0), s * (hw + w + 2.0)),
                                   max(s * (hw - 1.0), s * (hw + w + 2.0)), z, z + 0.1, th + s * E)
                com = prop.intersect(pr)
                if com.wrapped.IsNull() or com.Volume() < 1e-9:
                    out[f"side {s:+d} u{u:.1f} z{z:.1f}"] = 0.0
                    continue
                c2 = rg._rot_z(com, -(th + s * E))
                bb = c2.BoundingBox()
                out[f"side {s:+d} u{u:.1f} z{z:.1f}"] = bb.ymax - bb.ymin
    for z in zs:
        pr = rg._box_polar(rw - 1.0, rw + w + 2.0, -0.05, 0.05, z, z + 0.1, th)
        com = prop.intersect(pr)
        out[f"outer z{z:.1f}"] = 0.0 if com.wrapped.IsNull() else (lambda b: b.xmax - b.xmin)(rg._rot_z(com, -th).BoundingBox())
    for u in (L.body_r + 2.0, 0.5 * (L.body_r + rw), rw - 2.0):
        pr = rg._box_polar(u - 0.05, u + 0.05, -0.05, 0.05, L.lp_z_blister_bottom - 0.5, L.lp_z_fan_floor + 1.0, th)
        com = prop.intersect(pr)
        if com.wrapped.IsNull() or com.Volume() < 1e-9:
            out[f"floor u{u:.1f}"] = 0.0
        else:
            bb = com.BoundingBox()
            out[f"floor u{u:.1f} (from the fan floor down)"] = L.lp_z_fan_floor - max(bb.zmin, L.lp_z_blister_bottom)
    return out


# --------------------------------------------------------------------------------------------------------- C34
def coverage_recheck(L: Layout, step: float = 0.01) -> dict:
    """Independent of Layout's own loop: from the keyway list and the hover seats, every exit azimuth phi in
    [-180, 180) step 0.01 deg - the best seat's residual to theta_P; gaps where it exceeds E_cov."""
    angs = [c["a"] for c in L.ck_configs] + [c["a"] + 180.0 for c in L.ck_configs]
    n = int(round(360.0 / step))
    worst, bad, gaps, s0 = 0.0, 0, [], None
    for i in range(n):
        phi = -180.0 + step * i
        best = min(abs(((a + phi - L.wire_theta) + 180.0) % 360.0 - 180.0) for a in angs)
        worst = max(worst, best)
        ok = best <= L.lp_E_cov + 1e-9
        bad += not ok
        if not ok and s0 is None:
            s0 = phi
        if ok and s0 is not None:
            gaps.append((round(s0, 2), round(phi - step, 2)))
            s0 = None
    if s0 is not None:
        gaps.append((round(s0, 2), 180.0))
    return {"coverage": 1.0 - bad / n, "worst_residual_deg": worst, "gaps": gaps, "seats": len(set(round(a % 360, 3) for a in angs))}


# --------------------------------------------------------------------------------------------------------- C35
def design_torque(L: Layout) -> tuple[float, float]:
    """(full-power motor torque P / omega (E), C35 design torque max(1, 1.5 x it)) in N m."""
    e = L.p["edf"]
    omega = e["kv"] * e["voltage_nominal_V"] * 2 * math.pi / 60.0
    torque = e["power_nominal_W"] / omega
    return torque, max(1.0, 1.5 * torque)


def teeth_bending(L: Layout, prop: cq.Shape) -> dict:
    """Sleeve-tooth tangential bending (wire09c fix round, review F1), measured on the solid (M geometry, E loads): the
    module is cut by a thin ring inside the sleeve (edf_bore_r + 0.05 .. sleeve_r - 0.05) at mid height of the free span
    -> every piece is a tooth between two keyways or a keyway and the lead passage; its width is the arc at the inner
    radius, its radial depth from a radial probe at its middle, its free span from the measured keyway floors (the
    lower neighbour) to the measured deck bottom (the lowest material over the cavity just outside the tooth; where a
    web / wall sits at the tooth middle the largest free deck bottom of the other teeth is used - webs are never
    credited).  Load: the C35 design torque on ONE tab (hover seat; single contact in a two-tab seat; either turning
    direction), F = torque / tab mid radius, fixed-fixed, uniform over the span (M = F l / 12; point load F l / 8
    reported), sigma = M / (t w^2 / 6); factor = printer.bending_mpa / sigma >= RETENTION_FACTOR_MIN for every tooth."""
    sb = float(L.p["printer"]["bending_mpa"])
    _, t_des = design_torque(L)
    r_mid = 0.5 * (L.tab_r_in + L.tab_r_out)
    F = t_des / (r_mid * 1e-3)
    z_mid = 0.5 * (L.lp_z_floor + L.z_prop_spigot_bottom - L.body_wall)
    r0, r1 = L.edf_bore_r + 0.05, L.sleeve_r - 0.05
    slab = prop.intersect(rg._tube(r1, r0, z_mid, z_mid + 0.2))
    h = L.ck_kw / 2

    def wrap(a: float) -> float:
        return (a + 180.0) % 360.0 - 180.0

    floors: dict[float, float] = {}
    for a in L.ck_keyways:
        pr = rg._box_polar(L.edf_bore_r + 0.2, L.sleeve_r - 0.2, -0.3, 0.3, L.lp_z_floor - 8.0, z_mid, a)
        com = prop.intersect(pr)
        floors[a] = max((s.BoundingBox().zmax for s in com.Solids()), default=math.nan)
    teeth, deck_free = [], []
    for s in slab.Solids():
        c = s.Center()
        a_c = math.degrees(math.atan2(c.y, c.x))
        angs = [wrap(math.degrees(math.atan2(v.Y, v.X)) - a_c) for v in s.Vertices()
                if abs(math.hypot(v.X, v.Y) - r0) < 0.02]
        if len(angs) < 2:
            continue
        lo_a, hi_a = min(angs), max(angs)
        mid = a_c + 0.5 * (lo_a + hi_a)
        w = r0 * math.radians(hi_a - lo_a)
        sides = []
        for edge in (a_c + lo_a, a_c + hi_a):
            k = min(L.ck_keyways, key=lambda x: abs(wrap(x - edge)))
            sides.append(k if abs(wrap(k - edge)) < math.degrees(h / r0) + 1.0 else None)
        rp = rg._rot_z(prop.intersect(rg._box_polar(L.edf_bore_r - 1.0, L.sleeve_r + 3.0, -0.2, 0.2, z_mid, z_mid + 0.2,
                                                    mid)), -mid).BoundingBox()
        t = rp.xmax - rp.xmin if rp.xmax > rp.xmin else L.sleeve_r - L.edf_bore_r
        t = min(t, L.sleeve_r - L.edf_bore_r + 0.05)     # a web / wall at the middle is not credited
        cav = prop.intersect(rg._box_polar(L.sleeve_r + 0.3, L.sleeve_r + 0.9, -0.3, 0.3, L.lp_z_floor, L.keyway_z1,
                                           mid))
        above = [b.BoundingBox() for b in cav.Solids()]
        anchored = any(b.zmin <= z_mid + 0.2 <= b.zmax or b.zmin <= z_mid <= b.zmax for b in above)
        z_top = min((b.zmin for b in above if b.zmin > z_mid), default=math.nan)
        if not anchored and math.isfinite(z_top):
            deck_free.append(z_top)
        fl = [floors[k] for k in sides if k is not None and math.isfinite(floors[k])]
        teeth.append({"mid_deg": round(mid % 360.0, 3), "w_mm": w, "t_mm": t, "sides": sides, "anchored_mid": anchored,
                      "z_top": z_top, "z_floor": min(fl) if fl else math.nan})
    z_deck = max(deck_free) if deck_free else L.z_prop_spigot_bottom - L.body_wall
    worst = None
    for td in teeth:
        if td["sides"] == [None, None]:
            td["factor"] = math.inf          # no keyway beside it (cannot be pushed by a tab)
            continue
        zt = td["z_top"] if (math.isfinite(td["z_top"]) and not td["anchored_mid"]) else z_deck
        zf = td["z_floor"] if math.isfinite(td["z_floor"]) else L.lp_z_floor
        span = zt - zf
        Z = td["t_mm"] * td["w_mm"] ** 2 / 6.0
        td["span_mm"] = span
        td["factor"] = sb / (F * span / 12.0 / Z)
        td["factor_point"] = sb / (F * span / 8.0 / Z)
        if worst is None or td["factor"] < worst["factor"]:
            worst = td
    n_kw_sides = sum(1 for td in teeth for s_ in td["sides"] if s_ is not None)
    ok = (worst is not None and worst["factor"] >= RETENTION_FACTOR_MIN and n_kw_sides == 2 * len(L.ck_keyways))
    return {"torque_design_Nm": t_des, "tab_force_N": F, "z_mid": z_mid, "teeth": teeth, "keyway_floors": floors,
            "z_deck_bottom": z_deck, "keyway_sides_found": n_kw_sides, "worst": worst, "ok": ok,
            "factor_min": worst["factor"] if worst else math.nan, "w_min_mm": min(td["w_mm"] for td in teeth)}


def retention(L: Layout, intake: cq.Shape) -> dict:
    """Axial retention (A / E): foam compression in every stack case within [min, max]; the largest lift (lip_stop_gap +
    stack tolerance) and the keyway engagement left; the shelf (on the solid: its underside z over the lip) carrying the
    full-power thrust (edf.thrust_peak_N) as an annular cantilever from the lip-pocket wall; the keyway tooth bearing the
    full-power motor torque on ONE tab (a hover seat); the foam preload vs the EDF weight (A); the EDF's static weight
    on 2 / 1 foam pads (sag and spin-up travel to the shelf, reported, A - fix round F4)."""
    p = L.p
    e, ar, im = p["edf"], p["edf"]["axial_retention"], p["intake_module"]
    sb = float(p["printer"]["bending_mpa"])
    comp = dict(L.ar_foam_compression)
    lo, hi = float(ar["min_compression"]), float(ar["max_compression"])
    comp_ok = all(lo - 1e-9 <= v <= hi + 1e-9 for v in comp.values())
    # shelf on the solid: material over the lip ring just above the lip front face + lip_stop_gap
    r_c = 0.5 * (im["shelf_id"] / 2 + min(e["lip_od"], L.lip_pocket_id) / 2)
    probe = rg._tube(r_c + 0.5, r_c - 0.5, L.z_lip_pocket_top + 0.05, L.z_lip_pocket_top + im["shelf_t"] - 0.05)
    # the shelf is notched at the motor-lead gate and the servo-lead channel on purpose: those sectors (half width + 2 mm)
    # are left out of the probe ring
    notches = [(L.wire_theta, L.wire_gate_w / 2 + 2.0)]
    if getattr(L, "servo_leads", False):
        notches.append((L.sl_gather_theta, L.sl_ch_w / 2 + 2.0))
    for th_n, hw_n in notches:
        probe = probe.cut(rg._box_polar(r_c - 2.0, r_c + 2.0, -hw_n, hw_n, L.z_lip_pocket_top - 1.0,
                                        L.z_lip_pocket_top + im["shelf_t"] + 1.0, th_n))
    shelf_fill = _vpos(probe, intake) / probe.Volume()
    gap_probe = rg._tube(r_c + 0.5, r_c - 0.5, L.z_edf_lip_top + 0.05, L.z_lip_pocket_top - 0.05)
    gap_mat = _v(gap_probe, intake)
    T = float(e["thrust_peak_N"])
    r_w = L.lip_pocket_id / 2
    m = T / (2 * math.pi * r_w) * (r_w - r_c)
    sig = 6 * m / im["shelf_t"] ** 2
    shelf_factor = sb / sig
    torque, t_des = design_torque(L)
    f_tab = t_des / (0.5 * (L.tab_r_in + L.tab_r_out) * 1e-3)
    a_bear = (L.sleeve_r - L.edf_bore_r) * L.tab_len
    bear = f_tab / a_bear
    area_foam = 2 * L.tab_t * (L.tab_r_out - L.tab_r_in)
    s25, s50 = float(ar["foam_sigma_25pct_kpa_ASSUMED"]), float(ar["foam_sigma_50pct_kpa_ASSUMED"])

    def sigma(c: float) -> float:
        c = max(0.0, c)
        return s25 * c / 0.25 if c <= 0.25 else s25 + (s50 - s25) * (c - 0.25) / 0.25

    pre = sigma(comp["drawn position (lip_stop_gap under the shelf)"]) * 1e-3 * area_foam
    w_edf = e["mass_g"] * 1e-3 * 9.80665
    # fix round (review F4): the EDF's own weight on the foam at rest (A curve; reported - the compression band above
    # is the SEATING rule for the drawn stack, a different case).  Sag = drawn foam thickness (floor_drop) - the
    # thickness under the weight; travel to the shelf at spin-up = lip_stop_gap + stack tol + sag.  Beyond the 50 %
    # point of the assumed curve the foam is taken as bottoming: the bound is a flat foam (sag = floor_drop).
    static = {}
    for n_pad, nm in ((2, "2 pads (two-tab seat)"), (1, "1 pad (hover seat)")):
        s_kpa = w_edf / (n_pad * L.tab_t * (L.tab_r_out - L.tab_r_in)) * 1e3
        if s_kpa <= s25:
            c_ = 0.25 * s_kpa / s25
        elif s_kpa <= s50:
            c_ = 0.25 + 0.25 * (s_kpa - s25) / (s50 - s25)
        else:
            c_ = math.nan
        sag = (L.ar_floor_drop - L.ar_foam_t * (1.0 - c_)) if math.isfinite(c_) else L.ar_floor_drop
        static[nm] = {"stress_kpa": s_kpa, "compression": c_, "beyond_curve_bottoms": not math.isfinite(c_),
                      "sag_below_drawn_mm": sag, "travel_to_shelf_mm": L.ar_lip_gap + float(ar["stack_tol_ASSUMED"])
                      + max(sag, 0.0)}
    out = {"foam_compression": comp, "compression_band": [lo, hi], "compression_ok": comp_ok,
           "static_weight_A": static,
           "lift_max_mm": L.ar_lift_max, "engaged_after_lift_mm": L.ar_engaged_after_lift,
           "shelf_fill_over_lip": shelf_fill, "lip_to_shelf_gap_material_mm3": gap_mat,
           "thrust_peak_N": T, "shelf_sigma_mpa": sig, "shelf_factor": shelf_factor,
           "motor_torque_Nm_E": torque, "torque_design_Nm": t_des, "tab_force_N": f_tab, "bearing_mpa": bear,
           "bearing_factor": KEYWAY_BEARING_MPA / bear, "foam_preload_N_A": pre, "edf_weight_N": w_edf,
           "foam_holds_edf_up_at_rest": pre >= w_edf}
    out["ok"] = (comp_ok and L.ar_engaged_after_lift >= RETENTION_ENGAGED_MIN and shelf_fill >= 0.999 and gap_mat < 0.01
                 and shelf_factor >= RETENTION_FACTOR_MIN and out["bearing_factor"] >= RETENTION_FACTOR_MIN)
    return out
