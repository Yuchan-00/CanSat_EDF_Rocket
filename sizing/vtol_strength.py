"""v0.9 CAD-2 hand checks behind every VTOL lightening step (architect: "each saving listed in grams with its strength /
function check").  Geometry from cad/layout.py (VTOL configuration), masses from the CadQuery VTOL reports, material
from params.json printer (bending 53 MPa, tensile 30 MPa).  Load cases reuse the v0.9 print-mass calibration's ASSUMED
factors (docs/print_mass_calibration.json 'strength'): one leg takes the whole weight x2 (dynamic), ground friction 0.5;
touchdown deceleration 20 g; plus the tether report's design loads (working 100 N, proof 300 N at the hardpoint).  Every
factor marked ASSUMED is an engineering assumption, not a measurement; these are hand calculations, not tests.

Output: docs/vtol_strength_checks.json.  Usage: python sizing/vtol_strength.py
"""
from __future__ import annotations

import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(ROOT, "cad"))
import layout  # noqa: E402

G = 9.80665
DYN, MU, TOUCHDOWN_G = 2.0, 0.5, 20.0          # ASSUMED (as the print-mass calibration)


def tsec_Z(flange_w: float, flange_t: float, web_t: float, web_h: float) -> tuple[float, float]:
    """(I, Z_min) of a T section (flange on top) in mm^4 / mm^3."""
    a1, a2 = flange_w * flange_t, web_t * web_h
    y1, y2 = flange_t / 2, flange_t + web_h / 2
    yc = (a1 * y1 + a2 * y2) / (a1 + a2)
    i = flange_w * flange_t ** 3 / 12 + a1 * (y1 - yc) ** 2 + web_t * web_h ** 3 / 12 + a2 * (y2 - yc) ** 2
    return i, i / max(yc, flange_t + web_h - yc)


def _section_props(polys: list) -> dict:
    """Area, centroid and the minimum section modulus (principal I_min / farthest point) of shapely polygons (Green)."""
    from shapely.geometry.polygon import orient
    tot = [0.0] * 6
    for p_ in polys:
        p_ = orient(p_, 1.0)
        for ring in [p_.exterior.coords] + [i.coords for i in p_.interiors]:
            c = list(ring)
            for (x0, y0), (x1, y1) in zip(c[:-1], c[1:]):
                cr = x0 * y1 - x1 * y0
                tot[0] += cr / 2
                tot[1] += (x0 + x1) * cr / 6
                tot[2] += (y0 + y1) * cr / 6
                tot[3] += (y0 * y0 + y0 * y1 + y1 * y1) * cr / 12
                tot[4] += (x0 * x0 + x0 * x1 + x1 * x1) * cr / 12
                tot[5] += (x0 * y1 + 2 * x0 * y0 + 2 * x1 * y1 + x1 * y0) * cr / 24
    A, Sx, Sy, Ix, Iy, Ixy = tot
    cx, cy = Sx / A, Sy / A
    Ixc, Iyc, Ixyc = Ix - A * cy * cy, Iy - A * cx * cx, Ixy - A * cx * cy
    avg, rad = 0.5 * (Ixc + Iyc), math.hypot(0.5 * (Ixc - Iyc), Ixyc)
    cmax = max(math.hypot(x - cx, y - cy) for p_ in polys for x, y in p_.exterior.coords)
    return {"area_mm2": A, "centroid_mm": [cx, cy], "I_min_mm4": avg - rad, "Z_min_mm3": (avg - rad) / cmax}


def clock_sections(L, sb: float) -> dict:
    """wire09c: plan sections of the exported propulsion module (rocket and VTOL STLs) at the clock-mount heights."""
    import numpy as np
    import trimesh
    out = {}
    for cfg, stl, rep_name, kw in (("vtol", "stl_vtol", "cadquery_report_vtol.json", {"config": "vtol"}),
                                   ("rocket", "stl", "cadquery_report.json", {})):
        Lc = layout.build(**kw)
        path = os.path.join(ROOT, "cad", "exports", "cadquery", stl, "propulsion_module.stl")
        with open(os.path.join(ROOT, "cad", "exports", rep_name), "r", encoding="utf-8") as f:
            m = json.load(f)["totals"]["mass_g"] * 1e-3
        mesh = trimesh.load(path, force="mesh")
        f_lat, f_ax = m * G * DYN * MU, m * G * TOUCHDOWN_G
        zs = {"fan floor + 2": Lc.lp_z_fan_floor + 2.0, "under the rim (bundle run)": Lc.z_edf_housing_bottom - 6.0,
              "bend top / cylinder start": Lc.z_cyl0 - 1.0, "tab band low": Lc.tab_z0 + 8.0,
              "tab band mid": 0.5 * (Lc.tab_z0 + Lc.tab_z1), "deck": Lc.z_prop_spigot_bottom - 0.5,
              "spigot mid": 0.5 * (Lc.z_prop_spigot_bottom + Lc.z_prop_spigot_top)}
        res = {"stl": os.path.relpath(path, ROOT).replace("\\", "/"), "mass_kg": m, "lateral_tip_load_N": f_lat,
               "touchdown_axial_N": f_ax, "sections": {}}
        fmin = math.inf
        for nm, z in zs.items():
            s = mesh.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
            pl, _ = s.to_2D(to_2D=np.eye(4))
            pr = _section_props(list(pl.polygons_full))
            sbm = f_lat * (z - Lc.fin_te_z) / pr["Z_min_mm3"]
            sax = f_ax / pr["area_mm2"]
            pr.update({"z_mm": z, "bending_mpa": sbm, "axial_mpa": sax, "factor": sb / max(sbm, sax)})
            res["sections"][nm] = pr
            fmin = min(fmin, pr["factor"])
        res["factor_min"] = fmin
        out[cfg] = res
    out["note"] = ("M geometry (STL slices), E loads (one leg 2 W x friction 0.5 bent from the fin TE; 20 g touchdown); the "
                   "rocket's flight loads are not in this table")
    return out


def main() -> int:
    L = layout.build(config="vtol")
    p = L.p
    sb, st = p["printer"]["bending_mpa"], p["printer"]["tensile_mpa"]
    with open(os.path.join(ROOT, "cad", "exports", "cadquery_report_vtol.json"), "r", encoding="utf-8") as f:
        rep = json.load(f)
    m = rep["totals"]["mass_g"] * 1e-3
    out: dict = {"assumed": {"one_leg_dynamic": DYN, "ground_friction": MU, "touchdown_g": TOUCHDOWN_G,
                             "tether_working_N": p["power"]["design_load_N_ASSUMED"]["working"],
                             "tether_proof_N": p["power"]["design_load_N_ASSUMED"]["proof"]},
                 "material_mpa": {"bending": sb, "tensile": st}, "vtol_mass_kg": m}
    # 1. duct 1.6 mm: lateral fin-tip load bent through the duct at the shoulder ring, axial touchdown, vane boss bearing
    t_duct = (L.vane_section_od - L.duct_id) / 2
    r_m = (L.vane_section_r + L.duct_r) / 2
    f_lat = m * G * DYN * MU
    mom = f_lat * (L.z_vane_section_top - L.fin_te_z)
    zt = math.pi * r_m ** 2 * t_duct
    f_ax = m * G * TOUCHDOWN_G
    out["duct_1p6"] = {"wall_mm": t_duct, "lateral_tip_load_N": f_lat, "bending_mpa": mom / zt,
                       "touchdown_axial_mpa": f_ax / (2 * math.pi * r_m * t_duct),
                       "boss_bearing_mpa": L.vane_side_load_N / (L.vane_shaft_d * (L.hinge_boss_r_out - L.duct_r)),
                       "factor_min": sb / max(mom / zt, f_ax / (2 * math.pi * r_m * t_duct)),
                       "note": "4 perimeters (class A) print the 1.6 mm wall solid; the fins / fairings stiffen it locally"}
    # 1b. fix round (review 2026-10-29 G8): the motor-lead notch opens the duct wall at wire_theta (0 deg, above fin 1)
    #     from its bottom z_b up into the 22 mm slot; the outer cone skin stays (>= cone_wall + skin_min_wall at the notch
    #     corner).  Conservative net section at the notch's worst height (its chamfer top, where it reaches furthest out):
    #     ONLY the outer cone shell (t = cone_wall, r = cone_outer_radius) carries - the inner duct wall interrupted by the
    #     notch is ignored all round.  Loads: the fin-tip lateral load (calibration case) bent up to that height, and the
    #     touchdown axial load
    if getattr(L, "ml_low_cut_uz", None):
        (_, z_b), _, (u_o, z_ct), _, _ = L.ml_low_cut_uz
        r_c = L.cone_outer_radius(z_ct) - L.cone_wall / 2
        z_c = math.pi * r_c ** 2 * L.cone_wall
        a_c = 2 * math.pi * r_c * L.cone_wall
        mom_n = f_lat * (z_ct - L.fin_te_z)
        out["lead_notch_section"] = {
            "notch_z_mm": [z_b, z_ct], "notch_w_mm": L.ml_low_cut_w, "skin_at_corner_mm": L.ml_low_cut_skin,
            "shell_r_mm": r_c, "shell_t_mm": L.cone_wall, "lateral_tip_load_N": f_lat, "bending_mpa": mom_n / z_c,
            "touchdown_axial_mpa": f_ax / a_c, "factor_min": sb / max(mom_n / z_c, f_ax / a_c),
            "note": "conservative: the outer cone shell alone (the notch leaves it whole); the inner duct wall beside the "
                    "notch and fin 1 below it are not counted"}
    # 1c. wire09c clock mount (user 2026-10-30): the lead passage opens the duct / sleeve at theta_P over the fan width and
    #     the keyways cut the sleeve all round.  Sections sliced from the EXPORTED module STLs (M geometry; trimesh), area
    #     and minimum section modulus about the centroid; loads as 1. (fin-tip lateral bent up to the height, touchdown
    #     axial) with each configuration's CAD mass.  The keyway floor pad: one tab carries the EDF at the touchdown
    #     (fixed-fixed, point load mid span and uniform), pad_t deep (the 45 deg corbel not credited)
    if getattr(L, "clock_mount", False):
        out["clock_seat_sections"] = clock_sections(L, sb)
        F = p["edf"]["mass_g"] * 1e-3 * G * TOUCHDOWN_G
        b_, t_, l_ = L.ck_pad_w, L.ck_pad_t, L.ck_pad_span
        Zp = b_ * t_ * t_ / 6.0
        pad = {"F_touchdown_N": F, "pad_w_mm": b_, "pad_t_mm": t_, "span_mm": l_}
        for n_tab in (1, 2):
            for nm, M in (("point", F / n_tab * l_ / 8.0), ("uniform", F / n_tab * l_ / 12.0)):
                pad[f"{n_tab}tab_{nm}"] = {"sigma_mpa": M / Zp, "factor": sb / (M / Zp)}
        pad["note"] = ("judge fix: 1.2 mm gave a factor 1.2 (one tab, point load); C18 grades the one-tab point-load factor "
                       "(PASS >= 5, WARN 3-5, FAIL < 3)")
        out["keyway_pad"] = pad
    # 2. fins = landing legs.  v0.9 fix round: SOLID fins (the CAD-2 hole is gone - mass reviewer: with the hole the bottom
    #    strip carries ~90 % of the lateral load, margin ~1.5x, not 4.0x).  Out-of-plane bending of the fin plate at the
    #    fairing's outer face (the root of the free leg) under (a) the calibration case (one leg, 2 W, friction 0.5) and (b) a
    #    single-leg touchdown at the firmware's touchdown rate with a leg spring 100-350 N/mm (ASSUMED) and m_eff ~ the
    #    vehicle mass (mass reviewer's estimate), friction 0.5; in-plane: the one-leg load over the free leg as a cantilever
    t = L.fin_t
    u_root = L.fair_u[1] if L.direct_drive else L.fin_root_r
    chord = L.fin_le_z(u_root) - L.fin_te_z
    z_oop = chord * t * t / 6.0
    arm = L.fin_tip_r - u_root
    v_td = p["mission"]["vtol"]["touchdown_rate_mps"]
    fin = {"root_u_mm": u_root, "chord_at_root_mm": chord, "Z_out_of_plane_mm3": z_oop, "free_leg_mm": arm}
    for nm, k_leg in (("touchdown, leg 100 N/mm (ASSUMED)", 100e3), ("touchdown, leg 350 N/mm (ASSUMED)", 350e3)):
        f_vert = v_td * math.sqrt(k_leg * m)
        mo = MU * f_vert * arm
        fin[nm] = {"vertical_N": f_vert, "lateral_N": MU * f_vert, "stress_mpa": mo / z_oop, "factor": sb / (mo / z_oop)}
    mo = f_lat * arm
    fin["calibration case (one leg 2 W, mu 0.5)"] = {"lateral_N": f_lat, "stress_mpa": mo / z_oop, "factor": sb / (mo / z_oop)}
    f_v = m * G * DYN
    z_in = t * chord ** 2 / 6.0
    fin["in_plane_one_leg"] = {"load_N": f_v, "stress_mpa": f_v * arm / z_in, "factor": sb / (f_v * arm / z_in)}
    if getattr(L, "has_fin_hole", False):      # params that still carry fin.hole: the reviewer's stiffness split
        (u0, z0), (u1, _), (_, z1), (um, za), _ = L.fin_hole_uz
        ls = u1 - u0
        k_bot = 3.0 / ls ** 3 * (z0 - L.fin_te_z) ** 3
        k_tip = 12.0 / ls ** 3 * (L.fin_tip_r - u1) ** 3
        share = k_bot / (k_bot + k_tip)
        zb_ = (z0 - L.fin_te_z) * t * t / 6.0
        fin["with_hole_bottom_strip"] = {"share": share, "stress_mpa": share * f_lat * (L.fin_tip_r - u0) / zb_}
    out["fins_solid"] = fin
    # 2b. v0.9 fix round: hinge bushing bearing, coupler socket jaw at MG90S stall, load washer (conservative plate)
    stall_ = p["servo"]["torque_kgcm_4v8"] * 0.0980665
    if getattr(L, "dd_bushing", False):
        bl = L.dd_bush_u[1] - L.dd_bush_u[0]
        out["hinge_bushing"] = {"side_load_N": L.vane_side_load_N, "length_mm": bl,
                                "projected_bearing_mpa": L.vane_side_load_N / (L.vane_shaft_d * bl),
                                "printed_bore_behind_mpa": L.vane_side_load_N / (L.dd_bush_od * bl),
                                "tilt_deg": L.dd_bush_tilt_deg}
    if hasattr(L, "dd_arm_contact"):
        r_eff = L.dd_collar_relief_d / 2 + 0.5 * L.dd_arm_contact
        f_flank = stall_ / (2 * r_eff * 1e-3)
        a_fl = L.dd_horn_arm_t * L.dd_arm_contact
        hm = L.vane_hinge_moment_kgcm * 0.0980665
        out["coupler_socket"] = {"stall_torque_Nm": stall_, "flank_force_at_stall_N": f_flank, "flank_area_mm2": a_fl,
                                 "bearing_at_stall_mpa": f_flank / a_fl,
                                 "bearing_at_hinge_moment_mpa": hm / (2 * r_eff * 1e-3) / a_fl,
                                 "jaw_at_arm_tip_mm": L.dd_jaw_at_tip,
                                 "note": "PLA compressive yield ~50-60 MPa (not measured): the stall case is the servo pushing "
                                         "a jammed vane, the flight case is the hinge moment"}
    if getattr(L, "cap_hardpoint", False):
        b_in, a_out, nu = L.cap_nut_af / math.sqrt(3), L.cap_washer_d / 2, 0.33
        wsh = {}
        for case in ("working", "proof"):
            P = p["power"]["design_load_N_ASSUMED"][case]
            s_ = 3 * P * (1 + nu) * math.log(a_out / b_in) / (2 * math.pi * L.cap_washer_t ** 2)
            wsh[case] = {"load_N": P, "plate_stress_mpa": s_, "factor_6061_t6_240mpa": 240.0 / s_}
        out["load_washer"] = {"d_t_mm": [L.cap_washer_d, L.cap_washer_t], "mass_g": L.cap_washer_mass_g, "cases": wsh,
                              "note": "ring load at the nut corners, simply supported at the washer edge (conservative: the "
                                      "hub carries the inner part directly)"}
    # 3. fairing walls 1.2: servo stall torque reacted by the case on the pocket walls (worst: the servo stalls)
    stall = p["servo"]["torque_kgcm_4v8"] * 0.0980665          # N m
    f_case = stall / (p["servo"]["body_l"] * 1e-3)
    out["fairing_1p2"] = {"servo_stall_Nm": stall, "case_couple_force_N": f_case,
                          "wall_bearing_mpa": f_case / (p["servo"]["case_top_from_bottom"] * L.fair_wall),
                          "hinge_moment_kgcm": L.vane_hinge_moment_kgcm,
                          "note": "the vane hinge moment (0.12 kgf cm) is 15 x below the stall torque used here"}
    # 4. top cap ribs (simply supported diametral beams, central load) + rim annulus shear + skirt screws
    tc = p["top_cap"]
    flange = min(16 * L.cap_plate_t, 2 * math.pi * L.cap_rib_r[1] / L.cap_rib_n)
    _, z_rib = tsec_Z(flange, L.cap_plate_t, L.cap_rib_t, L.cap_rib_h)
    cap = {}
    for case in ("working", "proof"):
        P = p["power"]["design_load_N_ASSUMED"][case]
        mo = (P / L.cap_rib_n) * (L.cap_rib_r[1] - L.cap_rib_r[0])
        cap[case] = {"load_N": P, "rib_moment_Nmm": mo, "rib_stress_mpa": mo / z_rib, "rib_factor": sb / (mo / z_rib),
                     "annulus_shear_mpa": P / (2 * math.pi * L.spigot_id / 2 * L.cap_plate_t),
                     "skirt_screw_bearing_mpa": P / 4 / (3.0 * L.body_wall),
                     "spigot_tear_out_mpa": P / 4 / (2 * (p["body"]["spigot_h"] - p["body"]["joint_screw_z_from_spigot_bottom"])
                                                    * p["body"]["spigot_wall"])}
    out["top_cap"] = {"rib_Z_mm3": z_rib, "flange_w_mm": flange, "cases": cap,
                      "verdict": "working load factor >= 3; proof (no bungee) only ~1.1 on the ribs: the bungee shock section "
                                 "is mandatory (tether report)", "ring": [tc["ring_w"], tc["ring_h"]]}
    # 5. windowed avionics tube: net section in a tether hang (whole mass below the cap x the bungee arrest), struts
    circ = 2 * math.pi * (L.body_r - L.body_wall / 2)
    open_w = len(L.avw_thetas) * L.avw_w
    net = (circ - open_w) * L.body_wall
    hang = {}
    for case in ("working", "proof"):
        P = p["power"]["design_load_N_ASSUMED"][case]
        hang[case] = {"load_N": P, "net_tension_mpa": P / net, "factor": st / (P / net)}
    above = sum(e["mass_g"] for n, e in rep["parts"].items() if e["com_mm"][2] > L.avw_z1) * 1e-3
    out["avionics_windows"] = {"windows": len(L.avw_thetas), "net_section_mm2": net, "hang": hang,
                               "touchdown_compression_N": above * G * TOUCHDOWN_G,
                               "touchdown_compression_mpa": above * G * TOUCHDOWN_G / net, "strut_w_mm": L.avw_strut_w,
                               "note": "the battery / tray hang on the tray stops below the windows; only the cap, GPS and "
                                       "switch sit above them. Do not carry the vehicle by the windowed band"}
    # 6. intake bulkhead 1.2 mm: suction between the bay (open to ambient through the windows) and the plenum
    rho, d = p["aero"]["rho_kg_m3"], L.duct_id * 1e-3
    a_e = math.pi / 4 * (d * d - (p["edf"]["exit_hub_fraction"] * d) ** 2)
    mdot = math.sqrt(p["edf"]["thrust_peak_N"] * rho * a_e)
    v_in = mdot / (rho * L.inlet_area_mm2 * 1e-6)
    q_in = 0.5 * rho * v_in ** 2
    dp = 10 * q_in                                               # x10 margin over the window dynamic pressure (ASSUMED)
    a_ = L.body_id / 2
    out["intake_bulkhead_1p2"] = {"inlet_velocity_mps": v_in, "window_q_pa": q_in, "design_dp_pa": dp,
                                  "clamped_plate_stress_mpa": 3 * dp * 1e-6 * a_ ** 2 / (4 * L.p["intake_module"]["bulkhead_t"] ** 2),
                                  "carrier_on_foam_touchdown_N": L.fcu_mass_g * 1e-3 * G * TOUCHDOWN_G}
    # 7. tray with lightening holes: battery inertia at touchdown through the remaining tray section
    (yi, yo) = L.tray_hole_y
    net_tray = (L.tray_w - 2 * (yo - yi) - 2 * 2 * p["avionics_module"]["strap_slot_h"]) * L.tray_t
    f_bat = p["battery"]["mass_g"] * 1e-3 * G * TOUCHDOWN_G
    out["tray_holes"] = {"net_section_mm2": net_tray, "battery_touchdown_N": f_bat, "stress_mpa": f_bat / net_tray,
                         "factor": st / (f_bat / net_tray)}
    with open(os.path.join(ROOT, "docs", "vtol_strength_checks.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(out, f, indent=1, ensure_ascii=False)
        f.write("\n")
    print(json.dumps({k: v for k, v in out.items() if k not in ("assumed", "material_mpa")}, indent=1, ensure_ascii=False)[:6000])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
