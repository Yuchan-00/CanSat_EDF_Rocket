"""v0.9 CAD-2 sizing of the VTOL configuration (mission.config 'vtol') and the four-configuration thrust table.

User decisions 2 / 4 / 5 of 2026-10-27: the VTOL airframe flies with a battery, battery-less on a tether, or as a hybrid
(tether + buffer pack); the EDF thrust is UNKNOWN (the seller's 30.3 N / 3.09 kgf is unverified), so every T/W and hover
throttle is a TABLE over T_max = 2.0 / 2.3 / 2.6 / 3.09 kgf.

Invariants: masses, CG and inertia come only from the CadQuery twin's reports (cad/exports/cadquery_report[_vtol...].json,
built from this params.json; a report from another params.json, with failed components or missing parts is refused); the
rocket row uses sizing/sizing_report.json's flight mass (CAD + the 10 g spring-hardware balance) when it is from this
params.json, else the rocket CAD report.  The power model is the tether report's (docs/tether_power_proposal.json
model_assumptions): electrical P = P_nom (T / T_ref)^1.5 with P_nom = edf.power_nominal_W; P1 ("proxy efficiency")
T_ref = edf.thrust_nominal_N, P2 ("proxy full power", pessimistic) T_ref = the table's T_max.  Battery options
(battery_options_ASSUMED) are ESTIMATES: hover current, pack sag (V = OCV - I R, OCV 6 x 3.7 V), C rate and flight time on
the usable fraction.  Nothing here changes params.

Outputs: sizing/sizing_report_vtol[_tether|_hybrid].json / .md (one per power source) and sizing/tw_table_v09.json / .md
(all four configurations).

Usage: python sizing/vtol_sizing.py [--power battery|tether|hybrid|all]
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
from typing import Any

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
for _p in (HERE, os.path.join(ROOT, "cad")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import layout as layout_mod  # noqa: E402
from layout import PARAMS_PATH, Layout, load_params  # noqa: E402

G = 9.80665
THRUST_TABLE_KGF = (2.0, 2.3, 2.6, 3.09)     # user decision 4 (2026-10-27)
TW_TARGET = 1.30                             # user decision 4: target T/W >= 1.3
OCV_PER_CELL = 3.7                           # mid-charge open-circuit voltage for the sag estimate (ASSUMED)
KR_REGISTRATION_KG = 2.0
KR_CLASSES = [(0.25, "조종자 증명 불필요 (250 g 이하)"), (2.0, "4종 (250 g 초과 ~ 2 kg 이하)"),
              (7.0, "3종 (2 kg 초과 ~ 7 kg 이하)"), (25.0, "2종"), (math.inf, "1종")]


def sha256(path: str) -> str:
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def report_path(config: str, power: str = "battery") -> str:
    return os.path.join(ROOT, "cad", "exports", f"cadquery_report{layout_mod.config_suffix(config, power)}.json")


def expected_parts(L: Layout) -> list[str]:
    """Every component a VTOL CAD report must carry (rocket_geometry._build_all_vtol's list, from Layout)."""
    n = L.vane_count
    out = ["propulsion_module", "intake_module", "avionics_module", "avionics_tray", "top_cap"]
    if getattr(L, "has_livery", False):        # livery 2026-11-04: the orange inlay part of the avionics module
        out.append(L.livery_inlay_part)
    out += [f"vane_{i + 1}" for i in range(n)] + [f"vane_coupler_{i + 1}" for i in range(n)]
    out += [f"servo_cover_{i + 1}" for i in range(n)] + ["wire_cover", "edf_unit", "esc", "misc_electronics", L.fcu_part,
                                                         "gps", "gps_ground_plane"]
    if L.has_battery and not getattr(L, "buffer_z", None):
        out.append("battery")
    if getattr(L, "buffer_z", None):
        out.append("buffer_pack")
    out += [f"servo_vane_{i + 1}" for i in range(n)] + [f"servo_horn_{i + 1}" for i in range(n)]
    if getattr(L, "dd_bushing", False):        # v0.9 fix round
        out += [f"hinge_bushing_{i + 1}" for i in range(n)]
    out += list(getattr(L, "tether_boxes", {}))
    if L.power_source in ("tether", "hybrid") and getattr(L, "cap_hardpoint", False):
        out += ["tether_eyebolt", "tether_load_washer", "tether_nut"]
    out += ["joint_screws_inserts", "vane_drive_hardware", "servo_leads", "motor_leads", "gps_lead"] + list(L.tether_items)
    return out


def load_report(path: str, L: Layout | None, sha: str, warnings: list[str]) -> dict:
    """A CAD report that is evidence for this params.json: refused when missing, from another params.json, with failed
    components or (L given) missing a part of ``expected_parts``."""
    if not os.path.exists(path):
        raise FileNotFoundError(f"{path} missing - run cad/cadquery/build_rocket.py first")
    with open(path, "r", encoding="utf-8") as f:
        rep = json.load(f)
    if rep.get("params_sha256") != sha:
        raise ValueError(f"{os.path.basename(path)} is from another params.json ({str(rep.get('params_sha256'))[:12]}...) - "
                         "rebuild the twin")
    if rep.get("failed_components"):
        raise ValueError(f"{os.path.basename(path)} lists failed components {rep['failed_components']}")
    if L is not None:
        miss = [n for n in expected_parts(L) if n not in rep["parts"]]
        if miss:
            raise ValueError(f"{os.path.basename(path)} is missing {miss}")
        extra = [n for n in rep["parts"] if n not in expected_parts(L)]
        if extra:
            warnings.append(f"{os.path.basename(path)} carries parts outside the VTOL list: {extra}")
    return rep


def power_w(thrust_n: float, t_ref_n: float, p_nom_w: float) -> float:
    """Tether-report power model: P = P_nom (T / T_ref)^1.5."""
    return p_nom_w * (max(thrust_n, 0.0) / t_ref_n) ** 1.5


def pack_current(p_w: float, cells: int, r_ohm: float) -> tuple[float, float]:
    """(current A, loaded pack voltage V) for an electrical load P with V = OCV - I R (no solution -> (nan, nan))."""
    ocv = cells * OCV_PER_CELL
    disc = ocv * ocv - 4.0 * p_w * r_ohm
    if disc < 0:
        return math.nan, math.nan
    v = 0.5 * (ocv + math.sqrt(disc))
    return p_w / v, v


def tw_rows(mass_g: float, n_exp: float) -> list[dict[str, Any]]:
    w = mass_g * 1e-3 * G
    rows = []
    for t_kgf in THRUST_TABLE_KGF:
        t = t_kgf * G
        tw = t / w
        rows.append({"thrust_kgf": t_kgf, "thrust_N": t, "tw": tw, "tw_ok_1p3": tw >= TW_TARGET,
                     "u_hover": (w / t) ** (1.0 / n_exp) if tw >= 1.0 else None, "hover_possible": tw >= 1.0,
                     "mass_for_tw_1p3_g": t / (TW_TARGET * G) * 1e3})
    return rows


def vane_authority(p: dict, L: Layout, thrust_n: float, l_v_m: float, ixx: float, izz: float) -> dict[str, float]:
    """SPEC 8.4 at a given thrust: q_e = T / (2 A_e), F = q_e S_v C_L(delta_cmd); M_pitch = 2 l_v F, M_roll = 4 r_v F."""
    e, v = p["edf"], p["vane"]
    d = L.duct_id * 1e-3
    a_e = math.pi / 4 * (d * d - (e["exit_hub_fraction"] * d) ** 2)
    q_e = thrust_n / (2.0 * a_e)
    d_cmd = v["max_deflection_cmd_deg"]
    f = q_e * L.vane_area_mm2 * 1e-6 * v["cl_alpha_per_rad"] * math.radians(min(d_cmd, v["stall_deg"]))
    m_p = 2.0 * l_v_m * f
    m_r = 4.0 * v["force_centroid_r"] * 1e-3 * f
    return {"thrust_N": thrust_n, "q_e_Pa": q_e, "force_per_vane_N": f, "lever_arm_mm": l_v_m * 1e3,
            "M_pitch_Nm": m_p, "M_roll_Nm": m_r, "I_xx_kg_m2": ixx, "I_zz_kg_m2": izz,
            "alpha_pitch_rad_s2": m_p / ixx, "alpha_roll_rad_s2": m_r / izz}


def tip_over(com_z_mm: float, fin_tip_r_mm: float) -> dict[str, float]:
    """Static tip-over angles of a vehicle standing on its fin trailing edges (z = 0): toward a fin tip and between two
    fins (support square with the four tips on the axes: inradius r cos 45)."""
    return {"cg_height_mm": com_z_mm, "fin_tip_r_mm": fin_tip_r_mm,
            "angle_toward_a_fin_deg": math.degrees(math.atan2(fin_tip_r_mm, com_z_mm)),
            "angle_between_fins_deg": math.degrees(math.atan2(fin_tip_r_mm * math.cos(math.radians(45.0)), com_z_mm))}


def lead_drag_entry(L: Layout, mass_g: float) -> dict[str, Any] | None:
    """Review 2026-10-29 (FIND-1; photo round: the heat-shrink bundle): the motor leads cross the jet (Layout.ml_jet_drag,
    E): D = Cd q A_f with q = T / (2 A_e)
    (the vane_authority model), so D / T = k = Cd A_f / (2 A_e) and T/W 1.3 needs T = 1.3 W / (1 - k) - the thrust
    derate this table adds next to the mass-only figure.  None when the configuration has no leads in the jet.  Fix round
    (review 2026-10-29 FIND-M4): the factory routing already has the leads in the jet, so a bench thrust of this EDF as
    built includes the drag - the derate applies only to thrust that does not come from such a test (the proxy table)."""
    jd = getattr(L, "ml_jet_drag", None)
    if not jd:
        return None
    e = L.p["edf"]
    d = L.duct_id * 1e-3
    a_e = math.pi / 4 * (d * d - (e["exit_hub_fraction"] * d) ** 2)

    def k_(cd: float) -> float:
        return cd * jd["area_mm2"] * 1e-6 / (2.0 * a_e)

    k = k_(jd["cd_ASSUMED"])
    t13 = TW_TARGET * mass_g * 1e-3
    return {"area_mm2": jd["area_mm2"], "cd_ASSUMED": jd["cd_ASSUMED"], "cd_band_ASSUMED": list(jd["cd_band_ASSUMED"]),
            "drag_over_thrust": k, "drag_over_thrust_band": [k_(c) for c in jd["cd_band_ASSUMED"]],
            "hover_drag_N": k / (1.0 - k) * mass_g * 1e-3 * G, "thrust_for_tw_1p3_kgf": t13 / (1.0 - k),
            "thrust_for_tw_1p3_kgf_band": [t13 / (1.0 - k_(c)) for c in jd["cd_band_ASSUMED"]],
            "label": "E (Cd ASSUMED, drag area from Layout.ml_jet_drag)"}


def config_entry(name: str, rep: dict, L: Layout, p: dict, mass_g: float, source: str) -> dict[str, Any]:
    t = rep["totals"]
    com = t["com_mm"]
    I = t["inertia_about_com_kg_m2"]
    e = p["edf"]
    l_v = (com[2] - L.vane_hinge_z) * 1e-3
    w = mass_g * 1e-3 * G
    rows = tw_rows(mass_g, e["throttle_exponent"])
    for r in rows:
        for tag, t_ref in (("P1", e["thrust_nominal_N"]), ("P2", r["thrust_N"])):
            ph = power_w(w, t_ref, e["power_nominal_W"])
            pk = power_w(min(TW_TARGET * w, r["thrust_N"]), t_ref, e["power_nominal_W"])
            r[f"hover_power_W_{tag}"] = ph if r["hover_possible"] else None
            r[f"hover_current_A_{tag}"] = (ph / e["voltage_nominal_V"]) if r["hover_possible"] else None
            r[f"tw1p3_power_W_{tag}"] = pk if r["tw_ok_1p3"] else None
    auth_hover = vane_authority(p, L, w, l_v, I[0][0], I[2][2])
    auth_full = vane_authority(p, L, e["thrust_nominal_N"], l_v, I[0][0], I[2][2])
    lead_drag = lead_drag_entry(L, mass_g)
    return {"config": name, "source": source, "mass_g": mass_g, "cad_mass_g": t["mass_g"], "weight_N": w,
            "lead_drag": lead_drag,
            "com_mm": com, "inertia_diag_kg_m2": [I[0][0], I[1][1], I[2][2]], "printed_mass_g": t["printed_mass_g"],
            "vane_lever_arm_mm": l_v * 1e3, "vehicle_h_mm": L.vehicle_h, "footprint_d_mm": 2 * L.fin_tip_r,
            "tip_over": tip_over(com[2], L.fin_tip_r), "tw_table": rows,
            "vane_authority_at_hover": auth_hover, "vane_authority_at_thrust_nominal": auth_full}


def battery_options(p: dict, m_vtol_battery_g: float) -> list[dict[str, Any]]:
    """Each battery_options_ASSUMED pack in the VTOL battery configuration: mass, T/W table, hover / T/W-1.3 current at
    P1 and at P2 2.0 kgf, pack sag, C rate and flight time (usable_fraction of the capacity)."""
    bo, e = p["battery_options_ASSUMED"], p["edf"]
    base = p["battery"]["mass_g"]
    out = []
    for o in bo["options"]:
        m = m_vtol_battery_g - base + o["mass_g"]
        w = m * 1e-3 * G
        cap_ah = o["capacity_mAh"] / 1000.0
        r = o["r_pack_mohm_ASSUMED"] * 1e-3
        cases = {}
        for tag, t_ref in (("P1 (proxy efficiency)", e["thrust_nominal_N"]), ("P2 at 2.0 kgf (pessimistic)", 2.0 * G)):
            ph = power_w(w, t_ref, e["power_nominal_W"])
            # the T/W-1.3 point, or full throttle when the fan cannot make 1.3 W (P2 at 2.0 kgf: T_max = T_ref)
            pk = power_w(min(TW_TARGET * w, t_ref), t_ref, e["power_nominal_W"])
            ih, vh = pack_current(ph, p["battery"]["cells"], r)
            ik, vk = pack_current(pk, p["battery"]["cells"], r)
            cases[tag] = {"hover_power_W": ph, "hover_current_A": ih, "hover_v_pack": vh, "hover_sag_V": 6 * OCV_PER_CELL - vh,
                          "hover_c_rate": ih / cap_ah, "flight_time_hover_s": bo["usable_fraction"] * cap_ah / ih * 3600.0,
                          "tw1p3_current_A": ik, "tw1p3_v_pack": vk, "tw1p3_sag_V": 6 * OCV_PER_CELL - vk,
                          "tw1p3_c_rate": ik / cap_ah, "c_rating": o["c_rating"],
                          "c_ok": ik / cap_ah <= o["c_rating"]}
        out.append({"name": o["name"], "pack_g": o["mass_g"], "vehicle_mass_g": m, "delta_g": o["mass_g"] - base,
                    "capacity_mAh": o["capacity_mAh"], "r_pack_mohm_ASSUMED": o["r_pack_mohm_ASSUMED"],
                    "tw_table": tw_rows(m, e["throttle_exponent"]), "cases": cases,
                    "meets_1650": m <= 1650.0 + 1e-9, "meets_1550": m <= 1550.0 + 1e-9})
    return out


def thrust_voltage_exponent(e: dict) -> float:
    """T ~ V^k from the proxy table's two points (thrust_nominal_N at voltage_nominal_V, thrust_peak_N at 4.2 V/cell)."""
    v_peak = e["cells"] * 4.2
    return math.log(e["thrust_peak_N"] / e["thrust_nominal_N"]) / math.log(v_peak / e["voltage_nominal_V"])


def loaded_pack_tw(p: dict, mass_g: float) -> dict[str, Any]:
    """v0.9 fix round (mass reviewer: 'T_max is referenced to 22.2 V at the ESC; the table has no voltage condition'):
    T/W at full throttle from a pack at OCV_PER_CELL (end of a flight) under the nominal full-throttle current and the
    default pack's ASSUMED resistance: T_max(V) = T_max(22.2 V) (V / 22.2)^k."""
    e = p["edf"]
    k = thrust_voltage_exponent(e)
    r = next((o["r_pack_mohm_ASSUMED"] for o in p["battery_options_ASSUMED"]["options"]
              if abs(o["mass_g"] - p["battery"]["mass_g"]) < 1e-6), 18.0) * 1e-3
    v = e["cells"] * OCV_PER_CELL - e["current_nominal_A"] * r
    f = (v / e["voltage_nominal_V"]) ** k
    w = mass_g * 1e-3 * G
    return {"v_loaded": v, "r_pack_ohm_ASSUMED": r, "exponent": k, "thrust_factor": f,
            "tw": {f"{t:g}": t * G * f / w for t in THRUST_TABLE_KGF},
            "tmax_22v2_for_tw_1p3_kgf": TW_TARGET * mass_g * 1e-3 / f}


def tether_supply(p: dict, mass_g: float, source: str) -> dict[str, Any]:
    """v0.9 fix round (mass reviewer: 'tether T/W table ignores the supply limit', 'hybrid: tether alone carries the hover
    only under P1'): one power.supply (v_set, rated_A) on the recommended route (route_loop_mohm + connector_mohm_ASSUMED):
    at the supply's current limit V_esc = v_set - I R and P = V_esc I; the T/W available = min(T_max at V_esc, thrust at P
    (P1 / P2)) / W.  Hover on the tether: I from P_hover = I (v_set - I R).  Hybrid: the buffer (storage voltage) takes
    over above the crossover I_x = (v_set - cells x storage_v) / R - its share and endurance (usable_fraction) at hover."""
    e, ps = p["edf"], p["power"]["supply"]
    r = (ps["route_loop_mohm"] + ps.get("connector_mohm_ASSUMED", 0.0)) * 1e-3
    vs, imax = ps["v_set"], ps["rated_A"]
    v_lim = vs - imax * r
    p_lim = v_lim * imax
    k = thrust_voltage_exponent(e)
    w = mass_g * 1e-3 * G
    rows = []
    for t_kgf in THRUST_TABLE_KGF:
        t_max = t_kgf * G
        t_v = t_max * (v_lim / e["voltage_nominal_V"]) ** k
        row = {"thrust_kgf": t_kgf}
        for tag, t_ref in (("P1", e["thrust_nominal_N"]), ("P2", t_max)):
            t_p = t_ref * (p_lim / e["power_nominal_W"]) ** (2.0 / 3.0)
            row[f"tw_supply_{tag}"] = min(t_v, t_p) / w
            ph = power_w(w, t_ref, e["power_nominal_W"]) if t_max >= w else math.nan
            disc = vs * vs - 4.0 * r * ph if ph == ph else -1.0
            i_h = (vs - math.sqrt(disc)) / (2.0 * r) if disc >= 0 else math.nan
            row[f"hover_W_{tag}"] = ph
            row[f"hover_tether_A_{tag}"] = i_h
            row[f"hover_v_esc_{tag}"] = vs - i_h * r if i_h == i_h else math.nan
        rows.append(row)
    out = {"supply": ps.get("model"), "v_set": vs, "rated_A": imax, "r_loop_ohm": r, "v_esc_at_limit": v_lim,
           "p_at_limit_W": p_lim, "rows": rows}
    if source == "hybrid":
        bp = p["power"]["hybrid_extra_ASSUMED"]["buffer_pack"]
        v_b = e["cells"] * bp["storage_v_per_cell"]
        i_x = (vs - v_b) / r
        cap = bp["capacity_mAh"] / 1000.0
        uf = p["battery_options_ASSUMED"]["usable_fraction"]
        for row in rows:
            for tag in ("P1", "P2"):
                ph, i_h = row[f"hover_W_{tag}"], row[f"hover_tether_A_{tag}"]
                if not (ph == ph) or i_h != i_h or i_h <= i_x:
                    row[f"buffer_A_{tag}"], row[f"buffer_s_{tag}"] = 0.0, None
                else:
                    ib = ph / v_b - i_x
                    row[f"buffer_A_{tag}"], row[f"buffer_s_{tag}"] = ib, uf * cap / ib * 3600.0
        out.update({"buffer_v": v_b, "crossover_A": i_x,
                    "note": "hover is carried by the tether alone only while the tether current stays below the crossover; "
                            "above it the buffer discharges at buffer_A (endurance buffer_s)"})
    return out


def legal(mass_g: float) -> dict[str, Any]:
    kg = mass_g / 1000.0
    return {"takeoff_mass_kg": kg, "operator_certificate": next(lbl for b, lbl in KR_CLASSES if kg <= b),
            "aircraft_registration_required": kg > KR_REGISTRATION_KG}


def run(power: str = "battery", write: bool = True) -> dict[str, Any]:
    sha = sha256(PARAMS_PATH)
    warnings: list[str] = []
    p = load_params(PARAMS_PATH, None, "vtol", power)
    L = Layout(p)
    rep = load_report(report_path("vtol", L.power_source), L, sha, warnings)
    ent = config_entry(f"vtol_{L.power_source}", rep, L, p, rep["totals"]["mass_g"], os.path.basename(report_path("vtol", L.power_source)))
    vx = rep["extra"]["vtol"]
    out = {"generator": "vtol_sizing", "config": "vtol", "power_source": L.power_source, "preset": p["meta"]["preset"],
           "params_sha256": sha, "cad_report": os.path.relpath(report_path("vtol", L.power_source), ROOT).replace(os.sep, "/"),
           **ent, "targets_g": {"vtol_battery": 1650.0, "vtol_battery_stretch": 1550.0, "vtol_tether": 1400.0},
           "parts_g": {n: e["mass_g"] for n, e in rep["parts"].items()},
           "top_cap": vx["top_cap"], "tether_items_g": vx["tether_items_g"], "legal": legal(rep["totals"]["mass_g"]),
           "warnings": warnings}
    if L.power_source == "battery":
        out["battery_options"] = battery_options(p, rep["totals"]["mass_g"])
        out["esc_options_ASSUMED"] = p["battery_options_ASSUMED"]["esc_options_ASSUMED"]
        out["loaded_pack_tw"] = loaded_pack_tw(p, rep["totals"]["mass_g"])     # v0.9 fix round
    else:
        out["tether_supply"] = tether_supply(p, rep["totals"]["mass_g"], L.power_source)   # v0.9 fix round
    if write:
        sfx = layout_mod.config_suffix("vtol", L.power_source)
        jp, mp = os.path.join(HERE, f"sizing_report{sfx}.json"), os.path.join(HERE, f"sizing_report{sfx}.md")
        with open(jp, "w", encoding="utf-8", newline="\n") as f:
            json.dump(out, f, indent=2, ensure_ascii=False)
            f.write("\n")
        with open(mp, "w", encoding="utf-8", newline="\n") as f:
            f.write(render_md(out))
        out["_files"] = {"json": jp, "md": mp}
    return out


def rocket_entry(sha: str, warnings: list[str]) -> dict[str, Any]:
    """The rocket configuration's row: flight mass from sizing/sizing_report.json (CAD + 10 g hardware balance) when that
    report is from this params.json, else the rocket CAD report's mass (flagged)."""
    p = load_params(PARAMS_PATH, None, "rocket")
    L = Layout(p)
    rep = load_report(report_path("rocket"), None, sha, warnings)
    mass, src = rep["totals"]["mass_g"], "cadquery_report.json (CAD mass)"
    sp = os.path.join(HERE, "sizing_report.json")
    if os.path.exists(sp):
        with open(sp, "r", encoding="utf-8") as f:
            sr = json.load(f)
        if sr.get("params_sha256") == sha and sr.get("preset") == p["meta"]["preset"]:
            mass, src = sr["totals"]["mass_g"], "sizing_report.json (flight mass = CAD + 10 g hardware balance)"
        else:
            warnings.append("sizing/sizing_report.json is not from this params.json - rocket row uses the CAD mass")
    return config_entry("rocket", rep, L, p, mass, src)


def tw_table_all(write: bool = True) -> dict[str, Any]:
    """The user's four configurations side by side (decision 4)."""
    sha = sha256(PARAMS_PATH)
    warnings: list[str] = []
    rows = {}
    for pw in ("battery", "hybrid", "tether"):
        p = load_params(PARAMS_PATH, None, "vtol", pw)
        L = Layout(p)
        rep = load_report(report_path("vtol", pw), L, sha, warnings)
        rows[f"vtol_{pw}"] = config_entry(f"vtol_{pw}", rep, L, p, rep["totals"]["mass_g"],
                                          os.path.basename(report_path("vtol", pw)))
    rows["rocket"] = rocket_entry(sha, warnings)
    rb, rr = rows["vtol_battery"], rows["rocket"]
    cmp: dict[str, dict] = {}
    for k2 in ("vane_authority_at_hover", "vane_authority_at_thrust_nominal"):
        cmp[k2] = {k: {"rocket": rr[k2][k], "vtol_battery": rb[k2][k], "ratio_vtol_over_rocket": rb[k2][k] / rr[k2][k]}
                   for k in ("thrust_N", "lever_arm_mm", "M_pitch_Nm", "I_xx_kg_m2", "alpha_pitch_rad_s2", "M_roll_Nm",
                             "I_zz_kg_m2", "alpha_roll_rad_s2")}
    # v0.9 fix round: the supply-limited rows (tether / hybrid on one supply) and the end-of-pack voltage column (battery)
    pv = load_params(PARAMS_PATH, None, "vtol", "battery")
    supply = {k: tether_supply(load_params(PARAMS_PATH, None, "vtol", k.split("_")[1]), rows[k]["mass_g"], k.split("_")[1])
              for k in ("vtol_tether", "vtol_hybrid")}
    loaded = {k: loaded_pack_tw(pv, rows[k]["mass_g"]) for k in ("vtol_battery", "rocket")}
    out = {"generator": "vtol_sizing.tw_table_all", "params_sha256": sha, "thrust_table_kgf": list(THRUST_TABLE_KGF),
           "tw_target": TW_TARGET, "power_model": "P = P_nom (T/T_ref)^1.5; P1 T_ref = edf.thrust_nominal_N, P2 T_ref = T_max",
           "voltage_condition": "T_max at 22.2 V at the ESC; loaded_pack = full throttle at 3.7 V/cell OCV",
           "configs": rows, "supply_limited": supply, "loaded_pack": loaded,
           "vane_authority_vtol_vs_rocket": cmp, "warnings": warnings}
    if write:
        with open(os.path.join(HERE, "tw_table_v09.json"), "w", encoding="utf-8", newline="\n") as f:
            json.dump(out, f, indent=2, ensure_ascii=False)
            f.write("\n")
        with open(os.path.join(HERE, "tw_table_v09.md"), "w", encoding="utf-8", newline="\n") as f:
            f.write(render_tw_md(out))
    return out


def _f(x: Any, nd: int = 2) -> str:
    if x is None or (isinstance(x, float) and math.isnan(x)):
        return "-"
    return f"{x:.{nd}f}"


def render_tw_md(o: dict) -> str:
    L = ["# v0.9 추력/무게 비와 호버 스로틀 (4개 구성)", "",
         f"params.json sha256 `{o['params_sha256'][:16]}…`. 추력은 모릅니다(판매처 3.09 kgf 미검증). 표는 추력 가정별입니다.",
         "호버 스로틀 u = (W/T_max)^(1/2). 전력 P1 = 프록시 효율, P2 = 비관적 모델(T_max에서 2,264 W).", "",
         "| 구성 | 질량 (g) | 2.0 kgf | 2.3 kgf | 2.6 kgf | 3.09 kgf | T/W 1.3이 되는 추력 | 모터선 항력 포함 (E) |",
         "|---|---|---|---|---|---|---|---|"]
    names = {"vtol_battery": "VTOL 배터리", "vtol_hybrid": "VTOL 하이브리드", "vtol_tether": "VTOL 유선(배터리 없음)",
             "rocket": "로켓"}
    for k, r in o["configs"].items():
        cells = [f"T/W {_f(x['tw'])}, u {_f(x['u_hover'])}" for x in r["tw_table"]]
        ld = r.get("lead_drag")
        ld_txt = (f"{ld['thrust_for_tw_1p3_kgf']:.2f} kgf (Cd {ld['cd_band_ASSUMED'][0]:g}-{ld['cd_band_ASSUMED'][1]:g}: "
                  f"{ld['thrust_for_tw_1p3_kgf_band'][0]:.2f}-{ld['thrust_for_tw_1p3_kgf_band'][1]:.2f})") if ld else "-"
        L.append(f"| {names.get(k, k)} | {r['mass_g']:.1f} | " + " | ".join(cells) +
                 f" | {r['mass_g'] * 1e-3 * 1.3:.2f} kgf | {ld_txt} |")
    L += ["", "모터선 항력(E): 모터선 묶음(수축 튜브로 묶인 3가닥, 사진 기준)이 배기 바람을 가로지르며 받는 힘 D = Cd q A "
              "(Cd 1.0, 0.8-1.2 ASSUMED, A = Layout.ml_jet_drag). 추력의 약 " + " / ".join(
                  f"{100 * r['lead_drag']['drag_over_thrust']:.1f} %" for r in o["configs"].values() if r.get("lead_drag"))
          + "를 먹습니다. T/W 칸은 질량만 반영한 값입니다. 실측이 필요합니다. 이 EDF는 공장에서부터 선이 바람 속에 "
            "있으므로, 이 EDF를 조립한 그대로 시험대에서 잰 추력에는 이 항력이 이미 들어 있습니다. 그 값을 쓸 때는 "
            "항력을 다시 빼지 않습니다(두 번 빼는 셈). 항력 보정은 다른 EDF의 표(프록시) 추력에만 씁니다."]
    # v0.9 fix round: voltage condition and the supply limit
    L += ["", "추력은 ESC 22.2 V 기준입니다. 팩이 셀당 3.7 V까지 내려가면(전 스로틀 전압 처짐 포함) 추력이 줄어듭니다:", "",
          "| 구성 | 2.0 kgf | 2.3 kgf | 2.6 kgf | 3.09 kgf | 끝까지 T/W 1.3에 필요한 22.2 V 추력 |", "|---|---|---|---|---|---|"]
    for k, lp in o.get("loaded_pack", {}).items():
        L.append(f"| {names.get(k, k)} (셀당 3.7 V, {lp['v_loaded']:.1f} V) | " + " | ".join(f"T/W {v:.2f}" for v in lp["tw"].values())
                 + f" | {lp['tmax_22v2_for_tw_1p3_kgf']:.2f} kgf |")
    L += ["", "유선·하이브리드는 전원 장치 1대(Mean Well RSP-1500-24, 25.2 V / 60 A, 8.5 m 10 AWG)의 한계가 있습니다. "
              "전원 한계 T/W (P1 / P2, 질량만 반영 - 모터선 항력 미포함):", "", "| 구성 | 2.0 kgf | 2.3 kgf | 2.6 kgf | 3.09 kgf |", "|---|---|---|---|---|"]
    for k, ts in o.get("supply_limited", {}).items():
        L.append(f"| {names.get(k, k)} (전원 1대) | " + " | ".join(f"{_f(x['tw_supply_P1'])} / {_f(x['tw_supply_P2'])}"
                                                                for x in ts["rows"]) + " |")
    k_txt = " / ".join(f"{100 * ((1.0 / (1.0 - r['lead_drag']['drag_over_thrust'])) ** 1.5 - 1.0):.1f} %"
                       for r in o["configs"].values() if r.get("lead_drag"))
    L += ["", "## 호버 전력·전류 (P1 / P2, 22.2 V 기준)", "",
          "질량만 반영한 값입니다(모터선 항력 미포함). 항력 D/T = k 이면 호버 추력이 1/(1-k)배, 전력은 약 (1/(1-k))^1.5 배가 됩니다: "
          + (k_txt or "-") + " 더 (E). 시뮬레이션도 아직 이 항력을 넣지 않았습니다.", "",
          "| 구성 | 추력 | P1 W | P1 A | P2 W | P2 A |", "|---|---|---|---|---|---|"]
    for k, r in o["configs"].items():
        for x in r["tw_table"]:
            L.append(f"| {names.get(k, k)} | {x['thrust_kgf']} kgf | {_f(x['hover_power_W_P1'], 0)} | {_f(x['hover_current_A_P1'], 1)} | "
                     f"{_f(x['hover_power_W_P2'], 0)} | {_f(x['hover_current_A_P2'], 1)} |")
    L += ["", "## 베인 권한: 로켓 → VTOL 배터리", "", "| 항목 | 호버 추력에서 로켓 | VTOL | 비 | 공칭 추력에서 로켓 | VTOL | 비 |",
          "|---|---|---|---|---|---|---|"]
    a, b = o["vane_authority_vtol_vs_rocket"]["vane_authority_at_hover"], o["vane_authority_vtol_vs_rocket"]["vane_authority_at_thrust_nominal"]
    for k in a:
        L.append(f"| {k} | {a[k]['rocket']:.4g} | {a[k]['vtol_battery']:.4g} | {a[k]['ratio_vtol_over_rocket']:.3f} | "
                 f"{b[k]['rocket']:.4g} | {b[k]['vtol_battery']:.4g} | {b[k]['ratio_vtol_over_rocket']:.3f} |")
    L += ["", "## 무게중심·넘어짐", "", "| 구성 | CG z (mm) | 베인 팔 (mm) | Ixx (kg m²) | 넘어짐 각 (핀 쪽 / 핀 사이) |", "|---|---|---|---|---|"]
    for k, r in o["configs"].items():
        t = r["tip_over"]
        L.append(f"| {names.get(k, k)} | {r['com_mm'][2]:.1f} | {r['vane_lever_arm_mm']:.1f} | {r['inertia_diag_kg_m2'][0]:.4f} | "
                 f"{t['angle_toward_a_fin_deg']:.1f}° / {t['angle_between_fins_deg']:.1f}° |")
    if o["warnings"]:
        L += ["", "경고: " + "; ".join(o["warnings"])]
    return "\n".join(L) + "\n"


def render_md(o: dict) -> str:
    L = [f"# VTOL 크기 산정 - 전원 {o['power_source']}", "",
         f"CAD 보고서 `{o['cad_report']}` (params sha256 `{o['params_sha256'][:16]}…`).", "",
         f"* 질량 {o['mass_g']:.1f} g (출력 부품 {o['printed_mass_g']:.1f} g), CG z {o['com_mm'][2]:.1f} mm, 베인 팔 "
         f"{o['vane_lever_arm_mm']:.1f} mm, Ixx {o['inertia_diag_kg_m2'][0]:.4f} kg m², 높이 {o['vehicle_h_mm']:.1f} mm, 발자국 "
         f"Ø{o['footprint_d_mm']:.1f} mm",
         f"* 넘어짐 각: 핀 쪽 {o['tip_over']['angle_toward_a_fin_deg']:.1f}°, 핀 사이 {o['tip_over']['angle_between_fins_deg']:.1f}°",
         f"* 법규: {o['legal']['operator_certificate']}, 기체 신고 필요 {o['legal']['aircraft_registration_required']}", "",
         "| 추력 (kgf) | T/W | 호버 스로틀 | P1 호버 W | P2 호버 W | T/W 1.3이 되는 질량 (g) |", "|---|---|---|---|---|---|"]
    for x in o["tw_table"]:
        L.append(f"| {x['thrust_kgf']} | {_f(x['tw'])} | {_f(x['u_hover'])} | {_f(x['hover_power_W_P1'], 0)} | "
                 f"{_f(x['hover_power_W_P2'], 0)} | {x['mass_for_tw_1p3_g']:.0f} |")
    a = o["vane_authority_at_hover"]
    L += ["", f"베인 권한(호버 추력 {a['thrust_N']:.1f} N, 명령 한계): 피치 {a['M_pitch_Nm']:.3f} N m → {a['alpha_pitch_rad_s2']:.1f} rad/s², "
              f"롤 {a['M_roll_Nm']:.3f} N m → {a['alpha_roll_rad_s2']:.1f} rad/s²"]
    if o.get("lead_drag"):          # review 2026-10-29 (FIND-1)
        ld = o["lead_drag"]
        L += ["", f"모터선 항력(E): 배기 속 모터선 묶음(수축 튜브, 항력 면적 {ld['area_mm2']:.0f} mm², Cd {ld['cd_ASSUMED']:g} ASSUMED)이 추력의 "
                  f"{100 * ld['drag_over_thrust']:.1f} %(Cd {ld['cd_band_ASSUMED'][0]:g}-{ld['cd_band_ASSUMED'][1]:g}: "
                  f"{100 * ld['drag_over_thrust_band'][0]:.1f}-{100 * ld['drag_over_thrust_band'][1]:.1f} %)를 먹습니다. 호버에서 "
                  f"{ld['hover_drag_N']:.2f} N. 그래서 T/W 1.3에 필요한 추력은 {ld['thrust_for_tw_1p3_kgf']:.2f} kgf입니다"
                  f"(질량만 보면 {o['mass_g'] * 1e-3 * TW_TARGET:.2f} kgf). 위 표(T/W, 호버 전력)는 질량만 반영합니다. "
                  f"이 EDF를 조립한 그대로 시험대에서 잰 추력에는 이 항력이 이미 들어 있으므로, 그 값에는 보정을 다시 하지 않습니다."]
    if o.get("loaded_pack_tw"):
        lp = o["loaded_pack_tw"]
        L += ["", f"전압 조건: 위 표의 추력은 ESC 22.2 V 기준입니다. 팩이 셀당 {OCV_PER_CELL} V일 때 전 스로틀 전압은 "
                  f"{lp['v_loaded']:.2f} V(내부 저항 {lp['r_pack_ohm_ASSUMED'] * 1e3:.0f} mΩ ASSUMED)이고 추력은 "
                  f"{lp['thrust_factor']:.3f}배(T ∝ V^{lp['exponent']:.2f})입니다. 그때 T/W: "
                  + " / ".join(f"{k} kgf {v:.2f}" for k, v in lp["tw"].items())
                  + f". 비행 끝까지 T/W 1.3을 지키려면 22.2 V 기준 추력 {lp['tmax_22v2_for_tw_1p3_kgf']:.2f} kgf 이상이 필요합니다."]
    if o.get("tether_supply"):
        ts = o["tether_supply"]
        L += ["", f"## 전원 장치 한계 ({ts['supply']}, {ts['v_set']:g} V / {ts['rated_A']:g} A, 루프 {ts['r_loop_ohm'] * 1e3:.1f} mΩ)", "",
              f"전류 한계에서 ESC 전압 {ts['v_esc_at_limit']:.2f} V, 전력 {ts['p_at_limit_W']:.0f} W. 전원 1대로 낼 수 있는 T/W:", "",
              "| 추력 (kgf) | T/W 전원 한계 P1 | T/W 전원 한계 P2 | 호버 전선 전류 P1 A (ESC V) | P2 A (ESC V) |"
              + (" 버퍼 전류 P1 / P2 A | 버퍼 지속 P1 / P2 s |" if "crossover_A" in ts else ""),
              "|---|---|---|---|---|" + ("---|---|" if "crossover_A" in ts else "")]
        for x in ts["rows"]:
            line = (f"| {x['thrust_kgf']} | {_f(x['tw_supply_P1'])} | {_f(x['tw_supply_P2'])} | {_f(x['hover_tether_A_P1'], 1)} "
                    f"({_f(x['hover_v_esc_P1'], 2)}) | {_f(x['hover_tether_A_P2'], 1)} ({_f(x['hover_v_esc_P2'], 2)}) |")
            if "crossover_A" in ts:
                line += (f" {_f(x.get('buffer_A_P1'), 1)} / {_f(x.get('buffer_A_P2'), 1)} | {_f(x.get('buffer_s_P1'), 0)} / "
                         f"{_f(x.get('buffer_s_P2'), 0)} |")
            L.append(line)
        if "crossover_A" in ts:
            L += ["", f"하이브리드: 전선 전류가 {ts['crossover_A']:.1f} A(버퍼 {ts['buffer_v']:.1f} V)를 넘으면 버퍼 배터리가 함께 "
                      "방전합니다. '호버는 전선이 혼자 감당한다'는 말은 팬 효율이 P1(판매처 표)처럼 좋을 때만 맞습니다."]
    if o.get("battery_options"):
        L += ["", "## 배터리 선택지 (추정, 기본 팩은 그대로)", "",
              "| 팩 | 기체 질량 g | T/W @2.0/2.3/2.6/3.09 | P1 호버 A / 전압 처짐 V / 비행 s | P2(2.0 kgf) 호버 A / 처짐 V / 비행 s | 최대 C (P1 T/W 1.3 / P2 전 스로틀) |",
              "|---|---|---|---|---|---|"]
        for b in o["battery_options"]:
            c1, c2 = b["cases"]["P1 (proxy efficiency)"], b["cases"]["P2 at 2.0 kgf (pessimistic)"]
            L.append(f"| {b['name']} | {b['vehicle_mass_g']:.0f} | " + "/".join(_f(x['tw']) for x in b["tw_table"]) +
                     f" | {_f(c1['hover_current_A'], 1)} / {_f(c1['hover_sag_V'], 2)} / {_f(c1['flight_time_hover_s'], 0)} | "
                     f"{_f(c2['hover_current_A'], 1)} / {_f(c2['hover_sag_V'], 2)} / {_f(c2['flight_time_hover_s'], 0)} | "
                     f"{_f(c1['tw1p3_c_rate'], 0)} / {_f(c2['tw1p3_c_rate'], 0)} (정격 {b['cases']['P1 (proxy efficiency)']['c_rating']}) |")
    if o["warnings"]:
        L += ["", "경고: " + "; ".join(o["warnings"])]
    return "\n".join(L) + "\n"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--power", default="all", help="battery | tether | hybrid | all (default all + the four-config table)")
    a = ap.parse_args(argv)
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")  # type: ignore[attr-defined]
    except Exception:
        pass
    pows = ("battery", "tether", "hybrid") if a.power == "all" else (a.power,)
    for pw in pows:
        r = run(pw)
        rows = ", ".join(f"{x['thrust_kgf']}: {x['tw']:.2f}/{_f(x['u_hover'])}" for x in r["tw_table"])
        print(f"vtol {r['power_source']}: {r['mass_g']:.1f} g, CG {r['com_mm'][2]:.1f} mm, l_v {r['vane_lever_arm_mm']:.1f} mm, "
              f"T/W/u {rows} -> {r['_files']['md']}")
    if a.power == "all":
        t = tw_table_all()
        print("four-config table -> sizing/tw_table_v09.md")
        for k, v in t["configs"].items():
            print(f"  {k:13s} {v['mass_g']:7.1f} g  " + "  ".join(f"{x['thrust_kgf']}: {x['tw']:.2f}" for x in v["tw_table"]))
        for w in t["warnings"]:
            print("  WARN:", w)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
