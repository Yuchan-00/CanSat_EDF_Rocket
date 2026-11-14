"""CanSat EDF Rocket - sizing tool (SPEC section 11).

Reads ``params.json`` (+ optional preset) through ``cad/layout.py`` and, when one
exists, a CAD mass-property report; computes the mass budget, thrust-to-weight,
hover throttle, exhaust velocity, vane / roll authority, climb energy and peak
current, battery C-rate, parachute size, Barrowman static margin and the Korean
regulatory class; writes ``sizing_report.md`` + ``sizing_report.json``.

Invariants: no dimension, mass or gain is typed here - everything is read from
``Layout`` / ``params`` (the few SPEC literals live as named constants in
``structure_estimate.py`` and below, each cited).  Structure masses come from the
CAD report when present and are otherwise labelled ESTIMATE.  Every verdict
(PASS / WARN / FAIL) uses the thresholds of SPEC section 7 (C12, C13).

Usage: ``python sizing/sizing.py [--preset NAME] [--cad-report PATH | --estimate-only] [--out-dir DIR]``
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
from dataclasses import dataclass, field
from typing import Any

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
for _p in (HERE, os.path.join(ROOT, "cad")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import barrowman  # noqa: E402
import structure_estimate as se  # noqa: E402
from layout import PARAMS_PATH, Layout, load_params  # noqa: E402

G = 9.80665  # m/s^2

# SPEC s7 check thresholds
C12_TW_PASS, C12_TW_WARN = 1.30, 1.15
C13_CG_ABOVE_HINGE_MM = 250.0
C13_MARGIN_PASS_CAL, C13_MARGIN_WARN_CAL = 1.0, 0.5
# SPEC s6: post-ejection excludes these
EJECTED = ("cansat_payload", "nose_cone", "chute_pack")
# Korean regulation mass classes (kg): upper bound -> operator certificate class
KR_MASS_CLASSES = [(0.25, "해당 없음 (250 g 이하: 조종자 증명 불필요)"),
                   (2.0, "4종 (250 g 초과 ~ 2 kg 이하: 온라인 교육)"),
                   (7.0, "3종 (2 kg 초과 ~ 7 kg 이하: 필기시험)"),
                   (25.0, "2종 (7 kg 초과 ~ 25 kg 이하)"),
                   (math.inf, "1종 (25 kg 초과)")]
KR_REGISTRATION_KG = 2.0        # 기체신고: 최대이륙중량 2 kg 초과 (비사업용)
KR_APPROVAL_ALT_M = 150.0       # 비행승인: 고도 150 m 초과

GROUPS = {
    "edf_unit": "propulsion", "battery": "power", "esc": "avionics", "nav": "avionics", "fcu": "avionics",
    "misc_electronics": "avionics", "com": "avionics", "radio_log": "avionics", "cansat_payload": "payload",
    "chute_pack": "recovery",
    "spring": "ejection", "joint_screws_inserts": "hardware", "pushrods_horns_clevises": "hardware",
    "vane_drive_hardware": "hardware",                                           # v0.9 direct drive (cover / coupler screws)
    "motor_leads": "power",                                                      # v0.9 3 x 12 AWG + bullets (ASSUMED)
    "spring_piston_hardware_balance": "hardware",
    "gps": "avionics", "gps_ground_plane": "avionics", "gps_lead": "avionics",   # v0.8 GPS pod contents
    "gns": "avionics",                                                           # v0.8.1 GNS board (ATmega128A #4)
    "carrier": "avionics",                                    # rocket-carrier round (2026-11-14): the carrier PCB rev 3.1
    "logic_pack": "power", "logic_switch": "avionics", "logic_switch_lead": "avionics", "logic_retention": "hardware",
}


def group_of(name: str, kind: str) -> str:
    if kind == "printed":
        return "structure"
    if name.startswith(("servo_", "pushrod_", "hinge_bushing_")):   # v0.9 fix round: the brass hinge bushings
        return "actuation"
    return GROUPS.get(name, "other")


# --------------------------------------------------------------------------- #
# budget
# --------------------------------------------------------------------------- #
@dataclass
class BudgetItem:
    name: str
    kind: str
    group: str
    mass_g: float
    com_mm: tuple[float, float, float]
    inertia_about_com_kg_m2: list[list[float]]
    source: str                    # "params" | "ESTIMATE" | "CAD:<generator>"
    note: str = ""
    mass_solid_g: float | None = None
    volume_mm3: float = 0.0
    density_g_cm3: float = 0.0


@dataclass
class MassProps:
    mass_g: float
    com_mm: tuple[float, float, float]
    inertia_about_com_kg_m2: list[list[float]]


def mass_props(items: list[BudgetItem]) -> MassProps:
    m = sum(i.mass_g for i in items)
    com = tuple(sum(i.mass_g * i.com_mm[k] for i in items) / m for k in range(3))
    I = se.inertia_about([(i.mass_g, i.com_mm, i.inertia_about_com_kg_m2) for i in items], com)  # type: ignore[arg-type]
    return MassProps(m, com, I)  # type: ignore[arg-type]


def params_sha256(path: str = PARAMS_PATH) -> str:
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def missing_report_parts(rep: dict, expected: list[str] | tuple[str, ...] | set[str]) -> list[str]:
    """Expected components the report does not carry, neither by name nor (vane_3 -> vane) by the base name that
    ``_cad_part`` accepts.  A report without a ``parts`` dict misses all of them."""
    parts = rep.get("parts")
    if not isinstance(parts, dict):
        return sorted(expected)
    return sorted(n for n in expected if _cad_part(rep, n)[0] is None)


def find_cad_report(explicit: str | None, preset: str, sha: str, warnings: list[str],
                    default_preset: str | None = None, expected: list[str] | None = None) -> dict | None:
    """SPEC s6 report: explicit path, else per generator (cadquery, then fusion360) the preset-suffixed
    ``<generator>_report_<preset>.json`` (what build_rocket.py writes for an alt preset) and the plain
    ``<generator>_report.json``.  A report for another preset is ignored (unless explicit), one from another
    params.json is used but flagged.  A report that lists failed components or is missing a component of
    ``expected`` (default: ``structure_estimate.expected_report_parts`` of this preset; ``launch_pad`` optional)
    is no evidence: ignored, or ValueError when it was given explicitly.  sim/vehicle.py ``load_vehicle`` applies
    the same rules (with exact names only: it composes the report's parts itself)."""
    if expected is None:
        p_exp = load_params(PARAMS_PATH, preset)
        expected = se.expected_report_parts(Layout(p_exp), p_exp)
    suffix = "" if default_preset is None or preset == default_preset else f"_{preset}"
    candidates = [explicit] if explicit else [os.path.join(ROOT, "cad", "exports", f"{gen}_report{s}.json")
                                              for gen in ("cadquery", "fusion360") for s in dict.fromkeys((suffix, ""))]
    for path in candidates:
        if not path or not os.path.exists(path):
            if explicit:
                raise FileNotFoundError(f"CAD report not found: {path}")
            continue
        with open(path, "r", encoding="utf-8") as f:
            rep = json.load(f)
        if rep.get("preset") != preset:
            msg = f"CAD report {os.path.basename(path)} is for preset {rep.get('preset')!r}, not {preset!r}"
            if explicit:
                warnings.append(msg + " - used anyway because it was given explicitly")
            else:
                warnings.append(msg + " - ignored")
                continue
        if rep.get("failed_components"):
            # a generator that could not build a required feature still writes the partial part: no evidence
            msg = (f"CAD report {os.path.basename(path)} lists failed components {rep['failed_components']}")
            if explicit:
                raise ValueError(msg)
            warnings.append(msg + " - ignored")
            continue
        missing = missing_report_parts(rep, expected)
        if missing:
            # a report that silently dropped a part would put the budget together from CAD and estimate halves
            msg = (f"CAD report {os.path.basename(path)} is missing components {missing} "
                   f"(expected every SPEC s6 part; launch_pad optional)")
            if explicit:
                raise ValueError(msg)
            warnings.append(msg + " - ignored")
            continue
        if rep.get("params_sha256") != sha:
            warnings.append(f"CAD report {os.path.basename(path)} was generated from a different params.json "
                            f"(sha {str(rep.get('params_sha256'))[:12]}... vs {sha[:12]}...)")
        rep["_path"] = path
        return rep
    return None


def _cad_part(rep: dict, name: str) -> tuple[dict | None, bool]:
    """(part, exact) - exact name, else the un-suffixed base name (vane_3 -> vane)."""
    parts = rep.get("parts", {})
    if name in parts:
        return parts[name], True
    base = name.rsplit("_", 1)[0] if name[-1].isdigit() else name
    if base in parts:
        return parts[base], False
    return None, False


def build_budget(lay: Layout, p: dict, rep: dict | None, warnings: list[str]) -> tuple[list[BudgetItem], BudgetItem]:
    """Every SPEC s6 item + printed structure.  Returns (flight items, launch pad)."""
    est = se.StructureEstimator(lay, p)
    printed = est.estimate_all()
    placeholders = se.placeholder_parts(lay, p)
    items: list[BudgetItem] = []
    gen = f"CAD:{rep.get('generator', '?')}" if rep else None
    for name, pe in list(printed.items()) + list(placeholders.items()):
        src = "params" if pe.kind != "printed" else "ESTIMATE"
        mass, com, I, note, solid, vol, rho = pe.mass_g, pe.com_mm, pe.inertia_about_com_kg_m2, "; ".join(pe.notes), pe.mass_solid_g, pe.volume_mm3, pe.density_g_cm3
        if rep is not None:
            cp, exact = _cad_part(rep, name)
            if cp is not None and pe.kind != "lumped":
                src = gen or "CAD"
                mass = float(cp["mass_g"])
                vol = float(cp.get("volume_mm3", vol))
                rho = float(cp.get("density_g_cm3", rho))
                if exact and cp.get("com_mm"):
                    com = tuple(float(c) for c in cp["com_mm"])  # type: ignore[assignment]
                if exact and cp.get("inertia_about_com_kg_m2"):
                    I = [[float(x) for x in row] for row in cp["inertia_about_com_kg_m2"]]
                if pe.kind == "printed":
                    solid = mass
                    note = f"CAD report ({os.path.basename(rep['_path'])}); estimate was {pe.mass_g:.1f} g"
                if not exact:
                    note += " (CAD base-name match: mass from CAD, COM from estimate)"
        items.append(BudgetItem(name, pe.kind, group_of(name, pe.kind), mass, com, I, src, note, solid, vol, rho))  # type: ignore[arg-type]
    if rep is not None and "totals" in rep:
        cad_total = float(rep["totals"].get("mass_g", float("nan")))
        mine = sum(i.mass_g for i in items)
        if abs(cad_total - mine) > 1.0:
            warnings.append(f"CAD report total {cad_total:.1f} g differs from the budget sum {mine:.1f} g "
                            f"(items missing from one side, or the 10 g hardware balance)")
    pad_pe = est.launch_pad_estimate()
    pad = BudgetItem("launch_pad", "printed", "ground", pad_pe.mass_g, pad_pe.com_mm, pad_pe.inertia_about_com_kg_m2,
                     "ESTIMATE", "; ".join(pad_pe.notes), pad_pe.mass_solid_g, pad_pe.volume_mm3, pad_pe.density_g_cm3)
    return items, pad


# --------------------------------------------------------------------------- #
# physics
# --------------------------------------------------------------------------- #
def exhaust(p: dict, lay: Layout, thrust_N: float) -> dict[str, float]:
    """SPEC s8.3: V_e = sqrt(T / (rho A_e)), A_e = pi/4 (duct_id^2 - (f duct_id)^2)."""
    rho = p["aero"]["rho_kg_m3"]
    d = lay.duct_id * 1e-3
    a_e = math.pi / 4 * (d * d - (p["edf"]["exit_hub_fraction"] * d) ** 2)
    v_e = math.sqrt(max(thrust_N, 0.0) / (rho * a_e))
    return {"thrust_N": thrust_N, "A_e_m2": a_e, "V_e_mps": v_e, "q_e_Pa": 0.5 * rho * v_e * v_e,
            "mass_flow_kg_s": rho * a_e * v_e}


def vane_cl(p: dict, delta_deg: float) -> float:
    """C_L(delta) = cl_alpha * delta up to stall, flat beyond (SPEC s8.3)."""
    v = p["vane"]
    d = math.radians(min(abs(delta_deg), v["stall_deg"]))
    return v["cl_alpha_per_rad"] * d


def vane_authority(p: dict, lay: Layout, q_e: float, l_v_m: float, inertia: list[list[float]]) -> dict[str, float]:
    """SPEC s8.4 moments at max_deflection_cmd_deg; angular accelerations from the inertia estimate."""
    v = p["vane"]
    s_v = lay.vane_area_mm2 * 1e-6
    r_v = v["force_centroid_r"] * 1e-3
    d_cmd = v["max_deflection_cmd_deg"]
    f_vane = q_e * s_v * vane_cl(p, d_cmd)
    m_x = 2.0 * l_v_m * f_vane          # delta_x = d_cmd (vane 1 = +d, vane 3 = -d)
    m_z = 4.0 * r_v * f_vane            # delta_z = d_cmd on all four
    ixx, iyy, izz = inertia[0][0], inertia[1][1], inertia[2][2]
    return {"delta_cmd_deg": d_cmd, "S_v_m2": s_v, "force_per_vane_N": f_vane, "lever_arm_m": l_v_m,
            "M_pitch_Nm": m_x, "M_roll_Nm": m_z, "I_xx_kg_m2": ixx, "I_yy_kg_m2": iyy, "I_zz_kg_m2": izz,
            "alpha_pitch_rad_s2": m_x / ixx, "alpha_yaw_rad_s2": m_x / iyy, "alpha_roll_rad_s2": m_z / izz,
            "alpha_pitch_deg_s2": math.degrees(m_x / ixx), "alpha_roll_deg_s2": math.degrees(m_z / izz),
            "moment_per_deg_pitch_Nm": m_x / d_cmd, "moment_per_deg_roll_Nm": m_z / d_cmd}


def swirl_torque(p: dict) -> dict[str, float]:
    """tau_swirl = fraction * P / omega, omega from kv x V (unloaded rpm - a lower bound on tau)."""
    e = p["edf"]
    rpm = e["kv"] * e["voltage_nominal_V"]
    omega = rpm * 2 * math.pi / 60.0
    tau = e["residual_swirl_torque_fraction_ASSUMED"] * e["power_nominal_W"] / omega
    return {"rpm_unloaded": rpm, "omega_rad_s": omega, "P_over_omega_Nm": e["power_nominal_W"] / omega,
            "fraction_ASSUMED": e["residual_swirl_torque_fraction_ASSUMED"], "tau_swirl_Nm": tau}


def climb_1d(*, m_kg: float, t_max_N: float, n_exp: float, u_climb: float, ramp_s: float, tau_s: float,
             rho: float, cd: float, a_ref_m2: float, target_m: float, k_coast: float, p_nom_W: float,
             v_nom_V: float, batt_Wh: float, dt: float = 0.001, t_limit_s: float = 90.0) -> dict[str, Any]:
    """Vertical 1-D climb with the SPEC s8.7 thrust model and the s8.8 cut-off rule.

    LAUNCH: throttle ramps 0 -> u_climb in ramp_s; thrust lags first-order (tau_s).
    ASCENT -> COAST when h + k_coast v^2/(2g) >= target; COAST until v <= 0 (apogee).
    Drag 0.5 rho v^2 cd A_ref.  Electrical power P = p_nom u^3 (energy accounting).
    """
    h = v = t = thrust = e_j = 0.0
    phase = "LAUNCH"
    lag = 1.0 - math.exp(-dt / tau_s) if tau_s > 0 else 1.0
    out: dict[str, Any] = {"target_m": target_m, "u_climb": u_climb, "reached": False, "liftoff_t_s": None,
                           "t_cutoff_s": None, "h_cutoff_m": None, "v_cutoff_mps": None, "v_max_mps": 0.0,
                           "apogee_m": None, "t_apogee_s": None, "reason": "",
                           "thrust_climb_N": t_max_N * u_climb**n_exp, "weight_N": m_kg * G}
    out["tw_climb"] = out["thrust_climb_N"] / out["weight_N"]
    i_peak = 0.0
    while t < t_limit_s:
        if phase in ("LAUNCH", "ASCENT"):
            u = u_climb * (min(1.0, t / ramp_s) if ramp_s > 0 else 1.0)
            if phase == "LAUNCH" and t >= ramp_s:
                phase = "ASCENT"
            if h + k_coast * v * v / (2 * G) >= target_m:
                phase, u = "COAST", 0.0
                out.update(t_cutoff_s=t, h_cutoff_m=h, v_cutoff_mps=v)
        else:
            u = 0.0
        thrust += (t_max_N * u**n_exp - thrust) * lag
        drag = 0.5 * rho * v * abs(v) * cd * a_ref_m2
        acc = (thrust - m_kg * G - drag) / m_kg
        if h <= 0.0 and v <= 0.0 and acc <= 0.0:
            acc, v, h = 0.0, 0.0, 0.0
        elif out["liftoff_t_s"] is None:
            out["liftoff_t_s"] = t
        v += acc * dt
        h += v * dt
        power = p_nom_W * u**3
        e_j += power * dt
        i_peak = max(i_peak, power / v_nom_V)
        out["v_max_mps"] = max(out["v_max_mps"], v)
        t += dt
        if phase == "COAST" and v <= 0.0:
            out.update(reached=True, apogee_m=h, t_apogee_s=t)
            break
        if e_j / 3600.0 > batt_Wh:
            out["reason"] = "battery exhausted before apogee"
            break
    else:
        out["reason"] = f"t_limit {t_limit_s:.0f} s reached before apogee"
    if out["liftoff_t_s"] is None:
        out["reason"] = "thrust at climb throttle <= weight: no lift-off"
    out.update(t_end_s=t, h_end_m=h, energy_Wh=e_j / 3600.0, i_peak_A=i_peak,
               i_mean_A=(e_j / max(out["t_cutoff_s"] or t, dt)) / v_nom_V,
               overshoot_m=(out["apogee_m"] - target_m) if out["apogee_m"] is not None else None,
               powered_time_s=out["t_cutoff_s"] if out["t_cutoff_s"] is not None else t)
    return out


def battery_check(p: dict, flight: dict[str, Any]) -> dict[str, float | bool]:
    b = p["battery"]
    e = p["edf"]
    cap_ah = b["capacity_mAh"] / 1000.0
    cap_wh = cap_ah * e["voltage_nominal_V"]
    used_wh = flight["energy_Wh"]
    return {"capacity_Wh": cap_wh, "capacity_mAh": b["capacity_mAh"], "used_Wh": used_wh,
            "used_mAh": used_wh / e["voltage_nominal_V"] * 1000.0, "remaining_pct": 100.0 * (1.0 - used_wh / cap_wh),
            "c_rate_needed": flight["i_peak_A"] / cap_ah, "c_rating": b["c_rating"],
            "c_rate_ok": flight["i_peak_A"] / cap_ah <= b["c_rating"],
            "i_peak_A": flight["i_peak_A"], "i_edf_nominal_A": e["current_nominal_A"]}


def chute(p: dict, mass_g: float) -> dict[str, float]:
    rec, rho = p["recovery"], p["aero"]["rho_kg_m3"]
    m = mass_g * 1e-3
    v_t = rec["target_descent_mps"]
    a_req = 2 * m * G / (rho * rec["vehicle_chute_cd"] * v_t * v_t)
    a_act = math.pi / 4 * rec["vehicle_chute_d_m"] ** 2
    v_act = math.sqrt(2 * m * G / (rho * rec["vehicle_chute_cd"] * a_act))
    return {"mass_g": mass_g, "target_descent_mps": v_t, "cd": rec["vehicle_chute_cd"],
            "area_required_m2": a_req, "diameter_required_m": math.sqrt(4 * a_req / math.pi),
            "diameter_params_m": rec["vehicle_chute_d_m"], "area_params_m2": a_act,
            "descent_with_params_chute_mps": v_act, "landing_energy_J": 0.5 * m * v_act * v_act}


def stability(p: dict, lay: Layout, est: se.StructureEstimator, cg_full: MassProps, cg_post: MassProps) -> dict[str, Any]:
    """Barrowman CP (nose + 4 trapezoid fins, body-diameter reference) and static margins."""
    nose_vol = barrowman.nose_volume(lay.nose_outer_radius, lay.nose_len)
    nose = barrowman.nose_from_volume(lay.nose_len, lay.body_r, nose_vol)
    a_exp, b, s, z_le_root = est.fin_exposed_planform()
    fins = barrowman.trapezoid_fins(len(lay.fin_thetas), s, a_exp, b, z_le_root - lay.fin_tip_le_z,
                                    lay.vane_section_r, lay.body_od, lay.z_nose_tip - z_le_root)
    # boat tail = the outer skin from body_od at cone_z1 down to vane_section_od at z_vane_section_top
    # (Layout.outer_skin_profile).  v0.5 tabs: the skin is a cylinder (= body) from cone_z1 up to the spigot,
    # so the transition ends at cone_z1, not at z_prop_spigot_bottom.  The 45 deg chamfer at its foot is
    # folded into the straight cone (C_Na is exact - it depends on the end diameters only; the CP is not).
    boat = barrowman.transition(lay.body_od, lay.vane_section_od, lay.cone_z1 - lay.z_vane_section_top, lay.body_od,
                                lay.z_nose_tip - lay.cone_z1, "boat_tail_cone")
    cna, x_cp = barrowman.combine([nose, fins])
    cna_bt, x_cp_bt = barrowman.combine([nose, fins, boat])
    z_cp, z_cp_bt = lay.z_nose_tip - x_cp, lay.z_nose_tip - x_cp_bt

    def margin(z_cg: float, z_c: float) -> float:
        return (z_cg - z_c) / lay.body_od

    cg_above_hinge = cg_full.com_mm[2] - lay.vane_hinge_z
    sm_full = margin(cg_full.com_mm[2], z_cp)
    verdict = ("FAIL" if cg_above_hinge < C13_CG_ABOVE_HINGE_MM or sm_full < C13_MARGIN_WARN_CAL
               else "PASS" if sm_full >= C13_MARGIN_PASS_CAL else "WARN")
    return {
        "components": [{"name": c.name, "cna_per_rad": c.cna_per_rad, "x_from_tip_mm": c.x_cp_from_tip_mm,
                        "z_mm": lay.z_nose_tip - c.x_cp_from_tip_mm} for c in (nose, fins, boat)],
        "fin_exposed": {"root_chord_mm": a_exp, "tip_chord_mm": b, "semispan_mm": s,
                        "le_sweep_mm": z_le_root - lay.fin_tip_le_z, "root_radius_mm": lay.vane_section_r},
        "nose_volume_mm3": nose_vol,
        "cna_total_per_rad": cna, "z_cp_mm": z_cp, "z_cg_full_mm": cg_full.com_mm[2], "z_cg_post_ejection_mm": cg_post.com_mm[2],
        "static_margin_full_cal": sm_full, "static_margin_post_ejection_cal": margin(cg_post.com_mm[2], z_cp),
        "with_boat_tail": {"cna_total_per_rad": cna_bt, "z_cp_mm": z_cp_bt,
                           "static_margin_full_cal": margin(cg_full.com_mm[2], z_cp_bt)},
        "cg_above_hinge_mm": cg_above_hinge, "lever_arm_l_v_mm": cg_above_hinge,
        "cg_lateral_offset_mm": math.hypot(cg_full.com_mm[0], cg_full.com_mm[1]),
        "C13": {"verdict": verdict, "cg_above_hinge_min_mm": C13_CG_ABOVE_HINGE_MM,
                "margin_pass_cal": C13_MARGIN_PASS_CAL, "margin_warn_cal": C13_MARGIN_WARN_CAL},
    }


def legal(p: dict, mass_g: float, apogee_m: float | None) -> dict[str, Any]:
    m_kg = mass_g / 1000.0
    cls = next(label for bound, label in KR_MASS_CLASSES if m_kg <= bound)
    target = p["mission"]["target_altitude_m"]
    return {"takeoff_mass_kg": m_kg, "operator_certificate": cls,
            "aircraft_registration_required": m_kg > KR_REGISTRATION_KG,
            "registration_threshold_kg": KR_REGISTRATION_KG,
            "target_altitude_m": target, "flight_approval_required_at_target": target > KR_APPROVAL_ALT_M,
            "flight_approval_required_at_predicted_apogee": (apogee_m or 0.0) > KR_APPROVAL_ALT_M,
            "predicted_apogee_m": apogee_m, "approval_altitude_threshold_m": KR_APPROVAL_ALT_M,
            "competition_altitudes_m": p["payload"]["competition_deploy_alt_m"],
            "note_kr": p["mission"]["legal_note_kr"]}


# --------------------------------------------------------------------------- #
# run
# --------------------------------------------------------------------------- #
def run(preset: str | None = None, cad_report: str | None = None, out_dir: str | None = None,
        write: bool = True, use_cad: bool = True) -> dict[str, Any]:
    """Run the sizing.  ``use_cad=False`` forces the analytic structure ESTIMATE even when a CAD report exists."""
    p = load_params(PARAMS_PATH, preset)
    lay = Layout(p)
    preset_name = p["meta"]["preset"]
    default_preset = load_params(PARAMS_PATH)["meta"]["preset"]
    sha = params_sha256()
    warnings: list[str] = []
    rep = (find_cad_report(cad_report, preset_name, sha, warnings, default_preset, se.expected_report_parts(lay, p))
           if (use_cad or cad_report) else None)
    items, pad = build_budget(lay, p, rep, warnings)
    est = se.StructureEstimator(lay, p)

    full = mass_props(items)
    post_items = [i for i in items if i.name not in EJECTED]
    post = mass_props(post_items)
    chute_mass = sum(i.mass_g for i in items if i.name == "chute_pack")
    nose_mass = sum(i.mass_g for i in items if i.name == "nose_cone")
    printed = [i for i in items if i.kind == "printed"]
    structure_source = "ESTIMATE (analytic, as printed)" if rep is None else f"CAD report: {os.path.basename(rep['_path'])}"

    # thrust
    e = p["edf"]
    w_n = full.mass_g * 1e-3 * G
    n_exp = e["throttle_exponent"]
    u_hover = (w_n / e["thrust_nominal_N"]) ** (1.0 / n_exp)
    tw_nom = e["thrust_nominal_N"] / w_n
    thrust = {"weight_N": w_n, "mass_g": full.mass_g,
              "tw_nominal": tw_nom, "tw_peak": e["thrust_peak_N"] / w_n,
              "hover_throttle": u_hover, "hover_possible": u_hover <= 1.0,
              "hover_power_W": e["power_nominal_W"] * min(u_hover, 1.0) ** 3,
              "hover_current_A": e["power_nominal_W"] * min(u_hover, 1.0) ** 3 / e["voltage_nominal_V"],
              "climb_throttle": p["mission"]["climb_throttle"],
              "tw_at_climb_throttle": e["thrust_nominal_N"] * p["mission"]["climb_throttle"] ** n_exp / w_n,
              "throttle_for_tw_1p30": (C12_TW_PASS * w_n / e["thrust_nominal_N"]) ** (1.0 / n_exp),
              "max_mass_for_tw_1p30_g": e["thrust_nominal_N"] / (C12_TW_PASS * G) * 1000.0,
              "C12": {"verdict": "PASS" if tw_nom >= C12_TW_PASS else "WARN" if tw_nom >= C12_TW_WARN else "FAIL",
                      "pass_min": C12_TW_PASS, "warn_min": C12_TW_WARN}}
    if rep is None:
        solid_mass = sum((i.mass_solid_g or i.mass_g) for i in printed) + sum(i.mass_g for i in items if i.kind != "printed")
        thrust["tw_nominal_fully_solid_structure"] = e["thrust_nominal_N"] / (solid_mass * 1e-3 * G)
        thrust["mass_fully_solid_structure_g"] = solid_mass

    # exhaust + authority (at nominal thrust and at hover thrust)
    l_v = (full.com_mm[2] - lay.vane_hinge_z) * 1e-3
    ex_nom = exhaust(p, lay, e["thrust_nominal_N"])
    ex_hover = exhaust(p, lay, min(w_n, e["thrust_nominal_N"]))
    ex_climb = exhaust(p, lay, e["thrust_nominal_N"] * p["mission"]["climb_throttle"] ** n_exp)
    auth_nom = vane_authority(p, lay, ex_nom["q_e_Pa"], l_v, full.inertia_about_com_kg_m2)
    auth_hover = vane_authority(p, lay, ex_hover["q_e_Pa"], l_v, full.inertia_about_com_kg_m2)
    sw = swirl_torque(p)
    k_roll = 4 * p["vane"]["force_centroid_r"] * 1e-3 * ex_hover["q_e_Pa"] * lay.vane_area_mm2 * 1e-6 * p["vane"]["cl_alpha_per_rad"]
    roll = {**sw, "M_roll_max_hover_Nm": auth_hover["M_roll_Nm"], "M_roll_max_nominal_Nm": auth_nom["M_roll_Nm"],
            "authority_over_swirl_hover": auth_hover["M_roll_Nm"] / sw["tau_swirl_Nm"],
            "authority_over_swirl_nominal": auth_nom["M_roll_Nm"] / sw["tau_swirl_Nm"],
            "trim_delta_z_deg_hover": math.degrees(sw["tau_swirl_Nm"] / k_roll),
            "trim_fraction_of_cmd_limit": math.degrees(sw["tau_swirl_Nm"] / k_roll) / p["vane"]["max_deflection_cmd_deg"],
            "roll_priority_scale": p["control"]["roll_priority_scale"],
            "note": "swirl ~ u^2 and q_e ~ u^2 -> the trim deflection is roughly throttle-independent; "
                    "omega from kv x V is the unloaded speed, so tau_swirl is a lower bound"}

    # mission energy
    m_kg = full.mass_g * 1e-3
    a_ref = math.pi * (lay.body_r * 1e-3) ** 2
    # wire09c clock mount: the lead-passage blister stands out of the body (Layout.lp_blister_frontal_mm2, its axial
    # projection outside the body circle) - added to the drag reference area at the body Cd (E)
    a_blister = getattr(lay, "lp_blister_frontal_mm2", 0.0) * 1e-6
    a_ref += a_blister
    cap_wh = p["battery"]["capacity_mAh"] / 1000.0 * e["voltage_nominal_V"]
    common = dict(m_kg=m_kg, t_max_N=e["thrust_nominal_N"], n_exp=n_exp, ramp_s=p["mission"]["throttle_ramp_s"],
                  tau_s=e["thrust_time_constant_s"], rho=p["aero"]["rho_kg_m3"], cd=p["aero"]["body_cd"], a_ref_m2=a_ref,
                  k_coast=p["mission"]["coast_drag_factor"], p_nom_W=e["power_nominal_W"], v_nom_V=e["voltage_nominal_V"],
                  batt_Wh=cap_wh)
    scenarios: dict[str, dict[str, Any]] = {}
    alts: list[tuple[str, float, float]] = [(f"target_{p['mission']['target_altitude_m']:.0f}m (params)",
                                             p["mission"]["target_altitude_m"], p["mission"]["climb_throttle"])]
    for div, alt in p["payload"]["competition_deploy_alt_m"].items():
        alts.append((f"competition {div} {alt:.0f}m", float(alt), p["mission"]["climb_throttle"]))
    alts.append((f"target_{p['mission']['target_altitude_m']:.0f}m @ u=1.00 (sensitivity)", p["mission"]["target_altitude_m"], 1.0))
    for label, alt, u in alts:
        fl = climb_1d(target_m=alt, u_climb=u, **common)
        fl["label"] = label
        fl["battery"] = battery_check(p, fl)
        fl["geofence_m"] = p["mission"]["geofence_altitude_m"]
        fl["exceeds_geofence"] = (fl["apogee_m"] or fl["h_end_m"]) > p["mission"]["geofence_altitude_m"] or alt > p["mission"]["geofence_altitude_m"]
        scenarios[f"alt_{alt:.0f}m_u{u:.2f}"] = fl
    main_key = f"alt_{p['mission']['target_altitude_m']:.0f}m_u{p['mission']['climb_throttle']:.2f}"
    main_flight = scenarios[main_key]

    # recovery
    rec = {"post_ejection": chute(p, post.mass_g),
           "post_ejection_plus_chute_pack (SPEC descent_mass_g)": chute(p, post.mass_g + chute_mass),
           "post_ejection_plus_nose_and_chute_pack (tethered nose)": chute(p, post.mass_g + chute_mass + nose_mass)}

    stab = stability(p, lay, est, full, post)
    leg = legal(p, full.mass_g, main_flight["apogee_m"])
    a_pt, b_pt, rod_len = se.pushrod_endpoints(lay)

    # warnings / notes
    if thrust["C12"]["verdict"] != "PASS":
        warnings.append(f"C12 {thrust['C12']['verdict']}: T/W at thrust_nominal = {tw_nom:.3f} (PASS >= {C12_TW_PASS}, WARN >= {C12_TW_WARN}); "
                        f"mass {full.mass_g:.0f} g vs {thrust['max_mass_for_tw_1p30_g']:.0f} g allowed for 1.30")
    if not thrust["hover_possible"]:
        warnings.append(f"hover impossible: weight {w_n:.1f} N > thrust_nominal {e['thrust_nominal_N']} N (u_hover = {u_hover:.2f} > 1)")
    if main_flight["tw_climb"] < 1.0:
        warnings.append(f"no lift-off at climb_throttle {p['mission']['climb_throttle']}: thrust {main_flight['thrust_climb_N']:.1f} N <= weight {w_n:.1f} N")
    elif main_flight["tw_climb"] < C12_TW_WARN:
        warnings.append(f"marginal climb: T/W at climb_throttle {p['mission']['climb_throttle']} = {main_flight['tw_climb']:.3f}")
    if not main_flight["reached"]:
        warnings.append(f"mission to {p['mission']['target_altitude_m']:.0f} m not completed in the 1-D model: {main_flight['reason']}")
    for k, fl in scenarios.items():
        if fl["exceeds_geofence"]:
            warnings.append(f"scenario {fl['label']}: altitude exceeds geofence_altitude_m {p['mission']['geofence_altitude_m']:.0f} m -> ABORT by the onboard geofence interlock I5 (SPEC s8.8)")
    if stab["C13"]["verdict"] != "PASS":
        warnings.append(f"C13 {stab['C13']['verdict']}: static margin {stab['static_margin_full_cal']:.2f} cal, CG above hinge {stab['cg_above_hinge_mm']:.0f} mm")
    if rec["post_ejection"]["diameter_required_m"] > p["recovery"]["vehicle_chute_d_m"]:
        warnings.append(f"chute too small: required D {rec['post_ejection']['diameter_required_m']:.2f} m > {p['recovery']['vehicle_chute_d_m']} m")
    if leg["flight_approval_required_at_predicted_apogee"] and not leg["flight_approval_required_at_target"]:
        warnings.append(f"predicted apogee {main_flight['apogee_m']:.0f} m exceeds {KR_APPROVAL_ALT_M:.0f} m although the target does not -> 비행승인 strictly required")
    if rep is None:
        warnings.append("structure masses are an analytic ESTIMATE ("
                        + ("no cad/exports/*_report.json found" if use_cad else "CAD reports ignored: estimate-only run") + ")")
    assumed = sorted(k for sec in p.values() if isinstance(sec, dict) for k in sec if k.endswith("_ASSUMED"))

    def item_dict(i: BudgetItem) -> dict[str, Any]:
        return {"kind": i.kind, "group": i.group, "mass_g": i.mass_g, "mass_solid_g": i.mass_solid_g,
                "volume_mm3": i.volume_mm3, "density_g_cm3": i.density_g_cm3, "bbox_mm": None,
                "com_mm": list(i.com_mm), "inertia_about_com_kg_m2": i.inertia_about_com_kg_m2,
                "source": i.source, "note": i.note}

    result: dict[str, Any] = {
        "generator": "sizing", "preset": preset_name, "params_sha256": sha, "structure_source": structure_source,
        "cad_report": rep["_path"] if rep else None,
        "parts": {i.name: item_dict(i) for i in items},
        "ground_equipment": {pad.name: item_dict(pad)},
        "totals": {
            "mass_g": full.mass_g, "weight_N": w_n,
            "printed_mass_g": sum(i.mass_g for i in printed),
            "printed_mass_solid_g": sum((i.mass_solid_g or i.mass_g) for i in printed),
            "non_printed_mass_g": sum(i.mass_g for i in items if i.kind != "printed"),
            "by_group_g": {g: sum(i.mass_g for i in items if i.group == g) for g in sorted({i.group for i in items})},
            "com_mm": list(full.com_mm), "inertia_about_com_kg_m2": full.inertia_about_com_kg_m2,
            "post_ejection": {"mass_g": post.mass_g, "com_mm": list(post.com_mm),
                              "inertia_about_com_kg_m2": post.inertia_about_com_kg_m2, "excluded": list(EJECTED)},
            "descent_mass_g": post.mass_g + chute_mass,
            "descent_mass_incl_nose_g": post.mass_g + chute_mass + nose_mass,
        },
        "derived": {"pushrod_len_mm": rod_len, "pushrod_endpoints_mm": [list(a_pt), list(b_pt)],
                    "nose_cavity_volume_L": lay.nose_cavity_volume_L, "footprint_d_mm": lay.footprint_d,
                    "vehicle_h_mm": lay.vehicle_h, "inlet_area_ratio": lay.inlet_area_ratio,
                    "inlet_area_ratio_outer": lay.inlet_area_ratio_outer},   # v0.6.1: min / outer; v0.6.2: gussets included
        "thrust": thrust,
        "exhaust": {"at_nominal_thrust": ex_nom, "at_hover_thrust": ex_hover, "at_climb_throttle": ex_climb},
        "vane_authority": {"at_nominal_thrust": auth_nom, "at_hover_thrust": auth_hover},
        "roll": roll,
        "mission": {"main_scenario": main_key, "scenarios": scenarios, "battery_capacity_Wh": cap_wh,
                    "A_ref_m2": a_ref, "A_ref_blister_m2_E": a_blister, "model": "1-D vertical, thrust T_max u^n with first-order lag, ramp, drag 0.5 rho v^2 Cd A_ref, cut-off h + k v^2/2g"},
        "recovery": rec,
        "stability": stab,
        "legal": leg,
        "checks": {"C12": thrust["C12"], "C13": stab["C13"]},
        "assumed_keys": assumed,
        "warnings": warnings,
    }
    if write:
        out_dir = out_dir or HERE
        os.makedirs(out_dir, exist_ok=True)
        suffix = "" if preset_name == default_preset else f"_{preset_name}"
        json_path = os.path.join(out_dir, f"sizing_report{suffix}.json")
        md_path = os.path.join(out_dir, f"sizing_report{suffix}.md")
        with open(json_path, "w", encoding="utf-8", newline="\n") as f:
            json.dump(result, f, ensure_ascii=False, indent=2)
            f.write("\n")
        with open(md_path, "w", encoding="utf-8", newline="\n") as f:
            f.write(render_markdown(result, items, pad, lay, p))
        result["_files"] = {"json": json_path, "md": md_path}
    return result


# --------------------------------------------------------------------------- #
# markdown
# --------------------------------------------------------------------------- #
def _f(x: Any, nd: int = 1) -> str:
    if x is None:
        return "-"
    if isinstance(x, bool):
        return "yes" if x else "no"
    if isinstance(x, (int, float)):
        return f"{x:.{nd}f}"
    return str(x)


def _table(headers: list[str], rows: list[list[str]]) -> str:
    out = ["| " + " | ".join(headers) + " |", "|" + "|".join("---" for _ in headers) + "|"]
    out += ["| " + " | ".join(r) + " |" for r in rows]
    return "\n".join(out) + "\n"


def render_markdown(r: dict[str, Any], items: list[BudgetItem], pad: BudgetItem, lay: Layout, p: dict) -> str:
    t, th, ex, au, ro, st, lg = r["totals"], r["thrust"], r["exhaust"], r["vane_authority"], r["roll"], r["stability"], r["legal"]
    e = p["edf"]
    L: list[str] = []
    L.append(f"# 산정 보고서 (sizing) — preset `{r['preset']}`\n")
    L.append(f"- params.json sha256: `{r['params_sha256'][:16]}…`\n- 구조물 질량 출처: **{r['structure_source']}**\n"
             f"- EDF: {e['model']} — 공칭 추력 {e['thrust_nominal_N']} N / 피크 {e['thrust_peak_N']} N, {e['cells']}S {e['voltage_nominal_V']} V\n"
             f"- 전장 {_f(r['derived']['vehicle_h_mm'], 0)} mm, 핀 포함 외경 {_f(r['derived']['footprint_d_mm'], 0)} mm, 동체 OD {_f(lay.body_od, 0)} mm\n")
    L.append("## 판정 요약\n")
    L.append(_table(["검사", "결과", "값"], [
        ["C12 추중비 (T/W @ thrust_nominal)", f"**{th['C12']['verdict']}**", f"{_f(th['tw_nominal'], 3)} (PASS ≥ {th['C12']['pass_min']}, WARN ≥ {th['C12']['warn_min']})"],
        ["C13 정적 안정성 (nose+fins Barrowman)", f"**{st['C13']['verdict']}**", f"정적 여유 {_f(st['static_margin_full_cal'], 2)} cal, CG−hinge {_f(st['cg_above_hinge_mm'], 0)} mm (≥ {_f(st['C13']['cg_above_hinge_min_mm'], 0)})"],
        ["호버 가능 (u_h ≤ 1)", "**OK**" if th["hover_possible"] else "**NO**", f"u_h = {_f(th['hover_throttle'], 3)}"],
        ["임무 상승 (1-D) 목표 도달", "**OK**" if r["mission"]["scenarios"][r["mission"]["main_scenario"]]["reached"] else "**NO**",
         r["mission"]["scenarios"][r["mission"]["main_scenario"]]["reason"] or f"정점 {_f(r['mission']['scenarios'][r['mission']['main_scenario']]['apogee_m'], 1)} m"],
        ["낙하산 크기", "**OK**" if r["recovery"]["post_ejection"]["diameter_required_m"] <= p["recovery"]["vehicle_chute_d_m"] else "**NO**",
         f"요구 D {_f(r['recovery']['post_ejection']['diameter_required_m'], 2)} m vs params {p['recovery']['vehicle_chute_d_m']} m"],
    ]))

    L.append("## 1. 질량 예산 (mass budget)\n")
    rows = []
    for i in sorted(items, key=lambda i: (i.group != "structure", i.group, i.name)):
        rows.append([f"`{i.name}`", i.kind, i.group, _f(i.mass_g, 1), _f(i.mass_solid_g, 1) if i.kind == "printed" else "-",
                     f"{_f(i.com_mm[0], 1)}, {_f(i.com_mm[1], 1)}, {_f(i.com_mm[2], 1)}", i.source, i.note])
    L.append(_table(["항목", "종류", "그룹", "질량 g", "solid g", "CoM x,y,z mm", "출처", "비고"], rows))
    L.append(_table(["합계", "값"], [
        ["**전체 (full stack)**", f"**{_f(t['mass_g'], 1)} g** = {_f(t['weight_N'], 2)} N"],
        ["출력 구조물 (as printed / fully solid)", f"{_f(t['printed_mass_g'], 1)} g / {_f(t['printed_mass_solid_g'], 1)} g"],
        ["비출력 품목", f"{_f(t['non_printed_mass_g'], 1)} g"],
        *[[f"그룹 `{g}`", f"{_f(m, 1)} g"] for g, m in t["by_group_g"].items()],
        ["**사출 후 (post-ejection: −cansat, −nose_cone, −chute_pack)**", f"**{_f(t['post_ejection']['mass_g'], 1)} g**"],
        ["하강 질량 descent_mass_g (post-ejection + chute_pack)", f"{_f(t['descent_mass_g'], 1)} g"],
        ["하강 질량 (노즈콘 테더 포함)", f"{_f(t['descent_mass_incl_nose_g'], 1)} g"],
        ["CG full (x, y, z)", ", ".join(_f(c, 1) for c in t["com_mm"]) + " mm"],
        ["CG post-ejection (x, y, z)", ", ".join(_f(c, 1) for c in t["post_ejection"]["com_mm"]) + " mm"],
        ["I_xx / I_yy / I_zz (full, about CG)", " / ".join(_f(t["inertia_about_com_kg_m2"][k][k], 4) for k in range(3)) + " kg·m²"],
        ["I_xx / I_yy / I_zz (post-ejection)", " / ".join(_f(t["post_ejection"]["inertia_about_com_kg_m2"][k][k], 4) for k in range(3)) + " kg·m²"],
        ["발사대 `launch_pad` (지상 장비, 비행 질량 제외)", f"{_f(pad.mass_g, 1)} g (solid {_f(pad.mass_solid_g, 1)} g)"],
    ]))
    L.append("ESTIMATE 열: 출력 구조물은 해석식(튜브 = 면적 × 벽 × 밀도, 콘/노즈 = 회전 쉘 수치적분, 핀 = 평면형 × t 등)으로 추정. "
             f"'as printed'는 벽 {p['printer']['wall_loops']} 루프 × {p['printer']['line_w']} mm + {p['printer']['infill_pct']} % 인필 모델, "
             "'solid'는 CAD 질량특성 보고서가 보여줄 완전 충전 값. CAD 보고서가 생기면 자동으로 그 값을 사용한다.\n")

    L.append("## 2. 추력 / 추중비\n")
    rows = [
        ["중량 W", f"{_f(th['weight_N'], 2)} N ({_f(th['mass_g'], 0)} g)"],
        ["T/W @ thrust_nominal", f"{_f(th['tw_nominal'], 3)} → C12 **{th['C12']['verdict']}**"],
        ["T/W @ thrust_peak", _f(th["tw_peak"], 3)],
        [f"호버 스로틀 u_h = (W/T_max)^(1/{e['throttle_exponent']:g})", f"{_f(th['hover_throttle'], 3)} → P {_f(th['hover_power_W'], 0)} W, I {_f(th['hover_current_A'], 1)} A"],
        [f"T/W @ climb_throttle {th['climb_throttle']}", _f(th["tw_at_climb_throttle"], 3)],
        ["T/W = 1.30 에 필요한 스로틀", _f(th["throttle_for_tw_1p30"], 3)],
        ["T/W = 1.30 허용 최대 질량", f"{_f(th['max_mass_for_tw_1p30_g'], 0)} g"],
    ]
    if "tw_nominal_fully_solid_structure" in th:
        rows.append(["T/W (구조물 fully solid 가정)", f"{_f(th['tw_nominal_fully_solid_structure'], 3)} @ {_f(th['mass_fully_solid_structure_g'], 0)} g"])
    L.append(_table(["항목", "값"], rows))

    L.append("## 3. 배기 유속 · 베인 제어권한 (SPEC §8.3–8.4)\n")
    L.append(_table(["조건", "T N", "A_e m²", "V_e m/s", "q_e Pa", "ṁ kg/s"], [
        [k, _f(v["thrust_N"], 1), _f(v["A_e_m2"], 5), _f(v["V_e_mps"], 1), _f(v["q_e_Pa"], 0), _f(v["mass_flow_kg_s"], 3)]
        for k, v in ex.items()]))
    rows = []
    for k, a in au.items():
        rows.append([k, _f(a["force_per_vane_N"], 2), _f(a["M_pitch_Nm"], 3), _f(a["M_roll_Nm"], 3),
                     _f(a["alpha_pitch_rad_s2"], 2), _f(a["alpha_roll_rad_s2"], 2), _f(a["moment_per_deg_pitch_Nm"], 4)])
    L.append(_table(["조건", f"F/베인 @ {_f(p['vane']['max_deflection_cmd_deg'], 0)}° N", "M_pitch N·m", "M_roll N·m",
                     "α_pitch rad/s²", "α_roll rad/s²", "M_pitch per deg"], rows))
    a0 = au["at_nominal_thrust"]
    L.append(f"- 레버암 l_v = z_CG − vane_hinge_z = {_f(st['lever_arm_l_v_mm'], 1)} mm; S_v = {_f(a0['S_v_m2'] * 1e6, 0)} mm²; "
             f"C_Lα = {p['vane']['cl_alpha_per_rad']} /rad (stall {p['vane']['stall_deg']}°); r_v = {p['vane']['force_centroid_r']} mm\n"
             f"- 관성 (full, CG 기준): I_xx {_f(a0['I_xx_kg_m2'], 4)}, I_yy {_f(a0['I_yy_kg_m2'], 4)}, I_zz {_f(a0['I_zz_kg_m2'], 4)} kg·m² (점질량+봉/상자 근사)\n"
             f"- CG 횡방향 오프셋 {_f(st['cg_lateral_offset_mm'], 1)} mm (배터리 −x / ESC·NAV +x 배치) → 트림 토크 ≈ W × 오프셋 = "
             f"{_f(th['weight_N'] * st['cg_lateral_offset_mm'] * 1e-3, 3)} N·m\n")
    L.append("### 롤 권한 vs 잔류 스월 토크\n")
    L.append(_table(["항목", "값"], [
        ["ω (kv × V, 무부하)", f"{_f(ro['rpm_unloaded'], 0)} rpm = {_f(ro['omega_rad_s'], 0)} rad/s"],
        ["P/ω @ power_nominal", f"{_f(ro['P_over_omega_Nm'], 3)} N·m"],
        [f"τ_swirl = {ro['fraction_ASSUMED']} × P/ω (ASSUMED)", f"{_f(ro['tau_swirl_Nm'], 3)} N·m"],
        ["M_roll,max @ hover / @ nominal", f"{_f(ro['M_roll_max_hover_Nm'], 3)} / {_f(ro['M_roll_max_nominal_Nm'], 3)} N·m"],
        ["권한/스월 비 @ hover / @ nominal", f"{_f(ro['authority_over_swirl_hover'], 2)} / {_f(ro['authority_over_swirl_nominal'], 2)}"],
        ["스월 트림 δ_z", f"{_f(ro['trim_delta_z_deg_hover'], 2)}° = {_f(100 * ro['trim_fraction_of_cmd_limit'], 0)} % of δ_cmd,max (mixer roll_priority_scale {ro['roll_priority_scale']})"],
    ]))
    L.append(f"{ro['note']}.\n")

    L.append("## 4. 임무 에너지 (1-D 수직 상승)\n")
    L.append(f"모델: {r['mission']['model']}; A_ref = {_f(r['mission']['A_ref_m2'], 5)} m², Cd {p['aero']['body_cd']}, "
             f"램프 {p['mission']['throttle_ramp_s']} s, 추력 시정수 {e['thrust_time_constant_s']} s, coast_drag_factor {p['mission']['coast_drag_factor']}, "
             f"배터리 {p['battery']['capacity_mAh']} mAh × {e['voltage_nominal_V']} V = {_f(r['mission']['battery_capacity_Wh'], 1)} Wh, C-rating {p['battery']['c_rating']}C.\n")
    rows = []
    for k, fl in r["mission"]["scenarios"].items():
        b = fl["battery"]
        rows.append([fl["label"], _f(fl["tw_climb"], 2), "yes" if fl["reached"] else f"NO ({fl['reason']})",
                     _f(fl["t_cutoff_s"], 1), _f(fl["h_cutoff_m"], 1), _f(fl["v_cutoff_mps"], 1),
                     _f(fl["apogee_m"], 1), _f(fl["overshoot_m"], 1), _f(fl["t_apogee_s"], 1),
                     _f(b["i_peak_A"], 1), f"{_f(b['c_rate_needed'], 1)}C", _f(b["used_Wh"], 2), _f(b["remaining_pct"], 1),
                     "⚠ geofence" if fl["exceeds_geofence"] else "ok"])
    L.append(_table(["시나리오", "T/W@u", "도달", "t_cut s", "h_cut m", "v_cut m/s", "정점 m", "오버슈트 m", "t_apogee s",
                     "I_peak A", "C-rate", "Wh 사용", "잔량 %", "geofence"], rows))
    L.append(f"지오펜스 geofence_altitude_m = {p['mission']['geofence_altitude_m']} m: 이를 넘는 시나리오는 §8.8 상태기계에서 ABORT 된다.\n")

    L.append("## 5. 회수 (낙하산)\n")
    rows = []
    for k, c in r["recovery"].items():
        rows.append([k, _f(c["mass_g"], 0), _f(c["area_required_m2"], 3), _f(c["diameter_required_m"], 2),
                     _f(c["diameter_params_m"], 2), _f(c["descent_with_params_chute_mps"], 2), _f(c["landing_energy_J"], 1)])
    L.append(_table(["하강 질량 정의", "질량 g", f"요구 면적 m² @ {p['recovery']['target_descent_mps']} m/s", "요구 D m",
                     "params D m", "params 낙하산 하강속도 m/s", "착지 에너지 J"], rows))
    L.append(f"Cd = {p['recovery']['vehicle_chute_cd']}, ρ = {p['aero']['rho_kg_m3']} kg/m³. 캔위성은 자체 낙하산(별도).\n")

    L.append("## 6. 안정성 (CG vs Barrowman CP)\n")
    L.append(_table(["성분", "C_Nα /rad", "x from tip mm", "z mm"],
                    [[c["name"], _f(c["cna_per_rad"], 3), _f(c["x_from_tip_mm"], 1), _f(c["z_mm"], 1)] for c in st["components"]]))
    fe = st["fin_exposed"]
    L.append(_table(["항목", "값"], [
        ["핀 노출 평면형 (root c / tip c / s / LE sweep)", f"{_f(fe['root_chord_mm'], 1)} / {_f(fe['tip_chord_mm'], 1)} / {_f(fe['semispan_mm'], 1)} / {_f(fe['le_sweep_mm'], 1)} mm, 루트 반경 {_f(fe['root_radius_mm'], 1)} mm, 기준 직경 {_f(lay.body_od, 0)} mm"],
        ["CP (nose + fins)", f"z = {_f(st['z_cp_mm'], 1)} mm, C_Nα 합 {_f(st['cna_total_per_rad'], 3)} /rad"],
        ["CG full / post-ejection", f"z = {_f(st['z_cg_full_mm'], 1)} / {_f(st['z_cg_post_ejection_mm'], 1)} mm"],
        ["정적 여유 full (coast 형상)", f"**{_f(st['static_margin_full_cal'], 2)} cal** → C13 **{st['C13']['verdict']}** (PASS ≥ {st['C13']['margin_pass_cal']}, WARN ≥ {st['C13']['margin_warn_cal']})"],
        ["정적 여유 post-ejection (낙하산 하강 중 — 참고용)", f"{_f(st['static_margin_post_ejection_cal'], 2)} cal"],
        [f"감도: 보트테일(콘 {lay.body_od:g}→{lay.vane_section_od:g}"
         + (f", z {lay.z_vane_section_top:g}–{lay.cone_z1:.1f}; 그 위 원통부는 동체" if lay.tabs else "") + ") 포함",
         f"CP z = {_f(st['with_boat_tail']['z_cp_mm'], 1)} mm, 여유 {_f(st['with_boat_tail']['static_margin_full_cal'], 2)} cal"],
        ["CG − vane_hinge_z (= l_v)", f"{_f(st['cg_above_hinge_mm'], 1)} mm (C13 요구 ≥ {_f(st['C13']['cg_above_hinge_min_mm'], 0)})"],
    ]))

    L.append("## 7. 국내 법규 분류 (항공안전법, 질량 기준)\n")
    L.append(_table(["항목", "값"], [
        ["최대이륙중량", f"{_f(lg['takeoff_mass_kg'], 3)} kg"],
        ["조종자 증명", f"**{lg['operator_certificate']}**"],
        [f"기체신고 (> {lg['registration_threshold_kg']:g} kg)", "**필요**" if lg["aircraft_registration_required"] else "불필요"],
        [f"비행승인 (> {lg['approval_altitude_threshold_m']:.0f} m) @ 목표 고도 {lg['target_altitude_m']:.0f} m ★", "**필요**" if lg["flight_approval_required_at_target"] else "불필요 (고도 기준; 관제권/비행금지구역은 별도)"],
        [f"비행승인 @ 예측 정점 {_f(lg['predicted_apogee_m'], 1)} m", "**필요**" if lg["flight_approval_required_at_predicted_apogee"] else "불필요"],
        ["경연 사출 고도 (m)", ", ".join(f"{k} {v}" for k, v in lg["competition_altitudes_m"].items())
         + " → 비행승인 필요: " + (", ".join(k for k, v in lg["competition_altitudes_m"].items() if v > lg["approval_altitude_threshold_m"]) or "없음")],
    ]))
    L.append(f"> {lg['note_kr']}\n")

    L.append("## 8. 경고 · 가정\n")
    for w in r["warnings"]:
        L.append(f"- ⚠ {w}")
    L.append("")
    L.append(f"- `_ASSUMED` 키 (실측 전 신뢰 금지): {', '.join(r['assumed_keys'])}")
    L.append(f"- 푸시로드 길이 (arm tip → horn pin, 베인 1): {_f(r['derived']['pushrod_len_mm'], 2)} mm — CAD 보고서 `pushrod_len_mm` 와 대조")
    L.append(f"- 노즈 캐비티 {_f(r['derived']['nose_cavity_volume_L'], 2)} L (요구 {p['recovery']['chute_pack_volume_L_required']} L), 흡입 면적비 {_f(r['derived']['inlet_area_ratio'], 2)} (최소 개구, v0.6.2 창 아래 거싯 포함; 바깥면 {_f(r['derived'].get('inlet_area_ratio_outer'), 2)}) (요구 ≥ {p['intake_module']['min_inlet_area_ratio']})")
    L.append("- 추력·전력은 벤더 수치 (SPEC §12: 비행 전 벤치 추력 시험 필수); 배터리 전압 강하·ESC 효율은 모델에 없음.")
    L.append("")
    return "\n".join(L)


# --------------------------------------------------------------------------- #
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--preset", default=None, help="alt_presets name (default: params.meta.preset)")
    ap.add_argument("--cad-report", default=None, help="path to cad/exports/<generator>_report.json")
    ap.add_argument("--out-dir", default=None, help="output directory (default: sizing/)")
    ap.add_argument("--estimate-only", action="store_true",
                    help="ignore cad/exports/*_report.json and use the analytic structure ESTIMATE")
    a = ap.parse_args(argv)
    if a.estimate_only and a.cad_report:
        ap.error("--estimate-only and --cad-report are exclusive")
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")  # type: ignore[attr-defined]
    except Exception:
        pass
    r = run(a.preset, a.cad_report, a.out_dir, use_cad=not a.estimate_only)
    t, th, st = r["totals"], r["thrust"], r["stability"]
    fl = r["mission"]["scenarios"][r["mission"]["main_scenario"]]
    print(f"preset {r['preset']}  structure: {r['structure_source']}")
    print(f"mass {t['mass_g']:.1f} g (printed {t['printed_mass_g']:.1f}, post-ejection {t['post_ejection']['mass_g']:.1f})  "
          f"CG z {t['com_mm'][2]:.1f} mm")
    print(f"T/W nominal {th['tw_nominal']:.3f} [{th['C12']['verdict']}]  peak {th['tw_peak']:.3f}  u_hover {th['hover_throttle']:.3f}  "
          f"T/W@climb {th['tw_at_climb_throttle']:.3f}")
    print(f"V_e {r['exhaust']['at_nominal_thrust']['V_e_mps']:.1f} m/s  M_pitch {r['vane_authority']['at_nominal_thrust']['M_pitch_Nm']:.3f} N·m  "
          f"alpha {r['vane_authority']['at_nominal_thrust']['alpha_pitch_rad_s2']:.2f} rad/s²  roll/swirl {r['roll']['authority_over_swirl_hover']:.2f}")
    print(f"mission {fl['label']}: reached={fl['reached']} apogee {_f(fl['apogee_m'], 1)} m  t {_f(fl['t_apogee_s'], 1)} s  "
          f"I_peak {fl['i_peak_A']:.1f} A  {fl['energy_Wh']:.2f} Wh  {fl['reason']}")
    print(f"chute D_req {r['recovery']['post_ejection']['diameter_required_m']:.2f} m (params {r['recovery']['post_ejection']['diameter_params_m']} m)  "
          f"CP z {st['z_cp_mm']:.1f}  margin {st['static_margin_full_cal']:.2f} cal [{st['C13']['verdict']}]")
    print(f"legal: {r['legal']['operator_certificate']}; 기체신고 {r['legal']['aircraft_registration_required']}; "
          f"비행승인@target {r['legal']['flight_approval_required_at_target']}")
    for w in r["warnings"]:
        print("  WARN:", w)
    print("wrote", r["_files"]["md"], "and", r["_files"]["json"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
