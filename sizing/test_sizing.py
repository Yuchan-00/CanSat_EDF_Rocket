"""Tests for the sizing tool.  Run from the project root: ``python -m pytest sizing -q``.

Invariant checked here: the budget is internally consistent (sums, exclusions,
every SPEC s6 item present), the closed-form physics (T/W, hover throttle,
exhaust velocity, vane moments, chute) match their defining formulas, the
Barrowman implementation reproduces known cases, the CAD-report path overrides
the estimate (per preset: <generator>_report_<preset>.json for an alt preset),
and both presets run end-to-end and write their files.  v0.5: the tab-mount
solids (thin liner, per-screw lugs, holes as material removed, two-box wire
slot, wire channel + shelf notch) match hand volumes, lip_clamp adds none of
them, the edf_unit placeholder carries motor + tabs, and the boat tail ends at
cone_z1.  v0.5d: the channel floor wedge (unioned after the cuts), tabs at 90 / 270
with the leads at 0 deg, the tray stop in both presets, and a CAD report missing
any SPEC s6 component (launch_pad optional) is never used.  v0.6: the ``com`` placeholder (COM-board box,
electronics.com.mass_g, group avionics) and the payload module's COM bosses / blind holes match hand volumes in
both presets, and nothing else of the payload module changes.  v0.6.3: ``fc`` is ``nav`` (same box, electronics.nav)
and the ``fcu`` placeholder (box on the intake bulkhead, electronics.fcu.mass_g, group avionics) joins in both presets;
the servo-lead lump ends on the FCU.
"""
from __future__ import annotations

import hashlib
import importlib.util
import json
import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
for _p in (HERE, os.path.join(ROOT, "cad")):
    if _p not in sys.path:
        sys.path.insert(0, _p)


def _load_sizing():
    spec = importlib.util.spec_from_file_location("sizing_tool", os.path.join(HERE, "sizing.py"))
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = mod          # dataclasses resolve string annotations via sys.modules
    spec.loader.exec_module(mod)
    return mod


sz = _load_sizing()
bm = sz.barrowman
se = sz.se
layout = sys.modules["layout"]

DEFAULT = layout.load_params()["meta"]["preset"]            # BUDGET_90mm_6S since params v0.3
ALTS = list(layout.load_params()["alt_presets"])               # KAIST_90mm_12S (full design)
def spec6_placeholders(preset: str) -> list[str]:
    """SPEC s6 placeholder / lumped items of a preset.  v0.9 (vane_drive.mode): 'pushrod' keeps the arms, pushrods and
    their hardware lump; 'direct' has the trimmed stock horns and the cover / coupler screws lump instead, and the
    motor-lead lump (params motor_leads)."""
    names = ["edf_unit", "battery", "esc", "nav", "fcu", "misc_electronics", "com", "radio_log", "servo_release",
             "cansat_payload", "spring", "chute_pack", "joint_screws_inserts"]
    names += [f"servo_vane_{i}" for i in range(1, 5)]
    if layout.build(preset).vane_drive_mode == "direct":
        names += ["vane_drive_hardware", "motor_leads"] + [f"servo_horn_{i}" for i in range(1, 5)]
    else:
        names += ["pushrods_horns_clevises"] + [f"servo_arm_{i}" for i in range(1, 5)] + [f"pushrod_{i}" for i in range(1, 5)]
    return names


SPEC6_PLACEHOLDERS = spec6_placeholders(DEFAULT)


@pytest.fixture(scope="module")
def result():
    return sz.run(DEFAULT, write=False, use_cad=False)   # analytic ESTIMATE path, independent of cad/exports


# --------------------------------------------------------------------------- #
# budget
# --------------------------------------------------------------------------- #
def test_budget_sums_consistent(result):
    parts, t = result["parts"], result["totals"]
    total = sum(v["mass_g"] for v in parts.values())
    assert total == pytest.approx(t["mass_g"], abs=1e-9)
    assert sum(v["mass_g"] for v in parts.values() if v["kind"] == "printed") == pytest.approx(t["printed_mass_g"])
    assert t["printed_mass_g"] + t["non_printed_mass_g"] == pytest.approx(t["mass_g"])
    assert sum(t["by_group_g"].values()) == pytest.approx(t["mass_g"])
    ejected = sum(parts[n]["mass_g"] for n in sz.EJECTED)
    assert t["post_ejection"]["mass_g"] == pytest.approx(t["mass_g"] - ejected)
    assert t["descent_mass_g"] == pytest.approx(t["post_ejection"]["mass_g"] + parts["chute_pack"]["mass_g"])
    assert t["descent_mass_incl_nose_g"] == pytest.approx(t["descent_mass_g"] + parts["nose_cone"]["mass_g"])
    assert t["weight_N"] == pytest.approx(t["mass_g"] * 1e-3 * sz.G)
    assert t["printed_mass_solid_g"] >= t["printed_mass_g"]


def test_budget_contains_every_spec6_item_and_printed_part(result):
    parts = result["parts"]
    for name in SPEC6_PLACEHOLDERS:
        assert name in parts, name
        assert parts[name]["mass_g"] > 0
    lay = layout.build(DEFAULT)
    for name in lay.printed_parts:
        if name == "launch_pad":
            assert name in result["ground_equipment"] and name not in parts   # ground equipment, not flight mass
        elif name in ("vane", "vane_horn", "vane_coupler", "servo_cover"):     # v0.9: one per vane / fin
            assert all(f"{name}_{i}" in parts for i in range(1, lay.vane_count + 1))
        else:
            assert name in parts, name
            assert parts[name]["kind"] == "printed" and parts[name]["source"] == "ESTIMATE"
    p = lay.p
    assert parts["edf_unit"]["mass_g"] == p["edf"]["mass_g"]
    assert parts["battery"]["mass_g"] == p["battery"]["mass_g"]
    assert parts["cansat_payload"]["mass_g"] == p["payload"]["mass_g"]
    rec = p["recovery"]
    assert parts["chute_pack"]["mass_g"] == rec["chute_mass_g"] + rec["lines_swivel_mass_g"] + rec["shock_cord_mass_g"]
    assert parts["spring"]["mass_g"] + parts["spring_piston_hardware_balance"]["mass_g"] == pytest.approx(
        p["hardware_mass_g"]["spring_piston_hardware"])


def test_mass_props_point_masses():
    I0 = [[0.0] * 3 for _ in range(3)]
    a = sz.BudgetItem("a", "lumped", "x", 100.0, (0.0, 0.0, 0.0), I0, "t")
    b = sz.BudgetItem("b", "lumped", "x", 100.0, (0.0, 0.0, 200.0), I0, "t")
    mp = sz.mass_props([a, b])
    assert mp.mass_g == 200.0
    assert mp.com_mm == pytest.approx((0.0, 0.0, 100.0))
    expect = 2 * 0.1 * 0.1**2   # 2 m d^2, d = 100 mm
    assert mp.inertia_about_com_kg_m2[0][0] == pytest.approx(expect)
    assert mp.inertia_about_com_kg_m2[1][1] == pytest.approx(expect)
    assert mp.inertia_about_com_kg_m2[2][2] == pytest.approx(0.0)


def test_placeholder_density_is_mass_over_volume(result):
    parts = result["parts"]
    for name in ("battery", "esc", "nav", "fcu", "edf_unit", "cansat_payload"):
        v = parts[name]
        assert v["density_g_cm3"] == pytest.approx(v["mass_g"] / (v["volume_mm3"] * 1e-3), rel=1e-9)
    lay = layout.build(DEFAULT)
    bx, by, bz = lay.battery_x, lay.battery_y, lay.battery_z
    assert parts["battery"]["volume_mm3"] == pytest.approx((bx[1] - bx[0]) * (by[1] - by[0]) * (bz[1] - bz[0]))
    assert parts["battery"]["com_mm"] == pytest.approx([(bx[0] + bx[1]) / 2, (by[0] + by[1]) / 2, (bz[0] + bz[1]) / 2])


# --------------------------------------------------------------------------- #
# thrust / exhaust / vanes
# --------------------------------------------------------------------------- #
def test_tw_formula(result):
    p = layout.load_params(layout.PARAMS_PATH, DEFAULT)
    th, t = result["thrust"], result["totals"]
    w = t["mass_g"] * 1e-3 * sz.G
    assert th["tw_nominal"] == pytest.approx(p["edf"]["thrust_nominal_N"] / w)
    assert th["tw_peak"] == pytest.approx(p["edf"]["thrust_peak_N"] / w)
    n = p["edf"]["throttle_exponent"]
    assert p["edf"]["thrust_nominal_N"] * th["hover_throttle"] ** n == pytest.approx(w)
    assert th["tw_at_climb_throttle"] == pytest.approx(th["tw_nominal"] * p["mission"]["climb_throttle"] ** n)
    verdict = "PASS" if th["tw_nominal"] >= 1.30 else "WARN" if th["tw_nominal"] >= 1.15 else "FAIL"
    assert th["C12"]["verdict"] == verdict


def test_exhaust_velocity_formula():
    p = layout.load_params(layout.PARAMS_PATH, DEFAULT)
    lay = layout.Layout(p)
    ex = sz.exhaust(p, lay, p["edf"]["thrust_nominal_N"])
    d = lay.duct_id * 1e-3
    a_e = math.pi / 4 * (d**2 - (p["edf"]["exit_hub_fraction"] * d) ** 2)
    assert ex["A_e_m2"] == pytest.approx(a_e)
    assert ex["V_e_mps"] == pytest.approx(math.sqrt(p["edf"]["thrust_nominal_N"] / (p["aero"]["rho_kg_m3"] * a_e)))
    assert ex["q_e_Pa"] == pytest.approx(0.5 * p["aero"]["rho_kg_m3"] * ex["V_e_mps"] ** 2)
    # thrust = mdot * V_e for a uniform jet
    assert ex["mass_flow_kg_s"] * ex["V_e_mps"] == pytest.approx(p["edf"]["thrust_nominal_N"])


def test_vane_moments_match_spec_84():
    p = layout.load_params(layout.PARAMS_PATH, DEFAULT)
    lay = layout.Layout(p)
    q_e, l_v = 5000.0, 0.4
    I = [[0.2, 0, 0], [0, 0.2, 0], [0, 0, 0.006]]
    a = sz.vane_authority(p, lay, q_e, l_v, I)
    s_v = lay.vane_area_mm2 * 1e-6
    d = math.radians(p["vane"]["max_deflection_cmd_deg"])
    f = q_e * s_v * p["vane"]["cl_alpha_per_rad"] * d
    assert a["force_per_vane_N"] == pytest.approx(f)
    assert a["M_pitch_Nm"] == pytest.approx(2 * l_v * f)
    assert a["M_roll_Nm"] == pytest.approx(4 * p["vane"]["force_centroid_r"] * 1e-3 * f)
    assert a["alpha_pitch_rad_s2"] == pytest.approx(a["M_pitch_Nm"] / 0.2)
    assert a["alpha_roll_rad_s2"] == pytest.approx(a["M_roll_Nm"] / 0.006)
    # C_L is flat beyond stall
    assert sz.vane_cl(p, p["vane"]["stall_deg"] + 10) == pytest.approx(sz.vane_cl(p, p["vane"]["stall_deg"]))
    assert sz.vane_cl(p, 10.0) == pytest.approx(p["vane"]["cl_alpha_per_rad"] * math.radians(10.0))


def test_swirl_torque_formula():
    p = layout.load_params(layout.PARAMS_PATH, DEFAULT)
    s = sz.swirl_torque(p)
    omega = p["edf"]["kv"] * p["edf"]["voltage_nominal_V"] * 2 * math.pi / 60
    assert s["omega_rad_s"] == pytest.approx(omega)
    assert s["tau_swirl_Nm"] == pytest.approx(p["edf"]["residual_swirl_torque_fraction_ASSUMED"] * p["edf"]["power_nominal_W"] / omega)


# --------------------------------------------------------------------------- #
# Barrowman
# --------------------------------------------------------------------------- #
def test_barrowman_rectangular_fins_known_case():
    # 4 rectangular fins, s = d = a = b = 100, body radius 50, no sweep:
    # C_Na = (1 + 50/150) * 16 * 1 / (1 + sqrt(1 + (2*100/200)^2)) = (4/3) * 16 / (1 + sqrt(2))
    c = bm.trapezoid_fins(4, 100.0, 100.0, 100.0, 0.0, 50.0, 100.0, x_root_le_from_tip_mm=500.0)
    assert c.cna_per_rad == pytest.approx((4.0 / 3.0) * 16.0 / (1.0 + math.sqrt(2.0)))
    assert c.cna_per_rad == pytest.approx(8.8366, abs=1e-3)
    # rectangular fin CP at quarter chord
    assert c.x_cp_from_tip_mm == pytest.approx(500.0 + 25.0)


def test_barrowman_swept_trapezoid_hand_case():
    # a = 90, b = 40, s = 70, sweep 50 (the SPEC fin), d = 120, R = 49
    a, b, s, xr, R, d = 90.0, 40.0, 70.0, 50.0, 49.0, 120.0
    l_mid = math.hypot(s, xr + b / 2 - a / 2)
    cna = (1 + R / (s + R)) * (4 * 4 * (s / d) ** 2) / (1 + math.sqrt(1 + (2 * l_mid / (a + b)) ** 2))
    x = xr * (a + 2 * b) / (3 * (a + b)) + (a + b - a * b / (a + b)) / 6
    c = bm.trapezoid_fins(4, s, a, b, xr, R, d, 0.0)
    assert c.cna_per_rad == pytest.approx(cna)
    assert c.x_cp_from_tip_mm == pytest.approx(x)


def test_barrowman_nose_cone_and_von_karman():
    L, R = 300.0, 60.0
    cone = bm.nose_from_volume(L, R, math.pi * R * R * L / 3)
    assert cone.cna_per_rad == 2.0
    assert cone.x_cp_from_tip_mm == pytest.approx(2 * L / 3)
    lay = layout.build(DEFAULT)                      # haack_c = 0 -> von Karman -> CP at L/2
    vol = bm.nose_volume(lay.nose_outer_radius, lay.nose_len)
    assert vol == pytest.approx(math.pi * lay.body_r**2 * lay.nose_len / 2, rel=2e-3)
    vk = bm.nose_from_volume(lay.nose_len, lay.body_r, vol)
    assert vk.x_cp_from_tip_mm == pytest.approx(lay.nose_len / 2, rel=2e-3)


def test_barrowman_transition_and_combine():
    same = bm.transition(100.0, 100.0, 50.0, 100.0, 200.0)
    assert same.cna_per_rad == pytest.approx(0.0)
    boat = bm.transition(120.0, 98.0, 110.0, 120.0, 0.0)
    assert boat.cna_per_rad == pytest.approx(2 * ((98 / 120) ** 2 - 1))
    assert boat.cna_per_rad < 0
    shoulder = bm.transition(98.0, 120.0, 110.0, 120.0, 0.0)
    assert shoulder.cna_per_rad == pytest.approx(-boat.cna_per_rad)
    cna, x = bm.combine([bm.CpComponent("a", 2.0, 100.0), bm.CpComponent("b", 2.0, 300.0)])
    assert (cna, x) == (4.0, 200.0)


def test_static_margin_definition(result):
    st, lay = result["stability"], layout.build(DEFAULT)
    assert st["static_margin_full_cal"] == pytest.approx((st["z_cg_full_mm"] - st["z_cp_mm"]) / lay.body_od)
    assert st["z_cg_full_mm"] == pytest.approx(result["totals"]["com_mm"][2])
    assert st["lever_arm_l_v_mm"] == pytest.approx(st["z_cg_full_mm"] - lay.vane_hinge_z)
    names = [c["name"] for c in st["components"]]
    assert names[:2] == ["nose", "fins"]
    comp = {c["name"]: c for c in st["components"]}
    cna = comp["nose"]["cna_per_rad"] + comp["fins"]["cna_per_rad"]
    z_cp = (comp["nose"]["cna_per_rad"] * comp["nose"]["z_mm"] + comp["fins"]["cna_per_rad"] * comp["fins"]["z_mm"]) / cna
    assert st["z_cp_mm"] == pytest.approx(z_cp)


# --------------------------------------------------------------------------- #
# mission / recovery / legal
# --------------------------------------------------------------------------- #
def _climb(**over):
    kw = dict(m_kg=4.0, t_max_N=80.0, n_exp=2.0, u_climb=1.0, ramp_s=0.5, tau_s=0.15, rho=1.225, cd=0.6,
              a_ref_m2=0.0113, target_m=150.0, k_coast=0.85, p_nom_W=4530.0, v_nom_V=44.4, batt_Wh=97.7)
    kw.update(over)
    return sz.climb_1d(**kw)


def test_climb_reaches_target_and_obeys_cutoff_rule():
    fl = _climb()
    assert fl["reached"] and fl["liftoff_t_s"] is not None
    assert fl["h_cutoff_m"] + 0.85 * fl["v_cutoff_mps"] ** 2 / (2 * sz.G) >= 150.0 - 1e-6
    assert fl["apogee_m"] > fl["h_cutoff_m"]
    assert fl["t_apogee_s"] > fl["t_cutoff_s"]
    assert fl["i_peak_A"] == pytest.approx(4530.0 / 44.4)               # P_nom u^3 / V at u = 1
    assert 0 < fl["energy_Wh"] < 97.7
    # the coast factor < 1 means the vehicle overshoots a little (drag is below the vacuum estimate)
    assert 0 <= fl["overshoot_m"] < 20.0


def test_climb_no_liftoff_when_thrust_below_weight():
    fl = _climb(t_max_N=30.0)
    assert not fl["reached"] and fl["liftoff_t_s"] is None
    assert "no lift-off" in fl["reason"]
    assert fl["tw_climb"] == pytest.approx(30.0 / (4.0 * sz.G))


def test_battery_check_and_c_rate():
    p = layout.load_params(layout.PARAMS_PATH, DEFAULT)
    b = sz.battery_check(p, {"energy_Wh": 10.0, "i_peak_A": 110.0})
    cap_ah = p["battery"]["capacity_mAh"] / 1000
    assert b["capacity_Wh"] == pytest.approx(cap_ah * p["edf"]["voltage_nominal_V"])
    assert b["c_rate_needed"] == pytest.approx(110.0 / cap_ah)
    assert b["remaining_pct"] == pytest.approx(100 * (1 - 10.0 / b["capacity_Wh"]))


def test_chute_round_trip():
    p = layout.load_params(layout.PARAMS_PATH, DEFAULT)
    c = sz.chute(p, 2800.0)
    rho, cd, v = p["aero"]["rho_kg_m3"], p["recovery"]["vehicle_chute_cd"], p["recovery"]["target_descent_mps"]
    assert math.sqrt(2 * 2.8 * sz.G / (rho * cd * c["area_required_m2"])) == pytest.approx(v)
    assert c["diameter_required_m"] == pytest.approx(math.sqrt(4 * c["area_required_m2"] / math.pi))
    assert c["descent_with_params_chute_mps"] == pytest.approx(
        math.sqrt(2 * 2.8 * sz.G / (rho * cd * math.pi / 4 * p["recovery"]["vehicle_chute_d_m"] ** 2)))


def test_legal_classes():
    p = layout.load_params(layout.PARAMS_PATH, DEFAULT)
    assert sz.legal(p, 200.0, 100.0)["operator_certificate"].startswith("해당 없음")
    assert sz.legal(p, 1500.0, 100.0)["operator_certificate"].startswith("4종")
    assert sz.legal(p, 4000.0, 100.0)["operator_certificate"].startswith("3종")
    assert sz.legal(p, 10000.0, 100.0)["operator_certificate"].startswith("2종")
    assert sz.legal(p, 30000.0, 100.0)["operator_certificate"].startswith("1종")
    assert sz.legal(p, 1999.0, 100.0)["aircraft_registration_required"] is False
    assert sz.legal(p, 2001.0, 100.0)["aircraft_registration_required"] is True
    lg = sz.legal(p, 4000.0, 160.0)
    assert lg["flight_approval_required_at_target"] == (p["mission"]["target_altitude_m"] > 150.0)
    assert lg["flight_approval_required_at_predicted_apogee"] is True
    assert lg["note_kr"] == p["mission"]["legal_note_kr"]


# --------------------------------------------------------------------------- #
# CAD report path + presets
# --------------------------------------------------------------------------- #
def _complete_parts(preset: str, skip: tuple[str, ...] = ()) -> dict:
    """A CAD-schema ``parts`` dict carrying every component of ``se.expected_report_parts`` (values: the
    analytic estimate), minus ``skip``."""
    est = sz.run(preset, write=False, use_cad=False)["parts"]
    p = layout.load_params(layout.PARAMS_PATH, preset)
    return {n: {k: est[n][k] for k in ("kind", "volume_mm3", "density_g_cm3", "mass_g", "com_mm", "inertia_about_com_kg_m2")}
            for n in se.expected_report_parts(layout.Layout(p), p) if n not in skip}


def test_cad_report_overrides_estimate(tmp_path):
    sha = hashlib.sha256(open(layout.PARAMS_PATH, "rb").read()).hexdigest()
    parts = _complete_parts(DEFAULT, skip=tuple(f"vane_{i}" for i in range(1, 5)))
    parts["propulsion_module"] = {"kind": "printed", "volume_mm3": 400000.0, "density_g_cm3": 1.31, "mass_g": 999.0,
                                  "com_mm": [0.0, 0.0, 100.0],
                                  "inertia_about_com_kg_m2": [[1e-3, 0, 0], [0, 1e-3, 0], [0, 0, 5e-4]]}
    parts["vane"] = {"kind": "printed", "volume_mm3": 4000.0, "density_g_cm3": 1.31, "mass_g": 5.24,
                     "com_mm": [28.0, 0.0, 31.0], "inertia_about_com_kg_m2": [[0, 0, 0], [0, 0, 0], [0, 0, 0]]}
    parts["intake_module"]["mass_g"] = 123.0
    rep = {"generator": "cadquery", "generator_version": "test", "preset": DEFAULT, "params_sha256": sha,
           "parts": parts, "totals": {"mass_g": 1.0}, "derived": {}, "warnings": []}
    path = tmp_path / "cadquery_report.json"
    path.write_text(json.dumps(rep), encoding="utf-8")
    r = sz.run(DEFAULT, cad_report=str(path), out_dir=str(tmp_path))
    pm = r["parts"]["propulsion_module"]
    assert pm["source"] == "CAD:cadquery" and pm["mass_g"] == 999.0 and pm["com_mm"] == [0.0, 0.0, 100.0]
    assert r["parts"]["vane_3"]["source"] == "CAD:cadquery" and r["parts"]["vane_3"]["mass_g"] == 5.24
    est_x = sz.run(DEFAULT, write=False, use_cad=False)["parts"]["vane_3"]["com_mm"][0]
    assert r["parts"]["vane_3"]["com_mm"][0] == pytest.approx(est_x, abs=1e-6)   # COM kept from the estimate (base-name match)
    assert est_x < 0                                                               # vane 3 sits at theta = 180 deg
    assert r["parts"]["intake_module"]["source"] == "CAD:cadquery" and r["parts"]["intake_module"]["mass_g"] == 123.0
    assert r["parts"]["spring_piston_hardware_balance"]["source"] == "params"      # budget-only item, never in a report
    assert r["structure_source"].startswith("CAD report")
    assert any("differs from the budget sum" in w for w in r["warnings"])
    # a stale report (different params hash) is used but flagged
    rep["params_sha256"] = "0" * 64
    path.write_text(json.dumps(rep), encoding="utf-8")
    r2 = sz.run(DEFAULT, cad_report=str(path), out_dir=str(tmp_path))
    assert any("different params.json" in w for w in r2["warnings"])
    with pytest.raises(FileNotFoundError):
        sz.run(DEFAULT, cad_report=str(tmp_path / "missing.json"), out_dir=str(tmp_path))


def test_find_cad_report_per_preset(tmp_path, monkeypatch):
    """An alt preset reads <generator>_report_<preset>.json first; a plain report built for another preset is
    ignored with a warning; a report from another params.json is used but flagged (sim/vehicle.py mirrors this)."""
    alt = ALTS[0]
    exp = tmp_path / "cad" / "exports"
    exp.mkdir(parents=True)
    monkeypatch.setattr(sz, "ROOT", str(tmp_path))

    parts = {pr: _complete_parts(pr) for pr in (DEFAULT, alt)}

    def write(name: str, preset: str, sha: str) -> None:
        (exp / name).write_text(json.dumps({"generator": "cadquery", "preset": preset, "params_sha256": sha,
                                            "parts": parts[preset]}), encoding="utf-8")

    write("cadquery_report.json", DEFAULT, "a" * 64)
    w: list[str] = []
    assert sz.find_cad_report(None, alt, "a" * 64, w, DEFAULT) is None             # only the default preset's report
    assert any("cadquery_report.json" in m and "ignored" in m for m in w)
    w = []
    rep = sz.find_cad_report(None, DEFAULT, "a" * 64, w, DEFAULT)
    assert rep is not None and rep["_path"].endswith("cadquery_report.json") and w == []
    write(f"cadquery_report_{alt}.json", alt, "b" * 64)
    w = []
    rep = sz.find_cad_report(None, alt, "a" * 64, w, DEFAULT)
    assert rep is not None and rep["_path"].endswith(f"cadquery_report_{alt}.json")
    assert len(w) == 1 and "different params.json" in w[0]
    write("fusion360_report.json", alt, "a" * 64)                                   # Fusion writes no suffix
    (exp / f"cadquery_report_{alt}.json").unlink()
    w = []
    rep = sz.find_cad_report(None, alt, "a" * 64, w, DEFAULT)
    assert rep is not None and rep["_path"].endswith("fusion360_report.json")
    assert any("cadquery_report.json" in m and "ignored" in m for m in w)


@pytest.mark.parametrize("preset,suffix", [(DEFAULT, "")] + [(a, f"_{a}") for a in ALTS])
def test_both_presets_run_and_write(tmp_path, preset, suffix):
    r = sz.run(preset, out_dir=str(tmp_path))
    assert r["preset"] == preset
    jp, mp = tmp_path / f"sizing_report{suffix}.json", tmp_path / f"sizing_report{suffix}.md"
    assert jp.exists() and mp.exists()
    data = json.loads(jp.read_text(encoding="utf-8"))
    assert data["totals"]["mass_g"] == pytest.approx(r["totals"]["mass_g"])
    assert data["checks"]["C12"]["verdict"] in ("PASS", "WARN", "FAIL")
    assert data["checks"]["C13"]["verdict"] in ("PASS", "WARN", "FAIL")
    assert set(data["totals"]["post_ejection"]["excluded"]) == set(sz.EJECTED)
    assert len(data["totals"]["inertia_about_com_kg_m2"]) == 3
    text = mp.read_text(encoding="utf-8")
    assert "## 1. 질량 예산" in text and "## 7. 국내 법규" in text
    assert "\r" not in text
    main = data["mission"]["scenarios"][data["mission"]["main_scenario"]]
    assert main["target_m"] == data["legal"]["target_altitude_m"]
    assert 1000 < data["totals"]["mass_g"] < 8000


# --------------------------------------------------------------------------- #
# v0.5 tab mount (SPEC s3.7, s5.1, s6) in the analytic estimate
# --------------------------------------------------------------------------- #
TAB_LABELS = ("cylindrical skin", "liner", "screw lug", "keyway", "tab screw", "wire slot", "wire bulge",
              "wire channel", "wire shelf notch", "bulkhead wire hole", "motor", "tab", "tab slot")


def _est(preset):
    p = layout.load_params(layout.PARAMS_PATH, preset)
    lay = layout.Layout(p)
    return p, lay, se.StructureEstimator(lay, p)


def _est_screws(preset):
    """wire09c: the default params use the clock mount (propulsion_module.tab_mount 'clock_keyways'); the analytic
    estimator models the v0.5 SCREW tab mount (lugs, tab-screw holes, wire slot), so its hand-calc tests run on the
    same params with tab_mount 'screws' (a supported mode)."""
    p = layout.load_params(layout.PARAMS_PATH, preset)
    if "propulsion_module" in p:
        p["propulsion_module"]["tab_mount"] = "screws"
    lay = layout.Layout(p)
    return p, lay, se.StructureEstimator(lay, p)


def test_box_annulus_area_hand_cases():
    area, uc = se.box_annulus_area(10.0, 20.0, 1.0, 5.0, 30.0)          # box wholly inside the annulus
    assert area == pytest.approx(20.0, rel=1e-12) and uc == pytest.approx(15.0, rel=1e-12)
    R = 30.0
    area, uc = se.box_annulus_area(0.0, R, R, 0.0, R, n=4000)           # half disc
    assert area == pytest.approx(math.pi * R * R / 2, rel=1e-4)
    assert uc == pytest.approx(4 * R / (3 * math.pi), rel=1e-3)
    area, _ = se.box_annulus_area(0.0, 100.0, 1e-3, 40.0, 50.0)         # thin strip -> radial thickness
    assert area == pytest.approx(2e-3 * 10.0, rel=1e-6)
    area, _ = se.box_annulus_area(R - 5.0, R + 5.0, 2.0, R, math.inf)    # outside a circle: > half the box
    assert 20.0 < area < 21.0
    assert se.box_annulus_area(0.0, 10.0, 1.0, 20.0, 30.0)[0] == 0.0    # no overlap


def test_tangential_bore_in_annulus_hand_cases():
    rho, rs = 1.5, 50.0
    vol, uc, tc = se.tangential_bore_in_annulus(rs, rho, 2.0, 8.0, 40.0, 70.0)    # wholly inside: exact cylinder
    assert vol == pytest.approx(math.pi * rho * rho * 6.0, rel=1e-9)
    assert tc == pytest.approx(5.0, rel=1e-9) and uc == pytest.approx(rs, abs=1e-9)
    vol_n, _, tc_n = se.tangential_bore_in_annulus(rs, rho, -8.0, -2.0, 40.0, 70.0)  # mirrored t range
    assert vol_n == pytest.approx(vol, rel=1e-9) and tc_n == pytest.approx(-5.0, rel=1e-9)
    assert se.tangential_bore_in_annulus(rs, rho, 2.0, 8.0, 60.0, 70.0)[0] == 0.0   # never reaches r = 60
    # running far past the outer circle removes nothing more: the volume is set by where each line leaves r = b
    v1 = se.tangential_bore_in_annulus(rs, rho, 0.0, 40.0, 0.0, 60.0)[0]
    v2 = se.tangential_bore_in_annulus(rs, rho, 0.0, 400.0, 0.0, 60.0)[0]
    assert v1 == v2
    f, f2 = math.sqrt(3600 - rs * rs), -3600 / (3600 - rs * rs) ** 1.5
    assert v1 == pytest.approx(math.pi * rho * rho * (f + f2 * rho * rho / 8), rel=1e-4)
    # the inner annulus edge: from t = sqrt(a^2 - u^2) on (u < a), all of it when u > a
    va = se.tangential_bore_in_annulus(rs, rho, 0.0, 10.0, rs, 70.0)[0]
    assert va == pytest.approx(math.pi * rho * rho * 10.0 - se.tangential_bore_in_annulus(rs, rho, 0.0, 10.0, 0.0, rs)[0])


def test_sector_z_matches_tube_and_quarter_centroid():
    full = se.sector_z(40.0, 50.0, 0.0, 360.0, 0.0, 10.0)
    tube = se.tube_z(40.0, 50.0, 0.0, 10.0)
    assert full.volume_mm3 == pytest.approx(tube.volume_mm3, rel=1e-12)
    assert full.com_mm == pytest.approx((0.0, 0.0, 5.0), abs=1e-9)
    assert full.k_mm2 == pytest.approx(tube.k_mm2, rel=1e-9)
    q = se.sector_z(40.0, 50.0, 0.0, 90.0, 0.0, 10.0)
    c = 4.0 / (3.0 * math.pi) * (50.0**3 - 40.0**3) / (50.0**2 - 40.0**2)
    assert q.volume_mm3 == pytest.approx(tube.volume_mm3 / 4, rel=1e-12)
    assert q.com_mm[0] == pytest.approx(c) and q.com_mm[1] == pytest.approx(c)
    assert q.k_mm2[2] == pytest.approx((40.0**2 + 50.0**2) / 2 - 2 * c * c)   # polar moment about the centroid


def test_lip_clamp_estimate_has_no_tab_features():
    """KAIST (lip_clamp) keeps the v0.4 estimate: no v0.5 solid is added anywhere."""
    for preset in ALTS:
        p, lay, est = _est(preset)
        if lay.tabs:
            continue
        assert lay.cone_z1 == lay.z_prop_spigot_bottom
        for fn in (est.propulsion_module, est.intake_module):
            labels = {s.label for s in fn()}
            assert not {lb for lb in labels if lb.startswith(TAB_LABELS)}, labels
        edf = se.edf_unit_solids(lay, p["edf"])
        assert [s.label for s in edf] == ["lip", "housing", "hub"]
        assert edf[0].volume_mm3 == pytest.approx(math.pi * (p["edf"]["lip_od"] / 2) ** 2 * lay.edf_lip_t)   # full disc


def _tab_volumes(est) -> dict[str, float]:
    vol: dict[str, float] = {}
    for s in est.tab_mount_solids() + est.wire_channel_solids():
        key = s.label.split(" (")[0]
        vol[key] = vol.get(key, 0.0) + s.volume_mm3
    return vol


def _open_groove_hand_calc(L, vol: dict[str, float]) -> None:
    """v0.9 open motor-lead groove (intake wire_channel.open) against hand volumes: the two side walls and the roof wedge
    (shoelace area of wire_roof_rz x the bulge width) exactly, the groove through skirt / lip-pocket ring / shelf as flat
    slabs (curvature < 3 %), the gate exactly (F(a) = int_{-h}^{h} sqrt(a^2 - v^2) dv: below the groove top the shelf from
    its bore to wire_ch_r0, above it the window_wall_t band from r_band_in to the body circle), the bulkhead hole box."""
    hw, sw = L.wire_ch_w / 2, L.wire_side_wall
    assert vol["groove side wall"] == pytest.approx(2 * sw * (L.wire_ch_r1 - L.body_r) * (L.wire_ch_z1 - L.z_intake_bottom),
                                                    rel=1e-12)
    pts = [(float(u), float(z)) for u, z in L.wire_roof_rz]
    shoelace = 0.5 * abs(sum(u0 * z1 - u1 * z0 for (u0, z0), (u1, z1) in zip(pts, pts[1:] + pts[:1])))
    assert vol["groove roof wedge"] == pytest.approx(shoelace * 2 * (hw + sw), rel=1e-12)
    R, ri, r0 = L.body_r, L.body_id / 2, L.wire_ch_r0

    def plan(a: float, n: int = 20000) -> float:      # |v| <= hw, u >= r0, a <= r <= R (22 mm wide: the sag is ~1 mm)
        return sum(max(0.0, math.sqrt(R * R - v * v) - max(math.sqrt(max(a * a - v * v, 0.0)), r0)) * 2 * hw / n
                   for v in (-hw + (i + 0.5) * 2 * hw / n for i in range(n)))
    groove = (plan(ri) * (L.z_intake_skirt_top - L.z_intake_bottom)
              + plan(L.lip_pocket_id / 2) * (L.z_lip_pocket_top - L.z_intake_skirt_top)
              + plan(L.shelf_id / 2) * (L.wire_groove_z1 - L.z_lip_pocket_top))
    assert -vol["open groove"] == pytest.approx(groove, rel=1e-4)
    h = L.wire_gate_w / 2

    def F(a: float) -> float:
        return h * math.sqrt(a * a - h * h) + a * a * math.asin(h / a)
    a = L.shelf_id / 2
    assert L.wire_notch_r0 < math.sqrt(a * a - h * h) and math.hypot(r0, h) < R      # the box spans the shelf ring fully
    zg = L.wire_groove_z1
    gate = ((L.wire_gate_w * r0 - F(a)) * (zg - L.wire_notch_z0)
            + (F(R) - F(L.r_band_in)) * (L.wire_ch_z1 - zg))
    assert L.z_lip_pocket_top <= L.wire_notch_z0 < zg < L.wire_ch_z1 <= L.z_window_bottom
    assert -vol["gate"] == pytest.approx(gate, rel=1e-4)
    t_bh = L.z_intake_bulkhead_top - L.z_intake_bulkhead_bottom
    assert math.hypot(L.wire_hole_r1, hw) < R
    assert -vol["bulkhead wire hole"] == pytest.approx(L.wire_ch_w * (L.wire_hole_r1 - L.wire_hole_r0) * t_bh, rel=1e-9)


def test_tab_mount_feature_volumes_hand_calc():
    p, L, est = _est_screws(DEFAULT)
    if not L.tabs:
        pytest.skip("default preset is not a tab mount")
    vol = _tab_volumes(est)
    r_skin = L.body_r - L.cone_wall
    assert vol["cylindrical skin"] == pytest.approx(math.pi * (L.body_r**2 - r_skin**2) * (L.z_prop_spigot_bottom - L.cone_z1))
    # thin liner: duct tube + the top ring's part outside the tube and inside the sleeve bore (cavity behind the tube)
    assert vol["liner tube"] == pytest.approx(math.pi * (L.liner_r_out**2 - L.duct_r**2) * (L.z_liner_top - L.z_edf_shoulder))
    assert vol["liner ring"] == pytest.approx(math.pi * (L.edf_bore_r**2 - L.liner_r_out**2) * (L.z_liner_top - L.z_liner_ring_bottom))
    assert L.z_liner_ring_bottom > L.z_edf_shoulder + L.liner_wall
    # one lug per tab and boss_z_ranges entry; the lowest reaches below keyway_z0 (keyway floor)
    sector = 0.5 * math.radians(L.block_theta_far + L.block_theta_head) * (r_skin**2 - L.sleeve_r**2)
    lug_h = sum(z1 - z0 for z0, z1 in L.boss_z_ranges)
    assert len(L.boss_z_ranges) >= 2 and L.boss_z_ranges[0][0] < L.keyway_z0
    assert vol["screw lug"] == pytest.approx(L.tab_count * sector * lug_h)
    # holes: clearance and tap lie wholly in material -> pi/4 d^2 x length; the counterbore removes material
    # only up to the outer skin: per line u, sqrt(R^2 - u^2) - t_head; 2nd-order mean over the disc
    n_screw = L.tab_count * len(L.tab_screw_zs)
    clear = math.pi / 4 * L.tab_screw_d_clear**2 * (L.tab_screw_t_head - L.tab_screw_t_keyway)
    tap = math.pi / 4 * L.tab_screw_d_tap**2 * (-L.tab_screw_t_keyway - L.tab_screw_t_tap_end)
    R, rs, rc = L.body_r, L.tab_screw_r, L.tab_screw_d_cbore / 2
    f, f2 = math.sqrt(R * R - rs * rs), -R * R / (R * R - rs * rs) ** 1.5
    cbore = math.pi * rc * rc * (f + f2 * rc * rc / 8 - L.tab_screw_t_head)
    assert -vol["tab screw clearance hole"] == pytest.approx(n_screw * clear, rel=1e-9)
    assert -vol["tab screw tap hole"] == pytest.approx(n_screw * tap, rel=1e-9)
    assert -vol["tab screw counterbore"] == pytest.approx(n_screw * cbore, rel=1e-3)
    assert -vol["tab screw counterbore"] < n_screw * math.pi * rc * rc * (L.tab_screw_t_out - L.tab_screw_t_head)
    # keyway: thin box -> flat-slab formula (sleeve, the lugs' Z overlap, spigot ring) within 1 %
    lug_kw = sum(max(0.0, min(z1, L.keyway_z1) - max(z0, L.keyway_z0)) for z0, z1 in L.boss_z_ranges)
    kw = L.keyway_w * ((L.sleeve_r - L.edf_bore_r) * (L.z_prop_spigot_bottom - L.keyway_z0)
                       + (L.keyway_r_out - L.sleeve_r) * lug_kw
                       + (min(L.keyway_r_out, L.spigot_od / 2) - L.edf_bore_r) * (L.z_prop_spigot_top - L.z_prop_spigot_bottom))
    assert -vol["keyway"] == pytest.approx(L.tab_count * kw, rel=0.01)
    # wire slot: (a) through the sleeve only - its corners stay inside the skin; (b) through the spigot ring
    assert math.hypot(L.wire_slot_lower_r_out, L.wire_slot_w / 2) < r_skin
    # v0.9: the slot is 22 mm wide, so the flat slab w x (b - a) is 7 % off the thin sleeve's annulus (the chord through
    # a 1.6 mm wall at |v| = 11); the hand value is the box u [u0, u1], |v| <= w/2 inside a <= r <= b by midpoint strips
    def slot_plan(a: float, b: float, u0: float, u1: float, n: int = 20000) -> float:
        h, area = L.wire_slot_w / 2, 0.0
        for i in range(n):
            v = -h + (i + 0.5) * 2 * h / n
            area += max(0.0, min(math.sqrt(b * b - v * v), u1) - max(math.sqrt(max(a * a - v * v, 0.0)), u0)) * 2 * h / n
        return area
    ws_a = slot_plan(L.edf_bore_r, L.sleeve_r, L.wire_slot_r_in, L.wire_slot_lower_r_out) * (L.z_prop_spigot_bottom - L.wire_slot_z0)
    ws_b = slot_plan(L.edf_bore_r, L.spigot_od / 2, L.wire_slot_r_in, L.wire_slot_r_out) * (L.z_prop_spigot_top - L.z_prop_spigot_bottom)
    assert -vol["wire slot lower"] == pytest.approx(ws_a, rel=1e-4)
    assert -vol["wire slot upper"] == pytest.approx(ws_b, rel=1e-4)
    if getattr(L, "wire_ch_open", False):
        _open_groove_hand_calc(L, vol)
        return
    t_bh = L.z_intake_bulkhead_top - L.z_intake_bulkhead_bottom
    assert -vol["bulkhead wire hole"] == pytest.approx(L.wire_ch_w * (L.wire_hole_r1 - L.wire_hole_r0) * t_bh, rel=1e-9)
    bulge_flat = L.wire_bulge_w * (L.wire_bulge_r_out - L.body_r) * (L.wire_bulge_z1 - L.wire_bulge_z0)
    assert bulge_flat < vol["wire bulge"] < 1.05 * bulge_flat       # plus the sliver between chord and circle
    # channel: flat slabs through skirt, pocket ring, shelf, ring below the windows (v0.6.1: the window_wall_t band
    # from r_band_in) and the bulge (curvature < 3 %)
    hz = lambda z0, z1: max(0.0, min(z1, L.wire_ch_z1) - max(z0, L.wire_ch_z0))   # noqa: E731
    ri = L.body_id / 2
    ch = L.wire_ch_w * ((L.body_r - max(ri, L.wire_ch_r0)) * hz(L.z_intake_bottom, L.z_intake_skirt_top)
                        + (L.body_r - L.wire_ch_r0) * hz(L.z_intake_skirt_top, L.z_shelf_top)
                        + (L.body_r - max(L.r_band_in, L.wire_ch_r0)) * hz(L.z_shelf_top, L.z_window_bottom)
                        + (L.wire_ch_r1 - L.body_r) * hz(L.wire_bulge_z0, L.wire_bulge_z1))
    assert -vol["wire channel"] == pytest.approx(ch, rel=0.03)
    assert L.wire_bulge_z0 < L.wire_ch_z0 and L.wire_ch_z1 < L.wire_bulge_z1   # bulge closes the channel
    # shelf notch: from the bore (circle r = a) out to wire_ch_r0 (beyond it: channel void), through the shelf
    # (a = shelf_id/2, Z [wire_notch_z0, z_shelf_top]) and, v0.6.1, through the window_wall_t band above it (a = r_band_in,
    # Z [z_shelf_top, wire_ch_z1]): plan = w x wire_ch_r0 - int_{-h}^{h} sqrt(a^2 - v^2) dv for each
    h = L.wire_ch_w / 2

    def plan(a: float) -> float:
        return L.wire_ch_w * L.wire_ch_r0 - (h * math.sqrt(a * a - h * h) + a * a * math.asin(h / a))
    a = L.shelf_id / 2
    assert L.wire_notch_r0 < math.sqrt(a * a - h * h)                 # the notch opens the whole shelf bore edge
    assert math.sqrt(L.r_band_in ** 2 - h * h) < L.wire_ch_r0 < L.body_r and L.z_shelf_top < L.wire_ch_z1
    notch = plan(a) * (L.z_shelf_top - L.wire_notch_z0) + plan(L.r_band_in) * (L.wire_ch_z1 - L.z_shelf_top)
    assert -vol["wire shelf notch"] == pytest.approx(notch, rel=1e-4)
    # v0.5d floor wedge, unioned after the cuts: new material = its part inside the void (u <= wire_ch_r1), a right
    # triangle with legs wire_ch_r1 - body_id/2 (45 deg floor from the skirt's inner face) over the channel width
    leg = L.wire_ch_r1 - ri
    assert vol["wire channel floor wedge"] == pytest.approx(L.wire_ch_w * leg * leg / 2, rel=1e-12)


def test_v05d_contract_in_the_estimate():
    """SPEC v0.5d: tabs at 90 / 270 with the leads at 0 deg; channel void Z [wire_ch_z0, wire_ch_z1], r [57.95, 63.95];
    floor wedge after the cuts; bulge from 177.65 out to 65.15; pocket ID 113.5 - the estimate takes every one of them
    from Layout (nothing hard-codes 0 / 90 / 180)."""
    p, L, est = _est_screws(DEFAULT)
    if not L.tabs:
        pytest.skip("default preset is not a tab mount")
    assert L.tab_thetas == [90.0, 270.0] and L.wire_theta == 0.0
    if getattr(L, "wire_ch_open", False):
        # v0.9 (decision 3): the closed channel became the open groove - r [57.95, 68.95] (11 deep, outside r1 the
        # printed cover), side walls / roof out to r1, no floor wedge; the gate drops into the lip pocket's top
        assert (L.wire_ch_r0, L.wire_ch_r1, L.wire_bulge_r_out, L.lip_pocket_id) == pytest.approx((57.95, 68.95, 68.95, 113.5))
        assert L.wire_groove_z1 == pytest.approx(L.z_shelf_top) and L.wire_ch_floor_rz is None
        labels = [s.label for s in est.wire_channel_solids()]
        assert not any("floor wedge" in x for x in labels)
        assert labels.count("groove side wall") == 2 and "groove roof wedge" in labels
        assert any(x.startswith("open groove (") for x in labels) and any(x.startswith("gate (") for x in labels)
        return
    assert (L.wire_ch_r0, L.wire_ch_r1, L.wire_bulge_r_out, L.lip_pocket_id) == pytest.approx((57.95, 63.95, 65.15, 113.5))
    assert (L.wire_ch_z0, L.wire_bulge_z0) == pytest.approx((178.85, 177.65))
    sol = est.wire_channel_solids()
    wedge = [s for s in sol if s.label == "wire channel floor wedge"]
    assert len(wedge) == 1 and sol[-1] is wedge[0]                      # unioned after every cut
    w = wedge[0]
    assert w.com_mm[1] == pytest.approx(0.0, abs=1e-9) and L.body_id / 2 < w.com_mm[0] < L.wire_ch_r1   # at wire_theta = 0
    assert L.wire_ch_z0 < w.com_mm[2] < L.wire_gate_lo_z0
    ch = [s for s in sol if s.label.startswith("wire channel (")]
    assert ch and all(s.com_mm[0] > 0 and abs(s.com_mm[1]) < 1e-6 for s in ch)
    zmin = min(s.com_mm[2] for s in ch if "skirt" in s.label)
    assert zmin == pytest.approx((L.wire_ch_z0 + L.z_intake_skirt_top) / 2)   # the skirt cut starts at wire_ch_z0
    # the keyways sit at the tabs (90 / 270 deg), the wire slot at 0 deg
    for s in est.tab_mount_solids():
        if s.label.startswith("keyway"):
            assert abs(s.com_mm[0]) < 1e-6 and abs(s.com_mm[1]) > L.edf_bore_r
        if s.label.startswith("wire slot"):
            assert abs(s.com_mm[1]) < 1e-6 and s.com_mm[0] > L.edf_bore_r


@pytest.mark.parametrize("preset", [DEFAULT] + ALTS)
def test_tray_stop_blocks_both_presets(preset):
    """SPEC s5.2 v0.5d (both presets): one block per rail slot between the two ribs, the ribs' radial extent,
    Z [z_tray_stop_bottom, z_tray_bottom]; without tray_stop_h the avionics module is the v0.4 one exactly."""
    p, L, est = _est(preset)
    blocks = est.tray_stop_blocks()
    assert len(blocks) == 2 and L.tray_stop_h > 0
    h = L.z_tray_bottom - L.z_tray_stop_bottom
    assert h == pytest.approx(L.tray_stop_h) and L.z_tray_stop_bottom >= L.z_rail_bottom - 1e-9
    for b in blocks:
        assert b.volume_mm3 == pytest.approx(L.rail_gap * L.rail_depth * h)
        assert b.com_mm[0] == pytest.approx(L.tray_x)
        assert abs(b.com_mm[1]) == pytest.approx(L.rail_y - L.rail_depth / 2)
        assert b.com_mm[2] == pytest.approx(L.z_tray_bottom - h / 2)
    assert sorted(math.copysign(1.0, b.com_mm[1]) for b in blocks) == [-1.0, 1.0]
    p0 = json.loads(json.dumps(p))
    del p0["avionics_module"]["tray_stop_h"]
    L0 = layout.Layout(p0)
    a1 = est.estimate_all()["avionics_module"]
    a0 = se.StructureEstimator(L0, p0).estimate_all()["avionics_module"]
    rho = p["printer"]["density_g_cm3"] * 1e-3
    assert a1.mass_solid_g - a0.mass_solid_g == pytest.approx(2 * L.rail_gap * L.rail_depth * h * rho, rel=1e-9)


# v0.6.1 intake band (SPEC s5.1): (band volume over the margin rings, strut volume) mm^3 per preset, by hand -
# BUDGET: ring pi (60^2 - 57^2) x (5 + 4), struts 4 x 0.5 x (18/60) x (60^2 - 57^2) x 117;
# KAIST:  ring pi (60^2 - 57^2) x (4 + 4), struts 4 x 0.5 x (18/60) x (60^2 - 57^2) x 67
V061_BAND = {"BUDGET_90mm_6S": (math.pi * 351.0 * 9.0, 4 * 0.15 * 351.0 * 117.0),
             "KAIST_90mm_12S": (math.pi * 351.0 * 8.0, 4 * 0.15 * 351.0 * 67.0)}


@pytest.mark.parametrize("preset", [DEFAULT] + ALTS)
def test_intake_band_v061(preset):
    """SPEC s5.1 v0.6.1 (both presets): the windowed band is r_band_in .. body_r (window_wall_t 3.0, body OD 120 ->
    r_band_in 57.0), the struts are radial-sided sectors of 18 mm at body_r; the estimate's band solids are the hand
    volumes above (explicit numbers) and it differs from the same params with the v0.6.0 band (1.2 mm wall, 14 mm
    struts, slab windows modelled as the same strut fraction) by exactly the extra band material."""
    p, L, est = _est(preset)
    assert (L.body_r, L.window_wall_t, L.r_band_in, L.strut_w) == pytest.approx((60.0, 3.0, 57.0, 18.0))
    assert L.strut_angle_deg == pytest.approx(math.degrees(0.3)) and L.strut_w_in == pytest.approx(17.1)
    rings, struts = V061_BAND[preset]
    sol = {s.label: s for s in est.intake_module()}
    assert sol["ring below windows"].volume_mm3 + sol["ring above windows"].volume_mm3 == pytest.approx(rings, rel=1e-12)
    assert sol["struts"].volume_mm3 == pytest.approx(struts, rel=1e-12)
    assert sol["struts"].com_mm[2] == pytest.approx(0.5 * (L.z_window_bottom + L.z_window_top))
    # every band solid is printed at the fill of the 3.0 mm band (5 wall loops x 0.42 = 2.1 per side: solid)
    assert {sol[k].fill for k in ("ring below windows", "struts", "ring above windows")} == {est.fill_factor(3.0)} == {1.0}
    if not L.tabs:   # lip_clamp: nothing else of the intake estimate depends on the band (v0.6.2: and the gable)
        p0 = json.loads(json.dumps(p))
        p0["intake_module"].update(window_wall_t=p0["body"]["wall"], strut_w=14.0)
        del p0["intake_module"]["window_bottom_end"]                   # the v0.6.0 band had square windows
        L0 = layout.Layout(p0)
        old = {s.label: s.volume_mm3 for s in se.StructureEstimator(L0, p0).intake_module()}
        new = {k: s.volume_mm3 for k, s in sol.items()}
        assert set(new) - set(old) == {"window gussets"} and set(old) <= set(new)
        assert {k for k in old if abs(new[k] - old[k]) > 1e-9} == {"ring below windows", "struts", "ring above windows"}
        ri = L.body_id / 2
        v0 = (math.pi * (60.0 ** 2 - ri ** 2) * (L.z_window_bottom - L.z_shelf_top + L.z_intake_bulkhead_bottom - L.z_window_top)
              + 4 * 0.5 * (14.0 / 60.0) * (60.0 ** 2 - ri ** 2) * L.window_h)
        assert sum(new.values()) - sum(old.values()) == pytest.approx(rings + struts + L.gusset_volume_mm3 - v0, rel=1e-9)


@pytest.mark.parametrize("preset", [DEFAULT] + ALTS)
def test_window_gussets_v062(preset):
    """v0.6.2 (intake_module.window_bottom_end 45 deg / 20 mm, both presets): Layout's gusset volume and centroid equal
    an independent midpoint quadrature of the gusset height max(0, r |sin(phi)| - 10) x tan(45 deg) over the band
    r 57..60 and each window sector phi in [-a, a], a = (pi/2 - 18/60) / 2 (explicit numbers); the estimator's
    'window gussets' solid carries that volume on the axis at that height, at the band's fill."""
    import numpy as np
    p, L, est = _est(preset)
    wb = p["intake_module"]["window_bottom_end"]
    assert (wb["slope_deg"], wb["bridge_w"]) == (45.0, 20.0) and L.has_window_gable
    a = 0.5 * (math.pi / 2 - 18.0 / 60.0)
    assert math.radians(L.window_half_angle_deg) == pytest.approx(a, rel=1e-12)
    nr, nphi = 300, 6000
    r = 57.0 + (np.arange(nr) + 0.5) * 3.0 / nr
    phi = -a + (np.arange(nphi) + 0.5) * 2 * a / nphi
    R, PHI = np.meshgrid(r, phi, indexing="ij")
    h = np.maximum(0.0, R * np.abs(np.sin(PHI)) - 10.0) * 1.0
    dA = (3.0 / nr) * (2 * a / nphi)
    v_w = float(np.sum(h * R) * dA)
    mz_w = float(np.sum(0.5 * h * h * R) * dA)
    assert L.gusset_volume_per_window_mm3 == pytest.approx(v_w, rel=2e-6)
    assert L.gusset_volume_mm3 == pytest.approx(4 * v_w, rel=2e-6)
    assert L.gusset_centroid_z - L.z_window_bottom == pytest.approx(mz_w / v_w, rel=2e-5)
    # gable heights at the struts: (r sin a - 10) tan 45 at r_band_in / body_r
    assert (L.gable_h_inner, L.gable_h_outer) == pytest.approx((57.0 * math.sin(a) - 10.0, 60.0 * math.sin(a) - 10.0))
    g = {s.label: s for s in est.intake_module()}["window gussets"]
    assert g.volume_mm3 == pytest.approx(L.gusset_volume_mm3, rel=1e-12)
    assert abs(g.com_mm[0]) < 1e-9 and abs(g.com_mm[1]) < 1e-9 and g.com_mm[2] == pytest.approx(L.gusset_centroid_z)
    assert g.fill == est.fill_factor(L.window_wall_t)
    # without the gable key the estimate has no gussets and the intake is lighter by exactly that material
    p0 = json.loads(json.dumps(p))
    del p0["intake_module"]["window_bottom_end"]
    if getattr(L, "wire_ch_open", False):
        # v0.9: the open motor-lead groove's roof wedge rises above the plain window bottom on the strut beside the
        # window; without the gable the window cut would bite into it - Layout refuses that combination
        with pytest.raises(ValueError, match="reaches a window opening"):
            layout.Layout(p0)
        return
    L0 = layout.Layout(p0)
    assert not L0.has_window_gable and "window gussets" not in {s.label for s in se.StructureEstimator(L0, p0).intake_module()}
    a1 = est.estimate_all()["intake_module"]
    a0 = se.StructureEstimator(L0, p0).estimate_all()["intake_module"]
    assert a1.mass_solid_g - a0.mass_solid_g == pytest.approx(L.gusset_volume_mm3 * p["printer"]["density_g_cm3"] * 1e-3,
                                                              rel=1e-9)


@pytest.mark.parametrize("preset", [DEFAULT] + ALTS)
def test_radio_log_placeholder_v062(preset):
    """v0.6.2: the ``radio_log`` placeholder is the Layout box (30 x 20 x 15 at r 40, theta -90, top at the payload
    bulkhead underside) with electronics.radio_log.mass_g in the avionics group; misc_electronics keeps its box with the
    lighter params mass (BUDGET 40 g, KAIST 100 g: total electronics mass of KAIST unchanged); without the key there is no
    placeholder."""
    p, L, _ = _est(preset)
    assert L.has_radio_log
    ph = se.placeholder_parts(L, p)["radio_log"]
    assert ph.kind == "placeholder" and ph.mass_g == p["electronics"]["radio_log"]["mass_g"] == 20.0
    assert (L.radio_log_x, L.radio_log_y) == (pytest.approx((-15.0, 15.0)), pytest.approx((-50.0, -30.0)))
    assert L.radio_log_z == pytest.approx((L.z_payload_bulkhead_bottom - 15.0, L.z_payload_bulkhead_bottom))
    assert ph.volume_mm3 == pytest.approx(30.0 * 20.0 * 15.0)
    assert ph.com_mm == pytest.approx((0.0, -40.0, L.z_payload_bulkhead_bottom - 7.5))
    assert sz.group_of("radio_log", "placeholder") == "avionics"
    assert se.placeholder_parts(L, p)["misc_electronics"].mass_g == p["electronics"]["misc"]["mass_g"] == (
        40.0 if preset == DEFAULT else 100.0)
    p0 = json.loads(json.dumps(p))
    del p0["electronics"]["radio_log"]
    L0 = layout.Layout(p0)
    assert not L0.has_radio_log and "radio_log" not in se.placeholder_parts(L0, p0)


@pytest.mark.parametrize("preset", [DEFAULT] + ALTS)
def test_nav_and_fcu_placeholders_v063(preset):
    """v0.6.3: ``nav`` is the v0.6.2 ``fc`` box on the tray (+x face, ESC top + 10) with electronics.nav.mass_g; ``fcu`` is
    the Layout box flat on the intake bulkhead (50 x 50 x 15 centred, foam_t above its top face) with electronics.fcu.mass_g,
    both in the avionics group; without electronics.fcu there is no ``fcu`` placeholder and the servo-lead lump (tab
    mount) is heavier and higher - its three bay legs run to the NAV board on the tray instead."""
    p, L, _ = _est(preset)
    ph = se.placeholder_parts(L, p)
    assert "fc" not in ph and sz.group_of("nav", "placeholder") == sz.group_of("fcu", "placeholder") == "avionics"
    nl, nw, nh = p["electronics"]["nav"]["dims_lwh"]
    nav = ph["nav"]
    assert nav.mass_g == p["electronics"]["nav"]["mass_g"] and nav.volume_mm3 == pytest.approx(nl * nw * nh)
    assert nav.com_mm == pytest.approx((L.tray_x + L.tray_t / 2 + nh / 2, 0.0, L.esc_z[1] + 10.0 + nl / 2))
    fl, fw, fh = p["electronics"]["fcu"]["dims_lwh"]
    fcu = ph["fcu"]
    assert fcu.kind == "placeholder" and fcu.mass_g == p["electronics"]["fcu"]["mass_g"] == 20.0
    assert fcu.volume_mm3 == pytest.approx(fl * fw * fh)
    assert fcu.com_mm == pytest.approx((0.0, 0.0, L.z_intake_bulkhead_top + p["electronics"]["fcu"]["foam_t"] + fh / 2))
    assert fcu.density_g_cm3 == pytest.approx(fcu.mass_g / (fcu.volume_mm3 * 1e-3))
    p0 = json.loads(json.dumps(p))
    del p0["electronics"]["fcu"]
    L0 = layout.Layout(p0)
    ph0 = se.placeholder_parts(L0, p0)
    assert not L0.has_fcu and "fcu" not in ph0
    if L.servo_leads:
        assert ph0["servo_leads"].mass_g > ph["servo_leads"].mass_g and ph0["servo_leads"].com_mm[2] > ph["servo_leads"].com_mm[2]


def test_tab_mount_makes_the_propulsion_module_heavier():
    """Same params with edf.mount forced to lip_clamp: the tab version adds the cylinder, liner and lugs."""
    p, L, est = _est(DEFAULT)
    if not L.tabs:
        pytest.skip("default preset is not a tab mount")
    p_lc = json.loads(json.dumps(p))
    p_lc["edf"]["mount"] = "lip_clamp"
    L_lc = layout.Layout(p_lc)
    tabs = est.estimate_all()["propulsion_module"]
    lc = se.StructureEstimator(L_lc, p_lc).estimate_all()["propulsion_module"]
    rho = p["printer"]["density_g_cm3"] * 1e-3
    lugs = sum(s.volume_mm3 for s in est.tab_mount_solids() if s.label == "screw lug") * rho
    assert tabs.mass_solid_g > lc.mass_solid_g + lugs
    assert tabs.mass_g > lc.mass_g                                     # also as printed
    assert est.estimate_all()["intake_module"].mass_solid_g > se.StructureEstimator(L_lc, p_lc).estimate_all()["intake_module"].mass_solid_g


def test_edf_unit_tab_placeholder():
    p, L, _ = _est_screws(DEFAULT)
    if not L.tabs:
        pytest.skip("default preset is not a tab mount")
    e = p["edf"]
    edf = se.placeholder_parts(L, p)["edf_unit"]
    ring = lambda ri, ro, h: math.pi * (ro * ro - ri * ri) * h   # noqa: E731
    v = (ring(e["housing_id"] / 2, e["lip_od"] / 2, L.edf_lip_t)
         + ring(e["housing_id"] / 2, e["housing_od"] / 2, L.z_edf_lip_bottom - L.z_edf_housing_bottom)
         + ring(0.0, e["hub_d_ASSUMED"] / 2, min(se.EDF_HUB_LEN_MM, L.edf_housing_len))
         + ring(0.0, e["motor_d"] / 2, e["motor_protrusion_rear"])
         + L.tab_count * L.tab_t * ((L.tab_r_out - L.tab_r_in) * (L.tab_z1 - L.tab_z0)
                                    - len(L.tab_slot_zs) * L.tab_slot_w * L.tab_slot_len_radial))
    assert edf.volume_mm3 == pytest.approx(v)
    assert edf.mass_g == e["mass_g"]
    assert edf.density_g_cm3 == pytest.approx(e["mass_g"] / (v * 1e-3))
    # two tabs 180 deg apart (v0.5d: 90 / 270): the centroid stays on the axis
    assert L.tab_count == 2 and abs((L.tab_thetas[1] - L.tab_thetas[0]) % 360.0 - 180.0) < 1e-9
    assert edf.com_mm[0] == pytest.approx(0.0, abs=1e-9) and edf.com_mm[1] == pytest.approx(0.0, abs=1e-9)
    zs = [L.z_edf_housing_bottom - e["motor_protrusion_rear"], L.z_edf_lip_top]
    assert zs[0] < edf.com_mm[2] < zs[1]
    I = edf.inertia_about_com_kg_m2
    # the tab plates lie along their own radial direction: they add to the inertia about the perpendicular
    # in-plane axis more than to the one along them (tabs along y at 90 / 270 deg -> I_xx > I_yy)
    along_x = abs(math.cos(math.radians(L.tab_thetas[0]))) > abs(math.sin(math.radians(L.tab_thetas[0])))
    assert (I[1][1] > I[0][0]) if along_x else (I[0][0] > I[1][1])
    assert L.tab_thetas == [90.0, 270.0] and not along_x                        # SPEC s3.7 v0.5d


def test_boat_tail_ends_at_cone_z1(result):
    """Barrowman boat tail = the cone only (cone_z1 - z_vane_section_top); above cone_z1 the skin is body."""
    lay = layout.build(DEFAULT)
    boat = {c["name"]: c for c in result["stability"]["components"]}["boat_tail_cone"]
    length = lay.cone_z1 - lay.z_vane_section_top
    r = lay.body_od / lay.vane_section_od
    x_local = length / 3 * (1 + (1 - r) / (1 - r * r))
    assert boat["z_mm"] == pytest.approx(lay.cone_z1 - x_local)
    assert boat["cna_per_rad"] == pytest.approx(2 * ((lay.vane_section_od / lay.body_od) ** 2 - 1))
    assert lay.z_vane_section_top < boat["z_mm"] < lay.cone_z1 <= lay.z_prop_spigot_bottom


def test_cli_estimate_only(tmp_path, capsys):
    assert sz.main(["--estimate-only", "--out-dir", str(tmp_path)]) == 0
    data = json.loads((tmp_path / "sizing_report.json").read_text(encoding="utf-8"))
    assert data["structure_source"].startswith("ESTIMATE") and data["cad_report"] is None
    assert all(v["source"] in ("ESTIMATE", "params") for v in data["parts"].values())
    assert "structure: ESTIMATE" in capsys.readouterr().out


def test_json_schema_mirrors_cad_report(result):
    for key in ("generator", "preset", "params_sha256", "parts", "totals", "derived", "warnings"):
        assert key in result
    part = result["parts"]["battery"]
    for key in ("kind", "volume_mm3", "density_g_cm3", "mass_g", "bbox_mm", "com_mm", "inertia_about_com_kg_m2"):
        assert key in part
    for key in ("mass_g", "printed_mass_g", "com_mm", "inertia_about_com_kg_m2", "post_ejection"):
        assert key in result["totals"]
    assert set(result["derived"]) >= {"pushrod_len_mm", "nose_cavity_volume_L", "footprint_d_mm", "vehicle_h_mm", "inlet_area_ratio"}


def test_cad_report_with_failed_components_is_not_used(tmp_path):
    """A generator that could not build a required feature writes a partial part: never use that report."""
    rep = {"preset": "BUDGET_90mm_6S", "params_sha256": "x", "failed_components": ["propulsion_module"], "parts": {}}
    path = tmp_path / "fusion360_report.json"
    path.write_text(json.dumps(rep), encoding="utf-8")
    with pytest.raises(ValueError):
        sz.find_cad_report(str(path), "BUDGET_90mm_6S", "x", [])


@pytest.mark.parametrize("preset", [DEFAULT] + ALTS)
def test_cad_report_missing_components_is_not_used(tmp_path, monkeypatch, preset):
    """A report that silently dropped a part is no evidence: ignored when found, ValueError when given explicitly.
    launch_pad is optional; a part reported under its base name (vane for vane_1..4) counts as present."""
    p = layout.load_params(layout.PARAMS_PATH, preset)
    lay = layout.Layout(p)
    expected = se.expected_report_parts(lay, p)
    printed = set(se.StructureEstimator(lay, p).estimate_all())
    assert set(expected) == printed | (set(se.placeholder_parts(lay, p)) - set(se.BUDGET_ONLY_ITEMS))
    assert "launch_pad" not in expected and "spring_piston_hardware_balance" not in expected
    assert set(spec6_placeholders(preset)) <= set(expected) and len(expected) == len(set(expected))
    exp_dir = tmp_path / "cad" / "exports"
    exp_dir.mkdir(parents=True)
    monkeypatch.setattr(sz, "ROOT", str(tmp_path))
    suffix = "" if preset == DEFAULT else f"_{preset}"
    path = exp_dir / f"cadquery_report{suffix}.json"

    def write(parts: dict) -> None:
        path.write_text(json.dumps({"generator": "cadquery", "preset": preset, "params_sha256": "a" * 64, "parts": parts}),
                        encoding="utf-8")

    full = _complete_parts(preset)
    write(full)                                                       # complete, launch_pad absent: used
    w: list[str] = []
    assert sz.find_cad_report(None, preset, "a" * 64, w, DEFAULT) is not None and w == []
    per_vane = "vane_coupler_2" if lay.vane_drive_mode == "direct" else "vane_horn_2"     # v0.9: no horns in 'direct'
    for dropped in ("piston", "edf_unit", "joint_screws_inserts", per_vane):
        write({n: v for n, v in full.items() if n != dropped})
        w = []
        assert sz.find_cad_report(None, preset, "a" * 64, w, DEFAULT) is None
        assert any("missing components" in m and dropped in m and "ignored" in m for m in w), w
        with pytest.raises(ValueError, match="missing components"):
            sz.find_cad_report(str(path), preset, "a" * 64, [], DEFAULT)
        r = sz.run(preset, write=False)                               # the budget falls back to the estimate
        assert r["cad_report"] is None and any("missing components" in m for m in r["warnings"])
    write({})
    assert sz.find_cad_report(None, preset, "a" * 64, [], DEFAULT) is None
    # only the vane instances vane_1..n go under the base name (v0.9: vane_coupler_k / vane_drive_hardware stay)
    base = {n: v for n, v in full.items() if not (n.startswith("vane_") and n[len("vane_"):].isdigit())}
    base["vane"] = full["vane_1"]
    write(base)
    assert sz.find_cad_report(None, preset, "a" * 64, [], DEFAULT) is not None


@pytest.mark.parametrize("preset", [DEFAULT] + ALTS)
def test_com_board_placeholder_and_bosses(preset):
    """v0.6 (SPEC s5.3 / s6): the ``com`` placeholder is the Layout COM-board box with electronics.com.mass_g in the
    avionics group; the payload module gains exactly the boss rings (Ø com_boss_d minus the tap hole, com_boss_h tall)
    and loses the bulkhead material the blind holes remove above them; without the v0.6 params keys the payload
    module and the placeholder list are the v0.5 ones."""
    p, L, est = _est(preset)
    assert L.has_com_board
    ph = se.placeholder_parts(L, p)["com"]
    bx, by, bz = L.com_board_x, L.com_board_y, L.com_board_z
    assert ph.kind == "placeholder" and ph.mass_g == p["electronics"]["com"]["mass_g"]
    assert ph.volume_mm3 == pytest.approx((bx[1] - bx[0]) * (by[1] - by[0]) * (bz[1] - bz[0]))
    assert ph.com_mm == pytest.approx(((bx[0] + bx[1]) / 2, (by[0] + by[1]) / 2, (bz[0] + bz[1]) / 2))
    assert ph.density_g_cm3 == pytest.approx(ph.mass_g / (ph.volume_mm3 * 1e-3))
    assert sz.group_of("com", "placeholder") == "avionics"
    n = len(L.com_boss_xy)
    sol = est.com_mount_solids()
    v_boss = sum(s.volume_mm3 for s in sol if s.label == "COM boss")
    v_hole = sum(s.volume_mm3 for s in sol if s.label.startswith("COM blind hole"))
    assert n == 4 and v_boss == pytest.approx(n * math.pi / 4 * (L.com_boss_d ** 2 - L.com_hole_d ** 2) * L.com_boss_h)
    assert v_hole == pytest.approx(-n * math.pi / 4 * L.com_hole_d ** 2 * (L.com_hole_depth - L.com_boss_h))
    for s in sol:   # under the bulkhead (bosses) / inside it (removed hole material), on the pattern corners
        assert abs(abs(s.com_mm[0]) - L.com_pattern / 2) < 1e-9 and abs(abs(s.com_mm[1]) - L.com_pattern / 2) < 1e-9
        zlo, zhi = (L.z_com_boss_tip, L.z_com_boss_top) if s.label == "COM boss" else (L.z_com_boss_top, L.z_com_hole_end)
        assert zlo < s.com_mm[2] < zhi
    p0 = json.loads(json.dumps(p))
    del p0["payload_module"]["com_mount"], p0["electronics"]["com"]
    p0["electronics"].pop("radio_log", None)     # v0.6.2: the radio/log box takes its clearance from com_mount
    p0["electronics"].pop("fcu", None)           # v0.6.3: so does the FCU board
    L0 = layout.Layout(p0)
    assert not L0.has_com_board and "com" not in se.placeholder_parts(L0, p0)
    a1 = est.estimate_all()["payload_module"]
    a0 = se.StructureEstimator(L0, p0).estimate_all()["payload_module"]
    rho = p["printer"]["density_g_cm3"] * 1e-3
    assert a1.mass_solid_g - a0.mass_solid_g == pytest.approx((v_boss + v_hole) * rho, rel=1e-9)
    assert a1.mass_g > a0.mass_g



@pytest.mark.parametrize("preset", [None] + ALTS)
def test_gps_pod_estimate_v08(preset):
    """v0.8 (payload_module.gps_pod): the lid estimate is its analytic volume (plate between the arc r = gps_lid_r_in and
    gps_u_out minus the corner triangles, posts, minus post / screw holes); the payload module gains exactly
    gps_pod_solids; the placeholders gps / gps_ground_plane / gps_lead carry their params masses in the avionics group at
    the pod; the estimate agrees with the CAD report's measured pod / lid (when one for this params.json is on disk) to
    1 %; without the gps_pod key nothing GPS remains."""
    p, L, est = _est(preset)
    assert L.has_gps_pod
    R, w, c = L.gps_lid_r_in, L.gps_w_h, L.gps_corner_c

    def prim(v: float) -> float:
        return 0.5 * (v * math.sqrt(R * R - v * v) + R * R * math.asin(v / R))
    area = L.gps_u_out * 2 * w - (prim(w) - prim(-w)) - c * c
    lid = (area * L.gps_lid_t
           + len(L.gps_post_uv) * math.pi / 4 * (L.gps_post_d ** 2 * (L.gps_post_z[1] - L.gps_post_z[0])
                                                 - L.gps_post_hole_d ** 2 * (L.gps_post_hole_z1 - L.gps_post_z[0]))
           - len(L.gps_boss_hole_uv) * math.pi / 4 * L.gps_lid_screw_d ** 2 * L.gps_lid_t)
    v_lid = sum(s.volume_mm3 for s in est.gps_pod_lid())
    assert v_lid == pytest.approx(lid, rel=2e-3)             # strips over the arc: 80 of them
    parts = est.estimate_all()
    assert parts["gps_pod_lid"].kind == "printed" and "gps_pod_lid" in se.expected_report_parts(L, p)
    ph = se.placeholder_parts(L, p)
    assert ph["gps"].mass_g == p["electronics"]["gps"]["board_mass_g"]
    assert ph["gps_ground_plane"].mass_g == p["payload_module"]["gps_pod"]["ground_plane"]["mass_g_ASSUMED"]
    assert ph["gps_lead"].kind == "lumped" and ph["gps_lead"].mass_g == pytest.approx(L.gps_lead_mass_g)
    for n in ("gps", "gps_ground_plane", "gps_lead"):
        assert sz.group_of(n, ph[n].kind) == "avionics"
    for n in ("gps", "gps_ground_plane"):
        x, y, z = ph[n].com_mm
        assert math.hypot(x, y) > L.body_r and L.gps_z_board_bottom <= z <= L.gps_z_patch_top
    p0 = json.loads(json.dumps(p))
    del p0["payload_module"]["gps_pod"]
    p0["electronics"].pop("gns", None)                    # v0.8.1: the GNS board needs the pod
    L0 = layout.Layout(p0)
    est0 = se.StructureEstimator(L0, p0)
    assert not L0.has_gps_pod and est0.gps_pod_solids() == [] and "gps_pod_lid" not in est0.estimate_all()
    assert not any(n.startswith("gps") for n in se.placeholder_parts(L0, p0))
    rho = p["printer"]["density_g_cm3"] * 1e-3
    v_pod = sum(s.volume_mm3 for s in est.gps_pod_solids())
    assert parts["payload_module"].mass_solid_g - est0.estimate_all()["payload_module"].mass_solid_g == pytest.approx(
        v_pod * rho, rel=1e-9)
    sfx = "" if preset is None else f"_{preset}"
    rep_path = os.path.join(ROOT, "cad", "exports", f"cadquery_report{sfx}.json")
    if os.path.exists(rep_path):
        with open(rep_path, "r", encoding="utf-8") as f:
            rep = json.load(f)
        with open(layout.PARAMS_PATH, "rb") as f:
            same_params = rep.get("params_sha256") == hashlib.sha256(f.read()).hexdigest()
        gx = (rep.get("extra") or {}).get("gps")
        if same_params and gx:
            assert v_pod * rho == pytest.approx(gx["added_mass_g"]["pod + raceway + pocket (in payload_module)"], rel=0.01)
            assert v_lid * rho == pytest.approx(rep["parts"]["gps_pod_lid"]["mass_g"], rel=0.01)


@pytest.mark.parametrize("preset", [None] + ALTS)
def test_gns_board_estimate_v081(preset):
    """v0.8.1 GNS board: the analytic bosses (diamond x length to the bore, minus the tap holes) add to the avionics module
    exactly what gns_boss_solids lists, agree with the CAD-measured boss mass (extra.gns) within 2 % when the report is
    current, and the ``gns`` placeholder is the Layout box with Layout.gns_mass_g (board ASSUMED + screws), group avionics,
    in the expected report parts; without electronics.gns nothing of it remains."""
    p = layout.load_params(layout.PARAMS_PATH, preset)
    L = layout.Layout(p)
    assert L.has_gns
    est = se.StructureEstimator(L, p)
    sol = est.gns_boss_solids()
    assert len(sol) == 8 and sum(1 for s in sol if s.label == "GNS boss") == 4
    v_net = sum(s.volume_mm3 for s in sol)
    assert v_net > 0.0
    ph = se.placeholder_parts(L, p)
    g = ph["gns"]
    assert g.kind == "placeholder" and g.mass_g == pytest.approx(L.gns_mass_g)
    assert L.gns_mass_g == pytest.approx(p["electronics"]["gns"]["mass_g_ASSUMED"]
                                         + p["avionics_module"]["gns_mount"]["screws_g_ASSUMED"])
    assert g.com_mm == pytest.approx(L.gns_center) and "gns" in se.expected_report_parts(L, p)
    assert sz.group_of("gns", "placeholder") == "avionics"
    p0 = json.loads(json.dumps(p))
    del p0["electronics"]["gns"]
    L0 = layout.Layout(p0)
    est0 = se.StructureEstimator(L0, p0)
    assert not L0.has_gns and est0.gns_boss_solids() == [] and "gns" not in se.placeholder_parts(L0, p0)
    rho = p["printer"]["density_g_cm3"] * 1e-3
    assert (est.estimate_all()["avionics_module"].mass_solid_g - est0.estimate_all()["avionics_module"].mass_solid_g
            == pytest.approx(v_net * rho, rel=1e-9))
    sfx = "" if preset is None else f"_{preset}"
    rep_path = os.path.join(ROOT, "cad", "exports", f"cadquery_report{sfx}.json")
    if os.path.exists(rep_path):
        with open(rep_path, "r", encoding="utf-8") as f:
            rep = json.load(f)
        with open(layout.PARAMS_PATH, "rb") as f:
            same_params = rep.get("params_sha256") == hashlib.sha256(f.read()).hexdigest()
        gx = (rep.get("extra") or {}).get("gns")
        if same_params and gx:
            assert v_net * rho == pytest.approx(gx["added_mass_g"]["bosses + tap holes (in avionics_module)"], rel=0.02)
            assert rep["parts"]["gns"]["mass_g"] == pytest.approx(L.gns_mass_g)
