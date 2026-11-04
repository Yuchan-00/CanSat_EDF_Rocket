"""v0.9 CAD-2: the hardware configuration switch (mission.config) and the VTOL configuration (configs.vtol, power.source).

What these tests pin: (1) the rocket configuration is exactly the v0.9 CAD-1 layout (configs / power never leak into
it); (2) the VTOL layout builds for every power source, refuses the KAIST preset and the rocket + tether combination;
(3) the CadQuery VTOL part list is the one sizing/vtol_sizing.py expects, without any rocket-only part; (4) the VTOL
lightening features are real on the solids (fin holes, windows, tray holes, top cap holes); (5) the VTOL geometry checks
have no FAIL; (6) the committed VTOL reports (when they are from this params.json) meet the architect's mass targets.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
for _p in (HERE, os.path.join(ROOT, "cad"), os.path.join(ROOT, "sizing")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import layout  # noqa: E402
import rocket_geometry as rg  # noqa: E402

PARAMS = os.path.join(ROOT, "params.json")


def _sha() -> str:
    with open(PARAMS, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def _scalars(L) -> dict:
    return {k: v for k, v in L.as_dict().items() if isinstance(v, (int, float, str, bool)) or v is None}


def test_default_config_is_rocket_and_identical_to_explicit_rocket():
    p0 = layout.load_params()
    assert p0["mission"]["config"] == "rocket"
    a, b = layout.build(), layout.build(config="rocket")
    assert not a.vtol and a.power_source == "battery"
    assert _scalars(a) == _scalars(b)
    assert a.printed_parts == b.printed_parts


def test_configs_do_not_leak_into_the_rocket_dict():
    p = layout.load_params()
    for k in ("top_cap",):
        assert k not in p, f"{k} must only exist under configs.vtol"
    assert p["electronics"]["nav"] and p["electronics"]["fcu"]["mass_g"] == 20.0
    assert p["propulsion_module"]["vane_section_od"] == 94.8
    assert p["intake_module"]["bulkhead_t"] == 2.0


@pytest.mark.parametrize("power", ["battery", "tether", "hybrid"])
def test_vtol_layout_builds(power):
    L = layout.build(config="vtol", power=power)
    assert L.vtol and L.power_source == power and L.has_battery == (power != "tether")
    assert not hasattr(L, "z_payload_bottom") and not hasattr(L, "z_nose_tip")
    assert L.z_cap_plate_bottom == pytest.approx(L.z_avionics_top)
    assert L.vehicle_h == pytest.approx(L.z_cap_top - L.z_exit)
    assert L.vehicle_h < layout.build().vehicle_h - 300.0
    assert L.fcu_part == "carrier" and L.has_fcu and not L.has_nav and not L.has_gns and not L.has_com_board
    for k, v in L.cap_clearance.items():
        lim = L.cap_limits.get(k)
        if lim is not None:
            assert v >= lim - 1e-9, k
    assert L.cap_tether["free cable start (boot top) -> nearest intake window"] >= L.cap_tether["required (free cable)"]
    assert ("top_cap" in L.printed_parts and "payload_module" not in L.printed_parts and "nose_cone" not in L.printed_parts
            and "launch_pad" not in L.printed_parts)
    # v0.9 fix round: the cap bank / clamp / XT90-S (/ diode board) are placeholder boxes, the hardpoint is solids; lumped
    # are only the cable boot and the leads
    assert set(L.tether_items) == ({"tether_boot", "tether_power_lead"} if power == "tether" else
                                   {"tether_boot", "tether_power_lead", "hybrid_buffer_lead"} if power == "hybrid" else set())
    assert set(L.tether_boxes) == ({"tether_cap_bank", "tether_bus_clamp", "tether_xt90"} if power == "tether" else
                                   {"tether_cap_bank", "tether_bus_clamp", "tether_xt90", "hybrid_diode_board"}
                                   if power == "hybrid" else set())
    if L.tether_boxes:
        assert L.tether_cap_to_esc_mm <= 50.0                       # tether report: caps within 50 mm of the ESC pads
        assert min(L.tether_box_clearance.values()) >= 0.0
    # the hardpoint stack keeps top_cap.hardpoint.min_clear_below over every bay box (the flight pack too)
    assert min(L.cap_stack_clearance.values()) >= L.cap_stack_min_clear - 1e-9
    assert L.z_bolt_end <= L.z_cap_nut[0] + 1e-9                      # full jam-nut engagement


def test_power_aliases_and_refusals():
    assert layout.load_params(config="vtol", power="wired")["power"]["source"] == "tether"
    assert layout.load_params(config="vtol", power="wired_buffer")["power"]["source"] == "hybrid"
    with pytest.raises(ValueError):
        layout.load_params(power="tether")                        # the rocket is battery-powered
    with pytest.raises(KeyError):
        layout.load_params(config="glider")
    with pytest.raises(ValueError, match="BUDGET"):
        layout.build("KAIST_90mm_12S", config="vtol")
    assert layout.config_suffix("rocket") == "" and layout.config_suffix("vtol") == "_vtol"
    assert layout.config_suffix("vtol", "tether") == "_vtol_tether" and layout.config_suffix("vtol", "wired_buffer") == "_vtol_hybrid"


def test_vtol_overrides_applied():
    L, R = layout.build(config="vtol"), layout.build()
    assert (L.vane_section_od - L.duct_id) / 2 == pytest.approx(1.6)
    assert L.edf_section_h == 92.0 and R.edf_section_h == 100.0
    assert L.z_intake_bulkhead_top - L.z_intake_bulkhead_bottom == pytest.approx(1.2)
    assert L.vent_hole_thetas == [90.0, 270.0]
    assert L.fair_wall == pytest.approx(1.2) and R.fair_wall == pytest.approx(1.6)
    assert L.battery_z[0] == pytest.approx(L.z_tray_bottom + 2.0) and L.misc_side == "+x"
    assert L.avionics_h == 150.0 and len(L.avw_thetas) == 14 and L.avw_strut_w >= 2 * L.p["printer"]["min_feature"]


def test_vtol_part_list_matches_sizing_expectation():
    import vtol_sizing as vs
    for power in ("battery", "tether", "hybrid"):
        L = layout.build(config="vtol", power=power)
        parts = rg.build_all(L)
        assert sorted(parts) == sorted(vs.expected_parts(L)), power
        for gone in ("payload_module", "piston", "nose_cone", "latch_arm", "cansat_payload", "chute_pack", "spring",
                     "servo_release", "nav", "com", "radio_log", "gns", "fcu", "launch_pad", "gps_pod_lid"):
            assert gone not in parts


def test_vtol_lightening_features_on_the_solids():
    L = layout.build(config="vtol")
    prop = rg.propulsion_module(L)
    # v0.9 fix round: no fin lightening hole (leg strength) - the fin plate outboard of the fairing is solid, and the VTOL fin
    # span keeps the rocket's footprint
    assert not L.has_fin_hole and L.footprint_d == pytest.approx(layout.build().footprint_d)
    for th in L.fin_thetas:
        u0, u1 = L.fair_u[1] + 2.0, L.fin_tip_r - 2.0
        plate = rg._box_polar(u0, u1, -0.3, 0.3, L.fin_te_z + 1.0, L.fin_le_z(u1) - 1.0, th)
        assert rg.intersection_volume(prop, plate) == pytest.approx(plate.Volume(), rel=1e-6)
    av = rg.avionics_module(L)
    zc = 0.5 * (L.avw_z0 + L.avw_z1)
    for th in L.avw_thetas:
        assert rg.intersection_volume(av, rg._radial_cyl(2.0, L.body_id / 2 - 0.5, L.body_r + 0.5, th, zc)) < 1e-6
    tray = rg.avionics_tray(L)
    (yi, yo), (hz0, hz1) = L.tray_hole_y, L.tray_hole_z
    assert rg.intersection_volume(tray, rg._box(L.tray_x - 2, L.tray_x + 2, yi + 1, yo - 1, hz0 + 1, hz1 - 1)) < 1e-6
    cap = rg.top_cap(L)
    for x, y, d in L.cap_holes.values():
        assert rg.intersection_volume(cap, rg._cyl_z(0.3 * d, L.z_cap_plate_bottom - 0.5, L.z_cap_top + 0.5, x, y)) < 1e-6
    # the windows remove what Layout says they open (radial prisms through a 1.2 mm wall, within 3 %)
    full = rg.avionics_module(layout.Layout(_without_windows(L.p)))
    removed = full.Volume() - av.Volume()
    assert removed == pytest.approx(L.avw_open_area_mm2 * L.body_wall, rel=0.03)


def _without_windows(p: dict) -> dict:
    import copy
    q = copy.deepcopy(p)
    q["avionics_module"]["windows"] = None
    return q


def test_vtol_checks_have_no_fail():
    import check_geometry as cg
    import build_rocket as br
    L, parts, report = br.build(None, export=False, verbose=False, write_report=False, config="vtol")
    checks = cg.Checker(L, parts, report).run()
    fails = [(c.id, c.measured) for c in checks if c.status == cg.FAIL]
    assert not fails, fails
    ids = {c.id: c.status for c in checks}
    assert ids["C30"] == cg.PASS and ids["C31"] == cg.PASS and ids["C25"] == cg.PASS
    assert ids["C22"] == cg.NA and ids["C26"] == cg.NA and ids["X2"] == cg.NA
    # v0.9 fix round: assembly / retention / plenum / solid fin legs / lateral CG of the battery VTOL
    assert ids["C5"] == ids["C28"] == ids["C29"] == ids["C32"] == ids["C33"] == cg.PASS


def test_vtol_lateral_cg_and_tether_hardware():
    """v0.9 fix round: the tray plane move centres the battery VTOL's CG (|r| <= 1 mm), the tether / hybrid variants carry
    their kit as solids / boxes (no lumped mass inside another part), the gland sits over the battery slot (-x, clear of
    the tray plane) and the rails clear the side windows."""
    L = layout.build(config="vtol")
    assert L.tray_x < layout.build().tray_x and L.avw_rail_clear_deg > 0.0
    gx = [x for nm, (x, y, d) in L.cap_holes.items() if nm.startswith("gland")]
    assert all(x + 3.0 < L.tray_x - L.tray_t / 2 for x in gx)
    for power in ("tether", "hybrid"):
        Lp = layout.build(config="vtol", power=power)
        parts = rg.build_all(Lp)
        for nm in list(Lp.tether_boxes) + ["tether_eyebolt", "tether_load_washer", "tether_nut"]:
            assert parts[nm].kind == "placeholder" and parts[nm].shape is not None, nm
        assert not {"tether_bus_parts", "tether_hardpoint", "hybrid_diode_board"} & {
            n for n, q in parts.items() if q.kind == "lumped"}


@pytest.mark.parametrize("power,limit", [("battery", 1650.0), ("tether", 1400.0)])
def test_committed_vtol_report_meets_mass_target(power, limit):
    path = os.path.join(ROOT, "cad", "exports", f"cadquery_report{layout.config_suffix('vtol', power)}.json")
    if not os.path.exists(path):
        pytest.skip("VTOL report not built")
    with open(path, "r", encoding="utf-8") as f:
        rep = json.load(f)
    if rep.get("params_sha256") != _sha():
        pytest.skip("VTOL report is from another params.json")
    assert rep["config"] == "vtol" and rep["power_source"] == power
    assert not rep["warnings"], rep["warnings"]          # every printed part from the slicer table (no class fallback)
    assert rep["totals"]["mass_g"] <= limit
