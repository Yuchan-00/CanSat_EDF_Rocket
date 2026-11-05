"""2026-11-05 logic power (user decisions 1-4, params configs.vtol.logic_power): the carrier PCB rev 2c's 1S logic pack
at spot A, the MTS-102 switch through the top cap and the leads - VTOL battery build only.

What these tests pin: (1) the block never leaks into the rocket dict and only the battery build carries it (tether / hybrid
not maintained with the logic-battery carrier: their Layout and their top cap stay what they were); (2) the switch hole is
real on the printed cap and removes exactly a plate-thick cylinder; (3) the named mass switch
logic_power.in_flight_mass_model is honoured - false: no logic-power part in the report's parts, the cap's mass entry is
the cap without the hole, the pending numbers are the flight totals + the pending parts; true (in memory): the parts join
the report and its totals equal the false build's pending full totals; (4) C37-C40 PASS on the battery build, N/A
elsewhere, and FAIL when a solid intrudes (negative controls on the solids, the limits untouched); (5) the Layout refuses
a switch on a rib and a pack into the 6S pack.  Review fixes (2026-11-05): (6) C39 WARNs while the lever is the
vehicle top and C30 names it (derived.vehicle_h_mm counts it whatever the mass switch); (7) the cap joint screw length
limit and the nut bearing are enforced (Layout refuses an M3 x 8 cap screw / a hole wider than the nut flats allow);
(8) the strap / tie loops hold the switch lead inside and pass under the pack lead (Layout, E); (9) the Fusion X3 replay
record of the new parts is current (twin volumes = the live CadQuery solids) and its boolean differences < 1 mm^3;
(10) C40's wording follows the mass switch.
Switch-collar round (user decision 2026-11-05, '상단 캡과 한 몸으로 출력하는 보호 칼라를 달아줘', logic_power.collar): (11) the
printed cap = the holed cap + the collar (Layout volume), the flight-mass cap and the report's flight mass are those of the
previous round bit for bit (a build without the collar block gives the same parts / totals); (12) the collar rim is the
vehicle top - C30 names it, C39 PASSes on the lever envelope and FAILs when the solids' collar is too low, C41 PASSes
and FAILs on a finger-path intruder / a stricter sky limit; (13) Layout refuses a collar below the margin, a wall that
is not whole 0.4 mm lines, notches narrower than the fingertip and a throw that leaves the rim hull; (14) the collared cap
prints upright with supports and the committed slice / pending-mass report carry the collar.
(The review-round test that the LEVER is the vehicle top (6) is replaced by (12): the user's collar decision changed it.)
Collar review fixes (2026-11-05, independent review F1 / F2 / F5): (15) the notches are U slots and the finger PATH (the
push from the end position to past centre) keeps finger_path_margin_min from the collar - Layout refuses the old 16 mm
and a 14 mm notch and a bore inside the finger circle, C41 FAILs a solid-only 16 mm notch; (16) C30 skips only
logic_switch - any other logic_* solid above the patch fails it.
Run: python -m pytest cad/cadquery/test_logic_power.py -q
"""
from __future__ import annotations

import copy
import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
for _p in (HERE, os.path.join(ROOT, "cad"), os.path.join(ROOT, "sizing")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import build_rocket as br  # noqa: E402
import check_geometry as cg  # noqa: E402
import layout  # noqa: E402
import rocket_geometry as rg  # noqa: E402

LOGIC = ("logic_pack", "logic_switch", "logic_switch_lead", "logic_retention")


@pytest.fixture(scope="module")
def battery():
    L = layout.build(config="vtol")
    parts = rg.build_all(L)
    rep = br.make_report(L, parts)
    return L, parts, rep


def test_block_only_in_the_battery_build():
    assert "logic_power" not in layout.load_params()          # rocket dict
    L = layout.build(config="vtol")
    assert L.has_logic_power and not L.logic_in_flight
    assert "logic power switch" in L.cap_holes
    for power in ("tether", "hybrid"):
        Lp = layout.build(config="vtol", power=power)
        assert not Lp.has_logic_power and "logic power switch" not in Lp.cap_holes
        assert not hasattr(Lp, "lp_pack_x")
        parts = rg.build_all(Lp)
        assert not set(LOGIC) & set(parts)
        assert parts["top_cap"].mass_shape is None


def test_pack_at_spot_a_as_the_pcb_round_assumed(battery):
    L = battery[0]
    # PCB README 9.1: x -48.4..-37.9, y -27.75..27.75, z 346.1..413.1 (vehicle frame)
    for got, want in ((L.lp_pack_x, (-48.4, -37.9)), (L.lp_pack_y, (-27.75, 27.75)), (L.lp_pack_z, (346.1, 413.1))):
        assert got == pytest.approx(want, abs=1e-6)
    assert L.lp_pack_x[1] < L.battery_x[0]                   # on the 6S pack's -x face
    assert L.lp_strap_wraps_pack


def test_switch_hole_on_the_printed_cap_only(battery):
    L, parts, _ = battery
    cap, noh = parts["top_cap"].shape, parts["top_cap"].mass_shape
    sx, sy, d = L.cap_holes["logic power switch"]
    probe = rg._cyl_z(0.3 * d, L.z_cap_plate_bottom - 0.5, L.z_cap_top + 0.5, sx, sy)
    assert rg.intersection_volume(cap, probe) < 1e-6
    assert rg.intersection_volume(noh, probe) == pytest.approx(probe.Volume() * L.cap_plate_t / (L.cap_plate_t + 1.0),
                                                               rel=1e-6)
    hole_only = rg.top_cap(L, with_logic_collar=False)          # switch-collar round: the printed cap also has the collar
    removed = noh.Volume() - hole_only.Volume()
    assert removed == pytest.approx(math.pi / 4 * d * d * L.cap_plate_t, rel=1e-4)
    # review fix (2026-11-05): the U notches' curved intersections need the adaptive OCC integration (the default call
    # under-reports this difference by ~0.05 mm^3); same 1e-6 rule
    assert (br.mp.precise_volume(cap) - br.mp.precise_volume(hole_only)
            == pytest.approx(L.lp_col_volume_mm3, rel=1e-6))
    # the tether / hybrid cap is the hole-less cap
    Lt = layout.build(config="vtol", power="tether")
    assert rg.top_cap(Lt).Volume() == pytest.approx(noh.Volume(), rel=1e-9)


def test_mass_switch_false_keeps_the_flight_model(battery):
    L, parts, rep = battery
    assert not set(LOGIC) & set(rep["parts"])
    lp = rep["extra"]["logic_power"]
    assert rep["parts"]["top_cap"]["volume_mm3"] == pytest.approx(lp["top_cap"]["flight_model_volume_mm3 (without them)"],
                                                                  rel=1e-12)
    assert rep["extra"]["mass_model"]["parts"]["top_cap"]["model"] == "slicer_part_g"
    assert lp["flight_model"]["mass_g"] == rep["totals"]["mass_g"]
    want = (rep["totals"]["mass_g"] - lp["top_cap"]["flight_model_mass_g"] + lp["top_cap"]["printed_mass_g"]
            + sum(lp["pending_parts_g"].values()))
    assert lp["with_logic_power"]["mass_g"] == pytest.approx(want, abs=1e-9)
    assert lp["pending_parts_g"]["logic_pack"] == pytest.approx(84.0)


def test_mass_switch_true_adopts_the_pending_numbers(battery):
    L, _, rep = battery
    p = copy.deepcopy(L.p)
    p["logic_power"]["in_flight_mass_model"] = True
    L2 = layout.Layout(p)
    parts2 = rg.build_all(L2)
    assert set(LOGIC) <= set(parts2) and parts2["top_cap"].mass_shape is None
    rep2 = br.make_report(L2, parts2)
    w = rep["extra"]["logic_power"]["with_logic_power"]
    assert rep2["totals"]["mass_g"] == pytest.approx(w["mass_g"], abs=1e-6)
    for a, b in zip(rep2["totals"]["com_mm"], w["com_mm"]):
        assert a == pytest.approx(b, abs=1e-6)
    import vtol_sizing as vs
    assert sorted(parts2) == sorted(vs.expected_parts(L2))
    # review fix G7: C40's wording follows the switch (the flight model is the 'with' case)
    c = cg.Checker(L2, parts2, rep2).c40_logic_mass()
    assert c.status == cg.PASS, (c.measured, c.detail)
    assert "the flight model is the with case" in c.detail and "adopted)" in c.measured
    # review fix F1: the vehicle height is geometry, the same in both modes
    assert rep2["derived"]["vehicle_h_mm"] == pytest.approx(rep["derived"]["vehicle_h_mm"], abs=1e-9)


@pytest.fixture(scope="module")
def checker(battery):
    L, parts, rep = battery
    return cg.Checker(L, parts, rep)


def test_logic_checks_pass(checker):
    for fn in (checker.c37_logic_pack, checker.c38_logic_switch, checker.c39_logic_reach, checker.c40_logic_mass,
               checker.c41_logic_collar):
        c = fn()
        assert c.status == cg.PASS, (c.id, c.measured, c.detail)
    assert "with the logic power" in checker.c40_logic_mass().measured


def test_collar_is_the_vehicle_top_and_covers_the_lever(battery, checker):
    """Switch-collar round (replaces the review-fix G2 / F1 test that the LEVER is the vehicle top): the collar rim of the
    printed cap is the battery build's top, measured on the solids - C30 names the collar (and the lever inside it) and
    finds nothing else above the patch, C39 PASSes on the lever envelope (no WARN), the report height counts the rim."""
    L, parts, rep = battery
    assert not any(n.startswith("tether_eyebolt") for n in parts)
    rim = L.lp_col_z[1]
    others = max(s.BoundingBox().zmax for n, s in checker.shape.items() if not n.startswith("logic_") and n != "top_cap")
    assert rim > L.lp_sw_top_z + 2.0 - 1e-9 and L.lp_sw_top_z > others + 1.0
    c39 = checker.c39_logic_reach()
    assert c39.status == cg.PASS, (c39.measured, c39.detail)
    col = c39.values["collar"]
    assert col["rim_z"] == pytest.approx(rim, abs=1e-6)
    assert col["rim_margin"] >= L.p["logic_power"]["collar"]["rim_margin_min"]
    assert col["env_top_z"] == pytest.approx(L.lp_lever_env_top_z, abs=1e-6) and col["env_top_z"] > L.lp_sw_top_z
    assert col["hull_margin"] >= L.p["logic_power"]["collar"]["plan_margin_min"]
    assert col["lever_clear"] >= L.p["logic_power"]["collar"]["lever_clear_min"] and col["lever_x_cap_mm3"] == 0.0
    c30 = checker.c30_top_cap()
    assert c30.status == cg.PASS and "collar" in c30.title and "logic switch collar" in c30.detail
    assert c30.values["logic_switch_collar_top_z"] == pytest.approx(rim, abs=1e-6)
    assert c30.values["solid_tops_z"]["top_cap"] == pytest.approx(L.z_cap_top, abs=1e-6)   # nothing else of the cap is high
    assert c30.values["logic_switch_lever_top_z"] == pytest.approx(L.lp_sw_top_z, abs=1e-6)
    assert rep["derived"]["vehicle_h_mm"] == pytest.approx(rim - 0.0, abs=1e-6)


def test_collar_has_no_effect_on_the_flight_mass(battery):
    """Switch-collar round: the collar is logic-power kit - a build without the collar block gives the same report parts /
    totals bit for bit (the flight-mass cap has neither the hole nor the collar)."""
    import json
    L, parts, rep = battery
    p = copy.deepcopy(L.p)
    p["logic_power"].pop("collar")
    L0 = layout.Layout(p)
    assert not L0.has_lp_collar
    rep0 = br.make_report(L0, rg.build_all(L0))
    assert json.dumps(rep0["parts"], sort_keys=True) == json.dumps(rep["parts"], sort_keys=True)
    assert json.dumps(rep0["totals"], sort_keys=True) == json.dumps(rep["totals"], sort_keys=True)
    assert rep0["derived"]["vehicle_h_mm"] == pytest.approx(L.lp_sw_top_z, abs=1e-6)      # without it the lever is the top
    tc = rep["extra"]["logic_power"]["top_cap"]
    assert tc["has_collar"] and tc["collar_added_mm3"] == pytest.approx(L.lp_col_volume_mm3, rel=1e-6)
    assert tc["hole_removed_mm3"] == pytest.approx(math.pi / 4 * L.lp_sw_hole_d ** 2 * L.cap_plate_t, rel=1e-4)


def test_collar_dimensions_on_the_solid(battery):
    """The collar as built (M): rim / bore / notches / base ring from params via Layout, both arcs and the base ring
    wall_t thick, the bore and notches open, and C41's finger probes touch the lever ends without overlapping anything."""
    L, parts, _ = battery
    cap = parts["top_cap"].shape
    co = L.p["logic_power"]["collar"]
    sx, sy = L.lp_sw_xy
    assert L.lp_col_ri == 0.5 * co["inner_d"] and L.lp_col_t == co["wall_t"]
    assert L.lp_col_z[1] - L.z_cap_top == pytest.approx(co["h"], abs=1e-9)
    assert L.lp_col_notch_z0 - L.z_cap_top == pytest.approx(co["notch_bottom_h"], abs=1e-9)
    assert L.lp_col_notch_zc - L.lp_col_notch_z0 == pytest.approx(0.5 * co["notch_w"], abs=1e-9)     # U slot
    zt, z1, zn = L.z_cap_top, L.lp_col_z[1], L.lp_col_notch_z0
    ring = rg._cyl_z(L.lp_col_ro - 0.05, zt + 0.1, zn - 0.1, sx, sy).cut(rg._cyl_z(L.lp_col_ri + 0.05, zt, zn, sx, sy))
    assert rg.intersection_volume(cap, ring) == pytest.approx(ring.Volume(), rel=1e-6)            # base ring unbroken
    hw = 0.5 * L.lp_col_notch_w
    for sgn in (1.0, -1.0):
        y0, y1 = sorted((sy + sgn * (L.lp_col_ri - 0.2), sy + sgn * (L.lp_col_ro + 0.2)))
        assert rg.intersection_volume(cap, rg.logic_collar_slot(L, y0, y1, shrink=0.05)) < 1e-6     # the U slot is open
        # the round bottom is material just under the U (at the lowest point and at 45 deg on both sides)
        zc = L.lp_col_notch_zc
        for dx_, dz_ in ((0.0, -hw - 0.3), (0.7071 * (hw + 0.3), -0.7071 * (hw + 0.3)),
                         (-0.7071 * (hw + 0.3), -0.7071 * (hw + 0.3))):
            ym = sy + sgn * 0.5 * (math.sqrt(L.lp_col_ri ** 2 - dx_ ** 2) + math.sqrt(L.lp_col_ro ** 2 - dx_ ** 2))
            q = rg._box(sx + dx_ - 0.1, sx + dx_ + 0.1, ym - 0.1, ym + 0.1, zc + dz_ - 0.1, zc + dz_ + 0.1)
            assert rg.intersection_volume(cap, q) == pytest.approx(q.Volume(), rel=1e-6)
        x0, x1 = sorted((sx + sgn * (L.lp_col_ri + 0.05), sx + sgn * (L.lp_col_ro - 0.05)))
        arc = rg._box(x0, x1, sy - 0.5, sy + 0.5, zn + 0.1, z1 - 0.1)
        assert rg.intersection_volume(cap, arc) == pytest.approx(arc.Volume(), rel=1e-6)          # arcs solid to the rim
    assert rg.intersection_volume(cap, rg._cyl_z(L.lp_col_ri - 0.05, zt + 0.01, z1 + 1.0, sx, sy)) < 1e-6
    pos = rg.logic_lever_positions(L)
    th = L.lp_lever_throw_deg
    for nm, probe in rg.logic_finger_probes(L).items():
        lever = pos[round(th if nm == "+y" else -th, 6)]
        assert lever.distance(probe) < 0.05 and rg.intersection_volume(lever, probe) < 1e-6
        assert rg.intersection_volume(cap, probe) < 1e-6


def test_collar_checks_fail_on_a_bad_collar(battery):
    """Negative controls on the solids (the limits untouched): a collar built only 1 mm over the lever top fails C39's
    rim margin; a 3 mm cube in the +y finger path fails C41; a stricter sky limit than the shading fails C41."""
    L, parts, rep = battery
    L2 = copy.copy(L)
    L2.lp_col_z = (L.lp_col_z[0], L.lp_sw_top_z + 1.0)
    parts2 = rg.build_all(L2)
    c = cg.Checker(L2, parts2, rep).c39_logic_reach()
    assert c.status == cg.FAIL and "collar rim only" in c.detail, (c.measured, c.detail)
    x, y, z = L.lp_col_finger["centres"][0]
    ck = cg.Checker(L, parts, rep)
    ck.shape["intruder"] = rg._box(x - 1.5, x + 1.5, y + 2.0, y + 5.0, z - 1.5, z + 1.5)
    c = ck.c41_logic_collar()
    assert c.status == cg.FAIL and "finger probe blocked" in c.detail, (c.measured, c.detail)
    L3 = copy.copy(L)
    L3.lp_col_gps = {**L.lp_col_gps, "limit_pct": 0.5 * L.lp_col_gps["blocked_sky_above_mask_pct"]}
    c = cg.Checker(L3, parts, rep).c41_logic_collar()
    assert c.status == cg.FAIL and "GPS sky shading" in c.detail


def test_collared_cap_print_plan(battery):
    """The collared cap prints upright with supports (make_v09_print_files.part_rule); the tether cap keeps the flat,
    support-free plan; the committed slice (M) is of the current printed cap, its switch-hole / collar support check
    PASSes with no support inside the hole, and the hole-less, collar-less slice is kept for the flight-mass model."""
    import json
    import make_v09_print_files as m9
    L, parts, rep = battery
    q, cands, sup, why = m9.part_rule(L, "top_cap")
    assert cands == [[]] and sup and "upright" in why
    assert m9.part_rule(layout.build(config="vtol", power="tether"), "top_cap") == m9.PARTS["top_cap"]
    d = os.path.join(ROOT, "cad", "exports", "print", "provisional_v09_cadquery_vtol")
    with open(os.path.join(d, "print_plan_provisional.json"), encoding="utf-8") as f:
        plan = json.load(f)["parts"]["top_cap"]
    shs = plan["switch_hole_support"]
    assert shs["status"] == "PASS" and shs["support_samples_in_switch_hole"] == 0 and shs["support_samples_in_collar"] == 0
    assert plan["support_type_used"] != "none"
    with open(os.path.join(d, "slicer_part_masses.json"), encoding="utf-8") as f:
        tab = json.load(f)["parts"]
    v_print = rep["extra"]["printed_solid_volume_override_mm3"]["top_cap"]
    assert tab["top_cap"]["solid_volume_mm3"] == pytest.approx(v_print, rel=1e-6)
    assert tab["top_cap@no_logic_switch_hole"]["solid_volume_mm3"] == pytest.approx(rep["parts"]["top_cap"]["volume_mm3"],
                                                                                    rel=1e-6)
    assert rep["extra"]["logic_power"]["top_cap"]["printed_mass_model"]["model"] == "slicer_part_g"
    with open(os.path.join(ROOT, "cad", "exports", "logic_power_pending_mass_vtol.json"), encoding="utf-8") as f:
        pend = json.load(f)
    assert pend["top_cap"]["has_collar"] and "collar" in pend["layout"]
    assert pend["with_logic_power"]["mass_g"] == pytest.approx(rep["extra"]["logic_power"]["with_logic_power"]["mass_g"],
                                                               abs=1e-6)


def test_cap_screw_limit_and_nut_bearing(battery, checker):
    """Review fix G1 / G5: the cap joint screws are limited (switch.cap_joint_screw_max_len) and that screw keeps
    min_clear_to_tray from the switch on the solids; the any-length proxy overlaps it (why the limit exists)."""
    L = battery[0]
    mc = float(L.p["payload_module"]["com_mount"]["min_clear_to_tray"])
    c = checker.c38_logic_switch()
    assert min(c.values["cap_joint_screw_gap_mm"].values()) >= mc
    assert max(c.values["cap_joint_screw_any_len_overlap_mm3"].values()) > 1.0
    assert L.lp_cap_screw_max_len <= c.values["cap_joint_screw_longest_keeping_clearance_mm"]
    assert L.lp_sw_nut_bearing_w >= float(L.p["logic_power"]["switch"]["nut_min_bearing_w_ASSUMED"])


def test_band_loops_and_leads(battery):
    """Review fix G3 / DP-10 (E): the switch lead runs inside the strap and the tie loops, the pack lead outside both;
    the lengthened strap no longer wraps the 6S pack's -x corners (documented)."""
    L = battery[0]
    b = L.lp_band_leads
    assert b["switch lead vs strap"]["inside_loop"] and b["switch lead vs tie"]["inside_loop"]
    assert not b["pack lead vs strap"]["inside_loop"] and not b["pack lead vs tie"]["inside_loop"]
    assert L.lp_strap_wraps_flight_pack_corners == []


def test_fusion_replay_record_of_the_logic_parts_is_current():
    """Review fix G6: the X3 replay record (cad/fusion360/replay_x3.py output) of the holed top cap and the two logic
    placeholders: boolean differences < 1 mm^3 and its twin volumes equal the live CadQuery solids (a stale record fails).
    Also the tether build's cap (no hole)."""
    import json
    L = layout.build(config="vtol")
    live = {"top_cap": rg.top_cap(L).Volume(), "logic_pack": rg.logic_pack(L).Volume(),
            "logic_switch": rg.logic_switch(L).Volume()}
    with open(os.path.join(ROOT, "cad", "exports", "fusion_replay_vtol", "x3_replay.json"), encoding="utf-8") as f:
        rec = json.load(f)["parts"]
    for n, v in live.items():
        r = rec[n]
        assert r["status"] == "PASS", (n, r)
        assert r["replay_minus_twin_mm3"] < 1.0 and r["twin_minus_replay_mm3"] < 1.0, (n, r)
        assert r["twin_volume_mm3"] == pytest.approx(v, rel=1e-6), n
    Lt = layout.build(config="vtol", power="tether")
    with open(os.path.join(ROOT, "cad", "exports", "fusion_replay_vtol_tether", "x3_replay.json"), encoding="utf-8") as f:
        rt = json.load(f)["parts"]["top_cap"]
    assert rt["status"] == "PASS" and rt["twin_volume_mm3"] == pytest.approx(rg.top_cap(Lt).Volume(), rel=1e-6)


def test_logic_checks_na_elsewhere():
    for power in ("tether", "hybrid"):
        Lp = layout.build(config="vtol", power=power)
        parts = rg.build_all(Lp)
        ck = cg.Checker(Lp, parts, {"parts": {}, "totals": {}, "extra": {}})
        for fn in (ck.c37_logic_pack, ck.c38_logic_switch, ck.c39_logic_reach, ck.c40_logic_mass, ck.c41_logic_collar):
            c = fn()
            assert c.status == cg.NA and "not maintained" in c.measured


def test_logic_checks_fail_on_an_intruder(battery):
    """Negative controls: a 3 mm cube 1 mm from the pack (C37), right under the switch body (C38) and inside the
    finger circle over the cap (C39); the report's cap entry swapped for the holed cap (C40)."""
    L, parts, rep = battery
    x0 = L.lp_pack_x[0]
    cases = {
        "c37_logic_pack": rg._box(x0 - 4.0, x0 - 1.0, -1.5, 1.5, 380.0, 383.0),
        "c38_logic_switch": rg._box(L.lp_sw_xy[0] - 1.5, L.lp_sw_xy[0] + 1.5, L.lp_sw_xy[1] - 1.5, L.lp_sw_xy[1] + 1.5,
                                    L.lp_sw_lugs[2][0] - 4.0, L.lp_sw_lugs[2][0] - 1.0),
        "c39_logic_reach": rg._box(L.lp_sw_xy[0] + 5.0, L.lp_sw_xy[0] + 8.0, L.lp_sw_xy[1] - 1.5, L.lp_sw_xy[1] + 1.5,
                                   L.z_cap_top + 1.0, L.z_cap_top + 4.0),
    }
    for fn, box in cases.items():
        ck = cg.Checker(L, parts, rep)
        ck.shape["intruder"] = box
        c = getattr(ck, fn)()
        assert c.status == cg.FAIL, (fn, c.measured)
    rep2 = copy.deepcopy(rep)
    rep2["parts"]["top_cap"]["volume_mm3"] = rep["extra"]["logic_power"]["top_cap"][
        "printed_volume_mm3 (with the switch hole and the collar)"]
    c = cg.Checker(L, parts, rep2).c40_logic_mass()
    assert c.status == cg.FAIL


@pytest.mark.parametrize("path,value,needle", [
    (("switch", "theta_deg"), 113.0, "rib"),                  # on the rib at 112.5 deg
    (("pack", "gap_to_flight_pack"), -1.0, "6S pack"),          # into the 6S pack
    (("switch", "r"), 53.0, "ring"),                           # onto the rim ring
    (("switch", "cap_joint_screw_max_len"), 8.0, "cap joint screw"),   # review fix G1: M3 x 8 reaches the switch body
    (("switch", "hole_d"), 6.8, "nut bearing"),                 # review fix G5: hole wider than the nut flats allow
    (("collar", "h"), 16.0, "collar rim - lever sweep top"),    # switch collar: rim 1 mm over the lever (< 2 mm margin)
    (("collar", "wall_t"), 2.2, "whole number"),                # switch collar: not whole 0.4 mm lines
    (("collar", "notch_w"), 12.0, "finger tip sphere -> collar arcs"),   # notches narrower than the fingertip
    (("collar", "notch_w"), 16.0, "finger path"),               # review F1: the old notch, 0 mm side room on the push
    (("collar", "notch_w"), 14.0, "finger path"),               # review F1: blocks the push (passed the end-position rule)
    (("switch", "lever_throw_deg_ASSUMED"), 30.0, "rim hull"),  # a wider throw leaves the rim tops' hull
])
def test_layout_refuses(path, value, needle):
    p = layout.load_params(config="vtol")
    d = p["logic_power"]
    for k in path[:-1]:
        d = d[k]
    d[path[-1]] = value
    with pytest.raises(ValueError, match=needle):
        layout.Layout(p)


def test_layout_refuses_a_bore_inside_the_finger_circle():
    """Review fix F1: a bore smaller than the existing Ø finger circle is refused by the Layout (not only by C39); the
    wall is thickened so that the rim-hull rule does not stop it first."""
    p = layout.load_params(config="vtol")
    co = p["logic_power"]["collar"]
    co["inner_d"], co["wall_t"] = 19.0, 3.2
    with pytest.raises(ValueError, match="collar bore -> lever finger circle"):
        layout.Layout(p)


def test_finger_path_on_the_solids(battery, checker):
    """Review fix F1 (M): C41 measures the push on the solids - its minimum equals the Layout's (0.5 mm side room near
    centre, by construction notch_w = fingertip + 2 x margin) and a 16 mm notch on the solids alone FAILs C41."""
    L, parts, rep = battery
    c = checker.c41_logic_collar()
    assert c.status == cg.PASS, (c.measured, c.detail)
    m = L.p["logic_power"]["collar"]["finger_path_margin_min"]
    assert c.values["finger_path_min_mm"] == pytest.approx(L.lp_col_finger_path["min_mm"], abs=1e-3)
    assert c.values["finger_path_min_mm"] >= m - 1e-6
    assert len(c.values["finger_path_mm"]) == 2 * len(L.lp_col_finger_path["angles_deg"])
    assert min(L.lp_col_finger_path["angles_deg"]) < 0.0 < max(L.lp_col_finger_path["angles_deg"])   # past centre
    L2 = copy.copy(L)
    L2.lp_col_notch_w = 16.0
    L2.lp_col_notch_zc = L.lp_col_notch_z0 + 8.0
    c2 = cg.Checker(L2, rg.build_all(L2), rep).c41_logic_collar()
    assert c2.status == cg.FAIL and "finger path short" in c2.detail, (c2.measured, c2.detail)


def test_c30_counts_every_logic_solid_but_the_switch(battery):
    """Review fix F5: C30 skips only ``logic_switch`` (its lever is judged in C39) - a logic_* solid above the GPS patch
    fails the vehicle-top rule."""
    L, parts, rep = battery
    ck = cg.Checker(L, parts, rep)
    assert ck.c30_top_cap().status == cg.PASS
    x, y = L.lp_sw_xy
    ck.shape["logic_dummy"] = rg._box(x + 20.0, x + 23.0, y - 1.5, y + 1.5, L.cap_patch_z[1] + 1.0, L.cap_patch_z[1] + 4.0)
    c = ck.c30_top_cap()
    assert c.status == cg.FAIL and "below the vehicle top" in c.detail, (c.measured, c.detail)
