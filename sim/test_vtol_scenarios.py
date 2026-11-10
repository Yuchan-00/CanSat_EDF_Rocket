"""Tests of the VTOL integration in the 6-DOF (sim/sim6dof.py, mode "vtol"), the VTOL scenarios / operator /
renderer export (sim/vtol_scenarios.py) and the campaign runner (sim/vtol_campaign.py).

Every flight here is a real ``FlightSim`` run (supervisor -> vertical controller -> ESC -> attitude controller,
operator -> GroundAssist -> link model -> vehicle); runs are short (one hop at 5 m) to keep the suite fast.
The ballistic build is pinned to digests computed with the v0.6.3 snapshot's sim6dof (scratch run 2026-10-20) on the
archived v0.7.1 CAD vehicle (sim/out/vtol/regression/cadquery_report_v071.json) and to the digests the v0.7.1 snapshot
code (history/v0.7.1/code_snapshot.zip, scratch run 2026-10-21) gives on the v0.8 CAD vehicle; the v0.7.1 VTOL build
(``gps=None``) is pinned to the v0.7.1 snapshot code on the archived vehicle.

v0.8 (GPS position hold): ``vs.base_vtol`` builds the GPS build by default (``hop``); ``hop71`` is the v0.7.1 build
(``gps=None``) for the tests whose subject is not the GPS.

v0.9 (VTOL hardware round): ``vs.vehicle()`` is the VTOL hardware configuration (CAD report, battery 1647 g); the regression
pins fly with the v0.9 plant additions switched off (``PLANT_OFF``: no vane play, no rotor inertia - the v0.6.3 / v0.7.1
statements bit for bit) and the v0.9 rocket pins (``V09_BALLISTIC_DIGESTS``) with them on.  Closed-loop tests whose subject is
the v0.8.1 controller's performance fail on the light VTOL vehicle until the controller round re-tunes it (listed in the
plant round's report, not hidden here).
"""
from __future__ import annotations

import hashlib
import importlib.util
import json
import math
import os
import sys
from dataclasses import replace

import numpy as np
import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(_HERE)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from sim import gps_nav as gn  # noqa: E402
from sim import link_model as lm  # noqa: E402
from sim import link_proto as lp  # noqa: E402
from sim import quat  # noqa: E402
from sim import vehicle as vm  # noqa: E402
from sim import vtol_campaign as vc  # noqa: E402
from sim import vtol_scenarios as vs  # noqa: E402
from sim.onboard import State  # noqa: E402
from sim.fc_timing import FC_COMPUTE_DELAY_S_MEASURED, FC_VTOL_ESC_EXTRA_S_ASSUMED, FCTiming  # noqa: E402
from sim.sim6dof import (LEGACY_SENSOR_SET, FlightSim, GpsReceiverPlant, GpsSim, Gust, Push, Scenario,  # noqa: E402
                         SensorConfig, SimOptions, body_radius_mm, compute_metrics, neo_m8n, push_with_impulse, vtol_aero_geometry,
                         vtol_link_budget, vtol_round_trip)
from sim.vertical_ctrl import TD_NAMES, VerticalParams  # noqa: E402
from sim.controller import gains_from_control, vtol_overrides  # noqa: E402

VEH = vs.vehicle()
P = VEH.params
V = P["mission"]["vtol"]
V071_CAD_REPORT = os.path.join(PROJECT_ROOT, "sim", "out", "vtol", "regression", "cadquery_report_v071.json")


def vehicle_from_report(path: str):
    """The Vehicle with the mass properties of a given CAD report (``sim.vehicle.load_vehicle`` with that report)."""
    params_path = os.path.join(PROJECT_ROOT, "params.json")
    p = vm.layout_mod.load_params(params_path, None)
    L = vm.Layout(p)
    if path == V071_CAD_REPORT:
        # v0.9: the current params carry direct-drive / motor-lead / GPS parts the archived v0.7.1 report never had, and
        # the current Layout refuses the v0.7.1 params (its new bulkhead-hole-corner check: 0.56 mm to the band), so the
        # expected list of this frozen archive is its own part list (it was a complete report when archived)
        with open(path, encoding="utf-8") as f:
            expected = [n for n in json.load(f)["parts"] if n not in vm.NON_VEHICLE_PARTS]
    else:
        est = vm.estimate_report(L, p, params_path)
        expected = [n for n in est["parts"] if n not in vm.NON_VEHICLE_PARTS]
    full, post, descent, w = vm.load_cad_report(path, expected)
    return vm.Vehicle(p, L, os.path.basename(path), path, full, post, descent, vm.compute_aero(L, p, float(full.com_m[2])), w)


def _digest(res) -> str:
    h = hashlib.sha256()
    for k in sorted(res.log):
        h.update(k.encode())
        h.update(np.ascontiguousarray(res.log[k]).tobytes())
    for t, e in res.events:
        h.update(repr(float(t)).encode() + e.encode())
    return h.hexdigest()


def hop(name: str = "T_hop", seed: int = 70, t_end: float = 30.0, script=None, **kw) -> Scenario:
    """ON at 1 s, climb to the hover altitude, OFF 1 s after HOLD shows, landing (v0.8: the GPS build)."""
    if script is None:
        script = (vs.OpStep("on", t=1.0), vs.OpStep("off", phase="HOLD", after=1.0))
    return vs.base_vtol(name, "test hop", None, seed=seed, t_end=t_end, vtol_script=script, **kw)


def hop71(name: str = "T_hop71", **kw) -> Scenario:
    """The same hop flown by the v0.7.1 build (no GPS, the v0.7.1 NAV order, no take-off check)."""
    return hop(name, gps=None, **kw)


HOLD_ONLY = (vs.OpStep("on", t=1.0),)


@pytest.fixture(scope="module")
def hop_run():
    return vs.run_scenario(hop())


# --------------------------------------------------------------------------- #
# constants / helpers
# --------------------------------------------------------------------------- #
def test_criteria_are_the_task_numbers():
    """The task of 2026-10-20: touchdown |v_z| <= 1.0 m/s, horizontal <= 0.5 m/s, tilt <= 5 deg; hold RMS <= 0.5 m;
    attitude back within 2 deg <= 2 s after each gust.  Never loosened to make a run pass."""
    assert vs.NOMINAL_CRITERIA == {"touchdown_vz_max_mps": 1.0, "touchdown_vh_max_mps": 0.5, "touchdown_tilt_max_deg": 5.0,
                                   "hold_rms_err_max_m": 0.5, "gust_band_deg": 2.0, "gust_recovery_max_s": 2.0}


@pytest.mark.parametrize("radio", lm.RADIOS)
def test_vtol_round_trip_is_the_link_models_vtol_mode(radio):
    """v0.7.1: the link model sizes the TELEM2 reply itself (link_model.round_trip(cfg, mode='vtol')); v0.8: the GPS
    build's reply is TELEM3 (45 B) and the ground's timeout is built on it."""
    cfg = lm.LinkConfig.from_params(P, radio=radio)
    rt2 = vtol_round_trip(cfg)
    assert rt2 == lm.round_trip(cfg, mode=lp.MODE_VTOL)
    assert rt2["downlink_s"] > lm.round_trip(cfg)["downlink_s"]
    b = vtol_link_budget(cfg)
    assert b["telem_frame_bytes"] == lp.TELEM2_FRAME_BYTES == 39
    assert b["reply_timeout_s"] == rt2["reply_timeout_s"]
    rt3 = vtol_round_trip(cfg, gps=True)
    assert rt3 == lm.round_trip(cfg, mode=lp.MODE_VTOL, gps=True) and rt3["downlink_s"] > rt2["downlink_s"]
    assert vtol_link_budget(cfg, gps=True)["telem_frame_bytes"] == lp.TELEM3_FRAME_BYTES == 45
    # 2026-11-10: the battery build on the carrier replies with the logic-battery tail (TELEM v6 40 B / v7 46 B) and the
    # ground's timeout is built on it; without the tail (lbat_telemetry False) the v0.8 frames
    rt6, rt7 = vtol_round_trip(cfg, lbat=True), vtol_round_trip(cfg, gps=True, lbat=True)
    assert rt6 == lm.round_trip(cfg, mode=lp.MODE_VTOL, lbat=True) and rt6["downlink_s"] > rt2["downlink_s"]
    assert rt7 == lm.round_trip(cfg, mode=lp.MODE_VTOL, gps=True, lbat=True) and rt7["downlink_s"] > rt3["downlink_s"]
    assert vtol_link_budget(cfg, gps=True, lbat=True)["telem_frame_bytes"] == lp.TELEM7_FRAME_BYTES == 46
    fs = FlightSim(VEH, hop71(radio=radio))
    assert fs.lbat_link and fs.link_rt == rt6 and fs.gp.reply_timeout_s == rt6["reply_timeout_s"]
    fg = FlightSim(VEH, hop(radio=radio))
    assert fg.lbat_link and fg.link_rt == rt7 and fg.gp.reply_timeout_s == rt7["reply_timeout_s"]
    f0 = FlightSim(VEH, hop(radio=radio, lbat_telemetry=False))
    assert not f0.lbat_link and f0.link_rt == rt3 and f0.gp.reply_timeout_s == rt3["reply_timeout_s"]


def test_gust_envelope():
    g = Gust(10.0, 2.0, 6.0, 90.0)
    assert g.factor(9.999) == 0.0 and g.factor(10.0) == 1.0 and g.factor(11.999) == 1.0 and g.factor(12.0) == 0.0
    r = Gust(10.0, 2.0, 6.0, 90.0, rise_s=0.5)
    assert r.factor(10.0) == 0.0 and r.factor(10.25) == pytest.approx(0.5) and r.factor(11.0) == 1.0
    assert r.factor(11.75) == pytest.approx(0.5)


def test_ballistic_scenario_defaults_are_no_ops():
    sc = Scenario(name="x")
    assert sc.mode == lp.MODE_BALLISTIC and sc.vtol_script == () and sc.gusts == () and sc.vgains is None
    assert (sc.turbulence_sigma_mps, sc.mass_scale, sc.thrust_scale, sc.servo_tau_scale, sc.thrust_voltage_exp) == (
        0.0, 1.0, 1.0, 1.0, 0.0)
    assert sc.cg_offset_mm == (0.0, 0.0) and sc.vane_trim_err_deg == (0.0,) * 4 and sc.link_outages == ()
    assert sc.baro_fail_t_s is None and sc.imu_fail_t_s is None and sc.ground_z_m is None


# v0.6.3 snapshot sim6dof (scratchpad v07/pyref/v063, same venv), scratch run 2026-10-20, on the v0.6.2 / v0.7.1 CAD vehicle
# (the archived report: B:\Video\CAD\CadQuery\이전판_v0.7.1 -> sim/out/vtol/regression/cadquery_report_v071.json)
V063_DIGESTS = {
    "pin_flight": (1751, "ee1b959279db35e0cf411b80dff399061b118eed8855e2883a29cb560d841294"),
    "pin_kill": (1001, "38197e480dfaf7a5ffccd4db00e3e15703c0a18c461b577bc0f8332a933aee19"),
}
# the v0.7.1 snapshot code (history/v0.7.1/code_snapshot.zip; its sim6dof is the v0.6.3 ballistic code) on the v0.8 CAD
# vehicle (GPS pod, cad/exports/cadquery_report.json 2026-10-21 13:39), scratch run 2026-10-21 - the ballistic build flies
# the heavier vehicle with the unchanged code
V08_VEHICLE_DIGESTS = {
    "pin_flight": (1751, "db63c991fe0baff99e0cabe556f263f58ba755a3b4b904bea927cf8bb69d8b03"),
    "pin_kill": (1001, "49a98d851167c5cf20fe67f372fe3f13b009d3cd753878c7b5f6af72a34b019a"),
}
PLANT_OFF = {"vane_play_deg": 0.0, "rotor_inertia_kg_m2": 0.0}     # v0.9: the pre-v0.9 plant (regression pins)
# 2026-11-10: pinned to the lora_uart link model the snapshots flew (the default model follows link.vehicle_radio = the
# ESP bridge since the control round); the pins check that the code path is unchanged under the same link model.
# 2026-11-10 (BMP280 default): and to the barometer model the snapshots flew, BARO_MODELS["MS5611"] (0.25 m RMS, no
# output steps), with the altitude-filter crossover they flew (0.5 Hz; the default is 0.4 Hz since the BMP280
# retune) = sim6dof.LEGACY_SENSOR_SET - the digests are unchanged, so the code path under the legacy sensor set is
# bit-identical
PIN_BARO = LEGACY_SENSOR_SET
BALLISTIC_PINS = [
    Scenario(name="pin_flight", t_end=7.0, seed=11, wind_mps=3.0, wind_dir_deg=40.0, gust_mps=5.0,
             gust_t0_after_launch_s=1.5, gust_duration_s=1.0, radio="lora_uart", sensors=SensorConfig(**PIN_BARO)),
    Scenario(name="pin_kill", t_end=4.0, seed=12, kill_time_s=2.5, packet_loss=0.1, radio="lora_uart",
             sensors=SensorConfig(**PIN_BARO))]


@pytest.fixture(scope="module")
def veh071():
    return vehicle_from_report(V071_CAD_REPORT)


@pytest.mark.parametrize("sc", BALLISTIC_PINS, ids=lambda s: s.name)
def test_ballistic_build_is_bit_identical_to_v063(sc, veh071):
    """Every log array and event of a ballistic flight (ScriptedOperator, assist, link, supervisor) equals v0.6.3 (on the
    vehicle v0.6.3 flew)."""
    assert veh071.full.mass_kg == pytest.approx(2.5857, abs=1e-4)          # the pre-GPS CAD vehicle
    res = FlightSim(veh071, replace(sc, **PLANT_OFF)).run()                 # v0.9: the plant additions off
    assert (len(res.log["t"]), _digest(res)) == V063_DIGESTS[sc.name]
    assert res.vtol is None and "phase" not in res.log


@pytest.mark.parametrize("sc", BALLISTIC_PINS, ids=lambda s: s.name)
def test_ballistic_build_on_the_v08_vehicle_equals_the_v071_code(sc):
    """The v0.8 CAD vehicle (GPS pod) through the ballistic build: equal to the v0.7.1 snapshot code on that vehicle."""
    if not VEH.mass_source.startswith("cadquery_report.json") or abs(VEH.full.mass_kg - 2.64383) > 1e-4:
        pytest.skip(f"the CAD report changed since the pin ({VEH.mass_source}, {VEH.full.mass_kg:.5f} kg)")
    res = FlightSim(VEH, sc).run()
    assert (len(res.log["t"]), _digest(res)) == V08_VEHICLE_DIGESTS[sc.name]


# the v0.7.1 snapshot code's VTOL build on the archived v0.7.1 vehicle, scratch run 2026-10-21 (vtol_regress2.py)
# v0.9 fix round (2026-10-28) RE-PIN, intended control change (changes.json key fix_round F1-F8): the VTOL attitude
# options (vtol_overrides att_err_mode 2, ff_rotor_gyro 0, mix_yaw_trim_reserve 1) and the vertical laws (reference
# approach + take-off reference hold / learning, touchdown release, LAND rebase) fly in every VTOL build, the v0.7.1 one
# included (one vertical controller in the firmware).  Before (v0.8.2 pins, valid until the control round): hop (6707,
# 017b737a...), push (4001, f39f326b...).  After: hop 7501 ticks (the climb waits for the lift-off and decelerates into
# the target: +3.2 s; LANDED normally), push 4001 ticks (the fixed 16 s window; no ABORT in it).
V071_VTOL_DIGESTS = {
    "hop": (7501, "f51712385e1db135a455c40d00a4da160790230ed40bb519ad756b249351630a"),
    "push": (4001, "5b91c635cf35ef44df5b1b615c55c16326b6edce726ba0f49afcdec439ae3a73"),
}


def test_v071_vtol_build_is_bit_identical_to_the_v071_code(veh071, monkeypatch):
    """gps=None (nav_v08 / takeoff_check unset): the v0.7.1 VTOL build - every log array and event equal to the v0.7.1
    snapshot's on the same vehicle (the v0.8 reference changes to the supervisor, the vertical controller, the
    controller and the estimator are inert with their flags off)."""
    monkeypatch.setattr(vs, "_VEH", veh071)
    p = veh071.params
    # 2026-11-10: the v0.7.1 link (lora_uart model, TELEM v2 - no logic-battery tail): the code path under the same link
    # (and the barometer model + altitude-filter crossover the snapshot flew: LEGACY_SENSOR_SET, see PIN_BARO)
    link071 = dict(radio="lora_uart", lbat_telemetry=False, sensors=vs.vtol_sensors(**PIN_BARO))
    cases = {"hop": vs.base_vtol("R_hop", "hop", None, seed=70, t_end=30.0, gps=None,
                                 vtol_script=(vs.OpStep("on", t=1.0), vs.OpStep("off", phase="HOLD", after=1.0)),
                                 **PLANT_OFF, **link071),
             "push": vs.scenario_demo(p, None, name="R_push", t_end=16.0, vtol_script=vs.demo_script(t_off=12.0),
                                      pushes=(Push(9.0, 0.3, 10.0, azimuth_deg=30.0, z_body_m=0.4, offset_deg=10.0),),
                                      wind_mps=1.0, turbulence_sigma_mps=0.2, gps=None, **PLANT_OFF, **link071)}
    g = vs.attitude_gains(p)
    for name, sc in cases.items():
        fs = FlightSim(veh071, sc, g)
        assert not fs.gps_on and not fs.v08 and not fs.tko and not fs.sp.gps_hold and not fs.sp.takeoff_check
        res = fs.run()
        assert (len(res.log["t"]), _digest(res)) == V071_VTOL_DIGESTS[name], name


# v0.9 plant round (2026-10-28): the ballistic pins on the v0.9 rocket vehicle (cad/exports/cadquery_report.json, 2592.6 g,
# direct-drive vanes) with the v0.9 plant (vane play 1.65 deg, rotor inertia 4.5e-5 kg m2) - re-pinned because the
# HARDWARE changed (direct drive -> play; the rotor gyro / spool-up reaction now modelled).  Before (plant off) / after:
# pin_flight max tilt 2.36 / 2.48 deg, apogee 23.58 / 23.58 m, vane saturation 0 / 0.26 s, max body rate 83 / 144 deg/s;
# pin_kill max tilt 0.71 / 0.76 deg, touchdown 1.116 / 1.116 m/s, max body rate 83 / 1126 deg/s (the motor cut: the
# stators still turn the decaying swirl into a yaw torque ~0.3 N m - scratch v09sim/plant/pins.py)
V09_BALLISTIC_DIGESTS = {
    "pin_flight": (1751, "fdfff0533a53ad366078a7fa662224274157f8f0357d0266c703b3887e4a8d8c"),
    "pin_kill": (1001, "195e5c0e82eb751d5aede95fdcad353bb5583ab9bf9829b070f4b353936ab6ef"),
}


@pytest.mark.parametrize("sc", BALLISTIC_PINS, ids=lambda s: s.name)
def test_ballistic_build_on_the_v09_rocket_with_the_v09_plant(sc):
    """The ballistic build flies the rocket configuration (mission.config 'rocket') with the direct-drive vane play and
    the rotor terms: pinned (the plant changed on purpose; the v0.6.3 statements are pinned above with them off)."""
    veh = vm.load_vehicle()
    if not veh.mass_source.startswith("cadquery_report.json") or abs(veh.full.mass_kg - 2.592623578903419) > 1e-6:
        pytest.skip(f"the rocket CAD report changed since the pin ({veh.mass_source}, {veh.full.mass_kg:.6f} kg)")
    assert veh.hardware_config == "rocket" and veh.has_chute
    fs = FlightSim(veh, sc)
    assert fs.play_on and fs.rot_on and fs.pw is None and fs.rig is None
    res = fs.run()
    assert (len(res.log["t"]), _digest(res)) == V09_BALLISTIC_DIGESTS[sc.name]


def test_vtol_needs_the_supervisor():
    with pytest.raises(ValueError):
        FlightSim(VEH, hop(bypass_state_machine=True)).run()


def test_operator_takes_only_vtol_buttons():
    with pytest.raises(ValueError):
        vs.VtolOperator(hop(script=(vs.OpStep("arm"),)))
    with pytest.raises(ValueError):
        vs.VtolOperator(hop(script=(vs.OpStep("up", phase="CRUISE"),)))


# --------------------------------------------------------------------------- #
# a hop: ON -> SPOOL -> CLIMB -> HOLD -> OFF -> LAND -> LANDED -> SAFE
# --------------------------------------------------------------------------- #
def test_hop_flies_the_phases_and_lands(hop_run):
    res, m = hop_run
    L = res.log
    ph = L["phase"]
    seq = [int(ph[0])] + [int(ph[i]) for i in range(1, len(ph)) if ph[i] != ph[i - 1]]
    assert seq == [lp.PHASE_NONE, lp.PHASE_SPOOL, lp.PHASE_CLIMB, lp.PHASE_HOLD, lp.PHASE_LAND, lp.PHASE_LANDED,
                   lp.PHASE_NONE]
    assert m["interlock_codes"] == [] and not m["rejections"]
    assert m["landed_normally"] and m["t_safe_after_landed_s"] is not None and not m["false_touchdown"]
    assert abs(m["touchdown_vz"]) <= vs.NOMINAL_CRITERIA["touchdown_vz_max_mps"]
    assert m["first_target_m"] == pytest.approx(V["hover_altitude_m"])
    assert res.final_state == State.SAFE
    # spool warning: ARMED for spool_warning_s with the motor off, then the motor
    assert m["t_motor_start_s"] - m["t_on_s"] == pytest.approx(V["spool_warning_s"], abs=0.005)
    P_ = L["state"] == lp.STATE_POWERED
    assert np.all(L["throttle"][~P_] == 0.0) and np.all(L["throttle"][P_] > 0.0)
    assert np.all(L["throttle"] <= 1.0)
    # the hold: within the criterion; after the take-off ramp the throttle stays within [u_min, u_max]
    assert m["hold_rms_err_m"] <= vs.NOMINAL_CRITERIA["hold_rms_err_max_m"]
    ip = np.flatnonzero(P_)
    after_ramp = ip[ip >= ip[0] + res.vtol["vp"]["ramp_ticks"]]
    assert np.all(L["throttle"][after_ramp] >= V["u_min"]) and np.all(L["throttle"][after_ramp] <= V["u_max"])
    # h_target is the supervisor's target in SPOOL / CLIMB / HOLD, NaN otherwise; h_ref only while the loop runs
    tgt_ph = np.isin(L["phase"], (lp.PHASE_SPOOL, lp.PHASE_CLIMB, lp.PHASE_HOLD))
    assert np.all(np.isfinite(L["h_tgt"][tgt_ph])) and np.all(np.isnan(L["h_tgt"][~tgt_ph]))
    assert np.all(np.isfinite(L["h_ref"][P_]))


def test_hop_goes_through_the_real_ground_link(hop_run):
    res, m = hop_run
    lk = res.link
    names = [r["name"] for r in lk["records"]]
    assert names == ["ON", "OFF"]
    assert all(r["result"].startswith("accepted") for r in lk["records"])
    assert lk["budget"]["telem_frame_bytes"] == lp.TELEM7_FRAME_BYTES       # v0.8 GPS build: TELEM v3 (2026-11-10: + the
    assert lk["lbat_telemetry"] and lk["radio"] == "esp_bridge"            # logic-battery tail, TELEM v7; ESP link model)
    ex = [e for e in lk["exec_log"] if e["cmd"] != lp.CMD_NONE]
    assert [(e["cmd"], e["code"]) for e in ex] == [(lp.CMD_ON, 0), (lp.CMD_OFF, 0)]
    assert m["on_decision_to_exec_s"] > 0.0 and m["off_decision_to_exec_s"] > 0.0
    assert any("GS phase HOLD -> LAND" in e for _, e in res.events)


def test_vtol_is_deterministic_per_seed(hop_run):
    res, _ = hop_run
    again, _ = vs.run_scenario(hop())
    assert _digest(again) == _digest(res)
    other, _ = vs.run_scenario(hop(seed=71))
    assert _digest(other) != _digest(res)


def test_compute_metrics_dispatches_vtol(hop_run):
    res, m = hop_run
    m2 = compute_metrics(res)
    assert m2["mode"] == lp.MODE_VTOL and m2["touchdown_vz"] == m["touchdown_vz"]


# --------------------------------------------------------------------------- #
# landing surface
# --------------------------------------------------------------------------- #
def test_landing_on_ground_below_the_pad():
    """The ground 1.5 m below the pad top (pad on a platform): the vehicle descends past h = 0 at the touchdown rate
    and lands under power on the lower surface."""
    sc = hop("T_below", ground_z_m=-1.5, t_end=45.0)
    res, m = vs.run_scenario(sc)
    assert m["landed_normally"] and not m["false_touchdown"]
    if not m["touchdown_on_pad"]:
        assert m["touchdown_surface_z"] == -1.5
        assert -vs.NOMINAL_CRITERIA["touchdown_vz_max_mps"] <= m["touchdown_vz"] < 0.0
    assert m["landed_by"] in ("still", "support", "floor")       # the detector confirmed it, not the V10 timeout


def test_default_ground_is_the_pad_foot_level():
    fs = FlightSim(VEH, hop())
    lpd = P["launch_pad"]
    assert fs.ground_z == pytest.approx(-(lpd["plate_t"] + lpd["leg_h"] + lpd["foot_t"]) * 1e-3)
    assert fs.surface_z(0.0, 0.0) == 0.0 and fs.surface_z(1.0, 0.0) == fs.ground_z


# --------------------------------------------------------------------------- #
# safety rules in the closed loop (faults in HOLD at the hover altitude)
# --------------------------------------------------------------------------- #
FAULT_T = 10.0


@pytest.mark.parametrize("case", [
    dict(name="T_link", kw=dict(link_outages=((FAULT_T, float("inf")),)), rule="V1", outcome="landed"),
    dict(name="T_imu", kw=dict(imu_fail_t_s=FAULT_T), rule="I9 IMU", outcome="abort"),
    dict(name="T_kill", kw={}, script=HOLD_ONLY + (vs.OpStep("kill", t=FAULT_T),), rule="KILL", outcome="abort"),
    dict(name="T_vane", kw=dict(vane_failure_index=0, vane_failure_t_after_launch_s=FAULT_T - 3.7,
                                vane_failure_stuck_deg=float(P["vane"]["max_deflection_mech_deg"])),
         rule="I3|I2", outcome="abort"),
    dict(name="T_battery", kw=dict(options=replace(SimOptions(), batt_v_full_cell=vs.low_battery_v_rest(P))),
         rule="I8", outcome="landed"),
], ids=lambda c: c["name"])
def test_safety_rule_acts_in_the_closed_loop(case):
    sc = hop(case["name"], t_end=45.0, script=case.get("script", HOLD_ONLY), **case["kw"])
    res, m = vs.run_scenario(sc)
    exp = {"expect": case["rule"], "outcome": case["outcome"]}
    verdict, why = vs.judge(m, exp)
    assert verdict == "PASS", why
    if case["outcome"] == "abort":
        assert m["t_released_s"] is not None and m["t_released_s"] - m["t_abort_s"] == pytest.approx(
            P["mission"]["abort_release_delay_s"], abs=0.005)
        k = np.searchsorted(res.log["t"], m["t_abort_s"] + 0.01)
        assert np.all(res.log["throttle"][k:] == 0.0)
    if case["name"] == "T_link":
        # V1 acts link_loss_land_s after the last frame received before the outage (2 Hz polls, 5 % loss)
        assert FAULT_T + V["link_loss_land_s"] - 1.5 <= m["t_land_s"] <= FAULT_T + V["link_loss_land_s"] + 0.01


@pytest.fixture(scope="module")
def baro_run():
    return vs.run_scenario(hop("T_baro", t_end=45.0, script=HOLD_ONLY, baro_fail_t_s=FAULT_T))


def test_baro_failure_switches_to_the_dead_reckoned_landing(baro_run):
    """I9 baro (baro_fail_ticks without a sample) in HOLD -> LAND with baro_lost; v0.7.1: the vertical controller flies
    the descending reference at descent_rate (no flare) on the VTOL filter's dead reckoning (learnt accelerometer bias),
    and the V8 / V9 time limit bounds it."""
    res, m = baro_run
    L = res.log
    assert "I9 baro failure" in m["interlocks"] and m["land_by"] == "I9"
    k = np.flatnonzero(L["baro_lost"] & (L["state"] == lp.STATE_POWERED))
    assert k.size and np.all(L["phase"][k] == lp.PHASE_LAND)
    vp = VerticalParams.from_params(P, VEH.full.mass_kg, VEH.t_max_n)
    dref = np.diff(L["h_ref"][k])
    assert np.allclose(dref[np.isfinite(dref)], -vp.dn_um * 1e-6, atol=1e-9)       # descent_rate all the way, no flare
    # the altitude filter learnt a bias before the loss (pad calibration + flight) and keeps it while dead-reckoning
    kf = int(np.searchsorted(L["t"], FAULT_T))
    assert abs(L["alt_bias"][kf]) > 0.0 and np.all(L["alt_bias"][k] == L["alt_bias"][kf])


def test_baro_lost_landing_reaches_the_ground(baro_run):
    """v0.7.0 pinned a fly-away here (strict xfail: the inertial v_z integrated the 10 mg ASSUMED accelerometer bias, the
    vehicle climbed while it believed it descended, only V6 ended it).  v0.7.1 (learnt z-bias, descending reference on the
    dead reckoning, V8 / V9 time limit) lands this seed; the 20-seed study (sim/out/vtol/vtol_metrics.json
    studies.baro_fail_seeds) has no run climbing more than 2 m after the loss - the residual is V9 ABORTs at height."""
    res, m = baro_run
    L = res.log
    assert m["landed_normally"] and abs(m["touchdown_vz"]) <= vs.NOMINAL_CRITERIA["touchdown_vz_max_mps"]
    assert not any(c in m["interlock_codes"] for c in (lp.R_V_HARD_CUTOFF, lp.R_V_BARO_LIMIT_ABORT))
    kf = int(np.searchsorted(L["t"], FAULT_T))
    assert L["h_true"][kf:].max() <= L["h_true"][kf] + 1.0          # no climb away after the loss


def test_flight_time_limit_lands_before_the_hard_cutoff():
    sc = hop("T_v2", t_end=70.0, script=HOLD_ONLY)
    res, m = vs.run_scenario(sc)
    verdict, why = vs.judge(m, {"expect": "V2", "outcome": "landed", "forbid": "V6"})
    assert verdict == "PASS", why
    assert m["t_land_s"] - m["t_motor_start_s"] == pytest.approx(V["max_flight_s"] - V["land_reserve_s"], abs=0.01)


# --------------------------------------------------------------------------- #
# renderer log
# --------------------------------------------------------------------------- #
RENDER_KEYS = ("t", "pos", "q", "vel", "delta", "throttle", "thrust", "state", "phase", "h_target", "h_ref", "h_est",
               "tilt_deg", "wind", "gust_active", "ground_z", "event_t", "event_text", "cmd_t", "cmd_name", "meta",
               "wind_mean")


def test_export_npz_follows_the_renderer_contract(hop_run, tmp_path):
    res, m = hop_run
    path = tmp_path / "hop.npz"
    meta = vs.export_npz(res, str(path), VEH, {"verdict": "PASS"})
    with np.load(path, allow_pickle=False) as d:
        keys = set(d.files)
        assert set(RENDER_KEYS) <= keys
        n = d["t"].shape[0]
        assert d["pos"].shape == (n, 3) and d["q"].shape == (n, 4) and d["delta"].shape == (n, 4)
        assert d["wind"].shape == (n, 3) and d["phase"].shape == (n,) and d["h_target"].shape == (n,)
        assert np.all(np.diff(d["t"]) > 0) and float(d["ground_z"]) <= 0.0
        assert list(d["cmd_name"]) == ["ON", "OFF"]
        assert d["event_t"].shape[0] == len(d["event_text"])
        mj = json.loads(str(d["meta"]))
        # the exit-plane position starts on the pad top at the origin
        assert np.allclose(d["pos"][0], 0.0, atol=1e-9)
    for k in ("scenario", "title", "seed", "mode", "params_sha256", "notes", "pos_ref", "z_cg_m", "t_max_n", "limits",
              "poster_t"):
        assert k in mj
    assert mj["mode"] == "vtol" and mj["pos_ref"] == "exit_plane" and mj["verdict"] == "PASS"
    assert "ground-effect" in mj["notes"]
    assert meta["scenario"] == "T_hop"
    # the renderer's own loader reads it without a warning
    spec = importlib.util.spec_from_file_location("sim_video_t", os.path.join(PROJECT_ROOT, "tools", "sim_video.py"))
    sv = importlib.util.module_from_spec(spec)
    sys.modules["sim_video_t"] = sv
    try:
        spec.loader.exec_module(sv)
    finally:
        sys.modules.pop("sim_video_t", None)
    log = sv.load_log(str(path))
    assert log.warnings == []


# --------------------------------------------------------------------------- #
# scenario set, tuning helpers, campaign
# --------------------------------------------------------------------------- #
def test_scenario_set_has_every_safety_case():
    names = vs.scenario_names(P)
    for n in ("V_nominal", "V_nominal_ideal_att", "V_nominal_pad_gyro", "V_nominal_wind", "V_wind_swing",
              "V_wind_reversal", "V_gusty_swing", "V_ground_below", "V_link_loss", "V_baro_fail", "V_low_battery",
              "V_flight_time", "V_flight_time_alt_max", "V_stuck_vane", "V_imu_fail", "V_kill", "D_demo_nominal",
              "D_push_cg", "D_push_nose", "D_over_push", "D_over_push_slow", "D_baro_offset_uncal", "D_baro_offset_cal"):
        assert n in names
    assert any(n.startswith("V_gust_beyond_") for n in names)
    assert len(set(names)) == len(names)
    for sc, exp in vs.scenarios(P, None):
        assert exp.get("title_ko"), sc.name                       # Korean title for the video cards
        assert sc.mode == lp.MODE_VTOL
        assert sc.sensors.attitude_estimator in ("gated_mahony", "truth_noise", "pad_gyro")
        if sc.name.startswith("D_"):
            assert sc.profile == "demo"
    d = {sc.name: sc for sc, _ in vs.scenarios(P, None)}
    assert d["V_nominal"].sensors.attitude_estimator == "gated_mahony"
    # the demo nominal: 5 pushes 5-15 N, 0.2-0.4 s, from different directions, above the intake windows
    pu = d["D_demo_nominal"].pushes
    assert len(pu) == 5 and len({p_.azimuth_deg for p_ in pu}) == 5
    assert all(5.0 <= p_.force_n <= 15.0 and 0.2 <= p_.duration_s <= 0.4 for p_ in pu)
    assert all(p_.z_body_m > VEH.layout.z_window_top * 1e-3 for p_ in pu)
    # same impulse at the CG height and at the nose
    assert d["D_push_cg"].pushes[0].impulse_ns == pytest.approx(d["D_push_nose"].pushes[0].impulse_ns)
    assert d["D_push_cg"].pushes[0].z_body_m == pytest.approx(VEH.z_cg_m)


def test_vertical_margins_of_the_assumed_gains():
    vp = VerticalParams.from_params(P, VEH.full.mass_kg, VEH.t_max_n)
    mg = vs.vertical_margins(vp, VEH.t_max_n, VEH.full.mass_kg, float(P["edf"]["thrust_time_constant_s"]),
                             vs.actuator_delay_s(P))
    # b = n T_max u_h^(n-1) / m
    assert mg["b_mps2_per_u"] == pytest.approx(2.0 * VEH.t_max_n * vp.u_hover / VEH.full.mass_kg)
    assert 30.0 < mg["vz_pm_deg"] < 90.0 and 30.0 < mg["alt_pm_deg"] < 90.0


def test_tuned_gains_file_meets_the_margin_constraint():
    if not os.path.exists(vs.VTOL_GAINS_JSON):
        pytest.skip("sim/out/vtol_gains.json not written yet (python sim/vtol_scenarios.py --tune)")
    vg, src = vs.load_vgains()
    assert set(vg) - {"_profiles", "_position"} == set(vs.GAIN_KEYS) and all(vg[k] > 0 for k in vs.GAIN_KEYS)
    if "_position" in vg:
        # v0.8: the flown position gains: the tuner's candidate when it met the margin rule AND passed the validation
        # gate, else the prototype gains with the gate's record (v0.8: the candidate did not pass - the prototype stays)
        assert set(vg["_position"]) == set(vs.PGAIN_KEYS)
        hm = vs.horizontal_margins(vg["_position"])
        assert hm["phase_margin_deg"] > 0.0 and hm["gain_margin_x"] > 1.0                  # a stable linear loop
        with open(vs.VTOL_GAINS_JSON, "r", encoding="utf-8") as f:
            mp = json.load(f)["_meta_position"]
        if hm["phase_margin_deg"] < vs.PM_MIN_H_DEG:
            assert vg["_position"] == vs.prototype_pgains() and "validation_gate" in mp
        cm = vs.horizontal_margins(mp["candidate"]) if "candidate" in mp else hm
        assert cm["phase_margin_deg"] >= vs.PM_MIN_H_DEG and cm["gain_margin_x"] >= vs.GM_MIN_H_X
    # the estimator gains keep the Routh margin of the drag-lag loop
    assert vs.estimator_feasible(vg, vtol_aero_geometry(VEH)["drag_time_constant_hover_s"])
    vp = VerticalParams.from_params(P, VEH.full.mass_kg, VEH.t_max_n, **{k: vg[k] for k in vs.VGAIN_KEYS})
    mg = vs.vertical_margins(vp, VEH.t_max_n, VEH.full.mass_kg, float(P["edf"]["thrust_time_constant_s"]),
                             vs.actuator_delay_s(P))
    assert mg["vz_pm_deg"] >= vs.PM_MIN_DEG and mg["alt_pm_deg"] >= vs.PM_MIN_DEG


def test_campaign_sample_is_a_pure_function_of_seed_and_index():
    a, b = vc.sample(2026, 3), vc.sample(2026, 3)
    assert a == b
    assert vc.sample(2026, 4) != a and vc.sample(2027, 3) != a
    assert vc.sample(2026, 3, profile="demo") != a
    for i in range(40):
        s = vc.sample(1, i)
        for k in ("wind_mps", "packet_loss", "mass_scale", "thrust_scale", "batt_v_rest_cell", "initial_tilt_deg",
                  "crossflow_cd_scale", "ram_drag_scale", "vibration_g", "baro_thrust_offset_m"):
            lo, hi = vc.SAMPLING[k]
            assert lo <= s[k] <= hi
        assert 0 <= len(s["gusts"]) <= 4 and s["radio"] in vc.RADIOS
        assert s["t_off_s"] > s["t_down_s"]
        # fix round: the CG residual is drawn around the CAD lateral CG; the thrust spread, the voltage exponent and the
        # swirl scale are sampled
        cad = vs.cad_cg_offset_mm()
        assert math.hypot(s["cg_offset_mm"][0] - cad[0], s["cg_offset_mm"][1] - cad[1]) <= vc.SAMPLING["cg_offset_mm"][1] + 1e-9
        for k in ("thrust_voltage_exp", "swirl_scale"):
            lo, hi = vc.SAMPLING[k]
            # fix round: the swirl SIGN is drawn on its own stream (SAMPLING_FIX9 p_swirl_negative), the magnitude as before
            assert lo <= (abs(s[k]) if k == "swirl_scale" else s[k]) <= hi
        assert s["wind_swing_period_s"] <= vc.SAMPLING["wind_swing_period_s"][1]
        assert abs(s["baro_thrust_offset_m"] - s["baro_thrust_comp_m"]) <= 0.3 + 1e-12
        assert 0.0 <= s["wind_swing_deg"] <= vc.SAMPLING["wind_swing_deg"][1]
        sc = vc.build_scenario(P, s, None)
        assert sc.mode == lp.MODE_VTOL and sc.seed == s["seed"] and sc.sensors.attitude_estimator == "gated_mahony"
        assert sc.pushes == () and sc.profile is None
        d = vc.sample(1, i, profile="demo")
        D = vc.SAMPLING_DEMO
        assert d["gusts"] == [] and d["wind_mps"] <= D["wind_mps"][1] and d["wind_swing_deg"] == 0.0
        assert len(d["pushes"]) <= D["n_pushes"][1]
        t_prev_end = -1.0
        for q in d["pushes"]:
            assert D["push_force_n"][0] <= q["force_n"] <= D["push_force_n"][1]
            assert D["push_duration_s"][0] <= q["duration_s"] <= D["push_duration_s"][1]
            assert D["push_z_body_m"][0] <= q["z_body_m"] <= D["push_z_body_m"][1]
            assert q["t0_s"] >= t_prev_end + D["push_gap_s"] - 1e-9 and q["t0_s"] + q["duration_s"] <= d["t_off_s"] - 1.0
            t_prev_end = q["t0_s"] + q["duration_s"]
        dsc = vc.build_scenario(P, d, None)
        assert dsc.profile == "demo" and len(dsc.pushes) == len(d["pushes"])
        assert dsc.thrust_voltage_exp == d["thrust_voltage_exp"] and dsc.swirl_scale == d["swirl_scale"]
        # the live demo as planned: 5-15 N x 0.2-0.4 s between the window top and the same distance above the CG
        lt = vc.sample(1, i, profile="demo", pushes="light")
        lo, hi = vc.light_push_band_m()
        # v0.9 VTOL vehicle: the window top is above the CG - the band is the avionics section above the windows
        assert lo == pytest.approx(VEH.layout.z_window_top * 1e-3 + 0.01) and hi == pytest.approx(vs.vtol_push_top_m())
        for q in lt["pushes"]:
            assert 5.0 <= q["force_n"] <= 15.0 and 0.2 <= q["duration_s"] <= 0.4 and lo <= q["z_body_m"] <= hi
        # every variable but the pushes is the same draw as the "any" set (common random numbers)
        assert {k: v for k, v in lt.items() if k not in ("pushes", "push_set")} == {k: v for k, v in d.items() if k != "pushes"}
    with pytest.raises(ValueError):
        vc.sample(1, 0, profile="demo", pushes="hard")


def test_campaign_run_is_reproducible_and_writes_the_tracks(tmp_path):
    r1 = vc.run_one((5, 1, "pad_gyro", None))
    r2 = vc.run_one((5, 1, "pad_gyro", None))
    t1, t2 = r1.pop("_track"), r2.pop("_track")
    r1.pop("runtime_s")
    r2.pop("runtime_s")
    assert r1 == r2 and all(np.array_equal(t1[k], t2[k]) for k in t1)
    row = vc.flat_row(dict(r1, runtime_s=0.0))
    assert set(row) == set(vc.CSV_KEYS) and row["verdict"] in ("PASS", "FAIL")
    assert row["outcome"] in vc.OUTCOME_NAMES
    s = vc.summarize([dict(r1, runtime_s=0.0)])
    assert s["n"] == 1 and set(s["check_pass_rate"]) == set(r1["checks"])
    # tracks.npz in the renderer's contract (tools/campaign_video.load_tracks reads it)
    path = vc.write_tracks(str(tmp_path / "tracks.npz"), [dict(r1, _track=t1, runtime_s=0.0)],
                           {"title": "t", "profile": "field"})
    sys.path.insert(0, os.path.join(PROJECT_ROOT, "tools"))
    try:
        cv = importlib.import_module("campaign_video")
        tr = cv.load_tracks(path)
    finally:
        sys.path.remove(os.path.join(PROJECT_ROOT, "tools"))
    assert tr.pos.shape[0] == 1 and tr.pos.shape[2] == 3 and tr.t.size >= 2
    assert np.allclose(np.diff(tr.t), 1.0 / vc.TRACK_HZ)


# --------------------------------------------------------------------------- #
# v0.7.1: drag model, pushes, wind direction, timing, estimator, baro offset
# --------------------------------------------------------------------------- #
def test_vtol_aero_geometry_follows_the_params_note():
    ag = vtol_aero_geometry(VEH)
    L = VEH.layout
    assert ag["a_plan_m2"] == pytest.approx(P["body"]["od"] * (L.z_cap_top - L.z_exit) * 1e-6)     # v0.9: to the top cap
    # the silhouette of the body of revolution agrees with OD x length within 10 % (v0.9 VTOL: the 475 mm body makes the
    # two fin planforms 21 % of it, so the body + fins silhouette is 16 % above OD x length - the params A_plan definition
    # is kept, the campaign spreads C_dc +-30 %)
    assert ag["a_silhouette_body_m2"] == pytest.approx(ag["a_plan_m2"], rel=0.1)
    assert ag["a_silhouette_body_2fins_m2"] == pytest.approx(ag["a_silhouette_body_m2"] + 2.0 * ag["a_fin_planform_m2"])
    assert L.z_window_bottom * 1e-3 < ag["z_intake_m"] < L.z_window_top * 1e-3
    mdot = math.sqrt(P["aero"]["rho_kg_m3"] * VEH.a_e_m2 * VEH.full.mass_kg * 9.80665)
    assert ag["mdot_hover_kg_s"] == pytest.approx(mdot)
    # fix round (physics review, medium 2): the ram drag acts at the intake; the vane side force that holds its moment
    # - net damping mdot (1 + l_in / l_v); v0.9 VTOL configuration: the intake is 13 mm ABOVE the CG (rocket: 108 mm below),
    # factor 1.06, tau_d ~ 4.9 s (rocket 9.55 s)
    l_in = ag["z_intake_m"] - VEH.z_cg_m
    assert 0.0 < l_in < 0.03 and ag["vane_counterforce_factor_ram"] == pytest.approx(1.0 + l_in / VEH.l_v_m)
    assert ag["drag_derivative_hover_N_per_mps"] == pytest.approx(mdot * (1.0 + l_in / VEH.l_v_m))
    assert ag["drag_time_constant_hover_s"] == pytest.approx(VEH.full.mass_kg / (mdot * (1.0 + l_in / VEH.l_v_m)))
    assert ag["drag_time_constant_ram_only_s"] == pytest.approx(VEH.full.mass_kg / mdot)
    assert 4.5 < ag["drag_time_constant_hover_s"] < 5.5 and 4.8 < ag["drag_time_constant_ram_only_s"] < 5.6
    assert ag["drag_time_constant_at_1mps_s"] < ag["drag_time_constant_hover_s"]      # cross-flow adds damping in wind
    assert body_radius_mm(L, 300.0) == pytest.approx(L.body_r) and body_radius_mm(L, 50.0) == pytest.approx(L.vane_section_r)
    assert body_radius_mm(L, L.z_cap_top - 1.0) == pytest.approx(L.body_r) and body_radius_mm(L, L.z_cap_top + 1.0) == 0.0


def test_push_profile_impulse_and_contact():
    pu = Push(2.0, 0.4, 10.0, azimuth_deg=90.0, z_body_m=0.4)
    assert pu.factor(1.99) == 0.0 and pu.factor(2.2) == pytest.approx(1.0) and pu.factor(2.4) == 0.0
    assert pu.impulse_ns == pytest.approx(10.0 * 0.4 * 2.0 / math.pi)
    fx, fy, fz = pu.force_world(2.2)
    assert fx == pytest.approx(0.0, abs=1e-12) and fy == pytest.approx(10.0) and fz == 0.0
    tr = Push(0.0, 1.0, 4.0, profile="trapezoid", rise_s=0.25)
    assert tr.factor(0.125) == pytest.approx(0.5) and tr.factor(0.5) == 1.0 and tr.impulse_ns == pytest.approx(3.0)
    assert push_with_impulse(2.0, 0.3, t0_s=1.0).impulse_ns == pytest.approx(2.0)
    with pytest.raises(ValueError):
        Push(0.0, 0.3, 5.0, profile="square")
    fs = FlightSim(VEH, hop71(pushes=(pu,), cg_offset_mm=(0.0, 0.0)))
    q0 = np.array([1.0, 0.0, 0.0, 0.0])
    rx, ry, rz = fs.push_contact_body(pu, q0)
    # the force pushes toward +y: the person stands at -y, the contact point is on the -y surface
    assert ry == pytest.approx(-VEH.layout.body_r * 1e-3) and abs(rx) < 1e-12
    assert rz == pytest.approx(0.4 - fs.cfg_full.z_cg)
    off = Push(2.0, 0.4, 10.0, azimuth_deg=90.0, z_body_m=0.4, offset_deg=30.0)
    rx2, ry2, _ = fs.push_contact_body(off, q0)
    assert math.hypot(rx2, ry2) == pytest.approx(VEH.layout.body_r * 1e-3) and abs(rx2) > 0.01
    # fix round: the contact point is relative to the CG, which sits cg_offset_mm off the body axis
    fo = FlightSim(VEH, hop71(pushes=(pu,), cg_offset_mm=(2.0, -1.0)))
    rx3, ry3, rz3 = fo.push_contact_body(pu, q0)
    assert (rx3, ry3, rz3) == pytest.approx((rx - 2e-3, ry + 1e-3, rz))


@pytest.fixture(scope="module")
def push_runs():
    """One push in the demo hover at the CG height and at the nose (same impulse); CG on the body axis (the geometric
    checks below; the CAD offset is covered by test_push_profile_impulse_and_contact)."""
    out = {}
    for name, z in (("cg", VEH.z_cg_m), ("nose", vs.vtol_push_top_m())):          # v0.9: "nose" = the top cap
        pu = push_with_impulse(1.5, 0.3, t0_s=9.0, azimuth_deg=0.0, z_body_m=z)
        sc = vs.scenario_demo(P, None, name=f"T_push_{name}", pushes=(pu,), vtol_script=vs.demo_script(t_off=None),
                              t_end=14.0, cg_offset_mm=(0.0, 0.0), gps=None)
        out[name] = vs.run_scenario(sc)
    return out


def test_push_is_logged_and_pushes_the_vehicle(push_runs):
    res, m = push_runs["cg"]
    L = res.log
    act = L["push_active"]
    assert act.any() and np.all(L["push_force"][~act] == 0.0) and np.all(np.isnan(L["push_point"][~act]))
    imp = float(np.sum(L["push_force"][:, 0]) * (L["t"][1] - L["t"][0]))
    assert imp == pytest.approx(1.5, rel=0.02)                   # the logged force carries the push impulse
    r = m["pushes"][0]
    assert r["in_flight"] and r["peak_horiz_disp_m"] > 0.1      # no position hold: the push moves the vehicle
    # the contact point lies on the body surface at the push height
    k = int(np.flatnonzero(act)[len(np.flatnonzero(act)) // 2])
    pe = vs.exit_plane_positions(res)[k]
    rb = quat.to_matrix(L["q"][k]).T @ (L["push_point"][k] - pe)          # body frame, from the exit-plane centre
    assert rb[2] == pytest.approx(VEH.z_cg_m, abs=1e-6)
    assert math.hypot(rb[0], rb[1]) == pytest.approx(VEH.layout.body_r * 1e-3, abs=1e-6)


def test_push_far_from_the_cg_tilts_more_than_at_the_cg(push_runs):
    _, mc = push_runs["cg"]
    _, mn = push_runs["nose"]
    assert mn["pushes"][0]["peak_dev_deg"] > 3.0 * mc["pushes"][0]["peak_dev_deg"]
    # a central push (no tangential offset) has no lever arm about the long axis: the contact slides with the body
    assert mc["pushes"][0]["max_yaw_rate_deg_s"] < 60.0


def test_push_contact_follows_the_attitude():
    """The contact point stays on the side facing the person whatever the body's yaw (sliding contact): the force line
    passes through the axis for offset 0, and has the lever r sin(offset) about it otherwise (CG on the axis)."""
    fs = FlightSim(VEH, hop71(cg_offset_mm=(0.0, 0.0)))
    r = VEH.layout.body_r * 1e-3
    for yaw in (0.0, 30.0, 135.0):
        q = quat.from_yaw(math.radians(yaw))
        for off in (0.0, 20.0):
            pu = Push(1.0, 0.3, 10.0, azimuth_deg=60.0, z_body_m=0.45, offset_deg=off)
            rb = np.array(fs.push_contact_body(pu, q))
            f_b = quat.to_matrix(q).T @ np.array(pu.force_world(1.15))
            mz = rb[0] * f_b[1] - rb[1] * f_b[0]
            assert mz == pytest.approx(r * 10.0 * math.sin(math.radians(off)) * pu.factor(1.15), abs=1e-9)


def test_wind_mean_swing_reversal_and_walk():
    fs = FlightSim(VEH, hop(wind_mps=4.0, wind_dir_deg=10.0, wind_swing_deg=75.0, wind_swing_period_s=30.0))
    for t in (0.0, 7.5, 15.0, 22.5):
        wx, wy = fs.wind_mean_vtol(0, t, None)
        d = math.degrees(math.atan2(wy, wx))
        assert math.hypot(wx, wy) == pytest.approx(4.0)
        assert d == pytest.approx(10.0 + 75.0 * math.sin(2 * math.pi * t / 30.0), abs=1e-9)
    fr = FlightSim(VEH, hop(wind_mps=3.0, wind_reversal_t_s=5.0, wind_reversal_s=4.0))
    assert fr.wind_mean_vtol(0, 4.9, None) == pytest.approx((3.0, 0.0))
    assert fr.wind_mean_vtol(0, 9.1, None)[0] == pytest.approx(-3.0)
    assert fr.wind_mean_vtol(0, 7.0, None)[1] == pytest.approx(3.0)        # half way: 90 deg, counter-clockwise
    fw = FlightSim(VEH, hop(wind_mps=2.0, wind_dir_rate_sigma_dps=3.0))
    walk = fw.wind_direction_walk(2500)
    assert walk[0] == 0.0 and np.std(np.diff(walk)) > 0.0 and np.max(np.abs(walk)) < 180.0
    assert np.array_equal(walk, fw.wind_direction_walk(2500))
    fm = FlightSim(VEH, hop(wind_mps=2.0, wind_speed_mod_frac=0.25, wind_speed_mod_period_s=8.0))
    assert math.hypot(*fm.wind_mean_vtol(0, 2.0, None)) == pytest.approx(2.5)


def test_vtol_timing_vane_and_esc_paths():
    tim = FCTiming()
    dt = 1.0 / P["control"]["loop_hz"]
    assert tim.delay_s(dt) == FC_COMPUTE_DELAY_S_MEASURED == 4.71e-3
    assert tim.esc_delay_s(dt) == pytest.approx(FC_COMPUTE_DELAY_S_MEASURED + FC_VTOL_ESC_EXTRA_S_ASSUMED)
    fs = FlightSim(VEH, hop())
    res = fs.run()
    assert fs.latch.d_s == pytest.approx(4.71e-3) and fs.latch_esc.d_s == pytest.approx(tim.esc_delay_s(dt))
    assert fs.latch.n_ch == 4 and fs.latch_esc.n_ch == 1
    # v0.7.1 order: the attitude controller's gain schedule sees the previous tick's ESC throttle
    L = res.log
    assert np.array_equal(L["att_u"][1:], L["throttle"][:-1])


def test_baro_thrust_offset_uncalibrated_hovers_low():
    """A static-pressure error with the fan running (plant) that the firmware does not compensate makes the vehicle hover
    lower by about the offset (the baro reads high)."""
    sc = vs.scenario_demo(P, None, name="T_baro_off", pushes=(), vtol_script=vs.demo_script(t_off=None), t_end=14.0,
                          baro_thrust_offset_m=0.8, baro_thrust_comp_m=0.0)
    res, m = vs.run_scenario(sc)
    L = res.log
    hold = (L["phase"] == lp.PHASE_HOLD) & (L["t"] > 10.0)
    assert np.mean(L["h_true"][hold] - L["h_ref"][hold]) == pytest.approx(-0.8, abs=0.2)
    assert res.vtol["plant"]["baro_thrust_offset_m"] == 0.8 and res.vtol["alt_filter"]["comp_m"] == 0.0


def test_demo_profile_hovers_at_its_altitude():
    sc = vs.scenario_demo(P, None, name="T_demo", pushes=(), vtol_script=vs.demo_script(t_off=None), t_end=12.0)
    res, m = vs.run_scenario(sc)
    assert res.vtol["profile"] == "demo" and m["first_target_m"] == pytest.approx(1.5)
    assert m["interlock_codes"] == []
    hold = res.log["phase"] == lp.PHASE_HOLD
    assert abs(np.mean(res.log["h_true"][hold]) - 1.5) < 0.3


# --------------------------------------------------------------------------- #
# v0.7.1 fix round (physics review + v0.7.1b control fix)
# --------------------------------------------------------------------------- #
def test_vtol_flies_the_vtol_attitude_overrides_and_ballistic_does_not():
    """control.vtol_overrides (v0.7.1b: rate_i_max_z 0.25) reach the VTOL attitude controller; the ballistic build keeps
    the params / gains.json value bit for bit (its digests above)."""
    ov = vtol_overrides(P["control"])
    assert ov and "rate_i_max_z" in ov
    g = vs.attitude_gains(P)
    fv = FlightSim(VEH, hop(), g)
    fb = FlightSim(VEH, Scenario(name="b"), g)
    assert fv.gains["rate_i_max_z"] == ov["rate_i_max_z"] and fv.ctrl.g["rate_i_max_z"] == ov["rate_i_max_z"]
    assert fb.gains == gains_from_control(P["control"], g)
    res, m = vs.run_scenario(hop(t_end=12.0, script=HOLD_ONLY))
    assert res.vtol["attitude_gains"]["rate_i_max_z"] == ov["rate_i_max_z"]


def test_cad_cg_offset_is_the_vtol_default():
    c = VEH.full.com_m
    assert vs.cad_cg_offset_mm() == pytest.approx((c[0] * 1e3, c[1] * 1e3))
    assert 0.3 < math.hypot(*vs.cad_cg_offset_mm()) < 3.0                     # CAD: v0.8 0.92 mm (GPS pod), v0.7.1 2.57 mm
    assert hop().cg_offset_mm == pytest.approx(vs.cad_cg_offset_mm())
    assert Scenario(name="b").cg_offset_mm == (0.0, 0.0)                       # ballistic default unchanged
    d = {sc.name: sc for sc, _ in vs.scenarios(P, None)}
    assert d["V_nominal"].cg_offset_mm == pytest.approx(vs.cad_cg_offset_mm())
    assert d["V_nominal_ideal_att_axis_cg"].cg_offset_mm == (0.0, 0.0)
    assert d["V_wind_swing"].gusts == () and d["V_wind_reversal"].gusts == ()   # no inherited step gusts


def test_swirl_scale_scales_the_plant_swirl_only_in_vtol():
    f1 = FlightSim(VEH, hop())
    for sw in (0.0, 2.0):
        fs = FlightSim(VEH, hop(swirl_scale=sw))
        assert fs.swirl_per_N == pytest.approx(sw * f1.swirl_per_N)
    assert FlightSim(VEH, Scenario(name="b", swirl_scale=0.0)).swirl_per_N == f1.swirl_per_N   # ballistic: no-op


def test_estimator_rules_use_the_net_drag_time_constant():
    tau = vtol_aero_geometry(VEH)["drag_time_constant_hover_s"]
    # the v0.7.1 winner fails both rules with tau_d 9.55 s (physics review: Routh 1.57 < 2, kp tau_d 1.43 > 1)
    assert not vs.estimator_feasible({"est_kp": 0.15, "est_ki": 0.01}, 9.55)
    assert vs.estimator_feasible({"est_kp": 0.1, "est_ki": 0.002}, 9.55)
    assert not vs.estimator_feasible({"est_kp": 0.12, "est_ki": 0.001}, 9.55)          # 1 / kp < tau_d
    lp_ = vs.estimator_loop({"est_kp": 0.15, "est_ki": 0.01}, 9.55)     # the physics review's numbers at its tau_d 9.55 s
    assert lp_["period_s"] == pytest.approx(54.3, abs=0.5) and lp_["zeta"] == pytest.approx(0.118, abs=0.005)
    assert 4.5 < tau < 5.5                       # v0.9 VTOL configuration 4.89 s (v0.8 rocket with the GPS pod: 9.84 s)


def test_landed_by_names_follow_the_controller():
    assert vs.DETECTOR_SIGNATURES == tuple(v for k, v in sorted(TD_NAMES.items()) if k)


def test_thrust_learning_is_logged_and_learns_the_plant(hop_run):
    res, m = hop_run
    L = res.log
    assert L["vc_k_thrust"].shape == L["t"].shape and L["vc_kt_learn"].any()
    # v0.9: the plant's thrust ratio over HOLD includes the voltage term (the fresh pack: > 1)
    assert m["plant_thrust_ratio_hold"] > 1.05
    assert m["kt_hold_mean"] == pytest.approx(m["plant_thrust_ratio_hold"], abs=0.05)
    assert m["lifted_off"] and not m["no_liftoff"]
    assert m["landed_by"] in vs.DETECTOR_SIGNATURES


def test_strong_plant_lands_on_the_ground_by_the_detector():
    """Physics review HIGH (fixed by v0.7.1b): a plant x1.25 stronger than the model stopped the motor 6-8 m up; now
    k_T is learnt in flight and the landing is confirmed on the ground by a touchdown signature."""
    sc = hop("T_strong", thrust_scale=vs.PLANT_STRONG, t_end=45.0,
             script=(vs.OpStep("on", t=1.0), vs.OpStep("off", phase="HOLD", after=4.0)))
    res, m = vs.run_scenario(sc)
    verdict, why = vs.judge(m, {"expect": "", "outcome": "landed", "vertical": True, "forbid": vs.PLANT_FORBID})
    assert verdict == "PASS", why
    assert not m["false_touchdown"] and m["height_at_landed_m"] <= 0.05
    assert m["kt_hold_mean"] == pytest.approx(vs.PLANT_STRONG, abs=0.06)


def test_too_weak_plant_never_lifts_off_and_stays_safe():
    """Thrust x0.80 (T/W < 1): the motor runs on the pad (CLIMB never reaches HOLD, so the operator's OFF never comes);
    v0.7.1: V2 lands it after the flight-time budget, LANDED -> SAFE on the pad; v0.8: the take-off check V15 stops the
    motor after takeoff_check within_s at the ceiling -> SAFE.  Both: no ABORT, no release; the campaign files it as
    'other' (no lift-off)."""
    nl = vs.plant_no_lift_scale()                       # v0.9: T/W 0.93 on the light VTOL vehicle (x0.80 is T/W 1.5)
    for sc in (hop71("T_weak71", thrust_scale=nl, thrust_voltage_exp=1e-9, t_end=50.0),
               hop("T_weak", thrust_scale=nl, thrust_voltage_exp=1e-9, t_end=50.0)):
        res, m = vs.run_scenario(sc)
        assert m["no_liftoff"] and not m["lifted_off"] and m["cannot_fly"]
        assert m["t_abort_s"] is None and m["t_released_s"] is None
        assert res.final_state == State.SAFE
        assert vc.outcome_of(m, "FAIL") == vc.OUTCOME_NAMES.index("other")
        v15 = lp.R_V_TAKEOFF_CHECK in m["interlock_codes"]
        assert v15 == (sc.gps is not None)
        if v15:
            tc = V["position_hold"]["takeoff_check_ASSUMED"]
            # V15 once the ramp is over and the loop sat at u_max for within_s (+ the spool warning before the motor)
            assert m["motor_time_s"] <= P["mission"]["throttle_ramp_s"] + tc["within_s"] + 0.1


def test_rate_abort_cause_splits_tilt_rate_and_spin():
    """The I3 reason code is the same for both: the cause comes from the true rates over the persistence window."""
    pu = Push(9.0, 0.4, 25.0, azimuth_deg=0.0, z_body_m=0.80)
    sc = vs.scenario_demo(P, None, name="T_overpush", pushes=(pu,), vtol_script=vs.demo_script(t_off=None), t_end=13.0)
    res, m = vs.run_scenario(sc)
    assert m["t_abort_s"] is not None
    if lp.R_I3_RATE in m["interlock_codes"]:
        assert m["abort_rate_cause"] in ("tilt_rate", "spin", "tilt_rate+spin", "gyro_saturation")
        d = m["abort_rate_detail"]
        assert d["window_ticks"] >= res.vtol["rate_abort"]["rate_persist_ticks"]
        r = m["pushes"][0]
        assert r["abort_cause"] == m["abort_rate_cause"]
        assert r["max_rate_xy_deg_s"] > 0.0 and r["max_rate_z_deg_s"] == r["max_yaw_rate_deg_s"]


def test_envelope_by_height_limit_rule():
    rows = []
    for imp, out in ((0.5, "recovered"), (1.0, "recovered"), (1.5, "abort"), (2.0, "recovered")):
        for du in vs.ENVELOPE_DURATIONS_S:
            rows.append({"z_body_m": 0.05, "impulse_ns": imp, "duration_s": du, "outcome": out, "interlocks": ["I3"],
                         "abort_cause": "spin" if out == "abort" else None, "peak_tilt_deg": 1.0,
                         "max_rate_xy_deg_s": 10.0, "max_rate_z_deg_s": 5.0})
    byz = vs.envelope_by_height(rows, (0.21, 0.33))
    r = byz["0.050"]
    assert r["max_recovered_impulse_ns"] == 1.0 and r["first_abort_impulse_ns"] == 1.5
    assert r["abort_causes"] == ["spin"] and r["first_abort_causes"] == ["spin"] and not r["in_window_keepout"]


# --------------------------------------------------------------------------- #
# v0.8: GPS position hold (receiver model, NAV order, FCU position loop, rules, metrics, exports, campaign)
# --------------------------------------------------------------------------- #
@pytest.fixture(scope="module")
def gps_hop_run():
    return vs.run_scenario(hop("T_gps_hop", t_end=40.0, script=(vs.OpStep("on", t=1.0),
                                                              vs.OpStep("off", phase="HOLD", after=8.0))))


def test_build_flags_select_the_v08_build_and_the_v071_fallback():
    """GPS set: the v0.8 NAV order, the take-off check, the supervisor's GPS rules (ON gate, V11-V14, TELEM v3);
    gps=None: the v0.7.1 build; ``gps_off``: the v0.8 build without the GPS (indoor fallback / what-if)."""
    g = FlightSim(VEH, hop())
    assert g.gps_on and g.v08 and g.tko and g.sp.gps_hold and g.sp.takeoff_check and g.pp is not None
    assert g.fcu.pos is not None and g.vp.takeoff_ticks > 0
    o = FlightSim(VEH, vs.gps_off(hop()))
    assert not o.gps_on and o.v08 and o.tko and not o.sp.gps_hold and o.sp.takeoff_check and o.fcu.pos is None
    n = FlightSim(VEH, hop71())
    assert not (n.gps_on or n.v08 or n.tko or n.sp.gps_hold or n.sp.takeoff_check) and n.vp.takeoff_ticks == 0
    with pytest.raises(ValueError):
        FlightSim(VEH, hop(nav_v08=False))
    # the position params: model T/W from the CAD vehicle (GPS pod included), the profile's cap and fence
    pp = g.pp
    assert pp.tw == pytest.approx(VEH.t_max_n / (VEH.full.mass_kg * 9.80665))
    assert pp.tilt_cap_deg == V["position_hold"]["tilt_cap_deg"]["field"] and pp.fence_m == V["position_hold"]["fence_m"]["field"]
    d = FlightSim(VEH, hop(profile="demo"))
    assert d.pp.tilt_cap_deg == V["position_hold"]["tilt_cap_deg"]["demo"] and d.pp.fence_m == V["position_hold"]["fence_m"]["demo"]


def test_gps_receiver_uart_model_and_boot_configuration():
    """The receiver (plant): factory 9600 baud + NMEA, the NAV's configurator probes 38400 (no answer), configures at
    9600 -> 38400, CFG-RATE 5 Hz, CFG-MSG x3, CFG-NAV5 (every ACK checked), MON-VER u-blox 7 (the NEO-7M class default, 2026-11-10); then NAV-POSLLH / SOL /
    VELNED per epoch, the last byte latency_s after the measurement."""
    gp = gn.NavGpsParams.from_params(P)
    rx = GpsReceiverPlant(GpsSim(), seed=5, t_power=0.0)
    nav = gn.NavGps(gp, configure=True)
    baud = [gn.GPS_BAUD]
    dt = 1.0 / gp.loop_hz
    epochs = []
    parser, asm = gn.UbxParser(), gn.EpochAssembler()            # an independent copy of the NAV's epoch assembly
    for k in range(int(8.0 / dt)):
        t = k * dt
        rx.step(t, (1.0, 2.0, 0.5), (0.0, 0.0, 0.0))
        data = rx.deliver(t, baud[0])
        for msg in parser.feed(data):
            dm = gn.decode_msg(msg)
            ep = asm.add(dm, k) if dm is not None else None
            if ep is not None and ep.fix_ok:
                epochs.append((t, rx.t_of_tow(ep.itow_ms)))         # completion tick vs the measurement time
        acts, snap = nav.tick(k, data, (0.0, 0.0, 9.80665), (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0), True)
        for a in acts:
            if a[0] == "baud":
                baud[0] = int(a[1])
            else:
                rx.nav_tx(t, a[1], baud[0])
    cf = nav.cfg
    assert cf.done and cf.cfg_ok and cf.ver_ok and cf.ver.module == gn.MODULE_UBLOX7
    assert rx.baud == gn.GPS_BAUD and rx.meas_ms == 200 and not rx.nmea and len(rx.enabled) == 3
    assert rx.dyn == gn.DYN_AIRBORNE_1G and rx.stats["bytes_lost_baud"] > 0         # the factory NMEA at 9600 was lost
    # 5 Hz epochs once configured; each completes ~latency_s after its measurement (+ <= one 4 ms tick)
    assert len(epochs) >= 25
    lat = [ta - te for ta, te in epochs[-10:]]
    assert GpsSim().latency_s == 0.12 and all(0.12 - 1e-9 <= x <= 0.12 + dt + 1e-9 for x in lat)
    assert snap.flag(gn.GF_FIX_OK) and snap.flag(gn.GF_CFG_OK) and snap.flag(gn.GF_VER_OK)
    # a receiver that NAKs CFG-NAV5: the configuration is not OK (the flag the ground sees), the rest still works
    rx2 = GpsReceiverPlant(GpsSim(nak_ids=(gn.ID_CFG_NAV5,), backup_config=True), seed=5, t_power=0.0)
    nav2 = gn.NavGps(gp, configure=True)
    for k in range(int(5.0 / dt)):
        t = k * dt
        rx2.step(t, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        acts, _ = nav2.tick(k, rx2.deliver(t, gn.GPS_BAUD), (0.0, 0.0, 9.8), (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0), True)
        for a in acts:
            if a[0] == "tx":
                rx2.nav_tx(t, a[1], gn.GPS_BAUD)
    assert nav2.cfg.done and not nav2.cfg.cfg_ok and nav2.cfg.naks >= 1 and rx2.stats["cfg_naks"] >= 1
    # sky blocked: satellites x (1 - f), errors x sqrt(n_open / n); < 4 satellites: no fix
    assert GpsReceiverPlant(GpsSim(sky_blocked=0.34), 1, 0.0).n_sky == 6
    assert GpsReceiverPlant(GpsSim(sky_blocked=0.34), 1, 0.0).sky_scale == pytest.approx(math.sqrt(9 / 6))
    r6 = GpsReceiverPlant(GpsSim(sky_blocked=0.7, backup_config=True), 1, 0.0)
    r6.step(2.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert not r6.last["fix"]
    with pytest.raises(ValueError):
        GpsSim(receiver="NEO-7")
    m8 = neo_m8n()
    assert m8.rate_hz_max == 10.0 and m8.nav["rate_hz"] == 10.0 and m8.latency_s == 0.12


def test_gps_receiver_error_statistics():
    """The error model per axis: position = Gauss-Markov (pos_gm_sigma_m) + white, velocity white + Gauss-Markov - the
    sample statistics over many epochs match the stated sigmas (no steps, no quantisation beyond 1 cm)."""
    c = GpsSim(backup_config=True, step_rate_hz=0.0, pos_gm_tau_s=2.0, pos_gm_sigma_m=2.0, pos_white_m=0.3,
               vel_gm_sigma_mps=0.0, vel_white_mps=0.1)
    rx = GpsReceiverPlant(c, seed=3, t_power=-1.0)
    ep, ev = [], []
    for k in range(6000):
        rx.step(k * 0.2, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        if rx.last["fix"]:
            ep.append(rx.last["err_p"])
            ev.append(rx.last["err_v"])
    ep, ev = np.array(ep), np.array(ev)
    assert np.std(ep[:, 0]) == pytest.approx(math.hypot(2.0, 0.3), rel=0.1)
    assert np.std(ev[:, 1]) == pytest.approx(0.1, rel=0.1)


def test_gps_hop_holds_the_hold_point_and_lands_there(gps_hop_run):
    """A field hop with the GPS build: ON accepted with the gate, the hold point captured at the end of the climb, the
    position loop in POS mode through HOLD and LAND, station keeping / touchdown measured against the TRUE CG at the
    capture; TELEM v3 on the link; the tilt setpoint never beyond the cap (series tolerance)."""
    res, m = gps_hop_run
    L = res.log
    assert m["gps_build"] and m["interlock_codes"] == [] and not m["rejections"] and m["landed_normally"]
    g = res.vtol["gps"]
    assert g["configurator"]["cfg_ok"] and g["configurator"]["hw"] == gn.HW_UBLOX7
    assert len(g["hold_captures"]) == 1 and g["hold_captures"][0]["phase"] == "HOLD"
    hold = L["phase"] == lp.PHASE_HOLD
    assert np.all(L["pos_mode"][hold & (L["t"] > m["t_hold_s"] + 0.2)] == 2)       # MODE_POS
    assert m["t_hold_capture_s"] == pytest.approx(m["t_hold_s"], abs=0.05)
    assert np.allclose(L["hold_true"][hold & np.isfinite(L["hold_true"][:, 0])], g["hold_true_last"])
    for k in ("sk_rms_m", "sk_max_m", "nav_rms_m", "touchdown_from_hold_m", "touchdown_vh_ground"):
        assert m[k] is not None and m[k] >= 0.0
    assert m["nav_rms_m"] < m["sk_rms_m"] + 0.5               # the control error is the small part (GPS wander dominates)
    assert m["tilt_sp_max_deg"] <= m["tilt_cap_deg"] + vs.V08_TILT_SP_TOL_DEG
    assert res.link["budget"]["telem_frame_bytes"] == lp.TELEM7_FRAME_BYTES      # 2026-11-10: TELEM v3 + the logic tail
    assert res.vtol["gps"]["slot_sets"] > 100 and res.vtol["gps"]["slot_broken"] == 0
    # the frame the FCU flies on carries the previous tick's altitude (v0.8 NAV order)
    assert np.array_equal(L["h_pub"][1:], L["h_est"][:-1])
    # heading: the NAV starts at the pad azimuth; the vehicle was set up on it (error 0) - the error is the gyro drift
    assert abs(m["psi_err_start_deg"]) < 2.0


def test_gps_hop_export_follows_renderer_contract_v5(gps_hop_run, tmp_path):
    res, m = gps_hop_run
    path = tmp_path / "gps_hop.npz"
    meta = vs.export_npz(res, str(path), VEH, {"verdict": "PASS"})
    with np.load(path, allow_pickle=False) as d:
        n = d["t"].shape[0]
        for k, shp in (("hold_point", (n, 3)), ("pos_est", (n, 3)), ("gps_ok", (n,)), ("gps_sats", (n,)),
                       ("gps_hacc_m", (n,)), ("gps_fix", (n,)), ("tilt_sp_deg", (n,)), ("hold_point_est", (n, 3))):
            assert d[k].shape == shp, k
        hp = d["hold_point"]
        ok = np.isfinite(hp[:, 0])
        assert ok.any() and np.all(hp[ok, 2] == 0.0) and np.all(np.isnan(hp[~ok, 2]))
        assert set(np.unique(d["gps_fix"])) <= {0, 3}
        mj = json.loads(str(d["meta"]))
    assert mj["gps_on"] is True and meta["gps"]["module_hw"] == gn.HW_UBLOX7
    assert mj["criteria"]["station_keeping_max_m"] == V["position_hold"]["criteria_v08_ASSUMED"]["station_keeping_max_m"]
    assert "open sky" in mj["notes"].lower() or "OPEN SKY" in mj["notes"]
    spec = importlib.util.spec_from_file_location("sim_video_t5", os.path.join(PROJECT_ROOT, "tools", "sim_video.py"))
    sv = importlib.util.module_from_spec(spec)
    sys.modules["sim_video_t5"] = sv
    try:
        spec.loader.exec_module(sv)
    finally:
        sys.modules.pop("sim_video_t5", None)
    log = sv.load_log(str(path))
    assert log.warnings == [] and log.has_hold


def test_v08_criteria_are_the_params_numbers():
    c = vs.v08_criteria(P, "field")
    cv = V["position_hold"]["criteria_v08_ASSUMED"]
    assert c["station_keeping_max_m"] == cv["station_keeping_max_m"]["field"]
    assert vs.v08_criteria(P, "demo")["station_keeping_max_m"] == cv["station_keeping_max_m"]["demo"]
    assert (c["nav_frame_rms_m"], c["touchdown_ground_speed_mps"], c["touchdown_from_hold_point_m"],
            c["push_return_within_1m_s"]) == (cv["nav_frame_rms_m"], cv["touchdown_ground_speed_mps"],
                                              cv["touchdown_from_hold_point_m"], cv["push_3Ns_return_within_1m_s"])
    assert c["station_keeping_rms_p50_m"] == 1.0 and c["push_rule_max_impulse_ns"] == 3.0 and c["tip_margin_min_mps"] == 0.0
    assert c["nav_frame_rms_m"] == cv["controller_nav_frame_rms_m"]
    assert c["touchdown_vz_max_mps"] == 1.0 and c["touchdown_tilt_max_deg"] == 5.0 and c["hold_rms_err_max_m"] == 0.5


def test_pole_push_acts_along_the_pole():
    """v0.8 pole model: the force along the pole from the hand (hand_height_m above the ground) to the contact point
    (up when the hand is lower); the horizontal model is unchanged."""
    fs = FlightSim(VEH, hop())
    pu = Push(10.0, 0.3, 10.0, azimuth_deg=90.0, z_body_m=0.4, hand_height_m=1.0, pole_length_m=2.5)
    contact = np.array([0.0, -0.06, 1.9])
    u, el = fs.pole_direction(pu, contact)
    assert el == pytest.approx(math.degrees(math.asin((1.9 - (fs.ground_z + 1.0)) / 2.5)))
    assert np.linalg.norm(u) == pytest.approx(1.0) and u[0] == pytest.approx(0.0, abs=1e-12) and u[1] > 0 and u[2] > 0
    f = fs.push_force(0, pu, 10.15, {0: u})
    assert math.sqrt(sum(x * x for x in f)) == pytest.approx(10.0)          # force_n along the pole at the peak
    flat = Push(10.0, 0.3, 10.0, azimuth_deg=90.0, z_body_m=0.4, vertical_n=1.0)
    assert fs.push_force(0, flat, 10.15, {}) == flat.force_world(10.15)
    with pytest.raises(ValueError):
        Push(0.0, 0.3, 5.0, hand_height_m=1.0, pole_length_m=0.0)


def test_push_position_response_and_v08_push_check():
    """A 3 N s push in the GPS demo hover: excursion from the pre-push position and the return within 1 m are measured;
    the v0.8 push criterion judges it (<= 3 N s)."""
    sc = vs.scenario_demo(P, None, name="T_gps_push", pushes=(push_with_impulse(3.0, 0.3, t0_s=9.0, z_body_m=0.38),),
                          vtol_script=vs.demo_script(t_off=None), t_end=18.0)
    res, m = vs.run_scenario(sc)
    r = m["pushes"][0]
    assert r["in_flight"] and r["excursion_m"] > 0.05 and r["pre_xy"] is not None
    assert r["return_1p0m_s"] is not None and r["return_1p0m_s"] <= vs.v08_criteria(P, "demo")["push_return_within_1m_s"]
    c = vs.push_v08_checks(m, vs.v08_criteria(P, "demo"))
    assert c == {"push_1": True}
    big = dict(m, pushes=[dict(r, impulse_ns=5.0)])
    assert vs.push_v08_checks(big, vs.v08_criteria(P, "demo")) == {"push_1": None}      # reported, not judged


@pytest.mark.parametrize("case", [
    dict(name="T_gps_silent", gps=dict(silent=((-1e9, 1e9),)), expect="", outcome="no_flight", reject="GPS"),
    dict(name="T_gps_outage", gps=dict(outages=((10.0, 40.0),)), expect="V11", outcome="landed"),
    dict(name="T_gps_velbias", gps=dict(vel_bias=((10.0, 1e9, 1.0, 0.0),)), expect="V13", outcome="landed"),
], ids=lambda c: c["name"])
def test_gps_rules_act_in_the_closed_loop(case):
    """No GPS at ON -> REJ_GPS (never flies); a GPS outage in HOLD -> V11 LAND; a GPS velocity bias (position right) ->
    the plausibility rule V13 -> LAND."""
    sc = vs.with_gps(hop(case["name"], t_end=60.0 if case["outcome"] == "landed" else 8.0, script=HOLD_ONLY), **case["gps"])
    res, m = vs.run_scenario(sc)
    exp = {"expect": case["expect"], "outcome": case["outcome"]}
    if case.get("reject"):
        exp["reject"] = case["reject"]
    verdict, why = vs.judge(m, exp)
    assert verdict == "PASS", why
    if case["expect"] == "V11":
        # the LAND comes after FIX_OK clears (LOST_AFTER_EPOCHS), the FCU's timeout + land_after (params gps_loss)
        gl = V["position_hold"]["gps_loss"]
        assert 10.0 + gl["timeout_s"] + gl["land_after_s"] <= m["t_land_s"] <= 10.0 + gl["timeout_s"] + gl["land_after_s"] + 2.0
        assert "gps_lost" in m["pos_requests"]


def test_horizontal_margins_and_the_heading_tolerance():
    """The linear horizontal loop: the flown loop (IMU-propagated NAV state) vs a loop on the raw GPS velocity (+ the
    0.22 s latency): the latency compensation is what makes it stable.  The linear heading-rotation limit (= the phase
    margin) is reported as such - it is not the flown heading tolerance (physics review medium 5; params
    heading.tolerance_deg 15 deg)."""
    hm = vs.horizontal_margins(vs.prototype_pgains())
    gd = vs.horizontal_margins(vs.prototype_pgains(), gps_direct=True)
    assert hm["phase_margin_deg"] > 0.0 and hm["gain_margin_x"] > 1.0
    assert gd["phase_margin_deg"] < hm["phase_margin_deg"] - 20.0
    assert hm["linear_heading_rotation_limit_deg"] == hm["phase_margin_deg"] and "heading_error_tolerance_deg" not in hm
    assert V["position_hold"]["heading"]["tolerance_deg"] == 15.0
    assert 3.0 < hm["attitude_bandwidth_rad_s"] < 15.0 and 4.5 < hm["tau_d_s"] < 5.5       # v0.9 VTOL: 4.89 s
    # more acceleration feedback (kd) adds phase margin
    assert vs.horizontal_margins({"kd": 0.4})["phase_margin_deg"] > hm["phase_margin_deg"]


def test_campaign_gps_sample_and_paired_builds():
    """The v0.8 campaign variables come from their own stream (every v0.7.1 draw kept); 'on' / 'off' / 'v071' build the
    GPS build / the v0.8 build without it / the v0.7.1 run of the same sample; the v0.8 envelope (TI 0.2, gusts clipped
    to the operator limit, the demo outdoors)."""
    lim = V["position_hold"]["wind_limits_mps"]
    for i in range(30):
        s = vc.sample(9, i)
        g = s["gps"]
        for k in ("latency_s", "vel_white_mps", "pos_gm_sigma_m", "pos_gm_tau_s", "sky_blocked"):
            lo, hi = vc.SAMPLING_GPS[k]
            assert lo <= g[k] <= hi
        assert abs(g["heading_err_deg"]) <= 30.0
        env = s["v08_env"]
        assert env["turbulence_sigma_mps"] == pytest.approx(0.2 * s["wind_mps"])
        on = vc.build_scenario(P, s, None, "on")
        off = vc.build_scenario(P, s, None, "off")
        old = vc.build_scenario(P, s, None)
        assert on.gps is not None and on.gps.heading_err_deg == g["heading_err_deg"] and on.initial_yaw_deg == 0.0
        assert off.gps is None and off.nav_v08 and off.takeoff_check
        assert old.gps is None and old.nav_v08 is None and old.initial_yaw_deg == s["initial_yaw_deg"]
        assert all(gu.speed_mps <= max(0.0, lim["field_gust"] - s["wind_mps"]) + 1e-9 for gu in on.gusts)
        assert on.seed == off.seed == old.seed
        d = vc.sample(9, i, profile="demo")
        assert d["v08_env"]["wind_mps"] <= lim["demo_mean"]
        don = vc.build_scenario(P, d, None, "on")
        assert don.wind_mps == d["v08_env"]["wind_mps"] and all((q.hand_height_m is None) == ("hand_height_m" not in qq)
                                                                for q, qq in zip(don.pushes, d["pushes"]))
    with pytest.raises(ValueError):
        vc.build_scenario(P, vc.sample(9, 0), None, "maybe")


def test_tip_over_margin_energy_method():
    """Physics-review fix (medium 8): the energy-method tip-over threshold - the params tipover geometry (static angle
    14.2 deg, footprint radius 0.1234 m: CG 0.49 m) gives the architect's numbers (upright 0.62 m/s, 4 deg toward the
    motion 0.44, 4 deg away 0.81); lean toward the motion lowers it, the CG past the edge gives 0."""
    from sim.sim6dof import tip_over_vcrit
    veh = vm.load_vehicle()                             # v0.9: the params tipover numbers are the rocket vehicle's
    r = 0.5 * veh.layout.footprint_d * 1e-3
    h = r / math.tan(math.radians(V["position_hold"]["tipover"]["static_angle_deg"]))
    m = veh.full.mass_kg
    i_cg = float(min(veh.full.inertia_kg_m2[0][0], veh.full.inertia_kg_m2[1][1]))
    up, fwd, back = (tip_over_vcrit(m, i_cg, r, h, d) for d in (0.0, 4.0, -4.0))
    assert up == pytest.approx(0.62, abs=0.01) and fwd == pytest.approx(0.44, abs=0.01) and back == pytest.approx(0.81, abs=0.01)
    assert tip_over_vcrit(m, i_cg, r, h, 20.0) == 0.0
    geo = tip_over_vcrit(m, i_cg, r, veh.z_cg_m, 0.0)                   # the sim's contact plane: CG z_cg above it
    assert geo > up                                                     # the params value is the conservative one


def test_campaign_opts_latency_and_wind_limit():
    """Physics-review fix round: campaign-wide overrides - a fixed true GPS latency (medium 7) and the operator wind
    limit with the plant / sensor spread kept (medium 9)."""
    lim = V["position_hold"]["wind_limits_mps"]
    s0 = vc.sample(7070, 3)
    s = vc.apply_opts(s0, {"gps": {"latency_s": 0.45}, "wind_limit": True})
    assert s["gps"]["latency_s"] == 0.45 and s0["gps"]["latency_s"] != 0.45
    assert s["v08_env"]["wind_mps"] == lim["field_mean"]
    assert s["v08_env"]["turbulence_sigma_mps"] == pytest.approx(0.2 * lim["field_mean"])
    assert s["v08_env"]["gust_clip_mps"] == pytest.approx(lim["field_gust"] - lim["field_mean"])
    assert s["thrust_scale"] == s0["thrust_scale"] and s["opts"]["wind_limit"]
    sc = vc.build_scenario(P, s, None, "on")
    assert sc.gps.latency_s == 0.45 and sc.wind_mps == lim["field_mean"]
    assert vc.opts_tag({"gps": {"latency_s": 0.45}, "wind_limit": True}) == "_latency_s0.45_windlimit"
    assert vc.apply_opts(s0, None) is s0


# --------------------------------------------------------------------------- #
# v0.9 plant (VTOL hardware round): configuration, thrust / power, rotor, play, tether
# --------------------------------------------------------------------------- #
SIZING_TW = os.path.join(PROJECT_ROOT, "sizing", "tw_table_v09.json")


def test_vtol_vehicle_is_the_cad_report_per_power_source():
    """Architect decision a / e: the VTOL scenarios fly the VTOL hardware configuration, mass / CG / inertia from the CAD
    report of the power source (battery 1647.2 g, tether 1399.3 g, hybrid 1520.3 g); no chute; the lateral CG offsets of
    the reports (0.61 / 2.06 / 1.52 mm); the top-cap eye bolt and the top-cap GPS patch."""
    assert vs.hardware_config() == "vtol" and P["mission"]["config"] == "vtol"
    # v0.9 lead-exit round (2026-10-29, HARDWARE changed: the motor leads start at the motor's lowest end, +8.5 g of lead,
    # -0.8 g propulsion notch): pins 1647.17 / 1399.27 / 1520.27 g, 0.61 / 2.06 / 1.52 mm, rocket 2592.62 g -> 1654.88 /
    # 1406.98 / 1527.98 g, 0.78 / 2.25 / 1.70 mm, rocket 2600.23 g; review fix (2026-10-29: RTV bond / notch seal / chafe
    # tape 1.0 g ASSUMED in motor_leads) -> 1655.88 / 1407.98 / 1528.98 g, 0.80 / 2.28 / 1.73 mm, rocket 2601.23 g; photo round
    # (2026-10-29, HARDWARE changed - user photos: the leads leave the motor's front ring as a heat-shrink bundle along a stator
    # vane, no climb along the can: lead -16.2 mm each, RTV / notch seal / tape / shrink 1.0 -> 2.0 g ASSUMED, notch re-cut;
    # round 1 in history/v0.9_wire_round1) -> 1655.08 / 1407.18 / 1528.18 g, 0.80 / 2.28 / 1.73 mm, rocket 2600.30 g;
    # wire09c clock mount (2026-11-03, HARDWARE changed - user 2026-10-30 'generous, no measuring': keyways round the circle,
    # no tab screws, the lead passage + blister at 0 deg, longest-corner leads + foam pads / plug; history/v0.9_wire_round2)
    # -> 1652.90 / 1405.00 / 1526.00 g, 1.34 / 2.92 / 2.32 mm, rocket 2598.43 g; wire09c fix round (2026-11-04, HARDWARE
    # changed - independent review F1: keyways re-planned so every sleeve tooth carries the motor torque on one tab, 27 -> 25
    # keyways, propulsion module +0.70 g slicer; F5: fan floor 0.1 mm lower) -> 1653.61 / 1405.71 / 1526.71 g, 1.34 / 2.92 /
    # 2.32 mm, rocket 2599.14 g; LIVERY (2026-11-04, HARDWARE changed - user decisions: white body + the orange YUCHAN
    # inlay on the avionics module; history/v0.9_pre_livery): the module is the white body with a 0.42 mm logo pocket
    # (slicer 58.55 -> 57.48 g VTOL, 124.75 -> 123.62 g rocket) plus the new inlay part avionics_logo (1.29 g, the 2-colour
    # slice's orange part grams) -> 1653.83 / 1405.93 / 1526.93 g, 1.34 / 2.93 / 2.32 mm, rocket 2599.30 g (+0.22 g VTOL,
    # +0.16 g rocket; offsets within 0.01 mm, the tether one 2.92 -> 2.93); REVIEWFIX re-pin (2026-11-04, independent review
    # F1 / F3, not to hide a regression - same geometry, the slicer basis of two part masses corrected): F1 the orange
    # inlay was printed as a double pass (classic walls without detect_thin_wall: 1.29 g for a 0.89 g solid), now ONE
    # pass, 0.902 g; F3 the module's grams mixed two slicer settings (pipeline white body + resolved orange), now the
    # pipeline slice of the plain module + the livery delta measured inside the resolved slices (+0.044 g VTOL, +0.031 g
    # rocket): white body 57.74 g VTOL / 123.92 g rocket -> the reports below (vs the pre-livery 1653.61 / 1405.71 /
    # 1526.71 / 2599.14 g: +0.09 g VTOL, +0.07 g rocket; offsets within 0.01 mm: 1.338 / 2.922 / 2.319 mm)
    for pw, mass_g, off_mm in (("battery", 1653.70, 1.34), ("tether", 1405.80, 2.92), ("hybrid", 1526.80, 2.32)):
        v = vs.vehicle(pw)
        with open(v.source_path, encoding="utf-8") as f:
            rep = json.load(f)
        assert v.hardware_config == "vtol" and v.power_source == pw and rep.get("config") == "vtol"
        assert v.full.mass_kg * 1e3 == pytest.approx(mass_g, abs=0.05)
        assert math.hypot(v.full.com_m[0], v.full.com_m[1]) * 1e3 == pytest.approx(off_mm, abs=0.01)
        assert v.full.com_m[2] * 1e3 == pytest.approx(rep["totals"]["com_mm"][2], abs=1e-6)
        assert not v.has_chute and FlightSim(v, hop71(power_source=pw)).chute_cda_full == 0.0
        assert v.tether_attach_body_m[2] == pytest.approx((v.layout.z_cap_top + v.layout.cap_eye_h) * 1e-3)
        assert v.gps_antenna_mm[2] == pytest.approx(v.layout.cap_patch_z[1])
        assert vs.cad_cg_offset_mm(pw) == pytest.approx((v.full.com_m[0] * 1e3, v.full.com_m[1] * 1e3))
    # the ballistic mission keeps the rocket
    r = vm.load_vehicle()
    assert r.hardware_config == "rocket" and r.has_chute and r.full.mass_kg * 1e3 == pytest.approx(2599.21, abs=0.05)


def test_footprint_tip_over_between_fins_is_the_rigid_foot_limit():
    """Architect decision g: the solid-fin footprint is the square through the four fin tips (fins at the vane angles
    0 / 90 / 180 / 270 deg); between two fins the edge is fin_tip_r cos 45 deg; with the battery VTOL's CG 247 mm above the
    contact plane the energy-method tip-over speed between fins is ~0.64 m/s (the architect's number), toward a fin ~0.95."""
    from sim.sim6dof import tip_over_vcrit
    v = vs.vehicle()
    r = v.layout.fin_tip_r * 1e-3
    assert v.footprint_radius_m(0.0) == pytest.approx(r) and v.footprint_radius_m(math.pi / 2) == pytest.approx(r)
    assert v.footprint_radius_m(math.pi / 4) == pytest.approx(r * math.cos(math.pi / 4))
    i_cg = float(min(v.full.inertia_kg_m2[0][0], v.full.inertia_kg_m2[1][1]))
    between = tip_over_vcrit(v.full.mass_kg, i_cg, v.footprint_radius_m(math.pi / 4), v.z_cg_m, 0.0)
    toward = tip_over_vcrit(v.full.mass_kg, i_cg, v.footprint_radius_m(0.0), v.z_cg_m, 0.0)
    assert 0.6 < between < 0.7 and 0.9 < toward < 1.0
    assert vs.V09_TOUCHDOWN_VH_MAX_MPS == 0.3 and vs.v08_criteria(P, None)["touchdown_vh_ground_max_mps"] == 0.3


def test_hover_power_matches_the_sizing_report():
    """The fan power model (sim.tether.FanPower, P = 2264 W (T / T_ref)^1.5) reproduces the sizing report's hover power of
    the battery VTOL: P1 (T_ref 30.30 N) 881 W at every T_max, P2 (T_ref = T_max) per thrust-table row."""
    from sim import tether as tt
    with open(SIZING_TW, encoding="utf-8") as f:
        tw = json.load(f)
    v = vs.vehicle()
    w = v.full.mass_kg * 9.80665
    for row in tw["configs"]["vtol_battery"]["tw_table"]:
        p1 = tt.fan_power_from_params(P, row["thrust_N"], 0.0).power(w)
        p2 = tt.fan_power_from_params(P, row["thrust_N"], 1.0).power(w)
        assert p1 == pytest.approx(row["hover_power_W_P1"], rel=1e-6)
        assert p2 == pytest.approx(row["hover_power_W_P2"], rel=1e-6)
    fan = tt.FanPower()
    assert fan.thrust_at_power(fan.power(12.3)) == pytest.approx(12.3)


def test_tether_bus_voltage_drop_and_crossover_match_the_proposal():
    """docs/tether_power_proposal.json recommended_route: battery-less 1402 g hover P1 692 W on the 58.5 mOhm route ->
    29.5 A, V_esc 23.48 V (no diode in the battery-less build); the hybrid crossover (V_s - V_buf) / R_loop = 40.3 A at
    3.80 V / cell on 59.5 mOhm; the supply's CC limit and its latch-off after latch_s; a battery's loaded voltage."""
    from sim import tether as tt
    with open(os.path.join(PROJECT_ROOT, "docs", "tether_power_proposal.json"), encoding="utf-8") as f:
        prop = json.load(f)["key_results"]["recommended_route"]["VTOL1650_battery_less"]["hover_P1"]
    fan = tt.FanPower()
    ps = tt.PowerSystem(tt.PowerConfig(source="tether", r_loop_ohm=0.0585), fan)
    st = ps.step(0.0, 0.004, fan.thrust_at_power(prop["P_W"]))
    assert st.v_bus == pytest.approx(prop["V_esc"], abs=0.02) and st.i_load == pytest.approx(prop["I_A"], abs=0.05)
    assert st.supply_state == tt.SUPPLY_CV and st.connected and not st.limited
    # hybrid: below the crossover the buffer is idle, above it shares (I_x from params: 59.5 mOhm, 3.80 V / cell)
    cfg = tt.PowerConfig.from_params(P, source="hybrid")
    assert cfg.r_loop_ohm == pytest.approx(0.0595) and cfg.buf_v_rest_cell == 3.80
    i_x = (cfg.v_set - cfg.buf_cells * tt.ocv_cell(tt.soc_of_ocv(cfg.buf_v_rest_cell))) / cfg.r_loop_ohm
    assert i_x == pytest.approx(40.3, abs=0.1)
    ph = tt.PowerSystem(cfg, fan)
    lo = ph.step(0.0, 0.004, fan.thrust_at_power(0.9 * i_x * 22.8))
    hi = ph.step(0.004, 0.004, fan.thrust_at_power(1.4 * i_x * 22.5))
    assert lo.i_buffer == 0.0 and hi.i_buffer > 1.0 and hi.i_tether > i_x * 0.98
    # CC at the limit -> latch-off after latch_s in CC
    pc = tt.PowerSystem(tt.PowerConfig(source="tether", i_limit_a=60.0, latch_s=1.0), fan)
    t = 0.0
    while not pc.tripped and t < 3.0:
        st = pc.step(t, 0.01, fan.thrust_at_power(1700.0))
        t += 0.01
    assert pc.tripped and 0.99 <= t <= 1.03 and st.supply_state in (tt.SUPPLY_CC, tt.SUPPLY_TRIPPED)
    assert pc.step(t, 0.01, fan.thrust_at_power(800.0)).v_bus == 0.0                  # latched: nothing comes back
    # battery: 3.7 V / cell resting, full proxy power -> ~20.2 V at the ESC (sizing: 20.4 V, thrust x0.87)
    pb = tt.PowerSystem(tt.PowerConfig(source="battery", v_rest_cell=3.7), fan)
    vb = pb.step(0.0, 0.004, 30.3025).v_bus
    assert 20.0 < vb < 20.5 and 0.84 < (vb / 22.2) ** 1.61 < 0.88


def test_rotor_gyroscopic_moment_and_spool_reaction_signs():
    """Architect decision d: the fan turns CW seen from the intake (edf.rotation, swirl sign +1): its angular momentum
    points along -z_b, h = -I omega z_b.  A body pitching about +x (w_x > 0) then gets -w x h = (0, w_x h_z, 0): a moment
    about -y; a spool-up (dT/dt > 0) pushes the body about +z (the same sense as the residual swirl torque).  The 6-DOF
    derivative carries exactly these terms (rotor on - rotor off)."""
    from sim.edf import RotorModel
    rot = RotorModel.from_params(P, 30.3025)
    assert rot.sign == 1.0 and rot.i_rot_kg_m2 == pytest.approx(4.5e-5)
    T = 16.0
    hz = rot.h_z(T)
    assert hz < 0.0 and abs(hz) == pytest.approx(4.5e-5 * rot.omega_ref_rad_s * math.sqrt(T / 30.3025))
    mx, my, mz = rot.gyro_moment((1.0, 0.0, 0.0), T)
    assert mx == 0.0 and my == pytest.approx(hz) and my < 0.0 and mz == 0.0
    assert rot.reaction_moment_z(T, +10.0) > 0.0 and rot.reaction_moment_z(T, -10.0) < 0.0
    # domega/dt consistent with the thrust law omega ~ sqrt(T)
    assert rot.omega_rate(T, 1.0) == pytest.approx((rot.omega(T + 1e-4) - rot.omega(T)) / 1e-4, rel=1e-3)
    on = FlightSim(VEH, hop71(vane_play_deg=0.0))
    off = FlightSim(VEH, hop71(vane_play_deg=0.0, rotor_inertia_kg_m2=0.0))
    x = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, T, 0.0, 0.0, 0.0, 0.0]
    t_cmd = 20.0
    d_on = np.array(on.deriv(x, on.cfg_full, t_cmd, [0.0] * 4, (0.0, 0.0, 0.0), 0.0, -1))
    d_off = np.array(off.deriv(x, off.cfg_full, t_cmd, [0.0] * 4, (0.0, 0.0, 0.0), 0.0, -1))
    m_extra = np.array(on.cfg_full.I) @ (d_on[10:13] - d_off[10:13])
    rot_on = on.rotor
    assert m_extra == pytest.approx([0.0, 0.5 * rot_on.h_z(T),
                                     rot_on.reaction_moment_z(T, (t_cmd - T) / on.edf.tau_s)], abs=1e-12)


def test_vane_play_is_a_backlash_operator_in_the_plant():
    """Architect decision c: direct drive 1:1, lost motion 0.65 + gear backlash 1.0 deg ASSUMED = 1.65 deg total play.
    The operator: the vane stays inside servo +- half the play and moves only when dragged; 0 play = the servo angle.  In
    the 6-DOF the logged vane angle trails the servo output by half the play after a reversal, and the plant's default
    play is the params value (direct drive)."""
    from sim.edf import vane_play, vane_play_deg_from_params
    h = 0.5
    assert vane_play(0.0, 0.3, h) == 0.0 and vane_play(0.0, 0.8, h) == pytest.approx(0.3)
    assert vane_play(0.3, -0.1, h) == 0.3 and vane_play(0.3, -0.4, h) == pytest.approx(0.1)
    assert vane_play(0.123, 0.2, 0.0) == 0.2
    assert vane_play_deg_from_params(P) == pytest.approx(1.65)
    fs = FlightSim(VEH, hop71(t_end=12.0))
    assert fs.play_deg == (1.65,) * 4 and fs.play_on
    res = fs.run()
    L = res.log
    d, s_ = L["delta"], L["delta_servo"]
    assert np.all(np.abs(d - s_) <= math.radians(1.65) / 2 + 1e-12)
    P_ = L["state"] == lp.STATE_POWERED
    assert np.max(np.abs(d[P_] - s_[P_])) > 0.9 * math.radians(1.65) / 2          # the play is taken up in flight


def test_campaign_v09_sampler_and_tether_profiles():
    """The v0.9 campaign variables (own stream; architect decisions b-f): T_max in the envelope (thrust_scale fixed 1.0),
    the group range opts map the same draw, fan power model, rotor inertia, play per vane, the build tolerance (lateral CG
    residual <= 1.5 mm), the power source per profile, the tether rig / supply / buffer; the scenario builder passes them
    on (tether profiles: the tether / hybrid vehicle's CAD CG + the residual, indoors without the GPS)."""
    V9 = vc.SAMPLING_V09
    for i in range(12):
        s = vc.sample(8300, i)
        assert s["thrust_scale"] == 1.0 and V9["t_max_kgf"][0] <= s["t_max_kgf"] <= V9["t_max_kgf"][1]
        assert len(s["vane_play_deg"]) == 4 and all(0.8 <= x <= 3.0 for x in s["vane_play_deg"])
        assert math.hypot(*s["cg_residual_mm"]) <= 1.5 + 1e-9 and s["power_source"] == "battery"
        assert 1.0 <= s["thrust_voltage_exp"] <= 2.0
        lo = vc.apply_opts(s, {"t_max_kgf_range": [1.9, 2.14]})
        q = (s["t_max_kgf"] - 2.14) / (3.09 - 2.14)
        assert lo["t_max_kgf"] == pytest.approx(1.9 + q * 0.24)
        sc = vc.build_scenario(P, s, None, "on")
        assert sc.t_max_kgf == s["t_max_kgf"] and sc.vane_play_deg == tuple(s["vane_play_deg"])
        assert sc.rotor_inertia_kg_m2 == pytest.approx(4.5e-5 * s["rotor_inertia_scale"]) and sc.power_source == "battery"
        for prof, src in (("tether_test", "tether"), ("hybrid_demo", "hybrid")):
            t = vc.sample(8300, i, profile=prof)
            assert t["power_source"] == src and t["campaign_profile"] == prof and "tether" in t
            tsc = vc.build_scenario(P, t, None, "on")
            assert tsc.power_source == src and tsc.tether is not None and tsc.power.source == src
            assert tsc.profile == "demo" and (tsc.gps is None) == bool(t["indoor"])
            cad = vs.cad_cg_offset_mm(src)
            assert math.hypot(tsc.cg_offset_mm[0] - cad[0], tsc.cg_offset_mm[1] - cad[1]) <= 1.5 + 1e-9
            if prof == "tether_test":
                assert t["pushes"] == [] and tsc.pushes == ()
    assert vc.sample(8300, 3) == vc.sample(8300, 3)


def test_tether_plant_catenary_catch_line_and_unplug_fall():
    """Architect decision f: the slack cable hanging straight below the pulley loads the vehicle with half its slack's
    weight; the catch line is a spring-damper only when taut; a battery-less vehicle that loses its power in a hover falls
    into the catch line (bungee) - the fall is visible in the log: the thrust dies, the catch line takes the load."""
    from sim import tether as tt
    w = 0.139 * 9.80665
    f = tt.catenary_end_force((0.0, 0.0, 2.0), 2.3, w)
    assert f[2] == pytest.approx(-w * 0.15, rel=0.02) and abs(f[0]) < 0.05
    assert tt.catch_line_force((0.0, 0.0, 2.0), (0.0, 0.0, 0.0), 2.1, 600.0, 10.0)[3] == 0.0
    fx, fy, fz, ten = tt.catch_line_force((0.0, 0.0, 2.1), (0.0, 0.0, -1.0), 2.0, 600.0, 10.0)
    assert ten == pytest.approx(600.0 * 0.1 + 10.0 * 1.0) and fz == pytest.approx(ten)
    sc = vs.scenario_tether(P, None, "T_unplug", "unplug test", alt="1.5", t_off=None, t_end=12.0,
                            power_events=(tt.PowerEvent(9.0, "unplug"),), vane_play_deg=0.0, rotor_inertia_kg_m2=0.0)
    res, m = vs.run_scenario(sc)
    L = res.log
    assert m["power_source"] == "tether" and m["t_power_loss_s"] == pytest.approx(9.0, abs=0.005)
    k9 = int(np.searchsorted(L["t"], 9.0))
    assert np.all(L["tether_connected"][k9 + 1:] == 0) and np.all(L["v_bus"][k9 + 1:] == 0.0)
    assert L["thrust"][-1] < 0.5 and m["tether_catch_max_n"] > 10.0
    with tempfile_dir() as d:
        meta = vs.export_npz(res, os.path.join(d, "u.npz"), vs.vehicle("tether"))
        z = np.load(os.path.join(d, "u.npz"))
        assert str(z["hardware_config"]) == "vtol" and str(z["power_source"]) == "tether"
        assert z["tether_anchor_m"].shape == (3,) and np.all(np.isfinite(z["tether_anchor_m"]))
        assert z["tether_attach_body_m"].shape == (3,) and z["tether_tension_N"].shape == L["t"].shape
        assert z["tether_connected"].dtype == bool and not z["tether_connected"][-1] and z["tether_connected"][0]
        assert z["supply_V"].shape == L["t"].shape and z["supply_A"].shape == L["t"].shape and "buffer_soc" not in z
        assert meta["power_source"] == "tether"


class tempfile_dir:
    def __enter__(self):
        import tempfile
        self.d = tempfile.mkdtemp(prefix="v09_")
        return self.d

    def __exit__(self, *a):
        import shutil
        shutil.rmtree(self.d, ignore_errors=True)
