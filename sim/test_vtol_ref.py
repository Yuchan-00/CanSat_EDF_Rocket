"""Tests of the v0.7 VTOL references (SPEC 8.9): ``sim.onboard`` in the VTOL build, ``sim.vertical_ctrl``,
``sim.onboard.FcuStack`` (the per-tick order), the ground logic's ON / OFF / UP / DOWN, and the ballistic regression.

* command x state x phase table (every command in every VTOL state / phase: resulting state, phase, code, nonce, ack);
* every safety rule at its exact tick (n - 1 ticks: nothing, n: acts), the rule order when several apply, the timers
  (spool warning, take-off gate, CLIMB -> HOLD, touchdown -> LANDED, LANDED -> SAFE) and the cumulative motor time;
* the vertical controller's equations (reference rate limits, command limits, feed-forward, take-off ramp,
  anti-windup, touchdown detector, baro-failure mode, non-finite inputs) worked out by hand;
* closed loops of FcuStack with a one-dimensional point-mass plant written here (thrust lag, inelastic ground):
  NOT the 6-DOF simulation (``sim.sim6dof`` is the physics reference); they check that the reference pieces work
  together at the normative tick order, not flight performance;
* the ballistic build: a digest of deterministic random sequences pinned from the v0.6.3 supervisor (bit-identical),
  and ON / OFF / UP / DOWN answered with REJ_UNKNOWN_CMD as v0.6.3 did.

v0.7.1 additions: build profiles (field / demo, ``vtol_values``); the LANDED rules (I9 IMU / V4 / V5 -> SAFE); the
baro-lost time limit (V8 / V9) and the LAND floor timeout (V10) at their exact ticks; the vertical controller v2
(integer reference: exact arrival ticks, reference-rate feed-forward, cos(tilt) from the quaternion, the still /
support / floor touchdown signatures, low_in_land_ticks, ground_signature); the v0.7.1 FcuStack order (attitude
inputs of the previous tick); the VTOL NAV estimator (``GatedMahonyEstimator``) and altitude filter
(``VtolAltitudeFilter``) on synthetic streams; the ground logic's rule events and Korean texts; the TELEM2 link
budget.  The strict xfail ``sim/test_vtol_scenarios.py::test_baro_lost_landing_reaches_the_ground`` (the sim
agent's) stays until the 6-DOF shows the v0.7.1 fix: that is the sim agent's call.

v0.7.1b (the physics review's HIGH defect - a plant ~20 % stronger than the thrust model pinned the loop in the air and
the floor signature stopped the motor 6-8 m up): the thrust learning k_T (mission.vtol.thrust_learning) and the band
gating of the touchdown detector (mission.vtol.touchdown.gating_v071b) by hand; the 1-D stack with plants x0.80 /
0.90 / 1.00 / 1.25 / 1.35 of the model, both profiles, params.json's gains_ASSUMED and the flight gains
(sim/out/vtol_gains.json): every LAND ends on the ground by the detector, k_T converges, holds converge; an OFF during
the climb; k_T frozen at its clamps (one documented physical limit as a strict xfail); ground above the pad; the
ditches with the flight gains; the VTOL attitude override control.vtol_overrides (rate_i_max_z) vs the ballistic gains.

v0.8: params.json enables GPS position hold (mission.vtol.position_hold.enabled) and the take-off check; the tests here
pin the v0.7.1 VTOL build with ``SupervisorParams.from_params(..., gps=False, takeoff_check=False)`` (``V071``) - the
v0.7.1 rules are unchanged in v0.8.  The v0.8 additions (GPS, position hold, V11-V15, REJ_GPS, TELEM v3, nlink v3 slots,
the take-off check, the NAV predict / correct order) are tested in ``sim/test_gps_ref.py``.

Run: ``.venv/Scripts/python.exe -m pytest sim/test_vtol_ref.py -q -p no:cacheprovider``.
"""
from __future__ import annotations

import copy
import dataclasses
import hashlib
import json
import math
import os
import random
import sys

import pytest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

import numpy as np  # noqa: E402

from sim import link_model as lm  # noqa: E402
from sim import link_proto as lp  # noqa: E402
from sim import onboard as onb  # noqa: E402
from sim.nlink_proto import f32 as nl_f32  # noqa: E402
from sim import quat  # noqa: E402
from sim import vertical_ctrl as vcm  # noqa: E402
from sim.estimator import GatedMahonyEstimator, PadAlignGyroEstimator, up_body, up_error_rad  # noqa: E402
from sim.ground_station_logic import REASON_TEXT_KO, GroundAssist, GroundParams, reason_text_ko  # noqa: E402
from sim.onboard import (FcuStack, OnboardInputs, OnboardSupervisor, State, SupervisorParams, UplinkFrame,  # noqa: E402
                         lfsr16_next, ticks_ge, vtol_profiles, vtol_values)
from sim.state_machine import G0, AltitudeFilter, VtolAltitudeFilter, thrust_ratio  # noqa: E402
from sim.vertical_ctrl import (MODE_FLY, MODE_IDLE, VerticalController, VerticalInputs,  # noqa: E402
                               VerticalParams, cos_tilt_from_q, hover_throttle)

with open(os.path.join(_ROOT, "params.json"), encoding="utf-8") as _f:
    P = json.load(_f)
HZ = int(P["control"]["loop_hz"])
DT = 1.0 / HZ
VTOL = P["mission"]["vtol"]
V071 = dict(gps=False, takeoff_check=False)      # v0.8 params: the v0.7.1 VTOL rules (no GPS, no take-off check)
SP = SupervisorParams.from_params(P, mode=lp.MODE_VTOL, **V071)   # the v0.7.1 VTOL build (v0.8 tests: SP8)
T_MAX = float(P["edf"]["thrust_nominal_N"])
MASS = T_MAX / (1.19 * G0)                      # T/W 1.19 (the design's ~1.195): u_hover = sqrt(1 / 1.19) = 0.9167
VP = VerticalParams.from_params(P, mass_kg=MASS, t_max_n=T_MAX)
# v0.9: the hand-worked vertical tests below keep the v0.8.1 take-off law (0 -> u_max in ramp_ticks); params.json's
# mission.vtol.takeoff_v09 (two-slope ramp, thrust search) has its own tests (test_v09_*)
VP = dataclasses.replace(VP, to_u_start=0.0, to_slew_per_s=0.0, to_boost_per_s=0.0, u_slew_per_s=0.0,
                         to_learn_tau_s=0.0, to_ref_hold=False, td_release_per_s=0.0, ref_approach_shift=0,
                         land_ref_rebase=False)   # v0.9 fix round: the hand-worked laws keep the v0.8.1 vertical controller
with open(os.path.join(_ROOT, "sim", "out", "vtol_gains.json"), encoding="utf-8") as _f:
    _VGAINS = json.load(_f)
# v0.7.1b: the vertical gains the firmware flies (gen_firmware_params -> params_gen.h VC_*), next to params.json's
# gains_ASSUMED (VP): the physics review's HIGH defect needs these (u_hover - vz_kp vz_max_down - vz_i_max = 0.784)
TUNED_VGAINS = {k: float(_VGAINS[k]) for k in ("alt_kp", "vz_kp", "vz_ki", "vz_i_max")}
S_, A_, P_, U_, R_, AB_ =State.SAFE, State.ARMED, State.POWERED, State.UNPOWERED, State.RELEASED, State.ABORT
NONE, SPOOL, CLIMB, HOLD, LAND, LANDED = (lp.PHASE_NONE, lp.PHASE_SPOOL, lp.PHASE_CLIMB, lp.PHASE_HOLD, lp.PHASE_LAND,
                                          lp.PHASE_LANDED)


# =========================================================================== #
# parameters
# =========================================================================== #
def test_vtol_params_from_params():
    assert SupervisorParams.from_params(P).mode == lp.mission_mode(P) == lp.MODE_VTOL == P["mission"]["mode"]
    sp = SP
    assert sp.vtol and sp.spool_ticks == ticks_ge(VTOL["spool_warning_s"], HZ) == 500
    assert (sp.hover_dm, sp.alt_step_dm, sp.alt_min_dm, sp.alt_max_dm) == (50, 10, 20, 100)   # alt_max_m 10 (v0.7.1)
    assert sp.link_land_ticks == 500 and sp.endurance_ticks == ticks_ge(40.0 - 10.0, HZ) == 7500
    assert sp.hard_cutoff_ticks == 15000 and sp.v_fall_confirm_ticks == 75 and sp.safe_after_landed_ticks == 500
    assert (sp.ceiling_m, sp.v_geofence_m, sp.v_fall_vz_mps) == (25.0, 35.0, -3.0)
    # v0.7.1: LAND floor timeout 6 s; baro-lost limit 1.5 (h / 0.6 + 1 / 0.3) + 5 s = 10 s + 62.5 ticks per dm
    assert sp.profile == "field" and sp.land_timeout_ticks == 1500 and sp.v_geofence_dm == 350
    assert (sp.baro_limit_base_ticks, sp.baro_limit_mt_per_dm) == (2500, 62500)
    # the shared fields are the ballistic build's
    sb = SupervisorParams.from_params(P, mode=lp.MODE_BALLISTIC)
    # (v0.9 power round: the power-rule fields are VTOL fields too - onb.power_fields, the tether builds' rules)
    pw = onb.power_fields(P, HZ, None, None, float(P["battery"]["min_cell_v"]))
    shared = [f.name for f in dataclasses.fields(SupervisorParams) if f.name in onb.SupervisorParams.__annotations__
              and f.name not in onb.vtol_fields(P["mission"], HZ) and f.name not in pw and f.name != "mode"]
    assert all(getattr(sb, k) == getattr(sp, k) for k in shared)
    assert not sb.power_rules and sb.power == "battery"              # ballistic: the power rules at their defaults
    assert sb.spool_ticks == 0 and sb.hover_dm == 0                   # ballistic: VTOL fields at their defaults
    old = copy.deepcopy(P)
    for k in ("mode", "mode_note", "vtol"):
        old["mission"].pop(k)
    assert SupervisorParams.from_params(old).mode == lp.MODE_BALLISTIC  # a v0.6.3 params file is the ballistic mission
    with pytest.raises(ValueError):
        SupervisorParams.from_params(old, mode=lp.MODE_VTOL)            # no mission.vtol
    with pytest.raises(ValueError):
        SupervisorParams.from_params(P, mode="hover")
    for patch in ({"alt_max_m": 25.0}, {"hover_altitude_m": 1.0}, {"alt_step_m": 0.0}, {"ceiling_m": 40.0},
                  {"land_reserve_s": 41.0}, {"hard_cutoff_s": 20.0}, {"fall_vz_mps": 0.5}):
        q = copy.deepcopy(P)
        q["mission"]["vtol"].update(patch)
        with pytest.raises(ValueError):
            SupervisorParams.from_params(q)


def test_vtol_params_leave_time_to_land_from_alt_max():
    """v0.7.0 had alt_max_m 20 (landing ~35.5 s > the 30 s between V2 and V6); the architect set alt_max_m 12 (mission.vtol
    timing_note): a landing from the ceiling of the altitude command range now ends before the hard cutoff."""
    v = VTOL
    t_land = ((v["alt_max_m"] - v["flare_altitude_m"]) / v["descent_rate_mps"] + v["flare_altitude_m"] / v["touchdown_rate_mps"]
              + v["landed_confirm_s"])
    assert v["hard_cutoff_s"] - (v["max_flight_s"] - v["land_reserve_s"]) >= t_land, t_land


def test_stack_endurance_land_from_alt_max_lands_before_the_hard_cutoff():
    """What the parameter relation above means (1-D plant): held at alt_max_m until V2 (flight-time LAND), the vehicle
    lands and stops its motor before V6 (hard cutoff) - the v0.7.0 values (alt_max_m 20) were cut by V6 at ~2.6 m."""
    f = _Fly()
    f.run(0.3)
    f.tick(lp.CMD_ON)
    f.run(20.0, until=lambda o: o.sup.phase == HOLD)
    for _ in range((SP.alt_max_dm - SP.hover_dm) // SP.alt_step_dm):
        f.tick(lp.CMD_UP)
        f.run(0.3)
    o = f.run(60.0, until=lambda o: o.sup.phase == LAND)
    assert o.sup.reason == lp.R_V_ENDURANCE and abs(f.pl.z - VTOL["alt_max_m"]) < 0.2
    o = f.run(60.0, until=lambda o: o.sup.state != P_)
    assert o.sup.state != AB_ and o.sup.reason != lp.R_V_HARD_CUTOFF and f.pl.on_ground


def test_vertical_params_from_params():
    vp = VP
    g = VTOL["gains_ASSUMED"]
    assert (vp.alt_kp, vp.vz_kp, vp.vz_ki, vp.vz_i_max) == (g["alt_kp"], g["vz_kp"], g["vz_ki"], g["vz_i_max"])
    assert vp.u_hover == pytest.approx(math.sqrt(1 / 1.19), abs=1e-12) == hover_throttle(MASS, T_MAX, 2.0)
    assert vp.ramp_ticks == ticks_ge(P["mission"]["throttle_ramp_s"], HZ) == 125
    assert vp.landed_confirm_ticks == 125 and vp.td_vz_still_mps == 0.5 * VTOL["touchdown_rate_mps"]
    assert vp.cos_tilt_min == pytest.approx(math.cos(math.radians(P["mission"]["tilt_abort_deg"])))
    assert vp.hover_margin == pytest.approx(1.0 - vp.u_hover)
    assert VerticalParams.from_params(P, MASS, T_MAX, vz_kp=0.3).vz_kp == 0.3      # tuning overrides
    heavy = VerticalParams.from_params(P, mass_kg=T_MAX / G0 * 1.05, t_max_n=T_MAX)
    assert heavy.u_hover > 1.0 and heavy.hover_margin < 0                         # cannot hover: allowed, shown
    for bad in (dict(u_min=1.0), dict(u_max=1.2), dict(climb_rate_mps=0.0), dict(touchdown_rate_mps=2.0),
                dict(vz_kp=-0.1), dict(climb_rate_mps=1e-4)):                    # < 1 um per tick
        with pytest.raises(ValueError):
            VerticalParams.from_params(P, MASS, T_MAX, **bad)
    # v0.7.1: integer reference steps (exact), feed-forward, touchdown block, thrust lag
    td = VTOL["touchdown"]
    assert (vp.up_um, vp.dn_um, vp.td_um, vp.flare_um) == (4000, 2400, 1200, 1_000_000)
    assert vp.effective_rate(vp.up_um) == VTOL["climb_rate_mps"] and vp.effective_rate(vp.dn_um) == VTOL["descent_rate_mps"]
    assert vp.vz_ff is VTOL["vz_ref_feedforward"] is True and vp.support_g == td["support_g_ASSUMED"]
    assert vp.floor_ticks == ticks_ge(td["floor_s_ASSUMED"], HZ) == 250 and vp.profile == "field"
    assert vp.thrust_tau_s == P["edf"]["thrust_time_constant_s"]
    assert vp.thrust_alpha == pytest.approx(1 - math.exp(-DT / vp.thrust_tau_s), rel=1e-15)
    assert vp.still_alpha == pytest.approx(1 - math.exp(-DT / vcm.TD_STILL_LPF_TAU_S_ASSUMED), rel=1e-15)
    assert dataclasses.replace(vp, climb_rate_mps=0.5).up_um == 2000                # derived values follow overrides
    # on the ground the deepest command leaves u above u_min: the floor signature cannot occur with these limits
    assert vp.u_floor_reachable == pytest.approx(vp.u_hover - vp.vz_kp * vp.vz_max_down_mps - vp.vz_i_max)
    assert vp.u_floor_reachable > vp.u_min


def test_build_profiles_field_and_demo():
    """vtol_values merges mission.vtol with profiles[profile]; only the listed keys change, every rule stays."""
    assert vtol_profiles(P) == ("field", "demo") and VTOL["profile"] == "field"
    f = vtol_values(P)
    assert f["profile"] == "field" and "profiles" not in f
    assert {k: v for k, v in f.items() if k != "profile"} == {k: v for k, v in VTOL.items() if k not in ("profile", "profiles")}
    d = vtol_values(P, "demo")
    demo = VTOL["profiles"]["demo"]
    assert d["profile"] == "demo" and all(d[k] == demo[k] for k in demo)
    assert all(d[k] == VTOL[k] for k in VTOL if k not in demo and k not in ("profile", "profiles"))
    assert vtol_values(P) is not vtol_values(P) and VTOL["hover_altitude_m"] == 5.0          # deep copies, params untouched
    with pytest.raises(ValueError):
        vtol_values(P, "indoor")
    with pytest.raises(ValueError):
        vtol_values(P, "note")                                          # the profiles block's note is not a profile
    q = copy.deepcopy(P)
    q["mission"]["vtol"]["profiles"]["demo"]["hover_altitude"] = 1.0    # misspelt key: refused, never ignored
    with pytest.raises(ValueError):
        vtol_values(q, "demo")
    q = copy.deepcopy(P)
    q["mission"]["vtol"]["profile"] = "demo"                            # the default profile follows params.json
    assert SupervisorParams.from_params(q).profile == "demo"
    sd = SupervisorParams.from_params(P, profile="demo")
    vd = VerticalParams.from_params(P, MASS, T_MAX, profile="demo")
    assert (sd.hover_dm, sd.alt_step_dm, sd.alt_min_dm, sd.alt_max_dm) == (15, 5, 10, 30)
    assert (sd.ceiling_m, sd.v_geofence_m, sd.v_geofence_dm, sd.profile) == (5.0, 8.0, 80, "demo")
    assert (vd.up_um, vd.dn_um, vd.td_um, vd.flare_um, vd.profile) == (2000, 1600, 1200, 500_000, "demo")
    # every rule threshold that the profile does not list is the field value
    same = ("spool_ticks", "link_land_ticks", "endurance_ticks", "hard_cutoff_ticks", "v_fall_vz_mps", "v_fall_confirm_ticks",
            "safe_after_landed_ticks", "land_timeout_ticks", "tilt_abort_deg", "imu_fail_ticks", "baro_fail_ticks")
    assert all(getattr(sd, k) == getattr(SP, k) for k in same)
    # demo baro-lost limit: 1.5 (h / 0.4 + 0.5 / 0.3) + 5 s
    assert (sd.baro_limit_base_ticks, sd.baro_limit_mt_per_dm) == (ticks_ge(1.5 * 0.5 / 0.3 + 5.0, HZ), 93750)
    assert sd.baro_limit_ticks(1.5) == sd.baro_limit_base_ticks + math.ceil(15 * 93.75)


def test_vtol_attitude_gain_override_rate_i_max_z():
    """control.vtol_overrides (architect v0.7.1b, physics review: the body-axis rate integrator sat pinned at
    rate_i_max_z 0.15 in every hover against the EDF swirl): the VTOL build flies rate_i_max_z 0.25 on top of
    sim/out/gains.json; the ballistic build exactly its gains (the v0.6.3 dict).  A constant yaw-rate error winds the
    integrator to the build's limit."""
    from sim.controller import AttitudeController, MixerPlant, gains_for_mode, gains_from_control, vtol_overrides
    from sim.vehicle import load_vehicle
    ctl = P["control"]
    with open(os.path.join(_ROOT, "sim", "out", "gains.json"), encoding="utf-8") as f:
        gj = json.load(f)
    ov = vtol_overrides(ctl)             # v0.9: + the light-vehicle gains and options (test_v09_vtol_overrides_*)
    assert ov["rate_i_max_z"] == 0.25 and ctl["rate_i_max_z"] == gj["rate_i_max_z"] == 0.15
    gb = gains_for_mode(ctl, gj, lp.MODE_BALLISTIC)
    assert gb == gains_from_control(ctl, gj) and all(type(gb[k]) is type(v) for k, v in gains_from_control(ctl, gj).items())
    gv = gains_for_mode(ctl, gj, lp.MODE_VTOL)
    assert all(gv[k] == v for k, v in ov.items()) and {k: v for k, v in gv.items() if k not in ov} == {
        k: v for k, v in gb.items() if k not in ov}
    for bad in ({"rate_i_max_q": 0.3}, {"loop_hz": 500}):
        with pytest.raises(ValueError):
            vtol_overrides(dict(ctl, vtol_overrides=bad))
    assert vtol_overrides({k: v for k, v in ctl.items() if k != "vtol_overrides"}) == {}
    v = load_vehicle()
    plant = MixerPlant(l_v_m=v.l_v_m, r_v_m=v.r_v_m, s_v_m2=v.s_v_m2, cl_alpha=float(P["vane"]["cl_alpha_per_rad"]),
                       a_e_m2=v.a_e_m2, t_max_n=v.t_max_n, throttle_exp=float(P["edf"]["throttle_exponent"]),
                       delta_max_rad=math.radians(P["vane"]["max_deflection_cmd_deg"]),
                       pwm_center_us=float(P["servo"]["pwm_center_us"]), pwm_us_per_deg=float(P["servo"]["pwm_us_per_deg"]))
    for mode, lim in ((lp.MODE_BALLISTIC, 0.15), (lp.MODE_VTOL, 0.25)):
        ac = AttitudeController(ctl, plant, gains_for_mode(ctl, gj, mode))
        ac.set_yaw_reference(np.array([1.0, 0.0, 0.0, 0.0]))
        for _ in range(4000):                                               # 16 s of a -0.3 rad/s body-z rate
            co = ac.update(np.array([1.0, 0.0, 0.0, 0.0]), np.array([0.0, 0.0, -0.3]), 0.9)
        assert co.integrator[2] == lim and not co.saturated


def test_vehicle_hover_throttle_and_authority():
    """The real vehicle (sim.vehicle, the 6-DOF's mass) hovers below u_max, with little authority left (reported)."""
    from sim.vehicle import load_vehicle
    v = load_vehicle()
    vp = VerticalParams.from_params(v.params, mass_kg=v.full.mass_kg, t_max_n=v.t_max_n)
    assert 0.85 < vp.u_hover < vp.u_max and 0.0 < vp.hover_margin < 0.15
    # at u_max the vertical acceleration margin is T_max / (m g) - 1
    assert (vp.u_max / vp.u_hover) ** 2 - 1.0 == pytest.approx(v.t_max_n / (v.full.mass_kg * G0) - 1.0, rel=1e-9)


# =========================================================================== #
# supervisor harness (VTOL build): healthy, upright, on the pad by default
# =========================================================================== #
class _V:
    def __init__(self, sp: SupervisorParams = SP, seed: int = 3, reset_in_flight: bool = False, **inp):
        self.sp = sp
        self.sup = OnboardSupervisor(sp, seed=seed, reset_in_flight=reset_in_flight)
        self.k = 0
        self.seq = 0
        self.base = dict(h_m=0.0, vz_mps=0.0, tilt_deg=0.5, v_cell=4.1)
        self.base.update(inp)
        self.out = None

    def tick(self, frames=(), **kw):
        d = dict(self.base)
        d.update(kw)
        self.out = self.sup.update(OnboardInputs(tick=self.k, frames=list(frames), **d))
        self.k += 1
        return self.out

    def send(self, cmd: int, arg: int = 0, nonce: int | None = None, seq: int | None = None, **kw):
        if seq is None:
            self.seq = (self.seq + 1) & 0xFF
            seq = self.seq
        nonce = self.sup.nonce if nonce is None else nonce
        return self.tick([UplinkFrame(seq, lp.Command(cmd, arg, nonce))], **kw)

    def hb(self, **kw):
        return self.tick([UplinkFrame(self.seq, lp.Command(lp.CMD_NONE))], **kw)

    def run(self, n: int, hb_every: int | None = 50, **kw):
        for i in range(n):
            if hb_every and i % hb_every == 0:
                self.hb(**kw)
            else:
                self.tick(**kw)
        return self.out

    @property
    def code(self) -> int:
        return self.sup.exec_log[-1]["code"]

    @property
    def at(self) -> tuple[State, int]:
        return self.sup.state, self.sup.phase


def _to(label: str, sp: SupervisorParams = SP, **kw) -> _V:
    """A harness in SAFE / SPOOL / CLIMB / HOLD / LAND / LANDED / ABORT / RELEASED (the vertical controller's flags
    are given as inputs: at_target / touchdown for one tick)."""
    h = _V(sp, **kw)
    h.hb()
    h.tick()
    if label == "SAFE":
        return h
    assert h.send(lp.CMD_ON).state == A_ and h.sup.phase == SPOOL
    if label == "SPOOL":
        return h
    h.run(sp.spool_ticks)
    assert h.at == (P_, CLIMB), h.at
    if label in ("CLIMB", "ABORT", "RELEASED"):
        if label != "CLIMB":
            assert h.send(lp.CMD_KILL).state == AB_
            if label == "RELEASED":
                h.run(sp.abort_release_ticks)
                assert h.sup.state == R_
        return h
    h.tick(vc_at_target=True)
    assert h.at == (P_, HOLD)
    if label == "HOLD":
        return h
    h.send(lp.CMD_OFF)
    assert h.at == (P_, LAND)
    if label == "LAND":
        return h
    h.tick(vc_touchdown=True)
    assert h.at == (U_, LANDED)
    assert label == "LANDED"
    return h


# =========================================================================== #
# command x state x phase
# =========================================================================== #
VARIANTS = {"ON": (lp.CMD_ON, True), "ON_badnonce": (lp.CMD_ON, False), "OFF": (lp.CMD_OFF, True), "UP": (lp.CMD_UP, True),
            "DOWN": (lp.CMD_DOWN, True), "KILL": (lp.CMD_KILL, True), "ARM": (lp.CMD_ARM, True),
            "DISARM": (lp.CMD_DISARM, True), "LAUNCH": (lp.CMD_LAUNCH, True), "THROTTLE": (lp.CMD_THROTTLE, True),
            "CUT": (lp.CMD_CUT, True), "RELEASE": (lp.CMD_RELEASE, True), "UNKNOWN_12": (12, True),
            "UNKNOWN_255": (255, True)}
_RS, _RM, _RU, _RP = lp.REJ_STATE, lp.REJ_MODE, lp.REJ_UNKNOWN_CMD, lp.REJ_RELEASE_PENDING
_MODE = {c: _RM for c in ("ARM", "DISARM", "LAUNCH", "THROTTLE", "CUT", "RELEASE")}
_UNK = {"UNKNOWN_12": _RU, "UNKNOWN_255": _RU}
# start -> command -> (state, phase, code) after the tick (code 0 = executed); unlisted: no change with that code
VTABLE = {
    "SAFE": dict(ON=(A_, SPOOL, 0), ON_badnonce=lp.REJ_NONCE, OFF=_RS, UP=_RS, DOWN=_RS, KILL=_RS, **_MODE, **_UNK),
    "SPOOL": dict(ON=_RS, ON_badnonce=_RS, OFF=(S_, NONE, 0), UP=_RS, DOWN=_RS, KILL=(S_, NONE, 0), **_MODE, **_UNK),
    "CLIMB": dict(ON=_RS, ON_badnonce=_RS, OFF=(P_, LAND, 0), UP=0, DOWN=0, KILL=(AB_, NONE, 0), **_MODE, **_UNK),
    "HOLD": dict(ON=_RS, ON_badnonce=_RS, OFF=(P_, LAND, 0), UP=0, DOWN=0, KILL=(AB_, NONE, 0), **_MODE, **_UNK),
    "LAND": dict(ON=_RS, ON_badnonce=_RS, OFF=0, UP=lp.REJ_LANDING, DOWN=lp.REJ_LANDING, KILL=(AB_, NONE, 0), **_MODE,
                 **_UNK),
    "LANDED": dict(ON=_RS, ON_badnonce=_RS, OFF=0, UP=_RS, DOWN=_RS, KILL=(AB_, NONE, 0), **_MODE, **_UNK),
    "ABORT": dict(ON=_RP, ON_badnonce=_RP, OFF=_RP, UP=_RP, DOWN=_RP, KILL=0, **_MODE, **_UNK),
    "RELEASED": dict(ON=_RS, ON_badnonce=_RS, OFF=_RS, UP=_RS, DOWN=_RS, KILL=0, **_MODE, **_UNK),
}


@pytest.mark.parametrize("start", list(VTABLE))
def test_vtol_every_command_in_every_state_and_phase(start):
    for name, (code_cmd, good_nonce) in VARIANTS.items():
        h = _to(start)
        st0, ph0, tgt0, n0 = h.sup.state, h.sup.phase, h.sup.tgt_dm, h.sup.nonce
        out = h.send(code_cmd, 500 if code_cmd == lp.CMD_THROTTLE else 0, nonce=n0 if good_nonce else n0 ^ 0x5A5A)
        want = VTABLE[start][name]
        want_state, want_phase, want_code = (st0, ph0, want) if isinstance(want, int) else want
        assert (out.state, out.phase, h.code) == (want_state, want_phase, want_code), (start, name)
        tl = out.telemetry
        assert out.reply_seq == h.seq and tl.ack_seq == h.seq and tl.is_v2 and tl.phase == out.phase
        assert tl.flag(lp.FLAG_REJECTED) == (want_code != 0) and (not want_code or tl.reason == want_code)
        moved = start == "SAFE" and name == "ON"                      # only an executed ON advances the nonce
        assert h.sup.nonce == (lfsr16_next(n0) if moved else n0), (start, name)
        if name in ("UP", "DOWN") and want_code == 0:
            assert h.sup.tgt_dm == tgt0 + (SP.alt_step_dm if name == "UP" else -SP.alt_step_dm)
            assert tl.h_tgt_dm == h.sup.tgt_dm and out.h_tgt_m == h.sup.tgt_dm / 10
        else:
            assert h.sup.tgt_dm == (SP.hover_dm if moved else tgt0)
        assert out.motor_on == (out.state == P_) and out.control_active == (out.state == P_)
        assert out.buzzer == (out.state == A_) and out.vanes_centre == (out.state != P_)


def test_vtol_on_gates_in_order():
    """ON: state, nonce, then the v0.4 ARM gates in their order, then the cumulative motor time."""
    sp = SP
    cases = [(dict(arm_switch=False), lp.REJ_ARM_SWITCH), (dict(imu_ok=False), lp.REJ_SENSORS),
             (dict(baro_ok=False), lp.REJ_SENSORS), (dict(aligned=False), lp.REJ_NOT_ALIGNED),
             (dict(tilt_deg=sp.arm_tilt_max_deg), lp.REJ_TILT), (dict(v_cell=sp.min_cell_v - 1e-6), lp.REJ_VCELL),
             (dict(tilt_deg=sp.arm_tilt_max_deg - 1e-6, v_cell=sp.min_cell_v), 0),
             (dict(arm_switch=False, tilt_deg=30.0), lp.REJ_ARM_SWITCH)]
    for kw, code in cases:
        h = _V()
        h.hb()
        out = h.send(lp.CMD_ON, **kw)
        assert h.code == code and out.state == (A_ if code == 0 else S_), kw
    h = _V()
    h.hb()
    h.send(lp.CMD_ON, nonce=h.sup.nonce ^ 1, arm_switch=False)          # the nonce is checked before the gates
    assert h.code == lp.REJ_NONCE
    h = _V()
    h.send(lp.CMD_ON)                                                   # the first frame after a silence cannot take off
    assert h.code == lp.REJ_LINK
    h.send(lp.CMD_ON)
    assert h.code == 0
    h = _V()
    h.hb()
    h.sup.powered_ticks = sp.endurance_ticks                            # the pack's flight time is used up
    h.send(lp.CMD_ON)
    assert h.code == lp.REJ_ENDURANCE
    h.sup.powered_ticks = sp.endurance_ticks - 1
    h.send(lp.CMD_ON)
    assert h.code == 0


def test_vtol_up_down_limits_exact_integer_target():
    h = _to("HOLD")
    n_up = (SP.alt_max_dm - SP.hover_dm) // SP.alt_step_dm
    for i in range(n_up):
        h.send(lp.CMD_UP)
        assert h.code == 0
    assert h.sup.tgt_dm == SP.alt_max_dm == 100 and h.out.telemetry.h_tgt_dm == 100
    h.send(lp.CMD_UP)
    assert h.code == lp.REJ_ALT_LIMIT and h.sup.tgt_dm == 100 and h.out.telemetry.reason == lp.REJ_ALT_LIMIT
    for i in range((SP.alt_max_dm - SP.alt_min_dm) // SP.alt_step_dm):
        h.send(lp.CMD_DOWN)
        assert h.code == 0
    assert h.sup.tgt_dm == SP.alt_min_dm == 20
    h.send(lp.CMD_DOWN)
    assert h.code == lp.REJ_ALT_LIMIT and h.sup.tgt_dm == 20
    # a target that is not on the step grid: UP stops one short of the limit
    sp = dataclasses.replace(SP, alt_step_dm=8)
    k = (sp.alt_max_dm - sp.hover_dm) // 8                     # 6 steps: 50 -> 98, a 7th would pass 100
    assert (sp.alt_max_dm - sp.hover_dm) % 8 != 0
    h = _to("CLIMB", sp)
    for _ in range(k):
        h.send(lp.CMD_UP)
    assert h.sup.tgt_dm == sp.hover_dm + 8 * k == 98 and h.code == 0
    h.send(lp.CMD_UP)
    assert h.code == lp.REJ_ALT_LIMIT and h.sup.tgt_dm == 98
    # UP during CLIMB changes the target; the phase stays CLIMB until the reference arrives
    h = _to("CLIMB")
    h.send(lp.CMD_UP)
    assert h.at == (P_, CLIMB) and h.out.h_tgt_m == 6.0


def test_vtol_repeated_and_old_seq_never_execute_twice():
    h = _to("HOLD")
    h.send(lp.CMD_UP)
    s = h.seq
    h.send(lp.CMD_UP, seq=s)                                            # the ground's repeat: acknowledged only
    h.send(lp.CMD_UP, seq=(s - 5) & 0xFF)                               # an old SEQ
    assert h.sup.tgt_dm == 60 and len([e for e in h.sup.exec_log if e["cmd"] == lp.CMD_UP]) == 1
    assert h.out.telemetry.ack_seq == s
    e = h.sup.exec_log[-1]
    assert (e["phase_before"], e["phase_after"]) == (HOLD, HOLD)


# =========================================================================== #
# timers
# =========================================================================== #
def test_spool_warning_exact_ticks_then_climb():
    h = _V()
    h.hb()
    h.tick()
    out = h.send(lp.CMD_ON)
    t0 = h.k - 1
    assert out.state == A_ and out.buzzer and not out.motor_on and out.h_tgt_m == 5.0 and out.vanes_centre
    buzz = 1
    while h.sup.state == A_:
        out = h.tick() if (h.k - t0) % 50 else h.hb()
        buzz += int(out.buzzer)
    assert h.k - 1 == t0 + SP.spool_ticks and buzz == SP.spool_ticks          # buzzer exactly spool_warning_s
    assert out.state == P_ and out.phase == CLIMB and out.motor_on and out.control_active and not out.buzzer
    assert h.sup.powered_ticks == 1 and any("spool warning complete" in e for e in out.events)


@pytest.mark.parametrize("kw", [dict(tilt_deg=SP.arm_tilt_max_deg), dict(v_cell=SP.min_cell_v - 0.01)])
def test_takeoff_gate_at_the_end_of_the_spool(kw):
    h = _to("SPOOL")
    t0 = h.k - 1
    h.run(SP.spool_ticks - 1, **kw)                                       # bad during the warning: nothing yet
    assert h.sup.state == A_
    out = h.tick(**kw)                                                    # still bad at the spool end: never start
    assert h.k - 1 == t0 + SP.spool_ticks and out.state == S_ and out.reason == lp.R_V_TAKEOFF_GATE
    assert out.flags & lp.FLAG_INTERLOCK and not out.motor_on and h.sup.powered_ticks == 0
    h = _to("SPOOL")
    h.run(SP.spool_ticks - 1, **kw)
    assert h.tick().state == P_                                           # only the end of the warning counts


def test_climb_to_hold_and_land_to_landed_use_the_previous_tick_flags():
    h = _to("CLIMB")
    assert h.tick(vc_touchdown=True).phase == CLIMB                       # touchdown means nothing outside LAND
    out = h.tick(vc_at_target=True)
    assert out.phase == HOLD and any("phase CLIMB -> HOLD" in e for e in out.events)
    assert h.tick(vc_at_target=False).phase == HOLD                      # HOLD stays (UP / DOWN retarget inside HOLD)
    h.send(lp.CMD_OFF)
    assert h.tick(vc_at_target=True).phase == LAND
    t = h.k
    out = h.tick(vc_touchdown=True)
    assert out.state == U_ and out.phase == LANDED and not out.motor_on and out.vanes_centre and h.sup.t_entry == t
    p0 = h.sup.powered_ticks
    h.run(SP.safe_after_landed_ticks - 1)
    assert h.sup.state == U_ and h.sup.powered_ticks == p0
    out = h.tick()
    assert out.state == S_ and out.phase == NONE and h.k - 1 - t == SP.safe_after_landed_ticks
    h.hb()
    h.send(lp.CMD_ON)                                                     # another ON may fly again
    assert h.code == 0 and h.sup.state == A_


# =========================================================================== #
# safety rules - exact ticks
# =========================================================================== #
def _abort_then_release(h: _V, code: int) -> None:
    out = h.out
    assert out.state == AB_ and out.reason == code and out.flags & lp.FLAG_INTERLOCK and not out.motor_on, (out.state,
                                                                                                           out.reason)
    t_ab = h.k - 1
    h.run(SP.abort_release_ticks - 1)
    assert h.sup.state == AB_ and not h.out.release
    out = h.tick()
    assert out.state == R_ and out.release and h.k - 1 - t_ab == SP.abort_release_ticks


@pytest.mark.parametrize("label", ["CLIMB", "HOLD", "LAND", "LANDED"])
def test_arm_switch_imu_failure_abort_in_flight(label):
    h = _to(label)
    out = h.hb(arm_switch=False)
    if label == "LANDED":                                                 # on the ground: SAFE, the release never fires
        assert out.state == S_ and out.reason == lp.R_ARM_SWITCH_OPEN and out.flags & lp.FLAG_INTERLOCK
        assert h.run(3 * SP.abort_release_ticks, arm_switch=False).state == S_ and not h.out.release
    else:
        _abort_then_release(h, lp.R_ARM_SWITCH_OPEN)
    h = _to(label)
    h.run(SP.imu_fail_ticks - 1, imu_ok=False)
    h.hb()                                                                # one good read resets the count
    h.run(SP.imu_fail_ticks - 1, imu_ok=False)
    assert h.sup.state == (U_ if label == "LANDED" else P_)
    h.hb(imu_ok=False)
    if label == "LANDED":                                                 # v0.7.1: motor already off -> SAFE, no release
        _safe_never_release(h, lp.R_I9_IMU, imu_ok=False)
    else:
        _abort_then_release(h, lp.R_I9_IMU)


def _safe_never_release(h: _V, code: int, **kw) -> None:
    """v0.7.1 LANDED rules: the rule's code latched, SAFE at once, and the release never opens afterwards."""
    out = h.out
    assert out.state == S_ and out.phase == NONE and out.reason == code and out.flags & lp.FLAG_INTERLOCK, (out.state,
                                                                                                           out.reason)
    assert not out.motor_on and h.sup.release_deadline is None and any("no release" in e for e in out.events)
    h.run(3 * SP.abort_release_ticks, **kw)
    assert h.sup.state == S_ and not h.out.release and not h.sup.release_open


def test_pad_rules_in_spool():
    for kw, code in ((dict(arm_switch=False), lp.R_ARM_SWITCH_OPEN), (dict(imu_ok=False), lp.R_I9_IMU),
                     (dict(baro_ok=False), lp.R_I9_BARO)):
        h = _to("SPOOL")
        n = 1 if "arm_switch" in kw else (SP.imu_fail_ticks if "imu_ok" in kw else SP.baro_fail_ticks)
        h.run(n - 1, **kw)
        assert h.sup.state == A_ or n == 1
        out = h.hb(**kw)
        assert out.state == S_ and out.reason == code and not out.release, kw
    h = _to("SPOOL", dataclasses.replace(SP, spool_ticks=10 * SP.link_disarm_ticks))
    last = h.k - 1
    h.run(SP.link_disarm_ticks - 1, hb_every=None)
    assert h.sup.state == A_
    out = h.tick()
    assert out.state == S_ and out.reason == lp.R_I1P_LINK_LOSS_PAD and h.k - 1 - last == SP.link_disarm_ticks


def test_baro_failure_lands_on_the_inertial_speed():
    for label in ("CLIMB", "HOLD"):
        h = _to(label)
        h.run(SP.baro_fail_ticks - 1, baro_ok=False)
        assert h.at[1] == (CLIMB if label == "CLIMB" else HOLD) and not h.out.baro_lost
        out = h.hb(baro_ok=False)
        assert out.state == P_ and out.phase == LAND and out.baro_lost and out.reason == lp.R_I9_BARO
        assert out.flags & lp.FLAG_INTERLOCK and out.motor_on
        out = h.run(3 * SP.baro_fail_ticks)                              # the baro is back: still the inertial landing
        assert out.baro_lost and out.phase == LAND
    h = _to("LAND")
    out = h.run(SP.baro_fail_ticks, baro_ok=False)
    assert out.phase == LAND and out.baro_lost and out.reason == lp.R_I9_BARO
    h = _to("LANDED")
    out = h.run(SP.baro_fail_ticks + 5, baro_ok=False)
    assert out.state == U_ and not out.baro_lost and out.reason == lp.REASON_NONE
    # latched until the next ON
    h = _to("HOLD")
    h.run(SP.baro_fail_ticks, baro_ok=False)
    h.tick(vc_touchdown=True)
    h.run(SP.safe_after_landed_ticks)
    assert h.sup.state == S_ and h.out.baro_lost
    h.hb()
    h.send(lp.CMD_ON)
    assert h.code == 0 and not h.out.baro_lost


def _silence_until_due(h: _V, n: int) -> int:
    """No frames until the tick before ``n`` ticks have passed since the last frame; returns that last frame's tick."""
    last = h.sup.last_frame_tick
    h.run(last + n - h.k, hb_every=None)
    assert h.sup.ticks_since_frame(h.k) == n
    return last


def test_link_loss_lands_exactly_and_only_in_climb_hold():
    for label in ("CLIMB", "HOLD"):
        h = _to(label)
        last = _silence_until_due(h, SP.link_land_ticks)
        assert h.sup.phase == (CLIMB if label == "CLIMB" else HOLD)
        out = h.tick()
        assert out.phase == LAND and out.reason == lp.R_V_LINK_LAND and h.k - 1 - last == SP.link_land_ticks
        assert not out.flags & lp.FLAG_LINK_OK
        out = h.run(4 * SP.link_land_ticks, hb_every=None)              # no more rule: lands without the link
        assert out.state == P_ and out.phase == LAND
    for label in ("LAND", "LANDED"):
        h = _to(label)
        out = h.run(2 * SP.link_land_ticks, hb_every=None)
        assert out.reason == lp.REASON_NONE and out.state != AB_


@pytest.mark.parametrize("label", ["CLIMB", "HOLD", "LAND"])
def test_tilt_and_rate_abort_powered_only(label):
    h = _to(label)
    assert h.tick(tilt_deg=SP.tilt_abort_deg).state == P_
    h.tick(tilt_deg=SP.tilt_abort_deg + 0.01)
    _abort_then_release(h, lp.R_I2_TILT)
    for kw in (dict(gyro_rad_s=(SP.tilt_rate_abort_rad_s * 1.01, 0.0, 0.0)), dict(gyro_rad_s=(0.0, 0.0, -SP.spin_rate_abort_rad_s * 1.01)),
               dict(gyro_saturated=True)):
        h = _to(label)
        h.run(SP.rate_persist_ticks - 1, **kw)
        h.tick()
        h.run(SP.rate_persist_ticks - 1, **kw)
        assert h.sup.state == P_
        h.tick(**kw)
        _abort_then_release(h, lp.R_I3_RATE)
    h = _to("LANDED")
    assert h.run(20, tilt_deg=80.0, gyro_saturated=True).state == U_    # on the ground after landing: no I2 / I3


@pytest.mark.parametrize("label", ["CLIMB", "HOLD", "LAND", "LANDED"])
def test_geofence_and_fall_abort(label):
    """V4 / V5: ABORT in flight; v0.7.1 in LANDED: SAFE, never ABORT (landed_rules_note)."""
    h = _to(label)
    assert h.tick(h_m=SP.v_geofence_m).state not in (AB_, S_)
    h.tick(h_m=SP.v_geofence_m + 0.01)
    if label == "LANDED":
        _safe_never_release(h, lp.R_V_GEOFENCE)
    else:
        _abort_then_release(h, lp.R_V_GEOFENCE)
    h = _to(label)
    h.run(SP.v_fall_confirm_ticks - 1, vz_mps=SP.v_fall_vz_mps)
    h.tick(vz_mps=SP.v_fall_vz_mps + 0.01)                               # one slower tick restarts the count
    h.run(SP.v_fall_confirm_ticks - 1, vz_mps=SP.v_fall_vz_mps)
    assert h.sup.state not in (AB_, S_)
    h.tick(vz_mps=SP.v_fall_vz_mps)                                      # inclusive threshold
    if label == "LANDED":
        _safe_never_release(h, lp.R_V_FALL)
    else:
        _abort_then_release(h, lp.R_V_FALL)
    h = _to("SPOOL")
    assert h.run(2 * SP.v_fall_confirm_ticks, vz_mps=-20.0, h_m=100.0).state == A_   # pad: neither


def test_hard_cutoff_and_endurance_land_cumulative_motor_time():
    h = _to("HOLD")
    h.sup.powered_ticks = SP.endurance_ticks - 1
    h.tick()                                                              # the last tick before the budget
    assert h.sup.phase == HOLD and h.sup.powered_ticks == SP.endurance_ticks
    out = h.tick()
    assert out.phase == LAND and out.reason == lp.R_V_ENDURANCE
    h.sup.powered_ticks = SP.hard_cutoff_ticks - 1                       # still landing at the hard cut-off
    assert h.tick().state == P_
    h.tick()
    _abort_then_release(h, lp.R_V_HARD_CUTOFF)
    # the budget is cumulative over flights (the pack is not recharged): the second flight lands at the same total
    sp = dataclasses.replace(SP, endurance_ticks=1200, hard_cutoff_ticks=2400, spool_ticks=10, safe_after_landed_ticks=5)
    h = _to("HOLD", sp)
    h.run(400)
    h.send(lp.CMD_OFF)
    h.run(100)
    h.tick(vc_touchdown=True)
    h.run(10)
    assert h.sup.state == S_
    used = h.sup.powered_ticks
    h.send(lp.CMD_ON)
    h.run(sp.spool_ticks + 1)
    assert h.at == (P_, CLIMB)
    n = 0
    while h.sup.phase == CLIMB:
        h.tick() if n % 50 else h.hb()
        n += 1
    assert h.sup.reason == lp.R_V_ENDURANCE and h.sup.powered_ticks == sp.endurance_ticks + 1 and used < 1200
    h.tick(vc_touchdown=True)
    h.run(10)
    h.send(lp.CMD_ON)
    assert h.code == lp.REJ_ENDURANCE


@pytest.mark.parametrize("label", ["CLIMB", "HOLD"])
def test_vcell_and_ceiling_land(label):
    h = _to(label)
    assert h.tick(v_cell=SP.min_cell_v).phase != LAND
    out = h.tick(v_cell=SP.min_cell_v - 0.01)
    assert out.phase == LAND and out.reason == lp.R_I8_VCELL and out.state == P_
    h = _to(label)
    assert h.tick(h_m=SP.ceiling_m).phase != LAND
    out = h.tick(h_m=SP.ceiling_m + 0.01)
    assert out.phase == LAND and out.reason == lp.R_V_CEILING
    h = _to("LAND")
    out = h.run(20, v_cell=0.0, h_m=SP.ceiling_m + 1.0)                    # already landing: nothing more
    assert out.phase == LAND and out.reason == lp.REASON_NONE


def test_rule_order_when_several_apply():
    h = _to("HOLD")
    _silence_until_due(h, SP.link_land_ticks)
    out = h.tick(tilt_deg=60.0, v_cell=0.0)                               # tilt (ABORT) before link loss (LAND)
    assert out.state == AB_ and out.reason == lp.R_I2_TILT
    h = _to("HOLD")
    _silence_until_due(h, SP.link_land_ticks)
    assert h.sup.phase == HOLD
    out = h.tick(v_cell=0.0, h_m=SP.ceiling_m + 1)                       # V1 before I8 before V3
    assert out.phase == LAND and out.reason == lp.R_V_LINK_LAND
    h = _to("HOLD")
    out = h.tick(v_cell=0.0, h_m=SP.ceiling_m + 1)
    assert out.reason == lp.R_I8_VCELL
    h = _to("HOLD")
    h.run(SP.v_fall_confirm_ticks - 1, vz_mps=-5.0)
    out = h.tick(vz_mps=-5.0, h_m=SP.v_geofence_m + 1)                   # V4 geofence before V5 fall
    assert out.reason == lp.R_V_GEOFENCE
    h = _to("HOLD")
    out = h.hb(arm_switch=False, imu_ok=False, tilt_deg=80.0)            # the arm switch first
    assert out.state == AB_ and out.reason == lp.R_ARM_SWITCH_OPEN
    h = _to("HOLD")                                                       # commands first, the rule has the last word
    out = h.send(lp.CMD_UP, tilt_deg=SP.tilt_abort_deg + 5)
    assert h.code == 0 and out.state == AB_ and out.reason == lp.R_I2_TILT
    h = _to("CLIMB")                                                      # a rule pre-empts the timers of the tick
    out = h.tick(vc_at_target=True, v_cell=0.0)
    assert out.phase == LAND and out.reason == lp.R_I8_VCELL


def test_i10_reset_in_flight_boots_abort_and_releases():
    h = _V(reset_in_flight=True)
    out = h.hb()
    assert out.state == AB_ and not out.motor_on and out.reason == lp.R_I10_RESET and out.flags & lp.FLAG_INTERLOCK
    h.run(SP.abort_release_ticks - 1)
    assert h.sup.state == AB_
    out = h.tick()
    assert out.state == R_ and out.release and h.k - 1 == SP.abort_release_ticks
    h.send(lp.CMD_ON)
    assert h.code == lp.REJ_STATE and h.sup.state == R_                    # flying again needs a power cycle


def test_interlock_reason_outranks_rejections_vtol():
    h = _to("HOLD")
    h.tick(v_cell=0.0)
    out = h.send(lp.CMD_UP)
    assert h.code == lp.REJ_LANDING and out.telemetry.reason == lp.R_I8_VCELL and out.telemetry.flag(lp.FLAG_REJECTED)


def test_vtol_telemetry_v2_fields_and_ballistic_v1():
    h = _to("HOLD")
    out = h.hb(h_m=4.96, vz_mps=-0.05, tilt_deg=1.25, v_cell=3.92, throttle_prev=0.9183, roll_deg=1.0, loop_max_us=2900.0)
    t = out.telemetry
    assert t.is_v2 and t.phase == HOLD and t.h_tgt_dm == 50 and (t.h_dm, t.vz_cms, t.tilt_cdeg) == (50, -5, 125)
    assert t.throttle_milli == 918 and out.throttle_milli == 0 and out.throttle == 0.0    # VTOL: throttle = previous ESC
    assert lp.encode_telemetry(90, out.reply_seq, t)[4] == lp.TYPE_TELEM2
    want = lp.FLAG_ARM_SWITCH | lp.FLAG_SENSORS_OK | lp.FLAG_ALIGNED | lp.FLAG_LINK_OK | lp.FLAG_CONTROL_ACTIVE
    assert t.flags == want and t.reason == lp.REASON_NONE
    for label, ph, tgt in (("SAFE", NONE, 0), ("SPOOL", SPOOL, 50), ("CLIMB", CLIMB, 50), ("LAND", LAND, 0),
                           ("LANDED", LANDED, 0), ("ABORT", NONE, 0)):
        t = _to(label).hb().telemetry
        assert (t.phase, t.h_tgt_dm) == (ph, tgt), label
    hb = _V(SupervisorParams.from_params(P, mode=lp.MODE_BALLISTIC))
    hb.hb()
    t = hb.hb().telemetry
    assert not t.is_v2 and len(t.pack()) == 28 and hb.out.phase == NONE and not hb.out.buzzer


def vtol_random_sequence(rng: random.Random, sp: SupervisorParams, n_ticks: int, reset: bool = False, clean: bool = False):
    """Random ground + sensors + vertical-controller flags for the VTOL supervisor (a template for the C cross-check):
    yields (inputs, outputs, supervisor) per tick.  The operator mostly flies (ON, UP / DOWN, OFF), sometimes sends what
    does not apply (other build's codes, junk, wrong nonces, repeated / old SEQs); faults are random and more likely
    in flight.  ``clean``: no faults and no OFF / KILL in flight (the flights run into the flight-time rules)."""
    W = {S_: {lp.CMD_ON: 12, lp.CMD_NONE: 4, lp.CMD_OFF: 1, lp.CMD_UP: 1, lp.CMD_KILL: 1, lp.CMD_ARM: 1, 12: 1},
         A_: {lp.CMD_NONE: 10, lp.CMD_OFF: 1, lp.CMD_KILL: 1, lp.CMD_ON: 1, lp.CMD_UP: 1, lp.CMD_LAUNCH: 1},
         P_: {lp.CMD_NONE: 12, lp.CMD_UP: 4, lp.CMD_DOWN: 4, lp.CMD_OFF: 1, lp.CMD_KILL: 0.3, lp.CMD_ON: 0.5,
              lp.CMD_THROTTLE: 0.5, lp.CMD_CUT: 0.3},
         U_: {lp.CMD_NONE: 8, lp.CMD_OFF: 1, lp.CMD_ON: 1, lp.CMD_UP: 1, lp.CMD_KILL: 0.3},
         R_: {lp.CMD_NONE: 8, lp.CMD_ON: 1, lp.CMD_KILL: 1, lp.CMD_RELEASE: 1},
         AB_: {lp.CMD_NONE: 8, lp.CMD_ON: 1, lp.CMD_KILL: 1, lp.CMD_OFF: 1, lp.CMD_DOWN: 1}}
    sup = OnboardSupervisor(sp, seed=rng.randrange(65536), reset_in_flight=reset)
    poll = rng.choice([5, 20, 50, 120])
    seq, h, vz, fault, outage = rng.randrange(256), 0.0, 0.0, {}, -1
    low_cnt, gsig_p = 0, rng.choice([0.0, 0.5, 1.0])          # v0.7.1: the controller's low_in_land / ground signature
    aligned_from, arm_closed_from = rng.choice([0, 0, rng.randrange(0, 400)]), rng.choice([0, 0, rng.randrange(0, 600)])
    for t in range(n_ticks):
        frames = []
        if rng.random() < 0.0005:
            outage = t + rng.choice([sp.link_land_ticks // 2, sp.link_land_ticks + 3, 4 * sp.link_land_ticks])
        if t % poll == 0 and t >= outage and rng.random() > 0.05:
            r = rng.random()
            if r < 0.45:
                seq = (seq + 1) & 0xFF
                w = W[sup.state]
                if clean and sup.state == P_:
                    w = {lp.CMD_NONE: 6, lp.CMD_UP: 1, lp.CMD_DOWN: 1}
                c = rng.choices(list(w), list(w.values()))[0]
            else:
                c = lp.CMD_NONE
            s = seq if rng.random() > 0.03 else (seq - rng.randrange(1, 100)) & 0xFF
            nonce = sup.nonce if rng.random() < 0.85 else rng.randrange(65536)
            frames.append(UplinkFrame(s, lp.Command(c, rng.choice([0, 500]), nonce)))
        st = sup.state
        flying = st in (P_, U_)
        for k, p in (("arm", 0.0001), ("imu", 0.0002), ("baro", 0.0003), ("tilt", 0.0003), ("rate", 0.0003),
                     ("vcell", 0.0004), ("high", 0.0003), ("fall", 0.0003), ("gate", 0.001)):
            if fault.get(k, 0) > 0:
                fault[k] -= 1
            elif not (clean and k != "gate") and rng.random() < (0.02 if (k == "gate" and st == A_) else p * (8.0 if flying else 1.0)):
                fault[k] = rng.randrange(1, 2 * max(sp.imu_fail_ticks, sp.baro_fail_ticks, sp.v_fall_confirm_ticks) + 3)
        if st == P_:
            vz += rng.gauss(0.0, 0.05)
            vz = max(min(vz, 1.5), -1.0)
        else:
            vz = 0.0 if h <= 0.0 else vz - 9.8 * 0.004
        h = max(h + vz * 0.004, -0.5)
        f = lambda k: fault.get(k, 0) > 0  # noqa: E731
        low_cnt = (low_cnt + 1 if rng.random() < 0.998 else 0) if (sup.phase == LAND and not sup.baro_lost) else 0
        inp = OnboardInputs(
            tick=t, frames=frames, h_m=(rng.choice([sp.ceiling_m + 1.0, sp.v_geofence_m + 1.0]) if f("high") else h),
            vz_mps=(sp.v_fall_vz_mps - 0.5 if f("fall") else vz), tilt_deg=(60.0 if f("tilt") else (8.0 if f("gate") else 1.0)),
            gyro_rad_s=((10.0, 0.0, 0.0) if f("rate") else (0.1, -0.1, 0.2)), v_cell=(3.0 if f("vcell") else 4.0),
            imu_ok=not f("imu"), baro_ok=not f("baro"), arm_switch=not f("arm") and t >= arm_closed_from,
            aligned=t >= aligned_from,
            vc_at_target=sup.phase == CLIMB and rng.random() < 0.004,
            vc_touchdown=sup.phase == LAND and rng.random() < (0.0005 if clean else 0.004),
            throttle_prev=rng.random() if st == P_ else 0.0,
            vc_ground_signature=sup.phase == LAND and rng.random() < gsig_p, vc_low_in_land_ticks=low_cnt)
        out = sup.update(inp)
        yield inp, out, sup


PHASES_OF_STATE = {S_: {NONE}, A_: {SPOOL}, P_: {CLIMB, HOLD, LAND}, U_: {LANDED}, AB_: {NONE}, R_: {NONE}}


def test_vtol_random_sequences_invariants_and_every_code_reachable():
    rng = random.Random(20261020)
    seen_il, seen_rej, n_ticks, trans, landed_exits = set(), set(), 0, set(), set()
    for i in range(60):
        sp = dataclasses.replace(SP, spool_ticks=rng.choice([0, 1, 30, 400]), link_land_ticks=rng.choice([60, 200]),
                                 link_disarm_ticks=rng.choice([60, 200]), endurance_ticks=rng.choice([300, 1500]),
                                 v_fall_confirm_ticks=rng.choice([1, 10]), safe_after_landed_ticks=rng.choice([0, 20]),
                                 abort_release_ticks=rng.choice([0, 12]), imu_fail_ticks=rng.choice([1, 5]),
                                 baro_fail_ticks=rng.choice([2, 20]), rate_persist_ticks=rng.choice([1, 5]),
                                 land_timeout_ticks=rng.choice([0, 40, 250]), baro_limit_base_ticks=rng.choice([0, 30, 200]),
                                 baro_limit_mt_per_dm=rng.choice([0, 1000, 62500]))
        sp = dataclasses.replace(sp, hard_cutoff_ticks=sp.endurance_ticks + rng.choice([1, 300]))
        prev_state, prev_nonce, abort_at = None, None, None
        for inp, out, sup in vtol_random_sequence(rng, sp, rng.choice([2000, 4000]), reset=rng.random() < 0.05,
                                                  clean=rng.random() < 0.35):
            n_ticks += 1
            assert out.phase in PHASES_OF_STATE[out.state], (out.state, out.phase)
            assert out.motor_on == (out.state == P_) == out.control_active and out.buzzer == (out.state == A_)
            assert out.throttle == 0.0 and out.throttle_milli == 0                       # VTOL: the ESC is the controller's
            if out.phase in (SPOOL, CLIMB, HOLD):
                assert sp.alt_min_dm <= sup.tgt_dm <= sp.alt_max_dm and out.h_tgt_m == sup.tgt_dm / 10
            else:
                assert out.h_tgt_m == 0.0
            ex = [e for e in sup.exec_log if e["tick"] == inp.tick]
            executed_on = any(e["cmd"] == lp.CMD_ON and e["code"] == 0 for e in ex)
            if prev_state == U_ and out.state != U_:
                killed = any(e["cmd"] == lp.CMD_KILL and e["code"] == 0 for e in ex)
                landed_exits.add("KILL" if (out.state == AB_ and killed) else out.state.name)
            if prev_nonce is not None:
                assert (sup.nonce != prev_nonce) == executed_on
            for e in ex:
                if e["code"]:
                    seen_rej.add(e["code"])
            if out.state == AB_ and prev_state != AB_:
                abort_at = inp.tick
            if out.state == R_ and prev_state == AB_ and abort_at is not None:
                assert inp.tick - abort_at == sp.abort_release_ticks
            for e in out.events:                                                          # elementary transitions
                a, sep, rest = e.partition(" -> ")
                if sep and a.isupper() and " " not in a:
                    trans.add((a, rest.split(":")[0]))
            if out.reason and out.reason < 0x40:
                seen_il.add(out.reason)
            if out.telemetry is not None:
                assert out.telemetry.is_v2 and out.telemetry.phase == out.phase
            prev_state, prev_nonce = out.state, sup.nonce
    assert n_ticks > 150000
    want_il = set(lp.VTOL_INTERLOCK_REASONS) - {lp.R_I10_RESET}
    assert want_il <= seen_il | {lp.R_I10_RESET}, sorted(want_il - seen_il)
    assert set(lp.VTOL_REJECT_REASONS) <= seen_rej, sorted(set(lp.VTOL_REJECT_REASONS) - seen_rej)
    assert not seen_rej & {lp.REJ_THROTTLE_ZERO, lp.REJ_RELEASED}
    allowed = {("SAFE", "ARMED"), ("ARMED", "SAFE"), ("ARMED", "POWERED"), ("POWERED", "UNPOWERED"), ("POWERED", "ABORT"),
               ("UNPOWERED", "SAFE"), ("UNPOWERED", "ABORT"), ("ABORT", "RELEASED")}
    assert trans <= allowed and trans >= allowed - {("UNPOWERED", "ABORT")}, trans
    # v0.7.1: LANDED leaves only to SAFE (timer, arm switch, I9 IMU, V4, V5) or by KILL to ABORT
    assert landed_exits <= {"SAFE", "KILL"}, landed_exits


# =========================================================================== #
# vertical controller
# =========================================================================== #
def _vin(phase=HOLD, tgt=5.0, h=5.0, vz=0.0, tilt=0.0, baro_lost=False, c=None, fz=None) -> VerticalInputs:
    return VerticalInputs(phase=phase, h_tgt_m=tgt, baro_lost=baro_lost, h_m=h, vz_mps=vz, tilt_deg=tilt, cos_tilt=c,
                          f_z_mps2=fz)


def _vc(vp: VerticalParams = VP, warm: bool = True, **kw) -> VerticalController:
    """A controller past its take-off ramp (warm) at h = target."""
    c = VerticalController(vp)
    if warm:
        for _ in range(vp.ramp_ticks + 1):
            c.update(_vin(**kw))
    return c


def test_vc_idle_resets_and_outputs_zero():
    c = _vc()
    assert c.active and c.ramp_k == VP.ramp_ticks
    for ph in (NONE, SPOOL, LANDED):
        o = c.update(_vin(phase=ph, h=3.0, vz=-1.0))
        assert (o.mode, o.u, o.touchdown, o.at_target, o.integ) == (MODE_IDLE, 0.0, False, False, 0.0)
        assert not c.active and c.integ == 0.0 and c.ramp_k == 0
    o = c.update(_vin(phase=CLIMB, tgt=5.0, h=0.2))                      # start: h_ref from h, ramp from 0
    assert o.mode == MODE_FLY and c.active and o.h_ref == pytest.approx(0.2 + VP.climb_rate_mps * DT)
    assert o.u == pytest.approx(VP.u_max / VP.ramp_ticks) and o.sat == 1


def test_vc_reference_rate_limits_and_exact_arrival():
    """v0.7.1 integer reference: 5 m at 4 mm per tick arrives on tick 1250 exactly (v0.7.0: 1250 or 1251 by float
    rounding - the knife edge the C port reported); every step is exactly 4000 um."""
    c = VerticalController(VP)
    o = c.update(_vin(phase=CLIMB, tgt=5.0, h=0.0))
    refs = [o.ref_um]
    while not o.at_target:
        o = c.update(_vin(phase=CLIMB, tgt=5.0, h=0.0))
        refs.append(o.ref_um)
    steps = [b - a for a, b in zip([0] + refs, refs)]
    assert steps == [VP.up_um] * 1250 and len(refs) == 1250 and o.ref_um == 5_000_000 and o.h_ref == 5.0
    assert o.dh_ref_mps == VP.climb_rate_mps
    o = c.update(_vin(phase=CLIMB, tgt=5.0, h=0.0))                       # at the target: no step, at_target stays
    assert o.at_target and o.dh_ref_mps == 0.0 and o.ref_um == 5_000_000
    o = c.update(_vin(phase=HOLD, tgt=3.0, h=5.0))                        # DOWN steps descend at descent_rate
    assert o.ref_um == 5_000_000 - VP.dn_um and o.h_ref == pytest.approx(5.0 - VP.descent_rate_mps * DT)
    assert not o.at_target and o.dh_ref_mps == -VP.descent_rate_mps
    n = 1
    while not o.at_target:
        o = c.update(_vin(phase=HOLD, tgt=3.0, h=5.0))
        n += 1
    assert n == math.ceil(2_000_000 / VP.dn_um) == 834 and o.ref_um == 3_000_000     # 833 x 2.4 mm + 0.8 mm
    # LAND: descent rate above the flare, touchdown rate below it (by h_ref or by h), unbounded below the pad
    c = _vc(tgt=5.0, h=5.0)
    o = c.update(_vin(phase=LAND, h=5.0))
    assert o.h_ref == pytest.approx(5.0 - VP.descent_rate_mps * DT)
    c.h_ref = 0.5
    assert c.update(_vin(phase=LAND, h=3.0)).h_ref == pytest.approx(0.5 - VP.touchdown_rate_mps * DT)
    c.h_ref = 3.0
    assert c.update(_vin(phase=LAND, h=0.9)).h_ref == pytest.approx(3.0 - VP.touchdown_rate_mps * DT)
    c.h_ref = -2.0
    assert c.update(_vin(phase=LAND, h=-2.0)).h_ref < -2.0


def test_vc_command_limits_and_step_response_sign():
    c = _vc()
    o = c.update(_vin(h=5.0 - 0.3))                                       # below the reference: climb
    assert o.vz_cmd == pytest.approx(VP.alt_kp * 0.3) and o.u > o.u_ff
    c = _vc()
    o = c.update(_vin(h=5.0 + 0.3))
    assert o.vz_cmd == pytest.approx(-VP.alt_kp * 0.3) and o.u < o.u_ff
    assert _vc().update(_vin(h=-10.0)).vz_cmd == VP.vz_max_up_mps
    assert _vc().update(_vin(h=40.0)).vz_cmd == -VP.vz_max_down_mps
    c = _vc()
    o = c.update(_vin(vz=0.2))                                            # rising at the reference: brake
    assert o.vz_err == pytest.approx(-0.2) and o.u == pytest.approx(o.u_ff - VP.vz_kp * 0.2 + o.integ)


def test_vc_tilt_feed_forward():
    for tilt in (0.0, 10.0, 30.0):
        o = _vc(tilt=tilt).update(_vin(tilt=tilt))
        assert o.u_ff == pytest.approx(VP.u_hover / math.sqrt(math.cos(math.radians(tilt))), rel=1e-12)  # T/cos -> u
    assert _vc().update(_vin(tilt=80.0)).u_ff == pytest.approx(VP.u_hover / math.sqrt(VP.cos_tilt_min))
    assert _vc().update(_vin(tilt=float("nan"))).u_ff == VP.u_hover
    o = _vc().update(_vin(tilt=30.0))                                     # T/W 1.19: 30 deg still holds the weight ...
    assert o.u_ff < VP.u_max and o.sat == 0
    o = _vc().update(_vin(tilt=40.0))                                     # ... 40 deg needs more than u_max: saturated
    assert o.u_ff > VP.u_max and o.u == VP.u_max and o.sat == 1


def test_vc_takeoff_ramp_and_no_windup_on_the_pad():
    c = VerticalController(VP)
    us, ints = [], []
    for k in range(VP.ramp_ticks + 10):                                   # on the pad: h stays 0 while the ramp rises
        o = c.update(_vin(phase=CLIMB, tgt=5.0, h=0.0, vz=0.0))
        us.append(o.u)
        ints.append(o.integ)
    cap = [VP.u_max * min(1.0, (k + 1) / VP.ramp_ticks) for k in range(len(us))]
    assert all(u <= c_ + 1e-15 for u, c_ in zip(us, cap)) and us[0] == pytest.approx(VP.u_max / VP.ramp_ticks)
    assert us[VP.ramp_ticks // 2] == pytest.approx(cap[VP.ramp_ticks // 2])     # the ramp limits the take-off throttle
    assert all(i == 0.0 for i in ints[:VP.ramp_ticks // 2])                  # capped and e > 0: integrator frozen
    assert max(ints) <= VP.vz_i_max


def test_vc_anti_windup_and_integrator_limits():
    c = _vc()
    i0 = c.integ
    for _ in range(2000):                                                  # cannot climb (heavy / saturated): no windup
        o = c.update(_vin(h=0.0, vz=0.0))
    assert o.sat == 1 and o.u == VP.u_max and c.integ == i0
    c = _vc()
    for _ in range(4000):                                                  # cannot descend: P and I bottom out above u_min
        o = c.update(_vin(h=40.0, vz=0.0))                                 # (u_hover - vz_kp vz_max_down - vz_i_max > u_min)
    assert o.integ == -VP.vz_i_max and o.sat == 0
    assert o.u == pytest.approx(VP.u_hover - VP.vz_kp * VP.vz_max_down_mps - VP.vz_i_max) and o.u > VP.u_min
    vp = dataclasses.replace(VP, u_min=0.75)                               # a higher u_min: the lower saturation acts
    c = _vc(vp)
    for _ in range(4000):
        o = c.update(_vin(h=40.0, vz=0.0))
    # conditional integration: the integrator stops at the last step that keeps the output at or above u_min
    assert vp.u_min <= o.u < vp.u_min + vp.vz_ki * 1.0 * DT and -vp.vz_i_max < c.integ < 0
    i1 = c.integ
    c.update(_vin(h=40.0, vz=0.0))
    assert c.integ == i1
    o = c.update(_vin(h=40.0, vz=0.5))                                     # rising too: the P term reaches u_min
    assert o.u == vp.u_min and o.sat == -1 and c.integ == i1
    c = _vc()
    o = c.update(_vin(vz=-0.05))                                           # unsaturated: integrates vz_ki e dt
    assert o.integ == pytest.approx(VP.vz_ki * 0.05 * DT) and o.sat == 0
    vp = dataclasses.replace(VP, u_max=1.0, u_min=0.0, vz_kp=0.0, u_hover=0.5)
    c = _vc(vp)
    for _ in range(20000):
        o = c.update(_vin(vz=-0.5))
    assert o.integ == vp.vz_i_max and o.u == pytest.approx(0.5 + vp.vz_i_max)     # clamped at +i_max


def _lpf_ticks(alpha: float, frac: float) -> int:
    """Ticks k until a first-order lag x += alpha (1 - x) from 0 first exceeds ``frac`` (strictly)."""
    x, k = 0.0, 0
    while not x > frac:
        x += alpha * (1.0 - x)
        k += 1
    return k


def test_vc_touchdown_still_signature():
    """(a) still: |v_lpf| <= td_vz_still with the command at least the touchdown rate, while low; confirmed after
    landed_confirm_ticks; the low-pass means one noisy tick no longer restarts it (v0.7.0's failure under baro noise)."""
    n = VP.landed_confirm_ticks
    lead = VP.still_lead * VP.touchdown_rate_mps / VP.alt_kp              # the reference ran this far below the vehicle
    c = _vc()                                                             # warm at rest: v_lpf = 0
    c.h_ref = 0.5 - lead + VP.td_um * 1e-6                                # after this tick's step: exactly the lead
    outs = [c.update(_vin(phase=LAND, h=0.5, vz=0.0)) for _ in range(n)]
    assert outs[0].vz_cmd == pytest.approx(-(1 + VP.still_lead) * VP.touchdown_rate_mps)
    assert VP.still_lead == vcm.TD_STILL_LEAD_ASSUMED == 2.0
    c2 = _vc()                                                            # a hair less run-away: not still
    c2.h_ref = 0.5 - lead + VP.td_um * 1e-6 + 1e-5
    assert not c2.update(_vin(phase=LAND, h=0.5, vz=0.0)).still
    assert outs[0].dh_ref_mps == -VP.touchdown_rate_mps and outs[0].low and outs[0].still
    assert [o.touchdown for o in outs] == [False] * (n - 1) + [True] and outs[-1].td_count == n
    assert outs[-1].td_by == vcm.TD_STILL and not any(o.support or o.floor for o in outs)
    # the reference level with the vehicle (free flight in the flare, h_err ~ 0): never still, whatever v_z (v0.7.0 meaning)
    c = _vc()
    c.h_ref = 0.5 + VP.td_um * 1e-6
    o = c.update(_vin(phase=LAND, h=0.5, vz=0.0))
    assert o.low and not o.still and o.vz_cmd <= -VP.touchdown_rate_mps and o.td_count == 0
    # one tick of noise (|v_z| 0.3 m/s) moves v_lpf by alpha x 0.3 only: the count goes on
    c = _vc()
    c.h_ref = 0.5 - lead
    for _ in range(n - 2):
        c.update(_vin(phase=LAND, h=0.5, vz=0.0))
    o = c.update(_vin(phase=LAND, h=0.5, vz=-0.3))
    assert o.still and o.td_count == n - 1 and o.vz_lpf == pytest.approx(-0.3 * VP.still_alpha)
    assert c.update(_vin(phase=LAND, h=0.5, vz=0.0)).touchdown
    # moving (descending at the touchdown rate): v_lpf passes td_vz_still after k ticks -> the count restarts there
    c = _vc()
    c.h_ref = 0.5 - lead
    k = _lpf_ticks(VP.still_alpha, VP.td_vz_still_mps / VP.touchdown_rate_mps)
    outs = [c.update(_vin(phase=LAND, h=0.5, vz=-VP.touchdown_rate_mps)) for _ in range(k)]
    assert [o.still for o in outs] == [True] * (k - 1) + [False] and outs[-1].td_count == 0 and not outs[-1].touchdown
    # the reference above the vehicle: no still candidate
    c = _vc()
    c.h_ref = 0.6
    o = c.update(_vin(phase=LAND, h=0.5, vz=0.0))
    assert o.vz_cmd > -VP.touchdown_rate_mps and not o.still and o.td_count == 0
    # high up (ground above the pad / baro offset): low only with the altitude loop saturated downward - and even then
    # the still signature does not count alone (altitude band only: noise can saturate the loop at height, module
    # docstring 5); v0.7.1b: above the band only support, floor and still TOGETHER confirm such a touchdown
    c = _vc()
    c.h_ref = 3.0 - 0.3                                                   # -0.6 + alt_kp (-0.3024) > -vz_max_down
    o = c.update(_vin(phase=LAND, h=3.0, vz=0.0))
    assert o.vz_cmd > -VP.vz_max_down_mps and not o.low and o.td_count == 0
    c.h_ref = 3.0 - 2.0
    outs = [c.update(_vin(phase=LAND, h=3.0, vz=0.0)) for _ in range(3 * n)]
    assert outs[0].vz_cmd == -VP.vz_max_down_mps and outs[0].low and all(o.still for o in outs)
    assert not any(o.touchdown for o in outs) and max(o.td_count for o in outs) == 0 and not outs[-1].low_alt
    outs = [c.update(_vin(phase=LAND, h=3.0, vz=0.0, fz=G0)) for _ in range(20 * n)]    # resting: the ground carries it
    first_sup = next(i for i, o in enumerate(outs) if o.support)
    td = next(i for i, o in enumerate(outs) if o.touchdown)
    both = next(i for i, o in enumerate(outs) if o.support and o.floor and o.still)
    assert td == both + n - 1 > first_sup + n and outs[td].td_by == vcm.TD_ALL      # support alone did not confirm
    # FLY never detects touchdown
    o = _vc().update(_vin(phase=HOLD, tgt=0.0, h=0.0, vz=0.0))
    assert not o.touchdown and o.td_count == 0 and not o.low and o.low_in_land_ticks == 0


def test_vc_touchdown_support_signature_and_ground_signature():
    """(b) support: the body-z specific force exceeds the thrust model's free-flight value T(u) / m (through the EDF lag)
    by more than support_g g, low-passed; free flight with the model's own thrust never triggers it."""
    n = VP.landed_confirm_ticks
    c = _vc()
    c.h_ref = 5.0
    for _ in range(2500):          # free flight (LAND, 10 s) following its reference down: f_z = g r exactly
        o = c.update(_vin(phase=LAND, h=c.h_ref, vz=-VP.descent_rate_mps, fz=G0 * c.r_thr))
        assert not o.support and not o.ground_signature and not o.touchdown and not o.floor
    # the model through the lag: r follows (u_last / u_hover)^2 with alpha_T
    r0, u0 = c.r_thr, c.u_last
    o = c.update(_vin(phase=LAND, h=c.h_ref, vz=-VP.descent_rate_mps, fz=G0 * r0))
    assert o.thrust_ratio == pytest.approx(r0 + VP.thrust_alpha * ((u0 / VP.u_hover) ** 2 - r0), rel=1e-12)
    # moving (not still) in the flare band, the ground carries 0.5 g more than the fan: support after the LPF crosses
    c = _vc()
    c.h_ref = 0.5
    k = _lpf_ticks(VP.support_alpha, VP.support_g / 0.5)
    outs = []
    for _ in range(k + n):
        outs.append(c.update(_vin(phase=LAND, h=0.5, vz=-0.4, fz=G0 * c.r_thr + 0.5 * G0)))
    first = next(i for i, o in enumerate(outs) if o.support)
    assert abs(first - (k - 1)) <= 1                                       # the LPF of a (slowly varying) 0.5 g step
    td = next(i for i, o in enumerate(outs) if o.touchdown)
    assert td - first == n - 1 and outs[td].td_by == vcm.TD_SUPPORT and not outs[td].still
    assert all(o.ground_signature for o in outs[first:]) and not any(o.ground_signature for o in outs[:first])
    # support while NOT low (high up, loop not saturated): ground_signature yes (diagnostics, baro-lost rule), no touchdown
    c = _vc()
    c.h_ref = 3.0 - 0.1                                                   # 50 ticks: v_cmd >= -0.6 - alt_kp 0.22 > -1
    for _ in range(50):
        o = c.update(_vin(phase=LAND, h=3.0, vz=-0.6, fz=2.0 * G0))
    assert o.ground_signature and o.support and not o.low and o.td_count == 0 and not o.touchdown
    # f_z absent (None) or not finite: support never; the filter state holds
    c = _vc()
    c.h_ref = 0.5
    for _ in range(2 * n):
        o = c.update(_vin(phase=LAND, h=0.5, vz=-0.4, fz=None))
    assert not o.support and o.f_excess_mps2 == 0.0
    o = c.update(_vin(phase=LAND, h=0.5, vz=-0.4, fz=float("nan")))
    assert not o.support and o.f_excess_mps2 == 0.0


def test_vc_touchdown_floor_signature():
    """(c) floor: sat == -1 (u at u_min) for floor_ticks, then confirmed like the others.  Needs a u_min the loop can
    reach on the ground; with the params limits it cannot (u_floor_reachable > u_min: VerticalParams test)."""
    n = VP.landed_confirm_ticks
    vp = dataclasses.replace(VP, u_min=0.75)
    c = _vc(vp)
    c.h_ref = 0.5
    outs = [c.update(_vin(phase=LAND, h=0.5, vz=1.5)) for _ in range(vp.floor_ticks + n + 5)]   # rising: P term < u_min
    assert all(o.sat == -1 for o in outs) and sum(o.still for o in outs) < n      # still only while v_lpf builds up
    assert [o.floor for o in outs].index(True) == vp.floor_ticks - 1
    td = [o.touchdown for o in outs].index(True)
    assert td == vp.floor_ticks - 1 + n - 1 and outs[td].td_by == vcm.TD_FLOOR and outs[td].ground_signature
    c = _vc(vp)                                                           # one tick off the floor restarts the run
    c.h_ref = 0.5
    for _ in range(vp.floor_ticks - 1):
        c.update(_vin(phase=LAND, h=0.5, vz=1.5))
    o = c.update(_vin(phase=LAND, h=0.5, vz=-1.0))
    assert o.sat != -1 and not o.floor
    o = c.update(_vin(phase=LAND, h=0.5, vz=1.5))
    assert o.sat == -1 and not o.floor


def test_vc_touchdown_floor_signature_integrator_clause():
    """Architect v0.7.1: floor also when the integrator sits at -vz_i_max with the command at -vz_max_down (the u_min
    clause cannot occur on the ground with the tuned limits).  In the flare band floor alone confirms the touchdown;
    v0.7.1b (gating_v071b): above it (ground above the pad, no f_z: no support) floor alone never does - the HIGH defect
    of the physics review was exactly this signature confirming 6-8 m up with the loop pinned in the air."""
    n = VP.landed_confirm_ticks
    h_band = 0.8 * VP.flare_altitude_m
    c = _vc()
    c.h_ref = h_band
    outs = [c.update(_vin(phase=LAND, h=h_band, vz=0.4, fz=None)) for _ in range(5000)]   # v_z 0.4: never still
    first = next(i for i, o in enumerate(outs) if o.floor)
    pin0 = next(i for i, o in enumerate(outs) if o.integ == -VP.vz_i_max and o.vz_cmd == -VP.vz_max_down_mps)
    assert first == pin0 + VP.floor_ticks - 1 and all(o.sat != -1 for o in outs)   # pinned (the integrator clause)
    td = next(i for i, o in enumerate(outs) if o.touchdown)
    assert td - first == n - 1 and outs[td].td_by == vcm.TD_FLOOR and outs[td].low_alt and outs[td].ground_signature
    assert not any(o.still for o in outs)
    # the same above the band: floor (and still) for 20 s, never a touchdown without the support signature
    c = _vc()
    c.h_ref = 2.0
    outs = [c.update(_vin(phase=LAND, h=2.0, vz=0.0, fz=None)) for _ in range(5000)]
    assert sum(o.floor for o in outs) > 4000 and sum(o.still for o in outs) > 4000 and not outs[-1].low_alt
    assert not any(o.touchdown for o in outs) and max(o.td_count for o in outs) == 0 and outs[-1].ground_signature


def test_vc_low_in_land_counter_and_baro_lost_detector():
    lead_m = VP.still_lead * VP.touchdown_rate_mps / VP.alt_kp            # 0.48 m: the reference below the vehicle
    c = _vc()
    c.h_ref = 0.5 - lead_m - 0.01                                         # low and the reference has run away
    outs = [c.update(_vin(phase=LAND, h=0.5, vz=-0.3)) for _ in range(50)]
    assert [o.low_in_land_ticks for o in outs] == list(range(1, 51))
    c.h_ref = VP.flare_altitude_m + 0.5 - lead_m - 0.01
    o = c.update(_vin(phase=LAND, h=VP.flare_altitude_m + 0.5, vz=-0.3))    # above the flare, not saturated: reset
    assert not o.low and o.low_in_land_ticks == 0
    c.h_ref = VP.flare_altitude_m - lead_m - 0.01
    o = c.update(_vin(phase=LAND, h=VP.flare_altitude_m, vz=-0.3))          # h <= flare: inclusive
    assert o.low and o.low_in_land_ticks == 1
    # architect v0.7.1: low but FOLLOWING the descending reference (a ditch below the pad) - the timer does not run
    c = _vc()
    c.h_ref = 0.5
    for _ in range(2000):                                                 # 8 s, from 0.5 m to -1.9 m, following
        o = c.update(_vin(phase=LAND, h=c.h_ref, vz=-VP.touchdown_rate_mps))
        assert o.low and o.low_in_land_ticks == 0 and not o.floor and not o.touchdown
    # baro lost (v0.7.1): the LAND law on the dead-reckoned h, descent_rate all the way (no flare), the floor timeout's
    # counter does not run; "low" as with the baro (h <= flare or saturated)
    c = _vc()
    c.h_ref = 5.0
    o = c.update(_vin(phase=LAND, h=5.0, vz=-0.6, baro_lost=True))
    assert o.dh_ref_mps == -VP.descent_rate_mps and o.ref_um == 5_000_000 - VP.dn_um and not o.low
    c.h_ref = 0.5 - lead_m - 0.01
    o = c.update(_vin(phase=LAND, h=0.5, vz=-0.6, baro_lost=True))         # below the flare: still descent_rate
    assert o.dh_ref_mps == -VP.descent_rate_mps and o.low and o.low_in_land_ticks == 0
    o = c.update(_vin(phase=LAND, h=0.5, vz=-0.6, baro_lost=False))        # (with the baro: the flare rate)
    assert o.dh_ref_mps == -VP.touchdown_rate_mps and o.low_in_land_ticks == 1
    outs = [c.update(_vin(phase=LAND, h=0.5, vz=-0.4, baro_lost=True, fz=G0 * c.r_thr + 0.6 * G0)) for _ in range(400)]
    td = [o.touchdown for o in outs].index(True)
    assert outs[td].td_by == vcm.TD_SUPPORT and all(o.low_in_land_ticks == 0 for o in outs)


def test_vc_feed_forward_and_cos_tilt_from_quaternion():
    # reference-rate feed-forward: v_cmd = dh_ref/dt + alt_kp (h_ref - h); without it the v0.7.0 law
    c = VerticalController(VP)
    o = c.update(_vin(phase=CLIMB, tgt=5.0, h=0.0))
    assert o.vz_cmd == pytest.approx(VP.climb_rate_mps + VP.alt_kp * VP.up_um * 1e-6) and o.dh_ref_mps == 1.0
    c = VerticalController(dataclasses.replace(VP, vz_ff=False))
    o = c.update(_vin(phase=CLIMB, tgt=5.0, h=0.0))
    assert o.vz_cmd == pytest.approx(VP.alt_kp * VP.up_um * 1e-6)
    # a vehicle on the ramp (one tick behind its reference) is commanded the ramp rate + alt_kp x one step: it does not
    # lag by climb_rate / alt_kp (v0.7.0: the same vehicle got only alt_kp x one step, an error of ~1 m/s)
    for ff, want in ((True, VP.climb_rate_mps + VP.alt_kp * VP.up_um * 1e-6), (False, VP.alt_kp * VP.up_um * 1e-6)):
        c = VerticalController(dataclasses.replace(VP, vz_ff=ff))
        for k in range(400):
            o = c.update(_vin(phase=CLIMB, tgt=5.0, h=k * VP.up_um * 1e-6, vz=1.0))
        assert o.vz_cmd == pytest.approx(want, rel=1e-9) and o.vz_err == pytest.approx(want - 1.0, rel=1e-6)
    # cos(tilt) from the NAV quaternion: 1 - 2 (x^2 + y^2) == cos(tilt); u_ff = u_hover / sqrt(c) (n = 2)
    for tilt, axis in ((0.0, (1, 0, 0)), (10.0, (1, 0, 0)), (30.0, (0.6, 0.8, 0)), (44.0, (0, 1, 0))):
        q = quat.multiply(quat.from_yaw(0.7), quat.from_axis_angle(np.array(axis, float), math.radians(tilt)))
        cq = cos_tilt_from_q(q)
        assert cq == pytest.approx(math.cos(math.radians(tilt)), abs=1e-12)
        o1 = _vc(c=cq).update(_vin(c=cq))
        o2 = _vc(tilt=tilt).update(_vin(tilt=tilt))
        assert o1.u_ff == VP.u_hover / math.sqrt(cq) and o1.u_ff == pytest.approx(o2.u_ff, rel=1e-12)
        assert o1.cos_tilt == cq
    assert _vc().update(_vin(c=0.2)).cos_tilt == VP.cos_tilt_min               # floor at cos(tilt_abort)
    assert _vc().update(_vin(c=1.0000001)).cos_tilt == 1.0                     # a non-unit q cannot lower u_ff
    assert _vc().update(_vin(c=float("nan"))).u_ff == VP.u_hover
    vp3 = dataclasses.replace(VP, throttle_exp=3.0)                             # the general-n fallback
    assert _vc(vp3, c=0.8).update(_vin(c=0.8)).u_ff == pytest.approx(VP.u_hover * 0.8 ** (-1 / 3), rel=1e-12)


def test_vc_reference_is_exact_on_every_grid_start():
    """The knife edge of v0.7.0: every climb from the 0.1 m grid to a 0.1 m-grid target arrives after exactly
    ceil(distance / step) ticks, the same in any language (integers); random starts arrive by the same rule."""
    for h0_dm in range(0, 101, 7):
        for tgt_dm in range(20, 101, 9):
            c = VerticalController(VP)
            o = c.update(_vin(phase=CLIMB, tgt=tgt_dm / 10, h=h0_dm / 10))
            k = 1
            while not o.at_target:
                o = c.update(_vin(phase=CLIMB, tgt=tgt_dm / 10, h=h0_dm / 10))
                k += 1
            d = (tgt_dm - h0_dm) * vcm.UM_PER_DM
            step = VP.up_um if d > 0 else VP.dn_um
            assert k == max(1, math.ceil(abs(d) / step)) and o.ref_um == tgt_dm * vcm.UM_PER_DM, (h0_dm, tgt_dm)
    rng = random.Random(5)
    for _ in range(300):
        h0 = nl_f32(rng.uniform(-1, 12))
        c = VerticalController(VP)
        o = c.update(_vin(phase=CLIMB, tgt=6.0, h=h0))
        k = 1
        while not o.at_target:
            o = c.update(_vin(phase=CLIMB, tgt=6.0, h=h0))
            k += 1
        d = 6_000_000 - vcm.m_to_um(h0)
        assert k == max(1, math.ceil(abs(d) / (VP.up_um if d > 0 else VP.dn_um)))
    assert vcm.m_to_um(float("nan")) == 0 and vcm.m_to_um(1e300) == vcm.REF_LIMIT_UM
    assert vcm.m_to_um(-1e300) == -vcm.REF_LIMIT_UM and vcm.m_to_um(0.0000005) == 1 and vcm.m_to_um(-0.0000005) == -1
    assert vcm.target_um(0.7) == 700_000 and vcm.target_um(25.3) == 25_300_000 and vcm.target_um(float("nan")) == 0


def test_vc_nonfinite_inputs_keep_the_output_bounded():
    for kw in (dict(h=float("nan")), dict(vz=float("nan")), dict(tilt=float("inf")), dict(h=float("inf"), vz=float("-inf"))):
        c = _vc()
        for ph in (HOLD, LAND):
            o = c.update(_vin(phase=ph, **kw))
            assert math.isfinite(o.u) and VP.u_min <= o.u <= VP.u_max and math.isfinite(o.integ) and not o.touchdown, kw
    c = VerticalController(VP)
    o = c.update(_vin(phase=CLIMB, h=float("nan")))                      # start with no altitude: reference from 0
    assert c.h_ref == pytest.approx(VP.climb_rate_mps * DT) and math.isfinite(o.u)


def test_vc_output_bounds_random():
    rng = random.Random(9)
    c = VerticalController(VP)
    for _ in range(20000):
        ph = rng.choice([CLIMB, HOLD, LAND, LAND, NONE])
        o = c.update(_vin(phase=ph, tgt=rng.uniform(2, 20), h=rng.uniform(-3, 30), vz=rng.uniform(-4, 4),
                          tilt=rng.uniform(0, 50), baro_lost=rng.random() < 0.2))
        assert 0.0 <= o.u <= VP.u_max and abs(o.integ) <= VP.vz_i_max
        assert o.u == 0.0 if ph == NONE else (o.u >= min(VP.u_min, VP.u_max * (c.ramp_k / VP.ramp_ticks)) - 1e-12)
        assert VP.kt_min <= o.k_thrust <= VP.kt_max


# ---- v0.7.1b: thrust learning and the touchdown gating ---------------------------------------------------------- #
def _r_next(c: VerticalController) -> float:
    """The thrust-model state r this tick's update will use (module docstring 4)."""
    x = c.u_last * c.vp.inv_u_hover
    return c.r_thr + c.vp.thrust_alpha * (x * x - c.r_thr)


def test_vc_thrust_learning_equations_and_conditions():
    """mission.vtol.thrust_learning (module docstring 4b): k_T += a_K (f_z / (g r) - k_T), clamped to [k_min, k_max],
    only after the take-off ramp, with h > flare_altitude_m, tilt < max_tilt_deg (c > cos), f_z finite and r >= KT_R_MIN;
    frozen otherwise; k_T = 1 at each take-off; k_T scales the feed-forward (next tick) and the support signature's
    expected thrust (this tick)."""
    tl = VTOL["thrust_learning"]
    assert (VP.kt_tau_s, VP.kt_min, VP.kt_max, VP.kt_max_tilt_deg) == (tl["tau_s_ASSUMED"], tl["k_min"], tl["k_max"],
                                                                       tl["max_tilt_deg"])
    assert VP.kt_alpha == pytest.approx(1 - math.exp(-DT / tl["tau_s_ASSUMED"]), rel=1e-15) and VP.kt_reset == 1.0
    assert VP.kt_cos_min == pytest.approx(math.cos(math.radians(tl["max_tilt_deg"])), rel=1e-15)
    assert vcm.KT_R_MIN_ASSUMED == 0.25
    # one step by hand (warm at 5 m: the ramp is over; f_z given from this tick on)
    c = _vc()
    assert c.k_t == 1.0 and c.ramp_k == VP.ramp_ticks
    r1, d0 = _r_next(c), c.d_lpf
    fz = 1.3 * G0 * r1
    o = c.update(_vin(fz=fz))
    k1 = 1.0 + VP.kt_alpha * (fz / (G0 * r1) - 1.0)
    assert o.kt_learn and o.k_thrust == pytest.approx(k1, rel=1e-14) and c.k_t == o.k_thrust
    assert o.f_excess_mps2 == pytest.approx(d0 + VP.support_alpha * ((fz - k1 * (G0 * r1)) - d0), rel=1e-12, abs=1e-15)
    assert o.u_ff == VP.u_hover                                            # this tick's feed-forward: the previous k_T
    o = c.update(_vin(fz=1.3 * G0 * _r_next(c)))
    assert o.u_ff == pytest.approx(VP.u_hover / math.sqrt(k1), rel=1e-14)   # ... then u_hover / sqrt(k_T c)
    # converges like a first-order lag: after N ticks k = 1.3 - 0.3 (1 - a_K)^N (here 3 tau)
    c = _vc()
    n3 = int(round(3 * VP.kt_tau_s * HZ))
    for _ in range(n3):
        o = c.update(_vin(fz=1.3 * G0 * _r_next(c), c=math.cos(math.radians(10.0))))
    assert o.k_thrust == pytest.approx(1.3 - 0.3 * (1 - VP.kt_alpha) ** n3, rel=1e-12)
    assert abs(o.k_thrust - 1.3) < 0.05 * 0.3 + 1e-12                    # 3 tau: within 5 % of the step
    k_prev = c.k_t
    o = c.update(_vin(fz=1.3 * G0 * _r_next(c), c=math.cos(math.radians(10.0))))
    assert o.u_ff == pytest.approx(VP.u_hover / math.sqrt(k_prev * math.cos(math.radians(10.0))), rel=1e-14)
    # clamp
    c = _vc()
    for _ in range(3000):
        o = c.update(_vin(fz=9.0 * G0 * _r_next(c)))
    assert o.k_thrust == VP.kt_max
    for _ in range(6000):
        o = c.update(_vin(fz=0.1 * G0 * _r_next(c)))
    assert o.k_thrust == VP.kt_min
    # frozen: each condition alone stops the learning (the k_T of the previous tick stays bit for bit)
    def frozen(**kw) -> bool:
        cc = _vc()
        cc.k_t = 1.1
        oo = cc.update(_vin(**kw))
        return (not oo.kt_learn) and cc.k_t == 1.1
    assert frozen(h=VP.flare_altitude_m, fz=2 * G0)                       # h <= flare (inclusive): near the ground
    assert not frozen(h=VP.flare_altitude_m + 1e-6, fz=2 * G0)
    ct = VP.kt_cos_min
    assert frozen(c=ct, fz=2 * G0) and not frozen(c=math.nextafter(ct, 2.0), fz=2 * G0)   # tilt >= max_tilt_deg
    assert frozen(c=float("nan"), fz=2 * G0)                              # no attitude: c = 1 for u_ff, no learning
    assert frozen(fz=None) and frozen(fz=float("nan")) and frozen(fz=float("inf"))
    assert frozen(phase=LAND, h=0.5, fz=2 * G0)                           # LAND in the flare band: frozen
    assert not frozen(phase=LAND, h=3.0, fz=2 * G0)                       # LAND above it: free flight, learns
    cc = VerticalController(VP)                                            # the take-off ramp: frozen
    outs = [cc.update(_vin(phase=CLIMB, h=3.0, fz=2 * G0)) for _ in range(VP.ramp_ticks + 1)]
    assert [o.kt_learn for o in outs] == [False] * (VP.ramp_ticks - 1) + [True, True]
    cc = _vc()
    cc.u_last, cc.r_thr = 0.0, 0.2                                         # r < KT_R_MIN: the division guard
    assert not cc.update(_vin(fz=2 * G0)).kt_learn
    v0 = VerticalController(dataclasses.replace(VP, kt_tau_s=0.0))         # tau 0: no learning at all, k_T 1
    for _ in range(VP.ramp_ticks + 50):
        o = v0.update(_vin(fz=2 * G0))
    assert not o.kt_learn and o.k_thrust == 1.0 and o.u_ff == VP.u_hover
    # reset at each take-off (IDLE resets; the IDLE output reports the reset value)
    c = _vc()
    for _ in range(500):
        c.update(_vin(fz=1.3 * G0 * _r_next(c)))
    assert c.k_t > 1.1
    o = c.update(_vin(phase=LANDED))
    assert o.mode == MODE_IDLE and o.k_thrust == 1.0 and c.k_t == 1.0
    vf = dataclasses.replace(VP, kt_min=0.7, kt_max=0.7)                  # limits that exclude 1: reset to the clamp
    assert VerticalController(vf).k_t == 0.7 and VerticalController(vf).update(_vin(phase=NONE)).k_thrust == 0.7
    for bad in (dict(kt_min=0.0), dict(kt_min=1.5, kt_max=1.4), dict(kt_tau_s=-1.0), dict(kt_max_tilt_deg=90.0)):
        with pytest.raises(ValueError):
            VerticalParams.from_params(P, MASS, T_MAX, **bad)
    q = copy.deepcopy(P)
    del q["mission"]["vtol"]["thrust_learning"]                           # a params.json without the block: no learning
    assert VerticalParams.from_params(q, MASS, T_MAX).kt_tau_s == 0.0


def test_vc_gating_v071b_band_and_above():
    """mission.vtol.touchdown.gating_v071b (module docstring 5): in the flare band (h <= flare_altitude_m) any one of
    still / support / floor confirms; above it only support AND floor AND still together (with the baro lost: support
    AND floor); the V10 counter runs only in the band; ground_signature (V8) stays support or floor."""
    n = VP.landed_confirm_ticks
    hi = 3.0 * VP.flare_altitude_m
    vpt = dataclasses.replace(VP, **TUNED_VGAINS)          # the flight gains: pinned by the integrator clause (u > u_min)

    def land(h, vz, fz, steps, baro_lost=False, ref_below=2.0, vp=vpt):
        c = _vc(vp)
        c.h_ref = h - ref_below                                           # the reference far below: loop saturated
        return [c.update(_vin(phase=LAND, h=h, vz=vz, fz=fz, baro_lost=baro_lost)) for _ in range(steps)]

    # above the band, baro healthy: support and floor together but moving (v_z 0.4, not still): no touchdown
    outs = land(hi, 0.4, 2.0 * G0, 6000)
    assert sum(o.support and o.floor for o in outs) > 4000 and not any(o.still for o in outs[500:])  # (v_lpf from 0)
    assert not any(o.touchdown for o in outs) and max(o.td_count for o in outs) == 0
    assert all(o.ground_signature for o in outs[-100:]) and all(o.low and not o.low_alt for o in outs)
    assert all(o.low_in_land_ticks == 0 for o in outs)                    # V10 counts only in the band
    # ... standing still (the ground carries it): all three -> TD_ALL, exactly n ticks after the three first coincide
    outs = land(hi, 0.0, 2.0 * G0, 6000)
    both = next(i for i, o in enumerate(outs) if o.support and o.floor and o.still)
    td = next(i for i, o in enumerate(outs) if o.touchdown)
    assert td == both + n - 1 and outs[td].td_by == vcm.TD_ALL and outs[td].td_count == n
    # ... with the baro lost (dead-reckoned v_z: still is not required)
    outs = land(hi, 0.4, 2.0 * G0, 6000, baro_lost=True)
    td = next(i for i, o in enumerate(outs) if o.touchdown)
    assert outs[td].td_by == vcm.TD_ALL and not outs[td].still and outs[td].support and outs[td].floor
    # in the band each one alone: support (v_z 0.4: not still, loop not pinned within the confirmation)
    lo = 0.5 * VP.flare_altitude_m
    outs = land(lo, 0.4, 2.0 * G0, 400, ref_below=0.0)
    td = next(i for i, o in enumerate(outs) if o.touchdown)
    assert outs[td].td_by == vcm.TD_SUPPORT and not outs[td].floor and outs[td].low_alt
    # the band boundary is inclusive and checked per tick: a support run above the band does not carry into it
    c = _vc()
    c.h_ref = hi - 2.0
    for _ in range(3 * n):
        o = c.update(_vin(phase=LAND, h=hi, vz=0.4, fz=2.0 * G0))
    assert o.support and o.td_count == 0
    outs = [c.update(_vin(phase=LAND, h=VP.flare_altitude_m, vz=0.4, fz=2.0 * G0)) for _ in range(n)]
    assert [o.touchdown for o in outs] == [False] * (n - 1) + [True] and outs[-1].td_by == vcm.TD_SUPPORT


# =========================================================================== #
# FcuStack closed loops with a 1-D point-mass plant (this file's model, not the 6-DOF)
# =========================================================================== #
class Plant1D:
    """Vertical point mass: thrust lags T_max u^n with time constant tau (the exact discrete lag of sim.edf), gravity,
    inelastic ground at z_ground (contact velocity recorded).  No attitude, no drag, no ground effect.  v0.7.1: the
    body-z specific force the accelerometer measures, f_z = (T + N) / m: T / m in the air, g while the ground carries
    the vehicle, g + |v| / dt on the impact tick (the inelastic stop as one tick's reaction)."""

    def __init__(self, mass=MASS, t_max=T_MAX, n=2.0, tau=float(P["edf"]["thrust_time_constant_s"]), z_ground=0.0):
        self.m, self.t_max, self.n, self.tau, self.zg = mass, t_max, n, tau, z_ground
        self.z, self.v, self.T = z_ground, 0.0, 0.0
        self.on_ground = True
        self.impacts: list[float] = []
        self.f_z = G0

    def step(self, u: float, dt: float) -> None:
        self.T += (self.t_max * max(u, 0.0) ** self.n - self.T) * (1.0 - math.exp(-dt / self.tau))
        a = self.T / self.m - G0
        if self.on_ground and a <= 0.0:
            self.v = 0.0
            self.f_z = G0
            return
        self.on_ground = False
        self.f_z = self.T / self.m
        self.v += a * dt
        self.z += self.v * dt
        if self.z <= self.zg:
            self.impacts.append(self.v)
            self.f_z = G0 + abs(self.v) / dt
            self.z, self.v, self.on_ground = self.zg, 0.0, True


Q_NAV = tuple(float(x) for x in quat.from_axis_angle(np.array([1.0, 0.0, 0.0]), math.radians(0.3)))   # 0.3 deg tilt


class _Fly:
    """FcuStack + Plant1D + a scripted ground (frames injected at the FCU, heartbeat every 0.2 s).  v0.7.1: the NAV
    quaternion (0.3 deg tilt) and the plant's body-z specific force (+ ``f_noise`` white noise, the vibration stand-in)
    reach the vertical controller as on the FCU; ``accel=False`` withholds f_z (no support signature)."""

    def __init__(self, sp=SP, vp=VP, noise=0.02, seed=1, plant: Plant1D | None = None, vz_bias=0.0, accel=True,
                 f_noise=0.0):
        self.fcu = FcuStack(sp, vp, seed=seed)
        self.sp, self.vp = sp, vp
        self.pl = plant or Plant1D()
        self.rng = random.Random(seed)
        self.noise, self.vz_bias = noise, vz_bias
        self.accel, self.f_noise = accel, f_noise
        self.h_dr: float | None = None                                     # NAV dead reckoning while the baro is lost
        self.k = 0
        self.seq = 10
        self.link = True
        self.baro_ok = True
        self.h_off = 0.0                                                   # the NAV's pad reference (re-zeroed on the pad)
        self.log: list[tuple] = []

    def tick(self, cmd: int | None = None, **kw) -> onb.FcuOutputs:
        frames = []
        if self.link and (cmd is not None or self.k % 50 == 0):
            if cmd is not None:
                self.seq = (self.seq + 1) & 0xFF
            c = lp.Command(lp.CMD_NONE if cmd is None else cmd, 0, self.fcu.sup.nonce)
            frames = [UplinkFrame(self.seq, c)]
        st = self.fcu.sup.state
        if st in (S_, A_):                                                 # NAV pad mode: h held at 0 at the pad level
            self.h_off = self.pl.z
        h = self.pl.z - self.h_off + self.rng.gauss(0.0, self.noise)
        vz = self.pl.v + self.vz_bias + self.rng.gauss(0.0, self.noise)
        if not self.baro_ok:                                               # the NAV integrates its (biased) v_z
            self.h_dr = h if self.h_dr is None else self.h_dr + (self.pl.v + self.vz_bias) * DT
            h = self.h_dr
        else:
            self.h_dr = None
        fz = self.pl.f_z + self.rng.gauss(0.0, self.f_noise) if self.accel else None
        inp = OnboardInputs(tick=self.k, frames=frames, h_m=h, vz_mps=vz, tilt_deg=0.3, v_cell=4.0,
                            baro_ok=self.baro_ok, q_nav=Q_NAV, accel_z_mps2=fz, **kw)
        o = self.fcu.step(inp)
        self.pl.step(o.throttle, DT)
        self.log.append((self.k, int(o.sup.state), o.sup.phase, o.throttle, self.pl.z, self.pl.v, o.vc.h_ref if o.vc else 0.0,
                         o.vc.td_by if o.vc else 0, o.sup.reason))
        self.k += 1
        return o

    def run(self, seconds: float, until=None) -> onb.FcuOutputs:
        o = None
        for _ in range(int(round(seconds * HZ))):
            o = self.tick()
            if until is not None and until(o):
                break
        return o

    def phase_ticks(self, ph: int) -> list[int]:
        return [r[0] for r in self.log if r[2] == ph]


def test_stack_nominal_flight_takeoff_hold_up_down_land_safe():
    f = _Fly()
    f.run(0.5)
    o = f.tick(lp.CMD_ON)
    assert o.sup.state == A_ and o.sup.buzzer and o.throttle == 0.0
    t_on = f.k - 1
    o = f.run(3.0, until=lambda o: o.sup.state == P_)
    assert f.k - 1 - t_on == SP.spool_ticks and o.throttle > 0.0         # motor from the spool end, ramped
    assert o.throttle <= VP.u_max / VP.ramp_ticks + 1e-12
    o = f.run(12.0, until=lambda o: o.sup.phase == HOLD)
    assert o.sup.phase == HOLD
    f.run(6.0)
    zs = [r[4] for r in f.log[-500:]]
    assert max(abs(z - 5.0) for z in zs) < 0.1                            # holds 5 m (perfect thrust model, 2 cm noise)
    assert max(r[4] for r in f.log) < 5.0 + 0.6                           # climb overshoot bounded
    f.tick(lp.CMD_UP)
    f.tick(lp.CMD_UP)
    f.run(8.0)
    assert abs(f.pl.z - 7.0) < 0.1 and f.fcu.sup.tgt_dm == 70
    f.tick(lp.CMD_DOWN)
    f.run(6.0)
    assert abs(f.pl.z - 6.0) < 0.1
    o = f.tick(lp.CMD_OFF)
    assert o.sup.phase == LAND
    o = f.run(30.0, until=lambda o: o.sup.state == U_)
    assert o.sup.state == U_ and o.throttle == 0.0 and f.pl.on_ground
    k_landed = f.k - 1
    assert f.log[-2][2] == LAND and f.fcu.vc_out.mode == MODE_IDLE           # the flag came from the previous tick
    assert -0.45 < f.pl.impacts[0] < 0.0                                      # touchdown near the touchdown rate
    assert all(abs(v) < 0.02 for v in f.pl.impacts[1:])                    # at most mm hops while the thrust decays
    o = f.run(3.0, until=lambda o: o.sup.state == S_)
    assert o.sup.state == S_ and f.k - 1 - k_landed == SP.safe_after_landed_ticks
    assert max(r[3] for r in f.log) <= VP.u_max and f.fcu.sup.reason == lp.REASON_NONE
    # this flight (~25 s to the OFF plus the descent) used more than max_flight_s - land_reserve_s of motor time: the
    # pack's budget is spent, a second ON needs a power cycle (cumulative motor time, V2 / REJ_ENDURANCE)
    assert f.fcu.sup.powered_ticks > SP.endurance_ticks
    f.run(0.3)
    o = f.tick(lp.CMD_ON)
    assert o.sup.state == S_ and f.fcu.sup.exec_log[-1]["code"] == lp.REJ_ENDURANCE


@pytest.mark.parametrize("z_ground", [-0.11, -0.2, 3.0])
def test_stack_lands_on_ground_below_or_above_the_pad(z_ground):
    f = _Fly()
    f.run(0.3)
    f.tick(lp.CMD_ON)
    f.run(20.0, until=lambda o: o.sup.phase == HOLD)
    f.run(3.0)
    f.pl.zg = z_ground                                                     # the vehicle drifted over a ditch / a roof
    f.tick(lp.CMD_OFF)
    o = f.run(40.0, until=lambda o: o.sup.state == U_)
    assert o.sup.state == U_ and f.pl.on_ground and abs(f.pl.z - z_ground) < 1e-9
    assert -0.7 < f.pl.impacts[-1] < 0.0 and f.fcu.sup.reason == lp.REASON_NONE and f.log[-2][7] != vcm.TD_NONE


def test_stack_ground_half_a_metre_below_the_pad_lands_on_the_detector():
    """v0.7.0 reference: with "low" alone the 6 s V10 timer ran from h = 1 m and ended this landing (ground 0.5 m below
    the pad) by the timeout.  Architect v0.7.1: the timer counts only while the vehicle does not follow the descending
    reference (lead) - it descends to the ground and the touchdown detector confirms it."""
    f = _Fly()
    f.run(0.3)
    f.tick(lp.CMD_ON)
    f.run(20.0, until=lambda o: o.sup.phase == HOLD)
    f.run(3.0)
    f.pl.zg = -0.5
    f.tick(lp.CMD_OFF)
    o = f.run(40.0, until=lambda o: o.sup.state == U_)
    assert o.sup.state == U_ and o.sup.reason == lp.REASON_NONE and f.pl.on_ground and -0.4 < f.pl.impacts[-1] < 0.0
    assert f.log[-2][7] != vcm.TD_NONE


@pytest.mark.parametrize("gains", ["params", "tuned"])
@pytest.mark.parametrize("z_ground", [-1.5, -3.0])
def test_stack_deep_ditch_lands_on_the_detector_not_in_the_air(z_ground, gains):
    """v0.7.0 reference: over ground 1.5 m below the pad the V10 timer ran from h = 1 m and stopped the motor 0.7 m above
    the ground (3.4 m/s drop).  Architect v0.7.1: the timer counts only while the vehicle has stopped following the
    descending reference - it keeps descending at the touchdown rate to the real ground and the detector confirms the
    touchdown; never ABORT / release.  v0.7.1b: also with the flight gains (vtol_gains.json) and the band gating."""
    f = _Fly(vp=VP if gains == "params" else dataclasses.replace(VP, **TUNED_VGAINS))
    f.run(0.3)
    f.tick(lp.CMD_ON)
    f.run(20.0, until=lambda o: o.sup.phase == HOLD)
    f.run(3.0)
    f.pl.zg = z_ground
    f.tick(lp.CMD_OFF)
    o = f.run(60.0, until=lambda o: o.sup.state == U_)
    assert o.sup.state == U_ and o.sup.reason == lp.REASON_NONE and f.pl.on_ground and abs(f.pl.z - z_ground) < 1e-9
    assert -0.7 < f.pl.impacts[0] < 0.0 and f.log[-2][7] != vcm.TD_NONE
    f.run(3.0)
    assert f.fcu.sup.state == S_ and not f.fcu.sup.release_open and all(r[1] != int(AB_) for r in f.log)


def test_stack_touchdown_with_vibration_noise_on_the_accelerometer():
    """The support / still signatures under a 0.3 g white accelerometer noise (vibration stand-in) and 0.1 m / 0.1 m/s
    NAV noise: no false touchdown in 25 s of flight, the touchdown is confirmed on the ground (not by the timeout)."""
    f = _Fly(noise=0.1, f_noise=0.3 * G0, seed=4)
    f.run(0.3)
    f.tick(lp.CMD_ON)
    f.run(20.0, until=lambda o: o.sup.phase == HOLD)
    f.run(10.0)
    f.tick(lp.CMD_OFF)
    o = f.run(40.0, until=lambda o: o.sup.state == U_)
    assert o.sup.state == U_ and f.pl.on_ground and o.sup.reason == lp.REASON_NONE and f.log[-2][7] != vcm.TD_NONE
    assert len(f.pl.impacts) == 1 and -0.6 < f.pl.impacts[0] < 0.0


def test_stack_link_loss_lands_and_kill_aborts():
    f = _Fly()
    f.run(0.3)
    f.tick(lp.CMD_ON)
    f.run(20.0, until=lambda o: o.sup.phase == HOLD)
    f.link = False
    o = f.run(3.0, until=lambda o: o.sup.phase == LAND)
    assert o.sup.reason == lp.R_V_LINK_LAND
    o = f.run(30.0, until=lambda o: o.sup.state == U_)
    assert o.sup.state == U_ and f.pl.on_ground
    f = _Fly()
    f.run(0.3)
    f.tick(lp.CMD_ON)
    f.run(20.0, until=lambda o: o.sup.phase == HOLD)
    o = f.tick(lp.CMD_KILL)
    assert o.sup.state == AB_ and o.throttle == 0.0 and o.vc.mode == MODE_IDLE


@pytest.mark.parametrize("vz_bias, outcome", [(0.0, "landed"), (0.1, "landed"), (0.25, "landed"), (-0.25, "landed"),
                                              (-0.5, "V9"), (-0.6, "V9"), (-0.8, "V9")])
def test_stack_baro_failure_inertial_landing_and_its_time_limit(vz_bias, outcome):
    """Baro lost at 5 m: LAND on the dead-reckoned altitude (here: the NAV integrates its v_z, which carries a constant
    error - the 1-D stand-in for the filter's v_z error at the loss and a residual bias) at descent_rate all the way
    down; v0.7.1 time limit T_lim from the altitude at the loss (5 m: 22.5 s).  The true descent is 0.6 m/s + the error:
    -0.25 m/s still lands in time; -0.5 m/s (0.1 m/s true descent, 50 s from 5 m) and worse are still in the air at
    T_lim: no ground signature -> V9 ABORT, exactly at loss + T_lim (never the V6 hard cutoff any more).  On the ground the reference runs away from the dead reckoning
    and the support signature confirms the touchdown whatever the v_z error."""
    f = _Fly()
    f.run(0.3)
    f.tick(lp.CMD_ON)
    f.run(20.0, until=lambda o: o.sup.phase == HOLD)
    f.vz_bias = vz_bias
    f.baro_ok = False
    o = f.run(1.0, until=lambda o: o.sup.phase == LAND)
    assert o.sup.baro_lost and o.sup.reason == lp.R_I9_BARO
    k0, h0 = f.k - 1, f.h_dr                                               # the NAV altitude the rule saw
    lim = f.fcu.sup.baro_limit
    assert f.fcu.sup.baro_lost_tick == k0 and lim == SP.baro_limit_ticks(h0)
    o = f.run(65.0, until=lambda o: o.sup.state != P_)
    if outcome == "landed":
        assert o.sup.state == U_ and o.sup.phase == LANDED and f.pl.on_ground and f.k - 1 - k0 < lim
        assert o.sup.reason == lp.R_I9_BARO and f.log[-2][7] != vcm.TD_NONE
    else:
        assert o.sup.state == AB_ and o.sup.reason == lp.R_V_BARO_LIMIT_ABORT and f.k - 1 - k0 == lim
        assert not f.pl.on_ground and f.fcu.sup.powered_ticks < SP.hard_cutoff_ticks


def test_stack_cannot_climb_when_too_heavy_saturation_is_reported():
    vp = VerticalParams.from_params(P, mass_kg=T_MAX / G0 * 1.02, t_max_n=T_MAX)
    f = _Fly(vp=vp, plant=Plant1D(mass=T_MAX / G0 * 1.02))
    f.run(0.3)
    f.tick(lp.CMD_ON)
    o = f.run(6.0)
    assert f.pl.on_ground and o.sup.phase == CLIMB and o.vc.sat == 1 and o.throttle == vp.u_max
    assert o.vc.integ <= vp.vz_i_max


# ---- v0.7.1b: plants whose thrust is not the model's (the physics review's HIGH defect) ------------------------- #
THRUST_SCALES = (0.80, 0.90, 1.00, 1.25, 1.35)      # plant T_max / model T_max (proxy table: +23 % on a fresh pack)


def _sp_vp(profile: str, gains: str, **ov) -> tuple[SupervisorParams, VerticalParams]:
    """Supervisor and controller parameters of a profile with params.json's gains_ASSUMED ('params') or the flight
    gains vtol_gains.json ('tuned'); ``ov`` = VerticalParams overrides (e.g. k_T frozen: kt_min = kt_max)."""
    sp = SP if profile == "field" else SupervisorParams.from_params(P, mode=lp.MODE_VTOL, profile=profile, **V071)
    kw = dict(TUNED_VGAINS) if gains == "tuned" else {}
    kw.update(ov)
    return sp, VerticalParams.from_params(P, mass_kg=_mass(gains), t_max_n=T_MAX, profile=profile, **kw)


MASS_V09 = 1.647                                    # v0.9: the battery VTOL vehicle (CAD report) - model T/W 1.876


def _mass(gains: str) -> float:
    """v0.9: the flight gains (sim/out/vtol_gains.json, re-tuned in the v0.9 control round) belong to the VTOL vehicle
    (model T/W 1.876, u_hover 0.73); params.json's gains_ASSUMED keep the v0.7.1 vehicle of these tests (T/W 1.19)."""
    return MASS_V09 if gains == "tuned" else MASS


def _flight(profile: str = "field", gains: str = "tuned", scale: float = 1.0, *, hold_s: float = 20.0,
            early_off: bool = False, z_ground: float | None = None, max_s: float = 150.0, seed: int = 1, **ov) -> dict:
    """ON -> CLIMB -> HOLD for hold_s -> OFF (early_off: OFF 0.5 s after the vehicle passes flare + 0.2 m in the climb) on
    a Plant1D with T_max = scale x the controller's model (ground moved to z_ground at the OFF), until the motor stops
    (UNPOWERED / ABORT) or max_s.  Returns the record: stop (state, reason, height above the ground, signature, tick),
    the first learning tick, k_T per active tick, the hold error per tick of HOLD before the OFF."""
    sp, vp = _sp_vp(profile, gains, **ov)
    f = _Fly(sp, vp, plant=Plant1D(t_max=T_MAX * scale, mass=_mass(gains)), seed=seed)
    f.run(0.3)
    f.tick(lp.CMD_ON)
    tgt = sp.hover_dm / lp.H_SCALE
    learn0 = t_hold = off_at = stop = None
    ks: dict[int, float] = {}
    hold: list[float] = []
    while f.k < max_s * HZ:
        o = f.tick(lp.CMD_OFF if f.k == off_at else None)
        if o.vc is not None and o.vc.mode != MODE_IDLE:
            ks[f.k - 1] = o.vc.k_thrust
            if learn0 is None and o.vc.kt_learn:
                learn0 = f.k - 1
        if off_at is None:
            if early_off:
                if f.pl.z > vp.flare_altitude_m + 0.2:
                    off_at = f.k + HZ // 2
            elif o.sup.phase == HOLD:
                t_hold = f.k if t_hold is None else t_hold
                hold.append(f.pl.z - tgt)
                if f.k - t_hold >= hold_s * HZ:
                    off_at = f.k
        if z_ground is not None and o.sup.phase == LAND:
            f.pl.zg = z_ground
        if o.sup.state in (U_, AB_, R_) and any(r[1] == int(P_) for r in f.log):
            stop = dict(state=o.sup.state, reason=o.sup.reason, h=f.pl.z - f.pl.zg, on_ground=f.pl.on_ground,
                        td_by=f.log[-2][7], tick=f.k - 1)
            break
    return dict(f=f, sp=sp, vp=vp, stop=stop, learn0=learn0, ks=ks, hold=hold, scale=scale)


@pytest.mark.parametrize("scale", THRUST_SCALES)
@pytest.mark.parametrize("gains", ["params", "tuned"])
@pytest.mark.parametrize("profile", ["field", "demo"])
def test_stack_plant_thrust_off_the_model_lands_on_the_ground(profile, gains, scale):
    """Physics review HIGH: field thrust x1.20-1.30 / demo x1.25-1.35 confirmed a touchdown by 'floor' 6-8 m (1.6-2.4 m)
    up with the tuned gains - motor off, 10-12 m/s impact.  v0.7.1b (thrust learning + band gating), the 1-D plant: every
    LAND ends ON THE GROUND by the touchdown detector (height above the ground at the motor stop 0, no V-rule, no
    ABORT), k_T converges within 3 tau of its first learning tick (5 % of the initial error), the hold converges.  x0.80
    (T/W 0.95) cannot leave the pad: CLIMB saturated, nothing learnt (h never above the flare band), the OFF lands it
    on the pad by the detector."""
    r = _flight(profile, gains, scale)
    f, vp, st = r["f"], r["vp"], r["stop"]
    assert st is not None and st["state"] == U_ and st["reason"] == lp.REASON_NONE, st
    assert st["on_ground"] and st["h"] == 0.0 and st["td_by"] != vcm.TD_NONE, st
    assert not any(row[1] == int(AB_) for row in f.log)
    tw = T_MAX / (_mass(gains) * G0) * scale                              # the plant's T / W at u = 1
    if tw < 1.0:
        # v0.9: nothing LEARNT (h never above the flare band); the take-off thrust search lowered k_T to kt_min on the
        # pad (mission.vtol.takeoff_v09) - the v0.8 tests' "k_T stays 1" was the v0.8.1 law
        assert max(row[4] for row in f.log) == 0.0 and r["learn0"] is None and min(r["ks"].values()) == vp.kt_min
        assert any(row[2] == CLIMB and row[3] == vp.u_max for row in f.log)
        return
    assert -0.5 < f.pl.impacts[0] < 0.0 and all(abs(v) < 0.05 for v in f.pl.impacts[1:])
    k3 = r["learn0"] + int(round(3 * vp.kt_tau_s * HZ))
    k_start = r["ks"][r["learn0"] - 1] if r["learn0"] - 1 in r["ks"] else 1.0   # v0.9: k_T after the take-off search
    assert abs(r["ks"][k3] - scale) <= 0.05 * abs(scale - k_start) + 2e-3, (r["ks"][k3], scale, k_start)
    tail = r["hold"][-5 * HZ:]                                              # the last 5 s of a 20 s hold
    assert abs(sum(tail) / len(tail)) < 0.05 and max(abs(x) for x in tail) < 0.1


@pytest.mark.parametrize("scale", [1.25, 1.35])
@pytest.mark.parametrize("gains", ["params", "tuned"])
@pytest.mark.parametrize("profile", ["field", "demo"])
def test_stack_off_during_the_climb_strong_plant_lands_on_the_ground(profile, gains, scale):
    """OFF 0.5 s after the vehicle passes the flare band in the climb (k_T only partly learnt): no motor stop in the
    air.  Measured without learning (k_T fixed 1, the same gating): field x1.35 tuned - LANDED 3.9 m up (all three
    signatures in a pinned near-hover), x1.25 - 0.75 m up (floor).  With it the vehicle lands; the tuned loop, still
    pinned while k_T learns, overshoots and then chases a reference far below it: touchdown up to 1.13 m/s (demo
    x1.35, 1-D) - reported, not hidden by the bound below."""
    r = _flight(profile, gains, scale, early_off=True)
    f, st = r["f"], r["stop"]
    assert st is not None and st["state"] == U_ and st["reason"] == lp.REASON_NONE, st
    assert st["on_ground"] and st["h"] == 0.0 and st["td_by"] != vcm.TD_NONE, st
    assert -1.2 < f.pl.impacts[0] < 0.0


_KNOWN_LIMIT = pytest.mark.xfail(strict=True, reason=(
    "physical limit (vertical_ctrl docstring 5): with gains_ASSUMED (vz_kp 0.15, vz_i_max 0.15) a k_T frozen at 0.7 "
    "on a x1.25 plant (k_T / k_true 0.56) leaves the pinned loop in a near-hover (v_z -0.16 m/s) with support, floor and "
    "still all true - LANDED 5.6 m (field) / 2.1 m (demo) up; the flight gains (vtol_gains.json) climb away instead"))


@pytest.mark.parametrize("profile,gains,k,scale", [
    *[(pr, g, k, s) for pr in ("field", "demo") for g in ("tuned",) for k in (0.7, 1.4) for s in (1.0, 1.25)],
    *[(pr, "params", k, s) for pr in ("field", "demo") for k in (0.7, 1.4) for s in (1.0, 1.25) if (k, s) != (0.7, 1.25)],
    pytest.param("field", "params", 0.7, 1.25, marks=_KNOWN_LIMIT),
    pytest.param("demo", "params", 0.7, 1.25, marks=_KNOWN_LIMIT)])
def test_stack_mislearnt_thrust_ratio_never_stops_the_motor_above_the_flare(profile, gains, k, scale):
    """A k_T mis-learnt to the clamp (frozen: kt_min = kt_max = 0.7 or 1.4) still cannot stop the motor above
    flare_altitude_m: if the motor stops into LANDED, the vehicle is at most flare_altitude_m above the ground.  With
    the flight gains and k_T 0.7 the loop cannot hold the vehicle down (u_ff 20 % high, the loop can take off only
    0.131): it climbs away to the V4 geofence ABORT (release) - reported; with the architect's literal 'support AND
    floor' it was LANDED 12.9 m (field) / 5.3 m (demo) up, which is why the rule above the band also needs still."""
    r = _flight(profile, gains, scale, kt_min=k, kt_max=k)
    st = r["stop"]
    assert st is not None and set(r["ks"].values()) == {k}
    if st["state"] == U_:
        assert st["h"] <= r["vp"].flare_altitude_m, st
    else:
        assert st["state"] == AB_ and st["reason"] in (lp.R_V_GEOFENCE, lp.R_V_HARD_CUTOFF), st


@pytest.mark.parametrize("scale", [1.0, 1.25])
@pytest.mark.parametrize("gains", ["params", "tuned"])
@pytest.mark.parametrize("profile", ["field", "demo"])
def test_stack_ground_above_the_pad_lands_by_all_three_signatures(profile, gains, scale):
    """Ground above the flare band (a roof 3 m up in the field, 1 m in the demo): no single signature may confirm
    there (gating_v071b); support, floor and still together do - the learning runs on (h > flare) and drives k_T to
    kt_max, which lowers the thrust on the roof and brings the support signature back (module docstring 4b)."""
    zg = 3.0 if profile == "field" else 1.0
    r = _flight(profile, gains, scale, z_ground=zg, hold_s=8.0)
    f, st = r["f"], r["stop"]
    assert st is not None and st["state"] == U_ and st["reason"] == lp.REASON_NONE and st["on_ground"], st
    assert st["td_by"] == vcm.TD_ALL and abs(f.pl.z - zg) < 1e-9
    k_contact = next(row[0] for row in f.log if row[4] <= zg + 1e-9 and row[2] == LAND)
    t_confirm = (st["tick"] - k_contact) / HZ
    assert t_confirm < 15.0, t_confirm                                      # measured 5-9 s (1-D)
    assert -0.7 < f.pl.impacts[0] < 0.0 and r["ks"][st["tick"] - 1] == r["vp"].kt_max


def test_stack_ballistic_passes_the_supervisor_throttle():
    sp = SupervisorParams.from_params(P, mode=lp.MODE_BALLISTIC)
    st = FcuStack(sp)
    assert st.vc is None
    o = st.step(OnboardInputs(tick=0))
    assert o.vc is None and o.throttle == o.sup.throttle == 0.0
    with pytest.raises(ValueError):
        FcuStack(SP)                                                       # VTOL needs the controller's parameters


# =========================================================================== #
# ground station logic (VTOL)
# =========================================================================== #
def _gp(**kw) -> GroundParams:
    d = dict(vehicle_id=90, poll_hz=4.0, reply_timeout_s=0.5, t_link_default_s=0.4, target_altitude_m=140.0,
             coast_drag_factor=0.85, apogee_detect_vz_mps=0.0, apogee_confirm_s=0.3, coast_timeout_s=6.0,
             release_after_cut_s=0.2, release_compensated=True, mode=lp.MODE_VTOL)
    d.update(kw)
    return GroundParams(**d)


def _t2(seq: int, ack: int, t_ms: int, state=lp.STATE_SAFE, phase=NONE, tgt=0.0, nonce=0x2222, flags=0, reason=0) -> bytes:
    t = lp.Telemetry.from_physical(state=state, flags=flags, reason=reason, ack_seq=ack, nonce=nonce, t_ms=t_ms, h_m=0.0,
                                   vz_mps=0.0, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0, tilt_deg=0.0, throttle=0.0,
                                   vcell_v=4.0, loop_max_us=0.0, phase=phase, h_tgt_m=tgt)
    return lp.encode_telemetry(90, seq, t)


def _up(frame: bytes) -> tuple[int, lp.Command]:
    (fr,) = lp.FrameDecoder(90, (lp.TYPE_CMD,)).feed(frame)
    return fr.seq, lp.decode_command(fr)


def test_ground_vtol_mode_commands_assist_and_params():
    assert GroundParams.from_params(P, 0.5, 0.4).mode == lp.MODE_VTOL
    assert GroundParams.from_params(P, 0.5, 0.4, mode=lp.MODE_BALLISTIC).mode == lp.MODE_BALLISTIC
    gs = GroundAssist(_gp(), assist=True)
    assert not gs.assist                                                    # forced off in VTOL
    gs.set_assist(True, 1.0)
    assert not gs.assist and "not available" in gs.events[-1][1]
    for f in (gs.arm, gs.launch, gs.cut, gs.release, gs.disarm):
        with pytest.raises(ValueError):
            f(0.0)
    with pytest.raises(ValueError):
        gs.throttle(0.5, 0.0)
    gb = GroundAssist(_gp(mode=lp.MODE_BALLISTIC))
    for f in (gb.on, gb.off, gb.up, gb.down):
        with pytest.raises(ValueError):
            f(0.0)
    assert gb.assist and gb.kill(0.0).name == "KILL"


def test_ground_vtol_on_nonce_off_preempts_fifo_up_down():
    gs = GroundAssist(_gp())
    gs.poll(0.0)
    gs.receive(_t2(0, ack=7, t_ms=10, nonce=0xABCD), 0.3)
    gs.on(0.3)
    seq, c = _up(gs.poll(0.3))
    assert (seq, c.cmd, c.nonce) == (8, lp.CMD_ON, 0xABCD)                 # ON carries the latest telemetry nonce
    gs.receive(_t2(8, ack=8, t_ms=500, state=lp.STATE_ARMED, phase=SPOOL, tgt=5.0, nonce=0x1111), 0.6)
    assert gs.records[0].result == "accepted"
    gs.up(1.0)
    gs.up(1.0)
    gs.down(1.0)
    assert [r.name for r in [gs.pending] + list(gs.queue)] == ["UP", "UP", "DOWN"]
    seq, c = _up(gs.poll(1.0))
    assert (seq, c.cmd, c.nonce) == (9, lp.CMD_UP, 0)                       # UP carries no nonce
    gs.off(1.1)                                                              # pre-empts the pending UP and the queue
    assert [r.result for r in gs.records[1:4]] == ["superseded"] * 3 and gs.pending.name == "OFF"
    gs.receive(_t2(9, ack=9, t_ms=900, state=lp.STATE_POWERED, phase=CLIMB, tgt=6.0), 1.3)   # ack of the UP's SEQ
    assert gs.pending.name == "OFF"
    seq, c = _up(gs.poll(1.3))
    assert (seq, c.cmd) == (10, lp.CMD_OFF)
    gs.receive(_t2(10, ack=10, t_ms=1300, state=lp.STATE_POWERED, phase=LAND, tgt=0.0), 1.6)
    assert gs.records[-1].result == "accepted" and gs.pending is None
    gs.kill(2.0)
    seq, c = _up(gs.poll(2.0))
    assert c.cmd == lp.CMD_KILL


def test_ground_vtol_phase_target_tracking_and_rejections():
    gs = GroundAssist(_gp())
    gs.poll(0.0)
    gs.receive(_t2(0, ack=3, t_ms=10), 0.2)
    gs.receive(_t2(0, ack=3, t_ms=600, state=lp.STATE_ARMED, phase=SPOOL, tgt=5.0), 0.7)
    gs.receive(_t2(0, ack=3, t_ms=3000, state=lp.STATE_POWERED, phase=CLIMB, tgt=5.0), 3.1)
    gs.receive(_t2(0, ack=3, t_ms=9000, state=lp.STATE_POWERED, phase=HOLD, tgt=5.0), 9.1)
    gs.receive(_t2(0, ack=3, t_ms=9500, state=lp.STATE_POWERED, phase=HOLD, tgt=6.0), 9.6)
    ev = [e for _, e in gs.events]
    assert [e for e in ev if e.startswith("phase")] == ["phase - -> NONE (state SAFE)", "phase NONE -> SPOOL (state ARMED)",
                                                         "phase SPOOL -> CLIMB (state POWERED)",
                                                         "phase CLIMB -> HOLD (state POWERED)"]
    assert [e for e in ev if e.startswith("target")] == ["target - -> 0.0 m", "target 0.0 -> 5.0 m", "target 5.0 -> 6.0 m"]
    st = gs.vtol_status()
    assert (st["state"], st["phase"], st["h_tgt_m"]) == ("POWERED", "HOLD", 6.0)
    gs.up(10.0)
    seq, c = _up(gs.poll(10.0))
    gs.receive(_t2(seq, ack=seq, t_ms=10500, state=lp.STATE_POWERED, phase=HOLD, tgt=6.0, flags=lp.FLAG_REJECTED,
                   reason=lp.REJ_ALT_LIMIT), 10.3)
    assert gs.records[-1].result == "rejected: altitude limit"
    assert GroundAssist(_gp()).vtol_status()["state"] is None


def test_ground_link_and_stack_closed_loop_vtol():
    """Ground station + link model (lora_uart, no loss) + FcuStack + Plant1D: ON, UP, DOWN, OFF all accepted once,
    the vehicle lands and is SAFE again; TELEM2 frames on the air."""
    cfg = lm.LinkConfig.from_params(P, loss=0.0)
    rt = lm.round_trip(cfg)
    gs = GroundAssist(GroundParams.from_params(P, rt["reply_timeout_s"], rt["rtt_mean_s"]))
    f = _Fly(noise=0.0)
    link = lm.LinkModel(cfg, seed=2)
    dec = lp.FrameDecoder(90, (lp.TYPE_CMD,))
    plan = {1.0: "on", 12.0: "up", 18.0: "down", 24.0: "off"}
    done, types = set(), set()
    for k in range(int(45.0 * HZ)):
        t = k * DT
        for tr in link.deliver("down", t):
            types.add(tr.data[4])
            gs.receive(tr.data, tr.t_arrival)
        for tc, nm in plan.items():
            if t >= tc and nm not in done:
                done.add(nm)
                getattr(gs, nm)(t)
        up = gs.poll(t)
        if up:
            link.send("up", up, t)
        rx = b"".join(tr.data for tr in link.deliver("up", t))
        frames = [UplinkFrame.from_frame(fr) for fr in dec.feed(rx)] if rx else []
        st = f.fcu.sup.state
        if st in (S_, A_):
            f.h_off = f.pl.z
        o = f.fcu.step(OnboardInputs(tick=k, frames=frames, h_m=f.pl.z - f.h_off, vz_mps=f.pl.v, tilt_deg=0.3, v_cell=4.0))
        f.pl.step(o.throttle, DT)
        if o.sup.reply_seq is not None:
            link.send("down", lp.encode_telemetry(90, o.sup.reply_seq, o.sup.telemetry), t + lm.FC_REPLY_DELAY_S)
    assert [r.name for r in gs.records] == ["ON", "UP", "DOWN", "OFF"] and all(r.result == "accepted" for r in gs.records)
    assert [e["code"] for e in f.fcu.sup.exec_log] == [0, 0, 0, 0] and types == {lp.TYPE_TELEM2}
    assert f.fcu.sup.state == S_ and f.pl.on_ground and gs.latest.phase == NONE
    ev = [e for _, e in gs.events]
    assert {"phase SPOOL -> CLIMB (state POWERED)", "phase CLIMB -> HOLD (state POWERED)", "phase HOLD -> LAND (state POWERED)",
            "phase LAND -> LANDED (state UNPOWERED)"} <= set(ev)


# =========================================================================== #
# the ballistic build: bit-identical to v0.6.3
# =========================================================================== #
def ballistic_digest(onb_mod, lp_mod, p: dict, n_seq: int = 40, seed: int = 20261020) -> tuple[str, dict]:
    """Deterministic random command / sensor sequences (every command code 0..12 and junk codes, repeated / old SEQs,
    wrong nonces, shortened timers, faults, I10 boots) through the BALLISTIC supervisor of module ``onb_mod``; SHA-256
    over every tick's outputs (state, target, throttle, control / vanes / release, reason, flags, ack, nonce, reply SEQ,
    TELEM bytes, event text).  Module-agnostic so the same function runs the v0.6.3 supervisor (the pinned digest)."""
    q = copy.deepcopy(p)
    q["mission"].pop("mode", None)                                     # no mode key: both versions build ballistic
    base = onb_mod.SupervisorParams.from_params(q, fc_config_path=None)
    rng = random.Random(seed)
    hsh = hashlib.sha256()
    n = {"ticks": 0, "replies": 0, "transitions": 0, "rejections": 0, "vtol_codes": 0}
    W = {0: [2, 20, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1], 1: [3, 1, 2, 15, 4, 1, 1, 1, 1, 1, 1, 1, 1],   # per state, codes 0..12
         2: [10, 1, 1, 1, 3, 3, 2, 1, 1, 1, 1, 1, 1], 3: [10, 1, 1, 1, 3, 1, 3, 1, 1, 1, 1, 1, 1],
         4: [8, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1], 5: [8, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1]}
    for _ in range(n_seq):
        ov = dict(ramp_ticks=rng.choice([0, 3, 25]), coast_hold_ticks=rng.choice([0, 5]), fall_confirm_ticks=rng.choice([1, 8]),
                  link_disarm_ticks=rng.choice([30, 80]), max_motor_ticks=rng.choice([200, 600]),
                  release_after_cut_ticks=rng.choice([0, 6]), abort_release_ticks=rng.choice([0, 9]),
                  rate_persist_ticks=rng.choice([1, 4]), imu_fail_ticks=rng.choice([1, 5]), baro_fail_ticks=rng.choice([2, 12]),
                  apogee_limit_m=rng.choice([60.0, 150.0]), geofence_m=rng.choice([80.0, 160.0]))
        ov["link_abort_ticks"] = ov["link_disarm_ticks"] + rng.choice([1, 50])
        sp = dataclasses.replace(base, **ov)
        sup = onb_mod.OnboardSupervisor(sp, seed=rng.randrange(65536), reset_in_flight=rng.random() < 0.1)
        seq, h, vz, fault = rng.randrange(256), 0.0, 0.0, 0
        for t in range(rng.choice([800, 1500])):
            frames = []
            if rng.random() < 0.1:
                r = rng.random()
                s = (seq + 1) & 0xFF if r < 0.7 else (seq if r < 0.85 else (seq - rng.randrange(1, 140)) & 0xFF)
                if r < 0.7:
                    seq = s
                c = rng.choices(range(13), W[int(sup.state)])[0] if rng.random() < 0.97 else rng.choice([0x7F, 0xFF, 13])
                n["vtol_codes"] += int(8 <= c <= 11)
                arg = rng.choice([0, 1, 500, 1000, 4000])
                nonce = sup.nonce if rng.random() < 0.8 else rng.randrange(65536)
                frames.append(onb_mod.UplinkFrame(s, lp_mod.Command(c, arg, nonce)))
            if fault == 0 and rng.random() < 0.004:
                fault = rng.randrange(1, 7) * 1000 + rng.randrange(1, 40)
            kind = fault // 1000
            powered = int(sup.state) == 2
            vz = vz + (14.0 if powered else -9.8) * 0.004 + rng.gauss(0, 0.05)
            h = h + vz * 0.004
            if h < 0.0:
                h, vz = 0.0, 0.0
            out = sup.update(onb_mod.OnboardInputs(
                tick=t, frames=frames, h_m=h, vz_mps=vz, tilt_deg=60.0 if kind == 1 else rng.uniform(0, 4),
                gyro_rad_s=(4.0 if kind == 2 else rng.gauss(0, 0.3), rng.gauss(0, 0.3), rng.gauss(0, 0.5)),
                gyro_saturated=kind == 3, v_cell=3.0 if kind == 4 else rng.uniform(3.6, 4.2), imu_ok=kind != 5,
                baro_ok=kind != 6, aligned=t > 100, arm_switch=not (kind == 3 and fault % 7 == 0),
                roll_deg=rng.uniform(-5, 5), pitch_deg=rng.uniform(-5, 5), yaw_deg=rng.uniform(-180, 180),
                loop_max_us=rng.choice([0.0, 2875.0, 70000.5])))
            if fault:
                fault = fault - 1 if fault % 1000 > 1 else 0
            hsh.update(repr((int(out.state), sup.target_milli, out.throttle_milli, out.control_active, out.vanes_centre,
                             out.release, out.reason, out.flags, out.ack_seq, out.nonce, out.reply_seq,
                             out.telemetry.pack().hex() if out.telemetry is not None else None,
                             tuple(out.events))).encode())
            n["ticks"] += 1
            n["replies"] += int(out.telemetry is not None)
            n["transitions"] += sum(1 for e in out.events if " -> " in e and not e.startswith("REJECT"))
            n["rejections"] += sum(1 for e in out.events if e.startswith("REJECT"))
    return hsh.hexdigest(), n


# computed 2026-10-20 with the v0.6.3 snapshot's sim/onboard.py + sim/link_proto.py (renamed package, same function)
BALLISTIC_DIGEST_V063 = "1e1613bb7db2c867519c6686a7013e836ccc8538a6ca92b736c7e6b06360522f"


def test_ballistic_build_is_bit_identical_to_v063():
    d, n = ballistic_digest(onb, lp, P)
    assert n == {"ticks": 49500, "replies": 4989, "transitions": 170, "rejections": 2108, "vtol_codes": 878}
    assert d == BALLISTIC_DIGEST_V063, n


def test_ballistic_build_answers_vtol_codes_as_unknown_in_every_state():
    """v0.6.3 knew no codes 8..11: REJ_UNKNOWN_CMD, before the KILL / release-pending checks (bit-identical)."""
    sp = SupervisorParams.from_params(P, mode=lp.MODE_BALLISTIC)
    for start in ("SAFE", "ARMED", "POWERED", "ABORT"):
        for code in lp.VTOL_ONLY_CMDS:
            h = _V(sp)
            h.hb()
            h.tick()
            if start != "SAFE":
                h.send(lp.CMD_ARM)
            if start in ("POWERED", "ABORT"):
                h.send(lp.CMD_LAUNCH)
            if start == "ABORT":
                h.send(lp.CMD_KILL)
            st0, n0 = h.sup.state, h.sup.nonce
            assert st0.name == start
            out = h.send(code)
            assert h.code == lp.REJ_UNKNOWN_CMD and out.state == st0 and h.sup.nonce == n0 and not out.telemetry.is_v2


# =========================================================================== #
# v0.7.1 supervisor rules: LAND floor timeout (V10), baro-lost time limit (V8 / V9)
# =========================================================================== #
SP_DEMO = SupervisorParams.from_params(P, mode=lp.MODE_VTOL, profile="demo", **V071)


@pytest.mark.parametrize("sp", [SP, SP_DEMO], ids=["field", "demo"])
def test_land_floor_timeout_v10_exact_tick(sp):
    n = sp.land_timeout_ticks
    assert n == ticks_ge(VTOL["touchdown"]["land_floor_timeout_s_ASSUMED"], HZ)
    h = _to("LAND", sp)
    out = h.tick(vc_low_in_land_ticks=n - 1)
    assert (out.state, out.phase) == (P_, LAND) and out.reason == lp.REASON_NONE
    out = h.tick(vc_low_in_land_ticks=n)
    assert (out.state, out.phase) == (U_, LANDED) and out.reason == lp.R_V_LAND_TIMEOUT and out.flags & lp.FLAG_INTERLOCK
    assert not out.motor_on and h.sup.release_deadline is None and any(e.startswith("POWERED -> UNPOWERED: V10")
                                                                        for e in out.events)
    t = h.k - 1
    h.run(sp.safe_after_landed_ticks)
    assert h.sup.state == S_ and h.k - 1 - t == sp.safe_after_landed_ticks and not h.sup.release_open
    # a touchdown confirmed in the same previous tick wins (LANDED by touchdown, no V10 reason)
    h = _to("LAND", sp)
    out = h.tick(vc_low_in_land_ticks=n, vc_touchdown=True)
    assert out.state == U_ and out.reason == lp.REASON_NONE
    # not with the baro lost (the time limit bounds that case), not in CLIMB / HOLD, off when the timeout is 0
    h = _to("LAND", sp)
    h.run(sp.baro_fail_ticks, baro_ok=False)
    assert h.sup.baro_lost
    assert h.tick(vc_low_in_land_ticks=10 * n, baro_ok=False).state == P_
    for label in ("CLIMB", "HOLD"):
        h = _to(label, sp)
        assert h.tick(vc_low_in_land_ticks=10 * n).phase != LANDED
    h = _to("LAND", dataclasses.replace(sp, land_timeout_ticks=0))
    assert h.tick(vc_low_in_land_ticks=10 ** 6).state == P_


@pytest.mark.parametrize("ground", [True, False])
@pytest.mark.parametrize("sp, h_loss, want", [(SP, 4.97, 2500 + 3125), (SP_DEMO, 1.47, ticks_ge(1.5 * 0.5 / 0.3 + 5, HZ) + 1407)],
                         ids=["field", "demo"])
def test_baro_lost_time_limit_v8_v9_exact_tick(ground, sp, h_loss, want):
    """field: T_lim = 1.5 (4.97 / 0.6 + 1 / 0.3) + 5 s -> 2500 + ceil(50 x 62.5) = 5625 ticks (22.5 s);
    demo: 1.5 (1.47 / 0.4 + 0.5 / 0.3) + 5 s -> 1875 + ceil(15 x 93.75) = 3282 ticks."""
    h = _to("HOLD", sp)
    h.run(sp.baro_fail_ticks - 1, baro_ok=False, h_m=h_loss)
    out = h.hb(baro_ok=False, h_m=h_loss)                                  # the I9 baro rule acts: LAND + the limit
    assert out.phase == LAND and out.baro_lost and out.reason == lp.R_I9_BARO
    t0 = h.k - 1
    assert h.sup.baro_lost_tick == t0 and h.sup.baro_limit == sp.baro_limit_ticks(h_loss) == want
    assert any(f"time limit {want} ticks" in e for e in out.events)
    h.run(h.sup.baro_limit - 1, baro_ok=False, vc_ground_signature=ground)
    assert h.sup.state == P_ and h.sup.phase == LAND and h.k - 1 - t0 == h.sup.baro_limit - 1
    out = h.tick(baro_ok=False, vc_ground_signature=ground)
    assert h.k - 1 - t0 == h.sup.baro_limit
    if ground:
        assert (out.state, out.phase, out.reason) == (U_, LANDED, lp.R_V_BARO_LIMIT_LANDED) and not out.motor_on
        assert h.sup.release_deadline is None
        h.run(3 * sp.abort_release_ticks, baro_ok=False)
        assert not h.sup.release_open
    else:
        _abort_then_release(h, lp.R_V_BARO_LIMIT_ABORT)


def test_baro_lost_time_limit_altitude_rules():
    """h_dm clamped to [0, geofence_dm]; a non-finite h takes the longest limit; the touchdown confirmed before the
    limit lands normally; a loss during LAND starts the limit from that tick."""
    sp = SP
    assert sp.baro_limit_ticks(-2.0) == sp.baro_limit_ticks(0.0) == sp.baro_limit_base_ticks == 2500
    assert sp.baro_limit_ticks(0.04) == 2500 and sp.baro_limit_ticks(0.05) == 2500 + 63      # 1 dm: ceil(62.5)
    assert sp.baro_limit_ticks(10.0) == 2500 + 6250 == ticks_ge(1.5 * (10 / 0.6 + 1 / 0.3) + 5, HZ)
    for x in (float("nan"), float("inf"), 1e9):
        assert sp.baro_limit_ticks(x) == 2500 + 350 * 62500 // 1000
    h = _to("LAND")
    h.run(30)
    h.run(SP.baro_fail_ticks, baro_ok=False, h_m=0.8)
    t0, lim = h.sup.baro_lost_tick, h.sup.baro_limit
    assert lim == sp.baro_limit_ticks(0.8) and h.sup.phase == LAND
    h.run(t0 + lim - h.k, baro_ok=False)
    out = h.tick(baro_ok=False, vc_touchdown=True)                        # touchdown at the limit tick: LANDED, no V8 / V9
    assert out.state == U_ and out.reason == lp.R_I9_BARO
    h = _to("HOLD")                                                        # a new ON clears the latch and the limit
    h.run(SP.baro_fail_ticks, baro_ok=False)
    h.tick(vc_touchdown=True)
    h.run(SP.safe_after_landed_ticks)
    h.hb()
    h.send(lp.CMD_ON)
    assert h.code == 0 and not h.sup.baro_lost


# =========================================================================== #
# v0.7.1 FcuStack order: the attitude controller uses the previous tick's supervisor output and ESC throttle
# =========================================================================== #
def test_fcu_stack_v071_order_attitude_inputs_one_tick_late():
    f = _Fly()
    f.run(0.3)
    f.tick(lp.CMD_ON)
    outs = []
    while True:
        outs.append(f.tick())
        if outs[-1].sup.state == P_:
            break
    o = outs[-1]                                                           # the first POWERED tick: ESC on, vanes not yet
    assert o.throttle > 0.0 and o.sup.control_active and not o.att_active and o.att_u == 0.0
    for _ in range(2000):
        outs.append(f.tick())
    outs.append(f.tick(lp.CMD_UP))
    for _ in range(300):
        outs.append(f.tick())
    outs.append(f.tick(lp.CMD_KILL))
    k = outs[-1]
    assert k.sup.state == AB_ and k.throttle == 0.0 and k.att_active and k.att_u > 0.5   # ABORT: ESC now, vanes next tick
    outs.append(f.tick())
    assert not outs[-1].att_active and outs[-1].att_u == 0.0
    for a, b in zip(outs, outs[1:]):
        assert (b.att_active, b.att_u) == (a.sup.control_active, a.throttle)
    # cos(tilt) of the vertical controller comes from the NAV quaternion
    vo = outs[1000].vc
    assert vo.cos_tilt == cos_tilt_from_q(Q_NAV) and vo.u_ff == VP.u_hover / math.sqrt(vo.cos_tilt)
    # ballistic: the v0.6.3 order, this tick's values
    st = FcuStack(SupervisorParams.from_params(P, mode=lp.MODE_BALLISTIC))
    o = st.step(OnboardInputs(tick=0))
    assert (o.att_active, o.att_u) == (o.sup.control_active, o.sup.throttle) and o.vc is None


def test_stack_demo_profile_hover_up_down_land():
    """The demo build (user 2026-10-20: indoor, ~1-2 m hover): hover 1.5 m, steps 0.5 m in [1.0, 3.0], climb 0.5 m/s,
    the same rules.  1-D plant (not the 6-DOF: the push-disturbance runs are the sim agent's)."""
    sp = SupervisorParams.from_params(P, profile="demo", **V071)
    # fix round: the exact-tick law below is the reference law without the fix-round take-off / approach / touchdown
    # options (those have their own tests, test_v09fix_*)
    vp = dataclasses.replace(VerticalParams.from_params(P, MASS, T_MAX, profile="demo"), to_learn_tau_s=0.0,
                             to_ref_hold=False, td_release_per_s=0.0, ref_approach_shift=0, land_ref_rebase=False)
    f = _Fly(sp, vp, noise=0.0)
    f.run(0.3)
    f.tick(lp.CMD_ON)
    o = f.run(3.0, until=lambda o: o.sup.state == P_)
    k0 = f.k - 1
    o = f.run(10.0, until=lambda o: o.sup.phase == HOLD)
    assert f.k - 1 - k0 == 750                       # 1.5 m at 2 mm / tick from h = 0: at_target on the 750th update (tick
                                                     # k0 + 749), HOLD the tick after - exact (integer reference)
    f.run(5.0)
    assert abs(f.pl.z - 1.5) < 0.05 and max(r[4] for r in f.log) < 1.5 + 0.3
    f.tick(lp.CMD_UP)
    f.run(4.0)
    assert abs(f.pl.z - 2.0) < 0.05 and f.fcu.sup.tgt_dm == 20
    for _ in range(2):
        f.tick(lp.CMD_DOWN)
        f.run(0.2)
    assert f.fcu.sup.tgt_dm == 10
    f.tick(lp.CMD_DOWN)
    assert f.fcu.sup.exec_log[-1]["code"] == lp.REJ_ALT_LIMIT                # alt_min_m 1.0
    f.run(4.0)
    assert abs(f.pl.z - 1.0) < 0.05
    f.tick(lp.CMD_OFF)
    o = f.run(20.0, until=lambda o: o.sup.state == U_)
    assert o.sup.state == U_ and f.pl.on_ground and -0.45 < f.pl.impacts[0] < 0.0
    assert o.sup.reason == lp.REJ_ALT_LIMIT and not o.sup.flags & lp.FLAG_INTERLOCK    # the last rejection, no rule acted


# =========================================================================== #
# v0.7.1 VTOL NAV estimator (GatedMahonyEstimator) - synthetic streams, not the 6-DOF
# =========================================================================== #
NE = VTOL["nav_estimator"]
X_AXIS = np.array([1.0, 0.0, 0.0])
G_UP = np.array([0.0, 0.0, G0])


def _gm(**kw) -> GatedMahonyEstimator:
    return GatedMahonyEstimator.from_params(P, DT, 2.0, **kw)


def _err_deg(q_true, q_est) -> float:
    return math.degrees(up_error_rad(q_true, q_est))


def test_nav_estimator_params_and_pad_branch_is_v063():
    e = _gm()
    assert (e.kp, e.ki, e.accel_gate) == (NE["kp_ASSUMED"], NE["ki_ASSUMED"], NE["accel_gate"])
    assert e.alpha_a == pytest.approx(1 - math.exp(-2 * math.pi * NE["accel_lpf_hz_ASSUMED"] * DT), rel=1e-15)
    assert e.bias_limit == math.radians(NE["bias_limit_dps"]) and e.n_avg == 500
    q = copy.deepcopy(P)
    q["mission"]["vtol"]["nav_estimator"]["type"] = "ekf"
    with pytest.raises(ValueError):
        GatedMahonyEstimator.from_params(q, DT, 2.0)
    # the pad branch is PadAlignGyroEstimator's, bit for bit (bias average, levelling, n, aligned)
    rng = np.random.default_rng(3)
    a, b = _gm(), PadAlignGyroEstimator(DT, 2.0)
    f0 = quat.rotate_inverse(quat.from_axis_angle(np.array([0.6, 0.8, 0.0]), math.radians(2.0)), G_UP)
    for _ in range(1500):
        g = rng.normal(0.004, 0.001, 3)
        acc = f0 + rng.normal(0, 0.05, 3)
        assert np.array_equal(a.update(g, acc, True), b.update(g, acc, True))
    assert np.array_equal(a.bias, b.bias) and a.n == b.n and a.aligned and np.array_equal(a.a_lpf, a.f_mean)


def test_nav_estimator_no_correction_while_gated_climb():
    """T/W 1.19 climb (|f| = 1.19 g > (1 + accel_gate) g): the gate stays closed - exactly the v0.6.3 gyro-only
    integration (bit for bit), no bias learning."""
    rng = np.random.default_rng(4)
    a, b = _gm(), PadAlignGyroEstimator(DT, 2.0)
    for _ in range(600):
        g = rng.normal(0.003, 0.001, 3)
        a.update(g, G_UP, True)
        b.update(g, G_UP, True)
    # the low-pass needs a few ticks to leave the gate after the step from 1 g to 1.19 g (open while |a_lpf| < 1.08 g)
    k_open = 0
    while True:
        a.update(np.zeros(3), np.array([0.0, 0.0, 1.19 * G0]), False)
        if not a.gate:
            break
        k_open += 1
    assert k_open == math.ceil(math.log(1 - NE["accel_gate"] / 0.19) / math.log(1 - a.alpha_a)) - 1 == 10   # 40 ms
    b.q, b.bias = a.q.copy(), a.bias.copy()
    bias0 = a.bias.copy()
    for k in range(3000):
        g = np.array([0.02 * math.sin(k * 0.01), -0.01, 0.05]) + rng.normal(0, 0.002, 3)
        acc = np.array([0.3, -0.2, 1.19 * G0]) + rng.normal(0, 0.1, 3)
        assert np.array_equal(a.update(g, acc, False), b.update(g, acc, False))
        assert not a.gate and not a.e.any()
    assert np.array_equal(a.bias, bias0) and a.gated_ticks == k_open
    # the gate edge: |a_lpf| within accel_gate g opens it (strict), outside keeps it closed
    for ratio, want in ((1.0 + 0.95 * NE["accel_gate"], True), (1.0 - 0.95 * NE["accel_gate"], True),
                        (1.0 + 1.05 * NE["accel_gate"], False), (1.0 - 1.05 * NE["accel_gate"], False)):
        e = _gm()
        e.update(np.zeros(3), G_UP, True)
        for _ in range(500):
            e.update(np.zeros(3), G_UP * ratio, False)
        assert e.gate == want, ratio


def test_nav_estimator_converges_from_a_tilt_error_drag_balanced():
    """Hovering, un-accelerated: the specific force is the weight reaction (world up).  A 5 deg estimate error decays
    with ~1 / kp; the integral first takes part of it as a bias and returns it (bump), then the error goes to ~0."""
    e = _gm()
    for _ in range(600):
        e.update(np.zeros(3), G_UP, True)
    e.q = quat.from_axis_angle(X_AXIS, math.radians(5.0))
    errs = []
    for k in range(int(120 / DT)):
        e.update(np.zeros(3), G_UP, False)
        errs.append(_err_deg(quat.IDENTITY, e.q))
    at = lambda s: errs[int(s / DT) - 1]  # noqa: E731
    assert at(10) < 1.0 and max(errs[int(10 / DT):]) < 1.0 and at(120) < 0.1
    assert e.gated_ticks == len(errs)


def _hover_closed_loop(seconds: float, tau_d: float, x0_deg: float = 0.0, bias_dps: float = 0.0, k_att: float = 5.0,
                       est: GatedMahonyEstimator | None = None):
    """Synthetic hover (the estimator's reason for being): an ideal attitude loop rotates the vehicle about x at
    -k_att x (the estimated tilt), so the ESTIMATE is held upright; the thrust carries the weight (T cos x = m g); the
    horizontal speed v obeys dv/dt = -g tan(x) - v / tau_d (drag time constant tau_d = m / (dD/dv)); the accelerometer
    sees (thrust + drag) / m in the body frame; the gyro sees the rate + a bias appearing after the pad."""
    e = est or _gm()
    for _ in range(600):
        e.update(np.zeros(3), G_UP, True)
    b = np.array([math.radians(bias_dps), 0.0, 0.0])
    x, v = math.radians(x0_deg), 0.0
    log = []
    for k in range(int(seconds / DT)):
        ub = up_body(e.q)
        w = -k_att * math.atan2(ub[1], ub[2])
        t_m = G0 / math.cos(x)
        f_w = np.array([0.0, -t_m * math.sin(x) - v / tau_d, t_m * math.cos(x)])
        v += f_w[1] * DT
        a_b = quat.rotate_inverse(quat.from_axis_angle(X_AXIS, x), f_w)
        e.update(np.array([w, 0.0, 0.0]) + b, a_b, False)
        x += w * DT
        log.append((math.degrees(x), v, math.degrees(e.bias[0] - b[0])))
    return log, e


def test_nav_estimator_hover_closed_loop_with_drag():
    """The architect's mechanism: a tilt error turns into sideways drift until the drag balances it, then the
    specific force reveals the tilt.  tau_d 5 s, kp 0.15 (kp tau_d = 0.75 < 1): a 5 deg error is under 0.5 deg after
    40 s and 0.15 deg after 60 s; the drift peaks at ~2.6 m/s while it corrects (no position sensing - reported)."""
    log, _ = _hover_closed_loop(120.0, 5.0, x0_deg=5.0)
    x = [r[0] for r in log]
    vmax = max(abs(r[1]) for r in log)
    assert abs(x[int(40 / DT)]) < 0.5 and max(abs(t) for t in x[int(60 / DT):]) < 0.15 and 2.0 < vmax < 3.0, vmax
    log, _ = _hover_closed_loop(120.0, 2.6, x0_deg=5.0)                   # ram drag alone (~m / mdot): better damped
    assert max(abs(r[0]) for r in log[int(80 / DT):]) < 0.1


def test_nav_estimator_learns_a_gyro_bias_change_and_its_bound():
    log, e = _hover_closed_loop(300.0, 5.0, bias_dps=0.3)                 # 0.3 deg/s appears after the pad
    assert abs(log[-1][2]) < 0.01 and abs(log[-1][0]) < 0.05 and abs(log[int(100 / DT)][2]) < 0.03
    log, e = _hover_closed_loop(250.0, 5.0, bias_dps=1.0)                 # beyond bias_limit_dps 0.6: bounded learning
    lim = math.radians(NE["bias_limit_dps"])
    assert e.bias[0] - e.b_pad[0] == pytest.approx(lim, rel=1e-12)
    # the 0.4 deg/s the bound leaves is a steady tilt of 0.4 deg/s / kp (2.67 deg) and a steady drift (reported)
    assert log[-1][0] == pytest.approx(-math.degrees(math.radians(0.4) / NE["kp_ASSUMED"]), rel=0.02)


def test_nav_estimator_vibration_noise_robustness():
    """Documented vibration stand-in on the accelerometer (the MPU-6050 sees the EDF; nothing measured yet): white 0.3 g
    RMS per axis + a 1 g sinusoid at 61 Hz along body x (a fan harmonic aliased below the 125 Hz Nyquist of the 250 Hz
    tick).  Hovering upright: estimate error 0.50 deg RMS / 0.92 deg peak over 60 s (seed 11); the gate is open 91 % of
    the ticks (the 2 Hz low-pass leaves sqrt(alpha / (2 - alpha)) x 0.3 g = 0.047 g of noise on |a_lpf|, P(|N| > 0.08 g)
    ~ 9 %), so a stronger vibration starves the correction: measure the real spectrum (this stand-in is ASSUMED)."""
    rng = np.random.default_rng(11)
    e = _gm()
    for _ in range(600):
        e.update(np.zeros(3), G_UP, True)
    errs = []
    for k in range(int(60 / DT)):
        acc = G_UP + rng.normal(0, 0.3 * G0, 3) + np.array([G0 * math.sin(2 * math.pi * 61 * k * DT), 0.0, 0.0])
        e.update(rng.normal(0, 0.001, 3), acc, False)
        errs.append(_err_deg(quat.IDENTITY, e.q))
    rms = math.sqrt(sum(x * x for x in errs) / len(errs))
    assert rms < 0.6 and max(errs) < 1.5 and 0.85 * len(errs) < e.gated_ticks < 0.97 * len(errs), (rms, max(errs))


def test_nav_estimator_pad_reentry_levels_from_the_landing_spot():
    """LANDED -> SAFE: the pad branch starts from the specific force just measured (a_lpf), not from the previous pad -
    no attitude jump; PadAlignGyroEstimator would jump back to the old pad attitude."""
    e, old = _gm(), PadAlignGyroEstimator(DT, 2.0)
    for _ in range(600):
        e.update(np.zeros(3), G_UP, True)
        old.update(np.zeros(3), G_UP, True)
    q3 = quat.from_axis_angle(np.array([0.0, 1.0, 0.0]), math.radians(3.0))     # landed on a 3 deg slope
    f3 = quat.rotate_inverse(q3, G_UP)
    e.q = q3.copy()
    old.q = q3.copy()
    for _ in range(2000):
        e.update(np.zeros(3), f3, False)
        old.update(np.zeros(3), f3, False)
    q_new = e.update(np.zeros(3), f3, True)
    q_old = old.update(np.zeros(3), f3, True)
    assert _err_deg(q3, q_new) < 0.01 and _err_deg(q3, q_old) > 2.9


# =========================================================================== #
# v0.7.1 VTOL altitude filter (VtolAltitudeFilter) - synthetic streams
# =========================================================================== #
def test_vtol_alt_filter_gains_poles_and_the_v063_limit():
    f = VtolAltitudeFilter(0.5, 0.7, 40.0, 0.1)
    w, p = math.pi, 1 / 40.0
    assert (f.k1, f.k2, f.k3) == pytest.approx((1.4 * w + p, w * w + 1.4 * w * p, w * w * p), rel=1e-15)
    poles = sorted(f.poles, key=lambda z: (z.real, z.imag))
    want = sorted([complex(-p, 0), complex(-0.7 * w, w * math.sqrt(1 - 0.49)), complex(-0.7 * w, -w * math.sqrt(1 - 0.49))],
                  key=lambda z: (z.real, z.imag))
    assert all(abs(a - b) < 1e-9 for a, b in zip(poles, want))
    # bias pole -> 0: the v0.6.3 AltitudeFilter's numbers on the same stream
    old, new = AltitudeFilter(0.5, 0.7, 0.1), VtolAltitudeFilter(0.5, 0.7, 1e15, 0.1)
    old.reset(0.0)
    rng = random.Random(2)
    for k in range(5000):
        a = rng.gauss(0, 0.5)
        old.predict(a, DT)
        new.predict(a, DT)
        if k % 5 == 0:
            hb = rng.gauss(2.0, 0.3)
            old.correct(hb, k * DT)
            new.correct(hb, k * DT)
    assert new.h == pytest.approx(old.h, abs=1e-9) and new.v == pytest.approx(old.v, abs=1e-9) and abs(new.b) < 1e-9


def _alt_run(f: VtolAltitudeFilter, seconds: float, bias: float, t0: float = 0.0, pad: bool = False, baro: bool = True,
             h_of=lambda t: 0.0, comp: float = 0.0, u: float = 0.0, noise: float = 0.0, rng=None):
    """Truth h(t) (numerical a), accelerometer = a + bias, baro every 5 ticks = h + comp (u / u_hover)^2 + noise."""
    n = int(seconds / DT)
    for k in range(n):
        t = t0 + k * DT
        a = (h_of(t + DT) - 2 * h_of(t) + h_of(t - DT)) / (DT * DT)
        hb = None
        if baro and round(t / DT) % 5 == 0:
            hb = h_of(t) + comp + (rng.gauss(0, noise) if noise else 0.0)
        f.step(a + bias, DT, hb, t, pad, u)
    return t0 + n * DT


def test_vtol_alt_filter_pad_calibration_flight_learning_and_baro_lost():
    bias = 0.01 * G0                                                       # the 6-DOF's ASSUMED 10 mg residual
    f = VtolAltitudeFilter(0.5, 0.7, 40.0, 0.0)
    _alt_run(f, 2.0, bias, pad=True)                                       # N = 500 ticks: the running mean is exact
    assert f.n_pad == 500 and f.b == pytest.approx(bias, rel=1e-12)
    rng = random.Random(8)                                                 # with noise: the mean of the first N samples
    g = VtolAltitudeFilter(0.5, 0.7, 40.0, 0.0)
    xs = [bias + rng.gauss(0, 0.037) for _ in range(500)]
    for x in xs:
        g.pad(x, DT)
    assert g.b == pytest.approx(sum(xs) / 500, rel=1e-9)
    t = _alt_run(f, 18.0, bias, t0=2.0, pad=True)
    assert f.h == 0.0 and f.v == 0.0 and f.b == pytest.approx(bias, rel=1e-12) and f.n_pad == 500
    # in flight the bias moves (vibration rectification stand-in: + 3 mg): learnt with the 1 / 40 s pole
    t = _alt_run(f, 120.0, bias + 0.003 * G0, t0=t, h_of=lambda s: 5.0)
    assert f.b == pytest.approx(bias + 0.003 * G0, abs=0.003 * G0 * math.exp(-120 / 40) * 1.3)
    # baro lost for 20 s: predict only with the learnt bias; v0.6.3 filter (no bias state) for comparison
    v0, b0 = f.v, f.b
    _alt_run(f, 20.0, bias + 0.003 * G0, t0=t, baro=False, h_of=lambda s: 5.0)
    assert f.b == b0 and abs(f.h - 5.0) < 0.5 * abs(bias + 0.003 * G0 - b0) * 400 + abs(v0) * 20 + 0.02
    old = AltitudeFilter(0.5, 0.7, 0.0)
    old.reset(5.0)
    for _ in range(int(20 / DT)):
        old.predict(bias + 0.003 * G0, DT)
    assert old.h - 5.0 == pytest.approx(0.5 * 0.013 * G0 * 400, rel=1e-3) and abs(f.h - 5.0) < 0.05 * (old.h - 5.0)


def test_vtol_alt_filter_thrust_compensation_and_limits():
    u_h = 0.9148
    assert thrust_ratio(u_h, 1 / u_h, 2.0) == pytest.approx(1.0) and thrust_ratio(0.0, 1 / u_h, 2.0) == 0.0
    assert thrust_ratio(1.7, 1 / u_h, 2.0) == pytest.approx((1 / u_h) ** 2) and thrust_ratio(float("nan"), 2.0, 2.0) == 0.0
    assert thrust_ratio(0.5, 1 / u_h, 3.0) == pytest.approx((0.5 / u_h) ** 3)
    # the baro reads 1.2 m high at hover thrust: compensated, the filter holds the true 5 m; uncompensated: 6.2 m
    for comp_m, want in ((1.2, 5.0), (0.0, 6.2)):
        f = VtolAltitudeFilter(0.5, 0.7, 40.0, 0.0, baro_thrust_comp_m=comp_m, u_hover=u_h)
        _alt_run(f, 60.0, 0.0, h_of=lambda s: 5.0, comp=1.2 * thrust_ratio(u_h, 1 / u_h, 2.0), u=u_h)
        assert f.h == pytest.approx(want, abs=1e-3) and f.last_comp_m == pytest.approx(comp_m, rel=1e-12)
    assert VtolAltitudeFilter.from_params(P, u_h, 0.5, 0.7, 0.1).comp_m == VTOL["baro_thrust_comp_m"] == 0.0
    # the bias state is bounded; the pad only stamps baro times (the first flight correction spans one baro period)
    f = VtolAltitudeFilter(0.5, 0.7, 40.0, 0.0, bias_limit_mps2=0.2)
    _alt_run(f, 5.0, 3.0, pad=True)
    assert f.b == 0.2
    f = VtolAltitudeFilter(0.5, 0.7, 40.0, 0.0)
    f.step(0.0, DT, 0.0, 0.0, True)
    f.step(0.0, DT, 1.0, 5 * DT, False)
    assert f.h == pytest.approx(f.k1 * 1.0 * 5 * DT) and f.b == pytest.approx(-f.k3 * 1.0 * 5 * DT)


# =========================================================================== #
# v0.7.1 ground logic texts, link budget
# =========================================================================== #
def test_ground_vtol_rule_events_and_korean_texts():
    codes = {lp.REASON_NONE, *lp.VTOL_INTERLOCK_REASONS, *lp.VTOL_REJECT_REASONS}
    assert codes <= set(REASON_TEXT_KO) and all(REASON_TEXT_KO[c] for c in codes)
    assert reason_text_ko(lp.R_I4_APOGEE_LIMIT) == lp.REASON_NAMES[lp.R_I4_APOGEE_LIMIT] and reason_text_ko(200) == "reason 200"
    assert {lp.R_V_BARO_LIMIT_LANDED, lp.R_V_BARO_LIMIT_ABORT, lp.R_V_LAND_TIMEOUT} <= set(lp.VTOL_INTERLOCK_REASONS)
    assert lp.REASON_NAMES[lp.R_V_LAND_TIMEOUT].startswith("V10") and not set(lp.BALLISTIC_INTERLOCK_REASONS) & {21, 22, 23}
    gs = GroundAssist(_gp())
    gs.poll(0.0)
    gs.receive(_t2(0, ack=3, t_ms=10, state=lp.STATE_POWERED, phase=LAND), 0.2)
    gs.receive(_t2(0, ack=3, t_ms=600, state=lp.STATE_UNPOWERED, phase=LANDED, reason=lp.R_V_LAND_TIMEOUT,
                   flags=lp.FLAG_INTERLOCK), 0.7)
    gs.receive(_t2(0, ack=3, t_ms=900, state=lp.STATE_UNPOWERED, phase=LANDED, reason=lp.R_V_LAND_TIMEOUT,
                   flags=lp.FLAG_INTERLOCK), 1.0)
    ev = [e for _, e in gs.events if e.startswith("rule")]
    assert ev == ["rule V10 LAND floor timeout: landed (motor off, no release) -> UNPOWERED / LANDED"]
    assert gs.vtol_status()["reason_ko"] == REASON_TEXT_KO[lp.R_V_LAND_TIMEOUT]
    gb = GroundAssist(_gp(mode=lp.MODE_BALLISTIC))                         # ballistic: TELEM v1, no rule events
    gb.poll(0.0)
    t1 = lp.Telemetry.from_physical(state=lp.STATE_ABORT, flags=lp.FLAG_INTERLOCK, reason=lp.R_I2_TILT, ack_seq=1, nonce=1,
                                    t_ms=5, h_m=0, vz_mps=0, roll_deg=0, pitch_deg=0, yaw_deg=0, tilt_deg=0, throttle=0,
                                    vcell_v=4, loop_max_us=0)
    gb.receive(lp.encode_telemetry(90, 0, t1), 0.1)
    assert not [e for _, e in gb.events if e.startswith("rule")]


def test_link_budget_sizes_the_telem2_downlink():
    for r in lm.RADIOS:
        cfg = lm.LinkConfig.from_params(P, radio=r)
        b1, b2 = lm.budget(cfg), lm.budget(cfg, mode=lp.MODE_VTOL)
        assert b1["telem_frame_bytes"] == lp.TELEM_FRAME_BYTES == 36 and b2["telem_frame_bytes"] == lp.TELEM2_FRAME_BYTES == 39
        assert lm.round_trip(cfg) == lm.round_trip(cfg, mode=lp.MODE_BALLISTIC) == lm.round_trip(cfg, telem_bytes=36)
        assert b1["downlink_s"] == lm.hop_timing(cfg, "down", 36).arrival                  # the v0.6.3 numbers
        assert b2["downlink_s"] == lm.hop_timing(cfg, "down", 39).arrival > b1["downlink_s"]
        assert b2["reply_timeout_s"] - b1["reply_timeout_s"] == pytest.approx(b2["downlink_s"] - b1["downlink_s"])
        assert b2["sx127x"]["toa_telem_s"] == lm.lora_time_on_air(39, cfg.sf, cfg.bw_hz, cfg.cr_denom, cfg.preamble)
        assert b2["poll_hz_achievable"] == (1.0 / b2["rtt_worst_s"] >= cfg.poll_hz)
        d = (b2["downlink_s"] - b1["downlink_s"]) * 1e3
        assert d == pytest.approx(16.25 if r == "lora_uart" else 8.25, abs=0.01), (r, d)
    with pytest.raises(ValueError):
        lm.round_trip(lm.LinkConfig.from_params(P), telem_bytes=40)
    try:                                                                      # the sim's own wrapper agrees (if present)
        from sim.sim6dof import vtol_round_trip
    except Exception:  # pragma: no cover
        return
    cfg = lm.LinkConfig.from_params(P)
    assert vtol_round_trip(cfg) == lm.round_trip(cfg, mode=lp.MODE_VTOL)


# =========================================================================== #
# v0.9 control round (VTOL hardware): attitude options, rotor feed-forwards, take-off law, V16
# =========================================================================== #
def _mixer_plant(t_max=30.3):
    from sim.controller import MixerPlant
    return MixerPlant(l_v_m=0.21, r_v_m=0.028, s_v_m2=0.00096, cl_alpha=2.4, a_e_m2=0.005073, t_max_n=t_max,
                      throttle_exp=2.0, delta_max_rad=math.radians(20.0), pwm_center_us=1500.0, pwm_us_per_deg=11.1)


def _vtol_gains():
    from sim.controller import gains_for_mode
    with open(os.path.join(_ROOT, "sim", "out", "gains.json"), encoding="utf-8") as f:
        gj = json.load(f)
    return gains_for_mode(P["control"], gj, lp.MODE_VTOL)


def test_v09_reduced_error_ignores_the_heading_error():
    """quat.reduced_error: e_xy = 2 (w x - y z, w y + x z) is the same for every heading error (the rotation vector's xy
    part turns by phi / 2); e_z = 2 z; to first order both equal the rotation vector."""
    tilt = quat.from_axis_angle(np.array([1.0, 0.0, 0.0]), math.radians(10.0))
    base = None
    for phi in (0.0, 45.0, 120.0, 170.0):
        q_sp = quat.multiply(tilt, quat.from_yaw(0.0))
        q = quat.from_yaw(math.radians(phi))                        # level body, heading error phi
        e = quat.reduced_error(quat.error(q, q_sp))
        rv = quat.to_rotvec(quat.error(q, q_sp))
        exy_world = quat.rotate(q, np.array([e[0], e[1], 0.0]))[:2]  # the correction axis in the world frame
        base = exy_world if base is None else base
        assert np.allclose(exy_world, base, atol=1e-12)              # never turned by the heading error
        if phi >= 120.0:                                             # the rotation vector's xy part is turned
            assert not np.allclose(quat.rotate(q, np.array([rv[0], rv[1], 0.0]))[:2], base, atol=1e-2)
    rng = np.random.default_rng(9)
    for _ in range(50):                                              # small angles: e == rotation vector to 2nd order
        v = rng.normal(size=3) * 1e-3
        qe = quat.error(quat.from_rotvec(v), quat.from_yaw(0.0))
        assert np.allclose(quat.reduced_error(qe), quat.to_rotvec(qe), atol=2e-6)     # differ at 2nd order only


def test_v09_vtol_overrides_and_the_ballistic_build():
    """params control.vtol_overrides: the light-vehicle gains and the v0.9 options; the ballistic build flies exactly
    gains.json (no v0.9 key leaks into it); ff_rotor_h_nms = s I_rot omega_ref from params.edf."""
    from sim.controller import VTOL_EXTRA_KEYS, gains_for_mode, gains_from_control, vtol_overrides
    ctl = P["control"]
    ov = vtol_overrides(ctl)
    with open(os.path.join(_ROOT, "sim", "out", "gains.json"), encoding="utf-8") as f:
        gj = json.load(f)
    gb = gains_for_mode(ctl, gj, lp.MODE_BALLISTIC)
    assert gb == gains_from_control(ctl, gj) and not set(VTOL_EXTRA_KEYS) & set(gb)
    gv = gains_for_mode(ctl, gj, lp.MODE_VTOL)
    assert all(gv[k] == v for k, v in ov.items())
    e = P["edf"]
    h = 1.0 * float(e["thrust_v09"]["rotor_inertia_kg_m2_ASSUMED"]) * float(e["kv"]) * float(e["voltage_nominal_V"]) \
        * 2.0 * math.pi / 60.0
    assert "CW" in e["rotation"] and "CCW" not in e["rotation"]
    assert ov["ff_rotor_h_nms"] == pytest.approx(h, rel=1e-4)
    assert ov["ff_rotor_tau_s"] == e["thrust_time_constant_s"]
    for bad in ({"att_err_mode2": 1}, {"loop_hz": 500}):
        with pytest.raises(ValueError):
            vtol_overrides(dict(ctl, vtol_overrides=bad))


def test_v09_per_axis_anti_windup():
    """aw_per_axis: a yaw command shrunk by the mixer freezes only the yaw integrator (and only while the yaw error
    pushes further into the limit); v0.8.1 (aw_per_axis 0) freezes all three."""
    from sim.controller import AttitudeController
    for aw, xy_moves in ((1, True), (0, False)):
        g = dict(_vtol_gains(), aw_per_axis=aw, ff_rotor_h_nms=0.0, att_err_mode=0)
        c = AttitudeController(P["control"], _mixer_plant(), g)
        c.set_yaw_reference(quat.from_yaw(0.0))
        q = quat.from_yaw(math.radians(-60.0))                       # big heading error: yaw saturates the mixer
        for _ in range(3):
            o = c.update(q, np.array([1.0, 0.0, 0.0]), 0.75)         # + a pitch rate error: d_x ~18 deg + d_z ~5 deg
        assert o.saturated and c.mixer.last_lim == (False, True)       # yaw shrunk, pitch / roll not limited
        i0 = c.pid.integ.copy()
        c.update(q, np.array([1.0, 0.0, 0.0]), 0.75)
        assert (c.pid.integ[0] != i0[0]) == xy_moves                 # pitch integrates only per axis
        assert c.pid.integ[2] == i0[2]                               # yaw frozen (error pushes into the limit)


def test_v09_rotor_feed_forwards_cancel_the_plant_terms():
    """ff_rotor_react: the yaw command carries -h_ref (u - u_lag) / tau / roll_priority_scale; with the mixer's
    roll_priority_scale the physical torque cancels the plant's spool reaction s I_rot domega/dt (sim.edf.RotorModel)
    for omega = omega_ref u_lag.  ff_rotor_gyro: + (w_y h_z, -w_x h_z) cancels -omega x h."""
    from sim.controller import AttitudeController
    from sim.edf import RotorModel
    g0 = dict(_vtol_gains(), att_err_mode=0, aw_per_axis=1, ff_rotor_gyro=1)   # fix round: params fly the gyro FF off;
                                                                               # the equations are checked with it on
    rot = RotorModel.from_params(P, 30.3)
    c0 = AttitudeController(P["control"], _mixer_plant(), dict(g0, ff_rotor_h_nms=0.0))
    c1 = AttitudeController(P["control"], _mixer_plant(), g0)
    for c in (c0, c1):
        c.set_yaw_reference(quat.from_yaw(0.0))
    q = quat.from_yaw(0.0)
    w = np.array([0.4, -0.2, 0.0])
    u_prev = 0.6
    for c in (c0, c1):
        c.update(q, w, u_prev)
    o0, o1 = c0.update(q, w, 0.62), c1.update(q, w, 0.62)            # throttle step +0.02
    tau = g0["ff_rotor_tau_s"]
    du = 0.62 - 0.6
    dm = o1.moments - o0.moments
    assert dm[2] * g0["roll_priority_scale"] == pytest.approx(-g0["ff_rotor_h_nms"] * du / tau, rel=1e-9)
    # the plant's reaction for the same domega/dt (omega = omega_ref sqrt(T / T_fan), T = T_fan u^2): s I domega/dt
    domega = rot.omega_ref_rad_s * du / tau
    assert -dm[2] * g0["roll_priority_scale"] == pytest.approx(rot.sign * rot.i_rot_kg_m2 * domega, rel=1e-3)
    u_lag = c1.u_lag
    hz = -g0["ff_rotor_h_nms"] * u_lag
    assert dm[0] == pytest.approx(o1.gyro_filtered[1] * hz, rel=1e-9)
    assert dm[1] == pytest.approx(-o1.gyro_filtered[0] * hz, rel=1e-9)
    gm = rot.gyro_moment(tuple(o1.gyro_filtered), rot.t_fan_n * u_lag * u_lag)
    assert dm[0] + gm[0] == pytest.approx(0.0, abs=1e-3 * abs(gm[0]) + 1e-12)


def test_v09_takeoff_two_slope_ramp_and_thrust_search():
    """mission.vtol.takeoff_v09: the ceiling rises to u_start in ramp_ticks, then at slew_per_s; on the pad with the
    ramp over and no climb the thrust search lowers k_T (u_ff rises) until kt_min; the take-off check then acts at u_max
    (V15) - a vehicle that climbs passes and the search stops."""
    t9 = VTOL["takeoff_v09"]
    vp = VerticalParams.from_params(P, mass_kg=1.647, t_max_n=T_MAX)
    assert vp.to_u_start == t9["u_start"] and vp.to_slew_per_s == t9["slew_per_s"]
    assert vp.ramp_total_ticks == vp.ramp_ticks + math.ceil((vp.u_max - vp.to_u_start) * HZ / vp.to_slew_per_s - 1e-9)
    assert vp.cap_at(vp.ramp_ticks - 1) == pytest.approx(vp.to_u_start)
    assert vp.cap_at(vp.ramp_ticks + HZ - 1) == pytest.approx(min(vp.u_max, vp.to_u_start + vp.to_slew_per_s))
    assert vp.cap_at(vp.ramp_total_ticks + 10) == vp.u_max
    c = VerticalController(vp)
    us, kts, fail_at = [], [], None
    for k in range(vp.ramp_total_ticks + 20 * HZ):                    # never lifts: h stays 0
        o = c.update(_vin(phase=CLIMB, tgt=5.0, h=0.0, vz=0.0))
        us.append(o.u)
        kts.append(o.k_thrust)
        if o.takeoff_fail:
            fail_at = k
            break
    assert all(u <= vp.cap_at(k) + 1e-12 for k, u in enumerate(us))
    assert kts[vp.ramp_total_ticks - 2] == pytest.approx(1.0)        # no search on the ramp
    assert min(kts) == pytest.approx(vp.kt_min)                       # the search reached kt_min ...
    assert fail_at is not None and us[-1] == vp.u_max                 # ... and V15's flag came at u_max
    c2 = VerticalController(vp)                                       # a vehicle that climbs: the check passes
    h, vz = 0.0, vp.to_search_vz_mps + 0.2                            # faster than the search threshold: no search
    for k in range(vp.ramp_total_ticks + 3 * HZ):
        o = c2.update(_vin(phase=CLIMB, tgt=8.0, h=h, vz=vz if k > vp.ramp_ticks else 0.0))
        if k > vp.ramp_ticks:
            h += vz / HZ
    assert o.takeoff_passed and not o.takeoff_fail and o.k_thrust == pytest.approx(1.0)


def test_v09_v16_is_off_unless_enabled():
    """add-on C (V16): params without mission.vtol.yaw_authority_land_ASSUMED.enabled -> v16_ticks 0 (never acts)."""
    sp = SupervisorParams.from_params(P, mode=lp.MODE_VTOL)
    y = VTOL.get("yaw_authority_land_ASSUMED") or {}
    assert (sp.v16_ticks > 0) == bool(y.get("enabled"))
    p2 = copy.deepcopy(P)
    p2["mission"]["vtol"]["yaw_authority_land_ASSUMED"] = {"enabled": True, "rate_dps": 300.0, "persist_s": 0.25}
    sp2 = SupervisorParams.from_params(p2, mode=lp.MODE_VTOL)
    assert sp2.v16_ticks == ticks_ge(0.25, HZ) and sp2.v16_rate_rad_s == pytest.approx(math.radians(300.0))
