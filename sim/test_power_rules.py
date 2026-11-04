"""v0.9 power / tether round: the onboard power rules (sim.onboard "v0.9 power rules"), their codes and the proposed
TELEM v4 tail (sim.power_link), the ground station's power texts (sim.ground_station_logic) and the rule scenarios
(sim.tether_scenarios).  Unit tests drive the supervisor with synthetic ADC readings tick by tick (exact tick counts);
the 6-DOF tests fly one scenario per rule family (the full named set is the round's evidence, scratch v09sim/tether)."""
from __future__ import annotations

import json
import os

import pytest

from sim import link_proto as lp
from sim import power_link as pl
from sim.ground_station_logic import (POWER_ALARM_KO, REASON_TEXT_KO, GroundAssist, GroundParams, power_on_window_ok,
                                      power_status_ko)
from sim.onboard import (OnboardInputs, OnboardSupervisor, PowerMonitor, State, SupervisorParams, UplinkFrame,
                         q4_code, tether_limits, ticks_ge)
from sim.vertical_ctrl import VerticalController, VerticalInputs, VerticalParams

HERE = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(HERE, "..", "params.json"), encoding="utf-8") as _f:
    P = json.load(_f)
PR = P["mission"]["vtol"]["power_rules_ASSUMED"]
HZ = int(P["control"]["loop_hz"])
P_, A_, S_, U_, AB_ = State.POWERED, State.ARMED, State.SAFE, State.UNPOWERED, State.ABORT
CLIMB, HOLD, LAND = lp.PHASE_CLIMB, lp.PHASE_HOLD, lp.PHASE_LAND


def sp_of(power: str = "auto", profile: str = "demo") -> SupervisorParams:
    return SupervisorParams.from_params(P, mode="vtol", profile=profile, gps=False, takeoff_check=True, power=power)


# --------------------------------------------------------------------------- #
# sim.power_link
# --------------------------------------------------------------------------- #
def test_power_codes_are_free_and_named():
    """The power codes sit in the interlock range in a block of their own (no clash with link_proto's codes) and are
    named in link_proto.REASON_NAMES (registered at import until link_proto adopts them)."""
    taken = set(k for k in lp.REASON_NAMES if k not in pl.POWER_REASON_NAMES)
    assert not taken & set(pl.POWER_REASON_NAMES)
    assert all(0 < c < 0x40 for c in pl.POWER_INTERLOCK_REASONS) and pl.REJ_POWER >= 0x40
    for c, n in pl.POWER_REASON_NAMES.items():
        assert lp.REASON_NAMES[c] == n
    assert lp.REASON_NAMES[pl.R_W_LOST].startswith("W_LOST") and lp.REASON_NAMES[pl.R_W_SAG].startswith("W_SAG")
    assert pl.build_of_source("tether") == pl.build_of_source("wired") == pl.PWR_WIRED
    assert pl.build_of_source("hybrid") == pl.PWR_WIRED_BUFFER and pl.build_of_source(None) == pl.PWR_BATTERY
    with pytest.raises(ValueError):
        pl.build_of_source("solar")


def test_power_tail_encode_decode_and_budget():
    assert pl.TELEM2_PAYLOAD_LEN_REF == lp.TELEM2_PAYLOAD_LEN and pl.TELEM3_PAYLOAD_LEN_REF == lp.TELEM3_PAYLOAD_LEN
    assert pl.TELEM4_FRAME_BYTES == lp.TELEM2_FRAME_BYTES + 2 == 41 and pl.TELEM5_FRAME_BYTES == lp.TELEM3_FRAME_BYTES + 2
    for va, fl, b in ((22.8, pl.PF_TETHER_OK | pl.PF_BUF_SHARE, pl.PWR_WIRED_BUFFER), (0.0, 0, pl.PWR_WIRED),
                      (31.875, 0x3F, 3)):
        t = pl.PowerTelem.from_physical(va, fl, b)
        assert pl.PowerTelem.decode(t.encode()) == t and t.build == b and t.flags & 0x3F == fl
        assert t.v_aux_v == pytest.approx(va, abs=0.0625)
    assert pl.quant_v(None) == 0 and pl.quant_v(float("nan")) == 0 and pl.quant_v(-3.0) == 0 and pl.quant_v(99.0) == 255
    with pytest.raises(ValueError):
        pl.PowerTelem.decode(bytes(3))
    from sim import link_model as lm
    for radio in lm.RADIOS:
        cfg = lm.LinkConfig.from_params(P, radio=radio)
        b = pl.telem4_budget(cfg)
        # both proposed frames keep the 2 Hz poll on both backends; a 3-byte tail on TELEM v3 would not (lora_uart)
        assert b["telem4"]["poll_ok"] and b["telem5"]["poll_ok"]
        assert b["telem5"]["rtt_s"] > b["telem3"]["rtt_s"] > b["telem4"]["rtt_s"] > b["telem2"]["rtt_s"]
        if radio == "lora_uart":
            assert not b["telem3_plus3"]["poll_ok"]
        rt3 = lm.round_trip(cfg, mode=lp.MODE_VTOL, gps=True)
        assert b["telem3"]["rtt_s"] == pytest.approx(rt3["rtt_mean_s"]) and b["telem3"]["reply_timeout_s"] == pytest.approx(
            rt3["reply_timeout_s"])


# --------------------------------------------------------------------------- #
# parameters
# --------------------------------------------------------------------------- #
def test_power_params_from_params_and_limits():
    sp = sp_of()
    vpc = float(PR["adc_v_per_count"])
    assert vpc == pytest.approx(P["power"]["electrical_plant_ASSUMED"]["adc_full_scale_v"] / 1024.0)
    assert sp.power == "auto" and sp.power_rules
    assert sp.pw_sag_q == q4_code(PR["sag_v"], vpc) == lp.round_half_away(16 * PR["sag_v"] / vpc)
    assert sp.pw_buf_low_q == q4_code(6 * PR["buf_low_cell_v"], vpc)
    assert sp.pw_lost_ticks == ticks_ge(PR["lost_persist_s"], HZ) == 10
    assert sp.pw_sag_land_ticks == ticks_ge(PR["sag_land_s"], HZ)
    base, teth = tether_limits(sp, False), tether_limits(sp, True)
    v = P["mission"]["vtol"]["profiles"]["demo"]
    assert base.alt_max_dm == round(10 * v["alt_max_m"]) and base.ceiling_m == v["ceiling_m"]
    tl = PR["tether_limits"]
    assert teth.alt_max_dm == round(10 * min(tl["alt_max_m"], v["alt_max_m"]))
    assert teth.ceiling_m == min(tl["ceiling_m"], v["ceiling_m"]) and teth.geofence_m == min(tl["geofence_m"], v["geofence_m"])
    assert teth.endurance_ticks == ticks_ge(PR["tether_endurance"]["max_flight_s"] - PR["tether_endurance"]["land_reserve_s"], HZ)
    # the field profile: the tether block's smaller limits win
    spf = sp_of(profile="field")
    tf = tether_limits(spf, True)
    assert tf.hover_dm == 15 and tf.alt_min_dm == 10 and tf.alt_max_dm == 20
    # battery / ballistic: no power fields
    sb = SupervisorParams.from_params(P, mode="ballistic")
    assert not sb.power_rules and sb.power == "battery"


def test_inconsistent_power_thresholds_are_refused():
    import copy
    q = copy.deepcopy(P)
    q["mission"]["vtol"]["power_rules_ASSUMED"]["buf_low_cell_v"] = 3.8          # above the ON window
    with pytest.raises(ValueError):
        SupervisorParams.from_params(q, mode="vtol", gps=False)
    q = copy.deepcopy(P)
    q["mission"]["vtol"]["power_rules_ASSUMED"]["tether_limits"]["ceiling_m"] = 1.8    # below alt_max
    with pytest.raises(ValueError):
        SupervisorParams.from_params(q, mode="vtol", gps=False)


# --------------------------------------------------------------------------- #
# supervisor harness
# --------------------------------------------------------------------------- #
class _H:
    """The VTOL supervisor with synthetic power readings (healthy tether / buffer by default)."""

    def __init__(self, power: str = "wired", **base):
        self.sp = sp_of(power)
        self.sup = OnboardSupervisor(self.sp, seed=5)
        self.k = 0
        self.seq = 0
        self.base = dict(h_m=0.0, vz_mps=0.0, tilt_deg=0.5, v_cell=4.2, throttle_prev=0.0)
        if power in ("wired", "auto_wired"):
            self.base.update(v_in=25.2, v_tether=25.2)
        if power == "wired_buffer":
            self.base.update(v_in=25.18, v_tether=25.2, v_buffer=22.8, buf_i_pos=False)
        self.base.update(base)

    def tick(self, frames=(), **kw):
        d = dict(self.base)
        d.update(kw)
        out = self.sup.update(OnboardInputs(tick=self.k, frames=list(frames), **d))
        self.k += 1
        return out

    def send(self, cmd: int, **kw):
        self.seq = (self.seq + 1) & 0xFF
        return self.tick([UplinkFrame(self.seq, lp.Command(cmd, 0, self.sup.nonce))], **kw)

    def hb(self, **kw):
        return self.tick([UplinkFrame(self.seq, lp.Command(lp.CMD_NONE))], **kw)

    def run(self, n: int, **kw):
        out = None
        for i in range(n):
            out = self.hb(**kw) if i % 50 == 0 else self.tick(**kw)
        return out

    def to_hold(self, **kw):
        self.hb(**kw)
        self.tick(**kw)
        assert self.send(lp.CMD_ON, **kw).state == A_
        self.run(self.sp.spool_ticks, **kw)
        assert self.sup.state == P_ and self.sup.phase == CLIMB
        self.tick(vc_at_target=True, **kw)
        assert self.sup.phase == HOLD
        return self


def test_battery_build_ignores_v_in_and_keeps_i8():
    """'auto' with only V_in measured = the battery build: I8 on V_cell, the profile's limits (bit-identical to v0.8.1)."""
    h = _H("auto", v_in=24.0)
    h.to_hold()
    assert h.sup.build == pl.PWR_BATTERY and h.sup.lim == tether_limits(h.sp, False)
    out = h.tick(v_cell=3.2, v_in=19.2)
    assert out.phase == LAND and out.reason == lp.R_I8_VCELL and out.u_cap is None


def test_auto_build_follows_the_plant_signals_and_tether_limits():
    h = _H("auto", v_in=25.2, v_tether=25.2)
    h.hb()
    assert h.sup.build == pl.PWR_WIRED and h.sup.lim == tether_limits(h.sp, True)
    hb = _H("auto", v_in=25.18, v_tether=25.2, v_buffer=22.8)
    hb.hb()
    assert hb.sup.build == pl.PWR_WIRED_BUFFER


@pytest.mark.parametrize("v_in, ok", [(25.2, True), (24.7, True), (24.4, False), (25.9, False), (23.0, False)])
def test_wired_on_window(v_in, ok):
    h = _H("wired", v_in=v_in, v_tether=v_in)
    h.hb()
    h.tick()
    out = h.send(lp.CMD_ON)
    assert (out.state == A_) == ok
    if not ok:
        assert h.sup.exec_log[-1]["code"] == pl.REJ_POWER and out.reason == pl.REJ_POWER


@pytest.mark.parametrize("v_buf_cell, ok", [(3.80, True), (3.73, True), (3.70, False), (3.95, False)])
def test_hybrid_on_window_checks_the_buffer(v_buf_cell, ok):
    h = _H("wired_buffer", v_buffer=6 * v_buf_cell)
    h.hb()
    h.tick()
    assert (h.send(lp.CMD_ON).state == A_) == ok


def test_takeoff_gate_rechecks_the_power_window():
    h = _H("wired")
    h.hb()
    h.tick()
    assert h.send(lp.CMD_ON).state == A_
    h.run(h.sp.spool_ticks, v_in=22.0)                    # the supply sags during the spool warning
    assert h.sup.state == S_ and h.sup.reason == lp.R_V_TAKEOFF_GATE


def test_w_sag_freezes_the_ceiling_refuses_up_then_lands_with_hysteresis():
    sp = sp_of("wired")
    h = _H("wired").to_hold()
    h.run(5, throttle_prev=0.71)
    # the filter needs a few ticks to cross; then warn_ticks of persistence
    # the Q4 filter (shift 3) needs ~18 ticks to cross sag_v on a 25.2 -> 19.5 V step, then warn_ticks of persistence
    outs = [h.tick(v_in=19.5, throttle_prev=0.71) for _ in range(sp.pw_sag_warn_ticks + 30)]
    first = next(i for i, o in enumerate(outs) if o.u_cap is not None)
    assert sp.pw_sag_warn_ticks <= first + 1 <= sp.pw_sag_warn_ticks + 22
    assert outs[-1].u_cap == pytest.approx(0.71) and outs[-1].pwr.flag(pl.PF_SAG) and outs[-1].phase == HOLD
    assert h.send(lp.CMD_UP, v_in=19.5).reason == pl.REJ_POWER                 # UP refused while sagging
    # inside the hysteresis band (sag_v .. sag_v + hyst) the count is held: no land, no release
    band = PR["sag_v"] + 0.5 * PR["sag_hyst_v"]
    h.run(30, v_in=band)                                  # the filter settles into the band (~8 ticks counting on)
    n_held = h.sup.pm.sag_cnt
    h.run(100, v_in=band)
    assert h.sup.phase == HOLD and h.sup.pm.sag_cnt == n_held and h.sup.pm.sag_warn
    # back below: the LAND comes when the count reaches sag_land_ticks
    for _ in range(sp.pw_sag_land_ticks):
        out = h.tick(v_in=19.5)
        if out.phase == LAND:
            break
    assert out.phase == LAND and out.reason == pl.R_W_SAG and h.sup.pm.sag_cnt == sp.pw_sag_land_ticks
    assert any("W_SAG warning:" in e for _, e in h.sup.events)


def test_w_sag_warning_is_released_after_recovery():
    sp = sp_of("wired")
    h = _H("wired").to_hold()
    h.run(sp.pw_sag_warn_ticks + 40, v_in=19.8, throttle_prev=0.7)
    assert h.sup.pm.sag_warn
    h.run(sp.pw_sag_release_ticks + 20, v_in=23.0)
    assert not h.sup.pm.sag_warn and h.tick().u_cap is None and h.sup.phase == HOLD


def test_power_rule_before_the_takeoff_stops_the_motor_safe():
    """A power rule in CLIMB before the take-off check passed (on the pad): motor off -> SAFE, no release."""
    sp = sp_of("wired")
    h = _H("wired")
    h.hb()
    h.tick()
    h.send(lp.CMD_ON)
    h.run(sp.spool_ticks)
    assert h.sup.phase == CLIMB
    h.run(sp.pw_sag_land_ticks + 20, v_in=19.0, vc_takeoff_passed=False, vc_support=True)
    assert h.sup.state == S_ and h.sup.reason == pl.R_W_SAG and not h.sup.release_open


def test_power_rule_before_the_takeoff_in_the_air_lands():
    """v0.9 fix round (control review HIGH 1): the take-off check not passed yet but the vehicle off the pad (no support
    signature, or the lift-off latched): the power rule LANDs - it never stops the motor in the air."""
    for kw in ({"vc_support": False}, {"vc_support": True, "vc_airborne": True}):
        sp = sp_of("wired")
        h = _H("wired")
        h.hb()
        h.tick()
        h.send(lp.CMD_ON)
        h.run(sp.spool_ticks)
        assert h.sup.phase == CLIMB
        h.run(sp.pw_sag_land_ticks + 20, v_in=19.0, vc_takeoff_passed=False, **kw)
        assert h.sup.state == P_ and h.sup.phase == LAND and h.sup.reason == pl.R_W_SAG, kw


def test_w_lost_lands_within_the_proposal_time():
    """Hybrid: V_tether -> 0 at tick k0: LAND (R_W_LOST) within 0.1 s (proposal W2) - the filter + lost_persist_s."""
    sp = sp_of("wired_buffer")
    h = _H("wired_buffer").to_hold()
    k0 = h.k
    for _ in range(ticks_ge(0.1, HZ)):
        out = h.tick(v_tether=0.0, v_in=21.6, v_buffer=21.6, buf_i_pos=True)
        if out.phase == LAND:
            break
    assert out.phase == LAND and out.reason == pl.R_W_LOST and out.pwr.flag(pl.PF_LOST)
    assert sp.pw_lost_ticks <= h.k - k0 <= ticks_ge(0.1, HZ)


def test_w_lost_when_the_supply_falls_below_the_buffer_and_not_at_the_crossover():
    h = _H("wired_buffer").to_hold()
    # crossover: both conduct - V_tether = V_buffer terminal: no W_LOST
    h.run(500, v_tether=22.3, v_in=22.28, v_buffer=22.3, buf_i_pos=True)
    assert h.sup.phase == HOLD and h.sup.reason == 0
    # the supply 0.6 V below the buffer (D1 blocks): W_LOST
    h.run(30, v_tether=21.0, v_in=21.6, v_buffer=21.62, buf_i_pos=True)
    assert h.sup.phase == LAND and h.sup.reason == pl.R_W_LOST


def test_w_lost_during_land_latches_the_reason_once():
    h = _H("wired_buffer").to_hold()
    h.send(lp.CMD_OFF)
    assert h.sup.phase == LAND and h.sup.reason == 0
    h.run(40, v_tether=0.0, v_buffer=21.6)
    assert h.sup.phase == LAND and h.sup.reason == pl.R_W_LOST
    assert sum("W_LOST tether lost during LAND" in e for _, e in h.sup.events) == 1


@pytest.mark.parametrize("v_cell, n, code", [(3.35, 100, pl.R_W_BUF), (3.55, 600, pl.R_W_BUF)])
def test_w_buf_under_load_fast_and_reserve_slow(v_cell, n, code):
    sp = sp_of("wired_buffer")
    h = _H("wired_buffer").to_hold()
    out = h.run(n, v_buffer=6 * v_cell, v_tether=6 * v_cell, buf_i_pos=True)
    assert out.phase == LAND and out.reason == code
    if v_cell > 3.4:                                  # the reserve rule needs its full persistence
        assert h.sup.pm.res_cnt >= sp.pw_buf_res_ticks


def test_w_d2_buffer_charging_while_tethered():
    """No plant fault model yet (request): synthetic - the buffer pulled above d2_cell_v by the tether."""
    h = _H("wired_buffer").to_hold()
    out = h.run(200, v_buffer=6 * 4.05, v_tether=25.0)
    assert out.phase == LAND and out.reason == pl.R_W_D2


def test_i8_maps_to_the_buffer_in_the_hybrid_and_never_in_wired():
    h = _H("wired").to_hold()
    out = h.run(50, v_cell=3.0)                      # the supply is not a pack: no I8 in the wired build
    assert out.phase == HOLD and out.reason == 0


def test_tethered_altitude_limits_refuse_up_above_alt_max():
    h = _H("wired").to_hold()
    assert h.sup.tgt_dm == 15
    assert h.send(lp.CMD_UP).reason == 0 and h.sup.tgt_dm == 20
    out = h.send(lp.CMD_UP)
    assert h.sup.exec_log[-1]["code"] == lp.REJ_ALT_LIMIT and h.sup.tgt_dm == 20 and out.phase == HOLD
    ceil = tether_limits(h.sp, True).ceiling_m
    out = h.run(3, h_m=ceil + 0.05)
    assert out.phase == LAND and out.reason == lp.R_V_CEILING


def test_vertical_controller_power_ceiling():
    vp = VerticalParams.from_params(P, mass_kg=1.4, t_max_n=30.3)
    vc = VerticalController(vp)
    for _ in range(vp.ramp_total_ticks + 5):
        out = vc.update(VerticalInputs(phase=HOLD, h_tgt_m=5.0, baro_lost=False, h_m=0.0, vz_mps=0.0, cos_tilt=1.0,
                                       u_cap=0.62))
    assert out.u == pytest.approx(0.62) and out.sat == 1
    vc2 = VerticalController(vp)
    for _ in range(vp.ramp_total_ticks + 5):
        out2 = vc2.update(VerticalInputs(phase=HOLD, h_tgt_m=5.0, baro_lost=False, h_m=0.0, vz_mps=0.0, cos_tilt=1.0))
    assert out2.u > 0.62


def test_power_telem_flags_and_build_bits():
    h = _H("wired_buffer").to_hold()
    out = h.tick(buf_i_pos=True)
    assert out.pwr.build == pl.PWR_WIRED_BUFFER and out.pwr.flag(pl.PF_TETHER_OK) and out.pwr.flag(pl.PF_BUF_SHARE)
    assert out.pwr.v_aux_v == pytest.approx(22.8, abs=0.1)
    assert _H("auto", v_in=24.0).tick().pwr.build == pl.PWR_BATTERY


def test_monitor_filter_is_integer_and_converges():
    sp = sp_of("wired")
    pm = PowerMonitor(sp)
    pm.sample(OnboardInputs(tick=0, v_in=25.2), pl.PWR_WIRED, False)
    assert pm.y_in == lp.round_half_away(25.2 / sp.pw_v_per_count) << 4
    for _ in range(80):
        pm.sample(OnboardInputs(tick=0, v_in=19.0), pl.PWR_WIRED, False)
    assert isinstance(pm.y_in, int) and abs(pm.y_in - (lp.round_half_away(19.0 / sp.pw_v_per_count) << 4)) < 16


# --------------------------------------------------------------------------- #
# ground station
# --------------------------------------------------------------------------- #
def test_ground_texts_for_every_power_code_and_the_status_line():
    for c in pl.POWER_INTERLOCK_REASONS + pl.POWER_REJECT_REASONS:
        assert c in REASON_TEXT_KO and any("가" <= ch <= "힣" for ch in REASON_TEXT_KO[c])
    assert set(POWER_ALARM_KO) == set(pl.POWER_INTERLOCK_REASONS)
    t = lp.Telemetry.from_physical(state=lp.STATE_POWERED, flags=0, reason=pl.R_W_LOST, ack_seq=1, nonce=1, t_ms=1000,
                                   h_m=1.5, vz_mps=0.0, roll_deg=0, pitch_deg=0, yaw_deg=0, tilt_deg=0, throttle=0.6,
                                   vcell_v=3.6, loop_max_us=0, phase=LAND, h_tgt_m=0.0)
    line = power_status_ko(pl.PWR_WIRED_BUFFER, t)
    assert "버스 21.6 V" in line and POWER_ALARM_KO[pl.R_W_LOST] in line and "TELEM v4" in line
    assert "전선 21.6 V" in power_status_ko(pl.PWR_WIRED, t)
    pt = pl.PowerTelem.from_physical(21.6, pl.PF_LOST | pl.PF_BUF_SHARE, pl.PWR_WIRED_BUFFER)
    line4 = power_status_ko(pl.PWR_WIRED_BUFFER, t, pt)
    assert "버퍼 3.60 V/셀" in line4 and "전선 끊김" in line4 and "TELEM v4" not in line4
    assert power_status_ko(pl.PWR_BATTERY, t) == ""
    assert power_on_window_ok(pl.PWR_WIRED, t, 24.6, 25.6) is False
    assert power_on_window_ok(pl.PWR_WIRED_BUFFER, t, 24.6, 25.6) is True


def test_ground_logic_logs_the_power_alarm():
    gp = GroundParams.from_params(P, reply_timeout_s=0.5, t_link_default_s=0.4, mode=lp.MODE_VTOL)
    gs = GroundAssist(gp)
    for i, (reason, ph) in enumerate(((0, HOLD), (pl.R_W_SAG, LAND))):
        t = lp.Telemetry.from_physical(state=lp.STATE_POWERED, flags=0, reason=reason, ack_seq=0, nonce=1,
                                       t_ms=1000 * (i + 1), h_m=1.5, vz_mps=0.0, roll_deg=0, pitch_deg=0, yaw_deg=0,
                                       tilt_deg=0, throttle=0.6, vcell_v=3.3, loop_max_us=0, phase=ph, h_tgt_m=1.5)
        gs.receive(lp.encode_telemetry(gp.vehicle_id, 0, t), float(i))
    assert gs.power_alarm == pl.R_W_SAG and any(e.startswith("POWER ALARM") for _, e in gs.events)


# --------------------------------------------------------------------------- #
# closed loop (6-DOF): one scenario per rule family
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("set_name, name", [("final", "P_H_cut_2m"), ("tune", "T_W_sag_fast"), ("final", "P_W_low_vin_at_on"),
                                            ("final", "P_W_hover")])
def test_power_rule_scenarios_closed_loop(set_name, name):
    """W_LOST, W_SAG, W_ARM and a healthy tether flight in the 6-DOF (the W_SAG regression uses the tuning set's sag:
    the final P_W_supply_sag flight is lost to an I3 yaw spin in the hover before its sag at seed 314 - a control
    finding reported with the round's evidence, not hidden by another seed)."""
    from sim import tether_scenarios as ts
    rec = ts.run_one(name, set_name=set_name)
    assert rec["verdict"] == "PASS", rec["why"]
