"""v0.9 power / tether rule scenarios (power round): one named scenario per onboard power rule and per tether hazard, with
its expected outcome and the criteria it is judged on - the evidence for ``sim.onboard`` "v0.9 power rules".

    python -m sim.tether_scenarios [--only N1,N2] [--out DIR] [--export] [--set final|tune]

Builds on ``sim.vtol_scenarios`` (``scenario_tether``: the demo build profile, the overhead rig of params
power.tether_plant_ASSUMED, ``tether_checks``, ``judge``) and the v0.9 plant (``sim.tether``).  Two sets:

* ``final`` (``SCENARIOS``) - flown ONCE on the frozen tree at the end of the round (the named evidence);
* ``tune`` (``TUNE_SCENARIOS``) - the same hazards with other event times / heights / magnitudes and seeds, the only ones
  the rule thresholds were tuned on (with the campaign's TUNE seed 7400 for false triggers on healthy flights).

Criteria (``power_checks``; ``POWER_CRITERIA`` - the proposal's sim.scenarios W1-W6 and firmware rules, never loosened):

* the expected rule acted (``expect``, the interlock names) and no other power rule (``forbid``);
* ``no_power_rule``: healthy flights - no power-rule code (0x30-0x3F) latched and no W_SAG warning;
* ``land_after_loss``: W_LOST - the LAND within ``W_LAND_AFTER_LOSS_S`` (0.2 s: proposal W2 0.1 s after V_tether falls,
  + the 40 ms persistence + the filter + one tick) of the tether event;
* ``touchdown_le_1``: the powered landing after a power event touches down at <= 1.0 m/s (proposal W2);
* ``buffer``: the buffer >= 3.4 V/cell under load (W2 / W_BUF);
* ``land_before_trip``: W_SAG - the LAND (or the motor stop before the take-off) comes before any supply latch-off (W4:
  "lands before the 5 s latch"); ``no_trip_airborne``: a latch-off, if any, finds the vehicle on the ground;
* ``arrest`` / ``contact`` / ``in_flight`` (battery-less power loss, W3): catch-line max <= the 300 N proof load, every
  ground contact after the loss <= 1.0 m/s, the event came while the vehicle flew;
* ``on_refused``: W_ARM - ON refused with REJ_POWER, the motor never started;
* ``alt_limited``: tethered limits - no target above alt_max, at least one UP refused (REJ_ALT_LIMIT), no V3 / V4;
* ``no_abort``: snag / slack / pushes - no ABORT.

Known limit, stated (``P_W_cc_short_INFO``): a supply that reaches its current limit on a SHORT route keeps the bus near
V_set - I_lim R_loop (21-22 V on the 8.5 m route), above every threshold a healthy flight on a long route can also
reach (TUNE healthy minimum 20.57 V): the vehicle cannot tell it from a heavy load and the supply latches off after 5 s
(battery-less: the catch line arrests the fall).  Protection: size the supply on the bench (procedure: hover current
<= 0.7 x the current limit) or fly the hybrid.

Invariant: the scenarios use only the plant's events and parameters (``PowerEvent``, ``PowerConfig``, ``TetherConfig``);
the vehicle side is the unmodified ``FcuStack``; every verdict comes from logged numbers.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import replace
from typing import Any

try:
    from sim import vtol_scenarios as vs
    from sim.sim6dof import Push
    from sim.tether import PowerConfig, PowerEvent, TetherConfig
except ImportError:  # pragma: no cover
    import vtol_scenarios as vs  # type: ignore
    from sim6dof import Push  # type: ignore
    from tether import PowerConfig, PowerEvent, TetherConfig  # type: ignore

EV_T = vs.TETHER_EVENT_T                # 13 s: power events in HOLD (the hover is reached at ~6-8 s)
T_ON = vs.T_ON                          # the tether script's ON
UNPLUG_CLIMB_AFTER_ON_S = 4.0           # fix round: ON + spool + ramp -> the lift-off ~1 s before this (in the climb, before
                                        # the take-off check passes; P_H_unplug_climb checks event_in_flight)
W_LAND_AFTER_LOSS_S = vs.W_LAND_AFTER_LOSS_S
TD_MAX_MPS = 1.0                        # proposal W2: touchdown <= 1.0 m/s after a power event
POWER_CRITERIA = {"land_after_loss_s": W_LAND_AFTER_LOSS_S, "touchdown_max_mps": TD_MAX_MPS,
                  "buffer_min_cell_v": vs.W_BUFFER_MIN_CELL_V, "arrest_max_n": vs.W_PROOF_N,
                  "contact_max_mps": vs.W_CONTACT_MAX_MPS}
POWER_CODES = tuple(range(0x30, 0x40))


# --------------------------------------------------------------------------- #
# scenario builders
# --------------------------------------------------------------------------- #
def _tc(p: dict, **kw: Any) -> TetherConfig:
    return TetherConfig.from_params(p, **kw)


def _pc(p: dict, source: str, **kw: Any) -> PowerConfig:
    return PowerConfig.from_params(p, source=source, **kw)


def _w(p, vg, name, title, **kw):
    """A battery-less tether test (demo build profile, indoors without the GPS unless ``indoor=False``)."""
    return vs.scenario_tether(p, vg, name, title, source="tether", **kw)


def _h(p, vg, name, title, **kw):
    """A hybrid demo flight (tether + buffer)."""
    return vs.scenario_tether(p, vg, name, title, source="hybrid", **kw)


def _loss(t_end: float = EV_T + 8.0) -> dict:
    return {"t_off": None, "t_end": t_end}


def scenarios(p: dict, vg: dict | None) -> list[tuple[Any, dict[str, Any]]]:
    """The final set (module docstring): (Scenario, expectation)."""
    out: list[tuple[Any, dict[str, Any]]] = []
    tc = _tc(p)
    # ---- healthy tethered flights: no power rule may act (false triggers) ----
    out.append((_w(p, vg, "P_W_hover", "battery-less tether test: constant-tension feed, 1.5 m hover, OFF -> landing "
                   "(healthy: no power rule)", seed=301),
                {"expect": "", "outcome": "landed", "power": {"no_power_rule": True},
                 "title_ko": "유선(배터리 없음) 정상 호버링 1.5 m: 전원 규칙이 잘못 작동하면 안 됨"}))
    # fix round (physics review medium 4): the slack feed at the procedure's limit 0.25 m (1.0 m hung the loop at the
    # intake windows); the loop must stay clear of the windows (v08 check cable_clear_of_intake)
    out.append((_w(p, vg, "P_W_slack_hover", "battery-less: slack feed 0.25 m paid out (the procedure's limit; the cable "
                   "hangs as a catenary, its loop clear of the intake windows)",
                   seed=302, tether=replace(tc, feed="slack", cable_slack_m=0.25)),
                {"expect": "", "outcome": "landed", "power": {"no_power_rule": True, "no_abort": True},
                 "title_ko": "유선: 케이블을 0.25 m(절차 한계) 느슨하게 늘어뜨린 상태로 호버링 - 고리가 흡입창에 닿지 않음"}))
    out.append((_w(p, vg, "P_W_long_route_hover", "battery-less, healthy but heavy: weak fan (P2) at 2.14 kgf on a route "
                   "x1.5 (temperature / connector) - the bus sags to ~20.6 V in the climb, no rule may act", seed=303,
                   t_max_kgf=vs.T_ENV_LOW_KGF, fan_power_model=1.0, power=_pc(p, "tether", r_loop_ohm=0.0595 * 1.5)),
                {"expect": "", "outcome": "landed", "power": {"no_power_rule": True},
                 "title_ko": "유선: 약한 팬 + 긴 전선(저항 1.5배) - 전압이 낮아져도 규칙이 잘못 작동하지 않아야 함"}))
    out.append((_w(p, vg, "P_W_alt_limit", "battery-less: UP x3 from 1.5 m - the tethered alt_max 2.0 m refuses the 2nd "
                   "and 3rd UP (REJ_ALT_LIMIT), no V3 / V4", seed=304, alt="3.0"),
                {"expect": "", "forbid": "V3|V4", "outcome": "landed", "power": {"alt_limited": True},
                 "title_ko": "유선: UP 3번 - 최고 고도 2.0 m에서 더 올라가지 않음(거부)"}))
    out.append((_h(p, vg, "P_H_hover", "hybrid demo: 1.5 m hover, OFF -> landing (healthy, the buffer idle)", seed=305),
                {"expect": "", "outcome": "landed", "power": {"no_power_rule": True},
                 "title_ko": "하이브리드 정상 호버링 1.5 m: 전원 규칙이 잘못 작동하면 안 됨"}))
    out.append((_h(p, vg, "P_H_crossover", "hybrid, weak fan (P2) at 2.14 kgf: the hover current is above the 40 A "
                   "crossover - the buffer shares the load (no W_LOST: V_tether = V_buffer while both conduct)", seed=306,
                   t_max_kgf=vs.T_ENV_LOW_KGF, fan_power_model=1.0),
                {"expect": "", "outcome": "landed", "power": {"no_power_rule": True, "buffer": True},
                 "title_ko": "하이브리드: 약한 팬 - 호버 중 버퍼가 같이 방전(교차 전류 초과), 규칙 오작동 없음"}))
    # ---- W_ARM: no climb when the supply / buffer is not ready ----
    out.append((_w(p, vg, "P_W_low_vin_at_on", "battery-less: the supply set to 23.0 V (below the ON window 24.6-25.6 V): "
                   "ON refused (W_ARM, REJ_POWER) - never climbs on a weak supply", seed=307, t_end=12.0,
                   power_events=(PowerEvent(0.0, "sag", 23.0, 0.0),)),
                {"expect": "", "outcome": "no_flight", "reject": "power", "power": {"on_refused": True},
                 "title_ko": "유선: 전원 전압 23.0 V(낮음) - 이륙 명령 거부"}))
    out.append((_h(p, vg, "P_H_buffer_low_at_on", "hybrid: buffer at 3.60 V/cell rest (below the 3.72 V/cell ON window): "
                   "ON refused (W_ARM)", seed=308, t_end=12.0, power=_pc(p, "hybrid", buf_v_rest_cell=3.60)),
                {"expect": "", "outcome": "no_flight", "reject": "power", "power": {"on_refused": True},
                 "title_ko": "하이브리드: 버퍼 배터리 3.60 V/셀(부족) - 이륙 명령 거부"}))
    # ---- W_LOST: the tether lost in a hybrid flight -> LAND on the buffer ----
    out.append((_h(p, vg, "P_H_cut_2m", f"hybrid: the tether is cut at {EV_T:.0f} s in a 2.0 m hover (UP once): W_LOST "
                   "lands on the buffer at once", seed=309, alt="2.0", t_off=None, t_end=EV_T + 20.0,
                   power_events=(PowerEvent(EV_T, "cut"),)),
                {"expect": "W_LOST", "forbid": "I2|I3|V5", "outcome": "landed",
                 "tether": {"in_flight": True, "land_after_loss": True, "buffer": True}, "power": {"touchdown_le_1": True},
                 "title_ko": "하이브리드: 2.0 m에서 전선 절단 → 즉시 버퍼로 착륙 (W_LOST)"}))
    out.append((_h(p, vg, "P_H_unplug_1m", f"hybrid: the XT90-S pulls out at {EV_T:.0f} s in a 1.0 m hover (DOWN once): "
                   "W_LOST", seed=310, alt="1.0", t_off=None, t_end=EV_T + 20.0,
                   power_events=(PowerEvent(EV_T, "unplug"),)),
                {"expect": "W_LOST", "forbid": "I2|I3|V5", "outcome": "landed",
                 "tether": {"in_flight": True, "land_after_loss": True, "buffer": True}, "power": {"touchdown_le_1": True},
                 "title_ko": "하이브리드: 1.0 m에서 커넥터 빠짐 → 즉시 버퍼로 착륙 (W_LOST)"}))
    out.append((_h(p, vg, "P_H_supply_trip", f"hybrid: the supply trips (E-stop) at {EV_T:.0f} s in the 1.5 m hover: W_LOST",
                   seed=311, t_off=None, t_end=EV_T + 20.0, power_events=(PowerEvent(EV_T, "supply_trip"),)),
                {"expect": "W_LOST", "forbid": "I2|I3|V5", "outcome": "landed",
                 "tether": {"in_flight": True, "land_after_loss": True, "buffer": True}, "power": {"touchdown_le_1": True},
                 "title_ko": "하이브리드: 전원 장치 비상정지 → 버퍼로 착륙 (W_LOST)"}))
    out.append((_h(p, vg, "P_H_supply_sag", f"hybrid: the supply set point sags 25.2 -> 21.0 V over 4 s from {EV_T:.0f} s: "
                   "D1 blocks, the buffer carries the vehicle (V_tether below V_buffer - 0.5 V): W_LOST", seed=312,
                   t_off=None, t_end=EV_T + 22.0, power_events=(PowerEvent(EV_T, "sag", 21.0, 4.0),)),
                {"expect": "W_LOST", "forbid": "I2|I3|V5", "outcome": "landed", "tether": {"buffer": True},
                 "power": {"touchdown_le_1": True},
                 "title_ko": "하이브리드: 전원 전압이 21 V로 떨어짐 → 버퍼가 받음 → 착륙 (W_LOST)"}))
    # fix round (control review HIGH 1): the tether lost in the climb, before the take-off check passes - the rule must
    # LAND on the buffer (the v0.9 tether round stopped the motor in the air: SAFE before the take-off check)
    out.append((_h(p, vg, "P_H_unplug_climb", "hybrid: the XT90-S pulls out 1.0 s after the lift-off, in the climb to 1.5 m "
                   "(before the take-off check passes): W_LOST must LAND on the buffer, never stop the motor in the air",
                   seed=327, t_off=None, t_end=T_ON + 25.0,
                   power_events=(PowerEvent(T_ON + UNPLUG_CLIMB_AFTER_ON_S, "unplug"),)),
                {"expect": "W_LOST", "forbid": "I2|I3|V5", "outcome": "landed", "tether": {"buffer": True},
                 "power": {"touchdown_le_1": True, "no_safe_in_air": True},
                 "title_ko": "하이브리드: 이륙 직후(상승 중) 커넥터 빠짐 → 공중에서 모터를 끄지 않고 버퍼로 착륙 (W_LOST)"}))
    # ---- W_BUF: the buffer low during a tethered flight ----
    out.append((_h(p, vg, "P_H_buffer_drain", "hybrid on a long route (x1.6) with a weak fan (P2, 2.14 kgf) and the buffer at "
                   "3.75 V/cell (just inside the ON window): the hover current is far above the crossover - the buffer carries most "
                   "of the hover and drains; W_BUF (landing reserve 3.6 V/cell) lands while the tether still powers it",
                   seed=313, t_max_kgf=vs.T_ENV_LOW_KGF, fan_power_model=1.0, t_off=None, t_end=70.0,
                   power=_pc(p, "hybrid", r_loop_ohm=0.0595 * 1.6, buf_v_rest_cell=3.75)),   # fix round: 3.72 sat on the
                                                                                         # ON window edge (V7 refused)
                {"expect": "W_BUF", "forbid": "I2|I3|V5|W_LOST", "outcome": "landed", "tether": {"buffer": True},
                 "power": {"touchdown_le_1": True},
                 "title_ko": "하이브리드: 긴 전선 + 약한 팬 - 호버 중 버퍼가 계속 방전 → 착륙 예비량에서 착륙 (W_BUF)"}))
    # ---- W_SAG: the battery-less supply sags / current-limits ----
    out.append((_w(p, vg, "P_W_supply_sag", f"battery-less: the supply set point sags 25.2 -> 21.0 V over 4 s from "
                   f"{EV_T:.0f} s (failing supply / mains dip): W_SAG freezes the throttle ceiling, then lands", seed=314,
                   t_off=None, t_end=EV_T + 20.0, power_events=(PowerEvent(EV_T, "sag", 21.0, 4.0),)),
                {"expect": "W_SAG", "forbid": "I2|I3|V5", "outcome": "landed", "power": {"touchdown_le_1": True},
                 "title_ko": "유선: 전원 전압이 21 V로 떨어짐 → 추력 상한 고정 후 착륙 (W_SAG)"}))
    out.append((_w(p, vg, "P_W_cc_long", "battery-less, current limit on a long thin route: a 45 A supply (ASSUMED small "
                   "unit) on the route x2.0 with a weak fan (P2, 2.14 kgf), UP to 2.0 m - the climb pulls the supply into "
                   "its limit, the bus collapses: W_SAG must land before the 5 s latch", seed=315,
                   t_max_kgf=vs.T_ENV_LOW_KGF, fan_power_model=1.0, alt="2.0",
                   power=_pc(p, "tether", i_limit_a=45.0, r_loop_ohm=0.0595 * 2.0)),
                {"expect": "W_SAG|V15", "forbid": "I2|I3|V5", "outcome": "any",
                 "power": {"land_before_trip": True, "no_trip_airborne": True, "no_abort": True},
                 "title_ko": "유선: 작은 전원 + 긴 전선 - 전류 제한, 전압 붕괴 → 차단 전에 착륙 (W_SAG)"}))
    out.append((_w(p, vg, "P_W_cc_short_INFO", "KNOWN LIMIT: a 40 A supply on the nominal 8.5 m route with a weak fan (P2, "
                   "2.14 kgf): the supply current-limits but the bus stays near 22.8 V - no onboard rule can see it; "
                   "the supply latches off (5 s) and the catch line arrests the fall", seed=316,
                   t_max_kgf=vs.T_ENV_LOW_KGF, fan_power_model=1.0, alt="2.0", t_off=None, t_end=40.0,
                   power=_pc(p, "tether", i_limit_a=40.0)),
                {"expect": "", "outcome": "any", "info": True, "tether": {"arrest": True, "contact": True},
                 "title_ko": "알려진 한계: 짧은 전선 + 작은 전원 → 전류 제한을 기체가 알 수 없음 → 차단 후 안전줄로 잡힘"}))
    # ---- battery-less power loss: no power, the catch line + bungee ----
    for alt, nm, ev in (("1.0", "P_W_unplug_1m", "unplug"), ("2.0", "P_W_unplug_2m", "unplug")):
        out.append((_w(p, vg, nm, f"battery-less: the XT90-S pulls out at {EV_T:.0f} s in a {alt} m hover: no power - the "
                       "vehicle falls into the catch line + bungee (W3)", seed=317 if alt == "1.0" else 318, alt=alt,
                       power_events=(PowerEvent(EV_T, ev),), **_loss()),
                    {"expect": "", "outcome": "any", "tether": {"in_flight": True, "arrest": True, "contact": True},
                     "title_ko": f"유선(배터리 없음): {alt} m에서 커넥터 빠짐 → 낙하, 안전줄+번지로 잡힘"}))
    for slack, nm in ((0.1, "P_W_cut_2m_slack0.1"), (0.3, "P_W_cut_2m_slack0.3")):
        # fix round: the slack cases fly the auto-locking belay (the SRL keeps no slack; P_W_unplug_* fly the SRL)
        out.append((_w(p, vg, nm, f"battery-less: the conductors are cut at 2.0 m, auto-locking belay with {slack} m of "
                       "slack (proposal W3)",
                       seed=319 if slack < 0.2 else 320, alt="2.0",
                       tether=replace(tc, belay_device="autolock", belay_slack_m=slack),
                       power_events=(PowerEvent(EV_T, "cut"),), **_loss()),
                    {"expect": "", "outcome": "any", "tether": {"in_flight": True, "arrest": True, "contact": True},
                     "title_ko": f"유선: 2.0 m에서 전선 절단, 안전줄 여유 {slack} m → 낙하 하중과 착지 속도"}))
    out.append((_w(p, vg, "P_W_supply_trip", f"battery-less: the supply trips (E-stop) at {EV_T:.0f} s in the 1.5 m hover",
                   seed=321, power_events=(PowerEvent(EV_T, "supply_trip"),), **_loss()),
                {"expect": "", "outcome": "any", "tether": {"in_flight": True, "arrest": True, "contact": True},
                 "title_ko": "유선: 전원 장치 비상정지 → 낙하, 안전줄로 잡힘"}))
    # ---- mechanical tether disturbances ----
    out.append((_w(p, vg, "P_W_snag_jerk", "battery-less: a snag / handler jerk 20 N for 0.2 s at 30 deg from the vertical "
                   "in the 1.5 m hover (proposal W5)", seed=322, tether=replace(tc, snag_t_s=EV_T, snag_azimuth_deg=45.0)),
                {"expect": "", "forbid": "I2|I3", "outcome": "landed", "power": {"no_abort": True, "no_power_rule": True},
                 "title_ko": "유선: 줄 걸림/당김 20 N 0.2초 (W5)"}))
    out.append((_w(p, vg, "P_W_snag_pull", "battery-less: the cable catches and pulls sideways 8 N for 2 s at 60 deg from "
                   "the vertical (a snag the handler frees slowly)", seed=323,
                   tether=replace(tc, snag_t_s=EV_T, snag_force_n=8.0, snag_duration_s=2.0, snag_angle_deg=60.0,
                                  snag_azimuth_deg=200.0)),
                {"expect": "", "forbid": "I2|I3", "outcome": "landed", "power": {"no_abort": True, "no_power_rule": True},
                 "title_ko": "유선: 케이블이 걸려 옆으로 8 N, 2초 당김"}))
    out.append((_w(p, vg, "P_W_gps_loss", "battery-less outdoors with the GPS: the GPS is lost at 13 s - V11 lands (the "
                   "tether keeps powering, no power rule)", seed=324, indoor=False, t_off=None, t_end=EV_T + 25.0,
                   gps=replace(vs.GPS_DEFAULT, outages=((EV_T, 1e9),))),
                {"expect": "V11", "outcome": "landed", "power": {"no_power_rule": True},
                 "title_ko": "유선(실외, GPS): GPS 상실 → 착륙 (V11), 전원은 정상"}))
    # fix round (control review medium 5): the I3-safe push is about 3 N x 0.3 s at 0.40-0.46 m (5 N aborted 5-7 of 7)
    out.append((_w(p, vg, "P_W_push", "battery-less: three light pole pushes (3 N, 0.3 s, 0.40 m - inside the measured "
                   "I3-safe envelope) in the 1.5 m hover", seed=325,
                   pushes=tuple(Push(t0, 0.3, 3.0, azimuth_deg=az, z_body_m=0.40) for t0, az in
                                ((10.0, 0.0), (14.0, 120.0), (18.0, 240.0)))),
                {"expect": "", "forbid": "I2|I3", "outcome": "landed", "power": {"no_abort": True, "no_power_rule": True},
                 "title_ko": "유선: 가벼운 밀기 3번(3 N, 0.3초) - 비상정지 없이 유지"}))
    # ---- battery (unchanged rules, the end of the pack at the low thrust envelope) ----
    out.append((vs.base_vtol("P_B_end_of_pack", "battery VTOL, demo profile, pack at 3.58 V/cell rest and the plant at the "
                             "envelope's low end 2.14 kgf: the sag under load reaches I8 (3.3 V/cell) - LAND, unchanged rule",
                             vg, seed=326, profile="demo", t_max_kgf=vs.T_ENV_LOW_KGF, t_end=40.0, gps=None, nav_v08=True,
                             takeoff_check=True, options=replace(vs.SimOptions(), batt_v_full_cell=3.58),
                             vtol_script=(vs.OpStep("on", t=vs.T_ON),)),
                {"expect": "I8", "forbid": "I2|I3|V5", "outcome": "landed", "power": {"touchdown_le_1": True},
                 "title_ko": "배터리: 팩 말기 + 추력 하한 → 셀 전압 저하로 착륙 (I8, 기존 규칙)"}))
    return out


def tune_scenarios(p: dict, vg: dict | None) -> list[tuple[Any, dict[str, Any]]]:
    """The tuning set: the rule hazards with other times, heights, magnitudes and seeds (module docstring)."""
    out: list[tuple[Any, dict[str, Any]]] = []
    t1 = 11.0
    out.append((_h(p, vg, "T_H_cut_1p5", "tune: hybrid cut at 11 s in the 1.5 m hover", seed=401, t_off=None, t_end=t1 + 20.0,
                   power_events=(PowerEvent(t1, "cut"),)),
                {"expect": "W_LOST", "outcome": "landed", "tether": {"in_flight": True, "land_after_loss": True, "buffer": True},
                 "power": {"touchdown_le_1": True}}))
    out.append((_h(p, vg, "T_H_sag_slow", "tune: hybrid supply sags 25.2 -> 22.0 V over 8 s (still above the buffer under "
                   "load: a crossover, not a loss - no W_LOST)", seed=402, t_off=None, t_end=t1 + 25.0,
                   power_events=(PowerEvent(t1, "sag", 22.0, 8.0),)),
                {"expect": "", "forbid": "W_LOST", "outcome": "any", "info": True, "power": {}}))
    out.append((_h(p, vg, "T_H_buffer_drain", "tune: hybrid, route x1.4, P2 at 2.3 kgf, buffer 400 mAh at 3.80 V/cell",
                   seed=403, t_max_kgf=2.3, fan_power_model=1.0, t_off=None, t_end=90.0,
                   power=_pc(p, "hybrid", r_loop_ohm=0.0595 * 1.4, buf_capacity_mah=400.0, buf_v_rest_cell=3.80)),
                {"expect": "W_BUF", "outcome": "landed", "tether": {"buffer": True}, "power": {"touchdown_le_1": True}}))
    out.append((_w(p, vg, "T_W_sag_fast", "tune: battery-less supply sags 25.2 -> 20.0 V over 2 s", seed=404, t_off=None,
                   t_end=t1 + 20.0, power_events=(PowerEvent(t1, "sag", 20.0, 2.0),)),
                {"expect": "W_SAG", "outcome": "landed", "power": {"touchdown_le_1": True}}))
    out.append((_w(p, vg, "T_W_sag_shallow", "tune: battery-less supply sags 25.2 -> 22.0 V (bus ~20.3 V in the hover)",
                   seed=405, t_off=None, t_end=t1 + 20.0, power_events=(PowerEvent(t1, "sag", 22.0, 3.0),)),
                {"expect": "", "outcome": "any", "info": True, "power": {}}))
    out.append((_w(p, vg, "T_W_cc_long", "tune: 50 A supply, route x1.8, weak fan, UP to 2.0", seed=406, t_max_kgf=2.2,
                   fan_power_model=1.0, alt="2.0", power=_pc(p, "tether", i_limit_a=50.0, r_loop_ohm=0.0595 * 1.8)),
                {"expect": "W_SAG|V15", "outcome": "any", "power": {"land_before_trip": True, "no_trip_airborne": True,
                                                                     "no_abort": True}}))
    out.append((_w(p, vg, "T_W_long_heavy", "tune: healthy heavy load, route x1.5, P2 at 2.2 kgf, UP to 2.0", seed=407,
                   t_max_kgf=2.2, fan_power_model=1.0, alt="2.0", power=_pc(p, "tether", r_loop_ohm=0.0595 * 1.5)),
                {"expect": "", "outcome": "landed", "power": {"no_power_rule": True}}))
    out.append((_w(p, vg, "T_W_unplug_1p5", "tune: battery-less unplug in the 1.5 m hover", seed=408,
                   power_events=(PowerEvent(t1, "unplug"),), t_off=None, t_end=t1 + 8.0),
                {"expect": "", "outcome": "any", "tether": {"in_flight": True, "arrest": True, "contact": True}}))
    return out


SCENARIO_SETS = {"final": scenarios, "tune": tune_scenarios}


# --------------------------------------------------------------------------- #
# metrics / checks / verdict
# --------------------------------------------------------------------------- #
def power_metrics(res: Any, m: dict[str, Any]) -> dict[str, Any]:
    """The power-rule bookkeeping of a run (from the log and the events): power codes latched, W_SAG warnings, the LAND
    and its rule, the targets flown, UP refusals, the first touchdown speed after the first power event."""
    import numpy as np
    L = res.log
    out: dict[str, Any] = {}
    out["power_codes"] = sorted({int(c) for c in np.unique(L["reason"]) if int(c) in POWER_CODES})
    ev = res.events
    out["w_sag_warnings"] = sum(1 for _, e in ev if "W_SAG warning:" in e)
    out["t_w_sag_warning_s"] = next((float(t) for t, e in ev if "W_SAG warning:" in e), None)
    tg = L["h_tgt"][np.isfinite(L["h_tgt"])] if "h_tgt" in L else np.zeros(0)
    out["target_max_m"] = float(tg.max()) if tg.size else None
    out["up_refused_alt"] = sum(1 for r in m.get("rejections", []) if r["cmd"] == "UP" and "altitude" in r["reason"])
    out["on_refused_power"] = any(r["cmd"] == "ON" and "power" in r["reason"] for r in m.get("rejections", []))
    tl = m.get("t_power_loss_s")
    if tl is None:
        tl = next((float(t) for t, e in ev if e.startswith("POWER ")), None)       # the plant's events ("POWERED:" is not one)
    out["t_power_event_s"] = tl
    tds = (res.vtol or {}).get("touchdowns", [])
    after = [d for d in tds if tl is not None and d["t"] >= tl]
    out["touchdown_after_event_mps"] = (max(abs(d["vz"]) for d in after[:1]) if after else None)
    out["h_max_m"] = m.get("max_h_m")
    tt = m.get("t_supply_trip_s")
    if tt is not None and "h_true" in L:
        k = int(np.searchsorted(L["t"], tt))
        out["h_at_trip_m"] = float(L["h_true"][min(k, len(L["t"]) - 1)])
    else:
        out["h_at_trip_m"] = None
    # the motor stopped by the vehicle itself: LANDED, or SAFE from POWERED (V15) - the first such transition
    out["t_motor_stop_s"] = next((float(t) for t, e in ev if e.startswith("POWERED -> UNPOWERED") or
                                  e.startswith("POWERED -> SAFE") or e.startswith("POWERED -> ABORT")), None)
    # fix round (control review HIGH 1): a POWERED -> SAFE with the vehicle off the ground (true height > 5 cm)
    ts = next((float(t) for t, e in ev if e.startswith("POWERED -> SAFE")), None)
    out["safe_in_air"] = False
    if ts is not None and "h_true" in L:
        k = min(int(np.searchsorted(L["t"], ts)), len(L["t"]) - 1)
        out["h_at_safe_m"] = float(L["h_true"][k])
        out["safe_in_air"] = out["h_at_safe_m"] > 0.05
    return out


def power_checks(m: dict[str, Any], pm: dict[str, Any], want: dict[str, Any]) -> dict[str, bool | None]:
    """The power criteria of the module docstring (True / False; None = not applicable)."""
    out: dict[str, bool | None] = {}
    if want.get("no_power_rule"):
        out["no_power_rule"] = not pm["power_codes"] and pm["w_sag_warnings"] == 0
    if want.get("touchdown_le_1"):
        x = pm.get("touchdown_after_event_mps")
        if x is None:
            x = abs(m["touchdown_vz"]) if m.get("touchdown_vz") is not None else None
        out["touchdown_le_1"] = x is not None and x <= TD_MAX_MPS
    if want.get("land_before_trip"):
        # W4: the LAND (or the V15 motor stop on the pad) comes before any supply latch-off
        tt, tl, ts = m.get("t_supply_trip_s"), m.get("t_land_s"), pm.get("t_motor_stop_s")
        out["land_before_trip"] = tt is None or (tl is not None and tl < tt) or (ts is not None and ts < tt)
    if want.get("no_trip_airborne"):
        # W4: if the supply latches off at all, the vehicle is already on the ground then
        h = pm.get("h_at_trip_m")
        out["no_trip_airborne"] = m.get("t_supply_trip_s") is None or (h is not None and h <= 0.1)
    if want.get("on_refused"):
        out["on_refused"] = bool(pm["on_refused_power"]) and m.get("t_motor_start_s") is None
    if want.get("alt_limited"):
        out["alt_limited"] = (pm["target_max_m"] is not None and pm["target_max_m"] <= 2.0 + 1e-9 and pm["up_refused_alt"] >= 1)
    if want.get("no_abort"):
        out["no_abort"] = m.get("t_abort_s") is None
    if want.get("no_safe_in_air"):
        # fix round (control review HIGH 1): no power rule may stop the motor (SAFE) with the vehicle off the ground
        out["no_safe_in_air"] = not pm.get("safe_in_air")
    return out


def judge_power(m: dict[str, Any], pm: dict[str, Any], exp: dict[str, Any]) -> tuple[str, str, dict]:
    """``vs.judge`` (outcome, expect / forbid, reject, tether checks) + ``power_checks``."""
    base = {k: v for k, v in exp.items() if k not in ("power",)}
    verdict, why = vs.judge(m, base)
    pc = power_checks(m, pm, exp.get("power") or {})
    bad = [k for k, v in pc.items() if v is not None and not bool(v)]     # fix round: numpy bools are not 'is False'
    if pc:
        why += "; power: " + ", ".join(f"{k} {v}" for k, v in pc.items())
    why += (f" (power codes {[hex(c) for c in pm['power_codes']]}, W_SAG warnings {pm['w_sag_warnings']}, targets max "
            f"{pm['target_max_m']}, UP refused {pm['up_refused_alt']}, touchdown after the event "
            f"{pm['touchdown_after_event_mps']})")
    if verdict == "PASS" and bad:
        verdict = "FAIL"
    return verdict, why, pc


def run_one(name: str, vg: dict | None = None, set_name: str = "final", out_dir: str | None = None,
            export: bool = False) -> dict[str, Any]:
    """Fly scenario ``name`` of ``set_name``, judge it; returns a JSON-safe record (and writes the renderer log)."""
    p = vs.vehicle().params
    if vg is None:
        vg, _ = vs.load_vgains()
    found = [(sc, e) for sc, e in SCENARIO_SETS[set_name](p, vg) if sc.name == name]
    if not found:
        raise KeyError(f"no power scenario {name!r} in {set_name}")
    sc, exp = found[0]
    t0 = time.perf_counter()
    res, m = vs.run_scenario(sc)
    pm = power_metrics(res, m)
    verdict, why, pc = judge_power(m, pm, exp)
    keep = ("interlocks", "abort_reason", "land_by", "landed_by", "t_on_s", "t_motor_start_s", "t_land_s", "t_landed_s",
            "t_abort_s", "final_state", "touchdown_vz", "touchdown_vh", "max_h_m", "rejections", "power_source",
            "t_max_kgf_plant", "v_bus_min_powered", "i_bus_max", "supply_cc_s", "supply_tripped", "t_supply_trip_s",
            "t_power_loss_s", "land_after_power_loss_s", "contact_speed_max_after_loss_mps", "tether_catch_max_n",
            "buffer_soc_min", "buffer_cell_v_min", "buffer_a_max", "thrust_limited_s", "powered_at_power_loss",
            "h_at_power_loss_m", "power_events", "climb_overshoot_m", "max_tilt_powered_deg", "motor_time_s")
    rec = {"name": name, "set": set_name, "title": sc.title, "title_ko": exp.get("title_ko"), "expect": exp,
           "verdict": verdict, "why": why, "power_checks": pc, "power_metrics": pm,
           "metrics": {k: m.get(k) for k in keep}, "seed": sc.seed,
           "events": [(round(float(t), 3), e) for t, e in res.events if not e.startswith("GS TX")][:80],
           "runtime_s": round(time.perf_counter() - t0, 2)}
    if out_dir and export:
        veh = vs.vehicle(sc.power_source)
        vs.export_npz(res, os.path.join(out_dir, f"{name}.npz"), veh,
                      {"title_ko": exp.get("title_ko"), "expectation": exp, "verdict": verdict, "verdict_why": why,
                       "power_checks": pc, "power_metrics": pm})
        rec["npz"] = f"{name}.npz"
    return vs._clean(rec)


def names(set_name: str = "final") -> list[str]:
    p = vs.vehicle().params
    return [sc.name for sc, _ in SCENARIO_SETS[set_name](p, None)]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="v0.9 power / tether rule scenarios")
    ap.add_argument("--set", default="final", choices=tuple(SCENARIO_SETS))
    ap.add_argument("--only", default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--export", action="store_true")
    a = ap.parse_args(argv)
    todo = a.only.split(",") if a.only else names(a.set)
    vg, _ = vs.load_vgains()
    if a.out:
        os.makedirs(a.out, exist_ok=True)
    n_fail = 0
    for nm in todo:
        rec = run_one(nm, vg, a.set, a.out, a.export)
        n_fail += rec["verdict"] == "FAIL"
        print(f"{nm:24s} {rec['verdict']:5s} {rec['why'][:300]}", flush=True)
        if a.out:
            with open(os.path.join(a.out, f"{nm}.json"), "w", encoding="utf-8", newline="\n") as f:
                json.dump(rec, f, indent=1, ensure_ascii=False)
    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main())
