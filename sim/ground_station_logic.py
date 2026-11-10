"""Ground-station reference logic (SPEC §8.8 "Ground station", §10.3): link master,
command queue with repeat-until-acknowledged, link-delay estimate and the assist
mode that holds the former onboard mission rules.

Pure Python, no numpy, no I/O: ``tools/ground_station.py`` wraps it (serial port,
display, log), ``sim.sim6dof`` drives it through ``sim.link_model``.  Times are
the ground's own clock in seconds (any monotonic origin).

Link master (``poll``):
* one request (CMD frame) at a time: after sending, wait for the reply or for
  ``reply_timeout_s``; the vehicle only transmits as a reply, so with one request in
  flight the half-duplex radio never carries both directions at once;
* requests every 1 / ``link.poll_hz`` (measured from the previous request), and
  immediately when a new command is waiting and no reply is outstanding;
* every request carries the current command (pending until the telemetry's
  ack_seq equals its SEQ - repeated with the same SEQ, so the vehicle executes it
  at most once) or a NONE heartbeat that repeats the last assigned SEQ;
* nothing but heartbeats is sent before the first telemetry: the first reply's
  ack_seq synchronises the SEQ counter (new SEQ = ack_seq + 1, always newer than
  the vehicle's last processed SEQ); a pending command whose SEQ the vehicle can
  no longer accept (not newer than ack_seq and not equal to it) gets a new SEQ.

Command queue: ARM / DISARM / LAUNCH / THROTTLE are first-in first-out; CUT,
RELEASE and KILL pre-empt (the pending and queued commands are dropped as
"superseded" - a stale THROTTLE must never follow a CUT).  LAUNCH and THROTTLE carry
the nonce of the latest telemetry at the moment of each transmission.

Link delay t_link: mean of the last ``RTT_WINDOW_ASSUMED`` request -> reply times
(a reply is matched to the request whose SEQ it echoes); before the first one,
``t_link_default_s`` (the link model's nominal round trip).  The round trip is the
age of the telemetry when it arrives plus the uplink time of a command sent right
then, i.e. the delay from the measurement to the command taking effect.
2026-11-10 control round (rule R-ESP1, the ESP-NOW link): (a) a LATE reply - one that arrives after its request's reply
timeout but before the next request is sent, with the SEQ of that request - is a round-trip sample too (``stats``
"late_replies"; before, every late reply was "stale" and never measured), unless the previous request with the same
SEQ is still unanswered (a heartbeat repeats its SEQ: the reply could be that one's - not measured, "stale");
(b) ``t_link`` = max(mean of the last ``RTT_WINDOW_ASSUMED`` samples, the latest sample), so a rise of the link delay
moves CUT / RELEASE earlier at once while a fall lowers it only through the mean (the conservative direction for both
rules: an earlier CUT lowers the apogee, an earlier RELEASE opens the nose nearer apogee); (c) the reply timeout of a
request is max(``reply_timeout_s``, the longest round trip in the window + ``LATE_TIMEOUT_MARGIN_S_ASSUMED``), at most
``LATE_TIMEOUT_MAX_S_ASSUMED``: once late replies are measured the ground waits for them instead of transmitting while
the vehicle's reply is still on air (a half-duplex LoRa link timed for the ESP: the CUT / RELEASE sent at the short
timeout collided with the late reply and needed two or three exchanges).  Why: with the ESP model
(nominal round trip ~0.06 s, reply timeout ~0.25 s) a link whose delay rises above the timeout (the 2026-11-10 study:
the ESP link stepping to the LoRa model's 0.49 s; a ground station set up for the ESP on a LoRa link) answered every
request late, t_link stayed at 0.06 s, the CUT came late (I4 cut instead) and the RELEASE 0.4 - 1.2 s after apogee
(v_z -7 .. -12 m/s, S2 failed in 30 of 60 runs).  With the link model the ground is set up for, (a) and (c) change
nothing (every reply is on time, every round trip + margin is below the model's timeout) and (b) changes nothing for
the LoRa models (constant round trip); on the ESP model (b) can move the decisions earlier by a MAC-retry round trip.

Assist mode (default on; ``set_assist(False)`` = operator only), evaluated on each
new telemetry:
* auto-CUT once, in POWERED, when P + max(dP/dt, 0) T_tel / 2 >= target_altitude_m, with the
  predicted apogee P = h + v_z t_link + coast_drag_factor max(v_z, 0)^2 / (2 g), dP/dt measured
  between consecutive POWERED telemetry (vehicle time t_ms) and T_tel = max(1 / poll_hz, t_link).
  The half-interval lead centres the cut on the target: P grows by ~15 m per telemetry at
  2 Hz, so deciding on P alone overshoots by up to a whole step (SPEC 8.8 v0.4);
* auto-RELEASE once, in UNPOWERED after POWERED, when the downlinked v_z predicted to
  the moment the release servo will open, v_z - g (t_link + release_after_cut_s +
  T_tel / 2) with T_tel = max(1 / poll_hz, t_link), is <= apogee_detect_vz_mps
  (``mission.assist_release_rule`` = "predictive", SPEC 8.8 v0.4), or coast_timeout_s
  after the first UNPOWERED telemetry.  ``assist_release_rule`` = "literal" keeps the
  v0.3 rule (downlinked v_z <= apogee_detect_vz_mps on consecutive telemetry spanning
  >= apogee_confirm_s of vehicle time), which opens the nose about one link delay plus
  one telemetry interval after apogee;
* a lost exchange (reply timeout) is a decision point too (v0.6.3): the CUT rule and the
  predictive RELEASE rule are evaluated at the timeout on the latest telemetry
  extrapolated by its age a = now - its arrival time (CUT: P + dP/dt a; RELEASE: v_z - g a,
  coast time + a), so the decision the lost telemetry would have triggered is not
  postponed by a whole telemetry interval.  Without it, one lost frame near the cut moves
  the decision ~dP/dt x T_tel = ~11 m past the centred point - more than the 10 m between
  target_altitude_m and the onboard apogee limiter I4.  Practice mode, 100 seeds per radio
  at 5 % loss, v0.6.3 vehicle and link model: I4 cut first in 15 (lora_uart) / 19
  (sx127x) flights without this rule, 10 / 6 with it - nearly all of the rest lost the
  CUT exchange itself (repeated one exchange later, ~0.53 s), which no ground rule can
  undo without cutting earlier (apogee 147.6-149.5 m in every I4 flight).

VTOL build (v0.7, ``GroundParams.mode`` = ``mission.mode`` = "vtol", SPEC 8.9): the operator has
four buttons and KILL - ``on`` (take off: the vehicle spools up with a buzzer warning for
spool_warning_s, climbs to hover_altitude_m and holds it), ``off`` (land where it is / cancel the
spool-up), ``up`` / ``down`` (move the altitude target by alt_step_m), ``kill`` (emergency, ABORT).
* ON carries the nonce of the latest telemetry at each transmission (like LAUNCH): a stale or
  replayed ON cannot take off.
* OFF and KILL pre-empt (the pending and queued commands are dropped as "superseded": a stale UP
  must never follow an OFF); ON / UP / DOWN are first-in first-out, each press one command, executed
  at most once (repeat-until-acknowledged with the same SEQ).
* The other build's commands are refused locally (``command`` raises ValueError): ARM / DISARM /
  LAUNCH / THROTTLE / CUT / RELEASE in VTOL, ON / OFF / UP / DOWN in ballistic.  The vehicle would
  reject them anyway (REJ_MODE / REJ_UNKNOWN_CMD); refusing here keeps them off the air.
* Assist is off in VTOL (forced; ``set_assist(True)`` is refused with an event): the vehicle flies the
  altitude and every safety rule itself, the ballistic CUT / RELEASE rules do not apply.
* Phase and target tracking: every new TELEM v2 updates ``phase`` / ``h_tgt_m`` and logs changes as
  events ("phase CLIMB -> HOLD", "target 5.0 -> 6.0 m"); ``vtol_status()`` summarises them for a display.
* v0.7.1: a change of the vehicle's safety-rule reason (interlock codes 1..63) is logged as an event with the rule's
  name and the state / phase it left the vehicle in ("rule V10 LAND floor timeout: landed ... -> UNPOWERED / LANDED");
  ``REASON_TEXT_KO`` / ``reason_text_ko`` give the operator text in Korean for every VTOL code (the ground station UI
  shows them; codes, logs and identifiers stay English).  The new codes V8 / V9 (baro-lost time limit: landed / abort)
  and V10 (LAND floor timeout: landed, no release), and the LANDED rules (I9 IMU / V4 / V5 -> SAFE instead of ABORT),
  change nothing the operator does: the buttons are still ON / OFF / UP / DOWN and KILL.
* The decoder takes TELEM (v1) and TELEM2 (v2) frames.
* v0.8 (VTOL + GPS position hold): the decoder also takes TELEM3 (v3: + GPS flags, satellites, hAcc, position-hold
  mode, distance from the hold point).  ``vtol_status()`` adds them (``gps_ready`` = the vehicle's ON gate: press ON only
  when it is shown - otherwise REJ_GPS "GPS 준비 안 됨"), ``gps_status_ko`` gives one Korean line for the display, and
  the logic logs "GPS ready / not ready" and position-hold mode changes as events.  The buttons are unchanged (ON /
  OFF / UP / DOWN / KILL): the hold point is set by the vehicle itself at the end of the climb; there is no command to
  move it, and the pad heading is a set-up rule (the vehicle's +X toward params heading.pad_azimuth_deg), not a command.
  The new rules V11-V15 and REJ_GPS have Korean texts in ``REASON_TEXT_KO``.

* v0.9 (tether / hybrid power builds, ``sim.power_link``): the power rules' codes (W_LOST, W_SAG, W_BUF, W_D2) and
  REJ_POWER have Korean texts in ``REASON_TEXT_KO``; a power rule acting logs "POWER ALARM <banner>" (``POWER_ALARM_KO``,
  the station shows it and beeps) and sets ``power_alarm``; ``power_status_ko`` gives one Korean line for the power state
  (the bus from the vcell field - V_in / 6 in every build - and, once the firmware round sends the proposed TELEM v4 power
  tail, V_tether / V_buffer / the flags); ``power_on_window_ok`` is the ground pre-check of the battery-less ON window.
  The buttons are unchanged (ON / OFF / UP / DOWN / KILL).
* v0.9 firmware round (RP1): the decoder also takes TELEM v4 / v5 (``link_proto.TELEM_TYPES``: TELEM v2 / v3 + the
  power tail, the tether / hybrid builds); ``power_tail_of`` gives a frame's ``sim.power_link.PowerTelem``, the logic
  keeps the latest one (``ptel``, for ``power_status_ko``) and logs the first tail and every change of its flag byte as
  an event ("PWR tail ...": build, v_aux, flags).  Nothing the operator does depends on the tail (the ON pre-check
  ``power_on_window_ok`` stays on the vcell field), so a tether / hybrid run differs from its TELEM v2 / v3 twin only
  by the longer downlink frame (link timing) and these events.

* 2026-11-10 control round (logic-battery telemetry): the decoder also takes TELEM v6 / v7 (TELEM v2 / v3 + the 1-byte
  logic-battery tail of the carrier's battery build, ``link_proto.TYPE_TELEM6`` / ``TYPE_TELEM7``).  The logic keeps the
  latest reading (``lbat_v``), ``vtol_status`` shows it and ``lbat_status_ko`` gives one Korean line; below
  ``GroundParams.lbat_warn_v`` (params ``flight_computer.logic_battery_monitor.ground_warn_below_v_ASSUMED``, A) it logs
  "LOGIC BATTERY LOW" once and sets ``lbat_low`` (cleared above the threshold + ``LBAT_WARN_HYST_V``, logged again on the
  next drop).  A warning only: nothing is sent to the vehicle and the vehicle does nothing (a user decision).

Invariant: the logic never invents vehicle data - every decision uses decoded
telemetry only, and every command it sends is recorded in ``records``.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass

try:
    from sim import link_proto as lp
    from sim import power_link as pl
    from sim.state_machine import G0
except ImportError:  # pragma: no cover
    import link_proto as lp  # type: ignore
    import power_link as pl  # type: ignore
    from state_machine import G0  # type: ignore

pl.register_reason_names(lp)          # v0.9 power codes: a no-op since RP1 (sim.link_proto defines them)

RTT_WINDOW_ASSUMED = 8          # round-trip samples averaged into t_link
LBAT_WARN_HYST_V = 0.05         # 2026-11-10: the logic-battery warning clears this far above its threshold
# 2026-11-10 R-ESP1 (c): the adaptive reply timeout - margin over the longest measured round trip (= sim.link_model
# REPLY_TIMEOUT_MARGIN_S_ASSUMED) and its upper limit (with poll interval 0.5 s, one lost frame then leaves the vehicle
# <= 1.5 s without an uplink, below link_loss_disarm_s 2 s - A)
LATE_TIMEOUT_MARGIN_S_ASSUMED = 0.04
LATE_TIMEOUT_MAX_S_ASSUMED = 1.0
PRIORITY_CMDS = (lp.CMD_CUT, lp.CMD_RELEASE, lp.CMD_KILL, lp.CMD_OFF)   # pre-empt the pending / queued commands

# Operator text (Korean, user-facing) for every code the VTOL build can report; the English names stay in
# link_proto.REASON_NAMES (logs).  What happened and what the vehicle does now.
REASON_TEXT_KO = {
    lp.REASON_NONE: "정상",
    lp.R_I1P_LINK_LOSS_PAD: "I1p 패드에서 통신 끊김: 안전(SAFE)으로 복귀, 모터 시동 안 함",
    lp.R_I2_TILT: "I2 기울기 한계 초과: 비상정지(ABORT) - 모터 정지, 낙하산 사출",
    lp.R_I3_RATE: "I3 회전/자이로 포화: 비상정지(ABORT) - 모터 정지, 낙하산 사출",
    lp.R_I8_VCELL: "I8 배터리 전압 낮음: 제자리 착륙",
    lp.R_I9_IMU: "I9 자세센서(IMU/NAV) 고장: 비행 중이면 비상정지, 착륙 후·패드에서는 안전(SAFE)",
    lp.R_I9_BARO: "I9 기압계 고장: 가속도계 추측항법 고도로 제자리 착륙 (플레어 없음, 시간 제한 적용)",
    lp.R_I10_RESET: "I10 비행 중 재부팅: 비상정지(ABORT)",
    lp.R_ARM_SWITCH_OPEN: "아밍 스위치 열림: 비행 중이면 비상정지, 착륙 후에는 안전(SAFE)",
    lp.R_V_LINK_LAND: "V1 통신 끊김: 제자리 착륙",
    lp.R_V_ENDURANCE: "V2 비행시간 한계: 제자리 착륙",
    lp.R_V_CEILING: "V3 고도 상한 초과: 제자리 착륙",
    lp.R_V_GEOFENCE: "V4 고도 울타리 초과: 비행 중이면 비상정지, 착륙 후에는 안전(SAFE)",
    lp.R_V_FALL: "V5 추락 감지: 비행 중이면 비상정지, 착륙 후에는 안전(SAFE)",
    lp.R_V_HARD_CUTOFF: "V6 모터 시간 절대 한계: 비상정지(ABORT)",
    lp.R_V_TAKEOFF_GATE: "V7 이륙 직전 점검 실패(기울기/전압): 안전(SAFE)으로 복귀",
    lp.R_V_BARO_LIMIT_LANDED: "V8 기압계 고장 착륙 시간 한계: 지면 접촉 감지 - 모터 정지(착륙 완료), 사출 없음",
    lp.R_V_BARO_LIMIT_ABORT: "V9 기압계 고장 착륙 시간 한계: 지면 접촉 없음 - 비상정지(ABORT), 낙하산 사출",
    lp.R_V_LAND_TIMEOUT: "V10 착륙 바닥 시간 초과: 지면 근처에서 모터 정지(착륙 완료), 사출 없음",
    lp.R_V_GPS_LOST: "V11 GPS 신호 상실: 자세만 유지(바람에 밀림) 후 제자리 착륙",
    lp.R_V_FENCE: "V12 수평 울타리 초과(유지점에서 너무 멀어짐): 제자리 착륙 (사람 근처 비상정지 없음)",
    lp.R_V_GPS_IMPLAUSIBLE: "V13 GPS 값 이상(관성센서와 불일치): GPS 사용 중지, 자세만 유지하며 착륙",
    lp.R_V_AUTHORITY: "V14 추력 여유 부족(위치 유지용 기울기 불가): 제자리 착륙",
    lp.R_V_TAKEOFF_CHECK: "V15 이륙 점검 실패(최대 추력에서도 상승 안 함): 모터 정지, 안전(SAFE) - 사출 없음",
    lp.REJ_STATE: "거부: 지금 상태에서는 실행할 수 없는 명령",
    lp.REJ_NONCE: "거부: 오래된 명령(nonce 불일치) - 다시 누르세요",
    lp.REJ_ARM_SWITCH: "거부: 아밍 스위치 열림",
    lp.REJ_SENSORS: "거부: 센서 이상",
    lp.REJ_NOT_ALIGNED: "거부: 자세 정렬 미완료 - 기체를 가만히 두세요",
    lp.REJ_TILT: "거부: 기체 기울기 과다",
    lp.REJ_VCELL: "거부: 배터리 전압 낮음",
    lp.REJ_LINK: "거부: 통신 연결 확인 전 - 잠시 후 다시",
    lp.REJ_UNKNOWN_CMD: "거부: 알 수 없는 명령",
    lp.REJ_RELEASE_PENDING: "거부: 사출 대기 중 (KILL만 가능)",
    lp.REJ_MODE: "거부: 이 빌드(VTOL)의 명령이 아님",
    lp.REJ_ALT_LIMIT: "거부: 고도 범위 한계",
    lp.REJ_LANDING: "거부: 착륙 중 (착륙은 취소되지 않음)",
    lp.REJ_ENDURANCE: "거부: 이 배터리의 비행시간 소진 - 충전된 배터리로 교체 후 전원 재투입",
    lp.REJ_GPS: "거부: GPS 준비 안 됨 - 하늘이 트인 실외에서 3D 고정·위성 수·정확도가 5초 이상 유지될 때까지 대기",
    # v0.9 power rules (sim.power_link; tether / hybrid builds)
    pl.R_W_LOST: "W_LOST 전선 끊김(전원 상실): 버퍼 배터리로 즉시 착륙 - 전원 장치와 전선을 확인하세요",
    pl.R_W_SAG: "W_SAG 전원 전압 저하(전원 장치 전류 제한 또는 긴 전선): 추력 상한 고정 후 착륙",
    pl.R_W_BUF: "W_BUF 버퍼 배터리 전압 낮음: 착륙 (버퍼 충전 또는 교체)",
    pl.R_W_D2: "W_D2 비행 중 버퍼가 충전됨(D2 다이오드 단락 의심): 착륙 - 착륙 후 지상에서 전원 차단",
    pl.REJ_POWER: "거부: 전원 상태 불량 - 유선 전압(24.6-25.6 V) 또는 버퍼 전압(3.72-3.92 V/셀) 범위 밖, 또는 전압 저하 중 상승 금지",
}

# v0.9 power alarms (tether / hybrid builds): the banner the station shows (and beeps) when a power rule acts
POWER_ALARM_KO = {
    pl.R_W_LOST: "!!! 전선 끊김 - 버퍼로 착륙 중 !!!",
    pl.R_W_SAG: "!!! 전원 전압 저하 - 착륙 중 !!!",
    pl.R_W_BUF: "!!! 버퍼 배터리 부족 - 착륙 중 !!!",
    pl.R_W_D2: "!!! 버퍼 충전 이상(D2) - 착륙 중, 착륙 후 전원 차단 !!!",
}
# TELEM v4 power flags (sim.power_link PF_*; shown when the frame carries them)
POWER_FLAG_TEXT_KO = {pl.PF_TETHER_OK: "전선 전원 정상", pl.PF_SAG: "전압 저하(상승 금지)", pl.PF_BUF_LOW: "버퍼 부족",
                      pl.PF_BUF_SHARE: "버퍼 방전 중", pl.PF_LOST: "전선 끊김", pl.PF_D2: "버퍼 충전 이상"}

# v0.8 TELEM v3 GPS status (Korean operator text; sim.gps_nav GF_* bits, sim.position_ctrl MODE_*)
GPS_FLAG_TEXT_KO = {0x01: "GPS 사용 중", 0x02: "이륙 가능(GPS 준비)", 0x04: "방위 신뢰", 0x08: "3D 고정",
                    0x10: "수신기 설정 완료", 0x20: "기준점 설정", 0x40: "수신기 확인(MON-VER)",
                    0x80: "GPS-관성 속도 불일치"}
POS_MODE_TEXT_KO = {0: "위치 유지 꺼짐", 1: "속도 유지(상승 중)", 2: "위치 유지", 3: "추측항법(GPS 잠시 끊김)",
                    4: "자세만 유지(GPS 없음 - 바람에 밀림)"}


def reason_text_ko(code: int) -> str:
    """Korean operator text of a TELEM reason code (the English name for a code without one)."""
    return REASON_TEXT_KO.get(code, lp.REASON_NAMES.get(code, f"reason {code}"))


def power_tail_of(tlm: "lp.Telemetry | None") -> "pl.PowerTelem | None":
    """v0.9 RP1: the power tail of a TELEM v4 / v5 frame as ``sim.power_link.PowerTelem`` (None for TELEM v1 - v3)."""
    if tlm is None or not tlm.has_pwr:
        return None
    return pl.PowerTelem(*tlm.power_tail)


def power_status_ko(build: int, tlm: "lp.Telemetry | None", ptel: "pl.PowerTelem | None" = None) -> str:
    """v0.9: one Korean line for the power state of a tether / hybrid build ('' for the battery build).  From TELEM v2 /
    v3: the bus voltage (the vcell field is V_in / 6 in every build: the battery-less build's tether voltage, the hybrid's
    bus) and the power rule that acted (the reason byte); from the TELEM v4 / v5 power tail (``ptel``,
    ``sim.power_link.PowerTelem``; ``power_tail_of(tlm)`` when not given - RP1, the firmware round): the buffer per cell
    (hybrid) and the flags."""
    if ptel is None:
        ptel = power_tail_of(tlm)
    if build == pl.PWR_BATTERY:
        return ""
    parts = [pl.PWR_NAMES_KO.get(build, str(build))]
    if tlm is None:
        return " | ".join(parts + ["텔레메트리 없음"])
    parts.append(f"{'전선' if build == pl.PWR_WIRED else '버스'} {6.0 * tlm.vcell_v:.1f} V")
    if ptel is None:
        parts.append("버퍼 전압/전원 상태 비트는 TELEM v4 이후 표시" if build == pl.PWR_WIRED_BUFFER else
                     "전원 상태 비트는 TELEM v4 이후 표시")
    else:
        if build == pl.PWR_WIRED_BUFFER:
            parts.append(f"버퍼 {ptel.v_aux_v / 6.0:.2f} V/셀")
        parts.append(", ".join(txt for bit, txt in POWER_FLAG_TEXT_KO.items() if ptel.flag(bit)) or "-")
    if tlm.reason in POWER_ALARM_KO:
        parts.append(POWER_ALARM_KO[tlm.reason])
    return " | ".join(parts)


def lbat_status_ko(tlm: "lp.Telemetry | None", warn_v: float | None) -> str:
    """2026-11-10: one Korean line for the logic battery of a TELEM v6 / v7 ('' for a frame without the tail)."""
    if tlm is None or not tlm.has_lbat:
        return ""
    v = tlm.lbat_v
    low = warn_v is not None and v < warn_v
    lim = "" if warn_v is None else f" (경고 {warn_v:.2f} V 미만)"
    return f"로직 배터리 {v:.2f} V{lim}" + (" - 낮음: 착륙 후 충전하십시오 (기체는 자동 동작 없음)" if low else "")


def power_on_window_ok(build: int, tlm: "lp.Telemetry", lo_v: float, hi_v: float) -> bool:
    """v0.9 ground pre-check of W_ARM for the battery-less build (the bus at idle = the supply, from the vcell field):
    ON is not sent while 6 x vcell is outside [lo_v, hi_v] (the vehicle would refuse it, REJ_POWER).  The hybrid's buffer
    is not in TELEM v2 / v3: the vehicle checks it (True here)."""
    if build != pl.PWR_WIRED:
        return True
    return lo_v <= 6.0 * tlm.vcell_v <= hi_v


def gps_status_ko(tlm: lp.Telemetry) -> str:
    """One Korean line for a TELEM v3 GPS status ('' for v1 / v2): readiness, satellites, accuracy, mode, distance."""
    if not tlm.is_v3:
        return ""
    ready = "GPS 준비됨" if tlm.gps_flag(0x02) else ("GPS 고정됨(안정화 대기)" if tlm.gps_flag(0x08) else "GPS 고정 없음")
    acc = "-" if tlm.hacc_dm >= lp.HACC_NONE else f"{tlm.hacc_m:.1f} m"
    d = tlm.dist_m
    dist = "유지점 없음" if d is None else f"유지점에서 {d:.1f} m"
    return f"{ready} | 위성 {tlm.num_sv}개 | 정확도 {acc} | {POS_MODE_TEXT_KO.get(tlm.pos_mode, str(tlm.pos_mode))} | {dist}"


def _lbat_warn_v(p: dict) -> float | None:
    lbm = (p.get("flight_computer") or {}).get("logic_battery_monitor") or {}
    v = lbm.get("ground_warn_below_v_ASSUMED")
    return None if v is None else float(v)


@dataclass(frozen=True)
class GroundParams:
    vehicle_id: int
    poll_hz: float
    reply_timeout_s: float
    t_link_default_s: float
    target_altitude_m: float
    coast_drag_factor: float
    apogee_detect_vz_mps: float
    apogee_confirm_s: float
    coast_timeout_s: float
    release_after_cut_s: float = 0.0
    g: float = G0
    rtt_window: int = RTT_WINDOW_ASSUMED
    # SPEC 8.8 v0.4 (mission.assist_release_rule = "predictive"): RELEASE when v_z predicted to the servo-opening time
    # (v_z - g (t_link + release_after_cut_s + T_tel / 2), T_tel = max(1/poll_hz, t_link)) <= apogee_detect_vz;
    # apogee_confirm_s is not applied to the prediction.  False = the v0.3 literal rule.
    release_compensated: bool = True
    mode: str = lp.MODE_BALLISTIC          # v0.7: the vehicle build's mission.mode (VTOL: ON / OFF / UP / DOWN, no assist)
    lbat_warn_v: float | None = None       # 2026-11-10: logic-battery warning threshold (TELEM v6 / v7), None = no warning

    @property
    def vtol(self) -> bool:
        return self.mode == lp.MODE_VTOL

    @classmethod
    def from_params(cls, p: dict, reply_timeout_s: float, t_link_default_s: float,
                    target_altitude_m: float | None = None, coast_drag_factor: float | None = None,
                    release_compensated: bool | None = None, mode: str | None = None) -> "GroundParams":
        """``mode`` follows params.json ``mission.mode`` (absent: ballistic) unless given."""
        m, lk = p["mission"], p["flight_computer"]["link"]
        if release_compensated is None:
            rule = str(m.get("assist_release_rule", "predictive"))
            if rule not in ("predictive", "literal"):
                raise ValueError(f"mission.assist_release_rule must be 'predictive' or 'literal', not {rule!r}")
            release_compensated = rule == "predictive"
        return cls(vehicle_id=int(lk["vehicle_id"]), poll_hz=float(lk["poll_hz"]), reply_timeout_s=float(reply_timeout_s),
                   t_link_default_s=float(t_link_default_s),
                   target_altitude_m=float(m["target_altitude_m"] if target_altitude_m is None else target_altitude_m),
                   coast_drag_factor=float(m["coast_drag_factor"] if coast_drag_factor is None else coast_drag_factor),
                   apogee_detect_vz_mps=float(m["apogee_detect_vz_mps"]),
                   apogee_confirm_s=float(m["apogee_confirm_s"]), coast_timeout_s=float(m["coast_timeout_s"]),
                   release_after_cut_s=float(m["release_after_cut_s"]), release_compensated=bool(release_compensated),
                   mode=lp.mission_mode(p) if mode is None else lp.check_mode(mode),
                   lbat_warn_v=_lbat_warn_v(p))


@dataclass
class CommandRecord:
    cmd: int
    arg: int
    source: str                    # "operator" | "assist"
    t_queued: float
    seq: int | None = None
    t_first_tx: float | None = None
    n_tx: int = 0
    t_ack: float | None = None
    result: str = ""               # "accepted" | "rejected: <reason>" | "superseded"
    note: str = ""

    @property
    def name(self) -> str:
        return lp.CMD_NAMES.get(self.cmd, str(self.cmd))


class GroundAssist:
    def __init__(self, gp: GroundParams, assist: bool = True):
        self.gp = gp
        self.assist = bool(assist) and not gp.vtol          # VTOL: the vehicle flies the altitude itself, no assist
        self.decoder = lp.FrameDecoder(gp.vehicle_id, lp.TELEM_TYPES)
        self.queue: deque[CommandRecord] = deque()
        self.pending: CommandRecord | None = None
        self.records: list[CommandRecord] = []
        self.seq = 0                       # last SEQ assigned (heartbeats repeat it)
        self.synced = False
        self.latest: lp.Telemetry | None = None
        self.t_latest: float | None = None
        self.awaiting = False
        self.t_sent = 0.0
        self.sent_seq = 0
        self.deadline = 0.0
        self.next_poll = float("-inf")
        self.rtts: deque[float] = deque(maxlen=gp.rtt_window)
        self.rtt_all: list[tuple[float, float]] = []      # (t_reply, rtt)
        self.stats = {"polls": 0, "replies": 0, "timeouts": 0, "stale_replies": 0, "telemetry": 0, "late_replies": 0}
        self.reply_open = False             # R-ESP1: the latest request's reply has not arrived (on time or late) yet
        self.rtt_ambiguous = False          # R-ESP1: the request before it had the same SEQ and was never answered
        self.events: list[tuple[float, str]] = []
        self.assist_log: list[dict] = []
        self.cut_queued = False
        self.release_queued = False
        self.was_powered = False
        self.cut_prev: tuple[int, float] | None = None     # (t_ms, P) of the previous POWERED telemetry
        self.cut_rate = 0.0                                 # dP/dt measured at that telemetry [m/s]
        self.coast_t0_ms: int | None = None
        self.apogee_t0_ms: int | None = None
        self.phase: int | None = None                       # VTOL: latest TELEM v2 phase / target
        self.h_tgt_m: float | None = None
        self.rule_reason: int = lp.REASON_NONE              # VTOL: latest safety-rule reason seen (1..63)
        self.gps_ready: bool | None = None                  # v0.8: latest TELEM v3 GATE_OK
        self.pos_mode: int | None = None                    # v0.8: latest TELEM v3 position-hold mode
        self.power_alarm: int | None = None                 # v0.9: the latest power rule seen (POWER_ALARM_KO)
        self.ptel: pl.PowerTelem | None = None              # v0.9 RP1: the latest TELEM v4 / v5 power tail
        self.lbat_v: float | None = None                    # 2026-11-10: the latest TELEM v6 / v7 logic battery [V]
        self.lbat_low = False                               # below GroundParams.lbat_warn_v (hysteresis)
        if gp.vtol:
            self.events.append((0.0, "VTOL build: commands ON / OFF / UP / DOWN / KILL, assist off"))

    # ---- operator interface -----------------------------------------------------------------------
    def set_assist(self, on: bool, now: float = 0.0) -> None:
        if on and self.gp.vtol:
            self.events.append((now, "assist not available in the VTOL build (the vehicle flies the altitude itself)"))
            return
        self.assist = bool(on)
        self.events.append((now, f"assist {'on' if on else 'off'}"))

    def command(self, cmd: int, arg: int = 0, now: float = 0.0, source: str = "operator", note: str = "") -> CommandRecord:
        if cmd not in lp.CMD_NAMES or cmd == lp.CMD_NONE:
            raise ValueError(f"not a command: {cmd}")
        if cmd not in lp.MODE_COMMANDS[self.gp.mode]:
            raise ValueError(f"{lp.CMD_NAMES[cmd]} is not a command of the {self.gp.mode} build "
                             f"(use {', '.join(lp.CMD_NAMES[c] for c in lp.MODE_COMMANDS[self.gp.mode])})")
        rec = CommandRecord(cmd, int(arg), source, now, note=note)
        self.records.append(rec)
        if cmd in PRIORITY_CMDS:
            for old in ([self.pending] if self.pending else []) + list(self.queue):
                if not old.result:
                    old.result = "superseded"
            self.queue.clear()
            self.pending = rec
        else:
            self.queue.append(rec)
            if self.pending is None:
                self.pending = self.queue.popleft()
        self.events.append((now, f"queue {rec.name}{' ' + str(arg) if cmd == lp.CMD_THROTTLE else ''} ({source}{': ' + note if note else ''})"))
        return rec

    def arm(self, now: float) -> CommandRecord:
        return self.command(lp.CMD_ARM, now=now)

    def disarm(self, now: float) -> CommandRecord:
        return self.command(lp.CMD_DISARM, now=now)

    def launch(self, now: float) -> CommandRecord:
        return self.command(lp.CMD_LAUNCH, now=now)

    def throttle(self, u: float, now: float) -> CommandRecord:
        return self.command(lp.CMD_THROTTLE, lp.quantize(u, lp.THROTTLE_SCALE, lp.U16), now=now)

    def cut(self, now: float) -> CommandRecord:
        return self.command(lp.CMD_CUT, now=now)

    def release(self, now: float) -> CommandRecord:
        return self.command(lp.CMD_RELEASE, now=now)

    def kill(self, now: float) -> CommandRecord:
        return self.command(lp.CMD_KILL, now=now)

    # VTOL build (v0.7)
    def on(self, now: float) -> CommandRecord:
        """Take off: spool warning, climb to hover_altitude_m, hold (carries the latest telemetry nonce)."""
        return self.command(lp.CMD_ON, now=now)

    def off(self, now: float) -> CommandRecord:
        """Land where it is (or cancel the spool-up); pre-empts any pending UP / DOWN / ON."""
        return self.command(lp.CMD_OFF, now=now)

    def up(self, now: float) -> CommandRecord:
        """Altitude target + alt_step_m (rejected by the vehicle at alt_max_m or outside CLIMB / HOLD)."""
        return self.command(lp.CMD_UP, now=now)

    def down(self, now: float) -> CommandRecord:
        """Altitude target - alt_step_m (rejected by the vehicle at alt_min_m or outside CLIMB / HOLD)."""
        return self.command(lp.CMD_DOWN, now=now)

    def vtol_status(self) -> dict:
        """What a VTOL display shows, from the latest telemetry only (None before any)."""
        t = self.latest
        if t is None:
            return {"state": None, "phase": None, "h_tgt_m": None, "h_m": None, "vz_mps": None, "throttle": None}
        st = {"state": t.state_name, "phase": t.phase_name, "h_tgt_m": t.h_tgt_m, "h_m": t.h_m, "vz_mps": t.vz_mps,
              "throttle": t.throttle, "reason": t.reason_name, "interlock": t.flag(lp.FLAG_INTERLOCK),
              "reason_ko": reason_text_ko(t.reason)}
        if t.is_v3:                                             # v0.8 TELEM v3: GPS status
            st.update(gps_ready=t.gps_flag(0x02), gps_fix_ok=t.gps_flag(0x01), gps_3d=t.gps_flag(0x08),
                      num_sv=t.num_sv, hacc_m=None if t.hacc_dm >= lp.HACC_NONE else t.hacc_m, pos_mode=t.pos_mode,
                      dist_m=t.dist_m, gps_ko=gps_status_ko(t))
        if t.has_lbat:                                          # 2026-11-10 TELEM v6 / v7: the logic battery
            st.update(lbat_v=t.lbat_v, lbat_low=self.lbat_low, lbat_ko=lbat_status_ko(t, self.gp.lbat_warn_v))
        return st

    def reply_timeout(self) -> float:
        """R-ESP1 (c): max(the configured timeout, the longest round trip in the window + margin), <= the limit."""
        t = self.gp.reply_timeout_s
        if self.rtts:
            t = max(t, min(max(self.rtts) + LATE_TIMEOUT_MARGIN_S_ASSUMED, LATE_TIMEOUT_MAX_S_ASSUMED))
        return t

    @property
    def t_link(self) -> float:
        """R-ESP1 (2026-11-10): max(mean of the last rtt_window round trips, the latest one); the default before any."""
        if not self.rtts:
            return self.gp.t_link_default_s
        return max(sum(self.rtts) / len(self.rtts), self.rtts[-1])

    # ---- radio --------------------------------------------------------------------------------------
    def _next_seq(self) -> int:
        s = (self.seq + 1) & 0xFF
        if self.latest is not None and not lp.seq_newer(s, self.latest.ack_seq):
            s = (self.latest.ack_seq + 1) & 0xFF
        self.seq = s
        return s

    def poll(self, now: float) -> bytes | None:
        """The uplink frame to transmit now, or None."""
        if self.awaiting:
            if now < self.deadline:
                return None
            self.awaiting = False
            self.stats["timeouts"] += 1
            if self.assist:
                self._assist_on_timeout(now)
        rec = self.pending if self.synced else None
        fresh = rec is not None and rec.n_tx == 0
        if now < self.next_poll and not fresh:
            return None
        if rec is None:
            cmd, seq = lp.Command(lp.CMD_NONE), self.seq
        else:
            if rec.seq is None:
                rec.seq = self._next_seq()
            nonce = self.latest.nonce if (rec.cmd in lp.NONCE_COMMANDS and self.latest is not None) else 0
            cmd, seq = lp.Command(rec.cmd, rec.arg, nonce), rec.seq
            if rec.n_tx == 0:
                rec.t_first_tx = now
                self.events.append((now, f"TX {rec.name} seq {seq}"))
            rec.n_tx += 1
        self.rtt_ambiguous = self.reply_open and seq == self.sent_seq    # R-ESP1: the unanswered one had this SEQ
        self.reply_open = True
        self.awaiting = True
        self.t_sent = now
        self.sent_seq = seq
        self.deadline = now + self.reply_timeout()
        self.next_poll = now + 1.0 / self.gp.poll_hz
        self.stats["polls"] += 1
        return lp.encode_command(self.gp.vehicle_id, seq, cmd)

    def receive(self, data: bytes, now: float) -> list[lp.Telemetry]:
        """Feed downlink bytes; returns the telemetry frames decoded from them."""
        out = []
        for fr in self.decoder.feed(data):
            tlm = lp.decode_telemetry(fr)
            if self.awaiting and fr.seq == self.sent_seq:
                rtt = now - self.t_sent
                self.rtts.append(rtt)
                self.rtt_all.append((now, rtt))
                self.awaiting = False
                self.reply_open = False
                self.stats["replies"] += 1
            elif self.reply_open and fr.seq == self.sent_seq and not self.rtt_ambiguous:
                rtt = now - self.t_sent                       # R-ESP1 (a): a late reply, after its timeout
                self.rtts.append(rtt)
                self.rtt_all.append((now, rtt))
                self.reply_open = False
                self.stats["late_replies"] += 1
                self.events.append((now, f"late reply seq {fr.seq}: round trip {rtt:.3f} s > reply timeout "
                                         f"{self.gp.reply_timeout_s:.3f} s (t_link now {self.t_link:.3f} s, timeout "
                                         f"{self.reply_timeout():.3f} s)"))
            else:
                if fr.seq == self.sent_seq:
                    self.reply_open = False
                self.stats["stale_replies"] += 1
            if self.latest is not None and tlm.t_ms <= self.latest.t_ms:
                continue                                      # older than what we have: data ignored
            self._on_telemetry(tlm, now)
            out.append(tlm)
        return out

    # ---- telemetry handling ----------------------------------------------------------------------------
    def _advance(self) -> None:
        self.pending = self.queue.popleft() if self.queue else None

    def _on_telemetry(self, tlm: lp.Telemetry, now: float) -> None:
        self.latest = tlm
        self.t_latest = now
        self.stats["telemetry"] += 1
        if not self.synced:
            self.synced = True
            self.seq = tlm.ack_seq
            self.events.append((now, f"synced: vehicle ack_seq {tlm.ack_seq}, state {tlm.state_name}"))
        if tlm.is_v2:                                           # VTOL: phase / target tracking
            if tlm.phase != self.phase:
                old = "-" if self.phase is None else lp.PHASE_NAMES.get(self.phase, str(self.phase))
                self.events.append((now, f"phase {old} -> {tlm.phase_name} (state {tlm.state_name})"))
            if tlm.h_tgt_m != self.h_tgt_m:
                old = "-" if self.h_tgt_m is None else f"{self.h_tgt_m:.1f}"
                self.events.append((now, f"target {old} -> {tlm.h_tgt_m:.1f} m"))
            self.phase, self.h_tgt_m = tlm.phase, tlm.h_tgt_m
            if 0 < tlm.reason < 0x40 and tlm.reason != self.rule_reason:   # a (new) safety rule acted on the vehicle
                self.rule_reason = tlm.reason
                self.events.append((now, f"rule {tlm.reason_name} -> {tlm.state_name} / {tlm.phase_name}"))
                if tlm.reason in POWER_ALARM_KO:                # v0.9: a power rule - the station's alarm banner
                    self.power_alarm = tlm.reason
                    self.events.append((now, f"POWER ALARM {POWER_ALARM_KO[tlm.reason]}"))
            if tlm.has_pwr:                                     # v0.9 RP1: the power tail (TELEM v4 / v5)
                pt = pl.PowerTelem(*tlm.power_tail)
                if self.ptel is None or pt.flags != self.ptel.flags:
                    names = ", ".join(n for b, n in pl.PF_NAMES.items() if pt.flag(b)) or "-"
                    self.events.append((now, f"PWR tail {pl.PWR_NAMES.get(pt.build, pt.build)}: v_aux {pt.v_aux_v:.3f} V, "
                                             f"flags 0x{pt.flags:02X} ({names})"))
                self.ptel = pt
            if tlm.has_lbat:                                    # 2026-11-10: TELEM v6 / v7 logic battery (warning only)
                v = tlm.lbat_v
                if self.lbat_v is None:
                    self.events.append((now, f"logic battery {v:.2f} V (TELEM v6 / v7)"))
                self.lbat_v = v
                w = self.gp.lbat_warn_v
                if w is not None and not self.lbat_low and v < w:
                    self.lbat_low = True
                    self.events.append((now, f"LOGIC BATTERY LOW {v:.2f} V < {w:.2f} V (warning only, the vehicle does "
                                             "nothing)"))
                elif self.lbat_low and w is not None and v >= w + LBAT_WARN_HYST_V:
                    self.lbat_low = False
                    self.events.append((now, f"logic battery {v:.2f} V back above {w + LBAT_WARN_HYST_V:.2f} V"))
            if tlm.is_v3:                                       # v0.8: GPS readiness and position-hold mode changes
                ready = tlm.gps_flag(0x02)
                if ready != self.gps_ready:
                    self.events.append((now, f"GPS {'ready' if ready else 'not ready'} ({tlm.num_sv} SV, hAcc "
                                             f"{tlm.hacc_dm / 10:.1f} m)"))
                    self.gps_ready = ready
                if tlm.pos_mode != self.pos_mode:
                    self.events.append((now, f"position hold mode {self.pos_mode} -> {tlm.pos_mode}"))
                    self.pos_mode = tlm.pos_mode
        rec = self.pending
        if rec is not None and rec.seq is not None and rec.n_tx > 0:
            if tlm.ack_seq == rec.seq:
                rec.t_ack = now
                rn = tlm.reason_name
                if not tlm.flag(lp.FLAG_REJECTED):
                    rec.result = "accepted"
                elif tlm.reason >= 0x40:
                    rec.result = rn
                else:   # the latched interlock code outranks the rejection code (SPEC 10.3): only the flag says rejected
                    rec.result = f"rejected (cause not sent: the reason byte holds {rn}; check the telemetry flags)"
                self.events.append((now, f"ACK {rec.name} seq {rec.seq} {rec.result} (sent {rec.n_tx}x)"))
                self._advance()
            elif not lp.seq_newer(rec.seq, tlm.ack_seq):
                self.events.append((now, f"renumber {rec.name}: seq {rec.seq} not newer than vehicle ack_seq {tlm.ack_seq}"))
                rec.seq = None
        if self.assist:
            self._assist(tlm, now)

    def t_tel(self) -> float:
        """Telemetry interval: one reply per poll, never faster than the round trip."""
        return max(1.0 / self.gp.poll_hz, self.t_link)

    def predicted_apogee(self, h: float, vz: float, t_link: float) -> float:
        v = vz if vz > 0.0 else 0.0
        return h + vz * t_link + self.gp.coast_drag_factor * v * v / (2.0 * self.gp.g)

    def _assist(self, tlm: lp.Telemetry, now: float) -> None:
        gp = self.gp
        if tlm.state == lp.STATE_POWERED:
            self.was_powered = True
            self.coast_t0_ms = None
            self.apogee_t0_ms = None
            if not self.cut_queued:
                tl = self.t_link
                pred = self.predicted_apogee(tlm.h_m, tlm.vz_mps, tl)
                rate = 0.0
                if self.cut_prev is not None and tlm.t_ms > self.cut_prev[0]:
                    rate = max((pred - self.cut_prev[1]) / ((tlm.t_ms - self.cut_prev[0]) / 1000.0), 0.0)
                self.cut_prev = (tlm.t_ms, pred)
                self.cut_rate = rate
                lead_m = rate * 0.5 * self.t_tel()
                if pred + lead_m >= gp.target_altitude_m:
                    self.cut_queued = True
                    note = (f"h {tlm.h_m:.1f} m, v_z {tlm.vz_mps:.2f} m/s, t_link {tl:.3f} s, predicted {pred:.1f} m "
                            f"+ lead {lead_m:.1f} m >= {gp.target_altitude_m:.0f}")
                    self.assist_log.append({"t": now, "cmd": "CUT", "t_ms": tlm.t_ms, "h_m": tlm.h_m, "vz_mps": tlm.vz_mps,
                                            "t_link_s": tl, "predicted_m": pred, "rate_mps": rate, "lead_m": lead_m})
                    self.command(lp.CMD_CUT, now=now, source="assist", note=note)
        elif tlm.state == lp.STATE_UNPOWERED and self.was_powered and not self.release_queued:
            if self.coast_t0_ms is None:
                self.coast_t0_ms = tlm.t_ms
            tmo = (tlm.t_ms - self.coast_t0_ms) >= round(gp.coast_timeout_s * 1000.0)
            if gp.release_compensated:
                lead = self.t_link + gp.release_after_cut_s + 0.5 * self.t_tel()
                v_pred = tlm.vz_mps - gp.g * lead
                apo = v_pred <= gp.apogee_detect_vz_mps
                why_apo = f"compensated apogee: v_z {tlm.vz_mps:.2f} m/s predicted {v_pred:.2f} m/s after {lead:.2f} s"
            else:
                if tlm.vz_mps <= gp.apogee_detect_vz_mps:
                    if self.apogee_t0_ms is None:
                        self.apogee_t0_ms = tlm.t_ms
                else:
                    self.apogee_t0_ms = None
                apo = self.apogee_t0_ms is not None and (tlm.t_ms - self.apogee_t0_ms) >= round(gp.apogee_confirm_s * 1000.0)
                why_apo = (f"apogee: v_z <= {gp.apogee_detect_vz_mps:.1f} m/s for "
                           f"{(tlm.t_ms - self.apogee_t0_ms) / 1000:.2f} s" if apo else "")
            if apo or tmo:
                self.release_queued = True
                why = why_apo if apo else f"coast timeout {gp.coast_timeout_s:.1f} s"
                self.assist_log.append({"t": now, "cmd": "RELEASE", "t_ms": tlm.t_ms, "h_m": tlm.h_m, "vz_mps": tlm.vz_mps,
                                        "why": "apogee" if apo else "coast_timeout"})
                self.command(lp.CMD_RELEASE, now=now, source="assist", note=f"{why}, h {tlm.h_m:.1f} m")

    def _assist_on_timeout(self, now: float) -> None:
        """A request went unanswered: apply the CUT / predictive RELEASE rules to the latest telemetry extrapolated
        by its age (see the module docstring).  Nothing is extrapolated before two POWERED telemetry frames gave a
        dP/dt, nor for the literal release rule."""
        tlm, gp = self.latest, self.gp
        if tlm is None or self.t_latest is None:
            return
        age = max(now - self.t_latest, 0.0)
        if (tlm.state == lp.STATE_POWERED and not self.cut_queued and self.cut_prev is not None
                and self.cut_prev[0] == tlm.t_ms and self.cut_rate > 0.0):
            pred = self.cut_prev[1] + self.cut_rate * age
            lead_m = self.cut_rate * 0.5 * self.t_tel()
            if pred + lead_m >= gp.target_altitude_m:
                self.cut_queued = True
                note = (f"no reply: telemetry t_ms {tlm.t_ms} ({tlm.h_m:.1f} m, v_z {tlm.vz_mps:.2f} m/s) extrapolated "
                        f"{age:.3f} s at dP/dt {self.cut_rate:.1f} m/s: predicted {pred:.1f} m + lead {lead_m:.1f} m >= "
                        f"{gp.target_altitude_m:.0f}")
                self.assist_log.append({"t": now, "cmd": "CUT", "t_ms": tlm.t_ms, "h_m": tlm.h_m, "vz_mps": tlm.vz_mps,
                                        "t_link_s": self.t_link, "predicted_m": pred, "rate_mps": self.cut_rate,
                                        "lead_m": lead_m, "extrapolated": True, "age_s": age,
                                        "predicted_at_telemetry_m": self.cut_prev[1]})
                self.command(lp.CMD_CUT, now=now, source="assist", note=note)
        elif (tlm.state == lp.STATE_UNPOWERED and self.was_powered and not self.release_queued
              and gp.release_compensated and self.coast_t0_ms is not None):
            lead = self.t_link + gp.release_after_cut_s + 0.5 * self.t_tel()
            v_pred = tlm.vz_mps - gp.g * (age + lead)
            apo = v_pred <= gp.apogee_detect_vz_mps
            tmo = (tlm.t_ms + round(age * 1000.0) - self.coast_t0_ms) >= round(gp.coast_timeout_s * 1000.0)
            if apo or tmo:
                self.release_queued = True
                why = (f"no reply: compensated apogee: v_z {tlm.vz_mps:.2f} m/s (telemetry t_ms {tlm.t_ms}) predicted "
                       f"{v_pred:.2f} m/s after {age:.3f} + {lead:.2f} s" if apo else
                       f"no reply: coast timeout {gp.coast_timeout_s:.1f} s (extrapolated {age:.3f} s)")
                self.assist_log.append({"t": now, "cmd": "RELEASE", "t_ms": tlm.t_ms, "h_m": tlm.h_m, "vz_mps": tlm.vz_mps,
                                        "why": "apogee" if apo else "coast_timeout", "extrapolated": True, "age_s": age})
                self.command(lp.CMD_RELEASE, now=now, source="assist", note=f"{why}, h {tlm.h_m:.1f} m")
