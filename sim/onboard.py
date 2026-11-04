"""Onboard supervisor (SPEC §8.8 ballistic, v0.4; SPEC §8.9 VTOL, v0.7) - the only autonomous logic on the vehicle.

Executable specification for the firmware's command executor + safety
interlocks.  Pure Python (no numpy), integer control ticks (``control.loop_hz``)
for every timer, so a C port makes identical decisions.  Two builds, chosen at
build time by ``mission.mode`` (``SupervisorParams.mode``): ``"ballistic"`` (the
v0.4-v0.6.3 mission below, bit-identical to v0.6.3) and ``"vtol"`` (v0.7, the
section "VTOL build" further down).

Ballistic build (v0.4)
======================

The vehicle flies attitude control only; launch, throttle, motor
cut and nose release are ground commands received over the LoRa link
(``sim.link_proto``).

States::

    SAFE      motor off (1000 us), vanes centred, release latched      (boot state)
    ARMED     on the pad, motor off, vanes centred, estimator aligning
    POWERED   throttle target > 0, attitude control on, throttle slewed 0 -> 1 in throttle_ramp_s
    UNPOWERED throttle 0 after POWERED; attitude control on for coast_control_hold_s, then vanes centred
    RELEASED  release servo open; motor off until power cycle; vanes centred
    ABORT     motor off now; release after abort_release_delay_s, then RELEASED

Commands (valid CMD frame for this vehicle; executed only when its SEQ is newer
than the last processed SEQ - ``link_proto.seq_newer``; the first frame after
boot is always new; a NONE frame with a newer SEQ only adopts the SEQ):

    ARM       SAFE                      -> ARMED if arm switch closed, sensors healthy, aligned,
                                           tilt < arm_tilt_max_deg, V_cell >= min_cell_v, link alive
    DISARM    ARMED                     -> SAFE
    LAUNCH    ARMED                     nonce == telemetry nonce; target = climb_throttle -> POWERED
    THROTTLE  ARMED (nonce), POWERED, UNPOWERED
                                        target = clamp(u, 0, climb_throttle); u > 0 -> POWERED, u = 0 -> UNPOWERED
    CUT       POWERED                   target 0 -> UNPOWERED
    RELEASE   ARMED, POWERED, UNPOWERED throttle 0 now (POWERED -> UNPOWERED), release servo opens after
                                        release_after_cut_s -> RELEASED
    KILL      any except SAFE           ARMED -> SAFE; POWERED/UNPOWERED -> ABORT; ABORT/RELEASED: no change
    ON / OFF / UP / DOWN (v0.7 codes 8..11): REJ_UNKNOWN_CMD, exactly what v0.6.3 answered for them

Readings of the contract made here (all the safe side, see ``issues`` in the run report):

* "processed" SEQ: a rejected command consumes its SEQ (ack_seq confirms it, the REJECTED
  flag says it was not executed), so a repeated frame can never execute it later.
* "link alive" for ARM: a valid ground frame within ``link_loss_disarm_s`` *before* the ARM
  frame (the first frame after a silence cannot arm).
* THROTTLE(u = 0) in ARMED is rejected (it would enter UNPOWERED on the pad, where I1 can abort
  and fire the release).
* While a release is pending only KILL is accepted; KILL / an interlock abort keeps the earlier of
  the pending release time and abort_release_delay_s.  On the pad (ARMED) KILL, I1p, I9 and the
  arm switch cancel a pending release (-> SAFE).
* Throttle increases are slewed at 1 / throttle_ramp_s; decreases (THROTTLE down, CUT, RELEASE,
  ABORT, any exit from POWERED) are immediate.
* I7 counts the cumulative POWERED time since boot (re-lights add up).
* ARM is rejected while the release servo is open (after an I10 boot).

Interlocks, evaluated every tick after the received commands, in this order; the
first one that applies acts and the rest wait for the next tick:

    arm switch open    ARMED -> SAFE; POWERED/UNPOWERED -> ABORT                (acts as KILL)
    I9  IMU fail >= imu_fail_kill_s / baro fail >= baro_fail_kill_s: ARMED -> SAFE, POWERED/UNPOWERED -> ABORT
    I1p ARMED, no valid ground frame for link_loss_disarm_s                  -> SAFE
    I1  POWERED/UNPOWERED, no valid ground frame for link_loss_abort_s       -> ABORT
    I2  POWERED, tilt > tilt_abort_deg                                       -> ABORT
    I3  POWERED, gyro at full scale (gyro_saturation_abort) or |w_xy| > tilt_rate_abort_dps or
        |w_z| > spin_rate_abort_dps for rate_abort_persist_s                   -> ABORT
    I8  POWERED, V_cell < min_cell_v                                         -> ABORT
    I5  POWERED/UNPOWERED, h > geofence_altitude_m                           -> ABORT
    I4  POWERED, h + coast_drag_factor max(v_z, 0)^2 / (2 g) >= apogee_limit_m -> CUT
    I7  POWERED, cumulative POWERED time >= max_motor_time_s                  -> CUT
    I6  POWERED/UNPOWERED, no release open/pending, v_z <= fall_deploy_vz_mps for
        fall_deploy_confirm_s                                                 -> RELEASE
    I10 boot after a watchdog / brown-out reset in flight: SAFE with the release open

Then the release timer (pending release or ABORT) opens the servo -> RELEASED, the
throttle is slewed, and the POWERED-time counter advances.  Durations are tick
counts n = ticks_ge(seconds) = the smallest n with n / loop_hz >= seconds (the rule of
``tools/gen_firmware_params.py``); a condition "for T" fires on the tick its
counter reaches n (counters start at 1 on the first tick the condition holds).
Throttle is integer per mille: out = min(target, level x 1000 // ramp_ticks), level
+1 per tick while out < target.

Nonce: 16-bit Galois LFSR (taps 0xB400, maximal length, never 0).  At boot
state = seed (0 -> 0xACE1) and nonce = next(state); every executed LAUNCH,
THROTTLE-from-ARMED or (VTOL) ON advances it once.  The firmware seeds it from ADC noise.

VTOL build (v0.7, SPEC 8.9; user request 2026-10-20: vertical take-off and landing is the
primary goal, the ground sends only ON / OFF / UP / DOWN, the onboard code flies the altitude
and rejects attitude disturbances itself)
========================================================================================

The supervisor decides WHEN the motor may run and which vertical PHASE the vehicle is in;
the vertical controller (``sim.vertical_ctrl``, run after the supervisor in the same tick,
``FcuStack``) decides HOW MUCH throttle.  States keep their meaning, phases refine them::

    SAFE      phase NONE    motor off, vanes centred                                   (boot state)
    ARMED     phase SPOOL   buzzer on, motor off, vanes centred for spool_warning_s (people step back)
    POWERED   phase CLIMB   motor driven by the vertical controller: smooth take-off, climb at climb_rate_mps
                            to the target (hover_altitude_m at ON); CLIMB -> HOLD when the controller's
                            reference first reaches the target (its at_target flag of the previous tick)
              phase HOLD    hold the target; UP / DOWN move it by alt_step_m within [alt_min_m, alt_max_m]
              phase LAND    descend (descent_rate_mps, touchdown_rate_mps below flare_altitude_m) until the
                            controller confirms touchdown (its touchdown flag of the previous tick)
    UNPOWERED phase LANDED  motor off, vanes centred; SAFE after safe_after_landed_s (another ON may fly again)
    ABORT     phase NONE    motor off now; release after abort_release_delay_s, then RELEASED
    RELEASED  phase NONE    release servo open; motor off until power cycle

Attitude control (``control_active``) runs in POWERED only (all three phases).

Commands (same SEQ / NONE / repeat rules as the ballistic build):

    ON     SAFE   nonce == telemetry nonce, then every ARM gate (release not open, arm switch closed,
                  sensors healthy, aligned, tilt < arm_tilt_max_deg, V_cell >= min_cell_v, link alive)
                  and cumulative motor time < max_flight_s - land_reserve_s (REJ_ENDURANCE)
                  -> ARMED (SPOOL), target = hover_altitude_m, nonce advances.  Other states: REJ_STATE.
    OFF    ARMED -> SAFE (spool-up cancelled); POWERED CLIMB / HOLD -> LAND; POWERED LAND and
           UNPOWERED (LANDED): acknowledged, no change (already landing); SAFE / RELEASED: REJ_STATE.
    UP / DOWN  POWERED CLIMB / HOLD: target +- alt_step_m, REJ_ALT_LIMIT if it would leave
           [alt_min_m, alt_max_m]; POWERED LAND: REJ_LANDING (a landing is never cancelled);
           other states: REJ_STATE.
    KILL   as in the ballistic build: ARMED -> SAFE, POWERED / UNPOWERED -> ABORT, ABORT / RELEASED: no
           change, SAFE: REJ_STATE.
    ARM / DISARM / LAUNCH / THROTTLE / CUT / RELEASE: REJ_MODE (checked before "release pending").
    While a release is pending (ABORT) only KILL is accepted (REJ_RELEASE_PENDING), as in ballistic.

The target is held in integer decimetres (the TELEM v2 unit; every params value is rounded to
0.1 m once, in ``from_params``), so UP / DOWN and their limit checks are exact integer
arithmetic in Python and C.

Safety rules (VTOL), evaluated every tick after the commands; the first that applies acts,
the rest wait for the next tick; no command can disable any of them.  Every rule's reason:

    arm switch open  ARMED -> SAFE; POWERED -> ABORT; UNPOWERED (LANDED) -> SAFE.  The hardware cut-off is
                     the operator's last word, as in ballistic (R_ARM_SWITCH_OPEN).  LANDED differs from the
                     ballistic UNPOWERED (a coast in the air): the vehicle is on the ground with the motor off and
                     the switch being opened means a person is at it - the release must not fire at them.
    I9  IMU / NAV failure for imu_fail_kill_s: ARMED -> SAFE; POWERED -> ABORT; UNPOWERED (LANDED) -> SAFE
                     (v0.7.1).  Without attitude the vanes cannot keep the thrust vertical: continuing would
                     convert thrust into a sideways fly-away; stopping the motor is the bounded outcome.  In LANDED
                     the motor is already off: no release toward people handling a landed vehicle
                     (mission.vtol.landed_rules_note).
    I9  baro failure for baro_fail_kill_s: ARMED -> SAFE; POWERED CLIMB / HOLD -> LAND with the
                     vertical controller on the NAV's dead reckoning (``baro_lost``, latched until the next
                     ON; v0.7.1: the accelerometer with its learnt bias, descent_rate_mps to the ground,
                     no flare - ``sim.vertical_ctrl``); POWERED LAND -> switch to it (no state change);
                     LANDED: nothing.  Altitude is lost but attitude is not: a powered descent at a
                     bounded rate is safer than dropping from a few metres (a parachute cannot open there).
                     v0.7.1: the tick it acts also fixes the baro-lost time limit (V8 / V9 below) from the altitude
                     h at that tick.
    I1p ARMED: no valid ground frame for link_loss_disarm_s -> SAFE (never start the motor blind).
    I2  POWERED: tilt > tilt_abort_deg -> ABORT (the thrust can no longer hold the altitude, the
                     vehicle is out of control).
    I3  POWERED: spin / tilt rate / gyro saturation for rate_abort_persist_s -> ABORT (as ballistic).
    V4  POWERED: h > vtol.geofence_m -> ABORT (a climb past the ceiling LAND did not stop: the altitude
                     loop has failed; motor off is the only bound left).  UNPOWERED (LANDED): -> SAFE (v0.7.1:
                     motor already off; on the ground the reading is a baro glitch - no release).
    V5  POWERED: v_z <= vtol.fall_vz_mps for vtol.fall_confirm_s -> ABORT (commanded descents are
                     <= vz_max_down_mps; a fall means the thrust is gone or the vehicle is out of control - motor
                     off, parachute).  UNPOWERED (LANDED): -> SAFE (v0.7.1, the architect's landed_rules_note).
                     Consequence, stated: a FALSE touchdown confirmed in the air (motor off) is no longer caught by
                     a release - the protection is the touchdown detector itself (``sim.vertical_ctrl``: only while
                     low, physical signatures that free flight cannot produce; v0.7.1b: one signature only in the
                     flare band, above it support, floor and still together, and the learnt thrust ratio k_T
                     behind the support signature and the feed-forward).
    V6  POWERED: cumulative motor time >= hard_cutoff_s -> ABORT (absolute bound on battery use,
                     even if a landing is still under way).
    V8 / V9 (v0.7.1) POWERED LAND with baro_lost, no touchdown confirmed: the baro-lost time limit
                     T_lim = time_limit_factor (h_loss / descent_rate_mps + flare_altitude_m / touchdown_rate_mps)
                     + time_limit_margin_s (mission.vtol.baro_lost) after the loss is reached ->
                     a ground signature (the controller's support or floor, previous tick) -> UNPOWERED (LANDED,
                     R_V_BARO_LIMIT_LANDED, motor off, no release); none -> ABORT (R_V_BARO_LIMIT_ABORT: the altitude
                     is unknown, the vehicle may be high - motor off, parachute).  v0.7.0 had no limit: 7 of 20
                     seeds flew away on the integrated accelerometer bias until V6.
                     Integer form (identical in C): h_dm = round(h_loss x 10) clamped to [0, geofence_dm] (not
                     finite: geofence_dm - the longest limit); T_lim_ticks = baro_limit_base_ticks
                     + ceil(h_dm x baro_limit_mt_per_dm / 1000), base = ticks_ge(factor flare / td + margin),
                     mt_per_dm = round(factor x loop_hz / (10 descent) x 1000) (milli-ticks per decimetre).
                     Acts at tick t_loss + T_lim_ticks.
    V10 (v0.7.1) POWERED LAND with the baro healthy, no touchdown confirmed: the controller has been low for
                     land_timeout_ticks (mission.vtol.touchdown.land_floor_timeout_s; its low_in_land_ticks of the
                     previous tick - v0.7.1b, gating_v071b: counted only in the flare band h <= flare_altitude_m
                     and while the reference has run ahead of the vehicle, never on the loop's saturation alone)
                     -> UNPOWERED (LANDED, R_V_LAND_TIMEOUT, motor off, NO release): a vehicle 0-1 m
                     above the ground is safer with the fan stopped than with a release at the ground crew; v0.7.0
                     ran the motor on the ground until the V6 ABORT fired the release (5 of 20 campaign runs).
    then, in POWERED CLIMB / HOLD only (in LAND the vehicle is already doing what they ask):
    V1  no valid ground frame for link_loss_land_s -> LAND (without the link the operator can neither
                     command nor KILL; at a few metres a parachute does not open, so land under power).
    I8  V_cell < min_cell_v -> LAND (the pack is near empty: land while there is power to do it).
    V3  h > ceiling_m -> LAND (the target is limited to alt_max_m < ceiling_m; being above the
                     ceiling means the altitude loop misbehaves: come down under control).
    V2  cumulative motor time >= max_flight_s - land_reserve_s -> LAND (keep land_reserve_s of
                     battery for the descent).
    I10 boot after a watchdog / brown-out reset in flight -> ABORT (motor off, release after
                     abort_release_delay_s): the in-flight state is lost, the motor must not restart.

Timers (after the rules and the release timer, in this order; a rule that changed the
state or phase this tick pre-empts them):

    SPOOL   ARMED for spool_ticks -> take-off gate: tilt < arm_tilt_max_deg and V_cell >= min_cell_v
            -> POWERED (CLIMB, the motor starts this tick); else -> SAFE (V7, the vehicle was knocked
            over or the pack sagged during the warning - never start the motor then).
    CLIMB   the vertical controller's at_target of the previous tick -> HOLD.
    LAND    the vertical controller's touchdown of the previous tick -> UNPOWERED (LANDED), motor off.
    LANDED  UNPOWERED for safe_after_landed_ticks -> SAFE.

Cumulative motor time = POWERED ticks since boot (like ballistic I7): the pack is not recharged
between flights, so a second ON shares the first flight's budget (V2, V6, REJ_ENDURANCE); a
power cycle (fresh pack) resets it.

Build profiles (v0.7.1, mission.vtol.profile / profiles): ``vtol_values(p, profile)`` merges mission.vtol with
profiles[profile] ('field' = as written, 'demo' = the indoor 1-2 m hover demonstration); the supervisor
(``SupervisorParams.from_params(..., profile=)``) and the controller (``VerticalParams.from_params(..., profile=)``)
take the merged values; every safety rule is the same in both profiles.

Tick order (normative for the C port, ``FcuStack.step``; v0.7.1, mission.vtol.fcu_tick_order_note):
(1) attitude controller with this tick's NAV frame and the PREVIOUS tick's supervisor outputs
(``control_active``) and ESC throttle u (gain schedule) - ``FcuOutputs.att_active`` / ``att_u``;
(2) vane registers; (3) supervisor ``update``, seeing the vertical controller's touchdown / at_target /
ground_signature / low_in_land_ticks of the PREVIOUS tick and the ESC throttle applied in the previous
tick (``throttle_prev``, reported in TELEM v2); (4) vertical controller (``sim.vertical_ctrl``) with
this tick's phase, target, ``baro_lost``, NAV altitude, v_z, quaternion (cos(tilt)) and body-z
specific force; (5) ESC = controller throttle while ``motor_on`` (state POWERED), else 0.
Why (the v0.7 integration measured it): the supervisor + vertical controller cost 741-796 us; ahead of the
attitude controller they pushed sample -> vane PWM to 5.15 ms (co-simulation) against 4.66 ms ballistic and
the 6-DOF's 4.71 ms, and NAV tick -> vanes past one 4 ms tick.  Consequences: a supervisor decision
reaches the ESC in the same tick and the vanes one tick (4 ms) later; the gain schedule sees u one tick
old (thrust lag 0.15 s, 50 Hz ESC frames: immaterial).  The ballistic order (supervisor -> attitude ->
outputs) is unchanged.

v0.8 VTOL build with GPS position hold (architect 2026-10-21; params mission.vtol.position_hold; review v0.8)
==========================================================================================================
``SupervisorParams.gps_hold`` (params position_hold.enabled; ``from_params(..., gps=False)`` gives the v0.7.1 VTOL
build) adds, with their reason codes (``sim.link_proto``):

    ON gate  after "aligned": the NAV's GATE_OK of the newest complete GPS slot set (``sim.nlink_proto.GpsSlotRx``:
             3-D fix, numSV >= min_sv, hAcc <= max_hacc, sAcc <= max_sacc for stable_s; at most STALE_TICKS old) ->
             else REJ_GPS (its own code and Korean text - not "not aligned").  Why: position hold needs the GPS from
             the first second of the climb; a vehicle launched blind would only drift.  GPS needs open sky: indoors
             the ON is refused (the v0.7.1 build, gps=False, is the indoor fallback).
    V11 GPS lost (CLIMB / HOLD): the position controller's request REQ_GPS_LOST (no usable GPS for gps_loss.timeout_s
             -> attitude-only hold with the integrator's trim tilt, still none after land_after_s) -> LAND
             (R_V_GPS_LOST).  The landing then drifts with the wind like v0.7.1 (stated, not hidden).
    V12 fence (CLIMB / HOLD): REQ_FENCE (|p - hold point| > fence_m: field 15, demo 5 m) -> LAND (R_V_FENCE), never
             ABORT: an ABORT fires the release near people; a powered landing at the fence is bounded.
    V13 GPS implausible (CLIMB / HOLD): REQ_IMPLAUSIBLE (review v0.8 (7): tilt at its limit 3 s while |v| does not
             fall; |v| > v_max + 1 m/s for 2 s; 3 NAV resets in 10 s) -> the GPS is not used any more (attitude only)
             and LAND (R_V_GPS_IMPLAUSIBLE).
    V14 authority (CLIMB / HOLD): REQ_AUTHORITY (learnt k_T x T/W < authority_check_ASSUMED.min_kT_TW 1.08: no tilt
             left for position hold) -> LAND (R_V_AUTHORITY).
    These four are LAND rules: evaluated after V2, in this order, in CLIMB / HOLD only (in LAND the vehicle already
    lands; the position controller keeps flying the hold point or, without GPS, attitude only).  The requests come
    from the position controller's step of the PREVIOUS tick or earlier (latched there until the motor stops), like
    the vertical controller's flags, so a request raised at tick k acts at tick k + 1.

``SupervisorParams.takeoff_check`` (params position_hold.takeoff_check_ASSUMED present; ``takeoff_check=False`` = off)
adds V15 take-off check (POWERED CLIMB / HOLD, after V6): the vertical controller's takeoff_fail of the previous tick
(pinned at its ceiling u_max for within_s without climbing climb_m, ``sim.vertical_ctrl`` docstring 6) -> SAFE
(R_V_TAKEOFF_CHECK): motor off now, no release - the vehicle is on or just above the pad (v0.7.1 finding: a T/W < 1
vehicle sat at full throttle on the pad until the flight-time rule).

TELEM v3 (``link_proto.TYPE_TELEM3``, gps_hold builds): the v2 fields + gps_flags, num_sv, hacc_dm of the newest
complete slot set (0 / 0 / 255 when none is fresh), the position controller's mode and its distance from the hold
point (NaN -> 0xFFFF before the capture), all as of the previous tick (``OnboardInputs.tlm_gps``, FcuStack).

FCU tick order v0.8 (``FcuStack.step``; the v0.7.1 order with the position hold appended after the ESC write):
(0) the GPS slot of this tick's NAV frame into the slot receiver (``gps_gate_ok`` for the ON gate); (1) attitude
controller with the previous tick's control_active, ESC throttle AND tilt setpoint; (2) vanes; (3) supervisor (the
vertical controller's flags, the position controller's requests and the TELEM v3 GPS fields of the previous tick);
(4) vertical controller (cos(tilt) from the NAV frame's cos_tilt field - nlink v3; the quaternion only when no
cos_tilt is given); (5) ESC; (6) the position step when due (slot 4 received this tick, or 5 ticks since the last
step), with this tick's phase / motor_on, h, cos(tilt), body rates, the vertical controller's k_T and its saturation
(sat == +1 in any tick since the last step) - its outputs are COMMITTED ``sim.position_ctrl.PIPELINE_TICKS`` (4) ticks
later (the C port runs it as five <= 0.2 ms background jobs, review v0.8 (2)); (7) the tilt setpoint builder (250 Hz
slew) on the committed target - its output is the attitude controller's tilt in the NEXT tick.  So a step whose inputs
are latched at tick k moves the attitude setpoint from tick k + 5 and its LAND request acts at tick k + 5.

FCU tick order v0.8.1 (``FcuStack(chain="gns")``, the four-chip build: params flight_computer.architecture_v081; the
position controller runs on the GNS board, ``sim.glink_proto``): (0) the GNS slot of this tick's NAV frame into
``sim.nlink_proto.GnsSlotRx``: a TILT part is the builder's target from this tick; a set completed this tick gives
``gps_gate_ok`` (its GATE_OK), ``pc_requests`` (the GNS's latched REQ_* | REQ_GPS_LOST when no TILT came for land_steps
cycles - the FCU's own GNS-loss rule) and ``tlm_gps``; (1)-(5) unchanged; (6) no position step - instead the pflags / k_T
of this tick for the nlink v4 FCU frame (``FcuOutputs.fcu4``: phase, motor_on, sat == +1, low_alt, ground, lifted, k_T -
exactly the FCU-side inputs v0.8 handed its own position step); (7) the builder (250 Hz slew) on the slot's TILT (< 2
missed cycles), else on the newest set's TRIM (attitude only with the wind trim), active while the GNS says engaged and
motor_on.  The supervisor rules V11-V15 and their reason codes are unchanged (they read the same inputs).  The NAV sample
of tick k reaches the attitude controller at tick k + L + 1 (L = params glink.latency_ticks_ASSUMED 3: 16 ms) - v0.8:
k + 10 (``sim.fc_timing.gns_chain_ticks``).

v0.9 add-on C (V16 early yaw-authority LAND; params mission.vtol.yaw_authority_land_ASSUMED, absent or enabled false =
off - the v0.8.1 supervisor): in POWERED CLIMB / HOLD, |w_z| > rate_dps for persist_s -> LAND (after V2, before the GPS
requests V11-V14).  Reason code: R_V_AUTHORITY until link_proto gets its own code.  Evaluated on TUNE in the v0.9 control
round: NOT flown (TUNE seed 7400, 300 deg/s for 0.25 s: it acted in 20 of 143 runs and turned 1 I3 abort into a landing,
but 19 flights that would have recovered were cut short and two touchdown criteria got worse; scratch
v09sim/control/changes.json).

v0.9 power rules (power / tether round; params mission.vtol.power_rules_ASSUMED; codes, flags and the TELEM v4 power
tail in ``sim.power_link``; docs/tether_power_proposal.json firmware.rules_ASSUMED_thresholds, thresholds checked on the
TUNE seeds only)
========================================================================================================================
``SupervisorParams.power`` = the FCU's power build (a build-time choice like the profile): 'battery' (the v0.8.1 rules
below, unchanged), 'wired' (battery-less, over the tether: V_in = the tether node), 'wired_buffer' (tether + buffer behind
the ideal diodes: V_tether before D1, V_buffer before D2), or 'auto' (the simulation default: the build matching the
plant's signals - ``OnboardInputs.v_buffer`` present -> wired_buffer, ``v_tether`` present -> wired, else battery; resolved
while SAFE, never downgraded; the firmware fixes it at build time).  Without params mission.vtol.power_rules_ASSUMED the
build is always 'battery'.

Power monitor (``PowerMonitor``, integer arithmetic identical in C; VTOL builds only): every tick each measured voltage
(the ADC reading of the previous tick, ``OnboardInputs.v_in`` / ``v_tether`` / ``v_buffer``) is turned into its 10-bit code
x = round(v / adc_v_per_count) and filtered in Q4 (16 x code): y = 16 x on the first sample, then y += (16 x - y) >> shift
(arithmetic shift; shift 3 = a time constant of 8 ticks, 32 ms).  Every threshold is a Q4 code computed once
(``q4(v) = round(16 v / adc_v_per_count)``).  Conditions (persistence counters run in POWERED only and restart at 0):

    wired         sag: y_in < sag_v (count up) / y_in > sag_v + sag_hyst_v (count reset) / between (count held: hysteresis);
                  count >= sag_warn_s -> SAG WARNING: the throttle ceiling is frozen at the ESC throttle of the previous tick
                  (``OnboardOutputs.u_cap`` -> ``VerticalInputs.u_cap``: the vertical loop cannot push the supply further
                  into its limit - the constant-power load past the maximum-power point), UP is refused (REJ_POWER), PF_SAG;
                  released when y_in > sag_v + sag_hyst_v for sag_release_s; count >= sag_land_s -> the W_SAG condition.
    wired_buffer  lost: y_tet < lost_v or y_tet + lost_margin_v < y_buf for lost_persist_s -> the W_LOST condition (latched
                  for the flight: PF_LOST; while both diodes conduct - the crossover - V_tether = V_buffer and nothing
                  counts); buffer low: y_buf < 6 buf_low_cell_v for buf_persist_s (under load) or y_buf < 6
                  buf_reserve_cell_v for buf_reserve_s (the landing reserve: a long or thin tether leaves the buffer
                  sharing the hover until it cannot carry a landing any more) -> W_BUF; D2: y_buf > 6 d2_cell_v while
                  the tether is not lost for d2_persist_s -> W_D2; I8 maps to the buffer: y_buf < 6 min_cell_v.

Rules (the I8 slot of the CLIMB / HOLD LAND rules - after V1, before V3; every one a LAND, never an ABORT; before the
take-off check has passed - ``OnboardInputs.vc_takeoff_passed``, the vehicle never climbed - the same rule stops the motor
-> SAFE like V15 instead: a LAND there descends a reference from the target with the motor running on the pad):

    battery       I8 on V_cell (unchanged).
    wired         W_SAG (R_W_SAG) replaces I8 (V_cell = V_in / 6 is the supply, not a pack).  A total loss of the tether
                  (unplug / cut / supply trip) leaves no power: no rule can act - the catch line arrests the fall.
    wired_buffer  W_LOST (R_W_LOST; the tether is lost - land on the buffer at once), W_BUF (R_W_BUF), W_D2 (R_W_D2), I8 on
                  the buffer (R_I8_VCELL), in this order.  W_LOST becoming true during LAND latches its reason (the
                  ground's alarm) without changing anything else.

ON gate (W_ARM, at the V_cell gate's place; REJ_POWER): wired: y_in in [arm_v_tether_min, arm_v_tether_max] (the supply
set point at idle: "no climb when V_in is low at ON"); wired_buffer: y_tet in that window and y_buf in 6 x [arm_buffer_
cell_min, arm_buffer_cell_max] (the buffer's rest voltage: enough energy for the landing, not a full pack that the
tether's crossover would leave idle).  The V7 take-off gate at the end of the spool warning re-checks the same window.

Tethered limits (wired / wired_buffer; ``tether_limits`` / ``tether_endurance``): hover, alt_min, alt_max, ceiling and
geofence are the smaller of the profile's and the tether block's (alt_max 2.0 m: UP refused above; the ceiling / geofence
sit above the present take-off overshoot of the vertical loop, up to ~1.2 m on the strong light tether vehicle - the
overhead pulley must clear the geofence, params note); max_flight_s / land_reserve_s / hard_cutoff_s are the tether
block's (no pack: the limit is the conductor and motor heating).  Known limit: a supply in its current limit on a short
route keeps the bus above sag_v - the vehicle cannot see it (``sim.tether_scenarios`` P_W_cc_short_INFO).

Invariant: commands are executed before the interlocks of the same tick, so an
interlock always has the last word; no command can disable an interlock.
"""
from __future__ import annotations

import copy
import json
import math
import os
from dataclasses import dataclass, field, replace
from enum import IntEnum
from fractions import Fraction

try:  # package import (project root on sys.path) or flat import (sim/ on sys.path)
    from sim import link_proto as lp
    from sim import power_link as pl
    from sim.position_ctrl import REQ_AUTHORITY, REQ_FENCE, REQ_GPS_LOST, REQ_IMPLAUSIBLE
    from sim.state_machine import G0
except ImportError:  # pragma: no cover
    import link_proto as lp  # type: ignore
    import power_link as pl  # type: ignore
    from position_ctrl import REQ_AUTHORITY, REQ_FENCE, REQ_GPS_LOST, REQ_IMPLAUSIBLE  # type: ignore
    from state_machine import G0  # type: ignore

pl.register_reason_names(lp)          # v0.9 power codes (sim.power_link) until link_proto adopts them

_HERE = os.path.dirname(os.path.abspath(__file__))
FC_CONFIG_PATH = os.path.normpath(os.path.join(_HERE, "..", "firmware", "atmega128_fc", "fc_config.json"))
# SPEC §8.8 I9 row ("IMU failure > 0.1 s or baro failure > 0.5 s"); used only when the firmware's
# fc_config.json (imu_fail_kill_s / baro_fail_kill_s, the calibration file the firmware is built from) is absent.
I9_IMU_FAIL_S_SPEC = 0.1
I9_BARO_FAIL_S_SPEC = 0.5
LFSR_TAPS = 0xB400
LFSR_ZERO_SEED = 0xACE1


class State(IntEnum):
    SAFE = lp.STATE_SAFE
    ARMED = lp.STATE_ARMED
    POWERED = lp.STATE_POWERED
    UNPOWERED = lp.STATE_UNPOWERED
    RELEASED = lp.STATE_RELEASED
    ABORT = lp.STATE_ABORT


FLIGHT_STATES = (State.POWERED, State.UNPOWERED)
MOTOR_ENABLED_STATES = (State.ARMED, State.POWERED, State.UNPOWERED)
FLY_PHASES = (lp.PHASE_CLIMB, lp.PHASE_HOLD)          # VTOL: UP / DOWN and the LAND rules apply
TARGET_PHASES = (lp.PHASE_SPOOL, lp.PHASE_CLIMB, lp.PHASE_HOLD)   # VTOL: the altitude target is reported


def ticks_ge(seconds: float, loop_hz: int) -> int:
    """Smallest tick count n with n / loop_hz >= seconds (exact on the double value; values within
    1e-9 tick of an integer snap to it) - identical to tools/gen_firmware_params.py."""
    x = Fraction(float(seconds)) * int(loop_hz)
    n = math.ceil(x)
    if n - x > 1 - Fraction(1, 10 ** 9):
        n -= 1
    return max(int(n), 0)


def lfsr16_next(s: int) -> int:
    """One step of the 16-bit Galois LFSR x^16 + x^14 + x^13 + x^11 + 1 (period 65535, never 0)."""
    s &= 0xFFFF
    if s == 0:
        s = LFSR_ZERO_SEED
    lsb = s & 1
    s >>= 1
    if lsb:
        s ^= LFSR_TAPS
    return s


def _ceil_div(a: int, b: int) -> int:
    return -(-a // b)


def metres_to_dm(x: float) -> int:
    """Altitude [m] -> integer decimetres (round half away from zero, ``link_proto.quantize``)."""
    return lp.quantize(float(x), lp.H_SCALE, lp.I16)


# --------------------------------------------------------------------------- #
# parameters
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class SupervisorParams:
    """Every number the supervisor uses; times as control ticks.  ``from_params`` is the only
    constructor the sim uses (params.json + the firmware calibration file for I9).

    The VTOL fields (after ``mode``) are used only when ``mode == "vtol"``; their defaults keep
    a ballistic SupervisorParams exactly the v0.6.3 one."""

    loop_hz: int
    climb_milli: int
    ramp_ticks: int
    coast_hold_ticks: int
    tilt_abort_deg: float
    arm_tilt_max_deg: float
    min_cell_v: float
    geofence_m: float
    apogee_limit_m: float
    coast_drag_factor: float
    fall_vz_mps: float
    fall_confirm_ticks: int
    link_abort_ticks: int
    link_disarm_ticks: int
    max_motor_ticks: int
    release_after_cut_ticks: int
    abort_release_ticks: int
    tilt_rate_abort_rad_s: float
    spin_rate_abort_rad_s: float
    rate_persist_ticks: int
    gyro_saturation_abort: bool
    imu_fail_ticks: int
    baro_fail_ticks: int
    g: float = G0
    rate_abort_enabled: bool = True          # False only in the "why I3 exists" study
    i9_source: str = ""
    # ---- v0.7 VTOL build (mission.vtol) ----
    mode: str = lp.MODE_BALLISTIC
    spool_ticks: int = 0                     # spool_warning_s
    hover_dm: int = 0                        # hover_altitude_m [0.1 m]
    alt_step_dm: int = 0                     # alt_step_m
    alt_min_dm: int = 0                      # alt_min_m
    alt_max_dm: int = 0                      # alt_max_m
    link_land_ticks: int = 1                 # link_loss_land_s (V1)
    endurance_ticks: int = 1                 # max_flight_s - land_reserve_s (V2, REJ_ENDURANCE)
    hard_cutoff_ticks: int = 1               # hard_cutoff_s (V6)
    ceiling_m: float = 0.0                   # V3
    v_geofence_m: float = 0.0                # vtol.geofence_m (V4)
    v_fall_vz_mps: float = 0.0               # vtol.fall_vz_mps (V5)
    v_fall_confirm_ticks: int = 1            # vtol.fall_confirm_s (V5)
    safe_after_landed_ticks: int = 0         # safe_after_landed_s
    # ---- v0.7.1 ----
    land_timeout_ticks: int = 0              # touchdown.land_floor_timeout_s (V10); 0 = off
    baro_limit_base_ticks: int = 0           # baro_lost: ticks_ge(factor flare / td + margin)
    baro_limit_mt_per_dm: int = 0            # baro_lost: round(factor loop_hz / (10 descent) x 1000) milli-ticks per dm
    v_geofence_dm: int = 0                   # vtol.geofence_m in dm (the clamp of h_loss)
    profile: str = ""                        # mission.vtol profile the VTOL fields came from
    # ---- v0.8 ----
    gps_hold: bool = False                   # GPS position hold: ON gate, V11-V14, TELEM v3 (position_hold.enabled)
    takeoff_check: bool = False              # V15 (position_hold.takeoff_check_ASSUMED)
    # ---- v0.9 (add-on C, mission.vtol.yaw_authority_land_ASSUMED; absent = off) ----
    v16_rate_rad_s: float = 0.0              # V16 early yaw-authority LAND: |w_z| above this ...
    v16_ticks: int = 0                       # ... for this many ticks in CLIMB / HOLD -> LAND; 0 = off
    # ---- v0.9 power rules (mission.vtol.power_rules_ASSUMED; module docstring "v0.9 power rules") ----
    power: str = "battery"                   # 'battery' | 'wired' | 'wired_buffer' | 'auto' (the plant's signals)
    power_rules: bool = False                # the params block exists (False: always the battery build)
    pw_v_per_count: float = 0.0275           # ADC volts per code (the FCU's VBAT_V_PER_COUNT)
    pw_shift: int = 3                        # Q4 IIR: y += (16 x - y) >> shift
    pw_arm_lo_q: int = 0                     # W_ARM: V_in / V_tether window at idle [Q4 codes]
    pw_arm_hi_q: int = 0
    pw_buf_arm_lo_q: int = 0                 # W_ARM: the buffer's rest window (6 cells) [Q4]
    pw_buf_arm_hi_q: int = 0
    pw_lost_q: int = 0                       # W_LOST: V_tether below this ...
    pw_lost_margin_q: int = 0                # ... or below V_buffer - this [Q4] ...
    pw_lost_ticks: int = 1                   # ... for this many ticks
    pw_sag_q: int = 0                        # W_SAG: V_in below this counts ...
    pw_sag_clear_q: int = 0                  # ... above this resets the count (hysteresis band in between)
    pw_sag_warn_ticks: int = 1               # count -> ceiling frozen, UP refused
    pw_sag_land_ticks: int = 1               # count -> LAND
    pw_sag_release_ticks: int = 1            # above the clear level this long -> the ceiling is released
    pw_buf_low_q: int = 0                    # W_BUF: V_buffer below this (under load) ...
    pw_buf_ticks: int = 1                    # ... for this many ticks, or ...
    pw_buf_res_q: int = 0                    # ... below the landing reserve ...
    pw_buf_res_ticks: int = 1                # ... for this many ticks
    pw_d2_q: int = 0                         # W_D2: V_buffer above this while tethered ...
    pw_d2_ticks: int = 1                     # ... for this many ticks
    pw_i8buf_q: int = 0                      # I8 on the buffer: 6 min_cell_v [Q4]
    w_hover_dm: int = 0                      # tethered limits (0: the profile's)
    w_alt_min_dm: int = 0
    w_alt_max_dm: int = 0
    w_ceiling_m: float = 0.0
    w_geofence_m: float = 0.0
    w_endurance_ticks: int = 0               # tethered max_flight_s - land_reserve_s (0: the profile's)
    w_hard_cutoff_ticks: int = 0

    @property
    def apogee_k(self) -> float:
        """coast_drag_factor / (2 g)."""
        return self.coast_drag_factor / (2.0 * self.g)

    @property
    def vtol(self) -> bool:
        return self.mode == lp.MODE_VTOL

    @classmethod
    def from_params(cls, p: dict, fc_config_path: str | None = FC_CONFIG_PATH, **overrides) -> "SupervisorParams":
        """``mode`` follows params.json ``mission.mode`` (absent: ballistic) unless ``mode=`` is passed; any other
        keyword overrides the field of that name (``geofence_altitude_m`` is accepted for ``geofence_m``).  v0.8 (VTOL
        only): ``gps`` None -> mission.vtol.position_hold.enabled, False -> the v0.7.1 VTOL build (no GPS rules, TELEM
        v2); ``takeoff_check`` None -> on when position_hold.takeoff_check_ASSUMED exists."""
        m = p["mission"]
        hz = int(p["control"]["loop_hz"])
        mode = overrides.pop("mode", None)
        mode = lp.mission_mode(p) if mode is None else lp.check_mode(mode)
        profile = overrides.pop("profile", None)
        gps = overrides.pop("gps", None)
        tko = overrides.pop("takeoff_check", None)
        power = overrides.pop("power", None)
        imu_s, baro_s, src = I9_IMU_FAIL_S_SPEC, I9_BARO_FAIL_S_SPEC, "SPEC 8.8 I9 row (fc_config.json not found)"
        if fc_config_path and os.path.exists(fc_config_path):
            with open(fc_config_path, "r", encoding="utf-8") as f:
                cfg = json.load(f)
            if "imu_fail_kill_s" in cfg and "baro_fail_kill_s" in cfg:
                imu_s, baro_s = float(cfg["imu_fail_kill_s"]), float(cfg["baro_fail_kill_s"])
                src = os.path.relpath(fc_config_path, os.path.join(_HERE, "..")).replace("\\", "/")
        kw = dict(
            loop_hz=hz,
            climb_milli=lp.quantize(float(m["climb_throttle"]), lp.THROTTLE_SCALE, (0, lp.THROTTLE_SCALE)),
            ramp_ticks=ticks_ge(float(m["throttle_ramp_s"]), hz),
            coast_hold_ticks=ticks_ge(float(m.get("coast_control_hold_s", 0.0)), hz),
            tilt_abort_deg=float(m["tilt_abort_deg"]),
            arm_tilt_max_deg=float(m["arm_tilt_max_deg"]),
            min_cell_v=float(p["battery"]["min_cell_v"]),
            geofence_m=float(m["geofence_altitude_m"]),
            apogee_limit_m=float(m["apogee_limit_m"]),
            coast_drag_factor=float(m["coast_drag_factor"]),
            fall_vz_mps=float(m["fall_deploy_vz_mps"]),
            fall_confirm_ticks=max(1, ticks_ge(float(m["fall_deploy_confirm_s"]), hz)),
            link_abort_ticks=max(1, ticks_ge(float(m["link_loss_abort_s"]), hz)),
            link_disarm_ticks=max(1, ticks_ge(float(m["link_loss_disarm_s"]), hz)),
            max_motor_ticks=max(1, ticks_ge(float(m["max_motor_time_s"]), hz)),
            release_after_cut_ticks=ticks_ge(float(m["release_after_cut_s"]), hz),
            abort_release_ticks=ticks_ge(float(m["abort_release_delay_s"]), hz),
            tilt_rate_abort_rad_s=math.radians(float(m["tilt_rate_abort_dps"])),
            spin_rate_abort_rad_s=math.radians(float(m["spin_rate_abort_dps"])),
            rate_persist_ticks=max(1, ticks_ge(float(m["rate_abort_persist_s"]), hz)),
            gyro_saturation_abort=bool(m.get("gyro_saturation_abort", True)),
            imu_fail_ticks=max(1, ticks_ge(imu_s, hz)),
            baro_fail_ticks=max(1, ticks_ge(baro_s, hz)),
            i9_source=src,
        )
        if mode == lp.MODE_VTOL:
            kw.update(vtol_fields(m, hz, profile))
            if gps is not None:
                kw["gps_hold"] = bool(gps)
            if tko is not None:
                kw["takeoff_check"] = bool(tko)
            kw.update(power_fields(p, hz, profile, power, float(p["battery"]["min_cell_v"])))
        alias = {"geofence_altitude_m": "geofence_m"}          # params.json name -> field name
        for k, v in overrides.items():
            if v is None:
                continue
            kw[alias.get(k, k)] = v
        sp = cls(**kw)
        if sp.vtol:
            sp.check_vtol()
        return sp

    def check_vtol(self) -> None:
        """The VTOL parameter relations the rules rely on (ValueError otherwise)."""
        if not (0 < self.alt_min_dm <= self.hover_dm <= self.alt_max_dm and self.alt_step_dm > 0):
            raise ValueError(f"mission.vtol: need 0 < alt_min_m <= hover_altitude_m <= alt_max_m and alt_step_m > 0 "
                             f"(dm: {self.alt_min_dm}, {self.hover_dm}, {self.alt_max_dm}, step {self.alt_step_dm})")
        if not (self.alt_max_dm / lp.H_SCALE < self.ceiling_m < self.v_geofence_m):
            raise ValueError(f"mission.vtol: need alt_max_m < ceiling_m < geofence_m (an UP to alt_max_m must not trip "
                             f"the ceiling LAND, the ceiling LAND must come before the geofence ABORT): "
                             f"{self.alt_max_dm / lp.H_SCALE}, {self.ceiling_m}, {self.v_geofence_m}")
        if not (0 < self.endurance_ticks <= self.hard_cutoff_ticks):
            raise ValueError("mission.vtol: need 0 < max_flight_s - land_reserve_s <= hard_cutoff_s")
        if not self.v_fall_vz_mps < 0.0:
            raise ValueError("mission.vtol.fall_vz_mps must be negative")
        if not (self.land_timeout_ticks >= 0 and self.baro_limit_base_ticks >= 0 and self.baro_limit_mt_per_dm >= 0
                and self.v_geofence_dm >= 0):
            raise ValueError("mission.vtol: the v0.7.1 timeouts (touchdown.land_floor_timeout_s, baro_lost) must be >= 0")
        if self.v_geofence_dm * self.baro_limit_mt_per_dm > 0xFFFFFFFF:
            raise ValueError("mission.vtol.baro_lost: geofence_dm x milli-ticks per dm overflows uint32 (C port)")
        if self.power not in POWER_BUILDS + ("auto",):
            raise ValueError(f"power build {self.power!r} not in {POWER_BUILDS + ('auto',)}")
        if self.power_rules:
            lim = tether_limits(self)
            if not (0 < lim.alt_min_dm <= lim.hover_dm <= lim.alt_max_dm and lim.alt_max_dm / lp.H_SCALE < lim.ceiling_m
                    < lim.geofence_m and 0 < lim.endurance_ticks <= lim.hard_cutoff_ticks):
                raise ValueError(f"mission.vtol.power_rules_ASSUMED tether limits with the profile: need 0 < alt_min <= hover "
                                 f"<= alt_max < ceiling < geofence and 0 < endurance <= hard cutoff: {lim}")
            if not (self.pw_arm_lo_q < self.pw_arm_hi_q and self.pw_buf_arm_lo_q < self.pw_buf_arm_hi_q
                    and self.pw_sag_q < self.pw_sag_clear_q
                    and self.pw_buf_low_q < self.pw_buf_res_q < self.pw_buf_arm_lo_q < self.pw_buf_arm_hi_q
                    < self.pw_d2_q and self.pw_lost_q < self.pw_arm_lo_q and self.pw_sag_q < self.pw_arm_lo_q
                    and self.pw_sag_land_ticks >= self.pw_sag_warn_ticks and 0 <= self.pw_shift <= 8
                    and self.pw_arm_hi_q < 16 * 1024 and self.pw_d2_q < 16 * 1024):
                raise ValueError("mission.vtol.power_rules_ASSUMED: inconsistent thresholds (need buf_low < buffer ON window "
                                 "< d2, lost and sag below the ON window, sag hysteresis > 0, land after warn, Q4 codes "
                                 "inside the 10-bit ADC range)")

    def baro_limit_ticks(self, h_m: float) -> int:
        """V8 / V9: ticks from the baro loss to the time limit, from the NAV altitude at the loss (integer arithmetic,
        module docstring): base + ceil(h_dm x mt_per_dm / 1000), h_dm = round(h x 10) in [0, geofence_dm]; a
        non-finite h takes geofence_dm (the longest limit: the altitude is unknown)."""
        h_dm = metres_to_dm(h_m) if (h_m == h_m and not math.isinf(h_m)) else self.v_geofence_dm
        h_dm = min(max(h_dm, 0), self.v_geofence_dm)
        return self.baro_limit_base_ticks + _ceil_div(h_dm * self.baro_limit_mt_per_dm, 1000)


def vtol_profiles(p: dict) -> tuple[str, ...]:
    """The build profiles params.json offers: 'field' (mission.vtol as written) + every dict in mission.vtol.profiles."""
    profs = p["mission"]["vtol"].get("profiles", {})
    names = ["field"] + [k for k, val in profs.items() if isinstance(val, dict) and k != "field"]
    return tuple(names)


def vtol_values(p: dict, profile: str | None = None) -> dict:
    """mission.vtol merged with its build profile (v0.7.1; the single helper the sim, the firmware generator and the
    ground station use).  ``profile`` None -> mission.vtol.profile (absent: 'field').  'field' = mission.vtol as written
    (profiles.field, if present, is applied too - it is {} in params.json); any other name = mission.vtol with the keys of
    mission.vtol.profiles[name] replaced (a dict value is merged one level deep).  A profile may only override keys that
    mission.vtol has (a misspelt key must not silently do nothing).  The result is a deep copy without 'profiles' and with
    'profile' = the profile used.  Every safety rule is the same in every profile; only the listed numbers change."""
    m = p["mission"]
    if "vtol" not in m:
        raise ValueError("mission.mode = 'vtol' needs mission.vtol")
    v = m["vtol"]
    name = str(v.get("profile", "field")) if profile is None else str(profile)
    profs = v.get("profiles", {})
    if name not in vtol_profiles(p):
        raise ValueError(f"mission.vtol profile {name!r} not in {list(vtol_profiles(p))}")
    ov = profs.get(name, {})
    out = copy.deepcopy({k: val for k, val in v.items() if k != "profiles"})
    for k, val in ov.items():
        if k not in v or k in ("profile", "profiles"):
            raise ValueError(f"mission.vtol.profiles.{name}.{k}: mission.vtol has no key {k!r} to override")
        if isinstance(val, dict) and isinstance(out[k], dict):
            out[k] = {**out[k], **copy.deepcopy(val)}
        else:
            out[k] = copy.deepcopy(val)
    out["profile"] = name
    return out


def vtol_fields(m: dict, hz: int, profile: str | None = None) -> dict:
    """SupervisorParams VTOL fields from params.json ``mission`` (``mission.vtol`` merged with its build profile,
    ``vtol_values``)."""
    if "vtol" not in m:
        raise ValueError("mission.mode = 'vtol' needs mission.vtol")
    v = vtol_values({"mission": m}, profile)
    if not float(v["max_flight_s"]) > float(v["land_reserve_s"]) >= 0.0:
        raise ValueError("mission.vtol: need max_flight_s > land_reserve_s >= 0")
    td = v.get("touchdown", {})
    bl = v.get("baro_lost", {})
    fac = float(bl.get("time_limit_factor", 1.5))
    descent, flare, tdr = float(v["descent_rate_mps"]), float(v["flare_altitude_m"]), float(v["touchdown_rate_mps"])
    if not (fac >= 0 and descent > 0 and tdr > 0):
        raise ValueError("mission.vtol: need baro_lost.time_limit_factor >= 0, descent_rate_mps > 0, touchdown_rate_mps > 0")
    ph = v.get("position_hold") or {}
    return dict(
        land_timeout_ticks=ticks_ge(float(td.get("land_floor_timeout_s_ASSUMED", 0.0)), hz),
        baro_limit_base_ticks=ticks_ge(fac * flare / tdr + float(bl.get("time_limit_margin_s", 0.0)), hz),
        baro_limit_mt_per_dm=lp.round_half_away(fac * hz / (lp.H_SCALE * descent) * 1000.0),
        v_geofence_dm=metres_to_dm(v["geofence_m"]),
        profile=str(v["profile"]),
        gps_hold=bool(ph.get("enabled", False)),
        takeoff_check=bool(ph.get("takeoff_check_ASSUMED")),
        **_v16_fields(v, hz),
        **_vtol_fields_v070(v, hz))


def _v16_fields(v: dict, hz: int) -> dict:
    """v0.9 add-on C (V16 early yaw-authority LAND): mission.vtol.yaw_authority_land_ASSUMED {rate_dps, persist_s,
    enabled}; absent or not enabled -> off (no field set: the v0.8.1 supervisor)."""
    y = v.get("yaw_authority_land_ASSUMED") or {}
    if not y.get("enabled"):
        return {}
    return dict(v16_rate_rad_s=math.radians(float(y["rate_dps"])), v16_ticks=max(1, ticks_ge(float(y["persist_s"]), hz)))


POWER_BUILDS = ("battery", "wired", "wired_buffer")


@dataclass(frozen=True)
class FlightLimits:
    """The altitude / endurance limits the VTOL supervisor flies with (``tether_limits``): the profile's for the
    battery build, the smaller of the profile's and the tether block's when tethered."""

    hover_dm: int
    alt_min_dm: int
    alt_max_dm: int
    ceiling_m: float
    geofence_m: float
    endurance_ticks: int
    hard_cutoff_ticks: int


def tether_limits(sp: "SupervisorParams", tethered: bool = True) -> FlightLimits:
    """``FlightLimits`` of a build: ``tethered`` False (or no tether block) -> the profile's values exactly."""
    base = FlightLimits(sp.hover_dm, sp.alt_min_dm, sp.alt_max_dm, sp.ceiling_m, sp.v_geofence_m, sp.endurance_ticks,
                        sp.hard_cutoff_ticks)
    if not (tethered and sp.power_rules and sp.w_alt_max_dm > 0):
        return base

    def mn(a, b):
        return min(a, b) if b > 0 else a
    return FlightLimits(mn(sp.hover_dm, sp.w_hover_dm), mn(sp.alt_min_dm, sp.w_alt_min_dm), mn(sp.alt_max_dm, sp.w_alt_max_dm),
                        mn(sp.ceiling_m, sp.w_ceiling_m), mn(sp.v_geofence_m, sp.w_geofence_m),
                        sp.w_endurance_ticks or sp.endurance_ticks, sp.w_hard_cutoff_ticks or sp.hard_cutoff_ticks)


def q4_code(v: float, v_per_count: float) -> int:
    """A voltage as the power monitor's Q4 code: round(16 v / v_per_count) (identical in gen_firmware_params)."""
    return lp.round_half_away(16.0 * float(v) / float(v_per_count))


def power_fields(p: dict, hz: int, profile: str | None, power: str | None, min_cell_v: float) -> dict:
    """SupervisorParams power fields from params mission.vtol.power_rules_ASSUMED (merged with the build profile);
    ``power`` None -> 'auto' (the simulation: the plant's signals pick the build).  No block -> {} (battery build)."""
    v = vtol_values(p, profile)
    r = v.get("power_rules_ASSUMED")
    if not r:
        return {} if power in (None, "auto", "battery") else {"power": str(power)}
    vpc = float(r["adc_v_per_count"])
    q = lambda x: q4_code(x, vpc)          # noqa: E731
    tl = r.get("tether_limits") or {}
    te = r.get("tether_endurance") or {}
    cells = int(p["battery"]["cells"])
    bcells = 6
    d = dict(power="auto" if power is None else str(power), power_rules=True, pw_v_per_count=vpc,
             pw_shift=int(r.get("filter_shift", 3)),
             pw_arm_lo_q=q(r["arm_v_tether_min"]), pw_arm_hi_q=q(r["arm_v_tether_max"]),
             pw_buf_arm_lo_q=q(bcells * float(r["arm_buffer_cell_min"])),
             pw_buf_arm_hi_q=q(bcells * float(r["arm_buffer_cell_max"])),
             pw_lost_q=q(r["lost_v_min"]), pw_lost_margin_q=q(r["lost_below_buffer_v"]),
             pw_lost_ticks=max(1, ticks_ge(float(r["lost_persist_s"]), hz)),
             pw_sag_q=q(r["sag_v"]), pw_sag_clear_q=q(float(r["sag_v"]) + float(r["sag_hyst_v"])),
             pw_sag_warn_ticks=max(1, ticks_ge(float(r["sag_warn_s"]), hz)),
             pw_sag_land_ticks=max(1, ticks_ge(float(r["sag_land_s"]), hz)),
             pw_sag_release_ticks=max(1, ticks_ge(float(r["sag_release_s"]), hz)),
             pw_buf_low_q=q(bcells * float(r["buf_low_cell_v"])), pw_buf_ticks=max(1, ticks_ge(float(r["buf_persist_s"]), hz)),
             pw_buf_res_q=q(bcells * float(r["buf_reserve_cell_v"])),
             pw_buf_res_ticks=max(1, ticks_ge(float(r["buf_reserve_s"]), hz)),
             pw_d2_q=q(bcells * float(r["d2_cell_v"])), pw_d2_ticks=max(1, ticks_ge(float(r["d2_persist_s"]), hz)),
             pw_i8buf_q=q(bcells * float(min_cell_v)))
    if cells != 6:
        raise ValueError("power_rules_ASSUMED assumes 6-cell packs (battery.cells)")
    if tl:
        d.update(w_hover_dm=metres_to_dm(tl["hover_altitude_m"]), w_alt_min_dm=metres_to_dm(tl["alt_min_m"]),
                 w_alt_max_dm=metres_to_dm(tl["alt_max_m"]), w_ceiling_m=float(tl["ceiling_m"]),
                 w_geofence_m=float(tl["geofence_m"]))
    if te:
        if not float(te["max_flight_s"]) > float(te["land_reserve_s"]) >= 0.0:
            raise ValueError("power_rules_ASSUMED.tether_endurance: need max_flight_s > land_reserve_s >= 0")
        d.update(w_endurance_ticks=max(1, ticks_ge(float(te["max_flight_s"]) - float(te["land_reserve_s"]), hz)),
                 w_hard_cutoff_ticks=max(1, ticks_ge(float(te["hard_cutoff_s"]), hz)))
    return d


def _vtol_fields_v070(v: dict, hz: int) -> dict:
    """The v0.7.0 VTOL fields (from the merged values)."""
    return dict(
        mode=lp.MODE_VTOL,
        spool_ticks=ticks_ge(float(v["spool_warning_s"]), hz),
        hover_dm=metres_to_dm(v["hover_altitude_m"]),
        alt_step_dm=metres_to_dm(v["alt_step_m"]),
        alt_min_dm=metres_to_dm(v["alt_min_m"]),
        alt_max_dm=metres_to_dm(v["alt_max_m"]),
        link_land_ticks=max(1, ticks_ge(float(v["link_loss_land_s"]), hz)),
        endurance_ticks=max(1, ticks_ge(float(v["max_flight_s"]) - float(v["land_reserve_s"]), hz)),
        hard_cutoff_ticks=max(1, ticks_ge(float(v["hard_cutoff_s"]), hz)),
        ceiling_m=float(v["ceiling_m"]),
        v_geofence_m=float(v["geofence_m"]),
        v_fall_vz_mps=float(v["fall_vz_mps"]),
        v_fall_confirm_ticks=max(1, ticks_ge(float(v["fall_confirm_s"]), hz)),
        safe_after_landed_ticks=ticks_ge(float(v["safe_after_landed_s"]), hz),
    )


# --------------------------------------------------------------------------- #
# inputs / outputs
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class UplinkFrame:
    """A valid CMD frame for this vehicle (CRC, LEN, VEH and TYPE already checked by the decoder)."""

    seq: int
    cmd: lp.Command

    @classmethod
    def from_frame(cls, fr: lp.Frame) -> "UplinkFrame":
        return cls(fr.seq, lp.decode_command(fr))


@dataclass
class OnboardInputs:
    tick: int                                     # control tick since boot
    frames: list[UplinkFrame] = field(default_factory=list)   # received since the previous tick, oldest first
    h_m: float = 0.0                              # altitude filter (above the pad)
    vz_mps: float = 0.0
    tilt_deg: float = 0.0                         # attitude estimate
    gyro_rad_s: tuple[float, float, float] = (0.0, 0.0, 0.0)  # bias-corrected body rates
    gyro_saturated: bool = False                  # any gyro axis at full scale this tick
    v_cell: float = 4.2                           # filtered battery voltage per cell
    imu_ok: bool = True                           # this tick's IMU read succeeded
    baro_ok: bool = True                          # the baro is delivering samples on schedule
    aligned: bool = True                          # pad alignment complete
    arm_switch: bool = True                       # debounced, True = closed
    roll_deg: float = 0.0                         # telemetry only
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    loop_max_us: float = 0.0
    # v0.7 VTOL build (ignored by the ballistic build): the vertical controller's flags of the PREVIOUS tick
    vc_touchdown: bool = False                    # touchdown confirmed (LAND -> LANDED)
    vc_at_target: bool = False                    # reference reached the target (CLIMB -> HOLD)
    throttle_prev: float = 0.0                    # ESC throttle applied in the previous tick (TELEM v2 throttle)
    vc_ground_signature: bool = False             # v0.7.1: support or floor present (V8 / V9 baro-lost time limit)
    vc_low_in_land_ticks: int = 0                 # v0.7.1: consecutive LAND ticks low, baro healthy (V10 floor timeout;
                                                  # v0.7.1b: in the flare band only)
    # v0.7.1 VTOL NAV frame fields for the vertical controller (FcuStack; the supervisor does not read them)
    q_nav: tuple | None = None                    # NAV quaternion (w, x, y, z): cos(tilt) = 1 - 2 (x^2 + y^2)
    accel_z_mps2: float | None = None             # NAV body-z specific force (nlink accel[2]): the support signature
    # ---- v0.8 (VTOL + GPS) ----
    cos_tilt: float | None = None                 # nlink v3 cos_tilt (the vertical controller's c; wins over q_nav)
    gps_slot: bytes | None = None                 # nlink v3 GPS slot of this tick's valid NAV frame (FcuStack)
    gps_gate_ok: bool = False                     # ON gate: GATE_OK of the newest fresh complete slot set (FcuStack)
    pc_requests: int = 0                          # the position controller's REQ_* of the previous tick (FcuStack)
    vc_takeoff_fail: bool = False                 # the vertical controller's take-off check of the previous tick
    tlm_gps: tuple | None = None                  # TELEM v3: (gps_flags, num_sv, hacc_dm, pos_mode, dist_m) (FcuStack)
    # ---- v0.9 power (the ADC readings of the previous tick in volts; None = not measured on this hardware; the 6-DOF
    # fills them from sim.tether.PowerSystem.measure) ----
    v_in: float | None = None                     # V_in = the bus (PF0 divider)
    v_tether: float | None = None                 # before D1 (tethered hardware)
    v_buffer: float | None = None                 # before D2 (hybrid hardware)
    buf_i_pos: bool | None = None                 # the hybrid board's buffer-current sign (discharging)
    vc_takeoff_passed: bool = True                # the vertical controller has climbed takeoff_climb_m (previous tick;
                                                  # FcuStack; True without a take-off check): a power rule before it -> SAFE
    vc_support: bool = False                      # v0.9 fix round: the vertical controller's support signature (previous
    vc_airborne: bool = False                     # tick) and its lift-off latch - SAFE only on the pad (control review HIGH 1)


@dataclass
class OnboardOutputs:
    state: State
    throttle_target: float
    throttle: float                               # slewed command to the ESC / thrust estimate (ballistic; VTOL: 0,
                                                  # the ESC follows the vertical controller while motor_on)
    throttle_milli: int
    control_active: bool
    vanes_centre: bool
    release: bool                                 # release servo open
    reason: int
    flags: int
    ack_seq: int
    nonce: int
    events: list[str]                             # transitions / rejections of this tick
    reply_seq: int | None                         # SEQ of the uplink frame to answer this tick (None: no reply)
    telemetry: lp.Telemetry | None                # the reply's payload
    phase: int = lp.PHASE_NONE                    # VTOL phase (ballistic: NONE)
    h_tgt_m: float = 0.0                          # VTOL altitude target (SPOOL / CLIMB / HOLD; else 0)
    motor_on: bool = False                        # state POWERED: the ESC may follow the controller (VTOL)
    buzzer: bool = False                          # VTOL spool warning (ARMED)
    baro_lost: bool = False                       # VTOL: land on the NAV's dead reckoning (no baro)
    u_cap: float | None = None                    # v0.9 W_SAG: the frozen throttle ceiling for the vertical controller
    pwr: "pl.PowerTelem | None" = None            # v0.9: the TELEM v4 power tail of this tick (VTOL power builds)


# --------------------------------------------------------------------------- #
# v0.9 power monitor (module docstring "v0.9 power rules")
# --------------------------------------------------------------------------- #
class PowerMonitor:
    """Filters and persistence counters of the power rules (integer arithmetic; C: ``core/power_mon.c``).  ``sample``
    once per tick before the commands; the supervisor reads the conditions."""

    def __init__(self, sp: SupervisorParams):
        self.sp = sp
        self.y_in: int | None = None             # Q4 filtered codes (None until the first sample)
        self.y_tet: int | None = None
        self.y_buf: int | None = None
        self.sag_cnt = 0                         # W_SAG count (hysteresis), POWERED only
        self.rel_cnt = 0                         # above the clear level while the warning is on
        self.sag_warn = False                    # stage 1: ceiling frozen, UP refused
        self.u_cap: float | None = None
        self.lost_cnt = 0
        self.lost = False                        # W_LOST latched for the flight
        self.buf_cnt = 0
        self.res_cnt = 0                         # the buffer below its landing reserve
        self.d2_cnt = 0
        self.buf_share = False
        self.note: str | None = None             # a warning transition of this tick (the supervisor logs it)

    def _code(self, v: float) -> int:
        c = lp.round_half_away(float(v) / self.sp.pw_v_per_count) if v == v else 0
        return min(max(c, 0), 1023)

    def _filt(self, y: int | None, v: float | None) -> int | None:
        if v is None:
            return y
        x = self._code(v) << 4
        return x if y is None else y + ((x - y) >> self.sp.pw_shift)

    def reset_flight(self) -> None:
        self.sag_cnt = self.rel_cnt = self.lost_cnt = self.buf_cnt = self.res_cnt = self.d2_cnt = 0
        self.sag_warn = self.lost = False
        self.u_cap = None

    def sample(self, inp: OnboardInputs, build: int, powered: bool) -> None:
        sp = self.sp
        self.note = None
        self.y_in = self._filt(self.y_in, inp.v_in)
        self.y_tet = self._filt(self.y_tet, inp.v_tether)
        self.y_buf = self._filt(self.y_buf, inp.v_buffer)
        self.buf_share = bool(inp.buf_i_pos)
        if not powered:
            self.reset_flight()
            return
        if build == pl.PWR_WIRED and self.y_in is not None:
            y = self.y_in
            if y < sp.pw_sag_q:
                self.sag_cnt = min(self.sag_cnt + 1, 0xFFFF)
            elif y > sp.pw_sag_clear_q:
                self.sag_cnt = 0
            if not self.sag_warn and self.sag_cnt >= sp.pw_sag_warn_ticks:
                self.sag_warn = True
                self.u_cap = max(float(inp.throttle_prev), 0.0)
                self.rel_cnt = 0
                self.note = (f"W_SAG warning: V_in {self.volts(y):.2f} V below {self.volts(sp.pw_sag_q):.2f} V for "
                             f"{self.sag_cnt} ticks: throttle ceiling frozen at {self.u_cap:.3f}, UP refused")
            elif self.sag_warn:
                self.rel_cnt = self.rel_cnt + 1 if y > sp.pw_sag_clear_q else 0
                if self.rel_cnt >= sp.pw_sag_release_ticks:
                    self.sag_warn = False
                    self.u_cap = None
                    self.rel_cnt = 0
                    self.note = f"W_SAG warning cleared: V_in {self.volts(y):.2f} V for {sp.pw_sag_release_ticks} ticks"
        elif build == pl.PWR_WIRED_BUFFER and self.y_tet is not None and self.y_buf is not None:
            lost_now = self.y_tet < sp.pw_lost_q or self.y_tet + sp.pw_lost_margin_q < self.y_buf
            self.lost_cnt = min(self.lost_cnt + 1, 0xFF) if lost_now else 0
            if self.lost_cnt >= sp.pw_lost_ticks:
                self.lost = True
            self.buf_cnt = min(self.buf_cnt + 1, 0xFF) if self.y_buf < sp.pw_buf_low_q else 0
            self.res_cnt = min(self.res_cnt + 1, 0xFFFF) if self.y_buf < sp.pw_buf_res_q else 0
            d2_now = self.y_buf > sp.pw_d2_q and not lost_now
            self.d2_cnt = min(self.d2_cnt + 1, 0xFF) if d2_now else 0

    # conditions (the supervisor's rules)
    def sag_land(self) -> bool:
        return self.sag_cnt >= self.sp.pw_sag_land_ticks

    def lost_cond(self) -> bool:
        return self.lost_cnt >= self.sp.pw_lost_ticks

    def buf_cond(self) -> bool:
        return self.buf_cnt >= self.sp.pw_buf_ticks or self.res_cnt >= self.sp.pw_buf_res_ticks

    def d2_cond(self) -> bool:
        return self.d2_cnt >= self.sp.pw_d2_ticks

    def i8_buf(self) -> bool:
        return self.y_buf is not None and self.y_buf < self.sp.pw_i8buf_q

    def arm_ok(self, build: int) -> bool:
        """W_ARM: the idle voltages inside the ON window (wired: V_in; wired_buffer: V_tether and the buffer)."""
        sp = self.sp
        if build == pl.PWR_WIRED:
            return self.y_in is not None and sp.pw_arm_lo_q <= self.y_in <= sp.pw_arm_hi_q
        if build == pl.PWR_WIRED_BUFFER:
            return (self.y_tet is not None and self.y_buf is not None and sp.pw_arm_lo_q <= self.y_tet <= sp.pw_arm_hi_q
                    and sp.pw_buf_arm_lo_q <= self.y_buf <= sp.pw_buf_arm_hi_q)
        return True

    def volts(self, y: int | None) -> float | None:
        return None if y is None else y * self.sp.pw_v_per_count / 16.0

    def flags(self, build: int) -> int:
        f = 0
        if build == pl.PWR_WIRED:
            if self.y_in is not None and self.y_in >= self.sp.pw_lost_q:
                f |= pl.PF_TETHER_OK
            if self.sag_warn:
                f |= pl.PF_SAG
        elif build == pl.PWR_WIRED_BUFFER:
            if self.y_tet is not None and self.y_buf is not None and not (
                    self.y_tet < self.sp.pw_lost_q or self.y_tet + self.sp.pw_lost_margin_q < self.y_buf):
                f |= pl.PF_TETHER_OK
            if self.y_buf is not None and self.y_buf < self.sp.pw_buf_low_q:
                f |= pl.PF_BUF_LOW
            if self.buf_share:
                f |= pl.PF_BUF_SHARE
            if self.lost:
                f |= pl.PF_LOST
            if self.y_buf is not None and self.y_buf > self.sp.pw_d2_q:
                f |= pl.PF_D2
        return f


# --------------------------------------------------------------------------- #
# the supervisor
# --------------------------------------------------------------------------- #
class OnboardSupervisor:
    """SPEC §8.8 / §8.9 reference.  Call ``update`` exactly once per control tick with increasing ticks."""

    def __init__(self, sp: SupervisorParams, seed: int = 0, reset_in_flight: bool = False):
        self.sp = sp
        self.state = State.SAFE
        self.t_entry = 0
        self.nonce = lfsr16_next(seed & 0xFFFF)
        self.has_seq = False
        self.last_seq = 0
        self.ack_rejected = False
        self.last_frame_tick: int | None = None
        self.reason = lp.REASON_NONE
        self.interlock_latched = False
        self.target_milli = 0
        self.level = 0
        self.out_milli = 0
        self.powered_ticks = 0
        self.t_unpowered: int | None = None
        self.release_open = False
        self.release_deadline: int | None = None
        self.rate_count = 0
        self.v16_count = 0                       # v0.9 add-on C
        self.fall_count = 0
        self.imu_fail = 0
        self.baro_fail = 0
        self.phase = lp.PHASE_NONE               # VTOL
        self.tgt_dm = 0                          # VTOL altitude target [0.1 m]
        self.baro_lost = False                   # VTOL: latched baro failure in flight (until the next ON)
        self.baro_lost_tick = 0                  # v0.7.1: tick the baro-lost landing started (V8 / V9)
        self.baro_limit = 0                      # v0.7.1: its time limit [ticks] from the altitude at that tick
        # v0.9 power rules: the monitor (VTOL with the params block), the build (resolved while SAFE for 'auto'), the
        # limits it flies with
        self.pm = PowerMonitor(sp) if (sp.vtol and sp.power_rules) else None
        self.auto_power = sp.power == "auto"
        self.build = pl.PWR_BATTERY if sp.power in ("auto", "battery") else pl.build_of_source(sp.power)
        self.lim = tether_limits(sp, tethered=self.build != pl.PWR_BATTERY)
        self.lost_noted = False                  # W_LOST during LAND: reason latched once
        self.events: list[tuple[int, str]] = []
        self.exec_log: list[dict] = []           # every processed (non-NONE) command
        if reset_in_flight:
            self.reason = lp.R_I10_RESET
            self.interlock_latched = True
            if sp.vtol:
                self.state = State.ABORT
                self.release_deadline = sp.abort_release_ticks
                self.events.append((0, f"boot: I10 reset in flight -> ABORT (motor off), release in "
                                       f"{sp.abort_release_ticks} ticks"))
            else:
                self.release_open = True
                self.events.append((0, "boot: I10 reset in flight -> SAFE with the release open"))

    @property
    def vtol(self) -> bool:
        return self.sp.vtol

    # ---- helpers -------------------------------------------------------------------------------
    def predicted_apogee(self, h: float, vz: float) -> float:
        v = vz if vz > 0.0 else 0.0
        return h + self.sp.apogee_k * v * v

    def ticks_since_frame(self, t: int) -> int:
        return t - self.last_frame_tick if self.last_frame_tick is not None else 1 << 30

    def link_ok(self, t: int) -> bool:
        return self.ticks_since_frame(t) < self.sp.link_disarm_ticks

    def _enter(self, new: State, t: int, why: str) -> None:
        self.events.append((t, f"{self.state.name} -> {new.name}: {why}"))
        self.state = new
        self.t_entry = t
        if new != State.POWERED:
            self.target_milli = 0
        if new == State.UNPOWERED:
            self.t_unpowered = t
        if new == State.SAFE:
            self.release_deadline = None          # a release pending on the pad is cancelled
        if self.vtol:                             # the phase each state starts in (POWERED is entered from ARMED only)
            self.phase = {State.ARMED: lp.PHASE_SPOOL, State.POWERED: lp.PHASE_CLIMB,
                          State.UNPOWERED: lp.PHASE_LANDED}.get(new, lp.PHASE_NONE)

    def _set_phase(self, new: int, t: int, why: str) -> None:
        """A phase change inside POWERED (VTOL): CLIMB -> HOLD, CLIMB / HOLD -> LAND."""
        self.events.append((t, f"{self.state.name}: phase {lp.PHASE_NAMES[self.phase]} -> {lp.PHASE_NAMES[new]}: {why}"))
        self.phase = new

    def _interlock(self, code: int) -> None:
        self.reason = code
        self.interlock_latched = True

    def _abort(self, t: int, why: str) -> None:
        d = t + self.sp.abort_release_ticks
        self.release_deadline = d if self.release_deadline is None else min(self.release_deadline, d)
        self._enter(State.ABORT, t, why)

    def _start_release(self, t: int, why: str) -> None:
        self.release_deadline = t + self.sp.release_after_cut_ticks
        self.target_milli = 0
        if self.state == State.POWERED:
            self._enter(State.UNPOWERED, t, why)
        else:
            self.events.append((t, f"{self.state.name}: {why}"))

    def _kill(self, t: int, why: str) -> None:
        if self.state == State.ARMED:
            self._enter(State.SAFE, t, why)
        elif self.state in FLIGHT_STATES:
            self._abort(t, why)

    def h_tgt_m(self) -> float:
        """VTOL altitude target [m] as reported (TELEM v2) and given to the vertical controller: the target in
        SPOOL / CLIMB / HOLD, 0 otherwise (LAND descends to the ground, which may lie below the pad)."""
        return self.tgt_dm / lp.H_SCALE if self.phase in TARGET_PHASES else 0.0

    # ---- commands --------------------------------------------------------------------------------
    def _execute(self, fr: UplinkFrame, t: int, inp: OnboardInputs, link_ok_before: bool) -> int:
        """Execute one new command; returns 0 or the REJ_* code."""
        if self.vtol:
            return self._execute_vtol(fr, t, inp, link_ok_before)
        c, s, sp = fr.cmd.cmd, self.state, self.sp
        tag = f"{fr.cmd.name} (seq {fr.seq})"
        if c not in lp.CMD_NAMES or c == lp.CMD_NONE or c in lp.VTOL_ONLY_CMDS:
            return lp.REJ_UNKNOWN_CMD                  # v0.7 VTOL codes: unknown to the ballistic build, as in v0.6.3
        if c == lp.CMD_KILL:
            if s == State.SAFE:
                return lp.REJ_STATE
            if s in (State.ARMED,) + FLIGHT_STATES:
                self._kill(t, tag)
            return 0                                   # ABORT / RELEASED: already safe
        if self.release_deadline is not None:
            return lp.REJ_RELEASE_PENDING
        if c == lp.CMD_ARM:
            if s != State.SAFE:
                return lp.REJ_STATE
            code = self._arm_gates(inp, link_ok_before)
            if code:
                return code
            self._enter(State.ARMED, t, f"{tag}, tilt {inp.tilt_deg:.1f} deg, {inp.v_cell:.2f} V/cell")
            return 0
        if c == lp.CMD_DISARM:
            if s != State.ARMED:
                return lp.REJ_STATE
            self._enter(State.SAFE, t, tag)
            return 0
        if c == lp.CMD_LAUNCH:
            if s != State.ARMED:
                return lp.REJ_STATE
            if fr.cmd.nonce != self.nonce:
                return lp.REJ_NONCE
            self.nonce = lfsr16_next(self.nonce)
            self._enter(State.POWERED, t, tag)
            self.target_milli = sp.climb_milli
            return 0
        if c == lp.CMD_THROTTLE:
            u = min(fr.cmd.arg, sp.climb_milli)
            if s == State.ARMED:
                if fr.cmd.nonce != self.nonce:
                    return lp.REJ_NONCE
                if u == 0:
                    return lp.REJ_THROTTLE_ZERO
                self.nonce = lfsr16_next(self.nonce)
                self._enter(State.POWERED, t, f"{tag} u = {u / 1000:.3f}")
                self.target_milli = u
                return 0
            if s == State.POWERED:
                if u == 0:
                    self._enter(State.UNPOWERED, t, f"{tag} u = 0")
                else:
                    self.target_milli = u
                    self.events.append((t, f"POWERED: {tag} u = {u / 1000:.3f}"))
                return 0
            if s == State.UNPOWERED:
                if u > 0:
                    self._enter(State.POWERED, t, f"{tag} u = {u / 1000:.3f}")
                    self.target_milli = u
                return 0
            return lp.REJ_STATE
        if c == lp.CMD_CUT:
            if s != State.POWERED:
                return lp.REJ_STATE
            self._enter(State.UNPOWERED, t, tag)
            return 0
        if c == lp.CMD_RELEASE:
            if s not in (State.ARMED,) + FLIGHT_STATES:
                return lp.REJ_STATE
            self._start_release(t, f"{tag}: throttle 0, release in {sp.release_after_cut_ticks} ticks")
            return 0
        return lp.REJ_UNKNOWN_CMD  # pragma: no cover

    def _arm_gates(self, inp: OnboardInputs, link_ok_before: bool) -> int:
        """The ARM gates (ballistic ARM, VTOL ON) in their order: 0 or the REJ_* code of the first that fails."""
        sp = self.sp
        if self.release_open:
            return lp.REJ_RELEASED
        if not inp.arm_switch:
            return lp.REJ_ARM_SWITCH
        if not (inp.imu_ok and inp.baro_ok and self.imu_fail == 0 and self.baro_fail == 0):
            return lp.REJ_SENSORS
        if not inp.aligned:
            return lp.REJ_NOT_ALIGNED
        if sp.gps_hold and not inp.gps_gate_ok:                # v0.8 VTOL ON: the GPS must be ready
            return lp.REJ_GPS
        if not inp.tilt_deg < sp.arm_tilt_max_deg:
            return lp.REJ_TILT
        if self.pm is not None and self.build != pl.PWR_BATTERY:
            if not self.pm.arm_ok(self.build):                  # v0.9 W_ARM: the supply / buffer at idle
                return pl.REJ_POWER
        elif not inp.v_cell >= sp.min_cell_v:
            return lp.REJ_VCELL
        if not link_ok_before:
            return lp.REJ_LINK
        return 0

    def _execute_vtol(self, fr: UplinkFrame, t: int, inp: OnboardInputs, link_ok_before: bool) -> int:
        """VTOL command table (module docstring); returns 0 or the REJ_* code."""
        c, s, sp = fr.cmd.cmd, self.state, self.sp
        tag = f"{fr.cmd.name} (seq {fr.seq})"
        if c not in lp.CMD_NAMES or c == lp.CMD_NONE:
            return lp.REJ_UNKNOWN_CMD
        if c == lp.CMD_KILL:
            if s == State.SAFE:
                return lp.REJ_STATE
            if s in (State.ARMED,) + FLIGHT_STATES:
                self._kill(t, tag)
            return 0                                   # ABORT / RELEASED: already safe
        if c in lp.BALLISTIC_ONLY_CMDS:
            return lp.REJ_MODE
        if self.release_deadline is not None:
            return lp.REJ_RELEASE_PENDING
        if c == lp.CMD_ON:
            if s != State.SAFE:
                return lp.REJ_STATE
            if fr.cmd.nonce != self.nonce:
                return lp.REJ_NONCE
            code = self._arm_gates(inp, link_ok_before)
            if code:
                return code
            if self.powered_ticks >= self.lim.endurance_ticks:
                return lp.REJ_ENDURANCE
            self.nonce = lfsr16_next(self.nonce)
            self.tgt_dm = self.lim.hover_dm
            self.baro_lost = False
            self.lost_noted = False
            self._enter(State.ARMED, t, f"{tag}: spool warning {sp.spool_ticks} ticks, then CLIMB to "
                                        f"{self.lim.hover_dm / lp.H_SCALE:.1f} m (tilt {inp.tilt_deg:.1f} deg, "
                                        f"{inp.v_cell:.2f} V/cell{self._power_note()})")
            return 0
        if c == lp.CMD_OFF:
            if s == State.ARMED:
                self._enter(State.SAFE, t, f"{tag}: spool-up cancelled")
                return 0
            if s == State.POWERED and self.phase in FLY_PHASES:
                self._set_phase(lp.PHASE_LAND, t, tag)
                return 0
            if s in FLIGHT_STATES:                     # LAND / LANDED: already landing - acknowledged, no change
                self.events.append((t, f"{s.name}: {tag} ignored ({lp.PHASE_NAMES[self.phase]}: already landing)"))
                return 0
            return lp.REJ_STATE
        if c in (lp.CMD_UP, lp.CMD_DOWN):
            if s != State.POWERED:
                return lp.REJ_STATE
            if self.phase not in FLY_PHASES:
                return lp.REJ_LANDING
            if c == lp.CMD_UP and self.pm is not None and self.pm.sag_warn:
                return pl.REJ_POWER                              # v0.9 W_SAG stage 1: no more power demand
            new = self.tgt_dm + (sp.alt_step_dm if c == lp.CMD_UP else -sp.alt_step_dm)
            if not self.lim.alt_min_dm <= new <= self.lim.alt_max_dm:
                return lp.REJ_ALT_LIMIT
            self.events.append((t, f"POWERED: {tag} target {self.tgt_dm / lp.H_SCALE:.1f} -> {new / lp.H_SCALE:.1f} m"))
            self.tgt_dm = new
            return 0
        return lp.REJ_UNKNOWN_CMD  # pragma: no cover

    # ---- interlocks --------------------------------------------------------------------------------
    def _interlocks(self, t: int, inp: OnboardInputs) -> None:
        if self.vtol:
            self._rules_vtol(t, inp)
            return
        sp, s = self.sp, self.state
        # persistence counters on this tick's inputs (state after the commands)
        if s == State.POWERED and sp.rate_abort_enabled:
            wx, wy, wz = inp.gyro_rad_s
            bad = ((sp.gyro_saturation_abort and inp.gyro_saturated) or math.hypot(wx, wy) > sp.tilt_rate_abort_rad_s
                   or abs(wz) > sp.spin_rate_abort_rad_s)
            self.rate_count = self.rate_count + 1 if bad else 0
        else:
            self.rate_count = 0
        if s in FLIGHT_STATES and inp.vz_mps <= sp.fall_vz_mps:
            self.fall_count += 1
        else:
            self.fall_count = 0
        if s not in MOTOR_ENABLED_STATES:
            return
        on_pad = s == State.ARMED
        if not inp.arm_switch:
            self._interlock(lp.R_ARM_SWITCH_OPEN)
            self._kill(t, "arm switch opened (KILL)")
            return
        if self.imu_fail >= sp.imu_fail_ticks or self.baro_fail >= sp.baro_fail_ticks:
            code = lp.R_I9_IMU if self.imu_fail >= sp.imu_fail_ticks else lp.R_I9_BARO
            self._interlock(code)
            why = f"{lp.REASON_NAMES[code]} ({max(self.imu_fail, self.baro_fail)} ticks)"
            if on_pad:
                self._enter(State.SAFE, t, why)
            else:
                self._abort(t, why)
            return
        if on_pad:
            if self.ticks_since_frame(t) >= sp.link_disarm_ticks:
                self._interlock(lp.R_I1P_LINK_LOSS_PAD)
                self._enter(State.SAFE, t, f"I1p no ground frame for {self.ticks_since_frame(t)} ticks")
            return
        # POWERED / UNPOWERED
        if self.ticks_since_frame(t) >= sp.link_abort_ticks:
            self._interlock(lp.R_I1_LINK_LOSS)
            self._abort(t, f"I1 no ground frame for {self.ticks_since_frame(t)} ticks")
            return
        if s == State.POWERED:
            if inp.tilt_deg > sp.tilt_abort_deg:
                self._interlock(lp.R_I2_TILT)
                self._abort(t, f"I2 tilt {inp.tilt_deg:.1f} deg > {sp.tilt_abort_deg:.0f}")
                return
            if self.rate_count >= sp.rate_persist_ticks:
                wx, wy, wz = inp.gyro_rad_s
                self._interlock(lp.R_I3_RATE)
                self._abort(t, f"I3 |w_xy| {math.degrees(math.hypot(wx, wy)):.0f} deg/s, |w_z| {math.degrees(abs(wz)):.0f} "
                               f"deg/s{' (gyro at full scale)' if inp.gyro_saturated else ''} for {self.rate_count} ticks")
                return
            if inp.v_cell < sp.min_cell_v:
                self._interlock(lp.R_I8_VCELL)
                self._abort(t, f"I8 V_cell {inp.v_cell:.2f} < {sp.min_cell_v:.2f}")
                return
        if inp.h_m > sp.geofence_m:
            self._interlock(lp.R_I5_GEOFENCE)
            self._abort(t, f"I5 geofence h {inp.h_m:.1f} m > {sp.geofence_m:.0f}")
            return
        if s == State.POWERED:
            pred = self.predicted_apogee(inp.h_m, inp.vz_mps)
            if pred >= sp.apogee_limit_m:
                self._interlock(lp.R_I4_APOGEE_LIMIT)
                self._enter(State.UNPOWERED, t, f"I4 apogee limiter: h {inp.h_m:.1f} m, v_z {inp.vz_mps:.1f} m/s, "
                                                f"predicted {pred:.1f} m >= {sp.apogee_limit_m:.0f}")
                return
            if self.powered_ticks >= sp.max_motor_ticks:
                self._interlock(lp.R_I7_MOTOR_TIME)
                self._enter(State.UNPOWERED, t, f"I7 motor time {self.powered_ticks} ticks")
                return
        if not self.release_open and self.release_deadline is None and self.fall_count >= sp.fall_confirm_ticks:
            self._interlock(lp.R_I6_FALL)
            self._start_release(t, f"I6 fall-safe: v_z {inp.vz_mps:.1f} m/s <= {sp.fall_vz_mps:.0f} for "
                                   f"{self.fall_count} ticks, release in {sp.release_after_cut_ticks} ticks")

    def _land(self, t: int, code: int, why: str) -> None:
        """A LAND rule acted (VTOL CLIMB / HOLD): latch the reason, phase LAND."""
        self._interlock(code)
        self._set_phase(lp.PHASE_LAND, t, why)

    def _rules_vtol(self, t: int, inp: OnboardInputs) -> None:
        """VTOL safety rules (module docstring, in this order; the first that acts ends the evaluation)."""
        sp, s = self.sp, self.state
        if s == State.POWERED and sp.rate_abort_enabled:
            wx, wy, wz = inp.gyro_rad_s
            bad = ((sp.gyro_saturation_abort and inp.gyro_saturated) or math.hypot(wx, wy) > sp.tilt_rate_abort_rad_s
                   or abs(wz) > sp.spin_rate_abort_rad_s)
            self.rate_count = self.rate_count + 1 if bad else 0
        else:
            self.rate_count = 0
        if s in FLIGHT_STATES and inp.vz_mps <= sp.v_fall_vz_mps:
            self.fall_count += 1
        else:
            self.fall_count = 0
        if sp.v16_ticks > 0 and s == State.POWERED and self.phase in FLY_PHASES and abs(inp.gyro_rad_s[2]) > sp.v16_rate_rad_s:
            self.v16_count += 1                  # v0.9 add-on C: |w_z| above the V16 rate in CLIMB / HOLD
        else:
            self.v16_count = 0
        if s not in MOTOR_ENABLED_STATES:
            return
        on_pad = s == State.ARMED
        if not inp.arm_switch:
            self._interlock(lp.R_ARM_SWITCH_OPEN)
            if s == State.UNPOWERED:               # LANDED: somebody is at the vehicle - SAFE, never fire the release
                self._enter(State.SAFE, t, "arm switch opened on the ground (LANDED)")
            else:
                self._kill(t, "arm switch opened (KILL)")
            return
        landed = s == State.UNPOWERED                # VTOL: UNPOWERED is LANDED (motor off, on the ground)
        if self.imu_fail >= sp.imu_fail_ticks:
            self._interlock(lp.R_I9_IMU)
            why = f"{lp.REASON_NAMES[lp.R_I9_IMU]} ({self.imu_fail} ticks)"
            if on_pad:
                self._enter(State.SAFE, t, why)
            elif landed:                             # v0.7.1: no release toward people at a landed vehicle
                self._enter(State.SAFE, t, f"{why} in LANDED: motor already off, no release")
            else:
                self._abort(t, why)
            return
        if self.baro_fail >= sp.baro_fail_ticks:
            why = f"{lp.REASON_NAMES[lp.R_I9_BARO]} ({self.baro_fail} ticks)"
            if on_pad:
                self._interlock(lp.R_I9_BARO)
                self._enter(State.SAFE, t, why)
                return
            if s == State.POWERED and not self.baro_lost:
                self.baro_lost = True
                self.baro_lost_tick = t
                self.baro_limit = sp.baro_limit_ticks(inp.h_m)
                lim = f"time limit {self.baro_limit} ticks from h {inp.h_m:.1f} m"
                if self.phase in FLY_PHASES:
                    self._land(t, lp.R_I9_BARO, f"{why}: land on the dead-reckoned altitude, {lim}")
                else:
                    self._interlock(lp.R_I9_BARO)
                    self.events.append((t, f"POWERED: {why} during LAND: dead-reckoned altitude from now on, {lim}"))
                return
        if on_pad:
            if self.ticks_since_frame(t) >= sp.link_disarm_ticks:
                self._interlock(lp.R_I1P_LINK_LOSS_PAD)
                self._enter(State.SAFE, t, f"I1p no ground frame for {self.ticks_since_frame(t)} ticks")
            return
        # POWERED / UNPOWERED (LANDED): ABORT rules
        if s == State.POWERED:
            if inp.tilt_deg > sp.tilt_abort_deg:
                self._interlock(lp.R_I2_TILT)
                self._abort(t, f"I2 tilt {inp.tilt_deg:.1f} deg > {sp.tilt_abort_deg:.0f}")
                return
            if self.rate_count >= sp.rate_persist_ticks:
                wx, wy, wz = inp.gyro_rad_s
                self._interlock(lp.R_I3_RATE)
                self._abort(t, f"I3 |w_xy| {math.degrees(math.hypot(wx, wy)):.0f} deg/s, |w_z| {math.degrees(abs(wz)):.0f} "
                               f"deg/s{' (gyro at full scale)' if inp.gyro_saturated else ''} for {self.rate_count} ticks")
                return
        if inp.h_m > self.lim.geofence_m:
            self._interlock(lp.R_V_GEOFENCE)
            why = f"V4 geofence h {inp.h_m:.1f} m > {self.lim.geofence_m:.1f}"
            if landed:                               # v0.7.1: on the ground this is a baro glitch - SAFE, no release
                self._enter(State.SAFE, t, f"{why} in LANDED: motor already off, no release")
            else:
                self._abort(t, why)
            return
        if self.fall_count >= sp.v_fall_confirm_ticks:
            self._interlock(lp.R_V_FALL)
            why = f"V5 fall: v_z {inp.vz_mps:.2f} m/s <= {sp.v_fall_vz_mps:.1f} for {self.fall_count} ticks"
            if landed:                               # v0.7.1 (landed_rules_note): SAFE, never ABORT, in LANDED
                self._enter(State.SAFE, t, f"{why} in LANDED: motor already off, no release")
            else:
                self._abort(t, why)
            return
        if s == State.POWERED and self.powered_ticks >= self.lim.hard_cutoff_ticks:
            self._interlock(lp.R_V_HARD_CUTOFF)
            self._abort(t, f"V6 hard cutoff: motor time {self.powered_ticks} ticks")
            return
        # v0.8 V15 take-off check: pinned at the ceiling without climbing -> motor off, SAFE (no release)
        if sp.takeoff_check and s == State.POWERED and self.phase in FLY_PHASES and inp.vc_takeoff_fail:
            self._interlock(lp.R_V_TAKEOFF_CHECK)
            self._enter(State.SAFE, t, "V15 take-off check: the vertical controller at its ceiling without the climb - "
                                       "motor off, no release")
            return
        # v0.7.1 POWERED LAND without a confirmed touchdown: baro-lost time limit (V8 / V9), floor timeout (V10)
        if s == State.POWERED and self.phase == lp.PHASE_LAND and not inp.vc_touchdown:
            if self.baro_lost:
                if t - self.baro_lost_tick >= self.baro_limit:
                    why = f"baro-lost time limit {self.baro_limit} ticks (since tick {self.baro_lost_tick})"
                    if inp.vc_ground_signature:
                        self._interlock(lp.R_V_BARO_LIMIT_LANDED)
                        self._enter(State.UNPOWERED, t, f"V8 {why}, ground signature: motor off (LANDED)")
                    else:
                        self._interlock(lp.R_V_BARO_LIMIT_ABORT)
                        self._abort(t, f"V9 {why}, no ground signature")
                    return
            elif sp.land_timeout_ticks > 0 and inp.vc_low_in_land_ticks >= sp.land_timeout_ticks:
                self._interlock(lp.R_V_LAND_TIMEOUT)
                self._enter(State.UNPOWERED, t, f"V10 LAND floor timeout: low for {inp.vc_low_in_land_ticks} ticks without "
                                                f"a confirmed touchdown: motor off (LANDED), no release")
                return
        # v0.9 W_LOST during LAND (already landing): the reason is latched once for the ground's alarm, nothing else
        pm = self.pm
        if (pm is not None and self.build == pl.PWR_WIRED_BUFFER and s == State.POWERED and self.phase == lp.PHASE_LAND
                and pm.lost_cond() and not self.lost_noted):
            self.lost_noted = True
            self._interlock(pl.R_W_LOST)
            self.events.append((t, f"POWERED: W_LOST tether lost during LAND (V_tether {pm.volts(pm.y_tet):.2f} V, buffer "
                                   f"{pm.volts(pm.y_buf):.2f} V): landing on the buffer"))
        # POWERED CLIMB / HOLD: LAND rules
        if s != State.POWERED or self.phase not in FLY_PHASES:
            return
        if self.ticks_since_frame(t) >= sp.link_land_ticks:
            self._land(t, lp.R_V_LINK_LAND, f"V1 no ground frame for {self.ticks_since_frame(t)} ticks")
            return
        if pm is None or self.build == pl.PWR_BATTERY:
            if inp.v_cell < sp.min_cell_v:
                self._land(t, lp.R_I8_VCELL, f"I8 V_cell {inp.v_cell:.2f} < {sp.min_cell_v:.2f}")
                return
        elif self._power_land_rules(t, inp):                # v0.9: the power build's rules in the I8 slot
            return
        if inp.h_m > self.lim.ceiling_m:
            self._land(t, lp.R_V_CEILING, f"V3 ceiling h {inp.h_m:.1f} m > {self.lim.ceiling_m:.1f}")
            return
        if self.powered_ticks >= self.lim.endurance_ticks:
            self._land(t, lp.R_V_ENDURANCE, f"V2 flight time: motor time {self.powered_ticks} ticks")
            return
        if sp.v16_ticks > 0 and self.v16_count >= sp.v16_ticks:
            # v0.9 add-on C (V16): the yaw rate has stayed above the V16 rate - land under power before the I3 cut
            # (reason: the V14 code R_V_AUTHORITY until link_proto has its own; ground text 'no authority: land')
            self._land(t, lp.R_V_AUTHORITY, f"V16 yaw authority: |w_z| > {math.degrees(sp.v16_rate_rad_s):.0f} deg/s for "
                                            f"{self.v16_count} ticks: land")
            return
        if sp.gps_hold:                                          # v0.8: the position controller's LAND requests
            rq = inp.pc_requests
            for bit, code, why in ((REQ_GPS_LOST, lp.R_V_GPS_LOST, "V11 GPS lost: attitude-only hold, land"),
                                   (REQ_FENCE, lp.R_V_FENCE, "V12 horizontal fence: land"),
                                   (REQ_IMPLAUSIBLE, lp.R_V_GPS_IMPLAUSIBLE, "V13 GPS implausible: attitude only, land"),
                                   (REQ_AUTHORITY, lp.R_V_AUTHORITY, "V14 no tilt authority for position hold: land")):
                if rq & bit:
                    self._land(t, code, why)
                    return

    # ---- v0.9 power rules -----------------------------------------------------------------------------
    def _power_act(self, t: int, inp: OnboardInputs, code: int, why: str) -> None:
        """A power rule acted in CLIMB / HOLD: LAND - or, with the vehicle ON THE PAD (the take-off check not passed, the
        support signature present and no lift-off seen), motor off -> SAFE like V15 (a LAND there would descend a
        reference from the target while the motor runs on the pad - into the supply's latch-off).
        v0.9 fix round (control review HIGH 1): the take-off check passes 1.4-2.7 s after the lift-off, 0.6-1.8 m up, so
        'not passed' alone stopped the motor IN THE AIR (hybrid unplug in the climb: 6/6 falls).  Off the pad - support
        absent (thrust near or above the weight) or the lift-off latched - the rule LANDs: on the pad a LAND confirms its
        touchdown by still / support within landed_confirm_s and stops the motor the normal way."""
        if inp.vc_takeoff_passed or not inp.vc_support or inp.vc_airborne:
            self._land(t, code, why)
        else:
            self._interlock(code)
            self._enter(State.SAFE, t, f"{why} - on the pad before the take-off: motor off (SAFE), no release")

    def _power_land_rules(self, t: int, inp: OnboardInputs) -> bool:
        """The power build's LAND rules in the I8 slot (POWERED CLIMB / HOLD); True when one acted."""
        pm = self.pm
        if self.build == pl.PWR_WIRED:
            if pm.sag_land():
                self._power_act(t, inp, pl.R_W_SAG, f"W_SAG supply sag: V_in {pm.volts(pm.y_in):.2f} V below "
                                f"{pm.volts(self.sp.pw_sag_q):.2f} V for {pm.sag_cnt} ticks (ceiling frozen at "
                                f"{pm.u_cap if pm.u_cap is not None else float('nan'):.3f}): land while power remains")
                return True
            return False
        if pm.lost_cond():
            self.lost_noted = True
            self._power_act(t, inp, pl.R_W_LOST, f"W_LOST tether lost: V_tether {pm.volts(pm.y_tet):.2f} V, buffer "
                            f"{pm.volts(pm.y_buf):.2f} V for {pm.lost_cnt} ticks: land on the buffer")
            return True
        if pm.buf_cond():
            self._power_act(t, inp, pl.R_W_BUF, f"W_BUF buffer low: {pm.volts(pm.y_buf) / 6.0:.2f} V/cell (under load "
                            f"{pm.buf_cnt} ticks / reserve {pm.res_cnt} ticks): land while it can carry the landing")
            return True
        if pm.d2_cond():
            self._power_act(t, inp, pl.R_W_D2, f"W_D2 buffer charging while tethered: {pm.volts(pm.y_buf) / 6.0:.2f} V/cell "
                            f"for {pm.d2_cnt} ticks (D2 short?): land, cut the supply on the ground")
            return True
        if pm.i8_buf():
            self._power_act(t, inp, lp.R_I8_VCELL, f"I8 buffer V_cell {pm.volts(pm.y_buf) / 6.0:.2f} < "
                                                   f"{self.sp.min_cell_v:.2f}")
            return True
        return False

    def _power_note(self) -> str:
        """The ON event's power text (empty in the battery build: its event text is the v0.8.1 one)."""
        pm = self.pm
        if pm is None or self.build == pl.PWR_BATTERY:
            return ""
        if self.build == pl.PWR_WIRED:
            return f", {pl.PWR_NAMES[self.build]}: V_in {pm.volts(pm.y_in):.2f} V"
        return (f", {pl.PWR_NAMES[self.build]}: V_tether {pm.volts(pm.y_tet):.2f} V, buffer "
                f"{pm.volts(pm.y_buf) / 6.0:.3f} V/cell")

    def _resolve_power(self, inp: OnboardInputs) -> None:
        """'auto' (simulation): the build of the plant's signals, while SAFE, never downgraded."""
        if not (self.auto_power and self.pm is not None and self.state == State.SAFE):
            return
        b = (pl.PWR_WIRED_BUFFER if inp.v_buffer is not None else pl.PWR_WIRED if inp.v_tether is not None
             else pl.PWR_BATTERY)
        if b > self.build:
            self.build = b
            self.lim = tether_limits(self.sp, tethered=True)

    def power_telem(self) -> "pl.PowerTelem | None":
        """The TELEM v4 / v5 power tail of this tick (None: no power monitor): v_aux = the buffer (wired_buffer) or the
        tether node (wired), the flags, the build."""
        pm = self.pm
        if pm is None:
            return None
        va = pm.volts(pm.y_buf if self.build == pl.PWR_WIRED_BUFFER else pm.y_in)
        return pl.PowerTelem.from_physical(va, pm.flags(self.build), self.build)

    def _timers_vtol(self, t: int, inp: OnboardInputs) -> None:
        """VTOL timers after the rules (module docstring)."""
        sp, s = self.sp, self.state
        if s == State.ARMED:
            if t - self.t_entry >= sp.spool_ticks:
                pw_ok = (inp.v_cell >= sp.min_cell_v if (self.pm is None or self.build == pl.PWR_BATTERY)
                         else self.pm.arm_ok(self.build))           # v0.9: the power build's ON window again
                if inp.tilt_deg < sp.arm_tilt_max_deg and pw_ok:
                    self._enter(State.POWERED, t, f"spool warning complete: motor on, CLIMB to "
                                                  f"{self.tgt_dm / lp.H_SCALE:.1f} m")
                else:
                    self._interlock(lp.R_V_TAKEOFF_GATE)
                    self._enter(State.SAFE, t, f"V7 take-off gate: tilt {inp.tilt_deg:.1f} deg, "
                                               f"{inp.v_cell:.2f} V/cell at the end of the spool warning")
        elif s == State.POWERED:
            if self.phase == lp.PHASE_CLIMB and inp.vc_at_target:
                self._set_phase(lp.PHASE_HOLD, t, f"reference at the target {self.tgt_dm / lp.H_SCALE:.1f} m")
            elif self.phase == lp.PHASE_LAND and inp.vc_touchdown:
                self._enter(State.UNPOWERED, t, "touchdown confirmed: motor off (LANDED)")
        elif s == State.UNPOWERED:
            if t - self.t_entry >= sp.safe_after_landed_ticks:
                self._enter(State.SAFE, t, f"landed for {t - self.t_entry} ticks")

    # ---- throttle ------------------------------------------------------------------------------------
    def _slew(self) -> None:
        tgt = self.target_milli if self.state == State.POWERED else 0
        r = self.sp.ramp_ticks
        if r <= 0:
            self.out_milli = tgt
            return
        if tgt < self.out_milli:                        # decreases are immediate
            self.out_milli = tgt
            self.level = _ceil_div(tgt * r, lp.THROTTLE_SCALE)
        elif tgt > self.out_milli:
            self.level = min(self.level + 1, r)
            self.out_milli = min(tgt, self.level * lp.THROTTLE_SCALE // r)

    # ---- main ----------------------------------------------------------------------------------------
    def control_active(self, t: int) -> bool:
        if self.state == State.POWERED:
            return True
        if self.vtol:                                   # VTOL: attitude control in POWERED only (LANDED: vanes centred)
            return False
        return (self.state == State.UNPOWERED and self.t_unpowered is not None
                and t - self.t_unpowered < self.sp.coast_hold_ticks)

    def flags(self, t: int, inp: OnboardInputs) -> int:
        f = 0
        if inp.arm_switch:
            f |= lp.FLAG_ARM_SWITCH
        if inp.imu_ok and inp.baro_ok and self.imu_fail == 0 and self.baro_fail == 0:
            f |= lp.FLAG_SENSORS_OK
        if inp.aligned:
            f |= lp.FLAG_ALIGNED
        if self.link_ok(t):
            f |= lp.FLAG_LINK_OK
        if self.release_open:
            f |= lp.FLAG_RELEASED
        if self.control_active(t):
            f |= lp.FLAG_CONTROL_ACTIVE
        if self.ack_rejected:
            f |= lp.FLAG_REJECTED
        if self.interlock_latched:
            f |= lp.FLAG_INTERLOCK
        return f

    def telemetry(self, t: int, inp: OnboardInputs) -> lp.Telemetry:
        """The reply's payload: TELEM v1 (ballistic) or v2 (VTOL: + phase, target; throttle = the ESC throttle applied
        in the previous tick, ``inp.throttle_prev`` - the vertical controller runs after this)."""
        thr = inp.throttle_prev if self.vtol else self.out_milli / lp.THROTTLE_SCALE
        gps = None
        if self.vtol and self.sp.gps_hold:                       # v0.8 TELEM v3
            gps = inp.tlm_gps if inp.tlm_gps is not None else (0, 0, lp.HACC_NONE, 0, math.nan)
        return lp.Telemetry.from_physical(
            state=int(self.state), flags=self.flags(t, inp), reason=self.reason,
            ack_seq=self.last_seq if self.has_seq else 0, nonce=self.nonce,
            t_ms=(t * 1000) // self.sp.loop_hz, h_m=inp.h_m, vz_mps=inp.vz_mps, roll_deg=inp.roll_deg,
            pitch_deg=inp.pitch_deg, yaw_deg=inp.yaw_deg, tilt_deg=inp.tilt_deg,
            throttle=thr, vcell_v=inp.v_cell, loop_max_us=inp.loop_max_us,
            phase=self.phase if self.vtol else None, h_tgt_m=self.h_tgt_m() if self.vtol else None, gps=gps)

    def update(self, inp: OnboardInputs) -> OnboardOutputs:
        t = inp.tick
        n_ev = len(self.events)
        self.imu_fail = 0 if inp.imu_ok else self.imu_fail + 1
        self.baro_fail = 0 if inp.baro_ok else self.baro_fail + 1
        if self.pm is not None:                        # v0.9 power monitor: the build, then this tick's readings
            self._resolve_power(inp)
            self.pm.sample(inp, self.build, self.state == State.POWERED)
            if self.pm.note:
                self.events.append((t, f"POWERED: {self.pm.note}"))
        # 1. received commands
        reply_seq: int | None = None
        for fr in inp.frames:
            link_ok_before = self.link_ok(t)
            self.last_frame_tick = t
            reply_seq = fr.seq
            if self.has_seq and not lp.seq_newer(fr.seq, self.last_seq):
                continue                                   # repeat of a processed SEQ (or older): reply only
            self.has_seq = True
            self.last_seq = fr.seq
            if fr.cmd.cmd == lp.CMD_NONE:
                self.ack_rejected = False
                continue
            state_before, phase_before = self.state, self.phase
            code = self._execute(fr, t, inp, link_ok_before)
            self.ack_rejected = code != 0
            if code:
                if not self.interlock_latched:         # the interlock code outranks rejections (ground reads flags)
                    self.reason = code
                # the ballistic build names the v0.7 codes as v0.6.3 did (unknown: "CMD8"), so its log text is unchanged
                nm = fr.cmd.name if self.vtol or fr.cmd.cmd not in lp.VTOL_ONLY_CMDS else f"CMD{fr.cmd.cmd}"
                self.events.append((t, f"REJECT {nm} (seq {fr.seq}) in {state_before.name}: "
                                       f"{lp.REASON_NAMES.get(code, code)}"))
            rec = {"tick": t, "seq": fr.seq, "cmd": fr.cmd.cmd, "code": code,
                   "state_before": int(state_before), "state_after": int(self.state)}
            if self.vtol:
                rec.update(phase_before=phase_before, phase_after=self.phase)
            self.exec_log.append(rec)
        # 2. interlocks
        self._interlocks(t, inp)
        # 3. release timer (pending release or ABORT)
        if self.release_deadline is not None and t >= self.release_deadline:
            self.release_deadline = None
            self.release_open = True
            self._enter(State.RELEASED, t, "release servo open")
        if self.vtol:
            # 3b. VTOL timers (spool end + take-off gate, CLIMB -> HOLD, touchdown, LANDED -> SAFE)
            self._timers_vtol(t, inp)
        else:
            # 4. throttle slew (ballistic; VTOL: the vertical controller after this update)
            self._slew()
        # the I7 / V2 / V6 clock: POWERED ticks since boot
        if self.state == State.POWERED:
            self.powered_ticks += 1
        active = self.control_active(t)
        return OnboardOutputs(
            state=self.state, throttle_target=self.target_milli / lp.THROTTLE_SCALE,
            throttle=self.out_milli / lp.THROTTLE_SCALE, throttle_milli=self.out_milli, control_active=active,
            vanes_centre=not active, release=self.release_open, reason=self.reason, flags=self.flags(t, inp),
            ack_seq=self.last_seq if self.has_seq else 0, nonce=self.nonce,
            events=[e for _, e in self.events[n_ev:]], reply_seq=reply_seq,
            telemetry=self.telemetry(t, inp) if reply_seq is not None else None,
            phase=self.phase, h_tgt_m=self.h_tgt_m() if self.vtol else 0.0, motor_on=self.state == State.POWERED,
            buzzer=self.vtol and self.state == State.ARMED, baro_lost=self.baro_lost,
            u_cap=(self.pm.u_cap if (self.pm is not None and self.pm.sag_warn and self.state == State.POWERED) else None),
            pwr=self.power_telem())


# --------------------------------------------------------------------------- #
# one FCU control tick: supervisor -> vertical controller -> ESC (the tick order of the module docstring)
# --------------------------------------------------------------------------- #
@dataclass
class FcuOutputs:
    sup: OnboardOutputs
    vc: "object | None"                           # sim.vertical_ctrl.VerticalOutputs (VTOL), None (ballistic)
    throttle: float                               # the ESC command of this tick (0..1)
    att_active: bool = False                      # the attitude controller of THIS tick runs (else vanes centred) ...
    att_u: float = 0.0                            # ... with this throttle in its gain schedule.  VTOL (v0.7.1): the
                                                  # previous tick's supervisor control_active and ESC throttle;
                                                  # ballistic: this tick's (order unchanged)
    att_tilt: tuple = (0.0, 0.0)                  # v0.8: ... and this tilt setpoint (level frame; built last tick)
    pos: "object | None" = None                   # v0.8: sim.position_ctrl.PosOutputs computed from this tick's inputs
                                                  # (None: no step); committed PIPELINE_TICKS later (FcuStack.pos_out)
    gps: "object | None" = None                   # v0.8: the GPS set completed this tick (sim.position_ctrl.GpsData;
                                                  # v0.8.1 gns chain: sim.nlink_proto.GnsData)
    tilt_next: tuple = (0.0, 0.0)                 # v0.8: the tilt setpoint the NEXT tick's attitude controller uses
    fcu4: "tuple | None" = None                   # v0.8.1 gns chain: (pflags, kt_q) of this tick for the nlink v4 FCU
                                                  # frame (sim.nlink_proto.Fcu4) the NAV forwards to the GNS


class FcuStack:
    """The FCU's per-tick sequence (normative order, module docstring "Tick order" / "FCU tick order v0.8").

    VTOL (v0.7.1): (1) the attitude controller runs first with ``att_active`` / ``att_u`` = the previous tick's
    supervisor ``control_active`` and ESC throttle (the caller runs it and writes the vanes - this class holds no
    attitude controller); (3) the supervisor with the vertical controller's flags (touchdown, at_target,
    ground_signature, low_in_land_ticks) and the ESC throttle of the previous tick; (4) the vertical controller with
    cos(tilt) from ``inp.cos_tilt`` (nlink v3), else ``inp.q_nav`` (1 - 2 (x^2 + y^2)), else ``tilt_deg``, and the body-z
    specific force ``inp.accel_z_mps2``; (5) the ESC command = its throttle while ``motor_on``.  v0.8 (``sp.gps_hold``):
    (0) the slot receiver, (6) the position step when due, (7) the tilt setpoint builder - ``att_tilt`` is the tilt for
    this tick's attitude controller (built at the end of the previous tick), ``tilt_next`` the one for the next.
    Ballistic: the supervisor's slewed throttle, and the attitude controller after the supervisor with this tick's
    values (v0.6.3 order).  The caller fills every other ``OnboardInputs`` field; ``vc_*``, ``throttle_prev``,
    ``gps_gate_ok``, ``pc_requests`` and ``tlm_gps`` are overwritten here."""

    CHAINS = ("fcu", "gns")

    def __init__(self, sp: SupervisorParams, vp=None, seed: int = 0, reset_in_flight: bool = False, pp=None,
                 chain: str = "fcu"):
        self.sup = OnboardSupervisor(sp, seed=seed, reset_in_flight=reset_in_flight)
        self.vc = None
        self.pos = None
        if chain not in self.CHAINS:
            raise ValueError(f"FcuStack chain {chain!r} not in {self.CHAINS}")
        self.chain = chain                         # "fcu": v0.8 three chips (the position step here); "gns": v0.8.1
        self.gns = False
        if sp.vtol:
            try:
                from sim.vertical_ctrl import VerticalController
            except ImportError:  # pragma: no cover
                from vertical_ctrl import VerticalController  # type: ignore
            if vp is None:
                raise ValueError("the VTOL build needs the vertical controller's parameters (VerticalParams)")
            self.vc = VerticalController(vp)
            if sp.gps_hold:
                try:
                    from sim import nlink_proto as _nl
                    from sim.position_ctrl import PositionHold, TiltSetpoint
                except ImportError:  # pragma: no cover
                    import nlink_proto as _nl  # type: ignore
                    from position_ctrl import PositionHold, TiltSetpoint  # type: ignore
                if pp is None:
                    raise ValueError("the VTOL build with position hold needs PositionParams (sim.position_ctrl)")
                self.pp = pp
                self.tsp = TiltSetpoint(pp)
                if chain == "gns":
                    # v0.8.1 four chips: no position controller here - the GNS runs it; the FCU takes the tilt setpoint
                    # and the GNS status from the nlink v4 slot (sim.nlink_proto.GnsSlotRx) and returns its pflags / k_T
                    self.gns = True
                    self.rx = _nl.GnsSlotRx(pp.step_ticks)
                else:
                    self.pos = PositionHold(pp)
                    self.rx = _nl.GpsSlotRx()
        self.vc_out = None
        self.sup_out: OnboardOutputs | None = None
        self.throttle = 0.0
        self.tilt = (0.0, 0.0)                     # v0.8: the tilt the next attitude step uses
        self.pos_ticks = 0                         # v0.8: ticks since the last position step
        self.sat_hi = False                        # v0.8: vertical controller sat == +1 since the last step
        self.pos_pending: list = []                # v0.8: (publish tick, PosOutputs) of steps in the job pipeline
        self.pos_pub = None                        # v0.8: the committed (published) position-controller outputs
        self.k = -1                                # v0.8: FCU tick counter (the pipeline's clock)

    @classmethod
    def from_params(cls, p: dict, mass_kg: float, t_max_n: float, mode: str | None = None, profile: str | None = None,
                    gps: bool | None = None, takeoff_check: bool | None = None, seed: int = 0,
                    reset_in_flight: bool = False, vgains: dict | None = None, pgains: dict | None = None,
                    fc_config_path: str | None = FC_CONFIG_PATH, chain: str = "fcu", **sp_overrides) -> "FcuStack":
        """One consistent stack from params.json (v0.8): ``SupervisorParams.from_params(mode, profile, gps,
        takeoff_check)``, ``VerticalParams.from_params(mass, T_max, profile, **vgains)`` and, when the build has
        position hold, ``PositionParams.from_params(mass, T_max, profile, **pgains)`` - the controller's model mass
        (GPS pod included) and T_max for both.  ``gps=False`` = the v0.7.1 VTOL build.  ``chain`` = "fcu" (v0.8, three
        chips: the default here, so existing callers keep the v0.8 FCU) or "gns" (v0.8.1, four chips: the FCU without the
        position controller, nlink v4 slots - ``sim.glink_proto``; the 6-DOF picks it from params)."""
        sp = SupervisorParams.from_params(p, fc_config_path=fc_config_path, mode=mode, profile=profile, gps=gps,
                                          takeoff_check=takeoff_check, **sp_overrides)
        if not sp.vtol:
            return cls(sp, seed=seed, reset_in_flight=reset_in_flight)
        try:
            from sim.position_ctrl import PositionParams
            from sim.vertical_ctrl import VerticalParams
        except ImportError:  # pragma: no cover
            from position_ctrl import PositionParams  # type: ignore
            from vertical_ctrl import VerticalParams  # type: ignore
        vp = VerticalParams.from_params(p, mass_kg, t_max_n, g=sp.g, profile=profile, **(vgains or {}))
        pp = (PositionParams.from_params(p, mass_kg, t_max_n, g=sp.g, profile=profile, **(pgains or {}))
              if sp.gps_hold else None)
        return cls(sp, vp, seed=seed, reset_in_flight=reset_in_flight, pp=pp, chain=chain)

    def attitude_inputs(self) -> tuple[bool, float]:
        """VTOL: (control_active, u) the attitude controller of the NEXT step uses (= this tick's results)."""
        return bool(self.sup_out is not None and self.sup_out.control_active), self.throttle

    def attitude_tilt(self) -> tuple:
        """v0.8: the tilt setpoint (level-frame rotation vector) the attitude controller of the NEXT step uses."""
        return self.tilt

    def pos_out(self):
        """v0.8: the position controller's committed outputs (``sim.position_ctrl.PIPELINE_TICKS`` after the step's
        inputs); OFF before the first commit."""
        if self.pos_pub is None:
            try:
                from sim.position_ctrl import OFF_OUT
            except ImportError:  # pragma: no cover
                from position_ctrl import OFF_OUT  # type: ignore
            return OFF_OUT
        return self.pos_pub

    def _tlm_gps(self) -> tuple:
        """TELEM v3 fields: the newest complete set up to this tick's frame (flags only while fresh) and the position
        controller's committed mode / distance (as of the previous tick).  gns chain: ``GnsSlotRx.tlm`` (the newest fresh
        set's mode and distance; ATT while the FCU flies the trim fallback)."""
        if self.gns:
            return self.rx.tlm(bool(self.sup_out is not None and self.sup_out.motor_on))
        rx, po = self.rx, self.pos_out()
        g = rx.latest
        flags = g.flags if (g is not None and rx.fresh()) else 0
        sv = g.num_sv if g is not None else 0
        ha = g.hacc_dm if g is not None else lp.HACC_NONE
        return flags, sv, ha, po.mode, po.dist_m

    def step(self, inp: OnboardInputs) -> FcuOutputs:
        if self.vc is None:
            so = self.sup.update(inp)
            self.throttle = so.throttle
            self.sup_out = so
            return FcuOutputs(so, None, so.throttle, so.control_active, so.throttle)
        try:
            from sim.vertical_ctrl import VerticalInputs, cos_tilt_from_q
        except ImportError:  # pragma: no cover
            from vertical_ctrl import VerticalInputs, cos_tilt_from_q  # type: ignore
        gps_set = None
        extra = {}
        self.k += 1
        if self.pos is not None:                                # (0) the GPS slot of this tick's NAV frame
            gps_set = self.rx.feed(inp.gps_slot)
            extra = dict(gps_gate_ok=self.rx.gate_ok, pc_requests=self.pos_out().requests, tlm_gps=self._tlm_gps())
        elif self.gns:
            # v0.8.1 (0): the GNS slot of this tick's NAV frame: a TILT part is the builder's target from this tick, a set
            # completed now gives the gate, the GNS's latched requests (+ the FCU's own GNS-loss request) and TELEM v3
            gps_set = self.rx.feed(inp.gps_slot)
            extra = dict(gps_gate_ok=self.rx.gate_ok, pc_requests=self.rx.requests(self.pp.land_steps),
                         tlm_gps=self._tlm_gps())
        att_active, att_u = self.attitude_inputs()             # (1) attitude controller: previous tick's outputs
        att_tilt = self.tilt
        prev = self.vc_out
        inp = replace(inp, vc_touchdown=bool(prev is not None and prev.touchdown),
                      vc_at_target=bool(prev is not None and prev.at_target),
                      vc_ground_signature=bool(prev is not None and prev.ground_signature),
                      vc_low_in_land_ticks=int(prev.low_in_land_ticks) if prev is not None else 0,
                      vc_takeoff_fail=bool(prev is not None and prev.takeoff_fail),
                      vc_takeoff_passed=bool(self.vc.vp.takeoff_ticks <= 0 or (prev is not None and prev.takeoff_passed)),
                      vc_support=bool(prev is not None and prev.support),
                      vc_airborne=bool(prev is not None and prev.airborne),
                      throttle_prev=self.throttle, **extra)
        so = self.sup.update(inp)                              # (3) supervisor
        if inp.cos_tilt is not None:
            c = inp.cos_tilt
        else:
            c = cos_tilt_from_q(inp.q_nav) if inp.q_nav is not None else None
        vo = self.vc.update(VerticalInputs(phase=so.phase, h_tgt_m=so.h_tgt_m, baro_lost=so.baro_lost, h_m=inp.h_m,
                                           vz_mps=inp.vz_mps, tilt_deg=inp.tilt_deg, cos_tilt=c,
                                           f_z_mps2=inp.accel_z_mps2, u_cap=so.u_cap))   # (4) vertical controller
        self.vc_out = vo
        self.sup_out = so
        self.throttle = vo.u if so.motor_on else 0.0           # (5) ESC
        po = None
        if self.pos is not None:
            try:
                from sim.position_ctrl import PIPELINE_TICKS, PosInputs
            except ImportError:  # pragma: no cover
                from position_ctrl import PIPELINE_TICKS, PosInputs  # type: ignore
            self.sat_hi = self.sat_hi or vo.sat == 1
            self.pos_ticks += 1
            if self.rx.got_part4 or self.pos_ticks >= self.pp.step_ticks:      # (6) the position step when due
                cp = 1.0 if c is None or c != c else c
                po = self.pos.step(PosInputs(phase=so.phase, motor_on=so.motor_on, h_m=inp.h_m, cos_tilt=cp,
                                             rate=tuple(inp.gyro_rad_s), k_t=vo.k_thrust, vc_sat_high=self.sat_hi,
                                             data=gps_set, vc_low_alt=bool(vo.low_alt),
                                             vc_ground=bool(vo.low_alt and (vo.still or vo.support or vo.floor)),
                                             lifted=bool(self.vc.vp.takeoff_ticks <= 0 or vo.takeoff_passed)))
                self.pos_pending.append((self.k + PIPELINE_TICKS, po))   # committed by job J4, PIPELINE_TICKS later
                self.pos_ticks = 0
                self.sat_hi = False
            while self.pos_pending and self.pos_pending[0][0] <= self.k:
                self.pos_pub = self.pos_pending.pop(0)[1]
            out = self.pos_out()                                # (7) the tilt setpoint builder (250 Hz slew)
            self.tilt = self.tsp.tick(out.tilt_l, active=bool(out.engaged and so.motor_on))
        fcu4 = None
        if self.gns:
            try:
                from sim import nlink_proto as _nl
            except ImportError:  # pragma: no cover
                import nlink_proto as _nl  # type: ignore
            # (6) v0.8.1: this tick's FCU-side inputs of the GNS position controller for the nlink v4 FCU frame
            pf = _nl.fcu4_pflags(so.phase, so.motor_on, vo.sat == 1, bool(vo.low_alt),
                                 bool(vo.low_alt and (vo.still or vo.support or vo.floor)),
                                 bool(self.vc.vp.takeoff_ticks <= 0 or vo.takeoff_passed))
            fcu4 = (pf, _nl.kt_quant(vo.k_thrust))
            eng, tgt = self.rx.target()                         # (7) the builder on the slot's TILT / the TRIM fallback
            self.tilt = self.tsp.tick(tgt, active=bool(eng and so.motor_on))
        return FcuOutputs(so, vo, self.throttle, att_active, att_u, att_tilt, po, gps_set, self.tilt, fcu4)
