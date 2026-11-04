"""Vertical (altitude / vertical-speed) controller of the VTOL build (SPEC 8.9; v0.7, v2 = v0.7.1) - the reference.

Executable specification for ``firmware/atmega128_fc/src/core/vertical_ctrl.c`` (float32 on the FCU; this reference is
float64 except the altitude reference, which is an exact integer in both languages; the cross-check compares within a
float32 tolerance).  Pure Python, no numpy, every state explicit, every equation written out; deterministic (no clock,
no RNG).

Where it sits (``sim.onboard.FcuStack``, VTOL tick order of v0.7.1, mission.vtol.fcu_tick_order_note): attitude
controller (this tick's NAV frame, the previous tick's supervisor outputs and throttle) -> vane registers -> onboard
supervisor -> THIS controller -> ESC register.  The supervisor decides the phase, the altitude target, whether the
motor may run (``motor_on``) and whether the baro is lost; this controller computes the throttle u; the ESC gets u only
while the supervisor enables the motor.  Its ``touchdown`` / ``at_target`` / ``ground_signature`` /
``low_in_land_ticks`` reach the supervisor in the NEXT tick.

Units: altitude h, reference h_ref, target [m] above the pad (the NAV zeroes h on the pad); vertical speed v_z and the
commands [m/s], up positive; throttle u [0..1] (ESC command, T = T_max u^n); specific force f_z [m/s^2] (body z,
+g at rest upright); um = micrometre (the integer unit of the reference).

Mode (from the supervisor's phase): CLIMB / HOLD -> FLY, LAND -> LAND, anything else -> IDLE (the controller is reset
and outputs u = 0).  Per tick, dt = 1 / loop_hz:

0. Start (first non-IDLE tick = each take-off): ref = m_to_um(h) (0 if h is not finite), integrator i = 0, ramp
   counter k = 0, all touchdown counters 0, filters v_lpf = 0, d_lpf = 0, thrust state r = 0 and u_last = 0 (the motor
   was off), learnt thrust ratio k_T = clamp(1, kt_min, kt_max) (= 1 with the params.json limits).  The integrator is
   not preloaded: the hover feed-forward carries the weight from the first tick, the integrator only corrects the
   remaining thrust-model error (|i| <= vz_i_max).
1. Reference (exact integers, v0.7.1).  ref [um, int] with h_ref = ref x 1e-6 m; the steps are the rates per tick
   rounded to the micrometre, up = round(climb_rate_mps x 1e6 / loop_hz) (1 m/s at 250 Hz: 4000 um exactly; 0.6 ->
   2400, 0.3 -> 1200, demo 0.5 -> 2000, 0.4 -> 1600), dn (descent_rate_mps), td (touchdown_rate_mps); flare =
   round(flare_altitude_m x 1e6).  The target is the supervisor's integer decimetres: tgt = round(h_tgt_m x 10) x 1e5.
   FLY   d = tgt - ref; -dn <= d <= up -> delta = d (lands exactly on the target), else delta = +up (d > 0) or -dn.
         at_target = (ref + delta == tgt).
   LAND  delta = -td if ref <= flare or h <= flare_altitude_m (the slower of the two readings decides), else -dn.
         ref is not bounded below by the pad (the ground may lie below it); |ref| <= 2e9 um (int32).
         With the baro lost (v0.7.1) the same law on the NAV filter's dead reckoning (the accelerometer with the learnt
         z bias, ``sim.state_machine.VtolAltitudeFilter``), but delta = -dn all the way down (no flare).  Measured in the
         scratch study v071/pyref/td_study2.py (1-D plant, VTOL NAV filter, 100 baro losses in a 5 m hover per case):
         - v0.7.0 flew a fixed v_cmd there: on the ground that lowers the thrust by only ~0.1 g in 10 s (the error is
           just the touchdown rate, the integrator is slow), the support signature never appeared and the baro-lost
           time limit ended in V9 ABORT ON THE GROUND in 33 / 60; with the descending reference it runs away from the
           stopped vehicle and the thrust drops as in a normal landing;
         - the dead-reckoned v_z carries the filter's error at the loss (sigma ~0.15 m/s at 0.25 m baro noise, plus the
           correlated bias error): with the 0.3 m/s flare 19 / 30 % (baro noise 0.25 / 0.43 m) of the vehicles hovered
           until the V9 ABORT at 1-3 m; at descent_rate all the way 3 / 12 %; touchdown median 0.60 / 0.67 m/s, max
           0.93 / 1.18 m/s (with the flare: max 0.93 / 1.18 m/s as well).
   ref += delta;  dh_ref = delta x 1e-6 x loop_hz [m/s] (the reference rate of this tick).
   Why integers: v0.7.0 summed climb_rate x dt in floating point; a climb from the 0.1 m grid (h = 0 on the pad, or a
   previous target) to a 0.1 m-grid target is an exact number of 4 mm steps, so the arrival test d <= up was decided
   by rounding - the float32 C port and this float64 reference disagreed by one tick of CLIMB vs HOLD in every 6-DOF
   climb.  Integer micrometres make every step, every comparison and at_target exact and identical in both languages
   (the only conversion of a measured float, m_to_um(h) at the start, is one rounding per segment).  The C port replaces
   its compensated float32 sum with the same int32 arithmetic (an int32 add per tick: cheaper than the float sum).
2. Vertical-speed command:
   h_err = alt_kp (h_ref - h)   (the altitude-feedback part)
   v_cmd = clamp(ff dh_ref + h_err, -vz_max_down_mps, +vz_max_up_mps), ff = 1 when mission.vtol.vz_ref_feedforward
           (v0.7.1: removes the ~climb_rate / alt_kp ramp lag, 0.96 m peak on UP in v0.7.0), else 0 (the v0.7.0 law).
3. Throttle:
   c     = cos(tilt) from the NAV quaternion, 1 - 2 (qx^2 + qy^2) (``cos_tilt_from_q``; no cosf on the FCU), clamped to
           [cos(tilt_abort_deg), 1]; 1 if not finite.  (Callers without a quaternion pass tilt_deg: c = cos(tilt) - a
           reference convenience, not the FCU path.)
   u_ff  = u_hover / sqrt(k_T c) for throttle_exponent n = 2, else u_hover (k_T c)^(-1/n) (fallback): the thrust
           feed-forward m g / cos(tilt) through the inverse thrust model u = (T / (k_T T_max))^(1/n), u_hover =
           (m g / T_max)^(1/n), with the thrust ratio k_T learnt up to the previous tick (4b; v0.7.1b, architect
           mission.vtol.thrust_learning).  No battery-voltage term: a fresh pack's extra thrust (the proxy table: +23 % at
           25.2 V) is part of what k_T learns; the sag during the flight is followed at the learning rate.
   e     = v_cmd - v_z (0 if not finite)
   cap   = u_max min(1, (k + 1) / ramp_ticks)   (smooth take-off: the ceiling rises from 0 to u_max over
           throttle_ramp_s from the first active tick; ramp_ticks = 0 -> u_max)
   i'    = clamp(i + vz_ki e dt, -vz_i_max, +vz_i_max)
   anti-windup (conditional integration): i = i' unless u_ff + vz_kp e + i' > cap with e > 0 or
           u_ff + vz_kp e + i' < u_min with e < 0 (the output is saturated in the direction e pushes)
   u_raw = u_ff + vz_kp e + i;  u = min(max(u_raw, u_min), cap)  (the ramp ceiling wins over u_min)
   sat   = +1 if u_raw > cap, -1 if u_raw < u_min <= cap, else 0 (reported, never hidden: with T/W ~1.19 the hover
           throttle is ~0.91 and u_max = 1 leaves ~0.09 of authority)
   k     = min(k + 1, ramp_ticks)
3b. v0.9 take-off for the light VTOL vehicle (mission.vtol.takeoff_v09; absent = the v0.8.1 law above, bit for bit):
   * two-slope ceiling: cap = u_start (k + 1) / ramp_ticks for k + 1 <= ramp_ticks (on the pad: the pad holds the
     vehicle, the spool reaction torque goes into the ground), then cap = min(u_max, u_start + (k + 1 - ramp_ticks)
     slew_per_s / loop_hz); "the ramp is over" (every use of ramp_ticks above, the learning gate 4b and the take-off
     check 6) = k + 1 >= ramp_total_ticks = ramp_ticks + ceil((u_max - u_start) loop_hz / slew_per_s).  Why: the v0.8.1
     ramp reached u = 1 in 0.5 s; on the 1.65 kg vehicle (model hover u 0.73) that is T / W ~1.6-1.9 at the lift-off,
     a violent climb, the throttle then cut to u_min - and the EDF rotor's reaction I_rot domega/dt (0.15 N m per unit
     du/dt, ASSUMED rotor) against 0.14 N m of yaw authority spun the vehicle to I3 at lift-off (v0.9 plant round: 52 of
     79 field aborts in the climb).  u_start (0.5) is below the hover throttle of every plant of the envelope (strongest:
     3.09 kgf x 1.23 fresh pack on the 1.40 kg tether vehicle, u 0.61), so the lift-off happens on the slow segment.
   * take-off thrust search: on every FLY tick with the ramp over, the take-off check not passed, e > 0 and v_z <=
     TAKEOFF_SEARCH_VZ_FRAC_ASSUMED climb_rate_mps (the loop asks for a climb that has not happened, or a climb at less
     than half the commanded rate), after the throttle of this tick: k_T <- max(k_T (1 - 2 boost_per_s /
     (loop_hz u_hover)), kt_min) - the next tick's u_ff = u_hover / sqrt(k_T c) is ~boost_per_s dt higher; nothing else
     changes (no new state: the learnt thrust ratio carries the result into the flight, where the learning 4b goes on).
     Why: a plant weaker than the model needs u above u_ff + vz_kp vz_max_up + vz_i_max (0.73 + 0.075 + 0.05 on the
     battery vehicle = 0.86 < the 0.94 of 2.14 kgf at the end of the pack); without the search it sat on the pad below
     u_max, never saturated, so the take-off check (6) could not act (v0.9 plant round: V15 never fired on the VTOL
     vehicle).  With it the loop reaches u_max (k_T at kt_min 0.55: u_ff 0.98) and V15 stops the motor.  A tick that
     searched (k_T fell) also un-latches the take-off check's h0 and restarts its settle time (6): the thrust - and an
     uncalibrated thrust-proportional baro offset with it - is still rising, so h0 is latched only once the search has
     stopped (the vehicle climbs, or k_T reached kt_min at u_max).
   * output slew limit (u_slew_per_s > 0, ramp over): |u - u_last| <= u_slew_per_s dt (bounds the rotor reaction torque
     in flight).
3c. v0.9 power ceiling (tethered builds, ``VerticalInputs.u_cap``; None = none, every other build bit for bit): cap =
   min(cap, u_cap) - the supervisor's W_SAG warning freezes the ceiling at the throttle of its onset (sim.onboard "v0.9
   power rules": a supply in its limit is a constant-power source - more throttle past its maximum-power point lowers the
   bus and the thrust); the conditional anti-windup and sat above work on the capped ceiling (no wind-up above it).
4. Signals for the touchdown detector (every active tick; u_last = this controller's u of the previous tick = the ESC
   throttle then applied, since FLY / LAND means POWERED):
   r     += a_T ((u_last / u_hover)^n - r),  a_T = 1 - exp(-dt / edf.thrust_time_constant_s)
           the free-flight thrust per weight T_model / (m g) the thrust model predicts, through the EDF's first-order lag
4b. Thrust learning (v0.7.1b; architect mission.vtol.thrust_learning, after the independent physics review: with a
   plant ~20 % stronger than the thrust model the tuned loop - it can lower u only vz_kp vz_max_down + vz_i_max = 0.131
   below u_ff - could not follow the LAND reference down, sat pinned, and the floor signature confirmed a touchdown 6-8 m
   up: motor stop, 10-12 m/s impact):
   learn = kt_tau_s > 0 and the take-off ramp is over (k = ramp_ticks after step 3) and f_z finite and
           r >= KT_R_MIN_ASSUMED and h > flare_altitude_m and c finite and c > cos(kt_max_tilt_deg)
   if learn:  k_T += a_K (f_z / (g r) - k_T),  a_K = 1 - exp(-dt / kt_tau_s);  k_T = clamp(k_T, kt_min, kt_max)
   else:      k_T unchanged (frozen near the ground, at large tilt, on the ramp, without f_z)
   f_z / (g r) is the measured thrust per modelled thrust T_actual / T_model whatever the vertical acceleration: the
   body-z specific force is (T + axial drag) / m and excludes gravity, so climbs, descents and gusts do not bias it
   (only through the axial drag and the unmodelled ESC delay, both averaged by the 2 s low-pass); a constant
   accelerometer z bias b is absorbed (k_T = k_true (1 + b / g)), and then cancels in d below.  Units: f_z [m/s^2],
   g r [m/s^2], k_T [-].  k_T never commands anything by itself: it scales the feed-forward (3) and the support
   signature's expected thrust (d below); the supervisor never sees it.
   Why h > flare_altitude_m and not "free flight" (a ground reaction makes f_z / (g r) = 1 / r, which pulls k_T up): on
   the ground below the flare band learning is frozen; ON ground above the pad (a roof, or the NAV altitude reading high)
   it runs and drives k_T toward kt_max, which lowers u_ff - the fan pushes less while the vehicle sits there - until the
   clamp; the support signature then reappears (d = g (1 - kt_max r) with the loop pinned) and the touchdown confirms
   (5).  Freezing on "low" instead would freeze a not-yet-learnt k_T exactly when a strong plant pins the loop in the air
   (an OFF right after take-off) - the case the learning exists for.
   KT_R_MIN_ASSUMED (0.25) only guards the division: after the ramp u >= u_min, so r >= (u_min / u_hover)^2 ~ 0.30 once
   the thrust lag has settled.
   d     = f_z - k_T (g r)   the body-z specific force beyond what the fan alone gives (free flight with k_T learnt: ~0;
           on the ground: the ground's reaction N cos(tilt) / m); g r is evaluated once per tick (the learning divides
           by it), the operand order the C port keeps
   d_lpf += a_S (d - d_lpf),  a_S = 1 - exp(-dt / TD_SUPPORT_LPF_TAU_S_ASSUMED)  (vibration, the impact spike); held
           when f_z is absent or not finite
   v_lpf += a_V (v_z - v_lpf),  a_V = 1 - exp(-dt / TD_STILL_LPF_TAU_S_ASSUMED)  (1.0 s: the NAV v_z noise defeated the
           v0.7.0 raw test; see the constant for the study); held when v_z is not finite
   lead  = h_err <= -still_lead touchdown_rate_mps   (the reference has run still_lead touchdown_rate / alt_kp, 0.48 m,
           below the vehicle: the vehicle does not follow the descending reference)
   pinned = sat == -1 or (i == -vz_i_max and v_cmd == -vz_max_down_mps)   (the loop asks for all the descent it can:
           at u_min, or - architect, v0.7.1 - integrator and command both at their downward limits)
   floor run = floor run + 1 if pinned else 0
   signatures (mission.vtol.touchdown): still   = |v_lpf| <= td_vz_still_mps and lead
                                        support = d_lpf > support_g g (f_z given)
                                        floor   = floor run >= floor_ticks (pinned for floor_s: free flight cannot
                                                  sustain it WHEN the thrust model is right - with the integrator at
                                                  -vz_i_max and the command at -vz_max_down the thrust is below m g
                                                  (vtol_gains.json: u = u_ff - 0.131 = 0.784, T / m g = 0.73), the
                                                  vehicle accelerates downward, e turns positive and unpins the
                                                  integrator; on the ground nothing moves and it stays pinned.  With
                                                  a plant >= ~21 % stronger than the model (physics review) the
                                                  pinned thrust still carries the vehicle: pinned in the AIR - hence
                                                  k_T (4b) and the gating of 5)
   ground_signature = support or floor (for the supervisor's baro-lost time limit V8 / V9, unchanged in v0.7.1b; also
                      outside LAND, diagnostics)
   Why still asks h_err (not v_cmd) for the touchdown rate: in v0.7.0 v_cmd WAS alt_kp (h_ref - h), so "the command
   asks for at least the touchdown rate" meant "the reference has run touchdown_rate / alt_kp (0.24 m) below the
   vehicle and the vehicle does not follow".  With the feed-forward, v_cmd <= -touchdown_rate holds whenever h_ref <= h,
   i.e. most of the time in a normal flare; the low-passed v_z near 0 then sufficed and the study measured false
   touchdowns in 26-27 % of noisy landings and in the turnaround of an OFF during the climb (19-25 / 60).  h_err keeps
   the v0.7.0 meaning; still_lead = 2 (TD_STILL_LEAD_ASSUMED) doubles the required run-away because 1 still left
   2-2.5 % false touchdowns below 1 m at 0.43 m baro noise.
5. Touchdown detector (LAND only; ``touchdown`` = confirmed, the supervisor goes LANDED next tick).  Gating v0.7.1b
   (architect mission.vtol.touchdown.gating_v071b: no signature may stop the motor at altitude on the strength of the
   loop being saturated):
   low_alt = h <= flare_altitude_m (also with the baro lost: the dead-reckoned h; v0.7.0 took "always" there - with the
          low-passed v_z still showing the hover at the loss this confirmed a touchdown 5 m up in every study run)
   low  = low_alt or v_cmd <= -vz_max_down_mps (the altitude loop saturated downward: ground above the pad, baro offset)
   counters: count_still   = count_still + 1   if (low_alt and still)                                  else 0
             count_support = count_support + 1 if (low_alt and support)                                else 0
             count_floor   = count_floor + 1   if (low_alt and floor)                                  else 0
             count_all     = count_all + 1     if (low and support and floor and (still or baro_lost)) else 0
   touchdown = any count >= landed_confirm_ticks; td_by = the first confirmed of still / support / floor / all
   (1 / 2 / 3 / 4).  In the flare band any one signature confirms (v0.7.1); above it (ground above the pad, a NAV
   altitude reading high) only support and floor together, and with the baro healthy still as well.
   count_all's still (|v_lpf| <= td_vz_still_mps and the lead) is a deviation from gating_v071b ("support AND floor"),
   on the safe side, for the architect's confirmation: support and floor TOGETHER still happen in free flight when k_T
   is too low by more than support_g (k_T / k_true < 1 - support_g = 0.75: d = g (1 - k_T / k_true) in an
   unaccelerated flight) and the loop is pinned - with k_T frozen at kt_min = 0.7 on the model's own plant the tuned
   loop (vtol_gains.json) cannot hold the vehicle down and it climbs with both signatures true (test_vtol_ref
   test_stack_mislearnt_thrust_ratio_*).  The pinned equilibrium with |v_z| <= 0.15 m/s AND d > support_g g needs
   k_T / k_true in [0.748, 0.75) with the tuned gains - a 0.3 % band of a 25 % mis-learning; physical limit: a vehicle
   sitting on a roof and one hovering pinned at the same thrust give the same f_z, h and v_z - only a known k_T or a
   still altitude separates them.  With the baro lost the NAV v_z is dead reckoning (its error at the loss and the
   accelerometer bias error make a vehicle on the ground look moving: a 0.25 m/s v_z error kept the dead-reckoned h
   above the flare band and v_lpf at 0.25 m/s on the ground), so there the rule stays support AND floor - the same
   exposure the V8 rule has (ground_signature = support or floor, unchanged).
   low_in_land_ticks = low_in_land_ticks + 1 if (low_alt and lead and not baro_lost) else 0 - the supervisor's LAND floor
          timeout (mission.vtol.touchdown.land_floor_timeout_s) compares it; not counted with baro_lost (the
          dead-reckoned h drifts: the baro-lost time limit bounds that case).  ``baro_lost`` changes nothing else here.
          Why "and lead" (architect, v0.7.1, after the reference study): with "low" alone the timer ran while the vehicle
          was still descending normally below the pad level (a ditch, ground 1.5 m below the pad: motor stopped 0.7 m
          up, 3.4 m/s drop).  A vehicle that follows the descending reference is in the air; only one that has stopped
          following it by 0.48 m (supported) counts toward the timeout.  Why low_alt (v0.7.1b, gating_v071b): the
          saturated-loop clause of "low" let the timer run with the loop pinned in the air (physics review, strong
          plant).
   Why the still signature is gated by the altitude band only (v0.7.1): with 0.25-0.43 m baro noise the "loop saturated
   downward" clause also holds for 0.5 s at height (h reading 0.3 m above the reference), and right after the OFF the
   low-passed v_z still shows the hover - together a FALSE touchdown at height (scratch study: up to 2 % of landings,
   motor stopped 1-5 m up).  v0.7.1b extends the band gating to support and floor alone (gating_v071b).
   FLY: every count 0.
   Physical limits: no ground-effect model and no rangefinder.  The u_min clause of "pinned" alone cannot occur on the
   ground with the tuned gains (u_floor_reachable = u_ff - vz_kp vz_max_down - vz_i_max > u_min) - hence the integrator
   clause.  On the ground in the flare band the support margin is small with the tuned gains (d = g (1 - (1 - sqrt(k_T)
   0.131 / u_hover)^2) = 0.27 g at k_T = 1 against support_g 0.25 g; physics review): still or floor usually confirm
   there.

6. Take-off check (v0.8, architect; params mission.vtol.position_hold.takeoff_check_ASSUMED climb_m 0.3, within_s
   3.0; v0.7.1 finding: a T/W < 1 vehicle sat on the pad at u_max until the endurance rule).  The reference height h0
   is LATCHED once the take-off ramp is over (cap = u_max) and ``takeoff_settle_ticks`` more ticks have passed (h at
   that tick, 0 if not finite) - physics-review fix (medium 4; params takeoff_check note, architect 18:10): with h0 = h
   at the start, an uncalibrated thrust-proportional baro offset (baro_thrust_comp_m 0, true offset 1 m) raised the NAV
   h by 0.98 m within 1.2 s of the motor start with the vehicle ON the pad, 'passed' latched and a T/W 0.93 vehicle ran
   its motor 19.8 s on the pad (14.8 s at u_max); the offset follows the thrust, which is at u_max when the ramp ends,
   and reaches the NAV h after the baro latency plus the altitude filter's response, so the settle time is
   ``TAKEOFF_SETTLE_S_ASSUMED`` (>= baro latency 0.1 s (SPEC 9) + 1 / (2 pi altitude crossover 0.5 Hz) = 0.42 s).
   passed (latched for the flight) once h0 is latched and h >= h0 + takeoff_climb_m (``takeoff_passed`` also gates the
   position loop's engagement, ``sim.position_ctrl`` step 1).  Until then, in FLY, a tick with the take-off ramp over
   (cap = u_max) and the loop saturated at it (sat == +1) adds 1 to ``takeoff_run``; any other tick sets it to 0 (the
   run counts from the end of the ramp, h0 latched or not).  ``takeoff_fail`` = takeoff_run >= takeoff_ticks
   (ticks_ge(within_s)) - the supervisor (rule V15, next tick) stops the motor and goes SAFE.  Why "saturated,
   continuously": a healthy vehicle (T/W 1.19) climbs at 1 m/s with ~0.09 of throttle left and is not pinned at u_max;
   a vehicle that cannot lift itself is pinned and stays at the pad height.  takeoff_ticks = 0: the check is off (the
   v0.7.1 controller).  The flag only reports; the supervisor decides (``SupervisorParams.takeoff_check``).

Invariant: the output never leaves [0, u_max]; u = 0 exactly when IDLE; nothing here can start the motor (the
supervisor's motor_on gates the ESC).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace as _replace
from functools import cached_property

try:  # package import (project root on sys.path) or flat import (sim/ on sys.path)
    from sim import link_proto as lp
    from sim.onboard import ticks_ge, vtol_values
    from sim.state_machine import G0
except ImportError:  # pragma: no cover
    import link_proto as lp  # type: ignore
    from onboard import ticks_ge, vtol_values  # type: ignore
    from state_machine import G0  # type: ignore

TD_STILL_FRACTION_ASSUMED = 0.5     # |v_lpf| <= this x touchdown_rate_mps counts as "not moving" (still signature)
TD_STILL_LPF_TAU_S_ASSUMED = 1.0    # still signature: v_z low-pass time constant.  v0.7.0 used the raw v_z: its noise
                                    # under 0.43 m baro noise hid 5/20 touchdowns.  Scratch studies v071/pyref/td_study*.py
                                    # (1-D plant + the VTOL NAV filter, baro 0.25 / 0.43 m RMS): WITHOUT the lead
                                    # condition of docstring 4 a 0.5 s low-pass confirmed FALSE touchdowns 0.1-1 m up in
                                    # 26-27 % of landings (drops up to 3.5 m/s); with it, 1.0 s: 0 / 120 landings, 1 / 120
                                    # climb-turnarounds at 0.43 m noise (0.5 s: 1 / 120 and 0 / 120; 2.0 s: 0 and 1 plus
                                    # 8 / 120 V10 timeouts)
TD_STILL_LEAD_ASSUMED = 2.0         # still signature: the altitude-feedback part must ask for this x touchdown_rate_mps,
                                    # i.e. the reference 2 x 0.3 / alt_kp (0.48 m at alt_kp 1.25) below the vehicle.  1.0
                                    # is the v0.7.0 meaning; scratch study v071/pyref/td_study2/3.py (200 noisy landings
                                    # per case, 0.43 m baro noise): 1.0 -> 5 / 200 false touchdowns below 1 m (landing),
                                    # 4 / 200 (OFF in the climb); 2.0 -> 0 and 0, at 0.25 m noise 0 either way; the price
                                    # is a later confirmation (median 2.1-2.2 s after contact instead of 1.5-1.7 s) and
                                    # 9 / 200 (0.25 m) and 16 / 200 (0.43 m) landings ended by the V10 floor timeout - on
                                    # the ground (motor off, no release) - because 6 s is tight (see the report)
TD_SUPPORT_LPF_TAU_S_ASSUMED = 0.2  # support signature: low-pass of the specific-force excess (averages vibration by
                                    # sqrt(dt / 2 tau) = 0.1 per sample; the touchdown impact spike lasts << 0.5 s)
KT_R_MIN_ASSUMED = 0.25             # thrust learning (4b): learn only while the modelled thrust ratio r >= this - a
                                    # division guard; after the take-off ramp u >= u_min, so r >= (u_min / u_hover)^2
                                    # ~ 0.30 once the EDF lag has settled.  core/vertical_ctrl.h VC_KT_R_MIN must equal it
TAKEOFF_SETTLE_S_ASSUMED = 0.5      # take-off check (6): h0 latched this long after the ramp ends: >= the baro latency
                                    # 0.1 s (SPEC 9, SensorConfig.baro_latency_s) + 1 / (2 pi x the NAV altitude-filter
                                    # crossover 0.5 Hz, SensorConfig.altitude_crossover_hz) = 0.42 s, rounded up (the
                                    # physics review's "about 0.5 s"); core/vertical_ctrl.h VC_TAKEOFF_SETTLE_S must equal it
TAKEOFF_SEARCH_VZ_FRAC_ASSUMED = 0.5   # v0.9 take-off thrust search (3b): runs only while the NAV v_z is not above this
                                    # fraction of climb_rate_mps (field 0.5, demo 0.25 m/s): the vehicle does not climb, or
                                    # climbs at less than half the commanded rate (TUNE: an absolute 0.25 m/s stopped the
                                    # search on weak plants that then climbed slowly behind the reference - hold RMS 13 -> 20
                                    # of 20 failed in thrust_low)
UM_PER_M = 1_000_000
UM_PER_DM = 100_000
REF_LIMIT_UM = 2_000_000_000        # |ref| bound (int32 on the FCU)

MODE_IDLE = 0
MODE_FLY = 1
MODE_LAND = 2
MODE_NAMES = {MODE_IDLE: "IDLE", MODE_FLY: "FLY", MODE_LAND: "LAND"}
TD_NONE, TD_STILL, TD_SUPPORT, TD_FLOOR, TD_ALL = 0, 1, 2, 3, 4
TD_NAMES = {TD_NONE: "none", TD_STILL: "still", TD_SUPPORT: "support", TD_FLOOR: "floor", TD_ALL: "all"}
# TD_ALL (v0.7.1b): above the flare band only support, floor and still together confirm (module docstring 5)


def mode_of_phase(phase: int) -> int:
    """Supervisor phase -> controller mode: CLIMB / HOLD -> FLY, LAND -> LAND, else IDLE."""
    if phase in (lp.PHASE_CLIMB, lp.PHASE_HOLD):
        return MODE_FLY
    if phase == lp.PHASE_LAND:
        return MODE_LAND
    return MODE_IDLE


def hover_throttle(mass_kg: float, t_max_n: float, throttle_exp: float, g: float = G0) -> float:
    """u that makes T_max u^n = m g (level, no lag): (m g / T_max)^(1/n).  >= 1 means the vehicle cannot hover."""
    return (mass_kg * g / t_max_n) ** (1.0 / throttle_exp)


def clampf(x: float, lo: float, hi: float) -> float:
    """``fc_clampf``: x < lo ? lo : (x > hi ? hi : x)."""
    return lo if x < lo else (hi if x > hi else x)


def _finite(x: float) -> bool:
    return not (math.isnan(x) or math.isinf(x))


def m_to_um(x: float) -> int:
    """metres -> integer micrometres, round half away from zero (``link_proto.round_half_away``), saturated to
    +-REF_LIMIT_UM; NaN -> 0.  Python evaluates x x 1e6 exactly for a float32 x (24 + 20 bits < 53); the C port must
    round the exact product too (e.g. in two float32 parts) or accept a 1 um difference at segment starts."""
    if x != x:
        return 0
    v = x * UM_PER_M
    if v >= REF_LIMIT_UM:
        return REF_LIMIT_UM
    if v <= -REF_LIMIT_UM:
        return -REF_LIMIT_UM
    return lp.round_half_away(v)


def target_um(h_tgt_m: float) -> int:
    """The supervisor's target [m] (integer decimetres / 10) -> um: round_half_away(h_tgt x 10) dm (i16, NaN -> 0)
    x 100000 - exact in float32 and float64 for every dm value."""
    return lp.quantize(float(h_tgt_m), lp.H_SCALE, lp.I16) * UM_PER_DM


def cos_tilt_from_q(q) -> float:
    """cos(tilt) of a unit quaternion (w, x, y, z): the z-z element of R(q), 1 - 2 (x^2 + y^2)."""
    x, y = float(q[1]), float(q[2])
    return 1.0 - 2.0 * (x * x + y * y)


@dataclass(frozen=True)
class VerticalParams:
    """Every number the controller uses (SI; derived integers and filter coefficients are properties).  ``from_params``
    is the constructor the sim uses."""

    loop_hz: int
    climb_rate_mps: float
    descent_rate_mps: float
    touchdown_rate_mps: float
    flare_altitude_m: float
    alt_kp: float                 # [1/s]    v_cmd per metre of reference error
    vz_kp: float                  # [s/m]    throttle per m/s of vertical-speed error
    vz_ki: float                  # [1/m]    throttle per metre of integrated error
    vz_i_max: float               # [-]      integrator limit (throttle)
    vz_max_up_mps: float
    vz_max_down_mps: float
    u_min: float
    u_max: float
    u_hover: float                # level hover throttle of the thrust model
    throttle_exp: float           # n of T = T_max u^n
    cos_tilt_min: float           # cos(tilt_abort_deg): feed-forward floor
    ramp_ticks: int               # throttle_ramp_s: take-off ceiling ramp
    landed_confirm_ticks: int
    td_vz_still_mps: float
    # ---- v0.7.1 ----
    vz_ff: bool = True            # mission.vtol.vz_ref_feedforward
    support_g: float = 0.25       # mission.vtol.touchdown.support_g_ASSUMED
    floor_ticks: int = 250        # mission.vtol.touchdown.floor_s_ASSUMED
    thrust_tau_s: float = 0.15    # edf.thrust_time_constant_s
    still_tau_s: float = TD_STILL_LPF_TAU_S_ASSUMED
    still_lead: float = TD_STILL_LEAD_ASSUMED   # still needs h_err <= -still_lead x touchdown_rate_mps
    support_tau_s: float = TD_SUPPORT_LPF_TAU_S_ASSUMED
    g: float = G0
    profile: str = ""             # mission.vtol profile the values came from (informational)
    # ---- v0.7.1b: thrust learning (mission.vtol.thrust_learning; module docstring 4b) ----
    kt_tau_s: float = 0.0         # tau_s_ASSUMED [s]; 0 = no learning (k_T stays clamp(1, kt_min, kt_max)) - the default
                                  # of a VerticalParams built without the params block (from_params reads it)
    kt_min: float = 0.7           # k_min (clamp of k_T)
    kt_max: float = 1.4           # k_max
    kt_max_tilt_deg: float = 15.0  # max_tilt_deg: learn only below this tilt
    # ---- v0.8: take-off check (module docstring 6; mission.vtol.position_hold.takeoff_check_ASSUMED) ----
    takeoff_climb_m: float = 0.0  # climb_m
    takeoff_ticks: int = 0        # ticks_ge(within_s); 0 = off
    takeoff_settle_ticks: int = 0  # h0 latched this many ticks after the ramp ends (ticks_ge(TAKEOFF_SETTLE_S_ASSUMED))
    # ---- v0.9: take-off for the light VTOL vehicle (module docstring 3b; mission.vtol.takeoff_v09) ----
    to_u_start: float = 0.0       # u_start: the ramp ceiling reaches it in ramp_ticks, then rises at to_slew_per_s;
                                  # 0 = the v0.8.1 ramp (0 -> u_max in ramp_ticks)
    to_slew_per_s: float = 0.0    # slew_per_s: second ramp segment [1/s]
    to_boost_per_s: float = 0.0   # boost_per_s: take-off thrust search [1/s]; 0 = off
    u_slew_per_s: float = 0.0     # u_slew_per_s: output slew limit in FLY / LAND [1/s]; 0 = off
    # ---- v0.9 fix round (control review medium 6, the take-off overshoot): take-off thrust learning (docstring 4c) ----
    to_learn_tau_s: float = 0.0   # learn_tau_s: k_T time constant from the lift-off to the take-off check; 0 = off
    to_airborne_fz: float = 0.05  # airborne_fz_frac: lift-off = the specific force above (1 + this) g (on the pad f_z = g)
    to_cap_margin: float = 0.0    # cap_margin: the take-off ceiling stops rising at u(lift-off) + this; 0 = off
    to_ref_hold: bool = False     # ref_hold: the reference waits on the pad until the lift-off (airborne) or the ramp end
    land_ref_rebase: bool = False  # mission.vtol.land_ref_rebase: entering LAND, ref = max(ref, h) (docstring 1c)
    ref_approach_shift: int = 0   # mission.vtol.ref_approach_shift: in FLY the reference step is at most |d| >> shift
                                  # (first-order approach, tau = 2^shift / loop_hz; 8 -> 1.02 s); 0 = off (docstring 1b)
    ref_approach_min_mps: float = 0.2  # ref_approach_min_mps: ... but never slower than this (the arrival stays finite)
    # ---- v0.9 fix round (physics review medium 3, ground effect): touchdown thrust release (docstring 5b) ----
    td_release_per_s: float = 0.0  # mission.vtol.touchdown.release_per_s_ASSUMED: throttle bias rate while the vehicle stops
                                   # following the descending reference in the flare band; 0 = off
    td_release_max: float = 0.12   # release_max_ASSUMED: the bias limit; 'still' confirms only with the full bias applied

    @property
    def dt(self) -> float:
        return 1.0 / self.loop_hz

    @cached_property
    def approach_min_um(self) -> int:
        """v0.9 fix round (1b): the approach's minimum reference step [um / tick] (>= 1)."""
        return max(self._step_um(self.ref_approach_min_mps), 1)

    @cached_property
    def kt_alpha_to(self) -> float:
        """v0.9 fix round: the take-off learning rate (dt / learn_tau_s; 0 = off)."""
        return 1.0 - math.exp(-self.dt / self.to_learn_tau_s) if self.to_learn_tau_s > 0 else 0.0

    @cached_property
    def ramp_total_ticks(self) -> int:
        """Ticks until the take-off ceiling reaches u_max: ramp_ticks (v0.8.1), or (v0.9) ramp_ticks + the ticks of the
        second segment, ceil((u_max - u_start) loop_hz / slew_per_s)."""
        if self.to_u_start <= 0.0 or self.ramp_ticks <= 0:
            return max(self.ramp_ticks, 0)
        return self.ramp_ticks + math.ceil((self.u_max - self.to_u_start) * self.loop_hz / self.to_slew_per_s - 1e-9)

    @cached_property
    def to_search_factor(self) -> float:
        """v0.9 take-off thrust search (3b): k_T x this per tick = u_ff + ~boost_per_s dt at the model hover (u_ff = u_hover
        / sqrt(k_T): du_ff / u_ff = -dk_T / (2 k_T))."""
        return 1.0 - 2.0 * self.to_boost_per_s / (self.loop_hz * self.u_hover)

    @cached_property
    def to_search_vz_mps(self) -> float:
        """v0.9 take-off thrust search: the NAV v_z above which the vehicle counts as climbing (TAKEOFF_SEARCH_VZ_FRAC_ASSUMED
        x climb_rate_mps)."""
        return TAKEOFF_SEARCH_VZ_FRAC_ASSUMED * self.climb_rate_mps

    def cap_at(self, k: int) -> float:
        """The take-off ceiling of ramp tick k (k = 0 on the first active tick)."""
        if self.ramp_ticks <= 0:
            return self.u_max
        if self.to_u_start <= 0.0:
            return self.u_max * min(1.0, (k + 1) / self.ramp_ticks)
        j = k + 1
        if j <= self.ramp_ticks:
            return self.to_u_start * j / self.ramp_ticks
        return min(self.u_max, self.to_u_start + (j - self.ramp_ticks) * self.to_slew_per_s / self.loop_hz)

    def _step_um(self, rate: float) -> int:
        return lp.round_half_away(rate * UM_PER_M / self.loop_hz)

    @cached_property
    def up_um(self) -> int:
        return self._step_um(self.climb_rate_mps)

    @cached_property
    def dn_um(self) -> int:
        return self._step_um(self.descent_rate_mps)

    @cached_property
    def td_um(self) -> int:
        return self._step_um(self.touchdown_rate_mps)

    @cached_property
    def flare_um(self) -> int:
        return m_to_um(self.flare_altitude_m)

    @cached_property
    def inv_u_hover(self) -> float:
        return 1.0 / self.u_hover

    @cached_property
    def thrust_alpha(self) -> float:
        return 1.0 - math.exp(-self.dt / self.thrust_tau_s) if self.thrust_tau_s > 0 else 1.0

    @cached_property
    def still_alpha(self) -> float:
        return 1.0 - math.exp(-self.dt / self.still_tau_s) if self.still_tau_s > 0 else 1.0

    @cached_property
    def support_alpha(self) -> float:
        return 1.0 - math.exp(-self.dt / self.support_tau_s) if self.support_tau_s > 0 else 1.0

    @cached_property
    def kt_alpha(self) -> float:
        """a_K = 1 - exp(-dt / kt_tau_s); 0 when kt_tau_s is 0 (no learning)."""
        return 1.0 - math.exp(-self.dt / self.kt_tau_s) if self.kt_tau_s > 0 else 0.0

    @cached_property
    def kt_cos_min(self) -> float:
        """cos(kt_max_tilt_deg): learn only while c > this (tilt < kt_max_tilt_deg)."""
        return math.cos(math.radians(self.kt_max_tilt_deg))

    @cached_property
    def kt_reset(self) -> float:
        """k_T at each take-off: clamp(1, kt_min, kt_max) (1 with the params.json limits)."""
        return clampf(1.0, self.kt_min, self.kt_max)

    def effective_rate(self, step_um: int) -> float:
        """The rate a step really flies [m/s] (the micrometre rounding of rate x dt)."""
        return step_um * self.loop_hz / UM_PER_M

    @classmethod
    def from_params(cls, p: dict, mass_kg: float, t_max_n: float, g: float = G0, profile: str | None = None,
                    **overrides) -> "VerticalParams":
        """mission.vtol merged with its build profile (``sim.onboard.vtol_values``; gains from gains_ASSUMED, the
        touchdown block, vz_ref_feedforward, v0.7.1b thrust_learning - absent: no learning), mission.throttle_ramp_s /
        tilt_abort_deg, edf.throttle_exponent and thrust_time_constant_s; u_hover from the vehicle mass and T_max (the
        mixer's edf.thrust_nominal_N).  Keyword overrides replace fields (gain tuning)."""
        m = p["mission"]
        v = vtol_values(p, profile)
        gn = v["gains_ASSUMED"]
        td = v.get("touchdown", {})
        tl = v.get("thrust_learning") or {}
        hz = int(p["control"]["loop_hz"])
        n = float(p["edf"]["throttle_exponent"])
        tr = float(v["touchdown_rate_mps"])
        kw = dict(
            loop_hz=hz, climb_rate_mps=float(v["climb_rate_mps"]), descent_rate_mps=float(v["descent_rate_mps"]),
            touchdown_rate_mps=tr, flare_altitude_m=float(v["flare_altitude_m"]), alt_kp=float(gn["alt_kp"]),
            vz_kp=float(gn["vz_kp"]), vz_ki=float(gn["vz_ki"]), vz_i_max=float(gn["vz_i_max"]),
            vz_max_up_mps=float(v["vz_max_up_mps"]), vz_max_down_mps=float(v["vz_max_down_mps"]),
            u_min=float(v["u_min"]), u_max=float(v["u_max"]), u_hover=hover_throttle(mass_kg, t_max_n, n, g),
            throttle_exp=n, cos_tilt_min=math.cos(math.radians(float(m["tilt_abort_deg"]))),
            ramp_ticks=ticks_ge(float(m["throttle_ramp_s"]), hz),
            landed_confirm_ticks=max(1, ticks_ge(float(v["landed_confirm_s"]), hz)),
            td_vz_still_mps=TD_STILL_FRACTION_ASSUMED * tr,
            vz_ff=bool(v.get("vz_ref_feedforward", False)),
            support_g=float(td.get("support_g_ASSUMED", 0.25)),
            td_release_per_s=float(td.get("release_per_s_ASSUMED", 0.0)),
            td_release_max=float(td.get("release_max_ASSUMED", 0.12)),
            floor_ticks=max(1, ticks_ge(float(td.get("floor_s_ASSUMED", 1.0)), hz)),
            thrust_tau_s=float(p["edf"]["thrust_time_constant_s"]), g=g, profile=str(v.get("profile", "")))
        if tl:
            kw.update(kt_tau_s=float(tl["tau_s_ASSUMED"]), kt_min=float(tl["k_min"]), kt_max=float(tl["k_max"]),
                      kt_max_tilt_deg=float(tl["max_tilt_deg"]))
        tc = (v.get("position_hold") or {}).get("takeoff_check_ASSUMED")
        if tc:
            kw.update(takeoff_climb_m=float(tc["climb_m"]), takeoff_ticks=max(1, ticks_ge(float(tc["within_s"]), hz)),
                      takeoff_settle_ticks=ticks_ge(TAKEOFF_SETTLE_S_ASSUMED, hz))
        t9 = v.get("takeoff_v09")
        if t9:
            kw.update(to_u_start=float(t9.get("u_start", 0.0)), to_slew_per_s=float(t9.get("slew_per_s", 0.0)),
                      to_boost_per_s=float(t9.get("boost_per_s", 0.0)), u_slew_per_s=float(t9.get("u_slew_per_s", 0.0)),
                      to_learn_tau_s=float(t9.get("learn_tau_s", 0.0)),
                      to_airborne_fz=float(t9.get("airborne_fz_frac", 0.05)),
                      to_cap_margin=float(t9.get("cap_margin", 0.0)), to_ref_hold=bool(t9.get("ref_hold", False)))
        kw["ref_approach_shift"] = int(v.get("ref_approach_shift", 0))
        kw["ref_approach_min_mps"] = float(v.get("ref_approach_min_mps", 0.2))
        kw["land_ref_rebase"] = bool(v.get("land_ref_rebase", False))
        kw.update({k: val for k, val in overrides.items() if val is not None})
        vp = cls(**kw)
        vp.check()
        return vp

    def check(self) -> None:
        """Relations the equations rely on (ValueError otherwise).  u_hover >= u_max is allowed: the vehicle then
        cannot hover, which the simulation must show, not hide."""
        if not (0.0 <= self.u_min < self.u_max <= 1.0):
            raise ValueError(f"need 0 <= u_min < u_max <= 1, got {self.u_min}, {self.u_max}")
        for k in ("climb_rate_mps", "descent_rate_mps", "touchdown_rate_mps", "vz_max_up_mps", "vz_max_down_mps",
                  "throttle_exp", "loop_hz", "u_hover"):
            if not getattr(self, k) > 0:
                raise ValueError(f"{k} must be > 0")
        for k in ("alt_kp", "vz_kp", "vz_ki", "vz_i_max", "td_vz_still_mps", "flare_altitude_m", "support_g",
                  "thrust_tau_s", "still_tau_s", "support_tau_s"):
            if not getattr(self, k) >= 0:
                raise ValueError(f"{k} must be >= 0")
        if not 0.0 < self.cos_tilt_min <= 1.0:
            raise ValueError("cos_tilt_min must be in (0, 1]")
        if not self.touchdown_rate_mps <= self.vz_max_down_mps:
            raise ValueError("touchdown_rate_mps must not exceed vz_max_down_mps (the touchdown command would be "
                             "unreachable)")
        for k in ("up_um", "dn_um", "td_um"):
            if getattr(self, k) < 1:
                raise ValueError(f"{k} = {getattr(self, k)}: the rate is below 1 um per tick")
        if self.floor_ticks < 1 or self.landed_confirm_ticks < 1:
            raise ValueError("floor_ticks and landed_confirm_ticks must be >= 1")
        if not (self.kt_tau_s >= 0.0 and 0.0 < self.kt_min <= self.kt_max and 0.0 <= self.kt_max_tilt_deg < 90.0):
            raise ValueError("thrust learning: need kt_tau_s >= 0, 0 < kt_min <= kt_max, 0 <= kt_max_tilt_deg < 90")
        if not (self.takeoff_climb_m >= 0.0 and self.takeoff_ticks >= 0 and 0 <= self.takeoff_settle_ticks <= 0xFFFF):
            raise ValueError("take-off check: need climb_m >= 0, within_s >= 0 and 0 <= settle ticks <= 65535")
        if not (0.0 <= self.to_u_start < self.u_max and self.to_boost_per_s >= 0.0 and self.u_slew_per_s >= 0.0):
            raise ValueError("v0.9 take-off: need 0 <= u_start < u_max, boost_per_s >= 0, u_slew_per_s >= 0")
        if self.to_u_start > 0.0 and not self.to_slew_per_s > 0.0:
            raise ValueError("v0.9 take-off: u_start > 0 needs slew_per_s > 0")

    @property
    def hover_margin(self) -> float:
        """u_max - u_hover: throttle authority above level hover (<= 0: cannot hover)."""
        return self.u_max - self.u_hover

    @property
    def u_floor_reachable(self) -> float:
        """The lowest u the loop can command with the vehicle stopped (v_z = 0) at the deepest command:
        u_hover - vz_kp vz_max_down - vz_i_max (tilt 0); above u_min -> the sat == -1 clause of the floor signature cannot
        occur on the ground (the integrator clause can - docstring 4)."""
        return self.u_hover - self.vz_kp * self.vz_max_down_mps - self.vz_i_max


@dataclass(frozen=True)
class VerticalInputs:
    phase: int                    # supervisor phase of this tick (link_proto.PHASE_*)
    h_tgt_m: float                # supervisor altitude target (FLY; the integer dm / 10)
    baro_lost: bool               # supervisor: the baro is lost - h is the NAV's dead reckoning (no flare, no V10 count)
    h_m: float                    # NAV altitude filter
    vz_mps: float
    tilt_deg: float = 0.0         # attitude estimate - used only when cos_tilt is None (reference convenience)
    cos_tilt: float | None = None   # v0.7.1 FCU path: 1 - 2 (qx^2 + qy^2) of the NAV quaternion (``cos_tilt_from_q``)
    f_z_mps2: float | None = None   # v0.7.1: NAV body-z specific force (nlink accel[2]); None: no support signature
    u_cap: float | None = None      # v0.9: the supervisor's power ceiling (W_SAG stage 1: frozen at the throttle of the
                                    # warning's onset); None: none.  cap = min(take-off ceiling, u_cap)


@dataclass(frozen=True)
class VerticalOutputs:
    mode: int                     # MODE_*
    u: float                      # throttle command (0 when IDLE)
    h_ref: float
    vz_cmd: float
    vz_err: float                 # e = v_cmd - v_z (0 if not finite)
    u_ff: float
    integ: float                  # integrator (throttle units)
    sat: int                      # +1 at the ceiling (u_max or the take-off ramp), -1 at u_min, 0 inside
    at_target: bool               # FLY: h_ref == target this tick
    touchdown: bool               # LAND: touchdown confirmed (the supervisor goes LANDED next tick)
    td_count: int                 # the largest of the three signature counters
    # ---- v0.7.1 ----
    dh_ref_mps: float = 0.0       # reference rate of this tick (the feed-forward term)
    cos_tilt: float = 1.0         # c after the clamp
    low: bool = False             # LAND: h <= flare or the loop saturated downward
    still: bool = False           # signature (a)
    support: bool = False         # signature (b)
    floor: bool = False           # signature (c)
    ground_signature: bool = False  # support or floor (the supervisor's baro-lost time limit)
    low_in_land_ticks: int = 0    # consecutive LAND ticks in the flare band, leading, baro healthy (V10 floor timeout)
    td_by: int = TD_NONE          # which signature confirmed the touchdown (TD_*)
    vz_lpf: float = 0.0
    f_excess_mps2: float = 0.0    # d_lpf: body-z specific force beyond the (learnt) thrust model
    thrust_ratio: float = 0.0     # r: modelled T / (m g) through the thrust lag
    ref_um: int = 0               # the exact reference
    # ---- v0.7.1b ----
    k_thrust: float = 1.0         # k_T after this tick (the next tick's feed-forward; this tick's d)
    kt_learn: bool = False        # k_T learnt this tick (module docstring 4b)
    low_alt: bool = False         # LAND: h <= flare_altitude_m - the band in which one signature confirms (gating)
    # ---- v0.8 ----
    takeoff_fail: bool = False    # take-off check failed (module docstring 6): the supervisor stops the motor next tick
    takeoff_run: int = 0          # consecutive saturated ticks without the climb
    takeoff_passed: bool = False  # climbed takeoff_climb_m since the start
    airborne: bool = False        # v0.9 fix round (4c): the lift-off was seen (latched for the flight)


IDLE_OUT = VerticalOutputs(MODE_IDLE, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, False, False, 0)


class VerticalController:
    """The algorithm of the module docstring; call ``update`` once per control tick."""

    def __init__(self, vp: VerticalParams):
        self.vp = vp
        self.reset()

    def reset(self) -> None:
        self.active = False
        self.ref_um = 0
        self.integ = 0.0
        self.ramp_k = 0
        self.cnt = [0, 0, 0, 0]               # still, support, floor, all-three confirmation counters
        self.floor_run = 0
        self.low_in_land = 0
        self.v_lpf = 0.0
        self.d_lpf = 0.0
        self.r_thr = 0.0
        self.u_last = 0.0
        self.k_t = self.vp.kt_reset           # v0.7.1b: the learnt thrust ratio, 1 at each take-off
        self.to_h0 = 0.0                      # v0.8 take-off check
        self.to_passed = False
        self.to_run = 0
        self.to_h0_ok = False                 # h0 latched (the ramp over + the settle time)
        self.to_settle = 0                    # ticks since the ramp ended (saturating at the settle time)
        self.airborne = False                 # v0.9 fix round: lift-off seen (specific force > (1 + airborne_fz) g)
        self.cap_lo = 2.0                     # v0.9 fix round: the take-off ceiling frozen at the lift-off (+ margin)
        self.fz_lpf = self.vp.g               # v0.9 fix round: low-passed specific force until the lift-off
        self.td_bias = 0.0                    # v0.9 fix round (5b): the touchdown thrust release (<= 0)
        self.in_land = False                  # v0.9 fix round (1c): the previous tick was LAND

    @property
    def h_ref(self) -> float:
        return self.ref_um / UM_PER_M

    @h_ref.setter
    def h_ref(self, x: float) -> None:            # tests / studies: place the reference
        self.ref_um = m_to_um(x)

    @property
    def td_count(self) -> int:
        return max(self.cnt)

    def update(self, inp: VerticalInputs) -> VerticalOutputs:
        vp = self.vp
        mode = mode_of_phase(inp.phase)
        if mode == MODE_IDLE:
            self.reset()
            return IDLE_OUT if self.k_t == 1.0 else _replace(IDLE_OUT, k_thrust=self.k_t)
        h, vz = inp.h_m, inp.vz_mps
        if not self.active:                                         # 0. start
            self.reset()
            self.active = True
            self.ref_um = m_to_um(h) if _finite(h) else 0
            self.to_h0 = h if _finite(h) else 0.0
        # 1. reference (integer micrometres) and 2. vertical-speed command
        at_target = False
        was_land = self.in_land
        self.in_land = mode == MODE_LAND
        delta = 0
        if mode == MODE_FLY:
            tgt = target_um(inp.h_tgt_m)
            d = tgt - self.ref_um
            delta = d if -vp.dn_um <= d <= vp.up_um else (vp.up_um if d > 0 else -vp.dn_um)
            if vp.ref_approach_shift > 0 and d != 0:
                # 1b (fix round): the reference decelerates into its target (first-order approach, one shift per tick):
                # the feed-forward then fades instead of stepping from the climb rate to 0 at the target - the inner
                # velocity loop (crossover ~1.3 rad/s against alt_kp 1.0) carried the step into a 0.5-1 m overshoot
                lim = max(abs(d) >> vp.ref_approach_shift, vp.approach_min_um)
                if abs(delta) > lim:
                    delta = lim if delta > 0 else -lim
            if (vp.to_ref_hold and not self.airborne and not self.to_passed and vp.ramp_total_ticks > 0
                    and self.ramp_k + 1 < vp.ramp_total_ticks):
                # 4c (fix round): the climb reference waits on the pad (support signature of the previous tick) for the
                # lift-off - with the two-slope ramp it had run 1-1.6 m ahead by the lift-off and the vehicle climbed
                # through its target catching up (control review medium 6); it starts when the lift-off latches or the
                # ramp ends.  Tried on TUNE: starting it when the support goes or when the ceiling reaches the hover
                # feed-forward (no re-contact of a T ~ W plant with the pad: 6 -> 2 of 156 runs) raised the field
                # overshoot (> 0.5 m 8 -> 17 of 40) - not adopted; the re-contacts are judged apart (landing_touchdown)
                delta = 0
        else:                                                       # LAND (baro healthy or lost: the same law)
            if vp.land_ref_rebase and not was_land and _finite(h) and m_to_um(h) > self.ref_um:
                # 1c (fix round): a LAND that starts with the vehicle above its reference (a rule acting in the climb:
                # W_LOST 1 s after the lift-off) descends from the vehicle's height - the error would otherwise ask for
                # vz_max_down near the ground (P_H_unplug_climb touched at 1.41 m/s)
                self.ref_um = m_to_um(h)
            low_ref = (self.ref_um <= vp.flare_um or h <= vp.flare_altitude_m) and not inp.baro_lost
            delta = -(vp.td_um if low_ref else vp.dn_um)
        self.ref_um = min(max(self.ref_um + delta, -REF_LIMIT_UM), REF_LIMIT_UM)
        if mode == MODE_FLY:
            at_target = self.ref_um == tgt
        h_ref = self.ref_um / UM_PER_M
        dh_ref = delta * vp.loop_hz / UM_PER_M
        h_err = vp.alt_kp * (h_ref - h)                             # the altitude-feedback part of the command
        vz_cmd = clampf((dh_ref if vp.vz_ff else 0.0) + h_err, -vp.vz_max_down_mps, vp.vz_max_up_mps)
        # 3. throttle (k_T of the previous tick)
        if inp.cos_tilt is not None:
            c = inp.cos_tilt
        else:
            c = math.cos(math.radians(inp.tilt_deg)) if _finite(inp.tilt_deg) else 1.0
        c_ok = _finite(c)
        if not c_ok:
            c = 1.0
        c = clampf(c, vp.cos_tilt_min, 1.0)
        kc = self.k_t * c
        u_ff = vp.u_hover / math.sqrt(kc) if vp.throttle_exp == 2.0 else vp.u_hover * kc ** (-1.0 / vp.throttle_exp)
        e = vz_cmd - vz
        if not _finite(e):
            e = 0.0
        cap = vp.cap_at(self.ramp_k)
        if vp.to_cap_margin > 0.0 and self.airborne and self.cap_lo < cap:
            cap = self.cap_lo                  # v0.9 fix round (4c): the ceiling stops rising at the lift-off
        if inp.u_cap is not None and inp.u_cap < cap:
            cap = inp.u_cap                    # v0.9 W_SAG: the supervisor's frozen power ceiling (no wind-up above it)
        rt = vp.ramp_total_ticks
        ramp_over = rt <= 0 or self.ramp_k + 1 >= rt
        # v0.9 take-off thrust search (3b): in FLY before the take-off check passed, with the ramp over and the loop
        # asking for a climb, the learnt thrust ratio falls (u_ff of the NEXT tick rises by ~boost_per_s dt)
        searching = False
        if (vp.to_boost_per_s > 0.0 and vp.takeoff_ticks > 0 and mode == MODE_FLY and not self.to_passed and ramp_over
                and e > 0.0 and not vz > vp.to_search_vz_mps and self.k_t > vp.kt_min
                and not (self.airborne and vp.kt_alpha_to > 0.0)):      # fix round: airborne -> the learning (4c) rules
            self.k_t = max(self.k_t * vp.to_search_factor, vp.kt_min)
            searching = True
        i_try = clampf(self.integ + vp.vz_ki * e * vp.dt, -vp.vz_i_max, vp.vz_i_max)
        u_try = u_ff + vp.vz_kp * e + i_try
        if not ((u_try > cap and e > 0.0) or (u_try < vp.u_min and e < 0.0)):
            self.integ = i_try
        u_raw = u_ff + vp.vz_kp * e + self.integ + self.td_bias
        u = min(max(u_raw, vp.u_min), cap)
        sat = 1 if u_raw > cap else (-1 if u_raw < vp.u_min <= cap else 0)
        if vp.u_slew_per_s > 0.0 and ramp_over:
            # v0.9 output slew limit (3b): bounds the EDF rotor's spool reaction torque (I_rot domega/dt about body z);
            # the take-off ramp is slower than it anyway
            du_max = vp.u_slew_per_s * vp.dt
            u = min(max(u, self.u_last - du_max), self.u_last + du_max)
        # 6. take-off check (v0.8): ramp over (cap = u_max) and pinned at it without having climbed above the h0
        #    latched after the ramp + the settle time (physics-review fix)
        if vp.takeoff_ticks > 0 and not self.to_passed:
            if searching:                       # v0.9: the thrust still rises - h0 waits for it (thrust-proportional
                self.to_h0_ok = False           # baro offset: the physics-review fix assumed u constant after the ramp)
                self.to_settle = 0
            if not self.to_h0_ok and ramp_over:
                if self.to_settle >= vp.takeoff_settle_ticks:
                    self.to_h0 = h if _finite(h) else 0.0
                    self.to_h0_ok = True
                else:
                    self.to_settle += 1
            if self.to_h0_ok and _finite(h) and h >= self.to_h0 + vp.takeoff_climb_m:
                self.to_passed = True
                self.to_run = 0
            elif mode == MODE_FLY and ramp_over and sat == 1:
                self.to_run += 1
            else:
                self.to_run = 0
        takeoff_fail = vp.takeoff_ticks > 0 and self.to_run >= vp.takeoff_ticks
        self.ramp_k = min(self.ramp_k + 1, max(rt, 0))
        # 4. touchdown signals (u_last = the throttle applied since the previous tick)
        x = self.u_last * vp.inv_u_hover
        r_cmd = x * x if vp.throttle_exp == 2.0 else x ** vp.throttle_exp
        self.r_thr += vp.thrust_alpha * (r_cmd - self.r_thr)
        f_ok = inp.f_z_mps2 is not None and _finite(inp.f_z_mps2)
        gr = vp.g * self.r_thr                                      # [m/s^2] the model's specific force at k_T = 1
        # 4b. thrust learning (free flight above the flare band, after the take-off ramp, small tilt)
        learn = (vp.kt_alpha > 0.0 and self.ramp_k >= vp.ramp_total_ticks and f_ok and self.r_thr >= KT_R_MIN_ASSUMED
                 and h > vp.flare_altitude_m and c_ok and c > vp.kt_cos_min)
        # 4c (v0.9 fix round): from the lift-off to the take-off check the learning runs at the take-off rate - the
        # strong light vehicle (k 1.2-1.5 against the model) otherwise climbs through its target on a feed-forward the
        # +-vz_i_max integrator cannot correct before the slow learning starts above the flare band
        learn_to = (vp.kt_alpha_to > 0.0 and self.airborne and not self.to_passed and mode == MODE_FLY and f_ok
                    and self.r_thr >= KT_R_MIN_ASSUMED and c_ok and c > vp.kt_cos_min)
        if learn or learn_to:
            self.k_t += (vp.kt_alpha_to if learn_to else vp.kt_alpha) * (inp.f_z_mps2 / gr - self.k_t)
            self.k_t = clampf(self.k_t, vp.kt_min, vp.kt_max)
        if f_ok:
            self.d_lpf += vp.support_alpha * ((inp.f_z_mps2 - self.k_t * gr) - self.d_lpf)
            if ((vp.kt_alpha_to > 0.0 or vp.to_cap_margin > 0.0 or vp.to_ref_hold) and not self.airborne
                    and mode == MODE_FLY):
                # lift-off (4c): the specific force (low-passed like the support signature) above (1 + airborne_fz) g -
                # on the pad it stays at g whatever the thrust (the pad carries the rest); runs only until the lift-off
                self.fz_lpf += vp.support_alpha * (inp.f_z_mps2 - self.fz_lpf)
                if self.fz_lpf > vp.g * (1.0 + vp.to_airborne_fz):
                    self.airborne = True
                    self.cap_lo = min(self.u_last + vp.to_cap_margin, vp.u_max)
        learn = learn or learn_to
        if _finite(vz):
            self.v_lpf += vp.still_alpha * (vz - self.v_lpf)
        lead = h_err <= -vp.still_lead * vp.touchdown_rate_mps        # the vehicle does not follow the reference down
        pinned = sat == -1 or (self.integ <= -vp.vz_i_max and vz_cmd <= -vp.vz_max_down_mps)
        self.floor_run = self.floor_run + 1 if pinned else 0
        still = abs(self.v_lpf) <= vp.td_vz_still_mps and lead
        support = f_ok and self.d_lpf > vp.support_g * vp.g
        floor = self.floor_run >= vp.floor_ticks
        self.u_last = u
        # 5. touchdown detector (gating v0.7.1b: one signature in the flare band, all three together above it)
        touchdown, td_by, low, low_alt = False, TD_NONE, False, False
        if mode == MODE_LAND:
            low_alt = h <= vp.flare_altitude_m
            low = low_alt or vz_cmd <= -vp.vz_max_down_mps
            still_ok = still
            if vp.td_release_per_s > 0.0:
                # 5b (fix round): while the vehicle stands still and does not follow the descending reference in the
                # flare band, the throttle is released (bias down to -release_max, next tick); a vehicle floating on its
                # ground cushion
                # (the exit is the contact plane) sinks to the ground - 'still' confirms only with the release complete
                if low_alt and still and not inp.baro_lost:      # still = not moving (1 s v_z low-pass) AND the lead
                    self.td_bias = max(self.td_bias - vp.td_release_per_s * vp.dt, -vp.td_release_max)
                else:
                    self.td_bias = min(self.td_bias + vp.td_release_per_s * vp.dt, 0.0)
                still_ok = still and self.td_bias <= -vp.td_release_max + 1e-9
            for j, ok in enumerate((low_alt and still_ok, low_alt and support, low_alt and floor,
                                    low and support and floor and (still or inp.baro_lost))):
                self.cnt[j] = self.cnt[j] + 1 if ok else 0
            for j in range(4):
                if self.cnt[j] >= vp.landed_confirm_ticks:
                    touchdown, td_by = True, j + 1
                    break
            self.low_in_land = self.low_in_land + 1 if (low_alt and lead and not inp.baro_lost) else 0
        else:
            self.cnt = [0, 0, 0, 0]
            self.low_in_land = 0
            self.td_bias = 0.0
        return VerticalOutputs(mode, u, h_ref, vz_cmd, e, u_ff, self.integ, sat, at_target, touchdown, max(self.cnt),
                               dh_ref_mps=dh_ref, cos_tilt=c, low=low, still=still, support=support, floor=floor,
                               ground_signature=support or floor, low_in_land_ticks=self.low_in_land, td_by=td_by,
                               vz_lpf=self.v_lpf, f_excess_mps2=self.d_lpf, thrust_ratio=self.r_thr, ref_um=self.ref_um,
                               k_thrust=self.k_t, kt_learn=learn, low_alt=low_alt, takeoff_fail=takeoff_fail,
                               takeoff_run=self.to_run, takeoff_passed=self.to_passed, airborne=self.airborne)
