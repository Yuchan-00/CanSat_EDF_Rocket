"""v0.8 / v0.8.1 position hold (SPEC 8.9 + GPS chapter) - the reference.

Executable specification for ``firmware/atmega128_fc/src/core/position_ctrl.c`` (float32 on the board, float64 here;
counters and every limit decision are integer / comparison-exact in both up to float32 rounding at the thresholds).
Pure Python, no numpy, no trigonometry in the per-step path (review v0.8 (2): the prototype's acos / tan / atan / cos /
sin / hypot cost 0.82 ms per step and 0.52 ms per setpoint build on the ATmega128A; this trig-free form measured
0.50 / 0.14 ms in the review's simavr bench).

v0.8.1 (architect 2026-10-21 20:30, params flight_computer.architecture_v081): the step runs on the FOURTH ATmega128A, the
GNS board (``sim.glink_proto.GnsNode``), once per glink STATE frame (50 Hz), the whole step at once right after the GPS
filter's 50 Hz step (the GNS has the full 20 ms cycle: no job pipeline, ``PIPELINE_TICKS`` is the v0.8 FCU build's).
Inputs: the GPS data from the GNS's own filter (float, not quantised: ``GpsData`` built from ``sim.gps_nav.NavGpsFilter``
at its latch), h / cos(tilt) / body rates from the STATE frame (the NAV's tick 5 m + 4), phase / motor_on / k_T /
vc_sat_high (OR over the cycle) / vc_low_alt / vc_ground / lifted from the FCU's nlink v4 frame as the NAV last received it
(``sim.nlink_proto`` Fcu4).  Outputs go back as the glink RESULT (TILT = ``tilt_l``, TRIM = ``trim_l``, STATUS with the
mode and the requests, POS, VEL) and reach the FCU's tilt-setpoint builder through the nlink v4 slot; the step logic below
is unchanged - the FCU keeps the supervisor rules V11-V15 on the slot data (``sim.onboard.FcuStack`` gns mode).

Where it sits (``sim.onboard.FcuStack``, v0.8 FCU tick order): attitude controller (with the tilt setpoint built at
the end of the PREVIOUS tick) -> vanes -> supervisor -> vertical controller -> ESC -> [the position step, on the ticks
it is due, as the pipeline below] -> the tilt setpoint builder (``TiltSetpoint.tick``, every tick).  A step's tilt
target enters the builder at the end of its commit tick and the attitude controller in the tick after ("used from the
next tick" after the commit); its LAND requests reach the supervisor in the tick after the commit (like the vertical
controller's flags).

Input data (``sim.nlink_proto`` v3 GPS slots, complete set of one NAV step, quantised on the NAV): p, v (W frame: x
north, y west; int16 cm, cm/s), the NAV's cumulative reset offset (int16 cm) and reset counter, the heading cos / sin
(Q15), flags (FIX_OK, GATE_OK, ...), numSV, hAcc.  From the FCU: phase / motor_on (supervisor, this tick), h and
cos(tilt) and the body rates (this tick's NAV frame), the learnt thrust ratio k_T and the saturation flag of the
vertical controller (``vc_sat_high`` = sat == +1 in any tick since the previous step).

When the step runs (FcuStack): on every FCU tick whose NAV frame carried slot index 4, or when 5 ticks passed since
the previous step (a lost slot-4 frame); a step without a complete set of one cycle is a HOLD step: outputs and
the controller state unchanged, only the loss counter and the guard timer advance.  Step period T = 1 / loop_hz
(params mission.vtol.position_hold.loop_hz 50 Hz).

Pipeline (normative, review v0.8 (2): FCU background jobs <= 0.2 ms each): the step's INPUTS are latched at the due tick
k (after the ESC write), the work runs as five background jobs, one per tick - J0 (k) latch the inputs and the set,
loss counter, mode, guard trigger, tilt-limit cache; J1 (k + 1) hold-point rules, position error, distance, v_sp and its
limit, fence; J2 (k + 2) D term, integrator, a_cmd, conditional integration; J3 (k + 3) the a limit, plausibility,
authority; J4 (k + 4) tilt series, rotation into the level frame, COMMIT of the state and the outputs.  The outputs are
therefore used from tick k + ``PIPELINE_TICKS`` (4): the tilt builder at the end of k + 4 (the attitude controller at
k + 5), the supervisor's LAND requests and the TELEM fields at k + 5.  This reference computes the step at once at k
(its state is not read before k + 5, when the next step can come) and ``sim.onboard.FcuStack`` publishes the outputs
at k + 4 - the same numbers at the same ticks.  The added 16 ms is small against the 0.22 s GPS latency and the 20 ms
step.

Per step (all quantities W frame unless noted; |x|^2 = x . x; "limit x to L" = if |x|^2 > L^2: x *= L / sqrt(|x|^2) -
the only square roots, taken when a limit acts):

 0. Not flying (motor off or phase not CLIMB / HOLD / LAND): reset everything, tilt target 0.
 1. Engage when h > ``H_ENABLE_M`` (0.3 m; not while sitting on the pad - the take-off ramp is upright) AND the vertical
    controller confirmed the lift-off (``lifted`` = its take-off check passed: h rose takeoff_climb_m above the height
    latched after the ramp + settle; v0.8 physics review medium 4: an uncalibrated thrust-proportional baro offset put
    the NAV h at 0.98 m with the vehicle on the pad and engaged the loop there; ``lifted`` is True when the take-off
    check is off); before: tilt target 0.
 2. ok = complete set and FIX_OK and not implausible.  lost = consecutive steps without ok (a hold step counts).
    mode: ATT if implausible or lost >= timeout (gps_loss.timeout_s); else DR if not ok (the NAV's dead-reckoned v,
    velocity hold); else POS once a hold point exists, VEL before (the climb: v_sp = 0).
    LAND request R_V_GPS_LOST when lost >= timeout + land_after (gps_loss.land_after_s after the attitude-only hold
    began: 4 s after the last good step with params' 1 + 3 s; the demo profile's gps_loss.demo_land_after_s 0 lands
    1 s after the last good step - architect 18:10 after the physics review: a demo GPS loss in 4 m/s wind drifted to
    the spectator line).
 3. Hold point, held in the reset-free frame p_rf = p - offset (a NAV position reset moves p and offset together, so
    the vehicle does not jump with the GPS): captured = p_rf at the first ok step in HOLD or LAND (= the position at
    the end of the climb); UP / DOWN keep it; OFF -> LAND keeps it (lands at it); the GPS back during a LAND after a
    loss (lost >= timeout: the attitude-only hold had begun) -> captured again (land where it is, never fly back); a
    shorter gap (a lost frame, dead reckoning) keeps it; re-captured after every take-off (reset at 0).
 4. Tilt limit from the thrust margin (params tilt_margin_rule, prototype): x = k_T T/W (model T/W), m = min(0.1,
    0.5 max(x - 1, 0)), r = (1 + m) / x; tilt_lim = acos(r) capped at the profile's tilt_cap_deg (field 20 / demo 10;
    the same cap in the flare: the separate 8 deg flare cap is REMOVED - params flare_tilt_note, architect 18:10 after
    the v0.8 physics review HIGH 1: the steady trim is 7.0 deg at 5 m/s and 9.35 deg at 6 m/s, so 8 deg saturated the
    loop for the whole last metre and the vehicle touched down drifting at 0.5-1.0 m/s; ``flare_tilt_cap_deg`` stays a
    field equal to the profile cap - check() refuses a lower one - so the generator's POS_COS_FLARE_CAP and the C
    API keep their shape).  Trig-free: cos_lim = max(r, cos(cap)) (cos(cap) a constant), a_lim^2 = g^2 (1 - cos_lim^2)
    / cos_lim^2 (r >= 1: 0).  C: recomputed only when k_T or the cap changes.
    Touchdown rule (review v0.8 (5); switched on the VERTICAL CONTROLLER's flags, params flare_tilt_note): in LAND,
    when the vertical controller sees the ground (``vc_ground``: a touchdown signature - still / support / floor - in
    its flare band) a_lim^2 = 0 (upright on the ground, whatever the NAV height says); else in its flare band
    (``vc_low_alt``) a factor f = h / TILT_FADE_M for 0 <= h < TILT_FADE_M, f = 0 for -TILT_FADE_M <= h < 0 (the ground
    beside the pad lies ~0.11 m below it: the pad's foot level), and f = min((-h - TILT_FADE_M) / TILT_FADE_M, 1) further
    below - an airborne vehicle that far below the pad level has lower ground under it, so the hold comes back until the
    vertical controller sees the ground (physics review low: ground 1.5 m below the pad lost the hold for the last 1.5 m
    and touched down at 1.25 m/s).  What f does is the touchdown policy (params mission.vtol.position_hold.
    touchdown_policy.policy, ``PositionParams.touchdown_policy``):
      "keep_trim" (v0.8.1, architect 21:00): only the TRANSIENT above the wind trim is limited - after the velocity loop
          (step 8) e = a - i (i = the integrator = the trim) is limited to |e| <= f a_lim (|e|^2 > f^2 a_lim^2: e *= sqrt(
          f^2 a_lim^2 / |e|^2), the only square root, taken when it acts; f = 0: e = 0), a = i + e, then the profile
          limit as before.  The vehicle keeps leaning into the wind with its trim to the contact and holds position,
          corrections fade out over the last TILT_FADE_M.  Why (params touchdown_policy note, sim/out/vtol/
          v08_physreview_fix_studies.json): fading the whole tilt to 0 let the wind push the vehicle sideways at the
          contact - energy-method tip-over margin negative in 8 / 20 demo runs at 4 m/s; the physical criterion is the
          tip-over margin from the lean AND the ground speed (``sim.vtol_scenarios.v08_checks``).
      "fade" (v0.8, kept as the comparison build): a_lim^2 *= f^2 - the vehicle is upright when it touches pad-level
          ground (lean into the wind traded for drift in the last ~1 s).
      "keep_vel" (v0.9 control round): in the vertical controller's flare band (``vc_low_alt``, no ground signature
          yet) the velocity set point is 0 (step 6: no return to the hold point - stop over the ground) and the velocity
          loop keeps its full authority to the contact (no fade); ``vc_ground`` -> a_lim^2 = 0 as in every policy.
          Why: keep_trim fades every correction over the last 0.3 m, so a wind change in the last 1-2 s (the field's wind
          swings / gusts) carried the light vehicle into the contact at 0.5-1.4 m/s (v0.9 criterion: <= 0.3 m/s over the
          ground); the light vehicle's drag time constant is 4.9 s (rocket 9.6 s), the wind moves it twice as fast.
    Physical limit: no rangefinder - the height above the ground is the NAV's pad-relative h, the touchdown itself is
    only seen after the contact.
    No new tilt demand while the vertical loop is saturated high (review v0.8 (4)): vc_sat_high -> a_lim^2 =
    min(a_lim^2, |a_prev|^2) and the integrator frozen.
 5. Guard (spin / upset, prototype + review v0.8 (4)): trigger = |w_z| > spin_guard z threshold, or |w_xy| > the xy
    threshold, or the estimated tilt more than ``TILT_GUARD_EXCESS_DEG`` above tilt_lim (cos(tilt) < cos(lim + 5 deg)
    = cos_lim cos5 - sin_lim sin5); the guard holds for hold_s after the last trigger - so it releases only once the
    vehicle is back within 5 deg of the loop's limit.  While it holds: a = the integrator (trim tilt) only, integrator
    frozen, no new demand.
 6. v_sp: POS: e = hold - p_rf, v_sp = kp e limited to v_lim (v_max_mps; LAND v_max_land; LAND below the flare
    altitude flare_speed_mps); fence: |e|^2 > fence^2 in CLIMB / HOLD -> LAND request R_V_FENCE (never ABORT near
    people).  VEL / DR: v_sp = 0.
 7. D term: dv += a_d ((v - v_prev) / T - dv), a_d = 1 - exp(-2 pi kd_lpf_hz T) (a constant).
 8. Velocity loop (POS / VEL / DR, no guard): ev = v_sp - v; i' = i + ki ev T (not when frozen), limited to i_max;
    a = kv ev + i' - kd dv; conditional integration: if |a|^2 > a_lim^2 and the previous step was limited and
    ev . i > 0 (pushing further into the limit): i' = i.  a limited to a_lim (``limited``).  ATT: a = i limited
    (the trim kept, nothing else).
 9. Plausibility (review v0.8 (7); latched until the motor stops; -> attitude only + LAND request
    R_V_GPS_IMPLAUSIBLE): (a) limited for ``PLAUS_LIMITED_S`` while |v| has not fallen below its value at the start of
    that run - a limited step COUNTS only in CLIMB / HOLD above flare_altitude_m, with the vertical loop not saturated
    high (the binding limit is the profile cap / the thrust-margin authority) and the NAV's GPS-IMU inconsistency flag
    set (``GF_INCONS``: the GPS velocity rejected in >= 3 of the last 5 epochs, ``sim.gps_nav``); any other step ends
    the run (params plausibility_v13_note, architect 18:10 after the v0.8 physics review HIGH 2: at the operator wind
    limit the tilt authority saturates with a healthy GPS - the GPS velocity still agrees with the IMU - and the old
    rule dropped it: false LAND, attitude-only drift 15-35 m, touchdown 3-4 m/s); (b) |v| > v_max +
    ``PLAUS_V_EXCESS_MPS`` for ``PLAUS_V_EXCESS_S``; (c) ``PLAUS_RESETS`` NAV resets within ``PLAUS_RESET_WINDOW_S``.
    No rule latches while the vertical controller sees the ground (``vc_ground``: the touchdown, physics review: (a)
    latched on the ground after 6 of 9 windy landings).  Authority (params authority_check_ASSUMED): k_T T/W <
    min_kT_TW -> LAND request R_V_AUTHORITY (no tilt authority for position hold).
10. Tilt target: rotation vector of the tilt that turns the thrust axis toward (a_x, a_y, g): theta = atan(|a| / g),
    axis (-a_y, a_x) / |a|; series without sqrt: t_W = (-a_y, a_x) (1 / g) (1 - s / 3 + s^2 / 5), s = |a|^2 / g^2
    (atan(x) / x to 0.4 % at 20 deg: x = 0.364, next term x^6 / 7 = 3e-4).  Into the NAV level frame (the attitude
    controller's): t_l = Rz(-psi) t_W = (c t_x + s t_y, -s t_x + c t_y) with the heading cos / sin from the slots.
    v0.8.1: the same series for the trim - the integrator limited to this step's a_lim (sqrt only when it acts) ->
    ``trim_l`` (the glink RESULT / nlink v4 TRIM part: the FCU's target when the GNS results stop).

Tilt setpoint builder (``TiltSetpoint.tick``, every FCU tick at loop_hz 250): t_cur moves toward the latest target by at
most tilt_rate_dps / loop_hz (limit |d|, sqrt only when it acts); the attitude controller of the NEXT tick builds
q_sp = q(t_cur) (x) q_yaw, q(t) = (cos(th / 2), sin(th / 2) t / th) by series (th^2 = |t|^2: cos(th / 2) = 1 - th^2 / 8 +
th^4 / 384, sin(th / 2) / th = 1 / 2 - th^2 / 48 + th^4 / 3840; error < 1e-7 at 20 deg), expanded with the constant
yaw-hold quaternion (c_y, 0, 0, s_y): q_sp = (w c_y, x c_y + y s_y, y c_y - x s_y, w s_y) (``sim.controller``).

Physical limits stated: GPS needs open sky (no position hold indoors); the hold is to within the GPS wander (ASSUMED
Gauss-Markov ~2 m: quote ~1 m RMS / 2.5 m max until a real module is measured); no ground-effect model; heading error
tolerated as described in ``sim.gps_nav``.

Invariant: the tilt target never exceeds the limit of its step; the output is 0 whenever the vehicle is not flying
or not engaged; nothing here changes the phase - LAND requests are acted on by the supervisor only.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from fractions import Fraction

try:  # package import (project root on sys.path) or flat import (sim/ on sys.path)
    from sim import link_proto as lp
    from sim.state_machine import G0
except ImportError:  # pragma: no cover
    import link_proto as lp  # type: ignore
    from state_machine import G0  # type: ignore

# ---- constants (TUNED = v0.8 prototype study; ASSUMED = no source; the 6-DOF agent retunes the gains) ----
KP_POS = 0.9                   # [1/s]       v_sp per metre TUNED (prototype tune2 / tune3)
KV = 2.2                       # [1/s]       m/s^2 per m/s TUNED
KI = 0.35                      # [1/s^2]     TUNED
KD = 0.2                       # [s]         velocity-derivative (acceleration) feedback TUNED
KD_LPF_HZ = 5.0                # low-pass of the differentiated velocity ASSUMED
I_MAX_MPS2 = 3.0               # integrator limit (~17 deg of trim tilt) ASSUMED (prototype)
V_MAX_LAND_MPS = 0.5           # LAND: slower return to the hold point ASSUMED (prototype)
MARGIN_G = 0.10                # vertical authority kept (params tilt_margin_rule)
MARGIN_FRAC = 0.5              # m = min(MARGIN_G, MARGIN_FRAC (k_T T/W - 1)) (params tilt_margin_rule)
TILT_RATE_DPS = 90.0           # tilt-setpoint slew TUNED (prototype; the attitude loop's rate setpoint is clipped at
                               # rate_sp_max_xy 2.5 rad/s = 143 deg/s < the I3 tilt-rate abort 180 deg/s)
H_ENABLE_M = 0.3               # engage above this NAV height ASSUMED (prototype)
TILT_GUARD_EXCESS_DEG = 5.0    # guard while the tilt is more than this above the limit (review v0.8 (4))
TILT_FADE_M = 0.3              # touchdown tilt fade height (review v0.8 (5): "the last ~0.3 m")
PLAUS_LIMITED_S = 3.0          # review v0.8 (7) ~3 s
PLAUS_V_EXCESS_MPS = 1.0       # review v0.8 (7) v_max + 1 m/s
PLAUS_V_EXCESS_S = 2.0         # review v0.8 (7) for 2 s
PLAUS_RESETS = 3               # review v0.8 (7) "more than 2 resets in 10 s"
PLAUS_RESET_WINDOW_S = 10.0
SPIN_Z_DPS = 180.0             # params spin_guard_ASSUMED.rate_dps (prototype spin_guard_dps: |w_z|)
SPIN_XY_DPS = 120.0            # params spin_guard_ASSUMED.release_dps (prototype spin_guard_xy_dps: |w_xy|) - see
                               # the requests: the params name suggests a hysteresis, the prototype's numbers are two axes
GUARD_HOLD_S = 0.5             # params spin_guard_ASSUMED.hold_s

TOUCHDOWN_POLICIES = ("keep_trim", "fade", "keep_vel")   # docstring step 4 (params position_hold.touchdown_policy.policy)
PIPELINE_TICKS = 4             # v0.8 FCU build: J0..J4 over five FCU ticks: outputs published 4 ticks after the inputs
MODE_OFF, MODE_VEL, MODE_POS, MODE_DR, MODE_ATT = 0, 1, 2, 3, 4
MODE_NAMES = {MODE_OFF: "OFF", MODE_VEL: "VEL", MODE_POS: "POS", MODE_DR: "DR", MODE_ATT: "ATT"}
REQ_GPS_LOST = 0x01
REQ_FENCE = 0x02
REQ_IMPLAUSIBLE = 0x04
REQ_AUTHORITY = 0x08
REQ_NAMES = {REQ_GPS_LOST: "gps_lost", REQ_FENCE: "fence", REQ_IMPLAUSIBLE: "gps_implausible", REQ_AUTHORITY: "authority"}
FLY_PHASES = (lp.PHASE_CLIMB, lp.PHASE_HOLD, lp.PHASE_LAND)
GF_FIX_OK = 0x01               # == sim.gps_nav.GF_FIX_OK (not imported: this module stays light)
GF_INCONS = 0x80               # == sim.gps_nav.GF_INCONS: the NAV's GPS velocity disagrees with the IMU-propagated one


def ticks_ge(seconds: float, hz: int) -> int:
    """Smallest n with n / hz >= seconds (``sim.onboard.ticks_ge``; duplicated to keep the import graph acyclic)."""
    x = Fraction(float(seconds)) * int(hz)
    n = math.ceil(x)
    if n - x > 1 - Fraction(1, 10 ** 9):
        n -= 1
    return max(int(n), 0)


@dataclass(frozen=True)
class PositionParams:
    """Every number of the position hold.  ``from_params`` is the constructor the sim and the generator use."""

    loop_hz: int = 50
    fcu_hz: int = 250
    g: float = G0
    tw: float = 1.19                    # model T/W = T_max / (m g) (the controller's mass, GPS included)
    kp_pos: float = KP_POS
    kv: float = KV
    ki: float = KI
    kd: float = KD
    kd_lpf_hz: float = KD_LPF_HZ
    i_max: float = I_MAX_MPS2
    v_max: float = 1.5
    v_max_land: float = V_MAX_LAND_MPS
    flare_speed: float = 0.2
    flare_altitude_m: float = 1.0
    flare_tilt_cap_deg: float = 20.0    # = tilt_cap_deg (docstring 4: no separate flare cap; check() refuses a lower one)
    tilt_cap_deg: float = 20.0
    margin_g: float = MARGIN_G
    margin_frac: float = MARGIN_FRAC
    tilt_rate_dps: float = TILT_RATE_DPS
    h_enable_m: float = H_ENABLE_M
    fence_m: float = 15.0
    spin_z_dps: float = SPIN_Z_DPS
    spin_xy_dps: float = SPIN_XY_DPS
    guard_hold_s: float = GUARD_HOLD_S
    tilt_guard_excess_deg: float = TILT_GUARD_EXCESS_DEG
    gps_timeout_s: float = 1.0
    gps_land_after_s: float = 3.0
    plaus_limited_s: float = PLAUS_LIMITED_S
    plaus_v_excess_mps: float = PLAUS_V_EXCESS_MPS
    plaus_v_excess_s: float = PLAUS_V_EXCESS_S
    plaus_resets: int = PLAUS_RESETS
    plaus_reset_window_s: float = PLAUS_RESET_WINDOW_S
    authority_min_ktw: float = 1.08
    tilt_fade_m: float = TILT_FADE_M
    touchdown_policy: str = "keep_trim"  # params touchdown_policy.policy (v0.8.1); "fade" = the v0.8 law (comparison)
    profile: str = "field"

    # derived (integers / constants the C port precomputes)
    @property
    def T(self) -> float:
        return 1.0 / self.loop_hz

    @property
    def step_ticks(self) -> int:
        return self.fcu_hz // self.loop_hz

    @property
    def timeout_steps(self) -> int:
        return ticks_ge(self.gps_timeout_s, self.loop_hz)

    @property
    def land_steps(self) -> int:
        return ticks_ge(self.gps_timeout_s + self.gps_land_after_s, self.loop_hz)

    @property
    def guard_steps(self) -> int:
        return max(1, ticks_ge(self.guard_hold_s, self.loop_hz))

    @property
    def plaus_limited_steps(self) -> int:
        return ticks_ge(self.plaus_limited_s, self.loop_hz)

    @property
    def plaus_vexc_steps(self) -> int:
        return ticks_ge(self.plaus_v_excess_s, self.loop_hz)

    @property
    def plaus_reset_window_steps(self) -> int:
        return ticks_ge(self.plaus_reset_window_s, self.loop_hz)

    @property
    def alpha_d(self) -> float:
        return 1.0 - math.exp(-2.0 * math.pi * self.kd_lpf_hz * self.T)

    @property
    def cos_cap(self) -> float:
        return math.cos(math.radians(self.tilt_cap_deg))

    @property
    def cos_flare_cap(self) -> float:
        return math.cos(math.radians(min(self.tilt_cap_deg, self.flare_tilt_cap_deg)))

    @property
    def cos_excess(self) -> float:
        return math.cos(math.radians(self.tilt_guard_excess_deg))

    @property
    def sin_excess(self) -> float:
        return math.sin(math.radians(self.tilt_guard_excess_deg))

    @property
    def spin_z_rad(self) -> float:
        return math.radians(self.spin_z_dps)

    @property
    def spin_xy_rad2(self) -> float:
        return math.radians(self.spin_xy_dps) ** 2

    @property
    def slew_per_tick(self) -> float:
        return math.radians(self.tilt_rate_dps) / self.fcu_hz

    def check(self) -> None:
        if self.fcu_hz % self.loop_hz or self.loop_hz <= 0:
            raise ValueError("position_hold.loop_hz must divide control.loop_hz")
        for k in ("kp_pos", "kv", "ki", "kd", "i_max", "v_max", "v_max_land", "flare_speed", "tilt_rate_dps",
                  "fence_m", "guard_hold_s", "gps_timeout_s", "gps_land_after_s"):
            if not getattr(self, k) >= 0:
                raise ValueError(f"position_hold {k} must be >= 0")
        if not self.tilt_fade_m > 0:
            # physics review low: 0 passed the old check and divided by zero at touchdown (float32 C: 0/0 = NaN ->
            # 'na2 > a_lim2' false -> the tilt limit bypassed); params flare_tilt_note: tilt_fade_m > 0 enforced
            raise ValueError("position_hold tilt_fade_m must be > 0")
        if not (0.0 < self.tilt_cap_deg < 60.0 and 0.0 < self.flare_tilt_cap_deg < 60.0):
            raise ValueError("position_hold tilt caps must be in (0, 60) deg")
        if self.flare_tilt_cap_deg < self.tilt_cap_deg:
            # v0.8 physics review HIGH 1 / params flare_tilt_note: a flare cap below the profile cap saturates the loop
            # under the trim needed inside the wind limits (7.0 deg at 5 m/s, 9.35 deg at 6 m/s) for the whole last metre
            raise ValueError(f"position_hold flare tilt cap {self.flare_tilt_cap_deg:g} deg below the profile cap "
                             f"{self.tilt_cap_deg:g} deg (the separate flare cap was removed: params flare_tilt_note)")
        if not self.tw > 0:
            raise ValueError("T/W must be > 0")
        if self.touchdown_policy not in TOUCHDOWN_POLICIES:
            raise ValueError(f"position_hold touchdown_policy {self.touchdown_policy!r} not in {TOUCHDOWN_POLICIES}")

    @classmethod
    def from_params(cls, p: dict, mass_kg: float, t_max_n: float, g: float = G0, profile: str | None = None,
                    **overrides) -> "PositionParams":
        """mission.vtol.position_hold (merged with the build profile, ``sim.onboard.vtol_values``): loop_hz,
        tilt_cap_deg / fence_m by profile name, v_max_mps_ASSUMED, flare_speed_mps_ASSUMED, spin_guard_ASSUMED,
        gps_loss (timeout_s, land_after_s - by profile name when a dict - and ``<profile>_land_after_s``, e.g. the demo's
        demo_land_after_s 0), authority_check_ASSUMED; mission.vtol flare_altitude_m; control.loop_hz; T/W from the
        controller's model mass and T_max.  flare_tilt_cap_deg_ASSUMED was REMOVED from params (flare_tilt_note): the
        flare cap is the profile cap (a key still present is read and check() refuses it below the cap).
        ``gains_ASSUMED`` (kp_pos, kv, ki, kd, i_max) is read when present (the architect's record of the ported gains),
        else the module constants.  Overrides replace fields (tuning)."""
        try:
            from sim.onboard import vtol_values
        except ImportError:  # pragma: no cover
            from onboard import vtol_values  # type: ignore
        v = vtol_values(p, profile)
        ph = v["position_hold"]
        prof = str(v["profile"])

        def by_profile(x, default):
            if isinstance(x, dict):
                return float(x.get(prof, x.get("field", default)))
            return float(default if x is None else x)

        sg = ph.get("spin_guard_ASSUMED", {})
        gl = ph.get("gps_loss", {})
        gn = ph.get("gains_ASSUMED", {}) or {}
        cap = by_profile(ph.get("tilt_cap_deg"), 20.0)
        land_after = by_profile(gl.get("land_after_s"), 3.0)
        if f"{prof}_land_after_s" in gl:                  # e.g. gps_loss.demo_land_after_s (architect 18:10)
            land_after = float(gl[f"{prof}_land_after_s"])
        kw = dict(loop_hz=int(ph.get("loop_hz", 50)), fcu_hz=int(p["control"]["loop_hz"]), g=g,
                  tw=float(t_max_n) / (float(mass_kg) * g),
                  v_max=float(ph.get("v_max_mps_ASSUMED", 1.5)), flare_speed=float(ph.get("flare_speed_mps_ASSUMED", 0.2)),
                  flare_altitude_m=float(v["flare_altitude_m"]),
                  tilt_cap_deg=cap, fence_m=by_profile(ph.get("fence_m"), 15.0),
                  spin_z_dps=float(sg.get("rate_dps", SPIN_Z_DPS)), spin_xy_dps=float(sg.get("release_dps", SPIN_XY_DPS)),
                  guard_hold_s=float(sg.get("hold_s", GUARD_HOLD_S)),
                  gps_timeout_s=float(gl.get("timeout_s", 1.0)), gps_land_after_s=land_after,
                  authority_min_ktw=float(ph.get("authority_check_ASSUMED", {}).get("min_kT_TW", 1.08)),
                  profile=prof)
        for k, name in (("kp_pos", "kp_pos"), ("kv", "kv"), ("ki", "ki"), ("kd", "kd"), ("i_max", "i_max_mps2"),
                        ("kd_lpf_hz", "kd_lpf_hz"), ("tilt_rate_dps", "tilt_rate_dps"), ("v_max_land", "v_max_land_mps")):
            if name in gn:
                kw[k] = float(gn[name])
        for k in ("tilt_fade_m", "tilt_guard_excess_deg", "h_enable_m"):
            if k in ph:
                kw[k] = float(ph[k])
        tdp = ph.get("touchdown_policy")
        if isinstance(tdp, dict) and tdp.get("policy") is not None:
            kw["touchdown_policy"] = str(tdp["policy"])  # v0.8.1 (architect 21:00): "keep_trim"
        kw.update({k: val for k, val in overrides.items() if val is not None})
        if "flare_tilt_cap_deg" not in kw:             # no separate flare cap: the (possibly overridden) profile cap
            kw["flare_tilt_cap_deg"] = (by_profile(ph["flare_tilt_cap_deg_ASSUMED"], kw["tilt_cap_deg"])
                                        if "flare_tilt_cap_deg_ASSUMED" in ph else kw["tilt_cap_deg"])
        pp = cls(**kw)
        pp.check()
        return pp

    def tilt_limit(self, k_t: float, low_cap: bool = False) -> tuple[float, float, float]:
        """(cos_lim, sin_lim, a_lim^2) for the learnt thrust ratio (docstring 4); ``low_cap`` = LAND below the flare
        altitude (the flare cap = the profile cap since the physics-review fix, so both give the same limit; the
        argument stays for the C API ``pos_tilt_limit``).  The C port caches it per (k_T, cap)."""
        x = max(k_t, 1e-3) * self.tw
        m = min(self.margin_g, self.margin_frac * max(x - 1.0, 0.0))
        r = (1.0 + m) / x
        cc = self.cos_flare_cap if low_cap else self.cos_cap
        if r >= 1.0:
            return 1.0, 0.0, 0.0
        c = r if r > cc else cc
        s2 = 1.0 - c * c
        return c, math.sqrt(s2), self.g * self.g * s2 / (c * c)


@dataclass(frozen=True)
class GpsData:
    """The FCU's view of one complete slot set (dequantised; ``sim.nlink_proto.GpsSlotRx`` builds it)."""

    flags: int
    num_sv: int
    hacc_dm: int
    reset_count: int
    off: tuple                     # cumulative reset offset (m)
    p: tuple                       # CG position (m)
    v: tuple                       # CG velocity (m/s)
    cos_psi: float
    sin_psi: float
    cycle: int = 0

    @property
    def fix_ok(self) -> bool:
        return bool(self.flags & GF_FIX_OK)


@dataclass(frozen=True)
class PosInputs:
    phase: int
    motor_on: bool
    h_m: float
    cos_tilt: float                # NAV frame (this tick)
    rate: tuple                    # NAV body rates (this tick), rad/s
    k_t: float                     # vertical controller's learnt thrust ratio (after this tick)
    vc_sat_high: bool = False      # vertical controller sat == +1 in any tick since the previous step
    data: GpsData | None = None    # complete slot set of this cycle; None = hold step
    # ---- physics-review fix (vertical controller outputs of this tick, ``sim.vertical_ctrl.VerticalOutputs``) ----
    vc_low_alt: bool = False       # LAND and h <= flare_altitude_m (its flare band)
    vc_ground: bool = False        # low_alt and a touchdown signature (still / support / floor): on the ground
    lifted: bool = True            # its take-off check passed (True when the check is off)


@dataclass(frozen=True)
class PosOutputs:
    mode: int = MODE_OFF
    tilt_l: tuple = (0.0, 0.0)     # tilt target, rotation vector in the NAV level frame (rad)
    tilt_w: tuple = (0.0, 0.0)     # the same in W
    a_cmd: tuple = (0.0, 0.0)      # horizontal acceleration command (W)
    v_sp: tuple = (0.0, 0.0)
    hold: tuple | None = None      # hold point in the NAV frame (p_rf + offset), None before the capture
    dist_m: float = math.nan       # |hold - p| (NAV), NaN without a hold point
    requests: int = 0              # REQ_* (latched)
    limited: bool = False
    guard: bool = False
    engaged: bool = False
    cos_lim: float = 1.0
    a_lim2: float = 0.0
    lost_steps: int = 0
    integ: tuple = (0.0, 0.0)
    trim_l: tuple = (0.0, 0.0)     # v0.8.1: the trim tilt (integrator limited to a_lim), level frame (glink TRIM)


OFF_OUT = PosOutputs()


class PositionHold:
    """The FCU position loop (module docstring).  ``step`` on the due ticks; ``out`` holds the latest outputs."""

    def __init__(self, pp: PositionParams):
        pp.check()
        self.pp = pp
        self.n = 0                     # step counter
        self.reset()

    def reset(self) -> None:
        self.engaged = False
        self.hold_rf: tuple | None = None
        self.integ = [0.0, 0.0]
        self.dv = [0.0, 0.0]
        self.v_prev: tuple | None = None
        self.a_prev = (0.0, 0.0)
        self.limited_prev = False
        self.lost = 0
        self.guard_left = 0
        self.lim_run = 0
        self.lim_v2_start = 0.0
        self.vexc_run = 0
        self.reset_hist: list[int] = []
        self.last_reset_count: int | None = None
        self.implausible = False
        self.requests = 0
        self.last_data: GpsData | None = None
        self.out = OFF_OUT

    def step(self, inp: PosInputs) -> PosOutputs:
        pp = self.pp
        self.n += 1
        if not (inp.motor_on and inp.phase in FLY_PHASES):
            if self.engaged or self.out is not OFF_OUT:
                self.reset()
            return self.out
        h = inp.h_m
        if not self.engaged:
            if h == h and h > pp.h_enable_m and inp.lifted:
                self.engaged = True
                if inp.data is not None:
                    self.last_reset_count = inp.data.reset_count
            else:
                return self.out
        d = inp.data
        was_lost = self.lost >= pp.timeout_steps        # the GPS had been declared lost (attitude only), not a lost frame
        ok = d is not None and d.fix_ok and not self.implausible
        self.lost = 0 if ok else self.lost + 1
        # guard timer (step 5) advances on every step, hold steps included
        wx, wy, wz = (float(c) for c in inp.rate)
        land = inp.phase == lp.PHASE_LAND
        low = land and h == h and h < pp.flare_altitude_m
        cos_lim, sin_lim, a_lim2 = pp.tilt_limit(inp.k_t, low)
        cos_g = cos_lim * pp.cos_excess - sin_lim * pp.sin_excess       # cos(lim + excess)
        trig = abs(wz) > pp.spin_z_rad or wx * wx + wy * wy > pp.spin_xy_rad2 or inp.cos_tilt < cos_g
        if trig:
            self.guard_left = pp.guard_steps
        elif self.guard_left > 0:
            self.guard_left -= 1
        guard = trig or self.guard_left > 0
        # requests that do not need data
        if self.lost >= pp.land_steps:
            self.requests |= REQ_GPS_LOST
        if inp.k_t * pp.tw < pp.authority_min_ktw:
            self.requests |= REQ_AUTHORITY
        if d is None:                  # hold step: outputs unchanged
            self.out = PosOutputs(**{**self.out.__dict__, "requests": self.requests, "guard": guard,
                                     "lost_steps": self.lost})
            return self.out
        self.last_data = d
        # resets (plausibility (c)); the hold point is in the reset-free frame
        if self.last_reset_count is None:
            self.last_reset_count = d.reset_count
        elif d.reset_count != self.last_reset_count:
            nr = (d.reset_count - self.last_reset_count) & 0xFF
            self.reset_hist += [self.n] * nr
            self.last_reset_count = d.reset_count
        self.reset_hist = [x for x in self.reset_hist if self.n - x < pp.plaus_reset_window_steps]
        ground = inp.vc_ground                          # the vertical controller sees the ground: no rule latches
        if len(self.reset_hist) >= pp.plaus_resets and not ground:
            self.implausible = True
        if self.implausible:
            self.requests |= REQ_IMPLAUSIBLE
            ok = False
        prf = (d.p[0] - d.off[0], d.p[1] - d.off[1])
        if ok and land and was_lost and self.hold_rf is not None:
            self.hold_rf = prf                         # the GPS is back during a LAND: land where it is
        if ok and self.hold_rf is None and inp.phase in (lp.PHASE_HOLD, lp.PHASE_LAND):
            self.hold_rf = prf
        if self.implausible or (not ok and self.lost >= pp.timeout_steps):
            mode = MODE_ATT
        elif not ok:
            mode = MODE_DR
        elif self.hold_rf is not None:
            mode = MODE_POS
        else:
            mode = MODE_VEL
        # limits (step 4): the touchdown rule on the vertical controller's flags
        f_tr = 1.0                                      # keep_trim: the transient's limit factor (1: none)
        fade_band = False                               # v0.9 keep_vel: in the fade band (velocity hold, v_sp = 0)
        if land:
            if ground:
                a_lim2 = 0.0
            elif inp.vc_low_alt and h == h and h < pp.tilt_fade_m:
                if h >= 0.0:
                    f = h / pp.tilt_fade_m
                else:                                   # below the pad level: lower ground - the hold comes back
                    f = min(max(-h - pp.tilt_fade_m, 0.0) / pp.tilt_fade_m, 1.0)
                if pp.touchdown_policy == "fade":
                    a_lim2 *= f * f
                elif pp.touchdown_policy == "keep_trim":
                    f_tr = f
            if pp.touchdown_policy == "keep_vel" and not ground and inp.vc_low_alt:
                fade_band = True                        # v0.9 keep_vel: velocity hold in the flare band (step 6)
        if inp.vc_sat_high:
            ap2 = self.a_prev[0] ** 2 + self.a_prev[1] ** 2
            a_lim2 = min(a_lim2, ap2)
        # velocity set point (step 6)
        vsx = vsy = 0.0
        dist = math.nan
        if self.hold_rf is not None:
            ex, ey = self.hold_rf[0] - prf[0], self.hold_rf[1] - prf[1]
            dist2 = ex * ex + ey * ey
            dist = math.sqrt(dist2)
            if mode == MODE_POS:
                vsx, vsy = pp.kp_pos * ex, pp.kp_pos * ey
                vl = pp.v_max_land if land else pp.v_max
                if low:
                    vl = min(vl, pp.flare_speed)
                n2 = vsx * vsx + vsy * vsy
                if n2 > vl * vl:
                    s = vl / math.sqrt(n2)
                    vsx, vsy = vsx * s, vsy * s
                if inp.phase in (lp.PHASE_CLIMB, lp.PHASE_HOLD) and dist2 > pp.fence_m * pp.fence_m:
                    self.requests |= REQ_FENCE
                if fade_band:
                    vsx = vsy = 0.0                     # v0.9 keep_vel: stop over the ground, no return to the hold point
        # D term (step 7)
        vx, vy = d.v
        if self.v_prev is not None:
            ad = pp.alpha_d
            self.dv[0] += ad * ((vx - self.v_prev[0]) / pp.T - self.dv[0])
            self.dv[1] += ad * ((vy - self.v_prev[1]) / pp.T - self.dv[1])
        self.v_prev = (vx, vy)
        # velocity loop (step 8)
        ix, iy = self.integ
        if mode == MODE_ATT or guard:
            ax, ay = ix, iy
        else:
            evx, evy = vsx - vx, vsy - vy
            if inp.vc_sat_high:
                tx, ty = ix, iy
            else:
                tx, ty = ix + pp.ki * evx * pp.T, iy + pp.ki * evy * pp.T
                ni2 = tx * tx + ty * ty
                if ni2 > pp.i_max * pp.i_max:
                    s = pp.i_max / math.sqrt(ni2)
                    tx, ty = tx * s, ty * s
            ax = pp.kv * evx + tx - pp.kd * self.dv[0]
            ay = pp.kv * evy + ty - pp.kd * self.dv[1]
            if ax * ax + ay * ay > a_lim2 and self.limited_prev and evx * ix + evy * iy > 0.0:
                tx, ty = ix, iy                       # conditional integration: the limit already holds it back
                ax = pp.kv * evx + tx - pp.kd * self.dv[0]
                ay = pp.kv * evy + ty - pp.kd * self.dv[1]
            self.integ = [tx, ty]
            if f_tr < 1.0:                            # keep_trim (step 4): only the transient above the trim is limited
                ex_, ey_ = ax - tx, ay - ty
                le2 = f_tr * f_tr * a_lim2
                ne2 = ex_ * ex_ + ey_ * ey_
                if ne2 > le2:
                    s = math.sqrt(le2 / ne2) if le2 > 0.0 else 0.0
                    ex_, ey_ = ex_ * s, ey_ * s
                ax, ay = tx + ex_, ty + ey_
        na2 = ax * ax + ay * ay
        limited = na2 > a_lim2
        if limited:
            s = math.sqrt(a_lim2 / na2) if a_lim2 > 0.0 else 0.0
            ax, ay = ax * s, ay * s
        # plausibility (step 9): only while the GPS is being flown on
        if mode in (MODE_POS, MODE_VEL):
            v2 = vx * vx + vy * vy
            # (a) counts only a step limited by the profile cap / authority (not the vertical saturation) in CLIMB /
            # HOLD above the flare altitude while the NAV flags its GPS velocity as inconsistent with the IMU
            if (limited and not land and h == h and h > pp.flare_altitude_m and not inp.vc_sat_high
                    and d.flags & GF_INCONS):
                self.lim_run += 1
                if self.lim_run == 1:
                    self.lim_v2_start = v2
                if self.lim_run >= pp.plaus_limited_steps and v2 >= self.lim_v2_start:
                    self.implausible = True
            else:
                self.lim_run = 0
            vt = pp.v_max + pp.plaus_v_excess_mps
            if v2 > vt * vt:
                self.vexc_run += 1
                if self.vexc_run >= pp.plaus_vexc_steps and not ground:
                    self.implausible = True
            else:
                self.vexc_run = 0
            if self.implausible:
                self.requests |= REQ_IMPLAUSIBLE
        else:
            self.lim_run = 0
            self.vexc_run = 0
        # tilt target (step 10)
        g = pp.g
        s = (ax * ax + ay * ay) / (g * g)
        k = (1.0 - s / 3.0 + s * s / 5.0) / g
        twx, twy = -ay * k, ax * k
        c, sn = d.cos_psi, d.sin_psi
        tlx, tly = c * twx + sn * twy, -sn * twx + c * twy
        # v0.8.1 trim (glink TRIM): the integrator limited to this step's a_lim, through the same series and rotation
        rix, riy = self.integ
        ni2 = rix * rix + riy * riy
        if ni2 > a_lim2:
            s = math.sqrt(a_lim2 / ni2) if a_lim2 > 0.0 else 0.0
            rix, riy = rix * s, riy * s
        s = (rix * rix + riy * riy) / (g * g)
        k = (1.0 - s / 3.0 + s * s / 5.0) / g
        rwx, rwy = -riy * k, rix * k
        self.a_prev = (ax, ay)
        self.limited_prev = limited
        hold = None if self.hold_rf is None else (self.hold_rf[0] + d.off[0], self.hold_rf[1] + d.off[1])
        self.out = PosOutputs(mode=mode, tilt_l=(tlx, tly), tilt_w=(twx, twy), a_cmd=(ax, ay), v_sp=(vsx, vsy),
                              hold=hold, dist_m=dist, requests=self.requests, limited=limited, guard=guard,
                              engaged=True, cos_lim=cos_lim, a_lim2=a_lim2, lost_steps=self.lost,
                              integ=(self.integ[0], self.integ[1]),
                              trim_l=(c * rwx + sn * rwy, -sn * rwx + c * rwy))
        return self.out


class TiltSetpoint:
    """The FCU's 250 Hz tilt-setpoint builder: slews t_cur toward the position step's target (level frame)."""

    def __init__(self, pp: PositionParams):
        self.dmax = pp.slew_per_tick
        self.t = (0.0, 0.0)

    def reset(self) -> None:
        self.t = (0.0, 0.0)

    def tick(self, target: tuple, active: bool = True) -> tuple:
        """One FCU tick (after the position step); returns the tilt the NEXT tick's attitude controller uses."""
        if not active:
            self.t = (0.0, 0.0)
            return self.t
        dx, dy = target[0] - self.t[0], target[1] - self.t[1]
        n2 = dx * dx + dy * dy
        if n2 > self.dmax * self.dmax:
            s = self.dmax / math.sqrt(n2)
            dx, dy = dx * s, dy * s
        self.t = (self.t[0] + dx, self.t[1] + dy)
        return self.t


def tilt_quat(tx: float, ty: float) -> tuple[float, float, float, float]:
    """q(t) of a horizontal rotation vector t (level frame) by series (module docstring): (w, x, y, 0)."""
    th2 = tx * tx + ty * ty
    w = 1.0 - th2 / 8.0 + th2 * th2 / 384.0
    k = 0.5 - th2 / 48.0 + th2 * th2 / 3840.0
    return w, k * tx, k * ty, 0.0


def setpoint_quat(tilt: tuple, q_yaw: tuple) -> tuple[float, float, float, float]:
    """q_sp = q(tilt) (x) q_yaw with q_yaw = (c, 0, 0, s): (w c, x c + y s, y c - x s, w s)."""
    w, x, y, _ = tilt_quat(tilt[0], tilt[1])
    c, s = float(q_yaw[0]), float(q_yaw[3])
    return w * c, x * c + y * s, y * c - x * s, w * s


def tilt_of_accel(ax: float, ay: float, g: float = G0) -> tuple[float, float]:
    """The step-10 series in isolation (tests): W rotation vector for a horizontal acceleration command."""
    s = (ax * ax + ay * ay) / (g * g)
    k = (1.0 - s / 3.0 + s * s / 5.0) / g
    return -ay * k, ax * k
