/* vertical_ctrl.h - vertical (altitude / vertical-speed) controller of the VTOL build (SPEC 8.9; v2 = v0.7.1): a
 * tick-for-tick float32 port of sim/vertical_ctrl.py VerticalController (portable C99, no AVR headers).  The altitude
 * reference is an exact int32 in micrometres in both languages; everything else is float32 here, float64 there.
 *
 * Where it sits (sim/onboard.py FcuStack, the VTOL tick order of v0.7.1, params.json mission.vtol.fcu_tick_order_note;
 * onb_stack_step in onboard.h): attitude controller (this tick's NAV frame, the PREVIOUS tick's supervisor outputs and
 * ESC throttle) -> vane registers -> onboard supervisor -> THIS controller -> ESC register.  The supervisor decides the
 * phase, the altitude target, whether the motor may run (motor_on) and whether the baro is lost; this controller
 * computes the throttle u; the ESC gets u only while the supervisor enables the motor.  touchdown / at_target /
 * ground signature / low_in_land_ticks reach the supervisor in the NEXT tick.
 *
 * Units: altitude h, reference h_ref, target [m] above the pad; vertical speed v_z and the commands [m/s], up positive;
 * throttle u [0..1] (T = T_max u^n); specific force f_z [m/s^2] (body z, +g at rest upright); um = micrometre.
 *
 * Mode from the supervisor's phase: CLIMB / HOLD -> FLY, LAND -> LAND, anything else -> IDLE (reset, u = 0).  Per tick
 * (the reference's module docstring, in its order; dt = 1 / loop_hz):
 *   0. first non-IDLE tick (each take-off): ref = m_to_um(h) (0 if h is not finite), integrator 0, ramp counter 0, every
 *      touchdown counter 0, filters v_lpf = d_lpf = 0, thrust state r = 0, u_last = 0, k_T = clamp(1, kt_min, kt_max)
 *   1. reference, exact int32 um: steps up / dn / td = round(rate x 1e6 / loop_hz) (params_gen.h VC_*_UM)
 *      FLY:  tgt = h_tgt_dm x 1e5 (= round_half_away(h_tgt x 10) x 1e5 of the reference); ref + delta, delta = tgt - ref
 *            if -dn <= tgt - ref <= up, else
 *            +up / -dn; at_target = (ref == tgt)
 *      LAND: delta = -td if (ref <= flare_um or h <= flare_altitude_m) and not baro_lost, else -dn (baro lost: the same
 *            law on the NAV's dead-reckoned h, at descent_rate all the way down, no flare)
 *      |ref| <= 2e9 um; dh_ref = delta x loop_hz x 1e-6 [m/s]
 *   2. h_err = alt_kp (h_ref - h);  v_cmd = clamp(ff dh_ref + h_err, -vz_max_down, +vz_max_up), ff = vz_ref_feedforward
 *   3. c = cos(tilt) = 1 - 2 (qx^2 + qy^2) of the NAV quaternion (vc_cos_tilt_from_q: no cosf), 1 if not finite,
 *      clamped to [cos(tilt_abort), 1];  u_ff = u_hover / sqrtf(k_T c) (throttle_exponent 2, k_T of the previous tick;
 *      otherwise the powf fallback u_hover (k_T c)^(-1/n));  e = v_cmd - v_z (0 if not finite);
 *      cap = u_max min(1, (k + 1) / ramp_ticks);  i' = clamp(i + vz_ki dt e, +-vz_i_max), kept unless the output is
 *      saturated in the direction e pushes;  u_raw = u_ff + vz_kp e + i;  u = min(max(u_raw, u_min), cap);
 *      sat = +1 / -1 / 0;  k = min(k + 1, ramp_ticks)
 *   4. touchdown signals (every active tick): r += a_T ((u_last / u_hover)^n - r);  gr = g r;
 *      4b (v0.7.1b, params.json mission.vtol.thrust_learning) learn = kt_alpha > 0 and k == ramp_ticks and f_z finite
 *      and r >= VC_KT_R_MIN and h > flare_altitude_m and c finite and c > kt_cos_min:
 *      k_T += kt_alpha (f_z / gr - k_T), clamped to [kt_min, kt_max] (frozen otherwise);
 *      d_lpf += a_S ((f_z - k_T gr) - d_lpf) (held when f_z is not finite);  v_lpf += a_V (v_z - v_lpf) (held when
 *      v_z is not finite);  lead = h_err <= -still_lead touchdown_rate;  pinned = sat == -1 or (i <= -vz_i_max and
 *      v_cmd <= -vz_max_down);  floor run = pinned ? +1 : 0;  still = |v_lpf| <= td_vz_still and lead;
 *      support = f_z finite and d_lpf > support_g g;  floor = floor run >= floor_ticks;  ground signature = support or
 *      floor;  u_last = u
 *   5. LAND only (v0.7.1b gating, mission.vtol.touchdown.gating_v071b): low_alt = h <= flare_altitude_m;
 *      low = low_alt or v_cmd <= -vz_max_down;  counters still / support / floor +1 if low_alt and the signature, else
 *      0;  cnt[3] (all) +1 if low and support and floor and (still or baro_lost), else 0;  touchdown = any counter >=
 *      landed_confirm_ticks (td_by = the first of still / support / floor / all that confirms);  low_in_land_ticks = +1
 *      if low_alt and lead and not baro_lost, else 0 (the supervisor's V10 LAND floor timeout).  FLY: every counter 0.
 *   6. take-off check (v0.8, architect; params mission.vtol.position_hold.takeoff_check_ASSUMED climb_m, within_s):
 *      h0 is LATCHED at the tick takeoff_settle_ticks after the first tick with the ramp over (h then, 0 if not finite;
 *      physics-review fix: an uncalibrated thrust-proportional baro offset raised the NAV h by ~1 m on the pad and
 *      defeated the check when h0 was h at the start).  passed (latched for the flight) once h0 is latched and h >= h0 +
 *      takeoff_climb_m (the position loop engages only then: core/position_ctrl.h lifted).  Until then, in FLY, a tick
 *      with the take-off ramp over (cap = u_max) and sat == +1 adds 1 to takeoff_run; any other tick sets it to 0.
 *      takeoff_fail (VC_SIG_TAKEOFF_FAIL) = takeoff_run >= takeoff_ticks - the supervisor's V15 (next tick) stops the
 *      motor and goes SAFE.  takeoff_ticks = 0: the check is off (the v0.7.1 controller).  Runs after the throttle (step
 *      3) and before the ramp counter advances, like the reference.
 * No battery-voltage term (k_T absorbs a constant thrust error, the sag at the learning rate); no ground-effect model, no
 * rangefinder.  k_T lives in vc_t (like v_lpf / d_lpf / r_thr); the gating state in vc_outputs_t.sig (VC_SIG_LOW_ALT,
 * VC_SIG_KT_LEARN).
 *
 * float32 vs the float64 reference (tools/crosscheck_controller.py, sections 4c / 4d): the reference ref_um, every
 * reference step, at_target, the flare-band test on ref and every counter are exact integers in both (v0.7.0's
 * knife-edge arrival ticks are gone); h_ref = ref x 1e-6 within 2 ulp; the integrator and the four filters (v_lpf,
 * d_lpf, r, k_T) are the float accumulators (contractive, errors stay a few ulp / alpha); u_ff / e rounding <= 2 ulp.
 * Discrete decisions on floats (lead, still, support, the flare test on h, saturation, conditional integration, the
 * learning's tilt and r guards) may go the other way only when the float64 margin is below the cross-check's
 * knife-edge band.
 *
 * RAM (v0.8.1, the FCU keeps >= 1024 B of stack free): an AVR VTOL build (-D ONB_FIXED_MODE=LP_MODE_VTOL) keeps the
 * parameters in flash - vc_init takes fc_vertical_params() (a flash address, core/fc_config.c) and keeps the pointer,
 * every read goes through FC_FLASH_RD (LPM; core/fcmath.h): 132 B of RAM less.  Host builds and the ballistic FCU copy
 * them (the ballistic image keeps its bytes).
 *
 * Invariant: the output never leaves [0, u_max]; u = 0 exactly when IDLE; nothing here can start the motor (the
 * supervisor's motor_on gates the ESC).
 */
#ifndef FC_VERTICAL_CTRL_H
#define FC_VERTICAL_CTRL_H

#include "fcmath.h"
#include "link_proto.h"
#include "power_mon.h"                 /* v0.9 PM_WIRED (the W_SAG ceiling input) */

#if defined(__AVR__) && defined(ONB_FIXED_MODE)
#if ONB_FIXED_MODE == LP_MODE_VTOL
#define VC_PARAMS_FLASH 1              /* the parameters in flash (fc_vertical_params) */
#endif
#endif
#ifndef VC_PARAMS_FLASH
#define VC_PARAMS_FLASH 0
#endif

#define VC_MODE_IDLE 0u
#define VC_MODE_FLY 1u
#define VC_MODE_LAND 2u

#define VC_TD_NONE 0u                  /* vc_outputs_t.td_by: which signature confirmed the touchdown */
#define VC_TD_STILL 1u
#define VC_TD_SUPPORT 2u
#define VC_TD_FLOOR 3u
#define VC_TD_ALL 4u                   /* v0.7.1b: above the flare band support, floor and still together (cnt[3]) */

#define VC_SIG_LOW 0x01u               /* vc_outputs_t.sig: LAND, h <= flare or the loop saturated downward */
#define VC_SIG_STILL 0x02u             /* signature (a) */
#define VC_SIG_SUPPORT 0x04u           /* signature (b) */
#define VC_SIG_FLOOR 0x08u             /* signature (c) */
#define VC_SIG_GROUND 0x10u            /* support or floor (the supervisor's V8 / V9 baro-lost time limit) */
#define VC_SIG_LOW_ALT 0x20u           /* v0.7.1b: LAND, h <= flare_altitude_m (the band where one signature confirms) */
#define VC_SIG_KT_LEARN 0x40u          /* v0.7.1b: k_T learnt this tick */
#define VC_SIG_TAKEOFF_FAIL 0x80u      /* v0.8: take-off check failed (docstring 6): the supervisor's V15 next tick */

#define VC_UM_PER_DM 100000L
#define VC_REF_LIMIT_UM 2000000000L    /* |ref| bound (sim/vertical_ctrl.py REF_LIMIT_UM) */
#define VC_KT_R_MIN 0.25f              /* learn only while r >= this (sim/vertical_ctrl.py KT_R_MIN_ASSUMED; the division
                                        * guard - tools/test_gen_firmware_params.py checks they are equal) */
#define VC_TAKEOFF_SETTLE_MS 500u      /* take-off check: h0 latched this long after the ramp (sim/vertical_ctrl.py
                                        * TAKEOFF_SETTLE_S_ASSUMED 0.5 s >= baro latency 0.1 s + 1 / (2 pi 0.5 Hz); the sim
                                        * tests check they are equal) */
#define VC_TAKEOFF_SETTLE_TICKS(hz) ((VC_TAKEOFF_SETTLE_MS * (uint32_t)(hz) + 999u) / 1000u)   /* = ticks_ge(0.5 s, hz) */

typedef struct {                       /* sim.vertical_ctrl.VerticalParams, as the FCU uses it (float32; ticks; um) */
    int32_t  up_um, dn_um, td_um;      /* reference steps per tick: round(climb / descent / touchdown rate x 1e6 / hz) */
    int32_t  flare_um;                 /* round(flare_altitude_m x 1e6) */
    float    um_tick_to_mps;           /* loop_hz x 1e-6: dh_ref [m/s] per um of reference step */
    float    flare_altitude_m;
    float    alt_kp;                   /* [1/s]  v_cmd per metre of reference error */
    float    vz_kp;                    /* [s/m]  throttle per m/s of vertical-speed error */
    float    vz_ki_dt;                 /* vz_ki x dt: integrator step per m/s of error */
    float    vz_i_max;                 /* [-]    integrator limit (throttle) */
    float    vz_max_up_mps, vz_max_down_mps;
    float    u_min, u_max;
    float    u_hover, inv_u_hover;     /* (m g / T_max)^(1/n) and its reciprocal */
    int8_t   throttle_exp_int;         /* 2: u_ff = u_hover / sqrtf(k_T c), r = x^2 (params.json edf.throttle_exponent);
                                        * anything else: the powf fallback with throttle_exp */
    float    throttle_exp;             /* n (fallback only; v0.7.1b: -1/n is formed there, not stored - 4 B of RAM) */
    float    cos_tilt_min;             /* cos(tilt_abort_deg): feed-forward floor */
    uint16_t ramp_ticks;               /* mission.throttle_ramp_s: take-off ceiling ramp (v0.7.1b: (k + 1) / ramp_ticks
                                        * is divided during the ramp only - no stored reciprocal, 4 B of RAM) */
    uint16_t landed_confirm_ticks;     /* vtol.landed_confirm_s */
    uint16_t floor_ticks;              /* vtol.touchdown.floor_s_ASSUMED */
    float    td_vz_still_mps;          /* still: |v_lpf| <= this (0.5 touchdown_rate) */
    float    lead_mps;                 /* still / V10: h_err <= -lead_mps (still_lead x touchdown_rate) */
    float    support_mps2;             /* support: d_lpf > support_g x g */
    float    g;
    float    thrust_alpha, still_alpha, support_alpha;   /* 1 - exp(-dt / tau) of the EDF lag, the v_z and f_z low-passes */
    bool     vz_ff;                    /* mission.vtol.vz_ref_feedforward */
    /* v0.7.1b: thrust learning (mission.vtol.thrust_learning) */
    float    kt_alpha;                 /* 1 - exp(-dt / tau_s_ASSUMED); 0 = no learning (k_T stays clamp(1, kt_min, kt_max)) */
    float    kt_min, kt_max;           /* k_min / k_max: the clamp of k_T */
    float    kt_cos_min;               /* cos(max_tilt_deg): learn only while c > this */
    /* v0.8: take-off check (mission.vtol.position_hold.takeoff_check_ASSUMED; docstring 6) */
    float    takeoff_climb_m;          /* climb_m */
    uint16_t takeoff_ticks;            /* ticks_ge(within_s); 0 = off */
    uint16_t takeoff_settle_ticks;     /* h0 latched this many ticks after the ramp (physics-review fix) */
#if FC_V09
    /* v0.9 (sim/vertical_ctrl.py docstrings 1b 1c 3b 4c 5b; params_gen.h VC_TO_* / VC_REF_* / VC_TD_REL_*; all off = the
     * v0.8.1 law, bit for bit) */
    uint16_t ramp_total_ticks;         /* "the ramp is over" = k + 1 >= this (ramp_ticks without the two-slope ramp) */
    uint8_t  v9;                       /* VC_V9_* switches */
    uint8_t  approach_shift;           /* 1b: in FLY the reference step <= max(|d| >> shift, approach_min_um); 0 = off */
    int32_t  approach_min_um;
    float    to_u_start, to_s1, to_s2; /* 3b two-slope ceiling: (k+1) s1 on the pad segment, u_start + (k+1-K1) s2 after */
    float    to_search_f;              /* 3b thrust search: k_T x this per searching tick */
    float    to_search_vz;             /* ... while v_z <= this */
    float    kt_alpha_to;              /* 4c k_T learning rate from the lift-off to the take-off check (0 = off) */
    float    to_airborne_mps2;         /* 4c lift-off: the low-passed specific force above this */
    float    td_step, td_max, td_ok;   /* 5b touchdown release: bias step per tick, limit, 'still' confirms below td_ok */
#endif
} vc_params_t;

#if FC_V09
#define VC_V9_TWO_SLOPE 0x01u          /* 3b two-slope take-off ceiling (takeoff_v09.u_start > 0) */
#define VC_V9_BOOST 0x02u              /* 3b take-off thrust search (boost_per_s > 0 and the take-off check on) */
#define VC_V9_LIFTOFF 0x04u            /* 4c the lift-off latch runs (learn_tau_s > 0 or ref_hold) */
#define VC_V9_REF_HOLD 0x08u           /* 4c the reference waits on the pad for the lift-off */
#define VC_V9_LAND_REBASE 0x10u        /* 1c entering LAND, ref = max(ref, h) */
#define VC_V9_TD_RELEASE 0x20u         /* 5b touchdown thrust release */
#endif

typedef struct {                       /* sim.vertical_ctrl.VerticalInputs (the FCU path) */
    uint8_t phase;                     /* the supervisor's phase of this tick (LP_PHASE_*) */
    int16_t h_tgt_dm;                  /* the supervisor's altitude target [0.1 m] (FLY); the reference's h_tgt_m x 10 */
    bool    baro_lost;                 /* the supervisor: h is the NAV's dead reckoning (no flare, no V10 count) */
    float   h_m, vz_mps;               /* NAV altitude filter */
    float   cos_tilt;                  /* vc_cos_tilt_from_q(NAV q) */
    float   f_z_mps2;                  /* NAV body-z specific force (nlink accel[2]); NaN = none (no support signature) */
#if PM_WIRED
    float   u_cap;                     /* v0.9 W_SAG (3c): the supervisor's frozen power ceiling (cap = min(cap, u_cap)) ... */
    bool    u_capped;                  /* ... while this is set (false = None; a zero-initialised input has no ceiling) */
#endif
} vc_inputs_t;

typedef struct {                       /* sim.vertical_ctrl.VerticalOutputs (vz_lpf, f_excess, thrust_ratio, ref_um, k_thrust:
                                        * vc_t; low_alt, kt_learn: sig) */
    uint8_t  mode;                     /* VC_MODE_* */
    int8_t   sat;                      /* +1 at the ceiling (u_max or the take-off ramp), -1 at u_min, 0 inside */
    bool     at_target;                /* FLY: ref == target this tick */
    bool     touchdown;                /* LAND: touchdown confirmed (the supervisor goes LANDED next tick) */
    uint8_t  sig;                      /* VC_SIG_* of this tick */
    uint8_t  td_by;                    /* VC_TD_* */
    uint16_t td_count;                 /* the largest of the four confirmation counters */
    uint16_t low_in_land_ticks;        /* consecutive LAND ticks low, leading, baro healthy (V10) */
    float    u;                        /* throttle command (0 when IDLE) */
    float    h_ref, vz_cmd, vz_err, u_ff, integ;
    float    dh_ref_mps;               /* the reference rate of this tick (the feed-forward term) */
    float    cos_tilt;                 /* c after the clamp */
} vc_outputs_t;

typedef struct {
#if VC_PARAMS_FLASH
    const vc_params_t *p;              /* flash (fc_vertical_params) */
#else
    vc_params_t p;                     /* a copy */
#endif
#if FC_V09                             /* v0.9: the flags as bits of one byte (-2 B with two new ones) */
    bool     active : 1;
    bool     to_passed : 1;            /* climbed takeoff_climb_m above h0 */
    bool     to_h0_ok : 1;             /* h0 latched */
    bool     airborne : 1;             /* 4c the lift-off was seen (latched for the flight) */
    bool     in_land : 1;              /* 1c the previous tick was LAND */
    float    fz_lpf;                   /* 4c the low-passed specific force until the lift-off (g at the start) */
    float    td_bias;                  /* 5b the touchdown release (<= 0; 0 outside LAND - a LAND -> FLY tick adds the
                                        * leftover once, like the reference; one shared float, the C_port_notes' F5 / F8
                                        * proposal, breaks exactly there) */
#else
    bool     active;
#endif
    int32_t  ref_um;                   /* the exact reference */
    float    integ;
    uint16_t ramp_k;
    uint16_t cnt[4];                   /* still, support, floor, all-three (v0.7.1b) confirmation counters (saturating) */
    uint16_t floor_run, low_in_land;
    float    v_lpf, d_lpf, r_thr, u_last;
    float    k_t;                      /* v0.7.1b: the learnt thrust ratio T_actual / T_model (1 at each take-off) */
    float    to_h0;                    /* v0.8 take-off check: h latched after the ramp + the settle time (0 if not finite) */
    uint16_t to_run;                   /* consecutive saturated ticks without the climb (saturating) */
#if !FC_V09
    bool     to_passed;                /* climbed takeoff_climb_m above h0 */
    bool     to_h0_ok;                 /* h0 latched */
#endif
    uint16_t to_settle;                /* ticks since the ramp ended (up to takeoff_settle_ticks) */
} vc_t;

#if FC_V09
/* v0.9: the take-off check passed / the lift-off latch of the latest tick (the supervisor's power rules read them the
 * next tick: sim.onboard FcuStack vc_takeoff_passed / vc_airborne) */
static inline bool vc_takeoff_passed(const vc_t *c) { return c->to_passed; }
static inline bool vc_airborne(const vc_t *c) { return c->airborne; }
#endif

void    vc_init(vc_t *c, const vc_params_t *p);    /* AVR VTOL: p = fc_vertical_params() (flash, kept as a pointer) */
uint16_t vc_takeoff_ticks(const vc_t *c);          /* the take-off check's tick count (0 = off) */
void    vc_reset(vc_t *c);
void    vc_update(vc_t *c, const vc_inputs_t *in, vc_outputs_t *out);
uint8_t vc_mode_of_phase(uint8_t phase);
/* metres -> integer micrometres, round half away from zero of the EXACT product x 1e6 (integer arithmetic on the float32
 * bit pattern, as Python's float64 product of a float32 is exact), saturated to +-VC_REF_LIMIT_UM, NaN -> 0 */
int32_t vc_m_to_um(float x);
/* cos(tilt) of a unit quaternion (w, x, y, z): 1 - 2 (x^2 + y^2) */
float   vc_cos_tilt_from_q(const float q[4]);

#endif /* FC_VERTICAL_CTRL_H */
