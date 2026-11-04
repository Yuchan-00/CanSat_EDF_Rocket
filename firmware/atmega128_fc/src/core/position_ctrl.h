/* position_ctrl.h - the GPS position hold (v0.8 / v0.8.1, SPEC 8.9 + GPS chapter): a float32 port of sim/position_ctrl.py
 * (PositionHold, TiltSetpoint), C99; the only target-specific part is where the parameters live (below).
 *
 * Where it runs.  v0.8.1 (four ATmega128A, params flight_computer.architecture_v081): on the GNS board (core/gns_node.h),
 * the whole step at once per glink STATE frame (50 Hz), right after the GPS filter's step (pos_step); its outputs go back
 * to the NAV as the glink RESULT (TILT = tilt_l, TRIM = trim_l, STATUS with the mode and the requests, POS = p - hold, VEL)
 * and reach the FCU's tilt-setpoint builder through the nlink v4 slot (core/nlink_proto.h nl_gns_rx_*; the FCU keeps the
 * supervisor rules V11-V15 on the slot data, core/onboard.h).  v0.8 (three chips, the comparison build; host builds and
 * an FCU firmware built with -D ONB_CHAIN_FCU): on the FCU after the ESC write as POS_PIECES background pieces
 * (pos_latch at the due tick k, pieces over five ticks, outputs published at k + POS_PIPELINE_TICKS - sim.onboard.FcuStack
 * chain "fcu"): J0 = k: P0 start, OFF / engage, loss counter, tilt-limit cache (x, m, r, cos_lim), the data-free
 * requests; P1 the rest of the limit (sin, a_lim^2, cos(lim + excess)), the guard, the hold step's outputs.  J1 = k + 1:
 * P2 the NAV reset counter (plausibility (c)), the reset-free position, the hold-point rules, the mode, the touchdown and
 * saturation limits; P3 the distance and v_sp.  J2 = k + 2: P4 the v_sp limit and the fence, P5 the D term, P6 the
 * integrator candidate.  J3 = k + 3: P7 a_cmd and conditional integration, P8 the keep-trim transient limit, the a limit
 * and plausibility (a) / (b).  J4 = k + 4: P9 the tilt series (of a and of the trim), P10 the rotation into the level
 * frame and the COMMIT.  A step that falls due while the previous one is still in its pieces first finishes the
 * previous one; its outputs wait (pos_pipe_t.pend) until their own publication tick.
 *
 * Inputs (pos_inputs_t): the GPS data of this cycle (pos_data_t, physical units: the reset-free position p - offset, the
 * cumulative reset offset and counter, the velocity (W frame: x north, y west), the heading cos / sin, the flags) - the
 * GNS's own filter state at its latch (float, not quantised: sim.glink_proto.gps_data_from) or, in the v0.8 FCU chain,
 * the complete nlink v3 slot set (pos_inputs_t.set: cm, cm/s, Q15 - dequantised by pos_latch exactly as before: the
 * reset-free position is (p - offset) in integer cm times 0.01) - or none (a hold step); phase / motor_on, h, cos(tilt),
 * the body rates, the vertical controller's learnt k_T, its saturation (sat == +1 in any tick since the previous step),
 * its flare band / ground signature and the take-off check.
 *
 * Per step (sim/position_ctrl.py module docstring, the normative text; trig-free): OFF when not flying; engage above
 * h_enable once the vertical controller confirmed the lift-off (lifted); ok = data and FIX_OK and not implausible; lost
 * steps -> mode ATT after timeout_steps, LAND request GPS_LOST at land_steps; DR without data; POS once a hold point
 * exists, VEL before.  Hold point in the reset-free frame p - offset, captured at the first ok step in HOLD / LAND,
 * re-captured when the GPS returns during a LAND after a declared loss.  Tilt limit from the thrust margin (x = k_T T/W,
 * m = min(margin_g, margin_frac max(x - 1, 0)), r = (1 + m) / x, cos_lim = max(r, cos(cap)), a_lim^2 = g^2 (1 - cos_lim^2)
 * / cos_lim^2; cached per (k_T, flare cap)).  Touchdown rule in LAND on the vertical controller's flags (vc_ground: a_lim^2
 * = 0; in its flare band vc_low_alt: f = h / tilt_fade_m for 0 <= h < tilt_fade_m, 0 for -tilt_fade_m <= h < 0, min((-h -
 * tilt_fade_m) / tilt_fade_m, 1) further below - lower ground, the hold comes back), the policy params
 * position_hold.touchdown_policy.policy (pos_params_t.keep_trim):
 *   keep_trim (v0.8.1, architect 21:00): after the velocity loop only the TRANSIENT above the wind trim is limited: e = a -
 *     i (i = the integrator), |e|^2 > f^2 a_lim^2: e *= sqrt(f^2 a_lim^2 / |e|^2) (f = 0: e = 0), a = i + e (always, when
 *     f < 1), then the a limit as before - the vehicle keeps leaning into the wind with its trim to the contact.
 *   fade (v0.8, the comparison build): a_lim^2 *= f^2.
 * No new demand while the vertical loop is saturated high (a_lim^2 <= |a_prev|^2, the integrator frozen); guard (|w_z| >
 * spin_z, |w_xy| > spin_xy or the tilt more than tilt_guard_excess above the limit) holds guard_steps after the last
 * trigger: a = the integrator only.  v_sp = kp e limited to v_max (LAND v_max_land, below the flare altitude flare_speed);
 * fence -> LAND request (never ABORT).  D term on dv (alpha_d), velocity loop with conditional integration, a limited to
 * a_lim.  Plausibility (latched until the motor stops; none latches while vc_ground): (a) limited for
 * plaus_limited_steps while |v| does not fall, counting only in CLIMB / HOLD above flare_altitude_m with the vertical
 * loop not saturated high and POS_GF_INCONS set; (b) |v| > v_max + excess for plaus_vexc_steps; (c) plaus_resets NAV
 * resets within the window.  Authority: k_T T/W < min -> LAND request.  Tilt: t_W = (-a_y, a_x) (1/g) (1 - s/3 + s^2/5),
 * s = |a|^2/g^2; t_l = Rz(-psi) t_W.  Trim (v0.8.1, glink TRIM): the integrator limited to this step's a_lim (sqrt only
 * when it acts) through the same series and rotation -> trim_l (the FCU's target when the GNS results stop).
 * Tilt builder (the FCU, every tick): t_cur moves toward the target by at most slew_per_tick (sqrt only when the limit
 * acts); 0 while not engaged or the motor is off.  attitude_ctrl_set_tilt takes it for the next tick.
 * A gap of m lost STATE frames on the GNS is m hold steps (sim.glink_proto.GnsNode): pos_hold_steps does them in O(1)
 * (one step through the pieces, the other m - 1 from its result - identical, the inputs being the same).
 *
 * float32 against the reference's float64: the reference's operations in its order, except that divisions by
 * parameters are multiplications by their float32 reciprocals (1/T, 1/g, 1/g^2, 1/3, 1/5, 1/tilt_fade_m) and ki T is one
 * factor - each <= 1 ulp more rounding (tools/crosscheck_controller.py bounds the difference).
 *
 * RAM: on the AVR the parameters stay in flash (fc_config.c places the table with the progmem attribute, position_ctrl.c
 * reads it with LPM - inline asm, no AVR header; pos_init takes fc_position_params()) and only the published subset of the
 * outputs is kept (pos_pub_t); host builds copy the parameters and keep the reference's whole PosOutputs (pos_t.out).
 *
 * Physical limits stated (sim/position_ctrl.py): GPS needs open sky (no position hold indoors); the hold is to within the
 * GPS wander (quote ~1 m RMS / 2.5 m max until a real module is measured); no ground-effect model.
 * Invariant: the tilt target never exceeds the limit of its step; the output is 0 whenever the vehicle is not flying or
 * not engaged; nothing here changes the phase - LAND requests are acted on by the FCU's supervisor only.
 */
#ifndef FC_POSITION_CTRL_H
#define FC_POSITION_CTRL_H

#include "fcmath.h"
#include "link_proto.h"
#include "nlink_proto.h"

#if defined(__AVR__)
#define POS_PARAMS_FLASH 1             /* the parameters in flash; no full outputs */
#define POS_FULL_OUT 0
#else
#define POS_PARAMS_FLASH 0
#define POS_FULL_OUT 1
#endif

#define POS_PIPELINE_TICKS 4u
#define POS_PIECES 11u                 /* P0..P10, see above; pos_piece_tick(i) = the tick offset 0..4 of piece i */
#define POS_MODE_OFF 0u
#define POS_MODE_VEL 1u
#define POS_MODE_POS 2u
#define POS_MODE_DR 3u
#define POS_MODE_ATT 4u
#define POS_REQ_GPS_LOST 0x01u
#define POS_REQ_FENCE 0x02u
#define POS_REQ_IMPLAUSIBLE 0x04u
#define POS_REQ_AUTHORITY 0x08u
#define POS_RESETS_MAX 4u              /* plaus_resets <= this (the reset-stamp ring) */
#define POS_GF_FIX_OK 0x01u            /* = GPS_F_FIX_OK (core/gps_nav.h) */
#define POS_GF_INCONS 0x80u            /* = GPS_F_INCONS: the GPS velocity disagrees with the IMU-propagated one */

typedef struct {                       /* sim.position_ctrl.PositionParams (params_gen.h POS_*; float32, steps) */
    uint8_t  step_ticks;               /* control.loop_hz / loop_hz */
    float    T;                        /* 1 / loop_hz [s] */
    float    g, tw;                    /* model T/W = T_max / (m g) */
    float    kp_pos, kv, ki, kd, alpha_d, i_max;
    float    v_max, v_max_land, flare_speed, flare_altitude_m;
    float    cos_cap, cos_flare_cap, cos_excess, sin_excess;
    float    margin_g, margin_frac;
    float    h_enable_m, fence_m;
    float    spin_z_rad, spin_xy_rad2;
    uint16_t guard_steps, timeout_steps, land_steps;
    uint16_t plaus_limited_steps, plaus_vexc_steps, plaus_reset_window_steps;
    uint8_t  plaus_resets;
    float    plaus_vt;                 /* v_max + plaus_v_excess_mps */
    float    authority_min_ktw, tilt_fade_m;
    float    slew_per_tick;            /* radians(tilt_rate_dps) / control.loop_hz */
    uint8_t  keep_trim;                /* touchdown_policy (POS_TD_*): v0.8.1 1 keep_trim, 0 fade (the v0.8 law); v0.9 2
                                        * keep_vel */
    /* derived (pos_params_derive; the AVR flash table has them from the same float32 expressions) */
    float    g2, inv_t, ki_t, i_max2, fence2, plaus_vt2, inv_fade, inv_g, inv_g2;
} pos_params_t;

typedef struct {                       /* sim.position_ctrl.GpsData as the step reads it (physical units, W frame) */
    uint8_t  flags;                    /* GPS_F_* (FIX_OK, INCONS read here) */
    uint8_t  reset_count;              /* the filter's position + velocity reset counter (wraps) */
    float    prf[2];                   /* the reset-free position p - offset [m] */
    float    off[2];                   /* the cumulative reset offset [m] (the hold point's frame) */
    float    v[2];                     /* [m/s] */
    float    cs[2];                    /* cos, sin of the heading psi */
} pos_data_t;

typedef struct {                       /* sim.position_ctrl.PosInputs */
    uint8_t  phase;
    bool     motor_on;
    float    h_m;
    float    cos_tilt;                 /* 1 when none */
    float    rate[3];                  /* body rates, rad/s */
    float    k_t;                      /* the vertical controller's learnt thrust ratio */
    bool     vc_sat_high;              /* sat == +1 in any tick since the previous step */
    const pos_data_t *data;            /* the GPS data of this cycle (GNS); NULL = hold step unless set is given */
    const nl_gps_set_t *set;           /* v0.8 FCU chain: the complete slot set instead (dequantised by pos_latch) */
    bool     vc_low_alt;               /* LAND and h <= flare_altitude_m (VC_SIG_LOW_ALT / NL_PF_LOW_ALT) */
    bool     vc_ground;                /* vc_low_alt and a touchdown signature */
    bool     lifted;                   /* its take-off check passed, true when the check is off */
} pos_inputs_t;

#if POS_FULL_OUT
typedef struct {                       /* sim.position_ctrl.PosOutputs (host builds) */
    uint8_t  mode, requests;
    bool     limited, guard, engaged, has_hold;
    uint16_t lost_steps;
    float    tilt_l[2];                /* tilt target, rotation vector in the NAV level frame (rad) */
    float    tilt_w[2];                /* the same in W */
    float    a_cmd[2], v_sp[2];
    float    hold[2];                  /* hold point in the NAV frame (p_rf + offset); has_hold false = none */
    float    dist_m;                   /* |hold - p|, NaN without a hold point */
    float    cos_lim, a_lim2;
    float    integ[2];
    float    trim_l[2];                /* v0.8.1: the trim tilt (level frame) */
} pos_outputs_t;
#endif

typedef struct {                       /* the published subset (what the FCU chain / the GNS RESULT reads) */
    uint8_t  mode, requests;
    bool     engaged;
    float    tilt_l[2];
    float    dist_m;
    float    trim_l[2];                /* v0.8.1: the trim tilt (glink TRIM) */
    bool     has_hold;                 /* v0.8.1: hold = the hold point in the filter's frame (hold_rf + offset) */
    float    hold[2];
} pos_pub_t;

/* v0.9: position_hold.touchdown_policy.policy codes (pos_params_t.keep_trim; tools/gen_firmware_params.py
 * POS_TD_POLICY_CODES).  keep_vel (sim/position_ctrl.py step 4 / 6): in LAND with vc_low_alt and no ground signature the
 * velocity set point is 0 (after the fence test) and neither the tilt nor the transient is faded */
#define POS_TD_FADE 0u
#define POS_TD_KEEP_TRIM 1u
#define POS_TD_KEEP_VEL 2u

typedef struct {                       /* the step in progress between its pieces */
    /* the latched inputs (pos_latch) */
    uint8_t  phase;
    bool     motor_on, sat_high, has_data, low_alt, ground, lifted;
    float    h;
    pos_data_t d;                      /* the latched GPS data */
    /* the step's work */
    uint8_t  kind;                     /* POS_K_*: full step, hold step (P1 then commit), done (commit only) */
    bool     ok, was_lost, land, low, guard, trig, lim_b, limited, vl_skip, a_ok, fade_band;
    uint8_t  mode;
    float    cos_lim, cos_g, a_lim2, f_tr;
    float    vs[2], dist, a[2];
    union {                            /* values of disjoint piece ranges share their RAM */
        struct { float rate[3], cos_tilt, k_t; } i;      /* latched inputs: pos_latch .. P1 */
        struct { float dist2, vl, n2; } h;               /* P2 .. P4 */
        struct { float ev[2], t[2]; } v;                 /* P6 .. P7 */
        struct { float tw[2], rw[2]; } s;                /* P9 .. P10: the tilt and the trim in W */
    } u;
} pos_work_t;

typedef struct {                       /* sim.position_ctrl.PositionHold */
#if POS_PARAMS_FLASH
    const pos_params_t *p;             /* flash (fc_position_params) */
#else
    pos_params_t p;                    /* a copy, derived values filled */
#endif
    uint16_t n;                        /* step counter (mod 2^16: only differences < plaus_reset_window_steps matter) */
    bool     engaged, has_hold, has_vprev, limited_prev, implausible, has_lrc;
    float    hold_rf[2], integ[2], dv[2], v_prev[2], a_prev[2];
    uint16_t lost, lim_run, vexc_run;
    uint8_t  guard_left;
    float    lim_v2_start;
    uint16_t reset_stamp[POS_RESETS_MAX];   /* the step numbers of the newest NAV resets (newest last) */
    uint8_t  n_stamps;
    uint8_t  last_reset_count;
    uint8_t  requests;                 /* latched until the motor stops */
    pos_pub_t last;                    /* the published subset of the latest outputs (self.out) */
#if POS_FULL_OUT
    pos_outputs_t out;                 /* the latest outputs, whole (self.out) */
#endif
    bool     cache_ok, cache_low;      /* tilt-limit cache: (cos_lim, cos(lim + excess), a_lim^2) per (k_T, cap) */
    float    cache_kt, cache_c, cache_cg, cache_a2;
    uint8_t  job;                      /* the next piece + 1 of the step in progress, 0 = none */
    pos_work_t w;
} pos_t;

typedef struct {                       /* the v0.8 FCU chain's pipeline and publication (FcuStack pos_pending / pos_pub) */
    uint32_t k;                        /* FCU tick counter (the pipeline's clock) */
    uint32_t pub_tick;                 /* publication tick of the step in its pieces */
    bool     pend_valid;               /* outputs of a finished step waiting for their publication tick */
    uint32_t pend_tick;
    pos_pub_t pend;
    pos_pub_t pub;                     /* the published outputs (OFF before the first) */
    uint8_t  off;                      /* tick offset (0..4) of the step in its pieces */
    uint16_t overruns;                 /* a second overlap (cannot happen with a NAV of consecutive frames): published early */
} pos_pipe_t;

/* fill the derived fields from the others (float32; fc_default_position and every other producer of a pos_params_t) */
void pos_params_derive(pos_params_t *p);
/* p: host - copied (derived values recomputed); AVR - the flash table fc_position_params() (kept as a pointer) */
void pos_init(pos_t *s, const pos_params_t *p);
#if POS_PARAMS_FLASH
void pos_params_from_flash(pos_params_t *dst, const pos_params_t *flash);   /* a RAM copy of a flash table */
#endif
void pos_reset(pos_t *s);
/* (cos_lim, sin_lim, a_lim^2) for k_T (sim.position_ctrl.PositionParams.tilt_limit; tests - the step splits the same
 * arithmetic over P0 / P1 and caches it) */
void pos_tilt_limit(const pos_t *s, float k_t, bool low_cap, float *cos_lim, float *sin_lim, float *a_lim2);
/* the v0.8 slot set in the step's units (pos_latch does this for pos_inputs_t.set) */
void pos_data_from_set(pos_data_t *d, const nl_gps_set_t *set);
/* latch the inputs of a step at its due tick (the previous step's pieces must be done: pos_finish) */
void pos_latch(pos_t *s, const pos_inputs_t *in);
/* the tick offset (0..4) a piece belongs to */
uint8_t pos_piece_tick(uint8_t piece);
/* the step in progress has a piece for tick offset off (or earlier) left */
bool pos_piece_due(const pos_t *s, uint8_t off);
/* run the next piece; at the commit piece (P10) the published subset goes to *dst and true is returned */
bool pos_piece(pos_t *s, pos_pub_t *dst);
/* the remaining pieces at once; returns true when a step was committed (into *dst) */
bool pos_finish(pos_t *s, pos_pub_t *dst);
/* the whole step at once (the GNS; tests: the same numbers as the pipeline); returns the published subset */
const pos_pub_t *pos_step(pos_t *s, const pos_inputs_t *in);
/* m >= 1 hold steps (no data) with the same inputs at once (the GNS after m lost STATE frames): identical to m pos_step
 * calls with in->data = in->set = NULL */
const pos_pub_t *pos_hold_steps(pos_t *s, const pos_inputs_t *in, uint16_t m);
void pos_pub_off(pos_pub_t *pub);
uint8_t pos_step_ticks(const pos_t *s);
float   pos_slew_per_tick(const pos_t *s);
uint16_t pos_land_steps(const pos_t *s);
/* the same from a parameter table (fc_position_params(): flash on the AVR) without a pos_t */
float    pos_params_slew_per_tick(const pos_params_t *pp);
uint16_t pos_params_land_steps(const pos_params_t *pp);
uint8_t  pos_params_step_ticks(const pos_params_t *pp);

/* the 250 Hz tilt setpoint builder (sim.position_ctrl.TiltSetpoint): t moves toward target by <= slew per tick; inactive
 * -> 0.  t is the tilt the NEXT tick's attitude controller uses. */
void pos_tilt_tick(float t[2], const float target[2], bool active, float slew_per_tick);
/* the step-10 series in isolation (tests): W rotation vector of a horizontal acceleration command (the float32
 * reciprocals of the step: inv_g = 1 / g, inv_g2 = 1 / g^2) */
void pos_tilt_of_accel(float ax, float ay, float inv_g, float inv_g2, float tw[2]);

#endif /* FC_POSITION_CTRL_H */
