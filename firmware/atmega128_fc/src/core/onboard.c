/* onboard.c - see onboard.h (mirrors sim/onboard.py OnboardSupervisor branch by branch, both builds, and
 * FcuStack).  The ballistic functions are the v0.6.3 code unchanged except for the mode dispatch; the VTOL ones
 * carry each rule's rationale (the reference's module docstring, SPEC 8.9) next to it. */
#include "onboard.h"

#include "link_proto_v09.h"   /* v0.9 RP1: onboard_telemetry_frame of a power build (TELEM v4 / v5) */

#define LFSR_TAPS 0xB400u
#define LFSR_ZERO_SEED 0xACE1u

/* ONB_FIXED_MODE: a firmware build may pin the mission mode at compile time (-D ONB_FIXED_MODE=LP_MODE_VTOL or
 * LP_MODE_BALLISTIC), so the code of the other mode is dropped; onboard_init then forces p.mode to it.  Without it
 * (host tests, the cross-check) the mode is onb_params_t.mode. */
#if defined(ONB_FIXED_MODE)
#define IS_VTOL(s) (ONB_FIXED_MODE == LP_MODE_VTOL)
#else
#define IS_VTOL(s) ((s)->p.mode == LP_MODE_VTOL)
#endif

/* v0.8.1 RAM diet: OPAR(s) = the supervisor's parameter table, PRM(field) = one read of it (an LPM in a flash build) */
#if ONB_PARAMS_FLASH
#define OPAR(s) ((s)->p)
#define PRM(x) FC_FLASH_RD(x)
#else
#define OPAR(s) (&(s)->p)
#define PRM(x) (x)
#endif

static bool flight(uint8_t s)
{
    return s == ONB_POWERED || s == ONB_UNPOWERED;
}

static bool motor_enabled(uint8_t s)
{
    return s == ONB_ARMED || flight(s);
}

bool onboard_flying(uint8_t state)
{
    return flight(state) || state == ONB_ABORT || state == ONB_RELEASED;
}

bool onboard_flying_mode(uint8_t mode, uint8_t state)
{
    if (mode == LP_MODE_VTOL)                         /* UNPOWERED = LANDED: on the ground, motor off */
        return state == ONB_POWERED || state == ONB_ABORT || state == ONB_RELEASED;
    return onboard_flying(state);
}

bool onboard_vtol(const onboard_t *s)
{
    return IS_VTOL(s);
}

uint16_t onboard_loop_hz(const onboard_t *s)
{
    return PRM(OPAR(s)->loop_hz);
}

uint16_t onb_lfsr16_next(uint16_t s)
{
    uint16_t lsb;
    if (s == 0u)
        s = LFSR_ZERO_SEED;
    lsb = (uint16_t)(s & 1u);
    s >>= 1;
    if (lsb)
        s ^= LFSR_TAPS;
    return s;
}

void onboard_init(onboard_t *s, const onb_params_t *p, uint16_t seed, bool reset_in_flight)
{
#if ONB_PARAMS_FLASH
    s->p = p;                                         /* the flash table (its mode is the fixed VTOL build's) */
#else
    s->p = *p;
#if defined(ONB_FIXED_MODE)
    s->p.mode = ONB_FIXED_MODE;
#endif
#endif
    s->state = ONB_SAFE;
    s->t_entry = 0;
    s->nonce = onb_lfsr16_next(seed);
    s->has_seq = false;
    s->last_seq = 0;
    s->ack_rejected = false;
    s->has_frame = false;
    s->last_frame_tick = 0;
    s->reason = LP_REASON_NONE;
    s->interlock_latched = false;
#if ONB_HAS_BALLISTIC
    s->target_milli = 0;
    s->out_milli = 0;
    s->level = 0;
    s->has_unpowered = false;
    s->t_unpowered = 0;
#endif
    s->powered_ticks = 0;
    s->release_open = false;
    s->has_release_deadline = false;
    s->release_deadline = 0;
    s->rate_count = s->fall_count = s->imu_fail = s->baro_fail = 0;
    s->phase = LP_PHASE_NONE;
    s->tgt_dm = 0;
    s->baro_lost = false;
    s->baro_lost_tick = 0;
    s->baro_limit = 0;
    s->out = 0;
#if PM_ANY
    pm_init(&s->pm);                                  /* v0.9 power monitor */
#if PM_BUF
    s->lost_noted = false;
#endif
#endif
    if (reset_in_flight) {
        s->reason = LP_R_I10_RESET;
        s->interlock_latched = true;
        if (IS_VTOL(s)) {                             /* I10 VTOL: ABORT (motor off), release after abort_release_ticks */
            s->state = ONB_ABORT;                     /* (the in-flight state is lost: the motor must not restart) */
            s->has_release_deadline = true;
            s->release_deadline = PRM(OPAR(s)->abort_release_ticks);
        } else {
            s->release_open = true;                   /* I10 ballistic: SAFE with the release open, arming refused */
        }
    }
}

/* ---- helpers ------------------------------------------------------------------------- */
static uint32_t ticks_since_frame(const onboard_t *s, uint32_t t)
{
    return s->has_frame ? t - s->last_frame_tick : ONB_NO_FRAME_TICKS;
}

bool onboard_link_ok(const onboard_t *s, uint32_t tick)
{
    return ticks_since_frame(s, tick) < PRM(OPAR(s)->link_disarm_ticks);
}

bool onboard_control_active(const onboard_t *s, uint32_t tick)
{
    if (s->state == ONB_POWERED)
        return true;
    if (IS_VTOL(s))                                   /* VTOL: attitude control in POWERED only (LANDED: vanes centred) */
        return false;
#if ONB_HAS_BALLISTIC
    return s->state == ONB_UNPOWERED && s->has_unpowered && tick - s->t_unpowered < PRM(OPAR(s)->coast_hold_ticks);
#else
    (void)tick;
    return false;
#endif
}

static void log_trans(onboard_t *s, uint8_t to, uint8_t phase_to, uint8_t cause)
{
    onb_outputs_t *o = s->out;
    if (o && o->n_trans < ONB_MAX_TRANS) {
        onb_trans_t *tr = &o->trans[o->n_trans++];
        tr->from = s->state;
        tr->to = to;
        tr->cause = cause;
        tr->phase_from = s->phase;
        tr->phase_to = phase_to;
    }
}

static void enter(onboard_t *s, uint8_t new_state, uint32_t t, uint8_t cause)
{
    /* VTOL: the phase each state starts in (POWERED is entered from ARMED only, so it starts in CLIMB) */
    const uint8_t ph = !IS_VTOL(s) ? LP_PHASE_NONE : new_state == ONB_ARMED ? LP_PHASE_SPOOL :
                       new_state == ONB_POWERED ? LP_PHASE_CLIMB : new_state == ONB_UNPOWERED ? LP_PHASE_LANDED :
                       LP_PHASE_NONE;
    log_trans(s, new_state, ph, cause);
    s->state = new_state;
    s->t_entry = t;
#if ONB_HAS_BALLISTIC
    if (new_state != ONB_POWERED)
        s->target_milli = 0;
    if (new_state == ONB_UNPOWERED) {
        s->has_unpowered = true;
        s->t_unpowered = t;
    }
#endif
    if (new_state == ONB_SAFE)
        s->has_release_deadline = false;              /* a release pending on the pad is cancelled */
    s->phase = ph;
}

#if ONB_HAS_VTOL
static void set_phase(onboard_t *s, uint8_t new_phase, uint8_t cause)   /* VTOL, inside POWERED */
{
    log_trans(s, s->state, new_phase, cause);
    s->phase = new_phase;
}
#endif

static void interlock(onboard_t *s, uint8_t code)
{
    s->reason = code;
    s->interlock_latched = true;
    if (s->out)
        s->out->interlock = code;
}

static void do_abort(onboard_t *s, uint32_t t, uint8_t cause)
{
    uint32_t d = t + PRM(OPAR(s)->abort_release_ticks);
    if (!s->has_release_deadline || d < s->release_deadline)
        s->release_deadline = d;                      /* keeps the earlier of a pending release and the abort delay */
    s->has_release_deadline = true;
    enter(s, ONB_ABORT, t, cause);
}

#if ONB_HAS_BALLISTIC
static void start_release(onboard_t *s, uint32_t t, uint8_t cause)
{
    s->release_deadline = t + PRM(OPAR(s)->release_after_cut_ticks);
    s->has_release_deadline = true;
    s->target_milli = 0;
    if (s->state == ONB_POWERED)
        enter(s, ONB_UNPOWERED, t, cause);
}
#endif

static void kill(onboard_t *s, uint32_t t, uint8_t cause)
{
    if (s->state == ONB_ARMED)
        enter(s, ONB_SAFE, t, cause);
    else if (flight(s->state))
        do_abort(s, t, cause);
}

#if ONB_HAS_VTOL
static bool fly_phase(uint8_t phase)                  /* VTOL CLIMB / HOLD: UP / DOWN and the LAND rules apply */
{
    return phase == LP_PHASE_CLIMB || phase == LP_PHASE_HOLD;
}

/* VTOL altitude target as reported (TELEM v2) and given to the vertical controller: the target in SPOOL / CLIMB /
 * HOLD, 0 otherwise (LAND descends to the ground, which may lie below the pad) */
static int16_t h_tgt_dm(const onboard_t *s)
{
    return (s->phase == LP_PHASE_SPOOL || fly_phase(s->phase)) ? s->tgt_dm : 0;
}
#endif

/* ---- commands ---------------------------------------------------------------------------- */
/* the ARM gates (ballistic ARM, VTOL ON) in their order: 0 or the LP_REJ_* code of the first that fails */
#if FC_VTOL_RAM_TRIM
/* fw09/p1: kept inline in the VTOL FCU image as LTO did in v0.8.1 - the P1 code-size changes (table CRC, lp_quantize)
 * moved the inliner's decision and the call frame cost the deepest stack path 5 B (run_simbench.py: 515 -> 520 B) */
static inline __attribute__((always_inline)) uint8_t arm_gates(const onboard_t *s, const onb_inputs_t *in,
                                                               bool link_ok_before);
#endif
static uint8_t arm_gates(const onboard_t *s, const onb_inputs_t *in, bool link_ok_before)
{
    const onb_params_t *p = OPAR(s);
    if (s->release_open)
        return LP_REJ_RELEASED;
    if (!in->arm_switch)
        return LP_REJ_ARM_SWITCH;
    if (!(in->imu_ok && in->baro_ok && s->imu_fail == 0u && s->baro_fail == 0u))
        return LP_REJ_SENSORS;
    if (!in->aligned)
        return LP_REJ_NOT_ALIGNED;
#if ONB_HAS_VTOL
    if (IS_VTOL(s) && PRM(p->gps_hold) && !in->gps_gate_ok)   /* v0.8 VTOL ON: the GPS must be ready (position hold needs it */
        return LP_REJ_GPS;                            /* from the first second of the climb; open sky only) */
#endif
    if (!(in->tilt_deg < PRM(p->arm_tilt_max_deg)))
        return LP_REJ_TILT;
#if PM_ANY
    if (IS_VTOL(s) && PM_BUILD(&p->pw) != PWR_SRC_BATTERY) {   /* v0.9 W_ARM: the supply / buffer at idle */
        if (!pm_arm_ok(&s->pm, &p->pw))
            return LP_REJ_POWER;
    } else
#endif
    if (!(in->v_cell >= PRM(p->min_cell_v)))
        return LP_REJ_VCELL;
    if (!link_ok_before)
        return LP_REJ_LINK;
    return 0;
}

#if ONB_HAS_BALLISTIC
static uint8_t execute_ballistic(onboard_t *s, const onb_frame_t *f, uint32_t t, const onb_inputs_t *in,
                                 bool link_ok_before)
{
    const uint8_t c = f->cmd.cmd, st = s->state, cause = (uint8_t)(ONB_CAUSE_CMD | (c & 0x7Fu));
    const onb_params_t *p = OPAR(s);
    if (c > LP_CMD_MAX_BALLISTIC || c == LP_CMD_NONE)
        return LP_REJ_UNKNOWN_CMD;                    /* 8..11 (VTOL codes) included, exactly as v0.6.3 */
    if (c == LP_CMD_KILL) {
        if (st == ONB_SAFE)
            return LP_REJ_STATE;
        if (st == ONB_ARMED || flight(st))
            kill(s, t, cause);
        return 0;                                     /* ABORT / RELEASED: already safe */
    }
    if (s->has_release_deadline)
        return LP_REJ_RELEASE_PENDING;
    switch (c) {
    case LP_CMD_ARM: {
        uint8_t code;
        if (st != ONB_SAFE)
            return LP_REJ_STATE;
        code = arm_gates(s, in, link_ok_before);
        if (code)
            return code;
        enter(s, ONB_ARMED, t, cause);
        return 0;
    }
    case LP_CMD_DISARM:
        if (st != ONB_ARMED)
            return LP_REJ_STATE;
        enter(s, ONB_SAFE, t, cause);
        return 0;
    case LP_CMD_LAUNCH:
        if (st != ONB_ARMED)
            return LP_REJ_STATE;
        if (f->cmd.nonce != s->nonce)
            return LP_REJ_NONCE;
        s->nonce = onb_lfsr16_next(s->nonce);
        enter(s, ONB_POWERED, t, cause);
        s->target_milli = PRM(p->climb_milli);
        return 0;
    case LP_CMD_THROTTLE: {
        const uint16_t u = f->cmd.arg < PRM(p->climb_milli) ? f->cmd.arg : PRM(p->climb_milli);
        if (st == ONB_ARMED) {
            if (f->cmd.nonce != s->nonce)
                return LP_REJ_NONCE;
            if (u == 0u)
                return LP_REJ_THROTTLE_ZERO;
            s->nonce = onb_lfsr16_next(s->nonce);
            enter(s, ONB_POWERED, t, cause);
            s->target_milli = u;
            return 0;
        }
        if (st == ONB_POWERED) {
            if (u == 0u)
                enter(s, ONB_UNPOWERED, t, cause);
            else
                s->target_milli = u;
            return 0;
        }
        if (st == ONB_UNPOWERED) {
            if (u > 0u) {
                enter(s, ONB_POWERED, t, cause);
                s->target_milli = u;
            }
            return 0;
        }
        return LP_REJ_STATE;
    }
    case LP_CMD_CUT:
        if (st != ONB_POWERED)
            return LP_REJ_STATE;
        enter(s, ONB_UNPOWERED, t, cause);
        return 0;
    case LP_CMD_RELEASE:
        if (!(st == ONB_ARMED || flight(st)))
            return LP_REJ_STATE;
        start_release(s, t, cause);
        return 0;
    default:
        return LP_REJ_UNKNOWN_CMD;
    }
}

#endif

#if ONB_HAS_VTOL
/* VTOL command table (onboard.h; sim/onboard.py _execute_vtol) */
static uint8_t execute_vtol(onboard_t *s, const onb_frame_t *f, uint32_t t, const onb_inputs_t *in,
                            bool link_ok_before)
{
    const uint8_t c = f->cmd.cmd, st = s->state, cause = (uint8_t)(ONB_CAUSE_CMD | (c & 0x7Fu));
    const onb_params_t *p = OPAR(s);
    if (c > LP_CMD_MAX || c == LP_CMD_NONE)
        return LP_REJ_UNKNOWN_CMD;
    if (c == LP_CMD_KILL) {                           /* the operator's emergency stop, as in the ballistic build */
        if (st == ONB_SAFE)
            return LP_REJ_STATE;
        if (st == ONB_ARMED || flight(st))
            kill(s, t, cause);
        return 0;                                     /* ABORT / RELEASED: already safe */
    }
    if (c < LP_CMD_KILL)                              /* ARM / DISARM / LAUNCH / THROTTLE / CUT / RELEASE */
        return LP_REJ_MODE;
    if (s->has_release_deadline)                      /* a release is pending (ABORT): only KILL */
        return LP_REJ_RELEASE_PENDING;
    if (c == LP_CMD_ON) {
        uint8_t code;
        if (st != ONB_SAFE)
            return LP_REJ_STATE;
        if (f->cmd.nonce != s->nonce)                 /* the operator saw this vehicle's latest telemetry */
            return LP_REJ_NONCE;
        code = arm_gates(s, in, link_ok_before);      /* every ARM gate of the ballistic build */
        if (code)
            return code;
        if (s->powered_ticks >= PRM(p->endurance_ticks))   /* the pack's flight time is used up: no take-off it cannot */
            return LP_REJ_ENDURANCE;                  /* land from (power-cycle with a charged pack) */
        s->nonce = onb_lfsr16_next(s->nonce);
        s->tgt_dm = PRM(p->hover_dm);
        s->baro_lost = false;
#if PM_BUF
        s->lost_noted = false;
#endif
        enter(s, ONB_ARMED, t, cause);                /* SPOOL: buzzer warning, motor off */
        return 0;
    }
    if (c == LP_CMD_OFF) {
        if (st == ONB_ARMED) {                        /* spool-up cancelled */
            enter(s, ONB_SAFE, t, cause);
            return 0;
        }
        if (st == ONB_POWERED && fly_phase(s->phase)) {
            set_phase(s, LP_PHASE_LAND, cause);       /* land where it is */
            return 0;
        }
        if (flight(st))                               /* LAND / LANDED: already landing - acknowledged, no change */
            return 0;
        return LP_REJ_STATE;
    }
    {                                                 /* UP / DOWN: exact integer decimetres */
        int32_t nw;
        if (st != ONB_POWERED)
            return LP_REJ_STATE;
        if (!fly_phase(s->phase))                     /* a landing is never cancelled */
            return LP_REJ_LANDING;
#if PM_WIRED
        if (c == LP_CMD_UP && PM_BUILD(&p->pw) == PWR_SRC_WIRED && pm_sag_warn(&s->pm))
            return LP_REJ_POWER;                      /* v0.9 W_SAG stage 1: no more power demand */
#endif
        nw = (int32_t)s->tgt_dm + (c == LP_CMD_UP ? (int32_t)PRM(p->alt_step_dm) : -(int32_t)PRM(p->alt_step_dm));
        if (!((int32_t)PRM(p->alt_min_dm) <= nw && nw <= (int32_t)PRM(p->alt_max_dm)))
            return LP_REJ_ALT_LIMIT;
        s->tgt_dm = (int16_t)nw;
        return 0;
    }
}

#endif

#if ONB_HAS_BALLISTIC
/* ---- interlocks (ballistic, SPEC 8.8) --------------------------------------------------------- */
static void interlocks_ballistic(onboard_t *s, uint32_t t, const onb_inputs_t *in)
{
    const onb_params_t *p = OPAR(s);
    const uint8_t st = s->state;
    bool on_pad;
    uint32_t since;
    /* persistence counters on this tick's inputs (state after the commands) */
    if (st == ONB_POWERED) {
        const float wx = in->gyro[0], wy = in->gyro[1], wz = in->gyro[2];
        const bool bad = (PRM(p->gyro_saturation_abort) && in->gyro_saturated) || (wx * wx + wy * wy > PRM(p->tilt_rate_abort_sq)) ||
                         fabsf(wz) > PRM(p->spin_rate_abort);
        s->rate_count = bad ? s->rate_count + 1u : 0u;
    } else {
        s->rate_count = 0;
    }
    if (flight(st) && in->vz_mps <= PRM(p->fall_vz_mps))
        s->fall_count++;
    else
        s->fall_count = 0;
    if (!motor_enabled(st))
        return;
    on_pad = st == ONB_ARMED;
    if (!in->arm_switch) {                            /* arm switch opened: acts as KILL */
        interlock(s, LP_R_ARM_SWITCH_OPEN);
        kill(s, t, LP_R_ARM_SWITCH_OPEN);
        return;
    }
    if (s->imu_fail >= PRM(p->imu_fail_ticks) || s->baro_fail >= PRM(p->baro_fail_ticks)) {   /* I9 */
        const uint8_t code = s->imu_fail >= PRM(p->imu_fail_ticks) ? LP_R_I9_IMU : LP_R_I9_BARO;
        interlock(s, code);
        if (on_pad)
            enter(s, ONB_SAFE, t, code);
        else
            do_abort(s, t, code);
        return;
    }
    since = ticks_since_frame(s, t);
    if (on_pad) {                                     /* I1p */
        if (since >= PRM(p->link_disarm_ticks)) {
            interlock(s, LP_R_I1P_LINK_LOSS_PAD);
            enter(s, ONB_SAFE, t, LP_R_I1P_LINK_LOSS_PAD);
        }
        return;
    }
    if (since >= PRM(p->link_abort_ticks)) {               /* I1 */
        interlock(s, LP_R_I1_LINK_LOSS);
        do_abort(s, t, LP_R_I1_LINK_LOSS);
        return;
    }
    if (st == ONB_POWERED) {
        if (in->tilt_deg > PRM(p->tilt_abort_deg)) {       /* I2 */
            interlock(s, LP_R_I2_TILT);
            do_abort(s, t, LP_R_I2_TILT);
            return;
        }
        if (s->rate_count >= PRM(p->rate_persist_ticks)) {  /* I3 */
            interlock(s, LP_R_I3_RATE);
            do_abort(s, t, LP_R_I3_RATE);
            return;
        }
        if (in->v_cell < PRM(p->min_cell_v)) {             /* I8 */
            interlock(s, LP_R_I8_VCELL);
            do_abort(s, t, LP_R_I8_VCELL);
            return;
        }
    }
    if (in->h_m > PRM(p->geofence_m)) {                    /* I5 */
        interlock(s, LP_R_I5_GEOFENCE);
        do_abort(s, t, LP_R_I5_GEOFENCE);
        return;
    }
    if (st == ONB_POWERED) {
        const float v = in->vz_mps > 0.0f ? in->vz_mps : 0.0f;
        const float pred = in->h_m + PRM(p->apogee_k) * v * v;
        if (pred >= PRM(p->apogee_limit_m)) {              /* I4: CUT only */
            interlock(s, LP_R_I4_APOGEE_LIMIT);
            enter(s, ONB_UNPOWERED, t, LP_R_I4_APOGEE_LIMIT);
            return;
        }
        if (s->powered_ticks >= PRM(p->max_motor_ticks)) {  /* I7: CUT */
            interlock(s, LP_R_I7_MOTOR_TIME);
            enter(s, ONB_UNPOWERED, t, LP_R_I7_MOTOR_TIME);
            return;
        }
    }
    if (!s->release_open && !s->has_release_deadline && s->fall_count >= PRM(p->fall_confirm_ticks)) {   /* I6 */
        interlock(s, LP_R_I6_FALL);
        start_release(s, t, LP_R_I6_FALL);
    }
}

#endif

#if ONB_HAS_VTOL
/* ---- safety rules (VTOL, SPEC 8.9): in this order, the first that acts ends the evaluation ----------- */
static void land(onboard_t *s, uint8_t code)          /* a LAND rule acted (CLIMB / HOLD): latch the reason, phase LAND */
{
    interlock(s, code);
    set_phase(s, LP_PHASE_LAND, code);
}

uint32_t onboard_baro_limit_ticks(const onb_params_t *p, float h_m)
{
    int32_t h_dm;
    if (h_m - h_m == 0.0f)                            /* finite: round_half_away(h x 10), i16 (exact, lp_quantize) */
        h_dm = lp_quantize(h_m, LP_H_SCALE, LP_I16_MIN, LP_I16_MAX);
    else                                              /* the altitude is unknown: the longest limit */
        h_dm = PRM(p->v_geofence_dm);
    if (h_dm < 0)                                     /* min(max(h_dm, 0), geofence_dm) */
        h_dm = 0;
    if (h_dm > PRM(p->v_geofence_dm))
        h_dm = PRM(p->v_geofence_dm);
    {
        const uint32_t x = (uint32_t)h_dm * PRM(p->baro_limit_mt_per_dm);   /* <= geofence_dm x mt: checked to fit (generator) */
        return PRM(p->baro_limit_base_ticks) + x / 1000u + (x % 1000u != 0u ? 1u : 0u);
    }
}

#if PM_WIRED
/* v0.9 OnboardOutputs.u_cap: the W_SAG ceiling is on (POWERED, the wired build, the warning set) */
bool onboard_u_capped(const onboard_t *s)
{
    return IS_VTOL(s) && PM_BUILD(&OPAR(s)->pw) == PWR_SRC_WIRED && pm_sag_warn(&s->pm) && s->state == ONB_POWERED;
}
#endif

#if PM_ANY
/* v0.9 power rules (sim/onboard.py _power_act, fix round F1): LAND - or, ON THE PAD (take-off check not passed, the
 * support signature present, no lift-off seen; previous tick), motor off -> SAFE like V15, no release */
static void power_act(onboard_t *s, uint32_t t, const onb_inputs_t *in, uint8_t code)
{
    if (in->vc_takeoff_passed || !in->vc_support || in->vc_airborne) {
        land(s, code);
    } else {
        interlock(s, code);
        enter(s, ONB_SAFE, t, code);
    }
}

/* the power build's LAND rules in the I8 slot (POWERED CLIMB / HOLD); true when one acted (_power_land_rules) */
static bool power_land_rules(onboard_t *s, uint32_t t, const onb_inputs_t *in)
{
    const onb_params_t *p = OPAR(s);
#if PM_WIRED
    if (PM_BUILD(&p->pw) == PWR_SRC_WIRED) {
        if (pm_sag_land(&s->pm, &p->pw)) {
            power_act(s, t, in, LP_R_W_SAG);          /* W_SAG: land while power remains */
            return true;
        }
        return false;
    }
#endif
#if PM_BUF
    if (pm_lost_cond(&s->pm, &p->pw)) {               /* W_LOST: the tether is lost - land on the buffer at once */
        s->lost_noted = true;
        power_act(s, t, in, LP_R_W_LOST);
        return true;
    }
    if (pm_buf_cond(&s->pm, &p->pw)) {                /* W_BUF: land while the buffer can carry the landing */
        power_act(s, t, in, LP_R_W_BUF);
        return true;
    }
    if (pm_d2_cond(&s->pm, &p->pw)) {                 /* W_D2: the buffer charges while tethered (D2 short?) */
        power_act(s, t, in, LP_R_W_D2);
        return true;
    }
    if (pm_i8_buf(&s->pm, &p->pw)) {                  /* I8 on the buffer */
        power_act(s, t, in, LP_R_I8_VCELL);
        return true;
    }
#endif
    return false;
}
#endif

static void rules_vtol(onboard_t *s, uint32_t t, const onb_inputs_t *in)
{
    const onb_params_t *p = OPAR(s);
    const uint8_t st = s->state;
    bool on_pad, landed;
    uint32_t since;
    if (st == ONB_POWERED) {                          /* I3 persistence (as ballistic) */
        const float wx = in->gyro[0], wy = in->gyro[1], wz = in->gyro[2];
        const bool bad = (PRM(p->gyro_saturation_abort) && in->gyro_saturated) || (wx * wx + wy * wy > PRM(p->tilt_rate_abort_sq)) ||
                         fabsf(wz) > PRM(p->spin_rate_abort);
        s->rate_count = bad ? s->rate_count + 1u : 0u;
    } else {
        s->rate_count = 0;
    }
    if (flight(st) && in->vz_mps <= PRM(p->v_fall_vz_mps))  /* V5 persistence (POWERED / LANDED) */
        s->fall_count++;
    else
        s->fall_count = 0;
    if (!motor_enabled(st))
        return;
    on_pad = st == ONB_ARMED;
    /* arm switch: the hardware cut-off is the operator's last word (ARMED -> SAFE, POWERED -> ABORT).  LANDED -> SAFE:
     * on the ground with the motor off, the switch being opened means a person is at the vehicle - the release must
     * not fire at them (a false touchdown in the air is caught by V5 instead) */
    if (!in->arm_switch) {
        interlock(s, LP_R_ARM_SWITCH_OPEN);
        if (st == ONB_UNPOWERED)
            enter(s, ONB_SAFE, t, LP_R_ARM_SWITCH_OPEN);
        else
            kill(s, t, LP_R_ARM_SWITCH_OPEN);
        return;
    }
    landed = st == ONB_UNPOWERED;                     /* VTOL: UNPOWERED is LANDED (motor off, on the ground) */
    /* I9 IMU / NAV failure: without attitude the vanes cannot keep the thrust vertical; continuing would turn thrust
     * into a sideways fly-away - stopping the motor is the bounded outcome.  LANDED (v0.7.1): the motor is already
     * off - SAFE, no release toward people handling a landed vehicle (mission.vtol.landed_rules_note) */
    if (s->imu_fail >= PRM(p->imu_fail_ticks)) {
        interlock(s, LP_R_I9_IMU);
        if (on_pad || landed)
            enter(s, ONB_SAFE, t, LP_R_I9_IMU);
        else
            do_abort(s, t, LP_R_I9_IMU);
        return;
    }
    /* I9 baro failure: altitude is lost but attitude is not - a powered descent at a bounded rate on the NAV's dead
     * reckoning (the accelerometer with its learnt bias) is safer than dropping from a few metres (a parachute cannot
     * open there).  v0.7.1: the tick it acts also fixes the baro-lost time limit (V8 / V9) from h at that tick */
    if (s->baro_fail >= PRM(p->baro_fail_ticks)) {
        if (on_pad) {
            interlock(s, LP_R_I9_BARO);
            enter(s, ONB_SAFE, t, LP_R_I9_BARO);
            return;
        }
        if (st == ONB_POWERED && !s->baro_lost) {
            s->baro_lost = true;
            s->baro_lost_tick = t;
            s->baro_limit = onboard_baro_limit_ticks(p, in->h_m);
            if (fly_phase(s->phase))
                land(s, LP_R_I9_BARO);
            else
                interlock(s, LP_R_I9_BARO);           /* already landing: the dead-reckoned landing from now on */
            return;
        }
    }
    since = ticks_since_frame(s, t);
    if (on_pad) {                                     /* I1p: never start the motor blind */
        if (since >= PRM(p->link_disarm_ticks)) {
            interlock(s, LP_R_I1P_LINK_LOSS_PAD);
            enter(s, ONB_SAFE, t, LP_R_I1P_LINK_LOSS_PAD);
        }
        return;
    }
    if (st == ONB_POWERED) {
        if (in->tilt_deg > PRM(p->tilt_abort_deg)) {       /* I2: the thrust can no longer hold the altitude */
            interlock(s, LP_R_I2_TILT);
            do_abort(s, t, LP_R_I2_TILT);
            return;
        }
        if (s->rate_count >= PRM(p->rate_persist_ticks)) {  /* I3: spin / tilt rate / gyro saturation */
            interlock(s, LP_R_I3_RATE);
            do_abort(s, t, LP_R_I3_RATE);
            return;
        }
    }
    /* V4: a climb past the ceiling LAND did not stop - the altitude loop has failed; motor off is the only bound left.
     * LANDED (v0.7.1): motor already off; on the ground the reading is a baro glitch - SAFE, no release */
    if (in->h_m > PRM(p->v_geofence_m)) {
        interlock(s, LP_R_V_GEOFENCE);
        if (landed)
            enter(s, ONB_SAFE, t, LP_R_V_GEOFENCE);
        else
            do_abort(s, t, LP_R_V_GEOFENCE);
        return;
    }
    /* V5: commanded descents are <= vz_max_down; a fall means the thrust is gone or the vehicle is out of control - motor
     * off, parachute.  LANDED (v0.7.1, landed_rules_note): SAFE, never ABORT.  Consequence, stated: a FALSE touchdown
     * confirmed in the air is no longer caught by a release; the protection is the touchdown detector itself (only while
     * low, physical signatures that free flight cannot produce) */
    if (s->fall_count >= PRM(p->v_fall_confirm_ticks)) {
        interlock(s, LP_R_V_FALL);
        if (landed)
            enter(s, ONB_SAFE, t, LP_R_V_FALL);
        else
            do_abort(s, t, LP_R_V_FALL);
        return;
    }
    if (st == ONB_POWERED && s->powered_ticks >= PRM(p->hard_cutoff_ticks)) {   /* V6: absolute bound on battery use, even */
        interlock(s, LP_R_V_HARD_CUTOFF);                                  /* if a landing is still under way */
        do_abort(s, t, LP_R_V_HARD_CUTOFF);
        return;
    }
    /* V15 (v0.8) take-off check: the vertical controller pinned at its ceiling (u_max, ramp over) for within_s without
     * climbing climb_m (previous tick) -> motor off, SAFE, no release: the vehicle is on or just above the pad (v0.7.1
     * finding: a T/W < 1 vehicle sat at full throttle on the pad until the flight-time rule) */
    if (PRM(p->takeoff_check) && st == ONB_POWERED && fly_phase(s->phase) && in->vc_takeoff_fail) {
        interlock(s, LP_R_V_TAKEOFF_CHECK);
        enter(s, ONB_SAFE, t, LP_R_V_TAKEOFF_CHECK);
        return;
    }
    /* v0.7.1, POWERED LAND without a touchdown confirmed in the previous tick */
    if (st == ONB_POWERED && s->phase == LP_PHASE_LAND && !in->vc_touchdown) {
        if (s->baro_lost) {
            /* V8 / V9: the baro-lost time limit (v0.7.0 had none: the integrated accelerometer bias flew 7 of 20 seeds
             * away until V6).  A ground signature (support or floor, previous tick) -> LANDED, motor off, no release;
             * none -> ABORT: the altitude is unknown, the vehicle may be high - motor off, parachute */
            if (t - s->baro_lost_tick >= s->baro_limit) {
                if (in->vc_ground_signature) {
                    interlock(s, LP_R_V_BARO_LIMIT_LANDED);
                    enter(s, ONB_UNPOWERED, t, LP_R_V_BARO_LIMIT_LANDED);
                } else {
                    interlock(s, LP_R_V_BARO_LIMIT_ABORT);
                    do_abort(s, t, LP_R_V_BARO_LIMIT_ABORT);
                }
                return;
            }
        } else if (PRM(p->land_timeout_ticks) > 0u && in->vc_low_in_land_ticks >= PRM(p->land_timeout_ticks)) {
            /* V10 LAND floor timeout: low and not following the descending reference for land_timeout_ticks without a
             * confirmed touchdown -> LANDED, motor off, NO release: a vehicle 0-1 m above the ground is safer with the
             * fan stopped than with a release at the ground crew (v0.7.0 ran the motor on the ground until V6) */
            interlock(s, LP_R_V_LAND_TIMEOUT);
            enter(s, ONB_UNPOWERED, t, LP_R_V_LAND_TIMEOUT);
            return;
        }
    }
#if PM_BUF
    /* v0.9 W_LOST during LAND (already landing): the reason is latched once for the ground's alarm, nothing else */
    if (PM_BUILD(&p->pw) == PWR_SRC_WIRED_BUFFER && st == ONB_POWERED && s->phase == LP_PHASE_LAND &&
        pm_lost_cond(&s->pm, &p->pw) && !s->lost_noted) {
        s->lost_noted = true;
        interlock(s, LP_R_W_LOST);
    }
#endif
    if (st != ONB_POWERED || !fly_phase(s->phase))    /* LAND rules: CLIMB / HOLD only (in LAND the vehicle is */
        return;                                       /* already doing what they ask) */
    if (since >= PRM(p->link_land_ticks)) {                /* V1: without the link the operator can neither command nor */
        land(s, LP_R_V_LINK_LAND);                    /* KILL; at a few metres a parachute does not open - land */
        return;
    }
#if PM_ANY
    if (PM_BUILD(&p->pw) != PWR_SRC_BATTERY) {
        if (power_land_rules(s, t, in))               /* v0.9: the power build's rules in the I8 slot */
            return;
    } else
#endif
    if (in->v_cell < PRM(p->min_cell_v)) {                 /* I8: the pack is near empty - land while there is power */
        land(s, LP_R_I8_VCELL);
        return;
    }
    if (in->h_m > PRM(p->ceiling_m)) {                     /* V3: the target is limited to alt_max < ceiling; above it the */
        land(s, LP_R_V_CEILING);                      /* altitude loop misbehaves - come down under control */
        return;
    }
    if (s->powered_ticks >= PRM(p->endurance_ticks)) {     /* V2: keep land_reserve_s of battery for the descent */
        land(s, LP_R_V_ENDURANCE);
        return;
    }
    if (PRM(p->gps_hold)) {                                /* v0.8: the position controller's LAND requests (published, */
        const uint8_t rq = in->pc_requests;           /* previous tick) - a powered landing, never ABORT near people */
        if (rq & POS_REQ_GPS_LOST) {                  /* V11: no usable GPS for timeout_s (attitude-only hold), then */
            land(s, LP_R_V_GPS_LOST);                 /* land_after_s - the landing drifts with the wind */
            return;
        }
        if (rq & POS_REQ_FENCE) {                     /* V12: |p - hold point| > fence_m */
            land(s, LP_R_V_FENCE);
            return;
        }
        if (rq & POS_REQ_IMPLAUSIBLE) {               /* V13: GPS and IMU disagree - the GPS is not flown on any more */
            land(s, LP_R_V_GPS_IMPLAUSIBLE);
            return;
        }
        if (rq & POS_REQ_AUTHORITY) {                 /* V14: k_T T/W < min_kT_TW: no tilt left for position hold */
            land(s, LP_R_V_AUTHORITY);
            return;
        }
    }
}

/* VTOL timers, after the rules and the release timer */
static void timers_vtol(onboard_t *s, uint32_t t, const onb_inputs_t *in)
{
    const onb_params_t *p = OPAR(s);
    if (s->state == ONB_ARMED) {
        if (t - s->t_entry >= PRM(p->spool_ticks)) {
#if PM_ANY
            const bool pw_ok = PM_BUILD(&p->pw) == PWR_SRC_BATTERY ? in->v_cell >= PRM(p->min_cell_v)
                                                                   : pm_arm_ok(&s->pm, &p->pw);   /* v0.9: the ON window */
#else
            const bool pw_ok = in->v_cell >= PRM(p->min_cell_v);
#endif
            if (in->tilt_deg < PRM(p->arm_tilt_max_deg) && pw_ok) {
                enter(s, ONB_POWERED, t, ONB_CAUSE_SPOOL_DONE);   /* motor on: CLIMB */
            } else {                                  /* V7: knocked over or the pack sagged during the warning - */
                interlock(s, LP_R_V_TAKEOFF_GATE);    /* never start the motor then */
                enter(s, ONB_SAFE, t, LP_R_V_TAKEOFF_GATE);
            }
        }
    } else if (s->state == ONB_POWERED) {
        if (s->phase == LP_PHASE_CLIMB && in->vc_at_target)
            set_phase(s, LP_PHASE_HOLD, ONB_CAUSE_AT_TARGET);
        else if (s->phase == LP_PHASE_LAND && in->vc_touchdown)
            enter(s, ONB_UNPOWERED, t, ONB_CAUSE_TOUCHDOWN);       /* motor off (LANDED) */
    } else if (s->state == ONB_UNPOWERED) {
        if (t - s->t_entry >= PRM(p->safe_after_landed_ticks))
            enter(s, ONB_SAFE, t, ONB_CAUSE_LANDED_TIMER);
    }
}

#endif

#if ONB_HAS_BALLISTIC
/* ---- throttle (ballistic) ------------------------------------------------------------------- */
static void slew(onboard_t *s)
{
    const uint32_t tgt = s->state == ONB_POWERED ? s->target_milli : 0u;
    const uint32_t r = PRM(OPAR(s)->ramp_ticks);
    if (r == 0u) {
        s->out_milli = (uint16_t)tgt;
        return;
    }
    if (tgt < s->out_milli) {                         /* decreases are immediate */
        s->out_milli = (uint16_t)tgt;
        s->level = (tgt * r + (LP_THROTTLE_SCALE - 1u)) / LP_THROTTLE_SCALE;
    } else if (tgt > s->out_milli) {
        uint32_t o;
        s->level = s->level + 1u < r ? s->level + 1u : r;
        o = s->level * LP_THROTTLE_SCALE / r;
        s->out_milli = (uint16_t)(tgt < o ? tgt : o);
    }
}
#endif

static uint8_t flags(const onboard_t *s, uint32_t t, const onb_inputs_t *in)
{
    uint8_t f = 0;
    if (in->arm_switch)
        f |= LP_FLAG_ARM_SWITCH;
    if (in->imu_ok && in->baro_ok && s->imu_fail == 0u && s->baro_fail == 0u)
        f |= LP_FLAG_SENSORS_OK;
    if (in->aligned)
        f |= LP_FLAG_ALIGNED;
    if (onboard_link_ok(s, t))
        f |= LP_FLAG_LINK_OK;
    if (s->release_open)
        f |= LP_FLAG_RELEASED;
    if (onboard_control_active(s, t))
        f |= LP_FLAG_CONTROL_ACTIVE;
    if (s->ack_rejected)
        f |= LP_FLAG_REJECTED;
    if (s->interlock_latched)
        f |= LP_FLAG_INTERLOCK;
    return f;
}

/* ---- main ------------------------------------------------------------------------------------- */
void onboard_update(onboard_t *s, const onb_inputs_t *in, onb_outputs_t *out)
{
    const uint32_t t = in->tick;
    const bool vtol = IS_VTOL(s);
    uint8_t i;
    out->n_exec = 0;
    out->n_trans = 0;
    out->interlock = 0;
    out->reply = false;
    out->reply_seq = 0;
    s->out = out;
    if (in->imu_ok)
        s->imu_fail = 0;
    else if (s->imu_fail < 0xFFFFFFFFUL)
        s->imu_fail++;
    if (in->baro_ok)
        s->baro_fail = 0;
    else if (s->baro_fail < 0xFFFFFFFFUL)
        s->baro_fail++;
#if PM_ANY
    if (vtol)                                         /* v0.9 power monitor: this tick's readings (the reference's note */
        (void)pm_sample(&s->pm, &OPAR(s)->pw, &in->pw, s->state == ONB_POWERED, in->throttle_prev);   /* is log text) */
#endif
    /* 1. received commands */
    for (i = 0; i < in->n_frames; i++) {
        const onb_frame_t *f = &in->frames[i];
        const bool link_ok_before = onboard_link_ok(s, t);
        uint8_t code, before, ph_before;
        s->has_frame = true;
        s->last_frame_tick = t;
        out->reply = true;
        out->reply_seq = f->seq;
        before = s->state;
        ph_before = s->phase;
        if (s->has_seq && !lp_seq_newer(f->seq, s->last_seq)) {
            code = ONB_EXEC_REPEAT;                   /* repeat of a processed SEQ (or older): reply only */
        } else {
            s->has_seq = true;
            s->last_seq = f->seq;
            if (f->cmd.cmd == LP_CMD_NONE) {
                s->ack_rejected = false;
                code = ONB_EXEC_NONE;
            } else {
#if ONB_HAS_VTOL && ONB_HAS_BALLISTIC
                code = vtol ? execute_vtol(s, f, t, in, link_ok_before) : execute_ballistic(s, f, t, in, link_ok_before);
#elif ONB_HAS_VTOL
                code = execute_vtol(s, f, t, in, link_ok_before);
#else
                code = execute_ballistic(s, f, t, in, link_ok_before);
#endif
                s->ack_rejected = code != 0u;
                if (code && !s->interlock_latched)    /* the interlock code outranks rejections */
                    s->reason = code;
            }
        }
        if (out->n_exec < ONB_MAX_FRAMES) {
            onb_exec_t *e = &out->exec[out->n_exec++];
            e->seq = f->seq;
            e->cmd = f->cmd.cmd;
            e->code = code;
            e->state_before = before;
            e->state_after = s->state;
            e->phase_before = ph_before;
            e->phase_after = s->phase;
        }
    }
    /* 2. interlocks / safety rules */
#if ONB_HAS_VTOL && ONB_HAS_BALLISTIC
    if (vtol)
        rules_vtol(s, t, in);
    else
        interlocks_ballistic(s, t, in);
#elif ONB_HAS_VTOL
    rules_vtol(s, t, in);
#else
    interlocks_ballistic(s, t, in);
#endif
    /* 3. release timer (pending release or ABORT) */
    if (s->has_release_deadline && t >= s->release_deadline) {
        s->has_release_deadline = false;
        s->release_open = true;
        enter(s, ONB_RELEASED, t, ONB_CAUSE_RELEASE_TIMER);
    }
    /* 4. VTOL timers (spool end + take-off gate, CLIMB -> HOLD, touchdown, LANDED -> SAFE) / ballistic throttle
     * slew; then the I7 / V2 / V6 clock: POWERED ticks since boot */
#if ONB_HAS_VTOL && ONB_HAS_BALLISTIC
    if (vtol)
        timers_vtol(s, t, in);
    else
        slew(s);
#elif ONB_HAS_VTOL
    timers_vtol(s, t, in);
#else
    slew(s);
#endif
    if (s->state == ONB_POWERED)
        s->powered_ticks++;
    out->state = s->state;
#if ONB_HAS_BALLISTIC
    out->target_milli = s->target_milli;
    out->throttle_milli = s->out_milli;
#else
    out->target_milli = 0;                            /* VTOL: the ESC follows the vertical controller */
    out->throttle_milli = 0;
#endif
    out->control_active = onboard_control_active(s, t);
    out->release = s->release_open;
    out->reason = s->reason;
    out->flags = flags(s, t, in);
    out->ack_seq = s->has_seq ? s->last_seq : 0u;
    out->nonce = s->nonce;
    out->phase = s->phase;
#if ONB_HAS_VTOL
    out->h_tgt_dm = vtol ? h_tgt_dm(s) : 0;
#else
    out->h_tgt_dm = 0;
#endif
    out->motor_on = s->state == ONB_POWERED;
    out->buzzer = vtol && s->state == ONB_ARMED;
    out->baro_lost = s->baro_lost;

    if (out->reply) {
        onb_tlm_src_t *m = &out->tlm;
        m->state = out->state;
        m->flags = out->flags;
        m->reason = out->reason;
        m->ack_seq = out->ack_seq;
        m->nonce = out->nonce;
        /* TELEM v1: the slewed throttle; v2: the ESC throttle applied in the previous tick (the vertical controller
         * runs after the supervisor) */
        m->throttle_milli = vtol ? (uint16_t)lp_quantize(in->throttle_prev, LP_THROTTLE_SCALE, 0, LP_U16_MAX)
                                 : out->throttle_milli;
        m->tick = t;
        m->h_m = in->h_m;
        m->vz_mps = in->vz_mps;
        m->roll_deg = in->roll_deg;
        m->pitch_deg = in->pitch_deg;
        m->yaw_deg = in->yaw_deg;
        m->tilt_deg = in->tilt_deg;
        m->v_cell = in->v_cell;
        m->loop_max_us = in->loop_max_us;
        m->v2 = vtol;
        m->phase = vtol ? out->phase : LP_PHASE_NONE;
        m->h_tgt_m = (float)out->h_tgt_dm / 10.0f;    /* = float32(dm / 10.0) for every int16 dm; only for a reply (a
                                                       * float division per tick saved: the controller takes the dm) */
#if ONB_HAS_VTOL
        m->v3 = vtol && PRM(OPAR(s)->gps_hold);                /* v0.8 TELEM v3: the GPS fields as of the previous tick */
#else
        m->v3 = false;
#endif
        m->gps_flags = m->v3 ? in->tlm_gps.flags : 0u;
        m->num_sv = m->v3 ? in->tlm_gps.num_sv : 0u;
        m->hacc_dm = m->v3 ? in->tlm_gps.hacc_dm : 0u;
        m->pos_mode = m->v3 ? in->tlm_gps.pos_mode : 0u;
        m->dist_m = m->v3 ? in->tlm_gps.dist_m : 0.0f;
#if PM_ANY
        {                                             /* v0.9 TELEM v4 / v5 power tail (PROPOSED) */
            const bool v4 = vtol && PM_BUILD(&OPAR(s)->pw) != PWR_SRC_BATTERY;
            m->pw_vaux = v4 ? pm_vaux_q(&s->pm, &OPAR(s)->pw) : 0u;
            m->pw_flags = v4 ? pm_flags(&s->pm, &OPAR(s)->pw) : 0u;
        }
#endif
    }
    s->out = 0;
}

void onboard_telemetry_part(const onb_tlm_src_t *m, uint16_t loop_hz, lp_telem_t *t, uint8_t part)
{
    if (part == 0u) {
        const uint32_t q = m->tick / loop_hz, r = m->tick % loop_hz;
        t->state = m->state;
        t->flags = m->flags;
        t->reason = m->reason;
        t->ack_seq = m->ack_seq;
        t->nonce = m->nonce;
        t->t_ms = q * 1000u + (r * 1000u) / loop_hz;  /* = floor(tick * 1000 / loop_hz) mod 2^32 */
        t->h_dm = (int16_t)lp_quantize(m->h_m, LP_H_SCALE, LP_I16_MIN, LP_I16_MAX);
        t->vz_cms = (int16_t)lp_quantize(m->vz_mps, LP_VZ_SCALE, LP_I16_MIN, LP_I16_MAX);
        t->roll_cdeg = (int16_t)lp_quantize(m->roll_deg, LP_ANGLE_SCALE, LP_I16_MIN, LP_I16_MAX);
    } else {
        t->pitch_cdeg = (int16_t)lp_quantize(m->pitch_deg, LP_ANGLE_SCALE, LP_I16_MIN, LP_I16_MAX);
        t->yaw_cdeg = (int16_t)lp_quantize(m->yaw_deg, LP_ANGLE_SCALE, LP_I16_MIN, LP_I16_MAX);
        t->tilt_cdeg = (uint16_t)lp_quantize(m->tilt_deg, LP_ANGLE_SCALE, 0, LP_U16_MAX);
        t->throttle_milli = m->throttle_milli;        /* integer per mille already (quantize(n/1000, 1000) == n) */
        t->vcell_mv = (uint16_t)lp_quantize(m->v_cell, LP_VCELL_SCALE, 0, LP_U16_MAX);
        t->loop_max_us = (uint16_t)lp_quantize(m->loop_max_us, 1u, 0, LP_U16_MAX);
        t->phase = m->v2 ? m->phase : 0u;
        t->h_tgt_dm = m->v2 ? (int16_t)lp_quantize(m->h_tgt_m, LP_H_SCALE, LP_I16_MIN, LP_I16_MAX) : 0;
        if (m->v3) {                                  /* v0.8 TELEM v3 */
            t->gps_flags = m->gps_flags;
            t->num_sv = m->num_sv;
            t->hacc_dm = m->hacc_dm;
            t->pos_mode = m->pos_mode;
            t->dist_dm = m->dist_m != m->dist_m ? (uint16_t)LP_DIST_NONE
                                                : (uint16_t)lp_quantize(m->dist_m, LP_DIST_SCALE, 0, LP_DIST_NONE - 1L);
        } else {
            t->gps_flags = t->num_sv = t->hacc_dm = t->pos_mode = 0;
            t->dist_dm = 0;
        }
    }
}

void onboard_telemetry(const onb_tlm_src_t *m, uint16_t loop_hz, lp_telem_t *t)
{
    onboard_telemetry_part(m, loop_hz, t, 0);
    onboard_telemetry_part(m, loop_hz, t, 1);
}

uint8_t onboard_telemetry_frame(const onb_tlm_src_t *src, uint16_t loop_hz, uint8_t veh, uint8_t seq, uint8_t *out)
{
    lp_telem_t t;
    onboard_telemetry(src, loop_hz, &t);
#if PM_ANY
    if (src->v2 && LP_PWR_TAIL_DUE(src->pw_flags))      /* v0.9 RP1: a power build's TELEM v4 / v5 (com/com_link.c rule) */
        return src->v3 ? lp_encode_telem5(veh, seq, &t, src->pw_vaux, src->pw_flags, out)
                       : lp_encode_telem4(veh, seq, &t, src->pw_vaux, src->pw_flags, out);
#endif
    if (src->v3)
        return lp_encode_telem3(veh, seq, &t, out);
    return src->v2 ? lp_encode_telem2(veh, seq, &t, out) : lp_encode_telem(veh, seq, &t, out);
}

/* ---- the FCU stack (sim/onboard.py FcuStack) -------------------------------------------------------- */
void onb_stack_init(onb_stack_t *k, const onb_params_t *p, const vc_params_t *vp, const pos_params_t *pp, uint8_t chain,
                    uint16_t seed, bool reset_in_flight)
{
    onboard_init(&k->sup, p, seed, reset_in_flight);
#if ONB_HAS_VTOL
    vc_init(&k->vc, vp);
    k->have_vo = false;
    k->vo.touchdown = false;
    k->vo.at_target = false;
    k->vo.sig = 0;
    k->vo.low_in_land_ticks = 0;
#else
    (void)vp;                                         /* a ballistic fixed build has no vertical controller */
#endif
#if ONB_HAS_GPS
    k->gps = IS_VTOL(&k->sup) && PRM(OPAR(&k->sup)->gps_hold) && pp != 0;
#if ONB_HAS_GNS && ONB_HAS_FCU_POS
    k->chain = chain == ONB_CHAIN_FCU ? ONB_CHAIN_FCU : ONB_CHAIN_GNS;
#elif ONB_HAS_GNS
    (void)chain;
    k->chain = ONB_CHAIN_GNS;
#else
    (void)chain;
    k->chain = ONB_CHAIN_FCU;
#endif
    k->tilt[0] = k->tilt[1] = 0.0f;
    k->slew = pp != 0 ? pos_params_slew_per_tick(pp) : 0.0f;
    k->motor_on = false;
    k->bg_left = false;
    k->bg_late = 0;
#if ONB_HAS_GNS
    nl_gns_rx_init(&k->grx, pp != 0 ? pos_params_step_ticks(pp) : (uint8_t)5u);
    k->land_steps = pp != 0 ? pos_params_land_steps(pp) : 0u;
    k->pflags = 0;
    k->kt_q = (uint16_t)NL_KT_SCALE;
#endif
#if ONB_HAS_FCU_POS
    nl_slot_rx_init(&k->rx);
    if (pp != 0 && k->chain == ONB_CHAIN_FCU)
        pos_init(&k->pos, pp);
    k->pos.job = 0;
    k->pipe.k = 0xFFFFFFFFUL;                         /* the first step makes it 0 (FcuStack.k = -1) */
    k->pipe.pub_tick = 0;
    k->pipe.pend_valid = false;
    k->pipe.pend_tick = 0;
    pos_pub_off(&k->pipe.pend);
    pos_pub_off(&k->pipe.pub);
    k->pipe.overruns = 0;
    k->pipe.off = 0;
    k->pos_ticks = 0;
    k->sat_hi = false;
#endif
#else
    (void)pp;
    (void)chain;
#endif
    k->att_active = false;
    k->throttle = 0.0f;
}

#if ONB_HAS_GNS
/* v0.8.1 (0): the GNS slot of this tick's NAV frame; the supervisor's GPS inputs from the newest set (the TELEM v3 mode
 * with the previous tick's motor_on, like the reference's sup_out) */
static void stack_gns_in(onb_stack_t *k, onb_inputs_t *in)
{
    (void)nl_gns_rx_feed(&k->grx, in->gps_slot);
    in->gps_gate_ok = nl_gns_rx_gate_ok(&k->grx);
    in->pc_requests = nl_gns_rx_requests(&k->grx, k->land_steps);
    nl_gns_rx_tlm(&k->grx, k->motor_on, &in->tlm_gps.flags, &in->tlm_gps.num_sv, &in->tlm_gps.hacc_dm,
                  &in->tlm_gps.pos_mode, &in->tlm_gps.dist_m);
}

/* v0.8.1 (6): this tick's FCU-side inputs of the GNS position controller for the nlink v4 FCU frame (the reference's
 * fcu4_pflags(phase, motor_on, sat == +1, low_alt, low_alt and (still or support or floor), lifted), kt_quant(k_T)) */
static void stack_gns_rt(onb_stack_t *k, const onb_outputs_t *so)
{
    const uint8_t sig = k->vo.sig;
    const bool low_alt = (sig & VC_SIG_LOW_ALT) != 0u;
    k->pflags = nl_fcu4_pflags(so->phase, so->motor_on, k->vo.sat == 1, low_alt,
                               low_alt && (sig & (VC_SIG_STILL | VC_SIG_SUPPORT | VC_SIG_FLOOR)) != 0u,
                               vc_takeoff_ticks(&k->vc) == 0u || k->vc.to_passed);
    k->kt_q = nl_kt_quant(k->vc.k_t);
    k->motor_on = so->motor_on;
    k->bg_left = true;                                /* (7) the builder: onb_stack_bg */
}
#endif

#if ONB_HAS_FCU_POS
/* the position-hold part of onb_stack_step before the supervisor: (0) the slot receiver and the supervisor's GPS inputs */
static const nl_gps_set_t *stack_gps_in(onb_stack_t *k, onb_inputs_t *in)
{
    const nl_gps_set_t *g;
    k->pipe.k++;
    g = nl_slot_rx_feed(&k->rx, in->gps_slot);
    in->gps_gate_ok = nl_slot_rx_gate_ok(&k->rx);
    in->pc_requests = k->pipe.pub.requests;
    in->tlm_gps.flags = nl_slot_rx_fresh(&k->rx) ? k->rx.latest.flags : 0u;
    in->tlm_gps.num_sv = k->rx.has_latest ? k->rx.latest.num_sv : 0u;
    in->tlm_gps.hacc_dm = k->rx.has_latest ? k->rx.latest.hacc_dm : (uint8_t)LP_HACC_NONE;
    in->tlm_gps.pos_mode = k->pipe.pub.mode;
    in->tlm_gps.dist_m = k->pipe.pub.dist_m;
    return g;
}

/* (6) the position step when due: finish a previous one still in its pieces, latch the inputs (the rt part) */
static void stack_gps_rt(onb_stack_t *k, const onb_inputs_t *in, const onb_outputs_t *so, const nl_gps_set_t *g)
{
    pos_pipe_t *pp = &k->pipe;
    k->sat_hi = k->sat_hi || k->vo.sat == 1;
    k->motor_on = so->motor_on;
    if (k->pos_ticks < 0xFFu)
        k->pos_ticks++;
    if (k->rx.got_part4 || k->pos_ticks >= pos_step_ticks(&k->pos)) {
        pos_inputs_t pi;
        const float c = in->cos_tilt;
        if (k->pos.job != 0u) {                       /* the previous step (a hold step: a set needs five ticks) is */
            if (pp->pend_valid) {                     /* still in its pieces: finish it now; a second overlap */
                pp->pub = pp->pend;                   /* publishes the older one early (counted) */
                pp->overruns++;
            }
            (void)pos_finish(&k->pos, &pp->pend);     /* its outputs wait for their own publication tick */
            pp->pend_tick = pp->pub_tick;
            pp->pend_valid = true;
        }
        pi.phase = so->phase;
        pi.motor_on = so->motor_on;
        pi.h_m = in->h_m;
        pi.cos_tilt = c == c ? c : 1.0f;
        pi.rate[0] = in->gyro[0];
        pi.rate[1] = in->gyro[1];
        pi.rate[2] = in->gyro[2];
        pi.k_t = k->vc.k_t;
        pi.vc_sat_high = k->sat_hi;
        pi.data = 0;
        pi.set = g;                                   /* the slot set (dequantised by pos_latch, as v0.8) */
        /* physics-review fix: the vertical controller's flare band, touchdown signatures and take-off check of this tick */
        pi.vc_low_alt = (k->vo.sig & VC_SIG_LOW_ALT) != 0u;
        pi.vc_ground = pi.vc_low_alt && (k->vo.sig & (VC_SIG_STILL | VC_SIG_SUPPORT | VC_SIG_FLOOR)) != 0u;
        pi.lifted = vc_takeoff_ticks(&k->vc) == 0u || k->vc.to_passed;
        pos_latch(&k->pos, &pi);
        pp->pub_tick = pp->k + POS_PIPELINE_TICKS;
        pp->off = 0;
        k->pos_ticks = 0;
        k->sat_hi = false;
    } else if (k->pos.job != 0u && pp->off < 0xFFu) {
        pp->off++;
    }
    k->bg_left = true;
}
#endif

bool onb_stack_bg(onb_stack_t *k)
{
#if ONB_HAS_GPS
    if (!k->bg_left)
        return false;
#if ONB_HAS_GNS
    if (k->chain == ONB_CHAIN_GNS) {                  /* (7) the builder on the slot's TILT / the newest set's TRIM */
        float tgt[2];
        const bool eng = nl_gns_rx_target(&k->grx, tgt);
        pos_tilt_tick(k->tilt, tgt, eng && k->motor_on, k->slew);
        k->bg_left = false;
        return false;
    }
#endif
#if ONB_HAS_FCU_POS
    {
        pos_pipe_t *pp = &k->pipe;
        if (pos_piece_due(&k->pos, pp->off)) {        /* (6) one piece of the step in progress */
            pos_pub_t *dst = pp->k >= pp->pub_tick ? &pp->pub : &pp->pend;   /* the commit: at its publication tick */
            if (pos_piece(&k->pos, dst) && dst == &pp->pend) {               /* (cannot happen: P10 runs at k + 4) */
                pp->pend_tick = pp->pub_tick;
                pp->pend_valid = true;
            }
            return true;
        }
        if (pp->pend_valid && pp->pend_tick <= pp->k) {
            pp->pub = pp->pend;
            pp->pend_valid = false;
        }
        pos_tilt_tick(k->tilt, pp->pub.tilt_l, pp->pub.engaged && k->motor_on, k->slew);   /* (7) */
    }
#endif
    k->bg_left = false;
    return false;
#else
    (void)k;
    return false;
#endif
}

float onb_stack_step(onb_stack_t *k, onb_inputs_t *in, onb_outputs_t *so)
{
    const float u = onb_stack_rt(k, in, so);
    while (onb_stack_bg(k)) {
    }
    return u;
}

float onb_stack_rt(onb_stack_t *k, onb_inputs_t *in, onb_outputs_t *so)
{
#if ONB_HAS_GPS
    if (k->bg_left) {                                 /* the previous tick's background did not finish: now, first */
        while (onb_stack_bg(k)) {
        }
        if (k->bg_late < 0xFFFFu)
            k->bg_late++;
    }
#endif
#if ONB_HAS_BALLISTIC
    if (!IS_VTOL(&k->sup)) {                          /* ballistic: the supervisor's slewed throttle (v0.6.3 order: */
        onboard_update(&k->sup, in, so);              /* the attitude controller runs after this with this tick's) */
        k->throttle = (float)so->throttle_milli / (float)LP_THROTTLE_SCALE;
        k->att_active = so->control_active;
        return k->throttle;
    }
#endif
#if ONB_HAS_VTOL
    {
#if ONB_HAS_FCU_POS
        const nl_gps_set_t *g = 0;
#endif
#if ONB_HAS_GPS
        if (k->gps) {                                 /* (0) the GPS / GNS slot of this tick's NAV frame */
#if ONB_HAS_GNS
            if (k->chain == ONB_CHAIN_GNS)
                stack_gns_in(k, in);
#endif
#if ONB_HAS_FCU_POS
            if (k->chain == ONB_CHAIN_FCU)
                g = stack_gps_in(k, in);
#endif
        } else
#endif
        {                                             /* no position hold: no GPS gate, no requests */
            in->gps_gate_ok = false;
            in->pc_requests = 0;
            in->tlm_gps.flags = in->tlm_gps.num_sv = in->tlm_gps.pos_mode = 0;
            in->tlm_gps.hacc_dm = (uint8_t)LP_HACC_NONE;
            in->tlm_gps.dist_m = NAN;
        }
        /* (1) attitude controller and (2) vanes: the caller, before this call, with onb_stack_att_active / _att_u */
        in->vc_touchdown = k->have_vo && k->vo.touchdown;   /* (3) the previous tick's controller flags and throttle */
        in->vc_at_target = k->have_vo && k->vo.at_target;
        in->vc_ground_signature = k->have_vo && (k->vo.sig & VC_SIG_GROUND) != 0u;
        in->vc_low_in_land_ticks = k->have_vo ? k->vo.low_in_land_ticks : 0u;
        in->vc_takeoff_fail = k->have_vo && (k->vo.sig & VC_SIG_TAKEOFF_FAIL) != 0u;
#if PM_ANY
        /* v0.9 power rules: FcuStack vc_takeoff_passed (true without a take-off check) / vc_support / vc_airborne of the
         * previous tick (an IDLE tick reset the controller: false) */
        in->vc_takeoff_passed = vc_takeoff_ticks(&k->vc) == 0u || (k->have_vo && vc_takeoff_passed(&k->vc));
        in->vc_support = k->have_vo && (k->vo.sig & VC_SIG_SUPPORT) != 0u;
        in->vc_airborne = k->have_vo && vc_airborne(&k->vc);
#endif
        in->throttle_prev = k->throttle;
        onboard_update(&k->sup, in, so);
        {                                             /* (4) the vertical controller with this tick's phase */
            vc_inputs_t vi;
            vi.phase = so->phase;
            vi.h_tgt_dm = so->h_tgt_dm;
            vi.baro_lost = so->baro_lost;
            vi.h_m = in->h_m;
            vi.vz_mps = in->vz_mps;
            vi.cos_tilt = in->cos_tilt;
            vi.f_z_mps2 = in->accel_z_mps2;
#if PM_WIRED
            vi.u_capped = onboard_u_capped(&k->sup);  /* v0.9 W_SAG: the supervisor's frozen ceiling (OnboardOutputs.u_cap) */
            vi.u_cap = k->sup.pm.u_cap;
#endif
            vc_update(&k->vc, &vi, &k->vo);
        }
        k->have_vo = true;
        k->att_active = so->control_active;           /* the next tick's attitude controller */
        k->throttle = so->motor_on ? k->vo.u : 0.0f;  /* (5) the ESC follows the controller only while POWERED */
#if ONB_HAS_GPS
        if (k->gps) {
#if ONB_HAS_GNS
            if (k->chain == ONB_CHAIN_GNS)
                stack_gns_rt(k, so);                  /* (6) the FCU4 fields; (7) in onb_stack_bg */
#endif
#if ONB_HAS_FCU_POS
            if (k->chain == ONB_CHAIN_FCU)
                stack_gps_rt(k, in, so, g);           /* (6) the position step's latch; its pieces and (7): onb_stack_bg */
#endif
        }
#endif
    }
#endif
    return k->throttle;
}
