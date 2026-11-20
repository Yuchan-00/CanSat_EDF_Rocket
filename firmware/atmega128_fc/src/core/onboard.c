/* onboard.c - see onboard.h (mirrors sim/onboard.py OnboardSupervisor branch by branch).  The functions are the
 * v0.6.3 ballistic (rocket) code. */
#include "onboard.h"


#define LFSR_TAPS 0xB400u
#define LFSR_ZERO_SEED 0xACE1u


/* v0.8.1 RAM diet: OPAR(s) = the supervisor's parameter table, PRM(field) = one read of it (an LPM in a flash build) */
#define OPAR(s) (&(s)->p)
#define PRM(x) (x)

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
    s->p = *p;
    s->p.mode = LP_MODE_BALLISTIC;
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
    s->target_milli = 0;
    s->out_milli = 0;
    s->level = 0;
    s->has_unpowered = false;
    s->t_unpowered = 0;
    s->powered_ticks = 0;
    s->release_open = false;
    s->has_release_deadline = false;
    s->release_deadline = 0;
    s->rate_count = s->fall_count = s->imu_fail = s->baro_fail = 0;
    s->out = 0;
    if (reset_in_flight) {
        s->reason = LP_R_I10_RESET;
        s->interlock_latched = true;
        s->release_open = true;                       /* I10: SAFE with the release open, arming refused */
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
    return s->state == ONB_UNPOWERED && s->has_unpowered && tick - s->t_unpowered < PRM(OPAR(s)->coast_hold_ticks);
}

static void log_trans(onboard_t *s, uint8_t to, uint8_t cause)
{
    onb_outputs_t *o = s->out;
    if (o && o->n_trans < ONB_MAX_TRANS) {
        onb_trans_t *tr = &o->trans[o->n_trans++];
        tr->from = s->state;
        tr->to = to;
        tr->cause = cause;
    }
}

/* kept out of line (2026-11-19): with the removed VTOL phase bookkeeping gone avr-gcc / LTO inlined enter() into main
 * (onboard_update is inlined there) and the FCU's tick start -> last vane write grew by ~20 us in the co-simulation
 * (tools/run_cosim.py, every tick); out of line the timing is the previous image's again (-1.6 us median) */
#if defined(__GNUC__)
#define ONB_NOINLINE __attribute__((noinline))
#else
#define ONB_NOINLINE
#endif
ONB_NOINLINE static void enter(onboard_t *s, uint8_t new_state, uint32_t t, uint8_t cause)
{
    log_trans(s, new_state, cause);
    s->state = new_state;
    s->t_entry = t;
    if (new_state != ONB_POWERED)
        s->target_milli = 0;
    if (new_state == ONB_UNPOWERED) {
        s->has_unpowered = true;
        s->t_unpowered = t;
    }
    if (new_state == ONB_SAFE)
        s->has_release_deadline = false;              /* a release pending on the pad is cancelled */
}


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

static void start_release(onboard_t *s, uint32_t t, uint8_t cause)
{
    s->release_deadline = t + PRM(OPAR(s)->release_after_cut_ticks);
    s->has_release_deadline = true;
    s->target_milli = 0;
    if (s->state == ONB_POWERED)
        enter(s, ONB_UNPOWERED, t, cause);
}

static void kill(onboard_t *s, uint32_t t, uint8_t cause)
{
    if (s->state == ONB_ARMED)
        enter(s, ONB_SAFE, t, cause);
    else if (flight(s->state))
        do_abort(s, t, cause);
}


/* ---- commands ---------------------------------------------------------------------------- */
/* the ARM gates in their order: 0 or the LP_REJ_* code of the first that fails */
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
    if (!(in->tilt_deg < PRM(p->arm_tilt_max_deg)))
        return LP_REJ_TILT;
    if (!(in->v_cell >= PRM(p->min_cell_v)))
        return LP_REJ_VCELL;
    if (!link_ok_before)
        return LP_REJ_LINK;
    return 0;
}

static uint8_t execute_ballistic(onboard_t *s, const onb_frame_t *f, uint32_t t, const onb_inputs_t *in,
                                 bool link_ok_before)
{
    const uint8_t c = f->cmd.cmd, st = s->state, cause = (uint8_t)(ONB_CAUSE_CMD | (c & 0x7Fu));
    const onb_params_t *p = OPAR(s);
    if (c > LP_CMD_MAX_BALLISTIC || c == LP_CMD_NONE)
        return LP_REJ_UNKNOWN_CMD;                    /* 8..11 included, exactly as v0.6.3 */
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
    /* 1. received commands */
    for (i = 0; i < in->n_frames; i++) {
        const onb_frame_t *f = &in->frames[i];
        const bool link_ok_before = onboard_link_ok(s, t);
        uint8_t code, before;
        s->has_frame = true;
        s->last_frame_tick = t;
        out->reply = true;
        out->reply_seq = f->seq;
        before = s->state;
        if (s->has_seq && !lp_seq_newer(f->seq, s->last_seq)) {
            code = ONB_EXEC_REPEAT;                   /* repeat of a processed SEQ (or older): reply only */
        } else {
            s->has_seq = true;
            s->last_seq = f->seq;
            if (f->cmd.cmd == LP_CMD_NONE) {
                s->ack_rejected = false;
                code = ONB_EXEC_NONE;
            } else {
                code = execute_ballistic(s, f, t, in, link_ok_before);
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
        }
    }
    /* 2. interlocks / safety rules */
    interlocks_ballistic(s, t, in);
    /* 3. release timer (pending release or ABORT) */
    if (s->has_release_deadline && t >= s->release_deadline) {
        s->has_release_deadline = false;
        s->release_open = true;
        enter(s, ONB_RELEASED, t, ONB_CAUSE_RELEASE_TIMER);
    }
    /* 4. throttle slew; then the I7 clock: POWERED ticks since boot */
    slew(s);
    if (s->state == ONB_POWERED)
        s->powered_ticks++;
    out->state = s->state;
    out->target_milli = s->target_milli;
    out->throttle_milli = s->out_milli;
    out->control_active = onboard_control_active(s, t);
    out->release = s->release_open;
    out->reason = s->reason;
    out->flags = flags(s, t, in);
    out->ack_seq = s->has_seq ? s->last_seq : 0u;
    out->nonce = s->nonce;

    if (out->reply) {
        onb_tlm_src_t *m = &out->tlm;
        m->state = out->state;
        m->flags = out->flags;
        m->reason = out->reason;
        m->ack_seq = out->ack_seq;
        m->nonce = out->nonce;
        m->throttle_milli = out->throttle_milli;      /* TELEM v1: the slewed throttle */
        m->tick = t;
        m->h_m = in->h_m;
        m->vz_mps = in->vz_mps;
        m->roll_deg = in->roll_deg;
        m->pitch_deg = in->pitch_deg;
        m->yaw_deg = in->yaw_deg;
        m->tilt_deg = in->tilt_deg;
        m->v_cell = in->v_cell;
        m->loop_max_us = in->loop_max_us;
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
    return lp_encode_telem(veh, seq, &t, out);
}
