/* position_ctrl.c - see position_ctrl.h (mirrors sim/position_ctrl.py PositionHold.step branch by branch, split into the
 * pieces P0..P10; float32, the reference's operand order except the stated reciprocals). */
#include "position_ctrl.h"

#if POS_PARAMS_FLASH
/* LPM reads of the flash table (in the lower 64 KB like avr-libc's pgm_read_*: .progmem sits right after the vectors);
 * inline asm instead of <avr/pgmspace.h> - the core includes no AVR header */
static uint8_t flash_u8(const void *a)
{
    uint8_t r;
    __asm__ __volatile__("lpm %0, Z" : "=r"(r) : "z"(a));
    return r;
}

static uint16_t flash_u16(const void *a)
{
    uint16_t r;
    __asm__ __volatile__("lpm %A0, Z+\n\tlpm %B0, Z" : "=r"(r), "+z"(a));
    return r;
}

static float flash_f32(const void *a)
{
    float r;
    __asm__ __volatile__("lpm %A0, Z+\n\tlpm %B0, Z+\n\tlpm %C0, Z+\n\tlpm %D0, Z" : "=r"(r), "+z"(a));
    return r;
}

void pos_params_from_flash(pos_params_t *dst, const pos_params_t *flash)
{
    uint8_t *d = (uint8_t *)dst;
    const uint8_t *a = (const uint8_t *)flash;
    uint8_t i;
    for (i = 0; i < (uint8_t)sizeof *dst; i++)
        d[i] = flash_u8(a + i);
}

#define PF(f) flash_f32(&s->p->f)
#define PW(f) flash_u16(&s->p->f)
#define PB(f) flash_u8(&s->p->f)
#else
#define PF(f) (s->p.f)
#define PW(f) (s->p.f)
#define PB(f) (s->p.f)
#endif

#define CM_TO_M 0.01f                                  /* the slots' cm / 100 (the reference divides in float64) */
#define Q15_TO_1 (1.0f / 32767.0f)
#define POS_K_FULL 0u                                  /* a step with data: every piece */
#define POS_K_HOLD 1u                                  /* a hold step (no data): P1 writes its outputs */
#define POS_K_DONE 2u                                  /* nothing more to compute: the commit piece publishes */

static bool fly_phase3(uint8_t ph)                     /* CLIMB / HOLD / LAND (FLY_PHASES of the reference) */
{
    return ph == LP_PHASE_CLIMB || ph == LP_PHASE_HOLD || ph == LP_PHASE_LAND;
}

void pos_params_derive(pos_params_t *p)
{
    p->g2 = p->g * p->g;
    p->inv_t = 1.0f / p->T;
    p->ki_t = p->ki * p->T;
    p->i_max2 = p->i_max * p->i_max;
    p->fence2 = p->fence_m * p->fence_m;
    p->plaus_vt2 = p->plaus_vt * p->plaus_vt;
    p->inv_fade = 1.0f / p->tilt_fade_m;
    p->inv_g = 1.0f / p->g;
    p->inv_g2 = 1.0f / p->g2;
}

void pos_pub_off(pos_pub_t *pub)
{
    pub->mode = POS_MODE_OFF;
    pub->requests = 0;
    pub->engaged = false;
    pub->tilt_l[0] = pub->tilt_l[1] = 0.0f;
    pub->dist_m = NAN;
    pub->trim_l[0] = pub->trim_l[1] = 0.0f;
    pub->has_hold = false;
    pub->hold[0] = pub->hold[1] = 0.0f;
}

#if POS_FULL_OUT
static void out_off(pos_outputs_t *o)                  /* OFF_OUT */
{
    o->mode = POS_MODE_OFF;
    o->requests = 0;
    o->limited = o->guard = o->engaged = o->has_hold = false;
    o->lost_steps = 0;
    o->tilt_l[0] = o->tilt_l[1] = o->tilt_w[0] = o->tilt_w[1] = 0.0f;
    o->a_cmd[0] = o->a_cmd[1] = o->v_sp[0] = o->v_sp[1] = 0.0f;
    o->hold[0] = o->hold[1] = 0.0f;
    o->dist_m = NAN;
    o->cos_lim = 1.0f;
    o->a_lim2 = 0.0f;
    o->integ[0] = o->integ[1] = 0.0f;
    o->trim_l[0] = o->trim_l[1] = 0.0f;
}
#endif

void pos_reset(pos_t *s)
{
    s->engaged = false;
    s->has_hold = false;
    s->hold_rf[0] = s->hold_rf[1] = 0.0f;
    s->integ[0] = s->integ[1] = 0.0f;
    s->dv[0] = s->dv[1] = 0.0f;
    s->has_vprev = false;
    s->v_prev[0] = s->v_prev[1] = 0.0f;
    s->a_prev[0] = s->a_prev[1] = 0.0f;
    s->limited_prev = false;
    s->lost = 0;
    s->guard_left = 0;
    s->lim_run = 0;
    s->lim_v2_start = 0.0f;
    s->vexc_run = 0;
    s->n_stamps = 0;
    s->has_lrc = false;
    s->last_reset_count = 0;
    s->implausible = false;
    s->requests = 0;
    pos_pub_off(&s->last);
#if POS_FULL_OUT
    out_off(&s->out);
#endif
}

void pos_init(pos_t *s, const pos_params_t *p)
{
#if POS_PARAMS_FLASH
    s->p = p;
#else
    s->p = *p;
    pos_params_derive(&s->p);
#endif
    s->n = 0;
    s->cache_ok = false;
    s->cache_low = false;
    s->cache_kt = s->cache_c = s->cache_cg = s->cache_a2 = 0.0f;
    s->job = 0;
    s->w.kind = POS_K_DONE;
    pos_reset(s);
}

uint8_t pos_step_ticks(const pos_t *s)
{
    return PB(step_ticks);
}

float pos_slew_per_tick(const pos_t *s)
{
    return PF(slew_per_tick);
}

uint16_t pos_land_steps(const pos_t *s)
{
    return PW(land_steps);
}

/* the three numbers the v0.8.1 FCU (the GNS chain) reads from the table (flash on the AVR) */
#if POS_PARAMS_FLASH
float pos_params_slew_per_tick(const pos_params_t *pp) { return flash_f32(&pp->slew_per_tick); }
uint16_t pos_params_land_steps(const pos_params_t *pp) { return flash_u16(&pp->land_steps); }
uint8_t pos_params_step_ticks(const pos_params_t *pp) { return flash_u8(&pp->step_ticks); }
#else
float pos_params_slew_per_tick(const pos_params_t *pp) { return pp->slew_per_tick; }
uint16_t pos_params_land_steps(const pos_params_t *pp) { return pp->land_steps; }
uint8_t pos_params_step_ticks(const pos_params_t *pp) { return pp->step_ticks; }
#endif

void pos_tilt_limit(const pos_t *s, float k_t, bool low_cap, float *cos_lim, float *sin_lim, float *a_lim2)
{
    const float x = fc_maxf(k_t, 1e-3f) * PF(tw);
    const float m = fc_minf(PF(margin_g), PF(margin_frac) * fc_maxf(x - 1.0f, 0.0f));
    const float r = (1.0f + m) / x;
    const float cc = low_cap ? PF(cos_flare_cap) : PF(cos_cap);
    if (r >= 1.0f) {
        *cos_lim = 1.0f;
        *sin_lim = 0.0f;
        *a_lim2 = 0.0f;
    } else {
        const float c = r > cc ? r : cc;
        const float s2 = 1.0f - c * c;
        *cos_lim = c;
        *sin_lim = sqrtf(s2);
        *a_lim2 = PF(g2) * s2 / (c * c);
    }
}

void pos_tilt_of_accel(float ax, float ay, float inv_g, float inv_g2, float tw[2])
{
    const float sq = (ax * ax + ay * ay) * inv_g2;
    const float k = (1.0f - sq * (1.0f / 3.0f) + sq * sq * (1.0f / 5.0f)) * inv_g;
    tw[0] = -ay * k;
    tw[1] = ax * k;
}

void pos_data_from_set(pos_data_t *d, const nl_gps_set_t *set)
{
    uint8_t i;
    d->flags = set->flags;
    d->reset_count = set->reset_count;
    for (i = 0; i < 2u; i++) {
        d->prf[i] = (float)((int32_t)set->p[i] - (int32_t)set->off[i]) * CM_TO_M;   /* exact integer cm, one rounding */
        d->off[i] = (float)set->off[i] * CM_TO_M;
        d->v[i] = (float)set->v[i] * CM_TO_M;
        d->cs[i] = (float)set->cs[i] * Q15_TO_1;
    }
}

void pos_latch(pos_t *s, const pos_inputs_t *in)
{
    pos_work_t *w = &s->w;
    w->phase = in->phase;
    w->motor_on = in->motor_on;
    w->sat_high = in->vc_sat_high;
    w->h = in->h_m;
    w->u.i.cos_tilt = in->cos_tilt;
    w->u.i.k_t = in->k_t;
    w->u.i.rate[0] = in->rate[0];
    w->u.i.rate[1] = in->rate[1];
    w->u.i.rate[2] = in->rate[2];
    w->has_data = in->data != 0 || in->set != 0;
    if (in->data != 0)
        w->d = *in->data;
    else if (in->set != 0)
        pos_data_from_set(&w->d, in->set);
    w->low_alt = in->vc_low_alt;
    w->ground = in->vc_ground;
    w->lifted = in->lifted;
    s->job = 1;
}

/* P0 (J0): the step counter, OFF / engage, loss counter, the tilt limit's first part, the data-free requests */
static void piece0(pos_t *s)
{
    pos_work_t *w = &s->w;
    const float h = w->h;
    s->n++;
    w->kind = POS_K_DONE;
    if (!(w->motor_on && fly_phase3(w->phase))) {      /* 0. not flying: reset, tilt target 0 */
        if (s->engaged)
            pos_reset(s);
        return;
    }
    if (!s->engaged) {                                 /* 1. engage above h_enable once the lift-off is confirmed */
        if (h == h && h > PF(h_enable_m) && w->lifted) {
            s->engaged = true;
            if (w->has_data) {
                s->has_lrc = true;
                s->last_reset_count = w->d.reset_count;
            }
        } else {
            return;
        }
    }
    w->was_lost = s->lost >= PW(timeout_steps);        /* the GPS had been declared lost (attitude only) */
    w->ok = w->has_data && (w->d.flags & POS_GF_FIX_OK) != 0u && !s->implausible;
    s->lost = w->ok ? 0u : (s->lost < 0xFFFFu ? (uint16_t)(s->lost + 1u) : s->lost);
    w->land = w->phase == LP_PHASE_LAND;
    w->low = w->land && h == h && h < PF(flare_altitude_m);
    if (s->cache_ok && s->cache_low == w->low && s->cache_kt == w->u.i.k_t) {   /* NaN k_T never hits the cache */
        w->cos_lim = s->cache_c;
        w->cos_g = s->cache_cg;
        w->a_lim2 = s->cache_a2;
        w->lim_b = false;
    } else {                                           /* tilt_limit(k_T, low): x, m, r, cos_lim here, the rest in P1 */
        const float x = fc_maxf(w->u.i.k_t, 1e-3f) * PF(tw);
        const float m = fc_minf(PF(margin_g), PF(margin_frac) * fc_maxf(x - 1.0f, 0.0f));
        const float r = (1.0f + m) / x;
        const float cc = w->low ? PF(cos_flare_cap) : PF(cos_cap);
        s->cache_ok = false;
        if (r >= 1.0f) {                               /* (1, 0, 0): cos(lim + excess) = cos_excess exactly */
            w->cos_lim = 1.0f;
            w->a_lim2 = 0.0f;
            w->cos_g = PF(cos_excess);
            w->lim_b = false;
            s->cache_c = 1.0f;
            s->cache_cg = w->cos_g;
            s->cache_a2 = 0.0f;
            s->cache_ok = true;
            s->cache_low = w->low;
            s->cache_kt = w->u.i.k_t;
        } else {
            w->cos_lim = r > cc ? r : cc;
            w->lim_b = true;
        }
    }
    if (s->lost >= PW(land_steps))
        s->requests |= POS_REQ_GPS_LOST;
    if (w->u.i.k_t * PF(tw) < PF(authority_min_ktw))
        s->requests |= POS_REQ_AUTHORITY;
    w->kind = w->has_data ? POS_K_FULL : POS_K_HOLD;
}

/* P1 (J0): the tilt limit's second part (sin, a_lim^2, cos(lim + excess)), the guard, a hold step's outputs */
static void piece1(pos_t *s)
{
    pos_work_t *w = &s->w;
    float wx, wy, wz;
    bool trig;
    if (w->kind == POS_K_DONE)
        return;
    if (w->lim_b) {
        const float c = w->cos_lim;
        const float s2 = 1.0f - c * c;
        const float sn = sqrtf(s2);
        w->a_lim2 = PF(g2) * s2 / (c * c);
        w->cos_g = c * PF(cos_excess) - sn * PF(sin_excess);   /* cos(lim + excess) */
        s->cache_c = c;
        s->cache_cg = w->cos_g;
        s->cache_a2 = w->a_lim2;
        s->cache_ok = true;
        s->cache_low = w->low;
        s->cache_kt = w->u.i.k_t;
    }
    wx = w->u.i.rate[0];
    wy = w->u.i.rate[1];
    wz = w->u.i.rate[2];
    trig = fabsf(wz) > PF(spin_z_rad) || wx * wx + wy * wy > PF(spin_xy_rad2) || w->u.i.cos_tilt < w->cos_g;
    w->trig = trig;
    if (trig) {
        const uint16_t gs = PW(guard_steps);
        s->guard_left = (uint8_t)(gs < 255u ? gs : 255u);
    } else if (s->guard_left > 0u) {
        s->guard_left--;
    }
    w->guard = trig || s->guard_left > 0u;
    if (w->kind == POS_K_HOLD) {                       /* hold step: outputs unchanged but requests / guard / lost */
        s->last.requests = s->requests;
#if POS_FULL_OUT
        s->out.requests = s->requests;
        s->out.guard = w->guard;
        s->out.lost_steps = s->lost;
#endif
        w->kind = POS_K_DONE;
    }
}

/* P2 (J1): NAV resets (plausibility (c)), the reset-free position, hold point, mode, fade and saturation limits */
static void piece2(pos_t *s)
{
    pos_work_t *w = &s->w;
    const pos_data_t *d = &w->d;
    const float h = w->h;
    const uint8_t nres = PB(plaus_resets);
    uint8_t i;
    if (!s->has_lrc) {
        s->has_lrc = true;
        s->last_reset_count = d->reset_count;
    } else if (d->reset_count != s->last_reset_count) {
        uint8_t nr = (uint8_t)(d->reset_count - s->last_reset_count);   /* & 0xFF */
        if (nr > nres)
            nr = nres;                                 /* only the newest plaus_resets stamps can matter */
        while (nr--) {
            if (s->n_stamps >= nres) {                 /* drop the oldest */
                for (i = 1; i < s->n_stamps; i++)
                    s->reset_stamp[i - 1u] = s->reset_stamp[i];
                s->n_stamps--;
            }
            s->reset_stamp[s->n_stamps++] = s->n;
        }
        s->last_reset_count = d->reset_count;
    }
    /* len([x in hist if n - x < window]) >= plaus_resets <=> the plaus_resets-th newest stamp is inside the window
     * (uint16 differences: a flight is far shorter than 65536 steps, and pos_reset clears the stamps on the ground) */
    if (!w->ground) {                                  /* no rule latches on the ground (the vertical controller's flags) */
        if (nres > 0u && s->n_stamps >= nres &&
            (uint16_t)(s->n - s->reset_stamp[s->n_stamps - nres]) < PW(plaus_reset_window_steps))
            s->implausible = true;
        else if (nres == 0u)
            s->implausible = true;
    }
    if (s->implausible) {
        s->requests |= POS_REQ_IMPLAUSIBLE;
        w->ok = false;
    }
    if (w->ok && w->land && w->was_lost && s->has_hold) {
        s->hold_rf[0] = d->prf[0];                     /* the GPS is back during a LAND: land where it is */
        s->hold_rf[1] = d->prf[1];
    }
    if (w->ok && !s->has_hold && (w->phase == LP_PHASE_HOLD || w->phase == LP_PHASE_LAND)) {
        s->hold_rf[0] = d->prf[0];                     /* the reset-free frame p - offset */
        s->hold_rf[1] = d->prf[1];
        s->has_hold = true;
    }
    if (s->implausible || (!w->ok && s->lost >= PW(timeout_steps)))
        w->mode = POS_MODE_ATT;
    else if (!w->ok)
        w->mode = POS_MODE_DR;
    else if (s->has_hold)
        w->mode = POS_MODE_POS;
    else
        w->mode = POS_MODE_VEL;
    w->f_tr = 1.0f;                                    /* keep_trim: the transient's limit factor (1: none) */
    w->fade_band = false;                              /* v0.9 keep_vel: the velocity hold in the flare band */
    if (w->land) {                                     /* the touchdown rule on the vertical controller's flags */
        if (w->ground) {
            w->a_lim2 = 0.0f;                          /* on the ground: upright */
        } else if (w->low_alt && h == h && h < PF(tilt_fade_m)) {
            float f;
            if (h >= 0.0f)
                f = h * PF(inv_fade);
            else                                       /* below the pad level: lower ground - the hold comes back */
                f = fc_minf(fc_maxf(-h - PF(tilt_fade_m), 0.0f) * PF(inv_fade), 1.0f);
            if (PB(keep_trim) == POS_TD_KEEP_TRIM)
                w->f_tr = f;                           /* v0.8.1 keep_trim: only the transient (P8) */
            else if (PB(keep_trim) == POS_TD_FADE)
                w->a_lim2 *= f * f;                    /* fade (v0.8): the whole tilt */
        }
        if (PB(keep_trim) == POS_TD_KEEP_VEL && !w->ground && w->low_alt)
            w->fade_band = true;                       /* v0.9 keep_vel: v_sp = 0, full authority to the contact (P4) */
    }
    if (w->sat_high) {                                 /* no new demand while the vertical loop is saturated high */
        const float ap2 = s->a_prev[0] * s->a_prev[0] + s->a_prev[1] * s->a_prev[1];
        w->a_lim2 = fc_minf(w->a_lim2, ap2);
    }
}

/* P3 (J1): distance to the hold point, v_sp and its limit's magnitude */
static void piece3(pos_t *s)
{
    pos_work_t *w = &s->w;
    w->vs[0] = w->vs[1] = 0.0f;
    w->dist = NAN;
    if (s->has_hold) {
        const float ex = s->hold_rf[0] - w->d.prf[0], ey = s->hold_rf[1] - w->d.prf[1];
        w->u.h.dist2 = ex * ex + ey * ey;
        w->dist = sqrtf(w->u.h.dist2);
        if (w->mode == POS_MODE_POS) {
            float vl;
            w->vs[0] = PF(kp_pos) * ex;
            w->vs[1] = PF(kp_pos) * ey;
            vl = w->land ? PF(v_max_land) : PF(v_max);
            if (w->low)
                vl = fc_minf(vl, PF(flare_speed));
            w->u.h.vl = vl;
            w->u.h.n2 = w->vs[0] * w->vs[0] + w->vs[1] * w->vs[1];
        }
    }
}

/* P4 (J2): the v_sp limit, the fence; whether a limited step may count for plausibility (a) (physics-review fix: CLIMB /
 * HOLD above the flare altitude, the vertical loop not saturated high, the NAV's GPS-IMU inconsistency flag) */
static void piece4(pos_t *s)
{
    pos_work_t *w = &s->w;
    const float h = w->h;
    w->a_ok = !w->land && h == h && h > PF(flare_altitude_m) && !w->sat_high && (w->d.flags & POS_GF_INCONS) != 0u;
    if (s->has_hold && w->mode == POS_MODE_POS) {
        const float vl = w->u.h.vl;
        if (w->fade_band) {                            /* v0.9 keep_vel: stop over the ground, no return to the hold */
            w->vs[0] = w->vs[1] = 0.0f;                /* point (the reference zeroes after the limit: same result, */
        } else if (w->u.h.n2 > vl * vl) {              /* no sqrt) */
            const float sc = vl / sqrtf(w->u.h.n2);
            w->vs[0] *= sc;
            w->vs[1] *= sc;
        }
        if ((w->phase == LP_PHASE_CLIMB || w->phase == LP_PHASE_HOLD) && w->u.h.dist2 > PF(fence2))
            s->requests |= POS_REQ_FENCE;
    }
}

/* P5 (J2): the D term (step 7) */
static void piece5(pos_t *s)
{
    pos_work_t *w = &s->w;
    const float vx = w->d.v[0], vy = w->d.v[1];
    if (s->has_vprev) {
        const float ad = PF(alpha_d), it = PF(inv_t);
        s->dv[0] += ad * ((vx - s->v_prev[0]) * it - s->dv[0]);
        s->dv[1] += ad * ((vy - s->v_prev[1]) * it - s->dv[1]);
    }
    s->has_vprev = true;
    s->v_prev[0] = vx;
    s->v_prev[1] = vy;
}

/* P6 (J2): the velocity error and the integrator candidate (step 8) */
static void piece6(pos_t *s)
{
    pos_work_t *w = &s->w;
    const float ix = s->integ[0], iy = s->integ[1];
    float tx, ty;
    if (w->mode == POS_MODE_ATT || w->guard) {
        w->a[0] = ix;
        w->a[1] = iy;
        w->vl_skip = true;
        return;
    }
    w->vl_skip = false;
    w->u.v.ev[0] = w->vs[0] - w->d.v[0];
    w->u.v.ev[1] = w->vs[1] - w->d.v[1];
    if (w->sat_high) {
        tx = ix;
        ty = iy;
    } else {
        float ni2;
        tx = ix + PF(ki_t) * w->u.v.ev[0];
        ty = iy + PF(ki_t) * w->u.v.ev[1];
        ni2 = tx * tx + ty * ty;
        if (ni2 > PF(i_max2)) {
            const float sc = PF(i_max) / sqrtf(ni2);
            tx *= sc;
            ty *= sc;
        }
    }
    w->u.v.t[0] = tx;
    w->u.v.t[1] = ty;
}

/* P7 (J3): a_cmd, conditional integration */
static void piece7(pos_t *s)
{
    pos_work_t *w = &s->w;
    const float ix = s->integ[0], iy = s->integ[1];
    const float evx = w->u.v.ev[0], evy = w->u.v.ev[1];
    const float kv = PF(kv), kd = PF(kd);
    float tx = w->u.v.t[0], ty = w->u.v.t[1], ax, ay;
    if (w->vl_skip)
        return;
    ax = kv * evx + tx - kd * s->dv[0];
    ay = kv * evy + ty - kd * s->dv[1];
    if (ax * ax + ay * ay > w->a_lim2 && s->limited_prev && evx * ix + evy * iy > 0.0f) {
        tx = ix;                                       /* conditional integration: the limit already holds it back */
        ty = iy;
        ax = kv * evx + tx - kd * s->dv[0];
        ay = kv * evy + ty - kd * s->dv[1];
    }
    s->integ[0] = tx;
    s->integ[1] = ty;
    w->a[0] = ax;
    w->a[1] = ay;
}

/* P8 (J3): the a limit, plausibility (a) / (b) */
static void piece8(pos_t *s)
{
    pos_work_t *w = &s->w;
    float na2;
    if (!w->vl_skip && w->f_tr < 1.0f) {               /* keep_trim: only the transient above the trim is limited */
        const float tx = s->integ[0], ty = s->integ[1];
        const float le2 = w->f_tr * w->f_tr * w->a_lim2;
        float ex = w->a[0] - tx, ey = w->a[1] - ty;
        const float ne2 = ex * ex + ey * ey;
        if (ne2 > le2) {
            const float sc = le2 > 0.0f ? sqrtf(le2 / ne2) : 0.0f;
            ex *= sc;
            ey *= sc;
        }
        w->a[0] = tx + ex;
        w->a[1] = ty + ey;
    }
    na2 = w->a[0] * w->a[0] + w->a[1] * w->a[1];
    w->limited = na2 > w->a_lim2;
    if (w->limited) {
        const float sc = w->a_lim2 > 0.0f ? sqrtf(w->a_lim2 / na2) : 0.0f;
        w->a[0] *= sc;
        w->a[1] *= sc;
    }
    if (w->mode == POS_MODE_POS || w->mode == POS_MODE_VEL) {
        const float v2 = w->d.v[0] * w->d.v[0] + w->d.v[1] * w->d.v[1];
        if (w->limited && w->a_ok) {                   /* (a): only an inconsistent GPS at the authority limit (P4) */
            if (s->lim_run < 0xFFFFu)
                s->lim_run++;
            if (s->lim_run == 1u)
                s->lim_v2_start = v2;
            if (s->lim_run >= PW(plaus_limited_steps) && v2 >= s->lim_v2_start)
                s->implausible = true;
        } else {
            s->lim_run = 0;
        }
        if (v2 > PF(plaus_vt2)) {
            if (s->vexc_run < 0xFFFFu)
                s->vexc_run++;
            if (s->vexc_run >= PW(plaus_vexc_steps) && !w->ground)
                s->implausible = true;
        } else {
            s->vexc_run = 0;
        }
        if (s->implausible)
            s->requests |= POS_REQ_IMPLAUSIBLE;
    } else {
        s->lim_run = 0;
        s->vexc_run = 0;
    }
}

/* P9 (J4): the tilt series (step 10) and the v0.8.1 trim (the integrator limited to a_lim, the same series) */
static void piece9(pos_t *s)
{
    pos_work_t *w = &s->w;
    float rix = s->integ[0], riy = s->integ[1];
    const float ni2 = rix * rix + riy * riy;
    pos_tilt_of_accel(w->a[0], w->a[1], PF(inv_g), PF(inv_g2), w->u.s.tw);
    if (ni2 > w->a_lim2) {
        const float sc = w->a_lim2 > 0.0f ? sqrtf(w->a_lim2 / ni2) : 0.0f;
        rix *= sc;
        riy *= sc;
    }
    pos_tilt_of_accel(rix, riy, PF(inv_g), PF(inv_g2), w->u.s.rw);
}

/* P10 (J4): the rotation into the level frame, COMMIT */
static void piece10(pos_t *s)
{
    pos_work_t *w = &s->w;
    const float c = w->d.cs[0], sn = w->d.cs[1];
    const float tx = w->u.s.tw[0], ty = w->u.s.tw[1], rx = w->u.s.rw[0], ry = w->u.s.rw[1];
    pos_pub_t *o = &s->last;
    s->a_prev[0] = w->a[0];
    s->a_prev[1] = w->a[1];
    s->limited_prev = w->limited;
    o->mode = w->mode;
    o->requests = s->requests;
    o->engaged = true;
    o->tilt_l[0] = c * tx + sn * ty;
    o->tilt_l[1] = -sn * tx + c * ty;
    o->dist_m = w->dist;
    o->trim_l[0] = c * rx + sn * ry;
    o->trim_l[1] = -sn * rx + c * ry;
    o->has_hold = s->has_hold;
    if (s->has_hold) {
        o->hold[0] = s->hold_rf[0] + w->d.off[0];
        o->hold[1] = s->hold_rf[1] + w->d.off[1];
    } else {
        o->hold[0] = o->hold[1] = 0.0f;
    }
#if POS_FULL_OUT
    {
        pos_outputs_t *f = &s->out;
        f->mode = w->mode;
        f->tilt_w[0] = tx;
        f->tilt_w[1] = ty;
        f->tilt_l[0] = o->tilt_l[0];
        f->tilt_l[1] = o->tilt_l[1];
        f->a_cmd[0] = w->a[0];
        f->a_cmd[1] = w->a[1];
        f->v_sp[0] = w->vs[0];
        f->v_sp[1] = w->vs[1];
        f->has_hold = s->has_hold;
        f->hold[0] = o->hold[0];
        f->hold[1] = o->hold[1];
        f->trim_l[0] = o->trim_l[0];
        f->trim_l[1] = o->trim_l[1];
        f->dist_m = w->dist;
        f->requests = s->requests;
        f->limited = w->limited;
        f->guard = w->guard;
        f->engaged = true;
        f->cos_lim = w->cos_lim;
        f->a_lim2 = w->a_lim2;
        f->lost_steps = s->lost;
        f->integ[0] = s->integ[0];
        f->integ[1] = s->integ[1];
    }
#endif
}

uint8_t pos_piece_tick(uint8_t piece)
{
    return piece < 2u ? 0u : piece < 4u ? 1u : piece < 7u ? 2u : piece < 9u ? 3u : 4u;
}

bool pos_piece_due(const pos_t *s, uint8_t off)
{
    return s->job != 0u && pos_piece_tick((uint8_t)(s->job - 1u)) <= off;
}

bool pos_piece(pos_t *s, pos_pub_t *dst)
{
    uint8_t i;
    if (s->job == 0u)
        return false;
    i = (uint8_t)(s->job - 1u);
    if (i == 0u) {
        piece0(s);
    } else if (i == 1u) {
        piece1(s);
    } else if (s->w.kind == POS_K_FULL) {
        switch (i) {
        case 2: piece2(s); break;
        case 3: piece3(s); break;
        case 4: piece4(s); break;
        case 5: piece5(s); break;
        case 6: piece6(s); break;
        case 7: piece7(s); break;
        case 8: piece8(s); break;
        case 9: piece9(s); break;
        default: piece10(s); break;
        }
    }
    if (i >= POS_PIECES - 1u) {
        s->job = 0;
        if (dst != 0)
            *dst = s->last;
        return true;
    }
    s->job = (uint8_t)(i + 2u);
    return false;
}

bool pos_finish(pos_t *s, pos_pub_t *dst)
{
    bool done = false;
    while (s->job != 0u)
        done = pos_piece(s, dst);
    return done;
}

const pos_pub_t *pos_step(pos_t *s, const pos_inputs_t *in)
{
    (void)pos_finish(s, 0);
    pos_latch(s, in);
    (void)pos_finish(s, 0);
    return &s->last;
}

/* m hold steps: the first through the pieces; the other m - 1 see the same inputs, so (sim PositionHold.step without data):
 * not flying / not engaging -> nothing but the counter; engaged -> lost += m - 1 (saturating), the guard re-triggers on the
 * same rates (or counts down m - 1), the requests GPS_LOST (lost >= land_steps; AUTHORITY: the first step's) and the hold
 * step's outputs */
const pos_pub_t *pos_hold_steps(pos_t *s, const pos_inputs_t *in, uint16_t m)
{
    pos_inputs_t h = *in;
    pos_work_t *w = &s->w;
    uint16_t r;
    if (m == 0u)
        return &s->last;
    h.data = 0;
    h.set = 0;
    (void)pos_step(s, &h);
    r = (uint16_t)(m - 1u);
    s->n = (uint16_t)(s->n + r);
    if (r == 0u || !(w->motor_on && fly_phase3(w->phase)) || !s->engaged)
        return &s->last;
    s->lost = (uint16_t)(s->lost > (uint16_t)(0xFFFFu - r) ? 0xFFFFu : s->lost + r);
    if (!w->trig)
        s->guard_left = (uint8_t)(s->guard_left > r ? s->guard_left - r : 0u);
    w->guard = w->trig || s->guard_left > 0u;
    if (s->lost >= PW(land_steps))
        s->requests |= POS_REQ_GPS_LOST;
    s->last.requests = s->requests;
#if POS_FULL_OUT
    s->out.requests = s->requests;
    s->out.guard = w->guard;
    s->out.lost_steps = s->lost;
#endif
    return &s->last;
}

void pos_tilt_tick(float t[2], const float target[2], bool active, float slew_per_tick)
{
    float dx, dy, n2;
    if (!active) {
        t[0] = t[1] = 0.0f;
        return;
    }
    dx = target[0] - t[0];
    dy = target[1] - t[1];
    n2 = dx * dx + dy * dy;
    if (n2 > slew_per_tick * slew_per_tick) {
        const float sc = slew_per_tick / sqrtf(n2);
        dx *= sc;
        dy *= sc;
    }
    t[0] += dx;
    t[1] += dy;
}
