/* gps_nav.c - see gps_nav.h (mirrors sim/gps_nav.py NavGpsFilter / LocalFrame / NavGps branch by branch, float32;
 * every expression keeps the reference's operand order so the rounding differs only by float32 vs float64). */
#include "gps_nav.h"

#include "link_proto.h"

#define WGS84_A 6378137.0f
#define WGS84_E2 6.69437999014e-3f              /* f (2 - f), f = 1 / 298.257223563 */
#define RAD_PER_E7 1.745329251994329577e-9f     /* radians(1e-7 deg) */

/* ---- local frame ---- */
void gps_frame_at(gps_frame_t *fr, int32_t lat0_e7, int32_t lon0_e7)
{
    const float phi = (float)lat0_e7 * RAD_PER_E7;
    const float s = sinf(phi);
    const float w = 1.0f - WGS84_E2 * s * s;
    const float sw = sqrtf(w);
    const float m_rad = WGS84_A * (1.0f - WGS84_E2) / (w * sw);
    const float n_rad = WGS84_A / sw;
    fr->lat0 = lat0_e7;
    fr->lon0 = lon0_e7;
    fr->k_n = RAD_PER_E7 * m_rad;
    fr->k_e = RAD_PER_E7 * n_rad * cosf(phi);
}

void gps_frame_to_w(const gps_frame_t *fr, int32_t lat_e7, int32_t lon_e7, float *x, float *y)
{
    const int64_t dlat = (int64_t)lat_e7 - (int64_t)fr->lat0;   /* exact (Python int) */
    int64_t dlon = (int64_t)lon_e7 - (int64_t)fr->lon0 + 1800000000LL;
    while (dlon < 0)                                   /* Python ((d + 1.8e9) % 3.6e9) - 1.8e9: |d| < 2^32, so at most */
        dlon += 3600000000LL;                          /* two corrections either way */
    while (dlon >= 3600000000LL)
        dlon -= 3600000000LL;
    dlon -= 1800000000LL;
    *x = (float)dlat * fr->k_n;
    *y = -((float)dlon * fr->k_e);
}

/* ---- the filter ---- */
static void clear_buf(gps_filter_t *f)
{
    uint8_t i;
    for (i = 0; i < GPS_BUF_N; i++)
        f->buf[i].m = -1;                              /* the reference clears only m (the values stay) */
}

void gps_filter_init(gps_filter_t *f, const gps_nav_params_t *p)
{
    uint8_t i;
    f->p = *p;
    for (i = 0; i < GPS_BUF_N; i++) {                  /* the age terms of the reference ((s - s_e) t_step, 0.5 ae ae) */
        const float t = (float)i * p->t_step;
        f->tab_t[i] = t;
        f->tab_h[i] = 0.5f * t * t;
    }
    f->jc = 0;
    f->kq = 0;
    f->started = false;
    f->k = 0;
    f->acc[0] = f->acc[1] = f->acc[2] = 0.0f;
    f->n_acc = 0;
    for (i = 0; i < GPS_BUF_N; i++) {
        gps_entry_t *e = &f->buf[i];
        e->m = -1;
        e->pa[0] = e->pa[1] = e->va[0] = e->va[1] = e->u[0] = e->u[1] = e->lv[0] = e->lv[1] = 0.0f;
    }
    f->m_now = -1;
    f->has_pending = false;
    f->n_pending = 0;
    f->pending_te = 0;
    f->p_[0] = f->p_[1] = f->v[0] = f->v[1] = f->b[0] = f->b[1] = f->u[0] = f->u[1] = 0.0f;
    f->yaw_corr = 0.0f;
    f->c_psi = p->c_psi0;
    f->s_psi = p->s_psi0;
    f->p_psi = p->p_psi0;
    f->has_yaw_win = false;
    f->yaw_m0 = 0;
    f->yaw_g0[0] = f->yaw_g0[1] = f->yaw_u0[0] = f->yaw_u0[1] = 0.0f;
    f->yaw_used = 0;
    f->has_frame = false;
    f->init = false;
    f->pad = true;
    f->acq_left = 0;
    f->rej_p_run = f->rej_v_run = 0;
    f->rej_v_hist = 0;
    f->reset_count = 0;
    f->off[0] = f->off[1] = 0.0f;
    f->lever_now[0] = f->lever_now[1] = 0.0f;
    f->has_last_used = f->has_good_since = f->has_last_epoch = false;
    f->k_last_used = f->good_since = f->k_last_epoch = 0;
    f->last_fix3d = false;
    f->last_sv = 0;
    f->last_hacc_mm = 0xFFFFFFFFUL;
    f->cfg_ok = f->ver_ok = false;
    f->snap.flags = 0;
    f->snap.num_sv = 0;
    f->snap.hacc_dm = LP_HACC_NONE;
    f->snap.reset_count = 0;
    f->snap.off_x_cm = f->snap.off_y_cm = f->snap.px_cm = f->snap.py_cm = f->snap.vx_cms = f->snap.vy_cms = 0;
    f->snap.cos_q15 = NL_Q15;
    f->snap.sin_q15 = 0;
    f->cycle = 0;
    f->fu.stage = 0;
    f->sb_due = false;
    f->fe_te = 0;
    f->last_innov[0] = f->last_innov[1] = f->last_innov[2] = f->last_innov[3] = 0.0f;
    f->st_fused = f->st_rej_p = f->st_rej_v = f->st_pos_resets = f->st_vel_resets = 0;
    f->st_dropped_old = f->st_dropped_stale = f->st_n_epoch_dropped = f->st_unusable = 0;
    f->st_timebase_restarts = 0;
}

bool gps_filter_fix_ok(const gps_filter_t *f, uint32_t k)
{
    return f->init && f->has_last_used && k - f->k_last_used <= f->p.fresh_ticks;
}

bool gps_filter_gate_ok(const gps_filter_t *f, uint32_t k)
{
    return gps_filter_fix_ok(f, k) && f->has_good_since && k - f->good_since >= f->p.gate_stable_ticks;
}

bool gps_filter_head_ok(const gps_filter_t *f)
{
    return f->p_psi <= f->p.head_ok_rad2;
}

bool gps_filter_incons(const gps_filter_t *f)
{
    uint8_t h = (uint8_t)(f->rej_v_hist & ((1u << GPS_INCONS_EPOCHS) - 1u)), n = 0;
    while (h) {                                        /* popcount of the history */
        n = (uint8_t)(n + (h & 1u));
        h = (uint8_t)(h >> 1);
    }
    return n >= GPS_INCONS_MIN_REJ;
}

void gps_filter_epoch(gps_filter_t *f, const gps_epoch_t *e, uint32_t t_e_ms)
{
    f->pending = *e;
    f->pending_te = t_e_ms;
    f->has_pending = true;
    if (f->n_pending < 0xFFFFu)
        f->n_pending++;
}

static uint16_t inc16(uint16_t x)
{
    return x < 0xFFFFu ? (uint16_t)(x + 1u) : x;
}

/* the entry of step s (Python s % 16 for every int s: the two's complement low bits) */
static gps_entry_t *entry(gps_filter_t *f, int32_t s)
{
    return &f->buf[(uint8_t)((uint32_t)s & (GPS_BUF_N - 1u))];
}

/* level -> W: Rz(psi) */
static void rot(const gps_filter_t *f, float x, float y, float *ox, float *oy)
{
    *ox = f->c_psi * x - f->s_psi * y;
    *oy = f->s_psi * x + f->c_psi * y;
}

/* STATUS part of the snapshot (j = 4, with the step): the flags at this tick, satellites, hAcc, the reset counter */
static void latch_status(gps_filter_t *f, uint32_t k)
{
    uint8_t fl = 0;
    nl_gps_snap_t *s = &f->snap;
    if (gps_filter_fix_ok(f, k))
        fl |= GPS_F_FIX_OK;
    if (gps_filter_gate_ok(f, k))
        fl |= GPS_F_GATE_OK;
    if (gps_filter_head_ok(f))
        fl |= GPS_F_HEAD_OK;
    if (f->last_fix3d)
        fl |= GPS_F_FIX3D;
    if (f->cfg_ok)
        fl |= GPS_F_CFG_OK;
    if (f->init)
        fl |= GPS_F_INIT;
    if (f->ver_ok)
        fl |= GPS_F_VER_OK;
    if (gps_filter_incons(f))
        fl |= GPS_F_INCONS;
    s->flags = fl;
    s->num_sv = f->last_sv;
    if (f->last_hacc_mm >= 25450UL)                    /* min(255, (mm + 50) // 100); 0xFFFFFFFF (none) included */
        s->hacc_dm = LP_HACC_NONE;
    else
        s->hacc_dm = (uint8_t)((uint16_t)(f->last_hacc_mm + 50u) / 100u);   /* 16-bit division */
    s->reset_count = f->reset_count;
}

/* OFFSET / POS / VEL / HEAD parts (j = 0, after STEP-B): quantised from the state the step left - nothing changes it
 * between the step (j = 4) and here, so these are the reference's values latched at j = 4 */
static void latch_parts(gps_filter_t *f)
{
    nl_gps_snap_t *s = &f->snap;
    s->off_x_cm = (int16_t)lp_quantize(f->off[0], 100u, LP_I16_MIN, LP_I16_MAX);
    s->off_y_cm = (int16_t)lp_quantize(f->off[1], 100u, LP_I16_MIN, LP_I16_MAX);
    s->px_cm = (int16_t)lp_quantize(f->p_[0], 100u, LP_I16_MIN, LP_I16_MAX);
    s->py_cm = (int16_t)lp_quantize(f->p_[1], 100u, LP_I16_MIN, LP_I16_MAX);
    s->vx_cms = (int16_t)lp_quantize(f->v[0], 100u, LP_I16_MIN, LP_I16_MAX);
    s->vy_cms = (int16_t)lp_quantize(f->v[1], 100u, LP_I16_MIN, LP_I16_MAX);
    s->cos_q15 = (int16_t)lp_quantize(f->c_psi, NL_Q15, -NL_Q15, NL_Q15);
    s->sin_q15 = (int16_t)lp_quantize(f->s_psi, NL_Q15, -NL_Q15, NL_Q15);
}

/* STEP-A (j = 4): sim/gps_nav.py NavGpsFilter._step up to the integration - the mean force of the cycle (f0..f2, T: the
 * caller's), rows 0 and 1 of R(q) of the corrected attitude, a_W = Rz(psi) (R f)_xy, the pad / take-off rules, the
 * integration; R and w are kept for STEP-B.  2 x t is written t + t (exact). */
static void step_a(gps_filter_t *f, uint32_t k, uint32_t kq, const float q[4], const float w_b[3], bool on_pad, float f0,
                   float f1, float f2, float T)
{
    const gps_nav_params_t *p = &f->p;
    float ax, ay, t;
    const float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    const float xx = qx * qx, yy = qy * qy, zz = qz * qz, xy = qx * qy, wz = qw * qz, xz = qx * qz, wy = qw * qy;
    float (*r)[3] = f->rw;
    uint8_t i;
    t = yy + zz;
    r[0][0] = 1.0f - (t + t);
    t = xy - wz;
    r[0][1] = t + t;
    t = xz + wy;
    r[0][2] = t + t;
    t = xy + wz;
    r[1][0] = t + t;
    t = xx + zz;
    r[1][1] = 1.0f - (t + t);
    t = qy * qz - qw * qx;
    r[1][2] = t + t;
    rot(f, r[0][0] * f0 + r[0][1] * f1 + r[0][2] * f2, r[1][0] * f0 + r[1][1] * f1 + r[1][2] * f2, &ax, &ay);
    f->sw[0] = w_b[0];
    f->sw[1] = w_b[1];
    f->sw[2] = w_b[2];
    if (on_pad) {
        f->v[0] = f->v[1] = f->b[0] = f->b[1] = f->u[0] = f->u[1] = 0.0f;
        f->pad = true;
        f->has_yaw_win = false;
        f->rej_v_hist = 0;
    } else {
        if (f->pad) {                                  /* take-off: v known 0, b restarts, acquisition gains */
            f->v[0] = f->v[1] = f->b[0] = f->b[1] = f->u[0] = f->u[1] = 0.0f;
            f->acq_left = p->acq_epochs;
            f->pad = false;
        }
        for (i = 0; i < 2u; i++) {                     /* p += v T + a T^2 / 2 ((0.5 a) T = 0.5 (a T) exactly) */
            const float a = (i == 0u ? ax : ay) - f->b[i];
            const float aT = a * T;
            f->p_[i] += f->v[i] * T + 0.5f * aT * T;
            f->v[i] += aT;
            f->u[i] += aT;
        }
    }
    f->m_now = (int32_t)kq;                            /* k / step_ticks */
    f->cycle = (uint16_t)(f->cycle + 1u);
    f->sb_due = true;
    latch_status(f, k);
}

/* the NAV's STEP-A: the mean force of the accumulated cycle (x / n as x (1 / n) and T = t_step when n = step_ticks) */
static void step_a_nav(gps_filter_t *f, uint32_t k, uint32_t kq, const float q[4], const float w_b[3], bool on_pad)
{
    const gps_nav_params_t *p = &f->p;
    const uint8_t n = f->n_acc > 0u ? f->n_acc : 1u;
    float inv, T, f0, f1, f2;
    if (n == p->step_ticks) {
        inv = p->inv_step;
        T = p->t_step;
    } else {
        inv = 1.0f / (float)n;
        T = (float)n / p->loop_hz_f;
    }
    f0 = f->acc[0] * inv;
    f1 = f->acc[1] * inv;
    f2 = f->acc[2] * inv;
    f->acc[0] = f->acc[1] = f->acc[2] = 0.0f;
    f->n_acc = 0;
    step_a(f, k, kq, q, w_b, on_pad, f0, f1, f2, T);
}

/* STEP-B (j = 0 of the next cycle, before the fusion): the lever arm r = R l, v_lever = R (w x l) (= (R w) x (R l)), both
 * rotated by psi; the delayed-state entry m_now; the snapshot's other parts */
static void step_b(gps_filter_t *f)
{
    const gps_nav_params_t *p = &f->p;
    const float (*r)[3] = (const float (*)[3])f->rw;
    const float l0 = p->lever_b[0], l1 = p->lever_b[1], l2 = p->lever_b[2];
    const float w0 = f->sw[0], w1 = f->sw[1], w2 = f->sw[2];
    const float c0 = w1 * l2 - w2 * l1, c1 = w2 * l0 - w0 * l2, c2 = w0 * l1 - w1 * l0;   /* w x l (body) */
    float lpx, lpy, lvx, lvy;
    gps_entry_t *e = entry(f, f->m_now);
    rot(f, r[0][0] * l0 + r[0][1] * l1 + r[0][2] * l2, r[1][0] * l0 + r[1][1] * l1 + r[1][2] * l2, &lpx, &lpy);
    rot(f, r[0][0] * c0 + r[0][1] * c1 + r[0][2] * c2, r[1][0] * c0 + r[1][1] * c1 + r[1][2] * c2, &lvx, &lvy);
    e->m = f->m_now;
    e->pa[0] = f->p_[0] + lpx;
    e->pa[1] = f->p_[1] + lpy;
    e->va[0] = f->v[0] + lvx;
    e->va[1] = f->v[1] + lvy;
    e->u[0] = f->u[0];
    e->u[1] = f->u[1];
    e->lv[0] = lvx;
    e->lv[1] = lvy;
    f->lever_now[0] = lpx;
    f->lever_now[1] = lpy;
    f->sb_due = false;
    latch_parts(f);
}

/* the epoch completed by the fusion tick (j = 0) waits in fe for E0 (j = 1): an epoch completing at j = 1..4 belongs to
 * the next fusion, exactly as in the reference (which fuses at j = 0) */
static void promote(gps_filter_t *f, uint32_t k)
{
    if (f->n_pending > 1u)
        f->st_n_epoch_dropped = (uint16_t)(f->st_n_epoch_dropped + (f->n_pending - 1u));
    f->fe = f->pending;
    f->fe_te = f->pending_te;
    f->has_pending = false;
    f->n_pending = 0;
    f->fu.k = k;
    f->fu.stage = 1;
}

/* E0 (j = 1): bookkeeping, initialisation, delayed state, innovations (NavGpsFilter._fuse up to the gates; k = the fusion
 * tick j = 0) */
static void fuse_e0(gps_filter_t *f)
{
    const gps_nav_params_t *p = &f->p;
    const gps_epoch_t *ep = &f->fe;
    gps_fuse_t *fu = &f->fu;
    const uint32_t k = fu->k, te = f->fe_te;
    bool usable, good;
    float zx, zy;
    int32_t age_ms, a, s_e;
    const int32_t step_ms = (int32_t)p->ms_per_tick * (int32_t)p->step_ticks;
    gps_entry_t *ent;
    fu->stage = 0;
    if (f->has_last_epoch && k - f->k_last_epoch > p->fresh_ticks)
        f->has_good_since = false;                     /* a gap in the epochs restarts the ON gate's stability time */
    f->has_last_epoch = true;
    f->k_last_epoch = k;
    f->last_fix3d = ep->fix_ok && ep->gps_fix == UBX_FIX_3D;
    f->last_sv = ep->num_sv;
    f->last_hacc_mm = ep->hacc;
    usable = ep->fix_ok && ep->gps_fix >= UBX_FIX_3D && ep->num_sv >= p->ok_min_sv && ep->hacc <= p->ok_max_hacc_mm;
    good = ep->fix_ok && ep->gps_fix == UBX_FIX_3D && ep->num_sv >= p->gate_min_sv && ep->hacc <= p->gate_max_hacc_mm &&
           ep->sacc <= p->gate_max_sacc_cms;
    if (good) {
        if (!f->has_good_since) {
            f->has_good_since = true;
            f->good_since = k;
        }
    } else {
        f->has_good_since = false;
    }
    if (!usable) {
        f->st_unusable++;
        return;
    }
    if (!f->has_frame) {
        gps_frame_at(&f->frame, ep->lat, ep->lon);
        f->has_frame = true;
    }
    gps_frame_to_w(&f->frame, ep->lat, ep->lon, &zx, &zy);
    fu->zvx = (float)ep->vel_n * 0.01f;                /* cm/s / 100 (one float32 rounding of the same value) */
    fu->zvy = -((float)ep->vel_e * 0.01f);
    if (!f->init || f->m_now < 0) {                    /* first usable epoch: the origin, p = z - r_W */
        f->p_[0] = zx - f->lever_now[0];
        f->p_[1] = zy - f->lever_now[1];
        if (f->pad) {
            f->v[0] = f->v[1] = 0.0f;
        } else {
            f->v[0] = fu->zvx;
            f->v[1] = fu->zvy;
        }
        f->b[0] = f->b[1] = 0.0f;
        clear_buf(f);
        f->init = true;
        f->acq_left = p->acq_epochs;
        f->has_last_used = true;
        f->k_last_used = k;
        return;
    }
    age_ms = (int32_t)((uint32_t)p->ms_per_tick * ((uint32_t)p->step_ticks * (uint32_t)f->m_now + p->step_ticks - 1u) - te);
    /* a = floor((age_ms + step_ms / 2) / step_ms), 0 if negative; >= GPS_BUF_N: dropped (written without the int32
     * overflow of age_ms + 10 near INT32_MAX) */
    if (age_ms >= (int32_t)GPS_BUF_N * step_ms - step_ms / 2) {
        f->st_dropped_old++;
        return;
    }
    a = age_ms < -(step_ms / 2) ? 0 : (age_ms + step_ms / 2) / step_ms;
    s_e = f->m_now - a;
    ent = entry(f, s_e);
    if (ent->m != s_e) {
        f->st_dropped_stale++;
        return;
    }
    fu->s_e = s_e;
    fu->age = f->tab_t[a];                             /* a t_step, 0.5 (a t_step)^2 (0 <= a < GPS_BUF_N) */
    fu->hage = f->tab_h[a];
    fu->yp[0] = zx - ent->pa[0];
    fu->yp[1] = zy - ent->pa[1];
    fu->yv[0] = fu->zvx - ent->va[0];
    fu->yv[1] = fu->zvy - ent->va[1];
    f->last_innov[0] = fu->yp[0];
    f->last_innov[1] = fu->yp[1];
    f->last_innov[2] = fu->yv[0];
    f->last_innov[3] = fu->yv[1];
    fu->acq = f->acq_left > 0u;
    fu->used = false;
    fu->yaw_cs = false;
    fu->stage = 2;
}

/* the position reset: the state, the newest n_corr entries s >= s_e and the cumulative offset (_shift) */
static void shift(gps_filter_t *f, float dx, float dy, int32_t s_e)
{
    int32_t s;
    f->p_[0] += dx;
    f->p_[1] += dy;
    for (s = f->m_now - (int32_t)f->p.n_corr + 1; s <= f->m_now; s++) {
        gps_entry_t *en = entry(f, s);
        if (en->m == s && s >= s_e) {
            en->pa[0] += dx;
            en->pa[1] += dy;
        }
    }
    f->off[0] += dx;
    f->off[1] += dy;
    f->reset_count++;
}

/* E1 (j = 1, after E0): gates, rejection runs, position and velocity resets */
static void fuse_e1(gps_filter_t *f)
{
    const gps_nav_params_t *p = &f->p;
    gps_fuse_t *fu = &f->fu;
    const float *yp = fu->yp, *yv = fu->yv;
    fu->acc_p = yp[0] * yp[0] + yp[1] * yp[1] <= p->gate_p2;
    fu->acc_v[0] = yv[0] * yv[0] <= p->gate_v2;
    fu->acc_v[1] = yv[1] * yv[1] <= p->gate_v2;
    if (f->pad)
        fu->acc_v[0] = fu->acc_v[1] = false;           /* pad: velocity is known zero, position only */
    if (!fu->acc_p) {
        f->st_rej_p++;
        f->rej_p_run = inc16(f->rej_p_run);
    } else {
        f->rej_p_run = 0;
    }
    if (!f->pad) {
        const bool rej_v = !(fu->acc_v[0] && fu->acc_v[1]);
        if (rej_v) {
            f->st_rej_v++;
            f->rej_v_run = inc16(f->rej_v_run);
        } else {
            f->rej_v_run = 0;
        }
        f->rej_v_hist = (uint8_t)(((uint8_t)(f->rej_v_hist << 1) | (rej_v ? 1u : 0u)) & ((1u << GPS_INCONS_EPOCHS) - 1u));
    }
    if (!fu->acc_p && f->rej_p_run >= p->reset_after_epochs && (f->pad || (fu->acc_v[0] && fu->acc_v[1]))) {
        shift(f, yp[0], yp[1], fu->s_e);               /* position reset: a GPS jump never moves the vehicle */
        f->st_pos_resets++;
        f->rej_p_run = 0;
        f->acq_left = p->acq_epochs;
        fu->used = true;
    }
    if (!f->pad && p->vel_reset_epochs > 0u && f->rej_v_run >= p->vel_reset_epochs) {
        int32_t s;
        uint8_t i;
        for (i = 0; i < 2u; i++) {                     /* velocity reset (the gating death spiral) */
            f->v[i] += yv[i];
            f->p_[i] += yv[i] * fu->age;
            f->b[i] = 0.0f;
        }
        for (s = f->m_now - (int32_t)p->n_corr + 1; s <= f->m_now; s++) {
            gps_entry_t *en = entry(f, s);
            if (en->m == s && s >= fu->s_e) {
                const float ae = f->tab_t[(uint8_t)(s - fu->s_e)];
                for (i = 0; i < 2u; i++) {
                    en->va[i] += yv[i];
                    en->pa[i] += yv[i] * ae;
                }
            }
        }
        f->reset_count++;                              /* counted (FCU plausibility) but NOT in the offset */
        f->st_vel_resets++;
        f->rej_v_run = 0;
        f->acq_left = p->acq_epochs;
        fu->acc_v[0] = fu->acc_v[1] = false;
        fu->used = true;
    }
}

/* E2 (j = 2): the fixed-gain update of one axis, propagated to now (Phi(age)) and to the newest n_corr entries s >= s_e
 * (NavGpsFilter._apply; the per-entry ae and ae^2 / 2 of E0 are shared by both axes) */
static void fuse_e2_axis(gps_filter_t *f, uint8_t i)
{
    gps_fuse_t *fu = &f->fu;
    const gps_gain_set_t *G = fu->acq ? &f->p.acq : &f->p.steady;
    const bool up = fu->acc_p, uv = fu->acc_v[i];
    const float yp = fu->yp[i], yv = fu->yv[i];
    float d0, d1, d2;
    int32_t s;
    if (up && uv) {
        d0 = G->kpv[0][0] * yp + G->kpv[0][1] * yv;
        d1 = G->kpv[1][0] * yp + G->kpv[1][1] * yv;
        d2 = G->kpv[2][0] * yp + G->kpv[2][1] * yv;
    } else if (up) {
        d0 = G->kp[0] * yp;
        d1 = G->kp[1] * yp;
        d2 = G->kp[2] * yp;
    } else if (uv) {
        d0 = G->kv[0] * yv;
        d1 = G->kv[1] * yv;
        d2 = G->kv[2] * yv;
    } else {
        return;
    }
    if (f->pad)
        d1 = d2 = 0.0f;
    f->p_[i] += d0 + fu->age * d1 - fu->hage * d2;
    f->v[i] += d1 - fu->age * d2;
    f->b[i] += d2;
    for (s = f->m_now - (int32_t)f->p.n_corr + 1; s <= f->m_now; s++) {
        gps_entry_t *en = entry(f, s);
        if (en->m == s && s >= fu->s_e) {
            const uint8_t d = (uint8_t)(s - fu->s_e);
            const float ae = f->tab_t[d];
            en->pa[i] += d0 + ae * d1 - f->tab_h[d] * d2;
            en->va[i] += d1 - ae * d2;
        }
    }
}

/* E3 (j = 1, after E1): used-epoch bookkeeping and the information-weighted yaw update (its cos / sin at j = 2).  An
 * axis is updated by E2 iff its position or velocity was accepted (after E1), so "used" is known here; the yaw update
 * reads the epoch, the entry's u / v_lever (never changed by E2) and its own state - the reference's numbers. */
static void fuse_e3(gps_filter_t *f)
{
    const gps_nav_params_t *p = &f->p;
    gps_fuse_t *fu = &f->fu;
    fu->yaw_cs = false;
    if (fu->acc_p || fu->acc_v[0] || fu->acc_v[1])
        fu->used = true;
    if (fu->used) {
        f->has_last_used = true;
        f->k_last_used = fu->k;
        f->st_fused++;
        if (f->acq_left > 0u && !f->pad)
            f->acq_left--;
    }
    if (p->yaw_correction && !f->pad && fu->acc_v[0] && fu->acc_v[1]) {
        const gps_entry_t *ent = entry(f, fu->s_e);
        const float gx = fu->zvx - ent->lv[0], gy = fu->zvy - ent->lv[1];
        bool reopen = true;
        if (!f->has_yaw_win) {
            f->has_yaw_win = true;
        } else if (fu->s_e - f->yaw_m0 >= (int32_t)p->yaw_window_steps) {
            const float dux = ent->u[0] - f->yaw_u0[0], duy = ent->u[1] - f->yaw_u0[1];
            const float dgx = gx - f->yaw_g0[0], dgy = gy - f->yaw_g0[1];
            const float nu2 = dux * dux + duy * duy;
            /* the variance grows over every closed window, used or not (physics-review fix) */
            f->p_psi += p->yaw_q_rad2 * (float)(fu->s_e - f->yaw_m0) * p->t_step;
            if (nu2 >= p->yaw_min_dv2 && 4.0f * (dgx * dgx + dgy * dgy) > nu2) {
                const float ang = atan2f(dux * dgy - duy * dgx, dux * dgx + duy * dgy);
                float r, kk;
                r = p->yaw_sigma_dv2 / nu2 + p->yaw_sigma_rel2;
                kk = f->p_psi / (f->p_psi + r);
                f->yaw_corr += kk * ang;
                f->p_psi *= (1.0f - kk);
                fu->yaw_cs = true;                     /* cos / sin of psi in the next job */
                f->yaw_used++;
            }
        } else {
            reopen = false;                            /* the window is still open: kept */
        }
        if (reopen) {
            f->yaw_m0 = fu->s_e;                       /* the window (re)opens here */
            f->yaw_g0[0] = gx;
            f->yaw_g0[1] = gy;
            f->yaw_u0[0] = ent->u[0];
            f->yaw_u0[1] = ent->u[1];
        }
    }
}

static void yaw_cos_sin(gps_filter_t *f)
{
    const float psi = f->p.psi0 + f->yaw_corr;
    f->c_psi = cosf(psi);
    f->s_psi = sinf(psi);
    f->fu.yaw_cs = false;
}

/* the jobs of a fusion: 1 = E0 + E1 + E3 (j = 1), 2 = E2 x + cos / sin (j = 2), 3 = E2 y (j = 3) */
static void fuse_job(gps_filter_t *f)
{
    gps_fuse_t *fu = &f->fu;
    if (fu->stage == 1u) {
        fuse_e0(f);                                    /* stage 2 if the epoch is fused further, else 0 */
        if (fu->stage == 2u) {
            fuse_e1(f);
            fuse_e3(f);
            fu->stage = 2;
        }
    } else if (fu->stage == 2u) {
        fuse_e2_axis(f, 0);
        if (fu->yaw_cs)
            yaw_cos_sin(f);
        fu->stage = 3;
    } else if (fu->stage == 3u) {
        fuse_e2_axis(f, 1);
        fu->stage = 0;
    }
}

/* the remaining fusion jobs at once (ticks were skipped: the reference's at-once fusion) */
static void fuse_finish(gps_filter_t *f)
{
    while (f->fu.stage != 0u)
        fuse_job(f);
}

/* the five NAV ticks of one 50 Hz cycle (j = k mod 5; the reference: fusion at j = 0, step at j = 4, both at once):
 *   j = 0  STEP-B of the previous step (lever arm, delayed-state entry, snapshot parts), the epoch promoted for E0
 *   j = 1  E0 + E1 + E3 (bookkeeping, yaw)      j = 2  E2 x, cos / sin of psi      j = 3  E2 y
 *   j = 4  STEP-A (the integration, the STATUS part)
 * Nothing reads or writes the state between the jobs out of this order, so the numbers are the reference's.  j and
 * k / step_ticks are counted (no 32-bit division per tick) while k advances by one. */
uint8_t gps_filter_tick(gps_filter_t *f, uint32_t k, const float accel_b[3], const float q_corr[4], const float rate_b[3],
                        bool on_pad)
{
    uint8_t j, r = GPS_TICK_NONE;
    if (f->started && k == f->k + 1u) {
        j = (uint8_t)(f->jc + 1u);
        if (j >= f->p.step_ticks) {
            j = 0;
            f->kq++;
        }
    } else {
        j = (uint8_t)(k % f->p.step_ticks);
        f->kq = k / f->p.step_ticks;
        f->started = true;
    }
    f->jc = j;
    f->k = k;
    f->acc[0] += accel_b[0];
    f->acc[1] += accel_b[1];
    f->acc[2] += accel_b[2];
    if (f->n_acc < 0xFFu)
        f->n_acc++;
    if (j == 0u) {
        if (f->sb_due) {
            step_b(f);
            r = GPS_TICK_SNAP;
        }
        fuse_finish(f);
        if (f->has_pending)
            promote(f, k);
    } else if (j <= 3u && f->fu.stage == j) {
        fuse_job(f);
    }
    if (j == (uint8_t)(f->p.step_ticks - 1u)) {
        if (f->sb_due)                                 /* (ticks were skipped) */
            step_b(f);
        fuse_finish(f);
        step_a_nav(f, k, f->kq, q_corr, rate_b, on_pad);
        r = GPS_TICK_STEP;
    }
    return r;
}

/* ---- v0.8.1: the same filter on the GNS board (sim/gps_nav.py gns_step / gns_fuse / restart_timebase) ---- */
void gps_filter_gns_step(gps_filter_t *f, uint32_t k, const float fsum[3], uint8_t nf, uint32_t dk, const float q_corr[4],
                         const float rate_b[3], bool on_pad)
{
    const gps_nav_params_t *p = &f->p;
    float inv, T;
    if (nf == 0u)
        nf = p->step_ticks;
    inv = nf == p->step_ticks ? p->inv_step : 1.0f / (float)nf;   /* the mean force of the frame's ticks */
    T = dk == p->step_ticks ? p->t_step : (float)dk / p->loop_hz_f;
    f->k = k;
    f->jc = (uint8_t)(p->step_ticks - 1u);
    f->kq = k / p->step_ticks;
    f->started = false;                                /* (the NAV-tick job schedule is not used on the GNS) */
    if (f->fu.stage != 0u)                             /* (gps_filter_gns_fuse runs every job at once: never) */
        fuse_finish(f);
    step_a(f, k, f->kq, q_corr, rate_b, on_pad, fsum[0] * inv, fsum[1] * inv, fsum[2] * inv, T);
    step_b(f);                                         /* the whole step at once (the GNS has the 20 ms cycle) */
}

bool gps_filter_gns_fuse(gps_filter_t *f, uint32_t k)
{
    if (!f->has_pending)
        return false;
    promote(f, k);                                     /* the newest pending epoch; the older ones counted dropped */
    fuse_finish(f);                                    /* E0 .. E3 at once */
    return true;
}

void gps_filter_restart_timebase(gps_filter_t *f)
{
    clear_buf(f);
    f->has_pending = false;
    f->n_pending = 0;
    f->m_now = -1;
    f->acc[0] = f->acc[1] = f->acc[2] = 0.0f;
    f->n_acc = 0;
    f->init = false;
    f->has_last_used = false;
    f->has_good_since = false;
    f->has_last_epoch = false;
    f->has_yaw_win = false;
    f->fu.stage = 0;
    f->sb_due = false;
    f->st_timebase_restarts = inc16(f->st_timebase_restarts);
}

/* ---- NavGps ---- */
void gps_nav_init(gps_nav_t *g, const gps_nav_params_t *p, bool configure, uint16_t n_wait, uint16_t n_ack,
                  uint16_t n_settle, uint16_t n_ver, uint32_t baud)
{
    ubx_parser_init(&g->parser);
    gps_asm_init(&g->asm_);
    gps_itow_init(&g->clock, p->ms_per_tick, p->latency_ms);
    gps_cfg_init(&g->cfg, n_wait, n_ack, n_settle, n_ver, baud);
    g->configure = configure;
    gps_filter_init(&g->filter, p);
    if (!configure)
        g->filter.cfg_ok = true;
    g->epochs_this_tick = 0;
}

void gps_nav_poll(gps_nav_t *g, uint32_t k, const uint8_t *rx, uint16_t n_rx, gps_cfg_action_t *act)
{
    uint16_t i;
    g->epochs_this_tick = 0;
    for (i = 0; i < n_rx; i++) {
        uint8_t kind;
        const gps_epoch_t *e;
        if (!ubx_parse_byte(&g->parser, rx[i]))
            continue;
        kind = ubx_msg_kind(&g->parser);
        if (kind == UBX_K_NONE)
            continue;
        if (g->configure) {
            if (kind == UBX_K_ACK || kind == UBX_K_NAK)
                gps_cfg_msg(&g->cfg, kind, g->parser.buf[0], g->parser.buf[1], 0);
            else if (kind == UBX_K_MONVER)
                gps_cfg_msg(&g->cfg, kind, 0, 0, ubx_monver_module(g->parser.buf));
        }
        e = gps_asm_add(&g->asm_, kind, g->parser.buf, k);
        if (e != 0) {
            gps_filter_epoch(&g->filter, e, gps_itow_update(&g->clock, e->itow, k));
            g->epochs_this_tick++;
        }
    }
    act->baud = 0;
    act->tx = GPS_TX_NONE;
    if (g->configure) {
        gps_cfg_tick(&g->cfg, act);
        g->filter.cfg_ok = g->cfg.cfg_ok;
        g->filter.ver_ok = g->cfg.ver_ok;
    }
}

uint8_t gps_nav_tick(gps_nav_t *g, uint32_t k, const uint8_t *rx, uint16_t n_rx, const float accel_b[3],
                     const float q_corr[4], const float rate_b[3], bool on_pad, gps_cfg_action_t *act)
{
    gps_nav_poll(g, k, rx, n_rx, act);
    return gps_filter_tick(&g->filter, k, accel_b, q_corr, rate_b, on_pad);
}
