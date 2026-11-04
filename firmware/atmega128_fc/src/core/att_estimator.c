/* att_estimator.c - see att_estimator.h (mirrors sim/estimator.py PadAlignGyroEstimator). */
#include "att_estimator.h"
#include "quat.h"
#include "qfix.h"

void att_est_init(att_est_t *e, float dt, uint16_t n_avg)
{
    e->dt = dt;
    e->n_avg = n_avg > 0 ? n_avg : 1;
    e->n = 0;
    e->bias[0] = e->bias[1] = e->bias[2] = 0.0f;
    e->f_mean[0] = e->f_mean[1] = e->f_mean[2] = 0.0f;
    quat_identity(e->q);
    e->aligned = false;
}

/* level_from_accel(): yaw-0 attitude whose gravity reaction matches f_b,
 *   roll = atan2(f_y, f_z), pitch = atan2(-f_x, hypot(f_y, f_z)), q = q_y(pitch) (x) q_x(roll)
 * evaluated with half-angle identities instead of atan2/sin/cos (same value to float
 * precision, about a quarter of the cycles):
 *   cos(r/2) = sqrt((1 + cos r)/2), sin(r/2) = sin r / (2 cos(r/2))   (|r| < 154 deg)
 *   sin(r/2) = sign(f_y) sqrt((1 - cos r)/2), cos(r/2) = sin r / (2 sin(r/2))  (otherwise)
 *   pitch in [-90, 90] deg: cos(p/2) >= 0.707 always. */
void att_est_level_from_accel(const float f_b[3], float q[4])
{
    const float fx = f_b[0], fy = f_b[1], fz = f_b[2];
    const float nyz2 = fy * fy + fz * fz;
    const float nyz = sqrtf(nyz2);
    float cr = 1.0f, sr = 0.0f, cp = 1.0f, sp = 0.0f;
    if (nyz2 > 0.0f) {
        const float inv_nyz = 1.0f / nyz;
        const float c = fz * inv_nyz, s = fy * inv_nyz;
        if (c > -0.9f) {
            cr = sqrtf(0.5f * (1.0f + c));
            sr = s / (2.0f * cr);
        } else {
            sr = copysignf(sqrtf(0.5f * (1.0f - c)), fy);
            cr = s / (2.0f * sr);
        }
    }
    {
        const float n2 = nyz2 + fx * fx;
        if (n2 > 0.0f) {
            const float inv_n = 1.0f / sqrtf(n2);
            const float c = nyz * inv_n, s = -fx * inv_n;
            cp = sqrtf(0.5f * (1.0f + c));
            sp = s / (2.0f * cp);
        }
    }
    q[0] = cp * cr;
    q[1] = cp * sr;
    q[2] = sp * cr;
    q[3] = -sp * sr;
}

/* update() */
void att_est_update(att_est_t *e, const float gyro[3], const float accel[3], bool on_pad)
{
    int i;
    if (on_pad) {
        float inv_n;
        if (e->n < e->n_avg)
            e->n++;
        inv_n = 1.0f / (float)e->n;
        for (i = 0; i < 3; i++) {
            e->bias[i] = e->bias[i] + (gyro[i] - e->bias[i]) * inv_n;
            e->f_mean[i] = e->f_mean[i] + (accel[i] - e->f_mean[i]) * inv_n;
        }
        att_est_level_from_accel(e->f_mean, e->q);
        e->aligned = true;
    } else {
        float w[3] = {gyro[0] - e->bias[0], gyro[1] - e->bias[1], gyro[2] - e->bias[2]};
        float qn[4];
        quat_integrate(e->q, w, e->dt, qn);
        quat_copy(qn, e->q);
    }
}

/* rate(): bias-corrected body rate for the rate PID */
void att_est_rate(const att_est_t *e, const float gyro[3], float rate[3])
{
    rate[0] = gyro[0] - e->bias[0];
    rate[1] = gyro[1] - e->bias[1];
    rate[2] = gyro[2] - e->bias[2];
}

/* ---- v0.9 P1: the flight integration in fixed point (core/qfix.h) ----
 * quat_integrate_fx = quat_integrate (quat.c, sim/quat.py integrate) on a Q30 state: rv = omega dt (float, then Q32;
 * |rv_i| < 0.25), a2 = |rv|^2, x2 = a2 / 4; c = 1 - x2 (1/2 - x2/24) as q c = q - q delta with delta = x2/2 - x2^2/24;
 * h = sin(|rv|/2) / |rv| = 1/2 - x2/12 + x2^2/240 (the same series as exp_small_a2); r = q (x) (c, h rv); then the
 * Newton renormalisation r (1.5 - 0.5 |r|^2) inside the same (0.9999, 1.0001) window as quat_renormalize_near_unit.
 * Anything outside the series / window (never in flight: |rv| <= 0.14 rad at the gyro's full scale) runs the float
 * quat_integrate on the float copy instead (identical to the v0.8.1 code path). */
#define FX_K_1_24 178956971L                  /* 2^32 / 24 */
#define FX_K_1_12 357913941L                  /* 2^32 / 12 */
#define FX_K_1_240 17895697L                  /* 2^32 / 240 */
#define FX_A2_MAX 171798692L                  /* QUAT_SERIES_MAX_ANGLE^2 = 0.04 in Q32 */
#define FX_N2_TOL 26844L                      /* 1e-4 in Q28 */

static bool quat_integrate_fx(const int32_t q[4], const float w[3], float dt, int32_t out[4]) __attribute__((noinline));
static bool quat_integrate_fx(const int32_t q[4], const float w[3], float dt, int32_t out[4])
{
    const float f0 = w[0] * dt, f1 = w[1] * dt, f2 = w[2] * dt;
    int32_t r0, r1, r2, a2, x2, x2s, dl, hm, d1, d2, d3, p0, p1, p2, p3, n2, eps;
    const int32_t q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    if (fx_f_exp(f0) > 124u || fx_f_exp(f1) > 124u || fx_f_exp(f2) > 124u)   /* |rv_i| >= 0.25 (> the 0.2 rad series
                                                                              * range): the Q32 sum a2 could overflow */
        return false;
    r0 = fx_from_f(f0, 32);
    r1 = fx_from_f(f1, 32);
    r2 = fx_from_f(f2, 32);
    a2 = fx_mulhi(r0, r0) + fx_mulhi(r1, r1) + fx_mulhi(r2, r2);              /* Q32 */
    if (a2 >= FX_A2_MAX)
        return false;
    x2 = a2 >> 2;
    x2s = fx_mulhi(x2, x2);
    dl = (x2 >> 1) - fx_mulhi(x2s, FX_K_1_24);                                /* 1 - c, Q32 */
    hm = fx_mulhi(x2s, FX_K_1_240) - fx_mulhi(x2, FX_K_1_12);                 /* h - 1/2, Q32 */
    d1 = (r0 >> 1) + fx_mulhi(hm, r0);                                        /* h rv, Q32 */
    d2 = (r1 >> 1) + fx_mulhi(hm, r1);
    d3 = (r2 >> 1) + fx_mulhi(hm, r2);
    /* q (x) (c, d) = q - delta q + q (x) (0, d); Q30 x Q32 -> Q30 */
    p0 = q0 - fx_mulhi(q0, dl) - fx_mulhi(q1, d1) - fx_mulhi(q2, d2) - fx_mulhi(q3, d3);
    p1 = q1 - fx_mulhi(q1, dl) + fx_mulhi(q0, d1) + fx_mulhi(q2, d3) - fx_mulhi(q3, d2);
    p2 = q2 - fx_mulhi(q2, dl) + fx_mulhi(q0, d2) - fx_mulhi(q1, d3) + fx_mulhi(q3, d1);
    p3 = q3 - fx_mulhi(q3, dl) + fx_mulhi(q0, d3) + fx_mulhi(q1, d2) - fx_mulhi(q2, d1);
    n2 = fx_mulhi(p0, p0) + fx_mulhi(p1, p1) + fx_mulhi(p2, p2) + fx_mulhi(p3, p3);   /* Q28 */
    n2 = (1L << 28) - n2;                                                     /* 1 - |r|^2 */
    if (n2 >= FX_N2_TOL || n2 <= -FX_N2_TOL)
        return false;
    eps = n2 * 8;                                                             /* (1 - |r|^2) / 2 in Q32 */
    out[0] = p0 + fx_mulhi(p0, eps);                                          /* r (1.5 - 0.5 |r|^2) */
    out[1] = p1 + fx_mulhi(p1, eps);
    out[2] = p2 + fx_mulhi(p2, eps);
    out[3] = p3 + fx_mulhi(p3, eps);
    return true;
}

static void quat_copy_i32(const int32_t a[4], int32_t out[4])
{
    out[0] = a[0];
    out[1] = a[1];
    out[2] = a[2];
    out[3] = a[3];
}

static void fx_quat_to_f(const int32_t qf[4], float q[4])
{
    q[0] = fx_to_f(qf[0], 30);
    q[1] = fx_to_f(qf[1], 30);
    q[2] = fx_to_f(qf[2], 30);
    q[3] = fx_to_f(qf[3], 30);
}

static void fx_quat_from_f(const float q[4], int32_t qf[4])
{
    qf[0] = fx_from_f(q[0], 30);
    qf[1] = fx_from_f(q[1], 30);
    qf[2] = fx_from_f(q[2], 30);
    qf[3] = fx_from_f(q[3], 30);
}

/* one flight integration step of the estimator: qf -> qf_next (fixed point) or, outside its range, the float path on
 * pad.q; out = the float result (q_pub or the new pad.q) */
static void mahony_integrate(att_mahony_t *m, const float w[3], int32_t qn[4], float out[4])
{
    if (quat_integrate_fx(m->qf, w, m->pad.dt, qn)) {
        fx_quat_to_f(qn, out);
    } else {
        float qt[4];
        quat_integrate(m->pad.q, w, m->pad.dt, qt);
        quat_copy(qt, out);
        fx_quat_from_f(qt, qn);
    }
}

/* qf := pad.q, once per stay off the pad (the first flight predict / update) */
static void mahony_qf_sync(att_mahony_t *m)
{
    if (!m->qf_ok) {
        fx_quat_from_f(m->pad.q, m->qf);
        m->qf_ok = true;
    }
}

/* v = R(q)^T e_z from the float q (the pad branch; the v0.8.1 correction's / a_z_w's float expressions) */
static void mahony_up_f(att_mahony_t *m)
{
    const float *q = m->pad.q;
    const float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    float t;
    t = qx * qz - qw * qy;
    m->v[0] = t + t;
    t = qy * qz + qw * qx;
    m->v[1] = t + t;
    t = qx * qx + qy * qy;
    m->v[2] = 1.0f - (t + t);
}

/* the same from qf (Q30 x Q30 -> Q28; 2 t in Q28 = t in Q27) */
static void mahony_up_fx(att_mahony_t *m)
{
    const int32_t qw = m->qf[0], qx = m->qf[1], qy = m->qf[2], qz = m->qf[3];
    m->v[0] = fx_to_f(fx_mulhi(qx, qz) - fx_mulhi(qw, qy), 27);
    m->v[1] = fx_to_f(fx_mulhi(qy, qz) + fx_mulhi(qw, qx), 27);
    m->v[2] = fx_to_f((1L << 27) - (fx_mulhi(qx, qx) + fx_mulhi(qy, qy)), 27);
}

/* ---- v0.7.1 VTOL: GatedMahonyEstimator (att_estimator.h) ---- */
void att_mahony_init(att_mahony_t *m, float dt, uint16_t n_avg, float kp, float ki_dt, float gate_mps2, float g,
                     float alpha_a, float bias_limit)
{
    uint8_t i;
    att_est_init(&m->pad, dt, n_avg);
    m->qf_ok = false;
    mahony_up_f(m);
    m->kp = kp;
    m->ki_dt = ki_dt;
    m->gate_mps2 = gate_mps2;
    m->g = g;
    m->gate_lo2 = (g - gate_mps2) * (g - gate_mps2);
    m->gate_hi2 = (g + gate_mps2) * (g + gate_mps2);
    m->alpha_a = alpha_a;
    m->bias_limit = bias_limit;
    for (i = 0; i < 3u; i++)
        m->b_pad[i] = m->a_lpf[i] = m->e[i] = 0.0f;
    m->gate = false;
    m->was_pad = false;
    m->flight_ticks = 0;
    m->gated_ticks = 0;
}

/* The flight branch is written out per axis with the operands in locals (avr-gcc otherwise reloads every float through
 * the struct pointer): the same operations in the reference's order, ~20 % fewer cycles on the ATmega128.  Two exact
 * rewrites: 2 x t as t + t, and the gate | |a_lpf| - g | < accel_gate g (|a_lpf| > 0) as gate_lo2 < |a_lpf|^2 < gate_hi2
 * with gate_lo2 = (g - G)^2, gate_hi2 = (g + G)^2 (G = accel_gate g < g; equivalent for |a_lpf| >= 0, so the square root
 * and the division run only when the gate is open); with the gate closed e = 0 and the rate is g - b exactly as in the
 * reference (kp x 0 adds nothing). */
void att_mahony_update(att_mahony_t *m, const float gyro[3], const float accel[3], bool on_pad)
{
    uint8_t i;
    float a0, a1, a2, n2, w[3];
    int32_t qn[4];
    if (on_pad) {
        if (!m->was_pad && m->flight_ticks > 0u)     /* re-entry: level from the landing spot, not the previous pad */
            for (i = 0; i < 3u; i++)
                m->pad.f_mean[i] = m->a_lpf[i];
        att_est_update(&m->pad, gyro, accel, true);   /* the PadAlignGyroEstimator pad branch, bit for bit */
        for (i = 0; i < 3u; i++) {
            m->a_lpf[i] = m->pad.f_mean[i];
            m->b_pad[i] = m->pad.bias[i];
            m->e[i] = 0.0f;
        }
        m->gate = false;
        m->was_pad = true;
        m->qf_ok = false;                             /* P1: the float pad q is the state; its up vector in float */
        mahony_up_f(m);
        return;
    }
    mahony_qf_sync(m);                                /* P1: the first flight tick takes over pad.q */
    m->was_pad = false;
    if (m->flight_ticks < 0xFFFFFFFFUL)
        m->flight_ticks++;
    {
        const float al = m->alpha_a;
        a0 = m->a_lpf[0];
        a1 = m->a_lpf[1];
        a2 = m->a_lpf[2];
        a0 = a0 + al * (accel[0] - a0);
        a1 = a1 + al * (accel[1] - a1);
        a2 = a2 + al * (accel[2] - a2);
        m->a_lpf[0] = a0;
        m->a_lpf[1] = a1;
        m->a_lpf[2] = a2;
    }
    n2 = a0 * a0 + a1 * a1 + a2 * a2;
    m->gate = n2 > m->gate_lo2 && n2 < m->gate_hi2;
    if (m->gate) {
        const float inv = 1.0f / sqrtf(n2);
        const float ax = a0 * inv, ay = a1 * inv, az = a2 * inv;
        float vx, vy, vz, e0, e1, e2, d, *b = m->pad.bias;
        const float *bp = m->b_pad, lim = m->bias_limit, kidt = m->ki_dt, kp = m->kp;
        vx = m->v[0];                                 /* estimated world up in the body frame: R(q)^T e_z (P1: kept
                                                       * with q, core/att_estimator.h) */
        vy = m->v[1];
        vz = m->v[2];
        if (m->gated_ticks < 0xFFFFFFFFUL)
            m->gated_ticks++;
        e0 = ay * vz - az * vy;
        e1 = az * vx - ax * vz;
        e2 = ax * vy - ay * vx;
        m->e[0] = e0;
        m->e[1] = e1;
        m->e[2] = e2;
        d = b[0] - bp[0] - kidt * e0;                 /* in-flight bias learning, bounded relative to the pad value */
        b[0] = bp[0] + fc_clampf(d, -lim, lim);
        d = b[1] - bp[1] - kidt * e1;
        b[1] = bp[1] + fc_clampf(d, -lim, lim);
        d = b[2] - bp[2] - kidt * e2;
        b[2] = bp[2] + fc_clampf(d, -lim, lim);
        w[0] = gyro[0] - b[0] + kp * e0;
        w[1] = gyro[1] - b[1] + kp * e1;
        w[2] = gyro[2] - b[2] + kp * e2;
    } else {
        m->e[0] = m->e[1] = m->e[2] = 0.0f;
        w[0] = gyro[0] - m->pad.bias[0];
        w[1] = gyro[1] - m->pad.bias[1];
        w[2] = gyro[2] - m->pad.bias[2];
    }
    mahony_integrate(m, w, qn, m->pad.q);             /* P1: Q30 state, pad.q its float copy */
    quat_copy_i32(qn, m->qf);
    mahony_up_fx(m);
}

void att_mahony_rate(const att_mahony_t *m, const float gyro[3], float rate[3])
{
    att_est_rate(&m->pad, gyro, rate);
}

void att_mahony_predict(att_mahony_t *m, const float gyro[3], bool on_pad, float q_pub[4], float rate[3])
{
    att_est_rate(&m->pad, gyro, rate);                 /* g - b of the previous tick's correction */
    if (on_pad) {
        quat_copy(m->pad.q, q_pub);
    } else {
        int32_t qn[4];                                 /* P1: from the Q30 state; only the float q_pub is kept */
        mahony_qf_sync(m);
        mahony_integrate(m, rate, qn, q_pub);
    }
}
