/* quat.c - see quat.h; every block names the sim/quat.py function it mirrors. */
#include "quat.h"

void quat_identity(float q[4])
{
    q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;
}

void quat_copy(const float a[4], float out[4])
{
    out[0] = a[0]; out[1] = a[1]; out[2] = a[2]; out[3] = a[3];
}

/* normalize(): q / |q|, near-zero -> identity */
void quat_normalize(const float q[4], float out[4])
{
    float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n < 1e-12f) {
        quat_identity(out);
        return;
    }
    out[0] = q[0] / n; out[1] = q[1] / n; out[2] = q[2] / n; out[3] = q[3] / n;
}

/* multiply(): Hamilton product a (x) b */
void quat_multiply(const float a[4], const float b[4], float out[4])
{
    const float aw = a[0], ax = a[1], ay = a[2], az = a[3];
    const float bw = b[0], bx = b[1], by = b[2], bz = b[3];
    out[0] = aw * bw - ax * bx - ay * by - az * bz;
    out[1] = aw * bx + ax * bw + ay * bz - az * by;
    out[2] = aw * by - ax * bz + ay * bw + az * bx;
    out[3] = aw * bz + ax * by - ay * bx + az * bw;
}

void quat_conjugate(const float q[4], float out[4])
{
    out[0] = q[0]; out[1] = -q[1]; out[2] = -q[2]; out[3] = -q[3];
}

/* from_axis_angle() */
void quat_from_axis_angle(const float axis[3], float angle_rad, float out[4])
{
    float n = sqrtf(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    if (n < 1e-12f) {
        quat_identity(out);
        return;
    }
    float s = sinf(0.5f * angle_rad);
    out[0] = cosf(0.5f * angle_rad);
    out[1] = s * (axis[0] / n);
    out[2] = s * (axis[1] / n);
    out[3] = s * (axis[2] / n);
}

/* from_rotvec(): exponential map */
void quat_from_rotvec(const float rv[3], float out[4])
{
    float angle = sqrtf(rv[0] * rv[0] + rv[1] * rv[1] + rv[2] * rv[2]);
    if (angle < 1e-9f) {
        float t[4] = {1.0f, 0.5f * rv[0], 0.5f * rv[1], 0.5f * rv[2]};
        quat_normalize(t, out);
        return;
    }
    quat_from_axis_angle(rv, angle, out);
}

/* to_rotvec(): logarithmic map, shortest path; e = 2 atan2(|v|, w) v / |v|.
 * For |v|/w < 0.1 (rotation < 11.4 deg, the normal flight case) 2 atan(t)/|v| is
 * evaluated as (2/w) atan(t)/t with t^2 = |v|^2/w^2 and the series
 * 1 - t^2/3 + t^4/5 - t^6/7 + t^8/9 (next term < 1e-11): no sqrt/atan2. */
void quat_to_rotvec(const float q[4], float out[3])
{
    float w = q[0], vx = q[1], vy = q[2], vz = q[3];
    float n2, n;
    if (w < 0.0f) {
        w = -w; vx = -vx; vy = -vy; vz = -vz;
    }
    n2 = vx * vx + vy * vy + vz * vz;
    if (n2 < 1e-18f) {                           /* |v| < 1e-9 */
        out[0] = 2.0f * vx; out[1] = 2.0f * vy; out[2] = 2.0f * vz;
        return;
    }
    if (n2 < 0.01f * (w * w)) {
        float inv_w = 1.0f / w;
        float t2 = n2 * inv_w * inv_w;
        float s = (2.0f * inv_w) *
                  (1.0f - t2 * (1.0f / 3.0f - t2 * (1.0f / 5.0f - t2 * (1.0f / 7.0f - t2 * (1.0f / 9.0f)))));
        out[0] = s * vx; out[1] = s * vy; out[2] = s * vz;
        return;
    }
    n = sqrtf(n2);
    float angle = 2.0f * atan2f(n, w);
    float s = angle / n;
    out[0] = s * vx; out[1] = s * vy; out[2] = s * vz;
}

/* to_matrix(): R(q), body -> world */
void quat_to_matrix(const float q[4], float m[3][3])
{
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    const float ww = w * w, xx = x * x, yy = y * y, zz = z * z;
    m[0][0] = ww + xx - yy - zz;
    m[0][1] = 2.0f * (x * y - w * z);
    m[0][2] = 2.0f * (x * z + w * y);
    m[1][0] = 2.0f * (x * y + w * z);
    m[1][1] = ww - xx + yy - zz;
    m[1][2] = 2.0f * (y * z - w * x);
    m[2][0] = 2.0f * (x * z - w * y);
    m[2][1] = 2.0f * (y * z + w * x);
    m[2][2] = ww - xx - yy + zz;
}

void quat_rotate(const float q[4], const float v[3], float out[3])
{
    float m[3][3];
    quat_to_matrix(q, m);
    float r0 = m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2];
    float r1 = m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2];
    float r2 = m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2];
    out[0] = r0; out[1] = r1; out[2] = r2;
}

void quat_rotate_inverse(const float q[4], const float v[3], float out[3])
{
    float m[3][3];
    quat_to_matrix(q, m);
    float r0 = m[0][0] * v[0] + m[1][0] * v[1] + m[2][0] * v[2];
    float r1 = m[0][1] * v[0] + m[1][1] * v[1] + m[2][1] * v[2];
    float r2 = m[0][2] * v[0] + m[1][2] * v[1] + m[2][2] * v[2];
    out[0] = r0; out[1] = r1; out[2] = r2;
}

/* derivative(): 1/2 q (x) (0, omega) */
void quat_derivative(const float q[4], const float omega_b[3], float out[4])
{
    float p[4] = {0.0f, omega_b[0], omega_b[1], omega_b[2]};
    float r[4];
    quat_multiply(q, p, r);
    out[0] = 0.5f * r[0]; out[1] = 0.5f * r[1]; out[2] = 0.5f * r[2]; out[3] = 0.5f * r[3];
}

/* exp map by series: half angle x = |rv|/2, cos x and sin(x)/x to x^4 (exact in float for x < 0.1) */
static void exp_small_a2(const float rv[3], float a2, float out[4])
{
    float x2 = 0.25f * a2;
    float c = 1.0f - x2 * (1.0f / 2.0f - x2 * (1.0f / 24.0f));        /* cos x,     next term x^6/720  */
    float sinc = 1.0f - x2 * (1.0f / 6.0f - x2 * (1.0f / 120.0f));    /* sin(x)/x,  next term x^6/5040 */
    float h = 0.5f * sinc;               /* sin(|rv|/2) / |rv| */
    out[0] = c;
    out[1] = h * rv[0];
    out[2] = h * rv[1];
    out[3] = h * rv[2];
}

void quat_exp_small(const float rv[3], float out[4])
{
    exp_small_a2(rv, rv[0] * rv[0] + rv[1] * rv[1] + rv[2] * rv[2], out);
}

void quat_renormalize_near_unit(float q[4])
{
    float n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if (n2 > 0.9999f && n2 < 1.0001f) {
        float s = 1.5f - 0.5f * n2;      /* 1/sqrt(n2), Newton from 1: error 3/8 (n2-1)^2 < 4e-9 */
        q[0] *= s; q[1] *= s; q[2] *= s; q[3] *= s;
    } else {
        float t[4];
        quat_copy(q, t);
        quat_normalize(t, q);
    }
}

/* integrate(): normalize(q (x) exp(omega dt)) */
void quat_integrate(const float q[4], const float omega_b[3], float dt, float out[4])
{
    float rv[3] = {omega_b[0] * dt, omega_b[1] * dt, omega_b[2] * dt};
    float dq[4], r[4];
    float a2 = rv[0] * rv[0] + rv[1] * rv[1] + rv[2] * rv[2];
    if (a2 < QUAT_SERIES_MAX_ANGLE * QUAT_SERIES_MAX_ANGLE)
        exp_small_a2(rv, a2, dq);
    else
        quat_from_rotvec(rv, dq);
    quat_multiply(q, dq, r);
    quat_renormalize_near_unit(r);
    quat_copy(r, out);
}

/* error(): q_e = q^-1 (x) q_sp, w >= 0 */
void quat_error(const float q[4], const float q_sp[4], float out[4])
{
    float qc[4], qe[4];
    quat_conjugate(q, qc);
    quat_multiply(qc, q_sp, qe);
    if (qe[0] < 0.0f) {
        qe[0] = -qe[0]; qe[1] = -qe[1]; qe[2] = -qe[2]; qe[3] = -qe[3];
    }
    quat_copy(qe, out);
}

void quat_error_yaw_sp(const float q[4], const float q_sp[4], float out[4])
{
    /* conj(q) (x) (sw, 0, 0, sz) with conj(q) = (w, -x, -y, -z) */
    const float aw = q[0], ax = -q[1], ay = -q[2], az = -q[3], bw = q_sp[0], bz = q_sp[3];
    float qe[4];
    qe[0] = aw * bw - az * bz;
    qe[1] = ax * bw + ay * bz;
    qe[2] = -ax * bz + ay * bw;
    qe[3] = aw * bz + az * bw;
    if (qe[0] < 0.0f) {
        qe[0] = -qe[0]; qe[1] = -qe[1]; qe[2] = -qe[2]; qe[3] = -qe[3];
    }
    quat_copy(qe, out);
}

/* reduced_error(): 4 fmul + 2 fadd + 3 doublings */
void quat_reduced_error(const float qe[4], float out[3])
{
    const float w = qe[0], x = qe[1], y = qe[2], z = qe[3];
    const float ex = w * x - y * z, ey = w * y + x * z;
    out[0] = ex + ex;
    out[1] = ey + ey;
    out[2] = z + z;
}

/* yaw(): ZYX heading */
float quat_yaw(const float q[4])
{
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    return atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}

float quat_tilt_cos(const float q[4])
{
    const float x = q[1], y = q[2];
    return 1.0f - 2.0f * (x * x + y * y);
}

/* tilt(): acos(R[2][2]) */
float quat_tilt(const float q[4])
{
    return acosf(fc_clampf(quat_tilt_cos(q), -1.0f, 1.0f));
}

/* from_yaw(): upright with heading psi */
void quat_from_yaw(float psi_rad, float out[4])
{
    out[0] = cosf(0.5f * psi_rad);
    out[1] = 0.0f;
    out[2] = 0.0f;
    out[3] = sinf(0.5f * psi_rad);
}

/* from_euler(): ZYX */
void quat_from_euler(float roll, float pitch, float yaw, float out[4])
{
    static const float ez[3] = {0.0f, 0.0f, 1.0f}, ey[3] = {0.0f, 1.0f, 0.0f}, ex[3] = {1.0f, 0.0f, 0.0f};
    float qz[4], qy[4], qx[4], t[4], r[4];
    quat_from_axis_angle(ez, yaw, qz);
    quat_from_axis_angle(ey, pitch, qy);
    quat_from_axis_angle(ex, roll, qx);
    quat_multiply(qz, qy, t);
    quat_multiply(t, qx, r);
    quat_normalize(r, out);
}

/* to_euler(): ZYX (roll, pitch, yaw) */
void quat_to_euler(const float q[4], float *roll, float *pitch, float *yaw)
{
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    *roll = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
    *pitch = asinf(fc_clampf(2.0f * (w * y - z * x), -1.0f, 1.0f));
    *yaw = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}

/* tilt_vector(): rotvec of error(q, from_yaw(yaw(q))) */
void quat_tilt_vector(const float q[4], float out[3])
{
    float qy[4], qe[4];
    quat_from_yaw(quat_yaw(q), qy);
    quat_error(q, qy, qe);
    quat_to_rotvec(qe, out);
}
