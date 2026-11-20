/* att_estimator.c - see att_estimator.h (mirrors sim/estimator.py PadAlignGyroEstimator). */
#include "att_estimator.h"
#include "quat.h"

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
