/* pid.c - see pid.h (mirrors sim/controller.py LowPass, RatePID). */
#include "pid.h"

void lowpass3_init(lowpass3_t *lp, float f_cut_hz, float dt)
{
    lp->alpha = (f_cut_hz <= 0.0f) ? 1.0f : dt / (1.0f / (2.0f * FC_PI_F * f_cut_hz) + dt);
    lowpass3_reset(lp);
}

void lowpass3_reset(lowpass3_t *lp)
{
    lp->y[0] = lp->y[1] = lp->y[2] = 0.0f;
    lp->primed = false;
}

void lowpass3_update(lowpass3_t *lp, const float x[3], float out[3])
{
    int i;
    if (!lp->primed) {
        for (i = 0; i < 3; i++)
            lp->y[i] = x[i];
        lp->primed = true;
    } else {
        for (i = 0; i < 3; i++)
            lp->y[i] = lp->y[i] + lp->alpha * (x[i] - lp->y[i]);
    }
    for (i = 0; i < 3; i++)
        out[i] = lp->y[i];
}

void rate_pid_init(rate_pid_t *pid, const rate_gains_t *g, float dt)
{
    pid->dt = dt;
    pid->inv_dt = 1.0f / dt;
    pid->kp[0] = pid->kp[1] = g->rate_kp_xy; pid->kp[2] = g->rate_kp_z;
    pid->ki[0] = pid->ki[1] = g->rate_ki_xy; pid->ki[2] = g->rate_ki_z;
    pid->kd[0] = pid->kd[1] = g->rate_kd_xy; pid->kd[2] = g->rate_kd_z;
    pid->i_max[0] = pid->i_max[1] = g->rate_i_max_xy; pid->i_max[2] = g->rate_i_max_z;
    {
        int i;
        for (i = 0; i < 3; i++) {
            pid->ki_dt[i] = pid->ki[i] * dt;
            pid->kd_inv_dt[i] = pid->kd[i] * pid->inv_dt;
        }
    }
    lowpass3_init(&pid->gyro_lpf, g->gyro_lpf_hz, dt);
    lowpass3_init(&pid->d_lpf, g->d_term_lpf_hz, dt);
    rate_pid_reset(pid);
    pid->last_p[0] = pid->last_p[1] = pid->last_p[2] = 0.0f;
    pid->last_d[0] = pid->last_d[1] = pid->last_d[2] = 0.0f;
}

void rate_pid_reset(rate_pid_t *pid)
{
    int i;
    lowpass3_reset(&pid->gyro_lpf);
    lowpass3_reset(&pid->d_lpf);
    for (i = 0; i < 3; i++) {
        pid->integ[i] = 0.0f;
        pid->d_prev[i] = 0.0f;
    }
    pid->d_primed = false;
}

void rate_pid_update(rate_pid_t *pid, const float rate_sp[3], const float gyro[3], bool freeze_integrator,
                     float moment_out[3], float w_f_out[3])
{
    float w_f[3], err[3], w_d[3], d_rate[3];
    int i;
    lowpass3_update(&pid->gyro_lpf, gyro, w_f);
    for (i = 0; i < 3; i++)
        err[i] = rate_sp[i] - w_f[i];
    if (!freeze_integrator) {
        for (i = 0; i < 3; i++)
            pid->integ[i] = fc_clampf(pid->integ[i] + pid->ki_dt[i] * err[i], -pid->i_max[i], pid->i_max[i]);
    }
    lowpass3_update(&pid->d_lpf, w_f, w_d);
    if (pid->d_primed) {
        for (i = 0; i < 3; i++)
            d_rate[i] = w_d[i] - pid->d_prev[i];          /* times 1/dt folded into kd_inv_dt */
    } else {
        for (i = 0; i < 3; i++)
            d_rate[i] = 0.0f;
        pid->d_primed = true;
    }
    for (i = 0; i < 3; i++) {
        pid->d_prev[i] = w_d[i];
        pid->last_p[i] = pid->kp[i] * err[i];
        pid->last_d[i] = -pid->kd_inv_dt[i] * d_rate[i];  /* -Kd d(w_d)/dt: D on the measurement */
        moment_out[i] = pid->last_p[i] + pid->integ[i] + pid->last_d[i];
        w_f_out[i] = w_f[i];
    }
}

