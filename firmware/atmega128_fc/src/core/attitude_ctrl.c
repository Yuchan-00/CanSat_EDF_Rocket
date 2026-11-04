/* attitude_ctrl.c - see attitude_ctrl.h (mirrors sim/controller.py AttitudeController).
 *
 * v0.9 (FC_V09 builds): the reduced attitude error (att_err_mode, the v0.8.2 candidate D port B), the per-axis
 * conditional anti-windup, the rotor feed-forwards and the mixer's yaw-trim reserve - attitude_ctrl.h.  A ballistic fixed
 * build compiles the v0.8.1 functions below unchanged (its image keeps its bytes). */
#include "attitude_ctrl.h"

#include <string.h>

#if FC_V09
#if FC_VTOL_RAM_TRIM
#define V9(c, f) FC_FLASH_RD(fc_att_v09_flash.f)
#else
#define V9(c, f) ((c)->v9.f)
#endif
#endif

void attitude_ctrl_init(attitude_ctrl_t *c, const att_gains_t *g, const mixer_plant_t *plant, int throttle_exp_int)
{
#if FC_V09 && FC_VTOL_RAM_TRIM
    const float dt = 1.0f / (float)g->loop_hz;
#else
    c->dt = 1.0f / (float)g->loop_hz;
#endif
    c->att_kp[0] = c->att_kp[1] = g->att_kp_xy;
    c->att_kp[2] = g->att_kp_z;
    c->rate_sp_max[0] = c->rate_sp_max[1] = g->rate_sp_max_xy_rad_s;
    c->rate_sp_max[2] = g->rate_sp_max_z_rad_s;
    c->yaw_hold = g->yaw_hold;
#if FC_V09 && FC_VTOL_RAM_TRIM
    rate_pid_init(&c->pid, &g->rate, dt);
#else
    rate_pid_init(&c->pid, &g->rate, c->dt);
#endif
    mixer_init(&c->mixer, plant, g->thrust_estimate_floor_fraction, g->roll_priority_scale, throttle_exp_int);
    c->yaw_ref = 0.0f;
    quat_from_yaw(0.0f, c->q_sp_hold);
#if FC_V09
    c->aw = 0;
    c->tilt_sp[0] = c->tilt_sp[1] = 0.0f;
    c->q_t[0] = 1.0f;
    c->q_t[1] = c->q_t[2] = 0.0f;
    c->tilt_on = false;
    c->u_lag = -1.0f;                                  /* not primed */
#if !FC_VTOL_RAM_TRIM
    c->v9 = g->v9;
#endif
#else
    c->last_saturated = false;
#if ATT_HAS_TILT_SP
    c->tilt_sp[0] = c->tilt_sp[1] = 0.0f;
    quat_copy(c->q_sp_hold, c->q_sp_tilt);
    c->tilt_on = false;
    c->tilt_half = false;
#endif
#endif
}

#if ATT_HAS_TILT_SP
/* q(t) = (w, k tx, k ty) by series */
static void tilt_series(const float tilt[2], float wk[3])
{
    const float tx = tilt[0], ty = tilt[1];
    const float th2 = tx * tx + ty * ty;
    const float k = (th2 * (1.0f / 3840.0f) - (1.0f / 48.0f)) * th2 + 0.5f;   /* Horner, float32 reciprocals */
    wk[0] = (th2 * (1.0f / 384.0f) - 0.125f) * th2 + 1.0f;
    wk[1] = k * tx;
    wk[2] = k * ty;
}

/* q(t) (x) (c, 0, 0, s) */
static void tilt_times_yaw(const float wk[3], const float q_yaw[4], float q_sp[4])
{
    const float w = wk[0], kx = wk[1], ky = wk[2], c = q_yaw[0], s = q_yaw[3];
    q_sp[0] = w * c;
    q_sp[1] = kx * c + ky * s;
    q_sp[2] = ky * c - kx * s;
    q_sp[3] = w * s;
}

void attitude_ctrl_tilt_quat(const float tilt[2], const float q_yaw[4], float q_sp[4])
{
    float wk[3];
    tilt_series(tilt, wk);
    tilt_times_yaw(wk, q_yaw, q_sp);
}

#if FC_V09
void attitude_ctrl_set_tilt_finish(attitude_ctrl_t *c)
{
    (void)c;                                           /* v0.8.2 D port B: nothing left to finish (kept for the callers) */
}
#else
void attitude_ctrl_set_tilt_finish(attitude_ctrl_t *c)
{
    if (c->tilt_half) {                                /* q_sp_tilt[0..2] holds q(t) = (w, k tx, k ty) */
        const float wk[3] = {c->q_sp_tilt[0], c->q_sp_tilt[1], c->q_sp_tilt[2]};
        tilt_times_yaw(wk, c->q_sp_hold, c->q_sp_tilt);
        c->tilt_half = false;
    }
}
#endif

/* q_sp_tilt = tilt_quat(tilt_sp, q_sp_hold) holds whenever tilt_on && yaw_hold (here and in set_yaw_reference; yaw_hold
 * is fixed at init), so the same tilt bits again (the builder settled: most ticks) leave everything as it is */
static uint32_t f32_bits(float x)
{
    uint32_t u;
    memcpy(&u, &x, sizeof u);
    return u;
}

#if FC_V09
bool attitude_ctrl_set_tilt(attitude_ctrl_t *c, const float tilt[2])
{
    const uint32_t a = f32_bits(tilt[0]), b = f32_bits(tilt[1]);
    if (a == f32_bits(c->tilt_sp[0]) && b == f32_bits(c->tilt_sp[1]))
        return false;
    c->tilt_sp[0] = tilt[0];
    c->tilt_sp[1] = tilt[1];
    c->tilt_on = ((a | b) & 0x7FFFFFFFUL) != 0u;      /* != (0, 0) as the float compare: -0 is 0, a NaN is on */
    if (c->tilt_on)
        tilt_series(c->tilt_sp, c->q_t);               /* the whole background work: no second piece (port B) */
    return false;
}
#else
bool attitude_ctrl_set_tilt(attitude_ctrl_t *c, const float tilt[2])
{
    const uint32_t a = f32_bits(tilt[0]), b = f32_bits(tilt[1]);
    attitude_ctrl_set_tilt_finish(c);                  /* a previous half first */
    if (a == f32_bits(c->tilt_sp[0]) && b == f32_bits(c->tilt_sp[1]))
        return false;
    c->tilt_sp[0] = tilt[0];
    c->tilt_sp[1] = tilt[1];
    c->tilt_on = ((a | b) & 0x7FFFFFFFUL) != 0u;      /* != (0, 0) as the float compare: -0 is 0, a NaN is on */
    if (!(c->tilt_on && c->yaw_hold))
        return false;
    tilt_series(c->tilt_sp, c->q_sp_tilt);             /* the first half; attitude_ctrl_set_tilt_finish the second */
    c->tilt_half = true;
    return true;
}
#endif
#endif

void attitude_ctrl_reset(attitude_ctrl_t *c)
{
    rate_pid_reset(&c->pid);
#if FC_V09
    c->aw = 0;                                         /* last_saturated = false, last_moments = 0 */
    c->u_lag = -1.0f;                                  /* u_lag_primed = false */
#else
    c->last_saturated = false;
#endif
}

#if FC_V09
bool attitude_ctrl_last_saturated(const attitude_ctrl_t *c)
{
    return (c->aw & ATT_AW_SAT) != 0u;
}
#endif

/* set_yaw_reference(): capture the arming heading */
void attitude_ctrl_set_yaw_reference(attitude_ctrl_t *c, const float q_est[4])
{
    c->yaw_ref = quat_yaw(q_est);
    quat_from_yaw(c->yaw_ref, c->q_sp_hold);
#if ATT_HAS_TILT_SP && !FC_V09
    if (c->tilt_on && c->yaw_hold) {                   /* the tilted setpoint follows the new heading */
        attitude_ctrl_tilt_quat(c->tilt_sp, c->q_sp_hold, c->q_sp_tilt);
        c->tilt_half = false;
    }
#endif                                                 /* v0.9 port B: q_t has no yaw - nothing to rebuild */
}

/* setpoint() */
void attitude_ctrl_setpoint(const attitude_ctrl_t *c, const float q_est[4], float q_sp[4])
{
#if ATT_HAS_TILT_SP
    if (c->tilt_on) {                                  /* v0.8: q(tilt) (x) q_yaw */
        if (c->yaw_hold) {
#if FC_V09
            tilt_times_yaw(c->q_t, c->q_sp_hold, q_sp);
#else
            quat_copy(c->q_sp_tilt, q_sp);
#endif
        } else {
            float q_y[4];
            quat_from_yaw(quat_yaw(q_est), q_y);
            attitude_ctrl_tilt_quat(c->tilt_sp, q_y, q_sp);
        }
        return;
    }
#endif
    if (c->yaw_hold)
        quat_copy(c->q_sp_hold, q_sp);
    else
        quat_from_yaw(quat_yaw(q_est), q_sp);
}

#if FC_V09
/* the reduced error against q(t) (x) (c, 0, 0, s) without forming the set point (module docstring, port B) */
static void tilt_hold_error(const attitude_ctrl_t *c, const float q[4], float e[3]) __attribute__((noinline));
static void tilt_hold_error(const attitude_ctrl_t *c, const float q[4], float e[3])
{
    const float tw = c->q_t[0], tx = c->q_t[1], ty = c->q_t[2];
    const float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    const float yc = c->q_sp_hold[0], ys = c->q_sp_hold[3];
    const float pw = qw * tw + qx * tx + qy * ty;
    const float px = qw * tx - qx * tw + qz * ty;
    const float py = qw * ty - qy * tw - qz * tx;
    const float pz = qy * tx - qx * ty - qz * tw;
    const float ex = pw * px - py * pz, ey = pw * py + px * pz;
    float ze = pw * ys + pz * yc;
    if (pw * yc - pz * ys < 0.0f)                      /* w_e < 0: quat_error's sign rule (w >= 0) */
        ze = -ze;
    e[0] = ex + ex;
    e[1] = ey + ey;
    e[2] = ze + ze;
}

/* v0.9 att_err_mode 2 without a tilt, yaw hold: the same with q(t) = 1 - p = q^-1 = (w, -x, -y, -z):
 * e_x = 2 (-w x - y z), e_y = 2 (x z - w y), z_e = w s - z c, w_e = w c + z s */
static void level_hold_error(const attitude_ctrl_t *c, const float q[4], float e[3]) __attribute__((noinline));
static void level_hold_error(const attitude_ctrl_t *c, const float q[4], float e[3])
{
    const float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    const float yc = c->q_sp_hold[0], ys = c->q_sp_hold[3];
    const float ex = -(qw * qx) - qy * qz, ey = qx * qz - qw * qy;
    float ze = qw * ys - qz * yc;
    if (qw * yc + qz * ys < 0.0f)                      /* w_e < 0: quat_error's sign rule */
        ze = -ze;
    e[0] = ex + ex;
    e[1] = ey + ey;
    e[2] = ze + ze;
}

/* update() with the v0.9 options (sim/controller.py AttitudeController.update, step by step) */
void attitude_ctrl_update(attitude_ctrl_t *c, const float q_est[4], const float gyro[3], float throttle,
                          ctrl_output_t *out)
{
    float moments[3], w_f[3];
    int i;
    float t_model;
    bool floored;
    uint8_t fl;
    const uint8_t mode = V9(c, err_mode), v9f = V9(c, flags);
    /* error: reduced (mode 2, or mode 1 with a tilt) or the rotation vector */
    if (mode == 2u || (mode == 1u && c->tilt_on)) {
        if (c->yaw_hold) {
            if (c->tilt_on)
                tilt_hold_error(c, q_est, out->att_error);   /* port B: the set point product folded in */
            else
                level_hold_error(c, q_est, out->att_error);
        } else {
            float q_sp[4], q_e[4];
            attitude_ctrl_setpoint(c, q_est, q_sp);
            if (c->tilt_on)
                quat_error(q_est, q_sp, q_e);
            else
                quat_error_yaw_sp(q_est, q_sp, q_e);
            quat_reduced_error(q_e, out->att_error);
        }
    } else {
        float q_sp[4], q_e[4];
        attitude_ctrl_setpoint(c, q_est, q_sp);
        if (c->tilt_on)
            quat_error(q_est, q_sp, q_e);              /* v0.8 tilted setpoint: x, y != 0 */
        else
            quat_error_yaw_sp(q_est, q_sp, q_e);       /* q_sp = from_yaw(.) has x = y = 0 */
        quat_to_rotvec(q_e, out->att_error);
    }
    for (i = 0; i < 3; i++)
        out->rate_sp[i] = fc_clampf(c->att_kp[i] * out->att_error[i], -c->rate_sp_max[i], c->rate_sp_max[i]);
    t_model = mixer_thrust_model(&c->mixer, throttle);                   /* evaluated once: */
    out->thrust_est = fc_maxf(t_model, c->mixer.floor_thrust_n);         /*   thrust_estimate() */
    floored = t_model < c->mixer.floor_thrust_n;                         /*   thrust_floored()  */
    if (v9f & ATT_V9_AW)                               /* per-axis: the previous tick's limits and moment signs */
        rate_pid_update_aw(&c->pid, out->rate_sp, gyro, floored, c->aw, moments, w_f);
    else                                               /* v0.8.1: every integrator frozen after a saturated tick */
        rate_pid_update_aw(&c->pid, out->rate_sp, gyro, (c->aw & ATT_AW_SAT) != 0u || floored, 0u, moments, w_f);
    if (v9f & ATT_V9_FF) {                             /* rotor feed-forwards (after the PID, before the mixer) */
        const float u = fc_minf(fc_maxf(throttle, 0.0f), 1.0f);
        float du;
        if (c->u_lag < 0.0f)                           /* the first sample after a reset primes the lag */
            c->u_lag = u;
        du = u - c->u_lag;
        c->u_lag += V9(c, ff_a) * du;
        if (v9f & ATT_V9_REACT)                        /* spool-up reaction: -h_ref du/dt / tau / roll_priority_scale */
            moments[2] = moments[2] + V9(c, ff_nkr) * du;
        if (v9f & ATT_V9_GYRO) {                       /* gyroscopic decoupling, h_z = -h_ref u_lag */
            const float hz = -V9(c, ff_h) * c->u_lag;
            moments[0] = moments[0] + w_f[1] * hz;
            moments[1] = moments[1] + -w_f[0] * hz;
        }
    }
    fl = mixer_deflections_v9(&c->mixer, moments, out->thrust_est, (v9f & ATT_V9_TRIM) != 0u, c->pid.integ[2],
                              out->delta_cmd);
    out->saturated = (fl & MIX_SAT) != 0u;
    c->aw = (uint8_t)(((fl & MIX_SAT) ? ATT_AW_SAT : 0u) | ((fl & MIX_LIM_XY) ? RPID_AW_LIM_XY : 0u) |
                      rate_pid_aw_signs(moments));
    mixer_pwm(&c->mixer, out->delta_cmd, out->pwm_us);
    for (i = 0; i < 3; i++) {
        out->moments[i] = moments[i];
        out->gyro_filtered[i] = w_f[i];
        out->integrator[i] = c->pid.integ[i];
    }
}
#else
/* update() */
void attitude_ctrl_update(attitude_ctrl_t *c, const float q_est[4], const float gyro[3], float throttle,
                          ctrl_output_t *out)
{
    float q_sp[4], q_e[4], moments[3], w_f[3];
    int i;
    float t_model;
    bool freeze;
#if ATT_HAS_TILT_SP
    if (c->tilt_half)                                  /* a setpoint left half-built by the background: finish it */
        attitude_ctrl_set_tilt_finish(c);
#endif
    attitude_ctrl_setpoint(c, q_est, q_sp);
#if ATT_HAS_TILT_SP
    if (c->tilt_on)
        quat_error(q_est, q_sp, q_e);                  /* v0.8 tilted setpoint: x, y != 0 */
    else
#endif
        quat_error_yaw_sp(q_est, q_sp, q_e);           /* q_sp = from_yaw(.) has x = y = 0 */
    quat_to_rotvec(q_e, out->att_error);
    for (i = 0; i < 3; i++)
        out->rate_sp[i] = fc_clampf(c->att_kp[i] * out->att_error[i], -c->rate_sp_max[i], c->rate_sp_max[i]);
    t_model = mixer_thrust_model(&c->mixer, throttle);                   /* evaluated once: */
    out->thrust_est = fc_maxf(t_model, c->mixer.floor_thrust_n);         /*   thrust_estimate() */
    freeze = c->last_saturated || t_model < c->mixer.floor_thrust_n;     /*   thrust_floored()  */
    rate_pid_update(&c->pid, out->rate_sp, gyro, freeze, moments, w_f);
    out->saturated = mixer_deflections(&c->mixer, moments, out->thrust_est, out->delta_cmd);
    c->last_saturated = out->saturated;
    mixer_pwm(&c->mixer, out->delta_cmd, out->pwm_us);
    for (i = 0; i < 3; i++) {
        out->moments[i] = moments[i];
        out->gyro_filtered[i] = w_f[i];
        out->integrator[i] = c->pid.integ[i];
    }
}
#endif
