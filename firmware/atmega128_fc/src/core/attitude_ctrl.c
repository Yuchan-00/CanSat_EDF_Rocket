/* attitude_ctrl.c - see attitude_ctrl.h (mirrors sim/controller.py AttitudeController).
 *
 * The rocket's (v0.8.1 ballistic) functions only; the VTOL v0.9 options left with the VTOL program (attitude_ctrl.h). */
#include "attitude_ctrl.h"

#include <string.h>


void attitude_ctrl_init(attitude_ctrl_t *c, const att_gains_t *g, const mixer_plant_t *plant, int throttle_exp_int)
{
    c->dt = 1.0f / (float)g->loop_hz;
    c->att_kp[0] = c->att_kp[1] = g->att_kp_xy;
    c->att_kp[2] = g->att_kp_z;
    c->rate_sp_max[0] = c->rate_sp_max[1] = g->rate_sp_max_xy_rad_s;
    c->rate_sp_max[2] = g->rate_sp_max_z_rad_s;
    c->yaw_hold = g->yaw_hold;
    rate_pid_init(&c->pid, &g->rate, c->dt);
    mixer_init(&c->mixer, plant, g->thrust_estimate_floor_fraction, g->roll_priority_scale, throttle_exp_int);
    c->yaw_ref = 0.0f;
    quat_from_yaw(0.0f, c->q_sp_hold);
    c->last_saturated = false;
}


void attitude_ctrl_reset(attitude_ctrl_t *c)
{
    rate_pid_reset(&c->pid);
    c->last_saturated = false;
}


/* set_yaw_reference(): capture the arming heading */
void attitude_ctrl_set_yaw_reference(attitude_ctrl_t *c, const float q_est[4])
{
    c->yaw_ref = quat_yaw(q_est);
    quat_from_yaw(c->yaw_ref, c->q_sp_hold);
}

/* setpoint() */
void attitude_ctrl_setpoint(const attitude_ctrl_t *c, const float q_est[4], float q_sp[4])
{
    if (c->yaw_hold)
        quat_copy(c->q_sp_hold, q_sp);
    else
        quat_from_yaw(quat_yaw(q_est), q_sp);
}

/* update() */
void attitude_ctrl_update(attitude_ctrl_t *c, const float q_est[4], const float gyro[3], float throttle,
                          ctrl_output_t *out)
{
    float q_sp[4], q_e[4], moments[3], w_f[3];
    int i;
    float t_model;
    bool freeze;
    attitude_ctrl_setpoint(c, q_est, q_sp);
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
