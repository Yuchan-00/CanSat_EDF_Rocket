/* attitude_ctrl.h - AttitudeController of sim/controller.py (SPEC 8.6).
 *
 * update(q_est, gyro, u):
 *   q_sp    = from_yaw(yaw_ref) (yaw hold) or from_yaw(yaw(q_est))
 *   e       = to_rotvec(error(q_est, q_sp))
 *   rate_sp = clip(att_kp e, +-rate_sp_max)
 *   T_hat   = thrust_estimate(u); freeze = last_saturated or thrust_floored(u)
 *   M, w_f  = RatePID(rate_sp, gyro, freeze)
 *   delta, saturated = deflections(M, T_hat); last_saturated = saturated
 *
 * The rocket's controller only: the VTOL program's tilted set point (v0.8, GPS position hold) and its v0.9 options
 * (reduced attitude error, per-axis anti-windup, rotor feed-forwards, yaw-trim reserve) went with the VTOL vehicle
 * (2026-11-18 / 2026-11-19); their description is in this file at git e1c9d1d.
 *
 * Invariant: same state and order as the Python class.  With yaw hold the
 * setpoint quaternion is computed once in set_yaw_reference (cos/sin there,
 * not in the 250 Hz path).
 */
#ifndef FC_ATTITUDE_CTRL_H
#define FC_ATTITUDE_CTRL_H

#include "fcmath.h"
#include "mixer.h"
#include "pid.h"
#include "quat.h"


typedef struct {
    int   loop_hz;
    float att_kp_xy, att_kp_z, rate_sp_max_xy_rad_s, rate_sp_max_z_rad_s;
    rate_gains_t rate;
    float thrust_estimate_floor_fraction, roll_priority_scale;
    bool  yaw_hold;
} att_gains_t;

typedef struct {               /* ControllerOutput */
    float delta_cmd[4];
    float moments[3];
    float rate_sp[3];
    float att_error[3];
    float gyro_filtered[3];
    bool  saturated;
    float thrust_est;
    float pwm_us[4];
    float integrator[3];
} ctrl_output_t;


typedef struct {
    float dt;
    float att_kp[3];
    float rate_sp_max[3];
    bool  yaw_hold;
    rate_pid_t pid;
    vane_mixer_t mixer;
    float yaw_ref;
    float q_sp_hold[4];
    bool  last_saturated;
} attitude_ctrl_t;


void attitude_ctrl_init(attitude_ctrl_t *c, const att_gains_t *g, const mixer_plant_t *plant, int throttle_exp_int);
void attitude_ctrl_reset(attitude_ctrl_t *c);
void attitude_ctrl_set_yaw_reference(attitude_ctrl_t *c, const float q_est[4]);
void attitude_ctrl_setpoint(const attitude_ctrl_t *c, const float q_est[4], float q_sp[4]);
void attitude_ctrl_update(attitude_ctrl_t *c, const float q_est[4], const float gyro[3], float throttle,
                          ctrl_output_t *out);

#endif /* FC_ATTITUDE_CTRL_H */
