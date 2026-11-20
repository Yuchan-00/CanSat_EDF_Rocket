/* att_estimator.h - the flight computer's attitude estimator, a transcription of
 * sim/estimator.py PadAlignGyroEstimator (the simulation's "pad_gyro" mode).
 *
 * Called once per control tick with the bias-uncorrected gyro g [rad/s] and the
 * specific force a [m/s^2], both in the body frame:
 *
 *   on the pad (mission state IDLE or ARMED, the vehicle is static):
 *       n      = min(n + 1, N_avg)                 N_avg = round(pad_avg_tau_s / dt)
 *       b_hat += (g - b_hat) / n                   running mean, then EMA alpha = 1/N_avg
 *       f_hat += (a - f_hat) / n
 *       q_hat  = level_from_accel(f_hat):  roll = atan2(f_y, f_z),
 *                pitch = atan2(-f_x, hypot(f_y, f_z)),  q = q_zyx(roll, pitch, yaw = 0)
 *   after launch (every other state): gyro only, bias frozen, accelerometer unused
 *       q_hat  = integrate(q_hat, g - b_hat, dt)
 *   rate for the rate PID: g - b_hat
 *
 * Invariant: the caller decides on_pad from the mission state of the previous
 * tick (as the simulation does); nothing here reads the accelerometer once
 * launched - in powered flight it measures thrust, not gravity.  The trig of
 * the levelling runs only on the pad, where the controller is idle.
 */
#ifndef FC_ATT_ESTIMATOR_H
#define FC_ATT_ESTIMATOR_H

#include "fcmath.h"

typedef struct {
    float dt;
    uint16_t n_avg, n;
    float bias[3];
    float f_mean[3];
    float q[4];
    bool aligned;
} att_est_t;

void att_est_init(att_est_t *e, float dt, uint16_t n_avg);
void att_est_level_from_accel(const float f_b[3], float q[4]);
void att_est_update(att_est_t *e, const float gyro[3], const float accel[3], bool on_pad);
void att_est_rate(const att_est_t *e, const float gyro[3], float rate[3]);

#endif /* FC_ATT_ESTIMATOR_H */
