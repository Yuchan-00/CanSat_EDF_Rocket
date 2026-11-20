/* pid.h - LowPass and RatePID of sim/controller.py (SPEC 8.6), single precision.
 *
 * Invariant: identical state, identical update order as the Python classes:
 *   LowPass:  y += a (x - y), a = dt / (1/(2 pi f_c) + dt); f_c <= 0 -> pass-through;
 *             the first sample primes y (no start-up transient).
 *   RatePID:  w_f = LPF_gyro(gyro); err = sp - w_f;
 *             if !freeze: I = clip(I + (Ki err) dt, +-I_max);
 *             w_d = LPF_d(w_f); d_rate = (w_d - d_prev)/dt (0 on the first call);
 *             M = (Kp err + I) + (-Kd d_rate)        [N m]
 * Constant products are folded at init (Ki dt, Kd/dt); the result differs from
 * the Python order of operations only by float rounding (cross-checked).
 */
#ifndef FC_PID_H
#define FC_PID_H

#include "fcmath.h"

typedef struct {
    float alpha;
    float y[3];
    bool primed;
} lowpass3_t;

void lowpass3_init(lowpass3_t *lp, float f_cut_hz, float dt);
void lowpass3_reset(lowpass3_t *lp);
void lowpass3_update(lowpass3_t *lp, const float x[3], float out[3]);

typedef struct {
    float rate_kp_xy, rate_ki_xy, rate_kd_xy, rate_i_max_xy;
    float rate_kp_z, rate_ki_z, rate_kd_z, rate_i_max_z;
    float d_term_lpf_hz, gyro_lpf_hz;
} rate_gains_t;

typedef struct {
    float dt, inv_dt;
    float kp[3], ki[3], kd[3], i_max[3];
    float ki_dt[3], kd_inv_dt[3];   /* ki dt and kd / dt, folded once in rate_pid_init */
    lowpass3_t gyro_lpf, d_lpf;
    float integ[3];
    float d_prev[3];
    bool d_primed;
    float last_p[3], last_d[3];
} rate_pid_t;

void rate_pid_init(rate_pid_t *pid, const rate_gains_t *g, float dt);
void rate_pid_reset(rate_pid_t *pid);
/* moment_out [N m] and the filtered gyro w_f_out [rad/s] */
void rate_pid_update(rate_pid_t *pid, const float rate_sp[3], const float gyro[3], bool freeze_integrator,
                     float moment_out[3], float w_f_out[3]);


#endif /* FC_PID_H */
