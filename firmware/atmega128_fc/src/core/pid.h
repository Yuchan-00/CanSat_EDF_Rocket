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
#include "link_proto.h"   /* FC_VTOL_RAM_TRIM */

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
#if FC_VTOL_RAM_TRIM           /* dt, inv_dt, ki, kd are init-only: dropped in the VTOL AVR image (-32 B) */
    float kp[3], i_max[3];
#else
    float dt, inv_dt;
    float kp[3], ki[3], kd[3], i_max[3];
#endif
    float ki_dt[3], kd_inv_dt[3];   /* ki dt and kd / dt, folded once in rate_pid_init */
    lowpass3_t gyro_lpf, d_lpf;
    float integ[3];
    float d_prev[3];
    bool d_primed;
#if !FC_VTOL_RAM_TRIM          /* diagnostics of the last update (no reader on the AVR: dropped in the VTOL image, -24 B) */
    float last_p[3], last_d[3];
#endif
} rate_pid_t;

void rate_pid_init(rate_pid_t *pid, const rate_gains_t *g, float dt);
void rate_pid_reset(rate_pid_t *pid);
/* moment_out [N m] and the filtered gyro w_f_out [rad/s] */
void rate_pid_update(rate_pid_t *pid, const float rate_sp[3], const float gyro[3], bool freeze_integrator,
                     float moment_out[3], float w_f_out[3]);

#if FC_V09
/* v0.9 per-axis conditional anti-windup (sim/controller.py RatePID.update(freeze_fn=), AttitudeController._freeze_mask):
 * axis i's integrator is frozen when freeze_all (the thrust estimate floored; with aw_per_axis 0 also the previous tick's
 * saturation - the v0.8.1 rule), or when the mixer limited that axis in the previous tick (aw RPID_AW_LIM_XY for pitch /
 * roll, RPID_AW_LIM_Z for yaw), the axis' previous moment demand is nonzero and this tick's rate error has its sign
 * (err m > 0: integrating pushes further into the limit) - decided on the float sign bits, no multiply.  An error of 0
 * integrates nothing, frozen or not, so its sign bit never matters.  An error that opposes the demand always integrates
 * (it unwinds).  aw = 0 and freeze_all = false: every axis integrates (the v0.8.1 unsaturated path, the worst case). */
#define RPID_AW_NEG(i) ((uint8_t)(0x01u << (i)))      /* the previous moment of axis i is negative (sign bit) */
#define RPID_AW_NZ(i) ((uint8_t)(0x08u << (i)))       /* ... and nonzero */
#define RPID_AW_LIM_XY 0x40u                          /* the mixer scaled (d_x, d_y) in the previous tick */
#define RPID_AW_LIM_Z 0x80u                           /* the mixer limited d_z (= saturated) */
void rate_pid_update_aw(rate_pid_t *pid, const float rate_sp[3], const float gyro[3], bool freeze_all, uint8_t aw,
                        float moment_out[3], float w_f_out[3]);
uint8_t rate_pid_aw_signs(const float m[3]);          /* the RPID_AW_NEG / RPID_AW_NZ bits of a moment vector */
#endif

#endif /* FC_PID_H */
