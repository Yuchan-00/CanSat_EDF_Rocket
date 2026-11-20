/* quat.h - quaternion utilities, a 1:1 transcription of sim/quat.py.
 *
 * Convention (SPEC 8.1): q = (w, x, y, z) stored as float q[4], unit norm,
 * Hamilton product; R(q) maps body vectors into the world frame; body rates
 * integrate as dq/dt = 1/2 q (x) (0, omega_b).
 *
 * Invariant: every function is pure (inputs may alias outputs only where
 * noted), single precision, and keeps the name of its Python counterpart with
 * a quat_ prefix.  quat_integrate replaces sin/cos/sqrt by series that are
 * exact to float precision for |omega dt| < QUAT_SERIES_MAX_ANGLE (the 250 Hz
 * path never needs trig there); larger steps fall back to the exact form.
 */
#ifndef FC_QUAT_H
#define FC_QUAT_H

#include "fcmath.h"

#define QUAT_SERIES_MAX_ANGLE 0.2f   /* rad; truncation < 1.5e-9 below this; 2000 deg/s x 4 ms = 0.14 rad */

void  quat_identity(float q[4]);
void  quat_copy(const float a[4], float out[4]);
void  quat_normalize(const float q[4], float out[4]);
void  quat_multiply(const float a[4], const float b[4], float out[4]);  /* out must not alias a or b */
void  quat_conjugate(const float q[4], float out[4]);
void  quat_from_axis_angle(const float axis[3], float angle_rad, float out[4]);
void  quat_from_rotvec(const float rv[3], float out[4]);
void  quat_to_rotvec(const float q[4], float out[3]);
void  quat_to_matrix(const float q[4], float m[3][3]);
void  quat_rotate(const float q[4], const float v[3], float out[3]);
void  quat_rotate_inverse(const float q[4], const float v[3], float out[3]);
void  quat_derivative(const float q[4], const float omega_b[3], float out[4]);
void  quat_integrate(const float q[4], const float omega_b[3], float dt, float out[4]);
void  quat_error(const float q[4], const float q_sp[4], float out[4]);
/* error() for a pure-yaw setpoint q_sp = (w, 0, 0, z): the zero products dropped, same result */
void  quat_error_yaw_sp(const float q[4], const float q_sp[4], float out[4]);
/* v0.9 (v0.8.2 candidate D): reduced attitude error of q_e (w >= 0): e = (2(w x - y z), 2(w y + x z), 2 z) (sim/quat.py
 * reduced_error): the tilt part is body z x the set point z (a heading error does not turn it), no division / sqrt / trig */
void  quat_reduced_error(const float qe[4], float out[3]);
float quat_yaw(const float q[4]);
float quat_tilt(const float q[4]);
float quat_tilt_cos(const float q[4]);          /* R[2][2] = cos(tilt), no trig */
void  quat_from_yaw(float psi_rad, float out[4]);
void  quat_from_euler(float roll, float pitch, float yaw, float out[4]);
void  quat_to_euler(const float q[4], float *roll, float *pitch, float *yaw);
void  quat_tilt_vector(const float q[4], float out[3]);

/* exp map for small rotation vectors (series, no trig/sqrt); valid for |rv| < QUAT_SERIES_MAX_ANGLE */
void  quat_exp_small(const float rv[3], float out[4]);
/* normalise in place: one Newton step (no sqrt/div) when |q|^2 is within 1e-4 of 1, else exact */
void  quat_renormalize_near_unit(float q[4]);

#endif /* FC_QUAT_H */
