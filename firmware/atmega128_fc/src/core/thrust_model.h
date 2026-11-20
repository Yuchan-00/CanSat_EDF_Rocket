/* thrust_model.h - EDFModel of sim/edf.py (SPEC 8.7), the firmware's thrust estimate.
 *
 *   T_cmd(u) = T_max clip(u,0,1)^n                     thrust_steady
 *   T <- T + (T_cmd - T)(1 - exp(-dt/tau))              step (exact for u held over dt)
 *   V_e = sqrt(T / (rho A_e)),  q_e = T / (2 A_e)        exhaust_velocity, dynamic_pressure
 *   tau_swirl = sign f (P_nom/omega_max) T/T_max         swirl_torque
 *
 * Invariant: same formulas as the Python class.  edf_step_fixed() uses the
 * lag factor precomputed for the control period (no exp in the 250 Hz path);
 * the only per-tick transcendental is the sqrtf of V_e.
 */
#ifndef FC_THRUST_MODEL_H
#define FC_THRUST_MODEL_H

#include "fcmath.h"

typedef struct {
    float t_max_n, throttle_exp, tau_s, p_nom_w, omega_max_rad_s, swirl_fraction, swirl_sign, a_e_m2, rho;
    int   throttle_exp_int;
    float lag_alpha;           /* 1 - exp(-dt/tau) for the control period */
    float inv_rho_ae, inv_2ae;
} edf_model_t;

void  edf_init(edf_model_t *m, float t_max_n, float throttle_exp, int throttle_exp_int, float tau_s, float p_nom_w,
               float omega_max_rad_s, float swirl_fraction, float swirl_sign, float a_e_m2, float rho, float dt);
float edf_thrust_steady(const edf_model_t *m, float u);
float edf_thrust_rate(const edf_model_t *m, float thrust_n, float u);
float edf_step(const edf_model_t *m, float thrust_n, float u, float dt);
float edf_step_fixed(const edf_model_t *m, float thrust_n, float u);
float edf_power(const edf_model_t *m, float u);
float edf_throttle_for_thrust(const edf_model_t *m, float thrust_n);
float edf_exhaust_velocity(const edf_model_t *m, float thrust_n);
float edf_dynamic_pressure(const edf_model_t *m, float thrust_n);
float edf_swirl_torque(const edf_model_t *m, float thrust_n);

#endif /* FC_THRUST_MODEL_H */
