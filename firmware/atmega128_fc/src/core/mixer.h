/* mixer.h - VaneMixer of sim/controller.py (SPEC 8.4/8.5), single precision.
 *
 *   T_hat = max(T_max u^n, floor T_max)             (thrust_estimate)
 *   k     = T_hat S_v C_La / (2 A_e)                 (force_slope, N/rad per vane)
 *   d_x = M_x/(2 l_v k), d_y = M_y/(2 l_v k), d_z = roll_scale M_z/(4 r_v k)
 *   d1 = d_x + d_z, d2 = d_y + d_z, d3 = -d_x + d_z, d4 = -d_y + d_z
 *   saturation: if max(|d_x|,|d_y|) + |d_z| > d_max: first shrink |d_z| to
 *   d_max - max(|d_x|,|d_y|) (>= 0); if that is not enough drop d_z and scale
 *   (d_x, d_y) so the largest |d_i| = d_max.
 *   pwm_i = pwm_center + sign_i deg(d_i) pwm_us_per_deg + trim_i
 *
 * Invariant: the same equations as the Python class; the three divisions of
 * deflections() are replaced by one reciprocal of k and constant
 * reciprocals precomputed in mixer_init (no division by a constant in the
 * 250 Hz path).
 */
#ifndef FC_MIXER_H
#define FC_MIXER_H

#include "fcmath.h"

typedef struct {               /* MixerPlant (SI, rad) */
    float l_v_m, r_v_m, s_v_m2, cl_alpha, a_e_m2, t_max_n, throttle_exp, delta_max_rad;
    float pwm_center_us, pwm_us_per_deg;
    float servo_signs[4];
    float servo_trims_us[4];
} mixer_plant_t;

typedef mixer_plant_t mixer_plant_rt_t;

typedef struct {
    mixer_plant_rt_t p;
    float floor_fraction;
    float roll_priority_scale, delta_max;
    int   throttle_exp_int;    /* >0: u^n by repeated multiplication; 0: powf */
    /* precomputed in mixer_init */
    float k_per_n;             /* S_v C_La / (2 A_e)        : k = T_hat k_per_n */
    float inv_2lv, inv_4rv;    /* 1/(2 l_v), 1/(4 r_v)      */
    float floor_thrust_n;      /* floor_fraction T_max      */
    float us_per_rad;          /* pwm_us_per_deg * 180/pi   */
    float pwm_c0[4], pwm_k[4]; /* pwm_i = c0_i + k_i d_i: c0 = centre + trim, k = sign us_per_rad */
    float delta_max_eps;       /* delta_max + 1e-12 (the Python test threshold)                 */
} vane_mixer_t;

void  mixer_init(vane_mixer_t *m, const mixer_plant_t *plant, float floor_fraction, float roll_priority_scale,
                 int throttle_exp_int);
float mixer_thrust_model(const vane_mixer_t *m, float u);
float mixer_thrust_estimate(const vane_mixer_t *m, float u);
bool  mixer_thrust_floored(const vane_mixer_t *m, float u);
float mixer_force_slope(const vane_mixer_t *m, float t_hat);
void  mixer_moment_gains(const vane_mixer_t *m, float t_hat, float g[3]);
void  mixer_combine(float dx, float dy, float dz, float delta[4]);
void  mixer_split(const float delta[4], float *dx, float *dy, float *dz);
/* returns `saturated` */
bool  mixer_deflections(const vane_mixer_t *m, const float moments[3], float t_hat, float delta[4]);
void  mixer_moments(const vane_mixer_t *m, const float delta[4], float t_hat, float moments[3]);
void  mixer_pwm(const vane_mixer_t *m, const float delta[4], float pwm_us[4]);

#endif /* FC_MIXER_H */
