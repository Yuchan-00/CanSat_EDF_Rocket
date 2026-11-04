/* thrust_model.c - see thrust_model.h (mirrors sim/edf.py EDFModel). */
#include "thrust_model.h"

void edf_init(edf_model_t *m, float t_max_n, float throttle_exp, int throttle_exp_int, float tau_s, float p_nom_w,
              float omega_max_rad_s, float swirl_fraction, float swirl_sign, float a_e_m2, float rho, float dt)
{
    m->t_max_n = t_max_n;
    m->throttle_exp = throttle_exp;
    m->throttle_exp_int = throttle_exp_int;
    m->tau_s = tau_s;
    m->p_nom_w = p_nom_w;
    m->omega_max_rad_s = omega_max_rad_s;
    m->swirl_fraction = swirl_fraction;
    m->swirl_sign = swirl_sign;
    m->a_e_m2 = a_e_m2;
    m->rho = rho;
    m->lag_alpha = 1.0f - expf(-dt / tau_s);
    m->inv_rho_ae = 1.0f / (rho * a_e_m2);
    m->inv_2ae = 1.0f / (2.0f * a_e_m2);
}

static float upow(const edf_model_t *m, float uc)
{
    if (m->throttle_exp_int > 0) {
        float r = uc;
        int i;
        for (i = 1; i < m->throttle_exp_int; i++)
            r *= uc;
        return r;
    }
    return powf(uc, m->throttle_exp);
}

float edf_thrust_steady(const edf_model_t *m, float u)
{
    return m->t_max_n * upow(m, fc_clampf(u, 0.0f, 1.0f));
}

float edf_thrust_rate(const edf_model_t *m, float thrust_n, float u)
{
    return (edf_thrust_steady(m, u) - thrust_n) / m->tau_s;
}

float edf_step(const edf_model_t *m, float thrust_n, float u, float dt)
{
    return thrust_n + (edf_thrust_steady(m, u) - thrust_n) * (1.0f - expf(-dt / m->tau_s));
}

float edf_step_fixed(const edf_model_t *m, float thrust_n, float u)
{
    return thrust_n + (edf_thrust_steady(m, u) - thrust_n) * m->lag_alpha;
}

float edf_power(const edf_model_t *m, float u)
{
    float uc = fc_clampf(u, 0.0f, 1.0f);
    return m->p_nom_w * uc * uc * uc;
}

float edf_throttle_for_thrust(const edf_model_t *m, float thrust_n)
{
    return powf(fc_maxf(thrust_n, 0.0f) / m->t_max_n, 1.0f / m->throttle_exp);
}

float edf_exhaust_velocity(const edf_model_t *m, float thrust_n)
{
    return sqrtf(fc_maxf(thrust_n, 0.0f) * m->inv_rho_ae);
}

float edf_dynamic_pressure(const edf_model_t *m, float thrust_n)
{
    return fc_maxf(thrust_n, 0.0f) * m->inv_2ae;
}

float edf_swirl_torque(const edf_model_t *m, float thrust_n)
{
    return m->swirl_sign * m->swirl_fraction * m->p_nom_w / m->omega_max_rad_s * fc_maxf(thrust_n, 0.0f) / m->t_max_n;
}
