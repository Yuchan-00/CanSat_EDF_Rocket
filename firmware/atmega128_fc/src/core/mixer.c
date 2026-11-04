/* mixer.c - see mixer.h (mirrors sim/controller.py VaneMixer). */
#include "mixer.h"

void mixer_init(vane_mixer_t *m, const mixer_plant_t *plant, float floor_fraction, float roll_priority_scale,
                int throttle_exp_int)
{
#if FC_VTOL_RAM_TRIM
    m->p.l_v_m = plant->l_v_m;                        /* only what the 250 Hz path reads (the rest is folded below) */
    m->p.r_v_m = plant->r_v_m;
    m->p.t_max_n = plant->t_max_n;
    m->p.throttle_exp = plant->throttle_exp;
#else
    m->p = *plant;
#endif
#if !FC_VTOL_RAM_TRIM
    m->floor_fraction = floor_fraction;
#endif
    m->roll_priority_scale = roll_priority_scale;
    m->delta_max = plant->delta_max_rad;
    m->throttle_exp_int = throttle_exp_int;
    m->k_per_n = plant->s_v_m2 * plant->cl_alpha / (2.0f * plant->a_e_m2);
    m->inv_2lv = 1.0f / (2.0f * plant->l_v_m);
    m->inv_4rv = 1.0f / (4.0f * plant->r_v_m);
    m->floor_thrust_n = floor_fraction * plant->t_max_n;
#if FC_VTOL_RAM_TRIM
    {
        const float us_per_rad = plant->pwm_us_per_deg * FC_RAD2DEG_F;
        int i;
        m->delta_max_eps = m->delta_max + 1e-12f;
        for (i = 0; i < 4; i++) {
            m->pwm_c0[i] = plant->pwm_center_us + plant->servo_trims_us[i];
            m->pwm_k[i] = plant->servo_signs[i] * us_per_rad;
        }
    }
#else
    m->us_per_rad = plant->pwm_us_per_deg * FC_RAD2DEG_F;
    m->delta_max_eps = m->delta_max + 1e-12f;
    {
        int i;
        for (i = 0; i < 4; i++) {
            m->pwm_c0[i] = plant->pwm_center_us + plant->servo_trims_us[i];
            m->pwm_k[i] = plant->servo_signs[i] * m->us_per_rad;
        }
    }
#endif
}

/* thrust_model(): T_max clip(u,0,1)^n */
float mixer_thrust_model(const vane_mixer_t *m, float u)
{
    float uc = fc_minf(fc_maxf(u, 0.0f), 1.0f);
    float un;
    if (m->throttle_exp_int > 0) {
        int i;
        un = uc;
        for (i = 1; i < m->throttle_exp_int; i++)
            un *= uc;
    } else {
        un = powf(uc, m->p.throttle_exp);
    }
    return m->p.t_max_n * un;
}

/* thrust_estimate(): max(T_model(u), floor T_max) */
float mixer_thrust_estimate(const vane_mixer_t *m, float u)
{
    return fc_maxf(mixer_thrust_model(m, u), m->floor_thrust_n);
}

bool mixer_thrust_floored(const vane_mixer_t *m, float u)
{
    return mixer_thrust_model(m, u) < m->floor_thrust_n;
}

/* force_slope(): k = T_hat / (2 A_e) S_v C_La */
float mixer_force_slope(const vane_mixer_t *m, float t_hat)
{
    return t_hat * m->k_per_n;
}

/* moment_gains(): (2 l_v k, 2 l_v k, 4 r_v k) */
void mixer_moment_gains(const vane_mixer_t *m, float t_hat, float g[3])
{
    float k = mixer_force_slope(m, t_hat);
    g[0] = 2.0f * m->p.l_v_m * k;
    g[1] = 2.0f * m->p.l_v_m * k;
    g[2] = 4.0f * m->p.r_v_m * k;
}

void mixer_combine(float dx, float dy, float dz, float delta[4])
{
    delta[0] = dx + dz;
    delta[1] = dy + dz;
    delta[2] = -dx + dz;
    delta[3] = -dy + dz;
}

void mixer_split(const float delta[4], float *dx, float *dy, float *dz)
{
    *dx = 0.5f * (delta[0] - delta[2]);
    *dy = 0.5f * (delta[1] - delta[3]);
    *dz = 0.25f * (delta[0] + delta[1] + delta[2] + delta[3]);
}

/* deflections(): inverse mixer with saturation priority */
bool mixer_deflections(const vane_mixer_t *m, const float moments[3], float t_hat, float delta[4])
{
    float inv_k = 1.0f / mixer_force_slope(m, t_hat);
    float dx = moments[0] * inv_k * m->inv_2lv;
    float dy = moments[1] * inv_k * m->inv_2lv;
    float dz = m->roll_priority_scale * moments[2] * inv_k * m->inv_4rv;
    float m_xy = fc_maxf(fabsf(dx), fabsf(dy));
    bool saturated = false;
    if (m_xy + fabsf(dz) > m->delta_max_eps) {
        saturated = true;
        if (m_xy < m->delta_max) {
            dz = copysignf(m->delta_max - m_xy, dz);      /* 1) shrink roll */
        } else {
            float s = m->delta_max / m_xy;                /* 2) drop roll, scale pitch/yaw */
            dz = 0.0f;
            dx *= s;
            dy *= s;
        }
    }
    mixer_combine(dx, dy, dz, delta);
    return saturated;
}

/* moments(): forward linear model (physical roll moment 4 r_v k d_z) */
void mixer_moments(const vane_mixer_t *m, const float delta[4], float t_hat, float moments[3])
{
    float g[3], dx, dy, dz;
    mixer_moment_gains(m, t_hat, g);
    mixer_split(delta, &dx, &dy, &dz);
    moments[0] = g[0] * dx;
    moments[1] = g[1] * dy;
    moments[2] = g[2] * dz;
}

/* pwm(): centre + sign deg(d) us_per_deg + trim, with the constant parts folded in mixer_init */
void mixer_pwm(const vane_mixer_t *m, const float delta[4], float pwm_us[4])
{
    int i;
    for (i = 0; i < 4; i++)
        pwm_us[i] = m->pwm_c0[i] + m->pwm_k[i] * delta[i];
}

#if FC_V09
static uint32_t mix_f32u(float x)
{
    union {
        float f;
        uint32_t u;
    } v;
    v.f = x;
    return v.u;
}

uint8_t mixer_deflections_v9(const vane_mixer_t *m, const float moments[3], float t_hat, bool yaw_trim_on, float i_z,
                             float delta[4])
{
    float inv_k = 1.0f / mixer_force_slope(m, t_hat);
    float dx = moments[0] * inv_k * m->inv_2lv;
    float dy = moments[1] * inv_k * m->inv_2lv;
    float dz = m->roll_priority_scale * moments[2] * inv_k * m->inv_4rv;
    float m_xy = fc_maxf(fabsf(dx), fabsf(dy));
    uint8_t fl = 0;
    if (m_xy + fabsf(dz) > m->delta_max_eps) {
        float r = 0.0f;                                   /* the yaw share kept (v0.8.1: 0) */
        fl = MIX_SAT;
        if (yaw_trim_on) {                                /* fix round: the yaw integrator's trim, d_trim d_z > 0 */
            const float d_trim = m->roll_priority_scale * i_z * inv_k * m->inv_4rv;
            const uint32_t a = mix_f32u(d_trim), b = mix_f32u(dz);
            if ((a & 0x7FFFFFFFUL) != 0u && (b & 0x7FFFFFFFUL) != 0u && ((a ^ b) & 0x80000000UL) == 0u)
                r = fc_minf(fc_minf(fabsf(dz), fabsf(d_trim)), m->delta_max);
        }
        if (m_xy < m->delta_max - r) {
            dz = copysignf(m->delta_max - m_xy, dz);      /* 1) shrink roll */
        } else {
            float s = (m->delta_max - r) / m_xy;          /* 2) roll to its reserve, scale pitch / yaw into the rest */
            dz = copysignf(r, dz);
            dx *= s;
            dy *= s;
            fl |= MIX_LIM_XY;
        }
    }
    mixer_combine(dx, dy, dz, delta);
    return fl;
}
#endif
