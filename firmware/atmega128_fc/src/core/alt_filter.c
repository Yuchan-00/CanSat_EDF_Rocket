/* alt_filter.c - see alt_filter.h (mirrors sim/state_machine.py AltitudeFilter). */
#include "alt_filter.h"

void alt_filter_init(alt_filter_t *f, float crossover_hz, float zeta, float baro_latency_s, float tick_dt)
{
    float w_n = 2.0f * FC_PI_F * crossover_hz;
    f->k1 = 2.0f * zeta * w_n;
    f->k2 = w_n * w_n;
    f->latency = baro_latency_s;
    f->tick_dt = tick_dt;
    f->h = 0.0f;
    f->v = 0.0f;
    f->has_last_baro = false;
    f->t_last_baro = 0;
    f->initialised = false;
}

void alt_filter_reset(alt_filter_t *f, float h0)
{
    f->h = h0;
    f->v = 0.0f;
    f->has_last_baro = false;
    f->initialised = true;
}

void alt_filter_predict(alt_filter_t *f, float a_z_world, float dt)
{
    f->h += f->v * dt + 0.5f * a_z_world * dt * dt;
    f->v += a_z_world * dt;
}

void alt_filter_correct(alt_filter_t *f, float h_baro, uint32_t t_ticks)
{
    float dt_b, r;
    if (!f->initialised) {
        f->h = h_baro;
        f->v = 0.0f;
        f->initialised = true;
    }
    dt_b = (!f->has_last_baro || t_ticks < f->t_last_baro) ? 0.0f : (float)(t_ticks - f->t_last_baro) * f->tick_dt;
    f->t_last_baro = t_ticks;
    f->has_last_baro = true;
    if (dt_b <= 0.0f)
        return;
    r = (h_baro + f->v * f->latency) - f->h;
    f->h += f->k1 * r * dt_b;
    f->v += f->k2 * r * dt_b;
}

/* ---- v0.7.1 VTOL: VtolAltitudeFilter (alt_filter.h) ---- */
void alt_vtol_init(alt_vtol_t *f, const alt_vtol_params_t *p)
{
    f->p = *p;
    if (f->p.pad_n_avg == 0u)
        f->p.pad_n_avg = 1u;
    f->h = f->v = f->b = 0.0f;
    f->n_pad = 0;
    f->has_last_baro = false;
    f->t_last_baro = 0;
    f->last_comp_m = 0.0f;
}

float alt_vtol_thrust_ratio(const alt_vtol_params_t *p, float u)
{
    float x;
    if (!(u == u) || u <= 0.0f)                       /* NaN or <= 0: motor off */
        return 0.0f;
    x = (u < 1.0f ? u : 1.0f) * p->inv_u_hover;
    return p->throttle_exp_int == 2 ? x * x : powf(x, p->throttle_exp);
}

void alt_vtol_step(alt_vtol_t *f, float a_z_world, bool has_baro, float h_baro, uint32_t t_ticks, bool pad, float u_prev)
{
    const alt_vtol_params_t *p = &f->p;
    if (pad) {                                        /* pad hold + accelerometer-bias calibration (the vehicle is still) */
        f->h = 0.0f;
        f->v = 0.0f;
        if (f->n_pad < p->pad_n_avg)
            f->n_pad++;
        f->b += (a_z_world - f->b) / (float)f->n_pad;
        f->b = fc_clampf(f->b, -p->bias_limit_mps2, p->bias_limit_mps2);
    } else {                                          /* predict with the learnt bias removed */
        const float a = a_z_world - f->b;
        f->h += f->v * p->tick_dt + 0.5f * a * p->tick_dt * p->tick_dt;
        f->v += a * p->tick_dt;
    }
    if (!has_baro)
        return;
    if (pad) {                                        /* stamp only: h, v known, b calibrated directly */
        f->t_last_baro = t_ticks;
        f->has_last_baro = true;
        return;
    }
    {
        float dt_b, r;
        dt_b = (!f->has_last_baro || t_ticks < f->t_last_baro) ? 0.0f : (float)(t_ticks - f->t_last_baro) * p->tick_dt;
        f->t_last_baro = t_ticks;
        f->has_last_baro = true;
        f->last_comp_m = p->comp_m != 0.0f ? p->comp_m * alt_vtol_thrust_ratio(p, u_prev) : 0.0f;
        if (dt_b <= 0.0f)
            return;
        r = (h_baro - f->last_comp_m + f->v * p->latency_s) - f->h;
        f->h += p->k1 * r * dt_b;
        f->v += p->k2 * r * dt_b;
        f->b = fc_clampf(f->b - p->k3 * r * dt_b, -p->bias_limit_mps2, p->bias_limit_mps2);
    }
}
