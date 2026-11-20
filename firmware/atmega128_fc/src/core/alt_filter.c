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
