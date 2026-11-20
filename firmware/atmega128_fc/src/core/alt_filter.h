/* alt_filter.h - AltitudeFilter of sim/state_machine.py (baro + accelerometer
 * complementary filter for h and v_z that feeds the cut-off rule).
 *
 *   predict(a_z, dt):  h += v dt + 1/2 a_z dt^2;  v += a_z dt
 *   correct(h_b, t):   first sample initialises h = h_b, v = 0;
 *                      dt_b = t - t_last (0 on the first call -> no correction);
 *                      r = (h_b + v latency) - h;  h += k1 r dt_b;  v += k2 r dt_b
 *   k1 = 2 zeta w_n, k2 = w_n^2, w_n = 2 pi f_c
 *
 * Invariant: same state and order as the Python class; baro time stamps are
 * control ticks (dt_b = ticks * tick_dt).
 */
#ifndef FC_ALT_FILTER_H
#define FC_ALT_FILTER_H

#include "fcmath.h"

typedef struct {
    float k1, k2, latency, tick_dt;
    float h, v;
    bool has_last_baro;
    uint32_t t_last_baro;
    bool initialised;
} alt_filter_t;

void alt_filter_init(alt_filter_t *f, float crossover_hz, float zeta, float baro_latency_s, float tick_dt);
void alt_filter_reset(alt_filter_t *f, float h0);
void alt_filter_predict(alt_filter_t *f, float a_z_world, float dt);
void alt_filter_correct(alt_filter_t *f, float h_baro, uint32_t t_ticks);

#endif /* FC_ALT_FILTER_H */
