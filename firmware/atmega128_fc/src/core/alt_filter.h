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

/* ---- v0.7.1 VTOL NAV altitude filter: sim/state_machine.py VtolAltitudeFilter (the VTOL build only; the ballistic NAV
 * keeps alt_filter_* above bit for bit).  States h [m], v [m/s] and the accelerometer z-bias b [m/s^2].  Why the bias
 * state: v0.7.0 landed on the integrated accelerometer after a baro failure and the ASSUMED 10 mg residual bias flew 7
 * of 20 seeds away; learnt while the baro is healthy, the bias is removed before the integration.
 *   gains: k1 = 2 zeta w + p, k2 = w^2 + 2 zeta w p, k3 = w^2 p (w = 2 pi f_c, p = 1 / bias_tau_s; the generator
 *          computes them in double and rounds once: ALTV_K1..3)
 *   per NAV tick (alt_vtol_step), in this order:
 *     pad (sim.nlink_proto.nav_alt_pad_mode: the last valid FCU frame says SAFE / ARMED): h = v = 0;
 *          n = min(n + 1, N), b += (a - b) / n, b = clamp(b, +-b_lim) - a running mean of the first N = round(pad tau /
 *          dt) ticks, then an EMA with 1 / N (n never reset)
 *     flight: h += v dt + 1/2 (a - b) dt^2;  v += (a - b) dt
 *     then a new baro sample h_b at tick t: pad -> its time only (stamp);  flight -> h_c = h_b - comp_m T(u_prev) / m g
 *          (alt_vtol_thrust_ratio; u_prev = the FCU's thr_q: nl_fcu_throttle), dt_b = (t - t_last) dt (0 on the first
 *          sample -> stamp only), r = (h_c + v L) - h, h += k1 r dt_b, v += k2 r dt_b, b = clamp(b - k3 r dt_b, +-b_lim)
 *     baro lost: no samples -> predict only, with the learnt b (no mode).
 * Units: h, h_b [m] above the pad; v [m/s] up; a = the world-frame vertical specific force minus g [m/s^2]; t in ticks. */
typedef struct {
    float    k1, k2, k3;               /* [1/s], [1/s^2], [1/s^3] */
    float    latency_s;                /* baro latency L */
    float    tick_dt;                  /* [s] */
    uint16_t pad_n_avg;                /* N = round(pad_bias_tau_s / dt) */
    float    bias_limit_mps2;          /* b_lim */
    float    comp_m;                   /* mission.vtol.baro_thrust_comp_m (0 = uncalibrated: no compensation) */
    float    inv_u_hover;              /* 1 / u_hover of the vertical controller */
    int8_t   throttle_exp_int;         /* 2: T / m g = (u inv_u_hover)^2; other: powf with throttle_exp */
    float    throttle_exp;
} alt_vtol_params_t;

typedef struct {
    alt_vtol_params_t p;
    float    h, v, b;
    uint16_t n_pad;
    bool     has_last_baro;
    uint32_t t_last_baro;
    float    last_comp_m;              /* the compensation applied to the latest sample (diagnostics) */
} alt_vtol_t;

void  alt_vtol_init(alt_vtol_t *f, const alt_vtol_params_t *p);
/* T(u) / (m g) of the thrust model: (min(u, 1) inv_u_hover)^n; u <= 0 or NaN -> 0 */
float alt_vtol_thrust_ratio(const alt_vtol_params_t *p, float u);
void  alt_vtol_step(alt_vtol_t *f, float a_z_world, bool has_baro, float h_baro, uint32_t t_ticks, bool pad, float u_prev);

#endif /* FC_ALT_FILTER_H */
