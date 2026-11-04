/* cosim4.h - what the four-core co-simulation harness (cosim4.c) shares with its closed-loop plant (plant4.c) and the
 * NAV's TWI sensor models (twi_models.c).  Host C99, linked with PlatformIO's libsimavr.a (tools/run_cosim4.py builds it).
 *
 * Invariant: the plant and the sensor models only produce the INPUTS a real vehicle would give the firmware (the MPU-6050
 * and MS5611 register contents, the GPS antenna's position for the GNS's synthetic receiver); they never read or write a
 * firmware variable other than the harness-owned RAM block g_cosim_gps of the gns_cosim build (gns_simbench.h).
 */
#ifndef COSIM4_H
#define COSIM4_H

#include <stdint.h>

#include "sim_avr.h"

/* ------------------------------------------------------------------ plant (plant4.c) */
/* The vehicle the firmware image's own params_gen.h describes (run_cosim4.py reads the constants from the tree under
 * test and passes them as pl_* arguments), flown by the FCU's compare registers:
 *   thrust  T = T_max u_lag^n, u_lag the EDF's first-order lag (tau) of the ESC command u = (pulse - min) / (max - min);
 *           mass m = T_max u_hover^n / g (so the vehicle hovers at the image's u_hover, as the single-chip bench's plant);
 *   vanes   delta_i = (pulse_i - c0_i) / k_i (the image's servo centre / trim / sign / us per rad); moments from the image's
 *           own forward mixer model (mixer.c mixer_moments): k = T S_v CL_a / (2 A_e), M = (2 l_v k dx, 2 l_v k dy,
 *           4 r_v k dz); rigid body I w' = M - w x I w (diagonal I, the image's VEH_I*);
 *   motion  world frame = the NAV's level frame at boot (x = body x on the pad, z up); a horizontal wind with linear drag
 *           while airborne, a = wind - drag_k v (ASSUMED), so the position hold flies a tilt and an attitude-only build
 *           drifts at a bounded speed;
 *   ground  h = 0 carries the weight until T cos(tilt) > m g; on the ground the attitude is the pad's (roll / pitch below)
 *           and nothing moves; a touchdown (h <= 0 descending) records its speed and grounds the vehicle again.
 * Not modelled (stated in the report): drag, ground effect, servo lag / rate limit, the 50 Hz PWM frame hold (the plant
 * reads the compare registers at every step), the rotor's reaction torque, sensor dynamics (MPU DLPF), tether forces. */
typedef struct {
    double u_hover, n_exp, t_max, l_v, r_v, s_v, cl_a, a_e, ixx, iyy, izz, tau, g0;
    double esc_min_us, esc_max_us, c0_us[4], k_us_per_rad[4];
    double pad_roll, pad_pitch;                  /* rad: the pad attitude */
    double wind_ax, wind_ay;                     /* m/s^2 in the world frame while airborne at zero ground speed (ASSUMED) */
    double drag_k;                               /* 1/s: horizontal linear drag, a = wind - drag_k v (ASSUMED; the wind's
                                                  * air speed is wind / drag_k: an attitude-only build drifts towards it
                                                  * instead of accelerating for ever) */
    double lever_b[3];                           /* GPS antenna from the CG, body (GPS_LEVER_B_*) */
    double psi0;                                 /* GPS_PSI0_RAD: heading of the NAV's level frame in W (x north, y west) */
    double gyro_noise, accel_noise, baro_noise_pa;   /* 1-sigma-like uniform noise amplitudes (ASSUMED) */
    double p_ground_pa;                          /* pressure at the pad */
    double baro_comp_m;                          /* the baro's thrust-induced error at hover thrust, m (the NAV image's
                                                  * altitude filter subtracts comp_m T / m g: nav_simbench.h NAV_SB_COMP_M
                                                  * in a simbench build) - added to the baro altitude in the air */
    uint64_t seed;
} plant_par_t;

typedef struct {
    double t;                                    /* s, global */
    double h, vz, x, y, vx, vy;                  /* m, m/s, level frame (z up) */
    double q[4];                                 /* body -> level frame (w, x, y, z) */
    double w[3];                                 /* body rates rad/s */
    double u_cmd, u_lag, delta[4], thrust_n;
    double f_b[3];                               /* body specific force m/s^2 (what the accelerometer reads) */
    int airborne, motor_seen;
    double h_max, td_v, tilt_max_deg;            /* the run's extremes */
    unsigned touchdowns, liftoffs;
    uint64_t rng;
} plant_t;

void plant_init(plant_t *p, const plant_par_t *par);
/* one step of dt seconds with the compare registers pulse_us[0..3] = vanes 1..4, pulse_us[4] = ESC (0: not written yet) */
void plant_step(plant_t *p, const plant_par_t *par, const double pulse_us[5], double dt);
/* the sensor truth the models read: body rates (rad/s), body specific force (m/s^2), static pressure (Pa) */
void plant_sensors(plant_t *p, const plant_par_t *par, double gyro[3], double accel[3], double *p_pa);
/* the GPS antenna in W (x north, y west): north / east position (m) and velocity (m/s) */
void plant_gps(const plant_t *p, const plant_par_t *par, double *north, double *east, double *vn, double *ve);
double plant_tilt_deg(const plant_t *p);
void plant_euler(const plant_t *p, double *roll, double *pitch, double *yaw);

/* ------------------------------------------------------------------ TWI sensor models (twi_models.c) */
/* An MPU-6050 at 0x68 and an MS5611 at 0x77 on the NAV's TWI (timing82/profile/harness/navflight.c's register models),
 * their values from the plant (twi_set_truth); and simavr 1.8's TWI byte timing corrected to the bus clock the NAV
 * programs (TWBR / TWPS): simavr schedules each TWI state change N microseconds ahead for an N-bit step (9 us per byte
 * at any clock); with twi_fix the harness re-schedules it to N SCL periods (22.5 us per byte at 400 kHz). */
typedef struct {
    uint64_t msgs, reads, burst_reads, bursts, ms_cmds, d1_convs, d2_convs, adc_reads, adc_early, prom_reads;
    uint64_t rescaled, rescale_hist[16], not_found;
    uint64_t scl_cycles_per_bit;
} twi_stats_t;

void twi_models_init(avr_t *nav, int fix_byte_time, uint64_t seed, double noise_gyro, double noise_accel,
                     double noise_pa);
void twi_set_truth(const double gyro[3], const double accel[3], double p_pa);
void twi_models_reset(void);                     /* the NAV was reset: the sensors keep power (state kept), the bus idles */
const twi_stats_t *twi_models_stats(void);
extern int g_twi_debug_n;                        /* twidbg=N: the first N TWI delays simavr scheduled, on stderr */

#endif /* COSIM4_H */
