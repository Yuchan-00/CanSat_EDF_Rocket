/* att_estimator.h - the flight computer's attitude estimator, a transcription of
 * sim/estimator.py PadAlignGyroEstimator (the simulation's "pad_gyro" mode).
 *
 * Called once per control tick with the bias-uncorrected gyro g [rad/s] and the
 * specific force a [m/s^2], both in the body frame:
 *
 *   on the pad (mission state IDLE or ARMED, the vehicle is static):
 *       n      = min(n + 1, N_avg)                 N_avg = round(pad_avg_tau_s / dt)
 *       b_hat += (g - b_hat) / n                   running mean, then EMA alpha = 1/N_avg
 *       f_hat += (a - f_hat) / n
 *       q_hat  = level_from_accel(f_hat):  roll = atan2(f_y, f_z),
 *                pitch = atan2(-f_x, hypot(f_y, f_z)),  q = q_zyx(roll, pitch, yaw = 0)
 *   after launch (every other state): gyro only, bias frozen, accelerometer unused
 *       q_hat  = integrate(q_hat, g - b_hat, dt)
 *   rate for the rate PID: g - b_hat
 *
 * Invariant: the caller decides on_pad from the mission state of the previous
 * tick (as the simulation does); nothing here reads the accelerometer once
 * launched - in powered flight it measures thrust, not gravity.  The trig of
 * the levelling runs only on the pad, where the controller is idle.
 */
#ifndef FC_ATT_ESTIMATOR_H
#define FC_ATT_ESTIMATOR_H

#include "fcmath.h"

typedef struct {
    float dt;
    uint16_t n_avg, n;
    float bias[3];
    float f_mean[3];
    float q[4];
    bool aligned;
} att_est_t;

void att_est_init(att_est_t *e, float dt, uint16_t n_avg);
void att_est_level_from_accel(const float f_b[3], float q[4]);
void att_est_update(att_est_t *e, const float gyro[3], const float accel[3], bool on_pad);
void att_est_rate(const att_est_t *e, const float gyro[3], float rate[3]);

/* ---- v0.7.1 VTOL NAV estimator: sim/estimator.py GatedMahonyEstimator (params.json mission.vtol.nav_estimator; the
 * VTOL build only - the ballistic NAV keeps att_est_* above bit for bit).  Why: the gyro-only estimate drifted 7.6 deg
 * over a 40 s hover in the 6-DOF with the ASSUMED MPU-6050 bias wander, and nothing on the vehicle measures horizontal
 * motion.  Once per control tick, same call as att_est_update:
 *   pad (on_pad = sim.nlink_proto.nav_pad_mode: the last valid FCU frame says SAFE / ARMED, no FF_BENCH): exactly the
 *     PadAlignGyroEstimator pad branch (att_est_update(&m->pad, .., true): n, b_hat, f_hat, levelling), then
 *     a_lpf = f_hat, b_pad = b_hat, e = 0, gate closed.  Re-entry from flight (LANDED -> SAFE before another hop):
 *     f_hat := a_lpf first (level from the landing spot); b_hat continues; n stays at N_avg.
 *   flight: a_lpf += alpha_a (a - a_lpf), alpha_a = 1 - exp(-2 pi f_a dt);
 *     gate = |a_lpf| > 0 and | |a_lpf| - g | < accel_gate g;
 *     v = R(q)^T e_z = (2(x z - w y), 2(y z + w x), 1 - 2(x^2 + y^2)) (estimated up, body);
 *     e = (a_lpf / |a_lpf|) x v if gate, else 0 [rad];
 *     b_hat = b_pad + clamp(b_hat - b_pad - ki dt e, +-bias_limit) per axis (only while the gate is open);
 *     q = integrate(q, g - b_hat + kp e, dt)
 *   rate for the rate PID: g - b_hat (att_mahony_rate; the correction kp e is not a body rate).
 * Units: gyro [rad/s], accel [m/s^2] body frame; kp [1/s], ki [1/s^2], bias_limit [rad/s].  Yaw is not observable (no
 * magnetometer): e is horizontal in the world, the heading drifts with the residual z bias.  Invariant: the
 * accelerometer is used only through the low-passed, gated vector - while the vehicle accelerates (|f| != g) the
 * estimate is gyro-only exactly as v0.6.3.
 *
 * v0.9 timing proposal P1 (fw09/p1 prototype, core/qfix.h): in flight the integration runs in fixed point - the state
 * qf (Q30) is integrated by quat_integrate_fx (the reference's series and Newton renormalisation, Q30 / Q32 arithmetic)
 * and pad.q is its float copy (rounded, what the frame, the glink and the crosscheck see).  v = R(q)^T e_z of pad.q is
 * kept with it (the pad branch: float, as before; flight: from qf) - the next correction's v and the altitude filter's
 * world-z row (att_mahony_up), computed once instead of twice.  qf_ok = qf holds pad.q (false on the pad and after init;
 * rebuilt from pad.q at the first flight call). */
typedef struct {
    att_est_t pad;               /* the pad branch's state and the shared q, b_hat (pad.bias), n, aligned, dt */
    float kp;                    /* [1/s] */
    float ki_dt;                 /* ki x dt [1/s] */
    float gate_mps2;             /* accel_gate x g */
    float g;
    float gate_lo2, gate_hi2;    /* (g - accel_gate g)^2, (g + accel_gate g)^2: the gate on |a_lpf|^2 (no sqrt when closed) */
    float alpha_a;               /* 1 - exp(-2 pi accel_lpf_hz dt) */
    float bias_limit;            /* [rad/s], per axis, relative to b_pad */
    float b_pad[3], a_lpf[3], e[3];
    bool gate, was_pad;
    uint32_t flight_ticks, gated_ticks;   /* diagnostics (saturating) */
    int32_t qf[4];               /* P1: the flight state q in Q30 (pad.q = its float copy) */
    float v[3];                  /* P1: R(pad.q)^T e_z (estimated up, body) */
    bool qf_ok;                  /* P1: qf holds pad.q */
} att_mahony_t;

/* P1: R(q)^T e_z of the estimator's current q (the world-z row of R(q): a_z_w = v . f) */
static inline const float *att_mahony_up(const att_mahony_t *m) { return m->v; }

void att_mahony_init(att_mahony_t *m, float dt, uint16_t n_avg, float kp, float ki_dt, float gate_mps2, float g,
                     float alpha_a, float bias_limit);
void att_mahony_update(att_mahony_t *m, const float gyro[3], const float accel[3], bool on_pad);
void att_mahony_rate(const att_mahony_t *m, const float gyro[3], float rate[3]);

/* v0.8 NAV order (sim/estimator.py module docstring, params flight_computer.nav.schedule_note; the VTOL build):
 *   1. IMU read; 2. att_mahony_predict -> q_pub, rate (no state change); nl_nav3_tilt(q_pub); 3. PUBLISH the nlink v3
 *   frame (q_pub, rate, accel, the altitude filter's h / v_z of the previous tick, cos_tilt / tilt, flags, GPS slot);
 *   4. att_mahony_update (the v0.7.1 correction, unchanged, from the previous tick's q and b); 5. the VTOL altitude
 *   filter with the CORRECTED q; 6. GPS (core/gps_ubx.h, core/gps_nav.h); 7. background.
 * Why (the v0.7.1 integration measured IMU sample -> vane 5.2 ms > 4.71 ms and NAV tick -> vane 4.7 ms > 4.0 ms): only
 * the cheap gyro prediction stands between the IMU read and the frame.  The correction reaches the FCU one tick (4 ms)
 * later - immaterial for a correction whose time constant is 1 / kp (>= 6 s).
 * predict: rate = g - b (the bias of the previous correction); flight: q_pub = integrate(q, rate, dt); pad: q_pub = q
 * (the levelled pad attitude).  The ballistic NAV keeps att_est_* in the v0.6.3 order bit for bit. */
void att_mahony_predict(att_mahony_t *m, const float gyro[3], bool on_pad, float q_pub[4], float rate[3]);

#endif /* FC_ATT_ESTIMATOR_H */
