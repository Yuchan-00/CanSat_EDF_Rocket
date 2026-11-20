/* test_core.c - host tests of the portable firmware core (src/core) of the rocket and the
 * vector modes used by tools/crosscheck_controller.py.
 *
 *   test_core                                run every unit test, exit 1 on any failure
 *   test_core --vectors in.csv out.csv       attitude controller (sim/controller.py)
 *   test_core --onb-vectors in.txt out.txt   onboard supervisor (sim/onboard.py)
 *   test_core --proto-vectors in.txt out.txt link protocol: decode / encode (TELEM) / quantise (sim/link_proto.py)
 *   test_core --ilink-vectors in.txt out.txt FCU <-> COM ilink: decode / encode / CRC / SNAP -> TELEM (sim/ilink_proto.py)
 *   test_core --nlink-vectors in.txt out.txt NAV <-> FCU nlink: encode / decode of both frames (sim/nlink_proto.py)
 *   test_core --est-vectors in.csv out.csv   attitude estimator (sim/estimator.py PadAlignGyroEstimator)
 *   test_core --alt-vectors in.csv out.csv   altitude filter (sim/state_machine.py AltitudeFilter)
 *   test_core --log-sample out.bin           a few framed log records for tools/decode_log.py (every record type)
 *
 * Built by tools/build_host_test.py with `python -m ziglang cc -std=c99 -O1
 * -ffp-contract=off`, linking src/core only; build/link_vectors_gen.h and
 * build/ilink_vectors_gen.h / build/nlink_vectors_gen.h are generated from
 * sim/out/link_vectors.json, sim/out/ilink_vectors.json and sim/out/nlink_vectors.json
 * by the same script.  Invariant: the unit tests use
 * the generated parameter headers through fc_config.c, i.e. the numbers the
 * firmware flies, and compare against values worked out independently here
 * (double precision, closed form, the SPEC 8.8 / 10.3 tables, or the reference's
 * own byte vectors), never against the code under test.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/core/alt_filter.h"
#include "../src/core/att_estimator.h"
#include "../src/core/attitude_ctrl.h"
#include "../src/core/fc_config.h"
#include "../src/core/ilink_proto.h"
#include "../src/core/link_proto.h"
#include "../src/core/log_record.h"
#include "../src/core/mixer.h"
#include "../src/core/bmp280_math.h"
#include "../src/core/ms5611_math.h"
#include "../src/core/nlink_proto.h"
#include "../src/core/onboard.h"
#include "../src/core/pid.h"
#include "../src/core/quat.h"
#include "../src/core/thrust_model.h"
#include "../src/gains.h"
#include "../src/hal/pwm.h"          /* pwm_us_to_counts only: a portable inline (no AVR header) */
#include "../src/hal/baro.h"         /* fw09 --consts: BARO_LATENCY_S, the NAV's alt filter latency (macros only) */
#include "../src/hal/bmp280.h"       /* 2026-11-10: the BMP280 schedule macros (macros and types only) */
#include "../src/hal/ms5611.h"       /* the MS5611 schedule macros (the build option) */
#include "../src/params_gen.h"
#include "ilink_vectors_gen.h"
#include "link_vectors_gen.h"
#include "nlink_vectors_gen.h"

static int g_checks, g_fail;

#define CHECK(cond, ...)                                                   \
    do {                                                                   \
        g_checks++;                                                        \
        if (!(cond)) {                                                     \
            g_fail++;                                                      \
            printf("FAIL %s:%d: ", __FILE__, __LINE__);                    \
            printf(__VA_ARGS__);                                           \
            printf("\n");                                                  \
        }                                                                  \
    } while (0)
#define NEAR(a, b, tol) (fabs((double)(a) - (double)(b)) <= (tol))
#define CHECK_NEAR(a, b, tol, what) \
    CHECK(NEAR(a, b, tol), "%s: %.9g vs %.9g (tol %g)", what, (double)(a), (double)(b), (double)(tol))

static const char *state_name(int s)
{
    static const char *n[] = {"SAFE", "ARMED", "POWERED", "UNPOWERED", "RELEASED", "ABORT"};
    return (s >= 0 && s <= 5) ? n[s] : "?";
}

/* deterministic LCG for the tests */
static unsigned long g_rng = 12345UL;
static double urand(void)
{
    g_rng = g_rng * 1103515245UL + 12345UL;
    return (double)((g_rng >> 8) & 0xFFFFFFUL) / 16777216.0;
}
static double srand1(void) { return 2.0 * urand() - 1.0; }

/* ------------------------------------------------------------------------ */
/* quaternions                                                              */
/* ------------------------------------------------------------------------ */
static void test_quat(void)
{
    const float c = (float)sqrt(0.5);
    float qx90[4] = {c, c, 0, 0}, qy90[4] = {c, 0, c, 0}, qz90[4] = {c, 0, 0, c}, r[4], v[3], m[3][3];
    float ex[3] = {1, 0, 0};
    int i, k;

    quat_multiply(qx90, qy90, r);
    for (i = 0; i < 4; i++)
        CHECK_NEAR(r[i], 0.5, 1e-7, "qx90*qy90 = (0.5,0.5,0.5,0.5)");
    quat_rotate(qz90, ex, v);                                  /* z 90 deg carries x onto y */
    CHECK_NEAR(v[0], 0.0, 1e-7, "Rz90 x . x");
    CHECK_NEAR(v[1], 1.0, 1e-7, "Rz90 x . y");
    quat_rotate_inverse(qz90, v, v);
    CHECK_NEAR(v[0], 1.0, 3e-7, "inverse rotation");
    quat_to_matrix(qx90, m);                                   /* x 90: y -> z */
    CHECK_NEAR(m[2][1], 1.0, 1e-7, "Rx90[2][1]");
    CHECK_NEAR(m[1][2], -1.0, 1e-7, "Rx90[1][2]");

    {   /* yaw / tilt / from_yaw / euler */
        float q[4], roll, pitch, yaw;
        quat_from_yaw(0.7f, q);
        CHECK_NEAR(quat_yaw(q), 0.7, 1e-6, "yaw(from_yaw(0.7))");
        CHECK_NEAR(quat_tilt(q), 0.0, 1e-3, "tilt of an upright attitude");
        quat_from_euler(0.3f, -0.2f, 1.1f, q);
        quat_to_euler(q, &roll, &pitch, &yaw);
        CHECK_NEAR(roll, 0.3, 1e-6, "euler roll");
        CHECK_NEAR(pitch, -0.2, 1e-6, "euler pitch");
        CHECK_NEAR(yaw, 1.1, 1e-6, "euler yaw");
        /* tilt of roll 0.3 / pitch -0.2: cos(tilt) = cos(roll) cos(pitch) */
        CHECK_NEAR(quat_tilt(q), acos(cos(0.3) * cos(-0.2)), 2e-6, "tilt(roll, pitch)");
        CHECK_NEAR(quat_tilt_cos(q), cos(0.3) * cos(-0.2), 1e-6, "tilt_cos");
    }
    for (k = 0; k < 200; k++) {                                /* rotvec round trip, |v| < pi */
        float rv[3], q[4], back[3];
        double n;
        for (i = 0; i < 3; i++) rv[i] = (float)(1.7 * srand1());
        n = sqrt(rv[0] * rv[0] + rv[1] * rv[1] + rv[2] * rv[2]);
        if (n > 3.0) continue;
        quat_from_rotvec(rv, q);
        quat_to_rotvec(q, back);
        for (i = 0; i < 3; i++)
            CHECK_NEAR(back[i], rv[i], 2e-6, "to_rotvec(from_rotvec(v))");
    }
    for (k = 0; k < 200; k++) {                                /* series exp map vs trig */
        float rv[3], a[4], b[4];
        double s = 0.49 * urand();
        for (i = 0; i < 3; i++) rv[i] = (float)srand1();
        {
            double n = sqrt((double)rv[0] * rv[0] + (double)rv[1] * rv[1] + (double)rv[2] * rv[2]);
            for (i = 0; i < 3; i++) rv[i] = (float)(rv[i] / n * s);
        }
        quat_exp_small(rv, a);
        quat_from_rotvec(rv, b);
        for (i = 0; i < 4; i++)
            CHECK_NEAR(a[i], b[i], 3e-7, "exp_small == from_rotvec");
    }
    {   /* integrate: 1 rad/s about z for 1 s in 250 steps -> yaw 1 rad; about x 0.5 s at 2 rad/s -> tilt 1 rad */
        float q[4] = {1, 0, 0, 0}, w[3] = {0, 0, 1.0f}, w2[3] = {2.0f, 0, 0}, n;
        for (k = 0; k < 250; k++) quat_integrate(q, w, 0.004f, q);
        CHECK_NEAR(quat_yaw(q), 1.0, 2e-5, "integrated yaw");
        n = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
        CHECK_NEAR(n, 1.0, 1e-6, "integrate keeps unit norm");
        quat_identity(q);
        for (k = 0; k < 125; k++) quat_integrate(q, w2, 0.004f, q);
        CHECK_NEAR(quat_tilt(q), 1.0, 2e-5, "integrated tilt");
        quat_identity(q);
        {   /* a large step falls back to the exact exp map */
            float wbig[3] = {0, 200.0f, 0};
            quat_integrate(q, wbig, 0.004f, q);
            CHECK_NEAR(quat_tilt(q), 0.8, 1e-6, "large-step integrate (0.8 rad)");
        }
    }
    {   /* error: q^-1 q = identity; w >= 0 */
        float q[4], qn[4], e[4];
        quat_from_euler(0.2f, 0.1f, -2.0f, q);
        quat_error(q, q, e);
        CHECK_NEAR(e[0], 1.0, 1e-6, "error(q, q).w");
        for (i = 0; i < 4; i++) qn[i] = -q[i];                 /* same rotation, other sign */
        quat_error(q, qn, e);
        CHECK(e[0] >= 0.0f, "error() returns w >= 0 (got %g)", (double)e[0]);
        CHECK_NEAR(e[0], 1.0, 1e-6, "error(q, -q).w");
    }
}

/* ------------------------------------------------------------------------ */
/* mixer                                                                    */
/* ------------------------------------------------------------------------ */
static void make_ctrl(attitude_ctrl_t *c, att_gains_t *g, mixer_plant_t *p)
{
    fc_default_plant(p);
    fc_default_gains(g);
    attitude_ctrl_init(c, g, p, fc_throttle_exp_int());
}

static void test_mixer(void)
{
    attitude_ctrl_t c;
    att_gains_t g;
    mixer_plant_t p;
    vane_mixer_t *m;
    int k, i;
    double dmax, kslope;
    make_ctrl(&c, &g, &p);
    m = &c.mixer;
    dmax = MIX_DELTA_MAX_RAD;

    /* thrust model and floor */
    CHECK_NEAR(mixer_thrust_model(m, 0.5f), MIX_T_MAX_N * pow(0.5, MIX_THROTTLE_EXP), 1e-5, "T(0.5)");
    CHECK_NEAR(mixer_thrust_estimate(m, 0.0f), GAIN_THRUST_ESTIMATE_FLOOR_FRACTION * MIX_T_MAX_N, 1e-5, "floor");
    CHECK(mixer_thrust_floored(m, 0.0f), "u=0 is floored");
    CHECK(!mixer_thrust_floored(m, 1.0f), "u=1 is not floored");
    CHECK_NEAR(mixer_thrust_model(m, 1.7f), MIX_T_MAX_N, 1e-5, "u clipped to 1");
    /* force slope k = T S_v C_La / (2 A_e) worked out in double */
    kslope = 30.0 * (double)MIX_S_V_M2 * MIX_CL_ALPHA / (2.0 * MIX_A_E_M2);
    CHECK_NEAR(mixer_force_slope(m, 30.0f), kslope, kslope * 1e-6, "force slope");

    /* round trip inside the authority: moments -> deltas -> moments */
    for (k = 0; k < 500; k++) {
        float mom[3], d[4], back[3], t_hat = (float)(MIX_T_MAX_N * (0.3 + 0.7 * urand()));
        double gx = 2.0 * MIX_L_V_M * t_hat * (double)MIX_S_V_M2 * MIX_CL_ALPHA / (2.0 * MIX_A_E_M2);
        double gz = 4.0 * MIX_R_V_M * t_hat * (double)MIX_S_V_M2 * MIX_CL_ALPHA / (2.0 * MIX_A_E_M2);
        bool sat;
        mom[0] = (float)(0.4 * dmax * gx * srand1());
        mom[1] = (float)(0.4 * dmax * gx * srand1());
        mom[2] = (float)(0.3 * dmax * gz / GAIN_ROLL_PRIORITY_SCALE * srand1());
        sat = mixer_deflections(m, mom, t_hat, d);
        CHECK(!sat, "round trip case %d saturated", k);
        mixer_moments(m, d, t_hat, back);
        CHECK_NEAR(back[0], mom[0], 1e-5 * fabs(gx), "round trip M_x");
        CHECK_NEAR(back[1], mom[1], 1e-5 * fabs(gx), "round trip M_y");
        CHECK_NEAR(back[2], GAIN_ROLL_PRIORITY_SCALE * mom[2], 1e-5 * fabs(gz), "round trip M_z (x roll_priority_scale)");
    }
    {   /* saturation 1: |d_xy| < d_max, roll shrinks first, x/y untouched */
        float t_hat = MIX_T_MAX_N, mom[3], d[4], dx, dy, dz;
        double kk = t_hat * (double)MIX_S_V_M2 * MIX_CL_ALPHA / (2.0 * MIX_A_E_M2);
        double want_dx = 0.6 * dmax, want_dy = -0.3 * dmax, want_dz = 0.8 * dmax;
        mom[0] = (float)(want_dx * 2.0 * MIX_L_V_M * kk);
        mom[1] = (float)(want_dy * 2.0 * MIX_L_V_M * kk);
        mom[2] = (float)(want_dz * 4.0 * MIX_R_V_M * kk / GAIN_ROLL_PRIORITY_SCALE);
        CHECK(mixer_deflections(m, mom, t_hat, d), "roll-priority case saturates");
        mixer_split(d, &dx, &dy, &dz);
        CHECK_NEAR(dx, want_dx, 1e-6, "d_x kept");
        CHECK_NEAR(dy, want_dy, 1e-6, "d_y kept");
        CHECK_NEAR(dz, dmax - 0.6 * dmax, 1e-6, "d_z shrunk to d_max - max|d_xy|");
        {
            double mx = 0;
            for (i = 0; i < 4; i++) mx = fabs(d[i]) > mx ? fabs(d[i]) : mx;
            CHECK_NEAR(mx, dmax, 1e-6, "largest |d_i| = d_max");
        }
        /* negative roll keeps its sign */
        mom[2] = -mom[2];
        mixer_deflections(m, mom, t_hat, d);
        mixer_split(d, &dx, &dy, &dz);
        CHECK_NEAR(dz, -(dmax - 0.6 * dmax), 1e-6, "shrunk d_z keeps sign");
        /* saturation 2: |d_x| > d_max -> roll dropped, (d_x, d_y) scaled */
        want_dx = 1.5 * dmax;
        want_dy = 0.75 * dmax;
        mom[0] = (float)(want_dx * 2.0 * MIX_L_V_M * kk);
        mom[1] = (float)(want_dy * 2.0 * MIX_L_V_M * kk);
        CHECK(mixer_deflections(m, mom, t_hat, d), "pitch case saturates");
        mixer_split(d, &dx, &dy, &dz);
        CHECK_NEAR(dz, 0.0, 1e-7, "roll dropped");
        CHECK_NEAR(dx, dmax, 1e-6, "d_x scaled to d_max");
        CHECK_NEAR(dy, 0.5 * dmax, 1e-6, "d_y scaled by the same factor");
    }
    {   /* pwm: centre + sign deg(d) us/deg + trim */
        float d[4] = {0.0f, (float)(10.0 * FC_DEG2RAD_F), (float)(-5.0 * FC_DEG2RAD_F), 0.1f}, pw[4];
        mixer_pwm(m, d, pw);
        CHECK_NEAR(pw[0], MIX_PWM_CENTER_US + CAL_SERVO_TRIM_US_1, 1e-3, "pwm at 0");
        CHECK_NEAR(pw[1], MIX_PWM_CENTER_US + CAL_SERVO_SIGN_2 * 10.0 * MIX_PWM_US_PER_DEG + CAL_SERVO_TRIM_US_2, 2e-3, "pwm +10 deg");
        CHECK_NEAR(pw[2], MIX_PWM_CENTER_US - CAL_SERVO_SIGN_3 * 5.0 * MIX_PWM_US_PER_DEG + CAL_SERVO_TRIM_US_3, 2e-3, "pwm -5 deg");
        CHECK_NEAR(pw[3], MIX_PWM_CENTER_US + CAL_SERVO_SIGN_4 * 0.1 * 180.0 / 3.14159265358979 * MIX_PWM_US_PER_DEG + CAL_SERVO_TRIM_US_4,
                   2e-3, "pwm 0.1 rad");
    }
}

/* ------------------------------------------------------------------------ */
/* PID / integrator freeze rules                                            */
/* ------------------------------------------------------------------------ */
static void test_pid(void)
{
    rate_pid_t pid;
    rate_gains_t rg;
    float sp[3] = {1.0f, -1.0f, 0.5f}, gyro[3] = {0, 0, 0}, mo[3], wf[3];
    int k;
    att_gains_t g;
    fc_default_gains(&g);
    rg = g.rate;
    rate_pid_init(&pid, &rg, FC_DT_S);
    rate_pid_update(&pid, sp, gyro, false, mo, wf);
    CHECK_NEAR(pid.integ[0], GAIN_RATE_KI_XY * 1.0 * FC_DT_S, 1e-8, "first integration step");
    CHECK_NEAR(mo[0], GAIN_RATE_KP_XY * 1.0 + pid.integ[0], 1e-6, "P + I, D = 0 on the first call");
    {
        float before = pid.integ[0];
        rate_pid_update(&pid, sp, gyro, true, mo, wf);
        CHECK(pid.integ[0] == before, "freeze holds the integrator");
    }
    for (k = 0; k < 20000; k++) rate_pid_update(&pid, sp, gyro, false, mo, wf);
    CHECK_NEAR(pid.integ[0], GAIN_RATE_I_MAX_XY, 1e-7, "integrator clamps at +i_max_xy");
    CHECK_NEAR(pid.integ[1], -GAIN_RATE_I_MAX_XY, 1e-7, "integrator clamps at -i_max_xy");
    CHECK_NEAR(pid.integ[2], GAIN_RATE_KI_Z > 0 ? GAIN_RATE_I_MAX_Z : 0.0, 1e-7, "integrator clamps at i_max_z");
    {   /* D on the measurement: a gyro step gives -Kd * d(w_d)/dt */
        float g0[3] = {0, 0, 0}, g1[3] = {1.0f, 0, 0}, z[3] = {0, 0, 0};
        rate_pid_t p2;
        double a_g = FC_DT_S / (1.0 / (2.0 * 3.14159265358979 * GAIN_GYRO_LPF_HZ) + FC_DT_S);
        double a_d = FC_DT_S / (1.0 / (2.0 * 3.14159265358979 * GAIN_D_TERM_LPF_HZ) + FC_DT_S);
        rate_pid_init(&p2, &rg, FC_DT_S);
        rate_pid_update(&p2, z, g0, true, mo, wf);
        rate_pid_update(&p2, z, g1, true, mo, wf);
        CHECK_NEAR(wf[0], a_g, 1e-6, "gyro LPF first step");
        CHECK_NEAR(mo[0], -GAIN_RATE_KP_XY * a_g - GAIN_RATE_KD_XY * (a_d * a_g) / FC_DT_S, 1e-5, "P + D on gyro step");
    }
    {   /* controller-level freeze: saturated previous cycle or floored thrust */
        attitude_ctrl_t c;
        mixer_plant_t p;
        ctrl_output_t o;
        float q[4], w[3] = {0, 0, 0};
        float before;
        make_ctrl(&c, &g, &p);
        quat_from_euler(0.0f, 0.0f, 0.0f, q);
        attitude_ctrl_set_yaw_reference(&c, q);
        quat_from_euler(0.02f, 0.0f, 0.0f, q);                  /* small roll error: no saturation */
        attitude_ctrl_update(&c, q, w, 0.0f, &o);                /* u = 0: thrust floored */
        CHECK(!o.saturated, "small error does not saturate");
        CHECK(o.integrator[0] == 0.0f, "integrator frozen while T_hat is floored (got %g)", (double)o.integrator[0]);
        attitude_ctrl_update(&c, q, w, 1.0f, &o);
        CHECK(o.integrator[0] != 0.0f, "integrator runs at full thrust");
        quat_from_euler(0.3f, 0.0f, 0.0f, q);                   /* 17 deg roll error */
        /* force saturation with a huge rate error, check the next cycle is frozen */
        {
            float wbig[3] = {30.0f, 0, 0};
            attitude_ctrl_reset(&c);
            attitude_ctrl_update(&c, q, wbig, 1.0f, &o);
            CHECK(o.saturated, "large rate error saturates the vanes");
            before = o.integrator[0];
            attitude_ctrl_update(&c, q, wbig, 1.0f, &o);
            CHECK(o.integrator[0] == before, "integrator frozen in the cycle after saturation");
        }
    }
}

/* ------------------------------------------------------------------------ */
/* altitude filter step response vs an independent double implementation   */
/* ------------------------------------------------------------------------ */
static void test_alt_filter(void)
{
    alt_filter_t f;
    double k1, k2, wn = 2.0 * 3.14159265358979 * ALT_CROSSOVER_HZ, h = 0, v = 0, lat = 0.01, maxdiff = 0, peak = 0;
    uint32_t t;
    int n_baro = FC_BARO_DIV;
    k1 = 2.0 * ALT_ZETA * wn;
    k2 = wn * wn;
    fc_default_alt_filter(&f, (float)lat);
    alt_filter_reset(&f, 0.0f);
    alt_filter_correct(&f, 0.0f, 0);                          /* first baro sample: time base only */
    for (t = 1; t <= 10 * FC_LOOP_HZ; t++) {
        alt_filter_predict(&f, 0.0f, FC_DT_S);
        h += v * (double)FC_DT_S;
        if (t % n_baro == 0) {
            double r, dtb = n_baro * (double)FC_DT_S;
            alt_filter_correct(&f, 10.0f, t);
            r = (10.0 + v * lat) - h;
            h += k1 * r * dtb;
            v += k2 * r * dtb;
        }
        if (fabs(f.h - h) > maxdiff) maxdiff = fabs(f.h - h);
        if (f.h > peak) peak = f.h;
    }
    CHECK(maxdiff < 2e-4, "alt filter vs double reference: max |dh| = %g m", maxdiff);
    CHECK_NEAR(f.h, 10.0, 0.02, "alt filter settles on the baro step (10 s)");
    CHECK_NEAR(f.v, 0.0, 0.02, "alt filter v settles");
    CHECK(peak > 10.0 && peak < 13.0, "step overshoot of the 2nd-order filter within 30 %% (peak %g)", peak);
    /* predict: constant acceleration integrates exactly */
    fc_default_alt_filter(&f, 0.0f);
    alt_filter_reset(&f, 0.0f);
    for (t = 0; t < FC_LOOP_HZ; t++) alt_filter_predict(&f, 2.0f, FC_DT_S);
    CHECK_NEAR(f.h, 1.0, 1e-4, "1/2 a t^2 after 1 s at 2 m/s^2");
    CHECK_NEAR(f.v, 2.0, 1e-5, "a t after 1 s");
}

/* ------------------------------------------------------------------------ */
/* estimator, EDF model, MS5611, log framing                               */
/* ------------------------------------------------------------------------ */
static void test_estimator(void)
{
    att_est_t e;
    float a[3], q[4], b0[3] = {0.01f, -0.02f, 0.005f}, w[3];
    int k;
    double roll0 = 0.2, pitch0 = -0.1, sum = 0.0;
    fc_default_estimator(&e);
    CHECK(e.n_avg == GAIN_EST_PAD_N_AVG && !e.aligned, "estimator starts unaligned");
    /* body at roll 0.2 / pitch -0.1: gravity reaction seen in the body = R^T (0,0,g) */
    quat_from_euler((float)roll0, (float)pitch0, 0.0f, q);
    {
        float gw[3] = {0, 0, (float)FC_G0};
        quat_rotate_inverse(q, gw, a);
    }
    att_est_level_from_accel(a, q);
    {
        float r, p, y;
        quat_to_euler(q, &r, &p, &y);
        CHECK_NEAR(r, roll0, 2e-6, "level_from_accel roll");
        CHECK_NEAR(p, pitch0, 2e-6, "level_from_accel pitch");
        CHECK_NEAR(y, 0.0, 2e-6, "level_from_accel yaw = 0");
    }
    /* the half-angle levelling equals atan2 + from_euler for every direction (incl. upside down) */
    for (k = 0; k < 2000; k++) {
        float f[3], qa[4], qb[4], r, p;
        double dmax = 0.0;
        int j;
        f[0] = (float)(10.0 * srand1());
        f[1] = (float)(10.0 * srand1());
        f[2] = (float)(10.0 * srand1());
        if (k == 0) { f[0] = 0.0f; f[1] = 0.0f; f[2] = -9.8f; }      /* exactly inverted */
        if (k == 1) { f[0] = 3.0f; f[1] = -0.0f; f[2] = -9.8f; }
        if (k == 2) { f[0] = -9.8f; f[1] = 0.0f; f[2] = 0.0f; }      /* pitch +90 */
        att_est_level_from_accel(f, qa);
        r = atan2f(f[1], f[2]);
        p = atan2f(-f[0], sqrtf(f[1] * f[1] + f[2] * f[2]));
        quat_from_euler(r, p, 0.0f, qb);
        for (j = 0; j < 4; j++) {
            double d = fabs((double)qa[j] - qb[j]);
            if (d > dmax) dmax = d;
        }
        CHECK(dmax < 2e-6, "level_from_accel half-angle form vs trig, case %d: %g", k, dmax);
    }
    /* pad: running mean of the first N_avg samples ... */
    for (k = 1; k <= 10; k++) {
        float g[3] = {(float)k, 0.0f, 0.0f};
        att_est_update(&e, g, a, true);
        sum += k;
    }
    CHECK_NEAR(e.bias[0], sum / 10.0, 1e-5, "running mean while n < N_avg");
    CHECK(e.aligned && e.n == 10, "aligned after the first pad sample, n counts");
    /* ... then an EMA with alpha = 1/N_avg */
    fc_default_estimator(&e);
    for (k = 0; k < (int)GAIN_EST_PAD_N_AVG + 50; k++) att_est_update(&e, b0, a, true);
    CHECK(e.n == GAIN_EST_PAD_N_AVG, "n saturates at N_avg");
    {
        float g1[3] = {b0[0] + 1.0f, b0[1], b0[2]};
        att_est_update(&e, g1, a, true);
        CHECK_NEAR(e.bias[0], b0[0] + 1.0 / GAIN_EST_PAD_N_AVG, 1e-6, "EMA step 1/N_avg");
    }
    att_est_rate(&e, b0, w);
    CHECK_NEAR(w[0], -1.0 / GAIN_EST_PAD_N_AVG, 1e-6, "rate = gyro - bias");
    {
        float r, p, y;
        quat_to_euler(e.q, &r, &p, &y);
        CHECK_NEAR(r, roll0, 1e-5, "pad attitude from the averaged specific force");
    }
    /* after launch: gyro only, bias frozen, accelerometer ignored */
    {
        float big[3] = {0, 0, 30.0f}, bias_before[3], g2[3];
        int i;
        for (i = 0; i < 3; i++) bias_before[i] = e.bias[i];
        quat_identity(e.q);
        for (i = 0; i < 3; i++) g2[i] = e.bias[i];
        g2[2] += 0.5f;
        for (k = 0; k < FC_LOOP_HZ; k++) att_est_update(&e, g2, big, false);
        for (i = 0; i < 3; i++) CHECK(e.bias[i] == bias_before[i], "bias frozen after launch");
        CHECK_NEAR(quat_yaw(e.q), 0.5, 2e-5, "gyro-only integration after launch (0.5 rad/s for 1 s)");
        CHECK_NEAR(quat_tilt(e.q), 0.0, 1e-3, "accelerometer not used after launch");
    }
}

static void test_edf(void)
{
    edf_model_t m;
    float T = 0.0f;
    int k;
    fc_default_edf(&m);
    CHECK_NEAR(edf_thrust_steady(&m, 0.5f), MIX_T_MAX_N * pow(0.5, MIX_THROTTLE_EXP), 1e-5, "T_cmd(0.5)");
    for (k = 0; k < (int)(EDF_TAU_S * FC_LOOP_HZ + 0.5); k++) T = edf_step_fixed(&m, T, 1.0f);
    CHECK_NEAR(T, MIX_T_MAX_N * (1.0 - exp(-(double)k * FC_DT_S / EDF_TAU_S)), 1e-4, "thrust lag after ~1 tau");
    CHECK_NEAR(edf_step(&m, 0.0f, 1.0f, EDF_TAU_S), MIX_T_MAX_N * (1.0 - exp(-1.0)), 1e-4, "step(dt = tau)");
    CHECK_NEAR(edf_exhaust_velocity(&m, MIX_T_MAX_N), sqrt(MIX_T_MAX_N / (EDF_RHO * (double)MIX_A_E_M2)), 1e-3, "V_e");
    CHECK_NEAR(edf_dynamic_pressure(&m, MIX_T_MAX_N), MIX_T_MAX_N / (2.0 * MIX_A_E_M2), 0.1, "q_e = T/(2 A_e)");
    CHECK_NEAR(edf_throttle_for_thrust(&m, edf_thrust_steady(&m, 0.7f)), 0.7, 1e-6, "inverse thrust model");
}

static void test_ms5611(void)
{
    uint16_t prom[8] = {0, 40127, 36924, 23317, 23282, 33464, 28312, 0};
    uint16_t an520[8] = {0x3132, 0x3334, 0x3536, 0x3738, 0x3940, 0x4142, 0x4344, 0x4500};
    int32_t temp, p;
    ms5611_compensate(prom, 9085466UL, 8569150UL, &temp, &p);
    CHECK(temp == 2007, "datasheet example TEMP = 2007 (got %ld)", (long)temp);
    CHECK(p == 100009, "datasheet example P = 100009 Pa (got %ld)", (long)p);
    prom[7] = (uint16_t)(0xBE00u | ms5611_crc4(prom));
    CHECK(ms5611_prom_ok(prom), "PROM with its own CRC accepted");
    prom[3] ^= 0x0010u;
    CHECK(!ms5611_prom_ok(prom), "corrupted PROM rejected");
    printf("info: AN520 example vector crc4 = 0x%X\n", ms5611_crc4(an520));
    CHECK(ms5611_crc4(an520) == 0xB, "AN520 example CRC = 0xB");
    {   /* second order below 20 C lowers TEMP by dT^2/2^31 */
        uint16_t pr[8] = {0, 40127, 36924, 23317, 23282, 33464, 28312, 0};
        uint32_t d2 = 8569150UL - 600000UL;                      /* about -5 C */
        double dT = (double)d2 - 33464.0 * 256.0, t1 = 2000.0 + dT * 28312.0 / 8388608.0;
        ms5611_compensate(pr, 9085466UL, d2, &temp, &p);
        CHECK_NEAR(temp, t1 - floor(dT * dT / 2147483648.0), 1.0, "second-order TEMP");
    }
    {   /* ISA: 1 hPa ~ 8.3 m near sea level; standard pressure -> 0 m */
        CHECK_NEAR(ms5611_pressure_altitude(101325.0f, (float)FC_G0), 0.0, 1e-3, "h(P0) = 0");
        CHECK_NEAR(ms5611_pressure_altitude(101225.0f, (float)FC_G0) - ms5611_pressure_altitude(101325.0f, (float)FC_G0),
                   8.32, 0.02, "dh for -100 Pa at sea level");
        CHECK_NEAR(ms5611_pressure_altitude(89874.6f, (float)FC_G0), 1000.0, 0.5, "ISA 1000 m = 89874.6 Pa");
    }
}

/* BMP280 datasheet appendix 8.1: the compensation in double precision (the reference the integer code of 3.11.3 is
 * checked against; written from the published datasheet) */
static double bmp280_ref_double(const bmp280_calib_t *c, uint32_t adc_p, uint32_t adc_t, double *t_c)
{
    double var1 = ((double)adc_t / 16384.0 - (double)c->t1 / 1024.0) * (double)c->t2;
    double var2 = ((double)adc_t / 131072.0 - (double)c->t1 / 8192.0);
    double p;
    int32_t t_fine;
    var2 = var2 * var2 * (double)c->t3;
    t_fine = (int32_t)(var1 + var2);
    *t_c = (var1 + var2) / 5120.0;
    var1 = (double)t_fine / 2.0 - 64000.0;
    var2 = var1 * var1 * (double)c->p6 / 32768.0;
    var2 = var2 + var1 * (double)c->p5 * 2.0;
    var2 = var2 / 4.0 + (double)c->p4 * 65536.0;
    var1 = ((double)c->p3 * var1 * var1 / 524288.0 + (double)c->p2 * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * (double)c->p1;
    p = 1048576.0 - (double)adc_p;
    p = (p - var2 / 4096.0) * 6250.0 / var1;
    var1 = (double)c->p9 * p * p / 2147483648.0;
    var2 = p * (double)c->p8 / 32768.0;
    return p + (var1 + var2 + (double)c->p7) / 16.0;
}

/* 2026-11-10: the BMP280 integer compensation (core/bmp280_math.c) against the datasheet's calculation example (3.12:
 * its trimming parameters and raw values; t_fine and T are the table's, the pressure is compared with the appendix 8.1
 * double formula because the table's pressure is that formula's) and against the double formula on the driver's
 * operating grid (temperature x1 = adc_T in steps of 16; the integer T code drops the lower bits). */
static void test_bmp280(void)
{
    /* dig_T1..T3, dig_P1..P9 of the example as the registers 0x88 .. 0x9F hold them (little endian) */
    static const uint8_t raw[BMP280_CALIB_LEN] = {
        0x70, 0x6B, 0x43, 0x67, 0x18, 0xFC, 0x7D, 0x8E, 0x43, 0xD6, 0xD0, 0x0B,
        0x27, 0x0B, 0x8C, 0x00, 0xF9, 0xFF, 0x8C, 0x3C, 0xF8, 0xC6, 0x70, 0x17};
    static const uint8_t data[6] = {0x65, 0x5A, 0xC0, 0x7E, 0xED, 0x00};      /* 0xF7 .. 0xFC: adc_P 415148, adc_T 519888 */
    bmp280_calib_t c, z;
    bmp280_pwork_t w;
    int32_t temp = -1, t_fine;
    uint32_t p_q8 = 1u, adc_p, adc_t, q;
    double t_c, p_ref, worst = 0.0, prev;
    bmp280_calib_parse(raw, &c);
    CHECK(c.t1 == 27504u && c.t2 == 26435 && c.t3 == -1000, "dig_T1..T3 parsed (%u %d %d)", c.t1, c.t2, c.t3);
    CHECK(c.p1 == 36477u && c.p2 == -10685 && c.p3 == 3024 && c.p4 == 2855 && c.p5 == 140 && c.p6 == -7 &&
          c.p7 == 15500 && c.p8 == -14600 && c.p9 == 6000, "dig_P1..P9 parsed");
    CHECK(bmp280_calib_ok(&c), "example trimming parameters accepted");
    adc_p = bmp280_raw20(data);
    adc_t = bmp280_raw20(data + 3);
    CHECK(adc_p == 415148UL && adc_t == 519888UL, "raw 20-bit values (%lu %lu)", (unsigned long)adc_p, (unsigned long)adc_t);
    t_fine = bmp280_t_fine(&c, adc_t);
    CHECK(t_fine == 128422L, "datasheet example t_fine = 128422 (got %ld)", (long)t_fine);
    CHECK(bmp280_compensate(&c, adc_p, adc_t, &temp, &p_q8), "example compensates");
    CHECK(temp == 2508, "datasheet example T = 2508 (25.08 C) (got %ld)", (long)temp);
    CHECK(p_q8 == 25767233UL, "example P = 25767233 / 256 Pa (got %lu)", (unsigned long)p_q8);
    p_ref = bmp280_ref_double(&c, adc_p, adc_t, &t_c);
    printf("info: BMP280 example: int64 %.4f Pa, double formula %.4f Pa, T %.4f C\n", p_q8 / 256.0, p_ref, t_c);
    CHECK_NEAR(t_c, 25.08, 0.01, "double-formula T of the example");
    CHECK_NEAR(p_ref, 100653.27, 0.05, "double-formula P of the example (datasheet table: 100653.27 Pa)");
    CHECK_NEAR(p_q8 / 256.0, p_ref, 0.05, "int64 P of the example = the double formula");
    /* the two background steps of the driver = the one-call form */
    CHECK(bmp280_p_part1(&c, t_fine, &w), "step 1 (non-zero divisor)");
    CHECK(bmp280_p_part2(&c, adc_p, &w) == p_q8, "step 1 + step 2 = bmp280_compensate");
    /* the operating grid: about 300 .. 1100 hPa, -20 .. +60 C around the example */
    for (adc_t = 400000UL; adc_t <= 620000UL; adc_t += 16UL * 500UL) {
        prev = 1e12;
        for (adc_p = 250000UL; adc_p <= 500000UL; adc_p += 1237UL) {
            double e;
            if (!bmp280_compensate(&c, adc_p, adc_t, &temp, &q)) {
                CHECK(0, "grid point compensates");
                continue;
            }
            e = fabs(q / 256.0 - bmp280_ref_double(&c, adc_p, adc_t, &t_c));
            if (e > worst) worst = e;
            CHECK(q / 256.0 < prev, "P falls as adc_P rises");
            prev = q / 256.0;
        }
    }
    printf("info: BMP280 int64 vs double formula on the x1 temperature grid: worst %.4f Pa\n", worst);
    CHECK(worst <= 0.15, "int64 code within 0.15 Pa of the double formula (worst %.4f Pa)", worst);
    /* refused, never guessed */
    z = c;
    z.p1 = 0;
    temp = -77;
    p_q8 = 77u;
    CHECK(!bmp280_calib_ok(&z), "dig_P1 = 0 rejected");
    CHECK(!bmp280_compensate(&z, 415148UL, 519888UL, &temp, &p_q8) && temp == -77 && p_q8 == 77u,
          "zero divisor: no result, outputs untouched");
    memset(&z, 0xFF, sizeof z);
    CHECK(!bmp280_calib_ok(&z), "erased (all 1) trimming parameters rejected");
    memset(&z, 0, sizeof z);
    CHECK(!bmp280_calib_ok(&z), "all-0 trimming parameters rejected");
    /* the driver's schedule (hal/bmp280.h): the read comes after the longest measurement, inside the cycle */
    CHECK(BMP280_MEAS_TICKS * FC_TICK_US >= BMP280_MEAS_MAX_US, "read after the longest measurement");
    CHECK(BMP280_MEAS_TICKS < FC_BARO_DIV, "read before the next forced write");
    CHECK(BMP280_READ_PHASE == (BMP280_START_PHASE + BMP280_MEAS_TICKS) % FC_BARO_DIV && BMP280_READ_PHASE != BMP280_START_PHASE,
          "read phase = start phase + the measurement ticks");
    CHECK(BMP280_START_PHASE < FC_BARO_DIV, "start phase inside the baro cycle");
    CHECK_NEAR(BMP280_LATENCY_S, ((FC_BARO_DIV + 1) * (double)FC_TICK_US - 7250.0) * 1e-6, 1e-7, "BMP280_LATENCY_S");
}

static void test_log(void)
{
    const uint8_t abcde[5] = {'a', 'b', 'c', 'd', 'e'};
    uint8_t pay[LOG_MAX_PAYLOAD], fr[LOG_MAX_PAYLOAD + LOG_FRAME_OVERHEAD];
    log_flight_t r;
    uint8_t n;
    CHECK(log_fletcher16(abcde, 5) == 0xC8F0, "fletcher16(\"abcde\") = 0xC8F0 (got 0x%04X)", log_fletcher16(abcde, 5));
    memset(&r, 0, sizeof r);
    n = log_pack_flight(&r, pay);
    CHECK(n <= LOG_MAX_PAYLOAD, "flight payload %u bytes fits", n);
    printf("info: flight record payload %u bytes, frame %u bytes, %u B/s at %d Hz (link %lu B/s)\n", n,
           n + LOG_FRAME_OVERHEAD, (n + LOG_FRAME_OVERHEAD) * FC_LOG_HZ, FC_LOG_HZ, (unsigned long)(FC_LOG_BAUD / 10));
    CHECK((unsigned long)(n + LOG_FRAME_OVERHEAD) * FC_LOG_HZ < FC_LOG_BAUD / 10, "log stream fits the UART");
    CHECK(log_frame(LOG_TYPE_FLIGHT, pay, n, fr) == n + LOG_FRAME_OVERHEAD, "frame length");
    CHECK(n == LOG_FLIGHT_LEN, "LOG_FLIGHT_LEN matches the packer (%u)", n);
    {   /* the spread packer (parts + running checksum) builds the identical frame */
        uint8_t fb[LOG_FLIGHT_LEN + LOG_FRAME_OVERHEAD];
        uint16_t s1 = 0, s2 = 0, nf;
        uint8_t part, i;
        for (i = 0; i < 4; i++) { r.q[i] = 0.25f * (float)i - 0.4f; r.delta[i] = -0.01f * (float)i; }
        for (i = 0; i < 3; i++) { r.gyro_f[i] = 1.5f - (float)i; r.accel[i] = 9.0f * (float)i; r.moments[i] = 0.3f; }
        r.h_est = 123.456f; r.vz_est = -7.5f; r.h_baro = 99.9f; r.throttle = 0.5f; r.v_cell = 3.7f; r.tilt_deg = 44.4f;
        r.thrust_est = 1e9f;                           /* saturates */
        r.att_err[0] = 0.0f / 0.0f;                    /* NaN is logged as 0 */
        nf = log_frame(LOG_TYPE_FLIGHT, pay, log_pack_flight(&r, pay), fr);
        fb[0] = LOG_SYNC0; fb[1] = LOG_SYNC1; fb[2] = LOG_TYPE_FLIGHT; fb[3] = LOG_FLIGHT_LEN;
        log_fletcher16_acc(&s1, &s2, fb + 2, 2);
        for (part = 0; part < LOG_FLIGHT_PARTS; part++) {
            uint8_t a = log_flight_part_start(part), b = log_pack_flight_part(&r, fb + 4, part);
            log_fletcher16_acc(&s1, &s2, fb + 4 + a, (uint16_t)(b - a));
        }
        fb[4 + LOG_FLIGHT_LEN] = (uint8_t)s1;
        fb[5 + LOG_FLIGHT_LEN] = (uint8_t)s2;
        CHECK(nf == sizeof fb && memcmp(fb, fr, nf) == 0, "spread packer == one-shot frame");
        CHECK(pay[22 + 12] == 0 && pay[22 + 13] == 0, "NaN logged as 0");
        CHECK(pay[88] == 0xFF && pay[89] == 0xFF, "saturated u16 (thrust, last field) = 65535");
    }
    {   /* 'S' (format 4): every field at its documented offset, little-endian */
        log_status_t st;
        uint16_t *f16[] = {&st.stack_free, &st.twi_errors, &st.uart_dropped, &st.link_frames, &st.link_bad, &st.link_other,
                           &st.radio_errors, &st.replies, &st.baro_fails, &st.imu_fails, &st.vbat_mv};
        uint16_t *g16[] = {&st.il_cmds, &st.il_snaps, &st.il_bad, &st.il_uart_errors, &st.il_dropped};
        uint8_t i, ok = 1;
        memset(&st, 0, sizeof st);
        st.tick = 0x04030201UL;
        for (i = 0; i < 11u; i++) *f16[i] = (uint16_t)(0x1100u * (i + 1u) + i);
        st.reset_cause = 0x08;
        st.rssi_dbm = -97;
        for (i = 0; i < 5u; i++) *g16[i] = (uint16_t)(0xA0B0u + i);
        st.com_age_s = 3;
        st.com_flags = (uint8_t)(0x05u | LOG_COM_TAG_MISMATCH);
        n = log_pack_status(&st, pay);
        CHECK(n == LOG_STATUS_LEN && n == 40u, "status payload %u bytes (format %d: 40)", n, LOG_FORMAT_VERSION);
        CHECK(pay[0] == 0x01 && pay[3] == 0x04, "status tick LE");
        for (i = 0; i < 11u; i++)
            ok &= (uint8_t)(pay[4 + 2 * i] == (uint8_t)*f16[i] && pay[5 + 2 * i] == (uint8_t)(*f16[i] >> 8));
        CHECK(ok, "status u16 fields 4..25");
        CHECK(pay[26] == 0x08 && (int8_t)pay[27] == -97, "status reset cause / rssi at 26 / 27");
        for (i = 0, ok = 1; i < 5u; i++)
            ok &= (uint8_t)(pay[28 + 2 * i] == (uint8_t)*g16[i] && pay[29 + 2 * i] == (uint8_t)(*g16[i] >> 8));
        CHECK(ok, "status ilink counters at 28..37");
        CHECK(pay[38] == 3 && pay[39] == 0x85, "status COM age / flags at 38 / 39");
    }
    {   /* 'C': FCU tick + the IL_STAT payload byte for byte */
        il_stat_t s;
        uint8_t sp[IL_STAT_LEN], i;
        memset(&s, 0, sizeof s);
        s.com_ticks = 0x11223344UL;
        s.flags = IL_STAT_RADIO_OK | IL_STAT_BOOT;
        s.params_tag = 0x5A;
        s.loop_us_max = 0x0BCD;
        il_pack_stat(&s, sp);
        CHECK(LOG_COM_STAT_LEN == IL_STAT_LEN, "LOG_COM_STAT_LEN == IL_STAT_LEN");
        n = log_pack_com(0xCAFEF00DUL, sp, pay);
        CHECK(n == LOG_COM_LEN && n == 40u, "com payload %u bytes", n);
        CHECK(pay[0] == 0x0D && pay[1] == 0xF0 && pay[2] == 0xFE && pay[3] == 0xCA, "com tick LE");
        for (i = 0; i < IL_STAT_LEN && pay[4 + i] == sp[i]; i++) {
        }
        CHECK(i == IL_STAT_LEN, "com payload = IL_STAT bytes (first difference at %u)", i);
        CHECK(n + LOG_FRAME_OVERHEAD <= LOG_MAX_PAYLOAD + LOG_FRAME_OVERHEAD && LOG_STATUS_LEN <= LOG_MAX_PAYLOAD,
              "status / com records fit LOG_MAX_PAYLOAD");
    }
}

/* ------------------------------------------------------------------------ */
/* hal/pwm.h pwm_us_to_counts: the bit-level version against the v0.5 float  */
/* expression, for every float of the exponents around the outputs' range    */
/* ------------------------------------------------------------------------ */
static uint16_t pwm_counts_ref(float us)          /* src/hal/pwm.c up to v0.5 */
{
    float c = us * (float)FC_PWM_COUNTS_PER_US + 0.5f;
    if (!(c > 0.0f))
        return 0;
    if (c >= (float)FC_PWM_TOP)
        return FC_PWM_TOP;
    return (uint16_t)c;
}

static void test_pwm_counts(void)
{
    static const float edge[] = {0.0f, -0.0f, 0.2499999f, 0.25f, 0.2500001f, 0.5f, 0.7499999f, 0.75f, 1000.0f, 1000.25f,
                                 1000.5f, 1500.0f, 1999.5f, 2000.0f, 19999.0f, 19999.25f, 19999.5f, 32767.75f, 32768.0f,
                                 65535.5f, 65536.0f, 1e30f, -1e-30f, -1500.0f, 1e-45f};
    uint32_t u, n = 0, bad = 0, first = 0;
    float f;
    uint8_t i;
    for (u = (100UL << 23); u < (161UL << 23); u++) {       /* every float from 2^-27 to 2^34 */
        memcpy(&f, &u, sizeof f);
        n++;
        if (pwm_us_to_counts(f) != pwm_counts_ref(f) && bad++ == 0) first = u;
    }
    for (u = 0x00000007UL; u >= 0x00000007UL; u += 4093UL * 4099UL) {   /* a stride over all 2^32 patterns, both signs */
        memcpy(&f, &u, sizeof f);
        n++;
        if (pwm_us_to_counts(f) != pwm_counts_ref(f) && bad++ == 0) first = u;
        if (u > 0xFFFFFFFFUL - 4093UL * 4099UL) break;
    }
    for (i = 0; i < sizeof edge / sizeof edge[0]; i++) {
        n++;
        if (pwm_us_to_counts(edge[i]) != pwm_counts_ref(edge[i]) && bad++ == 0) memcpy(&first, &edge[i], sizeof first);
    }
    u = 0x7FC00000UL;                                         /* NaN, +inf, -inf */
    memcpy(&f, &u, sizeof f);
    CHECK(pwm_us_to_counts(f) == 0 && pwm_counts_ref(f) == 0, "pwm_us_to_counts(NaN) = 0");
    u = 0x7F800000UL;
    memcpy(&f, &u, sizeof f);
    CHECK(pwm_us_to_counts(f) == FC_PWM_TOP && pwm_counts_ref(f) == FC_PWM_TOP, "pwm_us_to_counts(+inf) = TOP");
    u = 0xFF800000UL;
    memcpy(&f, &u, sizeof f);
    CHECK(pwm_us_to_counts(f) == 0 && pwm_counts_ref(f) == 0, "pwm_us_to_counts(-inf) = 0");
    CHECK(bad == 0, "pwm_us_to_counts == float expression for %lu inputs (%lu differ, first 0x%08lX)", (unsigned long)n,
          (unsigned long)bad, (unsigned long)first);
    CHECK(pwm_us_to_counts(1500.0f) == 3000u && pwm_us_to_counts(1000.5f) == 2001u && pwm_us_to_counts(1999.5f) == 3999u,
          "pwm_us_to_counts at the servo centre / release positions");
    printf("info: pwm_us_to_counts checked against the float expression for %lu floats\n", (unsigned long)n);
}

/* ------------------------------------------------------------------------ */
/* link protocol (SPEC 10.3)                                                */
/* ------------------------------------------------------------------------ */
static int scale_of(const char *field, uint16_t *scale, int32_t *lo, int32_t *hi)
{
    *lo = LP_I16_MIN;
    *hi = LP_I16_MAX;
    if (!strcmp(field, "h_dm")) { *scale = LP_H_SCALE; return 1; }
    if (!strcmp(field, "vz_cms")) { *scale = LP_VZ_SCALE; return 1; }
    if (!strcmp(field, "roll_cdeg") || !strcmp(field, "pitch_cdeg") || !strcmp(field, "yaw_cdeg")) {
        *scale = LP_ANGLE_SCALE; return 1;
    }
    *lo = 0;
    *hi = LP_U16_MAX;
    if (!strcmp(field, "tilt_cdeg")) { *scale = LP_ANGLE_SCALE; return 1; }
    if (!strcmp(field, "throttle_milli")) { *scale = LP_THROTTLE_SCALE; return 1; }
    if (!strcmp(field, "vcell_mv")) { *scale = LP_VCELL_SCALE; return 1; }
    return 0;
}

/* the 15 TELEM v1 fields */
static void telem_fields(const lp_telem_t *t, long long f[LV_NF])
{
    f[0] = t->state; f[1] = t->flags; f[2] = t->reason; f[3] = t->ack_seq; f[4] = t->nonce; f[5] = (long long)t->t_ms;
    f[6] = t->h_dm; f[7] = t->vz_cms; f[8] = t->roll_cdeg; f[9] = t->pitch_cdeg; f[10] = t->yaw_cdeg;
    f[11] = t->tilt_cdeg; f[12] = t->throttle_milli; f[13] = t->vcell_mv; f[14] = t->loop_max_us;
}

static int telem_eq(const lp_telem_t *a, const lp_telem_t *b)
{
    long long fa[LV_NF], fb[LV_NF];
    int i;
    telem_fields(a, fa);
    telem_fields(b, fb);
    for (i = 0; i < LV_NF; i++)
        if (fa[i] != fb[i]) return 0;
    return 1;
}

static void test_link_proto(void)
{
    static const uint8_t ascii[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    int i, k;
    CHECK(lp_crc16(ascii, 9, LP_CRC_INIT) == LV_CRC_CHECK, "CRC-16/CCITT-FALSE(\"123456789\") = 0x%04X (vectors 0x%04X)",
          lp_crc16(ascii, 9, LP_CRC_INIT), LV_CRC_CHECK);
    CHECK(LV_CRC_CHECK == 0x29B1, "catalogue check value 0x29B1");
    {   /* bitwise MSB-first reference, computed here, against the branch-free form */
        for (k = 0; k < 300; k++) {
            uint8_t b[20];
            uint16_t ref = 0xFFFF;
            int n = 1 + (int)(urand() * 19.0), j, bit;
            for (j = 0; j < n; j++) b[j] = (uint8_t)(urand() * 256.0);
            for (j = 0; j < n; j++) {
                ref ^= (uint16_t)(b[j] << 8);
                for (bit = 0; bit < 8; bit++) ref = (ref & 0x8000u) ? (uint16_t)((ref << 1) ^ 0x1021u) : (uint16_t)(ref << 1);
            }
            CHECK(lp_crc16(b, (uint8_t)n, LP_CRC_INIT) == ref, "crc random case %d", k);
        }
    }
    CHECK(LP_CMD_FRAME_BYTES == 13 && LP_TELEM_FRAME_BYTES == 36 && LP_LEN_MAX == 40 && LP_FRAME_MAX_BYTES == 45 &&
          LV_LEN_MAX == LP_LEN_MAX && LP_CMD_MAX == LP_CMD_KILL,
          "frame sizes 13 / 36, LEN_MAX 40 = the reference's, command codes 0..7 (8..11 unknown)");
    CHECK(LV_VEHICLE_ID == LINK_VEHICLE_ID, "vectors built for vehicle %d, params %d", LV_VEHICLE_ID, LINK_VEHICLE_ID);
    /* serial-number arithmetic */
    CHECK(lp_seq_newer(1, 0) && lp_seq_newer(127, 0) && !lp_seq_newer(128, 0) && !lp_seq_newer(0, 0) &&
          lp_seq_newer(3, 250) && !lp_seq_newer(250, 3), "seq_newer: 1..127 ahead is newer, wrap-around");
    /* quantisation: the reference's scaling vectors, then edge cases worked out by hand */
    for (i = 0; i < LV_N_SCALING; i++) {
        uint16_t sc;
        int32_t lo, hi;
        CHECK(scale_of(LV_SCALING[i].field, &sc, &lo, &hi), "known field %s", LV_SCALING[i].field);
        CHECK(lp_quantize((float)LV_SCALING[i].physical, sc, lo, hi) == LV_SCALING[i].value, "scaling %s %g -> %ld (got %ld)",
              LV_SCALING[i].field, LV_SCALING[i].physical, LV_SCALING[i].value,
              (long)lp_quantize((float)LV_SCALING[i].physical, sc, lo, hi));
    }
    CHECK(lp_quantize(0.125f, 100, -32768L, 32767L) == 13 && lp_quantize(-0.125f, 100, -32768L, 32767L) == -13,
          "12.5 -> 13, -12.5 -> -13 (half away from zero)");
    CHECK(lp_quantize(0.015f, 100, -32768L, 32767L) == 1,
          "0.015f = 0.0149999997 -> 1 (exact; the float32 product rounds to 1.5 and would give 2)");
    CHECK(lp_quantize(2.5f, 1, 0, 65535L) == 3 && lp_quantize(-2.5f, 1, -32768L, 32767L) == -3, "2.5 -> 3, -2.5 -> -3");
    CHECK(lp_quantize(-1.0f, 100, 0, 65535L) == 0 && lp_quantize(-0.0f, 10, -32768L, 32767L) == 0, "negative -> 0 unsigned, -0 -> 0");
    CHECK(lp_quantize(NAN, 10, -32768L, 32767L) == 0, "NaN -> 0");
    CHECK(lp_quantize(INFINITY, 10, -32768L, 32767L) == 32767 && lp_quantize(-INFINITY, 10, -32768L, 32767L) == -32768,
          "+-inf saturate");
    CHECK(lp_quantize(3276.75f, 10, -32768L, 32767L) == 32767 && lp_quantize(-3276.85f, 10, -32768L, 32767L) == -32768,
          "saturation at the i16 rails");
    CHECK(lp_quantize(1e30f, 1000, 0, 65535L) == 65535 && lp_quantize(1e-40f, 1000, 0, 65535L) == 0, "huge / denormal");
    CHECK(lp_quantize(65.536f, 1000, 0, 65535L) == 65535 && lp_quantize(65.5f, 1000, 0, 65535L) == 65500, "u16 rail");
    for (k = 0; k <= 1000; k++)
        CHECK(lp_quantize((float)k / 1000.0f, 1000, 0, 65535L) == k, "milli %d", k);
    {   /* random floats: exact integer result vs a double computation (float x scale is exact in double) */
        long bad = 0;
        for (k = 0; k < 200000; k++) {
            float x = (float)(srand1() * pow(10.0, 4.0 * srand1()));
            static const uint16_t sc[] = {1, 10, 100, 1000};
            uint16_t s = sc[k & 3];
            double dv = (double)x * s, r = dv >= 0 ? floor(dv + 0.5) : -floor(-dv + 0.5);
            long want = r > 32767 ? 32767 : (r < -32768 ? -32768 : (long)r);
            if (lp_quantize(x, s, -32768L, 32767L) != want) bad++;
        }
        CHECK(bad == 0, "200000 random quantisations vs double: %ld differ", bad);
    }
    /* payload packing round trip and field layout */
    {
        lp_telem_t t, u;
        uint8_t pl[LP_TELEM_PAYLOAD_LEN], fr[LP_TELEM_FRAME_BYTES];
        lp_cmd_t c = {LP_CMD_THROTTLE, 0xBEEF, 0x1234}, d;
        uint8_t cp[LP_CMD_PAYLOAD_LEN];
        lp_pack_cmd(&c, cp);
        CHECK(cp[0] == 4 && cp[1] == 0xEF && cp[2] == 0xBE && cp[3] == 0x34 && cp[4] == 0x12, "CMD payload <BHH");
        lp_unpack_cmd(cp, &d);
        CHECK(d.cmd == c.cmd && d.arg == c.arg && d.nonce == c.nonce, "CMD unpack");
        memset(&t, 0, sizeof t);
        t.state = 5; t.flags = 0xA5; t.reason = 0x4B; t.ack_seq = 200; t.nonce = 0xCAFE; t.t_ms = 0x89ABCDEFUL;
        t.h_dm = -2; t.vz_cms = 32767; t.roll_cdeg = -32768; t.pitch_cdeg = 1; t.yaw_cdeg = -1; t.tilt_cdeg = 65535;
        t.throttle_milli = 1000; t.vcell_mv = 3950; t.loop_max_us = 2500;
        lp_pack_telem(&t, pl);
        CHECK(pl[6] == 0xEF && pl[9] == 0x89 && pl[10] == 0xFE && pl[11] == 0xFF && pl[27] == 0x09, "TELEM <BBBBHIhhhhhHHHH layout");
        lp_unpack_telem(pl, &u);
        CHECK(telem_eq(&t, &u), "TELEM unpack(pack(t)) == t");
        CHECK(lp_encode_telem(LINK_VEHICLE_ID, 7, &t, fr) == LP_TELEM_FRAME_BYTES && fr[0] == 0xA5 && fr[1] == 0x5A &&
              fr[2] == 31 && fr[3] == LINK_VEHICLE_ID && fr[4] == LP_TYPE_TELEM && fr[5] == 7, "TELEM frame header");
        CHECK(lp_encode_frame(1, 2, 3, pl, LP_MAX_PAYLOAD_LEN + 1, fr) == 0, "oversize payload refused");
    }
    /* the reference's byte vectors: frames emitted and every counter, byte-exact */
    for (i = 0; i < LV_N_VECTORS; i++) {
        const lv_vector_t *v = &LV_VECTORS[i];
        lp_decoder_t dec;
        lp_frame_t out[LP_DEC_MAX_OUT], all[16];
        int n_all = 0, j;
        lp_decoder_init(&dec, v->vid, v->accept);
        for (j = 0; j < (int)v->n; j++) {
            uint8_t n = lp_decoder_feed(&dec, v->data[j], out, LP_DEC_MAX_OUT), m;
            for (m = 0; m < n && n_all < 16; m++) all[n_all++] = out[m];
        }
        CHECK(n_all == v->n_frames, "vector %s: %d frames (want %d)", v->name, n_all, v->n_frames);
        for (j = 0; j < n_all && j < v->n_frames; j++) {
            const lv_frame_t *e = &v->frames[j];
            CHECK(all[j].veh == e->veh && all[j].type == e->type && all[j].seq == e->seq && all[j].len == e->len &&
                  memcmp(all[j].payload, e->payload, e->len) == 0, "vector %s frame %d fields", v->name, j);
            if (e->kind == 2) {                   /* TELEM v1 */
                lp_telem_t t;
                long long f[LV_NF];
                int q;
                lp_unpack_telem(all[j].payload, &t);
                telem_fields(&t, f);
                for (q = 0; q < LV_NF; q++)
                    CHECK(f[q] == e->f[q], "vector %s telem field %d: %lld vs %lld", v->name, q, f[q], e->f[q]);
                CHECK(all[j].type == LP_TYPE_TELEM && all[j].len == LP_TELEM_PAYLOAD_LEN, "vector %s telem TYPE / LEN", v->name);
            } else if (e->kind == 1) {
                lp_cmd_t c;
                lp_unpack_cmd(all[j].payload, &c);
                CHECK(c.cmd == e->f[0] && c.arg == e->f[1] && c.nonce == e->f[2], "vector %s cmd fields", v->name);
            }
        }
        {
            const uint16_t got[7] = {dec.stats.frames, dec.stats.bad_len, dec.stats.bad_crc, dec.stats.wrong_vehicle,
                                     dec.stats.wrong_type, dec.stats.bad_payload_len, dec.stats.skipped_bytes};
            for (j = 0; j < 7; j++)
                CHECK(got[j] == v->stats[j], "vector %s counter %d: %u vs %ld", v->name, j, got[j], v->stats[j]);
            CHECK(dec.stats.overflow == 0, "vector %s: no decoder overflow", v->name);
        }
        if (v->reencode && n_all == 1) {      /* encode(decode(x)) == x */
            uint8_t fr[LP_FRAME_MAX_BYTES];
            uint8_t n = lp_encode_frame(all[0].veh, all[0].type, all[0].seq, all[0].payload, all[0].len, fr);
            CHECK(n == v->n && memcmp(fr, v->data, n) == 0, "vector %s re-encodes byte-exact", v->name);
        }
    }
}

/* ------------------------------------------------------------------------ */
/* FCU <-> COM ilink (SPEC 10.4, v0.6)                                      */
/* ------------------------------------------------------------------------ */
static uint32_t il_bits(float f)
{
    uint32_t u;
    memcpy(&u, &f, sizeof u);
    return u;
}

static float il_from_bits(long long b)
{
    const uint32_t u = (uint32_t)b;
    float f;
    memcpy(&f, &u, sizeof f);
    return f;
}

/* unpack a payload into the vector's field list (struct order, floats as bit patterns); returns the count */
static int il_fields(uint8_t type, const uint8_t *pl, long long f[ILV_NF])
{
    int i;
    if (type == IL_TYPE_CMD) {
        il_cmd_t c;
        il_unpack_cmd(pl, &c);
        f[0] = c.seq; f[1] = c.cmd.cmd; f[2] = c.cmd.arg; f[3] = c.cmd.nonce; f[4] = c.rssi_dbm; f[5] = c.snr_q4;
        return 6;
    }
    if (type == IL_TYPE_SNAP) {
        il_snap_t s;
        il_unpack_snap(pl, &s);
        f[0] = s.reply_seq; f[1] = s.state; f[2] = s.flags; f[3] = s.reason; f[4] = s.ack_seq; f[5] = s.nonce;
        f[6] = s.throttle_milli; f[7] = (long long)s.tick; f[8] = il_bits(s.h_m); f[9] = il_bits(s.vz_mps);
        f[10] = il_bits(s.tilt_deg); f[11] = il_bits(s.v_cell); f[12] = il_bits(s.loop_max_us);
        for (i = 0; i < 4; i++) f[13 + i] = il_bits(s.q[i]);
        return 17;
    }
    {
        il_stat_t s;
        il_unpack_stat(pl, &s);
        f[0] = (long long)s.com_ticks; f[1] = s.reset_cause; f[2] = s.flags; f[3] = s.radio_version; f[4] = s.rssi_dbm;
        f[5] = s.snr_q4; f[6] = s.params_tag; f[7] = s.dec_frames; f[8] = s.dec_bad_crc; f[9] = s.dec_bad_len;
        f[10] = s.dec_other; f[11] = s.cmd_forwarded; f[12] = s.radio_rx_errors; f[13] = s.radio_rx_overflow;
        f[14] = s.radio_tx_timeouts; f[15] = s.replies; f[16] = s.replies_dropped; f[17] = s.il_bad;
        f[18] = s.il_tx_dropped; f[19] = s.loop_us_max;
        return 20;
    }
}

/* the struct written field by field from a vector's field list, then packed */
static void il_pack_fields(uint8_t type, const long long *f, uint8_t *out)
{
    int i;
    if (type == IL_TYPE_CMD) {
        il_cmd_t c;
        c.seq = (uint8_t)f[0]; c.cmd.cmd = (uint8_t)f[1]; c.cmd.arg = (uint16_t)f[2]; c.cmd.nonce = (uint16_t)f[3];
        c.rssi_dbm = (int8_t)f[4]; c.snr_q4 = (int8_t)f[5];
        il_pack_cmd(&c, out);
    } else if (type == IL_TYPE_SNAP) {
        il_snap_t sn;
        il_snap_t *s = &sn;
        s->reply_seq = (uint8_t)f[0]; s->state = (uint8_t)f[1]; s->flags = (uint8_t)f[2]; s->reason = (uint8_t)f[3];
        s->ack_seq = (uint8_t)f[4]; s->nonce = (uint16_t)f[5]; s->throttle_milli = (uint16_t)f[6]; s->tick = (uint32_t)f[7];
        s->h_m = il_from_bits(f[8]); s->vz_mps = il_from_bits(f[9]); s->tilt_deg = il_from_bits(f[10]);
        s->v_cell = il_from_bits(f[11]); s->loop_max_us = il_from_bits(f[12]);
        for (i = 0; i < 4; i++) s->q[i] = il_from_bits(f[13 + i]);
        il_pack_snap(s, out);
    } else {
        il_stat_t s;
        s.com_ticks = (uint32_t)f[0]; s.reset_cause = (uint8_t)f[1]; s.flags = (uint8_t)f[2]; s.radio_version = (uint8_t)f[3];
        s.rssi_dbm = (int8_t)f[4]; s.snr_q4 = (int8_t)f[5]; s.params_tag = (uint8_t)f[6]; s.dec_frames = (uint16_t)f[7];
        s.dec_bad_crc = (uint16_t)f[8]; s.dec_bad_len = (uint16_t)f[9]; s.dec_other = (uint16_t)f[10];
        s.cmd_forwarded = (uint16_t)f[11]; s.radio_rx_errors = (uint16_t)f[12]; s.radio_rx_overflow = (uint16_t)f[13];
        s.radio_tx_timeouts = (uint16_t)f[14]; s.replies = (uint16_t)f[15]; s.replies_dropped = (uint16_t)f[16];
        s.il_bad = (uint16_t)f[17]; s.il_tx_dropped = (uint16_t)f[18]; s.loop_us_max = (uint16_t)f[19];
        il_pack_stat(&s, out);
    }
}

/* pack(unpack(payload)) through the C structs (n must be the type's length) */
static void il_repack(uint8_t type, const uint8_t *pl, uint8_t *out)
{
    if (type == IL_TYPE_CMD) {
        il_cmd_t c;
        il_unpack_cmd(pl, &c);
        il_pack_cmd(&c, out);
    } else if (type == IL_TYPE_SNAP) {
        il_snap_t s;
        il_unpack_snap(pl, &s);
        il_pack_snap(&s, out);
    } else {
        il_stat_t s;
        il_unpack_stat(pl, &s);
        il_pack_stat(&s, out);
    }
}

#define IL_PI_D 3.14159265358979323846
#define IL_RAD2DEG_D (180.0 / IL_PI_D)

static double wrap180(double d)
{
    while (d > 180.0) d -= 360.0;
    while (d < -180.0) d += 360.0;
    return d;
}

static void test_ilink_proto(void)
{
    static const uint8_t ascii[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    static il_decoder_t dec;
    int i, k;
    CHECK(IL_CMD_FRAME_BYTES == 14 && IL_SNAP_FRAME_BYTES == 55 && IL_STAT_FRAME_BYTES == 42 && IL_FRAME_MAX_BYTES == 55 &&
          IL_MAX_PAYLOAD == IL_SNAP_LEN && ILV_FRAME_MAX_BYTES == IL_FRAME_MAX_BYTES && ILV_ACCEPT_ALL == IL_ACCEPT_ALL &&
          IL_ACCEPT_ALL == 7, "ilink frame sizes 14 / 55 / 42, longest 55 (= the reference's), ALL = 7");
    CHECK(ILV_VEHICLE_ID == LINK_VEHICLE_ID && ILV_LOOP_HZ == FC_LOOP_HZ, "ilink vectors for vehicle %d at %d Hz, params %d at %d Hz",
          ILV_VEHICLE_ID, ILV_LOOP_HZ, LINK_VEHICLE_ID, FC_LOOP_HZ);
    CHECK(ILV_DEC_MAX_OUT == IL_DEC_MAX_OUT && IL_DEC_MAX_OUT == 1u, "vectors' DEC_MAX_OUT %d == IL_DEC_MAX_OUT %u == 1",
          ILV_DEC_MAX_OUT, IL_DEC_MAX_OUT);
    CHECK(ILV_ACCEPT_FCU == IL_ACCEPT_FCU && ILV_ACCEPT_COM == IL_ACCEPT_COM && ILV_NEED_NEVER == IL_NEED_NEVER &&
          IL_ACCEPT_FCU == (IL_ACCEPT_CMD | IL_ACCEPT_STAT) && IL_ACCEPT_COM == IL_ACCEPT_SNAP &&
          IL_ACCEPT_CMD == 1u << (IL_TYPE_CMD - 1u) && IL_ACCEPT_SNAP == 1u << (IL_TYPE_SNAP - 1u) &&
          IL_ACCEPT_STAT == 1u << (IL_TYPE_STAT - 1u), "accept masks: FCU = CMD | STAT, COM = SNAP, bit TYPE - 1");
    /* no replay buffer: n, the accept mask, the counters, the CRC, the candidate frame (67 B on the AVR) */
    CHECK(sizeof(dec) <= 1u + sizeof(il_dec_stats_t) + 1u + 2u + sizeof(il_frame_t) + 1u,
          "il_decoder_t is %u bytes (FCU RAM; phase 2: 73 on the AVR)", (unsigned)sizeof(dec));
    /* CRC: the byte-wise update equals the bitwise definition for every register value and byte, and the link's CRC */
    CHECK(il_crc16(ascii, 9, LP_CRC_INIT) == ILV_CRC_CHECK && ILV_CRC_CHECK == 0x29B1, "il_crc16(\"123456789\") = 0x%04X",
          il_crc16(ascii, 9, LP_CRC_INIT));
    {
        long bad = 0, c;
        for (c = 0; c < 65536L; c++) {
            for (k = 0; k < 256; k++) {
                uint16_t r = (uint16_t)(c ^ ((uint16_t)k << 8));      /* MSB first, poly 0x1021 */
                int bit;
                for (bit = 0; bit < 8; bit++)
                    r = (uint16_t)(r & 0x8000u ? (r << 1) ^ 0x1021u : r << 1);
                if (il_crc16_update((uint16_t)c, (uint8_t)k) != r) bad++;
            }
        }
        CHECK(bad == 0, "il_crc16_update == the bitwise CRC-16/CCITT update for all 65536 x 256 (register, byte): %ld differ",
              bad);
    }
    {
        long bad = 0;
        uint8_t b[255];
        for (k = 0; k < 3000; k++) {
            int n = (int)(urand() * 256.0), j;
            uint16_t init = k < 1000 ? (uint16_t)LP_CRC_INIT : (uint16_t)(urand() * 65536.0);
            if (n > 255) n = 255;
            for (j = 0; j < n; j++) b[j] = (uint8_t)(urand() * 256.0);
            if (il_crc16(b, (uint8_t)n, init) != lp_crc16(b, (uint8_t)n, init)) bad++;
        }
        for (k = 0; k < 256; k++) {
            uint8_t one = (uint8_t)k;
            static const uint16_t inits[] = {0x0000, 0xFFFF, 0x8000, 0x0001, 0x1021, 0xC55C};
            int j;
            for (j = 0; j < 6; j++)
                if (il_crc16(&one, 1, inits[j]) != lp_crc16(&one, 1, inits[j])) bad++;
        }
        CHECK(bad == 0, "il_crc16 == lp_crc16 on 3000 random buffers and every byte from 6 start values: %ld differ", bad);
    }
    {
        int bad = 0;
        for (k = 0; k < 256; k++) {
            const uint8_t want = k == IL_TYPE_CMD ? IL_CMD_LEN : (k == IL_TYPE_SNAP ? IL_SNAP_LEN : (k == IL_TYPE_STAT ? IL_STAT_LEN :
                                 0));
            if (il_payload_len((uint8_t)k) != want) bad++;
        }
        CHECK(bad == 0, "il_payload_len: CMD 8, SNAP 49, STAT 36, every other TYPE (the removed 4 / 5 too) 0 (%d wrong)", bad);
    }
    /* payload vectors: fields <-> bytes <-> frame, byte-exact (NaN / inf / -0 / denormals / sNaN included) */
    for (i = 0; i < ILV_N_PAYLOADS; i++) {
        const ilv_payload_t *v = &ILV_PAYLOADS[i];
        long long f[ILV_NF];
        uint8_t pl[IL_MAX_PAYLOAD], fr[IL_FRAME_MAX_BYTES];
        int nf, j;
        uint8_t n;
        CHECK(il_payload_len(v->type) == v->n, "payload %s: length %u", v->name, v->n);
        nf = il_fields(v->type, v->payload, f);
        CHECK(nf == v->n_fields, "payload %s: %d fields (want %d)", v->name, nf, v->n_fields);
        for (j = 0; j < nf && j < v->n_fields; j++)
            CHECK(f[j] == v->f[j], "payload %s field %d: unpack %lld vs %lld", v->name, j, f[j], v->f[j]);
        memset(pl, 0xEE, sizeof pl);
        il_pack_fields(v->type, v->f, pl);
        CHECK(memcmp(pl, v->payload, v->n) == 0, "payload %s: pack(fields) == reference bytes", v->name);
        memset(pl, 0xEE, sizeof pl);
        il_repack(v->type, v->payload, pl);
        CHECK(memcmp(pl, v->payload, v->n) == 0, "payload %s: pack(unpack(bytes)) byte-exact", v->name);
        n = il_encode(v->type, v->payload, v->n, fr);
        CHECK(n == v->frame_n && memcmp(fr, v->frame, n) == 0, "payload %s: il_encode == reference frame", v->name);
        CHECK(il_encode(v->type, v->payload, (uint8_t)(v->n - 1u), fr) == 0 &&
              il_encode(v->type, v->payload, (uint8_t)(v->n + 1u), fr) == 0, "payload %s: wrong LEN refused", v->name);
    }
    {
        uint8_t pl[IL_MAX_PAYLOAD] = {0}, fr[IL_FRAME_MAX_BYTES];
        CHECK(il_encode(0x00, pl, 8, fr) == 0 && il_encode(0x06, pl, 55, fr) == 0 && il_encode(0xFF, pl, 49, fr) == 0 &&
              il_encode(LP_TYPE_CMD, pl, 5, fr) == 0 && il_encode(0x04, pl, 49, fr) == 0 && il_encode(0x05, pl, 49, fr) == 0,
              "unknown TYPEs (the removed VTOL SNAP2 / SNAP3 = 4 / 5 among them) refused");
    }
    /* decoder vectors: every returned frame, every counter, the open candidate, need / cmd_need after every byte; the
     * zero-copy il_decoder_push on a second decoder returns the same frames (max_out >= 1) and the same counters */
    for (i = 0; i < ILV_N_STREAMS; i++) {
        static il_decoder_t dp;
        const ilv_stream_t *v = &ILV_STREAMS[i];
        il_frame_t out[2];
        int n_all = 0, j, bad_frames = 0, bad_m = 0, bad_need = 0, bad_cneed = 0, bad_push = 0;
        long frame_bytes = 0;
        il_decoder_init(&dec, v->accept);
        il_decoder_init(&dp, v->accept);
        for (j = 0; j < (int)v->n; j++) {
            const uint16_t ov = dec.stats.overflow;
            const uint8_t m = il_decoder_feed(&dec, v->data[j], out, v->max_out);
            const il_frame_t *pf = il_decoder_push(&dp, v->data[j]);
            if (m > 1u || m > v->max_out) bad_m++;
            if (il_decoder_need(&dec) != v->need[j]) bad_need++;
            if (il_decoder_cmd_need(&dec) != v->cmd_need[j]) bad_cneed++;
            if ((pf != 0) != (m == 1u || dec.stats.overflow != ov) ||
                (m == 1u && (pf->type != out[0].type || pf->len != out[0].len || memcmp(pf->payload, out[0].payload, pf->len))))
                bad_push++;
            if (pf != 0)
                frame_bytes += pf->len + IL_FRAME_OVERHEAD;
            if (m == 1u) {
                if (n_all < v->n_frames && (out[0].type != v->frames[n_all].type || out[0].len != v->frames[n_all].len ||
                                            memcmp(out[0].payload, v->frames[n_all].payload, out[0].len) != 0))
                    bad_frames++;
                n_all++;
            }
        }
        CHECK(n_all == v->n_frames && bad_frames == 0, "stream %s: %d frames (want %d), %d differ", v->name, n_all, v->n_frames,
              bad_frames);
        {
            const unsigned got[6] = {dec.stats.frames, dec.stats.bad_crc, dec.stats.bad_len, dec.stats.bad_type,
                                     dec.stats.skipped_bytes, dec.stats.overflow};
            for (j = 0; j < 6; j++)
                CHECK(got[j] == v->stats[j], "stream %s counter %d: %u vs %u", v->name, j, got[j], v->stats[j]);
        }
        CHECK(dec.n == v->pending && bad_m == 0, "stream %s: candidate %u bytes (want %u), at most one frame per byte",
              v->name, dec.n, v->pending);
        CHECK(bad_need == 0 && bad_cneed == 0, "stream %s: il_decoder_need differs after %d bytes, il_decoder_cmd_need after %d",
              v->name, bad_need, bad_cneed);
        CHECK(dec.accept == v->accept, "stream %s: accept mask %u kept (want %u)", v->name, dec.accept, v->accept);
        CHECK(bad_push == 0 && dp.n == dec.n && dp.stats.frames == dec.stats.frames && dp.stats.bad_crc == dec.stats.bad_crc &&
              dp.stats.bad_len == dec.stats.bad_len && dp.stats.bad_type == dec.stats.bad_type &&
              dp.stats.skipped_bytes == dec.stats.skipped_bytes && dp.stats.overflow == 0u,
              "stream %s: il_decoder_push == il_decoder_feed (%d bytes differ)", v->name, bad_push);
        CHECK((((long)v->n - (long)dec.stats.skipped_bytes - frame_bytes - (long)dec.n) & 0xFFFF) == 0,
              "stream %s: bytes = skipped + frame bytes + pending", v->name);
    }
    {   /* a frame right behind a corrupted one survives ANY single substituted, dropped or inserted byte in it (rule 4
         * with the CRC low byte checked on arrival); the corrupted frame itself is never returned wrong */
        static const uint8_t cmd_b[IL_CMD_LEN] = {9, LP_CMD_THROTTLE, 0xBC, 0x02, 0xEF, 0xBE, 0xA3, 0xF2};
        uint8_t pl[IL_MAX_PAYLOAD], a[3][IL_FRAME_MAX_BYTES], b[IL_CMD_FRAME_BYTES], s[2 * IL_FRAME_MAX_BYTES];
        static const uint8_t types[3] = {IL_TYPE_CMD, IL_TYPE_STAT, IL_TYPE_SNAP};
        long cases = 0, lost = 0, wrong = 0;
        uint8_t na[3], t;
        (void)il_encode(IL_TYPE_CMD, cmd_b, IL_CMD_LEN, b);
        for (t = 0; t < 3; t++) {
            uint8_t q;
            for (q = 0; q < IL_MAX_PAYLOAD; q++)
                pl[q] = (uint8_t)(q * 29u + 7u + t);                       /* no 0xC5 5C pair inside */
            na[t] = il_encode(types[t], pl, il_payload_len(types[t]), a[t]);
        }
        for (t = 0; t < 3; t++) {
            int op, j, val;
            for (op = 0; op < 3; op++) {                                   /* 0 substitute, 1 drop, 2 insert */
                for (j = 0; j < na[t] + (op == 2); j++) {
                    for (val = 0; val < (op == 1 ? 1 : 256); val++) {
                        int ns = 0, q, got_b = 0;
                        il_frame_t o[1];
                        if (op == 0 && val == a[t][j]) continue;
                        for (q = 0; q < na[t]; q++) {
                            if (q == j && op == 2) s[ns++] = (uint8_t)val;
                            if (q == j && op == 0) { s[ns++] = (uint8_t)val; continue; }
                            if (q == j && op == 1) continue;
                            s[ns++] = a[t][q];
                        }
                        if (op == 2 && j == na[t]) s[ns++] = (uint8_t)val;
                        memcpy(s + ns, b, IL_CMD_FRAME_BYTES);
                        ns += IL_CMD_FRAME_BYTES;
                        il_decoder_init(&dec, IL_ACCEPT_ALL);
                        for (q = 0; q < ns; q++) {
                            if (il_decoder_feed(&dec, s[q], o, 1) == 1u) {
                                if (o[0].type == IL_TYPE_CMD && o[0].len == IL_CMD_LEN && memcmp(o[0].payload, cmd_b, IL_CMD_LEN) == 0 &&
                                    q == ns - 1)
                                    got_b = 1;
                                else if (!(o[0].type == types[t] && o[0].len == na[t] - IL_FRAME_OVERHEAD &&
                                           memcmp(o[0].payload, a[t] + 4, o[0].len) == 0))
                                    wrong++;       /* not A itself (an insert before it): a CRC-16 false accept */
                            }
                        }
                        cases++;
                        if (!got_b) lost++;
                    }
                }
            }
        }
        CHECK(lost == 0, "single-byte errors (substituted / dropped / inserted) in CMD / STAT / SNAP frames: %ld cases, "
              "next CMD lost %ld", cases, lost);
        CHECK(wrong <= 3, "single-byte errors: %ld corrupted frames passed the CRC (expected well below 1)", wrong);
        printf("info: single-byte error sweep: %ld cases, next frame lost %ld times, CRC false accepts %ld\n", cases, lost,
               wrong);
    }
    {   /* counters wrap at 16 bits; max_out 0 returns nothing and counts every frame as overflow */
        static const uint8_t cmd[IL_CMD_LEN] = {1, LP_CMD_ARM, 0, 0, 0, 0, 127u /* RADIO_RSSI_NONE */, 0};
        uint8_t fr[IL_CMD_FRAME_BYTES];
        il_frame_t out[1];
        long j;
        il_decoder_init(&dec, IL_ACCEPT_ALL);
        for (j = 0; j < 70000L; j++)
            (void)il_decoder_feed(&dec, (uint8_t)(j & 1 ? 0x00 : 0x5C), out, 1);
        CHECK(dec.stats.skipped_bytes == (uint16_t)(70000L & 0xFFFF) && dec.n == 0, "skipped_bytes wraps (%u)",
              dec.stats.skipped_bytes);
        (void)il_encode(IL_TYPE_CMD, cmd, IL_CMD_LEN, fr);
        il_decoder_init(&dec, IL_ACCEPT_FCU);
        for (j = 0; j < (long)sizeof fr; j++)
            CHECK(il_decoder_feed(&dec, fr[j], out, 0) == 0, "max_out 0 returns nothing");
        CHECK(dec.stats.frames == 1 && dec.stats.overflow == 1, "max_out 0: frame counted as overflow");
    }
    {   /* direction filter: every (LEN, TYPE) header pair against every mask - a candidate passes the header iff the TYPE
         * is known, accepted and LEN is its length; bits outside IL_ACCEPT_ALL (the removed VTOL SNAP2 / SNAP3 bits 0x08 /
         * 0x10 among them) are ignored; the COM never has an IL_CMD to wait for, a mask without CMD reports IL_NEED_NEVER;
         * the removed LENs 55 / 62 and TYPEs 4 / 5 never pass */
        static const uint8_t masks[] = {0u, IL_ACCEPT_CMD, IL_ACCEPT_SNAP, IL_ACCEPT_STAT, IL_ACCEPT_FCU, IL_ACCEPT_COM,
                                        IL_ACCEPT_CMD | IL_ACCEPT_SNAP, IL_ACCEPT_ALL, 0xE0u | IL_ACCEPT_FCU, 0x08u, 0x10u,
                                        0x18u, IL_ACCEPT_SNAP | 0x08u, 0xE8u | IL_ACCEPT_FCU, 0x10u | IL_ACCEPT_COM, 0x1Fu, 0xFFu};
        int m, ln, ty, bad = 0;
        for (m = 0; m < (int)sizeof masks; m++) {
            const uint8_t a = (uint8_t)(masks[m] & IL_ACCEPT_ALL);
            for (ln = 0; ln < 256; ln++) {
                for (ty = 0; ty < 256; ty += (ln == IL_CMD_LEN || ln == IL_STAT_LEN || ln == IL_SNAP_LEN || ln == 55 ||
                                              ln == 62) ? 1 : 64) {
                    const uint8_t hdr[4] = {IL_SYNC0, IL_SYNC1, (uint8_t)ln, (uint8_t)ty};
                    const uint8_t want_len = il_payload_len((uint8_t)ty);
                    const int pass = want_len != 0u && want_len == ln && (a & (1u << (ty - 1)));
                    const int len_ok = (ln == IL_CMD_LEN && (a & IL_ACCEPT_CMD)) || (ln == IL_STAT_LEN && (a & IL_ACCEPT_STAT)) ||
                                       (ln == IL_SNAP_LEN && (a & IL_ACCEPT_SNAP));
                    int q;
                    il_decoder_init(&dec, masks[m]);
                    for (q = 0; q < 4; q++)
                        (void)il_decoder_push(&dec, hdr[q]);
                    if ((dec.n == 4u) != pass) bad++;
                    if (!len_ok && !(dec.stats.bad_len == 1u && dec.stats.bad_type == 0u)) bad++;
                    if (len_ok && !pass && !(dec.stats.bad_type + dec.stats.bad_len == 1u)) bad++;
                }
            }
            il_decoder_init(&dec, masks[m]);
            if (dec.accept != a) bad++;
            if (il_decoder_cmd_need(&dec) != ((a & IL_ACCEPT_CMD) ? IL_CMD_FRAME_BYTES : IL_NEED_NEVER)) bad++;
            (void)il_decoder_push(&dec, IL_SYNC0);
            if (il_decoder_need(&dec) != ((a & IL_ACCEPT_CMD) ? IL_CMD_FRAME_BYTES - 1u : (a & IL_ACCEPT_STAT) ?
                                          IL_STAT_FRAME_BYTES - 1u : (a & IL_ACCEPT_SNAP) ? IL_SNAP_FRAME_BYTES - 1u :
                                          IL_NEED_NEVER))
                bad++;
        }
        CHECK(bad == 0, "direction filter: header pass / bad_len / bad_type / need / cmd_need for %d masks: %d wrong",
              (int)sizeof masks, bad);
    }
    /* FCU snapshot -> COM telemetry source */
    {
        onb_tlm_src_t src, back;
        il_snap_t s;
        const float q[4] = {0.92387953f, 0.0f, 0.38268343f, 0.0f};          /* +45 deg about y */
        memset(&src, 0, sizeof src);
        src.state = ONB_POWERED; src.flags = 0x2F; src.reason = LP_R_I4_APOGEE_LIMIT; src.ack_seq = 7; src.nonce = 0xBEEF;
        src.throttle_milli = 812; src.tick = 123456UL; src.h_m = 87.5f; src.vz_mps = 21.25f; src.roll_deg = 99.0f;
        src.pitch_deg = -99.0f; src.yaw_deg = 33.0f; src.tilt_deg = 45.0f; src.v_cell = 3.95f; src.loop_max_us = 2900.0f;
        il_snap_from_tlm_src(42, &src, q, &s);
        CHECK(s.reply_seq == 42 && s.state == src.state && s.flags == src.flags && s.reason == src.reason &&
              s.ack_seq == src.ack_seq && s.nonce == src.nonce && s.throttle_milli == src.throttle_milli && s.tick == src.tick &&
              s.h_m == src.h_m && s.vz_mps == src.vz_mps && s.tilt_deg == src.tilt_deg && s.v_cell == src.v_cell &&
              s.loop_max_us == src.loop_max_us && s.q[0] == q[0] && s.q[1] == q[1] && s.q[2] == q[2] && s.q[3] == q[3],
              "il_snap_from_tlm_src copies every field and q");
        il_snap_to_tlm_src(&s, &back);
        CHECK(back.state == src.state && back.flags == src.flags && back.reason == src.reason && back.ack_seq == src.ack_seq &&
              back.nonce == src.nonce && back.throttle_milli == src.throttle_milli && back.tick == src.tick &&
              back.h_m == src.h_m && back.vz_mps == src.vz_mps && back.tilt_deg == src.tilt_deg && back.v_cell == src.v_cell &&
              back.loop_max_us == src.loop_max_us, "il_snap_to_tlm_src restores every field");
        CHECK(NEAR(back.roll_deg, 0.0, 1e-4) && NEAR(back.pitch_deg, 45.0, 1e-4) && NEAR(back.yaw_deg, 0.0, 1e-4),
              "q(+45 deg about y) -> roll %g pitch %g yaw %g", (double)back.roll_deg, (double)back.pitch_deg, (double)back.yaw_deg);
    }
    {   /* random attitudes (tilt < 80 deg): float32 angles vs a double evaluation of the same formulas on the same q */
        double worst = 0.0;
        for (k = 0; k < 20000; k++) {
            const double tl = 80.0 * urand() / IL_RAD2DEG_D, az = IL_PI_D * srand1(), yw = IL_PI_D * srand1();
            const double tw = cos(tl / 2), tx = cos(az) * sin(tl / 2), ty = sin(az) * sin(tl / 2);
            const double yc = cos(yw / 2), ys = sin(yw / 2);
            il_snap_t s;
            onb_tlm_src_t src;
            double w, x, y, z, r, p, ya, sp;
            memset(&s, 0, sizeof s);
            s.q[0] = (float)(yc * tw);
            s.q[1] = (float)(yc * tx - ys * ty);
            s.q[2] = (float)(yc * ty + ys * tx);
            s.q[3] = (float)(ys * tw);
            il_snap_to_tlm_src(&s, &src);
            w = s.q[0]; x = s.q[1]; y = s.q[2]; z = s.q[3];
            r = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)) * IL_RAD2DEG_D;
            sp = 2.0 * (w * y - z * x);
            p = asin(sp < -1.0 ? -1.0 : (sp > 1.0 ? 1.0 : sp)) * IL_RAD2DEG_D;
            ya = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)) * IL_RAD2DEG_D;
            worst = fmax(worst, fabs(wrap180(src.roll_deg - r)));
            worst = fmax(worst, fabs(wrap180(src.pitch_deg - p)));
            worst = fmax(worst, fabs(wrap180(src.yaw_deg - ya)));
        }
        CHECK(worst < 2e-4, "float32 roll / pitch / yaw within 2e-4 deg of double on 20000 attitudes (worst %.3g deg)", worst);
    }
    /* the COM pipeline SNAP -> TELEM frame against the reference: angles within one quantum, every other byte equal */
    for (i = 0; i < ILV_N_TELEM; i++) {
        const ilv_telem_t *v = &ILV_TELEM[i];
        onb_tlm_src_t src;
        uint8_t fr[LP_FRAME_MAX_BYTES], n, seq;
        int j, a, other = 0, ang = 0;
        il_snap_t s;
        if (v->snap_n != IL_SNAP_LEN)
            continue;                                  /* the rocket's COM: SNAP -> TELEM only */
        memset(&src, 0, sizeof src);
        il_unpack_snap(v->snap, &s);
        il_snap_to_tlm_src(&s, &src);
        seq = s.reply_seq;
        n = onboard_telemetry_frame(&src, ILV_LOOP_HZ, ILV_VEHICLE_ID, seq, fr);
        CHECK(n == v->telem_n && n == LP_TELEM_FRAME_BYTES, "telem %s: frame length %u (want %u)", v->name, n, v->telem_n);
        for (j = 0; j < n - 2; j++)
            if ((j < 20 || j >= 26) && fr[j] != v->telem[j]) other++;
        for (a = 0; a < 3; a++) {
            const int c = (int16_t)(fr[20 + 2 * a] | (fr[21 + 2 * a] << 8));
            const int py = (int16_t)(v->telem[20 + 2 * a] | (v->telem[21 + 2 * a] << 8));
            CHECK(abs(c - py) <= 1, "telem %s angle %d: C %d vs reference %d (> 1 quantum)", v->name, a, c, py);
            if (c != py) ang++;
        }
        CHECK(other == 0, "telem %s: %d non-angle bytes differ", v->name, other);
        CHECK(lp_crc16(fr + 2, (uint8_t)(n - 4u), LP_CRC_INIT) == (uint16_t)(fr[n - 2] | (fr[n - 1] << 8)), "telem %s: CRC valid",
              v->name);
        if (ang == 0)
            CHECK(memcmp(fr, v->telem, n) == 0, "telem %s: frame byte-exact", v->name);
    }
}

/* ------------------------------------------------------------------------ */
/* NAV <-> FCU nlink (SPEC 10.5, v0.6.3)                                    */
/* ------------------------------------------------------------------------ */
/* nl_nav_t from the vector fields: 14 float bit patterns in wire order, then the 6 integers */
static void nl_nav_from_fields(nl_nav_t *n, uint8_t seq, const uint32_t fb[14], const long iv[6])
{
    float f[14];
    int i;
    memcpy(f, fb, sizeof f);
    n->seq = seq;
    for (i = 0; i < 4; i++) n->q[i] = f[i];
    for (i = 0; i < 3; i++) {
        n->rate[i] = f[4 + i];
        n->accel[i] = f[7 + i];
    }
    n->h_m = f[10];
    n->vz_mps = f[11];
    n->h_baro_rel_m = f[12];
    n->tilt_deg = f[13];
    n->imu_fail = (uint16_t)iv[0];
    n->baro_fails = (uint16_t)iv[1];
    n->twi_errors = (uint16_t)iv[2];
    n->loop_us_max = (uint16_t)iv[3];
    n->flags = (uint8_t)iv[4];
    n->reset_cause = (uint8_t)iv[5];
}

/* the struct's fields as the vector form (float bit patterns + integers) */
static void nl_nav_to_fields(const nl_nav_t *n, uint32_t fb[14], long iv[6])
{
    float f[14];
    int i;
    for (i = 0; i < 4; i++) f[i] = n->q[i];
    for (i = 0; i < 3; i++) {
        f[4 + i] = n->rate[i];
        f[7 + i] = n->accel[i];
    }
    f[10] = n->h_m;
    f[11] = n->vz_mps;
    f[12] = n->h_baro_rel_m;
    f[13] = n->tilt_deg;
    memcpy(fb, f, sizeof f);
    iv[0] = n->imu_fail;
    iv[1] = n->baro_fails;
    iv[2] = n->twi_errors;
    iv[3] = n->loop_us_max;
    iv[4] = n->flags;
    iv[5] = n->reset_cause;
}

static int nl_nav_matches(const nl_nav_t *n, uint8_t seq, const uint32_t fb[14], const long iv[6])
{
    uint32_t gb[14];
    long gi[6];
    nl_nav_to_fields(n, gb, gi);
    return n->seq == seq && memcmp(gb, fb, sizeof gb) == 0 && memcmp(gi, iv, sizeof gi) == 0;
}

static void test_nlink_proto(void)
{
    static const uint8_t flags[7] = {NL_F_IMU_OK, NL_F_BARO_OK, NL_F_ALIGNED, NL_F_GYRO_SAT, NL_F_EST_VALID, NL_F_BARO_NEW,
                                     NL_F_BOOT};
    static const long want_flags[7] = NLV_FLAGS;
    static const uint8_t ascii[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    nl_nav_t n, sentinel;
    nl_fcu_t f, fsent;
    uint8_t buf[NL_XFER_BYTES];
    int i, j, k, bad;
    CHECK(NL_SYNC0 == NLV_SYNC0 && NL_SYNC1 == NLV_SYNC1 && NL_TYPE_NAV == NLV_TYPE_NAV && NL_TYPE_FCU == NLV_TYPE_FCU,
          "nlink sync 0x%02X 0x%02X, TYPEs %u / %u == the reference", NL_SYNC0, NL_SYNC1, NL_TYPE_NAV, NL_TYPE_FCU);
    CHECK(NL_NAV_LEN == NLV_NAV_LEN && NL_NAV_LEN == 66u && NL_XFER_BYTES == NLV_XFER_BYTES && NL_XFER_BYTES == 72u &&
          NL_FCU_FRAME_BYTES == NLV_FCU_FRAME_BYTES && NL_FCU_FRAME_BYTES == 10u && NL_HEADER_BYTES == 4u && NL_FCU_LEN == 4u,
          "nlink sizes: payload 66, exchange 72, FCU frame 10");
    for (bad = 0, i = 0; i < 7; i++)
        if (flags[i] != want_flags[i] || flags[i] != (1u << i)) bad++;
    CHECK(bad == 0 && NL_FF_BENCH == NLV_FF_BENCH && NL_FF_RECOVERY == NLV_FF_RECOVERY, "nlink flag bits == the reference");
    CHECK(il_crc16(ascii, 9, LP_CRC_INIT) == NLV_CRC_CHECK, "nlink CRC check 0x%04X", NLV_CRC_CHECK);
    memset(&sentinel, 0xA7, sizeof sentinel);
    memset(&fsent, 0xA7, sizeof fsent);
    /* NAV vectors: fields -> 72 bytes byte-exact (NaN payloads, sNaN, inf, -0, denormals), bytes -> the same fields */
    for (i = 0; i < NLV_N_NAVS; i++) {
        const nlv_nav_t *v = &NLV_NAVS[i];
        uint8_t r;
        nl_nav_from_fields(&n, v->seq, v->fb, v->iv);
        memset(buf, 0xEE, sizeof buf);
        r = nl_encode_nav(&n, buf);
        CHECK(r == NL_XFER_BYTES && memcmp(buf, v->frame, NL_XFER_BYTES) == 0, "nav %s: nl_encode_nav == reference frame", v->name);
        CHECK(memcmp(buf + NL_HEADER_BYTES, v->payload, NL_NAV_LEN) == 0, "nav %s: payload bytes", v->name);
        n = sentinel;
        CHECK(nl_decode_nav(v->frame, &n) == 1u && nl_nav_matches(&n, v->seq, v->fb, v->iv), "nav %s: decode -> fields", v->name);
        f = fsent;
        CHECK(nl_decode_fcu(v->frame, &f) == 0u && memcmp(&f, &fsent, sizeof f) == 0, "nav %s: not an FCU frame", v->name);
    }
    /* FCU vectors: fields -> 10 bytes + 62 zeros, bytes -> fields */
    for (i = 0; i < NLV_N_FCUS; i++) {
        const nlv_fcu_t *v = &NLV_FCUS[i];
        f.seq = v->f[0];
        f.state = v->f[1];
        f.fflags = v->f[2];
        f.params_tag = v->f[3];
        f.thr_q = v->f[4];
        memset(buf, 0xEE, sizeof buf);
        CHECK(nl_encode_fcu(&f, buf) == NL_FCU_FRAME_BYTES && memcmp(buf, v->frame, NL_XFER_BYTES) == 0,
              "fcu %s: nl_encode_fcu == reference (10-byte frame, 62 zeros)", v->name);
        f = fsent;
        CHECK(nl_decode_fcu(v->frame, &f) == 1u && f.seq == v->f[0] && f.state == v->f[1] && f.fflags == v->f[2] &&
              f.params_tag == v->f[3] && f.thr_q == v->f[4], "fcu %s: decode -> fields (thr_q %u)", v->name, v->f[4]);
        n = sentinel;
        CHECK(nl_decode_nav(v->frame, &n) == 0u && memcmp(&n, &sentinel, sizeof n) == 0, "fcu %s: not a NAV frame", v->name);
    }
    /* decode cases: both decoders on every buffer; a refused buffer leaves the struct untouched */
    for (i = 0; i < NLV_N_DECODES; i++) {
        const nlv_dec_t *v = &NLV_DECODES[i];
        uint8_t rn, rf;
        n = sentinel;
        f = fsent;
        rn = nl_decode_nav(v->buf, &n);
        rf = nl_decode_fcu(v->buf, &f);
        CHECK(rn == v->nav_valid && rf == v->fcu_valid, "decode %s: nav %u fcu %u (want %u %u)", v->name, rn, rf, v->nav_valid,
              v->fcu_valid);
        if (rn)
            CHECK(nl_nav_matches(&n, v->seq, v->fb, v->iv), "decode %s: NAV fields", v->name);
        else
            CHECK(memcmp(&n, &sentinel, sizeof n) == 0, "decode %s: refused NAV leaves the struct untouched", v->name);
        if (rf)
            CHECK(f.seq == v->fcu[0] && f.state == v->fcu[1] && f.fflags == v->fcu[2] && f.params_tag == v->fcu[3] &&
                  f.thr_q == v->fcu[4], "decode %s: FCU fields", v->name);
        else
            CHECK(memcmp(&f, &fsent, sizeof f) == 0, "decode %s: refused FCU leaves the struct untouched", v->name);
    }
    /* every single-byte substitution of a NAV frame (72 x 255) and of an FCU frame's 10 bytes is refused (CRC-16 detects
     * every error burst <= 16 bits); bytes 10..71 of the FCU half never matter */
    {
        const nlv_nav_t *v = &NLV_NAVS[3];
        const nlv_fcu_t *w = &NLV_FCUS[2];
        long cases = 0, accepted = 0, tail_refused = 0;
        for (j = 0; j < (int)NL_XFER_BYTES; j++) {
            for (k = 0; k < 256; k++) {
                memcpy(buf, v->frame, NL_XFER_BYTES);
                if (k == buf[j]) continue;
                buf[j] = (uint8_t)k;
                cases++;
                if (nl_decode_nav(buf, &n)) accepted++;
                memcpy(buf, w->frame, NL_XFER_BYTES);
                if (k == buf[j]) continue;
                buf[j] = (uint8_t)k;
                if (j < (int)NL_FCU_FRAME_BYTES) {
                    cases++;
                    if (nl_decode_fcu(buf, &f)) accepted++;
                } else if (!nl_decode_fcu(buf, &f)) {
                    tail_refused++;
                }
            }
        }
        CHECK(accepted == 0 && tail_refused == 0, "single-byte substitutions: %ld cases, %ld accepted; FCU tail changes "
              "refused %ld", cases, accepted, tail_refused);
    }
}

/* ------------------------------------------------------------------------ */
/* onboard supervisor (SPEC 8.8 tables)                                     */
/* ------------------------------------------------------------------------ */
typedef struct {
    onboard_t s;
    onb_outputs_t o;
    onb_params_t p;
    uint32_t k;
    uint8_t seq;
    int last_code;                 /* code of the last processed command frame, -1 none */
    float h, vz, tilt, vcell, gyro[3], roll, pitch, yaw, loopmax;
    bool gsat, imu_ok, baro_ok, aligned, arm;
} oh_t;

static void oh_init_p(oh_t *h, const onb_params_t *p, uint16_t seed, bool reset_in_flight)
{
    memset(h, 0, sizeof *h);
    h->p = *p;
    onboard_init(&h->s, p, seed, reset_in_flight);
    h->tilt = 0.5f;
    h->vcell = 4.1f;
    h->imu_ok = h->baro_ok = h->aligned = h->arm = true;
    h->last_code = -1;
}

/* fc_default_onboard in the given build (params.json mission.mode is only the default) */
static void onb_params_mode(onb_params_t *p, uint8_t mode)
{
    fc_default_onboard(p);
    p->mode = mode;
}

static void oh_init(oh_t *h)
{
    onb_params_t p;
    onb_params_mode(&p, LP_MODE_BALLISTIC);
    oh_init_p(h, &p, 1, false);
}

static const onb_outputs_t *oh_tick_frames(oh_t *h, const onb_frame_t *f, uint8_t nf)
{
    onb_inputs_t in;
    uint8_t i;
    in.tick = h->k;
    in.frames = f;
    in.n_frames = nf;
    in.h_m = h->h;
    in.vz_mps = h->vz;
    in.tilt_deg = h->tilt;
    for (i = 0; i < 3; i++) in.gyro[i] = h->gyro[i];
    in.gyro_saturated = h->gsat;
    in.v_cell = h->vcell;
    in.imu_ok = h->imu_ok;
    in.baro_ok = h->baro_ok;
    in.aligned = h->aligned;
    in.arm_switch = h->arm;
    in.roll_deg = h->roll;
    in.pitch_deg = h->pitch;
    in.yaw_deg = h->yaw;
    in.loop_max_us = h->loopmax;
    onboard_update(&h->s, &in, &h->o);
    for (i = 0; i < h->o.n_exec; i++)
        if (h->o.exec[i].code < ONB_EXEC_NONE) h->last_code = h->o.exec[i].code;
    h->k++;
    return &h->o;
}

static const onb_outputs_t *oh_tick(oh_t *h) { return oh_tick_frames(h, 0, 0); }

static const onb_outputs_t *oh_frame(oh_t *h, uint8_t seq, uint8_t cmd, uint16_t arg, uint16_t nonce)
{
    onb_frame_t f;
    f.seq = seq;
    f.cmd.cmd = cmd;
    f.cmd.arg = arg;
    f.cmd.nonce = nonce;
    return oh_tick_frames(h, &f, 1);
}

static const onb_outputs_t *oh_send(oh_t *h, uint8_t cmd, uint16_t arg)
{
    h->seq++;
    return oh_frame(h, h->seq, cmd, arg, h->s.nonce);
}

static const onb_outputs_t *oh_hb(oh_t *h) { return oh_frame(h, h->seq, LP_CMD_NONE, 0, 0); }

static const onb_outputs_t *oh_run(oh_t *h, uint32_t n, uint32_t hb_every)
{
    uint32_t i;
    for (i = 0; i < n; i++) {
        if (hb_every && i % hb_every == 0) oh_hb(h);
        else oh_tick(h);
    }
    return &h->o;
}

/* SAFE, ARMED, POWERED, UNPOWERED, RELEASED, ABORT, ARMED+pending, UNPOWERED+pending */
static void oh_to(oh_t *h, const char *st)
{
    oh_hb(h);
    oh_tick(h);
    if (!strcmp(st, "SAFE")) return;
    CHECK(oh_send(h, LP_CMD_ARM, 0)->state == ONB_ARMED, "setup ARM (%s)", st);
    if (!strncmp(st, "ARMED", 5)) {
        if (strstr(st, "pending")) oh_send(h, LP_CMD_RELEASE, 0);
        return;
    }
    CHECK(oh_send(h, LP_CMD_LAUNCH, 0)->state == ONB_POWERED, "setup LAUNCH (%s)", st);
    if (!strcmp(st, "POWERED")) return;
    if (!strncmp(st, "UNPOWERED", 9)) {
        CHECK(oh_send(h, LP_CMD_CUT, 0)->state == ONB_UNPOWERED, "setup CUT");
        if (strstr(st, "pending")) oh_send(h, LP_CMD_RELEASE, 0);
        return;
    }
    if (!strcmp(st, "ABORT")) {
        CHECK(oh_send(h, LP_CMD_KILL, 0)->state == ONB_ABORT, "setup KILL");
        return;
    }
    oh_send(h, LP_CMD_RELEASE, 0);                     /* RELEASED */
    oh_run(h, h->p.release_after_cut_ticks + 1, 20);
    CHECK(h->s.state == ONB_RELEASED, "setup RELEASED");
}

static void test_onboard_table(void)
{
    /* SPEC 8.8 command table: start state x command -> (state after, code) */
    enum { S_ = ONB_SAFE, A_ = ONB_ARMED, P_ = ONB_POWERED, U_ = ONB_UNPOWERED, R_ = ONB_RELEASED, AB_ = ONB_ABORT };
    static const struct { uint8_t cmd; uint16_t arg; bool good_nonce; const char *name; } V[11] = {
        {LP_CMD_ARM, 0, true, "ARM"}, {LP_CMD_DISARM, 0, true, "DISARM"}, {LP_CMD_LAUNCH, 0, true, "LAUNCH"},
        {LP_CMD_LAUNCH, 0, false, "LAUNCH_badnonce"}, {LP_CMD_THROTTLE, 500, true, "THROTTLE_500"},
        {LP_CMD_THROTTLE, 500, false, "THROTTLE_500_badnonce"}, {LP_CMD_THROTTLE, 0, true, "THROTTLE_0"},
        {LP_CMD_CUT, 0, true, "CUT"}, {LP_CMD_RELEASE, 0, true, "RELEASE"}, {LP_CMD_KILL, 0, true, "KILL"},
        {9, 0, true, "UNKNOWN_9"}};
    static const char *starts[8] = {"SAFE", "ARMED", "POWERED", "UNPOWERED", "RELEASED", "ABORT", "ARMED+pending",
                                    "UNPOWERED+pending"};
    const uint8_t RS = LP_REJ_STATE, RN = LP_REJ_NONCE, RP = LP_REJ_RELEASE_PENDING, RU = LP_REJ_UNKNOWN_CMD;
    static uint8_t want_state[8][11], want_code[8][11];
    int a, c;
    const uint8_t t_safe[11][2] = {{A_, 0}, {S_, RS}, {S_, RS}, {S_, RS}, {S_, RS}, {S_, RS}, {S_, RS}, {S_, RS}, {S_, RS},
                                   {S_, RS}, {S_, RU}};
    const uint8_t t_armed[11][2] = {{A_, RS}, {S_, 0}, {P_, 0}, {A_, RN}, {P_, 0}, {A_, RN}, {A_, LP_REJ_THROTTLE_ZERO},
                                    {A_, RS}, {A_, 0}, {S_, 0}, {A_, RU}};
    const uint8_t t_pow[11][2] = {{P_, RS}, {P_, RS}, {P_, RS}, {P_, RS}, {P_, 0}, {P_, 0}, {U_, 0}, {U_, 0}, {U_, 0},
                                  {AB_, 0}, {P_, RU}};
    const uint8_t t_unp[11][2] = {{U_, RS}, {U_, RS}, {U_, RS}, {U_, RS}, {P_, 0}, {P_, 0}, {U_, 0}, {U_, RS}, {U_, 0},
                                  {AB_, 0}, {U_, RU}};
    for (c = 0; c < 11; c++) {
        want_state[0][c] = t_safe[c][0]; want_code[0][c] = t_safe[c][1];
        want_state[1][c] = t_armed[c][0]; want_code[1][c] = t_armed[c][1];
        want_state[2][c] = t_pow[c][0]; want_code[2][c] = t_pow[c][1];
        want_state[3][c] = t_unp[c][0]; want_code[3][c] = t_unp[c][1];
        want_state[4][c] = R_; want_code[4][c] = RS;               /* RELEASED: only KILL (no-op) */
        want_state[5][c] = AB_; want_code[5][c] = RP;              /* ABORT: release pending */
        want_state[6][c] = A_; want_code[6][c] = RP;
        want_state[7][c] = U_; want_code[7][c] = RP;
    }
    want_code[4][9] = 0; want_code[5][9] = 0;                      /* KILL in RELEASED / ABORT: accepted, no change */
    want_state[6][9] = S_; want_code[6][9] = 0;                    /* KILL on the pad cancels the pending release */
    want_state[7][9] = AB_; want_code[7][9] = 0;
    for (a = 4; a < 8; a++) want_code[a][10] = RU;                 /* unknown command code */
    for (a = 0; a < 8; a++) {
        for (c = 0; c < 11; c++) {
            oh_t h;
            const onb_outputs_t *o;
            uint16_t n0;
            bool pend0;
            uint32_t d0;
            lp_telem_t t;
            oh_init(&h);
            oh_to(&h, starts[a]);
            n0 = h.s.nonce;
            pend0 = h.s.has_release_deadline;
            d0 = h.s.release_deadline;
            h.seq++;
            o = oh_frame(&h, h.seq, V[c].cmd, V[c].arg, V[c].good_nonce ? n0 : (uint16_t)(n0 ^ 0x5A5Au));
            CHECK(o->state == want_state[a][c] && h.last_code == want_code[a][c], "%s + %s -> %s code 0x%02X (want %s 0x%02X)",
                  starts[a], V[c].name, state_name(o->state), h.last_code, state_name(want_state[a][c]), want_code[a][c]);
            CHECK(o->reply && o->reply_seq == h.seq && o->ack_seq == h.seq, "%s + %s acknowledged", starts[a], V[c].name);
            onboard_telemetry(&o->tlm, FC_LOOP_HZ, &t);
            CHECK(((t.flags & LP_FLAG_REJECTED) != 0) == (want_code[a][c] != 0), "%s + %s REJECTED flag", starts[a], V[c].name);
            if (want_code[a][c])
                CHECK(t.reason == want_code[a][c], "%s + %s reason", starts[a], V[c].name);
            {
                const bool moved = a == 1 && want_code[a][c] == 0 && (V[c].cmd == LP_CMD_LAUNCH || V[c].cmd == LP_CMD_THROTTLE);
                CHECK(h.s.nonce == (moved ? onb_lfsr16_next(n0) : n0), "%s + %s nonce", starts[a], V[c].name);
            }
            if (a == 7 && c == 9)
                CHECK(pend0 && h.s.has_release_deadline && h.s.release_deadline == d0, "KILL keeps the earlier release time");
            if (c == 4 && want_state[a][c] == ONB_POWERED)
                CHECK(h.s.target_milli == 500, "THROTTLE 500 target");
        }
    }
}

static void test_onboard_rules(void)
{
    oh_t h;
    const onb_outputs_t *o;
    onb_params_t p;
    uint32_t i, last;
    int k;
    onb_params_mode(&p, LP_MODE_BALLISTIC);
    /* LFSR: maximal length, 0 -> 0xACE1 */
    {
        uint16_t s = 1;
        long n = 0;
        do {
            s = onb_lfsr16_next(s);
            n++;
        } while (s != 1 && n < 70000L && s != 0);
        CHECK(n == 65535L, "LFSR period %ld", n);
        CHECK(onb_lfsr16_next(0) == onb_lfsr16_next(0xACE1u), "seed 0 -> 0xACE1");
    }
    /* sequence rules */
    oh_init(&h);
    o = oh_frame(&h, 200, LP_CMD_ARM, 0, 0);
    CHECK(o->state == ONB_SAFE && h.last_code == LP_REJ_LINK && o->ack_seq == 200 && (o->flags & LP_FLAG_REJECTED),
          "first frame after boot is new but cannot arm (no earlier frame)");
    h.seq = 200;
    o = oh_hb(&h);
    CHECK(o->reply && o->reply_seq == 200 && o->n_exec == 1 && o->exec[0].code == ONB_EXEC_REPEAT, "duplicate SEQ: reply only");
    o = oh_frame(&h, 201, LP_CMD_NONE, 0, 0);
    CHECK(o->ack_seq == 201 && !(o->flags & LP_FLAG_REJECTED) && o->exec[0].code == ONB_EXEC_NONE, "newer NONE adopts the SEQ");
    h.seq = 201;
    CHECK(oh_send(&h, LP_CMD_ARM, 0)->state == ONB_ARMED, "ARM seq 202");
    CHECK(oh_frame(&h, 202, LP_CMD_DISARM, 0, 0)->state == ONB_ARMED, "repeat of 202 not executed again");
    CHECK(oh_frame(&h, 150, LP_CMD_DISARM, 0, 0)->state == ONB_ARMED, "older SEQ ignored");
    h.s.last_seq = 250;
    CHECK(oh_frame(&h, 3, LP_CMD_DISARM, 0, 0)->state == ONB_SAFE, "wrap-around: 3 is newer than 250");
    o = oh_tick(&h);
    CHECK(!o->reply, "no frame -> no reply");
    /* ARM gates */
    {
        static const char *nm[] = {"arm switch", "imu", "baro", "aligned", "tilt == max", "vcell < min", "vcell == min",
                                   "tilt just below"};
        for (k = 0; k < 8; k++) {
            int want;
            oh_init(&h);
            oh_hb(&h);
            if (k == 0) h.arm = false;
            if (k == 1) h.imu_ok = false;
            if (k == 2) h.baro_ok = false;
            if (k == 3) h.aligned = false;
            if (k == 4) h.tilt = p.arm_tilt_max_deg;
            if (k == 5) h.vcell = nextafterf(p.min_cell_v, 0.0f);
            if (k == 6) h.vcell = p.min_cell_v;
            if (k == 7) h.tilt = nextafterf(p.arm_tilt_max_deg, 0.0f);
            o = oh_send(&h, LP_CMD_ARM, 0);
            want = k == 0 ? LP_REJ_ARM_SWITCH : k <= 2 ? LP_REJ_SENSORS : k == 3 ? LP_REJ_NOT_ALIGNED : k == 4 ? LP_REJ_TILT
                 : k == 5 ? LP_REJ_VCELL : 0;
            CHECK(h.last_code == want && o->state == (want ? ONB_SAFE : ONB_ARMED), "ARM gate %s: code 0x%02X", nm[k], h.last_code);
        }
        oh_init(&h);
        oh_hb(&h);
        oh_run(&h, p.link_disarm_ticks, 0);
        oh_send(&h, LP_CMD_ARM, 0);
        CHECK(h.last_code == LP_REJ_LINK, "ARM after a silence >= link_loss_disarm_s");
        oh_send(&h, LP_CMD_ARM, 0);
        CHECK(h.last_code == 0 && h.s.state == ONB_ARMED, "the previous frame is now recent");
        oh_init_p(&h, &p, 1, true);                   /* I10 */
        o = oh_hb(&h);
        CHECK(o->state == ONB_SAFE && o->release && o->reason == LP_R_I10_RESET && (o->flags & LP_FLAG_INTERLOCK) &&
              (o->flags & LP_FLAG_RELEASED), "I10 boot: SAFE, release open, interlock flag");
        oh_send(&h, LP_CMD_ARM, 0);
        CHECK(h.last_code == LP_REJ_RELEASED, "no arming after an in-flight reset");
    }
    /* throttle slew: 0 -> climb in ramp_ticks, decreases immediate, clamp to climb */
    oh_init(&h);
    oh_to(&h, "ARMED");
    {
        uint32_t want;
        o = oh_send(&h, LP_CMD_LAUNCH, 0);
        for (i = 0; i < p.ramp_ticks + 3; i++) {
            want = (i + 1) * 1000u / p.ramp_ticks;
            if (want > p.climb_milli) want = p.climb_milli;
            CHECK(o->throttle_milli == want, "ramp tick %lu: %u vs %lu", (unsigned long)i, o->throttle_milli, (unsigned long)want);
            o = oh_tick(&h);
        }
        CHECK(oh_send(&h, LP_CMD_THROTTLE, 400)->throttle_milli == 400 || p.climb_milli < 400, "decrease immediate");
        want = ((400u * p.ramp_ticks + 999u) / 1000u + 1u) * 1000u / p.ramp_ticks;
        CHECK(oh_send(&h, LP_CMD_THROTTLE, 1000)->throttle_milli == (want < p.climb_milli ? want : p.climb_milli),
              "increase slews from 400");
        CHECK(oh_send(&h, LP_CMD_CUT, 0)->throttle_milli == 0, "CUT -> 0");
        CHECK(oh_send(&h, LP_CMD_THROTTLE, 60000)->state == ONB_POWERED && h.s.target_milli == p.climb_milli,
              "THROTTLE clamps to climb_throttle and relights from UNPOWERED");
    }
    /* coast hold */
    oh_init(&h);
    oh_to(&h, "POWERED");
    CHECK(h.o.control_active, "POWERED: control on");
    o = oh_send(&h, LP_CMD_CUT, 0);
    CHECK(o->state == ONB_UNPOWERED && o->control_active == (p.coast_hold_ticks > 0), "CUT: coast hold starts");
    for (i = 1; i < p.coast_hold_ticks + 2; i++) {
        o = (i % 50) ? oh_tick(&h) : oh_hb(&h);
        CHECK(o->control_active == (i < p.coast_hold_ticks), "coast hold tick %lu", (unsigned long)i);
    }
    /* link loss: I1p on the pad, I1 in flight, then release after abort_release_ticks */
    {
        static const char *st[3] = {"ARMED", "POWERED", "UNPOWERED"};
        for (k = 0; k < 3; k++) {
            uint32_t n = k == 0 ? p.link_disarm_ticks : p.link_abort_ticks;
            oh_init(&h);
            oh_to(&h, st[k]);
            last = h.k - 1;
            oh_run(&h, n - 1 - (h.k - 1 - last), 0);
            CHECK(!strcmp(state_name(h.s.state), st[k]), "%s held one tick before the link timeout", st[k]);
            o = oh_tick(&h);
            CHECK(h.k - 1 - last == n && o->state == (k == 0 ? ONB_SAFE : ONB_ABORT) &&
                  o->reason == (k == 0 ? LP_R_I1P_LINK_LOSS_PAD : LP_R_I1_LINK_LOSS) && (o->flags & LP_FLAG_INTERLOCK),
                  "%s: link-loss interlock exactly at %lu ticks", st[k], (unsigned long)n);
            if (k) {
                CHECK(o->throttle_milli == 0, "ABORT: motor off now");
                oh_run(&h, p.abort_release_ticks - 1, 0);
                CHECK(h.s.state == ONB_ABORT && !h.o.release, "release not before abort_release_ticks");
                o = oh_tick(&h);
                CHECK(o->state == ONB_RELEASED && o->release, "RELEASED after abort_release_ticks");
            }
        }
        oh_init(&h);
        oh_to(&h, "SAFE");
        CHECK(oh_run(&h, 3 * p.link_abort_ticks, 0)->state == ONB_SAFE, "no link interlock in SAFE");
    }
    /* I2 strict, POWERED only */
    oh_init(&h);
    oh_to(&h, "POWERED");
    h.tilt = p.tilt_abort_deg;
    CHECK(oh_tick(&h)->state == ONB_POWERED, "tilt == limit: no abort");
    h.tilt = p.tilt_abort_deg + 0.01f;
    o = oh_tick(&h);
    CHECK(o->state == ONB_ABORT && o->reason == LP_R_I2_TILT && o->interlock == LP_R_I2_TILT, "I2 tilt");
    oh_init(&h);
    oh_to(&h, "UNPOWERED");
    h.tilt = 80.0f;
    CHECK(oh_tick(&h)->state == ONB_UNPOWERED, "I2 not in UNPOWERED");
    /* I3: three paths, persistence, reset by a good tick */
    for (k = 0; k < 3; k++) {
        oh_init(&h);
        oh_to(&h, "POWERED");
        for (i = 0; i < 2; i++) {
            uint32_t j;
            for (j = 0; j < p.rate_persist_ticks - 1; j++) {
                if (k == 0) h.gyro[0] = sqrtf(p.tilt_rate_abort_sq) * 1.01f;
                if (k == 1) h.gyro[2] = -p.spin_rate_abort * 1.01f;
                if (k == 2) h.gsat = true;
                oh_tick(&h);
            }
            h.gyro[0] = h.gyro[2] = 0.0f;
            h.gsat = false;
            if (i == 0) oh_tick(&h);
        }
        CHECK(h.s.state == ONB_POWERED, "I3 path %d: not before the persistence time", k);
        if (k == 0) h.gyro[0] = sqrtf(p.tilt_rate_abort_sq) * 1.01f;
        if (k == 1) h.gyro[2] = -p.spin_rate_abort * 1.01f;
        if (k == 2) h.gsat = true;
        o = oh_tick(&h);
        CHECK(o->state == ONB_ABORT && o->reason == LP_R_I3_RATE, "I3 path %d", k);
    }
    /* I8 POWERED only; I5 strict in both flight states */
    oh_init(&h);
    oh_to(&h, "POWERED");
    h.vcell = p.min_cell_v - 0.01f;
    o = oh_tick(&h);
    CHECK(o->state == ONB_ABORT && o->reason == LP_R_I8_VCELL, "I8");
    oh_init(&h);
    oh_to(&h, "UNPOWERED");
    h.vcell = p.min_cell_v - 0.5f;
    CHECK(oh_tick(&h)->state == ONB_UNPOWERED, "I8 not in UNPOWERED");
    for (k = 0; k < 2; k++) {
        oh_init(&h);
        h.p.apogee_limit_m = 10.0f * p.geofence_m;
        h.s.p.apogee_limit_m = 10.0f * p.geofence_m;
        oh_to(&h, k ? "UNPOWERED" : "POWERED");
        h.h = p.geofence_m;
        h.vz = -1.0f;
        CHECK(oh_tick(&h)->state == (k ? ONB_UNPOWERED : ONB_POWERED), "h == geofence: no abort");
        h.h = p.geofence_m + 0.01f;
        o = oh_tick(&h);
        CHECK(o->state == ONB_ABORT && o->reason == LP_R_I5_GEOFENCE, "I5 geofence (%d)", k);
    }
    /* I4 apogee limiter: CUT only, coast hold applies; falling: no v^2 term */
    {
        const double v = 20.0, hl = p.apogee_limit_m - (double)p.apogee_k * v * v;
        oh_init(&h);
        oh_to(&h, "POWERED");
        h.vz = (float)v;
        h.h = (float)(hl - 0.01);
        CHECK(oh_tick(&h)->state == ONB_POWERED, "below the apogee limit");
        h.h = (float)(hl + 0.01);
        o = oh_tick(&h);
        CHECK(o->state == ONB_UNPOWERED && o->reason == LP_R_I4_APOGEE_LIMIT && !o->release && o->control_active, "I4 CUT");
        oh_init(&h);
        oh_to(&h, "POWERED");
        h.vz = -(float)v;
        h.h = (float)(hl + 1.0);
        CHECK(oh_tick(&h)->state == ONB_POWERED, "I4: falling, prediction = h");
    }
    /* I7 cumulative powered time (shortened) */
    {
        onb_params_t p7 = p;
        uint32_t n = 0;
        p7.max_motor_ticks = 200;
        oh_init_p(&h, &p7, 1, false);
        oh_to(&h, "POWERED");
        oh_run(&h, 99, 20);
        oh_send(&h, LP_CMD_CUT, 0);
        oh_run(&h, 50, 20);
        oh_send(&h, LP_CMD_THROTTLE, 800);
        CHECK(h.s.state == ONB_POWERED && h.s.powered_ticks == 101, "relight; powered ticks %lu", (unsigned long)h.s.powered_ticks);
        while (h.s.state == ONB_POWERED && n < 1000) {
            if (n % 20) oh_tick(&h); else oh_hb(&h);
            n++;
        }
        CHECK(h.s.state == ONB_UNPOWERED && h.o.reason == LP_R_I7_MOTOR_TIME && h.s.powered_ticks == 200, "I7 at 200 ticks");
    }
    /* I6 fall-safe */
    for (k = 0; k < 2; k++) {
        uint32_t t_rel;
        oh_init(&h);
        oh_to(&h, k ? "UNPOWERED" : "POWERED");
        h.vz = p.fall_vz_mps;
        oh_run(&h, p.fall_confirm_ticks - 1, 0);
        h.vz = p.fall_vz_mps + 0.1f;
        oh_tick(&h);
        h.vz = p.fall_vz_mps;
        oh_run(&h, p.fall_confirm_ticks - 1, 0);
        CHECK(!h.s.has_release_deadline, "I6: a slower tick restarts the count");
        o = oh_tick(&h);
        CHECK(o->state == ONB_UNPOWERED && o->reason == LP_R_I6_FALL && o->throttle_milli == 0, "I6 fall-safe");
        t_rel = h.s.release_deadline;
        CHECK(t_rel == h.k - 1 + p.release_after_cut_ticks, "I6 release time");
        h.vz = -10.0f;
        while (h.s.state != ONB_RELEASED && h.k < t_rel + 10) oh_tick(&h);
        CHECK(h.k - 1 == t_rel && h.o.release, "release opens at release_after_cut_ticks");
    }
    /* I9 imu / baro on the pad and in flight; arm switch = KILL */
    {
        static const char *st[3] = {"ARMED", "POWERED", "UNPOWERED"};
        int which, s;
        for (which = 0; which < 2; which++) {
            for (s = 0; s < 3; s++) {
                uint32_t n = which ? p.baro_fail_ticks : p.imu_fail_ticks, j;
                oh_init(&h);
                oh_to(&h, st[s]);
                for (j = 0; j < 2; j++) {
                    if (which) h.baro_ok = false; else h.imu_ok = false;
                    oh_run(&h, n - 1, 20);
                    h.imu_ok = h.baro_ok = true;
                    if (j == 0) oh_hb(&h);
                }
                CHECK(!strcmp(state_name(h.s.state), st[s]), "I9 %d %s: one good read resets the count", which, st[s]);
                if (which) h.baro_ok = false; else h.imu_ok = false;
                o = oh_hb(&h);
                CHECK(o->state == (s ? ONB_ABORT : ONB_SAFE) && o->reason == (which ? LP_R_I9_BARO : LP_R_I9_IMU) &&
                      !(o->flags & LP_FLAG_SENSORS_OK), "I9 %s %s", which ? "baro" : "imu", st[s]);
            }
        }
        for (s = 0; s < 3; s++) {
            oh_init(&h);
            oh_to(&h, st[s]);
            h.arm = false;
            o = oh_hb(&h);
            CHECK(o->state == (s ? ONB_ABORT : ONB_SAFE) && o->reason == LP_R_ARM_SWITCH_OPEN && !(o->flags & LP_FLAG_ARM_SWITCH),
                  "arm switch open in %s acts as KILL", st[s]);
        }
        oh_init(&h);
        oh_to(&h, "SAFE");
        h.arm = false;
        CHECK(oh_hb(&h)->state == ONB_SAFE, "arm switch in SAFE: nothing");
    }
    /* release paths; an interlock has the last word */
    oh_init(&h);
    oh_to(&h, "ARMED+pending");
    last = h.k - 1;
    oh_run(&h, p.release_after_cut_ticks, 20);
    CHECK(h.s.state == ONB_RELEASED && h.k - 1 == last + p.release_after_cut_ticks, "RELEASE on the pad opens after the delay");
    oh_init(&h);
    oh_to(&h, "ARMED+pending");
    oh_send(&h, LP_CMD_KILL, 0);
    CHECK(oh_run(&h, 2 * p.release_after_cut_ticks, 20)->state == ONB_SAFE && !h.o.release, "KILL on the pad cancels it");
    oh_init(&h);
    oh_to(&h, "POWERED");
    h.tilt = p.tilt_abort_deg + 5.0f;
    o = oh_send(&h, LP_CMD_THROTTLE, 600);
    CHECK(h.last_code == 0 && o->state == ONB_ABORT && o->reason == LP_R_I2_TILT && o->n_trans == 1, "interlock after the command");
    /* telemetry fields and the frame round trip */
    {
        lp_telem_t t, u;
        uint8_t fr[LP_TELEM_FRAME_BYTES];
        lp_decoder_t dec;
        lp_frame_t out[LP_DEC_MAX_OUT];
        uint8_t j, n = 0;
        oh_init(&h);
        oh_to(&h, "POWERED");
        h.h = 12.34f; h.vz = -1.5f; h.roll = 1.0f; h.pitch = -2.0f; h.yaw = 170.0f; h.tilt = 2.25f; h.vcell = 3.95f;
        h.loopmax = 2500.0f;
        o = oh_hb(&h);
        onboard_telemetry(&o->tlm, FC_LOOP_HZ, &t);
        CHECK(t.state == LP_STATE_POWERED && t.t_ms == (h.k - 1) * 1000u / FC_LOOP_HZ && t.nonce == h.s.nonce, "telemetry header");
        CHECK(t.h_dm == 123 && t.vz_cms == -150 && t.roll_cdeg == 100 && t.pitch_cdeg == -200 && t.yaw_cdeg == 17000 &&
              t.tilt_cdeg == 225, "telemetry h/vz/angles (%d %d %d %d %d %u)", t.h_dm, t.vz_cms, t.roll_cdeg, t.pitch_cdeg,
              t.yaw_cdeg, t.tilt_cdeg);
        CHECK(t.vcell_mv == 3950 && t.loop_max_us == 2500 && t.throttle_milli == o->throttle_milli, "telemetry vcell/loop/throttle");
        CHECK(t.flags == (LP_FLAG_ARM_SWITCH | LP_FLAG_SENSORS_OK | LP_FLAG_ALIGNED | LP_FLAG_LINK_OK | LP_FLAG_CONTROL_ACTIVE) &&
              t.reason == LP_REASON_NONE, "telemetry flags 0x%02X", t.flags);
        lp_encode_telem(LINK_VEHICLE_ID, o->reply_seq, &t, fr);
        lp_decoder_init(&dec, LINK_VEHICLE_ID, LP_ACCEPT_TELEM);
        for (j = 0; j < LP_TELEM_FRAME_BYTES; j++) n = (uint8_t)(n + lp_decoder_feed(&dec, fr[j], out + n, (uint8_t)(LP_DEC_MAX_OUT - n)));
        CHECK(n == 1, "TELEM frame decodes");
        lp_unpack_telem(out[0].payload, &u);
        CHECK(telem_eq(&u, &t) && out[0].seq == o->reply_seq, "decode(encode(t)) == t");
        {   /* t_ms beyond 2^32 / 1000 ticks wraps like the reference's & 0xFFFFFFFF */
            onb_tlm_src_t m = o->tlm;
            m.tick = 0xFFFFFFF0UL;
            onboard_telemetry(&m, FC_LOOP_HZ, &t);
            CHECK(t.t_ms == (uint32_t)(((unsigned long long)0xFFFFFFF0ULL * 1000ULL) / FC_LOOP_HZ), "t_ms wraps mod 2^32");
        }
    }
}


static int write_log_sample(const char *path)
{
    FILE *f = fopen(path, "wb");
    uint8_t pay[LOG_MAX_PAYLOAD], fr[LOG_MAX_PAYLOAD + LOG_FRAME_OVERHEAD];
    log_flight_t r;
    log_status_t st;
    uint16_t n;
    int k, i;
    const char *txt = "boot sample", *info = "fw 3.0.0 role FCU BUDGET_90mm_6S params 0123456789abcdef";
    if (!f) return 1;
    pay[0] = LOG_FORMAT_VERSION;                              /* boot record: format, reset cause, loop Hz, counts/us, info */
    pay[1] = 0x01;
    pay[2] = (uint8_t)(FC_LOOP_HZ & 0xFF);
    pay[3] = (uint8_t)(FC_LOOP_HZ >> 8);
    pay[4] = (uint8_t)FC_PWM_COUNTS_PER_US;
    memcpy(pay + 5, info, strlen(info));
    n = log_frame(LOG_TYPE_BOOT, pay, (uint8_t)(5u + strlen(info)), fr);
    fwrite(fr, 1, n, f);
    n = log_frame(LOG_TYPE_TEXT, (const uint8_t *)txt, (uint8_t)strlen(txt), fr);
    fwrite(fr, 1, n, f);
    for (k = 0; k < 3; k++) {
        memset(&r, 0, sizeof r);
        r.tick = 1000u + 5u * (uint32_t)k;
        r.state = (uint8_t)k;
        r.flags = LOG_F_LINK_OK | LOG_F_SENSORS_OK;
        r.reason = (uint8_t)(k == 2 ? LP_R_I4_APOGEE_LIMIT : 0);
        r.aux_bits = LOG_A_ALIGNED | LOG_A_RADIO_OK;
        r.loop_us_last = 1234;
        r.loop_us_max = 2345;
        r.q[0] = 1.0f;
        r.gyro_f[0] = 0.125f * (float)k;
        r.accel[2] = 9.81f;
        r.delta[1] = -0.05f;
        for (i = 0; i < 6; i++) r.pwm_counts[i] = (uint16_t)(3000 + i);
        r.h_est = 12.345f;
        r.vz_est = -1.5f;
        r.h_baro = 12.5f;
        r.throttle = 0.75f;
        r.v_cell = 3.95f;
        r.tilt_deg = 2.5f;
        r.thrust_est = 29.5f;
        n = log_frame(LOG_TYPE_FLIGHT, pay, log_pack_flight(&r, pay), fr);
        fwrite(fr, 1, n, f);
        fputc(0x00, f);                                       /* garbage between frames must be skipped */
    }
    n = log_frame(LOG_TYPE_EVENT, pay, log_pack_event(1010u, ONB_ARMED, ONB_POWERED, ONB_CAUSE_CMD | LP_CMD_LAUNCH, 0, pay), fr);
    fwrite(fr, 1, n, f);
    n = log_frame(LOG_TYPE_LINK, pay, log_pack_link(1010u, 42, LP_CMD_LAUNCH, 0, 0xBEEF, 0, ONB_POWERED, -97, -6, pay), fr);
    fwrite(fr, 1, n, f);
    memset(&st, 0, sizeof st);
    st.tick = 1250u;
    st.stack_free = 1500;
    st.twi_errors = 11;
    st.uart_dropped = 12;
    st.link_frames = 77;
    st.link_bad = 3;
    st.link_other = 13;
    st.radio_errors = 14;
    st.replies = 76;
    st.baro_fails = 15;
    st.imu_fails = 16;
    st.vbat_mv = 23900;
    st.reset_cause = 0x01;
    st.rssi_dbm = -98;                                        /* a real RSSI (LOG_RSSI_NONE is decoded as None) */
    st.il_cmds = 78;
    st.il_snaps = 76;
    st.il_bad = 2;
    st.il_uart_errors = 1;
    st.il_dropped = 4;
    st.com_age_s = 1;
    st.com_flags = (uint8_t)(IL_STAT_RADIO_OK | LOG_COM_TAG_MISMATCH);
    n = log_frame(LOG_TYPE_STATUS, pay, log_pack_status(&st, pay), fr);
    fwrite(fr, 1, n, f);
    {   /* one IL_STAT as the FCU logs it */
        il_stat_t c;
        uint8_t sp[IL_STAT_LEN];
        memset(&c, 0, sizeof c);
        c.com_ticks = 1240u;
        c.reset_cause = 0x08;
        c.flags = IL_STAT_RADIO_OK | IL_STAT_BOOT | IL_STAT_SX127X;
        c.radio_version = 0x12;
        c.rssi_dbm = -101;
        c.snr_q4 = -10;
        c.params_tag = 0x48;
        c.dec_frames = 77;
        c.dec_bad_crc = 3;
        c.cmd_forwarded = 77;
        c.replies = 76;
        c.il_tx_dropped = 1;
        c.loop_us_max = 812;
        il_pack_stat(&c, sp);
        n = log_frame(LOG_TYPE_COM, pay, log_pack_com(1245u, sp, pay), fr);
        fwrite(fr, 1, n, f);
    }
    fclose(f);
    return 0;
}

/* ------------------------------------------------------------------------ */
/* onboard supervisor vectors (tools/crosscheck_controller.py)              */
/* ------------------------------------------------------------------------ */
static char g_line[1u << 17];

/* strtod that also reads nan, inf, -inf (as Python writes them) whatever the C runtime's strtod does */
static double strtod_x(const char *s, char **end)
{
    const char *q = s;
    int neg = 0;
    if (*q == '-' || *q == '+') neg = *q++ == '-';
    if (!strncmp(q, "nan", 3)) { *end = (char *)q + 3; return neg ? -(double)NAN : (double)NAN; }
    if (!strncmp(q, "inf", 3)) { *end = (char *)q + 3; return neg ? -(double)INFINITY : (double)INFINITY; }
    return strtod(s, end);
}

static int parse_numbers(const char *s, double *v, int maxn)
{
    char *end;
    int n = 0;
    while (n < maxn) {
        while (*s == ',' || *s == ' ') s++;
        if (*s == '\0' || *s == '\n' || *s == '\r') break;
        v[n] = strtod_x(s, &end);
        if (end == s) break;
        n++;
        s = end;
    }
    return n;
}

static int onb_override(onb_params_t *p, const char *kv)
{
    char key[64];
    const char *eq = strchr(kv, '=');
    double x;
    size_t n;
    if (!eq) return 0;
    n = (size_t)(eq - kv);
    if (n >= sizeof key) return 0;
    memcpy(key, kv, n);
    key[n] = '\0';
    x = strtod(eq + 1, 0);
#define U32(name) if (!strcmp(key, #name)) { p->name = (uint32_t)x; return 1; }
#define F32(name) if (!strcmp(key, #name)) { p->name = (float)x; return 1; }
#define I16(name) if (!strcmp(key, #name)) { p->name = (int16_t)x; return 1; }
    U32(ramp_ticks) U32(coast_hold_ticks) U32(fall_confirm_ticks) U32(link_abort_ticks) U32(link_disarm_ticks)
    U32(max_motor_ticks) U32(release_after_cut_ticks) U32(abort_release_ticks) U32(rate_persist_ticks)
    U32(imu_fail_ticks) U32(baro_fail_ticks)
    F32(tilt_abort_deg) F32(arm_tilt_max_deg) F32(min_cell_v) F32(geofence_m) F32(apogee_limit_m) F32(fall_vz_mps)
#undef U32
#undef F32
#undef I16
    if (!strcmp(key, "climb_milli")) { p->climb_milli = (uint16_t)x; return 1; }
    if (!strcmp(key, "gyro_saturation_abort")) { p->gyro_saturation_abort = x != 0.0; return 1; }
    if (!strcmp(key, "mode")) { p->mode = (uint8_t)x; return 1; }
    return 0;
}

/* supervisor vectors (the rocket's ballistic supervisor):
 *   #S id seed reset_in_flight [key=value ...]   new supervisor
 *   row: tick, n_frames, 4 x (seq cmd arg nonce), h vz tilt gx gy gz gsat vcell imu baro aligned arm roll pitch yaw loopmax
 *   out: seq,tick,state,target,out,active,release,reason,flags,ack,nonce,reply_seq,interlock,telem (TELEM v1, 28 bytes),
 *        exec (seq:cmd:code:sb:sa|...),trans (from:to:cause|...)
 *   (the VTOL program's stack rows and inputs went on 2026-11-18, its phase / target / motor / buzzer / baro-lost
 *   columns on 2026-11-19) */

static int run_onb_vectors(const char *in, const char *out)
{
    FILE *fi = fopen(in, "r"), *fo = fopen(out, "w");
    static onboard_t sup;
    static onb_outputs_t o;
    onboard_t *s = &sup;
    onb_frame_t fr[ONB_MAX_FRAMES];
    double v[64];
    long id = -1, rows = 0;
    if (!fi || !fo) return 2;
    fprintf(fo, "seq,tick,state,target,out,active,release,reason,flags,ack,nonce,reply_seq,interlock,telem,exec,trans\n");
    while (fgets(g_line, sizeof g_line, fi)) {
        if (g_line[0] == '#' && g_line[1] == 'S') {      /* #S id seed reset [key=value ...] */
            onb_params_t p;
            long seed = 0, reset = 0;
            char *tok;
            fc_default_onboard(&p);
            p.mode = LP_MODE_BALLISTIC;
            tok = strtok(g_line + 2, " \t\r\n");
            id = tok ? strtol(tok, 0, 10) : 0;
            tok = strtok(0, " \t\r\n");
            seed = tok ? strtol(tok, 0, 10) : 0;
            tok = strtok(0, " \t\r\n");
            reset = tok ? strtol(tok, 0, 10) : 0;
            while ((tok = strtok(0, " \t\r\n")) != 0) {
                if (!onb_override(&p, tok)) {
                    fprintf(stderr, "onb-vectors: unknown override %s\n", tok);
                    return 3;
                }
            }
            onboard_init(s, &p, (uint16_t)seed, reset != 0);
            continue;
        }
        if (g_line[0] == '#' || g_line[0] == 's' || g_line[0] == '\n' || g_line[0] == '\r') continue;
        {
            const int nv = parse_numbers(g_line, v, 64);
            onb_inputs_t inp;
            int nf = (int)v[1], i, b = 18;
            if (nv < 34) {
                fprintf(stderr, "onb-vectors: short row %ld\n", rows);
                return 3;
            }
            memset(&inp, 0, sizeof inp);
            for (i = 0; i < nf && i < (int)ONB_MAX_FRAMES; i++) {
                fr[i].seq = (uint8_t)v[2 + 4 * i];
                fr[i].cmd.cmd = (uint8_t)v[3 + 4 * i];
                fr[i].cmd.arg = (uint16_t)v[4 + 4 * i];
                fr[i].cmd.nonce = (uint16_t)v[5 + 4 * i];
            }
            inp.tick = (uint32_t)v[0];
            inp.frames = fr;
            inp.n_frames = (uint8_t)nf;
            inp.h_m = (float)v[b + 0];
            inp.vz_mps = (float)v[b + 1];
            inp.tilt_deg = (float)v[b + 2];
            inp.gyro[0] = (float)v[b + 3];
            inp.gyro[1] = (float)v[b + 4];
            inp.gyro[2] = (float)v[b + 5];
            inp.gyro_saturated = v[b + 6] != 0.0;
            inp.v_cell = (float)v[b + 7];
            inp.imu_ok = v[b + 8] != 0.0;
            inp.baro_ok = v[b + 9] != 0.0;
            inp.aligned = v[b + 10] != 0.0;
            inp.arm_switch = v[b + 11] != 0.0;
            inp.roll_deg = (float)v[b + 12];
            inp.pitch_deg = (float)v[b + 13];
            inp.yaw_deg = (float)v[b + 14];
            inp.loop_max_us = (float)v[b + 15];
            onboard_update(s, &inp, &o);
            fprintf(fo, "%ld,%lu,%u,%u,%u,%d,%d,%u,%u,%u,%u,%d,%u,", id, (unsigned long)inp.tick, o.state, o.target_milli,
                    o.throttle_milli, o.control_active ? 1 : 0, o.release ? 1 : 0, o.reason, o.flags, o.ack_seq, o.nonce,
                    o.reply ? (int)o.reply_seq : -1, o.interlock);
            if (o.reply) {
                lp_telem_t t;
                uint8_t pl[LP_TELEM_PAYLOAD_LEN], q;
                onboard_telemetry(&o.tlm, s->p.loop_hz, &t);
                lp_pack_telem(&t, pl);
                for (q = 0; q < LP_TELEM_PAYLOAD_LEN; q++) fprintf(fo, "%02x", pl[q]);
            } else {
                fputc('-', fo);
            }
            fputc(',', fo);
            if (!o.n_exec) fputc('-', fo);
            for (i = 0; i < o.n_exec; i++)
                fprintf(fo, "%s%u:%u:%u:%u:%u", i ? "|" : "", o.exec[i].seq, o.exec[i].cmd, o.exec[i].code,
                        o.exec[i].state_before, o.exec[i].state_after);
            fputc(',', fo);
            if (!o.n_trans) fputc('-', fo);
            for (i = 0; i < o.n_trans; i++)
                fprintf(fo, "%s%u:%u:%u", i ? "|" : "", o.trans[i].from, o.trans[i].to, o.trans[i].cause);
            fputc('\n', fo);
            rows++;
        }
    }
    fclose(fi);
    fclose(fo);
    fprintf(stderr, "onb-vectors: %ld rows\n", rows);
    return 0;
}

/* ------------------------------------------------------------------------ */
/* protocol vectors: D decode stream, C / T encode, Q quantise              */
/* ------------------------------------------------------------------------ */
static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static int run_proto_vectors(const char *in, const char *out)
{
    FILE *fi = fopen(in, "r"), *fo = fopen(out, "w");
    long rows = 0;
    if (!fi || !fo) return 2;
    while (fgets(g_line, sizeof g_line, fi)) {
        char kind = g_line[0];
        double v[32];
        uint8_t fr[LP_FRAME_MAX_BYTES];
        uint8_t i, n;
        if (kind == 'D') {                                      /* D vid accept hex */
            lp_decoder_t dec;
            lp_frame_t o[LP_DEC_MAX_OUT];
            char *p = g_line + 1, *end;
            long vid = strtol(p, &end, 10), acc;
            p = end;
            acc = strtol(p, &end, 10);
            p = end;
            while (*p == ' ') p++;
            lp_decoder_init(&dec, (int16_t)vid, (uint8_t)acc);
            fputc('D', fo);
            while (hexval(p[0]) >= 0 && hexval(p[1]) >= 0) {
                uint8_t b = (uint8_t)(hexval(p[0]) * 16 + hexval(p[1])), m, k;
                p += 2;
                m = lp_decoder_feed(&dec, b, o, LP_DEC_MAX_OUT);
                for (k = 0; k < m; k++) {
                    fprintf(fo, " %u:%u:%u:", o[k].veh, o[k].type, o[k].seq);
                    for (i = 0; i < o[k].len; i++) fprintf(fo, "%02x", o[k].payload[i]);
                }
            }
            fprintf(fo, " S %u %u %u %u %u %u %u %u\n", dec.stats.frames, dec.stats.bad_len, dec.stats.bad_crc,
                    dec.stats.wrong_vehicle, dec.stats.wrong_type, dec.stats.bad_payload_len, dec.stats.skipped_bytes,
                    dec.stats.overflow);
        } else if (kind == 'C') {                               /* C veh seq cmd arg nonce */
            lp_cmd_t c;
            parse_numbers(g_line + 1, v, 5);
            c.cmd = (uint8_t)v[2];
            c.arg = (uint16_t)v[3];
            c.nonce = (uint16_t)v[4];
            n = lp_encode_cmd((uint8_t)v[0], (uint8_t)v[1], &c, fr);
            fputc('C', fo);
            fputc(' ', fo);
            for (i = 0; i < n; i++) fprintf(fo, "%02x", fr[i]);
            fputc('\n', fo);
        } else if (kind == 'T') {                               /* T veh seq 15 fields */
            lp_telem_t t;
            parse_numbers(g_line + 1, v, 17);
            t.state = (uint8_t)v[2]; t.flags = (uint8_t)v[3]; t.reason = (uint8_t)v[4]; t.ack_seq = (uint8_t)v[5];
            t.nonce = (uint16_t)v[6]; t.t_ms = (uint32_t)v[7]; t.h_dm = (int16_t)v[8]; t.vz_cms = (int16_t)v[9];
            t.roll_cdeg = (int16_t)v[10]; t.pitch_cdeg = (int16_t)v[11]; t.yaw_cdeg = (int16_t)v[12];
            t.tilt_cdeg = (uint16_t)v[13]; t.throttle_milli = (uint16_t)v[14]; t.vcell_mv = (uint16_t)v[15];
            t.loop_max_us = (uint16_t)v[16];
            n = lp_encode_telem((uint8_t)v[0], (uint8_t)v[1], &t, fr);
            fputc('T', fo);
            fputc(' ', fo);
            for (i = 0; i < n; i++) fprintf(fo, "%02x", fr[i]);
            fputc('\n', fo);
        } else if (kind == 'Q') {                               /* Q x scale lo hi */
            parse_numbers(g_line + 1, v, 4);
            fprintf(fo, "Q %ld\n", (long)lp_quantize((float)v[0], (uint16_t)v[1], (int32_t)v[2], (int32_t)v[3]));
        } else {
            continue;
        }
        rows++;
    }
    fclose(fi);
    fclose(fo);
    fprintf(stderr, "proto-vectors: %ld rows\n", rows);
    return 0;
}

/* ------------------------------------------------------------------------ */
/* ilink vectors (tools/crosscheck_controller.py --only ilink):              */
/*   D max_out hex     decode a stream  -> D type:len:payload ... S counters P pending N need.. C cmd_need.. (one hex
 *                     byte per input byte; " PUSHDIFF" appended if il_decoder_push on a twin decoder disagrees)
 *   E type hex        unpack + pack + il_encode (raw il_encode if the length is wrong) -> E frame | E -
 *   R init hex        CRC -> R il_crc16 lp_crc16
 *   P veh hz snaphex  COM pipeline: il_unpack_snap, il_snap_to_tlm_src, onboard_telemetry, lp_encode_telem -> P frame
 *   F 20 integers     FCU: il_snap_from_tlm_src + il_pack_snap -> F payload
 *                     (reply_seq state flags reason ack nonce throttle tick, then h vz roll pitch yaw tilt vcell loop
 *                     q0..q3 as uint32 float bit patterns: exact for every value, NaN payloads included) */
/* ------------------------------------------------------------------------ */
static int parse_hex(const char *p, uint8_t *buf, int max)
{
    int n = 0;
    while (*p == ' ') p++;
    while (n < max && hexval(p[0]) >= 0 && hexval(p[1]) >= 0) {
        buf[n++] = (uint8_t)(hexval(p[0]) * 16 + hexval(p[1]));
        p += 2;
    }
    return n;
}

static void put_hex_line(FILE *fo, char kind, const uint8_t *b, int n)
{
    int i;
    fputc(kind, fo);
    fputc(' ', fo);
    for (i = 0; i < n; i++) fprintf(fo, "%02x", b[i]);
    fputc('\n', fo);
}

static int run_ilink_vectors(const char *in, const char *out)
{
    FILE *fi = fopen(in, "r"), *fo = fopen(out, "w");
    static il_decoder_t dec;
    long rows = 0;
    if (!fi || !fo) return 2;
    while (fgets(g_line, sizeof g_line, fi)) {
        const char kind = g_line[0];
        uint8_t buf[256], fr[IL_FRAME_MAX_BYTES + LP_TELEM_FRAME_BYTES];
        char *p = g_line + 1, *end;
        int n, i;
        if (kind == 'D') {
            static il_decoder_t dp;
            static uint8_t need[sizeof g_line / 2], cneed[sizeof g_line / 2];
            il_frame_t o[8];
            long mo = strtol(p, &end, 10), nb = 0, push_diff = 0, l, acc;
            p = end;
            acc = strtol(p, &end, 10);                 /* D <max_out> <accept mask> <hex> */
            p = end;
            while (*p == ' ') p++;
            if (mo < 0) mo = 0;
            if (mo > 8) mo = 8;
            il_decoder_init(&dec, (uint8_t)acc);
            il_decoder_init(&dp, (uint8_t)acc);
            fputc('D', fo);
            while (hexval(p[0]) >= 0 && hexval(p[1]) >= 0) {
                const uint8_t b = (uint8_t)(hexval(p[0]) * 16 + hexval(p[1]));
                const il_frame_t *pf;
                const uint16_t ov = dec.stats.overflow;
                uint8_t m, k, j;
                p += 2;
                m = il_decoder_feed(&dec, b, o, (uint8_t)mo);
                pf = il_decoder_push(&dp, b);          /* the zero-copy form on a twin decoder: the same frames */
                if ((pf != 0) != (m == 1u || dec.stats.overflow != ov) ||
                    (m == 1u && (pf->type != o[0].type || pf->len != o[0].len || memcmp(pf->payload, o[0].payload, pf->len) != 0)))
                    push_diff++;
                need[nb] = il_decoder_need(&dec);
                cneed[nb++] = il_decoder_cmd_need(&dec);
                for (k = 0; k < m; k++) {
                    fprintf(fo, " %u:%u:", o[k].type, o[k].len);
                    for (j = 0; j < o[k].len; j++) fprintf(fo, "%02x", o[k].payload[j]);
                }
            }
            if (dp.n != dec.n || dp.stats.frames != dec.stats.frames || dp.stats.skipped_bytes != dec.stats.skipped_bytes ||
                dp.stats.bad_crc != dec.stats.bad_crc || dp.stats.bad_len != dec.stats.bad_len ||
                dp.stats.bad_type != dec.stats.bad_type || dp.stats.overflow != 0u)
                push_diff++;
            fprintf(fo, " S %u %u %u %u %u %u P %u N ", dec.stats.frames, dec.stats.bad_crc, dec.stats.bad_len,
                    dec.stats.bad_type, dec.stats.skipped_bytes, dec.stats.overflow, dec.n);
            for (l = 0; l < nb; l++) fprintf(fo, "%02x", need[l]);
            fputs(" C ", fo);
            for (l = 0; l < nb; l++) fprintf(fo, "%02x", cneed[l]);
            fprintf(fo, "%s\n", push_diff ? " PUSHDIFF" : "");
        } else if (kind == 'E') {
            const long type = strtol(p, &end, 10);
            uint8_t pl[IL_MAX_PAYLOAD], len;
            n = parse_hex(end, buf, (int)sizeof buf);
            if (il_payload_len((uint8_t)type) != 0u && n == (int)il_payload_len((uint8_t)type)) {
                il_repack((uint8_t)type, buf, pl);
                len = il_encode((uint8_t)type, pl, (uint8_t)n, fr);
            } else {
                len = il_encode((uint8_t)type, buf, (uint8_t)n, fr);
            }
            if (len)
                put_hex_line(fo, 'E', fr, len);
            else
                fputs("E -\n", fo);
        } else if (kind == 'R') {
            const long init = strtol(p, &end, 10);
            n = parse_hex(end, buf, 255);
            fprintf(fo, "R %04x %04x\n", il_crc16(buf, (uint8_t)n, (uint16_t)init), lp_crc16(buf, (uint8_t)n, (uint16_t)init));
        } else if (kind == 'P') {                      /* SNAP -> TELEM (the rocket's COM) */
            const long veh = strtol(p, &end, 10), hz = strtol(end, &end, 10);
            onb_tlm_src_t src;
            lp_telem_t t;
            uint8_t seq;
            n = parse_hex(end, buf, (int)sizeof buf);
            memset(&src, 0, sizeof src);
            if (n == (int)IL_SNAP_LEN) {
                il_snap_t s;
                il_unpack_snap(buf, &s);
                il_snap_to_tlm_src(&s, &src);
                seq = s.reply_seq;
            } else {
                fprintf(stderr, "ilink-vectors: P line with %d snapshot bytes\n", n);
                return 3;
            }
            onboard_telemetry(&src, (uint16_t)hz, &t);
            put_hex_line(fo, 'P', fr, lp_encode_telem((uint8_t)veh, seq, &t, fr));
        } else if (kind == 'F') {                      /* 20 numbers: SNAP */
            double v[20];
            onb_tlm_src_t src;
            il_snap_t s;
            float q[4];
            const int nv = parse_numbers(p, v, 20);
            if (nv != 20) {
                fprintf(stderr, "ilink-vectors: short F line %ld\n", rows);
                return 3;
            }
            memset(&src, 0, sizeof src);               /* v0.9: pw_vaux / pw_flags 0 (the reference's TlmSource pwr None) */
            src.state = (uint8_t)v[1]; src.flags = (uint8_t)v[2]; src.reason = (uint8_t)v[3]; src.ack_seq = (uint8_t)v[4];
            src.nonce = (uint16_t)v[5]; src.throttle_milli = (uint16_t)v[6]; src.tick = (uint32_t)v[7];
            src.h_m = il_from_bits((long long)v[8]); src.vz_mps = il_from_bits((long long)v[9]);
            src.roll_deg = il_from_bits((long long)v[10]); src.pitch_deg = il_from_bits((long long)v[11]);
            src.yaw_deg = il_from_bits((long long)v[12]); src.tilt_deg = il_from_bits((long long)v[13]);
            src.v_cell = il_from_bits((long long)v[14]); src.loop_max_us = il_from_bits((long long)v[15]);
            for (i = 0; i < 4; i++) q[i] = il_from_bits((long long)v[16 + i]);
            {
                il_snap_from_tlm_src((uint8_t)v[0], &src, q, &s);
                il_pack_snap(&s, buf);
                put_hex_line(fo, 'F', buf, IL_SNAP_LEN);
            }
        } else {
            continue;
        }
        rows++;
    }
    fclose(fi);
    fclose(fo);
    fprintf(stderr, "ilink-vectors: %ld rows\n", rows);
    return 0;
}

/* nlink vectors: N / F encode, D decode */
static int run_nlink_vectors(const char *in, const char *out)
{
    FILE *fi = fopen(in, "r"), *fo = fopen(out, "w");
    long rows = 0;
    if (!fi || !fo) return 2;
    while (fgets(g_line, sizeof g_line, fi)) {
        const char kind = g_line[0];
        uint8_t buf[NL_XFER_BYTES];
        double v[21];
        int i;
        if (kind == 'N') {                     /* N seq fb0..fb13 iv0..iv5 -> N <72 bytes> */
            nl_nav_t n;
            uint32_t fb[14];
            long iv[6];
            if (parse_numbers(g_line + 1, v, 21) != 21) {
                fprintf(stderr, "nlink-vectors: short N line %ld\n", rows);
                return 3;
            }
            for (i = 0; i < 14; i++) fb[i] = (uint32_t)v[1 + i];
            for (i = 0; i < 6; i++) iv[i] = (long)v[15 + i];
            nl_nav_from_fields(&n, (uint8_t)v[0], fb, iv);
            memset(buf, 0xEE, sizeof buf);
            put_hex_line(fo, 'N', buf, nl_encode_nav(&n, buf) == NL_XFER_BYTES ? NL_XFER_BYTES : 0);
        } else if (kind == 'F') {              /* F seq state fflags tag [thr_q] -> F <72 bytes> (thr_q: the reserved byte) */
            nl_fcu_t f;
            const int nv = parse_numbers(g_line + 1, v, 5);
            if (nv != 4 && nv != 5) {
                fprintf(stderr, "nlink-vectors: short F line %ld\n", rows);
                return 3;
            }
            f.seq = (uint8_t)v[0];
            f.state = (uint8_t)v[1];
            f.fflags = (uint8_t)v[2];
            f.params_tag = (uint8_t)v[3];
            f.thr_q = nv == 5 ? (uint8_t)v[4] : 0u;
            memset(buf, 0xEE, sizeof buf);
            put_hex_line(fo, 'F', buf, nl_encode_fcu(&f, buf) == NL_FCU_FRAME_BYTES ? NL_XFER_BYTES : 0);
        } else if (kind == 'D') {              /* D <72 bytes> -> D <nav ok> [seq fb.. iv..] | <fcu ok> [seq st ff tag] */
            nl_nav_t n;
            nl_fcu_t f;
            if (parse_hex(g_line + 1, buf, NL_XFER_BYTES) != NL_XFER_BYTES) {
                fprintf(stderr, "nlink-vectors: D line %ld is not 72 bytes\n", rows);
                return 3;
            }
            if (nl_decode_nav(buf, &n)) {
                uint32_t fb[14];
                long iv[6];
                nl_nav_to_fields(&n, fb, iv);
                fprintf(fo, "D 1 %u", n.seq);
                for (i = 0; i < 14; i++) fprintf(fo, " %08lx", (unsigned long)fb[i]);
                for (i = 0; i < 6; i++) fprintf(fo, " %ld", iv[i]);
            } else {
                fputs("D 0", fo);
            }
            if (nl_decode_fcu(buf, &f))
                fprintf(fo, " | 1 %u %u %u %u %u\n", f.seq, f.state, f.fflags, f.params_tag, f.thr_q);
            else
                fputs(" | 0\n", fo);
        } else {
            continue;
        }
        rows++;
    }
    fclose(fi);
    fclose(fo);
    fprintf(stderr, "nlink-vectors: %ld rows\n", rows);
    return 0;
}

/* ------------------------------------------------------------------------ */
/* CSV vector modes                                                         */
/* ------------------------------------------------------------------------ */
#define MAXF 32
static int read_row(FILE *f, double *v, int maxn)
{
    char line[1024], *p, *end;
    int n = 0;
    for (;;) {
        if (!fgets(line, sizeof line, f)) return -1;
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') continue;
        if ((line[0] < '0' || line[0] > '9') && line[0] != '-' && line[0] != '+' && line[0] != '.') continue; /* header */
        break;
    }
    p = line;
    while (n < maxn) {
        v[n] = strtod(p, &end);
        if (end == p) break;
        n++;
        p = end;
        while (*p == ',' || *p == ' ') p++;
        if (*p == '\0' || *p == '\n' || *p == '\r') break;
    }
    return n;
}

static float F(double x) { return (float)x; }   /* inputs are written as exact floats (9 significant digits) */

/* the ballistic build's gains (fc_default_gains, v0.6.3); the VTOL gain set (--vectors-vtol) went with the VTOL program */
static int run_vectors(const char *in, const char *out)
{
    FILE *fi = fopen(in, "r"), *fo = fopen(out, "w");
    attitude_ctrl_t c;
    att_gains_t g;
    mixer_plant_t p;
    ctrl_output_t o;
    double v[MAXF];
    long rows = 0;
    if (!fi || !fo) return 2;
    make_ctrl(&c, &g, &p);
    fprintf(fo, "seq,d1,d2,d3,d4,mx,my,mz,rsx,rsy,rsz,ex,ey,ez,wfx,wfy,wfz,sat,that,ix,iy,iz,p1,p2,p3,p4\n");
    /* in: seq,reset,set_yaw,yqw,yqx,yqy,yqz,qw,qx,qy,qz,gx,gy,gz,u[,tx,ty,finish]
     * set_yaw: set_yaw_reference(yq) before the update; reset: reset() before the update; the optional tx, ty, finish
     * columns (the removed VTOL tilt set point) are read and ignored */
    int nr;
    while ((nr = read_row(fi, v, MAXF)) >= 15) {
        float yq[4] = {F(v[3]), F(v[4]), F(v[5]), F(v[6])};
        float q[4] = {F(v[7]), F(v[8]), F(v[9]), F(v[10])}, gy[3] = {F(v[11]), F(v[12]), F(v[13])};
        if (v[1] != 0.0) attitude_ctrl_reset(&c);
        if (v[2] != 0.0) attitude_ctrl_set_yaw_reference(&c, yq);
        attitude_ctrl_update(&c, q, gy, F(v[14]), &o);
        fprintf(fo, "%.0f,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%.9g,"
                    "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
                v[0], o.delta_cmd[0], o.delta_cmd[1], o.delta_cmd[2], o.delta_cmd[3], o.moments[0], o.moments[1],
                o.moments[2], o.rate_sp[0], o.rate_sp[1], o.rate_sp[2], o.att_error[0], o.att_error[1], o.att_error[2],
                o.gyro_filtered[0], o.gyro_filtered[1], o.gyro_filtered[2], o.saturated ? 1 : 0, o.thrust_est,
                o.integrator[0], o.integrator[1], o.integrator[2], o.pwm_us[0], o.pwm_us[1], o.pwm_us[2], o.pwm_us[3]);
        rows++;
    }
    fclose(fi);
    fclose(fo);
    fprintf(stderr, "vectors: %ld rows\n", rows);
    return 0;
}


static int run_est_vectors(const char *in, const char *out)
{
    FILE *fi = fopen(in, "r"), *fo = fopen(out, "w");
    att_est_t e;
    double v[MAXF];
    long rows = 0;
    if (!fi || !fo) return 2;
    fc_default_estimator(&e);
    fprintf(fo, "seq,qw,qx,qy,qz,bx,by,bz,rx,ry,rz\n");
    /* in: seq,init,gx,gy,gz,ax,ay,az,on_pad   (init: fresh estimator before this row) */
    while (read_row(fi, v, MAXF) >= 9) {
        float gy[3] = {F(v[2]), F(v[3]), F(v[4])}, ac[3] = {F(v[5]), F(v[6]), F(v[7])}, r[3];
        if (v[1] != 0.0) fc_default_estimator(&e);
        att_est_update(&e, gy, ac, v[8] != 0.0);
        att_est_rate(&e, gy, r);
        fprintf(fo, "%.0f,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n", v[0], e.q[0], e.q[1], e.q[2], e.q[3],
                e.bias[0], e.bias[1], e.bias[2], r[0], r[1], r[2]);
        rows++;
    }
    fclose(fi);
    fclose(fo);
    fprintf(stderr, "est-vectors: %ld rows\n", rows);
    return 0;
}

static int run_alt_vectors(const char *in, const char *out)
{
    FILE *fi = fopen(in, "r"), *fo = fopen(out, "w");
    alt_filter_t f;
    double v[MAXF], seq = -1;
    long rows = 0;
    if (!fi || !fo) return 2;
    fprintf(fo, "seq,t,h,v\n");
    /* in: seq,t,latency,az,has_baro,h_baro */
    while (read_row(fi, v, MAXF) >= 6) {
        if (v[0] != seq) {
            fc_default_alt_filter(&f, F(v[2]));
            alt_filter_reset(&f, 0.0f);
            seq = v[0];
        }
        alt_filter_predict(&f, F(v[3]), FC_DT_S);
        if (v[4] != 0.0) alt_filter_correct(&f, F(v[5]), (uint32_t)v[1]);
        fprintf(fo, "%.0f,%.0f,%.9g,%.9g\n", v[0], v[1], f.h, f.v);
        rows++;
    }
    fclose(fi);
    fclose(fo);
    fprintf(stderr, "alt-vectors: %ld rows\n", rows);
    return 0;
}


int main(int argc, char **argv)
{
    if (argc == 4 && strcmp(argv[1], "--vectors") == 0) return run_vectors(argv[2], argv[3]);
    if (argc == 4 && strcmp(argv[1], "--onb-vectors") == 0) return run_onb_vectors(argv[2], argv[3]);
    if (argc == 4 && strcmp(argv[1], "--proto-vectors") == 0) return run_proto_vectors(argv[2], argv[3]);
    if (argc == 4 && strcmp(argv[1], "--ilink-vectors") == 0) return run_ilink_vectors(argv[2], argv[3]);
    if (argc == 4 && strcmp(argv[1], "--nlink-vectors") == 0) return run_nlink_vectors(argv[2], argv[3]);
    if (argc == 4 && strcmp(argv[1], "--est-vectors") == 0) return run_est_vectors(argv[2], argv[3]);
    if (argc == 4 && strcmp(argv[1], "--alt-vectors") == 0) return run_alt_vectors(argv[2], argv[3]);
    if (argc == 3 && strcmp(argv[1], "--log-sample") == 0) return write_log_sample(argv[2]);
    if (argc != 1) {
        fprintf(stderr, "usage: %s [--vectors|--onb-vectors|--proto-vectors|--ilink-vectors|"
                        "--nlink-vectors|--est-vectors|--alt-vectors in out | "
                        "--log-sample out.bin | --consts out.txt]\n", argv[0]);
        return 2;
    }
    test_quat();
    test_mixer();
    test_pid();
    test_alt_filter();
    test_estimator();
    test_edf();
    test_ms5611();
    test_bmp280();
    test_log();
    test_pwm_counts();
    test_link_proto();
    test_onboard_table();
    test_onboard_rules();
    test_ilink_proto();                    /* random draws: after the v0.6.3 tests, so their sequences stay as they were */
    test_nlink_proto();                    /* no random draws */
    printf("test_core: %d checks, %d failures\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
