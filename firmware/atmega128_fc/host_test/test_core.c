/* test_core.c - host tests of the portable firmware core (src/core) and the
 * vector modes used by tools/crosscheck_controller.py.
 *
 *   test_core                                run every unit test, exit 1 on any failure
 *   test_core --vectors in.csv out.csv       attitude controller (sim/controller.py)
 *   test_core --vectors-vtol in.csv out.csv  the same with the VTOL build's gains (v0.7.1b fc_vtol_gains)
 *   test_core --onb-vectors in.txt out.txt   onboard supervisor (sim/onboard.py), both builds (#S ... mode=1: VTOL)
 *   test_core --stack-vectors in.txt out.txt the FCU stack supervisor -> vertical controller (sim/onboard.py FcuStack)
 *   test_core --vc-vectors in.txt out.txt    VTOL vertical controller (sim/vertical_ctrl.py)
 *   test_core --proto-vectors in.txt out.txt link protocol: decode / encode (TELEM, TELEM2) / quantise (sim/link_proto.py)
 *   test_core --ilink-vectors in.txt out.txt FCU <-> COM ilink: decode / encode / CRC / SNAP(2) -> TELEM(2) (sim/ilink_proto.py)
 *   test_core --nlink-vectors in.txt out.txt NAV <-> FCU nlink: encode / decode of both frames (sim/nlink_proto.py)
 *   test_core --est-vectors in.csv out.csv   attitude estimator (sim/estimator.py PadAlignGyroEstimator)
 *   test_core --alt-vectors in.csv out.csv   altitude filter (sim/state_machine.py AltitudeFilter)
 *   test_core --mahony-vectors in out        v0.7.1 VTOL NAV estimator (sim/estimator.py GatedMahonyEstimator)
 *   test_core --altv-vectors in out          v0.7.1 VTOL NAV altitude filter (sim/state_machine.py VtolAltitudeFilter)
 *   test_core --ubx-vectors in out           v0.8 UBX parser / decoders / epoch assembler / iTOW clock / configurator
 *   test_core --gpsnav-vectors in out        v0.8 NAV GPS side (sim/gps_nav.py NavGps / NavGpsFilter) on recorded ticks
 *   test_core --pos-vectors in out           v0.8 FCU position hold (sim/position_ctrl.py PositionHold, TiltSetpoint)
 *   test_core --glink-vectors in out         v0.8.1 glink frames / decoder / NAV side, nlink v4 slots, FCU4 / NAV4
 *   test_core --gns-vectors in out           v0.8.1 GNS node (sim/glink_proto.py GnsNode) on UBX bytes + STATE frames
 *   test_core --predict-vectors in out       v0.8 NAV gyro prediction (sim/estimator.py GatedMahonyEstimator.predict)
 *   test_core --log-sample out.bin           a few framed log records for tools/decode_log.py (every record type)
 *   test_core --consts out.txt               fw09 X1: the build's constants as the firmware structs hold them (fc_config.c:
 *                                            plant, VTOL gains, supervisor, vertical controller, position hold, NAV
 *                                            estimator / altitude filter, the per-configuration macros) - the crosscheck
 *                                            compares them with the 6-DOF's own parameters (-D FC_VCFG=n per vehicle)
 *
 * Built by tools/build_host_test.py with `python -m ziglang cc -std=c99 -O1
 * -ffp-contract=off`, linking src/core only; build/link_vectors_gen.h and
 * build/ilink_vectors_gen.h / build/nlink_vectors_gen.h are generated from
 * sim/out/link_vectors.json, sim/out/ilink_vectors.json and sim/out/nlink_vectors.json
 * by the same script.  Invariant: the unit tests use
 * the generated parameter headers through fc_config.c, i.e. the numbers the
 * firmware flies, and compare against values worked out independently here
 * (double precision, closed form, the SPEC 8.8 / 8.9 / 10.3 tables, or the reference's
 * own byte vectors), never against the code under test.  The supervisor tables and
 * rules run in both builds (onb_params_t.mode: the ballistic v0.6.3 tests force
 * LP_MODE_BALLISTIC, the v0.7 tests LP_MODE_VTOL, whatever params.json mission.mode is).
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/core/alt_filter.h"
#include "../src/core/att_estimator.h"
#include "../src/core/attitude_ctrl.h"
#include "../src/core/fc_config.h"
#include "../src/core/link_proto_v09.h"
#include "../src/core/glink_proto.h"
#include "../src/core/gns_node.h"
#include "../src/core/gps_nav.h"
#include "../src/core/gps_ubx.h"
#include "../src/core/ilink_proto.h"
#include "../src/core/link_proto.h"
#include "../src/core/log_record.h"
#include "../src/core/mixer.h"
#include "../src/core/bmp280_math.h"
#include "../src/core/ms5611_math.h"
#include "../src/core/nlink_proto.h"
#include "../src/core/onboard.h"
#include "../src/core/pid.h"
#include "../src/core/position_ctrl.h"
#include "../src/core/quat.h"
#include "../src/core/thrust_model.h"
#include "../src/gains.h"
#include "../src/hal/pwm.h"          /* pwm_us_to_counts only: a portable inline (no AVR header) */
#include "../src/hal/baro.h"         /* fw09 --consts: BARO_LATENCY_S, the NAV's alt filter latency (macros only) */
#include "../src/hal/bmp280.h"       /* 2026-11-10: the BMP280 schedule macros (macros and types only) */
#include "../src/hal/ms5611.h"       /* the MS5611 schedule macros (the build option) */
#include "../src/params_gen.h"
#include "glink_vectors_gen.h"
#include "gps_vectors_gen.h"
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

/* the 15 TELEM v1 fields, the 2 TELEM v2 fields (phase, h_tgt_dm; 0 after lp_unpack_telem) and the 5 TELEM v3 fields
 * (gps_flags, num_sv, hacc_dm, pos_mode, dist_dm; 0 after lp_unpack_telem / _telem2) */
static void telem_fields(const lp_telem_t *t, long long f[LV_NF])
{
    f[0] = t->state; f[1] = t->flags; f[2] = t->reason; f[3] = t->ack_seq; f[4] = t->nonce; f[5] = (long long)t->t_ms;
    f[6] = t->h_dm; f[7] = t->vz_cms; f[8] = t->roll_cdeg; f[9] = t->pitch_cdeg; f[10] = t->yaw_cdeg;
    f[11] = t->tilt_cdeg; f[12] = t->throttle_milli; f[13] = t->vcell_mv; f[14] = t->loop_max_us;
    f[15] = t->phase; f[16] = t->h_tgt_dm;
    f[17] = t->gps_flags; f[18] = t->num_sv; f[19] = t->hacc_dm; f[20] = t->pos_mode; f[21] = t->dist_dm;
    f[22] = 0; f[23] = 0;                                  /* v0.9 RP1: the TELEM v4 / v5 tail (not in lp_telem_t) */
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
    CHECK(LP_CMD_FRAME_BYTES == 13 && LP_TELEM_FRAME_BYTES == 36 && LP_TELEM2_FRAME_BYTES == 39 &&
          LP_TELEM3_FRAME_BYTES == 45 && LP_TELEM4_FRAME_BYTES == 41 && LP_TELEM5_FRAME_BYTES == 47 && LP_LEN_MAX == 42 &&
          LP_FRAME_MAX_BYTES == 47 && LP_TYPE_TELEM4 == 0x23u && LP_TYPE_TELEM5 == 0x24u && LP_ACCEPT_TELEM4 == 16u &&
          LP_ACCEPT_TELEM5 == 32u,
          "frame sizes 13 / 36 / 39 (TELEM2) / 45 (TELEM3) / 41 (TELEM4) / 47 (TELEM5), LEN_MAX 42 (v0.6.3: 31, v0.7: 34, "
          "v0.8: 40)");
    {
        static const int vc[4] = LV_CMD_ON_OFF_UP_DOWN;
        CHECK(LV_LEN_MAX == LP_LEN_MAX && LV_TYPE_TELEM2 == LP_TYPE_TELEM2 && LV_TELEM2_FRAME_BYTES == LP_TELEM2_FRAME_BYTES &&
              vc[0] == LP_CMD_ON && vc[1] == LP_CMD_OFF && vc[2] == LP_CMD_UP && vc[3] == LP_CMD_DOWN,
              "v0.7 constants equal the reference's (LEN_MAX, TYPE_TELEM2, TELEM2 frame, ON / OFF / UP / DOWN)");
        CHECK(LV_TYPE_TELEM3 == LP_TYPE_TELEM3 && LV_TELEM3_FRAME_BYTES == LP_TELEM3_FRAME_BYTES &&
              LV_DIST_NONE == LP_DIST_NONE && LP_ACCEPT_TELEM3 == 8u, "v0.8 TELEM3: TYPE 0x22, 45-byte frame, dist 0xFFFF");
    }
    CHECK(LV_VEHICLE_ID == LINK_VEHICLE_ID, "vectors built for vehicle %d, params %d", LV_VEHICLE_ID, LINK_VEHICLE_ID);
    /* 2026-11-10: TELEM v6 / v7 (the logic-battery tail) and the COM's reply rule (lp_encode_vtol_reply) */
    CHECK(LP_TYPE_TELEM6 == 0x25u && LP_TYPE_TELEM7 == 0x26u && LP_TELEM6_FRAME_BYTES == 40 && LP_TELEM7_FRAME_BYTES == 46 &&
          LP_ACCEPT_TELEM6 == 64u && LP_ACCEPT_TELEM7 == 128u && LP_LBAT_SCALE == 50u && LP_LEN_MAX == 42,
          "TELEM6 0x25 / 40 B, TELEM7 0x26 / 46 B, accept bits 64 / 128, lbat 0.02 V, LEN_MAX still 42");
    {
        lp_telem_t t, u;
        uint8_t fr[LP_FRAME_MAX_BYTES], lb = 0, n;
        memset(&t, 0, sizeof t);
        t.state = 2u; t.flags = 0x2Fu; t.ack_seq = 4u; t.nonce = 0x1234u; t.t_ms = 5000u; t.h_dm = 15; t.vz_cms = 10;
        t.phase = 3u; t.h_tgt_dm = 15; t.gps_flags = 0x2Fu; t.num_sv = 9u; t.hacc_dm = 12u; t.pos_mode = 2u; t.dist_dm = 4u;
        n = lp_encode_vtol_reply(LINK_VEHICLE_ID, 7u, &t, false, 0u, 0u, 193u, fr);
        CHECK(n == LP_TELEM6_FRAME_BYTES && fr[4] == LP_TYPE_TELEM6 && fr[3u + 1u + LP_HEADER_BYTES + LP_TELEM2_PAYLOAD_LEN - 1u] == 193u,
              "rule: SNAP2 with lbat 193 -> TELEM6, the byte after the TELEM2 fields");
        lp_unpack_telem6(fr + 3u + LP_HEADER_BYTES, &u, &lb);
        CHECK(lb == 193u && u.phase == t.phase && u.h_tgt_dm == t.h_tgt_dm && u.nonce == t.nonce, "TELEM6 unpacks");
        n = lp_encode_vtol_reply(LINK_VEHICLE_ID, 7u, &t, true, 0u, 0u, 193u, fr);
        lp_unpack_telem7(fr + 3u + LP_HEADER_BYTES, &u, &lb);
        CHECK(n == LP_TELEM7_FRAME_BYTES && fr[4] == LP_TYPE_TELEM7 && lb == 193u && u.dist_dm == t.dist_dm,
              "rule: SNAP3 with lbat -> TELEM7");
        CHECK(lp_encode_vtol_reply(LINK_VEHICLE_ID, 7u, &t, true, 0u, 0u, LP_LBAT_NONE, fr) == LP_TELEM3_FRAME_BYTES &&
              fr[4] == LP_TYPE_TELEM3 &&
              lp_encode_vtol_reply(LINK_VEHICLE_ID, 7u, &t, false, 0u, 0u, LP_LBAT_NONE, fr) == LP_TELEM2_FRAME_BYTES &&
              fr[4] == LP_TYPE_TELEM2, "rule: no monitor (lbat 0) -> TELEM3 / TELEM2 unchanged");
        CHECK(lp_encode_vtol_reply(LINK_VEHICLE_ID, 7u, &t, true, 190u, 0x81u, 193u, fr) == LP_TELEM5_FRAME_BYTES &&
              fr[4] == LP_TYPE_TELEM5 &&
              lp_encode_vtol_reply(LINK_VEHICLE_ID, 7u, &t, false, 190u, 0x41u, 193u, fr) == LP_TELEM4_FRAME_BYTES &&
              fr[4] == LP_TYPE_TELEM4, "rule: a power build's tail wins over the logic battery (TELEM5 / TELEM4)");
    }
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
    {   /* TELEM v2: the 28 v1 bytes, then phase u8 and h_tgt i16 */
        lp_telem_t t, u;
        uint8_t pl[LP_TELEM2_PAYLOAD_LEN], p1[LP_TELEM_PAYLOAD_LEN], fr[LP_TELEM2_FRAME_BYTES];
        memset(&t, 0, sizeof t);
        t.state = 2; t.flags = 0x2F; t.nonce = 0x5A5A; t.t_ms = 21384; t.h_dm = 50; t.throttle_milli = 918;
        t.phase = LP_PHASE_HOLD; t.h_tgt_dm = -12345;
        lp_pack_telem2(&t, pl);
        lp_pack_telem(&t, p1);
        CHECK(memcmp(pl, p1, LP_TELEM_PAYLOAD_LEN) == 0 && pl[28] == LP_PHASE_HOLD && pl[29] == 0xC7 && pl[30] == 0xCF,
              "TELEM2 <BBBBHIhhhhhHHHHBh layout");
        lp_unpack_telem2(pl, &u);
        CHECK(telem_eq(&t, &u), "TELEM2 unpack(pack(t)) == t");
        lp_unpack_telem(pl, &u);
        CHECK(u.phase == 0 && u.h_tgt_dm == 0 && u.throttle_milli == 918, "lp_unpack_telem: v1 fields, phase / h_tgt 0");
        CHECK(lp_encode_telem2(LINK_VEHICLE_ID, 9, &t, fr) == LP_TELEM2_FRAME_BYTES && fr[2] == 34 &&
              fr[4] == LP_TYPE_TELEM2 && fr[5] == 9, "TELEM2 frame header (LEN 34, TYPE 0x21)");
    }
    {   /* TELEM v3 (v0.8): the 31 v2 bytes, then gps_flags, num_sv, hacc_dm, pos_mode u8, dist_dm u16 */
        lp_telem_t t, u;
        uint8_t pl[LP_TELEM3_PAYLOAD_LEN], p2[LP_TELEM2_PAYLOAD_LEN], fr[LP_TELEM3_FRAME_BYTES];
        memset(&t, 0, sizeof t);
        t.state = 2; t.flags = 0x2F; t.nonce = 0x5A5A; t.t_ms = 21384; t.h_dm = 15; t.throttle_milli = 918;
        t.phase = LP_PHASE_HOLD; t.h_tgt_dm = 15;
        t.gps_flags = 0x2F; t.num_sv = 9; t.hacc_dm = 25; t.pos_mode = 2; t.dist_dm = 0xBEEF;
        lp_pack_telem3(&t, pl);
        lp_pack_telem2(&t, p2);
        CHECK(memcmp(pl, p2, LP_TELEM2_PAYLOAD_LEN) == 0 && pl[31] == 0x2F && pl[32] == 9 && pl[33] == 25 && pl[34] == 2 &&
              pl[35] == 0xEF && pl[36] == 0xBE, "TELEM3 <...BhBBBBH layout");
        lp_unpack_telem3(pl, &u);
        CHECK(telem_eq(&t, &u), "TELEM3 unpack(pack(t)) == t");
        lp_unpack_telem2(pl, &u);
        CHECK(u.gps_flags == 0 && u.dist_dm == 0 && u.phase == LP_PHASE_HOLD, "lp_unpack_telem2: v2 fields, the GPS ones 0");
        CHECK(lp_encode_telem3(LINK_VEHICLE_ID, 11, &t, fr) == LP_TELEM3_FRAME_BYTES && fr[2] == 40 &&
              fr[4] == LP_TYPE_TELEM3 && fr[5] == 11, "TELEM3 frame header (LEN 40, TYPE 0x22)");
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
            if (e->kind >= 2 && e->kind <= 8) {
                lp_telem_t t;
                long long f[LV_NF];
                uint8_t va = 0, pf = 0;
                int q;
                if (e->kind == 2)
                    lp_unpack_telem(all[j].payload, &t);
                else if (e->kind == 3)
                    lp_unpack_telem2(all[j].payload, &t);
                else if (e->kind == 4)
                    lp_unpack_telem3(all[j].payload, &t);
                else if (e->kind == 5)                     /* v0.9 RP1: TELEM v4 / v5 */
                    lp_unpack_telem4(all[j].payload, &t, &va, &pf);
                else if (e->kind == 6)
                    lp_unpack_telem5(all[j].payload, &t, &va, &pf);
                else if (e->kind == 7)                     /* 2026-11-10: TELEM v6 / v7 (lbat in f[22]) */
                    lp_unpack_telem6(all[j].payload, &t, &va);
                else
                    lp_unpack_telem7(all[j].payload, &t, &va);
                telem_fields(&t, f);
                f[22] = va;
                f[23] = pf;
                for (q = 0; q < LV_NF; q++)
                    CHECK(f[q] == e->f[q], "vector %s telem field %d: %lld vs %lld", v->name, q, f[q], e->f[q]);
                {
                    static const uint8_t ty[9] = {0, 0, LP_TYPE_TELEM, LP_TYPE_TELEM2, LP_TYPE_TELEM3, LP_TYPE_TELEM4,
                                                  LP_TYPE_TELEM5, LP_TYPE_TELEM6, LP_TYPE_TELEM7};
                    static const uint8_t ln[9] = {0, 0, LP_TELEM_PAYLOAD_LEN, LP_TELEM2_PAYLOAD_LEN, LP_TELEM3_PAYLOAD_LEN,
                                                  LP_TELEM4_PAYLOAD_LEN, LP_TELEM5_PAYLOAD_LEN, LP_TELEM6_PAYLOAD_LEN,
                                                  LP_TELEM7_PAYLOAD_LEN};
                    CHECK(all[j].type == ty[e->kind] && all[j].len == ln[e->kind], "vector %s telem TYPE / LEN", v->name);
                }
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
    if (type == IL_TYPE_SNAP3) {                       /* v0.8: the SNAP fields + phase, gps_flags, h_tgt, sv, hacc, mode, dist */
        il_snap3_t s3;
        il_snap_t *s = &s3.s;
        il_unpack_snap3(pl, &s3);
        f[0] = s->reply_seq; f[1] = s->state; f[2] = s->flags; f[3] = s->reason; f[4] = s->ack_seq; f[5] = s->nonce;
        f[6] = s->throttle_milli; f[7] = (long long)s->tick; f[8] = il_bits(s->h_m); f[9] = il_bits(s->vz_mps);
        f[10] = il_bits(s->tilt_deg); f[11] = il_bits(s->v_cell); f[12] = il_bits(s->loop_max_us);
        for (i = 0; i < 4; i++) f[13 + i] = il_bits(s->q[i]);
        f[17] = s3.phase; f[18] = s3.gps_flags; f[19] = il_bits(s3.h_tgt_m); f[20] = s3.num_sv; f[21] = s3.hacc_dm;
        f[22] = s3.pos_mode; f[23] = il_bits(s3.dist_m);
        f[24] = s3.pw_vaux; f[25] = s3.pw_flags; f[26] = s3.spare3;    /* v0.9 RP1 */
        return 27;
    }
    if (type == IL_TYPE_CMD) {
        il_cmd_t c;
        il_unpack_cmd(pl, &c);
        f[0] = c.seq; f[1] = c.cmd.cmd; f[2] = c.cmd.arg; f[3] = c.cmd.nonce; f[4] = c.rssi_dbm; f[5] = c.snr_q4;
        return 6;
    }
    if (type == IL_TYPE_SNAP || type == IL_TYPE_SNAP2) {
        il_snap2_t s2;
        il_snap_t s;
        if (type == IL_TYPE_SNAP2) {
            il_unpack_snap2(pl, &s2);
            s = s2.s;
        } else {
            il_unpack_snap(pl, &s);
        }
        f[0] = s.reply_seq; f[1] = s.state; f[2] = s.flags; f[3] = s.reason; f[4] = s.ack_seq; f[5] = s.nonce;
        f[6] = s.throttle_milli; f[7] = (long long)s.tick; f[8] = il_bits(s.h_m); f[9] = il_bits(s.vz_mps);
        f[10] = il_bits(s.tilt_deg); f[11] = il_bits(s.v_cell); f[12] = il_bits(s.loop_max_us);
        for (i = 0; i < 4; i++) f[13 + i] = il_bits(s.q[i]);
        if (type == IL_TYPE_SNAP)
            return 17;
        f[17] = s2.phase; f[18] = s2.spare; f[19] = il_bits(s2.h_tgt_m);
        f[20] = s2.pw_vaux; f[21] = s2.pw_flags;                       /* v0.9 RP1 */
        return 22;
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
    if (type == IL_TYPE_SNAP3) {
        il_snap3_t s3;
        il_snap_t *s = &s3.s;
        s->reply_seq = (uint8_t)f[0]; s->state = (uint8_t)f[1]; s->flags = (uint8_t)f[2]; s->reason = (uint8_t)f[3];
        s->ack_seq = (uint8_t)f[4]; s->nonce = (uint16_t)f[5]; s->throttle_milli = (uint16_t)f[6]; s->tick = (uint32_t)f[7];
        s->h_m = il_from_bits(f[8]); s->vz_mps = il_from_bits(f[9]); s->tilt_deg = il_from_bits(f[10]);
        s->v_cell = il_from_bits(f[11]); s->loop_max_us = il_from_bits(f[12]);
        for (i = 0; i < 4; i++) s->q[i] = il_from_bits(f[13 + i]);
        s3.phase = (uint8_t)f[17]; s3.gps_flags = (uint8_t)f[18]; s3.h_tgt_m = il_from_bits(f[19]);
        s3.num_sv = (uint8_t)f[20]; s3.hacc_dm = (uint8_t)f[21]; s3.pos_mode = (uint8_t)f[22]; s3.dist_m = il_from_bits(f[23]);
        s3.pw_vaux = (uint8_t)f[24]; s3.pw_flags = (uint8_t)f[25]; s3.spare3 = (uint8_t)f[26];
        il_pack_snap3(&s3, out);
        return;
    }
    if (type == IL_TYPE_CMD) {
        il_cmd_t c;
        c.seq = (uint8_t)f[0]; c.cmd.cmd = (uint8_t)f[1]; c.cmd.arg = (uint16_t)f[2]; c.cmd.nonce = (uint16_t)f[3];
        c.rssi_dbm = (int8_t)f[4]; c.snr_q4 = (int8_t)f[5];
        il_pack_cmd(&c, out);
    } else if (type == IL_TYPE_SNAP || type == IL_TYPE_SNAP2) {
        il_snap2_t s2;
        il_snap_t *s = &s2.s;
        s->reply_seq = (uint8_t)f[0]; s->state = (uint8_t)f[1]; s->flags = (uint8_t)f[2]; s->reason = (uint8_t)f[3];
        s->ack_seq = (uint8_t)f[4]; s->nonce = (uint16_t)f[5]; s->throttle_milli = (uint16_t)f[6]; s->tick = (uint32_t)f[7];
        s->h_m = il_from_bits(f[8]); s->vz_mps = il_from_bits(f[9]); s->tilt_deg = il_from_bits(f[10]);
        s->v_cell = il_from_bits(f[11]); s->loop_max_us = il_from_bits(f[12]);
        for (i = 0; i < 4; i++) s->q[i] = il_from_bits(f[13 + i]);
        if (type == IL_TYPE_SNAP) {
            il_pack_snap(s, out);
        } else {
            s2.phase = (uint8_t)f[17];
            s2.spare = (uint8_t)f[18];
            s2.h_tgt_m = il_from_bits(f[19]);
            s2.pw_vaux = (uint8_t)f[20];
            s2.pw_flags = (uint8_t)f[21];
            il_pack_snap2(&s2, out);
        }
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
    } else if (type == IL_TYPE_SNAP2) {
        il_snap2_t s;
        il_unpack_snap2(pl, &s);
        il_pack_snap2(&s, out);
    } else if (type == IL_TYPE_SNAP3) {
        il_snap3_t s;
        il_unpack_snap3(pl, &s);
        il_pack_snap3(&s, out);
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
    CHECK(IL_CMD_FRAME_BYTES == 14 && IL_SNAP_FRAME_BYTES == 55 && IL_STAT_FRAME_BYTES == 42 && IL_SNAP2_FRAME_BYTES == 63 &&
          IL_SNAP3_FRAME_BYTES == 71 && IL_FRAME_MAX_BYTES == 71 && ILV_FRAME_MAX_BYTES == IL_FRAME_MAX_BYTES,
          "ilink frame sizes 14 / 55 / 42 / 63 (SNAP2) / 71 (SNAP3), max 71 (v0.9 RP1)");
    CHECK(ILV_TYPE_SNAP3 == IL_TYPE_SNAP3 && ILV_SNAP3_LEN == IL_SNAP3_LEN && IL_SNAP3_LEN == 65u &&
          ILV_ACCEPT_SNAP3 == IL_ACCEPT_SNAP3 && ILV_ACCEPT_ALL8 == IL_ACCEPT_ALL8 && ILV_ACCEPT_COM_GPS == IL_ACCEPT_COM_GPS &&
          IL_ACCEPT_SNAP3 == 1u << (IL_TYPE_SNAP3 - 1u) && IL_ACCEPT_ALL8 == 31u, "v0.8 SNAP3: TYPE 5, LEN 65 (v0.9), accept 16, ALL8 31");
    CHECK(ILV_TYPE_SNAP2 == IL_TYPE_SNAP2 && ILV_SNAP2_LEN == IL_SNAP2_LEN && IL_SNAP2_LEN == 57u &&
          ILV_ACCEPT_SNAP2 == IL_ACCEPT_SNAP2 && ILV_ACCEPT_COM_VTOL == IL_ACCEPT_COM_VTOL && ILV_ACCEPT_ALL == IL_ACCEPT_ALL &&
          IL_ACCEPT_SNAP2 == 1u << (IL_TYPE_SNAP2 - 1u) && IL_ACCEPT_COM_VTOL == IL_ACCEPT_SNAP2 && IL_ACCEPT_ALL == 15u,
          "v0.7 SNAP2: TYPE 4, LEN 57 (v0.9), accept bit 8 = the VTOL COM's mask, ALL = 15");
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
                                 (k == IL_TYPE_SNAP2 ? IL_SNAP2_LEN : (k == IL_TYPE_SNAP3 ? IL_SNAP3_LEN : 0))));
            if (il_payload_len((uint8_t)k) != want) bad++;
        }
        CHECK(bad == 0, "il_payload_len: CMD 8, SNAP 49, STAT 36, SNAP2 57, SNAP3 65, every other TYPE 0 (%d wrong)", bad);
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
              il_encode(LP_TYPE_CMD, pl, 5, fr) == 0 && il_encode(IL_TYPE_SNAP2, pl, IL_SNAP_LEN, fr) == 0 &&
              il_encode(IL_TYPE_SNAP3, pl, IL_SNAP2_LEN, fr) == 0, "unknown TYPEs, SNAP2 / SNAP3 with a shorter length refused");
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
        uint8_t pl[IL_MAX_PAYLOAD], a[4][IL_FRAME_MAX_BYTES], b[IL_CMD_FRAME_BYTES], s[2 * IL_FRAME_MAX_BYTES];
        static const uint8_t types[4] = {IL_TYPE_CMD, IL_TYPE_STAT, IL_TYPE_SNAP, IL_TYPE_SNAP3};
        long cases = 0, lost = 0, wrong = 0;
        uint8_t na[4], t;
        (void)il_encode(IL_TYPE_CMD, cmd_b, IL_CMD_LEN, b);
        for (t = 0; t < 4; t++) {
            uint8_t q;
            for (q = 0; q < IL_MAX_PAYLOAD; q++)
                pl[q] = (uint8_t)(q * 29u + 7u + t);                       /* no 0xC5 5C pair inside */
            na[t] = il_encode(types[t], pl, il_payload_len(types[t]), a[t]);
        }
        for (t = 0; t < 4; t++) {
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
                        il_decoder_init(&dec, IL_ACCEPT_ALL8);
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
        CHECK(lost == 0, "single-byte errors (substituted / dropped / inserted) in CMD / STAT / SNAP / SNAP3 frames: %ld cases, "
              "next CMD lost %ld", cases, lost);
        CHECK(wrong <= 3, "single-byte errors: %ld corrupted frames passed the CRC (expected ~0.7 of the ~45000 inserts)", wrong);
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
         * is known, accepted and LEN is its length; bits outside IL_ACCEPT_ALL are ignored; the COM never has an IL_CMD
         * to wait for, a mask without CMD reports IL_NEED_NEVER */
        static const uint8_t masks[] = {0u, IL_ACCEPT_CMD, IL_ACCEPT_SNAP, IL_ACCEPT_STAT, IL_ACCEPT_FCU, IL_ACCEPT_COM,
                                        IL_ACCEPT_CMD | IL_ACCEPT_SNAP, IL_ACCEPT_ALL, 0xE0u | IL_ACCEPT_FCU, IL_ACCEPT_SNAP2,
                                        IL_ACCEPT_COM_VTOL, IL_ACCEPT_SNAP | IL_ACCEPT_SNAP2, 0xE8u | IL_ACCEPT_FCU,
                                        IL_ACCEPT_SNAP3, IL_ACCEPT_COM_GPS, IL_ACCEPT_ALL8, IL_ACCEPT_SNAP2 | IL_ACCEPT_SNAP3,
                                        0xE0u | IL_ACCEPT_SNAP3};
        int m, ln, ty, bad = 0;
        for (m = 0; m < (int)sizeof masks; m++) {
            const uint8_t a = (uint8_t)(masks[m] & IL_ACCEPT_ALL8);
            for (ln = 0; ln < 256; ln++) {
                for (ty = 0; ty < 256; ty += (ln == IL_CMD_LEN || ln == IL_STAT_LEN || ln == IL_SNAP_LEN || ln == IL_SNAP2_LEN ||
                                              ln == IL_SNAP3_LEN) ? 1 : 64) {
                    const uint8_t hdr[4] = {IL_SYNC0, IL_SYNC1, (uint8_t)ln, (uint8_t)ty};
                    const uint8_t want_len = il_payload_len((uint8_t)ty);
                    const int pass = want_len != 0u && want_len == ln && (a & (1u << (ty - 1)));
                    const int len_ok = (ln == IL_CMD_LEN && (a & IL_ACCEPT_CMD)) || (ln == IL_STAT_LEN && (a & IL_ACCEPT_STAT)) ||
                                       (ln == IL_SNAP_LEN && (a & IL_ACCEPT_SNAP)) || (ln == IL_SNAP2_LEN && (a & IL_ACCEPT_SNAP2)) ||
                                       (ln == IL_SNAP3_LEN && (a & IL_ACCEPT_SNAP3));
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
                                          (a & IL_ACCEPT_SNAP2) ? IL_SNAP2_FRAME_BYTES - 1u : (a & IL_ACCEPT_SNAP3) ?
                                          IL_SNAP3_FRAME_BYTES - 1u : IL_NEED_NEVER))
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
        CHECK(!back.v2 && back.phase == 0 && back.h_tgt_m == 0.0f, "SNAP -> a v1 source (no phase / target)");
        {   /* SNAP2 (VTOL): the SNAP fields + phase, spare 0, h_tgt_m; back to a v2 source */
            il_snap2_t s2, u2;
            uint8_t pl[IL_SNAP2_LEN], p1[IL_SNAP_LEN];
            src.v2 = true;
            src.phase = LP_PHASE_LAND;
            src.h_tgt_m = 7.5f;
            src.pw_vaux = 201;                         /* v0.9 RP1: a wired_buffer source's tail */
            src.pw_flags = 0x89;
            il_snap2_from_tlm_src(43, &src, q, &s2);
            CHECK(s2.s.reply_seq == 43 && s2.s.h_m == src.h_m && s2.phase == LP_PHASE_LAND && s2.spare == 0 &&
                  s2.h_tgt_m == 7.5f && s2.pw_vaux == 201 && s2.pw_flags == 0x89,
                  "il_snap2_from_tlm_src: SNAP fields + phase, spare 0, target, power tail");
            il_pack_snap2(&s2, pl);
            il_pack_snap(&s2.s, p1);
            CHECK(memcmp(pl, p1, IL_SNAP_LEN) == 0 && pl[49] == LP_PHASE_LAND && pl[50] == 0 && pl[51] == 0x00 && pl[52] == 0x00 &&
                  pl[53] == 0xF0 && pl[54] == 0x40 && pl[55] == 201 && pl[56] == 0x89,
                  "SNAP2 layout: the 49 SNAP bytes, phase, spare, h_tgt_m f32, pw_vaux, pw_flags");
            il_unpack_snap2(pl, &u2);
            memset(&back, 0, sizeof back);
            il_snap2_to_tlm_src(&u2, &back);
            CHECK(back.v2 && back.phase == LP_PHASE_LAND && back.h_tgt_m == 7.5f && back.h_m == src.h_m &&
                  back.nonce == src.nonce && back.pw_vaux == 201 && back.pw_flags == 0x89,
                  "il_snap2_to_tlm_src: a v2 source with the power tail");
            u2.spare = 0xAB;
            il_snap2_to_tlm_src(&u2, &back);
            CHECK(back.v2 && back.phase == LP_PHASE_LAND, "the spare byte is ignored by the COM");
        }
        {   /* SNAP3 (v0.8 VTOL + GPS): SNAP2 with spare = gps_flags, then sv, hacc, mode, dist f32; back to a v3 source */
            il_snap3_t s3, u3;
            uint8_t pl[IL_SNAP3_LEN], p2[IL_SNAP2_LEN];
            il_snap2_t s2;
            src.v3 = true;
            src.gps_flags = 0x2F;
            src.num_sv = 9;
            src.hacc_dm = 25;
            src.pos_mode = POS_MODE_POS;
            src.dist_m = 0.75f;
            il_snap3_from_tlm_src(44, &src, q, &s3);
            il_snap2_from_tlm_src(44, &src, q, &s2);
            s2.spare = 0x2F;
            il_pack_snap3(&s3, pl);
            il_pack_snap2(&s2, p2);
            CHECK(memcmp(pl, p2, IL_SNAP_LEN + 6u) == 0 && pl[55] == 9 && pl[56] == 25 && pl[57] == POS_MODE_POS &&
                  pl[58] == 0x00 && pl[59] == 0x00 && pl[60] == 0x40 && pl[61] == 0x3F && pl[62] == 201 && pl[63] == 0x89 &&
                  pl[64] == 0, "SNAP3 layout: the SNAP2 fields before the tail with gps_flags as spare, sv, hacc, mode, "
                  "dist_m f32, pw_vaux, pw_flags, spare3 0 (v0.9)");
            il_unpack_snap3(pl, &u3);
            memset(&back, 0, sizeof back);
            il_snap3_to_tlm_src(&u3, &back);
            CHECK(back.v2 && back.v3 && back.gps_flags == 0x2F && back.num_sv == 9 && back.hacc_dm == 25 &&
                  back.pos_mode == POS_MODE_POS && back.dist_m == 0.75f && back.phase == LP_PHASE_LAND &&
                  back.pw_vaux == 201 && back.pw_flags == 0x89, "il_snap3_to_tlm_src: a v3 source with the power tail");
        }
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
        uint8_t fr[LP_FRAME_MAX_BYTES], n, seq, pva = 0, pfl = 0, lbat = LP_LBAT_NONE;
        int j, a, other = 0, ang = 0;
        memset(&src, 0, sizeof src);
        if (v->snap_n == IL_SNAP3_LEN) {               /* VTOL + GPS COM: SNAP3 -> TELEM3 (v0.9: TELEM5 with a tail) */
            il_snap3_t s;
            il_unpack_snap3(v->snap, &s);
            il_snap3_to_tlm_src(&s, &src);
            seq = s.s.reply_seq;
            pva = s.pw_vaux; pfl = s.pw_flags; lbat = s.spare3;   /* 2026-11-10: + the logic battery (TELEM7) */
        } else if (v->snap_n == IL_SNAP2_LEN) {        /* VTOL COM: SNAP2 -> TELEM2 */
            il_snap2_t s;
            il_unpack_snap2(v->snap, &s);
            il_snap2_to_tlm_src(&s, &src);
            seq = s.s.reply_seq;
            pva = s.pw_vaux; pfl = s.pw_flags; lbat = s.spare;    /* 2026-11-10: + the logic battery (TELEM6) */
        } else {
            il_snap_t s;
            CHECK(v->snap_n == IL_SNAP_LEN, "telem %s: snapshot length %u", v->name, v->snap_n);
            il_unpack_snap(v->snap, &s);
            il_snap_to_tlm_src(&s, &src);
            seq = s.reply_seq;
        }
        if (v->snap_n == IL_SNAP_LEN) {
            n = onboard_telemetry_frame(&src, ILV_LOOP_HZ, ILV_VEHICLE_ID, seq, fr);
        } else {                                       /* the VTOL COM's reply (com/com_link.c: lp_encode_vtol_reply) */
            lp_telem_t t;
            onboard_telemetry(&src, ILV_LOOP_HZ, &t);
            n = lp_encode_vtol_reply(ILV_VEHICLE_ID, seq, &t, v->snap_n == IL_SNAP3_LEN, pva, pfl, lbat, fr);
        }
        CHECK(n == v->telem_n && n == (v->snap_n == IL_SNAP3_LEN ? (LP_PWR_TAIL_DUE(pfl) ? LP_TELEM5_FRAME_BYTES :
                                                                    LP_LBAT_TAIL_DUE(lbat, pfl) ? LP_TELEM7_FRAME_BYTES :
                                                                    LP_TELEM3_FRAME_BYTES) :
                                       v->snap_n == IL_SNAP2_LEN ? (LP_PWR_TAIL_DUE(pfl) ? LP_TELEM4_FRAME_BYTES :
                                                                    LP_LBAT_TAIL_DUE(lbat, pfl) ? LP_TELEM6_FRAME_BYTES :
                                                                    LP_TELEM2_FRAME_BYTES) : LP_TELEM_FRAME_BYTES),
              "telem %s: frame length %u (want %u)", v->name, n, v->telem_n);
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
    CHECK(NL_FCU_THR_SCALE == NLV_FCU_THR_SCALE, "v0.7.1 thr_q scale %u == the reference", NL_FCU_THR_SCALE);
    for (bad = 0, i = 0; i < NLV_N_THR; i++)
        if (nl_fcu_throttle_q((float)NLV_THR[i].u) != NLV_THR[i].thr_q) bad++;
    CHECK(bad == 0, "v0.7.1 thr_q quantisation == sim/nlink_proto.py fcu_throttle_q on %d values", NLV_N_THR);
    for (bad = 0, i = 0; i < 256; i++)
        if (nl_fcu_throttle((uint8_t)i) != (float)(i < 250 ? i : 250) / 250.0f) bad++;
    CHECK(bad == 0, "nl_fcu_throttle(thr_q) = min(thr_q, 250) / 250");
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
    bool td, at;                   /* VTOL: vc_touchdown / vc_at_target inputs */
    float thr_prev;                /* VTOL: throttle_prev input */
    bool gsig;                     /* v0.7.1: vc_ground_signature input */
    uint16_t lil;                  /* v0.7.1: vc_low_in_land_ticks input */
    bool gate;                     /* v0.8: gps_gate_ok input */
    uint8_t pcr;                   /* v0.8: pc_requests input */
    bool tof;                      /* v0.8: vc_takeoff_fail input */
    onb_tlm_gps_t tg;              /* v0.8: TELEM v3 GPS fields */
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
    in.vc_touchdown = h->td;
    in.vc_at_target = h->at;
    in.throttle_prev = h->thr_prev;
    in.vc_ground_signature = h->gsig;
    in.vc_low_in_land_ticks = h->lil;
    in.cos_tilt = 1.0f;
    in.accel_z_mps2 = NAN;
    in.gps_slot = 0;
    in.gps_gate_ok = h->gate;
    in.pc_requests = h->pcr;
    in.vc_takeoff_fail = h->tof;
    in.tlm_gps = h->tg;
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

/* ------------------------------------------------------------------------ */
/* VTOL build (SPEC 8.9, v0.7): supervisor tables and rules, vertical      */
/* controller, the FCU stack                                                */
/* ------------------------------------------------------------------------ */
/* the v0.7.1 VTOL build (gps_hold / takeoff_check off): the v0.7 / v0.7.1 tables and rules below; v0.8 tests set them */
static void vh_init_p(oh_t *h, const onb_params_t *p0, uint16_t seed, bool reset_in_flight)
{
    onb_params_t p = *p0;
    p.mode = LP_MODE_VTOL;
    p.gps_hold = false;
    p.takeoff_check = false;
    oh_init_p(h, &p, seed, reset_in_flight);
}

static void vh_init(oh_t *h)
{
    onb_params_t p;
    onb_params_mode(&p, LP_MODE_VTOL);
    p.gps_hold = false;
    p.takeoff_check = false;
    oh_init_p(h, &p, 1, false);
}

/* VTOL start points: SAFE, SPOOL, CLIMB, HOLD, LAND, LANDED, ABORT (release pending), RELEASED */
static void vh_to(oh_t *h, const char *st)
{
    oh_hb(h);
    oh_tick(h);
    if (!strcmp(st, "SAFE")) return;
    CHECK(oh_send(h, LP_CMD_ON, 0)->state == ONB_ARMED && h->o.phase == LP_PHASE_SPOOL, "setup ON (%s)", st);
    if (!strcmp(st, "SPOOL")) return;
    oh_run(h, h->p.spool_ticks, 20);
    CHECK(h->s.state == ONB_POWERED && h->s.phase == LP_PHASE_CLIMB, "setup spool -> CLIMB (%s)", st);
    if (!strcmp(st, "CLIMB")) return;
    if (!strcmp(st, "ABORT") || !strcmp(st, "RELEASED")) {
        CHECK(oh_send(h, LP_CMD_KILL, 0)->state == ONB_ABORT, "setup KILL (%s)", st);
        if (!strcmp(st, "RELEASED")) {
            oh_run(h, h->p.abort_release_ticks, 20);
            CHECK(h->s.state == ONB_RELEASED, "setup RELEASED");
        }
        return;
    }
    h->at = true;
    oh_hb(h);
    h->at = false;
    CHECK(h->s.phase == LP_PHASE_HOLD, "setup at_target -> HOLD (%s)", st);
    if (!strcmp(st, "HOLD")) return;
    CHECK(oh_send(h, LP_CMD_OFF, 0)->phase == LP_PHASE_LAND, "setup OFF -> LAND (%s)", st);
    if (!strcmp(st, "LAND")) return;
    h->td = true;
    oh_hb(h);
    h->td = false;
    CHECK(h->s.state == ONB_UNPOWERED && h->s.phase == LP_PHASE_LANDED, "setup touchdown -> LANDED (%s)", st);
}

static void test_onboard_vtol_table(void)
{
    /* SPEC 8.9 command table: start (state, phase) x command -> (state, phase, code) */
    enum { NC = 13, NS = 8 };
    static const struct { uint8_t cmd; bool good_nonce; const char *name; } V[NC] = {
        {LP_CMD_ON, true, "ON"}, {LP_CMD_ON, false, "ON_badnonce"}, {LP_CMD_OFF, true, "OFF"}, {LP_CMD_UP, true, "UP"},
        {LP_CMD_DOWN, true, "DOWN"}, {LP_CMD_KILL, true, "KILL"}, {LP_CMD_ARM, true, "ARM"}, {LP_CMD_DISARM, true, "DISARM"},
        {LP_CMD_LAUNCH, true, "LAUNCH"}, {LP_CMD_THROTTLE, true, "THROTTLE"}, {LP_CMD_CUT, true, "CUT"},
        {LP_CMD_RELEASE, true, "RELEASE"}, {12, true, "UNKNOWN_12"}};
    static const char *starts[NS] = {"SAFE", "SPOOL", "CLIMB", "HOLD", "LAND", "LANDED", "ABORT", "RELEASED"};
    enum { S_ = ONB_SAFE, A_ = ONB_ARMED, P_ = ONB_POWERED, U_ = ONB_UNPOWERED, R_ = ONB_RELEASED, AB_ = ONB_ABORT };
    enum { N0 = LP_PHASE_NONE, SP = LP_PHASE_SPOOL, CL = LP_PHASE_CLIMB, HO = LP_PHASE_HOLD, LA = LP_PHASE_LAND,
           LD = LP_PHASE_LANDED };
    const uint8_t RS = LP_REJ_STATE, RN = LP_REJ_NONCE, RP = LP_REJ_RELEASE_PENDING, RU = LP_REJ_UNKNOWN_CMD, RM = LP_REJ_MODE,
                  RL = LP_REJ_LANDING;
    /* {state, phase, code} per start for ON, ON_bad, OFF, UP, DOWN, KILL; the six ballistic commands: REJ_MODE, no change;
     * code 12: REJ_UNKNOWN_CMD, no change */
    static uint8_t want[NS][6][3];
    const uint8_t w_safe[6][3] = {{A_, SP, 0}, {S_, N0, RN}, {S_, N0, RS}, {S_, N0, RS}, {S_, N0, RS}, {S_, N0, RS}};
    const uint8_t w_spool[6][3] = {{A_, SP, RS}, {A_, SP, RS}, {S_, N0, 0}, {A_, SP, RS}, {A_, SP, RS}, {S_, N0, 0}};
    const uint8_t w_climb[6][3] = {{P_, CL, RS}, {P_, CL, RS}, {P_, LA, 0}, {P_, CL, 0}, {P_, CL, 0}, {AB_, N0, 0}};
    const uint8_t w_hold[6][3] = {{P_, HO, RS}, {P_, HO, RS}, {P_, LA, 0}, {P_, HO, 0}, {P_, HO, 0}, {AB_, N0, 0}};
    const uint8_t w_land[6][3] = {{P_, LA, RS}, {P_, LA, RS}, {P_, LA, 0}, {P_, LA, RL}, {P_, LA, RL}, {AB_, N0, 0}};
    const uint8_t w_landed[6][3] = {{U_, LD, RS}, {U_, LD, RS}, {U_, LD, 0}, {U_, LD, RS}, {U_, LD, RS}, {AB_, N0, 0}};
    const uint8_t w_abort[6][3] = {{AB_, N0, RP}, {AB_, N0, RP}, {AB_, N0, RP}, {AB_, N0, RP}, {AB_, N0, RP}, {AB_, N0, 0}};
    const uint8_t w_rel[6][3] = {{R_, N0, RS}, {R_, N0, RS}, {R_, N0, RS}, {R_, N0, RS}, {R_, N0, RS}, {R_, N0, 0}};
    const uint8_t (*tabs[NS])[3] = {w_safe, w_spool, w_climb, w_hold, w_land, w_landed, w_abort, w_rel};
    int a, c, j;
    for (a = 0; a < NS; a++)
        for (c = 0; c < 6; c++)
            for (j = 0; j < 3; j++) want[a][c][j] = tabs[a][c][j];
    for (a = 0; a < NS; a++) {
        for (c = 0; c < NC; c++) {
            oh_t h;
            const onb_outputs_t *o;
            uint16_t n0;
            int16_t tgt0;
            uint8_t ws, wp, wc;
            lp_telem_t t;
            vh_init(&h);
            vh_to(&h, starts[a]);
            n0 = h.s.nonce;
            tgt0 = h.s.tgt_dm;
            if (c < 6) {
                ws = want[a][c][0]; wp = want[a][c][1]; wc = want[a][c][2];
            } else {
                ws = h.s.state; wp = h.s.phase; wc = c == NC - 1 ? RU : RM;
            }
            h.seq++;
            o = oh_frame(&h, h.seq, V[c].cmd, 500, V[c].good_nonce ? n0 : (uint16_t)(n0 ^ 0x5A5Au));
            CHECK(o->state == ws && o->phase == wp && h.last_code == wc,
                  "VTOL %s + %s -> %s phase %u code 0x%02X (want %s phase %u 0x%02X)", starts[a], V[c].name,
                  state_name(o->state), o->phase, h.last_code, state_name(ws), wp, wc);
            CHECK(o->reply && o->reply_seq == h.seq && o->ack_seq == h.seq, "VTOL %s + %s acknowledged", starts[a], V[c].name);
            CHECK(o->n_exec == 1 && o->exec[0].phase_before == (a == 0 ? N0 : a == 1 ? SP : a == 2 ? CL : a == 3 ? HO :
                                                                  a == 4 ? LA : a == 5 ? LD : N0) &&
                  o->exec[0].phase_after == o->phase, "VTOL %s + %s exec phases", starts[a], V[c].name);
            onboard_telemetry(&o->tlm, FC_LOOP_HZ, &t);
            CHECK(o->tlm.v2 && t.phase == o->phase && ((t.flags & LP_FLAG_REJECTED) != 0) == (wc != 0),
                  "VTOL %s + %s: TELEM2 phase and REJECTED flag", starts[a], V[c].name);
            CHECK(h.s.nonce == ((V[c].cmd == LP_CMD_ON && wc == 0) ? onb_lfsr16_next(n0) : n0), "VTOL %s + %s nonce",
                  starts[a], V[c].name);
            if ((a == 2 || a == 3) && wc == 0 && (V[c].cmd == LP_CMD_UP || V[c].cmd == LP_CMD_DOWN))
                CHECK(h.s.tgt_dm == tgt0 + (V[c].cmd == LP_CMD_UP ? 1 : -1) * h.p.alt_step_dm && o->h_tgt_dm == h.s.tgt_dm &&
                      t.h_tgt_dm == h.s.tgt_dm, "VTOL %s + %s: target %d -> %d", starts[a], V[c].name, tgt0, h.s.tgt_dm);
            else if (!(a == 0 && c == 0))
                CHECK(h.s.tgt_dm == tgt0, "VTOL %s + %s: target unchanged", starts[a], V[c].name);
            CHECK(o->target_milli == 0 && o->throttle_milli == 0, "VTOL: the supervisor's own throttle stays 0");
            CHECK(o->motor_on == (o->state == ONB_POWERED) && o->control_active == (o->state == ONB_POWERED) &&
                  o->buzzer == (o->state == ONB_ARMED), "VTOL %s + %s: motor_on / control / buzzer", starts[a], V[c].name);
        }
    }
    {   /* the ballistic build answers 8..11 as v0.6.3 did, in every state */
        static const char *bst[4] = {"SAFE", "ARMED", "POWERED", "UNPOWERED"};
        for (a = 0; a < 4; a++) {
            for (c = (int)LP_CMD_ON; c <= (int)LP_CMD_DOWN; c++) {
                oh_t h;
                uint8_t st;
                oh_init(&h);
                oh_to(&h, bst[a]);
                st = h.s.state;
                oh_send(&h, (uint8_t)c, 0);
                CHECK(h.last_code == LP_REJ_UNKNOWN_CMD && h.s.state == st && h.o.phase == LP_PHASE_NONE && !h.o.tlm.v2,
                      "ballistic %s + code %d: REJ_UNKNOWN_CMD, TELEM v1", bst[a], c);
            }
        }
    }
}

static void test_onboard_vtol_rules(void)
{
    oh_t h;
    const onb_outputs_t *o;
    onb_params_t p;
    uint32_t i, k0;
    int k;
    onb_params_mode(&p, LP_MODE_VTOL);
    CHECK(p.hover_dm == ONB_V_HOVER_DM && p.alt_min_dm <= p.hover_dm && p.hover_dm <= p.alt_max_dm && p.alt_step_dm > 0 &&
          p.alt_max_dm < p.ceiling_m * 10.0f && p.ceiling_m < p.v_geofence_m && p.endurance_ticks <= p.hard_cutoff_ticks &&
          p.v_fall_vz_mps < 0.0f, "VTOL parameter relations (sim/onboard.py check_vtol)");
    /* ON gates in the ARM order, then the endurance gate */
    {
        static const char *nm[] = {"arm switch", "imu", "baro", "aligned", "tilt == max", "vcell < min", "vcell == min",
                                   "tilt just below"};
        int want;
        for (k = 0; k < 8; k++) {
            vh_init(&h);
            oh_hb(&h);
            if (k == 0) h.arm = false;
            if (k == 1) h.imu_ok = false;
            if (k == 2) h.baro_ok = false;
            if (k == 3) h.aligned = false;
            if (k == 4) h.tilt = p.arm_tilt_max_deg;
            if (k == 5) h.vcell = nextafterf(p.min_cell_v, 0.0f);
            if (k == 6) h.vcell = p.min_cell_v;
            if (k == 7) h.tilt = nextafterf(p.arm_tilt_max_deg, 0.0f);
            o = oh_send(&h, LP_CMD_ON, 0);
            want = k == 0 ? LP_REJ_ARM_SWITCH : k <= 2 ? LP_REJ_SENSORS : k == 3 ? LP_REJ_NOT_ALIGNED : k == 4 ? LP_REJ_TILT
                 : k == 5 ? LP_REJ_VCELL : 0;
            CHECK(h.last_code == want && o->state == (want ? ONB_SAFE : ONB_ARMED), "ON gate %s: code 0x%02X", nm[k], h.last_code);
        }
        vh_init(&h);
        oh_hb(&h);
        oh_run(&h, p.link_disarm_ticks, 0);
        oh_send(&h, LP_CMD_ON, 0);
        CHECK(h.last_code == LP_REJ_LINK, "ON after a silence >= link_loss_disarm_s");
        vh_init(&h);
        oh_hb(&h);
        h.arm = false;
        h.s.powered_ticks = p.endurance_ticks;
        oh_send(&h, LP_CMD_ON, 0);
        CHECK(h.last_code == LP_REJ_ARM_SWITCH, "ON: the ARM gates come before the endurance gate");
        h.arm = true;
        oh_send(&h, LP_CMD_ON, 0);
        CHECK(h.last_code == LP_REJ_ENDURANCE && h.s.state == ONB_SAFE, "ON with the pack's flight time used: REJ_ENDURANCE");
        h.s.powered_ticks = p.endurance_ticks - 1u;
        oh_send(&h, LP_CMD_ON, 0);
        CHECK(h.last_code == 0 && h.s.state == ONB_ARMED, "ON one tick below the endurance limit");
    }
    /* spool warning: exactly spool_ticks, buzzer on, motor off; the take-off gate (V7) at its end */
    for (k = 0; k < 3; k++) {
        vh_init(&h);
        oh_hb(&h);
        o = oh_send(&h, LP_CMD_ON, 0);
        k0 = h.k - 1;
        CHECK(o->buzzer && !o->motor_on && o->h_tgt_dm == p.hover_dm && o->tlm.h_tgt_m == (float)p.hover_dm / 10.0f,
              "SPOOL: buzzer, motor off, target hover");
        for (i = 1; i < p.spool_ticks; i++) o = (i % 20) ? oh_tick(&h) : oh_hb(&h);
        CHECK(o->state == ONB_ARMED && h.k - 1 == k0 + p.spool_ticks - 1u, "still SPOOL one tick before the end");
        if (k == 1) h.tilt = p.arm_tilt_max_deg;
        if (k == 2) h.vcell = nextafterf(p.min_cell_v, 0.0f);
        o = oh_tick(&h);
        if (k == 0) {
            CHECK(o->state == ONB_POWERED && o->phase == LP_PHASE_CLIMB && o->motor_on && !o->buzzer && o->control_active &&
                  o->n_trans == 1 && o->trans[0].cause == ONB_CAUSE_SPOOL_DONE && o->trans[0].phase_from == LP_PHASE_SPOOL &&
                  o->trans[0].phase_to == LP_PHASE_CLIMB, "spool end -> POWERED CLIMB at exactly spool_ticks");
            CHECK(h.s.powered_ticks == 1u, "the motor-time clock counts the take-off tick");
        } else {
            CHECK(o->state == ONB_SAFE && o->reason == LP_R_V_TAKEOFF_GATE && (o->flags & LP_FLAG_INTERLOCK) &&
                  o->interlock == LP_R_V_TAKEOFF_GATE, "V7 take-off gate (%s) -> SAFE", k == 1 ? "tilt" : "V_cell");
        }
    }
    /* CLIMB -> HOLD and LAND -> LANDED act on the previous tick's flags; the flags in other phases do nothing */
    vh_init(&h);
    vh_to(&h, "CLIMB");
    h.td = true;
    CHECK(oh_tick(&h)->phase == LP_PHASE_CLIMB, "touchdown flag in CLIMB: nothing");
    h.td = false;
    h.at = true;
    o = oh_tick(&h);
    CHECK(o->phase == LP_PHASE_HOLD && o->n_trans == 1 && o->trans[0].from == ONB_POWERED && o->trans[0].to == ONB_POWERED &&
          o->trans[0].cause == ONB_CAUSE_AT_TARGET, "at_target -> HOLD (a phase change logged as POWERED -> POWERED)");
    o = oh_tick(&h);
    CHECK(o->phase == LP_PHASE_HOLD && o->n_trans == 0, "at_target in HOLD: nothing");
    h.at = false;
    oh_send(&h, LP_CMD_OFF, 0);
    h.at = true;
    CHECK(oh_tick(&h)->phase == LP_PHASE_LAND, "at_target in LAND: nothing");
    h.at = false;
    h.td = true;
    o = oh_tick(&h);
    CHECK(o->state == ONB_UNPOWERED && o->phase == LP_PHASE_LANDED && !o->motor_on && !o->control_active &&
          o->trans[0].cause == ONB_CAUSE_TOUCHDOWN, "touchdown -> UNPOWERED LANDED, motor off, vanes centred");
    h.td = false;
    k0 = h.k - 1;
    oh_run(&h, p.safe_after_landed_ticks - 1u, 0);
    CHECK(h.s.state == ONB_UNPOWERED, "LANDED held one tick before safe_after_landed_ticks (no link needed)");
    o = oh_tick(&h);
    CHECK(o->state == ONB_SAFE && o->phase == LP_PHASE_NONE && h.k - 1 - k0 == p.safe_after_landed_ticks &&
          o->trans[0].cause == ONB_CAUSE_LANDED_TIMER, "LANDED -> SAFE after safe_after_landed_ticks");
    CHECK(!o->release && o->reason == LP_REASON_NONE, "a normal landing: no release, no interlock");
    oh_hb(&h);
    CHECK(oh_send(&h, LP_CMD_ON, 0)->state == ONB_ARMED, "another ON flies again");
    /* arm switch: SPOOL -> SAFE, CLIMB -> ABORT, LANDED -> SAFE (no release toward a person) */
    {
        static const char *st[3] = {"SPOOL", "CLIMB", "LANDED"};
        for (k = 0; k < 3; k++) {
            vh_init(&h);
            vh_to(&h, st[k]);
            h.arm = false;
            o = oh_hb(&h);
            CHECK(o->state == (k == 1 ? ONB_ABORT : ONB_SAFE) && o->reason == LP_R_ARM_SWITCH_OPEN, "arm switch in %s", st[k]);
            oh_run(&h, p.abort_release_ticks + 5u, 20);
            CHECK(h.o.release == (k == 1), "arm switch in %s: release %s", st[k], k == 1 ? "after the abort delay" : "never");
        }
    }
    /* I9 IMU: SPOOL -> SAFE, POWERED -> ABORT, LANDED -> SAFE (v0.7.1); I9 baro: SPOOL -> SAFE, CLIMB / HOLD -> LAND on the
     * dead reckoning, LAND -> dead reckoning from now on, LANDED -> nothing */
    {
        static const char *st[5] = {"SPOOL", "CLIMB", "HOLD", "LAND", "LANDED"};
        int s;
        for (s = 0; s < 5; s++) {
            vh_init(&h);
            vh_to(&h, st[s]);
            h.imu_ok = false;
            oh_run(&h, p.imu_fail_ticks - 1u, 20);
            CHECK(!strcmp(state_name(h.s.state), s == 0 ? "ARMED" : s == 4 ? "UNPOWERED" : "POWERED"), "I9 imu %s: not before",
                  st[s]);
            o = oh_hb(&h);
            CHECK(o->state == ((s == 0 || s == 4) ? ONB_SAFE : ONB_ABORT) && o->reason == LP_R_I9_IMU, "I9 imu in %s", st[s]);
            oh_run(&h, p.abort_release_ticks + 5u, 20);
            CHECK(h.o.release == (s >= 1 && s <= 3), "I9 imu in %s: release %s", st[s], (s >= 1 && s <= 3) ? "after the "
                  "abort delay" : "never");
        }
        for (s = 0; s < 5; s++) {
            uint8_t ph0;
            vh_init(&h);
            vh_to(&h, st[s]);
            ph0 = h.s.phase;
            h.baro_ok = false;
            oh_run(&h, p.baro_fail_ticks - 1u, 20);
            CHECK(!h.s.baro_lost && h.s.phase == ph0, "I9 baro %s: not before", st[s]);
            o = oh_hb(&h);
            if (s == 0)
                CHECK(o->state == ONB_SAFE && o->reason == LP_R_I9_BARO, "I9 baro in SPOOL -> SAFE");
            else if (s <= 2)
                CHECK(o->state == ONB_POWERED && o->phase == LP_PHASE_LAND && o->baro_lost && o->reason == LP_R_I9_BARO &&
                      o->interlock == LP_R_I9_BARO && h.s.baro_lost_tick == h.k - 1u &&
                      h.s.baro_limit == onboard_baro_limit_ticks(&p, h.h), "I9 baro in %s -> LAND on the dead reckoning, "
                      "time limit fixed", st[s]);
            else if (s == 3)
                CHECK(o->state == ONB_POWERED && o->phase == LP_PHASE_LAND && o->baro_lost && o->reason == LP_R_I9_BARO &&
                      o->n_trans == 0, "I9 baro in LAND: inertial landing, no transition");
            else
                CHECK(o->state == ONB_UNPOWERED && !o->baro_lost && o->reason == LP_REASON_NONE, "I9 baro in LANDED: nothing");
            if (s == 1) {                                /* baro_lost latches until the next ON */
                h.baro_ok = true;
                oh_run(&h, 50, 10);
                CHECK(h.s.baro_lost, "baro_lost stays latched");
                h.td = true;
                oh_hb(&h);
                h.td = false;
                oh_run(&h, p.safe_after_landed_ticks + 1u, 20);
                CHECK(h.s.state == ONB_SAFE && h.s.baro_lost, "still latched in SAFE");
                oh_send(&h, LP_CMD_ON, 0);
                CHECK(h.s.state == ONB_ARMED && !h.s.baro_lost, "the next ON clears baro_lost");
            }
        }
    }
    /* I1p in SPOOL; V1 link loss lands from CLIMB / HOLD exactly at link_land_ticks, nothing in LAND / LANDED */
    {
        static const char *st[5] = {"SPOOL", "CLIMB", "HOLD", "LAND", "LANDED"};
        for (k = 0; k < 5; k++) {
            uint32_t last, n = k == 0 ? p.link_disarm_ticks : p.link_land_ticks;
            onb_params_t q = p;
            q.spool_ticks = 2u * p.link_disarm_ticks;          /* long enough to see I1p in SPOOL */
            vh_init_p(&h, k == 0 ? &q : &p, 1, false);
            vh_to(&h, st[k]);
            last = h.s.last_frame_tick;                        /* vh_to ends with a frame except after the spool run */
            oh_run(&h, last + n - 1u - (h.k - 1u), 0);
            CHECK(!strcmp(state_name(h.s.state), k == 0 ? "ARMED" : k == 4 ? "UNPOWERED" : "POWERED"),
                  "%s held one tick before the link timeout", st[k]);
            o = oh_tick(&h);
            if (k == 0)
                CHECK(h.k - 1 - last == n && o->state == ONB_SAFE && o->reason == LP_R_I1P_LINK_LOSS_PAD, "I1p in SPOOL");
            else if (k <= 2)
                CHECK(h.k - 1 - last == n && o->state == ONB_POWERED && o->phase == LP_PHASE_LAND &&
                      o->reason == LP_R_V_LINK_LAND && o->trans[0].cause == LP_R_V_LINK_LAND,
                      "V1 link loss in %s -> LAND exactly at link_land_ticks", st[k]);
            else if (k == 3)
                CHECK(o->reason == LP_REASON_NONE && o->n_trans == 0 && o->phase == LP_PHASE_LAND, "no link rule in LAND");
            else                                       /* LANDED: safe_after_landed == link_land here: only the timer acts */
                CHECK(o->reason == LP_REASON_NONE && (o->n_trans == 0 || o->trans[0].cause == ONB_CAUSE_LANDED_TIMER),
                      "no link rule in LANDED");
        }
        vh_init(&h);
        vh_to(&h, "HOLD");
        oh_run(&h, 10u * p.link_land_ticks, 0);
        CHECK(h.s.state == ONB_POWERED && h.s.phase == LP_PHASE_LAND, "VTOL: no link-loss ABORT (a parachute cannot open at a "
              "few metres) - the landing continues");
    }
    /* I2 tilt (strict), I3 rate, POWERED only */
    vh_init(&h);
    vh_to(&h, "HOLD");
    h.tilt = p.tilt_abort_deg;
    CHECK(oh_tick(&h)->state == ONB_POWERED, "tilt == limit: no abort");
    h.tilt = nextafterf(p.tilt_abort_deg, 90.0f);
    o = oh_tick(&h);
    CHECK(o->state == ONB_ABORT && o->reason == LP_R_I2_TILT, "I2 tilt");
    vh_init(&h);
    vh_to(&h, "LANDED");
    h.tilt = 80.0f;
    CHECK(oh_tick(&h)->state == ONB_UNPOWERED, "I2 not in LANDED");
    vh_init(&h);
    vh_to(&h, "LAND");
    h.gyro[2] = p.spin_rate_abort * 1.01f;
    oh_run(&h, p.rate_persist_ticks - 1u, 0);
    CHECK(h.s.state == ONB_POWERED, "I3 not before the persistence time");
    o = oh_tick(&h);
    CHECK(o->state == ONB_ABORT && o->reason == LP_R_I3_RATE, "I3 spin in LAND");
    /* V4 geofence (strict) in POWERED and LANDED; V3 ceiling (strict) lands from CLIMB / HOLD; V4 before V3 */
    {
        static const char *st[2] = {"HOLD", "LANDED"};
        for (k = 0; k < 2; k++) {
            vh_init(&h);
            vh_to(&h, st[k]);
            h.h = p.v_geofence_m;
            o = oh_tick(&h);
            CHECK(o->state != ONB_ABORT, "h == geofence: no abort (%s)", st[k]);
            h.h = nextafterf(p.v_geofence_m, 1e9f);
            o = oh_tick(&h);
            CHECK(o->state == (k == 0 ? ONB_ABORT : ONB_SAFE) && o->reason == LP_R_V_GEOFENCE, "V4 geofence in %s -> %s",
                  st[k], k == 0 ? "ABORT" : "SAFE (v0.7.1 LANDED rule)");
        }
        vh_init(&h);
        vh_to(&h, "CLIMB");
        h.h = p.ceiling_m;
        CHECK(oh_tick(&h)->phase == LP_PHASE_CLIMB, "h == ceiling: nothing");
        h.h = nextafterf(p.ceiling_m, 1e9f);
        o = oh_tick(&h);
        CHECK(o->phase == LP_PHASE_LAND && o->reason == LP_R_V_CEILING, "V3 ceiling -> LAND");
        oh_tick(&h);
        CHECK(h.s.phase == LP_PHASE_LAND && h.s.state == ONB_POWERED, "V3 does not act in LAND");
    }
    /* V5 fall: v_z <= fall_vz for fall_confirm_ticks -> ABORT (POWERED), SAFE (LANDED, v0.7.1); a slower tick restarts
     * the count */
    {
        static const char *st[2] = {"HOLD", "LANDED"};
        for (k = 0; k < 2; k++) {
            vh_init(&h);
            vh_to(&h, st[k]);
            h.vz = p.v_fall_vz_mps;
            oh_run(&h, p.v_fall_confirm_ticks - 1u, 20);
            h.vz = nextafterf(p.v_fall_vz_mps, 0.0f);
            oh_tick(&h);
            h.vz = p.v_fall_vz_mps;
            oh_run(&h, p.v_fall_confirm_ticks - 1u, 20);
            CHECK(h.s.state != ONB_ABORT && h.s.state != ONB_SAFE, "V5 %s: a slower tick restarts the count", st[k]);
            o = oh_tick(&h);
            CHECK(o->state == (k == 0 ? ONB_ABORT : ONB_SAFE) && o->reason == LP_R_V_FALL, "V5 fall in %s", st[k]);
        }
    }
    /* v0.7.1 baro-lost time limit V8 / V9: T_lim = base + ceil(h_dm x mt_per_dm / 1000) from h at the loss */
    {
        onb_params_t q = p;
        const int32_t hdm = 50;
        uint32_t lim, t_loss;
        CHECK(onboard_baro_limit_ticks(&p, 5.0f) == p.baro_limit_base_ticks + ((uint32_t)hdm * p.baro_limit_mt_per_dm + 999u) /
              1000u, "baro limit at 5 m: base + ceil(50 x mt / 1000)");
        CHECK(onboard_baro_limit_ticks(&p, NAN) == onboard_baro_limit_ticks(&p, 1e6f) &&
              onboard_baro_limit_ticks(&p, INFINITY) == p.baro_limit_base_ticks + ((uint32_t)p.v_geofence_dm *
              p.baro_limit_mt_per_dm + 999u) / 1000u, "non-finite h / above the geofence: the longest limit");
        CHECK(onboard_baro_limit_ticks(&p, -3.0f) == p.baro_limit_base_ticks && onboard_baro_limit_ticks(&p, 0.04f) ==
              p.baro_limit_base_ticks && onboard_baro_limit_ticks(&p, 0.05f) == p.baro_limit_base_ticks +
              (p.baro_limit_mt_per_dm + 999u) / 1000u, "h below the pad: base; 0.05 m rounds to 1 dm");
        q.baro_limit_base_ticks = 40;
        q.baro_limit_mt_per_dm = 1500;                   /* 1.5 ticks per dm: 5 m -> 40 + 75 ticks */
        for (k = 0; k < 4; k++) {                        /* 0: V8 ground felt, 1: V9 none, 2: touchdown first, 3: from LAND */
            vh_init_p(&h, &q, 1, false);
            vh_to(&h, k == 3 ? "LAND" : "HOLD");
            h.h = 5.0f;
            h.baro_ok = false;
            oh_run(&h, q.baro_fail_ticks - 1u, 20);
            o = oh_hb(&h);
            t_loss = h.k - 1u;
            lim = onboard_baro_limit_ticks(&q, 5.0f);
            CHECK(lim == 115u && h.s.baro_lost && h.s.baro_lost_tick == t_loss && h.s.baro_limit == lim &&
                  o->phase == LP_PHASE_LAND && o->reason == LP_R_I9_BARO, "baro loss at 5 m (%s): limit %lu ticks fixed on "
                  "the tick I9 acts", k == 3 ? "in LAND" : "in HOLD", (unsigned long)h.s.baro_limit);
            h.h = 0.5f * q.v_geofence_m;                 /* the dead-reckoned h later: no effect on the limit */
            h.gsig = k != 1;
            h.lil = 60000u;                              /* V10 is not counted with the baro lost */
            oh_run(&h, t_loss + lim - 1u - (h.k - 1u), 20);
            CHECK(h.s.state == ONB_POWERED && h.s.phase == LP_PHASE_LAND, "one tick before the limit: still LAND");
            if (k == 2) h.td = true;
            o = oh_tick(&h);
            if (k == 1) {
                CHECK(o->state == ONB_ABORT && o->reason == LP_R_V_BARO_LIMIT_ABORT && o->interlock == LP_R_V_BARO_LIMIT_ABORT &&
                      o->trans[0].cause == LP_R_V_BARO_LIMIT_ABORT, "V9: limit without a ground signature -> ABORT");
                oh_run(&h, p.abort_release_ticks, 20);
                CHECK(h.o.release, "V9: release after the abort delay (the altitude is unknown)");
            } else if (k == 2) {
                CHECK(o->state == ONB_UNPOWERED && o->trans[0].cause == ONB_CAUSE_TOUCHDOWN && o->reason == LP_R_I9_BARO,
                      "a touchdown confirmed the previous tick wins over the time limit (LANDED by the timer)");
            } else {
                CHECK(o->state == ONB_UNPOWERED && o->phase == LP_PHASE_LANDED && !o->motor_on &&
                      o->reason == LP_R_V_BARO_LIMIT_LANDED && o->trans[0].cause == LP_R_V_BARO_LIMIT_LANDED,
                      "V8: limit with a ground signature -> LANDED, motor off");
                oh_run(&h, p.abort_release_ticks + 5u, 20);
                CHECK(!h.o.release && (h.s.state == ONB_UNPOWERED || h.s.state == ONB_SAFE), "V8: no release");
            }
            h.td = false;
            h.gsig = false;
            h.lil = 0;
        }
        vh_init_p(&h, &q, 1, false);                     /* the ground signature in HOLD / CLIMB does nothing */
        vh_to(&h, "HOLD");
        h.gsig = true;
        h.lil = 60000u;
        oh_run(&h, 200, 20);
        CHECK(h.s.state == ONB_POWERED && h.s.phase == LP_PHASE_HOLD && h.o.reason == LP_REASON_NONE,
              "ground signature / low_in_land outside LAND: nothing");
    }
    /* v0.7.1 V10 LAND floor timeout: the controller low and leading for land_timeout_ticks -> LANDED, no release */
    {
        onb_params_t q = p;
        CHECK(p.land_timeout_ticks == ONB_V_LAND_TIMEOUT_TICKS && p.land_timeout_ticks > 0u, "V10 timeout from params_gen.h");
        q.land_timeout_ticks = 300;
        vh_init_p(&h, &q, 1, false);
        vh_to(&h, "LAND");
        h.lil = 299;
        o = oh_hb(&h);
        CHECK(o->state == ONB_POWERED && o->phase == LP_PHASE_LAND, "V10: low_in_land 299 < 300: still LAND");
        h.lil = 300;
        o = oh_hb(&h);
        CHECK(o->state == ONB_UNPOWERED && o->phase == LP_PHASE_LANDED && o->reason == LP_R_V_LAND_TIMEOUT &&
              o->interlock == LP_R_V_LAND_TIMEOUT && o->trans[0].cause == LP_R_V_LAND_TIMEOUT && !o->motor_on,
              "V10 at exactly land_timeout_ticks -> LANDED, motor off");
        h.lil = 0;
        oh_run(&h, p.abort_release_ticks + 5u, 20);
        CHECK(!h.o.release, "V10: no release");
        q.land_timeout_ticks = 0;                        /* 0 = off */
        vh_init_p(&h, &q, 1, false);
        vh_to(&h, "LAND");
        h.lil = 65535u;
        oh_run(&h, 50, 10);
        CHECK(h.s.state == ONB_POWERED && h.s.phase == LP_PHASE_LAND, "land_timeout_ticks 0: V10 off");
        q.land_timeout_ticks = 300;                      /* V6 outranks V10 */
        q.hard_cutoff_ticks = q.endurance_ticks;
        vh_init_p(&h, &q, 1, false);
        vh_to(&h, "LAND");
        h.s.powered_ticks = q.hard_cutoff_ticks;
        h.lil = 300;
        o = oh_hb(&h);
        CHECK(o->state == ONB_ABORT && o->reason == LP_R_V_HARD_CUTOFF, "V6 outranks V10");
    }
    /* v0.7.1 LANDED rules: I9 IMU, V4, V5 -> SAFE, never ABORT (landed_rules_note) */
    {
        for (k = 0; k < 3; k++) {
            vh_init(&h);
            vh_to(&h, "LANDED");
            if (k == 0) h.imu_ok = false;
            if (k == 1) h.h = nextafterf(p.v_geofence_m, 1e9f);
            if (k == 2) h.vz = p.v_fall_vz_mps;
            oh_run(&h, (k == 0 ? p.imu_fail_ticks : k == 1 ? 1u : p.v_fall_confirm_ticks) - 1u, 20);
            CHECK(h.s.state == ONB_UNPOWERED, "LANDED rule %d: not before", k);
            o = oh_tick(&h);
            CHECK(o->state == ONB_SAFE && o->reason == (k == 0 ? LP_R_I9_IMU : k == 1 ? LP_R_V_GEOFENCE : LP_R_V_FALL) &&
                  o->trans[0].from == ONB_UNPOWERED && o->trans[0].to == ONB_SAFE, "LANDED + %s -> SAFE",
                  k == 0 ? "I9 IMU" : k == 1 ? "V4 geofence" : "V5 fall");
            oh_run(&h, p.abort_release_ticks + 5u, 20);
            CHECK(!h.o.release, "LANDED + %s: no release", k == 0 ? "I9 IMU" : k == 1 ? "V4" : "V5");
        }
    }
    /* V2 flight time (LAND) and V6 hard cutoff (ABORT) on the cumulative motor time; REJ_ENDURANCE afterwards */
    {
        onb_params_t q = p;
        uint32_t t_pow;
        q.endurance_ticks = 300;
        q.hard_cutoff_ticks = 500;
        vh_init_p(&h, &q, 1, false);
        vh_to(&h, "CLIMB");
        t_pow = h.k - 1u - (h.s.powered_ticks - 1u);                      /* the take-off tick */
        while (h.s.phase != LP_PHASE_LAND && h.k < t_pow + 1000u) o = ((h.k - t_pow) % 20u) ? oh_tick(&h) : oh_hb(&h);
        CHECK(h.k - 1u == t_pow + 300u && h.o.reason == LP_R_V_ENDURANCE && h.s.powered_ticks == 301u,
              "V2 flight-time LAND at motor time 300 ticks (tick %lu, take-off %lu)", (unsigned long)(h.k - 1u),
              (unsigned long)t_pow);
        while (h.s.state == ONB_POWERED && h.k < t_pow + 2000u) o = ((h.k - t_pow) % 20u) ? oh_tick(&h) : oh_hb(&h);
        CHECK(h.k - 1u == t_pow + 500u && h.o.reason == LP_R_V_HARD_CUTOFF && h.s.state == ONB_ABORT,
              "V6 hard cutoff aborts a landing still under way at 500 ticks");
        /* cumulative: land early, fly again - the second flight inherits the first one's motor time */
        vh_init_p(&h, &q, 1, false);
        vh_to(&h, "LANDED");
        oh_run(&h, q.safe_after_landed_ticks + 1u, 20);
        k0 = h.s.powered_ticks;
        CHECK(h.s.state == ONB_SAFE && k0 > 0u && k0 < 300u, "first hop used %lu ticks", (unsigned long)k0);
        oh_send(&h, LP_CMD_ON, 0);
        oh_run(&h, q.spool_ticks, 20);
        while (h.s.phase != LP_PHASE_LAND && h.k < 100000u) (void)((h.k % 20u) ? oh_tick(&h) : oh_hb(&h));
        CHECK(h.s.powered_ticks == 301u && h.o.reason == LP_R_V_ENDURANCE, "second hop: V2 at the cumulative 300 ticks");
        h.td = true;
        oh_hb(&h);
        h.td = false;
        oh_run(&h, q.safe_after_landed_ticks + 1u, 20);
        oh_send(&h, LP_CMD_ON, 0);
        CHECK(h.last_code == LP_REJ_ENDURANCE && h.s.state == ONB_SAFE, "the used pack refuses ON (REJ_ENDURANCE)");
    }
    /* I8 V_cell lands from CLIMB / HOLD, nothing in LAND */
    vh_init(&h);
    vh_to(&h, "HOLD");
    h.vcell = nextafterf(p.min_cell_v, 0.0f);
    o = oh_tick(&h);
    CHECK(o->phase == LP_PHASE_LAND && o->reason == LP_R_I8_VCELL, "I8 V_cell -> LAND");
    /* rule order: arm switch > IMU > baro > ... ; I2 > V4 > V5 > V6; V4 > V1 */
    vh_init(&h);
    vh_to(&h, "HOLD");
    h.arm = false;
    h.imu_ok = false;
    h.tilt = 80.0f;
    oh_run(&h, p.imu_fail_ticks, 20);
    CHECK(h.s.state == ONB_ABORT && h.o.reason == LP_R_ARM_SWITCH_OPEN, "arm switch outranks I9 and I2");
    vh_init(&h);
    vh_to(&h, "HOLD");
    h.tilt = 80.0f;
    h.h = 1000.0f;
    h.vz = -50.0f;
    CHECK(oh_tick(&h)->reason == LP_R_I2_TILT, "I2 outranks V4 / V5");
    vh_init(&h);
    vh_to(&h, "HOLD");
    oh_run(&h, p.link_land_ticks - 1u, 0);
    h.h = 1000.0f;
    h.vcell = 3.0f;
    o = oh_tick(&h);
    CHECK(o->reason == LP_R_V_GEOFENCE && o->state == ONB_ABORT, "V4 outranks the LAND rules (V1, I8, V3 due the same tick)");
    /* the interlock reason outranks a later rejection; the REJECTED flag still shows it */
    vh_init(&h);
    vh_to(&h, "HOLD");
    h.vcell = 3.0f;
    oh_hb(&h);
    o = oh_send(&h, LP_CMD_UP, 0);
    CHECK(h.last_code == LP_REJ_LANDING && o->reason == LP_R_I8_VCELL && (o->flags & LP_FLAG_REJECTED) &&
          (o->flags & LP_FLAG_INTERLOCK), "reason I8 kept over REJ_LANDING");
    /* UP / DOWN limits: exact integer decimetres */
    vh_init(&h);
    vh_to(&h, "HOLD");
    {
        int ups = 0, downs = 0;
        while (oh_send(&h, LP_CMD_UP, 0), h.last_code == 0 && ups < 1000) ups++;
        CHECK(h.last_code == LP_REJ_ALT_LIMIT && h.s.tgt_dm == p.alt_max_dm && ups == (p.alt_max_dm - p.hover_dm) / p.alt_step_dm,
              "UP to alt_max (%d steps), then REJ_ALT_LIMIT", ups);
        while (oh_send(&h, LP_CMD_DOWN, 0), h.last_code == 0 && downs < 1000) downs++;
        CHECK(h.last_code == LP_REJ_ALT_LIMIT && h.s.tgt_dm == p.alt_max_dm - downs * p.alt_step_dm &&
              h.s.tgt_dm - p.alt_step_dm < p.alt_min_dm, "DOWN to alt_min, then REJ_ALT_LIMIT");
    }
    /* I10 in the VTOL build: boot ABORT, release after abort_release_ticks, ON refused */
    vh_init_p(&h, &p, 1, true);
    o = oh_hb(&h);
    CHECK(o->state == ONB_ABORT && o->reason == LP_R_I10_RESET && (o->flags & LP_FLAG_INTERLOCK) && !o->release &&
          !o->motor_on, "I10 VTOL boot: ABORT, motor off");
    oh_send(&h, LP_CMD_ON, 0);
    CHECK(h.last_code == LP_REJ_RELEASE_PENDING, "ON while the release is pending");
    oh_run(&h, p.abort_release_ticks, 20);
    CHECK(h.s.state == ONB_RELEASED && h.o.release, "I10: RELEASED after abort_release_ticks");
    oh_send(&h, LP_CMD_ON, 0);
    CHECK(h.last_code == LP_REJ_STATE, "ON in RELEASED: REJ_STATE (power cycle)");
    /* TELEM v2: phase, target, throttle = the previous tick's ESC throttle; the frame decodes as TELEM2 */
    {
        uint8_t fr[LP_FRAME_MAX_BYTES], n, j, m = 0;
        lp_decoder_t dec;
        lp_frame_t out[LP_DEC_MAX_OUT];
        lp_telem_t t, u;
        vh_init(&h);
        vh_to(&h, "HOLD");
        oh_send(&h, LP_CMD_UP, 0);
        h.thr_prev = 0.9185f;
        h.h = 1.97f;                                     /* below the ceiling of every profile */
        o = oh_hb(&h);
        onboard_telemetry(&o->tlm, FC_LOOP_HZ, &t);
        CHECK(t.phase == LP_PHASE_HOLD && t.h_tgt_dm == p.hover_dm + p.alt_step_dm && t.throttle_milli == 919 &&
              t.h_dm == 20 && o->throttle_milli == 0, "TELEM2 phase %u target %d throttle %u", t.phase, t.h_tgt_dm,
              t.throttle_milli);
        n = onboard_telemetry_frame(&o->tlm, FC_LOOP_HZ, LINK_VEHICLE_ID, o->reply_seq, fr);
        CHECK(n == LP_TELEM2_FRAME_BYTES && fr[4] == LP_TYPE_TELEM2, "the VTOL reply is a 39-byte TELEM2 frame");
        lp_decoder_init(&dec, LINK_VEHICLE_ID, LP_ACCEPT_TELEM | LP_ACCEPT_TELEM2);
        for (j = 0; j < n; j++) m = (uint8_t)(m + lp_decoder_feed(&dec, fr[j], out + m, (uint8_t)(LP_DEC_MAX_OUT - m)));
        CHECK(m == 1 && out[0].type == LP_TYPE_TELEM2, "TELEM2 decodes");
        lp_unpack_telem2(out[0].payload, &u);
        CHECK(telem_eq(&t, &u), "decode(encode(TELEM2)) == t");
        lp_decoder_init(&dec, LINK_VEHICLE_ID, LP_ACCEPT_TELEM);
        for (j = 0, m = 0; j < n; j++) m = (uint8_t)(m + lp_decoder_feed(&dec, fr[j], out, LP_DEC_MAX_OUT));
        CHECK(m == 0 && dec.stats.wrong_type == 1, "a v0.6.3 (TELEM-only) ground decoder drops TELEM2 as wrong_type");
        o = oh_send(&h, LP_CMD_OFF, 0);
        onboard_telemetry(&o->tlm, FC_LOOP_HZ, &t);
        CHECK(t.phase == LP_PHASE_LAND && t.h_tgt_dm == 0, "LAND reports target 0");
    }
    CHECK(onboard_flying_mode(LP_MODE_VTOL, ONB_POWERED) && !onboard_flying_mode(LP_MODE_VTOL, ONB_UNPOWERED) &&
          !onboard_flying_mode(LP_MODE_VTOL, ONB_ARMED) && onboard_flying_mode(LP_MODE_VTOL, ONB_ABORT) &&
          onboard_flying_mode(LP_MODE_BALLISTIC, ONB_UNPOWERED) && onboard_flying(ONB_UNPOWERED),
          "I10 'in flight' states: VTOL leaves LANDED (UNPOWERED) out");
}

/* ---- vertical controller v2 (v0.7.1) ---- */
static void vc_default(vc_params_t *p)
{
    fc_default_vertical(p);
#if FC_V09
    /* the unit tests below check the v0.8.1 law (the v0.9 options off - the reference's "absent = bit for bit");
     * test_vc_v09 and tools/crosscheck_controller.py section 10 the v0.9 ones */
    p->v9 = 0;
    p->approach_shift = 0;
    p->ramp_total_ticks = p->ramp_ticks;
#endif
}

/* q of a tilt about the body x axis (cos(tilt) = 1 - 2 sin^2(tilt / 2)) */
static float vc_cos_of_tilt_deg(double tilt_deg)
{
    float q[4];
    q[0] = (float)cos(tilt_deg * IL_PI_D / 360.0);
    q[1] = (float)sin(tilt_deg * IL_PI_D / 360.0);
    q[2] = q[3] = 0.0f;
    return vc_cos_tilt_from_q(q);
}

static vc_outputs_t vc_step2(vc_t *c, uint8_t phase, float tgt, bool baro_lost, float h, float vz, float cos_tilt, float f_z)
{
    vc_inputs_t in;
    vc_outputs_t o;
    in.phase = phase;
    in.h_tgt_dm = (int16_t)lp_quantize(tgt, LP_H_SCALE, LP_I16_MIN, LP_I16_MAX);   /* the supervisor's integer target */
    in.baro_lost = baro_lost;
    in.h_m = h;
    in.vz_mps = vz;
    in.cos_tilt = cos_tilt;
    in.f_z_mps2 = f_z;
    vc_update(c, &in, &o);
    return o;
}

static vc_outputs_t vc_step(vc_t *c, uint8_t phase, float tgt, bool baro_lost, float h, float vz)
{
    return vc_step2(c, phase, tgt, baro_lost, h, vz, 1.0f, NAN);
}

/* round_half_away(x * 1e6) in double: exact for a float32 x (the product has <= 38 significant bits) */
static long long m_to_um_ref(float x)
{
    double v;
    if (x != x) return 0;
    v = (double)x * 1e6;
    if (v >= 2e9) return 2000000000LL;
    if (v <= -2e9) return -2000000000LL;
    return v >= 0.0 ? (long long)floor(v + 0.5) : -(long long)floor(-v + 0.5);
}

static void test_vertical_ctrl(void)
{
    vc_params_t p;
    vc_t c;
    vc_outputs_t o;
    int k;
    double uh;
    const double dt = 1.0 / FC_LOOP_HZ;
    vc_default(&p);
    uh = pow((double)VEH_MASS_KG * 9.80665 / (double)MIX_T_MAX_N, 1.0 / (double)MIX_THROTTLE_EXP);
    CHECK_NEAR(p.u_hover, uh, 2e-6, "u_hover = (m g / T_max)^(1/n)");
    CHECK_NEAR(p.inv_u_hover, 1.0 / uh, 2e-6, "inv_u_hover = 1 / u_hover");
    CHECK(p.up_um == (int32_t)floor((double)VC_CLIMB_RATE_MPS * 1e6 / FC_LOOP_HZ + 0.5) &&
          p.dn_um == (int32_t)floor((double)VC_DESCENT_RATE_MPS * 1e6 / FC_LOOP_HZ + 0.5) &&
          p.td_um == (int32_t)floor((double)VC_TOUCHDOWN_RATE_MPS * 1e6 / FC_LOOP_HZ + 0.5) &&
          p.flare_um == (int32_t)floor((double)VC_FLARE_ALTITUDE_M * 1e6 + 0.5),
          "reference steps = round(rate x 1e6 / loop_hz), flare = round(flare x 1e6) (%ld %ld %ld %ld)", (long)p.up_um,
          (long)p.dn_um, (long)p.td_um, (long)p.flare_um);
    CHECK(p.u_min < p.u_max && p.u_max <= 1.0f && p.td_um <= p.dn_um && p.ramp_ticks == ONB_RAMP_TICKS &&
          p.landed_confirm_ticks >= 1u && p.floor_ticks >= 1u && p.lead_mps > 0.0f && p.support_mps2 > 0.0f,
          "vertical parameter relations (VerticalParams.check), ramp = mission.throttle_ramp_s");
    CHECK_NEAR(p.thrust_alpha, 1.0 - exp(-dt / EDF_TAU_S), 1e-7, "thrust lag alpha = 1 - exp(-dt / tau_EDF)");
    /* vc_m_to_um: exact rounding of the product (ties away from zero), saturation, NaN */
    {
        long bad = 0, n = 0;
        const float edge[] = {0.0f, -0.0f, 0.0078125f, -0.0078125f, 1e-7f, 4.9e-7f, 5e-7f, 0.001f, 1.0f, 5.0f, -0.25f, 1999.9999f,
                              2000.0f, 2000.0002f, -2000.0f, -2000.0002f, 1e30f, -1e30f, 1e-38f, 3e-39f, 16777216.0f,
                              0.5f, 0.15f, 0.3f};
        for (k = 0; k < (int)(sizeof edge / sizeof edge[0]); k++, n++)
            if ((long long)vc_m_to_um(edge[k]) != m_to_um_ref(edge[k])) {
                bad++;
                printf("m_to_um(%.9g) = %ld, want %lld\n", (double)edge[k], (long)vc_m_to_um(edge[k]), m_to_um_ref(edge[k]));
            }
        CHECK(vc_m_to_um(0.0078125f) == 7813 && vc_m_to_um(-0.0078125f) == -7813, "m_to_um tie 7812.5 -> 7813 (away from 0)");
        CHECK(vc_m_to_um(NAN) == 0 && vc_m_to_um(INFINITY) == VC_REF_LIMIT_UM && vc_m_to_um(-INFINITY) == -VC_REF_LIMIT_UM,
              "m_to_um NaN -> 0, +-inf saturate");
        for (k = 0; k < 2000000; k++, n++) {
            union { float f; uint32_t u; } v;
            /* random bit patterns, exponents spread over 2^-40 .. 2^12 (the rounding cases of every shift) */
            v.u = (uint32_t)(urand() * 4294967296.0);
            v.u = (v.u & 0x807FFFFFu) | ((uint32_t)(87 + (int)(urand() * 53.0)) << 23);
            if ((long long)vc_m_to_um(v.f) != m_to_um_ref(v.f)) bad++;
        }
        CHECK(bad == 0, "vc_m_to_um == round_half_away(x 1e6) on %ld floats (%ld bad)", n, bad);
    }
    /* cos(tilt) from q: 1 - 2 (x^2 + y^2) */
    CHECK_NEAR(vc_cos_of_tilt_deg(30.0), cos(30.0 * IL_PI_D / 180.0), 3e-7, "cos_tilt_from_q(30 deg about x)");
    /* IDLE: reset, u = 0 */
    vc_init(&c, &p);
    o = vc_step(&c, LP_PHASE_HOLD, 5.0f, false, 4.0f, 0.0f);
    CHECK(o.mode == VC_MODE_FLY && c.active && c.ref_um == vc_m_to_um(4.0f) + p.up_um,
          "HOLD -> FLY; the reference starts at m_to_um(h) and steps up_um toward the target");
    for (k = 0; k < 6; k++) {
        static const uint8_t ph[3] = {LP_PHASE_NONE, LP_PHASE_SPOOL, LP_PHASE_LANDED};
        o = vc_step(&c, ph[k % 3], 5.0f, false, 4.0f, 1.0f);
        CHECK(o.mode == VC_MODE_IDLE && o.u == 0.0f && o.h_ref == 0.0f && o.integ == 0.0f && !c.active && !o.touchdown &&
              !o.at_target && o.sig == 0 && o.cos_tilt == 1.0f && c.ref_um == 0 && c.v_lpf == 0.0f && c.u_last == 0.0f,
              "phase %u: IDLE, u = 0, reset", ph[k % 3]);
    }
    /* take-off from the pad (h = 0 exactly): exact integer ramp, arrival on tick target / up exactly; the ceiling ramp;
     * the feed-forward of the reference rate */
    {
        const float tgt = 2.0f;                          /* reachable in 2000 ticks in every profile */
        const long n_exact = (long)(2000000L / p.up_um);
        long n_arr = -1, bad_ref = 0, bad_ff = 0;
        vc_init(&c, &p);
        for (k = 1; k <= 2000; k++) {
            o = vc_step(&c, LP_PHASE_CLIMB, tgt, false, 0.0f, 0.0f);
            if (k <= (long)p.ramp_ticks) {
                const double cap = (double)p.u_max * (double)k / (double)p.ramp_ticks;
                CHECK(o.u <= cap + 1e-7, "ramp tick %d: u %.6f <= cap %.6f", k, (double)o.u, cap);
            }
            if (n_arr < 0) {
                if (c.ref_um != (int32_t)((long)k * p.up_um > 2000000L ? 2000000L : (long)k * p.up_um)) bad_ref++;
                if (!(o.dh_ref_mps == (float)p.up_um * p.um_tick_to_mps &&
                      o.vz_cmd == fc_clampf(o.dh_ref_mps + p.alt_kp * (o.h_ref - 0.0f), -p.vz_max_down_mps, p.vz_max_up_mps)))
                    bad_ff++;
                if (o.at_target) n_arr = k;
            }
        }
        CHECK(n_arr == n_exact && c.ref_um == 2000000L && o.h_ref == 2.0f && o.dh_ref_mps == 0.0f,
              "reference reaches 2 m from the pad on tick %ld exactly (got %ld)", n_exact, n_arr);
        CHECK(bad_ref == 0 && bad_ff == 0, "integer reference k x up_um; v_cmd = dh_ref + alt_kp (h_ref - h) (%ld / %ld bad)",
              bad_ref, bad_ff);
        CHECK_NEAR(p.um_tick_to_mps * (float)p.up_um, VC_CLIMB_RATE_MPS, 1e-6, "dh_ref = climb rate during the climb");
        vc_init(&c, &p);
        o.at_target = false;
        for (k = 1; k <= 2000 && !o.at_target; k++) o = vc_step(&c, LP_PHASE_CLIMB, tgt, false, 0.001f, 0.0f);
        CHECK(k - 1 == n_exact && o.at_target, "from 0.001 m (1000 um): arrival on tick %ld (the snap)", n_exact);
    }
    /* descent rate, command clamps, feed-forward with tilt from the quaternion */
    vc_init(&c, &p);
    o = vc_step(&c, LP_PHASE_HOLD, 2.0f, false, 6.0f, 0.0f);
    CHECK(c.ref_um == 6000000L - p.dn_um && o.dh_ref_mps == -(float)p.dn_um * p.um_tick_to_mps,
          "target below: the reference descends dn_um per tick");
    o = vc_step(&c, LP_PHASE_HOLD, 2.0f, false, 9.0f, 0.0f);
    CHECK(o.vz_cmd == -p.vz_max_down_mps, "v_z command clamped at -vz_max_down");
    CHECK(o.u_ff == p.u_hover, "c = 1: u_ff = u_hover");
    o = vc_step2(&c, LP_PHASE_HOLD, 2.0f, false, 9.0f, 0.0f, vc_cos_of_tilt_deg(30.0), NAN);
    CHECK_NEAR(o.u_ff, (double)p.u_hover / sqrt(cos(30.0 * IL_PI_D / 180.0)), 3e-7, "tilt 30: u_ff = u_hover / sqrt(cos)");
    o = vc_step2(&c, LP_PHASE_HOLD, 2.0f, false, 9.0f, 0.0f, vc_cos_of_tilt_deg(80.0), NAN);
    CHECK_NEAR(o.u_ff, (double)p.u_hover / sqrt((double)p.cos_tilt_min), 2e-7, "tilt 80: feed-forward floored at cos(tilt_abort)");
    CHECK(o.cos_tilt == p.cos_tilt_min, "c reported after the clamp");
    o = vc_step2(&c, LP_PHASE_HOLD, 2.0f, false, 9.0f, 0.0f, NAN, NAN);
    CHECK(o.u_ff == p.u_hover && o.cos_tilt == 1.0f, "c NaN: c = 1");
    o = vc_step2(&c, LP_PHASE_HOLD, 2.0f, false, 9.0f, 0.0f, 1.2f, NAN);
    CHECK(o.cos_tilt == 1.0f, "c > 1 (an unnormalised q): clamped to 1");
    {   /* the powf fallback (another throttle exponent) */
        vc_params_t q = p;
        q.throttle_exp_int = 3;
        q.throttle_exp = 3.0f;
        vc_init(&c, &q);
        o = vc_step2(&c, LP_PHASE_HOLD, 2.0f, false, 2.0f, 0.0f, 0.8f, NAN);
        CHECK_NEAR(o.u_ff, (double)q.u_hover * pow(0.8, -1.0 / 3.0), 3e-7, "n = 3: u_ff = u_hover c^(-1/n) (powf fallback)");
    }
    /* integrator: one step = vz_ki dt e inside the limits; conditional integration at the ceiling; limits */
    vc_init(&c, &p);
    for (k = 0; k < (int)p.ramp_ticks + 1; k++) o = vc_step(&c, LP_PHASE_HOLD, 5.0f, false, 5.0f, 0.0f);
    {
        const float i0 = c.integ;
        const float e_up = (p.u_max + p.vz_i_max - p.u_hover) / p.vz_kp + 0.5f;
        o = vc_step(&c, LP_PHASE_HOLD, 5.0f, false, 5.0f, -0.2f);
        CHECK_NEAR(o.integ - i0, (double)p.vz_ki_dt * 0.2, 1e-8, "integrator step vz_ki dt e");
        CHECK(o.sat == 0 && NEAR(o.u, (double)o.u_ff + p.vz_kp * 0.2 + o.integ, 1e-6), "u = u_ff + vz_kp e + i inside the limits");
        for (k = 0; k < 5000; k++) o = vc_step(&c, LP_PHASE_HOLD, 5.0f, false, 5.0f, -e_up);
        CHECK(o.sat == 1 && o.u == p.u_max && o.integ < p.vz_i_max, "climbing demand saturates at u_max; integrator held "
              "below its limit (conditional integration, integ %.4f)", (double)o.integ);
        for (k = 0; k < 40000; k++) o = vc_step(&c, LP_PHASE_HOLD, 5.0f, false, 5.0f, 0.05f);
        CHECK(o.integ == -p.vz_i_max && o.sat == 0, "integrator limit -vz_i_max");
    }
    /* LAND: the reference steps (dn above the flare, td at / below it - by ref or by h), not bounded by the pad */
    vc_init(&c, &p);
    o = vc_step(&c, LP_PHASE_LAND, 0.0f, false, 3.0f, 0.0f);
    CHECK(c.ref_um == 3000000L - p.dn_um, "LAND above the flare: dn_um per tick");
    vc_init(&c, &p);
    o = vc_step(&c, LP_PHASE_LAND, 0.0f, false, VC_FLARE_ALTITUDE_M, 0.0f);
    CHECK(c.ref_um == p.flare_um - p.td_um, "LAND at the flare altitude (ref == flare_um): td_um");
    vc_init(&c, &p);
    o = vc_step(&c, LP_PHASE_LAND, 0.0f, false, 3.0f, 0.0f);
    o = vc_step(&c, LP_PHASE_LAND, 0.0f, false, 0.2f * VC_FLARE_ALTITUDE_M, 0.0f);
    CHECK(c.ref_um == 3000000L - p.dn_um - p.td_um, "LAND: h below the flare -> td_um even with the reference above it");
    vc_init(&c, &p);
    for (k = 0; k < 3000; k++) o = vc_step(&c, LP_PHASE_LAND, 0.0f, false, 0.0f, 0.0f);
    CHECK(c.ref_um == -3000L * p.td_um, "the LAND reference is not bounded by the pad (ground below it)");
    vc_init(&c, &p);
    o = vc_step(&c, LP_PHASE_LAND, 0.0f, true, 0.2f * VC_FLARE_ALTITUDE_M, 0.0f);
    o = vc_step(&c, LP_PHASE_LAND, 0.0f, true, 0.2f * VC_FLARE_ALTITUDE_M, 0.0f);
    CHECK(c.ref_um == vc_m_to_um(0.2f * VC_FLARE_ALTITUDE_M) - 2 * p.dn_um && (o.sig & VC_SIG_LOW),
          "baro lost: the same law on the dead-reckoned h, dn_um all the way (no flare)");
    /* touchdown signatures (each alone) */
    {
        const float h0 = 0.47f * VC_FLARE_ALTITUDE_M;    /* below the flare, stopped: the reference runs away (not on
                                                          * the um grid of the lead threshold: no knife-edge here) */
        long n_lead = -1, n_lead_c = -1, n_td = -1;
        int32_t r = vc_m_to_um(h0);
        vc_init(&c, &p);
        for (k = 1; k <= 3000 && n_td < 0; k++) {
            o = vc_step(&c, LP_PHASE_LAND, 0.0f, false, h0, 0.0f);
            r -= p.td_um;
            if (n_lead < 0 && (double)p.alt_kp * ((double)r * 1e-6 - h0) <= -(double)p.lead_mps) n_lead = k;
            if (n_lead_c < 0 && o.low_in_land_ticks > 0) n_lead_c = k;
            if (o.touchdown) n_td = k;
        }
        /* with h constant the lead threshold falls exactly on a step (still_lead x td_rate / alt_kp = 400 steps of
         * td_um): the double and the float32 test may decide that tick differently - one tick either way */
        CHECK(n_lead > 0 && labs(n_lead_c - n_lead) <= 1 && n_td == n_lead_c + (long)p.landed_confirm_ticks - 1 &&
              o.td_by == VC_TD_STILL, "still: the reference lead (%ld ticks; double %ld), then landed_confirm_ticks -> "
              "touchdown by still (tick %ld)", n_lead_c, n_lead, n_td);
        CHECK(o.low_in_land_ticks == (uint16_t)(n_td - n_lead_c + 1) && (o.sig & VC_SIG_LOW),
              "low_in_land counts low and leading ticks (%u)", o.low_in_land_ticks);
        o = vc_step(&c, LP_PHASE_LAND, 0.0f, false, h0, 0.3f);
        CHECK(o.touchdown, "the still count is not reset by one tick of v_z: it is the low-passed v_z (|v_lpf| %.3f)",
              (double)c.v_lpf);
        {   /* v0.9 gains: vz_kp 0.05 leaves u near 0.79 in the pinned descent - the support signature (d > 0.25 g needs r <
             * 0.75) is then a knife edge; the v0.8.1 gain 0.08 keeps this test's premise (the signature logic is the same) */
            vc_params_t p8 = p;
            p8.vz_kp = 0.08f;
            vc_init(&c, &p8);
        }
        o.touchdown = false;
        for (k = 1; k <= 5000 && !o.touchdown; k++) o = vc_step(&c, LP_PHASE_LAND, 0.0f, false, 3.0f * VC_FLARE_ALTITUDE_M, 0.0f);
        CHECK(!o.touchdown && o.td_count == 0u && (o.sig & VC_SIG_LOW) && (o.sig & VC_SIG_STILL) && (o.sig & VC_SIG_FLOOR) &&
              !(o.sig & VC_SIG_LOW_ALT) && c.cnt[0] == 0u && c.cnt[2] == 0u,
              "v0.7.1b: 3 flare heights up (ground above the pad, no f_z): still and floor, but above the band only all "
              "three together confirm - no touchdown in %d ticks (sig 0x%02x)", k - 1, o.sig);
        for (k = 1; k <= 8000 && !o.touchdown; k++)   /* v0.9: vz_ki 0.008 (was 0.0128) pins the integrator later */
            o = vc_step2(&c, LP_PHASE_LAND, 0.0f, false, 3.0f * VC_FLARE_ALTITUDE_M, 0.0f, 1.0f, 9.80665f);
        CHECK(o.touchdown && o.td_by == VC_TD_ALL && (o.sig & VC_SIG_SUPPORT) && (o.sig & VC_SIG_FLOOR) &&
              (o.sig & VC_SIG_STILL) && c.cnt[3] == p.landed_confirm_ticks,
              "... with the ground carrying it (f_z = g): support + floor + still -> touchdown by all three after %d ticks "
              "(td_by %u)", k - 1, o.td_by);
        vc_init(&c, &p);
        o.touchdown = false;
        for (k = 1; k <= 3000 && !o.touchdown; k++)
            o = vc_step2(&c, LP_PHASE_LAND, 0.0f, false, 0.9f * VC_FLARE_ALTITUDE_M, 0.2f, 1.0f, 2.0f * 9.80665f);
        CHECK(o.touchdown && o.td_by == VC_TD_SUPPORT && (o.sig & VC_SIG_GROUND) && k - 1 < 200,
              "support: f_z 2 g with the motor near hover -> touchdown by support after %d ticks", k - 1);
        vc_init(&c, &p);
        o.touchdown = false;
        for (k = 1; k <= 3000 && !o.touchdown; k++)
            o = vc_step2(&c, LP_PHASE_LAND, 0.0f, false, 0.9f * VC_FLARE_ALTITUDE_M, 0.2f, 1.0f, NAN);
        CHECK(o.touchdown && o.td_by == VC_TD_FLOOR && (o.sig & VC_SIG_GROUND) && c.integ == -p.vz_i_max &&
              o.vz_cmd == -p.vz_max_down_mps, "floor: integrator at -vz_i_max and v_cmd at -vz_max_down for floor_ticks "
              "(no f_z, rising v_z: no still / support) -> touchdown by floor after %d ticks", k - 1);
        vc_init(&c, &p);
        for (k = 0; k < 2000; k++) o = vc_step2(&c, LP_PHASE_HOLD, 2.0f, false, 2.0f, 0.0f, 1.0f, 3.0f * 9.80665f);
        CHECK(!o.touchdown && o.td_count == 0 && o.low_in_land_ticks == 0 && (o.sig & VC_SIG_SUPPORT) && !(o.sig & VC_SIG_LOW),
              "FLY: no counting (the signature itself is reported for diagnostics)");
        vc_init(&c, &p);
        for (k = 0; k < 2000; k++) o = vc_step(&c, LP_PHASE_LAND, 0.0f, true, 0.2f, 0.0f);
        CHECK(o.low_in_land_ticks == 0, "baro lost: no V10 count");
    }
    /* non-finite inputs never leave [0, u_max]; random bounds */
    vc_init(&c, &p);
    o = vc_step2(&c, LP_PHASE_CLIMB, 5.0f, false, NAN, NAN, INFINITY, NAN);
    CHECK(c.ref_um == p.up_um && o.vz_cmd != o.vz_cmd && o.vz_err == 0.0f && o.u >= 0.0f && o.u <= p.u_max,
          "NaN inputs: the reference starts at 0, v_z command NaN, e = 0, u bounded");
    {
        long bad = 0;
        vc_init(&c, &p);
        for (k = 0; k < 50000; k++) {
            static const uint8_t ph[5] = {LP_PHASE_CLIMB, LP_PHASE_HOLD, LP_PHASE_LAND, LP_PHASE_LAND, LP_PHASE_NONE};
            const uint8_t phase = ph[(int)(urand() * 5.0) % 5];
            o = vc_step2(&c, phase, (float)(2.0 + 18.0 * urand()), urand() < 0.2, (float)(-3.0 + 33.0 * urand()),
                         (float)(4.0 * srand1()), (float)urand(), (float)(30.0 * urand()));
            if (!(o.u >= 0.0f && o.u <= p.u_max && fabsf(o.integ) <= p.vz_i_max) || (phase == LP_PHASE_NONE && o.u != 0.0f))
                bad++;
        }
        CHECK(bad == 0, "50000 random steps: u in [0, u_max], |integ| <= vz_i_max, IDLE u = 0 (%ld bad)", bad);
    }
}

/* ---- v0.7.1b: thrust learning, the band gating of the touchdown detector, the VTOL attitude overrides ---- */
/* r of the next update (the controller's own thrust-lag step, in float32 as the core does it) */
static float vc_r_next(const vc_t *c)
{
    const float x = c->u_last * c->p.inv_u_hover;
    return c->r_thr + c->p.thrust_alpha * (x * x - c->r_thr);
}

static void test_vc_thrust_learning(void)
{
    vc_params_t p;
    vc_t c;
    vc_outputs_t o;
    int k;
    const double dt = 1.0 / FC_LOOP_HZ;
    const float hi = 5.0f;                               /* above the flare band in every profile */
    vc_default(&p);
    CHECK_NEAR(p.kt_alpha, 1.0 - exp(-dt / (double)VC_KT_TAU_S), 1e-9, "kt_alpha = 1 - exp(-dt / tau)");
    CHECK_NEAR(p.kt_cos_min, cos((double)VC_KT_MAX_TILT_DEG * IL_PI_D / 180.0), 1e-7, "kt_cos_min = cos(max_tilt_deg)");
    CHECK(p.kt_min > 0.0f && p.kt_min <= 1.0f && p.kt_max >= 1.0f && VC_KT_R_MIN == 0.25f,
          "k_T limits around 1 (k_min %.3f, k_max %.3f), the r guard 0.25", (double)p.kt_min, (double)p.kt_max);
    vc_init(&c, &p);
    CHECK(c.k_t == 1.0f, "k_T = 1 before the take-off");
    for (k = 0; k < (int)p.ramp_ticks + 1; k++) o = vc_step2(&c, LP_PHASE_HOLD, hi, false, hi, 0.0f, 1.0f, NAN);
    CHECK(c.k_t == 1.0f && !(o.sig & VC_SIG_KT_LEARN), "no f_z: nothing learnt");
    {   /* one learning step by hand: k += alpha (f_z / (g r) - k), d with k_T g r; u_ff of the next tick */
        const float r1 = vc_r_next(&c), d0 = c.d_lpf;
        const float fz = 1.3f * 9.80665f * r1;
        const float gr = p.g * r1;
        const float k1 = 1.0f + p.kt_alpha * (fz / gr - 1.0f);
        o = vc_step2(&c, LP_PHASE_HOLD, hi, false, hi, 0.0f, 1.0f, fz);
        CHECK((o.sig & VC_SIG_KT_LEARN) && c.k_t == k1 && o.u_ff == p.u_hover, "learning step k_T %.9g (want %.9g); this "
              "tick's u_ff still the previous k_T", (double)c.k_t, (double)k1);
        CHECK(c.d_lpf == d0 + p.support_alpha * ((fz - k1 * gr) - d0), "d = f_z - k_T (g r) with this tick's k_T");
        o = vc_step2(&c, LP_PHASE_HOLD, hi, false, hi, 0.0f, 1.0f, 1.3f * 9.80665f * vc_r_next(&c));
        CHECK(o.u_ff == p.u_hover / sqrtf(k1 * 1.0f), "u_ff = u_hover / sqrtf(k_T c)");
    }
    {   /* 3 tau of a constant ratio 1.3: within 5 % of the step (the lag (1 - a)^N in double) */
        const int n3 = (int)floor(3.0 * (double)VC_KT_TAU_S * FC_LOOP_HZ + 0.5);
        vc_init(&c, &p);
        for (k = 0; k < (int)p.ramp_ticks + 1; k++) o = vc_step2(&c, LP_PHASE_HOLD, hi, false, hi, 0.0f, 1.0f, NAN);
        for (k = 0; k < n3; k++) o = vc_step2(&c, LP_PHASE_HOLD, hi, false, hi, 0.0f, 0.99f, 1.3f * 9.80665f * vc_r_next(&c));
        CHECK_NEAR(c.k_t, 1.3 - 0.3 * pow(1.0 - (double)p.kt_alpha, n3), 2e-5, "k_T after 3 tau (float32 lag vs closed form)");
        CHECK(fabs(c.k_t - 1.3f) < 0.05f * 0.3f, "3 tau: within 5 %% of the step (k_T %.5f)", (double)c.k_t);
        for (k = 0; k < 6000; k++) o = vc_step2(&c, LP_PHASE_HOLD, hi, false, hi, 0.0f, 1.0f, 9.0f * 9.80665f * vc_r_next(&c));
        CHECK(c.k_t == p.kt_max, "clamped at k_max");
        for (k = 0; k < 9000; k++) o = vc_step2(&c, LP_PHASE_HOLD, hi, false, hi, 0.0f, 1.0f, 0.1f * 9.80665f * vc_r_next(&c));
        CHECK(c.k_t == p.kt_min, "clamped at k_min");
        o = vc_step(&c, LP_PHASE_LANDED, hi, false, hi, 0.0f);
        CHECK(c.k_t == 1.0f && o.mode == VC_MODE_IDLE, "IDLE (each take-off): k_T back to 1");
    }
    {   /* frozen: each condition alone */
        struct { uint8_t phase; float h, cs, fz; const char *what; } cs[] = {
            {LP_PHASE_HOLD, VC_FLARE_ALTITUDE_M, 1.0f, 19.6f, "h == flare (the band, inclusive)"},
            {LP_PHASE_LAND, 0.5f * VC_FLARE_ALTITUDE_M, 1.0f, 19.6f, "LAND in the band"},
            {LP_PHASE_HOLD, hi, VC_KT_COS_MIN, 19.6f, "tilt == max_tilt_deg"},
            {LP_PHASE_HOLD, hi, NAN, 19.6f, "no attitude (c not finite)"},
            {LP_PHASE_HOLD, hi, 1.0f, NAN, "no f_z"},
            {LP_PHASE_HOLD, hi, 1.0f, INFINITY, "f_z infinite"}};
        unsigned i;
        for (i = 0; i < sizeof cs / sizeof cs[0]; i++) {
            vc_init(&c, &p);
            for (k = 0; k < (int)p.ramp_ticks + 1; k++) o = vc_step2(&c, LP_PHASE_HOLD, hi, false, hi, 0.0f, 1.0f, NAN);
            c.k_t = 1.1f;
            o = vc_step2(&c, cs[i].phase, hi, false, cs[i].h, 0.0f, cs[i].cs, cs[i].fz);
            CHECK(c.k_t == 1.1f && !(o.sig & VC_SIG_KT_LEARN), "frozen: %s", cs[i].what);
        }
        vc_init(&c, &p);
        for (k = 0; k < (int)p.ramp_ticks + 1; k++) o = vc_step2(&c, LP_PHASE_HOLD, hi, false, hi, 0.0f, 1.0f, NAN);
        c.k_t = 1.1f;
        o = vc_step2(&c, LP_PHASE_LAND, hi, false, hi, 0.0f, 1.0f, 19.6f);
        CHECK(c.k_t != 1.1f && (o.sig & VC_SIG_KT_LEARN), "LAND above the band: free flight, learns");
        vc_init(&c, &p);
        c.k_t = 1.1f;
        o = vc_step2(&c, LP_PHASE_CLIMB, hi, false, hi, 0.0f, 1.0f, 19.6f);
        CHECK(c.k_t == 1.0f && !(o.sig & VC_SIG_KT_LEARN), "the take-off ramp: frozen (and the start resets k_T)");
        vc_init(&c, &p);
        for (k = 0; k < (int)p.ramp_ticks + 1; k++) o = vc_step2(&c, LP_PHASE_HOLD, hi, false, hi, 0.0f, 1.0f, NAN);
        c.u_last = 0.0f;
        c.r_thr = 0.2f;
        o = vc_step2(&c, LP_PHASE_HOLD, hi, false, hi, 0.0f, 1.0f, 19.6f);
        CHECK(c.k_t == 1.0f && !(o.sig & VC_SIG_KT_LEARN), "r < VC_KT_R_MIN: frozen (the division guard)");
        {
            vc_params_t q = p;
            q.kt_alpha = 0.0f;
            vc_init(&c, &q);
            for (k = 0; k < 2000; k++) o = vc_step2(&c, LP_PHASE_HOLD, hi, false, hi, 0.0f, 1.0f, 19.6f);
            CHECK(c.k_t == 1.0f && o.u_ff == q.u_hover, "kt_alpha 0: no learning at all");
            q = p;
            q.kt_min = q.kt_max = 0.7f;
            vc_init(&c, &q);
            CHECK(c.k_t == 0.7f, "limits that exclude 1: k_T starts at the clamp");
        }
    }
    {   /* the band gating: above it support + floor without still never confirms (baro healthy).
         * fw09 WP-X: the stimulus must reach the floor signature in every vehicle configuration (-D FC_VCFG: u_hover 0.73 /
         * 0.67 / 0.70 against the rocket's 0.92).  With v_z 0.4 the floor comes from the integrator pinned at -vz_i_max
         * (u_ff + vz_kp e - vz_i_max > u_min: the rocket); where that sum is not above u_min the conditional integration
         * stops the integrator just above u_min (u -> u_min with sat 0: neither pinned path - measured with the battery
         * set, u_hover 0.730), so the vehicle climbs faster there until u_raw is held below u_min (sat -1), the other
         * pinned path.  Same assertions, the default (rocket) build's stimulus unchanged. */
        const float u_ff_lo = p.u_hover / sqrtf(p.kt_max);  /* f_z = 2 g: k_T learns up to its clamp (above the band) */
        float vz_up = 0.4f;
        if (!(u_ff_lo - p.vz_kp * (p.vz_max_down_mps + vz_up) - p.vz_i_max > p.u_min + 0.01f))
            vz_up = (u_ff_lo + p.vz_i_max - p.u_min + 0.05f) / p.vz_kp - p.vz_max_down_mps;   /* u_raw < u_min - 0.05 */
        vc_init(&c, &p);
        for (k = 0; k < (int)p.ramp_ticks + 1; k++) o = vc_step2(&c, LP_PHASE_HOLD, hi, false, hi, 0.0f, 1.0f, NAN);
        o.touchdown = false;
        for (k = 1; k <= 6000 && !o.touchdown; k++)
            o = vc_step2(&c, LP_PHASE_LAND, 0.0f, false, 3.0f * VC_FLARE_ALTITUDE_M, vz_up, 1.0f, 2.0f * 9.80665f);
        CHECK(!o.touchdown && (o.sig & VC_SIG_SUPPORT) && (o.sig & VC_SIG_FLOOR) && !(o.sig & VC_SIG_STILL) &&
              c.cnt[3] == 0u && o.low_in_land_ticks == 0u, "above the band, moving (v_z %.2f): support + floor, no "
              "touchdown, no V10 count (sig 0x%02x)", (double)vz_up, o.sig);
        o.touchdown = false;
        for (k = 1; k <= 3000 && !o.touchdown; k++)
            o = vc_step2(&c, LP_PHASE_LAND, 0.0f, true, 3.0f * VC_FLARE_ALTITUDE_M, vz_up, 1.0f, 2.0f * 9.80665f);
        CHECK(o.touchdown && o.td_by == VC_TD_ALL && !(o.sig & VC_SIG_STILL) && k - 1 == (int)p.landed_confirm_ticks,
              "baro lost: support + floor confirm above the band without still (%d ticks, td_by %u)", k - 1, o.td_by);
        o = vc_step2(&c, LP_PHASE_LAND, 0.0f, false, VC_FLARE_ALTITUDE_M, vz_up, 1.0f, 2.0f * 9.80665f);
        CHECK((o.sig & VC_SIG_LOW_ALT) && c.cnt[1] == 1u && c.cnt[2] == 1u,
              "h == flare: in the band (inclusive), the single counters start from 1");
    }
}

static void test_vtol_gains(void)
{
    att_gains_t b, v;
    memset(&b, 0, sizeof b);                         /* padding: compare field by field below anyway */
    memset(&v, 0, sizeof v);
    fc_default_gains(&b);
    fc_vtol_gains(&v);
#ifdef VTOL_OV_RATE_I_MAX_Z
    CHECK(v.rate.rate_i_max_z == VTOL_OV_RATE_I_MAX_Z && b.rate.rate_i_max_z == GAIN_RATE_I_MAX_Z,
          "fc_vtol_gains: rate_i_max_z %.3f (ballistic %.3f)", (double)v.rate.rate_i_max_z, (double)b.rate.rate_i_max_z);
    v.rate.rate_i_max_z = b.rate.rate_i_max_z;
#endif
    /* v0.9: every VTOL_OV_* override field, then the rest equal to the ballistic gains */
#define OVR(macro, field) v.field = b.field;
#ifdef VTOL_OV_ATT_KP_XY
    OVR(VTOL_OV_ATT_KP_XY, att_kp_xy)
#endif
#ifdef VTOL_OV_ATT_KP_Z
    OVR(VTOL_OV_ATT_KP_Z, att_kp_z)
#endif
#ifdef VTOL_OV_RATE_KP_XY
    CHECK(v.rate.rate_kp_xy == VTOL_OV_RATE_KP_XY, "fc_vtol_gains: rate_kp_xy");
    OVR(VTOL_OV_RATE_KP_XY, rate.rate_kp_xy)
#endif
#ifdef VTOL_OV_RATE_KI_XY
    CHECK(v.rate.rate_ki_xy == VTOL_OV_RATE_KI_XY, "fc_vtol_gains: rate_ki_xy");
    OVR(VTOL_OV_RATE_KI_XY, rate.rate_ki_xy)
#endif
#ifdef VTOL_OV_RATE_KD_XY
    CHECK(v.rate.rate_kd_xy == VTOL_OV_RATE_KD_XY, "fc_vtol_gains: rate_kd_xy");
    OVR(VTOL_OV_RATE_KD_XY, rate.rate_kd_xy)
#endif
#ifdef VTOL_OV_RATE_KP_Z
    CHECK(v.rate.rate_kp_z == VTOL_OV_RATE_KP_Z, "fc_vtol_gains: rate_kp_z");
    OVR(VTOL_OV_RATE_KP_Z, rate.rate_kp_z)
#endif
#ifdef VTOL_OV_RATE_KI_Z
    CHECK(v.rate.rate_ki_z == VTOL_OV_RATE_KI_Z, "fc_vtol_gains: rate_ki_z");
    OVR(VTOL_OV_RATE_KI_Z, rate.rate_ki_z)
#endif
#undef OVR
#if FC_V09
    CHECK(v.v9.err_mode == VTOL_V09_ERR_MODE && v.v9.ff_a == VTOL_V09_FF_A && v.v9.ff_nkr == VTOL_V09_FF_NKR &&
          v.v9.ff_h == VTOL_V09_FF_H && ((v.v9.flags & ATT_V9_AW) != 0) == (VTOL_V09_AW_PER_AXIS != 0) &&
          ((v.v9.flags & ATT_V9_TRIM) != 0) == (VTOL_V09_YAW_TRIM_RESERVE != 0) &&
          ((v.v9.flags & ATT_V9_FF) != 0) == (VTOL_V09_FF_ON != 0) && b.v9.err_mode == 0 && b.v9.flags == 0,
          "fc_vtol_gains: the v0.9 options (err_mode %u, flags 0x%02X); fc_default_gains: all off", v.v9.err_mode,
          v.v9.flags);
#endif
    CHECK(v.loop_hz == b.loop_hz && v.att_kp_xy == b.att_kp_xy && v.att_kp_z == b.att_kp_z &&
          v.rate_sp_max_xy_rad_s == b.rate_sp_max_xy_rad_s && v.rate_sp_max_z_rad_s == b.rate_sp_max_z_rad_s &&
          v.rate.rate_kp_xy == b.rate.rate_kp_xy && v.rate.rate_ki_xy == b.rate.rate_ki_xy &&
          v.rate.rate_kd_xy == b.rate.rate_kd_xy && v.rate.rate_i_max_xy == b.rate.rate_i_max_xy &&
          v.rate.rate_kp_z == b.rate.rate_kp_z && v.rate.rate_ki_z == b.rate.rate_ki_z && v.rate.rate_kd_z == b.rate.rate_kd_z &&
          v.rate.rate_i_max_z == b.rate.rate_i_max_z && v.rate.d_term_lpf_hz == b.rate.d_term_lpf_hz &&
          v.rate.gyro_lpf_hz == b.rate.gyro_lpf_hz && v.thrust_estimate_floor_fraction == b.thrust_estimate_floor_fraction &&
          v.roll_priority_scale == b.roll_priority_scale && v.yaw_hold == b.yaw_hold,
          "fc_vtol_gains = fc_default_gains except the VTOL_OV_* fields (%d override)", VTOL_OV_COUNT);
}

/* ---- the FCU stack with a 1-D vertical plant (double precision, the reference test's Plant1D) ---- */
typedef struct {
    double z, v, T;
    bool ground;
    double impact;                 /* the first ground contact's vertical speed */
    int contacts;
} plant1d_t;

static void plant1d_step(plant1d_t *pl, double u, double dt)
{
    const double m = VEH_MASS_KG, tmax = MIX_T_MAX_N, tau = EDF_TAU_S, n = MIX_THROTTLE_EXP;
    double a;
    pl->T += (tmax * pow(u > 0.0 ? u : 0.0, n) - pl->T) * (1.0 - exp(-dt / tau));
    a = pl->T / m - 9.80665;
    if (pl->ground && a <= 0.0) {
        pl->v = 0.0;
        return;
    }
    pl->ground = false;
    pl->v += a * dt;
    pl->z += pl->v * dt;
    if (pl->z <= 0.0) {
        if (pl->contacts++ == 0) pl->impact = pl->v;
        pl->z = 0.0;
        pl->v = 0.0;
        pl->ground = true;
    }
}

/* body-z specific force of the 1-D plant: thrust / m in the air, g resting on the ground */
static double plant1d_fz(const plant1d_t *pl)
{
    return pl->ground ? 9.80665 : pl->T / VEH_MASS_KG;
}

static void test_stack(void)
{
    static onb_stack_t k;
    onb_params_t p;
    vc_params_t vp;
    onb_outputs_t so;
    plant1d_t pl = {0.0, 0.0, 0.0, true, 0.0, 0};
    onb_frame_t f;
    onb_inputs_t in;
    uint32_t t;
    uint8_t seq = 10, prev_phase = 0;
    int n_hold = -1, n_land = -1, n_landed = -1, n_safe = -1, n_on = 50, n_up = -1, n_off = -1, bad_order = 0, bad_esc = 0;
    int bad_att = 0, td_by = -1;
    double max_h = 0.0, prev_u = 0.0;
    bool prev_td = false, prev_at = false, prev_gs = false, prev_active = false;
    uint16_t prev_lil = 0;
    onb_params_mode(&p, LP_MODE_VTOL);
    p.gps_hold = false;                                /* the v0.7.1 VTOL stack (v0.8: test_stack_v08) */
    p.takeoff_check = false;
    fc_default_vertical(&vp);
    vp.takeoff_ticks = 0;
    onb_stack_init(&k, &p, &vp, 0, ONB_CHAIN_GNS, 7, false);
    memset(&in, 0, sizeof in);
    for (t = 0; t < 40000u && n_safe < 0; t++) {
        uint8_t cmd = 0xFF;
        float u;
        if ((int)t == n_on) cmd = LP_CMD_ON;
        if (n_hold > 0 && (int)t == n_hold + 1000) { cmd = LP_CMD_UP; n_up = (int)t; }
        if (n_hold > 0 && (int)t == n_hold + 2500) { cmd = LP_CMD_OFF; n_off = (int)t; }
        in.tick = t;
        in.n_frames = 0;
        if (cmd != 0xFF || t % 50u == 0u) {
            if (cmd != 0xFF) seq++;
            f.seq = seq;
            f.cmd.cmd = cmd == 0xFF ? LP_CMD_NONE : cmd;
            f.cmd.arg = 0;
            f.cmd.nonce = k.sup.nonce;
            in.frames = &f;
            in.n_frames = 1;
        }
        in.h_m = (float)pl.z;
        in.vz_mps = (float)pl.v;
        in.tilt_deg = 0.3f;
        {
            float q[4];
            q[0] = (float)cos(0.3 * IL_PI_D / 360.0);
            q[1] = (float)sin(0.3 * IL_PI_D / 360.0);
            q[2] = q[3] = 0.0f;
            in.cos_tilt = vc_cos_tilt_from_q(q);       /* a v2 frame's q -> cos(tilt), the v0.7.1 arithmetic */
        }
        in.accel_z_mps2 = (float)plant1d_fz(&pl);
        in.v_cell = 4.0f;
        in.imu_ok = in.baro_ok = in.aligned = in.arm_switch = true;
        /* (1) the attitude controller runs first with the previous tick's control_active and ESC throttle */
        if (onb_stack_att_active(&k) != prev_active || onb_stack_att_u(&k) != (float)prev_u) bad_att++;
        u = onb_stack_step(&k, &in, &so);
        if (in.vc_touchdown != prev_td || in.vc_at_target != prev_at || in.throttle_prev != (float)prev_u ||
            in.vc_ground_signature != prev_gs || in.vc_low_in_land_ticks != prev_lil)
            bad_order++;
        if (u != (so.motor_on ? k.vo.u : 0.0f)) bad_esc++;
        if (so.phase == LP_PHASE_HOLD && prev_phase == LP_PHASE_CLIMB) {
            n_hold = (int)t;
            CHECK(prev_at, "CLIMB -> HOLD the tick after the controller's at_target");
        }
        if (so.phase == LP_PHASE_LAND && prev_phase != LP_PHASE_LAND) n_land = (int)t;
        if (so.phase == LP_PHASE_LANDED && prev_phase == LP_PHASE_LAND) {
            n_landed = (int)t;
            CHECK(prev_td && u == 0.0f && so.trans[0].cause == ONB_CAUSE_TOUCHDOWN,
                  "LAND -> LANDED the tick after touchdown; motor off at once");
        }
        if (k.vo.touchdown && td_by < 0) td_by = k.vo.td_by;
        if (so.state == ONB_SAFE && n_landed > 0 && n_safe < 0) n_safe = (int)t;
        prev_td = k.vo.touchdown;
        prev_at = k.vo.at_target;
        prev_gs = (k.vo.sig & VC_SIG_GROUND) != 0;
        prev_lil = k.vo.low_in_land_ticks;
        prev_active = so.control_active;
        prev_u = u;
        prev_phase = so.phase;
        plant1d_step(&pl, u, (double)FC_DT_S);
        if (pl.z > max_h) max_h = pl.z;
    }
    CHECK(bad_order == 0 && bad_esc == 0 && bad_att == 0, "stack order (v0.7.1): attitude inputs = the previous tick's "
          "control_active / ESC throttle; the supervisor sees the previous tick's flags and ESC throttle; ESC = u while "
          "motor_on (%d / %d / %d bad)", bad_att, bad_order, bad_esc);
    CHECK(n_hold > n_on + (int)p.spool_ticks && n_land == n_off && n_landed > n_land && n_safe == n_landed +
          (int)p.safe_after_landed_ticks, "phases: ON %d, HOLD %d, UP %d, OFF / LAND %d, LANDED %d, SAFE %d", n_on, n_hold, n_up,
          n_land, n_landed, n_safe);
    CHECK(so.reason == LP_REASON_NONE && !so.release, "no interlock in a nominal hop");
    CHECK(max_h > 0.95 * (p.hover_dm + p.alt_step_dm) / 10.0 && max_h < 1.1 * (p.hover_dm + p.alt_step_dm) / 10.0 + 0.2 &&
          pl.impact > -1.0 && pl.impact < 0.0, "1-D plant: peak %.2f m (target %.1f m after UP), touchdown %.3f m/s", max_h,
          (p.hover_dm + p.alt_step_dm) / 10.0, pl.impact);
    printf("info: VTOL stack on a 1-D plant: take-off tick %d, HOLD %d, LAND %d, LANDED %d (touchdown by %s), peak %.3f m, "
           "first touchdown %.3f m/s (%d ground contacts)\n", n_on + (int)p.spool_ticks, n_hold, n_land, n_landed,
           td_by == VC_TD_STILL ? "still" : td_by == VC_TD_SUPPORT ? "support" : td_by == VC_TD_FLOOR ? "floor" : "?",
           max_h, pl.impact, pl.contacts);
    {   /* ballistic: the stack passes the supervisor's slewed throttle, the vertical controller does not run */
        bad_esc = 0;
        onb_params_mode(&p, LP_MODE_BALLISTIC);
        onb_stack_init(&k, &p, &vp, 0, ONB_CHAIN_GNS, 7, false);
        memset(&in, 0, sizeof in);
        in.imu_ok = in.baro_ok = in.aligned = in.arm_switch = true;
        in.tilt_deg = 0.5f;
        in.v_cell = 4.1f;
        for (t = 0; t < 400u; t++) {
            float u;
            in.tick = t;
            in.n_frames = 0;
            if (t % 20u == 0u || t == 30u || t == 31u) {
                f.seq = (uint8_t)(t == 30u ? 1u : t == 31u ? 2u : 0u);
                f.cmd.cmd = t == 30u ? LP_CMD_ARM : t == 31u ? LP_CMD_LAUNCH : LP_CMD_NONE;
                f.cmd.nonce = k.sup.nonce;
                f.cmd.arg = 0;
                if (t > 31u) f.seq = 2u;
                in.frames = &f;
                in.n_frames = 1;
            }
            u = onb_stack_step(&k, &in, &so);
            if (u != (float)so.throttle_milli / 1000.0f || k.have_vo || onb_stack_att_active(&k) != so.control_active)
                bad_esc++;
        }
        CHECK(bad_esc == 0 && so.state == ONB_POWERED && so.throttle_milli == p.climb_milli, "ballistic stack: ESC = slewed "
              "supervisor throttle, no vertical controller, attitude after the supervisor with this tick's values");
    }
}

/* ---- v0.7.1 VTOL NAV estimator (GatedMahonyEstimator) and altitude filter (VtolAltitudeFilter) ---- */
static void test_mahony(void)
{
    att_mahony_t m;
    att_est_t e;
    float g[3], a[3], q_pad[4];
    int k, i;
    const float g0 = (float)FC_G0;
    fc_default_mahony(&m);
    att_est_init(&e, FC_DT_S, GAIN_EST_PAD_N_AVG);
    CHECK(m.kp == EST_V_KP && m.ki_dt == EST_V_KI_DT && m.gate_mps2 == EST_V_GATE_MPS2 && m.pad.n_avg == GAIN_EST_PAD_N_AVG,
          "fc_default_mahony: params_gen.h EST_V_*, gains.h N_avg");
    CHECK_NEAR(m.alpha_a, 1.0 - exp(-2.0 * IL_PI_D * EST_V_ACCEL_LPF_HZ / FC_LOOP_HZ), 1e-7, "alpha_a = 1 - exp(-2 pi f dt)");
    /* pad: bit for bit the PadAlignGyroEstimator pad branch */
    for (k = 0; k < 700; k++) {
        g[0] = 0.01f + 0.001f * (float)srand1(); g[1] = -0.004f; g[2] = 0.002f * (float)srand1();
        a[0] = 0.3f + 0.05f * (float)srand1(); a[1] = -0.2f; a[2] = 1.3f * 9.78f + 0.05f * (float)srand1();
        att_mahony_update(&m, g, a, true);
        att_est_update(&e, g, a, true);
    }
    CHECK(!memcmp(m.pad.q, e.q, sizeof e.q) && !memcmp(m.pad.bias, e.bias, sizeof e.bias) &&
          !memcmp(m.pad.f_mean, e.f_mean, sizeof e.f_mean) && m.pad.n == e.n && m.pad.aligned,
          "pad branch identical to att_est_update (q, bias, f_mean, n)");
    CHECK(!memcmp(m.a_lpf, e.f_mean, sizeof m.a_lpf) && !memcmp(m.b_pad, e.bias, sizeof m.b_pad) && !m.gate && m.e[0] == 0.0f,
          "pad: a_lpf = f_hat, b_pad = b_hat, e = 0, gate closed");
    memcpy(q_pad, m.pad.q, sizeof q_pad);
    /* flight, gate closed (|a| = 1.3 g from the pad on, the take-off): gyro-only like att_est_update in flight.
     *
     * fw09 T1 (SPEC_firmware_v09 WP-X) - why "within 2e-6" and not bit for bit, and where 2e-6 comes from.  The VTOL
     * flight integration runs in Q30 / Q32 fixed point (core/qfix.h, att_estimator.c quat_integrate_fx); the ballistic
     * gyro-only estimator att_est_update keeps the float32 quat_integrate.  Both integrate the SAME float32 rate
     * w = gyro - b_pad (formed by the same float subtraction; b_pad is not touched while the gate is closed - checked
     * exactly below) with the same series and the same Newton renormalisation; only the number format differs, so bit
     * identity is impossible without changing one of the two estimators (not a test's business: the ballistic image is
     * fixed).  The check is split in two bounds against the exact integration qd (float64, exact exp(w dt / 2), exact
     * norm) of that same rate sequence, and 2e-6 >= B_X + B_F follows by the triangle inequality.  Stimulus: N = 400
     * ticks, |w| = 0.055 rad/s, dt = 4 ms (|rv| = 2.2e-4 rad a tick), |q_pad - (1, 0, 0, 0)| ~ 0.014, so the vector part
     * stays below 2^-4 (0.0625) and q_0 in (0.99, 1).  LSB30 = 2^-30 = 9.31e-10, LSB32 = 2^-32 = 2.33e-10.
     *  - Radial errors (a common scale of the four components) do not accumulate: the Newton step every tick removes
     *    them up to that tick's own rounding.  They show up in q_0 only: X <= 8 LSB30 (products) + 4 x 2 LSB28 / 2 (|r|^2
     *    in Q28, halved into eps) + 1/2 ulp(q_0 < 1) (the Q30 -> float copy) = 7.5e-9 + 1.5e-8 + 3.0e-8 < 6e-8;
     *    F <= 4 x 2^-25 (|r|^2 sum) / 2 + 2^-24 (1.5 - |r|^2 / 2) + 2^-25 (r s) = 6.0e-8 + 6.0e-8 + 3.0e-8 = 1.5e-7;
     *    the direction error below couples into q_0 only times |q_vec| (< 0.0625).
     *  - Tangential errors (i = 1..3) accumulate over the N ticks.  F (round to nearest, zero-mean): per tick 5 roundings
     *    of a value < 2^-4 (q_i dq_0, three sums, the Newton product), each <= 2^-29 = 1.86e-9; the products with dq_k
     *    (|dq_k| <= 1.1e-4) round at <= 6.6e-12.  Taken as independent bounded terms, Hoeffding with N x 5 = 2000 terms
     *    gives |sum| <= 2^-29 sqrt(2 x 2000 x ln(2 / 1e-9)) = 5.4e-7 except with probability 1e-9; the float32 rv = w dt
     *    is rounded the same way every tick (constant w): <= N x 2^-24 x 2.2e-4 / 2 = 2.6e-9.  B_F = 6e-7 (all four).
     *    X (truncating products, NOT zero-mean): per tick the four products of p_i and the Newton product.  Systematic
     *    parts, worst case: fx_mulhi(q_i, dl) is below 1 LSB30 (|q_i| dl < 0.0625 x 6.1e-9) and floors with the sign of
     *    q_i every tick (<= 1 LSB30); fx_mulhi(p_i, eps) floors to -1 LSB30 whenever p_i eps < 0 (<= 1 LSB30, plus the
     *    dropped-byte case floor - 1, probability < 2^-7: counted in); the two remaining products of p_i with a varying
     *    fraction carry the floor's mean -1/2 LSB30 each, net <= 1/2 LSB30 with their signs.  Sum: 2.5 LSB30 a tick =
     *    N x 2.5 x 9.31e-10 = 9.3e-7.  Their zero-mean remainder (3 terms of range 1 LSB30): Hoeffding
     *    LSB30 sqrt(N x 3 x ln(2e9) / 2) = 1.1e-7.  The Q32 truncation of rv (<= 1 LSB32 per axis, the same every tick):
     *    N x LSB32 / 2 = 4.7e-8; fx_from_f of q_pad (<= 1 LSB30) and the final copy (<= 2^-29): 2.8e-9.  B_X = 1.1e-6.
     *  B_X + B_F = 1.7e-6 <= 2e-6: the original bound is kept, and each estimator is checked against qd on every tick.
     *  (The worst case with every truncation and rounding adding up would be N x 10 LSB30 = 3.7e-6 for X and
     *  N x 5 x 2^-29 = 3.7e-6 for F; tools/crosscheck_controller.py section 7b bounds the flight q against the float64
     *  reference at MAHONY_Q_TOL 1e-4 on every 6-DOF replay.) */
    {
        double qd[4], ex = 0.0, ef = 0.0;
        for (i = 0; i < 4; i++)
            qd[i] = (double)q_pad[i];
        for (k = 0; k < 400; k++) {
            double rv[3], an, sh, dq[4], r[4], nr;
            g[0] = 0.02f; g[1] = -0.01f; g[2] = 0.05f;
            a[0] = 0.0f; a[1] = 0.0f; a[2] = 1.3f * g0;
            att_mahony_update(&m, g, a, false);
            att_est_update(&e, g, a, false);
            for (i = 0; i < 3; i++)
                rv[i] = (double)(float)(g[i] - e.bias[i]) * (double)e.dt;     /* the float32 rate both form, exact x dt */
            an = sqrt(rv[0] * rv[0] + rv[1] * rv[1] + rv[2] * rv[2]);
            sh = an > 0.0 ? sin(0.5 * an) / an : 0.5;
            dq[0] = cos(0.5 * an); dq[1] = sh * rv[0]; dq[2] = sh * rv[1]; dq[3] = sh * rv[2];
            r[0] = qd[0] * dq[0] - qd[1] * dq[1] - qd[2] * dq[2] - qd[3] * dq[3];
            r[1] = qd[0] * dq[1] + qd[1] * dq[0] + qd[2] * dq[3] - qd[3] * dq[2];
            r[2] = qd[0] * dq[2] - qd[1] * dq[3] + qd[2] * dq[0] + qd[3] * dq[1];
            r[3] = qd[0] * dq[3] + qd[1] * dq[2] - qd[2] * dq[1] + qd[3] * dq[0];
            nr = sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2] + r[3] * r[3]);
            for (i = 0; i < 4; i++) {
                qd[i] = r[i] / nr;
                ex = fmax(ex, fabs((double)m.pad.q[i] - qd[i]));
                ef = fmax(ef, fabs((double)e.q[i] - qd[i]));
            }
        }
        CHECK(ex <= 1.1e-6 && ef <= 6e-7, "T1: fixed-point q within B_X 1.1e-6 (max %.3g) and float q within B_F 6e-7 "
              "(max %.3g) of the exact integration on every tick", ex, ef);
        printf("info: T1 gate-closed flight, 400 ticks: max |q_fixed - q_exact| %.3g (B_X 1.1e-6), max |q_float - q_exact| "
               "%.3g (B_F 6e-7), final |q_fixed - q_float| %.3g (bound 2e-6 = B_X + B_F rounded up)\n", ex, ef,
               fmax(fmax(fabs((double)m.pad.q[0] - e.q[0]), fabs((double)m.pad.q[1] - e.q[1])),
                    fmax(fabs((double)m.pad.q[2] - e.q[2]), fabs((double)m.pad.q[3] - e.q[3]))));
    }
    CHECK(m.flight_ticks == 400u && m.gated_ticks == 0u && !m.gate, "gate closed while | |a_lpf| - g | >= accel_gate g");
    /* fw09/p1: the VTOL flight integration runs in fixed point (core/qfix.h): q within float32 rounding of the float
     * gyro-only estimator (no longer bit for bit; bound: the T1 derivation above), the bias identical (not touched while
     * the gate is closed) */
    CHECK(fabsf(m.pad.q[0] - e.q[0]) < 2e-6f && fabsf(m.pad.q[1] - e.q[1]) < 2e-6f && fabsf(m.pad.q[2] - e.q[2]) < 2e-6f &&
          fabsf(m.pad.q[3] - e.q[3]) < 2e-6f && !memcmp(m.pad.bias, e.bias, sizeof e.bias),
          "gate closed: q within 2e-6 of the gyro-only estimator, bias identical (%.3g %.3g %.3g %.3g)",
          (double)(m.pad.q[0] - e.q[0]), (double)(m.pad.q[1] - e.q[1]), (double)(m.pad.q[2] - e.q[2]),
          (double)(m.pad.q[3] - e.q[3]));
    /* a tilt error with |f| = g: the correction pulls the estimate toward the measured up, the bias learns, bounded */
    att_mahony_init(&m, FC_DT_S, GAIN_EST_PAD_N_AVG, EST_V_KP, EST_V_KI_DT, EST_V_GATE_MPS2, EST_V_G, EST_V_ALPHA_A,
                    EST_V_BIAS_LIMIT_RAD_S);
    for (k = 0; k < 600; k++) {
        g[0] = g[1] = g[2] = 0.0f;
        a[0] = 0.0f; a[1] = 0.0f; a[2] = g0;
        att_mahony_update(&m, g, a, true);
    }
    {
        const double tilt0 = 5.0 * IL_PI_D / 180.0;     /* true attitude level; estimate starts tilted 5 deg about x */
        double err0, err1;
        m.pad.q[0] = (float)cos(tilt0 / 2.0); m.pad.q[1] = (float)sin(tilt0 / 2.0); m.pad.q[2] = m.pad.q[3] = 0.0f;
        err0 = acos(fmin(1.0, 1.0 - 2.0 * ((double)m.pad.q[1] * m.pad.q[1] + (double)m.pad.q[2] * m.pad.q[2])));
        for (k = 0; k < 2500; k++) {                     /* 10 s, still vehicle */
            g[0] = g[1] = g[2] = 0.0f;
            a[0] = 0.0f; a[1] = 0.0f; a[2] = g0;
            att_mahony_update(&m, g, a, false);
        }
        err1 = acos(fmin(1.0, 1.0 - 2.0 * ((double)m.pad.q[1] * m.pad.q[1] + (double)m.pad.q[2] * m.pad.q[2])));
        CHECK(m.gate && m.gated_ticks == 2500u && err1 < err0 * exp(-0.8 * EST_V_KP * 10.0),
              "5 deg estimate error decays with ~1/kp (%.2f -> %.3f deg in 10 s, kp %.3f)", err0 * 180.0 / IL_PI_D,
              err1 * 180.0 / IL_PI_D, (double)EST_V_KP);
        for (i = 0; i < 3; i++)
            CHECK(fabsf(m.pad.bias[i] - m.b_pad[i]) <= EST_V_BIAS_LIMIT_RAD_S, "bias change bounded (axis %d)", i);
    }
    {   /* bias learning: a constant gyro offset with the accelerometer showing level - b converges toward it, bounded */
        att_mahony_init(&m, FC_DT_S, GAIN_EST_PAD_N_AVG, 1.0f, 0.5f * FC_DT_S, EST_V_GATE_MPS2, EST_V_G, EST_V_ALPHA_A,
                        0.02f);
        for (k = 0; k < 600; k++) {
            g[0] = g[1] = g[2] = 0.0f;
            a[0] = 0.0f; a[1] = 0.0f; a[2] = g0;
            att_mahony_update(&m, g, a, true);
        }
        for (k = 0; k < 25000; k++) {
            g[0] = 0.05f; g[1] = 0.0f; g[2] = 0.0f;     /* 0.05 rad/s offset: beyond the 0.02 limit */
            att_mahony_update(&m, g, a, false);
        }
        CHECK(fabsf(m.pad.bias[0] - m.b_pad[0] - 0.02f) < 1e-6f, "bias learning saturates at b_pad + bias_limit (%.5f)",
              (double)(m.pad.bias[0] - m.b_pad[0]));
        /* re-entry to the pad levels from the landing spot (a_lpf), not from the old pad mean */
        a[0] = 0.0f; a[1] = 2.0f; a[2] = sqrtf(g0 * g0 - 4.0f);
        for (k = 0; k < 400; k++) att_mahony_update(&m, g, a, false);
        {
            float f0[3];
            memcpy(f0, m.a_lpf, sizeof f0);
            att_mahony_update(&m, g, a, true);
            CHECK(m.pad.n == GAIN_EST_PAD_N_AVG && NEAR(m.pad.f_mean[1], f0[1] + (a[1] - f0[1]) / (float)GAIN_EST_PAD_N_AVG,
                                                         1e-5), "pad re-entry: f_hat := a_lpf, then the EMA");
        }
    }
}

static void test_alt_vtol(void)
{
    alt_vtol_t f;
    int k;
    double k1, k2, k3, w, pp;
    const float lat = 0.1f;
    fc_default_alt_vtol(&f, lat);
    w = 2.0 * IL_PI_D * ALT_CROSSOVER_HZ;
    pp = 1.0 / ALTV_BIAS_TAU_S;
    k1 = 2.0 * ALT_ZETA * w + pp;
    k2 = w * w + 2.0 * ALT_ZETA * w * pp;
    k3 = w * w * pp;
    CHECK(NEAR(f.p.k1, k1, 2e-6 * k1) && NEAR(f.p.k2, k2, 2e-6 * k2) && NEAR(f.p.k3, k3, 2e-6 * k3) &&
          f.p.pad_n_avg == ALTV_PAD_N_AVG && f.p.latency_s == lat, "gains: (s^2 + 2 zeta w s + w^2)(s + 1 / bias_tau)");
    /* pad: h = v = 0; b = running mean of the first N ticks, then EMA 1/N; a baro sample only stamps its time */
    {
        double mean = 0.0;
        for (k = 1; k <= (int)ALTV_PAD_N_AVG; k++) {
            const float a = 0.05f + 0.01f * (float)(k % 7);
            mean += (a - mean) / k;
            alt_vtol_step(&f, a, k % 5 == 0, 3.0f, (uint32_t)k, true, 0.0f);
        }
        CHECK(f.h == 0.0f && f.v == 0.0f && NEAR(f.b, mean, 2e-6) && f.n_pad == ALTV_PAD_N_AVG && f.has_last_baro &&
              f.t_last_baro == ALTV_PAD_N_AVG, "pad: running mean %.5f (C %.5f), h = v = 0, baro stamped only", mean,
              (double)f.b);
        alt_vtol_step(&f, 1.0f, false, 0.0f, ALTV_PAD_N_AVG + 1u, true, 0.0f);
        CHECK(NEAR(f.b, mean + (1.0 - mean) / ALTV_PAD_N_AVG, 2e-6), "then an EMA with 1/N");
        for (k = 0; k < 100000; k++) alt_vtol_step(&f, 5.0f, false, 0.0f, 0u, true, 0.0f);
        CHECK(f.b == ALTV_BIAS_LIMIT_MPS2, "pad bias clamped to the limit");
    }
    /* flight: predict with (a - b); the bias state learns a constant accelerometer offset from the baro */
    fc_default_alt_vtol(&f, 0.0f);
    {
        const float bias = 0.098f;                       /* 10 mg */
        double h_true = 0.0;
        for (k = 1; k <= 500; k++) alt_vtol_step(&f, bias, false, 0.0f, (uint32_t)k, true, 0.0f);
        CHECK(NEAR(f.b, bias, 1e-6), "pad calibration learns the still vehicle's offset");
        f.b = 0.0f;                                     /* forget it: the flight learning alone */
        for (k = 501; k <= 501 + 250 * 200; k++)       /* 200 s hover at 2 m, baro every 5 ticks */
            alt_vtol_step(&f, bias, k % 5 == 0, (float)(h_true + 2.0), (uint32_t)k, false, 0.9f);
        CHECK(NEAR(f.b, bias, 0.002) && NEAR(f.h, 2.0, 0.01) && NEAR(f.v, 0.0, 0.005),
              "flight: bias %.4f learnt (true %.4f), h %.3f, v %.4f", (double)f.b, (double)bias, (double)f.h, (double)f.v);
        {   /* baro lost: predict only with the learnt bias - 10 s of dead reckoning */
            const float h0 = f.h;
            for (k = 0; k < 2500; k++) alt_vtol_step(&f, bias, false, 0.0f, 0u, false, 0.9f);
            CHECK(fabsf(f.h - h0) < 0.25f, "baro lost: 10 s dead reckoning on (a - b) drifts %.3f m (without the bias state "
                  "0.5 b t^2 = %.2f m)", (double)(f.h - h0), 0.5 * bias * 100.0);
        }
    }
    /* thrust compensation: h_c = h_b - comp (u / u_hover)^n; comp 0 -> nothing at all */
    {
        alt_vtol_params_t q;
        fc_default_alt_vtol(&f, 0.0f);
        q = f.p;
        CHECK(alt_vtol_thrust_ratio(&q, 0.0f) == 0.0f && alt_vtol_thrust_ratio(&q, NAN) == 0.0f &&
              alt_vtol_thrust_ratio(&q, -1.0f) == 0.0f && NEAR(alt_vtol_thrust_ratio(&q, VC_U_HOVER), 1.0, 2e-6) &&
              alt_vtol_thrust_ratio(&q, 2.0f) == alt_vtol_thrust_ratio(&q, 1.0f), "thrust ratio (u / u_hover)^2, u clamped");
        q.comp_m = 0.8f;
        alt_vtol_init(&f, &q);
        alt_vtol_step(&f, 0.0f, true, 1.0f, 10u, false, VC_U_HOVER);
        alt_vtol_step(&f, 0.0f, true, 1.0f, 15u, false, VC_U_HOVER);
        CHECK(NEAR(f.last_comp_m, 0.8, 2e-6), "compensation at hover thrust = comp_m (%.4f)", (double)f.last_comp_m);
        {
            alt_vtol_t g2;
            q.comp_m = 0.0f;
            alt_vtol_init(&g2, &q);
            alt_vtol_init(&f, &q);
            alt_vtol_step(&f, 0.1f, true, 1.0f, 10u, false, 0.9f);
            alt_vtol_step(&g2, 0.1f, true, 1.0f, 10u, false, 0.0f);
            alt_vtol_step(&f, 0.1f, true, 1.3f, 15u, false, 0.9f);
            alt_vtol_step(&g2, 0.1f, true, 1.3f, 15u, false, 0.0f);
            CHECK(f.h == g2.h && f.v == g2.v && f.b == g2.b && f.last_comp_m == 0.0f, "comp 0: u_prev changes nothing");
        }
    }
    /* the nlink throttle byte the NAV reads */
    CHECK(nl_fcu_throttle_q(0.0f) == 0 && nl_fcu_throttle_q(1.0f) == 250 && nl_fcu_throttle_q(0.002f) == 1 &&
          nl_fcu_throttle_q(0.0019999f) == 0 && nl_fcu_throttle_q(1.5f) == 250 && nl_fcu_throttle_q(-0.3f) == 0 &&
          nl_fcu_throttle_q(NAN) == 0 && nl_fcu_throttle(250) == 1.0f && nl_fcu_throttle(255) == 1.0f &&
          nl_fcu_throttle(125) == 0.5f, "thr_q = round_half_away(u x 250) in 0..250; u = min(thr_q, 250) / 250");
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
    /* v0.7 VTOL fields */
    U32(spool_ticks) U32(link_land_ticks) U32(endurance_ticks) U32(hard_cutoff_ticks) U32(v_fall_confirm_ticks)
    U32(safe_after_landed_ticks)
    I16(hover_dm) I16(alt_step_dm) I16(alt_min_dm) I16(alt_max_dm)
    F32(ceiling_m) F32(v_geofence_m) F32(v_fall_vz_mps)
    /* v0.7.1 */
    U32(land_timeout_ticks) U32(baro_limit_base_ticks) U32(baro_limit_mt_per_dm) I16(v_geofence_dm)
#undef U32
#undef F32
#undef I16
    if (!strcmp(key, "climb_milli")) { p->climb_milli = (uint16_t)x; return 1; }
    if (!strcmp(key, "gyro_saturation_abort")) { p->gyro_saturation_abort = x != 0.0; return 1; }
    if (!strcmp(key, "mode")) { p->mode = (uint8_t)x; return 1; }
    if (!strcmp(key, "gps_hold")) { p->gps_hold = x != 0.0; return 1; }           /* v0.8 */
    if (!strcmp(key, "takeoff_check")) { p->takeoff_check = x != 0.0; return 1; }
#if PM_ANY
    if (!strncmp(key, "pw.", 3)) {                     /* v0.9 power rules (pm_params_t; integers) */
        const char *f = key + 3;
#define PW16(name) if (!strcmp(f, #name)) { p->pw.name = (uint16_t)x; return 1; }
#define PW8(name) if (!strcmp(f, #name)) { p->pw.name = (uint8_t)x; return 1; }
        PW8(build) PW8(shift) PW16(arm_lo_q) PW16(arm_hi_q) PW16(buf_arm_lo_q) PW16(buf_arm_hi_q) PW16(lost_q)
        PW16(lost_margin_q) PW16(sag_q) PW16(sag_clear_q) PW16(buf_low_q) PW16(buf_res_q) PW16(d2_q) PW16(i8buf_q)
        PW16(sag_warn_ticks) PW16(sag_land_ticks) PW16(buf_res_ticks) PW8(lost_ticks) PW8(buf_ticks) PW8(d2_ticks)
        PW8(sag_release_ticks)
#undef PW16
#undef PW8
    }
#endif
    return 0;
}

/* "pp.<field>=value": a position-hold parameter (v0.8; floats as exact float32 text; integers) */
static int pos_override(pos_params_t *p, const char *kv)
{
    char key[64];
    const char *eq = strchr(kv, '=');
    double x;
    size_t n;
    if (!eq || strncmp(kv, "pp.", 3) != 0) return 0;
    kv += 3;
    n = (size_t)(eq - kv);
    if (n >= sizeof key) return 0;
    memcpy(key, kv, n);
    key[n] = '\0';
    x = strtod(eq + 1, 0);
#define F32(name) if (!strcmp(key, #name)) { p->name = (float)x; return 1; }
#define U16(name) if (!strcmp(key, #name)) { p->name = (uint16_t)x; return 1; }
#define U8(name) if (!strcmp(key, #name)) { p->name = (uint8_t)x; return 1; }
    U8(step_ticks) F32(T) F32(g) F32(tw) F32(kp_pos) F32(kv) F32(ki) F32(kd) F32(alpha_d) F32(i_max) F32(v_max)
    F32(v_max_land) F32(flare_speed) F32(flare_altitude_m) F32(cos_cap) F32(cos_flare_cap) F32(cos_excess) F32(sin_excess)
    F32(margin_g) F32(margin_frac) F32(h_enable_m) F32(fence_m) F32(spin_z_rad) F32(spin_xy_rad2) U16(guard_steps)
    U16(timeout_steps) U16(land_steps) U16(plaus_limited_steps) U16(plaus_vexc_steps) U16(plaus_reset_window_steps)
    U8(plaus_resets) F32(plaus_vt) F32(authority_min_ktw) F32(tilt_fade_m) F32(slew_per_tick) U8(keep_trim)
#undef F32
#undef U16
#undef U8
    return 0;
}

/* "vc.<field>=value": a vertical-controller parameter (floats as written, exact float32 text; integers) */
static int vc_override(vc_params_t *p, const char *kv)
{
    char key[64];
    const char *eq = strchr(kv, '=');
    double x;
    size_t n;
    if (!eq || strncmp(kv, "vc.", 3) != 0) return 0;
    kv += 3;
    n = (size_t)(eq - kv);
    if (n >= sizeof key) return 0;
    memcpy(key, kv, n);
    key[n] = '\0';
    x = strtod(eq + 1, 0);
#define F32(name) if (!strcmp(key, #name)) { p->name = (float)x; return 1; }
#define I32(name) if (!strcmp(key, #name)) { p->name = (int32_t)x; return 1; }
#define U16(name) if (!strcmp(key, #name)) { p->name = (uint16_t)x; return 1; }
#if FC_V09
    U16(ramp_total_ticks) I32(approach_min_um) F32(to_u_start) F32(to_s1) F32(to_s2) F32(to_search_f) F32(to_search_vz)
    F32(kt_alpha_to) F32(to_airborne_mps2) F32(td_step) F32(td_max) F32(td_ok)
    if (!strcmp(key, "v9")) { p->v9 = (uint8_t)x; return 1; }
    if (!strcmp(key, "approach_shift")) { p->approach_shift = (uint8_t)x; return 1; }
#endif
    I32(up_um) I32(dn_um) I32(td_um) I32(flare_um) F32(takeoff_climb_m) U16(takeoff_ticks) U16(takeoff_settle_ticks)
    F32(um_tick_to_mps) F32(flare_altitude_m) F32(alt_kp) F32(vz_kp) F32(vz_ki_dt) F32(vz_i_max) F32(vz_max_up_mps)
    F32(vz_max_down_mps) F32(u_min) F32(u_max) F32(u_hover) F32(inv_u_hover) F32(throttle_exp)
    F32(cos_tilt_min) F32(td_vz_still_mps) F32(lead_mps) F32(support_mps2) F32(g) F32(thrust_alpha)
    F32(still_alpha) F32(support_alpha) F32(kt_alpha) F32(kt_min) F32(kt_max) F32(kt_cos_min)
    U16(ramp_ticks) U16(landed_confirm_ticks) U16(floor_ticks)
#undef F32
#undef I32
#undef U16
    if (!strcmp(key, "throttle_exp_int")) { p->throttle_exp_int = (int8_t)x; return 1; }
    if (!strcmp(key, "vz_ff")) { p->vz_ff = x != 0.0; return 1; }
    return 0;
}

/* a float as text Python's float() reads back exactly (9 significant digits; nan / inf spelled out) */
static void put_f(FILE *fo, float x)
{
    if (isnan(x))
        fputs("nan", fo);
    else if (isinf(x))
        fputs(x > 0.0f ? "inf" : "-inf", fo);
    else
        fprintf(fo, "%.9g", (double)x);
}

/* the controller's outputs and the state the reference reports as outputs (vz_lpf, f_excess, thrust_ratio, ref_um):
 * mode,u,h_ref,vz_cmd,vz_err,u_ff,integ,sat,at_target,touchdown,td_count,sig,td_by,low_in_land,dh_ref,cos_tilt,ref_um,
 * v_lpf,d_lpf,r_thr, then the counters floor_run,cnt0,cnt1,cnt2 and (v0.7.1b) cnt3,k_t */
static void put_vc_out(FILE *fo, const vc_outputs_t *v, const vc_t *c)
{
    fprintf(fo, "%u,", v->mode);
    put_f(fo, v->u); fputc(',', fo);
    put_f(fo, v->h_ref); fputc(',', fo);
    put_f(fo, v->vz_cmd); fputc(',', fo);
    put_f(fo, v->vz_err); fputc(',', fo);
    put_f(fo, v->u_ff); fputc(',', fo);
    put_f(fo, v->integ);
    fprintf(fo, ",%d,%d,%d,%u,%u,%u,%u,", v->sat, v->at_target ? 1 : 0, v->touchdown ? 1 : 0, v->td_count, v->sig, v->td_by,
            v->low_in_land_ticks);
    put_f(fo, v->dh_ref_mps); fputc(',', fo);
    put_f(fo, v->cos_tilt);
    fprintf(fo, ",%ld,", (long)c->ref_um);
    put_f(fo, c->v_lpf); fputc(',', fo);
    put_f(fo, c->d_lpf); fputc(',', fo);
    put_f(fo, c->r_thr);
    fprintf(fo, ",%u,%u,%u,%u,%u,", c->floor_run, c->cnt[0], c->cnt[1], c->cnt[2], c->cnt[3]);
    put_f(fo, c->k_t);
}

#define VC_OUT_HEAD "mode,u,h_ref,vz_cmd,vz_err,u_ff,integ,sat,at_target,touchdown,td_count,sig,td_by,low_in_land,dh_ref,cos_tilt," \
                    "ref_um,v_lpf,d_lpf,r_thr,floor_run,cnt0,cnt1,cnt2,cnt3,k_t"

/* supervisor vectors (stack = 0) and FCU-stack vectors (stack = 1):
 *   #S id seed reset_in_flight [key=value ...]   new supervisor (default: the BALLISTIC build; "mode=1" = VTOL)
 *   #K id seed reset_in_flight [key=value ...]   new stack (the same keys + "vc.<field>=value")
 *   row: tick, n_frames, 4 x (seq cmd arg nonce), h vz tilt gx gy gz gsat vcell imu baro aligned arm roll pitch yaw loopmax
 *        [vc_touchdown vc_at_target throttle_prev [vc_ground_signature vc_low_in_land_ticks q0 q1 q2 q3 accel_z [k_sync
 *        [reserved cos_tilt gps_gate_ok pc_requests vc_takeoff_fail tlm_flags tlm_sv tlm_hacc tlm_mode tlm_dist slot0..slot4]]]]
 *        (v0.8: cos_tilt NaN = vc_cos_tilt_from_q(q), else it wins; slot0 < 0 = no slot this tick; a stack row ignores the
 *        gps_gate_ok .. tlm_dist columns - the stack fills them from its slot receiver and position controller)
 *        (v0.7 VTOL supervisor rows; v0.7.1 adds the seven after throttle_prev; a stack row ignores the vc_* /
 *        throttle_prev columns - the stack feeds the previous tick's own flags - and uses q / accel_z for the vertical
 *        controller; v0.7.1b stack rows: k_sync, the reference's k_T before the tick, see run_vc_vectors)
 *   out: seq,tick,state,target,out,active,release,reason,flags,ack,nonce,reply_seq,interlock,telem (v1 28 / v2 31 bytes),
 *        exec (seq:cmd:code:sb:sa:pb:pa|...),trans (from:to:cause:pf:pt|...),phase,h_tgt_dm,motor_on,buzzer,baro_lost
 *        [stack: ,VC_OUT_HEAD,esc,att_active,att_u - att_* = the attitude controller's inputs of this tick, read before
 *        onb_stack_step (VTOL: the previous tick's; ballistic: read after it, this tick's); v0.8 (gps_hold): ,to_run,to_passed,
 *        att_tilt0,att_tilt1 (the attitude controller's tilt of this tick), then G, the published position outputs (mode,
 *        requests, engaged, tilt_l0, tilt_l1, dist), the builder's tilt for the next tick (2), gate_ok, fresh, the job in
 *        progress, and the position state (n, engaged, has_hold, hold_rf0, hold_rf1, integ0, integ1, dv0, dv1, lost,
 *        guard_left, implausible, requests, lim_run, vexc_run), pend_valid, overruns] */
#if PM_ANY
static pm_inputs_t g_pw_in = {{PM_CODE_NONE, PM_CODE_NONE, PM_CODE_NONE}, false};   /* v0.9 'P' lines */
static bool g_pw_tp = true, g_pw_sup, g_pw_air;

/* v0.9 power-monitor vectors:
 *   #P id [pw.<field>=value ...]   a fresh monitor (fc_default_onboard's pw + overrides; pw.build picks the build)
 *   row: powered throttle_prev c_in c_tet c_buf buf_i_pos   (codes < 0: not measured)
 *   out: id,k,note,y_in,y_tet,y_buf,fl,sag_cnt,rel_cnt,u_cap,lost_cnt,buf_cnt,res_cnt,d2_cnt,sag_land,lost,buf,d2,i8buf,arm,
 *        flags,vaux */
static int run_pm_vectors(const char *in, const char *out)
{
    FILE *fi = fopen(in, "r"), *fo = fopen(out, "w");
    onb_params_t p;
    pm_t m;
    double v[8];
    long id = -1, k = 0;
    if (!fi || !fo) return 2;
    fc_default_onboard(&p);
    pm_init(&m);
    fprintf(fo, "id,k,note,y_in,y_tet,y_buf,fl,sag_cnt,rel_cnt,u_cap,lost_cnt,buf_cnt,res_cnt,d2_cnt,sag_land,lost,buf,d2,"
                "i8buf,arm,flags,vaux\n");
    while (fgets(g_line, sizeof g_line, fi)) {
        if (g_line[0] == '#' && g_line[1] == 'P') {
            char *tok;
            fc_default_onboard(&p);
            tok = strtok(g_line + 2, " \t\r\n");
            id = tok ? strtol(tok, 0, 10) : 0;
            while ((tok = strtok(0, " \t\r\n")) != 0) {
                if (!onb_override(&p, tok)) {
                    fprintf(stderr, "pm-vectors: unknown override %s\n", tok);
                    return 3;
                }
            }
            pm_init(&m);
            k = 0;
            continue;
        }
        if (g_line[0] == '#' || g_line[0] == '\n' || g_line[0] == '\r') continue;
        {
            pm_inputs_t pi;
            uint8_t note;
            int q;
            const int nv = parse_numbers(g_line, v, 8);
            if (nv < 6) {
                fprintf(stderr, "pm-vectors: short row\n");
                return 3;
            }
            for (q = 0; q < 3; q++)
                pi.code[q] = v[2 + q] >= 0.0 ? (uint16_t)v[2 + q] : PM_CODE_NONE;
            pi.buf_i_pos = v[5] != 0.0;
            note = pm_sample(&m, &p.pw, &pi, v[0] != 0.0, (float)v[1]);
            fprintf(fo, "%ld,%ld,%u,%d,%d,%d,%u,%u,%u,", id, k, note, m.fl & PM_F_HAVE_IN ? m.y_in : -1,
                    m.fl & PM_F_HAVE_TET ? m.y_tet : -1, m.fl & PM_F_HAVE_BUF ? m.y_buf : -1, m.fl, m.sag_cnt, m.rel_cnt);
            put_f(fo, (m.fl & PM_F_SAG_WARN) ? m.u_cap : NAN);
            fprintf(fo, ",%u,%u,%u,%u,%d,%d,%d,%d,%d,%d,%u,%u\n", m.lost_cnt, m.buf_cnt, m.res_cnt, m.d2_cnt,
                    pm_sag_land(&m, &p.pw), pm_lost_cond(&m, &p.pw), pm_buf_cond(&m, &p.pw), pm_d2_cond(&m, &p.pw),
                    pm_i8_buf(&m, &p.pw), pm_arm_ok(&m, &p.pw), pm_flags(&m, &p.pw), pm_vaux_q(&m, &p.pw));
            k++;
        }
    }
    fclose(fi);
    fclose(fo);
    return 0;
}
#endif

/* v0.9 TELEM v4 / v5 prototype (core/link_proto_v09.h): rows "kind veh seq vaux flags payloadhex" (kind 4: a TELEM2
 * payload, 5: TELEM3), out: the frame hex (the payload unpacked and packed again by the C) */
static int run_telem4_vectors(const char *in, const char *out)
{
    FILE *fi = fopen(in, "r"), *fo = fopen(out, "w");
    if (!fi || !fo) return 2;
    while (fgets(g_line, sizeof g_line, fi)) {
        unsigned kind, veh, seq, va, fl;
        char hex[128];
        uint8_t pl[64], fr[64], n, i;
        lp_telem_t t;
        if (sscanf(g_line, "%u %u %u %u %u %127s", &kind, &veh, &seq, &va, &fl, hex) != 6) continue;
        for (i = 0; hex[2 * i] && hex[2 * i + 1] && i < sizeof pl; i++) {
            unsigned b;
            sscanf(hex + 2 * i, "%2x", &b);
            pl[i] = (uint8_t)b;
        }
        if (kind == 4u) {
            lp_unpack_telem2(pl, &t);
            n = lp_encode_telem4((uint8_t)veh, (uint8_t)seq, &t, (uint8_t)va, (uint8_t)fl, fr);
        } else if (kind == 6u) {                       /* 2026-11-10: TELEM v6 / v7, va = lbat (fl ignored) */
            lp_unpack_telem2(pl, &t);
            n = lp_encode_telem6((uint8_t)veh, (uint8_t)seq, &t, (uint8_t)va, fr);
        } else if (kind == 7u) {
            lp_unpack_telem3(pl, &t);
            n = lp_encode_telem7((uint8_t)veh, (uint8_t)seq, &t, (uint8_t)va, fr);
        } else {
            lp_unpack_telem3(pl, &t);
            n = lp_encode_telem5((uint8_t)veh, (uint8_t)seq, &t, (uint8_t)va, (uint8_t)fl, fr);
        }
        for (i = 0; i < n; i++) fprintf(fo, "%02x", fr[i]);
        fputc('\n', fo);
    }
    fclose(fi);
    fclose(fo);
    return 0;
}

static int run_onb_vectors_mode(const char *in, const char *out, int stack)
{
    FILE *fi = fopen(in, "r"), *fo = fopen(out, "w");
    static onb_stack_t k;
    static onb_outputs_t o;
    onboard_t *s = &k.sup;
    onb_frame_t fr[ONB_MAX_FRAMES];
    pos_params_t pp;
    double v[64];
    long id = -1, rows = 0;
    if (!fi || !fo) return 2;
    fprintf(fo, "seq,tick,state,target,out,active,release,reason,flags,ack,nonce,reply_seq,interlock,telem,exec,trans,phase,"
                "h_tgt_dm,motor_on,buzzer,baro_lost%s\n",
            stack ? ",vc_" VC_OUT_HEAD ",esc,att_active,att_u[,to_run,to_passed,att_tilt0,att_tilt1,G,...]" : "");
    while (fgets(g_line, sizeof g_line, fi)) {
        if (g_line[0] == '#' && (g_line[1] == 'S' || g_line[1] == 'K')) {   /* #S / #K id seed reset [key=value ...] */
            onb_params_t p;
            vc_params_t vp;
            long seed = 0, reset = 0;
            uint8_t chain = ONB_CHAIN_FCU;                  /* v0.8 rows: the three-chip FCU; 'chain=1': v0.8.1 GNS */
            char *tok;
            fc_default_onboard(&p);
            p.mode = LP_MODE_BALLISTIC;                     /* v0.6.3 rows name no mode */
            fc_default_vertical(&vp);
            fc_default_position(&pp);
            tok = strtok(g_line + 2, " \t\r\n");
            id = tok ? strtol(tok, 0, 10) : 0;
            tok = strtok(0, " \t\r\n");
            seed = tok ? strtol(tok, 0, 10) : 0;
            tok = strtok(0, " \t\r\n");
            reset = tok ? strtol(tok, 0, 10) : 0;
            while ((tok = strtok(0, " \t\r\n")) != 0) {
                if (strncmp(tok, "chain=", 6) == 0) {
                    chain = (uint8_t)strtol(tok + 6, 0, 10);
                    continue;
                }
                if (!onb_override(&p, tok) && !(stack && (vc_override(&vp, tok) || pos_override(&pp, tok)))) {
                    fprintf(stderr, "onb-vectors: unknown override %s\n", tok);
                    return 3;
                }
            }
            onb_stack_init(&k, &p, &vp, &pp, chain, (uint16_t)seed, reset != 0);
            continue;
        }
#if PM_ANY
        if (g_line[0] == 'P') {                        /* v0.9: "P c_in c_tet c_buf buf_i_pos takeoff_passed support airborne" */
            double pv[8];                              /* for the next row (codes < 0: none) */
            const int np = parse_numbers(g_line + 1, pv, 8);
            int q;
            for (q = 0; q < 3; q++)
                g_pw_in.code[q] = (np > q && pv[q] >= 0.0) ? (uint16_t)pv[q] : PM_CODE_NONE;
            g_pw_in.buf_i_pos = np > 3 && pv[3] != 0.0;
            g_pw_tp = np > 4 && pv[4] != 0.0;
            g_pw_sup = np > 5 && pv[5] != 0.0;
            g_pw_air = np > 6 && pv[6] != 0.0;
            continue;
        }
#endif
        if (g_line[0] == '#' || g_line[0] == 's' || g_line[0] == '\n' || g_line[0] == '\r') continue;
        {
            const int nv = parse_numbers(g_line, v, 64);
            onb_inputs_t inp;
            int nf = (int)v[1], i, b = 18;
            bool att_active = false;
            float att_u = 0.0f, att_tilt[2] = {0.0f, 0.0f}, qn[4];
            uint8_t slot[NL_SLOT_LEN];
            if (nv < 34) {
                fprintf(stderr, "onb-vectors: short row %ld\n", rows);
                return 3;
            }
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
            inp.vc_touchdown = nv >= 37 && v[34] != 0.0;
            inp.vc_at_target = nv >= 37 && v[35] != 0.0;
            inp.throttle_prev = nv >= 37 ? (float)v[36] : 0.0f;
            inp.vc_ground_signature = nv >= 44 && v[37] != 0.0;
            inp.vc_low_in_land_ticks = nv >= 44 ? (uint16_t)v[38] : 0u;
            qn[0] = nv >= 44 ? (float)v[39] : 1.0f;
            qn[1] = nv >= 44 ? (float)v[40] : 0.0f;
            qn[2] = nv >= 44 ? (float)v[41] : 0.0f;
            qn[3] = nv >= 44 ? (float)v[42] : 0.0f;
            /* v0.8: cos(tilt) of the frame (nlink v3) or, for a q row, the v0.7.1 arithmetic on q (vc_cos_tilt_from_q) */
            inp.cos_tilt = (nv >= 47 && !isnan(v[46])) ? (float)v[46] : vc_cos_tilt_from_q(qn);
            inp.accel_z_mps2 = nv >= 44 ? (float)v[43] : NAN;
            inp.gps_gate_ok = nv >= 56 && v[47] != 0.0;
            inp.pc_requests = nv >= 56 ? (uint8_t)v[48] : 0u;
            inp.vc_takeoff_fail = nv >= 56 && v[49] != 0.0;
            inp.tlm_gps.flags = nv >= 56 ? (uint8_t)v[50] : 0u;
            inp.tlm_gps.num_sv = nv >= 56 ? (uint8_t)v[51] : 0u;
            inp.tlm_gps.hacc_dm = nv >= 56 ? (uint8_t)v[52] : (uint8_t)LP_HACC_NONE;
            inp.tlm_gps.pos_mode = nv >= 56 ? (uint8_t)v[53] : 0u;
            inp.tlm_gps.dist_m = nv >= 56 ? (float)v[54] : NAN;
#if PM_ANY
            inp.pw = g_pw_in;                          /* v0.9 (the latest 'P' line; none: no codes) */
            inp.vc_takeoff_passed = g_pw_tp;
            inp.vc_support = g_pw_sup;
            inp.vc_airborne = g_pw_air;
#endif
            inp.gps_slot = 0;
            if (nv >= 60 && v[55] >= 0.0) {
                for (i = 0; i < (int)NL_SLOT_LEN; i++)
                    slot[i] = (uint8_t)v[55 + i];
                inp.gps_slot = slot;
            }
            if (stack) {
                att_active = onb_stack_att_active(&k);
                att_u = onb_stack_att_u(&k);
#if ONB_HAS_GPS
                att_tilt[0] = k.tilt[0];
                att_tilt[1] = k.tilt[1];
#endif
                if (nv >= 45 && isfinite(v[44]))
                    k.vc.k_t = (float)v[44];               /* v0.7.1b k_sync (see run_vc_vectors) */
                (void)onb_stack_step(&k, &inp, &o);
                if (!onboard_vtol(&k.sup)) {               /* ballistic: the attitude controller runs after the supervisor */
                    att_active = onb_stack_att_active(&k);
                    att_u = onb_stack_att_u(&k);
                }
            } else {
                onboard_update(s, &inp, &o);
            }
            fprintf(fo, "%ld,%lu,%u,%u,%u,%d,%d,%u,%u,%u,%u,%d,%u,", id, (unsigned long)inp.tick, o.state, o.target_milli,
                    o.throttle_milli, o.control_active ? 1 : 0, o.release ? 1 : 0, o.reason, o.flags, o.ack_seq, o.nonce,
                    o.reply ? (int)o.reply_seq : -1, o.interlock);
            if (o.reply) {
                lp_telem_t t;
                uint8_t pl[LP_TELEM3_PAYLOAD_LEN], q, n;
                onboard_telemetry(&o.tlm, s->p.loop_hz, &t);
                if (o.tlm.v3) {
                    lp_pack_telem3(&t, pl);
                    n = LP_TELEM3_PAYLOAD_LEN;
                } else if (o.tlm.v2) {
                    lp_pack_telem2(&t, pl);
                    n = LP_TELEM2_PAYLOAD_LEN;
                } else {
                    lp_pack_telem(&t, pl);
                    n = LP_TELEM_PAYLOAD_LEN;
                }
                for (q = 0; q < n; q++) fprintf(fo, "%02x", pl[q]);
            } else {
                fputc('-', fo);
            }
            fputc(',', fo);
            if (!o.n_exec) fputc('-', fo);
            for (i = 0; i < o.n_exec; i++)
                fprintf(fo, "%s%u:%u:%u:%u:%u:%u:%u", i ? "|" : "", o.exec[i].seq, o.exec[i].cmd, o.exec[i].code,
                        o.exec[i].state_before, o.exec[i].state_after, o.exec[i].phase_before, o.exec[i].phase_after);
            fputc(',', fo);
            if (!o.n_trans) fputc('-', fo);
            for (i = 0; i < o.n_trans; i++)
                fprintf(fo, "%s%u:%u:%u:%u:%u", i ? "|" : "", o.trans[i].from, o.trans[i].to, o.trans[i].cause,
                        o.trans[i].phase_from, o.trans[i].phase_to);
            fprintf(fo, ",%u,%d,%d,%d,%d", o.phase, o.h_tgt_dm, o.motor_on ? 1 : 0, o.buzzer ? 1 : 0, o.baro_lost ? 1 : 0);
#if PM_ANY
            if (!stack && s->p.pw.build != PWR_SRC_BATTERY) {   /* v0.9: the monitor after this tick + the TELEM tail */
                fprintf(fo, ",W,%d,%d,%d,%u,%u,", s->pm.fl & PM_F_HAVE_IN ? s->pm.y_in : -1,
                        s->pm.fl & PM_F_HAVE_TET ? s->pm.y_tet : -1, s->pm.fl & PM_F_HAVE_BUF ? s->pm.y_buf : -1,
                        s->pm.fl, onboard_u_capped(s) ? 1u : 0u);
                put_f(fo, onboard_u_capped(s) ? s->pm.u_cap : NAN);
                fprintf(fo, ",%u,%u", o.reply ? o.tlm.pw_vaux : 0u, o.reply ? o.tlm.pw_flags : 0u);
            }
#endif
            if (stack) {
                fputc(',', fo);
                if (k.have_vo) {
                    put_vc_out(fo, &k.vo, &k.vc);
                } else {
                    vc_outputs_t z;
                    memset(&z, 0, sizeof z);
                    z.cos_tilt = 1.0f;
                    put_vc_out(fo, &z, &k.vc);
                }
                fputc(',', fo);
                put_f(fo, k.throttle);
                fprintf(fo, ",%d,", att_active ? 1 : 0);
                put_f(fo, att_u);
                if (onboard_vtol(&k.sup)) {            /* v0.8 take-off check state, the attitude tilt of this tick */
                    fprintf(fo, ",%u,%d,", k.vc.to_run, k.vc.to_passed ? 1 : 0);
                    put_f(fo, att_tilt[0]); fputc(',', fo);
                    put_f(fo, att_tilt[1]);
                }
#if ONB_HAS_GNS
                if (k.gps && k.chain == ONB_CHAIN_GNS) {        /* v0.8.1: the GNS slot receiver and the FCU4 fields */
                    const nl_gns_rx_t *r = &k.grx;
                    float tg[2];
                    uint8_t tf, ts, th, tm;
                    float td;
                    const bool eng = nl_gns_rx_target(r, tg);
                    nl_gns_rx_tlm(r, k.motor_on, &tf, &ts, &th, &tm, &td);
                    fprintf(fo, ",N,%d,%d,%d,%d,", nl_gns_rx_gate_ok(r) ? 1 : 0, nl_gns_rx_fresh(r) ? 1 : 0,
                            nl_gns_rx_fallback(r) ? 1 : 0, eng ? 1 : 0);
                    put_f(fo, tg[0]); fputc(',', fo);
                    put_f(fo, tg[1]); fputc(',', fo);
                    put_f(fo, k.tilt[0]); fputc(',', fo);
                    put_f(fo, k.tilt[1]);
                    fprintf(fo, ",%u,%u,%u,%u,%u,%u,", nl_gns_rx_requests(r, k.land_steps), k.pflags, k.kt_q, tf, ts, th);
                    fprintf(fo, "%u,", tm);
                    put_f(fo, td);
                    fprintf(fo, ",%u,%u", r->sets, r->broken);
                }
#endif
#if ONB_HAS_FCU_POS
                if (k.gps && k.chain == ONB_CHAIN_FCU) {
                    const pos_t *ps = &k.pos;
                    const pos_pub_t *pb = &k.pipe.pub;
                    fprintf(fo, ",G,%u,%u,%d,", pb->mode, pb->requests, pb->engaged ? 1 : 0);
                    put_f(fo, pb->tilt_l[0]); fputc(',', fo);
                    put_f(fo, pb->tilt_l[1]); fputc(',', fo);
                    put_f(fo, pb->dist_m); fputc(',', fo);
                    put_f(fo, k.tilt[0]); fputc(',', fo);
                    put_f(fo, k.tilt[1]);
                    fprintf(fo, ",%d,%d,%u,%lu,%d,%d,", nl_slot_rx_gate_ok(&k.rx) ? 1 : 0, nl_slot_rx_fresh(&k.rx) ? 1 : 0,
                            ps->job, (unsigned long)ps->n, ps->engaged ? 1 : 0, ps->has_hold ? 1 : 0);
                    put_f(fo, ps->hold_rf[0]); fputc(',', fo);
                    put_f(fo, ps->hold_rf[1]); fputc(',', fo);
                    put_f(fo, ps->integ[0]); fputc(',', fo);
                    put_f(fo, ps->integ[1]); fputc(',', fo);
                    put_f(fo, ps->dv[0]); fputc(',', fo);
                    put_f(fo, ps->dv[1]);
                    fprintf(fo, ",%u,%u,%d,%u,%u,%u,%d,%u", ps->lost, ps->guard_left, ps->implausible ? 1 : 0, ps->requests,
                            ps->lim_run, ps->vexc_run, k.pipe.pend_valid ? 1 : 0, k.pipe.overruns);
                }
#endif
            }
            fputc('\n', fo);
            rows++;
        }
    }
    fclose(fi);
    fclose(fo);
    fprintf(stderr, "%s-vectors: %ld rows\n", stack ? "stack" : "onb", rows);
    return 0;
}

static int run_onb_vectors(const char *in, const char *out) { return run_onb_vectors_mode(in, out, 0); }
static int run_stack_vectors(const char *in, const char *out) { return run_onb_vectors_mode(in, out, 1); }

/* vertical-controller vectors:
 *   #V id [vc.<field>=value ...]   a fresh controller (fc_default_vertical + overrides)
 *   row: phase, h_tgt_m, baro_lost, h_m, vz_mps, q0..q3, f_z [, k_sync]   (nan / inf allowed; the target goes in as the
 *        supervisor's dm = round_half_away(h_tgt_m x 10), cos(tilt) = vc_cos_tilt_from_q(q): the FCU path; v0.7.1b
 *        k_sync = the reference's k_T before this tick, taken as vc_t.k_t before the step - tools/crosscheck_controller.py
 *        _vc_row says why)
 *   out: id,k,VC_OUT_HEAD */
static int run_vc_vectors(const char *in, const char *out)
{
    FILE *fi = fopen(in, "r"), *fo = fopen(out, "w");
    static vc_t c;
    vc_params_t vp;
    vc_outputs_t o;
    double v[12];
    int nv;
    long id = -1, k = 0, rows = 0;
    if (!fi || !fo) return 2;
    fc_default_vertical(&vp);
    vc_init(&c, &vp);
    fprintf(fo, "id,k," VC_OUT_HEAD "\n");
    while (fgets(g_line, sizeof g_line, fi)) {
        if (g_line[0] == '#' && g_line[1] == 'V') {
            char *tok;
            fc_default_vertical(&vp);
            tok = strtok(g_line + 2, " \t\r\n");
            id = tok ? strtol(tok, 0, 10) : 0;
            while ((tok = strtok(0, " \t\r\n")) != 0) {
                if (!vc_override(&vp, tok)) {
                    fprintf(stderr, "vc-vectors: unknown override %s\n", tok);
                    return 3;
                }
            }
            vc_init(&c, &vp);
            k = 0;
            continue;
        }
        if (g_line[0] == '#' || g_line[0] == '\n' || g_line[0] == '\r') continue;
        nv = parse_numbers(g_line, v, 12);
        if (nv < 10) {
            fprintf(stderr, "vc-vectors: short row %ld\n", rows);
            return 3;
        }
        {
            vc_inputs_t vi;
            float q[4];
            if (nv >= 11 && isfinite(v[10]))
                c.k_t = (float)v[10];                  /* v0.7.1b k_sync: the reference's k_T before this tick */
            q[0] = (float)v[5];
            q[1] = (float)v[6];
            q[2] = (float)v[7];
            q[3] = (float)v[8];
            vi.phase = (uint8_t)v[0];
            vi.h_tgt_dm = (int16_t)lp_quantize((float)v[1], LP_H_SCALE, LP_I16_MIN, LP_I16_MAX);   /* = target_um's dm */
            vi.baro_lost = v[2] != 0.0;
            vi.h_m = (float)v[3];
            vi.vz_mps = (float)v[4];
            vi.cos_tilt = vc_cos_tilt_from_q(q);
            vi.f_z_mps2 = (float)v[9];
#if PM_WIRED
            vi.u_capped = nv >= 12 && isfinite(v[11]);    /* v0.9 W_SAG ceiling column (nan = None) */
            vi.u_cap = vi.u_capped ? (float)v[11] : 0.0f;
#endif
            vc_update(&c, &vi, &o);
        }
        fprintf(fo, "%ld,%ld,", id, k);
        put_vc_out(fo, &o, &c);
#if FC_V09                                             /* v0.9: the lift-off latch, the take-off check, fz_lpf / td_bias */
        fprintf(fo, ",V9,%u,%u,%u,", c.airborne ? 1u : 0u, c.to_passed ? 1u : 0u, c.ramp_k);
        put_f(fo, c.fz_lpf);
        fputc(',', fo);
        put_f(fo, c.td_bias);
#endif
        fputc('\n', fo);
        k++;
        rows++;
    }
    fclose(fi);
    fclose(fo);
    fprintf(stderr, "vc-vectors: %ld rows\n", rows);
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
        } else if (kind == 'U') {                               /* U veh seq 17 fields: TELEM v2 (VTOL) */
            lp_telem_t t;
            parse_numbers(g_line + 1, v, 19);
            t.state = (uint8_t)v[2]; t.flags = (uint8_t)v[3]; t.reason = (uint8_t)v[4]; t.ack_seq = (uint8_t)v[5];
            t.nonce = (uint16_t)v[6]; t.t_ms = (uint32_t)v[7]; t.h_dm = (int16_t)v[8]; t.vz_cms = (int16_t)v[9];
            t.roll_cdeg = (int16_t)v[10]; t.pitch_cdeg = (int16_t)v[11]; t.yaw_cdeg = (int16_t)v[12];
            t.tilt_cdeg = (uint16_t)v[13]; t.throttle_milli = (uint16_t)v[14]; t.vcell_mv = (uint16_t)v[15];
            t.loop_max_us = (uint16_t)v[16]; t.phase = (uint8_t)v[17]; t.h_tgt_dm = (int16_t)v[18];
            n = lp_encode_telem2((uint8_t)v[0], (uint8_t)v[1], &t, fr);
            fputc('U', fo);
            fputc(' ', fo);
            for (i = 0; i < n; i++) fprintf(fo, "%02x", fr[i]);
            fputc('\n', fo);
        } else if (kind == 'V') {                               /* V veh seq 22 fields: TELEM v3 (v0.8 VTOL + GPS) */
            lp_telem_t t;
            parse_numbers(g_line + 1, v, 24);
            t.state = (uint8_t)v[2]; t.flags = (uint8_t)v[3]; t.reason = (uint8_t)v[4]; t.ack_seq = (uint8_t)v[5];
            t.nonce = (uint16_t)v[6]; t.t_ms = (uint32_t)v[7]; t.h_dm = (int16_t)v[8]; t.vz_cms = (int16_t)v[9];
            t.roll_cdeg = (int16_t)v[10]; t.pitch_cdeg = (int16_t)v[11]; t.yaw_cdeg = (int16_t)v[12];
            t.tilt_cdeg = (uint16_t)v[13]; t.throttle_milli = (uint16_t)v[14]; t.vcell_mv = (uint16_t)v[15];
            t.loop_max_us = (uint16_t)v[16]; t.phase = (uint8_t)v[17]; t.h_tgt_dm = (int16_t)v[18];
            t.gps_flags = (uint8_t)v[19]; t.num_sv = (uint8_t)v[20]; t.hacc_dm = (uint8_t)v[21]; t.pos_mode = (uint8_t)v[22];
            t.dist_dm = (uint16_t)v[23];
            n = lp_encode_telem3((uint8_t)v[0], (uint8_t)v[1], &t, fr);
            fputc('V', fo);
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
        } else if (kind == 'P') {                      /* SNAP -> TELEM (ballistic COM) or SNAP2 -> TELEM2 (VTOL COM) */
            const long veh = strtol(p, &end, 10), hz = strtol(end, &end, 10);
            onb_tlm_src_t src;
            lp_telem_t t;
            uint8_t seq, lbat = LP_LBAT_NONE;
            n = parse_hex(end, buf, (int)sizeof buf);
            memset(&src, 0, sizeof src);
            if (n == (int)IL_SNAP3_LEN) {              /* v0.9 RP1: the VTOL + GPS COM (SNAP3 -> TELEM3 / TELEM5) */
                il_snap3_t s;
                il_unpack_snap3(buf, &s);
                il_snap3_to_tlm_src(&s, &src);
                seq = s.s.reply_seq;
                lbat = s.spare3;                       /* 2026-11-10: the logic battery (TELEM7) */
            } else if (n == (int)IL_SNAP2_LEN) {
                il_snap2_t s;
                il_unpack_snap2(buf, &s);
                il_snap2_to_tlm_src(&s, &src);
                seq = s.s.reply_seq;
                lbat = s.spare;                        /* 2026-11-10: the logic battery (TELEM6) */
            } else if (n == (int)IL_SNAP_LEN) {
                il_snap_t s;
                il_unpack_snap(buf, &s);
                il_snap_to_tlm_src(&s, &src);
                seq = s.reply_seq;
            } else {
                fprintf(stderr, "ilink-vectors: P line with %d snapshot bytes\n", n);
                return 3;
            }
            onboard_telemetry(&src, (uint16_t)hz, &t);
            if (src.v2)                                /* the COM's rule (com/com_link.c: lp_encode_vtol_reply) */
                put_hex_line(fo, 'P', fr, lp_encode_vtol_reply((uint8_t)veh, seq, &t, src.v3, src.pw_vaux, src.pw_flags,
                                                               lbat, fr));
            else
                put_hex_line(fo, 'P', fr, lp_encode_telem((uint8_t)veh, seq, &t, fr));
        } else if (kind == 'F') {                      /* 20 numbers: SNAP; 22 (+ phase, h_tgt_m bits): SNAP2 */
            double v[22];
            onb_tlm_src_t src;
            il_snap_t s;
            float q[4];
            const int nv = parse_numbers(p, v, 22);
            if (nv != 20 && nv != 22) {
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
            if (nv == 22) {
                il_snap2_t s2;
                src.v2 = true;
                src.phase = (uint8_t)v[20];
                src.h_tgt_m = il_from_bits((long long)v[21]);
                il_snap2_from_tlm_src((uint8_t)v[0], &src, q, &s2);
                il_pack_snap2(&s2, buf);
                put_hex_line(fo, 'F', buf, IL_SNAP2_LEN);
            } else {
                src.v2 = false;
                src.phase = 0;
                src.h_tgt_m = 0.0f;
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

static void nav3_from_fields(nl_nav3_t *n, uint8_t seq, const uint32_t fb[12], const long iv[9], const uint8_t slot[5]);

/* nlink vectors: N / F encode, D decode (v0.6.3 / v0.7.1); v0.8: M seq fb0..fb11 iv0..iv8 s0..s4 -> M <72 bytes>,
 * X <72 bytes> -> X <nav3 ok> [seq fb.. iv.. slothex], W slot.. (hex or '-' per tick) -> W per tick got:gate:fresh[:set]
 * ... S sets broken */
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
        } else if (kind == 'F') {              /* F seq state fflags tag [thr_q] -> F <72 bytes> (thr_q: v0.7.1, else 0) */
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
        } else if (kind == 'M') {              /* v0.8 nav3 encode */
            double w[27];
            uint32_t fb[12];
            long iv[9];
            uint8_t sl[5];
            nl_nav3_t n3;
            if (parse_numbers(g_line + 1, w, 27) != 27) {
                fprintf(stderr, "nlink-vectors: short M line %ld\n", rows);
                return 3;
            }
            for (i = 0; i < 12; i++) fb[i] = (uint32_t)w[1 + i];
            for (i = 0; i < 9; i++) iv[i] = (long)w[13 + i];
            for (i = 0; i < 5; i++) sl[i] = (uint8_t)w[22 + i];
            nav3_from_fields(&n3, (uint8_t)w[0], fb, iv, sl);
            memset(buf, 0xEE, sizeof buf);
            put_hex_line(fo, 'M', buf, nl_encode_nav3(&n3, buf) == NL_XFER_BYTES ? NL_XFER_BYTES : 0);
        } else if (kind == 'X') {              /* v0.8 nav3 decode */
            nl_nav3_t n3;
            if (parse_hex(g_line + 1, buf, NL_XFER_BYTES) != NL_XFER_BYTES) {
                fprintf(stderr, "nlink-vectors: X line %ld is not 72 bytes\n", rows);
                return 3;
            }
            if (nl_decode_nav3(buf, &n3)) {
                uint32_t fb[12];
                const float *fs[12] = {&n3.q[0], &n3.q[1], &n3.q[2], &n3.q[3], &n3.rate[0], &n3.rate[1], &n3.rate[2],
                                       &n3.accel_z, &n3.h_m, &n3.vz_mps, &n3.cos_tilt, &n3.tilt_deg};
                for (i = 0; i < 12; i++) memcpy(&fb[i], fs[i], 4);
                fprintf(fo, "X 1 %u", n3.seq);
                for (i = 0; i < 12; i++) fprintf(fo, " %08lx", (unsigned long)fb[i]);
                fprintf(fo, " %d %d %d %u %u %u %u %u %u ", n3.accel_x_q, n3.accel_y_q, n3.h_baro_rel_cm, n3.imu_fail,
                        n3.baro_fails, n3.twi_errors, n3.loop_us_max, n3.flags, n3.reset_cause);
                for (i = 0; i < 5; i++) fprintf(fo, "%02x", n3.slot[i]);
                fputc('\n', fo);
            } else {
                fputs("X 0\n", fo);
            }
        } else if (kind == 'W') {              /* v0.8 slot receiver over a tick sequence */
            static nl_slot_rx_t rx;
            const char *p = g_line + 1;
            nl_slot_rx_init(&rx);
            fputc('W', fo);
            for (;;) {
                uint8_t sl[NL_SLOT_LEN];
                const nl_gps_set_t *g;
                while (*p == ' ') p++;
                if (*p == '-') {
                    p++;
                    g = nl_slot_rx_feed(&rx, 0);
                } else if (hexval(p[0]) >= 0) {
                    if (parse_hex(p, sl, NL_SLOT_LEN) != NL_SLOT_LEN) break;
                    p += 2 * NL_SLOT_LEN;
                    g = nl_slot_rx_feed(&rx, sl);
                } else {
                    break;
                }
                fprintf(fo, " %d:%d:%d", g != 0, nl_slot_rx_gate_ok(&rx) ? 1 : 0, nl_slot_rx_fresh(&rx) ? 1 : 0);
                if (g != 0)
                    fprintf(fo, ":%u:%u:%u:%u:%d:%d:%d:%d:%d:%d:%d:%d:%u", g->flags, g->num_sv, g->hacc_dm, g->reset_count,
                            g->off[0], g->off[1], g->p[0], g->p[1], g->v[0], g->v[1], g->cs[0], g->cs[1], g->cycle);
            }
            fprintf(fo, " S %u %u\n", rx.sets, rx.broken);
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

/* vtol = 0: the ballistic build's gains (fc_default_gains, v0.6.3); 1 (--vectors-vtol, v0.7.1b): fc_vtol_gains */
static int run_vectors_mode(const char *in, const char *out, int vtol)
{
    FILE *fi = fopen(in, "r"), *fo = fopen(out, "w");
    attitude_ctrl_t c;
    att_gains_t g;
    mixer_plant_t p;
    ctrl_output_t o;
    double v[MAXF];
    long rows = 0;
    if (!fi || !fo) return 2;
    if (vtol) {
        fc_default_plant(&p);
        fc_vtol_gains(&g);
        attitude_ctrl_init(&c, &g, &p, fc_throttle_exp_int());
    } else {
        make_ctrl(&c, &g, &p);
    }
    fprintf(fo, "seq,d1,d2,d3,d4,mx,my,mz,rsx,rsy,rsz,ex,ey,ez,wfx,wfy,wfz,sat,that,ix,iy,iz,p1,p2,p3,p4\n");
    /* in: seq,reset,set_yaw,yqw,yqx,yqy,yqz,qw,qx,qy,qz,gx,gy,gz,u[,tx,ty,finish]
     * set_yaw: set_yaw_reference(yq) before the update; reset: reset() before the update; v0.8 (VTOL gains): the tilt
     * setpoint (tx, ty) through attitude_ctrl_set_tilt before the update, its second half by
     * attitude_ctrl_set_tilt_finish when finish != 0, else left to the update */
    int nr;
    while ((nr = read_row(fi, v, MAXF)) >= 15) {
        float yq[4] = {F(v[3]), F(v[4]), F(v[5]), F(v[6])};
        float q[4] = {F(v[7]), F(v[8]), F(v[9]), F(v[10])}, gy[3] = {F(v[11]), F(v[12]), F(v[13])};
        if (v[1] != 0.0) attitude_ctrl_reset(&c);
        if (v[2] != 0.0) attitude_ctrl_set_yaw_reference(&c, yq);
#if ATT_HAS_TILT_SP
        if (nr >= 18) {
            const float t[2] = {F(v[15]), F(v[16])};
            if (attitude_ctrl_set_tilt(&c, t) && v[17] != 0.0)
                attitude_ctrl_set_tilt_finish(&c);
        }
#endif
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
    fprintf(stderr, "vectors%s: %ld rows\n", vtol ? " (VTOL gains)" : "", rows);
    return 0;
}

static int run_vectors(const char *in, const char *out) { return run_vectors_mode(in, out, 0); }
static int run_vectors_vtol(const char *in, const char *out) { return run_vectors_mode(in, out, 1); }

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

/* v0.7.1 VTOL NAV estimator (sim/estimator.py GatedMahonyEstimator):
 *   #M n_avg kp ki_dt gate_mps2 g alpha_a bias_limit   new estimator with these constants (float32 text)
 *   row: seq,gx,gy,gz,ax,ay,az,on_pad
 *   out: seq,qw,qx,qy,qz,bx,by,bz,rx,ry,rz,gate,ex,ey,ez,alx,aly,alz,bpx,bpy,bpz */
static int run_mahony_vectors(const char *in, const char *out)
{
    FILE *fi = fopen(in, "r"), *fo = fopen(out, "w");
    static att_mahony_t m;
    double v[MAXF];
    long rows = 0;
    if (!fi || !fo) return 2;
    fc_default_mahony(&m);
    fprintf(fo, "seq,qw,qx,qy,qz,bx,by,bz,rx,ry,rz,gate,ex,ey,ez,alx,aly,alz,bpx,bpy,bpz,pqw,pqx,pqy,pqz,prx,pry,prz\n");
    while (fgets(g_line, sizeof g_line, fi)) {
        if (g_line[0] == '#' && g_line[1] == 'M') {
            if (parse_numbers(g_line + 2, v, 8) != 7) {
                fprintf(stderr, "mahony-vectors: bad #M line\n");
                return 3;
            }
            att_mahony_init(&m, FC_DT_S, (uint16_t)v[0], F(v[1]), F(v[2]), F(v[3]), F(v[4]), F(v[5]), F(v[6]));
            continue;
        }
        if (g_line[0] == '#' || g_line[0] == 's' || g_line[0] == '\n' || g_line[0] == '\r') continue;
        if (parse_numbers(g_line, v, MAXF) < 8) {
            fprintf(stderr, "mahony-vectors: short row %ld\n", rows);
            return 3;
        }
        {
            float gy[3] = {F(v[1]), F(v[2]), F(v[3])}, ac[3] = {F(v[4]), F(v[5]), F(v[6])}, r[3], pq[4], pr[3];
            att_mahony_predict(&m, gy, v[7] != 0.0, pq, pr);           /* v0.8: the published prediction, before */
            att_mahony_update(&m, gy, ac, v[7] != 0.0);
            att_mahony_rate(&m, gy, r);
            fprintf(fo, "%.0f,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
                        "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
                    v[0], m.pad.q[0], m.pad.q[1], m.pad.q[2], m.pad.q[3], m.pad.bias[0], m.pad.bias[1], m.pad.bias[2], r[0],
                    r[1], r[2], m.gate ? 1 : 0, m.e[0], m.e[1], m.e[2], m.a_lpf[0], m.a_lpf[1], m.a_lpf[2], m.b_pad[0],
                    m.b_pad[1], m.b_pad[2], pq[0], pq[1], pq[2], pq[3], pr[0], pr[1], pr[2]);
        }
        rows++;
    }
    fclose(fi);
    fclose(fo);
    fprintf(stderr, "mahony-vectors: %ld rows\n", rows);
    return 0;
}

/* v0.7.1 VTOL NAV altitude filter (sim/state_machine.py VtolAltitudeFilter):
 *   #A k1 k2 k3 latency tick_dt pad_n_avg bias_limit comp_m inv_u_hover exp_int exp   new filter (float32 text)
 *   row: seq,t_ticks,az,has_baro,h_baro,pad,u_prev
 *   out: seq,t,h,v,b,last_comp */
static int run_altv_vectors(const char *in, const char *out)
{
    FILE *fi = fopen(in, "r"), *fo = fopen(out, "w");
    static alt_vtol_t f;
    double v[MAXF];
    long rows = 0;
    if (!fi || !fo) return 2;
    fc_default_alt_vtol(&f, 0.0f);
    fprintf(fo, "seq,t,h,v,b,last_comp\n");
    while (fgets(g_line, sizeof g_line, fi)) {
        if (g_line[0] == '#' && g_line[1] == 'A') {
            alt_vtol_params_t p;
            if (parse_numbers(g_line + 2, v, 12) != 11) {
                fprintf(stderr, "altv-vectors: bad #A line\n");
                return 3;
            }
            p.k1 = F(v[0]);
            p.k2 = F(v[1]);
            p.k3 = F(v[2]);
            p.latency_s = F(v[3]);
            p.tick_dt = F(v[4]);
            p.pad_n_avg = (uint16_t)v[5];
            p.bias_limit_mps2 = F(v[6]);
            p.comp_m = F(v[7]);
            p.inv_u_hover = F(v[8]);
            p.throttle_exp_int = (int8_t)v[9];
            p.throttle_exp = F(v[10]);
            alt_vtol_init(&f, &p);
            continue;
        }
        if (g_line[0] == '#' || g_line[0] == 's' || g_line[0] == '\n' || g_line[0] == '\r') continue;
        if (parse_numbers(g_line, v, MAXF) < 7) {
            fprintf(stderr, "altv-vectors: short row %ld\n", rows);
            return 3;
        }
        alt_vtol_step(&f, F(v[2]), v[3] != 0.0, F(v[4]), (uint32_t)v[1], v[5] != 0.0, F(v[6]));
        fprintf(fo, "%.0f,%.0f,%.9g,%.9g,%.9g,%.9g\n", v[0], v[1], f.h, f.v, f.b, f.last_comp_m);
        rows++;
    }
    fclose(fi);
    fclose(fo);
    fprintf(stderr, "altv-vectors: %ld rows\n", rows);
    return 0;
}

/* ------------------------------------------------------------------------ */
/* v0.8 vector modes (tools/crosscheck_controller.py sections 8a / 8b)      */
/* ------------------------------------------------------------------------ */
/* position controller (sim/position_ctrl.py PositionHold; --pos-vectors):
 *   #P id [pp.key=value ...]   a new controller: fc_default_position + overrides (pos_override), pos_init
 *   row: seq,phase,motor_on,h,cos_tilt,r0,r1,r2,k_t,sat_high,has_data,flags,reset_count,off0,off1,p0,p1,v0,v1,cs0,cs1
 *        [,vc_low_alt,vc_ground,lifted] (slot integers: cm, cm/s, Q15; the physics-review inputs, absent: 0, 0, 1)
 *   out: seq,mode,requests,limited,guard,engaged,has_hold,lost,tl0,tl1,tw0,tw1,a0,a1,vs0,vs1,hold0,hold1,dist,cos_lim,
 *        a_lim2,i0,i1,implausible,lim_run,vexc_run,guard_left,dv0,dv1 - each step through the background pieces
 *        (pos_step: pos_latch + P0..P10) */
static int run_pos_vectors(const char *in, const char *out)
{
    FILE *fi = fopen(in, "r"), *fo = fopen(out, "w");
    static pos_t s;
    pos_params_t pp;
    double v[40];
    long rows = 0;
    int nv;
    if (!fi || !fo) return 2;
    fc_default_position(&pp);
    pos_init(&s, &pp);
    fprintf(fo, "seq,mode,requests,limited,guard,engaged,has_hold,lost,tl0,tl1,tw0,tw1,a0,a1,vs0,vs1,hold0,hold1,dist,cos_lim,"
                "a_lim2,i0,i1,implausible,lim_run,vexc_run,guard_left,dv0,dv1,trim0,trim1\n");
    while (fgets(g_line, sizeof g_line, fi)) {
        if (g_line[0] == '#' && g_line[1] == 'P') {
            char *tok = strtok(g_line + 2, " \t\r\n");
            fc_default_position(&pp);
            while (tok != 0 && (tok = strtok(0, " \t\r\n")) != 0) {
                if (!pos_override(&pp, tok)) {
                    fprintf(stderr, "pos-vectors: unknown override %s\n", tok);
                    return 3;
                }
            }
            pos_init(&s, &pp);
            continue;
        }
        if (g_line[0] == '#' || g_line[0] == 's' || g_line[0] == '\n' || g_line[0] == '\r') continue;
        nv = parse_numbers(g_line, v, 40);
        if (nv < 21) {
            fprintf(stderr, "pos-vectors: short row %ld\n", rows);
            return 3;
        }
        {
            pos_inputs_t pi;
            nl_gps_set_t d;
            const pos_outputs_t *o = &s.out;
            memset(&d, 0, sizeof d);
            pi.vc_low_alt = nv >= 24 && v[21] != 0.0;
            pi.vc_ground = nv >= 24 && v[22] != 0.0;
            pi.lifted = nv < 24 || v[23] != 0.0;
            pi.phase = (uint8_t)v[1];
            pi.motor_on = v[2] != 0.0;
            pi.h_m = F(v[3]);
            pi.cos_tilt = F(v[4]);
            pi.rate[0] = F(v[5]);
            pi.rate[1] = F(v[6]);
            pi.rate[2] = F(v[7]);
            pi.k_t = F(v[8]);
            pi.vc_sat_high = v[9] != 0.0;
            d.flags = (uint8_t)v[11];
            d.reset_count = (uint8_t)v[12];
            d.off[0] = (int16_t)v[13];
            d.off[1] = (int16_t)v[14];
            d.p[0] = (int16_t)v[15];
            d.p[1] = (int16_t)v[16];
            d.v[0] = (int16_t)v[17];
            d.v[1] = (int16_t)v[18];
            d.cs[0] = (int16_t)v[19];
            d.cs[1] = (int16_t)v[20];
            pi.data = 0;
            pi.set = v[10] != 0.0 ? &d : 0;
            (void)pos_step(&s, &pi);
            fprintf(fo, "%.0f,%u,%u,%d,%d,%d,%d,%u,", v[0], o->mode, o->requests, o->limited ? 1 : 0, o->guard ? 1 : 0,
                    o->engaged ? 1 : 0, o->has_hold ? 1 : 0, o->lost_steps);
            put_f(fo, o->tilt_l[0]); fputc(',', fo);
            put_f(fo, o->tilt_l[1]); fputc(',', fo);
            put_f(fo, o->tilt_w[0]); fputc(',', fo);
            put_f(fo, o->tilt_w[1]); fputc(',', fo);
            put_f(fo, o->a_cmd[0]); fputc(',', fo);
            put_f(fo, o->a_cmd[1]); fputc(',', fo);
            put_f(fo, o->v_sp[0]); fputc(',', fo);
            put_f(fo, o->v_sp[1]); fputc(',', fo);
            put_f(fo, o->hold[0]); fputc(',', fo);
            put_f(fo, o->hold[1]); fputc(',', fo);
            put_f(fo, o->dist_m); fputc(',', fo);
            put_f(fo, o->cos_lim); fputc(',', fo);
            put_f(fo, o->a_lim2); fputc(',', fo);
            put_f(fo, o->integ[0]); fputc(',', fo);
            put_f(fo, o->integ[1]);
            fprintf(fo, ",%d,%u,%u,%u,", s.implausible ? 1 : 0, s.lim_run, s.vexc_run, s.guard_left);
            put_f(fo, s.dv[0]); fputc(',', fo);
            put_f(fo, s.dv[1]); fputc(',', fo);
            put_f(fo, o->trim_l[0]); fputc(',', fo);   /* v0.8.1 */
            put_f(fo, o->trim_l[1]);
            fputc('\n', fo);
        }
        rows++;
    }
    fclose(fi);
    fclose(fo);
    fprintf(stderr, "pos-vectors: %ld rows\n", rows);
    return 0;
}

/* the NAV's GPS side (sim/gps_nav.py NavGps: parser -> configurator / assembler + iTOW clock -> filter; --gpsnav-vectors):
 *   #G id configure        a new gps_nav_t (fc_default_gps: params_gen.h parameters, boot tick counts and baud)
 *   row: k,ax,ay,az,qw,qx,qy,qz,wx,wy,wz,pad,rxhex   (rxhex: the UART bytes of tick k, '-' = none)
 *   out: k,ret,act_baud,act_tx(hex|-),epochs,parser frames:bad_ck:bad_len:truncated:skipped,asm epochs:incomplete:duplicate,
 *        clock has:offset:relearned,cfg state:cfg_ok:ver_ok:module:naks:timeouts,init,pad,acq_left,reset_count,cycle,yaw_used,
 *        fused:rej_p:rej_v:pos_resets:vel_resets:dropped_old:dropped_stale:n_epoch_dropped:unusable,p0,p1,v0,v1,b0,b1,off0,off1,
 *        c_psi,s_psi,p_psi,yaw_corr,slot (the slot of the frame of tick k + 1, after this tick: nl_slot_tx latched on
 *        GPS_TICK_STEP / GPS_TICK_SNAP) */
static int run_gpsnav_vectors(const char *in, const char *out)
{
    FILE *fi = fopen(in, "r"), *fo = fopen(out, "w");
    static gps_nav_t g;
    static uint8_t rx[1u << 14];
    nl_slot_tx_t tx;
    double v[16];
    long rows = 0;
    if (!fi || !fo) return 2;
    fc_default_gps(&g, true);
    nl_slot_tx_init(&tx);
    fprintf(fo, "k,ret,baud,tx,epochs,parser,asm,clock,cfg,init,pad,acq_left,reset_count,cycle,yaw_used,stats,p0,p1,v0,v1,b0,"
                "b1,off0,off1,c_psi,s_psi,p_psi,yaw_corr,slot\n");
    while (fgets(g_line, sizeof g_line, fi)) {
        if (g_line[0] == '#' && g_line[1] == 'G') {
            double h[2];
            if (parse_numbers(g_line + 2, h, 2) != 2) {
                fprintf(stderr, "gpsnav-vectors: bad #G line\n");
                return 3;
            }
            fc_default_gps(&g, h[1] != 0.0);
            nl_slot_tx_init(&tx);
            continue;
        }
        if (g_line[0] == '#' || g_line[0] == 'k' || g_line[0] == '\n' || g_line[0] == '\r') continue;
        {
            char *p = g_line;
            int nf = 0, n_rx = 0, i;
            uint8_t ret, slot[NL_SLOT_LEN];
            uint32_t k;
            gps_cfg_action_t act;
            float a[3], q[4], w[3];
            const gps_filter_t *f = &g.filter;
            while (nf < 12) {                          /* 12 numbers, then the hex field */
                char *end;
                v[nf] = strtod_x(p, &end);
                if (end == p) break;
                nf++;
                p = end;
                while (*p == ',' || *p == ' ') p++;
            }
            if (nf != 12) {
                fprintf(stderr, "gpsnav-vectors: short row %ld\n", rows);
                return 3;
            }
            if (*p != '-')
                n_rx = parse_hex(p, rx, (int)sizeof rx);
            k = (uint32_t)v[0];
            for (i = 0; i < 3; i++) a[i] = F(v[1 + i]);
            for (i = 0; i < 4; i++) q[i] = F(v[4 + i]);
            for (i = 0; i < 3; i++) w[i] = F(v[8 + i]);
            ret = gps_nav_tick(&g, k, rx, (uint16_t)n_rx, a, q, w, v[11] != 0.0, &act);
            if (ret != GPS_TICK_NONE)
                nl_slot_tx_latch(&tx, &g.filter.snap, g.filter.cycle);
            nl_slot_tx_slot(&tx, k + 1u, slot);
            fprintf(fo, "%lu,%u,%lu,", (unsigned long)k, ret, (unsigned long)act.baud);
            if (act.tx != GPS_TX_NONE) {
                uint8_t fr[UBX_FRAME_MAX];
                const uint16_t n = gps_cfg_frame(act.tx, g.cfg.baud, GPS_MEAS_MS, fr);
                for (i = 0; i < (int)n; i++) fprintf(fo, "%02x", fr[i]);
            } else {
                fputc('-', fo);
            }
            fprintf(fo, ",%u,%u:%u:%u:%u:%u,%u:%u:%u,%d:%ld:%u,%u:%d:%d:%u:%u:%u,%d,%d,%u,%u,%u,%u,", g.epochs_this_tick,
                    g.parser.stats.frames, g.parser.stats.bad_ck, g.parser.stats.bad_len, g.parser.stats.truncated,
                    g.parser.stats.skipped, g.asm_.epochs, g.asm_.incomplete, g.asm_.duplicate, g.clock.has_offset ? 1 : 0,
                    (long)g.clock.offset, g.clock.relearned, g.cfg.state, g.cfg.cfg_ok ? 1 : 0, g.cfg.ver_ok ? 1 : 0,
                    g.cfg.module, g.cfg.naks, g.cfg.timeouts, f->init ? 1 : 0, f->pad ? 1 : 0, f->acq_left, f->reset_count,
                    f->cycle, f->yaw_used);
            fprintf(fo, "%u:%u:%u:%u:%u:%u:%u:%u:%u,", f->st_fused, f->st_rej_p, f->st_rej_v, f->st_pos_resets,
                    f->st_vel_resets, f->st_dropped_old, f->st_dropped_stale, f->st_n_epoch_dropped, f->st_unusable);
            put_f(fo, f->p_[0]); fputc(',', fo);
            put_f(fo, f->p_[1]); fputc(',', fo);
            put_f(fo, f->v[0]); fputc(',', fo);
            put_f(fo, f->v[1]); fputc(',', fo);
            put_f(fo, f->b[0]); fputc(',', fo);
            put_f(fo, f->b[1]); fputc(',', fo);
            put_f(fo, f->off[0]); fputc(',', fo);
            put_f(fo, f->off[1]); fputc(',', fo);
            put_f(fo, f->c_psi); fputc(',', fo);
            put_f(fo, f->s_psi); fputc(',', fo);
            put_f(fo, f->p_psi); fputc(',', fo);
            put_f(fo, f->yaw_corr); fputc(',', fo);
            for (i = 0; i < (int)NL_SLOT_LEN; i++) fprintf(fo, "%02x", slot[i]);
            fputc('\n', fo);
        }
        rows++;
    }
    fclose(fi);
    fclose(fo);
    fprintf(stderr, "gpsnav-vectors: %ld rows\n", rows);
    return 0;
}


/* ------------------------------------------------------------------------ */
/* v0.8 GPS position hold: UBX, nlink v3, take-off check, tilted setpoint,  */
/* NAV prediction, the NAV GPS filter, the position controller, the        */
/* supervisor rules and the FCU stack                                       */
/* ------------------------------------------------------------------------ */
static int ubx_fields(uint8_t kind, const ubx_parser_t *p, long long f[17])
{
    if (kind == UBX_K_POSLLH) {
        ubx_posllh_t m;
        ubx_get_posllh(p->buf, &m);
        f[0] = m.itow; f[1] = m.lon; f[2] = m.lat; f[3] = m.height; f[4] = m.hmsl; f[5] = m.hacc; f[6] = m.vacc;
        return 7;
    }
    if (kind == UBX_K_VELNED) {
        ubx_velned_t m;
        ubx_get_velned(p->buf, &m);
        f[0] = m.itow; f[1] = m.vel_n; f[2] = m.vel_e; f[3] = m.vel_d; f[4] = m.speed; f[5] = m.gspeed; f[6] = m.heading;
        f[7] = m.sacc; f[8] = m.cacc;
        return 9;
    }
    if (kind == UBX_K_SOL) {
        ubx_sol_t m;
        ubx_get_sol(p->buf, &m);
        f[0] = m.itow; f[1] = m.ftow; f[2] = m.week; f[3] = m.gps_fix; f[4] = m.flags; f[5] = m.ecef[0]; f[6] = m.ecef[1];
        f[7] = m.ecef[2]; f[8] = m.pacc; f[9] = m.ecef_v[0]; f[10] = m.ecef_v[1]; f[11] = m.ecef_v[2]; f[12] = m.sacc;
        f[13] = m.pdop; f[14] = m.res1; f[15] = m.num_sv; f[16] = m.res2;
        return 17;
    }
    if (kind == UBX_K_ACK || kind == UBX_K_NAK) {
        f[0] = p->buf[0];
        f[1] = p->buf[1];
        return 2;
    }
    if (kind == UBX_K_MONVER) {
        f[0] = ubx_monver_module(p->buf);
        return 1;
    }
    return 0;
}

/* one completed parser frame against a vector message: header, the stored payload, kind and fields */
static int ubx_msg_matches(const ubx_parser_t *p, const gpsv_msg_t *m)
{
    long long f[17];
    uint8_t kind = ubx_msg_kind(p);
    int nf, i;
    const uint16_t stored = p->length < UBX_MAX_PAYLOAD ? p->length : UBX_MAX_PAYLOAD;
    if (p->cls != m->cls || p->id != m->id || p->length != m->length || (p->length > UBX_MAX_PAYLOAD) != (m->truncated != 0) ||
        stored != m->n_pl || memcmp(p->buf, m->payload, stored) != 0 || kind != m->kind)
        return 0;
    nf = ubx_fields(kind, p, f);
    if (nf != m->n_f)
        return 0;
    for (i = 0; i < nf; i++)
        if (f[i] != m->f[i])
            return 0;
    return 1;
}

static void test_gps_ubx(void)
{
    static ubx_parser_t p;
    int i, j;
    {
        static const long ckw[2] = GPSV_CK_CHECK;
        const uint8_t d[2] = {1, 2};
        uint8_t a, b;
        ubx_checksum(d, 2, &a, &b);
        CHECK(a == ckw[0] && b == ckw[1], "UBX Fletcher checksum of 01 02 = %u %u (reference %ld %ld)", a, b, ckw[0], ckw[1]);
    }
    CHECK(GPSV_MAX_PAYLOAD == UBX_MAX_PAYLOAD && GPSV_LEN_LIMIT == UBX_LEN_LIMIT && GPSV_BUF_N == GPS_BUF_N,
          "UBX constants: stored payload 64, length limit 256, delayed-state ring 16");
    {
        static const long fl[8] = GPSV_FLAGS;
        CHECK(fl[0] == GPS_F_FIX_OK && fl[1] == GPS_F_GATE_OK && fl[2] == GPS_F_HEAD_OK && fl[3] == GPS_F_FIX3D &&
              fl[4] == GPS_F_CFG_OK && fl[5] == GPS_F_INIT && fl[6] == GPS_F_VER_OK && fl[7] == GPS_F_INCONS &&
              POS_GF_FIX_OK == GPS_F_FIX_OK && POS_GF_INCONS == GPS_F_INCONS, "GPS snapshot flag bits == sim/gps_nav.py GF_*");
    }
    /* messages: bytes -> one frame with the reference's fields; re-encoding a stored payload gives the bytes */
    for (i = 0; i < GPSV_N_MSGS; i++) {
        const gpsv_frame_t *v = &GPSV_MSGS[i];
        int n_fr = 0, ok = 0;
        ubx_parser_init(&p);
        for (j = 0; j < (int)v->n; j++)
            if (ubx_parse_byte(&p, v->frame[j])) {
                n_fr++;
                ok = ubx_msg_matches(&p, &v->m);
            }
        CHECK(n_fr == 1 && ok, "UBX message %s: one frame, fields == the reference (frames %d)", v->name, n_fr);
        if (!v->m.truncated) {
            uint8_t fr[UBX_OVERHEAD + 300];
            const uint16_t n = ubx_frame(v->m.cls, v->m.id, v->m.payload, v->m.length, fr);
            CHECK(n == v->n && memcmp(fr, v->frame, n) == 0, "UBX message %s: ubx_frame re-encodes byte-exact", v->name);
        }
    }
    /* the boot configuration frames, byte-exact (the published u-center bytes in the reference report) */
    for (i = 0; i < GPSV_N_CFG; i++) {
        const gpsv_cfg_t *v = &GPSV_CFG[i];
        uint8_t fr[UBX_FRAME_MAX];
        uint16_t n = 0;
        if (!strcmp(v->name, "CFG-PRT 38400")) n = ubx_cfg_prt_uart1(38400UL, fr);
        else if (!strncmp(v->name, "CFG-PRT 9600", 12)) n = ubx_cfg_prt_uart1(9600UL, fr);
        else if (!strcmp(v->name, "CFG-RATE")) n = gps_cfg_frame(GPS_TX_RATE, GPS_BAUD, GPS_MEAS_MS, fr);
        else if (!strcmp(v->name, "CFG-MSG POSLLH")) n = gps_cfg_frame(GPS_TX_MSG_POSLLH, GPS_BAUD, GPS_MEAS_MS, fr);
        else if (!strcmp(v->name, "CFG-MSG VELNED")) n = gps_cfg_frame(GPS_TX_MSG_VELNED, GPS_BAUD, GPS_MEAS_MS, fr);
        else if (!strcmp(v->name, "CFG-MSG SOL")) n = gps_cfg_frame(GPS_TX_MSG_SOL, GPS_BAUD, GPS_MEAS_MS, fr);
        else if (!strcmp(v->name, "CFG-NAV5")) n = gps_cfg_frame(GPS_TX_NAV5, GPS_BAUD, GPS_MEAS_MS, fr);
        else if (!strcmp(v->name, "MON-VER poll")) n = gps_cfg_frame(GPS_TX_VER, GPS_BAUD, GPS_MEAS_MS, fr);
        CHECK(n == v->n && memcmp(fr, v->frame, n) == 0, "configuration frame %s byte-exact (%u bytes)", v->name, n);
    }
    {
        uint8_t fr[UBX_FRAME_MAX];
        CHECK(gps_cfg_frame(GPS_TX_PRT, GPS_BAUD, GPS_MEAS_MS, fr) == UBX_OVERHEAD + UBX_LEN_CFG_PRT && fr[14] == 0x00 &&
              fr[15] == 0x96 && gps_cfg_frame(GPS_TX_NONE, 0, 0, fr) == 0, "GPS_TX_PRT = CFG-PRT at GPS_BAUD (38400)");
    }
    /* parser streams: every message, every counter, the state after the stream */
    for (i = 0; i < GPSV_N_STREAMS; i++) {
        const gpsv_stream_t *v = &GPSV_STREAMS[i];
        int n_msgs = 0, bad = 0;
        ubx_parser_init(&p);
        for (j = 0; j < (int)v->n; j++) {
            if (ubx_parse_byte(&p, v->data[j])) {
                if (n_msgs >= v->n_msgs || !ubx_msg_matches(&p, &v->msgs[n_msgs]))
                    bad++;
                n_msgs++;
            }
        }
        CHECK(n_msgs == v->n_msgs && bad == 0, "UBX stream %s: %d messages (want %d), %d differ", v->name, n_msgs, v->n_msgs, bad);
        CHECK(p.stats.frames == v->stats[0] && p.stats.bad_ck == v->stats[1] && p.stats.bad_len == v->stats[2] &&
              p.stats.truncated == v->stats[3] && p.stats.skipped == v->stats[4] && p.state == v->state_after,
              "UBX stream %s: counters %u %u %u %u %u state %u (reference %u %u %u %u %u state %u)", v->name, p.stats.frames,
              p.stats.bad_ck, p.stats.bad_len, p.stats.truncated, p.stats.skipped, p.state, v->stats[0], v->stats[1],
              v->stats[2], v->stats[3], v->stats[4], v->state_after);
    }
    /* epoch assembler: the reference's message sequences (its POSLLH / VELNED / SOL with the iTOW changed) */
    {
        const uint8_t *base[4] = {0, 0, 0, 0};
        for (i = 0; i < GPSV_N_MSGS; i++) {
            if (!strcmp(GPSV_MSGS[i].name, "posllh")) base[1] = GPSV_MSGS[i].m.payload;
            if (!strcmp(GPSV_MSGS[i].name, "velned")) base[2] = GPSV_MSGS[i].m.payload;
            if (!strcmp(GPSV_MSGS[i].name, "sol")) base[3] = GPSV_MSGS[i].m.payload;
        }
        CHECK(base[1] && base[2] && base[3], "the assembler's base messages are in the vectors");
        for (i = 0; i < GPSV_N_ASM && base[1] && base[2] && base[3]; i++) {
            const gpsv_asm_t *v = &GPSV_ASM[i];
            gps_asm_t a;
            int n_ep = 0, bad = 0, q;
            gps_asm_init(&a);
            for (j = 0; j < v->n_in; j++) {
                uint8_t pl[UBX_LEN_SOL];
                const uint8_t kind = v->in[j].kind;
                const gps_epoch_t *e;
                memcpy(pl, base[kind], kind == UBX_K_POSLLH ? UBX_LEN_POSLLH : kind == UBX_K_VELNED ? UBX_LEN_VELNED : UBX_LEN_SOL);
                pl[0] = (uint8_t)v->in[j].itow;
                pl[1] = (uint8_t)(v->in[j].itow >> 8);
                pl[2] = (uint8_t)(v->in[j].itow >> 16);
                pl[3] = (uint8_t)(v->in[j].itow >> 24);
                e = gps_asm_add(&a, kind, pl, v->in[j].tick);
                if (e) {
                    const long long f[15] = {e->itow, e->lat, e->lon, e->hmsl, e->hacc, e->vacc, e->vel_n, e->vel_e, e->vel_d,
                                             e->sacc, e->gps_fix, e->fix_ok ? 1 : 0, e->num_sv, e->pacc, e->tick};
                    if (n_ep >= v->n_ep)
                        bad++;
                    else
                        for (q = 0; q < 15; q++)
                            if (f[q] != v->ep[n_ep].f[q]) bad++;
                    n_ep++;
                }
            }
            CHECK(n_ep == v->n_ep && bad == 0 && a.epochs == v->stats[0] && a.incomplete == v->stats[1] &&
                  a.duplicate == v->stats[2], "epoch assembler %s: %d epochs (want %d), %d fields differ, counters %u %u %u",
                  v->name, n_ep, v->n_ep, bad, a.epochs, a.incomplete, a.duplicate);
        }
    }
    {   /* iTOW clock */
        gps_itow_t c;
        int bad = 0;
        gps_itow_init(&c, 4, 220);
        for (i = 0; i < GPSV_N_CLOCK; i++) {
            const uint32_t te = gps_itow_update(&c, GPSV_CLOCK[i].itow, GPSV_CLOCK[i].tick);
            if (te != GPSV_CLOCK[i].t_e || c.offset != GPSV_CLOCK[i].offset || c.relearned != GPSV_CLOCK[i].relearned)
                bad++;
        }
        CHECK(bad == 0, "iTOW clock: t_e, offset, re-learn == the reference on %d epochs (%d differ)", GPSV_N_CLOCK, bad);
    }
    {   /* local frame: float32 radii and positions within float32 rounding of the reference's float64 */
        gps_frame_t fr;
        double worst = 0.0;
        gps_frame_at(&fr, GPSV_LF_LAT0, GPSV_LF_LON0);
        CHECK(NEAR(fr.k_n, GPSV_LF_K_N, 2e-7 * GPSV_LF_K_N) && NEAR(fr.k_e, GPSV_LF_K_E, 2e-7 * GPSV_LF_K_E),
              "local frame radii k_n %.9g / k_e %.9g (reference %.9g / %.9g)", fr.k_n, fr.k_e, GPSV_LF_K_N, GPSV_LF_K_E);
        for (i = 0; i < GPSV_N_POINTS; i++) {
            float x, y;
            gps_frame_to_w(&fr, GPSV_POINTS[i].lat, GPSV_POINTS[i].lon, &x, &y);
            worst = fmax(worst, fabs(x - GPSV_POINTS[i].x) / (1.0 + fabs(GPSV_POINTS[i].x)));
            worst = fmax(worst, fabs(y - GPSV_POINTS[i].y) / (1.0 + fabs(GPSV_POINTS[i].y)));
        }
        CHECK(worst < 5e-7, "local frame: x north / y west within 5e-7 relative of the reference (worst %.2g)", worst);
        {
            float x, y;                                /* the longitude wraps at +-180 deg (Python floor modulo) */
            gps_frame_t f2;
            gps_frame_at(&f2, 0, 1799999000L);
            gps_frame_to_w(&f2, 0, -1799999000L, &x, &y);
            CHECK(NEAR(y, -2000.0 * f2.k_e, 1e-3), "antimeridian: 0.0002 deg east across +-180 deg is y = %.4f m", y);
        }
    }
    {   /* the boot configurator, scripted receivers (the cross-check runs the reference's receiver stub tick for tick) */
        gps_cfg_t c;
        gps_cfg_action_t a;
        int t, n_tx = 0, first_tx = -1;
        uint8_t seen[8] = {0};
        gps_cfg_init(&c, GPS_CFG_N_WAIT, GPS_CFG_N_ACK, GPS_CFG_N_SETTLE, GPS_CFG_N_VER, GPS_BAUD);
        /* A: a receiver kept at 38400 by its backup cell: answers every frame on the next tick */
        {
            uint8_t pending = GPS_TX_NONE;
            for (t = 0; t < 2000 && !gps_cfg_done(&c); t++) {
                if (pending == GPS_TX_VER)
                    gps_cfg_msg(&c, UBX_K_MONVER, 0, 0, UBX_MODULE_UBLOX6);
                else if (pending != GPS_TX_NONE)
                    gps_cfg_msg(&c, UBX_K_ACK, UBX_CLS_CFG, pending == GPS_TX_PRT ? UBX_ID_CFG_PRT : pending == GPS_TX_RATE ?
                                UBX_ID_CFG_RATE : pending == GPS_TX_NAV5 ? UBX_ID_CFG_NAV5 : UBX_ID_CFG_MSG, 0);
                gps_cfg_tick(&c, &a);
                pending = a.tx;
                if (a.tx != GPS_TX_NONE) {
                    if (first_tx < 0) first_tx = t + 1;
                    seen[a.tx] = 1;
                    n_tx++;
                }
            }
            CHECK(gps_cfg_done(&c) && c.cfg_ok && c.ver_ok && c.module == UBX_MODULE_UBLOX6 && c.naks == 0 &&
                  c.timeouts == 0 && n_tx == 7 && first_tx == (int)GPS_CFG_N_WAIT && t == (int)GPS_CFG_N_WAIT + 7,
                  "configurator, receiver at 38400: PRT RATE MSG x3 NAV5 VER in %d ticks after the %u-tick wait (sent %d, "
                  "cfg_ok %d ver_ok %d)", t - (int)GPS_CFG_N_WAIT, GPS_CFG_N_WAIT, n_tx, c.cfg_ok, c.ver_ok);
            for (j = (int)GPS_TX_PRT; j <= (int)GPS_TX_VER; j++)
                CHECK(seen[j], "configurator sent frame %d", j);
        }
        /* B: silent receiver: probe, three 9600 attempts, every frame three times, MON-VER three times */
        gps_cfg_init(&c, GPS_CFG_N_WAIT, GPS_CFG_N_ACK, GPS_CFG_N_SETTLE, GPS_CFG_N_VER, GPS_BAUD);
        {
            int n9600 = 0, n38400 = 0;
            n_tx = 0;
            for (t = 0; t < 20000 && !gps_cfg_done(&c); t++) {
                gps_cfg_tick(&c, &a);
                if (a.baud == UBX_BAUD_DEFAULT) n9600++;
                if (a.baud == GPS_BAUD) n38400++;
                if (a.tx != GPS_TX_NONE) n_tx++;
            }
            CHECK(gps_cfg_done(&c) && !c.cfg_ok && !c.ver_ok && c.timeouts == 20 && c.naks == 0 && n9600 == 3 &&
                  n38400 == 4 && n_tx == 1 + 3 + 3 + 15 + 3,
                  "configurator, silent receiver: %d timeouts (want 1 + 3 + 15 + 1), 9600 x %d, 38400 x %d, %d frames", c.timeouts,
                  n9600, n38400, n_tx);
        }
        /* C: NAK on CFG-NAV5 every time: three tries, then MON-VER with cfg_ok false */
        gps_cfg_init(&c, GPS_CFG_N_WAIT, GPS_CFG_N_ACK, GPS_CFG_N_SETTLE, GPS_CFG_N_VER, GPS_BAUD);
        {
            uint8_t pending = GPS_TX_NONE;
            int nav5 = 0;
            for (t = 0; t < 2000 && !gps_cfg_done(&c); t++) {
                if (pending == GPS_TX_VER)
                    gps_cfg_msg(&c, UBX_K_MONVER, 0, 0, UBX_MODULE_UNKNOWN);
                else if (pending == GPS_TX_NAV5)
                    gps_cfg_msg(&c, UBX_K_NAK, UBX_CLS_CFG, UBX_ID_CFG_NAV5, 0);
                else if (pending != GPS_TX_NONE)
                    gps_cfg_msg(&c, UBX_K_ACK, UBX_CLS_CFG, pending == GPS_TX_PRT ? UBX_ID_CFG_PRT : pending == GPS_TX_RATE ?
                                UBX_ID_CFG_RATE : UBX_ID_CFG_MSG, 0);
                gps_cfg_tick(&c, &a);
                pending = a.tx;
                if (a.tx == GPS_TX_NAV5) nav5++;
            }
            CHECK(gps_cfg_done(&c) && !c.cfg_ok && c.ver_ok && c.module == UBX_MODULE_UNKNOWN && c.naks == 3 && nav5 == 3 &&
                  c.acked == 4, "configurator, NAK on NAV5: sent 3 times, %u NAKs, acked %u, cfg_ok %d", c.naks, c.acked,
                  c.cfg_ok);
        }
    }
    {   /* MON-VER hwVersion -> module (NUL-terminated, Python strip() of ASCII whitespace) */
        uint8_t pl[40];
        memset(pl, 0, sizeof pl);
        memcpy(pl + 30, "00040007", 8);
        CHECK(ubx_monver_module(pl) == UBX_MODULE_UBLOX6, "hw 00040007 -> u-blox 6");
        memcpy(pl + 30, " 00080000\t", 10);
        CHECK(ubx_monver_module(pl) == UBX_MODULE_UBLOX8, "hw ' 00080000\\t' (stripped) -> u-blox 8");
        memset(pl + 30, 0, 10);
        memcpy(pl + 30, "00070000", 8);                /* 2026-11-06: NEO-7M (sim/gps_nav.py HW_UBLOX7) */
        CHECK(ubx_monver_module(pl) == UBX_MODULE_UBLOX7 && UBX_MODULE_UBLOX7 == 7u, "hw 00070000 -> u-blox 7");
        memcpy(pl + 30, "00070001", 8);
        CHECK(ubx_monver_module(pl) == UBX_MODULE_UNKNOWN, "hw 00070001 -> unknown");
        memcpy(pl + 30, "00040000", 8);
        CHECK(ubx_monver_module(pl) == UBX_MODULE_UNKNOWN, "hw 00040000 -> unknown");
        memset(pl + 30, 0, 10);
        memcpy(pl + 30, "0004000700", 10);
        CHECK(ubx_monver_module(pl) == UBX_MODULE_UNKNOWN, "hw 0004000700 (no NUL, 10 chars) -> unknown");
        memset(pl + 30, 0, 10);
        memcpy(pl + 30, "00040007", 8);
        pl[38] = 'x';
        CHECK(ubx_monver_module(pl) == UBX_MODULE_UNKNOWN, "hw 00040007x -> unknown");
    }
}

static void nav3_from_fields(nl_nav3_t *n, uint8_t seq, const uint32_t fb[12], const long iv[9], const uint8_t slot[5])
{
    float f[12];
    int i;
    for (i = 0; i < 12; i++) memcpy(&f[i], &fb[i], 4);
    n->seq = seq;
    for (i = 0; i < 4; i++) n->q[i] = f[i];
    for (i = 0; i < 3; i++) n->rate[i] = f[4 + i];
    n->accel_z = f[7]; n->h_m = f[8]; n->vz_mps = f[9]; n->cos_tilt = f[10]; n->tilt_deg = f[11];
    n->accel_x_q = (int16_t)iv[0]; n->accel_y_q = (int16_t)iv[1]; n->h_baro_rel_cm = (int16_t)iv[2];
    n->imu_fail = (uint8_t)iv[3]; n->baro_fails = (uint8_t)iv[4]; n->twi_errors = (uint8_t)iv[5];
    n->loop_us_max = (uint16_t)iv[6]; n->flags = (uint8_t)iv[7]; n->reset_cause = (uint8_t)iv[8];
    for (i = 0; i < 5; i++) n->slot[i] = slot[i];
}

static int nav3_matches(const nl_nav3_t *n, uint8_t seq, const uint32_t fb[12], const long iv[9], const uint8_t slot[5])
{
    nl_nav3_t m;
    nav3_from_fields(&m, seq, fb, iv, slot);
    return m.seq == n->seq && !memcmp(m.q, n->q, 16) && !memcmp(m.rate, n->rate, 12) && !memcmp(&m.accel_z, &n->accel_z, 4) &&
           !memcmp(&m.h_m, &n->h_m, 4) && !memcmp(&m.vz_mps, &n->vz_mps, 4) && !memcmp(&m.cos_tilt, &n->cos_tilt, 4) &&
           !memcmp(&m.tilt_deg, &n->tilt_deg, 4) && m.accel_x_q == n->accel_x_q && m.accel_y_q == n->accel_y_q &&
           m.h_baro_rel_cm == n->h_baro_rel_cm && m.imu_fail == n->imu_fail && m.baro_fails == n->baro_fails &&
           m.twi_errors == n->twi_errors && m.loop_us_max == n->loop_us_max && m.flags == n->flags &&
           m.reset_cause == n->reset_cause && !memcmp(m.slot, n->slot, 5);
}

static void test_nlink_v3(void)
{
    int i, j, bad;
    uint8_t buf[NL_XFER_BYTES];
    nl_nav3_t n;
    CHECK(NL_TYPE_NAV3 == NLV3_TYPE_NAV3 && NL_NAV3_LEN == NLV3_NAV3_LEN && NL_NAV3_LEN == NL_NAV_LEN &&
          NL_SLOT_LEN == NLV3_SLOT_LEN && NL_SLOT_STALE_TICKS == NLV3_STALE_TICKS && NL_XFER_BYTES == 72u,
          "nlink v3: TYPE 3, payload 66 (= v0.6.3), slot 5 bytes, stale 10 ticks, the 72-byte exchange kept");
    for (i = 0; i < NLV3_N_NAVS; i++) {
        const nlv3_nav_t *v = &NLV3_NAVS[i];
        nav3_from_fields(&n, v->seq, v->fb, v->iv, v->slot);
        memset(buf, 0xEE, sizeof buf);
        CHECK(nl_encode_nav3(&n, buf) == NL_XFER_BYTES && memcmp(buf, v->frame, NL_XFER_BYTES) == 0 &&
              memcmp(buf + NL_HEADER_BYTES, v->payload, NL_NAV3_LEN) == 0, "nav3 %s: nl_encode_nav3 == reference frame", v->name);
        memset(&n, 0xA7, sizeof n);
        CHECK(nl_decode_nav3(v->frame, &n) == 1u && nav3_matches(&n, v->seq, v->fb, v->iv, v->slot), "nav3 %s: decode -> fields",
              v->name);
        {   /* the NAV's cos_tilt / tilt from the published q: cos bit-exact, tilt (acosf vs acos) within 2e-5 deg */
            float c, t, fb10, fb11;
            nl_nav3_tilt(n.q, &c, &t);
            memcpy(&fb10, &v->fb[10], 4);
            memcpy(&fb11, &v->fb[11], 4);
            if (n.q[0] == n.q[0] && fabsf(n.q[0]) <= 1.0f)
                CHECK(c == fb10 && NEAR(t, fb11, 2e-5 * (1.0 + fabs(fb11))), "nav3 %s: nl_nav3_tilt cos %.9g / %.9g, tilt %.9g / %.9g",
                      v->name, c, fb10, t, fb11);
        }
    }
    for (i = 0, bad = 0; i < NLV3_N_DECODES; i++) {
        const nlv3_dec_t *v = &NLV3_DECODES[i];
        nl_nav_t nv;
        nl_fcu_t fc;
        const uint8_t ok3 = nl_decode_nav3(v->buf, &n);
        if (ok3 != v->nav3_valid || nl_decode_nav(v->buf, &nv) != v->nav_valid || nl_decode_fcu(v->buf, &fc) != v->fcu_valid)
            bad++;
        else if (ok3 && !nav3_matches(&n, v->seq, v->fb, v->iv, v->slot))
            bad++;
    }
    CHECK(bad == 0, "nlink v3 decodes: validity of the v3 / v0.6.3 NAV / FCU decoders and fields on %d buffers (%d differ)",
          NLV3_N_DECODES, bad);
    {   /* one snapshot -> its five slots; no snapshot -> 07 00 00 00 00 */
        static const long sv[12] = NLV3_SNAP;
        nl_gps_snap_t s;
        nl_slot_tx_t tx;
        uint8_t sl[NL_SLOT_LEN];
        s.flags = (uint8_t)sv[0]; s.num_sv = (uint8_t)sv[1]; s.hacc_dm = (uint8_t)sv[2]; s.reset_count = (uint8_t)sv[3];
        s.off_x_cm = (int16_t)sv[4]; s.off_y_cm = (int16_t)sv[5]; s.px_cm = (int16_t)sv[6]; s.py_cm = (int16_t)sv[7];
        s.vx_cms = (int16_t)sv[8]; s.vy_cms = (int16_t)sv[9]; s.cos_q15 = (int16_t)sv[10]; s.sin_q15 = (int16_t)sv[11];
        for (j = 0, bad = 0; j < 5; j++) {
            nl_slot_encode(&s, NLV3_SLOTS_CYCLE, (uint8_t)j, sl);
            if (memcmp(sl, NLV3_SLOTS[j], NL_SLOT_LEN)) bad++;
        }
        CHECK(bad == 0, "the snapshot's five slots == the reference's (%d differ)", bad);
        nl_slot_tx_init(&tx);
        nl_slot_tx_slot(&tx, 3, sl);
        CHECK(sl[0] == NL_SLOT_EMPTY_HDR && sl[1] == 0 && sl[2] == 0 && sl[3] == 0 && sl[4] == 0, "no snapshot yet: 07 00 00 00 00");
        nl_slot_tx_latch(&tx, &s, (uint16_t)(0x100u + NLV3_SLOTS_CYCLE));
        for (j = 0, bad = 0; j < 10; j++) {
            nl_slot_tx_slot(&tx, (uint32_t)(1000u + j), sl);
            if (memcmp(sl, NLV3_SLOTS[(1000 + j) % 5], NL_SLOT_LEN)) bad++;
        }
        CHECK(bad == 0, "slot TX: the frame of NAV tick k carries part k mod 5, cycle & 0x1F (%d differ)", bad);
    }
    for (i = 0; i < NLV3_N_RX; i++) {
        const nlv3_rx_t *v = &NLV3_RX[i];
        nl_slot_rx_t rx;
        bad = 0;
        nl_slot_rx_init(&rx);
        for (j = 0; j < v->n; j++) {
            const nl_gps_set_t *g = nl_slot_rx_feed(&rx, v->slots[j][0] ? v->slots[j] + 1 : 0);
            const nlv3_set_t *e = &v->out[j];
            if ((g != 0) != (e->valid != 0)) {
                bad++;
                continue;
            }
            if (g && (g->flags != e->flags || g->num_sv != e->num_sv || g->hacc_dm != e->hacc_dm ||
                      g->reset_count != e->reset_count || g->off[0] != e->off[0] || g->off[1] != e->off[1] ||
                      g->p[0] != e->p[0] || g->p[1] != e->p[1] || g->v[0] != e->v[0] || g->v[1] != e->v[1] ||
                      g->cs[0] != e->cs[0] || g->cs[1] != e->cs[1] || g->cycle != e->cycle))
                bad++;
        }
        CHECK(bad == 0 && nl_slot_rx_gate_ok(&rx) == (v->gate_ok_after != 0) && rx.sets == v->sets && rx.broken == v->broken,
              "slot RX %s: sets / fields / gate_ok / counters == the reference (%d ticks differ, sets %u broken %u)", v->name,
              bad, rx.sets, rx.broken);
    }
    {   /* freshness: the gate holds for NL_SLOT_STALE_TICKS ticks after the set, then lapses */
        nl_slot_rx_t rx;
        nl_slot_rx_init(&rx);
        for (j = 0; j < 5; j++)
            (void)nl_slot_rx_feed(&rx, NLV3_SLOTS[j]);
        CHECK(nl_slot_rx_fresh(&rx) && nl_slot_rx_gate_ok(&rx) && rx.got_part4, "complete set: fresh, gate ok");
        for (j = 0; j < (int)NL_SLOT_STALE_TICKS; j++)
            (void)nl_slot_rx_feed(&rx, 0);
        CHECK(nl_slot_rx_fresh(&rx) && !rx.got_part4, "fresh %u ticks after the set", NL_SLOT_STALE_TICKS);
        (void)nl_slot_rx_feed(&rx, 0);
        CHECK(!nl_slot_rx_fresh(&rx) && !nl_slot_rx_gate_ok(&rx), "stale after %u ticks: gate off", NL_SLOT_STALE_TICKS + 1);
    }
    {   /* nl_nav3_set_physical: quantisation and saturation of the log-only fields */
        const float a[3] = {400.0f, -0.125f, 9.81f};
        nl_nav3_set_physical(&n, a, -500.0f, 65535u, 254u, 255u);
        CHECK(n.accel_x_q == 32767 && n.accel_y_q == -13 && n.accel_z == 9.81f && n.h_baro_rel_cm == -32768 &&
              n.imu_fail == 255 && n.baro_fails == 254 && n.twi_errors == 255, "v3 log fields: x 100 saturating, counters <= 255");
    }
}

static void test_vc_takeoff(void)
{
    vc_t c;
    vc_params_t p;
    vc_outputs_t o;
    int t, fail_at = -1, run_from = -1;
    vc_default(&p);
    CHECK(p.takeoff_ticks == VC_TAKEOFF_TICKS && p.takeoff_ticks > 0u && p.takeoff_climb_m > 0.0f,
          "take-off check from params: %u ticks, climb %.2f m", p.takeoff_ticks, p.takeoff_climb_m);
    /* v0.9 gains (vz_kp 0.05): u_ff + vz_kp vz_max_up stays below u_max = 1 - with the conditional integration the v0.8.1 law
     * never pins a pad-bound vehicle at u_max (the reason for the v0.9 thrust search, test_vc_v09); a ceiling below
     * u_ff pins it as the v0.8 test assumes */
    if (p.u_hover + p.vz_kp * p.vz_max_up_mps < p.u_max)
        p.u_max = 0.95f * p.u_hover;
    vc_init(&c, &p);
    /* a vehicle that cannot lift itself: h stays 0, v_z 0 -> pinned at u_max after the ramp */
    for (t = 0; t < 4000 && fail_at < 0; t++) {
        vc_inputs_t in = {LP_PHASE_CLIMB, (int16_t)(10 * 5), false, 0.0f, 0.0f, 1.0f, 9.0f};
        vc_update(&c, &in, &o);
        if (run_from < 0 && c.to_run == 1u) run_from = t;
        if (o.sig & VC_SIG_TAKEOFF_FAIL) fail_at = t;
    }
    /* the run starts once the ramp is over and the loop is pinned at u_max (field: the ramp's last tick - 1; a profile with
     * a slower reference saturates later), and fails takeoff_ticks ticks later */
    CHECK(run_from >= (int)p.ramp_ticks - 1 && fail_at == run_from + (int)p.takeoff_ticks - 1,
          "take-off fail at tick %d: saturated from tick %d (ramp %u), + %u ticks", fail_at, run_from, p.ramp_ticks,
          p.takeoff_ticks);
    {   /* climbs takeoff_climb_m above the h0 latched after the ramp + the settle time: passed, never fails */
        int fails = 0;
        const int n_set = (int)p.ramp_ticks + (int)p.takeoff_settle_ticks;
        CHECK(p.takeoff_settle_ticks == VC_TAKEOFF_SETTLE_TICKS(FC_LOOP_HZ) && p.takeoff_settle_ticks == 125u,
              "take-off settle %u ticks (0.5 s)", p.takeoff_settle_ticks);
        vc_init(&c, &p);
        for (t = 0; t < 3000; t++) {
            vc_inputs_t in = {LP_PHASE_CLIMB, 50, false, t < n_set ? 0.0f : 0.31f, 0.0f, 1.0f, 9.0f};
            vc_update(&c, &in, &o);
            if (o.sig & VC_SIG_TAKEOFF_FAIL) fails++;
            if (t == n_set - 2) CHECK(!c.to_h0_ok, "h0 not latched before ramp + settle");
        }
        CHECK(fails == 0 && c.to_passed && c.to_run == 0 && c.to_h0_ok && c.to_h0 == 0.0f, "climb of 0.31 m: passed, no fail");
    }
    {   /* physics-review fix: a 1 m thrust-proportional baro offset while spooling / settling is inside h0: no pass */
        int fails = 0, passed = 0;
        const int n_set = (int)p.ramp_ticks + (int)p.takeoff_settle_ticks;
        vc_init(&c, &p);
        for (t = 0; t < n_set + 3 * (int)p.takeoff_ticks; t++) {   /* demo: the slower reference saturates later */
            const float h = t < n_set ? (float)t / (float)n_set : 1.0f;
            vc_inputs_t in = {LP_PHASE_CLIMB, 50, false, h, 0.0f, 1.0f, 9.0f};
            vc_update(&c, &in, &o);
            if (o.sig & VC_SIG_TAKEOFF_FAIL) fails++;
            if (c.to_passed) passed++;
        }
        CHECK(fails > 0 && passed == 0, "baro offset 0 -> 1 m on the pad during ramp + settle: no pass, take-off fail");
    }
    {   /* off (0 ticks): never; IDLE resets the run */
        int fails = 0;
        vc_params_t p0 = p;
        p0.takeoff_ticks = 0;
        vc_init(&c, &p0);
        for (t = 0; t < 2000; t++) {
            vc_inputs_t in = {LP_PHASE_CLIMB, 50, false, 0.0f, 0.0f, 1.0f, 9.0f};
            vc_update(&c, &in, &o);
            if (o.sig & VC_SIG_TAKEOFF_FAIL) fails++;
        }
        CHECK(fails == 0 && c.to_run == 0, "takeoff_ticks 0: the v0.7.1 controller (no check)");
        vc_init(&c, &p);
        for (t = 0; t < 500; t++) {
            vc_inputs_t in = {LP_PHASE_CLIMB, 50, false, 0.0f, 0.0f, 1.0f, 9.0f};
            vc_update(&c, &in, &o);
        }
        CHECK(c.to_run > 0u, "run counting");
        {
            vc_inputs_t in = {LP_PHASE_NONE, 0, false, 0.0f, 0.0f, 1.0f, 9.0f};
            vc_update(&c, &in, &o);
        }
        CHECK(c.to_run == 0 && !c.to_passed && o.sig == 0, "IDLE resets the take-off check");
    }
}

static void test_att_tilt(void)
{
    static attitude_ctrl_t a, b;
    att_gains_t g;
    mixer_plant_t pl;
    ctrl_output_t oa, ob;
    int k, bad = 0;
    double worst = 0.0;
    fc_vtol_gains(&g);
    fc_default_plant(&pl);
    attitude_ctrl_init(&a, &g, &pl, fc_throttle_exp_int());
    attitude_ctrl_init(&b, &g, &pl, fc_throttle_exp_int());
    {
        const float zero[2] = {0.0f, 0.0f};
        attitude_ctrl_set_tilt(&b, zero);
    }
    for (k = 0; k < 2000; k++) {                       /* tilt (0, 0): the v0.7.1 controller bit for bit */
        float q[4] = {1.0f, (float)(0.05 * srand1()), (float)(0.05 * srand1()), (float)(0.1 * srand1())}, qn[4];
        const float w[3] = {(float)(0.5 * srand1()), (float)(0.5 * srand1()), (float)(0.3 * srand1())};
        quat_normalize(q, qn);
        attitude_ctrl_update(&a, qn, w, 0.9f, &oa);
        attitude_ctrl_update(&b, qn, w, 0.9f, &ob);
        if (memcmp(oa.delta_cmd, ob.delta_cmd, sizeof oa.delta_cmd) || memcmp(oa.att_error, ob.att_error, sizeof oa.att_error))
            bad++;
    }
    CHECK(bad == 0, "tilt setpoint (0, 0): outputs bit-identical to the v0.7.1 controller (%d differ)", bad);
    for (k = 0; k < 20000; k++) {                      /* q(t) (x) q_yaw by series vs the exact quaternion product */
        const double th = 0.35 * urand(), az = IL_PI_D * srand1(), psi = IL_PI_D * srand1();
        const float t[2] = {(float)(th * cos(az)), (float)(th * sin(az))};
        const float qy[4] = {(float)cos(psi / 2), 0.0f, 0.0f, (float)sin(psi / 2)};
        float qs[4];
        const double tx = t[0], ty = t[1], tn = sqrt(tx * tx + ty * ty);
        const double cw = cos(tn / 2), sk = tn > 0 ? sin(tn / 2) / tn : 0.5;
        const double e[4] = {cw * qy[0], sk * tx * qy[0] + sk * ty * qy[3], sk * ty * qy[0] - sk * tx * qy[3], cw * qy[3]};
        int i;
        attitude_ctrl_tilt_quat(t, qy, qs);
        for (i = 0; i < 4; i++) worst = fmax(worst, fabs(qs[i] - e[i]));
    }
    CHECK(worst < 4e-7, "tilted setpoint series vs the exact q(t) (x) q_yaw up to 20 deg: %.2g", worst);
    {   /* v0.9 (v0.8.2 D port B): set_tilt builds q(t) in one piece (no finish half); update() folds the yaw product into
         * the error - the outputs do not depend on whether the (no-op) finish ran */
        const float q0[4] = {0.96592583f, 0.0f, 0.0f, 0.25881905f}, w[3] = {0.1f, -0.2f, 0.05f};
        const float one[4] = {1.0f, 0.0f, 0.0f, 0.0f};
        int halves = 0, wrong = 0;
        attitude_ctrl_init(&a, &g, &pl, fc_throttle_exp_int());
        attitude_ctrl_init(&b, &g, &pl, fc_throttle_exp_int());
        attitude_ctrl_set_yaw_reference(&a, q0);
        attitude_ctrl_set_yaw_reference(&b, q0);
        for (k = 0; k < 3000; k++) {
            const float t[2] = {(float)(0.3 * srand1()), (float)(0.3 * srand1())};
            float qs[4];
            const bool half = attitude_ctrl_set_tilt(&a, t);
            halves += half ? 1 : 0;
            if (k % 3 == 0)
                attitude_ctrl_set_tilt_finish(&a);     /* else update() or the next set_tilt finishes it */
            attitude_ctrl_tilt_quat(t, one, qs);       /* q(t) (x) 1 = (w, k tx, k ty, 0) */
            (void)attitude_ctrl_set_tilt(&b, t);
            attitude_ctrl_set_tilt_finish(&b);
            attitude_ctrl_update(&a, q0, w, 0.9f, &oa);
            attitude_ctrl_update(&b, q0, w, 0.9f, &ob);
            if (memcmp(a.q_t, qs, sizeof a.q_t) || qs[3] != 0.0f || memcmp(oa.delta_cmd, ob.delta_cmd, sizeof oa.delta_cmd))
                wrong++;
        }
        CHECK(wrong == 0 && halves == 0, "port B: set_tilt builds q(t) at once (%d halves); the finish is a no-op (%d wrong)",
              halves, wrong);
        CHECK(!attitude_ctrl_set_tilt(&b, b.tilt_sp), "the same tilt again: nothing to do");
    }
    {   /* a tilted setpoint: the attitude error points the thrust axis toward it */
        const float t[2] = {0.1f, 0.0f};
        const float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}, w[3] = {0.0f, 0.0f, 0.0f};
        attitude_ctrl_init(&a, &g, &pl, fc_throttle_exp_int());
        attitude_ctrl_set_yaw_reference(&a, q);
        attitude_ctrl_set_tilt(&a, t);
        attitude_ctrl_update(&a, q, w, 0.9f, &oa);
        /* v0.9: the rotation vector (att_err_mode 0) 0.1; the reduced error (1, 2) sin(0.1) */
        CHECK(a.tilt_on && NEAR(oa.att_error[0], g.v9.err_mode ? sin(0.1) : 0.1, 2e-6) && NEAR(oa.att_error[1], 0.0, 1e-7) &&
              NEAR(oa.att_error[2], 0.0, 1e-7),
              "tilt (0.1, 0) rad about level x at yaw 0: attitude error (%.7f, %.7f, %.7f)", oa.att_error[0], oa.att_error[1],
              oa.att_error[2]);
    }
}

static void test_predict(void)
{
    static att_mahony_t m;
    int k, bad = 0, bad_pad = 0;
    fc_default_mahony(&m);
    for (k = 0; k < 3000; k++) {                       /* gate closed (|f| far from g): predict == the correction's q */
        const bool pad = k < 1500;
        const float gy[3] = {(float)(0.01 + 0.2 * srand1()), (float)(0.2 * srand1()), (float)(0.1 * srand1())};
        const float ac[3] = {(float)(0.2 * srand1()), (float)(0.2 * srand1()), pad ? 9.80665f : 14.0f};
        float qp[4], rp[3], r2[3];
        att_mahony_predict(&m, gy, pad, qp, rp);
        att_mahony_rate(&m, gy, r2);
        att_mahony_update(&m, gy, ac, pad);
        if (memcmp(rp, r2, sizeof rp)) bad++;
        if (!pad && !m.gate && memcmp(qp, m.pad.q, sizeof qp)) bad++;
        if (pad && k > 0 && fabsf(qp[0] - m.pad.q[0]) > 1e-3f) bad_pad++;
    }
    CHECK(bad == 0 && bad_pad == 0, "NAV prediction: rate = g - b of the previous correction; in flight with the gate closed "
          "q_pub == the corrected q bit for bit (%d / %d)", bad, bad_pad);
}

/* ---- the NAV GPS filter: behaviour checks on synthetic ticks (the cross-check compares it with the reference) ---- */
static void gps_test_epoch(gps_epoch_t *e, const gps_frame_t *fr, double x, double y, double vx, double vy, uint32_t itow,
                           uint8_t sv, uint32_t hacc)
{
    memset(e, 0, sizeof *e);
    e->itow = itow;
    e->lat = fr->lat0 + (int32_t)floor(x / fr->k_n + 0.5);
    e->lon = fr->lon0 + (int32_t)floor(-y / fr->k_e + 0.5);
    e->hmsl = 41000;
    e->hacc = hacc;
    e->vacc = 3900;
    e->vel_n = (int32_t)floor(vx * 100.0 + 0.5);
    e->vel_e = (int32_t)floor(-vy * 100.0 + 0.5);
    e->sacc = 30;
    e->gps_fix = 3;
    e->fix_ok = true;
    e->num_sv = sv;
    e->pacc = 350;
}

static void test_gps_filter(void)
{
    static gps_filter_t f;
    gps_nav_params_t gp;
    gps_frame_t fr;
    gps_itow_t clk;
    const float q0[4] = {1.0f, 0.0f, 0.0f, 0.0f}, w0[3] = {0.0f, 0.0f, 0.0f}, a0[3] = {0.0f, 0.0f, 9.80665f};
    uint32_t k, init_k = 0, gate_k = 0;
    double vx = 0.0, x = 0.0;
    fc_default_gps_nav(&gp);
    gp.lever_b[0] = gp.lever_b[1] = gp.lever_b[2] = 0.0f;      /* no lever arm in this synthetic stream */
    gps_filter_init(&f, &gp);
    gps_frame_at(&fr, 375665432L, 1269780123L);
    gps_itow_init(&clk, gp.ms_per_tick, gp.latency_ms);
    CHECK(f.snap.flags == 0 && f.snap.hacc_dm == 255 && f.snap.cos_q15 == 32767 && f.pad, "filter initial snapshot");
    /* 10 s on the pad at (3, -2) m, then 10 s of flight at 0.5 m/s north (the IMU sees no acceleration) */
    for (k = 0; k < 5000u; k++) {
        const bool pad = k < 2500u;
        bool st;
        if (!pad) {
            vx = 0.5;
            x += vx * (double)FC_DT_S;
        }
        if (k % 50u == 30u) {                          /* an epoch measured 55 ticks (220 ms) before it completes */
            gps_epoch_t e;
            const uint32_t itow = 345600000UL + 4u * (k - 55u);
            gps_test_epoch(&e, &fr, 3.0 + x - vx * 0.22, -2.0, vx, 0.0, itow, 9, 2500);
            e.tick = k;
            gps_filter_epoch(&f, &e, gps_itow_update(&clk, itow, k));
        }
        st = gps_filter_tick(&f, k, a0, q0, w0, pad) == GPS_TICK_STEP;
        if (st != (k % 5u == 4u)) init_k = 0xFFFFFFFFUL;
        if (f.init && init_k == 0) init_k = k;
        if ((f.snap.flags & GPS_F_GATE_OK) && gate_k == 0) gate_k = k;
        if (k == 2499u) {
            CHECK(NEAR(f.p_[0], 0.0, 0.05) && NEAR(f.p_[1], 0.0, 0.05) && f.v[0] == 0.0f && f.v[1] == 0.0f &&
                  f.pad && !(f.snap.flags & GPS_F_INCONS) && (f.snap.flags & GPS_F_FIX_OK) && (f.snap.flags & GPS_F_INIT),
                  "pad: p = (%.3f, %.3f) in the frame of the first fix, v = 0, flags 0x%02X", f.p_[0], f.p_[1], f.snap.flags);
        }
    }
    CHECK(init_k == 31u, "first usable epoch (completed at tick 30, j = 0) initialises the filter in E0 at j = 1 (k %lu)",
          (unsigned long)init_k);
    CHECK(gate_k > 0 && gate_k <= 1500u, "ON gate (GATE_OK) %lu ticks after the start (stable %lu ticks)", (unsigned long)gate_k,
          (unsigned long)gp.gate_stable_ticks);
    CHECK(NEAR(f.v[0], 0.5, 0.02) && NEAR(f.p_[0], x, 0.5) && !f.pad && !(f.snap.flags & GPS_F_INCONS) && f.st_rej_v == 0,
          "flight: v %.3f m/s (truth 0.5), p %.2f m (truth %.2f) through the 0.22 s delayed state", f.v[0], f.p_[0], x);
    CHECK(f.snap.vx_cms == (int16_t)lp_quantize(f.v[0], 100, LP_I16_MIN, LP_I16_MAX) && f.snap.num_sv == 9 &&
          f.snap.hacc_dm == 25 && f.st_dropped_old == 0 && f.st_dropped_stale == 1,
          "snapshot quantisation, satellites, hAcc 2.5 m -> 25 dm; one epoch dropped (its delayed state predates the "
          "initialisation: %u stale, %u old)", f.st_dropped_stale, f.st_dropped_old);
    {   /* a 6 m position jump with a consistent velocity: rejected, then a position reset into the offset after 1.5 s */
        uint32_t k2;
        const uint16_t resets0 = f.st_pos_resets;
        for (k2 = k; k2 < k + 1500u; k2++) {
            x += vx * (double)FC_DT_S;
            if (k2 % 50u == 30u) {
                gps_epoch_t e;
                const uint32_t itow = 345600000UL + 4u * (k2 - 55u);
                gps_test_epoch(&e, &fr, 3.0 + x - vx * 0.22 + 6.0, -2.0, vx, 0.0, itow, 9, 2500);
                e.tick = k2;
                gps_filter_epoch(&f, &e, gps_itow_update(&clk, itow, k2));
            }
            (void)gps_filter_tick(&f, k2, a0, q0, w0, false);
        }
        CHECK(f.st_pos_resets == resets0 + 1u && NEAR(f.off[0], 6.0, 0.3) && f.reset_count == 1u &&
              NEAR(f.p_[0] - f.off[0], x, 0.4), "GPS jump of 6 m: one position reset, offset %.2f m, reset-free p - off "
              "%.2f (truth %.2f)", f.off[0], f.p_[0] - f.off[0], x);
    }
}

/* ---- the position controller ---- */
static void pos_rand_inputs(pos_inputs_t *in, nl_gps_set_t *d, uint32_t t)
{
    const int ph = (t / 400u) % 7u;
    in->phase = ph == 0 ? LP_PHASE_SPOOL : ph < 3 ? LP_PHASE_CLIMB : ph < 5 ? LP_PHASE_HOLD : ph == 5 ? LP_PHASE_LAND :
                LP_PHASE_LANDED;
    in->motor_on = in->phase != LP_PHASE_SPOOL && in->phase != LP_PHASE_LANDED;
    in->h_m = (float)(t % 400u < 30u ? 0.2 * urand() : 0.1 + 3.0 * urand());
    in->cos_tilt = (float)(1.0 - 0.06 * urand() * urand());
    in->rate[0] = (float)(0.4 * srand1());
    in->rate[1] = (float)(0.4 * srand1());
    in->rate[2] = (float)(urand() < 0.02 ? 4.0 * srand1() : 0.3 * srand1());
    in->k_t = (float)(0.85 + 0.3 * urand());
    in->vc_sat_high = urand() < 0.05;
    in->vc_low_alt = in->phase == LP_PHASE_LAND && in->h_m <= 1.0f;
    in->vc_ground = in->vc_low_alt && urand() < 0.1;
    in->lifted = urand() < 0.97;
    in->data = 0;
    in->set = 0;
    if (urand() < 0.85) {
        d->flags = (uint8_t)((urand() < 0.9 ? 0x2F : 0x2E) | ((t / 300u) % 3u == 1u ? POS_GF_INCONS : 0u));
        d->num_sv = 9;
        d->hacc_dm = 25;
        d->reset_count = (uint8_t)(t / 2000u + (urand() < 0.01 ? 1u : 0u));
        d->off[0] = (int16_t)(t / 2000u * 30u);
        d->off[1] = 0;
        d->p[0] = (int16_t)(300.0 * srand1());
        d->p[1] = (int16_t)(300.0 * srand1());
        d->v[0] = (int16_t)(150.0 * srand1());
        d->v[1] = (int16_t)(150.0 * srand1());
        d->cs[0] = 32610;
        d->cs[1] = -3212;
        d->cycle = (uint8_t)(t & 31u);
        in->set = d;
    }
}

static double g0sq(const pos_params_t *pp)
{
    return (double)pp->g * pp->g;
}

static bool pub_eq(const pos_pub_t *a, const pos_pub_t *b)
{
    return a->mode == b->mode && a->requests == b->requests && a->engaged == b->engaged &&
           memcmp(a->tilt_l, b->tilt_l, sizeof a->tilt_l) == 0 && memcmp(&a->dist_m, &b->dist_m, sizeof a->dist_m) == 0;
}

static void test_position(void)
{
    static pos_t a, b;
    pos_params_t pp;
    nl_gps_set_t d;
    pos_inputs_t in;
    uint32_t t;
    int bad = 0, steps = 0, limited = 0, engaged = 0, j, commits = 0, order_bad = 0;
    fc_default_position(&pp);
    CHECK(pp.step_ticks == 5u && pp.T == 0.02f && pp.plaus_resets <= POS_RESETS_MAX, "position params: 50 Hz, 5 FCU ticks");
    {   /* the derived fields: fc_default_position (constant-folded) == pos_params_derive (run time) */
        pos_params_t q = pp;
        pos_params_derive(&q);
        CHECK(memcmp(&q, &pp, sizeof q) == 0 && fc_position_params()->inv_g2 == pp.inv_g2,
              "position params: derived fields constant-folded == pos_params_derive");
    }
    pos_init(&a, &pp);
    pos_init(&b, &pp);
    /* the background pieces give the numbers of the step at once (a: pos_step; b: pos_latch, then the pieces of tick
     * offsets 0..4, the latched set overwritten between the ticks) */
    for (t = 0; t < 30000u; t++) {
        pos_pub_t dst;
        pos_rand_inputs(&in, &d, t);
        (void)pos_step(&a, &in);
        pos_latch(&b, &in);
        for (j = 0; j <= (int)POS_PIPELINE_TICKS; j++) {
            const nl_gps_set_t keep = d;
            int last_piece = -1;
            while (pos_piece_due(&b, (uint8_t)j)) {
                const uint8_t pc = (uint8_t)(b.job - 1u);
                if (pos_piece_tick(pc) != (uint8_t)j || (int)pc <= last_piece) order_bad++;
                last_piece = pc;
                if (pos_piece(&b, &dst)) {
                    commits++;
                    if (j != (int)POS_PIPELINE_TICKS || !pub_eq(&dst, &b.last)) order_bad++;
                }
            }
            memset(&d, 0x5A, sizeof d);                /* the slot receiver may overwrite its set: the latch copied it */
            d = keep;
        }
        steps++;
        if (memcmp(&a.out, &b.out, sizeof a.out) || !pub_eq(&a.last, &b.last) || memcmp(a.integ, b.integ, sizeof a.integ) ||
            a.lost != b.lost || a.requests != b.requests || a.implausible != b.implausible || memcmp(a.dv, b.dv, sizeof a.dv))
            bad++;
        if (a.out.mode != a.last.mode || a.out.requests != a.last.requests || a.out.engaged != a.last.engaged ||
            memcmp(a.out.tilt_l, a.last.tilt_l, sizeof a.out.tilt_l) || memcmp(&a.out.dist_m, &a.last.dist_m, 4))
            bad++;
        if (a.out.limited) limited++;
        if (a.out.engaged) engaged++;
    }
    CHECK(bad == 0 && b.job == 0u && commits == steps && order_bad == 0, "background pieces P0..P10 on tick offsets 0..4 == "
          "the step at once on %d random steps (%d differ; %d limited, %d engaged; the commit at offset 4 every step)", steps,
          bad, limited, engaged);
    {   /* the tilt limit from the thrust margin: closed form in double */
        int k, wrong = 0;
        for (k = 0; k < 2000; k++) {
            const float kt = (float)(0.8 + 0.5 * urand());
            const bool low = urand() < 0.3;
            float c, s, a2;
            const double x = (double)fmaxf(kt, 1e-3f) * pp.tw, m = fmin(pp.margin_g, pp.margin_frac * fmax(x - 1.0, 0.0));
            const double r = (1.0 + m) / x, cc = low ? pp.cos_flare_cap : pp.cos_cap;
            const double ce = r >= 1.0 ? 1.0 : fmax(r, cc), ae = r >= 1.0 ? 0.0 : pp.g * pp.g * (1.0 - ce * ce) / (ce * ce);
            pos_tilt_limit(&a, kt, low, &c, &s, &a2);
            /* s and a_lim^2 are ill-conditioned as cos_lim -> 1 (ds = c dc / s): compared through s^2 = 1 - c^2 */
            if (!NEAR(c, ce, 2e-7) || !NEAR((double)s * s, 1.0 - ce * ce, 1e-6) || !NEAR(a2, ae, 2e-5 * (1.0 + ae) + 2e-4 * g0sq(&pp)))
                wrong++;
        }
        CHECK(wrong == 0, "tilt limit (cos, sin, a_lim^2) vs double on 2000 (k_T, flare) pairs (%d wrong)", wrong);
    }
    {   /* the tilt series: atan(|a| / g) within 0.4 % up to 20 deg, direction (-a_y, a_x) */
        double worst = 0.0;
        int k;
        for (k = 0; k < 2000; k++) {
            const double th = 0.349 * urand(), az = IL_PI_D * srand1();
            const float ax = (float)(pp.g * tan(th) * cos(az)), ay = (float)(pp.g * tan(th) * sin(az));
            float tw[2];
            pos_tilt_of_accel(ax, ay, pp.inv_g, pp.inv_g2, tw);
            if (th > 1e-3) {
                const double mag = sqrt((double)tw[0] * tw[0] + (double)tw[1] * tw[1]);
                worst = fmax(worst, fabs(mag - th) / th);
                worst = fmax(worst, fabs(tw[0] * ax + tw[1] * ay) / (mag * sqrt((double)ax * ax + (double)ay * ay)));
            }
        }
        CHECK(worst < 4.5e-3, "tilt series vs atan: %.2g relative (< 0.4 %% by the docstring)", worst);
    }
    {   /* OFF when not flying, engage only above h_enable, requests */
        pos_init(&a, &pp);
        in.phase = LP_PHASE_HOLD; in.motor_on = true; in.h_m = 0.2f; in.cos_tilt = 1.0f;
        in.rate[0] = in.rate[1] = in.rate[2] = 0.0f; in.k_t = 1.0f; in.vc_sat_high = false;
        in.vc_low_alt = in.vc_ground = false; in.lifted = true;
        d.flags = 0x2F; d.reset_count = 0; d.off[0] = d.off[1] = 0; d.p[0] = 100; d.p[1] = -200; d.v[0] = d.v[1] = 0;
        d.cs[0] = 32767; d.cs[1] = 0;
        in.data = 0;
        in.set = &d;
        (void)pos_step(&a, &in);
        CHECK(!a.engaged && a.out.mode == POS_MODE_OFF && a.out.tilt_l[0] == 0.0f, "below h_enable: not engaged, tilt 0");
        in.h_m = 1.5f;
        (void)pos_step(&a, &in);
        CHECK(a.engaged && a.has_hold && a.out.mode == POS_MODE_POS && NEAR(a.out.hold[0], 1.0, 1e-6) &&
              NEAR(a.out.hold[1], -2.0, 1e-6) && NEAR(a.out.dist_m, 0.0, 1e-6), "HOLD: the hold point captured at (1, -2)");
        d.p[0] = 100 + 1600;                           /* 16 m north of the hold point: beyond the field fence (15 m), */
        d.v[0] = 150;                                  /* moving away at 1.5 m/s */
        (void)pos_step(&a, &in);
        CHECK((a.out.requests & POS_REQ_FENCE) && a.out.limited, "16 m from the hold point: FENCE request, a limited");
        in.set = 0;
        for (j = 0; j < (int)pp.land_steps; j++)
            (void)pos_step(&a, &in);
        CHECK((a.out.requests & POS_REQ_GPS_LOST) && a.out.lost_steps == pp.land_steps, "no data for land_steps %u: GPS_LOST",
              pp.land_steps);
        in.k_t = 0.5f;
        if (!(in.k_t * pp.tw < pp.authority_min_ktw))       /* fw09 WP-X: a light VTOL configuration (T/W up to 2.2) */
            in.k_t = 0.5f * pp.authority_min_ktw / pp.tw;    /* needs a smaller k_T for the same k_T T/W case */
        (void)pos_step(&a, &in);
        CHECK(a.out.requests & POS_REQ_AUTHORITY, "k_T T/W < %.2f: AUTHORITY (k_T %.3f, T/W %.3f)", pp.authority_min_ktw,
              (double)in.k_t, (double)pp.tw);
        in.motor_on = false;
        (void)pos_step(&a, &in);
        CHECK(!a.engaged && a.out.requests == 0 && a.out.mode == POS_MODE_OFF && a.requests == 0, "motor off: everything reset");
    }
    {   /* the tilt builder: slews at slew_per_tick, stops at the target, 0 when inactive */
        float tt[2] = {0.0f, 0.0f};
        const float tg[2] = {0.2f, -0.1f};
        int n;
        for (n = 0; n < 1000 && !(fabsf(tt[0] - tg[0]) < 1e-7f && fabsf(tt[1] - tg[1]) < 1e-7f); n++)
            pos_tilt_tick(tt, tg, true, pp.slew_per_tick);
        CHECK(n == (int)ceil(sqrt(0.05) / pp.slew_per_tick - 1e-6), "tilt builder: %d ticks to 0.224 rad at %.4f rad/tick", n,
              pp.slew_per_tick);
        pos_tilt_tick(tt, tg, false, pp.slew_per_tick);
        CHECK(tt[0] == 0.0f && tt[1] == 0.0f, "builder inactive: 0");
    }
}

/* ---- the v0.8 supervisor rules (REJ_GPS, V11-V14, V15, TELEM v3) ---- */
static void g8_init(oh_t *h)
{
    onb_params_t p;
    onb_params_mode(&p, LP_MODE_VTOL);
    p.gps_hold = true;
    p.takeoff_check = true;
    oh_init_p(h, &p, 1, false);
    h->tg.hacc_dm = 255;
    h->tg.dist_m = NAN;
}

static void test_onboard_v08(void)
{
    static oh_t h;
    int i;
    CHECK(LP_R_V_GPS_LOST == 24 && LP_R_V_FENCE == 25 && LP_R_V_GPS_IMPLAUSIBLE == 26 && LP_R_V_AUTHORITY == 27 &&
          LP_R_V_TAKEOFF_CHECK == 28 && LP_REJ_GPS == 0x50, "v0.8 reason codes 24..28, REJ_GPS 0x50 (sim/link_proto.py)");
    /* the ON gate: after "aligned", before the tilt / V_cell / link gates */
    g8_init(&h);
    vh_to(&h, "SAFE");
    h.aligned = false;
    CHECK(oh_send(&h, LP_CMD_ON, 0)->state == ONB_SAFE && h.last_code == LP_REJ_NOT_ALIGNED, "ON, not aligned: REJ_NOT_ALIGNED");
    h.aligned = true;
    h.tilt = 20.0f;
    CHECK(oh_send(&h, LP_CMD_ON, 0)->state == ONB_SAFE && h.last_code == LP_REJ_GPS, "ON, GPS not ready (and tilted): REJ_GPS "
          "first (%d)", h.last_code);
    h.gate = true;
    CHECK(oh_send(&h, LP_CMD_ON, 0)->state == ONB_SAFE && h.last_code == LP_REJ_TILT, "ON, GPS ready, tilted: REJ_TILT");
    h.tilt = 0.5f;
    CHECK(oh_send(&h, LP_CMD_ON, 0)->state == ONB_ARMED, "ON with the GPS ready: ARMED");
    {   /* the LAND requests: CLIMB / HOLD only, in the order GPS lost, fence, implausible, authority */
        static const uint8_t bits[4] = {POS_REQ_GPS_LOST, POS_REQ_FENCE, POS_REQ_IMPLAUSIBLE, POS_REQ_AUTHORITY};
        static const uint8_t codes[4] = {LP_R_V_GPS_LOST, LP_R_V_FENCE, LP_R_V_GPS_IMPLAUSIBLE, LP_R_V_AUTHORITY};
        for (i = 0; i < 4; i++) {
            g8_init(&h);
            h.gate = true;
            vh_to(&h, i % 2 ? "HOLD" : "CLIMB");
            h.pcr = (uint8_t)(bits[i] | (i < 3 ? bits[3] : 0u) | (i == 0 ? POS_REQ_FENCE : 0u));
            oh_hb(&h);
            CHECK(h.s.state == ONB_POWERED && h.s.phase == LP_PHASE_LAND && h.o.reason == codes[i] &&
                  h.o.interlock == codes[i] && !h.o.release, "request 0x%02X in %s: LAND with reason %u (%u)", h.pcr,
                  i % 2 ? "HOLD" : "CLIMB", codes[i], h.o.reason);
            oh_hb(&h);
            CHECK(h.s.phase == LP_PHASE_LAND && h.o.interlock == 0, "requests do nothing more in LAND");
        }
    }
    {   /* V15: the take-off check -> SAFE, motor off, no release; not in LAND; off without takeoff_check */
        g8_init(&h);
        h.gate = true;
        vh_to(&h, "CLIMB");
        h.tof = true;
        oh_hb(&h);
        CHECK(h.s.state == ONB_SAFE && h.o.reason == LP_R_V_TAKEOFF_CHECK && !h.o.release && !h.o.motor_on &&
              !h.s.has_release_deadline, "V15 in CLIMB: SAFE (reason 28), no release");
        g8_init(&h);
        h.gate = true;
        vh_to(&h, "LAND");
        h.tof = true;
        oh_hb(&h);
        CHECK(h.s.state == ONB_POWERED && h.s.phase == LP_PHASE_LAND, "V15 is not evaluated in LAND");
        g8_init(&h);
        h.p.takeoff_check = false;
        h.s.p.takeoff_check = false;
        h.gate = true;
        vh_to(&h, "CLIMB");
        h.tof = true;
        oh_hb(&h);
        CHECK(h.s.state == ONB_POWERED, "takeoff_check off: V15 inactive");
    }
    {   /* TELEM v3: the reply carries the GPS fields; dist NaN -> 0xFFFF */
        lp_telem_t t;
        uint8_t fr[LP_FRAME_MAX_BYTES], n;
        g8_init(&h);
        h.tg.flags = 0x2F; h.tg.num_sv = 9; h.tg.hacc_dm = 25; h.tg.pos_mode = POS_MODE_POS; h.tg.dist_m = 0.734f;
        oh_hb(&h);
        CHECK(h.o.reply && h.o.tlm.v2 && h.o.tlm.v3, "a reply in the GPS build is a v3 source");
        onboard_telemetry(&h.o.tlm, FC_LOOP_HZ, &t);
        CHECK(t.gps_flags == 0x2F && t.num_sv == 9 && t.hacc_dm == 25 && t.pos_mode == POS_MODE_POS && t.dist_dm == 7,
              "TELEM v3 fields (dist 0.734 m -> 7 dm)");
        n = onboard_telemetry_frame(&h.o.tlm, FC_LOOP_HZ, LINK_VEHICLE_ID, 5, fr);
        CHECK(n == LP_TELEM3_FRAME_BYTES && fr[4] == LP_TYPE_TELEM3, "TELEM3 frame (45 bytes, TYPE 0x22)");
        h.tg.dist_m = NAN;
        oh_hb(&h);
        onboard_telemetry(&h.o.tlm, FC_LOOP_HZ, &t);
        CHECK(t.dist_dm == LP_DIST_NONE, "no hold point: dist 0xFFFF");
        h.tg.dist_m = 1e6f;
        oh_hb(&h);
        onboard_telemetry(&h.o.tlm, FC_LOOP_HZ, &t);
        CHECK(t.dist_dm == LP_DIST_NONE - 1u, "dist saturates at 0xFFFE");
    }
    {   /* the ballistic build is untouched: gps_hold set in its parameters changes nothing */
        oh_t hb;
        onb_params_t p;
        onb_params_mode(&p, LP_MODE_BALLISTIC);
        p.gps_hold = true;
        oh_init_p(&hb, &p, 1, false);
        oh_hb(&hb);
        oh_tick(&hb);
        CHECK(oh_send(&hb, LP_CMD_ARM, 0)->state == ONB_ARMED && !hb.o.tlm.v3, "ballistic ARM ignores the GPS gate, TELEM v1");
    }
}

/* ---- v0.8 closed loop in C: NAV GPS filter -> nlink v3 slots -> FCU stack (supervisor, vertical controller, position
 * hold, tilt builder) on a 3-D point mass whose attitude follows the tilt setpoint with a first-order lag (a stand-in
 * for the closed attitude loop, as in the reference agent's harness; NOT the 6-DOF).  A sanity test of the C pieces
 * together: take-off, hold, a push, OFF, landing at the hold point, every v0.8 output in range. ---- */
typedef struct {
    double p[3], v[3], tilt[2], T;
    bool ground;
} pm3_t;

static void pm3_quat(const pm3_t *s, float q[4])
{
    const double tx = s->tilt[0], ty = s->tilt[1], th = sqrt(tx * tx + ty * ty);
    const double k = th > 1e-12 ? sin(th / 2.0) / th : 0.5;
    q[0] = (float)cos(th / 2.0);
    q[1] = (float)(k * tx);
    q[2] = (float)(k * ty);
    q[3] = 0.0f;
}

/* R(q) v for the plant (double) */
static void pm3_rot(const float qf[4], const double v[3], double o[3], bool transpose)
{
    const double w = qf[0], x = transpose ? -qf[1] : qf[1], y = transpose ? -qf[2] : qf[2], z = transpose ? -qf[3] : qf[3];
    const double r[3][3] = {{1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
                            {2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
                            {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)}};
    int i;
    for (i = 0; i < 3; i++)
        o[i] = r[i][0] * v[0] + r[i][1] * v[1] + r[i][2] * v[2];
}

static void test_stack_v08(void)
{
    static onb_stack_t k;
    static gps_filter_t F;
    onb_params_t p;
    vc_params_t vp;
    pos_params_t pp;
    gps_nav_params_t gp;
    gps_frame_t fr;
    gps_itow_t clk;
    nl_slot_tx_t tx;
    onb_outputs_t so;
    onb_frame_t f;
    onb_inputs_t in;
    pm3_t s;
    gps_epoch_t queue[4];
    uint32_t qt[4];
    int nq = 0;
    uint32_t t;
    uint8_t seq = 10, nav_state = LP_STATE_SAFE;
    const double m = VEH_MASS_KG, g0 = 9.80665, dt = FC_DT_S, a_att = 1.0 - exp(-dt / 0.15), a_T = 1.0 - exp(-dt / EDF_TAU_S);
    const double tau_d = 9.55;
    int t_on = 1750, t_hold = -1, t_push = -1, t_off = -1, t_landed = -1, t_safe = -1, max_mode_bad = 0, n_rej = 0;
    double hold[2] = {0.0, 0.0}, max_d_hold = 0.0, max_d_push = 0.0, d_back = -1.0, land_d = -1.0, max_tilt = 0.0;
    uint32_t lrng = 777u;
    fc_default_onboard(&p);
    p.mode = LP_MODE_VTOL;
    p.gps_hold = true;
    p.takeoff_check = true;
    fc_default_vertical(&vp);
    fc_default_position(&pp);
    fc_default_gps_nav(&gp);
    onb_stack_init(&k, &p, &vp, &pp, ONB_CHAIN_FCU, 7, false);
    gps_filter_init(&F, &gp);
    F.cfg_ok = true;
    gps_frame_at(&fr, 375665432L, 1269780123L);
    gps_itow_init(&clk, gp.ms_per_tick, gp.latency_ms);
    nl_slot_tx_init(&tx);
    memset(&s, 0, sizeof s);
    s.ground = true;
    memset(&in, 0, sizeof in);
    CHECK(k.gps, "the VTOL stack with gps_hold and position parameters runs the position hold");
    for (t = 0; t < 30000u && t_safe < 0; t++) {
        float q[4], ct, td, att_cmd[2], u;
        uint8_t slot[NL_SLOT_LEN], cmd = 0xFF;
        double nb[3], acc[3], fw[3], fb[3];
        const double ez[3] = {0.0, 0.0, 1.0};
        int i;
        if ((int)t == t_on) cmd = LP_CMD_ON;
        if (t_hold > 0 && (int)t == t_hold + 2500) cmd = LP_CMD_OFF;
        if (cmd == LP_CMD_OFF) t_off = (int)t;
        pm3_quat(&s, q);
        nl_nav3_tilt(q, &ct, &td);
        nl_slot_tx_slot(&tx, t, slot);
        in.tick = t;
        in.n_frames = 0;
        if (cmd != 0xFF || t % 50u == 0u) {
            if (cmd != 0xFF) seq++;
            f.seq = seq;
            f.cmd.cmd = cmd == 0xFF ? LP_CMD_NONE : cmd;
            f.cmd.arg = 0;
            f.cmd.nonce = k.sup.nonce;
            in.frames = &f;
            in.n_frames = 1;
        }
        in.h_m = (float)s.p[2];
        in.vz_mps = (float)s.v[2];
        in.tilt_deg = td;
        in.gyro[0] = in.gyro[1] = in.gyro[2] = 0.0f;
        in.v_cell = 4.0f;
        in.imu_ok = in.baro_ok = in.aligned = in.arm_switch = true;
        in.cos_tilt = ct;
        in.accel_z_mps2 = (float)(s.ground ? g0 : s.T / m);
        in.gps_slot = slot;
        att_cmd[0] = onb_stack_att_tilt(&k)[0];       /* (1) this tick's attitude controller: the previous tick's tilt */
        att_cmd[1] = onb_stack_att_tilt(&k)[1];
        u = onb_stack_step(&k, &in, &so);
        for (i = 0; i < so.n_exec; i++)
            if (so.exec[i].code != 0 && so.exec[i].code < ONB_EXEC_NONE) n_rej++;
        if (so.phase == LP_PHASE_HOLD && t_hold < 0) {
            t_hold = (int)t;
        }
        if (so.state == ONB_UNPOWERED && t_landed < 0) t_landed = (int)t;
        if (so.state == ONB_SAFE && t_landed > 0) t_safe = (int)t;
        /* plant */
        for (i = 0; i < 2; i++)
            s.tilt[i] = so.motor_on ? s.tilt[i] + a_att * ((double)att_cmd[i] - s.tilt[i]) : 0.0;
        s.T += (MIX_T_MAX_N * (double)u * (double)u - s.T) * a_T;
        pm3_quat(&s, q);
        pm3_rot(q, ez, nb, false);
        for (i = 0; i < 3; i++)
            acc[i] = s.T / m * nb[i] - (i == 2 ? g0 : s.v[i] / tau_d * (s.ground ? 0.0 : 1.0));
        if (t_hold > 0 && (int)t == t_hold + 1200) {  /* a 3 N s push north over one tick */
            acc[0] += 3.0 / m / dt;
            t_push = (int)t;
        }
        if (s.ground && acc[2] <= 0.0) {
            acc[0] = acc[1] = acc[2] = 0.0;
            s.v[0] = s.v[1] = s.v[2] = 0.0;
        } else {
            s.ground = false;
        }
        for (i = 0; i < 3; i++) {
            s.v[i] += acc[i] * dt;
            s.p[i] += s.v[i] * dt;
        }
        if (s.p[2] < 0.0) {
            s.p[2] = 0.0;
            s.v[0] = s.v[1] = s.v[2] = 0.0;
            s.ground = true;
        }
        if (s.tilt[0] * s.tilt[0] + s.tilt[1] * s.tilt[1] > max_tilt * max_tilt)
            max_tilt = sqrt(s.tilt[0] * s.tilt[0] + s.tilt[1] * s.tilt[1]);
        /* NAV correction part: body specific force, the corrected attitude, the GPS epochs */
        for (i = 0; i < 3; i++)
            fw[i] = s.ground ? (i == 2 ? g0 : 0.0) : acc[i] + (i == 2 ? g0 : 0.0);
        pm3_rot(q, fw, fb, true);
        if (t % 50u == 20u && nq < 4) {               /* measured now, complete 55 ticks (0.22 s) later */
            double r[3];
            const double lb[3] = {gp.lever_b[0], gp.lever_b[1], gp.lever_b[2]};
            double nx, ny;
            pm3_rot(q, lb, r, false);
            lrng = lrng * 1103515245u + 12345u;
            nx = 0.1 * ((double)((lrng >> 8) & 0xFFFF) / 65536.0 - 0.5);
            lrng = lrng * 1103515245u + 12345u;
            ny = 0.1 * ((double)((lrng >> 8) & 0xFFFF) / 65536.0 - 0.5);
            gps_test_epoch(&queue[nq], &fr, s.p[0] + r[0] + nx, s.p[1] + r[1] + ny, s.v[0], s.v[1], 345600000UL + 4u * t, 9,
                           2500);
            qt[nq++] = t + 55u;
        }
        while (nq > 0 && qt[0] <= t) {
            queue[0].tick = t;
            gps_filter_epoch(&F, &queue[0], gps_itow_update(&clk, queue[0].itow, t));
            for (i = 1; i < nq; i++) {
                queue[i - 1] = queue[i];
                qt[i - 1] = qt[i];
            }
            nq--;
        }
        {
            const float fbf[3] = {(float)fb[0], (float)fb[1], (float)fb[2]}, w0[3] = {0.0f, 0.0f, 0.0f};
            if (gps_filter_tick(&F, t, fbf, q, w0, nav_state == LP_STATE_SAFE || nav_state == LP_STATE_ARMED))
                nl_slot_tx_latch(&tx, &F.snap, F.cycle);
        }
        nav_state = so.state;                         /* the FCU frame of this tick reaches the NAV for the next one */
        /* measurements */
        if (so.phase == LP_PHASE_HOLD && k.pipe.pub.mode != POS_MODE_POS && t_hold > 0 && (int)t > t_hold + 10) max_mode_bad++;
        if (t_hold > 0 && (int)t == t_hold + 10) {
            hold[0] = s.p[0];
            hold[1] = s.p[1];
        }
        if (t_hold > 0 && (int)t > t_hold + 10 && (t_push < 0)) {
            const double d = hypot(s.p[0] - hold[0], s.p[1] - hold[1]);
            if (d > max_d_hold) max_d_hold = d;
        }
        if (t_push > 0 && (int)t > t_push && (int)t <= t_push + 1250) {
            const double d = hypot(s.p[0] - hold[0], s.p[1] - hold[1]);
            if (d > max_d_push) max_d_push = d;
            if ((int)t == t_push + 1250) d_back = d;
        }
        if ((int)t == t_landed) land_d = hypot(s.p[0] - hold[0], s.p[1] - hold[1]);
    }
    CHECK(n_rej == 0 && t_hold > t_on && t_landed > t_off && t_off > 0 && t_safe > t_landed, "GPS hop: ON (no reject) %d, HOLD %d, "
          "OFF %d, LANDED %d, SAFE %d", t_on, t_hold, t_off, t_landed, t_safe);
    CHECK(so.reason == LP_REASON_NONE && k.pipe.overruns == 0 && max_mode_bad == 0, "no interlock, no pipeline overrun, POS "
          "mode through HOLD (%d ticks not POS)", max_mode_bad);
    CHECK(max_d_hold < 0.5 && max_d_push > 0.3 && d_back < 0.5 && land_d >= 0.0 && land_d < 1.0 && max_tilt < 0.36,
          "hold within %.2f m, a 3 N s push displaces %.2f m and is back to %.2f m after 5 s, touchdown %.2f m from the hold "
          "point, max tilt %.1f deg", max_d_hold, max_d_push, d_back, land_d, max_tilt * 57.2958);
    printf("info: C closed loop (point mass, attitude lag 0.15 s, GPS 0.1 m noise, NOT the 6-DOF): HOLD at %.2f s, hold within "
           "%.3f m, 3 N s push max %.2f m (back to %.2f m in 5 s), touchdown %.2f m from the hold point at %.2f s, max tilt %.1f "
           "deg; NAV fused %u epochs, %u yaw updates\n", t_hold * dt, max_d_hold, max_d_push, d_back, land_d, t_landed * dt,
           max_tilt * 57.2958, F.st_fused, F.yaw_used);
}

/* ------------------------------------------------------------------------ */
/* v0.8.1 four-chip build: lp_quantize_wide, glink, nlink v4, the GNS node, */
/* keep_trim / trim / hold steps of the position controller, the FCU's GNS  */
/* chain and the whole C chain in a closed loop                             */
/* ------------------------------------------------------------------------ */
static float bits_f(uint32_t u)
{
    float f;
    memcpy(&f, &u, 4);
    return f;
}

static uint32_t f_bits(float f)
{
    uint32_t u;
    memcpy(&u, &f, 4);
    return u;
}

/* "gp.<field>=value": a GPS filter parameter (the names of tools/gen_firmware_params.py gps_c_fields) */
static int gp_override(gps_nav_params_t *p, const char *kv)
{
    char key[64];
    const char *eq = strchr(kv, '=');
    double x;
    size_t n;
    int r, c;
    if (!eq || strncmp(kv, "gp.", 3) != 0) return 0;
    kv += 3;
    n = (size_t)(eq - kv);
    if (n >= sizeof key) return 0;
    memcpy(key, kv, n);
    key[n] = '\0';
    x = strtod(eq + 1, 0);
#define F32(name) if (!strcmp(key, #name)) { p->name = (float)x; return 1; }
#define U32(name) if (!strcmp(key, #name)) { p->name = (uint32_t)x; return 1; }
#define U16(name) if (!strcmp(key, #name)) { p->name = (uint16_t)x; return 1; }
#define U8(name) if (!strcmp(key, #name)) { p->name = (uint8_t)x; return 1; }
    U8(step_ticks) U8(ms_per_tick) U16(latency_ms) F32(loop_hz_f) F32(inv_step) F32(t_step) F32(psi0) F32(c_psi0)
    F32(s_psi0) U8(gate_min_sv) U32(gate_max_hacc_mm) U32(gate_max_sacc_cms) U32(gate_stable_ticks) U8(ok_min_sv)
    U32(ok_max_hacc_mm) F32(gate_v2) F32(gate_p2) U8(n_corr) U16(acq_epochs) U16(reset_after_epochs) U16(vel_reset_epochs)
    U32(fresh_ticks) U16(yaw_window_steps) F32(yaw_min_dv2) F32(yaw_sigma_dv2) F32(yaw_q_rad2) F32(p_psi0) F32(head_ok_rad2)
    F32(yaw_sigma_rel2)
#undef F32
#undef U32
#undef U16
#undef U8
    if (!strcmp(key, "yaw_correction")) { p->yaw_correction = x != 0.0; return 1; }
    if (!strncmp(key, "lever_b", 7) && key[7] >= '0' && key[7] <= '2' && key[8] == '\0') {
        p->lever_b[key[7] - '0'] = (float)x;
        return 1;
    }
    for (r = 0; r < 3; r++) {
        char nm[16];
        for (c = 0; c < 2; c++) {
            sprintf(nm, "s_kpv%d%d", r, c);
            if (!strcmp(key, nm)) { p->steady.kpv[r][c] = (float)x; return 1; }
            sprintf(nm, "a_kpv%d%d", r, c);
            if (!strcmp(key, nm)) { p->acq.kpv[r][c] = (float)x; return 1; }
        }
        sprintf(nm, "s_kp%d", r);
        if (!strcmp(key, nm)) { p->steady.kp[r] = (float)x; return 1; }
        sprintf(nm, "s_kv%d", r);
        if (!strcmp(key, nm)) { p->steady.kv[r] = (float)x; return 1; }
        sprintf(nm, "a_kp%d", r);
        if (!strcmp(key, nm)) { p->acq.kp[r] = (float)x; return 1; }
        sprintf(nm, "a_kv%d", r);
        if (!strcmp(key, nm)) { p->acq.kv[r] = (float)x; return 1; }
    }
    return 0;
}

/* the reference's quantize on an exact double (link_proto.quantize; x scale exact for a float32 x and scale < 2^16) */
static long ref_quantize(double v, long lo, long hi)
{
    if (v != v) return 0;
    if (v >= (double)hi) return hi;
    if (v <= (double)lo) return lo;
    return v >= 0.0 ? (long)floor(v + 0.5) : -(long)floor(-v + 0.5);
}

static void test_quantize_wide(void)
{
    static const uint16_t scales[] = {10000u, 100u, 16384u, 32767u, 625u, 1u, 65535u};
    int i, s, bad = 0, same = 0, ties = 0;
    for (s = 0; s < (int)(sizeof scales / sizeof scales[0]); s++) {
        for (i = 0; i < 40000; i++) {
            float x;
            long want, got;
            const int kind = i % 4;
            if (kind == 0) {
                x = (float)(srand1() * 4.0);
            } else if (kind == 1) {                    /* near the half steps (x scale = n + 1/2 where exact) */
                x = (float)((floor(srand1() * 40000.0) + 0.5) / (double)scales[s]);
                ties++;
            } else if (kind == 2) {
                uint32_t u = (uint32_t)(urand() * 4294967295.0);
                memcpy(&x, &u, 4);
            } else {
                x = (float)(srand1() * 1e-3);
            }
            want = ref_quantize((double)x * (double)scales[s], -32767L, 32767L);
            got = lp_quantize_wide(x, scales[s], -32767L, 32767L);
            if (got != want) {
                if (bad < 5) printf("  quantize_wide x=%.9g scale %u: %ld vs %ld\n", (double)x, scales[s], got, want);
                bad++;
            }
            if (scales[s] == 100u || scales[s] == 16384u || scales[s] == 1u) {   /* odd part < 256: the narrow form too */
                if (lp_quantize(x, scales[s], -32767L, 32767L) != got) bad++;
                else same++;
            }
        }
    }
    CHECK(bad == 0, "lp_quantize_wide == round_half_away(x scale) saturated (exact), and == lp_quantize where both apply: "
          "%d mismatches (%d ties, %d narrow agreements)", bad, ties, same);
    CHECK(nl_tilt_q(0.0312f) == 312 && nl_tilt_q(-0.0871f) == -871 && nl_tilt_q(10.0f) == 32767 && nl_tilt_q(-10.0f) == -32767 &&
          nl_tilt_q(NAN) == 0 && nl_kt_quant(1.0f) == 16384u && nl_kt_quant(-1.0f) == 0u && nl_kt_quant(5.0f) == 65535u &&
          nl_kt_value(15943u) == 15943.0f / 16384.0f, "tilt_q / kt_quant / kt_value (TILT_NONE never produced)");
    CHECK(nl_pc_byte(POS_MODE_POS, true, 0x05u) == (POS_MODE_POS | NL_PC_HOLD | 0x50u) &&
          nl_fcu4_pflags(LP_PHASE_LAND, true, false, true, true, true) == (LP_PHASE_LAND | 0x08u | 0x20u | 0x40u | 0x80u),
          "pc byte and pflags packing (sim/nlink_proto.py pc_byte / fcu4_pflags)");
}

static void glv_state_of(const glv_state_t *v, gl_state_t *s)
{
    int i;
    s->tick = v->tick;
    for (i = 0; i < 3; i++) s->fsum[i] = bits_f(v->fsum[i]);
    s->n = v->n;
    for (i = 0; i < 4; i++) s->q[i] = bits_f(v->q[i]);
    for (i = 0; i < 3; i++) s->w[i] = bits_f(v->w[i]);
    s->h = bits_f(v->h);
    s->nflags = v->nflags;
    s->fcu_state = v->fcu_state;
    s->pflags = v->pflags;
    s->kt_q = v->kt_q;
}

static void glv_result_of(const long f[12], gl_result_t *r)
{
    r->tilt[0] = (int16_t)f[0];
    r->tilt[1] = (int16_t)f[1];
    r->flags = (uint8_t)f[2];
    r->num_sv = (uint8_t)f[3];
    r->hacc_dm = (uint8_t)f[4];
    r->pc = (uint8_t)f[5];
    r->pos[0] = (int16_t)f[6];
    r->pos[1] = (int16_t)f[7];
    r->vel[0] = (int16_t)f[8];
    r->vel[1] = (int16_t)f[9];
    r->trim[0] = (int16_t)f[10];
    r->trim[1] = (int16_t)f[11];
}

static void test_glink(void)
{
    int i, j, bad;
    uint8_t buf[GL_FRAME_MAX_BYTES], pl[GL_STATE_LEN];
    CHECK(GL_STATE_FRAME_BYTES == GLV_STATE_FRAME_BYTES && GL_RESULT_FRAME_BYTES == GLV_RESULT_FRAME_BYTES &&
          GL_STATE_LEN == GLV_STATE_LEN && GL_RESULT_LEN == GLV_RESULT_LEN && GL_GAP_RESTART_TICKS == GLV_GAP_RESTART_TICKS &&
          GL_CLOCK_SLIP_TICKS == GLV_CLOCK_SLIP_TICKS && NL_GNS_RING == GLV_RING, "glink sizes / rules == sim/glink_proto.py");
    {
        const uint8_t nf[4] = GLV_NF;
        CHECK(nf[0] == GL_NF_PAD && nf[1] == GL_NF_FCU_OK && nf[2] == GL_NF_IMU_OK && nf[3] == GL_NF_EST_VALID, "NF_* bits");
        CHECK(il_crc16((const uint8_t *)"123456789", 9, LP_CRC_INIT) == GLV_CRC_CHECK, "glink CRC check value");
    }
    for (i = 0; i < GLV_N_STATES; i++) {               /* STATE fields <-> payload <-> frame */
        const glv_state_t *v = &GLV_STATES[i];
        gl_state_t s, u;
        glv_state_of(v, &s);
        gl_pack_state(&s, pl);
        CHECK(memcmp(pl, v->payload, GL_STATE_LEN) == 0, "STATE %s: payload", v->name);
        CHECK(gl_encode_state(v->seq, &s, buf) == GL_STATE_FRAME_BYTES && memcmp(buf, v->frame, GL_STATE_FRAME_BYTES) == 0,
              "STATE %s: frame", v->name);
        gl_unpack_state(v->payload, &u);
        CHECK(u.tick == v->tick && f_bits(u.fsum[0]) == v->fsum[0] && f_bits(u.fsum[2]) == v->fsum[2] && u.n == v->n &&
              f_bits(u.q[3]) == v->q[3] && f_bits(u.w[1]) == v->w[1] && f_bits(u.h) == v->h && u.nflags == v->nflags &&
              u.fcu_state == v->fcu_state && u.pflags == v->pflags && u.kt_q == v->kt_q, "STATE %s: unpack bit for bit",
              v->name);
    }
    for (i = 0; i < GLV_N_RESULTS; i++) {              /* RESULT fields <-> payload (= the five slot parts) <-> frame */
        const glv_result_t *v = &GLV_RESULTS[i];
        gl_result_t r, u;
        glv_result_of(v->f, &r);
        gl_pack_result(&r, pl);
        CHECK(memcmp(pl, v->payload, GL_RESULT_LEN) == 0, "RESULT %s: payload", v->name);
        CHECK(gl_encode_result(v->seq, &r, buf) == GL_RESULT_FRAME_BYTES && memcmp(buf, v->frame, GL_RESULT_FRAME_BYTES) == 0,
              "RESULT %s: frame", v->name);
        gl_unpack_result(v->payload, &u);
        CHECK(memcmp(&u, &r, sizeof u) == 0 || (u.tilt[0] == r.tilt[0] && u.tilt[1] == r.tilt[1] && u.pc == r.pc &&
                                                 u.trim[1] == r.trim[1] && u.pos[0] == r.pos[0] && u.vel[1] == r.vel[1]),
              "RESULT %s: unpack", v->name);
    }
    bad = 0;
    for (i = 0; i < GLV_N_DECODES; i++) {              /* whole buffers: the reject reason / the decode */
        const glv_dec_t *v = &GLV_DECODES[i];
        uint8_t ty = 0, sq = 0;
        const uint8_t ok = gl_decode_frame(v->buf, v->n, &ty, &sq);
        if (gl_reject_reason(v->buf, v->n) != v->reason || ok != v->valid || (ok && (ty != v->type || sq != v->seq))) {
            if (bad < 5) printf("  glink decode %s: reason %u valid %u\n", v->name, gl_reject_reason(v->buf, v->n), ok);
            bad++;
        }
    }
    CHECK(bad == 0, "glink: %d whole-buffer decodes (valid, every single-bit error, TYPE / length) as the reference", GLV_N_DECODES);
    for (i = 0; i < GLV_N_STREAMS; i++) {              /* the streaming decoder: frames and counters */
        const glv_stream_t *v = &GLV_STREAMS[i];
        gl_decoder_t d;
        int nfr = 0, ok = 1;
        gl_decoder_init(&d, v->accept);
        for (j = 0; j < v->n; j++) {
            const uint8_t t = gl_decoder_byte(&d, v->data[j]);
            if (t) {
                if (nfr >= v->n_frames || t != v->frames[nfr].type || d.seq != v->frames[nfr].seq ||
                    memcmp(d.buf, v->frames[nfr].payload, t == GL_TYPE_STATE ? GL_STATE_LEN : GL_RESULT_LEN) != 0)
                    ok = 0;
                nfr++;
            }
        }
        CHECK(ok && nfr == v->n_frames && d.stats.frames == v->stats[0] && d.stats.bad_crc == v->stats[1] &&
              d.stats.bad_type == v->stats[2] && d.stats.wrong_dir == v->stats[3] && d.stats.skipped == v->stats[4],
              "glink stream %s: %d frames, counters %u %u %u %u %u", v->name, nfr, d.stats.frames, d.stats.bad_crc,
              d.stats.bad_type, d.stats.wrong_dir, d.stats.skipped);
    }
    for (i = 0; i < GLV_N_TX; i++) {                   /* the NAV's slot phase (GnsSlotTx) */
        const glv_tx_t *v = &GLV_TX[i];
        nl_gns_tx_t tx;
        int e = 0, ok = 1;
        uint8_t sl[NL_SLOT_LEN];
        nl_gns_tx_init(&tx, 5, v->L);
        for (j = 0; j < v->n_ticks; j++) {
            while (e < v->n_ev && v->ev[e].k == (uint32_t)j) {
                nl_gns_tx_result(&tx, v->ev[e].seq, v->ev[e].pl, (uint32_t)j);
                e++;
            }
            nl_gns_tx_slot(&tx, (uint32_t)j, sl);
            if (memcmp(sl, v->slots[j], NL_SLOT_LEN) != 0) {
                if (ok) printf("  slot_tx %s tick %d: %02x%02x%02x%02x%02x\n", v->name, j, sl[0], sl[1], sl[2], sl[3], sl[4]);
                ok = 0;
            }
        }
        CHECK(ok && tx.latched == v->latched && tx.missed == v->missed, "GnsSlotTx %s (L %u): every slot, latched %u / %u, "
              "missed %u / %u", v->name, v->L, tx.latched, v->latched, tx.missed, v->missed);
    }
    {   /* the slot phase also from a cold start in the middle (k not consecutive: the division path) */
        nl_gns_tx_t a, b;
        uint8_t s1[NL_SLOT_LEN], s2[NL_SLOT_LEN];
        int ok = 1;
        nl_gns_tx_init(&a, 5, 3);
        nl_gns_tx_init(&b, 5, 3);
        for (j = 0; j < 400; j++) {
            if (j % 5 == 0 && j > 0) {
                const uint8_t res[20] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, (uint8_t)j};
                nl_gns_tx_result(&a, (uint8_t)(j / 5 - 1), res, (uint32_t)j);
                nl_gns_tx_result(&b, (uint8_t)(j / 5 - 1), res, (uint32_t)j);
            }
            nl_gns_tx_slot(&a, (uint32_t)j, s1);
            b.ph_ok = false;                           /* b recomputes the phase by division every tick */
            nl_gns_tx_slot(&b, (uint32_t)j, s2);
            if (memcmp(s1, s2, NL_SLOT_LEN) != 0) ok = 0;
        }
        CHECK(ok && a.latched == b.latched && a.latched > 70u, "slot phase: counted == divided (%u sets latched)", a.latched);
    }
    for (i = 0; i < GLV_N_RX; i++) {                   /* the FCU's slot receiver (GnsSlotRx) */
        const glv_rx_t *v = &GLV_RX[i];
        nl_gns_rx_t rx;
        int ok = 1;
        nl_gns_rx_init(&rx, 5);
        for (j = 0; j < v->n; j++) {
            const glv_rxo_t *o = &v->out[j];
            const nl_gns_data_t *g = nl_gns_rx_feed(&rx, v->slots[j][0] ? &v->slots[j][1] : 0);
            float tg[2];
            const bool eng = nl_gns_rx_target(&rx, tg);
            int good = (g != 0) == (o->has_set != 0);
            if (g && good) {
                const int16_t tx = (int16_t)(rx.pl[NL_SLOT4_TILT][0] | (rx.pl[NL_SLOT4_TILT][1] << 8));
                const int16_t ty = (int16_t)(rx.pl[NL_SLOT4_TILT][2] | (rx.pl[NL_SLOT4_TILT][3] << 8));
                const bool en = tx != NL_TILT_NONE;
                good = g->flags == o->s[0] && g->num_sv == o->s[1] && g->hacc_dm == o->s[2] &&
                       (g->pc & NL_PC_MODE_MASK) == o->s[3] && ((g->pc & NL_PC_HOLD) != 0) == (o->s[4] != 0) &&
                       (g->pc >> NL_PC_REQ_SHIFT) == o->s[5] && g->pos[0] == o->s[6] && g->pos[1] == o->s[7] &&
                       g->vel[0] == o->s[8] && g->vel[1] == o->s[9] && g->trim[0] == o->s[10] && g->trim[1] == o->s[11] &&
                       (en ? tx : 0) == o->s[12] && (en ? ty : 0) == o->s[13] && en == (o->s[14] != 0) && g->cycle == o->s[15];
            }
            good = good && eng == (o->engaged != 0) && fabsf(tg[0] - bits_f(o->tgt[0])) <= 2e-7f &&
                   fabsf(tg[1] - bits_f(o->tgt[1])) <= 2e-7f && nl_gns_rx_fallback(&rx) == (o->fallback != 0) &&
                   nl_gns_rx_requests(&rx, v->land_steps) == o->requests && nl_gns_rx_gate_ok(&rx) == (o->gate_ok != 0);
            if (!good) {
                if (ok) printf("  slot_rx %s tick %d differs\n", v->name, j);
                ok = 0;
            }
        }
        CHECK(ok && rx.sets == v->sets && rx.broken == v->broken, "GnsSlotRx %s: every tick (set, target, fallback, requests, "
              "gate); sets %u broken %u", v->name, rx.sets, rx.broken);
    }
    for (i = 0; i < GLV_N_FCU4; i++) {                 /* the nlink v4 FCU frame (glink_vectors.json fcu4) */
        const glv_fcu4_t *v = &GLV_FCU4[i];
        nl_fcu4_t f, u;
        uint8_t x[NL_XFER_BYTES];
        f.seq = (uint8_t)v->f[0]; f.state = (uint8_t)v->f[1]; f.fflags = (uint8_t)v->f[2]; f.params_tag = (uint8_t)v->f[3];
        f.thr_q = (uint8_t)v->f[4]; f.pflags = (uint8_t)v->f[5]; f.kt_q = (uint16_t)v->f[6];
        CHECK(nl_encode_fcu4(&f, x) == NL_FCU4_FRAME_BYTES && memcmp(x, v->buf, NL_XFER_BYTES) == 0, "FCU4 %s: 72 bytes",
              v->name);
        CHECK(nl_decode_fcu4(v->buf, &u) == v->valid && u.kt_q == f.kt_q && u.pflags == f.pflags && u.thr_q == f.thr_q &&
              u.seq == f.seq, "FCU4 %s: decode", v->name);
    }
}

static void test_nlink_v4(void)
{
    int i, bad = 0;
    uint8_t x[NL_XFER_BYTES];
    const uint8_t pf[6] = NLV4_PF, pc[3] = NLV4_PC;
    CHECK(NL_TYPE_NAV4 == NLV4_TYPE_NAV4 && NL_TYPE_FCU4 == NLV4_TYPE_FCU4 && NL_FCU4_FRAME_BYTES == NLV4_FCU4_FRAME_BYTES &&
          NL_KT_SCALE == NLV4_KT_SCALE && NL_TILT_NONE == NLV4_TILT_NONE && NL_SLOT_STALE_TICKS == NLV4_STALE_TICKS &&
          NL_GNS_RING == NLV4_RING, "nlink v4 constants == sim/nlink_proto.py");
    CHECK(pf[0] == NL_PF_PHASE_MASK && pf[1] == NL_PF_MOTOR_ON && pf[2] == NL_PF_SAT_HI && pf[3] == NL_PF_LOW_ALT &&
          pf[4] == NL_PF_GROUND && pf[5] == NL_PF_LIFTED && pc[0] == NL_PC_MODE_MASK && pc[1] == NL_PC_HOLD &&
          pc[2] == NL_PC_REQ_SHIFT, "pflags / pc bits");
    for (i = 0; i < NLV4_N_NAVS; i++) {
        const nlv4_nav_t *v = &NLV4_NAVS[i];
        nl_nav3_t n, u;
        nav3_from_fields(&n, v->seq, v->fb, v->iv, v->slot);
        CHECK(nl_encode_nav4(&n, x) == NL_XFER_BYTES && memcmp(x, v->frame, NL_XFER_BYTES) == 0 &&
              memcmp(x + NL_HEADER_BYTES, v->payload, NL_NAV3_LEN) == 0, "NAV4 %s: frame", v->name);
        CHECK(nl_decode_nav4(v->frame, &u) && nav3_matches(&u, v->seq, v->fb, v->iv, v->slot) && !nl_decode_nav3(v->frame, &u),
              "NAV4 %s: decoded by the v4 decoder only", v->name);
    }
    for (i = 0; i < NLV4_N_FCUS; i++) {
        const nlv4_fcu_t *v = &NLV4_FCUS[i];
        nl_fcu4_t f;
        f.seq = (uint8_t)v->f[0]; f.state = (uint8_t)v->f[1]; f.fflags = (uint8_t)v->f[2]; f.params_tag = (uint8_t)v->f[3];
        f.thr_q = (uint8_t)v->f[4]; f.pflags = (uint8_t)v->f[5]; f.kt_q = (uint16_t)v->f[6];
        CHECK(nl_encode_fcu4(&f, x) == NL_FCU4_FRAME_BYTES && memcmp(x, v->buf, NL_XFER_BYTES) == 0, "FCU4 %s", v->name);
    }
    for (i = 0; i < NLV4_N_DECODES; i++) {
        const nlv4_dec_t *v = &NLV4_DECODES[i];
        nl_nav3_t n;
        nl_fcu4_t f;
        nl_fcu_t f3;
        const int a = nl_decode_nav4(v->buf, &n), b = nl_decode_fcu4(v->buf, &f), c = nl_decode_nav3(v->buf, &n),
                  d = nl_decode_fcu(v->buf, &f3);
        if (a != v->nav4_valid || b != v->fcu4_valid || c != v->nav3_valid || d != v->fcu_valid ||
            (b && (f.seq != v->fcu4[0] || f.state != v->fcu4[1] || f.fflags != v->fcu4[2] || f.params_tag != v->fcu4[3] ||
                   f.thr_q != v->fcu4[4] || f.pflags != v->fcu4[5] || f.kt_q != v->fcu4[6]))) {
            if (bad < 5) printf("  nlink v4 decode %s: %d %d %d %d\n", v->name, a, b, c, d);
            bad++;
        }
    }
    CHECK(bad == 0, "nlink v4: %d decode cases through the v4 / v3 / v0.7.1 decoders as the reference", NLV4_N_DECODES);
}

/* the GNS replay of sim/glink_proto.py gns_replay_vector: module-constant parameters (the generator's mapping as
 * override tokens), per NAV tick poll(k, UBX bytes), then the STATE sent in tick k - 1; every RESULT compared */
static void gns_apply_overrides(gps_nav_params_t *gp, pos_params_t *pp, const char *const *tok, int n)
{
    int i;
    for (i = 0; i < n; i++)
        CHECK(gp_override(gp, tok[i]) || pos_override(pp, tok[i]), "override %s known", tok[i]);
}

static void test_gns_replay(void)
{
    static gns_t g;
    gps_nav_params_t gp;
    pos_params_t pp;
    int i, u = 0, ci = 0, bad = 0, one = 0;
    double dp = 0.0, dv = 0.0, db = 0.0, dt_ = 0.0;
    const uint16_t st[7] = GLV_REPLAY_STATS, fs[9] = GLV_REPLAY_FSTATS;
    fc_default_gps_nav(&gp);
    fc_default_position(&pp);
    gns_apply_overrides(&gp, &pp, GLV_REPLAY_OV, GLV_REPLAY_N_OV);
    gns_init(&g, &gp, &pp, false, 1, 1, 1, 1, 38400UL);
    for (i = 0; ci < GLV_REPLAY_N && i < 20000; i++) {
        gps_cfg_action_t act;
        const uint32_t k = (uint32_t)i;
        const uint8_t *ub = 0;
        uint16_t nub = 0;
        if (u < GLV_REPLAY_N_UBX && GLV_REPLAY_UBX[u].tick == k) {
            ub = GLV_REPLAY_UBX[u].data;
            nub = GLV_REPLAY_UBX[u].n;
            u++;
        }
        gns_poll(&g, k, ub, nub, &act);
        if (k >= 5u && k % 5u == 0u) {                 /* the STATE sent in tick k - 1 */
            const glv_cycle_t *c = &GLV_REPLAY[ci];
            uint8_t out[GL_RESULT_FRAME_BYTES];
            int got = 0, j;
            for (j = 0; j < (int)GL_STATE_FRAME_BYTES; j++)
                got += gns_rx_byte(&g, c->state[j], out) ? 1 : 0;
            if (got == 1) {
                const gps_filter_t *f = &g.gps.filter;
                const pos_pub_t *po = &g.pos.last;
                int ok = 1;
                (void)gns_finish(&g);                  /* the reference records the filter after the cycle's fusion */
                for (j = 0; j < (int)GL_RESULT_FRAME_BYTES; j++) {
                    if (out[j] != c->result[j]) {
                        ok = 0;
                        break;
                    }
                }
                if (!ok) {                             /* one count at a float32 rounding boundary, nothing else */
                    gl_result_t a, b;
                    int m;
                    gl_unpack_result(out + GL_HEADER_BYTES, &a);
                    gl_unpack_result(c->result + GL_HEADER_BYTES, &b);
                    ok = a.flags == b.flags && a.num_sv == b.num_sv && a.hacc_dm == b.hacc_dm && a.pc == b.pc &&
                         (a.tilt[0] == NL_TILT_NONE) == (b.tilt[0] == NL_TILT_NONE);
                    for (m = 0; m < 2; m++)
                        ok = ok && abs(a.tilt[m] - b.tilt[m]) <= 1 && abs(a.trim[m] - b.trim[m]) <= 1 &&
                             abs(a.pos[m] - b.pos[m]) <= 1 && abs(a.vel[m] - b.vel[m]) <= 1;
                    if (ok) one++;
                }
                for (j = 0; j < 2; j++) {
                    dp = fmax(dp, fabs(f->p_[j] - c->p[j]));
                    dv = fmax(dv, fabs(f->v[j] - c->v[j]));
                    db = fmax(db, fabs(f->b[j] - c->b[j]));
                    dt_ = fmax(dt_, fmax(fabs(po->tilt_l[j] - c->tilt_l[j]), fabs(po->trim_l[j] - c->trim_l[j])));
                }
                ok = ok && po->mode == c->mode && po->requests == c->requests && f->reset_count == c->reset_count;
                if (!ok) {
                    if (bad < 4) printf("  gns replay cycle %d (tick %u): mode %u/%u requests %u/%u\n", ci, k - 1u, po->mode,
                                        c->mode, po->requests, c->requests);
                    bad++;
                }
            } else {
                bad++;
            }
            (void)gns_finish(&g);
            ci++;
        }
    }
    CHECK(ci == GLV_REPLAY_N && bad == 0 && dp < 1e-3 && dv < 1e-4 && db < 1e-4 && dt_ < 2e-5,
          "GNS replay (sim/glink_proto.py gns_replay_vector): %d cycles, %d differing, RESULT one count off at a rounding "
          "boundary %d; max |d| p %.2g m, v %.2g m/s, b %.2g m/s^2, tilt / trim %.2g rad", ci, bad, one, dp, dv, db, dt_);
    CHECK(g.st.states == st[0] && g.st.hold_steps == st[1] && g.st.gaps == st[2] && g.st.restarts == st[3] &&
          g.st.imu_bad == st[4] && g.st.clock_slips == st[5] && g.st.misaligned == st[6], "GNS replay: node counters");
    {
        const gps_filter_t *f = &g.gps.filter;
        CHECK(f->st_fused == fs[0] && f->st_rej_p == fs[1] && f->st_rej_v == fs[2] && f->st_pos_resets == fs[3] &&
              f->st_vel_resets == fs[4] && f->st_dropped_old == fs[5] && f->st_dropped_stale == fs[6] &&
              f->st_n_epoch_dropped == fs[7] && f->st_unusable == fs[8], "GNS replay: filter counters fused %u rej_v %u "
              "stale %u", f->st_fused, f->st_rej_v, f->st_dropped_stale);
    }
}

/* the position controller's v0.8.1 additions: keep_trim, the trim output, pos_hold_steps */
static void pos_engage_hover(pos_t *a, pos_inputs_t *in, pos_data_t *d, const pos_params_t *pp, float vx_wind)
{
    int j;
    pos_init(a, pp);
    memset(in, 0, sizeof *in);
    memset(d, 0, sizeof *d);
    in->phase = LP_PHASE_HOLD;
    in->motor_on = true;
    in->h_m = 3.0f;
    in->cos_tilt = 1.0f;
    in->k_t = 1.0f;
    in->lifted = true;
    d->flags = 0x2F;
    d->cs[0] = 1.0f;
    in->data = d;
    for (j = 0; j < 400; j++) {                        /* a steady drift the integrator learns (the wind trim) */
        d->v[0] = vx_wind;
        (void)pos_step(a, in);
    }
}

static void test_position_v081(void)
{
    static pos_t a, b;
    pos_params_t pp;
    pos_inputs_t in;
    pos_data_t d;
    int j, bad;
    fc_default_position(&pp);
    {   /* keep_trim: in the fade band only the transient above the trim is limited (|a - i| <= f a_lim) */
        float f, lim, e;
        pp.keep_trim = 1;
        pos_engage_hover(&a, &in, &d, &pp, 0.4f);
        CHECK(fabsf(a.integ[0]) > 0.3f && a.out.trim_l[1] != 0.0f, "a steady drift builds the trim (integ %.3f m/s^2, "
              "trim %.4f rad)", a.integ[0], a.out.trim_l[1]);
        in.phase = LP_PHASE_LAND;
        in.vc_low_alt = true;
        in.h_m = 0.15f;                                /* f = 0.15 / tilt_fade_m */
        d.v[0] = 1.2f;                                 /* a large velocity error: a transient beyond the limit */
        (void)pos_step(&a, &in);
        f = 0.15f / pp.tilt_fade_m;
        lim = f * sqrtf(a.out.a_lim2);
        e = hypotf(a.out.a_cmd[0] - a.integ[0], a.out.a_cmd[1] - a.integ[1]);
        CHECK(e <= lim * 1.0001f + 1e-6f && hypotf(a.out.a_cmd[0], a.out.a_cmd[1]) > lim,
              "keep_trim at h 0.15 m: transient %.4f <= f a_lim %.4f, |a| %.4f keeps the trim", e, lim,
              hypotf(a.out.a_cmd[0], a.out.a_cmd[1]));
        in.h_m = 0.0f;                                 /* f = 0: a = the trim exactly */
        (void)pos_step(&a, &in);
        CHECK(a.out.a_cmd[0] == a.integ[0] && a.out.a_cmd[1] == a.integ[1], "keep_trim at h 0: a = the integrator (trim)");
        in.vc_ground = true;                           /* on the ground: upright (a_lim 0) */
        (void)pos_step(&a, &in);
        CHECK(a.out.a_cmd[0] == 0.0f && a.out.a_cmd[1] == 0.0f && a.out.trim_l[0] == 0.0f && a.out.trim_l[1] == 0.0f,
              "ground signature: a = 0, trim limited to 0");
        pp.keep_trim = 0;                              /* the v0.8 fade: a_lim^2 f^2, |a| <= f a_lim */
        pos_engage_hover(&b, &in, &d, &pp, 0.4f);
        in.phase = LP_PHASE_LAND;
        in.vc_low_alt = true;
        in.h_m = 0.15f;
        d.v[0] = 1.2f;
        (void)pos_step(&b, &in);
        {
            float cl, sl, a2;
            pos_tilt_limit(&b, 1.0f, true, &cl, &sl, &a2);
            CHECK(NEAR(b.out.a_lim2, (double)a2 * f * f, 1e-6 * a2) &&
                  hypotf(b.out.a_cmd[0], b.out.a_cmd[1]) <= sqrtf(b.out.a_lim2) * 1.0001f,
                  "fade at h 0.15 m: a_lim^2 x f^2, |a| %.4f <= f a_lim %.4f", hypotf(b.out.a_cmd[0], b.out.a_cmd[1]),
                  sqrtf(b.out.a_lim2));
        }
    }
    {   /* the trim output: the integrator limited to a_lim through the step-10 series and the heading rotation */
        double tw[2], c, s;
        pp.keep_trim = 1;
        pos_engage_hover(&a, &in, &d, &pp, -0.3f);
        d.cs[0] = 0.8f;
        d.cs[1] = 0.6f;
        d.v[0] = 0.0f;
        (void)pos_step(&a, &in);
        {
            double ix = a.integ[0], iy = a.integ[1], n2 = ix * ix + iy * iy, gg = (double)pp.g, sq, kk;
            if (n2 > a.out.a_lim2) {
                const double sc = sqrt(a.out.a_lim2 / n2);
                ix *= sc;
                iy *= sc;
            }
            sq = (ix * ix + iy * iy) / (gg * gg);
            kk = (1.0 - sq / 3.0 + sq * sq / 5.0) / gg;
            tw[0] = -iy * kk;
            tw[1] = ix * kk;
            c = 0.8;
            s = 0.6;
        }
        CHECK(NEAR(a.out.trim_l[0], c * tw[0] + s * tw[1], 2e-6) && NEAR(a.out.trim_l[1], -s * tw[0] + c * tw[1], 2e-6) &&
              a.last.trim_l[0] == a.out.trim_l[0] && a.last.has_hold && a.last.hold[0] == a.out.hold[0],
              "trim_l = Rz(-psi) series(integ limited) (%.5f, %.5f); the published subset carries it and the hold point",
              a.out.trim_l[0], a.out.trim_l[1]);
    }
    {   /* pos_hold_steps(m) == m hold steps, from random states (engaged / guard / lost / not flying) */
        bad = 0;
        for (j = 0; j < 300; j++) {
            pos_inputs_t h;
            nl_gps_set_t sd;
            const uint16_t m = (uint16_t)(1 + (j % 7 == 0 ? 400 : (int)(urand() * 30.0)));
            uint16_t r;
            int t;
            pos_init(&a, &pp);
            for (t = 0; t < (int)(urand() * 300.0); t++) {
                pos_rand_inputs(&in, &sd, (uint32_t)(t + 37 * j));
                (void)pos_step(&a, &in);
            }
            pos_rand_inputs(&h, &sd, (uint32_t)(1000 + j));
            h.data = 0;
            h.set = 0;
            if (j % 5 == 1) { h.rate[2] = 5.0f; }       /* the guard re-triggers every step */
            if (j % 5 == 2) { h.motor_on = false; }    /* not flying */
            b = a;
            (void)pos_hold_steps(&a, &h, m);
            for (r = 0; r < m; r++)
                (void)pos_step(&b, &h);
            if (a.n != b.n || a.engaged != b.engaged || a.lost != b.lost || a.guard_left != b.guard_left ||
                a.requests != b.requests || a.implausible != b.implausible || a.has_hold != b.has_hold ||
                memcmp(&a.last, &b.last, sizeof a.last) != 0 || a.out.requests != b.out.requests ||
                a.out.guard != b.out.guard || a.out.lost_steps != b.out.lost_steps || a.out.mode != b.out.mode ||
                memcmp(a.integ, b.integ, sizeof a.integ) != 0)
                bad++;
        }
        CHECK(bad == 0, "pos_hold_steps(m) == m hold steps on 300 random states (%d differ)", bad);
    }
}

/* the FCU's GNS chain (ONB_CHAIN_GNS) - and the whole C chain in a closed loop: the NAV (gl_nav: force sum, STATE
 * frames, the RESULT ring and the slot phase), the GNS (gns_node: the GPS filter and the position controller, epochs
 * injected at the filter), the FCU (nl_gns_rx in the stack: supervisor, vertical controller, the tilt builder) on the
 * 3-D point mass of test_stack_v08 (attitude lag 0.15 s; NOT the 6-DOF), every frame through its encoder / decoder */
static void test_gns_chain(void)
{
    static onb_stack_t k;
    static gns_t G;
    static gl_nav_t N;
    onb_params_t p;
    vc_params_t vp;
    pos_params_t pp;
    gps_nav_params_t gp;
    gps_frame_t fr;
    onb_outputs_t so;
    onb_frame_t f;
    onb_inputs_t in;
    pm3_t s;
    gps_epoch_t queue[4];
    uint32_t qt[4];
    uint8_t st_buf[GL_STATE_FRAME_BYTES], res_buf[GL_RESULT_FRAME_BYTES], xfer[NL_XFER_BYTES];
    bool st_pending = false, res_pending = false;
    nl_fcu4_t fcu_cur, fcu_next;
    bool have_fcu = false;
    int nq = 0;
    uint32_t t;
    int dec_bad = 0;
    uint8_t seq = 10;
    const double m = VEH_MASS_KG, g0 = 9.80665, dt = FC_DT_S, a_att = 1.0 - exp(-dt / 0.15), a_T = 1.0 - exp(-dt / EDF_TAU_S);
    const double tau_d = 9.55;
    int t_on = 1750, t_hold = -1, t_push = -1, t_off = -1, t_landed = -1, t_safe = -1, n_rej = 0, not_pos = 0;
    double hold[2] = {0.0, 0.0}, max_d_hold = 0.0, max_d_push = 0.0, d_back = -1.0, land_d = -1.0, max_tilt = 0.0;
    uint32_t lrng = 4242u;
    fc_default_onboard(&p);
    p.mode = LP_MODE_VTOL;
    p.gps_hold = true;
    p.takeoff_check = true;
    fc_default_vertical(&vp);
    fc_default_position(&pp);
    fc_default_gps_nav(&gp);
    onb_stack_init(&k, &p, &vp, &pp, ONB_CHAIN_GNS, 7, false);
    gns_init(&G, &gp, &pp, false, 1, 1, 1, 1, 38400UL);
    gl_nav_init(&N, 5, GLINK_LATENCY_TICKS);
    gps_frame_at(&fr, 375665432L, 1269780123L);
    memset(&s, 0, sizeof s);
    s.ground = true;
    memset(&in, 0, sizeof in);
    memset(&fcu_cur, 0, sizeof fcu_cur);
    CHECK(k.gps && k.chain == ONB_CHAIN_GNS && k.land_steps == pp.land_steps, "the VTOL stack with gps_hold flies the GNS chain");
    for (t = 0; t < 30000u && t_safe < 0; t++) {
        float q[4], ct, td, att_cmd[2], u;
        uint8_t slot[NL_SLOT_LEN], cmd = 0xFF;
        double nb[3], acc[3], fw[3], fb[3];
        const double ez[3] = {0.0, 0.0, 1.0};
        nl_nav3_t nav;
        int i;
        if ((int)t == t_on) cmd = LP_CMD_ON;
        if (t_hold > 0 && (int)t == t_hold + 2500) cmd = LP_CMD_OFF;
        if (cmd == LP_CMD_OFF) t_off = (int)t;
        /* the NAV: the RESULT the GNS sent in the previous tick (decoded in the background), then this tick's frame */
        if (res_pending) {
            (void)gl_nav_rx(&N, res_buf, GL_RESULT_FRAME_BYTES, t);
            res_pending = false;
        }
        pm3_quat(&s, q);
        memset(&nav, 0, sizeof nav);
        nav.seq = (uint8_t)t;
        for (i = 0; i < 4; i++) nav.q[i] = q[i];
        nl_nav3_tilt(q, &nav.cos_tilt, &nav.tilt_deg);
        gl_nav_slot(&N, t, nav.slot);
        (void)nl_encode_nav4(&nav, xfer);
        dec_bad += nl_decode_nav4(xfer, &nav) != 1;    /* the FCU decodes the 72-byte exchange */
        for (i = 0; i < (int)NL_SLOT_LEN; i++) slot[i] = nav.slot[i];
        ct = nav.cos_tilt;
        td = nav.tilt_deg;
        /* the FCU */
        in.tick = t;
        in.n_frames = 0;
        if (cmd != 0xFF || t % 50u == 0u) {
            if (cmd != 0xFF) seq++;
            f.seq = seq;
            f.cmd.cmd = cmd == 0xFF ? LP_CMD_NONE : cmd;
            f.cmd.arg = 0;
            f.cmd.nonce = k.sup.nonce;
            in.frames = &f;
            in.n_frames = 1;
        }
        in.h_m = (float)s.p[2];
        in.vz_mps = (float)s.v[2];
        in.tilt_deg = td;
        in.gyro[0] = in.gyro[1] = in.gyro[2] = 0.0f;
        in.v_cell = 4.0f;
        in.imu_ok = in.baro_ok = in.aligned = in.arm_switch = true;
        in.cos_tilt = ct;
        in.accel_z_mps2 = (float)(s.ground ? g0 : s.T / m);
        in.gps_slot = slot;
        att_cmd[0] = onb_stack_att_tilt(&k)[0];
        att_cmd[1] = onb_stack_att_tilt(&k)[1];
        u = onb_stack_step(&k, &in, &so);
        for (i = 0; i < so.n_exec; i++)
            if (so.exec[i].code != 0 && so.exec[i].code < ONB_EXEC_NONE) n_rej++;
        if (so.phase == LP_PHASE_HOLD && t_hold < 0) t_hold = (int)t;
        if (so.state == ONB_UNPOWERED && t_landed < 0) t_landed = (int)t;
        if (so.state == ONB_SAFE && t_landed > 0) t_safe = (int)t;
        fcu_next.seq = nav.seq;                        /* its reply to the next exchange (nlink v4) */
        fcu_next.state = so.state;
        fcu_next.fflags = 0;
        fcu_next.params_tag = 0;
        fcu_next.thr_q = nl_fcu_throttle_q(k.throttle);
        fcu_next.pflags = onb_stack_fcu4_pflags(&k);
        fcu_next.kt_q = onb_stack_fcu4_kt_q(&k);
        (void)nl_encode_fcu4(&fcu_next, xfer);
        dec_bad += nl_decode_fcu4(xfer, &fcu_next) != 1;
        /* the plant */
        for (i = 0; i < 2; i++)
            s.tilt[i] = so.motor_on ? s.tilt[i] + a_att * ((double)att_cmd[i] - s.tilt[i]) : 0.0;
        s.T += (MIX_T_MAX_N * (double)u * (double)u - s.T) * a_T;
        pm3_quat(&s, q);
        pm3_rot(q, ez, nb, false);
        for (i = 0; i < 3; i++)
            acc[i] = s.T / m * nb[i] - (i == 2 ? g0 : s.v[i] / tau_d * (s.ground ? 0.0 : 1.0));
        if (t_hold > 0 && (int)t == t_hold + 1200) {  /* a 3 N s push north over one tick */
            acc[0] += 3.0 / m / dt;
            t_push = (int)t;
        }
        if (s.ground && acc[2] <= 0.0) {
            acc[0] = acc[1] = acc[2] = 0.0;
            s.v[0] = s.v[1] = s.v[2] = 0.0;
        } else {
            s.ground = false;
        }
        for (i = 0; i < 3; i++) {
            s.v[i] += acc[i] * dt;
            s.p[i] += s.v[i] * dt;
        }
        if (s.p[2] < 0.0) {
            s.p[2] = 0.0;
            s.v[0] = s.v[1] = s.v[2] = 0.0;
            s.ground = true;
        }
        if (s.tilt[0] * s.tilt[0] + s.tilt[1] * s.tilt[1] > max_tilt * max_tilt)
            max_tilt = sqrt(s.tilt[0] * s.tilt[0] + s.tilt[1] * s.tilt[1]);
        /* the NAV's correction part: the force sum and, at k mod 5 == 4, the STATE frame */
        for (i = 0; i < 3; i++)
            fw[i] = s.ground ? (i == 2 ? g0 : 0.0) : acc[i] + (i == 2 ? g0 : 0.0);
        pm3_rot(q, fw, fb, true);
        {
            const float fbf[3] = {(float)fb[0], (float)fb[1], (float)fb[2]}, w0[3] = {0.0f, 0.0f, 0.0f};
            const bool pad = !have_fcu || fcu_cur.state == LP_STATE_SAFE || fcu_cur.state == LP_STATE_ARMED;
            if (gl_nav_tick(&N, t, fbf, q, w0, (float)s.p[2], pad, have_fcu ? &fcu_cur : 0, have_fcu, true, true, st_buf))
                st_pending = true;
        }
        fcu_cur = fcu_next;                            /* the NAV sees the FCU's reply from the next exchange on */
        have_fcu = true;
        /* the GNS: its 4 ms poll (no UBX bytes: the epochs are injected at the filter below), then the STATE of this tick */
        {
            gps_cfg_action_t act;
            gns_poll(&G, t + 1u, 0, 0, &act);
            if (t % 50u == 20u && nq < 4) {           /* measured now, complete 55 ticks (0.22 s) later */
                double r[3];
                const double lb[3] = {gp.lever_b[0], gp.lever_b[1], gp.lever_b[2]};
                double nx, ny;
                pm3_rot(q, lb, r, false);
                lrng = lrng * 1103515245u + 12345u;
                nx = 0.1 * ((double)((lrng >> 8) & 0xFFFF) / 65536.0 - 0.5);
                lrng = lrng * 1103515245u + 12345u;
                ny = 0.1 * ((double)((lrng >> 8) & 0xFFFF) / 65536.0 - 0.5);
                gps_test_epoch(&queue[nq], &fr, s.p[0] + r[0] + nx, s.p[1] + r[1] + ny, s.v[0], s.v[1],
                               345600000UL + 4u * t, 9, 2500);
                qt[nq++] = t + 55u;
            }
            while (nq > 0 && qt[0] <= t) {
                queue[0].tick = t + 1u;
                gps_filter_epoch(&G.gps.filter, &queue[0], gps_itow_update(&G.gps.clock, queue[0].itow, t + 1u));
                for (i = 1; i < nq; i++) {
                    queue[i - 1] = queue[i];
                    qt[i - 1] = qt[i];
                }
                nq--;
            }
            if (st_pending) {                          /* the STATE reaches the GNS in the NAV's tick t + 1 */
                int got = 0;
                for (i = 0; i < (int)GL_STATE_FRAME_BYTES; i++)
                    got += gns_rx_byte(&G, st_buf[i], res_buf) ? 1 : 0;
                dec_bad += got != 1;            /* one RESULT per STATE */
                res_pending = true;
                st_pending = false;
                (void)gns_finish(&G);
            }
        }
        /* measurements */
        if (so.phase == LP_PHASE_HOLD && t_hold > 0 && (int)t > t_hold + 30 && k.grx.has_latest &&
            (k.grx.latest.pc & NL_PC_MODE_MASK) != POS_MODE_POS)
            not_pos++;
        if (t_hold > 0 && (int)t == t_hold + 30) {
            hold[0] = s.p[0];
            hold[1] = s.p[1];
        }
        if (t_hold > 0 && (int)t > t_hold + 30 && t_push < 0) {
            const double dd = hypot(s.p[0] - hold[0], s.p[1] - hold[1]);
            if (dd > max_d_hold) max_d_hold = dd;
        }
        if (t_push > 0 && (int)t > t_push && (int)t <= t_push + 1250) {
            const double dd = hypot(s.p[0] - hold[0], s.p[1] - hold[1]);
            if (dd > max_d_push) max_d_push = dd;
            if ((int)t == t_push + 1250) d_back = dd;
        }
        if ((int)t == t_landed) land_d = hypot(s.p[0] - hold[0], s.p[1] - hold[1]);
    }
    CHECK(dec_bad == 0, "every NAV4 / FCU4 exchange and every STATE -> RESULT decoded (%d failures)", dec_bad);
    CHECK(n_rej == 0 && t_hold > t_on && t_landed > t_off && t_off > 0 && t_safe > t_landed, "GNS-chain hop: ON (no reject) "
          "%d, HOLD %d, OFF %d, LANDED %d, SAFE %d", t_on, t_hold, t_off, t_landed, t_safe);
    CHECK(so.reason == LP_REASON_NONE && not_pos == 0 && k.grx.broken == 0 && k.bg_late == 0 && G.st.gaps == 0 &&
          G.st.restarts == 0 && N.rx.stats.bad_crc == 0 && G.dec.stats.bad_crc == 0, "no interlock, POS mode through HOLD, no "
          "broken slot set (%u sets), no glink error", k.grx.sets);
    CHECK(N.tx.missed <= 3u && N.tx.latched > 1000u, "the NAV latched %u results, missed %u (only before the first answer)",
          N.tx.latched, N.tx.missed);
    CHECK(max_d_hold < 0.5 && max_d_push > 0.3 && d_back < 0.5 && land_d >= 0.0 && land_d < 1.0 && max_tilt < 0.36,
          "hold within %.2f m, a 3 N s push displaces %.2f m and is back to %.2f m after 5 s, touchdown %.2f m from the hold "
          "point, max tilt %.1f deg", max_d_hold, max_d_push, d_back, land_d, max_tilt * 57.2958);
    printf("info: C four-chip closed loop (NAV glink + GNS node + FCU GNS chain, point mass, attitude lag 0.15 s, GPS 0.1 m "
           "noise, NOT the 6-DOF): HOLD at %.2f s, hold within %.3f m, 3 N s push max %.2f m (back to %.2f m in 5 s), touchdown "
           "%.2f m from the hold point at %.2f s, max tilt %.1f deg; GNS %u cycles, %u epochs fused; FCU %u GNS sets, NAV "
           "latched %u / missed %u\n", t_hold * dt, max_d_hold, max_d_push, d_back, land_d, t_landed * dt, max_tilt * 57.2958,
           G.cycles, G.gps.filter.st_fused, k.grx.sets, N.tx.latched, N.tx.missed);
    {   /* the latency of the chain: a STATE sampled at k reaches the attitude controller at k + L + 1 (one GNS cycle) */
        static gl_nav_t n2;
        uint8_t sl[NL_SLOT_LEN];
        const uint8_t r20[20] = {0x10, 0x00, 0x20, 0x00, 0x2F, 9, 25, 0x0A, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        uint32_t kk, first = 0;
        gl_nav_init(&n2, 5, GLINK_LATENCY_TICKS);
        for (kk = 0; kk < 40u; kk++) {
            if (kk == 25u)                             /* the RESULT of the sample at tick 24 (cycle 4) arrives at 25 */
                nl_gns_tx_result(&n2.tx, 4, r20, kk);
            gl_nav_slot(&n2, kk, sl);
            if (sl[0] == ((4u << 3) | NL_SLOT4_TILT) && first == 0u)
                first = kk;
        }
        CHECK(first == 24u + GLINK_LATENCY_TICKS, "the TILT part of the sample at tick 24 rides in the frame of tick %lu = k + L "
              "(%u); the FCU's builder takes it that tick, the attitude controller the next (k + L + 1)", (unsigned long)first,
              GLINK_LATENCY_TICKS);
    }
}

/* ---- v0.8.1 vector modes (tools/crosscheck_controller.py sections 9a / 9b) ---- */
static void put_hex(FILE *fo, const uint8_t *b, int n)
{
    int i;
    for (i = 0; i < n; i++) fprintf(fo, "%02x", b[i]);
}

/* the next space-separated token of *p (NUL-terminated in place); NULL at the end */
static char *next_tok(char **p)
{
    char *s = *p, *t;
    while (*s == ' ' || *s == '\t') s++;
    if (*s == '\0' || *s == '\n' || *s == '\r') return 0;
    t = s;
    while (*s && *s != ' ' && *s != '\t' && *s != '\n' && *s != '\r') s++;
    if (*s) *s++ = '\0';
    *p = s;
    return t;
}

/* glink vectors (sim/glink_proto.py, sim/nlink_proto.py v4):
 *   E type seq payloadhex        -> E <frame hex>                    (gl_encode)
 *   X hex                        -> X reason valid type seq          (gl_reject_reason / gl_decode_frame)
 *   D accept hex                 -> D [type:seq:payloadhex ...] S frames bad_crc bad_type wrong_dir skipped
 *   T L tok tok ...              -> T slot slot ... S latched missed  (tok per NAV tick: '-' or seq:payloadhex[;...] = the
 *                                   RESULT frames decoded in that tick, before its frame build)
 *   W land_steps tok tok ...     -> W per tick got:eng:t0:t1:fallback:requests:gate:fresh[:flags:sv:hacc:pc:pos0:pos1:vel0:
 *                                   vel1:trim0:trim1:cycle:dist] ... S sets broken   (tok '-': no valid frame, else the slot hex)
 *   F seq state fflags tag thr pflags kt   -> F <72-byte hex>        (nl_encode_fcu4)
 *   Y hex72                      -> Y nav4 fcu4 nav3 fcu reenc [seq state fflags tag thr pflags kt]
 *   #N L                         -> (no output) gl_nav_init(5, L)
 *   n k fb0 fb1 fb2 qb0..qb3 wb0..wb2 hb pad has_fcu fcu_valid state pflags kt imu est  (20 numbers; floats as bits)
 *                                -> n <STATE frame hex | -> */
static int run_glink_vectors(const char *in, const char *out)
{
    FILE *fi = fopen(in, "r"), *fo = fopen(out, "w");
    static gl_nav_t nav;
    static uint8_t buf[1u << 15];
    long rows = 0;
    if (!fi || !fo) return 2;
    gl_nav_init(&nav, 5, 3);
    while (fgets(g_line, sizeof g_line, fi)) {
        char *p = g_line + 1, *t;
        const char kind = g_line[0];
        if (kind == '\n' || kind == '\r') continue;
        if (kind == 'E') {
            uint8_t fr[GL_FRAME_MAX_BYTES], pl[GL_STATE_LEN];
            const int ty = (int)strtol(next_tok(&p), 0, 10), sq = (int)strtol(next_tok(&p), 0, 10);
            (void)parse_hex(next_tok(&p), pl, (int)sizeof pl);
            fputs("E ", fo);
            put_hex(fo, fr, gl_encode((uint8_t)ty, (uint8_t)sq, pl, fr));
        } else if (kind == 'X') {
            const int n = parse_hex(next_tok(&p), buf, (int)sizeof buf);
            uint8_t ty = 0, sq = 0;
            const uint8_t ok = gl_decode_frame(buf, (uint16_t)n, &ty, &sq);
            fprintf(fo, "X %u %u %u %u", gl_reject_reason(buf, (uint16_t)n), ok, ok ? ty : 0u, ok ? sq : 0u);
        } else if (kind == 'D') {
            gl_decoder_t d;
            const int acc = (int)strtol(next_tok(&p), 0, 10);
            const int n = parse_hex(next_tok(&p), buf, (int)sizeof buf);
            int i;
            gl_decoder_init(&d, (uint8_t)acc);
            fputc('D', fo);
            for (i = 0; i < n; i++) {
                const uint8_t ty = gl_decoder_byte(&d, buf[i]);
                if (ty) {
                    fprintf(fo, " %u:%u:", ty, d.seq);
                    put_hex(fo, d.buf, ty == GL_TYPE_STATE ? (int)GL_STATE_LEN : (int)GL_RESULT_LEN);
                }
            }
            fprintf(fo, " S %u %u %u %u %u", d.stats.frames, d.stats.bad_crc, d.stats.bad_type, d.stats.wrong_dir,
                    d.stats.skipped);
        } else if (kind == 'T') {
            nl_gns_tx_t tx;
            uint32_t k = 0;
            uint8_t sl[NL_SLOT_LEN];
            nl_gns_tx_init(&tx, 5, (uint8_t)strtol(next_tok(&p), 0, 10));
            fputc('T', fo);
            while ((t = next_tok(&p)) != 0) {
                if (*t != '-') {
                    char *e = t;
                    while (e && *e) {
                        uint8_t pl[NL_GNS_RESULT_LEN];
                        char *semi = strchr(e, ';');
                        const int sq = (int)strtol(e, 0, 10);
                        char *col = strchr(e, ':');
                        if (semi) *semi = '\0';
                        (void)parse_hex(col + 1, pl, (int)sizeof pl);
                        nl_gns_tx_result(&tx, (uint8_t)sq, pl, k);
                        e = semi ? semi + 1 : 0;
                    }
                }
                nl_gns_tx_slot(&tx, k, sl);
                fputc(' ', fo);
                put_hex(fo, sl, NL_SLOT_LEN);
                k++;
            }
            fprintf(fo, " S %u %u", tx.latched, tx.missed);
        } else if (kind == 'W') {
            nl_gns_rx_t rx;
            const uint16_t land = (uint16_t)strtol(next_tok(&p), 0, 10);
            nl_gns_rx_init(&rx, 5);
            fputc('W', fo);
            while ((t = next_tok(&p)) != 0) {
                uint8_t sl[NL_SLOT_LEN];
                const nl_gns_data_t *g;
                float tg[2];
                bool eng;
                if (*t != '-') (void)parse_hex(t, sl, NL_SLOT_LEN);
                g = nl_gns_rx_feed(&rx, *t != '-' ? sl : 0);
                eng = nl_gns_rx_target(&rx, tg);
                fprintf(fo, " %d:%d:", g ? 1 : 0, eng ? 1 : 0);
                put_f(fo, tg[0]); fputc(':', fo);
                put_f(fo, tg[1]);
                fprintf(fo, ":%d:%u:%d:%d", nl_gns_rx_fallback(&rx) ? 1 : 0, nl_gns_rx_requests(&rx, land),
                        nl_gns_rx_gate_ok(&rx) ? 1 : 0, nl_gns_rx_fresh(&rx) ? 1 : 0);
                if (g) {
                    fprintf(fo, ":%u:%u:%u:%u:%d:%d:%d:%d:%d:%d:%u:", g->flags, g->num_sv, g->hacc_dm, g->pc, g->pos[0],
                            g->pos[1], g->vel[0], g->vel[1], g->trim[0], g->trim[1], g->cycle);
                    put_f(fo, g->dist_m);
                }
            }
            fprintf(fo, " S %u %u", rx.sets, rx.broken);
        } else if (kind == 'F') {
            nl_fcu4_t f;
            uint8_t x[NL_XFER_BYTES];
            f.seq = (uint8_t)strtol(next_tok(&p), 0, 10);
            f.state = (uint8_t)strtol(next_tok(&p), 0, 10);
            f.fflags = (uint8_t)strtol(next_tok(&p), 0, 10);
            f.params_tag = (uint8_t)strtol(next_tok(&p), 0, 10);
            f.thr_q = (uint8_t)strtol(next_tok(&p), 0, 10);
            f.pflags = (uint8_t)strtol(next_tok(&p), 0, 10);
            f.kt_q = (uint16_t)strtol(next_tok(&p), 0, 10);
            (void)nl_encode_fcu4(&f, x);
            fputs("F ", fo);
            put_hex(fo, x, NL_XFER_BYTES);
        } else if (kind == 'Y') {
            uint8_t x[NL_XFER_BYTES], y[NL_XFER_BYTES];
            nl_nav3_t n;
            nl_fcu4_t f;
            nl_fcu_t f3;
            int a, b, c, d, re = 0;
            (void)parse_hex(next_tok(&p), x, NL_XFER_BYTES);
            a = nl_decode_nav4(x, &n);
            if (a) re = nl_encode_nav4(&n, y) == NL_XFER_BYTES && memcmp(x, y, NL_XFER_BYTES) == 0;
            b = nl_decode_fcu4(x, &f);
            c = nl_decode_nav3(x, &n);
            d = nl_decode_fcu(x, &f3);
            fprintf(fo, "Y %d %d %d %d %d", a, b, c, d, re);
            if (b) fprintf(fo, " %u %u %u %u %u %u %u", f.seq, f.state, f.fflags, f.params_tag, f.thr_q, f.pflags, f.kt_q);
        } else if (kind == '#') {
            if (g_line[1] == 'N') {
                p = g_line + 2;
                gl_nav_init(&nav, 5, (uint8_t)strtol(next_tok(&p), 0, 10));
            }
            continue;
        } else if (kind == 'n') {
            double v[20];
            float a[3], q[4], w[3];
            nl_fcu4_t f;
            uint8_t fr[GL_STATE_FRAME_BYTES];
            int i;
            if (parse_numbers(g_line + 1, v, 20) != 20) {
                fprintf(stderr, "glink-vectors: short n row %ld\n", rows);
                return 3;
            }
            for (i = 0; i < 3; i++) a[i] = bits_f((uint32_t)v[1 + i]);
            for (i = 0; i < 4; i++) q[i] = bits_f((uint32_t)v[4 + i]);
            for (i = 0; i < 3; i++) w[i] = bits_f((uint32_t)v[8 + i]);
            memset(&f, 0, sizeof f);
            f.state = (uint8_t)v[15];
            f.pflags = (uint8_t)v[16];
            f.kt_q = (uint16_t)v[17];
            fputs("n ", fo);
            if (gl_nav_tick(&nav, (uint32_t)v[0], a, q, w, bits_f((uint32_t)v[11]), v[12] != 0.0, v[13] != 0.0 ? &f : 0,
                            v[14] != 0.0, v[18] != 0.0, v[19] != 0.0, fr))
                put_hex(fo, fr, GL_STATE_FRAME_BYTES);
            else
                fputc('-', fo);
        } else {
            continue;
        }
        fputc('\n', fo);
        rows++;
    }
    fclose(fi);
    fclose(fo);
    fprintf(stderr, "glink-vectors: %ld rows\n", rows);
    return 0;
}

/* GNS node vectors (sim/glink_proto.py GnsNode; tools/crosscheck_controller.py check_gns):
 *   #N id configure gp.<k>=v ... pp.<k>=v ...   a fresh node (fc_default_gps_nav / fc_default_position + overrides,
 *                                                 the configurator's tick counts / baud of params_gen.h)
 *   P k hex|-     -> P baud txhex|-             gns_poll(k, the UART bytes)
 *   S hex         -> S - | S result <filter> <pos> <node> <dec>   the glink bytes; after a RESULT the cycle's fusion
 *                    (gns_finish) and the state:  filter = init pad acq_left reset_count cycle yaw_used fused rej_p rej_v
 *                    pos_resets vel_resets dropped_old dropped_stale n_epoch_dropped unusable timebase_restarts p0 p1 v0 v1
 *                    b0 b1 off0 off1 c_psi s_psi p_psi yaw_corr has_offset offset relearned;  pos = n engaged has_hold mode
 *                    requests lost guard_left implausible lim_run vexc_run tl0 tl1 tr0 tr1 i0 i1 hold0 hold1;  node = states
 *                    hold_steps gaps restarts imu_bad clock_slips misaligned cycles nt;  dec = frames bad_crc bad_type
 *                    wrong_dir skipped */
static int run_gns_vectors(const char *in, const char *out)
{
    FILE *fi = fopen(in, "r"), *fo = fopen(out, "w");
    static gns_t g;
    static uint8_t buf[1u << 15];
    long rows = 0;
    if (!fi || !fo) return 2;
    fc_default_gns(&g, false);
    while (fgets(g_line, sizeof g_line, fi)) {
        char *p = g_line + 1, *t;
        const char kind = g_line[0];
        if (kind == '#' && g_line[1] == 'N') {
            gps_nav_params_t gp;
            pos_params_t pp;
            int configure;
            p = g_line + 2;
            (void)next_tok(&p);                           /* id */
            configure = (int)strtol(next_tok(&p), 0, 10);
            fc_default_gps_nav(&gp);
            fc_default_position(&pp);
            while ((t = next_tok(&p)) != 0) {
                if (!gp_override(&gp, t) && !pos_override(&pp, t)) {
                    fprintf(stderr, "gns-vectors: unknown override %s\n", t);
                    return 3;
                }
            }
            gns_init(&g, &gp, &pp, configure != 0, GPS_CFG_N_WAIT, GPS_CFG_N_ACK, GPS_CFG_N_SETTLE, GPS_CFG_N_VER, GPS_BAUD);
            continue;
        }
        if (kind == 'P') {
            gps_cfg_action_t act;
            const uint32_t k = (uint32_t)strtoul(next_tok(&p), 0, 10);
            int n = 0;
            t = next_tok(&p);
            if (t && *t != '-') n = parse_hex(t, buf, (int)sizeof buf);
            gns_poll(&g, k, buf, (uint16_t)n, &act);
            fprintf(fo, "P %lu ", (unsigned long)act.baud);
            if (act.tx != GPS_TX_NONE) {
                uint8_t fr[UBX_FRAME_MAX];
                put_hex(fo, fr, gps_cfg_frame(act.tx, g.gps.cfg.baud, GPS_MEAS_MS, fr));
            } else {
                fputc('-', fo);
            }
        } else if (kind == 'S') {
            uint8_t res[GL_RESULT_FRAME_BYTES];
            int n = 0, i, got = 0;
            t = next_tok(&p);
            if (t && *t != '-') n = parse_hex(t, buf, (int)sizeof buf);
            fputc('S', fo);
            for (i = 0; i < n; i++) {
                if (gns_rx_byte(&g, buf[i], res)) {
                    const gps_filter_t *f = &g.gps.filter;
                    const pos_t *s = &g.pos;
                    const pos_pub_t *po = &s->last;
                    (void)gns_finish(&g);
                    fputs(got ? " | " : " ", fo);
                    put_hex(fo, res, GL_RESULT_FRAME_BYTES);
                    fprintf(fo, " %d %d %u %u %u %u %u %u %u %u %u %u %u %u %u %u ", f->init ? 1 : 0, f->pad ? 1 : 0,
                            f->acq_left, f->reset_count, f->cycle, f->yaw_used, f->st_fused, f->st_rej_p, f->st_rej_v,
                            f->st_pos_resets, f->st_vel_resets, f->st_dropped_old, f->st_dropped_stale, f->st_n_epoch_dropped,
                            f->st_unusable, f->st_timebase_restarts);
                    put_f(fo, f->p_[0]); fputc(' ', fo);
                    put_f(fo, f->p_[1]); fputc(' ', fo);
                    put_f(fo, f->v[0]); fputc(' ', fo);
                    put_f(fo, f->v[1]); fputc(' ', fo);
                    put_f(fo, f->b[0]); fputc(' ', fo);
                    put_f(fo, f->b[1]); fputc(' ', fo);
                    put_f(fo, f->off[0]); fputc(' ', fo);
                    put_f(fo, f->off[1]); fputc(' ', fo);
                    put_f(fo, f->c_psi); fputc(' ', fo);
                    put_f(fo, f->s_psi); fputc(' ', fo);
                    put_f(fo, f->p_psi); fputc(' ', fo);
                    put_f(fo, f->yaw_corr);
                    fprintf(fo, " %d %ld %u", g.gps.clock.has_offset ? 1 : 0, (long)g.gps.clock.offset, g.gps.clock.relearned);
                    fprintf(fo, " %u %d %d %u %u %u %u %d %u %u ", s->n, s->engaged ? 1 : 0, s->has_hold ? 1 : 0, po->mode,
                            po->requests, s->lost, s->guard_left, s->implausible ? 1 : 0, s->lim_run, s->vexc_run);
                    put_f(fo, po->tilt_l[0]); fputc(' ', fo);
                    put_f(fo, po->tilt_l[1]); fputc(' ', fo);
                    put_f(fo, po->trim_l[0]); fputc(' ', fo);
                    put_f(fo, po->trim_l[1]); fputc(' ', fo);
                    put_f(fo, s->integ[0]); fputc(' ', fo);
                    put_f(fo, s->integ[1]); fputc(' ', fo);
                    put_f(fo, s->hold_rf[0]); fputc(' ', fo);
                    put_f(fo, s->hold_rf[1]);
                    fprintf(fo, " %u %u %u %u %u %u %u %u %lu", g.st.states, g.st.hold_steps, g.st.gaps, g.st.restarts,
                            g.st.imu_bad, g.st.clock_slips, g.st.misaligned, g.cycles, (unsigned long)g.nt);
                    got++;
                }
            }
            if (!got) fputs(" -", fo);
            fprintf(fo, " D %u %u %u %u %u", g.dec.stats.frames, g.dec.stats.bad_crc, g.dec.stats.bad_type,
                    g.dec.stats.wrong_dir, g.dec.stats.skipped);
        } else {
            continue;
        }
        fputc('\n', fo);
        rows++;
    }
    fclose(fi);
    fclose(fo);
    fprintf(stderr, "gns-vectors: %ld rows\n", rows);
    return 0;
}

/* fw09 X1 (--consts): the constants of THIS build (-D FC_VCFG / -D MISSION_PROFILE_*) as the firmware's structs hold
 * them after fc_config.c filled them - "<key> <value>" lines, floats with 9 significant digits (exact float32 text).
 * tools/crosscheck_controller.py reads them for the per-configuration plant check and compares them, before any 6-DOF
 * replay, with the parameters the 6-DOF ran with (a firmware build that does not carry its configuration's vehicle
 * constants - the G1 defect of SPEC_firmware_v09 3.2 - fails there).  Keys: meta.*, macro.*, plant.*, gains.*, sup.*,
 * vc.*, pp.*, mah.*, altv.* (the names of the override tokens of this file where those exist). */
static int run_consts(const char *out)
{
    FILE *fo = fopen(out, "w");
    mixer_plant_t mp;
    att_gains_t ag;
    att_mahony_t mh;
    alt_vtol_t av;
    const vc_params_t *vp = fc_vertical_params();
    const onb_params_t *op = fc_onboard_params();
    const pos_params_t *pp = fc_position_params();
    if (!fo) return 2;
    fc_default_plant(&mp);
    fc_vtol_gains(&ag);
    fc_default_mahony(&mh);
    fc_default_alt_vtol(&av, BARO_LATENCY_S);
#define CF(key, x) fprintf(fo, "%s %.9g\n", key, (double)(x))
#define CI(key, x) fprintf(fo, "%s %ld\n", key, (long)(x))
#ifdef FC_VEHICLE
    CI("meta.vehicle", FC_VEHICLE);
#else
    CI("meta.vehicle", -1);                            /* a header without per-configuration constants (before G1) */
#endif
#ifdef FC_VEHICLE_NAME
    fprintf(fo, "meta.vehicle_name %s\n", FC_VEHICLE_NAME);
#else
    fprintf(fo, "meta.vehicle_name -\n");
#endif
#ifdef FC_VCFG
    CI("meta.fc_vcfg", FC_VCFG);
#else
    CI("meta.fc_vcfg", -1);
#endif
    fprintf(fo, "meta.profile %s\n", MISSION_PROFILE_NAME);
    fprintf(fo, "meta.params_sha256_prefix %s\n", FC_PARAMS_SHA256_PREFIX);
    CI("meta.params_tag", FC_PARAMS_TAG);
    CI("meta.loop_hz", FC_LOOP_HZ);
    CF("macro.FC_HOVER_THROTTLE", FC_HOVER_THROTTLE);
    CF("macro.VEH_MASS_KG", VEH_MASS_KG);
    CF("macro.VEH_Z_CG_M", VEH_Z_CG_M);
    CF("macro.VEH_IXX_KG_M2", VEH_IXX_KG_M2);
    CF("macro.VEH_IYY_KG_M2", VEH_IYY_KG_M2);
    CF("macro.VEH_IZZ_KG_M2", VEH_IZZ_KG_M2);
    CF("macro.BARO_LATENCY_S", BARO_LATENCY_S);
    CI("macro.FC_BARO_SENSOR", FC_BARO_SENSOR);
#define PF_(f) CF("plant." #f, mp.f)
    PF_(l_v_m); PF_(r_v_m); PF_(s_v_m2); PF_(cl_alpha); PF_(a_e_m2); PF_(t_max_n); PF_(throttle_exp); PF_(delta_max_rad);
    PF_(pwm_center_us); PF_(pwm_us_per_deg);
#undef PF_
#define GF_(f) CF("gains." #f, ag.f)
#define GR_(f) CF("gains." #f, ag.rate.f)
    CI("gains.loop_hz", ag.loop_hz);
    GF_(att_kp_xy); GF_(att_kp_z); GF_(rate_sp_max_xy_rad_s); GF_(rate_sp_max_z_rad_s);
    GR_(rate_kp_xy); GR_(rate_ki_xy); GR_(rate_kd_xy); GR_(rate_i_max_xy); GR_(rate_kp_z); GR_(rate_ki_z); GR_(rate_kd_z);
    GR_(rate_i_max_z); GR_(d_term_lpf_hz); GR_(gyro_lpf_hz);
    GF_(thrust_estimate_floor_fraction); GF_(roll_priority_scale);
    CI("gains.yaw_hold", ag.yaw_hold ? 1 : 0);
#if FC_V09
    CI("gains.v9.err_mode", ag.v9.err_mode); CI("gains.v9.flags", ag.v9.flags); CF("gains.v9.ff_a", ag.v9.ff_a);
    CF("gains.v9.ff_nkr", ag.v9.ff_nkr); CF("gains.v9.ff_h", ag.v9.ff_h);
#endif
#undef GF_
#undef GR_
#define SU_(f) CI("sup." #f, op->f)
#define SF_(f) CF("sup." #f, op->f)
    {   /* the flight limits: an AVR power image's flash table (core/fc_config.c, ONB_PARAMS_FLASH: FC_V_* = ONB_W_* when
         * PWR_SRC != 0) holds the tethered ones; the host fill (fc_default_onboard) keeps the profile's for the unit
         * tests - with -D PWR_SRC print what the AVR image holds (params_gen.h ONB_W_*) */
        long hover = (long)op->hover_dm, amin = (long)op->alt_min_dm, amax = (long)op->alt_max_dm;
        long endu = (long)op->endurance_ticks, hard = (long)op->hard_cutoff_ticks;
        double ceil_m = (double)op->ceiling_m, geo_m = (double)op->v_geofence_m;
#if defined(PWR_SRC) && PWR_SRC != 0 && defined(ONB_W_HOVER_DM)
        hover = ONB_W_HOVER_DM; amin = ONB_W_ALT_MIN_DM; amax = ONB_W_ALT_MAX_DM; endu = ONB_W_ENDURANCE_TICKS;
        hard = ONB_W_HARD_CUTOFF_TICKS; ceil_m = (double)(float)ONB_W_CEILING_M; geo_m = (double)(float)ONB_W_GEOFENCE_M;
#endif
        CI("sup.hover_dm", hover); CI("sup.alt_min_dm", amin); CI("sup.alt_max_dm", amax); CI("sup.endurance_ticks", endu);
        CI("sup.hard_cutoff_ticks", hard); CF("sup.ceiling_m", ceil_m); CF("sup.v_geofence_m", geo_m);
    }
    SU_(ramp_ticks); SU_(coast_hold_ticks); SU_(fall_confirm_ticks); SU_(link_abort_ticks); SU_(link_disarm_ticks);
    SU_(max_motor_ticks); SU_(release_after_cut_ticks); SU_(abort_release_ticks); SU_(rate_persist_ticks);
    SU_(imu_fail_ticks); SU_(baro_fail_ticks);
    SF_(tilt_abort_deg); SF_(arm_tilt_max_deg); SF_(min_cell_v); SF_(geofence_m); SF_(apogee_limit_m); SF_(fall_vz_mps);
    SU_(spool_ticks); SU_(link_land_ticks); SU_(v_fall_confirm_ticks);
    SU_(safe_after_landed_ticks); SU_(alt_step_dm);
    SF_(v_fall_vz_mps);
    SU_(land_timeout_ticks); SU_(baro_limit_base_ticks); SU_(baro_limit_mt_per_dm); SU_(v_geofence_dm); SU_(climb_milli);
    CI("sup.gyro_saturation_abort", op->gyro_saturation_abort ? 1 : 0);
    SU_(mode);
    CI("sup.gps_hold", op->gps_hold ? 1 : 0);
    CI("sup.takeoff_check", op->takeoff_check ? 1 : 0);
#if PM_ANY
    CI("sup.pw.build", op->pw.build); CI("sup.pw.shift", op->pw.shift);
    CI("sup.pw.arm_lo_q", op->pw.arm_lo_q); CI("sup.pw.arm_hi_q", op->pw.arm_hi_q);
    CI("sup.pw.buf_arm_lo_q", op->pw.buf_arm_lo_q); CI("sup.pw.buf_arm_hi_q", op->pw.buf_arm_hi_q);
    CI("sup.pw.lost_q", op->pw.lost_q); CI("sup.pw.lost_margin_q", op->pw.lost_margin_q);
    CI("sup.pw.sag_q", op->pw.sag_q); CI("sup.pw.sag_clear_q", op->pw.sag_clear_q);
    CI("sup.pw.buf_low_q", op->pw.buf_low_q); CI("sup.pw.buf_res_q", op->pw.buf_res_q); CI("sup.pw.d2_q", op->pw.d2_q);
    CI("sup.pw.i8buf_q", op->pw.i8buf_q); CI("sup.pw.sag_warn_ticks", op->pw.sag_warn_ticks);
    CI("sup.pw.sag_land_ticks", op->pw.sag_land_ticks); CI("sup.pw.buf_res_ticks", op->pw.buf_res_ticks);
    CI("sup.pw.lost_ticks", op->pw.lost_ticks); CI("sup.pw.buf_ticks", op->pw.buf_ticks);
    CI("sup.pw.d2_ticks", op->pw.d2_ticks); CI("sup.pw.sag_release_ticks", op->pw.sag_release_ticks);
#endif
#undef SU_
#undef SF_
#define VF_(f) CF("vc." #f, vp->f)
#define VI_(f) CI("vc." #f, vp->f)
#if FC_V09
    VI_(ramp_total_ticks); VI_(approach_min_um); VF_(to_u_start); VF_(to_s1); VF_(to_s2); VF_(to_search_f);
    VF_(to_search_vz); VF_(kt_alpha_to); VF_(to_airborne_mps2); VF_(td_step); VF_(td_max); VF_(td_ok); VI_(v9);
    VI_(approach_shift);
#endif
    VI_(up_um); VI_(dn_um); VI_(td_um); VI_(flare_um); VF_(takeoff_climb_m); VI_(takeoff_ticks); VI_(takeoff_settle_ticks);
    VF_(um_tick_to_mps); VF_(flare_altitude_m); VF_(alt_kp); VF_(vz_kp); VF_(vz_ki_dt); VF_(vz_i_max); VF_(vz_max_up_mps);
    VF_(vz_max_down_mps); VF_(u_min); VF_(u_max); VF_(u_hover); VF_(inv_u_hover); VF_(throttle_exp);
    VF_(cos_tilt_min); VF_(td_vz_still_mps); VF_(lead_mps); VF_(support_mps2); VF_(g); VF_(thrust_alpha);
    VF_(still_alpha); VF_(support_alpha); VF_(kt_alpha); VF_(kt_min); VF_(kt_max); VF_(kt_cos_min);
    VI_(ramp_ticks); VI_(landed_confirm_ticks); VI_(floor_ticks); VI_(throttle_exp_int);
    CI("vc.vz_ff", vp->vz_ff ? 1 : 0);
#undef VF_
#undef VI_
#define PPF_(f) CF("pp." #f, pp->f)
#define PPI_(f) CI("pp." #f, pp->f)
    PPI_(step_ticks); PPF_(T); PPF_(g); PPF_(tw); PPF_(kp_pos); PPF_(kv); PPF_(ki); PPF_(kd); PPF_(alpha_d); PPF_(i_max);
    PPF_(v_max); PPF_(v_max_land); PPF_(flare_speed); PPF_(flare_altitude_m); PPF_(cos_cap); PPF_(cos_flare_cap);
    PPF_(cos_excess); PPF_(sin_excess); PPF_(margin_g); PPF_(margin_frac); PPF_(h_enable_m); PPF_(fence_m);
    PPF_(spin_z_rad); PPF_(spin_xy_rad2); PPI_(guard_steps); PPI_(timeout_steps); PPI_(land_steps);
    PPI_(plaus_limited_steps); PPI_(plaus_vexc_steps); PPI_(plaus_reset_window_steps); PPI_(plaus_resets);
    PPF_(plaus_vt); PPF_(authority_min_ktw); PPF_(tilt_fade_m); PPF_(slew_per_tick); PPI_(keep_trim);
#undef PPF_
#undef PPI_
    CF("mah.dt", mh.pad.dt); CI("mah.n_avg", mh.pad.n_avg); CF("mah.kp", mh.kp); CF("mah.ki_dt", mh.ki_dt);
    CF("mah.gate_mps2", mh.gate_mps2); CF("mah.g", mh.g); CF("mah.alpha_a", mh.alpha_a); CF("mah.bias_limit", mh.bias_limit);
    CF("altv.k1", av.p.k1); CF("altv.k2", av.p.k2); CF("altv.k3", av.p.k3); CF("altv.latency_s", av.p.latency_s);
    CF("altv.tick_dt", av.p.tick_dt); CI("altv.pad_n_avg", av.p.pad_n_avg); CF("altv.bias_limit_mps2", av.p.bias_limit_mps2);
    CF("altv.comp_m", av.p.comp_m); CF("altv.inv_u_hover", av.p.inv_u_hover); CI("altv.throttle_exp_int", av.p.throttle_exp_int);
    CF("altv.throttle_exp", av.p.throttle_exp);
#undef CF
#undef CI
    fclose(fo);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc == 3 && strcmp(argv[1], "--consts") == 0) return run_consts(argv[2]);
    if (argc == 4 && strcmp(argv[1], "--vectors") == 0) return run_vectors(argv[2], argv[3]);
    if (argc == 4 && strcmp(argv[1], "--vectors-vtol") == 0) return run_vectors_vtol(argv[2], argv[3]);
    if (argc == 4 && strcmp(argv[1], "--onb-vectors") == 0) return run_onb_vectors(argv[2], argv[3]);
    if (argc == 4 && strcmp(argv[1], "--stack-vectors") == 0) return run_stack_vectors(argv[2], argv[3]);
    if (argc == 4 && strcmp(argv[1], "--vc-vectors") == 0) return run_vc_vectors(argv[2], argv[3]);
    if (argc == 4 && strcmp(argv[1], "--telem4-vectors") == 0) return run_telem4_vectors(argv[2], argv[3]);
#if PM_ANY
    if (argc == 4 && strcmp(argv[1], "--pm-vectors") == 0) return run_pm_vectors(argv[2], argv[3]);
#endif
    if (argc == 4 && strcmp(argv[1], "--proto-vectors") == 0) return run_proto_vectors(argv[2], argv[3]);
    if (argc == 4 && strcmp(argv[1], "--ilink-vectors") == 0) return run_ilink_vectors(argv[2], argv[3]);
    if (argc == 4 && strcmp(argv[1], "--nlink-vectors") == 0) return run_nlink_vectors(argv[2], argv[3]);
    if (argc == 4 && strcmp(argv[1], "--est-vectors") == 0) return run_est_vectors(argv[2], argv[3]);
    if (argc == 4 && strcmp(argv[1], "--alt-vectors") == 0) return run_alt_vectors(argv[2], argv[3]);
    if (argc == 4 && strcmp(argv[1], "--mahony-vectors") == 0) return run_mahony_vectors(argv[2], argv[3]);
    if (argc == 4 && strcmp(argv[1], "--altv-vectors") == 0) return run_altv_vectors(argv[2], argv[3]);
    if (argc == 4 && strcmp(argv[1], "--pos-vectors") == 0) return run_pos_vectors(argv[2], argv[3]);
    if (argc == 4 && strcmp(argv[1], "--gpsnav-vectors") == 0) return run_gpsnav_vectors(argv[2], argv[3]);
    if (argc == 4 && strcmp(argv[1], "--glink-vectors") == 0) return run_glink_vectors(argv[2], argv[3]);
    if (argc == 4 && strcmp(argv[1], "--gns-vectors") == 0) return run_gns_vectors(argv[2], argv[3]);
    if (argc == 3 && strcmp(argv[1], "--log-sample") == 0) return write_log_sample(argv[2]);
    if (argc != 1) {
        fprintf(stderr, "usage: %s [--vectors|--vectors-vtol|--onb-vectors|--stack-vectors|--vc-vectors|--proto-vectors|--ilink-vectors|"
                        "--nlink-vectors|--est-vectors|--alt-vectors|--mahony-vectors|--altv-vectors in out | "
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
    test_onboard_vtol_table();             /* v0.7 VTOL: after every v0.6.3 test (their random sequences unchanged) */
    test_onboard_vtol_rules();
    test_vertical_ctrl();
    test_stack();
    test_mahony();                         /* v0.7.1 VTOL NAV estimator and altitude filter */
    test_alt_vtol();
    test_vc_thrust_learning();             /* v0.7.1b (no random draws) */
    test_vtol_gains();
    test_gps_ubx();                        /* v0.8 GPS position hold */
    test_nlink_v3();
    test_vc_takeoff();
    test_att_tilt();
    test_predict();
    test_gps_filter();
    test_position();
    test_onboard_v08();
    test_stack_v08();
    test_quantize_wide();                  /* v0.8.1 four-chip build (GNS board, glink, nlink v4, keep_trim) */
    test_glink();
    test_nlink_v4();
    test_gns_replay();
    test_position_v081();
    test_gns_chain();
    printf("test_core: %d checks, %d failures\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
