/* fc_config.c - see fc_config.h */
#include "fc_config.h"
#include "nlink_proto.h"

#include "../gains.h"
#include "../params_gen.h"

#include <string.h>

#if GAIN_LOOP_HZ != FC_LOOP_HZ
#error "gains.h and params_gen.h disagree on loop_hz - re-run both generators"
#endif
#if !defined(MISSION_MODE) || MISSION_MODE_BALLISTIC != LP_MODE_BALLISTIC || MISSION_MODE_VTOL != LP_MODE_VTOL
#error "params_gen.h lacks MISSION_MODE or its values differ from link_proto.h LP_MODE_* - re-run gen_firmware_params.py"
#endif
#if MISSION_MODE == MISSION_MODE_VTOL && !MISSION_HAS_VTOL_PARAMS
#error "MISSION_MODE is VTOL but params_gen.h has no VTOL parameters"
#endif
#if MISSION_HAS_VTOL_PARAMS && (!defined(MISSION_PROFILE) || !defined(VC_UP_UM) || !defined(EST_V_KP) || !defined(ALTV_K1))
#error "params_gen.h predates v0.7.1 (no MISSION_PROFILE / VC_UP_UM / EST_V_* / ALTV_*) - re-run tools/gen_firmware_params.py"
#endif
#if MISSION_HAS_VTOL_PARAMS && (!defined(VC_KT_ALPHA) || !defined(VTOL_OV_COUNT))
#error "params_gen.h predates v0.7.1b (no VC_KT_* thrust learning / VTOL_OV_* overrides) - re-run tools/gen_firmware_params.py"
#endif
#if MISSION_HAS_VTOL_PARAMS && (!defined(POS_KP_POS) || !defined(GPS_K_STEADY_PV_00) || !defined(VC_TAKEOFF_TICKS))
#error "params_gen.h predates v0.8 (no POS_* position hold / GPS_* NAV filter / VC_TAKEOFF_*) - re-run tools/gen_firmware_params.py"
#endif
#if defined(GPS_STEP_TICKS) && (GPS_STEP_TICKS != NL_SLOT_PARTS || GPS_EPOCH_SPLIT_TICKS != 4)
#error "core/gps_nav.c runs the 50 Hz step at j = 4 and the epoch fusion in four jobs j = 0..3: fusion_hz = loop_hz / 5"
#endif
#if defined(GPS_N_CORR) && GPS_N_CORR > GPS_NCORR_MAX
#error "flight_computer.nav.gps: n_corr exceeds core/gps_nav.h GPS_NCORR_MAX"
#endif
#if defined(POS_PLAUS_RESETS) && POS_PLAUS_RESETS > POS_RESETS_MAX
#error "position_hold plausibility: more resets than core/position_ctrl.h POS_RESETS_MAX stamps"
#endif
#if MISSION_HAS_VTOL_PARAMS && defined(GPS_K_STEADY_PV_00) && !defined(POS_TOUCHDOWN_KEEP_TRIM)
#error "params_gen.h predates v0.8.1 (no POS_TOUCHDOWN_KEEP_TRIM) - re-run tools/gen_firmware_params.py"
#endif
#if defined(GLINK_STATE_FRAME_BYTES) && (GLINK_STATE_FRAME_BYTES != GL_STATE_FRAME_BYTES || \
                                         GLINK_RESULT_FRAME_BYTES != GL_RESULT_FRAME_BYTES)
#error "params_gen.h GLINK_*_FRAME_BYTES differ from core/glink_proto.h (sim/glink_proto.py)"
#endif
#if defined(GLINK_LATENCY_TICKS) && (GLINK_LATENCY_TICKS < NL_GNS_L_MIN || GLINK_LATENCY_TICKS > NL_GNS_L_MAX)
#error "params glink.latency_ticks_ASSUMED outside the NAV's slot ring (core/nlink_proto.h NL_GNS_L_MIN .. NL_GNS_L_MAX)"
#endif
#if defined(NLINK_FCU_THR_SCALE) && NLINK_FCU_THR_SCALE != NL_FCU_THR_SCALE
#error "params_gen.h NLINK_FCU_THR_SCALE differs from core/nlink_proto.h NL_FCU_THR_SCALE (sim/nlink_proto.py FCU_THR_SCALE)"
#endif

void fc_default_plant(mixer_plant_t *p)
{
    p->l_v_m = MIX_L_V_M;
    p->r_v_m = MIX_R_V_M;
    p->s_v_m2 = MIX_S_V_M2;
    p->cl_alpha = MIX_CL_ALPHA;
    p->a_e_m2 = MIX_A_E_M2;
    p->t_max_n = MIX_T_MAX_N;
    p->throttle_exp = MIX_THROTTLE_EXP;
    p->delta_max_rad = MIX_DELTA_MAX_RAD;
    p->pwm_center_us = MIX_PWM_CENTER_US;
    p->pwm_us_per_deg = MIX_PWM_US_PER_DEG;
    p->servo_signs[0] = CAL_SERVO_SIGN_1;
    p->servo_signs[1] = CAL_SERVO_SIGN_2;
    p->servo_signs[2] = CAL_SERVO_SIGN_3;
    p->servo_signs[3] = CAL_SERVO_SIGN_4;
    p->servo_trims_us[0] = CAL_SERVO_TRIM_US_1;
    p->servo_trims_us[1] = CAL_SERVO_TRIM_US_2;
    p->servo_trims_us[2] = CAL_SERVO_TRIM_US_3;
    p->servo_trims_us[3] = CAL_SERVO_TRIM_US_4;
}

int fc_throttle_exp_int(void)
{
    return MIX_THROTTLE_EXP_INT;
}

void fc_default_gains(att_gains_t *g)
{
    g->loop_hz = GAIN_LOOP_HZ;
    g->att_kp_xy = GAIN_ATT_KP_XY;
    g->att_kp_z = GAIN_ATT_KP_Z;
    g->rate_sp_max_xy_rad_s = GAIN_RATE_SP_MAX_XY_RAD_S;
    g->rate_sp_max_z_rad_s = GAIN_RATE_SP_MAX_Z_RAD_S;
    g->rate.rate_kp_xy = GAIN_RATE_KP_XY;
    g->rate.rate_ki_xy = GAIN_RATE_KI_XY;
    g->rate.rate_kd_xy = GAIN_RATE_KD_XY;
    g->rate.rate_i_max_xy = GAIN_RATE_I_MAX_XY;
    g->rate.rate_kp_z = GAIN_RATE_KP_Z;
    g->rate.rate_ki_z = GAIN_RATE_KI_Z;
    g->rate.rate_kd_z = GAIN_RATE_KD_Z;
    g->rate.rate_i_max_z = GAIN_RATE_I_MAX_Z;
    g->rate.d_term_lpf_hz = GAIN_D_TERM_LPF_HZ;
    g->rate.gyro_lpf_hz = GAIN_GYRO_LPF_HZ;
    g->thrust_estimate_floor_fraction = GAIN_THRUST_ESTIMATE_FLOOR_FRACTION;
    g->roll_priority_scale = GAIN_ROLL_PRIORITY_SCALE;
    g->yaw_hold = GAIN_YAW_HOLD != 0;
#if FC_V09
    memset(&g->v9, 0, sizeof g->v9);                  /* the ballistic gains: every v0.9 option off (the v0.8.1 law) */
#endif
}

#if FC_V09 && MISSION_HAS_VTOL_PARAMS
#if !defined(VTOL_V09_ERR_MODE) || !defined(VC_RAMP_TOTAL_TICKS) || !defined(PWR_RULES) || !defined(POS_TOUCHDOWN_POLICY)
#error "params_gen.h predates v0.9 (no VTOL_V09_* / VC_RAMP_TOTAL_TICKS / PWR_RULES / POS_TOUCHDOWN_POLICY) - re-run tools/gen_firmware_params.py"
#endif
#define FC_ATT_V09_FLAGS                                                                                               \
    (uint8_t)((VTOL_V09_AW_PER_AXIS ? ATT_V9_AW : 0u) | (VTOL_V09_FF_ON ? ATT_V9_FF : 0u) |                              \
              ((VTOL_V09_FF_ON && VTOL_V09_FF_NKR != 0.0f) ? ATT_V9_REACT : 0u) |                                         \
              ((VTOL_V09_FF_ON && VTOL_V09_FF_GYRO) ? ATT_V9_GYRO : 0u) | (VTOL_V09_YAW_TRIM_RESERVE ? ATT_V9_TRIM : 0u))
#define FC_ATT_V09_INIT                                                                                                \
    {.err_mode = VTOL_V09_ERR_MODE, .flags = FC_ATT_V09_FLAGS, .ff_a = VTOL_V09_FF_A, .ff_nkr = VTOL_V09_FF_NKR,        \
     .ff_h = VTOL_V09_FF_H}
#if FC_VTOL_RAM_TRIM
const att_v09_t fc_att_v09_flash __attribute__((__progmem__)) = FC_ATT_V09_INIT;   /* the AVR VTOL image's options */
#endif
#endif

/* v0.7.1b: the VTOL build's attitude gains = gains.h with params.json control.vtol_overrides on top (params_gen.h
 * VTOL_OV_<KEY>; sim/controller.py gains_for_mode(..., "vtol")).  The ballistic build calls fc_default_gains only. */
void fc_vtol_gains(att_gains_t *g)
{
    fc_default_gains(g);
#ifdef VTOL_OV_ATT_KP_XY
    g->att_kp_xy = VTOL_OV_ATT_KP_XY;
#endif
#ifdef VTOL_OV_ATT_KP_Z
    g->att_kp_z = VTOL_OV_ATT_KP_Z;
#endif
#ifdef VTOL_OV_RATE_SP_MAX_XY_RAD_S
    g->rate_sp_max_xy_rad_s = VTOL_OV_RATE_SP_MAX_XY_RAD_S;
#endif
#ifdef VTOL_OV_RATE_SP_MAX_Z_RAD_S
    g->rate_sp_max_z_rad_s = VTOL_OV_RATE_SP_MAX_Z_RAD_S;
#endif
#ifdef VTOL_OV_RATE_KP_XY
    g->rate.rate_kp_xy = VTOL_OV_RATE_KP_XY;
#endif
#ifdef VTOL_OV_RATE_KI_XY
    g->rate.rate_ki_xy = VTOL_OV_RATE_KI_XY;
#endif
#ifdef VTOL_OV_RATE_KD_XY
    g->rate.rate_kd_xy = VTOL_OV_RATE_KD_XY;
#endif
#ifdef VTOL_OV_RATE_I_MAX_XY
    g->rate.rate_i_max_xy = VTOL_OV_RATE_I_MAX_XY;
#endif
#ifdef VTOL_OV_RATE_KP_Z
    g->rate.rate_kp_z = VTOL_OV_RATE_KP_Z;
#endif
#ifdef VTOL_OV_RATE_KI_Z
    g->rate.rate_ki_z = VTOL_OV_RATE_KI_Z;
#endif
#ifdef VTOL_OV_RATE_KD_Z
    g->rate.rate_kd_z = VTOL_OV_RATE_KD_Z;
#endif
#ifdef VTOL_OV_RATE_I_MAX_Z
    g->rate.rate_i_max_z = VTOL_OV_RATE_I_MAX_Z;
#endif
#ifdef VTOL_OV_D_TERM_LPF_HZ
    g->rate.d_term_lpf_hz = VTOL_OV_D_TERM_LPF_HZ;
#endif
#ifdef VTOL_OV_GYRO_LPF_HZ
    g->rate.gyro_lpf_hz = VTOL_OV_GYRO_LPF_HZ;
#endif
#ifdef VTOL_OV_THRUST_ESTIMATE_FLOOR_FRACTION
    g->thrust_estimate_floor_fraction = VTOL_OV_THRUST_ESTIMATE_FLOOR_FRACTION;
#endif
#ifdef VTOL_OV_ROLL_PRIORITY_SCALE
    g->roll_priority_scale = VTOL_OV_ROLL_PRIORITY_SCALE;
#endif
#ifdef VTOL_OV_YAW_HOLD
    g->yaw_hold = VTOL_OV_YAW_HOLD != 0;
#endif
#if FC_V09 && MISSION_HAS_VTOL_PARAMS
    g->v9.err_mode = VTOL_V09_ERR_MODE;               /* v0.9 options (VTOL_V09_*: the reference's values, defaults */
    g->v9.flags = FC_ATT_V09_FLAGS;                   /* incl.; field by field - a static const table sits in AVR RAM) */
    g->v9.ff_a = VTOL_V09_FF_A;
    g->v9.ff_nkr = VTOL_V09_FF_NKR;
    g->v9.ff_h = VTOL_V09_FF_H;
#endif
}


/* v0.9 vertical controller (params_gen.h VC_TO_* / VC_REF_* / VC_TD_REL_*) */
#define FC_VC_V9_FLAGS                                                                                                 \
    (uint8_t)((VC_TO_TWO_SLOPE ? VC_V9_TWO_SLOPE : 0u) | (VC_TO_BOOST ? VC_V9_BOOST : 0u) |                            \
              (VC_TO_LIFTOFF ? VC_V9_LIFTOFF : 0u) | (VC_TO_REF_HOLD ? VC_V9_REF_HOLD : 0u) |                          \
              (VC_LAND_REF_REBASE ? VC_V9_LAND_REBASE : 0u) | (VC_TD_RELEASE ? VC_V9_TD_RELEASE : 0u))
/* v0.9 power rules (params_gen.h PWR_*, sim/onboard.py SupervisorParams pw_*); the build: PWR_SRC (AVR) */
#if PM_ANY
#if !PWR_RULES
#error "a power build (PWR_SRC) needs params mission.vtol.power_rules_ASSUMED (params_gen.h PWR_RULES)"
#endif
#ifdef PWR_SRC
#define FC_PW_BUILD PWR_SRC
#else
#define FC_PW_BUILD PWR_SRC_BATTERY                  /* host builds: battery unless the test sets pw.build */
#endif
#define FC_PW_FIELDS                                                                                                   \
    {.build = FC_PW_BUILD, .shift = PWR_SHIFT, .arm_lo_q = PWR_ARM_LO_Q, .arm_hi_q = PWR_ARM_HI_Q,                     \
     .buf_arm_lo_q = PWR_BUF_ARM_LO_Q, .buf_arm_hi_q = PWR_BUF_ARM_HI_Q, .lost_q = PWR_LOST_Q,                        \
     .lost_margin_q = PWR_LOST_MARGIN_Q, .sag_q = PWR_SAG_Q, .sag_clear_q = PWR_SAG_CLEAR_Q, .buf_low_q = PWR_BUF_LOW_Q, \
     .buf_res_q = PWR_BUF_RES_Q, .d2_q = PWR_D2_Q, .i8buf_q = PWR_I8BUF_Q, .sag_warn_ticks = PWR_SAG_WARN_TICKS,       \
     .sag_land_ticks = PWR_SAG_LAND_TICKS, .buf_res_ticks = PWR_BUF_RES_TICKS, .lost_ticks = PWR_LOST_TICKS,           \
     .buf_ticks = PWR_BUF_TICKS, .d2_ticks = PWR_D2_TICKS, .sag_release_ticks = PWR_SAG_RELEASE_TICKS,                 \
     .cal_gain_q15 = {PWR_CAL_IN_GAIN_Q15, PWR_CAL_TET_GAIN_Q15, PWR_CAL_BUF_GAIN_Q15},                                \
     .cal_offset = {PWR_CAL_IN_OFFSET, PWR_CAL_TET_OFFSET, PWR_CAL_BUF_OFFSET}, .vaux_mul = PWR_VAUX_MUL,              \
     .vaux_add = PWR_VAUX_ADD, .vaux_shift = PWR_VAUX_SHIFT}
#endif
/* the tethered limits of a wired AVR build (sim/onboard.py tether_limits); host builds: the profile's (the cross-check
 * sets the tethered ones itself) */
#if defined(PWR_SRC) && PWR_SRC != PWR_SRC_BATTERY
#define FC_V_HOVER_DM ONB_W_HOVER_DM
#define FC_V_ALT_MIN_DM ONB_W_ALT_MIN_DM
#define FC_V_ALT_MAX_DM ONB_W_ALT_MAX_DM
#define FC_V_CEILING_M ONB_W_CEILING_M
#define FC_V_GEOFENCE_M ONB_W_GEOFENCE_M
#define FC_V_ENDURANCE_TICKS ONB_W_ENDURANCE_TICKS
#define FC_V_HARD_CUTOFF_TICKS ONB_W_HARD_CUTOFF_TICKS
#else
#define FC_V_HOVER_DM ONB_V_HOVER_DM
#define FC_V_ALT_MIN_DM ONB_V_ALT_MIN_DM
#define FC_V_ALT_MAX_DM ONB_V_ALT_MAX_DM
#define FC_V_CEILING_M ONB_V_CEILING_M
#define FC_V_GEOFENCE_M ONB_V_GEOFENCE_M
#define FC_V_ENDURANCE_TICKS ONB_V_ENDURANCE_TICKS
#define FC_V_HARD_CUTOFF_TICKS ONB_V_HARD_CUTOFF_TICKS
#endif

#if ONB_PARAMS_FLASH
/* the VTOL build's table in flash (ONB_PARAMS_FLASH implies a fixed VTOL build: no ballistic fields; gps_hold off in an
 * ONB_NO_GPS build, like main.c's op.gps_hold = false did) */
static const onb_params_t fc_onb_params __attribute__((__progmem__)) = {
    .loop_hz = FC_LOOP_HZ, .tilt_abort_deg = ONB_TILT_ABORT_DEG, .arm_tilt_max_deg = ONB_ARM_TILT_MAX_DEG,
    .min_cell_v = ONB_MIN_CELL_V, .link_disarm_ticks = ONB_LINK_DISARM_TICKS, .abort_release_ticks = ONB_ABORT_RELEASE_TICKS,
    .tilt_rate_abort_sq = ONB_TILT_RATE_ABORT_SQ, .spin_rate_abort = ONB_SPIN_RATE_ABORT_RAD_S,
    .rate_persist_ticks = ONB_RATE_PERSIST_TICKS, .gyro_saturation_abort = ONB_GYRO_SATURATION_ABORT != 0,
    .imu_fail_ticks = ONB_IMU_FAIL_TICKS, .baro_fail_ticks = ONB_BARO_FAIL_TICKS, .mode = LP_MODE_VTOL,
    .spool_ticks = ONB_V_SPOOL_TICKS, .hover_dm = FC_V_HOVER_DM, .alt_step_dm = ONB_V_ALT_STEP_DM,
    .alt_min_dm = FC_V_ALT_MIN_DM, .alt_max_dm = FC_V_ALT_MAX_DM, .link_land_ticks = ONB_V_LINK_LAND_TICKS,
    .endurance_ticks = FC_V_ENDURANCE_TICKS, .hard_cutoff_ticks = FC_V_HARD_CUTOFF_TICKS, .ceiling_m = FC_V_CEILING_M,
    .v_geofence_m = FC_V_GEOFENCE_M, .v_fall_vz_mps = ONB_V_FALL_VZ_MPS, .v_fall_confirm_ticks = ONB_V_FALL_CONFIRM_TICKS,
    .safe_after_landed_ticks = ONB_V_SAFE_AFTER_LANDED_TICKS, .land_timeout_ticks = ONB_V_LAND_TIMEOUT_TICKS,
    .baro_limit_base_ticks = ONB_V_BARO_LIMIT_BASE_TICKS, .baro_limit_mt_per_dm = ONB_V_BARO_LIMIT_MT_PER_DM,
    .v_geofence_dm = ONB_V_GEOFENCE_DM, .gps_hold = ONB_V_GPS_HOLD != 0 && ONB_HAS_GPS,
    .takeoff_check = ONB_V_TAKEOFF_CHECK != 0
#if PM_ANY
    , .pw = FC_PW_FIELDS
#endif
};

const onb_params_t *fc_onboard_params(void)
{
    return &fc_onb_params;                            /* a flash address: onboard_init / onb_stack_init only */
}
#else
void fc_default_onboard(onb_params_t *op)
{
    op->loop_hz = FC_LOOP_HZ;
    op->tilt_abort_deg = ONB_TILT_ABORT_DEG;
    op->arm_tilt_max_deg = ONB_ARM_TILT_MAX_DEG;
    op->min_cell_v = ONB_MIN_CELL_V;
    op->link_disarm_ticks = ONB_LINK_DISARM_TICKS;
    op->abort_release_ticks = ONB_ABORT_RELEASE_TICKS;
#if ONB_HAS_BALLISTIC
    op->climb_milli = ONB_CLIMB_MILLI;
    op->ramp_ticks = ONB_RAMP_TICKS;
    op->coast_hold_ticks = ONB_COAST_HOLD_TICKS;
    op->geofence_m = ONB_GEOFENCE_M;
    op->apogee_limit_m = ONB_APOGEE_LIMIT_M;
    op->apogee_k = ONB_APOGEE_K;
    op->fall_vz_mps = ONB_FALL_VZ_MPS;
    op->fall_confirm_ticks = ONB_FALL_CONFIRM_TICKS;
    op->link_abort_ticks = ONB_LINK_ABORT_TICKS;
    op->max_motor_ticks = ONB_MAX_MOTOR_TICKS;
    op->release_after_cut_ticks = ONB_RELEASE_AFTER_CUT_TICKS;
#endif
    op->tilt_rate_abort_sq = ONB_TILT_RATE_ABORT_SQ;
    op->spin_rate_abort = ONB_SPIN_RATE_ABORT_RAD_S;
    op->rate_persist_ticks = ONB_RATE_PERSIST_TICKS;
    op->gyro_saturation_abort = ONB_GYRO_SATURATION_ABORT != 0;
    op->imu_fail_ticks = ONB_IMU_FAIL_TICKS;
    op->baro_fail_ticks = ONB_BARO_FAIL_TICKS;
    op->mode = (uint8_t)MISSION_MODE;
#if !ONB_HAS_VTOL
    /* a ballistic fixed build: no VTOL parameters */
#elif MISSION_HAS_VTOL_PARAMS
    op->spool_ticks = ONB_V_SPOOL_TICKS;
    op->hover_dm = ONB_V_HOVER_DM;
    op->alt_step_dm = ONB_V_ALT_STEP_DM;
    op->alt_min_dm = ONB_V_ALT_MIN_DM;
    op->alt_max_dm = ONB_V_ALT_MAX_DM;
    op->link_land_ticks = ONB_V_LINK_LAND_TICKS;
    op->endurance_ticks = ONB_V_ENDURANCE_TICKS;
    op->hard_cutoff_ticks = ONB_V_HARD_CUTOFF_TICKS;
    op->ceiling_m = ONB_V_CEILING_M;
    op->v_geofence_m = ONB_V_GEOFENCE_M;
    op->v_fall_vz_mps = ONB_V_FALL_VZ_MPS;
    op->v_fall_confirm_ticks = ONB_V_FALL_CONFIRM_TICKS;
    op->safe_after_landed_ticks = ONB_V_SAFE_AFTER_LANDED_TICKS;
    op->land_timeout_ticks = ONB_V_LAND_TIMEOUT_TICKS;
    op->baro_limit_base_ticks = ONB_V_BARO_LIMIT_BASE_TICKS;
    op->baro_limit_mt_per_dm = ONB_V_BARO_LIMIT_MT_PER_DM;
    op->v_geofence_dm = ONB_V_GEOFENCE_DM;
    op->gps_hold = ONB_V_GPS_HOLD != 0;               /* v0.8 */
    op->takeoff_check = ONB_V_TAKEOFF_CHECK != 0;
#if PM_ANY
    {
        static const pm_params_t pw = FC_PW_FIELDS;   /* (host / ballistic-capable builds: RAM) */
        op->pw = pw;
    }
#endif
#else                                                 /* a ballistic-only params.json (v0.6.3): never read */
    op->spool_ticks = 0;
    op->hover_dm = op->alt_step_dm = op->alt_min_dm = op->alt_max_dm = 0;
    op->link_land_ticks = op->endurance_ticks = op->hard_cutoff_ticks = 1;
    op->ceiling_m = op->v_geofence_m = op->v_fall_vz_mps = 0.0f;
    op->v_fall_confirm_ticks = 1;
    op->safe_after_landed_ticks = 0;
    op->land_timeout_ticks = op->baro_limit_base_ticks = op->baro_limit_mt_per_dm = 0;
    op->v_geofence_dm = 0;
    op->gps_hold = false;
    op->takeoff_check = false;
#if PM_ANY
    memset(&op->pw, 0, sizeof op->pw);                /* battery build */
#endif
#endif
}

const onb_params_t *fc_onboard_params(void)
{
    static onb_params_t t;                            /* host / ballistic: a RAM table filled once */
    static bool done = false;
    if (!done) {
        fc_default_onboard(&t);
        done = true;
    }
    return &t;
}
#endif

#if VC_PARAMS_FLASH
/* the VTOL build's vertical-controller table in flash (fc_default_vertical's values; MISSION_HAS_VTOL_PARAMS holds in a
 * VTOL build) */
static const vc_params_t fc_vc_params __attribute__((__progmem__)) = {
    .up_um = VC_UP_UM, .dn_um = VC_DN_UM, .td_um = VC_TD_UM, .flare_um = VC_FLARE_UM, .um_tick_to_mps = VC_UM_TICK_TO_MPS,
    .flare_altitude_m = VC_FLARE_ALTITUDE_M, .alt_kp = VC_ALT_KP, .vz_kp = VC_VZ_KP, .vz_ki_dt = VC_VZ_KI_DT,
    .vz_i_max = VC_VZ_I_MAX, .vz_max_up_mps = VC_VZ_MAX_UP_MPS, .vz_max_down_mps = VC_VZ_MAX_DOWN_MPS, .u_min = VC_U_MIN,
    .u_max = VC_U_MAX, .u_hover = VC_U_HOVER, .inv_u_hover = VC_INV_U_HOVER, .throttle_exp_int = (int8_t)VC_THROTTLE_EXP_INT,
    .throttle_exp = VC_THROTTLE_EXP, .cos_tilt_min = VC_COS_TILT_MIN, .ramp_ticks = VC_RAMP_TICKS,
    .landed_confirm_ticks = VC_LANDED_CONFIRM_TICKS, .floor_ticks = VC_FLOOR_TICKS, .td_vz_still_mps = VC_TD_VZ_STILL_MPS,
    .lead_mps = VC_LEAD_MPS, .support_mps2 = VC_SUPPORT_MPS2, .g = VC_G, .thrust_alpha = VC_THRUST_ALPHA,
    .still_alpha = VC_STILL_ALPHA, .support_alpha = VC_SUPPORT_ALPHA, .vz_ff = VC_VZ_FF != 0, .kt_alpha = VC_KT_ALPHA,
    .kt_min = VC_KT_MIN, .kt_max = VC_KT_MAX, .kt_cos_min = VC_KT_COS_MIN, .takeoff_climb_m = VC_TAKEOFF_CLIMB_M,
    .takeoff_ticks = VC_TAKEOFF_TICKS,
    .takeoff_settle_ticks = VC_TAKEOFF_TICKS > 0u ? (uint16_t)VC_TAKEOFF_SETTLE_TICKS(FC_LOOP_HZ) : 0u,
#if FC_V09
    .ramp_total_ticks = VC_RAMP_TOTAL_TICKS, .v9 = FC_VC_V9_FLAGS, .approach_shift = VC_REF_APPROACH_SHIFT,
    .approach_min_um = VC_REF_APPROACH_MIN_UM, .to_u_start = VC_TO_U_START, .to_s1 = VC_TO_S1, .to_s2 = VC_TO_S2,
    .to_search_f = VC_TO_SEARCH_F, .to_search_vz = VC_TO_SEARCH_VZ, .kt_alpha_to = VC_KT_ALPHA_TO,
    .to_airborne_mps2 = VC_TO_AIRBORNE_MPS2, .td_step = VC_TD_REL_STEP, .td_max = VC_TD_REL_MAX, .td_ok = VC_TD_REL_OK
#endif
};

const vc_params_t *fc_vertical_params(void)
{
    return &fc_vc_params;                             /* a flash address: vc_init / onb_stack_init only */
}
#else
void fc_default_vertical(vc_params_t *vp)
{
#if MISSION_HAS_VTOL_PARAMS
    vp->up_um = VC_UP_UM;
    vp->dn_um = VC_DN_UM;
    vp->td_um = VC_TD_UM;
    vp->flare_um = VC_FLARE_UM;
    vp->um_tick_to_mps = VC_UM_TICK_TO_MPS;
    vp->flare_altitude_m = VC_FLARE_ALTITUDE_M;
    vp->alt_kp = VC_ALT_KP;
    vp->vz_kp = VC_VZ_KP;
    vp->vz_ki_dt = VC_VZ_KI_DT;
    vp->vz_i_max = VC_VZ_I_MAX;
    vp->vz_max_up_mps = VC_VZ_MAX_UP_MPS;
    vp->vz_max_down_mps = VC_VZ_MAX_DOWN_MPS;
    vp->u_min = VC_U_MIN;
    vp->u_max = VC_U_MAX;
    vp->u_hover = VC_U_HOVER;
    vp->inv_u_hover = VC_INV_U_HOVER;
    vp->throttle_exp_int = (int8_t)VC_THROTTLE_EXP_INT;
    vp->throttle_exp = VC_THROTTLE_EXP;
    vp->cos_tilt_min = VC_COS_TILT_MIN;
    vp->ramp_ticks = VC_RAMP_TICKS;
    vp->landed_confirm_ticks = VC_LANDED_CONFIRM_TICKS;
    vp->floor_ticks = VC_FLOOR_TICKS;
    vp->td_vz_still_mps = VC_TD_VZ_STILL_MPS;
    vp->lead_mps = VC_LEAD_MPS;
    vp->support_mps2 = VC_SUPPORT_MPS2;
    vp->g = VC_G;
    vp->thrust_alpha = VC_THRUST_ALPHA;
    vp->still_alpha = VC_STILL_ALPHA;
    vp->support_alpha = VC_SUPPORT_ALPHA;
    vp->vz_ff = VC_VZ_FF != 0;
    vp->kt_alpha = VC_KT_ALPHA;
    vp->kt_min = VC_KT_MIN;
    vp->kt_max = VC_KT_MAX;
    vp->kt_cos_min = VC_KT_COS_MIN;
    vp->takeoff_climb_m = VC_TAKEOFF_CLIMB_M;         /* v0.8 take-off check (0 ticks = off) */
    vp->takeoff_ticks = VC_TAKEOFF_TICKS;
    /* physics-review fix: h0 latched VC_TAKEOFF_SETTLE_S after the ramp (core/vertical_ctrl.h; sim/vertical_ctrl.py
     * ticks_ge(TAKEOFF_SETTLE_S_ASSUMED, loop_hz) - VC_TAKEOFF_SETTLE_TICKS(FC_LOOP_HZ) is that, for an integer rate) */
    vp->takeoff_settle_ticks = VC_TAKEOFF_TICKS > 0u ? (uint16_t)VC_TAKEOFF_SETTLE_TICKS(FC_LOOP_HZ) : 0u;
#if FC_V09
    vp->ramp_total_ticks = VC_RAMP_TOTAL_TICKS;       /* v0.9 (sim/vertical_ctrl.py 1b 1c 3b 4c 5b) */
    vp->v9 = FC_VC_V9_FLAGS;
    vp->approach_shift = VC_REF_APPROACH_SHIFT;
    vp->approach_min_um = VC_REF_APPROACH_MIN_UM;
    vp->to_u_start = VC_TO_U_START;
    vp->to_s1 = VC_TO_S1;
    vp->to_s2 = VC_TO_S2;
    vp->to_search_f = VC_TO_SEARCH_F;
    vp->to_search_vz = VC_TO_SEARCH_VZ;
    vp->kt_alpha_to = VC_KT_ALPHA_TO;
    vp->to_airborne_mps2 = VC_TO_AIRBORNE_MPS2;
    vp->td_step = VC_TD_REL_STEP;
    vp->td_max = VC_TD_REL_MAX;
    vp->td_ok = VC_TD_REL_OK;
#endif
#else                                                 /* a ballistic-only params.json (v0.6.3): never flown */
    vp->up_um = vp->dn_um = vp->td_um = 1;
    vp->flare_um = 0;
    vp->um_tick_to_mps = 0.0f;
    vp->flare_altitude_m = 0.0f;
    vp->alt_kp = vp->vz_kp = vp->vz_ki_dt = vp->vz_i_max = 0.0f;
    vp->vz_max_up_mps = vp->vz_max_down_mps = 0.0f;
    vp->u_min = 0.0f;
    vp->u_max = 1.0f;
    vp->u_hover = 1.0f;
    vp->inv_u_hover = 1.0f;
    vp->throttle_exp_int = 2;
    vp->throttle_exp = 2.0f;
    vp->cos_tilt_min = 1.0f;
    vp->ramp_ticks = 0;
    vp->landed_confirm_ticks = 1;
    vp->floor_ticks = 1;
    vp->td_vz_still_mps = vp->lead_mps = vp->support_mps2 = 0.0f;
    vp->g = FC_G0;
    vp->thrust_alpha = vp->still_alpha = vp->support_alpha = 1.0f;
    vp->vz_ff = false;
    vp->kt_alpha = 0.0f;                              /* no learning */
    vp->kt_min = vp->kt_max = 1.0f;
    vp->kt_cos_min = 1.0f;
    vp->takeoff_climb_m = 0.0f;
    vp->takeoff_ticks = 0;
    vp->takeoff_settle_ticks = 0;
#if FC_V09
    vp->ramp_total_ticks = 0;
    vp->v9 = 0;
    vp->approach_shift = 0;
    vp->approach_min_um = 1;
    vp->to_u_start = vp->to_s1 = vp->to_s2 = 0.0f;
    vp->to_search_f = 1.0f;
    vp->to_search_vz = 0.0f;
    vp->kt_alpha_to = 0.0f;
    vp->to_airborne_mps2 = FC_G0;
    vp->td_step = 0.0f;
    vp->td_max = 0.12f;
    vp->td_ok = -0.12f;
#endif
#endif
}

const vc_params_t *fc_vertical_params(void)
{
    static vc_params_t t;                             /* host / ballistic: a RAM table filled once */
    static bool done = false;
    if (!done) {
        fc_default_vertical(&t);
        done = true;
    }
    return &t;
}
#endif

/* v0.8 position hold: one initializer for the host copy and the AVR flash table; the derived fields are the float32
 * expressions of pos_params_derive (constant-folded: IEEE float32 operations, the same values) */
#if MISSION_HAS_VTOL_PARAMS
#define FC_POS_FIELDS                                                                                                 \
    .step_ticks = POS_STEP_TICKS, .T = POS_T, .g = POS_G, .tw = POS_TW, .kp_pos = POS_KP_POS, .kv = POS_KV,             \
    .ki = POS_KI, .kd = POS_KD, .alpha_d = POS_ALPHA_D, .i_max = POS_I_MAX, .v_max = POS_V_MAX,                          \
    .v_max_land = POS_V_MAX_LAND, .flare_speed = POS_FLARE_SPEED, .flare_altitude_m = POS_FLARE_ALTITUDE_M,              \
    .cos_cap = POS_COS_CAP, .cos_flare_cap = POS_COS_FLARE_CAP, .cos_excess = POS_COS_EXCESS,                            \
    .sin_excess = POS_SIN_EXCESS, .margin_g = POS_MARGIN_G, .margin_frac = POS_MARGIN_FRAC,                              \
    .h_enable_m = POS_H_ENABLE_M, .fence_m = POS_FENCE_M, .spin_z_rad = POS_SPIN_Z_RAD,                                  \
    .spin_xy_rad2 = POS_SPIN_XY_RAD2, .guard_steps = POS_GUARD_STEPS, .timeout_steps = POS_TIMEOUT_STEPS,                \
    .land_steps = POS_LAND_STEPS, .plaus_limited_steps = POS_PLAUS_LIMITED_STEPS,                                        \
    .plaus_vexc_steps = POS_PLAUS_VEXC_STEPS, .plaus_reset_window_steps = POS_PLAUS_RESET_WINDOW_STEPS,                  \
    .plaus_resets = POS_PLAUS_RESETS, .plaus_vt = POS_PLAUS_VT, .authority_min_ktw = POS_AUTHORITY_MIN_KTW,              \
    .tilt_fade_m = POS_TILT_FADE_M, .slew_per_tick = POS_SLEW_PER_TICK, .keep_trim = POS_TOUCHDOWN_POLICY,              \
    .g2 = POS_G * POS_G, .inv_t = 1.0f / POS_T, .ki_t = POS_KI * POS_T, .i_max2 = POS_I_MAX * POS_I_MAX,                 \
    .fence2 = POS_FENCE_M * POS_FENCE_M, .plaus_vt2 = POS_PLAUS_VT * POS_PLAUS_VT, .inv_fade = 1.0f / POS_TILT_FADE_M,   \
    .inv_g = 1.0f / POS_G, .inv_g2 = 1.0f / (POS_G * POS_G)
#else                                                 /* a ballistic-only params.json: never flown */
#define FC_POS_FIELDS                                                                                                 \
    .step_ticks = 5, .T = 0.02f, .g = FC_G0, .tw = 1.0f, .kp_pos = 0.0f, .kv = 0.0f, .ki = 0.0f, .kd = 0.0f,              \
    .alpha_d = 0.0f, .i_max = 0.0f, .v_max = 0.0f, .v_max_land = 0.0f, .flare_speed = 0.0f, .flare_altitude_m = 0.0f,    \
    .cos_cap = 1.0f, .cos_flare_cap = 1.0f, .cos_excess = 1.0f, .sin_excess = 0.0f, .margin_g = 0.0f,                   \
    .margin_frac = 0.0f, .h_enable_m = 0.0f, .fence_m = 0.0f, .spin_z_rad = 0.0f, .spin_xy_rad2 = 0.0f,                  \
    .guard_steps = 1, .timeout_steps = 1, .land_steps = 1, .plaus_limited_steps = 1, .plaus_vexc_steps = 1,              \
    .plaus_reset_window_steps = 1, .plaus_resets = 1, .plaus_vt = 0.0f, .authority_min_ktw = 0.0f, .tilt_fade_m = 1.0f, \
    .slew_per_tick = 0.0f, .keep_trim = 1, .g2 = FC_G0 * FC_G0, .inv_t = 1.0f / 0.02f, .ki_t = 0.0f, .i_max2 = 0.0f, .fence2 = 0.0f,      \
    .plaus_vt2 = 0.0f, .inv_fade = 1.0f, .inv_g = 1.0f / FC_G0, .inv_g2 = 1.0f / (FC_G0 * FC_G0)
#endif

#if POS_PARAMS_FLASH
static const pos_params_t fc_pos_params __attribute__((__progmem__)) = {FC_POS_FIELDS};   /* flash, not .data */

const pos_params_t *fc_position_params(void)
{
    return &fc_pos_params;                            /* a flash address: pos_init / onb_stack_init only */
}

void fc_default_position(pos_params_t *pp)
{
    pos_params_from_flash(pp, &fc_pos_params);
}
#else
static const pos_params_t fc_pos_params = {FC_POS_FIELDS};

const pos_params_t *fc_position_params(void)
{
    return &fc_pos_params;
}

void fc_default_position(pos_params_t *pp)
{
    *pp = fc_pos_params;
}
#endif

void fc_default_gps_nav(gps_nav_params_t *gp)
{
#if MISSION_HAS_VTOL_PARAMS
    /* direct stores, no tables (a static const table is .data, i.e. RAM, on the AVR) */
    gp->steady.kpv[0][0] = GPS_K_STEADY_PV_00;
    gp->steady.kpv[0][1] = GPS_K_STEADY_PV_01;
    gp->steady.kp[0] = GPS_K_STEADY_P_0;
    gp->steady.kv[0] = GPS_K_STEADY_V_0;
    gp->steady.kpv[1][0] = GPS_K_STEADY_PV_10;
    gp->steady.kpv[1][1] = GPS_K_STEADY_PV_11;
    gp->steady.kp[1] = GPS_K_STEADY_P_1;
    gp->steady.kv[1] = GPS_K_STEADY_V_1;
    gp->steady.kpv[2][0] = GPS_K_STEADY_PV_20;
    gp->steady.kpv[2][1] = GPS_K_STEADY_PV_21;
    gp->steady.kp[2] = GPS_K_STEADY_P_2;
    gp->steady.kv[2] = GPS_K_STEADY_V_2;
    gp->acq.kpv[0][0] = GPS_K_ACQ_PV_00;
    gp->acq.kpv[0][1] = GPS_K_ACQ_PV_01;
    gp->acq.kp[0] = GPS_K_ACQ_P_0;
    gp->acq.kv[0] = GPS_K_ACQ_V_0;
    gp->acq.kpv[1][0] = GPS_K_ACQ_PV_10;
    gp->acq.kpv[1][1] = GPS_K_ACQ_PV_11;
    gp->acq.kp[1] = GPS_K_ACQ_P_1;
    gp->acq.kv[1] = GPS_K_ACQ_V_1;
    gp->acq.kpv[2][0] = GPS_K_ACQ_PV_20;
    gp->acq.kpv[2][1] = GPS_K_ACQ_PV_21;
    gp->acq.kp[2] = GPS_K_ACQ_P_2;
    gp->acq.kv[2] = GPS_K_ACQ_V_2;
    gp->step_ticks = GPS_STEP_TICKS;
    gp->ms_per_tick = GPS_MS_PER_TICK;
    gp->latency_ms = GPS_LATENCY_MS;
    gp->loop_hz_f = GPS_LOOP_HZ_F;
    gp->inv_step = GPS_INV_STEP;
    gp->t_step = GPS_T_STEP;
    gp->lever_b[0] = GPS_LEVER_B_X_M;
    gp->lever_b[1] = GPS_LEVER_B_Y_M;
    gp->lever_b[2] = GPS_LEVER_B_Z_M;
    gp->psi0 = GPS_PSI0_RAD;
    gp->c_psi0 = GPS_C_PSI0;
    gp->s_psi0 = GPS_S_PSI0;
    gp->gate_min_sv = GPS_GATE_MIN_SV;
    gp->gate_max_hacc_mm = GPS_GATE_MAX_HACC_MM;
    gp->gate_max_sacc_cms = GPS_GATE_MAX_SACC_CMS;
    gp->gate_stable_ticks = GPS_GATE_STABLE_TICKS;
    gp->ok_min_sv = GPS_OK_MIN_SV;
    gp->ok_max_hacc_mm = GPS_OK_MAX_HACC_MM;
    gp->gate_v2 = GPS_GATE_V2;
    gp->gate_p2 = GPS_GATE_P2;
    gp->n_corr = GPS_N_CORR;
    gp->acq_epochs = GPS_ACQ_EPOCHS;
    gp->reset_after_epochs = GPS_RESET_AFTER_EPOCHS;
    gp->vel_reset_epochs = GPS_VEL_RESET_EPOCHS;
    gp->fresh_ticks = GPS_FRESH_TICKS;
    gp->yaw_correction = GPS_YAW_CORRECTION != 0;
    gp->yaw_window_steps = GPS_YAW_WINDOW_STEPS;
    gp->yaw_min_dv2 = GPS_YAW_MIN_DV2;
    gp->yaw_sigma_dv2 = GPS_YAW_SIGMA_DV2;
    gp->yaw_q_rad2 = GPS_YAW_Q_RAD2;
    gp->p_psi0 = GPS_P_PSI0;
    gp->head_ok_rad2 = GPS_HEAD_OK_RAD2;
    gp->yaw_sigma_rel2 = GPS_YAW_SIGMA_REL2_ASSUMED;  /* core/gps_nav.h (sim/gps_nav.py YAW_SIGMA_REL^2) */
#else
    uint8_t r;
    for (r = 0; r < 3u; r++) {
        gp->steady.kpv[r][0] = gp->steady.kpv[r][1] = gp->steady.kp[r] = gp->steady.kv[r] = 0.0f;
        gp->acq.kpv[r][0] = gp->acq.kpv[r][1] = gp->acq.kp[r] = gp->acq.kv[r] = 0.0f;
    }
    gp->step_ticks = 5;
    gp->ms_per_tick = (uint8_t)(1000u / FC_LOOP_HZ);
    gp->latency_ms = 0;
    gp->loop_hz_f = (float)FC_LOOP_HZ;
    gp->inv_step = 0.2f;
    gp->t_step = 5.0f / (float)FC_LOOP_HZ;
    gp->lever_b[0] = gp->lever_b[1] = gp->lever_b[2] = 0.0f;
    gp->psi0 = 0.0f;
    gp->c_psi0 = 1.0f;
    gp->s_psi0 = 0.0f;
    gp->gate_min_sv = gp->ok_min_sv = 255;
    gp->gate_max_hacc_mm = gp->gate_max_sacc_cms = gp->ok_max_hacc_mm = 0;
    gp->gate_stable_ticks = 1;
    gp->gate_v2 = gp->gate_p2 = 0.0f;
    gp->n_corr = 1;
    gp->acq_epochs = gp->reset_after_epochs = gp->vel_reset_epochs = 1;
    gp->fresh_ticks = 1;
    gp->yaw_correction = false;
    gp->yaw_window_steps = 1;
    gp->yaw_min_dv2 = gp->yaw_sigma_dv2 = gp->yaw_q_rad2 = gp->p_psi0 = gp->head_ok_rad2 = 0.0f;
    gp->yaw_sigma_rel2 = 0.0f;
#endif
}

uint8_t fc_glink_latency_ticks(void)
{
#if defined(GLINK_LATENCY_TICKS)
    return (uint8_t)GLINK_LATENCY_TICKS;
#else
    return 3u;                                        /* (no glink in params.json: an inert value) */
#endif
}

void fc_default_gns(gns_t *g, bool configure)
{
    gps_nav_params_t gp;
    fc_default_gps_nav(&gp);
#if MISSION_HAS_VTOL_PARAMS
    gns_init(g, &gp, fc_position_params(), configure, GPS_CFG_N_WAIT, GPS_CFG_N_ACK, GPS_CFG_N_SETTLE, GPS_CFG_N_VER,
             GPS_BAUD);
#else
    gns_init(g, &gp, fc_position_params(), configure, 1, 1, 1, 1, 38400UL);
#endif
}

void fc_default_gps(gps_nav_t *g, bool configure)
{
    gps_nav_params_t gp;
    fc_default_gps_nav(&gp);
#if MISSION_HAS_VTOL_PARAMS
    gps_nav_init(g, &gp, configure, GPS_CFG_N_WAIT, GPS_CFG_N_ACK, GPS_CFG_N_SETTLE, GPS_CFG_N_VER, GPS_BAUD);
#else
    gps_nav_init(g, &gp, configure, 1, 1, 1, 1, 38400UL);
#endif
}

void fc_default_mahony(att_mahony_t *m)
{
#if MISSION_HAS_VTOL_PARAMS
    att_mahony_init(m, FC_DT_S, GAIN_EST_PAD_N_AVG, EST_V_KP, EST_V_KI_DT, EST_V_GATE_MPS2, EST_V_G, EST_V_ALPHA_A,
                    EST_V_BIAS_LIMIT_RAD_S);
#else                                                 /* without mission.vtol: gyro-only after the pad (gate never open) */
    att_mahony_init(m, FC_DT_S, GAIN_EST_PAD_N_AVG, 0.0f, 0.0f, 0.0f, FC_G0, 1.0f, 0.0f);
#endif
}

void fc_default_alt_vtol(alt_vtol_t *f, float baro_latency_s)
{
    alt_vtol_params_t p;
#if MISSION_HAS_VTOL_PARAMS
    p.k1 = ALTV_K1;
    p.k2 = ALTV_K2;
    p.k3 = ALTV_K3;
    p.pad_n_avg = ALTV_PAD_N_AVG;
    p.bias_limit_mps2 = ALTV_BIAS_LIMIT_MPS2;
    p.comp_m = ALTV_COMP_M;
    p.inv_u_hover = VC_INV_U_HOVER;
    p.throttle_exp_int = (int8_t)VC_THROTTLE_EXP_INT;
    p.throttle_exp = VC_THROTTLE_EXP;
#else
    p.k1 = p.k2 = p.k3 = 0.0f;
    p.pad_n_avg = 1;
    p.bias_limit_mps2 = 0.0f;
    p.comp_m = 0.0f;
    p.inv_u_hover = 1.0f;
    p.throttle_exp_int = 2;
    p.throttle_exp = 2.0f;
#endif
    p.latency_s = baro_latency_s;
    p.tick_dt = FC_DT_S;
    alt_vtol_init(f, &p);
}

void fc_default_edf(edf_model_t *m)
{
    edf_init(m, MIX_T_MAX_N, MIX_THROTTLE_EXP, MIX_THROTTLE_EXP_INT, EDF_TAU_S, EDF_P_NOM_W, EDF_OMEGA_MAX_RAD_S,
             EDF_SWIRL_FRACTION, EDF_SWIRL_SIGN, MIX_A_E_M2, EDF_RHO, FC_DT_S);
}

void fc_default_estimator(att_est_t *e)
{
    att_est_init(e, FC_DT_S, GAIN_EST_PAD_N_AVG);
}

void fc_default_alt_filter(alt_filter_t *f, float baro_latency_s)
{
    alt_filter_init(f, ALT_CROSSOVER_HZ, ALT_ZETA, baro_latency_s, FC_DT_S);
}
