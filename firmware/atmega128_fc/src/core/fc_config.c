/* fc_config.c - see fc_config.h */
#include "fc_config.h"
#include "nlink_proto.h"

#include "../gains.h"
#include "../params_gen.h"

#include <string.h>

#if GAIN_LOOP_HZ != FC_LOOP_HZ
#error "gains.h and params_gen.h disagree on loop_hz - re-run both generators"
#endif
#if !defined(MISSION_MODE) || MISSION_MODE_BALLISTIC != LP_MODE_BALLISTIC
#error "params_gen.h lacks MISSION_MODE or its value differs from link_proto.h LP_MODE_BALLISTIC - re-run gen_firmware_params.py"
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
}

void fc_default_onboard(onb_params_t *op)
{
    op->loop_hz = FC_LOOP_HZ;
    op->tilt_abort_deg = ONB_TILT_ABORT_DEG;
    op->arm_tilt_max_deg = ONB_ARM_TILT_MAX_DEG;
    op->min_cell_v = ONB_MIN_CELL_V;
    op->link_disarm_ticks = ONB_LINK_DISARM_TICKS;
    op->abort_release_ticks = ONB_ABORT_RELEASE_TICKS;
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
    op->tilt_rate_abort_sq = ONB_TILT_RATE_ABORT_SQ;
    op->spin_rate_abort = ONB_SPIN_RATE_ABORT_RAD_S;
    op->rate_persist_ticks = ONB_RATE_PERSIST_TICKS;
    op->gyro_saturation_abort = ONB_GYRO_SATURATION_ABORT != 0;
    op->imu_fail_ticks = ONB_IMU_FAIL_TICKS;
    op->baro_fail_ticks = ONB_BARO_FAIL_TICKS;
    op->mode = (uint8_t)MISSION_MODE;
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
