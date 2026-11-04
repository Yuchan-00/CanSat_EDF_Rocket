/* attitude_ctrl.h - AttitudeController of sim/controller.py (SPEC 8.6).
 *
 * update(q_est, gyro, u):
 *   q_sp    = from_yaw(yaw_ref) (yaw hold) or from_yaw(yaw(q_est))
 *   e       = to_rotvec(error(q_est, q_sp))
 *   rate_sp = clip(att_kp e, +-rate_sp_max)
 *   T_hat   = thrust_estimate(u); freeze = last_saturated or thrust_floored(u)
 *   M, w_f  = RatePID(rate_sp, gyro, freeze)
 *   delta, saturated = deflections(M, T_hat); last_saturated = saturated
 *
 * v0.8 (VTOL with GPS position hold, sim/controller.py): the setpoint can be tilted.  tilt_sp = (t_x, t_y), the rotation
 * vector of the tilt in the NAV level frame (rad) that the FCU's position hold built at the end of the previous tick
 * (core/position_ctrl.h pos_tilt_tick); q_sp = q(t) (x) q_yaw with q(t) by series (no trig: th^2 = t.t, w = 1 - th^2/8 +
 * th^4/384, k = 1/2 - th^2/48 + th^4/3840; in Horner form with float32 reciprocals - 0.2 ms background budget) expanded with the yaw quaternion (c, 0, 0,
 * s):
 *     q_sp = (w c, k t_x c + k t_y s, k t_y c - k t_x s, w s)
 * and the error is the general quat_error (q_sp has x, y != 0).  attitude_ctrl_set_tilt runs in the background after the
 * position step (the end of the FCU tick): with yaw hold it builds q_sp there, so the 250 Hz attitude path only copies it.
 * tilt_sp = (0, 0) (the reset value, and always in the ballistic build): the v0.7.1 setpoint and error exactly.
 * ATT_HAS_TILT_SP is 0 in a build pinned to the ballistic mode (ONB_FIXED_MODE = LP_MODE_BALLISTIC): no field, no code.
 *
 * v0.9 (control + fix rounds; sim/controller.py VTOL_EXTRA_KEYS, params control.vtol_overrides; FC_V09 builds only - a
 * ballistic fixed image keeps the v0.8.1 code and bytes).  att_v09_t (fc_vtol_gains; the ballistic gains: all off):
 *   err_mode 0: the rotation vector (v0.8.1); 1: the reduced attitude error (sim/quat.py reduced_error, e_xy = 2 (w x -
 *     y z, w y + x z), e_z = 2 z of q_e = error(q, q_sp)) while the tilt set point is non-zero; 2: in every tick.  The
 *     v0.8.2 candidate D port B: with yaw hold and a tilt the set point product is folded into the error
 *     (tilt_hold_error: p = q^-1 (x) q(t), e_xy from p - the tilt part is invariant under the right z rotation -, z_e
 *     and w_e of p (x) (c, 0, 0, s) for e_z with error()'s sign rule); with yaw hold and no tilt (mode 2) the same with
 *     q(t) = 1 (level_hold_error); without yaw hold the general setpoint / error / reduced_error.  attitude_ctrl_set_tilt
 *     builds only q(t) = (w, k t_x, k t_y) (q_t[3]); set_tilt_finish is a no-op (kept for the callers); 5 B less RAM.
 *   aw_per_axis: rate_pid_update_aw with the previous tick's mixer limits and moment signs (aw byte, which replaces
 *     last_saturated: ATT_AW_SAT = RPID_AW_LIM_Z) - pitch / roll frozen only when (d_x, d_y) were scaled, yaw only when
 *     d_z was limited, each only while the error pushes into the limit; the floored-thrust freeze stays for all axes.
 *   rotor feed-forwards (ff_on = ff_rotor_h_nms != 0; u = clip(throttle, 0, 1), the ESC throttle of the previous tick):
 *     u_lag primed with u after a reset (u_lag < 0 = not primed: u is never negative), du = u - u_lag, u_lag += a du;
 *     reaction (ff_rotor_react != 0): M_z += NKR du (NKR = -react h_ref / tau / roll_priority_scale, params_gen.h);
 *     gyroscopic (ff_rotor_gyro): h_z = -h_ref u_lag, M_x += w_f,y h_z, M_y -= w_f,x h_z (not flown: fix round F3).
 *     The moments after the feed-forwards are the mixer's input and the anti-windup's "previous demand".
 *   yaw_trim_reserve: mixer_deflections_v9 keeps the yaw integrator's share of d_z (fix round F4).
 * RAM (AVR VTOL image, FC_VTOL_RAM_TRIM): the options and constants stay in flash (fc_att_v09_flash, core/fc_config.c);
 * attitude_ctrl_t: q_t[3] for q_sp_tilt[4] + tilt_half (-5 B), u_lag (+4 B), aw for last_saturated (0 B), the init-only
 * dt dropped (-4 B): -5 B against v0.8.1.
 *
 * Invariant: same state and order as the Python class.  With yaw hold the
 * setpoint quaternion is computed once in set_yaw_reference (cos/sin there,
 * not in the 250 Hz path).
 */
#ifndef FC_ATTITUDE_CTRL_H
#define FC_ATTITUDE_CTRL_H

#include "fcmath.h"
#include "link_proto.h"
#include "mixer.h"
#include "pid.h"
#include "quat.h"

#if !defined(ONB_FIXED_MODE) || ONB_FIXED_MODE == LP_MODE_VTOL
#define ATT_HAS_TILT_SP 1
#else
#define ATT_HAS_TILT_SP 0
#endif

#if FC_V09
#define ATT_V9_AW 0x01u                /* aw_per_axis */
#define ATT_V9_FF 0x02u                /* rotor feed-forwards (ff_rotor_h_nms != 0): the u_lag filter runs */
#define ATT_V9_REACT 0x04u             /* ... the reaction term (ff_rotor_react != 0) */
#define ATT_V9_GYRO 0x08u              /* ... the gyroscopic term (ff_rotor_gyro) */
#define ATT_V9_TRIM 0x10u              /* mix_yaw_trim_reserve */
typedef struct {                       /* v0.9 VTOL options (module docstring); all 0 = the v0.8.1 controller */
    uint8_t err_mode;                  /* att_err_mode 0 / 1 / 2 */
    uint8_t flags;                     /* ATT_V9_* */
    float   ff_a;                      /* dt / (ff_rotor_tau_s + dt) */
    float   ff_nkr;                    /* -react h_ref / tau / roll_priority_scale */
    float   ff_h;                      /* ff_rotor_h_nms (signed) */
} att_v09_t;
#endif

typedef struct {
    int   loop_hz;
    float att_kp_xy, att_kp_z, rate_sp_max_xy_rad_s, rate_sp_max_z_rad_s;
    rate_gains_t rate;
    float thrust_estimate_floor_fraction, roll_priority_scale;
    bool  yaw_hold;
#if FC_V09
    att_v09_t v9;                      /* v0.9 options (fc_vtol_gains; fc_default_gains: all off) */
#endif
} att_gains_t;

typedef struct {               /* ControllerOutput */
    float delta_cmd[4];
    float moments[3];
    float rate_sp[3];
    float att_error[3];
    float gyro_filtered[3];
    bool  saturated;
    float thrust_est;
    float pwm_us[4];
    float integrator[3];
} ctrl_output_t;

#if FC_V09
#define ATT_AW_SAT RPID_AW_LIM_Z       /* attitude_ctrl_t.aw: the previous tick saturated (the v0.8.1 last_saturated) */
#endif

typedef struct {
#if !(FC_V09 && FC_VTOL_RAM_TRIM)      /* v0.9: init-only, dropped in the VTOL AVR image (-4 B) */
    float dt;
#endif
    float att_kp[3];
    float rate_sp_max[3];
    bool  yaw_hold;
    rate_pid_t pid;
    vane_mixer_t mixer;
    float yaw_ref;
    float q_sp_hold[4];
#if FC_V09
    uint8_t aw;                  /* v0.9: the previous tick's ATT_AW_SAT (= last_saturated) | RPID_AW_LIM_XY | moment signs */
#else
    bool  last_saturated;
#endif
#if ATT_HAS_TILT_SP
    float tilt_sp[2];            /* v0.8: level-frame tilt rotation vector (rad); (0, 0) = the v0.7.1 setpoint */
#if FC_V09
    float q_t[3];                /* v0.8.2 D port B: q(tilt_sp) = (w, k tx, k ty) (z = 0), built by attitude_ctrl_set_tilt */
#else
    float q_sp_tilt[4];          /* with yaw hold: q(tilt_sp) (x) q_sp_hold, built by attitude_ctrl_set_tilt */
#endif
    bool  tilt_on;               /* tilt_sp != (0, 0) */
#if !FC_V09
    bool  tilt_half;             /* q_sp_tilt holds only q(t) = (w, k tx, k ty): attitude_ctrl_set_tilt_finish due */
#endif
#endif
#if FC_V09
    float u_lag;                 /* v0.9 rotor feed-forwards: the lagged ESC throttle; < 0 = not primed */
#if !FC_VTOL_RAM_TRIM
    att_v09_t v9;                /* host builds: a copy of the options (the AVR VTOL image reads fc_att_v09_flash) */
#endif
#endif
} attitude_ctrl_t;

#if FC_V09 && FC_VTOL_RAM_TRIM
extern const att_v09_t fc_att_v09_flash;   /* core/fc_config.c, in flash: the VTOL AVR image's options (FC_FLASH_RD) */
#endif
#if FC_V09
bool attitude_ctrl_last_saturated(const attitude_ctrl_t *c);
#endif

void attitude_ctrl_init(attitude_ctrl_t *c, const att_gains_t *g, const mixer_plant_t *plant, int throttle_exp_int);
void attitude_ctrl_reset(attitude_ctrl_t *c);
void attitude_ctrl_set_yaw_reference(attitude_ctrl_t *c, const float q_est[4]);
void attitude_ctrl_setpoint(const attitude_ctrl_t *c, const float q_est[4], float q_sp[4]);
void attitude_ctrl_update(attitude_ctrl_t *c, const float q_est[4], const float gyro[3], float throttle,
                          ctrl_output_t *out);
#if ATT_HAS_TILT_SP
/* v0.8 VTOL: the tilt setpoint of the next update (sim.controller.AttitudeController.tilt_sp) */
/* in two background pieces of <= 0.2 ms (ATmega128A, measured): set_tilt builds q(t) and returns true when
 * attitude_ctrl_set_tilt_finish (the product with the yaw quaternion) is due - the next background slot; update() and
 * the next set_tilt finish a half-built setpoint first, so the numbers never depend on where the split fell */
bool attitude_ctrl_set_tilt(attitude_ctrl_t *c, const float tilt[2]);
void attitude_ctrl_set_tilt_finish(attitude_ctrl_t *c);
/* q(t) (x) (c, 0, 0, s) by the series (the module docstring); exposed for the host test */
void attitude_ctrl_tilt_quat(const float tilt[2], const float q_yaw[4], float q_sp[4]);
#endif

#endif /* FC_ATTITUDE_CTRL_H */
