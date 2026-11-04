/* power_mon.h - v0.9 power monitor of the tethered FCU builds (sim/onboard.py PowerMonitor, module docstring "v0.9 power
 * rules"; sim/power_link.py codes / flags / the TELEM v4 tail; params mission.vtol.power_rules_ASSUMED).
 *
 * Build selection (a build-time choice like the profile, sim/power_link.py): -D PWR_SRC=PWR_SRC_WIRED (1, battery-less:
 * V_in = the tether node) or PWR_SRC_WIRED_BUFFER (2, tether + 6S buffer behind the ideal diodes: V_tether before D1,
 * V_buffer before D2).  An AVR image without PWR_SRC is the battery build: none of this exists in it (no code, no RAM, no
 * cycles - every power branch is #if PM_ANY).  Host builds compile every build and choose it at run time
 * (onb_params_t.pw_build), so the cross-check runs all three against the reference.
 *
 * Arithmetic (integer, identical to the reference): every tick each measured channel's 10-bit code x (the conversion of
 * the previous tick; core: pm_sample) is filtered in Q4: y = 16 x on the first sample, then y += (16 x - y) >> shift
 * (arithmetic shift: avr-gcc asr, Python >> floors the same; |16 x - y| < 2^14).  The per-channel two-point calibration
 * (params power.adc_calibration_ASSUMED, params_gen.h PWR_CAL_*; identity by default) maps the raw ADC code first:
 * x = clamp(((raw gain_q15 + 16384) >> 15) + offset, 0, 1023) - the reference sees the calibrated code (its plant's
 * measure() is the true voltage; the firmware's calibration is what makes the ADC code that).  Persistence counters run
 * in POWERED only and restart at 0 outside (pm_sample(powered = false)), saturating like the reference's (0xFF / 0xFFFF).
 *
 *   wired         sag_cnt: y_in < sag (+1) / y_in > clear (0) / between (held: hysteresis); sag_cnt >= warn -> sag_warn
 *                 (u_cap = max(the previous tick's ESC throttle, 0), rel_cnt = 0); while warned: rel_cnt counts y_in >
 *                 clear (else 0), >= release -> warning off (u_cap none); W_SAG condition: sag_cnt >= land.
 *   wired_buffer  lost_now = y_tet < lost or y_tet + margin < y_buf: lost_cnt (+1, else 0), >= lost_ticks -> lost (latched
 *                 for the flight); buf_cnt: y_buf < buf_low; res_cnt: y_buf < buf_res; d2_cnt: y_buf > d2 and not
 *                 lost_now.  W_LOST: lost_cnt >= lost_ticks; W_BUF: buf_cnt >= buf_ticks or res_cnt >= res_ticks; W_D2:
 *                 d2_cnt >= d2_ticks; I8 on the buffer: y_buf < i8buf.
 *
 * RAM (AVR): wired 10 B (y_in, sag_cnt, rel_cnt, flags, u_cap as the reference's float), wired_buffer 10 B (y_tet,
 * y_buf, lost / buf / d2 u8 counters, res_cnt u16, flags).  y_in is not kept in wired_buffer (its rules, flags and
 * telemetry never read it).  Cycles: ~5-10 us per tick (estimate; measured on the bench, README).
 *
 * Invariant: the codes and tick counts are the reference's exactly (params_gen.h PWR_* from sim/onboard.py
 * SupervisorParams); nothing here can start the motor - the supervisor turns the conditions into LAND / SAFE / REJ_POWER.
 */
#ifndef FC_POWER_MON_H
#define FC_POWER_MON_H

#include "fcmath.h"

#define PWR_SRC_BATTERY 0
#define PWR_SRC_WIRED 1
#define PWR_SRC_WIRED_BUFFER 2

#if defined(__AVR__)
#ifndef PWR_SRC
#define PWR_SRC PWR_SRC_BATTERY
#endif
#define PM_WIRED (PWR_SRC == PWR_SRC_WIRED)
#define PM_BUF (PWR_SRC == PWR_SRC_WIRED_BUFFER)
#define PM_BUILD(p) ((uint8_t)PWR_SRC)
#define PM_PARAMS_FLASH (PM_WIRED || PM_BUF)   /* a power image is a fixed VTOL image: its tables are in flash */
#else
#define PM_WIRED 1
#define PM_BUF 1
#define PM_BUILD(p) ((p)->build)
#define PM_PARAMS_FLASH 0
#endif
#define PM_ANY (PM_WIRED || PM_BUF)

/* pwr_flags of the TELEM v4 / v5 tail (sim/power_link.py PF_*); bits 6-7 = the build */
#define PM_PF_TETHER_OK 0x01u
#define PM_PF_SAG 0x02u
#define PM_PF_BUF_LOW 0x04u
#define PM_PF_BUF_SHARE 0x08u
#define PM_PF_LOST 0x10u
#define PM_PF_D2 0x20u
#define PM_PF_BUILD_SHIFT 6

#define PM_CH_IN 0u                    /* channel index of pm_inputs_t.code */
#define PM_CH_TET 1u
#define PM_CH_BUF 2u
#define PM_CODE_NONE 0xFFFFu           /* not measured on this hardware (the reference's None) */

#if PM_ANY
typedef struct {                       /* sim.onboard.SupervisorParams pw_* (Q4 codes, ticks); flash in a power AVR image */
    uint8_t  build;                    /* PWR_SRC_* (host builds; the AVR image's is PWR_SRC) */
    uint8_t  shift;
    uint16_t arm_lo_q, arm_hi_q, buf_arm_lo_q, buf_arm_hi_q;
    uint16_t lost_q, lost_margin_q, sag_q, sag_clear_q, buf_low_q, buf_res_q, d2_q, i8buf_q;
    uint16_t sag_warn_ticks, sag_land_ticks, buf_res_ticks;
    uint8_t  lost_ticks, buf_ticks, d2_ticks, sag_release_ticks;
    uint16_t cal_gain_q15[3];          /* per channel (PM_CH_*): the two-point calibration, 32768 = identity */
    int8_t   cal_offset[3];
    uint32_t vaux_mul, vaux_add;       /* TELEM v_aux = min((y mul + add) >> shift, 255) = power_link.quant_v(volts) */
    uint8_t  vaux_shift;
} pm_params_t;

#define PM_F_SAG_WARN 0x01u
#define PM_F_LOST 0x02u
#define PM_F_BUF_SHARE 0x04u
#define PM_F_HAVE_IN 0x08u             /* y_in holds a sample (the reference's y is not None) */
#define PM_F_HAVE_TET 0x10u
#define PM_F_HAVE_BUF 0x20u

typedef struct {
    uint8_t  fl;                       /* PM_F_* */
#if PM_WIRED
    int16_t  y_in;                     /* Q4 filtered codes */
    uint16_t sag_cnt;
    uint8_t  rel_cnt;
    float    u_cap;                    /* the frozen throttle ceiling (valid while PM_F_SAG_WARN) */
#endif
#if PM_BUF
    int16_t  y_tet, y_buf;
    uint8_t  lost_cnt, buf_cnt, d2_cnt;
    uint16_t res_cnt;
#endif
} pm_t;

typedef struct {                       /* the ADC readings of the previous tick (raw 10-bit codes, PM_CODE_NONE = none) */
    uint16_t code[3];                  /* PM_CH_IN / TET / BUF */
    bool     buf_i_pos;                /* the hybrid board's buffer-current sign (discharging) */
} pm_inputs_t;

#define PM_NOTE_NONE 0u                /* pm_sample's warning transition (the supervisor logs it) */
#define PM_NOTE_SAG_WARN 1u
#define PM_NOTE_SAG_CLEAR 2u

void    pm_init(pm_t *m);
/* one tick: filters, then (powered) the persistence counters or (not powered) their reset; throttle_prev = the ESC
 * throttle of the previous tick (the W_SAG ceiling); returns PM_NOTE_* */
uint8_t pm_sample(pm_t *m, const pm_params_t *p, const pm_inputs_t *in, bool powered, float throttle_prev);
uint16_t pm_cal_code(const pm_params_t *p, uint8_t ch, uint16_t raw);   /* the calibrated code (0..1023) */
/* conditions (the supervisor's rules) */
bool    pm_sag_land(const pm_t *m, const pm_params_t *p);
bool    pm_lost_cond(const pm_t *m, const pm_params_t *p);
bool    pm_buf_cond(const pm_t *m, const pm_params_t *p);
bool    pm_d2_cond(const pm_t *m, const pm_params_t *p);
bool    pm_i8_buf(const pm_t *m, const pm_params_t *p);
bool    pm_arm_ok(const pm_t *m, const pm_params_t *p);
static inline bool pm_sag_warn(const pm_t *m) { return (m->fl & PM_F_SAG_WARN) != 0u; }
uint8_t pm_flags(const pm_t *m, const pm_params_t *p);                  /* PF_* | build << 6 */
uint8_t pm_vaux_q(const pm_t *m, const pm_params_t *p);                 /* the tail's v_aux byte */
#endif

#endif /* FC_POWER_MON_H */
