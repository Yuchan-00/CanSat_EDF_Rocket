/* nav_simbench.c - see nav_simbench.h (compiled only with -D FC_SIMBENCH). */
#ifdef FC_SIMBENCH
#include "nav_simbench.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <math.h>
#include <stdlib.h>

#include "fc_mode.h"
#include "core/nlink_proto.h"
#include "core/onboard.h"
#include "hal/mpu6050.h"
#include "hal/nlink_spi.h"
#include "hal/stack.h"
#include "hal/timer.h"
#include "hal/uart_log.h"
#include "nav/nav.h"
#include "params_gen.h"
#if FC_VTOL
#include "core/glink_proto.h"
#include "core/ilink_proto.h"                              /* il_crc16 (the SEQ fault frame) */
#include "core/gps_nav.h"                                 /* GPS_F_* (the synthetic GNS's STATUS flags) */
#include "core/position_ctrl.h"                          /* POS_MODE_* */
#include "hal/glink_uart.h"
#endif
#if FC_VTOL && !defined(FC_COSIM) && !defined(GLINK_WIRED)
#define NAV_SB_SYN_GNS 1                                 /* single chip: the synthetic GNS on the bench wire */
#else
#define NAV_SB_SYN_GNS 0
#endif

static uint8_t s_raw[MPU6050_BURST];
static uint16_t s_mark[NAV_SB_SECTIONS];
static uint16_t s_sec_max[NAV_SB_SECTIONS];
static uint16_t s_win_crit, s_win_ready, s_win_total, s_bg_max, s_ovh_max;
static uint16_t s_crit_pad, s_crit_flight, s_ready_pad, s_ready_flight;     /* cumulative */
static uint16_t s_ready_min = 0xFFFFu, s_ready_prev;                        /* cumulative */
static int16_t s_ready_jump = -32767;                     /* largest ready_us(N) - ready_us(N-1), cumulative */
static uint16_t s_report_phase;

void nav_sb_bg_step(uint16_t us)
{
    if (us > s_bg_max) s_bg_max = us;
}

void nav_sb_mark(uint8_t i)
{
    s_mark[i] = timer_tcnt1();
}

uint16_t nav_sb_ready_us(void)
{
    return (uint16_t)((timer_sim_now() - timer_sim_tick_start()) / FC_PWM_COUNTS_PER_US);
}

/* v0.7.1 VTOL: the compute end before the push floor (nav_simbench.h) */
static uint16_t s_ce_max, s_late_push;                    /* cumulative */
void nav_sb_compute_end(bool late)
{
    const uint16_t us = nav_sb_ready_us();
    if (us > s_ce_max) s_ce_max = us;
    if (late && s_late_push < 0xFFFFu) s_late_push++;
}

#if !NAV_SB_SYNTH_SENSORS
/* v0.9 nav_cosim_twi (nav_simbench.h NAV_SB_TWI): the real drivers read the harness's TWI sensor models; the synthetic
 * sources are empty (nav_main.c does not call them in this build; kept so a caller that still does compiles) */
void nav_sb_prepare(uint32_t next_tick, uint8_t state)
{
    (void)next_tick;
    (void)state;
}

const uint8_t *nav_sb_imu_raw(void)
{
    return s_raw;
}

void nav_sb_baro(uint32_t tick, uint8_t phase, baro_t *b)
{
    (void)tick;
    (void)phase;
    (void)b;
}
#else
/* ---- synthetic sensors: the FCU simbench's (src/simbench.c of v0.6.2), unchanged ---- */
static void put_be16(uint8_t *p, int16_t c)
{
    p[0] = (uint8_t)((uint16_t)c >> 8);
    p[1] = (uint8_t)c;
}

/* The synthetic IMU: on the pad still (small gyro offsets, 1 g), in flight oscillating rates
 *   gyro = base + amp sin(2 pi f t + phi)   (f 1.3 / 0.9 / 0.5 Hz, amp 0.6 / 0.5 / 0.3 rad/s, phi 0 / 90 / 0 deg)
 * and a thrust specific force.  Counts are prepared once; the per-tick path is a 64-step Q15 sine table with
 * linear interpolation and 16-bit phase accumulators. */
static const int16_t k_sin64[65] PROGMEM = {
    0, 3212, 6393, 9512, 12539, 15446, 18204, 20787, 23170, 25329, 27245, 28898, 30273, 31356, 32137, 32609, 32767,
    32609, 32137, 31356, 30273, 28898, 27245, 25329, 23170, 20787, 18204, 15446, 12539, 9512, 6393, 3212, 0,
    -3212, -6393, -9512, -12539, -15446, -18204, -20787, -23170, -25329, -27245, -28898, -30273, -31356, -32137,
    -32609, -32767, -32609, -32137, -31356, -30273, -28898, -27245, -25329, -23170, -20787, -18204, -15446, -12539,
    -9512, -6393, -3212, 0};
static const float k_g_hz[3] = {1.3f, 0.9f, 0.5f}, k_g_amp[3] = {0.6f, 0.5f, 0.3f}, k_g_base[3] = {0.010f, -0.005f, 0.002f};
static const uint16_t k_g_phase0[3] = {0x0000u, 0x4000u, 0x0000u};
#if FC_VTOL                                              /* v0.7.1: a hover's specific force (|f| = 9.79 m/s^2: the gate opens) */
static const float k_a_pad[3] = {0.0f, 0.0f, FC_G0}, k_a_fly[3] = {0.3f, -0.2f, 9.78f};
#else
static const float k_a_pad[3] = {0.0f, 0.0f, FC_G0}, k_a_fly[3] = {0.4f, -0.3f, 15.0f};
#endif
static int16_t s_g_base[3], s_g_amp[3], s_a_pad[3], s_a_fly[3];
static uint16_t s_g_step[3];
static bool s_imu_ready;

static int16_t sin_q15(uint16_t ph)
{
    const uint8_t i = (uint8_t)(ph >> 10), f = (uint8_t)(ph >> 2);
    const int16_t a = (int16_t)pgm_read_word(&k_sin64[i]), b = (int16_t)pgm_read_word(&k_sin64[i + 1u]);
    return (int16_t)(a + (int16_t)(((int32_t)(b - a) * f) >> 8));
}

void nav_sb_prepare(uint32_t next_tick, uint8_t state)
{
    uint8_t i;
    if (!s_imu_ready) {
        for (i = 0; i < 3; i++) {
            s_g_base[i] = (int16_t)lroundf(k_g_base[i] / IMU_GYRO_RAD_S_PER_LSB);
            s_g_amp[i] = (int16_t)lroundf(k_g_amp[i] / IMU_GYRO_RAD_S_PER_LSB);
            s_g_step[i] = (uint16_t)lroundf(k_g_hz[i] * 65536.0f / (float)FC_LOOP_HZ);
            s_a_pad[i] = (int16_t)lroundf(k_a_pad[i] / IMU_ACCEL_MPS2_PER_LSB);
            s_a_fly[i] = (int16_t)lroundf(k_a_fly[i] / IMU_ACCEL_MPS2_PER_LSB);
        }
        s_imu_ready = true;
    }
    /* the raw MPU-6050 burst (chip frame = body frame for the identity mounting) */
    for (i = 0; i < 3; i++) {
        int16_t g = s_g_base[i];
        if (state == ONB_SAFE || state == ONB_ARMED) {     /* still on the pad */
            put_be16(&s_raw[2 * i], s_a_pad[i]);
        } else {                                           /* flying */
            const uint16_t ph = (uint16_t)((uint16_t)next_tick * s_g_step[i] + k_g_phase0[i]);
            g = (int16_t)(g + (int16_t)(((int32_t)s_g_amp[i] * sin_q15(ph)) >> 15));
            put_be16(&s_raw[2 * i], s_a_fly[i]);
        }
        put_be16(&s_raw[8 + 2 * i], g);
    }
    s_raw[6] = 0;
    s_raw[7] = 0;
}

const uint8_t *nav_sb_imu_raw(void)
{
    return s_raw;
}

#if FC_BARO_SENSOR == FC_BARO_BMP280
/* bmp280_step without the bus: the datasheet's example trimming parameters (3.12), raw values around its example
 * appear at the read phase and the publish gate opens at the start phase, as the driver does; the compensation (two int64
 * steps) and the altitude then run through bmp280_background exactly as in the firmware.  adc_T moves in steps of
 * 16 = the x1 temperature oversampling's 16-bit grid. */
static const bmp280_calib_t k_cal = {27504u, 26435, -1000, 36477u, -10685, 3024, 2855, 140, -7, 15500, -14600, 6000};

void nav_sb_baro(uint32_t tick, uint8_t phase, baro_t *b)
{
    if (!b->cal_ok) {
        b->cal = k_cal;
        b->cal_ok = true;
    }
    if (phase == BMP280_START_PHASE) {
        b->gate_open = true;
    } else if (phase == BMP280_READ_PHASE) {
        if (b->stage != BMP280_ST_IDLE)                    /* the last sample was never published (bmp280_step) */
            b->fail_count++;
        b->gate_open = false;
        b->adc_p = 415148UL + (uint32_t)(tick & 0xFF);
        b->adc_t = 519888UL - 16UL * (uint32_t)(tick & 0x3F);
        b->stage = BMP280_ST_RAW;
    }
}
#else
/* datasheet example PROM, D1/D2 around it */
static const uint16_t k_prom[8] = {0, 40127, 36924, 23317, 23282, 33464, 28312, 0};

/* ms5611_step without the bus: D1/D2 appear at the CALC phase; the compensation and the
 * altitude then run through ms5611_background exactly as in the firmware */
void nav_sb_baro(uint32_t tick, uint8_t phase, baro_t *b)
{
    uint8_t i;
    if (!b->prom_ok) {
        for (i = 0; i < 8; i++) b->prom[i] = k_prom[i];
        b->prom_ok = true;
    }
    if (phase == MS5611_CALC_PHASE) {
        b->d1 = 9085466UL + (uint32_t)(tick & 0xFF);
        b->d2 = 8569150UL - (uint32_t)(tick & 0x3F);
        b->d1_valid = b->have_d2 = true;
        b->need_comp = true;
    }
}
#endif /* FC_BARO_SENSOR */
#endif /* NAV_SB_SYNTH_SENSORS */

/* ---- report lines: numbers in hex, nibble by nibble (no 32-bit division in the loop it measures) ---- */
static uint8_t put_u(char *buf, uint8_t pos, const char *key_P, uint32_t v)
{
    static const char hex[] PROGMEM = "0123456789abcdef";
    const uint8_t b[4] = {(uint8_t)(v >> 24), (uint8_t)(v >> 16), (uint8_t)(v >> 8), (uint8_t)v};
    uint8_t k;
    bool lead = true;
    char c;
    if (pos > (uint8_t)(216u - 18u))
        return pos;                                     /* the line is full: the field is left out (never overflows) */
    while ((c = (char)pgm_read_byte(key_P++)) != 0) buf[pos++] = c;
    buf[pos++] = '0';
    buf[pos++] = 'x';
    for (k = 0; k < 8u; k++) {
        const uint8_t d = (k & 1u) ? (uint8_t)(b[k >> 1] & 0xFu) : (uint8_t)(b[k >> 1] >> 4);
        if (d || !lead || k == 7u) {
            buf[pos++] = (char)pgm_read_byte(&hex[d]);
            lead = false;
        }
    }
    buf[pos++] = ' ';
    return pos;
}

static uint8_t line_start(char *buf, char c1, char c2)
{
    buf[0] = '\n';
    buf[1] = c1;
    buf[2] = c2;
    buf[3] = ' ';
    return 4;
}

static char s_buf[216];                                 /* static: the report must not deepen the measured stack */
#if FC_VTOL && !defined(FC_COSIM)
static nl_gns_rx_t s_grx;                                /* v0.8.1: the synthetic FCU's GNS slot receiver (nlink v4) */
#endif
static uint16_t s_reports_dropped;

static void send_line(uint8_t p)
{
    s_buf[p++] = '\n';
    if (!uart_log_write((const uint8_t *)s_buf, p))      /* never wait: the report must not cause a miss */
        s_reports_dropped++;
}

void nav_sb_init(void)
{
    uart_log_init();
#if FC_VTOL && !defined(FC_COSIM)
    nl_gns_rx_init(&s_grx, GL_STEP_TICKS);
#endif
#ifdef FC_COSIM
    PORTA = 0;                                           /* the tick marker port */
    DDRA = 0xFF;
#endif
}

void nav_sb_boot_line(uint8_t reset_cause)
{
    uint8_t p = line_start(s_buf, 'S', 'B');
    s_buf[p++] = 'B'; s_buf[p++] = 'O'; s_buf[p++] = 'O'; s_buf[p++] = 'T'; s_buf[p++] = ' ';
    p = put_u(s_buf, p, PSTR("rc="), reset_cause);
    p = put_u(s_buf, p, PSTR("sr="), stack_reserve_total());
    p = put_u(s_buf, p, PSTR("xb="), NL_XFER_BYTES);
    p = put_u(s_buf, p, PSTR("gc="), NLINK_BYTE_GAP_CYCLES);
    p = put_u(s_buf, p, PSTR("sh="), NLINK_SPI_HZ / 1000UL);
#ifdef FC_COSIM
    p = put_u(s_buf, p, PSTR("bd="), 1);
#else
    p = put_u(s_buf, p, PSTR("bd="), 0);
#endif
    send_line(p);
}

/* ======================================================================================== */
/* the synthetic FCU on the nlink (single-chip simbench; FC_COSIM wires a real FCU instead) */
/* ======================================================================================== */
#if !defined(FC_COSIM)
static uint8_t s_state = ONB_SAFE;                       /* the synthetic supervisor's state (the vehicle's truth) */
static uint8_t s_rep_state = ONB_SAFE;                   /* the state in the reply set for the next exchange ... */
static bool s_rep_valid;                                 /* ... and whether it decodes (no reply before the first) */
static uint8_t s_belief = ONB_SAFE;                      /* what the NAV holds: its last valid reply (SAFE before) */
static uint8_t s_last_seq, s_replies_since_corrupt;
static bool s_have_seq, s_tag_done;
static uint16_t s_nav_ok, s_nav_bad, s_seq_gaps, s_mode_bad, s_mode_checked, s_fly_noest, s_boot_frames, s_tag_frames;
static uint16_t s_corrupt, s_bad_returned, s_baro_late;
static uint16_t s_gate_flight, s_gate_pad;               /* v0.7.1: frames with NAV_F_EST_GATE in flight / pad mode */
static bool s_baro_seen;
static uint32_t s_t_aligned, s_t_estvalid, s_t_armed, s_t_powered, s_tag_tick;

#if FC_VTOL
/* v0.8: the frame of NAV tick k carries the altitude filter / EST_GATE / BARO_NEW of tick k-1, whose pad mode came from
 * the state the NAV held one exchange earlier.  v0.8.1: the GNS slots go through the FCU's own slot receiver (s_grx,
 * above); every TILT part must ride in the frame of NAV tick 5 m + 4 + L of its cycle m (the synthetic GNS writes m into
 * the part's y component) */
static uint8_t s_belief_prev = ONB_SAFE;
static uint16_t s_tilt_parts, s_tilt_ok, s_tilt_bad;
static uint32_t s_tilt_next_m;                           /* the next cycle whose TILT part is due */
static uint16_t s_tilt_gaps;                             /* cycles without their TILT part (since the first one) */
static bool s_tilt_seen;
#endif

static void sb_fcu_after_loop(uint32_t tick)
{
#if FC_VTOL
    nl_nav3_t n3;
    struct {
        uint8_t seq, flags, reset_cause;
        float h_m, vz_mps;
    } n;
    nl_fcu4_t f;
#else
    nl_nav_t n;
    nl_fcu_t f;
#endif
    bool valid, expect_pad;
    const uint8_t *s_txf = nlink_spi_take_tx_buf();       /* the bypass's own buffer (no copy) */
    uint8_t *s_rep = nlink_spi_reply_buf();               /* the reply is encoded in place */
    if (s_txf == 0)
        return;                                          /* no exchange this tick */
#if FC_VTOL
    valid = nl_decode_nav4(s_txf, &n3) != 0u;
    n.seq = n3.seq;
    n.flags = n3.flags;
    n.reset_cause = n3.reset_cause;
    n.h_m = n3.h_m;
    n.vz_mps = n3.vz_mps;
    expect_pad = s_belief_prev == ONB_SAFE || s_belief_prev == ONB_ARMED;   /* the mode of tick k-1 */
    s_belief_prev = s_belief;
    (void)nl_gns_rx_feed(&s_grx, valid ? n3.slot : 0);
    if (valid && (n3.slot[0] & 0x07u) == NL_SLOT4_TILT) {   /* a TILT part: cycle m in its y component */
        const uint16_t my = (uint16_t)(n3.slot[3] | ((uint16_t)n3.slot[4] << 8));
        const uint32_t r = tick - 4u - GLINK_LATENCY_TICKS;
        s_tilt_parts++;
        if (r % 5u == 0u && (uint16_t)((r / 5u) & 0x7FFFu) == my && (uint8_t)((r / 5u) & 0x1Fu) == (uint8_t)(n3.slot[0] >> 3))
            s_tilt_ok++;
        else
            s_tilt_bad++;
        if (s_tilt_seen && r / 5u > s_tilt_next_m)
            s_tilt_gaps = (uint16_t)(s_tilt_gaps + (uint16_t)(r / 5u - s_tilt_next_m));
        s_tilt_seen = true;
        s_tilt_next_m = r / 5u + 1u;
    }
#else
    valid = nl_decode_nav(s_txf, &n) != 0u;
    /* the NAV built this frame with the state it held after the previous exchange; this exchange returned
     * s_rep_*, which (if valid) it holds from now on */
    expect_pad = s_belief == ONB_SAFE || s_belief == ONB_ARMED;
#endif
    if (s_rep_valid)
        s_belief = s_rep_state;
    else
        s_bad_returned++;                                /* the NAV must have refused this exchange's reply */
    if (valid) {
        s_nav_ok++;
        if (s_have_seq && n.seq != (uint8_t)(s_last_seq + 1u))
            s_seq_gaps++;
        s_last_seq = n.seq;
        s_have_seq = true;
        if (n.flags & NL_F_BOOT)
            s_boot_frames++;
        if (n.reset_cause & NAV_RC_TAG_MISMATCH)
            s_tag_frames++;
        if (n.flags & NL_F_BARO_OK)
            s_baro_seen = true;
        else if (s_baro_seen)
            s_baro_late++;                               /* a baro publish slipped by more than the one tick of slack */
        if (!(n.flags & NL_F_BARO_NEW)) {                /* without a baro correction the pad mode holds h = v = 0 */
            const bool sig_pad = n.h_m == 0.0f && n.vz_mps == 0.0f;
            s_mode_checked++;
            if (sig_pad != expect_pad)
                s_mode_bad++;
        }
        if (n.flags & NAV_F_EST_GATE) {
            if (expect_pad)
                s_gate_pad++;                            /* the pad branch never corrects (attitude from the levelling) */
            else
                s_gate_flight++;
        }
        if (onboard_flying(s_state) && (n.flags & (NL_F_IMU_OK | NL_F_EST_VALID)) != (NL_F_IMU_OK | NL_F_EST_VALID))
            s_fly_noest++;                               /* the FCU would see imu_ok false: I9 */
        if ((n.flags & NL_F_ALIGNED) && !s_t_aligned)
            s_t_aligned = tick;
        if ((n.flags & NL_F_EST_VALID) && !s_t_estvalid)
            s_t_estvalid = tick;
        /* the supervisor in miniature: the ARM gate's NAV part, then LAUNCH */
        if (s_state == ONB_SAFE && tick >= (uint32_t)NAV_SB_ARM_S * FC_LOOP_HZ &&
            (n.flags & (NL_F_ALIGNED | NL_F_IMU_OK | NL_F_BARO_OK)) == (NL_F_ALIGNED | NL_F_IMU_OK | NL_F_BARO_OK)) {
            s_state = ONB_ARMED;
            s_t_armed = tick;
        } else if (s_state == ONB_ARMED && tick >= (uint32_t)NAV_SB_LAUNCH_S * FC_LOOP_HZ) {
            s_state = ONB_POWERED;
            s_t_powered = tick;
        }
    } else {
        s_nav_bad++;
    }
    /* the reply the next exchange returns: SEQ of the frame accepted, this state */
    f.seq = s_last_seq;
    f.state = s_state;
    f.fflags = 0;
    f.params_tag = FC_PARAMS_TAG;
#if FC_VTOL
    f.thr_q = s_state == ONB_POWERED ? NAV_SB_THR_Q_HOVER : 0u;   /* v0.7.1: the ESC throttle (baro compensation) */
    /* v0.8.1 nlink v4: the GNS position controller's FCU inputs - HOLD with the motor on while POWERED, lifted, k_T 1.0 */
    f.pflags = s_state == ONB_POWERED ? nl_fcu4_pflags(LP_PHASE_HOLD, true, (tick & 63u) == 0u, false, false, true)
                                      : nl_fcu4_pflags(LP_PHASE_NONE, false, false, false, false, false);
    f.kt_q = (uint16_t)NL_KT_SCALE;
#else
    f.thr_q = 0u;                                        /* the ballistic FCU's reserved byte */
#endif
    s_rep_valid = true;
    if (!s_tag_done && tick >= (uint32_t)NAV_SB_TAG_S * FC_LOOP_HZ) {
        f.params_tag = (uint8_t)~FC_PARAMS_TAG;          /* one valid reply from an FCU of another params build */
        s_tag_done = true;
        s_tag_tick = tick;
    }
#if FC_VTOL
    (void)nl_encode_fcu4(&f, s_rep);
#else
    (void)nl_encode_fcu(&f, s_rep);
#endif
    s_rep_state = s_state;
    /* not this reply nor the next one while the tag case runs: exactly one NAV frame must then carry the mismatch */
    if (tick >= (uint32_t)NAV_SB_CORRUPT_S * FC_LOOP_HZ && !(s_tag_done && tick - s_tag_tick <= 1u) &&
        ++s_replies_since_corrupt >= NAV_SB_CORRUPT_EVERY) {
        s_replies_since_corrupt = 0;
#if FC_VTOL
        s_rep[NL_FCU4_FRAME_BYTES - 1u] ^= 0x01u;        /* CRC hi byte: the NAV must refuse it */
#else
        s_rep[NL_FCU_FRAME_BYTES - 1u] ^= 0x01u;         /* CRC hi byte: the NAV must refuse it */
#endif
        s_rep_valid = false;
        s_corrupt++;
    }
    /* (encoded in place: the next exchange returns it) */
}
#endif /* !FC_COSIM */

#if FC_VTOL
/* ======================================================================================== */
/* v0.8.1: the NAV's glink side, measured; the synthetic GNS on the bench wire (single chip)  */
/* ======================================================================================== */
static uint16_t s_gl_state_max, s_gl_state_min = 0xFFFFu;     /* STATE written after the tick's Timer2 stamp, us */
static uint16_t s_gl_states, s_gl_refused;
static uint16_t s_build_max, s_pre_max, s_pre_nmax;           /* frame build after the stamp; pre-build decode */
static uint16_t s_gl_taken;                                   /* RESULT frames taken by the NAV's decoder */
static uint32_t s_tick_t0[8];                                 /* timer_sim_tick_start of the last 8 ticks (by tick & 7) */
static uint16_t s_gl_sec_max;                                 /* the glink section (gl_nav_tick + STATE write), us */
/* v0.8: tick -> the end of the mandatory work (SW line) */
static uint16_t s_work_win, s_work_pad, s_work_flight;        /* tick -> mandatory work done, us */
static uint32_t s_work_sum;                                   /* flight ticks: sum of work_us (mean = sum / n) */
static uint16_t s_work_n, s_work_over_lead, s_work_over_tick, s_late_start_max;

void nav_sb_glink_rx(uint32_t tick, uint8_t n, uint8_t taken)
{
    (void)tick;
    (void)n;
    s_gl_taken = (uint16_t)(s_gl_taken + taken);
}

void nav_sb_glink_state(uint32_t tick, bool ok)
{
    const uint16_t us = nav_sb_ready_us();
    (void)tick;
    if (ok) {
        s_gl_states++;
        if (us > s_gl_state_max) s_gl_state_max = us;
        if (us < s_gl_state_min) s_gl_state_min = us;
    } else {
        s_gl_refused++;
    }
}

#if NAV_SB_SYN_GNS
/* the synthetic GNS: its decoder, the RESULT it answers with, the completion time of every RESULT on the wire by cycle */
static uint16_t s_gs_frames, s_gs_bad, s_gs_sent, s_gs_faults;
static uint32_t s_gs_prev_tick;
static bool s_gs_have_prev;
#define GS_ARR_N 8u                                           /* RESULT completion times by cycle & 7 */
static uint32_t s_arr_t[GS_ARR_N], s_arr_m[GS_ARR_N];
static bool s_arr_v[GS_ARR_N];
static uint16_t s_arr_max, s_arr_min = 0xFFFFu;               /* completion after the sample tick's stamp, us */
static int16_t s_marg_min = 32767, s_marg_max = -32767;       /* build of frame 5 m + 4 + L minus the completion, us */
#endif

void nav_sb_slot_build(uint32_t tick, uint8_t nb, uint16_t us)
{
    const uint16_t b = nav_sb_ready_us();
    s_tick_t0[tick & 7u] = timer_sim_tick_start();
    if (b > s_build_max) s_build_max = b;
    if (us > s_pre_max) s_pre_max = us;
    if (nb > s_pre_nmax) s_pre_nmax = nb;
#if NAV_SB_SYN_GNS
    if (tick >= 4u + GLINK_LATENCY_TICKS && (tick - 4u - GLINK_LATENCY_TICKS) % 5u == 0u) {   /* a part-0 build */
        const uint32_t m = (tick - 4u - GLINK_LATENCY_TICKS) / 5u;
        const uint8_t i = (uint8_t)(m & (GS_ARR_N - 1u));
        if (s_arr_v[i] && s_arr_m[i] == m) {
            const int32_t d = (int32_t)(timer_sim_now() - s_arr_t[i]) / (int32_t)FC_PWM_COUNTS_PER_US;
            const int16_t dd = d > 32767 ? 32767 : (d < -32767 ? -32767 : (int16_t)d);
            if (dd < s_marg_min) s_marg_min = dd;
            if (dd > s_marg_max) s_marg_max = dd;
        }
    }
#endif
}

#if NAV_SB_SYN_GNS
/* the RESULT of cycle m for the STATE st (bench values, not a GNS model): TILT 0.02 rad in x and the cycle m in y while the
 * FCU fields say the motor runs (else NL_TILT_NONE in x), the ON gate's flags once the NAV's attitude is valid, 9
 * satellites, hAcc 1.8 m, POS mode with the hold point, small POS / VEL / TRIM */
static void gs_result(uint32_t m, const gl_state_t *st, gl_result_t *r)
{
    const bool on = (st->pflags & NL_PF_MOTOR_ON) != 0u;
    r->tilt[0] = on ? 200 : NL_TILT_NONE;
    r->tilt[1] = (int16_t)(m & 0x7FFFu);
    r->flags = (uint8_t)(GPS_F_FIX3D | GPS_F_CFG_OK | GPS_F_INIT | GPS_F_VER_OK |
                         ((st->nflags & GL_NF_EST_VALID) ? (GPS_F_FIX_OK | GPS_F_GATE_OK | GPS_F_HEAD_OK) : 0u));
    r->num_sv = 9u;
    r->hacc_dm = 18u;
    r->pc = nl_pc_byte(on ? POS_MODE_POS : POS_MODE_OFF, on, 0u);
    r->pos[0] = (int16_t)(m & 0x3Fu);
    r->pos[1] = (int16_t)-(int16_t)(m & 0x1Fu);
    r->vel[0] = 12;
    r->vel[1] = -7;
    r->trim[0] = on ? 150 : 0;
    r->trim[1] = on ? -80 : 0;
}

/* glink byte time in Timer1 counts (FC_PWM_COUNTS_PER_US per us) */
#define GS_BYTE_COUNTS ((uint32_t)(GLINK_RESULT_FRAME_US / GL_RESULT_FRAME_BYTES) * FC_PWM_COUNTS_PER_US)

/* after the NAV's background (nav_sb_end_of_loop): the STATE frame the NAV queued this tick -> checks -> a RESULT on the
 * bench wire, its first byte NAV_SB_GNS_CYCLE_US + one byte time after the STATE's last byte.  The frame is taken whole
 * (the NAV queues one 60-byte frame per cycle, gl_encode_state - byte-exact by tools/crosscheck_controller.py 9a): sync,
 * TYPE and length are checked here, not the CRC (the bench's own time must stay small: it runs after the NAV's
 * background and a long one delays the next tick's start - the SB "ox" / SW "ls" figures) */
static void gs_step(uint32_t tick)
{
    uint8_t b[GL_STATE_FRAME_BYTES], n;
    uint32_t tq;
    while ((n = glink_uart_bench_take_tx(b, (uint8_t)sizeof b, &tq)) != 0u) {
        {
            gl_state_t st;
            gl_result_t r;
            uint8_t fr[GL_RESULT_FRAME_BYTES], seq;
            uint32_t m, t_first, now, delay;
            uint8_t a;
            if (n != GL_STATE_FRAME_BYTES || b[0] != GL_SYNC0 || b[1] != GL_SYNC1 || b[2] != GL_TYPE_STATE) {
                s_gs_bad++;
                continue;
            }
            seq = b[3];
            gl_unpack_state(b + GL_HEADER_BYTES, &st);
            s_gs_frames++;
            m = st.tick / 5u;
            if (st.tick % 5u != 4u || seq != (uint8_t)m || (s_gs_have_prev && st.n != 5u) ||
                (s_gs_have_prev && st.tick != s_gs_prev_tick + 5u) || st.tick > tick ||
                ((st.nflags & GL_NF_FCU_OK) && st.kt_q != (uint16_t)NL_KT_SCALE))
                s_gs_bad++;
            s_gs_prev_tick = st.tick;
            s_gs_have_prev = true;
            gs_result(m, &st, &r);
            (void)gl_encode_result(seq, &r, fr);
            /* the STATE's last byte reached the GNS 60 byte times after it was queued (the UDRE ISR starts at once); the
             * GNS answers NAV_SB_GNS_CYCLE_US later; the RESULT's first byte completes one byte time after that */
            t_first = tq + (uint32_t)GL_STATE_FRAME_BYTES * GS_BYTE_COUNTS + (uint32_t)NAV_SB_GNS_CYCLE_US * FC_PWM_COUNTS_PER_US +
                      GS_BYTE_COUNTS;
            if (m == NAV_SB_GL_MISS_M) {
                s_gs_faults++;
                continue;                                      /* no RESULT for this cycle */
            }
            if (m == NAV_SB_GL_LATE_M) {                       /* its last byte completes one tick after the build of
                                                                * frame 5 m + 4 + L (that build is >= the tick start) */
                s_gs_faults++;
                t_first = s_tick_t0[st.tick & 7u] + (uint32_t)(GLINK_LATENCY_TICKS + 1u) * FC_TICK_US * FC_PWM_COUNTS_PER_US;
            }
            if (m == NAV_SB_GL_CRC_M) {
                s_gs_faults++;
                fr[10] ^= 0x10u;                               /* a payload bit: the CRC no longer matches */
            }
            if (m == NAV_SB_GL_SEQ_M) {
                uint16_t c;
                s_gs_faults++;
                fr[3] = (uint8_t)(fr[3] + 3u);                 /* another SEQ, with a valid CRC */
                c = il_crc16(fr + 2, (uint8_t)(2u + GL_RESULT_LEN), LP_CRC_INIT);
                fr[GL_RESULT_FRAME_BYTES - 2u] = (uint8_t)c;
                fr[GL_RESULT_FRAME_BYTES - 1u] = (uint8_t)(c >> 8);
            }
            now = timer_sim_now();
            delay = (int32_t)(t_first - now) > 0 ? (t_first - now) * (FC_F_CPU_HZ / 1000000UL / FC_PWM_COUNTS_PER_US) : 0u;
            if (glink_uart_bench_send(fr, GL_RESULT_FRAME_BYTES, delay) != GL_RESULT_FRAME_BYTES)
                continue;                                      /* the wire still busy (counted: sent < frames) */
            if (delay == 0u)
                t_first = now + GS_BYTE_COUNTS;
            a = (uint8_t)(m & (GS_ARR_N - 1u));
            s_gs_sent++;
            s_arr_t[a] = t_first + (uint32_t)(GL_RESULT_FRAME_BYTES - 1u) * GS_BYTE_COUNTS;   /* the last byte */
            s_arr_m[a] = m;
            s_arr_v[a] = m != NAV_SB_GL_CRC_M && m != NAV_SB_GL_SEQ_M && m != NAV_SB_GL_LATE_M;
            if (s_arr_v[a]) {
                const uint32_t d = (s_arr_t[a] - s_tick_t0[st.tick & 7u]) / FC_PWM_COUNTS_PER_US;
                const uint16_t du = d > 0xFFFFu ? 0xFFFFu : (uint16_t)d;
                if (du > s_arr_max) s_arr_max = du;
                if (du < s_arr_min) s_arr_min = du;
            }
        }
    }
}
#endif
#endif /* FC_VTOL */

void nav_sb_end_of_loop(const nav_sb_tick_t *t)
{
    const uint16_t ovh0 = timer_tcnt1();
    uint8_t i, p;
    for (i = 1; i < NAV_SB_SECTIONS; i++) {
        uint16_t d = timer_elapsed_counts(s_mark[i - 1], s_mark[i]);
        if (d > s_sec_max[i]) s_sec_max[i] = d;
    }
    if (t->crit_us > s_win_crit) s_win_crit = t->crit_us;
    if (t->ready_us > s_win_ready) s_win_ready = t->ready_us;
    if (t->ready_us < s_ready_min) s_ready_min = t->ready_us;
    if (s_ready_prev != 0u) {                              /* the FCU's tick spacing is the tick + this difference */
        const int16_t j = (int16_t)(t->ready_us - s_ready_prev);
        if (j > s_ready_jump) s_ready_jump = j;
    }
    s_ready_prev = t->ready_us;
    if (t->total_us > s_win_total) s_win_total = t->total_us;
#if FC_VTOL
    if (t->work_us > s_work_win) s_work_win = t->work_us;
    if (t->pad) {
        if (t->work_us > s_work_pad) s_work_pad = t->work_us;
    } else {
        if (t->work_us > s_work_flight) s_work_flight = t->work_us;
        s_work_sum += t->work_us;
        s_work_n++;
        if (t->work_us + (uint16_t)((uint32_t)FC_IMU_LEAD_COUNTS * FC_T2_PRESCALER / (FC_F_CPU_HZ / 1000000UL)) > FC_TICK_US)
            s_work_over_lead++;                          /* ends inside the next tick's IMU lead window */
        if (t->work_us > FC_TICK_US)
            s_work_over_tick++;                          /* ends after the next tick: that tick starts late */
    }
    {   /* the tick start's lateness after the Timer2 ISR (the previous tick's overrun) */
        const uint16_t late = (uint16_t)(t->ready_us - t->crit_us);
        if (late > s_late_start_max) s_late_start_max = late;
    }
    {   /* v0.8.1: the glink section (marks 4 -> 5: gl_nav_tick, the STATE frame at k mod 5 == 4) */
        const uint16_t g = (uint16_t)(timer_elapsed_counts(s_mark[4], s_mark[5]) / FC_PWM_COUNTS_PER_US);
        if (g > s_gl_sec_max) s_gl_sec_max = g;
    }
#endif
    if (t->pad) {
        if (t->crit_us > s_crit_pad) s_crit_pad = t->crit_us;
        if (t->ready_us > s_ready_pad) s_ready_pad = t->ready_us;
    } else {
        if (t->crit_us > s_crit_flight) s_crit_flight = t->crit_us;
        if (t->ready_us > s_ready_flight) s_ready_flight = t->ready_us;
    }
#if !defined(FC_COSIM)
    sb_fcu_after_loop(t->tick);
    nav_sb_prepare(t->tick + 1, s_state);                  /* the vehicle follows the (synthetic) supervisor */
#else
    nav_sb_prepare(t->tick + 1, t->fcu_state);             /* the vehicle follows the NAV's view of the real FCU */
#endif
#if NAV_SB_SYN_GNS
    gs_step(t->tick);                                      /* v0.8.1: the synthetic GNS's turn */
#endif
    {   /* the synthetic sources' own time (the report lines below come on top twice per second) */
        const uint16_t d = (uint16_t)(timer_elapsed_counts(ovh0, timer_tcnt1()) / FC_PWM_COUNTS_PER_US);
        if (d > s_ovh_max) s_ovh_max = d;
    }
    if (++s_report_phase >= FC_LOOP_HZ) {                    /* once per second: the timing line */
        s_report_phase = 0;
        p = line_start(s_buf, 'S', 'B');
        p = put_u(s_buf, p, PSTR("t="), t->tick);
        p = put_u(s_buf, p, PSTR("st="), t->fcu_state);
        p = put_u(s_buf, p, PSTR("pd="), t->pad);
        p = put_u(s_buf, p, PSTR("cm="), s_win_crit);
        p = put_u(s_buf, p, PSTR("cp="), s_crit_pad);
        p = put_u(s_buf, p, PSTR("cf="), s_crit_flight);
        p = put_u(s_buf, p, PSTR("fr="), s_win_ready);
        p = put_u(s_buf, p, PSTR("fp="), s_ready_pad);
        p = put_u(s_buf, p, PSTR("ff="), s_ready_flight);
        p = put_u(s_buf, p, PSTR("fn="), s_ready_min);
        p = put_u(s_buf, p, PSTR("jx="), (uint16_t)s_ready_jump);
        p = put_u(s_buf, p, PSTR("tm="), s_win_total);
        p = put_u(s_buf, p, PSTR("ov="), t->overruns);
        p = put_u(s_buf, p, PSTR("sf="), stack_free_min_step());
        p = put_u(s_buf, p, PSTR("bg="), s_bg_max);
        p = put_u(s_buf, p, PSTR("ox="), s_ovh_max);
#if FC_VTOL
        p = put_u(s_buf, p, PSTR("ce="), s_ce_max);                /* v0.7.1: compute end before the push floor, us */
        p = put_u(s_buf, p, PSTR("lp="), s_late_push);             /* ... and the ticks past NAV_PUSH_MIN_US_VTOL */
#endif
        p = put_u(s_buf, p, PSTR("s="), s_sec_max[1] / FC_PWM_COUNTS_PER_US);
        for (i = 2; i < NAV_SB_SECTIONS; i++) p = put_u(s_buf, p, PSTR(""), s_sec_max[i] / FC_PWM_COUNTS_PER_US);
        send_line(p);
        s_win_crit = s_win_ready = s_win_total = 0;
        s_bg_max = 0;
        s_ovh_max = 0;
        for (i = 0; i < NAV_SB_SECTIONS; i++) s_sec_max[i] = 0;
    } else if (s_report_phase == FC_LOOP_HZ / 2u) {          /* half a second later: the nlink line */
        p = line_start(s_buf, 'S', 'N');
        p = put_u(s_buf, p, PSTR("t="), t->tick);
        p = put_u(s_buf, p, PSTR("st="), t->fcu_state);
        p = put_u(s_buf, p, PSTR("fv="), t->fcu_valid);
        p = put_u(s_buf, p, PSTR("fb="), t->fcu_bad);
        p = put_u(s_buf, p, PSTR("ex="), nlink_spi_stats()->frames);
        p = put_u(s_buf, p, PSTR("rs="), nlink_spi_stats()->resyncs);   /* fw09 W1: exchanges its watchdog closed */
        p = put_u(s_buf, p, PSTR("lm="), t->loop_us_max);
        p = put_u(s_buf, p, PSTR("rc="), t->reset_cause);
        p = put_u(s_buf, p, PSTR("rd="), s_reports_dropped);
        send_line(p);
#if !defined(FC_COSIM)
    } else if (s_report_phase == (FC_LOOP_HZ * 3u) / 4u) {   /* a quarter second later: the synthetic FCU's checks */
        p = line_start(s_buf, 'S', 'F');
        p = put_u(s_buf, p, PSTR("t="), t->tick);
        p = put_u(s_buf, p, PSTR("nv="), s_nav_ok);
        p = put_u(s_buf, p, PSTR("nx="), s_nav_bad);
        p = put_u(s_buf, p, PSTR("sg="), s_seq_gaps);
        p = put_u(s_buf, p, PSTR("pm="), s_mode_bad);
        p = put_u(s_buf, p, PSTR("pc="), s_mode_checked);
        p = put_u(s_buf, p, PSTR("ef="), s_fly_noest);
        p = put_u(s_buf, p, PSTR("bc="), s_boot_frames);
        p = put_u(s_buf, p, PSTR("tg="), s_tag_frames);
        p = put_u(s_buf, p, PSTR("cr="), s_corrupt);
        p = put_u(s_buf, p, PSTR("br="), s_bad_returned);
        p = put_u(s_buf, p, PSTR("fb="), t->fcu_bad);
        p = put_u(s_buf, p, PSTR("bl="), s_baro_late);
        p = put_u(s_buf, p, PSTR("al="), s_t_aligned);
        p = put_u(s_buf, p, PSTR("ev="), s_t_estvalid);
        p = put_u(s_buf, p, PSTR("ar="), s_t_armed);
        p = put_u(s_buf, p, PSTR("pw="), s_t_powered);
        p = put_u(s_buf, p, PSTR("ss="), s_state);
        p = put_u(s_buf, p, PSTR("eg="), s_gate_flight);
        p = put_u(s_buf, p, PSTR("ep="), s_gate_pad);
        send_line(p);
#endif
    }
#if FC_VTOL
    /* v0.8.1 lines (each < 200 characters): SW work timing (v0.8), SG the glink side, SH glink timing, SJ the synthetic
     * GNS / the synthetic FCU's slot receiver */
    if (s_report_phase == FC_LOOP_HZ / 8u) {
        p = line_start(s_buf, 'S', 'W');
        p = put_u(s_buf, p, PSTR("t="), t->tick);
        p = put_u(s_buf, p, PSTR("wk="), s_work_win);              /* tick -> mandatory work done, window max, us */
        p = put_u(s_buf, p, PSTR("wp="), s_work_pad);              /* ... pad / flight cumulative max */
        p = put_u(s_buf, p, PSTR("wf="), s_work_flight);
        p = put_u(s_buf, p, PSTR("ce="), s_ce_max);
        p = put_u(s_buf, p, PSTR("lp="), s_late_push);
        p = put_u(s_buf, p, PSTR("ov="), t->overruns);
        p = put_u(s_buf, p, PSTR("sf="), stack_free_min_step());
        p = put_u(s_buf, p, PSTR("ws="), s_work_sum);                /* flight: sum / count = mean work, us */
        p = put_u(s_buf, p, PSTR("wn="), s_work_n);
        p = put_u(s_buf, p, PSTR("wl="), s_work_over_lead);
        p = put_u(s_buf, p, PSTR("wt="), s_work_over_tick);
        p = put_u(s_buf, p, PSTR("ls="), s_late_start_max);
        send_line(p);
        s_work_win = 0;
    } else if (s_report_phase == FC_LOOP_HZ / 4u) {         /* v0.8.1: the NAV's glink side */
        const gl_nav_t *g = t->gl;
        const glink_uart_stats_t *u = glink_uart_stats();
        p = line_start(s_buf, 'S', 'G');
        p = put_u(s_buf, p, PSTR("t="), t->tick);
        p = put_u(s_buf, p, PSTR("ss="), s_gl_states);
        p = put_u(s_buf, p, PSTR("rt="), s_gl_taken);
        p = put_u(s_buf, p, PSTR("la="), g->tx.latched);
        p = put_u(s_buf, p, PSTR("ms="), g->tx.missed);
        p = put_u(s_buf, p, PSTR("rs="), g->tx.results);
        p = put_u(s_buf, p, PSTR("df="), g->rx.stats.frames);
        p = put_u(s_buf, p, PSTR("dc="), g->rx.stats.bad_crc);
        p = put_u(s_buf, p, PSTR("dt="), g->rx.stats.bad_type);
        p = put_u(s_buf, p, PSTR("dw="), g->rx.stats.wrong_dir);
        p = put_u(s_buf, p, PSTR("ds="), g->rx.stats.skipped);
        p = put_u(s_buf, p, PSTR("ux="), u->rx_overflow);
        p = put_u(s_buf, p, PSTR("ue="), u->rx_errors);
        p = put_u(s_buf, p, PSTR("tr="), u->tx_refused + s_gl_refused);
        p = put_u(s_buf, p, PSTR("ub="), u->rx_bytes);
        p = put_u(s_buf, p, PSTR("tb="), u->tx_bytes);
        send_line(p);
    } else if (s_report_phase == (FC_LOOP_HZ * 5u) / 8u) {   /* v0.8.1: glink timing */
        p = line_start(s_buf, 'S', 'H');
        p = put_u(s_buf, p, PSTR("t="), t->tick);
        p = put_u(s_buf, p, PSTR("sa="), s_gl_state_max);
        p = put_u(s_buf, p, PSTR("sb="), s_gl_state_min);
        p = put_u(s_buf, p, PSTR("pb="), s_build_max);
        p = put_u(s_buf, p, PSTR("pn="), s_pre_nmax);
        p = put_u(s_buf, p, PSTR("pt="), s_pre_max);
        p = put_u(s_buf, p, PSTR("gb="), s_gl_sec_max);
#if NAV_SB_SYN_GNS
        p = put_u(s_buf, p, PSTR("ra="), s_arr_max);
        p = put_u(s_buf, p, PSTR("rm="), s_arr_min);
        p = put_u(s_buf, p, PSTR("mn="), (uint16_t)s_marg_min);
        p = put_u(s_buf, p, PSTR("mx="), (uint16_t)s_marg_max);
        p = put_u(s_buf, p, PSTR("gc="), NAV_SB_GNS_CYCLE_US);
#endif
        p = put_u(s_buf, p, PSTR("gl="), GLINK_LATENCY_TICKS);
        send_line(p);
#if !defined(FC_COSIM)
    } else if (s_report_phase == (FC_LOOP_HZ * 7u) / 8u) {   /* v0.8.1: the synthetic GNS and FCU slot checks */
        p = line_start(s_buf, 'S', 'J');
        p = put_u(s_buf, p, PSTR("t="), t->tick);
#if NAV_SB_SYN_GNS
        p = put_u(s_buf, p, PSTR("gs="), s_gs_frames);
        p = put_u(s_buf, p, PSTR("gx="), s_gs_bad);
        p = put_u(s_buf, p, PSTR("gr="), s_gs_sent);
        p = put_u(s_buf, p, PSTR("gk="), s_gs_faults);
#endif
        p = put_u(s_buf, p, PSTR("xs="), s_grx.sets);
        p = put_u(s_buf, p, PSTR("xb="), s_grx.broken);
        p = put_u(s_buf, p, PSTR("xt="), s_grx.tilts);
        p = put_u(s_buf, p, PSTR("ti="), s_tilt_parts);
        p = put_u(s_buf, p, PSTR("tk="), s_tilt_ok);
        p = put_u(s_buf, p, PSTR("tb="), s_tilt_bad);
        p = put_u(s_buf, p, PSTR("tm="), s_tilt_gaps);
        send_line(p);
#endif
    }
#endif
    if (t->tick >= (uint32_t)NAV_SB_SECONDS * FC_LOOP_HZ) {
        const char end[] = "\nSB END\n";
        uart_log_write((const uint8_t *)end, sizeof end - 1);
        while (uart_log_free() < UART_LOG_BUF - 1u)          /* drain the console ring */
            wdt_reset();
        cli();
        sleep_enable();
        sleep_cpu();                                          /* simavr stops on sleep with interrupts off */
    }
}
#endif /* FC_SIMBENCH */
