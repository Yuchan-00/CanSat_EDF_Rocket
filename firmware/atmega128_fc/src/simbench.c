/* simbench.c - see simbench.h (compiled only with -D FC_SIMBENCH). */
#ifdef FC_SIMBENCH
#include "simbench.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/atomic.h>
#include <math.h>

#include "core/attitude_ctrl.h"
#include "core/fc_config.h"
#include "core/ilink_proto.h"
#include "core/link_proto.h"
#include "core/log_record.h"
#include "core/nlink_proto.h"
#include "core/onboard.h"
#include "core/quat.h"
#include "hal/ilink_uart.h"
#include "hal/nlink_spi.h"
#include "hal/pwm.h"
#include "hal/stack.h"
#include "hal/timer.h"
#include "hal/uart_log.h"
#include "nav/nav.h"
#include "params_gen.h"

/* one scratch buffer, static so the report never deepens the measured stack: a SNAP frame taken from the TX ring, the
 * FCU's half of an exchange - never two at a time (v0.8: the report lines are streamed into the log ring, no line buffer;
 * the micro build's boot-time benchmarks still borrow 216 bytes) */
#ifdef FC_SIMBENCH_MICRO
#if LP_RP1
#define SB_SCRATCH_N 220u                  /* v0.9 RP1: il_frame_t 67 B at [96, 163) (SNAP3 65 B) */
#else
#define SB_SCRATCH_N 216u                  /* the ballistic images: the v0.8.1 il_frame_t (64 B) */
#endif
#else
#define SB_SCRATCH_N 72u
#endif
#if !defined(FC_COSIM) || defined(FC_SIMBENCH_MICRO)
static uint8_t s_scratch[SB_SCRATCH_N];
#endif
#if IL_SNAP_FRAME_BYTES > SB_SCRATCH_N || IL_SNAP2_FRAME_BYTES > SB_SCRATCH_N || IL_SNAP3_FRAME_BYTES > SB_SCRATCH_N || \
    NL_FCU_FRAME_BYTES > SB_SCRATCH_N
#error "s_scratch too small"
#endif
/* the FCU's reply snapshot in this build (main.c): IL_SNAP (ballistic) / IL_SNAP2 (VTOL) / IL_SNAP3 (v0.8 VTOL + GPS) */
#if FC_VTOL && ONB_HAS_GPS
#define SB_SNAP_TYPE IL_TYPE_SNAP3
#define SB_SNAP_LEN IL_SNAP3_LEN
#define SB_SNAP_FRAME_BYTES IL_SNAP3_FRAME_BYTES
#elif FC_VTOL
#define SB_SNAP_TYPE IL_TYPE_SNAP2
#define SB_SNAP_LEN IL_SNAP2_LEN
#define SB_SNAP_FRAME_BYTES IL_SNAP2_FRAME_BYTES
#else
#define SB_SNAP_TYPE IL_TYPE_SNAP
#define SB_SNAP_LEN IL_SNAP_LEN
#define SB_SNAP_FRAME_BYTES IL_SNAP_FRAME_BYTES
#endif
static uint16_t s_mark[SB_SECTIONS];
static uint16_t s_sec_max[SB_SECTIONS];
static uint16_t s_win_max, s_win_ctrl_max, s_win_total_max;
static uint32_t s_ctrl_ticks;
static uint16_t s_report_phase;
static uint8_t s_report_due;                               /* bit 0 SB, 1 SN, 2 SL, 3 SI: lines waiting for time */
static uint16_t s_bg_step_max, s_il_step_max, s_snap_step_max;
static uint8_t s_bg_step_kind;                             /* kind of the longest background job (main.c BG_*) */
#if FC_VTOL
/* VTOL report ("SV" line, cumulative unless marked): phases seen, the synthetic plant's height, the throttle, the
 * commands the synthetic ground got acknowledged / rejected, the LANDED and SAFE ticks of the first hop */
static struct {
    uint8_t phases;                                        /* bit (1 << LP_PHASE_*) for every phase seen */
    uint8_t powered_starts;                                /* ticks entering POWERED (motor starts) */
    uint16_t hold_ticks, sat_ticks;                        /* ticks in HOLD / with the controller at its ceiling */
    uint16_t u_max_milli, u_hold_min_milli;                /* ESC command max / min in HOLD (x1000) */
    uint32_t landed_tick, safe_tick;                       /* first LANDED / the SAFE after it */
    uint8_t  tq_max;                                       /* ... the largest thr_q seen */
    uint8_t  td_by;                                        /* ... the signature that confirmed the first touchdown */
    uint8_t  land_reason;                                  /* v0.8.1: the supervisor's reason at the first LAND phase */
    uint32_t land_tick;                                    /* ... and its tick */
    uint16_t own_max;                                      /* v0.8.1: POWERED ticks: the FCU's own work per tick (tick start ->
                                                            * background end, minus the bench's time in between), us max */
    uint32_t own_sum;                                      /* ... sum (mean = own_sum / own_n) */
    uint16_t own_n;
#if ONB_HAS_GPS
    uint8_t  pos_requests, pos_modes;                      /* v0.8: POS_REQ_* seen (OR), bit (1 << POS_MODE_*) seen */
    uint16_t pos_engaged_ticks, tilt_max_mrad;
    uint32_t gate_ok_tick;                                 /* the first FCU tick with the GPS ON gate ok */
    uint16_t fallback_ticks;                               /* v0.8.1: POWERED ticks in the GNS slot's TRIM fallback */
#endif
} s_v = {0, 0, 0, 0, 0, 0xFFFFu, 0, 0, 0, 0, 0, 0, 0, 0, 0
#if ONB_HAS_GPS
         , 0, 0, 0, 0, 0, 0
#endif
};
#endif
#if FC_VTOL
static uint16_t s_le_max;                                  /* v0.8.1: anchor -> the latest late-pass job's end, us */
#endif
#if ONB_HAS_GPS
static uint16_t s_pb_max, s_pf_max;                        /* v0.8: longest position job / late finish at a tick start, us */
static uint16_t s_pz_max;                                  /* anchor -> the end of a position job in the exchange window, us */
static uint16_t s_pj_max;                                  /* fw09: longest position piece (pos_job / pos_job_late), us */
#endif
static uint16_t s_ovh_max;                                 /* simbench_end_of_loop's own time (peers, report), us */
static uint8_t s_nw_max, s_nb_max, s_bb_max;               /* ilink bytes waiting at a tick / decoded in step 2 / bg share */
static uint16_t s_bj_max, s_bt_max;                        /* longest job on BG_RESERVE_US / a text job, us */
static uint16_t s_br_max;                                  /* longest record job (not SNAP / ilink / text), us (VTOL: on
                                                            * REC_RESERVE_US) */
#if FC_VTOL
static uint16_t s_nj_max, s_nt_max, s_nr_max;              /* fw09 integration: bj / bt / br of the NORMAL pass only (jobs that
                                                            * end before BG_END_US; the late pass's jobs: only in bj / bt / br) */
#endif
static uint16_t s_pw_max, s_pc_max;                        /* tick start -> vanes written (all / controller active), us */
static uint16_t s_lm_max;                                  /* exchange complete -> tick start, the FCU's own share, us */
static uint16_t s_ls_max;                                  /* ... the simbench's share (bypass injection, bookkeeping), us */
static uint16_t s_la_max, s_lc_max;                        /* NAV tick -> vanes written (all / controller active), us */
static int16_t s_bm_min = 32767;                           /* background end -> next exchange start, us (single-chip) */
static uint16_t s_be_max;                                  /* anchor -> background end, us (main.c BG_END_US) */
static uint16_t s_je_max;                                  /* anchor -> the end of the latest background job, us */
static uint16_t s_bn_max;                                  /* the same without the emulated RX interrupt time burned
                                                            * after the tick start (v0.7, single-chip): the real ISRs run
                                                            * as the bytes arrive, the bypass lumps them at its polls */
#if !defined(FC_COSIM)
static uint32_t s_tick_burned;                             /* ISR emulation burned since the tick start, counts */
#endif
static uint8_t s_prev_state;
static uint32_t s_abort_tick;
static uint8_t s_abort_reason;

#define SB_BG_SNAP 1u                                      /* main.c BG_SNAP, BG_ILINK, BG_COMTEXT, BG_NAVTEXT, BG_POS */
#define SB_BG_ILINK 2u
#define SB_BG_COMTEXT 7u
#define SB_BG_NAVTEXT 8u
#define SB_BG_POS 10u
#define SB_BG_STATUS 12u                                   /* fw09: main.c BG_STATUS, the status record (not a bg_job job) */
#define SB_BG_POSNOW 13u                                   /* fw09 integration: main.c BG_POSNOW, pos_work_now (no reserve) */

#if ONB_HAS_GPS
void simbench_pos_finish(uint16_t us)
{
    if (us > s_pf_max) s_pf_max = us;
}

#endif

void simbench_bg_step(uint16_t us, uint8_t kind)
{
#if FC_VTOL
    const bool late = (kind & SB_BG_LATE) != 0u;          /* fw09 integration: the late pass (LATE_WALL(reserve)) */
    kind = (uint8_t)(kind & (uint8_t)~SB_BG_LATE);
#endif
#if ONB_HAS_GPS
    /* fw09 integration (section 7 row 14): pos_work_now runs every tick right after the time-critical part, on no reserve;
     * a position PIECE (pos_job on POS_RESERVE_US, pos_job_late on POS_WALL_*) runs only when pos_work_now left work.
     * "pb" keeps both (as before), "pj" the pieces only - the jobs a reserve must cover */
    if (kind == SB_BG_POSNOW)
        kind = SB_BG_POS;
    else if (kind == SB_BG_POS && us > s_pj_max)
        s_pj_max = us;
    if (kind == SB_BG_POS && us > s_pb_max)
        s_pb_max = us;
#endif
    if (kind == SB_BG_ILINK) {                             /* chunk-wise, time-checked before every chunk */
        if (us > s_il_step_max) s_il_step_max = us;
        return;
    }
#if FC_VTOL
    /* fw09 (WP-F, the reserve re-derivation, main.c): every job against the reserve it STARTS on - BG_RESERVE_US: the
     * SNAP frame and the status record ("bj"); REC_RESERVE_US: the record jobs ("br"); COM_TEXT_RESERVE_US: the text jobs
     * ("bt"); the position work (pos_work_now right after the time-critical part: no reserve; its pieces: POS_RESERVE_US /
     * POS_WALL_*) only in "pb".  Before, every job but the text ones counted in bj and br: the GPS builds compared their
     * tilt work (~0.36 ms, pos_work_now) with BG_RESERVE_US and REC_RESERVE_US.  The status record stays out of bg / bk */
    if (kind == SB_BG_STATUS) {
        if (us > s_bj_max) s_bj_max = us;
        if (!late && us > s_nj_max) s_nj_max = us;
        return;
    }
#endif
    if (us > s_bg_step_max) {
        s_bg_step_max = us;
        s_bg_step_kind = kind;
    }
#if FC_VTOL
    if (kind == SB_BG_COMTEXT || kind == SB_BG_NAVTEXT) {
        if (us > s_bt_max) s_bt_max = us;
        if (!late && us > s_nt_max) s_nt_max = us;
    } else if (kind == SB_BG_SNAP) {
        if (us > s_bj_max) s_bj_max = us;
        if (!late && us > s_nj_max) s_nj_max = us;
    }
    if (kind != SB_BG_SNAP && kind != SB_BG_COMTEXT && kind != SB_BG_NAVTEXT && kind != SB_BG_POS) {
        if (us > s_br_max) s_br_max = us;
        if (!late && us > s_nr_max) s_nr_max = us;
    }
#else                                                      /* ballistic: every job but the texts starts on BG_RESERVE_US */
    if (kind == SB_BG_COMTEXT || kind == SB_BG_NAVTEXT) {
        if (us > s_bt_max) s_bt_max = us;
    } else if (us > s_bj_max) {
        s_bj_max = us;
    }
    if (kind != SB_BG_SNAP && kind != SB_BG_COMTEXT && kind != SB_BG_NAVTEXT && us > s_br_max)
        s_br_max = us;
#endif
    if (kind == SB_BG_SNAP && us > s_snap_step_max) s_snap_step_max = us;
}

uint16_t simbench_adc(void)
{
    return (uint16_t)(4.0f * (float)VBAT_CELLS / VBAT_V_PER_COUNT);   /* 4.0 V per cell */
}

#if PM_ANY
void simbench_adc_chain(uint16_t codes[3])
{
    codes[PM_CH_IN] = (uint16_t)(25.0f / VBAT_V_PER_COUNT + 0.5f);
    codes[PM_CH_TET] = PM_BUF ? (uint16_t)(25.0f / VBAT_V_PER_COUNT + 0.5f) : PM_CODE_NONE;
    codes[PM_CH_BUF] = PM_BUF ? (uint16_t)(6.0f * 3.80f / VBAT_V_PER_COUNT + 0.5f) : PM_CODE_NONE;
}
#endif

bool simbench_arm_switch(void)
{
    return true;                                           /* closed: the vehicle may be armed */
}

void simbench_mark(uint8_t i)
{
    s_mark[i] = timer_tcnt1();
}

/* numbers are printed in hex ("0x..."), nibble by nibble from the bytes (no 32-bit division or variable shift: the
 * report runs in the loop it measures, ~10 us per number) */
static uint8_t put_u_P(char *buf, uint8_t pos, const char *key_P, uint32_t v)
{
    static const char hex[] PROGMEM = "0123456789abcdef";
    const uint8_t b[4] = {(uint8_t)(v >> 24), (uint8_t)(v >> 16), (uint8_t)(v >> 8), (uint8_t)v};
    uint8_t k;
    bool lead = true;
    char c;
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

#define put_u(buf, pos, key, v) put_u_P(buf, pos, PSTR(key), v)    /* report keys live in flash */

static uint8_t line_start(char *buf, char c1, char c2) __attribute__((unused));
static uint8_t line_start(char *buf, char c1, char c2)
{
    buf[0] = '\n';
    buf[1] = c1;
    buf[2] = c2;
    buf[3] = ' ';
    return 4;
}

#define SB_US(us) ((uint32_t)(us) * FC_PWM_COUNTS_PER_US)                  /* us -> Timer1 counts */
#define SB_TICK_COUNTS SB_US(FC_TICK_US)                                   /* Timer1 counts per control tick */

/* ======================================================================================== */
/* single-chip: the simbench's clock, the synthetic NAV and the synthetic COM                */
/* ======================================================================================== */
#if !defined(FC_COSIM)
/* Timer1 counts since simbench_start, 32 bit: advanced by the Timer1 time between two calls (it wraps every PWM frame,
 * 20 ms; the loop calls it far more often) */
static uint32_t s_clk;
static uint16_t s_clk_t1;
static bool s_run;

static uint32_t sb_now(void)
{
    const uint16_t c = timer_tcnt1();
    s_clk += timer_elapsed_counts(s_clk_t1, c);
    s_clk_t1 = c;
    return s_clk;
}

static uint16_t counts_to_us16(uint32_t c)
{
    c /= FC_PWM_COUNTS_PER_US;
    return c > 0xFFFFu ? 0xFFFFu : (uint16_t)c;
}

/* ---------------------------------------------------------------- the synthetic NAV */
/* how much earlier than SB_NAV_FRAME_US after its tick each frame completes, us (pattern of 16; max SB_NAV_JITTER_US) */
static const uint8_t k_nav_jit[16] PROGMEM = {0, 25, 50, 0, 125, 10, 250, 0, 75, 200, 30, 0, 150, 100, 5, 225};
#if SB_NAV_JITTER_US != 250u
#error "k_nav_jit's largest entry must be SB_NAV_JITTER_US"
#endif

static uint32_t nav_frame_time(uint16_t k)                 /* NAV tick k's frame: its last byte at the FCU */
{
    return (uint32_t)k * SB_TICK_COUNTS + SB_US(SB_NAV_FRAME_US) - SB_US(pgm_read_byte(&k_nav_jit[k & 15u]));
}

/* the NAV reset: ballistic at a fixed NAV tick (v0.6.3); VTOL SB_NAV_RESET_AFTER_ON2_TICKS after the second hop's motor
 * start (set in simbench_end_of_loop; 0xFFFF = not yet) */
#if FC_VTOL
static uint16_t s_nav_reset_k = 0xFFFFu;
#define SB_NAV_RESET_TICK s_nav_reset_k
#else
#define SB_NAV_RESET_TICK SB_NAV_RESET_TICK_BALLISTIC
#endif

static bool nav_silent(uint16_t k)                         /* no exchange for NAV tick k */
{
    return k == SB_NAV_MISS_TICK || (k >= SB_NAV_GAP_TICK && k < SB_NAV_GAP_TICK + SB_NAV_GAP_TICKS) ||
           ((uint32_t)k >= SB_NAV_RESET_TICK && (uint32_t)k < (uint32_t)SB_NAV_RESET_TICK + SB_NAV_RESET_TICKS);
}

#if FC_VTOL
/* VTOL: the flight trajectory's per-index values, in flash.  The ballistic bench (v0.6.3) computes them in nav_build from
 * a Q15 sine table (sin64(i) = sin(2 pi i / 64)): roll r = 0.07 sin64(ir), pitch p = 0.08 sin64(ip + 16), yaw
 * y = 0.09 sin64(iy), rates 0.07 2 pi 1.3 sin64(ir + 16), 0.08 2 pi 0.9 sin64(ip + 32), 0.09 2 pi 0.5 sin64(iy + 16).  The
 * VTOL bench tabulates exactly those float32 values (generated with numpy float32 in the same operation order and
 * checked on the AVR with avr-gcc / avr-libc: 0 of 640 values differ), so both benches feed the FCU the same attitude;
 * the bench's own work after each FCU tick must fit the ~0.5 ms shorter idle time the VTOL FCU leaves (without the
 * tables a babble window made the simulated FCU fall behind its frames - a bench artifact, measured), and these ~5000
 * cycles were most of it.  The ballistic bench keeps the v0.6.3 arithmetic: its timing phase (which tick an early
 * frame or a lumped RX-interrupt emulation lands on) stays the v0.6.3 one.  k_tr_h* = 0.5 x angle (the quaternion's
 * vector part), k_tr_xx = angle^2, k_tr_w* = the body rate. */
static const float k_tr_hr[64] PROGMEM = {  /* 0.5 x 0.07 sin64(i + 0): q component */
    0.0f, 0.0034307860769331455f, 0.006828460842370987f, 0.010159912519156933f, 0.013393096625804901f,
    0.016498107463121414f, 0.019443970173597336f, 0.022202910855412483f, 0.02474823035299778f, 0.02705429121851921f,
    0.029100799933075905f, 0.030866393819451332f, 0.032335054129362106f, 0.033491820096969604f, 0.03432602062821388f,
    0.03483017161488533f, 0.03499893099069595f, 0.03483017161488533f, 0.03432602062821388f, 0.033491820096969604f,
    0.032335054129362106f, 0.030866393819451332f, 0.029100799933075905f, 0.02705429121851921f, 0.02474823035299778f,
    0.022202910855412483f, 0.019443970173597336f, 0.016498107463121414f, 0.013393096625804901f, 0.010159912519156933f,
    0.006828460842370987f, 0.0034307860769331455f, 0.0f, -0.0034307860769331455f, -0.006828460842370987f,
    -0.010159912519156933f, -0.013393096625804901f, -0.016498107463121414f, -0.019443970173597336f,
    -0.022202910855412483f, -0.02474823035299778f, -0.02705429121851921f, -0.029100799933075905f,
    -0.030866393819451332f, -0.032335054129362106f, -0.033491820096969604f, -0.03432602062821388f,
    -0.03483017161488533f, -0.03499893099069595f, -0.03483017161488533f, -0.03432602062821388f,
    -0.033491820096969604f, -0.032335054129362106f, -0.030866393819451332f, -0.029100799933075905f,
    -0.02705429121851921f, -0.02474823035299778f, -0.022202910855412483f, -0.019443970173597336f,
    -0.016498107463121414f, -0.013393096625804901f, -0.010159912519156933f, -0.006828460842370987f,
    -0.0034307860769331455f};
static const float k_tr_rr[64] PROGMEM = {  /* (0.07 sin64(i + 0))^2 */
    0.0f, 4.708117194240913e-05f, 0.00018651150458026677f, 0.0004128952859900892f, 0.0007175001665018499f,
    0.0010887501994147897f, 0.0015122719341889024f, 0.001971876947209239f, 0.0024498996790498495f,
    0.002927738707512617f, 0.003387426258996129f, 0.00381093705072999f, 0.00418222276493907f, 0.0044868080876767635f,
    0.004713102709501982f, 0.0048525636084377766f, 0.00489970063790679f, 0.0048525636084377766f,
    0.004713102709501982f, 0.0044868080876767635f, 0.00418222276493907f, 0.00381093705072999f, 0.003387426258996129f,
    0.002927738707512617f, 0.0024498996790498495f, 0.001971876947209239f, 0.0015122719341889024f,
    0.0010887501994147897f, 0.0007175001665018499f, 0.0004128952859900892f, 0.00018651150458026677f,
    4.708117194240913e-05f, 0.0f, 4.708117194240913e-05f, 0.00018651150458026677f, 0.0004128952859900892f,
    0.0007175001665018499f, 0.0010887501994147897f, 0.0015122719341889024f, 0.001971876947209239f,
    0.0024498996790498495f, 0.002927738707512617f, 0.003387426258996129f, 0.00381093705072999f, 0.00418222276493907f,
    0.0044868080876767635f, 0.004713102709501982f, 0.0048525636084377766f, 0.00489970063790679f,
    0.0048525636084377766f, 0.004713102709501982f, 0.0044868080876767635f, 0.00418222276493907f, 0.00381093705072999f,
    0.003387426258996129f, 0.002927738707512617f, 0.0024498996790498495f, 0.001971876947209239f,
    0.0015122719341889024f, 0.0010887501994147897f, 0.0007175001665018499f, 0.0004128952859900892f,
    0.00018651150458026677f, 4.708117194240913e-05f};
static const float k_tr_wr[64] PROGMEM = {  /* 0.07 2 pi 1.3 sin64(i + 16): body rate */
    0.5717523694038391f, 0.5689954161643982f, 0.5607594847679138f, 0.5471317768096924f, 0.5282344818115234f,
    0.5042420625686646f, 0.47539883852005005f, 0.4419665038585663f, 0.40429404377937317f, 0.3627130091190338f,
    0.3176421523094177f, 0.269517719745636f, 0.21879339218139648f, 0.1659751832485199f, 0.11155164986848831f,
    0.05604628473520279f, 0.0f, -0.05604628473520279f, -0.11155164986848831f, -0.1659751832485199f,
    -0.21879339218139648f, -0.269517719745636f, -0.3176421523094177f, -0.3627130091190338f, -0.40429404377937317f,
    -0.4419665038585663f, -0.47539883852005005f, -0.5042420625686646f, -0.5282344818115234f, -0.5471317768096924f,
    -0.5607594847679138f, -0.5689954161643982f, -0.5717523694038391f, -0.5689954161643982f, -0.5607594847679138f,
    -0.5471317768096924f, -0.5282344818115234f, -0.5042420625686646f, -0.47539883852005005f, -0.4419665038585663f,
    -0.40429404377937317f, -0.3627130091190338f, -0.3176421523094177f, -0.269517719745636f, -0.21879339218139648f,
    -0.1659751832485199f, -0.11155164986848831f, -0.05604628473520279f, 0.0f, 0.05604628473520279f,
    0.11155164986848831f, 0.1659751832485199f, 0.21879339218139648f, 0.269517719745636f, 0.3176421523094177f,
    0.3627130091190338f, 0.40429404377937317f, 0.4419665038585663f, 0.47539883852005005f, 0.5042420625686646f,
    0.5282344818115234f, 0.5471317768096924f, 0.5607594847679138f, 0.5689954161643982f};
static const float k_tr_hp[64] PROGMEM = {  /* 0.5 x 0.08 sin64(i + 16): q component */
    0.03999877721071243f, 0.039805907756090164f, 0.03922973573207855f, 0.038276366889476776f, 0.03695434331893921f,
    0.03527587652206421f, 0.03325805440545082f, 0.030919188633561134f, 0.02828369103372097f, 0.025374755263328552f,
    0.022221678867936134f, 0.018854979425668716f, 0.015306396409869194f, 0.011611327528953552f, 0.007803955115377903f,
    0.0039208983071148396f, 0.0f, -0.0039208983071148396f, -0.007803955115377903f, -0.011611327528953552f,
    -0.015306396409869194f, -0.018854979425668716f, -0.022221678867936134f, -0.025374755263328552f,
    -0.02828369103372097f, -0.030919188633561134f, -0.03325805440545082f, -0.03527587652206421f,
    -0.03695434331893921f, -0.038276366889476776f, -0.03922973573207855f, -0.039805907756090164f,
    -0.03999877721071243f, -0.039805907756090164f, -0.03922973573207855f, -0.038276366889476776f,
    -0.03695434331893921f, -0.03527587652206421f, -0.03325805440545082f, -0.030919188633561134f,
    -0.02828369103372097f, -0.025374755263328552f, -0.022221678867936134f, -0.018854979425668716f,
    -0.015306396409869194f, -0.011611327528953552f, -0.007803955115377903f, -0.0039208983071148396f, 0.0f,
    0.0039208983071148396f, 0.007803955115377903f, 0.011611327528953552f, 0.015306396409869194f,
    0.018854979425668716f, 0.022221678867936134f, 0.025374755263328552f, 0.02828369103372097f, 0.030919188633561134f,
    0.03325805440545082f, 0.03527587652206421f, 0.03695434331893921f, 0.038276366889476776f, 0.03922973573207855f,
    0.039805907756090164f};
static const float k_tr_pp[64] PROGMEM = {  /* (0.08 sin64(i + 16))^2 */
    0.0063996086828410625f, 0.00633804127573967f, 0.0061558885499835014f, 0.005860321223735809f,
    0.005462493747472763f, 0.004977549891918898f, 0.004424392711371183f, 0.003823984880000353f,
    0.0031998686026781797f, 0.002575512742623687f, 0.001975212013348937f, 0.0014220409793779254f,
    0.0009371431078761816f, 0.0005392917082644999f, 0.0002436068607494235f, 6.149377441033721e-05f, 0.0f,
    6.149377441033721e-05f, 0.0002436068607494235f, 0.0005392917082644999f, 0.0009371431078761816f,
    0.0014220409793779254f, 0.001975212013348937f, 0.002575512742623687f, 0.0031998686026781797f,
    0.003823984880000353f, 0.004424392711371183f, 0.004977549891918898f, 0.005462493747472763f, 0.005860321223735809f,
    0.0061558885499835014f, 0.00633804127573967f, 0.0063996086828410625f, 0.00633804127573967f,
    0.0061558885499835014f, 0.005860321223735809f, 0.005462493747472763f, 0.004977549891918898f,
    0.004424392711371183f, 0.003823984880000353f, 0.0031998686026781797f, 0.002575512742623687f,
    0.001975212013348937f, 0.0014220409793779254f, 0.0009371431078761816f, 0.0005392917082644999f,
    0.0002436068607494235f, 6.149377441033721e-05f, 0.0f, 6.149377441033721e-05f, 0.0002436068607494235f,
    0.0005392917082644999f, 0.0009371431078761816f, 0.0014220409793779254f, 0.001975212013348937f,
    0.002575512742623687f, 0.0031998686026781797f, 0.003823984880000353f, 0.004424392711371183f,
    0.004977549891918898f, 0.005462493747472763f, 0.005860321223735809f, 0.0061558885499835014f, 0.00633804127573967f};
static const float k_tr_wp[64] PROGMEM = {  /* 0.08 2 pi 0.9 sin64(i + 32): body rate */
    0.0f, -0.04434431716799736f, -0.08826065808534622f, -0.1313210278749466f, -0.17311127483844757f,
    -0.21324481070041656f, -0.25132128596305847f, -0.2869817316532135f, -0.31988102197647095f, -0.3496878147125244f,
    -0.376139760017395f, -0.39896079897880554f, -0.4179438054561615f, -0.4328955411911011f, -0.4436778724193573f,
    -0.45019423961639404f, -0.4523755609989166f, -0.45019423961639404f, -0.4436778724193573f, -0.4328955411911011f,
    -0.4179438054561615f, -0.39896079897880554f, -0.376139760017395f, -0.3496878147125244f, -0.31988102197647095f,
    -0.2869817316532135f, -0.25132128596305847f, -0.21324481070041656f, -0.17311127483844757f, -0.1313210278749466f,
    -0.08826065808534622f, -0.04434431716799736f, 0.0f, 0.04434431716799736f, 0.08826065808534622f,
    0.1313210278749466f, 0.17311127483844757f, 0.21324481070041656f, 0.25132128596305847f, 0.2869817316532135f,
    0.31988102197647095f, 0.3496878147125244f, 0.376139760017395f, 0.39896079897880554f, 0.4179438054561615f,
    0.4328955411911011f, 0.4436778724193573f, 0.45019423961639404f, 0.4523755609989166f, 0.45019423961639404f,
    0.4436778724193573f, 0.4328955411911011f, 0.4179438054561615f, 0.39896079897880554f, 0.376139760017395f,
    0.3496878147125244f, 0.31988102197647095f, 0.2869817316532135f, 0.25132128596305847f, 0.21324481070041656f,
    0.17311127483844757f, 0.1313210278749466f, 0.08826065808534622f, 0.04434431716799736f};
static const float k_tr_hy[64] PROGMEM = {  /* 0.5 x 0.09 sin64(i + 0): q component */
    0.0f, 0.004411011002957821f, 0.008779449388384819f, 0.013062744401395321f, 0.017219696193933487f,
    0.021211853250861168f, 0.02499939128756523f, 0.02854660153388977f, 0.03181915357708931f, 0.034784089773893356f,
    0.037415314465761185f, 0.039685364812612534f, 0.04157363995909691f, 0.04306091368198395f, 0.04413345456123352f,
    0.0447816476225853f, 0.04499862715601921f, 0.0447816476225853f, 0.04413345456123352f, 0.04306091368198395f,
    0.04157363995909691f, 0.039685364812612534f, 0.037415314465761185f, 0.034784089773893356f, 0.03181915357708931f,
    0.02854660153388977f, 0.02499939128756523f, 0.021211853250861168f, 0.017219696193933487f, 0.013062744401395321f,
    0.008779449388384819f, 0.004411011002957821f, 0.0f, -0.004411011002957821f, -0.008779449388384819f,
    -0.013062744401395321f, -0.017219696193933487f, -0.021211853250861168f, -0.02499939128756523f,
    -0.02854660153388977f, -0.03181915357708931f, -0.034784089773893356f, -0.037415314465761185f,
    -0.039685364812612534f, -0.04157363995909691f, -0.04306091368198395f, -0.04413345456123352f, -0.0447816476225853f,
    -0.04499862715601921f, -0.0447816476225853f, -0.04413345456123352f, -0.04306091368198395f, -0.04157363995909691f,
    -0.039685364812612534f, -0.037415314465761185f, -0.034784089773893356f, -0.03181915357708931f,
    -0.02854660153388977f, -0.02499939128756523f, -0.021211853250861168f, -0.017219696193933487f,
    -0.013062744401395321f, -0.008779449388384819f, -0.004411011002957821f};
static const float k_tr_yy[64] PROGMEM = {  /* (0.09 sin64(i + 0))^2 */
    0.0f, 7.782807369949296e-05f, 0.0003083149204030633f, 0.0006825411692261696f, 0.0011860717786476016f,
    0.0017997708637267351f, 0.0024998781736940145f, 0.0032596339005976915f, 0.004049834329634905f,
    0.004839731380343437f, 0.005599623080343008f, 0.00629971269518137f, 0.006913470104336739f, 0.0074169691652059555f,
    0.00779104745015502f, 0.008021583780646324f, 0.008099505677819252f, 0.008021583780646324f, 0.00779104745015502f,
    0.0074169691652059555f, 0.006913470104336739f, 0.00629971269518137f, 0.005599623080343008f, 0.004839731380343437f,
    0.004049834329634905f, 0.0032596339005976915f, 0.0024998781736940145f, 0.0017997708637267351f,
    0.0011860717786476016f, 0.0006825411692261696f, 0.0003083149204030633f, 7.782807369949296e-05f, 0.0f,
    7.782807369949296e-05f, 0.0003083149204030633f, 0.0006825411692261696f, 0.0011860717786476016f,
    0.0017997708637267351f, 0.0024998781736940145f, 0.0032596339005976915f, 0.004049834329634905f,
    0.004839731380343437f, 0.005599623080343008f, 0.00629971269518137f, 0.006913470104336739f, 0.0074169691652059555f,
    0.00779104745015502f, 0.008021583780646324f, 0.008099505677819252f, 0.008021583780646324f, 0.00779104745015502f,
    0.0074169691652059555f, 0.006913470104336739f, 0.00629971269518137f, 0.005599623080343008f, 0.004839731380343437f,
    0.004049834329634905f, 0.0032596339005976915f, 0.0024998781736940145f, 0.0017997708637267351f,
    0.0011860717786476016f, 0.0006825411692261696f, 0.0003083149204030633f, 7.782807369949296e-05f};
static const float k_tr_wy[64] PROGMEM = {  /* 0.09 2 pi 0.5 sin64(i + 16): body rate */
    0.2827347218990326f, 0.28137141466140747f, 0.2772986888885498f, 0.27055972814559937f, 0.2612149119377136f,
    0.24935051798820496f, 0.23508736491203308f, 0.21855488419532776f, 0.19992564618587494f, 0.17936359345912933f,
    0.15707580745220184f, 0.1332780122756958f, 0.10819455236196518f, 0.08207564800977707f, 0.05516291409730911f,
    0.027715200558304787f, 0.0f, -0.027715200558304787f, -0.05516291409730911f, -0.08207564800977707f,
    -0.10819455236196518f, -0.1332780122756958f, -0.15707580745220184f, -0.17936359345912933f, -0.19992564618587494f,
    -0.21855488419532776f, -0.23508736491203308f, -0.24935051798820496f, -0.2612149119377136f, -0.27055972814559937f,
    -0.2772986888885498f, -0.28137141466140747f, -0.2827347218990326f, -0.28137141466140747f, -0.2772986888885498f,
    -0.27055972814559937f, -0.2612149119377136f, -0.24935051798820496f, -0.23508736491203308f, -0.21855488419532776f,
    -0.19992564618587494f, -0.17936359345912933f, -0.15707580745220184f, -0.1332780122756958f, -0.10819455236196518f,
    -0.08207564800977707f, -0.05516291409730911f, -0.027715200558304787f, 0.0f, 0.027715200558304787f,
    0.05516291409730911f, 0.08207564800977707f, 0.10819455236196518f, 0.1332780122756958f, 0.15707580745220184f,
    0.17936359345912933f, 0.19992564618587494f, 0.21855488419532776f, 0.23508736491203308f, 0.24935051798820496f,
    0.2612149119377136f, 0.27055972814559937f, 0.2772986888885498f, 0.28137141466140747f};
#else
/* sin(2 pi i / 64) in Q15 (flash: the bench's static RAM is the stack's) */
static const int16_t k_sin64[64] PROGMEM = {
    0, 3212, 6393, 9512, 12539, 15446, 18204, 20787, 23170, 25329, 27245, 28898, 30273, 31356, 32137, 32609, 32767,
    32609, 32137, 31356, 30273, 28898, 27245, 25329, 23170, 20787, 18204, 15446, 12539, 9512, 6393, 3212, 0,
    -3212, -6393, -9512, -12539, -15446, -18204, -20787, -23170, -25329, -27245, -28898, -30273, -31356, -32137,
    -32609, -32767, -32609, -32137, -31356, -30273, -28898, -27245, -25329, -23170, -20787, -18204, -15446, -12539,
    -9512, -6393, -3212};
static float sin64(uint8_t i)
{
    return (float)(int16_t)pgm_read_word(&k_sin64[i & 63u]) * (1.0f / 32768.0f);
}
#endif
static uint16_t s_nav_k = SB_NAV_FIRST_TICK;               /* NAV tick of the next frame */
static bool s_nav_built, s_nav_reply_new;
/* frame s_nav_k, built ahead (simbench_end_of_loop) - v0.8: in place, in the buffer nlink_spi's bypass fills next
 * (nlink_spi_inject_buf: 72 bytes of RAM less) */
#define s_nav_buf (nlink_spi_inject_buf())
#if FC_VTOL
static nl_gns_tx_t s_gtx;                                  /* v0.8.1: the synthetic NAV's GNS slot transmitter (nlink v4) */
static bool s_gtx_init, s_nav_motor;                       /* ... initialised; the FCU4 reply's MOTOR_ON (the synthetic GNS) */
static uint16_t s_gns_results, s_gns_dropped;              /* synthetic GNS: RESULTs handed to the NAV / cycles left out */
#endif
static uint32_t s_nav_f;                                   /* completion time of the last injected frame */
static uint16_t s_nav_fk;                                  /* its NAV tick */
static bool s_nav_pad = true, s_nav_fcu_seen, s_nav_aligned;
static uint16_t s_nav_pad_ticks, s_nav_seq0;
static uint8_t s_nav_reset = 0x01;                         /* MCUCSR of the NAV's boot: power-on, then watchdog */
static float s_nav_h;
static uint16_t s_nav_built_late;                          /* frames that had to be built at their due time */
static uint32_t s_fcu_tick, s_nav_reset_fcu_tick;          /* the FCU's current tick / when the NAV went silent to reset */
static uint16_t s_tq_bad;                                  /* v0.7.1: FCU frames whose thr_q byte breaks its rule (SI "tq") */

#if FC_VTOL
/* VTOL: a vertical point mass flown by the FCU's own ESC command (simbench_end_of_loop copies sb_tick_t.esc_u, the
 * command written in the FCU's previous tick): thrust T = T_max u_lag^n with the EDF's first-order lag (EDF_TAU_S),
 * u_hover = (m g / T_max)^(1/n) from params_gen.h, so a = g ((u_lag / u_hover)^n cos(tilt) - 1); no drag, no ground
 * effect, no wind; the ground at h = 0 (the pad) stops it.  Timing input only: the NAV's altitude / v_z in the frame
 * are this plant's exact values (the NAV estimator is not simulated here). */
static struct {
    float h, v, a, u_lag, u_cmd, h_max, td_v;              /* m, m/s, m/s^2, -, -, m, the hardest ground contact m/s */
    float f_z;                                             /* v0.7.1: body-z specific force T / m (g on the ground), m/s^2 */
    bool airborne;
} s_pl;

static void plant_step(float cos_tilt)
{
#if VC_THROTTLE_EXP_INT == 2
    const float r = s_pl.u_lag * (1.0f / VC_U_HOVER), thrust = r * r;
#else
    const float thrust = powf(s_pl.u_lag * (1.0f / VC_U_HOVER), VC_THROTTLE_EXP);
#endif
    float a = FC_G0 * (thrust * cos_tilt - 1.0f);
    s_pl.u_lag += (s_pl.u_cmd - s_pl.u_lag) * (FC_DT_S / (EDF_TAU_S + FC_DT_S));
    s_pl.f_z = FC_G0 * thrust;                             /* in the air: the thrust's specific force along the body z */
    if (!s_pl.airborne && a <= 0.0f) {                     /* on the ground: the weight is carried */
        s_pl.a = 0.0f;
        s_pl.v = 0.0f;
        s_pl.f_z = FC_G0;                                  /* the ground carries the rest: the accelerometer reads 1 g */
        return;
    }
    s_pl.airborne = true;
    s_pl.a = a;
    s_pl.v += a * FC_DT_S;
    s_pl.h += s_pl.v * FC_DT_S;
    if (s_pl.h > s_pl.h_max) s_pl.h_max = s_pl.h;
    if (s_pl.h <= 0.0f && s_pl.v < 0.0f) {                 /* ground contact */
        if (-s_pl.v > s_pl.td_v) s_pl.td_v = -s_pl.v;
        s_pl.h = s_pl.v = s_pl.a = 0.0f;
        s_pl.f_z = FC_G0;
        s_pl.airborne = false;
    }
}
#endif

/* the trajectory: pad - still, roll 1 deg, pitch -0.5 deg; flight - roll / pitch / yaw = A sin(2 pi f t + phi) with
 * f 1.3 / 0.9 / 0.5 Hz, A 0.07 / 0.08 / 0.09 rad, phi 0 / 90 / 0 deg (rates A 2 pi f cos), climbing 3 m/s, a thrust
 * specific force; q to first order in the small angles (unit to ~1e-4) */
#define NAV_STEP(f) ((uint16_t)((f) * 65536.0f / (float)NAV_TICK_HZ + 0.5f))
#if FC_VTOL
/* v0.8.1: the synthetic GNS behind the synthetic NAV.  The RESULT of cycle m (the glink STATE of NAV tick 5 m + 4) reaches
 * the NAV in its tick 5 m + 6 (L = GLINK_LATENCY_TICKS >= 2: in time for the frame build of 5 m + 4 + L - the two-core
 * harness measures the real arrival) and goes through the NAV's own slot transmitter (core/nlink_proto.h nl_gns_tx): TILT
 * 0.02 rad oscillating in x (5.12 s) / 0.01 rad in y while the FCU's reply says the motor runs (else NL_TILT_NONE), the ON
 * gate's flags once the NAV's pad alignment is done, 9 satellites, hAcc 1.8 m, POS mode with the hold point while the
 * motor runs, POS on a 0.3 m / 0.2 m ellipse, small VEL / TRIM.  No RESULT for the first second after a NAV boot (the GNS
 * answers its first STATE) and none for SB_GNS_DROP_CYCLES cycles from SB_GNS_DROP_S (the FCU's TRIM fallback: >= 2 missed
 * cycles, fewer than land_steps - no V11).  Timing inputs, not a GNS model. */
#define SB_GNS_DROP_S 15u
#define SB_GNS_DROP_CYCLES 4u
static void nav_gns_result(uint16_t k)
{
    uint8_t pl[NL_GNS_RESULT_LEN];
    gl_result_t r;
    uint16_t m;
    const uint8_t i = (uint8_t)(k >> 4);                   /* 64 steps of 80 ms: 5.12 s */
    if (!s_gtx_init) {
        nl_gns_tx_init(&s_gtx, 5u, GLINK_LATENCY_TICKS);
        s_gtx_init = true;
    }
    if (k < 6u || (k % 5u) != 1u)
        return;                                            /* the RESULT of cycle m arrives in NAV tick 5 m + 6 */
    m = (uint16_t)((k - 6u) / 5u);
    if ((uint16_t)(k - s_nav_seq0) < (uint16_t)FC_LOOP_HZ)
        return;                                            /* the GNS's first answer after a NAV boot */
    if ((uint32_t)k >= (uint32_t)SB_GNS_DROP_S * FC_LOOP_HZ &&
        (uint32_t)k < (uint32_t)SB_GNS_DROP_S * FC_LOOP_HZ + 5u * SB_GNS_DROP_CYCLES) {
        s_gns_dropped++;
        return;                                            /* the GNS dropout: no RESULT */
    }
    r.tilt[0] = s_nav_motor ? (int16_t)(200.0f * pgm_read_float(&k_tr_hr[i]) * (1.0f / 0.035f)) : NL_TILT_NONE;
    r.tilt[1] = s_nav_motor ? (int16_t)100 : 0;
    r.flags = (uint8_t)(0x08u | 0x10u | 0x20u | 0x40u);    /* core/gps_nav.h FIX3D | CFG_OK | INIT | VER_OK */
    if (s_nav_aligned) r.flags |= (uint8_t)(0x01u | 0x02u | 0x04u);   /* FIX_OK | GATE_OK | HEAD_OK */
    r.num_sv = 9u;
    r.hacc_dm = 18u;
    r.pc = nl_pc_byte(s_nav_motor ? POS_MODE_POS : POS_MODE_OFF, s_nav_motor, 0u);
    r.pos[0] = (int16_t)(30.0f * pgm_read_float(&k_tr_hr[i]) * (1.0f / 0.035f));
    r.pos[1] = (int16_t)(20.0f * pgm_read_float(&k_tr_hp[i]) * (1.0f / 0.04f));
    r.vel[0] = 12;
    r.vel[1] = -7;
    r.trim[0] = s_nav_motor ? 150 : 0;
    r.trim[1] = s_nav_motor ? -80 : 0;
    gl_pack_result(&r, pl);
    nl_gns_tx_result(&s_gtx, (uint8_t)m, pl, k);
    s_gns_results++;
}
#endif

static void nav_build(uint16_t k)
{
#if FC_VTOL
    nl_nav3_t n;                                           /* v0.8: the nlink v3 payload; v0.8.1 under NL_TYPE_NAV4 */
    float c;
#else
    nl_nav_t n;
#endif
    uint8_t fl = NL_F_IMU_OK | NL_F_BARO_OK;
    if ((uint32_t)k == (uint32_t)SB_NAV_RESET_TICK + SB_NAV_RESET_TICKS) {   /* the first frame after the NAV's reset */
        s_nav_fcu_seen = false;
        s_nav_aligned = false;
        s_nav_pad = true;                                  /* until a valid FCU frame says otherwise */
        s_nav_pad_ticks = 0;
        s_nav_seq0 = k;
        s_nav_reset = 0x08;
#if FC_VTOL
        nl_gns_tx_init(&s_gtx, 5u, GLINK_LATENCY_TICKS);   /* v0.8.1: a rebooted NAV has no RESULT */
        s_gtx_init = true;
#endif
    }
    n.seq = (uint8_t)(k - s_nav_seq0);
    if (s_nav_pad) {
        static const float r = 0.017453293f, p = -0.0087266463f;
        n.q[0] = 1.0f - (r * r + p * p) * 0.125f;
        n.q[1] = 0.5f * r;
        n.q[2] = 0.5f * p;
        n.q[3] = 0.0f;
        n.rate[0] = 0.0005f;
        n.rate[1] = -0.0003f;
        n.rate[2] = 0.0002f;
#if FC_VTOL
        n.accel_x_q = n.accel_y_q = 0;
        n.accel_z = FC_G0;
        n.h_baro_rel_cm = (k & 1u) ? 2 : -2;
#else
        n.accel[0] = n.accel[1] = 0.0f;
        n.accel[2] = FC_G0;
        n.h_baro_rel_m = (k & 1u) ? 0.02f : -0.02f;
#endif
        n.h_m = n.vz_mps = 0.0f;
        n.tilt_deg = 1.118034f;
        s_nav_h = 0.0f;
#if FC_VTOL
        s_pl.h = s_pl.v = s_pl.a = s_pl.u_lag = 0.0f;     /* the NAV holds the pad reference: the plant restarts there */
        s_pl.f_z = FC_G0;
        s_pl.airborne = false;
#endif
        if (s_nav_pad_ticks < 0xFFFFu) s_nav_pad_ticks++;
        if (s_nav_pad_ticks >= SB_NAV_ALIGN_TICKS) s_nav_aligned = true;
    } else {
        const uint8_t ir = (uint8_t)((uint16_t)(k * NAV_STEP(1.3f)) >> 10), ip = (uint8_t)((uint16_t)(k * NAV_STEP(0.9f)) >> 10),
                      iy = (uint8_t)((uint16_t)(k * NAV_STEP(0.5f)) >> 10);
#if FC_VTOL
        const float rr = pgm_read_float(&k_tr_rr[ir]), pp = pgm_read_float(&k_tr_pp[ip]),
                    yy = pgm_read_float(&k_tr_yy[iy]);
        n.q[0] = 1.0f - (rr + pp + yy) * 0.125f;
        n.q[1] = pgm_read_float(&k_tr_hr[ir]);
        n.q[2] = pgm_read_float(&k_tr_hp[ip]);
        n.q[3] = pgm_read_float(&k_tr_hy[iy]);
        n.rate[0] = pgm_read_float(&k_tr_wr[ir]);
        n.rate[1] = pgm_read_float(&k_tr_wp[ip]);
        n.rate[2] = pgm_read_float(&k_tr_wy[iy]);
#else
        const float r = 0.07f * sin64(ir), p = 0.08f * sin64((uint8_t)(ip + 16u)), y = 0.09f * sin64(iy);
        n.q[0] = 1.0f - (r * r + p * p + y * y) * 0.125f;
        n.q[1] = 0.5f * r;
        n.q[2] = 0.5f * p;
        n.q[3] = 0.5f * y;
        n.rate[0] = 0.07f * 2.0f * FC_PI_F * 1.3f * sin64((uint8_t)(ir + 16u));
        n.rate[1] = 0.08f * 2.0f * FC_PI_F * 0.9f * sin64((uint8_t)(ip + 32u));
        n.rate[2] = 0.09f * 2.0f * FC_PI_F * 0.5f * sin64((uint8_t)(iy + 16u));
#endif
#if FC_VTOL
        n.accel_x_q = 40;                                  /* 0.4 / -0.3 m/s^2 in 0.01 m/s^2 (nlink v3) */
        n.accel_y_q = -30;
        plant_step(n.q[0] * n.q[0] - n.q[1] * n.q[1] - n.q[2] * n.q[2] + n.q[3] * n.q[3]);   /* cos(tilt) */
        n.accel_z = s_pl.f_z;                              /* v0.7.1: T / m in the air, g on the ground (support) */
        fl |= NAV_F_EST_GATE;                              /* v0.7.1: the VTOL NAV's estimator correction in a hover */
        n.h_m = s_pl.h;
        n.vz_mps = s_pl.v;
        n.h_baro_rel_cm = (int16_t)(s_pl.h * 100.0f) + ((k & 1u) ? 5 : -5);
#else
        n.accel[0] = 0.4f;
        n.accel[1] = -0.3f;
        n.accel[2] = 15.0f;
        s_nav_h += 3.0f * FC_DT_S;
        n.h_m = s_nav_h;
        n.vz_mps = 3.0f;
        n.h_baro_rel_m = s_nav_h + ((k & 1u) ? 0.05f : -0.05f);
#endif
#if FC_VTOL
        n.tilt_deg = sqrtf(rr + pp) * FC_RAD2DEG_F;
#else
        n.tilt_deg = sqrtf(r * r + p * p) * FC_RAD2DEG_F;
#endif
    }
    if (s_nav_aligned) fl |= NL_F_ALIGNED | NL_F_EST_VALID;   /* frozen off the pad; EST_VALID only since this boot */
    if (k % 5u == 0u) fl |= NL_F_BARO_NEW;
    if (!s_nav_fcu_seen) fl |= NL_F_BOOT;
    n.flags = fl;
    n.reset_cause = s_nav_reset;
    n.imu_fail = 0;
    n.baro_fails = 0;
    n.twi_errors = 0;
    n.loop_us_max = (uint16_t)(1300u + (k & 127u));
#if FC_VTOL
    c = 1.0f - 2.0f * (n.q[1] * n.q[1] + n.q[2] * n.q[2]);   /* nlink v3 cos_tilt of the published q (nl_nav3_tilt) */
    n.cos_tilt = c;
    nav_gns_result(k);
    nl_gns_tx_slot(&s_gtx, k, n.slot);
    (void)nl_encode_nav4(&n, s_nav_buf);
#else
    (void)nl_encode_nav(&n, s_nav_buf);
#endif
    if (k == SB_NAV_BAD_TICK)
        s_nav_buf[NL_XFER_BYTES - 1u] ^= 0x5Au;            /* CRC high byte broken */
    s_nav_built = true;
}

/* the FCU frame of the last exchange -> the NAV's pad / flight switch (sim/nlink_proto.py nav_pad_mode) */
static void nav_take_reply(void)
{
#if FC_VTOL
    nl_fcu4_t f;                                           /* v0.8.1: the nlink v4 FCU frame */
#else
    nl_fcu_t f;
#endif
    if (!s_nav_reply_new)
        return;
    s_nav_reply_new = false;
    nlink_spi_last_reply(s_scratch);
#if FC_VTOL
    if (nl_decode_fcu4(s_scratch, &f)) {
        s_nav_motor = (f.pflags & NL_PF_MOTOR_ON) != 0u;   /* the synthetic GNS engages on it */
#else
    if (nl_decode_fcu(s_scratch, &f)) {
#endif
        s_nav_fcu_seen = true;
        s_nav_pad = (f.state == ONB_SAFE || f.state == ONB_ARMED) && !(f.fflags & NL_FF_BENCH);
#if FC_VTOL                                                /* v0.7.1 thr_q: the ESC's u, 0 unless the motor may run */
        if ((f.state != ONB_POWERED && f.thr_q != 0u) || f.thr_q > NL_FCU_THR_SCALE)
            s_tq_bad++;
        if (f.thr_q > s_v.tq_max)
            s_v.tq_max = f.thr_q;
#else                                                      /* ballistic: the v0.6.3 reserved byte stays 0 */
        if (f.thr_q != 0u)
            s_tq_bad++;
#endif
    }
}

/* The simbench's own time (its hooks, minus the RX ISR time the COM's bytes burn on purpose) is not the FCU's: the
 * tick start's lateness after the frame is split into the FCU's share - from the frame (or the end of the simbench
 * hook that was running when it became due) to the tick start, minus the simbench time in between - and the rest. */
static uint32_t s_sb_time;                                 /* cumulative simbench time, counts */
static uint32_t s_sb_exit;                                 /* when the last simbench hook returned */
static uint32_t s_burned;                                  /* ISR emulation burned in the current hook, counts */
static uint32_t s_inj_from, s_inj_sb;                      /* the injected frame: due for the FCU from; s_sb_time then */
static uint32_t s_nav_due_t;                               /* nav_frame_time(s_nav_k), cached */

static void nav_due(uint32_t now)
{
    while (now >= s_nav_due_t) {
        const uint16_t k = s_nav_k;
        if (nav_silent(k)) {
            if (k == SB_NAV_RESET_TICK)
                s_nav_reset_fcu_tick = s_fcu_tick;
        } else {
            s_nav_f = s_nav_due_t;
            s_inj_from = s_nav_f > s_sb_exit ? s_nav_f : s_sb_exit;
            s_inj_sb = s_sb_time;
            if (!s_nav_built) {
                nav_build(k);
                s_nav_built_late++;
            }
            nlink_spi_inject_frame(s_nav_buf);
            s_nav_fk = k;
            s_nav_reply_new = true;
        }
        s_nav_built = false;
        s_nav_k++;
        s_nav_due_t = nav_frame_time(s_nav_k);
    }
}

/* ---------------------------------------------------------------- the synthetic COM */
#define SB_BYTE_COUNTS ((uint32_t)((10UL * 1000000UL * FC_PWM_COUNTS_PER_US + ILINK_BAUD / 2u) / ILINK_BAUD))  /* 8N1 */
#define SB_POLL_TICKS ((uint32_t)((float)FC_LOOP_HZ / LINK_POLL_HZ + 0.5f))   /* the ground's poll period */
#define SB_STAT_TICKS ((uint32_t)ILINK_STAT_DIV * FC_LOOP_HZ / COM_TICK_HZ)   /* the COM's IL_STAT period */
#define SB_FIRST_POLL 30u
#define SB_BOOT_STAT 25u               /* the COM's boot STAT, >= 100 ms after its reset (R3) */
#define SB_LEAD_TICKS 6u               /* a poll's frames are scheduled to arrive this many ticks later */
#define SB_RSSI (-70)
#define SB_SNR_Q4 28
#define SB_COUNT_CYCLES ((uint16_t)(FC_F_CPU_HZ / 1000000UL / FC_PWM_COUNTS_PER_US))   /* CPU cycles per Timer1 count */
/* The bypass has no RX interrupt; its CPU time is put back where the bytes enter the ring: sb_wire_until takes at least
 * SB_ISR_CYCLES per byte it injects (hal/ilink_uart.c RX ISR, 51 cycles; run_simbench.py checks this against the
 * fcu ELF).  The cost of bytes that arrive during the time-critical part therefore lands at the start of the
 * background (the real ISRs would stretch the time-critical part by as much): the per-tick total is right, the
 * time-critical figure stays ISR-free (run_simbench.py adds the ISR analytically). */
#define SB_ISR_CYCLES 51u

/* where a CMD's last byte arrives: this many us before an expected navigation frame (20 us = the worst case for the
 * time-critical decode) */
static const uint16_t k_cmd_end_us[] PROGMEM = {20, 1700, 60, 3100, 160, 600, 300, 2400, 20, 3600};
#define SB_N_PHASES (sizeof k_cmd_end_us / sizeof k_cmd_end_us[0])

typedef struct {
    uint8_t b[IL_STAT_FRAME_BYTES];   /* CMD (14) or STAT (42) frame */
    uint8_t n, pos;
    uint32_t next;                    /* Timer1-count time byte pos is complete (start + (pos + 1) byte times) */
} sb_wire_t;
#define SB_WQ 2u                     /* v0.8 (RAM): a poll queues a CMD and at most one STAT; 3 in v0.7.1 */
static sb_wire_t s_wq[SB_WQ];
static uint8_t s_wq_head, s_wq_n;
static uint32_t s_wire_free;          /* when the wire is free after the last scheduled frame */

static bool s_have_snap, s_pending_sent, s_out_valid, s_out_stat, s_out_after_babble, s_out_due_set;
static uint8_t s_gseq, s_pending, s_snap_state, s_snap_ack, s_out_seq, s_phase_i;
#if FC_VTOL
static uint8_t s_snap_flags, s_snap_phase, s_hops;                 /* SNAP2: flags, phase; ONs executed */
static bool s_up_done, s_down_done, s_off_done, s_landed_seen;
static uint16_t s_cmd_ok, s_cmd_rej;                               /* commands acknowledged: executed / rejected */
static uint8_t s_cmd_skip;                                         /* fw09: UP / DOWN the window did not allow (not sent) */
/* fw09 (SPEC_firmware_v09 WP-B / WP-F): the operator commands only what the build's altitude window allows - the tethered
 * limits in the power builds (params mission.vtol.power_rules_ASSUMED.tether_limits, params_gen.h ONB_W_*: hover 1.5 m in
 * [1.0, 2.0] m), the profile's otherwise (ONB_V_*).  UP when hover + alt_step stays <= alt_max, then DOWN when that target
 * - alt_step stays >= alt_min (tools/run_simbench.py power_profile_cmds counts the same).  The field profile's 1 m step
 * fits neither in a power build: its operator flies ON, OFF, ON2 - before, it sent UP and DOWN anyway and the supervisor
 * refused both (REJ_ALT_LIMIT, rightly), which failed the profile check.  The battery builds keep UP and DOWN (field
 * 5 m +- 1 m in [2, 10] m, demo 1.5 m + 0.5 m in [1, 3] m). */
#if PM_ANY
#define SB_HOVER_DM ONB_W_HOVER_DM
#define SB_ALT_MIN_DM ONB_W_ALT_MIN_DM
#define SB_ALT_MAX_DM ONB_W_ALT_MAX_DM
#else
#define SB_HOVER_DM ONB_V_HOVER_DM
#define SB_ALT_MIN_DM ONB_V_ALT_MIN_DM
#define SB_ALT_MAX_DM ONB_V_ALT_MAX_DM
#endif
#define SB_OP_UP (SB_HOVER_DM + ONB_V_ALT_STEP_DM <= SB_ALT_MAX_DM)
#define SB_OP_DOWN (SB_HOVER_DM + (SB_OP_UP ? ONB_V_ALT_STEP_DM : 0) - ONB_V_ALT_STEP_DM >= SB_ALT_MIN_DM)
#endif
static uint16_t s_snap_nonce;
static uint32_t s_next_poll = SB_FIRST_POLL, s_polls, s_out_due, s_out_sent, s_out_end, s_last_stat = SB_BOOT_STAT;
static uint16_t s_cmds, s_snaps_rx, s_stats_sent, s_unanswered, s_bad_snaps, s_wire_refused, s_unanswered_babble;
static int16_t s_lat_sup_max = -32767, s_lat_stat_max = -32767, s_lat_sup_min = 32767, s_lat_q_max = -32767;   /* window */

/* babble: the pattern at one byte per s_bab_period; fast (D6) from SB_BABBLE_S, slow (R1) from SB_SLOW_BABBLE_S.  The
 * patterns live in flash (v0.7: v0.6.3 built the fast one in a 64-byte RAM buffer, which the VTOL bench builds need for
 * their stack): fast = C5 5C 31 02 x 8, then noise with a 0xC5 every 5th byte - the v0.6.3 generator's bytes: Galois LFSR
 * from 0xACE1, taps 0xB400, one step per byte, the low byte at bytes 32..63 whose index is not a multiple of 5; slow =
 * C5 5C 08 01 (an IL_CMD header) */
static const uint8_t k_bab_fast[64] PROGMEM = {
    IL_SYNC0, IL_SYNC1, IL_SNAP_LEN, IL_TYPE_SNAP, IL_SYNC0, IL_SYNC1, IL_SNAP_LEN, IL_TYPE_SNAP,
    IL_SYNC0, IL_SYNC1, IL_SNAP_LEN, IL_TYPE_SNAP, IL_SYNC0, IL_SYNC1, IL_SNAP_LEN, IL_TYPE_SNAP,
    IL_SYNC0, IL_SYNC1, IL_SNAP_LEN, IL_TYPE_SNAP, IL_SYNC0, IL_SYNC1, IL_SNAP_LEN, IL_TYPE_SNAP,
    IL_SYNC0, IL_SYNC1, IL_SNAP_LEN, IL_TYPE_SNAP, IL_SYNC0, IL_SYNC1, IL_SNAP_LEN, IL_TYPE_SNAP,
    0x86, 0xC3, 0x61, IL_SYNC0, 0xD8, 0xEC, 0x76, 0xBB, IL_SYNC0, 0xEE, 0xF7, 0x7B, 0xBD, IL_SYNC0, 0xEF, 0x77,
    0xBB, 0xDD, IL_SYNC0, 0xF7, 0xFB, 0x7D, 0x3E, IL_SYNC0, 0x8F, 0xC7, 0xE3, 0xF1, IL_SYNC0, 0xFC, 0x7E, 0xBF};
static const uint8_t k_bab_slow[4] PROGMEM = {IL_SYNC0, IL_SYNC1, IL_CMD_LEN, IL_TYPE_CMD};
static bool s_bab_on, s_bab_fast, s_fast_done, s_slow_done, s_after_babble;
static uint32_t s_bab_t, s_bab_period, s_bab_end_tick, s_bab_bytes, s_bab_bytes_slow;
static uint8_t s_bab_k, s_bab_n;

static void sb_babble_start(bool fast, uint32_t tick)
{
    const uint32_t t = sb_now();
    if (fast) {
        s_bab_n = 64u;
        s_bab_period = SB_BYTE_COUNTS;
        s_bab_end_tick = tick + SB_BABBLE_TICKS;
    } else {
        s_bab_n = 4u;
        s_bab_period = SB_US(SB_SLOW_BABBLE_US);
        s_bab_end_tick = tick + SB_SLOW_BABBLE_TICKS;
    }
    s_bab_fast = fast;
    s_bab_k = 0;
    s_bab_on = true;
    s_bab_t = (s_wire_free > t ? s_wire_free : t) + SB_BYTE_COUNTS;
}

static uint16_t s_isr_owed;                              /* bytes injected at a tick, their ISR time not yet spent */

static void sb_isr_burn(uint16_t a, uint16_t bytes)
{
    const uint16_t counts = (uint16_t)(((uint32_t)bytes * SB_ISR_CYCLES + SB_COUNT_CYCLES - 1u) / SB_COUNT_CYCLES);
    while (timer_elapsed_counts(a, timer_tcnt1()) < counts) {
    }
    s_burned += counts;                                    /* emulated FCU interrupt time, not the simbench's */
    s_tick_burned += counts;
}

/* the bytes complete by time t -> RX ring; burn: spend their ISR time now (else at the next call: the tick start's own
 * injection must not lengthen the time-critical part) */
static void sb_wire_until(uint32_t t, bool burn)
{
    const uint16_t a = timer_tcnt1();
    uint16_t bytes = 0;
    while (s_wq_n) {
        sb_wire_t *w = &s_wq[s_wq_head];
        const uint8_t k0 = w->pos;
        while (w->pos < w->n && w->next <= t) {
            w->pos++;
            w->next += SB_BYTE_COUNTS;
        }
        if (w->pos > k0) {
            ilink_uart_inject_rx(w->b + k0, (uint8_t)(w->pos - k0));
            bytes = (uint16_t)(bytes + (uint8_t)(w->pos - k0));
        }
        if (w->pos < w->n)
            break;
        s_wq_head = (uint8_t)((s_wq_head + 1u) % SB_WQ);
        s_wq_n--;
    }
    while (s_bab_on && s_bab_t <= t) {                   /* babble, in runs of the pattern (<= 16 bytes a run) */
        uint8_t run[16], k = 0;
        const uint8_t k0 = s_bab_k;
        while (s_bab_t <= t && (uint8_t)(k0 + k) < s_bab_n && k < (uint8_t)sizeof run) {
            k++;
            s_bab_t += s_bab_period;
        }
        memcpy_P(run, (s_bab_fast ? k_bab_fast : k_bab_slow) + k0, k);
        ilink_uart_inject_rx(run, k);
        s_bab_k = (uint8_t)(k0 + k) >= s_bab_n ? 0u : (uint8_t)(k0 + k);
        if (s_bab_fast)
            s_bab_bytes += k;
        else
            s_bab_bytes_slow += k;
        bytes = (uint16_t)(bytes + k);
    }
    if (!burn) {
        s_isr_owed = (uint16_t)(s_isr_owed + bytes);
    } else if (bytes || s_isr_owed) {
        sb_isr_burn(a, (uint16_t)(bytes + s_isr_owed));
        s_isr_owed = 0;
    }
}

/* one frame on the wire, its last byte complete at end (or as soon after as the wire allows); false if the queue is
 * full; s_wire_free = the time its last byte is complete */
static bool sb_wire_put(uint8_t type, const uint8_t *payload, uint8_t len, uint32_t end, uint32_t now)
{
    sb_wire_t *w;
    uint32_t dur, start;
    if (s_wq_n >= SB_WQ) {
        s_wire_refused++;
        return false;
    }
    w = &s_wq[(uint8_t)((s_wq_head + s_wq_n) % SB_WQ)];
    w->n = il_encode(type, payload, len, w->b);
    w->pos = 0;
    dur = (uint32_t)w->n * SB_BYTE_COUNTS;
    start = end > dur ? end - dur : 0;
    if (start < now) start = now;
    if (start < s_wire_free) start = s_wire_free;
    w->next = start + SB_BYTE_COUNTS;
    s_wire_free = start + dur;
    s_wq_n++;
    return true;
}

static void sb_stat_put(uint32_t tick, bool boot, uint32_t end, uint32_t now)
{
    il_stat_t s;
    uint8_t p[IL_STAT_LEN];
    s.com_ticks = tick;                                   /* COM_TICK_HZ == FC_LOOP_HZ (generator check) */
    s.reset_cause = 0x01;
    s.flags = (uint8_t)(IL_STAT_RADIO_OK | (boot ? IL_STAT_BOOT : 0u));
    s.radio_version = 0;
    s.rssi_dbm = SB_RSSI;
    s.snr_q4 = SB_SNR_Q4;
    s.params_tag = FC_PARAMS_TAG;
    s.dec_frames = s_cmds;
    s.dec_bad_crc = s.dec_bad_len = s.dec_other = 0;
    s.cmd_forwarded = s_cmds;
    s.radio_rx_errors = s.radio_rx_overflow = s.radio_tx_timeouts = 0;
    s.replies = s_snaps_rx;
    s.replies_dropped = s.il_bad = s.il_tx_dropped = 0;
    s.loop_us_max = 1234;
    il_pack_stat(&s, p);
    if (sb_wire_put(IL_TYPE_STAT, p, IL_STAT_LEN, end, now)) {
        s_stats_sent++;
        s_last_stat = tick;
    }
}

/* the FCU's SNAPs (the TX ring holds whole frames: every ilink_uart_write is one frame), one frame per take */
static void sb_take_snaps(uint32_t tick)
{
    uint8_t n;
    while ((n = ilink_uart_take_tx(s_scratch, SB_SNAP_FRAME_BYTES)) != 0u) {
        const uint8_t *f = s_scratch, *p = f + 4;
        uint32_t snap_tick;
        if (n != SB_SNAP_FRAME_BYTES || f[0] != IL_SYNC0 || f[1] != IL_SYNC1 || f[2] != SB_SNAP_LEN ||
            f[3] != SB_SNAP_TYPE ||
            il_crc16(f + 2, SB_SNAP_LEN + 2u, LP_CRC_INIT) != (uint16_t)(f[4 + SB_SNAP_LEN] | (f[5 + SB_SNAP_LEN] << 8))) {
            s_bad_snaps++;
            continue;
        }
        s_snaps_rx++;
        s_have_snap = true;
        s_snap_state = p[1];
        s_snap_ack = p[4];
        s_snap_nonce = (uint16_t)(p[5] | ((uint16_t)p[6] << 8));
#if FC_VTOL
        s_snap_flags = p[2];
        s_snap_phase = p[IL_SNAP_LEN];                    /* SNAP2: phase right after the 49 SNAP bytes */
        if (s_snap_phase == LP_PHASE_LANDED)
            s_landed_seen = true;
#endif
        snap_tick = (uint32_t)p[9] | ((uint32_t)p[10] << 8) | ((uint32_t)p[11] << 16) | ((uint32_t)p[12] << 24);
        if (s_out_valid && s_out_due_set && p[0] == s_out_seq && snap_tick >= s_out_sent) {
            const int16_t ls = (int16_t)(int32_t)(snap_tick - s_out_due);   /* due -> supervisor answered */
            const int16_t lq = (int16_t)(int32_t)(tick - s_out_due);        /* due -> SNAP in the TX ring (taken now) */
            if (s_out_stat) {
                if (ls > s_lat_stat_max) s_lat_stat_max = ls;
            } else if (ls > s_lat_sup_max) {
                s_lat_sup_max = ls;
            }
            if (ls < s_lat_sup_min) s_lat_sup_min = ls;
            if (lq > s_lat_q_max) s_lat_q_max = lq;
            s_out_valid = false;
        }
    }
}

/* the ground's poll through the COM: one CMD; an IL_STAT right behind it once per 1 / stat_hz (D4), every
 * SB_STAT_FRONT_EVERY-th poll one right in front of it instead (stress); the babble windows */
static void sb_com_after_loop(uint32_t tick)
{
    uint32_t now, end;
    il_cmd_t c;
    uint8_t p[IL_CMD_LEN];
    bool front;
    sb_take_snaps(tick);
    if (!s_bab_on && !s_slow_done && tick >= (uint32_t)SB_SLOW_BABBLE_S * FC_LOOP_HZ) {
        sb_babble_start(false, tick);
        s_slow_done = true;
    } else if (!s_bab_on && !s_fast_done && tick >= (uint32_t)SB_BABBLE_S * FC_LOOP_HZ) {
        sb_babble_start(true, tick);
        s_fast_done = true;
    } else if (s_bab_on && tick >= s_bab_end_tick) {
        s_bab_on = false;
        s_after_babble = true;
        if (s_wire_free < s_bab_t) s_wire_free = s_bab_t;
    }
    if (tick < SB_BOOT_STAT || (tick > SB_BOOT_STAT && tick < s_next_poll))
        return;
    now = sb_now();
    if (tick == SB_BOOT_STAT) {                            /* the COM's first STAT after its reset */
        sb_stat_put(tick, true, now, now);
        return;
    }
    s_next_poll = tick + SB_POLL_TICKS;
    if (s_out_valid) {                                     /* the previous CMD got no SNAP within a poll period */
        if (s_out_after_babble)
            s_unanswered_babble++;
        else
            s_unanswered++;
        s_out_valid = false;
    }
    if (s_bab_on)
        return;                                            /* babbling: no frame */
    if (s_pending != LP_CMD_NONE && s_pending_sent && s_have_snap && s_snap_ack == s_gseq) {
#if FC_VTOL                                                /* the SNAP that acknowledged it: executed or rejected */
        if (s_snap_flags & LP_FLAG_REJECTED) {
            s_cmd_rej++;
        } else {
            s_cmd_ok++;
            if (s_pending == LP_CMD_ON)
                s_hops++;
        }
#endif
        s_pending = LP_CMD_NONE;                           /* acknowledged */
    }
    if (s_pending == LP_CMD_NONE && s_have_snap) {
        uint8_t want = LP_CMD_NONE;
#if FC_VTOL                                                /* the operator of the VTOL profile (simbench.h) */
        const bool flying = s_snap_phase == LP_PHASE_CLIMB || s_snap_phase == LP_PHASE_HOLD;
        if (s_snap_state == ONB_SAFE && ((s_hops == 0u && tick >= (uint32_t)SB_ON_S * FC_LOOP_HZ) ||
                                         (s_hops == 1u && s_landed_seen && tick >= (uint32_t)SB_ON2_S * FC_LOOP_HZ))) {
            want = LP_CMD_ON;
        } else if (s_hops == 1u && s_snap_phase == LP_PHASE_HOLD && !s_up_done && tick >= (uint32_t)SB_UP_S * FC_LOOP_HZ) {
            want = SB_OP_UP ? LP_CMD_UP : LP_CMD_NONE;     /* fw09: outside the window: not sent, counted (SV "sk") */
            if (!SB_OP_UP) s_cmd_skip++;
            s_up_done = true;
        } else if (s_hops == 1u && s_snap_phase == LP_PHASE_HOLD && !s_down_done &&
                   tick >= (uint32_t)SB_DOWN_S * FC_LOOP_HZ) {
            want = SB_OP_DOWN ? LP_CMD_DOWN : LP_CMD_NONE;
            if (!SB_OP_DOWN) s_cmd_skip++;
            s_down_done = true;
        } else if (s_hops == 1u && flying && !s_off_done && tick >= (uint32_t)SB_OFF_S * FC_LOOP_HZ) {
            want = LP_CMD_OFF;
            s_off_done = true;
        }
#else
        if (s_snap_state == ONB_SAFE && tick >= (uint32_t)SB_ARM_S * FC_LOOP_HZ)
            want = LP_CMD_ARM;
        else if (s_snap_state == ONB_ARMED && tick >= (uint32_t)SB_LAUNCH_S * FC_LOOP_HZ)
            want = LP_CMD_LAUNCH;
#endif
        if (want != LP_CMD_NONE) {
            s_gseq++;
            s_pending = want;
            s_pending_sent = false;
        }
    }
    c.seq = s_gseq;
    c.cmd.cmd = s_pending;
    c.cmd.arg = 0;
    c.cmd.nonce = s_snap_nonce;                            /* the nonce of the latest SNAP */
    c.rssi_dbm = SB_RSSI;
    c.snr_q4 = SB_SNR_Q4;
    il_pack_cmd(&c, p);
    /* the CMD's last byte k_cmd_end_us before the expected frame SB_LEAD_TICKS NAV ticks on */
    end = nav_frame_time((uint16_t)(s_nav_k + SB_LEAD_TICKS)) - SB_US(pgm_read_word(&k_cmd_end_us[s_phase_i]));
    s_phase_i = (uint8_t)((s_phase_i + 1u) % SB_N_PHASES);
    front = (s_polls % SB_STAT_FRONT_EVERY) == SB_STAT_FRONT_EVERY - 1u;
    if (front)                                             /* stress: a STAT right in front of the CMD */
        sb_stat_put(tick, false, end - (uint32_t)IL_CMD_FRAME_BYTES * SB_BYTE_COUNTS, now);
    if (sb_wire_put(IL_TYPE_CMD, p, IL_CMD_LEN, end, now)) {
        s_cmds++;
        s_out_valid = true;
        s_out_due_set = false;
        s_out_end = s_wire_free;
        s_out_stat = front;
        s_out_after_babble = s_after_babble;
        s_after_babble = false;
        s_out_seq = c.seq;
        s_out_sent = tick;
        s_pending_sent = true;
    }
    if (!front && tick - s_last_stat >= SB_STAT_TICKS)      /* D4: the STAT right behind a forwarded CMD */
        sb_stat_put(tick, false, end + (uint32_t)IL_STAT_FRAME_BYTES * SB_BYTE_COUNTS, now);
    s_polls++;
}

/* ---------------------------------------------------------------- the hooks main.c calls */
static uint32_t s_t0;                                      /* sim time of the current tick's start */
static uint32_t s_prev_t0;
static bool s_prev_nav = true, s_tick_nav;
static uint16_t s_tick_fk;                                 /* NAV tick of the frame the current FCU tick runs on */
static uint16_t s_f1_max, s_fp_max, s_fn_min = 0xFFFFu;    /* fallback ticks: first after a frame / period, us */

void simbench_start(void)
{
    s_clk_t1 = timer_tcnt1();
    s_clk = 0;
    s_run = true;
    s_prev_t0 = 0;
    s_nav_due_t = nav_frame_time(s_nav_k);
}

static void sb_hook_end(uint32_t entry)                    /* every hook's last statement */
{
    const uint32_t x = sb_now();
    s_sb_time += (x - entry) - s_burned;
    s_burned = 0;
    s_sb_exit = x;
}

void simbench_peers(void)
{
    uint32_t now;
    if (!s_run)
        return;
    now = sb_now();
    nav_due(now);
    if (s_wq_n || s_bab_on || s_isr_owed)
        sb_wire_until(now, true);
    sb_hook_end(now);
}

static uint16_t s_lm_tick;                                 /* this tick's FCU share of the lateness, us */
static uint32_t s_tick_sb;                                 /* s_sb_time when the tick-start hook returned */

void simbench_tick_start(uint32_t tick, bool nav)
{
    const uint32_t t = sb_now();
    s_fcu_tick = tick;
    s_t0 = t;
    s_tick_nav = nav;
    if (nav) {
        const uint32_t sb = s_sb_time - s_inj_sb;          /* simbench time since the frame became due */
        const uint32_t all = t - s_inj_from;
        const uint32_t own = all > sb ? all - sb : 0u;
        s_lm_tick = counts_to_us16(own);
        if (s_lm_tick > s_lm_max) s_lm_max = s_lm_tick;
        {
            const uint16_t ls = counts_to_us16(t - s_nav_f - own);
            if (ls > s_ls_max) s_ls_max = ls;
        }
        s_tick_fk = s_nav_fk;
    } else if (s_prev_t0 != 0u) {
        const uint16_t dt = counts_to_us16(t - s_prev_t0);
        if (s_prev_nav) {
            if (dt > s_f1_max) s_f1_max = dt;
        } else {
            if (dt > s_fp_max) s_fp_max = dt;
            if (dt < s_fn_min) s_fn_min = dt;
        }
    }
    s_prev_nav = nav;
    s_prev_t0 = t;
    if (s_out_valid && !s_out_due_set && t >= s_out_end) {   /* the first tick that starts after the CMD's last byte */
        s_out_due = tick;
        s_out_due_set = true;
    }
    sb_wire_until(t, false);                               /* the bytes that arrived before the tick (step 2's) */
    sb_hook_end(t);
    s_tick_sb = s_sb_time;
    s_tick_burned = 0;
}
#else  /* FC_COSIM */
/* ======================================================================================== */
/* co-simulation: the tick marker port and the SC lines                                     */
/* ======================================================================================== */
void simbench_cosim_init(void)
{
    PORTA = 0;
    DDRA = 0xFF;
}

#define SC_Q 4u
static struct {
    uint32_t tick;
    uint8_t n, q, r, w, b;
    uint16_t c;
} s_scq[SC_Q];
static uint8_t s_scq_head, s_scq_n;
static uint16_t s_sc_lost;

static void sc_flush(void)
{
    static char buf[80];
    while (s_scq_n) {
        const uint8_t i = s_scq_head;
        uint8_t p = line_start(buf, 'S', 'C');
        p = put_u(buf, p, "t=", s_scq[i].tick);
        p = put_u(buf, p, "n=", s_scq[i].n);
        p = put_u(buf, p, "q=", s_scq[i].q);
        p = put_u(buf, p, "r=", s_scq[i].r);
        p = put_u(buf, p, "w=", s_scq[i].w);
        p = put_u(buf, p, "b=", s_scq[i].b);
        p = put_u(buf, p, "c=", s_scq[i].c);
        buf[p++] = '\n';
        if (uart_log_free() < p || !uart_log_write((const uint8_t *)buf, p))
            return;                                        /* the log ring is busy: next tick */
        s_scq_head = (uint8_t)((s_scq_head + 1u) % SC_Q);
        s_scq_n--;
    }
}
#endif /* FC_COSIM */

/* ---------------------------------------------------------------- report lines */
/* v0.8: every report line is streamed straight into the log ring (hal/uart_log.h uart_log_begin / _put / _commit: no line
 * buffer - the v0.8 FCU and the synthetic peers must share the RAM) and is < SB_LINE_MAX bytes, so a line fits the bench's
 * 128-byte console ring; a kind's fields are spread over several lines with the same prefix and the same "t=" (the first
 * part's tick) - tools/run_simbench.py merges the rows of one window */
#define SB_LINE_MAX 124u
static uint8_t rput(uint8_t pos, const char *key_P, uint32_t v)
{
    char tmp[24];
    const uint8_t n = put_u_P(tmp, 0, key_P, v);
    if ((uint16_t)pos + n + 1u > SB_LINE_MAX)
        return pos;                                          /* never past the reserved length */
    uart_log_put((const uint8_t *)tmp, n);
    return (uint8_t)(pos + n);
}
#define rp(key, v) (p = rput(p, PSTR(key), (uint32_t)(v)))

static uint8_t rstart(char c2, uint32_t tick)
{
    const uint8_t h[4] = {'\n', 'S', (uint8_t)c2, ' '};
    uart_log_put(h, 4u);
    return rput(4u, PSTR("t="), tick);
}

/* kinds: B timing (3 lines), N nlink (2), L ilink (2), I ilink counters (2), V VTOL (2; v0.8.1 3), P v0.8 position hold
 * (v0.8.1: 2 - one line dropped the fields past SB_LINE_MAX: the gate tick, the dropout, the largest tilt) */
static uint32_t s_rep_t;                                     /* the tick in every line of the kind being written */
static bool report_line(char kind, uint8_t part, const sb_tick_t *t)
{
    uint8_t p, i;
    if (!uart_log_begin(SB_LINE_MAX))
        return false;                                        /* the log ring is busy: try again next tick */
    if (part == 0u)
        s_rep_t = t->tick;
    p = rstart(kind == 'P' ? 'P' : kind, s_rep_t);
    if (kind == 'B' && part == 0u) {                         /* once per second: the timing line */
        rp("st=", t->state);
        rp("cm=", s_win_max);                                /* time-critical part, max us */
        rp("cc=", s_win_ctrl_max);                           /* same, controller active */
        rp("tm=", s_win_total_max);                          /* incl. background */
        rp("ov=", t->overruns);                              /* ticks without a valid navigation frame */
        rp("sf=", stack_free_min_step());                    /* free stack, last complete background scan */
        rp("ct=", s_ctrl_ticks);
    } else if (kind == 'B' && part == 1u) {
        rp("bg=", s_bg_step_max);                            /* longest background job other than the ilink drain, us */
        rp("bk=", s_bg_step_kind);                           /* ... its kind (main.c BG_*) */
        rp("sn=", s_snap_step_max);                          /* longest SNAP job (pack + CRC + queue), us */
        rp("il=", s_il_step_max);                            /* longest ilink drain (byte-wise, time-checked), us */
        rp("ox=", s_ovh_max);                                /* the simbench's own work after the background, us */
        {   /* the compare registers actually driving the pins (timer counts, FC_PWM_COUNTS_PER_US per us) */
            uint16_t o1a, o3a, o3b, o3c;
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                o1a = OCR1A;
                o3a = OCR3A;
                o3b = OCR3B;
                o3c = OCR3C;
            }
            rp("o1a=", o1a);
            rp("o3a=", o3a);
            rp("o3b=", o3b);
            rp("o3c=", o3c);
        }
    } else if (kind == 'B') {
        rp("s=", s_sec_max[1] / FC_PWM_COUNTS_PER_US);
        for (i = 2; i < SB_SECTIONS; i++) rp("", s_sec_max[i] / FC_PWM_COUNTS_PER_US);
    } else if (kind == 'N' && part == 0u) {                  /* the nlink line */
        rp("st=", t->state);
        rp("fv=", t->nl_frames);                             /* valid navigation frames */
        rp("fb=", t->nl_bad);                                /* exchanges that failed nl_decode_nav */
        rp("ft=", t->nl_timeouts);                           /* ticks on the frame timeout / Timer2 */
        rp("fq=", t->nl_seq_lost);                           /* NAV ticks lost (SEQ gaps) */
        rp("rs=", t->nl_resyncs);                            /* SPI slave resyncs */
        rp("fo=", t->nl_overruns);                           /* exchanges replaced before they were taken */
        rp("pw=", s_pw_max);                                 /* tick start -> vanes written, us */
        rp("pc=", s_pc_max);                                 /* ... controller active */
        rp("la=", s_la_max);                                 /* NAV tick -> vanes written, us */
        rp("lc=", s_lc_max);                                 /* ... controller active */
    } else if (kind == 'N') {
#if !defined(FC_COSIM)
        rp("lm=", s_lm_max);                                 /* exchange complete -> tick start, the FCU's share, us */
        rp("ls=", s_ls_max);                                 /* ... the simbench's share (bypass injection), us */
        rp("bm=", (uint16_t)s_bm_min);                       /* background end -> next exchange start, us (signed) */
        rp("be=", s_be_max);                                 /* anchor -> background end, us */
        rp("f1=", s_f1_max);                                 /* first fallback tick after a frame, interval max, us */
        rp("fp=", s_fp_max);                                 /* fallback ticks in a row: period max ... */
        rp("fn=", s_fn_min);                                 /* ... and min, us */
        rp("nr=", s_nav_reset_fcu_tick);                     /* FCU tick the NAV went silent to reset */
        rp("nl=", s_nav_built_late);                         /* frames the simbench built at their due time */
#endif
        rp("at=", s_abort_tick);                             /* the first ABORT: FCU tick ... */
        rp("ar=", s_abort_reason);                           /* ... and the reason code */
        s_pw_max = s_pc_max = s_la_max = s_lc_max = s_lm_max = s_ls_max = s_be_max = 0;
        s_bm_min = 32767;
    } else if (kind == 'L' && part == 0u) {                  /* the ilink line */
        rp("st=", t->state);
        rp("nw=", s_nw_max);                                 /* ilink bytes waiting at a tick */
        rp("nb=", s_nb_max);                                 /* ... of them decoded in the time-critical part */
        rp("cx=", t->crit_over);                             /* ticks an IL_CMD could not complete within ILINK_CRIT_BYTES */
        rp("sd=", t->snap_dropped);                          /* SNAPs replaced before they left */
        rp("fd=", t->flog_dropped);                          /* flight records replaced before they were packed */
        rp("bl=", t->baro_late);                             /* valid frames without NL_F_BARO_OK */
        rp("bn=", s_bn_max);                                 /* anchor -> background end without the lumped RX-ISR emulation */
        rp("je=", s_je_max);                                 /* anchor -> the latest background job's end (FCU work only) */
#if FC_VTOL
        rp("le=", s_le_max);                                 /* v0.8.1: anchor -> the latest late-pass job's end (window max) */
        s_le_max = 0;
#endif
        s_bn_max = s_je_max = 0;
        s_nw_max = s_nb_max = 0;
    } else if (kind == 'L') {
#if !defined(FC_COSIM)
        rp("up=", s_cmds);                                   /* IL_CMD frames the synthetic COM sent */
        rp("rp=", s_snaps_rx);                               /* SNAP frames it received (header + CRC ok) */
        rp("sa=", s_stats_sent);                             /* IL_STAT frames it sent */
        rp("ua=", s_unanswered);                             /* CMDs without a SNAP within a poll period */
        rp("bs=", (uint32_t)s_bad_snaps + s_wire_refused);   /* bad SNAP frames + wire queue full */
        if (s_lat_q_max > -32767) {                          /* latencies in ticks, window extremes (two's complement) */
            rp("rn=", (uint16_t)s_lat_sup_min);              /* first tick after a CMD's last byte -> its supervisor, min */
            rp("rl=", (uint16_t)s_lat_q_max);                /* ... -> SNAP queued (taken from the TX ring), max */
        }
        if (s_lat_sup_max > -32767)
            rp("rs=", (uint16_t)s_lat_sup_max);              /* ... -> its supervisor, max, no STAT in front */
        if (s_lat_stat_max > -32767)
            rp("rt=", (uint16_t)s_lat_stat_max);             /* ... the same with a STAT right in front of the CMD */
        s_lat_sup_max = s_lat_stat_max = s_lat_q_max = -32767;
        s_lat_sup_min = 32767;
#endif
#if FC_VTOL
    } else if (kind == 'V' && part == 0u) {                  /* the VTOL line */
        rp("st=", t->state);
        rp("ph=", t->phase);                                 /* LP_PHASE_* now */
        rp("pv=", s_v.phases);                               /* phases seen, bit (1 << LP_PHASE_*) */
        rp("tg=", (uint16_t)t->h_tgt_dm);                    /* target, dm */
        rp("ps=", s_v.powered_starts);                       /* motor starts */
        rp("ho=", s_v.hold_ticks);                           /* ticks in HOLD */
        rp("sa=", s_v.sat_ticks);                            /* POWERED ticks with the controller at its ceiling */
        rp("um=", s_v.u_max_milli);                          /* ESC command max (x1000) */
        rp("uh=", s_v.u_hold_min_milli);                     /* ESC command min in HOLD (x1000) */
        rp("lt=", s_v.landed_tick);                          /* first LANDED tick */
    } else if (kind == 'V' && part == 1u) {
        rp("sf=", s_v.safe_tick);                            /* the SAFE tick after it */
        rp("vd=", t->vrec_dropped);                          /* 'V' records replaced before they were written */
#if !defined(FC_COSIM)
        rp("hm=", (uint32_t)(s_pl.h_max * 1000.0f));         /* the synthetic plant: max height, mm */
        rp("hc=", (uint32_t)(s_pl.h * 1000.0f));             /* ... height now, mm */
        rp("tv=", (uint32_t)(s_pl.td_v * 1000.0f));          /* ... hardest ground contact, mm/s */
        rp("ok=", s_cmd_ok);                                 /* commands the synthetic ground got executed ... */
        rp("rj=", s_cmd_rej);                                /* ... and rejected */
        rp("hp=", s_hops);                                   /* ONs executed */
        rp("tx=", s_v.tq_max);                               /* v0.7.1: the largest thr_q the FCU sent the NAV */
#endif
        rp("tb=", s_v.td_by);                                /* v0.7.1: the signature that confirmed the first touchdown */
    } else if (kind == 'V') {                                /* v0.8.1: the third SV line */
        rp("lr=", s_v.land_reason);                          /* v0.8.1: the supervisor's reason at the first LAND ... */
        rp("lk=", s_v.land_tick);                            /* ... and its tick */
        rp("wo=", s_v.own_max);                              /* v0.8.1: POWERED: the FCU's own work per tick, max us ... */
        rp("ws=", s_v.own_sum);                              /* ... sum and count (mean) */
        rp("wn=", s_v.own_n);
#if !defined(FC_COSIM)
        rp("sk=", s_cmd_skip);                               /* fw09: UP / DOWN the build's altitude window did not allow */
#endif
#if ONB_HAS_GPS
    } else if (kind == 'P' && part == 0u) {                  /* v0.8: the position-hold lines (v0.8.1: two, < 124 B) */
        rp("pm=", t->pos_mode);                              /* POS_MODE_* published now */
        rp("pq=", s_v.pos_requests);                         /* POS_REQ_* seen (OR) */
        rp("pe=", s_v.pos_engaged_ticks);                    /* ticks engaged */
        rp("pp=", s_v.pos_modes);                            /* position modes seen, bit (1 << POS_MODE_*) */
        rp("pl=", t->pos_late);                              /* ticks whose position work was finished at the next start */
        rp("pb=", s_pb_max);                                 /* longest position job (piece / builder / set_tilt), us */
        rp("pf=", s_pf_max);                                 /* longest late finish at a tick start, us */
        rp("pz=", s_pz_max);                                 /* anchor -> a position job's end in the exchange window, us */
        rp("gd=", t->grec_dropped);                          /* 'G' records replaced before they were written */
        rp("pj=", s_pj_max);                                 /* fw09: longest position piece (not pos_work_now), us */
        s_pb_max = s_pf_max = s_pz_max = s_pj_max = 0;
    } else if (kind == 'P') {                                /* v0.8.1: the second position-hold line (the GNS slot) */
        rp("xs=", t->slot_sets);                             /* complete GNS slot sets received (v0.8.1) */
        rp("xb=", t->slot_broken);                           /* broken slot cycles */
        rp("xt=", t->slot_tilts);                            /* v0.8.1: TILT parts received */
        rp("xf=", s_v.fallback_ticks);                       /* v0.8.1: ticks in the TRIM fallback */
#if !defined(FC_COSIM)
        rp("xr=", s_gns_results);                            /* v0.8.1: RESULTs the synthetic GNS handed the NAV */
        rp("xd=", s_gns_dropped);                            /* ... and the cycles of its dropout */
#endif
        rp("gg=", s_v.gate_ok_tick);                         /* first FCU tick with the GPS ON gate ok */
        rp("tz=", s_v.tilt_max_mrad);                        /* the largest tilt setpoint, mrad */
#endif
    } else if (kind == 'I' && part == 0u) {                  /* 'I': the FCU's ilink counters */
#else
    } else if (kind == 'I' && part == 0u) {                  /* 'I': the FCU's ilink counters */
#endif
        rp("st=", t->state);
        rp("ic=", t->il_cmds);                               /* IL_CMD frames handed to the supervisor */
        rp("is=", t->il_snaps);                              /* SNAP frames queued */
        rp("ib=", t->il_bad);                                /* decoder rejects (CRC, LEN, TYPE) */
        rp("ro=", t->rx_overflow);                           /* RX ring overflow */
        rp("re=", t->rx_errors);                             /* RX framing / overrun / parity errors */
        rp("bx=", t->bg_capped);                             /* ticks the background used up ILINK_BG_BYTES */
        rp("fl=", t->flood_ticks);                           /* ticks after an RX ring overflow (D6 flood: no ilink share) */
        rp("bb=", s_bb_max);                                 /* most bytes the background decoded ahead of the jobs */
        s_bb_max = 0;
    } else {
        rp("bj=", s_bj_max);                                 /* longest job started on BG_RESERVE_US, us */
        rp("bt=", s_bt_max);                                 /* longest text job (COM_TEXT_RESERVE_US), us */
        rp("ed=", t->evq_dropped);                           /* event / link records dropped (queue full) */
        rp("br=", s_br_max);                                 /* longest record job (flight part, E/L, V, C, N), us */
        rp("sr=", t->status_written);                        /* 1 Hz status records written (cumulative) */
#if !defined(FC_COSIM)
        rp("tq=", s_tq_bad);                                 /* FCU frames with a thr_q byte out of its rule (v0.7.1) */
        rp("ub=", s_unanswered_babble);                      /* CMDs lost right after a babble */
        rp("bz=", s_bab_bytes);                              /* fast babble bytes put on the wire */
        rp("sz=", s_bab_bytes_slow);                         /* slow babble bytes put on the wire */
#else
        rp("sl=", s_sc_lost);                                /* SC lines lost (queue full) */
#endif
#if FC_VTOL
        rp("nj=", s_nj_max);                                 /* fw09: bj / bt / br of the normal pass (before BG_END_US) */
        rp("nt=", s_nt_max);
        rp("nr=", s_nr_max);
        s_nj_max = s_nt_max = s_nr_max = 0;
#endif
        s_bj_max = s_bt_max = s_br_max = 0;
    }
    {
        const uint8_t nl = '\n';
        uart_log_put(&nl, 1u);
    }
    uart_log_commit();
    (void)p;
    return true;
}

#if ONB_HAS_GPS
void simbench_pos_end(uint16_t anchor_us)
{
#if !defined(FC_COSIM)
    const uint16_t sb = counts_to_us16(s_sb_time - s_tick_sb);    /* without the bench's own time since the tick start */
    const uint16_t e = anchor_us > sb ? (uint16_t)(anchor_us - sb) : 0u;
#else
    const uint16_t e = anchor_us;
#endif
    if (e > s_pz_max) s_pz_max = e;
}
#endif

#if FC_VTOL
/* v0.8.1: a job of the late pass (main.c g_late) ended: anchor-relative, without the bench's own time (as pos_end) */
void simbench_late_end(uint16_t anchor_us)
{
#if !defined(FC_COSIM)
    const uint16_t sb = counts_to_us16(s_sb_time - s_tick_sb);
    const uint16_t e = anchor_us > sb ? (uint16_t)(anchor_us - sb) : 0u;
#else
    const uint16_t e = anchor_us;
#endif
    if (e > s_le_max) s_le_max = e;
}
#endif

void simbench_bg_work_end(uint16_t anchor_us)
{
#if !defined(FC_COSIM)
    const uint16_t sb = counts_to_us16(s_sb_time - s_tick_sb);    /* the bench's own time since the tick start */
    const uint16_t je = anchor_us > sb ? (uint16_t)(anchor_us - sb) : 0u;
#else
    const uint16_t je = anchor_us;
#endif
    if (je > s_je_max) s_je_max = je;
}

void simbench_end_of_loop(const sb_tick_t *t)
{
    const uint16_t ovh0 = timer_tcnt1();
#if !defined(FC_COSIM)
    const uint32_t entry = sb_now();
#endif
    uint8_t i;
    for (i = 1; i < SB_SECTIONS; i++) {
        uint16_t d = timer_elapsed_counts(s_mark[i - 1], s_mark[i]);
        if (d > s_sec_max[i]) s_sec_max[i] = d;
    }
    if (t->loop_us > s_win_max) s_win_max = t->loop_us;
    if (t->total_us > s_win_total_max) s_win_total_max = t->total_us;
    if (t->n_wait > s_nw_max) s_nw_max = t->n_wait;
    if (t->n_crit > s_nb_max) s_nb_max = t->n_crit;
    if (t->bg_bytes > s_bb_max) s_bb_max = t->bg_bytes;
    if (t->pwm_us > s_pw_max) s_pw_max = t->pwm_us;
    if (t->state == ONB_POWERED || t->state == ONB_UNPOWERED) {
        s_ctrl_ticks++;
        if (t->loop_us > s_win_ctrl_max) s_win_ctrl_max = t->loop_us;
        if (t->pwm_us > s_pc_max) s_pc_max = t->pwm_us;
    }
    if (t->state == ONB_ABORT && s_prev_state != ONB_ABORT && s_abort_tick == 0u) {
        s_abort_tick = t->tick;
        s_abort_reason = t->reason;
    }
#if FC_VTOL
    {
        const uint16_t um = (uint16_t)(t->esc_u * 1000.0f + 0.5f);
        s_v.phases |= (uint8_t)(1u << (t->phase & 7u));
        if (t->state == ONB_POWERED && s_prev_state != ONB_POWERED) {
            s_v.powered_starts++;
#if !defined(FC_COSIM)
            if (s_v.powered_starts == 2u)                  /* the second hop: the NAV resets 0.5 s after its motor start */
                s_nav_reset_k = (uint16_t)(s_nav_k + SB_NAV_RESET_AFTER_ON2_TICKS);
#endif
        }
        if (um > s_v.u_max_milli) s_v.u_max_milli = um;
        if (t->phase == LP_PHASE_HOLD) {
            s_v.hold_ticks++;
            if (um < s_v.u_hold_min_milli) s_v.u_hold_min_milli = um;
        }
        if (t->state == ONB_POWERED && t->vc_sat > 0) s_v.sat_ticks++;
        if (t->phase == LP_PHASE_LANDED && s_v.landed_tick == 0u) s_v.landed_tick = t->tick;
        if (t->td_by != 0u && s_v.td_by == 0u) s_v.td_by = t->td_by;
#if !defined(FC_COSIM)
        if (t->state == ONB_POWERED) {                     /* v0.8.1: the FCU's own work per tick, without the bench's */
            const uint16_t sbt = counts_to_us16(s_sb_time - s_tick_sb);
            const uint16_t own = t->total_us > sbt ? (uint16_t)(t->total_us - sbt) : 0u;
            if (own > s_v.own_max) s_v.own_max = own;
            s_v.own_sum += own;
            if (s_v.own_n < 0xFFFFu) s_v.own_n++;
        }
#endif
        if (t->phase == LP_PHASE_LAND && s_v.land_tick == 0u) {
            s_v.land_tick = t->tick;
            s_v.land_reason = t->reason;
        }
        if (t->state == ONB_SAFE && s_v.landed_tick != 0u && s_v.safe_tick == 0u) s_v.safe_tick = t->tick;
#if ONB_HAS_GPS
        s_v.pos_requests |= t->pos_requests;
        s_v.pos_modes |= (uint8_t)(1u << (t->pos_mode & 7u));
        if (t->pos_engaged && s_v.pos_engaged_ticks < 0xFFFFu) s_v.pos_engaged_ticks++;
        if (t->tilt_mrad > s_v.tilt_max_mrad) s_v.tilt_max_mrad = t->tilt_mrad;
        if (t->gps_gate_ok && s_v.gate_ok_tick == 0u) s_v.gate_ok_tick = t->tick;
        if (t->slot_fallback && t->state == ONB_POWERED && s_v.fallback_ticks < 0xFFFFu) s_v.fallback_ticks++;
#endif
#if !defined(FC_COSIM)
        s_pl.u_cmd = t->esc_u;                             /* the synthetic plant flies the FCU's ESC command */
#endif
    }
#endif
    s_prev_state = t->state;
#if !defined(FC_COSIM)
    /* anchor -> background end without the simbench's own calls in between (its polls before every background job) */
    const uint16_t be = (uint16_t)(t->bg_end_us - counts_to_us16(s_sb_time - s_tick_sb));
    if (be > s_be_max) s_be_max = be;
    {
        const uint16_t bi = counts_to_us16(s_tick_burned);
        const uint16_t bn = be > bi ? (uint16_t)(be - bi) : 0u;
        if (bn > s_bn_max) s_bn_max = bn;
    }
    if (s_tick_nav && t->nav_ok) {
        /* NAV tick -> vanes written: the frame's time after the NAV tick + the FCU's share of the tick start's lateness +
         * tick start -> vanes (Timer1); and the margin the background would leave before the next exchange on the
         * vehicle, where the tick starts the FCU's share after the frame: next exchange start - (frame + that share +
         * anchor -> background end) */
        const uint32_t nav_tick = (uint32_t)s_tick_fk * SB_TICK_COUNTS;
        const uint32_t t0 = s_nav_f + SB_US(s_lm_tick);
        const uint16_t la = counts_to_us16(t0 + SB_US(t->pwm_us) - nav_tick);
        if (la > s_la_max) s_la_max = la;
        if ((t->state == ONB_POWERED || t->state == ONB_UNPOWERED) && la > s_lc_max) s_lc_max = la;
        if (!nav_silent(s_nav_k) && s_nav_k == (uint16_t)(s_tick_fk + 1u)) {
            const int32_t m = ((int32_t)(s_nav_due_t - SB_US(NLINK_XFER_US)) - (int32_t)(t0 + SB_US(be))) /
                              FC_PWM_COUNTS_PER_US;
            const int16_t m16 = m > 32767 ? 32767 : (m < -32767 ? -32767 : (int16_t)m);
            if (m16 < s_bm_min) s_bm_min = m16;
        }
    }
    sb_com_after_loop(t->tick);
    nav_take_reply();
    if (!s_nav_built && !nav_silent(s_nav_k))
        nav_build(s_nav_k);                                  /* the next frame, with this exchange's FCU state */
#else
    if (t->n_cmds) {                                         /* (la / lc stay 0: only the harness knows the NAV's tick) */
        if (s_scq_n < SC_Q) {
            const uint8_t k = (uint8_t)((s_scq_head + s_scq_n) % SC_Q);
            s_scq[k].tick = t->tick;
            s_scq[k].n = t->n_cmds;
            s_scq[k].q = t->seq_first;
            s_scq[k].r = t->seq_last;
            s_scq[k].w = t->n_wait;
            s_scq[k].b = t->n_crit;
            s_scq[k].c = t->loop_us;
            s_scq_n++;
        } else {
            s_sc_lost++;
        }
    }
    sc_flush();
#endif
    {   /* the synthetic peers' own time (the report lines below come on top a few times per second) */
        const uint16_t d = (uint16_t)(timer_elapsed_counts(ovh0, timer_tcnt1()) / FC_PWM_COUNTS_PER_US);
        if (d > s_ovh_max) s_ovh_max = d;
    }
    if (++s_report_phase >= FC_LOOP_HZ) {
        s_report_phase = 0;
        s_report_due |= 0x01u;
    } else if (s_report_phase == FC_LOOP_HZ / 4u) {
        s_report_due |= 0x02u;
    } else if (s_report_phase == FC_LOOP_HZ / 2u) {
        s_report_due |= 0x04u;
    } else if (s_report_phase == (FC_LOOP_HZ * 3u) / 4u) {
        s_report_due |= 0x08u;
#if FC_VTOL
    } else if (s_report_phase == (FC_LOOP_HZ * 7u) / 8u) {
        s_report_due |= 0x10u;                             /* VTOL: the SV line */
#endif
#if ONB_HAS_GPS
    } else if (s_report_phase == (FC_LOOP_HZ * 5u) / 8u) {
        s_report_due |= 0x20u;                             /* v0.8: the SP line */
#endif
    }
    if (s_report_due) {
#if !defined(FC_COSIM)
        /* a line (~0.4 ms) only while the next frame is far enough off: the simbench must not delay a tick */
        const uint32_t now = sb_now(), nf = s_nav_due_t;
        const bool room = nav_silent(s_nav_k) || (nf > now && nf - now > SB_US(700u));
#else
        const bool room = true;
#endif
        if (room) {
            static const char kinds[6] = {'B', 'N', 'L', 'I', 'V', 'P'};
#if FC_VTOL
            static const uint8_t parts[6] = {3, 2, 2, 2, 3, 2};   /* lines per kind (each < SB_LINE_MAX bytes; v0.8.1 SV 3, SP 2) */
#else
            static const uint8_t parts[6] = {3, 2, 2, 2, 2, 1};   /* lines per kind (each < SB_LINE_MAX bytes) */
#endif
            static uint8_t cur = 0xFFu, part;                    /* the kind being written and its next line */
            uint8_t k = cur;
            if (k == 0xFFu) {
                k = 0;
                while (k < 6u && !(s_report_due & (uint8_t)(1u << k)))
                    k++;                                         /* the oldest due kind: one line per tick */
                part = 0;
            }
            if (k < 6u && report_line(kinds[k], part, t) && ++part < parts[k]) {
                cur = k;                                         /* more lines of this kind follow */
            } else if (k < 6u && part >= parts[k]) {
                cur = 0xFFu;
                s_report_due &= (uint8_t)~(1u << k);
                if (k == 0u) {                                   /* the SB lines close the timing window */
                    s_win_max = 0;
                    s_win_ctrl_max = 0;
                    s_win_total_max = 0;
                    s_bg_step_max = 0;
                    s_snap_step_max = 0;
                    s_il_step_max = 0;
                    s_ovh_max = 0;
                    for (i = 0; i < SB_SECTIONS; i++) s_sec_max[i] = 0;
                }
            }
        }
    }
#if !defined(FC_COSIM)
    sb_hook_end(entry);
#endif
    if (t->tick >= (uint32_t)FC_SIMBENCH_SECONDS * FC_LOOP_HZ && !s_report_due) {
        const char end[] = "\nSB END\n";
        uart_log_write((const uint8_t *)end, sizeof end - 1);
        while (uart_log_free() < UART_LOG_BUF - 1u)          /* drain the log ring */
            wdt_reset();
        cli();
        sleep_enable();
        sleep_cpu();                                          /* simavr stops on sleep with interrupts off */
    }
}

#ifdef FC_SIMBENCH_MICRO
/* ---- micro-benchmarks: CPU cycles per call, Timer1 (8 cycles per count), 32 calls averaged ---- */
#define MB_N 32
static volatile float vf_a = 1.2345f, vf_b = 0.6789f, vf_r, vf_us = 1534.37f;
static volatile int32_t vi_r;
static const void *volatile vp_r;

/* The boot-time benchmarks keep their buffers in s_scratch, which nothing uses before the loop starts (report lines,
 * SNAP takes): [0, SB_MB_PAY) the benchmarks' payload buffer (simbench_scratch), [SB_MB_IN_OFS, +onb_inputs_t) the
 * supervisor benchmarks' inputs, [SB_MB_LINE_OFS, 216) one SBM line - no static RAM of their own (the bench builds
 * fill the 4 KiB; v0.7) */
#define SB_MB_PAY 96u
#define SB_MB_IN_OFS 96u
#if LP_RP1
#define SB_MB_LINE_OFS 164u                /* v0.9 RP1: behind the 67-byte il_frame_t at 96 (v0.8: 160) */
#else
#define SB_MB_LINE_OFS 160u
#endif
/* v0.7.1: onb_inputs_t grew to 85 B (q_nav, accel_z, the controller's V8 / V9 / V10 flags), more than [96, 160) holds:
 * the supervisor / stack benchmarks keep their inputs at the start of the payload buffer instead, which none of them
 * uses (simbench_micro packs the log records only after its supervisor benchmarks; micro_vtol_stack packs nothing) */
#define SB_MB_SUP_OFS 0u
typedef char sb_mb_in_fits[(SB_MB_SUP_OFS + sizeof(onb_inputs_t) <= SB_MB_PAY) ? 1 : -1];
typedef char sb_mb_line_fits[(SB_MB_LINE_OFS + 56u <= sizeof s_scratch) ? 1 : -1];
#define s_mb_in (*(onb_inputs_t *)(void *)(s_scratch + SB_MB_SUP_OFS))

uint8_t *simbench_scratch(void)
{
    return s_scratch;
}

void simbench_micro_line(const char *name_P, uint32_t cyc)
{
    char *const buf = (char *)(s_scratch + SB_MB_LINE_OFS);
    uint8_t p = 0;
    char ch;
    buf[p++] = '\n';
    buf[p++] = 'S'; buf[p++] = 'B'; buf[p++] = 'M'; buf[p++] = ' ';
    while ((ch = (char)pgm_read_byte(name_P++)) != 0 && p < 40u) buf[p++] = ch;
    p = put_u(buf, p, "=", cyc);
    buf[p++] = '\n';
    while (uart_log_free() < p) wdt_reset();
    uart_log_write((const uint8_t *)buf, p);
}

static void mb_line(const char *name_P, uint16_t counts_total, uint16_t counts_empty)
{
    uint32_t cyc = counts_total > counts_empty ? (uint32_t)(counts_total - counts_empty) : 0u;
    simbench_micro_line(name_P, cyc * (uint32_t)(FC_F_CPU_HZ / 1000000UL / FC_PWM_COUNTS_PER_US) / MB_N);
}

#define MB(name, stmt)                                      \
    do {                                                    \
        uint8_t k_;                                         \
        uint16_t a_ = timer_tcnt1(), b_;                    \
        for (k_ = 0; k_ < MB_N; k_++) {                     \
            stmt;                                           \
            wdt_reset();                                    \
        }                                                   \
        b_ = timer_tcnt1();                                 \
        mb_line(PSTR(name), timer_elapsed_counts(a_, b_), empty); \
        wdt_reset();                                        \
    } while (0)

/* the ilink / log part: its objects are locals of this call (boot only, called from main after simbench_micro
 * returned, so the two frames never nest): no static RAM for them */
static uint16_t s_mb_empty;

static void micro_ilink(uint16_t empty, onb_outputs_t *so, uint8_t *pay, uint8_t *fr, il_decoder_t *decp)
    __attribute__((noinline));
static void micro_ilink(uint16_t empty, onb_outputs_t *so, uint8_t *pay, uint8_t *fr, il_decoder_t *decp)
{
    /* frames in main's log buffers (fr: 102 B, pay: 96 B) rather than in locals: this build's static RAM is large.
     * v0.7: the decoder is main's own (re-initialised after the benchmarks) and the decoder's output frame lives in the
     * scratch behind pay ([96, 96 + il_frame_t), unused until simbench_micro_vtol) - the VTOL build's stack is tighter */
    uint8_t *const statf = fr, *const cmdf = fr + 48, *const statp = pay, *const out = pay + 40;
#define dec (*decp)
#define ilf (*(il_frame_t *)(void *)(s_scratch + SB_MB_IN_OFS))
    il_snap_t snap;
    il_stat_t stat;
    log_status_t st;
    log_nav_t nr;
    uint8_t cp8[IL_CMD_LEN], j;
    il_cmd_t c = {7, {LP_CMD_NONE, 0, 0}, -70, 28};
    const float q[4] = {0.99f, 0.05f, -0.08f, 0.1f};
    for (j = 0; j < sizeof st; j++) ((uint8_t *)&st)[j] = (uint8_t)(j + 3u);
    for (j = 0; j < sizeof nr; j++) ((uint8_t *)&nr)[j] = (uint8_t)(j + 5u);
    /* the reply snapshot (tick) and its frame (background) */
    MB("il_snap_from_tlm_src", il_snap_from_tlm_src(3, &so->tlm, q, &snap));
    MB("il_pack_snap", il_pack_snap(&snap, fr + 4));
    MB("il_encode_snap_55", vi_r = il_encode(IL_TYPE_SNAP, fr + 4, IL_SNAP_LEN, fr));
    /* the frames the COM sends (time-critical part / background / idle) */
    il_pack_cmd(&c, cp8);
    il_encode(IL_TYPE_CMD, cp8, IL_CMD_LEN, cmdf);
    for (j = 0; j < IL_STAT_LEN; j++) statp[j] = (uint8_t)(j * 7u + 1u);
    il_encode(IL_TYPE_STAT, statp, IL_STAT_LEN, statf);
    il_decoder_init(&dec, IL_ACCEPT_FCU);
    MB("il_decoder_cmd_frame_14", for (j = 0; j < IL_CMD_FRAME_BYTES; j++) vi_r = il_decoder_feed(&dec, cmdf[j], &ilf, 1u));
    MB("il_decoder_stat_frame_42",
       for (j = 0; j < IL_STAT_FRAME_BYTES; j++) vi_r = il_decoder_feed(&dec, statf[j], &ilf, 1u));
    MB("il_decoder_hunting_byte", vi_r = il_decoder_feed(&dec, 0x00, &ilf, 1u));
    MB("il_decoder_push_cmd_frame_14", for (j = 0; j < IL_CMD_FRAME_BYTES; j++) vp_r = il_decoder_push(&dec, cmdf[j]));
    MB("il_decoder_push_stat_frame_42",
       for (j = 0; j < IL_STAT_FRAME_BYTES; j++) vp_r = il_decoder_push(&dec, statf[j]));
    MB("il_decoder_push_hunting_byte", vp_r = il_decoder_push(&dec, 0x00));
    MB("il_decoder_need", vi_r = il_decoder_need(&dec));
    MB("il_decoder_cmd_need", vi_r = il_decoder_cmd_need(&dec));
    MB("il_unpack_cmd", il_unpack_cmd(cmdf + 4, &c));
    MB("il_unpack_stat", il_unpack_stat(statp, &stat));
    MB("log_pack_com", vi_r = log_pack_com(1234u, statp, out));
    MB("log_pack_status", vi_r = log_pack_status(&st, out));
    MB("log_pack_nav", vi_r = log_pack_nav(&nr, out));
#undef dec
#undef ilf
}
typedef char sb_mb_ilf_fits[(SB_MB_IN_OFS + sizeof(il_frame_t) <= SB_MB_LINE_OFS) ? 1 : -1];

void simbench_micro_ilink(onb_outputs_t *so, uint8_t *pay, uint8_t *fr, il_decoder_t *dec)
{
    micro_ilink(s_mb_empty, so, pay, fr, dec);
}

/* the nlink part: a valid / a broken navigation frame, the FCU frame, the single-chip bypass's take (frame and reply
 * buffers in main's log buffers) */
static void micro_nlink(uint16_t empty, uint8_t *pay, uint8_t *fr) __attribute__((noinline));
static void micro_nlink(uint16_t empty, uint8_t *pay, uint8_t *fr)
{
    nl_nav_t n, d;
    nl_fcu_t f = {7, ONB_POWERED, 0, FC_PARAMS_TAG, 229};
    uint8_t j;
    for (j = 0; j < sizeof n; j++) ((uint8_t *)&n)[j] = (uint8_t)(j * 3u + 1u);
    n.q[0] = 0.99f; n.q[1] = 0.05f; n.q[2] = -0.08f; n.q[3] = 0.1f; n.tilt_deg = 3.5f;
    MB("nl_encode_nav", vi_r = nl_encode_nav(&n, fr));
    MB("nl_decode_nav_valid", vi_r = nl_decode_nav(fr, &d));
    for (j = 0; j < NL_XFER_BYTES; j++) pay[j] = fr[j];
    pay[NL_XFER_BYTES - 1u] ^= 1u;
    MB("nl_decode_nav_bad_crc", vi_r = nl_decode_nav(pay, &d));
    MB("nl_encode_fcu", vi_r = nl_encode_fcu(&f, pay));
    MB("nlink_spi_set_reply", nlink_spi_set_reply(pay));
    MB("nlink_spi_frame_ready_none", vi_r = nlink_spi_frame_ready());
    MB("nlink_spi_inject_take_bypass", (nlink_spi_inject_frame(fr), vi_r = nlink_spi_take(pay)));
}

void simbench_micro_nlink(uint8_t *pay, uint8_t *fr)
{
    micro_nlink(s_mb_empty, pay, fr);
}

/* boot-time parameter structs only in these frames, not under the benchmarks below (the micro build's stack is tight) */
static void mb_init_ctrl(attitude_ctrl_t *cp, const float q[4]) __attribute__((noinline));
static void mb_init_ctrl(attitude_ctrl_t *cp, const float q[4])
{
    mixer_plant_t plant;
    att_gains_t g;
    fc_default_plant(&plant);
    fc_default_gains(&g);
    attitude_ctrl_init(cp, &g, &plant, fc_throttle_exp_int());
    attitude_ctrl_set_yaw_reference(cp, q);
}

static void mb_init_sup(onboard_t *sup) __attribute__((noinline));
static void mb_init_sup(onboard_t *sup)
{
#if ONB_PARAMS_FLASH
    onboard_init(sup, fc_onboard_params(), 1, false);   /* v0.8.1: the VTOL AVR build's flash table */
#else
    onb_params_t opar;
    fc_default_onboard(&opar);
    onboard_init(sup, &opar, 1, false);
#endif
}

static onb_frame_t s_mb_hb;                                /* the supervisor benchmarks' heartbeat (s_mb_in: s_scratch) */

void simbench_micro(attitude_ctrl_t *cp, ctrl_output_t *op, onboard_t *sup, onb_outputs_t *so, log_flight_t *recp,
                    uint8_t *pay, uint8_t *fr)
{
    /* v0.7: the quaternions and the rate vector live in fr (main's g_frame, unused until the log benchmarks at the end):
     * this frame sits under attitude_ctrl_update, the deepest call of the boot-time benchmarks */
    float *const q = (float *)(void *)fr, *const q2 = q + 4, *const r = q + 8, *const w = q + 12;
    uint16_t empty;
    {
        uint8_t k_;
        uint16_t a_ = timer_tcnt1(), b_;
        for (k_ = 0; k_ < MB_N; k_++) { vf_r = vf_a; wdt_reset(); }
        b_ = timer_tcnt1();
        empty = timer_elapsed_counts(a_, b_);
    }
    q[0] = 0.99f; q[1] = 0.05f; q[2] = -0.08f; q[3] = 0.1f;
    q2[0] = 0.98f; q2[1] = -0.1f; q2[2] = 0.05f; q2[3] = 0.15f;
    w[0] = 0.3f; w[1] = -0.2f; w[2] = 0.1f;
    mb_init_ctrl(cp, q);
    mb_init_sup(sup);
    s_mb_hb.seq = 1;
    s_mb_hb.cmd.cmd = LP_CMD_NONE;
    s_mb_hb.cmd.arg = 0;
    s_mb_hb.cmd.nonce = 0;
    s_mb_in.tick = 1; s_mb_in.frames = &s_mb_hb; s_mb_in.n_frames = 1; s_mb_in.h_m = 10.0f; s_mb_in.vz_mps = 5.0f; s_mb_in.tilt_deg = 3.0f;
    s_mb_in.gyro[0] = 0.1f; s_mb_in.gyro[1] = -0.2f; s_mb_in.gyro[2] = 0.3f; s_mb_in.gyro_saturated = false; s_mb_in.v_cell = 4.0f;
    s_mb_in.imu_ok = s_mb_in.baro_ok = s_mb_in.aligned = s_mb_in.arm_switch = true;
    s_mb_in.roll_deg = 1.0f; s_mb_in.pitch_deg = -2.0f; s_mb_in.yaw_deg = 30.0f; s_mb_in.loop_max_us = 3000.0f;
    s_mb_in.cos_tilt = 0.9986f;                            /* v0.8: cos(tilt) instead of the quaternion */
    s_mb_in.gps_slot = 0;
    s_mb_in.accel_z_mps2 = 9.7f;
    MB("fadd", vf_r = vf_a + vf_b);
    MB("fmul", vf_r = vf_a * vf_b);
    MB("fdiv", vf_r = vf_a / vf_b);
    MB("fcmp_lt", vi_r = vf_a < vf_b);
    MB("f2i32_cast", vi_r = (int32_t)(vf_a * 1000.0f));
    MB("lroundf", vi_r = lroundf(vf_a * 1000.0f));
    MB("sqrtf", vf_r = sqrtf(vf_a));
    MB("sinf", vf_r = sinf(vf_a));
    MB("cosf", vf_r = cosf(vf_a));
    MB("atan2f", vf_r = atan2f(vf_a, vf_b));
    MB("asinf", vf_r = asinf(vf_b));
    MB("acosf", vf_r = acosf(vf_b));
    MB("powf", vf_r = powf(vf_a, vf_b));
    MB("pwm_us_to_counts", vi_r = pwm_us_to_counts(vf_us));
    MB("quat_multiply", quat_multiply(q, q2, r));
    MB("quat_integrate", quat_integrate(q, w, FC_DT_S, r));
    MB("quat_to_rotvec", quat_to_rotvec(q, r));
    MB("attitude_ctrl_update", attitude_ctrl_update(cp, q, w, 0.9f, op));
    MB("attitude_ctrl_set_yaw_reference", attitude_ctrl_set_yaw_reference(cp, q));
#if FC_VTOL && ATT_HAS_TILT_SP
    {   /* v0.8.1: the tilted setpoint (the GNS chain in position hold): a new tilt every call (set_tilt + its finish), then
         * the controller update with tilt_on (the general quaternion error) - the cost the time-critical part carries in
         * position hold; then the setpoint back to level */
        static const float t1[2] = {0.02f, 0.01f}, t2[2] = {0.021f, 0.0105f}, t0[2] = {0.0f, 0.0f};
        MB("attitude_ctrl_set_tilt_new", (void)attitude_ctrl_set_tilt(cp, (k_ & 1u) ? t1 : t2));
        MB("attitude_ctrl_set_tilt_finish", ((void)attitude_ctrl_set_tilt(cp, (k_ & 1u) ? t2 : t1), attitude_ctrl_set_tilt_finish(cp)));
        MB("attitude_ctrl_update_tilt", attitude_ctrl_update(cp, q, w, 0.9f, op));
        (void)attitude_ctrl_set_tilt(cp, t0);
        attitude_ctrl_set_tilt_finish(cp);
        MB("attitude_ctrl_update_level", attitude_ctrl_update(cp, q, w, 0.9f, op));
    }
#endif
    MB("onboard_update_hb", (s_mb_in.tick++, onboard_update(sup, &s_mb_in, so)));
    s_mb_in.n_frames = 0;
    MB("onboard_update_noframe", (s_mb_in.tick++, onboard_update(sup, &s_mb_in, so)));
    s_mb_empty = empty;
    {
        uint8_t k;
        for (k = 0; k < 4; k++) {               /* realistic, non-constant record contents */
            recp->q[k] = vf_a * 0.3f;
            recp->delta[k] = vf_b * 0.1f;
        }
        for (k = 0; k < 3; k++) {
            recp->gyro_f[k] = vf_a; recp->accel[k] = vf_b * 9.0f; recp->att_err[k] = vf_b * 0.01f;
            recp->moments[k] = vf_a * 0.2f; recp->integ[k] = vf_b * 0.1f;
        }
        recp->h_est = vf_a * 100.0f; recp->vz_est = vf_b * 20.0f; recp->h_baro = vf_a * 99.0f;
        recp->throttle = vf_b; recp->v_cell = vf_a * 3.0f; recp->tilt_deg = vf_b * 10.0f; recp->thrust_est = vf_a * 30.0f;
    }
    MB("log_fixed", vi_r = log_fixed(vf_a, LOG_SHIFT_ANGLE, -32768L, 32767L));
    MB("log_pack_flight_part0", vi_r = log_pack_flight_part(recp, pay, 0));
    MB("log_pack_flight_part1", vi_r = log_pack_flight_part(recp, pay, 1));
    MB("log_pack_flight_part2", vi_r = log_pack_flight_part(recp, pay, 2));
    MB("log_pack_flight_part3", vi_r = log_pack_flight_part(recp, pay, 3));
    MB("log_pack_flight", vi_r = log_pack_flight(recp, pay));
    MB("log_frame_96", vi_r = log_frame(LOG_TYPE_FLIGHT, pay, 90, fr));
}

#if FC_VTOL
/* VTOL: the stack and the vertical controller on main's own object (before main initialises it), the SNAP2 path and
 * the format-5 records.  onb_stack_step in HOLD is timed with the supervisor put into POWERED / HOLD directly (a
 * benchmark, not a flight: getting there through ON and the spool warning would take 500 ticks) */
static void mb_init_stack(onb_stack_t *k) __attribute__((noinline));
static void mb_init_stack(onb_stack_t *k)
{
    /* v0.8.1: the flash tables of the VTOL AVR build (core/fc_config.h), the GNS chain */
#if ONB_HAS_GPS
    onb_stack_init(k, fc_onboard_params(), fc_vertical_params(), fc_position_params(), ONB_CHAIN_GNS, 1, false);
#else
    onb_stack_init(k, fc_onboard_params(), fc_vertical_params(), 0, ONB_CHAIN_GNS, 1, false);
#endif
}

/* the stack benchmarks: no locals of their own (s_mb_in lives in s_scratch), so onb_stack_step runs at about the depth it
 * has in the loop - the VTOL micro build has the least stack of all */
static void micro_vtol_stack(onb_stack_t *k, onb_outputs_t *so) __attribute__((noinline));
static void micro_vtol_stack(onb_stack_t *k, onb_outputs_t *so)
{
    const uint16_t empty = s_mb_empty;
    mb_init_stack(k);
    s_mb_hb.seq = 1;
    s_mb_hb.cmd.cmd = LP_CMD_NONE;
    s_mb_hb.cmd.arg = 0;
    s_mb_hb.cmd.nonce = 0;
    s_mb_in.tick = 1; s_mb_in.frames = &s_mb_hb; s_mb_in.n_frames = 1; s_mb_in.h_m = 4.9f; s_mb_in.vz_mps = 0.12f;
    s_mb_in.tilt_deg = 3.0f; s_mb_in.gyro[0] = 0.01f; s_mb_in.gyro[1] = -0.02f; s_mb_in.gyro[2] = 0.03f;   /* < I3 */
    s_mb_in.gyro_saturated = false; s_mb_in.v_cell = 4.0f;
    s_mb_in.imu_ok = s_mb_in.baro_ok = s_mb_in.aligned = s_mb_in.arm_switch = true;
    s_mb_in.roll_deg = 1.0f; s_mb_in.pitch_deg = -2.0f; s_mb_in.yaw_deg = 30.0f; s_mb_in.loop_max_us = 3000.0f;
    s_mb_in.cos_tilt = 0.9986f;                            /* v0.8: cos(tilt) instead of the quaternion */
    s_mb_in.gps_slot = 0;
    s_mb_in.accel_z_mps2 = 9.7f;                           /* a hover: no support signature */
    MB("onb_stack_step_safe_hb", (s_mb_in.tick++, vf_r = onb_stack_step(k, &s_mb_in, so)));
    k->sup.state = ONB_POWERED;                            /* benchmark only: HOLD at 5 m, motor on */
    k->sup.phase = LP_PHASE_HOLD;
    k->sup.tgt_dm = 50;
    k->sup.t_entry = s_mb_in.tick;
    MB("onb_stack_step_hold_hb", (s_mb_in.tick++, vf_r = onb_stack_step(k, &s_mb_in, so)));
    s_mb_in.n_frames = 0;
    MB("onb_stack_step_hold_noframe", (s_mb_in.tick++, vf_r = onb_stack_step(k, &s_mb_in, so)));
    simbench_micro_line(PSTR("vtol_check_hold_kept"), k->sup.state == ONB_POWERED && k->sup.phase == LP_PHASE_HOLD);
}

static void micro_vtol_vc(vc_t *vc, vc_outputs_t *vo) __attribute__((noinline));
static void micro_vtol_vc(vc_t *vc, vc_outputs_t *vo)
{
    const uint16_t empty = s_mb_empty;
    vc_inputs_t vi;
    const float q[4] = {0.999f, 0.02f, -0.015f, 0.03f};
    vi.phase = LP_PHASE_HOLD; vi.h_tgt_dm = 50; vi.baro_lost = false; vi.h_m = 4.9f; vi.vz_mps = 0.12f;
    vi.cos_tilt = vc_cos_tilt_from_q(q); vi.f_z_mps2 = 9.7f;
    MB("vc_cos_tilt_from_q", vf_r = vc_cos_tilt_from_q(q));
    MB("vc_update_hold", vc_update(vc, &vi, vo));
    vi.phase = LP_PHASE_CLIMB; vi.h_tgt_dm = 100;
    MB("vc_update_climb", vc_update(vc, &vi, vo));
    vi.phase = LP_PHASE_LAND; vi.h_m = 0.8f; vi.vz_mps = -0.05f;
    MB("vc_update_land_flare", vc_update(vc, &vi, vo));
    vi.baro_lost = true;
    MB("vc_update_land_baro_lost", vc_update(vc, &vi, vo));
    vi.phase = LP_PHASE_NONE;
    MB("vc_update_idle", vc_update(vc, &vi, vo));
}

/* the SNAP2 path and the format-6 records; objects in the borrowed buffers: pay [0, 55) the SNAP2 struct, [56, 103) the
 * 'V' record (s_scratch, idle here); fr [0, 61) the SNAP2 frame, [61, 102) one record frame (payload packed at its frame
 * offset) */
static void micro_vtol_records(onb_outputs_t *so, uint8_t *pay, uint8_t *fr) __attribute__((noinline));
static void micro_vtol_records(onb_outputs_t *so, uint8_t *pay, uint8_t *fr)
{
    const uint16_t empty = s_mb_empty;
    il_snap2_t *const s2 = (il_snap2_t *)(void *)pay;
    log_vtol_t *const vr = (log_vtol_t *)(void *)(pay + 56);
    uint8_t *const buf = fr, *const rec = fr + 61, j;     /* v0.7.1: the 41-byte format-6 frame behind the SNAP2 frame */
    const float q[4] = {0.99f, 0.05f, -0.08f, 0.1f};
    so->tlm.v2 = true;
    so->tlm.phase = LP_PHASE_HOLD;
    so->tlm.h_tgt_m = 5.0f;
    MB("il_snap2_from_tlm_src", il_snap2_from_tlm_src(3, &so->tlm, q, s2));
    MB("il_pack_snap2", il_pack_snap2(s2, buf + 4));
    MB("il_encode_snap2_61", vi_r = il_encode(IL_TYPE_SNAP2, buf + 4, IL_SNAP2_LEN, buf));
    for (j = 0; j < sizeof *vr; j++) ((uint8_t *)vr)[j] = (uint8_t)(j * 7u + 1u);
    vr->h_ref = 4.95f; vr->vz_cmd = -0.31f; vr->u = 0.93f; vr->u_ff = 0.917f; vr->integ = 0.013f;
    vr->vz_lpf = -0.21f; vr->f_excess = 0.37f;
    MB("log_pack_vtol", vi_r = log_pack_vtol(vr, rec + 4));
    MB("log_pack_event2", vi_r = log_pack_event2(1234u, ONB_POWERED, ONB_POWERED, 0x7Cu, 0, LP_PHASE_CLIMB,
                                                 LP_PHASE_HOLD, rec + 4));
    (void)log_pack_vtol(vr, rec + 4);
    MB("log_frame_vtol_41", vi_r = log_frame(LOG_TYPE_VTOL, rec + 4, LOG_VTOL_LEN, rec));
}

void simbench_micro_vtol(onb_stack_t *k, onb_outputs_t *so, uint8_t *pay, uint8_t *fr)
{
    micro_vtol_stack(k, so);
    micro_vtol_vc(&k->vc, &k->vo);
    micro_vtol_records(so, pay, fr);
}
#endif
#endif /* FC_SIMBENCH_MICRO */
#endif /* FC_SIMBENCH */
