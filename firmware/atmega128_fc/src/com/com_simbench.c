/* com_simbench.c - see com_simbench.h (compiled only with -D FC_SIMBENCH; FC_COSIM selects the co-simulation mode). */
#ifdef FC_SIMBENCH
#include "com_simbench.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <util/atomic.h>

#include "../fc_mode.h"
#include "../params_gen.h"
#include "../core/fcmath.h"
#include "../core/ilink_proto.h"
#include "../core/link_proto.h"
#include "../core/onboard.h"
#include "../hal/ilink_uart.h"
#include "../hal/radio.h"
#include "../hal/stack.h"
#include "com_cosim.h"
#include "com_link.h"
#include "com_timer.h"

#if COM_COSIM_STAT_BOOT != COM_STAT_KIND_BOOT || COM_COSIM_STAT_PIGGYBACK != COM_STAT_KIND_PIGGYBACK || \
    COM_COSIM_STAT_QUIET != COM_STAT_KIND_QUIET
#error "com_cosim.h COM_COSIM_STAT_* must equal com_link.h COM_STAT_KIND_*"
#endif

#if FC_LINK_RADIO == FC_RADIO_LORA_UART
#define SB_RADIO_NAME "lora_uart"
#define SB_BACKEND_FLAG 0u
/* the module's UART delivers one byte every 10 / LINK_UART_BAUD s (Timer1 counts) */
#define SB_RADIO_BYTE ((uint32_t)((COM_T1_COUNTS_PER_US * 10000000UL + LINK_UART_BAUD / 2u) / LINK_UART_BAUD))
#define SB_BURST_GAP_TICKS 0u                /* the next burst frame starts at the next tick after the previous one */
#elif FC_LINK_RADIO == FC_RADIO_ESP_BRIDGE
/* 2026-11-06: the ESP bridge delivers the bytes over USART1 at ESP_UART_BAUD, like a transparent module at its baud */
#define SB_RADIO_NAME "esp_bridge"
#define SB_BACKEND_FLAG 0u
#define SB_RADIO_BYTE ((uint32_t)((COM_T1_COUNTS_PER_US * 10000000UL + ESP_UART_BAUD / 2u) / ESP_UART_BAUD))
#define SB_BURST_GAP_TICKS 0u
#else
#define SB_RADIO_NAME "sx127x"
#define SB_BACKEND_FLAG IL_STAT_SX127X
#define SB_RADIO_BYTE 0u                     /* the whole packet appears at RxDone */
#define SB_BURST_GAP_TICKS 12u               /* a 13-byte CMD packet is ~46 ms on air at SF7 / 125 kHz / CR 4/5 */
#endif
#define SB_IL_BYTE ((uint32_t)((COM_T1_COUNTS_PER_US * 10000000UL + ILINK_BAUD / 2u) / ILINK_BAUD))   /* 40 us */
#define SB_TICK_COUNTS ((uint32_t)COM_T1_COUNTS_PER_US * (1000000UL / COM_TICK_HZ))                    /* 4 ms */
#define SB_POLL_TICKS ((uint16_t)(((uint32_t)LINK_RTT_WORST_MS * COM_TICK_HZ + 999u) / 1000u))
#define SB_BURST_HOLD_TICKS (3u * COM_TICK_HZ / 2u)   /* normal polling resumes 1.5 s after the burst started */
#define SB_FLAGS_PAD (LP_FLAG_ARM_SWITCH | LP_FLAG_SENSORS_OK | LP_FLAG_ALIGNED | LP_FLAG_LINK_OK)
/* D5 seen from the ground, in real time (the raw clock below): a TELEM must be handed to the radio less than the reply
 * window after its IL_CMD was queued (com_link.h: < COM_REPLY_WINDOW_TICKS after the queue tick began) plus the pass that
 * hands it over may run past a tick boundary: + 3 ms (> every measured pass) */
#define SB_WINDOW_COUNTS ((uint32_t)COM_REPLY_WINDOW_TICKS * SB_TICK_COUNTS + 3000UL * COM_T1_COUNTS_PER_US)
#define SB_QUIET_FROM ((uint32_t)SB_QUIET_S * COM_TICK_HZ)
#define SB_QUIET_UNTIL (SB_QUIET_FROM + SB_QUIET_TICKS)
#if defined(COM_SB_FIELD) && !(defined(FC_COSIM) && FC_VTOL)
#error "COM_SB_FIELD is the VTOL co-simulation's field profile (env com_cosim_cl)"
#endif
#ifdef COM_SB_FIELD
/* v0.9 closed-loop co-simulation (fw09 WP-S, tools/run_cosim4.py): the synthetic ground is the field profile's operator -
 * ON from SB_ARM_S (repeated until the FCU takes it: its GPS ON gate), then while POWERED: DOWN, UP, OFF; one hop; no
 * burst and no quiet window (both are fault patterns of the open-loop runs).  A tether build refuses a DOWN / UP outside
 * its altitude limits (REJ, by design; recorded by the co-simulation).  The seconds are the COM's ticks since its boot. */
#ifndef COM_SB_DOWN_S
#define COM_SB_DOWN_S 19u
#endif
#ifndef COM_SB_UP_S
#define COM_SB_UP_S 24u
#endif
#ifndef COM_SB_OFF_S
#define COM_SB_OFF_S 30u
#endif
static uint8_t s_field_done;                 /* bit 0 ON taken, 1 DOWN sent, 2 UP sent, 3 OFF sent */
#endif
/* this build's reply snapshot and TELEM (fc_mode.h, com_link.c): SNAP -> TELEM v1 (ballistic), SNAP3 -> TELEM3 (VTOL v0.8:
 * the synthetic FCU is the GPS position-hold FCU; SNAP2 -> TELEM2 of an ONB_NO_GPS FCU is the same path minus the GPS
 * fields, checked by the host test / cross-check vectors) */
#if FC_VTOL
#define SB_SNAP_TYPE IL_TYPE_SNAP3
#define SB_SNAP_LEN IL_SNAP3_LEN                         /* the longest snapshot (v0.9: SNAP2 57 every 4th, SNAP3 65) */
#define SB_SNAP_FRAME_BYTES IL_SNAP3_FRAME_BYTES
#define SB_TELEM_TYPE LP_TYPE_TELEM3
#define SB_TELEM_FRAME_BYTES LP_TELEM5_FRAME_BYTES       /* the longest TELEM (v0.9 RP1: TELEM v5, 47 B) */
#else
#define SB_SNAP_TYPE IL_TYPE_SNAP
#define SB_SNAP_LEN IL_SNAP_LEN
#define SB_SNAP_FRAME_BYTES IL_SNAP_FRAME_BYTES
#define SB_TELEM_TYPE LP_TYPE_TELEM
#define SB_TELEM_FRAME_BYTES LP_TELEM_FRAME_BYTES
#endif

/* ---- simbench clock: Timer1 counts of COM time, extended at every pass (passes << 32 ms).  Single chip: it stands
 * still while com_sb_pass runs, so the bench's own work (building SNAPs with sinf / cosf, formatting the report) is
 * invisible to the byte schedule and to every latency: the COM sees its peers as if they cost nothing.  Co-sim: real
 * time (the FCU on the other end does not stand still).  s_t1 = Timer1 where s_now was last brought up to date. ---- */
static uint32_t s_now;
static uint16_t s_t1;
/* the raw clock: Timer1 counts of real COM time since com_timer_init (Timer1 starts there from 0; com_sb_init takes it
 * over a few us later, long before its first wrap), never frozen (the COM tick and so the D5 window run on real time
 * even while the single-chip bench clock stands still); in a co-simulation it runs with the bench clock, ahead of it by
 * the few us between com_timer_init and com_sb_init */
static uint32_t s_raw;
static uint16_t s_raw_t1;

static uint32_t now_counts(void)
{
    return s_now + (uint16_t)(com_timer_tcnt1() - s_t1);
}

static uint32_t raw_counts(void)
{
    return s_raw + (uint16_t)(com_timer_tcnt1() - s_raw_t1);
}

static bool due(uint32_t t)
{
    return (int32_t)(s_now - t) >= 0;
}

static uint32_t counts_to_us(uint32_t c)
{
    c /= COM_T1_COUNTS_PER_US;
    return c;
}

/* ---- hook events: the hooks inside the measured pass only queue (kind, seq, time); com_sb_pass works them off ---- */
#define SB_EVQ 8u
static struct {
    uint32_t t, raw;                         /* bench clock, raw clock */
    uint8_t kind, seq;
} s_ev[SB_EVQ];
static uint8_t s_ev_n;
static uint16_t s_ev_lost;

void com_sb_event(uint8_t kind, uint8_t seq)
{
    if (s_ev_n >= SB_EVQ) {
        s_ev_lost++;
        return;
    }
    s_ev[s_ev_n].t = now_counts();
    s_ev[s_ev_n].raw = raw_counts();
    s_ev[s_ev_n].kind = kind;
    s_ev[s_ev_n].seq = seq;
    s_ev_n++;
}

/* ---- per-uplink records: co-sim in the report, single chip a short ring (latencies and the D5 check) ---- */
#ifdef FC_COSIM
volatile com_cosim_report_t g_cosim_report;
#define SB_UL_RING COM_COSIM_UPLINKS
#define SB_UL(i) (g_cosim_report.ul[(i)])
#define REP_BEGIN() (g_cosim_report.seqlock++)
#define REP_END() (g_cosim_report.seqlock++)
#else
#define SB_UL_RING 8u                        /* > the uplinks of one reply window */
static volatile com_cosim_uplink_t s_ul[SB_UL_RING];
#define SB_UL(i) (s_ul[(i)])
#define REP_BEGIN() ((void)0)
#define REP_END() ((void)0)
#endif
typedef volatile com_cosim_uplink_t sb_ul_t;
static uint16_t s_ul_count;                  /* uplinks started; record of uplink u at (u % SB_UL_RING) */
static uint32_t s_ul_raw_q[SB_UL_RING];      /* raw clock when each record's IL_CMD was queued (the D5 age check) */

/* the newest record with this seq that has every flag of need and none of lack, or 0 */
static sb_ul_t *ul_find(uint8_t seq, uint8_t need, uint8_t lack)
{
    uint16_t u = s_ul_count;
    uint8_t k;
    for (k = 0; k < SB_UL_RING && u > 0u; k++) {
        sb_ul_t *r = &SB_UL(--u & (SB_UL_RING - 1u));
        if (r->seq == seq && (r->flags & need) == need && (r->flags & lack) == 0u)
            return r;
    }
    return 0;
}

/* ---- the synthetic ground station ---- */
static struct {
    bool have_tlm, pending_sent, burst_done, burst_open, burst_reply, burst_expired, rx_new;   /* burst_open: until the next poll */
    uint8_t gseq, pending, tlm_state, tlm_ack, burst_left, burst_last;   /* burst_last: SEQ of its last frame */
    uint16_t tlm_nonce;
    uint8_t txb[LP_CMD_FRAME_BYTES], n, pos;
    uint32_t next_t;                         /* arrival time of the next byte */
    uint32_t next_poll, burst_next;
    lp_cmd_t sent[8];                        /* what was sent with SEQ & 7 (checked at the synthetic FCU) */
    uint8_t rx[SB_TELEM_FRAME_BYTES], rx_n;  /* the TELEM (VTOL: TELEM2 .. 5) the COM handed to the radio (hook copy) */
    uint16_t uplinks, replies, tlm_bad, out_of_window, unmatched, launches, refused_seen, expired_seen;
    uint32_t age_max;                        /* longest IL_CMD queued -> TELEM handed to the radio, raw counts */
    bool rx_seen;                            /* 2026-11-10 (F4): the first uplink byte went into the radio driver ... */
    uint32_t first_rx_tick;                  /* ... in this COM tick (com_timer_ticks) */
} s_g;

static uint8_t s_reset_cause;
static uint32_t s_tick;
static uint16_t s_mark[COM_SB_SECTIONS];
static uint32_t s_boot_raw = 0xFFFFFFFFUL;   /* R3: raw clock (counts since com_timer_init) when the COM queued its boot STAT */

/* ======================================================================================== */
#ifndef FC_COSIM
/* single chip: console, synthetic FCU, one-second report lines                              */
/* ======================================================================================== */
/* ---- console: USART0, TX only (the ilink USART is bypassed in this build), interrupt-driven ring ---- */
#define SB_CON_BAUD 250000UL
#define SB_CON_UBRR (FC_F_CPU_HZ / 16UL / SB_CON_BAUD - 1UL)
#define SB_CON_BUF 1024u                     /* power of two: the four report lines of one second (< 1000 B; the host checks td = 0) */
static volatile uint8_t s_con[SB_CON_BUF];
static volatile uint16_t s_con_head, s_con_tail;
static uint16_t s_con_dropped;
static char s_line[256];                     /* one line at a time, < 256 characters */

ISR(USART0_UDRE_vect)
{
    const uint16_t t = s_con_tail;
    if (s_con_head == t) {
        UCSR0B &= (uint8_t)~(1 << UDRIE0);
        return;
    }
    UDR0 = s_con[t];
    s_con_tail = (uint16_t)((t + 1u) & (SB_CON_BUF - 1u));
}

static uint16_t con_free(void)
{
    uint16_t h, t;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        h = s_con_head;
        t = s_con_tail;
    }
    return (uint16_t)((SB_CON_BUF - 1u) - (uint16_t)((h - t) & (SB_CON_BUF - 1u)));
}

static bool con_write(const char *s, uint16_t n)
{
    uint16_t h, i;
    if (n > con_free()) {
        s_con_dropped++;
        return false;
    }
    h = s_con_head;                          /* only this function writes the head */
    for (i = 0; i < n; i++) {
        s_con[h] = (uint8_t)s[i];
        h = (uint16_t)((h + 1u) & (SB_CON_BUF - 1u));
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        s_con_head = h;
        UCSR0B |= (uint8_t)(1 << UDRIE0);
    }
    return true;
}

/* line building: every key and text lives in flash (PSTR via the put_* macros below), so the report keys cost no RAM
 * (the bench build's static RAM leaves ~0.3 KiB for the stack) */
static const char s_hex[16] PROGMEM = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};

static uint16_t put_s_P(uint16_t p, const char *s)
{
    char c;
    while ((c = (char)pgm_read_byte(s++)) != '\0')
        s_line[p++] = c;
    return p;
}

/* "key0x<hex> " (hex: no 32-bit division) */
static uint16_t put_u_P(uint16_t p, const char *key, uint32_t v)
{
    int8_t sh;
    bool lead = true;
    p = put_s_P(p, key);
    s_line[p++] = '0';
    s_line[p++] = 'x';
    for (sh = 28; sh >= 0; sh -= 4) {
        const uint8_t d = (uint8_t)((v >> sh) & 0xFu);
        if (d || !lead || sh == 0) {
            s_line[p++] = (char)pgm_read_byte(&s_hex[d]);
            lead = false;
        }
    }
    s_line[p++] = ' ';
    return p;
}

static uint16_t put_hex_P(uint16_t p, const char *key, const uint8_t *b, uint8_t n)
{
    uint8_t i;
    p = put_s_P(p, key);
    for (i = 0; i < n; i++) {
        s_line[p++] = (char)pgm_read_byte(&s_hex[b[i] >> 4]);
        s_line[p++] = (char)pgm_read_byte(&s_hex[b[i] & 0xFu]);
    }
    s_line[p++] = ' ';
    return p;
}

/* key / text arguments are string literals */
#define put_s(p, s) put_s_P((p), PSTR(s))
#define put_u(p, key, v) put_u_P((p), PSTR(key), (v))
#define put_hex(p, key, b, n) put_hex_P((p), PSTR(key), (b), (n))

/* ---- the synthetic FCU ---- */
static struct {
    il_decoder_t dec;
    uint32_t take_t;                         /* the next COM TX byte has fully arrived at this time */
    il_cmd_t q[ONB_MAX_FRAMES];
    uint8_t nq;
    uint16_t cmds, cmd_bad, snaps, nonce_rej, stats, stat_bad, wrong_frames;
    uint16_t st_radio_ok, st_radio_nok;                 /* 2026-11-10 (F4): STATs with / without IL_STAT_RADIO_OK */
    /* supervisor model (VTOL: ON -> ARMED / SPOOL for ONB_V_SPOOL_TICKS -> POWERED / CLIMB at 1 m/s -> HOLD at the
     * hover target; the phase and the target go into the SNAP2) */
    uint8_t state, reason, last_seq, phase;
    bool has_seq, rejected;
    uint16_t nonce, thr;
    uint32_t t_launch;
    /* transmitter: up to two frames queued (SNAPs, and the R4 wrong-direction IL_CMD / IL_STAT); tx_bad: no TELEM to
     * time - a SNAP sent on purpose to be refused (D5) or a wrong-direction frame */
    uint8_t tx[2][SB_SNAP_FRAME_BYTES], tx_len[2], tx_seq[2], tx_head, tx_count, tx_pos;
    bool tx_bad[2];
    uint32_t tx_next_t;
    uint16_t tx_drop, bad_sent, wrong_sent;
    bool stray_done, dup_done, late_done, late_pending, wrong_done;
    uint8_t late_seq, late_pay[SB_SNAP_LEN], late_len;
    uint32_t late_tick;
    uint32_t snap_done_t[8];
    uint8_t sl_valid;
    uint8_t snap_pay[4][SB_SNAP_LEN], snap_pay_seq[4], snap_len[4];
    uint8_t snap_tlm_type[4];                /* v0.9: the TELEM TYPE the COM must build from that snapshot */
    il_stat_t last_stat;
    /* D4: the frames as they arrive (Timer1 counts / COM ticks) */
    bool have_cmd, have_stat;
    uint8_t last_type, last_stat_kind;
    uint32_t last_cmd_end, last_stat_end, last_cmd_tick;
    uint16_t st_boot, st_piggy, st_quiet, st_unpaced, cmd_behind_piggy, cmd_behind_other;
    uint32_t gap_min, gap_piggy_max, gap_quiet_max;   /* com_ticks between consecutive STATs */
    uint32_t boot_ticks;                     /* R3: com_ticks of the first STAT (0xFFFFFFFF: none yet) */
} s_f;

/* ---- measurement windows (one second) ---- */
static uint32_t s_next_report;
static uint16_t s_stack_free = 0xFFFFu;
static uint16_t s_sec_max[COM_SB_SECTIONS];
static uint16_t s_w_pm, s_w_np;
static uint32_t s_w_cl, s_w_sl, s_w_el;          /* latency maxima, us */
static uint32_t s_w_sum;
static bool s_w_burst;

void com_sb_init(uint8_t reset_cause)
{
    uint16_t p = 0;
    s_reset_cause = reset_cause;
    UBRR0H = (uint8_t)(SB_CON_UBRR >> 8);
    UBRR0L = (uint8_t)(SB_CON_UBRR & 0xFF);
    UCSR0A = 0;
    UCSR0C = (uint8_t)((1 << UCSZ01) | (1 << UCSZ00));     /* 8N1 */
    UCSR0B = (uint8_t)(1 << TXEN0);
    il_decoder_init(&s_f.dec, IL_ACCEPT_FCU);              /* what a real FCU receives: CMD and STAT */
    s_f.nonce = onb_lfsr16_next(0x2961u);                  /* the FCU's nonce, unknown to the ground until TELEM */
    s_f.state = ONB_SAFE;
    s_f.gap_min = 0xFFFFFFFFUL;
    s_f.boot_ticks = 0xFFFFFFFFUL;
    s_g.next_poll = SB_FIRST_POLL_TICKS;
    s_next_report = COM_TICK_HZ;
    s_t1 = s_raw_t1 = com_timer_tcnt1();
    s_raw = s_raw_t1;                                      /* Timer1 counts since com_timer_init */
    s_line[p++] = '\n';
    p = put_s(p, "SB BOOT radio=" SB_RADIO_NAME " ");
    p = put_u(p, "rc=", reset_cause);
    p = put_u(p, "stack=", stack_reserve_total());
    p = put_u(p, "tag=", FC_PARAMS_TAG);
    p = put_u(p, "poll=", SB_POLL_TICKS);
    p = put_u(p, "rbyte=", SB_RADIO_BYTE);
    p = put_u(p, "ilbyte=", SB_IL_BYTE);
    p = put_u(p, "win=", COM_REPLY_WINDOW_TICKS);
    p = put_u(p, "quiet=", COM_STAT_QUIET_TICKS);
    p = put_u(p, "div=", ILINK_STAT_DIV);
    p = put_u(p, "hz=", COM_TICK_HZ);
    p = put_u(p, "secs=", COM_SB_SECONDS);
    p = put_u(p, "boot=", COM_STAT_BOOT_TICKS);
    p = put_u(p, "wrong=", SB_WRONG_S);
    s_line[p++] = '\n';
    con_write(s_line, p);
}

/* -- FCU: the supervisor model for one IL_CMD (core/onboard.c rules for SEQ, NONE, ARM, LAUNCH) -- */
static void fcu_exec(const il_cmd_t *c, uint32_t tick)
{
    uint8_t code = 0;
    if (s_f.has_seq && !lp_seq_newer(c->seq, s_f.last_seq))
        return;                                            /* repeat: acknowledged only */
    s_f.has_seq = true;
    s_f.last_seq = c->seq;
    switch (c->cmd.cmd) {
    case LP_CMD_NONE:
        s_f.rejected = false;
        return;
#if FC_VTOL
    case LP_CMD_ON:                                        /* VTOL: the nonce, then the spool warning */
        if (s_f.state != ONB_SAFE) {
            code = LP_REJ_STATE;
        } else if (c->cmd.nonce != s_f.nonce) {
            code = LP_REJ_NONCE;
            s_f.nonce_rej++;
        } else {
            s_f.nonce = onb_lfsr16_next(s_f.nonce);
            s_f.state = ONB_ARMED;
            s_f.phase = LP_PHASE_SPOOL;
            s_f.t_launch = tick;
        }
        break;
#else
    case LP_CMD_ARM:
        if (s_f.state != ONB_SAFE)
            code = LP_REJ_STATE;
        else
            s_f.state = ONB_ARMED;
        break;
    case LP_CMD_LAUNCH:
        if (s_f.state != ONB_ARMED) {
            code = LP_REJ_STATE;
        } else if (c->cmd.nonce != s_f.nonce) {
            code = LP_REJ_NONCE;
            s_f.nonce_rej++;
        } else {
            s_f.nonce = onb_lfsr16_next(s_f.nonce);
            s_f.state = ONB_POWERED;
            s_f.t_launch = tick;
        }
        break;
#endif
    default:
        code = LP_REJ_UNKNOWN_CMD;                         /* the synthetic ground sends nothing else */
        break;
    }
    s_f.rejected = code != 0u;
    if (code)
        s_f.reason = code;
}

static void q_from_euler_deg(float r, float p, float y, float q[4])
{
    const float hr = 0.5f * r * FC_DEG2RAD_F, hp = 0.5f * p * FC_DEG2RAD_F, hy = 0.5f * y * FC_DEG2RAD_F;
    const float cr = cosf(hr), sr = sinf(hr), cp = cosf(hp), sp = sinf(hp), cy = cosf(hy), sy = sinf(hy);
    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

/* -- FCU: the snapshot payload of the model's current outputs, answering reply_seq (VTOL: SNAP3, every 4th a SNAP2;
 *    v0.9 RP1: the power build cycles battery / wired / wired_buffer per snapshot); returns the payload length and, in
 *    *tlm_type, the TELEM TYPE the COM must answer it with -- */
static uint8_t fcu_snap_payload(uint8_t reply_seq, uint32_t tick, uint8_t pay[SB_SNAP_LEN], uint8_t *tlm_type)
{
#if FC_VTOL
    /* v0.9: one buffer for both layouts - the SNAP fields are the common first member (il_snap_t s); no copies on the
     * bench stack (the bench build fills the 4 KiB: the COM bench's stack headroom is ~100 B) */
    union {
        il_snap3_t s3;
        il_snap2_t s2;
    } u;
#define s (u.s2.s)                                  /* the common first member (both views alias it) */
#define s3 (u.s3)
#else
    il_snap_t s;
#endif
    const float kf = (float)s_f.snaps;
    s.reply_seq = reply_seq;
    s.state = s_f.state;
    s.flags = (uint8_t)(SB_FLAGS_PAD | (s_f.state == ONB_POWERED ? LP_FLAG_CONTROL_ACTIVE : 0u) |
                        (s_f.rejected ? LP_FLAG_REJECTED : 0u));
    s.reason = s_f.reason;
    s.ack_seq = s_f.has_seq ? s_f.last_seq : 0u;
    s.nonce = s_f.nonce;
    s.throttle_milli = s_f.thr;
    s.tick = tick;
    if (s_f.state == ONB_POWERED) {
        const float t = (float)(tick - s_f.t_launch) * FC_DT_S;
#if FC_VTOL                                                /* CLIMB at 1 m/s, then HOLD at the hover target */
        const float h_tgt = (float)ONB_V_HOVER_DM * 0.1f;
        s.h_m = t < h_tgt ? t : h_tgt;
        s.vz_mps = t < h_tgt ? 1.0f : 0.0f;
#else
        s.h_m = 1.25f * t * t;
        s.vz_mps = 2.5f * t;
#endif
        q_from_euler_deg(12.0f * sinf(0.9f * kf), 9.0f * cosf(0.6f * kf), fmodf(37.0f * kf, 360.0f) - 179.5f, s.q);
    } else {
        s.h_m = 0.02f;
        s.vz_mps = -0.01f;
        q_from_euler_deg(1.5f * sinf(1.1f * kf), -2.0f * cosf(0.7f * kf), 30.0f + 5.0f * sinf(0.3f * kf), s.q);
    }
    s.tilt_deg = 2.5f + 0.25f * sinf(kf);
    s.v_cell = 4.05f - 0.001f * kf;
    s.loop_max_us = 2950.5f;
#if FC_VTOL
    s3.phase = s_f.phase;
    s3.h_tgt_m = (s_f.phase == LP_PHASE_SPOOL || s_f.phase == LP_PHASE_CLIMB || s_f.phase == LP_PHASE_HOLD)
                     ? (float)ONB_V_HOVER_DM * 0.1f : 0.0f;
    /* v0.8 GPS fields: a 3D fix that gets its ON gate, the hold engaged in flight (timing / encoding inputs) */
    s3.gps_flags = 0x7Bu;              /* core/gps_nav.h FIX_OK GATE_OK FIX3D CFG_OK INIT VER_OK (bit 7 INCONS never) */
    s3.num_sv = (uint8_t)(7u + (s_f.snaps & 3u));
    s3.hacc_dm = (uint8_t)(15u + (s_f.snaps & 7u));
    s3.pos_mode = s_f.state == ONB_POWERED ? 2u : 0u;                     /* POS_MODE_POS in flight, OFF on the pad */
    s3.dist_m = s_f.state == ONB_POWERED ? 0.35f + 0.05f * sinf(kf) : NAN;  /* NaN: no hold point (TELEM3 0xFFFF) */
    {
        /* v0.9 RP1: the power build of this snapshot (0 battery: the FCU sends 0, 0; 1 wired: TETHER_OK, SAG on every
         * 5th; 2 wired_buffer: TETHER_OK + BUF_SHARE, BUF_LOW on every 5th) and v_aux (19.75 .. 23.625 V) */
        const uint8_t build = (uint8_t)(s_f.snaps % 3u), k5 = (uint8_t)(s_f.snaps % 5u == 4u);
        const bool v2_only = (s_f.snaps & 3u) == 3u;      /* every 4th: an ONB_NO_GPS FCU's SNAP2 */
        /* 2026-11-10: the logic-battery byte (SNAP3 spare3 / SNAP2 spare, 3.00 .. 4.26 V): every power build's snapshot
         * carries one (the power tail must win: TELEM v5 / v4); the battery build's SNAP2 / SNAP3 alternate between none
         * (TELEM v2 / v3) and one (TELEM v6 / v7) - SNAP2 one in three without (only ~3 per run, the 2nd of them never
         * becomes TELEM: a D5 fault) - so each of the six VTOL replies is built */
        static uint8_t s_lb_n2, s_lb_n3;
        const uint8_t lbv = (uint8_t)(150u + (s_f.snaps & 63u));
        const uint8_t lb = build ? lbv : (v2_only ? ((s_lb_n2++ % 3u) ? lbv : 0u) : ((s_lb_n3++ & 1u) ? lbv : 0u));
        s3.pw_vaux = build ? (uint8_t)(158u + (s_f.snaps & 31u)) : 0u;
        s3.pw_flags = build == 0u ? 0u : (uint8_t)((build << LP_PWR_TAIL_BUILD_SHIFT) | 0x01u |
                                                   (build == 1u ? (k5 ? 0x02u : 0u) : (0x08u | (k5 ? 0x04u : 0u))));
        s3.spare3 = lb;
        s_f.snaps++;
        if (v2_only) {                                 /* the same SNAP fields, the SNAP2 tail in place */
            const uint8_t ph = s3.phase, va = s3.pw_vaux, pf = s3.pw_flags;
            const float ht = s3.h_tgt_m;
            u.s2.phase = ph;
            u.s2.spare = lb;
            u.s2.h_tgt_m = ht;
            u.s2.pw_vaux = va;
            u.s2.pw_flags = pf;
            il_pack_snap2(&u.s2, pay);
            *tlm_type = build ? LP_TYPE_TELEM4 : (lb ? LP_TYPE_TELEM6 : LP_TYPE_TELEM2);
            return IL_SNAP2_LEN;
        }
        il_pack_snap3(&s3, pay);
        *tlm_type = build ? LP_TYPE_TELEM5 : (lb ? LP_TYPE_TELEM7 : LP_TYPE_TELEM3);
        return IL_SNAP3_LEN;
    }
#undef s
#undef s3
#else
    il_pack_snap(&s, pay);
    s_f.snaps++;
    *tlm_type = LP_TYPE_TELEM;
    return IL_SNAP_LEN;
#endif
}

/* -- FCU: one frame (TYPE, payload) into its TX queue; bad: no TELEM to time; false if the queue is full -- */
static bool fcu_enqueue_frame(uint8_t type, const uint8_t *pay, uint8_t n, uint8_t seq, bool bad)
{
    uint8_t slot;
    if (s_f.tx_count >= 2u) {
        s_f.tx_drop++;                                     /* the FCU's TX ring would be full */
        return false;
    }
    slot = (uint8_t)((s_f.tx_head + s_f.tx_count) & 1u);
    s_f.tx_len[slot] = il_encode(type, pay, n, s_f.tx[slot]);
    s_f.tx_seq[slot] = seq;
    s_f.tx_bad[slot] = bad;
    if (s_f.tx_count++ == 0u) {
        s_f.tx_pos = 0;
        s_f.tx_next_t = s_now + SB_IL_BYTE;                /* the first byte has arrived one byte time from now */
    }
    return true;
}

/* -- FCU: one SNAP (VTOL: SNAP3 / SNAP2 by its length) frame into its TX queue (bad: sent on purpose to be refused, never
 *    a TELEM) -- */
static void fcu_enqueue(const uint8_t pay[SB_SNAP_LEN], uint8_t n, uint8_t seq, bool bad)
{
#if FC_VTOL
    const uint8_t type = n == IL_SNAP2_LEN ? IL_TYPE_SNAP2 : SB_SNAP_TYPE;
#else
    const uint8_t type = SB_SNAP_TYPE;
#endif
    if (fcu_enqueue_frame(type, pay, n, seq, bad) && bad)
        s_f.bad_sent++;
}

/* -- FCU, R4: an IL_CMD and an IL_STAT (valid frames of the COM -> FCU types) towards the COM, back to back; the COM's
 *    decoder (IL_ACCEPT_COM) must reject both in the header and stay in step for the next SNAP.  Their bytes go to the
 *    console ("SBW f=<hex>", one line per frame) for the host's check against sim/ilink_proto.py -- */
static void fcu_wrong_direction(void)
{
    il_cmd_t c;
    il_stat_t st;
    uint8_t pay[IL_STAT_LEN], k;
    uint16_t q;
    c.seq = 0x5Au;
    c.cmd.cmd = LP_CMD_ARM;
    c.cmd.arg = 0x0102u;
    c.cmd.nonce = 0x3E4Fu;
    c.rssi_dbm = -61;
    c.snr_q4 = 23;
    il_pack_cmd(&c, pay);
    (void)fcu_enqueue_frame(IL_TYPE_CMD, pay, IL_CMD_LEN, 0u, true);
    memset(&st, 0, sizeof st);                             /* a plausible STAT, fixed bytes in every run */
    st.com_ticks = (uint32_t)SB_WRONG_S * COM_TICK_HZ;
    st.flags = (uint8_t)(IL_STAT_RADIO_OK | SB_BACKEND_FLAG);
    st.params_tag = FC_PARAMS_TAG;
    st.rssi_dbm = RADIO_RSSI_NONE;
    st.dec_frames = st.cmd_forwarded = st.replies = 0x0101u;
    st.loop_us_max = 0x0123u;
    il_pack_stat(&st, pay);
    (void)fcu_enqueue_frame(IL_TYPE_STAT, pay, IL_STAT_LEN, 0u, true);
    s_f.wrong_done = true;
    for (k = 0; k < 2u; k++) {
        const uint8_t slot = (uint8_t)((s_f.tx_head + k) & 1u);
        q = put_s(0, "\nSBW ");
        q = put_hex(q, "f=", s_f.tx[slot], s_f.tx_len[slot]);
        s_line[q++] = '\n';
        con_write(s_line, q);
        s_f.wrong_sent++;
    }
}

/* -- FCU tick: the IL_CMDs received before this tick get ONE SNAP (reply_seq = the last one's SEQ); the three
 *    deliberate D5 faults: a stray SNAP, a reply sent twice, a reply held past the reply window; the R4 frames -- */
static void fcu_tick(uint32_t tick, uint32_t dticks)
{
    uint8_t i, seq, pay[SB_SNAP_LEN], n, tt;
#if FC_VTOL                                                /* the spool warning, then the climb; HOLD at the target */
    if (s_f.state == ONB_ARMED && tick - s_f.t_launch >= ONB_V_SPOOL_TICKS) {
        s_f.state = ONB_POWERED;
        s_f.phase = LP_PHASE_CLIMB;
        s_f.t_launch = tick;
    }
    if (s_f.phase == LP_PHASE_CLIMB && (float)(tick - s_f.t_launch) * FC_DT_S >= (float)ONB_V_HOVER_DM * 0.1f)
        s_f.phase = LP_PHASE_HOLD;
    if (s_f.state == ONB_POWERED)                          /* the throttle the controller holds: ~hover */
        s_f.thr = 915u;
    (void)dticks;
#else
    if (s_f.state == ONB_POWERED) {                        /* throttle ramp 0 -> 1000 in 0.5 s */
        const uint32_t up = s_f.thr + 8u * dticks;
        s_f.thr = (uint16_t)(up > 1000u ? 1000u : up);
    }
#endif
    if (s_f.late_pending && tick >= s_f.late_tick) {
        s_f.late_pending = false;
        fcu_enqueue(s_f.late_pay, s_f.late_len, s_f.late_seq, true);
    }
    if (s_f.nq == 0u) {
        if (!s_f.stray_done && s_f.has_seq && s_f.tx_count == 0u && tick >= (uint32_t)SB_STRAY_S * COM_TICK_HZ) {
            s_f.stray_done = true;                         /* a reply_seq the COM never forwarded */
            seq = (uint8_t)(s_f.last_seq + 0x80u);
            n = fcu_snap_payload(seq, tick, pay, &tt);
            fcu_enqueue(pay, n, seq, true);
        } else if (!s_f.wrong_done && s_f.tx_count == 0u && tick >= (uint32_t)SB_WRONG_S * COM_TICK_HZ) {
            fcu_wrong_direction();                         /* both frames take 2.24 ms: done before the next tick's SNAP */
        }
        return;
    }
    for (i = 0; i < s_f.nq; i++)
        fcu_exec(&s_f.q[i], tick);
    seq = s_f.q[s_f.nq - 1u].seq;
    s_f.nq = 0;
    n = fcu_snap_payload(seq, tick, pay, &tt);
    if (!s_f.late_done && tick >= (uint32_t)SB_LATE_S * COM_TICK_HZ) {
        s_f.late_done = s_f.late_pending = true;          /* held until the COM's window is over */
        s_f.late_tick = tick + COM_REPLY_WINDOW_TICKS + 3u;
        s_f.late_seq = seq;
        s_f.late_len = n;
        for (i = 0; i < n; i++)
            s_f.late_pay[i] = pay[i];
        return;
    }
    for (i = 0; i < n; i++)
        s_f.snap_pay[seq & 3u][i] = pay[i];
    s_f.snap_pay_seq[seq & 3u] = seq;
    s_f.snap_len[seq & 3u] = n;
    s_f.snap_tlm_type[seq & 3u] = tt;
    fcu_enqueue(pay, n, seq, false);
    if (!s_f.dup_done && tick >= (uint32_t)SB_DUP_S * COM_TICK_HZ) {
        s_f.dup_done = true;                               /* the same reply once more, back to back */
        fcu_enqueue(pay, n, seq, true);
    }
}

/* -- FCU: SNAP bytes that have arrived by now go into the COM's ilink receive ring -- */
static void fcu_tx(void)
{
    while (s_f.tx_count > 0u) {
        uint8_t k = 0;
        const uint8_t *fr = s_f.tx[s_f.tx_head];
        const uint8_t len = s_f.tx_len[s_f.tx_head];
        while ((uint8_t)(s_f.tx_pos + k) < len && due(s_f.tx_next_t)) {
            k++;
            s_f.tx_next_t += SB_IL_BYTE;
        }
        if (k) {
            ilink_uart_inject_rx(fr + s_f.tx_pos, k);
            s_f.tx_pos = (uint8_t)(s_f.tx_pos + k);
        }
        if (s_f.tx_pos < len)
            return;
        if (!s_f.tx_bad[s_f.tx_head]) {                    /* a refused SNAP / wrong-direction frame has no TELEM to time */
            const uint8_t j = (uint8_t)(s_f.tx_seq[s_f.tx_head] & 7u);
            s_f.snap_done_t[j] = s_f.tx_next_t - SB_IL_BYTE;
            s_f.sl_valid |= (uint8_t)(1u << j);
        }
        s_f.tx_head ^= 1u;
        s_f.tx_count--;
        s_f.tx_pos = 0;                                    /* the next frame follows back to back */
    }
}

/* -- 2026-11-10 (review F4): the RADIO_OK bit a STAT must carry.  lora_uart / sx127x: set in every STAT (unchanged).
 *    esp_bridge: radio_stats()->ok is "a byte from the ESP since this COM boot" (hal/esp_bridge.h) and the bench's only
 *    ESP bytes are the injected uplink frames: clear in every STAT built (st.com_ticks) before the tick of the first
 *    injected byte, set in every STAT built after it; a STAT built in that very tick may carry either -- */
static bool sb_radio_ok_expected(const il_stat_t *st)
{
    const bool set = (st->flags & IL_STAT_RADIO_OK) != 0u;
#if FC_LINK_RADIO == FC_RADIO_ESP_BRIDGE
    if (!s_g.rx_seen || st->com_ticks < s_g.first_rx_tick)
        return !set;
    if (st->com_ticks > s_g.first_rx_tick)
        return set;
    return true;
#else
    return set;
#endif
}

/* -- FCU: one STAT, its pacing (D4) and its content -- */
static void fcu_stat(const il_frame_t *fr, uint32_t start)
{
    il_stat_t st;
    bool ok;
    uint8_t kind;
    il_unpack_stat(fr->payload, &st);
    ok = st.params_tag == FC_PARAMS_TAG && (st.flags & IL_STAT_SX127X) == SB_BACKEND_FLAG &&
         sb_radio_ok_expected(&st) && ((st.flags & IL_STAT_BOOT) != 0u) == (s_f.stats == 0u) &&
         st.reset_cause == s_reset_cause && st.cmd_forwarded == s_f.cmds;   /* every IL_CMD before it has arrived */
    if (st.flags & IL_STAT_RADIO_OK)
        s_f.st_radio_ok++;
    else
        s_f.st_radio_nok++;
    if (s_f.stats == 0u) {
        kind = COM_STAT_KIND_BOOT;
        s_f.st_boot++;
        s_f.boot_ticks = st.com_ticks;                     /* R3: >= COM_STAT_BOOT_TICKS (checked by the host) */
    } else if (s_f.have_cmd && s_f.last_type == IL_TYPE_CMD && start - s_f.last_cmd_end <= SB_TICK_COUNTS) {
        kind = COM_STAT_KIND_PIGGYBACK;                    /* right behind an IL_CMD */
        s_f.st_piggy++;
    } else if (!s_f.have_cmd || st.com_ticks - s_f.last_cmd_tick >= COM_STAT_QUIET_TICKS - 1u) {
        kind = COM_STAT_KIND_QUIET;                        /* no IL_CMD for 1.5 / stat_hz */
        s_f.st_quiet++;
    } else {
        kind = 0xFFu;
        s_f.st_unpaced++;                                  /* D4 violated: a STAT the next IL_CMD may sit behind */
    }
    if (s_f.stats > 0u) {
        const uint32_t d = st.com_ticks - s_f.last_stat.com_ticks;
        ok = ok && d >= ILINK_STAT_DIV && st.loop_us_max > 0u;
        if (d < s_f.gap_min)
            s_f.gap_min = d;
        if (kind == COM_STAT_KIND_PIGGYBACK && d > s_f.gap_piggy_max)
            s_f.gap_piggy_max = d;
        if (kind == COM_STAT_KIND_QUIET && d > s_f.gap_quiet_max)
            s_f.gap_quiet_max = d;
    }
    if (!ok)
        s_f.stat_bad++;
    s_f.stats++;
    s_f.last_stat = st;
    s_f.have_stat = true;
    s_f.last_stat_kind = kind;
}

/* -- FCU: one frame from the COM, complete at time end (its first byte started frame-length byte times earlier) -- */
static void fcu_frame(const il_frame_t *fr, uint32_t end)
{
    const uint32_t start = end - (uint32_t)(fr->len + IL_FRAME_OVERHEAD) * SB_IL_BYTE;
    if (fr->type == IL_TYPE_CMD) {
        il_cmd_t c;
        const lp_cmd_t *g;
        il_unpack_cmd(fr->payload, &c);
        g = &s_g.sent[c.seq & 7u];
        s_f.cmds++;
        if (c.cmd.cmd != g->cmd || c.cmd.arg != g->arg || c.cmd.nonce != g->nonce || c.rssi_dbm != RADIO_RSSI_NONE)
            s_f.cmd_bad++;                                 /* no module in simavr: the radio reports no RSSI */
        if (s_f.nq < ONB_MAX_FRAMES)
            s_f.q[s_f.nq++] = c;
        else
            s_f.cmd_bad++;
        if (s_f.have_stat && start - s_f.last_stat_end < SB_TICK_COUNTS) {   /* D4: an IL_CMD right behind a STAT */
            if (s_f.last_stat_kind == COM_STAT_KIND_PIGGYBACK)
                s_f.cmd_behind_piggy++;
            else
                s_f.cmd_behind_other++;
        }
        s_f.have_cmd = true;
        s_f.last_cmd_end = end;
        s_f.last_cmd_tick = s_tick;
    } else if (fr->type == IL_TYPE_STAT) {
        fcu_stat(fr, start);
        s_f.last_stat_end = end;
    } else {
        s_f.wrong_frames++;                                /* defensive: IL_ACCEPT_FCU returns CMD / STAT only */
    }
    s_f.last_type = fr->type;
}

/* -- FCU: COM TX bytes that have been shifted out by now -- */
static void fcu_rx(void)
{
    uint8_t b;
    while (due(s_f.take_t)) {
        const il_frame_t *fr;
        const uint32_t end = s_f.take_t;
        if (ilink_uart_take_tx(&b, 1) == 0u) {
            s_f.take_t = s_now + SB_IL_BYTE;               /* idle line */
            return;
        }
        s_f.take_t += SB_IL_BYTE;
        fr = il_decoder_push(&s_f.dec, b);
        if (fr)
            fcu_frame(fr, end);
    }
}

/* -- ground: the receiver reads the TELEM the COM handed to the radio (SPEC 10.3 offsets) and prints it -- */
static void ground_print(const uint8_t *f, uint8_t seq)
{
    uint16_t q = 0;
    const uint8_t k = (uint8_t)(seq & 3u);
    if (s_f.snap_pay_seq[k] != seq) {
        s_g.tlm_bad++;                                     /* no SNAP with this SEQ was sent */
        return;
    }
    q = put_s(q, "\nSBT ");
    q = put_u(q, "s=", seq);
    q = put_hex(q, "n=", s_f.snap_pay[k], s_f.snap_len[k]);
    q = put_hex(q, "f=", f, s_g.rx_n);
    s_line[q++] = '\n';
    con_write(s_line, q);
}

/* ---- once per second: three report lines (simavr splits console lines longer than 255 characters) ---- */
static void report(uint32_t tick)
{
    const radio_stats_t *rs = radio_stats();
    const ilink_uart_stats_t *is = ilink_uart_stats();
    const com_link_stats_t *cs = com_link_stats();
    uint16_t p = 0;
    uint8_t i;
    s_line[p++] = '\n';
    p = put_s(p, "SB ");                                   /* timing */
    p = put_u(p, "t=", tick);
    p = put_u(p, "pm=", s_w_pm);                           /* longest main-loop pass, us */
    p = put_u(p, "pa=", s_w_np ? s_w_sum / s_w_np : 0u);   /* mean pass, us */
    p = put_u(p, "np=", s_w_np);
    p = put_u(p, "sf=", s_stack_free);                     /* never-touched stack, last complete scan */
    p = put_u(p, "cl=", s_w_cl);                           /* uplink frame complete at the COM -> IL_CMD queued, us */
    p = put_u(p, "sl=", s_w_sl);                           /* SNAP complete at the COM -> TELEM handed to the radio, us */
    p = put_u(p, "el=", s_w_el);                           /* uplink complete -> TELEM handed to the radio, us */
    p = put_u(p, "bu=", s_w_burst);
    p = put_u(p, "s=", (uint32_t)s_sec_max[1] / COM_T1_COUNTS_PER_US);
    for (i = 2; i < COM_SB_SECTIONS; i++)
        p = put_u(p, "", (uint32_t)s_sec_max[i] / COM_T1_COUNTS_PER_US);
    s_line[p++] = '\n';
    con_write(s_line, p);
    p = 0;
    s_line[p++] = '\n';
    p = put_s(p, "SL ");                                   /* link counters and checks */
    p = put_u(p, "t=", tick);
    p = put_u(p, "up=", s_g.uplinks);                      /* uplink frames the ground started */
    p = put_u(p, "fw=", s_f.cmds);                         /* IL_CMD frames the FCU received */
    p = put_u(p, "sn=", s_f.snaps);                        /* SNAPs the FCU produced (incl. the deliberate stray one) */
    p = put_u(p, "rp=", s_g.replies);                      /* TELEM frames the COM handed to the radio */
    p = put_u(p, "gs=", s_g.tlm_state);
    p = put_u(p, "ga=", (uint32_t)(s_g.have_tlm && s_g.tlm_ack == s_g.gseq));
    p = put_u(p, "st=", s_f.stats);
    p = put_u(p, "sx=", s_f.stat_bad);
    p = put_u(p, "rk=", s_f.st_radio_ok);                  /* 2026-11-10 (F4): STATs with RADIO_OK */
    p = put_u(p, "rz=", s_f.st_radio_nok);                 /* ... and without (esp_bridge: before the first ESP byte) */
    p = put_u(p, "cb=", s_f.cmd_bad);
    p = put_u(p, "nr=", s_f.nonce_rej);
    p = put_u(p, "tb=", s_g.tlm_bad);
    p = put_u(p, "bl=", s_g.burst_reply);
    p = put_u(p, "wf=", s_f.wrong_frames);
    p = put_u(p, "fd=", s_f.tx_drop);
    p = put_u(p, "xr=", s_f.last_stat.replies);
    p = put_u(p, "xd=", s_f.last_stat.replies_dropped);
    p = put_u(p, "xl=", s_f.last_stat.loop_us_max);
    p = put_u(p, "xb=", s_f.last_stat.il_bad);
    p = put_u(p, "xt=", s_f.last_stat.il_tx_dropped);
    p = put_u(p, "xo=", (uint32_t)is->rx_overflow + is->rx_errors);
    p = put_u(p, "ro=", (uint32_t)rs->rx_overflow + rs->rx_errors);
    p = put_u(p, "rt=", rs->tx_timeouts);
    p = put_u(p, "tf=", rs->tx_frames);                    /* frames the radio driver started sending */
    p = put_u(p, "td=", s_con_dropped);
    s_line[p++] = '\n';
    con_write(s_line, p);
    p = 0;
    s_line[p++] = '\n';
    p = put_s(p, "SD ");                                   /* D4 STAT pacing, D5 reply gate */
    p = put_u(p, "t=", tick);
    p = put_u(p, "rf=", cs->replies_refused);              /* COM: SNAPs refused by the reply gate */
    p = put_u(p, "re=", cs->replies_expired);              /* COM: TELEM dropped, window over before the radio took it */
    p = put_u(p, "rs=", cs->replies_superseded);           /* COM: replaced by a newer reply */
    p = put_u(p, "sa=", cs->snaps_accepted);
    p = put_u(p, "bs=", s_f.bad_sent);                     /* FCU: SNAPs sent on purpose to be refused */
    p = put_u(p, "gr=", s_g.refused_seen);                 /* refusals the hooks attributed to an uplink record */
    p = put_u(p, "ow=", s_g.out_of_window);                /* ground: TELEM >= window after its IL_CMD */
    p = put_u(p, "um=", s_g.unmatched);                    /* ground: TELEM with no uplink waiting for it */
    p = put_u(p, "ag=", counts_to_us(s_g.age_max));        /* ground: longest IL_CMD queued -> TELEM, us of real time */
    p = put_u(p, "sp=", cs->stats_piggyback);              /* COM: STAT kinds */
    p = put_u(p, "sq=", cs->stats_quiet);
    p = put_u(p, "ss=", cs->stats_sent);
    p = put_u(p, "fb=", s_f.st_boot);                      /* FCU: the same STATs classified on the wire */
    p = put_u(p, "fp=", s_f.st_piggy);
    p = put_u(p, "fq=", s_f.st_quiet);
    p = put_u(p, "fu=", s_f.st_unpaced);
    p = put_u(p, "cp=", s_f.cmd_behind_piggy);             /* FCU: IL_CMD starting < 1 tick after a piggyback STAT */
    p = put_u(p, "co=", s_f.cmd_behind_other);             /* ... after a boot / quiet STAT */
    p = put_u(p, "gm=", s_f.gap_min == 0xFFFFFFFFUL ? 0u : s_f.gap_min);   /* STAT gaps, COM ticks */
    p = put_u(p, "gp=", s_f.gap_piggy_max);
    p = put_u(p, "gq=", s_f.gap_quiet_max);
    p = put_u(p, "be=", s_g.burst_expired);                /* the newest burst reply expired (radio busy) */
    p = put_u(p, "xe=", s_g.expired_seen);                 /* expiry events seen by the hooks */
    p = put_u(p, "ev=", s_ev_lost);
    s_line[p++] = '\n';
    con_write(s_line, p);
    p = 0;
    s_line[p++] = '\n';
    p = put_s(p, "SI ");                                   /* R4 ilink decoders (cumulative), R3 boot STAT */
    p = put_u(p, "t=", tick);
    p = put_u(p, "cf=", cs->il_frames);                    /* COM decoder (IL_ACCEPT_COM): frames, rejects, skipped */
    p = put_u(p, "cc=", cs->il_bad_crc);
    p = put_u(p, "cl=", cs->il_bad_len);
    p = put_u(p, "ct=", cs->il_bad_type);
    p = put_u(p, "ck=", cs->il_skipped);
    p = put_u(p, "cw=", cs->il_wrong_type);
    p = put_u(p, "ff=", s_f.dec.stats.frames);             /* synthetic FCU decoder (IL_ACCEPT_FCU) */
    p = put_u(p, "fc=", s_f.dec.stats.bad_crc);
    p = put_u(p, "fl=", s_f.dec.stats.bad_len);
    p = put_u(p, "ft=", s_f.dec.stats.bad_type);
    p = put_u(p, "fk=", s_f.dec.stats.skipped_bytes);
    p = put_u(p, "ws=", s_f.wrong_sent);                   /* wrong-direction frames the synthetic FCU sent */
    p = put_u(p, "bt=", s_boot_raw == 0xFFFFFFFFUL ? 0xFFFFFFFFUL : counts_to_us(s_boot_raw));   /* us after com_timer_init */
    p = put_u(p, "bk=", s_f.boot_ticks);                   /* com_ticks of the first STAT at the FCU */
    s_line[p++] = '\n';
    con_write(s_line, p);
    s_w_pm = s_w_np = 0;
    s_w_cl = s_w_sl = s_w_el = 0;
    s_w_sum = 0;
    s_w_burst = false;
    for (i = 0; i < COM_SB_SECTIONS; i++)
        s_sec_max[i] = 0;
}

#else /* FC_COSIM */
/* ======================================================================================== */
/* co-simulation: the report struct (com_cosim.h); no console, no synthetic FCU             */
/* ======================================================================================== */
void com_sb_init(uint8_t reset_cause)
{
    uint8_t i;
    s_reset_cause = reset_cause;
    REP_BEGIN();
    g_cosim_report.magic = COM_COSIM_MAGIC;
    g_cosim_report.version = COM_COSIM_VERSION;
    g_cosim_report.size = (uint16_t)sizeof(com_cosim_report_t);
    g_cosim_report.radio = FC_LINK_RADIO;
    g_cosim_report.reset_cause = reset_cause;
    g_cosim_report.reply_window_ticks = COM_REPLY_WINDOW_TICKS;
    g_cosim_report.tick_hz = COM_TICK_HZ;
    g_cosim_report.ground_state = 0xFFu;
    g_cosim_report.stack_free_min = 0xFFFFu;
    for (i = 0; i < COM_COSIM_SECONDS; i++)
        g_cosim_report.pass_max_s_us[i] = 0;
    REP_END();
    s_g.next_poll = SB_FIRST_POLL_TICKS;
    s_t1 = s_raw_t1 = com_timer_tcnt1();
    s_raw = s_raw_t1;                                      /* Timer1 counts since com_timer_init */
}

/* every counter of the report (once per COM tick and at the end; outside the measured pass) */
static void cosim_counters(uint32_t tick)
{
    const com_link_stats_t *cs = com_link_stats();
    const ilink_uart_stats_t *is = ilink_uart_stats();
    const radio_stats_t *rs = radio_stats();
    volatile com_cosim_counters_t *c = &g_cosim_report.c;
    REP_BEGIN();
    g_cosim_report.t_us = counts_to_us(now_counts());
    g_cosim_report.ticks = tick;
    c->uplinks = s_g.uplinks;
    c->cmd_forwarded = cs->cmd_forwarded;
    c->il_tx_dropped = cs->il_tx_dropped;
    c->il_wrong_type = cs->il_wrong_type;
    c->snaps_accepted = cs->snaps_accepted;
    c->replies = cs->replies;
    c->replies_superseded = cs->replies_superseded;
    c->replies_refused = cs->replies_refused;
    c->replies_expired = cs->replies_expired;
    c->stats_sent = cs->stats_sent;
    c->stats_piggyback = cs->stats_piggyback;
    c->stats_quiet = cs->stats_quiet;
    c->il_rx_bytes = is->rx_bytes;
    c->il_rx_overflow = is->rx_overflow;
    c->il_rx_errors = is->rx_errors;
    c->il_bad_crc = cs->il_bad_crc;
    c->il_bad_len = cs->il_bad_len;
    c->il_bad_type = cs->il_bad_type;
    c->il_skipped = cs->il_skipped;
    c->radio_rx_overflow = rs->rx_overflow;
    c->radio_rx_errors = rs->rx_errors;
    c->radio_tx_frames = rs->tx_frames;
    c->radio_tx_timeouts = rs->tx_timeouts;
    c->telem_bad = s_g.tlm_bad;
    c->telem_out_of_window = s_g.out_of_window;
    c->telem_unmatched = s_g.unmatched;
    c->nonce_sent = s_g.launches;
    c->events_lost = s_ev_lost;
    g_cosim_report.ground_state = s_g.have_tlm ? s_g.tlm_state : 0xFFu;
    g_cosim_report.ground_acked = (uint8_t)(s_g.have_tlm && s_g.tlm_ack == s_g.gseq);
    g_cosim_report.ul_count = s_ul_count;
    REP_END();
}
#endif /* FC_COSIM */

/* ======================================================================================== */
/* both modes: the synthetic ground and the hook events                                     */
/* ======================================================================================== */
/* -- ground: one CMD frame, bytes from now on at the link byte rate; a new uplink record -- */
static void ground_send(uint8_t cmd)
{
    lp_cmd_t c;
    sb_ul_t *r = &SB_UL(s_ul_count & (SB_UL_RING - 1u));
    c.cmd = cmd;
    c.arg = 0;
    c.nonce = s_g.tlm_nonce;                               /* the nonce of the latest telemetry */
    s_g.n = lp_encode_cmd(LINK_VEHICLE_ID, s_g.gseq, &c, s_g.txb);
    s_g.pos = 0;
    s_g.next_t = s_now + SB_RADIO_BYTE;
    s_g.sent[s_g.gseq & 7u] = c;
    s_g.uplinks++;
    if (cmd == LP_CMD_LAUNCH || cmd == LP_CMD_ON)
        s_g.launches++;                                    /* the commands that carry the nonce */
    REP_BEGIN();
    r->t_uplink_us = r->t_cmd_queued_us = r->t_snap_us = r->t_telem_us = 0;
    r->seq = s_g.gseq;
    r->cmd = cmd;
    r->flags = 0;
    r->number = (uint8_t)s_ul_count;
    s_ul_count++;
    REP_END();
}

static void ground_tick(uint32_t tick)
{
    if (s_g.pos < s_g.n)
        return;                                            /* a frame is still arriving */
    if (s_g.burst_left > 0u) {
        if (tick < s_g.burst_next)
            return;
        s_g.burst_left--;
        s_g.gseq++;
        ground_send(LP_CMD_NONE);
        s_g.burst_next = tick + SB_BURST_GAP_TICKS;
        return;
    }
#ifdef COM_SB_FIELD
    s_g.burst_done = true;                                 /* the field profile: no stress burst */
#endif
    if (!s_g.burst_done && s_g.pending == LP_CMD_NONE && tick >= (uint32_t)SB_BURST_S * COM_TICK_HZ) {
        s_g.burst_done = s_g.burst_open = true;            /* stress: SB_BURST_N NONE frames back to back, */
        s_g.burst_left = SB_BURST_N - 1u;                  /* starting on the second so that one report window */
        s_g.burst_next = tick + SB_BURST_GAP_TICKS;        /* holds the whole burst */
        s_g.next_poll = tick + SB_BURST_HOLD_TICKS;
#ifndef FC_COSIM
        s_w_burst = true;
#endif
        s_g.gseq++;
        s_g.burst_last = (uint8_t)(s_g.gseq + SB_BURST_N - 1u);
        ground_send(LP_CMD_NONE);
        return;
    }
#ifdef COM_SB_FIELD
    if (tick < s_g.next_poll)
        return;                                            /* the field profile: polls only, no quiet window */
#else
    if (tick < s_g.next_poll || (tick >= SB_QUIET_FROM && tick < SB_QUIET_UNTIL))
        return;                                            /* between polls, or silent (D4 quiet STATs) */
#endif
    s_g.burst_open = false;                                /* a later poll may reuse the burst's last SEQ */
    if (s_g.pending != LP_CMD_NONE && s_g.pending_sent && s_g.have_tlm && s_g.tlm_ack == s_g.gseq)
        s_g.pending = LP_CMD_NONE;                         /* acknowledged */
    s_g.next_poll = tick + SB_POLL_TICKS;
    if (s_g.pending == LP_CMD_NONE && s_g.have_tlm) {
        uint8_t want = LP_CMD_NONE;
#if FC_VTOL && defined(COM_SB_FIELD)                     /* the field profile (above) */
        if (s_g.tlm_state != ONB_SAFE)
            s_field_done |= 1u;                            /* ON was taken */
        if (s_g.tlm_state == ONB_SAFE && !(s_field_done & 1u) && tick >= (uint32_t)SB_ARM_S * COM_TICK_HZ) {
            want = LP_CMD_ON;
        } else if (s_g.tlm_state == ONB_POWERED && !(s_field_done & 2u) && tick >= (uint32_t)COM_SB_DOWN_S * COM_TICK_HZ) {
            want = LP_CMD_DOWN;
            s_field_done |= 2u;
        } else if (s_g.tlm_state == ONB_POWERED && !(s_field_done & 4u) && tick >= (uint32_t)COM_SB_UP_S * COM_TICK_HZ) {
            want = LP_CMD_UP;
            s_field_done |= 4u;
        } else if (s_g.tlm_state == ONB_POWERED && !(s_field_done & 8u) && tick >= (uint32_t)COM_SB_OFF_S * COM_TICK_HZ) {
            want = LP_CMD_OFF;
            s_field_done |= 8u;
        }
#elif FC_VTOL                                              /* VTOL: ON (with the nonce of the latest TELEM2) */
        if (s_g.tlm_state == ONB_SAFE && tick >= (uint32_t)SB_ARM_S * COM_TICK_HZ)
            want = LP_CMD_ON;
#else
        if (s_g.tlm_state == ONB_SAFE && tick >= (uint32_t)SB_ARM_S * COM_TICK_HZ)
            want = LP_CMD_ARM;
        else if (s_g.tlm_state == ONB_ARMED && tick >= (uint32_t)SB_LAUNCH_S * COM_TICK_HZ)
            want = LP_CMD_LAUNCH;
#endif
        if (want != LP_CMD_NONE) {
            s_g.gseq++;
            s_g.pending = want;
            s_g.pending_sent = false;
        }
    }
    ground_send(s_g.pending);
    s_g.pending_sent = true;
}

/* -- ground: the frame's bytes that have arrived by now go into the radio driver -- */
static void ground_bytes(void)
{
    uint8_t k = 0;
    if (s_g.pos >= s_g.n)
        return;
    while ((uint8_t)(s_g.pos + k) < s_g.n && due(s_g.next_t)) {
        k++;
        s_g.next_t += SB_RADIO_BYTE;
    }
    if (k == 0u)
        return;
    if (!s_g.rx_seen) {                                    /* 2026-11-10 (F4): the ESP's first byte (sb_radio_ok_expected) */
        s_g.rx_seen = true;
        s_g.first_rx_tick = com_timer_ticks();
    }
    radio_inject_rx(s_g.txb + s_g.pos, k);
    s_g.pos = (uint8_t)(s_g.pos + k);
    if (s_g.pos >= s_g.n) {                                /* the uplink record: arrival of the last byte */
        sb_ul_t *r = &SB_UL((uint16_t)(s_ul_count - 1u) & (SB_UL_RING - 1u));
        REP_BEGIN();
        r->t_uplink_us = counts_to_us(s_g.next_t - SB_RADIO_BYTE);
        r->flags |= COM_COSIM_UL_DONE;
        REP_END();
    }
}

/* -- ground: the receiver reads the TELEM the COM handed to the radio (SPEC 10.3 offsets) -- */
static void ground_rx(void)
{
    const uint8_t *f = s_g.rx, *p = s_g.rx + 3u + LP_HEADER_BYTES;
    const uint8_t seq = f[5];
    if (!s_g.rx_new)
        return;
    s_g.rx_new = false;
    s_g.replies++;
#if FC_VTOL
    {   /* v0.9 RP1: TELEM v2 / v3 / v4 / v5, each with its own length, and the TYPE the snapshot with this SEQ asked for
         * (co-sim: the real FCU's snapshot is not known here - any VTOL TYPE with its length) */
        const uint8_t ty = f[4];
        const uint8_t want_n = ty == LP_TYPE_TELEM2 ? LP_TELEM2_FRAME_BYTES : ty == LP_TYPE_TELEM3 ? LP_TELEM3_FRAME_BYTES :
                               ty == LP_TYPE_TELEM4 ? LP_TELEM4_FRAME_BYTES : ty == LP_TYPE_TELEM5 ? LP_TELEM5_FRAME_BYTES :
                               ty == LP_TYPE_TELEM6 ? LP_TELEM6_FRAME_BYTES : ty == LP_TYPE_TELEM7 ? LP_TELEM7_FRAME_BYTES : 0u;
        if (want_n == 0u || s_g.rx_n != want_n || f[0] != LP_SYNC0 || f[1] != LP_SYNC1 || f[3] != LINK_VEHICLE_ID
#ifndef FC_COSIM
            || (s_f.snap_pay_seq[seq & 3u] == seq && s_f.snap_tlm_type[seq & 3u] != ty)
#endif
            ) {
            s_g.tlm_bad++;
            return;
        }
    }
#else
    if (s_g.rx_n != SB_TELEM_FRAME_BYTES || f[0] != LP_SYNC0 || f[1] != LP_SYNC1 || f[3] != LINK_VEHICLE_ID ||
        f[4] != SB_TELEM_TYPE) {
        s_g.tlm_bad++;
        return;
    }
#endif
    s_g.have_tlm = true;
    s_g.tlm_state = p[0];
    s_g.tlm_ack = p[3];
    s_g.tlm_nonce = (uint16_t)(p[4] | ((uint16_t)p[5] << 8));
    if (s_g.burst_open && seq == s_g.burst_last)
        s_g.burst_reply = true;                            /* the newest burst reply reached the radio */
#ifndef FC_COSIM
    ground_print(f, seq);                                  /* the SNAP that produced it and the frame, for the host */
#endif
}

/* -- one hook event -> the uplink records (and the single-chip latency windows) -- */
static void event(uint8_t kind, uint8_t seq, uint32_t t, uint32_t raw)
{
    sb_ul_t *r;
    switch (kind) {
    case COM_SB_EV_CMD:
        r = ul_find(seq, COM_COSIM_UL_DONE, COM_COSIM_UL_QUEUED);
        if (r) {
            REP_BEGIN();
            r->t_cmd_queued_us = counts_to_us(t);
            r->flags |= COM_COSIM_UL_QUEUED;
            REP_END();
            s_ul_raw_q[r - &SB_UL(0)] = raw;
#ifndef FC_COSIM
            if (r->t_cmd_queued_us - r->t_uplink_us > s_w_cl)
                s_w_cl = r->t_cmd_queued_us - r->t_uplink_us;
#endif
        }
        break;
    case COM_SB_EV_SNAP_OK:
        r = ul_find(seq, COM_COSIM_UL_QUEUED, COM_COSIM_UL_SNAP);
        if (r) {
            REP_BEGIN();
            r->t_snap_us = counts_to_us(t);
            r->flags |= COM_COSIM_UL_SNAP;
            REP_END();
        }
        break;
    case COM_SB_EV_SNAP_REFUSED:
        r = ul_find(seq, COM_COSIM_UL_QUEUED, 0u);
        if (r) {
            REP_BEGIN();
            r->flags |= COM_COSIM_UL_REFUSED;
            REP_END();
            s_g.refused_seen++;
        }
        break;
    case COM_SB_EV_EXPIRED:
        r = ul_find(seq, COM_COSIM_UL_SNAP, COM_COSIM_UL_TELEM | COM_COSIM_UL_EXPIRED);
        if (r) {
            REP_BEGIN();
            r->flags |= COM_COSIM_UL_EXPIRED;
            REP_END();
        }
        s_g.expired_seen++;
        if (s_g.burst_open && seq == s_g.burst_last)
            s_g.burst_expired = true;                      /* the newest burst reply waited too long for the radio */
        break;
    case COM_SB_EV_TELEM:
        r = ul_find(seq, COM_COSIM_UL_SNAP, COM_COSIM_UL_TELEM | COM_COSIM_UL_EXPIRED);
        if (r == 0) {
            s_g.unmatched++;                               /* a second TELEM for one uplink, or for none */
            break;
        }
        {
            const uint32_t age = raw - s_ul_raw_q[r - &SB_UL(0)];
            REP_BEGIN();
            r->t_telem_us = counts_to_us(t);
            r->flags |= COM_COSIM_UL_TELEM;
            REP_END();
            if (age > s_g.age_max)
                s_g.age_max = age;
            if (age >= SB_WINDOW_COUNTS)
                s_g.out_of_window++;                       /* D5 violated */
#ifndef FC_COSIM
            if (r->t_telem_us - r->t_uplink_us > s_w_el)
                s_w_el = r->t_telem_us - r->t_uplink_us;
            {
                const uint8_t j = (uint8_t)(seq & 7u);
                if (s_f.sl_valid & (1u << j)) {
                    const uint32_t us = counts_to_us(t - s_f.snap_done_t[j]);
                    s_f.sl_valid &= (uint8_t)~(1u << j);
                    if (us > s_w_sl)
                        s_w_sl = us;
                }
            }
#endif
        }
        break;
    case COM_SB_EV_STAT: {                                 /* seq = the STAT kind */
#ifdef FC_COSIM
        const uint16_t k = g_cosim_report.stat_count;
        volatile com_cosim_stat_t *s = &g_cosim_report.st[k & (COM_COSIM_STATS - 1u)];
        REP_BEGIN();
        s->t_us = counts_to_us(t);
        s->kind = seq;
        s->reserved = 0;
        s->number = k;
        g_cosim_report.stat_count = (uint16_t)(k + 1u);
        REP_END();
#endif
        if (seq == COM_STAT_KIND_BOOT && s_boot_raw == 0xFFFFFFFFUL)
            s_boot_raw = raw;                              /* R3: real time since com_timer_init */
        break;
    }
    default:
        break;
    }
}

static void events(void)
{
    uint8_t i;
    for (i = 0; i < s_ev_n; i++)
        event(s_ev[i].kind, s_ev[i].seq, s_ev[i].t, s_ev[i].raw);
    s_ev_n = 0;
}

static void sb_end(void)
{
#ifndef FC_COSIM
    static const char end[] = "\nSB END\n";
    while (!con_write(end, sizeof end - 1u))
        wdt_reset();
    while (con_free() < SB_CON_BUF - 1u)                   /* drain the console */
        wdt_reset();
    while (!(UCSR0A & (1 << TXC0)))
        wdt_reset();
#else
    cosim_counters(s_tick);
    REP_BEGIN();
    g_cosim_report.done = 1;
    REP_END();
#endif
    cli();
    sleep_enable();
    sleep_cpu();                                           /* simavr stops on sleep with interrupts off */
}

void com_sb_pass(void)
{
    const uint32_t tick = com_timer_ticks();
    const uint16_t t1 = com_timer_tcnt1();
    s_now += (uint16_t)(t1 - s_t1);                        /* COM time since the bench clock was last brought up */
    s_t1 = t1;
    s_raw += (uint16_t)(t1 - s_raw_t1);
    s_raw_t1 = t1;
    events();
    if (tick != s_tick) {                                  /* once per COM tick (the FCU's tick in this model) */
        const uint32_t d = tick - s_tick;
        s_tick = tick;
#ifndef FC_COSIM
        if (tick >= s_next_report) {                       /* closes the window [tick - 1 s, tick) first */
            s_next_report += COM_TICK_HZ;
            report(tick);
        }
        fcu_tick(tick, d);
#else
        (void)d;
        cosim_counters(tick);
#endif
        ground_tick(tick);
        if (tick >= (uint32_t)COM_SB_SECONDS * COM_TICK_HZ)
            sb_end();
    }
#ifndef FC_COSIM
    fcu_rx();
#endif
    ground_bytes();
#ifndef FC_COSIM
    fcu_tx();
#endif
    ground_rx();
#ifndef FC_COSIM
    s_t1 = com_timer_tcnt1();                              /* single chip: the time spent here does not count */
#endif
}

void com_sb_mark(uint8_t i)
{
    s_mark[i] = com_timer_tcnt1();
}

void com_sb_end_of_pass(uint16_t pass_us, uint16_t stack_free)
{
    uint8_t i;
#ifndef FC_COSIM
    for (i = 1; i < COM_SB_SECTIONS; i++) {
        const uint16_t d = (uint16_t)(s_mark[i] - s_mark[i - 1u]);
        if (d > s_sec_max[i])
            s_sec_max[i] = d;
    }
    if (pass_us > s_w_pm)
        s_w_pm = pass_us;
    s_w_np++;
    s_w_sum += pass_us;
    s_stack_free = stack_free;                             /* the main loop's stack_free_min_step() result */
#else
    const uint32_t sec = s_tick / COM_TICK_HZ;
    volatile uint16_t *pm = &g_cosim_report.pass_max_s_us[sec < COM_COSIM_SECONDS ? sec : COM_COSIM_SECONDS - 1u];
    REP_BEGIN();
    for (i = 1; i < COM_SB_SECTIONS; i++) {
        const uint16_t d = (uint16_t)((uint16_t)(s_mark[i] - s_mark[i - 1u]) / COM_T1_COUNTS_PER_US);
        if (d > g_cosim_report.sec_max_us[i])
            g_cosim_report.sec_max_us[i] = d;
    }
    if (pass_us > g_cosim_report.pass_max_us)
        g_cosim_report.pass_max_us = pass_us;
    if (pass_us > *pm)
        *pm = pass_us;
    g_cosim_report.passes++;
    if (stack_free < g_cosim_report.stack_free_min)
        g_cosim_report.stack_free_min = stack_free;
    REP_END();
#endif
}

void com_sb_telem_sent(const uint8_t *frame, uint8_t n)
{
    uint8_t i;
    com_sb_event(COM_SB_EV_TELEM, frame[5]);
    for (i = 0; i < n && i < SB_TELEM_FRAME_BYTES; i++)
        s_g.rx[i] = frame[i];
    s_g.rx_n = n;
    s_g.rx_new = true;
}

#endif /* FC_SIMBENCH */
