/* gns_simbench.c - see gns_simbench.h (compiled only with -D FC_SIMBENCH; v0.8.1). */
#ifdef FC_SIMBENCH
#include "gns_simbench.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <math.h>
#include <stdlib.h>

#include "fc_mode.h"
#include "params_gen.h"
#include "core/glink_proto.h"
#include "core/gps_nav.h"
#include "core/gps_ubx.h"
#include "core/link_proto.h"
#include "core/position_ctrl.h"
#include "hal/glink_uart.h"
#include "hal/gps_uart.h"
#include "hal/stack.h"
#include "hal/timer.h"
#if !defined(FC_COSIM)
#include "hal/uart_log.h"
#endif

/* sin(2 pi i / 64) in Q15 with linear interpolation (the NAV bench's table) */
static const int16_t k_sin64[65] PROGMEM = {
    0, 3212, 6393, 9512, 12539, 15446, 18204, 20787, 23170, 25329, 27245, 28898, 30273, 31356, 32137, 32609, 32767,
    32609, 32137, 31356, 30273, 28898, 27245, 25329, 23170, 20787, 18204, 15446, 12539, 9512, 6393, 3212, 0,
    -3212, -6393, -9512, -12539, -15446, -18204, -20787, -23170, -25329, -27245, -28898, -30273, -31356, -32137,
    -32609, -32767, -32609, -32137, -31356, -30273, -28898, -27245, -25329, -23170, -20787, -18204, -15446, -12539,
    -9512, -6393, -3212, 0};

static int16_t sin_q15(uint16_t ph)
{
    const uint8_t i = (uint8_t)(ph >> 10), f = (uint8_t)(ph >> 2);
    const int16_t a = (int16_t)pgm_read_word(&k_sin64[i]), b = (int16_t)pgm_read_word(&k_sin64[i + 1u]);
    return (int16_t)(a + (int16_t)(((int32_t)(b - a) * f) >> 8));
}

/* ======================================================================================== */
/* a synthetic u-blox receiver on the GPS UART (both builds: simavr has no receiver on RXD1; v0.8 NAV bench -> GNS).
 * It answers as the NEO-6M of v0.8 (below); the bought NEO-7M differs here only in MON-VER - bench not changed     */
/* ======================================================================================== */
/* Factory state: 9600 baud, NMEA out (a 1 Hz burst of RCV_NMEA_BYTES), no UBX NAV message, 1 Hz.  It parses the frames
 * the GNS queues (hal/gps_uart.c's copies) when they were sent at its own baud and answers RCV_RESP_TICKS later at its
 * baud: CFG-PRT for another baud -> switches (no ACK: the answer would leave at the old baud while it switches - the
 * reference configurator expects none on the 9600 path), CFG-PRT for its own baud / CFG-RATE / CFG-MSG / CFG-NAV5 ->
 * ACK-ACK (CFG-MSG enables the message, CFG-PRT's outProtoMask without NMEA stops the NMEA), MON-VER poll -> a 100-byte
 * MON-VER (sw "7.03 (45969)", hw "00040007" = u-blox 6 - the vehicle's NEO-7M says "00070000" - two 30-byte extensions; the GNS stores 64 of them).  Once a
 * message is enabled and the rate is 200 ms: one epoch every RCV_EPOCH_COUNTS (200.1 ms: the receiver's clock drifts
 * 500 ppm against the GNS's, so the epoch phase sweeps the 4 ms poll), NAV-POSLLH + NAV-SOL + NAV-VELNED back to back,
 * 3D fix, gpsFixOK, RCV_NUM_SV satellites, hAcc RCV_HACC_MM, sAcc RCV_SACC_CMS, the antenna moving on a 0.5 m / 0.3 m
 * ellipse of 20 s period (velocities its derivative).  Bytes sent at a baud the GNS does not listen at reach it as noise
 * (LFSR), one byte per byte time - the parser must skip them.  Every number here is a bench input, not a model of the
 * receiver's accuracy (ASSUMED). */
#define RCV_BAUD_FACTORY 9600UL
#define RCV_RESP_TICKS 5u                    /* the answer to a request, 20 ms later (ASSUMED) */
#define RCV_EPOCH_COUNTS 400200UL            /* 200.1 ms in Timer1 counts (FC_PWM_COUNTS_PER_US = 2) */
#define RCV_ITOW0 302400000UL                /* ms of the GPS week of the first epoch */
#define RCV_LAT0 375665000L                  /* 37.5665 deg N (1e-7 deg) */
#define RCV_LON0 1269780000L                 /* 126.978 deg E */
#define RCV_NUM_SV 9u
#define RCV_HACC_MM 1800UL
#define RCV_SACC_CMS 25UL
#define RCV_NMEA_BYTES 420u                  /* one second's NMEA burst at 9600 baud (GGA, GSA, GSV x3, RMC, VTG) */
#define RCV_MONVER_LEN 100u

static struct {
    uint32_t baud;                           /* the receiver's UART baud */
    bool     nmea;                           /* NMEA output on */
    uint8_t  msg_mask;                       /* 1 POSLLH, 2 VELNED, 4 SOL enabled */
    uint16_t meas_ms;
    uint8_t  f[UBX_FRAME_MAX];               /* the GNS's frame being collected (it queues one per poll) */
    uint8_t  fn;
    uint8_t  resp_kind;                      /* 0 none, 1 ACK, 2 MON-VER */
    uint8_t  resp_cls, resp_id;
    uint32_t resp_tick;
    bool     epochs_on;
    uint32_t next_epoch;                     /* Timer1 counts (timer_sim_now) of the next epoch */
    uint32_t epoch_n;
    uint16_t nmea_left;
    uint32_t next_nmea;
    uint16_t lfsr;
    uint16_t heard, heard_wrong_baud, acks, vers, switches, epochs, dropped, noise;
} s_rcv;                                     /* rcv_init: the factory state */

static uint16_t rcv_byte_cycles(void)
{
    return (uint16_t)(10UL * FC_F_CPU_HZ / s_rcv.baud);
}

/* queue bytes "on the wire": as they are when the GNS listens at the receiver's baud, else noise */
static void rcv_send(const uint8_t *b, uint8_t n)
{
    uint8_t buf[32], i, k, sent;
    const bool match = gps_uart_baud() == s_rcv.baud;
    if (s_rcv.lfsr == 0u)
        s_rcv.lfsr = 0xACE1u;
    while (n) {
        k = n < sizeof buf ? n : (uint8_t)sizeof buf;
        for (i = 0; i < k; i++) {
            if (match) {
                buf[i] = b[i];
            } else {
                s_rcv.lfsr = (uint16_t)((s_rcv.lfsr >> 1) ^ (-(uint16_t)(s_rcv.lfsr & 1u) & 0xB400u));
                buf[i] = (uint8_t)s_rcv.lfsr;
            }
        }
        sent = gps_uart_bench_send(buf, k, rcv_byte_cycles());
        if (!match)
            s_rcv.noise = (uint16_t)(s_rcv.noise + sent);
        if (sent != k) {
            s_rcv.dropped = (uint16_t)(s_rcv.dropped + (uint8_t)(n - sent));
            return;
        }
        b += k;
        n = (uint8_t)(n - k);
    }
}

static uint8_t *put_u32le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
    return p + 4;
}

/* the receiver's output, frames waiting for the wire (built in place: payload at + 6, then sealed); rcv_flush moves
 * them onto the wire as the bench queue frees up - the wire then carries them back to back at the receiver's baud */
#define RCV_OUT_N 160u
static uint8_t s_out[RCV_OUT_N];
static uint8_t s_out_n;

static uint8_t *rcv_begin(uint8_t n)                     /* the payload's place, NULL (dropped, counted) without room */
{
    if ((uint16_t)s_out_n + UBX_OVERHEAD + n > RCV_OUT_N) {
        s_rcv.dropped = (uint16_t)(s_rcv.dropped + UBX_OVERHEAD + n);
        return 0;
    }
    return s_out + s_out_n + 6;
}

static void rcv_frame(uint8_t cls, uint8_t id, uint8_t n)
{
    uint8_t *const f = s_out + s_out_n;
    s_out_n = (uint8_t)(s_out_n + ubx_frame(cls, id, f + 6, n, f));
}

static void rcv_flush(void)
{
    uint8_t room = (uint8_t)(GPS_UART_BENCH_Q - 1u - gps_uart_bench_pending()), k, i;
    k = s_out_n < room ? s_out_n : room;
    if (k == 0u)
        return;
    rcv_send(s_out, k);
    for (i = k; i < s_out_n; i++)
        s_out[i - k] = s_out[i];
    s_out_n = (uint8_t)(s_out_n - k);
}

#if defined(FC_COSIM)
/* v0.9 closed-loop co-simulation (tools/cosim/cosim4.c, gps=): the antenna's position comes from the harness's plant -
 * the harness finds g_cosim_gps with avr-nm and writes it between two instructions of this core (magic, then every
 * plant step: seq + 1).  Layout fixed, little-endian, 24 bytes (cosim4.c GPS_BLOCK_BYTES): magic u16 0x7067, seq u8,
 * cut u8 (1 = the GPS source is cut: the receiver sends nothing), north / east i32 mm, v_north / v_east i32 mm/s, up i32
 * mm.  Without the magic (open-loop runs) the receiver flies its ellipse as before. */
typedef struct {
    uint16_t magic;
    uint8_t  seq, cut;
    int32_t  north_mm, east_mm, vn_mms, ve_mms, up_mm;
} gns_cosim_gps_t;
volatile gns_cosim_gps_t g_cosim_gps;
static uint16_t s_gps_cut_epochs;                /* epochs not sent while cut (report: rcv_dropped's neighbour) */

/* a consistent copy (the harness writes between two instructions; a read spanning a write sees seq change) */
static bool cosim_gps(float *dn, float *de, float *vn, float *ve, int32_t *up, bool *cut)
{
    uint8_t k, s0;
    int32_t n = 0, e = 0, v1 = 0, v2 = 0, u = 0;
    uint8_t c = 0;
    if (g_cosim_gps.magic != 0x7067u)
        return false;
    for (k = 0; k < 4u; k++) {
        s0 = g_cosim_gps.seq;
        c = g_cosim_gps.cut;
        n = g_cosim_gps.north_mm;
        e = g_cosim_gps.east_mm;
        v1 = g_cosim_gps.vn_mms;
        v2 = g_cosim_gps.ve_mms;
        u = g_cosim_gps.up_mm;
        if (g_cosim_gps.seq == s0)
            break;
    }
    *dn = (float)n * 0.001f;
    *de = (float)e * 0.001f;
    *vn = (float)v1 * 0.001f;
    *ve = (float)v2 * 0.001f;
    *up = u;
    *cut = c != 0u;
    return true;
}
#endif

/* one epoch: POSLLH, SOL, VELNED (the enabled ones), back to back */
static void rcv_epoch(void)
{
    uint8_t *pl, *p, i;
    const uint32_t itow = RCV_ITOW0 + s_rcv.epoch_n * 200UL;
    /* the antenna on its ellipse: phase 2 pi n / 100 (20 s at 5 Hz), the 64-step sine table above */
    const uint16_t ph = (uint16_t)(s_rcv.epoch_n * 655u);
    const float sn = (float)sin_q15(ph) * (1.0f / 32767.0f), cs = (float)sin_q15((uint16_t)(ph + 0x4000u)) * (1.0f / 32767.0f);
    float dn = 0.5f * sn, de = 0.3f * cs;                                /* m north / east */
    float vn = 0.5f * 0.31416f * cs, ve = -0.3f * 0.31416f * sn;         /* m/s (omega = 2 pi / 20 s) */
    int32_t up_mm = 0, lat, lon;
#if defined(FC_COSIM)
    {
        bool cut;
        if (cosim_gps(&dn, &de, &vn, &ve, &up_mm, &cut) && cut) {   /* the GPS source cut: nothing on the wire */
            s_rcv.epoch_n++;
            s_gps_cut_epochs++;
            return;
        }
    }
#endif
    lat = RCV_LAT0 + (int32_t)lroundf(dn * 89.83f);
    lon = RCV_LON0 + (int32_t)lroundf(de * 113.36f);
    if ((s_rcv.msg_mask & 1u) && (pl = rcv_begin(UBX_LEN_POSLLH)) != 0) {   /* NAV-POSLLH */
        p = put_u32le(pl, itow);
        p = put_u32le(p, (uint32_t)lon);
        p = put_u32le(p, (uint32_t)lat);
        p = put_u32le(p, (uint32_t)(52000L + up_mm));                    /* height 52 m (+ the plant's, closed loop) */
        p = put_u32le(p, (uint32_t)(26000L + up_mm));                    /* hMSL */
        p = put_u32le(p, RCV_HACC_MM);
        (void)put_u32le(p, 2600UL);
        rcv_frame(UBX_CLS_NAV, UBX_ID_NAV_POSLLH, UBX_LEN_POSLLH);
    }
    if ((s_rcv.msg_mask & 4u) && (pl = rcv_begin(UBX_LEN_SOL)) != 0) {      /* NAV-SOL */
        for (i = 0; i < UBX_LEN_SOL; i++) pl[i] = 0;
        (void)put_u32le(pl, itow);
        pl[8] = (uint8_t)(2100u & 0xFFu);                                /* week */
        pl[9] = (uint8_t)(2100u >> 8);
        pl[10] = UBX_FIX_3D;
        pl[11] = 0x0Du;                                                  /* gpsFixOK | WKNSET | TOWSET */
        (void)put_u32le(pl + 24, 250UL);                                 /* pAcc 2.5 m */
        (void)put_u32le(pl + 40, RCV_SACC_CMS);
        pl[44] = 140u;                                                   /* pDOP 1.40 */
        pl[47] = RCV_NUM_SV;
        rcv_frame(UBX_CLS_NAV, UBX_ID_NAV_SOL, UBX_LEN_SOL);
    }
    if ((s_rcv.msg_mask & 2u) && (pl = rcv_begin(UBX_LEN_VELNED)) != 0) {   /* NAV-VELNED */
        const int32_t vn_c = (int32_t)lroundf(vn * 100.0f), ve_c = (int32_t)lroundf(ve * 100.0f);
        for (i = 0; i < UBX_LEN_VELNED; i++) pl[i] = 0;
        p = put_u32le(pl, itow);
        p = put_u32le(p, (uint32_t)vn_c);
        p = put_u32le(p, (uint32_t)ve_c);
        p = put_u32le(p, 0UL);
        p = put_u32le(p, (uint32_t)(labs(vn_c) + labs(ve_c)));           /* speed (bench: |vn| + |ve|) */
        p = put_u32le(p, (uint32_t)(labs(vn_c) + labs(ve_c)));
        p = put_u32le(p, 0UL);
        (void)put_u32le(p, RCV_SACC_CMS);
        rcv_frame(UBX_CLS_NAV, UBX_ID_NAV_VELNED, UBX_LEN_VELNED);
    }
    s_rcv.epoch_n++;
    s_rcv.epochs++;
}

static void rcv_monver(void)
{
    static const char sw[] PROGMEM = "7.03 (45969)";
    static const char hw[] PROGMEM = "00040007";
    static const char e1[] PROGMEM = "PROTVER 14.00";
    static const char e2[] PROGMEM = "GPS;SBAS;QZSS";
    uint8_t *const pl = rcv_begin(RCV_MONVER_LEN), i;
    if (pl == 0)
        return;
    for (i = 0; i < RCV_MONVER_LEN; i++) pl[i] = 0;
    for (i = 0; pgm_read_byte(&sw[i]); i++) pl[i] = pgm_read_byte(&sw[i]);
    for (i = 0; pgm_read_byte(&hw[i]); i++) pl[30 + i] = pgm_read_byte(&hw[i]);
    for (i = 0; pgm_read_byte(&e1[i]); i++) pl[40 + i] = pgm_read_byte(&e1[i]);
    for (i = 0; pgm_read_byte(&e2[i]); i++) pl[70 + i] = pgm_read_byte(&e2[i]);
    rcv_frame(UBX_CLS_MON, UBX_ID_MON_VER, RCV_MONVER_LEN);
    s_rcv.vers++;
}

/* a frame the GNS sent, heard at the receiver's baud */
static void rcv_heard(void)
{
    struct {
        uint8_t cls, id;
        uint16_t length;
        const uint8_t *buf;
    } pp, *p = &pp;
    pp.cls = s_rcv.f[2];
    pp.id = s_rcv.f[3];
    pp.length = (uint16_t)(s_rcv.f[4] | (s_rcv.f[5] << 8));
    pp.buf = s_rcv.f + 6;
    s_rcv.heard++;
    if (p->cls == UBX_CLS_CFG && p->id == UBX_ID_CFG_PRT && p->length == UBX_LEN_CFG_PRT) {
        const uint32_t b = (uint32_t)p->buf[8] | ((uint32_t)p->buf[9] << 8) | ((uint32_t)p->buf[10] << 16) |
                           ((uint32_t)p->buf[11] << 24);
        if (!(p->buf[14] & UBX_PROTO_NMEA))
            s_rcv.nmea = false;
        if (b != s_rcv.baud) {
            s_rcv.baud = b;                              /* switches; the ACK is lost in the switch */
            s_rcv.switches++;
            return;
        }
    } else if (p->cls == UBX_CLS_CFG && p->id == UBX_ID_CFG_RATE && p->length == UBX_LEN_CFG_RATE) {
        s_rcv.meas_ms = (uint16_t)(p->buf[0] | (p->buf[1] << 8));
    } else if (p->cls == UBX_CLS_CFG && p->id == UBX_ID_CFG_MSG && p->length == UBX_LEN_CFG_MSG) {
        if (p->buf[0] == UBX_CLS_NAV && p->buf[2] != 0u) {
            if (p->buf[1] == UBX_ID_NAV_POSLLH) s_rcv.msg_mask |= 1u;
            if (p->buf[1] == UBX_ID_NAV_VELNED) s_rcv.msg_mask |= 2u;
            if (p->buf[1] == UBX_ID_NAV_SOL) s_rcv.msg_mask |= 4u;
        }
    } else if (p->cls == UBX_CLS_MON && p->id == UBX_ID_MON_VER && p->length == 0u) {
        s_rcv.resp_kind = 2u;
        s_rcv.resp_tick = timer_ticks() + RCV_RESP_TICKS;
        return;
    } else if (p->cls != UBX_CLS_CFG) {
        return;
    }
    s_rcv.resp_kind = 1u;                                /* ACK-ACK */
    s_rcv.resp_cls = p->cls;
    s_rcv.resp_id = p->id;
    s_rcv.resp_tick = timer_ticks() + RCV_RESP_TICKS;
}

static void rcv_step(uint32_t tick)
{
    uint8_t b[16], n, i;
    const uint32_t now = timer_sim_now();
    if (s_rcv.baud == 0u) {                              /* the factory state, at the first call */
        s_rcv.baud = RCV_BAUD_FACTORY;
        s_rcv.nmea = true;
        s_rcv.meas_ms = 1000u;
        s_rcv.lfsr = 0xACE1u;
    }
    /* the GNS's frames (copies of what it queued; whole frames: sync, class, id, length, payload, checksum) */
    while ((n = gps_uart_bench_take_tx(b, (uint8_t)sizeof b)) != 0u) {
        for (i = 0; i < n; i++) {
            if (s_rcv.fn == 0u && b[i] != UBX_SYNC1)
                continue;
            if (s_rcv.fn < UBX_FRAME_MAX)
                s_rcv.f[s_rcv.fn++] = b[i];
            if (s_rcv.fn >= UBX_OVERHEAD && s_rcv.fn == (uint8_t)(UBX_OVERHEAD + s_rcv.f[4])) {
                uint8_t ca, cb;
                ubx_checksum(s_rcv.f + 2, (uint16_t)(s_rcv.f[4] + 4u), &ca, &cb);
                if (ca == s_rcv.f[s_rcv.fn - 2u] && cb == s_rcv.f[s_rcv.fn - 1u]) {
                    if (gps_uart_baud() == s_rcv.baud)
                        rcv_heard();
                    else
                        s_rcv.heard_wrong_baud++;
                }
                s_rcv.fn = 0;
            } else if (s_rcv.fn >= UBX_FRAME_MAX) {
                s_rcv.fn = 0;                            /* not a frame the GNS sends */
            }
        }
    }
    if (s_rcv.resp_kind != 0u && (int32_t)(tick - s_rcv.resp_tick) >= 0) {
        if (s_rcv.resp_kind == 1u) {
            uint8_t *const pl = rcv_begin(2u);
            if (pl != 0) {
                pl[0] = s_rcv.resp_cls;
                pl[1] = s_rcv.resp_id;
                rcv_frame(UBX_CLS_ACK, UBX_ID_ACK_ACK, 2u);
                s_rcv.acks++;
            }
        } else {
            rcv_monver();
        }
        s_rcv.resp_kind = 0;
    }
    /* the 5 Hz epochs once configured */
    if (!s_rcv.epochs_on && s_rcv.msg_mask != 0u && s_rcv.meas_ms == 200u) {
        s_rcv.epochs_on = true;
        s_rcv.next_epoch = now + 2000UL;                 /* 1 ms: the first epoch */
    }
    if (s_rcv.epochs_on && (int32_t)(now - s_rcv.next_epoch) >= 0) {
        rcv_epoch();
        s_rcv.next_epoch += RCV_EPOCH_COUNTS;
    }
    rcv_flush();
    /* the factory NMEA: a burst every second, topped up 32 bytes at a time */
    if (s_rcv.nmea && s_out_n == 0u) {
        if (s_rcv.nmea_left == 0u && (int32_t)(now - s_rcv.next_nmea) >= 0) {
            s_rcv.nmea_left = RCV_NMEA_BYTES;
            s_rcv.next_nmea = now + 2000000UL;           /* 1 s */
        }
        if (s_rcv.nmea_left != 0u && gps_uart_bench_pending() < 16u) {
            static const char gga[] PROGMEM = "$GPGGA,120000.00,3733.99000,N,12658.68000,E,1,09,1.0,26.0,M,26.0,M,,*5C\r\n";
            uint8_t t[32];
            const uint8_t k = s_rcv.nmea_left < 32u ? (uint8_t)s_rcv.nmea_left : 32u;
            for (i = 0; i < k; i++)
                t[i] = pgm_read_byte(&gga[(uint8_t)((RCV_NMEA_BYTES - s_rcv.nmea_left + i) % (sizeof gga - 1u))]);
            rcv_send(t, k);
            s_rcv.nmea_left = (uint16_t)(s_rcv.nmea_left - k);
        }
    }
}

/* ======================================================================================== */
/* measurements (both builds)                                                               */
/* ======================================================================================== */
#define SB_US(counts) ((uint32_t)(counts) / FC_PWM_COUNTS_PER_US)
static volatile uint32_t s_t_complete;          /* timer_sim_now of the newest STATE's completion (single chip) */
static volatile bool s_complete_pending;        /* ... not answered yet */
static uint32_t s_t_result, s_pass_t0, s_busy_win;
static uint16_t s_cycles, s_cyc_n, s_refused, s_cyc_max, s_cyc_last, s_fin_max, s_bm_max, s_poll_max, s_busy_pass_max;
static uint32_t s_cyc_sum, s_fin_sum;
static uint8_t s_poll_nmax;
static bool s_awake;

void gns_sb_pass_start(void)
{
    s_pass_t0 = timer_sim_now();
    s_awake = true;
}

void gns_sb_pass_end(void)
{
    if (s_awake) {
        const uint32_t d = timer_sim_now() - s_pass_t0;
        s_busy_win += d;
        if (SB_US(d) > s_busy_pass_max) s_busy_pass_max = (uint16_t)(SB_US(d) > 0xFFFFu ? 0xFFFFu : SB_US(d));
        s_awake = false;
    }
}

void gns_sb_poll(uint8_t n_rx, uint16_t us)
{
    if (us > s_poll_max) s_poll_max = us;
    if (n_rx > s_poll_nmax) s_poll_nmax = n_rx;
}

void gns_sb_result(bool queued)
{
    s_t_result = timer_sim_now();
    if (!queued) {
        s_refused++;
        return;
    }
    s_cycles++;
    if (s_complete_pending) {                    /* single chip: from the STATE's completion (the bench's COMPB) */
        const uint32_t d = SB_US(s_t_result - s_t_complete);
        s_cyc_last = (uint16_t)(d > 0xFFFFu ? 0xFFFFu : d);
        if (s_cyc_last > s_cyc_max) s_cyc_max = s_cyc_last;
        s_cyc_sum += s_cyc_last;
        s_cyc_n++;
    }
}

void gns_sb_finished(const gns_t *g)
{
    const uint32_t now = timer_sim_now();
    const uint32_t f = SB_US(now - s_t_result);
    (void)g;
    if (f > s_fin_max) s_fin_max = (uint16_t)(f > 0xFFFFu ? 0xFFFFu : f);
    s_fin_sum += f;
    if (s_complete_pending) {
        const uint32_t b = SB_US(now - s_t_complete);
        if (b > s_bm_max) s_bm_max = (uint16_t)(b > 0xFFFFu ? 0xFFFFu : b);
        s_complete_pending = false;
    }
}

#if defined(FC_COSIM)
/* ======================================================================================== */
/* gns_cosim: the figures in RAM for the harness (avr-nm g_gns_cosim_report; layout fixed, little-endian)                  */
/* ======================================================================================== */
typedef struct {
    uint16_t magic;                              /* 0x6E73 'sn' once initialised */
    uint16_t cycles, refused, fin_max_us, poll_max_us, busy_pass_max_us, stack_free;
    uint16_t states, hold_steps, gaps, restarts, imu_bad, clock_slips;
    uint8_t  cfg_state, cfg_ok, ver_ok, module, snap_flags, pos_mode;
    uint16_t epochs, fused, ubx_frames, ubx_bad_ck;
    uint16_t gl_rx_bytes, gl_rx_overflow, gl_rx_errors, gl_tx_bytes, gl_bad_crc;
    uint16_t gps_rx_overflow, rcv_epochs, rcv_dropped;
    uint32_t ticks;
    uint16_t gps_cut_epochs;                     /* v0.9: epochs not sent while the harness cut the GPS source */
} gns_cosim_report_t;
volatile gns_cosim_report_t g_gns_cosim_report;
#endif

#if !defined(FC_COSIM)
/* ======================================================================================== */
/* console (single chip)                                                                    */
/* ======================================================================================== */
static char s_buf[200];
static uint16_t s_reports_dropped;

static uint8_t put_u(uint8_t pos, const char *key_P, uint32_t v)
{
    static const char hex[] PROGMEM = "0123456789abcdef";
    const uint8_t b[4] = {(uint8_t)(v >> 24), (uint8_t)(v >> 16), (uint8_t)(v >> 8), (uint8_t)v};
    uint8_t k;
    bool lead = true;
    char c;
    if (pos > (uint8_t)(sizeof s_buf - 18u))
        return pos;                              /* the line is full: the field is left out (never overflows) */
    while ((c = (char)pgm_read_byte(key_P++)) != 0) s_buf[pos++] = c;
    s_buf[pos++] = '0';
    s_buf[pos++] = 'x';
    for (k = 0; k < 8u; k++) {
        const uint8_t d = (k & 1u) ? (uint8_t)(b[k >> 1] & 0xFu) : (uint8_t)(b[k >> 1] >> 4);
        if (d || !lead || k == 7u) {
            s_buf[pos++] = (char)pgm_read_byte(&hex[d]);
            lead = false;
        }
    }
    s_buf[pos++] = ' ';
    return pos;
}

static uint8_t line_start(char c1, char c2)
{
    s_buf[0] = '\n';
    s_buf[1] = c1;
    s_buf[2] = c2;
    s_buf[3] = ' ';
    return 4;
}

static void send_line(uint8_t p)
{
    s_buf[p++] = '\n';
    if (!uart_log_write((const uint8_t *)s_buf, p))   /* never wait */
        s_reports_dropped++;
}

/* ======================================================================================== */
/* the synthetic NAV on the glink (single chip): STATE frames by a Timer1 compare interrupt                               */
/* ======================================================================================== */
#define SN_PH_MIN 1000u                          /* the injection phase sweeps [SN_PH_MIN, SN_PH_MAX] Timer1 counts */
#define SN_PH_MAX 9000u                          /* (4 ms: one poll period) */
#define SN_LAST_BYTE_COUNTS ((uint16_t)((GL_STATE_FRAME_BYTES - 1u) * (GLINK_STATE_FRAME_US / GL_STATE_FRAME_BYTES) * \
                                        FC_PWM_COUNTS_PER_US))
static uint8_t s_sf[GL_STATE_FRAME_BYTES];       /* the frame of cycle s_m */
static volatile uint8_t s_inj;                   /* 0 none, 1 frame waits for COMPA, 2 for COMPB, 3 a gap cycle */
static uint32_t s_m;                             /* the cycle of the prepared frame (NAV tick 5 m + 4) */
static uint16_t s_ph = SN_PH_MIN;
static int16_t s_ph_step = GNS_SB_PHASE_STEP;
static uint16_t s_injected, s_gap_cycles, s_faults;
static float s_h;                                /* the synthetic altitude (m) */
static gl_decoder_t s_rdec;                      /* the RESULT frames the GNS queued */
static uint16_t s_r_frames, s_r_seq_bad, s_r_engaged, s_r_hold_none, s_gate_seen, s_mode_seen;
static volatile uint8_t s_r_expect_seq;          /* the SEQ of the newest complete STATE (set at its injection) */
static bool s_in_hold;

ISR(TIMER1_COMPA_vect)
{
    if (s_inj == 1u) {
        glink_uart_bench_inject(s_sf, (uint8_t)(GL_STATE_FRAME_BYTES - 1u));   /* bytes 0..58 */
        s_inj = 2u;
    } else if (s_inj == 3u) {
        s_inj = 0u;                                                             /* a gap cycle passed */
    }
}

ISR(TIMER1_COMPB_vect)
{
    if (s_inj == 2u) {
        glink_uart_bench_inject(&s_sf[GL_STATE_FRAME_BYTES - 1u], 1u);        /* the last byte: complete */
        s_t_complete = timer_sim_now();
        s_complete_pending = true;
        s_r_expect_seq = s_sf[3];
        s_injected++;
        s_inj = 0u;
    }
}

/* the STATE of cycle m (NAV tick 5 m + 4): a synthetic flight (gns_simbench.h) */
static void sn_build(uint32_t m)
{
    gl_state_t st;
    const uint32_t k = 5u * m + 4u;
    const uint16_t t_s = (uint16_t)(m / (uint32_t)GNS_LOOP_HZ);                 /* seconds */
    const uint16_t ph1 = (uint16_t)(k * (uint16_t)(1.3f * 65536.0f / FC_LOOP_HZ)),
                   ph2 = (uint16_t)(k * (uint16_t)(0.9f * 65536.0f / FC_LOOP_HZ)),
                   ph3 = (uint16_t)(k * (uint16_t)(0.5f * 65536.0f / FC_LOOP_HZ));
    const bool on = t_s >= GNS_SB_ON_S, land = t_s >= GNS_SB_LAND_S;
    uint8_t phase = LP_PHASE_NONE, pf = 0;
    bool motor = false;
    if (on && !land) {
        motor = true;
        if (!s_in_hold) {
            s_h += 0.5f / GNS_LOOP_HZ;                                         /* CLIMB at 0.5 m/s to 1.5 m */
            if (s_h >= 1.5f) {
                s_h = 1.5f;
                s_in_hold = true;
            }
        }
        phase = s_in_hold ? LP_PHASE_HOLD : LP_PHASE_CLIMB;
    } else if (land && s_h > 0.0f) {
        motor = true;
        s_h -= 0.5f / GNS_LOOP_HZ;                                              /* LAND at 0.5 m/s */
        if (s_h < 0.0f) s_h = 0.0f;
        phase = LP_PHASE_LAND;
    } else if (land) {
        phase = LP_PHASE_LANDED;
    }
    st.tick = k;
    {
        const float fx = motor ? 0.10f : 0.0f, fy = motor ? -0.05f : 0.0f, fz = 9.81f;
        st.fsum[0] = 5.0f * fx;
        st.fsum[1] = 5.0f * fy;
        st.fsum[2] = 5.0f * fz;
    }
    st.n = 5u;
    {
        const float r = motor ? 0.02f * (float)sin_q15(ph1) / 32767.0f : 0.005f, p = motor ? 0.025f * (float)sin_q15(ph2) / 32767.0f : -0.003f,
                    y = motor ? 0.03f * (float)sin_q15(ph3) / 32767.0f : 0.0f;
        st.q[0] = 1.0f - (r * r + p * p + y * y) * 0.125f;
        st.q[1] = 0.5f * r;
        st.q[2] = 0.5f * p;
        st.q[3] = 0.5f * y;
        st.w[0] = motor ? 0.02f * 2.0f * FC_PI_F * 1.3f * (float)sin_q15((uint16_t)(ph1 + 0x4000u)) / 32767.0f : 0.0005f;
        st.w[1] = motor ? 0.025f * 2.0f * FC_PI_F * 0.9f * (float)sin_q15((uint16_t)(ph2 + 0x4000u)) / 32767.0f : -0.0003f;
        st.w[2] = motor ? 0.03f * 2.0f * FC_PI_F * 0.5f * (float)sin_q15((uint16_t)(ph3 + 0x4000u)) / 32767.0f : 0.0002f;
    }
    st.h = s_h;
    st.nflags = (uint8_t)(GL_NF_FCU_OK | GL_NF_IMU_OK | (t_s >= GNS_SB_EST_S ? GL_NF_EST_VALID : 0u) |
                          (!on || phase == LP_PHASE_LANDED ? GL_NF_PAD : 0u));
    if (t_s == GNS_SB_IMUBAD_S && (m % GNS_LOOP_HZ) == 0u) {
        st.nflags &= (uint8_t)~GL_NF_IMU_OK;                                     /* fault: frozen IMU data */
        s_faults++;
    }
    st.fcu_state = motor ? (uint8_t)LP_STATE_POWERED : (phase == LP_PHASE_LANDED ? (uint8_t)LP_STATE_UNPOWERED : (uint8_t)LP_STATE_SAFE);
    pf = nl_fcu4_pflags(phase, motor, motor && (m & 7u) == 0u, phase == LP_PHASE_LAND && s_h <= 0.3f,
                        phase == LP_PHASE_LAND && s_h <= 0.05f, motor && t_s >= GNS_SB_ON_S + 1u);
    st.pflags = pf;
    st.kt_q = (uint16_t)NL_KT_SCALE;
    (void)gl_encode_state((uint8_t)m, &st, s_sf);
    if (t_s == GNS_SB_CRC_S && (m % GNS_LOOP_HZ) == 0u) {
        s_sf[20] ^= 0x04u;                                                       /* fault: a payload bit, CRC no longer matches */
        s_faults++;
    }
}

/* the next cycle's frame and the Timer1 compares (after the previous injection: the next match is in the next period) */
static void sn_prepare(void)
{
    if (s_inj != 0u)
        return;
    if (s_m == 0u && timer_ticks() < (uint32_t)FC_LOOP_HZ / 2u)
        return;                                                                  /* the NAV boots later than the GNS */
    s_m++;
    s_ph = (uint16_t)(s_ph + s_ph_step);
    if (s_ph >= SN_PH_MAX || s_ph <= SN_PH_MIN)
        s_ph_step = (int16_t)-s_ph_step;                                         /* the phase sweeps up and down */
    OCR1A = s_ph;
    OCR1B = (uint16_t)(s_ph + SN_LAST_BYTE_COUNTS);
    if ((uint32_t)(s_m / GNS_LOOP_HZ) == GNS_SB_GAP_S && (s_m % GNS_LOOP_HZ) < GNS_SB_GAP_CYCLES) {
        s_gap_cycles++;
        s_inj = 3u;                                                              /* fault: this cycle's frame is lost */
        return;
    }
    sn_build(s_m);
    s_inj = 1u;
}

/* the RESULT frames the GNS queued */
static void sn_take_results(void)
{
    uint8_t b[32], n, i;
    uint32_t tq;
    while ((n = glink_uart_bench_take_tx(b, (uint8_t)sizeof b, &tq)) != 0u) {
        for (i = 0; i < n; i++) {
            gl_result_t r;
            if (gl_decoder_byte(&s_rdec, b[i]) != GL_TYPE_RESULT)
                continue;
            gl_unpack_result(s_rdec.buf, &r);
            s_r_frames++;
            if (s_rdec.seq != s_r_expect_seq)
                s_r_seq_bad++;
            if (r.tilt[0] != NL_TILT_NONE)
                s_r_engaged++;
            else if (s_in_hold && (uint32_t)(s_m / GNS_LOOP_HZ) < GNS_SB_LAND_S)
                s_r_hold_none++;
            if (r.flags & GPS_F_GATE_OK)
                s_gate_seen++;
            s_mode_seen |= (uint16_t)(1u << (r.pc & NL_PC_MODE_MASK));
        }
    }
}
#endif /* !FC_COSIM */

/* ======================================================================================== */
void gns_sb_init(uint8_t reset_cause)
{
#if defined(FC_COSIM)
    (void)reset_cause;
    g_gns_cosim_report.magic = 0x6E73u;
#else
    uint8_t p;
    uart_log_init();
    gl_decoder_init(&s_rdec, GL_ACCEPT_RESULT);
    TIMSK |= (uint8_t)((1 << OCIE1A) | (1 << OCIE1B));   /* the synthetic NAV's injection compares on the time base */
    p = line_start('G', 'B');
    s_buf[p++] = 'B'; s_buf[p++] = 'O'; s_buf[p++] = 'O'; s_buf[p++] = 'T'; s_buf[p++] = ' ';
    p = put_u(p, PSTR("rc="), reset_cause);
    p = put_u(p, PSTR("sr="), stack_reserve_total());
    p = put_u(p, PSTR("gb="), GLINK_BAUD / 100UL);
    p = put_u(p, PSTR("gl="), GLINK_LATENCY_TICKS);
    p = put_u(p, PSTR("lp="), GNS_LOOP_HZ);
    send_line(p);
#endif
}

void gns_sb_tick(uint32_t t, const gns_t *g)
{
    const gps_nav_t *gp = &g->gps;
    rcv_step(t);                                          /* the synthetic receiver's turn */
#if defined(FC_COSIM)
    {
        volatile gns_cosim_report_t *r = &g_gns_cosim_report;
        const glink_uart_stats_t *gu = glink_uart_stats();
        r->cycles = s_cycles;
        r->refused = s_refused;
        r->fin_max_us = s_fin_max;
        r->poll_max_us = s_poll_max;
        r->busy_pass_max_us = s_busy_pass_max;
        r->stack_free = stack_free_min_step();
        r->states = g->st.states;
        r->hold_steps = g->st.hold_steps;
        r->gaps = g->st.gaps;
        r->restarts = g->st.restarts;
        r->imu_bad = g->st.imu_bad;
        r->clock_slips = g->st.clock_slips;
        r->cfg_state = gp->cfg.state;
        r->cfg_ok = gp->cfg.cfg_ok;
        r->ver_ok = gp->cfg.ver_ok;
        r->module = gp->cfg.module;
        r->snap_flags = gp->filter.snap.flags;
        r->pos_mode = g->pos.last.mode;
        r->epochs = gp->asm_.epochs;
        r->fused = gp->filter.st_fused;
        r->ubx_frames = gp->parser.stats.frames;
        r->ubx_bad_ck = gp->parser.stats.bad_ck;
        r->gl_rx_bytes = gu->rx_bytes;
        r->gl_rx_overflow = gu->rx_overflow;
        r->gl_rx_errors = gu->rx_errors;
        r->gl_tx_bytes = gu->tx_bytes;
        r->gl_bad_crc = g->dec.stats.bad_crc;
        r->gps_rx_overflow = gps_uart_stats()->rx_overflow;
        r->rcv_epochs = s_rcv.epochs;
        r->rcv_dropped = s_rcv.dropped;
        r->gps_cut_epochs = s_gps_cut_epochs;
        r->ticks = t;
    }
#else
    {
        static uint8_t s_phase_tick;
        uint8_t p;
        sn_take_results();
        sn_prepare();
        if (++s_phase_tick >= FC_LOOP_HZ)
            s_phase_tick = 0;
        if (s_phase_tick == 0u) {                         /* 1 Hz: the cycle timing */
            p = line_start('G', 'C');
            p = put_u(p, PSTR("t="), t);
            p = put_u(p, PSTR("cy="), s_cycles);
            p = put_u(p, PSTR("cn="), s_cyc_n);
            p = put_u(p, PSTR("cm="), s_cyc_max);
            p = put_u(p, PSTR("cs="), s_cyc_sum);
            p = put_u(p, PSTR("ca="), s_cyc_last);
            p = put_u(p, PSTR("fm="), s_fin_max);
            p = put_u(p, PSTR("fs="), s_fin_sum);
            p = put_u(p, PSTR("bm="), s_bm_max);
            p = put_u(p, PSTR("pm="), s_poll_max);
            p = put_u(p, PSTR("pn="), s_poll_nmax);
            p = put_u(p, PSTR("wk="), SB_US(s_busy_win));
            p = put_u(p, PSTR("wt="), s_busy_pass_max);
            p = put_u(p, PSTR("sf="), stack_free_min_step());
            p = put_u(p, PSTR("ra="), s_refused);
            send_line(p);
            s_busy_win = 0;
        } else if (s_phase_tick == FC_LOOP_HZ / 4u) {     /* the GPS side */
            p = line_start('G', 'G');
            p = put_u(p, PSTR("t="), t);
            p = put_u(p, PSTR("cs="), gp->cfg.state);
            p = put_u(p, PSTR("co="), gp->cfg.cfg_ok);
            p = put_u(p, PSTR("vo="), gp->cfg.ver_ok);
            p = put_u(p, PSTR("md="), gp->cfg.module);
            p = put_u(p, PSTR("nk="), gp->cfg.naks);
            p = put_u(p, PSTR("to="), gp->cfg.timeouts);
            p = put_u(p, PSTR("pf="), gp->parser.stats.frames);
            p = put_u(p, PSTR("pk="), gp->parser.stats.bad_ck);
            p = put_u(p, PSTR("pl="), gp->parser.stats.bad_len);
            p = put_u(p, PSTR("ae="), gp->asm_.epochs);
            p = put_u(p, PSTR("ai="), gp->asm_.incomplete);
            p = put_u(p, PSTR("fu="), gp->filter.st_fused);
            p = put_u(p, PSTR("rp="), gp->filter.st_rej_p);
            p = put_u(p, PSTR("rv="), gp->filter.st_rej_v);
            p = put_u(p, PSTR("ps="), gp->filter.st_pos_resets);
            p = put_u(p, PSTR("sf="), gp->filter.snap.flags);
            send_line(p);
        } else if (s_phase_tick == FC_LOOP_HZ / 2u) {     /* the UARTs */
            const gps_uart_stats_t *u = gps_uart_stats();
            const glink_uart_stats_t *gu = glink_uart_stats();
            p = line_start('G', 'U');
            p = put_u(p, PSTR("t="), t);
            p = put_u(p, PSTR("ub="), u->rx_bytes);
            p = put_u(p, PSTR("ux="), u->rx_overflow);
            p = put_u(p, PSTR("ue="), u->rx_errors);
            p = put_u(p, PSTR("tb="), u->tx_bytes);
            p = put_u(p, PSTR("tr="), u->tx_refused);
            p = put_u(p, PSTR("bc="), u->baud_changes);
            p = put_u(p, PSTR("gb="), gu->rx_bytes);
            p = put_u(p, PSTR("gx="), gu->rx_overflow);
            p = put_u(p, PSTR("ge="), g->dec.stats.bad_crc);
            p = put_u(p, PSTR("gt="), gu->tx_bytes);
            p = put_u(p, PSTR("rd="), s_reports_dropped);
            send_line(p);
        } else if (s_phase_tick == (FC_LOOP_HZ * 3u) / 4u) {   /* the synthetic NAV and the GNS's own counters */
            p = line_start('G', 'N');
            p = put_u(p, PSTR("t="), t);
            p = put_u(p, PSTR("ns="), s_injected);
            p = put_u(p, PSTR("ng="), s_gap_cycles);
            p = put_u(p, PSTR("nf="), s_faults);
            p = put_u(p, PSTR("nr="), s_r_frames);
            p = put_u(p, PSTR("nq="), s_r_seq_bad);
            p = put_u(p, PSTR("ne="), s_r_engaged);
            p = put_u(p, PSTR("nh="), s_r_hold_none);
            p = put_u(p, PSTR("gk="), s_gate_seen);
            p = put_u(p, PSTR("mm="), s_mode_seen);
            p = put_u(p, PSTR("hs="), g->st.hold_steps);
            p = put_u(p, PSTR("gp="), g->st.gaps);
            p = put_u(p, PSTR("rs="), g->st.restarts);
            p = put_u(p, PSTR("is="), g->st.imu_bad);
            p = put_u(p, PSTR("cl="), g->st.clock_slips);
            p = put_u(p, PSTR("mi="), g->st.misaligned);
            p = put_u(p, PSTR("pm="), g->pos.last.mode);
            send_line(p);
        } else if (s_phase_tick == (FC_LOOP_HZ * 7u) / 8u) {   /* the synthetic receiver */
            p = line_start('G', 'J');
            p = put_u(p, PSTR("t="), t);
            p = put_u(p, PSTR("ra="), s_rcv.acks);
            p = put_u(p, PSTR("rv="), s_rcv.vers);
            p = put_u(p, PSTR("rs="), s_rcv.switches);
            p = put_u(p, PSTR("re="), s_rcv.epochs);
            p = put_u(p, PSTR("rd="), s_rcv.dropped);
            p = put_u(p, PSTR("rn="), s_rcv.noise);
            p = put_u(p, PSTR("rh="), s_rcv.heard);
            p = put_u(p, PSTR("rw="), s_rcv.heard_wrong_baud);
            p = put_u(p, PSTR("rb="), s_rcv.baud / 100UL);
            send_line(p);
        }
        if (t >= (uint32_t)GNS_SB_SECONDS * FC_LOOP_HZ) {
            const char end[] = "\nGB END\n";
            uart_log_write((const uint8_t *)end, sizeof end - 1);
            while (uart_log_free() < UART_LOG_BUF - 1u)      /* drain the console ring */
                wdt_reset();
            cli();
            sleep_enable();
            sleep_cpu();                                     /* simavr stops on sleep with interrupts off */
        }
    }
#endif
}
#endif /* FC_SIMBENCH */
