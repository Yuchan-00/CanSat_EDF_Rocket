/* com_cosim.h - the COM's co-simulation report (v0.6, PlatformIO envs com_cosim (lora_uart) and
 * com_cosim_sx127x, -D FC_SIMBENCH -D FC_COSIM).  Plain C99, no AVR header: a co-simulation
 * harness on the host includes it to read the report out of the COM's simulated RAM.
 *
 * In a co-simulation the COM's USART0 is the real ilink, wired to a real FCU (env fcu_cosim), so the
 * COM has no console.  Its synthetic ground (the same as the single-chip com_simbench, src/com/
 * com_simbench.h: CMD frames into the radio driver - lora_uart one byte per 10 / LINK_UART_BAUD s,
 * sx127x the whole packet at RxDone - NONE heartbeats every LINK_RTT_WORST_MS, ARM, LAUNCH with the
 * nonce read back from the COM's own TELEM, a back-to-back burst, a silence) runs before every
 * main-loop pass and the COM writes this report into one global object:
 *
 *     volatile com_cosim_report_t g_cosim_report;          (src/com/com_simbench.c)
 *
 * The harness finds it with `avr-nm -S firmware.elf | grep g_cosim_report` (address in the data space:
 * subtract 0x800000 from the nm value for simavr's avr->data[] index; nm's size must equal
 * COM_COSIM_REPORT_BYTES) and copies COM_COSIM_REPORT_BYTES bytes.  Check magic, version and size
 * first.  The struct is valid when seqlock is even and unchanged across the copy (the COM increments
 * it before and after each update; nothing else writes the report, no ISR touches it).  done = 1
 * once COM_SB_SECONDS of COM ticks have passed; the COM then stops with interrupts off and sleeps
 * (simavr reports the core as done), so a harness can run both chips until both sleep.
 *
 * Layout: little-endian (AVR and x86 / ARM hosts), every field naturally aligned and every size a
 * multiple of 4, so the host compiler lays the struct out exactly as avr-gcc (no padding; the
 * typedef checks below fail to compile otherwise).  Times are COM microseconds since com_sb_init
 * (Timer1 / 2, wraps after 2147 s), ticks are COM ticks (COM_TICK_HZ, tick_hz below).
 *
 * Per-uplink records (ul[], ring of COM_COSIM_UPLINKS, uplink number u at ul[u % COM_COSIM_UPLINKS],
 * its low byte in .number):
 *   t_uplink_us      the frame's last byte reached the radio driver (its scheduled arrival time)
 *   t_cmd_queued_us  the COM queued the IL_CMD for it (ilink TX ring)
 *   t_snap_us        the COM's ilink decoder completed an IL_SNAP with reply_seq = seq and the reply gate
 *                    accepted it (the decode time, not the arrival of the SNAP's last byte)
 *   t_telem_us       the COM handed the TELEM with SEQ = seq to the radio driver
 * A field is valid only with its flag.  Heartbeats repeat the ground's SEQ: an event is attributed to
 * the newest record with that seq that still lacks it.
 * IL_STAT records (st[], ring of COM_COSIM_STATS, STAT number k at st[k % COM_COSIM_STATS]): when the
 * COM queued each IL_STAT and why (COM_COSIM_STAT_*).
 *
 * Invariant: this header and com_simbench.c's writer agree on every offset (avr-gcc builds the
 * typedef checks too); the report is written only outside the measured main-loop pass.
 */
#ifndef FC_COM_COSIM_H
#define FC_COM_COSIM_H

#include <stddef.h>
#include <stdint.h>

#define COM_COSIM_MAGIC 0x434F5336UL         /* "6SOC" in memory order */
#define COM_COSIM_VERSION 1u
#define COM_COSIM_UPLINKS 32u                /* power of two */
#define COM_COSIM_STATS 16u                  /* power of two */
#define COM_COSIM_SECONDS 16u                /* pass_max_s_us[] entries; later seconds go to the last one */

/* com_cosim_stat_t.kind (= com_link.h COM_STAT_KIND_*, checked in com_simbench.c) */
#define COM_COSIM_STAT_BOOT 0u                /* once after boot, at COM tick >= COM_STAT_BOOT_TICKS (100 ms, com_link.h R3) */
#define COM_COSIM_STAT_PIGGYBACK 1u           /* right behind an IL_CMD the COM had just queued */
#define COM_COSIM_STAT_QUIET 2u               /* on the divider: no IL_CMD for > 1.5 / stat_hz */

/* com_cosim_uplink_t.flags */
#define COM_COSIM_UL_DONE 0x01u              /* t_uplink_us valid */
#define COM_COSIM_UL_QUEUED 0x02u            /* t_cmd_queued_us valid */
#define COM_COSIM_UL_SNAP 0x04u              /* t_snap_us valid */
#define COM_COSIM_UL_TELEM 0x08u             /* t_telem_us valid */
#define COM_COSIM_UL_REFUSED 0x10u           /* a SNAP with this seq was refused by the reply gate (stray / dup / late) */
#define COM_COSIM_UL_EXPIRED 0x20u           /* its TELEM was dropped: the radio did not take it within the window */
#define COM_COSIM_UL_SUPERSEDED 0x40u        /* reserved (superseded replies are only counted) */

typedef struct {
    uint32_t t_uplink_us;                    /*  0 */
    uint32_t t_cmd_queued_us;                /*  4 */
    uint32_t t_snap_us;                      /*  8 */
    uint32_t t_telem_us;                     /* 12 */
    uint8_t  seq;                            /* 16 the uplink frame's SEQ */
    uint8_t  cmd;                            /* 17 LP_CMD_* */
    uint8_t  flags;                          /* 18 COM_COSIM_UL_* */
    uint8_t  number;                         /* 19 low byte of the uplink number */
} com_cosim_uplink_t;                        /* 20 bytes */

typedef struct {
    uint32_t t_us;                           /* 0 IL_STAT queued to the ilink TX ring */
    uint8_t  kind;                           /* 4 COM_COSIM_STAT_* */
    uint8_t  reserved;                       /* 5 */
    uint16_t number;                         /* 6 STAT number (0 = the boot STAT) */
} com_cosim_stat_t;                          /* 8 bytes */

typedef struct {                             /* all cumulative since boot */
    uint16_t uplinks;                        /*  0 frames the synthetic ground started */
    uint16_t cmd_forwarded;                  /*  2 com_link_stats_t ... */
    uint16_t il_tx_dropped;                  /*  4 */
    uint16_t il_wrong_type;                  /*  6 */
    uint16_t snaps_accepted;                 /*  8 */
    uint16_t replies;                        /* 10 TELEM handed to the radio */
    uint16_t replies_superseded;             /* 12 */
    uint16_t replies_refused;                /* 14 D5 reply gate: stray / duplicate / late SNAP */
    uint16_t replies_expired;                /* 16 D5: TELEM not taken by the radio within the window */
    uint16_t stats_sent;                     /* 18 IL_STAT queued (boot + piggyback + quiet) */
    uint16_t stats_piggyback;                /* 20 */
    uint16_t stats_quiet;                    /* 22 */
    uint16_t il_rx_bytes;                    /* 24 ilink_uart_stats_t */
    uint16_t il_rx_overflow;                 /* 26 */
    uint16_t il_rx_errors;                   /* 28 */
    uint16_t il_bad_crc;                     /* 30 il_dec_stats_t of the COM's decoder */
    uint16_t il_bad_len;                     /* 32 */
    uint16_t il_bad_type;                    /* 34 */
    uint16_t il_skipped;                     /* 36 */
    uint16_t radio_rx_overflow;              /* 38 radio_stats_t */
    uint16_t radio_rx_errors;                /* 40 */
    uint16_t radio_tx_frames;                /* 42 */
    uint16_t radio_tx_timeouts;              /* 44 */
    uint16_t telem_bad;                      /* 46 ground: TELEM with a wrong header */
    uint16_t telem_out_of_window;            /* 48 ground: TELEM handed to the radio >= reply window after its uplink */
    uint16_t telem_unmatched;                /* 50 ground: TELEM with no uplink record waiting for it (dup / unknown) */
    uint16_t nonce_sent;                     /* 52 ground: LAUNCH frames sent with a nonce read back from TELEM */
    uint16_t events_lost;                    /* 54 bench hook queue overflow (must stay 0) */
} com_cosim_counters_t;                      /* 56 bytes */

typedef struct {
    uint32_t magic;                          /*   0 COM_COSIM_MAGIC */
    uint16_t version;                        /*   4 COM_COSIM_VERSION */
    uint16_t size;                           /*   6 sizeof(com_cosim_report_t) as built */
    uint16_t seqlock;                        /*   8 odd while the COM updates the report */
    uint8_t  radio;                          /*  10 1 lora_uart, 2 sx127x (params_gen.h FC_RADIO_*) */
    uint8_t  done;                           /*  11 1 = COM_SB_SECONDS reached (the COM sleeps next) */
    uint32_t t_us;                           /*  12 COM time of the last update */
    uint32_t ticks;                          /*  16 COM ticks at the last update */
    uint32_t passes;                         /*  20 main-loop passes */
    uint16_t pass_max_us;                    /*  24 longest main-loop pass since boot (Timer1, 0.5 us resolution) */
    uint16_t stack_free_min;                 /*  26 never-touched stack bytes, last complete scan (0xFFFF none yet) */
    uint16_t sec_max_us[6];                  /*  28 [0] unused (0), [i] longest section i (COM_SB_MARK i-1 -> i) */
    uint16_t pass_max_s_us[COM_COSIM_SECONDS];   /* 40 longest pass in second s of COM time */
    uint8_t  ground_state;                   /*  72 ONB_* of the latest TELEM (0xFF: none yet) */
    uint8_t  ground_acked;                   /*  73 1: the latest TELEM acknowledged the ground's latest SEQ */
    uint8_t  reset_cause;                    /*  74 COM watchdog_reset_cause() at boot */
    uint8_t  reserved0;                      /*  75 */
    uint16_t reply_window_ticks;             /*  76 COM_REPLY_WINDOW_TICKS */
    uint16_t tick_hz;                        /*  78 COM_TICK_HZ */
    com_cosim_counters_t c;                  /*  80 */
    uint16_t ul_count;                       /* 136 uplink records written (= uplinks started) */
    uint16_t stat_count;                     /* 138 IL_STAT records written */
    com_cosim_uplink_t ul[COM_COSIM_UPLINKS];    /* 140 */
    com_cosim_stat_t st[COM_COSIM_STATS];        /* 780 */
} com_cosim_report_t;                        /* 908 bytes */

#define COM_COSIM_REPORT_BYTES 908u

/* layout checks (compile on the AVR and on the host): a mismatch is a negative array size */
typedef char com_cosim_check_uplink[(sizeof(com_cosim_uplink_t) == 20u) ? 1 : -1];
typedef char com_cosim_check_stat[(sizeof(com_cosim_stat_t) == 8u) ? 1 : -1];
typedef char com_cosim_check_counters[(sizeof(com_cosim_counters_t) == 56u) ? 1 : -1];
typedef char com_cosim_check_report[(sizeof(com_cosim_report_t) == COM_COSIM_REPORT_BYTES) ? 1 : -1];
typedef char com_cosim_check_off_t[(offsetof(com_cosim_report_t, t_us) == 12u) ? 1 : -1];
typedef char com_cosim_check_off_ground[(offsetof(com_cosim_report_t, ground_state) == 72u) ? 1 : -1];
typedef char com_cosim_check_off_c[(offsetof(com_cosim_report_t, c) == 80u) ? 1 : -1];
typedef char com_cosim_check_off_ul[(offsetof(com_cosim_report_t, ul) == 140u) ? 1 : -1];
typedef char com_cosim_check_off_st[(offsetof(com_cosim_report_t, st) == 780u) ? 1 : -1];

#endif /* FC_COM_COSIM_H */
