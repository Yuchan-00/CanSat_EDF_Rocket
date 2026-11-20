/* simbench.h - cycle measurement of the FCU's real 250 Hz loop in the simavr ATmega128 simulator
 * (PlatformIO envs "fcu_simbench_ballistic" / "fcu_simbench_micro_ballistic" / "fcu_cosim_ballistic", -D FC_SIMBENCH;
 * v0.6.3: the rocket's ballistic mission).  The VTOL profile (its synthetic operator and vertical plant, the "SV" report
 * lines and the VTOL fields of sb_tick_t) went with the VTOL program on 2026-11-18 / 2026-11-19.
 *
 * The firmware is unchanged except that its *peers* are synthetic: the battery ADC and the arm switch return fixed
 * values, and (single-chip bench) the NAV chip and the COM chip are emulated behind the HAL bypasses.
 *
 * ---- single-chip bench (fcu_simbench, fcu_simbench_micro: FC_SIMBENCH without FC_COSIM) ----
 * The simbench keeps its own clock (Timer1 counts since simbench_start, 32 bit) for both peers.
 *
 * A synthetic NAV stands on the nlink (hal/nlink_spi.c single-chip bypass: no SPI register, no ISR).  It runs its own
 * 250 Hz tick (NAV tick k at k * FC_TICK_US) and completes the navigation frame of tick k SB_NAV_FRAME_US after it
 * (NAV_TICK -> push at ~1.5 ms + the NLINK_XFER_US exchange), up to SB_NAV_JITTER_US earlier from a fixed pattern (its
 * time-critical part varies tick to tick); the frame is injected (nlink_spi_inject_frame) when that time has come, at
 * the FCU's polls (the wait loop and every background job).  The content is a real nl_encode_nav frame of a synthetic
 * trajectory, state-dependent like the NAV's (sim/nlink_proto.py nav_pad_mode on the FCU frame of the previous exchange,
 * nlink_spi_last_reply): pad mode - still, slightly tilted, altitude 0, NL_F_ALIGNED | NL_F_EST_VALID after
 * SB_NAV_ALIGN_TICKS pad ticks, NL_F_BOOT until a valid FCU frame came; flight mode - oscillating attitude and rates
 * (1.3 / 0.9 / 0.5 Hz, < 7 deg tilt), a thrust specific force, climbing 3 m/s.  Faults, at NAV ticks:
 *   SB_NAV_BAD_TICK      one frame with a broken CRC (the FCU's tick starts, the navigation data are invalid)
 *   SB_NAV_MISS_TICK     one frame missing (the FCU runs one tick on its frame timeout)
 *   SB_NAV_GAP_TICK      SB_NAV_GAP_TICKS frames missing (timeout, then the FCU's own Timer2 clock)
 *   SB_NAV_RESET_TICK    the NAV resets in flight: SB_NAV_RESET_TICKS silent, then frames with NL_F_BOOT and without
 *                        NL_F_EST_VALID, SEQ from 0 -> imu_ok false -> I9 -> ABORT ONB_IMU_FAIL_TICKS after the
 *                        silence started (nlink_proto.h)
 * A synthetic COM stands on the ilink (hal/ilink_uart.c single-chip bypass: no USART register, no ISR).  It
 *   - sends real IL_CMD frames at the ground's poll rate: NONE heartbeats, ARM after SB_ARM_S, LAUNCH after SB_LAUNCH_S
 *     with the nonce read back from the FCU's own SNAPs, each repeated until a SNAP acknowledges its SEQ;
 *   - sends IL_STAT as the COM does (D4): right behind an IL_CMD when >= 1 / stat_hz s has passed, plus the boot STAT;
 *     every SB_STAT_FRONT_EVERY-th poll one STAT right in FRONT of the CMD (stress) - those CMDs are reported apart;
 *   - babbles twice in flight, no frames meanwhile (< ONB_LINK_ABORT_TICKS each: a lost link to the supervisor, the
 *     flight goes on): from SB_SLOW_BABBLE_S for SB_SLOW_BABBLE_TICKS the v0.6 phase-3 sub-flood pattern C5 5C 08 01 (an
 *     IL_CMD header: the decoder's candidate can always still be an IL_CMD) one byte every SB_SLOW_BABBLE_US (R1: the log
 *     packing must not starve); from SB_BABBLE_S for SB_BABBLE_TICKS the wire full at ILINK_BAUD (C5 5C 31 02 sync spam,
 *     then noise; 100 bytes a tick: D6 flood, the RX ring overflows, the background ilink share is capped);
 *   - models the wire: every frame is scheduled to END at a chosen phase before an expected navigation frame (the FCU's
 *     tick start; k_cmd_end_us, down to 20 us), its bytes enter the RX ring one by one at their ILINK_BAUD 8N1 arrival
 *     times - those that arrived before a tick start at the tick start (step 2's bytes), the others at the polls;
 *   - takes the FCU's SNAP frames out of the TX ring (header + CRC checked) and measures, for every CMD, the latency from
 *     the first FCU tick that starts after its last byte to the tick whose supervisor answered it (v0.6 requires 0).
 * The bypasses have no interrupts: the synthetic COM spends the RX ISR's cycles (SB_ISR_CYCLES = 51 a byte,
 * hal/ilink_uart.c, checked against the fcu ELF by tools/run_simbench.py) where it puts the bytes into the ring; the
 * nlink slave ISR (~4.5k cycles per exchange, hal/nlink_spi.h) runs while the FCU waits for the frame, which the bench
 * checks (the background ends before the next exchange can start: margin "bm").  The time-critical figure stays
 * interrupt-free and run_simbench.py adds the interrupts analytically (ilink RX, the log UART's UDRE, counted from the
 * disassembly).  The ilink UDRE (TX) interrupt is held off during the time-critical part (ilink_uart_tx_hold).
 *
 * ---- co-simulation build (fcu_cosim: FC_SIMBENCH + FC_COSIM) ----
 * Battery / switch stay synthetic, the report lines and the micro-benchmarks (minus the main_ilink_* ones) stay; the
 * synthetic NAV and COM are compiled out and the REAL nlink SPI slave and ilink USART1 run (hal/nlink_spi.h FC_COSIM
 * notes for the harness: PB0 driven as an EXTERNAL level, the slave's SPDR byte taken when the master writes its own).
 * Note for a harness: simavr 1.7's USART delivers received bytes every 11 bit times (704 CPU cycles at 250 kbaud, not
 * 640): a synthetic peer must feed at that pace or simavr's 64-byte input FIFO drifts and overruns.  Per tick:
 *   PORTA (unused on the FCU board) = (tick << 1) | 1 when the tick starts (the frame was noticed or the timeout passed,
 *          before step 2 counts the waiting ilink bytes), and = tick << 1 at the end of the time-critical part.
 *          simavr's IOPORT 'A' IRQ gives the CPU cycle of both edges: PA0 high = the time-critical part INCLUDING the
 *          real interrupts; PA7..PA1 = the tick's low 7 bits.  DDRA = 0xFF from boot (simbench_cosim_init).
 *   "SC" line (ASCII on the log console USART0, 57600 8N1) for every tick whose supervisor got IL_CMD frames:
 *              \nSC t=<tick> n=<frames> q=<first seq> r=<last seq> w=<bytes waiting at the tick>
 *              b=<of them decoded in step 2> c=<time-critical part, us (Timer1)>\n
 * Report lines, every number hex "0x..", one line < 216 bytes, the same in both builds unless marked:
 *   "SB t= st= cm= cc= tm= ov= sf= ct= bg= bk= sn= il= ox= o1a= o3a= o3b= o3c= s=<7 sections>"   1 Hz: time-critical
 *          part max (cm; cc with the controller active), incl. background (tm), ticks without a valid navigation frame,
 *          never-touched stack, control ticks, longest background job / its kind / SNAP job / ilink drain, the simbench's
 *          own work, the compare registers, the sections' maxima (us);
 *   "SN t= st= fv= fb= ft= fq= rs= fo= lm= pw= pc= la= lc= bm= f1= fp= fn= nr= at= ar= nl="   0.25 s after SB: the nlink -
 *          valid frames, bad frames, timeout ticks, NAV ticks lost (SEQ gaps), SPI resyncs / overruns (cumulative); the
 *          tick start's lateness after the frame (lm), tick start -> last vane register write (pw; pc with the controller
 *          active), NAV tick -> last vane register write (la; lc with the controller active), the background's margin to
 *          the next exchange (bm, signed, min) (window extremes, us); single-chip: the first fallback tick after a frame
 *          (f1, max) and the fallback period (fp max, fn min, us), the FCU tick of the NAV reset, of the first ABORT and
 *          its reason (cumulative), frames the simbench built late (nl);
 *   "SL t= st= nw= nb= cx= sd= fd= bl= [up= rp= sa= ua= bs= rn= rl= rs= rt=]"   0.5 s after SB: ilink bytes waiting at a
 *          tick / decoded in step 2 (window max), ticks an IL_CMD could not complete within ILINK_CRIT_BYTES, SNAPs
 *          replaced, flight records dropped, ticks with a valid frame without NL_F_BARO_OK; [single-chip only: the
 *          synthetic COM's counts and latencies, see run_simbench.py];
 *   "SI t= st= ic= is= ib= ro= re= bx= fl= bb= bj= bt= ed= [ub= bz= sz= | sl=]"   0.75 s after SB: IL_CMDs to the
 *          supervisor, SNAPs queued, decoder rejects, RX ring overflow, RX errors, ticks the background used up
 *          ILINK_BG_BYTES, ticks after an RX ring overflow (all cumulative); most bytes the background decoded ahead of
 *          the jobs in one tick, longest job started on BG_RESERVE_US / a text job (window maxima, us); event / link
 *          records dropped; [single-chip: CMDs lost right after a babble, fast / slow babble bytes | co-simulation: SC
 *          lines lost];
 *   "SB END" before the CPU sleeps with interrupts off (simavr exits).
 * tools/run_simbench.py parses them.
 *
 * Invariant: compiled to nothing unless FC_SIMBENCH is defined; the synthetic sources replace only inputs, never a
 * line of the measured flight code (the co-simulation's tick marker adds two port writes per tick to its own build).
 */
#ifndef FC_SIMBENCH_H
#define FC_SIMBENCH_H

#if defined(FC_COSIM) && !defined(FC_SIMBENCH)
#error "FC_COSIM is a variant of the simbench: build it with -D FC_SIMBENCH -D FC_COSIM (env fcu_cosim)"
#endif

#if defined(FC_SIMBENCH) && defined(FC_COSIM)
#include <avr/io.h>
#define SB_COSIM_TICK(t) (PORTA = (uint8_t)((uint8_t)((uint8_t)(t) << 1) | 1u))
#define SB_COSIM_CRIT_END(t) (PORTA = (uint8_t)((uint8_t)(t) << 1))
#else
#define SB_COSIM_TICK(t) ((void)0)
#define SB_COSIM_CRIT_END(t) ((void)0)
#endif

#ifdef FC_SIMBENCH
#include <stdbool.h>
#include <stdint.h>

#include "fc_mode.h"
#include "core/attitude_ctrl.h"
#include "core/ilink_proto.h"
#include "core/log_record.h"
#include "core/onboard.h"

/* marks 0..7: sections 1..7 = NAV frame + arm switch, ilink RX, supervisor, controller, vanes written (the end of the
 * frame -> vane path), ESC / release written + anchor / battery / records / replies / LED / loop stamp, background */
#define SB_SECTIONS 8
/* ---- the flight profile (single chip, v0.6.3, unchanged): ARM, LAUNCH, powered climb, two babbles, the NAV reset in
 * flight -> I9 ABORT.  The suffixed names are what tools/run_simbench.py reads (--mode ballistic; the VTOL profile went
 * with the VTOL program on 2026-11-18). ---- */
#define SB_SECONDS_BALLISTIC 12u
#define SB_SLOW_BABBLE_S_BALLISTIC 5u        /* single-chip: sub-flood babble (R1) from this second ... */
#define SB_SLOW_BABBLE_TICKS_BALLISTIC 500u  /* ... for 2 s (< ONB_LINK_ABORT_TICKS) ... */
#define SB_BABBLE_S_BALLISTIC 8u             /* single-chip: the synthetic COM floods from this second ... */
#define SB_BABBLE_TICKS_BALLISTIC 375u       /* ... for 1.5 s (< ONB_LINK_ABORT_TICKS: the flight goes on) */
#define SB_NAV_RESET_TICK_BALLISTIC 2500u    /* 10.0 s: the NAV resets in flight */
#define SB_SECONDS_DEFAULT SB_SECONDS_BALLISTIC
#define SB_SLOW_BABBLE_S SB_SLOW_BABBLE_S_BALLISTIC
#define SB_SLOW_BABBLE_TICKS SB_SLOW_BABBLE_TICKS_BALLISTIC
#define SB_BABBLE_S SB_BABBLE_S_BALLISTIC
#define SB_BABBLE_TICKS SB_BABBLE_TICKS_BALLISTIC
#ifndef FC_SIMBENCH_SECONDS
#define FC_SIMBENCH_SECONDS SB_SECONDS_DEFAULT
#endif
#define SB_ARM_S 3u                  /* ballistic: the synthetic ground arms after this many seconds ... */
#define SB_LAUNCH_S 4u               /* ... and launches after this many */
#define SB_SLOW_BABBLE_US 300u       /* slow babble: one byte of C5 5C 08 01 every 300 us */
#define SB_STAT_FRONT_EVERY 8u       /* single-chip: every 8th poll a STAT right in front of the CMD */
/* the synthetic NAV (single-chip), NAV ticks of 4 ms from simbench_start.  NAV tick -> its exchange starts (sensors +
 * estimator + encode): v0.6.3's 1500 us */
#define SB_NAV_PUSH_US_BALLISTIC 1500u
#define SB_NAV_PUSH_US SB_NAV_PUSH_US_BALLISTIC
#define SB_NAV_FRAME_US (SB_NAV_PUSH_US + NLINK_XFER_US)   /* NAV tick -> the frame's last byte at the FCU */
#define SB_NAV_JITTER_US 250u        /* frames complete up to this much earlier (pattern in simbench.c) */
#define SB_NAV_FIRST_TICK 5u         /* the NAV's first frame (the FCU runs on its own clock before) */
#define SB_NAV_ALIGN_TICKS 520u      /* pad ticks until NL_F_ALIGNED | NL_F_EST_VALID (CAL_PAD_STILL_TICKS + window) */
#define SB_NAV_BAD_TICK 1150u        /* 4.6 s: one frame with a broken CRC */
#define SB_NAV_MISS_TICK 1825u       /* 7.3 s: one frame missing */
#define SB_NAV_GAP_TICK 1900u        /* 7.6 s: ... */
#define SB_NAV_GAP_TICKS 5u          /* ... five frames missing */
#define SB_NAV_RESET_TICKS 15u       /* the NAV reset (at SB_NAV_RESET_TICK_BALLISTIC):
                                      * silent for 60 ms */

/* one tick's figures, filled by main.c after the background (cumulative counters as they stand).  The fields up to
 * rx_errors keep the v0.6 layout (tools/run_cosim.py reads them by name through its layout probe); ph_baro is 0 and
 * baro_late counts valid frames without NL_F_BARO_OK since v0.6.3 */
typedef struct {
    uint32_t tick;
    uint8_t  state;
    uint8_t  ph_baro;
    uint16_t loop_us;                /* time-critical part, us */
    uint16_t total_us;               /* incl. the background, us */
    uint16_t overruns;               /* ticks without a valid navigation frame */
    uint8_t  n_wait, n_crit;         /* ilink bytes waiting at the tick / decoded in step 2 */
    uint8_t  n_cmds, seq_first, seq_last;   /* IL_CMD frames this tick's supervisor got (0: none) */
    uint8_t  bg_bytes;               /* bytes the background decoded ahead of the other jobs this tick */
    uint16_t crit_over, bg_capped, flood_ticks, snap_dropped, flog_dropped, baro_late;
    uint16_t il_cmds, il_snaps, il_bad, rx_overflow, rx_errors;
    /* v0.6.3 */
    uint16_t pwm_us;                 /* tick start -> the last vane compare register written, us */
    uint8_t  wait;                   /* main.c TICK_FRAME_SEEN 0 / TICK_FRAME_LATE 1 / TICK_TIMEOUT 2 */
    uint8_t  nav_ok;                 /* the tick ran on a valid navigation frame */
    uint8_t  reason;                 /* supervisor reason code */
    uint16_t nl_frames, nl_bad, nl_timeouts, nl_seq_lost, nl_resyncs, nl_overruns, evq_dropped;
    uint16_t bg_end_us;              /* the tick's anchor (its frame) -> the background's end, us (main.c BG_END_US) */
    uint16_t status_written;         /* v0.7: 1 Hz status records written since boot (cumulative) */
} sb_tick_t;

/* per-operation cycle counts, printed as SBM lines; borrows main's objects before they are initialised */
void     simbench_micro(attitude_ctrl_t *c, ctrl_output_t *o, onboard_t *sup, onb_outputs_t *so, log_flight_t *rec,
                        uint8_t *pay, uint8_t *fr);
#ifdef FC_SIMBENCH_MICRO
void     simbench_micro_line(const char *name_P, uint32_t cycles);   /* one "SBM name=cycles" line (main.c's own paths) */
uint8_t *simbench_scratch(void);     /* 96 bytes the boot-time benchmarks may use (the report buffer, idle before the loop) */
/* ilink / log records, after simbench_micro; borrows main's ilink decoder (main re-initialises it) */
void     simbench_micro_ilink(onb_outputs_t *so, uint8_t *pay, uint8_t *fr, il_decoder_t *dec);
void     simbench_micro_nlink(uint8_t *pay, uint8_t *fr);   /* nlink frames / SPI bypass, after simbench_micro_ilink */
#endif
uint16_t simbench_adc(void);
bool     simbench_arm_switch(void);
void     simbench_mark(uint8_t i);
/* one background job's duration; kind = main.c BG_* (1 SNAP, 2 ilink drain, 3 'N' record, 4 record, 5 flight log,
 * 6 'C' record, 7 COM text, 8 NAV text - the text jobs are started on their own reserve, COM_TEXT_RESERVE_US) */
void     simbench_bg_step(uint16_t us, uint8_t kind);
/* fw09 integration (section 7 row 14): main.c ORs SB_BG_LATE into kind for a job of the VTOL late pass (g_late: started
 * on LATE_WALL(reserve) before LATE_END_US, stretched by the exchange's slave interrupts) - the reserve a job must fit
 * differs between the two passes */
#define SB_BG_LATE 0x80u
/* v0.7: the anchor-relative time at which a piece of the FCU's background work ended (after every job, the stack-scan
 * slice and the status record): the window maximum of the latest end is "je" in the SL line - the FCU's own work, without
 * the bench's poll and its lumped RX-interrupt emulation that follow the last job */
void     simbench_bg_work_end(uint16_t anchor_us);
/* after the background: the peers' turn (single-chip), the report lines */
void     simbench_end_of_loop(const sb_tick_t *t);
#if !defined(FC_COSIM)
void     simbench_start(void);                       /* the peers' clock starts (right before the loop) */
void     simbench_peers(void);                       /* the NAV's frame and the COM's bytes that are due by now */
void     simbench_tick_start(uint32_t tick, bool nav);   /* the tick started (nav: on an exchange): timing, COM bytes */
#else
void     simbench_cosim_init(void);                  /* the tick marker port: DDRA = 0xFF, PORTA = 0 */
#endif
#endif

#endif /* FC_SIMBENCH_H */
