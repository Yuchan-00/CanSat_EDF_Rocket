/* gns_simbench.h - cycle measurement of the GNS board's real main loop in the simavr ATmega128 simulator (v0.8.1;
 * PlatformIO envs "gns_simbench" / "gns_simbench_demo" (single chip) and "gns_cosim" (two-core harness), -D FC_SIMBENCH).
 * tools/run_gns_simbench.py builds, runs and judges the single-chip bench (host_test/build/gns_simbench_report.json).
 *
 * The GNS firmware is unchanged except for its PEERS (never a line of the measured code):
 *   - a synthetic u-blox NEO-6M on the GPS UART (both builds: simavr has no receiver on RXD1) - the v0.8 NAV bench's
 *     receiver, moved here with the GPS: factory 9600 baud with NMEA; parses the frames the GNS queues (hal/gps_uart.c's
 *     copies) and answers RCV_RESP_TICKS later at its own baud (CFG-PRT for another baud -> switches, no ACK; the other CFG
 *     frames -> ACK-ACK; MON-VER poll -> a 100-byte MON-VER of a u-blox 6); once a NAV message is enabled and the rate is
 *     200 ms: one epoch every 200.1 ms (500 ppm against the GNS's clock: the epoch phase sweeps the 4 ms poll) - NAV-POSLLH +
 *     NAV-SOL + NAV-VELNED back to back, 3D fix, 9 satellites, hAcc 1.8 m, sAcc 0.25 m/s, the antenna on a 0.5 m / 0.3 m
 *     ellipse of 20 s period - delivered byte by byte by a Timer3 compare interrupt at the receiver's 8N1 byte time
 *     (hal/gps_uart.h bench wire; the flight build's USART1 RX interrupt is counted from its disassembly).  Every number is
 *     a bench input, not a model of the receiver's accuracy (ASSUMED);
 *   - gns_simbench (single chip, FC_SIMBENCH without FC_COSIM): a synthetic NAV on the glink - no USART0 access (USART0 is
 *     the console), hal/glink_uart.c's bypass: the STATE frame of NAV tick 5 m + 4 is injected into the RX ring by a Timer1
 *     compare interrupt - bytes 0..58 at OCR1A, the last byte GLINK_STATE_FRAME_US later at OCR1B (the frame's last byte
 *     completes then: the GNS decodes the first 59 as they come, as with the real RX interrupt) - once per Timer1 period
 *     (20 ms = one cycle), the phase moving GNS_SB_PHASE_STEP counts a cycle so the frame's completion sweeps the GNS's 4 ms
 *     poll (and the epochs).  The frames are gl_encode_state frames of a synthetic flight (below); the RESULT frames the GNS
 *     queues are taken from the TX ring and checked (sync / TYPE / CRC by a glink decoder, SEQ = the STATE's, TILT engaged
 *     in HOLD once the GPS gate is passed).  The RX interrupts of the 60 STATE bytes are not in the measured figures:
 *     run_gns_simbench.py adds them from the flight ELF's disassembly.
 *   - gns_cosim (FC_SIMBENCH + FC_COSIM): the synthetic receiver only; the REAL glink USART0 (a simavr harness wires a NAV
 *     to it - the glink latency measurement, from the bytes on the two wires); no console: the figures in the RAM struct
 *     g_gns_cosim_report (avr-nm; layout in gns_simbench.c).
 *     v0.9 closed loop (envs gns_cosim_cl / gns_cosim_cl_demo, tools/run_cosim4.py): the four-core harness writes the
 *     plant's GPS antenna into the RAM block g_cosim_gps (avr-nm; layout in gns_simbench.c: magic, seq, cut, north / east
 *     mm, v_north / v_east mm/s, up mm) and the receiver reports it in its epochs instead of the ellipse; cut = 1 (the
 *     harness's gps_cut fault) silences the receiver (g_gns_cosim_report.gps_cut_epochs counts the epochs not sent).
 *     Without the block's magic the receiver flies its ellipse as before.
 *
 * Synthetic flight (gns_simbench; NAV ticks of the STATE frames): SAFE on the pad (GL_NF_PAD, h 0, the pad's 1 g) until
 * GNS_SB_EST_S, EST_VALID from then; ON at GNS_SB_ON_S: POWERED CLIMB (pflags motor_on, lifted after 1 s) to 1.5 m at
 * 0.5 m/s, HOLD (sat_hi in every 8th cycle), LAND at GNS_SB_LAND_S, LANDED at h 0; the attitude oscillates (1.3 / 0.9 /
 * 0.5 Hz, < 3 deg), the body specific force is the hover's (|f| ~ g).  Faults: GNS_SB_GAP_CYCLES STATE frames missing at
 * GNS_SB_GAP_S (the gap rule: hold steps + propagation), one STATE without GL_NF_IMU_OK at GNS_SB_IMUBAD_S (one hold step,
 * the previous RESULT again), one STATE with a broken CRC at GNS_SB_CRC_S (dropped: no RESULT, a 2-cycle gap next).
 *
 * Report lines (USART0 57600 8N1, every number hex "0x..", one line < 200 bytes):
 *   "GB BOOT rc= sr= gb= gl= fw=" once: reset cause, stack reserve, GLINK_BAUD / 100, GLINK_LATENCY_TICKS, the firmware
 *          version's minor;
 *   "GC t= cy= cn= cm= cs= ca= fm= fs= bm= pm= pn= wk= wt= sf= ra=" 1 Hz: cycles (RESULTs queued), cycles with a measured
 *          completion, the STATE's completion -> RESULT queued max / sum (mean = cs / cn), the cycle of the newest one, the
 *          fusion (gns_finish) max / sum, completion -> fusion done max (bm), the 4 ms poll max / its bytes max, the busy time
 *          (awake) of the last window and its max of a pass, never-touched stack, RESULT frames refused by the TX ring (us);
 *   "GG t= cs= co= vo= md= nk= to= pf= pk= pl= ae= ai= fu= rp= rv= ps= sf=" 0.25 s later: configurator / parser / epochs /
 *          filter (as v0.8's NAV lines SG / SI);
 *   "GU t= ub= ux= ue= tb= tr= bc= gb= gx= ge= gt=" 0.5 s later: the GPS UART and the glink UART counters;
 *   "GN t= ns= nr= nk= nc= nq= ne= nt= nm= ng= hs= gp= rs= is= cl=" 0.75 s later: the synthetic NAV - STATE frames injected,
 *          RESULT frames taken, decoded ok / bad CRC / wrong SEQ, engaged TILTs, TILT parts not NONE in HOLD, position mode
 *          / GATE_OK seen; the GNS's own counters (gns_stats_t: hold steps, gaps, restarts, imu-bad frames, clock slips);
 *   "GJ t= ra= rv= rs= re= rd= rn= rh= rw= rb=" 7/8 s later: the synthetic receiver (as v0.8's SJ);
 *   "GB END" after GNS_SB_SECONDS, then the CPU sleeps with interrupts off (simavr exits).
 *
 * Invariant: compiled to nothing unless FC_SIMBENCH is defined; the synthetic sources replace only inputs.
 */
#ifndef FC_GNS_SIMBENCH_H
#define FC_GNS_SIMBENCH_H

#if defined(FC_COSIM) && !defined(FC_SIMBENCH)
#error "FC_COSIM is a variant of the simbench: build it with -D FC_SIMBENCH -D FC_COSIM (env gns_cosim)"
#endif

#ifdef FC_SIMBENCH
#include <stdbool.h>
#include <stdint.h>

#include "core/gns_node.h"

#ifndef GNS_SB_SECONDS
#define GNS_SB_SECONDS 40u
#endif
#define GNS_SB_EST_S 2u              /* the NAV's attitude valid (pad alignment) from this second */
#define GNS_SB_ON_S 12u              /* ON: POWERED CLIMB (the GPS gate needs its stable time first) */
#define GNS_SB_LAND_S 32u            /* LAND */
#define GNS_SB_GAP_S 20u             /* GNS_SB_GAP_CYCLES STATE frames missing here ... */
#define GNS_SB_GAP_CYCLES 3u
#define GNS_SB_IMUBAD_S 24u          /* ... one STATE without GL_NF_IMU_OK ... */
#define GNS_SB_CRC_S 27u             /* ... one STATE with a broken CRC */
#define GNS_SB_PHASE_STEP 733u       /* Timer1 counts the injection phase moves per cycle (sweeps the 4 ms poll) */

void gns_sb_init(uint8_t reset_cause);
void gns_sb_pass_start(void);                      /* the main loop woke up */
void gns_sb_pass_end(void);                        /* ... and goes to sleep (the pass's busy time) */
void gns_sb_poll(uint8_t n_rx, uint16_t us);       /* the 4 ms poll ran (gps_poll_step: n_rx GPS bytes, us long) */
void gns_sb_result(bool queued);                   /* a cycle's RESULT went into the TX ring (false: refused) */
void gns_sb_finished(const gns_t *g);              /* that cycle's fusion is done */
void gns_sb_tick(uint32_t t, const gns_t *g);      /* after the poll of tick t: the peers' turn, the report lines */
#endif

#endif /* FC_GNS_SIMBENCH_H */
