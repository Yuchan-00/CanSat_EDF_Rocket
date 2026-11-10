/* com_simbench.h - measurement of the COM main loop in the simavr ATmega128 simulator, -D FC_SIMBENCH (v0.9 RP1: the
 * VTOL bench's synthetic FCU cycles its snapshots through the three power builds - battery (0, 0: TELEM v3 / v2), wired
 * and wired_buffer (the power tail: TELEM v5 / v4) - and sends every 4th one as a SNAP2 (an ONB_NO_GPS FCU), so one run
 * exercises every reply the COM can build: TELEM v3 / v5 from SNAP3, v2 / v4 from SNAP2; the ground checks each TELEM's
 * TYPE and length against the snapshot that produced it.  A real vehicle's FCU has one build; the COM keeps no state
 * between snapshots, so the cycling covers the three builds' paths in one image.
 * v0.7: the VTOL
 * COM - the plain envs, params.json mission.mode = vtol - gets SNAP2 from its synthetic FCU and sends TELEM2; its
 * synthetic ground sends ON with the nonce of the latest TELEM2 instead of ARM / LAUNCH, and the synthetic FCU models
 * ON -> SPOOL (ONB_V_SPOOL_TICKS) -> CLIMB at 1 m/s -> HOLD at the hover target; the *_ballistic envs are the v0.6.3
 * bench):
 *   single chip  envs com_simbench (lora_uart) / com_simbench_sx127x: synthetic ground AND synthetic FCU,
 *                ASCII console on USART0 (the ilink USART is bypassed), run with
 *                firmware/atmega128_fc/tools/run_com_simbench.py;
 *   co-sim       envs com_cosim / com_cosim_sx127x (+ -D FC_COSIM): synthetic ground only, the real
 *                ilink USART0 wired to a real FCU (env fcu_cosim) by a two-chip harness, no console - the
 *                report is the RAM struct of src/com/com_cosim.h.
 *
 * The COM firmware runs unchanged; only its peers are synthetic, and they run in com_sb_pass() BEFORE
 * each main-loop pass, outside the measured time:
 *
 *   ground  the v0.5 synthetic ground station: a CMD frame (core/link_proto.c encoding) every
 *           LINK_RTT_WORST_MS from SB_FIRST_POLL_TICKS on, NONE as heartbeat (same SEQ), ARM after SB_ARM_S,
 *           LAUNCH with the nonce read back from the COM's own TELEM after SB_LAUNCH_S; the bytes go into
 *           the radio driver's receive ring (radio_inject_rx) at the link byte rate - lora_uart: one byte
 *           per 10 / LINK_UART_BAUD s, sx127x: the whole packet at once (RxDone).  At SB_BURST_S it sends
 *           SB_BURST_N NONE frames back to back (a stress pattern the real ground never sends) so the
 *           "newest reply wins" path runs; from SB_QUIET_S it is silent for SB_QUIET_TICKS so the COM's
 *           quiet IL_STAT divider runs (D4).  Its receiver reads every TELEM the COM hands to the radio
 *           and checks that it answers an uplink that completed less than the reply window ago and that
 *           no uplink gets two (D5, seen from outside).
 *   FCU     (single chip only) takes the COM's ilink TX bytes at ILINK_BAUD (ilink_uart_take_tx), decodes
 *           them with core/ilink_proto.c (IL_ACCEPT_FCU, as the real FCU), checks every IL_CMD against the
 *           frame the ground sent and every IL_STAT (params tag, backend, BOOT only in the first, counters)
 *           and its pacing (D4: each STAT is the boot STAT, right behind an IL_CMD, or quiet; no IL_CMD
 *           starts within one FCU tick after a STAT; gaps; R3: the boot STAT's com_ticks and the real time
 *           the COM queued it), and answers the IL_CMDs of one COM tick with ONE IL_SNAP at the next COM
 *           tick (one FCU tick later; the FCU's own compute time is not modelled) from a small supervisor
 *           model (SAFE -> ARMED -> POWERED, SEQ / repeat / nonce rules of core/onboard.c), with a varying
 *           attitude quaternion; the SNAP bytes go into the ilink receive ring at ILINK_BAUD
 *           (ilink_uart_inject_rx).  It misbehaves on purpose three times (D5): a stray SNAP whose
 *           reply_seq was never forwarded (SB_STRAY_S), one reply sent twice (SB_DUP_S), one reply held
 *           until the reply window is over (SB_LATE_S) - the COM must refuse all three; and once (R4,
 *           SB_WRONG_S) it sends a valid IL_CMD and a valid IL_STAT, the COM -> FCU types, which the COM's
 *           decoder (IL_ACCEPT_COM) must reject in the header without losing step.
 *
 * Measured with Timer1 (0.5 us): the main-loop pass (max per one-second window, and per section),
 * uplink frame complete at the COM -> IL_CMD queued, SNAP complete at the COM -> TELEM handed to the
 * radio, uplink complete -> TELEM handed to the radio, the stack high-water mark.  Times of the
 * synthetic bytes are their scheduled arrival times, so a latency includes waiting for the pass in
 * progress.  Single chip: the bench clock stands still while com_sb_pass runs, so the peers' own work
 * (sinf / cosf for a SNAP, formatting a report) adds nothing to a schedule or a latency; the console
 * ISR does run during measured passes (~one interrupt per 40 us while a report drains), a stand-in for
 * the ilink RX interrupts the bypass removes.  Co-sim: the clock is real (the FCU does not stand still),
 * so the ground's own work delays the COM like any other load outside the measured pass, and the real
 * ilink RX / UDRE interrupts run inside it.  Not simulated: the radio module (lora_uart: no RX ISR, the
 * TELEM really leaves through USART1; sx127x: no chip, DIO0 never rises - the FIFO read steps after
 * RxDone do not run and every TELEM ends in the TX-timeout path, SX127X_TX_TIMEOUT_TICKS = 208 ms; so in
 * the sx127x burst the newest reply waits for the radio past the reply window and is dropped - the D5 expiry path - where
 * a real SX127x would have raised TxDone after the TELEM's ~77 ms on air).
 * Single-chip console (USART0) lines: "SB BOOT ...", once per second "SB t=.. pm=.. ..." (timing),
 * "SL t=.. up=.. ..." (counters, checks), "SD t=.. ..." (D4 / D5 counters) and "SI t=.. ..." (both ilink
 * decoders' counters, the boot STAT's time), per TELEM "SBT s=.. n=<SNAP payload> f=<TELEM frame>" (hex)
 * for the host check against sim/ilink_proto.py, per wrong-direction frame "SBW f=<frame>" (hex), and
 * "SB END" before the CPU sleeps with interrupts off (simavr exits).  Every line stays below simavr's
 * 256-character console line.
 *
 * Invariant: compiled to nothing unless FC_SIMBENCH is defined; the measured pass contains only the
 * flight code plus the section marks and the hooks (an event timestamp each, one 36-byte TELEM copy).
 */
#ifndef FC_COM_SIMBENCH_H
#define FC_COM_SIMBENCH_H

#include <stdint.h>

#ifdef FC_SIMBENCH
#include "../params_gen.h"                   /* FC_LINK_RADIO */

#ifndef COM_SB_SECONDS
#define COM_SB_SECONDS 15u                   /* run length (co-sim: build the FCU with the same FC_SIMBENCH_SECONDS) */
#endif
#define SB_FIRST_POLL_TICKS 50u              /* the ground's first uplink 0.2 s after boot (after the boot STAT at 0.1 s) */
#define SB_ARM_S 3u                          /* the synthetic ground arms after this many seconds ... */
#define SB_LAUNCH_S 4u                       /* ... and launches after this many */
#define SB_STRAY_S 5u                        /* single chip: the synthetic FCU sends a stray SNAP ... */
#define SB_DUP_S 6u                          /* ... one reply twice ... */
#define SB_LATE_S 7u                         /* ... and one reply after the reply window */
#define SB_WRONG_S 8u                        /* R4: an IL_CMD and an IL_STAT towards the COM (wrong direction) */
#define SB_BURST_S 9u                        /* back-to-back uplink burst (stress) at this second */
/* frames in the burst.  lora_uart: 4 (13.5 ms apart; the radio is busy 37.5 ms per TELEM, so older replies are
 * superseded and the newest one goes out ~30 ms after its IL_CMD).  sx127x: 3 (48 ms apart; with no TxDone in simavr the
 * radio holds the first TELEM for its 208 ms TX timeout, so the newest reply would leave ~118 ms after its IL_CMD - well
 * past the 80 ms reply window: the bench sees the D5 expiry clearly, not at the window's edge) */
#if FC_LINK_RADIO == FC_RADIO_SX127X
#define SB_BURST_N 3u
#elif FC_LINK_RADIO == FC_RADIO_ESP_BRIDGE
/* 2026-11-10: the ESP's 38400-baud UART is busy only ~12 ms per TELEM (46 B), so 4 frames ~4 ms apart superseded an
 * older reply at most once (2026-11-06 bench: 1) and, after the synthetic ground's poll period moved by 5 ms (params_gen.h
 * LINK_RTT_WORST_MS 486 -> 491: the TELEM v7 byte), not at all; 6 frames (~24 ms) always do - the burst check is unchanged */
#define SB_BURST_N 6u
#else
#define SB_BURST_N 4u
#endif
#ifndef SB_QUIET_S
#define SB_QUIET_S 11u                       /* the ground is silent from this second ... */
#endif
#define SB_QUIET_TICKS 500u                  /* ... for 2 s (> COM_STAT_QUIET_TICKS 1.5 s: quiet STATs) */
#define COM_SB_SECTIONS 6u                   /* marks 0..5 */

/* hook events (single chip and co-sim) */
#define COM_SB_EV_CMD 1u
#define COM_SB_EV_SNAP_OK 2u
#define COM_SB_EV_SNAP_REFUSED 3u
#define COM_SB_EV_STAT 4u
#define COM_SB_EV_TELEM 5u
#define COM_SB_EV_EXPIRED 6u

void com_sb_init(uint8_t reset_cause);
void com_sb_pass(void);                                   /* synthetic peers, before the measured pass */
void com_sb_mark(uint8_t i);
void com_sb_end_of_pass(uint16_t pass_us, uint16_t stack_free);
void com_sb_event(uint8_t kind, uint8_t seq);             /* hook: timestamp + kind + seq into a small queue */
void com_sb_telem_sent(const uint8_t *frame, uint8_t n);  /* hook: radio_send accepted a TELEM (event + copy) */

#define COM_SB_MARK(i) com_sb_mark(i)
#define COM_SB_CMD_QUEUED(seq) com_sb_event(COM_SB_EV_CMD, (seq))
#define COM_SB_SNAP(seq, ok) com_sb_event((ok) ? COM_SB_EV_SNAP_OK : COM_SB_EV_SNAP_REFUSED, (seq))
#define COM_SB_STAT_SENT(kind) com_sb_event(COM_SB_EV_STAT, (kind))
#define COM_SB_TELEM_SENT(f, n) com_sb_telem_sent((f), (n))
#define COM_SB_TELEM_EXPIRED(seq) com_sb_event(COM_SB_EV_EXPIRED, (seq))
#else
#define COM_SB_MARK(i) ((void)0)
#define COM_SB_CMD_QUEUED(seq) ((void)0)
#define COM_SB_SNAP(seq, ok) ((void)0)
#define COM_SB_STAT_SENT(kind) ((void)0)
#define COM_SB_TELEM_SENT(f, n) ((void)0)
#define COM_SB_TELEM_EXPIRED(seq) ((void)0)
#endif

#endif /* FC_COM_SIMBENCH_H */
