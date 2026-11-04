/* com_link.h - the COM's link work (v0.6, SPEC 10.3 / 10.4): uplink CMD frames to the FCU,
 * FCU snapshots to TELEM replies, the IL_STAT.  No flight decision is made here.
 *
 * v0.7 (fc_mode.h): the VTOL build's FCU sends IL_SNAP2 (the SNAP + phase + altitude target) and the COM answers with
 * TELEM v2 (39-byte frame: + phase u8, h_tgt i16 in 0.1 m; core/link_proto.h); its decoder accepts SNAP2 only
 * (IL_ACCEPT_COM_VTOL), so a ballistic FCU's SNAP is rejected in its header and vice versa.  The ballistic build is the
 * v0.6.3 COM (IL_SNAP -> TELEM v1).  Everything below holds for both (the reply gate keys on reply_seq, the first byte
 * of both snapshots).
 * v0.9 RP1 (fw09 SPEC 6.1): SNAP2 / SNAP3 end with the power tail (pw_vaux, pw_flags; core/ilink_proto.h).  A snapshot
 * whose build bits (pw_flags >> 6) name a power build (wired / wired_buffer) becomes TELEM v4 (41 B, from SNAP2) / v5
 * (47 B, from SNAP3) - the TELEM v2 / v3 bytes + the two tail bytes unchanged (core/link_proto_v09.h); a battery build's
 * snapshot (0, 0) stays TELEM v2 / v3 byte for byte.  The COM keeps no power state: one COM image serves the battery,
 * tether and hybrid builds.  The longest reply is TELEM v5 (47 B): the reply window below holds for it (run_com_simbench
 * checks it with the 47-byte frame).
 *
 *   uplink   radio_read -> core/link_proto.c streaming decoder (vehicle LINK_VEHICLE_ID, CMD
 *            only) -> one IL_CMD per accepted frame (seq, cmd, and the radio's last RSSI / SNR),
 *            in arrival order, queued to the FCU with ilink_uart_write (all or nothing; a
 *            refused frame counts il_tx_dropped).  The COM does not interpret the command.  Every
 *            queued IL_CMD opens a reply slot (its seq and COM tick, COM_FWD_SLOTS of them).
 *   reply    ilink_uart_read -> core/ilink_proto.c decoder (il_decoder_push, zero copy; built with
 *            IL_ACCEPT_COM, v0.6.3: a header with the LEN or TYPE of an IL_CMD / IL_STAT - the COM ->
 *            FCU types - is rejected right there and counted bad_len / bad_type, so only IL_SNAP
 *            frames come out) -> reply gate (below) -> stage 1 il_snap_to_tlm_src (roll / pitch / yaw
 *            from q) -> stage 2 onboard_telemetry + lp_encode_telem(LINK_VEHICLE_ID, reply_seq) -> one
 *            pending TELEM frame -> radio_send when the radio is free.  Only the newest reply is kept: a
 *            SNAP or TELEM replaced before it reached the radio counts replies_dropped (superseded).
 *   status   IL_STAT (every il_stat_t field), paced so that it never stands in front of an IL_CMD
 *            on the wire (D4, below), plus one boot STAT (flag IL_STAT_BOOT until the first STAT was
 *            queued) no earlier than COM_STAT_BOOT_MS after the COM's reset (R3, below).
 *
 * Reply gate (D5, half-duplex hardening).  A SNAP becomes a TELEM only if its reply_seq is the seq
 * of an IL_CMD this COM queued less than COM_REPLY_WINDOW_TICKS ago whose slot is still open;
 * accepting it closes that slot and every older one (the FCU answers all IL_CMDs of one supervisor
 * tick with one SNAP, in order).  The pending TELEM is handed to the radio only while the same
 * window is still open, otherwise it is dropped.  So a stray SNAP (no such IL_CMD), a duplicate
 * (slot already closed) or a late one (window over, at the SNAP or at the radio) is refused and
 * counted in il_stat_t.replies_dropped (com_link_stats_t tells the three reasons apart): whatever
 * the FCU sends, radio_send runs only within COM_REPLY_WINDOW_MS of the IL_CMD it answers.
 *
 * Every call does a bounded amount of work: com_link_uplink <= COM_RADIO_CHUNK decoder bytes,
 * com_link_ilink <= COM_ILINK_CHUNK decoder bytes, com_link_job at most ONE of reply stage 1,
 * reply stage 2 or the STAT (the costly steps never share a main-loop pass).
 *
 * Invariant (SPEC 10.3): the COM transmits on the radio only a TELEM built from an FCU SNAP that
 * answers an IL_CMD it forwarded within COM_REPLY_WINDOW_TICKS - radio_send has one call site,
 * com_link_send, whose frame comes only from reply stage 2 behind the reply gate - so the vehicle
 * still answers only valid uplinks, never later than the ground's half-duplex scheduler allows.
 */
#ifndef FC_COM_LINK_H
#define FC_COM_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "../params_gen.h"

#define COM_RADIO_CHUNK 8u                   /* uplink decoder bytes per pass (lp_decoder_feed ~22 us each) */
/* ilink decoder bytes per pass: il_decoder_push costs <= 95 cycles a byte with IL_ACCEPT_COM (core/ilink_proto.c, the
 * v0.6.3 protocol bench in simavr), so 32 bytes are <= 0.19 ms; 32 bytes per pass keep up with a continuous 250 kbaud
 * stream (25 bytes per ms) as long as a pass is shorter than 1.28 ms (com_simbench: <= 0.93 ms), and the 64-byte RX
 * ring (2.56 ms) absorbs two such passes */
#define COM_ILINK_CHUNK 32u

/* ---- D5: the reply window.  The ground (sim/ground_station_logic.py) has one request in flight and gives it up after
 * link_model reply_timeout_s, counted from the moment it starts writing the request: 531.0 ms lora_uart, 292.4 ms
 * sx127x (params.json of 2026-10-15; tools/run_com_simbench.py recomputes every number here from sim/link_model.py and
 * checks the window against them on each run).  The uplink's last byte reaches the COM uplink_s after that moment
 * (149.2 / 64.9 ms) and the IL_CMD is queued < 1 ms later.  A TELEM handed to the radio h ms after the uplink's arrival
 *   - reaches the ground before its timeout iff h + start <= 108.0 ms (both radios: the timeout is the ground's
 *     round-trip model with sim/link_model.FC_REPLY_ALLOWANCE_S = 68 ms for the vehicle (v0.5's FC_REPLY_DELAY_TICKS + 1
 *     = 17 ticks) plus a 40 ms margin), start
 *     being the radio's own delay (lora_uart: the AUX wait, 0 with the module idle, at most LINK_AUX_WAIT_TICKS + 1
 *     = 56 ms with a stuck AUX; sx127x: the FIFO load, 5 radio_service steps = 5 main-loop passes, ~4.4 ms);
 *   - is off the air before the ground's next request can be on the air (written at the timeout) iff
 *     h + start <= 175.2 ms (lora_uart) / 168.0 ms (sx127x).
 * 80 ms: above the 68 ms the ground's timeout already grants the vehicle, ~10x the normal reply path (uplink -> IL_CMD
 * -> FCU tick -> SNAP 2.2 ms on the wire -> two reply stages: ~8 ms in com_simbench, ~12-16 ms with a real FCU's tick
 * phase and time-critical part), and still 23-28 ms inside the first bound and >= 39 ms inside the second one with a
 * stuck AUX.  (100 ms would leave the sx127x reply only ~3 ms inside the first bound, which rests on *_ASSUMED module
 * timings.)  "< 20 ticks" means < 80 ms after the start of the tick the IL_CMD was queued in. ---- */
#define COM_REPLY_WINDOW_MS 80u
#define COM_REPLY_WINDOW_TICKS ((uint16_t)(COM_REPLY_WINDOW_MS * COM_TICK_HZ / 1000u))     /* 20 ticks */
#define COM_FWD_SLOTS 4u                     /* open reply slots (power of two): >= the uplinks of one window (1 normally) */

/* ---- D4: IL_STAT pacing.  The FCU decodes, in its time-critical part, only the ilink bytes in which an IL_CMD can
 * complete (main.c step 5); an IL_CMD right behind a 42-byte IL_STAT reaches its supervisor one tick late.  So the COM
 * queues an IL_STAT
 *   - right behind an IL_CMD it has just queued, if >= ILINK_STAT_DIV ticks (1 / stat_hz) passed since the last STAT
 *     (the next uplink is >= ~13 ms away: one request at a time, 13 bytes at 9600 baud or a 46 ms LoRa packet);
 *   - on its divider (>= ILINK_STAT_DIV since the last STAT) while no IL_CMD was queued for > COM_STAT_QUIET_TICKS;
 *   - once after boot (IL_STAT_BOOT), in the first pass whose COM tick is >= COM_STAT_BOOT_TICKS (R3, below).
 * STAT gap: >= ILINK_STAT_DIV; <= ILINK_STAT_DIV + the uplink interval while uplinks flow (1.5 s at 2 Hz polling);
 * <= ILINK_STAT_DIV + COM_STAT_QUIET_TICKS + 1 tick (2.5 s) when they stop just before a STAT would ride on them; with
 * no uplink since boot: the boot STAT at COM tick >= 25 (0.1 s), then quiet STATs from tick 376 (1.504 s) on, every
 * ILINK_STAT_DIV.  The boot STAT is the one STAT not tied to the uplink: a CMD frame the radio delivers during its
 * 1.7 ms on the wire (only after a COM reset while the ground polls) waits behind it, and that IL_CMD reaches the
 * supervisor one FCU tick later - D4's cost, once per COM reset at most. ---- */
#define COM_STAT_QUIET_TICKS ((uint32_t)ILINK_STAT_DIV * 3u / 2u)   /* 1.5 / stat_hz */

/* ---- R3 (v0.6.3): the boot STAT waits COM_STAT_BOOT_MS after the COM's reset, so that an FCU powered up together
 * with the COM (one 5 V logic UBEC for all three boards, the same fuses) is receiving when it comes.  The v0.6.2 COM
 * queued it in its first pass and the co-simulation lost its first 10 bytes in front of the FCU's RXEN1
 * (normal_lora_uart_d000000).  The FCU's side of the timing:
 *   - reset start-up: all chips run the same fuses (platformio.ini: crystal CKSEL 1111, SUT 11, BOD 4.0 V): 16K CK of
 *     the crystal + 64K cycles of each chip's own watchdog oscillator (65 ms at 5 V in the ATmega128 datasheet's
 *     start-up table - a typical value only; it varies with the chip, VCC and temperature) after VCC passes the BOD
 *     level.  So the chips leave reset at nearly the same moment, the FCU later by the difference of the two delays;
 *   - code from its reset vector until a byte reaches its RX ring (RXEN1 + RXCIE1 set by ilink_uart_init, then sei):
 *     2.26 ms (RXEN1 at 2.14 ms) for the FCU flight build, 2.29 ms for fcu_cosim, in simavr (v0.6.2 FCU init order
 *     with nlink_spi_init in place of twi_init).  The STAT then waits in the FCU's 64-byte RX ring until its main
 *     loop reads it (v0.6.2 fcu_cosim: first tick 6.2 ms after reset).
 * The COM side, same simavr probe: the hold counts from com_timer_init, ~1.5 ms after the reset vector (behind the
 * .init stack paint), so the boot STAT's first byte leaves at 101.8 ms (com_lora_uart) / 101.7 ms (com_sx127x);
 * the v0.6.2 COM sent it at 1.79 ms (com_cosim), 0.39 ms before the FCU's RXEN1 = the 10 bytes the co-simulation
 * lost.  So an FCU that leaves reset up to ~99.5 ms after the COM still receives the whole boot STAT: one chip's
 * 65 ms start-up delay could be 2.5x the other's.  The hold costs nothing (the FCU only logs the STAT; the next one
 * follows within ILINK_STAT_DIV + COM_STAT_QUIET_TICKS). ---- */
#define COM_STAT_BOOT_MS 100u
#define COM_STAT_BOOT_TICKS ((COM_STAT_BOOT_MS * COM_TICK_HZ + 999u) / 1000u)   /* 25 ticks (also usable in #if) */

/* IL_STAT kinds (com_link_stats_t, the bench hooks) */
#define COM_STAT_KIND_BOOT 0u
#define COM_STAT_KIND_PIGGYBACK 1u
#define COM_STAT_KIND_QUIET 2u

typedef struct {
    uint16_t cmd_forwarded;                  /* IL_CMD frames queued to the FCU */
    uint16_t il_tx_dropped;                  /* IL_CMD / IL_STAT not queued: ilink TX ring full */
    uint16_t il_wrong_type;                  /* defensive, stays 0: a CMD / STAT (COM -> FCU types) returned by the decoder,
                                              * which IL_ACCEPT_COM rejects in the header (il_bad_len / il_bad_type) */
    uint16_t snaps_accepted;                 /* SNAPs through the reply gate */
    uint16_t replies;                        /* TELEM frames handed to the radio */
    uint16_t replies_superseded;             /* SNAP / TELEM replaced by a newer one, or the radio refused the frame */
    uint16_t replies_refused;                /* D5: SNAP without an open reply slot (stray, duplicate or late) */
    uint16_t replies_expired;                /* D5: TELEM not taken by the radio within the window */
    uint16_t stats_sent;                     /* IL_STAT frames queued, by kind: */
    uint16_t stats_piggyback, stats_quiet;   /*   (the boot STAT is stats_sent - piggyback - quiet) */
    uint16_t loop_us_max;                    /* longest pass since the previous STAT (the STAT's field) */
    uint16_t il_frames, il_bad_crc, il_bad_len, il_bad_type, il_skipped;   /* the ilink decoder's counters (copied by
                                                                            * com_link_stats) */
} com_link_stats_t;

void     com_link_init(uint8_t reset_cause);  /* decoders, counters; the boot STAT becomes due (at COM_STAT_BOOT_TICKS) */
void     com_link_uplink(uint32_t tick);      /* tick = COM tick (com_timer_ticks), stamps the reply slots */
void     com_link_ilink(uint32_t tick);
bool     com_link_job(uint32_t tick);         /* one costly step if any is pending; true if it ran one */
void     com_link_send(uint32_t tick);
void     com_link_pass_us(uint16_t us);       /* main-loop pass time -> the STAT's loop_us_max */
uint16_t com_link_cmd_forwarded(void);        /* IL_CMD frames queued since boot (LED) */
const com_link_stats_t *com_link_stats(void);

#endif /* FC_COM_LINK_H */
