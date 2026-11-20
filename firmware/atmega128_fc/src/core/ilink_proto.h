/* ilink_proto.h - the FCU <-> COM inter-processor link (SPEC 10.4, v0.6), portable C99.
 *
 * v0.6 split the flight computer over ATmega128A chips (params.json flight_computer.architecture;
 * v0.6.3 adds the NAV, which reads the sensors and runs the estimator - core/nlink_proto.h):
 *
 *   FCU  onboard supervisor (commands + interlocks I1-I10), attitude control, PWM outputs,
 *        OpenLog log.  Every flight decision is made here.
 *   COM  LoRa radio driver (hal/lora_uart.c or hal/sx127x.c), uplink frame decoding
 *        (core/link_proto.c), TELEM building from the FCU's snapshot.  No flight decisions.
 *
 * Wire: FCU USART1 (PD2 RXD1 / PD3 TXD1) <-> COM USART0 (PE0 RXD0 / PE1 TXD0), ILINK_BAUD
 * (params_gen.h, flight_computer.ilink.baud = 250000) 8N1, U2X = 0.  The FCU's TXD1 reaches
 * the COM's PE0 (also its ISP PDI) through a 1 kOhm series resistor.
 *
 * Frame:   0xC5 0x5C | LEN | TYPE | PAYLOAD[LEN] | CRC16 lo | CRC16 hi
 *          CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no final XOR) over
 *          LEN, TYPE and PAYLOAD - the same CRC as the radio link (lp_crc16); il_crc16_update is
 *          the byte-wise branch-free form (link_proto.c's update, written on the register bytes).
 *          LEN is the payload length; a frame whose LEN is not the fixed length of its TYPE is
 *          rejected (bad_len), as is an unknown TYPE (bad_type) or a CRC mismatch (bad_crc).
 *          The decoder does bounded work per byte and never replays: see "streaming decoder".
 *          Multi-byte fields are little-endian; float = IEEE-754 binary32 little-endian (the
 *          AVR's float and Python struct '<f').
 *
 * Types:
 *   IL_TYPE_CMD  (COM -> FCU, IL_CMD_LEN = 8):  one uplink CMD frame the COM's lp decoder
 *        accepted (vehicle id, length and CRC already checked), in arrival order:
 *        seq u8, cmd u8, arg u16, nonce u16, rssi_dbm i8 (RADIO_RSSI_NONE = 127 when the radio
 *        reports none), snr_q4 i8.  The FCU hands it to the supervisor as an onb_frame_t exactly
 *        as the v0.5 single-chip firmware did with a frame it decoded itself.
 *   IL_TYPE_SNAP (FCU -> COM, IL_SNAP_LEN = 49): the reply snapshot the supervisor captured in the
 *        tick it processed the uplink (onb_outputs_t.reply): reply_seq u8, state u8, flags u8,
 *        reason u8, ack_seq u8, nonce u16, throttle_milli u16, tick u32, h_m f32, vz_mps f32,
 *        tilt_deg f32, v_cell f32, loop_max_us f32, q[4] f32 (w, x, y, z of the attitude
 *        estimate in that tick).  The COM computes roll / pitch / yaw from q (il_snap_to_tlm_src,
 *        float32, the v0.5 formulas), quantises (onboard_telemetry) and sends TELEM with SEQ =
 *        reply_seq.  The COM never transmits anything that is not built from a SNAP.
 *   TYPE 4 / 5 (LEN 55 / 62) were the VTOL program's SNAP2 / SNAP3 (removed 2026-11-18; their decoder entries and
 *        accept bits went on 2026-11-19): an unknown TYPE (bad_type) and LEN (bad_len) now, exactly as under the
 *        rocket's masks before - no rocket decoder accepted them - and as sim/ilink_proto.py decides.
 *   IL_TYPE_STAT (COM -> FCU, IL_STAT_LEN = 36): see il_stat_t.  Paced by the COM (com/com_link.h
 *        D4): at least 1 / stat_hz apart (ILINK_STAT_DIV ticks), never queued in front of an IL_CMD,
 *        so up to ~2.5 / stat_hz apart when the uplink stops; plus one boot STAT (IL_STAT_BOOT) sent
 *        no earlier than 100 ms after the COM's reset, so an FCU powered up at the same moment is
 *        listening.  The FCU logs it (log record 'C') and copies the link counters into its 1 Hz
 *        status record; nothing in the supervisor depends on it.
 *
 * Direction filter (v0.6.3): il_decoder_init takes the set of TYPEs this side receives (IL_ACCEPT_*):
 * the FCU IL_ACCEPT_FCU (CMD | STAT), the COM IL_ACCEPT_COM (SNAP); IL_ACCEPT_ALL (CMD | SNAP | STAT) is for taps and
 * tests.  A LEN or TYPE that cannot travel in
 * that direction is rejected in the header, so a SNAP header in garbage on the FCU's line never
 * swallows the bytes behind it (and the COM never takes a CMD-length candidate).
 *
 * Failure semantics (SPEC 10.4): a dead / resetting COM or a broken wire means no IL_CMD reaches
 * the supervisor, which is exactly a lost radio link: I1p on the pad, I1 in flight (unchanged
 * v0.4 interlocks).  A dead FCU means no SNAP, so the COM sends nothing and the ground sees reply
 * timeouts.  Corrupted bytes are dropped by the CRC and counted on the receiving side.
 *
 * Invariant: the bytes of every frame are fully determined by the packed struct (no padding, no
 * uninitialised field), so the Python reference sim/ilink_proto.py produces the same bytes.
 */
#ifndef FC_ILINK_PROTO_H
#define FC_ILINK_PROTO_H

#include "fcmath.h"
#include "link_proto.h"
#include "onboard.h"

#define IL_SYNC0 0xC5u
#define IL_SYNC1 0x5Cu
#define IL_TYPE_CMD 0x01u
#define IL_TYPE_SNAP 0x02u
#define IL_TYPE_STAT 0x03u
#define IL_CMD_LEN 8u
#define IL_SNAP_LEN 49u
#define IL_STAT_LEN 36u
#define IL_MAX_PAYLOAD IL_SNAP_LEN
#define IL_FRAME_OVERHEAD 6u                                    /* sync x2, LEN, TYPE, CRC x2 */
#define IL_CMD_FRAME_BYTES (IL_FRAME_OVERHEAD + IL_CMD_LEN)    /* 14 */
#define IL_SNAP_FRAME_BYTES (IL_FRAME_OVERHEAD + IL_SNAP_LEN)  /* 55 */
#define IL_STAT_FRAME_BYTES (IL_FRAME_OVERHEAD + IL_STAT_LEN)  /* 42 */
#define IL_FRAME_MAX_BYTES IL_SNAP_FRAME_BYTES                 /* the longest frame */

/* il_decoder_init accept masks: one bit per TYPE (bit TYPE - 1) */
#define IL_ACCEPT_CMD 0x01u
#define IL_ACCEPT_SNAP 0x02u
#define IL_ACCEPT_STAT 0x04u
#define IL_ACCEPT_ALL 7                                                   /* CMD | SNAP | STAT: taps and tests */
#define IL_ACCEPT_FCU (IL_ACCEPT_CMD | IL_ACCEPT_STAT)                    /* what the FCU receives */
#define IL_ACCEPT_COM IL_ACCEPT_SNAP                                      /* what the COM receives */
#define IL_NEED_NEVER 0xFFu      /* il_decoder_need / il_decoder_cmd_need: the decoder can never return such a frame */

/* il_stat_t.flags */
#define IL_STAT_RADIO_OK 0x01u        /* sx127x: RegVersion 0x12 read back; lora_uart: always */
#define IL_STAT_BOOT 0x02u            /* first STAT after a COM reset */
#define IL_STAT_SX127X 0x04u          /* backend: set = sx127x, clear = lora_uart */

typedef struct {
    uint8_t  seq;
    lp_cmd_t cmd;
    int8_t   rssi_dbm, snr_q4;
} il_cmd_t;

typedef struct {
    uint8_t  reply_seq;
    uint8_t  state, flags, reason, ack_seq;
    uint16_t nonce, throttle_milli;
    uint32_t tick;
    float    h_m, vz_mps, tilt_deg, v_cell, loop_max_us;
    float    q[4];
} il_snap_t;

typedef struct {                       /* 36 bytes, in this order */
    uint32_t com_ticks;                /* COM tick counter (params com.tick_hz) */
    uint8_t  reset_cause;              /* COM MCUCSR at boot (watchdog_reset_cause() encoding) */
    uint8_t  flags;                    /* IL_STAT_* */
    uint8_t  radio_version;            /* sx127x RegVersion, 0 for lora_uart */
    int8_t   rssi_dbm, snr_q4;         /* last received packet */
    uint8_t  params_tag;               /* low byte of the params.json sha256 prefix the COM was built with */
    uint16_t dec_frames;               /* lp decoder: valid CMD frames */
    uint16_t dec_bad_crc, dec_bad_len;
    uint16_t dec_other;                /* wrong vehicle + wrong type + bad payload length + overflow */
    uint16_t cmd_forwarded;            /* IL_CMD frames queued to the FCU */
    uint16_t radio_rx_errors, radio_rx_overflow, radio_tx_timeouts;
    uint16_t replies, replies_dropped; /* TELEM handed to the radio / SNAPs dropped (radio busy, newer SNAP) */
    uint16_t il_bad;                   /* ilink frames the COM rejected (CRC / length / type) */
    uint16_t il_tx_dropped;            /* IL_CMD / IL_STAT not queued: COM ilink TX ring full */
    uint16_t loop_us_max;              /* longest COM main-loop pass since the previous STAT, us */
} il_stat_t;

typedef struct {
    uint16_t frames;                   /* complete frames (returned + overflow) */
    uint16_t bad_crc, bad_len, bad_type;
    uint16_t skipped_bytes;            /* input bytes discarded (hunted and not 0xC5, or part of a rejected candidate) */
    uint16_t overflow;                 /* complete frames not returned because the caller passed max_out = 0 */
} il_dec_stats_t;

typedef struct {
    uint8_t  type, len;
    uint8_t  payload[IL_MAX_PAYLOAD];
} il_frame_t;

typedef struct {
    uint8_t  n;                        /* bytes of the open candidate, 0 = hunting (public, read-only) */
    uint8_t  accept;                   /* IL_ACCEPT_* mask given to il_decoder_init (public, read-only) */
    il_dec_stats_t stats;              /* public, read-only (il_decoder_init zeroes them) */
    /* internal (ilink_proto.c) */
    uint16_t crc;                      /* running CRC over the candidate's LEN, TYPE and payload bytes */
    il_frame_t fr;                     /* the candidate: fr.len = LEN and fr.type = the TYPE that LEN carries (n >= 3),
                                        * payload */
} il_decoder_t;

#if defined(__GNUC__)
#define IL_ALWAYS_INLINE __attribute__((always_inline))
#else
#define IL_ALWAYS_INLINE
#endif

/* CRC-16/CCITT-FALSE, one byte: x = (crc >> 8) ^ b; x ^= x >> 4; crc = (crc << 8) ^ (x << 12) ^ (x << 5) ^ x,
 * written on the two register bytes (8-bit shifts only: SWAP / ANDI / LSL on the AVR).  == lp_crc16 byte by byte.
 * Always inlined (avr-gcc -Os would otherwise call it: ~20 cycles of call overhead on a ~20-cycle body). */
static inline IL_ALWAYS_INLINE uint16_t il_crc16_update(uint16_t crc, uint8_t b)
{
    uint8_t x = (uint8_t)((uint8_t)(crc >> 8) ^ b);
    x ^= (uint8_t)(x >> 4);
    return (uint16_t)(((uint16_t)(uint8_t)((uint8_t)crc ^ (uint8_t)(x << 4) ^ (uint8_t)(x >> 3)) << 8) |
                      (uint8_t)((uint8_t)(x << 5) ^ x));
}

uint16_t il_crc16(const uint8_t *data, uint8_t n, uint16_t crc);   /* il_crc16_update over data; == lp_crc16 */
uint8_t  il_payload_len(uint8_t type);                           /* fixed LEN of a TYPE, 0 = unknown type */

/* complete frame into out (>= IL_FRAME_OVERHEAD + n bytes); returns its length, 0 if (type, n) is invalid */
uint8_t  il_encode(uint8_t type, const uint8_t *payload, uint8_t n, uint8_t *out);

void     il_pack_cmd(const il_cmd_t *c, uint8_t out[IL_CMD_LEN]);
void     il_unpack_cmd(const uint8_t in[IL_CMD_LEN], il_cmd_t *c);
void     il_pack_snap(const il_snap_t *s, uint8_t out[IL_SNAP_LEN]);
void     il_unpack_snap(const uint8_t in[IL_SNAP_LEN], il_snap_t *s);
void     il_pack_stat(const il_stat_t *s, uint8_t out[IL_STAT_LEN]);
void     il_unpack_stat(const uint8_t in[IL_STAT_LEN], il_stat_t *s);

/* FCU side: the snapshot of one supervisor reply (src = onb_outputs_t.tlm, q = estimator quaternion) */
void     il_snap_from_tlm_src(uint8_t reply_seq, const onb_tlm_src_t *src, const float q[4], il_snap_t *s);
/* COM side: back to the TELEM source; roll / pitch / yaw from q in float32 (v0.5 FCU formulas) */
void     il_snap_to_tlm_src(const il_snap_t *s, onb_tlm_src_t *src);

/* streaming decoder (v0.6 phase 3: bounded work per byte, no replay).  The normative rules are the
 * module docstring of sim/ilink_proto.py (IlinkDecoder); Python and C agree on every returned frame,
 * every counter and il_decoder_need / il_decoder_cmd_need after every byte, for every byte stream (host
 * test vectors + tools/crosscheck_controller.py --only ilink).  In short:
 *   hunting: 0xC5 opens a candidate, any other byte is discarded (skipped_bytes);
 *   candidate byte n: 1 must be 0x5C; 2 LEN must be the length of an ACCEPTED TYPE (CMD 8, STAT 36, SNAP 49;
 *   bad_len); 3 TYPE accepted (unknown or not accepted: bad_type) with that LEN (bad_len); payload bytes are
 *   always taken (incremental CRC); the CRC low byte is checked when it arrives, the high byte completes the
 *   frame (bad_crc on a mismatch of either);
 *   reject: the candidate's n bytes are discarded (skipped_bytes += n) and the rejecting byte is examined
 *   ONCE more as a possible 0xC5 - nothing else is replayed, so a frame hidden inside a rejected candidate
 *   is lost (the ground station's retry covers a lost IL_CMD).  Bounded real-time cost on the FCU is the
 *   reason: the phase-2 replay cost up to 6316 cycles for one byte, ~1821 cycles per byte on sync spam.
 * At most ONE frame completes per input byte: il_decoder_feed returns 0 or 1 and writes it to out[0];
 * with max_out = 0 it is counted in stats.overflow instead.  IL_DEC_MAX_OUT stays for source compatibility. */
#define IL_DEC_MAX_OUT 1u
/* accept: IL_ACCEPT_FCU on the FCU, IL_ACCEPT_COM on the COM (IL_ACCEPT_ALL: taps, tests); bits outside IL_ACCEPT_ALL
 * are ignored */
void     il_decoder_init(il_decoder_t *d, uint8_t accept);
uint8_t  il_decoder_feed(il_decoder_t *d, uint8_t byte, il_frame_t *out, uint8_t max_out);
/* zero-copy form of il_decoder_feed (same rules and counters; overflow never counts): the complete frame inside the
 * decoder, valid until the next call, or 0 (NULL).  The bounded path for the FCU: <= 91 cycles for any byte in simavr
 * (phase 3; the v0.6.3 direction filter adds <= 4 with IL_ACCEPT_FCU, see ilink_proto.c), against <= 447 for
 * il_decoder_feed's frame-completing byte (it copies up to 49 payload bytes). */
const il_frame_t *il_decoder_push(il_decoder_t *d, uint8_t byte);
/* bytes still needed before the open candidate can complete: 0 while hunting, LEN + 6 - n once LEN is known
 * (n >= 3), the rest of the shortest ACCEPTED frame before that (FCU / ALL: IL_CMD, 14 - n; COM: SNAP, 55 - n;
 * IL_NEED_NEVER if the mask accepts nothing) */
uint8_t  il_decoder_need(const il_decoder_t *d);
/* the fewest further input bytes after which the decoder can return an IL_CMD (exact: some continuation of this
 * length returns one on its last byte, no shorter one can): 14 while hunting; il_decoder_need while the candidate
 * can still be an IL_CMD; else 14 in the header (a 0xC5 as TYPE rejects and opens the next CMD), need + 12 while
 * payload bytes of a longer frame are to come (a 0xC5 as its CRC low byte), and with only CRC bytes left 14, or 15
 * if the next expected CRC byte is itself 0xC5; IL_NEED_NEVER for a decoder that does not accept IL_CMD (the
 * COM's).  The FCU's "can an IL_CMD complete in the bytes waiting?" test. */
uint8_t  il_decoder_cmd_need(const il_decoder_t *d);

#endif /* FC_ILINK_PROTO_H */
