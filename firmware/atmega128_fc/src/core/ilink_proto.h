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
 *   IL_TYPE_SNAP2 (FCU -> COM, IL_SNAP2_LEN = 55, v0.7 VTOL build): the 49 SNAP bytes, then phase u8 (LP_PHASE_*),
 *        spare u8 (0; the COM ignores it) and the altitude target h_tgt_m f32 (il_snap2_t).  The COM turns it into
 *        a TELEM2 frame (lp_encode_telem2: the TELEM v1 fields + phase + h_tgt quantised to 0.1 m).  A new TYPE used
 *        only by the VTOL build: the ballistic build keeps sending the v0.6.3 SNAP bytes and its COM keeps
 *        IL_ACCEPT_COM.  The spare byte makes LEN 55, three or more bits away from the lengths the FCU accepts
 *        (CMD 8, STAT 36); no flight decoder accepts both SNAP types (VTOL COM: IL_ACCEPT_COM_VTOL = SNAP2 only).
 *   IL_TYPE_SNAP3 (FCU -> COM, IL_SNAP3_LEN = 62, v0.8 VTOL build with GPS position hold): the SNAP2 layout with the
 *        spare byte carrying gps_flags, then num_sv u8, hacc_dm u8 (0.1 m), pos_mode u8 and dist_m f32 (distance from the
 *        hold point, NaN = none; il_snap3_t).  The COM turns it into a TELEM3 frame (lp_encode_telem3).  62 is three or
 *        more bits from the lengths the FCU accepts (CMD 8: 4 bits, STAT 36: 3 bits); the VTOL + GPS COM accepts only
 *        SNAP3 (IL_ACCEPT_COM_GPS).  IL_ACCEPT_ALL keeps its v0.7 meaning (15: the v0.7 taps and vectors keep their
 *        bytes); IL_ACCEPT_ALL8 (31) adds SNAP3.
 *   v0.9 firmware round (RP1, fw09 SPEC 6.1; sim/ilink_proto.py): SNAP2 and SNAP3 end with the TELEM v4 / v5 power tail,
 *        pw_vaux u8 (0.125 V) and pw_flags u8 (PF_* | build << 6; onb_tlm_src_t pw_vaux / pw_flags of a power build,
 *        0 / 0 in the battery build): IL_SNAP2_LEN 57, IL_SNAP3_LEN 65 (+ a reserved byte spare3, 0, ignored by the COM:
 *        65 keeps the length rule - 3 bits from CMD 8, 4 bits from STAT 36; 64 would be 2 bits from 8; the VTOL COM's
 *        two lengths 57 / 65 are 4 bits apart).  The COM sends TELEM v4 / v5 for a snapshot whose build bits are not the
 *        battery build (LP_PWR_TAIL_DUE), else TELEM v2 / v3 - one COM image serves every power build.  The ballistic
 *        AVR images (LP_RP1 0, core/link_proto.h) keep the v0.8.1 lengths 55 / 62 (they never send or accept SNAP2 /
 *        SNAP3; their images stay byte-identical).
 *   IL_TYPE_STAT (COM -> FCU, IL_STAT_LEN = 36): see il_stat_t.  Paced by the COM (com/com_link.h
 *        D4): at least 1 / stat_hz apart (ILINK_STAT_DIV ticks), never queued in front of an IL_CMD,
 *        so up to ~2.5 / stat_hz apart when the uplink stops; plus one boot STAT (IL_STAT_BOOT) sent
 *        no earlier than 100 ms after the COM's reset, so an FCU powered up at the same moment is
 *        listening.  The FCU logs it (log record 'C') and copies the link counters into its 1 Hz
 *        status record; nothing in the supervisor depends on it.
 *
 * Direction filter (v0.6.3): il_decoder_init takes the set of TYPEs this side receives (IL_ACCEPT_*):
 * the FCU IL_ACCEPT_FCU (CMD | STAT), the COM IL_ACCEPT_COM (SNAP; v0.7 VTOL build: IL_ACCEPT_COM_VTOL, SNAP2
 * only); IL_ACCEPT_ALL (all four TYPEs since v0.7) is for taps and tests.  A LEN or TYPE that cannot travel in
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
#define IL_TYPE_SNAP2 0x04u                                     /* v0.7 VTOL build: SNAP + phase, spare, h_tgt_m */
#define IL_TYPE_SNAP3 0x05u                                     /* v0.8 VTOL + GPS: SNAP2 (spare = gps_flags) + GPS fields */
#define IL_CMD_LEN 8u
#define IL_SNAP_LEN 49u
#define IL_STAT_LEN 36u
#if LP_RP1
#define IL_SNAP2_LEN 57u                                        /* v0.9 RP1: + pw_vaux, pw_flags (v0.7: 55) */
#define IL_SNAP3_LEN 65u                                        /* v0.9 RP1: + pw_vaux, pw_flags, spare3 (v0.8: 62) */
#else
#define IL_SNAP2_LEN 55u                                        /* the ballistic images: the v0.8.1 lengths */
#define IL_SNAP3_LEN 62u
#endif
#define IL_MAX_PAYLOAD IL_SNAP3_LEN
#define IL_FRAME_OVERHEAD 6u                                    /* sync x2, LEN, TYPE, CRC x2 */
#define IL_CMD_FRAME_BYTES (IL_FRAME_OVERHEAD + IL_CMD_LEN)    /* 14 */
#define IL_SNAP_FRAME_BYTES (IL_FRAME_OVERHEAD + IL_SNAP_LEN)  /* 55 */
#define IL_STAT_FRAME_BYTES (IL_FRAME_OVERHEAD + IL_STAT_LEN)  /* 42 */
#define IL_SNAP2_FRAME_BYTES (IL_FRAME_OVERHEAD + IL_SNAP2_LEN) /* 63 (v0.7: 61) */
#define IL_SNAP3_FRAME_BYTES (IL_FRAME_OVERHEAD + IL_SNAP3_LEN) /* 71 (v0.8: 68) */
#define IL_FRAME_MAX_BYTES IL_SNAP3_FRAME_BYTES                /* v0.6.3: 55, v0.7: 61, v0.8: 68 */

/* il_decoder_init accept masks: one bit per TYPE (bit TYPE - 1) */
#define IL_ACCEPT_CMD 0x01u
#define IL_ACCEPT_SNAP 0x02u
#define IL_ACCEPT_STAT 0x04u
#define IL_ACCEPT_SNAP2 0x08u
#define IL_ACCEPT_SNAP3 0x10u
#define IL_ACCEPT_ALL (IL_ACCEPT_CMD | IL_ACCEPT_SNAP | IL_ACCEPT_STAT | IL_ACCEPT_SNAP2)   /* v0.7 taps and tests (v0.6.3: 7) */
#define IL_ACCEPT_ALL8 (IL_ACCEPT_ALL | IL_ACCEPT_SNAP3)                  /* every TYPE of v0.8 (31) */
#define IL_ACCEPT_FCU (IL_ACCEPT_CMD | IL_ACCEPT_STAT)                    /* what the FCU receives (both builds) */
#define IL_ACCEPT_COM IL_ACCEPT_SNAP                                      /* what the ballistic COM receives */
#define IL_ACCEPT_COM_VTOL 0x08u                                          /* what the VTOL COM receives (= SNAP2) */
#define IL_ACCEPT_COM_GPS 0x10u                                           /* what the VTOL + GPS COM receives (= SNAP3) */
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

typedef struct {                       /* IL_TYPE_SNAP2 (57 bytes): the SNAP fields, then these */
    il_snap_t s;
    uint8_t  phase;                    /* LP_PHASE_* */
    uint8_t  spare;                    /* reserved: the FCU sends 0, the COM ignores it (kept for a bit-exact repack) */
    float    h_tgt_m;                  /* altitude target [m] (the supervisor's, 0 outside SPOOL / CLIMB / HOLD) */
    uint8_t  pw_vaux, pw_flags;        /* v0.9 RP1: the TELEM v4 power tail (0, 0 in the battery build) */
} il_snap2_t;

typedef struct {                       /* IL_TYPE_SNAP3 (62 bytes): the SNAP fields, then these (v0.8) */
    il_snap_t s;
    uint8_t  phase;                    /* LP_PHASE_* */
    uint8_t  gps_flags;                /* in the SNAP2 spare byte's position: core/gps_nav.h GPS_F_* */
    float    h_tgt_m;
    uint8_t  num_sv, hacc_dm, pos_mode;   /* hacc 0.1 m (LP_HACC_NONE = none), pos_mode POS_MODE_* */
    float    dist_m;                   /* distance from the hold point [m], NaN = no hold point */
    uint8_t  pw_vaux, pw_flags;        /* v0.9 RP1: the TELEM v5 power tail (0, 0 in the battery build) */
    uint8_t  spare3;                   /* v0.9 RP1: reserved (the FCU sends 0, the COM ignores it): LEN 65 */
} il_snap3_t;

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
void     il_pack_snap2(const il_snap2_t *s, uint8_t out[IL_SNAP2_LEN]);
void     il_unpack_snap2(const uint8_t in[IL_SNAP2_LEN], il_snap2_t *s);
void     il_pack_snap3(const il_snap3_t *s, uint8_t out[IL_SNAP3_LEN]);
void     il_unpack_snap3(const uint8_t in[IL_SNAP3_LEN], il_snap3_t *s);
void     il_pack_stat(const il_stat_t *s, uint8_t out[IL_STAT_LEN]);
void     il_unpack_stat(const uint8_t in[IL_STAT_LEN], il_stat_t *s);

/* FCU side: the snapshot of one supervisor reply (src = onb_outputs_t.tlm, q = estimator quaternion) */
void     il_snap_from_tlm_src(uint8_t reply_seq, const onb_tlm_src_t *src, const float q[4], il_snap_t *s);
/* COM side: back to the TELEM source; roll / pitch / yaw from q in float32 (v0.5 FCU formulas); src->v2 = false */
void     il_snap_to_tlm_src(const il_snap_t *s, onb_tlm_src_t *src);
/* the same for the VTOL build: SNAP2 (spare = 0) and back (src->v2 = true, phase / h_tgt_m from the SNAP2); v0.9: the
 * power tail from / to src->pw_vaux / pw_flags where onb_tlm_src_t has them (PM_ANY: power and host builds), else 0 */
void     il_snap2_from_tlm_src(uint8_t reply_seq, const onb_tlm_src_t *src, const float q[4], il_snap2_t *s);
void     il_snap2_to_tlm_src(const il_snap2_t *s, onb_tlm_src_t *src);
/* v0.8 VTOL + GPS: SNAP3 (src->v3: gps_flags, num_sv, hacc_dm, pos_mode, dist_m) and back (src->v2 = src->v3 = true);
 * v0.9: the power tail as for SNAP2, spare3 = 0 */
void     il_snap3_from_tlm_src(uint8_t reply_seq, const onb_tlm_src_t *src, const float q[4], il_snap3_t *s);
void     il_snap3_to_tlm_src(const il_snap3_t *s, onb_tlm_src_t *src);

/* streaming decoder (v0.6 phase 3: bounded work per byte, no replay).  The normative rules are the
 * module docstring of sim/ilink_proto.py (IlinkDecoder); Python and C agree on every returned frame,
 * every counter and il_decoder_need / il_decoder_cmd_need after every byte, for every byte stream (host
 * test vectors + tools/crosscheck_controller.py --only ilink).  In short:
 *   hunting: 0xC5 opens a candidate, any other byte is discarded (skipped_bytes);
 *   candidate byte n: 1 must be 0x5C; 2 LEN must be the length of an ACCEPTED TYPE (CMD 8, STAT 36, SNAP 49,
 *   SNAP2 55, SNAP3 62; bad_len); 3 TYPE accepted (unknown or not accepted: bad_type) with that LEN (bad_len); payload bytes are
 *   always taken (incremental CRC); the CRC low byte is checked when it arrives, the high byte completes the
 *   frame (bad_crc on a mismatch of either);
 *   reject: the candidate's n bytes are discarded (skipped_bytes += n) and the rejecting byte is examined
 *   ONCE more as a possible 0xC5 - nothing else is replayed, so a frame hidden inside a rejected candidate
 *   is lost (the ground station's retry covers a lost IL_CMD).  Bounded real-time cost on the FCU is the
 *   reason: the phase-2 replay cost up to 6316 cycles for one byte, ~1821 cycles per byte on sync spam.
 * At most ONE frame completes per input byte: il_decoder_feed returns 0 or 1 and writes it to out[0];
 * with max_out = 0 it is counted in stats.overflow instead.  IL_DEC_MAX_OUT stays for source compatibility. */
#define IL_DEC_MAX_OUT 1u
/* accept: IL_ACCEPT_FCU on the FCU, IL_ACCEPT_COM on the ballistic COM, IL_ACCEPT_COM_VTOL on the VTOL COM, IL_ACCEPT_COM_GPS
 * on the VTOL + GPS COM (IL_ACCEPT_ALL / IL_ACCEPT_ALL8: taps, tests); bits outside IL_ACCEPT_ALL8 are ignored */
void     il_decoder_init(il_decoder_t *d, uint8_t accept);
uint8_t  il_decoder_feed(il_decoder_t *d, uint8_t byte, il_frame_t *out, uint8_t max_out);
/* zero-copy form of il_decoder_feed (same rules and counters; overflow never counts): the complete frame inside the
 * decoder, valid until the next call, or 0 (NULL).  The bounded path for the FCU: <= 91 cycles for any byte in simavr
 * (phase 3; the v0.6.3 direction filter adds <= 4 with IL_ACCEPT_FCU, see ilink_proto.c), against <= 447 for
 * il_decoder_feed's frame-completing byte (it copies up to 49 payload bytes). */
const il_frame_t *il_decoder_push(il_decoder_t *d, uint8_t byte);
/* bytes still needed before the open candidate can complete: 0 while hunting, LEN + 6 - n once LEN is known
 * (n >= 3), the rest of the shortest ACCEPTED frame before that (FCU / ALL: IL_CMD, 14 - n; COM: SNAP, 55 - n;
 * VTOL COM: SNAP2, 61 - n; VTOL + GPS COM: SNAP3, 68 - n; IL_NEED_NEVER if the mask accepts nothing) */
uint8_t  il_decoder_need(const il_decoder_t *d);
/* the fewest further input bytes after which the decoder can return an IL_CMD (exact: some continuation of this
 * length returns one on its last byte, no shorter one can): 14 while hunting; il_decoder_need while the candidate
 * can still be an IL_CMD; else 14 in the header (a 0xC5 as TYPE rejects and opens the next CMD), need + 12 while
 * payload bytes of a longer frame are to come (a 0xC5 as its CRC low byte), and with only CRC bytes left 14, or 15
 * if the next expected CRC byte is itself 0xC5; IL_NEED_NEVER for a decoder that does not accept IL_CMD (the
 * COM's).  The FCU's "can an IL_CMD complete in the bytes waiting?" test. */
uint8_t  il_decoder_cmd_need(const il_decoder_t *d);

#endif /* FC_ILINK_PROTO_H */
