/* glink_proto.h - the NAV <-> GNS link "glink" (v0.8.1 four-chip build), portable C99: a byte-exact port of
 * sim/glink_proto.py (frames, the streaming decoder, the NAV side GlinkNav).  The GNS side (the application cycle) is
 * core/gns_node.h.
 *
 * Why (architect 2026-10-21 20:30, params flight_computer.architecture_v081 / gns / glink / electronics.gns): the v0.8
 * integration measured the THREE-chip GPS build as not flyable (NAV mandatory work 98.7 % mean / 119.5 % max of the 4 ms
 * tick, FCU time-critical 83.3 % / 89.2 % with ISRs, no ilink command decoded in position hold, FCU stack free 436 B,
 * IMU -> vane up to 16.4 ms).  A FOURTH ATmega128A, the GNS board, runs the GPS receiver, UBX, the GPS-aided horizontal
 * filter (core/gps_nav.h) and the position controller (core/position_ctrl.h) at 50 Hz; the NAV keeps the attitude /
 * altitude estimators and exchanges one STATE / RESULT pair per 50 Hz cycle with the GNS; the NAV copies the RESULT into
 * the nlink v4 slot (core/nlink_proto.h nl_gns_tx_*); the FCU applies the tilt setpoint and keeps the supervisor rules
 * V11-V15 on the slot data (core/onboard.h).  The GNS makes no safety decision.
 *
 * Wire: NAV USART1 (PD3 TXD1 -> GNS PE0 RXD0, GNS PE1 TXD0 -> NAV PD2 RXD1), GLINK_BAUD (params glink.baud 250 000; UBRR
 * 3 at 16 MHz, 0 % error) 8N1, interrupt driven on both ends (RX ISR -> ring, TX ISR from a buffer; no blocking): 40 us
 * per byte.
 *
 * Frame: 0xA5 0x5A | TYPE | SEQ | PAYLOAD (fixed length per TYPE) | CRC-16/CCITT-FALSE lo hi over TYPE, SEQ and the payload
 * (il_crc16, the CRC of ilink / nlink).  Little-endian, floats IEEE-754 binary32 copied bit for bit.
 *   STATE  (NAV -> GNS, GL_TYPE_STATE, 54 B payload, frame 60 B = 2.40 ms) once per 50 Hz cycle m, SEQ = m & 0xFF:
 *     tick u32 (the NAV tick k = 5 m + 4 of the sample), fsum[3] f32 (body specific force summed over the cycle's ticks,
 *     float32 additions), n u8 (ticks in fsum), q[4] f32 (the CORRECTED attitude of tick k), w[3] f32 (tick k's
 *     bias-corrected rate), h f32 (the altitude filter after tick k), nflags u8 (GL_NF_*), fcu_state u8, pflags u8 (of the
 *     last valid FCU frame; NL_PF_SAT_HI = the OR over the cycle's valid FCU frames), kt_q u16.
 *   RESULT (GNS -> NAV, GL_TYPE_RESULT, 20 B payload, frame 26 B = 1.04 ms), SEQ = the STATE's SEQ: the five 4-byte parts
 *     of the nlink v4 slot, in order TILT, STATUS, POS, VEL, TRIM (core/nlink_proto.h).
 * Streaming decoder (gl_decoder_*, one per receiver, bounded work per byte): SYNC1 (0xA5, else skipped) -> SYNC2 (0x5A;
 * another 0xA5 keeps SYNC2; else skipped -> SYNC1) -> TYPE (a known TYPE gives the length; else bad_type -> SYNC1) -> SEQ
 * -> payload -> CRC lo -> CRC hi (wrong: bad_crc -> SYNC1); a valid frame of a TYPE this end does not accept is counted
 * wrong_dir and dropped.  No replay: a frame starting inside a rejected candidate is lost (the next 20 ms cycle replaces
 * it).  The CRC is accumulated per byte (the same CRC as the reference's over TYPE, SEQ, payload).
 *
 * NAV side (gl_nav_*, sim GlinkNav), normative order (sim/glink_proto.py module docstring "Per-tick timing"): the frame of
 * NAV tick k carries gl_nav_slot(k) (= nl_gns_tx_slot: TILT first at k_m + L); after the tick's correction part and the
 * altitude filter gl_nav_tick sums the force and ORs the FCU frames' SAT_HI, and at k mod 5 == 4 writes the STATE frame
 * for USART1; gl_nav_rx feeds the bytes the USART1 ring received (decoded in tick k) - RESULT frames go to the slot ring.
 *
 * Timing (ESTIMATES until the four-core co-simulation measures them; sim/glink_proto.py): the RESULT reaches the NAV at
 * ~ tick k + 9.4 ms -> L = 3 (params glink.latency_ticks_ASSUMED): the NAV sample of tick k reaches the attitude controller
 * at k + L + 1 = 4 ticks (16 ms; v0.8: 10).
 *
 * Invariant: gl_encode_* are the only producers of glink bytes and produce the reference's bytes; the decoders never
 * return a frame whose sync, TYPE, length or CRC is wrong.
 */
#ifndef FC_GLINK_PROTO_H
#define FC_GLINK_PROTO_H

#include "fcmath.h"
#include "nlink_proto.h"

#define GL_SYNC0 0xA5u
#define GL_SYNC1 0x5Au
#define GL_TYPE_STATE 0x01u
#define GL_TYPE_RESULT 0x02u
#define GL_STATE_LEN 54u
#define GL_RESULT_LEN 20u                                    /* = NL_GNS_RESULT_LEN */
#define GL_HEADER_BYTES 4u                                   /* sync x2, TYPE, SEQ */
#define GL_STATE_FRAME_BYTES (GL_HEADER_BYTES + GL_STATE_LEN + 2u)    /* 60 */
#define GL_RESULT_FRAME_BYTES (GL_HEADER_BYTES + GL_RESULT_LEN + 2u)  /* 26 */
#define GL_FRAME_MAX_BYTES GL_STATE_FRAME_BYTES
#define GL_NF_PAD 0x01u              /* the NAV estimator's pad mode (nlink nav_pad_mode) */
#define GL_NF_FCU_OK 0x02u           /* a valid FCU frame in this cycle */
#define GL_NF_IMU_OK 0x04u           /* tick k's IMU read succeeded */
#define GL_NF_EST_VALID 0x08u        /* the attitude comes from a pad alignment since the NAV boot */
#define GL_STEP_TICKS 5u             /* NAV ticks per 50 Hz cycle */
#define GL_GAP_RESTART_TICKS 2500L   /* a STATE gap longer than this (10 s) restarts the GNS filter's timebase */
#define GL_CLOCK_SLIP_TICKS 2L       /* the GNS's NAV-tick clock off by more than this at a STATE: the iTOW clock relearns */
#define GL_ACCEPT_STATE 0x01u        /* gl_decoder accept mask bits */
#define GL_ACCEPT_RESULT 0x02u
/* gl_reject_reason codes (sim reject_reason) */
#define GL_REJ_NONE 0u
#define GL_REJ_LENGTH 1u
#define GL_REJ_SYNC 2u
#define GL_REJ_TYPE 3u
#define GL_REJ_CRC 4u

typedef struct {                     /* sim.glink_proto.State (gl_state_t: the STATE payload, this order) */
    uint32_t tick;
    float    fsum[3];
    uint8_t  n;
    float    q[4];
    float    w[3];
    float    h;
    uint8_t  nflags, fcu_state, pflags;
    uint16_t kt_q;
} gl_state_t;

typedef struct {                     /* sim.glink_proto.Result (the five slot parts as wire integers) */
    int16_t  tilt[2];                /* tilt[0] = NL_TILT_NONE: not engaged */
    uint8_t  flags, num_sv, hacc_dm, pc;
    int16_t  pos[2], vel[2], trim[2];
} gl_result_t;

typedef struct {
    uint16_t frames, bad_crc, bad_type, wrong_dir, skipped;   /* sim DEC_STAT_KEYS order (wrap at 0xFFFF) */
} gl_stats_t;

typedef struct {                     /* sim.glink_proto.GlinkDecoder */
    uint8_t    accept;               /* GL_ACCEPT_* */
    uint8_t    st;
    uint8_t    typ, seq, need, idx, ck_lo;
    uint16_t   crc;                  /* CRC over TYPE, SEQ and the payload so far */
    uint8_t    buf[GL_STATE_LEN];    /* the payload of the frame in progress / just accepted */
    gl_stats_t stats;
} gl_decoder_t;

typedef struct {                     /* sim.glink_proto.GlinkNav (the NAV) */
    uint8_t      n;                  /* step ticks (5) */
    nl_gns_tx_t  tx;                 /* the RESULT ring and the slot phase */
    gl_decoder_t rx;                 /* USART1 bytes (accepts RESULT) */
    float        fsum[3];
    uint8_t      nsum;
    bool         sat_or, fcu_seen;
    bool         started;            /* the phase counter (j, kq) belongs to tick k */
    uint32_t     k, kq;
    uint8_t      j;
    uint16_t     states;             /* diagnostics (wrap) */
} gl_nav_t;

void    gl_pack_state(const gl_state_t *s, uint8_t out[GL_STATE_LEN]);
void    gl_unpack_state(const uint8_t in[GL_STATE_LEN], gl_state_t *s);
void    gl_pack_result(const gl_result_t *r, uint8_t out[GL_RESULT_LEN]);
void    gl_unpack_result(const uint8_t in[GL_RESULT_LEN], gl_result_t *r);
/* a whole frame from a payload: sync, TYPE, SEQ, payload, CRC; returns its length (60 / 26) */
uint8_t gl_encode(uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t *out);
uint8_t gl_encode_state(uint8_t seq, const gl_state_t *s, uint8_t out[GL_STATE_FRAME_BYTES]);
uint8_t gl_encode_result(uint8_t seq, const gl_result_t *r, uint8_t out[GL_RESULT_FRAME_BYTES]);
/* one whole buffer: GL_REJ_* (sim reject_reason); gl_decode_frame: 1 and (TYPE, SEQ, payload offset 4) when valid */
uint8_t gl_reject_reason(const uint8_t *buf, uint16_t n);
uint8_t gl_decode_frame(const uint8_t *buf, uint16_t n, uint8_t *type, uint8_t *seq);

void    gl_decoder_init(gl_decoder_t *d, uint8_t accept);
/* one received byte; returns the TYPE of an accepted frame it completed (payload in d->buf, SEQ in d->seq), else 0 */
uint8_t gl_decoder_byte(gl_decoder_t *d, uint8_t b);

void    gl_nav_init(gl_nav_t *g, uint8_t step_ticks, uint8_t latency_ticks);
/* the bytes received on USART1, decoded in NAV tick k (background); returns the RESULT frames taken */
uint8_t gl_nav_rx(gl_nav_t *g, const uint8_t *data, uint16_t n, uint32_t k);
/* the GNS slot of the nlink frame of NAV tick k (the frame build; ticks advance by one) */
void    gl_nav_slot(gl_nav_t *g, uint32_t k, uint8_t out[NL_SLOT_LEN]);
/* NAV tick k after the correction part and the altitude filter: body specific force, corrected q, bias-corrected rate, h,
 * the pad flag; fcu = the last valid FCU4 frame (NULL before the first), fcu_valid = this tick's exchange returned a valid
 * one.  At k mod 5 == 4 the STATE frame is written to out and true is returned. */
bool    gl_nav_tick(gl_nav_t *g, uint32_t k, const float accel_b[3], const float q_corr[4], const float rate_b[3], float h,
                    bool on_pad, const nl_fcu4_t *fcu, bool fcu_valid, bool imu_ok, bool est_valid,
                    uint8_t out[GL_STATE_FRAME_BYTES]);

#endif /* FC_GLINK_PROTO_H */
