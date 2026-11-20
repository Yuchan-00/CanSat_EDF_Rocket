/* link_proto.h - LoRa ground-link frame format (SPEC 10.3), a byte-exact port of
 * sim/link_proto.py (portable C99, no AVR headers).
 *
 * Frame (both directions, multi-byte fields little-endian):
 *
 *     0xA5 0x5A | LEN | VEH | TYPE | SEQ | PAYLOAD | CRC16
 *                 LEN counts VEH..PAYLOAD (= 3 + payload bytes)
 *                 CRC16 over LEN..PAYLOAD, transmitted low byte first
 *
 * CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, MSB first, no reflection, no
 * final XOR (check value of "123456789" = 0x29B1).
 *   uplink   LP_TYPE_CMD   (0x10)  5 bytes: cmd u8, arg u16 (throttle x1000), nonce u16
 *   downlink LP_TYPE_TELEM (0x20) 28 bytes: state, flags, reason, ack_seq u8, nonce u16, t_ms u32,
 *            h i16 (0.1 m), v_z i16 (cm/s), roll/pitch/yaw i16 (0.01 deg), tilt u16 (0.01 deg),
 *            throttle u16 (x1000), V_cell u16 (mV), loop_max u16 (us)               (TELEM v1, ballistic build)
 *   TYPE 0x21 / 0x22 were the TELEM v2 / v3 of the VTOL program (removed 2026-11-18; their packers, decoder entries
 *            and lp_telem_t fields went on 2026-11-19): unknown TYPEs now (wrong_type, as before - no rocket decoder
 *            accepted them).  LP_LEN_MAX stays 40, the decoder limit of the wire protocol (= sim/link_proto.py LEN_MAX
 *            and the ground / ESP bridges' LINK_LEN_MAX): a LEN of 32..40 is a candidate whose CRC is checked.
 *
 * Mission mode: the ballistic climb / CUT / RELEASE program only.  Codes 8..11 (the removed VTOL ON / OFF / UP / DOWN)
 * are unknown: LP_REJ_UNKNOWN_CMD, exactly what v0.6.3 answered.
 *
 * Streaming decoder (lp_decoder_feed): HUNT0 (0xA5) -> HUNT1 (0x5A; another 0xA5
 * restarts the candidate) -> LEN (LP_LEN_MIN..LP_LEN_MAX) -> BODY (LEN + 2 bytes).  A
 * failed candidate (LEN out of range or CRC mismatch) loses only its first byte; the
 * rest is parsed again from HUNT0 through a small replay queue, so a real frame that
 * starts inside garbage or inside a corrupted frame is still found.  A frame whose
 * CRC is correct is consumed whole even when it is then dropped for a wrong vehicle
 * id, TYPE or payload length (counted only).  The counters mirror the Python
 * reference (frames, bad_len, bad_crc, wrong_vehicle, wrong_type, bad_payload_len,
 * skipped_bytes).
 *
 * Quantisation (lp_quantize): value x scale rounded half away from zero, saturated,
 * NaN -> 0, computed EXACTLY in integer arithmetic from the float's bits, so a float32
 * input gives the same integer as the Python reference's float64 product (the product
 * of a 24-bit mantissa and a scale whose odd part is < 256 is exact in both).
 *
 * Invariant: lp_encode_*(lp_unpack_*(x)) == x for every valid frame, and the decoder
 * never emits a frame whose CRC, vehicle id, TYPE or payload length is wrong.
 */
#ifndef FC_LINK_PROTO_H
#define FC_LINK_PROTO_H

#include <stdbool.h>
#include <stdint.h>

#define LP_SYNC0 0xA5u
#define LP_SYNC1 0x5Au
#define LP_TYPE_CMD 0x10u
#define LP_TYPE_TELEM 0x20u
#define LP_HEADER_BYTES 3u
#define LP_CMD_PAYLOAD_LEN 5u
#define LP_TELEM_PAYLOAD_LEN 28u
#define LP_MAX_PAYLOAD_LEN 37u                                      /* the decoders' limit (wire protocol, module comment) */
#define LP_LEN_MIN LP_HEADER_BYTES
#define LP_LEN_MAX (LP_HEADER_BYTES + LP_MAX_PAYLOAD_LEN)          /* 40 */
#define LP_FRAME_OVERHEAD 5u                                        /* sync x2, LEN, CRC x2 */
#define LP_CMD_FRAME_BYTES (LP_FRAME_OVERHEAD + LP_HEADER_BYTES + LP_CMD_PAYLOAD_LEN)       /* 13 */
#define LP_TELEM_FRAME_BYTES (LP_FRAME_OVERHEAD + LP_HEADER_BYTES + LP_TELEM_PAYLOAD_LEN)   /* 36 */
#define LP_FRAME_MAX_BYTES (LP_FRAME_OVERHEAD + LP_LEN_MAX)        /* 45 */
#define LP_CRC_INIT 0xFFFFu
#define LP_CRC_POLY 0x1021u

/* mission mode (params.json mission.mode; params_gen.h MISSION_MODE_BALLISTIC has the same value, checked by fc_mode.h) */
#define LP_MODE_BALLISTIC 0u

/* uplink cmd codes */
#define LP_CMD_NONE 0u
#define LP_CMD_ARM 1u
#define LP_CMD_DISARM 2u
#define LP_CMD_LAUNCH 3u
#define LP_CMD_THROTTLE 4u
#define LP_CMD_CUT 5u
#define LP_CMD_RELEASE 6u
#define LP_CMD_KILL 7u
#define LP_CMD_MAX_BALLISTIC LP_CMD_KILL   /* 8..11 (the removed VTOL codes) are unknown: REJ_UNKNOWN_CMD, as v0.6.3 */
#define LP_CMD_MAX LP_CMD_KILL
#define LP_THROTTLE_SCALE 1000u

/* the phase byte of the snapshot / log formats: always LP_PHASE_NONE in the rocket (it carried the removed VTOL phases) */
#define LP_PHASE_NONE 0u

/* telemetry state u8 */
#define LP_STATE_SAFE 0u
#define LP_STATE_ARMED 1u
#define LP_STATE_POWERED 2u
#define LP_STATE_UNPOWERED 3u
#define LP_STATE_RELEASED 4u
#define LP_STATE_ABORT 5u

/* telemetry flag bits (SPEC 10.3 order) */
#define LP_FLAG_ARM_SWITCH 0x01u
#define LP_FLAG_SENSORS_OK 0x02u
#define LP_FLAG_ALIGNED 0x04u
#define LP_FLAG_LINK_OK 0x08u
#define LP_FLAG_RELEASED 0x10u
#define LP_FLAG_CONTROL_ACTIVE 0x20u
#define LP_FLAG_REJECTED 0x40u
#define LP_FLAG_INTERLOCK 0x80u

/* reason codes: interlocks 1..63, command rejections 0x40..0x7F (sim/link_proto.py).  The codes 14..28, 0x30..0x33 and
 * 0x4C..0x58 belonged to the removed VTOL program and are not used. */
#define LP_REASON_NONE 0u
#define LP_R_I1_LINK_LOSS 1u
#define LP_R_I1P_LINK_LOSS_PAD 2u
#define LP_R_I2_TILT 3u
#define LP_R_I3_RATE 4u
#define LP_R_I4_APOGEE_LIMIT 5u
#define LP_R_I5_GEOFENCE 6u
#define LP_R_I6_FALL 7u
#define LP_R_I7_MOTOR_TIME 8u
#define LP_R_I8_VCELL 9u
#define LP_R_I9_IMU 10u
#define LP_R_I9_BARO 11u
#define LP_R_I10_RESET 12u
#define LP_R_ARM_SWITCH_OPEN 13u
#define LP_REJ_STATE 0x40u
#define LP_REJ_NONCE 0x41u
#define LP_REJ_ARM_SWITCH 0x42u
#define LP_REJ_SENSORS 0x43u
#define LP_REJ_NOT_ALIGNED 0x44u
#define LP_REJ_TILT 0x45u
#define LP_REJ_VCELL 0x46u
#define LP_REJ_LINK 0x47u
#define LP_REJ_UNKNOWN_CMD 0x48u
#define LP_REJ_THROTTLE_ZERO 0x49u
#define LP_REJ_RELEASE_PENDING 0x4Au
#define LP_REJ_RELEASED 0x4Bu
/* telemetry scaling (wire unit per physical unit) */
#define LP_H_SCALE 10u          /* i16, 0.1 m */
#define LP_VZ_SCALE 100u        /* i16, cm/s */
#define LP_ANGLE_SCALE 100u     /* i16 roll/pitch/yaw, u16 tilt: 0.01 deg */
#define LP_VCELL_SCALE 1000u    /* u16, mV */
#define LP_I16_MIN (-32768L)
#define LP_I16_MAX 32767L
#define LP_U16_MAX 65535L

typedef struct {
    uint8_t  cmd;
    uint16_t arg;               /* THROTTLE: u x 1000 */
    uint16_t nonce;             /* LAUNCH / THROTTLE-from-ARMED: the nonce of the latest telemetry */
} lp_cmd_t;

typedef struct {
    uint8_t  state, flags, reason, ack_seq;
    uint16_t nonce;
    uint32_t t_ms;
    int16_t  h_dm, vz_cms, roll_cdeg, pitch_cdeg, yaw_cdeg;
    uint16_t tilt_cdeg, throttle_milli, vcell_mv, loop_max_us;
} lp_telem_t;
typedef struct {
    uint8_t veh, type, seq, len;              /* len = payload bytes */
    uint8_t payload[LP_MAX_PAYLOAD_LEN];
} lp_frame_t;

uint16_t lp_crc16(const uint8_t *data, uint8_t n, uint16_t crc);
bool     lp_seq_newer(uint8_t a, uint8_t b);   /* 1 <= (a - b) mod 256 <= 127 */
/* round_half_away(x * scale) saturated to [lo, hi]; NaN -> 0; exact for every float (scale's odd part < 256) */
int32_t  lp_quantize(float x, uint16_t scale, int32_t lo, int32_t hi);

void     lp_pack_cmd(const lp_cmd_t *c, uint8_t out[LP_CMD_PAYLOAD_LEN]);
void     lp_unpack_cmd(const uint8_t in[LP_CMD_PAYLOAD_LEN], lp_cmd_t *c);
void     lp_pack_telem(const lp_telem_t *t, uint8_t out[LP_TELEM_PAYLOAD_LEN]);      /* TELEM v1: 28 bytes */
void     lp_unpack_telem(const uint8_t in[LP_TELEM_PAYLOAD_LEN], lp_telem_t *t);
/* complete frame into out (>= LP_FRAME_OVERHEAD + LP_HEADER_BYTES + n bytes); returns its length, 0 if n is too big */
uint8_t  lp_encode_frame(uint8_t veh, uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t n, uint8_t *out);
uint8_t  lp_encode_cmd(uint8_t veh, uint8_t seq, const lp_cmd_t *c, uint8_t out[LP_CMD_FRAME_BYTES]);
uint8_t  lp_encode_telem(uint8_t veh, uint8_t seq, const lp_telem_t *t, uint8_t out[LP_TELEM_FRAME_BYTES]);

/* ---- streaming decoder ---- */
#define LP_ACCEPT_CMD 0x01u
#define LP_ACCEPT_TELEM 0x02u
#define LP_ANY_VEHICLE (-1)
#define LP_DEC_QUEUE (LP_FRAME_MAX_BYTES + 4u)  /* replay queue; <= LP_FRAME_MAX_BYTES + 1 are ever pending */
#define LP_DEC_MAX_OUT 4u                      /* frames one input byte can complete (<= 44 replayed bytes, >= 13 per frame) */

typedef struct {
    uint16_t frames, bad_len, bad_crc, wrong_vehicle, wrong_type, bad_payload_len, skipped_bytes;
    uint16_t overflow;                         /* firmware only: frames beyond max_out / queue overrun (0 in tests) */
} lp_dec_stats_t;

typedef struct {
    int16_t  vehicle_id;                       /* LP_ANY_VEHICLE or 0..255 */
    uint8_t  accept_mask;                      /* LP_ACCEPT_* */
    uint8_t  state, n, need;
    uint8_t  buf[LP_FRAME_MAX_BYTES];
    uint8_t  q[LP_DEC_QUEUE];
    uint8_t  q_head, q_len;
    lp_dec_stats_t stats;
} lp_decoder_t;

void    lp_decoder_init(lp_decoder_t *d, int16_t vehicle_id, uint8_t accept_mask);
void    lp_decoder_reset(lp_decoder_t *d);     /* drop any partial candidate (stats kept) */
/* feeds one byte; writes up to max_out completed valid frames to out, returns how many */
uint8_t lp_decoder_feed(lp_decoder_t *d, uint8_t byte, lp_frame_t *out, uint8_t max_out);

#endif /* FC_LINK_PROTO_H */
