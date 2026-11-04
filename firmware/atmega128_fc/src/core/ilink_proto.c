/* ilink_proto.c - see ilink_proto.h (mirrors sim/ilink_proto.py, the normative reference).
 *
 * Decoder (v0.6 phase 3): one state step per input byte plus, after a rejection, one hunt step on
 * the same byte - no replay buffer, nothing re-fed.  The candidate's LEN, TYPE and payload are kept
 * in the frame the decoder returns (d->fr) and the CRC is accumulated with il_crc16_update, so a
 * complete frame needs no second pass and il_decoder_push hands it out without a copy.
 * Measured in simavr (ATmega128 16 MHz, avr-gcc 7.3 -Os, one call per byte, 11 streams of ~1 KB:
 * uniform and protocol-rich noise, C5 5C 31 02 / C5 5C 24 03 / C5 5C 08 01 / C5 5C / C5 spam, frames
 * with bad CRC low / high bytes incl. 0xC5 ones, valid CMD / STAT / SNAP frames, every header
 * rejection with a re-examined 0xC5, a random mix; the AVR counters equal sim/ilink_proto.py's):
 *   il_decoder_push  worst byte 91 cycles, worst stream average 72 cycles per byte, worst 16
 *                    consecutive bytes 1181 cycles (74 us) - against 640 cycles of wire time per byte;
 *   il_decoder_feed  + the copy of the payload on the completing byte: worst byte 447 cycles (a SNAP,
 *                    49 bytes copied), every other byte <= 131, worst 16 consecutive bytes 2099 cycles;
 *   il_crc16         30.5 cycles per byte (51 bytes; the phase-2 nibble table 68, lp_crc16 52.5);
 *   sizeof(il_decoder_t) 67 bytes on the AVR (phase 2: 73).
 * Phase 2 for comparison: up to 6316 cycles for one byte (a 54-byte replay), 1821 cycles per byte on
 * C5 5C 31 02 spam.
 * v0.6.3 (direction filter, il_decoder_t.accept): the LEN step tests the accept mask and stores the TYPE that LEN
 * carries, so the TYPE step is one compare.  Timer1 around il_decoder_push in simavr (another bench than the
 * figures above: call included; random noise and C5 5C 08 01 / C5 5C 31 02 / C5 5C 24 03 spam): worst byte 88
 * cycles before the filter, 92 with IL_ACCEPT_FCU, 95 with IL_ACCEPT_ALL / IL_ACCEPT_COM; the simbench micro run
 * (fcu_simbench_micro) re-measures it in the FCU build.
 * v0.7 (SNAP2, IL_TYPE_SNAP2 / LEN 55, the VTOL build's snapshot): one more compare in the LEN step, the payload
 * buffer grows to 55 bytes (sizeof(il_decoder_t) 73 on the AVR); the cycle figures above were not re-measured.
 * v0.8 (SNAP3, IL_TYPE_SNAP3 / LEN 62, the VTOL + GPS build): one more compare in the LEN and TYPE steps, the payload
 * buffer grows to 62 bytes (sizeof(il_decoder_t) 80 on the AVR).
 * v0.9 RP1 (the power tail): SNAP2 57, SNAP3 65 bytes (ilink_proto.h) - the payload buffer grows to 65 bytes
 * (sizeof(il_decoder_t) 83 on the AVR VTOL images; the ballistic images keep 80); no new compare.
 *
 * Invariant: il_decoder_feed / il_decoder_push return exactly the frames sim/ilink_proto.py
 * IlinkDecoder(accept) returns for the same bytes and accept mask, the counters are equal and
 * il_decoder_need / il_decoder_cmd_need equal IlinkDecoder.need / cmd_need after every byte (host test
 * vectors + tools/crosscheck_controller.py --only ilink, with the ALL, FCU, COM and VTOL COM masks).
 */
#include "ilink_proto.h"

#if IL_ACCEPT_COM_VTOL != IL_ACCEPT_SNAP2 || IL_ACCEPT_SNAP2 != (1u << (IL_TYPE_SNAP2 - 1u))
#error "IL_ACCEPT_COM_VTOL must be the SNAP2 accept bit (bit TYPE - 1)"
#endif
#if IL_ACCEPT_COM_GPS != IL_ACCEPT_SNAP3 || IL_ACCEPT_SNAP3 != (1u << (IL_TYPE_SNAP3 - 1u))
#error "IL_ACCEPT_COM_GPS must be the SNAP3 accept bit (bit TYPE - 1)"
#endif

#if !defined(__BYTE_ORDER__) || __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "ilink_proto copies IEEE single bit patterns through a union (float and integer byte order must agree: AVR, x86, ARM)"
#endif
typedef char il_float_is_binary32[(sizeof(float) == 4u && sizeof(uint32_t) == 4u) ? 1 : -1];

/* ---- CRC-16/CCITT-FALSE: il_crc16_update (ilink_proto.h) is link_proto.c's branch-free update written on the
 * register bytes.  The phase-2 nibble table cost 68 cycles per byte in il_crc16 (simavr, avr-gcc 7.3 -Os) against 52
 * for lp_crc16; the protocol fixes the CRC value, not the method (sim/test_ilink_proto.py and the host test check the
 * update against the bitwise definition). ---- */
uint16_t il_crc16(const uint8_t *data, uint8_t n, uint16_t crc)
{
    uint8_t i;
    for (i = 0; i < n; i++)
        crc = il_crc16_update(crc, data[i]);
    return crc;
}

uint8_t il_payload_len(uint8_t type)
{
    switch (type) {
    case IL_TYPE_CMD:
        return IL_CMD_LEN;
    case IL_TYPE_SNAP:
        return IL_SNAP_LEN;
    case IL_TYPE_STAT:
        return IL_STAT_LEN;
    case IL_TYPE_SNAP2:
        return IL_SNAP2_LEN;
    case IL_TYPE_SNAP3:
        return IL_SNAP3_LEN;
    default:
        return 0;
    }
}

uint8_t il_encode(uint8_t type, const uint8_t *payload, uint8_t n, uint8_t *out)
{
    uint8_t i;
    uint16_t crc;
    const uint8_t len = il_payload_len(type);
    if (len == 0u || n != len)
        return 0;
    out[0] = IL_SYNC0;
    out[1] = IL_SYNC1;
    out[2] = n;
    out[3] = type;
    for (i = 0; i < n; i++)
        out[4u + i] = payload[i];
    crc = il_crc16(out + 2, (uint8_t)(n + 2u), LP_CRC_INIT);
    out[4u + n] = (uint8_t)crc;
    out[5u + n] = (uint8_t)(crc >> 8);
    return (uint8_t)(IL_FRAME_OVERHEAD + n);
}

/* ---- payloads: explicit little-endian, floats as their 4-byte IEEE pattern ---- */
typedef union {
    float f;
    uint32_t u;
} il_f32_t;

static uint8_t *put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    return p + 2;
}

static uint8_t *put32(uint8_t *p, uint32_t v)
{
    return put16(put16(p, (uint16_t)v), (uint16_t)(v >> 16));
}

static uint8_t *putf(uint8_t *p, float f)
{
    il_f32_t v;
    v.f = f;
    return put32(p, v.u);
}

static uint16_t get16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)get16(p) | ((uint32_t)get16(p + 2) << 16);
}

static float getf(const uint8_t *p)
{
    il_f32_t v;
    v.u = get32(p);
    return v.f;
}

void il_pack_cmd(const il_cmd_t *c, uint8_t out[IL_CMD_LEN])
{
    uint8_t *p = out;
    *p++ = c->seq;
    *p++ = c->cmd.cmd;
    p = put16(p, c->cmd.arg);
    p = put16(p, c->cmd.nonce);
    *p++ = (uint8_t)c->rssi_dbm;
    *p = (uint8_t)c->snr_q4;
}

void il_unpack_cmd(const uint8_t in[IL_CMD_LEN], il_cmd_t *c)
{
    c->seq = in[0];
    c->cmd.cmd = in[1];
    c->cmd.arg = get16(in + 2);
    c->cmd.nonce = get16(in + 4);
    c->rssi_dbm = (int8_t)in[6];
    c->snr_q4 = (int8_t)in[7];
}

void il_pack_snap(const il_snap_t *s, uint8_t out[IL_SNAP_LEN])
{
    uint8_t *p = out, i;
    *p++ = s->reply_seq;
    *p++ = s->state;
    *p++ = s->flags;
    *p++ = s->reason;
    *p++ = s->ack_seq;
    p = put16(p, s->nonce);
    p = put16(p, s->throttle_milli);
    p = put32(p, s->tick);
    p = putf(p, s->h_m);
    p = putf(p, s->vz_mps);
    p = putf(p, s->tilt_deg);
    p = putf(p, s->v_cell);
    p = putf(p, s->loop_max_us);
    for (i = 0; i < 4u; i++)
        p = putf(p, s->q[i]);
}

void il_unpack_snap(const uint8_t in[IL_SNAP_LEN], il_snap_t *s)
{
    uint8_t i;
    s->reply_seq = in[0];
    s->state = in[1];
    s->flags = in[2];
    s->reason = in[3];
    s->ack_seq = in[4];
    s->nonce = get16(in + 5);
    s->throttle_milli = get16(in + 7);
    s->tick = get32(in + 9);
    s->h_m = getf(in + 13);
    s->vz_mps = getf(in + 17);
    s->tilt_deg = getf(in + 21);
    s->v_cell = getf(in + 25);
    s->loop_max_us = getf(in + 29);
    for (i = 0; i < 4u; i++)
        s->q[i] = getf(in + 33 + 4u * i);
}

void il_pack_snap2(const il_snap2_t *s, uint8_t out[IL_SNAP2_LEN])
{
    il_pack_snap(&s->s, out);                          /* the 49 SNAP bytes, then phase u8, spare u8, h_tgt_m f32 */
    out[IL_SNAP_LEN] = s->phase;
    out[IL_SNAP_LEN + 1u] = s->spare;
    (void)putf(out + IL_SNAP_LEN + 2u, s->h_tgt_m);
#if LP_RP1
    out[IL_SNAP_LEN + 6u] = s->pw_vaux;                /* v0.9: the power tail */
    out[IL_SNAP_LEN + 7u] = s->pw_flags;
#endif
}

void il_unpack_snap2(const uint8_t in[IL_SNAP2_LEN], il_snap2_t *s)
{
    il_unpack_snap(in, &s->s);
    s->phase = in[IL_SNAP_LEN];
    s->spare = in[IL_SNAP_LEN + 1u];
    s->h_tgt_m = getf(in + IL_SNAP_LEN + 2u);
#if LP_RP1
    s->pw_vaux = in[IL_SNAP_LEN + 6u];
    s->pw_flags = in[IL_SNAP_LEN + 7u];
#else
    s->pw_vaux = s->pw_flags = 0;
#endif
}

void il_pack_snap3(const il_snap3_t *s, uint8_t out[IL_SNAP3_LEN])
{
    il_pack_snap(&s->s, out);                          /* the 49 SNAP bytes, then phase, gps_flags, h_tgt_m, num_sv, */
    out[IL_SNAP_LEN] = s->phase;                       /* hacc_dm, pos_mode, dist_m */
    out[IL_SNAP_LEN + 1u] = s->gps_flags;
    (void)putf(out + IL_SNAP_LEN + 2u, s->h_tgt_m);
    out[IL_SNAP_LEN + 6u] = s->num_sv;
    out[IL_SNAP_LEN + 7u] = s->hacc_dm;
    out[IL_SNAP_LEN + 8u] = s->pos_mode;
    (void)putf(out + IL_SNAP_LEN + 9u, s->dist_m);
#if LP_RP1
    out[IL_SNAP_LEN + 13u] = s->pw_vaux;               /* v0.9: the power tail, then the reserved byte */
    out[IL_SNAP_LEN + 14u] = s->pw_flags;
    out[IL_SNAP_LEN + 15u] = s->spare3;
#endif
}

void il_unpack_snap3(const uint8_t in[IL_SNAP3_LEN], il_snap3_t *s)
{
    il_unpack_snap(in, &s->s);
    s->phase = in[IL_SNAP_LEN];
    s->gps_flags = in[IL_SNAP_LEN + 1u];
    s->h_tgt_m = getf(in + IL_SNAP_LEN + 2u);
    s->num_sv = in[IL_SNAP_LEN + 6u];
    s->hacc_dm = in[IL_SNAP_LEN + 7u];
    s->pos_mode = in[IL_SNAP_LEN + 8u];
    s->dist_m = getf(in + IL_SNAP_LEN + 9u);
#if LP_RP1
    s->pw_vaux = in[IL_SNAP_LEN + 13u];
    s->pw_flags = in[IL_SNAP_LEN + 14u];
    s->spare3 = in[IL_SNAP_LEN + 15u];
#else
    s->pw_vaux = s->pw_flags = s->spare3 = 0;
#endif
}

void il_pack_stat(const il_stat_t *s, uint8_t out[IL_STAT_LEN])
{
    uint8_t *p = put32(out, s->com_ticks);
    *p++ = s->reset_cause;
    *p++ = s->flags;
    *p++ = s->radio_version;
    *p++ = (uint8_t)s->rssi_dbm;
    *p++ = (uint8_t)s->snr_q4;
    *p++ = s->params_tag;
    p = put16(p, s->dec_frames);
    p = put16(p, s->dec_bad_crc);
    p = put16(p, s->dec_bad_len);
    p = put16(p, s->dec_other);
    p = put16(p, s->cmd_forwarded);
    p = put16(p, s->radio_rx_errors);
    p = put16(p, s->radio_rx_overflow);
    p = put16(p, s->radio_tx_timeouts);
    p = put16(p, s->replies);
    p = put16(p, s->replies_dropped);
    p = put16(p, s->il_bad);
    p = put16(p, s->il_tx_dropped);
    (void)put16(p, s->loop_us_max);
}

void il_unpack_stat(const uint8_t in[IL_STAT_LEN], il_stat_t *s)
{
    s->com_ticks = get32(in);
    s->reset_cause = in[4];
    s->flags = in[5];
    s->radio_version = in[6];
    s->rssi_dbm = (int8_t)in[7];
    s->snr_q4 = (int8_t)in[8];
    s->params_tag = in[9];
    s->dec_frames = get16(in + 10);
    s->dec_bad_crc = get16(in + 12);
    s->dec_bad_len = get16(in + 14);
    s->dec_other = get16(in + 16);
    s->cmd_forwarded = get16(in + 18);
    s->radio_rx_errors = get16(in + 20);
    s->radio_rx_overflow = get16(in + 22);
    s->radio_tx_timeouts = get16(in + 24);
    s->replies = get16(in + 26);
    s->replies_dropped = get16(in + 28);
    s->il_bad = get16(in + 30);
    s->il_tx_dropped = get16(in + 32);
    s->loop_us_max = get16(in + 34);
}

/* ---- FCU snapshot <-> COM telemetry source ---- */
void il_snap_from_tlm_src(uint8_t reply_seq, const onb_tlm_src_t *src, const float q[4], il_snap_t *s)
{
    uint8_t i;
    s->reply_seq = reply_seq;
    s->state = src->state;
    s->flags = src->flags;
    s->reason = src->reason;
    s->ack_seq = src->ack_seq;
    s->nonce = src->nonce;
    s->throttle_milli = src->throttle_milli;
    s->tick = src->tick;
    s->h_m = src->h_m;
    s->vz_mps = src->vz_mps;
    s->tilt_deg = src->tilt_deg;
    s->v_cell = src->v_cell;
    s->loop_max_us = src->loop_max_us;
    for (i = 0; i < 4u; i++)
        s->q[i] = q[i];                                /* src's phase / h_tgt_m: il_snap2_from_tlm_src */
}

void il_snap_to_tlm_src(const il_snap_t *s, onb_tlm_src_t *src)
{
    const float *q = s->q;
    src->state = s->state;
    src->flags = s->flags;
    src->reason = s->reason;
    src->ack_seq = s->ack_seq;
    src->nonce = s->nonce;
    src->throttle_milli = s->throttle_milli;
    src->tick = s->tick;
    src->h_m = s->h_m;
    src->vz_mps = s->vz_mps;
    src->tilt_deg = s->tilt_deg;
    src->v_cell = s->v_cell;
    src->loop_max_us = s->loop_max_us;
    /* the v0.5 FCU link_step formulas (ZYX roll / pitch / yaw of q = w, x, y, z), float32 */
    src->roll_deg = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2])) *
                    FC_RAD2DEG_F;
    src->pitch_deg = asinf(fc_clampf(2.0f * (q[0] * q[2] - q[3] * q[1]), -1.0f, 1.0f)) * FC_RAD2DEG_F;
    src->yaw_deg = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3])) *
                   FC_RAD2DEG_F;
    src->v2 = false;
    src->phase = 0;
    src->h_tgt_m = 0.0f;
    src->v3 = false;
    src->gps_flags = src->num_sv = src->hacc_dm = src->pos_mode = 0;
    src->dist_m = 0.0f;
#if PM_ANY
    src->pw_vaux = src->pw_flags = 0;                  /* v0.9: no power tail in a SNAP */
#endif
}

void il_snap2_from_tlm_src(uint8_t reply_seq, const onb_tlm_src_t *src, const float q[4], il_snap2_t *s)
{
    il_snap_from_tlm_src(reply_seq, src, q, &s->s);
    s->phase = src->phase;
    s->spare = 0;
    s->h_tgt_m = src->h_tgt_m;
#if PM_ANY
    s->pw_vaux = src->pw_vaux;                         /* v0.9: the power tail (0, 0 in a battery build's source) */
    s->pw_flags = src->pw_flags;
#else
    s->pw_vaux = s->pw_flags = 0;                      /* the battery image: no power monitor */
#endif
}

void il_snap2_to_tlm_src(const il_snap2_t *s, onb_tlm_src_t *src)
{
    il_snap_to_tlm_src(&s->s, src);                    /* spare is ignored */
    src->v2 = true;
    src->phase = s->phase;
    src->h_tgt_m = s->h_tgt_m;
#if PM_ANY
    src->pw_vaux = s->pw_vaux;                         /* (the COM image has no PM_ANY: com_link.c reads the SNAP's bytes) */
    src->pw_flags = s->pw_flags;
#endif
}

void il_snap3_from_tlm_src(uint8_t reply_seq, const onb_tlm_src_t *src, const float q[4], il_snap3_t *s)
{
    il_snap_from_tlm_src(reply_seq, src, q, &s->s);
    s->phase = src->phase;
    s->gps_flags = src->gps_flags;
    s->h_tgt_m = src->h_tgt_m;
    s->num_sv = src->num_sv;
    s->hacc_dm = src->hacc_dm;
    s->pos_mode = src->pos_mode;
    s->dist_m = src->dist_m;
#if PM_ANY
    s->pw_vaux = src->pw_vaux;
    s->pw_flags = src->pw_flags;
#else
    s->pw_vaux = s->pw_flags = 0;
#endif
    s->spare3 = 0;
}

void il_snap3_to_tlm_src(const il_snap3_t *s, onb_tlm_src_t *src)
{
    il_snap_to_tlm_src(&s->s, src);
    src->v2 = true;
    src->phase = s->phase;
    src->h_tgt_m = s->h_tgt_m;
    src->v3 = true;
    src->gps_flags = s->gps_flags;
    src->num_sv = s->num_sv;
    src->hacc_dm = s->hacc_dm;
    src->pos_mode = s->pos_mode;
    src->dist_m = s->dist_m;
#if PM_ANY
    src->pw_vaux = s->pw_vaux;                         /* spare3 is ignored */
    src->pw_flags = s->pw_flags;
#endif
}

/* ---- streaming decoder (rules: sim/ilink_proto.py, normative) ---- */
void il_decoder_init(il_decoder_t *d, uint8_t accept)
{
    uint8_t i;
    d->n = 0;
    d->accept = (uint8_t)(accept & IL_ACCEPT_ALL8);
    d->crc = LP_CRC_INIT;
    d->fr.type = 0;
    d->fr.len = 0;
    for (i = 0; i < IL_MAX_PAYLOAD; i++)
        d->fr.payload[i] = 0;
    d->stats.frames = d->stats.bad_crc = d->stats.bad_len = d->stats.bad_type = 0;
    d->stats.skipped_bytes = d->stats.overflow = 0;
}

/* One input byte: true when it completes a frame (in d->fr).  Every path is a handful of compares and stores, at most
 * one il_crc16_update; a rejection costs one more hunt step on the same byte - nothing is replayed.  Written with one
 * CRC site and one "accept" tail so avr-gcc -Os inlines the update and keeps the function small. */
static bool dec_step(il_decoder_t *d, uint8_t b)
{
    const uint8_t n = d->n;
    uint16_t crc = d->crc;
    if (n >= 4u) {
        const uint8_t k = (uint8_t)(n - 4u), len = d->fr.len;
        if (k < len) {                                 /* payload */
            d->fr.payload[k] = b;
            goto accept_crc;
        }
        if (k == len) {                                /* CRC low byte: checked when it arrives */
            if (b == (uint8_t)crc)
                goto accept;
        } else if (b == (uint8_t)(crc >> 8)) {         /* CRC high byte: the frame is complete */
            d->stats.frames++;
            d->n = 0;
            return true;
        }
        d->stats.bad_crc++;
        goto reject;
    }
    if (n == 0u) {                                     /* hunting */
        if (b == IL_SYNC0)
            goto accept;
        d->stats.skipped_bytes++;
        return false;
    }
    if (n == 1u) {
        if (b == IL_SYNC1)
            goto accept;
        goto reject;                                   /* no reason counter */
    }
    if (n == 2u) {                                     /* LEN: the fixed length of an accepted TYPE */
        uint8_t t, bit;
        if (b == IL_CMD_LEN) {
            t = IL_TYPE_CMD;
            bit = IL_ACCEPT_CMD;
        } else if (b == IL_STAT_LEN) {
            t = IL_TYPE_STAT;
            bit = IL_ACCEPT_STAT;
        } else if (b == IL_SNAP_LEN) {
            t = IL_TYPE_SNAP;
            bit = IL_ACCEPT_SNAP;
        } else if (b == IL_SNAP2_LEN) {
            t = IL_TYPE_SNAP2;
            bit = IL_ACCEPT_SNAP2;
        } else if (b == IL_SNAP3_LEN) {
            t = IL_TYPE_SNAP3;
            bit = IL_ACCEPT_SNAP3;
        } else {
            bit = 0u;
            t = 0u;
        }
        if (bit & d->accept) {
            d->fr.len = b;
            d->fr.type = t;                            /* the one TYPE this LEN can carry (LENs are distinct) */
            crc = LP_CRC_INIT;
            goto accept_crc;
        }
        d->stats.bad_len++;
        goto reject;
    }
    if (b == d->fr.type)                               /* n == 3, TYPE: the type of the accepted LEN */
        goto accept_crc;
    {                                                  /* refused: an accepted TYPE of another LEN, or not accepted */
        const uint8_t bit = b == IL_TYPE_CMD ? IL_ACCEPT_CMD : (b == IL_TYPE_SNAP ? IL_ACCEPT_SNAP :
                            (b == IL_TYPE_STAT ? IL_ACCEPT_STAT : (b == IL_TYPE_SNAP2 ? IL_ACCEPT_SNAP2 :
                            (b == IL_TYPE_SNAP3 ? IL_ACCEPT_SNAP3 : 0u))));
        if (bit & d->accept)
            d->stats.bad_len++;
        else
            d->stats.bad_type++;
    }
    /* reject: the candidate's n bytes are discarded, the rejecting byte is examined once more as a possible SYNC0 */
reject:
    d->stats.skipped_bytes = (uint16_t)(d->stats.skipped_bytes + n);
    if (b == IL_SYNC0) {
        d->n = 1;
    } else {
        d->n = 0;
        d->stats.skipped_bytes++;
    }
    return false;
accept_crc:
    d->crc = il_crc16_update(crc, b);
accept:
    d->n = (uint8_t)(n + 1u);
    return false;
}

const il_frame_t *il_decoder_push(il_decoder_t *d, uint8_t byte)
{
    return dec_step(d, byte) ? &d->fr : 0;
}

uint8_t il_decoder_feed(il_decoder_t *d, uint8_t byte, il_frame_t *out, uint8_t max_out)
{
    const il_frame_t *f = il_decoder_push(d, byte);
    const uint8_t *src;
    uint8_t *dst, k;
    if (f == 0)
        return 0;
    if (max_out == 0u) {
        d->stats.overflow++;
        return 0;
    }
    k = f->len;
    out->type = f->type;
    out->len = k;
    src = f->payload;
    dst = out->payload;
    do {                                               /* LEN is 8, 36, 49, 55 or 62, never 0 */
        *dst++ = *src++;
    } while (--k != 0u);
    return 1;
}

uint8_t il_decoder_need(const il_decoder_t *d)
{
    const uint8_t n = d->n, a = d->accept;
    if (n == 0u)
        return 0;
    if (n < 3u) {                                      /* LEN not known yet: the rest of the shortest accepted frame */
        const uint8_t shortest = (a & IL_ACCEPT_CMD) ? IL_CMD_FRAME_BYTES : ((a & IL_ACCEPT_STAT) ? IL_STAT_FRAME_BYTES :
                                 ((a & IL_ACCEPT_SNAP) ? IL_SNAP_FRAME_BYTES : ((a & IL_ACCEPT_SNAP2) ? IL_SNAP2_FRAME_BYTES :
                                 ((a & IL_ACCEPT_SNAP3) ? IL_SNAP3_FRAME_BYTES : 0u))));
        return shortest != 0u ? (uint8_t)(shortest - n) : IL_NEED_NEVER;
    }
    return (uint8_t)(d->fr.len + IL_FRAME_OVERHEAD - n);
}

uint8_t il_decoder_cmd_need(const il_decoder_t *d)
{
    const uint8_t n = d->n, need = il_decoder_need(d);
    uint8_t next_crc;
    if (!(d->accept & IL_ACCEPT_CMD))
        return IL_NEED_NEVER;                          /* the COM's decoder never returns an IL_CMD */
    if (n == 0u)
        return IL_CMD_FRAME_BYTES;                     /* the next byte can be a CMD's 0xC5 */
    if (n < 3u || d->fr.len == IL_CMD_LEN)
        return need;                                   /* the candidate itself can still be an IL_CMD */
    if (n == 3u)
        return IL_CMD_FRAME_BYTES;                     /* a 0xC5 as TYPE rejects (bad_type) and opens the next CMD */
    if (need >= 3u)                                    /* a payload byte is left, so the CRC low byte can differ from */
        return (uint8_t)(need + IL_CMD_FRAME_BYTES - 2u);   /* 0xC5: a 0xC5 there (need - 1 bytes away) rejects + opens */
    next_crc = need == 2u ? (uint8_t)d->crc : (uint8_t)(d->crc >> 8);
    return next_crc != IL_SYNC0 ? IL_CMD_FRAME_BYTES   /* the next CRC byte: a 0xC5 rejects and opens */
                                : (uint8_t)(IL_CMD_FRAME_BYTES + 1u);   /* a 0xC5 would be accepted: one byte later */
}
