/* link_proto.c - see link_proto.h (mirrors sim/link_proto.py). */
#include "link_proto.h"
#include "../fc_mode.h"       /* P1: FC_VTOL */

/* CRC-16/CCITT-FALSE, one byte: the branch-free form of the bitwise MSB-first update with
 * poly 0x1021 (x = (crc >> 8) ^ b; x ^= x >> 4; crc = (crc << 8) ^ x << 12 ^ x << 5 ^ x) */
static uint16_t crc_byte(uint16_t crc, uint8_t b)
{
    uint8_t x = (uint8_t)((crc >> 8) ^ b);
    x ^= (uint8_t)(x >> 4);
    return (uint16_t)((crc << 8) ^ ((uint16_t)x << 12) ^ ((uint16_t)x << 5) ^ (uint16_t)x);
}

uint16_t lp_crc16(const uint8_t *data, uint8_t n, uint16_t crc)
{
    uint8_t i;
    for (i = 0; i < n; i++)
        crc = crc_byte(crc, data[i]);
    return crc;
}

bool lp_seq_newer(uint8_t a, uint8_t b)
{
    uint8_t d = (uint8_t)(a - b);
    return d >= 1u && d <= 127u;
}

#if !defined(__BYTE_ORDER__) || __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "lp_quantize reads the IEEE single bit pattern through a union (little-endian AVR, x86, ARM)"
#endif

#if FC_VTOL
/* P1 (fw09/p1): p >> n by whole bytes first (the -Os code of a variable 32-bit shift is a one-bit loop) */
static inline __attribute__((always_inline)) uint32_t lp_shr32(uint32_t p, uint8_t n)
{
    if (n >= 16u) {
        p >>= 16;
        n = (uint8_t)(n - 16u);
    }
    if (n >= 8u) {
        p >>= 8;
        n = (uint8_t)(n - 8u);
    }
    return p >> n;
}
#endif

int32_t lp_quantize(float x, uint16_t scale, int32_t lo, int32_t hi)
{
    union {
        float f;
        uint32_t u;
#if FC_VTOL
        uint16_t w[2];
        uint8_t b[4];
#endif
    } v;
    uint32_t mant, odd, p, r;
    int16_t e2;
    uint8_t ex;
    bool neg;
    v.f = x;
#if FC_VTOL
    neg = (v.b[3] & 0x80u) != 0u;                        /* P1: from the bytes (little-endian, checked above) */
    ex = (uint8_t)((uint8_t)(v.b[3] << 1) | (uint8_t)(v.b[2] >> 7));
#else
    neg = (v.u >> 31) != 0u;
    ex = (uint8_t)(v.u >> 23);
#endif
    if (ex == 0xFFu)                                     /* inf saturates, NaN -> 0 */
        return (v.u & 0x7FFFFFu) ? 0 : (neg ? lo : hi);
    if (ex == 0u || scale == 0u)                         /* zero / denormal: |x scale| < 0.5 */
        return 0;
#if FC_VTOL
    mant = ((uint32_t)(uint8_t)(v.b[2] | 0x80u) << 16) | v.w[0];   /* |x| = mant 2^(ex - 150) */
#else
    mant = (v.u & 0x7FFFFFu) | 0x800000u;                /* |x| = mant 2^(ex - 150) */
#endif
    e2 = (int16_t)((int16_t)ex - 150);
    odd = scale;
    while ((odd & 1u) == 0u) {                           /* scale = odd 2^k: k goes into the exponent */
        odd >>= 1;
        e2++;
    }
    if (odd > 255u) {                                    /* not exact any more: plain float (no protocol scale gets here) */
        float q = x * (float)scale;
        if (q >= (float)hi)
            return hi;
        if (q <= (float)lo)
            return lo;
        return q >= 0.0f ? (int32_t)(q + 0.5f) : -(int32_t)(-q + 0.5f);
    }
    p = mant * odd;                                      /* < 2^24 * 2^8: exact in 32 bits */
    if (e2 >= 0) {
        r = (e2 >= 32 || p > (0xFFFFFFFFu >> e2)) ? 0xFFFFFFFFu : (p << e2);
    } else {
        uint8_t n = (uint8_t)(-e2);
        if (n > 32u) {
            r = 0;                                       /* |x scale| < 2^32 / 2^33 = 0.5 */
        } else {
#if FC_VTOL
            uint32_t h = lp_shr32(p, (uint8_t)(n - 1u));  /* floor(2 |v|) */
#else
            uint32_t h = p >> (n - 1u);                  /* floor(2 |v|) */
#endif
            r = (h >> 1) + (h & 1u);                     /* floor(|v| + 1/2): half away from zero */
        }
    }
    if (!neg)
        return r > (uint32_t)hi ? hi : (int32_t)r;
    {
        uint32_t lim = (uint32_t)(-(lo + 1)) + 1u;       /* -lo without overflow (0 for lo = 0) */
        if (r >= lim)
            return lo;
        return -(int32_t)r;
    }
}

int32_t lp_quantize_wide(float x, uint16_t scale, int32_t lo, int32_t hi)
{
    union {
        float f;
        uint32_t u;
    } v;
    uint64_t p, r;
    int16_t e2;
    uint8_t ex;
    bool neg;
    v.f = x;
    neg = (v.u >> 31) != 0u;
    ex = (uint8_t)(v.u >> 23);
    if (ex == 0xFFu)                                     /* inf saturates, NaN -> 0 */
        return (v.u & 0x7FFFFFu) ? 0 : (neg ? lo : hi);
    if (ex == 0u || scale == 0u)                         /* zero / denormal: |x scale| < 0.5 */
        return 0;
    p = (uint64_t)((v.u & 0x7FFFFFu) | 0x800000u) * scale;   /* |x scale| = p 2^(ex - 150), p < 2^40: exact */
    e2 = (int16_t)((int16_t)ex - 150);
    if (e2 >= 0) {
        r = (e2 >= 24 || p > (0xFFFFFFFFFFFFFFFFull >> e2)) ? 0xFFFFFFFFull : (p << e2);   /* >= 2^40 saturates anyway */
        if (r > 0xFFFFFFFFull)
            r = 0xFFFFFFFFull;
    } else {
        uint8_t n = (uint8_t)(-e2);
        if (n > 41u) {
            r = 0;                                       /* |x scale| < 2^40 / 2^42 < 0.5 */
        } else {
            uint64_t h = p >> (n - 1u);                  /* floor(2 |v|) */
            r = (h >> 1) + (h & 1u);                     /* floor(|v| + 1/2): half away from zero */
        }
    }
    if (!neg)
        return r > (uint64_t)(uint32_t)hi ? hi : (int32_t)r;
    {
        uint32_t lim = (uint32_t)(-(lo + 1)) + 1u;       /* -lo without overflow (0 for lo = 0) */
        if (r >= lim)
            return lo;
        return -(int32_t)r;
    }
}

/* ---- payloads ---- */
static uint8_t *put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    return p + 2;
}

static uint16_t get16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

void lp_pack_cmd(const lp_cmd_t *c, uint8_t out[LP_CMD_PAYLOAD_LEN])
{
    out[0] = c->cmd;
    put16(put16(out + 1, c->arg), c->nonce);
}

void lp_unpack_cmd(const uint8_t in[LP_CMD_PAYLOAD_LEN], lp_cmd_t *c)
{
    c->cmd = in[0];
    c->arg = get16(in + 1);
    c->nonce = get16(in + 3);
}

void lp_pack_telem(const lp_telem_t *t, uint8_t out[LP_TELEM_PAYLOAD_LEN])
{
    uint8_t *p = out;
    *p++ = t->state;
    *p++ = t->flags;
    *p++ = t->reason;
    *p++ = t->ack_seq;
    p = put16(p, t->nonce);
    p = put16(p, (uint16_t)t->t_ms);
    p = put16(p, (uint16_t)(t->t_ms >> 16));
    p = put16(p, (uint16_t)t->h_dm);
    p = put16(p, (uint16_t)t->vz_cms);
    p = put16(p, (uint16_t)t->roll_cdeg);
    p = put16(p, (uint16_t)t->pitch_cdeg);
    p = put16(p, (uint16_t)t->yaw_cdeg);
    p = put16(p, t->tilt_cdeg);
    p = put16(p, t->throttle_milli);
    p = put16(p, t->vcell_mv);
    (void)put16(p, t->loop_max_us);
}

void lp_pack_telem2(const lp_telem_t *t, uint8_t out[LP_TELEM2_PAYLOAD_LEN])
{
    lp_pack_telem(t, out);                             /* the 28 TELEM v1 bytes, then phase u8 and h_tgt i16 */
    out[LP_TELEM_PAYLOAD_LEN] = t->phase;
    (void)put16(out + LP_TELEM_PAYLOAD_LEN + 1u, (uint16_t)t->h_tgt_dm);
}

void lp_unpack_telem2(const uint8_t in[LP_TELEM2_PAYLOAD_LEN], lp_telem_t *t)
{
    lp_unpack_telem(in, t);
    t->phase = in[LP_TELEM_PAYLOAD_LEN];
    t->h_tgt_dm = (int16_t)get16(in + LP_TELEM_PAYLOAD_LEN + 1u);
}

/* v0.8 TELEM v3: the 31 TELEM v2 bytes, then gps_flags, num_sv, hacc_dm, pos_mode u8 and dist_dm u16 */
void lp_pack_telem3(const lp_telem_t *t, uint8_t out[LP_TELEM3_PAYLOAD_LEN])
{
    lp_pack_telem2(t, out);
    out[LP_TELEM2_PAYLOAD_LEN] = t->gps_flags;
    out[LP_TELEM2_PAYLOAD_LEN + 1u] = t->num_sv;
    out[LP_TELEM2_PAYLOAD_LEN + 2u] = t->hacc_dm;
    out[LP_TELEM2_PAYLOAD_LEN + 3u] = t->pos_mode;
    (void)put16(out + LP_TELEM2_PAYLOAD_LEN + 4u, t->dist_dm);
}

void lp_unpack_telem3(const uint8_t in[LP_TELEM3_PAYLOAD_LEN], lp_telem_t *t)
{
    lp_unpack_telem2(in, t);
    t->gps_flags = in[LP_TELEM2_PAYLOAD_LEN];
    t->num_sv = in[LP_TELEM2_PAYLOAD_LEN + 1u];
    t->hacc_dm = in[LP_TELEM2_PAYLOAD_LEN + 2u];
    t->pos_mode = in[LP_TELEM2_PAYLOAD_LEN + 3u];
    t->dist_dm = get16(in + LP_TELEM2_PAYLOAD_LEN + 4u);
}

void lp_unpack_telem(const uint8_t in[LP_TELEM_PAYLOAD_LEN], lp_telem_t *t)
{
    t->phase = 0;
    t->h_tgt_dm = 0;
    t->gps_flags = t->num_sv = t->hacc_dm = t->pos_mode = 0;
    t->dist_dm = 0;
    t->state = in[0];
    t->flags = in[1];
    t->reason = in[2];
    t->ack_seq = in[3];
    t->nonce = get16(in + 4);
    t->t_ms = (uint32_t)get16(in + 6) | ((uint32_t)get16(in + 8) << 16);
    t->h_dm = (int16_t)get16(in + 10);
    t->vz_cms = (int16_t)get16(in + 12);
    t->roll_cdeg = (int16_t)get16(in + 14);
    t->pitch_cdeg = (int16_t)get16(in + 16);
    t->yaw_cdeg = (int16_t)get16(in + 18);
    t->tilt_cdeg = get16(in + 20);
    t->throttle_milli = get16(in + 22);
    t->vcell_mv = get16(in + 24);
    t->loop_max_us = get16(in + 26);
}

uint8_t lp_encode_frame(uint8_t veh, uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t n, uint8_t *out)
{
    uint8_t i;
    uint16_t crc;
    if (n > LP_MAX_PAYLOAD_LEN)
        return 0;
    out[0] = LP_SYNC0;
    out[1] = LP_SYNC1;
    out[2] = (uint8_t)(LP_HEADER_BYTES + n);
    out[3] = veh;
    out[4] = type;
    out[5] = seq;
    for (i = 0; i < n; i++)
        out[6 + i] = payload[i];
    crc = lp_crc16(out + 2, (uint8_t)(1u + LP_HEADER_BYTES + n), LP_CRC_INIT);
    out[6 + n] = (uint8_t)crc;
    out[7 + n] = (uint8_t)(crc >> 8);
    return (uint8_t)(LP_FRAME_OVERHEAD + LP_HEADER_BYTES + n);
}

uint8_t lp_encode_cmd(uint8_t veh, uint8_t seq, const lp_cmd_t *c, uint8_t out[LP_CMD_FRAME_BYTES])
{
    uint8_t pl[LP_CMD_PAYLOAD_LEN];
    lp_pack_cmd(c, pl);
    return lp_encode_frame(veh, LP_TYPE_CMD, seq, pl, LP_CMD_PAYLOAD_LEN, out);
}

uint8_t lp_encode_telem(uint8_t veh, uint8_t seq, const lp_telem_t *t, uint8_t out[LP_TELEM_FRAME_BYTES])
{
    uint8_t pl[LP_TELEM_PAYLOAD_LEN];
    lp_pack_telem(t, pl);
    return lp_encode_frame(veh, LP_TYPE_TELEM, seq, pl, LP_TELEM_PAYLOAD_LEN, out);
}

uint8_t lp_encode_telem2(uint8_t veh, uint8_t seq, const lp_telem_t *t, uint8_t out[LP_TELEM2_FRAME_BYTES])
{
    uint8_t pl[LP_TELEM2_PAYLOAD_LEN];
    lp_pack_telem2(t, pl);
    return lp_encode_frame(veh, LP_TYPE_TELEM2, seq, pl, LP_TELEM2_PAYLOAD_LEN, out);
}

uint8_t lp_encode_telem3(uint8_t veh, uint8_t seq, const lp_telem_t *t, uint8_t out[LP_TELEM3_FRAME_BYTES])
{
    uint8_t pl[LP_TELEM3_PAYLOAD_LEN];
    lp_pack_telem3(t, pl);
    return lp_encode_frame(veh, LP_TYPE_TELEM3, seq, pl, LP_TELEM3_PAYLOAD_LEN, out);
}

/* ---- streaming decoder ---- */
enum { ST_HUNT0 = 0, ST_HUNT1, ST_LEN, ST_BODY };

#if LP_DEC_QUEUE > 128u || LP_DEC_QUEUE < LP_FRAME_MAX_BYTES + 1u
#error "LP_DEC_QUEUE must be in LP_FRAME_MAX_BYTES + 1 .. 128"
#endif

static uint8_t q_index(uint8_t i)                      /* i < 2 LP_DEC_QUEUE -> i mod LP_DEC_QUEUE without a division */
{
    return i >= LP_DEC_QUEUE ? (uint8_t)(i - LP_DEC_QUEUE) : i;
}

void lp_decoder_reset(lp_decoder_t *d)
{
    d->state = ST_HUNT0;
    d->n = 0;
    d->need = 0;
    d->q_head = 0;
    d->q_len = 0;
}

void lp_decoder_init(lp_decoder_t *d, int16_t vehicle_id, uint8_t accept_mask)
{
    d->vehicle_id = vehicle_id;
    d->accept_mask = accept_mask;
    d->stats.frames = d->stats.bad_len = d->stats.bad_crc = d->stats.wrong_vehicle = 0;
    d->stats.wrong_type = d->stats.bad_payload_len = d->stats.skipped_bytes = d->stats.overflow = 0;
    lp_decoder_reset(d);
}

static bool q_push_front(lp_decoder_t *d, uint8_t b)
{
    if (d->q_len >= LP_DEC_QUEUE)
        return false;
    d->q_head = d->q_head == 0u ? (uint8_t)(LP_DEC_QUEUE - 1u) : (uint8_t)(d->q_head - 1u);
    d->q[d->q_head] = b;
    d->q_len++;
    return true;
}

/* the candidate failed: discard its first byte (the 0xA5), parse the rest again from HUNT0 */
static void fail(lp_decoder_t *d)
{
    uint8_t i;
    d->stats.skipped_bytes++;
    for (i = d->n; i > 1u; i--) {
        if (!q_push_front(d, d->buf[i - 1u]))
            d->stats.overflow++;                       /* cannot happen: queue + candidate <= LP_FRAME_MAX_BYTES */
    }
    d->n = 0;
    d->state = ST_HUNT0;
}

/* if / else, not switch: avr-gcc turns a dense switch on 0x10..0x22 that returns constants into a CSWTCH lookup table
 * in .data (18 B of RAM each, measured by the v0.7 integration on the COM); a chain of compares costs no RAM.
 * v0.9 RP1: TELEM4 / TELEM5 (0x23 / 0x24) in the builds with the v0.9 sizes (LP_RP1; the ballistic images keep the
 * v0.8.1 chain byte for byte - their decoders accept CMD only) */
static uint8_t type_payload_len(uint8_t type)
{
    if (type == LP_TYPE_CMD)
        return LP_CMD_PAYLOAD_LEN;
    if (type == LP_TYPE_TELEM)
        return LP_TELEM_PAYLOAD_LEN;
    if (type == LP_TYPE_TELEM2)
        return LP_TELEM2_PAYLOAD_LEN;
    if (type == LP_TYPE_TELEM3)
        return LP_TELEM3_PAYLOAD_LEN;
#if LP_RP1
    if (type == LP_TYPE_TELEM4)
        return LP_TELEM4_PAYLOAD_LEN;
    if (type == LP_TYPE_TELEM5)
        return LP_TELEM5_PAYLOAD_LEN;
    if (type == LP_TYPE_TELEM6)                        /* 2026-11-10: the logic-battery tail */
        return LP_TELEM6_PAYLOAD_LEN;
    if (type == LP_TYPE_TELEM7)
        return LP_TELEM7_PAYLOAD_LEN;
#endif
    return 0xFFu;
}

static uint8_t type_accept_bit(uint8_t type)
{
    if (type == LP_TYPE_CMD)
        return LP_ACCEPT_CMD;
    if (type == LP_TYPE_TELEM)
        return LP_ACCEPT_TELEM;
    if (type == LP_TYPE_TELEM2)
        return LP_ACCEPT_TELEM2;
    if (type == LP_TYPE_TELEM3)
        return LP_ACCEPT_TELEM3;
#if LP_RP1
    if (type == LP_TYPE_TELEM4)
        return LP_ACCEPT_TELEM4;
    if (type == LP_TYPE_TELEM5)
        return LP_ACCEPT_TELEM5;
    if (type == LP_TYPE_TELEM6)
        return LP_ACCEPT_TELEM6;
    if (type == LP_TYPE_TELEM7)
        return LP_ACCEPT_TELEM7;
#endif
    return 0u;
}

/* one byte through the state machine; returns true and fills *f when a valid frame completed */
static bool step(lp_decoder_t *d, uint8_t b, lp_frame_t *f)
{
    switch (d->state) {
    case ST_HUNT0:
        if (b == LP_SYNC0) {
            d->buf[0] = b;
            d->n = 1;
            d->state = ST_HUNT1;
        } else {
            d->stats.skipped_bytes++;
        }
        return false;
    case ST_HUNT1:
        if (b == LP_SYNC1) {
            d->buf[d->n++] = b;
            d->state = ST_LEN;
        } else if (b == LP_SYNC0) {                    /* A5 A5 5A ...: the second A5 starts the candidate */
            d->stats.skipped_bytes++;
            d->buf[0] = b;
            d->n = 1;
        } else {
            d->stats.skipped_bytes += 2u;
            d->n = 0;
            d->state = ST_HUNT0;
        }
        return false;
    case ST_LEN:
        d->buf[d->n++] = b;
        if (b >= LP_LEN_MIN && b <= LP_LEN_MAX) {
            d->need = (uint8_t)(b + 2u);               /* VEH..PAYLOAD + CRC */
            d->state = ST_BODY;
        } else {
            d->stats.bad_len++;
            fail(d);
        }
        return false;
    default: {                                         /* ST_BODY */
        uint8_t ln, veh, typ, i, plen;
        uint16_t crc_rx;
        d->buf[d->n++] = b;
        if (--d->need > 0u)
            return false;
        ln = d->buf[2];
        crc_rx = (uint16_t)(d->buf[3u + ln] | ((uint16_t)d->buf[4u + ln] << 8));
        if (lp_crc16(d->buf + 2, (uint8_t)(ln + 1u), LP_CRC_INIT) != crc_rx) {
            d->stats.bad_crc++;
            fail(d);
            return false;
        }
        veh = d->buf[3];
        typ = d->buf[4];
        plen = (uint8_t)(ln - LP_HEADER_BYTES);
        d->n = 0;
        d->state = ST_HUNT0;
        if (d->vehicle_id >= 0 && veh != (uint8_t)d->vehicle_id) {
            d->stats.wrong_vehicle++;
            return false;
        }
        if ((type_accept_bit(typ) & d->accept_mask) == 0u) {
            d->stats.wrong_type++;
            return false;
        }
        if (plen != type_payload_len(typ)) {
            d->stats.bad_payload_len++;
            return false;
        }
        d->stats.frames++;
        f->veh = veh;
        f->type = typ;
        f->seq = d->buf[5];
        f->len = plen;
        for (i = 0; i < plen; i++)
            f->payload[i] = d->buf[6u + i];
        return true;
    }
    }
}

uint8_t lp_decoder_feed(lp_decoder_t *d, uint8_t byte, lp_frame_t *out, uint8_t max_out)
{
    uint8_t n_out = 0;
    lp_frame_t spill;
    if (d->q_len >= LP_DEC_QUEUE) {                    /* cannot happen (the queue is drained below) */
        d->stats.overflow++;
        return 0;
    }
    d->q[q_index((uint8_t)(d->q_head + d->q_len))] = byte;
    d->q_len++;
    while (d->q_len > 0u) {
        uint8_t b = d->q[d->q_head];
        d->q_head = q_index((uint8_t)(d->q_head + 1u));
        d->q_len--;
        if (step(d, b, n_out < max_out ? &out[n_out] : &spill)) {
            if (n_out < max_out)
                n_out++;
            else
                d->stats.overflow++;
        }
    }
    return n_out;
}
