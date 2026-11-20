/* nlink_proto.c - see nlink_proto.h (mirrors sim/nlink_proto.py, the normative reference).
 *
 * Frames are built and checked in place in the 72-byte exchange buffer: explicit little-endian fields,
 * floats copied as their 4-byte IEEE pattern (NaN payloads, signalling NaNs, infinities, -0 and
 * denormals pass bit for bit), CRC-16/CCITT-FALSE with ilink_proto.h's il_crc16_update (the byte-wise
 * branch-free form, 30.5 cycles per byte on the AVR; == lp_crc16).
 *
 * Decoding: valid iff bytes 0, 1 = 0xD5 0x5D, byte 2 = the expected TYPE and the two CRC bytes equal the
 * CRC over TYPE, SEQ and the payload.  Nothing else is looked at (the FCU frame's reserved byte is
 * CRC-covered but not part of nl_fcu_t; bytes 10..71 of the FCU's half of the exchange are ignored).  The
 * struct is written only when the frame is valid.
 *
 * Invariant: nl_encode_* / nl_decode_* produce and accept exactly the bytes sim/nlink_proto.py does for the
 * same fields (host test vectors sim/out/nlink_vectors.json + tools/crosscheck_controller.py --only nlink).
 */
#include "nlink_proto.h"

#include "ilink_proto.h"      /* il_crc16_update / il_crc16 (the one CRC of every link) */
#include "../fc_mode.h"

#if !defined(__BYTE_ORDER__) || __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "nlink_proto copies IEEE single bit patterns through a union (float and integer byte order must agree: AVR, x86, ARM)"
#endif
typedef char nl_float_is_binary32[(sizeof(float) == 4u && sizeof(uint32_t) == 4u) ? 1 : -1];
typedef char nl_xfer_is_72[(NL_XFER_BYTES == 72u && NL_FCU_FRAME_BYTES == 10u) ? 1 : -1];

typedef union {
    float f;
    uint32_t u;
} nl_f32_t;

static uint8_t *put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    return p + 2;
}

static uint8_t *putf(uint8_t *p, float f)
{
    nl_f32_t v;
    v.f = f;
    return put16(put16(p, (uint16_t)v.u), (uint16_t)(v.u >> 16));
}

static uint8_t *putfs(uint8_t *p, const float *f, uint8_t n)
{
    uint8_t i;
    for (i = 0; i < n; i++)
        p = putf(p, f[i]);
    return p;
}

static uint16_t get16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static const uint8_t *getfs(const uint8_t *p, float *f, uint8_t n)
{
    uint8_t i;
    for (i = 0; i < n; i++, p += 4) {
        nl_f32_t v;
        v.u = (uint32_t)get16(p) | ((uint32_t)get16(p + 2) << 16);
        f[i] = v.f;
    }
    return p;
}

#define NL_CRC16 il_crc16

/* sync, TYPE, SEQ at out[0..3]; payload already at out[4..4+len); CRC after it; returns the frame length */
static uint8_t seal(uint8_t *out, uint8_t type, uint8_t seq, uint8_t len)
{
    uint16_t crc;
    out[0] = NL_SYNC0;
    out[1] = NL_SYNC1;
    out[2] = type;
    out[3] = seq;
    crc = NL_CRC16(out + 2, (uint8_t)(len + 2u), LP_CRC_INIT);
    out[NL_HEADER_BYTES + len] = (uint8_t)crc;
    out[NL_HEADER_BYTES + len + 1u] = (uint8_t)(crc >> 8);
    return (uint8_t)(NL_HEADER_BYTES + len + 2u);
}

/* sync, TYPE and CRC of the frame of payload length len at in[0..] */
static uint8_t check(const uint8_t *in, uint8_t type, uint8_t len)
{
    uint16_t crc;
    if (in[0] != NL_SYNC0 || in[1] != NL_SYNC1 || in[2] != type)
        return 0;
    crc = NL_CRC16(in + 2, (uint8_t)(len + 2u), LP_CRC_INIT);
    return (uint8_t)(in[NL_HEADER_BYTES + len] == (uint8_t)crc && in[NL_HEADER_BYTES + len + 1u] == (uint8_t)(crc >> 8));
}

uint8_t nl_encode_nav(const nl_nav_t *n, uint8_t out[NL_XFER_BYTES])
{
    uint8_t *p = out + NL_HEADER_BYTES;
    p = putfs(p, n->q, 4);
    p = putfs(p, n->rate, 3);
    p = putfs(p, n->accel, 3);
    p = putf(p, n->h_m);
    p = putf(p, n->vz_mps);
    p = putf(p, n->h_baro_rel_m);
    p = putf(p, n->tilt_deg);
    p = put16(p, n->imu_fail);
    p = put16(p, n->baro_fails);
    p = put16(p, n->twi_errors);
    p = put16(p, n->loop_us_max);
    *p++ = n->flags;
    *p = n->reset_cause;
    return seal(out, NL_TYPE_NAV, n->seq, NL_NAV_LEN);
}

uint8_t nl_encode_fcu(const nl_fcu_t *f, uint8_t out[NL_XFER_BYTES])
{
    uint8_t i, n;
    out[4] = f->state;
    out[5] = f->fflags;
    out[6] = f->params_tag;
    out[7] = f->thr_q;                                 /* v0.7.1 (v0.6.3: reserved 0; the ballistic FCU writes 0) */
    n = seal(out, NL_TYPE_FCU, f->seq, NL_FCU_LEN);
    for (i = NL_FCU_FRAME_BYTES; i < NL_XFER_BYTES; i++)
        out[i] = 0;
    return n;
}

uint8_t nl_decode_nav(const uint8_t in[NL_XFER_BYTES], nl_nav_t *n)
{
    const uint8_t *p = in + NL_HEADER_BYTES;
    if (!check(in, NL_TYPE_NAV, NL_NAV_LEN))
        return 0;
    n->seq = in[3];
    p = getfs(p, n->q, 4);
    p = getfs(p, n->rate, 3);
    p = getfs(p, n->accel, 3);
    p = getfs(p, &n->h_m, 1);
    p = getfs(p, &n->vz_mps, 1);
    p = getfs(p, &n->h_baro_rel_m, 1);
    p = getfs(p, &n->tilt_deg, 1);
    n->imu_fail = get16(p);
    n->baro_fails = get16(p + 2);
    n->twi_errors = get16(p + 4);
    n->loop_us_max = get16(p + 6);
    n->flags = p[8];
    n->reset_cause = p[9];
    return 1;
}

uint8_t nl_decode_fcu(const uint8_t in[NL_XFER_BYTES], nl_fcu_t *f)
{
    if (!check(in, NL_TYPE_FCU, NL_FCU_LEN))
        return 0;
    f->seq = in[3];
    f->state = in[4];
    f->fflags = in[5];
    f->params_tag = in[6];
    f->thr_q = in[7];
    return 1;
}
