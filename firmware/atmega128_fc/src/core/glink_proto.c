/* glink_proto.c - see glink_proto.h (mirrors sim/glink_proto.py: frames, GlinkDecoder, GlinkNav). */
#include "glink_proto.h"

#include "ilink_proto.h"      /* il_crc16_update / il_crc16 (the one CRC of every link) */

#if !defined(__BYTE_ORDER__) || __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "glink_proto copies IEEE single bit patterns through a union (float and integer byte order must agree)"
#endif
typedef char gl_sizes_ok[(GL_STATE_FRAME_BYTES == 60u && GL_RESULT_FRAME_BYTES == 26u && GL_RESULT_LEN == NL_GNS_RESULT_LEN)
                         ? 1 : -1];

typedef union {
    float f;
    uint32_t u;
} gl_f32_t;

enum { ST_SYNC1, ST_SYNC2, ST_TYPE, ST_SEQ, ST_PAY, ST_CK1, ST_CK2 };

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
    gl_f32_t v;
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
    gl_f32_t v;
    v.u = get32(p);
    return v.f;
}

static uint8_t payload_len(uint8_t type)
{
    return type == GL_TYPE_STATE ? GL_STATE_LEN : (type == GL_TYPE_RESULT ? GL_RESULT_LEN : 0u);
}

void gl_pack_state(const gl_state_t *s, uint8_t out[GL_STATE_LEN])
{
    uint8_t *p = put32(out, s->tick), i;
    for (i = 0; i < 3u; i++)
        p = putf(p, s->fsum[i]);
    *p++ = s->n;
    for (i = 0; i < 4u; i++)
        p = putf(p, s->q[i]);
    for (i = 0; i < 3u; i++)
        p = putf(p, s->w[i]);
    p = putf(p, s->h);
    *p++ = s->nflags;
    *p++ = s->fcu_state;
    *p++ = s->pflags;
    (void)put16(p, s->kt_q);
}

void gl_unpack_state(const uint8_t in[GL_STATE_LEN], gl_state_t *s)
{
    const uint8_t *p = in + 4;
    uint8_t i;
    s->tick = get32(in);
    for (i = 0; i < 3u; i++, p += 4)
        s->fsum[i] = getf(p);
    s->n = *p++;
    for (i = 0; i < 4u; i++, p += 4)
        s->q[i] = getf(p);
    for (i = 0; i < 3u; i++, p += 4)
        s->w[i] = getf(p);
    s->h = getf(p);
    p += 4;
    s->nflags = p[0];
    s->fcu_state = p[1];
    s->pflags = p[2];
    s->kt_q = get16(p + 3);
}

void gl_pack_result(const gl_result_t *r, uint8_t out[GL_RESULT_LEN])
{
    uint8_t *p = put16(put16(out, (uint16_t)r->tilt[0]), (uint16_t)r->tilt[1]);
    *p++ = r->flags;
    *p++ = r->num_sv;
    *p++ = r->hacc_dm;
    *p++ = r->pc;
    p = put16(put16(p, (uint16_t)r->pos[0]), (uint16_t)r->pos[1]);
    p = put16(put16(p, (uint16_t)r->vel[0]), (uint16_t)r->vel[1]);
    (void)put16(put16(p, (uint16_t)r->trim[0]), (uint16_t)r->trim[1]);
}

void gl_unpack_result(const uint8_t in[GL_RESULT_LEN], gl_result_t *r)
{
    r->tilt[0] = (int16_t)get16(in);
    r->tilt[1] = (int16_t)get16(in + 2);
    r->flags = in[4];
    r->num_sv = in[5];
    r->hacc_dm = in[6];
    r->pc = in[7];
    r->pos[0] = (int16_t)get16(in + 8);
    r->pos[1] = (int16_t)get16(in + 10);
    r->vel[0] = (int16_t)get16(in + 12);
    r->vel[1] = (int16_t)get16(in + 14);
    r->trim[0] = (int16_t)get16(in + 16);
    r->trim[1] = (int16_t)get16(in + 18);
}

uint8_t gl_encode(uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t *out)
{
    const uint8_t len = payload_len(type);
    uint16_t crc;
    uint8_t i;
    out[0] = GL_SYNC0;
    out[1] = GL_SYNC1;
    out[2] = type;
    out[3] = seq;
    if (payload != out + GL_HEADER_BYTES)
        for (i = 0; i < len; i++)
            out[GL_HEADER_BYTES + i] = payload[i];
    crc = il_crc16(out + 2, (uint8_t)(len + 2u), LP_CRC_INIT);
    out[GL_HEADER_BYTES + len] = (uint8_t)crc;
    out[GL_HEADER_BYTES + len + 1u] = (uint8_t)(crc >> 8);
    return (uint8_t)(GL_HEADER_BYTES + len + 2u);
}

uint8_t gl_encode_state(uint8_t seq, const gl_state_t *s, uint8_t out[GL_STATE_FRAME_BYTES])
{
    gl_pack_state(s, out + GL_HEADER_BYTES);
    return gl_encode(GL_TYPE_STATE, seq, out + GL_HEADER_BYTES, out);
}

uint8_t gl_encode_result(uint8_t seq, const gl_result_t *r, uint8_t out[GL_RESULT_FRAME_BYTES])
{
    gl_pack_result(r, out + GL_HEADER_BYTES);
    return gl_encode(GL_TYPE_RESULT, seq, out + GL_HEADER_BYTES, out);
}

uint8_t gl_reject_reason(const uint8_t *buf, uint16_t n)
{
    uint8_t len;
    uint16_t crc, end;
    if (n < GL_HEADER_BYTES + 2u)
        return GL_REJ_LENGTH;
    if (buf[0] != GL_SYNC0 || buf[1] != GL_SYNC1)
        return GL_REJ_SYNC;
    len = payload_len(buf[2]);
    if (len == 0u)
        return GL_REJ_TYPE;
    if (n != (uint16_t)(GL_HEADER_BYTES + len + 2u))
        return GL_REJ_LENGTH;
    end = (uint16_t)(GL_HEADER_BYTES + len);
    crc = il_crc16(buf + 2, (uint8_t)(len + 2u), LP_CRC_INIT);
    return (buf[end] | ((uint16_t)buf[end + 1u] << 8)) == crc ? GL_REJ_NONE : GL_REJ_CRC;
}

uint8_t gl_decode_frame(const uint8_t *buf, uint16_t n, uint8_t *type, uint8_t *seq)
{
    if (gl_reject_reason(buf, n) != GL_REJ_NONE)
        return 0;
    *type = buf[2];
    *seq = buf[3];
    return 1;
}

void gl_decoder_init(gl_decoder_t *d, uint8_t accept)
{
    d->accept = accept;
    d->st = ST_SYNC1;
    d->typ = d->seq = d->need = d->idx = d->ck_lo = 0;
    d->crc = LP_CRC_INIT;
    d->stats.frames = d->stats.bad_crc = d->stats.bad_type = d->stats.wrong_dir = d->stats.skipped = 0;
}

uint8_t gl_decoder_byte(gl_decoder_t *d, uint8_t b)
{
    switch (d->st) {
    case ST_SYNC1:
        if (b == GL_SYNC0)
            d->st = ST_SYNC2;
        else
            d->stats.skipped++;
        break;
    case ST_SYNC2:
        if (b == GL_SYNC1) {
            d->st = ST_TYPE;
        } else if (b != GL_SYNC0) {
            d->stats.skipped++;
            d->st = ST_SYNC1;
        }
        break;
    case ST_TYPE:
        d->need = payload_len(b);
        if (d->need != 0u) {
            d->typ = b;
            d->crc = il_crc16_update(LP_CRC_INIT, b);
            d->st = ST_SEQ;
        } else {
            d->stats.bad_type++;
            d->st = ST_SYNC1;
        }
        break;
    case ST_SEQ:
        d->seq = b;
        d->crc = il_crc16_update(d->crc, b);
        d->idx = 0;
        d->st = ST_PAY;
        break;
    case ST_PAY:
        d->buf[d->idx++] = b;
        d->crc = il_crc16_update(d->crc, b);
        if (d->idx == d->need)
            d->st = ST_CK1;
        break;
    case ST_CK1:
        d->ck_lo = b;
        d->st = ST_CK2;
        break;
    default:
        d->st = ST_SYNC1;
        if ((d->ck_lo | ((uint16_t)b << 8)) != d->crc) {
            d->stats.bad_crc++;
            return 0;
        }
        if (!(d->accept & (d->typ == GL_TYPE_STATE ? GL_ACCEPT_STATE : GL_ACCEPT_RESULT))) {
            d->stats.wrong_dir++;
            return 0;
        }
        d->stats.frames++;
        return d->typ;
    }
    return 0;
}

void gl_nav_init(gl_nav_t *g, uint8_t step_ticks, uint8_t latency_ticks)
{
    g->n = step_ticks;
    nl_gns_tx_init(&g->tx, step_ticks, latency_ticks);
    gl_decoder_init(&g->rx, GL_ACCEPT_RESULT);
    g->fsum[0] = g->fsum[1] = g->fsum[2] = 0.0f;
    g->nsum = 0;
    g->sat_or = g->fcu_seen = false;
    g->started = false;
    g->k = g->kq = 0;
    g->j = 0;
    g->states = 0;
}

uint8_t gl_nav_rx(gl_nav_t *g, const uint8_t *data, uint16_t n, uint32_t k)
{
    uint16_t i;
    uint8_t taken = 0;
    for (i = 0; i < n; i++) {
        if (gl_decoder_byte(&g->rx, data[i]) == GL_TYPE_RESULT) {
            nl_gns_tx_result(&g->tx, g->rx.seq, g->rx.buf, k);
            taken++;
        }
    }
    return taken;
}

void gl_nav_slot(gl_nav_t *g, uint32_t k, uint8_t out[NL_SLOT_LEN])
{
    nl_gns_tx_slot(&g->tx, k, out);
}

bool gl_nav_tick(gl_nav_t *g, uint32_t k, const float accel_b[3], const float q_corr[4], const float rate_b[3], float h,
                 bool on_pad, const nl_fcu4_t *fcu, bool fcu_valid, bool imu_ok, bool est_valid,
                 uint8_t out[GL_STATE_FRAME_BYTES])
{
    gl_state_t st;
    uint8_t i;
    if (g->started && k == g->k + 1u) {                /* k mod n and k / n counted while k advances by one */
        if (++g->j >= g->n) {
            g->j = 0;
            g->kq++;
        }
    } else {
        g->j = (uint8_t)(k % g->n);
        g->kq = k / g->n;
        g->started = true;
    }
    g->k = k;
    for (i = 0; i < 3u; i++)
        g->fsum[i] += accel_b[i];
    if (g->nsum < 0xFFu)
        g->nsum++;
    if (fcu_valid && fcu != 0) {
        g->fcu_seen = true;
        g->sat_or = g->sat_or || (fcu->pflags & NL_PF_SAT_HI) != 0u;
    }
    if (g->j != (uint8_t)(g->n - 1u))
        return false;
    st.tick = k;
    for (i = 0; i < 3u; i++)
        st.fsum[i] = g->fsum[i];
    st.n = g->nsum;
    for (i = 0; i < 4u; i++)
        st.q[i] = q_corr[i];
    for (i = 0; i < 3u; i++)
        st.w[i] = rate_b[i];
    st.h = h;
    st.nflags = (uint8_t)((on_pad ? GL_NF_PAD : 0u) | (g->fcu_seen ? GL_NF_FCU_OK : 0u) | (imu_ok ? GL_NF_IMU_OK : 0u) |
                          (est_valid ? GL_NF_EST_VALID : 0u));
    st.fcu_state = fcu != 0 ? fcu->state : 0u;
    st.pflags = (uint8_t)((fcu != 0 ? (fcu->pflags & (uint8_t)~NL_PF_SAT_HI) : 0u) | (g->sat_or ? NL_PF_SAT_HI : 0u));
    st.kt_q = fcu != 0 ? fcu->kt_q : (uint16_t)NL_KT_SCALE;
    g->fsum[0] = g->fsum[1] = g->fsum[2] = 0.0f;
    g->nsum = 0;
    g->sat_or = false;
    g->fcu_seen = false;
    g->states++;
    (void)gl_encode_state((uint8_t)g->kq, &st, out);
    return true;
}
