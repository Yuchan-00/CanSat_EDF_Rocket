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
#include "qfix.h"             /* P1: the tilt series in fixed point */
#include "../fc_mode.h"       /* P1: FC_VTOL (the table CRC and the tilt series are VTOL-only) */

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

#if FC_VTOL
/* P1 (fw09/p1): CRC-16/CCITT-FALSE by table, crc' = (crc << 8) ^ T[(crc >> 8) ^ b] - the same value as il_crc16 (the host
 * test checks every nlink vector; fw09/p1 measures ~20 cycles a byte against il_crc16_update's 30.5).  The table is in
 * flash (512 B, read with LPM); VTOL builds only, so the ballistic images keep il_crc16 byte for byte. */
#if defined(__AVR__)
#define NL_TAB_ATTR __attribute__((__progmem__))
#define NL_TAB(i) fc_lpm16(&s_nl_crc_tab[(i)])
#else
#define NL_TAB_ATTR
#define NL_TAB(i) (s_nl_crc_tab[(i)])
#endif
static const uint16_t s_nl_crc_tab[256] NL_TAB_ATTR = {
    0x0000u, 0x1021u, 0x2042u, 0x3063u, 0x4084u, 0x50A5u, 0x60C6u, 0x70E7u,
    0x8108u, 0x9129u, 0xA14Au, 0xB16Bu, 0xC18Cu, 0xD1ADu, 0xE1CEu, 0xF1EFu,
    0x1231u, 0x0210u, 0x3273u, 0x2252u, 0x52B5u, 0x4294u, 0x72F7u, 0x62D6u,
    0x9339u, 0x8318u, 0xB37Bu, 0xA35Au, 0xD3BDu, 0xC39Cu, 0xF3FFu, 0xE3DEu,
    0x2462u, 0x3443u, 0x0420u, 0x1401u, 0x64E6u, 0x74C7u, 0x44A4u, 0x5485u,
    0xA56Au, 0xB54Bu, 0x8528u, 0x9509u, 0xE5EEu, 0xF5CFu, 0xC5ACu, 0xD58Du,
    0x3653u, 0x2672u, 0x1611u, 0x0630u, 0x76D7u, 0x66F6u, 0x5695u, 0x46B4u,
    0xB75Bu, 0xA77Au, 0x9719u, 0x8738u, 0xF7DFu, 0xE7FEu, 0xD79Du, 0xC7BCu,
    0x48C4u, 0x58E5u, 0x6886u, 0x78A7u, 0x0840u, 0x1861u, 0x2802u, 0x3823u,
    0xC9CCu, 0xD9EDu, 0xE98Eu, 0xF9AFu, 0x8948u, 0x9969u, 0xA90Au, 0xB92Bu,
    0x5AF5u, 0x4AD4u, 0x7AB7u, 0x6A96u, 0x1A71u, 0x0A50u, 0x3A33u, 0x2A12u,
    0xDBFDu, 0xCBDCu, 0xFBBFu, 0xEB9Eu, 0x9B79u, 0x8B58u, 0xBB3Bu, 0xAB1Au,
    0x6CA6u, 0x7C87u, 0x4CE4u, 0x5CC5u, 0x2C22u, 0x3C03u, 0x0C60u, 0x1C41u,
    0xEDAEu, 0xFD8Fu, 0xCDECu, 0xDDCDu, 0xAD2Au, 0xBD0Bu, 0x8D68u, 0x9D49u,
    0x7E97u, 0x6EB6u, 0x5ED5u, 0x4EF4u, 0x3E13u, 0x2E32u, 0x1E51u, 0x0E70u,
    0xFF9Fu, 0xEFBEu, 0xDFDDu, 0xCFFCu, 0xBF1Bu, 0xAF3Au, 0x9F59u, 0x8F78u,
    0x9188u, 0x81A9u, 0xB1CAu, 0xA1EBu, 0xD10Cu, 0xC12Du, 0xF14Eu, 0xE16Fu,
    0x1080u, 0x00A1u, 0x30C2u, 0x20E3u, 0x5004u, 0x4025u, 0x7046u, 0x6067u,
    0x83B9u, 0x9398u, 0xA3FBu, 0xB3DAu, 0xC33Du, 0xD31Cu, 0xE37Fu, 0xF35Eu,
    0x02B1u, 0x1290u, 0x22F3u, 0x32D2u, 0x4235u, 0x5214u, 0x6277u, 0x7256u,
    0xB5EAu, 0xA5CBu, 0x95A8u, 0x8589u, 0xF56Eu, 0xE54Fu, 0xD52Cu, 0xC50Du,
    0x34E2u, 0x24C3u, 0x14A0u, 0x0481u, 0x7466u, 0x6447u, 0x5424u, 0x4405u,
    0xA7DBu, 0xB7FAu, 0x8799u, 0x97B8u, 0xE75Fu, 0xF77Eu, 0xC71Du, 0xD73Cu,
    0x26D3u, 0x36F2u, 0x0691u, 0x16B0u, 0x6657u, 0x7676u, 0x4615u, 0x5634u,
    0xD94Cu, 0xC96Du, 0xF90Eu, 0xE92Fu, 0x99C8u, 0x89E9u, 0xB98Au, 0xA9ABu,
    0x5844u, 0x4865u, 0x7806u, 0x6827u, 0x18C0u, 0x08E1u, 0x3882u, 0x28A3u,
    0xCB7Du, 0xDB5Cu, 0xEB3Fu, 0xFB1Eu, 0x8BF9u, 0x9BD8u, 0xABBBu, 0xBB9Au,
    0x4A75u, 0x5A54u, 0x6A37u, 0x7A16u, 0x0AF1u, 0x1AD0u, 0x2AB3u, 0x3A92u,
    0xFD2Eu, 0xED0Fu, 0xDD6Cu, 0xCD4Du, 0xBDAAu, 0xAD8Bu, 0x9DE8u, 0x8DC9u,
    0x7C26u, 0x6C07u, 0x5C64u, 0x4C45u, 0x3CA2u, 0x2C83u, 0x1CE0u, 0x0CC1u,
    0xEF1Fu, 0xFF3Eu, 0xCF5Du, 0xDF7Cu, 0xAF9Bu, 0xBFBAu, 0x8FD9u, 0x9FF8u,
    0x6E17u, 0x7E36u, 0x4E55u, 0x5E74u, 0x2E93u, 0x3EB2u, 0x0ED1u, 0x1EF0u,
};

static inline __attribute__((always_inline)) uint16_t nl_crc16(const uint8_t *d, uint8_t n, uint16_t crc)
{
    while (n--)
        crc = (uint16_t)((uint16_t)(crc << 8) ^ NL_TAB((uint8_t)((uint8_t)(crc >> 8) ^ *d++)));
    return crc;
}
#define NL_CRC16 nl_crc16
#else
#define NL_CRC16 il_crc16
#endif

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

/* ---- v0.8 nlink v3 ---------------------------------------------------------------------------- */
/* the v3 / v4 payload at out[4..70) (the frame is sealed by the caller with its TYPE) */
static void put_nav3(const nl_nav3_t *n, uint8_t *out)
{
    uint8_t *p = out + NL_HEADER_BYTES, i;
    p = putfs(p, n->q, 4);
    p = putfs(p, n->rate, 3);
    p = putf(p, n->accel_z);
    p = putf(p, n->h_m);
    p = putf(p, n->vz_mps);
    p = putf(p, n->cos_tilt);
    p = putf(p, n->tilt_deg);
    p = put16(p, (uint16_t)n->accel_x_q);
    p = put16(p, (uint16_t)n->accel_y_q);
    p = put16(p, (uint16_t)n->h_baro_rel_cm);
    *p++ = n->imu_fail;
    *p++ = n->baro_fails;
    *p++ = n->twi_errors;
    p = put16(p, n->loop_us_max);
    *p++ = n->flags;
    *p++ = n->reset_cause;
    for (i = 0; i < NL_SLOT_LEN; i++)
        *p++ = n->slot[i];
}

uint8_t nl_encode_nav3(const nl_nav3_t *n, uint8_t out[NL_XFER_BYTES])
{
    put_nav3(n, out);
    return seal(out, NL_TYPE_NAV3, n->seq, NL_NAV3_LEN);
}

/* the v3 / v4 payload of a checked buffer */
static void get_nav3(const uint8_t *in, nl_nav3_t *n)
{
    const uint8_t *p = in + NL_HEADER_BYTES;
    uint8_t i;
    n->seq = in[3];
    p = getfs(p, n->q, 4);
    p = getfs(p, n->rate, 3);
    p = getfs(p, &n->accel_z, 1);
    p = getfs(p, &n->h_m, 1);
    p = getfs(p, &n->vz_mps, 1);
    p = getfs(p, &n->cos_tilt, 1);
    p = getfs(p, &n->tilt_deg, 1);
    n->accel_x_q = (int16_t)get16(p);
    n->accel_y_q = (int16_t)get16(p + 2);
    n->h_baro_rel_cm = (int16_t)get16(p + 4);
    n->imu_fail = p[6];
    n->baro_fails = p[7];
    n->twi_errors = p[8];
    n->loop_us_max = get16(p + 9);
    n->flags = p[11];
    n->reset_cause = p[12];
    for (i = 0; i < NL_SLOT_LEN; i++)
        n->slot[i] = p[13 + i];
}

uint8_t nl_decode_nav3(const uint8_t in[NL_XFER_BYTES], nl_nav3_t *n)
{
    if (!check(in, NL_TYPE_NAV3, NL_NAV3_LEN))
        return 0;
    get_nav3(in, n);
    return 1;
}

#if FC_VTOL
/* P1: acos(1 - 2 s) = 2 asin(sqrt(s)), asin(u) = u (1 + s/6 + 3 s^2/40 + 5 s^3/112 + 35 s^4/1152 + 63 s^5/2816 + ...)
 * with s = u^2 (next term 231 s^6 / 13312 < 1.6e-9 for s < 0.067); the polynomial in Q30 on s in Q32 */
#define NL_TS_MAX 0.067f
#define NL_TS_C1 178956971L                   /* 1/6     x 2^30 */
#define NL_TS_C2 80530637L                    /* 3/40    x 2^30 */
#define NL_TS_C3 47934903L                    /* 5/112   x 2^30 */
#define NL_TS_C4 32622980L                    /* 35/1152 x 2^30 */
#define NL_TS_C5 24021797L                    /* 63/2816 x 2^30 */
#endif

void nl_nav3_tilt(const float q[4], float *cos_tilt, float *tilt_deg)
{
    const float x = q[1], y = q[2];
    const float x2 = x * x, y2 = y * y;
    const float s = x2 + y2;
    const float c = 1.0f - 2.0f * s;
    *cos_tilt = c;
#if FC_VTOL
    if (s < NL_TS_MAX) {                                   /* also s >= 0 (a sum of squares) */
        const int32_t sq = fx_from_f(s, 32);
        int32_t pp = NL_TS_C4 + fx_mulhi(sq, NL_TS_C5);
        pp = NL_TS_C3 + fx_mulhi(sq, pp);
        pp = NL_TS_C2 + fx_mulhi(sq, pp);
        pp = NL_TS_C1 + fx_mulhi(sq, pp);
        pp = (1L << 30) + fx_mulhi(sq, pp);                /* asin(u) / u, Q30 */
        *tilt_deg = (sqrtf(s) * (2.0f * FC_RAD2DEG_F)) * fx_to_f(pp, 30);
        return;
    }
#endif
    *tilt_deg = acosf(fc_clampf(c, -1.0f, 1.0f)) * FC_RAD2DEG_F;
}

static uint8_t sat255(uint16_t x)
{
    return x < 255u ? (uint8_t)x : 255u;
}

void nl_nav3_set_physical(nl_nav3_t *n, const float accel[3], float h_baro_rel_m, uint16_t imu_fail, uint16_t baro_fails,
                          uint16_t twi_errors)
{
    n->accel_z = accel[2];
    n->accel_x_q = (int16_t)lp_quantize(accel[0], NL_ACCEL_XY_SCALE, LP_I16_MIN, LP_I16_MAX);
    n->accel_y_q = (int16_t)lp_quantize(accel[1], NL_ACCEL_XY_SCALE, LP_I16_MIN, LP_I16_MAX);
    n->h_baro_rel_cm = (int16_t)lp_quantize(h_baro_rel_m, NL_H_BARO_SCALE, LP_I16_MIN, LP_I16_MAX);
    n->imu_fail = sat255(imu_fail);
    n->baro_fails = sat255(baro_fails);
    n->twi_errors = sat255(twi_errors);
}

void nl_slot_encode(const nl_gps_snap_t *snap, uint8_t cycle, uint8_t j, uint8_t out[NL_SLOT_LEN])
{
    if (snap == 0 || j >= NL_SLOT_PARTS) {
        out[0] = NL_SLOT_EMPTY_HDR;
        out[1] = out[2] = out[3] = out[4] = 0;
        return;
    }
    out[0] = (uint8_t)(((uint8_t)(cycle & 0x1Fu) << 3) | j);
    switch (j) {
    case NL_SLOT_STATUS:
        out[1] = snap->flags;
        out[2] = snap->num_sv;
        out[3] = snap->hacc_dm;
        out[4] = snap->reset_count;
        break;
    case NL_SLOT_OFFSET:
        (void)put16(put16(out + 1, (uint16_t)snap->off_x_cm), (uint16_t)snap->off_y_cm);
        break;
    case NL_SLOT_POS:
        (void)put16(put16(out + 1, (uint16_t)snap->px_cm), (uint16_t)snap->py_cm);
        break;
    case NL_SLOT_VEL:
        (void)put16(put16(out + 1, (uint16_t)snap->vx_cms), (uint16_t)snap->vy_cms);
        break;
    default:
        (void)put16(put16(out + 1, (uint16_t)snap->cos_q15), (uint16_t)snap->sin_q15);
        break;
    }
}

void nl_slot_tx_init(nl_slot_tx_t *tx)
{
    tx->has = false;
    tx->cycle = 0;
}

void nl_slot_tx_latch(nl_slot_tx_t *tx, const nl_gps_snap_t *snap, uint16_t cycle)
{
    tx->snap = *snap;
    tx->cycle = (uint8_t)cycle;
    tx->has = true;
}

void nl_slot_tx_slot(const nl_slot_tx_t *tx, uint32_t nav_tick, uint8_t out[NL_SLOT_LEN])
{
    nl_slot_encode(tx->has ? &tx->snap : 0, tx->cycle, (uint8_t)(nav_tick % NL_SLOT_PARTS), out);
}

void nl_slot_rx_init(nl_slot_rx_t *rx)
{
    uint8_t j;
    for (j = 0; j < NL_SLOT_PARTS; j++) {
        rx->age[j] = 0xFFu;
        rx->cyc[j] = 0;
    }
    rx->has_latest = false;
    rx->latest_age = 0xFFu;
    rx->got_part4 = false;
    rx->sets = rx->broken = 0;
}

/* sim.nlink_proto.GpsSlotRx.feed: the reference keeps absolute ticks; here each part's age counts the ticks since it
 * arrived (saturating at 255), which is exact for the only test the reference makes (tick - t_part == 4 - j <= 4) and
 * for fresh() (tick - t_latest <= 10). */
const nl_gps_set_t *nl_slot_rx_feed(nl_slot_rx_t *rx, const uint8_t *slot)
{
    uint8_t j, cyc;
    for (j = 0; j < NL_SLOT_PARTS; j++)
        if (rx->age[j] < 0xFFu)
            rx->age[j]++;
    if (rx->latest_age < 0xFFu)
        rx->latest_age++;
    rx->got_part4 = false;
    if (slot == 0)
        return 0;
    j = (uint8_t)(slot[0] & 0x07u);
    if (j >= NL_SLOT_PARTS)
        return 0;
    cyc = (uint8_t)(slot[0] >> 3);
    rx->age[j] = 0;
    rx->cyc[j] = cyc;
    rx->pl[j][0] = slot[1];
    rx->pl[j][1] = slot[2];
    rx->pl[j][2] = slot[3];
    rx->pl[j][3] = slot[4];
    if (j != NL_SLOT_PARTS - 1u)
        return 0;
    rx->got_part4 = true;
    for (j = 0; j < NL_SLOT_PARTS; j++) {
        if (rx->cyc[j] != cyc || rx->age[j] != (uint8_t)(NL_SLOT_PARTS - 1u - j)) {
            rx->broken++;
            return 0;
        }
    }
    {
        nl_gps_set_t *g = &rx->latest;
        g->flags = rx->pl[NL_SLOT_STATUS][0];
        g->num_sv = rx->pl[NL_SLOT_STATUS][1];
        g->hacc_dm = rx->pl[NL_SLOT_STATUS][2];
        g->reset_count = rx->pl[NL_SLOT_STATUS][3];
        g->off[0] = (int16_t)get16(rx->pl[NL_SLOT_OFFSET]);
        g->off[1] = (int16_t)get16(rx->pl[NL_SLOT_OFFSET] + 2);
        g->p[0] = (int16_t)get16(rx->pl[NL_SLOT_POS]);
        g->p[1] = (int16_t)get16(rx->pl[NL_SLOT_POS] + 2);
        g->v[0] = (int16_t)get16(rx->pl[NL_SLOT_VEL]);
        g->v[1] = (int16_t)get16(rx->pl[NL_SLOT_VEL] + 2);
        g->cs[0] = (int16_t)get16(rx->pl[NL_SLOT_HEAD]);
        g->cs[1] = (int16_t)get16(rx->pl[NL_SLOT_HEAD] + 2);
        g->cycle = cyc;
    }
    rx->has_latest = true;
    rx->latest_age = 0;
    rx->sets++;
    return &rx->latest;
}

bool nl_slot_rx_fresh(const nl_slot_rx_t *rx)
{
    return rx->has_latest && rx->latest_age <= NL_SLOT_STALE_TICKS;
}

bool nl_slot_rx_gate_ok(const nl_slot_rx_t *rx)
{
    return nl_slot_rx_fresh(rx) && (rx->latest.flags & 0x02u) != 0u;
}

/* ---- v0.8.1 nlink v4 ---------------------------------------------------------------------------- */
uint8_t nl_encode_nav4(const nl_nav3_t *n, uint8_t out[NL_XFER_BYTES])
{
    put_nav3(n, out);
    return seal(out, NL_TYPE_NAV4, n->seq, NL_NAV3_LEN);
}

uint8_t nl_decode_nav4(const uint8_t in[NL_XFER_BYTES], nl_nav3_t *n)
{
    if (!check(in, NL_TYPE_NAV4, NL_NAV3_LEN))
        return 0;
    get_nav3(in, n);
    return 1;
}

uint8_t nl_encode_fcu4(const nl_fcu4_t *f, uint8_t out[NL_XFER_BYTES])
{
    uint8_t i, n;
    out[4] = f->state;
    out[5] = f->fflags;
    out[6] = f->params_tag;
    out[7] = f->thr_q;
    out[8] = f->pflags;
    (void)put16(out + 9, f->kt_q);
    n = seal(out, NL_TYPE_FCU4, f->seq, NL_FCU4_LEN);
    for (i = NL_FCU4_FRAME_BYTES; i < NL_XFER_BYTES; i++)
        out[i] = 0;
    return n;
}

uint8_t nl_decode_fcu4(const uint8_t in[NL_XFER_BYTES], nl_fcu4_t *f)
{
    if (!check(in, NL_TYPE_FCU4, NL_FCU4_LEN))
        return 0;
    f->seq = in[3];
    f->state = in[4];
    f->fflags = in[5];
    f->params_tag = in[6];
    f->thr_q = in[7];
    f->pflags = in[8];
    f->kt_q = get16(in + 9);
    return 1;
}

uint8_t nl_fcu4_pflags(uint8_t phase, bool motor_on, bool sat_hi, bool low_alt, bool ground, bool lifted)
{
    return (uint8_t)((phase & NL_PF_PHASE_MASK) | (motor_on ? NL_PF_MOTOR_ON : 0u) | (sat_hi ? NL_PF_SAT_HI : 0u) |
                     (low_alt ? NL_PF_LOW_ALT : 0u) | (ground ? NL_PF_GROUND : 0u) | (lifted ? NL_PF_LIFTED : 0u));
}

uint16_t nl_kt_quant(float k_t)
{
    return (uint16_t)lp_quantize(k_t, NL_KT_SCALE, 0, LP_U16_MAX);
}

float nl_kt_value(uint16_t kt_q)
{
    return (float)kt_q * (1.0f / (float)NL_KT_SCALE);  /* a power of two: exact */
}

int16_t nl_tilt_q(float x)
{
    return (int16_t)lp_quantize_wide(x, NL_TILT_SCALE, -32767L, 32767L);
}

uint8_t nl_pc_byte(uint8_t mode, bool hold, uint8_t requests)
{
    return (uint8_t)((mode & NL_PC_MODE_MASK) | (hold ? NL_PC_HOLD : 0u) | (uint8_t)((requests & 0x0Fu) << NL_PC_REQ_SHIFT));
}

void nl_gns_tx_init(nl_gns_tx_t *tx, uint8_t step_ticks, uint8_t latency_ticks)
{
    uint8_t i;
    tx->n = step_ticks;
    tx->L = latency_ticks < NL_GNS_L_MIN ? NL_GNS_L_MIN : (latency_ticks > NL_GNS_L_MAX ? NL_GNS_L_MAX : latency_ticks);
    for (i = 0; i < NL_GNS_RING; i++)
        tx->ring[i].valid = false;
    tx->has_cur = false;
    tx->cur_m = -1;
    tx->ph_ok = false;
    tx->ph_k = 0;
    tx->ph_m = 0;
    tx->ph_i = 0;
    tx->latched = tx->missed = tx->results = 0;
}

void nl_gns_tx_result(nl_gns_tx_t *tx, uint8_t seq, const uint8_t pl[NL_GNS_RESULT_LEN], uint32_t k_rx)
{
    nl_gns_res_t *e = &tx->ring[seq & (NL_GNS_RING - 1u)];
    uint8_t i;
    e->valid = true;
    e->seq = seq;
    e->k_rx = k_rx;
    for (i = 0; i < NL_GNS_RESULT_LEN; i++)
        e->pl[i] = pl[i];
    tx->results++;
}

/* (cycle m, part i) of the frame of NAV tick k: r = k - (n - 1) - L, i = r mod n, m = (r - i) / n (floor division, m < 0
 * before the first result can be due) - counted while k advances by one, a division otherwise */
static void gns_phase(nl_gns_tx_t *tx, uint32_t k)
{
    if (tx->ph_ok && k == tx->ph_k + 1u) {
        if (++tx->ph_i >= tx->n) {
            tx->ph_i = 0;
            tx->ph_m++;
        }
    } else {
        const int64_t r = (int64_t)k - (int64_t)(tx->n - 1u) - (int64_t)tx->L;
        int64_t i = r % (int64_t)tx->n;
        if (i < 0)
            i += tx->n;
        tx->ph_i = (uint8_t)i;
        tx->ph_m = (int32_t)((r - i) / (int64_t)tx->n);
        tx->ph_ok = true;
    }
    tx->ph_k = k;
}

void nl_gns_tx_slot(nl_gns_tx_t *tx, uint32_t k, uint8_t out[NL_SLOT_LEN])
{
    int32_t m;
    uint8_t i, j;
    gns_phase(tx, k);
    m = tx->ph_m;
    i = tx->ph_i;
    if (m < 0) {
        nl_slot_encode(0, 0, 0, out);                 /* SLOT_EMPTY */
        return;
    }
    if (i == NL_SLOT4_TILT) {                         /* part 0: latch the result of cycle m, if it came in time */
        const nl_gns_res_t *e = &tx->ring[(uint8_t)m & (NL_GNS_RING - 1u)];
        const uint32_t km = (uint32_t)tx->n * (uint32_t)m + (uint32_t)(tx->n - 1u);
        if (e->valid && e->seq == (uint8_t)m && km < e->k_rx && e->k_rx <= k) {
            for (j = 0; j < NL_GNS_RESULT_LEN; j++)
                tx->cur[j] = e->pl[j];
            tx->has_cur = true;
            tx->latched++;
        } else {
            tx->has_cur = false;
            tx->missed++;
        }
        tx->cur_m = m;
    }
    if (!tx->has_cur || tx->cur_m != m) {
        nl_slot_encode(0, 0, 0, out);
        return;
    }
    out[0] = (uint8_t)(((uint8_t)((uint8_t)m & 0x1Fu) << 3) | i);
    for (j = 0; j < 4u; j++)
        out[1 + j] = tx->cur[4u * i + j];
}

void nl_gns_rx_init(nl_gns_rx_t *rx, uint8_t step_ticks)
{
    uint8_t j;
    rx->n = step_ticks;
    for (j = 0; j < NL_SLOT_PARTS; j++) {
        rx->age[j] = 0xFFu;
        rx->cyc[j] = 0;
    }
    rx->has_latest = false;
    rx->latest_age = 0xFFu;
    rx->got_part4 = false;
    rx->has_tilt = false;
    rx->tilt_on = false;
    rx->tilt[0] = rx->tilt[1] = 0;
    rx->tilt_age = 0xFFFFu;
    rx->sets = rx->broken = rx->tilts = 0;
}

/* sim.nlink_proto.GnsSlotRx.feed with ages instead of absolute ticks (exact for its tests: tick - t_part == 4 - i <= 4,
 * tick - t_latest <= 10, tick - t_tilt against 2 n and land_steps n <= 65534) */
const nl_gns_data_t *nl_gns_rx_feed(nl_gns_rx_t *rx, const uint8_t *slot)
{
    uint8_t j, i, cyc;
    for (j = 0; j < NL_SLOT_PARTS; j++)
        if (rx->age[j] < 0xFFu)
            rx->age[j]++;
    if (rx->latest_age < 0xFFu)
        rx->latest_age++;
    if (rx->tilt_age < 0xFFFFu)
        rx->tilt_age++;
    rx->got_part4 = false;
    if (slot == 0)
        return 0;
    i = (uint8_t)(slot[0] & 0x07u);
    if (i >= NL_SLOT_PARTS)
        return 0;
    cyc = (uint8_t)(slot[0] >> 3);
    if (i == NL_SLOT4_TILT) {                         /* applied at once (the builder's target from this tick) */
        rx->tilt[0] = (int16_t)get16(slot + 1);
        rx->tilt[1] = (int16_t)get16(slot + 3);
        rx->tilt_on = rx->tilt[0] != NL_TILT_NONE;
        if (!rx->tilt_on)
            rx->tilt[0] = rx->tilt[1] = 0;
        rx->has_tilt = true;
        rx->tilt_age = 0;
        rx->tilts++;
    }
    rx->age[i] = 0;
    rx->cyc[i] = cyc;
    rx->pl[i][0] = slot[1];
    rx->pl[i][1] = slot[2];
    rx->pl[i][2] = slot[3];
    rx->pl[i][3] = slot[4];
    if (i != NL_SLOT_PARTS - 1u)
        return 0;
    rx->got_part4 = true;
    for (j = 0; j < NL_SLOT_PARTS; j++) {
        if (rx->cyc[j] != cyc || rx->age[j] != (uint8_t)(NL_SLOT_PARTS - 1u - j)) {
            rx->broken++;
            return 0;
        }
    }
    {
        nl_gns_data_t *g = &rx->latest;
        g->flags = rx->pl[NL_SLOT4_STATUS][0];
        g->num_sv = rx->pl[NL_SLOT4_STATUS][1];
        g->hacc_dm = rx->pl[NL_SLOT4_STATUS][2];
        g->pc = rx->pl[NL_SLOT4_STATUS][3];
        g->pos[0] = (int16_t)get16(rx->pl[NL_SLOT4_POS]);
        g->pos[1] = (int16_t)get16(rx->pl[NL_SLOT4_POS] + 2);
        g->vel[0] = (int16_t)get16(rx->pl[NL_SLOT4_VEL]);
        g->vel[1] = (int16_t)get16(rx->pl[NL_SLOT4_VEL] + 2);
        g->trim[0] = (int16_t)get16(rx->pl[NL_SLOT4_TRIM]);
        g->trim[1] = (int16_t)get16(rx->pl[NL_SLOT4_TRIM] + 2);
        g->cycle = cyc;
        if (g->pc & NL_PC_HOLD) {                     /* GnsData.dist_m: hypot(pos / 100) */
            const float x = (float)g->pos[0] * 0.01f, y = (float)g->pos[1] * 0.01f;
            g->dist_m = sqrtf(x * x + y * y);
        } else {
            g->dist_m = NAN;
        }
    }
    rx->has_latest = true;
    rx->latest_age = 0;
    rx->sets++;
    return &rx->latest;
}

bool nl_gns_rx_fresh(const nl_gns_rx_t *rx)
{
    return rx->has_latest && rx->latest_age <= NL_SLOT_STALE_TICKS;
}

bool nl_gns_rx_gate_ok(const nl_gns_rx_t *rx)
{
    return nl_gns_rx_fresh(rx) && (rx->latest.flags & 0x02u) != 0u;
}

bool nl_gns_rx_fallback(const nl_gns_rx_t *rx)
{
    return !rx->has_tilt || rx->tilt_age >= (uint16_t)(2u * rx->n);
}

bool nl_gns_rx_target(const nl_gns_rx_t *rx, float tgt[2])
{
    const float k = 1.0f / (float)NL_TILT_SCALE;
    if (!nl_gns_rx_fallback(rx)) {
        tgt[0] = (float)rx->tilt[0] * k;
        tgt[1] = (float)rx->tilt[1] * k;
    } else if (rx->has_latest) {
        tgt[0] = (float)rx->latest.trim[0] * k;
        tgt[1] = (float)rx->latest.trim[1] * k;
    } else {
        tgt[0] = tgt[1] = 0.0f;
    }
    return rx->tilt_on;
}

uint8_t nl_gns_rx_requests(const nl_gns_rx_t *rx, uint16_t land_steps)
{
    uint8_t rq = rx->has_latest ? (uint8_t)(rx->latest.pc >> NL_PC_REQ_SHIFT) : 0u;
    if (!rx->has_tilt || (uint32_t)rx->tilt_age >= (uint32_t)land_steps * rx->n)
        rq |= 0x01u;
    return rq;
}

void nl_gns_rx_tlm(const nl_gns_rx_t *rx, bool motor_on, uint8_t *flags, uint8_t *num_sv, uint8_t *hacc_dm,
                   uint8_t *pos_mode, float *dist_m)
{
    const bool fresh = nl_gns_rx_fresh(rx);
    const nl_gns_data_t *g = &rx->latest;
    *flags = fresh ? g->flags : 0u;
    *num_sv = rx->has_latest ? g->num_sv : 0u;
    *hacc_dm = rx->has_latest ? g->hacc_dm : (uint8_t)LP_HACC_NONE;
    *pos_mode = fresh ? (uint8_t)(g->pc & NL_PC_MODE_MASK) : 0u;
    if (motor_on && rx->tilt_on && nl_gns_rx_fallback(rx))
        *pos_mode = 4u;                               /* POS_MODE_ATT: the FCU flies the trim */
    *dist_m = fresh ? g->dist_m : NAN;
}

uint8_t nl_fcu_throttle_q(float u)
{
    return (uint8_t)lp_quantize(u, NL_FCU_THR_SCALE, 0, NL_FCU_THR_SCALE);
}

float nl_fcu_throttle(uint8_t thr_q)
{
    return (float)(thr_q < NL_FCU_THR_SCALE ? thr_q : NL_FCU_THR_SCALE) / (float)NL_FCU_THR_SCALE;
}
