/* gps_ubx.c - see gps_ubx.h (mirrors sim/gps_nav.py Part 1, the normative reference, branch by branch). */
#include "gps_ubx.h"

/* parser states (sim/gps_nav.py _S_*) */
enum { S_SYNC1 = 0, S_SYNC2, S_CLASS, S_ID, S_LEN1, S_LEN2, S_PAYLOAD, S_CKA, S_CKB };

#define PART_POS 1u
#define PART_VEL 2u
#define PART_SOL 4u

/* ---- little-endian helpers ---- */
static uint16_t le16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)le16(p) | ((uint32_t)le16(p + 2) << 16);
}

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

/* ---- frames ---- */
void ubx_checksum(const uint8_t *data, uint16_t n, uint8_t *ck_a, uint8_t *ck_b)
{
    uint8_t a = 0, b = 0;
    uint16_t i;
    for (i = 0; i < n; i++) {
        a = (uint8_t)(a + data[i]);
        b = (uint8_t)(b + a);
    }
    *ck_a = a;
    *ck_b = b;
}

uint16_t ubx_frame(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t n, uint8_t *out)
{
    uint16_t i;
    out[0] = UBX_SYNC1;
    out[1] = UBX_SYNC2;
    out[2] = cls;
    out[3] = id;
    out[4] = (uint8_t)n;
    out[5] = (uint8_t)(n >> 8);
    if (payload != out + 6)
        for (i = 0; i < n; i++)
            out[6 + i] = payload[i];
    ubx_checksum(out + 2, (uint16_t)(n + 4u), &out[6 + n], &out[7 + n]);
    return (uint16_t)(n + UBX_OVERHEAD);
}

/* the payloads are written in place at out + 6, then sealed */
uint16_t ubx_cfg_prt_uart1(uint32_t baud, uint8_t out[UBX_OVERHEAD + UBX_LEN_CFG_PRT])
{
    uint8_t *p = out + 6;
    *p++ = 1;                                          /* portID UART1 */
    *p++ = 0;                                          /* reserved0 */
    p = put16(p, 0);                                   /* txReady */
    p = put32(p, UBX_PRT_MODE_8N1);
    p = put32(p, baud);
    p = put16(p, UBX_PROTO_UBX | UBX_PROTO_NMEA);      /* inProtoMask */
    p = put16(p, UBX_PROTO_UBX);                       /* outProtoMask: UBX only */
    p = put16(p, 0);                                   /* flags */
    (void)put16(p, 0);                                 /* reserved5 */
    return ubx_frame(UBX_CLS_CFG, UBX_ID_CFG_PRT, out + 6, UBX_LEN_CFG_PRT, out);
}

uint16_t ubx_cfg_rate(uint16_t meas_ms, uint8_t out[UBX_OVERHEAD + UBX_LEN_CFG_RATE])
{
    uint8_t *p = out + 6;
    p = put16(p, meas_ms);
    p = put16(p, 1);                                   /* navRate: 1 cycle */
    (void)put16(p, 1);                                 /* timeRef: GPS time */
    return ubx_frame(UBX_CLS_CFG, UBX_ID_CFG_RATE, out + 6, UBX_LEN_CFG_RATE, out);
}

uint16_t ubx_cfg_msg(uint8_t cls, uint8_t id, uint8_t rate, uint8_t out[UBX_OVERHEAD + UBX_LEN_CFG_MSG])
{
    out[6] = cls;
    out[7] = id;
    out[8] = rate;
    return ubx_frame(UBX_CLS_CFG, UBX_ID_CFG_MSG, out + 6, UBX_LEN_CFG_MSG, out);
}

uint16_t ubx_cfg_nav5_airborne(uint8_t out[UBX_OVERHEAD + UBX_LEN_CFG_NAV5])
{
    uint8_t i;
    for (i = 0; i < UBX_LEN_CFG_NAV5; i++)
        out[6 + i] = 0;
    (void)put16(out + 6, UBX_NAV5_MASK_DYN);           /* mask: dynModel only */
    out[8] = UBX_DYN_AIRBORNE_1G;                      /* dynModel 6: airborne < 1 g */
    return ubx_frame(UBX_CLS_CFG, UBX_ID_CFG_NAV5, out + 6, UBX_LEN_CFG_NAV5, out);
}

uint16_t ubx_mon_ver_poll(uint8_t out[UBX_OVERHEAD])
{
    return ubx_frame(UBX_CLS_MON, UBX_ID_MON_VER, out + 6, 0, out);
}

/* ---- parser ---- */
void ubx_parser_init(ubx_parser_t *p)
{
    p->state = S_SYNC1;
    p->cls = p->id = 0;
    p->ck_a = p->ck_b = 0;
    p->length = 0;
    p->n = 0;
    p->stats.frames = p->stats.bad_ck = p->stats.bad_len = p->stats.truncated = p->stats.skipped = 0;
}

static void ck(ubx_parser_t *p, uint8_t b)
{
    p->ck_a = (uint8_t)(p->ck_a + b);
    p->ck_b = (uint8_t)(p->ck_b + p->ck_a);
}

uint8_t ubx_parse_byte(ubx_parser_t *p, uint8_t b)
{
    switch (p->state) {
    case S_SYNC1:
        if (b == UBX_SYNC1)
            p->state = S_SYNC2;
        else
            p->stats.skipped++;
        return 0;
    case S_SYNC2:
        if (b == UBX_SYNC2) {
            p->state = S_CLASS;
            p->ck_a = p->ck_b = 0;
        } else if (b == UBX_SYNC1) {
            p->stats.skipped++;                        /* B5 B5 62: the second B5 starts the frame */
        } else {
            p->stats.skipped = (uint16_t)(p->stats.skipped + 2u);
            p->state = S_SYNC1;
        }
        return 0;
    case S_CLASS:
        p->cls = b;
        ck(p, b);
        p->state = S_ID;
        return 0;
    case S_ID:
        p->id = b;
        ck(p, b);
        p->state = S_LEN1;
        return 0;
    case S_LEN1:
        p->length = b;
        ck(p, b);
        p->state = S_LEN2;
        return 0;
    case S_LEN2:
        p->length = (uint16_t)(p->length | ((uint16_t)b << 8));
        ck(p, b);
        if (p->length > UBX_LEN_LIMIT) {
            p->stats.bad_len++;
            p->state = S_SYNC1;
            return 0;
        }
        p->n = 0;
        p->state = p->length > 0u ? S_PAYLOAD : S_CKA;
        return 0;
    case S_PAYLOAD:
        if (p->n < UBX_MAX_PAYLOAD)
            p->buf[p->n] = b;
        p->n++;
        ck(p, b);
        if (p->n >= p->length)
            p->state = S_CKA;
        return 0;
    case S_CKA:
        if (b != p->ck_a) {
            p->stats.bad_ck++;
            p->state = S_SYNC1;
            return 0;
        }
        p->state = S_CKB;
        return 0;
    default:                                           /* S_CKB */
        p->state = S_SYNC1;
        if (b != p->ck_b) {
            p->stats.bad_ck++;
            return 0;
        }
        p->stats.frames++;
        if (p->length > UBX_MAX_PAYLOAD)
            p->stats.truncated++;
        return 1;
    }
}

uint8_t ubx_msg_kind(const ubx_parser_t *p)
{
    const uint8_t c = p->cls, i = p->id;
    const uint16_t n = p->length;
    if (n > UBX_MAX_PAYLOAD)                           /* truncated: only MON-VER (its first 64 bytes) */
        return (c == UBX_CLS_MON && i == UBX_ID_MON_VER) ? UBX_K_MONVER : UBX_K_NONE;
    if (c == UBX_CLS_NAV) {
        if (i == UBX_ID_NAV_POSLLH && n == UBX_LEN_POSLLH)
            return UBX_K_POSLLH;
        if (i == UBX_ID_NAV_VELNED && n == UBX_LEN_VELNED)
            return UBX_K_VELNED;
        if (i == UBX_ID_NAV_SOL && n == UBX_LEN_SOL)
            return UBX_K_SOL;
        return UBX_K_NONE;
    }
    if (c == UBX_CLS_ACK && n == 2u)
        return i == UBX_ID_ACK_ACK ? UBX_K_ACK : (i == UBX_ID_ACK_NAK ? UBX_K_NAK : UBX_K_NONE);
    if (c == UBX_CLS_MON && i == UBX_ID_MON_VER && n >= UBX_LEN_MON_VER_MIN)
        return UBX_K_MONVER;
    return UBX_K_NONE;
}

void ubx_get_posllh(const uint8_t *pl, ubx_posllh_t *m)
{
    m->itow = le32(pl);
    m->lon = (int32_t)le32(pl + 4);
    m->lat = (int32_t)le32(pl + 8);
    m->height = (int32_t)le32(pl + 12);
    m->hmsl = (int32_t)le32(pl + 16);
    m->hacc = le32(pl + 20);
    m->vacc = le32(pl + 24);
}

void ubx_get_velned(const uint8_t *pl, ubx_velned_t *m)
{
    m->itow = le32(pl);
    m->vel_n = (int32_t)le32(pl + 4);
    m->vel_e = (int32_t)le32(pl + 8);
    m->vel_d = (int32_t)le32(pl + 12);
    m->speed = le32(pl + 16);
    m->gspeed = le32(pl + 20);
    m->heading = (int32_t)le32(pl + 24);
    m->sacc = le32(pl + 28);
    m->cacc = le32(pl + 32);
}

void ubx_get_sol(const uint8_t *pl, ubx_sol_t *m)
{
    m->itow = le32(pl);
    m->ftow = (int32_t)le32(pl + 4);
    m->week = (int16_t)le16(pl + 8);
    m->gps_fix = pl[10];
    m->flags = pl[11];
    m->ecef[0] = (int32_t)le32(pl + 12);
    m->ecef[1] = (int32_t)le32(pl + 16);
    m->ecef[2] = (int32_t)le32(pl + 20);
    m->pacc = le32(pl + 24);
    m->ecef_v[0] = (int32_t)le32(pl + 28);
    m->ecef_v[1] = (int32_t)le32(pl + 32);
    m->ecef_v[2] = (int32_t)le32(pl + 36);
    m->sacc = le32(pl + 40);
    m->pdop = le16(pl + 44);
    m->res1 = pl[46];
    m->num_sv = pl[47];
    m->res2 = le32(pl + 48);
}

/* Python str.strip() on the ASCII decode: the characters str.isspace() is true for */
static bool py_space(uint8_t c)
{
    return c == 0x20u || (c >= 0x09u && c <= 0x0Du) || (c >= 0x1Cu && c <= 0x1Fu);
}

/* "00040007" (u-blox 6) / "00070000" (u-blox 7, NEO-7M: 2026-11-06, report only) / "00080000" (u-blox 8): '0' everywhere
 * but at 3 and 7 - compared without string tables (a string literal is .data, i.e. RAM, on the AVR) */
uint8_t ubx_monver_module(const uint8_t *pl)
{
    const uint8_t *hw = pl + 30;
    uint8_t lo = 0, hi = 0, i;
    while (hi < 10u && hw[hi] != 0u)                   /* up to the first NUL of char[10] */
        hi++;
    while (lo < hi && py_space(hw[lo]))
        lo++;
    while (hi > lo && py_space(hw[hi - 1u]))
        hi--;
    if (hi - lo != 8u)
        return UBX_MODULE_UNKNOWN;
    hw += lo;
    for (i = 0; i < 7u; i++)
        if (i != 3u && hw[i] != (uint8_t)'0')
            return UBX_MODULE_UNKNOWN;
    if (hw[3] == (uint8_t)'4' && hw[7] == (uint8_t)'7')
        return UBX_MODULE_UBLOX6;
    if (hw[3] == (uint8_t)'7' && hw[7] == (uint8_t)'0')
        return UBX_MODULE_UBLOX7;
    if (hw[3] == (uint8_t)'8' && hw[7] == (uint8_t)'0')
        return UBX_MODULE_UBLOX8;
    return UBX_MODULE_UNKNOWN;
}

/* ---- epoch assembly ---- */
void gps_asm_init(gps_asm_t *a)
{
    a->has_itow = false;
    a->mask = 0;
    a->itow = 0;
    a->epochs = a->incomplete = a->duplicate = 0;
}

const gps_epoch_t *gps_asm_add(gps_asm_t *a, uint8_t kind, const uint8_t *pl, uint32_t tick)
{
    uint8_t part;
    const uint32_t itow = le32(pl);                    /* iTOW is the first field of all three */
    if (kind == UBX_K_POSLLH)
        part = PART_POS;
    else if (kind == UBX_K_VELNED)
        part = PART_VEL;
    else if (kind == UBX_K_SOL)
        part = PART_SOL;
    else
        return 0;
    if (!a->has_itow || itow != a->itow) {
        if (a->mask)
            a->incomplete++;
        a->has_itow = true;
        a->itow = itow;
        a->mask = 0;
    }
    if (a->mask & part)
        a->duplicate++;
    a->mask |= part;
    if (part == PART_POS) {                            /* the latest message of each kind wins (a duplicate replaces) */
        a->e.itow = itow;
        a->e.lon = (int32_t)le32(pl + 4);
        a->e.lat = (int32_t)le32(pl + 8);
        a->e.hmsl = (int32_t)le32(pl + 16);
        a->e.hacc = le32(pl + 20);
        a->e.vacc = le32(pl + 24);
    } else if (part == PART_VEL) {
        a->e.vel_n = (int32_t)le32(pl + 4);
        a->e.vel_e = (int32_t)le32(pl + 8);
        a->e.vel_d = (int32_t)le32(pl + 12);
        a->e.sacc = le32(pl + 28);
    } else {
        a->e.gps_fix = pl[10];
        a->e.fix_ok = (pl[11] & UBX_SOL_FLAG_FIX_OK) != 0u;
        a->e.pacc = le32(pl + 24);
        a->e.num_sv = pl[47];
    }
    if (a->mask != (PART_POS | PART_VEL | PART_SOL))
        return 0;
    a->has_itow = false;
    a->mask = 0;
    a->epochs++;
    a->e.tick = tick;
    return &a->e;
}

void gps_itow_init(gps_itow_t *c, uint8_t ms_per_tick, uint16_t latency_ms)
{
    c->ms_per_tick = ms_per_tick;
    c->latency_ms = latency_ms;
    c->has_offset = false;
    c->offset = 0;
    c->relearned = 0;
}

uint32_t gps_itow_update(gps_itow_t *c, uint32_t itow_ms, uint32_t tick)
{
    const int32_t d = (int32_t)((uint32_t)c->ms_per_tick * tick - itow_ms);   /* mod 2^32 as int32 */
    if (c->has_offset) {
        const int32_t dd = (int32_t)((uint32_t)d - (uint32_t)c->offset);
        if (dd > UBX_ITOW_JUMP_MS || dd < -UBX_ITOW_JUMP_MS) {
            c->relearned++;
            c->offset = d;
        } else {
            const int32_t leak = (int32_t)((uint32_t)c->offset + (uint32_t)UBX_ITOW_LEAK_MS);
            c->offset = d < leak ? d : leak;
        }
    } else {
        c->has_offset = true;
        c->offset = d;
    }
    return itow_ms + (uint32_t)c->offset - c->latency_ms;
}

/* ---- boot configuration ---- */
/* the ACK-checked sequence (sim/gps_nav.py config_sequence): RATE, MSG POSLLH, MSG VELNED, MSG SOL, NAV5 - its frames
 * are GPS_TX_RATE + idx; the id an ACK names (no tables: .data is RAM on the AVR) */
static uint8_t seq_tx(uint8_t idx)
{
    return (uint8_t)(GPS_TX_RATE + idx);
}

static uint8_t seq_id(uint8_t idx)
{
    return idx == 0u ? UBX_ID_CFG_RATE : (idx < 4u ? UBX_ID_CFG_MSG : UBX_ID_CFG_NAV5);
}

void gps_cfg_init(gps_cfg_t *c, uint16_t n_wait, uint16_t n_ack, uint16_t n_settle, uint16_t n_ver, uint32_t baud)
{
    c->state = GPS_CFGS_WAIT;
    c->t = 0;
    c->tries = 0;
    c->idx = 0;
    c->acked = 0;
    c->n_wait = n_wait;
    c->n_ack = n_ack;
    c->n_settle = n_settle;
    c->n_ver = n_ver;
    c->baud = baud;
    c->prt_acked = c->cfg_ok = c->ver_ok = false;
    c->module = UBX_MODULE_UNKNOWN;
    c->naks = c->timeouts = 0;
    c->ack = 0;
    c->ver_seen = false;
    c->ver_module = UBX_MODULE_UNKNOWN;
}

bool gps_cfg_done(const gps_cfg_t *c)
{
    return c->state == GPS_CFGS_DONE;
}

void gps_cfg_msg(gps_cfg_t *c, uint8_t kind, uint8_t cls, uint8_t id, uint8_t module)
{
    if (kind == UBX_K_ACK || kind == UBX_K_NAK) {
        uint8_t want;
        if (c->state == GPS_CFGS_PROBE || c->state == GPS_CFGS_PRT)
            want = UBX_ID_CFG_PRT;
        else if (c->state == GPS_CFGS_SEQ && c->idx < GPS_CFG_SEQ_N)
            want = seq_id(c->idx);
        else
            return;
        if (cls == UBX_CLS_CFG && id == want)
            c->ack = kind;                             /* the last ACK / NAK of the tick decides (got[-1]) */
    } else if (kind == UBX_K_MONVER) {
        c->ver_seen = true;
        c->ver_module = module;                        /* the last MON-VER of the tick (vs[-1]) */
    }
}

static void go(gps_cfg_t *c, uint8_t st)
{
    c->state = st;
    c->t = 0;
}

void gps_cfg_tick(gps_cfg_t *c, gps_cfg_action_t *act)
{
    const uint8_t got = c->ack, s = c->state;
    act->baud = 0;
    act->tx = GPS_TX_NONE;
    c->ack = 0;
    if (c->t < 0xFFFFu)
        c->t++;
    if (s == GPS_CFGS_WAIT) {
        if (c->t >= c->n_wait) {
            act->baud = c->baud;
            act->tx = GPS_TX_PRT;
            c->tries = 1;
            go(c, GPS_CFGS_PROBE);
        }
    } else if (s == GPS_CFGS_PROBE || s == GPS_CFGS_PRT) {
        if (got == UBX_K_ACK) {
            c->prt_acked = true;
            act->tx = seq_tx(0);
            c->idx = 0;
            c->tries = 1;
            go(c, GPS_CFGS_SEQ);
        } else if (got || c->t >= c->n_ack) {
            if (got)
                c->naks++;
            else
                c->timeouts++;
            if (s == GPS_CFGS_PROBE) {                 /* not at the configured baud: the factory default 9600 */
                act->baud = UBX_BAUD_DEFAULT;
                act->tx = GPS_TX_PRT;
                go(c, GPS_CFGS_SETTLE);
            } else if (c->tries < UBX_CFG_TRIES) {
                c->tries++;
                act->baud = UBX_BAUD_DEFAULT;
                act->tx = GPS_TX_PRT;
                go(c, GPS_CFGS_SETTLE);
            } else {                                   /* CFG-PRT never acknowledged: continue at the baud without it */
                act->tx = seq_tx(0);
                c->idx = 0;
                c->tries = 1;
                go(c, GPS_CFGS_SEQ);
            }
        }
    } else if (s == GPS_CFGS_SETTLE) {
        if (c->t >= c->n_settle) {
            act->baud = c->baud;
            act->tx = GPS_TX_PRT;
            go(c, GPS_CFGS_PRT);
        }
    } else if (s == GPS_CFGS_SEQ) {
        const bool ok = got == UBX_K_ACK;
        const bool fail = (got != 0u && !ok) || c->t >= c->n_ack;
        if (ok || (fail && c->tries >= UBX_CFG_TRIES)) {
            if (ok) {
                c->acked++;
            } else if (got) {
                c->naks++;
            } else {
                c->timeouts++;
            }
            c->idx++;
            c->tries = 1;
            if (c->idx < GPS_CFG_SEQ_N) {
                act->tx = seq_tx(c->idx);
                go(c, GPS_CFGS_SEQ);
            } else {
                c->cfg_ok = c->prt_acked && c->acked == GPS_CFG_SEQ_N;
                act->tx = GPS_TX_VER;
                c->tries = 1;
                go(c, GPS_CFGS_VER);
            }
        } else if (fail) {
            if (got)
                c->naks++;
            else
                c->timeouts++;
            c->tries++;
            act->tx = seq_tx(c->idx);
            go(c, GPS_CFGS_SEQ);
        }
    } else if (s == GPS_CFGS_VER) {
        if (c->ver_seen) {
            c->ver_ok = true;
            c->module = c->ver_module;
            go(c, GPS_CFGS_DONE);
        } else if (c->t >= c->n_ver) {
            if (c->tries < UBX_CFG_TRIES) {
                c->tries++;
                act->tx = GPS_TX_VER;
                go(c, GPS_CFGS_VER);
            } else {
                c->timeouts++;
                go(c, GPS_CFGS_DONE);
            }
        }
    }
    c->ver_seen = false;
}

uint16_t gps_cfg_frame(uint8_t tx, uint32_t baud, uint16_t meas_ms, uint8_t out[UBX_FRAME_MAX])
{
    switch (tx) {
    case GPS_TX_PRT:
        return ubx_cfg_prt_uart1(baud, out);
    case GPS_TX_RATE:
        return ubx_cfg_rate(meas_ms, out);
    case GPS_TX_MSG_POSLLH:
        return ubx_cfg_msg(UBX_CLS_NAV, UBX_ID_NAV_POSLLH, 1, out);
    case GPS_TX_MSG_VELNED:
        return ubx_cfg_msg(UBX_CLS_NAV, UBX_ID_NAV_VELNED, 1, out);
    case GPS_TX_MSG_SOL:
        return ubx_cfg_msg(UBX_CLS_NAV, UBX_ID_NAV_SOL, 1, out);
    case GPS_TX_NAV5:
        return ubx_cfg_nav5_airborne(out);
    case GPS_TX_VER:
        return ubx_mon_ver_poll(out);
    default:
        return 0;
    }
}
