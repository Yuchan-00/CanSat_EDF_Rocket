/* log_record.c - see log_record.h */
#include "log_record.h"

#include <stddef.h>


uint16_t log_fletcher16(const uint8_t *data, uint16_t len)
{
    uint16_t s1 = 0, s2 = 0;     /* both kept in 0..254: one conditional subtract replaces % 255 */
    uint16_t i;
    for (i = 0; i < len; i++) {
        s1 += data[i];
        if (s1 >= 255u) s1 -= 255u;
        s2 += s1;
        if (s2 >= 255u) s2 -= 255u;
    }
    return (uint16_t)((s2 << 8) | s1);
}

uint16_t log_frame(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t *out)
{
    uint16_t i, c;
    out[0] = LOG_SYNC0;
    out[1] = LOG_SYNC1;
    out[2] = type;
    out[3] = len;
    for (i = 0; i < len; i++)
        out[4 + i] = payload[i];
    c = log_fletcher16(out + 2, (uint16_t)(len + 2));
    out[4 + len] = (uint8_t)(c & 0xFF);
    out[5 + len] = (uint8_t)(c >> 8);
    return (uint16_t)(len + LOG_FRAME_OVERHEAD);
}

static uint8_t *put_u8(uint8_t *p, uint8_t v) { *p++ = v; return p; }
static uint8_t *put_u16(uint8_t *p, uint16_t v) { *p++ = (uint8_t)v; *p++ = (uint8_t)(v >> 8); return p; }
static uint8_t *put_u32(uint8_t *p, uint32_t v)
{
    *p++ = (uint8_t)v; *p++ = (uint8_t)(v >> 8); *p++ = (uint8_t)(v >> 16); *p++ = (uint8_t)(v >> 24);
    return p;
}

#if !defined(__BYTE_ORDER__) || __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "log_fixed assumes little-endian IEEE single (AVR, x86, ARM)"
#endif
/* Byte access to the float: on the AVR a 32-bit shift by 23 is a 23-step loop, the
 * exponent bytes are two loads.  b[3] = sign | e[7:1], b[2] = e[0] | mantissa[22:16]. */
int32_t log_fixed(float x, int8_t shift, int32_t lo, int32_t hi)
{
    union {
        float f;
        uint8_t b[4];
    } v;
    uint8_t e, neg;
    int16_t es;
    int32_t i;
    v.f = x;
    neg = v.b[3] & 0x80u;
    e = (uint8_t)((uint8_t)(v.b[3] << 1) | (v.b[2] >> 7));
    if (e == 0xFFu)                                   /* inf / NaN */
        return ((v.b[2] & 0x7Fu) | v.b[1] | v.b[0]) ? 0 : (neg ? lo : hi);
    if (e == 0u)                                      /* zero / denormal */
        return 0;
    es = (int16_t)e + shift;
    if (es >= 127 + 31)                               /* |x 2^shift| >= 2^31 */
        return neg ? lo : hi;
    if (es <= 0)                                      /* underflows to 0 */
        return 0;
    v.b[3] = (uint8_t)(neg | ((uint8_t)es >> 1));    /* x * 2^shift: exponent replaced */
    v.b[2] = (uint8_t)((v.b[2] & 0x7Fu) | (uint8_t)(es << 7));
    i = (int32_t)v.f;                                 /* truncation toward zero */
    return i < lo ? lo : (i > hi ? hi : i);
}

static int16_t sat16(float x, int8_t shift)
{
    return (int16_t)log_fixed(x, shift, -32768L, 32767L);
}

static uint16_t satu16(float x, int8_t shift)
{
    return (uint16_t)log_fixed(x, shift, 0L, 65535L);
}

static int32_t sat32(float x, int8_t shift)
{
    return log_fixed(x, shift, -2147483647L - 1L, 2147483647L);
}

static uint8_t *put_vec(uint8_t *p, const float *v, uint8_t n, int8_t shift)
{
    uint8_t i;
    for (i = 0; i < n; i++)
        p = put_u16(p, (uint16_t)sat16(v[i], shift));
    return p;
}

/* byte offsets of the parts: 0 header+q, 1 gyro/accel/att_err, 2 moments/integ/delta, 3 pwm/altitude/... */
static const uint8_t k_part_start[LOG_FLIGHT_PARTS + 1] = {0, 22, 40, 60, LOG_FLIGHT_LEN};

uint8_t log_flight_part_start(uint8_t part)
{
    return part <= LOG_FLIGHT_PARTS ? k_part_start[part] : LOG_FLIGHT_LEN;
}

uint8_t log_pack_flight_part(const log_flight_t *r, uint8_t *payload, uint8_t part)
{
    uint8_t *p = payload + log_flight_part_start(part), i;
    switch (part) {
    case 0:
        p = put_u32(p, r->tick);
        p = put_u8(p, r->state);
        p = put_u8(p, r->flags);
        p = put_u8(p, r->reason);
        p = put_u8(p, r->aux_bits);
        p = put_u16(p, r->loop_us_last);
        p = put_u16(p, r->loop_us_max);
        p = put_u16(p, r->overruns);
        p = put_vec(p, r->q, 4, LOG_SHIFT_Q);
        break;
    case 1:
        p = put_vec(p, r->gyro_f, 3, LOG_SHIFT_RATE);
        p = put_vec(p, r->accel, 3, LOG_SHIFT_ACCEL);
        p = put_vec(p, r->att_err, 3, LOG_SHIFT_ANGLE);
        break;
    case 2:
        p = put_vec(p, r->moments, 3, LOG_SHIFT_MOMENT);
        p = put_vec(p, r->integ, 3, LOG_SHIFT_MOMENT);
        p = put_vec(p, r->delta, 4, LOG_SHIFT_ANGLE);
        break;
    case 3:
        for (i = 0; i < 6; i++)
            p = put_u16(p, r->pwm_counts[i]);
        p = put_u32(p, (uint32_t)sat32(r->h_est, LOG_SHIFT_H));
        p = put_u16(p, (uint16_t)sat16(r->vz_est, LOG_SHIFT_VZ));
        p = put_u32(p, (uint32_t)sat32(r->h_baro, LOG_SHIFT_H));
        p = put_u16(p, satu16(r->throttle, LOG_SHIFT_THROTTLE));
        p = put_u16(p, satu16(r->v_cell, LOG_SHIFT_VCELL));
        p = put_u16(p, (uint16_t)sat16(r->tilt_deg, LOG_SHIFT_TILT));
        p = put_u16(p, satu16(r->thrust_est, LOG_SHIFT_THRUST));
        break;
    default:
        break;
    }
    return (uint8_t)(p - payload);
}

/* the fine split (v0.7): 0 header+q [0,22), 1 gyro/accel [22,34), 2 att_err/moments [34,46), 3 integ/delta [46,60),
 * 4 pwm/h_est/vz [60,78), 5 h_baro/throttle/v_cell/tilt/thrust [78,90) */
static const uint8_t k_part_fine_start[LOG_FLIGHT_PARTS_FINE + 1] = {0, 22, 34, 46, 60, 78, LOG_FLIGHT_LEN};

uint8_t log_flight_part_fine_start(uint8_t part)
{
    return part <= LOG_FLIGHT_PARTS_FINE ? k_part_fine_start[part] : LOG_FLIGHT_LEN;
}

uint8_t log_pack_flight_part_fine(const log_flight_t *r, uint8_t *payload, uint8_t part)
{
    uint8_t *p = payload + log_flight_part_fine_start(part), i;
    switch (part) {
    case 0:
        return log_pack_flight_part(r, payload, 0);
    case 1:
        p = put_vec(p, r->gyro_f, 3, LOG_SHIFT_RATE);
        p = put_vec(p, r->accel, 3, LOG_SHIFT_ACCEL);
        break;
    case 2:
        p = put_vec(p, r->att_err, 3, LOG_SHIFT_ANGLE);
        p = put_vec(p, r->moments, 3, LOG_SHIFT_MOMENT);
        break;
    case 3:
        p = put_vec(p, r->integ, 3, LOG_SHIFT_MOMENT);
        p = put_vec(p, r->delta, 4, LOG_SHIFT_ANGLE);
        break;
    case 4:
        for (i = 0; i < 6; i++)
            p = put_u16(p, r->pwm_counts[i]);
        p = put_u32(p, (uint32_t)sat32(r->h_est, LOG_SHIFT_H));
        p = put_u16(p, (uint16_t)sat16(r->vz_est, LOG_SHIFT_VZ));
        break;
    case 5:
        p = put_u32(p, (uint32_t)sat32(r->h_baro, LOG_SHIFT_H));
        p = put_u16(p, satu16(r->throttle, LOG_SHIFT_THROTTLE));
        p = put_u16(p, satu16(r->v_cell, LOG_SHIFT_VCELL));
        p = put_u16(p, (uint16_t)sat16(r->tilt_deg, LOG_SHIFT_TILT));
        p = put_u16(p, satu16(r->thrust_est, LOG_SHIFT_THRUST));
        break;
    default:
        break;
    }
    return (uint8_t)(p - payload);
}

uint8_t log_pack_flight(const log_flight_t *r, uint8_t *payload)
{
    uint8_t part, n = 0;
    for (part = 0; part < LOG_FLIGHT_PARTS; part++)
        n = log_pack_flight_part(r, payload, part);
    return n;
}

void log_fletcher16_acc(uint16_t *s1, uint16_t *s2, const uint8_t *data, uint16_t len)
{
    uint16_t a = *s1, b = *s2, i;
    for (i = 0; i < len; i++) {
        a += data[i];
        if (a >= 255u) a -= 255u;
        b += a;
        if (b >= 255u) b -= 255u;
    }
    *s1 = a;
    *s2 = b;
}

uint8_t log_pack_event(uint32_t tick, uint8_t from, uint8_t to, uint8_t cause, uint8_t reason, uint8_t *payload)
{
    uint8_t *p = payload;
    p = put_u32(p, tick);
    p = put_u8(p, from);
    p = put_u8(p, to);
    p = put_u8(p, cause);
    p = put_u8(p, reason);
    return (uint8_t)(p - payload);
}

uint8_t log_pack_link(uint32_t tick, uint8_t seq, uint8_t cmd, uint16_t arg, uint16_t nonce, uint8_t code,
                      uint8_t state_after, int8_t rssi_dbm, int8_t snr_q4, uint8_t *payload)
{
    uint8_t *p = payload;
    p = put_u32(p, tick);
    p = put_u8(p, seq);
    p = put_u8(p, cmd);
    p = put_u16(p, arg);
    p = put_u16(p, nonce);
    p = put_u8(p, code);
    p = put_u8(p, state_after);
    p = put_u8(p, (uint8_t)rssi_dbm);
    p = put_u8(p, (uint8_t)snr_q4);
    return (uint8_t)(p - payload);
}

uint8_t log_pack_event2(uint32_t tick, uint8_t from, uint8_t to, uint8_t cause, uint8_t reason, uint8_t phase_from,
                        uint8_t phase_to, uint8_t *payload)
{
    uint8_t *p = payload + log_pack_event(tick, from, to, cause, reason, payload);
    p = put_u8(p, phase_from);
    p = put_u8(p, phase_to);
    return (uint8_t)(p - payload);
}

uint8_t log_pack_link2(uint32_t tick, uint8_t seq, uint8_t cmd, uint16_t arg, uint16_t nonce, uint8_t code,
                       uint8_t state_after, int8_t rssi_dbm, int8_t snr_q4, uint8_t phase_after, uint8_t *payload)
{
    uint8_t *p = payload + log_pack_link(tick, seq, cmd, arg, nonce, code, state_after, rssi_dbm, snr_q4, payload);
    p = put_u8(p, phase_after);
    return (uint8_t)(p - payload);
}

uint8_t log_pack_vtol(const log_vtol_t *v, uint8_t *payload)
{
    uint8_t *p = payload;
    p = put_u32(p, v->tick);
    p = put_u8(p, v->phase);
    p = put_u8(p, v->vflags);
    p = put_u8(p, v->reason);
    p = put_u8(p, v->vc_mode);
    p = put_u16(p, (uint16_t)v->h_tgt_dm);
    p = put_u32(p, (uint32_t)sat32(v->h_ref, LOG_SHIFT_H));
    p = put_u16(p, (uint16_t)sat16(v->vz_cmd, LOG_SHIFT_VZ));
    p = put_u16(p, satu16(v->u, LOG_SHIFT_THROTTLE));
    p = put_u16(p, satu16(v->u_ff, LOG_SHIFT_THROTTLE));
    p = put_u16(p, (uint16_t)sat16(v->integ, LOG_SHIFT_INTEG));
    p = put_u16(p, v->td_count);
    p = put_u16(p, v->motor_ticks);
    p = put_u8(p, v->sig);                            /* format 6 (v0.7.1) */
    p = put_u8(p, v->td_by);
    p = put_u16(p, v->low_in_land);
    p = put_u16(p, (uint16_t)sat16(v->vz_lpf, LOG_SHIFT_VZ));
    p = put_u16(p, (uint16_t)sat16(v->f_excess, LOG_SHIFT_ACCEL));
    p = put_u8(p, v->est_gate);
    return (uint8_t)(p - payload);
}

/* log_record.h: every field's payload offset (fixed by the wire format, LOG_VTOL_LEN) <= its offset in the struct, so the
 * sequential pack above never overwrites a field before it has read it */
typedef char log_vtol_in_place_ok[(offsetof(log_vtol_t, h_ref) >= 10u && offsetof(log_vtol_t, vz_cmd) >= 14u &&
                                   offsetof(log_vtol_t, u) >= 16u && offsetof(log_vtol_t, u_ff) >= 18u &&
                                   offsetof(log_vtol_t, integ) >= 20u && offsetof(log_vtol_t, td_count) >= 22u &&
                                   offsetof(log_vtol_t, motor_ticks) >= 24u && offsetof(log_vtol_t, sig) >= 26u &&
                                   offsetof(log_vtol_t, td_by) >= 27u && offsetof(log_vtol_t, low_in_land) >= 28u &&
                                   offsetof(log_vtol_t, vz_lpf) >= 30u && offsetof(log_vtol_t, f_excess) >= 32u &&
                                   offsetof(log_vtol_t, est_gate) >= 34u && sizeof(log_vtol_t) >= LOG_VTOL_LEN) ? 1 : -1];

uint8_t log_pack_vtol_in_place(log_vtol_t *v)
{
    return log_pack_vtol(v, (uint8_t *)(void *)v);
}

uint8_t log_pack_status(const log_status_t *s, uint8_t *payload)
{
    uint8_t *p = payload;
    p = put_u32(p, s->tick);
    p = put_u16(p, s->stack_free);
    p = put_u16(p, s->twi_errors);
    p = put_u16(p, s->uart_dropped);
    p = put_u16(p, s->link_frames);
    p = put_u16(p, s->link_bad);
    p = put_u16(p, s->link_other);
    p = put_u16(p, s->radio_errors);
    p = put_u16(p, s->replies);
    p = put_u16(p, s->baro_fails);
    p = put_u16(p, s->imu_fails);
    p = put_u16(p, s->vbat_mv);
    p = put_u8(p, s->reset_cause);
    p = put_u8(p, (uint8_t)s->rssi_dbm);
    p = put_u16(p, s->il_cmds);
    p = put_u16(p, s->il_snaps);
    p = put_u16(p, s->il_bad);
    p = put_u16(p, s->il_uart_errors);
    p = put_u16(p, s->il_dropped);
    p = put_u8(p, s->com_age_s);
    p = put_u8(p, s->com_flags);
    return (uint8_t)(p - payload);
}

uint8_t log_pack_com(uint32_t tick, const uint8_t stat_payload[LOG_COM_STAT_LEN], uint8_t *payload)
{
    uint8_t *p = put_u32(payload, tick), i;
    for (i = 0; i < LOG_COM_STAT_LEN; i++)
        *p++ = stat_payload[i];
    return (uint8_t)(p - payload);
}

uint8_t log_pack_nav(const log_nav_t *n, uint8_t *payload)
{
    uint8_t *p = payload;
    p = put_u32(p, n->tick);
    p = put_u16(p, n->frames);
    p = put_u16(p, n->bad);
    p = put_u16(p, n->timeouts);
    p = put_u16(p, n->seq_lost);
    p = put_u16(p, n->resyncs);
    p = put_u16(p, n->spi_overruns);
    p = put_u16(p, n->nav_loop_us_max);
    p = put_u8(p, n->nav_flags);
    p = put_u8(p, n->nav_reset_cause);
    return (uint8_t)(p - payload);
}

/* ---- format 7 'G' (v0.8): integers in struct order, little-endian ---- */
uint8_t log_pack_gps(const log_gps_t *g, uint8_t *payload)
{
    uint8_t *p = payload;
    uint8_t i;
    p = put_u32(p, g->tick);
    p = put_u8(p, g->gps_flags);
    p = put_u8(p, g->num_sv);
    p = put_u8(p, g->hacc_dm);
    p = put_u8(p, g->reset_count);
    p = put_u8(p, g->pos_mode);
    p = put_u8(p, g->requests);
    p = put_u8(p, g->gflags);
    p = put_u8(p, g->set_age);
    for (i = 0; i < 2u; i++) p = put_u16(p, (uint16_t)g->p_cm[i]);
    for (i = 0; i < 2u; i++) p = put_u16(p, (uint16_t)g->v_cms[i]);
    for (i = 0; i < 2u; i++) p = put_u16(p, (uint16_t)g->hold_cm[i]);
    p = put_u16(p, g->dist_dm);
    for (i = 0; i < 2u; i++) p = put_u16(p, (uint16_t)g->tilt_mrad[i]);
    for (i = 0; i < 2u; i++) p = put_u16(p, (uint16_t)g->head_q15[i]);
    p = put_u16(p, g->sets);
    p = put_u16(p, g->broken);
    p = put_u16(p, g->bg_late);
    return (uint8_t)(p - payload);
}

/* in place: every field's payload offset is <= its struct offset (the struct holds them in the same order) */
typedef char log_gps_in_place_ok[(offsetof(log_gps_t, p_cm) >= 12u && offsetof(log_gps_t, v_cms) >= 16u &&
                                  offsetof(log_gps_t, hold_cm) >= 20u && offsetof(log_gps_t, dist_dm) >= 24u &&
                                  offsetof(log_gps_t, tilt_mrad) >= 26u && offsetof(log_gps_t, head_q15) >= 30u &&
                                  offsetof(log_gps_t, sets) >= 34u && offsetof(log_gps_t, broken) >= 36u &&
                                  offsetof(log_gps_t, bg_late) >= 38u && sizeof(log_gps_t) >= LOG_GPS_LEN) ? 1 : -1];

uint8_t log_pack_gps_in_place(log_gps_t *g)
{
    return log_pack_gps(g, (uint8_t *)(void *)g);
}

/* format 8: the GNS form of 'G' */
uint8_t log_pack_gns(const log_gns_t *g, uint8_t *payload)
{
    uint8_t *p = payload;
    uint8_t i;
    p = put_u32(p, g->tick);
    p = put_u8(p, g->gps_flags);
    p = put_u8(p, g->num_sv);
    p = put_u8(p, g->hacc_dm);
    p = put_u8(p, g->pc);
    p = put_u8(p, g->gflags);
    p = put_u8(p, g->set_age);
    for (i = 0; i < 2u; i++) p = put_u16(p, (uint16_t)g->pos_cm[i]);
    for (i = 0; i < 2u; i++) p = put_u16(p, (uint16_t)g->vel_cms[i]);
    for (i = 0; i < 2u; i++) p = put_u16(p, (uint16_t)g->trim[i]);
    for (i = 0; i < 2u; i++) p = put_u16(p, (uint16_t)g->tilt_tgt[i]);
    for (i = 0; i < 2u; i++) p = put_u16(p, (uint16_t)g->tilt_mrad[i]);
    p = put_u16(p, g->dist_dm);
    p = put_u16(p, g->sets);
    p = put_u16(p, g->broken);
    p = put_u16(p, g->tilts);
    p = put_u16(p, g->bg_late);
    return (uint8_t)(p - payload);
}

/* in place: the struct holds the fields in payload order without padding (every payload offset <= its struct offset) */
typedef char log_gns_in_place_ok[(offsetof(log_gns_t, pos_cm) >= 10u && offsetof(log_gns_t, vel_cms) >= 14u &&
                                  offsetof(log_gns_t, trim) >= 18u && offsetof(log_gns_t, tilt_tgt) >= 22u &&
                                  offsetof(log_gns_t, tilt_mrad) >= 26u && offsetof(log_gns_t, dist_dm) >= 30u &&
                                  offsetof(log_gns_t, sets) >= 32u && offsetof(log_gns_t, broken) >= 34u &&
                                  offsetof(log_gns_t, tilts) >= 36u && offsetof(log_gns_t, bg_late) >= 38u &&
                                  sizeof(log_gns_t) >= LOG_GNS_LEN) ? 1 : -1];

uint8_t log_pack_gns_in_place(log_gns_t *g)
{
    return log_pack_gns(g, (uint8_t *)(void *)g);
}
