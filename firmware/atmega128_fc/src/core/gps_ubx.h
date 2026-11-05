/* gps_ubx.h - the u-blox UBX protocol on the NAV (v0.8 GPS), a byte-exact port of sim/gps_nav.py Part 1 (portable C99,
 * no AVR headers): Fletcher checksum, frame encoding, the boot configuration frames, the streaming parser, the message
 * decoders, the epoch assembler (iTOW matching), the iTOW clock and the ACK-checked boot configurator.
 *
 * Hardware (params.json electronics.gps, flight_computer.nav.gps; architect v0.8): u-blox NEO-6M on a GY-NEO6MV2 board on
 * the NAV's USART1 (PD2 RXD1 / PD3 TXD1), 38400 8N1 through a level shifter; UBX NAV-POSLLH + NAV-VELNED + NAV-SOL at
 * 5 Hz.  The NEO-6M has no flash: the NAV sends its configuration at every boot (gps_cfg_*).
 *
 * Frame: 0xB5 0x62 | class | id | length u16 LE | payload | CK_A CK_B, the 8-bit Fletcher checksum over class, id, the two
 * length bytes and the payload (CK_A += b; CK_B += CK_A, mod 256).  Messages used (u-blox 6 protocol, GPS.G6-SW-10018):
 *   NAV-POSLLH 01 02 28 B  iTOW U4 ms, lon I4 1e-7 deg, lat I4, height I4 mm, hMSL I4 mm, hAcc U4 mm, vAcc U4 mm
 *   NAV-SOL    01 06 52 B  iTOW U4, fTOW I4, week I2, gpsFix U1 (3 = 3D), flags X1 (bit0 gpsFixOK), ecefX/Y/Z I4 cm, pAcc U4
 *                          cm, ecefVX/VY/VZ I4 cm/s, sAcc U4 cm/s, pDOP U2, res U1, numSV U1, res U4
 *   NAV-VELNED 01 12 36 B  iTOW U4, velN/E/D I4 cm/s, speed U4, gSpeed U4, heading I4 1e-5 deg, sAcc U4 cm/s, cAcc U4
 *   ACK-ACK 05 01 / ACK-NAK 05 00  2 B: clsID, msgID;  MON-VER 0A 04 (poll: length 0; answer: sw char[30], hw char[10],
 *   ...);  CFG-PRT 06 00 20 B, CFG-RATE 06 08 6 B, CFG-MSG 06 01 3 B, CFG-NAV5 06 24 36 B (mask 0x0001: dynModel 6 only).
 *
 * Parser (ubx_parse_byte, bounded work per byte, no replay): SYNC1 (0xB5, else skipped) -> SYNC2 (0x62; another 0xB5
 * keeps SYNC2; else skipped += 2, back to SYNC1) -> class -> id -> length (> UBX_LEN_LIMIT: bad_len, back to SYNC1) ->
 * payload (the first UBX_MAX_PAYLOAD bytes stored, the rest counted: truncated) -> CK_A -> CK_B (either wrong: bad_ck,
 * back to SYNC1).  A frame starting inside a rejected candidate is lost (the next 5 Hz epoch replaces it).  Counters
 * uint16 (wrap).  Measured on the AVR: see the c-core v0.8 report (cycles per byte / per message).
 *
 * Epoch assembly (gps_asm_add): POSLLH, VELNED and SOL with the SAME iTOW form one epoch, in any order, complete at the
 * NAV tick of its last message; a message with another iTOW drops an open epoch (incomplete).  Why: the three messages
 * leave the receiver 10-15 ms apart; pairing by arrival would mix epochs.
 * iTOW clock (gps_itow_update): D = (NAV_ms - iTOW) mod 2^32 as int32 at the completion tick (NAV_ms = tick x ms_per_tick);
 * offset O = D on the first epoch or when |D - O| > UBX_ITOW_JUMP_MS, else O = min(D, O + UBX_ITOW_LEAK_MS); the
 * measurement time t_e = iTOW + O - latency_ms (uint32) - the arrival jitter is removed by the minimum, the latency
 * (params flight_computer.nav.gps.latency_s_ASSUMED) stays ASSUMED.
 * Boot configurator (gps_cfg_*; sim/gps_nav.py GpsConfigurator, one call per NAV tick, never blocks): wait; probe at 38400
 * (CFG-PRT, ACK checked); else 9600 (the factory default) CFG-PRT (no ACK expected: the receiver switches), settle, back
 * to 38400, CFG-PRT with its ACK checked (UBX_CFG_TRIES in all); then CFG-RATE, CFG-MSG x 3, CFG-NAV5, each ACK-ACK
 * checked (NAK or timeout: retry, UBX_CFG_TRIES in all); then poll MON-VER.  cfg_ok = every frame acknowledged, ver_ok =
 * MON-VER answered, module from hwVersion ("00040007" u-blox 6, "00070000" u-blox 7 / NEO-7M, "00080000" u-blox 8;
 * the module class is reported only (the GNS bench / debugger), nothing depends on it; clones often answer "00040007" too:
 * a fake is NOT reliably detected).
 *
 * Invariant: every function here gives exactly the bytes, messages, counters, epochs, times and actions of the Python
 * reference for the same input (host vectors sim/out/gps_vectors.json, tools/crosscheck_controller.py UBX section).
 */
#ifndef FC_GPS_UBX_H
#define FC_GPS_UBX_H

#include <stdbool.h>
#include <stdint.h>

#define UBX_SYNC1 0xB5u
#define UBX_SYNC2 0x62u
#define UBX_CLS_NAV 0x01u
#define UBX_CLS_ACK 0x05u
#define UBX_CLS_CFG 0x06u
#define UBX_CLS_MON 0x0Au
#define UBX_ID_NAV_POSLLH 0x02u
#define UBX_ID_NAV_SOL 0x06u
#define UBX_ID_NAV_VELNED 0x12u
#define UBX_ID_ACK_NAK 0x00u
#define UBX_ID_ACK_ACK 0x01u
#define UBX_ID_CFG_PRT 0x00u
#define UBX_ID_CFG_MSG 0x01u
#define UBX_ID_CFG_RATE 0x08u
#define UBX_ID_CFG_NAV5 0x24u
#define UBX_ID_MON_VER 0x04u
#define UBX_LEN_POSLLH 28u
#define UBX_LEN_VELNED 36u
#define UBX_LEN_SOL 52u
#define UBX_LEN_CFG_PRT 20u
#define UBX_LEN_CFG_RATE 6u
#define UBX_LEN_CFG_MSG 3u
#define UBX_LEN_CFG_NAV5 36u
#define UBX_LEN_MON_VER_MIN 40u        /* sw char[30] + hw char[10] */
#define UBX_OVERHEAD 8u                /* sync x2, class, id, length x2, CK_A, CK_B */
#define UBX_MAX_PAYLOAD 64u            /* bytes stored (NAV-SOL 52, MON-VER sw + hw 40) */
#define UBX_LEN_LIMIT 256u             /* a longer length field is corruption (resync) */
#define UBX_FRAME_MAX (UBX_OVERHEAD + UBX_LEN_CFG_NAV5)   /* the longest frame the NAV sends: 44 */
#define UBX_SOL_FLAG_FIX_OK 0x01u
#define UBX_FIX_3D 3u
#define UBX_PRT_MODE_8N1 0x000008D0UL
#define UBX_PROTO_UBX 0x0001u
#define UBX_PROTO_NMEA 0x0002u
#define UBX_DYN_AIRBORNE_1G 6u
#define UBX_NAV5_MASK_DYN 0x0001u
#define UBX_BAUD_DEFAULT 9600UL        /* NEO-6 factory default (data sheet) */
#define UBX_MODULE_UNKNOWN 0u
#define UBX_MODULE_UBLOX6 6u
#define UBX_MODULE_UBLOX7 7u           /* NEO-7M, hwVersion "00070000" (2026-11-06; report only, as the others) */
#define UBX_MODULE_UBLOX8 8u
#define UBX_ITOW_LEAK_MS 1             /* offset leak per epoch (5 ms/s at 5 Hz) */
#define UBX_ITOW_JUMP_MS 1000          /* |D - O| above this re-learns the offset */
#define UBX_CFG_TRIES 3u               /* attempts per configuration frame */

/* ubx_msg_kind */
#define UBX_K_NONE 0u
#define UBX_K_POSLLH 1u
#define UBX_K_VELNED 2u
#define UBX_K_SOL 3u
#define UBX_K_ACK 4u
#define UBX_K_NAK 5u
#define UBX_K_MONVER 6u

typedef struct {
    uint16_t frames, bad_ck, bad_len, truncated, skipped;
} ubx_stats_t;

typedef struct {
    uint8_t  state;
    uint8_t  cls, id;                  /* of the frame in progress / just completed */
    uint8_t  ck_a, ck_b;
    uint16_t length;                   /* payload length field */
    uint16_t n;                        /* payload bytes seen */
    uint8_t  buf[UBX_MAX_PAYLOAD];     /* the stored payload (the first UBX_MAX_PAYLOAD bytes) */
    ubx_stats_t stats;
} ubx_parser_t;

typedef struct {
    uint32_t itow;
    int32_t  lon, lat, height, hmsl;
    uint32_t hacc, vacc;
} ubx_posllh_t;

typedef struct {
    uint32_t itow;
    int32_t  vel_n, vel_e, vel_d;
    uint32_t speed, gspeed;
    int32_t  heading;
    uint32_t sacc, cacc;
} ubx_velned_t;

typedef struct {
    uint32_t itow;
    int32_t  ftow;
    int16_t  week;
    uint8_t  gps_fix, flags;
    int32_t  ecef[3];
    uint32_t pacc;
    int32_t  ecef_v[3];
    uint32_t sacc;
    uint16_t pdop;
    uint8_t  res1, num_sv;
    uint32_t res2;
} ubx_sol_t;

typedef struct {                       /* one navigation epoch as the NAV holds it (sim/gps_nav.py GpsEpoch) */
    uint32_t itow;
    int32_t  lat, lon, hmsl;           /* 1e-7 deg, 1e-7 deg, mm */
    uint32_t hacc, vacc;               /* mm */
    int32_t  vel_n, vel_e, vel_d;      /* cm/s */
    uint32_t sacc;                     /* cm/s */
    uint8_t  gps_fix;
    bool     fix_ok;
    uint8_t  num_sv;
    uint32_t pacc;                     /* cm */
    uint32_t tick;                     /* NAV tick at which it completed */
} gps_epoch_t;

typedef struct {
    bool     has_itow;
    uint8_t  mask;                     /* parts of the open epoch: 1 POSLLH, 2 VELNED, 4 SOL */
    uint32_t itow;
    gps_epoch_t e;                     /* the open epoch's fields */
    uint16_t epochs, incomplete, duplicate;
} gps_asm_t;

typedef struct {
    uint8_t  ms_per_tick;
    uint16_t latency_ms;
    bool     has_offset;
    int32_t  offset;
    uint16_t relearned;
} gps_itow_t;

/* configurator states (sim/gps_nav.py CFGS_*) and the frames it sends */
#define GPS_CFGS_WAIT 0u
#define GPS_CFGS_PROBE 1u
#define GPS_CFGS_SWITCH 2u
#define GPS_CFGS_SETTLE 3u
#define GPS_CFGS_PRT 4u
#define GPS_CFGS_SEQ 5u
#define GPS_CFGS_VER 6u
#define GPS_CFGS_DONE 7u
#define GPS_TX_NONE 0u
#define GPS_TX_PRT 1u                  /* CFG-PRT UART1 at the configured baud */
#define GPS_TX_RATE 2u
#define GPS_TX_MSG_POSLLH 3u
#define GPS_TX_MSG_VELNED 4u
#define GPS_TX_MSG_SOL 5u
#define GPS_TX_NAV5 6u
#define GPS_TX_VER 7u                  /* MON-VER poll */
#define GPS_CFG_SEQ_N 5u               /* RATE, MSG POSLLH, MSG VELNED, MSG SOL, NAV5 */

typedef struct {
    uint8_t  state, tries, idx, acked;
    uint16_t t;                        /* ticks in the current state */
    uint16_t n_wait, n_ack, n_settle, n_ver;
    uint32_t baud;
    bool     prt_acked, cfg_ok, ver_ok;
    uint8_t  module;                   /* UBX_MODULE_* of the MON-VER answer */
    uint16_t naks, timeouts;
    /* the messages of the tick in progress (gps_cfg_msg before gps_cfg_tick) */
    uint8_t  ack;                      /* UBX_K_ACK / UBX_K_NAK of the last ACK / NAK for the awaited frame, else 0 */
    bool     ver_seen;
    uint8_t  ver_module;
} gps_cfg_t;

typedef struct {                       /* actions of one configurator tick, in this order */
    uint32_t baud;                     /* switch the NAV UART to this baud first (0 = no change) */
    uint8_t  tx;                       /* then send this frame (GPS_TX_*; gps_cfg_frame builds it) */
} gps_cfg_action_t;

/* ---- frames ---- */
void     ubx_checksum(const uint8_t *data, uint16_t n, uint8_t *ck_a, uint8_t *ck_b);
/* a complete frame into out (>= UBX_OVERHEAD + n bytes); returns its length */
uint16_t ubx_frame(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t n, uint8_t *out);
uint16_t ubx_cfg_prt_uart1(uint32_t baud, uint8_t out[UBX_OVERHEAD + UBX_LEN_CFG_PRT]);
uint16_t ubx_cfg_rate(uint16_t meas_ms, uint8_t out[UBX_OVERHEAD + UBX_LEN_CFG_RATE]);
uint16_t ubx_cfg_msg(uint8_t cls, uint8_t id, uint8_t rate, uint8_t out[UBX_OVERHEAD + UBX_LEN_CFG_MSG]);
uint16_t ubx_cfg_nav5_airborne(uint8_t out[UBX_OVERHEAD + UBX_LEN_CFG_NAV5]);
uint16_t ubx_mon_ver_poll(uint8_t out[UBX_OVERHEAD]);

/* ---- parser ---- */
void     ubx_parser_init(ubx_parser_t *p);
/* one byte; returns 1 when it completes a frame (p->cls, p->id, p->length, p->buf valid until the next call) */
uint8_t  ubx_parse_byte(ubx_parser_t *p, uint8_t b);
/* the kind of the frame just completed (sim/gps_nav.py decode_msg: a message whose length is not its type's is never
 * decoded; a truncated frame is decoded only as MON-VER with length >= 40) */
uint8_t  ubx_msg_kind(const ubx_parser_t *p);
void     ubx_get_posllh(const uint8_t *pl, ubx_posllh_t *m);
void     ubx_get_velned(const uint8_t *pl, ubx_velned_t *m);
void     ubx_get_sol(const uint8_t *pl, ubx_sol_t *m);
/* MON-VER hwVersion (payload bytes 30..39, NUL-terminated, ASCII whitespace stripped) -> UBX_MODULE_* */
uint8_t  ubx_monver_module(const uint8_t *pl);

/* ---- epoch assembly and time ---- */
void     gps_asm_init(gps_asm_t *a);
/* feed one decoded NAV message (kind UBX_K_POSLLH / VELNED / SOL, its payload) completed at NAV tick `tick`; returns the
 * epoch it completes (valid until the next call), else NULL */
const gps_epoch_t *gps_asm_add(gps_asm_t *a, uint8_t kind, const uint8_t *pl, uint32_t tick);
void     gps_itow_init(gps_itow_t *c, uint8_t ms_per_tick, uint16_t latency_ms);
/* learn from an epoch completed at NAV tick `tick`; returns its measurement time t_e on the NAV ms clock (uint32) */
uint32_t gps_itow_update(gps_itow_t *c, uint32_t itow_ms, uint32_t tick);

/* ---- boot configuration ---- */
/* n_* = sim/gps_nav.py GpsConfigurator's tick counts (params_gen.h GPS_CFG_N_*: max(1, round(CFG_*_S x loop_hz))) */
void     gps_cfg_init(gps_cfg_t *c, uint16_t n_wait, uint16_t n_ack, uint16_t n_settle, uint16_t n_ver, uint32_t baud);
/* a decoded message of this tick (UBX_K_ACK / UBX_K_NAK with the acknowledged class / id, or UBX_K_MONVER with the
 * module); others are ignored.  Call for every message parsed in the tick, before gps_cfg_tick. */
void     gps_cfg_msg(gps_cfg_t *c, uint8_t kind, uint8_t cls, uint8_t id, uint8_t module);
/* once per NAV tick, after the tick's messages: the configurator step and its actions */
void     gps_cfg_tick(gps_cfg_t *c, gps_cfg_action_t *act);
bool     gps_cfg_done(const gps_cfg_t *c);
/* the bytes of a GPS_TX_* frame (meas_ms = 1000 / rate_hz for CFG-RATE, baud for CFG-PRT); returns the length */
uint16_t gps_cfg_frame(uint8_t tx, uint32_t baud, uint16_t meas_ms, uint8_t out[UBX_FRAME_MAX]);

#endif /* FC_GPS_UBX_H */
