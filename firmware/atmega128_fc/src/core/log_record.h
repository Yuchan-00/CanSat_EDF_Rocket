/* log_record.h - binary flight-log records the FCU writes to the OpenLog (portable packer).
 *
 * Frame:  0xA5 0x5A | type (u8) | len (u8) | payload[len] | fletcher16 (u16 LE)
 * The checksum covers type, len and payload.  All fields little-endian.
 * Text records (type 'T') carry ASCII; flight records (type 'F') carry
 * log_flight_t as fixed-point integers value * 2^LOG_SHIFT_* (truncated toward
 * zero, saturated, NaN -> 0); event records (type 'E') carry one onboard-supervisor
 * transition (tick, from-state, to-state, cause, reason); link records (type 'L')
 * one uplink CMD the COM forwarded (IL_CMD) and what the supervisor did with it (tick,
 * SEQ, cmd, arg, nonce, result code, state after, RSSI/SNR when the radio reports them);
 * COM records (type 'C', v0.6) one IL_STAT of the COM chip as received (FCU tick + the
 * 36-byte payload of core/ilink_proto.h, byte for byte); status records (type 'S', 1 Hz)
 * carry the FCU's health counters, the COM's link counters from the newest IL_STAT, the
 * FCU's own ilink counters and the stack high-water mark; NAV-link records (type 'N', 1 Hz,
 * v0.6.3) the FCU's counters of the navigation frames it received from the NAV chip
 * (core/nlink_proto.h) and the NAV's own loop time / flags / reset cause.
 * firmware/atmega128_fc/tools/decode_log.py turns a log file into CSV.
 *
 * Format 4 (v0.6): 'S' grew from 28 to 40 bytes (ilink counters, COM status age / flags;
 * its link fields now come from the COM), 'C' is new.  Format 3 = v0.5 single chip.
 * v0.6.3 keeps format 4 byte for byte and only ADDS the 'N' record (a format-4 decoder that
 * does not know it skips it): the sensors and the estimator moved to the NAV, so the 'F'
 * record's q / accel / h_est / vz_est / h_baro / tilt and the 'S' record's twi_errors /
 * baro_fails / imu_fails are the NAV's values from the navigation frame, 'F' overruns counts
 * the FCU ticks run without a valid navigation frame (frame timeout or a frame that failed its
 * check), and loop_us_* is the FCU's time-critical part from the frame to the end of its tick.
 *
 * Format 5 (v0.7, the VTOL build only; the ballistic build keeps writing format 4 byte for byte): the boot record
 * says 5; 'E' grows from 8 to 10 bytes (+ phase_from, phase_to: a VTOL phase change inside POWERED is an 'E' record
 * with from == to == POWERED, core/onboard.h onb_trans_t), 'L' from 14 to 15 bytes (+ phase_after), and the 'V'
 * record (vertical controller, every LOG_VTOL_DIV flight records = 10 Hz at 250 Hz / FC_LOG_DIV 5) is new: phase,
 * the supervisor's flags of the vertical mission, the controller's mode / reference / vertical-speed command /
 * throttle / feed-forward / integrator / touchdown counter and the cumulative motor time.  Every other record keeps
 * its format-4 layout.  The 'V' rate is set by the log port's budget (main.c checks it at compile time): 'F' alone
 * fills 83 % of the 57600-baud port.
 *
 * Format 6 (v0.7.1, the VTOL build; format 5 was never flown, decode_log.py still reads it): only the 'V' record grows,
 * from 26 to 35 bytes, appending the touchdown detector v2 and the NAV estimator's correction (params.json
 * mission.vtol.touchdown / nav_estimator): the signature bits of the tick (VC_SIG_*: low / still / support / floor /
 * ground), which signature confirmed the touchdown (VC_TD_*), the V10 LAND-floor counter, the still signature's
 * filtered v_z and the support signature's filtered excess specific force (the controller's own filter states), and
 * how many of the navigation frames since the previous 'V' record carried the NAV's estimator-correction flag
 * (nav/nav.h NAV_F_EST_GATE).  The estimator's gyro / accelerometer bias estimates stay on the NAV (no room in the
 * nlink frame).  The V8 / V9 / V10 interlocks (reason codes 21 / 22 / 23) need no new field: they are 'E' records.
 *
 * Invariant: the layout here and in decode_log.py change together
 * (LOG_FORMAT_VERSION is written in the boot record); a record never exceeds
 * LOG_MAX_PAYLOAD so a frame always fits the UART ring buffer.
 */
#ifndef FC_LOG_RECORD_H
#define FC_LOG_RECORD_H

#include <stdbool.h>
#include <stdint.h>

#define LOG_SYNC0 0xA5
#define LOG_SYNC1 0x5A
#define LOG_TYPE_FLIGHT 'F'
#define LOG_TYPE_EVENT 'E'
#define LOG_TYPE_TEXT 'T'
#define LOG_TYPE_BOOT 'B'
#define LOG_TYPE_STATUS 'S'
#define LOG_TYPE_LINK 'L'
#define LOG_TYPE_COM 'C'
#define LOG_FORMAT_VERSION 4           /* the ballistic build (v0.6 - v0.6.3 layouts) */
#define LOG_FORMAT_VERSION_VTOL 6      /* the VTOL build (v0.7.1; v0.7: 5): 'E' 10 B, 'L' 15 B, 'V' 35 B */
#define LOG_TYPE_VTOL 'V'
#define LOG_MAX_PAYLOAD 96            /* >= the flight record (90) and the longest text (boot ~91) */
#define LOG_FRAME_OVERHEAD 6

/* fixed point: stored integer = trunc(value * 2^shift), saturated to the field type.
 * Power-of-two scales are an exponent add on the float bit pattern (no float multiply,
 * no rounding call): ~90 CPU cycles per field instead of ~450 on the AVR. */
#define LOG_SHIFT_Q        15         /* quaternion, int16, 3.1e-5            */
#define LOG_SHIFT_RATE     10         /* rad/s, int16, +-32 rad/s, 0.98 mrad/s */
#define LOG_SHIFT_ACCEL    7          /* m/s^2, int16, +-256, 7.8 mm/s^2      */
#define LOG_SHIFT_ANGLE    13         /* rad, int16, +-4 rad, 1.2e-4 rad      */
#define LOG_SHIFT_MOMENT   12         /* N m, int16, +-8 N m, 2.4e-4 N m      */
#define LOG_SHIFT_H        10         /* m, int32, 0.98 mm                    */
#define LOG_SHIFT_VZ       7          /* m/s, int16, +-256 m/s, 7.8 mm/s      */
#define LOG_SHIFT_THROTTLE 15         /* 0..1, uint16, 3.1e-5                 */
#define LOG_SHIFT_VCELL    12         /* V, uint16, 0..16 V, 0.24 mV          */
#define LOG_SHIFT_TILT     7          /* deg, int16, +-256 deg, 0.0078 deg    */
#define LOG_SHIFT_THRUST   8          /* N, uint16, 0..256 N, 3.9 mN          */

/* flags byte */
#define LOG_F_CONTROL_ACTIVE 0x01
#define LOG_F_SATURATED      0x02
#define LOG_F_RELEASE        0x04
#define LOG_F_LINK_OK        0x08      /* a valid ground frame within link_loss_disarm_s */
#define LOG_F_INTERLOCK      0x10      /* an interlock has acted since boot */
#define LOG_F_ARM_SWITCH     0x20
#define LOG_F_SENSORS_OK     0x40
#define LOG_F_BENCH          0x80

typedef struct {
    uint32_t tick;
    uint8_t  state, flags, reason, aux_bits;   /* reason: supervisor reason code; aux_bits: LOG_A_* */
    uint16_t loop_us_last, loop_us_max, overruns;
    float    q[4];
    float    gyro_f[3];        /* filtered body rate, rad/s */
    float    accel[3];         /* specific force, body, m/s^2 */
    float    att_err[3];       /* rad */
    float    moments[3];       /* N m */
    float    integ[3];         /* N m */
    float    delta[4];         /* rad */
    uint16_t pwm_counts[6];    /* timer counts actually written (vane1..4, esc, release) */
    float    h_est, vz_est, h_baro;
    float    throttle, v_cell, tilt_deg, thrust_est;
} log_flight_t;

uint16_t log_fletcher16(const uint8_t *data, uint16_t len);
/* trunc(x * 2^shift) saturated to [lo, hi]; NaN -> 0 */
int32_t  log_fixed(float x, int8_t shift, int32_t lo, int32_t hi);
/* frames payload into out (needs len + LOG_FRAME_OVERHEAD bytes); returns frame length */
uint16_t log_frame(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t *out);
/* packs a flight record payload; returns its length (= LOG_FLIGHT_LEN) */
uint8_t  log_pack_flight(const log_flight_t *r, uint8_t *payload);
/* the same in LOG_FLIGHT_PARTS pieces at fixed offsets, so the 250 Hz loop can spread the
 * float->int conversions over the ticks between two records; returns the end offset */
#define LOG_FLIGHT_LEN 90
#define LOG_FLIGHT_PARTS 4
uint8_t  log_flight_part_start(uint8_t part);
uint8_t  log_pack_flight_part(const log_flight_t *r, uint8_t *payload, uint8_t part);
/* v0.7 (the VTOL build): the same record in LOG_FLIGHT_PARTS_FINE shorter pieces (<= 7 fixed-point fields each, about
 * 0.12 ms on the AVR instead of up to 0.22 ms), so a part fits the shorter background window the VTOL time-critical part
 * leaves; same bytes, same offsets within the record */
#define LOG_FLIGHT_PARTS_FINE 6
uint8_t  log_flight_part_fine_start(uint8_t part);
uint8_t  log_pack_flight_part_fine(const log_flight_t *r, uint8_t *payload, uint8_t part);
/* running Fletcher-16 (s1, s2 start at 0); log_fletcher16() == the same over one buffer */
void     log_fletcher16_acc(uint16_t *s1, uint16_t *s2, const uint8_t *data, uint16_t len);
/* aux_bits of the flight record */
#define LOG_A_ALIGNED        0x01      /* estimator window full, ground reference, vehicle still */
#define LOG_A_RECOVERY       0x02      /* I10 boot (reset in flight) */
#define LOG_A_REPLY_PENDING  0x04      /* a reply snapshot (IL_SNAP) is waiting for the ilink or still shifting out */
#define LOG_A_FRAME_RX       0x08      /* an uplink frame was handled in this record's tick */
#define LOG_A_ACK_REJECTED   0x10      /* the last processed command was rejected */
#define LOG_A_RADIO_OK       0x20      /* an IL_STAT with IL_STAT_RADIO_OK arrived within the last 3 s (v0.6.3: the COM's
                                        * STATs are up to ~2.5 s apart when the uplink stops, ilink_proto.h) */

typedef struct {
    uint32_t tick;
    uint16_t stack_free, twi_errors, uart_dropped;
    /* COM, from the newest IL_STAT (0 / LOG_RSSI_NONE before the first) */
    uint16_t link_frames;      /* valid uplink CMD frames for this vehicle */
    uint16_t link_bad;         /* bad CRC + bad LEN candidates */
    uint16_t link_other;       /* valid CRC but other vehicle / TYPE / length, decoder overflow */
    uint16_t radio_errors;     /* radio RX errors + RX ring overflow + TX timeouts + replies dropped */
    uint16_t replies;          /* TELEM frames handed to the radio */
    uint16_t baro_fails, imu_fails, vbat_mv;
    uint8_t  reset_cause;      /* FCU */
    int8_t   rssi_dbm;         /* COM: last packet RSSI, LOG_RSSI_NONE when the radio does not report it */
    /* FCU side of the ilink (v0.6) */
    uint16_t il_cmds;          /* IL_CMD frames handed to the supervisor */
    uint16_t il_snaps;         /* IL_SNAP frames queued to the COM */
    uint16_t il_bad;           /* decoder rejects: bad CRC + bad LEN + bad TYPE */
    uint16_t il_uart_errors;   /* USART framing / overrun / parity + RX ring overflow (v0.6 phase 3: the zero-copy decoder
                                * has no output overflow; a babbling COM shows here) */
    uint16_t il_dropped;       /* SNAPs replaced before they left + IL_CMD beyond ONB_MAX_FRAMES a tick + unexpected TYPE */
    uint8_t  com_age_s;        /* seconds since the newest IL_STAT (254 = 254 s or more), LOG_COM_AGE_NONE before the first */
    uint8_t  com_flags;        /* IL_STAT_* flags of the newest IL_STAT | LOG_COM_TAG_MISMATCH */
} log_status_t;
#define LOG_RSSI_NONE 127
#define LOG_COM_AGE_NONE 255u
#define LOG_COM_TAG_MISMATCH 0x80u     /* the COM's params tag differs from the FCU's FC_PARAMS_TAG */
#define LOG_STATUS_LEN 40

uint8_t  log_pack_status(const log_status_t *s, uint8_t *payload);
/* one IL_STAT as received: FCU tick u32, then the LOG_COM_STAT_LEN payload bytes unchanged */
#define LOG_COM_STAT_LEN 36            /* == IL_STAT_LEN (core/ilink_proto.h; the host test checks it) */
#define LOG_COM_LEN (4 + LOG_COM_STAT_LEN)
uint8_t  log_pack_com(uint32_t tick, const uint8_t stat_payload[LOG_COM_STAT_LEN], uint8_t *payload);
/* NAV link (v0.6.3), 1 Hz: what the FCU received from the NAV over the nlink (counters cumulative since the FCU's boot)
 * and the NAV's own figures from its newest valid frame */
typedef struct {
    uint32_t tick;
    uint16_t frames;           /* valid navigation frames (nl_decode_nav accepted them) */
    uint16_t bad;              /* complete exchanges that failed nl_decode_nav (sync, TYPE, CRC) */
    uint16_t timeouts;         /* FCU ticks run on the frame timeout / Timer2 (no exchange arrived) */
    uint16_t seq_lost;         /* NAV ticks without a valid frame at the FCU (SEQ gaps between valid frames) */
    uint16_t resyncs;          /* partial exchanges the SPI slave discarded (hal/nlink_spi.h) */
    uint16_t spi_overruns;     /* complete exchanges replaced before the FCU took them */
    uint16_t nav_loop_us_max;  /* the NAV's time-critical maximum over the record's window, us (0: no frame) */
    uint8_t  nav_flags;        /* NL_F_* of the newest valid frame (0 before the first) */
    uint8_t  nav_reset_cause;  /* the newest valid frame's reset_cause (0 before the first): bits 4..0 the NAV's MCUCSR
                                * at its boot, bits 7..5 NAV_RC_IMU_INIT_FAIL 0x80 / NAV_RC_BARO_INIT_FAIL 0x40 /
                                * NAV_RC_TAG_MISMATCH 0x20 (nlink_proto.h) */
} log_nav_t;
#define LOG_TYPE_NAV 'N'
#define LOG_NAV_LEN 20
uint8_t  log_pack_nav(const log_nav_t *n, uint8_t *payload);
/* one supervisor transition: cause = 0x80|cmd, an interlock reason code or 0x7E (release timer) */
#define LOG_EVENT_LEN 8
uint8_t  log_pack_event(uint32_t tick, uint8_t from, uint8_t to, uint8_t cause, uint8_t reason, uint8_t *payload);
/* one received uplink frame: code 0 executed, 0x40.. rejected, 0xFE NONE adopted, 0xFF repeat (reply only) */
#define LOG_LINK_LEN 14
uint8_t  log_pack_link(uint32_t tick, uint8_t seq, uint8_t cmd, uint16_t arg, uint16_t nonce, uint8_t code,
                       uint8_t state_after, int8_t rssi_dbm, int8_t snr_q4, uint8_t *payload);

/* ---- format 5 (VTOL build) ---- */
/* 'E': the format-4 fields + the VTOL phase before / after (LP_PHASE_*); cause also 0x7D spool done, 0x7C at target,
 * 0x7B touchdown, 0x7A LANDED timer (core/onboard.h ONB_CAUSE_*) */
#define LOG_EVENT2_LEN 10
uint8_t  log_pack_event2(uint32_t tick, uint8_t from, uint8_t to, uint8_t cause, uint8_t reason, uint8_t phase_from,
                         uint8_t phase_to, uint8_t *payload);
/* 'L': the format-4 fields + the phase after the command */
#define LOG_LINK2_LEN 15
uint8_t  log_pack_link2(uint32_t tick, uint8_t seq, uint8_t cmd, uint16_t arg, uint16_t nonce, uint8_t code,
                        uint8_t state_after, int8_t rssi_dbm, int8_t snr_q4, uint8_t phase_after, uint8_t *payload);
/* 'V' vertical-controller record (fixed point as 'F': value * 2^shift, truncated, saturated, NaN -> 0) */
#define LOG_V_MOTOR_ON   0x01u         /* supervisor: state POWERED, the ESC follows the controller */
#define LOG_V_BUZZER     0x02u         /* supervisor: spool warning (ARMED) */
#define LOG_V_BARO_LOST  0x04u         /* supervisor: baro lost in flight, landing on the inertial vertical speed */
#define LOG_V_AT_TARGET  0x08u         /* controller: h_ref == target this tick */
#define LOG_V_TOUCHDOWN  0x10u         /* controller: touchdown confirmed (LANDED next tick) */
#define LOG_V_SAT_HIGH   0x20u         /* controller: u at its ceiling (u_max or the take-off ramp) */
#define LOG_V_SAT_LOW    0x40u         /* controller: u at u_min */
typedef struct {
    uint32_t tick;
    uint8_t  phase;                    /* LP_PHASE_* */
    uint8_t  vflags;                   /* LOG_V_* */
    uint8_t  reason;                   /* supervisor reason code (the interlock that acted, or the last rejection) */
    uint8_t  vc_mode;                  /* VC_MODE_* (core/vertical_ctrl.h) */
    int16_t  h_tgt_dm;                 /* the supervisor's altitude target [0.1 m] (0 outside SPOOL / CLIMB / HOLD) */
    float    h_ref;                    /* m,   int32 << LOG_SHIFT_H */
    float    vz_cmd;                   /* m/s, int16 << LOG_SHIFT_VZ */
    float    u;                        /* 0..1, uint16 << LOG_SHIFT_THROTTLE: the controller's throttle */
    float    u_ff;                     /* 0..1, uint16 << LOG_SHIFT_THROTTLE: hover feed-forward / cos(tilt) */
    float    integ;                    /* throttle, int16 << LOG_SHIFT_INTEG */
    uint16_t td_count;                 /* touchdown-detector count, ticks */
    uint16_t motor_ticks;              /* cumulative POWERED ticks since boot (V2 / V6), saturated at 65535 */
    /* ---- format 6 (v0.7.1) ---- */
    uint8_t  sig;                      /* the controller's signature bits of this tick (vertical_ctrl.h VC_SIG_*) */
    uint8_t  td_by;                    /* the signature that confirmed the touchdown (VC_TD_*; 0 none) */
    uint16_t low_in_land;              /* V10: consecutive LAND ticks low, leading, baro healthy (vc_outputs_t) */
    float    vz_lpf;                   /* m/s, int16 << LOG_SHIFT_VZ: the still signature's filtered v_z (vc_t.v_lpf) */
    float    f_excess;                 /* m/s^2, int16 << LOG_SHIFT_ACCEL: the support signature's filtered excess body-z
                                        * specific force over the commanded thrust's (vc_t.d_lpf) */
    uint8_t  est_gate;                 /* navigation frames with nav.h NAV_F_EST_GATE since the previous 'V' record */
} log_vtol_t;
#define LOG_SHIFT_INTEG 15             /* throttle integrator, int16, +-1, 3.1e-5 */
#define LOG_VTOL_LEN 35                /* format 6 (format 5: 26, without the fields from sig on) */
#define LOG_VTOL_LEN_V5 26
uint8_t  log_pack_vtol(const log_vtol_t *v, uint8_t *payload);
/* the same, packed IN PLACE: the payload overwrites the struct from its first byte (every field's payload offset is <=
 * its struct offset, checked at compile time in log_record.c); returns LOG_VTOL_LEN, the payload at (uint8_t *)v.  main.c
 * packs the captured record in one background step and frames it in the next (each under REC_RESERVE_US) without a
 * second buffer (v0.7.1: the format-6 record, pack + frame in one step, took longer than that reserve) */
uint8_t  log_pack_vtol_in_place(log_vtol_t *v);

/* ---- format 7 (v0.8, the VTOL build with GPS position hold; a VTOL build without it - -D ONB_NO_GPS - keeps writing format
 * 6): every format-6 record unchanged, plus the 'G' record (GPS / position hold), every LOG_GPS_DIV-th 'V' slot (2.5 Hz at
 * 250 Hz / FC_LOG_DIV 5 / LOG_VTOL_DIV 5 / 4 - the log port's budget, main.c checks it): the newest complete GPS slot set
 * the FCU received from the NAV (core/nlink_proto.h nl_gps_set_t: flags, satellites, accuracy, the NAV's reset counter,
 * position and velocity in the NAV frame W - x north, y west - and the heading), the position controller's published
 * mode / LAND requests / distance to the hold point (core/position_ctrl.h pos_pub_t), the hold point itself in the NAV
 * frame (pos_t.hold_rf + the set's cumulative reset offset), the tilt setpoint the next attitude step flies, the slot
 * receiver's counters and the ticks whose position work had to be finished at the next tick's start.  All integers, no
 * scaling beyond the wire's: cm, cm/s, 0.1 m, mrad (round half away from zero, saturated), Q15. */
#define LOG_TYPE_GPS 'G'
#define LOG_FORMAT_VERSION_GPS 7       /* the VTOL build with GPS position hold: format 6 + 'G' */
#define LOG_G_ENGAGED  0x01u           /* the position controller is engaged (pos_pub_t.engaged) */
#define LOG_G_HOLD     0x02u           /* a hold point exists */
#define LOG_G_FRESH    0x04u           /* the newest slot set is fresh (nl_slot_rx_fresh) */
#define LOG_G_GATE_OK  0x08u           /* the ON gate's GPS part (nl_slot_rx_gate_ok) */
#define LOG_G_NO_SET   0x10u           /* no complete slot set since boot (the set fields are 0) */
#define LOG_HOLD_NONE  0x7FFF          /* hold_cm: no hold point */
#define LOG_DIST_NONE  0xFFFFu         /* dist_dm: no hold point */
typedef struct {
    uint32_t tick;
    uint8_t  gps_flags;                /* GPS_F_* of the newest complete set (core/gps_nav.h) */
    uint8_t  num_sv;
    uint8_t  hacc_dm;                  /* 0.1 m, 255 = none / >= 25.5 m */
    uint8_t  reset_count;              /* the NAV's position-reset counter (wraps) */
    uint8_t  pos_mode;                 /* POS_MODE_* published (core/position_ctrl.h) */
    uint8_t  requests;                 /* POS_REQ_* published (latched until the motor stops: V11 - V14) */
    uint8_t  gflags;                   /* LOG_G_* */
    uint8_t  set_age;                  /* FCU ticks since the newest complete set (255 = 255 or more / none) */
    int16_t  p_cm[2];                  /* NAV position x / y (W), cm */
    int16_t  v_cms[2];                 /* NAV velocity x / y (W), cm/s */
    int16_t  hold_cm[2];               /* hold point x / y (W, the NAV frame), cm; LOG_HOLD_NONE without one */
    uint16_t dist_dm;                  /* |hold - p| of the published step, 0.1 m; LOG_DIST_NONE without a hold point */
    int16_t  tilt_mrad[2];             /* the tilt setpoint the next attitude step uses (level frame rotation vector), mrad */
    int16_t  head_q15[2];              /* the NAV heading cos / sin of the newest set, Q15 */
    uint16_t sets, broken;             /* slot receiver: complete sets / broken cycles (wrap) */
    uint16_t bg_late;                  /* ticks whose position work was finished at the next tick start (saturating) */
} log_gps_t;
#define LOG_GPS_LEN 40
uint8_t  log_pack_gps(const log_gps_t *g, uint8_t *payload);
uint8_t  log_pack_gps_in_place(log_gps_t *g);  /* as log_pack_vtol_in_place: the payload at (uint8_t *)g */

/* ---- format 8 (v0.8.1, the four-chip VTOL build with GPS position hold - the FCU's GNS chain, core/onboard.h
 * ONB_CHAIN_GNS): every format-6 record unchanged, plus the 'G' record in its GNS form, every LOG_GPS_DIV-th 'V' slot as
 * format 7: the newest complete GNS slot set the FCU received (core/nlink_proto.h nl_gns_data_t: the GNS's flags,
 * satellites, accuracy and its STATUS byte pc = position-controller mode | NL_PC_HOLD | its latched LAND requests << 4;
 * POS relative to the hold point when NL_PC_HOLD - the GNS keeps the hold point -, VEL, the wind TRIM), the newest TILT
 * part, the setpoint the next attitude step flies, the receiver's counters and the ticks whose builder work was finished
 * at the next tick start.  The v0.8 fields the GNS slot no longer carries (the NAV's reset counter, the hold point in the
 * NAV frame, the heading cos / sin) are gone (the slot has five 4-byte parts: TILT, STATUS, POS, VEL, TRIM).  All wire
 * integers, no rescaling: cm, cm/s, 0.1 m, x 1e4 rad (TILT / TRIM parts), mrad (the setpoint). */
#define LOG_FORMAT_VERSION_GNS 8       /* the four-chip VTOL build with GPS position hold: format 6 + 'G' (GNS form) */
#define LOG_G4_ENGAGED  0x01u          /* the newest TILT part is not NL_TILT_NONE (the GNS's position loop is engaged) */
#define LOG_G4_HOLD     0x02u          /* the newest set's NL_PC_HOLD (POS is relative to the hold point) */
#define LOG_G4_FRESH    0x04u          /* the newest set is fresh (nl_gns_rx_fresh) */
#define LOG_G4_GATE_OK  0x08u          /* the ON gate's GPS part (nl_gns_rx_gate_ok) */
#define LOG_G4_NO_SET   0x10u          /* no complete set since boot (the set fields are 0, hacc LP_HACC_NONE) */
#define LOG_G4_FALLBACK 0x20u          /* no TILT part for >= 2 cycles: the builder flies the newest set's TRIM */
#define LOG_G4_OWN_LOST 0x40u          /* the FCU's own GPS_LOST request (no TILT part for land_steps cycles -> V11) */
#define LOG_G4_NO_TILT  0x80u          /* no TILT part since boot */
typedef struct {                       /* in payload order (log_pack_gns_in_place), 40 bytes, no padding on the AVR */
    uint32_t tick;
    uint8_t  gps_flags, num_sv, hacc_dm;
    uint8_t  pc;                       /* the GNS's position mode | NL_PC_HOLD | its requests << 4 */
    uint8_t  gflags;                   /* LOG_G4_* */
    uint8_t  set_age;                  /* FCU ticks since the newest complete set (255 = 255 or more / none) */
    int16_t  pos_cm[2];                /* POS part (W, cm): p - hold with NL_PC_HOLD, else p */
    int16_t  vel_cms[2];               /* VEL part (W, cm/s) */
    int16_t  trim[2];                  /* TRIM part, x 1e4 rad (level frame) */
    int16_t  tilt_tgt[2];              /* the newest TILT part, x 1e4 rad ([0] = NL_TILT_NONE: not engaged) */
    int16_t  tilt_mrad[2];             /* the tilt setpoint the next attitude step uses, mrad */
    uint16_t dist_dm;                  /* |POS| with NL_PC_HOLD (the TELEM v3 distance), 0.1 m; LOG_DIST_NONE without */
    uint16_t sets, broken, tilts;      /* slot receiver: complete sets / broken cycles / TILT parts (wrap) */
    uint16_t bg_late;                  /* ticks whose builder work was finished at the next tick start (saturating) */
} log_gns_t;
#define LOG_GNS_LEN 40
uint8_t  log_pack_gns(const log_gns_t *g, uint8_t *payload);
uint8_t  log_pack_gns_in_place(log_gns_t *g);  /* the payload at (uint8_t *)g (every field's payload offset == its struct offset) */

#endif /* FC_LOG_RECORD_H */
