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
 * Formats 5 - 8 belonged to the removed VTOL program (2026-11-18): the rocket writes format 4.
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
#define LOG_FORMAT_VERSION 4           /* the rocket (v0.6 - v0.6.3 layouts) */
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

#endif /* FC_LOG_RECORD_H */
