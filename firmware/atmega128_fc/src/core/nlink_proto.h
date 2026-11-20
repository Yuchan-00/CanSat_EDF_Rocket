/* nlink_proto.h - the NAV <-> FCU navigation link (SPEC 10.5, v0.6.3), portable C99.
 *
 * v0.6.3 runs the flight computer on three ATmega128A (params.json flight_computer.architecture):
 *
 *   NAV  MPU-6050 + MS5611 on I2C, pad-alignment / gyro-only attitude estimator, tilt, altitude
 *        filter, sensor health.  No flight decisions.
 *   FCU  onboard supervisor (commands + interlocks I1-I10), attitude controller, PWM outputs,
 *        OpenLog log, COM link (ilink, ilink_proto.h).  Every flight decision is made here.
 *   COM  LoRa radio.
 *
 * Why: the v0.6 FCU's time-critical part (sensors + estimator + supervisor + controller + outputs,
 * one serial chain per 4 ms tick) measured 78.8 % of the tick with its interrupts and ~83-86 % in
 * the worst composition (sensor I2C interrupts, log UART, ilink).  Moving the sensors and the
 * estimator to the NAV takes ~1.4 ms and every I2C interrupt off the FCU.
 *
 * Wire: SPI, NAV = master (PB0 = slave select output, PB1 SCK, PB2 MOSI, PB3 MISO input),
 * FCU = slave (PB0 SS, PB1 SCK, PB2 MOSI in, PB3 MISO out); mode 0, MSB first,
 * NLINK_SPI_HZ (params flight_computer.nlink.spi_clock_hz = 2 MHz), NLINK_BYTE_GAP_US between
 * bytes so the FCU's slave interrupt can load its next return byte.  Every NAV tick, as soon as the
 * navigation state of that tick is computed, the NAV pulls SS low, exchanges exactly NL_XFER_BYTES
 * bytes and releases SS.  The FCU's tick starts when the last byte of a frame has arrived
 * (frame-triggered loop); if none arrives within NLINK_FRAME_TIMEOUT_US of the previous tick start
 * the FCU runs the tick on its own Timer2 with the navigation data invalid.
 *
 * NAV -> FCU (MOSI), NL_XFER_BYTES = 72:
 *   0xD5 0x5D | NL_TYPE_NAV | SEQ | PAYLOAD[NL_NAV_LEN = 66] | CRC16 lo | CRC16 hi
 *   SEQ = the NAV's tick counter, low byte.  CRC16-CCITT-FALSE (the lp / il CRC) over TYPE, SEQ and
 *   PAYLOAD.  Payload (little-endian, float = IEEE-754 binary32), see nl_nav_t for the order:
 *   q[4] f32, rate[3] f32, accel[3] f32, h_m f32, vz_mps f32, h_baro_rel_m f32, tilt_deg f32,
 *   imu_fail u16, baro_fails u16, twi_errors u16, loop_us_max u16, flags u8, reset_cause u8.
 *   reset_cause: bits 4..0 = the NAV's MCUCSR at boot (PORF 0x01, EXTRF 0x02, BORF 0x04, WDRF 0x08,
 *   JTRF 0x10 - the RESET_* of hal/watchdog.h; MCUCSR has no other reset flag), bits 7..5 = the
 *   NAV's boot / link results (nav/nav.h, sim/nlink_proto.py NAV_RC_*):
 *     0x80 NAV_RC_IMU_INIT_FAIL   MPU-6050 WHO_AM_I not recognised after 3 tries at boot
 *     0x40 NAV_RC_BARO_INIT_FAIL  MS5611 reset / PROM read / PROM CRC / first D2 failed at boot
 *     0x20 NAV_RC_TAG_MISMATCH    the last valid FCU frame carried a params_tag other than the NAV's
 *                                 own FC_PARAMS_TAG (the two chips were built from different
 *                                 params_gen.h); clear before the first valid FCU frame
 *   Bits 7 / 6 are fixed from boot to the next reset, bit 5 follows the FCU frames.  They only
 *   report (the FCU logs them in its 'N' record); no flight decision reads them.
 *
 * FCU -> NAV (MISO), the same 72-byte exchange; only the first NL_FCU_FRAME_BYTES = 10 matter,
 * the rest are 0x00:
 *   0xD5 0x5D | NL_TYPE_FCU | SEQ | state u8 | fflags u8 | params_tag u8 | thr_q u8 | CRC16 lo | hi
 *   thr_q: the v0.6.3 reserved byte - the rocket's FCU writes 0 and its NAV ignores it (the removed VTOL program sent its
 *   ESC throttle in it; the byte stays so the exchange and the flight images are unchanged).
 *   SEQ = the SEQ of the last NAV frame the FCU accepted; state = the supervisor state after the
 *   FCU's latest tick; fflags = NL_FF_*.  Because the bytes go out while the FCU is still waiting for
 *   the frame of this tick, the NAV acts on a state that is one FCU tick old, i.e. the NAV's pad /
 *   flight switch lags the supervisor by two control ticks (v0.5: one).  At LAUNCH the throttle ramps
 *   from 0 over 0.5 s, so the vehicle has not moved when the estimator leaves pad alignment.
 *
 * Semantics on the FCU (SPEC 10.5, the supervisor itself is unchanged):
 *   - imu_ok handed to the supervisor = a valid NAV frame started this tick && NL_F_IMU_OK &&
 *     (NL_F_EST_VALID || !onboard_flying(state)).  A NAV that reset in flight restarts without a
 *     valid attitude (EST_VALID needs a completed pad alignment) -> imu_ok false -> I9 -> ABORT.
 *   - baro_ok = a valid NAV frame && NL_F_BARO_OK; aligned = NL_F_ALIGNED; gyro_saturated =
 *     NL_F_GYRO_SAT; h, v_z, tilt, rates from the frame; the controller uses q and rate.
 *   - no valid frame (timeout, CRC, length, sync) -> imu_ok = baro_ok = false for that tick: a dead
 *     NAV or a cut wire is an IMU + baro failure, I9 after ONB_IMU_FAIL_TICKS, exactly like a dead
 *     MPU-6050 on the v0.5 single chip.
 * Semantics on the NAV (two switches, both from the state of the last valid FCU frame; before the
 * first valid FCU frame after a NAV boot the NAV assumes SAFE, so EST_VALID can only be earned on
 * the pad):
 *   - altitude filter: held at the pad (h, v_z reset to 0 every tick before that tick's baro
 *     correction) and the ground reference low-pass filtered while that state is SAFE or ARMED,
 *     whatever the fflags; predict + baro correct against the frozen ground reference otherwise
 *     (sim/nlink_proto.py nav_alt_pad_mode).
 *   - attitude estimator: pad alignment (and the pad-still counter behind ALIGNED / EST_VALID) while
 *     that state is SAFE or ARMED and NL_FF_BENCH is clear; gyro-only otherwise.  NL_FF_BENCH (the
 *     FCU's bench_vanes build) switches only the estimator to gyro-only on the pad - the altitude stays
 *     held (sim/nlink_proto.py nav_pad_mode).
 *
 * TYPE 0x03 .. 0x05 (the removed VTOL program's nlink v3 / v4) are not used: both decoders refuse them.
 *
 * Invariant: the bytes of every frame are fully determined by the packed struct (no padding, no
 * uninitialised field), so the Python reference sim/nlink_proto.py produces the same bytes.
 */
#ifndef FC_NLINK_PROTO_H
#define FC_NLINK_PROTO_H

#include "fcmath.h"
#include "link_proto.h"

#define NL_SYNC0 0xD5u
#define NL_SYNC1 0x5Du
#define NL_TYPE_NAV 0x01u
#define NL_TYPE_FCU 0x02u
#define NL_NAV_LEN 66u
#define NL_HEADER_BYTES 4u                                      /* sync x2, TYPE, SEQ */
#define NL_XFER_BYTES (NL_HEADER_BYTES + NL_NAV_LEN + 2u)        /* 72 */
#define NL_FCU_LEN 4u                                           /* state, fflags, params_tag, thr_q (reserved, 0) */
#define NL_FCU_FRAME_BYTES (NL_HEADER_BYTES + NL_FCU_LEN + 2u)   /* 10 */

/* nl_nav_t.flags */
#define NL_F_IMU_OK 0x01u        /* this tick's MPU-6050 burst read succeeded */
#define NL_F_BARO_OK 0x02u       /* MS5611 samples on schedule (v0.5 baro_ok rule) */
#define NL_F_ALIGNED 0x04u       /* pad alignment complete (v0.5 'aligned', frozen off the pad) */
#define NL_F_GYRO_SAT 0x08u      /* MPU-6050 gyro at its int16 limit this tick */
#define NL_F_EST_VALID 0x10u     /* the attitude came from a pad alignment completed since this NAV boot */
#define NL_F_BARO_NEW 0x20u      /* a new baro sample was used by the altitude filter this tick */
#define NL_F_BOOT 0x40u          /* set until the NAV has received a valid FCU frame since its reset */

/* nl_fcu_t.fflags */
#define NL_FF_BENCH 0x01u        /* bench_vanes mode: the estimator stays gyro-only on the pad (altitude still held) */
#define NL_FF_RECOVERY 0x02u     /* the FCU booted by I10 (in-flight reset) */

typedef struct {
    uint8_t  seq;
    float    q[4];               /* w, x, y, z */
    float    rate[3];            /* bias-corrected body rates, rad/s */
    float    accel[3];           /* body specific force, m/s^2 */
    float    h_m, vz_mps;        /* altitude filter (0 on the pad) */
    float    h_baro_rel_m;       /* baro altitude above the ground reference */
    float    tilt_deg;
    uint16_t imu_fail;           /* consecutive failed IMU reads */
    uint16_t baro_fails;         /* MS5611 failures since boot */
    uint16_t twi_errors;
    uint16_t loop_us_max;        /* NAV time-critical maximum since its previous frame, us */
    uint8_t  flags;              /* NL_F_* */
    uint8_t  reset_cause;        /* bits 4..0 NAV MCUCSR at boot, bits 7..5 NAV_RC_* (see above) */
} nl_nav_t;

typedef struct {
    uint8_t seq;                 /* SEQ of the last NAV frame the FCU accepted */
    uint8_t state;               /* supervisor state after the FCU's latest tick (LP_STATE_*) */
    uint8_t fflags;              /* NL_FF_* */
    uint8_t params_tag;          /* FC_PARAMS_TAG of the FCU build */
    uint8_t thr_q;               /* reserved byte: the rocket's FCU writes 0, its NAV ignores it */
} nl_fcu_t;

uint8_t nl_encode_nav(const nl_nav_t *n, uint8_t out[NL_XFER_BYTES]);
uint8_t nl_encode_fcu(const nl_fcu_t *f, uint8_t out[NL_XFER_BYTES]);   /* pads bytes 10..71 with 0x00 */
/* check sync, TYPE, CRC of a whole exchange buffer; return 1 and fill the struct if valid */
uint8_t nl_decode_nav(const uint8_t in[NL_XFER_BYTES], nl_nav_t *n);
uint8_t nl_decode_fcu(const uint8_t in[NL_XFER_BYTES], nl_fcu_t *f);

#endif /* FC_NLINK_PROTO_H */
