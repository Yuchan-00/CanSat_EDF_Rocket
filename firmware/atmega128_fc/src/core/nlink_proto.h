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
 *   thr_q (v0.7.1, the byte that was "reserved, always 0" in v0.6.3; the frame size is unchanged): the VTOL FCU's ESC
 *   throttle of its latest tick, round_half_away(u x 250) (nl_fcu_throttle_q), so the NAV can apply the baro thrust
 *   compensation (mission.vtol.baro_thrust_comp_m) BEFORE its altitude filter (core/alt_filter.h alt_vtol_step); the
 *   ballistic FCU writes 0 (v0.6.3 exchange bytes kept) and the ballistic NAV ignores it.  NAV side:
 *   nl_fcu_throttle(thr_q) = min(thr_q, 250) / 250 of the last valid FCU frame (0 before the first one).
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
 *   v0.7.1 VTOL build: the same two switches select the pad branches of the VTOL estimator (att_mahony_*: the
 *   gated Mahony correction instead of gyro-only in flight) and of the VTOL altitude filter (alt_vtol_*: h = v = 0
 *   and the accelerometer-bias calibration on the pad); the filter's baro thrust compensation takes u =
 *   nl_fcu_throttle(thr_q) of the last valid FCU frame (0 before the first one).
 *
 * nlink v3 (v0.8, the VTOL build with GPS position hold; architect + review v0.8 (3): the 72-byte exchange is KEPT - no
 * frame growth, the FCU's 0.15 ms frame-timeout margin stays): the NAV sends NL_TYPE_NAV3 (0x03) instead of NL_TYPE_NAV
 * with the 66-byte payload of nl_nav3_t (little-endian, no padding):
 *   q[4] f32       the PUBLISHED attitude = the gyro prediction of this tick (att_mahony_predict; the correction lands
 *                  in the next tick's prediction - the v0.8 NAV order predict -> publish -> correct)
 *   rate[3] f32    gyro - bias (the bias of the previous tick's correction), rad/s
 *   accel_z f32    body-z specific force of this tick (the vertical controller's support signature, k_T learning)
 *   h_m, vz_mps f32  the altitude filter's output of the PREVIOUS tick (it runs after the push)
 *   cos_tilt f32   1 - 2 (qx^2 + qy^2) of the published q (nl_nav3_tilt) - the vertical controller's c and the
 *                  position guard's tilt: the FCU stack input carries 4 bytes instead of the quaternion
 *   tilt_deg f32   acosf(clamp(cos_tilt, -1, 1)) x FC_RAD2DEG_F (the supervisor's I2 / ON / V7 gates, TELEM, log; the
 *                  last bit may differ from the reference's float64 acos - the FCU compares it with a threshold only)
 *   accel_x_q, accel_y_q i16  body x / y specific force x 100 (0.01 m/s^2, saturating; log only)
 *   h_baro_rel_cm i16         baro altitude above the ground reference x 100 (saturating; log only)
 *   imu_fail, baro_fails, twi_errors u8   the v0.6.3 counters saturated at 255 (log only)
 *   loop_us_max u16, flags u8, reset_cause u8   unchanged meaning
 *   gps_slot[5]    the rotating GPS slot (below)
 * GPS slot: byte 0 = header (cycle & 0x1F) << 3 | j, bytes 1..4 = part j of the snapshot the NAV latched at its last 50 Hz
 * step (core/gps_nav.h: the step runs at NAV tick k with k mod 5 == 4, after the push), sent in the frames of the next
 * five ticks, j = (frame tick) mod 5:
 *   j = 0 STATUS  flags u8 (GPS_F_*), numSV u8, hAcc u8 (0.1 m, saturating 255), reset_count u8 (wraps)
 *   j = 1 OFFSET  cumulative reset offset x / y i16 cm (W frame: x north, y west)
 *   j = 2 POS     CG position x / y i16 cm;   j = 3 VEL  CG velocity x / y i16 cm/s
 *   j = 4 HEAD    cos / sin of the NAV's heading psi, i16 Q15 (x 32767)
 *   j = 5..7      no snapshot yet (before the NAV's first step after boot: header 0x07, payload 0)
 * cycle = the NAV's step counter mod 32: the FCU (nl_slot_rx_*) takes a set only when slots 0..4 of ONE cycle arrived in
 * five consecutive ticks, completing at slot 4 - a lost frame costs that 20 ms cycle, never mixes two snapshots.  The
 * FCU frame is unchanged.  The ballistic build keeps NL_TYPE_NAV and the v0.6.3 payload bit for bit; a VTOL v0.8 FCU
 * refuses an NL_TYPE_NAV frame and vice versa (nl_decode_nav3: TYPE) - a board flashed with the other build is an I9 on
 * the pad.
 *
 * nlink v4 (v0.8.1, the four-chip build: params flight_computer.architecture_v081 - the GNS board runs the GPS receiver,
 * the GPS filter and the position controller; core/glink_proto.h is the NAV <-> GNS link and holds the normative per-tick
 * timing).  The 72-byte exchange is kept; both TYPEs are new, so a v0.8 (three-chip) board and a v0.8.1 board refuse each
 * other ('type' -> I9 on the pad):
 *   NAV -> FCU  NL_TYPE_NAV4 (0x04): the nl_nav3_t payload unchanged except that the slot is a GNS slot (below)
 *   FCU -> NAV  NL_TYPE_FCU4 (0x05): 0xD5 0x5D | 0x05 | SEQ | state | fflags | params_tag | thr_q | pflags | kt_q lo hi |
 *               CRC lo hi (NL_FCU4_FRAME_BYTES = 13, then 59 x 0x00).  pflags = phase (bits 2..0, LP_PHASE_*) | NL_PF_MOTOR_ON |
 *               NL_PF_SAT_HI (the vertical controller's sat == +1 this tick) | NL_PF_LOW_ALT | NL_PF_GROUND (low_alt and a
 *               touchdown signature this tick) | NL_PF_LIFTED (the take-off check passed, or off); kt_q = round_half_away(k_T x
 *               16384) saturated to u16 - every FCU-side input of the GNS position controller, of the FCU's latest tick.
 * GNS slot (nl_gns_tx_* NAV side, nl_gns_rx_* FCU side; sim/nlink_proto.py GnsSlotTx / GnsSlotRx): header (cycle & 0x1F) << 3
 * | i as in v3 (7 = empty); the five parts of the GNS result of one 50 Hz cycle = the bytes of the glink RESULT payload,
 * copied, not re-quantised:
 *   i = 0 TILT    tilt setpoint x / y i16 (rotation vector, the attitude controller's level frame, x 1e4 rad; x = NL_TILT_NONE
 *                 (-32768): the position loop is not engaged -> the FCU's builder inactive, target 0)
 *   i = 1 STATUS  flags u8 (GPS_F_*), numSV u8, hAcc u8 (0.1 m), pc u8 = mode (bits 2..0) | NL_PC_HOLD | requests << 4
 *   i = 2 POS     x / y i16 cm (W): p - hold point when NL_PC_HOLD is set (the FCU's TELEM distance), else p
 *   i = 3 VEL     x / y i16 cm/s (W)
 *   i = 4 TRIM    the integrator's trim tilt x / y i16 x 1e4 rad (level frame) - the FCU's fallback target
 * Phase: the parts of the result of cycle m (glink STATE sampled at NAV tick 5 m + 4) ride in the frames of NAV ticks
 * 5 m + 4 + L + i, L = params flight_computer.glink.latency_ticks_ASSUMED (GLINK_LATENCY_TICKS): TILT first.  At part 0 the
 * NAV latches the result of cycle m if it holds one with SEQ == m & 0xFF received after the cycle's sample tick and by
 * this frame; otherwise the five slots of cycle m are SLOT_EMPTY (a late result is never sent).  FCU staleness
 * (nl_gns_rx): < 2 missed cycles hold the last TILT; >= 2 -> the newest set's TRIM; >= land_steps cycles -> REQ_GPS_LOST
 * raised by the FCU itself (V11).
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
#define NL_TYPE_NAV3 0x03u                                      /* v0.8 VTOL + GPS: nl_nav3_t (the 72-byte exchange kept) */
#define NL_NAV_LEN 66u
#define NL_NAV3_LEN 66u                                         /* = NL_NAV_LEN */
#define NL_SLOT_LEN 5u                                          /* the v3 GPS slot: header + 4 payload bytes */
#define NL_SLOT_PARTS 5u                                        /* STATUS, OFFSET, POS, VEL, HEAD */
#define NL_SLOT_STATUS 0u
#define NL_SLOT_OFFSET 1u
#define NL_SLOT_POS 2u
#define NL_SLOT_VEL 3u
#define NL_SLOT_HEAD 4u
#define NL_SLOT_EMPTY_HDR 0x07u                                 /* no snapshot yet: header 0x07, payload 0 */
#define NL_SLOT_STALE_TICKS 10u                                 /* nl_slot_rx gate / fresh: the newest set at most this old */
#define NL_ACCEL_XY_SCALE 100u                                  /* v3 accel x / y: 0.01 m/s^2 */
#define NL_H_BARO_SCALE 100u                                    /* v3 h_baro_rel: cm */
#define NL_Q15 32767
#define NL_HEADER_BYTES 4u                                      /* sync x2, TYPE, SEQ */
#define NL_XFER_BYTES (NL_HEADER_BYTES + NL_NAV_LEN + 2u)        /* 72 */
#define NL_FCU_LEN 4u                                           /* state, fflags, params_tag, thr_q (v0.6.3: reserved) */
#define NL_FCU_THR_SCALE 250u                                   /* v0.7.1 thr_q = round_half_away(u x 250), 0..250 */
#define NL_FCU_FRAME_BYTES (NL_HEADER_BYTES + NL_FCU_LEN + 2u)   /* 10 */
/* v0.8.1 nlink v4 */
#define NL_TYPE_NAV4 0x04u                                      /* the nl_nav3_t payload with a GNS slot */
#define NL_TYPE_FCU4 0x05u                                      /* the FCU frame + pflags + kt_q */
#define NL_FCU4_LEN 7u                                          /* state, fflags, params_tag, thr_q, pflags, kt_q (u16) */
#define NL_FCU4_FRAME_BYTES (NL_HEADER_BYTES + NL_FCU4_LEN + 2u) /* 13 */
#define NL_PF_PHASE_MASK 0x07u                                  /* pflags: the supervisor phase after the FCU's latest tick */
#define NL_PF_MOTOR_ON 0x08u                                    /* state POWERED */
#define NL_PF_SAT_HI 0x10u                                      /* vertical controller sat == +1 (the NAV ORs it per cycle) */
#define NL_PF_LOW_ALT 0x20u                                     /* the vertical controller's flare band (LAND, h <= flare) */
#define NL_PF_GROUND 0x40u                                      /* low_alt and a touchdown signature this tick */
#define NL_PF_LIFTED 0x80u                                      /* the take-off check passed (or the check is off) */
#define NL_KT_SCALE 16384u                                      /* kt_q = k_T x 16384 (Q14) */
#define NL_TILT_SCALE 10000u                                    /* tilt / trim: rotation-vector components x 1e4 rad */
#define NL_TILT_NONE (-32768)                                   /* TILT x: the position loop is not engaged */
#define NL_PC_MODE_MASK 0x07u                                   /* STATUS byte 3: position-controller mode (POS_MODE_*) */
#define NL_PC_HOLD 0x08u                                        /* ... the hold point is captured (POS = p - hold) */
#define NL_PC_REQ_SHIFT 4u                                      /* ... its latched LAND requests (POS_REQ_*) << 4 */
#define NL_SLOT4_TILT 0u
#define NL_SLOT4_STATUS 1u
#define NL_SLOT4_POS 2u
#define NL_SLOT4_VEL 3u
#define NL_SLOT4_TRIM 4u
#define NL_GNS_RESULT_LEN 20u                                   /* the five 4-byte parts = the glink RESULT payload */
#define NL_GNS_RING 4u                                          /* nl_gns_tx: results kept by SEQ mod 4 (L <= 16 ticks) */
#define NL_GNS_L_MIN 2u                                         /* glink latency: the GNS answers in the NAV's next tick */
#define NL_GNS_L_MAX 16u                                        /* ... and the 4-entry ring holds L <= 16 */

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
    uint8_t thr_q;               /* v0.7.1 VTOL: ESC throttle of the FCU's latest tick, nl_fcu_throttle_q(u) (ballistic 0) */
} nl_fcu_t;

typedef struct {                 /* v0.8 nlink v3 payload (66 bytes, this order) */
    uint8_t  seq;
    float    q[4];               /* the published (predicted) attitude */
    float    rate[3];            /* gyro - bias, rad/s */
    float    accel_z;            /* body-z specific force, m/s^2 */
    float    h_m, vz_mps;        /* altitude filter of the previous tick */
    float    cos_tilt, tilt_deg; /* nl_nav3_tilt(q) */
    int16_t  accel_x_q, accel_y_q;   /* 0.01 m/s^2 (saturating) */
    int16_t  h_baro_rel_cm;      /* cm (saturating) */
    uint8_t  imu_fail, baro_fails, twi_errors;   /* saturated at 255 */
    uint16_t loop_us_max;
    uint8_t  flags, reset_cause;
    uint8_t  slot[NL_SLOT_LEN];  /* the rotating GPS slot */
} nl_nav3_t;

/* the GPS snapshot the NAV latches at each 50 Hz step, as wire integers (core/gps_nav.h quantises it once) */
typedef struct {
    uint8_t  flags;              /* GPS_F_* (core/gps_nav.h) */
    uint8_t  num_sv, hacc_dm, reset_count;
    int16_t  off_x_cm, off_y_cm, px_cm, py_cm, vx_cms, vy_cms;
    int16_t  cos_q15, sin_q15;
} nl_gps_snap_t;

/* the FCU's view of one complete slot set (the wire integers; position_ctrl dequantises: cm / 100, Q15 / 32767) */
typedef struct {
    uint8_t  flags, num_sv, hacc_dm, reset_count;
    int16_t  off[2], p[2], v[2];  /* cm, cm, cm/s (W frame) */
    int16_t  cs[2];               /* cos, sin of the NAV heading, Q15 */
    uint8_t  cycle;
} nl_gps_set_t;

typedef struct {                 /* NAV side (sim.nlink_proto.GpsSlotTx) */
    bool          has;           /* a snapshot was latched since boot */
    uint8_t       cycle;         /* the filter's step counter (low byte; & 0x1F on the wire) */
    nl_gps_snap_t snap;
} nl_slot_tx_t;

typedef struct {                 /* FCU side (sim.nlink_proto.GpsSlotRx) */
    uint8_t      age[NL_SLOT_PARTS];             /* ticks since part j arrived (saturating 255; 255 = never) */
    uint8_t      cyc[NL_SLOT_PARTS];
    uint8_t      pl[NL_SLOT_PARTS][4];
    bool         has_latest;
    uint8_t      latest_age;                     /* ticks since the newest complete set (saturating) */
    bool         got_part4;                      /* this tick's slot was part 4 (the position step is due) */
    nl_gps_set_t latest;
    uint16_t     sets, broken;                   /* diagnostics (wrap) */
} nl_slot_rx_t;

/* ---- v0.8.1 nlink v4 ---- */
typedef struct {                 /* sim.nlink_proto.Fcu4 */
    uint8_t  seq, state, fflags, params_tag, thr_q;
    uint8_t  pflags;             /* NL_PF_* | phase */
    uint16_t kt_q;               /* k_T x 16384 */
} nl_fcu4_t;

typedef struct {                 /* one glink RESULT held by the NAV (a ring entry) */
    bool     valid;
    uint8_t  seq;
    uint32_t k_rx;               /* the NAV tick in which it was decoded */
    uint8_t  pl[NL_GNS_RESULT_LEN];
} nl_gns_res_t;

typedef struct {                 /* NAV side (sim.nlink_proto.GnsSlotTx) */
    uint8_t      n, L;           /* step ticks (5), latency ticks (GLINK_LATENCY_TICKS) */
    nl_gns_res_t ring[NL_GNS_RING];
    bool         has_cur;        /* the set of cycle cur_m is being sent */
    int32_t      cur_m;          /* cycle of the set being sent (or of the last part-0 decision); -1 none */
    uint8_t      cur[NL_GNS_RESULT_LEN];
    bool         ph_ok;          /* the phase (ph_m, ph_i) belongs to tick ph_k */
    uint32_t     ph_k;
    int32_t      ph_m;
    uint8_t      ph_i;
    uint16_t     latched, missed, results;   /* diagnostics (wrap) */
} nl_gns_tx_t;

typedef struct {                 /* the FCU's view of the newest complete v4 set (sim.nlink_proto.GnsData), wire integers */
    uint8_t  flags, num_sv, hacc_dm;
    uint8_t  pc;                 /* mode | NL_PC_HOLD | requests << 4 */
    int16_t  pos[2], vel[2];     /* cm, cm/s (W) */
    int16_t  trim[2];            /* x 1e4 rad (level frame) */
    uint8_t  cycle;              /* (the set's TILT part: nl_gns_rx_t.pl[NL_SLOT4_TILT] right after the completing feed) */
    float    dist_m;             /* |pos| (m) when NL_PC_HOLD, else NaN - the TELEM v3 distance (one sqrt per set) */
} nl_gns_data_t;

typedef struct {                 /* FCU side (sim.nlink_proto.GnsSlotRx) */
    uint8_t       n;             /* step ticks (5) */
    uint8_t       age[NL_SLOT_PARTS];            /* ticks since part i arrived (saturating 255; 255 = never) */
    uint8_t       cyc[NL_SLOT_PARTS];
    uint8_t       pl[NL_SLOT_PARTS][4];
    bool          has_latest;
    uint8_t       latest_age;                    /* ticks since the newest complete set (saturating) */
    bool          got_part4;
    nl_gns_data_t latest;
    bool          has_tilt;                      /* a TILT part arrived since boot */
    bool          tilt_on;                       /* the newest TILT part was not NL_TILT_NONE */
    int16_t       tilt[2];                       /* the newest TILT part (x 1e4 rad) */
    uint16_t      tilt_age;                      /* FCU ticks since it (saturating 0xFFFF) */
    uint16_t      sets, broken, tilts;           /* diagnostics (wrap) */
} nl_gns_rx_t;

uint8_t nl_encode_nav4(const nl_nav3_t *n, uint8_t out[NL_XFER_BYTES]);  /* the v3 payload under NL_TYPE_NAV4 */
uint8_t nl_decode_nav4(const uint8_t in[NL_XFER_BYTES], nl_nav3_t *n);   /* TYPE must be NL_TYPE_NAV4 */
uint8_t nl_encode_fcu4(const nl_fcu4_t *f, uint8_t out[NL_XFER_BYTES]);  /* 13 bytes, then 59 x 0x00 */
uint8_t nl_decode_fcu4(const uint8_t in[NL_XFER_BYTES], nl_fcu4_t *f);   /* bytes 13..71 ignored */
/* FCU side: pflags of this tick; kt_q = round_half_away(k_T x 16384) saturated to 0..65535, NaN -> 0 */
uint8_t  nl_fcu4_pflags(uint8_t phase, bool motor_on, bool sat_hi, bool low_alt, bool ground, bool lifted);
uint16_t nl_kt_quant(float k_t);
float    nl_kt_value(uint16_t kt_q);                                    /* kt_q / 16384 (exact) */
/* GNS side: one tilt component (rad) -> round_half_away(x 1e4) saturated to -32767..32767 (NL_TILT_NONE stays reserved),
 * NaN -> 0; exact for every float (lp_quantize_wide) */
int16_t  nl_tilt_q(float x);
uint8_t  nl_pc_byte(uint8_t mode, bool hold, uint8_t requests);        /* mode & 7 | HOLD | (requests & 15) << 4 */
/* NAV side: the GNS slot of the frame of NAV tick k (sim GnsSlotTx); call nl_gns_tx_result for every valid RESULT frame
 * decoded in tick k_rx (before the frame build of a later tick), nl_gns_tx_slot once per frame build (ticks advance by 1) */
void     nl_gns_tx_init(nl_gns_tx_t *tx, uint8_t step_ticks, uint8_t latency_ticks);
void     nl_gns_tx_result(nl_gns_tx_t *tx, uint8_t seq, const uint8_t pl[NL_GNS_RESULT_LEN], uint32_t k_rx);
void     nl_gns_tx_slot(nl_gns_tx_t *tx, uint32_t k, uint8_t out[NL_SLOT_LEN]);
/* FCU side, once per FCU tick with the slot of this tick's valid v4 frame (NULL: none); returns the complete set when this
 * tick's slot was part 4 and parts 0..4 of one cycle arrived in the last five ticks, else NULL */
void     nl_gns_rx_init(nl_gns_rx_t *rx, uint8_t step_ticks);
const nl_gns_data_t *nl_gns_rx_feed(nl_gns_rx_t *rx, const uint8_t *slot);
bool     nl_gns_rx_fresh(const nl_gns_rx_t *rx);                       /* a set completed within NL_SLOT_STALE_TICKS */
bool     nl_gns_rx_gate_ok(const nl_gns_rx_t *rx);                     /* fresh and GPS_F_GATE_OK (0x02) */
bool     nl_gns_rx_fallback(const nl_gns_rx_t *rx);                    /* no TILT part for >= 2 cycles (or never) */
/* the builder's target of this tick: returns engaged (the newest TILT part was not NONE); tgt = that TILT, or in the
 * fallback the newest set's TRIM (0 without a set), rad */
bool     nl_gns_rx_target(const nl_gns_rx_t *rx, float tgt[2]);
/* REQ_* for the supervisor: the newest set's latched requests | 0x01 (GPS_LOST) once no TILT came for land_steps cycles */
uint8_t  nl_gns_rx_requests(const nl_gns_rx_t *rx, uint16_t land_steps);
/* TELEM v3 GPS fields (flags / mode / dist of the newest fresh set, sv / hAcc of the newest set; mode ATT (4) in the trim
 * fallback while motor_on and engaged) */
void     nl_gns_rx_tlm(const nl_gns_rx_t *rx, bool motor_on, uint8_t *flags, uint8_t *num_sv, uint8_t *hacc_dm,
                       uint8_t *pos_mode, float *dist_m);

/* complete frames; return the number of meaningful bytes written (NL_XFER_BYTES / NL_FCU_FRAME_BYTES) */
uint8_t nl_encode_nav(const nl_nav_t *n, uint8_t out[NL_XFER_BYTES]);
uint8_t nl_encode_fcu(const nl_fcu_t *f, uint8_t out[NL_XFER_BYTES]);   /* pads bytes 10..71 with 0x00 */
/* check sync, TYPE, CRC of a whole exchange buffer; return 1 and fill the struct if valid */
uint8_t nl_decode_nav(const uint8_t in[NL_XFER_BYTES], nl_nav_t *n);
uint8_t nl_decode_fcu(const uint8_t in[NL_XFER_BYTES], nl_fcu_t *f);
/* v0.7.1 thr_q: FCU side u (0..1) -> round_half_away(u x NL_FCU_THR_SCALE) saturated to 0..250, NaN -> 0 (exact:
 * lp_quantize); NAV side thr_q -> min(thr_q, 250) / 250 (the u of the baro thrust compensation, alt_vtol_step) */
uint8_t nl_fcu_throttle_q(float u);
float   nl_fcu_throttle(uint8_t thr_q);

/* ---- v0.8 nlink v3 ---- */
uint8_t nl_encode_nav3(const nl_nav3_t *n, uint8_t out[NL_XFER_BYTES]);
uint8_t nl_decode_nav3(const uint8_t in[NL_XFER_BYTES], nl_nav3_t *n);   /* TYPE must be NL_TYPE_NAV3 */
/* NAV side: cos_tilt = 1 - 2 (x^2 + y^2) of the published q in float32 steps, tilt_deg = acosf(clamp) x FC_RAD2DEG_F */
void    nl_nav3_tilt(const float q[4], float *cos_tilt, float *tilt_deg);
/* NAV side: the quantised / saturated v3 fields from physical values (sim/nlink_proto.py Nav3.from_physical) */
void    nl_nav3_set_physical(nl_nav3_t *n, const float accel[3], float h_baro_rel_m, uint16_t imu_fail, uint16_t baro_fails,
                             uint16_t twi_errors);
/* the 5 slot bytes of part j (0..4) of a snapshot (NULL: no snapshot yet -> 07 00 00 00 00) */
void    nl_slot_encode(const nl_gps_snap_t *snap, uint8_t cycle, uint8_t j, uint8_t out[NL_SLOT_LEN]);
void    nl_slot_tx_init(nl_slot_tx_t *tx);
void    nl_slot_tx_latch(nl_slot_tx_t *tx, const nl_gps_snap_t *snap, uint16_t cycle);   /* right after the filter step */
void    nl_slot_tx_slot(const nl_slot_tx_t *tx, uint32_t nav_tick, uint8_t out[NL_SLOT_LEN]);   /* part nav_tick mod 5 */
void    nl_slot_rx_init(nl_slot_rx_t *rx);
/* once per FCU tick with the slot of this tick's valid v3 frame (NULL: no valid frame); returns the complete set when this
 * tick's slot was part 4 and parts 0..4 of the same cycle arrived in the last five ticks, else NULL */
const nl_gps_set_t *nl_slot_rx_feed(nl_slot_rx_t *rx, const uint8_t *slot);
bool    nl_slot_rx_fresh(const nl_slot_rx_t *rx);       /* a set completed within NL_SLOT_STALE_TICKS */
bool    nl_slot_rx_gate_ok(const nl_slot_rx_t *rx);     /* fresh and its flags have GPS_F_GATE_OK (0x02) */

#endif /* FC_NLINK_PROTO_H */
