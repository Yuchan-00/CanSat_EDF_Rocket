/* link_proto.h - LoRa ground-link frame format (SPEC 10.3), a byte-exact port of
 * sim/link_proto.py (portable C99, no AVR headers).
 *
 * Frame (both directions, multi-byte fields little-endian):
 *
 *     0xA5 0x5A | LEN | VEH | TYPE | SEQ | PAYLOAD | CRC16
 *                 LEN counts VEH..PAYLOAD (= 3 + payload bytes)
 *                 CRC16 over LEN..PAYLOAD, transmitted low byte first
 *
 * CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, MSB first, no reflection, no
 * final XOR (check value of "123456789" = 0x29B1).
 *   uplink   LP_TYPE_CMD   (0x10)  5 bytes: cmd u8, arg u16 (throttle x1000), nonce u16
 *   downlink LP_TYPE_TELEM (0x20) 28 bytes: state, flags, reason, ack_seq u8, nonce u16, t_ms u32,
 *            h i16 (0.1 m), v_z i16 (cm/s), roll/pitch/yaw i16 (0.01 deg), tilt u16 (0.01 deg),
 *            throttle u16 (x1000), V_cell u16 (mV), loop_max u16 (us)               (TELEM v1, ballistic build)
 *   downlink LP_TYPE_TELEM2 (0x21) 31 bytes: the 28 TELEM v1 bytes, then phase u8 (LP_PHASE_*) and the
 *            altitude target h_tgt i16 (0.1 m)                                    (TELEM v2, v0.7 VTOL build)
 *            A new TYPE rather than a longer 0x20: every TYPE keeps one fixed payload length (the decoder
 *            rule), and the ballistic build keeps sending the v0.6.3 TELEM bytes.
 *   downlink LP_TYPE_TELEM3 (0x22) 37 bytes (v0.8, the VTOL build with GPS position hold): the 31 TELEM v2 bytes, then
 *            gps_flags u8 (core/gps_nav.h GPS_F_*), num_sv u8, hacc u8 (0.1 m, 255 = >= 25.5 m / none), pos_mode u8
 *            (core/position_ctrl.h POS_MODE_*) and dist u16 (distance from the hold point, 0.1 m; LP_DIST_NONE 0xFFFF = no
 *            hold point).  LP_LEN_MAX is 40 in every build (v0.6.3: 31, v0.7: 34) - the decoders take the longest frame
 *            of any build (sim/link_proto.py LEN_MAX); a valid ballistic or v0.7 frame decodes exactly as before, only the
 *            error class of a corrupted candidate with LEN 35..40 changes (CRC / payload length instead of bad_len).
 *   downlink LP_TYPE_TELEM4 (0x23) 33 bytes / LP_TYPE_TELEM5 (0x24) 39 bytes (v0.9 firmware round RP1, the tether / hybrid
 *            power builds): the TELEM v2 / v3 bytes, then the power tail of sim/power_link.py - v_aux u8 (0.125 V: the
 *            buffer in wired_buffer, V_tether = V_in in wired) and pwr_flags u8 (PF_* | build << 6).  A reply carries
 *            the tail exactly when its build bits (pwr_flags >> LP_PWR_TAIL_BUILD_SHIFT) are not the battery build
 *            (LP_PWR_TAIL_DUE; sim/link_proto.py power_tail_due): the COM decides per snapshot (one COM image for every
 *            power build), the battery VTOL build keeps TELEM v2 / v3 byte for byte.  LP_LEN_MAX is 42 (TELEM5) in every
 *            build that can fly VTOL and in the host builds (sim/link_proto.py LEN_MAX 42).  The ballistic AVR images
 *            (-D FC_MODE_BALLISTIC) keep the v0.8.1 sizes (LP_RP1 0: LEN_MAX 40, frame buffers 45 B) so they stay
 *            byte-identical (fw09 SPEC section 7 row 21); their uplink decoder accepts CMD only, so a candidate with
 *            LEN 41 / 42 is noise there either way - it is counted bad_len at its LEN byte instead of bad_crc 43 / 44
 *            bytes later (every frame found is the same, as the failed candidate's bytes are re-parsed).
 *   downlink LP_TYPE_TELEM6 (0x25) 32 bytes / LP_TYPE_TELEM7 (0x26) 38 bytes (2026-11-10 control round, the VTOL battery
 *            build on the carrier rev 2c): the TELEM v2 / v3 bytes, then the logic-battery tail lbat u8 (the FCU's PF3
 *            reading, LP_LBAT_SCALE = 50 per volt: 0.02 V steps, 1..255).  The FCU sends it in the snapshot's spare byte
 *            (SNAP2 spare / SNAP3 spare3; 0 = LP_LBAT_NONE, this build has no monitor); the COM appends it exactly when
 *            that byte is not 0 and the snapshot carries no power tail (LP_LBAT_TAIL_DUE; sim/link_proto.py
 *            logic_tail_due) - one COM image for every build, every other build keeps its TELEM byte for byte.
 *            LP_LEN_MAX stays 42.  Only in the LP_RP1 builds (the ballistic images never send or accept them).
 *
 * Mission modes (v0.7, params.json mission.mode, a BUILD-TIME choice, params_gen.h MISSION_MODE): VTOL (the
 * ground sends ON / OFF / UP / DOWN and KILL, SPEC 8.9) and BALLISTIC (the v0.4-v0.6.3 ARM / LAUNCH /
 * THROTTLE / CUT / RELEASE mission, bit-identical).  One command table for both builds; the vehicle
 * rejects the other build's commands (VTOL: LP_REJ_MODE; ballistic: LP_REJ_UNKNOWN_CMD for 8..11, exactly
 * what v0.6.3 answered).
 *
 * Streaming decoder (lp_decoder_feed): HUNT0 (0xA5) -> HUNT1 (0x5A; another 0xA5
 * restarts the candidate) -> LEN (LP_LEN_MIN..LP_LEN_MAX) -> BODY (LEN + 2 bytes).  A
 * failed candidate (LEN out of range or CRC mismatch) loses only its first byte; the
 * rest is parsed again from HUNT0 through a small replay queue, so a real frame that
 * starts inside garbage or inside a corrupted frame is still found.  A frame whose
 * CRC is correct is consumed whole even when it is then dropped for a wrong vehicle
 * id, TYPE or payload length (counted only).  The counters mirror the Python
 * reference (frames, bad_len, bad_crc, wrong_vehicle, wrong_type, bad_payload_len,
 * skipped_bytes).
 *
 * Quantisation (lp_quantize): value x scale rounded half away from zero, saturated,
 * NaN -> 0, computed EXACTLY in integer arithmetic from the float's bits, so a float32
 * input gives the same integer as the Python reference's float64 product (the product
 * of a 24-bit mantissa and a scale whose odd part is < 256 is exact in both).
 *
 * Invariant: lp_encode_*(lp_unpack_*(x)) == x for every valid frame, and the decoder
 * never emits a frame whose CRC, vehicle id, TYPE or payload length is wrong.
 */
#ifndef FC_LINK_PROTO_H
#define FC_LINK_PROTO_H

#include <stdbool.h>
#include <stdint.h>

#define LP_SYNC0 0xA5u
#define LP_SYNC1 0x5Au
#define LP_TYPE_CMD 0x10u
#define LP_TYPE_TELEM 0x20u
#define LP_TYPE_TELEM2 0x21u                                        /* v0.7 TELEM v2 (VTOL build) */
#define LP_TYPE_TELEM3 0x22u                                        /* v0.8 TELEM v3 (VTOL build with GPS position hold) */
#define LP_TYPE_TELEM4 0x23u                                        /* v0.9 TELEM v4: TELEM v2 + the power tail (RP1) */
#define LP_TYPE_TELEM5 0x24u                                        /* v0.9 TELEM v5: TELEM v3 + the power tail (RP1) */
#define LP_TYPE_TELEM6 0x25u                                        /* 2026-11-10 TELEM v6: TELEM v2 + the logic-battery tail */
#define LP_TYPE_TELEM7 0x26u                                        /* 2026-11-10 TELEM v7: TELEM v3 + the logic-battery tail */
/* v0.9 RP1: 1 = the v0.9 link sizes (LEN_MAX 42); 0 = the ballistic AVR images (FC_MODE_BALLISTIC), byte-identical to
 * v0.8.1 (LEN_MAX 40) - see the TELEM4 / TELEM5 paragraph above */
#if defined(__AVR__) && defined(FC_MODE_BALLISTIC)
#define LP_RP1 0
#else
#define LP_RP1 1
#endif
#define LP_HEADER_BYTES 3u
#define LP_CMD_PAYLOAD_LEN 5u
#define LP_TELEM_PAYLOAD_LEN 28u
#define LP_TELEM2_PAYLOAD_LEN 31u                                   /* TELEM v1 + phase u8 + h_tgt i16 */
#define LP_TELEM3_PAYLOAD_LEN 37u                                   /* TELEM v2 + gps_flags, num_sv, hacc, pos_mode u8, dist u16 */
#define LP_PWR_TAIL_BYTES 2u                                        /* v0.9 power tail: v_aux u8, pwr_flags u8 */
#define LP_PWR_TAIL_BUILD_SHIFT 6u                                  /* pwr_flags bits 6-7: the power build (0 = battery) */
#define LP_PWR_TAIL_DUE(pwr_flags) ((((uint8_t)(pwr_flags) >> LP_PWR_TAIL_BUILD_SHIFT) & 0x03u) != 0u)
#define LP_TELEM4_PAYLOAD_LEN (LP_TELEM2_PAYLOAD_LEN + LP_PWR_TAIL_BYTES)    /* 33 */
#define LP_TELEM5_PAYLOAD_LEN (LP_TELEM3_PAYLOAD_LEN + LP_PWR_TAIL_BYTES)    /* 39 */
#define LP_LBAT_TAIL_BYTES 1u                                       /* 2026-11-10 logic-battery tail: lbat u8 */
#define LP_LBAT_NONE 0u                                             /* snapshot lbat byte: no monitor in this build */
#define LP_LBAT_SCALE 50u                                           /* lbat = round(V x 50): 0.02 V steps */
#define LP_LBAT_TAIL_DUE(lbat, pwr_flags) ((uint8_t)(lbat) != LP_LBAT_NONE && !LP_PWR_TAIL_DUE(pwr_flags))
#define LP_TELEM6_PAYLOAD_LEN (LP_TELEM2_PAYLOAD_LEN + LP_LBAT_TAIL_BYTES)   /* 32 */
#define LP_TELEM7_PAYLOAD_LEN (LP_TELEM3_PAYLOAD_LEN + LP_LBAT_TAIL_BYTES)   /* 38 */
#if LP_RP1
#define LP_MAX_PAYLOAD_LEN 39u                                      /* TELEM v5 (v0.8: 37) */
#else
#define LP_MAX_PAYLOAD_LEN 37u                                      /* the ballistic images: the v0.8.1 size */
#endif
#define LP_LEN_MIN LP_HEADER_BYTES
#define LP_LEN_MAX (LP_HEADER_BYTES + LP_MAX_PAYLOAD_LEN)          /* 42 (v0.6.3: 31, v0.7: 34, v0.8: 40) */
#define LP_FRAME_OVERHEAD 5u                                        /* sync x2, LEN, CRC x2 */
#define LP_CMD_FRAME_BYTES (LP_FRAME_OVERHEAD + LP_HEADER_BYTES + LP_CMD_PAYLOAD_LEN)       /* 13 */
#define LP_TELEM_FRAME_BYTES (LP_FRAME_OVERHEAD + LP_HEADER_BYTES + LP_TELEM_PAYLOAD_LEN)   /* 36 */
#define LP_TELEM2_FRAME_BYTES (LP_FRAME_OVERHEAD + LP_HEADER_BYTES + LP_TELEM2_PAYLOAD_LEN) /* 39 */
#define LP_TELEM3_FRAME_BYTES (LP_FRAME_OVERHEAD + LP_HEADER_BYTES + LP_TELEM3_PAYLOAD_LEN) /* 45 */
#define LP_TELEM4_FRAME_BYTES (LP_FRAME_OVERHEAD + LP_HEADER_BYTES + LP_TELEM4_PAYLOAD_LEN) /* 41 */
#define LP_TELEM5_FRAME_BYTES (LP_FRAME_OVERHEAD + LP_HEADER_BYTES + LP_TELEM5_PAYLOAD_LEN) /* 47 */
#define LP_TELEM6_FRAME_BYTES (LP_FRAME_OVERHEAD + LP_HEADER_BYTES + LP_TELEM6_PAYLOAD_LEN) /* 40 */
#define LP_TELEM7_FRAME_BYTES (LP_FRAME_OVERHEAD + LP_HEADER_BYTES + LP_TELEM7_PAYLOAD_LEN) /* 46 */
#define LP_FRAME_MAX_BYTES (LP_FRAME_OVERHEAD + LP_LEN_MAX)        /* 47 (v0.7: 39, v0.8: 45) */
#define LP_DIST_NONE 0xFFFFu                                        /* TELEM v3 dist: no hold point */
#define LP_DIST_SCALE 10u                                           /* TELEM v3 dist: 0.1 m */
#define LP_HACC_NONE 255u                                           /* TELEM v3 hacc: none / >= 25.5 m */
#define LP_CRC_INIT 0xFFFFu
#define LP_CRC_POLY 0x1021u

/* mission modes (params.json mission.mode; params_gen.h MISSION_MODE_* has the same values, checked by fc_config.c) */
#define LP_MODE_BALLISTIC 0u
#define LP_MODE_VTOL 1u
/* v0.8.1 RAM diet: a VTOL-only AVR image (-D ONB_FIXED_MODE=LP_MODE_VTOL, the FCU's VTOL envs) may drop fields that are
 * constant after init or diagnostics nobody reads (core/mixer.h, core/pid.h); host builds and the ballistic image keep
 * their layout (and the ballistic image its bytes).  The numbers are the same either way. */
#if defined(__AVR__) && defined(ONB_FIXED_MODE)
#if ONB_FIXED_MODE == LP_MODE_VTOL
#define FC_VTOL_RAM_TRIM 1
#endif
#endif
#ifndef FC_VTOL_RAM_TRIM
#define FC_VTOL_RAM_TRIM 0
#endif
/* v0.9 (control / fix rounds, sim/controller.py VTOL_EXTRA_KEYS, sim/vertical_ctrl.py, sim/onboard.py): the VTOL options
 * are compiled into every build that can fly VTOL (host builds, the FCU's VTOL envs); a ballistic fixed build
 * (ONB_FIXED_MODE = LP_MODE_BALLISTIC) keeps the v0.8.1 code - its image stays byte-identical */
#if !defined(ONB_FIXED_MODE) || ONB_FIXED_MODE == LP_MODE_VTOL
#define FC_V09 1
#else
#define FC_V09 0
#endif

/* uplink cmd codes */
#define LP_CMD_NONE 0u
#define LP_CMD_ARM 1u
#define LP_CMD_DISARM 2u
#define LP_CMD_LAUNCH 3u
#define LP_CMD_THROTTLE 4u
#define LP_CMD_CUT 5u
#define LP_CMD_RELEASE 6u
#define LP_CMD_KILL 7u
#define LP_CMD_ON 8u                  /* v0.7 VTOL: take off (carries the telemetry nonce, like LAUNCH) */
#define LP_CMD_OFF 9u                 /* v0.7 VTOL: land where it is / cancel the spool-up */
#define LP_CMD_UP 10u                 /* v0.7 VTOL: altitude target + alt_step_m */
#define LP_CMD_DOWN 11u               /* v0.7 VTOL: altitude target - alt_step_m */
#define LP_CMD_MAX_BALLISTIC LP_CMD_KILL   /* the ballistic build: 8..11 are unknown (REJ_UNKNOWN_CMD, as v0.6.3) */
#define LP_CMD_MAX LP_CMD_DOWN
#define LP_THROTTLE_SCALE 1000u

/* VTOL phases (TELEM v2 phase u8): SAFE = NONE, ARMED = SPOOL, POWERED = CLIMB / HOLD / LAND, UNPOWERED = LANDED,
 * ABORT / RELEASED = NONE; the ballistic build has no phase (TELEM v1) */
#define LP_PHASE_NONE 0u
#define LP_PHASE_SPOOL 1u             /* ARMED: buzzer warning, motor off, spool_warning_s */
#define LP_PHASE_CLIMB 2u             /* POWERED: take-off climb until the reference first reaches the target */
#define LP_PHASE_HOLD 3u              /* POWERED: holding the altitude target (UP / DOWN move it) */
#define LP_PHASE_LAND 4u              /* POWERED: descending to touchdown */
#define LP_PHASE_LANDED 5u            /* UNPOWERED: touchdown confirmed, motor off; SAFE after safe_after_landed_s */

/* telemetry state u8 */
#define LP_STATE_SAFE 0u
#define LP_STATE_ARMED 1u
#define LP_STATE_POWERED 2u
#define LP_STATE_UNPOWERED 3u
#define LP_STATE_RELEASED 4u
#define LP_STATE_ABORT 5u

/* telemetry flag bits (SPEC 10.3 order) */
#define LP_FLAG_ARM_SWITCH 0x01u
#define LP_FLAG_SENSORS_OK 0x02u
#define LP_FLAG_ALIGNED 0x04u
#define LP_FLAG_LINK_OK 0x08u
#define LP_FLAG_RELEASED 0x10u
#define LP_FLAG_CONTROL_ACTIVE 0x20u
#define LP_FLAG_REJECTED 0x40u
#define LP_FLAG_INTERLOCK 0x80u

/* reason codes: interlocks 1..63, command rejections 0x40..0x7F.  A code names the RULE (its condition and
 * parameters); the action may differ between the builds: I8 and I9-baro land in the VTOL build and abort in the
 * ballistic one, I10 boots ABORT (VTOL) or SAFE with the release open (ballistic).  V1..V10: VTOL-only rules; V11..V15
 * and LP_REJ_GPS: the v0.8 VTOL build with GPS position hold (onb_params_t.gps_hold / takeoff_check). */
#define LP_REASON_NONE 0u
#define LP_R_I1_LINK_LOSS 1u
#define LP_R_I1P_LINK_LOSS_PAD 2u
#define LP_R_I2_TILT 3u
#define LP_R_I3_RATE 4u
#define LP_R_I4_APOGEE_LIMIT 5u
#define LP_R_I5_GEOFENCE 6u
#define LP_R_I6_FALL 7u
#define LP_R_I7_MOTOR_TIME 8u
#define LP_R_I8_VCELL 9u
#define LP_R_I9_IMU 10u
#define LP_R_I9_BARO 11u
#define LP_R_I10_RESET 12u
#define LP_R_ARM_SWITCH_OPEN 13u
#define LP_R_V_LINK_LAND 14u          /* V1 VTOL CLIMB/HOLD: no ground frame for link_loss_land_s -> LAND */
#define LP_R_V_ENDURANCE 15u          /* V2 VTOL CLIMB/HOLD: motor time >= max_flight_s - land_reserve_s -> LAND */
#define LP_R_V_CEILING 16u            /* V3 VTOL CLIMB/HOLD: h > ceiling_m -> LAND */
#define LP_R_V_GEOFENCE 17u           /* V4 VTOL POWERED/UNPOWERED: h > vtol.geofence_m -> ABORT */
#define LP_R_V_FALL 18u               /* V5 VTOL POWERED/UNPOWERED: v_z <= vtol.fall_vz_mps for fall_confirm_s -> ABORT */
#define LP_R_V_HARD_CUTOFF 19u        /* V6 VTOL POWERED: motor time >= hard_cutoff_s -> ABORT */
#define LP_R_V_TAKEOFF_GATE 20u       /* V7 VTOL end of the spool warning: tilt / V_cell gate failed -> SAFE */
#define LP_R_V_BARO_LIMIT_LANDED 21u  /* V8 (v0.7.1) VTOL LAND, baro lost: time limit reached with a ground signature ->
                                       * UNPOWERED (LANDED), motor off, no release */
#define LP_R_V_BARO_LIMIT_ABORT 22u   /* V9 (v0.7.1) the same limit without a ground signature -> ABORT (altitude unknown) */
#define LP_R_V_LAND_TIMEOUT 23u       /* V10 (v0.7.1) VTOL LAND, baro healthy: low and not following the descending
                                       * reference for the floor timeout -> UNPOWERED (LANDED), motor off, no release */
#define LP_R_V_GPS_LOST 24u           /* V11 (v0.8) VTOL CLIMB/HOLD: no usable GPS for gps_loss.timeout_s (attitude-only
                                       * hold) and then land_after_s -> LAND (the landing drifts with the wind) */
#define LP_R_V_FENCE 25u              /* V12 (v0.8) VTOL CLIMB/HOLD: |p - hold point| > fence_m -> LAND (never ABORT) */
#define LP_R_V_GPS_IMPLAUSIBLE 26u    /* V13 (v0.8) VTOL CLIMB/HOLD: GPS and IMU disagree -> GPS unusable, attitude only, LAND */
#define LP_R_V_AUTHORITY 27u          /* V14 (v0.8) VTOL CLIMB/HOLD: learnt k_T x T/W < min_kT_TW -> LAND */
#define LP_R_V_TAKEOFF_CHECK 28u      /* V15 (v0.8) VTOL CLIMB/HOLD: pinned at u_max without the climb -> motor off, SAFE */
#define LP_REJ_STATE 0x40u
#define LP_REJ_NONCE 0x41u
#define LP_REJ_ARM_SWITCH 0x42u
#define LP_REJ_SENSORS 0x43u
#define LP_REJ_NOT_ALIGNED 0x44u
#define LP_REJ_TILT 0x45u
#define LP_REJ_VCELL 0x46u
#define LP_REJ_LINK 0x47u
#define LP_REJ_UNKNOWN_CMD 0x48u
#define LP_REJ_THROTTLE_ZERO 0x49u
#define LP_REJ_RELEASE_PENDING 0x4Au
#define LP_REJ_RELEASED 0x4Bu
#define LP_REJ_MODE 0x4Cu             /* VTOL build: ARM / DISARM / LAUNCH / THROTTLE / CUT / RELEASE are not used */
#define LP_REJ_ALT_LIMIT 0x4Du        /* UP / DOWN: the target would leave [alt_min_m, alt_max_m] */
#define LP_REJ_LANDING 0x4Eu          /* UP / DOWN during LAND: a landing is never cancelled */
#define LP_REJ_ENDURANCE 0x4Fu        /* ON: cumulative motor time >= max_flight_s - land_reserve_s */
#define LP_REJ_GPS 0x50u              /* v0.8 ON: the GPS is not ready (the NAV's GATE_OK of the newest fresh slot set) */
/* v0.9 power rules (sim/power_link.py: a block of their own from 0x30 so pending interlock codes cannot collide) */
#define LP_R_W_LOST 0x30u             /* wired_buffer: the tether is lost (V_tether low / below the buffer) -> LAND */
#define LP_R_W_SAG 0x31u              /* wired: the supply sags / is in its current limit -> ceiling frozen, then LAND */
#define LP_R_W_BUF 0x32u              /* wired_buffer: the buffer below buf_low / its landing reserve -> LAND */
#define LP_R_W_D2 0x33u               /* wired_buffer: the buffer charging while tethered (D2 short) -> LAND */
#define LP_REJ_POWER 0x58u            /* ON / UP refused: the supply / tether / buffer outside the ON window, or sagging */
/* (v0.9 RP1: sim/link_proto.py defines these five codes since the firmware round; sim/power_link.py re-exports them) */

/* telemetry scaling (wire unit per physical unit) */
#define LP_H_SCALE 10u          /* i16, 0.1 m */
#define LP_VZ_SCALE 100u        /* i16, cm/s */
#define LP_ANGLE_SCALE 100u     /* i16 roll/pitch/yaw, u16 tilt: 0.01 deg */
#define LP_VCELL_SCALE 1000u    /* u16, mV */
#define LP_I16_MIN (-32768L)
#define LP_I16_MAX 32767L
#define LP_U16_MAX 65535L

typedef struct {
    uint8_t  cmd;
    uint16_t arg;               /* THROTTLE: u x 1000 */
    uint16_t nonce;             /* LAUNCH / THROTTLE-from-ARMED: the nonce of the latest telemetry */
} lp_cmd_t;

typedef struct {
    uint8_t  state, flags, reason, ack_seq;
    uint16_t nonce;
    uint32_t t_ms;
    int16_t  h_dm, vz_cms, roll_cdeg, pitch_cdeg, yaw_cdeg;
    uint16_t tilt_cdeg, throttle_milli, vcell_mv, loop_max_us;
    uint8_t  phase;             /* TELEM v2 / v3 only (lp_pack_telem ignores it): LP_PHASE_* */
    int16_t  h_tgt_dm;          /* TELEM v2 / v3 only: altitude target, 0.1 m */
    uint8_t  gps_flags;         /* TELEM v3 only: core/gps_nav.h GPS_F_* */
    uint8_t  num_sv;            /* TELEM v3 only */
    uint8_t  hacc_dm;           /* TELEM v3 only: 0.1 m, LP_HACC_NONE = none */
    uint8_t  pos_mode;          /* TELEM v3 only: POS_MODE_* */
    uint16_t dist_dm;           /* TELEM v3 only: 0.1 m from the hold point, LP_DIST_NONE = no hold point */
} lp_telem_t;
/* (the TELEM v4 / v5 power tail is not part of lp_telem_t: core/link_proto_v09.h lp_encode_telem4 / 5 take its two bytes
 * from the snapshot, lp_unpack_telem4 / 5 return them separately - lp_telem_t keeps its size in every build) */

typedef struct {
    uint8_t veh, type, seq, len;              /* len = payload bytes */
    uint8_t payload[LP_MAX_PAYLOAD_LEN];
} lp_frame_t;

uint16_t lp_crc16(const uint8_t *data, uint8_t n, uint16_t crc);
bool     lp_seq_newer(uint8_t a, uint8_t b);   /* 1 <= (a - b) mod 256 <= 127 */
/* round_half_away(x * scale) saturated to [lo, hi]; NaN -> 0; exact for every float (scale's odd part < 256) */
int32_t  lp_quantize(float x, uint16_t scale, int32_t lo, int32_t hi);
/* v0.8.1: the same, exact for EVERY scale (a 64-bit product; the glink tilt scale 10000 has the odd part 625) - a separate
 * function so the images that never call it (the ballistic builds) keep their bytes */
int32_t  lp_quantize_wide(float x, uint16_t scale, int32_t lo, int32_t hi);

void     lp_pack_cmd(const lp_cmd_t *c, uint8_t out[LP_CMD_PAYLOAD_LEN]);
void     lp_unpack_cmd(const uint8_t in[LP_CMD_PAYLOAD_LEN], lp_cmd_t *c);
void     lp_pack_telem(const lp_telem_t *t, uint8_t out[LP_TELEM_PAYLOAD_LEN]);      /* TELEM v1: 28 bytes */
void     lp_unpack_telem(const uint8_t in[LP_TELEM_PAYLOAD_LEN], lp_telem_t *t);    /* phase, h_tgt_dm := 0 */
void     lp_pack_telem2(const lp_telem_t *t, uint8_t out[LP_TELEM2_PAYLOAD_LEN]);    /* TELEM v2: v1 + phase + h_tgt */
void     lp_unpack_telem2(const uint8_t in[LP_TELEM2_PAYLOAD_LEN], lp_telem_t *t);   /* the v3 fields := 0 */
void     lp_pack_telem3(const lp_telem_t *t, uint8_t out[LP_TELEM3_PAYLOAD_LEN]);    /* TELEM v3: v2 + the GPS fields */
void     lp_unpack_telem3(const uint8_t in[LP_TELEM3_PAYLOAD_LEN], lp_telem_t *t);
/* complete frame into out (>= LP_FRAME_OVERHEAD + LP_HEADER_BYTES + n bytes); returns its length, 0 if n is too big */
uint8_t  lp_encode_frame(uint8_t veh, uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t n, uint8_t *out);
uint8_t  lp_encode_cmd(uint8_t veh, uint8_t seq, const lp_cmd_t *c, uint8_t out[LP_CMD_FRAME_BYTES]);
uint8_t  lp_encode_telem(uint8_t veh, uint8_t seq, const lp_telem_t *t, uint8_t out[LP_TELEM_FRAME_BYTES]);
uint8_t  lp_encode_telem2(uint8_t veh, uint8_t seq, const lp_telem_t *t, uint8_t out[LP_TELEM2_FRAME_BYTES]);
uint8_t  lp_encode_telem3(uint8_t veh, uint8_t seq, const lp_telem_t *t, uint8_t out[LP_TELEM3_FRAME_BYTES]);

/* ---- streaming decoder ---- */
#define LP_ACCEPT_CMD 0x01u
#define LP_ACCEPT_TELEM 0x02u
#define LP_ACCEPT_TELEM2 0x04u                 /* v0.7: the ground (bridge) of a VTOL vehicle accepts TELEM | TELEM2 */
#define LP_ACCEPT_TELEM3 0x08u                 /* v0.8: the ground of a VTOL + GPS vehicle also accepts TELEM3 */
#define LP_ACCEPT_TELEM4 0x10u                 /* v0.9 RP1: the ground of a tether / hybrid vehicle: TELEM4 */
#define LP_ACCEPT_TELEM5 0x20u                 /* v0.9 RP1: ... and TELEM5 (the power builds with the GPS) */
#define LP_ACCEPT_TELEM6 0x40u                 /* 2026-11-10: the ground of the carrier battery build: TELEM6 */
#define LP_ACCEPT_TELEM7 0x80u                 /* 2026-11-10: ... and TELEM7 (with the GPS) */
#define LP_ANY_VEHICLE (-1)
#define LP_DEC_QUEUE (LP_FRAME_MAX_BYTES + 4u)  /* replay queue; <= LP_FRAME_MAX_BYTES + 1 are ever pending */
#define LP_DEC_MAX_OUT 4u                      /* frames one input byte can complete (<= 46 replayed bytes, >= 13 per frame) */

typedef struct {
    uint16_t frames, bad_len, bad_crc, wrong_vehicle, wrong_type, bad_payload_len, skipped_bytes;
    uint16_t overflow;                         /* firmware only: frames beyond max_out / queue overrun (0 in tests) */
} lp_dec_stats_t;

typedef struct {
    int16_t  vehicle_id;                       /* LP_ANY_VEHICLE or 0..255 */
    uint8_t  accept_mask;                      /* LP_ACCEPT_* */
    uint8_t  state, n, need;
    uint8_t  buf[LP_FRAME_MAX_BYTES];
    uint8_t  q[LP_DEC_QUEUE];
    uint8_t  q_head, q_len;
    lp_dec_stats_t stats;
} lp_decoder_t;

void    lp_decoder_init(lp_decoder_t *d, int16_t vehicle_id, uint8_t accept_mask);
void    lp_decoder_reset(lp_decoder_t *d);     /* drop any partial candidate (stats kept) */
/* feeds one byte; writes up to max_out completed valid frames to out, returns how many */
uint8_t lp_decoder_feed(lp_decoder_t *d, uint8_t byte, lp_frame_t *out, uint8_t max_out);

#endif /* FC_LINK_PROTO_H */
