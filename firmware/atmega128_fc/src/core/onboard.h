/* onboard.h - onboard supervisor (SPEC 8.8 ballistic, v0.4; SPEC 8.9 VTOL, v0.7 / v0.7.1): command executor + safety
 * interlocks, a tick-for-tick port of sim/onboard.py OnboardSupervisor (portable C99, float32, integer control
 * ticks for every timer), and the FCU's per-tick stack (sim/onboard.py FcuStack: onb_stack_step; VTOL v0.7.1 order
 * attitude -> vanes -> supervisor -> vertical controller -> ESC, see onb_stack_t).
 *
 * Two builds, chosen by onb_params_t.mode (params.json mission.mode -> params_gen.h MISSION_MODE ->
 * fc_default_onboard).  A firmware build may also pin the mode at compile time with -D ONB_FIXED_MODE=LP_MODE_VTOL
 * (or LP_MODE_BALLISTIC): onboard_init then forces p.mode and the compiler drops the other build's code.  The host
 * test and tools/crosscheck_controller.py run both builds through the same object code (mode at run time).
 *
 * BALLISTIC (v0.4 - v0.6.3, bit-identical): the vehicle flies attitude control only.  Launch, throttle, motor
 * cut and nose release are ground commands (LoRa, core/link_proto.h); this module decides which of them to
 * execute and applies the interlocks that no command can disable:
 *
 *   SAFE      motor off, vanes centred, release latched                    (boot state)
 *   ARMED     on the pad, motor off, vanes centred, estimator aligning
 *   POWERED   throttle target > 0, attitude control on, throttle slewed 0 -> 1 in ramp_ticks
 *   UNPOWERED throttle 0 after POWERED; attitude control for coast_hold_ticks, then vanes centred
 *   RELEASED  release servo open; motor off until power cycle
 *   ABORT     motor off now; release after abort_release_ticks, then RELEASED
 *
 * onboard_update() once per control tick, in the reference's order:
 *   1. IMU / baro failure counters
 *   2. received CMD frames, oldest first: every valid frame refreshes the link timer and
 *      gets a reply; a SEQ that is not newer than the last processed one is only
 *      acknowledged; a newer NONE adopts the SEQ; any other newer command is executed
 *      or rejected (a rejected command consumes its SEQ)
 *   3. interlocks (the first that applies acts): arm switch open (= KILL), I9, I1p, I1,
 *      I2, I3, I8, I5, I4, I7, I6 - after the commands, so an interlock has the last word
 *   4. release timer -> RELEASED;  5. throttle slew (+1/ramp per tick up, decreases
 *      immediate) and the I7 powered-time counter
 *   ON / OFF / UP / DOWN (8..11) are answered LP_REJ_UNKNOWN_CMD, exactly as v0.6.3 did.
 *
 * VTOL (v0.7, SPEC 8.9; the user's primary goal: vertical take-off and landing, the ground sends only ON / OFF /
 * UP / DOWN, the vehicle flies the altitude itself).  The supervisor decides WHEN the motor may run and the
 * vertical PHASE; core/vertical_ctrl decides HOW MUCH throttle (onb_stack_step):
 *
 *   SAFE      phase NONE    motor off, vanes centred                                  (boot state)
 *   ARMED     phase SPOOL   buzzer on, motor off for spool_ticks; then the take-off gate (tilt, V_cell; V7)
 *   POWERED   phase CLIMB   motor driven by the vertical controller, climb to the target (hover_dm at ON);
 *                           CLIMB -> HOLD on the controller's at_target of the previous tick
 *             phase HOLD    hold the target; UP / DOWN move it by alt_step_dm within [alt_min_dm, alt_max_dm]
 *             phase LAND    descend until the controller's touchdown of the previous tick -> UNPOWERED
 *   UNPOWERED phase LANDED  motor off, vanes centred; SAFE after safe_after_landed_ticks
 *   ABORT / RELEASED        as in the ballistic build (phase NONE)
 *
 *   commands: ON (SAFE: nonce, every ARM gate, motor time < endurance -> ARMED), OFF (ARMED -> SAFE, CLIMB /
 *   HOLD -> LAND, LAND / LANDED: acknowledged, no change), UP / DOWN (CLIMB / HOLD: target +- step, REJ_ALT_LIMIT
 *   at the limits, REJ_LANDING in LAND), KILL (as ballistic); ARM / DISARM / LAUNCH / THROTTLE / CUT / RELEASE:
 *   LP_REJ_MODE (before "release pending").
 *   rules, after the commands, the first that applies acts (each rule's rationale is next to it in onboard.c and
 *   in sim/onboard.py): arm switch (ARMED -> SAFE, POWERED -> ABORT, LANDED -> SAFE), I9 IMU (ARMED / LANDED ->
 *   SAFE, POWERED -> ABORT), I9 baro (CLIMB / HOLD -> LAND on the NAV's dead reckoning, baro_lost latched until the
 *   next ON; v0.7.1: the tick it acts fixes the baro-lost time limit from h), I1p (pad), I2, I3, V4 geofence and V5
 *   fall (POWERED -> ABORT, LANDED -> SAFE: v0.7.1 landed_rules_note, no release toward people at a landed
 *   vehicle), V6 hard cutoff (ABORT); v0.7.1 in POWERED LAND without a confirmed touchdown: baro lost and the time
 *   limit reached -> V8 (ground signature: LANDED) / V9 (none: ABORT), baro healthy and the controller low and
 *   leading for land_timeout_ticks -> V10 (LANDED, motor off, no release); then in CLIMB / HOLD only V1 link loss,
 *   I8 V_cell, V3 ceiling, V2 flight time (LAND); then the release timer, then the VTOL timers (spool end + V7, CLIMB
 *   -> HOLD, touchdown -> LANDED, LANDED -> SAFE); I10 boots ABORT.  Motor time is cumulative POWERED ticks since boot.
 *   The altitude target is integer decimetres (TELEM v2 unit), so UP / DOWN and their limits are exact.
 *
 * v0.8 VTOL build with GPS position hold (onb_params_t.gps_hold; params mission.vtol.position_hold; sim/onboard.py
 * "v0.8 VTOL build with GPS position hold"): ON gate after "aligned": the NAV's GATE_OK of the newest fresh complete GPS
 * slot set (in->gps_gate_ok, onb_stack_step) -> else LP_REJ_GPS (GPS needs open sky: indoors the ON is refused; the build
 * without position hold is the indoor fallback).  LAND rules after V2 in CLIMB / HOLD, from the position controller's
 * published requests (in->pc_requests, as of the previous tick): V11 GPS lost (R_V_GPS_LOST), V12 fence (R_V_FENCE), V13
 * GPS implausible (R_V_GPS_IMPLAUSIBLE), V14 authority (R_V_AUTHORITY) - never ABORT near people.
 * onb_params_t.takeoff_check: V15 take-off check (POWERED CLIMB / HOLD, after V6): the vertical controller's takeoff_fail
 * of the previous tick (VC_SIG_TAKEOFF_FAIL) -> SAFE (R_V_TAKEOFF_CHECK): motor off now, no release.  TELEM v3 (gps_hold
 * builds): the v2 fields + gps_flags, num_sv, hacc_dm, pos_mode, dist of the previous tick (in->tlm_gps).
 *
 * v0.8.1 four-chip build (params flight_computer.architecture_v081; sim/onboard.py "FCU tick order v0.8.1"): the GNS
 * board runs the GPS filter and the position controller; the FCU's stack (chain ONB_CHAIN_GNS) takes the nlink v4 GNS slot
 * (core/nlink_proto.h nl_gns_rx_*): in->gps_gate_ok = its GATE_OK, in->pc_requests = the GNS's latched requests | the FCU's
 * own GNS-loss request (no TILT part for land_steps cycles -> V11), in->tlm_gps from the newest set; no position step -
 * the pflags / k_T of the tick go back in the nlink v4 FCU frame (onb_stack_fcu4); the tilt-setpoint builder follows the
 * slot's TILT (< 2 missed cycles), else the newest set's TRIM.  The rules V11-V15 and their reason codes are unchanged.
 *
 * Every comparison uses the float32 thresholds of params_gen.h; the only arithmetic
 * on sensor values is |w_xy|^2 (I3) and h + k max(v_z,0)^2 (I4, ballistic), so the C supervisor
 * decides exactly like the float64 reference except within float32 rounding of
 * those two thresholds (tools/crosscheck_controller.py keeps its random inputs out
 * of that band and reports how many it moved).
 *
 * Invariant: no command can disable an interlock; every boot is SAFE with the motor
 * off; an in-flight watchdog/brown-out reset (I10) boots SAFE with the release open
 * and arming refused until a power cycle (ballistic) or ABORT with the motor off (VTOL).
 */
#ifndef FC_ONBOARD_H
#define FC_ONBOARD_H

#include "fcmath.h"
#include "link_proto.h"
#include "nlink_proto.h"
#include "position_ctrl.h"
#include "vertical_ctrl.h"

#define ONB_MAX_FRAMES 4u                    /* CMD frames handled per tick */
#define ONB_MAX_TRANS (ONB_MAX_FRAMES + 3u)  /* one per command + one rule + the release timer + one VTOL timer */

/* exec result codes besides 0 (executed) and LP_REJ_*: frames that were not commands */
#define ONB_EXEC_NONE 0xFEu                  /* newer NONE: SEQ adopted, REJECTED flag cleared */
#define ONB_EXEC_REPEAT 0xFFu                /* SEQ not newer than the last processed one: acknowledged only */
/* transition causes: 0x80 | cmd for a command, the interlock reason code (1..63), the release timer, a VTOL timer */
#define ONB_CAUSE_CMD 0x80u
#define ONB_CAUSE_RELEASE_TIMER 0x7Eu
#define ONB_CAUSE_SPOOL_DONE 0x7Du           /* VTOL: spool warning complete, take-off gate passed -> POWERED (CLIMB) */
#define ONB_CAUSE_AT_TARGET 0x7Cu            /* VTOL: the controller's reference reached the target -> HOLD */
#define ONB_CAUSE_TOUCHDOWN 0x7Bu            /* VTOL: touchdown confirmed -> UNPOWERED (LANDED) */
#define ONB_CAUSE_LANDED_TIMER 0x7Au         /* VTOL: LANDED for safe_after_landed_ticks -> SAFE */
#define ONB_NO_FRAME_TICKS 0x40000000UL      /* "ticks since the last frame" before the first frame (1 << 30) */

/* RAM trimming (v0.7.1): a firmware build that pins the mode (ONB_FIXED_MODE) carries only that mode's parameters and
 * state - the VTOL FCU drops the ballistic-only fields (55 B of static RAM), the ballistic FCU the VTOL-only parameters
 * (58 B) and the vertical controller of onb_stack_t (196 B).  Without ONB_FIXED_MODE (host test, cross-check) both sets exist.  The trimmed fields were dead in that build
 * anyway (written at most, never read), so the behaviour is unchanged. */
#if !defined(ONB_FIXED_MODE) || ONB_FIXED_MODE == LP_MODE_BALLISTIC
#define ONB_HAS_BALLISTIC 1
#else
#define ONB_HAS_BALLISTIC 0
#endif
#if !defined(ONB_FIXED_MODE) || ONB_FIXED_MODE == LP_MODE_VTOL
#define ONB_HAS_VTOL 1
#else
#define ONB_HAS_VTOL 0
#endif
/* v0.8: the GPS position hold of the VTOL build (slot receiver, position controller, tilt builder: ~0.4 KB of static RAM);
 * -D ONB_NO_GPS drops it from a VTOL firmware built without position hold (the indoor fallback, gps_hold = false) */
#if ONB_HAS_VTOL && !defined(ONB_NO_GPS)
#define ONB_HAS_GPS 1
#else
#define ONB_HAS_GPS 0
#endif
/* v0.8.1: the two chains of the GPS build (onb_stack_init's chain).  ONB_CHAIN_GNS = the four-chip FCU (the GNS slot
 * receiver, no position controller: the flight build); ONB_CHAIN_FCU = the v0.8 three-chip FCU (the slot set + the
 * position controller as background pieces: the comparison build).  Host builds carry both; an AVR firmware carries the
 * GNS chain only (-D ONB_V08_FCU_CHAIN builds the v0.8 FCU instead). */
#define ONB_CHAIN_FCU 0u
#define ONB_CHAIN_GNS 1u
#if ONB_HAS_GPS && !(defined(__AVR__) && defined(ONB_V08_FCU_CHAIN))
#define ONB_HAS_GNS 1
#else
#define ONB_HAS_GNS 0
#endif
#if ONB_HAS_GPS && (!defined(__AVR__) || defined(ONB_V08_FCU_CHAIN))
#define ONB_HAS_FCU_POS 1
#else
#define ONB_HAS_FCU_POS 0
#endif
/* v0.8.1 RAM diet (the FCU keeps >= 1024 B of stack free): a VTOL AVR build keeps the supervisor's parameters in flash -
 * onboard_init / onb_stack_init take fc_onboard_params() (a flash address, core/fc_config.c; its gps_hold is off in an
 * ONB_NO_GPS build) and keep the pointer; every read is an LPM (FC_FLASH_RD).  Host builds and the ballistic FCU copy them
 * (the ballistic image keeps its bytes). */
#if defined(__AVR__) && ONB_HAS_VTOL && !ONB_HAS_BALLISTIC
#define ONB_PARAMS_FLASH 1
#else
#define ONB_PARAMS_FLASH 0
#endif

typedef enum {
    ONB_SAFE = LP_STATE_SAFE, ONB_ARMED = LP_STATE_ARMED, ONB_POWERED = LP_STATE_POWERED,
    ONB_UNPOWERED = LP_STATE_UNPOWERED, ONB_RELEASED = LP_STATE_RELEASED, ONB_ABORT = LP_STATE_ABORT
} onb_state_t;

typedef struct {                             /* sim.onboard.SupervisorParams (times in control ticks) */
    uint16_t loop_hz;
    float    tilt_abort_deg, arm_tilt_max_deg, min_cell_v;
    uint32_t link_disarm_ticks, abort_release_ticks;
    float    tilt_rate_abort_sq;             /* (rad/s)^2 */
    float    spin_rate_abort;                /* rad/s */
    uint32_t rate_persist_ticks;
    bool     gyro_saturation_abort;
    uint32_t imu_fail_ticks, baro_fail_ticks;
    uint8_t  mode;                           /* LP_MODE_BALLISTIC / LP_MODE_VTOL */
#if ONB_HAS_BALLISTIC
    /* ---- the ballistic build only ---- */
    uint16_t climb_milli;
    uint32_t ramp_ticks, coast_hold_ticks;
    float    geofence_m, apogee_limit_m;
    float    apogee_k;                       /* coast_drag_factor / (2 g) */
    float    fall_vz_mps;
    uint32_t fall_confirm_ticks, link_abort_ticks, max_motor_ticks;
    uint32_t release_after_cut_ticks;
#endif
#if ONB_HAS_VTOL
    /* ---- v0.7 VTOL build (mission.vtol) only ---- */
    uint32_t spool_ticks;                    /* spool_warning_s */
    int16_t  hover_dm, alt_step_dm, alt_min_dm, alt_max_dm;   /* [0.1 m] */
    uint32_t link_land_ticks;                /* V1 link_loss_land_s */
    uint32_t endurance_ticks;                /* V2 / REJ_ENDURANCE: max_flight_s - land_reserve_s */
    uint32_t hard_cutoff_ticks;              /* V6 hard_cutoff_s */
    float    ceiling_m;                      /* V3 */
    float    v_geofence_m;                   /* V4 vtol.geofence_m */
    float    v_fall_vz_mps;                  /* V5 vtol.fall_vz_mps */
    uint32_t v_fall_confirm_ticks;           /* V5 vtol.fall_confirm_s */
    uint32_t safe_after_landed_ticks;        /* LANDED -> SAFE */
    /* ---- v0.7.1 (VTOL) ---- */
    uint32_t land_timeout_ticks;             /* V10 vtol.touchdown.land_floor_timeout_s; 0 = off (<= 65535: the counter) */
    uint32_t baro_limit_base_ticks;          /* V8 / V9: ticks_ge(factor flare / touchdown_rate + margin) */
    uint32_t baro_limit_mt_per_dm;           /* V8 / V9: round(factor loop_hz / (10 descent_rate) x 1000) milli-ticks/dm */
    int16_t  v_geofence_dm;                  /* vtol.geofence_m [0.1 m]: the clamp of h at the baro loss */
    /* ---- v0.8 (VTOL) ---- */
    bool     gps_hold;                       /* GPS position hold: ON gate, V11-V14, TELEM v3 (position_hold.enabled) */
    bool     takeoff_check;                  /* V15 (position_hold.takeoff_check_ASSUMED) */
#if PM_ANY
    pm_params_t pw;                          /* v0.9 power rules (core/power_mon.h; pw.build: the power build) - the
                                              * hover / alt / ceiling / geofence / endurance / hard-cutoff fields above
                                              * are the tethered limits in a wired build (fc_config.c, tether_limits) */
#endif
#endif
} onb_params_t;

typedef struct {
    uint8_t  seq;
    lp_cmd_t cmd;
} onb_frame_t;

typedef struct {                             /* v0.8 TELEM v3 GPS fields (sim.onboard.FcuStack._tlm_gps) */
    uint8_t  flags, num_sv, hacc_dm, pos_mode;
    float    dist_m;                         /* NaN = no hold point */
} onb_tlm_gps_t;

typedef struct {                             /* sim.onboard.OnboardInputs */
    uint32_t tick;
    const onb_frame_t *frames;               /* received since the previous tick, oldest first */
    uint8_t  n_frames;
    float    h_m, vz_mps, tilt_deg;
    float    gyro[3];                        /* bias-corrected body rates, rad/s */
    bool     gyro_saturated;
    float    v_cell;
    bool     imu_ok, baro_ok, aligned, arm_switch;
    float    roll_deg, pitch_deg, yaw_deg, loop_max_us;   /* telemetry only */
    /* v0.7 VTOL build (ignored by the ballistic build; onb_stack_step fills them): the vertical controller's flags
     * of the PREVIOUS tick and the ESC throttle applied in the previous tick (the TELEM v2 throttle) */
    bool     vc_touchdown, vc_at_target;
    float    throttle_prev;
    bool     vc_ground_signature;            /* v0.7.1: support or floor present (V8 / V9 baro-lost time limit) */
    uint16_t vc_low_in_land_ticks;           /* v0.7.1: consecutive LAND ticks low, leading, baro healthy (V10) */
    /* VTOL NAV frame fields for the vertical controller and the position hold (onb_stack_step; the supervisor does not
     * read them).  v0.8: cos(tilt) replaces the v0.7.1 quaternion (12 B less on the FCU stack): the nlink v3 frame's
     * cos_tilt; a v2 frame (NL_TYPE_NAV, the build without GPS): vc_cos_tilt_from_q(q) - the v0.7.1 arithmetic; no
     * valid frame: 1 (the v0.7.1 identity quaternion); NaN is taken as 1 by both controllers */
    float    cos_tilt;
    float    accel_z_mps2;                   /* NAV body-z specific force (nlink accel[2]); NaN = none (no support) */
    /* v0.8 VTOL + GPS (onb_stack_step fills all but gps_slot) */
    const uint8_t *gps_slot;                 /* NL_SLOT_LEN bytes of this tick's valid nlink v3 frame, NULL = none */
    bool     gps_gate_ok;                    /* ON gate: GATE_OK of the newest fresh complete slot set */
    uint8_t  pc_requests;                    /* the position controller's published POS_REQ_* (previous tick) */
    bool     vc_takeoff_fail;                /* the vertical controller's take-off check of the previous tick */
    onb_tlm_gps_t tlm_gps;                   /* TELEM v3 (as of the previous tick) */
#if PM_ANY
    pm_inputs_t pw;                          /* v0.9: the power ADC codes of the previous tick + the buffer-current sign */
    bool     vc_takeoff_passed;              /* v0.9: the vertical controller's take-off check passed (previous tick; true */
                                             /* without a take-off check), its support signature and lift-off latch: */
    bool     vc_support, vc_airborne;        /* a power rule on the pad stops the motor (SAFE), elsewhere it lands */
#endif
} onb_inputs_t;

typedef struct {                             /* everything the TELEM reply needs, captured in the tick */
    uint8_t  state, flags, reason, ack_seq;
    uint16_t nonce, throttle_milli;          /* VTOL: the previous tick's ESC throttle, quantised (x1000) */
    uint32_t tick;
    float    h_m, vz_mps, roll_deg, pitch_deg, yaw_deg, tilt_deg, v_cell, loop_max_us;
    bool     v2;                             /* VTOL build: TELEM v2 / SNAP2 (phase, h_tgt_m valid) */
    uint8_t  phase;
    float    h_tgt_m;                        /* the target in SPOOL / CLIMB / HOLD, else 0 */
    bool     v3;                             /* v0.8 VTOL + GPS build: TELEM v3 / SNAP3 (the GPS fields valid) */
    uint8_t  gps_flags, num_sv, hacc_dm, pos_mode;
    float    dist_m;                         /* NaN = no hold point (TELEM v3 dist 0xFFFF) */
#if PM_ANY
    uint8_t  pw_vaux, pw_flags;              /* v0.9 TELEM v4 / v5 tail (PROPOSED; sim/power_link.py PowerTelem: 0.125 V,
                                              * PF_* | build << 6); flags bits 6-7 = 0: the battery build, no tail */
#endif
} onb_tlm_src_t;

typedef struct {
    uint8_t seq, cmd, code, state_before, state_after;
    uint8_t phase_before, phase_after;       /* VTOL (ballistic: 0) */
} onb_exec_t;

/* one state transition (from != to) or, VTOL only, one phase change inside POWERED (from == to == POWERED), in the
 * order they happened; the phases are 0 in the ballistic build */
typedef struct {
    uint8_t from, to, cause;
    uint8_t phase_from, phase_to;
} onb_trans_t;

typedef struct {                             /* sim.onboard.OnboardOutputs */
    uint8_t  state;
    uint16_t target_milli, throttle_milli;   /* ballistic; VTOL: 0 (the ESC follows the vertical controller) */
    bool     control_active, release;
    uint8_t  reason, flags, ack_seq;
    uint16_t nonce;
    bool     reply;                          /* answer the uplink with SEQ reply_seq */
    uint8_t  reply_seq;
    onb_tlm_src_t tlm;                       /* valid when reply */
    uint8_t  interlock;                      /* reason code of the interlock that acted this tick, 0 = none */
    uint8_t  n_exec, n_trans;
    onb_exec_t  exec[ONB_MAX_FRAMES];
    onb_trans_t trans[ONB_MAX_TRANS];
    /* v0.7 VTOL build (ballistic: phase NONE, target 0, motor_on = POWERED, buzzer / baro_lost false) */
    uint8_t  phase;                          /* LP_PHASE_* */
    int16_t  h_tgt_dm;                       /* altitude target [0.1 m] in SPOOL / CLIMB / HOLD, else 0 (the vertical
                                              * controller's target; in metres in tlm.h_tgt_m of a reply) */
    bool     motor_on;                       /* state POWERED: the ESC may follow the vertical controller */
    bool     buzzer;                         /* VTOL spool warning (ARMED) */
    bool     baro_lost;                      /* land on the inertial vertical speed */
} onb_outputs_t;

typedef struct {
#if ONB_PARAMS_FLASH
    const onb_params_t *p;                   /* flash (fc_onboard_params) */
#else
    onb_params_t p;                          /* a copy */
#endif
    uint8_t  state;
    uint32_t t_entry;
    uint16_t nonce;
    bool     has_seq;
    uint8_t  last_seq;
    bool     ack_rejected;
    bool     has_frame;
    uint32_t last_frame_tick;
    uint8_t  reason;
    bool     interlock_latched;
#if ONB_HAS_BALLISTIC
    uint16_t target_milli, out_milli;        /* the ballistic throttle slew */
    uint32_t level;
    bool     has_unpowered;                  /* the ballistic coast hold */
    uint32_t t_unpowered;
#endif
    uint32_t powered_ticks;
    bool     release_open;
    bool     has_release_deadline;
    uint32_t release_deadline;
    uint32_t rate_count, fall_count, imu_fail, baro_fail;
    onb_outputs_t *out;                      /* the outputs of the update in progress (transition log) */
    /* v0.7 VTOL build */
    uint8_t  phase;
    int16_t  tgt_dm;                         /* altitude target [0.1 m] */
    bool     baro_lost;                      /* latched baro failure in flight (until the next ON) */
    uint32_t baro_lost_tick;                 /* v0.7.1: tick the baro-lost landing started (V8 / V9) */
    uint32_t baro_limit;                     /* v0.7.1: its time limit [ticks] from the altitude at that tick */
#if PM_ANY
    pm_t     pm;                             /* v0.9 power monitor (core/power_mon.h) */
#if PM_BUF
    bool     lost_noted;                     /* W_LOST during LAND: its reason latched once */
#endif
#endif
} onboard_t;

uint16_t onb_lfsr16_next(uint16_t s);      /* Galois LFSR, taps 0xB400, 0 -> 0xACE1 */
/* AVR VTOL build: p = fc_onboard_params() (flash, kept as a pointer; core/onboard.h ONB_PARAMS_FLASH) */
void     onboard_init(onboard_t *s, const onb_params_t *p, uint16_t seed, bool reset_in_flight);
uint16_t onboard_loop_hz(const onboard_t *s);       /* p.loop_hz (the TELEM t_ms scale) */
void     onboard_update(onboard_t *s, const onb_inputs_t *in, onb_outputs_t *out);
bool     onboard_control_active(const onboard_t *s, uint32_t tick);
bool     onboard_link_ok(const onboard_t *s, uint32_t tick);
bool     onboard_vtol(const onboard_t *s);
#if PM_WIRED
bool     onboard_u_capped(const onboard_t *s);   /* v0.9 W_SAG: the ceiling for the vertical controller is on */
#endif   /* the VTOL build (p.mode, or the compile-time ONB_FIXED_MODE) */
/* TELEM payload fields from a captured source (quantisation in lp_quantize; t_ms = tick * 1000 / loop_hz; a v2
 * source also fills phase and h_tgt_dm, a v1 source sets them 0) */
void     onboard_telemetry(const onb_tlm_src_t *src, uint16_t loop_hz, lp_telem_t *t);
/* the same in two halves (part 0: header, t_ms, h, v_z, roll; part 1: the rest) so the firmware can
 * spread the quantisation over two background steps */
void     onboard_telemetry_part(const onb_tlm_src_t *src, uint16_t loop_hz, lp_telem_t *t, uint8_t part);
/* the reply frame of a captured source: TELEM (v1 source, 36 bytes), TELEM2 (v2 source, 39 bytes) or TELEM3 (v3 source,
 * 45 bytes); returns its length; out has room for LP_FRAME_MAX_BYTES */
uint8_t  onboard_telemetry_frame(const onb_tlm_src_t *src, uint16_t loop_hz, uint8_t veh, uint8_t seq, uint8_t *out);
/* the states in which a watchdog / brown-out reset counts as "in flight" (I10): POWERED, UNPOWERED, ABORT, RELEASED
 * (ballistic, v0.6.3).  onboard_flying_mode: the same for a build; VTOL leaves UNPOWERED out - it is LANDED, on the
 * ground with the motor off, and an I10 boot there would ABORT and fire the release at the people handling it. */
bool     onboard_flying(uint8_t state);
bool     onboard_flying_mode(uint8_t mode, uint8_t state);

/* V8 / V9 (v0.7.1): ticks from the baro loss to the time limit, from the NAV altitude at the loss, integer arithmetic
 * identical to sim/onboard.py SupervisorParams.baro_limit_ticks: base + ceil(h_dm x mt_per_dm / 1000), h_dm =
 * round_half_away(h x 10) clamped to [0, v_geofence_dm]; a non-finite h takes v_geofence_dm (the longest limit) */
#if ONB_HAS_VTOL
uint32_t onboard_baro_limit_ticks(const onb_params_t *p, float h_m);
#endif

/* ---- the FCU's per-tick stack (sim/onboard.py FcuStack, the normative order) ----
 * VTOL (v0.7.1, params.json mission.vtol.fcu_tick_order_note): the caller runs (1) the attitude controller FIRST, with
 * this tick's NAV frame and onb_stack_att_active(k) / onb_stack_att_u(k) - the PREVIOUS tick's supervisor
 * control_active and ESC throttle (vanes centred when not active; the throttle feeds the gain schedule) - and (2)
 * writes the vane registers; then onb_stack_step: (3) the supervisor, given the vertical controller's touchdown /
 * at_target / ground signature / low_in_land_ticks of the previous tick and the ESC throttle applied in the previous
 * tick (in->vc_* and throttle_prev are overwritten), (4) the vertical controller with this tick's phase, target,
 * baro_lost, NAV altitude, v_z, cos(tilt) from in->q_nav and the body-z specific force in->accel_z_mps2 (k->vo),
 * (5) returns the ESC command = the controller's u while motor_on (state POWERED), else 0 - for the ESC register.
 * Why (the v0.7 integration measured it): supervisor + vertical controller ahead of the attitude controller pushed
 * sample -> vane PWM to 5.15 ms against the 6-DOF's 4.71 ms.  A supervisor decision reaches the ESC in the same tick and
 * the vanes one tick later; the gain schedule sees u one tick old.
 * Ballistic (unchanged, v0.6.3 order): onb_stack_step = the supervisor only, returns its slewed throttle; the attitude
 * controller then runs with this tick's so->control_active and that throttle.
 * v0.8 VTOL + GPS (p.gps_hold; sim/onboard.py "FCU tick order v0.8"): (0) the GPS slot of this tick's NAV frame into the
 * slot receiver (in->gps_slot; in->gps_gate_ok, in->pc_requests, in->tlm_gps are filled from it and the published
 * position outputs); (1) attitude controller with the previous tick's control_active, ESC throttle AND tilt setpoint
 * (onb_stack_att_tilt: attitude_ctrl_set_tilt in the background of the previous tick); (2) vanes; (3) supervisor;
 * (4) vertical controller (in->cos_tilt); (5) ESC; (6) the position step when due (slot part 4 received this tick, or 5
 * ticks since the previous step): its inputs latched now, its pieces P0..P10 in the background of this and the next four
 * ticks (core/position_ctrl.h, each <= 0.2 ms), outputs published POS_PIPELINE_TICKS later; (7) the tilt setpoint
 * builder on the published target - onb_stack_att_tilt is the tilt for the NEXT tick's attitude controller.
 * The firmware splits the tick: onb_stack_rt = (0), (3)..(5) and the latch of (6) - the time-critical part, its return
 * value goes to the ESC register; onb_stack_bg = ONE background piece per call (the position pieces of this tick, then
 * the publication and the builder (7)); it returns true while pieces are left for this tick.  The caller then hands
 * onb_stack_att_tilt to attitude_ctrl_set_tilt (in the background too).  Pieces still left when the next tick starts
 * (onb_stack_bg_pending) must be finished BEFORE that tick's attitude controller (they are counted in bg_late; the rt
 * part also finishes them first, so the numbers never change).  onb_stack_step = onb_stack_rt + every piece at once:
 * what the host tests and the cross-checks run - the same numbers as sim.onboard.FcuStack.
 * v0.8.1 GNS chain (ONB_CHAIN_GNS, sim FcuStack(chain="gns")): (0) the GNS slot of this tick's NAV frame (in->gps_slot of
 * an NL_TYPE_NAV4 frame) into nl_gns_rx; the supervisor's GPS inputs from it (gate, requests incl. the FCU's own GNS-loss
 * request, TELEM v3); (1)-(5) unchanged; (6) no position step: the pflags / k_T of this tick (onb_stack_fcu4_*) for the
 * FCU's nlink v4 reply; (7) onb_stack_bg = the builder on the slot's TILT (< 2 missed cycles) or the newest set's TRIM,
 * active while the GNS says engaged and motor_on - one call, no pieces. */
typedef struct {
    onboard_t    sup;
#if ONB_HAS_VTOL                             /* a ballistic fixed build carries no vertical controller (196 B) */
    vc_t         vc;
    vc_outputs_t vo;                         /* the vertical controller's outputs of the latest tick (VTOL) */
    bool         have_vo;                    /* vo holds a tick's outputs (false before the first VTOL tick) */
#endif
#if ONB_HAS_GPS                              /* GPS position hold (VTOL, p.gps_hold) */
    bool         gps;                        /* the position hold is on (p.gps_hold and a pos_params_t given) */
    uint8_t      chain;                      /* ONB_CHAIN_GNS / ONB_CHAIN_FCU */
    float        tilt[2];                    /* the tilt setpoint the next attitude step uses (level frame, rad) */
    float        slew;                       /* the builder's slew per tick (pos_params_t.slew_per_tick) */
    bool         motor_on;                   /* this tick's supervisor motor_on (the builder in the background) */
    bool         bg_left;                    /* this tick's background work is not all done */
    uint16_t     bg_late;                    /* ticks whose background work the next rt part had to finish */
#if ONB_HAS_GNS                              /* v0.8.1 four chips: the GNS slot (nlink v4) */
    nl_gns_rx_t  grx;
    uint16_t     land_steps;                 /* pos_params_t.land_steps: the FCU's own GNS-loss request */
    uint8_t      pflags;                     /* the nlink v4 FCU frame of the latest tick (onb_stack_fcu4) */
    uint16_t     kt_q;
#endif
#if ONB_HAS_FCU_POS                          /* v0.8 three chips: the slot set + the position controller */
    nl_slot_rx_t rx;
    pos_t        pos;
    pos_pipe_t   pipe;
    uint8_t      pos_ticks;                  /* FCU ticks since the last position step */
    bool         sat_hi;                     /* the vertical controller's sat == +1 since the last step */
#endif
#endif
    bool         att_active;                 /* VTOL: control_active of the latest tick (the next attitude step's) */
    float        throttle;                   /* the ESC command of the latest tick */
} onb_stack_t;

/* p, vp, pp: the supervisor's, the vertical controller's and the position hold's parameters - on an AVR VTOL build the
 * flash tables fc_onboard_params(), fc_vertical_params(), fc_position_params() (core/fc_config.h); pp NULL or p->gps_hold
 * false: no position hold.  chain (GPS builds): ONB_CHAIN_GNS (v0.8.1, the four-chip FCU: only pp's land_steps /
 * slew_per_tick are read) or ONB_CHAIN_FCU (v0.8, where compiled: ONB_HAS_FCU_POS); a chain the build lacks takes the one
 * it has. */
void  onb_stack_init(onb_stack_t *k, const onb_params_t *p, const vc_params_t *vp, const pos_params_t *pp, uint8_t chain,
                     uint16_t seed, bool reset_in_flight);
float onb_stack_step(onb_stack_t *k, onb_inputs_t *in, onb_outputs_t *so);   /* = onb_stack_rt + all of onb_stack_bg */
float onb_stack_rt(onb_stack_t *k, onb_inputs_t *in, onb_outputs_t *so);     /* the time-critical part: the ESC value */
bool  onb_stack_bg(onb_stack_t *k);          /* one background piece; true while this tick has pieces left */
/* VTOL: the attitude controller's inputs for the coming tick (before onb_stack_step): the previous tick's supervisor
 * control_active and ESC throttle (false / 0 before the first step) */
static inline bool  onb_stack_att_active(const onb_stack_t *k) { return k->att_active; }
static inline float onb_stack_att_u(const onb_stack_t *k) { return k->throttle; }
#if ONB_HAS_GPS
/* the tilt setpoint for the NEXT tick's attitude controller (attitude_ctrl_set_tilt after the background) */
static inline const float *onb_stack_att_tilt(const onb_stack_t *k) { return k->tilt; }
static inline bool onb_stack_bg_pending(const onb_stack_t *k) { return k->bg_left; }
#endif
#if ONB_HAS_GNS
/* v0.8.1: the nlink v4 FCU frame fields of the latest tick (pflags: phase, motor_on, sat == +1, low_alt, ground, lifted;
 * kt_q = k_T x 16384) - the FCU's reply to the next NAV frame carries them (nl_encode_fcu4) */
static inline uint8_t  onb_stack_fcu4_pflags(const onb_stack_t *k) { return k->pflags; }
static inline uint16_t onb_stack_fcu4_kt_q(const onb_stack_t *k) { return k->kt_q; }
static inline const nl_gns_rx_t *onb_stack_gns(const onb_stack_t *k) { return &k->grx; }
#endif
#if ONB_HAS_FCU_POS
/* v0.8: the position controller's published outputs (FcuStack.pos_out) and its latest computed ones (logs) */
static inline const pos_pub_t *onb_stack_pos_pub(const onb_stack_t *k) { return &k->pipe.pub; }
#if POS_FULL_OUT
static inline const pos_outputs_t *onb_stack_pos_out(const onb_stack_t *k) { return &k->pos.out; }   /* host builds */
#endif
#endif

#endif /* FC_ONBOARD_H */
