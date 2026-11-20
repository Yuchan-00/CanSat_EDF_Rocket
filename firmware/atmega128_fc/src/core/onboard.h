/* onboard.h - onboard supervisor of the rocket (SPEC 8.8 ballistic mission, v0.4): command executor + safety
 * interlocks, a tick-for-tick port of sim/onboard.py OnboardSupervisor (portable C99, float32, integer control
 * ticks for every timer).
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
 * Every comparison uses the float32 thresholds of params_gen.h; the only arithmetic
 * on sensor values is |w_xy|^2 (I3) and h + k max(v_z,0)^2 (I4, ballistic), so the C supervisor
 * decides exactly like the float64 reference except within float32 rounding of
 * those two thresholds (tools/crosscheck_controller.py keeps its random inputs out
 * of that band and reports how many it moved).
 *
 * Invariant: no command can disable an interlock; every boot is SAFE with the motor
 * off; an in-flight watchdog/brown-out reset (I10) boots SAFE with the release open
 * and arming refused until a power cycle.
 */
#ifndef FC_ONBOARD_H
#define FC_ONBOARD_H

#include "fcmath.h"
#include "link_proto.h"
#include "nlink_proto.h"

#define ONB_MAX_FRAMES 4u                    /* CMD frames handled per tick */
#define ONB_MAX_TRANS (ONB_MAX_FRAMES + 3u)  /* one per command + one rule + the release timer + 1 spare (layout kept) */

/* exec result codes besides 0 (executed) and LP_REJ_*: frames that were not commands */
#define ONB_EXEC_NONE 0xFEu                  /* newer NONE: SEQ adopted, REJECTED flag cleared */
#define ONB_EXEC_REPEAT 0xFFu                /* SEQ not newer than the last processed one: acknowledged only */
/* transition causes: 0x80 | cmd for a command, the interlock reason code (1..63), the release timer */
#define ONB_CAUSE_CMD 0x80u
#define ONB_CAUSE_RELEASE_TIMER 0x7Eu
#define ONB_NO_FRAME_TICKS 0x40000000UL      /* "ticks since the last frame" before the first frame (1 << 30) */

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
    uint8_t  mode;                           /* LP_MODE_BALLISTIC (onboard_init forces it) */
    uint16_t climb_milli;
    uint32_t ramp_ticks, coast_hold_ticks;
    float    geofence_m, apogee_limit_m;
    float    apogee_k;                       /* coast_drag_factor / (2 g) */
    float    fall_vz_mps;
    uint32_t fall_confirm_ticks, link_abort_ticks, max_motor_ticks;
    uint32_t release_after_cut_ticks;
} onb_params_t;

typedef struct {
    uint8_t  seq;
    lp_cmd_t cmd;
} onb_frame_t;

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
} onb_inputs_t;

typedef struct {                             /* everything the TELEM reply needs, captured in the tick */
    uint8_t  state, flags, reason, ack_seq;
    uint16_t nonce, throttle_milli;          /* the slewed throttle, quantised (x1000) */
    uint32_t tick;
    float    h_m, vz_mps, roll_deg, pitch_deg, yaw_deg, tilt_deg, v_cell, loop_max_us;
} onb_tlm_src_t;

typedef struct {
    uint8_t seq, cmd, code, state_before, state_after;
} onb_exec_t;

/* one state transition (from != to), in the order they happened */
typedef struct {
    uint8_t from, to, cause;
} onb_trans_t;

typedef struct {                             /* sim.onboard.OnboardOutputs */
    uint8_t  state;
    uint16_t target_milli, throttle_milli;   /* the throttle target and the slewed throttle */
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
} onb_outputs_t;

typedef struct {
    onb_params_t p;                          /* a copy */
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
    uint16_t target_milli, out_milli;        /* the ballistic throttle slew */
    uint32_t level;
    bool     has_unpowered;                  /* the ballistic coast hold */
    uint32_t t_unpowered;
    uint32_t powered_ticks;
    bool     release_open;
    bool     has_release_deadline;
    uint32_t release_deadline;
    uint32_t rate_count, fall_count, imu_fail, baro_fail;
    onb_outputs_t *out;                      /* the outputs of the update in progress (transition log) */
} onboard_t;

uint16_t onb_lfsr16_next(uint16_t s);      /* Galois LFSR, taps 0xB400, 0 -> 0xACE1 */
void     onboard_init(onboard_t *s, const onb_params_t *p, uint16_t seed, bool reset_in_flight);
uint16_t onboard_loop_hz(const onboard_t *s);       /* p.loop_hz (the TELEM t_ms scale) */
void     onboard_update(onboard_t *s, const onb_inputs_t *in, onb_outputs_t *out);
bool     onboard_control_active(const onboard_t *s, uint32_t tick);
bool     onboard_link_ok(const onboard_t *s, uint32_t tick);
/* TELEM payload fields from a captured source (quantisation in lp_quantize; t_ms = tick * 1000 / loop_hz) */
void     onboard_telemetry(const onb_tlm_src_t *src, uint16_t loop_hz, lp_telem_t *t);
/* the same in two halves (part 0: header, t_ms, h, v_z, roll; part 1: the rest) so the firmware can
 * spread the quantisation over two background steps */
void     onboard_telemetry_part(const onb_tlm_src_t *src, uint16_t loop_hz, lp_telem_t *t, uint8_t part);
/* the reply frame of a captured source: TELEM v1 (36 bytes); returns its length; out has room for LP_TELEM_FRAME_BYTES */
uint8_t  onboard_telemetry_frame(const onb_tlm_src_t *src, uint16_t loop_hz, uint8_t veh, uint8_t seq, uint8_t *out);
/* the states in which a watchdog / brown-out reset counts as "in flight" (I10): POWERED, UNPOWERED, ABORT, RELEASED
 * (v0.6.3) */
bool     onboard_flying(uint8_t state);

#endif /* FC_ONBOARD_H */
