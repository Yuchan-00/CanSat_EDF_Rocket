/* main.c - ATmega128A flight control unit (FCU) of the CanSat EDF rocket: the 250 Hz loop (v0.6.3).
 *
 * v0.6.3 (params.json flight_computer.architecture, SPEC 10.4 / 10.5) runs the flight computer on three
 * ATmega128A.  The NAV (src/nav/) reads the MPU-6050 and the MS5611 and runs the attitude estimator, the tilt and
 * the altitude filter; at the end of each of its ticks it pushes the navigation frame (core/nlink_proto.h) to this
 * chip over the nlink SPI (hal/nlink_spi.h: NAV = master, FCU = slave) and receives this chip's state bytes in the
 * same exchange.  This chip, the FCU, makes every flight decision: the onboard supervisor (core/onboard.c, the port
 * of sim/onboard.py: the ground's commands and the interlocks I1-I10 that no command can disable), attitude control,
 * PWM outputs and the OpenLog log.  The COM chip (src/com/) owns the LoRa radio: it forwards every valid uplink CMD
 * as an IL_CMD frame over the ilink (core/ilink_proto.h; FCU USART1 <-> COM USART0, ILINK_BAUD) and builds the TELEM
 * reply from the IL_SNAP this chip sends back.  Launch, throttle, motor cut and nose release are still ground
 * commands (SPEC 10.3); the COM only carries them, the supervisor here decides.
 *
 * The tick STARTS when a complete navigation exchange has arrived (frame-triggered: the NAV's clock paces the loop
 * at NAV_TICK_HZ == FC_LOOP_HZ).  If none arrives within NLINK_FRAME_TIMEOUT_US of the previous tick start, the FCU
 * runs the tick on its own clock with the navigation data invalid, and keeps doing so every FC_TICK_US (Timer2,
 * hal/timer.h) until frames come back.  Time-critical part, in the order of sim/sim6dof.py FlightSim.run:
 *   1. the exchange: nlink_spi_take + nl_decode_nav (sync, TYPE, CRC); arm switch
 *   2. ilink RX: of the bytes that were already in the RX ring when the tick started, exactly those an IL_CMD can
 *      complete in go through the decoder (il_decoder_cmd_need, at most ILINK_CRIT_BYTES), so an IL_CMD that is
 *      complete on the wire before the tick reaches this tick's supervisor; bounded work per byte (il_decoder_push)
 *   3. onboard supervisor: the IL_CMD frames decoded since its previous call; the sensor booleans from the frame
 *      exactly as nlink_proto.h defines them (imu_ok = a valid frame && NL_F_IMU_OK && (NL_F_EST_VALID || the
 *      previous state is not a flying one), baro_ok = valid && NL_F_BARO_OK, aligned, gyro_saturated; no valid frame
 *      -> all false: a dead NAV or a cut wire is an IMU + baro failure, I9); h, v_z, tilt, rates from the frame (the
 *      newest valid one is held while none comes)
 *   4. attitude controller + mixer on the frame's q and rates while the supervisor says control_active; the vane
 *      pulses written right after: frame -> vane register is the FCU's share of the sample -> PWM path, which must
 *      stay within one control tick (the 6-DOF simulation's one tick of compute delay)
 *   5. ESC and release pulses (the supervisor's outputs) written right after they are computed, ~45 us after the vanes
 *   6. background bookkeeping (anchor, phases), battery ADC (2 Hz LPF; the supervisor uses the value filtered up to the
 *      previous tick, which keeps it off the frame -> vane path), event / link records, the reply snapshot (IL_SNAP:
 *      the supervisor's outputs + the frame's q), the FCU frame
 *      the NEXT exchange returns to the NAV (nlink_spi_set_reply: the new state, NL_FF_BENCH / NL_FF_RECOVERY,
 *      FC_PARAMS_TAG, the SEQ of the frame accepted), LED / buzzer, loop-time stamp; end of the time-critical part
 *   7. the logged-only EDF thrust estimate; log snapshot every FC_LOG_DIV ticks
 * The ilink's per-byte TX interrupt is held off during the time-critical part (a SNAP shifting out costs nothing
 * there); the RX interrupt cannot be held.  No timer interrupt runs on the FCU.
 * Background, one bounded job per pass, each started only if it ends BG_RESERVE_US before the window in which the
 * NAV's next exchange can be in progress (from the expected frame time - NLINK_EARLY_US - NLINK_XFER_US on: the
 * slave interrupt then owns the CPU for ~4.5k cycles, and no job may still run when the frame completes): the SNAP
 * frame to the COM first, then ilink bytes -> decoder (at most ILINK_BG_BYTES per tick ahead of the other jobs, so a
 * babbling COM cannot starve them), event / link records, flight-log packing, IL_STAT -> 'C' record, the 1 Hz 'N'
 * (NAV link) record, the COM / NAV text records, stack scan, the 1 Hz status record.  Then the loop waits for the
 * next frame, polling (no sleep: the frame is noticed within a few us, and no wake-up interrupt is needed): it
 * decodes ilink bytes as they arrive - ILINK_CHUNK at a time while the frame is not yet due, single bytes after -
 * so at the tick only the last byte or two are left for step 2.  (v0.6 held the background jobs back while a frame
 * that could be an IL_CMD was arriving; under slow babble that hold starved the log packing - R1.  Since v0.6.3 the
 * jobs end ~1 ms before the next frame and the idle decode takes everything that arrives after them, so the hold is
 * gone.)  Bytes the FCU cannot take (a babbling COM sends 100 per tick) overflow the RX ring and are counted; without
 * an IL_CMD the supervisor sees a lost link (I1p / I1).
 *
 * Safety invariants: the ESC sees its off pulse from the first PWM frame after reset and in every state except
 * POWERED; arming only by a valid ARM command with every SPEC 8.8 gate (arm switch, sensors, alignment, tilt,
 * battery, a live link); the arm switch opening acts as KILL; the radio and the sensors run on other chips, so no
 * radio or I2C fault can stall this loop: a dead or resetting COM only stops the IL_CMD frames (a lost radio link,
 * I1p / I1), a dead or resetting NAV only stops the navigation frames (the tick runs on the FCU's own clock with
 * imu_ok = baro_ok = false -> I9 after ONB_IMU_FAIL_TICKS; a NAV that reset in flight has no NL_F_EST_VALID -> I9);
 * the links are read from rings / whole exchanges and written by whole-frame writes that never wait; the watchdog
 * resets a hung loop into a SAFE boot, which opens the release if the reset happened in flight (I10).
 *
 * v0.7 (SPEC 8.9, params.json mission.mode, fc_mode.h): one source, two builds.
 *   BALLISTIC (FC_MISSION_MODE == MISSION_MODE_BALLISTIC): everything above, the v0.6.3 code path unchanged
 *     (onboard_update, the supervisor's slewed throttle, IL_SNAP, log format 4).
 *   VTOL (the default, params.json mission.mode = "vtol"): the ground sends only ON / OFF / UP / DOWN (+ KILL) and the
 *     vehicle flies the altitude itself.  Step 3 becomes the FCU stack of core/onboard.h (onb_stack_step, the port of
 *     sim/onboard.py FcuStack, the normative tick order the cross-check replays): the supervisor - given the vertical
 *     controller's touchdown / at_target and the ESC throttle of the PREVIOUS tick - decides the state, the vertical
 *     phase (SPOOL / CLIMB / HOLD / LAND / LANDED), the altitude target and whether the motor may run; then the
 *     vertical controller (core/vertical_ctrl.c) computes the throttle u from the frame's altitude, v_z and tilt; the
 *     ESC gets u only while the supervisor enables the motor (state POWERED), else its off pulse.  Step 4's attitude
 *     controller uses that u for its gain schedule.  ARMED is the spool warning: motor off, the buzzer sounds
 *     (onb_outputs_t.buzzer), the LED shows the phase (indicators_vtol).  The reply is an IL_SNAP2 (phase + target,
 *     the COM sends TELEM v2), the log is format 5 (core/log_record.h: phases in 'E' / 'L', the 'V' record).  An
 *     in-flight reset (I10) boots ABORT: the release opens abort_release_delay_s later (the supervisor's timer), not
 *     at boot; a reset in LANDED (UNPOWERED) is not "in flight" (hal/watchdog.c, onboard_flying_mode).  No command
 *     can disable a VTOL rule (V1-V10, I2 / I3 / I8 / I9, arm switch, KILL): they live in the supervisor.
 *   v0.7.1 VTOL tick order (params.json mission.vtol.fcu_tick_order_note; v0.7.0 measured sample -> vane 5.15 ms with the
 *     supervisor + vertical controller in front of the vanes): 1. the exchange, 2. the attitude controller on this
 *     frame with the PREVIOUS tick's control_active and ESC throttle (onb_stack_att_active / _att_u), 3. the vanes,
 *     4. ilink RX (the bytes counted at the tick start, as before), 5. onb_stack_step (supervisor -> vertical
 *     controller v2 on the frame's h, v_z, cos(tilt) from q and body-z specific force), 6. ESC / release + the rest of
 *     step 6.  The FCU frame back to the NAV carries the ESC's u (thr_q, the NAV's baro thrust compensation); the log
 *     is format 6 (the 'V' record adds the touchdown signatures and the NAV's estimator-correction count); the
 *     boot text names the build profile (params_gen.h MISSION_PROFILE_NAME: field, or demo with -D MISSION_PROFILE_DEMO).
 *   v0.8 VTOL build with GPS position hold (params.json mission.vtol.position_hold; core/onboard.h "v0.8"; -D ONB_NO_GPS
 *     builds the same VTOL firmware without it - env fcu_nogps, the indoor fallback: GPS needs open sky): the NAV sends
 *     nlink v3 frames (core/nlink_proto.h nl_nav3_t: the PREDICTED attitude of its tick, cos_tilt / tilt, the altitude
 *     filter of its previous tick, the 5-byte rotating GPS slot); a v2 frame is refused (TYPE) - a NAV of another build is
 *     an I9 on the pad.  Tick: 1. the exchange (nl_decode_nav3); 1b. the position work the previous tick's background did
 *     not finish (onb_stack_bg pieces, the tilt builder, attitude_ctrl_set_tilt / _finish: counted, g_pos_late - the
 *     attitude controller must fly the reference's setpoint); 2. the attitude controller (the previous tick's
 *     control_active, ESC throttle AND tilt setpoint); 3. vanes; 4. ilink RX; 5. onb_stack_rt (the slot receiver, the
 *     supervisor with the ON gate / V11 - V15, the vertical controller on the frame's cos_tilt, the position step's latch
 *     when due) -> 6. the ESC, the rest of step 6.  Background: the position work FIRST - one piece per job (core/
 *     position_ctrl.h P0..P10 of the step in progress, then the publication and the tilt builder), then
 *     attitude_ctrl_set_tilt and its finish (each <= 0.2 ms, measured by the core agent on the ATmega128A) - then the SNAP3
 *     (IL_TYPE_SNAP3: the COM sends TELEM v3 with the GPS status, satellites, accuracy, position mode, distance to the hold
 *     point), the records.  Log format 7 adds the 'G' record (core/log_record.h, 2.5 Hz); FW 3.4.0.
 *   v0.8.1, the FOUR-chip set (architect 2026-10-21 20:30, params flight_computer.architecture_v081 / glink / gns; the v0.8
 *     integration measured the three-chip GPS FCU at 83.3 % time-critical (89.2 % with ISRs), its background starved - no
 *     ilink command decoded in position hold -, 436 B of stack): the GNS board runs the GPS filter and the position
 *     controller; this FCU runs the GNS chain (core/onboard.h ONB_CHAIN_GNS: the nlink v4 GNS slot receiver nl_gns_rx, the
 *     supervisor's V11-V15 on its data - incl. its OWN GPS_LOST when no TILT part came for land_steps cycles -, no position
 *     controller).  The NAV sends nlink v4 (NL_TYPE_NAV4: the v3 payload, the slot a GNS slot - TILT, STATUS, POS, VEL,
 *     TRIM); a v3 / v2 frame is refused (TYPE: a NAV of another build is an I9 on the pad).  This chip answers with the v4
 *     FCU frame (NL_TYPE_FCU4: + pflags = phase | motor_on | sat == +1 | low_alt | ground | lifted, kt_q = k_T x 16384 of
 *     its latest tick: the GNS position controller's FCU inputs, which the NAV forwards in its glink STATE).  Tick order
 *     unchanged: 1. the exchange (nl_decode_nav4); 1b. the builder work the previous tick's background left (counted,
 *     g_pos_late); 2. the attitude controller with the previous tick's control_active, ESC throttle AND tilt setpoint;
 *     3. vanes; 4. ilink RX; 5. onb_stack_rt (the slot receiver, the supervisor, the vertical controller, the FCU4 fields);
 *     6. ESC, the rest of step 6 (the v4 reply).  Background: first the builder (onb_stack_bg: one call - the slot's TILT,
 *     or the newest set's TRIM after 2 missed cycles, slewed), then attitude_ctrl_set_tilt / _finish, then the v0.8 jobs.
 *     The NAV sample of tick k reaches the attitude controller at k + L + 1 (L = GLINK_LATENCY_TICKS, the TILT part's
 *     frame; core/glink_proto.h).  Log format 8: the 'G' record in its GNS form (core/log_record.h log_gns_t); FW 3.5.0.
 *     fcu_nogps (-D ONB_NO_GPS) reads the same v4 frames (the slot ignored) and sends the same v4 reply, so the one NAV
 *     image serves both; it keeps log format 6 and SNAP2.
 */
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#include "board.h"
#include "fc_mode.h"
#include "gains.h"
#include "params_gen.h"

#include "core/attitude_ctrl.h"
#include "core/fc_config.h"
#include "core/ilink_proto.h"
#include "core/link_proto.h"
#include "core/log_record.h"
#include "core/nlink_proto.h"
#include "core/onboard.h"
#include "core/thrust_model.h"

#include "hal/adc.h"
#include "hal/gpio.h"
#include "hal/ilink_uart.h"
#include "hal/nlink_spi.h"
#include "hal/pwm.h"
#include "hal/stack.h"
#include "hal/timer.h"
#include "hal/uart_log.h"
#include "hal/watchdog.h"
#include "nav/nav.h"                                       /* NAV_F_EST_GATE (the NAV's frame flag bit 7, VTOL) */
#include "simbench.h"

#ifndef FC_ROLE_FCU
#error "main.c is the FCU firmware: build it with -D FC_ROLE_FCU (platformio.ini [fcu_common])"
#endif
#if LOG_COM_STAT_LEN != IL_STAT_LEN
#error "log_record.h LOG_COM_STAT_LEN must equal ilink_proto.h IL_STAT_LEN (the 'C' record carries the IL_STAT payload)"
#endif
#if !defined(NAV_TICK_HZ) || NAV_TICK_HZ != FC_LOOP_HZ
#error "the frame-triggered loop needs NAV_TICK_HZ == FC_LOOP_HZ (params_gen.h, flight_computer.nav.tick_hz)"
#endif
#if !defined(NLINK_FRAME_TIMEOUT_T1_COUNTS) || !defined(NLINK_XFER_US)
#error "NLINK_* missing: re-run tools/gen_firmware_params.py (params.json flight_computer.nlink)"
#endif

#ifdef FC_SIMBENCH
#define SB_MARK(i) simbench_mark(i)
#else
#define SB_MARK(i) ((void)0)
#endif
#if defined(FC_SIMBENCH) && !defined(FC_COSIM)            /* single-chip simbench: the synthetic NAV and COM */
#define SB_PEERS() simbench_peers()
#define SB_TICK_START(t, nav) simbench_tick_start(t, nav)
#else
#define SB_PEERS() ((void)0)
#define SB_TICK_START(t, nav) ((void)0)
#endif

#if GAINS_LOOP_HZ_MISMATCH
#warning "gains.h: sim/out/gains.json was tuned at another loop_hz than the firmware runs - re-run the simulation tuning"
#endif
#if GAINS_PARAMS_STALE
#warning "gains.h: sim/out/gains.json was tuned against a different params.json"
#endif
#if FC_VTOL && !VC_GAINS_TUNED
#warning "params_gen.h: the vertical controller flies mission.vtol.gains_ASSUMED (no sim/out/vtol_gains.json)"
#endif
#if FC_VTOL && VC_GAINS_PARAMS_STALE
#warning "params_gen.h: sim/out/vtol_gains.json was tuned against a different params.json"
#endif

/* bench builds (platformio.ini) cap the throttle sent to the ESC:
 *   fcu_bench_tether: FC_THROTTLE_LIMIT (fraction)      fcu_bench_hover: hover throttle + a margin */
#if defined(FC_THROTTLE_LIMIT_HOVER_MARGIN) && !defined(FC_THROTTLE_LIMIT)
#define FC_THROTTLE_LIMIT (FC_HOVER_THROTTLE + FC_THROTTLE_LIMIT_HOVER_MARGIN)
#endif
#ifndef FC_THROTTLE_LIMIT
#define FC_THROTTLE_LIMIT 1.0f
#endif

#if ONB_HAS_FCU_POS
#error "main.c v0.8.1 builds the four-chip FCU (ONB_CHAIN_GNS); the v0.8 three-chip FCU (-D ONB_V08_FCU_CHAIN) is archived"
#endif
#if FC_VTOL && ONB_HAS_GPS
#define FW_VERSION "3.5.0"                  /* v0.8.1 four-chip VTOL build with GPS position hold: nlink v4, SNAP3, format 8 */
#elif FC_VTOL
#define FW_VERSION "3.5.0-nogps"            /* v0.8.1 VTOL build without position hold (-D ONB_NO_GPS): nlink v4, format 6 */
#else
#define FW_VERSION "3.2.0"                  /* the ballistic build: v0.7.0's string, 3.1.0's code path (v0.6.3) */
#endif

/* logical outputs -> compare channel (params.flight_computer.pins, the FCU's pin table) */
enum { OUT_VANE1, OUT_VANE2, OUT_VANE3, OUT_VANE4, OUT_ESC, OUT_RELEASE };
static const uint8_t k_out_channel[PWM_N_CH] = {PWM_OUT_VANE1, PWM_OUT_VANE2, PWM_OUT_VANE3, PWM_OUT_VANE4,
                                                PWM_OUT_ESC, PWM_OUT_RELEASE};
static const float k_vane_min_us[4] = {CAL_VANE_MIN_US_1, CAL_VANE_MIN_US_2, CAL_VANE_MIN_US_3, CAL_VANE_MIN_US_4};
static const float k_vane_max_us[4] = {CAL_VANE_MAX_US_1, CAL_VANE_MAX_US_2, CAL_VANE_MAX_US_3, CAL_VANE_MAX_US_4};

static attitude_ctrl_t g_ctrl;
static ctrl_output_t g_co;
#if FC_VTOL
static onb_stack_t g_stk;                              /* supervisor + vertical controller (core/onboard.h onb_stack_*) */
#if ONB_HAS_GPS
static uint16_t g_pos_late;                            /* v0.8: ticks whose position work was finished at the next tick start */
#endif
#define g_sup (g_stk.sup)                              /* the supervisor inside it (simbench, co-simulation probes) */
#else
static onboard_t g_sup;
#endif
static onb_outputs_t g_so;
static edf_model_t g_edf;
static log_flight_t g_rec;
#if FC_VTOL || defined(FC_SIMBENCH)
/* VTOL (v0.7): every record payload is packed straight into its place in the frame (log_frame copies it onto itself,
 * as snap_step's il_encode already does): one buffer instead of two gives back 96 of the bytes the VTOL stack
 * (onb_stack_t) and records add to the static RAM, so the flight build keeps >= 1 KiB of stack.  The simbench builds of
 * both modes do the same (their own buffers fill the RAM; the boot-time micro-benchmarks borrow simbench_scratch()
 * instead).  The ballistic flight build keeps the v0.6.3 pair of buffers. */
static uint8_t g_frame[LOG_MAX_PAYLOAD + LOG_FRAME_OVERHEAD];   /* log frames; also the SNAP frame (background) */
#define g_payload (g_frame + 4)
#else
static uint8_t g_payload[LOG_MAX_PAYLOAD];
static uint8_t g_frame[LOG_MAX_PAYLOAD + LOG_FRAME_OVERHEAD];   /* log frames; also the SNAP frame (background) */
#endif
#if IL_SNAP_FRAME_BYTES > LOG_MAX_PAYLOAD + LOG_FRAME_OVERHEAD || IL_SNAP2_FRAME_BYTES > LOG_MAX_PAYLOAD + LOG_FRAME_OVERHEAD || \
    IL_SNAP3_FRAME_BYTES > LOG_MAX_PAYLOAD + LOG_FRAME_OVERHEAD
#error "g_frame must hold an IL_SNAP / IL_SNAP2 / IL_SNAP3 frame"
#endif
/* the reply snapshot of this build: IL_SNAP (ballistic, 55-byte frame), IL_SNAP2 (VTOL: + phase, target; 61 bytes) or
 * IL_SNAP3 (v0.8 VTOL with GPS position hold: + the GPS fields; 68 bytes) */
#if FC_VTOL && ONB_HAS_GPS
#define SNAP_TYPE IL_TYPE_SNAP3
#define SNAP_LEN IL_SNAP3_LEN
#define SNAP_FRAME_BYTES IL_SNAP3_FRAME_BYTES
typedef il_snap3_t snap_t;
#elif FC_VTOL
#define SNAP_TYPE IL_TYPE_SNAP2
#define SNAP_LEN IL_SNAP2_LEN
#define SNAP_FRAME_BYTES IL_SNAP2_FRAME_BYTES
typedef il_snap2_t snap_t;
#else
#define SNAP_TYPE IL_TYPE_SNAP
#define SNAP_LEN IL_SNAP_LEN
#define SNAP_FRAME_BYTES IL_SNAP_FRAME_BYTES
typedef il_snap_t snap_t;
#endif
#if SNAP_FRAME_BYTES >= ILINK_UART_TX_BUF
#error "the ilink TX ring must hold one reply snapshot frame (hal/ilink_uart.h ILINK_UART_TX_BUF)"
#endif

/* ---- nlink (NAV -> FCU navigation frames, core/nlink_proto.h) ---- */
#define T1_COUNTS(us) ((uint16_t)((uint32_t)(us) * FC_PWM_COUNTS_PER_US))   /* us -> Timer1 counts */
#define NLINK_EARLY_US 350u           /* the next frame may complete this much before one FC_TICK_US after this tick's
                                       * start (g_anchor): the NAV's tick-to-tick variation (its time-critical part: pad /
                                       * flight estimator, baro phases, its own I2C interrupts; the simbench's synthetic
                                       * NAV sweeps SB_NAV_JITTER_US = 250 us) + how late this tick started after its frame
                                       * (one pass of the wait loop or one ilink byte's decode, ~10-20 us; <= 100 us in
                                       * the simbench, whose own calls sit in that loop).  The background keeps out of
                                       * the exchange that early */
/* background jobs end before the next exchange can start: the expected frame - NLINK_EARLY_US - NLINK_XFER_US */
#define BG_END_US (FC_TICK_US - NLINK_EARLY_US - NLINK_XFER_US)
#define IDLE_END_US (FC_TICK_US - NLINK_EARLY_US)     /* the idle decode switches to single bytes from here on */
/* the exchange taken in step 1 and the reply encoded in step 6 share g_frame: the background's jobs build their frames
 * in it and are done with it before the tick starts */
#define g_nlbuf g_frame
#if NL_XFER_BYTES > LOG_MAX_PAYLOAD + LOG_FRAME_OVERHEAD
#error "g_frame must hold an nlink exchange"
#endif
#if FC_VTOL
/* v0.8: the nlink v3 payload (core/nlink_proto.h nl_nav3_t); v0.8.1: under NL_TYPE_NAV4 with the GNS slot (the VTOL NAV
 * sends nothing else).  Its quantised fields as the v0.7.1 code used them: accel x / y in 0.01 m/s^2, h_baro_rel in cm
 * (log only) */
static nl_nav3_t g_nav;                                /* the newest valid navigation frame (held while none comes) */
#define NAV_ACCEL(i) ((i) == 2 ? g_nav.accel_z : (float)((i) == 0 ? g_nav.accel_x_q : g_nav.accel_y_q) * (1.0f / NL_ACCEL_XY_SCALE))
#define NAV_H_BARO_REL() ((float)g_nav.h_baro_rel_cm * (1.0f / NL_H_BARO_SCALE))
#define NAV_DECODE(buf, n) nl_decode_nav4((buf), (n))
#else
static nl_nav_t g_nav;                                 /* the newest valid navigation frame (held while none comes) */
#define NAV_ACCEL(i) (g_nav.accel[(i)])
#define NAV_H_BARO_REL() (g_nav.h_baro_rel_m)
#define NAV_DECODE(buf, n) nl_decode_nav((buf), (n))
#endif
static uint16_t g_anchor;                              /* Timer1 count at which this tick's frame completed (or, after a
                                                        * late or missing one, would have): the next is due 1 tick on */
static struct {
    uint16_t frames, bad, timeouts, seq_lost;          /* log 'N' (log_nav_t), cumulative */
    uint16_t loop_max;                                 /* the NAV's loop_us_max over the 'N' window */
    uint8_t last_seq;                                  /* SEQ of the newest valid frame (the reply's SEQ) */
    bool have;                                         /* a valid frame since boot */
    bool boot_prev;                                    /* NL_F_BOOT in the previous valid frame */
    bool announce;                                     /* text record due: first frame, or the NAV rebooted */
    bool rec_due;                                      /* 'N' record due */
} g_nl;

/* ---- ilink (FCU <-> COM).  The decoder is fed only from the main loop (time-critical part, background, idle wait) -
 * never from an ISR - so the CMD queue it fills and the supervisor that empties it need no locking.  Every byte goes
 * through il_decoder_push (core/ilink_proto.h: no replay, bounded cycles a byte, the complete frame handed out in place),
 * so decoding costs a bounded amount per byte fed, and these byte budgets bound it per tick ---- */
#define ILINK_CRIT_BYTES 16u   /* step 2 budget: >= one IL_CMD frame (14 B) and >= the bytes the background and the idle
                                * decode can leave at a tick: normally the byte or two that arrived while the frame was
                                * being noticed; at most those of one background job overtaken by an early frame
                                * (BG_RESERVE_US: 7 bytes at 40 us) or the wait's last chunk.  Worst case 16 bytes ~ 1930
                                * cycles = 0.12 ms */
#define ILINK_BG_BYTES 16u     /* D6: bytes the background decodes per tick AHEAD of the record / log jobs: >= one IL_CMD
                                * frame, ~16 x 136 cycles = 0.14 ms at most; the rest (an IL_STAT's tail, a babbling COM's
                                * stream) waits until those jobs are done (idle decode) */
/* D6, flood: a working COM's largest burst (IL_CMD + IL_STAT, 56 B) fits the 63-byte RX ring, so an RX ring overflow
 * means the COM sends more than the FCU takes (babble, a stuck transmitter).  In every tick after one in which the ring
 * overflowed, step 2 and the background share decode nothing - only the idle wait after the record / log jobs does; the
 * excess bytes overflow the ring (counted), no IL_CMD gets through, the supervisor sees a lost link (I1p / I1) */
static bool g_il_flood;
static uint16_t g_il_ovf_seen, g_il_flood_ticks;
static il_decoder_t g_ild;
static onb_frame_t g_cmdq[ONB_MAX_FRAMES];             /* IL_CMD frames for the next supervisor call, in arrival order */
static int8_t g_cmdq_rssi[ONB_MAX_FRAMES], g_cmdq_snr[ONB_MAX_FRAMES];
static uint8_t g_ncmd;
static uint16_t g_il_cmds;                             /* IL_CMD frames queued for the supervisor */
static uint16_t g_il_dropped;                          /* IL_CMD beyond ONB_MAX_FRAMES a tick + frames of an unexpected TYPE */
static uint16_t g_il_crit_over;                        /* ticks an IL_CMD could not complete within ILINK_CRIT_BYTES */
static uint16_t g_il_bg_capped;                        /* ticks the background used up ILINK_BG_BYTES (simbench report) */
static uint8_t g_il_bg_left;                           /* this tick's background byte budget */
static uint32_t g_tick;                                /* the tick being processed (IL_STAT receive tick) */
static snap_t g_snap;                                  /* the reply snapshot captured by the supervisor's tick */
static bool g_snap_pending;
static uint16_t g_snaps, g_snap_dropped;               /* SNAP frames queued / replaced before they left */
static struct {
    uint8_t raw[IL_STAT_LEN];                          /* the newest IL_STAT payload, as received ('C' record) */
    uint32_t raw_tick;                                 /* FCU tick it was decoded in */
    bool fresh;                                        /* raw not yet unpacked / logged */
    bool have;                                         /* s is valid (an IL_STAT was received since boot) */
    bool announce;                                     /* text record due: first STAT, or the COM rebooted */
    il_stat_t s;                                       /* the newest IL_STAT, unpacked */
    uint32_t s_tick;
} g_com;

/* events / link records: packed in the tick (byte copies), framed and written in the background (VTOL, format 5:
 * 'E' 10 bytes, 'L' 15 bytes) */
#define EVQ_N 6u
#if FC_VTOL
#define EVQ_D LOG_LINK2_LEN
#else
#define EVQ_D LOG_LINK_LEN
#endif
#if LOG_EVENT2_LEN > LOG_LINK2_LEN || LOG_EVENT_LEN > LOG_LINK_LEN
#error "an event record must fit the record queue slot"
#endif
typedef struct {
    uint8_t type, len;
    uint8_t d[EVQ_D];
} evq_t;
static evq_t g_evq[EVQ_N];
static uint8_t g_evq_head, g_evq_n;
static uint16_t g_evq_dropped;

static evq_t *evq_slot(uint8_t type)
{
    evq_t *e;
    if (g_evq_n >= EVQ_N) {
        g_evq_dropped++;
        return 0;
    }
    e = &g_evq[(uint8_t)((g_evq_head + g_evq_n) % EVQ_N)];
    g_evq_n++;
    e->type = type;
    return e;
}

/* ------------------------------------------------------------------------ */
static void log_send_buf(uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint16_t n = log_frame(type, payload, len, g_frame);
#ifdef FC_SIMBENCH
    if (type == LOG_TYPE_FLIGHT || type == LOG_TYPE_STATUS || type == LOG_TYPE_EVENT || type == LOG_TYPE_LINK ||
        type == LOG_TYPE_COM || type == LOG_TYPE_NAV || type == LOG_TYPE_VTOL || type == LOG_TYPE_GPS)
        return;                                   /* packed and framed (timed), not sent: the console shows the report */
#endif
    uart_log_write(g_frame, n);
}

static void log_send(uint8_t type, uint8_t len)
{
    log_send_buf(type, g_payload, len);
}

static uint8_t copy_P(uint8_t pos, const char *s_P)
{
    char c;
    while (pos < LOG_MAX_PAYLOAD && (c = (char)pgm_read_byte(s_P)) != 0) {
        g_payload[pos++] = (uint8_t)c;
        s_P++;
    }
    return pos;
}

static uint8_t put_hex(uint8_t pos, uint16_t v, uint8_t digits)
{
    static const char hex[] PROGMEM = "0123456789ABCDEF";
    while (digits-- && pos < LOG_MAX_PAYLOAD)
        g_payload[pos++] = (uint8_t)pgm_read_byte(&hex[(v >> (4 * digits)) & 0xF]);
    return pos;
}

static void log_text_hex_P(const char *s_P, uint16_t v)
{
    uint8_t n = copy_P(0, s_P);
    log_send(LOG_TYPE_TEXT, put_hex(n, v, 4));
}

#if FC_VTOL
/* fw09: the VTOL boot texts at boot only (before the loop): wait until the log ring has room for the record.  The boot
 * record and the build's texts are queued back to back - VTOL GPS build 95 + 72 + 102 B framed, no-GPS build 101 + 72 +
 * 84 B, against the 256-byte ring (128 B in the simbench builds) at 174 us a byte - so without the wait the last of them
 * and the RECOVERY text were dropped (uart_log_write refuses a frame that does not fit).  Bounded: <= LOG_MAX_PAYLOAD +
 * LOG_FRAME_OVERHEAD bytes to drain (~18 ms), the watchdog kicked meanwhile.  The ballistic build's boot texts fit the
 * ring (100 + 87 B): its code is unchanged. */
static void log_boot_room(uint8_t len)
{
    while (uart_log_free() < (uint16_t)(len + LOG_FRAME_OVERHEAD))
        watchdog_kick();
}

static void log_boot_text(uint8_t len)
{
    log_boot_room(len);
    log_send(LOG_TYPE_TEXT, len);
}
#endif

/* the boot record: log format (4 ballistic, 5 VTOL), reset cause, loop rate, PWM counts per us, then
 * "fw 3.2.0 role FCU <mode> <preset> params <sha prefix> gains <sha prefix>" (decode_log.py reads role and mode) */
#if FC_VTOL && ONB_HAS_GPS
#define LOG_FORMAT LOG_FORMAT_VERSION_GNS                 /* v0.8.1: format 8 ('G' in its GNS form) */
#elif FC_VTOL
#define LOG_FORMAT LOG_FORMAT_VERSION_VTOL
#else
#define LOG_FORMAT LOG_FORMAT_VERSION
#endif
static void log_boot(uint8_t reset_cause)
{
    uint8_t n = 0;
    g_payload[n++] = LOG_FORMAT;
    g_payload[n++] = reset_cause;
    g_payload[n++] = (uint8_t)(FC_LOOP_HZ & 0xFF);
    g_payload[n++] = (uint8_t)(FC_LOOP_HZ >> 8);
    g_payload[n++] = (uint8_t)FC_PWM_COUNTS_PER_US;
    n = copy_P(n, PSTR("fw " FW_VERSION " role FCU " FC_MODE_NAME " " FC_PRESET " params " FC_PARAMS_SHA256_PREFIX
                       " gains " GAINS_SOURCE_SHA256_PREFIX));
    log_send(LOG_TYPE_BOOT, n);
#if FC_VTOL                                            /* which profile and vertical gains this build flies */
    /* v0.7.1: "VTOL profile field: vertical gains sim/out/vtol_gains.json (tuned)" - the build profile (params_gen.h
     * MISSION_PROFILE_NAME; decode_log.py reads it) does not fit the boot record's 96 bytes */
    n = copy_P(0, PSTR("VTOL profile " MISSION_PROFILE_NAME ": vertical gains " VC_GAINS_SOURCE));
    n = copy_P(n, VC_GAINS_TUNED ? PSTR(" (tuned)") : PSTR(" (mission.vtol.gains_ASSUMED)"));
    log_boot_text(n);
    if (VC_GAINS_PARAMS_STALE)                        /* fw09: its own record (it was cut at the 96-byte payload) */
        log_boot_text(copy_P(0, PSTR("VTOL vertical gains STALE: tuned against another params.json")));
#if ONB_HAS_GPS
    /* v0.8.1: "GPS position hold on (GNS board, glink L=3): gains <source> (tuned | ASSUMED)" (decode_log.py shows it) -
     * the position controller runs on the GNS; the gains named are the ones the GNS image of the same params carries.
     * fw09: two records - with today's source (85 characters) the one record was cut at the 96-byte payload and lost the
     * "(tuned | ASSUMED)" verdict: "GPS position hold on (GNS board, glink L=3): gains (ASSUMED)", "gains: <source>" */
    n = copy_P(0, PSTR("GPS position hold on (GNS board, glink L="));
    n = put_hex(n, GLINK_LATENCY_TICKS, 1);
    n = copy_P(n, POS_GAINS_TUNED ? PSTR("): gains (tuned)") : PSTR("): gains (ASSUMED)"));
    log_boot_text(n);
    log_boot_text(copy_P(0, PSTR("gains: " POS_GAINS_SOURCE)));
#else
    log_boot_text(copy_P(0, PSTR("GPS position hold OFF (build without it: attitude-only hover, the v0.7.1 VTOL)")));
#endif
    /* fw09 G1: the vehicle configuration whose constants this image flies (params_gen.h FC_VEHICLE_NAME, -D FC_VCFG) and
     * its tag - the boot record has no room for it (94 of its 96 bytes), the params sha in it is the same for every
     * configuration: "vehicle vtol_battery, params tag 0x09" */
    n = copy_P(0, PSTR("vehicle " FC_VEHICLE_NAME ", params tag 0x"));
    log_boot_text(put_hex(n, FC_PARAMS_TAG, 2));
#endif
}

/* ------------------------------------------------------------------------ */
static uint16_t k_esc_off_counts, k_release_open_counts, k_release_latched_counts;   /* set once at boot */

static uint16_t esc_counts(bool powered, float u)
{
    float us = CAL_ESC_OFF_US;
    if (powered)
        us = CAL_ESC_MIN_US + fc_clampf(u, 0.0f, 1.0f) * (CAL_ESC_MAX_US - CAL_ESC_MIN_US);
    return pwm_us_to_counts(us);
}

static void vane_counts(const float pwm_us[4], uint16_t out[PWM_N_CH])
{
    uint8_t i;
    for (i = 0; i < 4; i++)
        out[OUT_VANE1 + i] = pwm_us_to_counts(fc_clampf(pwm_us[i], k_vane_min_us[i], k_vane_max_us[i]));
}

static void safe_outputs(uint16_t out[PWM_N_CH], bool release_open)
{
    float d0[4] = {0, 0, 0, 0}, pw[4];
    mixer_pwm(&g_ctrl.mixer, d0, pw);                 /* centre + trim */
    vane_counts(pw, out);
    out[OUT_ESC] = esc_counts(false, 0.0f);
    out[OUT_RELEASE] = pwm_us_to_counts(release_open ? CAL_RELEASE_RELEASED_US : CAL_RELEASE_LATCHED_US);
}

/* ---- background flight-record packing (spread over the idle time between ticks) ---- */
static uint8_t g_fframe[LOG_FLIGHT_LEN + LOG_FRAME_OVERHEAD];
#if FC_VTOL                                            /* VTOL: the fine split, shorter jobs (log_record.h) */
#define FLOG_PARTS LOG_FLIGHT_PARTS_FINE
#define flog_part_start log_flight_part_fine_start
#define flog_pack_part log_pack_flight_part_fine
#else
#define FLOG_PARTS LOG_FLIGHT_PARTS
#define flog_part_start log_flight_part_start
#define flog_pack_part log_pack_flight_part
#endif
static uint8_t g_fpart = FLOG_PARTS;                  /* FLOG_PARTS = idle */
static uint16_t g_fs1, g_fs2;
static uint16_t log_flight_dropped;

static bool flight_log_busy(void)
{
    return g_fpart < FLOG_PARTS;
}

static void flight_log_start(void)
{
    g_fframe[0] = LOG_SYNC0;
    g_fframe[1] = LOG_SYNC1;
    g_fframe[2] = LOG_TYPE_FLIGHT;
    g_fframe[3] = LOG_FLIGHT_LEN;
    g_fs1 = g_fs2 = 0;
    log_fletcher16_acc(&g_fs1, &g_fs2, g_fframe + 2, 2);
    g_fpart = 0;
}

static bool flight_log_step(void)
{
    uint8_t a, b;
    if (g_fpart >= FLOG_PARTS)
        return false;
    a = flog_part_start(g_fpart);
    b = flog_pack_part(&g_rec, g_fframe + 4, g_fpart);
    log_fletcher16_acc(&g_fs1, &g_fs2, g_fframe + 4 + a, (uint16_t)(b - a));
    if (++g_fpart == FLOG_PARTS) {
        g_fframe[4 + LOG_FLIGHT_LEN] = (uint8_t)g_fs1;
        g_fframe[5 + LOG_FLIGHT_LEN] = (uint8_t)g_fs2;
#ifndef FC_SIMBENCH
        uart_log_write(g_fframe, sizeof g_fframe);
#endif
    }
    return true;
}

/* ---- time checks.  The background and the idle decode are timed from g_anchor (when this tick's frame completed, or
 * would have): the next frame is due FC_TICK_US after it, at the earliest NLINK_EARLY_US before that ---- */
static uint16_t t1_add(uint16_t t, uint16_t counts)   /* Timer1 count + counts, modulo FC_PWM_TOP + 1 */
{
    const uint32_t s = (uint32_t)t + counts;
    return (uint16_t)(s > FC_PWM_TOP ? s - (FC_PWM_TOP + 1UL) : s);
}

#if FC_VTOL
/* v0.8.1 VTOL: the exchange window.  The background jobs the window before BG_END_US could not take go on after it, in a
 * second pass (g_late: the background loop's jobs, the stack-scan slice, the status record) up to the earliest next frame
 * LATE_END_US (FC_TICK_US - NLINK_EARLY_US after the anchor) - each started only if it ends before that frame even
 * stretched by the nlink slave interrupt, which takes 62 of every 140 cycles while the NAV's exchange is in progress
 * (hal/nlink_spi.h Timing): the job's reserve x 140 / 78 (LATE_WALL).  A frame that comes later only finds the loop
 * already waiting; the tick start is never delayed by a job.  The jobs are plain code with only short interrupt-off
 * sections (ring head updates), as the v0.8 position pieces that already used this window (pos_job_late).  Why: the GNS
 * chain's time-critical part (~2.9 ms in position hold: the tilted setpoint's general quaternion error, the slot
 * receiver) leaves almost no window before BG_END_US - without this pass the SNAP replies, the records and the ilink
 * share starved (tools/run_simbench.py, v0.8.1 integration: V1 LAND before HOLD, the v0.8 failure mode). */
#define LATE_END_US (FC_TICK_US - NLINK_EARLY_US)
/* fw09/p1: x 115 / 64 (1.7969 >= 140 / 78 = 1.7949: never a smaller wall) - a 32-bit multiply and a shift instead of
 * the 32-bit division (__udivmodsi4, ~37 us a call, 2.5 calls a tick in the timing82 profile) */
#define LATE_WALL(us) ((uint16_t)(((uint32_t)(us) * 115u) >> 6) + 10u)
static bool g_late;
/* fw09 fix loop (SPEC_firmware_v09 section 7 row 14): the text records (com_text_step, nav_text_step) start only in a
 * tick whose anchor a frame set - not in a TICK_FRAME_LATE tick, whose anchor is a prediction (one tick after the previous
 * anchor, or the tick start).  After the FCU's own clock (the NAV's first frames after a NAV start) that prediction can lie
 * a large part of a tick after the frame: measured in the closed-loop co-simulation (fw09/impl/accept), the NAV text
 * record started 1686 us after such an anchor, the NAV's next exchange ran inside it, and the job took 709 us instead of
 * 450 us (the nlink slave interrupt takes 62 of every 140 cycles of an exchange) - past COM_TEXT_RESERVE_US, and the
 * next tick started late.  A text record can wait a tick (the next frame-seen tick, or a tick on the FCU's own clock
 * when the NAV is silent); the other jobs keep their reserves as before */
static bool g_text_ok;
/* fw09 review F1 (SPEC_firmware_v09 section 3.2, G1: "a mixed image set is refused at boot"): FC_PARAMS_TAG carries the
 * vehicle configuration (battery / tether / hybrid / rocket).  The NAV flags an FCU of another tag in every frame it sends
 * (reset_cause bit FCU_NAV_RC_TAG_MISMATCH = nav/nav.h NAV_RC_TAG_MISMATCH, core/nlink_proto.h), the COM sends its tag in
 * every IL_STAT; either mismatch latches g_images_mixed until power-off.  While it is set and the vehicle is SAFE or ARMED
 * (motor off) the supervisor gets its arm-switch input open: ON is refused (LP_REJ_ARM_SWITCH; the TELEM ARM_SWITCH flag
 * shows the arming chain open) and a spool-up in progress ends in SAFE.  In flight it changes nothing (no motor cut from a
 * configuration flag).  A matched image set never sets it, so the supervisor's inputs - and the flight - are unchanged
 * (the reference has one configuration per vehicle by construction).  The cause is in the log: the COM text record
 * ("params tag ... MISMATCH"), the 'N' / status records (nav_reset_cause 0x20, LOG_COM_TAG_MISMATCH), the NAV LED.  The GNS
 * is not covered: the glink carries no tag (sim/glink_proto.py, gns/gns_main.c).  The flag lives in .noinit (cleared by
 * main at every boot): one more byte of .bss would move the 16-byte-aligned nlink reply rows (hal/nlink_spi.c s_tx) to
 * the next boundary - 16 B of static RAM in fcu / fcu_demo, measured - which is the FCU's stack */
#define FCU_NAV_RC_TAG_MISMATCH 0x20u
static bool g_images_mixed __attribute__((section(".noinit")));
#endif

/* true while a piece of work that takes up to reserve_us still ends limit_us after the anchor, and no frame is waiting
 * (limit_us > reserve_us for every caller).  VTOL (v0.8.1): in the late pass a BG_END_US limit is LATE_END_US with the
 * stretched reserve (above) */
static bool time_left_us(uint16_t limit_us, uint16_t reserve_us)
{
#if FC_VTOL
    if (g_late && limit_us == BG_END_US) {
        limit_us = LATE_END_US;
        reserve_us = LATE_WALL(reserve_us);
    }
#endif
    return !nlink_spi_frame_ready() &&
           timer_elapsed_counts(g_anchor, timer_tcnt1()) < T1_COUNTS(limit_us - reserve_us);
}

/* background work may start a step only if it ends BG_RESERVE_US before the next exchange can start */
#define BG_RESERVE_US_BALLISTIC 280u  /* > the longest background job (SNAP frame: pack + CRC + queue, ~0.22 ms in simavr;
                                       * flight-record part / status record ~0.15 ms) + the RX interrupts of a COM sending
                                       * all the time (3.2 us per 40 us) and the log UDRE (2.9 us per 174 us).  v0.6: 420 us
                                       * for the MS5611 altitude (powf), which moved to the NAV */
/* fw09 (SPEC_firmware_v09 WP-F, section 7 row 14): the VTOL reserves are their own constants, so the integration round
 * re-derives them without touching the ballistic image (BG_RESERVE_US_BALLISTIC / COM_TEXT_EXTRA_US_BALLISTIC keep the
 * v0.6.3 bytes; tools/run_simbench.py reads the *_VTOL / *_BALLISTIC value of the mode it measures).  Method: the
 * closed-loop four-core co-simulation (real nlink slave / ilink / log interrupts, no bench load) reports on the FCU core
 * the longest job per reserve (simbench.c: SI "bj" = started on BG_RESERVE_US - the SNAP frame, the status record; "br"
 * = a record job on REC_RESERVE_US; "bt" = a text job on COM_TEXT_RESERVE_US; the position work has its own,
 * POS_RESERVE_US / POS_WALL_*).  Its maxima over every scenario go into FC_*_JOB_US_VTOL below (with the run that
 * measured them); each reserve must then be >= FC_RESERVE_FOR_US(job): the job stretched by a COM sending all the time
 * (the ilink RX interrupt, ILINK_RX_ISR_CYCLES of every 640-cycle byte at 250 kbaud) and the log UDRE (46 of every 2778
 * cycles at 57600 baud), rounded up to 10 us - the compile-time check below refuses a smaller one.  0 = not measured
 * yet: the v0.8.1 values stand (the single-chip bench's job lengths include the bench's own work, SPEC section 7 note). */
/* fw09 integration round (2026-10-29): the closed-loop co-simulation tools/run_cosim4.py, every closed-loop scenario
 * (19 runs, 52 s, fcu / fcu_demo / fcu_nogps / fcu_wired / fcu_wired_buffer) + the open-loop run, simbench.c's SI line
 * per reserve AND per pass (nj / nr / nt = the normal pass, before BG_END_US; bj / br / bt = both passes, an upper bound
 * of the late pass, which starts a job on LATE_WALL(reserve) before LATE_END_US - the exchange's slave interrupts are
 * inside the measured length there).  Report fw09/impl/integ/runs/round2/cosim4_report.json, section 7 row 14. */
#define FC_BG_JOB_US_VTOL 264u        /* us, normal pass, SNAP frame / status record (cl_nav_pulse; normal runs 248) */
#define FC_REC_JOB_US_VTOL 177u       /* us, normal pass, the record jobs (cl_normal_fcu) */
#define FC_TEXT_JOB_US_VTOL 456u      /* us, normal pass, the text records (cl_nav_pulse; normal runs 450, no-GPS builds 410):
                                       * fw09 fix loop, report fw09/impl/accept/runs/a1/cosim4_report.json.  The integration
                                       * round measured 709 us (cl_demo): a text record started on a predicted anchor
                                       * (TICK_FRAME_LATE at the NAV's first frames) with the next exchange inside it -
                                       * fixed by g_text_ok (above), not by a larger reserve */
#define FC_BG_LATE_JOB_US_VTOL 383u   /* us, both passes (cl_babble): LATE_WALL(BG_RESERVE_US) must cover it */
#define FC_REC_LATE_JOB_US_VTOL 357u  /* us, both passes (cl_gns_hold, cl_nl_flip): LATE_WALL(REC_RESERVE_US) */
#define FC_TEXT_LATE_JOB_US_VTOL 458u /* us, both passes (ol_normal; fw09 fix loop, accept/runs/a1): LATE_WALL(COM_TEXT_..) */
#define BG_RESERVE_US_VTOL 290u       /* fw09: FC_RESERVE_FOR_US(264) = 290 (v0.8.1: 280) */
#define REC_RESERVE_US_VTOL 220u      /* fw09: LATE_WALL(220) = 405 >= FC_RESERVE_FOR_US(357) = 400 (v0.8.1: 200) */
#define COM_TEXT_EXTRA_US_BALLISTIC 200u   /* COM_TEXT_RESERVE_US = BG_RESERVE_US + this (com_text_step) */
#define COM_TEXT_EXTRA_US_VTOL 210u   /* fw09 fix loop: COM_TEXT_RESERVE_US 500 us = FC_RESERVE_FOR_US(456) (v0.8.1: 200,
                                       * 490 us); ILINK_CRIT_BYTES: (500 + 39) / 40 = 13 <= 16 */
#define ILINK_RX_ISR_CYCLES 51u       /* hal/ilink_uart.c RX ISR incl. its entry (tools/run_simbench.py reads the ELF's) */
#define FC_RESERVE_FOR_US(job)                                                                                         \
    (((((job) * (640UL + ILINK_RX_ISR_CYCLES) * 2778UL + (job) * 46UL * 640UL + 640UL * 2778UL - 1UL) /                  \
       (640UL * 2778UL)) + 9UL) / 10UL * 10UL)
#if BG_RESERVE_US_VTOL < FC_RESERVE_FOR_US(FC_BG_JOB_US_VTOL) ||                                                       \
    REC_RESERVE_US_VTOL < FC_RESERVE_FOR_US(FC_REC_JOB_US_VTOL) ||                                                     \
    BG_RESERVE_US_VTOL + COM_TEXT_EXTRA_US_VTOL < FC_RESERVE_FOR_US(FC_TEXT_JOB_US_VTOL)
#error "a VTOL background reserve is below FC_RESERVE_FOR_US of the measured job it must cover (main.c, fw09 WP-F)"
#endif
/* the late pass (fw09 integration): a job started there must fit LATE_WALL(reserve) (above, here for #if) */
#define FC_LATE_WALL_PP(us) ((((us) * 115UL) >> 6) + 10UL)
#if FC_LATE_WALL_PP(BG_RESERVE_US_VTOL) < FC_RESERVE_FOR_US(FC_BG_LATE_JOB_US_VTOL) ||                                \
    FC_LATE_WALL_PP(REC_RESERVE_US_VTOL) < FC_RESERVE_FOR_US(FC_REC_LATE_JOB_US_VTOL) ||                              \
    FC_LATE_WALL_PP(BG_RESERVE_US_VTOL + COM_TEXT_EXTRA_US_VTOL) < FC_RESERVE_FOR_US(FC_TEXT_LATE_JOB_US_VTOL)
#error "a VTOL background reserve's late-pass wall LATE_WALL(reserve) is below FC_RESERVE_FOR_US of its late job"
#endif
#if FC_VTOL
#define BG_RESERVE_US BG_RESERVE_US_VTOL
#define COM_TEXT_EXTRA_US COM_TEXT_EXTRA_US_VTOL
#else
#define BG_RESERVE_US BG_RESERVE_US_BALLISTIC
#define COM_TEXT_EXTRA_US COM_TEXT_EXTRA_US_BALLISTIC
#endif
/* a frame that completes early overtakes at most one job: its bytes (one per 40 us, at most the longest reserve's worth)
 * must fit ILINK_CRIT_BYTES (step 2's budget) */
#if (BG_RESERVE_US + COM_TEXT_EXTRA_US + 39u) / 40u > ILINK_CRIT_BYTES
#error "ILINK_CRIT_BYTES is below the ilink bytes of the longest background job (COM_TEXT_RESERVE_US / 40 us)"
#endif
static bool bg_time_left(void)
{
    return time_left_us(BG_END_US, BG_RESERVE_US);
}

#if FC_VTOL
/* VTOL (v0.7): the FCU stack makes the time-critical part ~0.5 ms longer, so after it only ~0.2 ms of the background
 * window is left in which a job may START on BG_RESERVE_US - one job a tick, while the flight record alone needs four
 * parts every FC_LOG_DIV ticks.  The record jobs (flight-record parts, event / link, 'V', 'C', 'N' records, the status
 * record) are shorter than the SNAP frame the reserve was sized for, so they start on their own, smaller reserve
 * REC_RESERVE_US (run_simbench.py checks their longest duration against it); the SNAP frame keeps BG_RESERVE_US
 * (snap_step), the text records COM_TEXT_RESERVE_US, the ilink drain its per-chunk reserve.  The ballistic build keeps
 * the v0.6.3 single reserve. */
#define REC_RESERVE_US REC_RESERVE_US_VTOL
static bool rec_time_left(void)
{
    return time_left_us(BG_END_US, REC_RESERVE_US);
}
#define bg_loop_time_left rec_time_left
#else
#define bg_loop_time_left bg_time_left
#endif

/* An ilink decoder chunk of k bytes may start only if it ends in time.  Every byte costs the same bounded work whatever
 * the candidate (il_decoder_push, no replay), so the reserve is linear in k: ILINK_BYTE_US per byte (push + ring read +
 * loop, and the RX ISR of a byte arriving meanwhile: 3.2 us per 40 us) plus ILINK_FRAME_US for the one frame a chunk can
 * complete (ilink_frame: an IL_STAT's 36-byte copy, an IL_CMD's unpack + queue).  Measured in simavr (fcu_simbench_micro,
 * main_ilink_* lines), see run_simbench.py's checks. */
#define ILINK_BYTE_US 10u
#define ILINK_FRAME_US 30u
#define ILINK_CHUNK 8u                /* bytes per ring read (one time check per chunk) */
static bool ilink_time_left(uint16_t limit_us, uint8_t k)
{
    return time_left_us(limit_us, (uint16_t)(ILINK_FRAME_US + (uint16_t)k * ILINK_BYTE_US));
}

/* ---- ilink receive: one byte -> decoder -> frames.  IL_CMD -> supervisor queue (handed over in the next supervisor
 * call: in this tick if decoded in step 2, in the next one if decoded later); IL_STAT -> raw copy, unpacked and logged
 * in the background; the decoder accepts nothing else (IL_ACCEPT_FCU: a SNAP travels FCU -> COM only) ---- */
static void ilink_frame(const il_frame_t *f)
{
    uint8_t i;
    if (f->type == IL_TYPE_CMD) {
        il_cmd_t c;
        if (g_ncmd >= ONB_MAX_FRAMES) {
            g_il_dropped++;
            return;
        }
        il_unpack_cmd(f->payload, &c);
        g_cmdq[g_ncmd].seq = c.seq;
        g_cmdq[g_ncmd].cmd = c.cmd;
        g_cmdq_rssi[g_ncmd] = c.rssi_dbm;
        g_cmdq_snr[g_ncmd] = c.snr_q4;
        g_ncmd++;
        g_il_cmds++;
    } else if (f->type == IL_TYPE_STAT) {
        for (i = 0; i < IL_STAT_LEN; i++)                /* a STAT not yet logged is replaced (1 Hz: cannot happen) */
            g_com.raw[i] = f->payload[i];
        g_com.raw_tick = g_tick;
        g_com.fresh = true;
    } else {
        g_il_dropped++;                                  /* not reachable with IL_ACCEPT_FCU: kept as a guard */
    }
}

/* bytes -> decoder (zero-copy: a complete frame is handled in place, before the next byte overwrites it) */
static void ilink_feed(const uint8_t *b, uint8_t n)
{
    uint8_t i;
    for (i = 0; i < n; i++) {
        const il_frame_t *f = il_decoder_push(&g_ild, b[i]);
        if (f)
            ilink_frame(f);
    }
}

/* step 2: of the bytes that were waiting when the tick started (the oldest in the ring), exactly those an IL_CMD can
 * complete in: il_decoder_cmd_need is the fewest bytes after which the decoder can return an IL_CMD, so `need` bytes are
 * fed while that many are still waiting - a STAT's middle or the first bytes of a CMD stay in the ring for the idle wait,
 * at no cost here.  At most ILINK_CRIT_BYTES are fed (bounded work).  Not inlined: its buffer lives only during this
 * call, not under the supervisor / controller. */
static uint8_t ilink_rx_crit(uint8_t n_at_tick) __attribute__((noinline));
static uint8_t ilink_rx_crit(uint8_t n_at_tick)
{
    uint8_t buf[ILINK_CRIT_BYTES], n = 0, need, k;
    const uint8_t avail = n_at_tick < ILINK_CRIT_BYTES ? n_at_tick : (uint8_t)ILINK_CRIT_BYTES;
    while ((need = il_decoder_cmd_need(&g_ild)) != 0u && need <= (uint8_t)(avail - n)) {   /* need >= 1 (API) */
        k = ilink_uart_read(buf, need);                   /* all there: counted at the tick, only this loop reads */
        ilink_feed(buf, k);
        n = (uint8_t)(n + k);
        if (k != need)
            break;
    }
    if (n_at_tick > avail && il_decoder_cmd_need(&g_ild) <= (uint8_t)(n_at_tick - n))
        g_il_crit_over++;                                 /* an IL_CMD can complete beyond the budget: background, 1 tick */
    return n;
}

/* background: the waiting bytes, at most `budget`, ILINK_CHUNK per ring read while such a chunk still ends before the
 * background's end, then single bytes; returns the bytes decoded */
static uint8_t ilink_rx_bg(uint8_t budget)
{
    uint8_t buf[ILINK_CHUNK], k, n = 0;
    while (n < budget && ilink_uart_rx_count() != 0u) {
        k = (uint8_t)(budget - n) < ILINK_CHUNK ? (uint8_t)(budget - n) : (uint8_t)ILINK_CHUNK;
        if (!ilink_time_left(BG_END_US, k)) {
            k = 1u;
            if (!ilink_time_left(BG_END_US, k))
                break;
        }
        k = ilink_uart_read(buf, k);
        ilink_feed(buf, k);
        n = (uint8_t)(n + k);
    }
    return n;
}

/* the idle wait: one ring read per call - ILINK_CHUNK bytes while such a chunk ends before the earliest frame, single
 * bytes after (the caller looks for the frame between two calls, so the tick starts at most one byte's decode late) */
static void ilink_rx_idle(void)
{
    uint8_t buf[ILINK_CHUNK], k;
    k = ilink_time_left(IDLE_END_US, ILINK_CHUNK) ? (uint8_t)ILINK_CHUNK : 1u;
    k = ilink_uart_read(buf, k);
    ilink_feed(buf, k);
}

static bool ilink_rx_waiting(void)
{
    return ilink_uart_rx_count() != 0u;
}

/* the reply snapshot -> IL_SNAP (VTOL: IL_SNAP2) frame -> ilink TX ring; built in place in g_frame (payload at its
 * frame offset) */
static bool snap_step(void)
{
    if (!g_snap_pending || ilink_uart_tx_free() < SNAP_FRAME_BYTES)
        return false;                                     /* nothing to send / the previous frames still shifting out */
#if FC_VTOL
    if (!bg_time_left())
        return false;                                     /* VTOL: the SNAP frame starts on BG_RESERVE_US (REC_RESERVE_US) */
#endif
#if FC_VTOL && ONB_HAS_GPS
    il_pack_snap3(&g_snap, g_frame + 4);
#elif FC_VTOL
    il_pack_snap2(&g_snap, g_frame + 4);
#else
    il_pack_snap(&g_snap, g_frame + 4);
#endif
    (void)il_encode(SNAP_TYPE, g_frame + 4, SNAP_LEN, g_frame);
    (void)ilink_uart_write(g_frame, SNAP_FRAME_BYTES);    /* room checked above: cannot be refused */
    g_snap_pending = false;
    g_snaps++;
    return true;
}

/* R2 (v0.6.3): 3 s, because the COM's STATs are up to ~2.5 s apart when the uplink stops (ilink_proto.h D4) */
static bool com_radio_ok(uint32_t tick)
{
    return g_com.have && (g_com.s.flags & IL_STAT_RADIO_OK) && (tick - g_com.s_tick) <= 3UL * FC_LOOP_HZ;
}

/* the newest IL_STAT: unpacked copy for the status record and the aux bit, 'C' record with the raw payload */
static bool com_stat_step(void)
{
    if (!g_com.fresh || uart_log_free() < (uint16_t)(LOG_COM_LEN + LOG_FRAME_OVERHEAD))
        return false;
    il_unpack_stat(g_com.raw, &g_com.s);
#if FC_VTOL
    if (g_com.s.params_tag != FC_PARAMS_TAG)
        g_images_mixed = true;                            /* fw09 review F1: the COM is of another configuration */
#endif
    g_com.s_tick = g_com.raw_tick;
    if (!g_com.have || (g_com.s.flags & IL_STAT_BOOT))
        g_com.announce = true;
    g_com.have = true;
    g_com.fresh = false;
    log_send(LOG_TYPE_COM, log_pack_com(g_com.raw_tick, g_com.raw, g_payload));
    return true;
}

/* one text record when the COM is first heard from and after every COM reset (IL_STAT_BOOT):
 * "COM boot: radio lora_uart ok, version 0x00, reset 0x01, params tag 0x48 match".  ~80 characters composed, framed
 * and queued in one step (~0.4 ms in simavr): started only with COM_TEXT_RESERVE_US left (it can wait a tick) */
#define COM_TEXT_RESERVE_US (BG_RESERVE_US + COM_TEXT_EXTRA_US)
static bool com_text_step(void)
{
    const il_stat_t *s = &g_com.s;
    uint8_t n;
    if (!g_com.announce || uart_log_free() < (uint16_t)(LOG_MAX_PAYLOAD + LOG_FRAME_OVERHEAD) ||
        !time_left_us(BG_END_US, COM_TEXT_RESERVE_US))
        return false;
#if FC_VTOL
    if (!g_text_ok)
        return false;                                     /* fw09: not on a predicted anchor (g_text_ok) */
#endif
    n = copy_P(0, (s->flags & IL_STAT_BOOT) ? PSTR("COM boot: radio ") : PSTR("COM already running: radio "));
    n = copy_P(n, (s->flags & IL_STAT_SX127X) ? PSTR("sx127x") : PSTR("lora_uart"));
    n = copy_P(n, (s->flags & IL_STAT_RADIO_OK) ? PSTR(" ok, version 0x") : PSTR(" FAIL, version 0x"));
    n = put_hex(n, s->radio_version, 2);
    n = copy_P(n, PSTR(", reset 0x"));
    n = put_hex(n, s->reset_cause, 2);
    n = copy_P(n, PSTR(", params tag 0x"));
    n = put_hex(n, s->params_tag, 2);
    if (s->params_tag == FC_PARAMS_TAG) {
        n = copy_P(n, PSTR(" match"));
    } else {
        n = copy_P(n, PSTR(" MISMATCH, FCU 0x"));
        n = put_hex(n, FC_PARAMS_TAG, 2);
    }
    log_send(LOG_TYPE_TEXT, n);
    g_com.announce = false;
    return true;
}

/* one text record when the NAV is first heard from and after every NAV reset (NL_F_BOOT appears):
 * "NAV boot: reset 0x01, flags 0x5F, loop max 1480 us" (hex) - on the same reserve as the COM's */
static bool nav_text_step(void)
{
    uint8_t n;
    if (!g_nl.announce || uart_log_free() < (uint16_t)(LOG_MAX_PAYLOAD + LOG_FRAME_OVERHEAD) ||
        !time_left_us(BG_END_US, COM_TEXT_RESERVE_US))
        return false;
#if FC_VTOL
    if (!g_text_ok)
        return false;                                     /* fw09: not on a predicted anchor (g_text_ok) */
#endif
    n = copy_P(0, (g_nav.flags & NL_F_BOOT) ? PSTR("NAV boot: reset 0x") : PSTR("NAV already running: reset 0x"));
    n = put_hex(n, g_nav.reset_cause, 2);
    n = copy_P(n, PSTR(", flags 0x"));
    n = put_hex(n, g_nav.flags, 2);
    n = copy_P(n, PSTR(", loop max 0x"));
    n = put_hex(n, g_nav.loop_us_max, 4);
    n = copy_P(n, PSTR(" us, I2C errors 0x"));
    n = put_hex(n, g_nav.twi_errors, 4);                  /* (v0.8 frame: saturated at 255) */
    log_send(LOG_TYPE_TEXT, n);
    g_nl.announce = false;
    return true;
}

/* 1 Hz 'N' record: the navigation frames received, the SPI slave's counters, the NAV's own figures */
static bool nav_record_step(void)
{
    log_nav_t r;
    const nlink_spi_stats_t *s;
    if (!g_nl.rec_due || uart_log_free() < (uint16_t)(LOG_NAV_LEN + LOG_FRAME_OVERHEAD))
        return false;
    s = nlink_spi_stats();
    r.tick = g_tick;
    r.frames = g_nl.frames;
    r.bad = g_nl.bad;
    r.timeouts = g_nl.timeouts;
    r.seq_lost = g_nl.seq_lost;
    r.resyncs = s->resyncs;
    r.spi_overruns = s->overruns;
    r.nav_loop_us_max = g_nl.loop_max;
    r.nav_flags = g_nl.have ? g_nav.flags : 0u;
    r.nav_reset_cause = g_nl.have ? g_nav.reset_cause : 0u;
    g_nl.loop_max = 0;
    g_nl.rec_due = false;
    log_send(LOG_TYPE_NAV, log_pack_nav(&r, g_payload));
    return true;
}

static bool evq_step(void)
{
    if (g_evq_n > 0u && uart_log_free() >= (uint16_t)(sizeof g_evq[0].d + LOG_FRAME_OVERHEAD)) {
        evq_t *e = &g_evq[g_evq_head];
        log_send_buf(e->type, e->d, e->len);
        g_evq_head = (uint8_t)((g_evq_head + 1u) % EVQ_N);
        g_evq_n--;
        return true;
    }
    return false;
}

#if FC_VTOL
/* the 'V' record (log format 6; 5 in v0.7.0): the vertical mission every LOG_VTOL_DIV flight records, captured with the
 * flight record after the tick's loop-time stamp, then written by two background steps of one job kind (BG_VREC): the
 * fixed-point pack, in place in g_vrec (core/log_record.h log_pack_vtol_in_place, ~0.11 ms), and the frame + the UART
 * ring (~0.09 ms).  v0.7.1: format 6's pack + frame in one step took 0.21 ms, more than REC_RESERVE_US. */
#define LOG_VTOL_DIV 5u                       /* every 5th flight record: 10 Hz at 250 Hz / FC_LOG_DIV 5 */
#if ONB_HAS_GPS
/* v0.8.1: the 'V' and the 'G' record share one buffer (they take turns in the same LOG_VTOL_DIV slot, 0.1 s apart: one is
 * written long before the next is captured - a capture that finds the other one unwritten replaces it, counted) - 40 B of
 * static RAM for the FCU's stack */
static union {
    log_vtol_t v;
    log_gns_t  g;
} g_vg;
#define g_vrec (g_vg.v)
#define g_grec (g_vg.g)
#else
static log_vtol_t g_vrec;                     /* the captured record; after the pack step its first LOG_VTOL_LEN bytes */
#endif
enum { VREC_NONE, VREC_CAPTURED, VREC_PACKED };
static uint8_t g_vrec_state;
static uint16_t g_vrec_dropped;               /* 'V' snapshots replaced before they were written (status record) */
static uint8_t g_vrec_div;
static uint8_t g_vgate;                       /* format 6: navigation frames with NAV_F_EST_GATE since the last 'V' record */

static bool vrec_step(void)
{
    if (g_vrec_state == VREC_CAPTURED) {
        (void)log_pack_vtol_in_place(&g_vrec);
        g_vrec_state = VREC_PACKED;
        return true;
    }
    if (g_vrec_state != VREC_PACKED || uart_log_free() < (uint16_t)(LOG_VTOL_LEN + LOG_FRAME_OVERHEAD))
        return false;
    log_send_buf(LOG_TYPE_VTOL, (const uint8_t *)(const void *)&g_vrec, LOG_VTOL_LEN);
    g_vrec_state = VREC_NONE;
    return true;
}
#if ONB_HAS_GPS
/* v0.8 'G' record (log format 7; v0.8.1 format 8 - its GNS form, core/log_record.h log_gns_t): captured at every
 * LOG_GPS_DIV-th 'V' slot, framed by one background step (the payload packed in place, as 'V') */
#define LOG_GPS_DIV 4u                        /* every 4th 'V' slot: 'G' 2.5 Hz, 'V' 7.5 Hz - the port's budget (below; with
                                               * 'G' on top of the 10 Hz 'V' it would need 5548 B/s > 95 % of 57600 baud) */
#if LOG_GNS_LEN != LOG_GPS_LEN
#error "the log port budget below counts LOG_GPS_LEN for the 'G' record"
#endif
static bool g_grec_due;
static uint8_t g_grec_div;
static uint16_t g_grec_dropped;               /* 'G' snapshots replaced before they were written (status record) */

static bool grec_step(void)
{
    if (!g_grec_due || uart_log_free() < (uint16_t)(LOG_GNS_LEN + LOG_FRAME_OVERHEAD))
        return false;
    (void)log_pack_gns_in_place(&g_grec);
    log_send_buf(LOG_TYPE_GPS, (const uint8_t *)(const void *)&g_grec, LOG_GNS_LEN);
    g_grec_due = false;
    return true;
}

/* round half away from zero, saturated to int16 */
static int16_t q16(float x, float scale)
{
    float v = x * scale;
    if (!(v == v))
        return 0;
    v += v < 0.0f ? -0.5f : 0.5f;
    if (v > 32767.0f) return 32767;
    if (v < -32767.0f) return -32767;
    return (int16_t)v;
}

/* the 'G' snapshot (in the tick, after the loop stamp; integers, two float conversions): the GNS slot receiver's newest set,
 * its newest TILT part, the setpoint the next attitude step flies */
static void grec_capture(uint32_t tick) __attribute__((noinline));
static void grec_capture(uint32_t tick)
{
    const nl_gns_rx_t *rx = onb_stack_gns(&g_stk);
    const nl_gns_data_t *d = &rx->latest;
    const float *t = onb_stack_att_tilt(&g_stk);
    uint8_t gf = 0;
    if (g_grec_due)
        g_grec_dropped++;                     /* the previous one was not written: replaced */
    if (g_vrec_state != VREC_NONE) {          /* the shared buffer still holds an unwritten 'V' record: replaced */
        g_vrec_dropped++;
        g_vrec_state = VREC_NONE;
    }
    g_grec.tick = tick;
    if (rx->has_latest) {
        g_grec.gps_flags = d->flags;
        g_grec.num_sv = d->num_sv;
        g_grec.hacc_dm = d->hacc_dm;
        g_grec.pc = d->pc;
        g_grec.pos_cm[0] = d->pos[0];
        g_grec.pos_cm[1] = d->pos[1];
        g_grec.vel_cms[0] = d->vel[0];
        g_grec.vel_cms[1] = d->vel[1];
        g_grec.trim[0] = d->trim[0];
        g_grec.trim[1] = d->trim[1];
        if (d->pc & NL_PC_HOLD) gf |= LOG_G4_HOLD;
        {
            const float dm = d->dist_m;
            g_grec.dist_dm = (dm == dm && dm >= 0.0f) ? (dm < 6553.4f ? (uint16_t)(dm * 10.0f + 0.5f) : 0xFFFEu)
                                                      : LOG_DIST_NONE;
        }
    } else {
        g_grec.gps_flags = g_grec.num_sv = g_grec.pc = 0;
        g_grec.hacc_dm = (uint8_t)LP_HACC_NONE;
        g_grec.pos_cm[0] = g_grec.pos_cm[1] = g_grec.vel_cms[0] = g_grec.vel_cms[1] = 0;
        g_grec.trim[0] = g_grec.trim[1] = 0;
        g_grec.dist_dm = LOG_DIST_NONE;
        gf |= LOG_G4_NO_SET;
    }
    if (rx->has_tilt) {
        g_grec.tilt_tgt[0] = rx->tilt[0];
        g_grec.tilt_tgt[1] = rx->tilt[1];
        if (rx->tilt_on) gf |= LOG_G4_ENGAGED;
    } else {
        g_grec.tilt_tgt[0] = NL_TILT_NONE;
        g_grec.tilt_tgt[1] = 0;
        gf |= LOG_G4_NO_TILT;
    }
    if (nl_gns_rx_fresh(rx)) gf |= LOG_G4_FRESH;
    if (nl_gns_rx_gate_ok(rx)) gf |= LOG_G4_GATE_OK;
    if (nl_gns_rx_fallback(rx)) gf |= LOG_G4_FALLBACK;
    if (nl_gns_rx_requests(rx, g_stk.land_steps) & ~(uint8_t)(d->pc >> NL_PC_REQ_SHIFT) & 0x01u) gf |= LOG_G4_OWN_LOST;
    g_grec.gflags = gf;
    g_grec.set_age = rx->has_latest ? rx->latest_age : 0xFFu;
    g_grec.tilt_mrad[0] = q16(t[0], 1000.0f);
    g_grec.tilt_mrad[1] = q16(t[1], 1000.0f);
    g_grec.sets = rx->sets;
    g_grec.broken = rx->broken;
    g_grec.tilts = rx->tilts;
    g_grec.bg_late = g_pos_late;
    g_grec_due = true;
}
/* B/s of the 'V' / 'G' slot: 'G' in one of LOG_GPS_DIV slots, 'V' in the others */
#define LOG_VG_BPS (((LOG_GPS_LEN + LOG_FRAME_OVERHEAD) + (LOG_GPS_DIV - 1u) * (LOG_VTOL_LEN + LOG_FRAME_OVERHEAD)) *        \
                    (FC_LOOP_HZ / FC_LOG_DIV / LOG_VTOL_DIV) / LOG_GPS_DIV)
#else
#define LOG_VG_BPS ((LOG_VTOL_LEN + LOG_FRAME_OVERHEAD) * (FC_LOOP_HZ / FC_LOG_DIV / LOG_VTOL_DIV))
#endif
/* the log port's budget (FC_LOG_BAUD 8N1): 'F' at FC_LOOP_HZ / FC_LOG_DIV, 'V' at that / LOG_VTOL_DIV ('G' in every
 * LOG_GPS_DIV-th of those slots, v0.8), 'S' 'N' 'C' at 1 Hz, 'L' at <= 5 uplinks a second (the ground's request rate is <=
 * its achievable poll rate, 5.04 Hz with sx127x), 'E' / 'T' rare; <= 95 % of the port (57600 baud: v0.7.1 5433 of 5760
 * B/s, 94.3 %; v0.8 with 'G': 5445 B/s, 94.5 %) */
#define LOG_LOAD_BPS ((LOG_FLIGHT_LEN + LOG_FRAME_OVERHEAD) * (FC_LOOP_HZ / FC_LOG_DIV) + LOG_VG_BPS +           \
                      (LOG_STATUS_LEN + LOG_FRAME_OVERHEAD) + (LOG_NAV_LEN + LOG_FRAME_OVERHEAD) +              \
                      (LOG_COM_LEN + LOG_FRAME_OVERHEAD) + 5 * (LOG_LINK2_LEN + LOG_FRAME_OVERHEAD))
#if 100UL * LOG_LOAD_BPS > 95UL * (FC_LOG_BAUD / 10UL)
#error "the VTOL log records exceed 95 % of the log port (FC_LOG_BAUD): raise LOG_VTOL_DIV / LOG_GPS_DIV"
#endif
#endif

#if ONB_HAS_GPS
/* v0.8 position hold in the background (core/onboard.h onb_stack_bg, core/attitude_ctrl.h attitude_ctrl_set_tilt): one
 * job = one piece of the position step in progress (P0..P10, <= 0.2 ms each) or the publication + tilt builder, then
 * attitude_ctrl_set_tilt (q(t), 141 us) and attitude_ctrl_set_tilt_finish (the product with the yaw hold, 84 us) - the
 * core agent's ATmega128A measurements.  First in the background's job list (POS_RESERVE_US before BG_END_US), then -
 * after the other jobs - on into the window of the NAV's next exchange: a piece is plain arithmetic (no interrupt-off
 * section, so the SPI slave interrupt keeps its latency), and it may start while it still ends before the earliest next
 * frame (FC_TICK_US - NLINK_EARLY_US after the anchor) even stretched by the exchange's slave interrupts (POS_WALL_US) -
 * a later frame only finds the loop already waiting.  What a tick could not finish is finished at the start of the next
 * tick, before its attitude controller (pos_finish_late: counted in g_pos_late) - the controller then flies exactly the
 * reference's setpoint. */
#define POS_RESERVE_US 230u                   /* > the longest piece (P7 191.5 us) + the RX interrupts of a busy ilink */
#define POS_WALL_US 350u                      /* the longest piece during an exchange: 192 us / (1 - 62 / 140), the slave
                                               * interrupt's share of a 140-cycle byte (hal/nlink_spi.h Timing) */
/* v0.8.1 GNS chain: one wall per job kind (the core agent's ATmega128A maxima x 140 / 78 + 10 us): the tilt builder 203 us,
 * attitude_ctrl_set_tilt 149 us, attitude_ctrl_set_tilt_finish 72 us */
#define POS_WALL_BUILDER_US 375u
#define POS_WALL_SET_US 280u
#define POS_WALL_FINISH_US 140u
#define POS_END_US (FC_TICK_US - NLINK_EARLY_US)   /* the earliest next frame after the anchor */
static uint8_t g_tilt_stage;                  /* 0 done, 1 set_tilt due, 2 set_tilt_finish due */

/* v0.8.1 GNS chain: the tilt builder (onb_stack_bg = pos_tilt_tick toward the slot's TILT, or the newest set's TRIM in the
 * fallback, active while the GNS says engaged and the motor runs) changes nothing when its inputs - the wire integers of
 * its target, the fallback / engaged / motor flags - are those of its previous run and that run left the tilt exactly on
 * the target (t == target bit for bit: dx = dy = +0, t + 0 = t; the target, int16 x 1e-4, is never -0) or, inactive, at
 * (+0, +0).  Then the call is skipped (its only other effect, this tick's background flag, is cleared here): the setpoint
 * is the same bits, attitude_ctrl_set_tilt has nothing to do either.  In a steady hover a TILT part arrives once per 5
 * ticks and the builder reaches it in one tick (|step| <= slew_per_tick): 4 of 5 builder runs are skipped - the ~0.15 ms a
 * run costs (core agent's measurement: 111 / 203 us) the background cannot spare (tools/run_simbench.py). */
static struct {
    int16_t src[2];                           /* the target's wire integers at the last run */
    uint8_t fl;                               /* bit 0 fallback, 1 engaged, 2 motor_on, 3 a set exists */
    bool    valid, at;                        /* a run was recorded; it left the tilt on its target */
} g_bld;
static uint16_t g_bld_skipped;                /* builder runs skipped (simbench report) */
static bool g_bld_ran;                        /* the last builder_step ran the builder (not skipped) */

static void builder_inputs(int16_t src[2], uint8_t *fl)
{
    const nl_gns_rx_t *rx = onb_stack_gns(&g_stk);
    const bool fb = nl_gns_rx_fallback(rx);
    *fl = (uint8_t)((fb ? 1u : 0u) | (rx->tilt_on ? 2u : 0u) | (g_stk.motor_on ? 4u : 0u) | (rx->has_latest ? 8u : 0u));
    if (!fb) {
        src[0] = rx->tilt[0];
        src[1] = rx->tilt[1];
    } else if (rx->has_latest) {
        src[0] = rx->latest.trim[0];
        src[1] = rx->latest.trim[1];
    } else {
        src[0] = src[1] = 0;
    }
}

static uint32_t f32_bits_of(float x)
{
    union {
        float f;
        uint32_t u;
    } v;
    v.f = x;
    return v.u;
}

/* one builder step: run it (and record where it left the tilt) or skip a no-op; returns onb_stack_bg's value */
static bool builder_step(void)
{
    int16_t src[2];
    uint8_t fl;
    bool more;
    builder_inputs(src, &fl);
    if (g_bld.valid && g_bld.at && fl == g_bld.fl && src[0] == g_bld.src[0] && src[1] == g_bld.src[1]) {
        g_stk.bg_left = false;                /* the builder's flag: this tick's background work is done */
        g_bld_ran = false;                    /* nothing changed: no attitude_ctrl_set_tilt either */
        if (g_bld_skipped < 0xFFFFu)
            g_bld_skipped++;
        return false;
    }
    more = onb_stack_bg(&g_stk);
    if (!more) {
        const float *t = onb_stack_att_tilt(&g_stk);
        float tgt[2];
        const bool eng = nl_gns_rx_target(onb_stack_gns(&g_stk), tgt);
        g_bld.src[0] = src[0];
        g_bld.src[1] = src[1];
        g_bld.fl = fl;
        g_bld.valid = true;
        g_bld.at = (eng && g_stk.motor_on)
                       ? (f32_bits_of(t[0]) == f32_bits_of(tgt[0]) && f32_bits_of(t[1]) == f32_bits_of(tgt[1]))
                       : (f32_bits_of(t[0]) == 0u && f32_bits_of(t[1]) == 0u);
    }
    g_bld_ran = true;
    return more;
}

static bool pos_job_until(uint16_t limit_us, uint16_t reserve_us)
{
    if (g_tilt_stage == 0u && !onb_stack_bg_pending(&g_stk))
        return false;                         /* this tick's position work is done */
    if (!time_left_us(limit_us, reserve_us))
        return false;
    if (onb_stack_bg_pending(&g_stk)) {
        if (!builder_step() && g_bld_ran)
            g_tilt_stage = 1u;                /* the builder ran: its tilt goes to the attitude controller next */
    } else if (g_tilt_stage == 1u) {
        g_tilt_stage = attitude_ctrl_set_tilt(&g_ctrl, onb_stack_att_tilt(&g_stk)) ? 2u : 0u;
    } else {
        attitude_ctrl_set_tilt_finish(&g_ctrl);
        g_tilt_stage = 0u;
    }
    return true;
}

static bool pos_job(void)                     /* in the background's job list */
{
    return pos_job_until(BG_END_US, POS_RESERVE_US);
}

/* after the other jobs: on into the next exchange's window */
static bool pos_job_late(void)
{
    return pos_job_until(POS_END_US, onb_stack_bg_pending(&g_stk) ? POS_WALL_BUILDER_US
                                     : (g_tilt_stage == 1u ? POS_WALL_SET_US : POS_WALL_FINISH_US));
}

/* v0.8.1 GNS chain: the tilt work of this tick (the builder, attitude_ctrl_set_tilt, its finish) right after the
 * time-critical part, before every other background job and without a time check: the reference order puts the builder
 * at the end of the FCU tick (sim/onboard.py "FCU tick order v0.8.1", step 7) and the next attitude controller flies its
 * tilt.  v0.8 ran these pieces as time-checked jobs and finished what was left at the next tick start, before the
 * attitude controller (pos_finish_late); with the GNS chain in position hold that late finish (~0.3 ms) lengthened the
 * next time-critical part, which shortened its background, which left more work late: the tick overran and the ilink
 * starved (tools/run_simbench.py, v0.8.1 integration: V1 LAND before HOLD).  Here the work is bounded (builder <= 203 us,
 * set_tilt 149 us, finish 72 us: the core agent's maxima) and done once, at a fixed place: no late finish, the attitude
 * controller's input is the reference's, the time-critical part does not grow.  (Skipped: the builder when it would
 * change nothing, builder_step; set_tilt when the tilt did not change, attitude_ctrl_set_tilt itself.) */
static void pos_work_now(void)
{
    while (onb_stack_bg_pending(&g_stk))
        if (!builder_step() && g_bld_ran)
            g_tilt_stage = 1u;
    if (g_tilt_stage == 1u)
        g_tilt_stage = attitude_ctrl_set_tilt(&g_ctrl, onb_stack_att_tilt(&g_stk)) ? 2u : 0u;
    if (g_tilt_stage == 2u) {
        attitude_ctrl_set_tilt_finish(&g_ctrl);
        g_tilt_stage = 0u;
    }
}

/* at the tick start, before the attitude controller: whatever the previous tick's background left (v0.8.1: nothing - the
 * tilt work runs right after the time-critical part, pos_work_now; kept as the guard the numbers rely on) */
static void pos_finish_late(void)
{
    if (g_tilt_stage == 0u && !onb_stack_bg_pending(&g_stk))
        return;
    while (onb_stack_bg_pending(&g_stk))
        if (!builder_step() && g_bld_ran)
            g_tilt_stage = 1u;
    if (g_tilt_stage == 1u && attitude_ctrl_set_tilt(&g_ctrl, onb_stack_att_tilt(&g_stk)))
        g_tilt_stage = 2u;                    /* (attitude_ctrl_update finishes a half-built setpoint itself) */
    g_tilt_stage = 0u;
    if (g_pos_late < 0xFFFFu)
        g_pos_late++;
}
#endif

/* One bounded background job per call, in priority order; returns its kind (simbench), BG_NONE when idle.
 * Not inlined: the jobs' locals live only during the background call, not on top of the tick's stack. */
enum { BG_NONE, BG_SNAP, BG_ILINK, BG_NAVREC, BG_RECORD, BG_FLOG, BG_COM, BG_COMTEXT, BG_NAVTEXT, BG_VREC, BG_POS, BG_GREC,
       BG_STATUS,     /* fw09: BG_STATUS = the status record, simbench report only (it is not one of bg_job's jobs) */
       BG_POSNOW };   /* fw09 integration: pos_work_now (no reserve: right after the time-critical part), simbench only */
static uint8_t bg_job(void) __attribute__((noinline));
/* the background's ilink share: at most ILINK_BG_BYTES a tick (D6) */
static bool ilink_share_step(void)
{
    if (g_il_bg_left != 0u) {
        const uint8_t n = ilink_rx_bg(g_il_bg_left);
        if (n != 0u) {
            g_il_bg_left = (uint8_t)(g_il_bg_left - n);
            if (g_il_bg_left == 0u)
                g_il_bg_capped++;
            return true;
        }
    }
    return false;
}

static uint8_t bg_job(void)
{
#if ONB_HAS_GPS
    if (pos_job())
        return BG_POS;                        /* v0.8, first: the next tick's attitude controller needs the tilt setpoint */
#endif
    if (snap_step())
        return BG_SNAP;                       /* first: the COM is waiting for it to answer the ground */
#if !FC_VTOL
    if (ilink_share_step())                   /* then the ilink, ahead of the jobs below for ILINK_BG_BYTES a tick */
        return BG_ILINK;
#endif
    if (evq_step())
        return BG_RECORD;
#if FC_VTOL
    if (vrec_step())                          /* 'V' (one per 25 ticks) ahead of the flight-record parts (6 per 5) */
        return BG_VREC;
#endif
#if ONB_HAS_GPS
    if (grec_step())                          /* 'G' (one per 100 ticks) */
        return BG_GREC;
#endif
    if (flight_log_step())
        return BG_FLOG;
#if FC_VTOL
    /* VTOL: the ilink share after the flight record.  The VTOL time-critical part leaves a background window that a
     * babbling COM's 16-byte share would take every tick (the log then starves: 22 flight records lost in a 0.44 s
     * sub-flood babble, measured); the idle decode after the background takes those bytes anyway, and an IL_CMD decoded
     * there still reaches the next tick's supervisor, so the command latency does not change */
    if (ilink_share_step())
        return BG_ILINK;
#endif
    if (com_stat_step())
        return BG_COM;
    if (nav_record_step())
        return BG_NAVREC;
    if (com_text_step())
        return BG_COMTEXT;                    /* started on its own reserve, COM_TEXT_RESERVE_US */
    if (nav_text_step())
        return BG_NAVTEXT;                    /* the same */
    return BG_NONE;
}

/* 1 Hz status record: FCU health, the NAV's sensor counters from its newest frame, the COM's link counters from its
 * newest IL_STAT, the FCU's ilink counters */
static void status_record(uint32_t tick, uint8_t reset_cause, float v_bat) __attribute__((noinline));
static void status_record(uint32_t tick, uint8_t reset_cause, float v_bat)
{
    log_status_t st;
    const ilink_uart_stats_t *us = ilink_uart_stats();
    const il_dec_stats_t *ds = &g_ild.stats;
    st.tick = tick;
    st.stack_free = stack_free_min_step();
    st.twi_errors = g_nav.twi_errors;                 /* v0.6.3: the NAV's I2C (from its newest valid frame) */
#if ONB_HAS_GPS
    st.uart_dropped = (uint16_t)(uart_log_dropped() + log_flight_dropped + g_evq_dropped + g_vrec_dropped + g_grec_dropped);
#elif FC_VTOL
    st.uart_dropped = (uint16_t)(uart_log_dropped() + log_flight_dropped + g_evq_dropped + g_vrec_dropped);
#else
    st.uart_dropped = (uint16_t)(uart_log_dropped() + log_flight_dropped + g_evq_dropped);
#endif
    if (g_com.have) {
        const il_stat_t *c = &g_com.s;
        const uint32_t age = (tick - g_com.s_tick) / FC_LOOP_HZ;
        st.link_frames = c->dec_frames;
        st.link_bad = (uint16_t)(c->dec_bad_crc + c->dec_bad_len);
        st.link_other = c->dec_other;
        st.radio_errors = (uint16_t)(c->radio_rx_errors + c->radio_rx_overflow + c->radio_tx_timeouts + c->replies_dropped);
        st.replies = c->replies;
        st.rssi_dbm = c->rssi_dbm;
        st.com_age_s = (uint8_t)(age > 254u ? 254u : age);
        st.com_flags = (uint8_t)(c->flags | (c->params_tag != FC_PARAMS_TAG ? LOG_COM_TAG_MISMATCH : 0u));
    } else {
        st.link_frames = st.link_bad = st.link_other = st.radio_errors = st.replies = 0;
        st.rssi_dbm = LOG_RSSI_NONE;
        st.com_age_s = LOG_COM_AGE_NONE;
        st.com_flags = 0;
    }
    st.baro_fails = g_nav.baro_fails;                 /* the NAV's MS5611 failures since its boot */
    st.imu_fails = g_nav.imu_fail;                    /* the NAV's consecutive failed IMU reads */
    st.vbat_mv = (uint16_t)(v_bat * 1000.0f);
    st.reset_cause = reset_cause;
    st.il_cmds = g_il_cmds;
    st.il_snaps = g_snaps;
    st.il_bad = (uint16_t)(ds->bad_crc + ds->bad_len + ds->bad_type);
    st.il_uart_errors = (uint16_t)(us->rx_errors + us->rx_overflow);   /* il_decoder_push never counts overflow */
    st.il_dropped = (uint16_t)(g_il_dropped + g_snap_dropped);
    log_send(LOG_TYPE_STATUS, log_pack_status(&st, g_payload));
}

/* wait for the next tick: a navigation exchange (the normal case) or, when none comes, the FCU's own clock -
 * NLINK_FRAME_TIMEOUT_US after the previous tick start, FC_TICK_US after it when that tick already ran without one
 * (Timer2).  Polls; decodes ilink bytes as they arrive meanwhile.  nlink_spi_frame_ready applies the slave's resync
 * rule on every call, the timeout path included. */
enum { TICK_FRAME_SEEN, TICK_FRAME_LATE, TICK_TIMEOUT };
static uint8_t wait_for_tick(uint16_t t_prev, bool fallback_prev)
{
    if (nlink_spi_frame_ready())
        return TICK_FRAME_LATE;                           /* completed while the previous tick was still busy */
    for (;;) {
        SB_PEERS();                                       /* simbench: the synthetic NAV's frame, the COM's bytes */
        if (nlink_spi_frame_ready())
            return TICK_FRAME_SEEN;
        if (fallback_prev ? timer_period_due()
                          : timer_elapsed_counts(t_prev, timer_tcnt1()) >= NLINK_FRAME_TIMEOUT_T1_COUNTS)
            return nlink_spi_frame_ready() ? TICK_FRAME_SEEN : TICK_TIMEOUT;
        if (ilink_rx_waiting())
            ilink_rx_idle();
    }
}

#if defined(FC_SIMBENCH_MICRO) && !defined(FC_COSIM)        /* feeds the RX ring directly: single-chip bench only */
/* this file's own ilink paths on real frames, one timed call each (SBM lines, CPU cycles, 8-cycle resolution);
 * each with a fresh anchor, so the time checks see a whole tick */
static uint32_t micro_cycles(uint16_t a)
{
    return (uint32_t)timer_elapsed_counts(a, timer_tcnt1()) * (uint32_t)(FC_F_CPU_HZ / 1000000UL / FC_PWM_COUNTS_PER_US);
}

static void micro_ilink_paths(void) __attribute__((noinline));
static void micro_ilink_paths(void)
{
    /* the frames live in g_frame (the SNAP step at the end overwrites them) and the simbench's scratch (unused before
     * the loop): no large locals in this build.  g_frame: STAT frame [0, 42), CMD frame [48, 62), C5 5C 31 02 sync spam
     * [64, 96); scratch: noise after the CMD */
    uint8_t *const pay = simbench_scratch(), *const fs = g_frame, *const fc = g_frame + 48, *const spam = g_frame + 64, i, n;
    const il_cmd_t c = {1, {LP_CMD_NONE, 0, 0}, -70, 28};
    uint8_t j2[2] = {0x00, 0x5C};                         /* junk in front of a CMD: discarded while hunting */
    uint16_t a, lfsr = 0xACE1u;
    for (i = 0; i < IL_STAT_LEN; i++) pay[i] = (uint8_t)(i * 5u + 3u);
    il_encode(IL_TYPE_STAT, pay, IL_STAT_LEN, fs);
    il_pack_cmd(&c, pay);
    il_encode(IL_TYPE_CMD, pay, IL_CMD_LEN, fc);
    for (i = 0; i < 32u; i++) {
        static const uint8_t pat[4] PROGMEM = {0xC5, 0x5C, 0x31, 0x02};
        spam[i] = pgm_read_byte(&pat[i & 3u]);
        lfsr = (uint16_t)((lfsr >> 1) ^ (-(uint16_t)(lfsr & 1u) & 0xB400u));
        pay[i] = (i % 5u) == 0u ? IL_SYNC0 : (uint8_t)lfsr;          /* noise with a 0xC5 every 5th byte */
    }
    il_decoder_init(&g_ild, IL_ACCEPT_FCU);
    ilink_uart_inject_rx(fs, IL_STAT_FRAME_BYTES);
    ilink_uart_inject_rx(fc, IL_CMD_FRAME_BYTES);
    g_anchor = a = timer_tcnt1();
    (void)ilink_rx_bg(0xFFu);
    simbench_micro_line(PSTR("main_ilink_rx_bg_stat_cmd_56B"), micro_cycles(a));
    ilink_uart_inject_rx(fc, IL_CMD_FRAME_BYTES);
    a = timer_tcnt1();
    (void)ilink_rx_crit(IL_CMD_FRAME_BYTES);
    simbench_micro_line(PSTR("main_ilink_rx_crit_cmd_14B"), micro_cycles(a));
    g_anchor = timer_tcnt1();                             /* the lines above took time: a fresh time base */
    ilink_uart_inject_rx(fc, 4u);
    (void)ilink_rx_bg(0xFFu);                             /* the head through the background path */
    ilink_uart_inject_rx(fc + 4, IL_CMD_FRAME_BYTES - 4u);
    a = timer_tcnt1();
    (void)ilink_rx_crit(IL_CMD_FRAME_BYTES - 4u);
    simbench_micro_line(PSTR("main_ilink_rx_crit_cmd_tail_10B"), micro_cycles(a));
    ilink_uart_inject_rx(j2, 2u);                         /* step 2's budget filled: 2 junk bytes + a CMD = 16 bytes */
    ilink_uart_inject_rx(fc, IL_CMD_FRAME_BYTES);
    a = timer_tcnt1();
    n = ilink_rx_crit(ILINK_CRIT_BYTES);
    simbench_micro_line(PSTR("main_ilink_rx_crit_junk2_cmd_16B"), micro_cycles(a));
    simbench_micro_line(PSTR("main_check_crit_bytes_of_16"), n);
    il_decoder_init(&g_ild, IL_ACCEPT_FCU);
    ilink_uart_inject_rx(spam, ILINK_CRIT_BYTES);         /* 16 bytes of sync spam waiting at a tick */
    a = timer_tcnt1();
    n = ilink_rx_crit(ILINK_CRIT_BYTES);
    simbench_micro_line(PSTR("main_ilink_rx_crit_syncspam_16B"), micro_cycles(a));
    simbench_micro_line(PSTR("main_check_crit_bytes_syncspam"), n);
    il_decoder_init(&g_ild, IL_ACCEPT_FCU);
    ilink_uart_inject_rx(spam, 32u);                      /* 32 bytes of babble through the background path, two patterns */
    g_anchor = a = timer_tcnt1();
    n = ilink_rx_bg(32u);
    simbench_micro_line(PSTR("main_ilink_rx_bg_syncspam_32B"), micro_cycles(a));
    il_decoder_init(&g_ild, IL_ACCEPT_FCU);
    ilink_uart_inject_rx(pay, 32u);
    g_anchor = a = timer_tcnt1();
    n = (uint8_t)(n + ilink_rx_bg(32u));
    simbench_micro_line(PSTR("main_ilink_rx_bg_noise_32B"), micro_cycles(a));
    simbench_micro_line(PSTR("main_check_bg_bytes_of_64"), n);
    a = timer_tcnt1();
    n = il_decoder_cmd_need(&g_ild);
    simbench_micro_line(PSTR("main_il_decoder_cmd_need_call"), micro_cycles(a));
    g_snap_pending = true;
    g_anchor = a = timer_tcnt1();                         /* a fresh time base: the SNAP job checks its reserve (VTOL) */
    (void)snap_step();
    simbench_micro_line(PSTR("main_snap_step_55B"), micro_cycles(a));
    (void)ilink_uart_take_tx(pay, IL_STAT_LEN);
    (void)ilink_uart_take_tx(pay, IL_STAT_LEN);
    simbench_micro_line(PSTR("main_check_cmds_decoded_of_4"), g_il_cmds);     /* the four CMD frames fed above */
    g_ncmd = 0;
    g_il_cmds = g_snaps = 0;
    g_il_crit_over = 0;
    g_com.fresh = false;
    il_decoder_init(&g_ild, IL_ACCEPT_FCU);
}
#endif

static float lpf_alpha(float f_c, float dt)
{
    return dt / (1.0f / (2.0f * FC_PI_F * f_c) + dt);
}

#if FC_LOGIC_BAT
/* 2026-11-10 (hal/adc.h FC_LOGIC_BAT): the logic battery for the reply snapshot (SNAP2 spare / SNAP3 spare3 -> the COM's
 * TELEM v6 / v7), sim/link_proto.py lbat_q_from_counts; never LP_LBAT_NONE once read (boot) */
static uint8_t g_lbat_q;
static uint16_t g_lbat_div;
#if defined(FC_SIMBENCH)
#define SB_LBAT_COUNTS 772u                              /* simbench: a synthetic 3.86 V logic pack (no ADC model) */
#endif

static uint8_t lbat_q_of(uint16_t counts)
{
    const uint32_t q = ((uint32_t)counts * LOGIC_BAT_Q_X1024 + 512u) >> 10;
    return q < 1u ? 1u : (q > 255u ? 255u : (uint8_t)q);
}
#endif

/* the battery ADC result of the conversion started in the previous tick (simbench: a synthetic 4.0 V per cell) */
static bool battery_sample(uint16_t *counts)
{
#if defined(FC_SIMBENCH)
#if FC_LOGIC_BAT
    if (++g_lbat_div >= LOGIC_BAT_DIV_TICKS) {        /* the flight build's PF3 slot: no V_cell sample this tick */
        g_lbat_div = 0;
        g_lbat_q = lbat_q_of(SB_LBAT_COUNTS);
        return false;
    }
#endif
    *counts = simbench_adc();
    return true;
#elif PM_ANY
    *counts = adc_chain_vin();                        /* v0.9: the chain's V_in (the tether node / the bus) */
    return *counts != PM_CODE_NONE;
#elif FC_LOGIC_BAT
    /* 2026-11-10: every LOGIC_BAT_DIV_TICKS ticks the conversion started now is PF3's; a PF3 result updates g_lbat_q and
     * is not a V_cell sample (the 2 Hz V_cell filter skips that tick) */
    uint8_t ch;
    uint8_t next;
    if (g_lbat_div < LOGIC_BAT_DIV_TICKS)
        g_lbat_div++;
    next = g_lbat_div >= LOGIC_BAT_DIV_TICKS ? (uint8_t)LOGIC_BAT_ADC_CHANNEL : (uint8_t)VBAT_ADC_CHANNEL;
    if (!adc_poll_mux(counts, &ch, next))
        return false;                                 /* still converting (cannot in 4 ms): the PF3 slot waits a tick */
    if (next == (uint8_t)LOGIC_BAT_ADC_CHANNEL)
        g_lbat_div = 0;
    if (ch == (uint8_t)LOGIC_BAT_ADC_CHANNEL) {
        g_lbat_q = lbat_q_of(*counts);
        return false;
    }
    return true;
#else
    return adc_poll(counts);
#endif
}

#if !FC_VTOL
/* LED / buzzer patterns from a free-running tick counter (power-of-two periods: no division) */
static void indicators(uint16_t ph, uint8_t st, bool ready, bool bench, bool recovery, bool low_batt)
{
    const bool fast = (ph & 0x20u) != 0;               /* toggles every 32 ticks  */
    const bool slow = (ph & 0x80u) != 0;               /* toggles every 128 ticks */
    const bool flash = (ph & 0xFFu) < 0x18u;           /* short flash every 256 ticks */
    bool led, buz = false;
    if (recovery) {
        led = fast;
        buz = fast;
    } else if (bench) {
        led = slow;
    } else {
        switch (st) {
        case ONB_SAFE:
            led = ready ? flash : fast;                /* ready = aligned, sensors ok, link alive */
            buz = low_batt && fast;
            break;
        case ONB_ARMED:
            led = true;
            buz = flash;
            break;
        case ONB_POWERED:
        case ONB_UNPOWERED:
            led = true;
            break;
        case ONB_RELEASED:
            led = slow;
            buz = (ph & 0x1FFu) < 0x30u;               /* locator beep */
            break;
        default:                                       /* ABORT */
            led = fast;
            break;
        }
    }
    gpio_led(led);
    gpio_buzzer(buz);
}
#endif

#if FC_VTOL
/* VTOL (v0.7): the LED shows the vertical phase, the buzzer the spool warning (the supervisor's output: ARMED = the
 * motor starts when it ends - SPEC 8.9 "spool_warning_s buzzer warning") and the landed / released / safe states
 *   SAFE      LED: ready ? short flash : fast blink; buzzer only for a low battery (as ballistic)
 *   SPOOL     LED on; buzzer fast beeps (4 Hz) for the whole warning - stand clear
 *   CLIMB     LED 2 Hz blink           HOLD  LED on            LAND  LED fast blink (4 Hz)
 *   LANDED    LED slow blink (1 Hz), buzzer silent (motor off, SAFE follows after safe_after_landed_s)
 *   ABORT     LED fast blink           RELEASED  LED slow blink + locator beep (as ballistic)
 * (power-of-two periods of the free-running tick counter: no division) */
static void indicators_vtol(uint16_t ph, uint8_t st, uint8_t phase, bool spool_buzzer, bool ready, bool bench,
                            bool recovery, bool low_batt)
{
    const bool fast = (ph & 0x20u) != 0;               /* toggles every 32 ticks  */
    const bool mid = (ph & 0x40u) != 0;                /* toggles every 64 ticks  */
    const bool slow = (ph & 0x80u) != 0;               /* toggles every 128 ticks */
    const bool flash = (ph & 0xFFu) < 0x18u;           /* short flash every 256 ticks */
    bool led, buz = false;
    if (bench) {
        led = slow;
    } else if (spool_buzzer) {
        led = true;
        buz = fast;
    } else {
        switch (st) {
        case ONB_SAFE:
            led = ready ? flash : fast;
            buz = low_batt && fast;
            break;
        case ONB_ARMED:                                /* never without the buzzer output in this build */
            led = true;
            buz = fast;
            break;
        case ONB_POWERED:
            led = phase == LP_PHASE_HOLD ? true : (phase == LP_PHASE_LAND ? fast : mid);
            break;
        case ONB_UNPOWERED:                            /* LANDED */
            led = slow;
            break;
        case ONB_RELEASED:
            led = slow;
            buz = (ph & 0x1FFu) < 0x30u;               /* locator beep */
            break;
        default:                                       /* ABORT (also the I10 boot) */
            led = fast;
            buz = recovery && fast;
            break;
        }
    }
    gpio_led(led);
    gpio_buzzer(buz);
}
#endif

/* the FCU frame the next exchange returns to the NAV (its pad / flight switch, core/nlink_proto.h).  thr_q (v0.7.1,
 * byte 7): the VTOL build's ESC throttle of this tick, nl_fcu_throttle_q(u) - the u the ESC was actually given (0 with
 * the motor off; a bench build's cap included) - for the NAV's baro thrust compensation; the ballistic build sends 0,
 * the v0.6.3 "reserved" byte, so its exchange bytes are v0.6.3's */
#if FC_VTOL
/* v0.8.1: the nlink v4 FCU frame (NL_TYPE_FCU4) - the v0.7.1 fields + pflags / kt_q of this tick: the GNS chain's
 * onb_stack_fcu4_* (the GNS position controller's FCU inputs, forwarded by the NAV in its glink STATE); the build without
 * position hold sends the phase / motor bits and k_T 1.0 (no GNS reads them: a vehicle without the GNS board) */
static void nav_reply(uint8_t state, bool bench, bool recovery, uint8_t thr_q)
{
    nl_fcu4_t r;
    r.seq = g_nl.last_seq;
    r.state = state;
    r.fflags = (uint8_t)((bench ? NL_FF_BENCH : 0u) | (recovery ? NL_FF_RECOVERY : 0u));
    r.params_tag = FC_PARAMS_TAG;
    r.thr_q = thr_q;
#if ONB_HAS_GPS
    r.pflags = onb_stack_fcu4_pflags(&g_stk);
    r.kt_q = onb_stack_fcu4_kt_q(&g_stk);
#else
    r.pflags = (uint8_t)((g_so.phase & NL_PF_PHASE_MASK) | (g_so.motor_on ? NL_PF_MOTOR_ON : 0u));
    r.kt_q = (uint16_t)NL_KT_SCALE;
#endif
    (void)nl_encode_fcu4(&r, g_nlbuf);
    nlink_spi_set_reply(g_nlbuf);
}
#else
static void nav_reply(uint8_t state, bool bench, bool recovery, uint8_t thr_q)
{
    nl_fcu_t r;
    r.seq = g_nl.last_seq;
    r.state = state;
    r.fflags = (uint8_t)((bench ? NL_FF_BENCH : 0u) | (recovery ? NL_FF_RECOVERY : 0u));
    r.params_tag = FC_PARAMS_TAG;
    r.thr_q = thr_q;
    (void)nl_encode_fcu(&r, g_nlbuf);
    nlink_spi_set_reply(g_nlbuf);
}
#endif

/* boot-time parameter structs live only in these frames, not in main's for the whole flight */
static void controller_init(void) __attribute__((noinline));
static void controller_init(void)
{
    mixer_plant_t plant;
    att_gains_t gains;
    fc_default_plant(&plant);
    fc_default_gains(&gains);
    attitude_ctrl_init(&g_ctrl, &gains, &plant, fc_throttle_exp_int());
}

static void supervisor_init(uint16_t seed, bool recovery) __attribute__((noinline));
static void supervisor_init(uint16_t seed, bool recovery)
{
#if FC_VTOL
    /* v0.8.1 (core/onboard.h, core/fc_config.h): the VTOL AVR build keeps the supervisor's and the vertical controller's
     * tables in flash (fc_onboard_params / fc_vertical_params: flash addresses, never a RAM copy); its gps_hold is
     * compile-time (off in an ONB_NO_GPS build: no ON gate, no V11-V14, TELEM v2); the GPS build runs the GNS chain (only
     * the position table's land_steps / slew_per_tick are read on the FCU) */
#if ONB_HAS_GPS
    onb_stack_init(&g_stk, fc_onboard_params(), fc_vertical_params(), fc_position_params(), ONB_CHAIN_GNS, seed, recovery);
#else
    onb_stack_init(&g_stk, fc_onboard_params(), fc_vertical_params(), 0, ONB_CHAIN_GNS, seed, recovery);
#endif
#else
    onb_params_t op;
    fc_default_onboard(&op);
    op.mode = (uint8_t)FC_MISSION_MODE;               /* this build's mission (fc_mode.h): params.json's unless pinned */
    onboard_init(&g_sup, &op, seed, recovery);
#endif
}

/* ------------------------------------------------------------------------ */
int main(void)
{
    static uint16_t out[PWM_N_CH];
    const uint8_t reset_cause = watchdog_reset_cause();
    const bool recovery = flight_record_inflight_reset();
    uint8_t i;
    uint16_t overruns = 0, loop_us_max = 0, loop_us_last = 0, loop_us_link = 0, t0;
    uint8_t arm_db = 0, prev_state = ONB_SAFE, wait = TICK_TIMEOUT;
    bool arm_sw = false, prev_active = false, prev_bench = false, frame_rx_seen = false, fallback = false;
    bool have_vbat = false, status_due = false, prev_ctrl_shown = true;
    float v_bat = 0.0f, v_cell = 0.0f, thrust_lag = 0.0f;
    uint32_t tick = 0;
    uint8_t ph_log = 0;                                   /* tick % FC_LOG_DIV */
    uint16_t ph_sec = 0;                                  /* tick % FC_LOOP_HZ */
    uint16_t seed;
#ifdef FC_SIMBENCH
    static sb_tick_t sb;                                  /* this tick's figures for the report (simbench.h) */
    uint16_t sb_baro_late = 0;                            /* ticks with baro_ok false after the first valid baro */
    uint16_t pwm_us = 0;
    bool sb_baro_seen = false;
#endif
    const float a_vbat = lpf_alpha(CAL_VBAT_LPF_HZ, FC_DT_S);

    /* ---- 0. safe outputs before anything else -------------------------------- */
#if FC_VTOL
    g_images_mixed = false;                               /* fw09 review F1: .noinit, so cleared at every boot here */
#endif
    k_esc_off_counts = pwm_us_to_counts(CAL_ESC_OFF_US);
    k_release_open_counts = pwm_us_to_counts(CAL_RELEASE_RELEASED_US);
    k_release_latched_counts = pwm_us_to_counts(CAL_RELEASE_LATCHED_US);
    controller_init();
    gpio_init();
    /* I10: ballistic boots SAFE with the release open; VTOL boots ABORT and the supervisor opens it after
     * abort_release_delay_s (its timer), so the release stays latched here */
    safe_outputs(out, FC_VTOL ? false : recovery);
    pwm_init(out);
    {
        uint8_t m = (uint8_t)(MCUCSR | (1 << JTD));      /* JTAG off even if the JTAGEN fuse is left programmed */
        MCUCSR = m;
        MCUCSR = m;
    }
    watchdog_start();
    timer_init();                                         /* FCU: no timer interrupt (hal/timer.h) */
    uart_log_init();
    ilink_uart_init();                                    /* TXD1 idle-high towards the COM, RX ring empty */
    il_decoder_init(&g_ild, IL_ACCEPT_FCU);               /* the FCU receives IL_CMD and IL_STAT only (R4) */
    nlink_spi_init();                                     /* SPI slave, PB0 SS with pull-up: an absent NAV = idle */
    g_nav.q[0] = 1.0f;                                    /* until the first frame: identity attitude, zero rates */
    nav_reply(ONB_SAFE, false, recovery, 0u);             /* the first exchange already returns a valid FCU frame */
    adc_init();
    sei();
#ifdef FC_SIMBENCH_MICRO
    simbench_micro(&g_ctrl, &g_co, &g_sup, &g_so, &g_rec, simbench_scratch(), g_frame);
    simbench_micro_ilink(&g_so, simbench_scratch(), g_frame, &g_ild);
    simbench_micro_nlink(simbench_scratch(), g_frame);
#if FC_VTOL
    simbench_micro_vtol(&g_stk, &g_so, simbench_scratch(), g_frame);   /* vc_update, onb_stack_step, SNAP2, 'V' record */
#endif
#if !defined(FC_COSIM)
    micro_ilink_paths();                                  /* feeds the RX ring directly: single-chip bench only */
#endif
    controller_init();
    nlink_spi_init();                                     /* the benchmarks injected / took exchanges and set replies */
    il_decoder_init(&g_ild, IL_ACCEPT_FCU);               /* ... and used the ilink decoder */
    nav_reply(ONB_SAFE, false, recovery, 0u);
#endif
#ifdef FC_COSIM
    simbench_cosim_init();                                /* the tick marker port (simbench.h) */
#endif

    seed = adc_entropy_seed(LINK_NONCE_SEED_CONVERSIONS);   /* LAUNCH nonce seed (SPEC 10.3) */
#if FC_LOGIC_BAT && defined(FC_SIMBENCH)
    g_lbat_q = lbat_q_of(SB_LBAT_COUNTS);
#elif FC_LOGIC_BAT
    g_lbat_q = lbat_q_of(adc_read_blocking(LOGIC_BAT_ADC_CHANNEL));   /* 2026-11-10: valid from the first reply */
#endif
#if PM_ANY
    adc_chain_init();                                     /* v0.9: the power monitor's conversion chain */
#if PM_BUF
    FC_CAT(DDR, PIN_BUF_ISIGN_PORT) &= (uint8_t)~(1u << PIN_BUF_ISIGN_BIT);   /* buffer-current sign: input */
#endif
#endif
    supervisor_init(seed, recovery);
#if FC_VTOL
    g_so.state = recovery ? ONB_ABORT : ONB_SAFE;         /* until the first tick (onboard_init's state) */
    g_so.phase = LP_PHASE_NONE;
    g_so.motor_on = g_so.buzzer = false;
    g_so.throttle_milli = 0;
    g_so.control_active = false;
    g_so.release = false;
#else
    g_so.state = ONB_SAFE;
    g_so.throttle_milli = 0;
    g_so.control_active = false;
    g_so.release = recovery;
#endif
    fc_default_edf(&g_edf);

    log_boot(reset_cause);
    if (recovery)
#if FC_VTOL
    {   /* fw09 T2: the VTOL vehicle has no recovery device (no parachute, piston, release servo or latch): its RELEASED is
         * the motor locked off until a power cycle (the release pulse on PE5 drives nothing).  The text, 122 characters +
         * the state's 4 hex digits, is longer than one TEXT record (LOG_MAX_PAYLOAD 96: the v0.8.1 VTOL text, 103
         * characters, lost its "last state" digits): two records, their texts concatenated = the whole sentence */
        uint8_t n;
        log_boot_text(copy_P(0, PSTR("RECOVERY (I10): reset in flight -> ABORT, motor off and locked until power cycle ")));
        n = copy_P(0, PSTR("(VTOL: no recovery device); last state 0x"));
        log_boot_text(put_hex(n, flight_record_state(), 4));
    }
#else
        log_text_hex_P(PSTR("RECOVERY (I10): reset in flight, release opened, arming locked; last state 0x"),
                       flight_record_state());
#endif
    else
        flight_record_store(ONB_SAFE);

#ifdef FC_SIMBENCH
#ifdef FC_COSIM
    log_text_hex_P(PSTR("COSIM FCU build: real nlink SPI slave and ilink USART1, loop-time measurement, stack 0x"),
                   stack_reserve_total());
#else
    log_text_hex_P(PSTR("SIMBENCH FCU build: synthetic NAV and COM, loop-time measurement, stack 0x"),
                   stack_reserve_total());
    simbench_start();                                     /* the synthetic peers' clock starts here */
#endif
#endif

    /* ---- the control loop ----------------------------------------------------- */
    t0 = timer_tcnt1();
    g_anchor = t0;
    for (;;) {
        uint8_t n_at_tick, n_crit;
        bool nav_ok, powered, bench, flying;
        uint8_t flags = 0, aux = 0, nfl;
        float u;
        onb_inputs_t in;

        /* ---- wait for the navigation frame (or the FCU's own clock) ---- */
        {
            wait = wait_for_tick(t0, fallback);
            timer_period_restart();                       /* the fallback clock counts from here */
            tick++;
            g_tick = tick;
            SB_TICK_START(tick, wait != TICK_TIMEOUT);    /* simbench: timing, the COM's bytes that arrived before now */
            t0 = timer_tcnt1();                           /* the tick starts */
            SB_COSIM_TICK(tick);                          /* co-simulation: tick marker up (simbench.h) */
            n_at_tick = ilink_uart_rx_count();            /* step 2 decodes (some of) these */
            ilink_uart_tx_hold(true);                     /* no per-byte TX interrupt in the time-critical part */
            SB_MARK(0);
            watchdog_kick();
        }

        /* 1. the navigation exchange; arm switch */
        nav_ok = false;
        if (wait != TICK_TIMEOUT && nlink_spi_take(g_nlbuf)) {
            if (NAV_DECODE(g_nlbuf, &g_nav)) {             /* fills g_nav only if valid: the newest valid one is kept */
                const bool boot = (g_nav.flags & NL_F_BOOT) != 0u;
                nav_ok = true;
                if (!g_nl.have || (boot && !g_nl.boot_prev))
                    g_nl.announce = true;                 /* first frame, or the NAV rebooted (its BOOT flag reappeared) */
                else
                    g_nl.seq_lost = (uint16_t)(g_nl.seq_lost + (uint8_t)(g_nav.seq - g_nl.last_seq - 1u));
                g_nl.boot_prev = boot;
                g_nl.have = true;
                g_nl.last_seq = g_nav.seq;
                g_nl.frames++;
                if (g_nav.loop_us_max > g_nl.loop_max)
                    g_nl.loop_max = g_nav.loop_us_max;
#if FC_VTOL
                if (g_nav.reset_cause & FCU_NAV_RC_TAG_MISMATCH)
                    g_images_mixed = true;                /* fw09 review F1: the NAV is of another configuration */
#endif
            } else {
                g_nl.bad++;
            }
        } else {
            g_nl.timeouts++;
        }
        if (!nav_ok && overruns < 0xFFFFu)
            overruns++;                                   /* a tick without a valid navigation frame */
        fallback = wait == TICK_TIMEOUT;
#ifdef FC_SIMBENCH
        if (simbench_arm_switch()) {
#else
        if (gpio_arm_switch_raw()) {
#endif
            if (arm_db < CAL_ARM_DEBOUNCE_TICKS) arm_db++;
        } else if (arm_db > 0) {
            arm_db--;
        }
        if (arm_db == CAL_ARM_DEBOUNCE_TICKS) arm_sw = true;
        if (arm_db == 0) arm_sw = false;
        SB_MARK(1);

#if FC_VTOL
        /* VTOL (v0.7.1, params.json mission.vtol.fcu_tick_order_note, core/onboard.h onb_stack_*): 2. the attitude
         * controller FIRST, on this frame's q / rates with the PREVIOUS tick's supervisor control_active and ESC throttle
         * (its gain schedule), 3. the vane pulses written right after - no supervisor, vertical controller or ilink
         * decode on the frame -> vane path any more (v0.7.0 measured 5.15 ms sample -> vane with them in front) - then
         * 4. ilink RX, 5. the FCU stack (supervisor -> vertical controller), 6. the ESC.  A supervisor decision reaches
         * the ESC in this tick and the vanes in the next (4 ms); the gain schedule sees u one tick old (sim/onboard.py
         * FcuStack, the order the cross-check replays and the 6-DOF flies).  The sensor booleans come first: the bench
         * build's vane test needs them before the controller. */
        flying = onboard_flying(prev_state);
        nfl = nav_ok ? g_nav.flags : 0u;
        in.imu_ok = (nfl & NL_F_IMU_OK) && ((nfl & NL_F_EST_VALID) || !flying);
        in.baro_ok = (nfl & NL_F_BARO_OK) != 0u;
        in.aligned = (nfl & NL_F_ALIGNED) != 0u;
        in.gyro_saturated = (nfl & NL_F_GYRO_SAT) != 0u;
#ifdef FC_BENCH_VANES
        bench = prev_state == ONB_SAFE && !arm_sw && in.imu_ok && in.aligned && !recovery;
#else
        bench = false;
#endif
#if ONB_HAS_GPS
        {   /* v0.8: the setpoint of the previous tick's position work (normally done in its background) */
#ifdef FC_SIMBENCH
            const uint16_t a = timer_tcnt1();
            const uint16_t late0 = g_pos_late;
#endif
            pos_finish_late();
#ifdef FC_SIMBENCH
            if (g_pos_late != late0)
                simbench_pos_finish((uint16_t)(timer_elapsed_counts(a, timer_tcnt1()) / FC_PWM_COUNTS_PER_US));
#endif
        }
#endif
        /* 2. attitude control on the frame's attitude and rates (the previous tick's supervisor outputs) */
        if (bench && !prev_bench)
            attitude_ctrl_set_yaw_reference(&g_ctrl, g_nav.q);
        prev_bench = bench;
        {
            const bool active = onb_stack_att_active(&g_stk) || bench;
            if (active && !prev_active)
                attitude_ctrl_reset(&g_ctrl);
            prev_active = active;
            if (active) {
                const float u_att = fc_minf(onb_stack_att_u(&g_stk), FC_THROTTLE_LIMIT);   /* = the ESC's last u */
#ifdef FC_BENCH_VANES
                attitude_ctrl_update(&g_ctrl, g_nav.q, g_nav.rate, bench ? FC_HOVER_THROTTLE : u_att, &g_co);
#else
                attitude_ctrl_update(&g_ctrl, g_nav.q, g_nav.rate, u_att, &g_co);
#endif
                flags |= LOG_F_CONTROL_ACTIVE;
                prev_ctrl_shown = true;
            } else {
                if (prev_ctrl_shown) {                        /* vanes centred (+trim): computed once */
                    for (i = 0; i < 4; i++) g_co.delta_cmd[i] = 0.0f;
                    for (i = 0; i < 3; i++) {
                        g_co.moments[i] = 0.0f;
                        g_co.att_error[i] = 0.0f;
                        g_co.integrator[i] = 0.0f;
                    }
                    g_co.saturated = false;
                    mixer_pwm(&g_ctrl.mixer, g_co.delta_cmd, g_co.pwm_us);
                    prev_ctrl_shown = false;
                }
                for (i = 0; i < 3; i++) g_co.gyro_filtered[i] = g_nav.rate[i];
            }
        }
        SB_MARK(2);
        /* 3. the vane pulses: the end of the frame -> vane path (the 6-DOF's FC_COMPUTE_DELAY_S_MEASURED) */
        vane_counts(g_co.pwm_us, out);
        for (i = 0; i < 4; i++)
            pwm_write(k_out_channel[OUT_VANE1 + i], out[OUT_VANE1 + i]);
        SB_MARK(3);
#ifdef FC_SIMBENCH
        pwm_us = (uint16_t)(timer_elapsed_counts(t0, timer_tcnt1()) / FC_PWM_COUNTS_PER_US);
#endif

        /* 4. ilink RX: of the bytes counted at the tick start (n_at_tick), those an IL_CMD can complete in - the same
         * IL_CMDs reach this tick's supervisor as when this ran before the controller (not while the RX ring overflows:
         * D6 flood) */
        n_crit = (n_at_tick && !g_il_flood) ? ilink_rx_crit(n_at_tick) : 0u;
        (void)n_crit;                                     /* simbench report only */
        SB_MARK(4);

        /* 5. the FCU stack (sim/onboard.py FcuStack): the supervisor - the IL_CMD frames decoded since its previous call,
         * the sensor booleans above, the vertical controller's flags and the ESC throttle of the PREVIOUS tick
         * (onb_stack_step fills in.vc_* / throttle_prev) - then the vertical controller on this frame's h, v_z, cos(tilt)
         * from q and body-z specific force -> the ESC command (the controller's u while POWERED, else 0) */
        {
            const uint8_t nf = g_ncmd;
            frame_rx_seen = frame_rx_seen || nf > 0u;
#ifdef FC_SIMBENCH
            sb.n_cmds = nf;
            if (nf) {
                sb.seq_first = g_cmdq[0].seq;
                sb.seq_last = g_cmdq[nf - 1u].seq;
            }
            if (in.baro_ok) {
                sb_baro_seen = true;
            } else if (nav_ok && sb_baro_seen && sb_baro_late < 0xFFFFu) {
                sb_baro_late++;
            }
#endif
            in.tick = tick;
            in.frames = g_cmdq;
            in.n_frames = nf;
            in.h_m = g_nav.h_m;
            in.vz_mps = g_nav.vz_mps;
            in.tilt_deg = g_nav.tilt_deg;
            for (i = 0; i < 3; i++) in.gyro[i] = g_nav.rate[i];
            in.v_cell = v_cell;                                 /* filtered up to the previous tick (step 6) */
            /* fw09 review F1: a mixed image set holds the arming chain open on the ground (g_images_mixed, above) */
            in.arm_switch = arm_sw && !(g_images_mixed && (prev_state == ONB_SAFE || prev_state == ONB_ARMED));
            in.roll_deg = in.pitch_deg = in.yaw_deg = 0.0f;    /* telemetry only: the COM computes them from the SNAP's q */
            in.loop_max_us = (float)loop_us_link;
            /* the vertical controller's NAV inputs (v0.8): the newest valid frame's cos_tilt (nlink v3: of the published
             * attitude; 1 before the first frame, the v0.7.1 identity quaternion) and body-z specific force (NaN before
             * the first frame: no support signature).  Held while no frame comes, exactly like h / v_z / tilt above - the
             * 6-DOF's frozen NAV outputs; a dead NAV is I9 after ONB_IMU_FAIL_TICKS anyway.  The GPS slot only from a
             * valid frame of this tick (the slot receiver ages its parts otherwise) */
            in.cos_tilt = g_nl.have ? g_nav.cos_tilt : 1.0f;
            in.accel_z_mps2 = g_nl.have ? g_nav.accel_z : NAN;
            in.gps_slot = nav_ok ? g_nav.slot : 0;
#if PM_ANY
            {                                             /* v0.9: the chain started in the previous tick; the next one */
#ifdef FC_SIMBENCH                                        /* runs now, right after the exchange (core/power_mon.h) */
                simbench_adc_chain(in.pw.code);
#else
                adc_chain_take(in.pw.code);
#endif
#if PM_BUF
                in.pw.buf_i_pos = (FC_CAT(PIN, PIN_BUF_ISIGN_PORT) & (1u << PIN_BUF_ISIGN_BIT)) != 0u;
#else
                in.pw.code[PM_CH_TET] = in.pw.code[PM_CH_BUF] = PM_CODE_NONE;
                in.pw.buf_i_pos = false;
#endif
            }
#endif
            u = onb_stack_rt(&g_stk, &in, &g_so);         /* the time-critical part; the position pieces: background */
        }
        if (g_so.state != prev_state && !recovery)
            flight_record_store(g_so.state);
        powered = g_so.state == ONB_POWERED;
        u = fc_minf(u, FC_THROTTLE_LIMIT);                      /* bench builds cap the ESC (the controller does not know) */
        SB_MARK(5);
#else  /* ballistic: the v0.6.3 order, unchanged */
        /* 2. ilink RX: an IL_CMD complete on the wire before this tick reaches this tick's supervisor (not while the
         * RX ring overflows: D6 flood) */
        n_crit = (n_at_tick && !g_il_flood) ? ilink_rx_crit(n_at_tick) : 0u;
        (void)n_crit;                                     /* simbench report only */
        SB_MARK(2);

        /* 3. onboard supervisor: the IL_CMD frames decoded since its previous call, the sensor booleans of
         * core/nlink_proto.h (no valid frame -> all false) */
        flying = onboard_flying(prev_state);
        nfl = nav_ok ? g_nav.flags : 0u;
        in.imu_ok = (nfl & NL_F_IMU_OK) && ((nfl & NL_F_EST_VALID) || !flying);
        in.baro_ok = (nfl & NL_F_BARO_OK) != 0u;
        in.aligned = (nfl & NL_F_ALIGNED) != 0u;
        in.gyro_saturated = (nfl & NL_F_GYRO_SAT) != 0u;
#ifdef FC_BENCH_VANES
        bench = prev_state == ONB_SAFE && !arm_sw && in.imu_ok && in.aligned && !recovery;
#else
        bench = false;
#endif
        {   /* g_cmdq is filled only by the main loop (step 2, background, idle wait): no copy needed */
            const uint8_t nf = g_ncmd;
            frame_rx_seen = frame_rx_seen || nf > 0u;
#ifdef FC_SIMBENCH
            sb.n_cmds = nf;                                     /* the report's record of what this supervisor got */
            if (nf) {
                sb.seq_first = g_cmdq[0].seq;
                sb.seq_last = g_cmdq[nf - 1u].seq;
            }
            if (in.baro_ok) {
                sb_baro_seen = true;
            } else if (nav_ok && sb_baro_seen && sb_baro_late < 0xFFFFu) {
                sb_baro_late++;                                 /* a valid frame without NL_F_BARO_OK */
            }
#endif
            in.tick = tick;
            in.frames = g_cmdq;
            in.n_frames = nf;
            in.h_m = g_nav.h_m;
            in.vz_mps = g_nav.vz_mps;
            in.tilt_deg = g_nav.tilt_deg;
            for (i = 0; i < 3; i++) in.gyro[i] = g_nav.rate[i];
            in.v_cell = v_cell;                                 /* filtered up to the previous tick (step 6) */
            in.arm_switch = arm_sw;
            in.roll_deg = in.pitch_deg = in.yaw_deg = 0.0f;    /* telemetry only: the COM computes them from the SNAP's q */
            in.loop_max_us = (float)loop_us_link;
            onboard_update(&g_sup, &in, &g_so);
        }
        if (g_so.state != prev_state && !recovery)
            flight_record_store(g_so.state);
        powered = g_so.state == ONB_POWERED;
        u = fc_minf((float)g_so.throttle_milli * (1.0f / (float)LP_THROTTLE_SCALE), FC_THROTTLE_LIMIT);
        SB_MARK(3);

        /* 4. control on the frame's attitude and rates; the vane pulses written right after: the end of the frame ->
         * vane path, the control loop whose delay the 6-DOF simulation models */
        if (bench && !prev_bench)
            attitude_ctrl_set_yaw_reference(&g_ctrl, g_nav.q);
        prev_bench = bench;
        {
            const bool active = g_so.control_active || bench;
            if (active && !prev_active)
                attitude_ctrl_reset(&g_ctrl);
            prev_active = active;
            if (active) {
#ifdef FC_BENCH_VANES
                attitude_ctrl_update(&g_ctrl, g_nav.q, g_nav.rate, bench ? FC_HOVER_THROTTLE : u, &g_co);
#else
                attitude_ctrl_update(&g_ctrl, g_nav.q, g_nav.rate, u, &g_co);
#endif
                flags |= LOG_F_CONTROL_ACTIVE;
                prev_ctrl_shown = true;
            } else {
                if (prev_ctrl_shown) {                        /* vanes centred (+trim): computed once */
                    for (i = 0; i < 4; i++) g_co.delta_cmd[i] = 0.0f;
                    for (i = 0; i < 3; i++) {
                        g_co.moments[i] = 0.0f;
                        g_co.att_error[i] = 0.0f;
                        g_co.integrator[i] = 0.0f;
                    }
                    g_co.saturated = false;
                    mixer_pwm(&g_ctrl.mixer, g_co.delta_cmd, g_co.pwm_us);
                    prev_ctrl_shown = false;
                }
                for (i = 0; i < 3; i++) g_co.gyro_filtered[i] = g_nav.rate[i];
            }
        }
        SB_MARK(4);
        vane_counts(g_co.pwm_us, out);
        for (i = 0; i < 4; i++)
            pwm_write(k_out_channel[OUT_VANE1 + i], out[OUT_VANE1 + i]);
        SB_MARK(5);
#ifdef FC_SIMBENCH
        pwm_us = (uint16_t)(timer_elapsed_counts(t0, timer_tcnt1()) / FC_PWM_COUNTS_PER_US);
#endif
#endif /* FC_VTOL */

        /* 5. ESC and release pulses (the supervisor's outputs: they do not wait for the controller, but the vanes do not
         * wait for them either - written right after they are computed, ~45 us after the vanes; VTOL: right after the
         * FCU stack) */
        out[OUT_ESC] = (powered && !bench) ? esc_counts(true, u) : k_esc_off_counts;
        out[OUT_RELEASE] = g_so.release ? k_release_open_counts : k_release_latched_counts;
        pwm_write(k_out_channel[OUT_ESC], out[OUT_ESC]);
        pwm_write(k_out_channel[OUT_RELEASE], out[OUT_RELEASE]);
        if (g_so.state == ONB_ARMED && prev_state != ONB_ARMED)
            attitude_ctrl_set_yaw_reference(&g_ctrl, g_nav.q);   /* the arming heading (the controller is off in ARMED) */

        /* 6. this tick's anchor (when its frame completed - seen arriving: the tick start; overtaken by a late tick or
         * missing: one tick after the previous anchor, unless that lies after the tick start (an early frame, the first
         * after a NAV restart): the tick start), phases, battery (2 Hz LPF, for the next tick's supervisor), records,
         * reply snapshot, the NAV's reply, indicators, loop-time stamp */
#if FC_VTOL
        g_text_ok = wait != TICK_FRAME_LATE;                  /* fw09 fix loop: text records not on a predicted anchor */
#endif
        if (wait == TICK_FRAME_SEEN) {
            g_anchor = t0;
        } else {
            const uint16_t pred = t1_add(g_anchor, T1_COUNTS(FC_TICK_US));
            g_anchor = timer_elapsed_counts(pred, t0) <= T1_COUNTS(FC_TICK_US / 2u) ? pred : t0;
        }
        if (++ph_log >= FC_LOG_DIV)
            ph_log = 0;
        if (++ph_sec >= FC_LOOP_HZ)
            ph_sec = 0;
        {
            uint16_t adc_counts;
            if (battery_sample(&adc_counts)) {
                const float v = (float)adc_counts * VBAT_V_PER_COUNT;
                v_bat = have_vbat ? v_bat + a_vbat * (v - v_bat) : v;
                have_vbat = true;
                v_cell = v_bat / (float)VBAT_CELLS;
            }
        }
        for (i = 0; i < g_so.n_trans; i++) {                    /* records packed now, written in the background */
            evq_t *e = evq_slot(LOG_TYPE_EVENT);
#if FC_VTOL                                                     /* format 5: + the phases (a phase change: from == to) */
            if (e)
                e->len = log_pack_event2(tick, g_so.trans[i].from, g_so.trans[i].to, g_so.trans[i].cause, g_so.reason,
                                         g_so.trans[i].phase_from, g_so.trans[i].phase_to, e->d);
#else
            if (e)
                e->len = log_pack_event(tick, g_so.trans[i].from, g_so.trans[i].to, g_so.trans[i].cause, g_so.reason,
                                        e->d);
#endif
        }
        for (i = 0; i < g_so.n_exec; i++) {
            evq_t *e = evq_slot(LOG_TYPE_LINK);
#if FC_VTOL                                                     /* format 5: + the phase after the command */
            if (e)
                e->len = log_pack_link2(tick, g_so.exec[i].seq, g_so.exec[i].cmd, g_cmdq[i].cmd.arg, g_cmdq[i].cmd.nonce,
                                        g_so.exec[i].code, g_so.exec[i].state_after, g_cmdq_rssi[i], g_cmdq_snr[i],
                                        g_so.exec[i].phase_after, e->d);
#else
            if (e)
                e->len = log_pack_link(tick, g_so.exec[i].seq, g_so.exec[i].cmd, g_cmdq[i].cmd.arg, g_cmdq[i].cmd.nonce,
                                       g_so.exec[i].code, g_so.exec[i].state_after, g_cmdq_rssi[i], g_cmdq_snr[i], e->d);
#endif
        }
        g_ncmd = 0;                                             /* consumed */
        if (g_so.reply) {                                       /* reply snapshot now, framed in the background */
            if (g_snap_pending)
                g_snap_dropped++;                               /* the previous one never left: the newest wins */
#if FC_VTOL && ONB_HAS_GPS
            il_snap3_from_tlm_src(g_so.reply_seq, &g_so.tlm, g_nav.q, &g_snap);   /* + the GPS fields (TELEM v3) */
#if FC_LOGIC_BAT
            g_snap.spare3 = g_lbat_q;                            /* 2026-11-10: the logic battery -> TELEM v7 */
#endif
#elif FC_VTOL
            il_snap2_from_tlm_src(g_so.reply_seq, &g_so.tlm, g_nav.q, &g_snap);   /* + phase, target (TELEM v2) */
#if FC_LOGIC_BAT
            g_snap.spare = g_lbat_q;                             /* 2026-11-10: the logic battery -> TELEM v6 */
#endif
#else
            il_snap_from_tlm_src(g_so.reply_seq, &g_so.tlm, g_nav.q, &g_snap);
#endif
            g_snap_pending = true;
            loop_us_link = 0;
        }
#if FC_VTOL                                                     /* returned by the next exchange; + the ESC's u (thr_q) */
        nav_reply(g_so.state, bench, recovery, nl_fcu_throttle_q((powered && !bench) ? u : 0.0f));
        if (nav_ok && (g_nav.flags & NAV_F_EST_GATE) && g_vgate < 0xFFu)
            g_vgate++;                                          /* the NAV's estimator correction, for the 'V' record */
#else
        nav_reply(g_so.state, bench, recovery, 0u);             /* returned by the next exchange */
#endif
        prev_state = g_so.state;
#if FC_VTOL
        indicators_vtol((uint16_t)tick, g_so.state, g_so.phase, g_so.buzzer, in.aligned &&
                        (g_so.flags & LP_FLAG_SENSORS_OK) && (g_so.flags & LP_FLAG_LINK_OK), bench, recovery,
                        have_vbat && v_cell < ONB_MIN_CELL_V);
#else
        indicators((uint16_t)tick, g_so.state, in.aligned && (g_so.flags & LP_FLAG_SENSORS_OK) &&
                   (g_so.flags & LP_FLAG_LINK_OK), bench, recovery, have_vbat && v_cell < ONB_MIN_CELL_V);
#endif

        /* loop time of the time-critical part, from the frame (Timer1 counts at FC_PWM_COUNTS_PER_US per us) */
        loop_us_last = (uint16_t)(timer_elapsed_counts(t0, timer_tcnt1()) / FC_PWM_COUNTS_PER_US);
        if (loop_us_last > loop_us_max)
            loop_us_max = loop_us_last;
        if (loop_us_last > loop_us_link)
            loop_us_link = loop_us_last;                        /* reported in the next SNAP */
        SB_MARK(6);
        SB_COSIM_CRIT_END(tick);                                /* co-simulation: tick marker down */
        ilink_uart_tx_hold(false);                              /* the SNAP bytes go out in the background and the wait */
        {   /* D6: this tick's ilink share ahead of the jobs - none while the RX ring overflows (flood) */
            const uint16_t ovf = ilink_uart_stats()->rx_overflow;
            g_il_flood = ovf != g_il_ovf_seen;
            g_il_ovf_seen = ovf;
            if (g_il_flood)
                g_il_flood_ticks++;
            g_il_bg_left = g_il_flood ? 0u : (uint8_t)ILINK_BG_BYTES;
        }
        /* 7. the EDF thrust estimate (first-order lag, every tick): logged only, so after the loop-time stamp */
        thrust_lag = edf_step_fixed(&g_edf, thrust_lag, powered ? u : 0.0f);

        /* log snapshot every FC_LOG_DIV ticks; the packing runs in the background below */
        if (ph_log == 0) {
            if (g_co.saturated) flags |= LOG_F_SATURATED;
            if (g_so.release) flags |= LOG_F_RELEASE;
            if (g_so.flags & LP_FLAG_LINK_OK) flags |= LOG_F_LINK_OK;
            if (g_so.flags & LP_FLAG_INTERLOCK) flags |= LOG_F_INTERLOCK;
            if (arm_sw) flags |= LOG_F_ARM_SWITCH;
            if (g_so.flags & LP_FLAG_SENSORS_OK) flags |= LOG_F_SENSORS_OK;
            if (bench) flags |= LOG_F_BENCH;
            if (in.aligned) aux |= LOG_A_ALIGNED;
            if (recovery) aux |= LOG_A_RECOVERY;
            if (g_snap_pending || ilink_uart_tx_busy()) aux |= LOG_A_REPLY_PENDING;
            if (frame_rx_seen) aux |= LOG_A_FRAME_RX;
            if (g_so.flags & LP_FLAG_REJECTED) aux |= LOG_A_ACK_REJECTED;
            if (com_radio_ok(tick)) aux |= LOG_A_RADIO_OK;
            frame_rx_seen = false;
            if (flight_log_busy())
                log_flight_dropped++;                          /* previous record not finished: replaced */
            g_rec.tick = tick;
            g_rec.state = g_so.state;
            g_rec.flags = flags;
            g_rec.reason = g_so.reason;
            g_rec.aux_bits = aux;
            g_rec.loop_us_last = loop_us_last;
            g_rec.loop_us_max = loop_us_max;
            g_rec.overruns = overruns;
            for (i = 0; i < 4; i++) {
                g_rec.q[i] = g_nav.q[i];
                g_rec.delta[i] = g_co.delta_cmd[i];
            }
            for (i = 0; i < 3; i++) {
                g_rec.gyro_f[i] = g_co.gyro_filtered[i];
                g_rec.accel[i] = NAV_ACCEL(i);
                g_rec.att_err[i] = g_co.att_error[i];
                g_rec.moments[i] = g_co.moments[i];
                g_rec.integ[i] = g_co.integrator[i];
            }
            for (i = 0; i < PWM_N_CH; i++)
                g_rec.pwm_counts[i] = out[i];
            g_rec.h_est = g_nav.h_m;
            g_rec.vz_est = g_nav.vz_mps;
            g_rec.h_baro = NAV_H_BARO_REL();
            g_rec.throttle = powered ? u : 0.0f;
            g_rec.v_cell = v_cell;
            g_rec.tilt_deg = g_nav.tilt_deg;
            g_rec.thrust_est = thrust_lag;
            flight_log_start();
            loop_us_max = 0;
#if FC_VTOL
            if (++g_vrec_div >= LOG_VTOL_DIV) {                 /* the 'V' record, every LOG_VTOL_DIV flight records */
#if ONB_HAS_GPS
                if (++g_grec_div >= LOG_GPS_DIV) {              /* v0.8: every LOG_GPS_DIV-th slot carries the 'G' record */
                    g_vrec_div = 0;
                    g_grec_div = 0;
                    grec_capture(tick);
                } else
#endif
              {
                const vc_outputs_t *vo = &g_stk.vo;
                const uint32_t mt = g_stk.sup.powered_ticks;
                uint8_t vf = 0;
                g_vrec_div = 0;
                if (g_vrec_state != VREC_NONE)
                    g_vrec_dropped++;                           /* the previous one was not written: replaced */
#if ONB_HAS_GPS
                if (g_grec_due) {                               /* v0.8.1: the shared buffer's unwritten 'G': replaced */
                    g_grec_dropped++;
                    g_grec_due = false;
                }
#endif
                if (g_so.motor_on) vf |= LOG_V_MOTOR_ON;
                if (g_so.buzzer) vf |= LOG_V_BUZZER;
                if (g_so.baro_lost) vf |= LOG_V_BARO_LOST;
                if (g_stk.have_vo) {
                    if (vo->at_target) vf |= LOG_V_AT_TARGET;
                    if (vo->touchdown) vf |= LOG_V_TOUCHDOWN;
                    if (vo->sat > 0) vf |= LOG_V_SAT_HIGH;
                    if (vo->sat < 0) vf |= LOG_V_SAT_LOW;
                }
                g_vrec.tick = tick;
                g_vrec.phase = g_so.phase;
                g_vrec.vflags = vf;
                g_vrec.reason = g_so.reason;
                g_vrec.vc_mode = g_stk.have_vo ? vo->mode : VC_MODE_IDLE;
                g_vrec.h_tgt_dm = g_so.h_tgt_dm;
                g_vrec.h_ref = vo->h_ref;
                g_vrec.vz_cmd = vo->vz_cmd;
                g_vrec.u = vo->u;
                g_vrec.u_ff = vo->u_ff;
                g_vrec.integ = vo->integ;
                g_vrec.td_count = vo->td_count;
                g_vrec.motor_ticks = (uint16_t)(mt > 0xFFFFUL ? 0xFFFFUL : mt);
                g_vrec.sig = g_stk.have_vo ? vo->sig : 0u;      /* format 6: the touchdown detector v2 */
                g_vrec.td_by = g_stk.have_vo ? vo->td_by : VC_TD_NONE;
                g_vrec.low_in_land = g_stk.have_vo ? vo->low_in_land_ticks : 0u;
                g_vrec.vz_lpf = g_stk.vc.v_lpf;
                g_vrec.f_excess = g_stk.vc.d_lpf;
                g_vrec.est_gate = g_vgate;                      /* ... and the NAV's estimator correction */
                g_vgate = 0;
                g_vrec_state = VREC_CAPTURED;                   /* packed and written by vrec_step (two steps) */
              }
            }
#endif
        } else if (ph_sec == 1) {
            status_due = true;
        } else if (ph_sec == 2) {
            g_nl.rec_due = true;
        }

#if ONB_HAS_GPS
        {   /* v0.8.1: the tilt work of this tick, first (pos_work_now) */
#ifdef FC_SIMBENCH
            const uint16_t a = timer_tcnt1();
#endif
            pos_work_now();
#ifdef FC_SIMBENCH
            simbench_bg_step((uint16_t)(timer_elapsed_counts(a, timer_tcnt1()) / FC_PWM_COUNTS_PER_US), BG_POSNOW);
#endif
        }
#endif
        /* background until BG_RESERVE_US before the next exchange: one bounded job per pass, SNAP first */
        for (;;) {
            uint8_t job;
#ifdef FC_SIMBENCH
            uint16_t a;
#endif
            SB_PEERS();                                       /* simbench: the synthetic NAV's frame, the COM's bytes that
                                                               * arrived since the last look (+ their ISR time) */
            if (!bg_loop_time_left())
                break;
#ifdef FC_SIMBENCH
            a = timer_tcnt1();
#endif
            job = bg_job();
            if (job == BG_NONE)
                break;
#ifdef FC_SIMBENCH
            simbench_bg_step((uint16_t)(timer_elapsed_counts(a, timer_tcnt1()) / FC_PWM_COUNTS_PER_US), job);
            simbench_bg_work_end((uint16_t)(timer_elapsed_counts(g_anchor, timer_tcnt1()) / FC_PWM_COUNTS_PER_US));
#endif
        }
#if ONB_HAS_GPS
        for (;;) {                                            /* v0.8: the position work on into the exchange window */
#ifdef FC_SIMBENCH
            uint16_t a;
            SB_PEERS();
            a = timer_tcnt1();
#endif
            if (!pos_job_late())
                break;
#ifdef FC_SIMBENCH
            simbench_bg_step((uint16_t)(timer_elapsed_counts(a, timer_tcnt1()) / FC_PWM_COUNTS_PER_US), BG_POS);
            simbench_pos_end((uint16_t)(timer_elapsed_counts(g_anchor, timer_tcnt1()) / FC_PWM_COUNTS_PER_US));
#endif
        }
#endif
#if FC_VTOL
        /* v0.8.1: the jobs the window before BG_END_US left, on into the exchange window (time_left_us: LATE_END_US, the
         * stretched reserves); then the stack scan / status record below in the same pass */
        g_late = true;
        for (;;) {
            uint8_t job;
#ifdef FC_SIMBENCH
            uint16_t a;
#endif
            SB_PEERS();
            if (!bg_loop_time_left())
                break;
#ifdef FC_SIMBENCH
            a = timer_tcnt1();
#endif
            job = bg_job();
            if (job == BG_NONE)
                break;
#ifdef FC_SIMBENCH
            simbench_bg_step((uint16_t)(timer_elapsed_counts(a, timer_tcnt1()) / FC_PWM_COUNTS_PER_US),
                             (uint8_t)(job | SB_BG_LATE));             /* fw09 integration: a late-pass job */
            simbench_late_end((uint16_t)(timer_elapsed_counts(g_anchor, timer_tcnt1()) / FC_PWM_COUNTS_PER_US));
#endif
        }
#endif
        /* both on BG_RESERVE_US in every build (VTOL: the status record, ~0.2 ms with its own stack-scan slice, is longer
         * than the record jobs' REC_RESERVE_US allows) */
        if (bg_time_left())
            (void)stack_free_min_step();                      /* 64-byte slice of the stack high-water scan */
        if (status_due && bg_time_left() && !flight_log_busy()) {
#if defined(FC_SIMBENCH) && FC_VTOL
            const uint16_t a_st = timer_tcnt1();
#endif
            status_record(tick, reset_cause, v_bat);
            status_due = false;
#ifdef FC_SIMBENCH
            sb.status_written++;                                /* the 1 Hz record still gets its slot (report) */
#if FC_VTOL
            /* fw09: the status record (+ its stack-scan slice) is a job started on BG_RESERVE_US (simbench.c "bj") */
            simbench_bg_step((uint16_t)(timer_elapsed_counts(a_st, timer_tcnt1()) / FC_PWM_COUNTS_PER_US),
                             g_late ? (uint8_t)(BG_STATUS | SB_BG_LATE) : (uint8_t)BG_STATUS);
            if (g_late)
                simbench_late_end((uint16_t)(timer_elapsed_counts(g_anchor, timer_tcnt1()) / FC_PWM_COUNTS_PER_US));
            else
#endif
                simbench_bg_work_end((uint16_t)(timer_elapsed_counts(g_anchor, timer_tcnt1()) / FC_PWM_COUNTS_PER_US));
#endif
        }
#if FC_VTOL
        g_late = false;
#endif
        SB_MARK(7);
#ifdef FC_SIMBENCH
        sb.bg_end_us = (uint16_t)(timer_elapsed_counts(g_anchor, timer_tcnt1()) / FC_PWM_COUNTS_PER_US);
        sb.total_us = (uint16_t)(timer_elapsed_counts(t0, timer_tcnt1()) / FC_PWM_COUNTS_PER_US);
        sb.tick = tick;
        sb.state = g_so.state;
        sb.ph_baro = 0;                                       /* v0.6.3: no MS5611 on the FCU */
        sb.loop_us = loop_us_last;
        sb.overruns = overruns;
        sb.n_wait = n_at_tick;
        sb.n_crit = n_crit;
        sb.bg_bytes = (uint8_t)(ILINK_BG_BYTES - g_il_bg_left);
        sb.crit_over = g_il_crit_over;
        sb.bg_capped = g_il_bg_capped;
        sb.flood_ticks = g_il_flood_ticks;
        sb.snap_dropped = g_snap_dropped;
        sb.flog_dropped = log_flight_dropped;
        sb.baro_late = sb_baro_late;
        sb.il_cmds = g_il_cmds;
        sb.il_snaps = g_snaps;
        sb.il_bad = (uint16_t)(g_ild.stats.bad_crc + g_ild.stats.bad_len + g_ild.stats.bad_type);
        {
            const ilink_uart_stats_t *us = ilink_uart_stats();
            const nlink_spi_stats_t *ns = nlink_spi_stats();
            sb.rx_overflow = us->rx_overflow;
            sb.rx_errors = us->rx_errors;
            sb.nl_resyncs = ns->resyncs;
            sb.nl_overruns = ns->overruns;
        }
        sb.pwm_us = pwm_us;
        sb.wait = wait;
        sb.nav_ok = nav_ok;
        sb.reason = g_so.reason;
        sb.nl_frames = g_nl.frames;
        sb.nl_bad = g_nl.bad;
        sb.nl_timeouts = g_nl.timeouts;
        sb.nl_seq_lost = g_nl.seq_lost;
        sb.evq_dropped = g_evq_dropped;
#if FC_VTOL
        sb.phase = g_so.phase;
        sb.vc_sat = g_stk.vo.sat;
        sb.h_tgt_dm = g_so.h_tgt_dm;
        sb.vrec_dropped = g_vrec_dropped;
        sb.esc_u = (powered && !bench) ? u : 0.0f;              /* the synthetic NAV's vertical plant flies this */
        sb.vc_u = g_stk.vo.u;
        sb.thr_q = nl_fcu_throttle_q(sb.esc_u);                 /* what nav_reply sent (the report checks the bytes) */
        sb.td_by = g_stk.vo.touchdown ? g_stk.vo.td_by : VC_TD_NONE;
#endif
#if ONB_HAS_GPS
        {   /* v0.8.1: the GNS chain - the slot receiver's view (mode / requests / engaged of the newest set and TILT) */
            const nl_gns_rx_t *grx = onb_stack_gns(&g_stk);
            const float *tl = onb_stack_att_tilt(&g_stk);
            const float tx = fabsf(tl[0]), ty = fabsf(tl[1]);
            sb.pos_mode = grx->has_latest ? (uint8_t)(grx->latest.pc & NL_PC_MODE_MASK) : 0u;
            sb.pos_requests = nl_gns_rx_requests(grx, g_stk.land_steps);
            sb.pos_engaged = grx->has_tilt && grx->tilt_on;
            sb.gps_gate_ok = nl_gns_rx_gate_ok(grx);
            sb.pos_late = g_pos_late;
            sb.grec_dropped = g_grec_dropped;
            sb.slot_sets = grx->sets;
            sb.slot_broken = grx->broken;
            sb.tilt_mrad = (uint16_t)((tx > ty ? tx : ty) * 1000.0f + 0.5f);
            sb.slot_tilts = grx->tilts;
            sb.slot_fallback = nl_gns_rx_fallback(grx);
        }
#endif
        simbench_end_of_loop(&sb);
        sb.n_cmds = 0;
#endif
        /* then: wait for the next frame (top of the loop), decoding ilink bytes as they arrive */
    }
}
