/* gns_node.h - the GNS board's application core (v0.8.1, the FOURTH ATmega128A: params flight_computer.gns), a port of
 * sim/glink_proto.py GnsNode on core/gps_ubx.h + core/gps_nav.h (the v0.8 NAV GPS side) and core/position_ctrl.h.
 *
 * The GNS runs the GPS receiver (NEO-7M; NEO-6M in v0.8 - on its USART1, 38400, the boot configurator), UBX, the GPS-aided horizontal filter
 * and the position controller at 50 Hz; it makes no safety decision (its LAND requests are data for the FCU's supervisor,
 * V11-V14; the FCU detects a silent GNS itself, core/nlink_proto.h nl_gns_rx_requests).  Calls (the GNS main loop):
 *   every 4 ms timer tick: gns_poll(g, g->nt + 1, the bytes USART1 received, &act) - the UBX parser, the configurator's
 *     UART actions (baud first, then gps_cfg_frame(act.tx, ...)), the epoch assembler and the iTOW clock; the epochs are
 *     stamped with the GNS's NAV-tick clock nt (nt := STATE tick + 1 at each valid STATE frame, + 1 per 4 ms timer tick
 *     between frames; the reference polls on the NAV tick itself).
 *   every byte USART0 (glink) received: gns_rx_byte(g, b, out) - true when a valid STATE frame completed: out holds the
 *     RESULT frame (GL_RESULT_FRAME_BYTES) - start its TX (ISR), then call gns_finish(g) (the epoch fusion, off the latency
 *     path; gns_poll also runs it first if it was left).
 * One 50 Hz cycle (gns_on_state, the normative order of sim/glink_proto.py): 1. the NAV-tick clock (a slip > 2 ticks:
 * the iTOW offset relearns); 2. the gap rules - dk = STATE tick - previous STATE tick: dk = 5 normal; dk = 5 (1 + j): j
 * hold steps of the position controller (pos_hold_steps), then the filter STEP over dk ticks with this frame's mean force;
 * dk <= 0 (a NAV reset) or dk > GL_GAP_RESTART_TICKS (10 s): gps_filter_restart_timebase and the iTOW clock relearns; a
 * STATE without GL_NF_IMU_OK is a lost frame (one hold step, the previous RESULT sent again); 3. gps_filter_gns_step;
 * 4. the position step on the latched state (pos_step, the whole step at once); 5. the RESULT (gns_result_of: TILT /
 * TRIM = nl_tilt_q of tilt_l / trim_l when engaged, else NL_TILT_NONE, 0 / 0, 0; STATUS = the filter snapshot's flags,
 * numSV, hAcc and nl_pc_byte(mode, hold, requests & 15); POS = p - hold point when one is captured, else p; VEL = v;
 * cm / cm/s); 6. after the TX: gps_filter_gns_fuse(tick + 1).
 *
 * Budget: one cycle per 20 ms (params flight_computer.gns.loop_hz 50); the v0.8 C cycle counts of the parts (STEP 1.23
 * ms, position 0.82 ms, fusion <= 0.86 ms per job) leave most of it - measured on the ATmega128A in the v0.8.1 core report.
 *
 * Physical limits stated: GPS needs open sky (no position hold indoors); the hold is to within the GPS wander (~1 m RMS /
 * 2.5 m max until a real module is measured); no ground-effect model.
 * Invariant: the cycle is a pure function of the STATE frames and the UBX bytes it was given (the reference's numbers in
 * float32); nothing here sees the truth or decides a phase.
 */
#ifndef FC_GNS_NODE_H
#define FC_GNS_NODE_H

#include "glink_proto.h"
#include "gps_nav.h"
#include "position_ctrl.h"

typedef struct {                       /* sim GnsNode.stats */
    uint16_t states, hold_steps, gaps, restarts, imu_bad, clock_slips, misaligned;
} gns_stats_t;

typedef struct {                       /* sim.glink_proto.GnsNode */
    gps_nav_t    gps;                  /* parser, assembler, iTOW clock, configurator, filter */
    pos_t        pos;
    gl_decoder_t dec;                  /* USART0 bytes (accepts STATE) */
    uint8_t      n;                    /* step ticks (5) */
    bool         has_nt;
    uint32_t     nt;                   /* the GNS's NAV-tick clock */
    bool         has_kstate;
    uint32_t     k_state;              /* tick of the last STATE frame used */
    bool         has_last;
    uint8_t      last[GL_RESULT_LEN];  /* the last RESULT payload (sent again on an IMU-bad STATE) */
    bool         fuse_due;             /* gns_finish: the fusion of the cycle just answered */
    uint32_t     fuse_k;
    uint16_t     cycles;
    gns_stats_t  st;
} gns_t;

/* gp: the GPS filter's parameters (copied); pp: the position controller's (fc_position_params(): flash on the AVR);
 * configure: run the boot configurator (false: a pre-configured receiver, cfg_ok set) with its timeouts / baud */
void gns_init(gns_t *g, const gps_nav_params_t *gp, const pos_params_t *pp, bool configure, uint16_t n_wait, uint16_t n_ack,
              uint16_t n_settle, uint16_t n_ver, uint32_t baud);
/* every 4 ms: nt = the NAV tick this poll stands for; the bytes the GPS UART received; *act = the configurator's actions */
void gns_poll(gns_t *g, uint32_t nt, const uint8_t *rx, uint16_t n_rx, gps_cfg_action_t *act);
/* one glink byte; true when a STATE frame completed the cycle up to its RESULT (in out); then gns_finish after the TX */
bool gns_rx_byte(gns_t *g, uint8_t b, uint8_t out[GL_RESULT_FRAME_BYTES]);
/* the cycle of one STATE (steps 1-5); the RESULT frame in out (SEQ = seq); gns_finish does step 6 */
void gns_on_state(gns_t *g, uint8_t seq, const gl_state_t *st, uint8_t out[GL_RESULT_FRAME_BYTES]);
/* step 6: the fusion of the newest pending epoch (true when one was fused / rejected) */
bool gns_finish(gns_t *g);
/* the RESULT of the current filter / position state (sim result_from) */
void gns_result_of(const gns_t *g, gl_result_t *r);
/* the position controller's inputs of a STATE frame (sim pos_inputs_from; data NULL) and its GPS data (gps_data_from) */
void gns_pos_inputs(const gl_state_t *st, pos_inputs_t *pin);
void gns_pos_data(const gps_filter_t *f, pos_data_t *d);

#endif /* FC_GNS_NODE_H */
