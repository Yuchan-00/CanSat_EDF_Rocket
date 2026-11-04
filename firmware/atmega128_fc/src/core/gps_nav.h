/* gps_nav.h - the NAV's GPS-aided horizontal estimator (v0.8): a tick-for-tick float32 port of sim/gps_nav.py Part 2
 * (NavGpsFilter, LocalFrame) and NavGps (the whole GPS side per NAV tick), portable C99, no AVR headers.
 *
 * Frames (z up): body b (x to vane 1); the NAV estimator's level frame l (yaw 0 = the pad heading); world W = x NORTH,
 * y WEST.  GPS -> W: x = (lat - lat0) k_N, y = -(lon - lon0) k_E with the WGS-84 meridian / prime-vertical radii at the
 * origin (lat0, lon0 = the first usable fix after boot; gps_frame_*), v_x = velN, v_y = -velE.  Heading psi = the yaw of
 * l in W = -pad_azimuth (params mission.vtol.position_hold.heading.pad_azimuth_deg) + the yaw correction; c_psi, s_psi
 * recomputed (cosf / sinf) only when the correction changes.
 *
 * Per axis i of W: p_i, v_i (CG position / velocity) and b_i (the error of the measured horizontal acceleration).  Every
 * NAV tick (the correction part, AFTER the frame push) the body specific force is summed; with j = k mod 5 (k = NAV ticks
 * since boot, 5 = loop_hz / fusion_hz):
 *   j = 4  STEP (50 Hz): f = mean force of the cycle; R = R(q) of the CORRECTED attitude (rows 0, 1 only); a_W = Rz(psi)
 *          (R f)_xy; p += v T + (a_W - b) T^2 / 2, v += (a_W - b) T, u += (a_W - b) T (T = n / loop_hz); lever arm
 *          r = R l_b, v_lever = R (w x l_b) (w = the bias-corrected body rate), both rotated by psi; the delayed-state
 *          ring entry m = k / 5 (GPS_BUF_N = 16, indexed m mod 16, no search) stores the ANTENNA position p + r_W,
 *          velocity v + v_lever,W, the IMU-only velocity u and v_lever,W; then the nlink snapshot is latched
 *          (gps_nav_snapshot; nl_slot_tx_latch carries it in the frames of the next five ticks).  pad: v = b = u = 0.
 *   j = 0..3  EPOCH FUSION of the newest epoch completed since the previous fusion (at j = 0, as in the reference),
 *          split into jobs (params flight_computer.nav.gps.epoch_split_ticks 4; nothing else touches the state between
 *          them, so the numbers are the reference's doing it at once at j = 0):
 *          j = 0  STEP-B of the previous step (below), the epoch promoted;
 *          j = 1  E0: quality / ON-gate bookkeeping (k = the j = 0 tick), the origin and first-fix initialisation, the
 *                 delayed-state index (age_ms = 20 m_now + 16 - t_e, a = floor((age_ms + 10) / 20) steps), the
 *                 innovations y_p, y_v; E1: the gates (|y_p|^2 > gate_p^2; y_v,i^2 > gate_v2), the rejection runs, the
 *                 position reset (p, the newest n_corr entries and the cumulative offset += y_p) and the velocity reset
 *                 (v += y_v, p += y_v age, b = 0; not in the offset); E3: the used-epoch bookkeeping (FIX_OK freshness;
 *                 an axis is updated iff its position or velocity was accepted, so this needs only E1's decisions) and
 *                 the information-weighted yaw update (1-state Kalman: at every closed window P += q (window time),
 *                 used or not; if used R = (sigma_dv / |du|)^2 + sigma_rel^2, K = P / (P + R), correction += K
 *                 atan2(du x dg, du . dg), P *= 1 - K; it reads only the epoch, the delayed-state entry's u / v_lever,
 *                 which E2 never changes, and its own state - v0.8 physics-review fix: P grew only on used windows,
 *                 q was 0.04 deg^2/s and there was no angle-noise floor, so HEAD_OK was overconfident); E1 also shifts
 *                 "velocity rejected" into the 5-epoch history behind GPS_F_INCONS (physics review HIGH 2);
 *          j = 2  E2 x: the fixed-gain update (K_pv / K_p / K_v of the steady or the acquisition set, chosen at E0 before
 *                 the resets) propagated to now and to the newest n_corr entries s >= s_e; cos / sin of the new psi;
 *          j = 3  E2 y.
 *          The 50 Hz step is split in two (measured on the ATmega128A, the whole step took 1.45 ms): STEP-A at j = 4 (the
 *          integration, the STATUS part of the snapshot) and STEP-B at j = 0 of the next cycle before the fusion (the
 *          lever arm, the delayed-state entry m_now, the other snapshot parts) - between them nothing changes the state.
 * Gains (params_gen.h GPS_K_*): the steady-state / acquisition Kalman gains of the model q_acc, q_b, sigma_p, sigma_v at
 * the epoch rate, computed once in float64 by sim/gps_nav.py compute_gains (no covariance propagation on the AVR).
 * Heading error: params heading.tolerance_deg 15 deg (physics review: without the yaw correction the loop held 20 deg,
 * failed at 30-45 deg - the limit is the NAV velocity gate / reset spiral, not the loop's phase margin); see
 * sim/gps_nav.py "Heading error".
 *
 * float32 vs the float64 reference (tools/crosscheck_controller.py GPS estimator section): the state is a sum of
 * contractive corrections and IMU integrations; position errors grow at most like the velocity rounding x time between
 * the 5 Hz corrections (gains 0.01 / 0.36) - bounds and justification in the cross-check.  Integer decisions (ages,
 * gates, counters, the ON gate, FIX_OK) are exact except where a gate's float64 margin is below the float32 rounding
 * (counted as knife-edges there).  The snapshot integers may differ by one count at a rounding tie.
 *
 * v0.8.1 (the four-chip build): the same filter runs on the GNS board (core/gns_node.c), one cycle per glink STATE
 * frame instead of five NAV ticks: gps_filter_gns_step takes the cycle's force SUM and sample count, the attitude and
 * rate the NAV sampled, and the NAV tick k of the frame (dk = the ticks since the previous STATE: T = dk / loop_hz when
 * frames were missed, else t_step; kq = k / 5 as above) and runs STEP-A then STEP-B at once (the GNS has the whole
 * 20 ms cycle); gps_filter_gns_fuse(k + 1) runs the promote + the four fusion jobs after the RESULT has been sent.
 * gps_filter_restart_timebase re-learns the NAV clock after a NAV reset or a STATE gap > 10 s (counted in
 * st_timebase_restarts); gps_nav_poll is the receiver side of gps_nav_tick without the filter (the GNS's 4 ms poll).
 * The four-chip NAV does not run it (its USART1 carries glink; with no caller --gc-sections drops the code).
 *
 * Invariant: nothing here sees the truth; inputs are UBX epochs (gps_ubx.h), the IMU samples and the NAV's corrected
 * attitude; outputs are the snapshot (nlink v3 slots; v0.8.1: the GNS's RESULT) and diagnostics.
 */
#ifndef FC_GPS_NAV_H
#define FC_GPS_NAV_H

#include "fcmath.h"
#include "gps_ubx.h"
#include "nlink_proto.h"

#define GPS_BUF_N 16u                  /* delayed-state ring (steps): 0.32 s > the 0.22 s latency + jitter; power of 2 */
#define GPS_NCORR_MAX 4u               /* n_corr <= this (the per-entry coefficients of a fusion) */
/* gps_filter_tick: what the NAV does with the snapshot after the tick (nl_slot_tx_latch(&tx, &f->snap, f->cycle) on
 * either: the STATUS part and the cycle at the step, j = 4; the OFFSET / POS / VEL / HEAD parts at j = 0, before the
 * frame of j = 1 that carries the first of them - the slots carry the reference's snapshot latched at j = 4) */
#define GPS_TICK_NONE 0u
#define GPS_TICK_STEP 1u
#define GPS_TICK_SNAP 2u

/* snapshot flags (nlink v3 slot STATUS byte 0, TELEM v3 gps_flags, SNAP3; sim/gps_nav.py GF_*) */
#define GPS_F_FIX_OK 0x01u             /* the estimator's GPS is current */
#define GPS_F_GATE_OK 0x02u            /* ON gate passed (good fix for stable_s) */
#define GPS_F_HEAD_OK 0x04u            /* heading confidence */
#define GPS_F_FIX3D 0x08u              /* the latest epoch: gpsFixOK and 3D */
#define GPS_F_CFG_OK 0x10u             /* boot configuration acknowledged */
#define GPS_F_INIT 0x20u               /* origin set, estimator initialised */
#define GPS_F_VER_OK 0x40u             /* MON-VER answered */
#define GPS_F_INCONS 0x80u             /* GPS-IMU inconsistent: the GPS velocity rejected in >= GPS_INCONS_MIN_REJ of the
                                        * last GPS_INCONS_EPOCHS flight epochs (physics-review fix; was GPS_F_PAD) */
#define GPS_INCONS_EPOCHS 5u           /* sim/gps_nav.py INCONS_EPOCHS (the history is a uint8: <= 8) */
#define GPS_INCONS_MIN_REJ 3u          /* sim/gps_nav.py INCONS_MIN_REJ */
#define GPS_YAW_SIGMA_REL2_ASSUMED 0.0625f /* sim/gps_nav.py YAW_SIGMA_REL^2 (0.25 rad): the yaw window's angle-noise floor */

typedef struct {
    float kpv[3][2];                   /* columns p, v (rows p, v, b) */
    float kp[3], kv[3];                /* one accepted measurement */
} gps_gain_set_t;

typedef struct {                       /* sim.gps_nav.NavGpsParams as the NAV uses it (params_gen.h GPS_*) */
    uint8_t  step_ticks;               /* loop_hz / fusion_hz = 5 */
    uint8_t  ms_per_tick;              /* 1000 / loop_hz = 4 */
    uint16_t latency_ms;               /* round(latency_s x 1000) */
    float    loop_hz_f;                /* T = n_acc / this (n_acc != step_ticks only after skipped ticks) */
    float    inv_step;                 /* 1 / step_ticks (the mean force of a normal cycle) */
    float    t_step;                   /* step_ticks / loop_hz */
    float    lever_b[3];               /* antenna from the CG, body [m] (0 when lever_comp is off) */
    float    psi0, c_psi0, s_psi0;     /* -pad_azimuth [rad] and its cos / sin */
    uint8_t  gate_min_sv;              /* ON gate (gps_gate_on_ASSUMED) */
    uint32_t gate_max_hacc_mm, gate_max_sacc_cms;   /* the largest integers with x / 1000 <= max_hacc_m (x / 100 <= max_sacc) */
    uint32_t gate_stable_ticks;
    uint8_t  ok_min_sv;                /* in flight: usable */
    uint32_t ok_max_hacc_mm;
    gps_gain_set_t steady, acq;
    float    gate_v2;                  /* gate_sigma^2 S_vv (velocity gate per axis) */
    float    gate_p2;                  /* gate_p^2 (position gate, 2-D) */
    uint8_t  n_corr;                   /* buffer entries corrected after a fusion */
    uint16_t acq_epochs;
    uint16_t reset_after_epochs, vel_reset_epochs;
    uint32_t fresh_ticks;              /* FIX_OK: an epoch used within this */
    bool     yaw_correction;
    uint16_t yaw_window_steps;         /* the smallest d with d t_step >= yaw_window_s - 1e-9 */
    float    yaw_min_dv2;              /* yaw_min_dv^2 */
    float    yaw_sigma_dv2;            /* yaw_sigma_dv^2 */
    float    yaw_q_rad2;               /* radians(1)^2 x yaw_q_deg2_per_s (x window steps x t_step at use) */
    float    p_psi0;                   /* radians(yaw_sigma0)^2 */
    float    head_ok_rad2;             /* radians(head_ok)^2 */
    float    yaw_sigma_rel2;           /* the window's angle-noise floor^2 [rad^2] (GPS_YAW_SIGMA_REL2_ASSUMED) */
} gps_nav_params_t;

typedef struct {
    int32_t m;                         /* the step index of this entry, -1 = cleared */
    float   pa[2], va[2], u[2], lv[2]; /* antenna position / velocity, IMU-only velocity, lever velocity (W) */
} gps_entry_t;

typedef struct {
    int32_t lat0, lon0;
    float   k_n, k_e;                  /* metres per 1e-7 deg of latitude / longitude */
} gps_frame_t;

typedef struct {                       /* the epoch fusion in progress between E0 and E3 */
    uint8_t  stage;                    /* 0 = none, 1 E0 + E1 + yaw, 2 E2 x (+ cos / sin), 3 E2 y is the next job */
    uint32_t k;                        /* NAV tick of E0 (k_last_used of a used epoch) */
    int32_t  s_e;                      /* the delayed-state step of the epoch */
    float    age;                      /* a x t_step */
    float    hage;                     /* 0.5 age age */
    bool     yaw_cs;                   /* the yaw update changed the correction: cos / sin of psi still to compute */
    float    zvx, zvy, yp[2], yv[2];
    bool     acc_p, acc_v[2], used;
    bool     acq;                      /* the acquisition gains (chosen at E0, before the resets) */
} gps_fuse_t;

typedef struct {                       /* sim.gps_nav.NavGpsFilter */
    gps_nav_params_t p;
    uint32_t k;                        /* NAV tick of the last call */
    float    acc[3];
    uint8_t  n_acc;
    gps_entry_t buf[GPS_BUF_N];
    int32_t  m_now;                    /* the last step index, -1 before the first step */
    bool     has_pending;              /* an epoch completed since the last fusion (the newest one) */
    uint16_t n_pending;
    gps_epoch_t pending;
    uint32_t pending_te;
    float    p_[2], v[2], b[2], u[2];
    float    yaw_corr, c_psi, s_psi, p_psi;
    bool     has_yaw_win;
    int32_t  yaw_m0;
    float    yaw_g0[2], yaw_u0[2];
    uint16_t yaw_used;
    bool     has_frame;
    gps_frame_t frame;
    bool     init, pad;
    uint16_t acq_left;
    uint16_t rej_p_run, rej_v_run;
    uint8_t  rej_v_hist;               /* the last GPS_INCONS_EPOCHS flight epochs, bit 0 = newest: velocity rejected */
    uint8_t  reset_count;              /* the wire counter (wraps) */
    float    off[2];                   /* cumulative reset offset (W, m) */
    float    lever_now[2];             /* r_W of the last step (the first fix: p = z - r_W) */
    bool     has_last_used, has_good_since, has_last_epoch;
    uint32_t k_last_used, good_since, k_last_epoch;
    bool     last_fix3d;
    uint8_t  last_sv;
    uint32_t last_hacc_mm;
    bool     cfg_ok, ver_ok;
    nl_gps_snap_t snap;                /* the latest latched snapshot */
    uint16_t cycle;                    /* step counter (the slot cycle counter) */
    gps_fuse_t fu;
    gps_epoch_t fe;                    /* the epoch promoted at the fusion tick (j = 0) for E0 */
    uint32_t fe_te;
    float    rw[2][3];                 /* rows 0, 1 of R(q) of the last step (STEP-A -> STEP-B) */
    float    tab_t[GPS_BUF_N], tab_h[GPS_BUF_N];   /* d t_step and 0.5 (d t_step)^2 for an age of d steps (init) */
    uint8_t  jc;                       /* k mod step_ticks of the last call */
    uint32_t kq;                       /* k / step_ticks of the last call */
    bool     started;
    float    sw[3];                    /* the body rate of the last step */
    bool     sb_due;                   /* STEP-B of the last step still to run */
    float    last_innov[4];            /* diagnostics: y_p x, y, y_v x, y of the last fused epoch */
    uint16_t st_fused, st_rej_p, st_rej_v, st_pos_resets, st_vel_resets, st_dropped_old, st_dropped_stale;
    uint16_t st_n_epoch_dropped, st_unusable;
    uint16_t st_timebase_restarts;     /* v0.8.1 GNS: gps_filter_restart_timebase calls */
} gps_filter_t;

void gps_filter_init(gps_filter_t *f, const gps_nav_params_t *p);
/* sim.gps_nav.LocalFrame.at / to_w */
void gps_frame_at(gps_frame_t *fr, int32_t lat0_e7, int32_t lon0_e7);
void gps_frame_to_w(const gps_frame_t *fr, int32_t lat_e7, int32_t lon_e7, float *x, float *y);
/* the epochs completed in this tick, with their t_e (gps_itow_update), BEFORE gps_filter_tick of the same tick */
void gps_filter_epoch(gps_filter_t *f, const gps_epoch_t *e, uint32_t t_e_ms);
/* NAV tick k (the correction part, after the frame push): the tick's body specific force, the CORRECTED attitude, the
 * bias-corrected body rate, the pad flag (nl pad mode of the last valid FCU frame); returns GPS_TICK_STEP (the 50 Hz step
 * ran: the snapshot's STATUS part and cycle are new) or GPS_TICK_SNAP (its other parts are new) - on either the NAV
 * re-latches nl_slot_tx with f->snap and f->cycle - or GPS_TICK_NONE */
uint8_t gps_filter_tick(gps_filter_t *f, uint32_t k, const float accel_b[3], const float q_corr[4], const float rate_b[3],
                        bool on_pad);
bool gps_filter_fix_ok(const gps_filter_t *f, uint32_t k);
bool gps_filter_gate_ok(const gps_filter_t *f, uint32_t k);
bool gps_filter_head_ok(const gps_filter_t *f);
bool gps_filter_incons(const gps_filter_t *f);   /* GPS_F_INCONS: >= GPS_INCONS_MIN_REJ of the history bits set */
/* ---- v0.8.1: the same filter on the GNS board (core/gns_node.h), driven by the NAV's glink STATE frame ----
 * gps_filter_gns_step: the 50 Hz STEP of NAV tick k (k mod 5 == 4) at once (STEP-A + STEP-B): fsum = the body specific
 * force summed over nf NAV ticks (the frame's sum; mean = fsum / nf), dk = NAV ticks since the previous step (T = dk /
 * loop_hz: a lost frame's gap is propagated with this frame's mean force), the corrected attitude and bias-corrected rate
 * of tick k, the NAV's pad flag (sim/gps_nav.py NavGpsFilter.gns_step).  gps_filter_gns_fuse: the j = 0 fusion of the
 * newest pending epoch at once (E0 .. E3), k = the NAV tick it stands for (the STATE tick + 1); false when none was
 * pending.  gps_filter_restart_timebase: a NAV reset or a STATE gap > 10 s - buffer, pending epochs and the estimator
 * state discarded (re-initialised at the next usable epoch), the local frame and the cumulative offset kept. */
void gps_filter_gns_step(gps_filter_t *f, uint32_t k, const float fsum[3], uint8_t nf, uint32_t dk, const float q_corr[4],
                         const float rate_b[3], bool on_pad);
bool gps_filter_gns_fuse(gps_filter_t *f, uint32_t k);
void gps_filter_restart_timebase(gps_filter_t *f);

/* ---- the NAV's whole GPS side per tick (sim.gps_nav.NavGps): UART bytes -> parser -> configurator / epoch assembler +
 * iTOW clock -> filter ---- */
typedef struct {
    ubx_parser_t  parser;
    gps_asm_t     asm_;
    gps_itow_t    clock;
    gps_cfg_t     cfg;
    bool          configure;           /* false: no configurator (cfg_ok = true; tests, a pre-configured receiver) */
    gps_filter_t  filter;
    uint16_t      epochs_this_tick;    /* diagnostics */
} gps_nav_t;

void gps_nav_init(gps_nav_t *g, const gps_nav_params_t *p, bool configure, uint16_t n_wait, uint16_t n_ack,
                  uint16_t n_settle, uint16_t n_ver, uint32_t baud);
/* one NAV tick: the bytes received since the previous tick, the IMU / estimator values of gps_filter_tick; *act = the
 * configurator's UART actions (baud first, then the frame to send); returns gps_filter_tick's value */
uint8_t gps_nav_tick(gps_nav_t *g, uint32_t k, const uint8_t *rx, uint16_t n_rx, const float accel_b[3],
                     const float q_corr[4], const float rate_b[3], bool on_pad, gps_cfg_action_t *act);
/* the receiver half of gps_nav_tick (parser, configurator, assembler + iTOW clock -> pending epochs), no filter tick - the
 * GNS's 4 ms poll (sim/glink_proto.py GnsNode.poll); gps_nav_tick = gps_nav_poll + gps_filter_tick */
void    gps_nav_poll(gps_nav_t *g, uint32_t k, const uint8_t *rx, uint16_t n_rx, gps_cfg_action_t *act);

#endif /* FC_GPS_NAV_H */
