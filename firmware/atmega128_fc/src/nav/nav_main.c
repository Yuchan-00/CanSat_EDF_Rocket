/* nav_main.c - ATmega128A navigation computer (NAV) of the CanSat EDF rocket: the 250 Hz loop (v0.6.3).
 *
 * v0.6.3 (params.json flight_computer.architecture, SPEC 10.5) splits the v0.6 FCU in two: this chip reads the
 * MPU-6050 and the barometer (hal/baro.h: BMP280 by default, MS5611 = build option) on I2C and computes the navigation state - the steps 2-4 of the v0.6.2 FCU loop, the
 * same core functions in the same order - and pushes it to the FCU every tick as the nlink SPI master
 * (core/nlink_proto.h, hal/nlink_spi.h); the FCU (src/main.c) makes every flight decision.  The NAV makes none:
 * its only input from the FCU is the supervisor state, which selects pad or flight mode of the estimator.
 *
 * Every tick (Timer2, NAV_TICK_HZ = FC_LOOP_HZ), time-critical part:
 *   1. (nothing to latch: the NAV drives no output but the nlink and its LED)
 *   2. sensors: the MPU-6050 burst read that the Timer0 lead ISR started FC_IMU_LEAD_COUNTS before the tick
 *      (v0.5 timing) is converted; a failed read counts imu_fail and every ONB_IMU_FAIL_TICKS + 1 consecutive
 *      failures clock the bus free (twi_recover); then the barometer step of this tick's phase (its bus script runs
 *      during step 3)
 *   3. attitude estimate (sim/estimator.py PadAlignGyroEstimator): pad alignment while on_pad, gyro-only after;
 *      bias-corrected rates, tilt, the pad-still counter
 *   4. altitude filter: held at 0 in pad mode, predict + baro correct otherwise; the ground reference (LPF in pad
 *      mode, frozen after); baro_ok (a sample every FC_BARO_DIV ticks + one of slack); aligned (ARM gate, frozen
 *      off the pad); EST_VALID latched by the first aligned
 *   5. the navigation frame (nl_encode_nav) and the exchange (nlink_spi_exchange: SS low, 72 bytes, SS high) -
 *      the FCU's tick starts when its last byte is in; the FCU's state bytes come back in the same exchange
 *   end of the time-critical part (the loop time measured here is tick -> frame delivered)
 *   6. the FCU's reply: valid (nl_decode_fcu: sync, TYPE, CRC) -> its state and fflags select the mode of the NEXT
 *      tick; invalid -> the last valid ones stay.  LED.
 * Background until NAV_BG_RESERVE_US before the next tick: the barometer math (the driver's compensation, then the powf
 * altitude: one step per pass, published for the next tick), the stack scan slice.  Then the CPU sleeps.
 *
 * Pad / flight (nlink_proto.h "Semantics on the NAV", sim/nlink_proto.py nav_pad_mode): the v0.6.2 FCU used its
 * own previous state; the NAV uses the state of the last valid FCU frame - SAFE before the first one after a NAV
 * boot - which is two control ticks behind the supervisor (nlink_proto.h; at LAUNCH the throttle ramps from 0 over
 * 0.5 s).  Exactly as in v0.6.2 the altitude filter / ground reference follow the state alone (pad_alt = SAFE or
 * ARMED; sim/nlink_proto.py nav_alt_pad_mode) and the estimator's pad alignment is that minus the bench-vanes mode
 * (on_pad = pad_alt && !NL_FF_BENCH; nav_pad_mode): the bench test keeps the altitude held on the pad while the
 * estimator integrates the gyro.
 *
 * Flags (nlink_proto.h): IMU_OK = this tick's read succeeded; BARO_OK = baro_ok; ALIGNED = aligned; GYRO_SAT =
 * the sample in hand is at the gyro rail (the previous sample when this tick's read failed, as v0.6.2 handed the
 * supervisor); EST_VALID = aligned has been true in pad mode since this boot (the attitude, from then on
 * integrated from that alignment, is the vehicle's; a NAV that reset in flight has none - the FCU's I9 rule);
 * BARO_NEW = this tick's altitude filter took a new baro sample; BOOT = no valid FCU frame since this reset.
 * reset_cause and the LED report the boot results (nav.h).  loop_us_max = the time-critical part (tick -> frame
 * delivered, Timer1) of the previous tick: a frame cannot carry its own delivery time, and with one frame per tick
 * the previous tick is the only one since the previous frame (a missed tick is not run at all).
 *
 * Measured (tools/run_nav_simbench.py, simavr, host_test/build/nav_simbench_report.json): the estimator section is the
 * v0.6.2 FCU's (1.2 ms in flight mode), the frame build + nl_encode_nav ~0.19 ms, the exchange ~0.63 ms on the wire;
 * the frame is delivered ~1.8 ms (pad) / ~2.2 ms (flight) after the tick.
 *
 * v0.7.1, two builds (src/fc_mode.h: params.json mission.mode, or -D FC_MODE_BALLISTIC in the nav_ballistic envs):
 *   BALLISTIC: everything above, the v0.6.3 estimator (PadAlignGyroEstimator) and altitude filter, bit for bit; the FCU
 *     frame's thr_q byte is ignored.  (v0.8: unchanged - the ballistic NAV is the v0.7.1 image.)
 *   VTOL: v0.8, below (v0.7.1's VTOL loop - the gated Mahony estimator and the VTOL altitude filter BEFORE the push - is
 *     gone: the v0.8 FCU refuses its NL_TYPE_NAV frames).
 *
 * v0.8 VTOL build (GPS position hold; params.json flight_computer.nav.schedule_note, electronics.gps, flight_computer.nav
 * .gps; sim/estimator.py "v0.8 NAV tick order" - the normative order; the v0.7.1 integration measured IMU sample -> vane
 * 5.2 ms > 4.71 ms and NAV tick -> vane 4.7 ms > 4.0 ms because the whole correction ran before the push).  Per NAV tick k:
 *   2. sensors, as above.
 *   3. PREDICT (core/att_estimator.h att_mahony_predict: rate = g - b of tick k-1; flight q_pub = integrate(q_{k-1}, rate),
 *      pad q_pub = q_{k-1}) and nl_nav3_tilt(q_pub); PUBLISH the nlink v3 frame (core/nlink_proto.h nl_nav3_t: q_pub,
 *      rate, the accelerometer, the altitude filter's h / v_z of tick k-1, cos_tilt / tilt, the flags - IMU_OK / GYRO_SAT
 *      of this read, ALIGNED / EST_VALID / EST_GATE / BARO_NEW / BARO_OK as tick k-1 left them - and GPS slot part k mod 5
 *      of the snapshot latched at the last 50 Hz step, nl_slot_tx_slot); the push waits for the floor
 *      NAV_PUSH_MIN_US_VTOL (nav.h: the pad-mode prediction is cheaper than the flight-mode one), then the exchange.
 *      End of the time-critical part (tick -> frame delivered).
 *   4. CORRECT (att_mahony_update, the v0.7.1 correction from q_{k-1}, b_{k-1}) -> q_k, b_k; the pad-still counter on
 *      g - b_k (as v0.7.1).
 *   5. the VTOL altitude filter (alt_vtol_step) with the world-z specific force from the CORRECTED q_k; baro sample /
 *      ground reference / BARO_OK / ALIGNED / EST_VALID as v0.7.1 - all for the frame of tick k+1.
 *   6. GPS (core/gps_nav.h gps_nav_tick): the bytes the USART1 ring holds (hal/gps_uart.h, filled by the RX interrupt)
 *      -> UBX parser -> boot configurator (its actions: the baud switch, then one CFG / MON-VER frame into the TX ring) /
 *      epoch assembler + iTOW clock -> the fixed-gain filter on a_k, q_k and g - b_k, pad = pad_alt (the vehicle stands on
 *      the pad in the bench-vanes mode too); at the 50 Hz step (k mod 5 == 4) and the snapshot completion (k mod 5 == 0)
 *      the snapshot is latched for the slots (nl_slot_tx_latch).
 *   7. the FCU's reply (its state selects pad / flight of tick k+1, as before), LED; background: the barometer math.
 * Steps 4-6 are not optional: they run every tick, and with step 3 they must end before the next tick (the GPS work is
 * split so no tick carries more than one job of it: core/gps_nav.h).  The reference's single pad flag per tick: pad_alt /
 * on_pad / thr_q are taken from the FCU frame of the previous exchange at the start of the tick and kept for all of it
 * (the reply of this tick's exchange is read in step 7).  The GPS boot configuration and its result travel in the slot
 * STATUS flags (core/gps_nav.h GPS_F_CFG_OK / GPS_F_VER_OK / FIX3D / FIX_OK / GATE_OK; the FCU logs them and sends them
 * in TELEM v3) - reset_cause has no free bit.  loop_us_max keeps its meaning (tick -> frame delivered of the previous
 * tick).  Measured: tools/run_nav_simbench.py --mode vtol (a synthetic u-blox receiver - the NEO-6M of v0.8 - on a byte-accurate Timer3 wire).
 *
 * v0.8.1 VTOL build, the FOUR-chip set (architect 2026-10-21 20:30, params flight_computer.architecture_v081 / glink / gns;
 * the v0.8 integration measured the three-chip GPS build at 98.7 % mean / 119.5 % max NAV mandatory work): the GPS
 * receiver, UBX, the GPS filter and the position controller moved to the GNS board (src/gns/); USART1 now carries the
 * glink to it (hal/glink_uart.h, core/glink_proto.h).  Per NAV tick k, the v0.8 order minus step 6:
 *   2. sensors.
 *   3. PREDICT and PUBLISH: the glink bytes still waiting are decoded first (at most NAV_GL_PREBUILD_BYTES, k_rx = k: a
 *      RESULT that completed after the previous tick's background still makes this frame), then the GNS slot of frame k
 *      (gl_nav_slot = nl_gns_tx_slot: TILT first at k_m + L, L = GLINK_LATENCY_TICKS) in the nlink v4 frame
 *      (nl_encode_nav4: the v3 payload under NL_TYPE_NAV4), push floor, exchange.
 *   3b. every glink byte waiting (gl_nav_rx, k_rx = k): outside the time-critical part, before the correction.
 *   4. CORRECT; 5. altitude (as v0.8).
 *   6. glink: gl_nav_tick - this tick's body specific force into the cycle's sum, the FCU frame's SAT_HI into the OR; at
 *      k mod 5 == 4 the STATE frame (the corrected q, the bias-corrected rate, h after tick k, the pad flag pad_alt, the
 *      FCU4 fields) goes into the USART1 TX ring (60 B, 2.40 ms on the line, UDRE interrupt).  The FCU fields are those of
 *      the reply this tick's pad mode came from (the previous exchange: the FCU's tick k - 2, the 6-DOF's
 *      nav_state_lag_ticks = 2 - the frame sim/sim6dof.py hands GlinkNav.tick).  Then every glink byte waiting again.
 *   7. the FCU's reply (nl_decode_fcu4: the v4 frame with pflags / kt_q) -> the mode of tick k+1; LED.
 *   background: the glink bytes still arriving -> gl_nav_rx (k_rx = k); the barometer math; the stack scan.
 * The RESULT bytes are decoded where the CPU is in the tick - a RESULT completing at any time before the frame build of
 * k_m + L is in the 4-entry ring the build latches from (3b / 6 / background of the earlier ticks, or the capped pre-build
 * decode for one that completed just before the build); the decode is mandatory work (~15 us a byte, 26 bytes a cycle).
 * The NAV makes no decision on the GNS data: it copies the RESULT bytes into the slot (or SLOT_EMPTY when the RESULT of a
 * cycle was not there by its frame build); the FCU detects a silent GNS itself (nl_gns_rx: TRIM fallback, its own V11).
 * One NAV image serves every VTOL FCU build (fcu, fcu_demo, fcu_nogps: the latter ignores the slot) - a vehicle without
 * the GNS board only sends STATE frames into nothing.
 *
 * Safety invariants: no flight decision here - a dead or resetting NAV is an IMU + baro failure for the FCU (no
 * valid frame -> imu_ok = baro_ok = false -> I9); SS is an output held high from the first instructions of main,
 * before the SPI is enabled as master, so the FCU's slave never sees a half-made exchange from the NAV's boot; the
 * exchange always releases SS; every I2C wait is bounded (hal/twi.h) and the watchdog resets a hung loop; the glink UART
 * never waits (hal/glink_uart.h) and nothing the GNS sends can make the NAV skip a frame (bounded work per byte, a capped
 * number of bytes before the push).
 */
#include <avr/interrupt.h>
#include <avr/io.h>

#include "board.h"
#include "fc_mode.h"
#include "gains.h"
#include "params_gen.h"

#include "core/alt_filter.h"
#include "core/att_estimator.h"
#include "core/fc_config.h"
#include "core/nlink_proto.h"
#include "core/onboard.h"
#include "core/quat.h"
#if FC_VTOL
#include "core/glink_proto.h"
#include "hal/glink_uart.h"
#endif

#include "hal/mpu6050.h"
#include "hal/baro.h"
#include "hal/nlink_spi.h"
#include "hal/stack.h"
#include "hal/timer.h"
#include "hal/twi.h"
#include "hal/watchdog.h"
#include "nav/nav.h"
#include "nav/nav_simbench.h"

#ifndef FC_ROLE_NAV
#error "nav_main.c is the NAV firmware: build it with -D FC_ROLE_NAV (platformio.ini [nav_common])"
#endif
#if NAV_TICK_HZ != FC_LOOP_HZ
#error "the NAV runs the estimator at the control rate: flight_computer.nav.tick_hz must equal control.loop_hz"
#endif
#if GAINS_LOOP_HZ_MISMATCH
#warning "gains.h: sim/out/gains.json was tuned at another loop_hz than the firmware runs - re-run the simulation tuning"
#endif
#if GAINS_PARAMS_STALE
#warning "gains.h: sim/out/gains.json was tuned against a different params.json"
#endif
#if FC_VTOL && (GPS_STEP_TICKS != NL_SLOT_PARTS || GPS_STEP_TICKS != GL_STEP_TICKS)
#error "the GNS slot rotation (nlink v4) needs one slot part per NAV tick of the 50 Hz glink cycle (GPS_STEP_TICKS)"
#endif
#if FC_VTOL && (!defined(GLINK_LATENCY_TICKS) || GLINK_LATENCY_TICKS < NL_GNS_L_MIN || GLINK_LATENCY_TICKS > NL_GNS_L_MAX)
#error "GLINK_LATENCY_TICKS (params flight_computer.glink.latency_ticks_ASSUMED) must be 2..16 (the NAV's RESULT ring)"
#endif
/* fw09 W1: Timer3 is the VTOL nlink's (hal/nlink_spi.h): no other Timer3 driver in the VTOL NAV (hal/pwm.c drives Timer1 /
 * Timer3 PWM, hal/gps_uart.c's bench wire Timer3 compare A; the glink bench wire checks itself, hal/glink_uart.c) */
#if FC_VTOL && (defined(FC_PWM_H) || defined(FC_GPS_UART_H))
#error "the VTOL NAV's Timer3 belongs to the nlink (hal/nlink_spi.c, W1): hal/pwm.h / hal/gps_uart.h are not NAV drivers"
#endif
/* the latest W1 close (start + the watchdog; the start is >= the push floor after the tick) must stay inside the tick */
#if FC_VTOL && defined(NLINK_WD_COUNTS) && \
    NLINK_WD_COUNTS >= (FC_TICK_US - NAV_PUSH_MIN_US_VTOL) * (FC_F_CPU_HZ / 1000000UL)
#error "NLINK_WD_COUNTS (params flight_computer.nlink.exchange_watchdog_us) must be < the tick - NAV_PUSH_MIN_US_VTOL"
#endif

#ifdef FC_SIMBENCH
#define SB_MARK(i) nav_sb_mark(i)
#else
#define SB_MARK(i) ((void)0)
#endif

#if FC_VTOL
/* v0.8 VTOL build (params.json mission.vtol.nav_estimator / baro_lost; core/att_estimator.h att_mahony_*,
 * core/alt_filter.h alt_vtol_*): the gated Mahony estimator - its pad branch IS the ballistic pad alignment (g_mah.pad,
 * att_est_t) - and the VTOL altitude filter with the accelerometer z-bias state and the baro thrust compensation.
 * v0.8.1: the glink's NAV side (core/glink_proto.h gl_nav_t: the force sum, the STATE frames, the RESULT decoder and the
 * slot transmitter of the nlink v4 frame) instead of v0.8's GPS side */
static att_mahony_t g_mah;
static alt_vtol_t g_altv;
static gl_nav_t g_gl;
static nl_fcu4_t g_fcu4;                         /* the last valid FCU frame (nlink v4: pflags, kt_q for the STATE) */
static nl_nav3_t g_nav3;
#define g_est (g_mah.pad)                        /* q, b_hat, n, n_avg of the pad branch (the same fields as ballistic) */
/* the glink bytes decoded right before the frame build (step 3, in the time-critical part): a RESULT completed after the
 * previous tick's background still reaches this frame; bounded (~15 us a byte, the CRC per byte) - a frame whose tail is
 * still waiting misses this build and is decoded in the background (the slot then sends SLOT_EMPTY for its cycle only if
 * this was the last build before k_m + L: nl_gns_tx counts it as missed) */
#define NAV_GL_PREBUILD_BYTES 8u
#define NAV_GL_BG_CHUNK 16u                      /* the background decodes the ring in chunks of this many bytes */
#define NAV_GL_BG_RESERVE_US 300u                /* > one chunk's decode (16 x ~15 us) + a RESULT handed to the ring */
#else
/* the ballistic build: the v0.6.3 estimator and altitude filter, bit for bit */
static att_est_t g_est;
static alt_filter_t g_alt;
#endif
static imu_sample_t g_imu;
static baro_t g_baro;                                  /* hal/baro.h: BMP280 (default) or MS5611 */
#if !FC_VTOL
static nl_nav_t g_nav;
#endif
#if !FC_VTOL
static uint8_t g_tx[NL_XFER_BYTES], g_rx[NL_XFER_BYTES];
#endif                                           /* VTOL (P1): the driver's nlink_spi_buf, in place */

/* the FCU as the NAV knows it: its last valid frame */
static struct {
    uint8_t state, fflags, params_tag;
    bool have;                                   /* a valid FCU frame since this reset (else BOOT) */
    uint16_t age;                                /* NAV ticks since it (saturating) */
    uint16_t valid, bad;                         /* replies accepted / refused (simbench report) */
#if FC_VTOL
    uint8_t thr_q;                               /* v0.7.1: its ESC throttle (nlink byte 7; 0 before the first frame) */
#endif
} g_fcu = {ONB_SAFE, 0u, FC_PARAMS_TAG, false, 0xFFFFu, 0u, 0u
#if FC_VTOL
           , 0u
#endif
};

/* ------------------------------------------------------------------------ */
/* phase counter: advances by the ticks elapsed (1 normally), no 32-bit division in the loop (v0.6.2 main.c) */
static uint8_t phase_advance8(uint8_t ph, uint32_t d, uint8_t div)
{
    if (d >= div)
        d %= div;                                /* only after missed ticks */
    ph = (uint8_t)(ph + d);
    return ph >= div ? (uint8_t)(ph - div) : ph;
}

/* background work may start a step only if it ends well before the next tick */
#define NAV_BG_RESERVE_US 450u        /* > the longest background step (the altitude powf: ~0.37 ms in simavr, measured with the MS5611 build) + the
                                       * TWI interrupts of the MPU-6050 burst read the Timer0 lead ISR starts 496 us
                                       * before the tick */
static bool bg_time_left(uint16_t t0)
{
    return !timer_tick_pending() &&
           timer_elapsed_counts(t0, timer_tcnt1()) < (uint16_t)((FC_TICK_US - NAV_BG_RESERVE_US) * FC_PWM_COUNTS_PER_US);
}

/* ---- LED (nav.h): power-of-two tick periods, no division ---- */
static void nav_led_init(void)
{
    NAV_LED_PORT &= (uint8_t)~(1 << PIN_NAV_LED_BIT);
    NAV_LED_DDR |= (uint8_t)(1 << PIN_NAV_LED_BIT);
}

static void nav_led(uint16_t ph, bool imu_bad, bool baro_bad, bool tag_mismatch, bool fcu_lost, bool pad, bool aligned)
{
    const uint8_t p = (uint8_t)ph;
    bool on;
    if (imu_bad)
        on = (ph & 0x20u) != 0;                          /* fast blink */
    else if (baro_bad)
        on = (ph & 0x80u) != 0;                          /* slow blink */
    else if (tag_mismatch)
        on = p < 0x10u || (p >= 0x20u && p < 0x30u);    /* double flash */
    else if (fcu_lost)
        on = p < 0x18u;                                  /* one short flash */
    else if (pad && !aligned)
        on = p >= 0x18u;                                 /* on, short dark blip */
    else
        on = true;
    if (on)
        NAV_LED_PORT |= (uint8_t)(1 << PIN_NAV_LED_BIT);
    else
        NAV_LED_PORT &= (uint8_t)~(1 << PIN_NAV_LED_BIT);
}

#if FC_VTOL
/* fw09 W1 boot check failed (hal/nlink_spi.h nlink_spi_t3_owned: another driver changed Timer3 after nlink_spi_init - a
 * build error, the same at every boot): the interrupt-paced exchange cannot be trusted, so none is ever started - SS stays
 * high, the FCU sees a NAV that never answers (no valid frame: no EST_VALID, it never arms; in flight its I9 rule).  The
 * loop only kicks the watchdog (no reset storm) and blinks the LED very fast (nav.h).  Never returns. */
static void nav_halt_timer3(void) __attribute__((noreturn, noinline));
static void nav_halt_timer3(void)
{
    timer_set_lead_hook(0);
    for (;;) {
        timer_wait_tick();
        watchdog_kick();
        if ((uint8_t)timer_ticks() & 0x08u)
            NAV_LED_PORT |= (uint8_t)(1 << PIN_NAV_LED_BIT);
        else
            NAV_LED_PORT &= (uint8_t)~(1 << PIN_NAV_LED_BIT);
    }
}
#endif

#if FC_VTOL
/* VTOL build (nav.h NAV_PUSH_MIN_US_VTOL): the exchange does not start before NAV_PUSH_MIN_US_VTOL after the tick start
 * t0, so the frame - and the FCU's frame-triggered tick - jumps by at most the width between that floor and the
 * worst-case compute from one tick to the next, whatever the estimator branch costs; a longer compute pushes at once. */
#define NAV_PUSH_MIN_COUNTS_VTOL ((uint16_t)(NAV_PUSH_MIN_US_VTOL * FC_PWM_COUNTS_PER_US))
static void push_wait(uint16_t t0)
{
#ifdef FC_SIMBENCH
    nav_sb_compute_end(timer_elapsed_counts(t0, timer_tcnt1()) >= NAV_PUSH_MIN_COUNTS_VTOL);
#endif
    while (timer_elapsed_counts(t0, timer_tcnt1()) < NAV_PUSH_MIN_COUNTS_VTOL) {
    }
}

/* v0.8.1 glink receive: at most max bytes of the USART1 ring -> the RESULT decoder (decoded in tick k); returns the
 * bytes taken.  Not inlined: its byte buffer lives only during this call */
static uint8_t glink_rx(uint32_t tick, uint8_t max) __attribute__((noinline));
static uint8_t glink_rx(uint32_t tick, uint8_t max)
{
    uint8_t b[NAV_GL_BG_CHUNK];
    const uint8_t n = glink_uart_read(b, max < sizeof b ? max : (uint8_t)sizeof b);
    if (n != 0u) {
        const uint8_t r = gl_nav_rx(&g_gl, b, n, tick);
#ifdef FC_SIMBENCH
        nav_sb_glink_rx(tick, n, r);
#else
        (void)r;
#endif
    }
    return n;
}

/* every waiting glink byte, in chunks (the ring holds at most two RESULT frames: <= 64 bytes, ~15 us a byte) */
static void glink_rx_all(uint32_t tick)
{
    while (glink_rx(tick, NAV_GL_BG_CHUNK) != 0u) {
    }
}

/* v0.8.1 step 6: the glink part of tick k (the STATE frame at k mod 5 == 4).  Not inlined: the frame buffer (60 B) lives
 * only during this call */
static void glink_tick(uint32_t tick, const float rate_corr[3], bool pad, bool imu_ok, bool est_valid)
    __attribute__((noinline));
static void glink_tick(uint32_t tick, const float rate_corr[3], bool pad, bool imu_ok, bool est_valid)
{
    uint8_t f[GL_STATE_FRAME_BYTES];
    if (gl_nav_tick(&g_gl, tick, g_imu.accel, g_est.q, rate_corr, g_altv.h, pad, g_fcu.have ? &g_fcu4 : 0,
                    g_fcu.age == 0u, imu_ok, est_valid, f)) {
        const bool ok = glink_uart_write(f, GL_STATE_FRAME_BYTES);   /* refused only if the previous one never left */
#ifdef FC_SIMBENCH
        nav_sb_glink_state(tick, ok);
#else
        (void)ok;
#endif
    }
}
#endif

/* boot-time sensor init (v0.5 retries; the results go to reset_cause and the LED, nav.h) */
static uint8_t sensors_init(bool *imu_present, bool *baro_present) __attribute__((noinline));
static uint8_t sensors_init(bool *imu_present, bool *baro_present)
{
    uint8_t rc = 0;
#if NAV_SB_SYNTH_SENSORS
    *imu_present = *baro_present = true;         /* synthetic sources, see nav_simbench.h */
#else
    uint8_t who = 0, i;
    for (i = 0; i < 3 && !mpu6050_whoami_ok(who); i++) {
        who = mpu6050_init();
        if (!mpu6050_whoami_ok(who))
            twi_recover();
    }
    *imu_present = mpu6050_whoami_ok(who);
    *baro_present = baro_init(&g_baro);
#endif
    if (!*imu_present)
        rc |= NAV_RC_IMU_INIT_FAIL;
    if (!*baro_present)
        rc |= NAV_RC_BARO_INIT_FAIL;
    return rc;
}

/* the FCU's half of the exchange: valid -> its state for the next tick; invalid -> the last valid one stays */
#if FC_VTOL
/* v0.8.1: the nlink v4 FCU frame (NL_TYPE_FCU4: + pflags, kt_q for the glink STATE); a v0.8 FCU's NL_TYPE_FCU frame is
 * refused (an FCU of another build: no valid reply, the NAV stays in pad mode - and that FCU refuses the v4 frames) */
static void fcu_reply(const uint8_t *rx)
{
    nl_fcu4_t f;
    if (nl_decode_fcu4(rx, &f)) {
        g_fcu4 = f;
        g_fcu.state = f.state;
        g_fcu.fflags = f.fflags;
        g_fcu.params_tag = f.params_tag;
        g_fcu.thr_q = f.thr_q;                   /* the baro thrust compensation's u */
        g_fcu.have = true;
        g_fcu.age = 0;
        g_fcu.valid++;
    } else {
        if (g_fcu.age < 0xFFFFu)
            g_fcu.age++;
        g_fcu.bad++;
    }
}
#else
static void fcu_reply(void)
{
    nl_fcu_t f;
    if (nl_decode_fcu(g_rx, &f)) {
        g_fcu.state = f.state;
        g_fcu.fflags = f.fflags;
        g_fcu.params_tag = f.params_tag;
        g_fcu.have = true;
        g_fcu.age = 0;
        g_fcu.valid++;
    } else {
        if (g_fcu.age < 0xFFFFu)
            g_fcu.age++;
        g_fcu.bad++;
    }
}
#endif

/* ------------------------------------------------------------------------ */
int main(void)
{
    uint8_t reset_cause = (uint8_t)(watchdog_reset_cause() & NAV_RC_MCUCSR_MASK);
#if !FC_VTOL
    uint8_t i;
#endif
    bool imu_present, baro_present;
    uint16_t imu_fail = 0, overruns = 0, pad_still = 0, crit_us_last = 0;
    bool aligned = false, est_valid = false, have_ground = false;
    float h_ground = 0.0f, h_baro_rel = 0.0f;
    uint32_t last_baro_tick = 0, last_tick = 0;
    uint8_t ph_baro = 0;                                  /* tick % FC_BARO_DIV */
#ifdef FC_SIMBENCH
    static nav_sb_tick_t sb;                              /* this tick's figures for the report (nav_simbench.h) */
#endif
#if FC_VTOL
    bool baro_ok_prev = false, baro_new_prev = false;     /* v0.8: the flags as tick k-1 left them (frame of tick k) */
#endif
    const float a_ground = (FC_BARO_DIV * FC_DT_S) / (CAL_GROUND_BARO_TAU_S + FC_BARO_DIV * FC_DT_S);

    /* ---- 0. the nlink first: SS an output held high before the SPI becomes master ------------------------------- */
    nlink_spi_init();
    nav_led_init();
    {
        uint8_t m = (uint8_t)(MCUCSR | (1 << JTD));      /* JTAG off even if the JTAGEN fuse is left programmed */
        MCUCSR = m;
        MCUCSR = m;
    }
    watchdog_start();
    timer_init();
    nav_timebase_init();                                  /* Timer1: the 0.5 us time base of the loop-time stamps */
    twi_init();
#if FC_VTOL
    glink_uart_init();                                    /* v0.8.1: USART1 = the glink to the GNS, RX interrupt on */
#endif
    sei();
#ifdef FC_SIMBENCH
    nav_sb_init();
#endif
#if FC_VTOL
    fc_default_mahony(&g_mah);
    fc_default_alt_vtol(&g_altv, BARO_LATENCY_S);
#if defined(FC_SIMBENCH) && defined(NAV_SB_COMP_M)
    g_altv.p.comp_m = NAV_SB_COMP_M;                      /* simbench input: time the thrust compensation (nav_simbench.h) */
#endif
    gl_nav_init(&g_gl, GL_STEP_TICKS, fc_glink_latency_ticks());   /* v0.8.1: no RESULT yet: slot header 0x07 */
#else
    fc_default_estimator(&g_est);
    fc_default_alt_filter(&g_alt, BARO_LATENCY_S);
    alt_filter_reset(&g_alt, 0.0f);
#endif

    reset_cause |= sensors_init(&imu_present, &baro_present);
#ifdef FC_SIMBENCH
    nav_sb_prepare(1, ONB_SAFE);
    nav_sb_boot_line(reset_cause);
#endif
#if !NAV_SB_SYNTH_SENSORS
    if (imu_present)
        timer_set_lead_hook(mpu6050_start_async);         /* the burst read starts FC_IMU_LEAD_COUNTS before each tick */
#endif
    /* 2026-11-10 (four-chip co-simulation row 3, found with the BMP280): the tick timer has been running since
     * timer_init, so a tick that fired during the initialisation above is still flagged and the loop's first
     * timer_wait_tick() returned at once - in the middle of that tick (measured: 2.1 ms into it after the BMP280's
     * reset wait + trim read, 1.1 ms with the MS5611).  That first pass then ended inside the next tick's IMU lead
     * (3967 us after its tick with the BMP280; 2981 us with the MS5611, inside the limit only by the length of the
     * initialisation).  A stale tick is dropped here (nothing happens when none is flagged), so the loop always starts
     * at a tick boundary, whatever the sensors' initialisation takes.  Pad only (the vehicle is SAFE); the tick
     * numbering is the timer's and does not change. */
    (void)timer_tick_take();

#if FC_VTOL
    /* fw09 W1: every driver is initialised - Timer3 must still be the nlink's (nlink_spi_init), else no exchange ever */
    if (!nlink_spi_t3_owned())
        nav_halt_timer3();
    /* ---- the v0.8.1 navigation loop (VTOL): predict -> publish -> correct -> altitude -> glink ------------------------ */
    for (;;) {
        uint32_t tick;
        uint16_t t0, crit_us;
        bool pad_alt, on_pad, bench, imu_new, imu_ok, baro_new = false;
        uint8_t flags = 0, thr_q;
        float rate[3];

        timer_wait_tick();
        tick = timer_ticks();
        t0 = timer_tcnt1();
        NAV_SB_COSIM_TICK(tick);                              /* co-simulation: tick marker up (nav_simbench.h) */
        if (last_tick == 0) {                                 /* first tick: the only division */
            ph_baro = (uint8_t)(tick % FC_BARO_DIV);
        } else {
            const uint32_t d = tick - last_tick;
            if (d != 1u && overruns < 0xFFFFu)
                overruns++;                                   /* a tick was missed */
            ph_baro = phase_advance8(ph_baro, d, FC_BARO_DIV);
        }
        last_tick = tick;
        SB_MARK(0);
        watchdog_kick();

        /* 2. sensors: the IMU sample read during the lead time, then the baro */
#if NAV_SB_SYNTH_SENSORS
        mpu6050_convert(nav_sb_imu_raw(), &g_imu);
        imu_new = true;
#else
        imu_new = imu_present && mpu6050_finish_async(&g_imu);
#endif
        if (imu_new) {
            imu_fail = 0;
        } else {
            if (imu_fail < 0xFFFFu)
                imu_fail++;
            if (imu_present && (imu_fail % (ONB_IMU_FAIL_TICKS + 1u)) == ONB_IMU_FAIL_TICKS)
                twi_recover();
        }
        imu_ok = imu_new;                                     /* this tick's read succeeded (I9 counts on the FCU) */
#if NAV_SB_SYNTH_SENSORS
        nav_sb_baro(tick, ph_baro, &g_baro);
#else
        baro_step(&g_baro, ph_baro);                          /* after the IMU read: one bus user at a time */
#endif
        SB_MARK(1);

        /* the tick's pad flags and FCU throttle: the FCU frame of the previous exchange, for the whole tick */
        pad_alt = g_fcu.state == ONB_SAFE || g_fcu.state == ONB_ARMED;
        bench = (g_fcu.fflags & NL_FF_BENCH) != 0u;
        on_pad = pad_alt && !bench;
        thr_q = g_fcu.thr_q;

        /* 3. PREDICT and PUBLISH: the gyro prediction from tick k-1's corrected state; the altitude filter's h / v_z and
         * the flags as tick k-1 left them; the GNS slot of frame k (v0.8.1: the RESULT bytes still waiting decoded first,
         * capped) */
        att_mahony_predict(&g_mah, g_imu.gyro, on_pad, g_nav3.q, g_nav3.rate);
        nl_nav3_tilt(g_nav3.q, &g_nav3.cos_tilt, &g_nav3.tilt_deg);
        nl_nav3_set_physical(&g_nav3, g_imu.accel, h_baro_rel, imu_fail, g_baro.fail_count, twi_error_count());
        g_nav3.h_m = g_altv.h;
        g_nav3.vz_mps = g_altv.v;
        if (imu_ok) flags |= NL_F_IMU_OK;
        if (baro_ok_prev) flags |= NL_F_BARO_OK;
        if (aligned) flags |= NL_F_ALIGNED;
        if (g_imu.gyro_saturated) flags |= NL_F_GYRO_SAT;
        if (est_valid) flags |= NL_F_EST_VALID;
        if (baro_new_prev) flags |= NL_F_BARO_NEW;
        if (!g_fcu.have) flags |= NL_F_BOOT;
        if (g_mah.gate) flags |= NAV_F_EST_GATE;              /* report only (nav.h): the correction of tick k-1 */
        g_nav3.seq = (uint8_t)tick;
        g_nav3.loop_us_max = crit_us_last;                    /* the previous tick (one frame per tick) */
        g_nav3.flags = flags;
        g_nav3.reset_cause = (uint8_t)(reset_cause | (g_fcu.params_tag != FC_PARAMS_TAG ? NAV_RC_TAG_MISMATCH : 0u));
#ifdef FC_SIMBENCH
        {
            const uint16_t a = timer_tcnt1();
            const uint8_t nb = glink_rx(tick, NAV_GL_PREBUILD_BYTES);   /* a RESULT completed since the background */
            nav_sb_slot_build(tick, nb, (uint16_t)(timer_elapsed_counts(a, timer_tcnt1()) / FC_PWM_COUNTS_PER_US));
        }
#else
        (void)glink_rx(tick, NAV_GL_PREBUILD_BYTES);          /* a RESULT completed since the background: this build */
#endif
        gl_nav_slot(&g_gl, tick, g_nav3.slot);
        {
            uint8_t i = nl_encode_nav4(&g_nav3, nlink_spi_buf);   /* P1: into the driver's buffer (in place) */
            while (i < NL_XFER_BYTES)
                nlink_spi_buf[i++] = 0u;                      /* the padding (the last exchange left the FCU's bytes) */
        }
        push_wait(t0);                                        /* not before the push floor (nav.h) */
        SB_MARK(2);
        nlink_spi_exchange_start();                           /* P1: the bytes go out from the Timer3 interrupt during
                                                               * steps 3b - 6; step 7 takes the end */
        SB_MARK(3);

        /* 3b. v0.8.1: the glink bytes that arrived meanwhile (a RESULT for the next frame builds; after the push, so
         * outside the time-critical part, before the correction: the earliest point a RESULT of ~ k_m + 9 ms is taken) */
        glink_rx_all(tick);

        /* 4. CORRECT (the v0.7.1 gated Mahony correction from q_{k-1}, b_{k-1}); the pad-still counter on g - b_k */
        att_mahony_update(&g_mah, g_imu.gyro, g_imu.accel, on_pad);
        att_mahony_rate(&g_mah, g_imu.gyro, rate);
        if (on_pad) {
            const float a_n = sqrtf(g_imu.accel[0] * g_imu.accel[0] + g_imu.accel[1] * g_imu.accel[1] +
                                    g_imu.accel[2] * g_imu.accel[2]);
            if (fabsf(rate[0]) < CAL_PAD_STILL_RAD_S && fabsf(rate[1]) < CAL_PAD_STILL_RAD_S &&
                fabsf(rate[2]) < CAL_PAD_STILL_RAD_S && fabsf(a_n - FC_G0) < CAL_PAD_STILL_ACCEL_TOL) {
                if (pad_still < 0xFFFFu) pad_still++;
            } else {
                pad_still = 0;
            }
        } else {
            pad_still = 0;
        }

        /* 5. altitude (alt_vtol_step, sim/state_machine.py VtolAltitudeFilter) on the CORRECTED attitude: the world-z
         * specific force minus g - on the pad it calibrates the accelerometer z bias (h = v = 0 held), in flight it is
         * integrated with that bias removed; a new baro sample (relative to the ground reference) corrects h, v and the
         * bias, its thrust offset compensated with the FCU's ESC throttle of this tick's FCU frame (thr_q) */
        {
            const float *v = att_mahony_up(&g_mah), *f = g_imu.accel;   /* P1: R(q_k)^T e_z, kept by the estimator */
            const float a_z_w = v[0] * f[0] + v[1] * f[1] + v[2] * f[2] - FC_G0;
            bool has_baro = false;
            if (g_baro.new_sample) {
                g_baro.new_sample = false;
                baro_new = true;
                last_baro_tick = tick;
                if (pad_alt) {                                 /* ground reference: LPF on the pad, frozen at launch */
                    h_ground = have_ground ? h_ground + a_ground * (g_baro.altitude_m - h_ground) : g_baro.altitude_m;
                    have_ground = true;
                }
                h_baro_rel = g_baro.altitude_m - h_ground;
                has_baro = true;
            }
            alt_vtol_step(&g_altv, a_z_w, has_baro, h_baro_rel, tick, pad_alt, nl_fcu_throttle(thr_q));
        }
        /* a sample every FC_BARO_DIV ticks; one tick of slack for a publish that slipped in the background */
        baro_ok_prev = baro_present && g_baro.samples > 0 && (tick - last_baro_tick) <= (uint32_t)FC_BARO_DIV + 1u;
        baro_new_prev = baro_new;
        if (on_pad) {                                         /* ARM gate; frozen (reported) once off the pad */
            aligned = have_ground && g_est.n >= g_est.n_avg && pad_still >= CAL_PAD_STILL_TICKS;
            if (aligned)
                est_valid = true;                             /* latched: the attitude now comes from this alignment */
        }
        SB_MARK(4);

        /* 6. glink: the force sum; at k mod 5 == 4 the STATE frame (q_k, g - b_k, h_k, the pad flag, the FCU4 fields of the
         * previous exchange) into the USART1 TX ring */
        glink_tick(tick, rate, pad_alt, imu_ok, est_valid);
        glink_rx_all(tick);                                   /* what arrived during the correction (a late RESULT) */
        SB_MARK(5);

        /* 7. the end of the exchange (P1: it ran from its interrupt during steps 3b - 6): the time-critical figure = tick ->
         * the SS rise the interrupt stamped; the FCU's reply -> the mode of the next tick; LED.  fw09 W1: the wait is
         * bounded (an exchange that lost its compare interrupt is closed here NLINK_WD_COUNTS after its start: its stamp is
         * that close, its reply is refused, the last valid FCU frame stays) */
        {
            uint16_t t_end;
            const uint8_t *rx = nlink_spi_exchange_wait(&t_end);
            NAV_SB_COSIM_SENT(tick);                          /* co-simulation: tick marker down (exchange over) */
            crit_us = (uint16_t)(timer_elapsed_counts(t0, t_end) / FC_PWM_COUNTS_PER_US);
            crit_us_last = crit_us;                           /* the next frame reports this tick */
#ifdef FC_SIMBENCH
            sb.ready_us = (uint16_t)(nav_sb_ready_us() -
                                     timer_elapsed_counts(t_end, timer_tcnt1()) / FC_PWM_COUNTS_PER_US);
#endif
            fcu_reply(rx);
        }
        nav_led((uint16_t)tick, !imu_present || imu_fail >= ONB_IMU_FAIL_TICKS, !baro_ok_prev,
                g_fcu.have && g_fcu.params_tag != FC_PARAMS_TAG, g_fcu.age > NAV_FCU_LOST_TICKS, pad_alt, aligned);
        SB_MARK(6);
#ifdef FC_SIMBENCH
        sb.work_us = (uint16_t)(timer_elapsed_counts(t0, timer_tcnt1()) / FC_PWM_COUNTS_PER_US);
#endif

        /* 8. background until NAV_BG_RESERVE_US before the next tick: the glink RESULT bytes (v0.8.1, first: the next
         * frame builds latch them), barometer math, stack scan */
        for (;;) {
#ifdef FC_SIMBENCH
            const uint16_t a = timer_tcnt1();
#endif
            if (glink_uart_rx_count() != 0u && !timer_tick_pending() &&
                timer_elapsed_counts(t0, timer_tcnt1()) <
                    (uint16_t)((FC_TICK_US - NAV_GL_BG_RESERVE_US) * FC_PWM_COUNTS_PER_US)) {
                (void)glink_rx(tick, NAV_GL_BG_CHUNK);
            } else if (!bg_time_left(t0) || !baro_background(&g_baro)) {
                break;
            }
#ifdef FC_SIMBENCH
            nav_sb_bg_step((uint16_t)(timer_elapsed_counts(a, timer_tcnt1()) / FC_PWM_COUNTS_PER_US));
#endif
        }
        if (bg_time_left(t0))
            (void)stack_free_min_step();                      /* 64-byte slice of the stack high-water scan */
        SB_MARK(7);
#ifdef FC_SIMBENCH
        sb.tick = tick;
        sb.fcu_state = g_fcu.state;
        sb.pad = pad_alt;
        sb.crit_us = crit_us;
        sb.total_us = (uint16_t)(timer_elapsed_counts(t0, timer_tcnt1()) / FC_PWM_COUNTS_PER_US);
        sb.overruns = overruns;
        sb.fcu_valid = g_fcu.valid;
        sb.fcu_bad = g_fcu.bad;
        sb.loop_us_max = g_nav3.loop_us_max;
        sb.reset_cause = g_nav3.reset_cause;
        sb.gl = &g_gl;
        nav_sb_end_of_loop(&sb);
#endif
    }
#else
    /* ---- the navigation loop ------------------------------------------------------------------------------------- */
    for (;;) {
        uint32_t tick;
        uint16_t t0, crit_us;
        bool pad_alt, on_pad, bench, imu_new, imu_ok, baro_ok, baro_new = false;
        uint8_t flags = 0;
        float rate[3], a_n, tilt_cos, tilt_deg;

        timer_wait_tick();
        tick = timer_ticks();
        t0 = timer_tcnt1();
        NAV_SB_COSIM_TICK(tick);                              /* co-simulation: tick marker up (nav_simbench.h) */
        if (last_tick == 0) {                                 /* first tick: the only division */
            ph_baro = (uint8_t)(tick % FC_BARO_DIV);
        } else {
            const uint32_t d = tick - last_tick;
            if (d != 1u && overruns < 0xFFFFu)
                overruns++;                                   /* a tick was missed */
            ph_baro = phase_advance8(ph_baro, d, FC_BARO_DIV);
        }
        last_tick = tick;
        SB_MARK(0);
        watchdog_kick();

        /* 2. sensors: the IMU sample read during the lead time, then the baro */
#if NAV_SB_SYNTH_SENSORS
        mpu6050_convert(nav_sb_imu_raw(), &g_imu);
        imu_new = true;
#else
        imu_new = imu_present && mpu6050_finish_async(&g_imu);
#endif
        if (imu_new) {
            imu_fail = 0;
        } else {
            if (imu_fail < 0xFFFFu)
                imu_fail++;
            if (imu_present && (imu_fail % (ONB_IMU_FAIL_TICKS + 1u)) == ONB_IMU_FAIL_TICKS)
                twi_recover();
        }
        imu_ok = imu_new;                                     /* this tick's read succeeded (I9 counts on the FCU) */
#if NAV_SB_SYNTH_SENSORS
        nav_sb_baro(tick, ph_baro, &g_baro);
#else
        baro_step(&g_baro, ph_baro);                          /* after the IMU read: one bus user at a time */
#endif
        SB_MARK(1);

        /* 3. attitude estimate: pad alignment while the FCU's last valid state was SAFE/ARMED (bench: gyro-only) */
        pad_alt = g_fcu.state == ONB_SAFE || g_fcu.state == ONB_ARMED;
        bench = (g_fcu.fflags & NL_FF_BENCH) != 0u;
        on_pad = pad_alt && !bench;
        att_est_update(&g_est, g_imu.gyro, g_imu.accel, on_pad);
        att_est_rate(&g_est, g_imu.gyro, rate);
        tilt_cos = quat_tilt_cos(g_est.q);
        tilt_deg = acosf(fc_clampf(tilt_cos, -1.0f, 1.0f)) * FC_RAD2DEG_F;
        if (on_pad) {
            a_n = sqrtf(g_imu.accel[0] * g_imu.accel[0] + g_imu.accel[1] * g_imu.accel[1] +
                        g_imu.accel[2] * g_imu.accel[2]);
            if (fabsf(rate[0]) < CAL_PAD_STILL_RAD_S && fabsf(rate[1]) < CAL_PAD_STILL_RAD_S &&
                fabsf(rate[2]) < CAL_PAD_STILL_RAD_S && fabsf(a_n - FC_G0) < CAL_PAD_STILL_ACCEL_TOL) {
                if (pad_still < 0xFFFFu) pad_still++;
            } else {
                pad_still = 0;
            }
        } else {
            pad_still = 0;
        }

        /* 4. altitude: held at the pad reference until launch, then predict + correct */
        if (pad_alt) {
            g_alt.h = 0.0f;
            g_alt.v = 0.0f;
        } else {                                              /* world-z specific force minus g: R(q) row 2 */
            const float *q = g_est.q, *f = g_imu.accel;
            float a_z_w = 2.0f * (q[1] * q[3] - q[0] * q[2]) * f[0] + 2.0f * (q[2] * q[3] + q[0] * q[1]) * f[1] +
                          tilt_cos * f[2] - FC_G0;
            alt_filter_predict(&g_alt, a_z_w, FC_DT_S);
        }
        if (g_baro.new_sample) {
            g_baro.new_sample = false;
            baro_new = true;
            last_baro_tick = tick;
            if (pad_alt) {                                     /* ground reference: LPF on the pad, frozen at launch */
                h_ground = have_ground ? h_ground + a_ground * (g_baro.altitude_m - h_ground) : g_baro.altitude_m;
                have_ground = true;
            }
            h_baro_rel = g_baro.altitude_m - h_ground;
            alt_filter_correct(&g_alt, h_baro_rel, tick);
        }
        /* a sample every FC_BARO_DIV ticks; one tick of slack for a publish that slipped in the background */
        baro_ok = baro_present && g_baro.samples > 0 && (tick - last_baro_tick) <= (uint32_t)FC_BARO_DIV + 1u;
        if (on_pad) {                                         /* ARM gate; frozen (reported) once off the pad */
            aligned = have_ground && g_est.n >= g_est.n_avg && pad_still >= CAL_PAD_STILL_TICKS;
            if (aligned)
                est_valid = true;                             /* latched: the attitude now comes from this alignment */
        }
        SB_MARK(2);

        /* 5. the navigation frame -> FCU; the FCU's state bytes come back in the same exchange */
        if (imu_ok) flags |= NL_F_IMU_OK;
        if (baro_ok) flags |= NL_F_BARO_OK;
        if (aligned) flags |= NL_F_ALIGNED;
        if (g_imu.gyro_saturated) flags |= NL_F_GYRO_SAT;
        if (est_valid) flags |= NL_F_EST_VALID;
        if (baro_new) flags |= NL_F_BARO_NEW;
        if (!g_fcu.have) flags |= NL_F_BOOT;
        g_nav.seq = (uint8_t)tick;
        for (i = 0; i < 4; i++)
            g_nav.q[i] = g_est.q[i];
        for (i = 0; i < 3; i++) {
            g_nav.rate[i] = rate[i];
            g_nav.accel[i] = g_imu.accel[i];
        }
        g_nav.h_m = g_alt.h;
        g_nav.vz_mps = g_alt.v;
        g_nav.h_baro_rel_m = h_baro_rel;
        g_nav.tilt_deg = tilt_deg;
        g_nav.imu_fail = imu_fail;
        g_nav.baro_fails = g_baro.fail_count;
        g_nav.twi_errors = twi_error_count();
        g_nav.loop_us_max = crit_us_last;                     /* the previous tick (one frame per tick) */
        g_nav.flags = flags;
        g_nav.reset_cause = (uint8_t)(reset_cause | (g_fcu.params_tag != FC_PARAMS_TAG ? NAV_RC_TAG_MISMATCH : 0u));
        (void)nl_encode_nav(&g_nav, g_tx);
        SB_MARK(3);
        nlink_spi_exchange(g_tx, g_rx);
        NAV_SB_COSIM_SENT(tick);                              /* co-simulation: tick marker down (frame delivered) */
        SB_MARK(4);

        /* loop time of the time-critical part: tick -> frame delivered (Timer1, FC_PWM_COUNTS_PER_US per us) */
        crit_us = (uint16_t)(timer_elapsed_counts(t0, timer_tcnt1()) / FC_PWM_COUNTS_PER_US);
        crit_us_last = crit_us;                               /* the next frame reports this tick */
#ifdef FC_SIMBENCH
        sb.ready_us = nav_sb_ready_us();
#endif

        /* 6. the FCU's reply -> the mode of the next tick; LED */
        fcu_reply();
        nav_led((uint16_t)tick, !imu_present || imu_fail >= ONB_IMU_FAIL_TICKS, !baro_ok,
                g_fcu.have && g_fcu.params_tag != FC_PARAMS_TAG, g_fcu.age > NAV_FCU_LOST_TICKS, pad_alt, aligned);
        SB_MARK(5);

        /* 7. background until NAV_BG_RESERVE_US before the next tick: barometer math, stack scan */
        for (;;) {
#ifdef FC_SIMBENCH
            const uint16_t a = timer_tcnt1();
#endif
            if (!bg_time_left(t0) || !baro_background(&g_baro))
                break;
#ifdef FC_SIMBENCH
            nav_sb_bg_step((uint16_t)(timer_elapsed_counts(a, timer_tcnt1()) / FC_PWM_COUNTS_PER_US));
#endif
        }
        if (bg_time_left(t0))
            (void)stack_free_min_step();                      /* 64-byte slice of the stack high-water scan */
        SB_MARK(6);
#ifdef FC_SIMBENCH
        sb.tick = tick;
        sb.fcu_state = g_fcu.state;
        sb.pad = pad_alt;
        sb.crit_us = crit_us;
        sb.total_us = (uint16_t)(timer_elapsed_counts(t0, timer_tcnt1()) / FC_PWM_COUNTS_PER_US);
        sb.overruns = overruns;
        sb.fcu_valid = g_fcu.valid;
        sb.fcu_bad = g_fcu.bad;
        sb.loop_us_max = g_nav.loop_us_max;
        sb.reset_cause = g_nav.reset_cause;
        nav_sb_end_of_loop(&sb);
#endif
    }
#endif /* FC_VTOL */
}
