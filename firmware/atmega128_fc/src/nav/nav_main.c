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
 * v0.7.1 - v0.9 also had VTOL builds (the gated Mahony estimator, the VTOL altitude filter, the nlink v3 / v4 frames, the
 * GPS and later the glink to the GNS board); they went with the VTOL program on 2026-11-18.  The rocket's NAV
 * (nav_ballistic, -D FC_MODE_BALLISTIC) is everything above - the v0.6.3 estimator (PadAlignGyroEstimator) and altitude
 * filter, bit for bit - and ignores the FCU frame's thr_q byte.
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
/* fw09 W1: Timer3 is the VTOL nlink's (hal/nlink_spi.h): no other Timer3 driver in the VTOL NAV (hal/pwm.c drives Timer1 /
 * Timer3 PWM, hal/gps_uart.c's bench wire Timer3 compare A; the glink bench wire checks itself, hal/glink_uart.c) */
/* the latest W1 close (start + the watchdog; the start is >= the push floor after the tick) must stay inside the tick */

#ifdef FC_SIMBENCH
#define SB_MARK(i) nav_sb_mark(i)
#else
#define SB_MARK(i) ((void)0)
#endif

/* the ballistic build: the v0.6.3 estimator and altitude filter, bit for bit */
static att_est_t g_est;
static alt_filter_t g_alt;
static imu_sample_t g_imu;
static baro_t g_baro;                                  /* hal/baro.h: BMP280 (default) or MS5611 */
static nl_nav_t g_nav;
static uint8_t g_tx[NL_XFER_BYTES], g_rx[NL_XFER_BYTES];

/* the FCU as the NAV knows it: its last valid frame */
static struct {
    uint8_t state, fflags, params_tag;
    bool have;                                   /* a valid FCU frame since this reset (else BOOT) */
    uint16_t age;                                /* NAV ticks since it (saturating) */
    uint16_t valid, bad;                         /* replies accepted / refused (simbench report) */
} g_fcu = {ONB_SAFE, 0u, FC_PARAMS_TAG, false, 0xFFFFu, 0u, 0u
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

/* ------------------------------------------------------------------------ */
int main(void)
{
    uint8_t reset_cause = (uint8_t)(watchdog_reset_cause() & NAV_RC_MCUCSR_MASK);
    uint8_t i;
    bool imu_present, baro_present;
    uint16_t imu_fail = 0, overruns = 0, pad_still = 0, crit_us_last = 0;
    bool aligned = false, est_valid = false, have_ground = false;
    float h_ground = 0.0f, h_baro_rel = 0.0f;
    uint32_t last_baro_tick = 0, last_tick = 0;
    uint8_t ph_baro = 0;                                  /* tick % FC_BARO_DIV */
#ifdef FC_SIMBENCH
    static nav_sb_tick_t sb;                              /* this tick's figures for the report (nav_simbench.h) */
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
    sei();
#ifdef FC_SIMBENCH
    nav_sb_init();
#endif
    fc_default_estimator(&g_est);
    fc_default_alt_filter(&g_alt, BARO_LATENCY_S);
    alt_filter_reset(&g_alt, 0.0f);

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
}
