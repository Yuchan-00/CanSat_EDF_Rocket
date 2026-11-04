/* gns_main.c - ATmega128A #4, the GNS board (GPS navigation + station keeping) of the v0.8.1 four-chip set.
 *
 * Why (architect 2026-10-21 20:30, params flight_computer.architecture_v081 / gns / glink, electronics.gns): the v0.8
 * integration measured the three-chip GPS build as not flyable (NAV mandatory work 98.7 % mean / 119.5 % max of the 4 ms
 * tick, FCU time-critical 83.3 % / 89.2 % with ISRs, no ilink command decoded in position hold).  This board runs the
 * u-blox NEO-6M (USART1 PD2 / PD3 through a BSS138, GPS_BAUD 8N1 after the boot configuration), UBX, the GPS-aided
 * horizontal filter (core/gps_nav.h) and the position controller (core/position_ctrl.h) at 50 Hz; it exchanges one STATE /
 * RESULT pair per cycle with the NAV over the glink (USART0 PE0 / PE1, GLINK_BAUD, hal/glink_uart.h; core/glink_proto.h);
 * the NAV copies the RESULT into the nlink v4 slot and the FCU applies the tilt setpoint.  The GNS makes NO safety
 * decision: its LAND requests (V11-V14) are data in the RESULT's STATUS part for the FCU's supervisor, and the FCU detects a
 * silent GNS itself (no TILT part for land_steps cycles -> its own GPS_LOST, core/nlink_proto.h nl_gns_rx).
 *
 * Main loop (core/gns_node.h, the normative order of sim/glink_proto.py GnsNode; no timer work, no flight output):
 *   1. glink bytes first (the latency path): every byte the USART0 RX interrupt put into the ring -> gns_rx_byte; when a
 *      valid STATE frame completes, the 50 Hz cycle runs at once (gns_on_state: the NAV-tick clock, the gap rules, the
 *      filter STEP, the position step, the RESULT frame) and its RESULT goes into the USART0 TX ring (26 B, 1.04 ms on the
 *      line, UDRE interrupt); then gns_finish - the epoch fusion of that cycle - while the RESULT shifts out (off the
 *      latency path).
 *   2. every Timer2 tick (4 ms, FC_TICK_US: the NAV's tick rate; hal/timer.h): gns_poll with the GNS's NAV-tick clock
 *      nt + the ticks elapsed (nt := STATE tick + 1 at each valid STATE, + 1 per 4 ms between frames) and the bytes the
 *      USART1 RX interrupt put into the GPS ring (hal/gps_uart.h) - the UBX parser, the boot configurator (its baud switch,
 *      then its CFG / MON-VER frame into the GPS TX ring), the epoch assembler and the iTOW clock; gps_uart_poll (a pending
 *      baud switch); watchdog; LED.
 *   3. sleep until the next interrupt (glink / GPS bytes, the tick) - timer_idle checks the rings with interrupts off.
 * Budget: one cycle per 20 ms (params flight_computer.gns.loop_hz 50).  The core agent's simavr bench: worst cycle 7.14 ms
 * incl. the fusion (3.67 ms to the RESULT); tools/run_gns_simbench.py measures this loop (gns_simbench).
 *
 * LED (PC0, board.h GNS_LED_*), in this priority:
 *   no valid STATE frame for GNS_NAV_LOST_TICKS (NAV silent / glink cut)      one short flash every 256 ticks (~1 s)
 *   receiver not configured yet (no ACKed CFG set)                            fast blink (~4 Hz)
 *   configured, no current GPS fix (FIX_OK clear)                             slow blink (~1 Hz)
 *   fix, ON gate not passed yet (GATE_OK clear)                               double flash
 *   GATE_OK (the FCU will accept ON)                                          steady on
 * It only reports - like the NAV's.
 *
 * Physical limits stated: GPS needs open sky (no position hold indoors - the indoor build is fcu_nogps); the hold is to
 * within the GPS wander (~1 m RMS / 2.5 m max until a real module is measured); no ground-effect model.
 * Invariants: the glink and GPS UARTs never wait (hal/glink_uart.h, hal/gps_uart.h); bounded work per received byte; the
 * watchdog resets a hung loop (the NAV then sends SLOT_EMPTY: the FCU's TRIM fallback and its own V11).
 */
#include <avr/interrupt.h>
#include <avr/io.h>

#include "board.h"
#include "fc_mode.h"
#include "params_gen.h"

#include "core/fc_config.h"
#include "core/gns_node.h"
#include "core/gps_ubx.h"

#include "hal/glink_uart.h"
#include "hal/gps_uart.h"
#include "hal/stack.h"
#include "hal/timer.h"
#include "hal/watchdog.h"
#include "nav/nav.h"                                       /* nav_timebase_init: Timer1 as the 0.5 us time base */
#include "gns/gns_simbench.h"

#ifndef FC_ROLE_GNS
#error "gns_main.c is the GNS firmware: build it with -D FC_ROLE_GNS (platformio.ini [gns_common])"
#endif
#if !FC_VTOL
#error "the GNS board belongs to the VTOL mission (params.json mission.mode vtol)"
#endif
#if !defined(GLINK_BAUD) || !defined(GNS_LOOP_HZ) || !defined(GPS_BAUD)
#error "GLINK_* / GNS_LOOP_HZ / GPS_* missing: re-run tools/gen_firmware_params.py (params flight_computer.glink / gns)"
#endif
#if GNS_LOOP_HZ * GL_STEP_TICKS != FC_LOOP_HZ
#error "the GNS cycle must be GL_STEP_TICKS NAV ticks (params flight_computer.gns.loop_hz x 5 = control.loop_hz)"
#endif
/* fw09 G1 (SPEC_firmware_v09 3.2): the position controller flies the thrust-to-weight ratio of ITS vehicle configuration
 * (params_gen.h POS_TW per FC_VEHICLE, selected by -D FC_VCFG: gns 0 battery, gns_wired 1 tether, gns_wired_buffer 2
 * hybrid; gns_demo 0).  params_gen.h already refuses an AVR VTOL build without FC_VCFG or with the rocket's set; this
 * image states it once more for its own role, because the GNS has no run-time check: the glink carries no params tag,
 * so a GNS image of another configuration would go unnoticed by the NAV and the FCU - flash the GNS of the same
 * configuration set as the NAV / FCU / COM (platformio.ini, the vehicle table) */
#if !defined(FC_VCFG) || !defined(FC_VEHICLE) || FC_VEHICLE != FC_VCFG || FC_VEHICLE == FC_VCFG_ROCKET || \
    !defined(POS_TW) || !defined(FC_VEHICLE_NAME)
#error "the GNS image flies ONE VTOL vehicle configuration (fw09 G1): -D FC_VCFG=0 (battery), 1 (tether) or 2 (hybrid)"
#endif

#define FW_VERSION_GNS "1.0.0"                             /* v0.8.1: the first GNS image */
#define GNS_NAV_LOST_TICKS 250u                            /* LED: no STATE frame for 1 s (information only) */

static gns_t g_gns;

/* the configurator's frame (CFG-* / MON-VER poll) into the GPS TX ring; refused only with a full ring (the configurator
 * retries).  Not inlined: its frame buffer lives only during this call */
static void gps_send_cfg(uint8_t tx) __attribute__((noinline));
static void gps_send_cfg(uint8_t tx)
{
    uint8_t f[UBX_FRAME_MAX];
    const uint16_t len = gps_cfg_frame(tx, g_gns.gps.cfg.baud, GPS_MEAS_MS, f);
    (void)gps_uart_write(f, (uint8_t)len);
}

/* step 2: the 4 ms poll for d ticks elapsed.  Not inlined: its byte buffer lives only during this call */
static void gps_poll_step(uint32_t d) __attribute__((noinline));
static void gps_poll_step(uint32_t d)
{
    uint8_t rx[GPS_UART_RX_BUF];
    gps_cfg_action_t act;
#ifdef FC_SIMBENCH
    const uint16_t a = timer_tcnt1();
#endif
    const uint8_t n = gps_uart_read(rx, (uint8_t)sizeof rx);
    gns_poll(&g_gns, g_gns.nt + d, rx, n, &act);
    if (act.baud != 0u)
        gps_uart_set_baud(act.baud);                       /* first the baud (deferred while sending) ... */
    if (act.tx != GPS_TX_NONE)
        gps_send_cfg(act.tx);                              /* ... then the configurator's frame */
    gps_uart_poll();
#ifdef FC_SIMBENCH
    gns_sb_poll(n, (uint16_t)(timer_elapsed_counts(a, timer_tcnt1()) / FC_PWM_COUNTS_PER_US));
#endif
}

/* step 1: the glink bytes waiting; returns true when a cycle ran.  Not inlined: the buffers live only during this call */
static bool glink_step(void) __attribute__((noinline));
static bool glink_step(void)
{
    uint8_t b[GLINK_UART_RX_BUF / 2u], out[GL_RESULT_FRAME_BYTES], n, i;
    bool cycle = false;
    while ((n = glink_uart_read(b, (uint8_t)sizeof b)) != 0u) {
        for (i = 0; i < n; i++) {
            if (gns_rx_byte(&g_gns, b[i], out)) {          /* a STATE frame completed: its cycle ran, RESULT in out */
                const bool ok = glink_uart_write(out, GL_RESULT_FRAME_BYTES);
#ifdef FC_SIMBENCH
                gns_sb_result(ok);
#else
                (void)ok;
#endif
                (void)gns_finish(&g_gns);                  /* the fusion while the RESULT shifts out */
#ifdef FC_SIMBENCH
                gns_sb_finished(&g_gns);
#endif
                cycle = true;
            }
        }
    }
    return cycle;
}

static bool work_waiting(void)                             /* called by timer_idle with interrupts off */
{
    return glink_uart_rx_count() != 0u || gps_uart_rx_count() != 0u;
}

static void led_init(void)
{
    GNS_LED_PORT &= (uint8_t)~(1 << PIN_GNS_LED_BIT);
    GNS_LED_DDR |= (uint8_t)(1 << PIN_GNS_LED_BIT);
}

static void led(uint16_t ph, bool nav_lost)
{
    const uint8_t p = (uint8_t)ph, f = g_gns.gps.filter.snap.flags;
    bool on;
    if (nav_lost)
        on = p < 0x18u;                                    /* one short flash */
    else if (!(f & GPS_F_CFG_OK))
        on = (ph & 0x20u) != 0;                            /* fast blink */
    else if (!(f & GPS_F_FIX_OK))
        on = (ph & 0x80u) != 0;                            /* slow blink */
    else if (!(f & GPS_F_GATE_OK))
        on = p < 0x10u || (p >= 0x20u && p < 0x30u);       /* double flash */
    else
        on = true;
    if (on)
        GNS_LED_PORT |= (uint8_t)(1 << PIN_GNS_LED_BIT);
    else
        GNS_LED_PORT &= (uint8_t)~(1 << PIN_GNS_LED_BIT);
}

int main(void)
{
    const uint8_t reset_cause = watchdog_reset_cause();
    uint32_t last_t, last_state_t = 0;
    uint16_t states_seen = 0;

    led_init();
    {
        uint8_t m = (uint8_t)(MCUCSR | (1 << JTD));       /* JTAG off even if the JTAGEN fuse is left programmed */
        MCUCSR = m;
        MCUCSR = m;
    }
    watchdog_start();
    timer_init();                                          /* Timer2: the 4 ms poll tick (Timer0: no lead hook here) */
    nav_timebase_init();                                   /* Timer1: the 0.5 us time base (the bench's figures) */
    glink_uart_init();                                     /* USART0: the glink, RX interrupt on */
    gps_uart_init();                                       /* USART1: the NEO-6M at GPS_BAUD (the configurator switches) */
    sei();
    fc_default_gns(&g_gns, true);                          /* the boot configurator on: CFG-PRT / RATE / MSG / NAV5, MON-VER */
#ifdef FC_SIMBENCH
    gns_sb_init(reset_cause);
#else
    (void)reset_cause;
#endif
    last_t = timer_ticks();
    for (;;) {
        uint32_t t;
#ifdef FC_SIMBENCH
        gns_sb_pass_start();
#endif
        /* 1. the glink first: a completed STATE frame runs the cycle and answers at once */
        if (glink_step()) {
            last_state_t = timer_ticks();
            states_seen++;
        }
        /* 2. the 4 ms poll(s) */
        if (timer_tick_take() && (t = timer_ticks()) != last_t) {
            const uint32_t d = t - last_t;
            last_t = t;
            watchdog_kick();
            gps_poll_step(d);
            led((uint16_t)t, states_seen == 0u || (t - last_state_t) > GNS_NAV_LOST_TICKS);
            if (!timer_tick_pending())
                (void)stack_free_min_step();               /* 64-byte slice of the stack high-water scan */
#ifdef FC_SIMBENCH
            gns_sb_tick(t, &g_gns);
#endif
        }
        /* 3. sleep until an interrupt (bytes or the tick) */
#ifdef FC_SIMBENCH
        gns_sb_pass_end();
#endif
        timer_idle(work_waiting);
    }
}
