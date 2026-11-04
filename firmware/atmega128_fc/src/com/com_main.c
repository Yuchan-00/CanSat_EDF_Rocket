/* com_main.c - the COM (LoRa link processor) of the flight computer (v0.6.3: NAV + FCU + COM, SPEC 10.4,
 * params.json flight_computer.com).  Built by the platformio.ini COM envs (-D FC_ROLE_COM).
 *
 * The COM moves the ground link between the radio and the FCU and makes no flight decision:
 * uplink CMD frames are decoded (core/link_proto.c) and forwarded as IL_CMD; the FCU's reply
 * snapshots (IL_SNAP, the only ilink type the COM's decoder accepts) become TELEM frames when they
 * answer an IL_CMD of the last 80 ms; an IL_STAT goes to the FCU right behind an IL_CMD at most once
 * per ILINK_STAT_DIV ticks (on its own divider while no uplink comes) and once after boot, no earlier
 * than COM_STAT_BOOT_MS after reset (src/com/com_link.h D4, R3).  The onboard supervisor, the
 * interlocks I1-I10, the nonce and every state change stay on the FCU: a dead COM is a lost radio
 * link for the FCU (I1p / I1).  The COM has no part in the NAV <-> FCU nlink.
 *
 * Boot: radio pins safe (Ebyte M0/M1 low = normal mode; SX127x deselected and held in reset),
 * status LED off, watchdog (WDT_TIMEOUT), COM time base (Timer2 tick + free-running Timer1,
 * src/com/com_timer.c), ilink USART0, interrupts on, radio_init (SX127x: ~12 ms with the
 * watchdog kicked), decoders; the boot IL_STAT (IL_STAT_BOOT) is the first costly job once the COM
 * tick reaches COM_STAT_BOOT_TICKS (100 ms: an FCU that started with the COM is listening by then).
 *
 * Main loop: no hard deadline, every step bounded.  One pass = watchdog kick; once per COM tick
 * the LED; one radio_service step; <= COM_RADIO_CHUNK uplink bytes decoded (IL_CMD queued at once);
 * <= COM_ILINK_CHUNK ilink bytes decoded; at most one costly job (reply stage 1 or 2, or the STAT);
 * the pending TELEM to the radio if it is free; a 64-byte slice of the stack scan.  The pass time
 * (Timer1) feeds the STAT's loop_us_max.  The ilink receive ring (64 B = 2.56 ms of bytes at
 * 250 kbaud) bounds how long a pass may take.
 *
 * LED (PIN_COM_LED, active high): radio missing (SX127x RegVersion wrong) -> fast blink; otherwise
 * on for COM_LED_UPLINK_TICKS after every forwarded uplink frame, else a short flash every 1.02 s.
 *
 * Invariant: the COM transmits on the radio only a TELEM built from an FCU SNAP (com_link.h); it
 * never waits on the radio or the ilink, so the watchdog only fires on a real hang.
 */
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>

#include "../board.h"
#include "../params_gen.h"
#include "../hal/ilink_uart.h"
#include "../hal/radio.h"
#include "../hal/stack.h"
#include "../hal/watchdog.h"
#include "com_link.h"
#include "com_simbench.h"
#include "com_timer.h"

#ifndef FC_ROLE_COM
#error "src/com/ is the COM firmware: build it with -D FC_ROLE_COM (platformio.ini [com_common])"
#endif
#if ILINK_USART != ILINK_USART_COM
#error "the COM's ilink runs on USART0 (platformio.ini [com_common] -D ILINK_USART=0)"
#endif
#if FC_LINK_RADIO != FC_RADIO_LORA_UART && FC_LINK_RADIO != FC_RADIO_SX127X
#error "FC_LINK_RADIO must be FC_RADIO_LORA_UART or FC_RADIO_SX127X"
#endif

#define COM_LED_UPLINK_TICKS 25u             /* 100 ms on after each forwarded uplink */

/* last complete stack high-water scan (bytes never touched); volatile: kept for the debugger / bench */
static volatile uint16_t g_stack_free = 0xFFFFu;

static void led_set(bool on)
{
    if (on)
        COM_LED_PORT |= (uint8_t)(1 << PIN_COM_LED_BIT);
    else
        COM_LED_PORT &= (uint8_t)~(1 << PIN_COM_LED_BIT);
}

/* power-of-two periods of the free-running tick: no division */
static void led_update(uint16_t ph, bool radio_ok, bool uplink)
{
    if (!radio_ok)
        led_set((ph & 0x20u) != 0u);                 /* toggles every 32 ticks */
    else if (uplink)
        led_set(true);
    else
        led_set((ph & 0xFFu) < 0x08u);               /* 32 ms flash every 256 ticks */
}

int main(void)
{
    const uint8_t reset_cause = watchdog_reset_cause();
    uint32_t last_tick;
    uint16_t led_hold = 0, fwd_seen = 0;

    /* ---- safe pins first --------------------------------------------------------------- */
    radio_pins_safe();
    led_set(false);
    COM_LED_DDR |= (uint8_t)(1 << PIN_COM_LED_BIT);
    {
        const uint8_t m = (uint8_t)(MCUCSR | (1 << JTD));   /* JTAG off even if the JTAGEN fuse is left programmed */
        MCUCSR = m;
        MCUCSR = m;
    }
    watchdog_start();
    com_timer_init();
    ilink_uart_init();
#ifdef FC_SIMBENCH
    com_sb_init(reset_cause);
#endif
    sei();
    radio_init();
    com_link_init(reset_cause);
    last_tick = com_timer_ticks();

    /* ---- main loop ------------------------------------------------------------------------ */
    for (;;) {
        uint32_t tick;
        uint16_t t0, pass_us, sf;
#ifdef FC_SIMBENCH
        com_sb_pass();                               /* synthetic ground (and FCU): outside the measured pass */
#endif
        t0 = com_timer_tcnt1();
        COM_SB_MARK(0);
        watchdog_kick();
        tick = com_timer_ticks();
        if (tick != last_tick) {                     /* once per COM tick (more than one after a long pass) */
            uint32_t d = tick - last_tick;
            const uint16_t fwd = com_link_cmd_forwarded();
            last_tick = tick;
            if (d > COM_LED_UPLINK_TICKS)
                d = COM_LED_UPLINK_TICKS;
            if (fwd != fwd_seen) {
                fwd_seen = fwd;
                led_hold = COM_LED_UPLINK_TICKS;
            } else if (led_hold > 0u) {
                led_hold = (uint16_t)(led_hold - (d < led_hold ? (uint16_t)d : led_hold));
            }
            led_update((uint16_t)tick, radio_stats()->ok, led_hold > 0u);
        }
        (void)radio_service(tick);                   /* one bounded step (SX127x: one register access or FIFO chunk) */
        COM_SB_MARK(1);
        com_link_uplink(tick);
        COM_SB_MARK(2);
        com_link_ilink(tick);
        COM_SB_MARK(3);
        (void)com_link_job(tick);
        COM_SB_MARK(4);
        com_link_send(tick);
        sf = stack_free_min_step();
        g_stack_free = sf;
        COM_SB_MARK(5);
        pass_us = com_timer_us_since(t0, com_timer_tcnt1());
        com_link_pass_us(pass_us);
#ifdef FC_SIMBENCH
        com_sb_end_of_pass(pass_us, sf);
#endif
    }
}
