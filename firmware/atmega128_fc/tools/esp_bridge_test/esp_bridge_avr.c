/* esp_bridge_avr (2026-11-06): the COM's ESP bridge backend (src/hal/esp_bridge.c) on the AVR, in simavr, driven by
 * esp_bridge_sim.c.  Built with the -D flags of the env com_esp_bridge by run_esp_bridge_test.py.  It runs the backend the
 * way com_main.c does (radio_pins_safe first, interrupts on, radio_init, then one radio_service step and one radio_read
 * per pass with the 4 ms COM tick) and reports on USART0:
 *   EB pins ddr6=<PC6 output?> port6=<PC6 level> after radio_pins_safe (the ESP held in reset)
 *   EB init t=<us after the release>              radio_init returned
 *   RX t=<us> b=<hex>                             every byte radio_read returned
 *   TX t=<us> q=<1 queued> q2=<0: second send refused while busy>   one 47-byte reply at 900 ms
 *   TX busy=<radio_tx_busy() 200 ms later>
 *   EB stats rx=.. err=.. ovf=.. disc=.. txf=.. ok=.. boot_ticks=.. hold_ms=..
 *   EB end
 * Times are Timer1 microseconds since main() (clk / 8, extended in software).  Ends with a sleep, interrupts off.
 * EB_RESET_CAUSE (-D, default 0 = no flag: a cold boot) stands in for hal/watchdog.c's MCUCSR capture: the backend resets
 * the ESP only on a cold COM boot (review F2); run_esp_bridge_test.py builds a cold and a watchdog-reset image. */
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <stdint.h>
#include <stdio.h>

#include "hal/esp_bridge.h"
#include "hal/radio.h"
#include "params_gen.h"

#ifndef EB_RESET_CAUSE
#define EB_RESET_CAUSE 0
#endif

uint8_t watchdog_reset_cause(void);
uint8_t watchdog_reset_cause(void)
{
    return (uint8_t)EB_RESET_CAUSE;
}

static int uart_putchar(char c, FILE *s)
{
    (void)s;
    while (!(UCSR0A & (1 << UDRE0))) {
    }
    UDR0 = (uint8_t)c;
    return 0;
}
static FILE uart_out = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

static uint32_t g_acc;
static uint16_t g_last;

static uint32_t now_us(void)
{
    const uint16_t t = TCNT1;
    g_acc += (uint16_t)(t - g_last);
    g_last = t;
    return g_acc / 2u;
}

int main(void)
{
    uint8_t frame[47], buf[8], i, n;
    uint32_t t, t_rel;
    bool sent = false, checked = false;
    const radio_stats_t *rs;
    TCCR1A = 0;
    TCCR1B = (uint8_t)(1 << CS11);                      /* clk / 8: 2 counts per us */
    g_last = TCNT1;
    radio_pins_safe();
    UBRR0H = 0;
    UBRR0L = 0;                                          /* console 1 Mbaud */
    UCSR0B = (uint8_t)(1 << TXEN0);
    UCSR0C = (uint8_t)((1 << UCSZ01) | (1 << UCSZ00));
    stdout = &uart_out;
    printf("EB pins ddr6=%u port6=%u\n", (DDRC >> 6) & 1u, (PORTC >> 6) & 1u);
    for (i = 0; i < sizeof frame; i++)
        frame[i] = (uint8_t)(0xA5 ^ (i * 37u));
    sei();
    t = now_us();
    radio_init();
    t_rel = now_us();
    printf("EB init t=%lu hold_us=%lu\n", (unsigned long)t_rel, (unsigned long)(t_rel - t));
    for (;;) {
        t = now_us();
        if (t >= 1200000UL)
            break;
        (void)radio_service(t / 4000u);                  /* the COM tick (COM_TICK_HZ 250) */
        n = radio_read(buf, sizeof buf);
        for (i = 0; i < n; i++)
            printf("RX t=%lu b=%02x\n", (unsigned long)t, buf[i]);
        if (!sent && t >= 900000UL) {
            const bool q = radio_send(frame, sizeof frame), q2 = radio_send(frame, sizeof frame);
            sent = true;
            printf("TX t=%lu q=%u q2=%u\n", (unsigned long)t, q, q2);
        }
        if (sent && !checked && t >= 1100000UL) {
            checked = true;
            printf("TX busy=%u\n", radio_tx_busy());
        }
    }
    rs = radio_stats();
    printf("EB stats rx=%u err=%u ovf=%u disc=%u txf=%u ok=%u boot_ticks=%lu hold_ms=%u\n", rs->rx_bytes, rs->rx_errors,
           rs->rx_overflow, esp_bridge_rx_discarded(), rs->tx_frames, rs->ok, (unsigned long)ESP_BOOT_TICKS,
           (unsigned)ESP_RESET_HOLD_MS);
    printf("EB end\n");
    cli();
    sleep_enable();
    sleep_cpu();
    return 0;
}
