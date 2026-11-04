/* adc_chain_avr (fw09 review F2): the v0.9 power builds' ADC conversion chain (src/hal/adc.c: adc_chain_init,
 * adc_chain_take, the ADC interrupt, adc_chain_vin) run on the AVR, in simavr, with distinct voltages on the chain's input
 * channels (adc_chain_sim.c drives them).  The flight images run this chain; every bench / co-simulation build replaces
 * it by simbench_adc_chain (constant codes), so this test is where the flight path runs in simulation.
 * Built with a power FCU env's -D flags (PWR_SRC / FC_VCFG select PM_WIRED or PM_BUF and the channels) by
 * run_adc_chain_test.py; output on USART0:
 *   ADC conv_us=<one conversion, polled, Timer1>    (simavr converts faster than the chip's 13 ADC clocks = 104 us, so
 *                                                    the partial-chain steps wait fractions of this measured time)
 *   ADC t=<step> code=<in>,<tet>,<buf> vin=<adc_chain_vin 2 conversions after the take>
 * Steps: 0 = the first take after adc_chain_init (nothing converted: NONE x 3); 1..5 = takes 4 ms apart (every channel
 * converted); 6 = a take conv/4 after a take (the chain stopped before V_in converted: NONE x 3); 7 = a take 4 ms later
 * (full); 8 = a take 1.5 conv after a take (V_in converted, the rest not: in, NONE, NONE); 9 = a take 4 ms later (full).
 * Ends with "ADC end" and a sleep with interrupts off (simavr stops). */
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <stdint.h>
#include <stdio.h>

#include "hal/adc.h"

static int uart_putchar(char c, FILE *s)
{
    (void)s;
    while (!(UCSR0A & (1 << UDRE0))) {
    }
    UDR0 = (uint8_t)c;
    return 0;
}
static FILE uart_out = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

/* Timer1 at clk / 8: 2 counts per us */
static void wait_counts(uint16_t n)
{
    const uint16_t t0 = TCNT1;
    while ((uint16_t)(TCNT1 - t0) < n) {
    }
}

static uint16_t g_conv;                           /* one conversion, Timer1 counts */

static void take_and_report(uint8_t step)
{
    uint16_t c[3], vin;
    adc_chain_take(c);
    wait_counts((uint16_t)(2u * g_conv));
    vin = adc_chain_vin();
    printf("ADC t=%u code=%u,%u,%u vin=%u\n", step, c[0], c[1], c[2], vin);
}

int main(void)
{
    uint8_t s;
    uint16_t c[3], t0;
    stdout = &uart_out;
    TCCR1A = 0;
    TCCR1B = (uint8_t)(1 << CS11);
    adc_init();                                   /* as main.c: adc_init, the boot entropy conversions, the chain */
    (void)adc_entropy_seed(4u);
    while (ADCSRA & (1 << ADSC)) {                /* the simulator's conversion time (interrupts still off) */
    }
    t0 = TCNT1;
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC)) {
    }
    g_conv = (uint16_t)(TCNT1 - t0);
    printf("ADC conv_us=%u\n", (uint16_t)(g_conv / 2u));
    adc_chain_init();
    sei();
    take_and_report(0u);
    for (s = 1u; s <= 5u; s++) {
        wait_counts(7600u);
        take_and_report(s);
    }
    wait_counts(7600u);
    adc_chain_take(c);                            /* restart, then a take a quarter conversion later */
    wait_counts((uint16_t)(g_conv / 4u));
    take_and_report(6u);
    wait_counts(7600u);
    take_and_report(7u);
    wait_counts(7600u);
    adc_chain_take(c);                            /* restart, then a take 1.5 conversions later */
    wait_counts((uint16_t)(g_conv + g_conv / 2u));
    take_and_report(8u);
    wait_counts(7600u);
    take_and_report(9u);
    printf("ADC end\n");
    cli();
    sleep_enable();
    sleep_cpu();
    return 0;
}
