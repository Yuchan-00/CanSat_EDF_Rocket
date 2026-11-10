/* lbat_avr (2026-11-10 control round): the logic-battery path of src/hal/adc.c (FC_LOGIC_BAT: the VTOL battery FCU on the
 * carrier rev 2c - adc_read_blocking at boot, then adc_poll_mux with the main.c schedule: one PF3 conversion every
 * LBAT_TEST_DIV ticks in place of the V_cell one) run on the AVR, in simavr, with distinct voltages on VBAT_ADC_CHANNEL
 * and LOGIC_BAT_ADC_CHANNEL (adc_chain_sim.c drives them).  Every bench / co-simulation build replaces the FCU's ADC by
 * synthetic codes, so this test is where the flight path of the logic-battery reading runs in simulation.
 * Built with the env fcu's -D flags by run_adc_chain_test.py; output on USART0:
 *   LB boot code=<adc_read_blocking(LOGIC_BAT_ADC_CHANNEL)> q=<lbat byte, main.c lbat_q_of>
 *   LB t=<tick> ch=<channel of the result> code=<code> q=<lbat byte when ch is PF3, else 0>   one line per poll result
 *   LB end
 * The lbat byte uses main.c's formula ((counts x LOGIC_BAT_Q_X1024 + 512) >> 10, 1..255; sim/link_proto.py
 * lbat_q_from_counts).  Ticks are 4 ms (Timer1). */
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <stdint.h>
#include <stdio.h>

#include "hal/adc.h"
#include "params_gen.h"

#if !FC_LOGIC_BAT
#error "lbat_avr is built with the -D flags of a logic-battery FCU env (fcu): FC_LOGIC_BAT must be 1"
#endif

#define LBAT_TEST_DIV 3u                          /* the flight build: LOGIC_BAT_DIV_TICKS (250); 3 keeps the log short */

static int uart_putchar(char c, FILE *s)
{
    (void)s;
    while (!(UCSR0A & (1 << UDRE0))) {
    }
    UDR0 = (uint8_t)c;
    return 0;
}
static FILE uart_out = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

static void wait_counts(uint16_t n)
{
    const uint16_t t0 = TCNT1;
    while ((uint16_t)(TCNT1 - t0) < n) {
    }
}

static uint8_t lbat_q_of(uint16_t counts)            /* = main.c */
{
    const uint32_t q = ((uint32_t)counts * LOGIC_BAT_Q_X1024 + 512u) >> 10;
    return q < 1u ? 1u : (q > 255u ? 255u : (uint8_t)q);
}

int main(void)
{
    uint16_t code, div = 0;
    uint8_t ch, k;
    stdout = &uart_out;
    TCCR1A = 0;
    TCCR1B = (uint8_t)(1 << CS11);                /* clk / 8: 2 counts per us */
    adc_init();                                   /* as main.c: adc_init, the boot entropy conversions, the boot read */
    (void)adc_entropy_seed(4u);
    code = adc_read_blocking(LOGIC_BAT_ADC_CHANNEL);
    printf("LB boot code=%u q=%u\n", code, lbat_q_of(code));
    for (k = 0; k < 12u; k++) {
        uint8_t next;
        wait_counts(8000u);                       /* one 4 ms tick */
        if (div < LBAT_TEST_DIV)
            div++;
        next = div >= LBAT_TEST_DIV ? (uint8_t)LOGIC_BAT_ADC_CHANNEL : (uint8_t)VBAT_ADC_CHANNEL;
        if (!adc_poll_mux(&code, &ch, next)) {
            printf("LB t=%u busy\n", k);
            continue;
        }
        if (next == (uint8_t)LOGIC_BAT_ADC_CHANNEL)
            div = 0;
        printf("LB t=%u ch=%u code=%u q=%u\n", k, ch, code, ch == (uint8_t)LOGIC_BAT_ADC_CHANNEL ? lbat_q_of(code) : 0u);
    }
    printf("LB end\n");
    cli();
    sleep_enable();
    sleep_cpu();
    return 0;
}
