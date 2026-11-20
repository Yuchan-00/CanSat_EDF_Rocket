/* isr_cost_avr.c - ATmega128 probe firmware for tools/cosim/isr_cost.c: how many cycles simavr charges per interrupt.
 *
 * A busy loop of a fixed length runs between two PORTA marks (PORTA = 1 / PORTA = 0); built with -D WITH_ISR, a
 * Timer0 compare interrupt fires every 100 cycles meanwhile, its ISR a bare RETI (ISR_NAKED).  The difference of the
 * two windows divided by the interrupts taken is what simavr charges per interrupt; on the chip it is the 4-cycle
 * interrupt response + the 3-cycle JMP in the vector table + the 4-cycle RETI = 11 cycles (ATmega128 datasheet,
 * "Interrupt Response Time").  tools/run_cosim.py builds both variants with avr-gcc and corrects the co-simulation's
 * cycle counts by the difference per interrupt.
 */
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>

ISR(TIMER0_COMP_vect, ISR_NAKED)
{
    __asm__ __volatile__("reti");
}

int main(void)
{
    DDRA = 0xFF;
    PORTA = 0;
#ifdef WITH_ISR
    OCR0 = 99;                                   /* CTC, no prescaler: an interrupt every 100 cycles */
    TCCR0 = (uint8_t)((1 << WGM01) | (1 << CS00));
    TIMSK |= (uint8_t)(1 << OCIE0);
#endif
    sei();
    PORTA = 1;
    __asm__ __volatile__(
        "ldi r24, lo8(20000)\n\t"
        "ldi r25, hi8(20000)\n\t"
        "1: sbiw r24, 1\n\t"                     /* 2 cycles */
        "brne 1b\n\t"                            /* 2 taken, 1 not */
        ::: "r24", "r25");
    PORTA = 0;
    cli();
    sleep_enable();
    sleep_cpu();                                 /* simavr stops: sleep with interrupts off */
    return 0;
}
