/* com_timer.c - see com_timer.h */
#include "com_timer.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>

#include "../params_gen.h"

#if COM_TICK_HZ != FC_LOOP_HZ
#error "the COM tick reuses FC_T2_*: flight_computer.com.tick_hz must equal control.loop_hz (the generator checks it)"
#endif
#if FC_T2_OCR > 255
#error "Timer2 is 8-bit"
#endif
#if FC_F_CPU_HZ / COM_T1_PRESCALER / 1000000UL != COM_T1_COUNTS_PER_US || FC_F_CPU_HZ % (COM_T1_PRESCALER * 1000000UL) != 0
#error "COM_T1_COUNTS_PER_US must equal F_CPU / 8 / 1 MHz exactly"
#endif

static volatile uint32_t s_ticks;

ISR(TIMER2_COMP_vect)
{
    s_ticks++;
}

void com_timer_init(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        s_ticks = 0;
        TCCR2 = 0;
        ASSR = 0;
        TCNT2 = 0;
        OCR2 = FC_T2_OCR;
        TCCR2 = (uint8_t)((1 << WGM21) | (FC_T2_CS_BITS & 0x07));   /* CTC, prescaler from params_gen.h */
        TCCR1A = 0;                                                  /* Timer1: normal mode, no output compare */
        TCCR1B = 0;
        TCNT1 = 0;
        TCCR1B = (uint8_t)(1 << CS11);                               /* clk/8 */
        TIFR = (uint8_t)(1 << OCF2);
        TIMSK |= (uint8_t)(1 << OCIE2);
    }
}

uint32_t com_timer_ticks(void)
{
    uint32_t t;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        t = s_ticks;
    }
    return t;
}

uint16_t com_timer_tcnt1(void)
{
    uint16_t t;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        t = TCNT1;
    }
    return t;
}
