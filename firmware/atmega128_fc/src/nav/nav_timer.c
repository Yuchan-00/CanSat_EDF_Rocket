/* nav_timer.c - see nav.h: Timer1 time base of the NAV (the FCU's Timer1 is its PWM timer, started by hal/pwm.c,
 * which the NAV does not link).
 *
 * Mode 14 (fast PWM, TOP = ICR1) with every output-compare pin disconnected: the counter runs 0..FC_PWM_TOP and
 * wraps exactly like the FCU's PWM timer, so hal/timer.c's timer_tcnt1 / timer_elapsed_counts (and the simbench's
 * timer_sim_now) need no NAV variant.  Not CTC mode 12 (TOP = ICR1, the same count sequence on the chip): simavr's
 * ATmega128 Timer1 does not run in mode 12 (measured: TCNT1 stayed 0), and the simbench measures with this timer. */
#include "nav.h"

#include <avr/io.h>
#include <util/atomic.h>

#include "../params_gen.h"

void nav_timebase_init(void)
{
    TCCR1B = 0;                                    /* stopped while configuring */
    TCCR1A = (uint8_t)(1 << WGM11);                /* COM1A/B/C = 0: PB5..PB7 stay plain port pins */
    TCCR1C = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {            /* 16-bit registers: TEMP is shared with every ISR */
        ICR1 = FC_PWM_TOP;
        TCNT1 = 0;
    }
    TIMSK &= (uint8_t)~((1 << TICIE1) | (1 << OCIE1A) | (1 << OCIE1B) | (1 << TOIE1));   /* no Timer1 interrupt */
    ETIMSK &= (uint8_t)~(1 << OCIE1C);
    /* WGM13:0 = 1110; prescaler FC_PWM_CS_BITS -> FC_PWM_COUNTS_PER_US counts per us */
    TCCR1B = (uint8_t)((1 << WGM13) | (1 << WGM12) | (FC_PWM_CS_BITS & 0x07));
}
