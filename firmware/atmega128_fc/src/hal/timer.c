/* timer.c - see timer.h */
#include "timer.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <util/atomic.h>

#include "../params_gen.h"

#if FC_T2_OCR > 255
#error "Timer2 is 8-bit"
#endif

#if defined(FC_ROLE_FCU)
/* ---------------------------------------------------------------- FCU: no timer interrupt, Timer2 polled */
void timer_init(void)
{
    TCCR2 = 0;
    TCCR0 = 0;                                      /* Timer0 (the NAV's IMU lead timer) stays off on the FCU */
    TIMSK &= (uint8_t)~((1 << OCIE2) | (1 << TOIE2) | (1 << OCIE0) | (1 << TOIE0));
    OCR2 = FC_T2_OCR;
    TCNT2 = 0;
    TCCR2 = (uint8_t)((1 << WGM21) | (FC_T2_CS_BITS & 0x07));   /* CTC, prescaler from params_gen.h */
    TIFR = (uint8_t)((1 << OCF2) | (1 << TOV2) | (1 << OCF0) | (1 << TOV0));
    set_sleep_mode(SLEEP_MODE_IDLE);
}

/* the prescaler (shared with Timer1 / Timer3, the PWM) is not reset: the first count comes up to one prescaler period
 * (16 us) early, so the flag sets FC_TICK_US - 16 us .. FC_TICK_US after the restart */
void timer_period_restart(void)
{
    TCNT2 = 0;                                      /* also blocks a compare match in the next timer clock */
    TIFR = (uint8_t)(1 << OCF2);                    /* write one to clear */
}

bool timer_period_due(void)
{
    return (TIFR & (uint8_t)(1 << OCF2)) != 0u;
}

#else
/* ---------------------------------------------------------------- NAV and others: Timer2 tick + Timer0 IMU lead */
#if FC_IMU_LEAD_COUNTS >= FC_T2_OCR
#error "IMU lead must be shorter than the control period"
#endif

static volatile uint32_t s_ticks;
static volatile uint8_t s_tick_flag;
static void (*volatile s_lead_hook)(void);
#ifdef FC_SIMBENCH
static volatile uint16_t s_tick_tcnt1;
static volatile uint32_t s_tick_base;
#endif

ISR(TIMER2_COMP_vect)
{
    s_ticks++;
    s_tick_flag = 1;
#ifdef FC_SIMBENCH
    s_tick_tcnt1 = TCNT1;                           /* interrupts are off here: the 16-bit read is atomic */
    s_tick_base += (uint32_t)FC_TICK_US * FC_PWM_COUNTS_PER_US;
#endif
}

ISR(TIMER0_COMP_vect)
{
    void (*h)(void) = s_lead_hook;
    if (h)
        h();
}

void timer_init(void)
{
    TCCR2 = 0;
    TCCR0 = 0;
    ASSR = 0;                                       /* Timer0 clocked synchronously from the I/O clock */
    OCR2 = FC_T2_OCR;
    OCR0 = FC_T2_OCR;
    /* hold both prescalers in reset, preload the phase, then release them together:
     * Timer0 reaches its compare FC_IMU_LEAD_COUNTS counts before Timer2, forever (same clock, same TOP) */
    SFIOR |= (uint8_t)((1 << TSM) | (1 << PSR0) | (1 << PSR321));
    TCNT2 = 0;
    TCNT0 = FC_IMU_LEAD_COUNTS;
    TCCR2 = (uint8_t)((1 << WGM21) | (FC_T2_CS_BITS & 0x07));   /* CTC, prescaler from params_gen.h */
    TCCR0 = (uint8_t)((1 << WGM01) | (FC_T0_CS_BITS & 0x07));
    TIFR = (uint8_t)((1 << OCF2) | (1 << OCF0));
    TIMSK |= (uint8_t)((1 << OCIE2) | (1 << OCIE0));
    SFIOR &= (uint8_t)~(1 << TSM);
    set_sleep_mode(SLEEP_MODE_IDLE);
}

void timer_set_lead_hook(void (*hook)(void))
{
    s_lead_hook = hook;
}

bool timer_tick_pending(void)
{
    return s_tick_flag != 0;
}

#if defined(FC_ROLE_GNS) || defined(FC_ROLE_NAV)
bool timer_tick_take(void)
{
    bool f;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        f = s_tick_flag != 0;
        s_tick_flag = 0;
    }
    return f;
}
#endif

uint32_t timer_ticks(void)
{
    uint32_t t;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { t = s_ticks; }
    return t;
}

void timer_wait_tick(void)
{
    for (;;) {
        cli();
        if (s_tick_flag) {
            s_tick_flag = 0;
            sei();
            return;
        }
        sleep_enable();
        sei();              /* the instruction after sei executes before any interrupt: no lost wake-up */
        sleep_cpu();
        sleep_disable();
    }
}

void timer_idle(bool (*work)(void))
{
    cli();
    if (s_tick_flag || (work && work())) {
        sei();
        return;
    }
    sleep_enable();
    sei();                  /* as in timer_wait_tick: an interrupt pending since the check wakes the sleep at once */
    sleep_cpu();
    sleep_disable();
}

#ifdef FC_SIMBENCH
uint32_t timer_sim_now(void)
{
    uint32_t base;
    uint16_t c0, c;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        base = s_tick_base;
        c0 = s_tick_tcnt1;
        c = TCNT1;
    }
    return base + timer_elapsed_counts(c0, c);      /* a tick ISR still pending shows as > one tick: still right */
}

uint32_t timer_sim_tick_start(void)
{
    uint32_t base;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { base = s_tick_base; }
    return base;
}
#endif
#endif /* FC_ROLE_FCU */

uint16_t timer_tcnt1(void)
{
    uint16_t t;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { t = TCNT1; }
    return t;
}

uint16_t timer_elapsed_counts(uint16_t from, uint16_t to)
{
    return (uint16_t)(to >= from ? to - from : (uint16_t)(FC_PWM_TOP + 1u - from + to));
}
