/* timer.h - control-tick timers and the 0.5 us time base.
 *
 * NAV (and every role but the FCU): Timer2 CTC at FC_LOOP_HZ is the control tick (its ISR only counts), Timer0 has
 * the same period with its compare FC_IMU_LEAD_COUNTS earlier (the IMU lead: its ISR starts the MPU-6050 burst read so
 * the sample is in RAM when the tick fires).
 *
 * FCU (v0.6.3, -D FC_ROLE_FCU): the tick is started by the NAV's navigation frame (main.c, core/nlink_proto.h), so no
 * timer interrupt runs at all - a timer ISR firing during an nlink exchange would delay the SPI slave interrupt
 * (hal/nlink_spi.h Timing).  Timer2 still runs in CTC at FC_LOOP_HZ, its compare flag polled: it is the FCU's own
 * clock for the ticks it runs without a frame (timer_period_restart at every tick start, timer_period_due once
 * FC_TICK_US have passed).  Timer0 is stopped.
 *
 * Invariant: no timer ISR does work (the NAV's only count, the FCU has none); all work runs in the main loop.
 * timer_tcnt1() reads Timer1's counter (FC_PWM_COUNTS_PER_US counts per us, wraps every PWM frame, started by
 * pwm_init) atomically - every 16-bit timer access in the firmware is atomic because the AVR shares one TEMP register
 * between them and an ISR could otherwise corrupt a main-loop access.
 */
#ifndef FC_TIMER_H
#define FC_TIMER_H

#include <stdbool.h>
#include <stdint.h>

void     timer_init(void);
uint16_t timer_tcnt1(void);           /* Timer1 counter, FC_PWM_COUNTS_PER_US per us, 0..FC_PWM_TOP */
uint16_t timer_elapsed_counts(uint16_t from, uint16_t to);   /* modulo FC_PWM_TOP+1 */

#if defined(FC_ROLE_FCU)
void     timer_period_restart(void);  /* Timer2 from 0, compare flag cleared: due FC_TICK_US from now (-1 count) */
bool     timer_period_due(void);      /* FC_TICK_US have passed since the last restart (the flag stays set) */
#else
uint32_t timer_ticks(void);           /* control ticks since boot */
void     timer_wait_tick(void);       /* sleep until the next tick ISR */
bool     timer_tick_pending(void);    /* the next tick has fired (background work must stop) */
#if defined(FC_ROLE_GNS) || defined(FC_ROLE_NAV)
bool     timer_tick_take(void);       /* v0.8.1 GNS: the tick has fired since the last call (the flag is cleared);
                                       * NAV (2026-11-10): drops a tick that fired during the boot initialisation */
#endif
/* sleep until the next interrupt of any kind, unless the tick has fired or work() reports work - both are
 * checked with interrupts off, so an interrupt that makes either true cannot be slept through */
void     timer_idle(bool (*work)(void));
void     timer_set_lead_hook(void (*hook)(void));   /* called from the Timer0 lead ISR */
#ifdef FC_SIMBENCH
/* simbench: time since boot in Timer1 counts (FC_PWM_COUNTS_PER_US per us), from a base the tick ISR advances by one
 * tick and the Timer1 count it saw (the synthetic peer's clock); timer_sim_tick_start = that base (the current tick) */
uint32_t timer_sim_now(void);
uint32_t timer_sim_tick_start(void);
#endif
#endif

#endif /* FC_TIMER_H */
