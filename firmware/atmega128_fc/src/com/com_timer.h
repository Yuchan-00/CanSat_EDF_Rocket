/* com_timer.h - the COM's time base (v0.6, SPEC 10.4): the COM tick and a free-running
 * microsecond counter.  The FCU's hal/timer.c (Timer1 is its PWM timer there) is not used.
 *
 *   Timer2  CTC at COM_TICK_HZ (params flight_computer.com.tick_hz; the generator enforces
 *           COM_TICK_HZ == FC_LOOP_HZ, so FC_T2_PRESCALER / FC_T2_OCR give exactly that rate).
 *           The ISR only counts.  Tick counts are the time unit of the radio drivers'
 *           timeouts (LINK_AUX_WAIT_TICKS, SX127X_TX_TIMEOUT_TICKS), of the IL_STAT pacing
 *           (ILINK_STAT_DIV) and of the boot STAT's hold (com_link.h COM_STAT_BOOT_TICKS).
 *   Timer1  normal mode, clk/8: COM_T1_COUNTS_PER_US counts per us, wraps every 32.768 ms.
 *           Read only, for the main-loop pass time; no interrupt.
 *
 * Invariant: no ISR does work beyond counting, and every 16-bit timer read is atomic (the AVR
 * shares one TEMP register between the 16-bit timers).  An elapsed time is the uint16
 * difference of two reads, valid for intervals below 32.768 ms (a main-loop pass is far
 * shorter; the 60 ms watchdog bounds it anyway).
 */
#ifndef FC_COM_TIMER_H
#define FC_COM_TIMER_H

#include <stdbool.h>
#include <stdint.h>

#define COM_T1_PRESCALER 8u
#define COM_T1_COUNTS_PER_US 2u              /* F_CPU / 8 / 1 MHz (checked in com_timer.c) */

void     com_timer_init(void);
uint32_t com_timer_ticks(void);             /* COM ticks since boot */
uint16_t com_timer_tcnt1(void);             /* Timer1 counter, COM_T1_COUNTS_PER_US per us, free running */

static inline uint16_t com_timer_us_since(uint16_t t0, uint16_t t1)   /* (t1 - t0) in us, t1 - t0 < 32.768 ms */
{
    return (uint16_t)((uint16_t)(t1 - t0) / COM_T1_COUNTS_PER_US);
}

#endif /* FC_COM_TIMER_H */
