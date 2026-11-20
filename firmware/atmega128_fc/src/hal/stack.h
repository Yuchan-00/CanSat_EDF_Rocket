/* stack.h - stack high-water mark: RAM above the static data is painted with a
 * canary in .init1 (before the stack exists); stack_free_min() counts the
 * canary bytes that were never overwritten.  The firmware logs it once per
 * second so the bench run proves the >= 1 KB stack margin on the hardware.
 * Invariant: the firmware uses no heap (no malloc), so everything between
 * _end and the stack pointer is stack reserve.
 */
#ifndef FC_STACK_H
#define FC_STACK_H

#include <stdint.h>

uint16_t stack_free_min(void);          /* full scan (~0.7 ms): boot / bench only */
uint16_t stack_free_min_step(void);     /* <= 64 bytes per call, returns the last complete scan (0xFFFF before the first) */
uint16_t stack_reserve_total(void);      /* RAMEND - _end + 1 */

#endif /* FC_STACK_H */
