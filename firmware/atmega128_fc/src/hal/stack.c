/* stack.c - see stack.h */
#include "stack.h"

#include <avr/io.h>

#define STACK_CANARY 0xC5

extern uint8_t _end;
extern uint8_t __stack;

void stack_paint(void) __attribute__((naked, used, section(".init1")));
void stack_paint(void)
{
    /* r1 is not yet zero and there is no stack: registers only */
    __asm__ volatile(
        "ldi r30, lo8(_end)\n\t"
        "ldi r31, hi8(_end)\n\t"
        "ldi r24, 0xC5\n\t"
        "ldi r25, hi8(__stack)\n\t"
        "rjmp 2f\n"
        "1:\n\t"
        "st Z+, r24\n"
        "2:\n\t"
        "cpi r30, lo8(__stack)\n\t"
        "cpc r31, r25\n\t"
        "brlo 1b\n\t"
        "breq 1b\n\t" ::);
}

uint16_t stack_free_min(void)
{
    const volatile uint8_t *p = &_end;
    uint16_t n = 0;
    while (p <= &__stack && *p == STACK_CANARY) {
        p++;
        n++;
    }
    return n;
}

/* resumable scan: at most STACK_SCAN_STEP bytes per call, result of the last full scan */
#define STACK_SCAN_STEP 64u
uint16_t stack_free_min_step(void)
{
    static const volatile uint8_t *p;
    static uint16_t n, result = 0xFFFFu;
    uint8_t k;
    if (!p) {
        p = &_end;
        n = 0;
    }
    for (k = 0; k < STACK_SCAN_STEP; k++) {
        if (p > &__stack || *p != STACK_CANARY) {      /* first used byte: scan complete */
            result = n;
            p = 0;
            break;
        }
        p++;
        n++;
    }
    return result;
}

uint16_t stack_reserve_total(void)
{
    return (uint16_t)(&__stack - &_end + 1);
}
