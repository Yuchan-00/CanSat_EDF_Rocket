/* qfix_test (fw09/p1, adopted in the v0.9 firmware round as part of the regular test set, SPEC_firmware_v09 WP-X): the
 * AVR inline-asm fx_mulhi (core/qfix.h) against its C definition fx_mulhi_ref, on the AVR, in simavr, on 200000
 * operand pairs of every magnitude; also that each result is the exact floor of the 64-bit product or one less (the
 * three dropped low byte products); plus the cycles of one fx_mulhi and of avr-libc's fmul / fadd (Timer1 at clk / 1).
 * Output on USART0.  Run: run_qfix_test.py in this folder (tools/build_host_test.py --qfix, pytest test_qfix_avr.py):
 * avr-gcc -mmcu=atmega128 -Os -I <firmware src> qfix_test.c -o qfix_test.elf ; simavr -m atmega128 -f 16000000 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdint.h>
#include <stdio.h>

#include "core/qfix.h"

static int uart_putchar(char c, FILE *s)
{
    (void)s;
    while (!(UCSR0A & (1 << UDRE0))) {
    }
    UDR0 = (uint8_t)c;
    return 0;
}
static FILE uart_out = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

static uint32_t s_x = 2463534242ul;
static uint32_t xs(void)
{
    s_x ^= s_x << 13;
    s_x ^= s_x >> 17;
    s_x ^= s_x << 5;
    return s_x;
}

/* operands of every magnitude: full-range, Q30 quaternion-like, small rotation-vector-like, zero / extremes */
static int32_t operand(uint8_t kind)
{
    uint32_t r = xs();
    switch (kind & 7u) {
    case 0: return (int32_t)r;
    case 1: return (int32_t)(r >> 1) - 0x40000000L;               /* [-1, 1) in Q30 */
    case 2: return (int32_t)(r >> 8) - 0x00800000L;               /* small */
    case 3: return (int32_t)(r >> 20) - 0x800L;                   /* tiny */
    case 4: return (r & 1u) ? 0x7FFFFFFFL : (int32_t)0x80000000UL; /* extremes */
    case 5: return (r & 1u) ? 0 : -1;
    case 6: return (int32_t)(r | 0x80000000UL);                   /* negative */
    default: return (int32_t)(r & 0x7FFFFFFFUL);                  /* positive */
    }
}

volatile int32_t g_sink;
volatile float g_fa = 1.2345f, g_fb = -0.98765f, g_fs;

int main(void)
{
    uint32_t n, bad = 0, bad_exact = 0;
    uint16_t t0, t1, c_mul, c_fmul, c_fadd, c_ref;
    int32_t a, b;
    UBRR0H = 0;
    UBRR0L = 8;
    UCSR0B = (uint8_t)(1 << TXEN0);
    UCSR0C = (uint8_t)((1 << UCSZ01) | (1 << UCSZ00));
    stdout = &uart_out;
    TCCR1A = 0;
    TCCR1B = (uint8_t)(1 << CS10);
    for (n = 0; n < 200000ul; n++) {
        int64_t p;
        int32_t r, e;
        a = operand((uint8_t)xs());
        b = operand((uint8_t)xs());
        r = fx_mulhi(a, b);
        if (r != fx_mulhi_ref(a, b))
            bad++;
        p = (int64_t)a * (int64_t)b;
        e = (int32_t)(p >> 32);                                      /* exact floor */
        if (!(r == e || r == e - 1))
            bad_exact++;
    }
    a = 0x2345678L;
    b = -0x1234567L;
    t0 = TCNT1;
    g_sink = fx_mulhi(a, b);
    t1 = TCNT1;
    c_mul = (uint16_t)(t1 - t0);
    t0 = TCNT1;
    g_sink = fx_mulhi_ref(a, b);
    t1 = TCNT1;
    c_ref = (uint16_t)(t1 - t0);
    t0 = TCNT1;
    g_fs = g_fa * g_fb;
    t1 = TCNT1;
    c_fmul = (uint16_t)(t1 - t0);
    t0 = TCNT1;
    g_fs = g_fa + g_fb;
    t1 = TCNT1;
    c_fadd = (uint16_t)(t1 - t0);
    printf("QFIX pairs=%lu asm_vs_ref_mismatch=%lu outside_floor_or_floor_minus_1=%lu\n", (unsigned long)n,
           (unsigned long)bad, (unsigned long)bad_exact);
    printf("QFIX cycles (incl. operand loads/stores and the TCNT1 reads): fx_mulhi %u, fx_mulhi_ref %u, fmul %u, fadd %u\n",
           c_mul, c_ref, c_fmul, c_fadd);
    for (n = 0; n < 20000ul; n++) {
    }
    cli();
    sleep_enable();
    sleep_cpu();
    return 0;
}
