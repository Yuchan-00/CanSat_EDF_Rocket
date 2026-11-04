/* pwm.h - six hardware PWM outputs on Timer1 (OC1A/B/C) and Timer3 (OC3A/B/C).
 *
 * Fast PWM mode 14 (TOP = ICRn = FC_PWM_TOP), prescaler FC_PWM_PRESCALER ->
 * FC_PWM_FRAME_HZ frames with 1/FC_PWM_COUNTS_PER_US us resolution.
 *
 * Invariant: pwm_init writes every compare register before the pins become
 * outputs, so the first pulse ever produced is the boot value (ESC off).  The
 * compare registers are double-buffered by the hardware (updated at BOTTOM),
 * so a write never produces a torn pulse; 16-bit writes are atomic.
 * pwm_us_to_counts is portable C (no AVR header) so the host test can check it
 * against the float expression it replaces.
 */
#ifndef FC_PWM_H
#define FC_PWM_H

#include <stdint.h>

#include "../params_gen.h"

#define PWM_N_CH 6

void     pwm_init(const uint16_t counts[PWM_N_CH]);
void     pwm_write(uint8_t ch, uint16_t counts);

#if FC_PWM_COUNTS_PER_US == 1
#define PWM_US_SHIFT 0u
#elif FC_PWM_COUNTS_PER_US == 2
#define PWM_US_SHIFT 1u
#elif FC_PWM_COUNTS_PER_US == 4
#define PWM_US_SHIFT 2u
#elif FC_PWM_COUNTS_PER_US == 8
#define PWM_US_SHIFT 3u
#else
#error "pwm_us_to_counts needs FC_PWM_COUNTS_PER_US = 2^j (j = 0..3)"
#endif
#if FC_PWM_TOP > 65535u
#error "FC_PWM_TOP must fit the 16-bit compare registers"
#endif

/* us -> compare counts, rounded half up and clamped to 0..FC_PWM_TOP: exactly the v0.5 float expression
 *     c = us * FC_PWM_COUNTS_PER_US + 0.5f;  !(c > 0) -> 0;  c >= FC_PWM_TOP -> FC_PWM_TOP;  else (uint16_t)c
 * computed on the float's bits.  For every us K >= 0.5 below the clamp that expression is exact (us * 2^j only moves
 * the exponent, and + 0.5 needs no rounding below 2^16), so c = M 2^(e - 150 + j) + 0.5 with M the 24-bit significand
 * and e the biased exponent, and (uint16_t)c = ((M >> t) + 1) >> 1 with t = 149 - j - e; below 0.5 the sum is < 1
 * except for the one input whose + 0.5f rounds up to 1.0.  A fraction of the CPU time of the float version, which runs
 * five times per control tick (the host test compares both for every float from 2^-27 to 2^34, a stride over all
 * 2^32 patterns and the edges). */
static inline uint16_t pwm_us_to_counts(float us)
{
    union {
        float f;
        uint32_t u;
        uint8_t b[4];
    } v;
    uint16_t e;
    uint32_t r;
    v.f = us;
#if !defined(__BYTE_ORDER__) || __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "pwm_us_to_counts reads the float's bytes little-endian (AVR, x86, ARM)"
#endif
    if (v.b[3] & 0x80u)
        return 0;                                        /* negative, -0, -inf, -NaN: c <= 0.5 (or NaN) */
    e = (uint16_t)((uint8_t)((uint8_t)(v.b[3] << 1) | (uint8_t)(v.b[2] >> 7)));
    if (e == 0xFFu)
        return ((v.b[2] & 0x7Fu) | v.b[1] | v.b[0]) ? 0u : (uint16_t)FC_PWM_TOP;   /* NaN -> 0, +inf -> TOP */
    if (e + PWM_US_SHIFT < 126u)                         /* us K < 0.5 (zero and denormals included): c < 1 -> 0, */
        return (e + PWM_US_SHIFT == 125u && (v.u & 0x007FFFFFUL) == 0x007FFFFFUL) ? 1u : 0u;
                                                         /* except us K = 0.5 - 2^-25: + 0.5f ties to even, c = 1.0 */
    if (e + PWM_US_SHIFT > 142u)
        return (uint16_t)FC_PWM_TOP;                     /* us K >= 2^16 > TOP */
    r = (((v.u & 0x007FFFFFUL) | 0x00800000UL) >> (uint8_t)(149u - PWM_US_SHIFT - e)) + 1u;
    r >>= 1;
    return r >= FC_PWM_TOP ? (uint16_t)FC_PWM_TOP : (uint16_t)r;
}

#endif /* FC_PWM_H */
