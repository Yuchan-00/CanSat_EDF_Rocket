/* qfix.h - signed 32-bit fixed-point helpers of the VTOL NAV estimator (v0.9 timing proposal P1, fw09/p1 prototype).
 *
 * Why: the NAV's soft-float time was 2143 us of every 4000 us tick (timing82 profile); the two quaternion integrations
 * alone 2 x 586 us.  Integer adds cost 4 cycles instead of avr-libc's ~110-cycle fadd, and a 32 x 32 multiply from the
 * chip's 8 x 8 MUL ~80 cycles instead of the ~128-cycle fmul.  The FORMULATION is the reference's (sim/quat.py
 * integrate: q (x) exp(omega dt) by the same series, renormalised by the same Newton step every tick); only the number
 * format of the intermediate values changes: Q30 / Q32 (resolution 9.3e-10 / 2.3e-10) instead of float32 (2^-24 relative,
 * 6e-8 for values near 1).  tools/crosscheck_controller.py (section 7b, MAHONY_Q_TOL 1e-4) bounds the result against the
 * float64 reference.
 *
 * fx_mulhi(a, b) = floor(a b / 2^32) of the signed 64-bit product, from the 13 byte products of weight >= 2 (the three
 * of weight 0 and 1 are dropped: the result is the exact floor or one less, the latter with a probability < 2^-7).  The
 * AVR version is inline asm (77 cycles + operand moves); every other compiler runs fx_mulhi_ref, the same arithmetic in
 * C (bit-identical: the host test and the crosscheck run it; fw09/p1/tools/qfix_test checks the two on simavr).
 * Formats: Qn = value x 2^n in an int32_t.  Q30 x Q32 -> fx_mulhi -> Q30; Q30 x Q30 -> Q28; Q32 x Q32 -> Q32.
 * Portable C99 (no AVR header: core/ rule of tools/build_host_test.py).
 */
#ifndef FC_QFIX_H
#define FC_QFIX_H

#include <stdint.h>

/* the AVR asm's arithmetic, in C (unsigned byte products of weight >= 2, then the two's-complement corrections) */
static inline int32_t fx_mulhi_ref(int32_t a, int32_t b)
{
    const uint32_t ua = (uint32_t)a, ub = (uint32_t)b;
    uint64_t p = 0;
    uint8_t i, j;
    uint32_t hi;
    for (i = 0; i < 4u; i++)
        for (j = 0; j < 4u; j++)
            if (i + j >= 2u)
                p += (uint64_t)(((ua >> (8u * i)) & 0xFFu) * ((ub >> (8u * j)) & 0xFFu)) << (8u * (i + j));
    hi = (uint32_t)(p >> 32);
    if (a < 0)
        hi -= ub;
    if (b < 0)
        hi -= ua;
    return (int32_t)hi;
}

#if defined(__AVR__)
static inline __attribute__((always_inline)) int32_t fx_mulhi(int32_t a, int32_t b)
{
    int32_t r;
    uint16_t t;
    uint8_t z;
    __asm__(
        "clr  %[z]"              "\n\t"
        "mul  %A[a], %C[b]"      "\n\t"   /* weight 2: a0 b2 -> T2:T3 */
        "movw %A[t], r0"         "\n\t"
        "clr  %A[r]"             "\n\t"
        "clr  %B[r]"             "\n\t"
        "clr  %C[r]"             "\n\t"
        "clr  %D[r]"             "\n\t"
        "mul  %B[a], %B[b]"      "\n\t"   /* a1 b1 */
        "add  %A[t], r0"         "\n\t"
        "adc  %B[t], r1"         "\n\t"
        "adc  %A[r], %[z]"       "\n\t"
        "mul  %C[a], %A[b]"      "\n\t"   /* a2 b0 */
        "add  %A[t], r0"         "\n\t"
        "adc  %B[t], r1"         "\n\t"
        "adc  %A[r], %[z]"       "\n\t"
        "mul  %A[a], %D[b]"      "\n\t"   /* weight 3: a0 b3 -> T3:R4, carry R5 */
        "add  %B[t], r0"         "\n\t"
        "adc  %A[r], r1"         "\n\t"
        "adc  %B[r], %[z]"       "\n\t"
        "mul  %B[a], %C[b]"      "\n\t"   /* a1 b2 */
        "add  %B[t], r0"         "\n\t"
        "adc  %A[r], r1"         "\n\t"
        "adc  %B[r], %[z]"       "\n\t"
        "mul  %C[a], %B[b]"      "\n\t"   /* a2 b1 */
        "add  %B[t], r0"         "\n\t"
        "adc  %A[r], r1"         "\n\t"
        "adc  %B[r], %[z]"       "\n\t"
        "mul  %D[a], %A[b]"      "\n\t"   /* a3 b0 */
        "add  %B[t], r0"         "\n\t"
        "adc  %A[r], r1"         "\n\t"
        "adc  %B[r], %[z]"       "\n\t"
        "mul  %B[a], %D[b]"      "\n\t"   /* weight 4: a1 b3 -> R4:R5, carry R6 */
        "add  %A[r], r0"         "\n\t"
        "adc  %B[r], r1"         "\n\t"
        "adc  %C[r], %[z]"       "\n\t"
        "mul  %C[a], %C[b]"      "\n\t"   /* a2 b2 */
        "add  %A[r], r0"         "\n\t"
        "adc  %B[r], r1"         "\n\t"
        "adc  %C[r], %[z]"       "\n\t"
        "mul  %D[a], %B[b]"      "\n\t"   /* a3 b1 */
        "add  %A[r], r0"         "\n\t"
        "adc  %B[r], r1"         "\n\t"
        "adc  %C[r], %[z]"       "\n\t"
        "mul  %C[a], %D[b]"      "\n\t"   /* weight 5: a2 b3 -> R5:R6, carry R7 */
        "add  %B[r], r0"         "\n\t"
        "adc  %C[r], r1"         "\n\t"
        "adc  %D[r], %[z]"       "\n\t"
        "mul  %D[a], %C[b]"      "\n\t"   /* a3 b2 */
        "add  %B[r], r0"         "\n\t"
        "adc  %C[r], r1"         "\n\t"
        "adc  %D[r], %[z]"       "\n\t"
        "mul  %D[a], %D[b]"      "\n\t"   /* weight 6: a3 b3 -> R6:R7 */
        "add  %C[r], r0"         "\n\t"
        "adc  %D[r], r1"         "\n\t"
        "sbrs %D[a], 7"          "\n\t"   /* a < 0: hi -= b */
        "rjmp 1f"                "\n\t"
        "sub  %A[r], %A[b]"      "\n\t"
        "sbc  %B[r], %B[b]"      "\n\t"
        "sbc  %C[r], %C[b]"      "\n\t"
        "sbc  %D[r], %D[b]"      "\n\t"
        "1:"                     "\n\t"
        "sbrs %D[b], 7"          "\n\t"   /* b < 0: hi -= a */
        "rjmp 2f"                "\n\t"
        "sub  %A[r], %A[a]"      "\n\t"
        "sbc  %B[r], %B[a]"      "\n\t"
        "sbc  %C[r], %C[a]"      "\n\t"
        "sbc  %D[r], %D[a]"      "\n\t"
        "2:"                     "\n\t"
        "clr  __zero_reg__"
        : [r] "=&r"(r), [t] "=&r"(t), [z] "=&r"(z)
        : [a] "r"(a), [b] "r"(b));
    return r;
}
#else
#define fx_mulhi fx_mulhi_ref
#endif

typedef union {
    float f;
    uint32_t u;
    uint16_t w[2];
    uint8_t b[4];                                        /* little-endian (AVR, x86): b[3] = sign + exponent high bits */
} fx_f32_t;

/* the biased exponent from the bytes (avr-gcc -Os turns a 32-bit `>> 23` into a 23-step loop) */
static inline uint8_t fx_exp_of(const fx_f32_t *v)
{
    return (uint8_t)((uint8_t)(v->b[3] << 1) | (uint8_t)(v->b[2] >> 7));
}

/* float -> Qq (truncated toward zero; 0 for |x| < 2^-q, zero and denormals).  The caller guarantees |x| 2^q < 2^31 */
static inline int32_t fx_from_f(float x, uint8_t q)
{
    fx_f32_t v;
    uint32_t m;
    uint8_t ex;
    int8_t sh;
    v.f = x;
    ex = fx_exp_of(&v);
    if (ex == 0u)
        return 0;
    m = ((uint32_t)(uint8_t)(v.b[2] | 0x80u) << 16) | v.w[0];   /* |x| = m 2^(ex - 150) */
    sh = (int8_t)((int16_t)ex - (int16_t)(150 - q));     /* |x| 2^q = m 2^sh */
    if (sh >= 0) {
        m <<= (uint8_t)sh;
    } else {
        uint8_t r = (uint8_t)(-sh);
        if (r >= 24u) {
            m = 0;
        } else {
            if (r >= 16u) {                              /* whole bytes first: byte moves, not bit loops */
                m >>= 16;
                r = (uint8_t)(r - 16u);
            }
            if (r >= 8u) {
                m >>= 8;
                r = (uint8_t)(r - 8u);
            }
            m >>= r;
        }
    }
    return (v.b[3] & 0x80u) ? -(int32_t)m : (int32_t)m;
}

/* Qq -> float, correctly rounded (int32 -> float, then the exponent - q: exact) */
static inline float fx_to_f(int32_t x, uint8_t q)
{
    fx_f32_t v;
    v.f = (float)x;
    if (v.u & 0x7F800000ul)                              /* x != 0: |x| >= 1, exponent >= 127 > q */
        v.u -= (uint32_t)q << 23;
    return v.f;
}

/* the biased exponent of a float (0 for zero / denormals): |x| < 2^(e - 126) */
static inline uint8_t fx_f_exp(float x)
{
    fx_f32_t v;
    v.f = x;
    return fx_exp_of(&v);
}

#endif /* FC_QFIX_H */
