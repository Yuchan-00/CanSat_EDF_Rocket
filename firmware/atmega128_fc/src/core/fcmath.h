/* fcmath.h - scalar helpers shared by the portable core (no AVR includes).
 *
 * Invariant: single-precision only.  On avr-gcc `float` and `double` are both
 * IEEE single; on the host the core is compiled with -ffp-contract=off so the
 * same expression order gives the same rounding as on the target.
 */
#ifndef FC_FCMATH_H
#define FC_FCMATH_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define FC_PI_F       3.14159265358979323846f
#define FC_RAD2DEG_F  57.2957795130823208768f
#define FC_DEG2RAD_F  0.01745329251994329577f

static inline float fc_clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

static inline float fc_minf(float a, float b) { return a < b ? a : b; }
static inline float fc_maxf(float a, float b) { return a > b ? a : b; }

#if defined(__AVR__)
/* v0.8.1 RAM diet: parameter tables kept in flash (the progmem attribute, core/fc_config.c) are read with LPM - inline
 * asm, no AVR header; the tables sit in the lower 64 KB like avr-libc's pgm_read_* assume (.progmem follows the vectors).
 * FC_FLASH_RD(lvalue) reads one field of a flash table through its own type (_Generic; the comma drops the const of the
 * field so every qualified form matches; a type not listed is a compile error, never a wrong-width read). */
static inline __attribute__((always_inline)) uint8_t fc_lpm8(const void *a)
{
    uint8_t r;
    __asm__("lpm %0, Z" : "=r"(r) : "z"(a));
    return r;
}

static inline __attribute__((always_inline)) uint16_t fc_lpm16(const void *a)
{
    uint16_t r;
    __asm__("lpm %A0, Z+\n\tlpm %B0, Z" : "=r"(r), "+z"(a));
    return r;
}

static inline __attribute__((always_inline)) uint32_t fc_lpm32(const void *a)
{
    uint32_t r;
    __asm__("lpm %A0, Z+\n\tlpm %B0, Z+\n\tlpm %C0, Z+\n\tlpm %D0, Z" : "=r"(r), "+z"(a));
    return r;
}

static inline __attribute__((always_inline)) float fc_lpm_f32(const void *a)
{
    union {
        uint32_t u;
        float f;
    } v;
    v.u = fc_lpm32(a);
    return v.f;
}

static inline __attribute__((always_inline)) int32_t fc_lpm_i32(const void *a) { return (int32_t)fc_lpm32(a); }
static inline __attribute__((always_inline)) int16_t fc_lpm_i16(const void *a) { return (int16_t)fc_lpm16(a); }
static inline __attribute__((always_inline)) int8_t fc_lpm_i8(const void *a) { return (int8_t)fc_lpm8(a); }
static inline __attribute__((always_inline)) bool fc_lpm_b(const void *a) { return fc_lpm8(a) != 0u; }

#define FC_FLASH_RD(lv)                                                                                            \
    _Generic(((void)0, (lv)), float: fc_lpm_f32, uint32_t: fc_lpm32, int32_t: fc_lpm_i32, uint16_t: fc_lpm16,       \
             int16_t: fc_lpm_i16, uint8_t: fc_lpm8, int8_t: fc_lpm_i8, bool: fc_lpm_b)(&(lv))
#endif

#endif /* FC_FCMATH_H */
