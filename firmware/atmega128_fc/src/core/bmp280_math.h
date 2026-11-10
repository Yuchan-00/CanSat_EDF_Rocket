/* bmp280_math.h - Bosch BMP280 trimming parameters and the integer compensation of the datasheet
 * (BST-BMP280-DS001: 3.11.2 trimming parameter readout, 3.11.3 compensation formula - the 32-bit temperature and
 * the 64-bit pressure code of that section; 3.12 calculation example).  Portable part of the driver (hal/bmp280.c);
 * the ISA pressure -> altitude conversion is the one of core/ms5611_math.h (same constants for both barometers).
 *
 * Invariant: integer arithmetic exactly as the datasheet code (int32 for T, int64 for P, arithmetic right shifts -
 * avr-gcc and the host compilers shift signed values arithmetically), so the datasheet example
 * (dig_T1..T3 = 27504, 26435, -1000; dig_P1..P9 = 36477, -10685, 3024, 2855, 140, -7, 15500, -14600, 6000;
 * adc_T = 519888, adc_P = 415148) gives t_fine = 128422, T = 2508 (25.08 C) and P = 25767233 / 256 = 100653.25 Pa
 * (the datasheet table prints 100653.27 Pa for its floating-point code; the integer T code drops the low adc_T
 * bits, which are 0 at the x1 temperature oversampling the driver uses).
 * The pressure code is split in two steps (the multiplications, then the division) because the NAV runs it in its
 * background time one bounded step at a time (nav_main.c NAV_BG_RESERVE_US); bmp280_compensate is both in a row.
 * A result is refused, never guessed: trims that are all 0 / all 1, or a zero divisor (datasheet: "avoid exception
 * caused by division by zero"), return false.
 *
 * The datasheet is not in the repository: the register map and the formulas are written from the published
 * datasheet (ASSUMED until checked against the PDF / the part); the host test (host_test/test_core.c test_bmp280)
 * checks the integer code against the datasheet's floating-point formula (appendix 8.1) and the 3.12 example.
 */
#ifndef FC_BMP280_MATH_H
#define FC_BMP280_MATH_H

#include <stdbool.h>
#include <stdint.h>

#define BMP280_CALIB_LEN 24u           /* registers 0x88 .. 0x9F: dig_T1..T3, dig_P1..P9, little endian */
#define BMP280_ADC_SKIPPED 0x80000UL   /* data register value when a measurement was skipped / never made */

typedef struct {
    uint16_t t1;
    int16_t  t2, t3;
    uint16_t p1;
    int16_t  p2, p3, p4, p5, p6, p7, p8, p9;
} bmp280_calib_t;

typedef struct {
    int64_t var1, var2;                /* the divisor and the offset term of the pressure formula */
} bmp280_pwork_t;

void    bmp280_calib_parse(const uint8_t raw[BMP280_CALIB_LEN], bmp280_calib_t *c);
bool    bmp280_calib_ok(const bmp280_calib_t *c);          /* not erased / stuck: T1, P1 != 0 and not all 0xFFFF */
/* raw 20-bit data registers (msb, lsb, xlsb): 0xF7.. pressure, 0xFA.. temperature */
uint32_t bmp280_raw20(const uint8_t r[3]);
int32_t bmp280_t_fine(const bmp280_calib_t *c, uint32_t adc_t);
int32_t bmp280_temp_c100(int32_t t_fine);                  /* [0.01 C] */
/* step 1: var1, var2 from t_fine (64-bit multiplications); false if the divisor is 0 */
bool    bmp280_p_part1(const bmp280_calib_t *c, int32_t t_fine, bmp280_pwork_t *w);
/* step 2: the 64-bit division and the P7..P9 terms -> pressure [Pa] in Q24.8 */
uint32_t bmp280_p_part2(const bmp280_calib_t *c, uint32_t adc_p, const bmp280_pwork_t *w);
/* both steps; false (nothing written) if the divisor is 0 */
bool    bmp280_compensate(const bmp280_calib_t *c, uint32_t adc_p, uint32_t adc_t, int32_t *temp_c100, uint32_t *p_q8);

#endif /* FC_BMP280_MATH_H */
