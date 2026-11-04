/* ms5611_math.h - MS5611-01BA03 PROM CRC4 (TE AN520) and first/second order
 * temperature compensation (datasheet "Pressure and temperature calculation"),
 * plus the ISA pressure -> altitude conversion.  Portable part of the driver.
 *
 * Invariant: integer arithmetic exactly as the datasheet (int64 for OFF/SENS),
 * so the datasheet example (C1..C6 = 40127, 36924, 23317, 23282, 33464, 28312;
 * D1 = 9085466, D2 = 8569150) gives TEMP = 2007 (20.07 C), P = 100009 Pa.
 */
#ifndef FC_MS5611_MATH_H
#define FC_MS5611_MATH_H

#include <stdbool.h>
#include <stdint.h>

/* ISA troposphere: T0 = 288.15 K, lapse 0.0065 K/m, R = 287.05287 J/(kg K), P0 = 101325 Pa */
#define ISA_T0_K        288.15f
#define ISA_LAPSE_K_M   0.0065f
#define ISA_R_J_KGK     287.05287f
#define ISA_P0_PA       101325.0f

uint8_t ms5611_crc4(const uint16_t prom[8]);          /* 4-bit CRC over C0..C7 (C7 low nibble zeroed) */
bool    ms5611_prom_ok(const uint16_t prom[8]);        /* crc4 == prom[7] & 0xF, and not all 0 / all 1 */
/* D1, D2 raw 24-bit ADC results -> TEMP [0.01 C], P [Pa]; second order below 20 C */
void    ms5611_compensate(const uint16_t prom[8], uint32_t d1, uint32_t d2, int32_t *temp_c100, int32_t *p_pa);
/* ISA altitude above the 101325 Pa level [m] (only differences are used) */
float   ms5611_pressure_altitude(float p_pa, float g0);

#endif /* FC_MS5611_MATH_H */
