/* baro.h - the NAV barometer: one interface, two compile-time drivers selected by FC_BARO_SENSOR (params_gen.h,
 * params flight_computer.nav.baro_sensor; a PlatformIO env may force the other with -D FC_BARO_SENSOR=...):
 *   FC_BARO_BMP280  hal/bmp280.c  Bosch BMP280, forced mode x4 / x1 (the default since 2026-11-10: the part on the
 *                                 carrier, module TYE-BMP29)
 *   FC_BARO_MS5611  hal/ms5611.c  MS5611 (GY-63), the barometer of v0.5 .. v0.9 (kept as a build option)
 *
 * Invariant: both drivers publish the same thing - altitude_m (ISA altitude of core/ms5611_math.h), new_sample,
 * samples, fail_count, consecutive_fail - once per FC_BARO_DIV ticks, BARO_LATENCY_S old at the first tick that
 * uses it; the NAV code uses only these and the three functions below, so the estimator sees no other difference
 * than the latency constant and the sensor's noise.
 */
#ifndef FC_BARO_H
#define FC_BARO_H

#include "../params_gen.h"

#if FC_BARO_SENSOR == FC_BARO_BMP280
#include "bmp280.h"
typedef bmp280_t baro_t;
#define baro_init(b) bmp280_init(b)
#define baro_step(b, phase) bmp280_step((b), (phase))
#define baro_background(b) bmp280_background(b)
#define BARO_LATENCY_S BMP280_LATENCY_S
#define BARO_NAME "BMP280"
#elif FC_BARO_SENSOR == FC_BARO_MS5611
#include "ms5611.h"
typedef ms5611_t baro_t;
#define baro_init(b) ms5611_init(b)
#define baro_step(b, phase) ms5611_step((b), (phase))
#define baro_background(b) ms5611_background(b)
#define BARO_LATENCY_S MS5611_LATENCY_S
#define BARO_NAME "MS5611"
#else
#error "FC_BARO_SENSOR must be FC_BARO_BMP280 or FC_BARO_MS5611"
#endif

#endif /* FC_BARO_H */
