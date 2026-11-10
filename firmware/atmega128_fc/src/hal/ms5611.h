/* ms5611.h - MS5611 (GY-63) barometer as a non-blocking state machine.  Since 2026-11-10 a BUILD OPTION (env
 * nav_ms5611, -D FC_BARO_SENSOR=FC_BARO_MS5611): the default NAV build drives the BMP280 (hal/bmp280.h, hal/baro.h).
 *
 * One measurement cycle every FC_BARO_DIV control ticks (control.baro_hz); the phase
 * is tick % FC_BARO_DIV (kept as a counter by the caller), so the schedule is fixed
 * relative to the log schedule:
 *   phase 0                 script: read D2 (temperature) of the last cycle, start D1 OSR 4096
 *   phase MS5611_D1_PHASE   take D2; script: read D1, start D2 OSR 2048   (D1 waited >= 12 ms)
 *   phase MS5611_CALC_PHASE take D1
 *   background (same tick)  compensate (datasheet 1st + 2nd order, int64), then the ISA
 *                           altitude (powf) -> one new sample, used from the next tick on
 * The bus transfers are interrupt-driven scripts (twi_async_start) started right
 * after the IMU sample is taken, so they overlap the computation; the int64 and
 * powf work run in the loop's background time (ms5611_background), never in the
 * time-critical part.  The pressure is sampled during the D1 conversion; the sample
 * is MS5611_LATENCY_S old when it is published, which the altitude filter
 * compensates (AltitudeFilter latency).
 *
 * Invariant: the PROM is used only if its CRC4 (AN520) matches; a cycle with
 * an I2C error or an ADC result of 0 (read during conversion) publishes
 * nothing and counts a failure.
 */
#ifndef FC_MS5611_H
#define FC_MS5611_H

#include <stdbool.h>
#include <stdint.h>

#include "../params_gen.h"

#define MS5611_D1_CONV_MAX_US 9040UL     /* OSR 4096, datasheet max */
#define MS5611_D1_CONV_TYP_US 8220UL     /* OSR 4096, datasheet typ */
#define MS5611_D2_CONV_MAX_US 4540UL     /* OSR 2048, datasheet max */
#define MS5611_D1_PHASE ((MS5611_D1_CONV_MAX_US + FC_TICK_US - 1) / FC_TICK_US)   /* plain integers: used in #if */
#define MS5611_CALC_PHASE (MS5611_D1_PHASE + 1)
/* ticks from the start of D1 (phase 0) to the first tick that uses the sample */
#define MS5611_PUBLISH_TICKS (MS5611_CALC_PHASE + 1)
#define MS5611_D2_WAIT_TICKS (FC_BARO_DIV - MS5611_D1_PHASE)
#define MS5611_LATENCY_S ((float)(MS5611_PUBLISH_TICKS * FC_TICK_US - MS5611_D1_CONV_TYP_US / 2) * 1e-6f)

#if MS5611_D2_WAIT_TICKS * FC_TICK_US < MS5611_D2_CONV_MAX_US
#error "baro_hz too high for the MS5611 schedule (D2 conversion would be read early)"
#endif
#if MS5611_CALC_PHASE >= FC_BARO_DIV
#error "MS5611 schedule needs a free tick for the compensation"
#endif

typedef struct {
    uint16_t prom[8];
    bool     prom_ok;
    bool     have_p, need_comp;
    uint32_t d1, d2;
    bool     have_d2, d2_pending, d1_valid;
    int32_t  temp_c100, p_pa;
    float    altitude_m;          /* ISA altitude of the last sample */
    bool     new_sample;          /* set when a sample is published, cleared by the caller */
    uint16_t fail_count, consecutive_fail;
    uint32_t samples;
} ms5611_t;

bool ms5611_init(ms5611_t *b);    /* reset, PROM + CRC, first D2 (blocking, boot only) */
void ms5611_step(ms5611_t *b, uint8_t phase);    /* once per tick, phase = tick % FC_BARO_DIV */
bool ms5611_background(ms5611_t *b);              /* one math step if pending (compensation, then altitude) */

#endif /* FC_MS5611_H */
