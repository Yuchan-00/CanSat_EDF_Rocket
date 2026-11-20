/* bmp280.h - Bosch BMP280 barometer (module TYE-BMP29, I2C 0x77) as a non-blocking state machine: the default NAV
 * barometer since 2026-11-10 (params flight_computer.nav.baro_sensor = bmp280, FC_BARO_SENSOR; hal/baro.h selects
 * this driver or hal/ms5611.h).
 *
 * Mode (datasheet BST-BMP280-DS001, section numbers; the PDF is not in the repository -> ASSUMED until checked):
 *   forced mode (3.6.2): one measurement per write of ctrl_meas, then sleep.  Chosen over normal mode because the
 *   sample instant is then fixed relative to the NAV tick (normal mode free-runs on the sensor's own +-25 % clock,
 *   so the age of a sample would wander over a whole standby period), and because every cycle re-writes the
 *   oversampling, so a sensor that reset in flight is back after one cycle.
 *   osrs_p x4 (18 bit, 0.66 Pa steps), osrs_t x1 (3.3.1 / 3.3.2; "standard resolution" of 3.4 without the filter):
 *   measurement time 11.5 ms typ / 13.3 ms max (3.8.1) fits the 20 ms cycle of control.baro_hz = 50 Hz; x8
 *   (19.5 / 22.5 ms) does not.  IIR filter off (3.3.3): it would add lag the altitude filter does not model.
 *
 * One measurement cycle every FC_BARO_DIV control ticks; the phase is tick % FC_BARO_DIV (kept by the caller):
 *   phase BMP280_START_PHASE script A: write ctrl_meas (forced, x4 / x1) -> the sensor measures T, then P
 *   phase BMP280_READ_PHASE  script B: burst read 0xF7..0xFC (3.9: one burst = one consistent sample), started
 *                            BMP280_MEAS_TICKS ticks >= BMP280_MEAS_MAX_US after the forced write
 *   background               1. take the raw values, t_fine and the pressure multiplications (int64)
 *                            2. the pressure division (int64)
 *                            3. after the next BMP280_START_PHASE: the ISA altitude (powf) -> one new sample
 * The sample is published in the background of the next cycle's start-phase tick whatever tick finished the math,
 * so it is used from the tick after it on: BMP280_LATENCY_S old (the middle of the pressure sampling to the first
 * use), a constant the altitude filter compensates.  The start phase is 0, so the sample is used in phase 1 (the
 * MS5611's are used in phase 0).  Both placements were measured on 2026-11-10 with the NAV bench
 * (tools/run_nav_simbench.py) against the unchanged limits:
 *   start phase 0 (this one): every check passes; the altitude correction shares the phase-1 tick with the glink
 *     RESULT decode, so the VTOL mandatory work per tick is 2981 us (74.5 % of the tick, limit 80 %; MS5611 2782 us)
 *     and the ballistic NAV's frame-ready jitter is 403 us (limit 600 us; MS5611 274 us);
 *   start phase FC_BARO_DIV - 1 (use in phase 0): those two figures stay at the MS5611's, but the VTOL pre-push
 *     compute ends 11 us later (994 us), which is outside the measured push floor (nav.h NAV_PUSH_MIN_US_VTOL) -
 *     it would need the floor and the FCU bench's NAV push time re-derived, so it is not used.
 * The bus transfers are interrupt-driven scripts (twi_async_start)
 * started right after the IMU sample is taken; the int64 and powf work runs in the loop's background time, one
 * bounded step per call (nav_main.c NAV_BG_RESERVE_US), never in the time-critical part.
 *
 * Invariant: the trimming parameters are used only if they are plausible (bmp280_calib_ok); a cycle with an I2C
 * error, the "never measured" data value 0x80000, an ADC value at a rail, a zero divisor or a pressure outside the
 * sensor's range (3.1: 300 .. 1100 hPa) publishes nothing and counts a failure; a sample that was not published
 * by the next read is dropped (never published late).
 */
#ifndef FC_BMP280_H
#define FC_BMP280_H

#include <stdbool.h>
#include <stdint.h>

#include "../core/bmp280_math.h"
#include "../params_gen.h"

#define BMP280_MEAS_MAX_US 13300UL     /* forced, osrs_p x4 + osrs_t x1: datasheet 3.8.1 max */
#define BMP280_MEAS_TYP_US 11500UL     /* typ: 1 ms start + 2 ms temperature + (4 x 2 + 0.5) ms pressure */
#define BMP280_P_MID_TYP_US 7250UL     /* middle of the pressure sampling after the forced write: 3 ms + 8.5 ms / 2 */
#define BMP280_MEAS_TICKS ((BMP280_MEAS_MAX_US + FC_TICK_US - 1) / FC_TICK_US)   /* plain integers: used in #if */
#define BMP280_START_PHASE 0
#define BMP280_READ_PHASE ((BMP280_START_PHASE + BMP280_MEAS_TICKS) % FC_BARO_DIV)
/* ticks from the forced write to the first tick that uses the sample (the tick after the next start phase) */
#define BMP280_PUBLISH_TICKS (FC_BARO_DIV + 1)
#define BMP280_LATENCY_S ((float)(BMP280_PUBLISH_TICKS * FC_TICK_US - BMP280_P_MID_TYP_US) * 1e-6f)

#if BMP280_MEAS_TICKS >= FC_BARO_DIV
#error "baro_hz too high for the BMP280 schedule (forced x4 / x1 measurement would be read early or not at all)"
#endif

#define BMP280_P_MIN_Q8 (30000UL * 256UL)      /* 300 hPa, datasheet 3.1 operating range */
#define BMP280_P_MAX_Q8 (110000UL * 256UL)     /* 1100 hPa */

enum {
    BMP280_ST_IDLE = 0,
    BMP280_ST_READING,           /* script B running / finished, raw values not taken yet */
    BMP280_ST_RAW,               /* adc_p, adc_t valid: step 1 pending */
    BMP280_ST_DIV,               /* t_fine, work valid: step 2 pending */
    BMP280_ST_HAVE_P             /* p_q8 valid: step 3 (publish) pending, waits for gate_open */
};

typedef struct {
    bmp280_calib_t cal;
    bool     cal_ok;
    uint8_t  chip_id;
    uint8_t  stage;               /* BMP280_ST_* */
    bool     gate_open;           /* the start phase after the read was reached: step 3 may publish */
    uint32_t adc_p, adc_t;
    int32_t  t_fine, temp_c100;
    bmp280_pwork_t work;
    uint32_t p_q8;                /* pressure [Pa] in Q24.8 */
    float    altitude_m;          /* ISA altitude of the last sample */
    bool     new_sample;          /* set when a sample is published, cleared by the caller */
    uint16_t fail_count, consecutive_fail;
    uint32_t samples;
} bmp280_t;

bool bmp280_init(bmp280_t *b);    /* chip id, reset, trimming parameters, one forced measurement (blocking, boot only) */
void bmp280_step(bmp280_t *b, uint8_t phase);    /* once per tick, phase = tick % FC_BARO_DIV */
bool bmp280_background(bmp280_t *b);              /* one math step if pending; false = nothing to do now */

#endif /* FC_BMP280_H */
