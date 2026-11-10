/* ms5611.c - see ms5611.h (compiled only for FC_BARO_SENSOR == FC_BARO_MS5611, hal/baro.h).  Commands: MS5611-01BA03
 * datasheet. */
#include "ms5611.h"

#if FC_BARO_SENSOR == FC_BARO_MS5611

#include <avr/wdt.h>
#include <util/delay.h>

#include "../core/ms5611_math.h"
#include "twi.h"

#define CMD_RESET 0x1E
#define CMD_ADC_READ 0x00
#define CMD_PROM_READ 0xA0
#define CMD_CONV_D1_OSR4096 0x48
#define CMD_CONV_D2_OSR2048 0x56

static bool cmd(uint8_t c)
{
    return twi_write(CAL_MS5611_ADDR, &c, 1);
}

static bool adc_read(uint32_t *v)
{
    uint8_t b[3];
    if (!cmd(CMD_ADC_READ) || !twi_read(CAL_MS5611_ADDR, b, 3))   /* command with STOP, then read */
        return false;
    *v = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
    return *v != 0;                       /* 0 = read during a conversion / no conversion started */
}

bool ms5611_init(ms5611_t *b)
{
    uint8_t i, w[2];
    b->prom_ok = false;
    b->have_p = false;
    b->need_comp = false;
    b->have_d2 = b->d2_pending = b->d1_valid = b->new_sample = false;
    b->fail_count = b->consecutive_fail = 0;
    b->samples = 0;
    b->altitude_m = 0.0f;
    if (!cmd(CMD_RESET))
        return false;
    for (i = 0; i < 4; i++) {             /* 2.8 ms reload */
        _delay_ms(1);
        wdt_reset();
    }
    for (i = 0; i < 8; i++) {
        if (!cmd((uint8_t)(CMD_PROM_READ + 2 * i)) || !twi_read(CAL_MS5611_ADDR, w, 2))
            return false;
        b->prom[i] = (uint16_t)(((uint16_t)w[0] << 8) | w[1]);
    }
    b->prom_ok = ms5611_prom_ok(b->prom);
    if (!b->prom_ok)
        return false;
    if (!cmd(CMD_CONV_D2_OSR2048))
        return false;
    for (i = 0; i < 6; i++) {
        _delay_ms(1);
        wdt_reset();
    }
    b->have_d2 = adc_read(&b->d2);
    return b->have_d2;
}

static void failed(ms5611_t *b)
{
    b->fail_count++;
    if (b->consecutive_fail < 0xFFFF)
        b->consecutive_fail++;
}

/* bus scripts (static: they must outlive the transfer) */
static uint8_t s_cmd_adc = CMD_ADC_READ, s_cmd_d1 = CMD_CONV_D1_OSR4096, s_cmd_d2 = CMD_CONV_D2_OSR2048;
static uint8_t s_raw_a[3], s_raw_b[3];
static volatile uint8_t s_stat_a, s_stat_b;
static bool s_a_reads_d2, s_d1_started;
static const twi_op_t k_read_d2_start_d1[3] = {
    {CAL_MS5611_ADDR, 0, 1, &s_cmd_adc},
    {CAL_MS5611_ADDR, TWI_OP_READ, 3, s_raw_a},
    {CAL_MS5611_ADDR, 0, 1, &s_cmd_d1},
};
static const twi_op_t k_read_d1_start_d2[3] = {
    {CAL_MS5611_ADDR, 0, 1, &s_cmd_adc},
    {CAL_MS5611_ADDR, TWI_OP_READ, 3, s_raw_b},
    {CAL_MS5611_ADDR, 0, 1, &s_cmd_d2},
};

static uint32_t be24(const uint8_t *r)
{
    return ((uint32_t)r[0] << 16) | ((uint32_t)r[1] << 8) | r[2];
}

void ms5611_step(ms5611_t *b, uint8_t phase)
{
    if (!b->prom_ok)
        return;
    if (phase == 0) {                                /* [read D2 of the last cycle], start D1 */
        s_a_reads_d2 = b->d2_pending;
        b->d2_pending = false;
        b->d1_valid = false;
        s_d1_started = s_a_reads_d2 ? twi_async_start(k_read_d2_start_d1, 3, &s_stat_a)
                                    : twi_async_start(&k_read_d2_start_d1[2], 1, &s_stat_a);
        if (!s_d1_started)
            failed(b);
    } else if (phase == MS5611_D1_PHASE) {           /* script A finished long ago: take D2, read D1, start D2 */
        if (s_d1_started && s_stat_a == TWI_ASYNC_DONE) {
            if (s_a_reads_d2) {
                uint32_t d2 = be24(s_raw_a);
                if (d2 != 0) {
                    b->d2 = d2;
                    b->have_d2 = true;
                } else {
                    failed(b);
                }
            }
            b->d2_pending = twi_async_start(k_read_d1_start_d2, 3, &s_stat_b);
            if (!b->d2_pending)
                failed(b);
        } else {
            failed(b);
        }
    } else if (phase == MS5611_CALC_PHASE) {         /* script B done: D1; the math runs in the background */
        b->have_p = false;
        b->need_comp = false;
        if (b->d2_pending && s_stat_b == TWI_ASYNC_DONE) {
            b->d1 = be24(s_raw_b);
            b->d1_valid = b->d1 != 0;
        } else {
            b->d2_pending = false;
        }
        if (!b->d1_valid)
            failed(b);
        b->need_comp = b->d1_valid && b->have_d2;
    }
}

bool ms5611_background(ms5611_t *b)
{
    if (b->need_comp) {                              /* step 1: int64 compensation */
        ms5611_compensate(b->prom, b->d1, b->d2, &b->temp_c100, &b->p_pa);
        b->need_comp = false;
        b->have_p = true;
        return true;
    }
    if (b->have_p) {                                 /* step 2: ISA altitude -> publish */
        b->altitude_m = ms5611_pressure_altitude((float)b->p_pa, FC_G0);
        b->new_sample = true;
        b->consecutive_fail = 0;
        b->samples++;
        b->have_p = false;
        return true;
    }
    return false;
}

#endif /* FC_BARO_SENSOR == FC_BARO_MS5611 */
