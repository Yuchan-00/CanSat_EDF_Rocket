/* bmp280.c - see bmp280.h (compiled only for FC_BARO_SENSOR == FC_BARO_BMP280).  Registers: BMP280 datasheet
 * BST-BMP280-DS001 chapter 4.3 (memory map), written from the published datasheet (ASSUMED until checked). */
#include "bmp280.h"

#if FC_BARO_SENSOR == FC_BARO_BMP280

#include <avr/wdt.h>
#include <util/delay.h>

#include "../core/ms5611_math.h"        /* the ISA pressure -> altitude conversion (shared by both barometers) */
#include "twi.h"

#define REG_CALIB 0x88                  /* 4.3.2 trimming parameters 0x88 .. 0x9F */
#define REG_ID 0xD0                     /* 4.3.1 */
#define REG_RESET 0xE0                  /* 4.3.2 */
#define REG_STATUS 0xF3                 /* 4.3.3: bit 3 measuring, bit 0 im_update */
#define REG_CTRL_MEAS 0xF4              /* 4.3.4: osrs_t[7:5] osrs_p[4:2] mode[1:0] */
#define REG_CONFIG 0xF5                 /* 4.3.5: t_sb[7:5] filter[4:2] spi3w_en[0] */
#define REG_DATA 0xF7                   /* 4.3.6 press_msb .. 4.3.7 temp_xlsb (0xFC) */
#define RESET_CMD 0xB6
#define STATUS_IM_UPDATE 0x01
/* osrs_t x1 (001), osrs_p x4 (011), forced mode (01) */
#define CTRL_MEAS_FORCED ((uint8_t)((1u << 5) | (3u << 2) | 1u))
#define CONFIG_FILTER_OFF 0x00          /* t_sb unused in forced mode, IIR off, 4-wire */

/* 4.3.1: 0x58 = BMP280 (0x56 / 0x57 = its engineering samples); 0x60 = BME280, whose temperature / pressure path,
 * registers and trimming are the same (its humidity oversampling is 0 = skipped after reset) - sold on the same
 * modules.  Anything else is not this sensor. */
static bool id_ok(uint8_t id)
{
    return id == 0x58u || id == 0x57u || id == 0x56u || id == 0x60u;
}

static void wait_ms(uint8_t ms)
{
    while (ms--) {
        _delay_ms(1);
        wdt_reset();
    }
}

static bool raw_ok(uint32_t adc_p, uint32_t adc_t)
{
    if (adc_p == BMP280_ADC_SKIPPED && adc_t == BMP280_ADC_SKIPPED)   /* reset value: nothing was measured */
        return false;
    if (adc_p == 0u || adc_p >= 0xFFFFFUL || adc_t == 0u || adc_t >= 0xFFFFFUL)   /* a rail / a dead bus */
        return false;
    return true;
}

bool bmp280_init(bmp280_t *b)
{
    uint8_t raw[BMP280_CALIB_LEN], st = 0xFF;
    b->cal_ok = false;
    b->chip_id = 0;
    b->stage = BMP280_ST_IDLE;
    b->gate_open = false;
    b->new_sample = false;
    b->fail_count = b->consecutive_fail = 0;
    b->samples = 0;
    b->altitude_m = 0.0f;
    if (!twi_read_reg(CAL_BMP280_ADDR, REG_ID, &b->chip_id, 1) || !id_ok(b->chip_id))
        return false;
    if (!twi_write_reg(CAL_BMP280_ADDR, REG_RESET, RESET_CMD))
        return false;
    wait_ms(4);                                       /* start-up time 2 ms max (3.1), NVM copy */
    if (!twi_read_reg(CAL_BMP280_ADDR, REG_STATUS, &st, 1) || (st & STATUS_IM_UPDATE))
        return false;
    if (!twi_read_reg(CAL_BMP280_ADDR, REG_CALIB, raw, BMP280_CALIB_LEN))
        return false;
    bmp280_calib_parse(raw, &b->cal);
    if (!bmp280_calib_ok(&b->cal))
        return false;
    if (!twi_write_reg(CAL_BMP280_ADDR, REG_CONFIG, CONFIG_FILTER_OFF) ||
        !twi_write_reg(CAL_BMP280_ADDR, REG_CTRL_MEAS, CTRL_MEAS_FORCED))
        return false;
    wait_ms((uint8_t)((BMP280_MEAS_MAX_US + 1999UL) / 1000UL));
    if (!twi_read_reg(CAL_BMP280_ADDR, REG_DATA, raw, 6))
        return false;
    if (!raw_ok(bmp280_raw20(raw), bmp280_raw20(raw + 3)))
        return false;
    b->cal_ok = true;
    return true;
}

static void failed(bmp280_t *b)
{
    b->fail_count++;
    if (b->consecutive_fail < 0xFFFF)
        b->consecutive_fail++;
    b->stage = BMP280_ST_IDLE;
}

/* bus scripts (static: they must outlive the transfer) */
static uint8_t s_forced[2] = {REG_CTRL_MEAS, CTRL_MEAS_FORCED};
static uint8_t s_reg_data = REG_DATA;
static uint8_t s_raw[6];
static volatile uint8_t s_stat_a, s_stat_b;
static bool s_meas_started;
static const twi_op_t k_start_forced[1] = {
    {CAL_BMP280_ADDR, 0, 2, s_forced},
};
static const twi_op_t k_read_data[2] = {
    {CAL_BMP280_ADDR, TWI_OP_RESTART, 1, &s_reg_data},
    {CAL_BMP280_ADDR, TWI_OP_READ, 6, s_raw},
};

void bmp280_step(bmp280_t *b, uint8_t phase)
{
    if (!b->cal_ok)
        return;
    if (phase == BMP280_START_PHASE) {                /* start the next measurement; the last one may publish */
        b->gate_open = true;
        s_meas_started = twi_async_start(k_start_forced, 1, &s_stat_a);
        if (!s_meas_started) {
            b->fail_count++;                          /* the pending sample of the last cycle stays valid */
            if (b->consecutive_fail < 0xFFFF)
                b->consecutive_fail++;
        }
    } else if (phase == BMP280_READ_PHASE) {          /* script A finished long ago, the measurement is over */
        if (b->stage != BMP280_ST_IDLE)
            failed(b);                                /* the last sample was never published: dropped, not late */
        b->gate_open = false;
        if (s_meas_started && s_stat_a == TWI_ASYNC_DONE && twi_async_start(k_read_data, 2, &s_stat_b))
            b->stage = BMP280_ST_READING;
        else
            failed(b);
        s_meas_started = false;
    }
}

bool bmp280_background(bmp280_t *b)
{
    switch (b->stage) {
    case BMP280_ST_READING:                           /* step 1: raw values, then as BMP280_ST_RAW */
        if (s_stat_b == TWI_ASYNC_BUSY)
            return false;
        if (s_stat_b != TWI_ASYNC_DONE) {
            failed(b);
            return false;
        }
        b->adc_p = bmp280_raw20(s_raw);
        b->adc_t = bmp280_raw20(s_raw + 3);
        if (!raw_ok(b->adc_p, b->adc_t)) {
            failed(b);
            return false;
        }
        /* fall through */
    case BMP280_ST_RAW:                               /* step 1: t_fine and the int64 multiplications */
        b->t_fine = bmp280_t_fine(&b->cal, b->adc_t);
        b->temp_c100 = bmp280_temp_c100(b->t_fine);
        if (!bmp280_p_part1(&b->cal, b->t_fine, &b->work)) {
            failed(b);
            return false;
        }
        b->stage = BMP280_ST_DIV;
        return true;
    case BMP280_ST_DIV:                               /* step 2: the int64 division */
        b->p_q8 = bmp280_p_part2(&b->cal, b->adc_p, &b->work);
        if (b->p_q8 < BMP280_P_MIN_Q8 || b->p_q8 > BMP280_P_MAX_Q8) {
            failed(b);
            return false;
        }
        b->stage = BMP280_ST_HAVE_P;
        return true;
    case BMP280_ST_HAVE_P:                            /* step 3: ISA altitude -> publish (from the start phase on) */
        if (!b->gate_open)
            return false;
        b->altitude_m = ms5611_pressure_altitude((float)b->p_q8 * (1.0f / 256.0f), FC_G0);
        b->new_sample = true;
        b->consecutive_fail = 0;
        b->samples++;
        b->stage = BMP280_ST_IDLE;
        return true;
    default:
        return false;
    }
}

#endif /* FC_BARO_SENSOR == FC_BARO_BMP280 */
