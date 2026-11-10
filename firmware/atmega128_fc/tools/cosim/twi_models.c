/* twi_models.c - the NAV's I2C sensors in the four-core co-simulation (cosim4.c), from timing82/profile/harness/
 * navflight.c's register models, their values from the closed-loop plant (plant4.c) instead of a fixed hover:
 *   MPU-6050 at 0x68: register file (WHO_AM_I 0x75 = 0x68, the configuration writes kept); a read that starts at the
 *     burst address 0x3B latches the plant's body specific force and rates into 0x3B..0x48 at the scale the NAV
 *     configured (ACCEL_CONFIG / GYRO_CONFIG), temperature fixed;
 *   MS5611 at 0x77: reset, PROM (the datasheet's example C1..C6, a valid CRC4), D1 / D2 conversions with the datasheet's
 *     maximum conversion times (OSR 4096: 9.04 ms, 2048: 4.54 ms, ... ) - an ADC read before the conversion is done
 *     returns 0, as the chip does (counted) - and the ADC read; D2 is the example's 8569150 (20.07 C, no second-order
 *     term), D1 the inverse of the firmware's own first-order compensation (core/ms5611_math.c) for the plant's static
 *     pressure, so the NAV's integer Pascal is the plant's pressure;
 *   BMP280 at 0x77 instead of the MS5611 (twi_models_set_baro(TWI_BARO_BMP280); the harness's baro=bmp280, the NAV
 *     image's FC_BARO_SENSOR): the register file of datasheet BST-BMP280-DS001 chapter 4.3 (written from the published
 *     datasheet; the PDF is not in the repository -> ASSUMED until checked) - id 0xD0 = 0x58, reset 0xE0 = 0xB6
 *     (im_update for the 2 ms start-up time), the trimming parameters 0x88..0x9F of the datasheet's calculation example
 *     (3.12), status 0xF3, ctrl_meas 0xF4, config 0xF5, data 0xF7..0xFC.  A write of forced mode to ctrl_meas starts
 *     one measurement of the datasheet's MAXIMUM measurement time for the oversampling written (3.8.1:
 *     1.25 + 2.3 osrs_t + 2.3 osrs_p + 0.575 ms; x1 / x4: 13.3 ms); the pressure is the plant's at the typical middle
 *     of the pressure sampling (1 + 2 osrs_t + (2 osrs_p + 0.5) / 2 ms after the write; x1 / x4: 7.25 ms - the instant
 *     the firmware's BMP280_LATENCY_S counts from); the data registers change when the measurement ends, so a read
 *     before that returns the PREVIOUS sample (after reset the "skipped" value 0x80000), as the chip's shadowing does
 *     (counted: bmp_early).  adc_T is the example's 519888 (25.08 C); adc_P is the inverse of the datasheet's
 *     double-precision compensation (appendix 8.1 - not the firmware's int64 code, so an error there shows as an
 *     altitude error) for the plant's pressure, rounded to the resolution of the oversampling (3.3.1: 16 + (osrs_p - 1)
 *     bits with the IIR filter off, 20 bits with it on; x4 = 18 bits = 4 LSB of the 20-bit value, about 0.66 Pa).
 *     Not modelled: normal mode (a write of mode 11 measures nothing; counted), the IIR filter's dynamics, noise (the
 *     plant adds its own to the pressure), temperature changes.
 * simavr 1.8's TWI master schedules every state change N microseconds after the TWCR write for an N-bit step (a byte
 * 9 us whatever TWBR says).  With fix_byte_time the harness looks up that cycle timer right after each TWCR write and
 * moves it to N SCL periods of the clock the NAV programmed (SCL = F_CPU / (16 + 2 TWBR 4^TWPS): 400 kHz = 40 cycles per
 * bit, 22.5 us per byte) - the bus time the chip has.  The delays found are histogrammed (stats.rescale_hist, in us).
 *
 * Invariant: register models and bus timing only; no firmware RAM is read or written.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "avr_twi.h"
#include "sim_cycle_timers.h"
#include "sim_io.h"
#include "sim_irq.h"

#include "cosim4.h"

#define D_TWBR 0x70                                  /* ATmega128 data-space addresses */
#define D_TWSR 0x71
#define D_TWCR 0x74

static avr_t *g_nav;
static avr_twi_t *g_twi;
static avr_irq_t *g_twi_in;
static int g_fix;
static twi_stats_t g_st;
static avr_cycle_count_t g_twi_when;
int g_twi_debug_n;                                   /* twidbg=N: print the first N delays found (stderr) */
#define g_debug g_twi_debug_n                 /* the state timer the harness set last (absolute cycle) */

static double g_gyro[3], g_accel[3], g_p_pa = 100009.0;
static uint8_t g_mpu_reg[128], g_mpu_ptr;
static int g_mpu_first_write;
static uint16_t g_prom[8];
static uint32_t g_adc;
static int g_conv;                                   /* 0 none, 1 D1, 2 D2 */
static avr_cycle_count_t g_conv_done;                /* NAV cycle the conversion completes */
static uint8_t g_ms_out[3];
static int g_ms_out_n, g_ms_out_i;
static uint8_t g_sel;
static const uint32_t k_d2 = 8569150u;

/* BMP280 */
#define BMP_SKIPPED 0x80000UL                        /* reset value of the data registers: nothing measured */
#define BMP_ADC_T 519888UL                           /* the datasheet example's raw temperature (25.08 C) */
static int g_baro_kind = TWI_BARO_MS5611;
static uint8_t g_bmp_calib[26];                      /* 0x88 .. 0xA1 */
static uint8_t g_bmp_ctrl, g_bmp_config, g_bmp_ptr;
static uint8_t g_bmp_data[6];                        /* 0xF7 .. 0xFC as the bus reads them */
static uint32_t g_bmp_adc_p = BMP_SKIPPED, g_bmp_adc_t = BMP_SKIPPED;   /* the running measurement's result */
static int g_bmp_measuring, g_bmp_sampled, g_bmp_expect_reg;
static avr_cycle_count_t g_bmp_sample_at, g_bmp_done_at, g_bmp_im_until;
static struct {
    double t1, t2, t3, p1, p2, p3, p4, p5, p6, p7, p8, p9;
} g_bc;

static unsigned crc4(uint16_t n_prom[8])             /* MS5611 AN520 */
{
    unsigned n_rem = 0, cnt, n_bit;
    uint16_t crc_read = n_prom[7];
    n_prom[7] = (uint16_t)(0xFF00u & n_prom[7]);
    for (cnt = 0; cnt < 16; cnt++) {
        if (cnt % 2 == 1)
            n_rem ^= (unsigned)(n_prom[cnt >> 1] & 0x00FFu);
        else
            n_rem ^= (unsigned)(n_prom[cnt >> 1] >> 8);
        for (n_bit = 8; n_bit > 0; n_bit--) {
            if (n_rem & 0x8000u)
                n_rem = (n_rem << 1) ^ 0x3000u;
            else
                n_rem = (n_rem << 1);
        }
    }
    n_rem = (0x000Fu & (n_rem >> 12));
    n_prom[7] = crc_read;
    return n_rem;
}

static void be16put(uint8_t *p, long v)
{
    if (v > 32767) v = 32767;
    if (v < -32768) v = -32768;
    p[0] = (uint8_t)((unsigned long)v >> 8);
    p[1] = (uint8_t)v;
}

/* D1 for a pressure: the inverse of core/ms5611_math.c ms5611_compensate (first order: TEMP 2007 >= 2000) */
static uint32_t d1_for(double p_pa)
{
    const int64_t dT = (int64_t)k_d2 - ((int64_t)g_prom[5] << 8);
    const int64_t off = ((int64_t)g_prom[2] << 16) + ((int64_t)g_prom[4] * dT) / 128;
    const int64_t sens = ((int64_t)g_prom[1] << 15) + ((int64_t)g_prom[3] * dT) / 256;
    double d1 = ((p_pa + 0.5) * 32768.0 + (double)off) * 2097152.0 / (double)sens;
    if (d1 < 0.0) d1 = 0.0;
    if (d1 > 16777215.0) d1 = 16777215.0;
    return (uint32_t)d1;
}

/* BMP280 datasheet appendix 8.1 (double precision): pressure [Pa] of a raw pair with the model's trimming values */
static double bmp_p_double(uint32_t adc_p, uint32_t adc_t)
{
    double var1 = ((double)adc_t / 16384.0 - g_bc.t1 / 1024.0) * g_bc.t2;
    double var2 = (double)adc_t / 131072.0 - g_bc.t1 / 8192.0;
    double p;
    int32_t t_fine;
    var2 = var2 * var2 * g_bc.t3;
    t_fine = (int32_t)(var1 + var2);
    var1 = (double)t_fine / 2.0 - 64000.0;
    var2 = var1 * var1 * g_bc.p6 / 32768.0;
    var2 = var2 + var1 * g_bc.p5 * 2.0;
    var2 = var2 / 4.0 + g_bc.p4 * 65536.0;
    var1 = (g_bc.p3 * var1 * var1 / 524288.0 + g_bc.p2 * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * g_bc.p1;
    p = 1048576.0 - (double)adc_p;
    p = (p - var2 / 4096.0) * 6250.0 / var1;
    var1 = g_bc.p9 * p * p / 2147483648.0;
    var2 = p * g_bc.p8 / 32768.0;
    return p + (var1 + var2 + g_bc.p7) / 16.0;
}

/* adc_P for a pressure: the 20-bit value whose appendix-8.1 pressure is nearest (the pressure falls as adc_P rises),
 * rounded to the oversampling's resolution */
static uint32_t bmp_adc_p_for(double p_pa)
{
    const unsigned osrs_p = (g_bmp_ctrl >> 2) & 7u;
    const unsigned bits = ((g_bmp_config >> 2) & 7u) ? 20u : 16u + ((osrs_p > 5u ? 5u : osrs_p) - 1u);
    const uint32_t step = 1UL << (20u - bits);
    uint32_t lo = 1u, hi = 0xFFFFEUL, a;
    while (lo < hi) {                                /* the smallest adc_P with p(adc_P) <= p_pa */
        const uint32_t mid = lo + (hi - lo) / 2u;
        if (bmp_p_double(mid, BMP_ADC_T) <= p_pa)
            hi = mid;
        else
            lo = mid + 1u;
    }
    a = (lo + step / 2u) / step * step;              /* the ADC's resolution: the lower bits are 0 */
    if (a < step) a = step;
    if (a > 0xFFFFFUL - step) a = (0xFFFFFUL - step) / step * step;
    return a;
}

static void bmp_put20(uint8_t *d, uint32_t v)        /* msb, lsb, xlsb[7:4] */
{
    d[0] = (uint8_t)(v >> 12);
    d[1] = (uint8_t)(v >> 4);
    d[2] = (uint8_t)((v & 0x0Fu) << 4);
}

static void bmp_reset(void)
{
    g_bmp_ctrl = g_bmp_config = 0;
    g_bmp_measuring = g_bmp_sampled = 0;
    g_bmp_adc_p = g_bmp_adc_t = BMP_SKIPPED;
    bmp_put20(g_bmp_data, BMP_SKIPPED);
    bmp_put20(g_bmp_data + 3, BMP_SKIPPED);
    g_bmp_im_until = g_nav->cycle + 2u * 16000u;     /* start-up time 2 ms max (3.1): the NVM copy */
    g_st.bmp_resets++;
}

/* the measurement's progress up to the NAV's clock now: the pressure sampling instant, the end (data registers) */
static void bmp_advance(void)
{
    if (!g_bmp_measuring) return;
    if (!g_bmp_sampled && g_nav->cycle >= g_bmp_sample_at) {
        g_bmp_adc_p = ((g_bmp_ctrl >> 2) & 7u) ? bmp_adc_p_for(g_p_pa) : BMP_SKIPPED;
        g_bmp_adc_t = (g_bmp_ctrl >> 5) ? BMP_ADC_T : BMP_SKIPPED;
        g_bmp_sampled = 1;
    }
    if (g_nav->cycle >= g_bmp_done_at) {
        bmp_put20(g_bmp_data, g_bmp_adc_p);
        bmp_put20(g_bmp_data + 3, g_bmp_adc_t);
        g_bmp_measuring = 0;
        g_bmp_ctrl &= (uint8_t)~3u;                  /* forced mode: back to sleep */
        g_st.bmp_done++;
    }
}

static void bmp_write(uint8_t reg, uint8_t v)
{
    bmp_advance();
    if (reg == 0xE0) {
        if (v == 0xB6) bmp_reset();
    } else if (reg == 0xF5) {
        g_bmp_config = v;
    } else if (reg == 0xF4) {
        const unsigned mode = v & 3u, osrs_t = v >> 5, osrs_p = (v >> 2) & 7u;
        const unsigned nt = osrs_t > 5u ? 16u : (osrs_t ? 1u << (osrs_t - 1u) : 0u);   /* oversampling factors */
        const unsigned np = osrs_p > 5u ? 16u : (osrs_p ? 1u << (osrs_p - 1u) : 0u);
        if (g_bmp_measuring) {                       /* the running measurement goes on with its settings */
            g_st.bmp_busy_writes++;
            return;
        }
        g_bmp_ctrl = v;
        if (mode == 1u || mode == 2u) {              /* forced: one measurement, then sleep */
            const double max_ms = 1.25 + 2.3 * nt + (np ? 2.3 * np + 0.575 : 0.0);
            const double mid_ms = 1.0 + 2.0 * nt + (np ? (2.0 * np + 0.5) / 2.0 : 0.0);
            g_bmp_measuring = 1;
            g_bmp_sampled = 0;
            g_bmp_sample_at = g_nav->cycle + (avr_cycle_count_t)(mid_ms * 16000.0 + 0.5);
            g_bmp_done_at = g_nav->cycle + (avr_cycle_count_t)(max_ms * 16000.0 + 0.5);
            g_st.bmp_forced++;
        } else if (mode == 3u) {
            g_st.bmp_normal_writes++;                /* normal mode: not modelled, nothing is measured */
        }
    }
}

static uint8_t bmp_read(uint8_t reg)
{
    bmp_advance();
    if (reg >= 0x88 && reg <= 0xA1) {
        if (reg == 0x88) g_st.bmp_calib_reads++;
        return g_bmp_calib[reg - 0x88];
    }
    if (reg == 0xD0) return 0x58;
    if (reg == 0xF3) return (uint8_t)((g_bmp_measuring ? 0x08u : 0u) | (g_nav->cycle < g_bmp_im_until ? 0x01u : 0u));
    if (reg == 0xF4) return g_bmp_ctrl;
    if (reg == 0xF5) return g_bmp_config;
    if (reg >= 0xF7 && reg <= 0xFC) {
        if (reg == 0xF7) {
            g_st.bmp_data_reads++;
            if (g_bmp_measuring) g_st.bmp_early++;
        }
        return g_bmp_data[reg - 0xF7];
    }
    return 0x00;
}

static void mpu_latch(void)                          /* the burst the NAV reads: the plant's state now */
{
    const double ga = 16384.0 / pow(2.0, (g_mpu_reg[0x1C] >> 3) & 3);              /* LSB per g */
    const double gs = 131.0 / pow(2.0, (g_mpu_reg[0x1B] >> 3) & 3) * 57.29577951308232;   /* LSB per rad/s */
    const double g0 = 9.80665;
    be16put(&g_mpu_reg[0x3B], lround(g_accel[0] / g0 * ga));
    be16put(&g_mpu_reg[0x3D], lround(g_accel[1] / g0 * ga));
    be16put(&g_mpu_reg[0x3F], lround(g_accel[2] / g0 * ga));
    be16put(&g_mpu_reg[0x41], 1200);
    be16put(&g_mpu_reg[0x43], lround(g_gyro[0] * gs));
    be16put(&g_mpu_reg[0x45], lround(g_gyro[1] * gs));
    be16put(&g_mpu_reg[0x47], lround(g_gyro[2] * gs));
    g_st.bursts++;
}

static avr_cycle_count_t conv_cycles(uint8_t c)      /* datasheet maximum conversion time by OSR (c & 0x0E) */
{
    static const double ms[5] = {0.60, 1.17, 2.28, 4.54, 9.04};
    const unsigned osr = (c & 0x0Fu) >> 1;
    return (avr_cycle_count_t)(ms[osr < 5u ? osr : 4u] * 16000.0 + 0.5);
}

static void twi_out_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    avr_twi_msg_irq_t v;
    (void)irq;
    (void)param;
    v.u.v = value;
    g_st.msgs++;
    if (v.u.twi.msg & TWI_COND_STOP) g_sel = 0;
    if (v.u.twi.msg & TWI_COND_START) {
        g_sel = 0;
        if ((v.u.twi.addr >> 1) == 0x68 || (v.u.twi.addr >> 1) == 0x77) {
            g_sel = v.u.twi.addr;
            avr_raise_irq(g_twi_in, avr_twi_irq_msg(TWI_COND_ACK, g_sel, 1));
            if ((g_sel >> 1) == 0x68) {
                g_mpu_first_write = 1;
                if ((g_sel & 1) && g_mpu_ptr == 0x3B) mpu_latch();
            }
            if ((g_sel >> 1) == 0x77 && (g_sel & 1)) g_ms_out_i = 0;
            if ((g_sel >> 1) == 0x77 && !(g_sel & 1)) g_bmp_expect_reg = 1;
        }
    }
    if (!g_sel) return;
    if (v.u.twi.msg & TWI_COND_WRITE) {
        avr_raise_irq(g_twi_in, avr_twi_irq_msg(TWI_COND_ACK, g_sel, 1));
        if ((g_sel >> 1) == 0x68) {
            if (g_mpu_first_write) {
                g_mpu_ptr = v.u.twi.data & 0x7Fu;
                g_mpu_first_write = 0;
            } else {
                g_mpu_reg[g_mpu_ptr & 0x7Fu] = v.u.twi.data;
                g_mpu_ptr++;
            }
        } else if (g_baro_kind == TWI_BARO_BMP280) {      /* 5.2.1: register address, then data / address pairs */
            g_st.ms_cmds++;
            if (g_bmp_expect_reg) {
                g_bmp_ptr = v.u.twi.data;
                g_bmp_expect_reg = 0;
            } else {
                bmp_write(g_bmp_ptr, v.u.twi.data);
                g_bmp_expect_reg = 1;
            }
        } else {
            const uint8_t c = v.u.twi.data;
            g_st.ms_cmds++;
            if (c == 0x1E) {
                g_adc = 0;
                g_conv = 0;
            } else if (c >= 0xA0 && c <= 0xAE) {
                const uint16_t w = g_prom[(c - 0xA0) >> 1];
                g_ms_out[0] = (uint8_t)(w >> 8);
                g_ms_out[1] = (uint8_t)w;
                g_ms_out_n = 2;
                g_st.prom_reads++;
            } else if (c >= 0x40 && c <= 0x48) {
                g_conv = 1;
                g_adc = d1_for(g_p_pa);              /* the pressure at the conversion's start */
                g_conv_done = g_nav->cycle + conv_cycles(c);
                g_st.d1_convs++;
            } else if (c >= 0x50 && c <= 0x58) {
                g_conv = 2;
                g_adc = k_d2;
                g_conv_done = g_nav->cycle + conv_cycles(c);
                g_st.d2_convs++;
            } else if (c == 0x00) {
                const int early = g_conv && g_nav->cycle < g_conv_done;
                const uint32_t r = (g_conv && !early) ? g_adc : 0u;   /* not done / none: the chip returns 0 */
                g_ms_out[0] = (uint8_t)(r >> 16);
                g_ms_out[1] = (uint8_t)(r >> 8);
                g_ms_out[2] = (uint8_t)r;
                g_ms_out_n = 3;
                g_st.adc_reads++;
                if (early) g_st.adc_early++;
                g_conv = 0;
                g_adc = 0;
            }
        }
    }
    if (v.u.twi.msg & TWI_COND_READ) {
        uint8_t d = 0xFF;
        g_st.reads++;
        if ((g_sel >> 1) == 0x68) {
            d = g_mpu_ptr == 0x75 ? 0x68 : g_mpu_reg[g_mpu_ptr & 0x7Fu];
            if (g_mpu_ptr >= 0x3B && g_mpu_ptr < 0x49) g_st.burst_reads++;
            g_mpu_ptr++;
        } else if (g_baro_kind == TWI_BARO_BMP280) {      /* 5.2.2: the address increments on a read */
            d = bmp_read(g_bmp_ptr);
            g_bmp_ptr++;
        } else {
            d = g_ms_out_i < g_ms_out_n ? g_ms_out[g_ms_out_i] : 0;
            g_ms_out_i++;
        }
        avr_raise_irq(g_twi_in, avr_twi_irq_msg(TWI_COND_READ, g_sel, d));
    }
}

/* after simavr's own TWCR write handler (chained): the state timer it just scheduled -> the chip's bus time.
 * Measured on this simavr (twidbg=N prints them; TWBR 12, TWPS 0 = 400 kHz, 40 cycles per SCL period): a data byte or an
 * SLA after a REPEATED START is scheduled 360 cycles after the TWCR write (= 9 SCL periods = 22.5 us, already right), but
 * a START (TWSTA) and the SLA byte right after a first START (TWSR 0x08) are scheduled 0 cycles after it.  The chip needs
 * the START condition (~1 SCL period, ASSUMED: bus free + tHD;STA at 400 kHz) and 9 SCL periods for every byte, so a
 * transfer started with START is ~25 us short per transaction in simavr.  The harness moves the timer to
 * TWSTA ? 1 : 9 SCL periods (the clock from TWBR / TWPS) and histograms the delays it found (stats.rescale_hist[k]:
 * k = round(delay / 40 cycles), 15 = 15 or more). */
#define TWCR_TWINT 0x80u
#define TWCR_TWSTA 0x20u
#define TWCR_TWSTO 0x10u
static void twcr_write_hook(struct avr_t *avr, avr_io_addr_t addr, uint8_t v, void *param)
{
    avr_cycle_timer_slot_p s;
    (void)addr;
    (void)param;
    if (!g_fix || !(v & TWCR_TWINT)) return;            /* only a write that clears TWINT starts a bus step */
    for (s = avr->cycle_timers.timer; s; s = s->next) {
        if (s->param == (void *)g_twi && s->when != g_twi_when && s->when >= avr->cycle) {
            const avr_cycle_timer_t fn = s->timer;
            const avr_cycle_count_t d = s->when - avr->cycle;
            const unsigned twps = avr->data[D_TWSR] & 3u;
            const uint64_t cpb = 16u + 2u * (uint64_t)avr->data[D_TWBR] * (1u << (2u * twps));
            const avr_cycle_count_t nd = (avr_cycle_count_t)(((v & TWCR_TWSTA) ? 1u : 9u) * cpb);
            const uint64_t k = (d + cpb / 2u) / cpb;
            if (g_debug > 0) {
                g_debug--;
                fprintf(stderr, "twi: TWCR=0x%02x TWSR=0x%02x TWBR=%u simavr delay %llu cycles -> %llu\n", v,
                        avr->data[D_TWSR], avr->data[D_TWBR], (unsigned long long)d, (unsigned long long)nd);
            }
            g_st.scl_cycles_per_bit = cpb;
            g_st.rescale_hist[k < 15u ? k : 15u]++;
            if (nd != d) {
                avr_cycle_timer_cancel(avr, fn, (void *)g_twi);
                avr_cycle_timer_register(avr, nd, fn, (void *)g_twi);
                g_st.rescaled++;
            }
            g_twi_when = avr->cycle + nd;
            return;
        }
    }
    if (!(v & TWCR_TWSTO)) g_st.not_found++;            /* a STOP schedules nothing */
}

void twi_models_init(avr_t *nav, int fix_byte_time, uint64_t seed, double noise_gyro, double noise_accel,
                     double noise_pa)
{
    avr_io_t *p;
    (void)seed;
    (void)noise_gyro;
    (void)noise_accel;
    (void)noise_pa;
    g_nav = nav;
    g_fix = fix_byte_time;
    for (p = nav->io_port; p; p = p->next)
        if (p->kind && !strcmp(p->kind, "twi")) g_twi = (avr_twi_t *)p;
    g_prom[0] = 0x0000;
    g_prom[1] = 40127;
    g_prom[2] = 36924;
    g_prom[3] = 23317;
    g_prom[4] = 23282;
    g_prom[5] = 33464;
    g_prom[6] = 28312;
    g_prom[7] = 0x0000;
    g_prom[7] = (uint16_t)((g_prom[7] & 0xFFF0u) | crc4(g_prom));
    {
        /* BMP280 datasheet 3.12 calculation example: dig_T1..T3, dig_P1..P9 (0x88 .. 0x9F little endian; 0xA0 / 0xA1
         * reserved) */
        static const long k_trim[12] = {27504, 26435, -1000, 36477, -10685, 3024, 2855, 140, -7, 15500, -14600, 6000};
        unsigned i;
        for (i = 0; i < 12u; i++) {
            g_bmp_calib[2u * i] = (uint8_t)((unsigned long)k_trim[i] & 0xFFu);
            g_bmp_calib[2u * i + 1u] = (uint8_t)(((unsigned long)k_trim[i] >> 8) & 0xFFu);
        }
        g_bc.t1 = 27504.0, g_bc.t2 = 26435.0, g_bc.t3 = -1000.0;
        g_bc.p1 = 36477.0, g_bc.p2 = -10685.0, g_bc.p3 = 3024.0, g_bc.p4 = 2855.0, g_bc.p5 = 140.0, g_bc.p6 = -7.0;
        g_bc.p7 = 15500.0, g_bc.p8 = -14600.0, g_bc.p9 = 6000.0;
        bmp_put20(g_bmp_data, BMP_SKIPPED);
        bmp_put20(g_bmp_data + 3, BMP_SKIPPED);
    }
    g_twi_in = avr_io_getirq(nav, AVR_IOCTL_TWI_GETIRQ(0), TWI_IRQ_INPUT);
    avr_irq_register_notify(avr_io_getirq(nav, AVR_IOCTL_TWI_GETIRQ(0), TWI_IRQ_OUTPUT), twi_out_hook, NULL);
    if (g_fix && g_twi) avr_register_io_write(nav, D_TWCR, twcr_write_hook, NULL);
    if (!g_twi) g_fix = 0;
}

void twi_set_truth(const double gyro[3], const double accel[3], double p_pa)
{
    memcpy(g_gyro, gyro, sizeof g_gyro);
    memcpy(g_accel, accel, sizeof g_accel);
    if (g_baro_kind == TWI_BARO_BMP280 && g_nav)
        bmp_advance();                               /* a sampling instant that passed takes the pressure before it */
    g_p_pa = p_pa;
}

void twi_models_set_baro(int kind)
{
    g_baro_kind = kind;
}

const char *twi_models_baro_name(void)
{
    return g_baro_kind == TWI_BARO_BMP280 ? "bmp280" : "ms5611";
}

void twi_models_reset(void)
{
    g_sel = 0;
    g_ms_out_n = g_ms_out_i = 0;
    g_bmp_expect_reg = 0;
    g_twi_when = 0;
}

const twi_stats_t *twi_models_stats(void)
{
    return &g_st;
}
