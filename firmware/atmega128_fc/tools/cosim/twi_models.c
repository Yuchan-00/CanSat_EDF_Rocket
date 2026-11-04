/* twi_models.c - the NAV's I2C sensors in the four-core co-simulation (cosim4.c), from timing82/profile/harness/
 * navflight.c's register models, their values from the closed-loop plant (plant4.c) instead of a fixed hover:
 *   MPU-6050 at 0x68: register file (WHO_AM_I 0x75 = 0x68, the configuration writes kept); a read that starts at the
 *     burst address 0x3B latches the plant's body specific force and rates into 0x3B..0x48 at the scale the NAV
 *     configured (ACCEL_CONFIG / GYRO_CONFIG), temperature fixed;
 *   MS5611 at 0x77: reset, PROM (the datasheet's example C1..C6, a valid CRC4), D1 / D2 conversions with the datasheet's
 *     maximum conversion times (OSR 4096: 9.04 ms, 2048: 4.54 ms, ... ) - an ADC read before the conversion is done
 *     returns 0, as the chip does (counted) - and the ADC read; D2 is the example's 8569150 (20.07 C, no second-order
 *     term), D1 the inverse of the firmware's own first-order compensation (core/ms5611_math.c) for the plant's static
 *     pressure, so the NAV's integer Pascal is the plant's pressure.
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
    g_twi_in = avr_io_getirq(nav, AVR_IOCTL_TWI_GETIRQ(0), TWI_IRQ_INPUT);
    avr_irq_register_notify(avr_io_getirq(nav, AVR_IOCTL_TWI_GETIRQ(0), TWI_IRQ_OUTPUT), twi_out_hook, NULL);
    if (g_fix && g_twi) avr_register_io_write(nav, D_TWCR, twcr_write_hook, NULL);
    if (!g_twi) g_fix = 0;
}

void twi_set_truth(const double gyro[3], const double accel[3], double p_pa)
{
    memcpy(g_gyro, gyro, sizeof g_gyro);
    memcpy(g_accel, accel, sizeof g_accel);
    g_p_pa = p_pa;
}

void twi_models_reset(void)
{
    g_sel = 0;
    g_ms_out_n = g_ms_out_i = 0;
    g_twi_when = 0;
}

const twi_stats_t *twi_models_stats(void)
{
    return &g_st;
}
