/* spi_probe.c - runs tools/cosim/spi_probe_avr.c (Intel HEX, master or slave build) in libsimavr and prints what
 * simavr's SPI did, the facts cosim.c's nlink wiring rests on.  tools/run_cosim.py runs it on both builds every time.
 *
 * usage: spi_probe master <master.hex>   prints: write_to_output=<cycles from the SPDR write to SPI_IRQ_OUTPUT>
 *                                                output=<byte> received=<byte the firmware read back after SPIF>
 *        spi_probe slave <slave.hex>     prints: ss_pin=<PINB.0 when the byte was fed> output=<byte simavr's slave put on
 *                                                SPI_IRQ_OUTPUT when it received 0x3C, -1 none> loaded=<the byte the
 *                                                slave had written to SPDR, 0xA5> received=<byte the firmware read back>
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "avr_ioport.h"
#include "avr_spi.h"
#include "sim_avr.h"
#include "sim_hex.h"
#include "sim_io.h"

#define D_SPDR 0x2F
#define D_PINB 0x36
#define D_PORTC 0x35

static avr_t *g_avr;
static int g_slave;
static unsigned long long g_t_write, g_t_out;
static int g_out = -1, g_ss = -1;

static void spdr_write(struct avr_t *avr, avr_io_addr_t addr, uint8_t v, void *param)
{
    (void)addr;
    (void)param;
    (void)v;
    if (!g_t_write) g_t_write = avr->cycle;
}

static void spi_out(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq;
    (void)param;
    if (g_out >= 0) return;
    g_out = (int)(value & 0xFFu);
    g_t_out = g_avr->cycle;
    if (!g_slave)                                   /* the master's byte is complete: answer it on MISO */
        avr_raise_irq(avr_io_getirq(g_avr, AVR_IOCTL_SPI_GETIRQ(0), SPI_IRQ_INPUT), 0xC3);
}

static void porta(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq;
    (void)param;
    if (g_slave && value == 1) {                    /* the slave is ready: feed a byte while its SS is high */
        g_ss = g_avr->data[D_PINB] & 1;
        avr_raise_irq(avr_io_getirq(g_avr, AVR_IOCTL_SPI_GETIRQ(0), SPI_IRQ_INPUT), 0x3C);
    }
}

int main(int argc, char **argv)
{
    uint32_t size = 0, start = 0;
    uint8_t *code;
    int st;
    if (argc < 3 || (strcmp(argv[1], "master") && strcmp(argv[1], "slave"))) {
        fprintf(stderr, "usage: spi_probe master|slave <hex>\n");
        return 2;
    }
    g_slave = !strcmp(argv[1], "slave");
    g_avr = avr_make_mcu_by_name("atmega128");
    if (!g_avr) return 1;
    avr_init(g_avr);
    g_avr->frequency = 16000000;
    code = read_ihex_file(argv[2], &size, &start);
    if (!code) {
        fprintf(stderr, "cannot read %s\n", argv[2]);
        return 1;
    }
    memcpy(g_avr->flash + start, code, size);
    g_avr->codeend = start + size - 1u;
    g_avr->pc = start;
    free(code);
    avr_register_io_write(g_avr, D_SPDR, spdr_write, NULL);
    avr_irq_register_notify(avr_io_getirq(g_avr, AVR_IOCTL_SPI_GETIRQ(0), SPI_IRQ_OUTPUT), spi_out, NULL);
    avr_irq_register_notify(avr_io_getirq(g_avr, AVR_IOCTL_IOPORT_GETIRQ('A'), IOPORT_IRQ_REG_PORT), porta, NULL);
    do
        st = avr_run(g_avr);
    while (st != cpu_Done && st != cpu_Crashed && g_avr->cycle < 1000000u);
    if (st != cpu_Done) {
        fprintf(stderr, "probe did not finish (state %d)\n", st);
        return 1;
    }
    if (g_slave)
        printf("ss_pin=%d output=%d loaded=%d received=%d\n", g_ss, g_out, 0xA5, g_avr->data[D_PORTC]);
    else
        printf("write_to_output=%llu output=%d received=%d\n", g_t_out - g_t_write, g_out, g_avr->data[D_PORTC]);
    return 0;
}
