/* esp_bridge_sim (2026-11-06): runs esp_bridge_avr.hex on simavr's ATmega128 at 16 MHz as the ESP-12E's side of the
 * wire: it watches PC6 (the ESP's RST through Q5), feeds bytes into the AVR's USART1 receiver (the ESP's TXD0) and records
 * what the AVR sends on USART1 (the ESP's RXD0).  The AVR's USART0 console is copied to stdout, the events below go to
 * stderr (so they never split a console line):
 *   PIN t_us=<us> v=<0|1>      every change of the ESP's RST level: PC6 driven (DDR 1) -> its PORT bit, PC6 an input ->
 *                              1 (Q5's pull-ups; review F1: the line is open-drain); the start level is 1 (AVR in reset)
 *   DRVHI t_us=<us>            PC6 driven high (DDR 1 and PORT 1): must never happen
 *   U1 t_us=<us> b=<hex>       every byte the AVR's USART1 transmitted
 *   INJ t_us=<us> n=<bytes>    a burst fed to USART1 RX: garbage at garbage_ms (inside the boot window), a 48-byte burst
 *                              starting at span_ms (meant to straddle the window's end; 0 = none, review F5), a 13-byte
 *                              CMD-like frame at frame_ms (after it)
 *   SIM cycles=<n> state=<n>
 * simavr's USART input takes whole bytes: a 74880-baud framing error (FE1) cannot be injected (A; the link decoder's
 * resync on garbage is covered by host_test/test_core.c).
 * usage: esp_bridge_sim <hex> [garbage_ms=200] [span_ms=0] [frame_ms=600] [max_ms=1300]
 * Built by run_esp_bridge_test.py with zig cc against PlatformIO's libsimavr.a (as tools/adc_chain_test). */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "avr_ioport.h"
#include "avr_uart.h"
#include "sim_avr.h"
#include "sim_hex.h"
#include "sim_irq.h"

static avr_t *g_avr;

static double t_us(void)
{
    return (double)g_avr->cycle / 16.0;
}

static void console_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq;
    (void)param;
    putchar((int)(value & 0xFFu));
}

static void u1_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq;
    (void)param;
    fprintf(stderr, "U1 t_us=%.1f b=%02x\n", t_us(), (unsigned)(value & 0xFFu));
}

static unsigned g_ddr6, g_port6, g_level = 1u;  /* PC6 direction / PORT bit, the RST level (pull-up when not driven) */

static void rst_update(void)
{
    const unsigned lv = g_ddr6 ? g_port6 : 1u;
    if (g_ddr6 && g_port6)
        fprintf(stderr, "DRVHI t_us=%.1f\n", t_us());
    if (lv != g_level) {
        g_level = lv;
        fprintf(stderr, "PIN t_us=%.1f v=%u\n", t_us(), lv);
    }
}

static void ddr_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq;
    (void)param;
    g_ddr6 = (value >> 6) & 1u;
    rst_update();
}

static void port_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq;
    (void)param;
    g_port6 = (value >> 6) & 1u;
    rst_update();
}

static void die(const char *m)
{
    fprintf(stderr, "esp_bridge_sim: %s\n", m);
    exit(2);
}

static void inject(const uint8_t *b, unsigned n)
{
    unsigned i;
    avr_irq_t *in = avr_io_getirq(g_avr, AVR_IOCTL_UART_GETIRQ('1'), UART_IRQ_INPUT);
    fprintf(stderr, "INJ t_us=%.1f n=%u\n", t_us(), n);
    for (i = 0; i < n; i++)
        avr_raise_irq(in, b[i]);
}

int main(int argc, char **argv)
{
    uint32_t size = 0, start = 0, flags = 0;
    uint8_t *code;
    double garbage_ms = 200.0, span_ms = 0.0, frame_ms = 600.0, max_ms = 1300.0;
    int i, st = cpu_Running, g_done = 0, s_done = 0, f_done = 0;
    uint8_t span[48];                                  /* < simavr's 64-entry UART input FIFO */
    static const uint8_t garbage[16] = {0x55, 0x00, 0xFF, 0xA5, 0x5A, 0x10, 0x72, 0x6F, 0x6D, 0xA5, 0xA5, 0x5A, 0x05,
                                        0x0D, 0x0A, 0x80};
    static const uint8_t cmd[13] = {0xA5, 0x5A, 0x08, 0x5A, 0x10, 0x07, 0x01, 0x00, 0x00, 0x34, 0x12, 0xC3, 0x3C};
    FILE *probe;
    if (argc < 2)
        die("usage: esp_bridge_sim <hex> [garbage_ms=] [frame_ms=] [max_ms=]");
    for (i = 2; i < argc; i++) {
        if (!strncmp(argv[i], "garbage_ms=", 11))
            garbage_ms = atof(argv[i] + 11);
        else if (!strncmp(argv[i], "span_ms=", 8))
            span_ms = atof(argv[i] + 8);
        else if (!strncmp(argv[i], "frame_ms=", 9))
            frame_ms = atof(argv[i] + 9);
        else if (!strncmp(argv[i], "max_ms=", 7))
            max_ms = atof(argv[i] + 7);
    }
    for (i = 0; i < (int)sizeof span; i++)
        span[i] = (uint8_t)(0x30u + (unsigned)i);       /* 0x30..0x5F: a recognisable sequence (no 0xA5 sync byte) */
    probe = fopen(argv[1], "rb");
    if (!probe)
        die("cannot open the hex file");
    fclose(probe);
    g_avr = avr_make_mcu_by_name("atmega128");
    if (!g_avr)
        die("simavr has no atmega128 core");
    avr_init(g_avr);
    g_avr->frequency = 16000000u;
    g_avr->vcc = g_avr->avcc = 5000u;
    code = read_ihex_file(argv[1], &size, &start);
    if (!code)
        die("cannot read the hex file");
    if (start + size > g_avr->flashend + 1u)
        die("the image does not fit the flash");
    memcpy(g_avr->flash + start, code, size);
    g_avr->codeend = start + size - 1u;
    g_avr->pc = start;
    free(code);
    for (i = 0; i < 2; i++) {
        const char u = (char)('0' + i);
        avr_ioctl(g_avr, AVR_IOCTL_UART_GET_FLAGS(u), &flags);
        flags &= ~(uint32_t)(AVR_UART_FLAG_STDIO | AVR_UART_FLAG_POLL_SLEEP);
        avr_ioctl(g_avr, AVR_IOCTL_UART_SET_FLAGS(u), &flags);
    }
    avr_irq_register_notify(avr_io_getirq(g_avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_OUTPUT), console_hook, NULL);
    avr_irq_register_notify(avr_io_getirq(g_avr, AVR_IOCTL_UART_GETIRQ('1'), UART_IRQ_OUTPUT), u1_hook, NULL);
    avr_irq_register_notify(avr_io_getirq(g_avr, AVR_IOCTL_IOPORT_GETIRQ('C'), IOPORT_IRQ_DIRECTION_ALL), ddr_hook, NULL);
    avr_irq_register_notify(avr_io_getirq(g_avr, AVR_IOCTL_IOPORT_GETIRQ('C'), IOPORT_IRQ_REG_PORT), port_hook, NULL);
    while (st != cpu_Done && st != cpu_Crashed && t_us() < max_ms * 1000.0) {
        st = avr_run(g_avr);
        if (!g_done && t_us() >= garbage_ms * 1000.0) {
            g_done = 1;
            inject(garbage, sizeof garbage);
        }
        if (!s_done && span_ms > 0.0 && t_us() >= span_ms * 1000.0) {
            s_done = 1;
            inject(span, sizeof span);
        }
        if (!f_done && t_us() >= frame_ms * 1000.0) {
            f_done = 1;
            inject(cmd, sizeof cmd);
        }
    }
    printf("SIM cycles=%llu state=%d\n", (unsigned long long)g_avr->cycle, st);
    return st == cpu_Crashed ? 3 : 0;
}
