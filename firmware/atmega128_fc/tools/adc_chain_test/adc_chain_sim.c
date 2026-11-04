/* adc_chain_sim (fw09 review F2): runs adc_chain_avr.hex on simavr's ATmega128 at 16 MHz with fixed millivolts on ADC
 * inputs 0..7 and copies the AVR's USART0 output to stdout.  It also counts the ADC interrupts the AVR executed (the
 * vector-21 "running" IRQ of simavr's interrupt table) and prints "SIM adc_isr=<n> cycles=<n> state=<n>" at the end.
 * usage: adc_chain_sim <hex> mv=<ch0>,<ch1>,...,<ch7> [max_ms=200]
 * Built by run_adc_chain_test.py with zig cc against PlatformIO's libsimavr.a (as tools/cosim/cosim4.c). */
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "avr_adc.h"
#include "avr_uart.h"
#include "sim_avr.h"
#include "sim_hex.h"
#include "sim_interrupts.h"
#include "sim_irq.h"

#define ADC_VECTOR 21

static unsigned long g_adc_isr;

static void uart_out_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq;
    (void)param;
    putchar((int)(value & 0xFFu));
}

static void isr_running_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq;
    (void)param;
    if (value)
        g_adc_isr++;                          /* the ADC vector became the running interrupt */
}

static void die(const char *m)
{
    fprintf(stderr, "adc_chain_sim: %s\n", m);
    exit(2);
}

int main(int argc, char **argv)
{
    avr_t *avr;
    uint32_t size = 0, start = 0, flags = 0;
    uint8_t *code;
    unsigned mv[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    double max_ms = 200.0;
    int i, st = cpu_Running;
    FILE *probe;
    if (argc < 2)
        die("usage: adc_chain_sim <hex> mv=a,b,... [max_ms=200]");
    for (i = 2; i < argc; i++) {
        if (!strncmp(argv[i], "mv=", 3)) {
            const char *p = argv[i] + 3;
            unsigned k = 0;
            while (*p && k < 8u) {
                mv[k++] = (unsigned)strtoul(p, (char **)&p, 10);
                if (*p == ',')
                    p++;
            }
        } else if (!strncmp(argv[i], "max_ms=", 7)) {
            max_ms = atof(argv[i] + 7);
        }
    }
    probe = fopen(argv[1], "rb");
    if (!probe)
        die("cannot open the hex file");
    fclose(probe);
    avr = avr_make_mcu_by_name("atmega128");
    if (!avr)
        die("simavr has no atmega128 core");
    avr_init(avr);
    avr->frequency = 16000000u;
    avr->vcc = avr->avcc = 5000u;
    avr->aref = 2560u;
    code = read_ihex_file(argv[1], &size, &start);
    if (!code)
        die("cannot read the hex file");
    if (start + size > avr->flashend + 1u)
        die("the image does not fit the flash");
    memcpy(avr->flash + start, code, size);
    avr->codeend = start + size - 1u;
    avr->pc = start;
    free(code);
    avr_ioctl(avr, AVR_IOCTL_UART_GET_FLAGS('0'), &flags);
    flags &= ~(uint32_t)(AVR_UART_FLAG_STDIO | AVR_UART_FLAG_POLL_SLEEP);
    avr_ioctl(avr, AVR_IOCTL_UART_SET_FLAGS('0'), &flags);
    avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_OUTPUT), uart_out_hook, NULL);
    avr_irq_register_notify(avr_get_interrupt_irq(avr, ADC_VECTOR) + AVR_INT_IRQ_RUNNING, isr_running_hook, NULL);
    for (i = 0; i < 8; i++)
        avr_raise_irq(avr_io_getirq(avr, AVR_IOCTL_ADC_GETIRQ, ADC_IRQ_ADC0 + i), mv[i]);
    while (st != cpu_Done && st != cpu_Crashed && (double)avr->cycle < max_ms * 16000.0)
        st = avr_run(avr);
    printf("SIM adc_isr=%lu cycles=%llu state=%d\n", g_adc_isr, (unsigned long long)avr->cycle, st);
    return st == cpu_Crashed ? 3 : 0;
}
