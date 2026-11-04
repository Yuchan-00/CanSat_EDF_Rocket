/* isr_cost.c - runs the isr_cost_avr.c firmware (Intel HEX) in libsimavr and prints the cycles between its two PORTA
 * marks, the Timer0 compare interrupts taken (vector 15) and simavr's RUNNING window per interrupt (vector entry ->
 * RETI).  tools/run_cosim.py runs it on the build without and with -D WITH_ISR, once with resp=0 (simavr's own charge:
 * 7 cycles per bare-RETI interrupt) and once with the co-simulation's correction (resp=4: cosim.c adds the chip's
 * 4-cycle interrupt response to the core's clock when an interrupt starts running, the same way as here: 11).
 *
 * usage: isr_cost <isr_cost_avr.hex> [resp]   prints: window=<cycles> isr=<runs> running=<cycles in RUNNING windows>
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "avr_ioport.h"
#include "sim_avr.h"
#include "sim_hex.h"
#include "sim_interrupts.h"

static avr_t *g_avr;
static unsigned long long g_up, g_down, g_isr, g_in, g_running;
static int g_resp;

static void porta_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq;
    (void)param;
    if (value & 1u) g_up = g_avr->cycle;
    else if (g_up && !g_down) g_down = g_avr->cycle;
}

static void running_hook(struct avr_irq_t *irq, uint32_t value, void *param)
{
    (void)irq;
    (void)param;
    if (value) {
        g_in = g_avr->cycle;
        g_avr->cycle += (avr_cycle_count_t)g_resp;     /* as cosim.c isr_running_hook (isr_resp) */
    } else {
        g_isr++;
        g_running += g_avr->cycle - g_in;
    }
}

int main(int argc, char **argv)
{
    uint32_t size = 0, start = 0;
    uint8_t *code;
    int st;
    if (argc < 2) {
        fprintf(stderr, "usage: isr_cost <isr_cost_avr.hex> [resp]\n");
        return 2;
    }
    if (argc > 2) g_resp = atoi(argv[2]);
    g_avr = avr_make_mcu_by_name("atmega128");
    if (!g_avr) return 1;
    avr_init(g_avr);
    g_avr->frequency = 16000000;
    code = read_ihex_file(argv[1], &size, &start);
    if (!code) {
        fprintf(stderr, "cannot read %s\n", argv[1]);
        return 1;
    }
    memcpy(g_avr->flash + start, code, size);
    g_avr->codeend = start + size - 1u;
    g_avr->pc = start;
    free(code);
    avr_irq_register_notify(avr_io_getirq(g_avr, AVR_IOCTL_IOPORT_GETIRQ('A'), IOPORT_IRQ_REG_PORT), porta_hook, NULL);
    avr_irq_register_notify(avr_get_interrupt_irq(g_avr, 15) + AVR_INT_IRQ_RUNNING, running_hook, NULL);
    do
        st = avr_run(g_avr);
    while (st != cpu_Done && st != cpu_Crashed && g_avr->cycle < 10000000u);
    if (!g_down) {
        fprintf(stderr, "no end mark\n");
        return 1;
    }
    printf("window=%llu isr=%llu running=%llu\n", g_down - g_up, g_isr, g_running);
    return 0;
}
