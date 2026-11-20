/* spi_probe_avr.c - ATmega128 probe firmware for tools/cosim/spi_probe.c: how simavr's SPI behaves as a master and
 * as a slave, the facts the three-chip co-simulation's nlink wiring (cosim.c) compensates for.
 *
 * MASTER (default): SS (PB0) an output held high, master at F_CPU / 8 (SPR1:0 = 01, SPI2X = 1: the nlink's
 * params_gen.h NLINK_SPI_SPR / NLINK_SPI_2X), PORTA = 1, SPDR = 0x5A, busy-wait for SPIF, the byte read back goes to
 * PORTC, PORTA = 2.  The host answers the master's output byte with 0xC3 on the SPI input.
 * SLAVE (-D SLAVE): SPI enabled as slave (no interrupt), PB0 an input with its pull-up (SS HIGH: not selected), SPDR =
 * 0xA5 (the byte the slave would return), PORTA = 1; the host then feeds 0x3C into the slave's SPI input; busy-wait for
 * SPIF (with SS high the chip would never set it), SPDR -> PORTC, PORTA = 2.
 * Then sleep with interrupts off (simavr stops).
 */
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>

int main(void)
{
    uint8_t r;
    DDRA = 0xFF;
    PORTA = 0;
    DDRC = 0xFF;
    PORTC = 0;
#ifdef SLAVE
    DDRB = (uint8_t)(1 << PB3);                       /* MISO out, SS / SCK / MOSI in */
    PORTB = (uint8_t)(1 << PB0);                      /* SS pull-up: not selected */
    SPCR = (uint8_t)(1 << SPE);
    SPDR = 0xA5;
#else
    PORTB = (uint8_t)(1 << PB0);                      /* SS high before it becomes an output */
    DDRB = (uint8_t)((1 << PB0) | (1 << PB1) | (1 << PB2));
    SPSR = (uint8_t)(1 << SPI2X);
    SPCR = (uint8_t)((1 << SPE) | (1 << MSTR) | (1 << SPR0));   /* F_CPU / 16, x2: F_CPU / 8 */
#endif
    PORTA = 1;
#ifndef SLAVE
    SPDR = 0x5A;
#endif
    while (!(SPSR & (1 << SPIF))) {
    }
    r = SPDR;
    PORTC = r;
    PORTA = 2;
    cli();
    sleep_enable();
    sleep_cpu();
    return 0;
}
