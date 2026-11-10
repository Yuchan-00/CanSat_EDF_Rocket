/* gps_uart.h - the GNS board's USART1 to the u-blox GPS receiver (NEO-7M; NEO-6M in v0.8) (v0.8.1 four-chip set: params flight_computer.gns,
 * electronics.gps; v0.8 ran this driver on the NAV, whose USART1 is the glink since v0.8.1): PD2 RXD1 <- GPS TX, PD3 TXD1
 * -> GPS RX, both through the GNS's BSS138 level shifter (the receiver's RX is not 5 V tolerant), GPS_BAUD 8N1 (params_gen.h
 * GPS_UBRR, U2X 0), interrupt-driven rings.  The frames are core/gps_ubx.h's business; this layer moves bytes.  (Below,
 * "the NAV" / "NAV tick" of the v0.8 text read "the GNS" / "its 4 ms poll", src/gns/gns_main.c.)
 *
 * RX: the USART1 RX-complete interrupt (hand-written, the same 51-cycle path as hal/ilink_uart.c's) puts every byte into
 * a GPS_UART_RX_BUF ring; a byte with a framing / overrun / parity error is dropped and counted (rx_errors: at the wrong
 * baud - before the boot configuration switched the receiver - the parser sees nothing but these and noise), a byte that
 * finds the ring full is dropped and counted (rx_overflow).  The NAV drains the ring once per tick (gps_uart_read) into
 * core/gps_nav.h gps_nav_tick: at 38400 baud ~15 bytes arrive per 4 ms tick at most, the ring holds 4 ticks' worth.
 * TX (only the boot configuration: CFG-PRT / CFG-RATE / CFG-MSG x 3 / CFG-NAV5 / MON-VER poll, <= UBX_FRAME_MAX bytes per
 * NAV tick): gps_uart_write queues a whole frame or refuses it (counted, never waits); the UDRE interrupt sends it.
 * Baud: the configurator (gps_cfg_tick's action) switches the NAV side between the receiver's factory 9600 and GPS_BAUD;
 * gps_uart_set_baud applies at once when the transmitter is idle (ring empty, last byte shifted out), else it is pending
 * and the frame written after it waits in a one-frame buffer until gps_uart_poll (once per NAV tick) finds the
 * transmitter idle - so a frame never leaves at the wrong baud (with params_gen.h's configurator timing the transmitter is
 * always idle by then: the 9600-baud CFG-PRT takes 29 ms, the settle time GPS_CFG_N_SETTLE ticks is 100 ms).
 *
 * Single-chip simbench / co-simulation NAV (FC_SIMBENCH): the transmitter is the real USART1 (simavr times it and runs
 * the UDRE interrupt); a copy of every queued frame goes to the synthetic receiver (gps_uart_bench_take_tx).  simavr has
 * no GPS on RXD1, so the received bytes come from the bench's own queue: a Timer3 compare interrupt (CTC, prescaler 1,
 * one interrupt per byte time of the synthetic receiver's baud - byte-accurate 8N1 timing) moves the next queued byte
 * into the RX ring through the same ring code as the RX interrupt (hand-written, a few cycles longer: it also pops the
 * bench queue - tools/run_nav_simbench.py counts both from the disassembly).  The flight build has no Timer3 interrupt.
 *
 * Invariant: no call ever waits for the UART; the interrupts only move bytes and indices (bounded cycles).
 */
#ifndef FC_GPS_UART_H
#define FC_GPS_UART_H

#include <stdbool.h>
#include <stdint.h>

#define GPS_UART_RX_BUF 64u          /* power of two <= 256; 4 ticks of a 38400-baud stream */
#define GPS_UART_TX_BUF 64u          /* power of two <= 256; > UBX_FRAME_MAX (44) */
#define GPS_UART_DEFER_MAX 48u       /* the one frame that waits for a pending baud switch (>= UBX_FRAME_MAX) */

typedef struct {
    uint16_t rx_bytes;               /* bytes received (taken + waiting in the ring), wraps */
    uint16_t rx_overflow;            /* bytes dropped: ring full */
    uint16_t rx_errors;              /* bytes dropped: framing / data overrun / parity */
    uint16_t tx_bytes;               /* bytes queued for sending */
    uint16_t tx_refused;             /* frames refused (ring or deferred buffer full) */
    uint16_t baud_changes;           /* baud switches applied */
} gps_uart_stats_t;

void     gps_uart_init(void);                         /* GPS_BAUD 8N1, RX interrupt on, rings empty */
uint8_t  gps_uart_read(uint8_t *buf, uint8_t max);    /* the waiting bytes, oldest first (main context only) */
uint8_t  gps_uart_rx_count(void);
bool     gps_uart_write(const uint8_t *frame, uint8_t n);   /* whole frame queued (true) or refused (counted) */
void     gps_uart_set_baud(uint32_t baud);            /* 9600 or GPS_BAUD: now if the transmitter is idle, else pending */
void     gps_uart_poll(void);                         /* once per NAV tick: a pending switch + its deferred frame */
uint32_t gps_uart_baud(void);                         /* the baud the NAV side receives / sends at now */
bool     gps_uart_tx_idle(void);
const gps_uart_stats_t *gps_uart_stats(void);         /* a snapshot taken with interrupts off */

#ifdef FC_SIMBENCH
#define GPS_UART_BENCH_Q 64u         /* the bench wire queue (power of two): topped up every NAV tick */
/* the synthetic receiver's bytes "on the wire" towards the NAV: queued now, delivered one per byte_cycles CPU cycles
 * (10 bit times of the receiver's baud) by the Timer3 interrupt; returns the bytes accepted (the queue holds GPS_UART_BENCH_Q - 1) */
uint8_t  gps_uart_bench_send(const uint8_t *b, uint8_t n, uint16_t byte_cycles);
uint8_t  gps_uart_bench_pending(void);               /* queued bytes not delivered yet */
uint8_t  gps_uart_bench_take_tx(uint8_t *b, uint8_t max);   /* copies of the frames the NAV queued for sending */
#endif

#endif /* FC_GPS_UART_H */
