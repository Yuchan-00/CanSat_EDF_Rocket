/* ilink_uart.h - the FCU <-> COM inter-processor UART (SPEC 10.4, v0.6): byte rings behind the
 * USART that ILINK_USART selects (platformio.ini: FCU -D ILINK_USART=1, COM -D ILINK_USART=0).
 *
 *   FCU  USART1  PD2 RXD1 <- COM TXD0 (direct)     PD3 TXD1 -> COM RXD0 through 1 kOhm
 *   COM  USART0  PE0 RXD0 <- FCU TXD1 (1 kOhm)     PE1 TXD0 -> FCU RXD1 (direct)
 *
 * ILINK_BAUD 8N1 (params_gen.h ILINK_UBRR / ILINK_U2X, flight_computer.ilink.baud = 250 000:
 * UBRR 3, U2X 0, 0 % error at 16 MHz) - one byte every 40 us.  The RX-complete ISR stores each
 * byte in a 64-byte ring (a byte with a framing / overrun / parity error is counted in rx_errors
 * and dropped; a full ring counts rx_overflow); it is hand-written (51 CPU cycles a byte, 3.2 us:
 * ilink_uart.c) because on the FCU it also runs inside the time-critical part.  The UDRE ISR
 * empties the TX ring (FCU 64 B, COM 128 B); hand-written too since v0.6.3 (47 cycles a byte),
 * because on the FCU both can run while the NAV's nlink SPI exchange is in progress, where every
 * FCU interrupt must stay under ~50 cycles (hal/nlink_spi.h Timing).  The frames themselves are
 * core/ilink_proto.c's business: this layer only moves bytes.
 *
 * The RX pin's pull-up is switched on (PORT bit; the USART receiver keeps it controllable), so a
 * peer that is unpowered, held in reset or unplugged leaves the line idle-high instead of floating
 * into the receiver as noise.
 *
 * Single-chip simbench (FC_SIMBENCH without FC_COSIM): the peer is synthetic and the hardware is
 * bypassed - no USART register is touched and no ISR is defined (the same USART may carry the
 * simbench console), ilink_uart_inject_rx() puts bytes into the RX ring as if received and
 * ilink_uart_take_tx() takes queued bytes out of the TX ring as if sent; the per-byte ISR cost is
 * therefore not part of that measurement.  FC_COSIM (two chips in simavr, USARTs wired together)
 * uses the real hardware path.
 *
 * Invariant: the ISRs only move bytes; ilink_uart_write queues a whole frame or nothing and never
 * waits, so the link can never stall the caller's loop.
 */
#ifndef FC_ILINK_UART_H
#define FC_ILINK_UART_H

#include <stdbool.h>
#include <stdint.h>

#include "../fc_mode.h"

#if defined(FC_ROLE_COM) && FC_VTOL
#define ILINK_UART_RX_BUF 128u               /* VTOL COM, power of two; > one SNAP3 frame (v0.8 GPS build: 68 B; v0.7: 64 > SNAP2
                                              * 62 B); the ballistic COM keeps 64 (> SNAP 55 B: its image stays v0.7.1's) */
#else
#define ILINK_UART_RX_BUF 64u                /* power of two; > the FCU's largest burst (IL_CMD + IL_STAT, 56 B) */
#endif
#ifndef ILINK_UART_TX_BUF
#if defined(FC_ROLE_FCU) && FC_VTOL && !defined(ONB_NO_GPS)
#define ILINK_UART_TX_BUF 128u               /* FCU, v0.8 VTOL build with GPS position hold: one SNAP3 frame (68 B) */
#elif defined(FC_ROLE_FCU)
#define ILINK_UART_TX_BUF 64u                /* FCU: power of two; holds 63 bytes = one SNAP frame (55 B) - the FCU sends at
                                              * most one SNAP per control tick and a SNAP takes 2.2 ms on the wire */
#else
#define ILINK_UART_TX_BUF 128u               /* power of two; holds 127 bytes: two SNAP frames + a STAT */
#endif
#endif

typedef struct {
    uint16_t rx_bytes;                       /* bytes stored in the RX ring since init (read + still waiting) */
    uint16_t rx_overflow;                    /* bytes lost to a full RX ring */
    uint16_t rx_errors;                      /* bytes dropped for a framing / overrun / parity error */
    uint16_t tx_bytes;                       /* bytes accepted by ilink_uart_write */
} ilink_uart_stats_t;

void     ilink_uart_init(void);                          /* USART ILINK_USART, ILINK_UBRR / ILINK_U2X, 8N1, RX + UDRE ISRs */
uint8_t  ilink_uart_read(uint8_t *buf, uint8_t max);     /* bytes received since the last call (at most max) */
uint8_t  ilink_uart_rx_count(void);                      /* bytes waiting in the RX ring (callable with interrupts off) */
bool     ilink_uart_write(const uint8_t *frame, uint8_t n);   /* all n bytes queued, or none -> false */
uint8_t  ilink_uart_tx_free(void);                       /* bytes ilink_uart_write can take now */
bool     ilink_uart_tx_busy(void);                       /* bytes queued or still shifting out */
/* hold = true: no UDRE interrupt (the queued bytes wait; the one in UDR / the shift register still goes out), so a
 * caller can keep the per-byte TX interrupt out of a time-critical stretch; false: resume.  Writes queue as usual. */
void     ilink_uart_tx_hold(bool hold);
const ilink_uart_stats_t *ilink_uart_stats(void);

#if defined(FC_SIMBENCH) && !defined(FC_COSIM)            /* single-chip simbench: the peer is synthetic */
void     ilink_uart_inject_rx(const uint8_t *b, uint8_t n);   /* bytes as if received (ring full: counted, rest dropped) */
uint8_t  ilink_uart_take_tx(uint8_t *b, uint8_t max);     /* queued bytes as if sent (oldest first) */
#endif

#endif /* FC_ILINK_UART_H */
