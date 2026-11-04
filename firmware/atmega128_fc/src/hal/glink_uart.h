/* glink_uart.h - the NAV <-> GNS inter-processor UART "glink" (v0.8.1 four-chip build; params flight_computer.glink,
 * flight_computer.gns.pins): byte rings behind the USART that GLINK_USART selects (platformio.ini: NAV -D GLINK_USART=1,
 * GNS -D GLINK_USART=0).  The frames are core/glink_proto.h's business (STATE 60 B NAV -> GNS, RESULT 26 B GNS -> NAV,
 * once per 50 Hz cycle); this layer only moves bytes.
 *
 *   NAV  USART1  PD3 TXD1 -> GNS PE0 RXD0          PD2 RXD1 <- GNS PE1 TXD0
 *   GNS  USART0  PE0 RXD0 <- NAV PD3 TXD1 (ISP PDI) PE1 TXD0 -> NAV PD2 RXD1 (ISP PDO): unplug glink to program the GNS
 *
 * GLINK_BAUD 8N1 (params_gen.h GLINK_UBRR / GLINK_U2X: 250 000 baud, UBRR 3, U2X 0, 0 % error at 16 MHz) - one byte every
 * 40 us.  Both ends are interrupt driven (architect v0.8.1: "NAV sends / receives by UART ISRs (no blocking)"):
 *   RX  the RX-complete ISR stores each byte in a ring (hand-written, the ilink's 51-cycle path: 4 response + 3 JMP + 40
 *       body + 4 RETI); a byte with a framing / overrun / parity error is dropped and counted (rx_errors), a byte that
 *       finds the ring full is dropped and counted (rx_overflow).  The NAV drains its ring in the background of every tick
 *       and once more right before its frame build (nav_main.c); the GNS drains it in its main loop.
 *   TX  glink_uart_write queues a whole frame or refuses it (counted, never waits); the UDRE ISR sends it (47 cycles a
 *       byte).  The NAV writes one STATE frame per 50 Hz cycle (60 B, 2.40 ms on the line), the GNS one RESULT (26 B).
 * The RX pin's pull-up is on, so an unplugged / unpowered peer reads idle-high instead of noise.
 *
 * Bench builds (FC_SIMBENCH; never flash):
 *   NAV single-chip simbench (nav_simbench: FC_SIMBENCH without GLINK_WIRED): the transmitter is the real USART1
 *       (simavr times it and runs the UDRE interrupt) and a copy of every queued frame goes to the synthetic GNS
 *       (glink_uart_bench_take_tx, with the time the frame was queued); simavr has nothing on RXD1, so the received bytes
 *       come from the bench's own queue: glink_uart_bench_send(bytes, delay) schedules them on a Timer3 wire - the first
 *       byte completes `delay` CPU cycles from now, every next one one byte time (640 cycles) later - and the Timer3
 *       compare interrupt moves each into the RX ring through the RX ISR's ring code (hand-written, 18 cycles longer than
 *       the RX ISR: the queue pop and the next compare; tools/run_nav_simbench.py counts both from the disassembly).
 *   NAV two-core build (nav_simbench_glink: FC_SIMBENCH + GLINK_WIRED), NAV / GNS co-simulation builds (nav_cosim,
 *       gns_cosim: FC_SIMBENCH + FC_COSIM): the real USART on both directions - a simavr harness wires the chips.
 *   GNS single-chip simbench (gns_simbench: FC_SIMBENCH without FC_COSIM): no USART register is touched and no glink ISR
 *       is defined (USART0 carries the simbench console): glink_uart_bench_inject puts bytes into the RX ring as if
 *       received, glink_uart_bench_take_tx takes the queued bytes as if sent.
 * The flight builds have none of these paths.
 *
 * Invariant: no call ever waits for the UART; the ISRs only move bytes and indices (bounded cycles).
 */
#ifndef FC_GLINK_UART_H
#define FC_GLINK_UART_H

#include <stdbool.h>
#include <stdint.h>

#if defined(FC_ROLE_GNS)
#define GLINK_UART_RX_BUF 128u               /* GNS: power of two; two STATE frames (the main loop may be in a 7 ms cycle) */
#define GLINK_UART_TX_BUF 32u                /* GNS: one RESULT frame (26 B) */
#else
#define GLINK_UART_RX_BUF 64u                /* NAV: power of two; two RESULT frames (26 B each) */
#define GLINK_UART_TX_BUF 64u                /* NAV: one STATE frame (60 B) */
#endif

/* the build's glink paths (see above) */
#if defined(FC_SIMBENCH) && defined(FC_ROLE_GNS) && !defined(FC_COSIM)
#define GLINK_UART_HW 0                      /* gns_simbench: rings only */
#else
#define GLINK_UART_HW 1
#endif
#if GLINK_UART_HW && defined(FC_SIMBENCH) && defined(FC_ROLE_NAV) && !defined(GLINK_WIRED) && !defined(FC_COSIM)
#define GLINK_UART_BENCH_WIRE 1              /* nav_simbench: real TX + snoop, RX from the Timer3 bench wire */
#else
#define GLINK_UART_BENCH_WIRE 0
#endif

typedef struct {
    uint16_t rx_bytes;                       /* bytes stored in the RX ring since init (read + still waiting), wraps */
    uint16_t rx_overflow;                    /* bytes lost to a full RX ring */
    uint16_t rx_errors;                      /* bytes dropped for a framing / overrun / parity error */
    uint16_t tx_bytes;                       /* bytes accepted by glink_uart_write */
    uint16_t tx_refused;                     /* frames refused (ring full) */
} glink_uart_stats_t;

void     glink_uart_init(void);                          /* GLINK_USART at GLINK_UBRR / GLINK_U2X, 8N1, RX + UDRE ISRs */
uint8_t  glink_uart_read(uint8_t *buf, uint8_t max);     /* the waiting bytes, oldest first (main context only) */
uint8_t  glink_uart_rx_count(void);
bool     glink_uart_write(const uint8_t *frame, uint8_t n);   /* all n bytes queued, or none (counted) */
bool     glink_uart_tx_busy(void);                       /* bytes queued or still shifting out */
const glink_uart_stats_t *glink_uart_stats(void);        /* a snapshot taken with interrupts off */

#if defined(FC_SIMBENCH) && (GLINK_UART_BENCH_WIRE || !GLINK_UART_HW)
#define GLINK_UART_BENCH_Q 64u               /* the bench wire queue (power of two): two RESULT frames */
/* copies of the frames queued for sending (oldest first); *t_queued = timer_sim_now() when the newest of them was queued
 * (Timer1 counts since boot, FC_PWM_COUNTS_PER_US per us) */
uint8_t  glink_uart_bench_take_tx(uint8_t *b, uint8_t max, uint32_t *t_queued);
#endif
#if defined(FC_SIMBENCH) && GLINK_UART_BENCH_WIRE
/* NAV single-chip bench: n bytes onto the Timer3 wire towards RXD1 - the first completes delay_cycles from now (one byte
 * time .. 32 ms: Timer3 CTC at clk / 8), the others one byte time (640 cycles) apart; returns the bytes accepted (0 while
 * the wire still carries the previous frame: one frame at a time) */
uint8_t  glink_uart_bench_send(const uint8_t *b, uint8_t n, uint32_t delay_cycles);
bool     glink_uart_bench_busy(void);                    /* a frame is still on the bench wire */
#endif
#if defined(FC_SIMBENCH) && !GLINK_UART_HW
void     glink_uart_bench_inject(const uint8_t *b, uint8_t n);   /* GNS single-chip: bytes as if received */
#endif

#endif /* FC_GLINK_UART_H */
