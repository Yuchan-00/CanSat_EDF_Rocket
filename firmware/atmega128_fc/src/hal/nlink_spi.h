/* nlink_spi.h - SPI driver of the NAV <-> FCU navigation link (SPEC 10.5, v0.6.3); the frames are
 * core/nlink_proto.h's business, this layer moves the 72-byte exchanges.  Role by -D FC_ROLE_NAV /
 * -D FC_ROLE_FCU (platformio.ini).
 *
 * Wire (board.h): NAV PB0 (SS out) -> FCU PB0 (SS in, pull-up), NAV PB1 SCK -> FCU PB1, NAV PB2 MOSI ->
 * FCU PB2, FCU PB3 MISO -> NAV PB3 (pull-up), common GND.  SPI mode 0, MSB first, NLINK_SPI_HZ
 * (params_gen.h: NLINK_SPI_SPR / NLINK_SPI_2X, flight_computer.nlink.spi_clock_hz = 2 MHz).  FCU PB1 is
 * also the FCU's ISP SCK: hold the NAV in reset or unplug the nlink connector to program the FCU.
 *
 * NAV (SPI master, no interrupt): nlink_spi_exchange pulls SS low, waits NLINK_BYTE_GAP_CYCLES, then for
 * each of the NL_XFER_BYTES bytes writes SPDR, busy-waits for SPIF, stores the returned byte and waits
 * NLINK_BYTE_GAP_CYCLES (the FCU's slave interrupt loads its next return byte in that gap), and releases
 * SS.  Interrupts stay enabled (an interrupt between two bytes only lengthens that gap).  ~630 us at
 * 2 MHz with a 4 us gap (NLINK_XFER_US).  PB0 is made an output (high) BEFORE the SPI is enabled as
 * master: an SS input that went low would drop the SPI out of master mode (datasheet, "SS Pin
 * Functionality").
 *
 * FCU (SPI slave, SPI_STC interrupt, hand-written: nlink_spi.c), per byte: SPDR <- reply byte k+1 of the
 * active reply buffer (its first NL_FCU_FRAME_BYTES bytes, then zeros), the received byte -> RX buffer,
 * count; on the NL_XFER_BYTES-th byte the exchange is complete: it becomes the ready frame (an untaken one
 * is replaced: overruns), frames++, the other RX buffer takes the next exchange, a pending reply becomes
 * active and its byte 0 is loaded.  The driver looks at no content: the caller runs nl_decode_nav, which
 * checks sync, TYPE and CRC.  Byte alignment (normative model: sim/nlink_proto.py SpiSlaveModel):
 *   - resync: nlink_spi_frame_ready / nlink_spi_take first look (interrupts off) for SS high with no byte
 *     waiting for its interrupt; then an open partial exchange (index != 0: a byte lost to interrupt
 *     latency, an extra SCK edge, an SS glitch, a NAV reset mid-frame) is discarded (resyncs++), a pending
 *     reply becomes active and byte 0 of the active reply is (re)loaded into SPDR - so byte 0 is right even
 *     if the SS rise after the last exchange reset the shift register.  The FCU waits for its next frame
 *     calling nlink_spi_frame_ready with SS high every tick, so an error costs the exchange it happened in
 *     (it never completes, or completes misaligned and fails the CRC) and at most one more if no poll came
 *     between them.  Call nlink_spi_frame_ready also when the frame timeout passes: it applies the same rule,
 *     and that is all a timeout needs - with SS still low no exchange can complete anyway, and every way the NAV
 *     can stop in the middle of one (reset, brown-out, a cut SS wire) lets the FCU's pull-up take SS high.
 *   - reply: nlink_spi_set_reply makes a new reply active at once when no exchange is in progress (index 0,
 *     SS high) and loads its byte 0; otherwise it is pending until the exchange in progress ends.  Call it
 *     once per tick, after the supervisor ran: the next exchange returns that state (one FCU tick old when
 *     the NAV uses it).  A slave interrupt later than the byte gap (another interrupt running, a long cli
 *     section) leaves the previous MOSI byte in the shift register: if that was reply byte 1..9 the NAV's
 *     CRC check refuses that FCU frame (the NAV keeps the last valid one); bytes 10..71 are not checked.
 *   - nlink_spi_take copies the newest complete exchange (interrupts on during the 72-byte copy: the ISR
 *     can only reach that buffer again after one more complete exchange, >= 3.4 ms later).
 *
 * Timing (avr-objdump of nlink_spi.o; simavr runs of this driver with the chip's SPI shift-register behaviour
 * modelled - two test firmwares, the master's byte timing as nlink_spi_exchange produces it):
 *   NAV  per byte: 64 cycles of transfer (2 MHz) + 75 cycles from SPIF to its next SPDR write (64-cycle gap +
 *        11 cycles of loop); SS low -> first byte 72 cycles; one exchange 10.1k cycles (630 us) without interrupts.
 *   FCU  slave interrupt 62 cycles per byte, its next return byte written 23 cycles after SPIF; the end of an
 *        exchange <= 138 cycles; ~4.5k cycles (280 us) per exchange.  Another interrupt or a cli section that is
 *        running when SPIF comes delays it.  Measured with a competing interrupt of known length against the
 *        master's 75-cycle gap: 58 cycles - every byte on time; 82 cycles - some return bytes late (the NAV refuses
 *        that FCU frame; harmless for bytes 10..71), received bytes intact up to 106 cycles; 130 cycles - received
 *        bytes overwritten by the next one (that exchange is lost: resync, the FCU's tick for it runs on the frame
 *        timeout with the navigation data invalid).  Keep every FCU interrupt and cli section under ~50 cycles
 *        around the exchange (ilink RX 51, log UDRE 46 cycles: at the edge).
 *
 * Single-chip simbench (FC_SIMBENCH without FC_COSIM): no SPI register is touched and no ISR is defined;
 * the FCU gets exchanges from nlink_spi_inject_frame (as if the NAV had made one) and nlink_spi_last_reply
 * gives the bytes that exchange returned; the NAV's exchanges go to nlink_spi_take_tx and return what
 * nlink_spi_inject_reply set last (zeros before) - like ilink_uart's bypass.  FC_COSIM (two chips in simavr)
 * uses the real hardware path; the harness must (1) drive FCU PB0 from NAV PB0 as an EXTERNAL level
 * (AVR_IOCTL_IOPORT_SET_EXTERNAL - otherwise simavr re-applies the FCU's own SS pull-up whenever the FCU writes
 * PORTB), (2) pass the NAV's SPI output byte to the FCU's SPI input and the FCU's byte back, taking the FCU's byte
 * as its SPDR was when the NAV wrote its SPDR (simavr's slave sends SPDR as it is at the END of the byte); the
 * driver reads SPDR before writing it, which simavr's single SPDR byte needs.
 *
 * Invariant: the FCU's ISR only moves bytes and pointers (bounded, nlink_spi.c gives its cycles); the NAV's
 * exchange is the only code that drives SS low and it always releases it.
 */
#ifndef FC_NLINK_SPI_H
#define FC_NLINK_SPI_H

#include <stdbool.h>
#include <stdint.h>

#include "../core/nlink_proto.h"
#include "../fc_mode.h"

typedef struct {
    uint16_t frames;                 /* FCU: complete exchanges received; NAV: exchanges made */
    uint16_t resyncs;                /* FCU: partial exchanges discarded (SS high with the index != 0); VTOL NAV (flight /
                                      * co-simulation builds): exchanges closed by the W1 watchdog (saturates at 255) */
    uint16_t overruns;               /* FCU: complete exchanges replaced before nlink_spi_take took them */
} nlink_spi_stats_t;

void nlink_spi_init(void);
const nlink_spi_stats_t *nlink_spi_stats(void);   /* a snapshot taken with interrupts off */

#if defined(FC_ROLE_NAV) && FC_VTOL
/* v0.9 timing proposal P1 (fw09/p1 prototype; nlink_spi.c "VTOL NAV"): the exchange runs from a Timer3 compare-B
 * interrupt, one per byte, the bytes and their spacing on the wire as the busy-wait master's (>= 75 cycles from the end
 * of a byte to the next SPDR write).  The frame is encoded into nlink_spi_buf; nlink_spi_exchange_start pulls SS low and
 * returns at once; the interrupt replaces every sent byte by the FCU's byte (in place) and releases SS after byte 71,
 * stamping Timer1.  nlink_spi_exchange_wait (never before the start) waits for the end and returns the FCU's bytes
 * (the single-chip bench: the bypass reply - its interrupt clocks the SPI the same way but leaves the frame in place for
 * the synthetic FCU) and the Timer1 count of the SS rise.
 * W1 (fw09 SPEC_firmware_v09 3.3, flight / co-simulation builds): the wait is bounded - an exchange still open
 * NLINK_WD_COUNTS Timer3 counts after its start (params flight_computer.nlink.exchange_watchdog_us) is closed by the wait
 * itself (compare off, SS high, its Timer1 stamp, a buffer nl_decode_fcu4 refuses, one trip in nlink_spi_stats()->resyncs):
 * a lost compare interrupt costs one FCU frame, never a stalled NAV.  Timer3 belongs to this driver in those builds:
 * nlink_spi_t3_owned (after every driver's init, before the first exchange) is true only if TCCR3A / TCCR3B are still
 * nlink_spi_init's (normal mode, clk / 1) and no Timer3 interrupt is enabled (bench builds: always true). */
extern uint8_t nlink_spi_buf[NL_XFER_BYTES];
void nlink_spi_exchange_start(void);
const uint8_t *nlink_spi_exchange_wait(uint16_t *t_ss_high);
bool nlink_spi_t3_owned(void);
#if defined(FC_SIMBENCH) && !defined(FC_COSIM)
void nlink_spi_inject_reply(const uint8_t rx[NL_XFER_BYTES]); /* what every following exchange returns */
const uint8_t *nlink_spi_take_tx_buf(void);                   /* the newest exchange's MOSI bytes (NULL: none new) ... */
uint8_t *nlink_spi_reply_buf(void);                           /* ... and the reply buffer, encoded in place */
#endif

#elif defined(FC_ROLE_NAV)
/* SS low, NL_XFER_BYTES bytes with NLINK_BYTE_GAP_CYCLES gaps, SS high; interrupts stay enabled */
void nlink_spi_exchange(const uint8_t tx[NL_XFER_BYTES], uint8_t rx[NL_XFER_BYTES]);
#if defined(FC_SIMBENCH) && !defined(FC_COSIM)
bool nlink_spi_take_tx(uint8_t tx[NL_XFER_BYTES]);           /* the newest exchange's MOSI bytes (true once per exchange) */
void nlink_spi_inject_reply(const uint8_t rx[NL_XFER_BYTES]); /* what every following exchange returns */
const uint8_t *nlink_spi_take_tx_buf(void);                   /* the same in place (NULL: no new exchange) ... */
uint8_t *nlink_spi_reply_buf(void);                           /* ... and the reply buffer, encoded in place */
#endif

#elif defined(FC_ROLE_FCU)
/* the FCU frame (nl_encode_fcu's first NL_FCU_FRAME_BYTES bytes) returned by the next exchange; the rest are 0x00 */
void nlink_spi_set_reply(const uint8_t tx[NL_FCU_FRAME_BYTES]);
bool nlink_spi_frame_ready(void);                             /* resync check, then: a complete exchange waits */
bool nlink_spi_take(uint8_t rx[NL_XFER_BYTES]);               /* resync check, then: copy the newest one (true) */
#if defined(FC_SIMBENCH) && !defined(FC_COSIM)
void nlink_spi_inject_frame(const uint8_t rx[NL_XFER_BYTES]); /* one complete exchange with these MOSI bytes */
uint8_t *nlink_spi_inject_buf(void);                          /* the buffer the next injection fills: a frame built there
                                                              * in place is injected without a copy */
void nlink_spi_last_reply(uint8_t tx[NL_XFER_BYTES]);         /* the FCU frame (first NL_FCU_FRAME_BYTES MISO bytes) of
                                                              * the last injected exchange */
#endif

#else
#error "nlink_spi: build with -D FC_ROLE_NAV or -D FC_ROLE_FCU (platformio.ini)"
#endif

#endif /* FC_NLINK_SPI_H */
