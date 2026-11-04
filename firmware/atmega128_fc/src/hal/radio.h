/* radio.h - the LoRa ground-link radio (SPEC 10.3) of the COM (v0.6: the radio is wired to the
 * COM ATmega128A only, params.json flight_computer.com.pins; the FCU has no radio and its build
 * excludes both backends).  One interface, two compile-time backends selected by FC_LINK_RADIO
 * (params_gen.h from flight_computer.link.radio; a PlatformIO env may force the other one):
 *
 *   FC_RADIO_LORA_UART  hal/lora_uart.c  Ebyte E22/E220/E32-class transparent module on the COM's USART1
 *   FC_RADIO_SX127X     hal/sx127x.c     SX1276/77/78 on the COM's SPI (Ra-01/Ra-02, RFM95/96)
 *
 * The vehicle never transmits on its own: the COM calls radio_send() only with a TELEM built
 * from an FCU snapshot, and the FCU's supervisor produces a snapshot only as the reply to a
 * valid uplink frame, so uplink and downlink never meet on the half-duplex radio.  Received
 * bytes are returned raw (radio_read); the streaming decoder of core/link_proto.c finds the
 * frames, whatever way the radio chops them.  Timeouts count COM ticks (the tick argument of
 * radio_service, src/com/com_timer.c, COM_TICK_HZ).
 *
 * Invariant: every function returns within a bounded time (no waiting on the radio in the
 * COM main loop); radio_service() does at most one short step (<= ~0.25 ms) per call; the
 * only ISR work is moving UART bytes (lora_uart).
 */
#ifndef FC_RADIO_H
#define FC_RADIO_H

#include <stdbool.h>
#include <stdint.h>

#define RADIO_RSSI_NONE 127                 /* = LOG_RSSI_NONE: the backend reports no RSSI */

typedef struct {
    uint16_t rx_bytes;                      /* bytes received */
    uint16_t rx_overflow;                   /* bytes lost to a full receive ring */
    uint16_t rx_errors;                     /* UART framing/overrun; SX127x payload CRC error / oversize packet */
    uint16_t tx_frames;                     /* replies handed to the radio */
    uint16_t tx_timeouts;                   /* lora_uart: AUX never became ready; sx127x: no TxDone */
    int8_t   rssi_dbm;                      /* last packet, RADIO_RSSI_NONE if unknown */
    int8_t   snr_q4;                        /* last packet SNR x 4 (sx127x) */
    bool     ok;                            /* sx127x: RegVersion read back 0x12; lora_uart: always */
    uint8_t  version;                       /* sx127x RegVersion */
} radio_stats_t;

void    radio_pins_safe(void);              /* first thing at boot: module mode pins / chip select in a defined state */
void    radio_init(void);                   /* after sei(); may block a few ms (boot only) */
bool    radio_service(uint32_t tick);       /* one bounded step (tick = COM tick); true if it did bus work (call again) */
uint8_t radio_read(uint8_t *buf, uint8_t max);       /* received bytes since the last call */
bool    radio_send(const uint8_t *frame, uint8_t len);   /* queue one reply; false (dropped) if one is still going out */
bool    radio_tx_busy(void);
const radio_stats_t *radio_stats(void);
#ifdef FC_SIMBENCH
void    radio_inject_rx(const uint8_t *bytes, uint8_t n);   /* simbench: bytes as if received over the air */
#endif

#endif /* FC_RADIO_H */
