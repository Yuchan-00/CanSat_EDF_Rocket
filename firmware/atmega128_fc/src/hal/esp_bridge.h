/* esp_bridge.h - ESP8266 Wi-Fi bridge (ESP-12E running firmware/esp_bridge: a transparent UART <-> ESP-NOW bridge) on the
 * COM's USART1: backend FC_RADIO_ESP_BRIDGE of hal/radio.h (2026-11-06 firmware round; params.json
 * flight_computer.link.vehicle_radio = "esp_bridge", settings flight_computer.link.esp_bridge).
 *
 *   PD2 RXD1  <- ESP TXD0 (GPIO1)   VTOL carrier: through BSS138 channel Q11 (rev 2c); rocket COM board: README 4.2
 *   PD3 TXD1  -> ESP RXD0 (GPIO3)   through a BSS138 channel (Q4) and 1 kOhm (3.3 V side)
 *   PC6       -> ESP RST            through a BSS138 channel (Q5; its 3.3 V pull-up is the RST pull-up): non-inverting,
 *                                   open-drain use (2026-11-06 review F1): PC6 driven low = ESP in reset, PC6 input (Hi-Z,
 *                                   the channel's pull-ups) = running; never driven high, so a manual RST-GND short (carrier
 *                                   J3 5-6, the rocket's programming header) cannot short a driven pin (params
 *                                   flight_computer.com.pins.lora_reset: the same pin as the SX127x RESET - one radio is fitted)
 *   (PC3 M0, PC4 M1, PC5 AUX of the Ebyte backend are not used: no mode pins, no busy line)
 *
 * The frame protocol is untouched (core/link_proto*, the COM's decoder): the ESP pair only carries bytes.  What the backend
 * does instead of the Ebyte M0 / M1 / AUX handling:
 *   - USART1 at ESP_UART_BAUD 8N1, U2X = 0 (params link.esp_bridge.uart_baud; 38400: UBRR 25, +0.16 %; the ESP side's
 *     divider of its 80 MHz UART clock is +0.016 %, tools/gen_firmware_params.py checks both against
 *     link.esp_bridge.uart_max_error).  U2X = 0 as on the ilink: simavr's USART model ignores U2X.
 *   - reset, cold COM boot only (power-on, or no reset flag): radio_pins_safe() drives PC6 low (the ESP is held in reset
 *     from the COM's first instructions); radio_init() keeps it low for ESP_RESET_HOLD_MS (watchdog kicked) and releases
 *     it (input).  After a COM watchdog / brown-out / external reset (MCUCSR without PORF) the ESP is left running: PC6
 *     stays an input, no hold, no boot window (review F2: a COM restart in flight costs the link what it cost with the
 *     Ebyte module, which a COM reset never touched - co-simulation row 18).  A hung ESP is restarted by its own
 *     watchdog (A) or a power cycle, not by a COM reset.
 *   - boot garbage: after the release the ESP8266 ROM prints its boot log at 74880 baud on TXD0 (A: Espressif; cannot be
 *     switched off without strapping changes).  The RX interrupt drops every byte that arrives in the first
 *     ESP_BOOT_TICKS COM ticks after the release (counted, esp_bridge_rx_discarded(); framing errors in that time are not
 *     counted as errors; the simbench's injected bytes bypass it, see radio_inject_rx).  After that window every byte goes to
 *     the link decoder, which skips anything that is not a valid frame (sync, LEN, CRC, vehicle id) - a later ESP restart in flight costs only garbage the decoder skips.  The ESP sends
 *     no "ready" byte (it stays byte-transparent): a request the ground sends before the bridge is up is simply not
 *     answered, exactly like a lost radio packet.
 *   - no busy line: a reply is handed to the UART at once (the ground's half-duplex polling already paces the frames:
 *     one request in flight, the vehicle answers only it); tx_timeouts stays 0.  The ESP-NOW side has its own MAC-layer
 *     ACK / retry; the vehicle side is the same as an Ebyte module with AUX idle.
 *   - no RSSI (RADIO_RSSI_NONE) and IL_STAT shows the backend as "not sx127x" (sim/ilink_proto.py STAT_SX127X clear,
 *     unchanged protocol).
 *   - radio_stats()->ok (IL_STAT_RADIO_OK, the COM LED; 2026-11-10 control round, review F4 - it was always true): set
 *     from the first byte the ESP put on the COM's RXD1 since this COM boot (the ROM boot log inside the boot window
 *     after a cold boot, any link byte after it or after a warm boot) and never cleared.  It says "an ESP is on the
 *     line and its UART runs", nothing about the ESP-NOW link (only the ground's replies show that); an absent or
 *     dead ESP keeps it clear (the LED blinks "radio not ok", the FCU logs it - no flight decision uses it).
 *
 * Invariant: the ISRs only move (or, inside the boot window, drop) bytes; radio_send copies a whole frame or nothing;
 * every function returns within a bounded time.
 */
#ifndef FC_ESP_BRIDGE_H
#define FC_ESP_BRIDGE_H

#include "radio.h"

#define ESP_BRIDGE_RX_BUF 64u                /* power of two; 16.7 ms of 38400-baud bytes (the COM main loop reads every pass) */
#define ESP_BRIDGE_TX_BUF 64u                /* power of two; > the longest reply (TELEM v5, 47 B) */

uint16_t esp_bridge_rx_discarded(void);      /* bytes dropped inside the boot window since boot (debugger / bench) */

#endif /* FC_ESP_BRIDGE_H */
