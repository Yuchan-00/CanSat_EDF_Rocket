/* bridge_pins.h - wiring of the SX1276-class module to the Arduino Nano / Uno (not in params.json: this is
 * the ground bridge's own board, see README.md).  Hardware SPI: D13 SCK, D12 MISO, D11 MOSI.
 *
 * Level shifting (SPEC 10.3 rule, same as on the vehicle): the module is a 3.3 V part.  Every Nano -> module
 * line (NSS, SCK, MOSI, RESET) goes through a BSS138 channel to 3.3 V; module -> Nano lines (MISO, DIO0) go
 * direct (3.3 V >= the ATmega328P's 0.6 VCC = 3.0 V input-high threshold at 5 V).  The module's VCC comes
 * from a separate 3.3 V regulator (the Nano's 3V3 pin cannot supply the transmit current).
 */
#ifndef BRIDGE_PINS_H
#define BRIDGE_PINS_H

#define PIN_LORA_NSS    10   /* D10 -> BSS138 -> module NSS */
#define PIN_LORA_RESET  9    /* D9  -> BSS138 -> module RESET */
#define PIN_LORA_DIO0   2    /* D2 (INT0) <- module DIO0 (direct) */

/* SPI clock.  A BSS138 channel's rising edge on the 3.3 V side is its 10 kOhm pull-up into the module's input
 * and wiring capacitance (tens of pF): 0.2-0.5 us, so the LoRa library's default 8 MHz is far too fast.
 * 1 MHz is ASSUMED safe for 10 kOhm modules (measure SCK on a scope if in doubt).  With a 74LVC125 /
 * 74AHC125 buffer at 3.3 V instead of the BSS138 board, the library default would work. */
#define BRIDGE_SPI_HZ   1000000UL

/* boot without an answering SX127x: repeat the ASCII error line this often (human-readable only) */
#define BRIDGE_NOT_FOUND_REPEAT_MS  1000UL

#endif /* BRIDGE_PINS_H */
