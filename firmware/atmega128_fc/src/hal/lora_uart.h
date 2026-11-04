/* lora_uart.h - Ebyte E22 / E220 / E32-class LoRa module in transparent mode on
 * the COM's USART1 (backend FC_RADIO_LORA_UART of hal/radio.h, SPEC 10.3; v0.6: COM
 * firmware only, pins from params.json flight_computer.com.pins; the COM's USART0 is the
 * ilink to the FCU).
 *
 *   PD2 RXD1  <- module TXD   direct (3.3 V >= the AVR's 3.0 V V_IH at 5 V)
 *   PD3 TXD1  -> module RXD   through a BSS138 channel to 3.3 V
 *   PC3 M0, PC4 M1 -> module  through BSS138 channels, driven LOW = normal (transparent) mode
 *   PC5 AUX   <- module       direct, input with the internal pull-up (an unconnected AUX reads "ready")
 *
 * USART1 runs LINK_UART_BAUD 8N1 (params flight_computer.link.uart_baud).  The RX ISR
 * stores every byte in a 64-byte ring (framing / overrun errors are counted and the
 * byte dropped); the UDRE ISR empties a 64-byte TX ring.  Channel, air rate and
 * address are set beforehand with the vendor tool - the MCU only moves bytes.  A
 * reply is started when AUX is high (module idle) or LINK_AUX_WAIT_TICKS COM ticks after
 * it was queued (counted as tx_timeouts), so a missing or stuck AUX line can delay a
 * reply but never block the loop.  The module reports no RSSI in transparent mode
 * (RADIO_RSSI_NONE).
 *
 * Invariant: the ISRs only move bytes; radio_send copies a whole frame or nothing.
 */
#ifndef FC_LORA_UART_H
#define FC_LORA_UART_H

#include "radio.h"

#define LORA_UART_RX_BUF 32u                /* power of two; ~32 ms of 9600-baud traffic (the COM main loop reads every pass) */
#define LORA_UART_TX_BUF 64u                /* power of two; >= one TELEM frame (36 bytes) */

#endif /* FC_LORA_UART_H */
