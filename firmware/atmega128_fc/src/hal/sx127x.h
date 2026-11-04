/* sx127x.h - SX1276/77/78 LoRa transceiver on the COM's SPI port (backend FC_RADIO_SX127X of
 * hal/radio.h, SPEC 10.3; v0.6: COM firmware only, pins from params.json
 * flight_computer.com.pins).  Ra-01/Ra-02 (SX1278, 410-525 MHz) and RFM95/96 (SX1276).
 *
 *   PB0 NSS, PB1 SCK, PB2 MOSI -> module   through BSS138 channels to 3.3 V
 *   PC6 RESET                  -> module   through a BSS138 channel (low = reset; high = released,
 *                                          the module's own pull-up holds it)
 *   PB3 MISO, PE7 DIO0         <- module   direct (3.3 V >= the AVR's 3.0 V V_IH at 5 V)
 *
 * SPI master mode 0, MSB first, F_CPU / 2^n <= fc_config link.sx127x_spi_hz_max (the
 * BSS138 pull-up rise time limits it).  LoRa mode with every register from params_gen.h
 * (generated from flight_computer.link.sx127x): FRF, PA config, LNA, BW / CR / explicit
 * header, SF / payload CRC on, LDRO / AGC, preamble, sync word; FIFO TX base 0x80,
 * RX base 0x00.  The radio sits in continuous RX; DIO0 (mapped to RxDone) is polled by
 * the COM main loop; on RxDone the payload is read in chunks (a payload-CRC error is
 * counted and dropped) together with the packet RSSI / SNR.  A reply switches the radio
 * to standby, loads the FIFO in chunks, starts TX with DIO0 mapped to TxDone, and goes
 * back to continuous RX on TxDone or after SX127X_TX_TIMEOUT_TICKS COM ticks (counted).
 *
 * Invariant: no step waits for the radio; each service step is one register access
 * or one FIFO chunk (<= SX127X_CHUNK bytes, ~0.25 ms at 500 kHz).
 */
#ifndef FC_SX127X_H
#define FC_SX127X_H

#include "radio.h"

#define SX127X_CHUNK 12u                     /* FIFO bytes per background step */
#define SX127X_RX_BUF 64u                    /* received bytes waiting for the decoder (power of two; >= one packet) */
#define SX127X_VERSION_EXPECTED 0x12u        /* RegVersion of the SX1276/77/78 */

/* registers (datasheet section 6.4, LoRa mode) */
#define SX_REG_FIFO 0x00
#define SX_REG_OP_MODE 0x01
#define SX_REG_FRF_MSB 0x06
#define SX_REG_FRF_MID 0x07
#define SX_REG_FRF_LSB 0x08
#define SX_REG_PA_CONFIG 0x09
#define SX_REG_LNA 0x0C
#define SX_REG_FIFO_ADDR_PTR 0x0D
#define SX_REG_FIFO_TX_BASE 0x0E
#define SX_REG_FIFO_RX_BASE 0x0F
#define SX_REG_FIFO_RX_CURRENT 0x10
#define SX_REG_IRQ_FLAGS_MASK 0x11
#define SX_REG_IRQ_FLAGS 0x12
#define SX_REG_RX_NB_BYTES 0x13
#define SX_REG_PKT_SNR 0x19
#define SX_REG_PKT_RSSI 0x1A
#define SX_REG_MODEM_CONFIG1 0x1D
#define SX_REG_MODEM_CONFIG2 0x1E
#define SX_REG_PREAMBLE_MSB 0x20
#define SX_REG_PREAMBLE_LSB 0x21
#define SX_REG_PAYLOAD_LENGTH 0x22
#define SX_REG_PAYLOAD_MAX 0x23
#define SX_REG_MODEM_CONFIG3 0x26
#define SX_REG_SYNC_WORD 0x39
#define SX_REG_DIO_MAPPING1 0x40
#define SX_REG_VERSION 0x42
#define SX_MODE_LORA 0x80
#define SX_MODE_SLEEP 0x00
#define SX_MODE_STDBY 0x01
#define SX_MODE_TX 0x03
#define SX_MODE_RX_CONT 0x05
#define SX_IRQ_RX_DONE 0x40
#define SX_IRQ_CRC_ERROR 0x20
#define SX_IRQ_TX_DONE 0x08
#define SX_DIO0_RX_DONE 0x00                 /* RegDioMapping1 bits 7:6 */
#define SX_DIO0_TX_DONE 0x40
#define SX_FIFO_TX_BASE 0x80
#define SX_FIFO_RX_BASE 0x00

#endif /* FC_SX127X_H */
