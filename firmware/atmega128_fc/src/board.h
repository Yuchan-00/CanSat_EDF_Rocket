/* board.h - pin maps of the three ATmega128A of the flight computer (TQFP-64, 16 MHz crystal, 5 V).
 *
 * v0.6.3 (params.json flight_computer.architecture, SPEC 10.4 / 10.5): every flight computation runs on an
 * ATmega128A, split over three chips built from this one source tree (platformio.ini):
 *   NAV  -D FC_ROLE_NAV  MPU-6050 + MS5611 on I2C, pad alignment / gyro-only attitude estimator, tilt, altitude
 *                        filter, sensor health; pushes the navigation frame to the FCU every tick (nlink SPI
 *                        master).  No flight decisions.
 *   FCU  -D FC_ROLE_FCU  onboard supervisor (commands + interlocks I1-I10), attitude control, PWM outputs,
 *                        OpenLog log; nlink SPI slave (its tick starts on the NAV's frame), ilink to the COM.
 *                        Every flight decision.
 *   COM  -D FC_ROLE_COM  LoRa radio driver, uplink decoding, TELEM building from the FCU's snapshot, status.
 *                        No flight decisions.
 *
 * Every assignment below comes from params.json - FCU: flight_computer.pins, NAV: flight_computer.nav.pins,
 * COM: flight_computer.com.pins - through src/params_gen.h.  The generator checks each chip on its own against
 * the ATmega128 alternate-function table, refuses a pin used twice on the same chip, an ilink that is not on that
 * chip's USART and an nlink that is not on the SPI pins (PB0 = SS on both ends); this file only documents the pins
 * and turns the port letters into register names.  The same pin name on another chip is a different pin.
 *
 * ---- FCU (flight control unit: ilink on USART1, nlink SPI slave) ------------------------------------------------
 *  Signal              Pin        Function / level / notes
 *  ------------------  ---------  -------------------------------------------------------------
 *  vane 1 servo        PB5 OC1A   Timer1 fast PWM mode 14, 50 Hz, 5 V logic -> MG90S signal
 *  vane 2 servo        PB6 OC1B   Timer1
 *  vane 3 servo        PB7 OC1C   Timer1
 *  vane 4 servo        PE3 OC3A   Timer3 (same TOP / prescaler as Timer1)
 *  ESC throttle        PE4 OC3B   Timer3, 1000 us = off at boot and in every state except POWERED
 *  release servo       PE5 OC3C   Timer3, latched / released positions from fc_config.json
 *  nlink SS            PB0 SS     input with pull-up <- NAV PB0 (low during an exchange; an absent / resetting
 *                                 NAV leaves it high: the slave ignores SCK)
 *  nlink SCK           PB1 SCK    input <- NAV PB1.  PB1 is also the FCU's ISP SCK: hold the NAV in reset or
 *                                 unplug the nlink connector to program the FCU
 *  nlink MOSI          PB2 MOSI   input <- NAV PB2 (navigation frame)
 *  nlink MISO          PB3 MISO   output -> NAV PB3 (FCU state bytes)
 *  log out             PE1 TXD0   USART0 57600 8N1 -> OpenLog RXI.  PE0/PE1 are also the ISP
 *                                 PDI/PDO pins: unplug the OpenLog while programming the FCU
 *  battery voltage     PF0 ADC0   divider, internal 2.56 V reference (100 nF on AREF)
 *  status LED          PC0        active high, 1k resistor
 *  buzzer              PC1        active high into an NPN driver (5 V active buzzer)
 *  arm switch          PC2        to GND when armed, internal pull-up
 *  ilink RX            PD2 RXD1   <- COM PE1 TXD0, direct (ILINK_BAUD 8N1)
 *  ilink TX            PD3 TXD1   -> 1 kOhm series resistor -> COM PE0 RXD0
 *  ISP                 PE0/PE1/PB1/RESET  PDI / PDO / SCK - NOT PB2/PB3 on the ATmega128
 *  (v0.6.3: no I2C, no sensor on the FCU - PD0/PD1/PE6 are free)
 *
 * ---- NAV (navigation computer: sensors on TWI, nlink SPI master) ------------------------------------------------
 *  I2C SCL / SDA       PD0 / PD1  TWI 400 kHz, 5 V side of the BSS138 shifter; 3.3 V side to
 *                                 GY-521 (MPU-6050 0x68) and GY-63 (MS5611 0x77)
 *  MPU INT (optional)  PE6 INT6   not used by the baseline firmware (polled at the 250 Hz tick)
 *  nlink SS            PB0        output, high = idle (made an output BEFORE the SPI is enabled as master)
 *  nlink SCK           PB1 SCK    output -> FCU PB1 (also the NAV's own ISP SCK)
 *  nlink MOSI          PB2 MOSI   output -> FCU PB2
 *  nlink MISO          PB3 MISO   input with pull-up <- FCU PB3 (0xFF when no FCU answers)
 *  status LED          PC1        active high, 1k resistor (NAV_LED_* below)
 *  ISP                 PE0/PE1/PB1/RESET  while the NAV is held in reset its PB0-PB2 float: the FCU's SS
 *                                 pull-up keeps the FCU's SPI idle while the programmer clocks PB1
 *
 * ---- COM (LoRa link processor, ilink on USART0) -----------------------------------------------------------------
 *  ilink RX            PE0 RXD0   <- 1 kOhm <- FCU PD3 TXD1.  PE0 is also the COM's ISP PDI:
 *                                 unplug the ilink connector to program the COM (left plugged by
 *                                 mistake, the resistor caps the FCU-vs-programmer current at 5 mA)
 *  ilink TX            PE1 TXD0   -> FCU PD2 RXD1, direct.  Also the COM's ISP PDO
 *  status LED          PC0        active high, 1k resistor (COM_LED_* below)
 *  --- LoRa ground link (SPEC 10.3), backend lora_uart (Ebyte transparent module) ---
 *  link RX             PD2 RXD1   <- module TXD, direct
 *  link TX             PD3 TXD1   -> module RXD through a BSS138 channel
 *  M0 / M1             PC3 / PC4  -> module through BSS138 channels, driven low = normal mode
 *  AUX                 PC5        <- module, direct, input with pull-up
 *  --- backend sx127x (SX1276/77/78 on SPI) ---
 *  NSS / SCK / MOSI    PB0/PB1/PB2 -> module through BSS138 channels (NSS is the SPI SS pin: an
 *                                 output, so the SPI stays master)
 *  MISO                PB3        <- module, direct
 *  DIO0                PE7        <- module, direct (polled)
 *  RESET               PC6        -> module through a BSS138 channel
 *  ISP                 PE0/PE1/PB1/RESET  PB1 is also the radio SCK: the shifter pull-up keeps
 *                                 NSS high while the COM is held in reset for programming
 *
 * ---- nlink wiring (flight_computer.nlink: SPI mode 0, NLINK_SPI_HZ, NLINK_BYTE_GAP_US; SPEC 10.5) --------------
 *   NAV PB0 SS   ------------> FCU PB0 SS      (FCU pull-up: idle high)
 *   NAV PB1 SCK  ------------> FCU PB1 SCK     (FCU ISP SCK: hold the NAV in reset or unplug the nlink
 *                                               connector to program the FCU)
 *   NAV PB2 MOSI ------------> FCU PB2 MOSI
 *   NAV PB3 MISO <------------ FCU PB3 MISO    (NAV pull-up)
 *   GND -------------------- GND               (common ground; both 5 V parts on the 5 V logic UBEC, no shifter)
 *   A dead NAV or a cut wire: no valid frame, the FCU's tick runs on its frame timeout with imu_ok = baro_ok =
 *   false -> I9 after the supervisor's IMU failure time (SPEC 10.5).
 *
 * ---- ilink wiring (flight_computer.ilink: ILINK_BAUD 8N1, U2X = 0) ----------------------------------------------
 *   FCU PD3 TXD1 --[1 kOhm]--> COM PE0 RXD0      (series_resistor_ohm; COM ISP PDI)
 *   FCU PD2 RXD1 <------------ COM PE1 TXD0      (COM ISP PDO)
 *   GND -------------------- GND                (common ground)
 *   Both chips are 5 V parts on the 5 V logic UBEC: no level shifter on the ilink.
 *   A dead COM or a broken wire is a lost radio link for the FCU's supervisor (I1p / I1).
 *
 * ---- v0.8.1: GNS (the fourth ATmega128A: GPS receiver, UBX, GPS filter, position controller; -D FC_ROLE_GNS) ------
 * (params flight_computer.gns.pins, electronics.gns, electronics.gps; src/gns/gns_main.c)
 *  GPS RX              PD2 RXD1   <- NEO-6M TX through the GNS's BSS138 channel (GPS_BAUD 8N1 after the boot config)
 *  GPS TX              PD3 TXD1   -> BSS138 -> NEO-6M RX (3.3 V side: the receiver's RX is not 5 V tolerant)
 *  glink RX            PE0 RXD0   <- NAV PD3 TXD1 (GLINK_BAUD 8N1).  PE0 is the GNS's ISP PDI
 *  glink TX            PE1 TXD0   -> NAV PD2 RXD1.  PE1 is the GNS's ISP PDO: unplug the glink to program the GNS
 *  status LED          PC0        active high, 1k resistor (GNS_LED_* below)
 *  (the NAV's USART1 PD2 / PD3 is the glink since v0.8.1 - the GPS lead goes to the GNS)
 *
 * ---- glink wiring (flight_computer.glink: GLINK_BAUD 8N1, U2X = 0; core/glink_proto.h) ----------------------------
 *   NAV PD3 TXD1 ------------> GNS PE0 RXD0     (GNS ISP PDI)
 *   NAV PD2 RXD1 <------------ GNS PE1 TXD0     (GNS ISP PDO)
 *   GND -------------------- GND                (common ground; both 5 V parts on the 5 V logic UBEC, no shifter)
 *   A dead GNS or a cut wire: no RESULT frames, the NAV sends SLOT_EMPTY, the FCU falls back to the TRIM after 2 cycles
 *   and raises its own GPS_LOST (V11 LAND) after land_steps cycles (core/nlink_proto.h nl_gns_rx).
 */
#ifndef FC_BOARD_H
#define FC_BOARD_H

#include <avr/io.h>

#include "params_gen.h"

#if (defined(FC_ROLE_FCU) + defined(FC_ROLE_COM) + defined(FC_ROLE_NAV) + defined(FC_ROLE_GNS)) > 1
#error "build exactly one role: FC_ROLE_NAV, FC_ROLE_FCU, FC_ROLE_COM or FC_ROLE_GNS (platformio.ini)"
#endif

#define FC_CAT2(a, b) a##b
#define FC_CAT(a, b) FC_CAT2(a, b)

/* FCU */
#define LED_PORT    FC_CAT(PORT, PIN_LED_PORT)
#define LED_DDR     FC_CAT(DDR, PIN_LED_PORT)
#define BUZZER_PORT FC_CAT(PORT, PIN_BUZZER_PORT)
#define BUZZER_DDR  FC_CAT(DDR, PIN_BUZZER_PORT)
#define ARM_SW_PORT FC_CAT(PORT, PIN_ARM_SW_PORT)
#define ARM_SW_DDR  FC_CAT(DDR, PIN_ARM_SW_PORT)
#define ARM_SW_PINR FC_CAT(PIN, PIN_ARM_SW_PORT)

/* NAV */
#define NAV_LED_PORT     FC_CAT(PORT, PIN_NAV_LED_PORT)
#define NAV_LED_DDR      FC_CAT(DDR, PIN_NAV_LED_PORT)
#define NAV_LED_PINR     FC_CAT(PIN, PIN_NAV_LED_PORT)

/* GNS (v0.8.1): params flight_computer.gns.pins.led = "PC0"; tools/gen_firmware_params.py emits no PIN_GNS_LED_* yet, so
 * the pin is taken from that params entry here (the generator's pin check does not cover the GNS - a request) */
#ifndef PIN_GNS_LED_PORT
#define PIN_GNS_LED_PORT C
#define PIN_GNS_LED_BIT  0
#endif
#define GNS_LED_PORT     FC_CAT(PORT, PIN_GNS_LED_PORT)
#define GNS_LED_DDR      FC_CAT(DDR, PIN_GNS_LED_PORT)

/* COM */
#define COM_LED_PORT     FC_CAT(PORT, PIN_COM_LED_PORT)
#define COM_LED_DDR      FC_CAT(DDR, PIN_COM_LED_PORT)
#define COM_LED_PINR     FC_CAT(PIN, PIN_COM_LED_PORT)

#define LINK_M0_PORTR    FC_CAT(PORT, PIN_LINK_M0_PORT)
#define LINK_M0_DDR      FC_CAT(DDR, PIN_LINK_M0_PORT)
#define LINK_M1_PORTR    FC_CAT(PORT, PIN_LINK_M1_PORT)
#define LINK_M1_DDR      FC_CAT(DDR, PIN_LINK_M1_PORT)
#define LINK_AUX_PORTR   FC_CAT(PORT, PIN_LINK_AUX_PORT)
#define LINK_AUX_DDR     FC_CAT(DDR, PIN_LINK_AUX_PORT)
#define LINK_AUX_PINR    FC_CAT(PIN, PIN_LINK_AUX_PORT)
#define LORA_NSS_PORTR   FC_CAT(PORT, PIN_LORA_NSS_PORT)
#define LORA_NSS_DDR     FC_CAT(DDR, PIN_LORA_NSS_PORT)
#define LORA_RESET_PORTR FC_CAT(PORT, PIN_LORA_RESET_PORT)
#define LORA_RESET_DDR   FC_CAT(DDR, PIN_LORA_RESET_PORT)
#define LORA_DIO0_PORTR  FC_CAT(PORT, PIN_LORA_DIO0_PORT)
#define LORA_DIO0_DDR    FC_CAT(DDR, PIN_LORA_DIO0_PORT)
#define LORA_DIO0_PINR   FC_CAT(PIN, PIN_LORA_DIO0_PORT)

#if !defined(__AVR_ATmega128__) && !defined(__AVR_ATmega128A__)
#error "this firmware targets the ATmega128/ATmega128A"
#endif

#endif /* FC_BOARD_H */
