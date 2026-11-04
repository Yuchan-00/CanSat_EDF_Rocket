/* USB <-> LoRa ground bridge (SPEC 10.3, sx127x backend) - Arduino Nano / Uno + SX1276-class module.
 *
 * Byte-transparent in both directions:
 *   USB serial -> radio : the bytes the ground station writes are sent as LoRa packets unchanged.  A complete
 *                         link frame (A5 5A LEN ... CRC) goes out as one packet as soon as its last byte is in;
 *                         anything else goes out after BRIDGE_IDLE_FLUSH_US of serial silence (an incomplete
 *                         frame after BRIDGE_FRAME_WAIT_US).  Nothing is checked, changed or dropped: the
 *                         vehicle's decoder checks CRC / length / vehicle id.
 *   radio -> USB serial : every received packet (payload CRC good - checked by the SX127x) is written to the
 *                         serial port unchanged.
 * v0.9 (fw09 RP1): the tether / hybrid builds answer with TELEM v4 / v5 (TYPE 0x23 / 0x24, 41 / 47-byte frames); the
 * bridge passes them like every other packet.  The only frame knowledge here is the uplink framing (frame_bytes: LEN
 * in LINK_LEN_MIN..LINK_LEN_MAX), and LINK_LEN_MAX / LINK_MAX_FRAME_BYTES come from sim/link_proto.py through the
 * generated header (LEN_MAX 42, 47-byte buffer since RP1) - no code change, the header is regenerated.
 * The ground station is the link master (it sends only when no reply is outstanding), so the half-duplex
 * radio never has to transmit while a reply is arriving.
 *
 * The serial port carries only radio bytes.  If the SX127x does not answer at boot, the bridge writes the ASCII
 * line "BRIDGE: SX127x not found" once a second (it contains no 0xA5 byte, so the ground station's decoder
 * skips it as noise; any serial terminal shows it).
 *
 * Every number comes from bridge_config_gen.h (generated from params.json) or bridge_pins.h (this board).
 */
#include <Arduino.h>
#include <LoRa.h>
#include <SPI.h>
#include <string.h>

#include "bridge_config_gen.h"
#include "bridge_pins.h"

static uint8_t up_buf[LINK_MAX_FRAME_BYTES];
static uint8_t up_n = 0;
static uint32_t t_last_rx_us = 0;

/* Bytes of the complete frame at the start of up_buf, 0 when up_buf does not start with a valid frame header. */
static uint8_t frame_bytes(void)
{
    if (up_n < 3 || up_buf[0] != LINK_SYNC0 || up_buf[1] != LINK_SYNC1) {
        return 0;
    }
    uint8_t len = up_buf[2];
    if (len < LINK_LEN_MIN || len > LINK_LEN_MAX) {
        return 0;
    }
    return (uint8_t)(len + LINK_FRAME_OVERHEAD);
}

static void radio_send(uint8_t n)
{
    LoRa.beginPacket(0);            /* explicit header */
    LoRa.write(up_buf, n);
    LoRa.endPacket(false);          /* blocking until TX done; parsePacket() below re-enters receive */
    memmove(up_buf, up_buf + n, up_n - n);
    up_n = (uint8_t)(up_n - n);
}

void setup()
{
    Serial.begin(BRIDGE_SERIAL_BAUD);
    LoRa.setPins(PIN_LORA_NSS, PIN_LORA_RESET, PIN_LORA_DIO0);
    LoRa.setSPIFrequency(BRIDGE_SPI_HZ);
    while (!LoRa.begin(BRIDGE_LORA_FREQ_HZ)) {
        Serial.print(F("BRIDGE: SX127x not found\r\n"));
        delay(BRIDGE_NOT_FOUND_REPEAT_MS);
    }
    LoRa.setSpreadingFactor(BRIDGE_LORA_SF);
    LoRa.setSignalBandwidth(BRIDGE_LORA_BW_HZ);
    LoRa.setCodingRate4(BRIDGE_LORA_CR_DENOM);
    LoRa.setSyncWord(BRIDGE_LORA_SYNC_WORD);
    LoRa.setPreambleLength(BRIDGE_LORA_PREAMBLE);
    LoRa.setTxPower(BRIDGE_LORA_TX_POWER_DBM);   /* PA_BOOST output (RFM95W, Ra-01H) */
    LoRa.enableCrc();
    LoRa.disableInvertIQ();
}

void loop()
{
    /* USB serial -> radio */
    while (Serial.available() > 0 && up_n < sizeof(up_buf)) {
        up_buf[up_n++] = (uint8_t)Serial.read();
        t_last_rx_us = micros();
    }
    if (up_n > 0) {
        uint8_t need = frame_bytes();
        uint32_t quiet_us = micros() - t_last_rx_us;
        if (need > 0 && up_n >= need) {
            radio_send(need);
        } else if (up_n == sizeof(up_buf) || quiet_us >= (need > 0 ? BRIDGE_FRAME_WAIT_US : BRIDGE_IDLE_FLUSH_US)) {
            radio_send(up_n);
        }
    }
    /* radio -> USB serial */
    int n = LoRa.parsePacket();
    if (n > 0) {
        while (LoRa.available() > 0) {
            Serial.write((uint8_t)LoRa.read());
        }
    }
}
