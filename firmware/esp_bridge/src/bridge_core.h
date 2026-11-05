/* bridge_core.h - the portable part of the ESP8266 UART <-> ESP-NOW bridge (firmware/esp_bridge, README.md).
 *
 * No Arduino and no SDK header here: this unit is compiled by the bridge sketch (src/main.cpp) and by the host test
 * (host_test/test_bridge_core.cpp, zig c++ through tools/test_esp_bridge.py).  Two pieces:
 *
 *   BridgeChunker  UART -> air.  Collects the bytes the UART delivers and says when to send them as one ESP-NOW
 *                  packet.  The rule is the one of the USB <-> LoRa ground bridge (firmware/ground_bridge/src/main.cpp):
 *                  a complete link frame (A5 5A LEN ... CRC, LEN in BRIDGE_LEN_MIN..BRIDGE_LEN_MAX) at the start of
 *                  the buffer goes out as soon as its last byte is in; any other bytes go out after idle_us of UART
 *                  silence; an incomplete frame after frame_wait_us; a full buffer (cap bytes, <= the ESP-NOW payload
 *                  limit of 250 B) at once.  Nothing is checked, changed or dropped: the receiving decoder (the COM's
 *                  core/link_proto.c, the ground station's sim/link_proto.py) checks CRC / length / vehicle id.
 *   ByteRing       air -> UART.  Received packets are copied here (the ESP-NOW receive callback runs in the Wi-Fi
 *                  task) and written to the UART from loop(); a packet that does not fit is dropped whole and counted.
 *
 * Invariant: byte-transparent - the concatenation of every packet the chunker emits equals the byte stream it was fed,
 * in order, and every emitted packet holds 1..cap bytes.  Time is a free-running uint32 microsecond counter (micros()
 * on the ESP; it wraps after 71.6 minutes - every comparison is a wrap-safe difference).
 */
#ifndef ESP_BRIDGE_CORE_H
#define ESP_BRIDGE_CORE_H

#include <stddef.h>
#include <stdint.h>

#define BRIDGE_ESPNOW_MAX 250u               /* ESP-NOW payload limit (A: Espressif ESP-NOW user guide, not on disk) */

struct BridgeFrameRule {
    uint8_t sync0, sync1;                     /* sim/link_proto.py SYNC0 / SYNC1 */
    uint8_t len_min, len_max;                 /* LEN_MIN / LEN_MAX */
    uint8_t overhead;                         /* FRAME_OVERHEAD: frame bytes = LEN + overhead */
};

class BridgeChunker {
public:
    /* cap: packet size limit (1..BRIDGE_ESPNOW_MAX); idle_us: flush after this much UART silence; frame_wait_us: the
     * same for a buffer that starts with an incomplete frame header */
    BridgeChunker(const BridgeFrameRule &rule, uint16_t cap, uint32_t idle_us, uint32_t frame_wait_us);

    /* one UART byte at time now_us; returns false (byte not taken) only if the buffer is full - call due() / take()
     * first, as the sketch does */
    bool push(uint8_t b, uint32_t now_us);
    /* bytes to send now (0 = nothing yet): see the rule above */
    uint16_t due(uint32_t now_us) const;
    /* copy the first n bytes (n = due()) to out and remove them; returns n */
    uint16_t take(uint8_t *out, uint16_t n);

    uint16_t pending() const { return n_; }
    uint16_t capacity() const { return cap_; }

private:
    uint16_t frame_bytes() const;             /* LEN + overhead of the frame at buf_[0], 0 if buf_ holds no header */

    BridgeFrameRule rule_;
    uint16_t cap_;
    uint32_t idle_us_, frame_wait_us_;
    uint8_t buf_[BRIDGE_ESPNOW_MAX];
    uint16_t n_;
    uint32_t t_last_;                         /* time of the newest byte */
};

/* single producer (the receive callback) / single consumer (loop()) byte ring; the sketch guards put() / get() the
 * way its platform needs (ESP8266: the ESP-NOW callback and loop() run in the same task, no lock needed - A) */
class ByteRing {
public:
    ByteRing() : head_(0), tail_(0), dropped_packets_(0), dropped_bytes_(0) {}
    static const uint16_t SIZE = 1024u;      /* power of two; 20 TELEM v5 frames */

    /* a whole packet or nothing: false (counted) if n bytes do not fit */
    bool put(const uint8_t *p, uint16_t n);
    /* up to max bytes into out; returns the count */
    uint16_t get(uint8_t *out, uint16_t max);
    uint16_t used() const { return (uint16_t)((head_ - tail_) & (SIZE - 1u)); }
    uint16_t space() const { return (uint16_t)(SIZE - 1u - used()); }
    uint32_t dropped_packets() const { return dropped_packets_; }
    uint32_t dropped_bytes() const { return dropped_bytes_; }

private:
    uint8_t buf_[SIZE];
    volatile uint16_t head_, tail_;
    uint32_t dropped_packets_, dropped_bytes_;
};

/* "02:CA:5A:00:00:01" -> 6 bytes; false on any other form (the config generator writes the bytes, this is for tests and
 * for a MAC typed into a serial console later) */
bool bridge_parse_mac(const char *s, uint8_t mac[6]);

#endif /* ESP_BRIDGE_CORE_H */
