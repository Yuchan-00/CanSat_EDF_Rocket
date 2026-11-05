/* ESP8266 UART <-> ESP-NOW bridge (README.md): ONE sketch for both ends of the vehicle link.
 *
 *   -D BRIDGE_ROLE_VEHICLE  the ESP-12E soldered on the VTOL carrier (U7) or wired to the rocket's COM board: UART0
 *                           (GPIO1 TXD0 / GPIO3 RXD0) <-> the COM's USART1 through BSS138 channels; the COM holds it in
 *                           reset at a cold COM boot and releases it (COM PC6 -> RST, open-drain, hal/esp_bridge.c); a
 *                           COM-only reset leaves it running
 *   -D BRIDGE_ROLE_GROUND   an ESP-07S-class module (u.FL, external 8-14 dBi antenna) on a 3.3 V USB-UART: UART0 <-> the
 *                           PC (tools/ground_station.py --link-hw esp_bridge)
 *
 * Byte-transparent in both directions (no flight logic here - project rule: all flight computing on the ATmega128s):
 *   UART -> air : src/bridge_core.h BridgeChunker - a complete link frame goes out as one ESP-NOW packet as soon as its
 *                 last byte is in, other bytes after BRIDGE_IDLE_FLUSH_US of UART silence (an incomplete frame after
 *                 BRIDGE_FRAME_WAIT_US), never more than 250 bytes per packet.  Nothing is checked, changed or dropped.
 *   air -> UART : every ESP-NOW packet from the configured peer is written to the UART unchanged (packets from any other
 *                 sender are ignored); the receive callback only copies into a ByteRing, loop() writes the UART.
 * The vehicle-side frame protocol (core/link_proto*, CRC, nonce, TELEM formats, the command set) is untouched: the COM
 * and the ground station see exactly the bytes they saw over a transparent LoRa module.
 *
 * The UART carries only link bytes: the sketch never prints (the ESP8266 ROM prints its boot log at 74880 baud on TXD0
 * after every reset - the COM drops its UART input for ESP_BOOT_TICKS after releasing the reset, and both decoders skip
 * garbage anyway).
 *
 * Radio settings (ESP8266 Arduino core 3.x API; compiled on this PC with the core esp8266:esp8266 3.1.2 - build_arduino_cli.py,
 * 2026-11-06: names and prototypes M; nothing was run on hardware - what each call does at run time is A):
 *   WiFi.mode(WIFI_STA), WiFi.disconnect()        station interface, not associated (ESP-NOW only)
 *   wifi_set_macaddr(STATION_IF, own MAC)         SDK user_interface.h: both MACs come from params.json, so each end
 *                                                 knows its peer without reading the modules
 *   WiFi.setPhyMode(WIFI_PHY_MODE_11B)            802.11b only (DSSS: the best sensitivity)
 *   WiFi.setOutputPower(dBm)                      0..20.5 dBm (link.esp_bridge.tx_power_dbm_*)
 *   WiFi.setSleepMode(WIFI_NONE_SLEEP)            no modem sleep (latency)
 *   wifi_set_channel(ch)                          SDK: the fixed channel (link.esp_bridge.espnow_channel)
 *   esp_now_init / esp_now_set_self_role(ESP_NOW_ROLE_COMBO) / esp_now_add_peer(mac, ESP_NOW_ROLE_COMBO, ch, NULL, 0) /
 *   esp_now_register_recv_cb / esp_now_register_send_cb / esp_now_send   (espnow.h, unicast: the MAC layer ACKs and
 *                                                 retries each packet)
 *   bit rate: ESP-NOW frames go out at 1 Mbps by default (A: Espressif ESP-NOW user guide, not on disk).  The SDK's
 *   wifi_set_user_rate_limit is compiled only with -D BRIDGE_USE_RATE_LIMIT_API (present in the 3.1.2 SDK, compile M;
 *   review LB-7: ESP-NOW action frames may follow a P2P rate class instead of RC_LIMIT_11B - the 3.1.2 user_interface.h
 *   has RC_LIMIT_P2P_11G / RC_LIMIT_P2P_11N but no RC_LIMIT_P2P_11B (M); whether the limit reaches ESP-NOW frames is A).
 *
 * Sending (review LB-3): one ESP-NOW packet in flight at a time.  A packet taken from the chunker waits in s_tx until
 * esp_now_send accepts it (a refusal is retried on the next pass - order kept) and is dropped (counted) only after
 * BRIDGE_TX_GIVE_UP_US of refusals; the next one goes out after the send callback of the previous one, or after
 * BRIDGE_SEND_TIMEOUT_US without one (A: the SDK's callback timing is not measured).  The link protocol is half-duplex
 * polling (one request, one reply, 2 Hz), so a packet rarely waits.
 *
 * Diagnostics (review LB-4): -D BRIDGE_DEBUG_UART1 prints the counters once per second on Serial1 (GPIO2, UART1 TX only,
 * 115200 8N1) - for the range test with a 3.3 V USB-UART on GPIO2; GPIO2 already has its 10 k boot pull-up and UART1
 * idles high, so the boot strap is kept (A; the ESP-12E's on-module LED may sit on GPIO2 and blink, A).  Off by default:
 * the link UART (UART0) never carries anything but link bytes.
 */
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
extern "C" {
#include <user_interface.h>
}

#include "bridge_core.h"
#include "esp_bridge_config_gen.h"

#if defined(BRIDGE_ROLE_VEHICLE) == defined(BRIDGE_ROLE_GROUND)
#error "build exactly one role: -D BRIDGE_ROLE_VEHICLE or -D BRIDGE_ROLE_GROUND (platformio.ini)"
#endif

#ifdef BRIDGE_ROLE_VEHICLE
static uint8_t s_own[6] = BRIDGE_MAC_VEHICLE;
static uint8_t s_peer[6] = BRIDGE_MAC_GROUND;
#define BRIDGE_TX_DBM BRIDGE_TX_DBM_VEHICLE
#else
static uint8_t s_own[6] = BRIDGE_MAC_GROUND;
static uint8_t s_peer[6] = BRIDGE_MAC_VEHICLE;
#define BRIDGE_TX_DBM BRIDGE_TX_DBM_GROUND
#endif

static const BridgeFrameRule kRule = {LINK_SYNC0, LINK_SYNC1, LINK_LEN_MIN, LINK_LEN_MAX, LINK_FRAME_OVERHEAD};
static BridgeChunker s_up(kRule, BRIDGE_ESPNOW_MAX, BRIDGE_IDLE_FLUSH_US, BRIDGE_FRAME_WAIT_US);
static ByteRing s_down;
static volatile uint32_t s_sent_ok, s_sent_fail, s_rx_packets, s_rx_foreign;
static uint32_t s_send_refused, s_send_dropped, s_send_timeouts;

#define BRIDGE_SEND_TIMEOUT_US 30000UL           /* no send callback this long after esp_now_send: free the slot (A) */
#define BRIDGE_TX_GIVE_UP_US 50000UL             /* a packet esp_now_send refuses this long is dropped (lost packet) */
static uint8_t s_tx[BRIDGE_ESPNOW_MAX];          /* the packet taken from the chunker, not yet accepted by the SDK */
static uint16_t s_tx_n;
static uint32_t s_tx_t0, s_sent_t;
static volatile bool s_in_flight;

static bool same_mac(const uint8_t *a, const uint8_t *b)
{
    return memcmp(a, b, 6) == 0;
}

static void on_recv(uint8_t *mac, uint8_t *data, uint8_t len)
{
    if (!same_mac(mac, s_peer)) {
        s_rx_foreign++;
        return;
    }
    s_rx_packets++;
    (void)s_down.put(data, len);                  /* a packet that does not fit is dropped whole and counted */
}

static void on_sent(uint8_t *mac, uint8_t status)
{
    (void)mac;
    s_in_flight = false;
    if (status == 0)
        s_sent_ok++;
    else
        s_sent_fail++;                            /* no ACK after the MAC-layer retries: the link lost it */
}

void setup()
{
    Serial.setRxBufferSize(1024);                 /* before begin (core API) */
    Serial.begin(BRIDGE_UART_BAUD, SERIAL_8N1);
    Serial.setDebugOutput(false);                 /* the UART carries only link bytes */
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    wifi_set_macaddr(STATION_IF, s_own);
    WiFi.setPhyMode(WIFI_PHY_MODE_11B);
    WiFi.setOutputPower(BRIDGE_TX_DBM);
    WiFi.setSleepMode(WIFI_NONE_SLEEP);
    wifi_set_channel(BRIDGE_ESPNOW_CHANNEL);
#ifdef BRIDGE_USE_RATE_LIMIT_API
    wifi_set_user_rate_limit(RC_LIMIT_11B, STATION_IF, RATE_11B_B1M, RATE_11B_B1M);   /* A: see the header comment */
#endif
    if (esp_now_init() != 0) {
        delay(100);
        ESP.restart();                            /* the COM / the ground station see a silent link, as with a dead radio */
    }
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
    esp_now_add_peer(s_peer, ESP_NOW_ROLE_COMBO, BRIDGE_ESPNOW_CHANNEL, NULL, 0);
    esp_now_register_recv_cb(on_recv);
    esp_now_register_send_cb(on_sent);
#ifdef BRIDGE_DEBUG_UART1
    Serial1.begin(115200, SERIAL_8N1);            /* GPIO2, TX only */
#endif
}

#ifdef BRIDGE_DEBUG_UART1
/* once per second: sent ok / failed (no ACK), refused / dropped / timed-out sends, packets from the peer / from anyone
 * else (a wrong MAC shows up here), downlink ring drops - review LB-4 */
static void debug_counters(void)
{
    static uint32_t t_last;
    const uint32_t now = millis();
    if (now - t_last < 1000UL)
        return;
    t_last = now;
    static const char *const names[] = {"EB ok=", " fail=", " refused=", " dropped=", " tmo=", " rx=", " foreign=",
                                        " ring_drop=", "/"};
    const unsigned long v[] = {(unsigned long)s_sent_ok, (unsigned long)s_sent_fail, (unsigned long)s_send_refused,
                               (unsigned long)s_send_dropped, (unsigned long)s_send_timeouts,
                               (unsigned long)s_rx_packets, (unsigned long)s_rx_foreign,
                               (unsigned long)s_down.dropped_packets(), (unsigned long)s_down.dropped_bytes()};
    for (unsigned i = 0; i < sizeof v / sizeof v[0]; i++) {   /* UART1 only (GPIO2); UART0 stays link bytes */
        Serial1.print(names[i]);
        Serial1.print(v[i]);
    }
    Serial1.print("\r\n");
}
#endif

void loop()
{
    uint8_t pkt[BRIDGE_ESPNOW_MAX];
    uint16_t n;
    /* UART -> air */
    while (Serial.available() > 0 && s_up.pending() < s_up.capacity())
        (void)s_up.push((uint8_t)Serial.read(), micros());
    if (s_in_flight && micros() - s_sent_t >= BRIDGE_SEND_TIMEOUT_US) {
        s_in_flight = false;                      /* no callback: free the slot (counted) */
        s_send_timeouts++;
    }
    if (s_tx_n == 0u) {
        n = s_up.due(micros());
        if (n > 0u) {
            s_tx_n = s_up.take(s_tx, n);
            s_tx_t0 = micros();
        }
    }
    if (s_tx_n > 0u && !s_in_flight) {
        s_in_flight = true;                       /* before the call: a callback from inside it clears it again (A) */
        s_sent_t = micros();
        if (esp_now_send(s_peer, s_tx, (int)s_tx_n) == 0) {
            s_tx_n = 0;                           /* the SDK copied it (A) */
        } else {
            s_in_flight = false;
            s_send_refused++;                     /* the SDK was busy: retried on the next pass */
            if (micros() - s_tx_t0 >= BRIDGE_TX_GIVE_UP_US) {
                s_tx_n = 0;                       /* lost like a radio packet */
                s_send_dropped++;
            }
        }
    }
    /* air -> UART */
    while ((n = s_down.get(pkt, sizeof pkt)) > 0u)
        Serial.write(pkt, n);
#ifdef BRIDGE_DEBUG_UART1
    debug_counters();
#endif
}
