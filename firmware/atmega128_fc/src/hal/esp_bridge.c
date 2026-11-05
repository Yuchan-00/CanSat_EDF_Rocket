/* esp_bridge.c - see esp_bridge.h (compiled only for FC_LINK_RADIO == FC_RADIO_ESP_BRIDGE). */
#include "../params_gen.h"

#if defined(FC_ROLE_FCU) || defined(FC_ROLE_NAV)
#error "hal/esp_bridge.c is COM firmware: the FCU and the NAV have no radio (platformio.ini [fcu_common] / [nav_common] leave it out)"
#endif

#if FC_LINK_RADIO == FC_RADIO_ESP_BRIDGE
#include "esp_bridge.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <util/atomic.h>
#include <util/delay.h>

#include "../board.h"
#include "watchdog.h"

#if (ESP_BRIDGE_RX_BUF & (ESP_BRIDGE_RX_BUF - 1u)) != 0 || ESP_BRIDGE_RX_BUF > 256u
#error "ESP_BRIDGE_RX_BUF must be a power of two <= 256"
#endif
#if (ESP_BRIDGE_TX_BUF & (ESP_BRIDGE_TX_BUF - 1u)) != 0 || ESP_BRIDGE_TX_BUF > 256u
#error "ESP_BRIDGE_TX_BUF must be a power of two <= 256"
#endif
#if ESP_UART_U2X != 0
#error "the ESP bridge USART runs U2X = 0 (esp_bridge.h)"
#endif

/* the ESP's RST: the radio reset pin of params flight_computer.com.pins.lora_reset (PC6, through a BSS138 channel) */
#define ESP_RST_PORTR LORA_RESET_PORTR
#define ESP_RST_DDR   LORA_RESET_DDR
#define ESP_RST_BIT   PIN_LORA_RESET_BIT

static volatile uint8_t s_rx[ESP_BRIDGE_RX_BUF];
static volatile uint8_t s_rx_head, s_rx_tail;        /* head: ISR writes, tail: main reads */
static volatile uint16_t s_rx_errors_isr, s_rx_overflow_isr, s_rx_bytes_isr, s_rx_discard_isr;
static volatile uint8_t s_booting;                    /* 1: inside the boot window, the RX ISR drops every byte */
static volatile uint8_t s_tx[ESP_BRIDGE_TX_BUF];
static volatile uint8_t s_tx_head, s_tx_tail;        /* head: main writes, tail: ISR sends */
static uint32_t s_boot_t0;
static bool s_have_t0;
static radio_stats_t s_stats;

ISR(USART1_RX_vect)
{
    const uint8_t st = UCSR1A;
    const uint8_t b = UDR1;
    uint8_t h;
    if (s_booting) {                                  /* the ESP's 74880-baud boot log: not link bytes */
        s_rx_discard_isr++;
        return;
    }
    if (st & ((1 << FE1) | (1 << DOR1))) {
        s_rx_errors_isr++;
        return;
    }
    h = (uint8_t)((s_rx_head + 1u) & (ESP_BRIDGE_RX_BUF - 1u));
    if (h == s_rx_tail) {
        s_rx_overflow_isr++;
        return;
    }
    s_rx[s_rx_head] = b;
    s_rx_head = h;
    s_rx_bytes_isr++;
}

ISR(USART1_UDRE_vect)
{
    if (s_tx_head == s_tx_tail) {
        UCSR1B &= (uint8_t)~(1 << UDRIE1);
        return;
    }
    UDR1 = s_tx[s_tx_tail];
    s_tx_tail = (uint8_t)((s_tx_tail + 1u) & (ESP_BRIDGE_TX_BUF - 1u));
}

/* cold COM boot: power-on, or no reset flag at all (the debugger, a jump to 0).  A watchdog / brown-out / external reset
 * of the COM alone leaves the ESP running (2026-11-06 review F2: as the Ebyte module, which a COM reset never touched) */
static bool esp_cold_boot(void)
{
    const uint8_t rc = watchdog_reset_cause();
    return (rc & RESET_POWER_ON) != 0u || (rc & (RESET_WATCHDOG | RESET_BROWN_OUT | RESET_EXTERNAL | RESET_JTAG)) == 0u;
}

/* PC6 is open-drain (review F1): PORT bit always 0; DDR 1 = RST held low, DDR 0 = released (Hi-Z, Q5's pull-ups raise RST)
 * - never driven high, so a manual RST-GND short (J3 5-6, the rocket's programming header) cannot short a driven pin */
void radio_pins_safe(void)
{
    ESP_RST_PORTR &= (uint8_t)~(1 << ESP_RST_BIT);           /* PORT 0 first: setting DDR then can only drive low */
    if (esp_cold_boot())
        ESP_RST_DDR |= (uint8_t)(1 << ESP_RST_BIT);          /* hold the ESP in reset until radio_init */
}

void radio_init(void)
{
    uint8_t i;
    const bool cold = esp_cold_boot();
    radio_pins_safe();
    UBRR1H = (uint8_t)(ESP_UART_UBRR >> 8);
    UBRR1L = (uint8_t)(ESP_UART_UBRR & 0xFF);
    UCSR1A = 0;                                               /* U2X = 0 */
    UCSR1C = (uint8_t)((1 << UCSZ11) | (1 << UCSZ10));       /* 8N1 */
    if (cold) {
        for (i = 0; i < ESP_RESET_HOLD_MS; i++) {             /* RST low >= ESP_RESET_HOLD_MS (cold boot only) */
            _delay_ms(1);
            wdt_reset();
        }
    }
    s_booting = cold ? 1u : 0u;                               /* warm: the ESP kept running, its bytes are link bytes */
    s_have_t0 = false;
    UCSR1B = (uint8_t)((1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1));
    ESP_RST_DDR &= (uint8_t)~(1 << ESP_RST_BIT);              /* release (Hi-Z): the ESP boots / keeps running */
    s_stats.ok = true;
    s_stats.version = 0;
    s_stats.rssi_dbm = RADIO_RSSI_NONE;
    s_stats.snr_q4 = 0;
}

bool radio_service(uint32_t tick)
{
    if (s_booting) {                                          /* the window counts from the first service after the release */
        if (!s_have_t0) {
            s_boot_t0 = tick;
            s_have_t0 = true;
        } else if (tick - s_boot_t0 >= ESP_BOOT_TICKS) {
            s_booting = 0;
        }
    }
    return false;
}

uint8_t radio_read(uint8_t *buf, uint8_t max)
{
    uint8_t n = 0, h;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        h = s_rx_head;
    }
    while (s_rx_tail != h && n < max) {
        buf[n++] = s_rx[s_rx_tail];
        s_rx_tail = (uint8_t)((s_rx_tail + 1u) & (ESP_BRIDGE_RX_BUF - 1u));
    }
    return n;
}

bool radio_tx_busy(void)
{
    uint8_t h, t;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        h = s_tx_head;
        t = s_tx_tail;
    }
    return h != t;
}

/* no busy line: the frame goes to the UART at once (esp_bridge.h) */
bool radio_send(const uint8_t *frame, uint8_t len)
{
    uint8_t i, h;
    if (len >= ESP_BRIDGE_TX_BUF || radio_tx_busy())
        return false;
    h = s_tx_head;                                            /* the ring is empty: only this function writes head */
    for (i = 0; i < len; i++) {
        s_tx[h] = frame[i];
        h = (uint8_t)((h + 1u) & (ESP_BRIDGE_TX_BUF - 1u));
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        s_tx_head = h;
        UCSR1B |= (uint8_t)(1 << UDRIE1);
    }
    s_stats.tx_frames++;
    return true;
}

const radio_stats_t *radio_stats(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        s_stats.rx_bytes = s_rx_bytes_isr;
        s_stats.rx_errors = s_rx_errors_isr;
        s_stats.rx_overflow = s_rx_overflow_isr;
    }
    return &s_stats;
}

uint16_t esp_bridge_rx_discarded(void)
{
    uint16_t n;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        n = s_rx_discard_isr;
    }
    return n;
}

#ifdef FC_SIMBENCH
/* simbench: bytes as if the ESP had received them over the air.  They bypass the boot window (it filters only the real RX
 * pin): the benches' synthetic ground polls from 0.2 s, inside the window of the cold boot every bench starts with, and
 * the single-chip bench's stress-burst check depends on that poll sequence.  The window itself is measured on the pin by
 * tools/esp_bridge_test (cold: dropped and counted; warm COM boot: no window) - 2026-11-06 review F2 */
void radio_inject_rx(const uint8_t *bytes, uint8_t n)
{
    uint8_t i;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        for (i = 0; i < n; i++) {
            uint8_t h = (uint8_t)((s_rx_head + 1u) & (ESP_BRIDGE_RX_BUF - 1u));
            if (h == s_rx_tail) {
                s_rx_overflow_isr++;
                break;
            }
            s_rx[s_rx_head] = bytes[i];
            s_rx_head = h;
            s_rx_bytes_isr++;
        }
    }
}
#endif

#endif /* FC_LINK_RADIO == FC_RADIO_ESP_BRIDGE */
