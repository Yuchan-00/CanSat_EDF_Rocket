/* lora_uart.c - see lora_uart.h (compiled only for FC_LINK_RADIO == FC_RADIO_LORA_UART). */
#include "../params_gen.h"

#if defined(FC_ROLE_FCU) || defined(FC_ROLE_NAV)
#error "hal/lora_uart.c is COM firmware: the FCU and the NAV have no radio (platformio.ini [fcu_common] / [nav_common] leave it out)"
#endif

#if FC_LINK_RADIO == FC_RADIO_LORA_UART
#include "lora_uart.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>

#include "../board.h"

#if (LORA_UART_RX_BUF & (LORA_UART_RX_BUF - 1u)) != 0 || LORA_UART_RX_BUF > 256u
#error "LORA_UART_RX_BUF must be a power of two <= 256"
#endif
#if (LORA_UART_TX_BUF & (LORA_UART_TX_BUF - 1u)) != 0 || LORA_UART_TX_BUF > 256u
#error "LORA_UART_TX_BUF must be a power of two <= 256"
#endif

static volatile uint8_t s_rx[LORA_UART_RX_BUF];
static volatile uint8_t s_rx_head, s_rx_tail;        /* head: ISR writes, tail: main reads */
static volatile uint16_t s_rx_errors_isr, s_rx_overflow_isr, s_rx_bytes_isr;
static volatile uint8_t s_tx[LORA_UART_TX_BUF];
static volatile uint8_t s_tx_head, s_tx_tail;        /* head: main writes, tail: ISR sends */
static bool s_tx_pending;                             /* queued, waiting for AUX */
static uint32_t s_tx_queued_tick;
static bool s_have_queued_tick;
static radio_stats_t s_stats;

ISR(USART1_RX_vect)
{
    const uint8_t st = UCSR1A;
    const uint8_t b = UDR1;
    uint8_t h;
    if (st & ((1 << FE1) | (1 << DOR1))) {
        s_rx_errors_isr++;
        return;
    }
    h = (uint8_t)((s_rx_head + 1u) & (LORA_UART_RX_BUF - 1u));
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
    s_tx_tail = (uint8_t)((s_tx_tail + 1u) & (LORA_UART_TX_BUF - 1u));
}

void radio_pins_safe(void)
{
    LINK_M0_PORTR &= (uint8_t)~(1 << PIN_LINK_M0_BIT);       /* M0 = M1 = 0: normal (transparent) mode */
    LINK_M1_PORTR &= (uint8_t)~(1 << PIN_LINK_M1_BIT);
    LINK_M0_DDR |= (uint8_t)(1 << PIN_LINK_M0_BIT);
    LINK_M1_DDR |= (uint8_t)(1 << PIN_LINK_M1_BIT);
    LINK_AUX_DDR &= (uint8_t)~(1 << PIN_LINK_AUX_BIT);
    LINK_AUX_PORTR |= (uint8_t)(1 << PIN_LINK_AUX_BIT);      /* pull-up: an unconnected AUX reads ready */
}

void radio_init(void)
{
    radio_pins_safe();
    UBRR1H = (uint8_t)(LINK_UART_UBRR >> 8);
    UBRR1L = (uint8_t)(LINK_UART_UBRR & 0xFF);
    UCSR1A = LINK_UART_U2X ? (1 << U2X1) : 0;
    UCSR1C = (uint8_t)((1 << UCSZ11) | (1 << UCSZ10));       /* 8N1 */
    UCSR1B = (uint8_t)((1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1));
    s_stats.ok = true;
    s_stats.version = 0;
    s_stats.rssi_dbm = RADIO_RSSI_NONE;
    s_stats.snr_q4 = 0;
}

static bool aux_ready(void)
{
#ifdef FC_SIMBENCH
    return true;                                            /* simavr: no module on the pin */
#else
    return (LINK_AUX_PINR & (1 << PIN_LINK_AUX_BIT)) != 0;
#endif
}

bool radio_service(uint32_t tick)
{
    if (!s_tx_pending)
        return false;
    if (!s_have_queued_tick) {                              /* the wait starts at the first service after the queue */
        s_tx_queued_tick = tick;
        s_have_queued_tick = true;
    }
    if (aux_ready() || tick - s_tx_queued_tick >= LINK_AUX_WAIT_TICKS) {
        if (!aux_ready())
            s_stats.tx_timeouts++;
        s_tx_pending = false;
        s_have_queued_tick = false;
        s_stats.tx_frames++;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            UCSR1B |= (uint8_t)(1 << UDRIE1);
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
        s_rx_tail = (uint8_t)((s_rx_tail + 1u) & (LORA_UART_RX_BUF - 1u));
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
    return s_tx_pending || h != t;
}

bool radio_send(const uint8_t *frame, uint8_t len)
{
    uint8_t i, h;
    if (len >= LORA_UART_TX_BUF || radio_tx_busy())
        return false;
    h = s_tx_head;                                          /* the ring is empty: only this function writes head */
    for (i = 0; i < len; i++) {
        s_tx[h] = frame[i];
        h = (uint8_t)((h + 1u) & (LORA_UART_TX_BUF - 1u));
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        s_tx_head = h;
    }
    s_tx_pending = true;
    s_have_queued_tick = false;
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

#ifdef FC_SIMBENCH
void radio_inject_rx(const uint8_t *bytes, uint8_t n)
{
    uint8_t i;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        for (i = 0; i < n; i++) {
            uint8_t h = (uint8_t)((s_rx_head + 1u) & (LORA_UART_RX_BUF - 1u));
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

#endif /* FC_LINK_RADIO == FC_RADIO_LORA_UART */
