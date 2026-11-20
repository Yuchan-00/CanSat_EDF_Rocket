/* sx127x.c - see sx127x.h (compiled only for FC_LINK_RADIO == FC_RADIO_SX127X). */
#include "../params_gen.h"

#if defined(FC_ROLE_FCU) || defined(FC_ROLE_NAV)
#error "hal/sx127x.c is COM firmware: the FCU and the NAV have no radio (platformio.ini [fcu_common] / [nav_common] leave it out)"
#endif

#if FC_LINK_RADIO == FC_RADIO_SX127X
#include "sx127x.h"

#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include "../board.h"
#include "../core/link_proto.h"

#if (SX127X_RX_BUF & (SX127X_RX_BUF - 1u)) != 0 || SX127X_RX_BUF > 256u
#error "SX127X_RX_BUF must be a power of two <= 256"
#endif

#define OPMODE(m) ((uint8_t)(SX_MODE_LORA | (SX127X_LOW_FREQ_MODE ? 0x08u : 0u) | (m)))
#define SPI_WAIT_LOOPS 255u                  /* > 8 bit times at the slowest divider (F_CPU/128: 64 cycles per bit) */

typedef enum { S_OFF = 0, S_RX, S_RX_READ, S_TX_LOAD, S_TX_START, S_TX_WAIT } sx_state_t;

static sx_state_t s_state;
static uint8_t s_rx[SX127X_RX_BUF];
static uint8_t s_rx_head, s_rx_tail;
static uint8_t s_pkt_left;                   /* payload bytes still to read from the FIFO */
static uint8_t s_tx[LP_FRAME_MAX_BYTES];
static uint8_t s_tx_len, s_tx_done;
static bool s_tx_pending;
static uint32_t s_tx_start;
static bool s_tx_started;
static radio_stats_t s_stats;

/* ---- SPI --------------------------------------------------------------------------------- */
static void nss(bool high)
{
    if (high)
        LORA_NSS_PORTR |= (uint8_t)(1 << PIN_LORA_NSS_BIT);
    else
        LORA_NSS_PORTR &= (uint8_t)~(1 << PIN_LORA_NSS_BIT);
}

static uint8_t xfer(uint8_t b)
{
    uint8_t n = 0;
    SPDR = b;
    while (!(SPSR & (1 << SPIF)) && ++n < SPI_WAIT_LOOPS) {
    }
    return SPDR;
}

static void reg_write(uint8_t reg, uint8_t v)
{
    nss(false);
    xfer((uint8_t)(reg | 0x80u));
    xfer(v);
    nss(true);
}

static uint8_t reg_read(uint8_t reg)
{
    uint8_t v;
    nss(false);
    xfer((uint8_t)(reg & 0x7Fu));
    v = xfer(0);
    nss(true);
    return v;
}

static bool dio0(void)
{
    return (LORA_DIO0_PINR & (1 << PIN_LORA_DIO0_BIT)) != 0;
}

/* ---- pins / init ------------------------------------------------------------------------ */
void radio_pins_safe(void)
{
    nss(true);                                              /* chip deselected before the pin drives */
    LORA_NSS_DDR |= (uint8_t)(1 << PIN_LORA_NSS_BIT);
    PORTB |= (uint8_t)(1 << PB0);                           /* SS high whether or not it is NSS */
    DDRB |= (uint8_t)((1 << PB0) | (1 << PB1) | (1 << PB2));   /* SS (master mode needs it as output), SCK, MOSI */
    DDRB &= (uint8_t)~(1 << PB3);                           /* MISO */
    LORA_RESET_PORTR &= (uint8_t)~(1 << PIN_LORA_RESET_BIT);   /* hold the radio in reset until radio_init */
    LORA_RESET_DDR |= (uint8_t)(1 << PIN_LORA_RESET_BIT);
    LORA_DIO0_DDR &= (uint8_t)~(1 << PIN_LORA_DIO0_BIT);
    LORA_DIO0_PORTR &= (uint8_t)~(1 << PIN_LORA_DIO0_BIT);  /* push-pull output of the radio: no pull-up */
}

static void enter_rx(void)
{
    reg_write(SX_REG_OP_MODE, OPMODE(SX_MODE_STDBY));
    reg_write(SX_REG_DIO_MAPPING1, SX_DIO0_RX_DONE);
    reg_write(SX_REG_FIFO_ADDR_PTR, SX_FIFO_RX_BASE);
    reg_write(SX_REG_IRQ_FLAGS, 0xFF);
    reg_write(SX_REG_OP_MODE, OPMODE(SX_MODE_RX_CONT));
    s_state = S_RX;
}

void radio_init(void)
{
    uint8_t i;
    radio_pins_safe();
    SPCR = (uint8_t)((1 << SPE) | (1 << MSTR) | (SX127X_SPI_SPR & 0x03));   /* mode 0, MSB first */
    SPSR = SX127X_SPI_2X ? (uint8_t)(1 << SPI2X) : 0;
    for (i = 0; i < 2; i++) {                               /* reset low >= 100 us, then >= 5 ms (datasheet 7.2.2) */
        _delay_ms(1);
        wdt_reset();
    }
    LORA_RESET_PORTR |= (uint8_t)(1 << PIN_LORA_RESET_BIT);
    for (i = 0; i < 10; i++) {
        _delay_ms(1);
        wdt_reset();
    }
    s_stats.rssi_dbm = RADIO_RSSI_NONE;
    s_stats.version = reg_read(SX_REG_VERSION);
    s_stats.ok = s_stats.version == SX127X_VERSION_EXPECTED;
#ifdef FC_SIMBENCH
    s_stats.ok = true;                                      /* simavr has no chip: run the state machine on dead SPI */
#endif
    if (!s_stats.ok) {
        s_state = S_OFF;                                    /* no radio: no link, the vehicle cannot arm */
        return;
    }
    reg_write(SX_REG_OP_MODE, SX_MODE_SLEEP);               /* LongRangeMode can only change in sleep */
    reg_write(SX_REG_OP_MODE, OPMODE(SX_MODE_SLEEP));
    reg_write(SX_REG_FRF_MSB, SX127X_FRF_MSB);
    reg_write(SX_REG_FRF_MID, SX127X_FRF_MID);
    reg_write(SX_REG_FRF_LSB, SX127X_FRF_LSB);
    reg_write(SX_REG_PA_CONFIG, SX127X_PA_CONFIG);
    reg_write(SX_REG_LNA, SX127X_LNA);
    reg_write(SX_REG_MODEM_CONFIG1, SX127X_MODEM_CONFIG1);
    reg_write(SX_REG_MODEM_CONFIG2, SX127X_MODEM_CONFIG2);
    reg_write(SX_REG_MODEM_CONFIG3, SX127X_MODEM_CONFIG3);
    reg_write(SX_REG_PREAMBLE_MSB, (uint8_t)(SX127X_PREAMBLE >> 8));
    reg_write(SX_REG_PREAMBLE_LSB, (uint8_t)(SX127X_PREAMBLE & 0xFF));
    reg_write(SX_REG_SYNC_WORD, SX127X_SYNC_WORD);
    reg_write(SX_REG_PAYLOAD_MAX, SX127X_RX_BUF);
    reg_write(SX_REG_FIFO_TX_BASE, SX_FIFO_TX_BASE);
    reg_write(SX_REG_FIFO_RX_BASE, SX_FIFO_RX_BASE);
    reg_write(SX_REG_IRQ_FLAGS_MASK, (uint8_t)~(SX_IRQ_RX_DONE | SX_IRQ_CRC_ERROR | SX_IRQ_TX_DONE));
    enter_rx();
}

/* ---- background ------------------------------------------------------------------------- */
static void rx_push(uint8_t b)
{
    uint8_t h = (uint8_t)((s_rx_head + 1u) & (SX127X_RX_BUF - 1u));
    if (h == s_rx_tail) {
        s_stats.rx_overflow++;
        return;
    }
    s_rx[s_rx_head] = b;
    s_rx_head = h;
    s_stats.rx_bytes++;
}

bool radio_service(uint32_t tick)
{
    switch (s_state) {
    case S_RX:
        if (s_tx_pending) {                                 /* a reply: standby, load the FIFO from its TX base */
            reg_write(SX_REG_OP_MODE, OPMODE(SX_MODE_STDBY));
            reg_write(SX_REG_FIFO_ADDR_PTR, SX_FIFO_TX_BASE);
            s_tx_done = 0;
            s_tx_pending = false;
            s_state = S_TX_LOAD;
            return true;
        }
        if (dio0()) {                                       /* RxDone */
            const uint8_t flags = reg_read(SX_REG_IRQ_FLAGS);
            reg_write(SX_REG_IRQ_FLAGS, 0xFF);
            if (!(flags & SX_IRQ_RX_DONE))
                return true;
            if (flags & SX_IRQ_CRC_ERROR) {
                s_stats.rx_errors++;
                return true;
            }
            {
                const uint8_t n = reg_read(SX_REG_RX_NB_BYTES);
                const uint8_t at = reg_read(SX_REG_FIFO_RX_CURRENT);
                const int8_t snr = (int8_t)reg_read(SX_REG_PKT_SNR);
                int16_t rssi = (int16_t)(SX127X_RSSI_OFFSET + (int16_t)reg_read(SX_REG_PKT_RSSI));
                if (snr < 0)
                    rssi = (int16_t)(rssi + snr / 4);
                s_stats.snr_q4 = snr;
                s_stats.rssi_dbm = (int8_t)(rssi < -128 ? -128 : (rssi > 126 ? 126 : rssi));
                if (n == 0u || n > SX127X_RX_BUF) {
                    s_stats.rx_errors++;
                    return true;
                }
                reg_write(SX_REG_FIFO_ADDR_PTR, at);
                s_pkt_left = n;
                s_state = S_RX_READ;
            }
            return true;
        }
        return false;
    case S_RX_READ: {
        uint8_t k = s_pkt_left < SX127X_CHUNK ? s_pkt_left : SX127X_CHUNK, i;
        nss(false);
        xfer(SX_REG_FIFO);
        for (i = 0; i < k; i++)
            rx_push(xfer(0));
        nss(true);
        s_pkt_left = (uint8_t)(s_pkt_left - k);
        if (s_pkt_left == 0u)
            s_state = S_RX;                                 /* still in continuous RX */
        return true;
    }
    case S_TX_LOAD: {
        uint8_t k = (uint8_t)(s_tx_len - s_tx_done), i;
        if (k > SX127X_CHUNK)
            k = SX127X_CHUNK;
        nss(false);
        xfer((uint8_t)(SX_REG_FIFO | 0x80u));
        for (i = 0; i < k; i++)
            xfer(s_tx[s_tx_done + i]);
        nss(true);
        s_tx_done = (uint8_t)(s_tx_done + k);
        if (s_tx_done >= s_tx_len)
            s_state = S_TX_START;
        return true;
    }
    case S_TX_START:
        reg_write(SX_REG_PAYLOAD_LENGTH, s_tx_len);
        reg_write(SX_REG_DIO_MAPPING1, SX_DIO0_TX_DONE);
        reg_write(SX_REG_IRQ_FLAGS, 0xFF);
        reg_write(SX_REG_OP_MODE, OPMODE(SX_MODE_TX));
        s_tx_started = false;
        s_state = S_TX_WAIT;
        s_stats.tx_frames++;
        return true;
    case S_TX_WAIT:
        if (!s_tx_started) {                                /* the timeout counts from the first service after TX */
            s_tx_start = tick;
            s_tx_started = true;
        }
        if (dio0()) {
            reg_write(SX_REG_IRQ_FLAGS, 0xFF);
            enter_rx();
            return true;
        }
        if (tick - s_tx_start >= SX127X_TX_TIMEOUT_TICKS) {
            s_stats.tx_timeouts++;
            enter_rx();
            return true;
        }
        return false;
    default:
        return false;
    }
}

uint8_t radio_read(uint8_t *buf, uint8_t max)
{
    uint8_t n = 0;
    while (s_rx_tail != s_rx_head && n < max) {
        buf[n++] = s_rx[s_rx_tail];
        s_rx_tail = (uint8_t)((s_rx_tail + 1u) & (SX127X_RX_BUF - 1u));
    }
    return n;
}

bool radio_tx_busy(void)
{
    return s_tx_pending || s_state == S_TX_LOAD || s_state == S_TX_START || s_state == S_TX_WAIT;
}

bool radio_send(const uint8_t *frame, uint8_t len)
{
    uint8_t i;
    if (s_state == S_OFF || len == 0u || len > sizeof s_tx || radio_tx_busy())
        return false;
    for (i = 0; i < len; i++)
        s_tx[i] = frame[i];
    s_tx_len = len;
    s_tx_pending = true;                                    /* started by radio_service once a packet read is finished */
    return true;
}

const radio_stats_t *radio_stats(void)
{
    return &s_stats;
}

#ifdef FC_SIMBENCH
void radio_inject_rx(const uint8_t *bytes, uint8_t n)
{
    uint8_t i;
    for (i = 0; i < n; i++)
        rx_push(bytes[i]);
}
#endif

#endif /* FC_LINK_RADIO == FC_RADIO_SX127X */
