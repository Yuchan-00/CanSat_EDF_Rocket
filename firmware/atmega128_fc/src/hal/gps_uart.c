/* gps_uart.c - see gps_uart.h (the GNS's USART1 to the GPS receiver; v0.8: the NAV's). */
#ifdef FC_ROLE_NAV   /* fw09 W1 (integration, agent A's request): no NAV env lists this file (platformio.ini nav_common) */
#error "Timer3 belongs to the VTOL NAV nlink (hal/nlink_spi.c, W1): this driver writes Timer3"
#endif
#include "gps_uart.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <string.h>
#include <util/atomic.h>

#include "../fc_mode.h"
#include "../params_gen.h"

/* v0.8.1: the GNS board only (platformio.ini [gns_common] lists it; v0.8 ran it on the VTOL NAV, whose USART1 is the glink
 * since v0.8.1 - the NAV and FCU filters leave this file out) */
#if defined(FC_ROLE_GNS)
#if !defined(GPS_BAUD) || !defined(GPS_UBRR) || !defined(GPS_U2X)
#error "GPS_BAUD / GPS_UBRR / GPS_U2X missing: re-run tools/gen_firmware_params.py (params.json flight_computer.nav.gps)"
#endif
#if GPS_U2X != 0
#error "gps_uart.c computes the 9600-baud UBRR for U2X = 0 (params.json flight_computer.nav.gps)"
#endif
#if (GPS_UART_RX_BUF & (GPS_UART_RX_BUF - 1u)) != 0 || GPS_UART_RX_BUF > 256u
#error "GPS_UART_RX_BUF must be a power of two <= 256"
#endif
#if (GPS_UART_TX_BUF & (GPS_UART_TX_BUF - 1u)) != 0 || GPS_UART_TX_BUF > 256u
#error "GPS_UART_TX_BUF must be a power of two <= 256"
#endif

/* UBRR of a baud at U2X = 0, rounded: F_CPU / (16 baud) - 1 */
#define GPS_UBRR_OF(b) ((uint16_t)(((uint32_t)FC_F_CPU_HZ + 8UL * (uint32_t)(b)) / (16UL * (uint32_t)(b)) - 1UL))
#if ((FC_F_CPU_HZ + 8UL * GPS_BAUD) / (16UL * GPS_BAUD) - 1UL) != GPS_UBRR
#error "params_gen.h GPS_UBRR is not the rounded UBRR of GPS_BAUD at U2X = 0"
#endif

static volatile uint8_t s_rx[GPS_UART_RX_BUF];
static volatile uint8_t s_rx_head, s_rx_tail;        /* head: RX ISR (bench: Timer3 ISR), tail: gps_uart_read */
static volatile uint16_t s_rx_overflow, s_rx_errors;
static uint16_t s_rx_taken;
static volatile uint8_t s_tx[GPS_UART_TX_BUF];
static volatile uint8_t s_tx_head, s_tx_tail;        /* head: gps_uart_write, tail: UDRE ISR */
static volatile bool s_tx_started;                   /* a byte went to UDR1 since init (TXC1 is meaningful) */
static uint16_t s_tx_bytes, s_tx_refused, s_baud_changes;
static uint32_t s_baud, s_pending_baud;              /* s_pending_baud != 0: a switch waits for the idle transmitter */
static uint8_t s_defer[GPS_UART_DEFER_MAX], s_defer_n;
static gps_uart_stats_t s_stats;
#ifdef FC_SIMBENCH
#define GPS_SNOOP_N 48u                              /* > one frame (the NAV queues <= UBX_FRAME_MAX a tick) */
static uint8_t s_snoop[GPS_SNOOP_N];                 /* copies of the queued frames for the synthetic receiver */
static uint8_t s_snoop_head, s_snoop_tail;           /* power-of-two ring */
#endif

#define GPS_RX_ERR_MASK ((1 << FE1) | (1 << DOR1) | (1 << UPE1))
#if GPS_RX_ERR_MASK > 255 || GPS_UART_RX_BUF - 1u > 255 || GPS_UART_TX_BUF - 1u > 255
#error "ISR immediates must be 8-bit"
#endif

/* RX complete, once per received byte (every 260 us while the receiver sends at 38400 baud; 140 bytes per 5 Hz epoch).
 * The leanest form of (hal/ilink_uart.c's RX ISR, the same instructions on USART1):
 *     st = UCSR1A;                                     status first: reading UDR1 clears FE / DOR / UPE
 *     if (st & (FE | DOR | UPE)) { (void)UDR1; s_rx_errors++; }                byte dropped, counted
 *     else if ((head + 1) % RX_BUF == tail) { (void)UDR1; s_rx_overflow++; }   ring full: dropped, counted
 *     else { head' = (head + 1) % RX_BUF; s_rx[head] = UDR1; head = head'; }
 * Normal path: 4 response + 3 vector JMP + 40 body + 4 RETI = 51 CPU cycles (3.2 us) per byte.  Uses r24, r30, r31 and
 * SREG only.  tools/run_nav_simbench.py re-counts it from the nav ELF's disassembly. */
ISR(USART1_RX_vect, ISR_NAKED)
{
    __asm__ __volatile__(
        "push r24"                      "\n\t"
        "in   r24, __SREG__"            "\n\t"
        "push r24"                      "\n\t"
        "push r30"                      "\n\t"
        "push r31"                      "\n\t"
        "lds  r24, %[ucsra]"            "\n\t"   /* status first */
        "andi r24, %[err]"              "\n\t"
        "brne 2f"                       "\n\t"
        "lds  r30, %[head]"             "\n\t"
        "mov  r24, r30"                 "\n\t"
        "inc  r24"                      "\n\t"
        "andi r24, %[mask]"             "\n\t"   /* r24 = next head */
        "lds  r31, %[tail]"             "\n\t"
        "cp   r24, r31"                 "\n\t"
        "breq 3f"                       "\n\t"   /* ring full */
        "sts  %[head], r24"             "\n\t"
        "ldi  r31, 0"                   "\n\t"
        "subi r30, lo8(-(%[ring]))"     "\n\t"   /* Z = &s_rx[old head] */
        "sbci r31, hi8(-(%[ring]))"     "\n\t"
        "lds  r24, %[udr]"              "\n\t"
        "st   Z, r24"                   "\n\t"
        "1:"                            "\n\t"
        "pop  r31"                      "\n\t"
        "pop  r30"                      "\n\t"
        "pop  r24"                      "\n\t"
        "out  __SREG__, r24"            "\n\t"
        "pop  r24"                      "\n\t"
        "reti"                          "\n\t"
        "2:"                            "\n\t"   /* framing / overrun / parity error */
        "lds  r24, %[udr]"              "\n\t"
        "lds  r30, %[errs]"             "\n\t"
        "lds  r31, %[errs]+1"           "\n\t"
        "adiw r30, 1"                   "\n\t"
        "sts  %[errs]+1, r31"           "\n\t"
        "sts  %[errs], r30"             "\n\t"
        "rjmp 1b"                       "\n\t"
        "3:"                            "\n\t"   /* ring full */
        "lds  r24, %[udr]"              "\n\t"
        "lds  r30, %[ovfs]"             "\n\t"
        "lds  r31, %[ovfs]+1"           "\n\t"
        "adiw r30, 1"                   "\n\t"
        "sts  %[ovfs]+1, r31"           "\n\t"
        "sts  %[ovfs], r30"             "\n\t"
        "rjmp 1b"                       "\n\t"
        :
        : [ucsra] "n"(_SFR_MEM_ADDR(UCSR1A)), [udr] "n"(_SFR_MEM_ADDR(UDR1)), [err] "M"(GPS_RX_ERR_MASK),
          [mask] "M"(GPS_UART_RX_BUF - 1u), [head] "i"(&s_rx_head), [tail] "i"(&s_rx_tail), [ring] "i"(&s_rx[0]),
          [errs] "i"(&s_rx_errors), [ovfs] "i"(&s_rx_overflow)
        : "memory");
}

/* UDR1 empty, once per byte sent (boot configuration only): if (head == tail) UDRIE1 off; else UDR1 = s_tx[tail++].
 * 47 CPU cycles a byte (hal/ilink_uart.c's UDRE ISR on USART1).  Uses r24, r30, r31 and SREG only. */
ISR(USART1_UDRE_vect, ISR_NAKED)
{
    __asm__ __volatile__(
        "push r24"                      "\n\t"
        "in   r24, __SREG__"            "\n\t"
        "push r24"                      "\n\t"
        "push r30"                      "\n\t"
        "push r31"                      "\n\t"
        "lds  r30, %[tail]"             "\n\t"
        "lds  r24, %[head]"             "\n\t"
        "cp   r30, r24"                 "\n\t"
        "breq 2f"                       "\n\t"   /* ring empty */
        "mov  r24, r30"                 "\n\t"
        "inc  r24"                      "\n\t"
        "andi r24, %[mask]"             "\n\t"
        "sts  %[tail], r24"             "\n\t"   /* tail + 1 (only this ISR writes the tail) */
        "ldi  r31, 0"                   "\n\t"
        "subi r30, lo8(-(%[ring]))"     "\n\t"   /* Z = &s_tx[old tail] */
        "sbci r31, hi8(-(%[ring]))"     "\n\t"
        "ld   r24, Z"                   "\n\t"
        "sts  %[udr], r24"              "\n\t"
        "1:"                            "\n\t"
        "pop  r31"                      "\n\t"
        "pop  r30"                      "\n\t"
        "pop  r24"                      "\n\t"
        "out  __SREG__, r24"            "\n\t"
        "pop  r24"                      "\n\t"
        "reti"                          "\n\t"
        "2:"                            "\n\t"   /* nothing to send: interrupt off until the next write */
        "lds  r24, %[ucsrb]"            "\n\t"
        "andi r24, %[noudrie]"          "\n\t"
        "sts  %[ucsrb], r24"            "\n\t"
        "rjmp 1b"                       "\n\t"
        :
        : [tail] "i"(&s_tx_tail), [head] "i"(&s_tx_head), [ring] "i"(&s_tx[0]), [mask] "M"(GPS_UART_TX_BUF - 1u),
          [udr] "n"(_SFR_MEM_ADDR(UDR1)), [ucsrb] "n"(_SFR_MEM_ADDR(UCSR1B)),
          [noudrie] "M"((uint8_t)~(1u << UDRIE1) & 0xFFu)
        : "memory");
}

#ifdef FC_SIMBENCH
/* ---- the synthetic receiver's wire (simavr has no GPS on RXD1): a Timer3 compare interrupt per byte time ---- */
#define GPS_BQ_N GPS_UART_BENCH_Q                    /* the bench wire queue (power of two) */
static volatile uint8_t s_bq[GPS_BQ_N];              /* bytes "on the wire" */
static volatile uint8_t s_bq_head, s_bq_tail;

/* Timer3 compare A (CTC, one byte time): the next queued byte into the RX ring - the RX ISR's ring code above with the
 * byte taken from the bench queue instead of UDR1; no byte left: the interrupt switches itself off.  Normal path
 * 4 response + 3 JMP + 52 body + 4 RETI = 63 cycles (12 more than the RX ISR: the queue pop).  Uses r24, r25, r30, r31,
 * SREG. */
ISR(TIMER3_COMPA_vect, ISR_NAKED)
{
    __asm__ __volatile__(
        "push r24"                      "\n\t"
        "in   r24, __SREG__"            "\n\t"
        "push r24"                      "\n\t"
        "push r25"                      "\n\t"
        "push r30"                      "\n\t"
        "push r31"                      "\n\t"
        "lds  r30, %[qt]"               "\n\t"
        "lds  r24, %[qh]"               "\n\t"
        "cp   r30, r24"                 "\n\t"
        "breq 4f"                       "\n\t"   /* nothing queued */
        "mov  r24, r30"                 "\n\t"
        "inc  r24"                      "\n\t"
        "andi r24, %[qmask]"            "\n\t"
        "sts  %[qt], r24"               "\n\t"
        "ldi  r31, 0"                   "\n\t"
        "subi r30, lo8(-(%[bq]))"       "\n\t"
        "sbci r31, hi8(-(%[bq]))"       "\n\t"
        "ld   r25, Z"                   "\n\t"   /* the byte */
        "lds  r30, %[head]"             "\n\t"
        "mov  r24, r30"                 "\n\t"
        "inc  r24"                      "\n\t"
        "andi r24, %[mask]"             "\n\t"
        "lds  r31, %[tail]"             "\n\t"
        "cp   r24, r31"                 "\n\t"
        "breq 3f"                       "\n\t"   /* ring full */
        "sts  %[head], r24"             "\n\t"
        "ldi  r31, 0"                   "\n\t"
        "subi r30, lo8(-(%[ring]))"     "\n\t"
        "sbci r31, hi8(-(%[ring]))"     "\n\t"
        "st   Z, r25"                   "\n\t"
        "1:"                            "\n\t"
        "pop  r31"                      "\n\t"
        "pop  r30"                      "\n\t"
        "pop  r25"                      "\n\t"
        "pop  r24"                      "\n\t"
        "out  __SREG__, r24"            "\n\t"
        "pop  r24"                      "\n\t"
        "reti"                          "\n\t"
        "3:"                            "\n\t"
        "lds  r30, %[ovfs]"             "\n\t"
        "lds  r31, %[ovfs]+1"           "\n\t"
        "adiw r30, 1"                   "\n\t"
        "sts  %[ovfs]+1, r31"           "\n\t"
        "sts  %[ovfs], r30"             "\n\t"
        "rjmp 1b"                       "\n\t"
        "4:"                            "\n\t"
        "lds  r24, %[etimsk]"           "\n\t"
        "andi r24, %[noocie]"           "\n\t"
        "sts  %[etimsk], r24"           "\n\t"
        "rjmp 1b"                       "\n\t"
        :
        : [qt] "i"(&s_bq_tail), [qh] "i"(&s_bq_head), [bq] "i"(&s_bq[0]), [head] "i"(&s_rx_head),
          [tail] "i"(&s_rx_tail), [ring] "i"(&s_rx[0]), [mask] "M"(GPS_UART_RX_BUF - 1u), [ovfs] "i"(&s_rx_overflow),
          [etimsk] "n"(_SFR_MEM_ADDR(ETIMSK)), [noocie] "M"((uint8_t)~(1u << OCIE3A) & 0xFFu),
          [qmask] "M"(GPS_BQ_N - 1u)
        : "memory");
}

uint8_t gps_uart_bench_send(const uint8_t *b, uint8_t n, uint16_t byte_cycles)
{
    uint8_t i, h;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        h = s_bq_head;
        for (i = 0; i < n && (uint8_t)((h + 1u) & (GPS_BQ_N - 1u)) != s_bq_tail; i++) {
            s_bq[h] = b[i];
            h = (uint8_t)((h + 1u) & (GPS_BQ_N - 1u));
        }
        s_bq_head = h;
        if (i != 0u && !(ETIMSK & (uint8_t)(1 << OCIE3A))) {   /* idle wire: the first byte one byte time from now */
            TCCR3B = 0;
            TCCR3A = 0;
            OCR3A = (uint16_t)(byte_cycles - 1u);
            TCNT3 = 0;
            ETIFR = (uint8_t)(1 << OCF3A);
            ETIMSK |= (uint8_t)(1 << OCIE3A);
            TCCR3B = (uint8_t)((1 << WGM32) | (1 << CS30));      /* CTC on OCR3A, clk / 1 */
        } else if (i != 0u) {
            OCR3A = (uint16_t)(byte_cycles - 1u);                /* a baud change takes effect from the next byte */
        }
    }
    return i;
}

uint8_t gps_uart_bench_pending(void)
{
    uint8_t h, t;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        h = s_bq_head;
        t = s_bq_tail;
    }
    return (uint8_t)((h - t) & (GPS_BQ_N - 1u));
}

uint8_t gps_uart_bench_take_tx(uint8_t *b, uint8_t max)
{
    uint8_t n = 0;
    while (s_snoop_tail != s_snoop_head && n < max) {
        b[n++] = s_snoop[s_snoop_tail];
        s_snoop_tail = (uint8_t)((s_snoop_tail + 1u) % GPS_SNOOP_N);
    }
    return n;
}
#endif

static void apply_baud(uint32_t baud)
{
    const uint16_t ubrr = baud == (uint32_t)GPS_BAUD ? (uint16_t)GPS_UBRR : GPS_UBRR_OF(baud);
    UBRR1H = (uint8_t)(ubrr >> 8);
    UBRR1L = (uint8_t)(ubrr & 0xFFu);                 /* writing the low byte updates the prescaler */
    s_baud = baud;
}

void gps_uart_init(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        s_rx_head = s_rx_tail = 0;
        s_tx_head = s_tx_tail = 0;
        s_rx_overflow = s_rx_errors = 0;
        s_rx_taken = 0;
        s_tx_started = false;
        s_tx_bytes = s_tx_refused = s_baud_changes = 0;
        s_pending_baud = 0;
        s_defer_n = 0;
        PORTD |= (uint8_t)(1 << PD2);                /* RXD1 pull-up: an absent receiver reads idle-high, not noise */
        UCSR1B = 0;
        UCSR1A = 0;                                   /* U2X1 = 0 (params_gen.h GPS_U2X) */
        UCSR1C = (uint8_t)((1 << UCSZ11) | (1 << UCSZ10));   /* 8N1, asynchronous */
        apply_baud(GPS_BAUD);
        UCSR1B = (uint8_t)((1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1));
#ifdef FC_SIMBENCH
        s_bq_head = s_bq_tail = 0;
        s_snoop_head = s_snoop_tail = 0;
#endif
    }
}

uint8_t gps_uart_read(uint8_t *buf, uint8_t max)
{
    uint8_t n = 0, h, t = s_rx_tail;                  /* only this function writes the tail */
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        h = s_rx_head;
    }
    while (t != h && n < max) {
        buf[n++] = s_rx[t];
        t = (uint8_t)((t + 1u) & (GPS_UART_RX_BUF - 1u));
    }
    s_rx_tail = t;                                    /* single-byte store: atomic */
    s_rx_taken = (uint16_t)(s_rx_taken + n);
    return n;
}

uint8_t gps_uart_rx_count(void)
{
    const uint8_t h = s_rx_head, t = s_rx_tail;
    return (uint8_t)((h - t) & (GPS_UART_RX_BUF - 1u));
}

static uint8_t tx_free(void)
{
    uint8_t h, t;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        h = s_tx_head;
        t = s_tx_tail;
    }
    return (uint8_t)((GPS_UART_TX_BUF - 1u) - (uint8_t)((h - t) & (GPS_UART_TX_BUF - 1u)));
}

static bool queue(const uint8_t *frame, uint8_t n)
{
    uint8_t i, h;
    if (n > tx_free()) {
        s_tx_refused++;
        return false;
    }
    h = s_tx_head;                                    /* only this function writes the head */
    for (i = 0; i < n; i++) {
        s_tx[h] = frame[i];
        h = (uint8_t)((h + 1u) & (GPS_UART_TX_BUF - 1u));
#ifdef FC_SIMBENCH
        s_snoop[s_snoop_head] = frame[i];
        s_snoop_head = (uint8_t)((s_snoop_head + 1u) % GPS_SNOOP_N);
#endif
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        s_tx_head = h;
        /* TXC1 cleared once per frame (write 1; keep U2X1 / MPCM1): set again only after the frame's last byte */
        UCSR1A = (uint8_t)((UCSR1A & (uint8_t)((1 << U2X1) | (1 << MPCM1))) | (1 << TXC1));
        s_tx_started = true;
        UCSR1B |= (uint8_t)(1 << UDRIE1);
    }
    s_tx_bytes = (uint16_t)(s_tx_bytes + n);
    return true;
}

bool gps_uart_tx_idle(void)
{
    bool idle;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        idle = s_tx_head == s_tx_tail && (!s_tx_started || (UCSR1A & (uint8_t)(1 << TXC1)) != 0u);
    }
    return idle;
}

bool gps_uart_write(const uint8_t *frame, uint8_t n)
{
    if (s_pending_baud != 0u) {                       /* a baud switch waits for the transmitter: this frame waits too */
        if (s_defer_n != 0u || n > GPS_UART_DEFER_MAX) {
            s_tx_refused++;
            return false;
        }
        memcpy(s_defer, frame, n);
        s_defer_n = n;
        return true;
    }
    return queue(frame, n);
}

void gps_uart_poll(void)
{
    if (s_pending_baud == 0u || !gps_uart_tx_idle())
        return;
    apply_baud(s_pending_baud);
    s_pending_baud = 0;
    s_baud_changes++;
    if (s_defer_n != 0u) {
        (void)queue(s_defer, s_defer_n);
        s_defer_n = 0;
    }
}

void gps_uart_set_baud(uint32_t baud)
{
    if (baud == 0u)
        return;
    if (s_pending_baud == 0u && gps_uart_tx_idle()) {
        if (baud != s_baud)
            s_baud_changes++;
        apply_baud(baud);
    } else {
        s_pending_baud = baud;                        /* applied by gps_uart_poll once the transmitter is idle */
    }
}

uint32_t gps_uart_baud(void)
{
    return s_baud;
}

const gps_uart_stats_t *gps_uart_stats(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        s_stats.rx_bytes = (uint16_t)(s_rx_taken + (uint8_t)((s_rx_head - s_rx_tail) & (GPS_UART_RX_BUF - 1u)));
        s_stats.rx_overflow = s_rx_overflow;
        s_stats.rx_errors = s_rx_errors;
    }
    s_stats.tx_bytes = s_tx_bytes;
    s_stats.tx_refused = s_tx_refused;
    s_stats.baud_changes = s_baud_changes;
    return &s_stats;
}
#endif /* FC_ROLE_GNS */
