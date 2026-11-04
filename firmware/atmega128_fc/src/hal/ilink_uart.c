/* ilink_uart.c - see ilink_uart.h (FCU: ILINK_USART=1, COM: ILINK_USART=0). */
#include "ilink_uart.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>

#include "../params_gen.h"

#if !defined(ILINK_UBRR) || !defined(ILINK_U2X)
#error "ILINK_UBRR / ILINK_U2X missing: re-run tools/gen_firmware_params.py (params.json flight_computer.ilink)"
#endif
#if (ILINK_UART_RX_BUF & (ILINK_UART_RX_BUF - 1u)) != 0 || ILINK_UART_RX_BUF > 256u
#error "ILINK_UART_RX_BUF must be a power of two <= 256"
#endif
#if (ILINK_UART_TX_BUF & (ILINK_UART_TX_BUF - 1u)) != 0 || ILINK_UART_TX_BUF > 256u
#error "ILINK_UART_TX_BUF must be a power of two <= 256"
#endif

#if defined(FC_SIMBENCH) && !defined(FC_COSIM)
#define ILINK_HW 0                           /* single-chip simbench: rings only, no USART access */
#else
#define ILINK_HW 1
#endif

#if ILINK_HW
#if !defined(ILINK_USART)
#error "ILINK_USART (0 or 1) must be defined by the build (platformio.ini: FCU 1, COM 0)"
#elif ILINK_USART == 0
#define ILU_UDR    UDR0
#define ILU_UCSRA  UCSR0A
#define ILU_UCSRB  UCSR0B
#define ILU_UCSRC  UCSR0C
#define ILU_UBRRH  UBRR0H
#define ILU_UBRRL  UBRR0L
#define ILU_RX_VECT   USART0_RX_vect
#define ILU_UDRE_VECT USART0_UDRE_vect
#define ILU_RXCIE  RXCIE0
#define ILU_UDRIE  UDRIE0
#define ILU_RXEN   RXEN0
#define ILU_TXEN   TXEN0
#define ILU_UCSZ0  UCSZ00
#define ILU_UCSZ1  UCSZ01
#define ILU_FE     FE0
#define ILU_DOR    DOR0
#define ILU_UPE    UPE0
#define ILU_TXC    TXC0
#define ILU_U2X    U2X0
#define ILU_MPCM   MPCM0
#define ILU_RX_PORT PORTE                    /* RXD0 = PE0 */
#define ILU_RX_BIT  PE0
#elif ILINK_USART == 1
#define ILU_UDR    UDR1
#define ILU_UCSRA  UCSR1A
#define ILU_UCSRB  UCSR1B
#define ILU_UCSRC  UCSR1C
#define ILU_UBRRH  UBRR1H
#define ILU_UBRRL  UBRR1L
#define ILU_RX_VECT   USART1_RX_vect
#define ILU_UDRE_VECT USART1_UDRE_vect
#define ILU_RXCIE  RXCIE1
#define ILU_UDRIE  UDRIE1
#define ILU_RXEN   RXEN1
#define ILU_TXEN   TXEN1
#define ILU_UCSZ0  UCSZ10
#define ILU_UCSZ1  UCSZ11
#define ILU_FE     FE1
#define ILU_DOR    DOR1
#define ILU_UPE    UPE1
#define ILU_TXC    TXC1
#define ILU_U2X    U2X1
#define ILU_MPCM   MPCM1
#define ILU_RX_PORT PORTD                    /* RXD1 = PD2 */
#define ILU_RX_BIT  PD2
#else
#error "ILINK_USART must be 0 or 1"
#endif
#endif /* ILINK_HW */

static volatile uint8_t s_rx[ILINK_UART_RX_BUF];
static volatile uint8_t s_rx_head, s_rx_tail;        /* head: ISR (inject) writes, tail: ilink_uart_read */
static volatile uint16_t s_rx_overflow, s_rx_errors;
static uint16_t s_rx_taken;                          /* bytes ilink_uart_read returned (main context only) */
static volatile uint8_t s_tx[ILINK_UART_TX_BUF];
static volatile uint8_t s_tx_head, s_tx_tail;        /* head: ilink_uart_write, tail: UDRE ISR (take) */
static uint16_t s_tx_bytes;
static ilink_uart_stats_t s_stats;
#if ILINK_HW
static volatile bool s_tx_started;                   /* a byte has gone to UDR since init (TXC is meaningful) */
static volatile bool s_tx_hold;                      /* ilink_uart_tx_hold: UDRE interrupt kept off */
#endif

#if !ILINK_HW
/* one injected byte into the RX ring (single-chip simbench, inside an atomic block): the C form of the RX ISR below */
static inline void rx_put(uint8_t b)
{
    const uint8_t h = s_rx_head, n = (uint8_t)((h + 1u) & (ILINK_UART_RX_BUF - 1u));
    if (n == s_rx_tail) {
        s_rx_overflow++;
        return;
    }
    s_rx[h] = b;
    s_rx_head = n;
}
#endif

#if ILINK_HW
#define ILU_RX_ERR_MASK ((1 << ILU_FE) | (1 << ILU_DOR) | (1 << ILU_UPE))
#if ILU_RX_ERR_MASK > 255 || ILINK_UART_RX_BUF - 1u > 255
#error "RX ISR immediates must be 8-bit"
#endif
/* RX complete, once per received byte (every 40 us while the peer sends) - and, on the FCU, also inside the
 * time-critical part, which no ilink_uart call can hold off.  Hand-written so the per-byte cost is minimal; it is
 *     st = UCSRA;                                      status first: reading UDR clears FE / DOR / UPE
 *     if (st & (FE | DOR | UPE)) { (void)UDR; s_rx_errors++; }                byte dropped, counted
 *     else if ((head + 1) % RX_BUF == tail) { (void)UDR; s_rx_overflow++; }   ring full: dropped, counted
 *     else { head' = (head + 1) % RX_BUF; s_rx[head] = UDR; head = head'; }
 * The normal path is straight-line (the two rare paths branch out of it and return through the same epilogue):
 * 4 cycles interrupt response + 3 vector JMP + 40 body + 4 RETI = 51 CPU cycles (3.2 us at 16 MHz) per byte, against 78
 * for the avr-gcc 7.3 -Os C version (it saves r0, r1, RAMPZ and five registers).  Uses r24, r30, r31 and SREG only;
 * RAMPZ is not touched (ST Z on a part with < 64 KB of RAM ignores it).  The new head is stored before the byte: the
 * main context runs only between interrupts, so it cannot see the two stores apart.
 * firmware/atmega128_fc/tools/run_simbench.py re-counts the normal path from the fcu ELF's disassembly. */
ISR(ILU_RX_VECT, ISR_NAKED)
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
        : [ucsra] "n"(_SFR_MEM_ADDR(ILU_UCSRA)), [udr] "n"(_SFR_MEM_ADDR(ILU_UDR)), [err] "M"(ILU_RX_ERR_MASK),
          [mask] "M"(ILINK_UART_RX_BUF - 1u), [head] "i"(&s_rx_head), [tail] "i"(&s_rx_tail), [ring] "i"(&s_rx[0]),
          [errs] "i"(&s_rx_errors), [ovfs] "i"(&s_rx_overflow)
        : "memory");
}

/* UDR empty, once per byte sent (every 40 us while a frame goes out).  v0.6.3: hand-written like the RX ISR, because
 * on the FCU it can run while the NAV's nlink exchange is in progress, and every FCU interrupt must stay under ~50
 * cycles there (hal/nlink_spi.h Timing; the v0.6 C version took 81).  The leanest form of
 *     if (head == tail) UDRIE off; else { UDR = s_tx[tail]; tail = (tail + 1) % TX_BUF; }
 * TXC is no longer cleared per byte: ilink_uart_write clears it once when it queues a frame, and the bytes then follow
 * each other without a gap, so TXC = 1 still means "everything shifted out" (ilink_uart_tx_busy).
 * The send path is straight-line: 4 response + 3 vector JMP + 36 body + 4 RETI = 47 CPU cycles (2.9 us) a byte.  Uses
 * r24, r30, r31 and SREG only.  firmware/atmega128_fc/tools/run_simbench.py re-counts it from the fcu ELF. */
#if ILINK_UART_TX_BUF - 1u > 255
#error "UDRE ISR mask must be 8-bit"
#endif
ISR(ILU_UDRE_VECT, ISR_NAKED)
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
        : [tail] "i"(&s_tx_tail), [head] "i"(&s_tx_head), [ring] "i"(&s_tx[0]), [mask] "M"(ILINK_UART_TX_BUF - 1u),
          [udr] "n"(_SFR_MEM_ADDR(ILU_UDR)), [ucsrb] "n"(_SFR_MEM_ADDR(ILU_UCSRB)),
          [noudrie] "M"((uint8_t)~(1u << ILU_UDRIE) & 0xFFu)
        : "memory");
}
#endif

void ilink_uart_init(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        s_rx_head = s_rx_tail = 0;
        s_tx_head = s_tx_tail = 0;
        s_rx_overflow = s_rx_errors = 0;
        s_rx_taken = 0;
        s_tx_bytes = 0;
#if ILINK_HW
        s_tx_started = false;
        s_tx_hold = false;
        ILU_RX_PORT |= (uint8_t)(1 << ILU_RX_BIT);  /* pull-up: an absent / resetting peer reads idle-high, not noise */
        ILU_UBRRH = (uint8_t)((ILINK_UBRR) >> 8);
        ILU_UBRRL = (uint8_t)((ILINK_UBRR) & 0xFF);
        ILU_UCSRA = (ILINK_U2X) ? (uint8_t)(1 << ILU_U2X) : (uint8_t)0;
        ILU_UCSRC = (uint8_t)((1 << ILU_UCSZ1) | (1 << ILU_UCSZ0));          /* 8N1, asynchronous */
        ILU_UCSRB = (uint8_t)((1 << ILU_RXEN) | (1 << ILU_TXEN) | (1 << ILU_RXCIE));
#endif
    }
}

uint8_t ilink_uart_read(uint8_t *buf, uint8_t max)
{
    uint8_t n = 0, h, t = s_rx_tail;                 /* only this function writes the tail */
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        h = s_rx_head;
    }
    while (t != h && n < max) {
        buf[n++] = s_rx[t];
        t = (uint8_t)((t + 1u) & (ILINK_UART_RX_BUF - 1u));
    }
    s_rx_tail = t;                                   /* single-byte store: atomic */
    s_rx_taken = (uint16_t)(s_rx_taken + n);
    return n;
}

uint8_t ilink_uart_rx_count(void)
{
    /* head (ISR / inject) and tail (ilink_uart_read) are single bytes: each read is atomic, and a byte stored
     * between the two reads only makes the count one too small, never wrong for the bytes it reports */
    const uint8_t h = s_rx_head, t = s_rx_tail;
    return (uint8_t)((h - t) & (ILINK_UART_RX_BUF - 1u));
}

uint8_t ilink_uart_tx_free(void)
{
    uint8_t h, t;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        h = s_tx_head;
        t = s_tx_tail;
    }
    return (uint8_t)((ILINK_UART_TX_BUF - 1u) - (uint8_t)((h - t) & (ILINK_UART_TX_BUF - 1u)));
}

bool ilink_uart_write(const uint8_t *frame, uint8_t n)
{
    uint8_t i, h;
    if (n > ilink_uart_tx_free())
        return false;
    h = s_tx_head;                                   /* only this function writes the head */
    for (i = 0; i < n; i++) {
        s_tx[h] = frame[i];
        h = (uint8_t)((h + 1u) & (ILINK_UART_TX_BUF - 1u));
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        s_tx_head = h;
#if ILINK_HW
        /* TXC cleared once per frame (write 1; keep U2X / MPCM): it sets again only when the shift register runs dry,
         * i.e. after this frame's last byte (the UDRE ISR feeds the bytes back to back) */
        ILU_UCSRA = (uint8_t)((ILU_UCSRA & (uint8_t)((1 << ILU_U2X) | (1 << ILU_MPCM))) | (1 << ILU_TXC));
        s_tx_started = true;
        if (!s_tx_hold)
            ILU_UCSRB |= (uint8_t)(1 << ILU_UDRIE);
#endif
    }
    s_tx_bytes = (uint16_t)(s_tx_bytes + n);
    return true;
}

bool ilink_uart_tx_busy(void)
{
    bool busy;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        busy = s_tx_head != s_tx_tail;
#if ILINK_HW
        if (!busy && s_tx_started)
            busy = (ILU_UCSRA & (uint8_t)(1 << ILU_TXC)) == 0u;   /* last byte still in UDR / the shift register */
#endif
    }
    return busy;
}

void ilink_uart_tx_hold(bool hold)
{
#if ILINK_HW
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        s_tx_hold = hold;
        if (hold) {
            ILU_UCSRB &= (uint8_t)~(1 << ILU_UDRIE); /* the byte in UDR / the shift register still goes out */
        } else if (s_tx_head != s_tx_tail) {
            /* the shift register may have run dry during the hold (TXC set): clear it for the bytes still queued */
            ILU_UCSRA = (uint8_t)((ILU_UCSRA & (uint8_t)((1 << ILU_U2X) | (1 << ILU_MPCM))) | (1 << ILU_TXC));
            ILU_UCSRB |= (uint8_t)(1 << ILU_UDRIE);
        }
    }
#else
    (void)hold;                                      /* single-chip simbench: the synthetic peer takes the bytes */
#endif
}

const ilink_uart_stats_t *ilink_uart_stats(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        s_stats.rx_bytes = (uint16_t)(s_rx_taken + (uint8_t)((s_rx_head - s_rx_tail) & (ILINK_UART_RX_BUF - 1u)));
        s_stats.rx_overflow = s_rx_overflow;
        s_stats.rx_errors = s_rx_errors;
    }
    s_stats.tx_bytes = s_tx_bytes;
    return &s_stats;
}

#if !ILINK_HW
void ilink_uart_inject_rx(const uint8_t *b, uint8_t n)
{
    uint8_t i;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        for (i = 0; i < n; i++) {
            const uint16_t before = s_rx_overflow;
            rx_put(b[i]);
            if (s_rx_overflow != before) {           /* ring full: count every byte not stored */
                s_rx_overflow = (uint16_t)(s_rx_overflow + (uint8_t)(n - i - 1u));
                break;
            }
        }
    }
}

uint8_t ilink_uart_take_tx(uint8_t *b, uint8_t max)
{
    uint8_t n = 0, h;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        h = s_tx_head;
    }
    while (s_tx_tail != h && n < max) {
        b[n++] = s_tx[s_tx_tail];
        s_tx_tail = (uint8_t)((s_tx_tail + 1u) & (ILINK_UART_TX_BUF - 1u));
    }
    return n;
}
#endif
