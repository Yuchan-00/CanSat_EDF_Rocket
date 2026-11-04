/* uart_log.c - see uart_log.h */
#include "uart_log.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>

#include "../fc_mode.h"
#include "../params_gen.h"

/* v0.8: the VTOL NAV's flight build has no console (the NAV has no log port; uart_log is its simbench console) - the
 * ring and the UDRE vector are left out, 256 bytes of RAM for the GPS side's stack margin.  The ballistic NAV keeps
 * them (its image stays v0.7.1's), the FCU uses this port for the flight log. */
#if !(defined(FC_ROLE_NAV) && FC_VTOL && !defined(FC_SIMBENCH))

#if (UART_LOG_BUF & (UART_LOG_BUF - 1)) != 0 || UART_LOG_BUF > 256
#error "UART_LOG_BUF must be a power of two <= 256 (uint8_t indices)"
#endif

static volatile uint8_t s_buf[UART_LOG_BUF];
static volatile uint8_t s_head, s_tail;      /* head: next write, tail: next send; full = UART_LOG_BUF-1 bytes */
static uint16_t s_dropped;

/* UDR empty, once per byte sent (every 174 us at 57600 baud): the flight log keeps the port ~90 % busy, so this
 * interrupt also runs ~18 times inside every time-critical part.  Hand-written, the leanest form of
 *     if (head == tail) UDRIE off; else { UDR = s_buf[tail]; tail = (tail + 1) % BUF; }
 * The send path is straight-line: 4 response + 3 vector JMP + 35 body + 4 RETI = 46 CPU cycles (2.9 us) a byte,
 * against 70 for the avr-gcc 7.3 -Os C version.  Uses r24, r30, r31 and SREG only. */
#if (UART_LOG_BUF - 1) > 255
#error "UDRE ISR mask must be 8-bit"
#endif
ISR(USART0_UDRE_vect, ISR_NAKED)
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
        "subi r30, lo8(-(%[buf]))"      "\n\t"   /* Z = &s_buf[old tail] */
        "sbci r31, hi8(-(%[buf]))"      "\n\t"
        "ld   r24, Z"                   "\n\t"
        "out  %[udr], r24"              "\n\t"
        "1:"                            "\n\t"
        "pop  r31"                      "\n\t"
        "pop  r30"                      "\n\t"
        "pop  r24"                      "\n\t"
        "out  __SREG__, r24"            "\n\t"
        "pop  r24"                      "\n\t"
        "reti"                          "\n\t"
        "2:"                            "\n\t"
        "cbi  %[ucsrb], %[udrie]"       "\n\t"   /* nothing to send: interrupt off until the next write */
        "rjmp 1b"                       "\n\t"
        :
        : [tail] "i"(&s_tail), [head] "i"(&s_head), [buf] "i"(&s_buf[0]), [mask] "M"(UART_LOG_BUF - 1u),
          [udr] "I"(_SFR_IO_ADDR(UDR0)), [ucsrb] "I"(_SFR_IO_ADDR(UCSR0B)), [udrie] "I"(UDRIE0)
        : "memory");
}

void uart_log_init(void)
{
    UBRR0H = (uint8_t)(FC_LOG_UBRR >> 8);
    UBRR0L = (uint8_t)(FC_LOG_UBRR & 0xFF);
    UCSR0A = FC_LOG_U2X ? (1 << U2X0) : 0;
    UCSR0C = (uint8_t)((1 << UCSZ01) | (1 << UCSZ00));     /* 8N1 */
    UCSR0B = (uint8_t)(1 << TXEN0);                       /* TX only: PE0 stays free for ISP */
}

uint16_t uart_log_free(void)
{
    uint8_t h, t;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        h = s_head;
        t = s_tail;
    }
    return (uint16_t)((UART_LOG_BUF - 1u) - (uint8_t)((h - t) & (UART_LOG_BUF - 1)));
}

bool uart_log_write(const uint8_t *data, uint16_t len)
{
    uint16_t i;
    uint8_t h;
    if (len > uart_log_free()) {
        s_dropped++;
        return false;
    }
    h = s_head;                                  /* only this function writes s_head */
    for (i = 0; i < len; i++) {
        s_buf[h] = data[i];
        h = (uint8_t)((h + 1) & (UART_LOG_BUF - 1));
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        s_head = h;
        UCSR0B |= (uint8_t)(1 << UDRIE0);
    }
    return true;
}

uint16_t uart_log_dropped(void)
{
    return s_dropped;
}

static uint8_t s_wpos;                       /* streaming: the next byte's index (beyond s_head until committed) */

bool uart_log_begin(uint16_t len)
{
    if (len > uart_log_free())
        return false;
    s_wpos = s_head;                         /* only the main context writes s_head */
    return true;
}

void uart_log_put(const uint8_t *b, uint8_t n)
{
    uint8_t i, w = s_wpos;
    for (i = 0; i < n; i++) {
        s_buf[w] = b[i];
        w = (uint8_t)((w + 1) & (UART_LOG_BUF - 1));
    }
    s_wpos = w;
}

void uart_log_commit(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        s_head = s_wpos;
        UCSR0B |= (uint8_t)(1 << UDRIE0);
    }
}
#endif /* not the VTOL NAV flight build */
