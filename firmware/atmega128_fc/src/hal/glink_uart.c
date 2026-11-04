/* glink_uart.c - see glink_uart.h (NAV: GLINK_USART=1, GNS: GLINK_USART=0; v0.8.1 four-chip build). */
#include "glink_uart.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>

#include "../fc_mode.h"
#include "../params_gen.h"
#if defined(FC_SIMBENCH)
#include "timer.h"
#endif

/* the VTOL NAV and the GNS only (the ballistic NAV compiles this file to nothing: its image stays v0.7.1's; the FCU and
 * the COM do not list it) */
#if (defined(FC_ROLE_NAV) && FC_VTOL) || defined(FC_ROLE_GNS)

#if !defined(GLINK_UBRR) || !defined(GLINK_U2X)
#error "GLINK_UBRR / GLINK_U2X missing: re-run tools/gen_firmware_params.py (params.json flight_computer.glink)"
#endif
#if (GLINK_UART_RX_BUF & (GLINK_UART_RX_BUF - 1u)) != 0 || GLINK_UART_RX_BUF > 256u
#error "GLINK_UART_RX_BUF must be a power of two <= 256"
#endif
#if (GLINK_UART_TX_BUF & (GLINK_UART_TX_BUF - 1u)) != 0 || GLINK_UART_TX_BUF > 256u
#error "GLINK_UART_TX_BUF must be a power of two <= 256"
#endif

#if GLINK_UART_HW
#if !defined(GLINK_USART)
#error "GLINK_USART (0 or 1) must be defined by the build (platformio.ini: NAV 1, GNS 0)"
#elif defined(FC_ROLE_NAV) && GLINK_USART != 1
#error "the NAV's glink is USART1 (params flight_computer.gns.pins: NAV PD3 TXD1 -> GNS PE0 RXD0)"
#elif defined(FC_ROLE_GNS) && GLINK_USART != 0
#error "the GNS's glink is USART0 (params flight_computer.gns.pins glink_rx PE0 / glink_tx PE1)"
#elif GLINK_USART == 0
#define GLU_UDR    UDR0
#define GLU_UCSRA  UCSR0A
#define GLU_UCSRB  UCSR0B
#define GLU_UCSRC  UCSR0C
#define GLU_UBRRH  UBRR0H
#define GLU_UBRRL  UBRR0L
#define GLU_RX_VECT   USART0_RX_vect
#define GLU_UDRE_VECT USART0_UDRE_vect
#define GLU_RXCIE  RXCIE0
#define GLU_UDRIE  UDRIE0
#define GLU_RXEN   RXEN0
#define GLU_TXEN   TXEN0
#define GLU_UCSZ0  UCSZ00
#define GLU_UCSZ1  UCSZ01
#define GLU_FE     FE0
#define GLU_DOR    DOR0
#define GLU_UPE    UPE0
#define GLU_TXC    TXC0
#define GLU_U2X    U2X0
#define GLU_MPCM   MPCM0
#define GLU_RX_PORT PORTE                    /* RXD0 = PE0 */
#define GLU_RX_BIT  PE0
#elif GLINK_USART == 1
#define GLU_UDR    UDR1
#define GLU_UCSRA  UCSR1A
#define GLU_UCSRB  UCSR1B
#define GLU_UCSRC  UCSR1C
#define GLU_UBRRH  UBRR1H
#define GLU_UBRRL  UBRR1L
#define GLU_RX_VECT   USART1_RX_vect
#define GLU_UDRE_VECT USART1_UDRE_vect
#define GLU_RXCIE  RXCIE1
#define GLU_UDRIE  UDRIE1
#define GLU_RXEN   RXEN1
#define GLU_TXEN   TXEN1
#define GLU_UCSZ0  UCSZ10
#define GLU_UCSZ1  UCSZ11
#define GLU_FE     FE1
#define GLU_DOR    DOR1
#define GLU_UPE    UPE1
#define GLU_TXC    TXC1
#define GLU_U2X    U2X1
#define GLU_MPCM   MPCM1
#define GLU_RX_PORT PORTD                    /* RXD1 = PD2 */
#define GLU_RX_BIT  PD2
#else
#error "GLINK_USART must be 0 or 1"
#endif
#endif /* GLINK_UART_HW */

static volatile uint8_t s_rx[GLINK_UART_RX_BUF];
static volatile uint8_t s_rx_head, s_rx_tail;        /* head: RX ISR (bench: Timer3 ISR / inject), tail: glink_uart_read */
static volatile uint16_t s_rx_overflow, s_rx_errors;
static uint16_t s_rx_taken;                          /* bytes glink_uart_read returned (main context only) */
static volatile uint8_t s_tx[GLINK_UART_TX_BUF];
static volatile uint8_t s_tx_head, s_tx_tail;        /* head: glink_uart_write, tail: UDRE ISR (bench: take_tx) */
static uint16_t s_tx_bytes, s_tx_refused;
static glink_uart_stats_t s_stats;
#if GLINK_UART_HW
static volatile bool s_tx_started;                   /* a byte has gone to UDR since init (TXC is meaningful) */
#endif

#if GLINK_UART_HW
#define GLU_RX_ERR_MASK ((1 << GLU_FE) | (1 << GLU_DOR) | (1 << GLU_UPE))
#if GLU_RX_ERR_MASK > 255 || GLINK_UART_RX_BUF - 1u > 255 || GLINK_UART_TX_BUF - 1u > 255
#error "ISR immediates must be 8-bit"
#endif
/* RX complete, once per received byte (every 40 us while the peer sends: the NAV 26 a cycle, the GNS 60).  The ilink's
 * hand-written RX ISR (hal/ilink_uart.c) on this USART:
 *     st = UCSRA;                                      status first: reading UDR clears FE / DOR / UPE
 *     if (st & (FE | DOR | UPE)) { (void)UDR; s_rx_errors++; }                byte dropped, counted
 *     else if ((head + 1) % RX_BUF == tail) { (void)UDR; s_rx_overflow++; }   ring full: dropped, counted
 *     else { head' = (head + 1) % RX_BUF; s_rx[head] = UDR; head = head'; }
 * Normal path: 4 response + 3 vector JMP + 40 body + 4 RETI = 51 CPU cycles (3.2 us) a byte.  Uses r24, r30, r31 and SREG
 * only.  tools/run_nav_simbench.py / run_gns_simbench.py re-count it from the ELFs' disassembly. */
ISR(GLU_RX_VECT, ISR_NAKED)
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
        : [ucsra] "n"(_SFR_MEM_ADDR(GLU_UCSRA)), [udr] "n"(_SFR_MEM_ADDR(GLU_UDR)), [err] "M"(GLU_RX_ERR_MASK),
          [mask] "M"(GLINK_UART_RX_BUF - 1u), [head] "i"(&s_rx_head), [tail] "i"(&s_rx_tail), [ring] "i"(&s_rx[0]),
          [errs] "i"(&s_rx_errors), [ovfs] "i"(&s_rx_overflow)
        : "memory");
}

/* UDR empty, once per byte sent: if (head == tail) UDRIE off; else UDR = s_tx[tail++].  The ilink's hand-written UDRE
 * ISR: 4 response + 3 vector JMP + 36 body + 4 RETI = 47 CPU cycles (2.9 us) a byte.  Uses r24, r30, r31 and SREG only.
 * TXC is cleared once per frame by glink_uart_write (the bytes follow back to back: TXC = 1 means all shifted out). */
ISR(GLU_UDRE_VECT, ISR_NAKED)
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
        : [tail] "i"(&s_tx_tail), [head] "i"(&s_tx_head), [ring] "i"(&s_tx[0]), [mask] "M"(GLINK_UART_TX_BUF - 1u),
          [udr] "n"(_SFR_MEM_ADDR(GLU_UDR)), [ucsrb] "n"(_SFR_MEM_ADDR(GLU_UCSRB)),
          [noudrie] "M"((uint8_t)~(1u << GLU_UDRIE) & 0xFFu)
        : "memory");
}
#endif /* GLINK_UART_HW */

#if defined(FC_SIMBENCH) && (GLINK_UART_BENCH_WIRE || !GLINK_UART_HW)
/* the bench's view of the frames queued for sending (NAV single-chip: copies of what goes out of the real USART1;
 * GNS single-chip: the TX ring itself, below) and when the newest was queued */
static uint32_t s_tq_time;
#endif
#if defined(FC_SIMBENCH) && GLINK_UART_BENCH_WIRE
/* fw09 W1: the bench wire writes Timer3 (OCR3A, TCCR3A / B); in a NAV build with the real nlink (flight, co-simulation:
 * hal/nlink_spi.c NLINK_HW) Timer3 is the exchange's alone (clk / 1, reset to 0 at every exchange, the W1 watchdog) */
#if defined(FC_ROLE_NAV) && (!defined(FC_SIMBENCH) || defined(FC_COSIM))
#error "glink bench wire in a NAV build with the hardware nlink: Timer3 belongs to hal/nlink_spi.c (W1)"
#endif
#define GL_SNOOP_N 64u                               /* power of two, > one STATE frame (60 B) */
static uint8_t s_snoop[GL_SNOOP_N];
static uint8_t s_snoop_head, s_snoop_tail;

/* ---- the synthetic GNS's wire towards RXD1 (simavr has no peer there): a Timer3 compare interrupt per byte ---- */
#define GL_BQ_N GLINK_UART_BENCH_Q
#define GL_BYTE_T3_N (10UL * FC_F_CPU_HZ / GLINK_BAUD / 8UL)
#define GL_BYTE_T3 ((uint16_t)GL_BYTE_T3_N)                  /* one 8N1 byte time in Timer3 counts at clk / 8 */
#if GL_BYTE_T3_N * 8UL * GLINK_BAUD != 10UL * FC_F_CPU_HZ
#error "the bench wire needs a byte time that is a whole number of clk / 8 periods (GLINK_BAUD)"
#endif
static volatile uint8_t s_bq[GL_BQ_N];
static volatile uint8_t s_bq_head, s_bq_tail;

/* Timer3 compare A (CTC, clk / 8): the next queued byte into the RX ring - the RX ISR's ring code with the byte taken from
 * the bench queue instead of UDR1, then OCR3A = one byte time (the first compare was the frame's delay); no byte left:
 * the interrupt switches itself off.  Normal path 4 response + 3 JMP + 58 body + 4 RETI = 69 cycles (18 more than the
 * RX ISR: the queue pop and the compare).  Uses r24, r25, r30, r31, SREG. */
ISR(TIMER3_COMPA_vect, ISR_NAKED)
{
    __asm__ __volatile__(
        "push r24"                      "\n\t"
        "in   r24, __SREG__"            "\n\t"
        "push r24"                      "\n\t"
        "push r25"                      "\n\t"
        "push r30"                      "\n\t"
        "push r31"                      "\n\t"
#if FC_VTOL
        "lds  r24, %[ocrl]"             "\n\t"   /* P1: OCR3A += one byte time (Timer3 free running, shared) */
        "lds  r25, %[ocrh]"             "\n\t"
        "subi r24, %[bt_lo]"            "\n\t"
        "sbci r25, %[bt_hi]"            "\n\t"
        "sts  %[ocrh], r25"             "\n\t"
        "sts  %[ocrl], r24"             "\n\t"
#else
        "ldi  r24, %[bt_hi]"            "\n\t"   /* OCR3A = one byte time - 1 (high byte first: TEMP) */
        "sts  %[ocrh], r24"             "\n\t"
        "ldi  r24, %[bt_lo]"            "\n\t"
        "sts  %[ocrl], r24"             "\n\t"
#endif
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
          [tail] "i"(&s_rx_tail), [ring] "i"(&s_rx[0]), [mask] "M"(GLINK_UART_RX_BUF - 1u), [ovfs] "i"(&s_rx_overflow),
          [etimsk] "n"(_SFR_MEM_ADDR(ETIMSK)), [noocie] "M"((uint8_t)~(1u << OCIE3A) & 0xFFu),
          [qmask] "M"(GL_BQ_N - 1u), [ocrh] "n"(_SFR_MEM_ADDR(OCR3AH)), [ocrl] "n"(_SFR_MEM_ADDR(OCR3AL)),
#if FC_VTOL
          [bt_hi] "M"(((65536UL - GL_BYTE_T3_N) >> 8) & 0xFFUL), [bt_lo] "M"((65536UL - GL_BYTE_T3_N) & 0xFFUL)
#else
          [bt_hi] "M"((uint8_t)((GL_BYTE_T3 - 1u) >> 8)), [bt_lo] "M"((uint8_t)((GL_BYTE_T3 - 1u) & 0xFFu))
#endif
        : "memory");
}

bool glink_uart_bench_busy(void)
{
    bool busy;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        busy = s_bq_head != s_bq_tail || (ETIMSK & (uint8_t)(1 << OCIE3A)) != 0u;
    }
    return busy;
}

uint8_t glink_uart_bench_send(const uint8_t *b, uint8_t n, uint32_t delay_cycles)
{
    uint8_t i = 0, h;
    uint32_t c = delay_cycles / 8UL;
    if (glink_uart_bench_busy() || n == 0u)
        return 0;
    if (c < GL_BYTE_T3) c = GL_BYTE_T3;
    if (c > 65535UL) c = 65535UL;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        h = s_bq_head;
        for (i = 0; i < n && (uint8_t)((h + 1u) & (GL_BQ_N - 1u)) != s_bq_tail; i++) {
            s_bq[h] = b[i];
            h = (uint8_t)((h + 1u) & (GL_BQ_N - 1u));
        }
        s_bq_head = h;
#if FC_VTOL
        if (TCCR3B != (uint8_t)(1 << CS31) || TCCR3A != 0u) {   /* P1: Timer3 free running, clk / 8 (nlink_spi_init) */
            TCCR3A = 0;
            TCCR3B = (uint8_t)(1 << CS31);
        }
        OCR3A = (uint16_t)(TCNT3 + (uint16_t)c);        /* the first byte completes after the frame's delay */
        ETIFR = (uint8_t)(1 << OCF3A);
        ETIMSK |= (uint8_t)(1 << OCIE3A);
#else
        TCCR3B = 0;
        TCCR3A = 0;
        TCNT3 = 0;
        OCR3A = (uint16_t)(c - 1u);                     /* the first byte completes after the frame's delay */
        ETIFR = (uint8_t)(1 << OCF3A);
        ETIMSK |= (uint8_t)(1 << OCIE3A);
        TCCR3B = (uint8_t)((1 << WGM32) | (1 << CS31));  /* CTC on OCR3A, clk / 8 */
#endif
    }
    return i;
}
#endif /* FC_SIMBENCH && GLINK_UART_BENCH_WIRE */

void glink_uart_init(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        s_rx_head = s_rx_tail = 0;
        s_tx_head = s_tx_tail = 0;
        s_rx_overflow = s_rx_errors = 0;
        s_rx_taken = 0;
        s_tx_bytes = s_tx_refused = 0;
#if GLINK_UART_HW
        s_tx_started = false;
        GLU_RX_PORT |= (uint8_t)(1 << GLU_RX_BIT);   /* pull-up: an absent / resetting peer reads idle-high, not noise */
        GLU_UCSRB = 0;
        GLU_UBRRH = (uint8_t)((GLINK_UBRR) >> 8);
        GLU_UBRRL = (uint8_t)((GLINK_UBRR) & 0xFF);
        GLU_UCSRA = (GLINK_U2X) ? (uint8_t)(1 << GLU_U2X) : (uint8_t)0;
        GLU_UCSRC = (uint8_t)((1 << GLU_UCSZ1) | (1 << GLU_UCSZ0));          /* 8N1, asynchronous */
        GLU_UCSRB = (uint8_t)((1 << GLU_RXEN) | (1 << GLU_TXEN) | (1 << GLU_RXCIE));
#endif
#if defined(FC_SIMBENCH) && GLINK_UART_BENCH_WIRE
        s_snoop_head = s_snoop_tail = 0;
        s_bq_head = s_bq_tail = 0;
#endif
    }
}

uint8_t glink_uart_read(uint8_t *buf, uint8_t max)
{
    uint8_t n = 0, h, t = s_rx_tail;                 /* only this function writes the tail */
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        h = s_rx_head;
    }
    while (t != h && n < max) {
        buf[n++] = s_rx[t];
        t = (uint8_t)((t + 1u) & (GLINK_UART_RX_BUF - 1u));
    }
    s_rx_tail = t;                                   /* single-byte store: atomic */
    s_rx_taken = (uint16_t)(s_rx_taken + n);
    return n;
}

uint8_t glink_uart_rx_count(void)
{
    const uint8_t h = s_rx_head, t = s_rx_tail;      /* single bytes: each read atomic (a byte between: one too few) */
    return (uint8_t)((h - t) & (GLINK_UART_RX_BUF - 1u));
}

static uint8_t tx_free(void)
{
    uint8_t h, t;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        h = s_tx_head;
        t = s_tx_tail;
    }
    return (uint8_t)((GLINK_UART_TX_BUF - 1u) - (uint8_t)((h - t) & (GLINK_UART_TX_BUF - 1u)));
}

bool glink_uart_write(const uint8_t *frame, uint8_t n)
{
    uint8_t i, h;
    if (n > tx_free()) {
        s_tx_refused++;
        return false;
    }
    h = s_tx_head;                                   /* only this function writes the head */
    for (i = 0; i < n; i++) {
        s_tx[h] = frame[i];
        h = (uint8_t)((h + 1u) & (GLINK_UART_TX_BUF - 1u));
#if defined(FC_SIMBENCH) && GLINK_UART_BENCH_WIRE
        s_snoop[s_snoop_head] = frame[i];
        s_snoop_head = (uint8_t)((s_snoop_head + 1u) & (GL_SNOOP_N - 1u));
#endif
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        s_tx_head = h;
#if GLINK_UART_HW
        /* TXC cleared once per frame (write 1; keep U2X / MPCM): set again only after this frame's last byte */
        GLU_UCSRA = (uint8_t)((GLU_UCSRA & (uint8_t)((1 << GLU_U2X) | (1 << GLU_MPCM))) | (1 << GLU_TXC));
        s_tx_started = true;
        GLU_UCSRB |= (uint8_t)(1 << GLU_UDRIE);
#endif
    }
#if defined(FC_SIMBENCH) && (GLINK_UART_BENCH_WIRE || !GLINK_UART_HW)
    s_tq_time = timer_sim_now();
#endif
    s_tx_bytes = (uint16_t)(s_tx_bytes + n);
    return true;
}

bool glink_uart_tx_busy(void)
{
    bool busy;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        busy = s_tx_head != s_tx_tail;
#if GLINK_UART_HW
        if (!busy && s_tx_started)
            busy = (GLU_UCSRA & (uint8_t)(1 << GLU_TXC)) == 0u;   /* last byte still in UDR / the shift register */
#endif
    }
    return busy;
}

const glink_uart_stats_t *glink_uart_stats(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        s_stats.rx_bytes = (uint16_t)(s_rx_taken + (uint8_t)((s_rx_head - s_rx_tail) & (GLINK_UART_RX_BUF - 1u)));
        s_stats.rx_overflow = s_rx_overflow;
        s_stats.rx_errors = s_rx_errors;
    }
    s_stats.tx_bytes = s_tx_bytes;
    s_stats.tx_refused = s_tx_refused;
    return &s_stats;
}

#if defined(FC_SIMBENCH) && GLINK_UART_BENCH_WIRE
uint8_t glink_uart_bench_take_tx(uint8_t *b, uint8_t max, uint32_t *t_queued)
{
    uint8_t n = 0;
    while (s_snoop_tail != s_snoop_head && n < max) {
        b[n++] = s_snoop[s_snoop_tail];
        s_snoop_tail = (uint8_t)((s_snoop_tail + 1u) & (GL_SNOOP_N - 1u));
    }
    *t_queued = s_tq_time;
    return n;
}
#endif

#if defined(FC_SIMBENCH) && !GLINK_UART_HW
void glink_uart_bench_inject(const uint8_t *b, uint8_t n)
{
    uint8_t i;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        for (i = 0; i < n; i++) {
            const uint8_t h = s_rx_head, nx = (uint8_t)((h + 1u) & (GLINK_UART_RX_BUF - 1u));
            if (nx == s_rx_tail) {
                s_rx_overflow = (uint16_t)(s_rx_overflow + (uint8_t)(n - i));
                break;
            }
            s_rx[h] = b[i];
            s_rx_head = nx;
        }
    }
}

uint8_t glink_uart_bench_take_tx(uint8_t *b, uint8_t max, uint32_t *t_queued)
{
    uint8_t n = 0, h;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        h = s_tx_head;
    }
    while (s_tx_tail != h && n < max) {
        b[n++] = s_tx[s_tx_tail];
        s_tx_tail = (uint8_t)((s_tx_tail + 1u) & (GLINK_UART_TX_BUF - 1u));
    }
    *t_queued = s_tq_time;
    return n;
}
#endif

#endif /* (FC_ROLE_NAV && FC_VTOL) || FC_ROLE_GNS */
