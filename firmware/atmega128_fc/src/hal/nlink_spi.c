/* nlink_spi.c - see nlink_spi.h (NAV: SPI master, FCU: SPI slave; mode 0, MSB first). */
#include "nlink_spi.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <string.h>
#include <util/atomic.h>

#include "../fc_mode.h"
#include "../params_gen.h"

#if !defined(NLINK_SPI_SPR) || !defined(NLINK_SPI_2X) || !defined(NLINK_BYTE_GAP_CYCLES)
#error "NLINK_SPI_SPR / NLINK_SPI_2X / NLINK_BYTE_GAP_CYCLES missing: re-run tools/gen_firmware_params.py (flight_computer.nlink)"
#endif
#if NL_XFER_BYTES != 72 || NL_FCU_FRAME_BYTES != 10 || NL_FCU4_FRAME_BYTES != 13
#error "nlink_spi.c's buffer offsets (73-byte reply buffers, the end-of-exchange compare) assume 72 / 10 / 13 bytes"
#endif
/* the FCU frame the slave returns: v0.8.1 VTOL the nlink v4 frame (NL_TYPE_FCU4, 13 bytes: + pflags, kt_q - the
 * reference's SpiSlaveModel(reply_bytes=13)); the ballistic build the v0.6.3 frame (10 bytes, its image unchanged) */
#if FC_VTOL
#define NL_REPLY_BYTES NL_FCU4_FRAME_BYTES
#else
#define NL_REPLY_BYTES NL_FCU_FRAME_BYTES
#endif

#if defined(FC_SIMBENCH) && !defined(FC_COSIM)
#define NLINK_HW 0                   /* single-chip simbench: buffers only, no SPI register, no ISR */
#else
#define NLINK_HW 1
#endif

/* the ATmega128 SPI pins (PB0 SS, PB1 SCK, PB2 MOSI, PB3 MISO; params_gen.h checks params.json against them) */
#define NL_SS   PB0
#define NL_SCK  PB1
#define NL_MOSI PB2
#define NL_MISO PB3

static nlink_spi_stats_t s_snap;     /* nlink_spi_stats() copy */

/* ================================================================================================ NAV, VTOL (P1) */
#if defined(FC_ROLE_NAV) && FC_VTOL
/* v0.9 timing proposal P1 (fw09/p1 prototype): the exchange from a Timer3 compare-B interrupt (nlink_spi.h).  Per byte
 * i < 71 (hand-written, counted from the instruction list: 4-cycle response + 3 JMP + 44 body + 4 RETI = 55 cycles; SPDR
 * written 28 cycles and TCNT3 read 33 cycles after the compare):
 *     b = SPDR (the FCU's byte i: the byte ended >= 50 cycles ago); buf[i] = b; SPDR = buf[i + 1];
 *     the next compare NL_PACE counts after the TCNT3 read.
 * Flight / co-simulation builds (NLINK_HW): Timer3 (normal mode, clk / 1) is the exchange's alone and starts from 0 at
 * every exchange (~10.1k cycles: no wrap inside one); NL_PACE = 107 cycles -> write-to-write 140 cycles (+0..3 for the
 * instruction the compare interrupts): the end of a byte (64 cycles at 2 MHz) to the next write >= 76 cycles (the
 * busy-wait master: 75; the FCU slave writes its next return byte 23 cycles after its SPIF), ~632 us on the wire
 * (NLINK_XFER_US 630).  [A first version paced relative compares across the 16-bit wrap: 3 of 3750 nav_cosim exchanges
 * lost a compare and took 4.8 ms - one full wrap, every 5.12 s = 1250 wraps, a fixed phase; a polled probe
 * (fw09/p1/tools/qfix_test/t3wrap_test.c) did not reproduce it: most likely simavr's timer at the overflow cycle.  The
 * reset removes the wrap from the exchange on any chip or simulator.]  Single-chip bench (NLINK_HW 0): Timer3 at clk / 8
 * shared with the glink bench wire (hal/glink_uart.c, delays up to ~16 ms), relative compares, NL_PACE = 14 counts ->
 * 137..145 cycles a byte (the same interrupts and body).  Another NAV interrupt in the way only lengthens that gap (the
 * schedule is relative to the read).  After byte 71: SS high, the Timer1 stamp, the compare off, busy = 0 (+ 20 cycles).
 * 72 interrupts ~ 4.0k cycles (250 us) of CPU per exchange instead of the ~10k-cycle busy-wait.  SPIF is never cleared
 * (no SPI interrupt is enabled; a set SPIF does not block the next transfer).  The single-chip bench clocks the same bytes
 * through the same SPI and interrupt (2 cycles of `adiw` instead of the 2-cycle `st`: the frame stays in place for the
 * synthetic FCU, the reply is the bypass buffer) - its loop times contain the real interrupt load.
 *
 * W1 exchange watchdog (fw09 SPEC_firmware_v09 3.3; flight and co-simulation builds, NLINK_HW): an exchange whose compare
 * interrupt is lost (another driver clearing OCIE3B or reprogramming Timer3, a glitch) would leave s_busy set and the NAV
 * spinning in nlink_spi_exchange_wait until the hardware watchdog resets it - in flight the FCU's I9 ABORT.  So the wait
 * is bounded: while the exchange is open it reads TCNT3 (atomically: the 16-bit TEMP register is shared with the compare
 * ISR's TCNT3 read and OCR3B write) and the TOV3 flag; TCNT3 >= NLINK_WD_COUNTS (params flight_computer.nlink.
 * exchange_watchdog_us, 1000 us = 16000 counts; the exchange itself ~10.1k) or TOV3 set (Timer3 wrapped: the main loop
 * did not look for > 4.1 ms) closes it, with interrupts off and only if it is still open: compare-B interrupt off, SS
 * high, the Timer1 stamp, byte 0 of the buffer cleared (the buffer now holds the FCU's bytes 0..k-1 and the NAV's own
 * k..71: for 3 <= k <= 12 only the CRC-16 would stand between that mix and a wrong FCU state - with byte 0 cleared
 * nl_decode_fcu4 refuses it on the sync byte; the NAV keeps the last valid FCU frame), busy = 0, one trip counted
 * (nlink_spi_stats()->resyncs).  Exchange start clears TOV3 with OCF3B (one ETIFR write, no extra cycle).  The FCU sees an
 * exchange that ends early (SS high with its index != 0: a resync) or none: one missing frame, its existing timeout /
 * fallback tick - never a stalled NAV.  The watchdog only runs inside the wait, which the loop reaches after steps 3b - 6
 * (normally after the exchange ended: 0 cycles); the latest close is therefore max(start + exchange_watchdog_us, the
 * step-7 wait).  Single-chip bench builds (NLINK_HW 0): no watchdog - Timer3 runs at clk / 8 shared with the glink bench
 * wire (no TCNT3 reset per exchange), and the bench has no stall source.
 *
 * Timer3 ownership (flight NAV image): nlink_spi_init leaves TCCR3A = 0, TCCR3B = CS30 (normal mode, clk / 1) and no
 * Timer3 interrupt enabled; nlink_spi_t3_owned (nav_main.c calls it after every driver's init, before the first exchange)
 * checks that this is still so (the NAV starts no exchange if it is not: the FCU sees no NAV); compile time: the glink
 * bench wire (hal/glink_uart.c) refuses an NLINK_HW NAV build, nav_main.c refuses hal/pwm.h / hal/gps_uart.h (the other
 * Timer3 drivers). */
#if NLINK_HW
#define NL_PACE 107u                                 /* Timer3 counts = cycles (clk / 1) */
#define NL_T3_CS ((uint8_t)(1 << CS30))
#ifndef NLINK_WD_COUNTS
#error "NLINK_WD_COUNTS missing: re-run tools/gen_firmware_params.py (params flight_computer.nlink.exchange_watchdog_us, W1)"
#elif NLINK_WD_COUNTS * 2UL <= 3UL * NL_XFER_BYTES * (NL_PACE + 33UL) || NLINK_WD_COUNTS > 65535UL
#error "W1: NLINK_WD_COUNTS must exceed 1.5 x the interrupt-paced exchange (72 x 140 cycles) and fit Timer3 (16 bits)"
#endif
#define NL_T3_IRQ_BITS ((uint8_t)((1 << TICIE3) | (1 << OCIE3A) | (1 << OCIE3B) | (1 << OCIE3C) | (1 << TOIE3)))
#else
#define NL_PACE 14u                                  /* Timer3 counts of 8 cycles (clk / 8, the bench wire's clock) */
#define NL_T3_CS ((uint8_t)(1 << CS31))
#endif
#if NLINK_SPI_SPR != 1 || !NLINK_SPI_2X
#error "P1: the interrupt pacing assumes the 2 MHz SPI (64 cycles a byte; params flight_computer.nlink.spi_clock_hz)"
#endif
uint8_t nlink_spi_buf[NL_XFER_BYTES];
static uint8_t *volatile s_xp;                       /* the byte in flight (written / read by the ISR only while busy) */
static volatile uint8_t s_busy;                      /* an exchange is open (cleared by the ISR after byte 71, or by W1) */
static volatile uint16_t s_xend;                     /* Timer1 count at the SS rise of the last exchange */
static uint16_t s_exchanges;
#if NLINK_HW
static uint8_t s_wd_trips;                           /* W1: exchanges the watchdog closed (saturating; main context only) */
#else
static uint8_t s_reply[NL_XFER_BYTES];
static bool s_tx_new;
#endif

void nlink_spi_init(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        SPCR = 0;
        PORTB |= (uint8_t)(1 << NL_SS);                     /* SS idle high BEFORE it becomes an output */
        DDRB |= (uint8_t)((1 << NL_SS) | (1 << NL_SCK) | (1 << NL_MOSI));
        DDRB &= (uint8_t)~(1 << NL_MISO);
        PORTB |= (uint8_t)(1 << NL_MISO);                   /* pull-up: an absent FCU reads 0xFF (nl_decode_fcu refuses) */
        PORTB &= (uint8_t)~((1 << NL_SCK) | (1 << NL_MOSI));
        SPSR = (uint8_t)((NLINK_SPI_2X) ? (1 << SPI2X) : 0);
        /* master, mode 0 (CPOL 0, CPHA 0), MSB first, no SPI interrupt: SS is an output, so the SPI stays master */
        SPCR = (uint8_t)((1 << SPE) | (1 << MSTR) | ((NLINK_SPI_SPR) & 0x03));
        (void)SPSR;
        (void)SPDR;
        ETIMSK &= (uint8_t)~(1 << OCIE3B);
        TCCR3A = 0;                                         /* Timer3: normal mode (NL_T3_CS, see above) */
        TCCR3C = 0;
        TCCR3B = NL_T3_CS;
        s_busy = 0;
        s_xp = nlink_spi_buf;
    }
    s_exchanges = 0;
}

ISR(TIMER3_COMPB_vect, ISR_NAKED)
{
    __asm__ __volatile__(
        "push r24"                      "\n\t"
        "in   r24, __SREG__"            "\n\t"
        "push r24"                      "\n\t"
        "push r30"                      "\n\t"
        "push r31"                      "\n\t"
        "lds  r30, %[xp]"               "\n\t"
        "lds  r31, %[xp]+1"             "\n\t"
        "in   r24, %[spdr]"             "\n\t"   /* byte i from the FCU */
#if NLINK_HW
        "st   Z+, r24"                  "\n\t"   /* in place: buf[i] = the FCU's byte i */
#else
        "adiw r30, 1"                   "\n\t"   /* bench: the frame stays (same 2 cycles) */
#endif
        "cpi  r30, lo8(%[end])"         "\n\t"   /* 72 consecutive addresses: the low byte alone is unique */
        "breq 2f"                       "\n\t"
        "ld   r24, Z"                   "\n\t"   /* byte i + 1 */
        "out  %[spdr], r24"             "\n\t"
        "sts  %[xp]+1, r31"             "\n\t"
        "sts  %[xp], r30"               "\n\t"
        "lds  r30, %[t3l]"              "\n\t"   /* the next compare NL_PACE counts after this read */
        "lds  r31, %[t3h]"              "\n\t"
        "subi r30, %[pace_lo]"          "\n\t"   /* + NL_PACE (2 cycles, as adiw) */
        "sbci r31, %[pace_hi]"          "\n\t"
        "sts  %[o3bh], r31"             "\n\t"
        "sts  %[o3bl], r30"             "\n\t"
        "1:"                            "\n\t"
        "pop  r31"                      "\n\t"
        "pop  r30"                      "\n\t"
        "pop  r24"                      "\n\t"
        "out  __SREG__, r24"            "\n\t"
        "pop  r24"                      "\n\t"
        "reti"                          "\n\t"
        "2:"                            "\n\t"   /* byte 71 done: SS high, its Timer1 stamp, the compare off */
        "sbi  %[portb], %[ssbit]"       "\n\t"
        "in   r30, %[t1l]"              "\n\t"
        "in   r31, %[t1h]"              "\n\t"
        "sts  %[xend]+1, r31"           "\n\t"
        "sts  %[xend], r30"             "\n\t"
        "lds  r30, %[etimsk]"           "\n\t"
        "andi r30, %[nocie]"            "\n\t"
        "sts  %[etimsk], r30"           "\n\t"
        "clr  r30"                      "\n\t"
        "sts  %[busy], r30"             "\n\t"
        "rjmp 1b"                       "\n\t"
        :
        : [xp] "i"(&s_xp), [spdr] "I"(_SFR_IO_ADDR(SPDR)), [end] "i"(&nlink_spi_buf[NL_XFER_BYTES]),
          [t3l] "n"(_SFR_MEM_ADDR(TCNT3L)), [t3h] "n"(_SFR_MEM_ADDR(TCNT3H)), [pace_lo] "M"((65536UL - NL_PACE) & 0xFFUL),
          [pace_hi] "M"(((65536UL - NL_PACE) >> 8) & 0xFFUL),
          [o3bh] "n"(_SFR_MEM_ADDR(OCR3BH)), [o3bl] "n"(_SFR_MEM_ADDR(OCR3BL)), [portb] "I"(_SFR_IO_ADDR(PORTB)),
          [ssbit] "I"(NL_SS), [t1l] "I"(_SFR_IO_ADDR(TCNT1L)), [t1h] "I"(_SFR_IO_ADDR(TCNT1H)), [xend] "i"(&s_xend),
          [etimsk] "n"(_SFR_MEM_ADDR(ETIMSK)), [nocie] "M"((uint8_t)~(1u << OCIE3B) & 0xFFu), [busy] "i"(&s_busy)
        : "memory");
}

#if NLINK_HW
/* W1: the open exchange has run NLINK_WD_COUNTS Timer3 counts since its start (TCNT3 = 0 there), or Timer3 wrapped since
 * (TOV3, cleared at the start) */
static inline bool wd_expired(void)
{
    uint16_t c;
    uint8_t f;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {                     /* TEMP: shared with the compare ISR's TCNT3 / OCR3B access */
        c = TCNT3;
        f = ETIFR;
    }
    return (f & (uint8_t)(1 << TOV3)) != 0u || c >= (uint16_t)NLINK_WD_COUNTS;
}

/* W1: the exchange is still open - wait for its end, closing it if the watchdog expires.  Out of line: the normal path
 * (the exchange ended during steps 3b - 6, and the start's "never two at once" check) only tests s_busy, so the wait
 * stays as small as the v0.9 P1 loop and is inlined where it was */
static void wait_open(void) __attribute__((noinline));
static void wait_open(void)
{
    while (s_busy) {
        if (wd_expired()) {                                 /* the compare interrupt was lost - close it here */
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                if (s_busy) {                               /* the ISR may have ended it meanwhile */
                    ETIMSK &= (uint8_t)~(1 << OCIE3B);
                    PORTB |= (uint8_t)(1 << NL_SS);
                    s_xend = TCNT1;
                    nlink_spi_buf[0] = 0u;                  /* no FCU frame from a cut exchange (sync byte) */
                    s_busy = 0;
                    if (s_wd_trips != 0xFFu)
                        s_wd_trips++;
                }
            }
        }
    }
}
#endif

/* the end of the open exchange (W1: bounded), inlined at both callers: the normal path is one s_busy test */
static inline __attribute__((always_inline)) void wait_end(void)
{
#if NLINK_HW
    if (s_busy)
        wait_open();
#else
    while (s_busy) {
    }
#endif
}

const uint8_t *nlink_spi_exchange_wait(uint16_t *t_ss_high)
{
    uint16_t t;
    wait_end();
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { t = s_xend; }
    *t_ss_high = t;
#if NLINK_HW
    return nlink_spi_buf;
#else
    return s_reply;
#endif
}

void nlink_spi_exchange_start(void)
{
    wait_end();                                             /* never two at once */
    PORTB &= (uint8_t)~(1 << NL_SS);                        /* select: the FCU's byte 0 already sits in its SPDR */
    __builtin_avr_delay_cycles(NLINK_BYTE_GAP_CYCLES);     /* SS -> first SCK: >= one byte gap */
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        s_xp = nlink_spi_buf;
        s_busy = 1;
#if NLINK_HW
        TCNT3 = 0;                                          /* no 16-bit wrap inside the exchange (see above) */
#endif
        SPDR = nlink_spi_buf[0];
        OCR3B = (uint16_t)(TCNT3 + NL_PACE);
#if NLINK_HW
        ETIFR = (uint8_t)((1 << OCF3B) | (1 << TOV3));      /* a stale flag would fire at once; W1's wrap flag */
#else
        ETIFR = (uint8_t)(1 << OCF3B);                      /* a stale flag would fire at once */
#endif
        ETIMSK |= (uint8_t)(1 << OCIE3B);
    }
    s_exchanges++;
#if !NLINK_HW
    s_tx_new = true;
#endif
}

bool nlink_spi_t3_owned(void)
{
#if NLINK_HW
    bool ok;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        ok = TCCR3A == 0u && TCCR3B == NL_T3_CS && (ETIMSK & NL_T3_IRQ_BITS) == 0u;
    }
    return ok;
#else
    return true;                                            /* bench: Timer3 is shared with the glink bench wire */
#endif
}

#if !NLINK_HW
void nlink_spi_inject_reply(const uint8_t rx[NL_XFER_BYTES])
{
    memcpy(s_reply, rx, NL_XFER_BYTES);
}

const uint8_t *nlink_spi_take_tx_buf(void)
{
    if (!s_tx_new)
        return 0;
    s_tx_new = false;
    return nlink_spi_buf;
}

uint8_t *nlink_spi_reply_buf(void)
{
    return s_reply;
}
#endif

const nlink_spi_stats_t *nlink_spi_stats(void)
{
    s_snap.frames = s_exchanges;
#if NLINK_HW
    s_snap.resyncs = s_wd_trips;                            /* W1: exchanges closed by the watchdog */
#else
    s_snap.resyncs = 0;
#endif
    s_snap.overruns = 0;
    return &s_snap;
}

/* ================================================================================================ NAV */
#elif defined(FC_ROLE_NAV)

static uint16_t s_exchanges;
#if !NLINK_HW
static uint8_t s_reply[NL_XFER_BYTES];
static const uint8_t *s_tx_last;                     /* the caller's frame (valid until its next exchange) */
static bool s_tx_new;
#endif

void nlink_spi_init(void)
{
#if NLINK_HW
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        SPCR = 0;
        PORTB |= (uint8_t)(1 << NL_SS);                     /* SS idle high BEFORE it becomes an output */
        DDRB |= (uint8_t)((1 << NL_SS) | (1 << NL_SCK) | (1 << NL_MOSI));
        DDRB &= (uint8_t)~(1 << NL_MISO);
        PORTB |= (uint8_t)(1 << NL_MISO);                   /* pull-up: an absent FCU reads 0xFF (nl_decode_fcu refuses) */
        PORTB &= (uint8_t)~((1 << NL_SCK) | (1 << NL_MOSI));
        SPSR = (uint8_t)((NLINK_SPI_2X) ? (1 << SPI2X) : 0);
        /* master, mode 0 (CPOL 0, CPHA 0), MSB first, no interrupt: SS is an output, so the SPI stays master */
        SPCR = (uint8_t)((1 << SPE) | (1 << MSTR) | ((NLINK_SPI_SPR) & 0x03));
        (void)SPSR;
        (void)SPDR;
    }
#endif
    s_exchanges = 0;
}

#if NLINK_HW
void nlink_spi_exchange(const uint8_t tx[NL_XFER_BYTES], uint8_t rx[NL_XFER_BYTES])
{
    uint8_t i;
    PORTB &= (uint8_t)~(1 << NL_SS);                        /* select: the FCU's byte 0 already sits in its SPDR */
    __builtin_avr_delay_cycles(NLINK_BYTE_GAP_CYCLES);     /* SS -> first SCK: >= one byte gap (the FCU may be
                                                             * (re)loading its byte 0 right at the falling edge) */
    for (i = 0; i < NL_XFER_BYTES; i++) {
        SPDR = tx[i];
        while (!(SPSR & (1 << SPIF))) {
        }
        rx[i] = SPDR;                                       /* reads SPDR after SPSR with SPIF set: clears SPIF */
        __builtin_avr_delay_cycles(NLINK_BYTE_GAP_CYCLES); /* the FCU's interrupt loads its next return byte */
    }
    PORTB |= (uint8_t)(1 << NL_SS);
    s_exchanges++;
}
#else
void nlink_spi_exchange(const uint8_t tx[NL_XFER_BYTES], uint8_t rx[NL_XFER_BYTES])
{
    s_tx_last = tx;
    memcpy(rx, s_reply, NL_XFER_BYTES);
    s_tx_new = true;
    s_exchanges++;
}

bool nlink_spi_take_tx(uint8_t tx[NL_XFER_BYTES])
{
    if (!s_tx_new)
        return false;
    memcpy(tx, s_tx_last, NL_XFER_BYTES);
    s_tx_new = false;
    return true;
}

void nlink_spi_inject_reply(const uint8_t rx[NL_XFER_BYTES])
{
    memcpy(s_reply, rx, NL_XFER_BYTES);
}

/* v0.8: the same without copies (the VTOL NAV bench's RAM): the newest exchange's MOSI bytes in place (NULL: none new),
 * the reply buffer the next exchanges return, to encode in place */
const uint8_t *nlink_spi_take_tx_buf(void)
{
    if (!s_tx_new)
        return 0;
    s_tx_new = false;
    return s_tx_last;
}

uint8_t *nlink_spi_reply_buf(void)
{
    return s_reply;
}
#endif

const nlink_spi_stats_t *nlink_spi_stats(void)
{
    s_snap.frames = s_exchanges;
    s_snap.resyncs = 0;
    s_snap.overruns = 0;
    return &s_snap;
}

/* ================================================================================================ FCU */
#elif defined(FC_ROLE_FCU)

/* Two RX buffers (the ISR fills s_rx[s_wsel], the newest complete one is s_rx[s_ready - 1]) and two reply buffers of
 * 73 bytes: [0..NL_REPLY_BYTES-1] the FCU frame (10 bytes ballistic, 13 VTOL v0.8.1), then zeros (never written after
 * init), [72] = [0], so the per-byte path loads reply byte k + 1 for every k without a compare (k = 71 loads byte 0 of
 * the next exchange). */
static uint8_t s_rx[2][NL_XFER_BYTES];
#if NLINK_HW && FC_VTOL
/* fw09/p1 FCU RAM: 13-byte frame + one zero byte per buffer, 16-byte rows, 16-byte aligned (low nibble 0x0D = the zero
 * byte of either row, 0x0E = just past it): 32 B instead of 146 B (the ISR keeps sending the zero byte) */
#define NL_TX_STRIDE 16u
#define NL_TX_PAST_LO 0x0Eu
typedef char nl_tx_row_fits[(NL_REPLY_BYTES + 1u <= NL_TX_STRIDE && NL_TX_PAST_LO == NL_REPLY_BYTES + 1u) ? 1 : -1];
static uint8_t s_tx[2][NL_TX_STRIDE] __attribute__((aligned(16)));
#elif NLINK_HW
static uint8_t s_tx[2][NL_XFER_BYTES + 1u];
#else
static uint8_t s_tx[2][NL_REPLY_BYTES];            /* bypass: no ISR walks the reply (v0.8: 124 bytes of bench RAM less) */
#endif
static volatile uint8_t s_idx;                     /* bytes of the open exchange */
static uint8_t *volatile s_rxn;                    /* where the next received byte goes */
static const uint8_t *volatile s_txn;              /* the next return byte */
static volatile uint8_t s_wsel;                    /* RX buffer the ISR writes */
static volatile uint8_t s_ready;                   /* 0 none, else 1 + the RX buffer holding the newest exchange */
static volatile uint8_t s_asel;                    /* active reply buffer */
static volatile uint8_t s_pending;                 /* a new reply waits in s_tx[s_asel ^ 1] */
static volatile uint16_t s_frames, s_resyncs, s_overruns;
#if !NLINK_HW
static uint8_t s_last_reply[NL_REPLY_BYTES];       /* v0.7.1: the FCU frame only (the rest of the exchange is padding):
                                                    * 62 bytes less static RAM in the single-chip simbench builds */
#endif

#if NLINK_HW
/* SPI transfer complete (slave), once per byte: 72 per exchange.  Hand-written so the return byte is loaded early and
 * the per-byte cost is small; the C equivalent of the per-byte path is
 *     b = SPDR;                             the byte just received (double-buffered: valid until the next completes)
 *     SPDR = *s_txn++;                      return byte k + 1 (it must be in SPDR before the master's next byte)
 *     *s_rxn++ = b;
 *     if (++s_idx == NL_XFER_BYTES) { ... } the end of the exchange (label 2), below
 * SPDR is read before it is written: the chip keeps the receive buffer and the transmit shift register apart, but
 * simavr's SPI (the FC_COSIM co-simulation) holds one data byte, so a write first would make the ISR store its own
 * return byte as the received one.
 * Per-byte path, counted from the instruction list (ATmega128: push / pop / lds / sts / ld / st 2 cycles, in / out /
 * inc / cpi 1, untaken branch 1, reti 4) plus the 4-cycle interrupt response and the 3-cycle vector JMP:
 *     SPDR written 23 cycles after SPIF (+ up to 4 for a multi-cycle instruction in progress, + whatever interrupt or
 *     cli section is running); the whole interrupt 62 cycles (3.9 us); 72 bytes ~ 4.5k cycles per exchange.
 * End of exchange (label 2, once per 72 bytes): 120 cycles, 125 when a pending reply becomes active, 13 more when an
 * untaken exchange is replaced (overrun): <= 138 cycles.
 * Uses r24, r25, r30, r31 and SREG; r1 (the C zero register) is not assumed zero.  The avr-objdump of the FCU build is
 * the reference for these counts (tools/gen_firmware_params.py NLINK_SLAVE_REPLY_CYCLES uses the 23 + 4). */
ISR(SPI_STC_vect, ISR_NAKED)
{
    __asm__ __volatile__(
        "push r24"                      "\n\t"
        "push r25"                      "\n\t"
        "push r30"                      "\n\t"
        "push r31"                      "\n\t"
        "in   r25, %[spdr]"             "\n\t"   /* the received byte */
        "lds  r30, %[txn]"              "\n\t"
        "lds  r31, %[txn]+1"            "\n\t"
        "ld   r24, Z+"                  "\n\t"   /* return byte k + 1 (no flag changed so far) */
        "out  %[spdr], r24"             "\n\t"
#if FC_VTOL
        "in   r24, __SREG__"            "\n\t"   /* fw09/p1: SREG first, then the stay-at-zero step */
        "push r24"                      "\n\t"
        "mov  r24, r30"                 "\n\t"
        "andi r24, 0x0F"                "\n\t"
        "cpi  r24, %[past]"             "\n\t"
        "brne 8f"                       "\n\t"
        "sbiw r30, 1"                   "\n\t"   /* past the frame: stay on its zero byte */
        "8:"                            "\n\t"
        "sts  %[txn]+1, r31"            "\n\t"
        "sts  %[txn], r30"              "\n\t"
#else
        "sts  %[txn]+1, r31"            "\n\t"
        "sts  %[txn], r30"              "\n\t"
        "in   r24, __SREG__"            "\n\t"
        "push r24"                      "\n\t"
#endif
        "lds  r30, %[rxn]"              "\n\t"
        "lds  r31, %[rxn]+1"            "\n\t"
        "st   Z+, r25"                  "\n\t"
        "sts  %[rxn]+1, r31"            "\n\t"
        "sts  %[rxn], r30"              "\n\t"
        "lds  r24, %[idx]"              "\n\t"
        "inc  r24"                      "\n\t"
        "cpi  r24, %[nx]"               "\n\t"
        "breq 2f"                       "\n\t"
        "sts  %[idx], r24"              "\n\t"
        "1:"                            "\n\t"
        "pop  r24"                      "\n\t"
        "out  __SREG__, r24"            "\n\t"
        "pop  r31"                      "\n\t"
        "pop  r30"                      "\n\t"
        "pop  r25"                      "\n\t"
        "pop  r24"                      "\n\t"
        "reti"                          "\n\t"
        /* ---- end of the exchange ---- */
        "2:"                            "\n\t"
        "ldi  r25, 0"                   "\n\t"
        "sts  %[idx], r25"              "\n\t"   /* idx = 0 */
        "lds  r24, %[ready]"            "\n\t"
        "cpse r24, r25"                 "\n\t"   /* an untaken exchange is replaced: overruns++ */
        "rjmp 3f"                       "\n\t"
        "4:"                            "\n\t"
        "lds  r30, %[frames]"           "\n\t"
        "lds  r31, %[frames]+1"         "\n\t"
        "adiw r30, 1"                   "\n\t"
        "sts  %[frames]+1, r31"         "\n\t"
        "sts  %[frames], r30"           "\n\t"
        "lds  r24, %[wsel]"             "\n\t"
        "mov  r25, r24"                 "\n\t"
        "inc  r25"                      "\n\t"
        "sts  %[ready], r25"            "\n\t"   /* ready = 1 + the buffer just filled */
        "ldi  r25, 1"                   "\n\t"
        "eor  r24, r25"                 "\n\t"
        "sts  %[wsel], r24"             "\n\t"   /* the other buffer takes the next exchange */
        "ldi  r30, lo8(%[rx0])"         "\n\t"
        "ldi  r31, hi8(%[rx0])"         "\n\t"
        "cpse r24, r25"                 "\n\t"
        "rjmp 5f"                       "\n\t"
        "subi r30, lo8(-(%[nx]))"       "\n\t"   /* wsel 1: + 72 */
        "sbci r31, hi8(-(%[nx]))"       "\n\t"
        "5:"                            "\n\t"
        "sts  %[rxn]+1, r31"            "\n\t"
        "sts  %[rxn], r30"              "\n\t"
        "lds  r24, %[asel]"             "\n\t"
        "lds  r30, %[pend]"             "\n\t"
        "tst  r30"                      "\n\t"
        "breq 6f"                       "\n\t"
        "eor  r24, r25"                 "\n\t"   /* a pending reply becomes active */
        "sts  %[asel], r24"             "\n\t"
        "ldi  r30, 0"                   "\n\t"
        "sts  %[pend], r30"             "\n\t"
        "6:"                            "\n\t"
        "ldi  r30, lo8(%[tx0])"         "\n\t"
        "ldi  r31, hi8(%[tx0])"         "\n\t"
        "cpse r24, r25"                 "\n\t"
        "rjmp 7f"                       "\n\t"
        "subi r30, lo8(-(%[nt]))"       "\n\t"   /* asel 1: + 73 */
        "sbci r31, hi8(-(%[nt]))"       "\n\t"
        "7:"                            "\n\t"
        "ld   r24, Z+"                  "\n\t"
        "out  %[spdr], r24"             "\n\t"   /* byte 0 of the next exchange (no transfer in progress) */
        "sts  %[txn]+1, r31"            "\n\t"
        "sts  %[txn], r30"              "\n\t"
        "rjmp 1b"                       "\n\t"
        "3:"                            "\n\t"
        "lds  r30, %[ovr]"              "\n\t"
        "lds  r31, %[ovr]+1"            "\n\t"
        "adiw r30, 1"                   "\n\t"
        "sts  %[ovr]+1, r31"            "\n\t"
        "sts  %[ovr], r30"              "\n\t"
        "rjmp 4b"                       "\n\t"
        :
        : [spdr] "I"(_SFR_IO_ADDR(SPDR)), [txn] "i"(&s_txn), [rxn] "i"(&s_rxn), [idx] "i"(&s_idx),
#if FC_VTOL
          [nx] "M"(NL_XFER_BYTES), [nt] "M"(NL_TX_STRIDE), [past] "M"(NL_TX_PAST_LO), [ready] "i"(&s_ready),
          [frames] "i"(&s_frames),
#else
          [nx] "M"(NL_XFER_BYTES), [nt] "M"(NL_XFER_BYTES + 1u), [ready] "i"(&s_ready), [frames] "i"(&s_frames),
#endif
          [ovr] "i"(&s_overruns), [wsel] "i"(&s_wsel), [rx0] "i"(&s_rx[0][0]), [asel] "i"(&s_asel), [pend] "i"(&s_pending),
          [tx0] "i"(&s_tx[0][0])
        : "memory");
}

/* SS high and no byte waiting for its interrupt (call with interrupts off) */
static inline bool ss_idle(void)
{
    return (PINB & (uint8_t)(1 << NL_SS)) != 0u && (SPSR & (uint8_t)(1 << SPIF)) == 0u;
}
#else
static inline bool ss_idle(void)
{
    return true;                                      /* simbench: exchanges are injected whole */
}
#endif

/* make reply buffer sel the active one and load its byte 0 (interrupts off, no exchange in progress) */
static void activate(uint8_t sel)
{
    s_asel = sel;
    s_txn = &s_tx[sel][1];
#if NLINK_HW
    SPDR = s_tx[sel][0];                              /* SS high: loaded for the next exchange (no write collision) */
#endif
}

/* rule 2 of sim/nlink_proto.py SpiSlaveModel (interrupts off) */
static void poll(void)
{
    if (!ss_idle())
        return;
    if (s_idx != 0u) {                                /* a partial exchange: discard it */
        s_idx = 0;
        s_rxn = s_rx[s_wsel];
        s_resyncs++;
    }
    if (s_pending) {
        s_pending = 0;
        activate((uint8_t)(s_asel ^ 1u));
    } else {
        activate(s_asel);                             /* (re)load byte 0: the SS rise may have reset the shifter */
    }
}

void nlink_spi_init(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        memset(s_rx, 0, sizeof s_rx);
        memset(s_tx, 0, sizeof s_tx);                 /* no reply yet: zeros (the NAV's nl_decode_fcu refuses them) */
        s_idx = 0;
        s_wsel = 0;
        s_rxn = s_rx[0];
        s_ready = 0;
        s_pending = 0;
        s_frames = s_resyncs = s_overruns = 0;
#if NLINK_HW
        SPCR = 0;
        DDRB &= (uint8_t)~((1 << NL_SS) | (1 << NL_SCK) | (1 << NL_MOSI));
        PORTB |= (uint8_t)((1 << NL_SS) | (1 << NL_SCK) | (1 << NL_MOSI));   /* pull-ups: an absent NAV = SS high */
        DDRB |= (uint8_t)(1 << NL_MISO);
        SPSR = 0;
        SPCR = (uint8_t)((1 << SPIE) | (1 << SPE));    /* slave, mode 0, MSB first, interrupt per byte */
        (void)SPSR;
        (void)SPDR;
#endif
        activate(0);
    }
}

void nlink_spi_set_reply(const uint8_t tx[NL_FCU_FRAME_BYTES])
{
    uint8_t *b;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        s_pending = 0;                                /* the inactive buffer is ours: the ISR cannot swap it in */
        b = s_tx[s_asel ^ 1u];
    }
    memcpy(b, tx, NL_REPLY_BYTES);
#if NLINK_HW && !FC_VTOL
    b[NL_XFER_BYTES] = tx[0];
#endif
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (s_idx == 0u && ss_idle())
            activate((uint8_t)(s_asel ^ 1u));          /* between exchanges: the next one returns it */
        else
            s_pending = 1;                            /* an exchange is open: from its end on */
    }
}

bool nlink_spi_frame_ready(void)
{
    bool r;
#if NLINK_HW
    if (!(PINB & (uint8_t)(1 << NL_SS)))
        return s_ready != 0u;                         /* SS low: an exchange is open, rule 2 has nothing to do - and the
                                                       * main loop spinning here keeps its cli sections out of it */
#endif
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        poll();
        r = s_ready != 0u;
    }
    return r;
}

bool nlink_spi_take(uint8_t rx[NL_XFER_BYTES])
{
    uint8_t r;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        poll();
        r = s_ready;
        s_ready = 0;
    }
    if (r == 0u)
        return false;
    memcpy(rx, s_rx[r - 1u], NL_XFER_BYTES);          /* the ISR fills the other buffer until one more exchange ended */
    return true;
}

const nlink_spi_stats_t *nlink_spi_stats(void)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        s_snap.frames = s_frames;
        s_snap.resyncs = s_resyncs;
        s_snap.overruns = s_overruns;
    }
    return &s_snap;
}

#if !NLINK_HW
/* the end-of-exchange step of the ISR, in C (simbench only) */
uint8_t *nlink_spi_inject_buf(void)
{
    return s_rx[s_wsel];
}

void nlink_spi_inject_frame(const uint8_t rx[NL_XFER_BYTES])
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        memcpy(s_last_reply, s_tx[s_asel], NL_REPLY_BYTES);
        if (rx != s_rx[s_wsel])
            memcpy(s_rx[s_wsel], rx, NL_XFER_BYTES);
        if (s_ready != 0u)
            s_overruns++;
        s_ready = (uint8_t)(s_wsel + 1u);
        s_wsel ^= 1u;
        s_rxn = s_rx[s_wsel];
        s_idx = 0;
        s_frames++;
        if (s_pending) {
            s_pending = 0;
            activate((uint8_t)(s_asel ^ 1u));
        }
    }
}

void nlink_spi_last_reply(uint8_t tx[NL_XFER_BYTES])
{
    memcpy(tx, s_last_reply, NL_REPLY_BYTES);               /* what nl_decode_fcu(4) reads; the rest of tx is left as is */
}
#endif

#endif /* FC_ROLE_FCU */
