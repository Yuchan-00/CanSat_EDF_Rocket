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
#if NL_XFER_BYTES != 72 || NL_FCU_FRAME_BYTES != 10
#error "nlink_spi.c's buffer offsets (73-byte reply buffers, the end-of-exchange compare) assume 72 / 10 bytes"
#endif
/* the FCU frame the slave returns: the v0.6.3 frame (10 bytes; the 73-byte buffers were sized for the removed VTOL
 * program's 13-byte frame and stay so the flight images are unchanged) */
#define NL_REPLY_BYTES NL_FCU_FRAME_BYTES

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

/* ================================================================================================ NAV */
#if defined(FC_ROLE_NAV)

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
 * 73 bytes: [0..NL_REPLY_BYTES-1] the FCU frame (10 bytes), then zeros (never written after
 * init), [72] = [0], so the per-byte path loads reply byte k + 1 for every k without a compare (k = 71 loads byte 0 of
 * the next exchange). */
static uint8_t s_rx[2][NL_XFER_BYTES];
#if NLINK_HW
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
        "sts  %[txn]+1, r31"            "\n\t"
        "sts  %[txn], r30"              "\n\t"
        "in   r24, __SREG__"            "\n\t"
        "push r24"                      "\n\t"
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
          [nx] "M"(NL_XFER_BYTES), [nt] "M"(NL_XFER_BYTES + 1u), [ready] "i"(&s_ready), [frames] "i"(&s_frames),
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
#if NLINK_HW
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
