/* twi.c - see twi.h */
#include "twi.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>
#include <util/delay.h>

#include "../params_gen.h"

#define TWI_WAIT_LOOPS 2000u          /* ~1 ms at 16 MHz; one byte at 400 kHz takes 22.5 us */
#define TW_START 0x08
#define TW_REP_START 0x10
#define TW_MT_SLA_ACK 0x18
#define TW_MT_DATA_ACK 0x28
#define TW_MR_SLA_ACK 0x40
#define TW_MR_DATA_ACK 0x50
#define TW_MR_DATA_NACK 0x58


static uint16_t s_errors;

/* ---- interrupt-driven transfer scripts (one script at a time) -------------
 * A script is a list of operations, each START (or repeated START) + SLA + data;
 * an op flagged TWI_OP_RESTART is followed by a repeated START, otherwise by
 * STOP (+ START for the next op).  The TWI ISR walks the list; the caller polls
 * twi_async_busy() / twi_async_wait().  The op list and its buffers must stay
 * valid until the script ends. */
#define A_IDLE TWI_ASYNC_IDLE
#define A_BUSY TWI_ASYNC_BUSY
#define A_DONE TWI_ASYNC_DONE
#define A_ERROR TWI_ASYNC_ERROR
static volatile uint8_t a_state = A_IDLE, a_op, a_nops, a_idx;
static volatile uint8_t *volatile a_owner;        /* the script's own status byte */
static const twi_op_t *volatile a_ops;
static volatile bool s_blocking;                  /* a blocking transfer owns the bus */

static inline __attribute__((always_inline)) void a_finish(uint8_t state)
{
    TWCR = (uint8_t)((1 << TWINT) | (1 << TWEN) | (1 << TWSTO));     /* STOP, interrupt off */
    a_state = state;
    if (a_owner)
        *a_owner = state;
}

/* current op done: next op (repeated START or STOP+START) or the end of the script */
static inline __attribute__((always_inline)) void a_next_op(void)
{
    const uint8_t go = (uint8_t)((1 << TWINT) | (1 << TWEN) | (1 << TWIE));
    const bool restart = (a_ops[a_op].flags & TWI_OP_RESTART) != 0;
    a_op++;
    a_idx = 0;
    if (a_op >= a_nops) {
        a_finish(A_DONE);
    } else if (restart) {
        TWCR = (uint8_t)(go | (1 << TWSTA));
    } else {
        TWCR = (uint8_t)(go | (1 << TWSTA) | (1 << TWSTO));          /* STOP then START */
    }
}

ISR(TWI_vect)
{
    const uint8_t st = (uint8_t)(TWSR & 0xF8);
    const uint8_t go = (uint8_t)((1 << TWINT) | (1 << TWEN) | (1 << TWIE));
    const twi_op_t *op = &a_ops[a_op];
    switch (st) {
    case TW_START:
    case TW_REP_START:
        TWDR = (uint8_t)((op->addr7 << 1) | ((op->flags & TWI_OP_READ) ? 1 : 0));
        TWCR = go;
        break;
    case TW_MT_SLA_ACK:
    case TW_MT_DATA_ACK:
        if (a_idx < op->len) {
            TWDR = op->data[a_idx++];
            TWCR = go;
        } else {
            a_next_op();
        }
        break;
    case TW_MR_SLA_ACK:
        TWCR = (uint8_t)(op->len > 1 ? (go | (1 << TWEA)) : go);
        break;
    case TW_MR_DATA_ACK:
        op->data[a_idx++] = TWDR;
        TWCR = (uint8_t)((uint8_t)(a_idx + 1) < op->len ? (go | (1 << TWEA)) : go);
        break;
    case TW_MR_DATA_NACK:                         /* last byte of a read */
        op->data[a_idx++] = TWDR;
        a_next_op();
        break;
    default:                                      /* NACK, arbitration lost, bus error */
        s_errors++;
        a_finish(A_ERROR);
        break;
    }
}

bool twi_async_start(const twi_op_t *ops, uint8_t n_ops, volatile uint8_t *status)
{
    bool ok = false;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (!s_blocking && a_state != A_BUSY && !(TWCR & (1 << TWSTO)) && n_ops > 0) {
            a_owner = status;
            if (status)
                *status = A_BUSY;
            a_ops = ops;
            a_nops = n_ops;
            a_op = 0;
            a_idx = 0;
            a_state = A_BUSY;
            TWCR = (uint8_t)((1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (1 << TWIE));
            ok = true;
        }
    }
    return ok;
}

bool twi_async_busy(void)
{
    return a_state == A_BUSY;
}

bool twi_async_ok(void)
{
    __asm__ __volatile__("" ::: "memory");      /* buffers written by the ISR are read after this */
    return a_state == A_DONE;
}

bool twi_async_wait(void)
{
    uint16_t n = 2u * TWI_WAIT_LOOPS;
    while (a_state == A_BUSY) {
        if (--n == 0) {                           /* hung transfer: abort it */
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                s_errors++;
                a_finish(A_ERROR);
            }
            break;
        }
    }
    __asm__ __volatile__("" ::: "memory");      /* buffers written by the ISR are read after this */
    return a_state == A_DONE;
}

/* blocking transfers wait for the async one and lock it out while they run */
static void claim(void)
{
    uint16_t n = 2u * TWI_WAIT_LOOPS;
    s_blocking = true;                            /* first: no new async read can start now */
    while (a_state == A_BUSY && --n) {            /* let a running one finish */
    }
    n = TWI_WAIT_LOOPS;
    while ((TWCR & (1 << TWSTO)) && --n) {
    }
    TWCR = (uint8_t)(1 << TWEN);                  /* TWIE off: blocking code polls TWINT */
}

static void release(void)
{
    s_blocking = false;
}

static bool wait_twint(void)
{
    uint16_t n = TWI_WAIT_LOOPS;
    while (!(TWCR & (1 << TWINT))) {
        if (--n == 0)
            return false;
    }
    return true;
}

static uint8_t status(void) { return (uint8_t)(TWSR & 0xF8); }

static void stop(void)
{
    uint16_t n = TWI_WAIT_LOOPS;
    TWCR = (uint8_t)((1 << TWINT) | (1 << TWEN) | (1 << TWSTO));
    while ((TWCR & (1 << TWSTO)) && --n) {
    }
}

static bool fail(void)
{
    s_errors++;
    stop();
    release();
    return false;
}

static bool start(uint8_t sla_rw)
{
    TWCR = (uint8_t)((1 << TWINT) | (1 << TWSTA) | (1 << TWEN));
    if (!wait_twint())
        return false;
    if (status() != TW_START && status() != TW_REP_START)
        return false;
    TWDR = sla_rw;
    TWCR = (uint8_t)((1 << TWINT) | (1 << TWEN));
    if (!wait_twint())
        return false;
    return status() == ((sla_rw & 1) ? TW_MR_SLA_ACK : TW_MT_SLA_ACK);
}

static bool put(uint8_t b)
{
    TWDR = b;
    TWCR = (uint8_t)((1 << TWINT) | (1 << TWEN));
    return wait_twint() && status() == TW_MT_DATA_ACK;
}

static bool get(uint8_t *buf, uint8_t len)
{
    uint8_t i;
    for (i = 0; i < len; i++) {
        bool last = (uint8_t)(i + 1) == len;
        TWCR = (uint8_t)((1 << TWINT) | (1 << TWEN) | (last ? 0 : (1 << TWEA)));
        if (!wait_twint() || status() != (last ? TW_MR_DATA_NACK : TW_MR_DATA_ACK))
            return false;
        buf[i] = TWDR;
    }
    return true;
}

void twi_init(void)
{
    PORTD &= (uint8_t)~((1 << PD0) | (1 << PD1));    /* external pull-ups on the level shifter */
    TWSR = 0;                                         /* prescaler 1 */
    TWBR = FC_TWBR;
    TWCR = (1 << TWEN);
}

bool twi_write(uint8_t addr7, const uint8_t *data, uint8_t len)
{
    uint8_t i;
    claim();
    if (!start((uint8_t)(addr7 << 1)))
        return fail();
    for (i = 0; i < len; i++)
        if (!put(data[i]))
            return fail();
    stop();
    release();
    return true;
}

bool twi_write_reg(uint8_t addr7, uint8_t reg, uint8_t value)
{
    uint8_t b[2] = {reg, value};
    return twi_write(addr7, b, 2);
}

bool twi_read_reg(uint8_t addr7, uint8_t reg, uint8_t *buf, uint8_t len)
{
    claim();
    if (!start((uint8_t)(addr7 << 1)) || !put(reg))
        return fail();
    if (!start((uint8_t)((addr7 << 1) | 1)) || !get(buf, len))
        return fail();
    stop();
    release();
    return true;
}

bool twi_read(uint8_t addr7, uint8_t *buf, uint8_t len)
{
    claim();
    if (!start((uint8_t)((addr7 << 1) | 1)) || !get(buf, len))
        return fail();
    stop();
    release();
    return true;
}

void twi_recover(void)
{
    uint8_t i;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        a_state = A_IDLE;
        s_blocking = true;
    }
    TWCR = 0;                                         /* release the pins to the port */
    DDRD &= (uint8_t)~(1 << PD1);                     /* SDA input (external pull-up) */
    for (i = 0; i < 9; i++) {                         /* nine SCL pulses free a slave mid-byte */
        DDRD |= (uint8_t)(1 << PD0);                  /* SCL low (PORTD0 = 0) */
        _delay_us(5);
        DDRD &= (uint8_t)~(1 << PD0);                 /* SCL released high */
        _delay_us(5);
    }
    twi_init();
    stop();
    s_blocking = false;
}

uint16_t twi_error_count(void)
{
    return s_errors;
}
