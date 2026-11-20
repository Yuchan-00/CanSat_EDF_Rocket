/* twi.h - I2C master (FC_I2C_HZ, TWBR from params_gen.h): blocking transfers with
 * bounded waits (boot), plus interrupt-driven transfer scripts (twi_async_start): the
 * Timer0 lead ISR starts the MPU-6050 burst read so the sample is ready at the control
 * tick, and the MS5611 command/read sequences run while the CPU computes.
 *
 * Invariant: no function can hang - every wait for TWINT is bounded
 * (TWI_WAIT_LOOPS, about 1 ms) and an error always ends with a STOP; after
 * repeated errors the caller runs twi_recover(), which clocks SCL nine
 * times to release a slave that holds SDA low (MPU-6050 after a brown-out).
 */
#ifndef FC_TWI_H
#define FC_TWI_H

#include <stdbool.h>
#include <stdint.h>

void twi_init(void);
bool twi_write(uint8_t addr7, const uint8_t *data, uint8_t len);
bool twi_write_reg(uint8_t addr7, uint8_t reg, uint8_t value);
bool twi_read_reg(uint8_t addr7, uint8_t reg, uint8_t *buf, uint8_t len);   /* write reg, repeated START, read */
bool twi_read(uint8_t addr7, uint8_t *buf, uint8_t len);
void twi_recover(void);
/* interrupt-driven scripts: each op = START/repeated START, SLA+R/W, len data bytes */
#define TWI_OP_READ 0x01
#define TWI_OP_RESTART 0x02          /* followed by a repeated START instead of STOP */
typedef struct {
    uint8_t addr7, flags, len;
    uint8_t *data;
} twi_op_t;
#define TWI_ASYNC_IDLE 0
#define TWI_ASYNC_BUSY 1
#define TWI_ASYNC_DONE 2
#define TWI_ASYNC_ERROR 3
/* false if the bus is in use; *status (optional) follows the script: BUSY, then DONE or ERROR */
bool twi_async_start(const twi_op_t *ops, uint8_t n_ops, volatile uint8_t *status);
bool twi_async_busy(void);
bool twi_async_ok(void);          /* the last script completed without error */
bool twi_async_wait(void);        /* bounded wait for the running script; true if it completed */
uint16_t twi_error_count(void);

#endif /* FC_TWI_H */
