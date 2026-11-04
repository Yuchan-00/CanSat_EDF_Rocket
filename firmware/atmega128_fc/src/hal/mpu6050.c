/* mpu6050.c - see mpu6050.h.  Register addresses: MPU-6000/6050 Register Map rev 4.2. */
#include "mpu6050.h"

#include <avr/wdt.h>
#include <util/delay.h>

#include "../params_gen.h"
#include "twi.h"

#define REG_SMPLRT_DIV 0x19
#define REG_CONFIG 0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_INT_PIN_CFG 0x37
#define REG_INT_ENABLE 0x38
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1 0x6B
#define REG_WHO_AM_I 0x75

#if IMU_DLPF_CFG < 1 || IMU_DLPF_CFG > 6
#error "DLPF_CFG must be 1..6 (1 kHz gyro rate)"
#endif

static void delay_ms_wdt(uint16_t ms)
{
    while (ms--) {
        _delay_ms(1);
        wdt_reset();
    }
}

bool mpu6050_whoami_ok(uint8_t who)
{
    /* 0x68: MPU-6050; 0x72 / 0x98: register-compatible MPU-6050 clones found on GY-521 boards */
    return who == 0x68 || who == 0x72 || who == 0x98;
}

uint8_t mpu6050_init(void)
{
    uint8_t who = 0;
    if (!twi_write_reg(CAL_MPU6050_ADDR, REG_PWR_MGMT_1, 0x80))       /* device reset */
        return 0;
    delay_ms_wdt(100);
    if (!twi_write_reg(CAL_MPU6050_ADDR, REG_PWR_MGMT_1, 0x01))       /* wake, PLL with X gyro */
        return 0;
    delay_ms_wdt(10);
    if (!twi_read_reg(CAL_MPU6050_ADDR, REG_WHO_AM_I, &who, 1))
        return 0;
    if (!twi_write_reg(CAL_MPU6050_ADDR, REG_SMPLRT_DIV, FC_MPU_SMPLRT_DIV) ||
        !twi_write_reg(CAL_MPU6050_ADDR, REG_CONFIG, IMU_DLPF_CFG) ||
        !twi_write_reg(CAL_MPU6050_ADDR, REG_GYRO_CONFIG, (uint8_t)(IMU_GYRO_FS_SEL << 3)) ||
        !twi_write_reg(CAL_MPU6050_ADDR, REG_ACCEL_CONFIG, (uint8_t)(IMU_ACCEL_AFS_SEL << 3)) ||
        !twi_write_reg(CAL_MPU6050_ADDR, REG_INT_PIN_CFG, 0x10) ||      /* INT clears on any read */
        !twi_write_reg(CAL_MPU6050_ADDR, REG_INT_ENABLE, 0x01))         /* DATA_RDY on INT (optional pin) */
        return 0;
    delay_ms_wdt(30);                                                   /* gyro start-up */
    return who;
}

static int16_t be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static uint8_t s_async_reg = REG_ACCEL_XOUT_H;
static uint8_t s_async_buf[MPU6050_BURST];
static volatile bool s_async_started;
static const twi_op_t k_burst[2] = {
    {CAL_MPU6050_ADDR, TWI_OP_RESTART, 1, &s_async_reg},
    {CAL_MPU6050_ADDR, TWI_OP_READ, MPU6050_BURST, s_async_buf},
};

void mpu6050_start_async(void)
{
    s_async_started = twi_async_start(k_burst, 2, 0);
}

bool mpu6050_finish_async(imu_sample_t *s)
{
    uint8_t b[MPU6050_BURST], i;
    if (!s_async_started || !twi_async_wait())
        return false;
    s_async_started = false;
    for (i = 0; i < MPU6050_BURST; i++)
        b[i] = s_async_buf[i];
    mpu6050_convert(b, s);
    return true;
}

bool mpu6050_read(imu_sample_t *s)
{
    uint8_t b[MPU6050_BURST];
    if (!twi_read_reg(CAL_MPU6050_ADDR, REG_ACCEL_XOUT_H, b, MPU6050_BURST))
        return false;
    mpu6050_convert(b, s);
    return true;
}

void mpu6050_convert(const uint8_t b[MPU6050_BURST], imu_sample_t *s)
{
    uint8_t i;
    static const uint8_t idx[3] = {CAL_IMU_BODY_X_INDEX, CAL_IMU_BODY_Y_INDEX, CAL_IMU_BODY_Z_INDEX};
    static const float kg[3] = {CAL_IMU_BODY_X_SIGN * IMU_GYRO_RAD_S_PER_LSB, CAL_IMU_BODY_Y_SIGN * IMU_GYRO_RAD_S_PER_LSB,
                                CAL_IMU_BODY_Z_SIGN * IMU_GYRO_RAD_S_PER_LSB};
    static const float ka[3] = {CAL_IMU_BODY_X_SIGN * IMU_ACCEL_MPS2_PER_LSB, CAL_IMU_BODY_Y_SIGN * IMU_ACCEL_MPS2_PER_LSB,
                                CAL_IMU_BODY_Z_SIGN * IMU_ACCEL_MPS2_PER_LSB};
    for (i = 0; i < 3; i++) {
        s->raw_accel[i] = be16(&b[2 * i]);
        s->raw_gyro[i] = be16(&b[8 + 2 * i]);
    }
    s->raw_temp = be16(&b[6]);
    s->gyro_saturated = false;
    for (i = 0; i < 3; i++) {
        int16_t g = s->raw_gyro[idx[i]], a = s->raw_accel[idx[i]];
        if (g >= 32767 || g <= -32767)               /* |g| >= full scale - 1/2 LSB, as sim/estimator.py */
            s->gyro_saturated = true;
        s->gyro[i] = (float)g * kg[i];             /* sign folded into the scale: one conversion, one multiply */
        s->accel[i] = (float)a * ka[i];
    }
}
