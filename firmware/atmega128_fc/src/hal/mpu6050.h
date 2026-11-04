/* mpu6050.h - MPU-6050 (GY-521) driver: configuration, burst read, body-frame SI.
 *
 * Configuration (all from params_gen.h): SMPLRT_DIV = FC_MPU_SMPLRT_DIV
 * (1 kHz / (1 + div) = control.imu_hz), DLPF_CFG = IMU_DLPF_CFG (gyro 98 Hz,
 * 2.8 ms), FS_SEL = IMU_GYRO_FS_SEL (+-2000 deg/s), AFS_SEL = IMU_ACCEL_AFS_SEL
 * (+-16 g); clock = PLL on the X gyro.  One 14-byte burst read per tick gets the
 * newest 1 kHz sample - started by the Timer0 lead ISR FC_IMU_LEAD_COUNTS before the
 * control tick and finished in the main loop (no blocking bus time in the loop);
 * the body frame is the chip frame rotated by the signed axis permutation
 * CAL_IMU_BODY_* (fc_config.json imu_mounting).
 *
 * Invariant: the driver never returns stale data as fresh - a failed read
 * returns false and leaves the previous sample untouched, the caller counts
 * failures.  Gyro bias is not removed here: that is the estimator's pad
 * alignment (sim/estimator.py PadAlignGyroEstimator).
 */
#ifndef FC_MPU6050_H
#define FC_MPU6050_H

#include <stdbool.h>
#include <stdint.h>

#define MPU6050_BURST 14                            /* ACCEL_XOUT_H .. GYRO_ZOUT_L */

typedef struct {
    int16_t raw_accel[3], raw_temp, raw_gyro[3];   /* chip frame counts */
    float gyro[3];                                  /* body frame, rad/s */
    float accel[3];                                 /* body frame specific force, m/s^2 */
    bool  gyro_saturated;                           /* any gyro axis at the int16 rail */
} imu_sample_t;

uint8_t mpu6050_init(void);                         /* returns WHO_AM_I (0 on I2C failure) */
bool    mpu6050_whoami_ok(uint8_t who);
bool    mpu6050_read(imu_sample_t *s);                 /* blocking burst read (boot) */
void    mpu6050_start_async(void);                     /* Timer0 lead ISR: start the burst read */
bool    mpu6050_finish_async(imu_sample_t *s);         /* main loop: wait (bounded) and convert */
void    mpu6050_convert(const uint8_t b[MPU6050_BURST], imu_sample_t *s);

#endif /* FC_MPU6050_H */
