/* nav.h - the navigation computer's (NAV, v0.6.3) own definitions: what it reports in the navigation frame
 * beyond core/nlink_proto.h's fields, its LED and its Timer1 time base.
 *
 * The NAV has no log port (params.json flight_computer.architecture: its only link is the nlink to the FCU),
 * so everything it knows about itself travels in the navigation frame every tick and the FCU logs it:
 *   - flags (nlink_proto.h NL_F_*): IMU_OK / BARO_OK / ALIGNED / GYRO_SAT / EST_VALID / BARO_NEW / BOOT, as
 *     the contract defines them; bit 7 NAV_F_EST_GATE (below, v0.7.1 VTOL build only);
 *   - imu_fail (consecutive failed MPU-6050 reads; climbs from boot when the MPU-6050 never answered),
 *     baro_fails (MS5611 cycles that published nothing), twi_errors (the TWI driver's error count);
 *   - reset_cause: bits 4..0 = the NAV's MCUCSR at boot (RESET_* of hal/watchdog.h, as the contract says),
 *     bits 7..5 = the NAV_RC_* bits below.  MCUCSR has only five reset flags (watchdog.c keeps & 0x1F), so the
 *     two fields never overlap.  Bits 7 / 6 are the boot results that v0.5 wrote as text records ("MPU-6050
 *     FAIL, WHO_AM_I 0x..", "MS5611 FAIL (I2C or PROM CRC)"): without them a missing MS5611 would only show as
 *     a BARO_OK that never comes (its state machine never runs, so baro_fails stays 0).
 * On the board the NAV's status LED (PC1, board.h NAV_LED_*) shows the same, in this priority:
 *   IMU absent or failing (>= ONB_IMU_FAIL_TICKS failed reads)   fast blink, 32 ticks on / 32 off (~4 Hz)
 *   baro absent or not on schedule (BARO_OK clear)               slow blink, 128 / 128 ticks (~1 Hz)
 *   FCU params tag differs from this build's FC_PARAMS_TAG       double flash every 256 ticks (~1 s)
 *   no valid FCU frame for NAV_FCU_LOST_TICKS (or never)         one short flash every 256 ticks
 *   pad mode, not aligned yet                                    on, with a short dark blip every 256 ticks
 *   aligned on the pad, or flying                                steady on
 * and, VTOL build only (fw09 W1, nav_main.c nav_halt_timer3): Timer3 not as the nlink set it after the drivers' init (a
 * build that links another Timer3 driver): very fast blink, 8 ticks on / 8 off (~16 Hz), and no nlink exchange at all.
 *
 * Invariant: the NAV makes no flight decision - these bits and the LED only report.
 */
#ifndef FC_NAV_H
#define FC_NAV_H

#include <stdbool.h>
#include <stdint.h>

#define NAV_RC_MCUCSR_MASK 0x1Fu     /* reset_cause bits 4..0: MCUCSR at boot (PORF, EXTRF, BORF, WDRF, JTRF) */
#define NAV_RC_TAG_MISMATCH 0x20u    /* the last valid FCU frame carried a params_tag != this build's FC_PARAMS_TAG */
#define NAV_RC_BARO_INIT_FAIL 0x40u  /* MS5611 reset / PROM read / PROM CRC / first D2 failed at boot */
#define NAV_RC_IMU_INIT_FAIL 0x80u   /* MPU-6050 WHO_AM_I not recognised after 3 tries (with twi_recover between) */

/* nl_nav_t.flags bit 7 (v0.7.1 - v0.9: the VTOL NAV's estimator-correction flag; core/nlink_proto.h's NL_F_* end at 0x40):
 * the rocket's NAV never sets it - the NAV bench checks that (nav_simbench.c).  The VTOL NAV's push floor
 * (NAV_PUSH_MIN_US_VTOL) went with the VTOL builds on 2026-11-18; the NAV pushes its frame as soon as it is ready. */
#define NAV_F_EST_GATE 0x80u

#define NAV_FCU_LOST_TICKS 250u      /* LED: no valid FCU frame for 1 s (information only, changes nothing) */

/* Timer1 as a free-running 0.5 us time base (FC_PWM_COUNTS_PER_US counts per us, wrapping after FC_PWM_TOP like
 * the FCU's PWM timer), so hal/timer.c's timer_tcnt1 / timer_elapsed_counts work on the NAV: mode 14 with
 * TOP = ICR1 as on the FCU, no output-compare pin connected, no interrupt (nav_timer.c) */
void nav_timebase_init(void);

#endif /* FC_NAV_H */
