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

/* nl_nav_t.flags bit 7 (v0.7.1; core/nlink_proto.h's NL_F_* end at 0x40, bit 7 was always 0): the VTOL NAV's gated
 * Mahony estimator (core/att_estimator.h att_mahony_t.gate) applied its accelerometer correction this tick
 * (| |a_lpf| - g | < accel_gate g).  Report only, like the NAV_RC_* bits: the FCU counts it into the log's 'V' record
 * (format 6, est_gate); no flight decision reads it.  The ballistic NAV never sets it (its frames stay v0.6.3's). */
#define NAV_F_EST_GATE 0x80u

/* VTOL build (v0.7.1; v0.8 below): the navigation frame never leaves earlier than NAV_PUSH_MIN_US_VTOL after the loop's tick start
 * (nav_main.c push_wait; later when this tick's compute takes longer - then at once).  The gated Mahony estimator costs
 * ~9.6 k cycles on the pad, ~12.4 k with its gate closed and ~20.0 k with it open, the VTOL altitude filter ~1.5 k
 * without and ~4.4 k with a baro sample + compensation (core agent's AVR micro-benchmarks): pushed as soon as ready,
 * the frame time jumped by up to 749 us from one tick to the next (pad -> flight switch, measured incl. the MS5611 bus
 * interrupts by tools/run_nav_simbench.py), more than the FCU's frame-timeout margin (NLINK_FRAME_TIMEOUT_US - tick =
 * 600 us): the FCU would have run ticks on its own timer without a frame.  With the floor the frame time stays within
 * [NAV_PUSH_MIN_US_VTOL, the worst compute + the bus interrupts] + the exchange, so the tick-to-tick variation is
 * bounded by that width.  The FCU's background timing (main.c NLINK_EARLY_US 350 us) allows the NAV a variation of
 * 250 us (simbench.h SB_NAV_JITTER_US) + its own tick-start lateness: the floor is the measured worst compute end +
 * bus interrupts (2406 us) - 250 us, rounded up (tools/run_nav_simbench.py checks the width against SB_NAV_JITTER_US).
 * Why a floor and not a fixed push at the worst case (tried: 2450 us): the background (MS5611 math, one powf step
 * ~0.37 ms) must end before the MPU-6050 lead (FC_IMU_LEAD_COUNTS before the tick); after a fixed push at the worst case
 * it did not (3725 us > 3504 us), while a frame pushed as soon as the worst-case compute ends leaves the same room as
 * without the floor.  The worst case - the delay the 6-DOF models (the measured maximum) - is unchanged.  The
 * ballistic NAV pushes as soon as ready (v0.6.3, unchanged). */
/* v0.8 (nav_main.c: predict -> publish -> correct): the compute before the push is only the sensors, the gyro prediction
 * (flight: one quaternion integration, ~7.5 k cycles; pad: a copy), nl_nav3_tilt (acosf), the quantised fields, the slot
 * and nl_encode_nav3.  The floor is re-derived the same way from the v0.8 measurement: the worst compute end incl. the
 * MS5611 bus interrupts - SB_NAV_JITTER_US, rounded up to 10 us (tools/run_nav_simbench.py checks it) */
/* v0.8.1 (the glink: the pre-build decode, the GNS slot, the STATE frame's UDRE interrupts that run into the next tick's
 * compute): re-derived from tools/run_nav_simbench.py --mode vtol of 2026-10-22 - compute end incl. the MS5611 bus
 * interrupts 1559.2 us - SB_NAV_JITTER_US 250 us = 1309.2, rounded up (v0.8: 1180) */
/* fw09/p1 (P1: fixed-point estimator, interrupt-driven exchange, table CRC, byte-wise lp_quantize): re-derived from
 * run_nav_simbench.py --mode vtol with the floor at 400 us (below every compute end): compute end incl. the MS5611 bus
 * interrupts 1137.7 us - SB_NAV_JITTER_US 250 us = 887.7, rounded up (v0.8.1: 1310) */
#define NAV_PUSH_MIN_US_VTOL 890u

#define NAV_FCU_LOST_TICKS 250u      /* LED: no valid FCU frame for 1 s (information only, changes nothing) */

/* Timer1 as a free-running 0.5 us time base (FC_PWM_COUNTS_PER_US counts per us, wrapping after FC_PWM_TOP like
 * the FCU's PWM timer), so hal/timer.c's timer_tcnt1 / timer_elapsed_counts work on the NAV: mode 14 with
 * TOP = ICR1 as on the FCU, no output-compare pin connected, no interrupt (nav_timer.c) */
void nav_timebase_init(void);

#endif /* FC_NAV_H */
