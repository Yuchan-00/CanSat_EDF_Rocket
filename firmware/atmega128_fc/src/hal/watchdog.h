/* watchdog.h - watchdog, reset cause and the in-flight record that survives a reset.
 *
 * v0.7 (fc_mode.h): "in flight" follows the build's mission (core/onboard.h onboard_flying_mode).  VTOL: POWERED, ABORT,
 * RELEASED - not UNPOWERED, which is LANDED (on the ground, motor off): a reset there must not boot ABORT and fire the
 * release at the people handling the vehicle; a VTOL I10 boot is ABORT with the release opened by the supervisor
 * abort_release_delay_s later (main.c), not at boot.  Ballistic: unchanged (below).
 *
 * The reset cause (MCUCSR) is captured in .init3, before main, and the WDT is
 * disabled there so a slow initialisation cannot loop in resets; main enables
 * it (WDT_TIMEOUT) once the outputs are in their safe state.
 *
 * A small record in .noinit RAM holds the onboard supervisor state.  After a
 * watchdog or brown-out reset (RAM content kept) a valid record whose state is
 * POWERED, UNPOWERED, ABORT or RELEASED means the vehicle was flying (interlock I10,
 * SPEC 8.8): the firmware boots SAFE with the motor off as always, opens the
 * release servo (recovery) and refuses to re-arm until a power cycle.  A power-on
 * reset never trusts the record.
 *
 * Invariant: every boot is disarmed with the ESC at its off pulse; the record
 * can only add "release open", never "motor on".
 */
#ifndef FC_WATCHDOG_H
#define FC_WATCHDOG_H

#include <stdbool.h>
#include <stdint.h>

#define WDT_TIMEOUT WDTO_60MS

#define RESET_POWER_ON 0x01
#define RESET_EXTERNAL 0x02
#define RESET_BROWN_OUT 0x04
#define RESET_WATCHDOG 0x08
#define RESET_JTAG 0x10

uint8_t watchdog_reset_cause(void);          /* MCUCSR bits at boot */
void    watchdog_start(void);
void    watchdog_kick(void);
void    flight_record_store(uint8_t state);
bool    flight_record_inflight_reset(void);  /* evaluated once at boot */
uint8_t flight_record_state(void);

#endif /* FC_WATCHDOG_H */
