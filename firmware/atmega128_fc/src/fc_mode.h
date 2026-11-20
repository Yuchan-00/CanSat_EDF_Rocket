/* fc_mode.h - the mission a firmware build flies (params.json mission.mode).
 *
 * The rocket flies one mission: the v0.4 - v0.6.3 climb / CUT / RELEASE program (MISSION_MODE_BALLISTIC; TELEM v1 /
 * IL_SNAP, log format 4).  Every PlatformIO env builds it with -D FC_MODE_BALLISTIC (the FCU envs also
 * -D ONB_FIXED_MODE=LP_MODE_BALLISTIC).  The VTOL program (MISSION_MODE_VTOL) was removed with the VTOL vehicle on
 * 2026-11-18; params_gen.h may still define its value until the next generator run.
 *
 * Invariant: FC_MISSION_MODE is a preprocessor constant, identical for every translation unit of one build, and equal to
 * the mode the supervisor runs (onboard_init forces LP_MODE_BALLISTIC).
 */
#ifndef FC_FC_MODE_H
#define FC_FC_MODE_H

#include "params_gen.h"
#include "core/link_proto.h"

#if !defined(MISSION_MODE) || !defined(MISSION_MODE_BALLISTIC)
#error "params_gen.h has no MISSION_MODE: re-run tools/gen_firmware_params.py"
#endif
#if MISSION_MODE_BALLISTIC != LP_MODE_BALLISTIC
#error "params_gen.h MISSION_MODE_BALLISTIC differs from link_proto.h LP_MODE_BALLISTIC"
#endif
#if defined(ONB_FIXED_MODE) && ONB_FIXED_MODE != LP_MODE_BALLISTIC
#error "ONB_FIXED_MODE must be LP_MODE_BALLISTIC (the rocket's only mission)"
#endif
#define FC_MISSION_MODE MISSION_MODE_BALLISTIC
#define FC_MODE_NAME "ballistic"

#endif /* FC_FC_MODE_H */
