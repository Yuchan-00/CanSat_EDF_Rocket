/* fc_mode.h - the mission mode a firmware build flies (v0.7, SPEC 8.9; params.json mission.mode).
 *
 * mission.mode is a BUILD-TIME choice (params.json mission.mode_note): params_gen.h carries MISSION_MODE, and every
 * chip's firmware (FCU main.c, the COM's com_link.c, the FCU HAL's I10 record) is compiled for exactly one mode:
 *   FC_MISSION_MODE   MISSION_MODE_VTOL      ON / OFF / UP / DOWN, the vertical controller, TELEM v2 / IL_SNAP2,
 *                                            log format 5 (default: params.json mission.mode = "vtol")
 *                     MISSION_MODE_BALLISTIC the v0.4 - v0.6.3 climb / CUT / RELEASE mission, bit-identical in
 *                                            behaviour to v0.6.3 (TELEM v1 / IL_SNAP, log format 4)
 * The mode comes from params_gen.h unless a PlatformIO env pins it (platformio.ini: the *_ballistic envs build
 * -D FC_MODE_BALLISTIC -D ONB_FIXED_MODE=LP_MODE_BALLISTIC from the same params.json; -D FC_MODE_VTOL likewise).
 * ONB_FIXED_MODE (core/onboard.c) drops the other mode's supervisor code; when a build sets it, it must name the
 * same mode (checked below), so the supervisor and the code around it can never disagree.
 *
 * Invariant: FC_MISSION_MODE is a preprocessor constant, identical for every translation unit of one build, and
 * equal to the mode the supervisor runs (main.c passes it to onboard_init / onb_stack_init through op.mode).
 */
#ifndef FC_FC_MODE_H
#define FC_FC_MODE_H

#include "params_gen.h"
#include "core/link_proto.h"

#if !defined(MISSION_MODE) || !defined(MISSION_MODE_BALLISTIC) || !defined(MISSION_MODE_VTOL)
#error "params_gen.h has no MISSION_MODE: re-run tools/gen_firmware_params.py (params.json v0.7 mission.mode)"
#endif
#if MISSION_MODE_BALLISTIC != LP_MODE_BALLISTIC || MISSION_MODE_VTOL != LP_MODE_VTOL
#error "params_gen.h MISSION_MODE_* differ from link_proto.h LP_MODE_*"
#endif
#if defined(FC_MODE_BALLISTIC) && defined(FC_MODE_VTOL)
#error "a build flies one mission: -D FC_MODE_BALLISTIC or -D FC_MODE_VTOL, not both"
#endif

#if defined(FC_MODE_BALLISTIC)
#define FC_MISSION_MODE MISSION_MODE_BALLISTIC
#elif defined(FC_MODE_VTOL)
#define FC_MISSION_MODE MISSION_MODE_VTOL
#else
#define FC_MISSION_MODE MISSION_MODE          /* params.json mission.mode */
#endif

#define FC_VTOL (FC_MISSION_MODE == MISSION_MODE_VTOL)

#if FC_VTOL && !MISSION_HAS_VTOL_PARAMS
#error "a VTOL build needs params.json mission.vtol (params_gen.h MISSION_HAS_VTOL_PARAMS)"
#endif
#if defined(ONB_FIXED_MODE) && ONB_FIXED_MODE != FC_MISSION_MODE
#error "ONB_FIXED_MODE (the supervisor's compile-time mode) differs from this build's FC_MISSION_MODE"
#endif

#if FC_VTOL
#define FC_MODE_NAME "vtol"
#else
#define FC_MODE_NAME "ballistic"
#endif

#endif /* FC_FC_MODE_H */
