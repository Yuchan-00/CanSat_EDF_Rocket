/* fc_config.h - the firmware's parameter structs, filled only from the generated
 * headers src/params_gen.h (tools/gen_firmware_params.py) and src/gains.h
 * (tools/gen_gains_header.py).
 *
 * Invariant: this is the single place where generated macros become runtime
 * structs; no other file reads a plant, gain or supervisor macro, so the host
 * tests and the cross-check exercise exactly the numbers the firmware flies.
 */
#ifndef FC_FC_CONFIG_H
#define FC_FC_CONFIG_H

#include "alt_filter.h"
#include "att_estimator.h"
#include "attitude_ctrl.h"
#include "glink_proto.h"
#include "gns_node.h"
#include "gps_nav.h"
#include "onboard.h"
#include "position_ctrl.h"
#include "thrust_model.h"
#include "vertical_ctrl.h"

void fc_default_plant(mixer_plant_t *p);
void fc_default_gains(att_gains_t *g);          /* gains.h: the ballistic build's attitude gains (v0.6.3) */
void fc_vtol_gains(att_gains_t *g);             /* v0.7.1b: gains.h + params.json control.vtol_overrides (VTOL build) */
/* the supervisor's and the vertical controller's parameters.  v0.8.1 RAM diet: an AVR VTOL build (ONB_PARAMS_FLASH /
 * VC_PARAMS_FLASH) keeps them in flash - onboard_init / onb_stack_init take fc_onboard_params() and fc_vertical_params()
 * (flash addresses: never pass a RAM copy there) and the RAM fillers below do not exist in it (a compile error rather than
 * a RAM pointer read as flash); host builds and the ballistic FCU fill RAM copies */
#if !ONB_PARAMS_FLASH
void fc_default_onboard(onb_params_t *op);      /* mode = MISSION_MODE; the VTOL fields when mission.vtol exists */
#endif
#if !VC_PARAMS_FLASH
void fc_default_vertical(vc_params_t *vp);      /* VTOL vertical controller (inert values without mission.vtol) */
#endif
const onb_params_t *fc_onboard_params(void);    /* flash (ONB_PARAMS_FLASH) or a const RAM table (mode = MISSION_MODE) */
const vc_params_t *fc_vertical_params(void);    /* flash (VC_PARAMS_FLASH) or a const RAM table */
void fc_default_mahony(att_mahony_t *m);         /* v0.7.1 VTOL NAV estimator (mission.vtol.nav_estimator) */
void fc_default_alt_vtol(alt_vtol_t *f, float baro_latency_s);   /* v0.7.1 VTOL NAV altitude filter */
/* v0.8 GPS position hold (params mission.vtol.position_hold for the build profile, flight_computer.nav.gps) */
void fc_default_position(pos_params_t *pp);     /* FCU position controller (sim.position_ctrl.PositionParams) */
/* the same parameters for pos_init / onb_stack_init: on the AVR a FLASH address (POS_PARAMS_FLASH, read with pgm_read_*;
 * never pass a RAM copy there), on the host a const RAM struct */
const pos_params_t *fc_position_params(void);
void fc_default_gps_nav(gps_nav_params_t *gp);  /* NAV GPS filter (sim.gps_nav.NavGpsParams + compute_gains) */
/* the NAV's whole GPS side with the boot configurator's tick counts and baud of params_gen.h (v0.8, three chips) */
void fc_default_gps(gps_nav_t *g, bool configure);
/* v0.8.1: the GNS board's application core (core/gns_node.h) with the GPS parameters, the position controller's table
 * (fc_position_params) and the boot configurator of params_gen.h */
void fc_default_gns(gns_t *g, bool configure);
uint8_t fc_glink_latency_ticks(void);           /* GLINK_LATENCY_TICKS (params glink.latency_ticks_ASSUMED) */
int  fc_throttle_exp_int(void);
void fc_default_edf(edf_model_t *m);
void fc_default_estimator(att_est_t *e);
void fc_default_alt_filter(alt_filter_t *f, float baro_latency_s);

#endif /* FC_FC_CONFIG_H */
