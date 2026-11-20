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
#include "onboard.h"
#include "thrust_model.h"

void fc_default_plant(mixer_plant_t *p);
void fc_default_gains(att_gains_t *g);          /* gains.h: the rocket's attitude gains (v0.6.3) */
void fc_default_onboard(onb_params_t *op);      /* the supervisor's parameters (mode = MISSION_MODE) */
const onb_params_t *fc_onboard_params(void);    /* the same as a const RAM table filled once */
int  fc_throttle_exp_int(void);
void fc_default_edf(edf_model_t *m);
void fc_default_estimator(att_est_t *e);
void fc_default_alt_filter(alt_filter_t *f, float baro_latency_s);

#endif /* FC_FC_CONFIG_H */
