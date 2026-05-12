#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include "gnc_types.h"

void autopilot_init(ap_state_t *state);

ap_output_t autopilot_step(float nz_cmd, float ny_cmd,
                           const nav_state_t *nav,
                           const gnc_config_t *cfg,
                           ap_state_t *state);

#endif /* AUTOPILOT_H */
