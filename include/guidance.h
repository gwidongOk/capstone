#ifndef GUIDANCE_H
#define GUIDANCE_H

#include "gnc_types.h"

void guidance_init(guid_state_t *state);

guid_output_t guidance_step(const nav_state_t *nav,
                            const gnc_config_t *cfg,
                            guid_state_t *state,
                            float t);

#endif /* GUIDANCE_H */
