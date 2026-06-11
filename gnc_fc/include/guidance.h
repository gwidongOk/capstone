#ifndef GUIDANCE_H
#define GUIDANCE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gnc_types.h"

void guidance_init(guid_state_t *state);

guid_output_t guidance_step(const nav_state_t *nav, const gnc_config_t *cfg,
                            guid_state_t *state, float t);

#ifdef __cplusplus
}
#endif

#endif /* GUIDANCE_H */
