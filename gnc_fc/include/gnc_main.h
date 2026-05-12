#ifndef GNC_MAIN_H
#define GNC_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gnc_types.h"
#include "flight_phase.h"

typedef struct {
    servo_output_t  servo;
    flight_phase_t  phase;
    guid_output_t   guid;
    ap_output_t     ap;
} gnc_output_t;

void gnc_init(gnc_state_t *state, phase_state_t *phase_state);

gnc_output_t gnc_step(const nav_state_t *nav, const gnc_config_t *cfg,
                       gnc_state_t *state, phase_state_t *phase_state, float t);

#ifdef __cplusplus
}
#endif

#endif /* GNC_MAIN_H */
