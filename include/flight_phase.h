#ifndef FLIGHT_PHASE_H
#define FLIGHT_PHASE_H

#include "gnc_types.h"

typedef enum {
    PHASE_RAIL = 0,
    PHASE_BOOST,
    PHASE_GUIDE,
    PHASE_CPA,
    PHASE_DESCENT,
    PHASE_COUNT
} flight_phase_t;

typedef struct {
    flight_phase_t phase;
    float          t_phase_enter;
    float          v_down_prev;
    bool           initialized;
} phase_state_t;

void phase_init(phase_state_t *ps);

flight_phase_t phase_update(phase_state_t *ps,
                            const nav_state_t *nav,
                            const gnc_config_t *cfg,
                            float t);

const char *phase_name(flight_phase_t phase);

#endif /* FLIGHT_PHASE_H */
