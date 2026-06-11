/**
 * @file gnc_main.h
 * @brief GNC orchestrator: phase -> guidance -> autopilot -> servo -> recovery.
 */

#ifndef GNC_MAIN_H
#define GNC_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "flight_phase.h"
#include "gnc_types.h"
#include "recovery.h"

typedef struct {
  servo_output_t servo;
  flight_phase_t phase;
  guid_output_t guid;
  ap_output_t ap;
  recovery_state_t recovery;
} gnc_output_t;

void gnc_init(gnc_state_t *state, phase_state_t *phase_state,
              recovery_state_t *rcv_state);

gnc_output_t gnc_step(const nav_state_t *nav, const gnc_config_t *cfg,
                      gnc_state_t *state, phase_state_t *phase_state,
                      recovery_state_t *rcv_state, float t,
                      float baro_alt_raw);

#ifdef __cplusplus
}
#endif

#endif /* GNC_MAIN_H */