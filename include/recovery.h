/**
 * @file recovery.h
 * @brief Parachute recovery sequencer.
 *
 * Two-event deployment:
 *   1. Drogue at apogee (or timer backup)
 *   2. Main at altitude threshold
 *
 * Pyro HAL is stubbed out -- replace with real GPIO driver.
 */

#ifndef RECOVERY_H
#define RECOVERY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "flight_phase.h"

/* ---- Config (placeholder values, replace before flight) ---- */

#define RECOVERY_MAIN_ALT_M           150.0f  /* main deploy alt AGL [m] */
#define RECOVERY_TIMER_BACKUP_S        15.0f  /* backup drogue timer [s] */
#define RECOVERY_PYRO_PULSE_MS         200    /* pyro pulse duration [ms] */
#define RECOVERY_DROGUE_GPIO           (-1)   /* placeholder pin */
#define RECOVERY_MAIN_GPIO             (-1)   /* placeholder pin */
#define RECOVERY_DROGUE_TO_MAIN_MIN_S   2.0f  /* min delay drogue->main [s] */

/* ---- State ---- */

typedef struct {
    bool  drogue_fired;
    bool  main_fired;
    float t_drogue_fire;    /* mission time of drogue fire [s] */
    float t_main_fire;
    bool  initialized;
} recovery_state_t;

/* ---- API ---- */

void recovery_init(recovery_state_t *state);

/**
 * Call once per GNC cycle.
 *
 * Drogue fires when: phase==PHASE_DESCENT OR mission_time >= backup timer.
 * Main fires when: drogue fired AND alt <= threshold AND min delay elapsed.
 */
void recovery_update(recovery_state_t *state,
                     flight_phase_t phase,
                     float alt_agl,
                     float mission_time);

#ifdef __cplusplus
}
#endif

#endif /* RECOVERY_H */