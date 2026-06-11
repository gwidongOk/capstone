/**
 * @file recovery.h
 * @brief Parachute recovery sequencer.
 *
 * Two-event deployment:
 *   1. Drogue at apogee (phase, baro-independent, or timer backup)
 *   2. Main at altitude threshold
 *
 * Drogue triggers (OR):
 *   - Phase-based apogee: phase == PHASE_DESCENT (EKF-dependent)
 *   - Baro-independent apogee: raw baro alt peaked and descending
 *   - Timer backup: mission_time >= RECOVERY_TIMER_BACKUP_S
 *
 * Pyro GPIO driven by main.cpp via drogue_fired / main_fired flags.
 * Pulse timing managed by pyro_tick() at 10Hz in flight_task.
 */

#ifndef RECOVERY_H
#define RECOVERY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "flight_phase.h"

/* ---- Config ---- */

#define RECOVERY_MAIN_ALT_M           450.0f  /* main deploy alt AGL [m] */
#define RECOVERY_TIMER_BACKUP_S        10.0f  /* backup drogue timer [s] */
#define RECOVERY_DROGUE_TO_MAIN_MIN_S   2.0f  /* min delay drogue->main [s] */

/* Baro-independent apogee detection (EKF-free fallback).
 * Uses raw barometer altitude, independent of flight_phase state machine.
 *   ARM:     baro_alt > BARO_ARM_ALT  (latch)
 *   TRIGGER: baro_alt_max - baro_alt > BARO_DROP_THRESH for N consecutive cycles
 */
#define RECOVERY_BARO_ARM_ALT          50.0f  /* arm altitude AGL [m] */
#define RECOVERY_BARO_DROP_THRESH      10.0f  /* peak-to-current drop [m] */
#define RECOVERY_BARO_CONFIRM_COUNT    3      /* consecutive descent samples */

/* ---- State ---- */

typedef struct {
    bool  drogue_fired;
    bool  main_fired;
    float t_drogue_fire;    /* mission time of drogue fire [s] */
    float t_main_fire;
    bool  initialized;

    /* Baro-independent apogee detection state */
    float   baro_alt_max;        /* peak baro altitude tracked [m] */
    bool    baro_armed;          /* true once baro_alt > arm threshold */
    uint8_t baro_descent_count;  /* consecutive descent sample counter */
} recovery_state_t;

/* ---- API ---- */

void recovery_init(recovery_state_t *state);

/**
 * Call once per GNC cycle.
 *
 * @param state        Recovery state (persistent)
 * @param phase        Current flight phase (from phase state machine)
 * @param alt_agl      EKF-derived altitude AGL [m] (for main deploy)
 * @param baro_alt_raw Raw barometer altitude AGL [m] (EKF-independent)
 * @param mission_time Time since launch detection [s]
 *
 * Drogue fires when: phase==PHASE_DESCENT
 *                  OR baro apogee detected (raw baro peaked + descending)
 *                  OR mission_time >= backup timer.
 * Main fires when: drogue fired AND alt <= threshold AND min delay elapsed.
 */
void recovery_update(recovery_state_t *state,
                     flight_phase_t phase,
                     float alt_agl,
                     float baro_alt_raw,
                     float mission_time);

#ifdef __cplusplus
}
#endif

#endif /* RECOVERY_H */