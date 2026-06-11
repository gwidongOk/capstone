/**
 * @file recovery.c
 * @brief Parachute recovery sequencer implementation.
 *
 * Two-event deployment, each flag is set exactly once.
 *
 * Drogue triggers (OR, first to fire wins):
 *   1. Phase-based apogee:  phase == PHASE_DESCENT (EKF v_down reversal)
 *   2. Baro-independent:    raw baro alt peaked + confirmed descent
 *   3. Timer backup:        mission_time >= RECOVERY_TIMER_BACKUP_S
 *
 * Main trigger (AND):
 *   drogue_fired AND alt_agl <= threshold AND min delay elapsed
 *
 * GPIO driving is handled by main.cpp pyro_tick(), not by this module.
 */

#include "recovery.h"

/* ---- Init ---- */

void recovery_init(recovery_state_t *state)
{
    state->drogue_fired       = false;
    state->main_fired         = false;
    state->t_drogue_fire      = 0.0f;
    state->t_main_fire        = 0.0f;
    state->initialized        = true;

    /* Baro-independent apogee state */
    state->baro_alt_max       = 0.0f;
    state->baro_armed         = false;
    state->baro_descent_count = 0;
}

/* ---- Update (once per GNC cycle) ---- */

void recovery_update(recovery_state_t *state,
                     flight_phase_t phase,
                     float alt_agl,
                     float baro_alt_raw,
                     float mission_time)
{
    /* Drogue */
    if (!state->drogue_fired) {
        /* Trigger 1: EKF-based phase state machine detected apogee */
        bool phase_trigger = (phase == PHASE_DESCENT);

        /* Trigger 2: Baro-independent apogee detection.
         * Track peak raw baro altitude. Once armed (above threshold),
         * confirm descent by counting consecutive samples where
         * current alt is below peak by more than drop threshold. */
        bool baro_trigger = false;

        if (baro_alt_raw > state->baro_alt_max) {
            state->baro_alt_max = baro_alt_raw;
        }

        if (!state->baro_armed && baro_alt_raw > RECOVERY_BARO_ARM_ALT) {
            state->baro_armed = true;
        }

        if (state->baro_armed) {
            float drop = state->baro_alt_max - baro_alt_raw;
            if (drop > RECOVERY_BARO_DROP_THRESH) {
                state->baro_descent_count++;
                if (state->baro_descent_count >= RECOVERY_BARO_CONFIRM_COUNT) {
                    baro_trigger = true;
                }
            } else {
                state->baro_descent_count = 0;
            }
        }

        /* Trigger 3: Timer backup */
        bool timer_trigger = (mission_time >= RECOVERY_TIMER_BACKUP_S);

        if (phase_trigger || baro_trigger || timer_trigger) {
            state->drogue_fired  = true;
            state->t_drogue_fire = mission_time;
        }
    }

    /* Main */
    if (state->drogue_fired && !state->main_fired) {
        float dt = mission_time - state->t_drogue_fire;

        if (alt_agl <= RECOVERY_MAIN_ALT_M &&
            dt >= RECOVERY_DROGUE_TO_MAIN_MIN_S) {
            state->main_fired  = true;
            state->t_main_fire = mission_time;
        }
    }
}