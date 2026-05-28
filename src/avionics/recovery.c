/**
 * @file recovery.c
 * @brief Parachute recovery sequencer implementation.
 *
 * Two-event: drogue (apogee or timer backup) then main (altitude).
 * Each pyro fires exactly once.
 */

#include "recovery.h"

/* ---- Pyro HAL stubs (replace with real GPIO driver) ---- */

static void recovery_fire_drogue_hal(void)
{
    /* TODO: gpio_set_level(RECOVERY_DROGUE_GPIO, 1);
     * + one-shot timer to turn off after RECOVERY_PYRO_PULSE_MS */
    (void)0;
}

static void recovery_fire_main_hal(void)
{
    /* TODO: same pattern as drogue */
    (void)0;
}

/* ---- Init ---- */

void recovery_init(recovery_state_t *state)
{
    state->drogue_fired  = false;
    state->main_fired    = false;
    state->t_drogue_fire = 0.0f;
    state->t_main_fire   = 0.0f;
    state->initialized   = true;
}

/* ---- Update (once per GNC cycle) ---- */

void recovery_update(recovery_state_t *state,
                     flight_phase_t phase,
                     float alt_agl,
                     float mission_time)
{
    /* Drogue */
    if (!state->drogue_fired) {
        bool apogee_trigger = (phase == PHASE_DESCENT);
        bool timer_trigger  = (mission_time >= RECOVERY_TIMER_BACKUP_S);

        if (apogee_trigger || timer_trigger) {
            recovery_fire_drogue_hal();
            state->drogue_fired  = true;
            state->t_drogue_fire = mission_time;
        }
    }

    /* Main */
    if (state->drogue_fired && !state->main_fired) {
        float dt = mission_time - state->t_drogue_fire;

        if (alt_agl <= RECOVERY_MAIN_ALT_M &&
            dt >= RECOVERY_DROGUE_TO_MAIN_MIN_S) {
            recovery_fire_main_hal();
            state->main_fired  = true;
            state->t_main_fire = mission_time;
        }
    }
}