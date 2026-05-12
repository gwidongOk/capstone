/**
 * @file flight_phase.c
 * Flight phase state machine. All transitions are one-directional.
 *
 *   RAIL:     fins = 0, no control
 *   BOOST:    rate damping only (weathercock stability)
 *   GUIDE:    full guidance + autopilot
 *   CPA:      ballistic + rate damping
 *   DESCENT:  fins = 0, recovery handles parachute
 */

#include "flight_phase.h"
#include <math.h>

void phase_init(phase_state_t *ps)
{
    ps->phase         = PHASE_RAIL;
    ps->t_phase_enter = 0.0f;
    ps->v_down_prev   = 0.0f;
    ps->initialized   = true;
}

flight_phase_t phase_update(phase_state_t *ps,
                            const nav_state_t *nav,
                            const gnc_config_t *cfg,
                            float t)
{
    float V_air = nav->airspeed;
    if (V_air < 1.0f) V_air = 1.0f;

    float alt = -nav->pos_ned.z;
    if (alt < 0.0f) alt = 0.0f;
    float rho = 1.225f * expf(-alt / 8500.0f);
    float qbar = 0.5f * rho * V_air * V_air;

    float v_down = nav->vel_ned.z;

    switch (ps->phase) {

    case PHASE_RAIL:
        if (t > 0.01f) {
            ps->phase = PHASE_BOOST;
            ps->t_phase_enter = t;
        }
        break;

    case PHASE_BOOST:
        {
            bool aero_ok;
            if (cfg->V_ctrl_on > 0.0f) {
                aero_ok = (V_air > cfg->V_ctrl_on);
            } else {
                aero_ok = (qbar > cfg->qbar_min_ctrl);
            }

            if (aero_ok && t > cfg->t_guide_on && nav->valid) {
                ps->phase = PHASE_GUIDE;
                ps->t_phase_enter = t;
            }
        }
        break;

    case PHASE_GUIDE:
        /* Transition to CPA when closing velocity goes negative.
         * Guard: at least 1s after guidance start to avoid false triggers. */
        {
            vec3_t los = {cfg->target_ned.x - nav->pos_ned.x,
                          cfg->target_ned.y - nav->pos_ned.y,
                          cfg->target_ned.z - nav->pos_ned.z};
            float R = sqrtf(los.x*los.x + los.y*los.y + los.z*los.z);

            float v_closing = 0.0f;
            if (R > 2.0f) {
                v_closing = (los.x * nav->vel_ned.x +
                             los.y * nav->vel_ned.y +
                             los.z * nav->vel_ned.z) / R;
            }

            if (v_closing < 0.0f && t > cfg->t_guide_on + 1.0f) {
                ps->phase = PHASE_CPA;
                ps->t_phase_enter = t;
            }
        }
        break;

    case PHASE_CPA:
        /* Transition to DESCENT at apogee: v_down crosses from negative to positive.
         * Guard: at least 0.5s after CPA. */
        if (ps->v_down_prev < 0.0f && v_down >= 0.0f &&
            t > ps->t_phase_enter + 0.5f) {
            ps->phase = PHASE_DESCENT;
            ps->t_phase_enter = t;
        }
        break;

    case PHASE_DESCENT:
        break;

    default:
        break;
    }

    ps->v_down_prev = v_down;

    return ps->phase;
}

const char *phase_name(flight_phase_t phase)
{
    static const char *names[] = {
        "RAIL",
        "BOOST",
        "GUIDE",
        "CPA",
        "DESCENT"
    };
    if (phase < PHASE_COUNT) {
        return names[phase];
    }
    return "UNKNOWN";
}
