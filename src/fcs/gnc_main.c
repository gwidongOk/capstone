/**
 * @file gnc_main.c
 * GNC orchestrator: phase -> guidance -> autopilot -> servo.
 * Called directly from flight_task (no FreeRTOS, no mutex).
 */

#include "gnc_main.h"
#include "gnc_config.h"
#include "guidance.h"
#include "autopilot.h"
#include "math_utils.h"

#include <string.h>

static servo_output_t fin_to_servo(const float fin_cmd[4], float delta_max)
{
    servo_output_t srv;

    static const int8_t  servo_dir[4]  = {+1, +1, +1, +1};
    static const int16_t servo_trim[4] = {0, 0, 0, 0};

    for (int i = 0; i < 4; i++) {
        float normalized = fin_cmd[i] / delta_max;
        normalized = clampf(normalized, -1.0f, 1.0f);

        float half_range = (float)(SERVO_PWM_MAX_US - SERVO_PWM_MIN_US) * 0.5f;
        float center = (float)SERVO_PWM_CENTER_US;
        float pulse = center + (float)servo_dir[i] * normalized * half_range
                      + (float)servo_trim[i];

        if (pulse < (float)SERVO_PWM_MIN_US) pulse = (float)SERVO_PWM_MIN_US;
        if (pulse > (float)SERVO_PWM_MAX_US) pulse = (float)SERVO_PWM_MAX_US;

        srv.pulse_us[i] = (uint16_t)(pulse + 0.5f);
    }

    return srv;
}

void gnc_init(gnc_state_t *state, phase_state_t *phase_state)
{
    memset(state, 0, sizeof(gnc_state_t));
    guidance_init(&state->guid);
    autopilot_init(&state->ap);
    state->armed = false;
    state->mission_time = 0.0f;
    state->end_reason = MISSION_ACTIVE;
    state->prev_v_closing = -1.0f;
    state->prev_vD = -1.0f;

    phase_init(phase_state);
}

gnc_output_t gnc_step(const nav_state_t *nav, const gnc_config_t *cfg,
                       gnc_state_t *state, phase_state_t *phase_state, float t)
{
    gnc_output_t out;
    memset(&out, 0, sizeof(out));

    out.phase = phase_update(phase_state, nav, cfg, t);

    float nz_cmd = 0.0f;
    float ny_cmd = 0.0f;

    switch (out.phase) {
    case PHASE_RAIL:
        for (int i = 0; i < 4; i++) out.ap.fin_cmd[i] = 0.0f;
        out.servo = fin_to_servo(out.ap.fin_cmd, cfg->delta_max);
        return out;

    case PHASE_BOOST:
        nz_cmd = 0.0f;
        ny_cmd = 0.0f;
        break;

    case PHASE_GUIDE:
        out.guid = guidance_step(nav, cfg, &state->guid, t);
        nz_cmd = out.guid.nz_cmd;
        ny_cmd = out.guid.ny_cmd;
        break;

    case PHASE_CPA:
        /* Post-CPA: fins locked to zero. Passive aero stability only;
         * active rate damping at low airspeed causes oscillation. */
        for (int i = 0; i < 4; i++) out.ap.fin_cmd[i] = 0.0f;
        out.servo = fin_to_servo(out.ap.fin_cmd, cfg->delta_max);
        return out;

    case PHASE_DESCENT:
        for (int i = 0; i < 4; i++) out.ap.fin_cmd[i] = 0.0f;
        out.servo = fin_to_servo(out.ap.fin_cmd, cfg->delta_max);
        return out;

    default:
        out.servo = fin_to_servo(out.ap.fin_cmd, cfg->delta_max);
        return out;
    }

    out.ap = autopilot_step(nz_cmd, ny_cmd, nav, cfg, &state->ap);
    out.servo = fin_to_servo(out.ap.fin_cmd, cfg->delta_max);

    return out;
}
