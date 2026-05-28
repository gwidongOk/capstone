/**
 * @file gnc_main.c
 * @brief GNC orchestrator: phase -> guidance -> autopilot -> servo -> recovery.
 */

#include "gnc_main.h"
#include "autopilot.h"
#include "gnc_config.h"
#include "guidance.h"
#include "math_utils.h"
#include "recovery.h"

#include <string.h>

/* ---- Fin command -> Servo PWM ---- */

static servo_output_t fin_to_servo(const float fin_cmd[4], float delta_max) {
  servo_output_t srv;

  static const int8_t servo_dir[4] = {+1, +1, +1, +1};
  static const int16_t servo_trim[4] = {0, 0, 0, 0};

  for (int i = 0; i < 4; i++) {
    float normalized = fin_cmd[i] / delta_max;
    normalized = clampf(normalized, -1.0f, 1.0f);

    float half_range = (float)(SERVO_PWM_MAX_US - SERVO_PWM_MIN_US) * 0.5f;
    float center = (float)SERVO_PWM_CENTER_US;
    float pulse = center + (float)servo_dir[i] * normalized * half_range +
                  (float)servo_trim[i];

    if (pulse < (float)SERVO_PWM_MIN_US)
      pulse = (float)SERVO_PWM_MIN_US;
    if (pulse > (float)SERVO_PWM_MAX_US)
      pulse = (float)SERVO_PWM_MAX_US;

    srv.pulse_us[i] = (uint16_t)(pulse + 0.5f);
  }

  return srv;
}

/* ---- GNC Init ---- */

void gnc_init(gnc_state_t *state, phase_state_t *phase_state,
              recovery_state_t *rcv_state) {
  memset(state, 0, sizeof(gnc_state_t));
  guidance_init(&state->guid);
  autopilot_init(&state->ap);
  state->armed = false;
  state->mission_time = 0.0f;
  state->end_reason = MISSION_ACTIVE;
  state->prev_v_closing = -1.0f;
  state->prev_vD = -1.0f;

  phase_init(phase_state);
  recovery_init(rcv_state);
}

/* ---- GNC Step ---- */

gnc_output_t gnc_step(const nav_state_t *nav, const gnc_config_t *cfg,
                      gnc_state_t *state, phase_state_t *phase_state,
                      recovery_state_t *rcv_state, float t) {
  gnc_output_t out;
  memset(&out, 0, sizeof(out));

  /* 1. Phase update */
  out.phase = phase_update(phase_state, nav, cfg, t);

  /* 2. Recovery (runs every cycle, uses phase + altitude + time) */
  float alt_agl = -nav->pos_ned.z;
  if (alt_agl < 0.0f)
    alt_agl = 0.0f;
  recovery_update(rcv_state, out.phase, alt_agl, t);
  out.recovery = *rcv_state;

  /* 3. Dispatch guidance/autopilot */
  float nz_cmd = 0.0f;
  float ny_cmd = 0.0f;

  switch (out.phase) {
  case PHASE_RAIL:
    for (int i = 0; i < 4; i++)
      out.ap.fin_cmd[i] = 0.0f;
    out.servo = fin_to_servo(out.ap.fin_cmd, cfg->delta_max);
    return out;

  case PHASE_BOOST:
    nz_cmd = 0.0f;
    ny_cmd = 0.0f;
    break;

  case PHASE_GUIDE:
    if (cfg->guidance_enabled) {
      out.guid = guidance_step(nav, cfg, &state->guid, t);
      nz_cmd = out.guid.nz_cmd;
      ny_cmd = out.guid.ny_cmd;
    } else {
      nz_cmd = 0.0f;
      ny_cmd = 0.0f;
    }
    break;

  case PHASE_CPA:
    nz_cmd = 0.0f;
    ny_cmd = 0.0f;
    break;

  case PHASE_DESCENT:
  case PHASE_LANDED:
    for (int i = 0; i < 4; i++)
      out.ap.fin_cmd[i] = 0.0f;
    out.servo = fin_to_servo(out.ap.fin_cmd, cfg->delta_max);
    return out;

  default:
    out.servo = fin_to_servo(out.ap.fin_cmd, cfg->delta_max);
    return out;
  }

  /* 4. Autopilot */
  out.ap = autopilot_step(nz_cmd, ny_cmd, nav, cfg, &state->ap);

  /* 5. Fin -> Servo */
  out.servo = fin_to_servo(out.ap.fin_cmd, cfg->delta_max);

  return out;
}