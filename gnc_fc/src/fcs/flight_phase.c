/**
 * @file flight_phase.c
 * @brief Flight phase state machine.
 *
 * Transition criteria:
 *   RAIL  -> BOOST:   accel_b.x > LAUNCH_ACCEL_G * g  AND  nav valid
 *   BOOST -> GUIDE:   V_air > V_ctrl_on  AND  t > t_guide_on  AND  nav valid
 *   GUIDE -> CPA:     v_closing < 0  (guidance enabled)
 *   GUIDE -> DESCENT: v_down sign change (guidance disabled, flight-1)
 *   CPA   -> DESCENT: v_down sign change
 */

#include "flight_phase.h"
#include "gnc_config.h"
#include <math.h>

void phase_init(phase_state_t *ps) {
  ps->phase = PHASE_RAIL;
  ps->t_phase_enter = 0.0f;
  ps->v_down_prev = 0.0f;
  ps->launch_detected = false;
  ps->initialized = true;
}

flight_phase_t phase_update(phase_state_t *ps, const nav_state_t *nav,
                            const gnc_config_t *cfg, float t) {
  float V_air = nav->airspeed;
  if (V_air < 1.0f)
    V_air = 1.0f;

  float v_down = nav->vel_ned.z;

  switch (ps->phase) {

  /* ---- RAIL: waiting for launch ---- */
  case PHASE_RAIL: {
    /* Launch detection: body-x acceleration exceeds threshold.
     * LAUNCH_ACCEL_G is in g-units, multiply by cfg->g for m/s^2.
     * Also require nav valid (sensors initialized, EKF converged). */
    float accel_threshold = LAUNCH_ACCEL_G * cfg->g;

    if (nav->accel_b.x > accel_threshold && nav->valid) {
      ps->launch_detected = true;
      ps->phase = PHASE_BOOST;
      ps->t_phase_enter = t;
    }
    break;
  }

  /* ---- BOOST: motor burning, rate damping only ---- */
  case PHASE_BOOST: {
    bool aero_ok = (V_air > cfg->V_ctrl_on);

    if (aero_ok && t > cfg->t_guide_on && nav->valid) {
      ps->phase = PHASE_GUIDE;
      ps->t_phase_enter = t;
    }
    break;
  }

  /* ---- GUIDE: guidance + control (or control-only if no guidance) ---- */
  case PHASE_GUIDE:
    if (cfg->guidance_enabled) {
      /* Normal mission: CPA detection */
      vec3_t los = {cfg->target_ned.x - nav->pos_ned.x,
                    cfg->target_ned.y - nav->pos_ned.y,
                    cfg->target_ned.z - nav->pos_ned.z};
      float R = sqrtf(los.x * los.x + los.y * los.y + los.z * los.z);

      float v_closing = 0.0f;
      if (R > 2.0f) {
        v_closing = (los.x * nav->vel_ned.x + los.y * nav->vel_ned.y +
                     los.z * nav->vel_ned.z) /
                    R;
      }

      if (v_closing < 0.0f && t > cfg->t_guide_on + 1.0f) {
        ps->phase = PHASE_CPA;
        ps->t_phase_enter = t;
      }
    } else {
      /* No guidance (flight-1): detect apogee directly.
       * Guard: 2 s after GUIDE entry to reject boost tail-off. */
      if (ps->v_down_prev < 0.0f && v_down >= 0.0f &&
          t > ps->t_phase_enter + 2.0f) {
        ps->phase = PHASE_DESCENT;
        ps->t_phase_enter = t;
      }
    }
    break;

  /* ---- CPA: ballistic coast, waiting for apogee ---- */
  case PHASE_CPA:
    if (ps->v_down_prev < 0.0f && v_down >= 0.0f &&
        t > ps->t_phase_enter + 0.5f) {
      ps->phase = PHASE_DESCENT;
      ps->t_phase_enter = t;
    }
    break;

  /* ---- DESCENT: recovery module handles chutes, detect landing ---- */
  case PHASE_DESCENT: {
    float alt_agl = -nav->pos_ned.z;
    if (alt_agl < 0.0f)
      alt_agl = 0.0f;
    float vel_d_abs = fabsf(nav->vel_ned.z);

    if (alt_agl < LANDED_ALT_THRESH && vel_d_abs < LANDED_VEL_THRESH &&
        t > ps->t_phase_enter + LANDED_DWELL_S) {
      ps->phase = PHASE_LANDED;
      ps->t_phase_enter = t;
    }
    break;
  }

  /* ---- LANDED: mission complete ---- */
  case PHASE_LANDED:
    break;

  default:
    break;
  }

  ps->v_down_prev = v_down;
  return ps->phase;
}

const char *phase_name(flight_phase_t phase) {
  static const char *names[] = {"RAIL", "BOOST",   "ACTIVE",
                                "CPA",  "DESCENT", "LANDED"};
  if (phase < PHASE_COUNT) {
    return names[phase];
  }
  return "UNKNOWN";
}