/**
 * @file flight_phase.h
 * @brief Flight phase state machine.
 *
 * Phases (one-directional):
 *   RAIL    -> BOOST   : launch accel detected AND nav valid
 *   BOOST   -> GUIDE   : qbar sufficient AND t > t_guide_on AND nav valid
 *   GUIDE   -> CPA     : closing vel < 0 (guidance ON)
 *   GUIDE   -> DESCENT : apogee detected (guidance OFF, flight-1)
 *   CPA     -> DESCENT : apogee detected
 *   DESCENT            : terminal, recovery module takes over
 */

#ifndef FLIGHT_PHASE_H
#define FLIGHT_PHASE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gnc_types.h"

/* Launch detection threshold [g-units].
 * Overridable in gnc_config.h; default = 2.0g. */
#ifndef LAUNCH_ACCEL_G
#define LAUNCH_ACCEL_G 2.0f
#endif

/* Landing detection thresholds.
 * DESCENT -> LANDED: alt < LANDED_ALT_THRESH AND |vel_D| < LANDED_VEL_THRESH
 * with a minimum dwell time in DESCENT to prevent false triggers. */
#ifndef LANDED_ALT_THRESH
#define LANDED_ALT_THRESH 10.0f  /* AGL [m] */
#endif
#ifndef LANDED_VEL_THRESH
#define LANDED_VEL_THRESH 1.0f   /* |vel_down| [m/s] */
#endif
#ifndef LANDED_DWELL_S
#define LANDED_DWELL_S 5.0f      /* min time in DESCENT before landing [s] */
#endif

typedef enum {
  PHASE_RAIL = 0,
  PHASE_BOOST,
  PHASE_GUIDE,
  PHASE_CPA,
  PHASE_DESCENT,
  PHASE_LANDED,
  PHASE_COUNT
} flight_phase_t;

typedef struct {
  flight_phase_t phase;
  float t_phase_enter;
  float v_down_prev;
  bool launch_detected; /* latched on first accel trigger */
  bool initialized;
} phase_state_t;

void phase_init(phase_state_t *ps);

flight_phase_t phase_update(phase_state_t *ps, const nav_state_t *nav,
                            const gnc_config_t *cfg, float t);

const char *phase_name(flight_phase_t phase);

#ifdef __cplusplus
}
#endif

#endif /* FLIGHT_PHASE_H */