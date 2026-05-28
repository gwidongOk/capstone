/**
 * @file gnc_types.h
 * @brief GNC type definitions.
 *
 * Shared across all GNC modules.
 * float (single precision) for ESP32-S3 FPU.
 * Frame: NED, quaternion scalar-first, angles in radians.
 */

#ifndef GNC_TYPES_H
#define GNC_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* ---- Vector / Quaternion ---- */

typedef struct {
  float x, y, z;
} vec3_t;

typedef struct {
  float q0, q1, q2, q3; /* scalar-first */
} quat_t;

/* ---- Nav state (input from nav module) ---- */

typedef struct {
  vec3_t pos_ned;  /* position NED [m]                */
  vec3_t vel_ned;  /* velocity NED [m/s]              */
  quat_t quat;     /* attitude quat (body <- NED)     */
  vec3_t omega_b;  /* angular rate body [rad/s] p,q,r */
  vec3_t accel_b;  /* accel body [m/s^2] (info only)  */
  vec3_t wind_est; /* wind NED [m/s], {0,0,0} if N/A  */
  float airspeed;  /* airspeed [m/s]                  */
  uint32_t timestamp_ms;
  bool valid;
} nav_state_t;

/* ---- Control mode (per-axis) ---- */

typedef enum {
  CTRL_OFF = 0,   /* zero deflection on this axis       */
  CTRL_RATE_DAMP, /* rate feedback only (-KR * omega)   */
  CTRL_FULL       /* full loop (angle/accel tracking)   */
} ctrl_mode_t;

/* ---- Guidance output ---- */

typedef struct {
  float nz_cmd;       /* pitch accel cmd [g]     */
  float ny_cmd;       /* yaw accel cmd [g]       */
  float range_to_tgt; /* slant range [m]         */
  float v_closing;    /* closing speed [m/s]     */
  bool past_cpa;
  bool active;
} guid_output_t;

/* ---- Guidance internal state ---- */

typedef struct {
  float los_rate_filt[3]; /* LOS rate filter (NED) */
  float ramp_timer;
  bool initialized;
} guid_state_t;

/* ---- Autopilot output ---- */

typedef struct {
  float fin_cmd[4]; /* fin commands [rad]          */
  float de_cmd;     /* pitch cmd [rad] (diag)      */
  float dr_cmd;     /* yaw cmd [rad] (diag)        */
  float da_cmd;     /* roll cmd [rad] (diag)       */
} ap_output_t;

/* ---- Autopilot internal state ---- */

typedef struct {
  float roll_int;
  float roll_e_prev;
  float roll_ramp; /* ramp-in timer [s] after Mach guard release */
  float pitch_int; /* unused if wI=0 */
  float pitch_e_prev;
  float yaw_int;
  float yaw_e_prev;
  bool initialized;
} ap_state_t;

/* ---- Gain table (velocity scheduling) ---- */

#define GAIN_TABLE_MAX_PTS 20

typedef struct {
  uint8_t n_pts;
  float V_bp[GAIN_TABLE_MAX_PTS]; /* velocity breakpoints [m/s] */
  float KR[GAIN_TABLE_MAX_PTS];
  float KA[GAIN_TABLE_MAX_PTS];
  float KDC[GAIN_TABLE_MAX_PTS];
  float K_phi[GAIN_TABLE_MAX_PTS];
  float Kp_roll[GAIN_TABLE_MAX_PTS];
  float Ki_roll[GAIN_TABLE_MAX_PTS];
  /* wI not scheduled -- scalar in gnc_config_t */
} gain_table_t;

/* Interpolated gains at a single velocity */
typedef struct {
  float KR, KA, KDC;
  float K_phi, Kp_roll, Ki_roll;
} gains_interp_t;

/* ---- Clda 2D table dimensions ---- */

#define CLDA_N_ALPHA_MAX 8
#define CLDA_N_MACH_MAX 8

/* ---- Aero 2D table dimensions (all 6 coefficients share breakpoints) ---- */

#define AERO_2D_N_ALPHA_MAX 10
#define AERO_2D_N_MACH_MAX 10

/* ---- GNC config (matches rocket_params.m) ---- */

typedef struct {
  /* Physical */
  float g;    /* gravity [m/s^2]              */
  float Sref; /* reference area [m^2]         */
  float d;    /* body diameter [m]            */

  /* Aero (scalar fallback, used if aero_n_alpha == 0) */
  float CNa, CNd;
  float delta_max;      /* max fin deflection [rad]     */
  float roll_alloc_max; /* roll authority cap [rad]     */

  /* Guidance */
  vec3_t target_ned; /* target NED [m]               */
  float t_guide_on;  /* guidance enable time [s]     */
  float K_guid;
  float K_guid_far;
  float K_guid_near;
  float R_gain_sched; /* gain transition range [m]    */
  float R_terminal;   /* terminal phase range [m]     */
  float K_guid_vert;  /* post-CPA vertical gain       */

  /* Control */
  float Ts;        /* control period [s]           */
  float wI;        /* accel integrator [rad/s], 0=off */
  float V_ctrl_on; /* control activation speed [m/s]  */

  /* Mode flags (flight-1/2 switching).
   * Default: CTRL_FULL, true (backward compatible). */
  ctrl_mode_t roll_mode;
  ctrl_mode_t pitch_yaw_mode;
  bool guidance_enabled;

  /* Motor */
  float m0;     /* initial mass [kg]            */
  float m_prop; /* propellant mass [kg]         */
  float t_burn; /* burn time [s]                */
  float T_avg;  /* average thrust [N]           */

  /* Gain table */
  gain_table_t gain_table;

  /* ---- Aero 2D tables: C(alpha, Mach) from CFD ----
   * Common breakpoints for all 6 coefficients.
   * Row-major: tbl[i_alpha * n_mach + i_mach].
   * Set aero_n_alpha = 0 to disable (use scalar fallback). */
  uint8_t aero_n_alpha;
  uint8_t aero_n_mach;
  float aero_alpha_bp[AERO_2D_N_ALPHA_MAX]; /* [deg] */
  float aero_mach_bp[AERO_2D_N_MACH_MAX];
  float CNa_2d[AERO_2D_N_ALPHA_MAX * AERO_2D_N_MACH_MAX];
  float CNd_2d[AERO_2D_N_ALPHA_MAX * AERO_2D_N_MACH_MAX];
  float CMa_2d[AERO_2D_N_ALPHA_MAX * AERO_2D_N_MACH_MAX];
  float CMd_2d[AERO_2D_N_ALPHA_MAX * AERO_2D_N_MACH_MAX];
  float CA0_2d[AERO_2D_N_ALPHA_MAX * AERO_2D_N_MACH_MAX];
  float Clda_2d_new[AERO_2D_N_ALPHA_MAX * AERO_2D_N_MACH_MAX];

  /* ---- Legacy Clda 2D table (backward-compatible) ----
   * Used if aero_n_alpha == 0 (old CFD data format).
   * Separate breakpoints from main aero table. */
  uint8_t clda_n_alpha;
  uint8_t clda_n_mach;
  float clda_alpha_bp[CLDA_N_ALPHA_MAX]; /* [deg]   */
  float clda_mach_bp[CLDA_N_MACH_MAX];
  float clda_2d[CLDA_N_ALPHA_MAX * CLDA_N_MACH_MAX];
  float Clda_design; /* Clda at design condition (alpha=0, design Mach) */
} gnc_config_t;

/* ---- Mission termination ---- */

typedef enum {
  MISSION_ACTIVE = 0,
  MISSION_END_CPA,
  MISSION_END_APOGEE,
  MISSION_END_GROUND,
  MISSION_END_TIMEOUT,
} mission_end_t;

/* ---- GNC aggregate state ---- */

typedef struct {
  guid_state_t guid;
  ap_state_t ap;
  guid_output_t guid_out;
  ap_output_t ap_out;
  float mission_time; /* elapsed [s] */
  bool armed;

  mission_end_t end_reason;
  float prev_v_closing;
  float prev_vD;
} gnc_state_t;

/* ---- Servo PWM output ---- */

typedef struct {
  uint16_t pulse_us[4];
} servo_output_t;

#ifdef __cplusplus
}
#endif

#endif /* GNC_TYPES_H */