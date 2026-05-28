/**
 * @file config.h
 * @brief GNC configuration — auto-generated from rocket_params.m
 *
 * Generated: 2026-05-28 18:15:03
 * Source:     rocket_params.m + build_gain_table_for_sim.m
 *
 * !! DO NOT EDIT MANUALLY !!
 * Re-generate:
 *   par = rocket_params();
 *   par = build_gain_table_for_sim(par, 'bode');
 *   generate_config_header(par);
 */

#ifndef GNC_CONFIG_H
#define GNC_CONFIG_H

#include "gnc_types.h"
#include "math_utils.h"
#include "motor_data.h"

/* ================================================================
 *  System
 * ================================================================ */

#define GNC_LOOP_RATE_HZ 400
#define GNC_TS 0.0025f /* 1/GNC_LOOP_RATE_HZ */

/* ================================================================
 *  Physical Constants
 * ================================================================ */

#define GRAVITY 9.810000f /* m/s^2 */
#define RHO_SL 1.225f     /* sea level density [kg/m^3] */
#define H_SCALE 8500.0f   /* scale height [m] */

/* ================================================================
 *  Mass & Geometry
 * ================================================================ */

#define ROCKET_M0 2.900000f     /* initial mass [kg] */
#define ROCKET_M_PROP 0.457110f /* propellant mass [kg] (from motor CSV) */
#define ROCKET_D 0.0870000000f  /* body diameter [m] */
#define ROCKET_SREF                                                            \
  (3.14159265f / 4.0f * ROCKET_D * ROCKET_D) /* ref area [m^2] */
#define ROCKET_T_AVG 311.250636f             /* average thrust [N] */
#define ROCKET_T_BURN 1.530000f /* burn time [s] (from motor CSV) */

/* ================================================================
 *  Moment of Inertia
 * ================================================================ */

#define ROCKET_IXX 0.0030000000f /* roll inertia [kg*m^2] */
#define ROCKET_IYY 0.324000f     /* pitch inertia [kg*m^2] */
#define ROCKET_IZZ 0.324000f     /* yaw inertia [kg*m^2] */

/* ================================================================
 *  Aerodynamic Coefficients
 * ================================================================ */

#define AERO_CNA 18.138857f  /* normal force slope [/rad] */
#define AERO_CND 1.453283f   /* normal force control [/rad] */
#define AERO_CMA -20.664985f /* pitch moment slope [/rad] */
#define AERO_CMD 22.704292f  /* pitch moment control [/rad] */
#define AERO_CMQ -660.0f     /* pitch damping [/rad] */
#define AERO_CA0 0.610082f   /* axial drag coeff */
#define AERO_CLP -53.400000f /* roll damping [/rad] */
#define AERO_CLDA 0.170000f  /* roll control [/rad] (scalar fallback) */

/* Aero 2D tables: C(alpha [deg], Mach) from CFD.
 * Common breakpoints, row-major: tbl[i_alpha * N_MACH + i_mach]. */
#define AERO_2D_N_ALPHA 6
#define AERO_2D_N_MACH 5
#define AERO_2D_ALPHA_BP                                                       \
  {0.000000f, 2.000000f, 4.000000f, 6.000000f, 8.000000f, 10.000000f}          \
  /* alpha breakpoints [deg] */
#define AERO_2D_MACH_BP                                                        \
  {0.100000f, 0.200000f, 0.300000f, 0.400000f, 0.500000f} /* Mach breakpoints  \
                                                           */

#define AERO_2D_CNA_TABLE                                                      \
  {17.774431f, 17.999062f, 18.138857f, 18.281453f, 18.452457f, 17.230897f,     \
   17.401446f, 17.508867f, 17.619154f, 17.754899f, 16.640444f, 16.760959f,     \
   16.844917f, 16.930723f, 17.040277f, 17.740412f, 17.892650f, 18.004983f,     \
   18.113631f, 18.219491f, 19.608681f, 20.883000f, 21.259930f, 21.408399f,     \
   19.163764f, 19.731939f, 19.295819f, 19.340200f, 19.589176f, 20.244001f}
#define AERO_2D_CND_TABLE                                                      \
  {1.407985f,  1.429108f,  1.453283f,  1.482264f,  1.512216f,  1.494955f,      \
   1.514428f,  1.534402f,  1.559287f,  1.586939f,  1.533221f,  1.559866f,      \
   1.577842f,  1.595358f,  1.608247f,  1.186166f,  1.163016f,  0.977951f,      \
   0.873030f,  0.476566f,  1.293398f,  0.850164f,  -0.080572f, -0.797424f,     \
   -0.192059f, -0.217871f, -1.214635f, -1.393626f, -1.250599f, 0.735005f}
#define AERO_2D_CMA_TABLE                                                      \
  {                                                                            \
      -19.551443f, -20.291412f, -20.664985f, -21.014742f, -21.427827f,         \
      -16.001947f, -16.486040f, -16.662769f, -16.758952f, -16.854050f,         \
      -12.214479f, -12.575776f, -12.714428f, -12.737422f, -12.733948f,         \
      -15.164191f, -15.789306f, -16.164935f, -16.409938f, -16.724953f,         \
      -24.917095f, -28.069107f, -27.815173f, -27.701591f, -33.397193f,         \
      -36.788985f, -41.571293f, -41.179524f, -40.889974f, -50.358006f}
#define AERO_2D_CMD_TABLE                                                      \
  {22.118110f, 22.539461f, 22.704292f, 22.811290f, 22.918999f, 21.984345f,     \
   22.385564f, 22.553245f, 22.678870f, 22.815995f, 22.147363f, 22.351436f,     \
   22.354181f, 22.266310f, 22.084470f, 21.866843f, 21.901938f, 20.948882f,     \
   18.466528f, 18.299489f, 27.407651f, 26.478845f, 23.668500f, 21.582164f,     \
   22.838782f, 9.591761f,  10.304495f, 7.409443f,  4.695233f,  7.339246f}
#define AERO_2D_CA0_TABLE                                                      \
  {0.640482f, 0.619070f, 0.610082f, 0.605276f, 0.603319f, 0.644883f,           \
   0.622424f, 0.612713f, 0.607460f, 0.605160f, 0.648730f, 0.624131f,           \
   0.613324f, 0.607203f, 0.604167f, 0.643866f, 0.618048f, 0.606463f,           \
   0.599636f, 0.596001f, 0.634299f, 0.608336f, 0.597912f, 0.593934f,           \
   0.595583f, 0.643573f, 0.619222f, 0.616944f, 0.619131f, 0.609870f}
#define AERO_2D_CLDA_TABLE                                                     \
  {0.250419f,  0.197054f,  0.171959f,  0.153807f,  0.138130f,  0.234529f,      \
   0.179870f,  0.151562f,  0.132005f,  0.115791f,  0.142256f,  0.082593f,      \
   0.053308f,  0.034023f,  0.021574f,  -0.263663f, -0.328181f, -0.361772f,     \
   -0.388939f, -0.348344f, -0.895606f, -0.827160f, -1.016741f, -1.153211f,     \
   -1.557593f, -1.559548f, -1.762390f, -1.768226f, -1.667149f, -1.526866f}

/* Legacy Clda 2D table (backward-compatible).
 * Row-major: CLDA_2D[i_alpha * CLDA_N_MACH + i_mach]. */
#define CLDA_N_ALPHA 6
#define CLDA_N_MACH 5
#define CLDA_ALPHA_BP                                                          \
  {0.000000f, 2.000000f, 4.000000f, 6.000000f, 8.000000f, 10.000000f}          \
  /* alpha breakpoints [deg] */
#define CLDA_MACH_BP                                                           \
  {0.100000f, 0.200000f, 0.300000f, 0.400000f, 0.500000f} /* Mach breakpoints  \
                                                           */
#define CLDA_2D_TABLE                                                          \
  {0.250419f,  0.197054f,  0.171959f,  0.153807f,  0.138130f,  0.234529f,      \
   0.179870f,  0.151562f,  0.132005f,  0.115791f,  0.142256f,  0.082593f,      \
   0.053308f,  0.034023f,  0.021574f,  -0.263663f, -0.328181f, -0.361772f,     \
   -0.388939f, -0.348344f, -0.895606f, -0.827160f, -1.016741f, -1.153211f,     \
   -1.557593f, -1.559548f, -1.762390f, -1.768226f, -1.667149f, -1.526866f}
/* Design Clda: value at alpha=0, design Mach.
 * Used as denominator in gain correction: roll_corr = Clda_design / Clda_actual
 */
#define CLDA_DESIGN 0.250419f /* Clda(alpha=0, Mach=0.1) */

/* ================================================================
 *  Actuator
 * ================================================================ */

#define DELTA_MAX_RAD 0.261799f      /* max fin deflection [rad] */
#define DELTA_RATE_MAX 5.218534f     /* max fin rate [rad/s] */
#define ROLL_ALLOC_MAX_RAD 0.174533f /* roll authority reserve [rad] */

/* Servo PWM — hardware-specific, not from rocket_params.m */
#define SERVO_PWM_MIN_US 500     /* min PWM [us] = -delta_max */
#define SERVO_PWM_CENTER_US 1500 /* center PWM [us] = 0 deg  */
#define SERVO_PWM_MAX_US 2500    /* max PWM [us] = +delta_max */
#define SERVO_PWM_FREQ_HZ 333    /* PWM frequency [Hz]       */

/* ================================================================
 *  Guidance
 * ================================================================ */

#define GUID_T_GUIDE_ON 1.0f  /* guidance enable time [s] */
#define GUID_K_PP 3.0f        /* PP gain (fallback) */
#define GUID_K_FAR 3.0f       /* augmented PP far-range gain */
#define GUID_K_NEAR 5.0f      /* augmented PP near-range gain */
#define GUID_R_SCHED 50.0f    /* gain transition distance [m] */
#define GUID_R_TERMINAL 30.0f /* terminal phase distance [m] */
#define GUID_K_VERT 1.500000f /* post-CPA vertical gain */
#define GUID_RAMP_TIME 0.5f   /* guidance ramp-up time [s] */

/* ================================================================
 *  Target (NED)
 * ================================================================ */

#define TARGET_N 100.0f  /* North [m] */
#define TARGET_E 100.0f  /* East [m] */
#define TARGET_D -500.0f /* Down [m] */

/* ================================================================
 *  Mission Termination
 * ================================================================ */

#define MISSION_T_MIN 3.0f       /* min flight time [s] */
#define MISSION_T_MAX 15.0f      /* max flight time [s] */
#define MISSION_CPA_R_MAX 100.0f /* CPA detect max range [m] */

/* ================================================================
 *  Launch Conditions
 * ================================================================ */

#define LAUNCH_V0 15.0f /* rail exit velocity [m/s] */

/* ================================================================
 *  Autopilot Design Parameters
 * ================================================================ */

#define AP_WC_RATE 25.0f       /* rate loop crossover [rad/s] */
#define AP_WC_ACCEL 10.0f      /* accel loop crossover [rad/s] */
#define AP_WI_ACCEL 1.0f       /* accel integrator freq [rad/s] */
#define AP_WC_ROLL_INNER 25.0f /* roll inner crossover [rad/s] */
#define AP_WC_ROLL_OUTER 10.0f /* roll outer crossover [rad/s] */

/* V_ctrl_on: airspeed-based control activation [m/s].            */
/*   0 = use qbar criterion (GUID_QBAR_MIN).                     */
/*   >0 = activate when airspeed > this value.                    */
#define V_CTRL_ON 30.0f /* control activation airspeed [m/s] */

/* ================================================================
 *  Gain Table
 * ================================================================ */

#define GAIN_TABLE_N_PTS 13

#define GAIN_TABLE_V_BP                                                        \
  {40.000000f,  50.000000f,  60.000000f,  70.000000f,  80.000000f,             \
   90.000000f,  100.000000f, 110.000000f, 120.000000f, 130.000000f,            \
   140.000000f, 150.000000f, 160.000000f} /* velocity breakpoints [m/s] */
#define GAIN_TABLE_KR                                                          \
  {                                                                            \
      0.768815f, 0.475574f, 0.316734f, 0.221436f, 0.160378f,                   \
      0.118655f, 0.088967f, 0.078928f, 0.091574f, 0.078067f,                   \
      0.079496f, 0.073033f, 0.060897f} /* rate gain */
#define GAIN_TABLE_KA                                                          \
  {                                                                            \
      0.298416f, 0.177449f, 0.121793f, 0.091932f, 0.074714f,                   \
      0.064591f, 0.058986f, 0.052679f, 0.042794f, 0.033580f,                   \
      0.029102f, 0.027473f, 0.028133f} /* accel gain */
#define GAIN_TABLE_KDC                                                         \
  {                                                                            \
      1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f,                   \
      1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f,                   \
      1.000000f, 1.000000f, 1.000000f} /* DC gain */
#define GAIN_TABLE_K_PHI                                                       \
  {12.609388f, 13.298733f, 13.917950f, 14.467734f, 14.951798f,                 \
   15.375642f, 15.745575f, 16.068028f, 16.349137f, 16.594504f,                 \
   16.809111f, 16.997301f, 17.162820f} /* roll outer gain */
#define GAIN_TABLE_KP_ROLL                                                     \
  {                                                                            \
      0.734991f, 0.520582f, 0.403905f, 0.331813f, 0.276058f,                   \
      0.237755f, 0.210442f, 0.188612f, 0.171598f, 0.158403f,                   \
      0.147758f, 0.138854f, 0.131652f} /* roll proportional */
#define GAIN_TABLE_KI_ROLL                                                     \
  {                                                                            \
      0.734991f, 0.520582f, 0.403905f, 0.331813f, 0.276058f,                   \
      0.237755f, 0.210442f, 0.188612f, 0.171598f, 0.158403f,                   \
      0.147758f, 0.138854f, 0.131652f} /* roll integral */

/* wI: accel integrator crossover [rad/s]. Scalar, not scheduled. */
#define ACCEL_WI 1.0f /* accel integrator freq [rad/s] */

/* ================================================================
 *  Motor Selection
 * ================================================================ */

#define DEFAULT_MOTOR_SELECT MOTOR_SELECT_SHORT

/* ================================================================
 *  Config Initialization
 * ================================================================ */

/**
 * @brief Create config with specified motor.
 */
static inline gnc_config_t gnc_config_with_motor(motor_select_t motor_sel) {
  gnc_config_t cfg = {0};

  cfg.g = GRAVITY;
  cfg.Sref = ROCKET_SREF;
  cfg.d = ROCKET_D;
  cfg.CNa = AERO_CNA;
  cfg.CNd = AERO_CND;
  cfg.delta_max = DELTA_MAX_RAD;
  cfg.roll_alloc_max = ROLL_ALLOC_MAX_RAD;
  cfg.V_ctrl_on = V_CTRL_ON;

  cfg.target_ned = (vec3_t){TARGET_N, TARGET_E, TARGET_D};
  cfg.t_guide_on = GUID_T_GUIDE_ON;
  cfg.K_guid = GUID_K_PP;
  cfg.K_guid_far = GUID_K_FAR;
  cfg.K_guid_near = GUID_K_NEAR;
  cfg.R_gain_sched = GUID_R_SCHED;
  cfg.R_terminal = GUID_R_TERMINAL;
  cfg.K_guid_vert = GUID_K_VERT;

  cfg.Ts = GNC_TS;
  cfg.wI = ACCEL_WI;

  /* Motor profile */
  cfg.m0 = ROCKET_M0;
  if (motor_sel == MOTOR_SELECT_SHORT) {
    cfg.m_prop = MOTOR_SHORT_M_PROP;
    cfg.t_burn = MOTOR_SHORT_T_BURN;
    cfg.T_avg = MOTOR_SHORT_T_AVG;
  } else {
    cfg.m_prop = MOTOR_LONG_M_PROP;
    cfg.t_burn = MOTOR_LONG_T_BURN;
    cfg.T_avg = MOTOR_LONG_T_AVG;
  }

  /* Gain table */
  cfg.gain_table.n_pts = GAIN_TABLE_N_PTS;
  float v_bp[] = GAIN_TABLE_V_BP;
  float kr[] = GAIN_TABLE_KR;
  float ka[] = GAIN_TABLE_KA;
  float kdc[] = GAIN_TABLE_KDC;
  float kphi[] = GAIN_TABLE_K_PHI;
  float kpr[] = GAIN_TABLE_KP_ROLL;
  float kir[] = GAIN_TABLE_KI_ROLL;

  for (int i = 0; i < GAIN_TABLE_N_PTS; i++) {
    cfg.gain_table.V_bp[i] = v_bp[i];
    cfg.gain_table.KR[i] = kr[i];
    cfg.gain_table.KA[i] = ka[i];
    cfg.gain_table.KDC[i] = kdc[i];
    cfg.gain_table.K_phi[i] = kphi[i];
    cfg.gain_table.Kp_roll[i] = kpr[i];
    cfg.gain_table.Ki_roll[i] = kir[i];
  }

  /* Clda 2D table */
  cfg.clda_n_alpha = CLDA_N_ALPHA;
  cfg.clda_n_mach = CLDA_N_MACH;
  float clda_abp[] = CLDA_ALPHA_BP;
  float clda_mbp[] = CLDA_MACH_BP;
  float clda_2d[] = CLDA_2D_TABLE;
  for (int i = 0; i < CLDA_N_ALPHA; i++)
    cfg.clda_alpha_bp[i] = clda_abp[i];
  for (int i = 0; i < CLDA_N_MACH; i++)
    cfg.clda_mach_bp[i] = clda_mbp[i];
  for (int i = 0; i < CLDA_N_ALPHA * CLDA_N_MACH; i++)
    cfg.clda_2d[i] = clda_2d[i];
  cfg.Clda_design = CLDA_DESIGN;

  /* Aero 2D tables (all 6 coefficients) */
  cfg.aero_n_alpha = AERO_2D_N_ALPHA;
  cfg.aero_n_mach = AERO_2D_N_MACH;
  {
    float a_abp[] = AERO_2D_ALPHA_BP;
    float a_mbp[] = AERO_2D_MACH_BP;
    for (int i = 0; i < AERO_2D_N_ALPHA; i++)
      cfg.aero_alpha_bp[i] = a_abp[i];
    for (int i = 0; i < AERO_2D_N_MACH; i++)
      cfg.aero_mach_bp[i] = a_mbp[i];
    {
      float t[] = AERO_2D_CNA_TABLE;
      for (int i = 0; i < AERO_2D_N_ALPHA * AERO_2D_N_MACH; i++)
        cfg.CNa_2d[i] = t[i];
    }
    {
      float t[] = AERO_2D_CND_TABLE;
      for (int i = 0; i < AERO_2D_N_ALPHA * AERO_2D_N_MACH; i++)
        cfg.CNd_2d[i] = t[i];
    }
    {
      float t[] = AERO_2D_CMA_TABLE;
      for (int i = 0; i < AERO_2D_N_ALPHA * AERO_2D_N_MACH; i++)
        cfg.CMa_2d[i] = t[i];
    }
    {
      float t[] = AERO_2D_CMD_TABLE;
      for (int i = 0; i < AERO_2D_N_ALPHA * AERO_2D_N_MACH; i++)
        cfg.CMd_2d[i] = t[i];
    }
    {
      float t[] = AERO_2D_CA0_TABLE;
      for (int i = 0; i < AERO_2D_N_ALPHA * AERO_2D_N_MACH; i++)
        cfg.CA0_2d[i] = t[i];
    }
    {
      float t[] = AERO_2D_CLDA_TABLE;
      for (int i = 0; i < AERO_2D_N_ALPHA * AERO_2D_N_MACH; i++)
        cfg.Clda_2d_new[i] = t[i];
    }
  }

  /* Control modes: CTRL_OFF=0, CTRL_RATE_DAMP=1, CTRL_FULL=2 */
  cfg.roll_mode = CTRL_FULL;
  cfg.pitch_yaw_mode = CTRL_FULL;
  cfg.guidance_enabled = true;

  return cfg;
}

/**
 * @brief Default config (uses DEFAULT_MOTOR_SELECT).
 */
static inline gnc_config_t gnc_config_default(void) {
  return gnc_config_with_motor(DEFAULT_MOTOR_SELECT);
}

/**
 * @brief Flight-1 config: roll damper only, no guidance.
 * For launch ejection + roll control verification.
 */
static inline gnc_config_t gnc_config_flight1(void) {
  gnc_config_t cfg = gnc_config_default();
  cfg.roll_mode = CTRL_FULL;
  cfg.pitch_yaw_mode = CTRL_FULL;
  cfg.guidance_enabled = false;
  return cfg;
}

#endif /* GNC_CONFIG_H */
