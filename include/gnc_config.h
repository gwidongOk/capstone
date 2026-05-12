/* Auto-generated from rocket_params.m — DO NOT EDIT MANUALLY
 * Generated: 2026-05-02 23:11:04
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

/* System */
#define GNC_LOOP_RATE_HZ    400
#define GNC_TS              0.0025f

/* Physical constants */
#define GRAVITY             9.810000f
#define RHO_SL              1.225f
#define H_SCALE             8500.0f

/* Mass & geometry */
#define ROCKET_M0                     2.500000f
#define ROCKET_M_PROP                 0.457110f
#define ROCKET_D                      0.100000f
#define ROCKET_SREF                 (3.14159265f/4.0f * ROCKET_D * ROCKET_D)
#define ROCKET_T_AVG                311.250636f
#define ROCKET_T_BURN                 1.530000f

/* Moment of inertia */
#define ROCKET_IXX                  0.0026000000f
#define ROCKET_IYY                    0.110600f
#define ROCKET_IZZ                    0.110600f

/* Aerodynamic coefficients */
#define AERO_CNA                      8.899800f
#define AERO_CND                      1.500000f
#define AERO_CMA                     -4.400000f
#define AERO_CMD                      2.500000f
#define AERO_CMQ                       -50.0f
#define AERO_CA0                      0.420000f
#define AERO_CLP                     -0.500000f
#define AERO_CLDA                     0.120000f

/* Actuator */
#define DELTA_MAX_RAD                 0.261799f
#define DELTA_RATE_MAX                6.283185f
#define ROLL_ALLOC_MAX_RAD            0.174533f

/* Servo PWM */
#define SERVO_PWM_MIN_US    500
#define SERVO_PWM_CENTER_US 1500
#define SERVO_PWM_MAX_US    2500
#define SERVO_PWM_FREQ_HZ   333

/* Guidance */
#define GUID_T_GUIDE_ON                  1.0f
#define GUID_QBAR_MIN                   50.0f
#define GUID_K_PP                        3.0f
#define GUID_K_FAR                    1.500000f
#define GUID_K_NEAR                      3.0f
#define GUID_R_SCHED                   150.0f
#define GUID_R_TERMINAL                 50.0f
#define GUID_K_VERT                   1.500000f
#define GUID_RAMP_TIME      0.5f

/* Target (NED) */
#define TARGET_N            100.0f
#define TARGET_E            100.0f
#define TARGET_D            -500.0f

/* Mission termination */
#define MISSION_T_MIN       3.0f
#define MISSION_T_MAX       15.0f
#define MISSION_CPA_R_MAX   100.0f

/* Launch conditions */
#define LAUNCH_V0                       15.0f

/* Autopilot design parameters */
#define AP_WC_RATE                      25.0f
#define AP_WC_ACCEL                     10.0f
#define AP_WI_ACCEL                      1.0f
#define AP_WC_ROLL_INNER                25.0f
#define AP_WC_ROLL_OUTER                10.0f

/* V_ctrl_on: 0 = use qbar criterion, >0 = airspeed threshold [m/s] */
#define V_CTRL_ON           0.0f

/* Gain table */
#define GAIN_TABLE_N_PTS    15

#define GAIN_TABLE_V_BP         { 40.000000f, 50.000000f, 60.000000f, 70.000000f, 80.000000f, 90.000000f, 100.000000f, 110.000000f, 120.000000f, 130.000000f, 140.000000f, 150.000000f, 160.000000f, 170.000000f, 180.000000f }
#define GAIN_TABLE_KR           { 1.434345f, 0.892373f, 0.598034f, 0.420630f, 0.305569f, 0.226777f, 0.170526f, 0.129037f, 0.097645f, 0.073431f, 0.054522f, 0.039732f, 0.028411f, 0.020457f, 0.016325f }
#define GAIN_TABLE_KA           { 1.043264f, 0.306639f, 0.168678f, 0.113909f, 0.086495f, 0.071505f, 0.063352f, 0.059576f, 0.059061f, 0.061445f, 0.066942f, 0.076331f, 0.090744f, 0.109464f, 0.121957f }
#define GAIN_TABLE_KDC          { 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f }
#define GAIN_TABLE_K_PHI        { 9.152149f, 9.169450f, 9.186750f, 9.204050f, 9.221349f, 9.238648f, 9.255945f, 9.273241f, 9.290536f, 9.307829f, 9.325121f, 9.342411f, 9.359698f, 9.376984f, 9.394267f }
#define GAIN_TABLE_KP_ROLL      { 0.737454f, 0.471977f, 0.327768f, 0.240814f, 0.184378f, 0.145685f, 0.118008f, 0.097531f, 0.081956f, 0.069835f, 0.060218f, 0.052459f, 0.046108f, 0.040846f, 0.036435f }
#define GAIN_TABLE_KI_ROLL      { 1.474907f, 0.943954f, 0.655535f, 0.481628f, 0.368755f, 0.291370f, 0.236017f, 0.195062f, 0.163912f, 0.139670f, 0.120435f, 0.104917f, 0.092217f, 0.081691f, 0.072871f }

#define ACCEL_WI                         1.0f

/* Motor selection */
#define DEFAULT_MOTOR_SELECT  MOTOR_SELECT_SHORT

/* Config initialization */
static inline gnc_config_t gnc_config_with_motor(motor_select_t motor_sel)
{
    gnc_config_t cfg = {0};

    cfg.g             = GRAVITY;
    cfg.Sref          = ROCKET_SREF;
    cfg.d             = ROCKET_D;
    cfg.CNa           = AERO_CNA;
    cfg.CNd           = AERO_CND;
    cfg.delta_max     = DELTA_MAX_RAD;
    cfg.roll_alloc_max = ROLL_ALLOC_MAX_RAD;
    cfg.V_ctrl_on     = V_CTRL_ON;

    cfg.target_ned    = (vec3_t){TARGET_N, TARGET_E, TARGET_D};
    cfg.t_guide_on    = GUID_T_GUIDE_ON;
    cfg.qbar_min_ctrl = GUID_QBAR_MIN;
    cfg.K_guid        = GUID_K_PP;
    cfg.K_guid_far    = GUID_K_FAR;
    cfg.K_guid_near   = GUID_K_NEAR;
    cfg.R_gain_sched  = GUID_R_SCHED;
    cfg.R_terminal    = GUID_R_TERMINAL;
    cfg.K_guid_vert   = GUID_K_VERT;

    cfg.Ts            = GNC_TS;
    cfg.wI            = ACCEL_WI;

    cfg.m0 = ROCKET_M0;
    if (motor_sel == MOTOR_SELECT_SHORT) {
        cfg.m_prop = MOTOR_SHORT_M_PROP;
        cfg.t_burn = MOTOR_SHORT_T_BURN;
        cfg.T_avg  = MOTOR_SHORT_T_AVG;
    } else {
        cfg.m_prop = MOTOR_LONG_M_PROP;
        cfg.t_burn = MOTOR_LONG_T_BURN;
        cfg.T_avg  = MOTOR_LONG_T_AVG;
    }

    cfg.gain_table.n_pts = GAIN_TABLE_N_PTS;
    float v_bp[]   = GAIN_TABLE_V_BP;
    float kr[]     = GAIN_TABLE_KR;
    float ka[]     = GAIN_TABLE_KA;
    float kdc[]    = GAIN_TABLE_KDC;
    float kphi[]   = GAIN_TABLE_K_PHI;
    float kpr[]    = GAIN_TABLE_KP_ROLL;
    float kir[]    = GAIN_TABLE_KI_ROLL;

    for (int i = 0; i < GAIN_TABLE_N_PTS; i++) {
        cfg.gain_table.V_bp[i]    = v_bp[i];
        cfg.gain_table.KR[i]      = kr[i];
        cfg.gain_table.KA[i]      = ka[i];
        cfg.gain_table.KDC[i]     = kdc[i];
        cfg.gain_table.K_phi[i]   = kphi[i];
        cfg.gain_table.Kp_roll[i] = kpr[i];
        cfg.gain_table.Ki_roll[i] = kir[i];
    }

    return cfg;
}

static inline gnc_config_t gnc_config_default(void)
{
    return gnc_config_with_motor(DEFAULT_MOTOR_SELECT);
}

#endif /* GNC_CONFIG_H */
