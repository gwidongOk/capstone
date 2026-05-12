#ifndef GNC_TYPES_H
#define GNC_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* --- Basic types --- */

typedef struct {
    float x, y, z;
} vec3_t;

typedef struct {
    float q0, q1, q2, q3;  /* scalar-first */
} quat_t;

/* --- Navigation state (input to GNC) --- */

typedef struct {
    vec3_t pos_ned;
    vec3_t vel_ned;
    quat_t quat;            /* body <- NED */
    vec3_t omega_b;          /* body angular rate [rad/s] */
    vec3_t accel_b;
    vec3_t wind_est;         /* estimated wind NED [m/s] */
    float  airspeed;
    uint32_t timestamp_ms;
    bool valid;
} nav_state_t;

/* --- Guidance output --- */

typedef struct {
    float nz_cmd;            /* pitch accel cmd [g] */
    float ny_cmd;            /* yaw accel cmd [g] */
    float range_to_tgt;
    float v_closing;
    bool  past_cpa;
    bool  active;
} guid_output_t;

/* --- Guidance state --- */

typedef struct {
    float los_rate_filt[3];
    float ramp_timer;
    bool  initialized;
} guid_state_t;

/* --- Autopilot output --- */

typedef struct {
    float fin_cmd[4];        /* fin commands [rad] */
    float de_cmd;            /* pitch [rad] (diag) */
    float dr_cmd;            /* yaw [rad] (diag) */
    float da_cmd;            /* roll [rad] (diag) */
} ap_output_t;

/* --- Autopilot state --- */

typedef struct {
    float roll_int;
    float roll_e_prev;
    float pitch_int;
    float pitch_e_prev;
    float yaw_int;
    float yaw_e_prev;
    bool initialized;
} ap_state_t;

/* --- Gain table --- */

#define GAIN_TABLE_MAX_PTS  20

typedef struct {
    uint8_t n_pts;
    float V_bp[GAIN_TABLE_MAX_PTS];
    float KR[GAIN_TABLE_MAX_PTS];
    float KA[GAIN_TABLE_MAX_PTS];
    float KDC[GAIN_TABLE_MAX_PTS];
    float K_phi[GAIN_TABLE_MAX_PTS];
    float Kp_roll[GAIN_TABLE_MAX_PTS];
    float Ki_roll[GAIN_TABLE_MAX_PTS];
} gain_table_t;

typedef struct {
    float KR, KA, KDC;
    float K_phi, Kp_roll, Ki_roll;
} gains_interp_t;

/* --- GNC config --- */

typedef struct {
    float g;
    float Sref;
    float d;
    float CNa, CNd;
    float delta_max;
    float roll_alloc_max;

    vec3_t target_ned;
    float t_guide_on;
    float qbar_min_ctrl;
    float K_guid;
    float K_guid_far;
    float K_guid_near;
    float R_gain_sched;
    float R_terminal;
    float K_guid_vert;

    float Ts;
    float wI;
    float V_ctrl_on;

    float m0;
    float m_prop;
    float t_burn;
    float T_avg;

    gain_table_t gain_table;
} gnc_config_t;

/* --- GNC state --- */

typedef enum {
    MISSION_ACTIVE = 0,
    MISSION_END_CPA,
    MISSION_END_APOGEE,
    MISSION_END_GROUND,
    MISSION_END_TIMEOUT,
} mission_end_t;

typedef struct {
    guid_state_t  guid;
    ap_state_t    ap;
    guid_output_t guid_out;
    ap_output_t   ap_out;
    float mission_time;
    bool  armed;
    mission_end_t end_reason;
    float prev_v_closing;
    float prev_vD;
} gnc_state_t;

/* --- Servo output --- */

typedef struct {
    uint16_t pulse_us[4];
} servo_output_t;

#endif /* GNC_TYPES_H */
