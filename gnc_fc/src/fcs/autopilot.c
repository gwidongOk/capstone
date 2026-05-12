/**
 * @file autopilot.c
 * Discrete-time 3-loop cascade autopilot + roll damper + fin mixing.
 * Integrators use Tustin (bilinear) discretization.
 */

#include "autopilot.h"
#include "math_utils.h"
#include "gain_table.h"
#include "gnc_config.h"

void autopilot_init(ap_state_t *state)
{
    state->roll_int    = 0.0f;
    state->roll_e_prev = 0.0f;
    state->pitch_int   = 0.0f;
    state->pitch_e_prev= 0.0f;
    state->yaw_int     = 0.0f;
    state->yaw_e_prev  = 0.0f;
    state->initialized = true;
}

ap_output_t autopilot_step(float nz_cmd, float ny_cmd,
                           const nav_state_t *nav,
                           const gnc_config_t *cfg,
                           ap_state_t *state)
{
    ap_output_t out = {0};
    const float Ts = cfg->Ts;
    const float delta_max = cfg->delta_max;

    float p = nav->omega_b.x;
    float q_rate = nav->omega_b.y;
    float r = nav->omega_b.z;

    float V = nav->airspeed;
    if (V < 1.0f) V = 1.0f;
    float alt = -nav->pos_ned.z;
    if (alt < 0.0f) alt = 0.0f;
    float rho = RHO_SL * expf(-alt / H_SCALE);
    float qbar = 0.5f * rho * V * V;

    /* Body-x velocity for nz_fb: nz ~ q*u/g, use R_BN*(vel-wind).x */
    quat_t q_norm = quat_normalize(nav->quat);

    float R_BN[9];
    quat_to_dcm_bn(q_norm, R_BN);
    vec3_t vel_air = vec3_sub(nav->vel_ned, nav->wind_est);
    vec3_t vb = dcm_bn_mul_vec(R_BN, vel_air);
    float u_body = vb.x;
    if (u_body < 1.0f) u_body = 1.0f;

    bool ctrl_active;
    if (cfg->V_ctrl_on > 0.0f) {
        ctrl_active = (V > cfg->V_ctrl_on);
    } else {
        ctrl_active = (qbar > cfg->qbar_min_ctrl);
    }

    gains_interp_t g = gain_table_lookup(&cfg->gain_table, V);

    /* Pitch / Yaw: 3-loop cascade */
    float de_v, dr_v;

    if (ctrl_active) {
        float nz_fb =  (q_rate * u_body) / cfg->g;
        float ny_fb = -(r * u_body) / cfg->g;

        float nz_err = g.KDC * nz_cmd - nz_fb;
        float ny_err = g.KDC * ny_cmd - ny_fb;

        /* Tustin integrator */
        if (cfg->wI > 0.0001f) {
            state->pitch_int += (Ts * 0.5f) * (nz_err + state->pitch_e_prev);
            state->yaw_int   += (Ts * 0.5f) * (ny_err + state->yaw_e_prev);
        }

        de_v = g.KR * (g.KA * nz_err + cfg->wI * state->pitch_int - q_rate);
        dr_v = g.KR * (g.KA * ny_err + cfg->wI * state->yaw_int   - r);

        /* Anti-windup (clamping) */
        if (cfg->wI > 0.0001f) {
            if (fabsf(de_v) > delta_max && signf(nz_err) == signf(de_v)) {
                state->pitch_int -= (Ts * 0.5f) * (nz_err + state->pitch_e_prev);
            }
            if (fabsf(dr_v) > delta_max && signf(ny_err) == signf(dr_v)) {
                state->yaw_int -= (Ts * 0.5f) * (ny_err + state->yaw_e_prev);
            }
        }

        state->pitch_e_prev = nz_err;
        state->yaw_e_prev   = ny_err;
    } else {
        /* Rate damping only */
        de_v = -g.KR * q_rate;
        dr_v = -g.KR * r;
    }

    /* Roll control */
    float phi, theta;
    {
        float q0 = q_norm.q0, q1 = q_norm.q1;
        float q2 = q_norm.q2, q3 = q_norm.q3;
        theta = asinf(clampf(2.0f*(q0*q2 - q3*q1), -1.0f, 1.0f));
        phi   = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2));
    }

    float abs_theta = fabsf(theta);
    float p_cmd;

    if (abs_theta < 70.0f * DEG2RAD_F) {
        float e_phi = wrap_pi(-phi);
        p_cmd = g.K_phi * e_phi;
    } else if (abs_theta > 80.0f * DEG2RAD_F) {
        p_cmd = 0.0f;
    } else {
        float e_phi = wrap_pi(-phi);
        float blend = (abs_theta - 70.0f * DEG2RAD_F) / (10.0f * DEG2RAD_F);
        p_cmd = g.K_phi * e_phi * (1.0f - blend);
    }

    float e_p = p_cmd - p;

    /* Tustin PI */
    state->roll_int += (Ts * 0.5f) * (e_p + state->roll_e_prev);
    float da_v_raw = g.Kp_roll * e_p + g.Ki_roll * state->roll_int;
    float da_v = clampf(da_v_raw, -delta_max, delta_max);

    /* Anti-windup */
    if (fabsf(da_v_raw) > delta_max && signf(e_p) == signf(da_v_raw)) {
        state->roll_int -= (Ts * 0.5f) * (e_p + state->roll_e_prev);
    }
    state->roll_e_prev = e_p;

    /* Fin mixing: + layout, roll-priority allocation */
    float roll_lim = cfg->roll_alloc_max;
    if (roll_lim <= 0.0f || roll_lim > delta_max) roll_lim = delta_max;
    float da_alloc = clampf(da_v, -roll_lim, roll_lim);
    float remaining = delta_max - fabsf(da_alloc);
    float de_alloc = clampf(de_v, -remaining, remaining);
    float dr_alloc = clampf(dr_v, -remaining, remaining);

    out.fin_cmd[0] = clampf( de_alloc + da_alloc, -delta_max, delta_max);
    out.fin_cmd[1] = clampf( dr_alloc + da_alloc, -delta_max, delta_max);
    out.fin_cmd[2] = clampf(-de_alloc + da_alloc, -delta_max, delta_max);
    out.fin_cmd[3] = clampf(-dr_alloc + da_alloc, -delta_max, delta_max);

    out.de_cmd = de_alloc;
    out.dr_cmd = dr_alloc;
    out.da_cmd = da_alloc;

    return out;
}
