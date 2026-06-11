/**
 * @file autopilot.c
 * @brief Discrete 3-loop cascade autopilot + roll damper + fin mixing.
 *
 * Pitch/Yaw: nz_cmd -> KDC -> accel_err -> KA + wI*int -> rate_err -> KR ->
 * de/dr Roll:      phi_cmd=0 -> K_phi -> p_cmd -> PI -> da Fin Mix:   roll
 * priority -> pitch/yaw alloc -> per-fin saturation
 *
 * Mode flags (cfg->roll_mode, cfg->pitch_yaw_mode) select control depth:
 *   CTRL_OFF       : zero deflection
 *   CTRL_RATE_DAMP : rate damping only
 *   CTRL_FULL      : full loop
 */

#include "autopilot.h"
#include "gain_table.h"
#include "gnc_config.h"
#include "math_utils.h"
#include <stdio.h>

/* ---- Init ---- */

void autopilot_init(ap_state_t *state) {
  state->roll_int = 0.0f;
  state->roll_e_prev = 0.0f;
  state->roll_ramp = 0.0f;
  state->pitch_int = 0.0f;
  state->pitch_e_prev = 0.0f;
  state->yaw_int = 0.0f;
  state->yaw_e_prev = 0.0f;
  state->initialized = true;
}

/* ---- Main step ---- */

ap_output_t autopilot_step(float nz_cmd, float ny_cmd, const nav_state_t *nav,
                           const gnc_config_t *cfg, ap_state_t *state) {
  ap_output_t out = {0};
  const float Ts = cfg->Ts;
  const float delta_max = cfg->delta_max;

  float p = nav->omega_b.x;
  float q_rate = nav->omega_b.y;
  float r = nav->omega_b.z;

  /* Dynamic pressure */
  float V = nav->airspeed;
  if (V < 1.0f)
    V = 1.0f;
  float alt = -nav->pos_ned.z;
  if (alt < 0.0f)
    alt = 0.0f;
  float rho = RHO_SL * expf(-alt / H_SCALE);
  float qbar = 0.5f * rho * V * V;

  /* Body-x velocity for nz feedback: nz_fb = q*u/g */
  quat_t q_norm = quat_normalize(nav->quat);
  float R_BN[9];
  quat_to_dcm_bn(q_norm, R_BN);
  vec3_t vel_air = vec3_sub(nav->vel_ned, nav->wind_est);
  vec3_t vb = dcm_bn_mul_vec(R_BN, vel_air);
  float u_body = vb.x;
  if (u_body < 1.0f)
    u_body = 1.0f;

  /* Control activation (airspeed threshold) */
  bool ctrl_active = (V > cfg->V_ctrl_on);

  /* Gain lookup */
  gains_interp_t g = gain_table_lookup(&cfg->gain_table, V);

  /* ---- Pitch/Yaw alpha correction ----
   * Gains designed at alpha=0; correct for actual alpha:
   *   KR  *= CMd(0,M) / CMd(a,M)
   *   KA  *= CNd(0,M) / CNd(a,M)
   *   wI  *= CNd(0,M) / CNd(a,M)
   *   KDC *= [CNa(0,M)/CNa(a,M)] / [CNd(0,M)/CNd(a,M)]  */
  float Mach_ap = V / 340.3f;
  float u_safe = fmaxf(u_body, 1.0f);

  float alpha_body = atan2f(vb.z, u_safe);
  float alpha_deg_ap = fabsf(alpha_body * RAD2DEG_F);

  float v_yz_norm = sqrtf(vb.y * vb.y + vb.z * vb.z);
  float alpha_total = atan2f(v_yz_norm, u_safe);
  float alpha_deg_roll = fabsf(alpha_total * RAD2DEG_F);

  float wI_eff = cfg->wI;

  if (cfg->aero_n_alpha > 0 && alpha_deg_ap > 0.5f) {
    float CMd_0 = interp2_bilinear(cfg->aero_alpha_bp, cfg->aero_n_alpha,
                                   cfg->aero_mach_bp, cfg->aero_n_mach,
                                   cfg->CMd_2d, 0.0f, Mach_ap);
    float CMd_a = interp2_bilinear(cfg->aero_alpha_bp, cfg->aero_n_alpha,
                                   cfg->aero_mach_bp, cfg->aero_n_mach,
                                   cfg->CMd_2d, alpha_deg_ap, Mach_ap);
    float CNd_0 = interp2_bilinear(cfg->aero_alpha_bp, cfg->aero_n_alpha,
                                   cfg->aero_mach_bp, cfg->aero_n_mach,
                                   cfg->CNd_2d, 0.0f, Mach_ap);
    float CNd_a = interp2_bilinear(cfg->aero_alpha_bp, cfg->aero_n_alpha,
                                   cfg->aero_mach_bp, cfg->aero_n_mach,
                                   cfg->CNd_2d, alpha_deg_ap, Mach_ap);
    float CNa_0 = interp2_bilinear(cfg->aero_alpha_bp, cfg->aero_n_alpha,
                                   cfg->aero_mach_bp, cfg->aero_n_mach,
                                   cfg->CNa_2d, 0.0f, Mach_ap);
    float CNa_a = interp2_bilinear(cfg->aero_alpha_bp, cfg->aero_n_alpha,
                                   cfg->aero_mach_bp, cfg->aero_n_mach,
                                   cfg->CNa_2d, alpha_deg_ap, Mach_ap);

    float rc_CMd = 1.0f, rc_CNd = 1.0f, rc_CNa = 1.0f;
    if (fabsf(CMd_a) > 0.1f)
      rc_CMd = clampf(CMd_0 / CMd_a, 0.3f, 3.0f);
    if (fabsf(CNd_a) > 0.01f)
      rc_CNd = clampf(CNd_0 / CNd_a, 0.3f, 3.0f);
    if (fabsf(CNa_a) > 0.1f)
      rc_CNa = clampf(CNa_0 / CNa_a, 0.3f, 3.0f);

    g.KR *= rc_CMd;
    g.KA *= rc_CNd;
    wI_eff *= rc_CNd;
    g.KDC *= (rc_CNa / rc_CNd);
  }

  /* ---- Pitch / Yaw ---- */
  float de_v, dr_v;

  if (cfg->pitch_yaw_mode == CTRL_OFF) {
    de_v = 0.0f;
    dr_v = 0.0f;

  } else if (cfg->pitch_yaw_mode == CTRL_RATE_DAMP) {
    de_v = -g.KR * q_rate;
    dr_v = -g.KR * r;

  } else {
    /* CTRL_FULL: 3-loop cascade */
    if (ctrl_active) {
      float nz_fb = (q_rate * u_body) / cfg->g;
      float ny_fb = (r * u_body) / cfg->g;

      float nz_err = g.KDC * nz_cmd - nz_fb;
      float ny_err = g.KDC * ny_cmd - ny_fb;

      /* Tustin integrator */
      if (wI_eff > 0.0001f) {
        state->pitch_int += (Ts * 0.5f) * (nz_err + state->pitch_e_prev);
        state->yaw_int += (Ts * 0.5f) * (ny_err + state->yaw_e_prev);
      }

      de_v = g.KR * (g.KA * nz_err + wI_eff * state->pitch_int - q_rate);
      dr_v = g.KR * (g.KA * ny_err + wI_eff * state->yaw_int - r);

      /* Anti-windup (clamping) */
      if (wI_eff > 0.0001f) {
        if (fabsf(de_v) > delta_max && signf(nz_err) == signf(de_v)) {
          state->pitch_int -= (Ts * 0.5f) * (nz_err + state->pitch_e_prev);
        }
        if (fabsf(dr_v) > delta_max && signf(ny_err) == signf(dr_v)) {
          state->yaw_int -= (Ts * 0.5f) * (ny_err + state->yaw_e_prev);
        }
      }

      state->pitch_e_prev = nz_err;
      state->yaw_e_prev = ny_err;
    } else {
      /* Low speed (V < V_ctrl_on): zero fins, no authority */
      de_v = 0.0f;
      dr_v = 0.0f;
    }
  }

  /* ---- Roll ---- */
  float da_v = 0.0f;

  if (cfg->roll_mode == CTRL_OFF) {
    da_v = 0.0f;

  } else if (cfg->roll_mode == CTRL_RATE_DAMP) {
    /* P-only rate damping, reuses Kp_roll */
    da_v = clampf(-g.Kp_roll * p, -delta_max, delta_max);

  } else {
    /* CTRL_FULL: gravity-referenced roll angle tracking + Mach guard */
    /*   phi_grav = atan2(gb_y, gb_z)
     *   where gb = R_BN * [0,0,1] = 3rd column of R_BN.
     *   atan2(sin(phi)*cos(theta), cos(phi)*cos(theta)) = phi
     *   cos(theta) cancels — well-defined at any pitch
     *   except exactly theta = +/-90 deg.
     *   R_BN[9] row-major already computed above.                    */
    float Mach = V / 340.3f;
    if (Mach > 1.0f || !ctrl_active) {
      /* Aileron reversal zone or low speed: disable roll */
      da_v = 0.0f;
      /* Reset for bumpless transfer on re-activation */
      state->roll_int = 0.0f;
      state->roll_e_prev = 0.0f;
      state->roll_ramp = 0.0f;
    } else {
      /* Ramp-in gain over 0.5s after Mach guard release */
      const float roll_ramp_time = 0.5f;
      state->roll_ramp += Ts;
      if (state->roll_ramp > roll_ramp_time)
        state->roll_ramp = roll_ramp_time;
      float ramp_gain = state->roll_ramp / roll_ramp_time;

      /* ---- Clda(alpha, Mach) gain correction ----
       * Lookup Clda_actual and Clda_design(alpha=0) from 2D table.
       * roll_corr = Clda_design / Clda_actual.
       * Uses new aero tables if available, else legacy Clda table. */
      float roll_corr = 1.0f;

      if (cfg->aero_n_alpha > 0) {
        /* New unified aero table */
        float Clda_actual = interp2_bilinear(
            cfg->aero_alpha_bp, cfg->aero_n_alpha, cfg->aero_mach_bp,
            cfg->aero_n_mach, cfg->Clda_2d_new, alpha_deg_roll, Mach);
        float Clda_des = interp2_bilinear(cfg->aero_alpha_bp, cfg->aero_n_alpha,
                                          cfg->aero_mach_bp, cfg->aero_n_mach,
                                          cfg->Clda_2d_new, 0.0f, Mach);
        if (fabsf(Clda_actual) > 0.005f) {
          roll_corr = clampf(Clda_des / Clda_actual, 0.1f, 3.0f);
        }
      } else if (cfg->clda_n_alpha > 0) {
        /* Legacy Clda-only table */
        float alpha_max = cfg->clda_alpha_bp[cfg->clda_n_alpha - 1];
        if (alpha_deg_ap <= alpha_max) {
          float Clda_actual = interp2_bilinear(
              cfg->clda_alpha_bp, cfg->clda_n_alpha, cfg->clda_mach_bp,
              cfg->clda_n_mach, cfg->clda_2d, alpha_deg_ap, Mach);
          if (fabsf(Clda_actual) > 0.005f) {
            roll_corr = clampf(cfg->Clda_design / Clda_actual, 0.1f, 3.0f);
          }
        }
      }

      /* Gravity-referenced roll angle */
      float gb_y = R_BN[5]; /* R_BN(1,2) 0-indexed = row1 col2 */
      float gb_z = R_BN[8]; /* R_BN(2,2) 0-indexed = row2 col2 */
      float phi_grav = atan2f(gb_y, gb_z);
      float p_cmd = ramp_gain * (g.K_phi * roll_corr) * (-phi_grav);
      float e_p = p_cmd - p;

      /* Tustin PI (gains corrected for Clda) */
      state->roll_int += (Ts * 0.5f) * (e_p + state->roll_e_prev);
      float da_v_raw = (g.Kp_roll * roll_corr) * e_p +
                       (g.Ki_roll * roll_corr) * state->roll_int;
      da_v = clampf(da_v_raw, -delta_max, delta_max);

      /* Anti-windup */
      if (fabsf(da_v_raw) > delta_max && signf(e_p) == signf(da_v_raw)) {
        state->roll_int -= (Ts * 0.5f) * (e_p + state->roll_e_prev);
      }
      state->roll_e_prev = e_p;
    }
  }

  /* ---- Fin mixing: + config, roll priority ---- */
  float roll_lim = cfg->roll_alloc_max;
  if (roll_lim <= 0.0f || roll_lim > delta_max)
    roll_lim = delta_max;
  float da_alloc = clampf(da_v, -roll_lim, roll_lim);
  float remaining = delta_max - fabsf(da_alloc);
  float de_alloc = clampf(de_v, -remaining, remaining);
  float dr_alloc = clampf(dr_v, -remaining, remaining);

  out.fin_cmd[0] = clampf(de_alloc + da_alloc, -delta_max, delta_max);
  out.fin_cmd[1] = clampf(dr_alloc + da_alloc, -delta_max, delta_max);
  out.fin_cmd[2] = clampf(-de_alloc + da_alloc, -delta_max, delta_max);
  out.fin_cmd[3] = clampf(-dr_alloc + da_alloc, -delta_max, delta_max);

  out.de_cmd = de_alloc;
  out.dr_cmd = dr_alloc;
  out.da_cmd = da_alloc;

  return out;
}