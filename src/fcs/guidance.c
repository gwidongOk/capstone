/**
 * @file guidance.c
 * Augmented Pure Pursuit guidance with range-based gain scheduling,
 * gravity lead compensation, wind drift lead, terminal gain boost,
 * and vector saturation. Called only during PHASE_GUIDE.
 */

#include "guidance.h"
#include "gnc_config.h"
#include "math_utils.h"

static vec3_t guid_augmented_pp(float V_mag, vec3_t u_V, vec3_t u_LOS,
                                float R_tgt, vec3_t los_NED,
                                const gnc_config_t *cfg);

void guidance_init(guid_state_t *state) {
  state->los_rate_filt[0] = 0.0f;
  state->los_rate_filt[1] = 0.0f;
  state->los_rate_filt[2] = 0.0f;
  state->ramp_timer = 0.0f;
  state->initialized = true;
}

guid_output_t guidance_step(const nav_state_t *nav, const gnc_config_t *cfg,
                            guid_state_t *state, float t) {
  guid_output_t out = {0};
  out.active = false;
  out.past_cpa = false;

  /* Ground velocity for guidance commands */
  float V_ground = vec3_norm(nav->vel_ned);
  if (V_ground < 1.0f)
    V_ground = 1.0f;

  float V_air = nav->airspeed;
  if (V_air < 1.0f)
    V_air = 1.0f;

  /* Defense-in-depth: pre-guidance guard */
  if (V_air < cfg->V_ctrl_on || t <= cfg->t_guide_on) {
    return out;
  }

  /* Dynamic pressure (for command saturation limit) */
  float alt = -nav->pos_ned.z;
  if (alt < 0.0f)
    alt = 0.0f;
  float rho = RHO_SL * expf(-alt / H_SCALE);
  float qbar = 0.5f * rho * V_air * V_air;

  vec3_t los_NED = vec3_sub(cfg->target_ned, nav->pos_ned);
  float R_tgt = vec3_norm(los_NED);
  out.range_to_tgt = R_tgt;

  if (R_tgt <= 2.0f) {
    return out;
  }

  /* Wind Drift Lead: correct LOS by predicted wind drift.
   * Safety guard: only apply if offset < 0.5 * R_tgt to prevent LOS reversal.
   */
  float wind_mag = vec3_norm(nav->wind_est);
  if (wind_mag > 0.01f) {
    float t_go_wind = R_tgt / V_ground;
    if (t_go_wind > 10.0f)
      t_go_wind = 10.0f;
    vec3_t wind_offset = vec3_scale(nav->wind_est, t_go_wind);

    if (vec3_norm(wind_offset) < 0.5f * R_tgt) {
      vec3_t los_corrected = vec3_sub(los_NED, wind_offset);
      float R_corr = vec3_norm(los_corrected);
      if (R_corr > 2.0f) {
        los_NED = los_corrected;
        R_tgt = R_corr;
      }
    }
  }

  float V_mag = V_ground;
  vec3_t u_V = vec3_scale(nav->vel_ned, 1.0f / V_mag);
  vec3_t u_LOS = vec3_scale(los_NED, 1.0f / R_tgt);

  float v_closing = vec3_dot(los_NED, nav->vel_ned) / R_tgt;
  out.v_closing = v_closing;
  out.active = true;

  /* Defense-in-depth: post-CPA guard */
  if (v_closing < 0.0f && t > cfg->t_guide_on + 1.0f) {
    out.past_cpa = true;
    out.nz_cmd = 0.0f;
    out.ny_cmd = 0.0f;
    return out;
  }

  vec3_t a_cmd = guid_augmented_pp(V_mag, u_V, u_LOS, R_tgt, los_NED, cfg);

  /* Gravity compensation -> body frame -> nz/ny */
  vec3_t gravity_ned = {0.0f, 0.0f, cfg->g};
  vec3_t a_need_ned = vec3_sub(a_cmd, gravity_ned);

  quat_t q_norm = quat_normalize(nav->quat);

  float R_BN[9];
  quat_to_dcm_bn(q_norm, R_BN);
  vec3_t a_need_body = dcm_bn_mul_vec(R_BN, a_need_ned);

  float nz_cmd = -a_need_body.z / cfg->g;
  float ny_cmd = a_need_body.y / cfg->g;

  /* Vector saturation (roll-authority-aware).
   * delta_avail = delta_max - roll_alloc_max to avoid commanding
   * more than the autopilot can track after roll allocation. */
  float m_est = cfg->m0 - cfg->m_prop;
  if (t < cfg->t_burn) {
    m_est = cfg->m0 - (cfg->m_prop / cfg->t_burn) * t;
  }
  float delta_avail = cfg->delta_max;
  if (cfg->roll_alloc_max > 0.0f) {
    delta_avail = cfg->delta_max - cfg->roll_alloc_max;
    if (delta_avail < 0.1f * cfg->delta_max) {
      delta_avail = 0.1f * cfg->delta_max;
    }
  }
  float nz_max = qbar * cfg->Sref * cfg->CNd * delta_avail / (m_est * cfg->g);

  float cmd_mag = sqrtf(nz_cmd * nz_cmd + ny_cmd * ny_cmd);
  if (cmd_mag > nz_max && cmd_mag > 1e-6f) {
    float scale = nz_max / cmd_mag;
    nz_cmd *= scale;
    ny_cmd *= scale;
  }

  /* Ramp-up at guidance start */
  float dt_guid = t - cfg->t_guide_on;
  if (dt_guid < GUID_RAMP_TIME) {
    float ramp = dt_guid / GUID_RAMP_TIME;
    nz_cmd *= ramp;
    ny_cmd *= ramp;
  }

  out.nz_cmd = nz_cmd;
  out.ny_cmd = ny_cmd;
  return out;
}

static vec3_t guid_augmented_pp(float V_mag, vec3_t u_V, vec3_t u_LOS,
                                float R_tgt, vec3_t los_NED,
                                const gnc_config_t *cfg) {
  /* Range-based gain scheduling */
  float progress = 1.0f - R_tgt / cfg->R_gain_sched;
  progress = clampf(progress, 0.0f, 1.0f);
  float K_guid =
      cfg->K_guid_far + (cfg->K_guid_near - cfg->K_guid_far) * progress;

  /* Gravity lead: predict free-fall drop over t_go, scaled by cos(gamma) */
  float t_go = R_tgt / V_mag;
  float gamma = asinf(clampf(-u_V.z, -1.0f, 1.0f));
  float grav_factor = cosf(gamma);
  vec3_t grav_drop = {0.0f, 0.0f, 0.5f * cfg->g * t_go * t_go * grav_factor};

  vec3_t los_lead = vec3_add(los_NED, grav_drop);
  float los_lead_n = vec3_norm(los_lead);

  vec3_t a_cmd;

  if (los_lead_n > 1.0f) {
    vec3_t u_LOS_lead = vec3_scale(los_lead, 1.0f / los_lead_n);
    vec3_t a_lead = vec3_scale(vec3_sub(u_LOS_lead, u_V), K_guid * V_mag);
    vec3_t a_pure = vec3_scale(vec3_sub(u_LOS, u_V), K_guid * V_mag);

    /* Blend: lead weight decreases as range decreases */
    float lead_w = clampf(R_tgt / cfg->R_gain_sched, 0.0f, 1.0f);
    a_cmd =
        vec3_add(vec3_scale(a_lead, lead_w), vec3_scale(a_pure, 1.0f - lead_w));
  } else {
    a_cmd = vec3_scale(vec3_sub(u_LOS, u_V), K_guid * V_mag);
  }

  /* Terminal gain boost (smooth cosine blend) */
  if (R_tgt < cfg->R_terminal) {
    float t_prog = 1.0f - R_tgt / cfg->R_terminal;
    float blend = 0.5f * (1.0f - cosf(PI_F * t_prog));
    vec3_t a_terminal = vec3_scale(vec3_sub(u_LOS, u_V), K_guid * 1.5f * V_mag);
    a_cmd = vec3_add(vec3_scale(a_cmd, 1.0f - blend),
                     vec3_scale(a_terminal, blend));
  }

  return a_cmd;
}