/**
 * @file autopilot.h
 * @brief 3-loop cascade autopilot + roll damper — eom_6dof.m autopilot 대응
 */

#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gnc_types.h"

/**
 * @brief 오토파일럿 상태 초기화.
 */
void autopilot_init(ap_state_t *state);

/**
 * @brief 오토파일럿 루프 1회 실행.
 *
 * @param nz_cmd   pitch 가속도 명령 (g)
 * @param ny_cmd   yaw 가속도 명령 (g)
 * @param nav      항법 상태
 * @param cfg      GNC 설정
 * @param state    오토파일럿 내부 상태 (in/out)
 * @return         핀 명령 (rad)
 */
ap_output_t autopilot_step(float nz_cmd, float ny_cmd, const nav_state_t *nav,
                           const gnc_config_t *cfg, ap_state_t *state);

#ifdef __cplusplus
}
#endif

#endif /* AUTOPILOT_H */