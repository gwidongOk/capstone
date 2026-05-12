#ifndef SERVO_HAL_H
#define SERVO_HAL_H

#include "gnc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void servo_init(void);
void servo_write(const servo_output_t *cmd);
void servo_center_all(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_HAL_H */
