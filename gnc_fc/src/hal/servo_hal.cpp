/**
 * @file servo_hal.cpp
 * Servo HAL — ESP32-S3 LEDC PWM (4 channels).
 * Uses Arduino ESP32 Core 2.x LEDC API.
 */

#include <Arduino.h>

#include "servo_hal.h"
#include "board_config.h"
#include "gnc_config.h"

static const int servo_pins[4] = {
    SERVO_1_PIN, SERVO_2_PIN, SERVO_3_PIN, SERVO_4_PIN
};

static const uint8_t LEDC_CHANNELS[4] = {0, 1, 2, 3};
static const uint32_t LEDC_FREQ = SERVO_PWM_FREQ_HZ;
static const uint8_t  LEDC_RES  = 14;

void servo_init(void)
{
    for (int i = 0; i < 4; i++) {
        ledcSetup(LEDC_CHANNELS[i], LEDC_FREQ, LEDC_RES);
        ledcAttachPin(servo_pins[i], LEDC_CHANNELS[i]);
    }
    servo_center_all();
}

void servo_write(const servo_output_t *cmd)
{
    float period_us = 1e6f / (float)LEDC_FREQ;
    uint32_t max_duty = (1u << LEDC_RES) - 1;

    for (int i = 0; i < 4; i++) {
        uint32_t duty = (uint32_t)((float)cmd->pulse_us[i] / period_us * (float)max_duty);
        if (duty > max_duty) duty = max_duty;
        ledcWrite(LEDC_CHANNELS[i], duty);
    }
}

void servo_center_all(void)
{
    servo_output_t center;
    for (int i = 0; i < 4; i++) {
        center.pulse_us[i] = SERVO_PWM_CENTER_US;
    }
    servo_write(&center);
}
