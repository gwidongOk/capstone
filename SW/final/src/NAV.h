#ifndef NAV_H
#define NAV_H

#include <Arduino.h>
#include <math.h>

struct Raw_imu {
    unsigned long timestamp;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
};

struct RocketState_imu {
    unsigned long timestamp;
    float ax, ay, az;
    float gx, gy, gz;
};

struct Raw_press {
    unsigned long timestamp;
    float p;
};

struct RocketState_PRESS {
    unsigned long timestamp;
    float pressure;
    float altitude;
};

class NAV {
private:
    Raw_imu          raw_imu;
    RocketState_imu  state_imu;
    RocketState_PRESS state_press;

    // Calibration biases (float — must keep fractional part)
    float c_accel_x = 0.0f, c_accel_y = 0.0f, c_accel_z = 0.0f;
    float c_gyro_x  = 0.0f, c_gyro_y  = 0.0f, c_gyro_z  = 0.0f;

    Raw_imu axis(Raw_imu data);

    const float ACCEL_SCALE = 0.976f * 0.001f * 9.80665f;
    const float GYRO_SCALE  = 70.0f  * 0.001f * (M_PI / 180.0f);

    float _padpressure = 101325.0f;
    float getAltitude(float current_pressure);

public:
    NAV();

    void updateIMU(Raw_imu raw);
    void updatePress(Raw_press press);

    // calibrate() takes float averages already computed by imu.calibrate()
    void calibrate(float c_gx, float c_gy, float c_gz,
                   float c_ax, float c_ay, float c_az, float c_p);

    RocketState_imu   getState_imu()   const { return state_imu;  }
    RocketState_PRESS getState_press() const { return state_press; }
    Raw_imu           getRaw_imu()     const { return raw_imu;    }
};

#endif
