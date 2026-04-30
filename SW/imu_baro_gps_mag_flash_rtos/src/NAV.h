#ifndef NAV_H
#define NAV_H

#include <Arduino.h>
#include <math.h>

#include "sensor_data.h"

class NAV {
private:
    Raw_imu   _raw_imu   = {};
    State_imu _state_imu = {};
    Raw_press _press     = {};
    Raw_mag   _mag       = {};
    Raw_gps   _gps       = {};
    State_nominal _nominal = {};

    // LSM6DSO32 ±32g / ±2000dps sensitivities
    // (LSM6DSO32::readCalibratedIMU returns int16 in SENSOR LSBs after axis remap)
    static constexpr float ACCEL_SCALE = 0.976f * 0.001f * 9.80665f;
    static constexpr float GYRO_SCALE  = 70.0f  * 0.001f * (M_PI / 180.0f);

public:
    NAV();

    // Sensor task callbacks — copy + (for IMU) convert to SI
    void updateIMU(const Raw_imu &raw);
    void updatePress(const Raw_press &p);
    void updateMag(const Raw_mag &m);
    void updateGps(const Raw_gps &g);

    // ES-EKF hooks (stubs; implement with ESEKF.m port)
    void ekfPredict(float dt);
    void ekfUpdateBaro();
    void ekfUpdateMag();
    void ekfUpdateGps();

    // Accessors
    Raw_imu       getRawImu()   const { return _raw_imu; }
    State_imu     getStateImu() const { return _state_imu; }
    Raw_press     getPress()    const { return _press; }
    Raw_mag       getMag()      const { return _mag; }
    Raw_gps       getGps()      const { return _gps; }
    State_nominal getNominal()  const { return _nominal; }
};

#endif
