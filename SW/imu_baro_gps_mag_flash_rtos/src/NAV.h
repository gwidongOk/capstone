#ifndef NAV_H
#define NAV_H

#include <Arduino.h>
#include <math.h>

// ==================== Sensor inputs (from sensor libs) ====================

// IMU — axis-aligned + int16 bias removed (by LSM6DSO32::readCalibratedIMU)
struct Raw_imu {
    uint32_t timestamp;
    int16_t gx, gy, gz;
    int16_t ax, ay, az;
};

// BARO — altitude (m) above pad (from BMP388::readAltitude)
struct Raw_press {
    uint32_t timestamp;
    float alt;
};

// MAG — fully calibrated Gauss (from MMC5983MA::readCalibratedMag)
struct Raw_mag {
    uint32_t timestamp;
    float mx, my, mz;
};

// GPS — NED pos/vel + accuracy + fix (from NEOM9N::getNED)
struct Raw_gps {
    uint32_t timestamp;
    float pn, pe, pd;
    float vn, ve, vd;
    float hAcc, vAcc;
    uint8_t fixType;
    uint8_t numSV;
    bool hasPos;   // true if origin set AND fix>=3
};

// ==================== EKF-side state ====================

// SI-converted IMU state (float), used by EKF predict
struct State_imu {
    uint32_t timestamp;
    float ax, ay, az;   // [m/s^2]
    float gx, gy, gz;   // [rad/s]
};

// ES-EKF nominal state (16 floats)
struct State_nominal {
    uint32_t timestamp;
    float p[3];   // NED position [m]
    float v[3];   // NED velocity [m/s]
    float q[4];   // quaternion (w, x, y, z)
    float ba[3];  // accel bias
    float bg[3];  // gyro bias
};

// ==================== NAV class ====================

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
