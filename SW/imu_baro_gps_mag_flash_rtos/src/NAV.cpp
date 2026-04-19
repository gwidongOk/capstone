#include "NAV.h"

NAV::NAV() {
    // Identity quaternion
    _nominal.q[0] = 1.0f;
    _nominal.q[1] = 0.0f;
    _nominal.q[2] = 0.0f;
    _nominal.q[3] = 0.0f;
}

void NAV::updateIMU(const Raw_imu &raw) {
    _raw_imu = raw;
    _state_imu.timestamp = raw.timestamp;
    _state_imu.ax = (float)raw.ax * ACCEL_SCALE;
    _state_imu.ay = (float)raw.ay * ACCEL_SCALE;
    _state_imu.az = (float)raw.az * ACCEL_SCALE;
    _state_imu.gx = (float)raw.gx * GYRO_SCALE;
    _state_imu.gy = (float)raw.gy * GYRO_SCALE;
    _state_imu.gz = (float)raw.gz * GYRO_SCALE;
}

void NAV::updatePress(const Raw_press &p) { _press = p; }
void NAV::updateMag(const Raw_mag &m)     { _mag   = m; }
void NAV::updateGps(const Raw_gps &g)     { _gps   = g; }

// ==================== ES-EKF stubs ====================
// Port ESEKF.m here. Read from _state_imu / _press / _mag / _gps and
// write to _nominal (+ internal covariance).

void NAV::ekfPredict(float dt) {
    (void)dt;
    _nominal.timestamp = _state_imu.timestamp;
    // TODO: nominal propagation (q, v, p) + covariance predict
}

void NAV::ekfUpdateBaro() { /* TODO: alt measurement update */ }
void NAV::ekfUpdateMag()  { /* TODO: mag yaw/tilt update */ }
void NAV::ekfUpdateGps()  { /* TODO: GPS pos+vel update */ }
