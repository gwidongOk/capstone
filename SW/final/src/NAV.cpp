#include "NAV.h"

NAV::NAV() {
    state_imu   = {0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    state_press = {0, 0.0f, 0.0f};
}

Raw_imu NAV::axis(Raw_imu data) {
    Raw_imu a;
    a.timestamp = data.timestamp;
    a.ax =  data.ay;  a.ay =  data.ax;  a.az = -data.az;
    a.gx =  data.gy;  a.gy =  data.gx;  a.gz = -data.gz;
    return a;
}

void NAV::updateIMU(Raw_imu raw) {
    raw_imu = axis(raw);

    state_imu.ax = ((float)raw_imu.ax - c_accel_x) * ACCEL_SCALE;
    state_imu.ay = ((float)raw_imu.ay - c_accel_y) * ACCEL_SCALE;
    state_imu.az = ((float)raw_imu.az - c_accel_z) * ACCEL_SCALE;
    state_imu.gx = ((float)raw_imu.gx - c_gyro_x)  * GYRO_SCALE;
    state_imu.gy = ((float)raw_imu.gy - c_gyro_y)  * GYRO_SCALE;
    state_imu.gz = ((float)raw_imu.gz - c_gyro_z)  * GYRO_SCALE;
    state_imu.timestamp = raw.timestamp;
}

void NAV::updatePress(Raw_press press) {
    state_press.pressure  = press.p;
    state_press.altitude  = getAltitude(press.p);
    state_press.timestamp = press.timestamp;
}

float NAV::getAltitude(float p) {
    return 44330.0f * (1.0f - powf(p / _padpressure, 0.1903f));
}

void NAV::calibrate(float c_gx, float c_gy, float c_gz,
                    float c_ax, float c_ay, float c_az, float c_p) {
    // Apply same axis remap as axis()
    c_accel_x =  c_ay; c_accel_y =  c_ax; c_accel_z = -c_az;
    c_gyro_x  =  c_gy; c_gyro_y  =  c_gx; c_gyro_z  = -c_gz;
    _padpressure = c_p;
}
