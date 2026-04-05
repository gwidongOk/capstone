#include "NAV.h"

NAV::NAV() {
    state_acc  = {0, 0.0f, 0.0f, 0.0f};
    state_gyro = {0, 0.0f, 0.0f, 0.0f};
    state_press = {0, 0.0f, 0.0f};
}

Raw_imu NAV::axis(Raw_imu data) {
    Raw_imu aligned;
    aligned.timestamp = data.timestamp;
    aligned.x =  data.y;
    aligned.y =  data.x;
    aligned.z = -data.z;
    return aligned;
}

void NAV::updateAccel(Raw_imu raw) {
    raw_acc = axis(raw);

    // 캘리브레이션 바이어스 보정 (float 정밀도 유지)
    float corr_x = (float)raw_acc.x - c_accel_x;
    float corr_y = (float)raw_acc.y - c_accel_y;
    float corr_z = (float)raw_acc.z - c_accel_z;

    state_acc.x = corr_x * ACCEL_SCALE;
    state_acc.y = corr_y * ACCEL_SCALE;
    state_acc.z = corr_z * ACCEL_SCALE;
    state_acc.timestamp = raw.timestamp;
}

void NAV::updateGyro(Raw_imu raw) {
    raw_gyro = axis(raw);

    // 캘리브레이션 바이어스 보정 (float 정밀도 유지)
    float corr_x = (float)raw_gyro.x - c_gyro_x;
    float corr_y = (float)raw_gyro.y - c_gyro_y;
    float corr_z = (float)raw_gyro.z - c_gyro_z;

    state_gyro.x = corr_x * GYRO_SCALE;
    state_gyro.y = corr_y * GYRO_SCALE;
    state_gyro.z = corr_z * GYRO_SCALE;
    state_gyro.timestamp = raw.timestamp;
}

void NAV::updatePress(Raw_press press) {
    state_press.pressure = press.p;
    state_press.altitude = getAltitude(press.p);
    state_press.timestamp = press.timestamp;
}

float NAV::getAltitude(float current_pressure) {
    return 44330.0f * (1.0f - pow(current_pressure / _padpressure, 0.1903f));
}

// =======================================================
// 캘리브레이션
// =======================================================
void NAV::calibrate(float c_gx, float c_gy, float c_gz,
                    float c_ax, float c_ay, float c_az, float c_p) {
    // 축 변환 적용 (axis 매핑: x←y, y←x, z←-z)
    // float 정밀도를 그대로 유지하여 바이어스 저장
    c_accel_x =  c_ay;   // axis: x ← y
    c_accel_y =  c_ax;   // axis: y ← x
    c_accel_z = -c_az;   // axis: z ← -z

    c_gyro_x =  c_gy;
    c_gyro_y =  c_gx;
    c_gyro_z = -c_gz;

    _padpressure = c_p;
}

RocketState_imu NAV::getState_acc()     { return state_acc; }
RocketState_imu NAV::getState_gyro()    { return state_gyro; }
RocketState_PRESS NAV::getState_press() { return state_press; }

Raw_imu NAV::getraw_acc()  { return raw_acc; }
Raw_imu NAV::getraw_gyro() { return raw_gyro; }
