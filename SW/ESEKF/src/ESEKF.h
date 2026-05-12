#ifndef ESEKF_H
#define ESEKF_H

#include <Arduino.h>
#include <ArduinoEigen.h>

// 15-state Error-State EKF (MATLAB ESEKF.m 포트)
// state: [p(3) v(3) q(4) ba(3) bg(3)] / error: [dp dv dth dba dbg]
class ESEKF {
public:
    static constexpr int N = 15;

    // MATLAB SensorSpec 와 동일
    static constexpr float G_VAL    = 9.81f;
    static constexpr float DT_IMU   = 1.0f / 416.0f;
    static constexpr float VAR_ACC  = 2.729445e-4f;
    static constexpr float VAR_GYRO = 8.061781e-7f;
    static constexpr float VAR_BA   = 1.0e-6f;     // (1e-3)^2
    static constexpr float VAR_BG   = 1.0e-8f;     // (1e-4)^2
    static constexpr float VAR_BARO       = 4.920342e-2f;
    static constexpr float VAR_GPS_POS_H  = 0.694f * 0.694f;
    static constexpr float VAR_GPS_POS_V  = 1.282927f * 1.282927f;
    static constexpr float VAR_GPS_VEL_H  = 0.2684742f * 0.2684742f;
    static constexpr float VAR_GPS_VEL_V  = 0.2684742f * 0.2684742f;
    static constexpr float VAR_MAG        = (1.963f / 500.0f) * (1.963f / 500.0f);

    // P0 (초기 공분산)
    static constexpr float P0_POS = 3.0f;
    static constexpr float P0_VEL = 1.0f;
    static constexpr float P0_ATT = 0.087f;        // ~5deg
    static constexpr float P0_BA  = 0.5f;
    static constexpr float P0_BG  = 0.01f;

    ESEKF();
    void reset();
    void init(const float p0[3], const float v0[3], const float q0[4]);

    // TRIAD 초기 자세
    static void triad(const float acc[3], const float mag[3],
                      float q_out[4],
                      float lat_deg = NAN, float lon_deg = NAN);

    // 필터 사이클
    void predict(const float a_m[3], const float w_m[3], float dt);
    void updateGps(const float p_meas[3], const float v_meas[3], float hAcc, float vAcc);
    void updateBaro(float altitude_m);
    void updateMag(const float m_body[3]);
    void updateZupt();
    void updateAccStatic(const float a_m[3]);
    void updateGyroStatic(const float w_m[3]);

    // 접근자
    const float* p()  const { return _p.data(); }
    const float* v()  const { return _v.data(); }
    const float* q()  const { return _q_raw; }    // [w,x,y,z]
    const float* ba() const { return _ba.data(); }
    const float* bg() const { return _bg.data(); }
    float lastAccelMag() const { return _last_accel_mag; }
    void  covDiag(float out[15]) const;

    static Eigen::Matrix3f skew(const Eigen::Vector3f& v);

private:
    // nominal state
    Eigen::Vector3f    _p;
    Eigen::Vector3f    _v;
    Eigen::Quaternionf _q;       // body -> NED, [w,x,y,z]
    float              _q_raw[4];
    Eigen::Vector3f    _ba;
    Eigen::Vector3f    _bg;

    // error covariance
    Eigen::Matrix<float, 15, 15> _covP;

    Eigen::Vector3f _g_ned;
    Eigen::Vector3f _m_ref_ned;
    float           _last_accel_mag;

    void syncQuatRaw();
    static void wmmKorea(float lat, float lon, float& D, float& I);

    // 측정 업데이트 (fixed-size, no heap)
    template <int M>
    void measurementUpdate(const Eigen::Matrix<float, M, 15>& H,
                           const Eigen::Matrix<float, M, 1>& y,
                           const Eigen::Matrix<float, M, M>& R)
    {
        using namespace Eigen;
        Matrix<float, M, M>  S  = H * _covP * H.transpose() + R;
        Matrix<float, 15, M> K  = _covP * H.transpose() * S.inverse();
        Matrix<float, 15, 1> dx = K * y;

        Matrix<float, 15, 15> I15 = Matrix<float, 15, 15>::Identity();
        Matrix<float, 15, 15> IKH = I15 - K * H;
        _covP = IKH * _covP * IKH.transpose() + K * R * K.transpose();
        _covP = 0.5f * (_covP + _covP.transpose()).eval();

        // 오차 주입
        _p(0) += dx(0);  _p(1) += dx(1);  _p(2) += dx(2);
        _v(0) += dx(3);  _v(1) += dx(4);  _v(2) += dx(5);
        Vector3f dth(dx(6), dx(7), dx(8));
        _ba(0) += dx(9);  _ba(1) += dx(10); _ba(2) += dx(11);
        _bg(0) += dx(12); _bg(1) += dx(13); _bg(2) += dx(14);

        // dq from dth
        float dth_n = dth.norm();
        Quaternionf dq;
        if (dth_n > 1e-10f) {
            float s = sinf(dth_n * 0.5f) / dth_n;
            dq.w() = cosf(dth_n * 0.5f);
            dq.x() = s * dth.x();
            dq.y() = s * dth.y();
            dq.z() = s * dth.z();
        } else {
            dq.w() = 1.0f;
            dq.x() = dth.x() * 0.5f;
            dq.y() = dth.y() * 0.5f;
            dq.z() = dth.z() * 0.5f;
        }
        dq.normalize();
        _q = (_q * dq).normalized();
        syncQuatRaw();

        // 리셋 야코비안 G (자세 블록만 비단위)
        Matrix<float, 15, 15> G = I15;
        G.block<3,3>(6,6) = Matrix3f::Identity() - skew(dth * 0.5f);
        _covP = G * _covP * G.transpose();
        _covP = 0.5f * (_covP + _covP.transpose()).eval();
    }
};

#endif // ESEKF_H
