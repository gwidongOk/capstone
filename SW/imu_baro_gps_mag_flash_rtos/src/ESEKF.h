#ifndef ESEKF_H
#define ESEKF_H

#include <Arduino.h>
#include <ArduinoEigen.h>
#include "Config.h"


class ESEKF {
public:
    static constexpr int N_STATE = 15;

    static constexpr float G_VAL     = EKF_G_VAL;
    static constexpr float DT_IMU    = 1.0f / EKF_IMU_RATE_HZ;
    static constexpr float VAR_ACC_STATIC  = EKF_VAR_ACC_STATIC;
    static constexpr float VAR_GYRO_STATIC = EKF_VAR_GYRO_STATIC;
    static constexpr float VAR_ACC_DS      = ((EKF_ACC_NOISE_DENSITY * G_VAL) * (EKF_ACC_NOISE_DENSITY * G_VAL)) / DT_IMU;
    static constexpr float VAR_GYRO_DS     = ((EKF_GYRO_NOISE_DENSITY * 3.14159265358979323846f / 180.0f) *
                                              (EKF_GYRO_NOISE_DENSITY * 3.14159265358979323846f / 180.0f)) / DT_IMU;
    static constexpr float VAR_ACC         = (VAR_ACC_DS > VAR_ACC_STATIC) ? VAR_ACC_DS : VAR_ACC_STATIC;
    static constexpr float VAR_GYRO        = (VAR_GYRO_DS > VAR_GYRO_STATIC) ? VAR_GYRO_DS : VAR_GYRO_STATIC;
    static constexpr float Q_ACC_SCALE     = EKF_Q_ACC_SCALE;
    static constexpr float Q_GYRO_SCALE    = EKF_Q_GYRO_SCALE;
    static constexpr float VAR_BA    = EKF_VAR_BA;
    static constexpr float VAR_BG    = EKF_VAR_BG;

    static constexpr float VAR_BARO      = EKF_VAR_BARO;
    static constexpr float VAR_GPS_POS_H = EKF_VAR_GPS_POS_H;
    static constexpr float VAR_GPS_POS_V = EKF_VAR_GPS_POS_V;
    static constexpr float VAR_GPS_VEL_H = EKF_VAR_GPS_VEL_H;
    static constexpr float VAR_GPS_VEL_V = EKF_VAR_GPS_VEL_V;
    static constexpr float VAR_MAG       = EKF_VAR_MAG;
    static constexpr float GPS_POS_INFLATION = EKF_GPS_POS_INFLATION;
    static constexpr float JERK_ALPHA        = EKF_JERK_ALPHA;
    static constexpr float JERK_THRESH       = EKF_JERK_THRESH;
    static constexpr float ANG_ACC_THRESH    = EKF_ANG_ACC_THRESH;
    static constexpr float JERK_SCALE_MAX    = EKF_JERK_SCALE_MAX;

    // Filter Lifecycle
    ESEKF();
    void reset();
    void init(const float p0[3], const float v0[3], const float q0[4]);
    void setMagReferenceByLocation(float lat_deg, float lon_deg);

    // TRIAD Alignment
    static void triad(const float acc[3], const float mag[3], float q_out[4], 
                      float lat_deg = NAN, float lon_deg = NAN);

    // Filter Cycle
    void predict(const float a_m[3], const float w_m[3], float dt);
    void predictAdaptiveJerk(const float a_m[3], const float w_m[3], float dt);
    void updateGps(float pn, float pe, float pd, float vn, float ve, float vd, float hAcc, float vAcc, float sAcc);
    void updateBaro(float altitude_m);
    void updateMag(const float m_body[3]);
    void updateZupt();
    void updateAccStatic(const float a_m[3]);
    void updateGyroStatic(const float w_m[3]);

    typedef void (*IMUProvider)(float*, float*);
    typedef void (*MagProvider)(float*);
    bool runZuptAlignment(IMUProvider imu_proc, float dt, float threshold = 1e-4f, 
                          float max_time = 30.0f, MagProvider mag_proc = nullptr);

    // Accessors
    const float* position()   const { return _p.data(); }
    const float* velocity()   const { return _v.data(); }
    const float* quaternion() const { return _q_raw;    }
    const float* accelBias()  const { return _ba.data(); }
    const float* gyroBias()   const { return _bg.data(); }
    const float* covariance() const { return _covP.data();  }
    float jerkScale() const { return _current_jerk_scale_a; }

    float attBiasCovTrace() const { return _covP.diagonal().segment<9>(6).sum(); }

    static void quat2euler(const float q[4], float rpy[3]);
    static Eigen::Matrix3f skew(const Eigen::Vector3f& v);

private:
    // Nominal State
    Eigen::Vector3f _p;
    Eigen::Vector3f _v;
    Eigen::Quaternionf _q;
    float _q_raw[4]; // [w,x,y,z]
    Eigen::Vector3f _ba;
    Eigen::Vector3f _bg;

    // Error Covariance
    Eigen::Matrix<float, 15, 15> _covP;

    float _v_acc, _v_gyro, _v_ba, _v_bg;
    float _v_gps_ph, _v_gps_pv, _v_gps_vh, _v_gps_vv;
    float _v_baro, _v_mag;

    Eigen::Vector3f _g_ned;
    Eigen::Vector3f _m_ref_ned;

    float _last_accel_mag;
    bool _jerk_initialized;
    float _current_jerk_scale_a;
    Eigen::Vector3f _ema_a;
    Eigen::Vector3f _ema_w;
    Eigen::Vector3f _last_a_lpf;
    Eigen::Vector3f _last_w_lpf;

    // Internal Helpers
    void propagate(const float a_m[3], const float w_m[3], float dt, float q_acc_scale, float q_gyro_scale);
    // Joseph update
    template <int M>
    void measurementUpdate(const Eigen::Matrix<float, M, N_STATE>& H,
                           const Eigen::Matrix<float, M, 1>& y,
                           const Eigen::Matrix<float, M, M>& R) {
        // Kalman gain
        Eigen::Matrix<float, M, M> S = H * _covP * H.transpose() + R;
        Eigen::Matrix<float, N_STATE, M> PHt = _covP * H.transpose();
        Eigen::Matrix<float, M, N_STATE> solved = S.ldlt().solve(PHt.transpose());
        Eigen::Matrix<float, N_STATE, M> K = solved.transpose();
        Eigen::Matrix<float, N_STATE, 1> dx = K * y;

        // Covariance
        Eigen::Matrix<float, N_STATE, N_STATE> IKH =
            Eigen::Matrix<float, N_STATE, N_STATE>::Identity() - K * H;
        _covP = IKH * _covP * IKH.transpose() + K * R * K.transpose();
        _covP = 0.5f * (_covP + _covP.transpose()).eval();

        // State injection
        _p  += dx.template segment<3>(0);
        _v  += dx.template segment<3>(3);
        _ba += dx.template segment<3>(9);
        _bg += dx.template segment<3>(12);

        // Attitude injection
        Eigen::Vector3f dth = dx.template segment<3>(6);
        float dth_n = dth.norm();
        if (dth_n > 1e-10f) {
            _q = (_q * Eigen::Quaternionf(Eigen::AngleAxisf(dth_n, dth / dth_n))).normalized();
        } else {
            _q = (_q * Eigen::Quaternionf(1.0f, dth.x() * 0.5f, dth.y() * 0.5f, dth.z() * 0.5f)).normalized();
        }
        syncQuaternionRaw();

        // Reset Jacobian
        Eigen::Matrix<float, N_STATE, N_STATE> G =
            Eigen::Matrix<float, N_STATE, N_STATE>::Identity();
        G.template block<3,3>(6, 6) = Eigen::Matrix3f::Identity() - skew(dth * 0.5f);
        _covP = G * _covP * G.transpose();
        _covP = 0.5f * (_covP + _covP.transpose()).eval();
    }
    void syncQuaternionRaw();
    static void wmmKorea(float lat, float lon, float &d, float &i);
};

#endif
