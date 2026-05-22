#include "ESEKF.h"

using namespace Eigen;

// IMU ODR and full-scale
constexpr float ESEKF::G_VAL;
constexpr float ESEKF::DT_IMU;
constexpr float ESEKF::VAR_ACC_STATIC;
constexpr float ESEKF::VAR_GYRO_STATIC;
constexpr float ESEKF::VAR_ACC_DS;
constexpr float ESEKF::VAR_GYRO_DS;
constexpr float ESEKF::VAR_ACC;
constexpr float ESEKF::VAR_GYRO;
constexpr float ESEKF::Q_ACC_SCALE;
constexpr float ESEKF::Q_GYRO_SCALE;
constexpr float ESEKF::VAR_BA;
constexpr float ESEKF::VAR_BG;
constexpr float ESEKF::VAR_BARO;
constexpr float ESEKF::VAR_GPS_POS_H;
constexpr float ESEKF::VAR_GPS_POS_V;
constexpr float ESEKF::VAR_GPS_VEL_H;
constexpr float ESEKF::VAR_GPS_VEL_V;
constexpr float ESEKF::VAR_MAG;
constexpr float ESEKF::GPS_POS_INFLATION;
constexpr float ESEKF::JERK_ALPHA;
constexpr float ESEKF::JERK_THRESH;
constexpr float ESEKF::ANG_ACC_THRESH;
constexpr float ESEKF::JERK_SCALE_MAX;

// =============================================================================
// 생성자 / 초기화
// =============================================================================

ESEKF::ESEKF() {
    reset();
}

// ----------------------------------------------------------------------------
// reset() : 필터 전체를 콜드 스타트 상태로 초기화
//   - nominal state(p, v, q, ba, bg)를 0/I로 리셋
//   - 잡음 분산 캐시(_v_*)를 Config 값으로 채움
//   - 공분산 P를 초기 불확실성에 맞춰 대각으로 세팅
//     (pos 3m, vel 1m/s, att 5deg, ba 0.5m/s^2, bg 0.01rad/s)
//   - Adaptive-Q EMA 버퍼도 모두 클리어
// ----------------------------------------------------------------------------
void ESEKF::reset() {
    _p.setZero();
    _v.setZero();
    _q.setIdentity();
    _ba.setZero();
    _bg.setZero();
    syncQuaternionRaw();

    // 프로세스/측정 노이즈 분산 캐시 (Config 값 기반)
    _v_acc  = Q_ACC_SCALE * VAR_ACC;
    _v_gyro = Q_GYRO_SCALE * VAR_GYRO;
    _v_ba   = VAR_BA;
    _v_bg   = VAR_BG;

    _v_baro = VAR_BARO;
    _v_mag  = VAR_MAG;
    _v_gps_ph = VAR_GPS_POS_H; _v_gps_pv = VAR_GPS_POS_V;
    _v_gps_vh = VAR_GPS_VEL_H; _v_gps_vv = VAR_GPS_VEL_V;

    // NED 프레임 기준 중력 / 자기장 (Korea 평균 WMM 모델)
    _g_ned << 0.0f, 0.0f, G_VAL;
    _m_ref_ned << 0.5961f, -0.0838f, 0.7986f; // 한국 표준 자기장 단위벡터

    // Adaptive-Q EMA 상태 초기화
    _last_accel_mag = 0.0f;
    _jerk_initialized = false;
    _current_jerk_scale_a = 1.0f;
    _ema_a.setZero();
    _ema_w.setZero();
    _last_a_lpf.setZero();
    _last_w_lpf.setZero();

    // 오차 공분산 P 초기화 (대각만 채움)
    _covP.setZero();
    _covP.diagonal().segment<3>(0).array()  = 3.0f * 3.0f;     // 위치 sigma=3m
    _covP.diagonal().segment<3>(3).array()  = 1.0f * 1.0f;     // 속도 sigma=1m/s
    _covP.diagonal().segment<3>(6).array()  = 0.087f * 0.087f; // 자세 sigma=5°
    _covP.diagonal().segment<3>(9).array()  = 0.5f * 0.5f;     // 가속바이어스 sigma=0.5
    _covP.diagonal().segment<3>(12).array() = 0.01f * 0.01f;   // 자이로바이어스 sigma=0.01
}

// ----------------------------------------------------------------------------
// init() : 외부에서 계산한 초기 자세/위치/속도로 EKF nominal state를 시드한다.
//   - p0, v0 : NED 프레임 위치/속도 (보통 GPS origin과 정지값)
//   - q0     : TRIAD 정렬 결과 쿼터니언 [w,x,y,z]
// ----------------------------------------------------------------------------
void ESEKF::init(const float p0[3], const float v0[3], const float q0[4]) {
    _p = Vector3f(p0[0], p0[1], p0[2]);
    _v = Vector3f(v0[0], v0[1], v0[2]);
    _q = Quaternionf(q0[0], q0[1], q0[2], q0[3]); // [w,x,y,z]
    _q.normalize();
    syncQuaternionRaw();
}

void ESEKF::setMagReferenceByLocation(float lat_deg, float lon_deg) {
    if (isnan(lat_deg) || isnan(lon_deg)) return;

    float decl_deg, incl_deg;
    wmmKorea(lat_deg, lon_deg, decl_deg, incl_deg);

    const float decl = decl_deg * 0.01745329252f;
    const float incl = incl_deg * 0.01745329252f;
    _m_ref_ned << cosf(incl) * cosf(decl),
                  cosf(incl) * sinf(decl),
                  sinf(incl);
    _m_ref_ned.normalize();
}

// =============================================================================
// 필터 사이클 (Predict / Update)
// =============================================================================

// ----------------------------------------------------------------------------
// predict() : 고정 Q 예측 단계
//   - IMU 측정값(a_m, w_m)으로 nominal state를 dt만큼 적분
//   - Q 스케일링 없이 (1.0, 1.0) 기본 Q 사용
// ----------------------------------------------------------------------------
void ESEKF::predict(const float a_m[3], const float w_m[3], float dt) {
    _current_jerk_scale_a = 1.0f;
    propagate(a_m, w_m, dt, 1.0f, 1.0f);
}

// ----------------------------------------------------------------------------
// predictAdaptiveJerk() : 저크 기반 적응형 Q 예측 단계
//   1) IMU 신호에 EMA 저역통과 필터를 걸어 부드러운 a, ω 추정
//   2) (EMA 미분) = 저크/각가속도 추정값
//   3) 저크가 임계(JERK_THRESH/ANG_ACC_THRESH)를 넘으면 r=저크/임계 ratio로 Q 부풀리기
//      qs = r^2, 단 JERK_SCALE_MAX 상한
//   - 직진 안정구간에서는 Q≈기본, 고기동 구간에서는 Q를 늘려 잔차 추종력 향상
// ----------------------------------------------------------------------------
void ESEKF::predictAdaptiveJerk(const float a_m[3], const float w_m[3], float dt) {
    Vector3f am(a_m[0], a_m[1], a_m[2]);
    Vector3f wm(w_m[0], w_m[1], w_m[2]);

    if (!_jerk_initialized) {
        _ema_a = am;
        _ema_w = wm;
        _last_a_lpf = am;
        _last_w_lpf = wm;
        _jerk_initialized = true;
    }

    _ema_a = JERK_ALPHA * am + (1.0f - JERK_ALPHA) * _ema_a;
    _ema_w = JERK_ALPHA * wm + (1.0f - JERK_ALPHA) * _ema_w;

    const float inv_dt = (dt > 1.0e-6f) ? (1.0f / dt) : 0.0f;
    const float jerk_a = (_ema_a - _last_a_lpf).norm() * inv_dt;
    const float jerk_w = (_ema_w - _last_w_lpf).norm() * inv_dt;

    _last_a_lpf = _ema_a;
    _last_w_lpf = _ema_w;

    float qs_a = 1.0f;
    float qs_w = 1.0f;
    if (jerk_a > JERK_THRESH) {
        const float r = jerk_a / JERK_THRESH;
        qs_a = r * r;
        if (qs_a > JERK_SCALE_MAX) qs_a = JERK_SCALE_MAX;
    }
    if (jerk_w > ANG_ACC_THRESH) {
        const float r = jerk_w / ANG_ACC_THRESH;
        qs_w = r * r;
        if (qs_w > JERK_SCALE_MAX) qs_w = JERK_SCALE_MAX;
    }

    _current_jerk_scale_a = qs_a;
    propagate(a_m, w_m, dt, qs_a, qs_w);
}

// ----------------------------------------------------------------------------
// propagate() : ES-EKF predict 본체 (nominal state 적분 + 공분산 전파)
//   입력 : a_m, w_m (IMU 측정), dt, q_acc_scale, q_gyro_scale (적응형 부풀리기 계수)
//
//   [1] Nominal State 적분
//       - 바이어스 보정 : a_hat = a_m - ba,  w_hat = w_m - bg
//       - NED 가속 :     a_ned = R_nb * a_hat + g_ned  (중력 더해줌)
//       - 위치/속도 :    p += v*dt + 1/2*a*dt^2,  v += a*dt
//       - 자세 :         q = q ⊗ exp(w_hat*dt)  (작은 각일 때 1차 근사)
//
//   [2] Error-State 공분산 P 전파
//       - 연속 야코비안 Fc 구성 (오차상태 d_p, d_v, d_θ, d_ba, d_bg 에 대해)
//       - F = exp(Fc*dt) ≈ I + Fc*dt + (Fc*dt)^2/2 + (Fc*dt)^3/6 (3차 테일러)
//       - 이산 Q = diag(v_acc*qs*dt^2, v_gyro*qs*dt^2, v_ba*dt, v_bg*dt)
//       - P = F*P*F^T + Q,  대칭화로 수치오차 억제
// ----------------------------------------------------------------------------
void ESEKF::propagate(const float a_m[3], const float w_m[3], float dt, float q_acc_scale, float q_gyro_scale) {
    Vector3f am(a_m[0], a_m[1], a_m[2]);
    Vector3f wm(w_m[0], w_m[1], w_m[2]);

    // 바이어스 보정된 IMU
    Vector3f a_hat = am - _ba;
    Vector3f w_hat = wm - _bg;

    _last_accel_mag = a_hat.norm();  // GPS R 인플레이션에 사용 (고G 구간 감지)

    Matrix3f R_nb = _q.toRotationMatrix();

    // [1] Nominal state 적분 (위치 → 속도 → 자세)
    Vector3f a_ned = R_nb * a_hat + _g_ned;     // body → NED 변환 후 중력 더함
    _p += _v * dt + 0.5f * a_ned * dt * dt;     // 2차 적분 (정확도 ↑)
    _v += a_ned * dt;

    // 자세 업데이트 : 작은 각 회전을 쿼터니언 곱셈으로 적용
    Vector3f theta = w_hat * dt;
    float th_n = theta.norm();
    if (th_n > 1e-10f) {
        _q = (_q * Quaternionf(AngleAxisf(th_n, theta/th_n))).normalized();
    } else {
        // 각이 매우 작으면 1차 근사 (수치 안정성)
        _q = (_q * Quaternionf(1.0f, theta.x()*0.5f, theta.y()*0.5f, theta.z()*0.5f)).normalized();
    }
    syncQuaternionRaw();

    // [2] 오차상태 공분산 전파 (15x15)
    //   상태 순서 : [d_p(0..2), d_v(3..5), d_θ(6..8), d_ba(9..11), d_bg(12..14)]
    Matrix<float, 15, 15> Fc = Matrix<float, 15, 15>::Zero();
    Fc.block<3,3>(0, 3) = Matrix3f::Identity();        // dp/dt = dv
    Fc.block<3,3>(3, 6) = -R_nb * skew(a_hat);         // dv/dθ : 자세오차가 가속에 미치는 영향
    Fc.block<3,3>(3, 9) = -R_nb;                       // dv/dba : 가속 바이어스가 속도에 직접
    Fc.block<3,3>(6, 6) = -skew(w_hat);                // dθ/dθ : 자이로 자체 항
    Fc.block<3,3>(6, 12)= -Matrix3f::Identity();       // dθ/dbg : 자이로 바이어스

    Matrix<float, 15, 15> Fdt = Fc * dt;
    Matrix<float, 15, 15> Fdt2 = Fdt * Fdt;
    Matrix<float, 15, 15> Fdt3 = Fdt2 * Fdt;

    // 이산화 : F ≈ I + Fdt + Fdt^2/2 + Fdt^3/6 (3차 매트릭스 익스포넨셜)
    Matrix<float, 15, 15> F = Matrix<float, 15, 15>::Identity() + Fdt + 0.5f * Fdt2 + (1.0f/6.0f) * Fdt3;

    // 이산 프로세스 잡음 Q (적응형 스케일 q_acc_scale, q_gyro_scale 곱)
    Matrix<float, 15, 15> Q = Matrix<float, 15, 15>::Zero();
    Q.diagonal().segment<3>(3).array()  = _v_acc * q_acc_scale * dt * dt;   // 속도 잡음
    Q.diagonal().segment<3>(6).array()  = _v_gyro * q_gyro_scale * dt * dt; // 자세 잡음
    Q.diagonal().segment<3>(9).array()  = _v_ba * dt;                       // 가속 바이어스 워크
    Q.diagonal().segment<3>(12).array() = _v_bg * dt;                       // 자이로 바이어스 워크

    _covP = F * _covP * F.transpose() + Q;
    _covP = 0.5f * (_covP + _covP.transpose()).eval();  // 대칭화 (부동소수 오차 보정)
}

// =============================================================================
// 센서 측정 업데이트 (Measurement Update)
// =============================================================================

// ----------------------------------------------------------------------------
// updateGps() : GPS 위치+속도 측정 융합 (NED 6차원)
//   - 측정모델 H = [I_3 0 0 0 0; 0 I_3 0 0 0] → 위치/속도 직접 관측
//   - innovation y = (GPS측정) - (현재 추정)
//   - R 행렬은 hAcc/vAcc/sAcc에 INFLATION 곱하고, 고G(>4g) 구간에서는
//     high_g_penalty (최대 20배)로 더 부풀려 자이로/가속 누적오차의 영향 차단
// ----------------------------------------------------------------------------
void ESEKF::updateGps(float pn, float pe, float pd, float vn, float ve, float vd, float hAcc, float vAcc, float sAcc) {
    // 관측 행렬 H : 6x15 (위치 3 + 속도 3)
    Matrix<float, 6, 15> H = Matrix<float, 6, 15>::Zero();
    H.block<3,3>(0,0) = Matrix3f::Identity();   // 위치 직접 관측
    H.block<3,3>(3,3) = Matrix3f::Identity();   // 속도 직접 관측

    // 잔차(innovation)
    Vector<float, 6> y;
    y << pn-_p.x(), pe-_p.y(), pd-_p.z(), vn-_v.x(), ve-_v.y(), vd-_v.z();

    // GPS 자체 보고 정확도에 INFLATION 곱 (수신기는 보통 낙관적)
    const float pos_h = hAcc * GPS_POS_INFLATION;
    const float pos_v = vAcc * GPS_POS_INFLATION;
    const float vel_s = (sAcc > 0.01f) ? sAcc : sqrtf(_v_gps_vh);

    // 고G 구간 페널티 : 4g 초과부터 quadratic 증가, 상한 20배
    //   → 모터 점화/공력 충격 중에는 GPS를 거의 무시
    float high_g_penalty = 1.0f;
    const float accel_g = _last_accel_mag / G_VAL;
    if (accel_g > 4.0f) {
        const float excess_g = accel_g - 4.0f;
        high_g_penalty = 1.0f + 2.0f * excess_g * excess_g;
        if (high_g_penalty > 20.0f) high_g_penalty = 20.0f;
    }

    float var_h  = pos_h * pos_h * high_g_penalty;
    float var_v  = pos_v * pos_v * high_g_penalty;
    float var_vh = vel_s * vel_s * high_g_penalty;
    float var_vv = vel_s * vel_s * high_g_penalty;

    Matrix<float, 6, 6> R = Matrix<float, 6, 6>::Zero();
    R.diagonal() << var_h, var_h, var_v, var_vh, var_vh, var_vv;

    measurementUpdate(H, y, R);
}

// ----------------------------------------------------------------------------
// updateBaro() : 기압고도 측정 (1차원)
//   - NED p.z는 down이 양수이므로 고도 = -p.z
//   - H = [0 0 -1 0..0] 1x15
// ----------------------------------------------------------------------------
void ESEKF::updateBaro(float alt) {
    Matrix<float, 1, 15> H = Matrix<float, 1, 15>::Zero();
    H(0, 2) = -1.0f;                                    // 고도 = -p_D
    Matrix<float, 1, 1> y; y(0, 0) = alt - (-_p.z());   // 잔차
    Matrix<float, 1, 1> R; R(0, 0) = _v_baro;
    measurementUpdate(H, y, R);
}

// ----------------------------------------------------------------------------
// updateMag() : 자력계 단위벡터 측정으로 yaw 보정
//   - 측정값 mb를 단위벡터로 정규화 → 자세만 추정 (강도는 무시)
//   - 예측값 z_pred = R_bn * m_ref_NED (NED 자기장 → body 변환)
//   - 자세오차 d_θ의 영향 : H_θ = skew(z_pred)
// ----------------------------------------------------------------------------
void ESEKF::updateMag(const float m_body[3]) {
    Vector3f mb(m_body[0], m_body[1], m_body[2]);
    if (mb.norm() < 1e-4f) return;  // 측정값이 비정상이면 스킵
    mb.normalize();

    Vector3f z_pred = _q.toRotationMatrix().transpose() * _m_ref_ned;
    Vector3f y = mb - z_pred;

    Matrix<float, 3, 15> H = Matrix<float, 3, 15>::Zero();
    H.block<3,3>(0, 6) = skew(z_pred);  // 자세오차에 대한 야코비안

    Matrix3f R = Matrix3f::Identity() * _v_mag;
    measurementUpdate(H, y, R);
}

// ----------------------------------------------------------------------------
// updateZupt() : Zero-Velocity Update (정지 상태 의사측정)
//   - 진짜 정지 중일 때만 호출. 측정 = 0 - v (속도가 0이어야 함)
//   - 매우 작은 R(1e-4)로 강하게 속도/위치/자세 오차 보정 → 정렬 가속
// ----------------------------------------------------------------------------
void ESEKF::updateZupt() {
    Matrix<float, 3, 15> H = Matrix<float, 3, 15>::Zero();
    H.block<3,3>(0, 3) = Matrix3f::Identity();
    Vector3f y = -_v;                                  // 측정(0) - 추정(v)
    Matrix3f R = Matrix3f::Identity() * 1e-4f;         // 매우 작은 잡음 → 강한 보정
    measurementUpdate(H, y, R);
}

// ----------------------------------------------------------------------------
// updateAccStatic() : 정지 상태 가속도 측정으로 자세 + 가속 바이어스 동시 보정
//   - 정지 가정 : am = R_bn * (-g_ned) + ba
//   - H_θ = skew(gravity_body), H_ba = I  → 두 항을 동시에 추정
// ----------------------------------------------------------------------------
void ESEKF::updateAccStatic(const float a_m[3]) {
    Vector3f am(a_m[0], a_m[1], a_m[2]);
    Vector3f gravity_body = _q.toRotationMatrix().transpose() * (-_g_ned);
    Vector3f y = am - (gravity_body + _ba);

    Matrix<float, 3, 15> H = Matrix<float, 3, 15>::Zero();
    H.block<3,3>(0, 6) = skew(gravity_body);
    H.block<3,3>(0, 9) = Matrix3f::Identity();

    Matrix3f R = Matrix3f::Identity() * VAR_ACC;
    measurementUpdate(H, y, R);
}

// ----------------------------------------------------------------------------
// updateGyroStatic() : 정지 중 자이로 측정으로 자이로 바이어스 직접 보정
//   - 정지 가정 : wm ≈ bg
//   - H_bg = I
// ----------------------------------------------------------------------------
void ESEKF::updateGyroStatic(const float w_m[3]) {
    Vector3f wm(w_m[0], w_m[1], w_m[2]);
    Vector3f y = wm - _bg;
    Matrix<float, 3, 15> H = Matrix<float, 3, 15>::Zero();
    H.block<3,3>(0, 12) = Matrix3f::Identity();
    Matrix3f R = Matrix3f::Identity() * VAR_GYRO;
    measurementUpdate(H, y, R);
}

// =============================================================================
// 정지 정렬 루프 (오프라인/시뮬용)
// =============================================================================

// ----------------------------------------------------------------------------
// runZuptAlignment() : 0.1초 간격으로 predict 후 ZUPT+AccStatic+GyroStatic+Mag 적용
//   - 자세/바이어스 공분산 trace 변화율이 threshold 이하로 떨어지면 수렴으로 판정
//   - 실시간 메인 펌웨어에서는 main.cpp의 ZUPT 루프가 같은 일을 수행 (이 함수는 미사용)
// ----------------------------------------------------------------------------
bool ESEKF::runZuptAlignment(IMUProvider imu_proc, float dt, float threshold, float max_time, MagProvider mag_proc) {
    float t = 0;
    int steps = round(0.1f / dt);
    while (t < max_time) {
        float am[3], wm[3];
        for (int i=0; i<steps; i++) { imu_proc(am, wm); predict(am, wm, dt); t += dt; }
        
        float P_before = _covP.diagonal().segment<9>(6).sum();
        updateZupt(); updateAccStatic(am); updateGyroStatic(wm);
        if (mag_proc) { float mm[3]; mag_proc(mm); updateMag(mm); }
        
        float P_after = _covP.diagonal().segment<9>(6).sum();
        if (P_before > 0 && fabsf(P_before - P_after) / P_before < threshold) return true;
    }
    return false;
}

// Math Utilities

void ESEKF::syncQuaternionRaw() {
    _q_raw[0] = _q.w(); _q_raw[1] = _q.x(); _q_raw[2] = _q.y(); _q_raw[3] = _q.z();
}

Matrix3f ESEKF::skew(const Vector3f& v) {
    Matrix3f S;
    S << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return S;
}

void ESEKF::quat2euler(const float q[4], float rpy[3]) {
    Quaternionf eig_q(q[0], q[1], q[2], q[3]);
    auto eul = eig_q.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX
    rpy[0] = eul[2]; rpy[1] = eul[1]; rpy[2] = eul[0];
}

// ----------------------------------------------------------------------------
// triad() : TRIAD 알고리즘으로 초기 자세 추정 (가속 + 자력 → 쿼터니언)
//   - body 가속 → -z_NED (중력 방향) 매칭
//   - body 자력 → r2_NED (한국 자기장 단위벡터) 매칭
//   - 두 벡터쌍으로 직교기저를 만들어 회전행렬 R_bn 구성 후 쿼터니언으로 변환
//   - lat/lon이 주어지면 WMM 한국 근사식으로 r2 다시 계산
// ----------------------------------------------------------------------------
void ESEKF::triad(const float acc[3], const float mag[3], float q_out[4], float lat, float lon) {
    Vector3f a(acc[0], acc[1], acc[2]); a.normalize();
    Vector3f m(mag[0], mag[1], mag[2]); m.normalize();

    Vector3f r1(0, 0, 1);
    Vector3f r2(0.5961f, -0.0838f, 0.7986f);
    if (!isnan(lat) && !isnan(lon)) {
        float d, i; wmmKorea(lat, lon, d, i);
        float dr = d*0.01745f, ir = i*0.01745f;
        r2 << cosf(ir)*cosf(dr), cosf(ir)*sinf(dr), sinf(ir);
    }

    auto build = [](const Vector3f& v1, const Vector3f& v2) {
        Vector3f u1 = v1;
        Vector3f u2 = u1.cross(v2).normalized();
        Vector3f u3 = u1.cross(u2);
        Matrix3f M; M << u1, u2, u3; return M;
    };

    Matrix3f R_bn = build(Vector3f(0,0,1), r2) * build(-a, m).transpose();
    Quaternionf q(R_bn);
    q_out[0] = q.w(); q_out[1] = q.x(); q_out[2] = q.y(); q_out[3] = q.z();
}

void ESEKF::wmmKorea(float lat, float lon, float &d, float &i) {
    d = -7.9f - 0.35f*(lat-36.0f) + 0.15f*(lon-127.0f);
    i = 51.0f + 1.40f*(lat-36.0f) + 0.05f*(lon-127.0f);
}
