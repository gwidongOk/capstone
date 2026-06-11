#include "NAV.h"
#include <math.h>

NAV::NAV() {
    _nominal.q[0] = 1.0f;
    _nominal.q[1] = 0.0f;
    _nominal.q[2] = 0.0f;
    _nominal.q[3] = 0.0f;
}

// =============================================================================
// 센서 태스크 콜백 (각 센서가 새 raw 데이터를 받을 때 호출)
//   - raw int16 → SI 단위 float 변환 후 EKF 함수 호출
//   - 모든 호출은 navMutex로 보호된 컨텍스트에서 실행됨 (main.cpp 측 책임)
// =============================================================================

// ----------------------------------------------------------------------------
// updateIMU() : IMU 콜백
//   - int16 raw → float (m/s^2, rad/s) 스케일링
//   - START 직후 _init_avg_active 동안 가속/각속도 평균 누적 (TRIAD 평균용)
//   - 이전 호출과의 시간차로 dt 계산 후 ekfPredict
// ----------------------------------------------------------------------------
void NAV::updateIMU(const Raw_imu &raw) {
    _raw_imu = raw;
    _state_imu.timestamp = raw.timestamp;
    _state_imu.ax = (float)raw.ax * ACCEL_SCALE;
    _state_imu.ay = (float)raw.ay * ACCEL_SCALE;
    _state_imu.az = (float)raw.az * ACCEL_SCALE;
    _state_imu.gx = (float)raw.gx * GYRO_SCALE;
    _state_imu.gy = (float)raw.gy * GYRO_SCALE;
    _state_imu.gz = (float)raw.gz * GYRO_SCALE;

    if (_init_avg_active) {
        _init_acc_sum[0] += _state_imu.ax;
        _init_acc_sum[1] += _state_imu.ay;
        _init_acc_sum[2] += _state_imu.az;
        if (_init_imu_count < 65535) _init_imu_count++;
    }

    // dt
    const uint32_t now_us = raw.timestamp;
    if (_last_imu_time_us != 0) {
        const uint32_t dt_us = now_us - _last_imu_time_us;
        float dt = (float)dt_us * 1e-6f;
        if (dt > 0 && dt < 1.0f) { // Sanity check
            ekfPredict(dt);
        }
    }
    _last_imu_time_us = now_us;
}

void NAV::updatePress(const Raw_press &p) { 
    _press = p; 
    ekfUpdateBaro();
}

void NAV::updateMag(const Raw_mag &m) { 
    _mag = m; 
    if (_init_avg_active) {
        _init_mag_sum[0] += _mag.mx;
        _init_mag_sum[1] += _mag.my;
        _init_mag_sum[2] += _mag.mz;
        if (_init_mag_count < 65535) _init_mag_count++;
    }
    ekfUpdateMag();
}

void NAV::updateGps(const Raw_gps &g) {
    _gps = g;
    if (_ekf_ready && _gps.hasPos) {
        _ekf.updateGps(_gps.pn, _gps.pe, _gps.pd,
                       _gps.vn, _gps.ve, _gps.vd,
                       _gps.hAcc, _gps.vAcc, _gps.sAcc);
        syncNominal();
    }
}

// Internal: sync nominal from filter

void NAV::syncNominal() {
    const float* p  = _ekf.position();
    const float* v  = _ekf.velocity();
    const float* q  = _ekf.quaternion();
    const float* ba = _ekf.accelBias();
    const float* bg = _ekf.gyroBias();
    _nominal.p[0]  = p[0];   _nominal.p[1]  = p[1];   _nominal.p[2]  = p[2];
    _nominal.v[0]  = v[0];   _nominal.v[1]  = v[1];   _nominal.v[2]  = v[2];
    _nominal.q[0]  = q[0];   _nominal.q[1]  = q[1];
    _nominal.q[2]  = q[2];   _nominal.q[3]  = q[3];
    _nominal.ba[0] = ba[0];  _nominal.ba[1] = ba[1];  _nominal.ba[2] = ba[2];
    _nominal.bg[0] = bg[0];  _nominal.bg[1] = bg[1];  _nominal.bg[2] = bg[2];
}

// =============================================================================
// ES-EKF 생명주기
// =============================================================================

// ----------------------------------------------------------------------------
// ekfBegin() : 즉시 TRIAD 정렬 (현재 1샘플의 IMU+Mag 사용)
//   - 노이즈에 민감하므로 보통 ekfBeginAveraged()를 권장
//   - GPS hasPos이면 origin NED 좌표/속도로 초기화, 아니면 (0,0,0)
// ----------------------------------------------------------------------------
bool NAV::ekfBegin() {
    const float a[3] = { _state_imu.ax, _state_imu.ay, _state_imu.az };
    const float m[3] = { _mag.mx,        _mag.my,        _mag.mz       };

    const float a_n = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    const float m_n = sqrtf(m[0]*m[0] + m[1]*m[1] + m[2]*m[2]);
    if (a_n < 1.0f || m_n < 1e-6f) return false;   // bogus IMU/MAG

    float q0[4];
    if (_launch_site_set) {
        ESEKF::triad(a, m, q0, _launch_lat_deg, _launch_lon_deg);
    } else {
        ESEKF::triad(a, m, q0);
    }

    float p0[3] = {0.0f, 0.0f, 0.0f};
    float v0[3] = {0.0f, 0.0f, 0.0f};
    if (_gps.hasPos) {
        p0[0] = _gps.pn; p0[1] = _gps.pe; p0[2] = _gps.pd;
        v0[0] = _gps.vn; v0[1] = _gps.ve; v0[2] = _gps.vd;
    }

    _ekf.init(p0, v0, q0);
    if (_launch_site_set) {
        _ekf.setMagReferenceByLocation(_launch_lat_deg, _launch_lon_deg);
    }
    _ekf_ready = true;
    syncNominal();
    return true;
}

// ----------------------------------------------------------------------------
// ekfBeginAveraged() : START 명령에서 사용
//   - START_TRIAD_AVG_MS 동안 누적된 _init_acc_sum / _init_mag_sum 평균으로 TRIAD
//   - 평균 샘플이 부족하면 false (보통 minImu=100, minMag=5)
//   - 노이즈 평균화로 초기 자세 오차를 크게 줄임
// ----------------------------------------------------------------------------
bool NAV::ekfBeginAveraged(uint16_t minImuSamples, uint16_t minMagSamples) {
    if (_init_imu_count < minImuSamples || _init_mag_count < minMagSamples) {
        _init_avg_active = false;
        return false;
    }

    const float inv_imu = 1.0f / (float)_init_imu_count;
    const float inv_mag = 1.0f / (float)_init_mag_count;
    const float a[3] = {
        _init_acc_sum[0] * inv_imu,
        _init_acc_sum[1] * inv_imu,
        _init_acc_sum[2] * inv_imu
    };
    const float m[3] = {
        _init_mag_sum[0] * inv_mag,
        _init_mag_sum[1] * inv_mag,
        _init_mag_sum[2] * inv_mag
    };

    _init_avg_active = false;

    const float a_n = sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    const float m_n = sqrtf(m[0]*m[0] + m[1]*m[1] + m[2]*m[2]);
    if (a_n < 1.0f || m_n < 1e-6f) return false;

    float q0[4];
    if (_launch_site_set) {
        ESEKF::triad(a, m, q0, _launch_lat_deg, _launch_lon_deg);
    } else {
        ESEKF::triad(a, m, q0);
    }

    float p0[3] = {0.0f, 0.0f, 0.0f};
    float v0[3] = {0.0f, 0.0f, 0.0f};
    if (_gps.hasPos) {
        p0[0] = _gps.pn; p0[1] = _gps.pe; p0[2] = _gps.pd;
        v0[0] = _gps.vn; v0[1] = _gps.ve; v0[2] = _gps.vd;
    }

    _ekf.init(p0, v0, q0);
    if (_launch_site_set) {
        _ekf.setMagReferenceByLocation(_launch_lat_deg, _launch_lon_deg);
    }
    _ekf_ready = true;
    syncNominal();
    return true;
}

void NAV::ekfBegin(const float p0[3], const float v0[3], const float q0[4]) {
    _ekf.init(p0, v0, q0);
    if (_launch_site_set) {
        _ekf.setMagReferenceByLocation(_launch_lat_deg, _launch_lon_deg);
    }
    _ekf_ready = true;
    syncNominal();
}

void NAV::ekfReset() {
    _ekf_ready = false;
    _init_avg_active = false;
    _ekf.reset();
    _nominal = State_nominal{};
    _nominal.q[0] = 1.0f;
}

void NAV::resetInitAverage() {
    _init_avg_active = true;
    _init_imu_count = 0;
    _init_mag_count = 0;
    _init_acc_sum[0] = 0.0f; _init_acc_sum[1] = 0.0f; _init_acc_sum[2] = 0.0f;
    _init_mag_sum[0] = 0.0f; _init_mag_sum[1] = 0.0f; _init_mag_sum[2] = 0.0f;
}

void NAV::setLaunchSite(float lat_deg, float lon_deg) {
    if (isnan(lat_deg) || isnan(lon_deg)) return;
    _launch_lat_deg = lat_deg;
    _launch_lon_deg = lon_deg;
    _launch_site_set = true;
    _ekf.setMagReferenceByLocation(lat_deg, lon_deg);
}

// =============================================================================
// ES-EKF 실시간 사이클 (predict + 측정 업데이트 래퍼)
//   - 각 함수는 _ekf_ready 체크 후 호출 → 정렬 전엔 무시
//   - 호출 후 syncNominal()로 외부 노출용 _nominal 구조체 갱신
// =============================================================================

// ----------------------------------------------------------------------------
// ekfPredict() : IMU 입력으로 ES-EKF predict (적응형 Q 사용)
// ----------------------------------------------------------------------------
void NAV::ekfPredict(float dt) {
    _nominal.timestamp = _state_imu.timestamp;
    if (!_ekf_ready) return;
    const float a[3] = { _state_imu.ax, _state_imu.ay, _state_imu.az };
    const float w[3] = { _state_imu.gx, _state_imu.gy, _state_imu.gz };
    _ekf.predictAdaptiveJerk(a, w, dt);
    syncNominal();
}

void NAV::ekfUpdateBaro() {
    if (!_ekf_ready) return;
    _ekf.updateBaro(_press.alt);
    syncNominal();
}

void NAV::ekfUpdateMag() {
    if (!_ekf_ready) return;
    const float m[3] = { _mag.mx, _mag.my, _mag.mz };
    _ekf.updateMag(m);
    syncNominal();
}

// ----------------------------------------------------------------------------
// ekfUpdateStaticAlignment() : 정지 상태 4종 의사측정 한꺼번에 적용
//   1) ZUPT     : 속도=0 강제
//   2) AccStatic: a=R*(-g)+ba 로 자세 + 가속 바이어스 보정
//   3) GyroStatic: w=bg 로 자이로 바이어스 직접 보정
//   4) Mag      : yaw 보정
//   - 발사대 위 정지 정렬(ZUPT 루프)에서만 호출 (비행 중에는 절대 호출 X)
// ----------------------------------------------------------------------------
void NAV::ekfUpdateStaticAlignment() {
    if (!_ekf_ready) return;
    
    _ekf.updateZupt();
    
    const float a[3] = { _state_imu.ax, _state_imu.ay, _state_imu.az };
    _ekf.updateAccStatic(a);
    
    const float w[3] = { _state_imu.gx, _state_imu.gy, _state_imu.gz };
    _ekf.updateGyroStatic(w);
    
    const float m[3] = { _mag.mx, _mag.my, _mag.mz };
    _ekf.updateMag(m);
    
    syncNominal();
}

void NAV::ekfUpdateGps() {
    if (!_ekf_ready || !_gps.hasPos) return;
    _ekf.updateGps(_gps.pn, _gps.pe, _gps.pd,
                   _gps.vn, _gps.ve, _gps.vd,
                   _gps.hAcc, _gps.vAcc, _gps.sAcc);
    syncNominal();
}
