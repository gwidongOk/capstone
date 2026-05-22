#include "ESEKF.h"

using namespace Eigen;

ESEKF::ESEKF() { reset(); }

void ESEKF::reset() {
    _p.setZero();
    _v.setZero();
    _q.setIdentity();
    _ba.setZero();
    _bg.setZero();
    syncQuatRaw();

    _g_ned     << 0.0f, 0.0f, G_VAL;
    _m_ref_ned << 0.5961f, -0.0838f, 0.7986f;   // Korea ref
    _last_accel_mag = 0.0f;

    // Reset adaptive Q window
    for (int i = 0; i < AQ_WIN; i++) _aq_win[i] = 0.0f;
    _aq_idx  = 0;
    _aq_full = false;
    _aq_var  = VAR_ACC;

    _covP.setZero();
    _covP.diagonal().segment<3>(0).array()  = P0_POS * P0_POS;
    _covP.diagonal().segment<3>(3).array()  = P0_VEL * P0_VEL;
    _covP.diagonal().segment<3>(6).array()  = P0_ATT * P0_ATT;
    _covP.diagonal().segment<3>(9).array()  = P0_BA  * P0_BA;
    _covP.diagonal().segment<3>(12).array() = P0_BG  * P0_BG;
}

void ESEKF::init(const float p0[3], const float v0[3], const float q0[4]) {
    _p = Vector3f(p0[0], p0[1], p0[2]);
    _v = Vector3f(v0[0], v0[1], v0[2]);
    _q = Quaternionf(q0[0], q0[1], q0[2], q0[3]); // [w,x,y,z]
    _q.normalize();
    syncQuatRaw();
}

// ──────────────────────────────────────────────
// predict
// ──────────────────────────────────────────────
void ESEKF::predict(const float a_m[3], const float w_m[3], float dt) {
    Vector3f am(a_m[0], a_m[1], a_m[2]);
    Vector3f wm(w_m[0], w_m[1], w_m[2]);

    Vector3f a_hat = am - _ba;
    Vector3f w_hat = wm - _bg;
    _last_accel_mag = a_hat.norm();

    // ── Adaptive Q: update rolling window with bias-corrected specific force ──
    _aq_win[_aq_idx] = _last_accel_mag;
    _aq_idx = (_aq_idx + 1) % AQ_WIN;
    if (_aq_idx == 0) _aq_full = true;
    {
        int n = _aq_full ? AQ_WIN : _aq_idx;
        if (n >= 2) {
            float s = 0.0f, sq = 0.0f;
            for (int i = 0; i < n; i++) { s += _aq_win[i]; sq += _aq_win[i]*_aq_win[i]; }
            float mean = s / n;
            float var  = sq / n - mean * mean;
            _aq_var = (var > VAR_ACC) ? var : VAR_ACC;
        }
    }
    float q_acc_scale = _aq_var / VAR_ACC;
    if (q_acc_scale > AQ_SCALE_MAX) q_acc_scale = AQ_SCALE_MAX;

    Matrix3f R_nb = _q.toRotationMatrix();
    Vector3f a_ned = R_nb * a_hat + _g_ned;

    // nominal propagate
    _p += _v * dt + 0.5f * a_ned * dt * dt;
    _v += a_ned * dt;

    Vector3f theta = w_hat * dt;
    float th_n = theta.norm();
    Quaternionf dq;
    if (th_n > 1e-10f) {
        float s = sinf(th_n * 0.5f) / th_n;
        dq.w() = cosf(th_n * 0.5f);
        dq.x() = s * theta.x();
        dq.y() = s * theta.y();
        dq.z() = s * theta.z();
    } else {
        dq.w() = 1.0f;
        dq.x() = theta.x() * 0.5f;
        dq.y() = theta.y() * 0.5f;
        dq.z() = theta.z() * 0.5f;
    }
    _q = (_q * dq).normalized();
    syncQuatRaw();

    // continuous Fc -> discrete F (Taylor 3rd)
    Matrix<float, 15, 15> Fc = Matrix<float, 15, 15>::Zero();
    Fc.block<3,3>(0, 3)  =  Matrix3f::Identity();
    Fc.block<3,3>(3, 6)  = -R_nb * skew(a_hat);
    Fc.block<3,3>(3, 9)  = -R_nb;
    Fc.block<3,3>(6, 6)  = -skew(w_hat);
    Fc.block<3,3>(6, 12) = -Matrix3f::Identity();

    Matrix<float, 15, 15> Fdt  = Fc * dt;
    Matrix<float, 15, 15> Fdt2 = Fdt * Fdt;
    Matrix<float, 15, 15> Fdt3 = Fdt2 * Fdt;
    Matrix<float, 15, 15> F    = Matrix<float, 15, 15>::Identity()
                               + Fdt + 0.5f * Fdt2 + (1.0f / 6.0f) * Fdt3;

    // discrete Q — velocity/attitude rows scale with adaptive q_acc_scale
    Matrix<float, 15, 15> Q = Matrix<float, 15, 15>::Zero();
    Q.diagonal().segment<3>(3).array()  = VAR_ACC  * q_acc_scale * dt;  // adaptive
    Q.diagonal().segment<3>(6).array()  = VAR_GYRO * q_acc_scale * dt;  // adaptive
    Q.diagonal().segment<3>(9).array()  = VAR_BA   * dt;
    Q.diagonal().segment<3>(12).array() = VAR_BG   * dt;

    _covP = F * _covP * F.transpose() + Q;
    _covP = 0.5f * (_covP + _covP.transpose()).eval();
}

// ──────────────────────────────────────────────
// GPS update (dynamic R: 3x multiplier + 4g penalty)
// ──────────────────────────────────────────────
void ESEKF::updateGps(const float p_meas[3], const float v_meas[3], float hAcc, float vAcc) {
    Matrix<float, 6, 15> H = Matrix<float, 6, 15>::Zero();
    H.block<3,3>(0, 0) = Matrix3f::Identity();
    H.block<3,3>(3, 3) = Matrix3f::Identity();

    Matrix<float, 6, 1> y;
    y << p_meas[0] - _p.x(), p_meas[1] - _p.y(), p_meas[2] - _p.z(),
         v_meas[0] - _v.x(), v_meas[1] - _v.y(), v_meas[2] - _v.z();

    const float mult     = 3.0f;
    const float G4_LIMIT = 4.0f * G_VAL;
    float pen = 1.0f;
    if (_last_accel_mag > G4_LIMIT) {
        float exc = _last_accel_mag - G4_LIMIT;
        pen = 1.0f + exc * exc * 10.0f;
        if (pen > 1000.0f) pen = 1000.0f;
    }
    float fm    = mult * pen;
    float var_h = (hAcc * fm) * (hAcc * fm);
    float var_v = (vAcc * fm) * (vAcc * fm);
    float var_vh = var_h * 0.2f;
    float var_vv = var_v * 0.2f;

    Matrix<float, 6, 6> R = Matrix<float, 6, 6>::Zero();
    R.diagonal() << var_h, var_h, var_v, var_vh, var_vh, var_vv;

    measurementUpdate<6>(H, y, R);
}

// ──────────────────────────────────────────────
// Baro update (h = -pD)
// ──────────────────────────────────────────────
void ESEKF::updateBaro(float alt) {
    Matrix<float, 1, 15> H = Matrix<float, 1, 15>::Zero();
    H(0, 2) = -1.0f;
    Matrix<float, 1, 1> y; y(0) = alt - (-_p.z());
    Matrix<float, 1, 1> R; R(0,0) = VAR_BARO;
    measurementUpdate<1>(H, y, R);
}

// ──────────────────────────────────────────────
// Mag update (unit vector)
// ──────────────────────────────────────────────
void ESEKF::updateMag(const float m_body[3]) {
    Vector3f mb(m_body[0], m_body[1], m_body[2]);
    if (mb.norm() < 1e-4f) return;
    mb.normalize();

    Matrix3f R_nb  = _q.toRotationMatrix();
    Vector3f mref  = _m_ref_ned / _m_ref_ned.norm();
    Vector3f zhat  = R_nb.transpose() * mref;
    Matrix<float, 3, 1> y = mb - zhat;

    Matrix<float, 3, 15> H = Matrix<float, 3, 15>::Zero();
    H.block<3,3>(0, 6) = skew(zhat);

    Matrix<float, 3, 3> R = Matrix3f::Identity() * VAR_MAG;
    measurementUpdate<3>(H, y, R);
}

// ──────────────────────────────────────────────
// ZUPT
// ──────────────────────────────────────────────
void ESEKF::updateZupt() {
    Matrix<float, 3, 15> H = Matrix<float, 3, 15>::Zero();
    H.block<3,3>(0, 3) = Matrix3f::Identity();
    Matrix<float, 3, 1> y = -_v;
    Matrix<float, 3, 3> R = Matrix3f::Identity() * 1e-4f;
    measurementUpdate<3>(H, y, R);
}

// ──────────────────────────────────────────────
// Inclinometer (정지 acc)
// ──────────────────────────────────────────────
void ESEKF::updateAccStatic(const float a_m[3]) {
    Vector3f am(a_m[0], a_m[1], a_m[2]);
    Matrix3f R_nb  = _q.toRotationMatrix();
    Vector3f g_b   = R_nb.transpose() * (-_g_ned);
    Vector3f zhat  = g_b + _ba;
    Matrix<float, 3, 1> y = am - zhat;

    Matrix<float, 3, 15> H = Matrix<float, 3, 15>::Zero();
    H.block<3,3>(0, 6) = skew(g_b);
    H.block<3,3>(0, 9) = Matrix3f::Identity();

    Matrix<float, 3, 3> R = Matrix3f::Identity() * (VAR_ACC / DT_IMU);
    measurementUpdate<3>(H, y, R);
}

// ──────────────────────────────────────────────
// ZARU (정지 gyro)
// ──────────────────────────────────────────────
void ESEKF::updateGyroStatic(const float w_m[3]) {
    Vector3f wm(w_m[0], w_m[1], w_m[2]);
    Matrix<float, 3, 1> y = wm - _bg;

    Matrix<float, 3, 15> H = Matrix<float, 3, 15>::Zero();
    H.block<3,3>(0, 12) = Matrix3f::Identity();

    Matrix<float, 3, 3> R = Matrix3f::Identity() * (VAR_GYRO / DT_IMU);
    measurementUpdate<3>(H, y, R);
}

// ──────────────────────────────────────────────
// TRIAD
// ──────────────────────────────────────────────
void ESEKF::triad(const float acc[3], const float mag[3],
                  float q_out[4], float lat_deg, float lon_deg)
{
    Vector3f a(acc[0], acc[1], acc[2]);
    Vector3f m(mag[0], mag[1], mag[2]);

    Vector3f r1(0.0f, 0.0f, 1.0f);
    Vector3f r2(0.5961f, -0.0838f, 0.7986f);
    if (!isnan(lat_deg) && !isnan(lon_deg)) {
        float D, I; wmmKorea(lat_deg, lon_deg, D, I);
        const float DEG = 0.017453292519943295f;
        float cD = cosf(D * DEG), sD = sinf(D * DEG);
        float cI = cosf(I * DEG), sI = sinf(I * DEG);
        r2 << cI * cD, cI * sD, sI;
    }

    Vector3f b1 = -a / a.norm();
    Vector3f b2 =  m / m.norm();

    Vector3f u1 = r1;
    Vector3f u2 = u1.cross(r2).normalized();
    Vector3f u3 = u1.cross(u2);
    Matrix3f Mref;  Mref  << u1, u2, u3;

    Vector3f v1 = b1;
    Vector3f v2 = v1.cross(b2).normalized();
    Vector3f v3 = v1.cross(v2);
    Matrix3f Mbody; Mbody << v1, v2, v3;

    Matrix3f C_bn = Mref * Mbody.transpose();
    Quaternionf q(C_bn);
    if (q.w() < 0.0f) {
        q.w() = -q.w(); q.x() = -q.x();
        q.y() = -q.y(); q.z() = -q.z();
    }
    q.normalize();
    q_out[0] = q.w();
    q_out[1] = q.x();
    q_out[2] = q.y();
    q_out[3] = q.z();
}

// ──────────────────────────────────────────────
// helpers
// ──────────────────────────────────────────────
void ESEKF::covDiag(float out[15]) const {
    for (int i = 0; i < 15; ++i) out[i] = _covP(i, i);
}

void ESEKF::syncQuatRaw() {
    _q_raw[0] = _q.w();
    _q_raw[1] = _q.x();
    _q_raw[2] = _q.y();
    _q_raw[3] = _q.z();
}

Matrix3f ESEKF::skew(const Vector3f& v) {
    Matrix3f S;
    S <<   0.0f, -v.z(),  v.y(),
          v.z(),   0.0f, -v.x(),
         -v.y(),  v.x(),   0.0f;
    return S;
}

void ESEKF::wmmKorea(float lat, float lon, float& D, float& I) {
    D = -7.9f - 0.35f * (lat - 36.0f) + 0.15f * (lon - 127.0f);
    I = 51.0f + 1.40f * (lat - 36.0f) + 0.05f * (lon - 127.0f);
}
