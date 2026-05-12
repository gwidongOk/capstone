#include "HilProtocol.h"
#include <string.h>

namespace Hil {

// ──────────────────────────────────────────────
// CRC16-CCITT-FALSE
// ──────────────────────────────────────────────
uint16_t crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= ((uint16_t)data[i]) << 8;
        for (int j = 0; j < 8; ++j) {
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
        }
    }
    return crc;
}

// ──────────────────────────────────────────────
// Framer
// ──────────────────────────────────────────────
Framer::Framer() { reset(); }

void Framer::reset() {
    _state = S_SYNC0;
    _idx   = 0;
    _len   = 0;
}

bool Framer::feed(uint8_t b, uint8_t& outId, uint8_t* outPayload, size_t& outLen) {
    switch (_state) {
    case S_SYNC0:
        if (b == SYNC0) _state = S_SYNC1;
        return false;

    case S_SYNC1:
        if      (b == SYNC1) _state = S_ID;
        else if (b != SYNC0) _state = S_SYNC0;
        return false;

    case S_ID:
        _id = b;
        _state = S_LEN_LO;
        return false;

    case S_LEN_LO:
        _len = b;
        _state = S_LEN_HI;
        return false;

    case S_LEN_HI:
        _len |= ((uint16_t)b) << 8;
        if (_len > MAX_PAYLOAD) { reset(); return false; }
        _idx = 0;
        _state = (_len == 0) ? S_CRC_LO : S_PAYLOAD;
        return false;

    case S_PAYLOAD:
        _buf[_idx++] = b;
        if (_idx >= _len) _state = S_CRC_LO;
        return false;

    case S_CRC_LO:
        _crc_rx = b;
        _state = S_CRC_HI;
        return false;

    case S_CRC_HI: {
        _crc_rx |= ((uint16_t)b) << 8;
        // CRC over ID|LEN|PAYLOAD
        uint8_t tmp[3 + MAX_PAYLOAD];
        tmp[0] = _id;
        tmp[1] = (uint8_t)(_len & 0xFF);
        tmp[2] = (uint8_t)((_len >> 8) & 0xFF);
        if (_len) memcpy(tmp + 3, _buf, _len);
        uint16_t crc = crc16(tmp, 3 + _len);

        bool ok = (crc == _crc_rx);
        if (ok) {
            outId  = _id;
            outLen = _len;
            if (_len) memcpy(outPayload, _buf, _len);
        }
        reset();
        return ok;
    }
    }
    return false;
}

// ──────────────────────────────────────────────
// sendFrame
// ──────────────────────────────────────────────
void sendFrame(Stream& s, uint8_t id, const uint8_t* payload, size_t len) {
    uint8_t tmp[3 + MAX_PAYLOAD];
    tmp[0] = id;
    tmp[1] = (uint8_t)(len & 0xFF);
    tmp[2] = (uint8_t)((len >> 8) & 0xFF);
    if (len) memcpy(tmp + 3, payload, len);
    uint16_t crc = crc16(tmp, 3 + len);

    s.write(SYNC0);
    s.write(SYNC1);
    s.write(tmp, 3 + len);
    s.write((uint8_t)(crc & 0xFF));
    s.write((uint8_t)((crc >> 8) & 0xFF));
}

// ──────────────────────────────────────────────
// helpers
// ──────────────────────────────────────────────
static void sendState(ESEKF& ekf, Stream& out) {
    float buf[16];
    memcpy(&buf[0],  ekf.p(),  12);
    memcpy(&buf[3],  ekf.v(),  12);
    memcpy(&buf[6],  ekf.q(),  16);
    memcpy(&buf[10], ekf.ba(), 12);
    memcpy(&buf[13], ekf.bg(), 12);
    sendFrame(out, RSP_STATE, (const uint8_t*)buf, sizeof(buf));
}

static void sendNack(Stream& out, uint8_t origId, uint8_t err) {
    uint8_t pl[2] = { origId, err };
    sendFrame(out, RSP_NACK, pl, 2);
}

static inline bool checkLen(Stream& out, size_t got, size_t need, uint8_t origId) {
    if (got >= need) return true;
    sendNack(out, origId, ERR_BAD_LEN);
    return false;
}

// ──────────────────────────────────────────────
// handle
// ──────────────────────────────────────────────
void handle(ESEKF& ekf, uint8_t id, const uint8_t* payload, size_t len, Stream& out) {
    switch (id) {

    case CMD_RESET:
        ekf.reset();
        sendState(ekf, out);
        break;

    case CMD_INIT: {
        if (!checkLen(out, len, 40, id)) return;
        float p0[3], v0[3], q0[4];
        memcpy(p0, payload +  0, 12);
        memcpy(v0, payload + 12, 12);
        memcpy(q0, payload + 24, 16);
        ekf.init(p0, v0, q0);
        sendState(ekf, out);
        break;
    }

    case CMD_TRIAD: {
        if (!checkLen(out, len, 32, id)) return;
        float acc[3], mag[3], lat, lon;
        memcpy(acc, payload +  0, 12);
        memcpy(mag, payload + 12, 12);
        memcpy(&lat, payload + 24, 4);
        memcpy(&lon, payload + 28, 4);
        float q[4];
        ESEKF::triad(acc, mag, q, lat, lon);
        sendFrame(out, RSP_TRIAD, (const uint8_t*)q, sizeof(q));
        break;
    }

    case CMD_PREDICT: {
        if (!checkLen(out, len, 28, id)) return;
        float am[3], wm[3], dt;
        memcpy(am, payload +  0, 12);
        memcpy(wm, payload + 12, 12);
        memcpy(&dt, payload + 24, 4);
        ekf.predict(am, wm, dt);
        sendState(ekf, out);
        break;
    }

    case CMD_UPDATE_GPS: {
        if (!checkLen(out, len, 32, id)) return;
        float pm[3], vm[3], hAcc, vAcc;
        memcpy(pm, payload +  0, 12);
        memcpy(vm, payload + 12, 12);
        memcpy(&hAcc, payload + 24, 4);
        memcpy(&vAcc, payload + 28, 4);
        ekf.updateGps(pm, vm, hAcc, vAcc);
        sendState(ekf, out);
        break;
    }

    case CMD_UPDATE_BARO: {
        if (!checkLen(out, len, 4, id)) return;
        float alt;
        memcpy(&alt, payload, 4);
        ekf.updateBaro(alt);
        sendState(ekf, out);
        break;
    }

    case CMD_UPDATE_MAG: {
        if (!checkLen(out, len, 12, id)) return;
        float m[3];
        memcpy(m, payload, 12);
        ekf.updateMag(m);
        sendState(ekf, out);
        break;
    }

    case CMD_UPDATE_ZUPT:
        ekf.updateZupt();
        sendState(ekf, out);
        break;

    case CMD_UPDATE_ACC_STATIC: {
        if (!checkLen(out, len, 12, id)) return;
        float a[3];
        memcpy(a, payload, 12);
        ekf.updateAccStatic(a);
        sendState(ekf, out);
        break;
    }

    case CMD_UPDATE_GYRO_STATIC: {
        if (!checkLen(out, len, 12, id)) return;
        float w[3];
        memcpy(w, payload, 12);
        ekf.updateGyroStatic(w);
        sendState(ekf, out);
        break;
    }

    case CMD_GET_STATE:
        sendState(ekf, out);
        break;

    case CMD_GET_COV_DIAG: {
        float d[15];
        ekf.covDiag(d);
        sendFrame(out, RSP_COV_DIAG, (const uint8_t*)d, sizeof(d));
        break;
    }

    default:
        sendNack(out, id, ERR_UNKNOWN);
        break;
    }
}

} // namespace Hil
