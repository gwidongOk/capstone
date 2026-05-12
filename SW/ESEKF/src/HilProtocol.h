#ifndef HIL_PROTOCOL_H
#define HIL_PROTOCOL_H

#include <Arduino.h>
#include "ESEKF.h"

// ──────────────────────────────────────────────
// HIL Frame (binary, little-endian floats)
//   [0xAA][0x55][ID:u8][LEN:u16][PAYLOAD][CRC16:u16]
//   CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF) over ID|LEN|PAYLOAD
//
// Host → MCU:
//   0x01 RESET             ()
//   0x02 INIT              (p[3], v[3], q[4])                       40B
//   0x03 TRIAD             (acc[3], mag[3], lat, lon)               32B  (lat/lon = NaN 가능)
//   0x10 PREDICT           (a_m[3], w_m[3], dt)                     28B
//   0x20 UPDATE_GPS        (p[3], v[3], hAcc, vAcc)                 32B
//   0x21 UPDATE_BARO       (alt)                                     4B
//   0x22 UPDATE_MAG        (m[3])                                   12B
//   0x23 UPDATE_ZUPT       ()
//   0x24 UPDATE_ACC_STATIC (a[3])                                   12B
//   0x25 UPDATE_GYRO_STATIC(w[3])                                   12B
//   0x30 GET_STATE         ()
//   0x31 GET_COV_DIAG      ()
//
// MCU → Host:
//   0x80 STATE             (p[3], v[3], q[4], ba[3], bg[3])         64B   ← 모든 명령 응답 (TRIAD 제외)
//   0x81 COV_DIAG          (diag P[15])                             60B
//   0x82 TRIAD_Q           (q[4])                                   16B
//   0xFF NACK              (orig_id, err_code)                       2B
// ──────────────────────────────────────────────

namespace Hil {

constexpr uint8_t SYNC0 = 0xAA;
constexpr uint8_t SYNC1 = 0x55;
constexpr size_t  MAX_PAYLOAD = 128;

// commands
constexpr uint8_t CMD_RESET              = 0x01;
constexpr uint8_t CMD_INIT               = 0x02;
constexpr uint8_t CMD_TRIAD              = 0x03;
constexpr uint8_t CMD_PREDICT            = 0x10;
constexpr uint8_t CMD_UPDATE_GPS         = 0x20;
constexpr uint8_t CMD_UPDATE_BARO        = 0x21;
constexpr uint8_t CMD_UPDATE_MAG         = 0x22;
constexpr uint8_t CMD_UPDATE_ZUPT        = 0x23;
constexpr uint8_t CMD_UPDATE_ACC_STATIC  = 0x24;
constexpr uint8_t CMD_UPDATE_GYRO_STATIC = 0x25;
constexpr uint8_t CMD_GET_STATE          = 0x30;
constexpr uint8_t CMD_GET_COV_DIAG       = 0x31;

// responses
constexpr uint8_t RSP_STATE    = 0x80;
constexpr uint8_t RSP_COV_DIAG = 0x81;
constexpr uint8_t RSP_TRIAD    = 0x82;
constexpr uint8_t RSP_NACK     = 0xFF;

// nack error codes
constexpr uint8_t ERR_BAD_LEN  = 0x01;
constexpr uint8_t ERR_UNKNOWN  = 0x02;

uint16_t crc16(const uint8_t* data, size_t len);

class Framer {
public:
    Framer();
    void reset();
    // 한 바이트 투입 → 완성 시 true 반환 (id, payload, len 채워짐)
    bool feed(uint8_t b, uint8_t& outId, uint8_t* outPayload, size_t& outLen);

private:
    enum State : uint8_t {
        S_SYNC0, S_SYNC1, S_ID, S_LEN_LO, S_LEN_HI,
        S_PAYLOAD, S_CRC_LO, S_CRC_HI
    };
    State    _state;
    uint8_t  _id;
    uint16_t _len;
    uint16_t _idx;
    uint16_t _crc_rx;
    uint8_t  _buf[MAX_PAYLOAD];
};

void sendFrame(Stream& s, uint8_t id, const uint8_t* payload, size_t len);
void handle(ESEKF& ekf, uint8_t id, const uint8_t* payload, size_t len, Stream& out);

} // namespace Hil

#endif // HIL_PROTOCOL_H
