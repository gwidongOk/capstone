#include <Arduino.h>
#include "ESEKF.h"

// loop task 스택 증설 — predict()의 15x15 Eigen 행렬이 기본 8KB 초과
SET_LOOP_TASK_STACK_SIZE(24 * 1024);

// ──────────────────────────────────────────────
// HIL Serial Protocol (lock-step + seq dedup + seq echo)
//
//   sync = 0xA5
//   TX (host → mcu): [0xA5][CMD][SEQ:u8][payload]
//   RX (mcu → host) — 응답에도 SEQ echo (MATLAB이 검증해서 false sync 차단)
//     state 응답 (70B): [0xA5][SEQ][p(12) v(12) q(16) ba(12) bg(12) us(4)]
//     TRIAD 응답 (22B): [0xA5][SEQ][q(16) us(4)]
//
//   같은 SEQ 가 다시 오면 → 캐시된 응답 재전송 (중복 처리 방지).
// ──────────────────────────────────────────────

static ESEKF ekf;
constexpr uint8_t SYNC = 0xA5;

// ── 응답 캐시 + 시퀀스 추적 ────────────────────────
static uint8_t last_seq;
static bool    has_cache    = false;
static uint8_t last_resp[70];
static size_t  last_resp_len = 0;

static bool readN(void* dst, size_t n) {
    uint8_t* p = (uint8_t*)dst;
    size_t got = 0;
    uint32_t last_byte_ms = millis();
    while (got < n) {
        if (Serial.available()) {
            p[got++] = (uint8_t)Serial.read();
            last_byte_ms = millis();
        } else if (millis() - last_byte_ms > 1000) {
            return false;
        } else {
            delay(1);
        }
    }
    return true;
}

// state 응답: [SYNC][SEQ][64 byte state][4 byte us] = 70B
static void cacheState(uint32_t us, uint8_t seq) {
    last_resp[0] = SYNC;
    last_resp[1] = seq;
    memcpy(last_resp +  2, ekf.p(),  12);
    memcpy(last_resp + 14, ekf.v(),  12);
    memcpy(last_resp + 26, ekf.q(),  16);
    memcpy(last_resp + 42, ekf.ba(), 12);
    memcpy(last_resp + 54, ekf.bg(), 12);
    memcpy(last_resp + 66, &us,       4);
    last_resp_len = 70;
    has_cache = true;
}

// TRIAD 응답: [SYNC][SEQ][16 byte q][4 byte us] = 22B
static void cacheTriad(const float q[4], uint32_t us, uint8_t seq) {
    last_resp[0] = SYNC;
    last_resp[1] = seq;
    memcpy(last_resp +  2, q,    16);
    memcpy(last_resp + 18, &us,   4);
    last_resp_len = 22;
    has_cache = true;
}

static inline void sendCached() {
    Serial.write(last_resp, last_resp_len);
}

void setup() {
    Serial.setRxBufferSize(4096);
    Serial.setTxBufferSize(4096);
    Serial.begin(921600);
    while (!Serial && millis() < 3000) { delay(10); }
    delay(200);
}

void loop() {
    if (!Serial.available()) { delay(0); return; }
    if ((uint8_t)Serial.read() != SYNC) return;

    uint8_t cmd, seq;
    if (!readN(&cmd, 1)) return;
    if (!readN(&seq, 1)) return;

    bool dup = has_cache && (seq == last_seq);
    uint32_t t0, us;

    switch (cmd) {

    case 0x01:  // RESET
        if (!dup) {
            ekf.reset();
            cacheState(0, seq);
            last_seq = seq;
        }
        sendCached();
        break;

    case 0x02: {  // INIT
        float p[3], v[3], q[4];
        if (!readN(p, 12) || !readN(v, 12) || !readN(q, 16)) return;
        if (!dup) {
            ekf.init(p, v, q);
            cacheState(0, seq);
            last_seq = seq;
        }
        sendCached();
        break;
    }

    case 0x03: {  // TRIAD
        float acc[3], mag[3], lat, lon, q_out[4];
        if (!readN(acc, 12) || !readN(mag, 12) || !readN(&lat, 4) || !readN(&lon, 4)) return;
        if (!dup) {
            t0 = micros();
            ESEKF::triad(acc, mag, q_out, lat, lon);
            us = micros() - t0;
            cacheTriad(q_out, us, seq);
            last_seq = seq;
        }
        sendCached();
        break;
    }

    case 0x10: {  // PREDICT
        float a[3], w[3], dt;
        if (!readN(a, 12) || !readN(w, 12) || !readN(&dt, 4)) return;
        if (!dup) {
            t0 = micros();
            ekf.predict(a, w, dt);
            us = micros() - t0;
            cacheState(us, seq);
            last_seq = seq;
        }
        sendCached();
        break;
    }

    case 0x20: {  // UPDATE_GPS
        float p[3], v[3], hAcc, vAcc;
        if (!readN(p, 12) || !readN(v, 12) || !readN(&hAcc, 4) || !readN(&vAcc, 4)) return;
        if (!dup) {
            t0 = micros();
            ekf.updateGps(p, v, hAcc, vAcc);
            us = micros() - t0;
            cacheState(us, seq);
            last_seq = seq;
        }
        sendCached();
        break;
    }

    case 0x21: {  // UPDATE_BARO
        float alt;
        if (!readN(&alt, 4)) return;
        if (!dup) {
            t0 = micros();
            ekf.updateBaro(alt);
            us = micros() - t0;
            cacheState(us, seq);
            last_seq = seq;
        }
        sendCached();
        break;
    }

    case 0x22: {  // UPDATE_MAG
        float m[3];
        if (!readN(m, 12)) return;
        if (!dup) {
            t0 = micros();
            ekf.updateMag(m);
            us = micros() - t0;
            cacheState(us, seq);
            last_seq = seq;
        }
        sendCached();
        break;
    }

    case 0x23:  // UPDATE_ZUPT
        if (!dup) {
            t0 = micros();
            ekf.updateZupt();
            us = micros() - t0;
            cacheState(us, seq);
            last_seq = seq;
        }
        sendCached();
        break;

    case 0x24: {  // UPDATE_ACC_STATIC
        float a[3];
        if (!readN(a, 12)) return;
        if (!dup) {
            t0 = micros();
            ekf.updateAccStatic(a);
            us = micros() - t0;
            cacheState(us, seq);
            last_seq = seq;
        }
        sendCached();
        break;
    }

    case 0x25: {  // UPDATE_GYRO_STATIC
        float w[3];
        if (!readN(w, 12)) return;
        if (!dup) {
            t0 = micros();
            ekf.updateGyroStatic(w);
            us = micros() - t0;
            cacheState(us, seq);
            last_seq = seq;
        }
        sendCached();
        break;
    }

    case 0x30:  // GET_STATE (현재 상태 조회, 항상 fresh — dedup 안 함)
        cacheState(0, seq);
        last_seq = seq;
        sendCached();
        break;

    default:
        break;
    }
}
