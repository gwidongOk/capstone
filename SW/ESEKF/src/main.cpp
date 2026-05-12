#include <Arduino.h>
#include "ESEKF.h"
#include "HilProtocol.h"

// HIL host(MATLAB) ↔ ESP32-S3 verification loop
//   - 921600 baud USB-CDC
//   - lock-step: 명령 1개 → 응답 1개
//   - 모든 부동소수점은 little-endian float (4B)
//   - 프로토콜 사양은 HilProtocol.h 참조

static ESEKF        ekf;
static Hil::Framer  framer;
static uint8_t      payload[Hil::MAX_PAYLOAD];

void setup() {
    Serial.begin(921600);
    while (!Serial && millis() < 2000) { /* USB enumerate */ }
    framer.reset();
    // host(MATLAB)가 CMD_GET_STATE(0x30) 로 핸드셰이크 시작
}

void loop() {
    while (Serial.available()) {
        uint8_t id;
        size_t  len = 0;
        if (framer.feed((uint8_t)Serial.read(), id, payload, len)) {
            Hil::handle(ekf, id, payload, len, Serial);
        }
    }
}
