#include <Arduino.h>
#include <Wire.h>
#include <esp_timer.h>
#include "MMC5983MA.h"

// ============================================================
// Pin / Config
// ============================================================
static const uint8_t  MAG_SDA_PIN = 5;
static const uint8_t  MAG_SCL_PIN = 4;

// Polling interval — short enough to catch every sample at any supported ODR.
static const TickType_t MAG_POLL_TICKS = pdMS_TO_TICKS(2);

// Sensor tuning (BW / CMM / PRD_SET) lives in MMC5983MA.h as DEFAULT_*.
// Only the calibration window stays here because it's a user-facing UX choice.
static const uint32_t CAL_DURATION_MS = 30000;

// ============================================================
// Global objects (actual memory)
// ============================================================
MMC5983MA mag(&Wire);

struct MagCal {
    float biasX  = 0.0f, biasY  = 0.0f, biasZ  = 0.0f;
    float scaleX = 1.0f, scaleY = 1.0f, scaleZ = 1.0f;
};
MagCal magCal;

struct MagSample {
    uint32_t timestamp_us;
    float    mx, my, mz;   // calibrated, Gauss
};
MagSample latestMag = {0, 0, 0, 0};

TaskHandle_t      TaskHandle_MAG = NULL;
SemaphoreHandle_t i2cMutex       = NULL;   // protects Wire
SemaphoreHandle_t dataMutex      = NULL;   // protects latestMag

// ============================================================
// utility
// ============================================================
inline uint32_t getTimeUs32() { return (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF); }

// Polling read — used during calibration (blocking, single-shot style)
static bool pollReadMag(float& mx, float& my, float& mz, uint32_t timeoutMs = 50) {
    const uint32_t start = millis();
    while (!mag.isDataReady()) {
        if (millis() - start > timeoutMs) return false;
    }
    mag.readMag(mx, my, mz);
    mag.clearInterruptFlag();
    return true;
}

// ============================================================
// Calibration (blocking, runs once before tasks start)
// ============================================================
void calibrateMag(uint32_t durationMs) {
    Serial.println();
    Serial.println("=== Magnetometer Calibration ===");
    Serial.printf("Rotate the board in ALL orientations for %lu s (figure-8).\n",
                  durationMs / 1000);
    Serial.println("Start in 3..."); delay(1000);
    Serial.println("      2...");    delay(1000);
    Serial.println("      1...");    delay(1000);
    Serial.println("GO!");

    // Boost CMM rate during calibration for better orientation coverage.
    // Restored via mag.applyDefaults() below.
    mag.setContinuousFrequency(1000);

    float minX =  1e9f, maxX = -1e9f;
    float minY =  1e9f, maxY = -1e9f;
    float minZ =  1e9f, maxZ = -1e9f;
    uint32_t samples = 0;

    mag.clearInterruptFlag();

    const uint32_t start = millis();
    uint32_t lastReport  = start;

    while (millis() - start < durationMs) {
        float mx, my, mz;
        if (!pollReadMag(mx, my, mz)) {
            Serial.println("  [!] sample timeout");
            continue;
        }
        if (mx < minX) minX = mx;  if (mx > maxX) maxX = mx;
        if (my < minY) minY = my;  if (my > maxY) maxY = my;
        if (mz < minZ) minZ = mz;  if (mz > maxZ) maxZ = mz;
        samples++;

        if (millis() - lastReport >= 1000) {
            lastReport = millis();
            uint32_t remain = (durationMs - (millis() - start)) / 1000;
            Serial.printf("  [%2lus left] X[%+.3f,%+.3f] Y[%+.3f,%+.3f] Z[%+.3f,%+.3f]\n",
                          remain, minX, maxX, minY, maxY, minZ, maxZ);
        }
    }

    magCal.biasX = 0.5f * (maxX + minX);
    magCal.biasY = 0.5f * (maxY + minY);
    magCal.biasZ = 0.5f * (maxZ + minZ);
    float rX = 0.5f * (maxX - minX);
    float rY = 0.5f * (maxY - minY);
    float rZ = 0.5f * (maxZ - minZ);
    float rAvg = (rX + rY + rZ) / 3.0f;
    magCal.scaleX = (rX > 1e-6f) ? (rAvg / rX) : 1.0f;
    magCal.scaleY = (rY > 1e-6f) ? (rAvg / rY) : 1.0f;
    magCal.scaleZ = (rZ > 1e-6f) ? (rAvg / rZ) : 1.0f;

    mag.applyDefaults();   // restore BW/AUTO_SR/PRD_SET/CMM to ES-EKF config
    mag.clearInterruptFlag();

    Serial.println("=== Calibration complete ===");
    Serial.printf("  samples : %lu\n", samples);
    Serial.printf("  bias  (Gauss): %+.4f  %+.4f  %+.4f\n",
                  magCal.biasX,  magCal.biasY,  magCal.biasZ);
    Serial.printf("  scale        : %.4f  %.4f  %.4f\n",
                  magCal.scaleX, magCal.scaleY, magCal.scaleZ);
    Serial.println();
}

// ============================================================
// Core 1 : MAG 수집 + 전송 태스크 (polling)
// ============================================================
void MAG_Task(void *pvParameters) {
    for (;;) {
        bool     got = false;
        uint32_t ts  = 0;
        float    mx = 0, my = 0, mz = 0;

        // 한 번의 mutex 구간에서 ready 체크 + 읽기 + flag 클리어까지 수행
        if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
            if (mag.isDataReady()) {
                ts = getTimeUs32();
                mag.readMag(mx, my, mz);
                mag.clearInterruptFlag();   // W1C → 다음 샘플 완료를 감지할 수 있게
                got = true;
            }
            xSemaphoreGive(i2cMutex);
        }

        if (!got) {
            vTaskDelay(MAG_POLL_TICKS);     // 아직 안 찼으면 yield 후 재시도
            continue;
        }

        // 캘리브레이션 적용
        mx = (mx - magCal.biasX) * magCal.scaleX;
        my = (my - magCal.biasY) * magCal.scaleY;
        mz = (mz - magCal.biasZ) * magCal.scaleZ;

        // 공유 데이터 업데이트
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            latestMag.timestamp_us = ts;
            latestMag.mx = mx;
            latestMag.my = my;
            latestMag.mz = mz;
            xSemaphoreGive(dataMutex);
        }

        // ★ 이 자리에 EKF mag update / 로그 큐 전송을 붙일 것
        //   xTaskNotify(TaskHandle_NAV, EVENT_MAG_UPDATE, eSetBits);

        // 현재는 "데이터 전송" = Serial 출력
        Serial.printf("Mag: %+.4f  %+.4f  %+.4f (Gauss)\n", mx, my, mz);
    }
}

// ============================================================
// setup
// ============================================================
void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    i2cMutex  = xSemaphoreCreateMutex();
    dataMutex = xSemaphoreCreateMutex();
    if (!i2cMutex || !dataMutex) {
        Serial.println("동기화 객체 생성 에러");
        while (1);
    }

    Wire.begin(MAG_SDA_PIN, MAG_SCL_PIN, 400000);

    if (!mag.begin()) {
        Serial.println("MMC5983MA init failed!");
        while (1) delay(1000);
    }
    Serial.println("MMC5983MA OK");

    calibrateMag(CAL_DURATION_MS);

    // MAG 태스크 생성 (Core 1) — 태스크 내부가 직접 STATUS 를 polling
    mag.clearInterruptFlag();   // 잔여 flag 클리어 후 태스크 시작
    xTaskCreatePinnedToCore(MAG_Task, "MAG_T", 4096, NULL, 5, &TaskHandle_MAG, 1);

    Serial.println("MAG SYSTEM ACTIVE");
}

void loop() {}
