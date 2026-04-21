#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <esp_timer.h>
#include "MMC5983MA.h"
#include "MX25Logger.h"
#include "sensor_data.h"

// ============================================================
// Pin / Config
// ============================================================
static const uint8_t  MAG_SDA_PIN = 5;
static const uint8_t  MAG_SCL_PIN = 4;

static const uint8_t  FLASH_SCK_PIN  = 16;
static const uint8_t  FLASH_MISO_PIN = 7;
static const uint8_t  FLASH_MOSI_PIN = 15;
static const uint8_t  FLASH_CS_PIN   = 6;

static const TickType_t MAG_POLL_TICKS = pdMS_TO_TICKS(2);
static const uint32_t   CAL_DURATION_MS = 30000;

// ============================================================
// Log queue item
// ============================================================
#define LOG_ITEM_MAX_SIZE 32        // mag_pkt = 19 B, 여유
#define LOG_QUEUE_LENGTH  128

struct LogItem {
  uint8_t data[LOG_ITEM_MAX_SIZE];
  uint8_t len;
};

// ============================================================
// Globals
// ============================================================
SPIClass flashSPI(FSPI);
MMC5983MA mag(&Wire);
MX25Logger logger;

struct MagCal {
    float biasX  = 0.0f, biasY  = 0.0f, biasZ  = 0.0f;
    float scaleX = 1.0f, scaleY = 1.0f, scaleZ = 1.0f;
};
MagCal magCal;

TaskHandle_t      TaskHandle_MAG   = NULL;
TaskHandle_t      FlushTaskHandle  = NULL;
SemaphoreHandle_t i2cMutex         = NULL;   // protects Wire
SemaphoreHandle_t flashMutex       = NULL;   // protects flash dump vs FlushTask
QueueHandle_t     logQueue         = NULL;

volatile bool isLogging = false;

// ============================================================
// utility
// ============================================================
inline uint32_t getTimeUs32() { return (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF); }

// ============================================================
// Calibration routine (CALIBRATE_MAG 명령으로 호출)
//   MAG_Task 를 suspend 하고 i2cMutex 를 잡은 채
//   CMM 1kHz + isDataReady() 폴링으로 durationMs 동안 최대한 수집
// ============================================================
void runCalibration(uint32_t durationMs) {
    bool wasLogging = isLogging;
    isLogging = false;
    if (TaskHandle_MAG) vTaskSuspend(TaskHandle_MAG);

    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) != pdTRUE) {
        if (TaskHandle_MAG) vTaskResume(TaskHandle_MAG);
        if (wasLogging) isLogging = true;
        return;
    }

    Serial.println();
    Serial.println("=== Magnetometer Calibration ===");
    Serial.printf("Rotate the board in ALL orientations for %lu s (figure-8).\n",
                  durationMs / 1000);
    Serial.println("Start in 3..."); delay(1000);
    Serial.println("      2...");    delay(1000);
    Serial.println("      1...");    delay(1000);
    Serial.println("GO!");

    // 연속측정(CMM) 을 최대 속도로 돌려두고 isDataReady 폴링으로 최대한 많이 수집
    mag.setContinuousFrequency(1000);
    mag.clearInterruptFlag();

    float minX =  1e9f, maxX = -1e9f;
    float minY =  1e9f, maxY = -1e9f;
    float minZ =  1e9f, maxZ = -1e9f;
    uint32_t samples = 0;

    const uint32_t start = millis();
    uint32_t lastReport  = start;

    while (millis() - start < durationMs) {
        if (!mag.isDataReady()) {
            continue;   // busy-poll: 새 샘플 나올 때까지 기다림
        }

        float mx, my, mz;
        mag.readMag(mx, my, mz);
        mag.clearInterruptFlag();   // MEAS_M_DONE 은 W1C — 다음 샘플 감지용으로 클리어

        if (mx < minX) minX = mx;  if (mx > maxX) maxX = mx;
        if (my < minY) minY = my;  if (my > maxY) maxY = my;
        if (mz < minZ) minZ = mz;  if (mz > maxZ) maxZ = mz;
        samples++;

        if (millis() - lastReport >= 1000) {
            lastReport = millis();
            uint32_t remain = (durationMs - (millis() - start)) / 1000;
            Serial.printf("  [%2lus left] N=%lu X[%+.3f,%+.3f] Y[%+.3f,%+.3f] Z[%+.3f,%+.3f]\n",
                          remain, samples, minX, maxX, minY, maxY, minZ, maxZ);
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

    mag.applyDefaults();
    mag.clearInterruptFlag();

    Serial.println("=== Calibration complete ===");
    Serial.printf("  samples : %lu\n", samples);
    Serial.printf("  bias  (Gauss): %+.4f  %+.4f  %+.4f\n",
                  magCal.biasX,  magCal.biasY,  magCal.biasZ);
    Serial.printf("  scale        : %.4f  %.4f  %.4f\n",
                  magCal.scaleX, magCal.scaleY, magCal.scaleZ);

    xSemaphoreGive(i2cMutex);
    if (TaskHandle_MAG) vTaskResume(TaskHandle_MAG);
    if (wasLogging) isLogging = true;
}

// ============================================================
// push to log queue (drops silently if queue full)
// ============================================================
static void pushLog(const void *pkt, uint8_t len) {
    if (!isLogging) return;
    if (len > LOG_ITEM_MAX_SIZE) return;
    LogItem item;
    item.len = len;
    memcpy(item.data, pkt, len);
    xQueueSend(logQueue, &item, 0);
}

// ============================================================
// Core 1 : MAG 수집 태스크 (polling)
// ============================================================
void MAG_Task(void *pvParameters) {
    for (;;) {
        bool     got = false;
        uint32_t ts  = 0;
        float    mx = 0, my = 0, mz = 0;

        if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
            if (mag.isDataReady()) {
                ts = getTimeUs32();
                mag.readMag(mx, my, mz);
                mag.clearInterruptFlag();
                got = true;
            }
            xSemaphoreGive(i2cMutex);
        }

        if (!got) {
            vTaskDelay(MAG_POLL_TICKS);
            continue;
        }

        mx = (mx - magCal.biasX) * magCal.scaleX;
        my = (my - magCal.biasY) * magCal.scaleY;
        mz = (mz - magCal.biasZ) * magCal.scaleZ;

        mag_pkt pkt;
        pkt.header.SYNC_BYTE = 0xAA;
        pkt.header.id        = ID_MAG;
        pkt.header.len       = sizeof(mag_pkt);
        pkt.t  = ts;
        pkt.mx = mx;
        pkt.my = my;
        pkt.mz = mz;
        pushLog(&pkt, sizeof(pkt));
    }
}

// ============================================================
// Core 0 : Flash flush task
// ============================================================
void FlushTask(void *pvParameters) {
    LogItem item;
    for (;;) {
        if (xQueueReceive(logQueue, &item, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (xSemaphoreTake(flashMutex, portMAX_DELAY) == pdTRUE) {
                logger.appendRaw(item.data, item.len);
                xSemaphoreGive(flashMutex);
            }
        }
        if (logger.hasFullPage()) {
            if (xSemaphoreTake(flashMutex, portMAX_DELAY) == pdTRUE) {
                logger.flushPages();
                xSemaphoreGive(flashMutex);
            }
        }
    }
}

// ============================================================
// Core 0 : Serial 명령 처리
//   지원 명령: START / STOP / PARSE / ERASEALL / CALIBRATE_MAG
// ============================================================
static String readSerialLine() {
    if (Serial.available()) {
        String s = Serial.readStringUntil('\n');
        s.trim();
        return s;
    }
    return "";
}

void processCommandTask(void *pvParameters) {
    for (;;) {
        String cmd = readSerialLine();
        if (cmd.length() > 0) {
            cmd.toUpperCase();

            if (cmd == "START") {
                if (!isLogging) {
                    isLogging = true;
                    Serial.println("LOGGING STARTED.");
                } else {
                    Serial.println("ALREADY LOGGING.");
                }
            }
            else if (cmd == "STOP") {
                if (isLogging) {
                    isLogging = false;
                    vTaskDelay(pdMS_TO_TICKS(100));   // 큐 소진 대기
                    if (xSemaphoreTake(flashMutex, portMAX_DELAY) == pdTRUE) {
                        logger.forceFlushBuffer();
                        xSemaphoreGive(flashMutex);
                    }
                    Serial.println("LOGGING STOPPED.");
                } else {
                    Serial.println("NOT LOGGING.");
                }
            }
            else if (cmd == "PARSE") {
                bool wasLogging = isLogging;
                if (wasLogging) {
                    isLogging = false;
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                if (xSemaphoreTake(flashMutex, portMAX_DELAY) == pdTRUE) {
                    Serial.println("DUMP START...");
                    logger.forceFlushBuffer();
                    logger.dumpRawBinary(Serial);
                    Serial.println("\nDUMP DONE.");
                    xSemaphoreGive(flashMutex);
                }
                if (wasLogging) isLogging = true;
            }
            else if (cmd == "ERASEALL") {
                bool wasLogging = isLogging;
                if (wasLogging) {
                    isLogging = false;
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                if (xSemaphoreTake(flashMutex, portMAX_DELAY) == pdTRUE) {
                    Serial.println("FLASH ERASING...");
                    logger.eraseAll();
                    xSemaphoreGive(flashMutex);
                }
                Serial.println("FLASH ERASED.");
                if (wasLogging) isLogging = true;
            }
            else if (cmd == "CALIBRATE_MAG") {
                Serial.println("MAG CALIBRATION START...");
                runCalibration(CAL_DURATION_MS);
                Serial.println("MAG CALIBRATION DONE.");
            }
            else {
                Serial.printf("UNKNOWN CMD: %s\n", cmd.c_str());
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ============================================================
// setup
// ============================================================
void setup() {
    Serial.begin(921600);   // parse/main.py 기본 baud
    while (!Serial) delay(10);

    i2cMutex   = xSemaphoreCreateMutex();
    flashMutex = xSemaphoreCreateMutex();
    logQueue   = xQueueCreate(LOG_QUEUE_LENGTH, sizeof(LogItem));
    if (!i2cMutex || !flashMutex || !logQueue) {
        Serial.println("동기화 객체 생성 에러");
        while (1);
    }

    Wire.begin(MAG_SDA_PIN, MAG_SCL_PIN, 400000);

    if (!mag.begin()) {
        Serial.println("MMC5983MA init failed!");
        while (1) delay(1000);
    }
    Serial.println("MMC5983MA OK");

    logger.begin(&flashSPI, FLASH_SCK_PIN, FLASH_MISO_PIN, FLASH_MOSI_PIN, FLASH_CS_PIN);
    Serial.println("MX25 LOGGER OK");

    mag.clearInterruptFlag();

    // 태스크 생성 — 모두 즉시 동작, 로깅은 isLogging 게이트로 제어
    xTaskCreatePinnedToCore(processCommandTask, "CmdTask", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(MAG_Task,  "MAG_T",   4096, NULL, 5, &TaskHandle_MAG,  1);
    xTaskCreatePinnedToCore(FlushTask, "Flush_T", 4096, NULL, 2, &FlushTaskHandle, 0);

    Serial.println("READY. Commands: CALIBRATE_MAG / START / STOP / PARSE / ERASEALL");
}

void loop() {}
