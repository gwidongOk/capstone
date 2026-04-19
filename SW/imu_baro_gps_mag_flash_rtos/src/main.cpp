#include <Arduino.h>
#include <Wire.h>
#include <esp_timer.h>
#include "Config.h"
#include "BLE.h"

// ============================================================
// Global objects
// ============================================================
SPIClass sensorSPI(HSPI);
SPIClass flashSPI(FSPI);

LSM6DSO32 imu(IMU_CS_PIN, &sensorSPI);
BMP388    bmp(BMP_CS_PIN, &sensorSPI);
MMC5983MA mag(&Wire);
NEOM9N    gps(Serial1, GPS_RX_PIN, GPS_TX_PIN);
NAV       nav;
MX25Logger logger;

TaskHandle_t TaskHandle_IMU    = NULL;
TaskHandle_t TaskHandle_BMP    = NULL;
TaskHandle_t TaskHandle_MAG    = NULL;
TaskHandle_t TaskHandle_GPS    = NULL;
TaskHandle_t TaskHandle_NAV    = NULL;
TaskHandle_t FlushTaskHandle   = NULL;

SemaphoreHandle_t spiMutex   = NULL;
SemaphoreHandle_t i2cMutex   = NULL;
SemaphoreHandle_t dataMutex  = NULL;
SemaphoreHandle_t flashMutex = NULL;
QueueHandle_t     logQueue   = NULL;

volatile bool isLogging     = false;
volatile bool systemStarted = false;

static const TickType_t MAG_POLL_TICKS = pdMS_TO_TICKS(2);
static const TickType_t GPS_POLL_TICKS = pdMS_TO_TICKS(10);

// ============================================================
// Utility / ISR
// ============================================================
inline int64_t  getTimeUs()   { return esp_timer_get_time(); }
inline uint32_t getTimeUs32() { return (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF); }

void IRAM_ATTR IMUInterruptHandler() {
  BaseType_t woken = pdFALSE;
  if (TaskHandle_IMU) vTaskNotifyGiveFromISR(TaskHandle_IMU, &woken);
  portYIELD_FROM_ISR(woken);
}

void IRAM_ATTR BMPInterruptHandler() {
  BaseType_t woken = pdFALSE;
  if (TaskHandle_BMP) vTaskNotifyGiveFromISR(TaskHandle_BMP, &woken);
  portYIELD_FROM_ISR(woken);
}

void attachSensorInterrupts() {
  attachInterrupt(digitalPinToInterrupt(IMU_INT1_PIN), IMUInterruptHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(BMP_INT_PIN),  BMPInterruptHandler,  RISING);
}

void detachSensorInterrupts() {
  detachInterrupt(digitalPinToInterrupt(IMU_INT1_PIN));
  detachInterrupt(digitalPinToInterrupt(BMP_INT_PIN));
}

// ============================================================
// Calibration routines (blocking, triggered from command task)
// ============================================================
void performCalibrationImuBaro() {
  bool wasLogging = isLogging;
  isLogging = false;

  detachSensorInterrupts();
  vTaskDelay(pdMS_TO_TICKS(100));

  if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
    imu.calibrate(100);
    bmp.calibrate(100);
    xSemaphoreGive(spiMutex);
  }

  // Flush residual samples that arrived during bias estimation
  if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
    int16_t d1,d2,d3,d4,d5,d6; float df;
    imu.readRawIMU(d1,d2,d3,d4,d5,d6);
    bmp.readData(df);
    xSemaphoreGive(spiMutex);
  }

  attachSensorInterrupts();
  if (wasLogging) isLogging = true;
}

void performCalibrationMag() {
  bool wasLogging = isLogging;
  isLogging = false;

  // Suspend MAG task; 30s blocking routine takes i2cMutex itself is not needed
  // because MAG task is the only I2C user. We still take it for safety.
  if (TaskHandle_MAG) vTaskSuspend(TaskHandle_MAG);

  if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
    mag.calibrate(30000);
    mag.clearInterruptFlag();
    xSemaphoreGive(i2cMutex);
  }

  if (TaskHandle_MAG) vTaskResume(TaskHandle_MAG);
  if (wasLogging) isLogging = true;
}

bool performCalibrationGps() {
  return gps.calibrate();
}

// ============================================================
// Command processing (Core 0)
// ============================================================
void processCommandTask(void *pvParameters) {
  for (;;) {
    String cmd = getIncomingRaw();
    if (cmd.length() > 0) {
      cmd.toUpperCase();

      if (cmd == "START") {
        if (!systemStarted) {
          systemStarted = true;
          sendResponse("SYSTEM STARTING...\n");
        } else if (!isLogging) {
          isLogging = true;
          attachSensorInterrupts();
          sendResponse("FLIGHT MODE RESUMED.\n");
        }
      }
      else if (cmd == "STOP") {
        if (isLogging) {
          isLogging = false;
          detachSensorInterrupts();
          vTaskDelay(pdMS_TO_TICKS(100));

          if (xSemaphoreTake(flashMutex, portMAX_DELAY) == pdTRUE) {
            logger.forceFlushBuffer();
            xSemaphoreGive(flashMutex);
          }
          sendResponse("FLIGHT MODE SUSPENDED.\n");
        }
      }
      else if (cmd == "CALIBRATE") {
        sendResponse("IMU+BARO CALIBRATION...\n");
        performCalibrationImuBaro();
        sendResponse("IMU+BARO CALIBRATION DONE.\n");
      }
      else if (cmd == "CALIBRATE_MAG") {
        sendResponse("MAG CALIBRATION START (rotate board 30s)...\n");
        performCalibrationMag();
        sendResponse("MAG CALIBRATION DONE.\n");
      }
      else if (cmd == "CALIBRATE_GPS") {
        sendResponse("GPS ORIGIN SET...\n");
        if (performCalibrationGps()) sendResponse("GPS ORIGIN OK.\n");
        else                         sendResponse("GPS ORIGIN FAILED (no fix / poor accuracy).\n");
      }
      else if (cmd == "PARSE") {
        bool wasLogging = isLogging;
        if (wasLogging) {
          isLogging = false;
          detachSensorInterrupts();
          vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (xSemaphoreTake(flashMutex, portMAX_DELAY) == pdTRUE) {
          sendResponse("DUMP START...\n");
          logger.forceFlushBuffer();
          logger.dumpRawBinary(Serial);
          sendResponse("DUMP DONE.\n");
          xSemaphoreGive(flashMutex);
        }
        if (wasLogging) {
          attachSensorInterrupts();
          isLogging = true;
        }
      }
      else if (cmd == "ERASE") {
        bool wasLogging = isLogging;
        if (wasLogging) {
          isLogging = false;
          detachSensorInterrupts();
          vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (xSemaphoreTake(flashMutex, portMAX_DELAY) == pdTRUE) {
          sendResponse("FLASH ERASING...\n");
          logger.eraseAll();
          xSemaphoreGive(flashMutex);
        }
        sendResponse("FLASH ERASED.\n");
        if (wasLogging) {
          attachSensorInterrupts();
          isLogging = true;
        }
      }
      else if (cmd == "REBOOT") {
        sendResponse("REBOOTING...\n");
        vTaskDelay(pdMS_TO_TICKS(200));
        ESP.restart();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ============================================================
// Core 1 : Sensor acquisition tasks
// ============================================================
void IMU_Task(void *pvParameters) {
  Raw_imu raw;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
      raw.timestamp = getTimeUs32();
      imu.readCalibratedIMU(raw.gx, raw.gy, raw.gz, raw.ax, raw.ay, raw.az);
      xSemaphoreGive(spiMutex);
    }
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      nav.updateIMU(raw);
      xSemaphoreGive(dataMutex);
    }
    xTaskNotify(TaskHandle_NAV, EVENT_IMU_UPDATE, eSetBits);
  }
}

void BMP_Task(void *pvParameters) {
  Raw_press p;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
      p.timestamp = getTimeUs32();
      bmp.readAltitude(p.alt);
      xSemaphoreGive(spiMutex);
    }
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      nav.updatePress(p);
      xSemaphoreGive(dataMutex);
    }
    xTaskNotify(TaskHandle_NAV, EVENT_BMP_UPDATE, eSetBits);
  }
}

void MAG_Task(void *pvParameters) {
  Raw_mag m;
  for (;;) {
    bool got = false;

    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
      if (mag.isDataReady()) {
        m.timestamp = getTimeUs32();
        mag.readCalibratedMag(m.mx, m.my, m.mz);
        mag.clearInterruptFlag();
        got = true;
      }
      xSemaphoreGive(i2cMutex);
    }

    if (!got) { vTaskDelay(MAG_POLL_TICKS); continue; }

    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
      nav.updateMag(m);
      xSemaphoreGive(dataMutex);
    }
    xTaskNotify(TaskHandle_NAV, EVENT_MAG_UPDATE, eSetBits);
  }
}

void GPS_Task(void *pvParameters) {
  Raw_gps g;
  for (;;) {
    if (gps.update()) {
      g.timestamp = getTimeUs32();
      g.hasPos    = gps.getNED(g.pn, g.pe, g.pd,
                                g.vn, g.ve, g.vd,
                                g.hAcc, g.vAcc,
                                g.fixType, g.numSV);

      if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        nav.updateGps(g);
        xSemaphoreGive(dataMutex);
      }
      xTaskNotify(TaskHandle_NAV, EVENT_GPS_UPDATE, eSetBits);
    }
    vTaskDelay(GPS_POLL_TICKS);
  }
}

// ============================================================
// Core 1 : NAV task — ES-EKF + log packet generation
// ============================================================
static void pushLog(const void *pkt, uint8_t len) {
  if (!isLogging) return;
  if (len > LOG_ITEM_MAX_SIZE) return;
  LogItem item;
  item.len = len;
  memcpy(item.data, pkt, len);
  xQueueSend(logQueue, &item, 0);
}

void NAV_Task(void *pvParameters) {
  uint32_t notifiedValue;
  int64_t  lastImuTime_us = 0;

  for (;;) {
    xTaskNotifyWait(0x00, 0xFFFFFFFF, &notifiedValue, portMAX_DELAY);
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) != pdTRUE) continue;

    // -------- IMU: predict + log imu_pkt + log state_pkt --------
    if (notifiedValue & EVENT_IMU_UPDATE) {
      Raw_imu ri = nav.getRawImu();
      int64_t now_us = getTimeUs();
      float dt = (lastImuTime_us > 0) ? (float)(now_us - lastImuTime_us) * 1e-6f : 0.0f;
      lastImuTime_us = now_us;

      nav.ekfPredict(dt);

      imu_pkt ipkt;
      ipkt.header.SYNC_BYTE = 0xAA;
      ipkt.header.id        = 2;
      ipkt.header.len       = sizeof(imu_pkt);
      ipkt.t  = ri.timestamp;
      ipkt.gx = ri.gx; ipkt.gy = ri.gy; ipkt.gz = ri.gz;
      ipkt.ax = ri.ax; ipkt.ay = ri.ay; ipkt.az = ri.az;
      pushLog(&ipkt, sizeof(ipkt));

      State_nominal nom = nav.getNominal();
      state_pkt spkt;
      spkt.header.SYNC_BYTE = 0xAA;
      spkt.header.id        = 5;
      spkt.header.len       = sizeof(state_pkt);
      spkt.t = nom.timestamp;
      memcpy(spkt.p,  nom.p,  sizeof(nom.p));
      memcpy(spkt.v,  nom.v,  sizeof(nom.v));
      memcpy(spkt.q,  nom.q,  sizeof(nom.q));
      memcpy(spkt.ba, nom.ba, sizeof(nom.ba));
      memcpy(spkt.bg, nom.bg, sizeof(nom.bg));
      pushLog(&spkt, sizeof(spkt));
    }

    // -------- BARO: update + log baro_pkt --------
    if (notifiedValue & EVENT_BMP_UPDATE) {
      Raw_press p = nav.getPress();
      nav.ekfUpdateBaro();

      baro_pkt pkt;
      pkt.header.SYNC_BYTE = 0xAA;
      pkt.header.id        = 1;
      pkt.header.len       = sizeof(baro_pkt);
      pkt.t   = p.timestamp;
      pkt.alt = p.alt;
      pushLog(&pkt, sizeof(pkt));
    }

    // -------- MAG: update + log mag_pkt --------
    if (notifiedValue & EVENT_MAG_UPDATE) {
      Raw_mag m = nav.getMag();
      nav.ekfUpdateMag();

      mag_pkt pkt;
      pkt.header.SYNC_BYTE = 0xAA;
      pkt.header.id        = 3;
      pkt.header.len       = sizeof(mag_pkt);
      pkt.t  = m.timestamp;
      pkt.mx = m.mx; pkt.my = m.my; pkt.mz = m.mz;
      pushLog(&pkt, sizeof(pkt));
    }

    // -------- GPS: update + log gps_pkt --------
    if (notifiedValue & EVENT_GPS_UPDATE) {
      Raw_gps g = nav.getGps();
      nav.ekfUpdateGps();

      gps_pkt pkt;
      pkt.header.SYNC_BYTE = 0xAA;
      pkt.header.id        = 4;
      pkt.header.len       = sizeof(gps_pkt);
      pkt.t  = g.timestamp;
      pkt.pn = g.pn; pkt.pe = g.pe; pkt.pd = g.pd;
      pkt.vn = g.vn; pkt.ve = g.ve; pkt.vd = g.vd;
      pkt.hAcc = g.hAcc; pkt.vAcc = g.vAcc;
      pkt.fixType = g.fixType;
      pkt.numSV   = g.numSV;
      pushLog(&pkt, sizeof(pkt));
    }

    xSemaphoreGive(dataMutex);
  }
}

// ============================================================
// Core 0 : Flash flush (page-based, unchanged flow)
// ============================================================
void FlushTask(void *pvParameters) {
  LogItem item;
  for (;;) {
    if (xQueueReceive(logQueue, &item, pdMS_TO_TICKS(10)) == pdTRUE) {
      logger.appendRaw(item.data, item.len);
    }
    if (logger.hasFullPage()) {
      logger.flushPages();
    }
  }
}

// ============================================================
// setup / loop
// ============================================================
void setup() {
  Serial.begin(921600);

  spiMutex   = xSemaphoreCreateMutex();
  i2cMutex   = xSemaphoreCreateMutex();
  dataMutex  = xSemaphoreCreateMutex();
  flashMutex = xSemaphoreCreateMutex();
  logQueue   = xQueueCreate(LOG_QUEUE_LENGTH, sizeof(LogItem));
  if (!spiMutex || !i2cMutex || !dataMutex || !flashMutex || !logQueue) {
    sendResponse("동기화 객체 생성 에러\n");
    while (1);
  }

  initBLE("2026ALTIS");

  // Buses
  sensorSPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
  Wire.begin(MAG_SDA_PIN, MAG_SCL_PIN, 400000);

  // Sensors
  if (!imu.begin()) { sendResponse("LSM6DSO32 INIT FAIL\n"); while (1); }
  if (!bmp.begin()) { sendResponse("BMP388 INIT FAIL\n");    while (1); }
  if (!mag.begin()) { sendResponse("MMC5983MA INIT FAIL\n"); while (1); }
  if (!gps.begin(10)) { sendResponse("NEOM9N INIT FAIL (non-fatal)\n"); }
  sendResponse("ALL SENSORS OK\n");

  // Flash (flashSPI is initialized inside logger.begin)
  logger.begin(&flashSPI, FLASH_SCK_PIN, FLASH_MISO_PIN, FLASH_MOSI_PIN, FLASH_CS_PIN);

  pinMode(IMU_INT1_PIN, INPUT_PULLDOWN);
  pinMode(BMP_INT_PIN,  INPUT);

  // Cmd task on Core 0
  xTaskCreatePinnedToCore(processCommandTask, "CmdTask", 4096, NULL, 1, NULL, 0);

  sendResponse(">>> WAITING FOR START COMMAND...\n");
  while (!systemStarted) vTaskDelay(pdMS_TO_TICKS(10));

  // Flush residual sensor data
  {
    int16_t d1,d2,d3,d4,d5,d6; float df;
    imu.readRawIMU(d1,d2,d3,d4,d5,d6);
    bmp.readData(df);
    mag.clearInterruptFlag();
  }

  isLogging = true;

  // Sensor tasks on Core 1
  xTaskCreatePinnedToCore(IMU_Task, "IMU_T",  4096, NULL, 5, &TaskHandle_IMU, 1);
  xTaskCreatePinnedToCore(BMP_Task, "BMP_T",  4096, NULL, 4, &TaskHandle_BMP, 1);
  xTaskCreatePinnedToCore(MAG_Task, "MAG_T",  4096, NULL, 4, &TaskHandle_MAG, 1);
  xTaskCreatePinnedToCore(GPS_Task, "GPS_T",  4096, NULL, 3, &TaskHandle_GPS, 1);
  xTaskCreatePinnedToCore(NAV_Task, "NAV_T",  8192, NULL, 3, &TaskHandle_NAV, 1);
  xTaskCreatePinnedToCore(FlushTask,"Flush_T",4096, NULL, 2, &FlushTaskHandle, 0);

  attachSensorInterrupts();
  imu.enableAccelDataReadyInterrupt(1);

  sendResponse("FLIGHT SYSTEM ACTIVE\n");
}

void loop() {}
