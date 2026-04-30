#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <esp_timer.h>

#include "Config.h"
#include "LSM6DSO32.h"
#include "BMP388.h"
#include "MMC5983MA.h"
#include "NEOM9N.h"
#include "NAV.h"
#include "MX25Logger.h"
#include "sensor_data.h"
#include "BLE.h"

// ============================================================
// Global objects
// ============================================================
static SPIClass sensorSPI(HSPI);
static SPIClass flashSPI(FSPI);

static LSM6DSO32 imu(IMU_CS_PIN, &sensorSPI);
static BMP388    bmp(BMP_CS_PIN, &sensorSPI);
static MMC5983MA mag(&Wire);
static NEOM9N    gps(Serial1, GPS_RX_PIN, GPS_TX_PIN);
static NAV       nav;
static MX25Logger logger;

static TaskHandle_t TaskHandle_IMU  = NULL;
static TaskHandle_t TaskHandle_BMP  = NULL;
static TaskHandle_t TaskHandle_MAG  = NULL;
static TaskHandle_t TaskHandle_GPS  = NULL;
static TaskHandle_t FlushTaskHandle = NULL;

static SemaphoreHandle_t spiMutex   = NULL;
static SemaphoreHandle_t i2cMutex   = NULL;
static SemaphoreHandle_t navMutex   = NULL;
static SemaphoreHandle_t flashMutex = NULL;

static volatile bool flightActive = false;

// Forward Declarations
void IMU_Task(void *pvParameters);
void BMP_Task(void *pvParameters);
void MAG_Task(void *pvParameters);
void GPS_Task(void *pvParameters);
void FlushTask(void *pvParameters);
void beep(int ms);
void errorBeep();
void clearSensors();
void attachSensorInterrupts();
void detachSensorInterrupts();

// ============================================================
// setup() : Initialization
// ============================================================
void setup() {
  Serial.begin(SERIAL_BAUD);

  // 1. Pins & Feedback
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN,    OUTPUT);
  pinMode(PYRO_1_PIN, OUTPUT); pinMode(PYRO_2_PIN, OUTPUT);
  pinMode(SERVO_1_PIN, OUTPUT); pinMode(SERVO_2_PIN, OUTPUT);
  pinMode(SERVO_3_PIN, OUTPUT); pinMode(SERVO_4_PIN, OUTPUT);
  
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN,    LOW);
  digitalWrite(PYRO_1_PIN, LOW); digitalWrite(PYRO_2_PIN, LOW);

  // 2. Synchronizations
  spiMutex   = xSemaphoreCreateMutex();
  i2cMutex   = xSemaphoreCreateMutex();
  navMutex   = xSemaphoreCreateMutex();
  flashMutex = xSemaphoreCreateMutex();
  if (!spiMutex || !i2cMutex || !navMutex || !flashMutex) {
    errorBeep();
    while (1);
  }

  // 3. Communications
  initBLE(BLE_DEVICE_NAME);
  sensorSPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
  Wire.begin(MAG_SDA_PIN, MAG_SCL_PIN, 400000);

  // 4. Sensors
  bool ok = true;
  if (!imu.begin()) { sendResponse("IMU FAIL\n"); ok = false; }
  if (!bmp.begin()) { sendResponse("BMP FAIL\n"); ok = false; }
  if (!mag.begin()) { sendResponse("MAG FAIL\n"); ok = false; }
  if (!gps.begin(10)) { sendResponse("GPS WARN\n"); } // Non-fatal
  
  if (!ok) { errorBeep(); while(1); }
  sendResponse("ALL SENSORS OK\n");

  // 5. Storage
  logger.begin(&flashSPI, FLASH_SCK_PIN, FLASH_MISO_PIN, FLASH_MOSI_PIN, FLASH_CS_PIN);

  pinMode(IMU_INT1_PIN, INPUT_PULLDOWN);
  pinMode(BMP_INT_PIN,  INPUT);

  // 6. Tasks
  xTaskCreatePinnedToCore(IMU_Task, "IMU_T",  STACK_SIZE_SENSOR, NULL, TASK_C1_PRIO_IMU, &TaskHandle_IMU, 1);
  xTaskCreatePinnedToCore(BMP_Task, "BMP_T",  STACK_SIZE_SENSOR, NULL, TASK_C1_PRIO_BMP, &TaskHandle_BMP, 1);
  xTaskCreatePinnedToCore(MAG_Task, "MAG_T",  STACK_SIZE_SENSOR, NULL, TASK_C1_PRIO_MAG, &TaskHandle_MAG, 1);
  xTaskCreatePinnedToCore(GPS_Task, "GPS_T",  STACK_SIZE_SENSOR, NULL, TASK_C1_PRIO_GPS, &TaskHandle_GPS, 1);
  xTaskCreatePinnedToCore(FlushTask,"Flush_T",STACK_SIZE_FLUSH,  NULL, TASK_C0_PRIO_FLUSH, &FlushTaskHandle, 0);

  beep(200);
  sendResponse(">>> READY\n");
}

// ============================================================
// loop() : Command Dispatcher
// ============================================================
void loop() {
  String cmd = getIncomingRaw();
  if (cmd.length() == 0) { vTaskDelay(pdMS_TO_TICKS(50)); return; }
  cmd.toUpperCase();

  // --- Common Commands ---
  if (cmd == "REBOOT") {
    sendResponse("REBOOTING...\n");
    if (logger.isEnabled()) {
      logger.setEnabled(false);
      detachSensorInterrupts();
      vTaskDelay(pdMS_TO_TICKS(100));
      if (xSemaphoreTake(flashMutex, portMAX_DELAY) == pdTRUE) {
        logger.forceFlushBuffer();
        xSemaphoreGive(flashMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP.restart();
  }

  // --- Pre-Flight Commands ---
  else if (!flightActive) {
    if (cmd == "CALIBRATE") {
      digitalWrite(LED_PIN, HIGH);
      sendResponse("CALIBRATING IMU+BMP...\n");
      imu.calibrate(CALIB_SAMPLES);
      bmp.calibrate(CALIB_SAMPLES);
      clearSensors();
      sendResponse("DONE.\n");
      digitalWrite(LED_PIN, LOW); beep(100);
    }
    else if (cmd == "CALIBRATE_MAG") {
      digitalWrite(LED_PIN, HIGH);
      sendResponse("CALIBRATING MAG (30S)...\n");
      mag.calibrate(MAG_CALIB_MS);
      mag.clearInterruptFlag();
      sendResponse("DONE.\n");
      digitalWrite(LED_PIN, LOW); beep(100);
    }
    else if (cmd == "CALIBRATE_GPS") {
      if (gps.calibrate()) { sendResponse("GPS ORIGIN OK\n"); beep(100); }
      else { sendResponse("GPS ORIGIN FAIL\n"); }
    }
    else if (cmd == "ERASE") {
      sendResponse("ERASING FLASH...\n");
      logger.eraseAll();
      sendResponse("DONE.\n");
      beep(500);
    }
    else if (cmd == "START") {
      sendResponse("STARTING...\n");
      clearSensors();
      mag.clearInterruptFlag();
      logger.setEnabled(true);
      flightActive = true;
      attachSensorInterrupts();
      imu.enableAccelDataReadyInterrupt(1);
      clearSensors(); // Final clear after INT attached
      digitalWrite(LED_PIN, HIGH);
      beep(300);
      sendResponse("FLIGHT ACTIVE\n");
    }
  }

  // --- Flight/Active Commands ---
  else {
    if (cmd == "STOP") {
      if (logger.isEnabled()) {
        logger.setEnabled(false);
        detachSensorInterrupts();
        vTaskDelay(pdMS_TO_TICKS(100));
        if (xSemaphoreTake(flashMutex, portMAX_DELAY) == pdTRUE) {
          logger.forceFlushBuffer();
          xSemaphoreGive(flashMutex);
        }
        flightActive = false;
        digitalWrite(LED_PIN, LOW);
        beep(200); vTaskDelay(pdMS_TO_TICKS(50)); beep(200);
        sendResponse("STOPPED.\n");
      }
    }
    else if (cmd == "PARSE") {
      bool wasLogging = logger.isEnabled();
      if (wasLogging) {
        logger.setEnabled(false);
        detachSensorInterrupts();
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      if (xSemaphoreTake(flashMutex, portMAX_DELAY) == pdTRUE) {
        sendResponse("DUMP START\n");
        logger.forceFlushBuffer();
        logger.dumpRawBinary(Serial);
        sendResponse("DUMP DONE\n");
        xSemaphoreGive(flashMutex);
      }
      if (wasLogging) { flightActive = false; digitalWrite(LED_PIN, LOW); }
    }
  }

  vTaskDelay(pdMS_TO_TICKS(50));
}

// ============================================================
// Sensor & Utility Tasks (Core 1)
// ============================================================
void IMU_Task(void *pvParameters) {
  Raw_imu raw;
  static int64_t lastImuTime_us = 0;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (!flightActive) continue;

    if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
      raw.timestamp = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
      imu.readCalibratedIMU(raw.gx, raw.gy, raw.gz, raw.ax, raw.ay, raw.az);
      xSemaphoreGive(spiMutex);
    }
    logger.logImu(raw);

    int64_t now_us = esp_timer_get_time();
    float dt = (lastImuTime_us > 0) ? (float)(now_us - lastImuTime_us) * 1e-6f : 0.0f;
    lastImuTime_us = now_us;

    if (xSemaphoreTake(navMutex, portMAX_DELAY) == pdTRUE) {
      nav.updateIMU(raw);
      nav.ekfPredict(dt);
      logger.logState(nav.getNominal());
      xSemaphoreGive(navMutex);
    }
  }
}

void BMP_Task(void *pvParameters) {
  Raw_press p;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (!flightActive) continue;

    if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
      p.timestamp = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
      bmp.readAltitude(p.alt);
      xSemaphoreGive(spiMutex);
    }
    logger.logBaro(p);

    if (xSemaphoreTake(navMutex, portMAX_DELAY) == pdTRUE) {
      nav.updatePress(p);
      nav.ekfUpdateBaro();
      xSemaphoreGive(navMutex);
    }
  }
}

void MAG_Task(void *pvParameters) {
  Raw_mag m;
  for (;;) {
    if (!flightActive) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
    bool got = false;
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
      if (mag.isDataReady()) {
        m.timestamp = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
        mag.readCalibratedMag(m.mx, m.my, m.mz);
        mag.clearInterruptFlag();
        got = true;
      }
      xSemaphoreGive(i2cMutex);
    }
    if (!got) { vTaskDelay(pdMS_TO_TICKS(MAG_POLL_MS)); continue; }
    logger.logMag(m);
    if (xSemaphoreTake(navMutex, portMAX_DELAY) == pdTRUE) {
      nav.updateMag(m); nav.ekfUpdateMag();
      xSemaphoreGive(navMutex);
    }
  }
}

void GPS_Task(void *pvParameters) {
  Raw_gps g;
  for (;;) {
    if (!flightActive) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
    if (gps.update()) {
      g.timestamp = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
      g.hasPos    = gps.getNED(g.pn, g.pe, g.pd, g.vn, g.ve, g.vd, g.hAcc, g.vAcc, g.fixType, g.numSV);
      logger.logGps(g);
      if (xSemaphoreTake(navMutex, portMAX_DELAY) == pdTRUE) {
        nav.updateGps(g); nav.ekfUpdateGps();
        xSemaphoreGive(navMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(GPS_POLL_MS));
  }
}

void FlushTask(void *pvParameters) {
  for (;;) { logger.serviceFlush(); }
}

// ============================================================
// Utilities
// ============================================================
void beep(int ms) {
  digitalWrite(BUZZER_PIN, HIGH);
  vTaskDelay(pdMS_TO_TICKS(ms));
  digitalWrite(BUZZER_PIN, LOW);
}

void errorBeep() {
  for(int i=0; i<3; i++) { beep(100); vTaskDelay(pdMS_TO_TICKS(100)); }
}

void clearSensors() {
  int16_t d1, d2, d3, d4, d5, d6; float df;
  imu.readRawIMU(d1, d2, d3, d4, d5, d6);
  bmp.readData(df);
}

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
