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

// Global objects
static SPIClass sensorSPI(HSPI);
static SPIClass flashSPI(FSPI);

static LSM6DSO32 imu(IMU_CS_PIN, &sensorSPI);
static BMP388    bmp(BMP_CS_PIN, &sensorSPI);
static MMC5983MA mag(&Wire);
static NEOM9N    gps(Serial1, GPS_RX_PIN, GPS_TX_PIN);
static NAV       nav;
static MX25Logger logger;

static TaskHandle_t TaskHandle_IMU   = NULL;
static TaskHandle_t TaskHandle_BMP   = NULL;
static TaskHandle_t TaskHandle_MAG   = NULL;
static TaskHandle_t TaskHandle_GPS   = NULL;
static TaskHandle_t FlightTaskHandle = NULL;
static TaskHandle_t FlushTaskHandle  = NULL;

static SemaphoreHandle_t spiMutex   = NULL;
static SemaphoreHandle_t i2cMutex   = NULL;
static SemaphoreHandle_t navMutex   = NULL;
static SemaphoreHandle_t flashMutex = NULL;

static volatile bool flightActive = false;
static volatile bool navTestActive = false;
static FlightPhase flightPhase = FlightPhase::PRE_FLIGHT;

// Forward Declarations
void IMU_Task(void *pvParameters);
void BMP_Task(void *pvParameters);
void MAG_Task(void *pvParameters);
void GPS_Task(void *pvParameters);
void Flight_Task(void *pvParameters);
void FlushTask(void *pvParameters);
void beep(int ms, int count = 1);
void clearSensors();
void attachSensorInterrupts();
void detachSensorInterrupts();

// =============================================================================
// setup() : 부트 초기화 (한 번만 실행)
//   순서 : 핀 → 뮤텍스 → 통신(BLE/SPI/I2C) → 센서 begin → 플래시 → 인터럽트 → RTOS 태스크
//   - 센서 begin 실패 시 부저 3비프 후 무한루프 (실패 박제)
//   - GPS는 non-fatal : 없어도 EKF는 IMU+Baro+Mag로 동작
// =============================================================================
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
    beep(100, 3);
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
  if (!gps.begin(GPS_RATE_HZ)) { sendResponse("GPS WARN\n"); } // Non-fatal
  
  if (!ok) { beep(100, 3); while(1); }
  sendResponse("ALL SENSORS OK\n");

  // 5. Storage
  logger.begin(&flashSPI, FLASH_SCK_PIN, FLASH_MISO_PIN, FLASH_MOSI_PIN, FLASH_CS_PIN, flashMutex);

  // 6. Interrupts (Always active to keep NAV updated)
  pinMode(IMU_INT1_PIN, INPUT_PULLDOWN);
  pinMode(BMP_INT_PIN,  INPUT);
  attachSensorInterrupts();
  imu.enableAccelDataReadyInterrupt(1);

  // 7. Tasks
  xTaskCreatePinnedToCore(IMU_Task, "IMU_T",  STACK_SIZE_SENSOR, NULL, TASK_C1_PRIO_IMU, &TaskHandle_IMU, 1);
  xTaskCreatePinnedToCore(BMP_Task, "BMP_T",  STACK_SIZE_SENSOR, NULL, TASK_C0_PRIO_BMP, &TaskHandle_BMP, 0);
  xTaskCreatePinnedToCore(MAG_Task, "MAG_T",  STACK_SIZE_SENSOR, NULL, TASK_C0_PRIO_MAG, &TaskHandle_MAG, 0);
  xTaskCreatePinnedToCore(GPS_Task, "GPS_T",  STACK_SIZE_SENSOR, NULL, TASK_C0_PRIO_GPS, &TaskHandle_GPS, 0);
  xTaskCreatePinnedToCore(Flight_Task,"Flt_T", 4096,              NULL, 3,                 &FlightTaskHandle, 0);
  xTaskCreatePinnedToCore(FlushTask,"Flush_T",STACK_SIZE_FLUSH,  NULL, TASK_C0_PRIO_FLUSH, &FlushTaskHandle, 0);

  beep(200);
  sendResponse(">>> READY\n");
}

// =============================================================================
// loop() : USB/BLE 명령어 디스패처
//   - 명령 모음:
//     * REBOOT         : 안전 재부팅 (로그 강제 플러시 후 ESP.restart())
//     * CALIBRATE      : IMU + 기압계 정적 영점 (반복, 안정될 때까지)
//     * CALIBRATE_MAG  : 자력계 8자 회전 보정 (30초)
//     * CALIBRATE_GPS  : GPS origin 평균 캘리브레이션 (정지 상태)
//     * ERASE          : 플래시 칩 전체 소거
//     * ZUPT           : EKF 정렬 (TRIAD 없이 그냥 ZUPT 수렴)
//     * START          : 평균 TRIAD + ZUPT 수렴 → 비행 시작 (로깅 ON)
//     * TEST_NAV       : START와 동일하지만 Pyro 점화 비활성
//     * STOP           : 비행 종료 (로깅 OFF, 파이로 해제)
//     * PARSE          : 플래시 → USB로 raw 바이너리 덤프
// =============================================================================
void loop() {
  String cmd = getIncomingRaw();
  if (cmd.length() == 0) { vTaskDelay(pdMS_TO_TICKS(50)); return; }
  cmd.toUpperCase();

  // Common Commands
  if (cmd == "REBOOT") {
    sendResponse("REBOOTING...\n");
    if (logger.isEnabled()) {
      logger.setEnabled(false);
      vTaskDelay(pdMS_TO_TICKS(100));
      logger.forceFlushBuffer();
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP.restart();
  }

  // Pre-Flight Commands
  else if (!flightActive) {
    if (cmd == "CALIBRATE") {
      digitalWrite(LED_PIN, HIGH);
      bool imuOk = false, bmpOk = false;
      for (int attempt = 1; attempt <= CALIB_MAX_ATTEMPTS && !(imuOk && bmpOk); attempt++) {
        sendResponse("CALIBRATING IMU+BMP...\n");
        imuOk = imu.calibrate(CALIB_SAMPLES);
        bmpOk = bmp.calibrate(CALIB_SAMPLES);
        clearSensors();

        if (imuOk && bmpOk) {
          sendResponse("CALIBRATION DONE.\n");
          digitalWrite(LED_PIN, LOW); beep(200);
        } else {
          if (!imuOk) sendResponse("IMU NOISY - RETRYING...\n");
          if (!bmpOk) sendResponse("BARO UNSTABLE - RETRYING...\n");
          beep(100, 3); 
          vTaskDelay(pdMS_TO_TICKS(1000));
        }
      }
      if (!(imuOk && bmpOk)) {
        sendResponse("CALIBRATION FAIL\n");
        digitalWrite(LED_PIN, LOW);
      }
    }
    else if (cmd == "CALIBRATE_MAG") {
      digitalWrite(LED_PIN, HIGH);
      bool magOk = false;
      for (int attempt = 1; attempt <= CALIB_MAX_ATTEMPTS && !magOk; attempt++) {
        sendResponse("CALIBRATING MAG (30S)...\n");
        if (mag.calibrate(MAG_CALIB_MS)) {
          mag.clearInterruptFlag();
          magOk = true;
          sendResponse("MAG CALIB DONE.\n");
          digitalWrite(LED_PIN, LOW); beep(200);
        } else {
          sendResponse("INSUFFICIENT ROTATION - RETRYING...\n");
          beep(100, 3);
          vTaskDelay(pdMS_TO_TICKS(2000));
        }
      }
      if (!magOk) {
        sendResponse("MAG CALIB FAIL\n");
        digitalWrite(LED_PIN, LOW);
      }
    }
    else if (cmd == "CALIBRATE_GPS") {
      sendResponse("CALIBRATING GPS (AVERAGING)...\n");
      bool gpsOk = false;
      for (int attempt = 1; attempt <= CALIB_MAX_ATTEMPTS && !gpsOk; attempt++) {
        if (gps.calibrate()) {
          if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(NAV_MUTEX_LONG_WAIT_MS)) == pdTRUE) {
            nav.setLaunchSite(gps.getOriginLatDeg(), gps.getOriginLonDeg());
            xSemaphoreGive(navMutex);
            gpsOk = true;
            sendResponse("GPS ORIGIN OK\n");
            beep(200);
          } else {
            sendResponse("NAV LOCK TIMEOUT\n");
          }
        } else {
          sendResponse("GPS DRIFTING OR NO FIX - RETRYING...\n");
          beep(100, 3);
          vTaskDelay(pdMS_TO_TICKS(2000));
        }
      }
      if (!gpsOk) {
        sendResponse("GPS CALIB FAIL\n");
      }
    }
    else if (cmd == "ERASE") {
      sendResponse("ERASING FLASH...\n");
      logger.eraseAll();
      sendResponse("DONE.\n");
      beep(500);
    }
    else if (cmd == "ZUPT") {
      sendResponse("ALIGNING EKF...\n");
      digitalWrite(LED_PIN, HIGH);

      float P_prev = 0.0f;
      int   stable_count = 0;
      int   iter = 0;
      bool  converged = false;

      for (iter = 0; iter < ZUPT_MAX_ITER; iter++) {
        if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(ZUPT_MUTEX_WAIT_MS)) == pdTRUE) {
          if (!nav.isEkfReady()) nav.ekfBegin();
          nav.ekfUpdateStaticAlignment();
          float P_now = nav.ekf().attBiasCovTrace();
          xSemaphoreGive(navMutex);

          if (iter >= ZUPT_MIN_ITER && P_prev > 0.0f) {
            float rel = fabsf(P_prev - P_now) / P_prev;
            if (rel < ZUPT_REL_THRESH) {
              if (++stable_count >= ZUPT_STABLE_REQ) { converged = true; break; }
            } else {
              stable_count = 0;
            }
          }
          P_prev = P_now;
        }
        vTaskDelay(pdMS_TO_TICKS(ZUPT_PERIOD_MS));
      }

      char buf[80];
      snprintf(buf, sizeof(buf), "ZUPT %s in %.1fs (P_trace=%.3e)\n",
               converged ? "CONVERGED" : "TIMEOUT",
               (iter + 1) * (float)ZUPT_PERIOD_MS * 0.001f, P_prev);
      sendResponse(buf);
      digitalWrite(LED_PIN, LOW); beep(100);
    }
    else if (cmd == "START") {
      sendResponse("STARTING...\n");
      digitalWrite(LED_PIN, HIGH);
      navTestActive = false;

      if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(NAV_MUTEX_LONG_WAIT_MS)) == pdTRUE) {
        nav.ekfReset();
        nav.resetInitAverage();
        xSemaphoreGive(navMutex);
      } else {
        sendResponse("NAV LOCK TIMEOUT\n");
        digitalWrite(LED_PIN, LOW);
        beep(100, 3); return;
      }

      sendResponse("TRIAD AVG...\n");
      vTaskDelay(pdMS_TO_TICKS(START_TRIAD_AVG_MS));

      if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(NAV_MUTEX_LONG_WAIT_MS)) == pdTRUE) {
        if (!nav.ekfBeginAveraged(START_TRIAD_MIN_IMU, START_TRIAD_MIN_MAG)) {
          sendResponse("EKF INIT FAIL\n");
          nav.ekfReset();
          xSemaphoreGive(navMutex);
          digitalWrite(PYRO_1_PIN, LOW);
          digitalWrite(PYRO_2_PIN, LOW);
          digitalWrite(LED_PIN, LOW);
          beep(100, 3); return;
        }
        xSemaphoreGive(navMutex);
      } else {
        sendResponse("NAV LOCK TIMEOUT\n");
        digitalWrite(PYRO_1_PIN, LOW);
        digitalWrite(PYRO_2_PIN, LOW);
        digitalWrite(LED_PIN, LOW);
        beep(100, 3); return;
      }

      sendResponse("ALIGNING EKF...\n");
      float P_prev = 0.0f;
      int stable_count = 0;
      int iter = 0;
      bool converged = false;

      for (iter = 0; iter < ZUPT_MAX_ITER; iter++) {
        if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(ZUPT_MUTEX_WAIT_MS)) == pdTRUE) {
          nav.ekfUpdateStaticAlignment();
          float P_now = nav.ekf().attBiasCovTrace();
          xSemaphoreGive(navMutex);

          if (iter >= ZUPT_MIN_ITER && P_prev > 0.0f) {
            float rel = fabsf(P_prev - P_now) / P_prev;
            if (rel < ZUPT_REL_THRESH) {
              if (++stable_count >= ZUPT_STABLE_REQ) { converged = true; break; }
            } else {
              stable_count = 0;
            }
          }
          P_prev = P_now;
        }
        vTaskDelay(pdMS_TO_TICKS(ZUPT_PERIOD_MS));
      }

      char startBuf[80];
      snprintf(startBuf, sizeof(startBuf), "START ZUPT %s in %.1fs (P_trace=%.3e)\n",
               converged ? "CONVERGED" : "TIMEOUT",
               (iter + 1) * (float)ZUPT_PERIOD_MS * 0.001f, P_prev);
      sendResponse(startBuf);

      if (!converged) {
        if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(NAV_MUTEX_LONG_WAIT_MS)) == pdTRUE) {
          nav.ekfReset();
          xSemaphoreGive(navMutex);
        }
        digitalWrite(PYRO_1_PIN, LOW);
        digitalWrite(PYRO_2_PIN, LOW);
        digitalWrite(LED_PIN, LOW);
        beep(100, 3); return;
      }

      sendResponse("EKF READY\n");

      clearSensors();
      mag.clearInterruptFlag();
      logger.setEnabled(true);
      flightActive = true;
      clearSensors(); 
      digitalWrite(LED_PIN, HIGH);
      beep(300);
      sendResponse("FLIGHT ACTIVE\n");
      xTaskNotifyGive(FlightTaskHandle);
    }
    else if (cmd == "TEST_NAV") {
      sendResponse("TEST_NAV STARTING...\n");
      digitalWrite(LED_PIN, HIGH);
      digitalWrite(PYRO_1_PIN, LOW);
      digitalWrite(PYRO_2_PIN, LOW);
      navTestActive = true;

      if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(NAV_MUTEX_LONG_WAIT_MS)) == pdTRUE) {
        nav.ekfReset();
        nav.resetInitAverage();
        xSemaphoreGive(navMutex);
      } else {
        sendResponse("NAV LOCK TIMEOUT\n");
        navTestActive = false;
        digitalWrite(LED_PIN, LOW);
        beep(100, 3); return;
      }

      sendResponse("TRIAD AVG...\n");
      vTaskDelay(pdMS_TO_TICKS(START_TRIAD_AVG_MS));

      if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(NAV_MUTEX_LONG_WAIT_MS)) == pdTRUE) {
        if (!nav.ekfBeginAveraged(START_TRIAD_MIN_IMU, START_TRIAD_MIN_MAG)) {
          sendResponse("EKF INIT FAIL\n");
          nav.ekfReset();
          xSemaphoreGive(navMutex);
          navTestActive = false;
          digitalWrite(PYRO_1_PIN, LOW);
          digitalWrite(PYRO_2_PIN, LOW);
          digitalWrite(LED_PIN, LOW);
          beep(100, 3); return;
        }
        xSemaphoreGive(navMutex);
      } else {
        sendResponse("NAV LOCK TIMEOUT\n");
        navTestActive = false;
        digitalWrite(PYRO_1_PIN, LOW);
        digitalWrite(PYRO_2_PIN, LOW);
        digitalWrite(LED_PIN, LOW);
        beep(100, 3); return;
      }

      sendResponse("ALIGNING EKF...\n");
      float P_prev = 0.0f;
      int stable_count = 0;
      int iter = 0;
      bool converged = false;

      for (iter = 0; iter < ZUPT_MAX_ITER; iter++) {
        if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(ZUPT_MUTEX_WAIT_MS)) == pdTRUE) {
          nav.ekfUpdateStaticAlignment();
          float P_now = nav.ekf().attBiasCovTrace();
          xSemaphoreGive(navMutex);

          if (iter >= ZUPT_MIN_ITER && P_prev > 0.0f) {
            float rel = fabsf(P_prev - P_now) / P_prev;
            if (rel < ZUPT_REL_THRESH) {
              if (++stable_count >= ZUPT_STABLE_REQ) { converged = true; break; }
            } else {
              stable_count = 0;
            }
          }
          P_prev = P_now;
        }
        vTaskDelay(pdMS_TO_TICKS(ZUPT_PERIOD_MS));
      }

      char testBuf[80];
      snprintf(testBuf, sizeof(testBuf), "TEST_NAV ZUPT %s in %.1fs (P_trace=%.3e)\n",
               converged ? "CONVERGED" : "TIMEOUT",
               (iter + 1) * (float)ZUPT_PERIOD_MS * 0.001f, P_prev);
      sendResponse(testBuf);

      if (!converged) {
        if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(NAV_MUTEX_LONG_WAIT_MS)) == pdTRUE) {
          nav.ekfReset();
          xSemaphoreGive(navMutex);
        }
        navTestActive = false;
        digitalWrite(PYRO_1_PIN, LOW);
        digitalWrite(PYRO_2_PIN, LOW);
        digitalWrite(LED_PIN, LOW);
        beep(100, 3); return;
      }

      clearSensors();
      mag.clearInterruptFlag();
      logger.setEnabled(true);
      flightActive = true;
      clearSensors();
      digitalWrite(LED_PIN, HIGH);
      beep(300);
      sendResponse("NAV TEST ACTIVE\n");
    }
  }

  // Flight/Active Commands
  else {
    if (cmd == "STOP") {
      digitalWrite(PYRO_1_PIN, LOW);
      digitalWrite(PYRO_2_PIN, LOW);
      flightActive = false;
      navTestActive = false;
      if (logger.isEnabled()) {
        logger.setEnabled(false);
        vTaskDelay(pdMS_TO_TICKS(100));
        logger.forceFlushBuffer();
      }
      digitalWrite(LED_PIN, LOW);
      beep(200); vTaskDelay(pdMS_TO_TICKS(50)); beep(200);
      sendResponse("STOPPED.\n");
    }
    else if (cmd == "PARSE") {
      bool wasLogging = logger.isEnabled();
      if (wasLogging) {
        logger.setEnabled(false);
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      sendResponse("DUMP START\n");
      logger.forceFlushBuffer();
      logger.dumpRawBinary(Serial);
      sendResponse("DUMP DONE\n");
      
      if (wasLogging) {
        flightActive = false;
        navTestActive = false;
        digitalWrite(PYRO_1_PIN, LOW);
        digitalWrite(PYRO_2_PIN, LOW);
        digitalWrite(LED_PIN, LOW);
      }
    }
  }

  vTaskDelay(pdMS_TO_TICKS(50));
}

// =============================================================================
// Flight_Task : 비행 단계 자동 검출 + 파이로 점화 (Core 0, 10Hz)
//   상태머신 : PRE_FLIGHT → POWERED_FLIGHT → COASTING → DESCENT → LANDED
//   각 단계 전이 조건은 Config.h의 LAUNCH_*, BURNOUT_*, APOGEE_*, LANDED_* 참조
//
//   - DESCENT 진입 시 Pyro1 HIGH → PYRO1_FIRE_MS(1초) 후 자동 LOW
//   - START 명령에서 xTaskNotifyGive로 깨워짐, STOP 시 flightActive=false로 빠져나옴
// =============================================================================
void Flight_Task(void *pvParameters) {
  uint32_t pyro1_start_ms = 0;
  bool pyro1_active = false;

  for (;;) {
    // wait START
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (navTestActive) continue;
    
    flightPhase = FlightPhase::PRE_FLIGHT;
    bool reached_high_g = false;
    uint8_t launch_count = 0;
    uint8_t descent_count = 0;
    uint32_t launch_ms = 0;
    uint32_t coast_ms = 0;
    pyro1_active = false;
    digitalWrite(PYRO_1_PIN, LOW);
    digitalWrite(PYRO_2_PIN, LOW);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (flightActive) {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(FLIGHT_TASK_PERIOD_MS));

      uint32_t now_ms = millis();
      if (pyro1_active && (now_ms - pyro1_start_ms > PYRO1_FIRE_MS)) {
        digitalWrite(PYRO_1_PIN, LOW);
        pyro1_active = false;
      }

      State_nominal s;
      State_imu imu_state;
      if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(FLIGHT_NAV_MUTEX_WAIT_MS)) == pdTRUE) {
        s = nav.getNominal();
        imu_state = nav.getStateImu();
        xSemaphoreGive(navMutex);
      } else continue;

      float alt = -s.p[2];
      float vel_up = -s.v[2];
      
      // accel norm
      float ax = imu_state.ax - s.ba[0];
      float ay = imu_state.ay - s.ba[1];
      float az = imu_state.az - s.ba[2];
      float amag = sqrtf(ax*ax + ay*ay + az*az);

      switch (flightPhase) {
        case FlightPhase::PRE_FLIGHT:
          if (alt > LAUNCH_ALT_M || vel_up > LAUNCH_VEL_UP_MPS || amag > LAUNCH_ACCEL_MPS2) {
            launch_count++;
          } else {
            launch_count = 0;
          }

          if (launch_count >= LAUNCH_CONFIRM_COUNT) {
            flightPhase = FlightPhase::POWERED_FLIGHT;
            launch_ms = now_ms;
            reached_high_g = false;
            logger.logEvent(flightPhase, 1);
            sendResponse("LAUNCH\n");
          }
          break;

        case FlightPhase::POWERED_FLIGHT:
          if (amag > BURNOUT_HIGH_ACCEL_MPS2) reached_high_g = true;
          
          if ((reached_high_g && amag < BURNOUT_LOW_ACCEL_MPS2) ||
              (now_ms - launch_ms > BURNOUT_TIMEOUT_MS) ||
              (vel_up > BURNOUT_VEL_UP_MIN_MPS && amag < BURNOUT_VEL_ACCEL_MAX_MPS2)) {
            flightPhase = FlightPhase::COASTING;
            coast_ms = now_ms;
            descent_count = 0;
            logger.logEvent(flightPhase, 2);
            sendResponse("BO\n");
            reached_high_g = false;
          }
          break;

        case FlightPhase::COASTING: {
          if ((now_ms - coast_ms > APOGEE_ARM_MS) &&
              vel_up < APOGEE_VEL_UP_MPS &&
              alt > APOGEE_MIN_ALT_M) {
            descent_count++;
            if (descent_count >= APOGEE_CONFIRM_COUNT) {
              flightPhase = FlightPhase::DESCENT;
              logger.logEvent(flightPhase, 3);
              sendResponse("APG\n");
              
              digitalWrite(PYRO_1_PIN, HIGH);
              pyro1_start_ms = millis();
              pyro1_active = true;
              
              descent_count = 0;
            }
          } else {
            descent_count = 0;
          }
          break;
        }

        case FlightPhase::DESCENT:
          if (alt < LANDED_ALT_M && fabsf(vel_up) < LANDED_VEL_UP_ABS_MPS) {
            flightPhase = FlightPhase::LANDED;
            logger.logEvent(flightPhase, 4);
            digitalWrite(PYRO_1_PIN, LOW);
            digitalWrite(PYRO_2_PIN, LOW);
            pyro1_active = false;
            sendResponse("LAND\n");
          }
          break;

        case FlightPhase::LANDED:
          digitalWrite(PYRO_1_PIN, LOW);
          digitalWrite(PYRO_2_PIN, LOW);
          pyro1_active = false;
          break;
      }
    }

    digitalWrite(PYRO_1_PIN, LOW);
    digitalWrite(PYRO_2_PIN, LOW);
    pyro1_active = false;
  }
}

// =============================================================================
// 센서 태스크들 — 각 센서가 인터럽트/폴링으로 트리거되면 EKF에 데이터 주입
// =============================================================================

// ----------------------------------------------------------------------------
// IMU_Task (Core 1, prio 5)
//   - IMU INT1 RISING 인터럽트로 깨어남 (~416Hz)
//   - readCalibratedIMU → ekf.predictAdaptiveJerk → State_nominal 로그
//   - flightActive 중에는 raw IMU 패킷도 별도 로깅
// ----------------------------------------------------------------------------
void IMU_Task(void *pvParameters) {
  Raw_imu raw;
  State_nominal stateToLog;
  uint32_t lastStateLogMs = 0;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
      raw.timestamp = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
      imu.readCalibratedIMU(raw.gx, raw.gy, raw.gz, raw.ax, raw.ay, raw.az);
      xSemaphoreGive(spiMutex);
    }
    
    if (flightActive) logger.logImu(raw);

    bool logStateNow = false;
    const uint32_t nowMs = millis();
    if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(NAV_MUTEX_FAST_WAIT_MS)) == pdTRUE) {
      nav.updateIMU(raw);
      if (flightActive && nav.isEkfReady() &&
          (uint32_t)(nowMs - lastStateLogMs) >= STATE_LOG_PERIOD_MS) {
        stateToLog = nav.getNominal();
        lastStateLogMs = nowMs;
        logStateNow = true;
      }
      xSemaphoreGive(navMutex);
    }
    if (logStateNow) logger.logState(stateToLog);
  }
}

// ----------------------------------------------------------------------------
// BMP_Task (Core 0, prio 4)
//   - 기압계 INT RISING 인터럽트로 깨어남 (~50Hz)
//   - readAltitude → ekf.updateBaro (수직 위치 1차원 측정)
// ----------------------------------------------------------------------------
void BMP_Task(void *pvParameters) {
  Raw_press p;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (xSemaphoreTake(spiMutex, portMAX_DELAY) == pdTRUE) {
      p.timestamp = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
      bmp.readAltitude(p.alt);
      xSemaphoreGive(spiMutex);
    }
    
    if (flightActive) logger.logBaro(p);

    if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(NAV_MUTEX_FAST_WAIT_MS)) == pdTRUE) {
      nav.updatePress(p);
      xSemaphoreGive(navMutex);
    }
  }
}

// ----------------------------------------------------------------------------
// MAG_Task (Core 0, prio 4)
//   - 자력계는 인터럽트 핀이 없거나 노이즈가 많아 폴링 (MAG_POLL_MS=10ms)
//   - isDataReady() 체크 후 읽고, ekf.updateMag로 yaw 보정
// ----------------------------------------------------------------------------
void MAG_Task(void *pvParameters) {
  Raw_mag m;
  for (;;) {
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
    
    if (flightActive) logger.logMag(m);
    
    if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(NAV_MUTEX_FAST_WAIT_MS)) == pdTRUE) {
      nav.updateMag(m); 
      xSemaphoreGive(navMutex);
    }
  }
}

// ----------------------------------------------------------------------------
// GPS_Task (Core 0, prio 3)
//   - UBX 프레임을 GPS_POLL_MS(20ms) 간격으로 파싱
//   - NAV-PVT 메시지 수신 시 getNED → ekf.updateGps (위치+속도 6차원 측정)
//   - origin 미설정 또는 fix < 3이면 EKF 갱신은 스킵
// ----------------------------------------------------------------------------
void GPS_Task(void *pvParameters) {
  Raw_gps g;
  for (;;) {
    if (gps.update()) {
      g.timestamp = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
      g.hasPos    = gps.getNED(g.pn, g.pe, g.pd, g.vn, g.ve, g.vd, g.hAcc, g.vAcc, g.sAcc, g.fixType, g.numSV);
      
      if (flightActive) logger.logGps(g);
      
      if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(NAV_MUTEX_FAST_WAIT_MS)) == pdTRUE) {
        nav.updateGps(g); 
        xSemaphoreGive(navMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(GPS_POLL_MS));
  }
}

// ----------------------------------------------------------------------------
// FlushTask (Core 0, prio 2 - 가장 낮음)
//   - 로깅 큐에서 패킷을 꺼내 페이지 버퍼에 누적 → 256B 페이지 채워지면 플래시 쓰기
//   - 센서 태스크보다 우선순위가 낮아 데이터 수집을 절대 막지 않음
// ----------------------------------------------------------------------------
void FlushTask(void *pvParameters) {
  for (;;) { logger.serviceFlush(); }
}

// Utilities
void beep(int ms, int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(ms));
    digitalWrite(BUZZER_PIN, LOW);
    if (i < count - 1) vTaskDelay(pdMS_TO_TICKS(ms));
  }
}

void clearSensors() {
  int16_t d1, d2, d3, d4, d5, d6; float df;
  if (xSemaphoreTake(spiMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    imu.readRawIMU(d1, d2, d3, d4, d5, d6);
    bmp.readData(df);
    xSemaphoreGive(spiMutex);
  }
}

// ----------------------------------------------------------------------------
// 인터럽트 ISR (IRAM에 상주, 가능한 짧게)
//   - 센서 데이터레디 핀이 RISING 될 때 vTaskNotifyGiveFromISR로
//     해당 태스크를 깨우기만 함. 실제 SPI 읽기는 태스크 컨텍스트에서.
// ----------------------------------------------------------------------------
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
