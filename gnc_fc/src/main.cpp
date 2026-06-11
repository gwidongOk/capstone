/**
 * @file main.cpp
 * Unified Flight Software — Avionics + NAV + GNC Integration.
 *
 * Core 1: flight_task (400Hz) — sensor → EKF → GNC → servo → log
 * Core 0: gps_task, flush_task, loop() (BLE commands)
 * ESP32-S3 dual-core, PlatformIO Arduino + FreeRTOS.
 *
 * Pre-flight sequence (from avionics):
 *   CALIBRATE_STATIC → CALIBRATE_MAG → READY → START
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <esp_timer.h>

#include "BLE.h"
#include "BMP388.h"
#include "LSM6DSO32.h"
#include "MMC5983MA.h"
#include "MX25Logger.h"
#include "NAV.h"
#include "NEOM9N.h"
#include "board_config.h"
#include "sensor_data.h"
#include "servo_hal.h"

extern "C" {
#include "flight_phase.h"
#include "gnc_config.h"
#include "gnc_types.h"
#include "recovery.h"
}
#include "gnc_main.h"

#ifdef PLAYBACK_DATA
#include "playback_data.h"
#endif

/* ---- Hardware objects ---- */
static SPIClass sensorSPI(HSPI);
static SPIClass flashSPI(FSPI);
static SemaphoreHandle_t flashMutex = NULL;

static LSM6DSO32 imu(IMU_CS_PIN, &sensorSPI);
static BMP388 bmp(BMP_CS_PIN, &sensorSPI);
static MMC5983MA mag(&Wire);
static NEOM9N gps(Serial1, GPS_RX_PIN, GPS_TX_PIN);
static NAV nav;
static MX25Logger logger;

/* ---- GNC state ---- */
static gnc_config_t gnc_cfg;
static gnc_state_t gnc_state;
static phase_state_t phase_state;
static recovery_state_t rcv_state;

/* ---- GPS double buffer (Core 0 → Core 1, lock-free) ---- */
static Raw_gps gps_buf[2] = {};
static volatile int gps_write_idx = 0;

/* ---- Flight control flags ---- */
static volatile bool flightActive = false;
static volatile bool zupt_in_progress = false;
static volatile bool launched = false;
static volatile bool alignReady = false; /* READY 완료 (EKF 수렴됨) */
static volatile bool padAlignActive =
    false; /* 정렬 유지 중 (ZUPT 주기적 호출) */
static volatile bool navTestActive =
    false; /* NAV_TEST 모드 (서보/파이로 비활성) */
static volatile bool desk_test_mode = false;
static volatile bool desk_test_start_pending = false;
static volatile bool desk_test_align_requested = false;

/* ---- Pyro state (driven by recovery module) ---- */
static bool pyro1_active = false;
static bool pyro2_active = false;
static uint32_t pyro1_start_ms = 0;
static uint32_t pyro2_start_ms = 0;

/* ---- Task handles ---- */
static TaskHandle_t flightTaskHandle = NULL;
static TaskHandle_t gpsTaskHandle = NULL;
static TaskHandle_t flushTaskHandle = NULL;

static void beep(uint32_t duration_ms, uint8_t count = 1) {
  for (uint8_t i = 0; i < count; i++) {
    if (i > 0)
      delay(100);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(duration_ms);
    digitalWrite(BUZZER_PIN, LOW);
  }
}

/* ---- flight_phase_t → FlightPhase (logging) mapping ---- */
static FlightPhase phase_to_log(flight_phase_t p) {
  switch (p) {
  case PHASE_RAIL:
    return FlightPhase::PRE_FLIGHT;
  case PHASE_BOOST:
    return FlightPhase::POWERED_FLIGHT;
  case PHASE_GUIDE:
    return FlightPhase::COASTING;
  case PHASE_CPA:
    return FlightPhase::COASTING;
  case PHASE_DESCENT:
    return FlightPhase::DESCENT;
  case PHASE_LANDED:
    return FlightPhase::LANDED;
  default:
    return FlightPhase::PRE_FLIGHT;
  }
}

/* ---- NAV → GNC state conversion ---- */
static nav_state_t build_nav_state(const State_nominal &nom,
                                   const State_imu &si, uint32_t t_ms,
                                   bool ekf_ready) {
  nav_state_t ns = {};
  ns.pos_ned = {nom.p[0], nom.p[1], nom.p[2]};
  ns.vel_ned = {nom.v[0], nom.v[1], nom.v[2]};
  ns.quat = {nom.q[0], nom.q[1], nom.q[2], nom.q[3]};
  ns.omega_b = {si.gx - nom.bg[0], si.gy - nom.bg[1], si.gz - nom.bg[2]};
  ns.accel_b = {si.ax - nom.ba[0], si.ay - nom.ba[1], si.az - nom.ba[2]};
  ns.airspeed =
      sqrtf(nom.v[0] * nom.v[0] + nom.v[1] * nom.v[1] + nom.v[2] * nom.v[2]);
  ns.timestamp_ms = t_ms;
  ns.valid = ekf_ready;
  return ns;
}

/* ---- Pyro GPIO driver (avionics pattern) ----
 * Called at 10Hz from flight_task (every 40 cycles).
 * On rising edge of drogue_fired / main_fired:
 *   set GPIO HIGH, record start time.
 * Every tick: if pulse duration exceeded, set GPIO LOW.
 * One-shot: fires exactly once per flag. */
static void pyro_tick(const recovery_state_t *rcv) {
  uint32_t now_ms = millis();

  /* Auto-off: check pulse timeout first (runs regardless of new triggers) */
  if (pyro1_active && (now_ms - pyro1_start_ms > PYRO1_FIRE_MS)) {
    digitalWrite(PYRO_1_PIN, LOW);
    pyro1_active = false;
  }
  if (pyro2_active && (now_ms - pyro2_start_ms > PYRO2_FIRE_MS)) {
    digitalWrite(PYRO_2_PIN, LOW);
    pyro2_active = false;
  }

  /* Drogue: PYRO_1 — rising edge of drogue_fired */
  if (rcv->drogue_fired && !pyro1_active && pyro1_start_ms == 0) {
    digitalWrite(PYRO_1_PIN, HIGH);
    pyro1_start_ms = now_ms;
    pyro1_active = true;
    logger.logEvent(FlightPhase::DESCENT, 3);
    sendResponse("DROGUE\n");
  }

  /* Main: PYRO_2 — rising edge of main_fired */
  if (rcv->main_fired && !pyro2_active && pyro2_start_ms == 0) {
    digitalWrite(PYRO_2_PIN, HIGH);
    pyro2_start_ms = now_ms;
    pyro2_active = true;
    logger.logEvent(FlightPhase::DESCENT, 4);
    sendResponse("MAIN\n");
  }
}

/* ---- GPS Task (Core 0) ---- */
static void gps_task(void *arg) {
  (void)arg;
  for (;;) {
    if (gps.update()) {
      Raw_gps g = {};
      g.timestamp = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
      g.hasPos = gps.getNED(g.pn, g.pe, g.pd, g.vn, g.ve, g.vd, g.hAcc, g.vAcc,
                            g.sAcc, g.fixType, g.numSV);
      int idx = 1 - gps_write_idx;
      gps_buf[idx] = g;
      __atomic_store_n(&gps_write_idx, idx, __ATOMIC_RELEASE);
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

/* ---- Flush Task (Core 0) ---- */
static void flush_task(void *arg) {
  (void)arg;
  for (;;) {
    logger.serviceFlush();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

/* ---- Flight Task (Core 1, 400Hz) ---- */
static void flight_task(void *arg) {
  (void)arg;

  uint32_t cycle = 0;
  float mission_time = 0.0f;
  uint32_t launch_time_us = 0;
  uint32_t last_gps_ts = 0;
  uint32_t last_zupt_ms = 0;
  static float last_baro_alt = 0.0f;
  uint8_t launch_confirm_count = 0;
#ifdef PLAYBACK_DATA
  uint32_t pb_idx = 0;
#endif

  /* Seed NAV timestamp */
  {
    int16_t gx, gy, gz, ax, ay, az;
    imu.readCalibratedIMU(gx, gy, gz, ax, ay, az);
    Raw_imu r = {};
    r.timestamp = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
    r.gx = gx;
    r.gy = gy;
    r.gz = gz;
    r.ax = ax;
    r.ay = ay;
    r.az = az;
    nav.updateIMU(r);
  }

  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2));
    uint32_t t_us = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
    uint32_t t_ms = t_us / 1000;

    /* Skip if ZUPT/calibration in progress */
    if (zupt_in_progress) {
      cycle++;
      continue;
    }

    /* ---- Desk test alignment (simplified, GNC-included bench test) ---- */
    if (desk_test_align_requested) {
      /* EKF init */
      if (!nav.isEkfReady()) {
        if (!nav.ekfBegin()) {
          Serial.println("TRIAD failed, using accel-only init");
          State_imu si = nav.getStateImu();
          float ax_v = si.ax, ay_v = si.ay, az_v = si.az;
          float pitch = atan2f(-ax_v, sqrtf(ay_v * ay_v + az_v * az_v));
          float roll = atan2f(ay_v, -az_v);
          float yaw = 0.0f;
          float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
          float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
          float cy = cosf(yaw * 0.5f), sy = sinf(yaw * 0.5f);
          float q0[4] = {
              cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
              cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy};
          float p0[3] = {0, 0, 0};
          float v0[3] = {0, 0, 0};
          nav.ekfBegin(p0, v0, q0);
        }
      }

      /* ZUPT convergence */
      const int MAX_ITER = 200;
      const float REL_THRESH = 1e-3f;
      const int STABLE_REQ = 3;
      float P_prev = 0.0f;
      int stable_count = 0;
      for (int iter = 0; iter < MAX_ITER; iter++) {
        int16_t gx, gy, gz, ax, ay, az;
        imu.readCalibratedIMU(gx, gy, gz, ax, ay, az);
        Raw_imu r = {};
        r.timestamp = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
        r.gx = gx;
        r.gy = gy;
        r.gz = gz;
        r.ax = ax;
        r.ay = ay;
        r.az = az;
        nav.updateIMU(r);
        nav.ekfUpdateStaticAlignment();
        float P_now = nav.ekf().attBiasCovTrace();
        if (iter >= 10 && P_prev > 0.0f) {
          float rel = fabsf(P_prev - P_now) / P_prev;
          if (rel < REL_THRESH) {
            if (++stable_count >= STABLE_REQ)
              break;
          } else {
            stable_count = 0;
          }
        }
        P_prev = P_now;
        vTaskDelay(pdMS_TO_TICKS(50));
      }

      Serial.println("EKF ALIGNED!");
      gnc_init(&gnc_state, &phase_state, &rcv_state);
      phase_state.phase = PHASE_GUIDE; /* ACTIVE phase for desk test */
      phase_state.t_phase_enter = 0.0f;
      desk_test_mode = true;
      flightActive = true;
      launched = true;
      desk_test_start_pending = true;
      desk_test_align_requested = false;
#ifdef PLAYBACK_DATA
      pb_idx = 0;
      Serial.printf("PLAYBACK: %d samples (%.2f s)\n",
                    PLAYBACK_N_SAMPLES,
                    PLAYBACK_N_SAMPLES * PLAYBACK_DT_MS * 0.001f);
#endif
      cycle++;
      continue;
    }

    /* ================================================================
     * 1. SENSE
     * ================================================================ */

    /* IMU: every cycle */
    Raw_imu imu_raw = {};
    {
      int16_t gx, gy, gz, ax, ay, az;
      imu.readCalibratedIMU(gx, gy, gz, ax, ay, az);
      imu_raw.timestamp = t_us;
      imu_raw.gx = gx;
      imu_raw.gy = gy;
      imu_raw.gz = gz;
      imu_raw.ax = ax;
      imu_raw.ay = ay;
      imu_raw.az = az;
    }

    /* Baro: every 8 cycles (~50Hz) */
    bool baro_new = (cycle % 8 == 4);
    Raw_press press = {};
    if (baro_new) {
      float alt;
      if (bmp.readAltitude(alt)) {
        press.timestamp = t_us;
        press.alt = alt;
        last_baro_alt = alt;
      } else {
        baro_new = false;
      }
    }

    /* Mag: every 8 cycles (~50Hz) */
    bool mag_new = (cycle % 8 == 4);
    Raw_mag mag_raw = {};
    if (mag_new && mag.isDataReady()) {
      float mx, my, mz;
      if (mag.readCalibratedMag(mx, my, mz)) {
        mag_raw.timestamp = t_us;
        mag_raw.mx = mx;
        mag_raw.my = my;
        mag_raw.mz = mz;
        mag.clearInterruptFlag();
      } else {
        mag_new = false;
      }
    } else {
      mag_new = false;
    }

    /* GPS: atomic read from double buffer */
    Raw_gps gps_data;
    {
      int idx = __atomic_load_n(&gps_write_idx, __ATOMIC_ACQUIRE);
      gps_data = (Raw_gps)gps_buf[idx];
    }
    bool gps_new = (gps_data.timestamp != last_gps_ts && gps_data.hasPos);
    if (gps_new)
      last_gps_ts = gps_data.timestamp;

    /* ================================================================
     * 2. NAVIGATE
     * ================================================================ */
    if (gps_new)
      nav.updateGps(gps_data);
    if (baro_new)
      nav.updatePress(press);
    if (mag_new)
      nav.updateMag(mag_raw);
    nav.updateIMU(imu_raw);

    /* ================================================================
     * 3. PAD ALIGNMENT MAINTENANCE
     *    READY 후 START 전까지 EKF 드리프트 방지를 위해
     *    ZUPT를 주기적으로 호출하고, 발사 가속도 감시
     * ================================================================ */
    if (padAlignActive && !flightActive && nav.isEkfReady()) {
      State_imu si = nav.getStateImu();
      State_nominal nom = nav.getNominal();
      float acx = si.ax - nom.ba[0];
      float acy = si.ay - nom.ba[1];
      float acz = si.az - nom.ba[2];
      float amag = sqrtf(acx * acx + acy * acy + acz * acz);

      if (amag > LAUNCH_ACCEL_MPS2) {
        /* 예상치 못한 움직임 — 정렬 무효화 */
        padAlignActive = false;
        alignReady = false;
      } else if (t_ms - last_zupt_ms >= ZUPT_PERIOD_MS) {
        nav.ekfUpdateStaticAlignment();
        last_zupt_ms = t_ms;
      }
    }

    /* Pre-flight: idle until flightActive */
    if (!flightActive) {
      if (cycle % 100 == 0) {
        logger.logImu(imu_raw);
        if (nav.isEkfReady())
          logger.logState(nav.getNominal());
      }
      cycle++;
      continue;
    }

    /* ================================================================
     * 4. LAUNCH DETECTION
     *    avionics 패턴: 연속 확인 카운터
     * ================================================================ */
    if (!launched) {
      State_imu si = nav.getStateImu();
      State_nominal nom = nav.getNominal();
      float acx = si.ax - nom.ba[0];
      float acy = si.ay - nom.ba[1];
      float acz = si.az - nom.ba[2];
      float amag = sqrtf(acx * acx + acy * acy + acz * acz);
      float alt = -nom.p[2];
      float vel_up = -nom.v[2];

      bool launch_cond = (amag > LAUNCH_ACCEL_MPS2 || alt > LAUNCH_ALT_M ||
                          vel_up > LAUNCH_VEL_UP_MPS);

      if (launch_cond) {
        launch_confirm_count++;
      } else {
        launch_confirm_count = 0;
      }

      if (launch_confirm_count >= LAUNCH_CONFIRM_COUNT) {
        launched = true;
        padAlignActive = false;
        launch_time_us = t_us;
        mission_time = 0.0f;
        launch_confirm_count = 0;
        logger.logEvent(FlightPhase::POWERED_FLIGHT, 1);
        sendResponse("LAUNCH\n");
      }

      logger.logImu(imu_raw);
      if (baro_new)
        logger.logBaro(press);
      if (mag_new)
        logger.logMag(mag_raw);
      if (gps_new)
        logger.logGps(gps_data);
      cycle++;
      continue;
    }

    /* Desk test sync */
    if (desk_test_start_pending) {
      launch_time_us = t_us;
      mission_time = 0.0f;
      desk_test_start_pending = false;
    }

    /* ================================================================
     * 5. TIME — update before GNC so phase transitions use current time
     * ================================================================ */
    mission_time = (float)(t_us - launch_time_us) * 1e-6f;

    /* ================================================================
     * 6. NAV → GNC
     * ================================================================ */
#ifdef PLAYBACK_DATA
    /* Playback mode: read nav_state from pre-recorded array.
     * Overrides EKF output when desk_test_mode is active. */
    nav_state_t nav_gnc;
    if (desk_test_mode && pb_idx < PLAYBACK_N_SAMPLES) {
      const playback_sample_t *s = &playback_data[pb_idx];
      nav_gnc.pos_ned  = (vec3_t){s->pos_n, s->pos_e, s->pos_d};
      nav_gnc.vel_ned  = (vec3_t){s->vel_n, s->vel_e, s->vel_d};
      nav_gnc.quat     = (quat_t){s->q0, s->q1, s->q2, s->q3};
      nav_gnc.omega_b  = (vec3_t){s->p, s->q, s->r};
      nav_gnc.accel_b  = (vec3_t){0.0f, 0.0f, 0.0f};
      nav_gnc.wind_est = (vec3_t){0.0f, 0.0f, 0.0f};
      nav_gnc.airspeed = s->airspeed;
      nav_gnc.timestamp_ms = t_ms;
      nav_gnc.valid    = true;

      /* Override phase from playback data */
      phase_state.phase = (flight_phase_t)s->phase;

      pb_idx++;

      /* End of playback: stop servos */
      if (pb_idx >= PLAYBACK_N_SAMPLES) {
        servo_center_all();
        sendResponse("PLAYBACK DONE\n");
      }
    } else {
      State_nominal nom = nav.getNominal();
      State_imu si = nav.getStateImu();
      nav_gnc = build_nav_state(nom, si, t_ms, nav.isEkfReady());
    }
#else
    State_nominal nom = nav.getNominal();
    State_imu si = nav.getStateImu();
    nav_state_t nav_gnc = build_nav_state(nom, si, t_ms, nav.isEkfReady());
#endif

    /* ================================================================
     * 7. GNC (includes phase + recovery sequencer)
     * ================================================================ */
    gnc_output_t gnc_out = gnc_step(&nav_gnc, &gnc_cfg, &gnc_state,
                                    &phase_state, &rcv_state, mission_time,
                                    last_baro_alt);

    if (desk_test_mode) {
#ifdef PLAYBACK_DATA
      /* Playback: phase comes from array, already set before gnc_step */
      gnc_out.phase = phase_state.phase;
#else
      phase_state.phase = PHASE_GUIDE; /* Hold ACTIVE phase during desk test */
      gnc_out.phase = PHASE_GUIDE;
#endif
    }

    /* ================================================================
     * 8. ACTUATE (skip in NAV_TEST mode)
     * ================================================================ */
    if (!navTestActive) {
      servo_write(&gnc_out.servo);
    }

    /* ================================================================
     * 9. RECOVERY: drive pyro GPIOs from recovery module flags
     *    10Hz tick (every 40 cycles). Skip in desk test / NAV_TEST.
     * ================================================================ */
    if (!desk_test_mode && !navTestActive && (cycle % 40 == 0)) {
      pyro_tick(&gnc_out.recovery);
    }

    /* ================================================================
     * 10. LANDING — react to PHASE_LANDED from state machine
     *    (flight_phase.c handles DESCENT→LANDED transition)
     * ================================================================ */
    if (gnc_out.phase == PHASE_LANDED && !desk_test_mode && !navTestActive) {
      flightActive = false;
      launched = false;
      servo_center_all();
      logger.logEvent(FlightPhase::LANDED, 5);
      logger.setEnabled(false);
      vTaskDelay(pdMS_TO_TICKS(100));
      logger.forceFlushBuffer();
      digitalWrite(LED_PIN, LOW);
      sendResponse("LAND\n");
    }

    /* ================================================================
     * 11. TELEMETRY (10Hz serial)
     * ================================================================ */
    if (flightActive && (cycle % 40 == 0)) {
#ifdef PLAYBACK_DATA
      if (desk_test_mode) {
        Serial.printf("[ PB ] Idx: %d/%d | V: %.1f m/s | Phase: %d\n",
                      pb_idx, PLAYBACK_N_SAMPLES, nav_gnc.airspeed,
                      (int)phase_state.phase);
      } else {
        State_imu si_tl = nav.getStateImu();
        State_nominal nom_tl = nav.getNominal();
        Serial.printf("[SENS] Gyro: %5.2f %5.2f %5.2f | Accel: %6.2f %6.2f %6.2f "
                      "| Baro: %.2f m\n",
                      si_tl.gx, si_tl.gy, si_tl.gz, si_tl.ax, si_tl.ay, si_tl.az,
                      last_baro_alt);
        Serial.printf("[ NAV] Pos: [%6.1f, %6.1f, %6.1f] | Vel: [%5.1f, %5.1f, "
                      "%5.1f] | Valid: %d | T: %.2f s\n",
                      nom_tl.p[0], nom_tl.p[1], nom_tl.p[2], nom_tl.v[0],
                      nom_tl.v[1], nom_tl.v[2], nav.isEkfReady(), mission_time);
      }
#else
      Serial.printf("[SENS] Gyro: %5.2f %5.2f %5.2f | Accel: %6.2f %6.2f %6.2f "
                    "| Baro: %.2f m\n",
                    si.gx, si.gy, si.gz, si.ax, si.ay, si.az, last_baro_alt);
      Serial.printf("[ NAV] Pos: [%6.1f, %6.1f, %6.1f] | Vel: [%5.1f, %5.1f, "
                    "%5.1f] | Valid: %d | T: %.2f s\n",
                    nom.p[0], nom.p[1], nom.p[2], nom.v[0], nom.v[1], nom.v[2],
                    nav.isEkfReady(), mission_time);
#endif
      Serial.printf(
          "[ GNC] Phase: %-7s | Cmd(Nz,Ny): %5.2f, %5.2f | Fins(deg): %6.1f "
          "%6.1f %6.1f %6.1f\n",
          phase_name(gnc_out.phase), gnc_out.guid.nz_cmd, gnc_out.guid.ny_cmd,
          gnc_out.ap.fin_cmd[0] * RAD2DEG_F, gnc_out.ap.fin_cmd[1] * RAD2DEG_F,
          gnc_out.ap.fin_cmd[2] * RAD2DEG_F, gnc_out.ap.fin_cmd[3] * RAD2DEG_F);
      Serial.printf("[ RCV] Drogue: %d | Main: %d\n\n",
                    gnc_out.recovery.drogue_fired, gnc_out.recovery.main_fired);
    }

    /* ================================================================
     * 12. LOGGING
     * ================================================================ */
    logger.logImu(imu_raw);
    if (baro_new)
      logger.logBaro(press);
    if (mag_new)
      logger.logMag(mag_raw);
    if (gps_new)
      logger.logGps(gps_data);
    if (cycle % 8 == 0) {
#ifdef PLAYBACK_DATA
      if (!desk_test_mode) {
        State_nominal nom_lg = nav.getNominal();
        logger.logState(nom_lg);
      }
#else
      logger.logState(nom);
#endif
      logger.logGnc(gnc_out.guid.nz_cmd, gnc_out.guid.ny_cmd,
                    gnc_out.ap.fin_cmd, (uint8_t)gnc_out.phase);
    }

    /* 13. CYCLE */
    cycle++;
  }
}

/* ================================================================
 *  setup()
 *  부트 초기화: 핀 → 통신(BLE/SPI/I2C) → 센서 begin → 플래시 → 서보
 *             → GNC config → RTOS tasks
 * ================================================================ */
void setup() {
  Serial.begin(SERIAL_BAUD);

  /* 1. Pins & Feedback */
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(PYRO_1_PIN, OUTPUT);
  pinMode(PYRO_2_PIN, OUTPUT);
  pinMode(SERVO_1_PIN, OUTPUT);
  pinMode(SERVO_2_PIN, OUTPUT);
  pinMode(SERVO_3_PIN, OUTPUT);
  pinMode(SERVO_4_PIN, OUTPUT);

  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(PYRO_1_PIN, LOW);
  digitalWrite(PYRO_2_PIN, LOW);

  /* 2. Mutex */
  flashMutex = xSemaphoreCreateMutex();

  /* 3. Communications */
  initBLE(BLE_DEVICE_NAME);
  sensorSPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
  Wire.begin(MAG_SDA_PIN, MAG_SCL_PIN, 400000);

  /* 4. Sensors */
  bool ok = true;
  if (!imu.begin()) {
    Serial.println("IMU FAIL");
    sendResponse("INIT FAIL: IMU\n");
    ok = false;
  }
  if (!bmp.begin()) {
    Serial.println("BMP FAIL");
    sendResponse("INIT FAIL: BARO\n");
    ok = false;
  }
  if (!mag.begin()) {
    Serial.println("MAG FAIL");
    sendResponse("INIT FAIL: MAG\n");
    ok = false;
  }
  bool gps_ok = gps.begin(GPS_RATE_HZ);
  if (!gps_ok) {
    Serial.println("GPS WARN (non-fatal)");
    sendResponse("INIT WARN: GPS (non-fatal)\n");
  }
  if (!ok) {
    sendResponse("SENSOR INIT FAILED — CHECK HARDWARE\n");
    beep(100, 3);
    while (1) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
  Serial.println("SENSORS READY");
  sendResponse("SENSORS READY\n");

  /* 5. Storage */
  logger.begin(&flashSPI, FLASH_SCK_PIN, FLASH_MISO_PIN, FLASH_MOSI_PIN,
               FLASH_CS_PIN, flashMutex);

  /* 6. Servo */
  servo_init();
  servo_center_all();

  /* 7. GNC config */
  gnc_cfg = gnc_config_default();
  gnc_init(&gnc_state, &phase_state, &rcv_state);

  /* 8. RTOS tasks */
  xTaskCreatePinnedToCore(flight_task, "flight", STACK_SIZE_FLIGHT, NULL,
                          TASK_C1_PRIO_FLIGHT, &flightTaskHandle, 1);
  xTaskCreatePinnedToCore(gps_task, "gps", STACK_SIZE_GPS, NULL,
                          TASK_C0_PRIO_GPS, &gpsTaskHandle, 0);
  xTaskCreatePinnedToCore(flush_task, "flush", STACK_SIZE_FLUSH, NULL,
                          TASK_C0_PRIO_FLUSH, &flushTaskHandle, 0);

  beep(200);
  sendResponse(">>> READY\n");
}

/* ================================================================
 *  loop() — BLE/USB 명령어 디스패처 (Core 0)
 *
 *  발사 전 시퀀스:
 *    CALIBRATE_STATIC : IMU + Baro + GPS 정적 보정
 *    CALIBRATE_MAG    : 자력계 8자 회전 보정 (30초)
 *    READY            : TRIAD 평균 + ZUPT 발사각 정렬
 *    START            : 정렬 필수 → 로깅 시작 → 발사 감지
 *    NAV_TEST         : 항법 only (서보/파이로 비활성)
 *    DESK_TEST        : GNC 포함 벤치 테스트
 *
 *  공용: REBOOT, ERASE, PARSE, STOP, TEST_SENS
 * ================================================================ */
void loop() {
  String cmd = getIncomingRaw();
  if (cmd.length() == 0) {
    vTaskDelay(pdMS_TO_TICKS(50));
    return;
  }

  cmd.trim();
  cmd.toUpperCase();

  /* ---- Common Commands (any state) ---- */

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

  if (cmd == "PARSE") {
    bool wasLogging = logger.isEnabled();
    if (wasLogging) {
      logger.setEnabled(false);
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    flightActive = false;
    navTestActive = false;
    desk_test_mode = false;
    padAlignActive = false;
    alignReady = false;
    launched = false;
    digitalWrite(PYRO_1_PIN, LOW);
    digitalWrite(PYRO_2_PIN, LOW);
    servo_center_all();
    digitalWrite(LED_PIN, LOW);
    sendResponse("DUMP START\n");
    logger.forceFlushBuffer();
    logger.dumpRawBinary(Serial);
    sendResponse("DUMP DONE\n");
    return;
  }

  if (cmd == "DESK_TEST") {
    Serial.println(">>> DESK TEST <<<");
    zupt_in_progress = true;
    vTaskDelay(pdMS_TO_TICKS(50));
    Serial.println("Calibrating Baro...");
    bmp.calibrate(50);
    desk_test_align_requested = true;
    zupt_in_progress = false;
    while (desk_test_align_requested) {
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    Serial.println("Phase forced to ACTIVE for desk test");
    return;
  }

  /* ---- Pre-Flight Commands ---- */

  else if (!flightActive) {

    /* CALIBRATE_STATIC: IMU + Baro + GPS 통합 정적 보정 */
    if (cmd == "CALIBRATE_STATIC") {
      zupt_in_progress = true;
      vTaskDelay(pdMS_TO_TICKS(50));
      alignReady = false;
      padAlignActive = false;
      digitalWrite(LED_PIN, HIGH);

      /* IMU + Baro (required) */
      sendResponse("CALIBRATING IMU+BARO...\n");
      bool imuOk = imu.calibrate(CALIB_SAMPLES);
      bool bmpOk = bmp.calibrate(CALIB_SAMPLES);

      /* GPS origin (optional — proceed without if unavailable) */
      sendResponse("CALIBRATING GPS...\n");
      uint32_t gpsWaitStart = millis();
      while ((uint32_t)(millis() - gpsWaitStart) < 5000) {
        if (gps.update())
          break;
        vTaskDelay(pdMS_TO_TICKS(GPS_POLL_MS));
      }
      bool gpsOk = gps.calibrate();
      if (gpsOk) {
        nav.setLaunchSite(gps.getOriginLatDeg(), gps.getOriginLonDeg());
      }

      zupt_in_progress = false;

      if (!imuOk)
        sendResponse("CALIB FAIL: IMU\n");
      if (!bmpOk)
        sendResponse("CALIB FAIL: BARO\n");
      if (!gpsOk)
        sendResponse("CALIB WARN: GPS (proceeding without GPS)\n");

      if (imuOk && bmpOk) {
        if (gpsOk)
          sendResponse("CALIBRATION DONE\n");
        else
          sendResponse("CALIBRATION DONE (NO GPS)\n");
        digitalWrite(LED_PIN, LOW);
        beep(200);
      } else {
        nav.ekfReset();
        sendResponse("CALIBRATION FAILED\n");
        digitalWrite(LED_PIN, LOW);
        beep(100, 3);
      }
    }

    /* CALIBRATE_MAG: 자력계 8자 회전 보정 */
    else if (cmd == "CALIBRATE_MAG") {
      zupt_in_progress = true;
      vTaskDelay(pdMS_TO_TICKS(50));
      alignReady = false;
      padAlignActive = false;
      digitalWrite(LED_PIN, HIGH);

      bool magOk = false;
      for (int attempt = 1; attempt <= CALIB_MAX_ATTEMPTS && !magOk;
           attempt++) {
        sendResponse("MAG CALIB (30s)...\n");
        magOk = mag.calibrate(MAG_CALIB_MS);
        mag.clearInterruptFlag();
        if (magOk) {
          sendResponse("MAG CALIB DONE\n");
          digitalWrite(LED_PIN, LOW);
          beep(200);
        } else {
          sendResponse("INSUFFICIENT ROTATION — RETRYING...\n");
          beep(100, 3);
          vTaskDelay(pdMS_TO_TICKS(2000));
        }
      }
      if (!magOk) {
        sendResponse("MAG CALIB FAIL\n");
        digitalWrite(LED_PIN, LOW);
      }

      zupt_in_progress = false;
    }

    /* READY: TRIAD 평균 정렬 + ZUPT 수렴 */
    else if (cmd == "READY") {
      sendResponse("READY START\n");
      zupt_in_progress = true;
      vTaskDelay(pdMS_TO_TICKS(50));
      alignReady = false;
      padAlignActive = false;
      digitalWrite(LED_PIN, HIGH);

      /* 1. Reset EKF + init average */
      nav.ekfReset();
      nav.resetInitAverage();

      /* 2. Collect TRIAD average data (let flight_task feed sensors) */
      sendResponse("TRIAD AVG...\n");
      zupt_in_progress = false;
      vTaskDelay(pdMS_TO_TICKS(START_TRIAD_AVG_MS));
      zupt_in_progress = true;
      vTaskDelay(pdMS_TO_TICKS(50));

      /* 3. EKF init from averaged TRIAD */
      if (!nav.ekfBeginAveraged(START_TRIAD_MIN_IMU, START_TRIAD_MIN_MAG)) {
        sendResponse("EKF INIT FAIL\n");
        nav.ekfReset();
        zupt_in_progress = false;
        digitalWrite(LED_PIN, LOW);
        beep(100, 3);
        vTaskDelay(pdMS_TO_TICKS(50));
        return;
      }

      /* 4. ZUPT convergence loop */
      sendResponse("ALIGNING EKF...\n");
      float P_prev = 0.0f;
      int stable_count = 0;
      bool converged = false;
      int iter = 0;

      for (iter = 0; iter < ZUPT_MAX_ITER; iter++) {
        int16_t gx, gy, gz, ax, ay, az;
        imu.readCalibratedIMU(gx, gy, gz, ax, ay, az);
        Raw_imu r = {};
        r.timestamp = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
        r.gx = gx;
        r.gy = gy;
        r.gz = gz;
        r.ax = ax;
        r.ay = ay;
        r.az = az;
        nav.updateIMU(r);

        if (mag.isDataReady()) {
          float mx, my, mz;
          if (mag.readCalibratedMag(mx, my, mz)) {
            Raw_mag m = {};
            m.timestamp = r.timestamp;
            m.mx = mx;
            m.my = my;
            m.mz = mz;
            nav.updateMag(m);
            mag.clearInterruptFlag();
          }
        }

        nav.ekfUpdateStaticAlignment();
        float P_now = nav.ekf().attBiasCovTrace();

        if (iter >= ZUPT_MIN_ITER && P_prev > 0.0f) {
          float rel = fabsf(P_prev - P_now) / P_prev;
          if (rel < ZUPT_REL_THRESH) {
            if (++stable_count >= ZUPT_STABLE_REQ) {
              converged = true;
              break;
            }
          } else {
            stable_count = 0;
          }
        }
        P_prev = P_now;
        vTaskDelay(pdMS_TO_TICKS(ZUPT_PERIOD_MS));
      }

      char buf[80];
      snprintf(buf, sizeof(buf), "ZUPT %s in %.1fs (P=%.3e)\n",
               converged ? "CONVERGED" : "TIMEOUT",
               (iter + 1) * (float)ZUPT_PERIOD_MS * 0.001f, P_prev);
      sendResponse(buf);

      if (converged) {
        alignReady = true;
        padAlignActive = true;
        zupt_in_progress = false;
        digitalWrite(LED_PIN, LOW);
        beep(200);
        sendResponse("EKF READY\n");
      } else {
        nav.ekfReset();
        zupt_in_progress = false;
        digitalWrite(LED_PIN, LOW);
        beep(100, 3);
      }
    }

    /* START: 비행 시작 (alignReady 필수) */
    else if (cmd == "START") {
      sendResponse("STARTING...\n");
      navTestActive = false;

      if (!alignReady) {
        sendResponse("READY REQUIRED\n");
        beep(100, 3);
        vTaskDelay(pdMS_TO_TICKS(50));
        return;
      }

      padAlignActive = false;
      alignReady = false;
      logger.setEnabled(true);
      flightActive = true;
      gnc_init(&gnc_state, &phase_state, &rcv_state);
      pyro1_start_ms = 0;
      pyro2_start_ms = 0;
      pyro1_active = false;
      pyro2_active = false;
      digitalWrite(LED_PIN, HIGH);
      beep(300);
      sendResponse("FLIGHT ACTIVE\n");
    }

    /* NAV_TEST: 항법 only 테스트 (서보/파이로 비활성) */
    else if (cmd == "NAV_TEST") {
      sendResponse("NAV_TEST STARTING...\n");

      if (!alignReady) {
        sendResponse("READY REQUIRED\n");
        beep(100, 3);
        vTaskDelay(pdMS_TO_TICKS(50));
        return;
      }

      padAlignActive = false;
      alignReady = false;
      navTestActive = true;
      logger.setEnabled(true);
      flightActive = true;
      gnc_init(&gnc_state, &phase_state, &rcv_state);
      digitalWrite(PYRO_1_PIN, LOW);
      digitalWrite(PYRO_2_PIN, LOW);
      digitalWrite(LED_PIN, HIGH);
      beep(300);
      sendResponse("NAV TEST ACTIVE\n");
    }

    else if (cmd == "ERASE") {
      alignReady = false;
      padAlignActive = false;
      sendResponse("ERASING FLASH...\n");
      logger.eraseAll();
      sendResponse("ERASE DONE\n");
      beep(100);
    }

    else if (cmd == "TEST_SENS") {
      zupt_in_progress = true;
      vTaskDelay(pdMS_TO_TICKS(10));
      char buf[256];

      /* IMU */
      int16_t gx, gy, gz, ax, ay, az;
      imu.readCalibratedIMU(gx, gy, gz, ax, ay, az);
      const float A_SCALE = 0.976e-3f * 9.80665f;  /* LSM6DSO32 FS=32g */
      const float G_SCALE = 70.0e-3f * (3.14159265f / 180.0f); /* LSM6DSO32 FS=2000dps */
      float axf = ax * A_SCALE;
      float ayf = ay * A_SCALE;
      float azf = az * A_SCALE;
      float gxf = gx * G_SCALE;
      float gyf = gy * G_SCALE;
      float gzf = gz * G_SCALE;
      snprintf(buf, sizeof(buf),
               "[IMU] Gyro: %6.3f %6.3f %6.3f rad/s | Accel: %6.2f %6.2f %6.2f m/s2\n",
               gxf, gyf, gzf, axf, ayf, azf);
      sendResponse(buf);

      /* Baro */
      float alt;
      if (bmp.readAltitude(alt)) {
        snprintf(buf, sizeof(buf), "[BARO] Alt: %.2f m\n", alt);
      } else {
        snprintf(buf, sizeof(buf), "[BARO] READ FAIL\n");
      }
      sendResponse(buf);

      /* Mag */
      float mx, my, mz;
      if (mag.readCalibratedMag(mx, my, mz)) {
        snprintf(buf, sizeof(buf),
                 "[MAG] Mx: %6.3f My: %6.3f Mz: %6.3f Gauss\n", mx, my, mz);
      } else {
        snprintf(buf, sizeof(buf), "[MAG] READ FAIL\n");
      }
      sendResponse(buf);

      /* GPS */
      gps.update();
      if (gps.hasFix()) {
        snprintf(buf, sizeof(buf),
                 "[GPS] Fix: 3D | Lat: %.6f | Lon: %.6f | hAcc: %.1fm | Sats: %d\n",
                 gps.getLatDeg(), gps.getLonDeg(),
                 gps.getHorizontalAcc(),
                 gps.getRaw().numSV);
      } else {
        snprintf(buf, sizeof(buf),
                 "[GPS] No Fix | Sats: %d\n",
                 gps.getRaw().numSV);
      }
      sendResponse(buf);

      sendResponse("TEST_SENS OK\n");
      zupt_in_progress = false;
    }

    /* Legacy commands → redirect */
    else if (cmd == "CALIBRATE") {
      sendResponse("USE CALIBRATE_STATIC\n");
    } else if (cmd == "ZUPT") {
      sendResponse("USE READY\n");
    }

    else {
      sendResponse("UNKNOWN CMD\n");
    }
  }

  /* ---- Flight-active Commands ---- */
  else {
    if (cmd == "STOP") {
      digitalWrite(PYRO_1_PIN, LOW);
      digitalWrite(PYRO_2_PIN, LOW);
      flightActive = false;
      navTestActive = false;
      desk_test_mode = false;
      padAlignActive = false;
      alignReady = false;
      launched = false;
      if (logger.isEnabled()) {
        logger.setEnabled(false);
        vTaskDelay(pdMS_TO_TICKS(100));
        logger.forceFlushBuffer();
      }
      servo_center_all();
      digitalWrite(LED_PIN, LOW);
      beep(200);
      vTaskDelay(pdMS_TO_TICKS(50));
      beep(200);
      sendResponse("STOPPED.\n");
    }
  }

  vTaskDelay(pdMS_TO_TICKS(50));
}