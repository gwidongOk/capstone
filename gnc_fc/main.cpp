/**
 * @file main.cpp
 * Unified Flight Software — NAV + GNC Integration.
 *
 * Core 1: flight_task (400Hz) — sensor -> EKF -> GNC -> servo -> log
 * Core 0: gps_task, flush_task, loop() (BLE commands)
 * ESP32-S3 dual-core, PlatformIO Arduino + FreeRTOS.
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <esp_timer.h>

#include "board_config.h"
#include "LSM6DSO32.h"
#include "BMP388.h"
#include "MMC5983MA.h"
#include "NEOM9N.h"
#include "NAV.h"
#include "MX25Logger.h"
#include "sensor_data.h"
#include "BLE.h"
#include "servo_hal.h"

extern "C" {
#include "gnc_types.h"
#include "gnc_config.h"
#include "flight_phase.h"
}
#include "gnc_main.h"

// Hardware objects
static SPIClass sensorSPI(HSPI);
static SPIClass flashSPI(FSPI);
static SemaphoreHandle_t flashMutex = NULL;

static LSM6DSO32  imu(IMU_CS_PIN, &sensorSPI);
static BMP388     bmp(BMP_CS_PIN, &sensorSPI);
static MMC5983MA  mag(&Wire);
static NEOM9N     gps(Serial1, GPS_RX_PIN, GPS_TX_PIN);
static NAV        nav;
static MX25Logger logger;

// GNC state
static gnc_config_t  gnc_cfg;
static gnc_state_t   gnc_state;
static phase_state_t phase_state;

// GPS double buffer (Core 0 -> Core 1, lock-free)
static Raw_gps gps_buf[2] = {};
static volatile int gps_write_idx = 0;

// Flight control flags
static volatile bool flightActive = false;
static volatile bool chute_fired = false;
static volatile bool zupt_in_progress = false;
static volatile bool launched = false;
static volatile bool desk_test_mode = false;
static volatile bool desk_test_start_pending = false;
static volatile bool desk_test_align_requested = false;

// Task handles
static TaskHandle_t flightTaskHandle = NULL;
static TaskHandle_t gpsTaskHandle    = NULL;
static TaskHandle_t flushTaskHandle  = NULL;

static void beep(uint32_t duration_ms, uint8_t count = 1)
{
    for (uint8_t i = 0; i < count; i++) {
        if (i > 0) delay(100);
        digitalWrite(BUZZER_PIN, HIGH);
        delay(duration_ms);
        digitalWrite(BUZZER_PIN, LOW);
    }
}

// PIL (Processor-In-The-Loop) protocol structs
#pragma pack(push, 1)
typedef struct {
    float pn, pe, pd;
    float vn, ve, vd;
    float qw, qx, qy, qz;
    float p, q, r;
    float ax, ay, az;
    float wx, wy, wz;
    float airspeed;
    float mission_time;
    uint32_t t_ms;
} pil_packet_t;  // 88 bytes

typedef struct {
    float fin_cmd[4];
    float nz_cmd;
    float ny_cmd;
    uint8_t phase;
} pil_response_t;  // 25 bytes
#pragma pack(pop)

// NAV -> GNC state conversion
static nav_state_t build_nav_state(const State_nominal &nom, const State_imu &si,
                                    uint32_t t_ms, bool ekf_ready)
{
    nav_state_t ns = {};
    ns.pos_ned  = {nom.p[0], nom.p[1], nom.p[2]};
    ns.vel_ned  = {nom.v[0], nom.v[1], nom.v[2]};
    ns.quat     = {nom.q[0], nom.q[1], nom.q[2], nom.q[3]};
    ns.omega_b  = {si.gx - nom.bg[0], si.gy - nom.bg[1], si.gz - nom.bg[2]};
    ns.accel_b  = {si.ax - nom.ba[0], si.ay - nom.ba[1], si.az - nom.ba[2]};
    ns.airspeed = sqrtf(nom.v[0]*nom.v[0] + nom.v[1]*nom.v[1] + nom.v[2]*nom.v[2]);
    ns.timestamp_ms = t_ms;
    ns.valid = ekf_ready;
    return ns;
}

// GPS Task — Core 0
static void gps_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (gps.update()) {
            Raw_gps g = {};
            g.timestamp = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
            g.hasPos = gps.getNED(g.pn, g.pe, g.pd, g.vn, g.ve, g.vd,
                                  g.hAcc, g.vAcc, g.fixType, g.numSV);
            int idx = 1 - gps_write_idx;
            gps_buf[idx] = g;
            __atomic_store_n(&gps_write_idx, idx, __ATOMIC_RELEASE);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// Flush Task — Core 0
static void flush_task(void *arg)
{
    (void)arg;
    for (;;) {
        logger.serviceFlush();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// Flight Task — Core 1 (400Hz)
static void flight_task(void *arg)
{
    (void)arg;

    uint32_t cycle = 0;
    float mission_time = 0.0f;
    uint32_t launch_time_us = 0;
    uint32_t last_gps_ts = 0;
    uint32_t pyro1_start_ms = 0;
    bool pyro1_active = false;
    static float last_baro_alt = 0.0f;

    // Seed NAV timestamp with initial IMU read
    {
        int16_t gx, gy, gz, ax, ay, az;
        imu.readCalibratedIMU(gx, gy, gz, ax, ay, az);
        Raw_imu r = {};
        r.timestamp = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
        r.gx = gx; r.gy = gy; r.gz = gz;
        r.ax = ax; r.ay = ay; r.az = az;
        nav.updateIMU(r);
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2));
        uint32_t t_us = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
        uint32_t t_ms = t_us / 1000;

        // Pyro timer (non-blocking)
        if (pyro1_active && (millis() - pyro1_start_ms > 1000)) {
            digitalWrite(PYRO_1_PIN, LOW);
            pyro1_active = false;
        }

        // Skip if ZUPT/calibration in progress
        if (zupt_in_progress) {
            cycle++;
            continue;
        }

        if (desk_test_align_requested) {
            // EKF init
            if (!nav.isEkfReady()) {
                if (!nav.ekfBegin()) {
                    Serial.println("TRIAD failed (no MAG), using accel-only init");

                    State_imu si = nav.getStateImu();
                    float ax = si.ax, ay = si.ay, az = si.az;

                    float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
                    float roll  = atan2f(ay, -az);
                    float yaw   = 0.0f;

                    float cr = cosf(roll*0.5f),  sr = sinf(roll*0.5f);
                    float cp = cosf(pitch*0.5f), sp = sinf(pitch*0.5f);
                    float cy = cosf(yaw*0.5f),   sy = sinf(yaw*0.5f);

                    float q0[4] = {
                        cr*cp*cy + sr*sp*sy,
                        sr*cp*cy - cr*sp*sy,
                        cr*sp*cy + sr*cp*sy,
                        cr*cp*sy - sr*sp*cy
                    };

                    float p0[3] = {0, 0, 0};
                    float v0[3] = {0, 0, 0};
                    nav.ekfBegin(p0, v0, q0);

                    char buf[128];
                    snprintf(buf, sizeof(buf), "Init RPY: R=%.1f P=%.1f Y=%.1f deg\n",
                            roll * 57.2958f, pitch * 57.2958f, yaw * 57.2958f);
                    Serial.print(buf);
                }
            }

            // ZUPT convergence loop
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
                r.gx = gx; r.gy = gy; r.gz = gz;
                r.ax = ax; r.ay = ay; r.az = az;
                nav.updateIMU(r);

                nav.ekfUpdateStaticAlignment();
                float P_now = nav.ekf().attBiasCovTrace();

                if (iter >= 10 && P_prev > 0.0f) {
                    float rel = fabsf(P_prev - P_now) / P_prev;
                    if (rel < REL_THRESH) {
                        if (++stable_count >= STABLE_REQ) break;
                    } else {
                        stable_count = 0;
                    }
                }
                P_prev = P_now;
                vTaskDelay(pdMS_TO_TICKS(50));
            }

            Serial.println("EKF ALIGNED!");

            gnc_init(&gnc_state, &phase_state);
            phase_state.phase = PHASE_GUIDE;
            phase_state.t_phase_enter = 0.0f;
            desk_test_mode = true;
            flightActive = true;
            launched = true;
            desk_test_start_pending = true;

            desk_test_align_requested = false;
            cycle++;
            continue;
        }

        // 1. SENSE

        // IMU: every cycle
        Raw_imu imu_raw = {};
        {
            int16_t gx, gy, gz, ax, ay, az;
            imu.readCalibratedIMU(gx, gy, gz, ax, ay, az);
            imu_raw.timestamp = t_us;
            imu_raw.gx = gx; imu_raw.gy = gy; imu_raw.gz = gz;
            imu_raw.ax = ax; imu_raw.ay = ay; imu_raw.az = az;
        }

        // Baro: every 16 cycles (~25Hz)
        bool baro_new = (cycle % 16 == 0);
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

        // Mag: every 8 cycles (~50Hz), offset from baro
        bool mag_new = (cycle % 8 == 4);
        Raw_mag mag_raw = {};
        if (mag_new && mag.isDataReady()) {
            float mx, my, mz;
            if (mag.readCalibratedMag(mx, my, mz)) {
                mag_raw.timestamp = t_us;
                mag_raw.mx = mx; mag_raw.my = my; mag_raw.mz = mz;
                mag.clearInterruptFlag();
            } else {
                mag_new = false;
            }
        } else {
            mag_new = false;
        }

        // GPS: atomic read from double buffer
        Raw_gps gps_data;
        {
            int idx = __atomic_load_n(&gps_write_idx, __ATOMIC_ACQUIRE);
            gps_data = (Raw_gps)gps_buf[idx];
        }
        bool gps_new = (gps_data.timestamp != last_gps_ts && gps_data.hasPos);
        if (gps_new) last_gps_ts = gps_data.timestamp;

        // 2. NAVIGATE — EKF updates then predict
        if (gps_new)  nav.updateGps(gps_data);
        if (baro_new) nav.updatePress(press);
        if (mag_new)  nav.updateMag(mag_raw);
        nav.updateIMU(imu_raw);

        // Pre-flight: idle until flightActive
        if (!flightActive) {
            if (cycle % 100 == 0) {
                logger.logImu(imu_raw);
                if (nav.isEkfReady()) logger.logState(nav.getNominal());
            }
            cycle++;
            continue;
        }

        // 3. LAUNCH DETECTION
        if (!launched) {
            State_imu si = nav.getStateImu();
            State_nominal nom = nav.getNominal();
            float acx = si.ax - nom.ba[0];
            float acy = si.ay - nom.ba[1];
            float acz = si.az - nom.ba[2];
            float amag = sqrtf(acx*acx + acy*acy + acz*acz);

            float alt = -nom.p[2];
            float vel_up = -nom.v[2];

            if (amag > LAUNCH_ACCEL_G * 9.81f || alt > LAUNCH_ALT_M || vel_up > LAUNCH_VEL_M_S) {
                launched = true;
                launch_time_us = t_us;
                mission_time = 0.0f;
                logger.logEvent(FlightPhase::POWERED_FLIGHT, 1);
                sendResponse("LAUNCH\n");
            }

            logger.logImu(imu_raw);
            if (baro_new) logger.logBaro(press);
            if (mag_new)  logger.logMag(mag_raw);
            if (gps_new)  logger.logGps(gps_data);
            cycle++;
            continue;
        }

        // Desk test sync
        if (desk_test_start_pending) {
            launch_time_us = t_us;
            mission_time = 0.0f;
            desk_test_start_pending = false;
        }

        // 4. NAV -> GNC
        State_nominal nom = nav.getNominal();
        State_imu     si  = nav.getStateImu();
        nav_state_t nav_gnc = build_nav_state(nom, si, t_ms, nav.isEkfReady());

        // 5. GNC
        gnc_output_t gnc_out = gnc_step(&nav_gnc, &gnc_cfg, &gnc_state,
                                         &phase_state, mission_time);

        if (desk_test_mode) {
            phase_state.phase = PHASE_GUIDE;
            gnc_out.phase = PHASE_GUIDE;
        }

        // 6. ACTUATE (disabled for desk test)
        // servo_write(&gnc_out.servo);

        // 7. RECOVERY — parachute deploy
        if (gnc_out.phase == PHASE_DESCENT && !chute_fired) {
            digitalWrite(PYRO_1_PIN, HIGH);
            pyro1_start_ms = millis();
            pyro1_active = true;
            chute_fired = true;
            logger.logEvent(FlightPhase::DESCENT, 2);
            sendResponse("APG\n");
        }

        // 8. LANDING DETECTION
        if (gnc_out.phase == PHASE_DESCENT && !desk_test_mode) {
            float alt = -nom.p[2];
            float vel_up = -nom.v[2];
            if (alt < LAND_ALT_M && fabsf(vel_up) < LAND_VEL_M_S) {
                flightActive = false;
                launched = false;
                // servo_center_all();
                logger.logEvent(FlightPhase::LANDED, 4);
                logger.setEnabled(false);
                vTaskDelay(pdMS_TO_TICKS(100));
                logger.forceFlushBuffer();
                digitalWrite(LED_PIN, LOW);
                sendResponse("LAND\n");
            }
        }

        // 10Hz serial output
        if (flightActive && (cycle % 40 == 0)) {
            Serial.printf("[SENS] Gyro: %5.2f %5.2f %5.2f | Accel: %6.2f %6.2f %6.2f | Baro: %.2f m\n",
                          si.gx, si.gy, si.gz, si.ax, si.ay, si.az, last_baro_alt);

            Serial.printf("[ NAV] Pos: [%6.1f, %6.1f, %6.1f] | Vel: [%5.1f, %5.1f, %5.1f] | Valid: %d | T: %.2f s\n",
              nom.p[0], nom.p[1], nom.p[2],
              nom.v[0], nom.v[1], nom.v[2],
              nav.isEkfReady(), mission_time);

            Serial.printf("[ GNC] Phase: %-7s | Cmd(Nz,Ny): %5.2f, %5.2f | Fins(deg): %6.1f %6.1f %6.1f %6.1f\n\n",
                          phase_name(gnc_out.phase),
                          gnc_out.guid.nz_cmd, gnc_out.guid.ny_cmd,
                          gnc_out.ap.fin_cmd[0] * RAD2DEG_F,
                          gnc_out.ap.fin_cmd[1] * RAD2DEG_F,
                          gnc_out.ap.fin_cmd[2] * RAD2DEG_F,
                          gnc_out.ap.fin_cmd[3] * RAD2DEG_F);
        }

        // 9. LOGGING
        logger.logImu(imu_raw);
        if (baro_new) logger.logBaro(press);
        if (mag_new)  logger.logMag(mag_raw);
        if (gps_new)  logger.logGps(gps_data);
        if (cycle % 8 == 0) {
            logger.logState(nom);
            logger.logGnc(gnc_out.guid.nz_cmd, gnc_out.guid.ny_cmd,
                          gnc_out.ap.fin_cmd, (uint8_t)gnc_out.phase);
        }

        // 10. TIME
        mission_time = (float)(t_us - launch_time_us) * 1e-6f;
        cycle++;
    }
}

// setup()
void setup()
{
    Serial.begin(SERIAL_BAUD);

    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(PYRO_1_PIN, OUTPUT);
    pinMode(PYRO_2_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    digitalWrite(PYRO_1_PIN, LOW);
    digitalWrite(PYRO_2_PIN, LOW);

    flashMutex = xSemaphoreCreateMutex();

    initBLE(BLE_DEVICE_NAME);
    sensorSPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
    Wire.begin(MAG_SDA_PIN, MAG_SCL_PIN, 400000);

    bool ok = true;
    if (!imu.begin()) { Serial.println("IMU FAIL"); ok = false; }
    if (!bmp.begin()) { Serial.println("BMP FAIL"); ok = false; }
    bool gps_ok = gps.begin(20);
    if (!gps_ok) Serial.println("GPS WARN (task disabled)");
    if (!ok) { beep(100, 3); while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); } }
    Serial.println("SENSORS READY (IMU/BMP)");

    logger.begin(&flashSPI, FLASH_SCK_PIN, FLASH_MISO_PIN, FLASH_MOSI_PIN,
                 FLASH_CS_PIN, flashMutex);

    // Servo disabled for desk test
    // servo_init();
    // servo_center_all();

    gnc_cfg = gnc_config_default();
    gnc_init(&gnc_state, &phase_state);

    xTaskCreatePinnedToCore(flight_task, "flight", STACK_SIZE_FLIGHT, NULL,
                            TASK_C1_PRIO_FLIGHT, &flightTaskHandle, 1);
    xTaskCreatePinnedToCore(gps_task, "gps", STACK_SIZE_GPS, NULL,
                            TASK_C0_PRIO_GPS, &gpsTaskHandle, 0);
    xTaskCreatePinnedToCore(flush_task, "flush", STACK_SIZE_FLUSH, NULL,
                            TASK_C0_PRIO_FLUSH, &flushTaskHandle, 0);

    beep(200);
    sendResponse(">>> READY\n");
}

// loop() — BLE Command Dispatcher (Core 0)
void loop()
{
    String cmd = getIncomingRaw();
    if (cmd.length() == 0) {
        vTaskDelay(pdMS_TO_TICKS(50));
        return;
    }

    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "REBOOT") {
        sendResponse("REBOOTING...\n");
        delay(100);
        ESP.restart();
    }

    if (cmd == "DESK_TEST") {
        Serial.println(">>> DESK TEST FORCE STARTED <<<");

        zupt_in_progress = true;
        vTaskDelay(pdMS_TO_TICKS(50));

        Serial.println("Calibrating Baro to 0m...");
        bmp.calibrate(50);

        // Delegate EKF alignment to flight_task (needs its stack for Eigen)
        desk_test_align_requested = true;
        zupt_in_progress = false;

        while (desk_test_align_requested) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        Serial.println("Phase forced to GUIDE for desk test");
        return;
    }

    else if (!flightActive) {

        if (cmd == "CALIBRATE") {
            zupt_in_progress = true;
            vTaskDelay(pdMS_TO_TICKS(10));
            digitalWrite(LED_PIN, HIGH);

            bool imuOk = false, bmpOk = false;
            while (!(imuOk && bmpOk)) {
                sendResponse("CALIBRATING IMU+BMP...\n");
                imuOk = imu.calibrate(CALIB_SAMPLES);
                bmpOk = bmp.calibrate(CALIB_SAMPLES);

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
            zupt_in_progress = false;
        }

        else if (cmd == "ZUPT") {
            zupt_in_progress = true;
            vTaskDelay(pdMS_TO_TICKS(10));
            sendResponse("ALIGNING EKF...\n");
            digitalWrite(LED_PIN, HIGH);

            if (!nav.isEkfReady()) nav.ekfBegin();

            const int   MIN_ITER     = 10;
            const int   MAX_ITER     = 200;
            const float REL_THRESH   = 1e-3f;
            const int   STABLE_REQ   = 3;

            float P_prev = 0.0f;
            int   stable_count = 0;
            int   iter = 0;
            bool  converged = false;

            for (iter = 0; iter < MAX_ITER; iter++) {
                int16_t gx, gy, gz, ax, ay, az;
                imu.readCalibratedIMU(gx, gy, gz, ax, ay, az);
                Raw_imu r = {};
                r.timestamp = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
                r.gx = gx; r.gy = gy; r.gz = gz;
                r.ax = ax; r.ay = ay; r.az = az;
                nav.updateIMU(r);

                if (mag.isDataReady()) {
                    float mx, my, mz;
                    if (mag.readCalibratedMag(mx, my, mz)) {
                        Raw_mag m = {};
                        m.timestamp = r.timestamp;
                        m.mx = mx; m.my = my; m.mz = mz;
                        nav.updateMag(m);
                        mag.clearInterruptFlag();
                    }
                }

                nav.ekfUpdateStaticAlignment();
                float P_now = nav.ekf().attBiasCovTrace();

                if (iter >= MIN_ITER && P_prev > 0.0f) {
                    float rel = fabsf(P_prev - P_now) / P_prev;
                    if (rel < REL_THRESH) {
                        if (++stable_count >= STABLE_REQ) { converged = true; break; }
                    } else {
                        stable_count = 0;
                    }
                }
                P_prev = P_now;
                vTaskDelay(pdMS_TO_TICKS(100));
            }

            char buf[80];
            snprintf(buf, sizeof(buf), "ZUPT %s in %.1fs (P_trace=%.3e)\n",
                     converged ? "CONVERGED" : "TIMEOUT",
                     (iter + 1) * 0.1f, P_prev);
            sendResponse(buf);
            digitalWrite(LED_PIN, LOW); beep(100);
            zupt_in_progress = false;
        }

        else if (cmd == "START") {
            sendResponse("STARTING...\n");
            if (!nav.isEkfReady()) {
                if (!nav.ekfBegin()) {
                    sendResponse("EKF INIT FAIL\n");
                    beep(100, 3); return;
                }
            }
            sendResponse("EKF READY\n");

            logger.setEnabled(true);
            flightActive = true;
            chute_fired = false;
            gnc_init(&gnc_state, &phase_state);
            digitalWrite(LED_PIN, HIGH);
            beep(300);
            sendResponse("FLIGHT ACTIVE\n");
        }

        else if (cmd == "ERASE") {
            sendResponse("ERASING FLASH...\n");
            logger.eraseAll();
            sendResponse("ERASE DONE\n");
            beep(100);
        }

        else if (cmd == "PARSE") {
            sendResponse("DUMPING...\n");
            logger.dumpRawBinary(Serial);
            sendResponse("DUMP DONE\n");
        }

        else if (cmd.startsWith("TARGET ")) {
            float n, e, d;
            if (sscanf(cmd.c_str() + 7, "%f %f %f", &n, &e, &d) == 3) {
                gnc_cfg.target_ned = (vec3_t){n, e, d};
                char buf[64];
                snprintf(buf, sizeof(buf), "TGT [%.0f,%.0f,%.0f]\n", n, e, d);
                sendResponse(buf);
            } else {
                sendResponse("TARGET ERR\n");
            }
        }

        else if (cmd == "TEST_SENS") {
            zupt_in_progress = true;
            vTaskDelay(pdMS_TO_TICKS(10));

            char buf[256];

            int16_t gx, gy, gz, ax, ay, az;
            imu.readCalibratedIMU(gx, gy, gz, ax, ay, az);
            snprintf(buf, sizeof(buf),
                     "IMU: gx=%d gy=%d gz=%d ax=%d ay=%d az=%d\n",
                     gx, gy, gz, ax, ay, az);
            sendResponse(buf);

            float alt;
            if (bmp.readAltitude(alt)) {
                snprintf(buf, sizeof(buf), "BMP: alt=%.2f m\n", alt);
            } else {
                snprintf(buf, sizeof(buf), "BMP: READ FAIL\n");
            }
            sendResponse(buf);

            if (mag.isDataReady()) {
                float mx, my, mz;
                if (mag.readCalibratedMag(mx, my, mz)) {
                    snprintf(buf, sizeof(buf), "MAG: mx=%.4f my=%.4f mz=%.4f G\n", mx, my, mz);
                    mag.clearInterruptFlag();
                } else {
                    snprintf(buf, sizeof(buf), "MAG: READ FAIL\n");
                }
            } else {
                snprintf(buf, sizeof(buf), "MAG: NOT READY\n");
            }
            sendResponse(buf);

            sendResponse("TEST_SENS OK\n");
            zupt_in_progress = false;
        }

        else if (cmd == "TEST_LOG") {
            zupt_in_progress = true;
            vTaskDelay(pdMS_TO_TICKS(10));

            logger.setEnabled(true);
            sendResponse("LOGGING 50 SAMPLES...\n");

            for (int i = 0; i < 50; i++) {
                int16_t gx, gy, gz, ax, ay, az;
                imu.readCalibratedIMU(gx, gy, gz, ax, ay, az);
                Raw_imu r = {};
                r.timestamp = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
                r.gx = gx; r.gy = gy; r.gz = gz;
                r.ax = ax; r.ay = ay; r.az = az;
                logger.logImu(r);

                float alt;
                if (bmp.readAltitude(alt)) {
                    Raw_press p = {};
                    p.timestamp = r.timestamp;
                    p.alt = alt;
                    logger.logBaro(p);
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            logger.setEnabled(false);
            vTaskDelay(pdMS_TO_TICKS(50));
            logger.forceFlushBuffer();

            char buf[64];
            snprintf(buf, sizeof(buf), "LOGGED. addr=0x%08lX\n",
                     (unsigned long)logger.getCurrentAddress());
            sendResponse(buf);
            sendResponse("TEST_LOG OK\n");
            zupt_in_progress = false;
        }

        else if (cmd == "TEST_GNC") {
            zupt_in_progress = true;
            vTaskDelay(pdMS_TO_TICKS(10));

            nav_state_t fake = {};
            fake.pos_ned  = {50.0f, 50.0f, -100.0f};
            fake.vel_ned  = {30.0f, 30.0f, -80.0f};
            fake.quat     = {1.0f, 0.0f, 0.0f, 0.0f};
            fake.omega_b  = {0.0f, 0.01f, 0.0f};
            fake.accel_b  = {0.0f, 0.0f, -40.0f};
            fake.airspeed = 92.0f;
            fake.valid    = true;

            gnc_init(&gnc_state, &phase_state);

            char buf[256];
            sendResponse("GNC TEST: 5 steps\n");

            float t_test[] = {0.5f, 1.0f, 1.5f, 2.0f, 2.5f};
            for (int i = 0; i < 5; i++) {
                gnc_output_t out = gnc_step(&fake, &gnc_cfg, &gnc_state,
                                             &phase_state, t_test[i]);
                snprintf(buf, sizeof(buf),
                         "t=%.1f phase=%d nz=%.3f ny=%.3f fin=[%.4f,%.4f,%.4f,%.4f] servo=[%u,%u,%u,%u]\n",
                         t_test[i], (int)out.phase,
                         out.guid.nz_cmd, out.guid.ny_cmd,
                         out.ap.fin_cmd[0], out.ap.fin_cmd[1],
                         out.ap.fin_cmd[2], out.ap.fin_cmd[3],
                         out.servo.pulse_us[0], out.servo.pulse_us[1],
                         out.servo.pulse_us[2], out.servo.pulse_us[3]);
                sendResponse(buf);
            }

            logger.setEnabled(true);
            gnc_output_t out = gnc_step(&fake, &gnc_cfg, &gnc_state,
                                         &phase_state, 3.0f);
            logger.logGnc(out.guid.nz_cmd, out.guid.ny_cmd,
                          out.ap.fin_cmd, (uint8_t)out.phase);
            logger.setEnabled(false);
            vTaskDelay(pdMS_TO_TICKS(50));
            logger.forceFlushBuffer();

            sendResponse("TEST_GNC OK\n");

            gnc_init(&gnc_state, &phase_state);
            zupt_in_progress = false;
        }

        else if (cmd == "PIL_START") {
            Serial.println("PIL READY");

            zupt_in_progress = true;
            vTaskDelay(pdMS_TO_TICKS(50));

            gnc_init(&gnc_state, &phase_state);

            Serial.setTimeout(5000);

            uint8_t rx_buf[128];
            bool running = true;
            uint32_t step_count = 0;

            while (running) {
                size_t n = Serial.readBytes(rx_buf, sizeof(pil_packet_t));

                if (n == 0) {
                    continue;
                }

                if (n >= 8 && rx_buf[0] == 'P' && rx_buf[1] == 'I' && rx_buf[2] == 'L') {
                    break;
                }

                if (n != sizeof(pil_packet_t)) {
                    continue;
                }

                pil_packet_t *pkt = (pil_packet_t*)rx_buf;

                nav_state_t nav_gnc = {};
                nav_gnc.pos_ned  = {pkt->pn, pkt->pe, pkt->pd};
                nav_gnc.vel_ned  = {pkt->vn, pkt->ve, pkt->vd};
                nav_gnc.quat     = {pkt->qw, pkt->qx, pkt->qy, pkt->qz};
                nav_gnc.omega_b  = {pkt->p,  pkt->q,  pkt->r};
                nav_gnc.accel_b  = {pkt->ax, pkt->ay, pkt->az};
                nav_gnc.wind_est = {pkt->wx, pkt->wy, pkt->wz};
                nav_gnc.airspeed = pkt->airspeed;
                nav_gnc.timestamp_ms = pkt->t_ms;
                nav_gnc.valid = true;

                gnc_output_t out = gnc_step(&nav_gnc, &gnc_cfg, &gnc_state,
                                            &phase_state, pkt->mission_time);

                pil_response_t resp;
                resp.fin_cmd[0] = out.ap.fin_cmd[0];
                resp.fin_cmd[1] = out.ap.fin_cmd[1];
                resp.fin_cmd[2] = out.ap.fin_cmd[2];
                resp.fin_cmd[3] = out.ap.fin_cmd[3];
                resp.nz_cmd = out.guid.nz_cmd;
                resp.ny_cmd = out.guid.ny_cmd;
                resp.phase  = (uint8_t)out.phase;

                Serial.write((uint8_t*)&resp, sizeof(pil_response_t));

                step_count++;
            }

            Serial.setTimeout(1000);
            gnc_init(&gnc_state, &phase_state);
            zupt_in_progress = false;

            char buf[64];
            snprintf(buf, sizeof(buf), "PIL DONE (%lu steps)\n", (unsigned long)step_count);
            Serial.println(buf);
        }

        else {
            sendResponse("UNKNOWN CMD\n");
        }
    }

    // Flight-active commands
    else {
        if (cmd == "STOP") {
            flightActive = false;
            desk_test_mode = false;
            vTaskDelay(pdMS_TO_TICKS(100));
            logger.setEnabled(false);
            vTaskDelay(pdMS_TO_TICKS(100));
            logger.forceFlushBuffer();
            // servo_center_all();
            digitalWrite(LED_PIN, LOW);
            beep(200);
            vTaskDelay(pdMS_TO_TICKS(50));
            beep(200);
            sendResponse("STOPPED.\n");
        }
        else if (cmd == "PARSE") {
            sendResponse("DUMPING...\n");
            logger.dumpRawBinary(Serial);
            sendResponse("DUMP DONE\n");
        }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
}
