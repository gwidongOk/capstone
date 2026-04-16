#include <Arduino.h>
#include <math.h>
#include "NEOM9N.h"

// ── Pin Configuration ───────────────────────────────────
#define GPS_RX_PIN 2   // ESP32 RX ← GPS TX
#define GPS_TX_PIN 1   // ESP32 TX → GPS RX

// ── GPS Instance ────────────────────────────────────────
NEOM9N gps(Serial1, GPS_RX_PIN, GPS_TX_PIN);

// ── NED Origin & Output (추후 NAV 라이브러리로 이동) ────

static constexpr double R_EARTH = 6378137.0;
static constexpr double DEG2RAD = M_PI / 180.0;

struct GPS_NED {
    float n, e, d;       // position [m]
    float vn, ve, vd;    // velocity [m/s]
    float hAcc;          // horizontal accuracy [m]
    float vAcc;          // vertical accuracy [m]
    float sAcc;          // speed accuracy [m/s]
    uint8_t fixType;
    uint8_t numSV;
    uint32_t tow;
    bool valid;
};

static double origin_lat = 0;  // [rad]
static double origin_lon = 0;  // [rad]
static double origin_alt = 0;  // [m]
static bool   origin_set = false;

// 현재 GPS raw 데이터로 원점 설정
bool gps_calibrate(const GPS_Raw &raw) {
    if (raw.fixType < 3) {
        Serial.println("[NAV] Cannot calibrate: no 3D fix");
        return false;
    }
    double lat = raw.lat * 1e-7;
    double lon = raw.lon * 1e-7;
    double alt = raw.alt * 1e-3;

    origin_lat = lat * DEG2RAD;
    origin_lon = lon * DEG2RAD;
    origin_alt = alt;
    origin_set = true;

    Serial.printf("[NAV] Origin: lat=%.7f lon=%.7f alt=%.2f m\n", lat, lon, alt);
    return true;
}

// GPS raw → NED 변환
GPS_NED gps_to_ned(const GPS_Raw &raw) {
    GPS_NED ned = {};
    ned.fixType = raw.fixType;
    ned.numSV   = raw.numSV;
    ned.tow     = raw.iTOW;
    ned.valid   = (raw.fixType >= 3);

    // velocity [mm/s → m/s]
    ned.vn = raw.velN * 0.001f;
    ned.ve = raw.velE * 0.001f;
    ned.vd = raw.velD * 0.001f;

    // accuracy
    ned.hAcc = raw.hAcc * 0.001f;             // [mm → m]
    ned.vAcc = raw.vAcc * 0.001f;             // [mm → m]
    ned.sAcc = raw.sAcc * 0.001f;             // [mm/s → m/s]

    // position [NED m]
    if (origin_set) {
        double lat = raw.lat * 1e-7 * DEG2RAD;
        double lon = raw.lon * 1e-7 * DEG2RAD;
        double alt = raw.alt * 1e-3;

        ned.n = (float)((lat - origin_lat) * R_EARTH);
        ned.e = (float)((lon - origin_lon) * R_EARTH * cos(origin_lat));
        ned.d = (float)(-(alt - origin_alt));
    }
    return ned;
}

// ── Serial Command ──────────────────────────────────────
static String cmdBuf;

void processCommand(const String &cmd) {
    if (cmd == "CAL" || cmd == "CALIBRATE") {
        GPS_Raw raw = gps.getRaw();
        if (gps_calibrate(raw)) {
            Serial.println("Origin calibrated.");
        } else {
            Serial.println("Calibration failed (need 3D fix).");
        }
    } else if (cmd == "STATUS") {
        GPS_Raw raw = gps.getRaw();
        Serial.printf("Fix: %s | Calibrated: %s | Sats: %d\n",
                       gps.hasFix() ? "3D" : "NO",
                       origin_set ? "YES" : "NO",
                       raw.numSV);
    } else {
        Serial.println("Commands: CAL | STATUS");
    }
}

// ── Setup & Loop ────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("=== NEO-M9N GPS (Rocket Mode) ===");

    if (!gps.begin(25)) {
        Serial.println("[GPS] Init FAILED — check wiring");
        while (1) delay(1000);
    }

    Serial.println("[GPS] Ready. Type CAL to set origin.");
}

void loop() {
    if (gps.update()) {
        GPS_Raw raw = gps.getRaw();
        GPS_NED ned = gps_to_ned(raw);

        Serial.printf("N:%7.2f E:%7.2f D:%7.2f [m]  "
                       "Vn:%6.2f Ve:%6.2f Vd:%6.2f [m/s]  "
                       "fix=%d sats=%d %s\n",
                       ned.n, ned.e, ned.d,
                       ned.vn, ned.ve, ned.vd,
                       ned.fixType, ned.numSV,
                       origin_set ? "" : "(no origin)");
    }

    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            cmdBuf.trim();
            cmdBuf.toUpperCase();
            if (cmdBuf.length() > 0) processCommand(cmdBuf);
            cmdBuf = "";
        } else {
            cmdBuf += c;
        }
    }
}
