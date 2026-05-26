#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <ctype.h>

static constexpr uint32_t SERIAL_BAUD = 115200;

static constexpr int SPI_SCK_PIN  = 13;
static constexpr int SPI_MISO_PIN = 12;
static constexpr int SPI_MOSI_PIN = 11;
static constexpr int IMU_CS_PIN   = 14;

static constexpr int MAG_SDA_PIN = 2;
static constexpr int MAG_SCL_PIN = 1;
static constexpr int LED_PIN     = 48;

static constexpr float G0 = 9.80665f;
static constexpr float ACCEL_LSB_TO_MPS2 = 0.976f * 0.001f * G0;
static constexpr float TRIAD_RAD_TO_DEG = 57.2957795131f;

static constexpr uint16_t TRIAD_SAMPLES = 100;
static constexpr uint16_t TRIAD_DT_MS   = 20;
static constexpr uint32_t MAGCAL_MS     = 30000;

static constexpr float M_REF_NED[3] = {0.5961f, -0.0838f, 0.7986f};

static SPIClass sensorSPI(HSPI);
static SPISettings imuSpi(5000000, MSBFIRST, SPI_MODE3);

struct Vec3 {
  float x;
  float y;
  float z;
};

struct Quat {
  float w;
  float x;
  float y;
  float z;
};

struct MagCal {
  bool valid;
  Vec3 bias;
  Vec3 scale;
};

static MagCal magCal = {
  false,
  {0.0f, 0.0f, 0.0f},
  {1.0f, 1.0f, 1.0f}
};

static bool streamMode = false;
static uint32_t lastStreamMs = 0;

static float vecDot(const Vec3& a, const Vec3& b) {
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

static Vec3 vecCross(const Vec3& a, const Vec3& b) {
  return {
    a.y*b.z - a.z*b.y,
    a.z*b.x - a.x*b.z,
    a.x*b.y - a.y*b.x
  };
}

static float vecNorm(const Vec3& v) {
  return sqrtf(vecDot(v, v));
}

static bool vecNormalize(Vec3& v) {
  const float n = vecNorm(v);
  if (n < 1.0e-6f) return false;
  const float inv = 1.0f / n;
  v.x *= inv;
  v.y *= inv;
  v.z *= inv;
  return true;
}

static Vec3 vecSub(const Vec3& a, const Vec3& b) {
  return {a.x-b.x, a.y-b.y, a.z-b.z};
}

static Vec3 vecMul(const Vec3& a, const Vec3& b) {
  return {a.x*b.x, a.y*b.y, a.z*b.z};
}

static void printVec(const char* name, const Vec3& v, uint8_t prec) {
  Serial.print(name);
  Serial.print(": ");
  Serial.print(v.x, prec);
  Serial.print(", ");
  Serial.print(v.y, prec);
  Serial.print(", ");
  Serial.println(v.z, prec);
}

static void imuWrite(uint8_t reg, uint8_t val) {
  sensorSPI.beginTransaction(imuSpi);
  digitalWrite(IMU_CS_PIN, LOW);
  sensorSPI.transfer(reg & 0x7F);
  sensorSPI.transfer(val);
  digitalWrite(IMU_CS_PIN, HIGH);
  sensorSPI.endTransaction();
}

static uint8_t imuRead(uint8_t reg) {
  sensorSPI.beginTransaction(imuSpi);
  digitalWrite(IMU_CS_PIN, LOW);
  sensorSPI.transfer(reg | 0x80);
  const uint8_t val = sensorSPI.transfer(0x00);
  digitalWrite(IMU_CS_PIN, HIGH);
  sensorSPI.endTransaction();
  return val;
}

static void imuReadBurst(uint8_t reg, uint8_t* buf, uint8_t len) {
  sensorSPI.beginTransaction(imuSpi);
  digitalWrite(IMU_CS_PIN, LOW);
  sensorSPI.transfer(reg | 0x80);
  for (uint8_t i = 0; i < len; i++) buf[i] = sensorSPI.transfer(0x00);
  digitalWrite(IMU_CS_PIN, HIGH);
  sensorSPI.endTransaction();
}

static bool imuBegin() {
  pinMode(IMU_CS_PIN, OUTPUT);
  digitalWrite(IMU_CS_PIN, HIGH);
  delay(10);

  const uint8_t who = imuRead(0x0F);
  if (who != 0x6C) {
    Serial.print("IMU FAIL WHO=0x");
    Serial.println(who, HEX);
    return false;
  }

  imuWrite(0x12, 0x01);
  delay(10);
  imuWrite(0x12, 0x44);
  imuWrite(0x13, 0x04);
  imuWrite(0x10, 0x64);
  imuWrite(0x11, 0x6C);

  uint8_t ctrl1 = imuRead(0x10);
  imuWrite(0x10, ctrl1 | 0x02);
  uint8_t ctrl8 = imuRead(0x17);
  ctrl8 &= 0x1F;
  ctrl8 |= (0x01 << 5);
  imuWrite(0x17, ctrl8);

  uint8_t ctrl4 = imuRead(0x13);
  imuWrite(0x13, ctrl4 | 0x02);
  uint8_t ctrl6 = imuRead(0x15);
  ctrl6 &= 0xF8;
  imuWrite(0x15, ctrl6);
  return true;
}

static bool readAccel(Vec3& acc) {
  uint8_t b[12];
  imuReadBurst(0x22, b, 12);

  const int16_t axRaw = (int16_t)((b[7]  << 8) | b[6]);
  const int16_t ayRaw = (int16_t)((b[9]  << 8) | b[8]);
  const int16_t azRaw = (int16_t)((b[11] << 8) | b[10]);

  acc.x = (float)ayRaw * ACCEL_LSB_TO_MPS2;
  acc.y = (float)axRaw * ACCEL_LSB_TO_MPS2;
  acc.z = (float)(-azRaw) * ACCEL_LSB_TO_MPS2;
  return true;
}

static void magWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(0x30);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static uint8_t magReadReg(uint8_t reg) {
  Wire.beginTransmission(0x30);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x30, (uint8_t)1);
  if (Wire.available() < 1) return 0;
  return Wire.read();
}

static bool magReadBurst(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(0x30);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  const uint8_t got = Wire.requestFrom((uint8_t)0x30, len);
  if (got != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

static bool magBegin() {
  const uint8_t id = magReadReg(0x2F);
  if (id != 0x30) {
    Serial.print("MAG FAIL ID=0x");
    Serial.println(id, HEX);
    return false;
  }

  magWrite(0x0A, 0x80);
  delay(10);
  magWrite(0x09, 0x20);
  magWrite(0x0A, 0x00);
  magWrite(0x0B, 0xAC);
  delay(20);
  return true;
}

static bool readMagRaw(Vec3& mag) {
  uint8_t b[7];
  if (!magReadBurst(0x00, b, 7)) return false;

  const uint32_t rx = ((uint32_t)b[0] << 10) | ((uint32_t)b[1] << 2) | (b[6] >> 6);
  const uint32_t ry = ((uint32_t)b[2] << 10) | ((uint32_t)b[3] << 2) | ((b[6] >> 4) & 0x03);
  const uint32_t rz = ((uint32_t)b[4] << 10) | ((uint32_t)b[5] << 2) | ((b[6] >> 2) & 0x03);

  mag.x = ((float)rx - 131072.0f) / 16384.0f;
  mag.y = ((float)ry - 131072.0f) / 16384.0f;
  mag.z = ((float)rz - 131072.0f) / 16384.0f;
  return true;
}

static bool readMag(Vec3& mag) {
  Vec3 raw;
  if (!readMagRaw(raw)) return false;
  if (magCal.valid) {
    mag = vecMul(vecSub(raw, magCal.bias), magCal.scale);
  } else {
    mag = raw;
  }
  return true;
}

static void buildFrame(const Vec3& v1, const Vec3& v2, float M[3][3]) {
  Vec3 u1 = v1;
  Vec3 u2 = vecCross(u1, v2);
  vecNormalize(u2);
  Vec3 u3 = vecCross(u1, u2);

  M[0][0] = u1.x; M[1][0] = u1.y; M[2][0] = u1.z;
  M[0][1] = u2.x; M[1][1] = u2.y; M[2][1] = u2.z;
  M[0][2] = u3.x; M[1][2] = u3.y; M[2][2] = u3.z;
}

static void matMulAT(float A[3][3], float B[3][3], float C[3][3]) {
  for (uint8_t r = 0; r < 3; r++) {
    for (uint8_t c = 0; c < 3; c++) {
      C[r][c] = A[r][0]*B[c][0] + A[r][1]*B[c][1] + A[r][2]*B[c][2];
    }
  }
}

static void dcmToQuat(float R[3][3], Quat& q) {
  const float tr = R[0][0] + R[1][1] + R[2][2];
  if (tr > R[0][0] && tr > R[1][1] && tr > R[2][2]) {
    q.w = 0.5f * sqrtf(1.0f + tr);
    const float k = 0.25f / q.w;
    q.x = k * (R[2][1] - R[1][2]);
    q.y = k * (R[0][2] - R[2][0]);
    q.z = k * (R[1][0] - R[0][1]);
  } else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
    q.x = 0.5f * sqrtf(1.0f + 2.0f*R[0][0] - tr);
    const float k = 0.25f / q.x;
    q.w = k * (R[2][1] - R[1][2]);
    q.y = k * (R[0][1] + R[1][0]);
    q.z = k * (R[0][2] + R[2][0]);
  } else if (R[1][1] > R[2][2]) {
    q.y = 0.5f * sqrtf(1.0f + 2.0f*R[1][1] - tr);
    const float k = 0.25f / q.y;
    q.w = k * (R[0][2] - R[2][0]);
    q.x = k * (R[0][1] + R[1][0]);
    q.z = k * (R[1][2] + R[2][1]);
  } else {
    q.z = 0.5f * sqrtf(1.0f + 2.0f*R[2][2] - tr);
    const float k = 0.25f / q.z;
    q.w = k * (R[1][0] - R[0][1]);
    q.x = k * (R[0][2] + R[2][0]);
    q.y = k * (R[1][2] + R[2][1]);
  }

  if (q.w < 0.0f) {
    q.w = -q.w; q.x = -q.x; q.y = -q.y; q.z = -q.z;
  }
  const float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  if (n > 1.0e-6f) {
    const float inv = 1.0f / n;
    q.w *= inv; q.x *= inv; q.y *= inv; q.z *= inv;
  }
}

static void quatToEuler(const Quat& q, Vec3& rpy) {
  const float sinr = 2.0f * (q.w*q.x + q.y*q.z);
  const float cosr = 1.0f - 2.0f * (q.x*q.x + q.y*q.y);
  rpy.x = atan2f(sinr, cosr);

  const float sinp = 2.0f * (q.w*q.y - q.z*q.x);
  if (fabsf(sinp) >= 1.0f) {
    rpy.y = (sinp > 0.0f) ? 1.57079632679f : -1.57079632679f;
  } else {
    rpy.y = asinf(sinp);
  }

  const float siny = 2.0f * (q.w*q.z + q.x*q.y);
  const float cosy = 1.0f - 2.0f * (q.y*q.y + q.z*q.z);
  rpy.z = atan2f(siny, cosy);
}

static bool triad(const Vec3& accIn, const Vec3& magIn, Quat& q, float R[3][3], float& parallel) {
  Vec3 r1 = {0.0f, 0.0f, 1.0f};
  Vec3 r2 = {M_REF_NED[0], M_REF_NED[1], M_REF_NED[2]};
  Vec3 b1 = {-accIn.x, -accIn.y, -accIn.z};
  Vec3 b2 = magIn;

  if (!vecNormalize(r2) || !vecNormalize(b1) || !vecNormalize(b2)) return false;
  parallel = fabsf(vecDot(b1, b2));
  if (parallel > 0.999f) return false;

  float Mref[3][3];
  float Mbody[3][3];
  buildFrame(r1, r2, Mref);
  buildFrame(b1, b2, Mbody);
  matMulAT(Mref, Mbody, R);
  dcmToQuat(R, q);
  return true;
}

static bool averageSensors(Vec3& accAvg, Vec3& magAvg) {
  Vec3 accSum = {0.0f, 0.0f, 0.0f};
  Vec3 magSum = {0.0f, 0.0f, 0.0f};
  uint16_t magCount = 0;

  for (uint16_t i = 0; i < TRIAD_SAMPLES; i++) {
    Vec3 a;
    Vec3 m;
    readAccel(a);
    accSum.x += a.x;
    accSum.y += a.y;
    accSum.z += a.z;

    if (readMag(m)) {
      magSum.x += m.x;
      magSum.y += m.y;
      magSum.z += m.z;
      magCount++;
    }
    delay(TRIAD_DT_MS);
  }

  const float invAcc = 1.0f / (float)TRIAD_SAMPLES;
  accAvg.x = accSum.x * invAcc;
  accAvg.y = accSum.y * invAcc;
  accAvg.z = accSum.z * invAcc;

  if (magCount == 0) return false;
  const float invMag = 1.0f / (float)magCount;
  magAvg.x = magSum.x * invMag;
  magAvg.y = magSum.y * invMag;
  magAvg.z = magSum.z * invMag;
  return true;
}

static void printTriad() {
  digitalWrite(LED_PIN, HIGH);

  Vec3 acc;
  Vec3 mag;
  if (!averageSensors(acc, mag)) {
    Serial.println("TRIAD FAIL: MAG READ");
    digitalWrite(LED_PIN, LOW);
    return;
  }

  const float accNorm = vecNorm(acc);
  const float magNorm = vecNorm(mag);

  Quat q;
  float R[3][3];
  float parallel = 1.0f;
  const bool ok = triad(acc, mag, q, R, parallel);

  Serial.println();
  Serial.println("TRIAD SAMPLE");
  printVec("acc_mps2", acc, 5);
  Serial.print("acc_norm_g: ");
  Serial.println(accNorm / G0, 5);
  printVec("mag_gauss", mag, 5);
  Serial.print("mag_norm_gs: ");
  Serial.println(magNorm, 5);
  Serial.print("acc_mag_parallel: ");
  Serial.println(parallel, 6);

  if (!ok) {
    Serial.println("TRIAD FAIL: GEOMETRY");
    digitalWrite(LED_PIN, LOW);
    return;
  }

  Vec3 rpy;
  quatToEuler(q, rpy);

  Serial.print("q_wxyz: ");
  Serial.print(q.w, 7); Serial.print(", ");
  Serial.print(q.x, 7); Serial.print(", ");
  Serial.print(q.y, 7); Serial.print(", ");
  Serial.println(q.z, 7);

  Serial.print("rpy_deg: ");
  Serial.print(rpy.x * TRIAD_RAD_TO_DEG, 3); Serial.print(", ");
  Serial.print(rpy.y * TRIAD_RAD_TO_DEG, 3); Serial.print(", ");
  Serial.println(rpy.z * TRIAD_RAD_TO_DEG, 3);

  Serial.println("R_bn:");
  for (uint8_t r = 0; r < 3; r++) {
    Serial.print(R[r][0], 6); Serial.print(", ");
    Serial.print(R[r][1], 6); Serial.print(", ");
    Serial.println(R[r][2], 6);
  }

  const bool accOk = (accNorm > 7.0f && accNorm < 12.5f);
  const bool magOk = (magNorm > 0.20f && magNorm < 0.80f);
  const bool geoOk = (parallel < 0.98f);
  Serial.print("CHECK: ");
  Serial.println((accOk && magOk && geoOk) ? "PASS" : "WARN");

  digitalWrite(LED_PIN, LOW);
}

static void printRaw() {
  Vec3 acc;
  Vec3 magRaw;
  Vec3 mag;
  readAccel(acc);
  readMagRaw(magRaw);
  readMag(mag);

  Serial.println();
  printVec("acc_mps2", acc, 5);
  printVec("mag_raw_gauss", magRaw, 5);
  printVec("mag_cal_gauss", mag, 5);
}

static void runMagCal() {
  Serial.println();
  Serial.println("MAGCAL START");
  Serial.println("Rotate all axes for 30 s");

  Vec3 minV = { 1000.0f,  1000.0f,  1000.0f};
  Vec3 maxV = {-1000.0f, -1000.0f, -1000.0f};
  uint32_t lastPrint = 0;
  uint16_t count = 0;
  const uint32_t start = millis();

  while ((uint32_t)(millis() - start) < MAGCAL_MS) {
    Vec3 m;
    if (readMagRaw(m)) {
      if (m.x < minV.x) minV.x = m.x;
      if (m.y < minV.y) minV.y = m.y;
      if (m.z < minV.z) minV.z = m.z;
      if (m.x > maxV.x) maxV.x = m.x;
      if (m.y > maxV.y) maxV.y = m.y;
      if (m.z > maxV.z) maxV.z = m.z;
      count++;
    }

    const uint32_t now = millis();
    if (now - lastPrint >= 1000) {
      lastPrint = now;
      Serial.print("MAGCAL ");
      Serial.print((now - start) / 1000);
      Serial.println(" s");
    }
    delay(20);
  }

  const Vec3 spread = {maxV.x-minV.x, maxV.y-minV.y, maxV.z-minV.z};
  if (count < 100 || spread.x < 0.30f || spread.y < 0.30f || spread.z < 0.30f) {
    Serial.println("MAGCAL FAIL");
    printVec("spread", spread, 5);
    return;
  }

  const Vec3 radius = {spread.x*0.5f, spread.y*0.5f, spread.z*0.5f};
  const float avgRadius = (radius.x + radius.y + radius.z) / 3.0f;

  magCal.bias = {
    (maxV.x + minV.x) * 0.5f,
    (maxV.y + minV.y) * 0.5f,
    (maxV.z + minV.z) * 0.5f
  };
  magCal.scale = {
    avgRadius / radius.x,
    avgRadius / radius.y,
    avgRadius / radius.z
  };
  magCal.valid = true;

  Serial.println("MAGCAL OK");
  printVec("bias", magCal.bias, 6);
  printVec("scale", magCal.scale, 6);
}

static void printHelp() {
  Serial.println();
  Serial.println("Commands");
  Serial.println("TRIAD  : 2 s average + TRIAD");
  Serial.println("RAW    : one raw sample");
  Serial.println("MAGCAL : 30 s mag calibration");
  Serial.println("STREAM : toggle TRIAD stream");
  Serial.println("HELP   : command list");
}

static void handleCommand(const char* cmd) {
  if (cmd[0] == '\0' || strcmp(cmd, "TRIAD") == 0) {
    printTriad();
  } else if (strcmp(cmd, "RAW") == 0) {
    printRaw();
  } else if (strcmp(cmd, "MAGCAL") == 0 || strcmp(cmd, "CALIBRATE_MAG") == 0) {
    runMagCal();
  } else if (strcmp(cmd, "STREAM") == 0) {
    streamMode = !streamMode;
    Serial.print("STREAM ");
    Serial.println(streamMode ? "ON" : "OFF");
  } else {
    printHelp();
  }
}

static void pollSerial() {
  static char cmd[24];
  static uint8_t idx = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      cmd[idx] = '\0';
      handleCommand(cmd);
      idx = 0;
      cmd[0] = '\0';
    } else if (idx < sizeof(cmd) - 1) {
      cmd[idx++] = (char)toupper((unsigned char)c);
    }
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(SERIAL_BAUD);
  delay(1500);

  sensorSPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN);
  Wire.begin(MAG_SDA_PIN, MAG_SCL_PIN, 400000);

  Serial.println();
  Serial.println("TRIAD CHECK");

  const bool imuOk = imuBegin();
  const bool magOk = magBegin();

  Serial.print("IMU: ");
  Serial.println(imuOk ? "OK" : "FAIL");
  Serial.print("MAG: ");
  Serial.println(magOk ? "OK" : "FAIL");

  printHelp();
}

void loop() {
  pollSerial();

  if (streamMode && (uint32_t)(millis() - lastStreamMs) > 3000) {
    lastStreamMs = millis();
    printTriad();
  }

  delay(5);
}
