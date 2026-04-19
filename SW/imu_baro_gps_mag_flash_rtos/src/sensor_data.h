#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <stdint.h>

#pragma pack(push, 1)

struct PacketHeader {
  uint8_t SYNC_BYTE = 0xAA;
  uint8_t id;
  uint8_t len;
};

// ID 1 : Barometer altitude (m), pad-referenced
struct baro_pkt {
  PacketHeader header;
  uint32_t t;
  float alt;
};

// ID 2 : IMU calibrated raw (axis-aligned + int16 bias subtracted)
struct imu_pkt {
  PacketHeader header;
  uint32_t t;
  int16_t gx, gy, gz;
  int16_t ax, ay, az;
};

// ID 3 : Magnetometer fully calibrated Gauss (hard+soft iron + axis)
struct mag_pkt {
  PacketHeader header;
  uint32_t t;
  float mx, my, mz;
};

// ID 4 : GPS NED from origin + velocity + accuracy + fix info
struct gps_pkt {
  PacketHeader header;
  uint32_t t;
  float pn, pe, pd;
  float vn, ve, vd;
  float hAcc, vAcc;
  uint8_t fixType;
  uint8_t numSV;
};

// ID 5 : ES-EKF nominal state
struct state_pkt {
  PacketHeader header;
  uint32_t t;
  float p[3];
  float v[3];
  float q[4];
  float ba[3];
  float bg[3];
};

#pragma pack(pop)

#endif
