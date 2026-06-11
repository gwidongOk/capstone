#ifndef PYTHON_HEADER_H
#define PYTHON_HEADER_H

#include <stdint.h>

#pragma pack(push, 1)

// Packet IDs (Used by Python Parser)
#define ID_BARO  1
#define ID_IMU   2
#define ID_MAG   3
#define ID_GPS   4
#define ID_STATE 5
#define ID_EVENT 6
#define ID_GNC   7

struct PacketHeader {
  uint8_t SYNC_BYTE = 0xAA;
  uint8_t id;
  uint8_t len;
};

// ID 1 : Barometer altitude
struct baro_pkt {
  PacketHeader header;
  uint32_t t;
  float alt;
};

// ID 2 : IMU raw
struct imu_pkt {
  PacketHeader header;
  uint32_t t;
  int16_t gx, gy, gz;
  int16_t ax, ay, az;
};

// ID 3 : Magnetometer
struct mag_pkt {
  PacketHeader header;
  uint32_t t;
  float mx, my, mz;
};

// ID 4 : GPS NED
struct gps_pkt {
  PacketHeader header;
  uint32_t t;
  float pn, pe, pd;
  float vn, ve, vd;
  float hAcc, vAcc, sAcc;
  uint8_t fixType;
  uint8_t numSV;
};

// ID 5 : ES-EKF nominal state (Flattened for Python header_parser.py)
struct state_pkt {
  PacketHeader header;
  uint32_t t;
  float pn, pe, pd;
  float vn, ve, vd;
  float qw, qx, qy, qz;
  float ba_x, ba_y, ba_z;
  float bg_x, bg_y, bg_z;
};

// ID 6 : Flight Event
struct event_pkt {
  PacketHeader header;
  uint32_t t;
  uint8_t  phase;
  uint8_t  event_id;
};

// ID 7 : GNC output
struct gnc_pkt {
  PacketHeader header;
  uint32_t t;
  float nz_cmd;
  float ny_cmd;
  float fin_cmd[4];
  uint8_t phase;
};

#pragma pack(pop)

#endif
