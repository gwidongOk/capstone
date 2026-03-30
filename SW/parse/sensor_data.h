#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H
#include <stdint.h>
#pragma pack(push, 1)
#define SYNC_BYTE 0xAA

struct PacketHeader {
  uint8_t sync;  // 동기화 바이트 (0xAA)
  uint8_t id;    // 구조체 종류 (0~255)
  uint8_t len;   // 이 패킷의 전체 크기
};

#define ID_BARO 1
#define ID_IMU  2

struct baro {
  PacketHeader header;
  uint32_t t;
  float rawalt;
};

struct imu {
  PacketHeader header;
  uint32_t t;
  uint16_t rawacc_x;
  uint16_t rawacc_y;
  uint16_t rawacc_z;
  uint16_t rawgyro_x;
  uint16_t rawgyro_y;
  uint16_t rawgyro_z;
};

#pragma pack(pop)

#endif
