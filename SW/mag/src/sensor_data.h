#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <stdint.h>

#pragma pack(push, 1)

// ★ parse/sensor_data.h 와 반드시 동일하게 유지!
#define ID_BARO 1
#define ID_IMU  2
#define ID_MAG  3

struct PacketHeader {
  uint8_t SYNC_BYTE = 0xAA;
  uint8_t id;
  uint8_t len;
};

// ID=3 : 지자기 패킷 (19 bytes) — 하드/소프트 아이언 보정 완료, Gauss 단위
struct mag_pkt {
  PacketHeader header;
  uint32_t t;          // 타임스탬프 [µs]
  float mx;
  float my;
  float mz;
};

#pragma pack(pop)

#endif
