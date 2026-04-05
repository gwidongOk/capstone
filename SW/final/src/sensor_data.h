#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <stdint.h>

#pragma pack(push, 1)

// 공용 헤더 구조체 정의
struct PacketHeader {
  uint8_t SYNC_BYTE = 0xAA;
  uint8_t id;   // 센서 종류 (1=baro, 2=imu)
  uint8_t len;  // 패킷 전체 크기 (헤더 포함)
};

// ID=1 : 기압계 패킷 (11 bytes)
struct baro_pkt {
  PacketHeader header;
  uint32_t t;          // 타임스탬프 [µs] (하위 32비트, ~71분 wrap)
  float rawalt;        // 상대 고도 [m]
};

// ID=2 : IMU 패킷 (19 bytes)
struct imu_pkt {
  PacketHeader header;
  uint32_t t;          // 타임스탬프 [µs]
  int16_t rawacc_x;    // Raw 가속도 (±32g, 0.976 mg/LSB)
  int16_t rawacc_y;
  int16_t rawacc_z;
  int16_t rawgyro_x;   // Raw 자이로 (±2000 dps, 70 mdps/LSB)
  int16_t rawgyro_y;
  int16_t rawgyro_z;
};

#pragma pack(pop)

#endif
