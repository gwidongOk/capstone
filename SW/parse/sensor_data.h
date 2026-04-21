#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <stdint.h>

#pragma pack(push, 1)

// ★ 파서가 인식할 수 있도록 ID 정의를 반드시 추가해야 합니다.
#define ID_BARO 1
#define ID_IMU  2
#define ID_MAG  3


struct PacketHeader {
  uint8_t SYNC_BYTE = 0xAA;
  uint8_t id;   // 센서 종류 (1=baro, 2=imu, 3=mag)
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

// ID=3 : 지자기 패킷 (19 bytes) — 하드/소프트 아이언 보정 완료, Gauss 단위
struct mag_pkt {
  PacketHeader header;
  uint32_t t;          // 타임스탬프 [µs]
  float mx;            // 보정된 자기장 X [Gauss]
  float my;            // 보정된 자기장 Y [Gauss]
  float mz;            // 보정된 자기장 Z [Gauss]
};

#pragma pack(pop)

#endif