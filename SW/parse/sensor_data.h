#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <stdint.h>

#pragma pack(push, 1)

// ★ 파서가 인식할 수 있도록 ID 정의를 반드시 추가해야 합니다.
#define ID_BARO 1
#define ID_IMU  2

// 공용 헤더 구조체 정의 (3 bytes)
struct PacketHeader {
  uint8_t SYNC_BYTE; // 0xAA (Python 파서가 이 값을 검사함)
  uint8_t id;        // 센서 종류 (1=baro, 2=imu)
  uint8_t len;       // 패킷 전체 크기 (헤더 포함)
};

// ID=1 : 기압계 패킷 (11 bytes)
struct baro_pkt {
  PacketHeader header;
  uint32_t t;          
  float rawalt;        
};

// ID=2 : IMU 패킷 (19 bytes)
struct imu_pkt {
  PacketHeader header;
  uint32_t t;          
  int16_t rawacc_x;    
  int16_t rawacc_y;
  int16_t rawacc_z;
  int16_t rawgyro_x;   
  int16_t rawgyro_y;
  int16_t rawgyro_z;
};

#pragma pack(pop)

#endif