#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <SPI.h>
#include "LSM6DSO32.h"
#include "BMP388.h"
#include "NAV.h"
#include "MX25Logger.h"
#include "sensor_data.h"

// ============================================================
// 하드웨어 핀 설정 (2026 ALTIS AVIONICS V1.1)
// ============================================================
#define SPI_SCK_PIN   13
#define SPI_MISO_PIN  12
#define SPI_MOSI_PIN  11
#define IMU_CS_PIN    14
#define BMP_CS_PIN    10
#define IMU_INT1_PIN  47
#define BMP_INT_PIN   9

#define FLASH_SCK_PIN   16
#define FLASH_MISO_PIN  7
#define FLASH_MOSI_PIN  15
#define FLASH_CS_PIN    6

// ============================================================
// 시스템 상수 및 구조체
// ============================================================
#define EVENT_IMU_UPDATE  (1 << 0)
#define EVENT_BMP_UPDATE  (1 << 1)

#define LOG_ITEM_MAX_SIZE 32   // state_pkt = 3+4+4+4 = 15B, fits OK
#define LOG_QUEUE_LENGTH  256  // increased from 48 — KF state packets add load
#define BLE_DEVICE_NAME   "CANARD_2026"
#define SERIAL_BAUD       921600

struct LogItem {
  uint8_t data[LOG_ITEM_MAX_SIZE];
  uint8_t len;
};

// ============================================================
// 전역 객체 extern 선언 (main.cpp에서 실제 할당)
// ============================================================
extern SPIClass sensorSPI;
extern SPIClass flashSPI;
extern LSM6DSO32 imu;
extern BMP388 bmp;
extern NAV nav;
extern MX25Logger logger;

extern TaskHandle_t TaskHandle_IMU;
extern TaskHandle_t TaskHandle_BMP;
extern TaskHandle_t TaskHandle_NAV;
extern TaskHandle_t FlushTaskHandle;

extern SemaphoreHandle_t spiMutex;
extern SemaphoreHandle_t dataMutex;
extern SemaphoreHandle_t flashMutex;
extern QueueHandle_t logQueue;

extern volatile bool isLogging;
extern volatile bool systemStarted;

#endif