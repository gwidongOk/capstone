#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "LSM6DSO32.h"
#include "BMP388.h"
#include "MMC5983MA.h"
#include "NEOM9N.h"
#include "NAV.h"
#include "MX25Logger.h"
#include "sensor_data.h"

// ============================================================
// Hardware pins (2026 ALTIS AVIONICS V1.1)
// ============================================================

// Sensor SPI (IMU + BARO)
#define SPI_SCK_PIN   13
#define SPI_MISO_PIN  12
#define SPI_MOSI_PIN  11
#define IMU_CS_PIN    14
#define BMP_CS_PIN    10
#define IMU_INT1_PIN  47
#define BMP_INT_PIN   9

// Flash SPI
#define FLASH_SCK_PIN   16
#define FLASH_MISO_PIN  7
#define FLASH_MOSI_PIN  15
#define FLASH_CS_PIN    6

// Magnetometer I2C
#define MAG_SDA_PIN  5
#define MAG_SCL_PIN  4

// GPS UART (Serial1)
#define GPS_RX_PIN   2
#define GPS_TX_PIN   1

// ============================================================
// RTOS events (NAV task wakes on these)
// ============================================================
#define EVENT_IMU_UPDATE  (1 << 0)
#define EVENT_BMP_UPDATE  (1 << 1)
#define EVENT_MAG_UPDATE  (1 << 2)
#define EVENT_GPS_UPDATE  (1 << 3)

// ============================================================
// Log queue: holds one packet per item, flushed to flash by FlushTask (core 0)
// Flash write stays page-based (256 B pages from MX25Logger._dataBuffer).
// ============================================================
#define LOG_ITEM_MAX_SIZE 80       // fits largest packet (state_pkt = 71 B)
#define LOG_QUEUE_LENGTH  128      // safety headroom (128 × 81 B ≈ 10 KB RAM)

struct LogItem {
  uint8_t data[LOG_ITEM_MAX_SIZE];
  uint8_t len;
};

// ============================================================
// Global objects (defined in main.cpp)
// ============================================================
extern SPIClass sensorSPI;
extern SPIClass flashSPI;

extern LSM6DSO32 imu;
extern BMP388    bmp;
extern MMC5983MA mag;
extern NEOM9N    gps;
extern NAV       nav;
extern MX25Logger logger;

extern TaskHandle_t TaskHandle_IMU;
extern TaskHandle_t TaskHandle_BMP;
extern TaskHandle_t TaskHandle_MAG;
extern TaskHandle_t TaskHandle_GPS;
extern TaskHandle_t TaskHandle_NAV;
extern TaskHandle_t FlushTaskHandle;

extern SemaphoreHandle_t spiMutex;
extern SemaphoreHandle_t i2cMutex;
extern SemaphoreHandle_t dataMutex;
extern SemaphoreHandle_t flashMutex;
extern QueueHandle_t     logQueue;

extern volatile bool isLogging;
extern volatile bool systemStarted;

#endif
