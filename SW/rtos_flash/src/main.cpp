#include <Arduino.h>
#include <SPI.h>
#include "MX25Logger.h"
#include "sensor_data.h"


#define FLASH_SCK_PIN   16
#define FLASH_MISO_PIN  7
#define FLASH_MOSI_PIN  15
#define FLASH_CS_PIN    6

// ★ 핵심: ESP32-S3의 남는 SPI 버스(FSPI)를 독립적으로 할당합니다.
// (일반 센서들은 기본 SPI(VSPI)를 사용하게 두면 완전히 격리됩니다.)
SPIClass flashSPI(FSPI); 

MX25Logger logger;

bool isLogging = true;      
TaskHandle_t LoggerTaskHandle;

// ========================================================
// FreeRTOS 전용 로깅 태스크
// ========================================================
void LoggerTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(50); // 50ms (20Hz)

  while (true) {
    if (isLogging) {
      // 센서 데이터 생성
      baro myBaro;
      myBaro.header.id = 1;
      myBaro.header.len = sizeof(baro); // 구조체의 전체 크기 자동 입력
      myBaro.t = millis();
      myBaro.rawalt = 105.3f;

      imu myImu;
      myImu.header.id = 2;
      myImu.header.len = sizeof(imu);
      myImu.t = millis();
      myImu.rawacc_x = 200;

      // 라이브러리로 전송
      logger.logData(myBaro);
      logger.logData(myImu);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) vTaskDelay(pdMS_TO_TICKS(10));
  vTaskDelay(pdMS_TO_TICKS(1000));

  logger.begin(&flashSPI, FLASH_SCK_PIN, FLASH_MISO_PIN, FLASH_MOSI_PIN, FLASH_CS_PIN);
  logger.eraseAll(); 

  xTaskCreatePinnedToCore(
    LoggerTask, "LoggerTask", 4096, NULL, 1, &LoggerTaskHandle, 1
  );
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if ((cmd == 'p' || cmd == 'P') && isLogging) {
      isLogging = false; 
      logger.forceFlushBuffer(); 
      logger.dumpRawBinary(Serial); 
    }
  }
  
  vTaskDelay(pdMS_TO_TICKS(10)); 
}