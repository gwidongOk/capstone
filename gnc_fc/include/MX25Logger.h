#ifndef MX25LOGGER_H
#define MX25LOGGER_H

#include <Arduino.h>
#include <SPI.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

#include "sensor_data.h"

class MX25Logger {
  public:
    MX25Logger();

    bool begin(SPIClass *spi, int sck, int miso, int mosi, int cs, SemaphoreHandle_t spiMutex = NULL);
    void eraseAll();

    template <typename T>
    void appendData(T& data) {
      xSemaphoreTake(_bufferMutex, portMAX_DELAY);
      uint16_t len = sizeof(T);
      if (_bufferIndex + len <= BUFFER_SIZE) {
        memcpy(&_dataBuffer[_bufferIndex], &data, len);
        _bufferIndex += len;
      }
      xSemaphoreGive(_bufferMutex);
    }

    void appendRaw(const uint8_t *data, uint16_t len) {
      xSemaphoreTake(_bufferMutex, portMAX_DELAY);
      if (_bufferIndex + len <= BUFFER_SIZE) {
        memcpy(&_dataBuffer[_bufferIndex], data, len);
        _bufferIndex += len;
      }
      xSemaphoreGive(_bufferMutex);
    }

    void flushPages();

    void forceFlushBuffer();

    void dumpRawBinary(Stream &out = Serial);

    bool hasFullPage();

    uint32_t getStartAddress() { return START_ADDRESS; }
    uint32_t getCurrentAddress() { return _currentFlashAddress; }

    // Typed log entry points
    // To change the on-flash format, edit only the corresponding body.
    void logImu  (const Raw_imu       &raw);
    void logBaro (const Raw_press     &p);
    void logMag  (const Raw_mag       &m);
    void logGps  (const Raw_gps       &g);
    void logState(const State_nominal &nom);
    void logEvent(FlightPhase phase, uint8_t eventId);
    void logGnc(float nz_cmd, float ny_cmd, const float fin_cmd[4], uint8_t phase);

    void serviceFlush();

    // Logging gate
    void setEnabled(bool e) { _enabled = e; }
    bool isEnabled() const  { return _enabled; }

  private:
    SPIClass *_spi;
    int _csPin;
    SemaphoreHandle_t _spiMutex;
    uint32_t _currentFlashAddress;

    static const uint16_t BUFFER_SIZE = 2048;
    uint8_t _dataBuffer[BUFFER_SIZE];
    uint16_t _bufferIndex;

    SemaphoreHandle_t _bufferMutex;
    Preferences _prefs;

    const uint32_t START_ADDRESS = 0x0000000;
    // MX25L25645GM2I-08G : 256Mbit = 32MB
    static const uint32_t MAX_ADDRESS = 0x02000000;

    static const uint8_t  ITEM_MAX_SIZE = 80;   // largest packet (state_pkt = 71 B)
    static const uint16_t QUEUE_LENGTH  = 128;
    struct Item {
      uint8_t data[ITEM_MAX_SIZE];
      uint8_t len;
    };
    QueueHandle_t _queue;
    volatile bool _enabled;
    void _push(const void *pkt, uint8_t len);

    void saveAddress();
    void loadAddress();

    void writePage(uint8_t *page);
    void readFlash(uint32_t addr, uint8_t *buf, uint32_t len);
    void eraseSector(uint32_t addr);
    void writeEnable();
    void waitUntilDone();
    void enter4ByteMode();
};

#endif
