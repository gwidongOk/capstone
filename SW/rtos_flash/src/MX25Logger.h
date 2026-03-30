#ifndef MX25LOGGER_H
#define MX25LOGGER_H

#include <Arduino.h>
#include <SPI.h>

class MX25Logger {
  public:
    MX25Logger();
    
    // ★ 수정: 특정 SPI 채널 객체의 포인터(&)를 받아 독립적으로 작동합니다.
    bool begin(SPIClass *spi, int sck, int miso, int mosi, int cs);
    void eraseAll();
    void forceFlushBuffer();

    // ★ 수정: 라이브러리 내부로 이동된 바이너리 덤프 함수
    // 인자로 Stream을 받아 Serial 뿐만 아니라 Bluetooth 등으로도 전송 가능합니다. (기본값: Serial)
    void dumpRawBinary(Stream &out = Serial);

    template <typename T>
    void logData(T& data) {
      appendAndSliceData((uint8_t*)&data, sizeof(T));
    }

    uint32_t getStartAddress() { return START_ADDRESS; }
    uint32_t getCurrentAddress() { return _currentFlashAddress; }

  private:
    SPIClass *_spi; // 전달받은 SPI 채널을 기억할 포인터 변수
    int _csPin;
    uint32_t _currentFlashAddress;
    
    static const uint16_t BUFFER_SIZE = 2048;
    uint8_t _dataBuffer[BUFFER_SIZE];
    uint16_t _bufferIndex;

    const uint32_t START_ADDRESS = 0x1000000;

    void appendAndSliceData(uint8_t *data, uint16_t len);
    void readFlash(uint32_t addr, uint8_t *buf, uint32_t len);
    void eraseSector(uint32_t addr);
    void writeEnable();
    void waitUntilDone();
    void enter4ByteMode();
};

#endif