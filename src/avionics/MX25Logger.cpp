#include "MX25Logger.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define CMD_WREN         0x06
#define CMD_RDSR         0x05
#define CMD_WRITE        0x02
#define CMD_READ         0x03
#define CMD_EN4B         0xB7
#define CMD_SECTOR_ERASE 0x20
#define CMD_CHIP_ERASE   0xC7

static const char* NVS_NAMESPACE = "mx25log";
static const char* NVS_KEY_ADDR  = "endAddr";

MX25Logger::MX25Logger() {
  _spi = nullptr;
  _bufferIndex = 0;
  _currentFlashAddress = 0x0000000;
  _bufferMutex = xSemaphoreCreateMutex();
  _spiMutex = nullptr;
  _queue = NULL;
  _enabled = false;
}

// ----------------------------------------------------------------------------
// begin() : 플래시 로거 초기화
//   - FreeRTOS 큐 생성 (센서 태스크 → FlushTask 패킷 전달용)
//   - 32MB 칩의 4-byte 주소 모드 활성 (EN4B 명령)
//   - NVS에서 마지막 쓰기 주소 복구 (전원 재인가 후에도 이어 쓰기 위함)
// ----------------------------------------------------------------------------
bool MX25Logger::begin(SPIClass *spi, int sck, int miso, int mosi, int cs, SemaphoreHandle_t spiMutex) {
  _queue = xQueueCreate(QUEUE_LENGTH, sizeof(Item));
  if (!_queue) return false;

  _spi = spi;
  _csPin = cs;
  _spiMutex = spiMutex;
  _bufferIndex = 0;

  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);
  
  if (_spiMutex) xSemaphoreTake(_spiMutex, portMAX_DELAY);
  _spi->begin(sck, miso, mosi, _csPin);
  enter4ByteMode();
  if (_spiMutex) xSemaphoreGive(_spiMutex);

  vTaskDelay(pdMS_TO_TICKS(10));

  loadAddress();

  return true;
}

void MX25Logger::saveAddress() {
  _prefs.begin(NVS_NAMESPACE, false);
  _prefs.putUInt(NVS_KEY_ADDR, _currentFlashAddress);
  _prefs.end();
}

void MX25Logger::loadAddress() {
  _prefs.begin(NVS_NAMESPACE, true);   // read-only
  _currentFlashAddress = _prefs.getUInt(NVS_KEY_ADDR, START_ADDRESS);
  _prefs.end();

  if (_currentFlashAddress < START_ADDRESS || _currentFlashAddress >= MAX_ADDRESS) {
    Serial.printf("Flash addr 0x%08X out of range - resetting to START\n", _currentFlashAddress);
    _currentFlashAddress = START_ADDRESS;
  }

  Serial.printf("Flash resume addr: 0x%08X (%u bytes logged, %u bytes free)\n",
                _currentFlashAddress,
                _currentFlashAddress - START_ADDRESS,
                MAX_ADDRESS - _currentFlashAddress);
}

bool MX25Logger::hasFullPage() {
  xSemaphoreTake(_bufferMutex, portMAX_DELAY);
  bool result = (_bufferIndex >= 256);
  xSemaphoreGive(_bufferMutex);
  return result;
}

// ----------------------------------------------------------------------------
// flushPages() : 버퍼에 쌓인 256B 페이지를 차례로 플래시에 기록
//   - bufferMutex로 보호된 영역에서 페이지 복사 후 mutex 해제 → writePage
//   - writePage가 SPI 점유 동안 다른 센서 태스크가 큐에 push할 수 있도록 함
// ----------------------------------------------------------------------------
void MX25Logger::flushPages() {
  xSemaphoreTake(_bufferMutex, portMAX_DELAY);

  while (_bufferIndex >= 256) {
    uint8_t page[256];
    memcpy(page, _dataBuffer, 256);

    uint16_t remaining = _bufferIndex - 256;
    if (remaining > 0) {
      memmove(_dataBuffer, &_dataBuffer[256], remaining);
    }
    _bufferIndex = remaining;

    xSemaphoreGive(_bufferMutex);

    writePage(page);

    xSemaphoreTake(_bufferMutex, portMAX_DELAY);
  }

  xSemaphoreGive(_bufferMutex);
}

void MX25Logger::writePage(uint8_t *page) {
  if (_currentFlashAddress + 256 > MAX_ADDRESS) {
    if (_enabled) {
      _enabled = false;
      Serial.println("Flash FULL - logging disabled");
    }
    return;
  }

  if (_spiMutex) xSemaphoreTake(_spiMutex, portMAX_DELAY);

  // Note: Erase is handled pre-flight via eraseAll() to avoid latency.
  writeEnable();
  digitalWrite(_csPin, LOW); _spi->transfer(CMD_WRITE);
  _spi->transfer((_currentFlashAddress >> 24) & 0xFF);
  _spi->transfer((_currentFlashAddress >> 16) & 0xFF);
  _spi->transfer((_currentFlashAddress >> 8) & 0xFF);
  _spi->transfer(_currentFlashAddress & 0xFF);

  for (uint16_t i = 0; i < 256; i++) { _spi->transfer(page[i]); }
  digitalWrite(_csPin, HIGH);
  waitUntilDone();

  _currentFlashAddress += 256;

  if (_spiMutex) xSemaphoreGive(_spiMutex);
}

// ----------------------------------------------------------------------------
// forceFlushBuffer() : 마지막 잔여 < 256B도 강제 기록 (STOP / REBOOT 시)
//   - 페이지 정렬 후, 남은 부분은 잘려도 무방 (어차피 마지막 패킷)
//   - 끝에 NVS로 _currentFlashAddress 저장 → 다음 부팅에 이어쓰기
// ----------------------------------------------------------------------------
void MX25Logger::forceFlushBuffer() {
  flushPages();

  xSemaphoreTake(_bufferMutex, portMAX_DELAY);
  if (_bufferIndex == 0) {
    xSemaphoreGive(_bufferMutex);
    saveAddress();
    return;
  }

  uint8_t remaining[256];
  uint16_t len = _bufferIndex;
  memcpy(remaining, _dataBuffer, len);
  _bufferIndex = 0;
  xSemaphoreGive(_bufferMutex);

  if (_currentFlashAddress + len > MAX_ADDRESS) {
    Serial.println("Flash FULL - dropping tail buffer");
    saveAddress();
    return;
  }

  if (_spiMutex) xSemaphoreTake(_spiMutex, portMAX_DELAY);

  // Note: Erase is handled pre-flight via eraseAll() to avoid latency.
  writeEnable();
  digitalWrite(_csPin, LOW); _spi->transfer(CMD_WRITE);
  _spi->transfer((_currentFlashAddress >> 24) & 0xFF);
  _spi->transfer((_currentFlashAddress >> 16) & 0xFF);
  _spi->transfer((_currentFlashAddress >> 8) & 0xFF);
  _spi->transfer(_currentFlashAddress & 0xFF);

  for (uint16_t i = 0; i < len; i++) { _spi->transfer(remaining[i]); }
  digitalWrite(_csPin, HIGH);
  waitUntilDone();

  _currentFlashAddress += len;
  
  if (_spiMutex) xSemaphoreGive(_spiMutex);

  saveAddress();
}

void MX25Logger::dumpRawBinary(Stream &out) {
  uint32_t readAddr = START_ADDRESS;
  uint8_t buffer[256];
  uint32_t totalBytes = _currentFlashAddress - START_ADDRESS;

  while (readAddr < _currentFlashAddress) {
    uint32_t readLen = _currentFlashAddress - readAddr;
    if (readLen > 256) readLen = 256;

    readFlash(readAddr, buffer, readLen);
    out.write(buffer, readLen);

    readAddr += readLen;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  out.flush();
}

// ----------------------------------------------------------------------------
// eraseAll() : 칩 전체 소거 (~80초 소요 가능)
//   - 비행 직전 ERASE 명령에서 호출. 비행 중에는 절대 호출 X (지연 폭탄)
//   - Status 레지스터 WIP 비트 폴링으로 완료 대기
// ----------------------------------------------------------------------------
void MX25Logger::eraseAll() {
  if (_spiMutex) xSemaphoreTake(_spiMutex, portMAX_DELAY);

  writeEnable();
  digitalWrite(_csPin, LOW);
  _spi->transfer(CMD_CHIP_ERASE);
  digitalWrite(_csPin, HIGH);

  digitalWrite(_csPin, LOW);
  _spi->transfer(CMD_RDSR);
  while ((_spi->transfer(0x00) & 0x01) == 1) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  digitalWrite(_csPin, HIGH);

  if (_spiMutex) xSemaphoreGive(_spiMutex);

  _currentFlashAddress = START_ADDRESS;
  if (_queue) xQueueReset(_queue);
  xSemaphoreTake(_bufferMutex, portMAX_DELAY);
  _bufferIndex = 0;
  xSemaphoreGive(_bufferMutex);

  saveAddress();
}

void MX25Logger::readFlash(uint32_t addr, uint8_t *buf, uint32_t len) {
  if (_spiMutex) xSemaphoreTake(_spiMutex, portMAX_DELAY);
  digitalWrite(_csPin, LOW); _spi->transfer(CMD_READ);
  _spi->transfer((addr >> 24) & 0xFF); _spi->transfer((addr >> 16) & 0xFF);
  _spi->transfer((addr >> 8) & 0xFF); _spi->transfer(addr & 0xFF);
  for (uint32_t i = 0; i < len; i++) { buf[i] = _spi->transfer(0x00); }
  digitalWrite(_csPin, HIGH);
  if (_spiMutex) xSemaphoreGive(_spiMutex);
}

void MX25Logger::eraseSector(uint32_t addr) {
  digitalWrite(_csPin, LOW); _spi->transfer(CMD_SECTOR_ERASE);
  _spi->transfer((addr >> 24) & 0xFF); _spi->transfer((addr >> 16) & 0xFF);
  _spi->transfer((addr >> 8) & 0xFF); _spi->transfer(addr & 0xFF);
  digitalWrite(_csPin, HIGH);
}

void MX25Logger::writeEnable() {
  digitalWrite(_csPin, LOW); _spi->transfer(CMD_WREN); digitalWrite(_csPin, HIGH);
}

void MX25Logger::waitUntilDone() {
  digitalWrite(_csPin, LOW); _spi->transfer(CMD_RDSR);
  while ((_spi->transfer(0x00) & 0x01) == 1) {
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  digitalWrite(_csPin, HIGH);
}

void MX25Logger::enter4ByteMode() {
  digitalWrite(_csPin, LOW); _spi->transfer(CMD_EN4B); digitalWrite(_csPin, HIGH);
}

// Typed log methods
void MX25Logger::logImu(const Raw_imu &raw) {
  imu_pkt pkt;
  pkt.header.SYNC_BYTE = 0xAA;
  pkt.header.id        = ID_IMU;
  pkt.header.len       = sizeof(pkt);
  pkt.t  = raw.timestamp;
  pkt.gx = raw.gx; pkt.gy = raw.gy; pkt.gz = raw.gz;
  pkt.ax = raw.ax; pkt.ay = raw.ay; pkt.az = raw.az;
  _push(&pkt, sizeof(pkt));
}

void MX25Logger::logBaro(const Raw_press &p) {
  baro_pkt pkt;
  pkt.header.SYNC_BYTE = 0xAA;
  pkt.header.id        = ID_BARO;
  pkt.header.len       = sizeof(pkt);
  pkt.t   = p.timestamp;
  pkt.alt = p.alt;
  _push(&pkt, sizeof(pkt));
}

void MX25Logger::logMag(const Raw_mag &m) {
  mag_pkt pkt;
  pkt.header.SYNC_BYTE = 0xAA;
  pkt.header.id        = ID_MAG;
  pkt.header.len       = sizeof(pkt);
  pkt.t  = m.timestamp;
  pkt.mx = m.mx; pkt.my = m.my; pkt.mz = m.mz;
  _push(&pkt, sizeof(pkt));
}

void MX25Logger::logGps(const Raw_gps &g) {
  gps_pkt pkt;
  pkt.header.SYNC_BYTE = 0xAA;
  pkt.header.id        = ID_GPS;
  pkt.header.len       = sizeof(pkt);
  pkt.t  = g.timestamp;
  pkt.pn = g.pn; pkt.pe = g.pe; pkt.pd = g.pd;
  pkt.vn = g.vn; pkt.ve = g.ve; pkt.vd = g.vd;
  pkt.hAcc = g.hAcc; pkt.vAcc = g.vAcc; pkt.sAcc = g.sAcc;
  pkt.fixType = g.fixType;
  pkt.numSV   = g.numSV;
  _push(&pkt, sizeof(pkt));
}

void MX25Logger::logState(const State_nominal &nom) {
  state_pkt pkt;
  pkt.header.SYNC_BYTE = 0xAA;
  pkt.header.id        = ID_STATE;
  pkt.header.len       = sizeof(pkt);
  pkt.t = nom.timestamp;
  memcpy(pkt.p,  nom.p,  sizeof(nom.p));
  memcpy(pkt.v,  nom.v,  sizeof(nom.v));
  memcpy(pkt.q,  nom.q,  sizeof(nom.q));
  memcpy(pkt.ba, nom.ba, sizeof(nom.ba));
  memcpy(pkt.bg, nom.bg, sizeof(nom.bg));
  _push(&pkt, sizeof(pkt));
}

void MX25Logger::logEvent(FlightPhase phase, uint8_t eventId) {
  event_pkt pkt;
  pkt.header.SYNC_BYTE = 0xAA;
  pkt.header.id        = ID_EVENT;
  pkt.header.len       = sizeof(pkt);
  pkt.t = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
  pkt.phase    = (uint8_t)phase;
  pkt.event_id = eventId;
  _push(&pkt, sizeof(pkt));
}

void MX25Logger::logGnc(float nz_cmd, float ny_cmd, const float fin_cmd[4], uint8_t phase) {
  gnc_pkt pkt;
  pkt.header.SYNC_BYTE = 0xAA;
  pkt.header.id        = ID_GNC;
  pkt.header.len       = sizeof(pkt);
  pkt.t      = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFF);
  pkt.nz_cmd = nz_cmd;
  pkt.ny_cmd = ny_cmd;
  memcpy(pkt.fin_cmd, fin_cmd, sizeof(pkt.fin_cmd));
  pkt.phase  = phase;
  _push(&pkt, sizeof(pkt));
}

// ----------------------------------------------------------------------------
// serviceFlush() : FlushTask 본체 한 사이클
//   - 큐에서 10ms 대기하며 패킷 1개 꺼내 _dataBuffer에 append
//   - 256B 채워졌으면 flushPages() 호출
// ----------------------------------------------------------------------------
void MX25Logger::serviceFlush() {
  Item item;
  if (xQueueReceive(_queue, &item, pdMS_TO_TICKS(10)) == pdTRUE) {
    appendRaw(item.data, item.len);
  }
  if (hasFullPage()) {
    flushPages();
  }
}

void MX25Logger::_push(const void *pkt, uint8_t len) {
  if (!_enabled) return;
  if (len > ITEM_MAX_SIZE) return;
  Item item;
  item.len = len;
  memcpy(item.data, pkt, len);
  xQueueSend(_queue, &item, 0);
}
