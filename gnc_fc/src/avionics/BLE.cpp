#include "BLE.h"
#include <NimBLEDevice.h>

static QueueHandle_t bleRxQueue = nullptr;
static NimBLECharacteristic *pTxChar = nullptr;

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *s, NimBLEConnInfo &connInfo) override {
  }
  void onDisconnect(NimBLEServer *s, NimBLEConnInfo &connInfo, int reason) override {
    NimBLEDevice::startAdvertising();
  }
};

class RxCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *pChar, NimBLEConnInfo &connInfo) override {
    std::string val = pChar->getValue();
    if (val.empty() || !bleRxQueue) return;

    char buf[32] = {0};
    size_t len = val.size();
    if (len > 31) len = 31;
    memcpy(buf, val.c_str(), len);
    xQueueSend(bleRxQueue, buf, pdMS_TO_TICKS(10));
  }
};

// ----------------------------------------------------------------------------
// initBLE() : Nordic UART Service (NUS) 기반 BLE 셋업
//   - TX char (NOTIFY) : MCU → 클라이언트 응답
//   - RX char (WRITE)  : 클라이언트 → MCU 명령 (큐로 적재)
//   - 광고 간격 20~40ms로 짧게 → 폰에서 빨리 잡힘
//   - 전력 최대(P9)로 설정 (로켓 거리 확보)
// ----------------------------------------------------------------------------
void initBLE(const char *deviceName) {
  bleRxQueue = xQueueCreate(4, 32);

  NimBLEDevice::init(deviceName);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setMTU(128);

  NimBLEServer *pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // Nordic UART Service (NUS)
  NimBLEService *pService = pServer->createService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");

  pTxChar = pService->createCharacteristic(
    "6E400003-B5A3-F393-E0A9-E50E24DCCA9E",
    NIMBLE_PROPERTY::NOTIFY
  );

  NimBLECharacteristic *pRxChar = pService->createCharacteristic(
    "6E400002-B5A3-F393-E0A9-E50E24DCCA9E",
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  pRxChar->setCallbacks(new RxCallbacks());

  pServer->start();

  NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
  pAdv->reset();

  NimBLEAdvertisementData advData;
  advData.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
  advData.addServiceUUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
  pAdv->setAdvertisementData(advData);

  NimBLEAdvertisementData scanResp;
  scanResp.setName(deviceName);
  pAdv->setScanResponseData(scanResp);

  pAdv->enableScanResponse(true);
  pAdv->setMinInterval(0x20);
  pAdv->setMaxInterval(0x40);         // 40ms

  bool ok = pAdv->start(0);
  Serial.printf("BLE advertising %s (%s)\n", ok ? "started" : "FAILED", deviceName);
}

// ----------------------------------------------------------------------------
// getIncomingRaw() : USB Serial과 BLE RX 큐를 둘 다 확인 → 명령 1줄 반환
//   - USB가 연결돼 있으면 그쪽이 우선
// ----------------------------------------------------------------------------
String getIncomingRaw() {
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    return s;
  }

  if (bleRxQueue) {
    char buf[32];
    if (xQueueReceive(bleRxQueue, buf, 0) == pdTRUE) {
      String s = String(buf);
      s.trim();
      return s;
    }
  }

  return "";
}

void sendResponse(const char *msg) {
  Serial.print(msg);
  if (pTxChar) {
    pTxChar->setValue((const uint8_t *)msg, strlen(msg));
    pTxChar->notify();
  }
}
