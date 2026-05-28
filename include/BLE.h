#ifndef BLE_H
#define BLE_H

#include <Arduino.h>

void initBLE(const char *deviceName);

String getIncomingRaw();

void sendResponse(const char *msg);

#endif