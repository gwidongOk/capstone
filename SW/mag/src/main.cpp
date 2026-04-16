#include <Arduino.h>
#include "MMC5983MA.h"

// --- Pin definitions ---
static const uint8_t MAG_CS_PIN  = 10;
static const uint8_t MAG_INT_PIN = 44;   // Data ready interrupt pin

MMC5983MA mag(MAG_CS_PIN, &SPI);

volatile bool magDataReady = false;

void IRAM_ATTR onMagDataReady() {
    magDataReady = true;
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    SPI.begin();

    if (!mag.begin()) {
        Serial.println("MMC5983MA init failed!");
        while (1) delay(1000);
    }
    Serial.println("MMC5983MA OK");

    // Attach interrupt (DRDY is active high)
    pinMode(MAG_INT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(MAG_INT_PIN), onMagDataReady, RISING);

    // (Optional) Change settings at runtime:
    // mag.setFilterBandwidth(400);
    // mag.setContinuousFrequency(200);
    // mag.enableAutoSetReset(false);
    //
    // Or apply a full config:
    // MMC5983MA_Config cfg = {400, 200, true, true};
    // mag.applyConfig(cfg);
}

void loop() {
    if (magDataReady) {
        magDataReady = false;

        uint32_t rawX, rawY, rawZ;
        mag.readRawMag(rawX, rawY, rawZ);

        // 18-bit offset binary -> signed Gauss
        // 131072 = zero field, sensitivity = 16384 counts/Gauss (8 Gauss range)
        float mx = ((float)rawX - 131072.0f) / 16384.0f;
        float my = ((float)rawY - 131072.0f) / 16384.0f;
        float mz = ((float)rawZ - 131072.0f) / 16384.0f;

        // TODO: Feed mx, my, mz into ES-EKF attitude update

        Serial.printf("Mag: %.4f  %.4f  %.4f (Gauss)\n", mx, my, mz);
    }
}
