#ifndef MMC5983MA_H
#define MMC5983MA_H

#include <Arduino.h>
#include <SPI.h>

struct MMC5983MA_Config {
    uint16_t bandwidth;       // 100, 200, 400, 800 (Hz)
    uint16_t cmFrequency;     // Continuous mode: 0(off), 1, 10, 20, 50, 100, 200, 1000 (Hz)
    bool     autoSetReset;    // Auto SET/RESET enable
    bool     continuousMode;  // Continuous measurement enable
};

class MMC5983MA {
private:
    uint8_t    _csPin;
    SPIClass*  _spi;
    SPISettings _spiSettings;

    // --- Register addresses ---
    static const uint8_t REG_X_OUT_0    = 0x00;
    static const uint8_t REG_X_OUT_1    = 0x01;
    static const uint8_t REG_Y_OUT_0    = 0x02;
    static const uint8_t REG_Y_OUT_1    = 0x03;
    static const uint8_t REG_Z_OUT_0    = 0x04;
    static const uint8_t REG_Z_OUT_1    = 0x05;
    static const uint8_t REG_XYZ_OUT_2  = 0x06;
    static const uint8_t REG_T_OUT      = 0x07;
    static const uint8_t REG_STATUS     = 0x08;
    static const uint8_t REG_CTRL0      = 0x09;
    static const uint8_t REG_CTRL1      = 0x0A;
    static const uint8_t REG_CTRL2      = 0x0B;
    static const uint8_t REG_CTRL3      = 0x0C;
    static const uint8_t REG_PROD_ID    = 0x2F;

    // --- Control 0 bits ---
    static const uint8_t TM_M              = 0x01;
    static const uint8_t TM_T              = 0x02;
    static const uint8_t INT_MEAS_DONE_EN  = 0x04;
    static const uint8_t SET_OPERATION     = 0x08;
    static const uint8_t RESET_OPERATION   = 0x10;
    static const uint8_t AUTO_SR_EN        = 0x20;

    // --- Control 1 bits ---
    static const uint8_t BW0       = 0x01;
    static const uint8_t BW1       = 0x02;
    static const uint8_t X_INHIBIT = 0x04;
    static const uint8_t YZ_INHIBIT= 0x08;
    static const uint8_t SW_RST    = 0x10;

    // --- Control 2 bits ---
    static const uint8_t CM_FREQ_0 = 0x01;
    static const uint8_t CM_FREQ_1 = 0x02;
    static const uint8_t CM_FREQ_2 = 0x04;
    static const uint8_t CMM_EN    = 0x08;
    static const uint8_t PRD_SET_0 = 0x10;
    static const uint8_t PRD_SET_1 = 0x20;
    static const uint8_t PRD_SET_2 = 0x40;
    static const uint8_t EN_PRD_SET= 0x80;

    // --- Status bits ---
    static const uint8_t MEAS_M_DONE = 0x01;
    static const uint8_t MEAS_T_DONE = 0x02;

    static const uint8_t PROD_ID_VAL = 0x30;

    // Shadow registers (write-only registers need shadow tracking)
    uint8_t _shadowCtrl0 = 0x00;
    uint8_t _shadowCtrl1 = 0x00;
    uint8_t _shadowCtrl2 = 0x00;
    uint8_t _shadowCtrl3 = 0x00;

    // SPI low-level
    uint8_t readRegister(uint8_t reg);
    void    readRegisters(uint8_t reg, uint8_t* buffer, uint8_t len);
    void    writeRegister(uint8_t reg, uint8_t data);

    // Shadow register helpers
    void shadowSet(uint8_t reg, uint8_t mask, bool write = true);
    void shadowClear(uint8_t reg, uint8_t mask, bool write = true);

public:
    MMC5983MA(uint8_t csPin, SPIClass* spi = &SPI);

    // Initialize with default rocket config
    bool begin();

    // ===== Settings =====
    bool setFilterBandwidth(uint16_t bw);       // 100, 200, 400, 800
    bool setContinuousFrequency(uint16_t freq);  // 0, 1, 10, 20, 50, 100, 200, 1000
    void enableAutoSetReset(bool enable);
    void enableContinuousMode(bool enable);
    void applyConfig(const MMC5983MA_Config& cfg);
    MMC5983MA_Config getConfig();

    // ===== Raw data =====
    // 18-bit raw magnetic field (0 ~ 262143, offset binary: 131072 = 0 Gauss)
    void readRawMag(uint32_t &mx, uint32_t &my, uint32_t &mz);

    // ===== Interrupt =====
    void enableDataReadyInterrupt(bool enable);

    // ===== Utility =====
    bool isConnected();
    void softReset();
    void performSetOperation();
    void performResetOperation();
};

#endif
