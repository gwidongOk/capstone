#include "MMC5983MA.h"

MMC5983MA::MMC5983MA(uint8_t csPin, SPIClass* spi) {
    _csPin = csPin;
    _spi   = spi;
    // MMC5983MA: SPI Mode 3 (CPOL=1, CPHA=1), max 10MHz
    _spiSettings = SPISettings(5000000, MSBFIRST, SPI_MODE3);
}

bool MMC5983MA::begin() {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);

    if (!isConnected()) return false;

    softReset();

    // --- Rocket default config ---
    // 800Hz bandwidth for fast attitude update
    setFilterBandwidth(800);
    // Auto SET/RESET for vibration/temperature robustness
    enableAutoSetReset(true);
    // Continuous mode at 1000Hz
    setContinuousFrequency(1000);
    enableContinuousMode(true);
    // Enable data ready interrupt
    enableDataReadyInterrupt(true);

    return true;
}

// ==================== Settings ====================

bool MMC5983MA::setFilterBandwidth(uint16_t bw) {
    // Clear BW bits in shadow first (no write)
    shadowClear(REG_CTRL1, BW0 | BW1, false);

    switch (bw) {
        case 800: shadowSet(REG_CTRL1, BW0 | BW1); break;
        case 400: shadowSet(REG_CTRL1, BW1);        break;
        case 200: shadowSet(REG_CTRL1, BW0);        break;
        case 100: writeRegister(REG_CTRL1, _shadowCtrl1); break; // already cleared
        default:  return false;
    }
    return true;
}

bool MMC5983MA::setContinuousFrequency(uint16_t freq) {
    // Clear CM_FREQ bits (no write)
    shadowClear(REG_CTRL2, CM_FREQ_0 | CM_FREQ_1 | CM_FREQ_2, false);

    uint8_t val = 0;
    switch (freq) {
        case 0:    val = 0; break;
        case 1:    val = 1; break;
        case 10:   val = 2; break;
        case 20:   val = 3; break;
        case 50:   val = 4; break;
        case 100:  val = 5; break;
        case 200:  val = 6; break;
        case 1000: val = 7; break;
        default:   return false;
    }

    shadowSet(REG_CTRL2, val & 0x07);
    return true;
}

void MMC5983MA::enableAutoSetReset(bool enable) {
    if (enable)
        shadowSet(REG_CTRL0, AUTO_SR_EN);
    else
        shadowClear(REG_CTRL0, AUTO_SR_EN);
}

void MMC5983MA::enableContinuousMode(bool enable) {
    if (enable)
        shadowSet(REG_CTRL2, CMM_EN);
    else
        shadowClear(REG_CTRL2, CMM_EN);
}

void MMC5983MA::applyConfig(const MMC5983MA_Config& cfg) {
    setFilterBandwidth(cfg.bandwidth);
    enableAutoSetReset(cfg.autoSetReset);
    setContinuousFrequency(cfg.cmFrequency);
    enableContinuousMode(cfg.continuousMode);
}

MMC5983MA_Config MMC5983MA::getConfig() {
    MMC5983MA_Config cfg;

    // Bandwidth from shadow
    uint8_t bwBits = _shadowCtrl1 & (BW0 | BW1);
    switch (bwBits) {
        case 0x03: cfg.bandwidth = 800; break;
        case 0x02: cfg.bandwidth = 400; break;
        case 0x01: cfg.bandwidth = 200; break;
        default:   cfg.bandwidth = 100; break;
    }

    // Continuous frequency from shadow
    uint8_t cmBits = _shadowCtrl2 & 0x07;
    const uint16_t freqTable[] = {0, 1, 10, 20, 50, 100, 200, 1000};
    cfg.cmFrequency = freqTable[cmBits];

    cfg.autoSetReset  = (_shadowCtrl0 & AUTO_SR_EN) != 0;
    cfg.continuousMode = (_shadowCtrl2 & CMM_EN) != 0;

    return cfg;
}

// ==================== Raw Data ====================

void MMC5983MA::readRawMag(uint32_t &mx, uint32_t &my, uint32_t &mz) {
    uint8_t buf[7];
    readRegisters(REG_X_OUT_0, buf, 7);

    mx = (uint32_t)buf[0] << 10 | (uint32_t)buf[1] << 2 | (buf[6] >> 6);
    my = (uint32_t)buf[2] << 10 | (uint32_t)buf[3] << 2 | ((buf[6] >> 4) & 0x03);
    mz = (uint32_t)buf[4] << 10 | (uint32_t)buf[5] << 2 | ((buf[6] >> 2) & 0x03);
}

// ==================== Interrupt ====================

void MMC5983MA::enableDataReadyInterrupt(bool enable) {
    if (enable)
        shadowSet(REG_CTRL0, INT_MEAS_DONE_EN);
    else
        shadowClear(REG_CTRL0, INT_MEAS_DONE_EN);
}

// ==================== Utility ====================

bool MMC5983MA::isConnected() {
    uint8_t id = readRegister(REG_PROD_ID);
    return (id == PROD_ID_VAL);
}

void MMC5983MA::softReset() {
    shadowSet(REG_CTRL1, SW_RST);
    // Reset clears all registers - reset shadows
    _shadowCtrl0 = 0;
    _shadowCtrl1 = 0;
    _shadowCtrl2 = 0;
    _shadowCtrl3 = 0;
    delay(15);
}

void MMC5983MA::performSetOperation() {
    shadowSet(REG_CTRL0, SET_OPERATION);
    shadowClear(REG_CTRL0, SET_OPERATION, false); // auto-clear in shadow only
    delay(1);
}

void MMC5983MA::performResetOperation() {
    shadowSet(REG_CTRL0, RESET_OPERATION);
    shadowClear(REG_CTRL0, RESET_OPERATION, false); // auto-clear in shadow only
    delay(1);
}

// ==================== Shadow Register Helpers ====================

void MMC5983MA::shadowSet(uint8_t reg, uint8_t mask, bool write) {
    uint8_t* shadow = nullptr;
    switch (reg) {
        case REG_CTRL0: shadow = &_shadowCtrl0; break;
        case REG_CTRL1: shadow = &_shadowCtrl1; break;
        case REG_CTRL2: shadow = &_shadowCtrl2; break;
        case REG_CTRL3: shadow = &_shadowCtrl3; break;
        default: return;
    }
    *shadow |= mask;
    if (write) writeRegister(reg, *shadow);
}

void MMC5983MA::shadowClear(uint8_t reg, uint8_t mask, bool write) {
    uint8_t* shadow = nullptr;
    switch (reg) {
        case REG_CTRL0: shadow = &_shadowCtrl0; break;
        case REG_CTRL1: shadow = &_shadowCtrl1; break;
        case REG_CTRL2: shadow = &_shadowCtrl2; break;
        case REG_CTRL3: shadow = &_shadowCtrl3; break;
        default: return;
    }
    *shadow &= ~mask;
    if (write) writeRegister(reg, *shadow);
}

// ==================== SPI Low-Level ====================

uint8_t MMC5983MA::readRegister(uint8_t reg) {
    uint8_t value;
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    _spi->transfer(reg | 0x80);  // Read: bit7 = 1
    value = _spi->transfer(0x00);
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
    return value;
}

void MMC5983MA::readRegisters(uint8_t reg, uint8_t* buffer, uint8_t len) {
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    _spi->transfer(reg | 0x80);  // Read: bit7 = 1
    for (uint8_t i = 0; i < len; i++) buffer[i] = _spi->transfer(0x00);
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
}

void MMC5983MA::writeRegister(uint8_t reg, uint8_t data) {
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    _spi->transfer(reg & 0x7F);  // Write: bit7 = 0
    _spi->transfer(data);
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
}
