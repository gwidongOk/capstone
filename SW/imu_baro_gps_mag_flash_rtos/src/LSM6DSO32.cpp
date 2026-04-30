#include "LSM6DSO32.h"

LSM6DSO32::LSM6DSO32(uint8_t csPin, SPIClass* spi) {
    _csPin = csPin;
    _spi = spi;
    _spiSettings = SPISettings(5000000, MSBFIRST, SPI_MODE3);
}

bool LSM6DSO32::begin() {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);

    uint8_t whoAmI = readRegister(REG_WHO_AM_I);
    if (whoAmI != 0x6C) {
        Serial.printf("LSM6DSO32 에러! WHO_AM_I = 0x%02X\n", whoAmI);
        return false;
    }

    writeRegister(REG_CTRL3_C, 0x01);
    delay(10);
    writeRegister(REG_CTRL3_C, 0x44);
    writeRegister(REG_CTRL4_C, 0x04);

    // 416Hz ODR, ±32g, ±2000dps
    writeRegister(REG_CTRL1_XL, 0x64);
    writeRegister(REG_CTRL2_G,  0x6C);

    // Accel LPF2 on, HPCF_XL = 001 → cutoff = ODR/10 = 41.6 Hz
    uint8_t ctrl1 = readRegister(REG_CTRL1_XL);
    writeRegister(REG_CTRL1_XL, ctrl1 | 0x02);
    uint8_t ctrl8 = readRegister(REG_CTRL8_XL);
    ctrl8 &= 0x1F;
    ctrl8 |= (0x01 << 5);
    writeRegister(REG_CTRL8_XL, ctrl8);

    // Gyro LPF1 on, FTYPE = 000 → cutoff = 136.6 Hz
    uint8_t ctrl4 = readRegister(REG_CTRL4_C);
    writeRegister(REG_CTRL4_C, ctrl4 | 0x02);
    uint8_t ctrl6 = readRegister(REG_CTRL6_C);
    ctrl6 &= 0xF8;
    writeRegister(REG_CTRL6_C, ctrl6);

    return true;
}

void LSM6DSO32::calibrate(uint16_t nSamples) {
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;

    for (uint16_t i = 0; i < nSamples; i++) {
        int16_t gx, gy, gz, ax, ay, az;
        readRawIMU(gx, gy, gz, ax, ay, az);
        sum_gx += gx; sum_gy += gy; sum_gz += gz;
        sum_ax += ax; sum_ay += ay; sum_az += az;
        vTaskDelay(pdMS_TO_TICKS(3));   // 416Hz = 2.4ms, 3ms 여유
    }

    _bias_gx = (int16_t)(sum_gx / (int32_t)nSamples);
    _bias_gy = (int16_t)(sum_gy / (int32_t)nSamples);
    _bias_gz = (int16_t)(sum_gz / (int32_t)nSamples);
    _bias_ax = (int16_t)(sum_ax / (int32_t)nSamples);
    _bias_ay = (int16_t)(sum_ay / (int32_t)nSamples);
    _bias_az = (int16_t)(sum_az / (int32_t)nSamples);
}

void LSM6DSO32::enableAccelDataReadyInterrupt(uint8_t intPin) {
    uint8_t reg = (intPin == 1) ? REG_INT1_CTRL : REG_INT2_CTRL;
    writeRegister(reg, readRegister(reg) | 0x01);
}

void LSM6DSO32::enableGyroDataReadyInterrupt(uint8_t intPin) {
    uint8_t reg = (intPin == 1) ? REG_INT1_CTRL : REG_INT2_CTRL;
    writeRegister(reg, readRegister(reg) | 0x02);
}

void LSM6DSO32::readRawIMU(int16_t &gx, int16_t &gy, int16_t &gz,
                           int16_t &ax, int16_t &ay, int16_t &az) {
    uint8_t buffer[12];
    readRegisters(REG_OUTX_L_G, buffer, 12);
    gx = (int16_t)((buffer[1]  << 8) | buffer[0]);
    gy = (int16_t)((buffer[3]  << 8) | buffer[2]);
    gz = (int16_t)((buffer[5]  << 8) | buffer[4]);
    ax = (int16_t)((buffer[7]  << 8) | buffer[6]);
    ay = (int16_t)((buffer[9]  << 8) | buffer[8]);
    az = (int16_t)((buffer[11] << 8) | buffer[10]);
}

void LSM6DSO32::readCalibratedIMU(int16_t &gx, int16_t &gy, int16_t &gz,
                                  int16_t &ax, int16_t &ay, int16_t &az) {
    int16_t rgx, rgy, rgz, rax, ray, raz;
    readRawIMU(rgx, rgy, rgz, rax, ray, raz);

    // 1. Subtract bias in RAW sensor axis (captured at horizontal rest)
    rgx -= _bias_gx; rgy -= _bias_gy; rgz -= _bias_gz;
    rax -= _bias_ax; ray -= _bias_ay; raz -= _bias_az;

    // 2. Align to Rocket Body-Axis (align_axis logic)
    // Body X = Sensor Y (Nosecone)
    // Body Y = Sensor X (Right)
    // Body Z = -Sensor Z (Down, Right-hand rule)
    gx =  rgy; 
    gy =  rgx; 
    gz = -rgz;

    // 3. Add 1g to Body X-axis as requested (LSB for +/-32g is ~0.976mg)
    // 1g in LSB = 1000mg / 0.976mg/LSB = 1024.59...
    const int16_t G_LSB = 1025; 

    ax =  ray + G_LSB; // Body X (Nosecone)
    ay =  rax;         // Body Y (Right)
    az = -raz;         // Body Z (Down)
}

uint8_t LSM6DSO32::readRegister(uint8_t reg) {
    uint8_t value;
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    _spi->transfer(reg | 0x80);
    value = _spi->transfer(0x00);
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
    return value;
}

void LSM6DSO32::readRegisters(uint8_t reg, uint8_t* buffer, uint8_t len) {
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    _spi->transfer(reg | 0x80);
    for (uint8_t i = 0; i < len; i++) buffer[i] = _spi->transfer(0x00);
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
}

void LSM6DSO32::writeRegister(uint8_t reg, uint8_t data) {
    _spi->beginTransaction(_spiSettings);
    digitalWrite(_csPin, LOW);
    _spi->transfer(reg & 0x7F);
    _spi->transfer(data);
    digitalWrite(_csPin, HIGH);
    _spi->endTransaction();
}
