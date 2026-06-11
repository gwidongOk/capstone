#include "LSM6DSO32.h"

LSM6DSO32::LSM6DSO32(uint8_t csPin, SPIClass* spi) {
    _csPin = csPin;
    _spi = spi;
    _spiSettings = SPISettings(5000000, MSBFIRST, SPI_MODE3);
}

// ----------------------------------------------------------------------------
// begin() : LSM6DSO32 초기 설정
//   - WHO_AM_I 확인 (0x6C) → soft reset → BDU + AutoIncrement 활성
//   - CTRL1_XL = 0x64 : 가속 416Hz, ±32g (로켓용 풀스케일)
//   - CTRL2_G  = 0x6C : 자이로 416Hz, ±2000dps
//   - LPF1 활성 (CTRL1_XL[1]=1), CTRL8_XL bandwidth 100Hz로 고주파 노이즈 차단
// ----------------------------------------------------------------------------
bool LSM6DSO32::begin() {
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);

    uint8_t whoAmI = readRegister(REG_WHO_AM_I);
    if (whoAmI != 0x6C) {
        Serial.printf("LSM6DSO32 ERROR: WHO_AM_I = 0x%02X\n", whoAmI);
        return false;
    }

    writeRegister(REG_CTRL3_C, 0x01);
    delay(10);
    writeRegister(REG_CTRL3_C, 0x44);
    writeRegister(REG_CTRL4_C, 0x04);

    // IMU ODR and full-scale
    writeRegister(REG_CTRL1_XL, 0x64);
    writeRegister(REG_CTRL2_G,  0x6C);

    // IMU ODR and full-scale
    uint8_t ctrl1 = readRegister(REG_CTRL1_XL);
    writeRegister(REG_CTRL1_XL, ctrl1 | 0x02);
    uint8_t ctrl8 = readRegister(REG_CTRL8_XL);
    ctrl8 &= 0x1F;
    ctrl8 |= (0x01 << 5);
    writeRegister(REG_CTRL8_XL, ctrl8);

    uint8_t ctrl4 = readRegister(REG_CTRL4_C);
    writeRegister(REG_CTRL4_C, ctrl4 | 0x02);
    uint8_t ctrl6 = readRegister(REG_CTRL6_C);
    ctrl6 &= 0xF8;
    writeRegister(REG_CTRL6_C, ctrl6);

    return true;
}

// ----------------------------------------------------------------------------
// calibrate() : 정적 영점 보정
//   - nSamples 만큼 평균을 내서 자이로/가속의 int16 바이어스 저장
//   - 가속의 Z축에 대해서는 1g(=1025 LSB at ±32g/0.976mg)를 빼고 저장
//     (보드를 Z-up 정자세로 둔 채 보정한다는 가정)
//   - 가속/자이로 분산이 임계 이상이면 false (보드가 흔들렸다는 뜻)
// ----------------------------------------------------------------------------
bool LSM6DSO32::calibrate(uint16_t nSamples) {
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
    
    double sq_sum_gx = 0, sq_sum_ax = 0;

    for (uint16_t i = 0; i < nSamples; i++) {
        int16_t gx, gy, gz, ax, ay, az;
        readRawIMU(gx, gy, gz, ax, ay, az);
        
        sum_gx += gx; sum_gy += gy; sum_gz += gz;
        sum_ax += ax; sum_ay += ay; sum_az += az;
        
        sq_sum_gx += (double)gx * gx;
        sq_sum_ax += (double)ax * ax;
        
        vTaskDelay(pdMS_TO_TICKS(3));
    }

    _bias_gx = (int16_t)(sum_gx / (int32_t)nSamples);
    _bias_gy = (int16_t)(sum_gy / (int32_t)nSamples);
    _bias_gz = (int16_t)(sum_gz / (int32_t)nSamples);

    const int16_t G_LSB = 1025;

    _bias_ax = (int16_t)(sum_ax / (int32_t)nSamples);                     // X: 0g
    _bias_ay = (int16_t)((sum_ay / (int32_t)nSamples) - G_LSB);
    _bias_az = (int16_t)(sum_az / (int32_t)nSamples);                     // Z: 0g

    double var_gx = (sq_sum_gx / nSamples) - ((double)_bias_gx * _bias_gx);
    double var_ax = (sq_sum_ax / nSamples) - ((double)_bias_ax * _bias_ax);
    
    if (var_gx > 400.0 || var_ax > 2500.0) {
        Serial.printf("IMU CAL FAIL: noise high (G_Var:%.1f, A_Var:%.1f)\n", var_gx, var_ax);
        return false;
    }

    return true;
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

// ----------------------------------------------------------------------------
// readCalibratedIMU() : 보정 + 축정렬된 IMU
//   1) raw 읽고 int16 바이어스 빼기 (센서 축 기준)
//   2) 센서축 → 로켓 body축 매핑
//      body_x = sensor_y, body_y = sensor_x, body_z = -sensor_z
//      (보드 실장 방향에 따라 결정된 매핑 — PCB 회전 변경 시 수정 필요)
// ----------------------------------------------------------------------------
void LSM6DSO32::readCalibratedIMU(int16_t &gx, int16_t &gy, int16_t &gz,
                                  int16_t &ax, int16_t &ay, int16_t &az) {
    int16_t rgx, rgy, rgz, rax, ray, raz;
    readRawIMU(rgx, rgy, rgz, rax, ray, raz);

    rgx -= _bias_gx; rgy -= _bias_gy; rgz -= _bias_gz;
    rax -= _bias_ax; ray -= _bias_ay; raz -= _bias_az;

    
    gx =  rgy; 
    gy =  rgx; 
    gz = -rgz;

    ax =  ray; 
    ay =  rax; 
    az = -raz;
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
