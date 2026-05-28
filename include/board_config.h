#ifndef CONFIG_H
#define CONFIG_H

// =============================================================================
// 2026 ALTIS AVIONICS V1.1 하드웨어 핀맵 및 시스템 파라미터
// =============================================================================

// -----------------------------------------------------------------------------
// [1] 센서 SPI 핀 (IMU + 기압계 공용 버스)
// -----------------------------------------------------------------------------
#define SPI_SCK_PIN   13   // 센서 SPI 클럭 (HSPI SCK)
#define SPI_MISO_PIN  12   // 센서 SPI 데이터-입력 (MCU ← 센서)
#define SPI_MOSI_PIN  11   // 센서 SPI 데이터-출력 (MCU → 센서)
#define IMU_CS_PIN    14   // LSM6DSO32 IMU 칩셀렉트
#define BMP_CS_PIN    10   // BMP388 기압계 칩셀렉트
#define IMU_INT1_PIN  47   // IMU 데이터-레디 인터럽트 (가속/자이로 416Hz 트리거)
#define BMP_INT_PIN   9    // BMP388 데이터-레디 인터럽트 (50Hz 트리거)

// -----------------------------------------------------------------------------
// [2] 외장 플래시 SPI 핀 (MX25L25645G, 32MB, 비행 로그 저장)
// -----------------------------------------------------------------------------
#define FLASH_SCK_PIN   16
#define FLASH_MISO_PIN  7
#define FLASH_MOSI_PIN  15
#define FLASH_CS_PIN    6

// -----------------------------------------------------------------------------
// [3] 자력계 I2C 핀 (MMC5983MA)
// -----------------------------------------------------------------------------
#define MAG_SDA_PIN  5     // 자력계 I2C 데이터 라인
#define MAG_SCL_PIN  4     // 자력계 I2C 클럭 라인 (400kHz Fast mode)

// -----------------------------------------------------------------------------
// [4] GPS UART 핀 (NEO-M9N, Serial1)
// -----------------------------------------------------------------------------
#define GPS_RX_PIN   2     // MCU 수신 ← GPS TX
#define GPS_TX_PIN   1     // MCU 송신 → GPS RX

// -----------------------------------------------------------------------------
// [5] 사용자 피드백 핀 (부저 + LED)
// -----------------------------------------------------------------------------
#define BUZZER_PIN   17    // 부저 (단계별 비프 알림)
#define LED_PIN      48    // 사용자 LED (보정/정렬/비행 중 상태 표시)

// -----------------------------------------------------------------------------
// [6] 서보 출력 핀 (TVC/공력면 제어용 4채널)
// -----------------------------------------------------------------------------
#define SERVO_1_PIN   41
#define SERVO_2_PIN   40
#define SERVO_3_PIN   39
#define SERVO_4_PIN   38

// -----------------------------------------------------------------------------
// [7] 화이로 (Pyrotechnic) 점화 출력 핀
// -----------------------------------------------------------------------------
#define PYRO_1_PIN   18    // Pyro1: Drogue
#define PYRO_2_PIN   8     // Pyro2: Main

// -----------------------------------------------------------------------------
// [8] 통신 설정
// -----------------------------------------------------------------------------
#define SERIAL_BAUD       921600        // USB-Serial 디버그/로그덤프 baudrate
#define BLE_DEVICE_NAME   "2026ALTIS"   // BLE 광고 이름 (Nordic UART Service)

// =============================================================================
// RTOS 태스크 우선순위 / 스택 크기
//  - ESP32-S3 듀얼코어 가정. Core 1 = 고속 실시간, Core 0 = 보조/시스템
//  - FreeRTOS 우선순위: 숫자가 클수록 높음 (5가 최고)
// =============================================================================

// Core 1 (고속 실시간 — 통합 flight_task)
#define TASK_C1_PRIO_FLIGHT  5     // 통합 400Hz flight_task (최우선)
#define STACK_SIZE_FLIGHT    8192  // flight_task 스택

// Core 1 (센서별 태스크 — avionics 아키텍처, 참조용)
#define TASK_C1_PRIO_IMU     5     // IMU 416Hz 데이터-레디 인터럽트 처리

// Core 0 (보조 센서 + 시스템 작업)
#define TASK_C0_PRIO_BMP     4     // 기압계 50Hz 인터럽트 처리
#define TASK_C0_PRIO_MAG     4     // 자력계 100Hz 폴링 처리
#define TASK_C0_PRIO_GPS     3     // GPS UART UBX 파싱 (25Hz)
#define TASK_C0_PRIO_FLUSH   2     // 플래시 페이지 라이트 (가장 낮음, 센서가 우선)

#define STACK_SIZE_SENSOR    8192  // 센서 태스크 스택 (Eigen 행렬 연산 여유분 포함)
#define STACK_SIZE_GPS       4096  // GPS 태스크 스택
#define STACK_SIZE_FLUSH     4096  // 플러시 태스크 스택 (단순 SPI 전송 위주라 작음)

// =============================================================================
// 센서 폴링 / 보정 파라미터
// =============================================================================
#define MAG_POLL_MS       10       // 자력계 데이터-레디 폴링 주기 [ms] (~100Hz)
#define GPS_POLL_MS       20       // GPS UART 폴링 주기 [ms]
#define GPS_RATE_HZ       25       // GPS 출력 레이트 [Hz] (NEO-M9N NAV-PVT)
#define CALIB_SAMPLES     100      // IMU/기압계 정적 캘리브레이션 샘플 수
#define MAG_CALIB_MS      30000    // 자력계 8자 회전 캘리브레이션 시간 [ms] (30초)

// =============================================================================
// EKF Start 시점 정렬 (TRIAD + ZUPT 수렴 판정)
// =============================================================================
#define START_TRIAD_AVG_MS       1000   // START 후 평균을 모을 시간 [ms]
#define START_TRIAD_MIN_IMU      100    // TRIAD 평균에 필요한 최소 IMU 샘플 수
#define START_TRIAD_MIN_MAG      5      // TRIAD 평균에 필요한 최소 MAG 샘플 수
#define ZUPT_MIN_ITER            10     // ZUPT 수렴 판정 시작 전 최소 반복 횟수
#define ZUPT_MAX_ITER            200    // ZUPT 최대 반복 (타임아웃 보호)
#define ZUPT_PERIOD_MS           100    // ZUPT 한 사이클 주기 [ms]
#define NAV_MUTEX_FAST_WAIT_MS   5
#define NAV_MUTEX_LONG_WAIT_MS   5000
#define ZUPT_MUTEX_WAIT_MS       50     // navMutex 획득 최대 대기 [ms]
#define ZUPT_REL_THRESH          (1.0e-3f)  // P-trace 상대변화 임계 (0.1%)
#define ZUPT_STABLE_REQ          3      // 연속 ZUPT_REL_THRESH 만족 횟수 → 수렴 인정

// =============================================================================
// ES-EKF 노이즈 모델 & Adaptive-Q 파라미터
//  - VAR_* : 분산(variance, sigma^2). datasheet 노이즈 밀도와 정적 측정값 중 큰 쪽 선택
//  - Q_SCALE : 프로세스 노이즈 부풀리기 계수 (필터 신뢰도 조정용)
// =============================================================================
#define EKF_G_VAL                9.80665f      // 표준 중력가속도 [m/s^2]
#define EKF_IMU_RATE_HZ          416.0f        // IMU ODR [Hz] (LSM6DSO32 416Hz 모드)

// IMU 정적 분산 (벤치에서 직접 측정한 값, RMS^2)
#define EKF_VAR_ACC_STATIC       2.729445e-4f  // 가속도계 정적 분산 [(m/s^2)^2]
#define EKF_VAR_GYRO_STATIC      8.061781e-7f  // 자이로 정적 분산 [(rad/s)^2]

// 데이터시트 노이즈 밀도 → 분산 환산
#define EKF_ACC_NOISE_DENSITY    220.0e-6f     // 가속도계 노이즈 밀도 [g/√Hz]
#define EKF_GYRO_NOISE_DENSITY   3.8e-3f       // 자이로 노이즈 밀도 [dps/√Hz]

// 프로세스 노이즈 부풀리기 (실제 필터 Q에 곱해짐)
#define EKF_Q_ACC_SCALE          1000.0f       // 가속 잡음 스케일 (모델 불확실성 반영, 크게)
#define EKF_Q_GYRO_SCALE         10.0f         // 자이로 잡음 스케일
#define EKF_VAR_BA               1.0e-6f       // 가속도 바이어스 랜덤워크 분산 (가벼운 발산 허용)
#define EKF_VAR_BG               1.0e-8f       // 자이로 바이어스 랜덤워크 분산

// 측정 잡음 분산 (R 행렬 대각)
#define EKF_VAR_BARO_DS          (0.11f * 0.11f)
#define EKF_VAR_BARO_STATIC      4.920342e-2f
#define EKF_VAR_BARO             ((EKF_VAR_BARO_DS > EKF_VAR_BARO_STATIC) ? EKF_VAR_BARO_DS : EKF_VAR_BARO_STATIC)

#define EKF_VAR_GPS_POS_H_DS     ((2.0f / 1.1774f) * (2.0f / 1.1774f))
#define EKF_VAR_GPS_POS_V_DS     ((4.0f / 1.1774f) * (4.0f / 1.1774f))
#define EKF_VAR_GPS_POS_H_STATIC (0.694f * 0.694f)
#define EKF_VAR_GPS_POS_V_STATIC (1.282927f * 1.282927f)
#define EKF_VAR_GPS_POS_H        ((EKF_VAR_GPS_POS_H_DS > EKF_VAR_GPS_POS_H_STATIC) ? EKF_VAR_GPS_POS_H_DS : EKF_VAR_GPS_POS_H_STATIC)
#define EKF_VAR_GPS_POS_V        ((EKF_VAR_GPS_POS_V_DS > EKF_VAR_GPS_POS_V_STATIC) ? EKF_VAR_GPS_POS_V_DS : EKF_VAR_GPS_POS_V_STATIC)

#define EKF_VAR_GPS_VEL_H_DS     (0.05f * 0.05f)
#define EKF_VAR_GPS_VEL_V_DS     (0.05f * 0.05f)
#define EKF_VAR_GPS_VEL_H_STATIC (0.2684742f * 0.2684742f)
#define EKF_VAR_GPS_VEL_V_STATIC (0.2684742f * 0.2684742f)
#define EKF_VAR_GPS_VEL_H        ((EKF_VAR_GPS_VEL_H_DS > EKF_VAR_GPS_VEL_H_STATIC) ? EKF_VAR_GPS_VEL_H_DS : EKF_VAR_GPS_VEL_H_STATIC)
#define EKF_VAR_GPS_VEL_V        ((EKF_VAR_GPS_VEL_V_DS > EKF_VAR_GPS_VEL_V_STATIC) ? EKF_VAR_GPS_VEL_V_DS : EKF_VAR_GPS_VEL_V_STATIC)

#define EKF_VAR_MAG_DS           ((0.4f / 500.0f) * (0.4f / 500.0f))
#define EKF_VAR_MAG_STATIC       ((1.963f / 500.0f) * (1.963f / 500.0f))
#define EKF_VAR_MAG              ((EKF_VAR_MAG_DS > EKF_VAR_MAG_STATIC) ? EKF_VAR_MAG_DS : EKF_VAR_MAG_STATIC)

// GPS 위치 R 인플레이션 (수신기 hAcc/vAcc를 그대로 믿지 않고 곱해줌)
#define EKF_GPS_POS_INFLATION    3.0f          // GPS hAcc/vAcc 곱셈 보정 (실측 기반)

// Adaptive-Q (저크 기반 잡음 부풀리기): 기동 중에는 Q를 키워 잔차를 빨리 추적
#define EKF_JERK_ALPHA           0.02f         // 저크 EMA 필터의 시정수 계수 (작을수록 느림)
#define EKF_JERK_THRESH          120.0f        // 가속도 저크 임계 [m/s^3] (초과 시 Q 부풀리기)
#define EKF_ANG_ACC_THRESH       8.0f          // 각가속도 임계 [rad/s^2]
#define EKF_JERK_SCALE_MAX       12.0f         // Q 부풀리기 상한 (폭주 방지)

#define STATE_LOG_HZ             100
#define STATE_LOG_PERIOD_MS      (1000 / STATE_LOG_HZ)
#define CALIB_MAX_ATTEMPTS       5

// =============================================================================
// 비행 이벤트 검출 파라미터 (State Machine)
//  PRE_FLIGHT → POWERED_FLIGHT → COASTING → DESCENT → LANDED
// =============================================================================
#define FLIGHT_TASK_PERIOD_MS        100     // 비행 상태 검출 주기 [ms] (10Hz)
#define FLIGHT_NAV_MUTEX_WAIT_MS     2       // navMutex 짧은 대기 (실패 시 다음 사이클로)

// [Launch] 발사 검출 — 이 셋 중 하나라도 만족하면 카운트
#define LAUNCH_ALT_M                 5.0f    // 패드 대비 고도 [m]
#define LAUNCH_VEL_UP_MPS            5.0f    // 상승속도 [m/s]
#define LAUNCH_ACCEL_MPS2            20.0f   // 가속도 크기 [m/s^2] (~2g)
#define LAUNCH_CONFIRM_COUNT         2       // 연속 만족 횟수 → 발사 확정

// [Burnout] 모터 연소 종료 검출
#define BURNOUT_HIGH_ACCEL_MPS2      20.0f   // 일단 이 값 넘어야 "고추력" 도달로 판정
#define BURNOUT_LOW_ACCEL_MPS2       10.0f   // 고추력 후 이 값 미만으로 떨어지면 burnout
#define BURNOUT_TIMEOUT_MS           3500    // 최대 연소시간 [ms] (강제 burnout)
#define BURNOUT_VEL_UP_MIN_MPS       10.0f   // 충분히 빠르고
#define BURNOUT_VEL_ACCEL_MAX_MPS2   15.0f   // 가속도가 낮으면 burnout (백업 조건)

// [Apogee] 정점 검출 (Pyro1 점화 트리거)
#define APOGEE_ARM_MS                500     // burnout 후 ARM까지 무시할 시간
#define APOGEE_VEL_UP_MPS           -1.0f    // 상승속도 < 이 값 → 하강 시작
#define APOGEE_MIN_ALT_M             15.0f   // 너무 낮은 고도에서 트리거 방지
#define APOGEE_CONFIRM_COUNT         3       // 연속 만족 → 정점 확정

#define PYRO1_FIRE_MS                1000    // Pyro1 점화 펄스 길이 [ms] (1초)

// [메인 낙하산] 메인 낙하산 사출 (Pyro2 점화 트리거)
#define MAIN_PARACHUTE_ALT_M         100.0f   // 100m 고도
#define MAIN_PARACHUTE_CONFIRM_COUNT 3
#define PYRO2_FIRE_MS                1000    // Pyro2 점화 펄스 길이 [ms] (1초)

// [Landed] 착지 검출
#define LANDED_ALT_M                 10.0f   // 패드 대비 10m 이하
#define LANDED_VEL_UP_ABS_MPS        1.0f    // |vz| 1m/s 미만 → 착지

#endif
