#include "step_cadence.h"
#include "config.h"
#include "Arduino.h"

#if defined(PIN_LSM6DS3TR_C_INT1)
  static constexpr int INT1_PIN = PIN_LSM6DS3TR_C_INT1;
#elif defined(PIN_LSM6DS3_INT1)
  static constexpr int INT1_PIN = PIN_LSM6DS3_INT1;
#else
  static constexpr int INT1_PIN = 3;
#endif

volatile bool g_stepFlag   = false;
volatile bool g_sampleFlag = false;
LSM6DS3       xIMU(I2C_MODE, 0x6A);
bool          imuReady     = false;
float         cadenceSpm   = 0.0f;

static uint16_t stepHistory[10] = {0};
static uint8_t  stepHistHead    = 0;

static void stepISR() { g_stepFlag = true; }

extern "C" void TIMER4_IRQHandler() {
  if (NRF_TIMER4->EVENTS_COMPARE[0]) {
    NRF_TIMER4->EVENTS_COMPARE[0] = 0;
    g_sampleFlag = true;
  }
}

void setupSampleTimer() {
  NRF_TIMER4->TASKS_STOP  = 1;
  NRF_TIMER4->MODE        = TIMER_MODE_MODE_Timer;
  NRF_TIMER4->BITMODE     = TIMER_BITMODE_BITMODE_32Bit;
  NRF_TIMER4->PRESCALER   = 4;
  NRF_TIMER4->CC[0]       = 1000000UL / static_cast<uint32_t>(SAMPLE_RATE_HZ);
  NRF_TIMER4->SHORTS      = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
  NRF_TIMER4->INTENSET    = TIMER_INTENSET_COMPARE0_Msk;
  NVIC_SetPriority(TIMER4_IRQn, 3);
  NVIC_EnableIRQ(TIMER4_IRQn);
  NRF_TIMER4->TASKS_START = 1;
}

static int configPedometer() {
  uint8_t err = 0;
  uint8_t v   = 0;

  err += xIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL,
                             static_cast<uint8_t>(LSM6DS3_ACC_GYRO_ODR_XL_52Hz)
                             | static_cast<uint8_t>(LSM6DS3_ACC_GYRO_FS_XL_2g));

  if (xIMU.readRegister(&v, LSM6DS3_ACC_GYRO_CTRL3_C) == IMU_SUCCESS) {
    v |= static_cast<uint8_t>(LSM6DS3_ACC_GYRO_BDU_BLOCK_UPDATE);
    err += xIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL3_C, v);
  } else { err++; }

  if (xIMU.readRegister(&v, LSM6DS3_ACC_GYRO_CTRL10_C) == IMU_SUCCESS) {
    v |= static_cast<uint8_t>(LSM6DS3_ACC_GYRO_FUNC_EN_ENABLED);
    v |= 0x10u;
    v |= 0x20u;
    err += xIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL10_C, v);
  } else { err++; }

  if (xIMU.readRegister(&v, LSM6DS3_ACC_GYRO_TAP_CFG1) == IMU_SUCCESS) {
    v |= static_cast<uint8_t>(LSM6DS3_ACC_GYRO_PEDO_EN_ENABLED);
    v |= static_cast<uint8_t>(LSM6DS3_ACC_GYRO_TIMER_EN_ENABLED);
    err += xIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, v);
  } else { err++; }

  err += xIMU.writeRegister(LSM6DS3_ACC_GYRO_INT1_CTRL,
                             static_cast<uint8_t>(LSM6DS3_ACC_GYRO_INT1_PEDO_ENABLED));

  xIMU.writeRegister(LSM6DS3_ACC_GYRO_RAM_ACCESS, 0x80u);
  delayMicroseconds(20);
  xIMU.writeRegister(LSM6DS3_ACC_GYRO_CONFIG_PEDO_THS_MIN, 0x06u);
  delayMicroseconds(20);
  xIMU.writeRegister(LSM6DS3_ACC_GYRO_RAM_ACCESS, 0x00u);
  delayMicroseconds(20);

  return err;
}

static uint16_t readStepCount() {
  uint8_t lo = 0, hi = 0;
  if (xIMU.readRegister(&lo, LSM6DS3_ACC_GYRO_STEP_COUNTER_L) != IMU_SUCCESS) return 0;
  if (xIMU.readRegister(&hi, LSM6DS3_ACC_GYRO_STEP_COUNTER_H) != IMU_SUCCESS) return 0;
  return static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8);
}

bool initIMU() {
  if (xIMU.begin() != 0) {
    Serial.println("IMU init failed.");
    imuReady = false;
    return false;
  }
  Serial.println("IMU OK.");
  delay(10);
  const int pedoErr = configPedometer();
  if (pedoErr != 0) {
    Serial.print("Pedometer config errors: ");
    Serial.println(pedoErr);
  } else {
    Serial.println("Pedometer ON @ 52 Hz.");
  }
  imuReady = true;
  pinMode(INT1_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INT1_PIN), stepISR, RISING);
  return true;
}

uint16_t pollSteps() {
  if (g_stepFlag) {
    noInterrupts();
    g_stepFlag = false;
    interrupts();
  }
  return readStepCount();
}

void readAccel(float& ax, float& ay, float& az) {
  ax = xIMU.readFloatAccelX();
  ay = xIMU.readFloatAccelY();
  az = xIMU.readFloatAccelZ();
}

void updateCadence(uint16_t steps) {
  stepHistory[stepHistHead] = steps;
  stepHistHead = (stepHistHead + 1) % 10;
  const uint16_t oldSteps = stepHistory[stepHistHead];
  cadenceSpm = (steps >= oldSteps) ? (steps - oldSteps) * 6.0f : 0.0f;
}
