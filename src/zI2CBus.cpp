#include <Configuration.h>
#include <Wire.h>
#include "zI2CBus.h"
#include "zDebugLog.h"

static SemaphoreHandle_t i2cMutex = NULL;
// Lifetime count of lock attempts that timed out (ADC and IMU share the bus).
static uint32_t i2cMutexTimeouts = 0;
uint32_t i2cBusMutexTimeouts() { return i2cMutexTimeouts; }

// Window counters (reset by i2cBusResetWindow() from the health line).
static uint32_t mutexMaxWaitUs = 0;
static uint32_t bnoMaxUs = 0, bnoSumUs = 0, bnoCount = 0;
static uint32_t adcMaxUs = 0, adcSumUs = 0, adcCount = 0;

void i2cBusInit() {
  i2cMutex = xSemaphoreCreateMutex();
  Wire.setBufferSize(128);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setTimeOut(I2C_TRANSACTION_TIMEOUT_MS);
  DEBUG_PRINTF("[I2C] bus init: %lu Hz, timeout %u ms, mutex timeout %u ms\n",
               (unsigned long)I2C_CLOCK_HZ, (unsigned)I2C_TRANSACTION_TIMEOUT_MS,
               (unsigned)I2C_MUTEX_TIMEOUT_MS);
}

bool i2cBusLock() {
  if (i2cMutex == NULL) return false;
  uint32_t t0 = micros();
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) != pdTRUE) {
    i2cMutexTimeouts++;
    return false;
  }
  uint32_t waited = (uint32_t)(micros() - t0);
  if (waited > mutexMaxWaitUs) mutexMaxWaitUs = waited;
  return true;
}

void i2cBusUnlock() {
  if (i2cMutex != NULL) xSemaphoreGive(i2cMutex);
}

// Bounded address probe. Skips the ADS1115 address so the IMU is never
// initialised on the ADC.
uint8_t i2cBusFindBno(const uint8_t *candidates, uint8_t count) {
  uint8_t found = 0;
  for (uint8_t i = 0; i < count; i++) {
    uint8_t addr = candidates[i];
    bool isAdc = (addr == ADS1115_I2C_ADDR);
    if (!i2cBusLock()) {
      DEBUG_PRINTLN("[I2C] scan: mutex timeout");
      continue;
    }
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    i2cBusUnlock();
    if (err == 0) {
      DEBUG_PRINTF("[I2C] device found at 0x%02X%s\n", addr, isAdc ? " (ADS1115 ADC)" : "");
      if (!isAdc && found == 0) found = addr;
    } else {
      DEBUG_PRINTF("[I2C] no device at 0x%02X (err %u)\n", addr, (unsigned)err);
    }
  }
  return found;
}

// Same recovery sequence as the former imuBusRecover(), now under the mutex
// and centralised here. Pins are the ESP32 Wire defaults (SDA=21, SCL=22).
void i2cBusRecover() {
  if (!i2cBusLock()) {
    DEBUG_PRINTLN("[I2C] recover: mutex timeout, skipped");
    return;
  }
  Wire.end();
  pinMode(SCL, OUTPUT);
  pinMode(SDA, INPUT_PULLUP);
  for (uint8_t i = 0; i < 9; i++) {
    digitalWrite(SCL, LOW);
    delayMicroseconds(10);
    digitalWrite(SCL, HIGH);
    delayMicroseconds(10);
  }
  // STOP condition: SDA low -> high while SCL is high
  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, LOW);
  delayMicroseconds(10);
  digitalWrite(SCL, HIGH);
  delayMicroseconds(10);
  digitalWrite(SDA, HIGH);
  delayMicroseconds(10);
  Wire.begin(SDA, SCL);
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setTimeOut(I2C_TRANSACTION_TIMEOUT_MS);
  i2cBusUnlock();
}

// Transaction timing helpers used by the BNO and ADC paths (called under the
// mutex by the owners, so the window counters need no extra locking).
void i2cBusRecordBno(uint32_t us) {
  if (us > bnoMaxUs) bnoMaxUs = us;
  bnoSumUs += us;
  bnoCount++;
}

void i2cBusRecordAdc(uint32_t us) {
  if (us > adcMaxUs) adcMaxUs = us;
  adcSumUs += us;
  adcCount++;
}

uint32_t i2cBusMutexMaxWaitUs() { return mutexMaxWaitUs; }
uint32_t i2cBusBnoMaxUs() { return bnoMaxUs; }
uint32_t i2cBusBnoAvgUs() { return bnoCount ? bnoSumUs / bnoCount : 0; }
uint32_t i2cBusAdcMaxUs() { return adcMaxUs; }
uint32_t i2cBusAdcAvgUs() { return adcCount ? adcSumUs / adcCount : 0; }

void i2cBusResetWindow() {
  mutexMaxWaitUs = 0;
  bnoMaxUs = bnoSumUs = bnoCount = 0;
  adcMaxUs = adcSumUs = adcCount = 0;
}

