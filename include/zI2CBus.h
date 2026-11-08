#ifndef ZI2CBUS_H
#define ZI2CBUS_H

#include <Arduino.h>
#include <freertos/semphr.h>

// Shared I2C bus (BNO08x IMU + ADS1115 steering ADC on the same Wire bus).
// Every Wire transaction runs inside i2cBusLock()/i2cBusUnlock(), so the ADC
// task (Core 1) and the loop()/IMU path never talk to the bus at the same time.
// ISRs must never call into this module.

// Initialise the bus (Wire.begin + clock + transaction timeout) and the mutex.
void i2cBusInit();

// Take the bus mutex. Returns false if it could not be taken within
// I2C_MUTEX_TIMEOUT_MS; the caller must then skip this transaction.
bool i2cBusLock();
void i2cBusUnlock();

// Scans the BNO08x candidate addresses; ADS1115_I2C_ADDR is never returned.
// Logs every device found. Returns the first BNO candidate address or 0.
uint8_t i2cBusFindBno(const uint8_t *candidates, uint8_t count);

// Software recovery: bus reset (9 SCL pulses + STOP), Wire re-init. Runs under
// the mutex. Blocking part is bounded (~0.1 ms + short delays).
void i2cBusRecover();

// Health counters for the 10 s [I2C] log line.
uint32_t i2cBusMutexMaxWaitUs();   // max mutex wait since the last health window
uint32_t i2cBusBnoMaxUs();         // max BNO transaction time in the window
uint32_t i2cBusBnoAvgUs();         // average BNO transaction time in the window
uint32_t i2cBusAdcMaxUs();         // max ADC transaction time in the window
uint32_t i2cBusAdcAvgUs();         // average ADC transaction time in the window
void i2cBusResetWindow();          // clears the window counters
uint32_t i2cBusMutexTimeouts();    // lifetime count of mutex lock timeouts
// Record one transaction duration (us). Call while holding the bus mutex.
void i2cBusRecordBno(uint32_t us);
void i2cBusRecordAdc(uint32_t us);

#endif

