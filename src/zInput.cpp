#include <Configuration.h>
#include "zTaskMonitor.h"
#include <zInput.h>
#include <zI2CBus.h>
#include <zSteerGuard.h>
#include <Wire.h>
#include <freertos/semphr.h>

int16_t steeringPosition = 0;
Adafruit_ADS1115 adc;
bool adcConnected = false;
int16_t current_zero = 0;
int16_t cachedSteeringSensor = 0;
int16_t cachedCurrentSensor = 0;
int16_t filteredSteeringSensor = 0;  // EMA filtered steering
int16_t filteredCurrentSensor = 0;    // EMA filtered current
uint32_t lastADCReadTime = 0;

// Mutex for cache access from both ADC task and main loop
SemaphoreHandle_t adcCacheMutex = NULL;

// Adaptive ADC sampling based on sensor type
const uint16_t ADC_READ_INTERVAL_MS = 10;  // 100Hz for all sensor types (~10ms lag with median-3)

const float ADC_FILTER_ALPHA = 1.0f;  // EMA bypassed (alpha=1.0) – median-3 alone gives ~10ms lag

// Median filter circular buffers (one per channel) for spike rejection
static int16_t steerMedianBuf[ADC_MEDIAN_FILTER_SIZE] = {};
static int16_t currentMedianBuf[ADC_MEDIAN_FILTER_SIZE] = {};
static uint8_t medianIdx = 0;

// Compute median of ADC_MEDIAN_FILTER_SIZE samples via insertion sort on a local copy
static int16_t computeMedian(const int16_t* buf) {
  int16_t tmp[ADC_MEDIAN_FILTER_SIZE];
  for (int i = 0; i < ADC_MEDIAN_FILTER_SIZE; i++) tmp[i] = buf[i];
  for (int i = 1; i < ADC_MEDIAN_FILTER_SIZE; i++) {
    int16_t key = tmp[i];
    int j = i - 1;
    while (j >= 0 && tmp[j] > key) { tmp[j + 1] = tmp[j]; j--; }
    tmp[j + 1] = key;
  }
  return tmp[ADC_MEDIAN_FILTER_SIZE / 2];
}

// ── Bounded ADS1115 reader ─────────────────────────────────────────────────
// The Adafruit library's readADC_SingleEnded() spins forever on
// conversionComplete() and reports no bus error, so the ADC task could hang on
// a stuck bus. This reader does the same single-shot conversion (same config
// bits as the library: single-shot, 860 SPS, +/-6.144 V) with explicit
// endTransmission() checks and a bounded poll for the conversion-ready bit.
static uint32_t adcIoErrors = 0;        // failed transactions (lifetime)
static AdcPlausibility steerPlaus = {0, false, 0};
static AdcPlausibility currentPlaus = {0, false, 0};
static int16_t lastGoodSteerRaw = 0;
static int16_t lastGoodCurrentRaw = 0;
static uint32_t steerValidMs = 0;      // millis() of the last plausible WAS sample
static WasGuard wasGuard = {0, false, false, 0, 0};

static bool adsWriteReg(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(ADS1115_I2C_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)(value >> 8));
  Wire.write((uint8_t)(value & 0xFF));
  return Wire.endTransmission() == 0;
}

static bool adsReadReg(uint8_t reg, uint16_t &out) {
  Wire.beginTransmission(ADS1115_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)ADS1115_I2C_ADDR, (uint8_t)2) != 2) return false;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  out = ((uint16_t)hi << 8) | lo;
  return true;
}

// Return codes of adsReadChannelBounded(), counted separately in the health line.
enum AdsResult : uint8_t {
  ADS_OK = 0,
  ADS_ERR_MUTEX = 1,   // bus busy: mutex not obtained within I2C_MUTEX_TIMEOUT_MS
  ADS_ERR_NOT_READY = 2,  // conversion not finished within the bounded poll
  ADS_ERR_BUS = 3,     // NACK / timeout / short read
};
static uint32_t adcErrMutex = 0, adcErrNotReady = 0, adcErrBus = 0;
static uint8_t adcCycleCount = 0;        // ADC cycle counter (current-channel divider)
static uint32_t adcRetryRecovered = 0;   // not-ready at first read, OK after the retry
static uint16_t adcLastNotReadyCfg = 0;  // raw CONFIG register of the last not-ready read

// Two-phase read so the bus mutex is NOT held during the ~1.2 ms conversion:
//   phase 1 (locked): write the config, which starts the single-shot conversion
//   wait (unlocked):  vTaskDelay, so the IMU can use the bus meanwhile
//   phase 2 (locked): check the OS/ready bit once, then read the result
// The ready check is a single read, not a poll, so the locked time stays short.
static const TickType_t ADS_CONVERSION_WAIT_TICKS = pdMS_TO_TICKS(2);  // >= 1.2 ms conversion at 860 SPS

static AdsResult adsReadChannelBounded(uint8_t channel, int16_t &out) {
  if (channel > 3) return ADS_ERR_BUS;

  // Phase 1: start the conversion (locked, short)
  if (!i2cBusLock()) {
    adcErrMutex++;
    return ADS_ERR_MUTEX;
  }
  uint32_t t0 = micros();
  // Same bits as Adafruit_ADS1X15::startADCReading() for single-shot mode.
  uint16_t config = ADS1X15_REG_CONFIG_CQUE_1CONV | ADS1X15_REG_CONFIG_CLAT_NONLAT |
                    ADS1X15_REG_CONFIG_CPOL_ACTVLOW | ADS1X15_REG_CONFIG_CMODE_TRAD |
                    ADS1X15_REG_CONFIG_MODE_SINGLE | GAIN_TWOTHIRDS | RATE_ADS1115_860SPS |
                    MUX_BY_CHANNEL[channel] | ADS1X15_REG_CONFIG_OS_SINGLE;
  bool busOk = adsWriteReg(ADS1X15_REG_POINTER_CONFIG, config);
  i2cBusUnlock();
  if (!busOk) {
    adcErrBus++;
    return ADS_ERR_BUS;
  }

  // Wait for the conversion without holding the bus (task yields, not spins)
  vTaskDelay(ADS_CONVERSION_WAIT_TICKS);

  // Phase 2: read the result. One retry after another unlocked wait covers a
  // conversion that finishes late. The bus is locked only for each read.
  uint16_t raw = 0;
  AdsResult result = ADS_ERR_NOT_READY;
  for (uint8_t attempt = 0; attempt < 2 && result == ADS_ERR_NOT_READY; attempt++) {
    if (attempt > 0) {
      vTaskDelay(ADS_CONVERSION_WAIT_TICKS);  // unlocked
    }
    if (!i2cBusLock()) {
      adcErrMutex++;
      return ADS_ERR_MUTEX;
    }
    uint16_t cfg = 0;
    if (!adsReadReg(ADS1X15_REG_POINTER_CONFIG, cfg)) {
      result = ADS_ERR_BUS;
    } else if ((cfg & 0x8000) == 0) {
      result = ADS_ERR_NOT_READY;  // OS bit 0 = conversion still running
      adcLastNotReadyCfg = cfg;
    } else if (!adsReadReg(ADS1X15_REG_POINTER_CONVERT, raw)) {
      result = ADS_ERR_BUS;
    } else {
      result = ADS_OK;
      if (attempt > 0) adcRetryRecovered++;
    }
    i2cBusUnlock();
  }
  i2cBusRecordAdc((uint32_t)(micros() - t0));
  if (result == ADS_ERR_BUS) adcErrBus++;
  if (result == ADS_ERR_NOT_READY) adcErrNotReady++;
  if (result != ADS_OK) return result;
  out = (int16_t)raw;  // ADS1115 is 16-bit: no shift needed
  return ADS_OK;
}

// Forward declarations
void adcTaskFunction(void* parameter);
void initInput() {
  // Setup switch pins with internal pull-up (active-low configuration)
  pinMode(WORKSW_PIN, INPUT_PULLUP);   // Work switch - pulled high, shorted to GND when active
  pinMode(STEERSW_PIN, INPUT_PULLUP);  // Steer switch - pulled high, shorted to GND when active
  
  // Create mutex for ADC cache protection
  adcCacheMutex = xSemaphoreCreateMutex();
  
  // Check ADC
  if (adc.begin(0x48)) {  // Specify I2C address explicitly
    DEBUG_PRINTLN("ADC Connection OK");
    adc.setDataRate(RATE_ADS1115_860SPS);  // 860SPS: ~1.2ms/conversion, 2-ch = ~2.5ms; spike rejection done by median filter
    adc.setGain(GAIN_TWOTHIRDS);
    adcConnected = true;
    
    // Cache initial readings (blocking, but only once on startup)
    cachedCurrentSensor = adc.readADC_SingleEnded(ADC_CHANNEL_SENSOR);
    cachedSteeringSensor = adc.readADC_SingleEnded(ADC_CHANNEL_STEER);
    // Initialize filtered values
    filteredCurrentSensor = cachedCurrentSensor;
    filteredSteeringSensor = cachedSteeringSensor;
    current_zero = cachedCurrentSensor;
    lastADCReadTime = millis();
    
    // Create FreeRTOS task for ADC reading (runs on Core 1, low priority)
    TaskHandle_t adcTaskHandle = NULL;
    xTaskCreatePinnedToCore(
      adcTaskFunction,
      "adcRead",
      4096,     // Stack size (HWM measured: 820 B free at 2048)
      NULL,      // Parameter
      1,         // Priority (low - will not block main loop)
      &adcTaskHandle,  // Task handle
      1          // Core 1 (leaves Core 0 for WiFi)
    );
    taskMonitorRegister("adcRead", adcTaskHandle);
    
    DEBUG_PRINTLN("[ADC] Background task created for non-blocking reads");
  } else {
    DEBUG_PRINTLN("ADC Connection FAILED!");
  }
}

void inputHandler() {
  if (!adcConnected) {
    return;
  }
  
  // Get filtered sensor value with mutex protection (non-blocking read from cache)
  int16_t sensor = filteredCurrentSensor;
  
  if (xSemaphoreTake(adcCacheMutex, 0) == pdTRUE) {
    sensor = filteredCurrentSensor;  // Safe read
    xSemaphoreGive(adcCacheMutex);
  }
  // If mutex is busy, just use stale value - main loop must not block!
  
  // Validate sensor reading
  if (sensor < 0 || sensor > 32767) {
    DEBUG_PRINTLN("ERROR: Invalid ADC reading");
    return;
  }

  // Pressure sensor?
  if (steerConfig.pressureSensor) {
    // Sensor value already EMA-filtered in ADC task, just scale it
    sensorReading = sensor * 0.25f;
    if (sensorReading >= steerConfig.pulseCountMax) {
      steerSwitch = 1; // reset values like it turned off
    }
  }

  // Current sensor? (ACS712) - the client expects a 0-255 value, same as the
  // original AoG firmware. No voltage/ampere conversion, just offset + scale.
  if (steerConfig.currentSensor) {
    // Absolute deviation from the learned zero point
    // (original 10-bit logic: abs(775 - analogRead))
    float deltaCounts = (float)sensor - (float)current_zero;
    float sensorSample = fabsf(deltaCounts) * CURRENT_SENSORE_MODIFIER;

    // Deadband: noise around the zero point reads as 0
    if (fabsf(deltaCounts) < CURRENT_DEADBAND_COUNTS) {
      sensorSample = 0.0f;
    }

    // Auto-zero: re-learn the zero point while the motor is idle
    // (only when autosteer is OFF so motor current can't corrupt it)
    if (sensorSample < CURRENT_AUTOZERO_MAX && steerSwitch == 1) {
      current_zero += (int16_t)(((float)sensor - (float)current_zero) * 0.01f);
    }

    // Same smoothing as the original firmware: 70% previous + 30% new
    sensorReading = sensorReading * 0.7f + sensorSample * 0.3f;
    sensorReading = constrain(sensorReading, 0.0f, 255.0f);

    if (sensorReading >= (float)steerConfig.pulseCountMax) {
      steerSwitch = 1;  // kickout - reset values like the switch was turned off
      // (the button toggle state re-syncs itself via steerEnable in
      // readInputSwitches(), so no currentState/previous reset is needed here)
    }
  }
}

void calcSteerAngle() {
  if (!adcConnected) {
    return;
  }
  
  // Use cached steering position - already EMA filtered in ADC task
  int16_t sensor = filteredSteeringSensor;
  
  // Validate sensor reading
  if (sensor < 0 || sensor > 32767) {
    DEBUG_PRINTLN("ERROR: Invalid steering position ADC reading");
    return;
  }
  
  // Use filtered sensor value directly (already EMA filtered in ADC task)
  steeringPosition = sensor >> 1;
  
  helloSteerPosition = steeringPosition - (WAS_CENTER_POSITION - 5);
  
  // Convert position to steer angle
  // Sensor is pre-filtered, just apply offset and scaling
  int16_t offsetPosition;
  if (steerConfig.invertWas) {
    offsetPosition = steeringPosition - WAS_CENTER_POSITION - steerSettings.wasOffset;
    steerAngleActual = (float)offsetPosition / -(float)steerSettings.countsPerDegree;
  } else {
    offsetPosition = steeringPosition - WAS_CENTER_POSITION + steerSettings.wasOffset;
    steerAngleActual = (float)offsetPosition / (float)steerSettings.countsPerDegree;
  }

  // Ackerman fix - only apply when steering left (negative angle)
  if (steerAngleActual < 0) {
    steerAngleActual = steerAngleActual * (steerSettings.ackermannX100 * 0.01f);
  }
}

/**
 * ADC Background Task - Runs on Core 1 with low priority
 * Non-blocking I2C reads happen here, not in main loop
 * Reads every 10ms (100Hz) for all sensor types – targets ~10ms total lag with median-3
 * No mutex needed during initialization - only main loop reads from cache after setup
 */
void adcTaskFunction(void* parameter) {
  uint32_t lastReadTime = millis();
  
  while (1) {
    if (!adcConnected) {
      vTaskDelay(pdMS_TO_TICKS(ADC_READ_INTERVAL_MS));
      continue;
    }
    
    uint32_t currentTime = millis();
    if (currentTime - lastReadTime >= ADC_READ_INTERVAL_MS) {
      lastReadTime = currentTime;
      
      // Read both ADC channels through the bounded, mutex-protected reader.
      // A failed transaction returns false and keeps the last good value.
      int16_t rawSteer = lastGoodSteerRaw;
      int16_t rawCurrent = lastGoodCurrentRaw;
      // WAS: every cycle. Current/pressure: every ADC_CURRENT_READ_DIVIDER cycles.
      bool readCurrentThisCycle = (adcCycleCount % ADC_CURRENT_READ_DIVIDER) == 0;
      adcCycleCount++;
      bool okSteer = (adsReadChannelBounded(ADC_CHANNEL_STEER, rawSteer) == ADS_OK);
      bool okCurrent = true;  // not read on skipped cycles: not a failure
      if (readCurrentThisCycle) {
        okCurrent = (adsReadChannelBounded(ADC_CHANNEL_SENSOR, rawCurrent) == ADS_OK);
      }
      if (!okSteer || !okCurrent) {
        adcIoErrors++;
        if (adcIoErrors % ADC_FAIL_LOG_EVERY == 1) {
          DEBUG_PRINTF("[ADC] I2C fail #%lu (steer=%d current=%d) mutex=%lu notReady=%lu bus=%lu "
                       "retryOk=%lu lastCfg=0x%04X\n",
                       (unsigned long)adcIoErrors, okSteer, okCurrent,
                       (unsigned long)adcErrMutex, (unsigned long)adcErrNotReady,
                       (unsigned long)adcErrBus, (unsigned long)adcRetryRecovered,
                       (unsigned)adcLastNotReadyCfg);
        }
      }
      // Steering: range + step plausibility. Current/pressure: range only
      // (its legitimate changes can be fast, the median stage handles spikes).
      if (okSteer && adcSampleAccept(steerPlaus, rawSteer, ADC_WAS_MIN_COUNTS,
                                     ADC_WAS_MAX_COUNTS, ADC_WAS_MAX_STEP_COUNTS)) {
        lastGoodSteerRaw = rawSteer;
        steerValidMs = millis();  // read by loop() via adcLastSteerValidMs()
      } else {
        rawSteer = lastGoodSteerRaw;  // failed or implausible: keep last good
      }
      if (readCurrentThisCycle) {
        if (okCurrent && adcSampleAccept(currentPlaus, rawCurrent, ADC_WAS_MIN_COUNTS,
                                         ADC_WAS_MAX_COUNTS, ADC_WAS_MAX_COUNTS)) {
          lastGoodCurrentRaw = rawCurrent;
        } else {
          rawCurrent = lastGoodCurrentRaw;
        }
      }

      // --- Stage 1: Median filter (spike rejection) ---
      // Store new samples in circular buffer, then take the middle value.
      // A single bad I2C read can never affect the output.
      steerMedianBuf[medianIdx] = rawSteer;
      currentMedianBuf[medianIdx] = rawCurrent;
      medianIdx = (medianIdx + 1) % ADC_MEDIAN_FILTER_SIZE;
      int16_t medSteer   = computeMedian(steerMedianBuf);
      int16_t medCurrent = computeMedian(currentMedianBuf);

      // --- Stage 2: EMA on spike-free median output (smooth response) ---
      // Update cache with mutex protection
      if (xSemaphoreTake(adcCacheMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        cachedSteeringSensor = rawSteer;
        cachedCurrentSensor  = rawCurrent;
        filteredSteeringSensor = (int16_t)((1.0f - ADC_FILTER_ALPHA) * filteredSteeringSensor +
                                            ADC_FILTER_ALPHA * medSteer);
        filteredCurrentSensor  = (int16_t)((1.0f - ADC_FILTER_ALPHA) * filteredCurrentSensor +
                                            ADC_FILTER_ALPHA * medCurrent);
        xSemaphoreGive(adcCacheMutex);
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(1));  // Prevent task starvation, short sleep
  }
}

// Steering-input safety accessors (called from loop() / autosteerLoop()).
// The ADC task only writes steerValidMs and adcIoErrors; loop() reads them.
uint32_t adcLastSteerValidMs() { return steerValidMs; }
uint32_t adcFailCount() { return adcIoErrors; }
uint32_t adcFailMutex() { return adcErrMutex; }
uint32_t adcFailNotReady() { return adcErrNotReady; }
uint32_t adcFailBus() { return adcErrBus; }
uint32_t adcRetryOk() { return adcRetryRecovered; }
WasGuard &wasSteerGuard() { return wasGuard; }

// Non-blocking helper (stub for profiling, actual work is in adcTaskFunction)
void updateADCCacheAsync() {
  // No-op: ADC reading happens in background task
  // This function is kept for optional profiling or future extensions
}
