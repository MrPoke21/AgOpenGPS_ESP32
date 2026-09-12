#include <Configuration.h>
#include <zInput.h>
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
    xTaskCreatePinnedToCore(
      adcTaskFunction,
      "adcRead",
      2048,      // Stack size
      NULL,      // Parameter
      1,         // Priority (low - won't block main loop)
      NULL,      // Task handle
      1          // Core 1 (leaves Core 0 for WiFi)
    );
    
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
  if (steerConfig.PressureSensor) {
    // Sensor value already EMA-filtered in ADC task, just scale it
    sensorReading = sensor * 0.25f;
    if (sensorReading >= steerConfig.PulseCountMax) {
      steerSwitch = 1; // reset values like it turned off
    }
  }

  // Current sensor?
  if (steerConfig.CurrentSensor) {
    // Sensor value already EMA-filtered in ADC task, just offset and scale
    sensorReading = abs((float)sensor - current_zero) * CURRENT_SENSORE_MODIFIER;
    sensorReading = constrain(sensorReading, 0, 255);
    if (sensorReading >= steerConfig.PulseCountMax) {
      steerSwitch = 1; // reset values like it turned off
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
  if (steerConfig.InvertWAS) {
    offsetPosition = steeringPosition - WAS_CENTER_POSITION - steerSettings.wasOffset;
    steerAngleActual = (float)offsetPosition / -(float)steerSettings.steerSensorCounts;
  } else {
    offsetPosition = steeringPosition - WAS_CENTER_POSITION + steerSettings.wasOffset;
    steerAngleActual = (float)offsetPosition / (float)steerSettings.steerSensorCounts;
  }

  // Ackerman fix - only apply when steering left (negative angle)
  if (steerAngleActual < 0) {
    steerAngleActual = steerAngleActual * steerSettings.AckermanFix;
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
      
      // Read both ADC channels (blocking I2C, but in background task)
      int16_t rawSteer = adc.readADC_SingleEnded(ADC_CHANNEL_STEER);
      int16_t rawCurrent = adc.readADC_SingleEnded(ADC_CHANNEL_SENSOR);

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

// Non-blocking helper (stub for profiling, actual work is in adcTaskFunction)
void updateADCCacheAsync() {
  // No-op: ADC reading happens in background task
  // This function is kept for optional profiling or future extensions
}
