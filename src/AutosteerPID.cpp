
#include <AutosteerPID.h>
#include <Configuration.h>
#include <algorithm>

static bool motorWasEnabled = false;

// Global PWM variables (defined here, declared as extern in header)
int16_t pwmDrive = 0;
float pValue = 0, errorAbs = 0, highLowPerDeg = 0;

// ── Auto-tune state ────────────────────────────────────────────────────────────
#ifdef USE_AUTOTUNE_PID
struct AutoTuneData {
  uint16_t magic;
  float    Kd;
};
#define KD_AUTOTUNE_MAGIC 0xA1B2

static float    learnedKd   = 0.0f;
static bool     kdDirty     = false;
static uint32_t lastKdSave  = 0;
static float    lastError   = 0.0f;
// I-term accumulator - lives at file scope so the web telemetry (/status)
// can report the live I output between PID ticks
static float    integralError = 0.0f;

// Circular buffer for zero-crossing detection (KD_TUNE_WINDOW samples)
static float   errorHistory[KD_TUNE_WINDOW] = {};
static uint8_t errorIdx   = 0;
static uint8_t tuneCounter = 0;

static void saveAutoTuneKd() {
  AutoTuneData d = { KD_AUTOTUNE_MAGIC, learnedKd };
  EEPROM.put(80, d);
  EEPROM.commit();
  kdDirty    = false;
  lastKdSave = millis();
  Serial.printf("[AUTOTUNE] Kd=%.1f mentve EEPROM-ba\n", learnedKd);
}

void loadAutoTuneKd() {
  AutoTuneData d;
  EEPROM.get(80, d);
  if (d.magic == KD_AUTOTUNE_MAGIC) {
    learnedKd = constrain(d.Kd, 0.0f, KD_MAX);
    Serial.printf("[AUTOTUNE] Kd=%.1f betöltve\n", learnedKd);
  } else {
    learnedKd = 0.0f;
    Serial.println("[AUTOTUNE] Nincs mentett Kd, nulláról indul");
  }
}

// Hívd 50 Hz-en (minden PID tick-ben)
static void autoTuneStep(float error) {
  errorHistory[errorIdx] = error;
  errorIdx = (errorIdx + 1) % KD_TUNE_WINDOW;

  // Csak minden KD_TUNE_WINDOW tick-ben értékel (= KD_TUNE_WINDOW * 20ms)
  if (++tuneCounter < KD_TUNE_WINDOW) return;
  tuneCounter = 0;

  // Nullátmenetek számlálása az ablakban
  uint8_t crossings = 0;
  for (uint8_t i = 1; i < KD_TUNE_WINDOW; i++) {
    uint8_t a = (errorIdx + i - 1) % KD_TUNE_WINDOW;
    uint8_t b = (errorIdx + i)     % KD_TUNE_WINDOW;
    if (errorHistory[a] * errorHistory[b] < 0.0f) crossings++;
  }

  float prevKd = learnedKd;
  if (crossings >= 2) {
    // Oszcillál → több csillapítás kell
    learnedKd = constrain(learnedKd + KD_STEP_UP, 0.0f, KD_MAX);
  } else if (crossings == 0 && learnedKd > 0.0f) {
    // Jól csillapított → lassan visszavesz
    learnedKd = constrain(learnedKd - KD_STEP_DOWN, 0.0f, KD_MAX);
  }

  if (learnedKd != prevKd) {
    kdDirty = true;
    DEBUG_PRINTF("[AUTOTUNE] Kd: %.1f → %.1f (crossings=%d)\n", prevKd, learnedKd, crossings);
  }

  // Időzített mentés
  if (kdDirty && (millis() - lastKdSave > KD_SAVE_INTERVAL_MS)) {
    saveAutoTuneKd();
  }
}

// ── Telemetry getters (web UI Telemetria tab / /status endpoint) ──────────────
float getLearnedKd(void)    { return learnedKd; }
float getIntegralError(void) { return integralError; }
#endif // USE_AUTOTUNE_PID
// ──────────────────────────────────────────────────────────────────────────────

/**
 * Motor State Control - manages motor enable/disable state transitions
 * Prevents repeated on/off cycles and ensures clean state management
 */
void motorStateControl(void) {
  if (motorWasEnabled && !steerEnable) {
    // Transition: Motor was ON, now should be OFF
    pwmDisplay = 0;  // Clear PWM display value
    digitalWrite(PWM_ENABLE, LOW);
    ledcWrite(PWM_CHANNEL_LPWM, 0);
    ledcWrite(PWM_CHANNEL_RPWM, 0);
    motorWasEnabled = false;
#ifdef USE_AUTOTUNE_PID
    lastError = 0.0f;  // D tag állapot törlése
    if (kdDirty) saveAutoTuneKd();  // Mentés munkamenet végén
#endif
    Serial.println("[MOTOR] Motor safely shut down");
  } else if (steerEnable && !motorWasEnabled) {
    // Transition: Motor should be ON
    motorWasEnabled = true;
    digitalWrite(PWM_ENABLE, HIGH);
    Serial.println("[MOTOR] Motor enabled");
  }
}

void calcSteeringPID(void) {
  float steerAngleError = steerAngleActual - steerAngleSetPoint;
  errorAbs = abs(steerAngleError);

#ifdef USE_AUTOTUNE_PID
  // ── Teljes PID (P+I+D) + öntanuló D tag ────────────────────────────────────
  // P tag
  pValue = steerSettings.Kp * steerAngleError;

  // I tag – steady-state hiba megszüntetése (anti-windup clamp)
  integralError = constrain(integralError + steerAngleError, -KI_MAX_INTEGRAL, KI_MAX_INTEGRAL);
  float iValue = KI_GAIN * integralError;

  // D tag – öntanult csillapítás (nullátmenet-alapú auto-tune)
  float dValue = learnedKd * (steerAngleError - lastError);
  lastError = steerAngleError;
  autoTuneStep(steerAngleError);

  pwmDrive = (int16_t)(pValue + iValue + dValue);

#else
  // ── Eredeti P-only logika ──────────────────────────────────────────────────
  // P szabályozó
  pValue = steerSettings.Kp * steerAngleError;
  pwmDrive = (int16_t)pValue;

#endif // USE_AUTOTUNE_PID

  // PWM maximum kiszámítása
  int16_t newMax = 0;
  if (errorAbs < LOW_HIGH_DEGREES) {
    newMax = (errorAbs * highLowPerDeg) + steerSettings.lowPWM;
  } else {
    newMax = steerSettings.highPWM;
  }

  // HOLTSÁV KOMPENZÁCIÓ (motor indítás)
  if (pwmDrive > 0) {
    pwmDrive += steerSettings.minPWM;
  } else if (pwmDrive < 0) {
    pwmDrive -= steerSettings.minPWM;
  }

  // Limitálás ELŐBB – hogy a ramp referencia helyes legyen
  if (pwmDrive > newMax) pwmDrive = newMax;
  if (pwmDrive < -newMax) pwmDrive = -newMax;

  if (steerConfig.MotorDriveDirection) pwmDrive *= -1;

  motorDrive();
}


//#########################################################################################

void motorDrive(void) {
  // Motor on/off is handled by motorStateControl()
  // This function only handles PWM direction and speed
  // Scale 0-255 PWM to 0-1020 for 10-bit resolution (multiply by 4)
  
  if (!steerEnable) {
    return;  // No PWM output when steering disabled
  }
  
  pwmDisplay = abs(pwmDrive); // Update display variable with absolute PWM value
  int16_t scaledPWM = pwmDrive * 4;  // Scale from 8-bit to 10-bit range
  
  if (steerConfig.CytronDriver) {
    // Cytron MD30C Driver Dir + PWM Signal
    if (pwmDrive >= 0) {
      ledcWrite(PWM_CHANNEL_LPWM, 1023);  // Full scale for 10-bit
    } else {
      ledcWrite(PWM_CHANNEL_LPWM, 0);
    }
    //write out the scaled PWM value
    ledcWrite(PWM_CHANNEL_RPWM, abs(scaledPWM));
  } else {
    // Standard dual direction control
    if (pwmDrive > 0) {
      ledcWrite(PWM_CHANNEL_RPWM, 0);
      ledcWrite(PWM_CHANNEL_LPWM, scaledPWM);
    } else {
      ledcWrite(PWM_CHANNEL_LPWM, 0);
      ledcWrite(PWM_CHANNEL_RPWM, abs(scaledPWM));
    }
  }
}
