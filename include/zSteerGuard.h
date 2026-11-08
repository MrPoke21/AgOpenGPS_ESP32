#pragma once

// Hardware-independent steering-input safety logic (ADC plausibility filter,
// WAS-stale watchdog, soft-start ramp). No Arduino / Wire includes on purpose:
// compiled and unit-tested natively (PlatformIO env:native, test/test_steer_guard).
// This header only gates the INPUT and the PWM enable. It does not change the
// PID / PWM-curve logic.

#include <stdint.h>

// ── ADC plausibility ────────────────────────────────────────────────────────
// A raw ADS1115 sample is accepted only if it lies in [minCounts, maxCounts]
// and changes by no more than maxStep from the last accepted sample. A failed
// I2C transaction is reported by the caller as "no sample" and never reaches
// this filter.
struct AdcPlausibility {
  int16_t lastGood;   // last accepted raw value
  bool hasLastGood;   // lastGood holds a value
  uint32_t badCount;  // rejected samples (lifetime)
};

inline bool adcSampleAccept(AdcPlausibility &p, int16_t raw,
                            int16_t minCounts, int16_t maxCounts,
                            int16_t maxStep) {
  if (raw < minCounts || raw > maxCounts) {
    p.badCount++;
    return false;
  }
  if (p.hasLastGood) {
    int32_t d = (int32_t)raw - (int32_t)p.lastGood;
    if (d < 0) d = -d;
    if (d > (int32_t)maxStep) {
      p.badCount++;
      return false;
    }
  }
  p.lastGood = raw;
  p.hasLastGood = true;
  return true;
}

// ── WAS stale watchdog + soft start ─────────────────────────────────────────
// Tracks the age of the last valid steering sample. When it is older than
// staleMs the PWM must be disabled. After a valid sample returns, the PWM
// magnitude limit ramps up from 0 (soft start), so the motor does not kick.
struct WasGuard {
  uint32_t lastValidMs;  // millis() of the last valid sample
  bool hasValid;         // at least one valid sample seen
  bool stale;            // currently in the stale (PWM-off) state
  uint32_t staleEvents;  // transitions into stale (lifetime)
  int16_t rampLimit;     // current allowed |PWM| during soft start
};

inline void wasGuardOnValid(WasGuard &g, uint32_t nowMs) {
  g.lastValidMs = nowMs;
  g.hasValid = true;
}

// Updates the stale state. Returns true while the PWM is allowed.
inline bool wasGuardUpdate(WasGuard &g, uint32_t nowMs, uint32_t staleMs) {
  bool fresh = g.hasValid && (uint32_t)(nowMs - g.lastValidMs) <= staleMs;
  if (!fresh) {
    if (!g.stale) {
      g.staleEvents++;
    }
    g.stale = true;
    g.rampLimit = 0;  // restart the soft start on the next valid sample
    return false;
  }
  g.stale = false;
  return true;
}

// Soft start: returns the |PWM| limit for this control tick. Call once per
// tick while the PWM is allowed; the limit grows by step up to fullLimit.
inline int16_t wasGuardRamp(WasGuard &g, int16_t fullLimit, int16_t step) {
  if (g.rampLimit < fullLimit) {
    int16_t next = (int16_t)(g.rampLimit + step);
    g.rampLimit = (next > fullLimit) ? fullLimit : next;
  }
  return g.rampLimit;
}

