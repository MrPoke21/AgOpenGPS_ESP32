#pragma once

// Hardware-independent IMU logic (yaw continuity after a sensor reset,
// status-0 grace, batch timestamps, stale detection, recovery backoff).
// No Arduino / BNO08x includes on purpose: this header is compiled and
// unit-tested natively (PlatformIO env:native, see test/test_imu_logic).

#include <stdint.h>
#include <math.h>

// Wrap an angle into [0, 360). NaN/Inf are returned unchanged (callers
// filter them before this point).
inline float imuWrap360(float a) {
  a = fmodf(a, 360.0f);
  if (a < 0.0f) a += 360.0f;
  if (a >= 360.0f) a -= 360.0f;
  return a;
}

// ── Yaw continuity ──────────────────────────────────────────────────────────
// The game rotation vector restarts at 0 after a sensor reset, so the raw yaw
// jumps to a new reference. The offset re-bases the raw yaw so the published
// yaw continues from the last value published before the reset. The offset
// lives until the next reset.
struct ImuYawContinuity {
  float offset;       // added to raw yaw [deg]
  float lastOut;      // last published (offset-corrected) yaw [deg]
  bool hasLastOut;    // lastOut holds a valid value
  bool alignPending;  // a reset happened: re-base on the next raw yaw
};

inline void imuYawOnReset(ImuYawContinuity &s) {
  // Only re-base if there is a published value to continue from.
  s.alignPending = s.hasLastOut;
}

// rawYaw: yaw from the quaternion, [0, 360). Returns the published yaw.
inline float imuYawApply(ImuYawContinuity &s, float rawYaw) {
  if (s.alignPending) {
    s.offset = s.lastOut - rawYaw;
    s.alignPending = false;
  }
  float out = imuWrap360(rawYaw + s.offset);
  s.lastOut = out;
  s.hasLastOut = true;
  return out;
}

// ── Status-0 grace ──────────────────────────────────────────────────────────
// A status-0 sample (fusion "unreliable") is dropped only while the previous
// accepted sample had status >= 1 AND the status-0 run is shorter than the
// grace period. After a longer run the samples are accepted again.
enum ImuStatusVerdict {
  IMU_STATUS_REJECT = 0,
  IMU_STATUS_ACCEPT = 1,
  IMU_STATUS_ACCEPT_GRACE_EXPIRED = 2,  // accepted after a long status-0 run (log it)
};

struct ImuStatusGate {
  bool lastAcceptedPositive;  // previous accepted sample had status >= 1
  bool zeroRun;               // currently inside a status-0 run
  uint32_t zeroRunStartMs;    // millis() at the start of the current run
};

inline ImuStatusVerdict imuStatusGate(ImuStatusGate &g, uint8_t status,
                                      uint32_t nowMs, uint32_t graceMs) {
  if (status != 0) {
    g.zeroRun = false;
    g.lastAcceptedPositive = true;
    return IMU_STATUS_ACCEPT;
  }
  if (!g.zeroRun) {
    g.zeroRun = true;
    g.zeroRunStartMs = nowMs;
  }
  if (!g.lastAcceptedPositive) {
    return IMU_STATUS_ACCEPT;  // no good history to protect
  }
  if ((uint32_t)(nowMs - g.zeroRunStartMs) < graceMs) {
    return IMU_STATUS_REJECT;  // short status-0 blip: keep the old reference
  }
  g.lastAcceptedPositive = false;
  return IMU_STATUS_ACCEPT_GRACE_EXPIRED;
}

// ── Batch timestamps ────────────────────────────────────────────────────────
// Samples drained in one pass all share the same micros() reading. The newest
// one gets tNow, the earlier ones are spaced back by periodUs. Output is
// strictly increasing, never later than tNow, and wrap-safe (uint32 deltas).
inline void imuAssignBatchTimes(uint32_t tNow, uint32_t periodUs,
                                uint32_t lastStamp, uint8_t n, uint32_t *out) {
  for (uint8_t i = 0; i < n; i++) {
    uint32_t back = (uint32_t)(n - 1 - i) * periodUs;
    uint32_t t = tNow - back;
    if ((int32_t)(t - lastStamp) <= 0) t = lastStamp + 1;  // keep strictly increasing
    if ((int32_t)(t - tNow) > 0) t = tNow;                 // never in the future
    out[i] = t;
    lastStamp = t;
  }
}

// ── Stale detection ─────────────────────────────────────────────────────────
// True when the newest accepted sample is older than staleUs (wrap-safe).
inline bool imuIsStale(uint32_t nowUs, uint32_t lastAcceptedUs, uint32_t staleUs) {
  return (uint32_t)(nowUs - lastAcceptedUs) > staleUs;
}

// ── Recovery backoff ────────────────────────────────────────────────────────
// Returns steps[attempt], clamped to the last step (non-blocking retry wait).
inline uint32_t imuNextBackoffMs(uint8_t attempt, const uint32_t *steps, uint8_t nSteps) {
  if (nSteps == 0) return 0;
  uint8_t idx = attempt < nSteps ? attempt : (uint8_t)(nSteps - 1);
  return steps[idx];
}
