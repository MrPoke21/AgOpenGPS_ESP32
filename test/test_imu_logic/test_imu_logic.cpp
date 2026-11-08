// Native unit tests for the hardware-independent IMU logic.
// Run: pio test -e native
#include <unity.h>
#include "zImuLogic.h"

void setUp() {}
void tearDown() {}

// ── yaw continuity ──────────────────────────────────────────────────────────
void test_yaw_no_reset_passthrough_wraps() {
  ImuYawContinuity s = {0.0f, 0.0f, false, false};
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, imuYawApply(s, 10.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 359.0f, imuYawApply(s, -1.0f));
}

void test_yaw_reset_keeps_continuity() {
  ImuYawContinuity s = {0.0f, 0.0f, false, false};
  imuYawApply(s, 120.0f);              // published 120
  imuYawOnReset(s);                    // sensor restarts from 0
  float out = imuYawApply(s, 3.0f);    // raw restarts at 3
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 120.0f, out);
  // later samples keep the same offset (offset lives until next reset)
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 125.0f, imuYawApply(s, 8.0f));
}

void test_yaw_reset_without_history_no_offset() {
  ImuYawContinuity s = {0.0f, 0.0f, false, false};
  imuYawOnReset(s);                    // nothing published yet
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 42.0f, imuYawApply(s, 42.0f));
}

void test_yaw_reset_wraps_across_360() {
  ImuYawContinuity s = {0.0f, 0.0f, false, false};
  imuYawApply(s, 350.0f);
  imuYawOnReset(s);
  float out = imuYawApply(s, 20.0f);   // offset = 330 -> out = 350
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 350.0f, out);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, imuYawApply(s, 40.0f));  // 370 -> 10
}

// ── status-0 grace ──────────────────────────────────────────────────────────
void test_status0_short_blip_rejected() {
  ImuStatusGate g = {true, false, 0};
  TEST_ASSERT_EQUAL(IMU_STATUS_ACCEPT, imuStatusGate(g, 3, 0, 1500));
  TEST_ASSERT_EQUAL(IMU_STATUS_REJECT, imuStatusGate(g, 0, 100, 1500));
  TEST_ASSERT_EQUAL(IMU_STATUS_REJECT, imuStatusGate(g, 0, 1000, 1500));
}

void test_status0_long_run_accepted_once_logged() {
  ImuStatusGate g = {true, false, 0};
  imuStatusGate(g, 3, 0, 1500);
  TEST_ASSERT_EQUAL(IMU_STATUS_REJECT, imuStatusGate(g, 0, 100, 1500));
  TEST_ASSERT_EQUAL(IMU_STATUS_ACCEPT_GRACE_EXPIRED, imuStatusGate(g, 0, 1700, 1500));
  // after expiry the status-0 samples keep being accepted
  TEST_ASSERT_EQUAL(IMU_STATUS_ACCEPT, imuStatusGate(g, 0, 1800, 1500));
}

void test_status0_no_positive_history_accepts() {
  ImuStatusGate g = {false, false, 0};  // e.g. after reset, never had status>=1
  TEST_ASSERT_EQUAL(IMU_STATUS_ACCEPT, imuStatusGate(g, 0, 0, 1500));
  TEST_ASSERT_EQUAL(IMU_STATUS_ACCEPT, imuStatusGate(g, 0, 100, 1500));
}

void test_status0_recovers_after_positive() {
  ImuStatusGate g = {true, false, 0};
  imuStatusGate(g, 0, 0, 1500);
  TEST_ASSERT_EQUAL(IMU_STATUS_ACCEPT, imuStatusGate(g, 2, 200, 1500));
  TEST_ASSERT_EQUAL(IMU_STATUS_REJECT, imuStatusGate(g, 0, 300, 1500));  // new run
}

// ── batch timestamps ────────────────────────────────────────────────────────
void test_batch_times_spaced_back_from_now() {
  uint32_t out[8];
  imuAssignBatchTimes(100000, 10000, 0, 4, out);
  TEST_ASSERT_EQUAL_UINT32(70000, out[0]);
  TEST_ASSERT_EQUAL_UINT32(80000, out[1]);
  TEST_ASSERT_EQUAL_UINT32(90000, out[2]);
  TEST_ASSERT_EQUAL_UINT32(100000, out[3]);
}

void test_batch_times_strictly_increasing_and_not_future() {
  uint32_t out[8];
  // lastStamp close to tNow: spacing would go below it -> clamp upward, but never > tNow
  imuAssignBatchTimes(1000, 10000, 995, 3, out);
  TEST_ASSERT_TRUE((int32_t)(out[1] - out[0]) > 0);
  TEST_ASSERT_TRUE((int32_t)(out[2] - out[1]) > 0);
  TEST_ASSERT_TRUE((int32_t)(out[2] - 1000) <= 0);
}

void test_batch_times_wrap_safe() {
  uint32_t out[8];
  uint32_t tNow = 5000;  // micros() wrapped past 2^32 just before
  imuAssignBatchTimes(tNow, 10000, 0xFFFFF000u, 2, out);
  TEST_ASSERT_TRUE((int32_t)(out[1] - out[0]) > 0);
  TEST_ASSERT_TRUE((int32_t)(out[1] - tNow) <= 0);
}

// ── stale detection ─────────────────────────────────────────────────────────
void test_stale_threshold() {
  TEST_ASSERT_FALSE(imuIsStale(1000000, 999000, 250000));
  TEST_ASSERT_TRUE(imuIsStale(1000000, 700000, 250000));
}

void test_stale_wrap_safe() {
  uint32_t last = 0xFFFFFF00u;
  uint32_t now = last + 100000u;  // wrapped
  TEST_ASSERT_FALSE(imuIsStale(now, last, 250000));
  TEST_ASSERT_TRUE(imuIsStale(last + 300000u, last, 250000));
}

// ── recovery backoff ────────────────────────────────────────────────────────
void test_backoff_sequence_clamped() {
  const uint32_t steps[] = {1000, 2000, 5000, 10000};
  TEST_ASSERT_EQUAL_UINT32(1000, imuNextBackoffMs(0, steps, 4));
  TEST_ASSERT_EQUAL_UINT32(2000, imuNextBackoffMs(1, steps, 4));
  TEST_ASSERT_EQUAL_UINT32(5000, imuNextBackoffMs(2, steps, 4));
  TEST_ASSERT_EQUAL_UINT32(10000, imuNextBackoffMs(3, steps, 4));
  TEST_ASSERT_EQUAL_UINT32(10000, imuNextBackoffMs(200, steps, 4));
}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_yaw_no_reset_passthrough_wraps);
  RUN_TEST(test_yaw_reset_keeps_continuity);
  RUN_TEST(test_yaw_reset_without_history_no_offset);
  RUN_TEST(test_yaw_reset_wraps_across_360);
  RUN_TEST(test_status0_short_blip_rejected);
  RUN_TEST(test_status0_long_run_accepted_once_logged);
  RUN_TEST(test_status0_no_positive_history_accepts);
  RUN_TEST(test_status0_recovers_after_positive);
  RUN_TEST(test_batch_times_spaced_back_from_now);
  RUN_TEST(test_batch_times_strictly_increasing_and_not_future);
  RUN_TEST(test_batch_times_wrap_safe);
  RUN_TEST(test_stale_threshold);
  RUN_TEST(test_stale_wrap_safe);
  RUN_TEST(test_backoff_sequence_clamped);
  return UNITY_END();
}
