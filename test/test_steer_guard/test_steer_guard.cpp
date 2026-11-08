// Native unit tests for the hardware-independent steering-input safety logic.
// Run: pio test -e native
#include <unity.h>
#include "zSteerGuard.h"

void setUp() {}
void tearDown() {}

// ── ADC plausibility ────────────────────────────────────────────────────────
void test_adc_accepts_plausible_sample() {
  AdcPlausibility p = {0, false, 0};
  TEST_ASSERT_TRUE(adcSampleAccept(p, 6800, 0, 32767, 1500));
  TEST_ASSERT_TRUE(adcSampleAccept(p, 6900, 0, 32767, 1500));
  TEST_ASSERT_EQUAL_UINT32(0, p.badCount);
}

void test_adc_rejects_out_of_range() {
  AdcPlausibility p = {0, false, 0};
  TEST_ASSERT_FALSE(adcSampleAccept(p, -5, 0, 32767, 1500));
  TEST_ASSERT_FALSE(adcSampleAccept(p, 32000, 0, 31999, 1500));
  TEST_ASSERT_EQUAL_UINT32(2, p.badCount);
}

void test_adc_rejects_jump_keeps_last_good() {
  AdcPlausibility p = {0, false, 0};
  TEST_ASSERT_TRUE(adcSampleAccept(p, 6800, 0, 32767, 1500));
  TEST_ASSERT_FALSE(adcSampleAccept(p, 8800, 0, 32767, 1500));
  TEST_ASSERT_EQUAL_INT16(6800, p.lastGood);  // last good value is kept
  TEST_ASSERT_TRUE(adcSampleAccept(p, 6850, 0, 32767, 1500));
  TEST_ASSERT_EQUAL_UINT32(1, p.badCount);
}

void test_adc_negative_step_limited_symmetrically() {
  AdcPlausibility p = {0, false, 0};
  TEST_ASSERT_TRUE(adcSampleAccept(p, 5000, 0, 32767, 1500));
  TEST_ASSERT_FALSE(adcSampleAccept(p, 3399, 0, 32767, 1500));  // step 1601
  TEST_ASSERT_TRUE(adcSampleAccept(p, 3500, 0, 32767, 1500));   // step 1500
}

// ── WAS stale watchdog ──────────────────────────────────────────────────────
void test_was_no_valid_sample_is_stale() {
  WasGuard g = {0, false, false, 0, 0};
  TEST_ASSERT_FALSE(wasGuardUpdate(g, 1000, 100));  // never had a valid sample
  TEST_ASSERT_TRUE(g.stale);
  TEST_ASSERT_EQUAL_UINT32(1, g.staleEvents);
}

void test_was_fresh_within_window() {
  WasGuard g = {0, false, false, 0, 0};
  wasGuardOnValid(g, 1000);
  TEST_ASSERT_TRUE(wasGuardUpdate(g, 1100, 100));  // exactly at the limit: fresh
  TEST_ASSERT_FALSE(g.stale);
}

void test_was_stale_after_window_counts_once() {
  WasGuard g = {0, false, false, 0, 0};
  wasGuardOnValid(g, 1000);
  TEST_ASSERT_FALSE(wasGuardUpdate(g, 1101, 100));
  TEST_ASSERT_FALSE(wasGuardUpdate(g, 1200, 100));  // still stale, same event
  TEST_ASSERT_EQUAL_UINT32(1, g.staleEvents);
}

void test_was_recovers_with_soft_start() {
  WasGuard g = {0, false, false, 0, 0};
  wasGuardOnValid(g, 1000);
  wasGuardUpdate(g, 1500, 100);             // stale
  wasGuardOnValid(g, 1500);                 // valid sample returns
  TEST_ASSERT_TRUE(wasGuardUpdate(g, 1500, 100));
  TEST_ASSERT_EQUAL_INT16(8, wasGuardRamp(g, 255, 8));    // ramp starts from 0
  TEST_ASSERT_EQUAL_INT16(16, wasGuardRamp(g, 255, 8));
}

void test_ramp_clamps_to_full_limit() {
  WasGuard g = {0, true, false, 0, 0};
  int16_t v = 0;
  for (int i = 0; i < 100; i++) v = wasGuardRamp(g, 255, 8);
  TEST_ASSERT_EQUAL_INT16(255, v);
}

void test_ramp_resets_after_stale() {
  WasGuard g = {0, true, false, 0, 0};
  for (int i = 0; i < 50; i++) wasGuardRamp(g, 255, 8);
  wasGuardUpdate(g, 5000, 100);             // stale -> ramp reset
  TEST_ASSERT_EQUAL_INT16(0, g.rampLimit);
}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_adc_accepts_plausible_sample);
  RUN_TEST(test_adc_rejects_out_of_range);
  RUN_TEST(test_adc_rejects_jump_keeps_last_good);
  RUN_TEST(test_adc_negative_step_limited_symmetrically);
  RUN_TEST(test_was_no_valid_sample_is_stale);
  RUN_TEST(test_was_fresh_within_window);
  RUN_TEST(test_was_stale_after_window_counts_once);
  RUN_TEST(test_was_recovers_with_soft_start);
  RUN_TEST(test_ramp_clamps_to_full_limit);
  RUN_TEST(test_ramp_resets_after_stale);
  return UNITY_END();
}

