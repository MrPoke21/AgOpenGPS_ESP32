#pragma once

#include <Arduino.h>

// Print sink used by DEBUG_PRINT/DEBUG_PRINTLN/DEBUG_PRINTF (see Configuration.h).
// Mirrors everything to Serial AND keeps a ring buffer so the web UI can show a live log.
class DebugLogger : public Print {
public:
  size_t write(uint8_t c) override;
  size_t write(const uint8_t* buffer, size_t size) override;

  // Copies bytes written since `cursor` into dest (no null terminator added).
  // `cursor` is updated in place to resume from on the next call.
  size_t readNew(char* dest, size_t maxLen, uint32_t& cursor);
};

extern DebugLogger DebugLog;

/** Runtime debug on/off switch - always starts disabled on boot, never persisted to NVS */
void setDebugEnabled(bool enabled);
bool isDebugEnabled();
