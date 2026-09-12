#include "zDebugLog.h"

static const size_t LOG_BUF_SIZE = 4096;
static char logBuf[LOG_BUF_SIZE];
static uint32_t totalWritten = 0;  // monotonically increasing count of bytes ever written
static portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool debugEnabled = false;  // always starts OFF on boot, never persisted

DebugLogger DebugLog;

void setDebugEnabled(bool enabled) {
  debugEnabled = enabled;
}

bool isDebugEnabled() {
  return debugEnabled;
}

size_t DebugLogger::write(uint8_t c) {
  if (!debugEnabled) return 1;
  //Serial.write(c);
  portENTER_CRITICAL(&logMux);
  logBuf[totalWritten % LOG_BUF_SIZE] = (char)c;
  totalWritten++;
  portEXIT_CRITICAL(&logMux);
  return 1;
}

size_t DebugLogger::write(const uint8_t* buffer, size_t size) {
  if (!debugEnabled) return size;
  //Serial.write(buffer, size);
  portENTER_CRITICAL(&logMux);
  for (size_t i = 0; i < size; i++) {
    logBuf[totalWritten % LOG_BUF_SIZE] = (char)buffer[i];
    totalWritten++;
  }
  portEXIT_CRITICAL(&logMux);
  return size;
}

size_t DebugLogger::readNew(char* dest, size_t maxLen, uint32_t& cursor) {
  size_t copied;
  portENTER_CRITICAL(&logMux);
  uint32_t total = totalWritten;
  if (cursor > total) cursor = total;
  uint32_t available = total - cursor;
  if (available > LOG_BUF_SIZE) {
    cursor = total - LOG_BUF_SIZE;  // reader fell behind - skip to oldest available byte
    available = LOG_BUF_SIZE;
  }
  copied = (available < maxLen) ? available : maxLen;
  for (size_t i = 0; i < copied; i++) {
    dest[i] = logBuf[(cursor + i) % LOG_BUF_SIZE];
  }
  cursor += copied;
  portEXIT_CRITICAL(&logMux);
  return copied;
}
