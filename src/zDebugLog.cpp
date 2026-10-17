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
  return write(&c, 1);
}

// Timestamped ring-buffer write: every log line gets a "[ss.mmm] " prefix so
// freezes can be correlated with the exact moment each line was emitted.
size_t DebugLogger::write(const uint8_t* buffer, size_t size) {
  if (!debugEnabled) return size;
  static bool atLineStart = true;

  for (size_t i = 0; i < size; i++) {
    if (atLineStart) {
      uint32_t ms = millis();
      char pre[16];
      int n = snprintf(pre, sizeof(pre), "[%lu.%03lu] ",
                       (unsigned long)(ms / 1000UL), (unsigned long)(ms % 1000UL));
      if (n > 0) {
        portENTER_CRITICAL(&logMux);
        for (int j = 0; j < n; j++) {
          logBuf[totalWritten % LOG_BUF_SIZE] = pre[j];
          totalWritten++;
        }
        portEXIT_CRITICAL(&logMux);
      }
      atLineStart = false;
    }

    portENTER_CRITICAL(&logMux);
    logBuf[totalWritten % LOG_BUF_SIZE] = (char)buffer[i];
    totalWritten++;
    portEXIT_CRITICAL(&logMux);

    if (buffer[i] == '\n') atLineStart = true;
  }
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
