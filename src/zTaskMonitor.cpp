#include "zTaskMonitor.h"
#include "Configuration.h"
#include <esp_task_wdt.h>
#include <freertos/task.h>
#include "zSerial.h"
#if ENABLE_UDP
  #include "zUDP.h"
  extern QueueHandle_t udpSendQueue;  // defined in zUDP.cpp
#endif

#define MON_MAX_TASKS 12
struct MonEntry {
  const char* name;
  TaskHandle_t handle;
};
static MonEntry monEntries[MON_MAX_TASKS];
static int monEntryCount = 0;
static TaskHandle_t monHandle = NULL;
static uint32_t monCycles = 0;

static void taskMonitorTask(void* params);

// ===== INITIALIZATION =====
void initTaskMonitor() {
  // IDF 4.4 signature (Arduino core 2.x). Harmless if already initialized
  // by the system - the return code is deliberately ignored.
  // 10 s timeout, no panic (a trigger only logs, it does not reboot).
  esp_task_wdt_init(10, true);

  xTaskCreatePinnedToCore(
    taskMonitorTask,
    "taskMon",
    2048,   // Stack size
    NULL,
    1,      // Low priority - supervision must not disturb the control loop
    &monHandle,
    0       // Core 0
  );
  if (monHandle != NULL) {
    taskMonitorRegister("taskMon", monHandle);
    esp_task_wdt_add(monHandle);
  }
  DEBUG_PRINTLN("[TASKMON] Task monitor started (5 s period, TWDT 10 s)");
}

// ===== TASK REGISTRATION (for stack high-water mark reporting) =====
void taskMonitorRegister(const char* name, TaskHandle_t handle) {
  if (handle == NULL || monEntryCount >= MON_MAX_TASKS) {
    return;
  }
  monEntries[monEntryCount].name = name;
  monEntries[monEntryCount].handle = handle;
  monEntryCount++;
}

// ===== MONITOR TASK =====
static void taskMonitorTask(void* params) {
  for (;;) {
    monCycles++;

    DEBUG_PRINTF("[TASKMON] cycle %lu: heap=%u minFree=%u\n",
                 (unsigned long)monCycles,
                 (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap());

    // --- Stack high-water mark of every registered task ---
    // A low value means the task came close to overflowing its stack.
    for (int i = 0; i < monEntryCount; i++) {
      UBaseType_t hwm = uxTaskGetStackHighWaterMark(monEntries[i].handle);
      DEBUG_PRINTF("[TASKMON]   %-14s hwm=%u words\n",
                   monEntries[i].name, (unsigned)hwm);
    }

    // --- Queue depths: show packet backlog before packets get dropped ---
#if ENABLE_UDP
    if (udpSendQueue != NULL) {
      DEBUG_PRINTF("[TASKMON] udpSendQueue depth=%u\n",
                   (unsigned)uxQueueMessagesWaiting(udpSendQueue));
    }
#endif
#if !ENABLE_UDP
    if (serialSendQueue != NULL) {
      DEBUG_PRINTF("[TASKMON] serialSendQueue depth=%u\n",
                   (unsigned)uxQueueMessagesWaiting(serialSendQueue));
    }
#endif

    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}
