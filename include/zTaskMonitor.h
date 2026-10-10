#ifndef ZTASKMONITOR_H
#define ZTASKMONITOR_H

#include <Arduino.h>

/**
 * Lightweight task / heap supervisor ("task manager").
 *
 * Every 5 s it logs to the debug log:
 *  - every FreeRTOS task: name, state, priority, stack high-water mark
 *  - free heap and minimum free heap since boot
 *  - depth of the communication queues (packet backlog before a drop)
 *
 * It also initializes the Task Watchdog Timer (TWDT, 10 s timeout) so the
 * critical tasks (packet parser, UDP I/O, monitor itself) can subscribe and
 * feed it - a hung task shows up in the log instead of freezing silently.
 *
 * The stack high-water marks logged here are the measurement basis for
 * further stack tuning (see the UART_RX task stack in main.cpp: if the
 * monitor reports a HWM below ~1 KB, raise that stack back).
 */

/**
 * Register a task for stack high-water mark monitoring.
 * Call right after creating the task, passing its handle.
 */
void taskMonitorRegister(const char* name, TaskHandle_t handle);

/**
 * Append the task-monitor telemetry fields to a JSON object under
 * construction (called by the web UI /status handler). Appends
 * leading-comma separated "key":value pairs - call it before the closing
 * brace of the JSON object:
 *   - "tmTasks": number of registered tasks
 *   - "udpq":    UDP send-queue depth (-1 when the queue does not exist)
 *   - "tmHwm":   per-task stack high-water mark in bytes (ESP-IDF measures
 *                stack HWM in bytes, not FreeRTOS words), keyed by task name
 */
void taskMonitorAppendStatus(String& j);

void initTaskMonitor();

#endif  // ZTASKMONITOR_H
