#pragma once

#include "zDebugLog.h"

/**
 * AgOpenGPS ESP32 Autosteer Configuration
 * Define system parameters, intervals, and calibration constants
 */

// ==================== DEBUG CONFIGURATION ====================
/** Enable debug output to Serial (1: enabled, 0: disabled) */
#define DEBUG 1

// Debug macro - only prints if DEBUG is enabled
// Routed through DebugLog (not Serial directly) so the web UI can show a live log too
#if DEBUG
  #define DEBUG_PRINT(...) DebugLog.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) DebugLog.println(__VA_ARGS__)
  #define DEBUG_PRINTF(...) DebugLog.printf(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...) ((void)0)
  #define DEBUG_PRINTLN(...) ((void)0)
  #define DEBUG_PRINTF(...) ((void)0)
#endif

// ==================== TIMING ====================
/** Autosteer PID calculation interval in milliseconds */
#define AUTOSTEER_INTERVAL 20  // 50Hz control loop

// ==================== MOTOR CONTROL ====================

// ── Vezérlési mód választó ───────────────────────────────────────────────────
// USE_AUTOTUNE_PID: teljes PID (P+I+D) öntanuló D taggal
// Kommenteld ki az eredeti P-only logikához
#define USE_AUTOTUNE_PID
// ────────────────────────────────────────────────────────────────────────────

// ── Öntanuló D tag (auto-tune) – csak USE_AUTOTUNE_PID esetén aktív ─────────
/** Maximális D erősítés értéke */
#define KD_MAX            200.0f
/** Értékelési ablak (sample-ek száma): KD_TUNE_WINDOW * 20ms = értékelési periódus */
#define KD_TUNE_WINDOW    25       // 25 * 20ms = 500ms
/** Kd növelési lépés oszcilláció detektálásakor */
#define KD_STEP_UP        2.0f
/** Kd csökkentési lépés jól csillapított menet esetén */
#define KD_STEP_DOWN      0.5f
/** EEPROM mentési periódus (ms) – legalább ennyi idő teljen el mentések között */
#define KD_SAVE_INTERVAL_MS  300000UL  // 5 perc
/** I tag erősítés (fixed, nincs AOG beállítás) */
#define KI_GAIN           0.5f
/** I tag anti-windup korlát (összegyűlt hiba maximuma) */
#define KI_MAX_INTEGRAL   50.0f
// ────────────────────────────────────────────────────────────────────────────

// ==================== SENSOR CALIBRATION ====================    
/** Current sensor scaling modifier (0.1 = 0.1A per ADC unit for 40W 12V motor) */
#define CURRENT_SENSORE_MODIFIER 0.1

// ==================== ADC CONFIGURATION ====================

/** Median filter window size for spike rejection (must be odd: 3, 5 or 7)
 *  5 samples @ 50Hz = 80ms latency window; 3 samples = 40ms (faster, less rejection) */
#define ADC_MEDIAN_FILTER_SIZE 3

// ==================== SAFETY LIMITS ====================
/** Maximum steering angle limit in degrees */
#define MAX_STEER_ANGLE 45.0f

/** Maximum PWM value for safety */
#define MAX_PWM_VALUE 255

// ==================== WiFi & UDP CONFIGURATION ====================
/** Enable the WiFi radio + web configuration portal (1: enabled, 0: disabled).
 *  Independent from ENABLE_UDP: the config web page can run even while
 *  autosteer data still goes over USB/Serial, so WiFi/latency can be
 *  tested without touching the live data path. */
#define ENABLE_WIFI_CONFIG 1

/** Enable WiFi/UDP as the autosteer data transport instead of Serial (1: enabled, 0: disabled).
 *  Requires ENABLE_WIFI_CONFIG to be enabled as well. */
#define ENABLE_UDP 1

/** Port for the WiFi configuration web page */
#define WEB_SERVER_PORT 80

/** UDP buffer size - max packet size in bytes */
#define UDP_BUFFER_SIZE 1024

/** WiFi TX Power (dBm): 8=7dBm, 20=20dBm, 78=20.5dBm (maximum) */
#define WIFI_TX_POWER 78

/** WiFi RX buffer count (default 16, increase to 32 for lower latency at cost of RAM) */
#define WIFI_RX_BUF_COUNT 32

/** WiFi TX buffer count (default 32, increase to 64 for better throughput) */
#define WIFI_TX_BUF_COUNT 64

// ── The following are DEFAULT values only ───────────────────────────────────
// They seed NVS (Preferences) storage on first boot. After that, the actual
// mode/SSID/password/UDP port are runtime-configurable via the web page at
// http://<device-ip>/ and persist across reboots/reflashes.

/** Default WiFi operating mode: 1 = AP (creates network), 0 = STA (connects to existing) */
/** Note: AP mode has lower latency (~10-20ms), STA mode higher (~30-50ms) */
#define WIFI_MODE 1

/** Default UDP port for data exchange */
#define UDP_PORT 8888

/** Default WiFi Access Point / Station SSID (max 32 characters) */
#define WIFI_SSID "AGOPEN_ESP32_AP"

/** Default WiFi Access Point / Station password (max 63 characters) */
#define WIFI_PASS "12345678"

/** Default AP WiFi channel (1-13). 2.4GHz non-overlapping channels: 1, 6, 11.
 *  Runtime-changeable via the web UI to work around local channel congestion. */
#define WIFI_CHANNEL 6

/** UDP broadcast IP address for STA mode */
#define BROADCAST_IP "192.168.0.255"

// ==================== ADC SENSOR CONFIGURATION ====================
/** ADC Channel 0: Steering Wheel Angle Sensor (WAS) */
#define ADC_CHANNEL_STEER 0

/** ADC Channel 1: Pressure/Current Sensor 
 * Note: Future change - will be replaced with PID motor controller angle sensor */
#define ADC_CHANNEL_SENSOR 1

/** ADC Sensor Type: 0 = Current/Pressure, 1 = PID Motor Angle Feedback (future) 
 * 
 * Sampling Rates:
 * - Type 0 (Pressure/Current): 50Hz (20ms interval) - sufficient for slow pressure/current feedback
 * - Type 1 (PID Motor Angle): 100Hz (10ms interval) - required for fast PID control loop
 */
#define ADC_SENSOR_TYPE 0  // Change to 1 when PID motor angle feedback is implemented

// ==================== AUTOSTEER PID CONSTANTS ====================
/** Angle range (degrees) for low-to-high PWM transition curve */
#define LOW_HIGH_DEGREES 3.0f

// ==================== STEERING SENSOR CALIBRATION ====================
/** Steering wheel angle sensor (WAS) center position ADC value */
#define WAS_CENTER_POSITION 6805

/** Steering sensor scaling reference point (usually CENTER_POSITION - 5) */
#define WAS_HELLO_POSITION 6800

/** Serial buffer size */
#define SERIAL_BUFFER_SIZE 32768

/** Speed output pin configured*/
#define SPEED_IMPULSE_ENABLED       //  Uncomment to enable speed impulse output
#define IMPULSE_PIN 25              // GPIO pin az impulzus kimenethez
#define PULSES_PER_METER 10         // Impulzusok száma méterenként