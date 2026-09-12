#pragma once

#include <Arduino.h>
#include "Configuration.h"

// Runtime WiFi configuration, persisted in NVS (Preferences).
// Editable at runtime via the web UI served at http://<device-ip>/
// Defaults (used only to seed NVS on first boot) come from Configuration.h.
struct WiFiRuntimeConfig {
  uint8_t mode;          // 1 = Access Point, 0 = Station (client)
  char apSsid[33];
  char apPass[64];
  char staSsid[33];
  char staPass[64];
  uint16_t udpPort;
  uint8_t channel;       // AP WiFi channel (1-13) - lets the user dodge local congestion
};

extern WiFiRuntimeConfig wifiRuntimeConfig;

/** Load WiFi settings from NVS, seeding defaults from Configuration.h on first boot */
void loadWiFiRuntimeConfig();

/** Persist the current wifiRuntimeConfig to NVS */
void saveWiFiRuntimeConfig();

/**
 * Load the WiFi config, start WiFi (AP or STA per config) and the config
 * web server (serviced by its own Core 0 task - isolated from the
 * autosteer/GPS loop). Call once from setup().
 * @return true if WiFi started successfully
 */
bool initWiFiConfigPortal();
