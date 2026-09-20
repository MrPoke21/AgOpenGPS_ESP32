#include "zUDP.h"
#include "zSerial.h"
#include "zWebConfig.h"
#include "Configuration.h"
#include <esp_wifi.h>

// ===== GLOBAL VARIABLES =====
WiFiStatus wifiStatus = WIFI_INIT;
WiFiUDP udp;
WiFiUDP rtcmUdp;
IPAddress udpRemoteIP;
uint16_t udpRemotePort = 0;

static constexpr uint16_t RTCM_UDP_PORT = 2233;
static constexpr size_t RTCM_PACKET_BUFFER_SIZE = 1024;
static volatile bool udpSocketReady = false;

// UDP send queue
QueueHandle_t udpSendQueue = NULL;

// Creates the send queue up front so it's never NULL once any task can reach it
void initUDPQueues() {
  if (udpSendQueue == NULL) {
    udpSendQueue = xQueueCreate(30, sizeof(UDPPacket));
  }
}

// ===== WiFi INITIALIZATION =====
bool initWiFi() {
  DEBUG_PRINTLN("\n[UDP] Initializing WiFi...");
  DEBUG_PRINTF("[UDP] WiFi Buffer Config: RX=%d, TX=%d (optimized for low latency)\n",
               WIFI_RX_BUF_COUNT, WIFI_TX_BUF_COUNT);
  
  // Note: RX/TX buffer tuning requires custom esp_wifi_init_config
  // For now, rely on WiFiUDP + disabling power save

  if (wifiRuntimeConfig.mode == 1) {
    // AP Mode - create access point (lower latency)
    DEBUG_PRINTLN("[UDP] Starting WiFi Access Point (Lower Latency)");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(wifiRuntimeConfig.apSsid, wifiRuntimeConfig.apPass, wifiRuntimeConfig.channel);

    // Power save can only be applied once the WiFi driver is actually running -
    // calling this before WiFi.mode()/softAP() is a silent no-op (driver not init yet)
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);  // Always disabled - power save adds ms of latency
    DEBUG_PRINTLN("[UDP] WiFi Power Save: DISABLED");

    // Set TX power to maximum
    WiFi.setTxPower((wifi_power_t)WIFI_TX_POWER);

    // NOTE: previously also forced 40MHz (HT40) bandwidth here for lower latency,
    // but many client WiFi chipsets (phones especially) don't reliably support
    // 2.4GHz channel bonding in AP mode - they can still associate/get a DHCP
    // lease but then silently fail to complete TCP connections (e.g. the config
    // web page). Stick to 20MHz for compatibility; the channel itself (NOT
    // auto-selected by ESP32 softAP - it always uses a fixed channel) is
    // user-configurable via the web UI to work around local 2.4GHz congestion.

    IPAddress apIP(192, 168, 4, 1);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));

    DEBUG_PRINT("[UDP] AP IP: ");
    DEBUG_PRINTLN(WiFi.softAPIP());
    DEBUG_PRINTLN("[UDP] AP Mode: Recommended for lowest latency");

    wifiStatus = WIFI_AP_READY;
    return true;

  } else {
    // STA Mode - connect to existing network (higher latency)
    DEBUG_PRINTLN("[UDP] Starting WiFi Station Mode (Higher Latency ~30-50ms)");
    WiFi.mode(WIFI_STA);

    // Set TX power to maximum
    WiFi.setTxPower((wifi_power_t)WIFI_TX_POWER);
    wifi_config_t conf = {};
    esp_wifi_get_config(WIFI_IF_STA, &conf);
    conf.sta.listen_interval = 1;  // 1 beacon-intervallumonként ébredjen
    esp_wifi_set_config(WIFI_IF_STA, &conf);
    WiFi.begin(wifiRuntimeConfig.staSsid, wifiRuntimeConfig.staPass);

    uint8_t attempts = 0;
    wifiStatus = WIFI_STA_CONNECTING;

    while (WiFi.status() != WL_CONNECTED && attempts++ < 120) {
      delay(500);
      DEBUG_PRINT(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
      DEBUG_PRINTLN("\n[UDP] WiFi Connected!");
      DEBUG_PRINT("[UDP] IP: ");
      DEBUG_PRINTLN(WiFi.localIP());
      wifiStatus = WIFI_STA_CONNECTED;
      WiFi.setSleep(false);
      esp_wifi_set_ps(WIFI_PS_NONE);
      return true;
    } else {
      DEBUG_PRINTLN("\n[UDP] WiFi Connection Failed after 60 seconds!");
      DEBUG_PRINTLN("[UDP] Falling back to Access Point mode");

      WiFi.disconnect(true);
      WiFi.mode(WIFI_AP);
      WiFi.softAP(wifiRuntimeConfig.apSsid, wifiRuntimeConfig.apPass, wifiRuntimeConfig.channel);

      WiFi.setSleep(false);
      esp_wifi_set_ps(WIFI_PS_NONE);
      WiFi.setTxPower((wifi_power_t)WIFI_TX_POWER);

      IPAddress apIP(192, 168, 4, 1);
      WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));

      DEBUG_PRINT("[UDP] AP IP: ");
      DEBUG_PRINTLN(WiFi.softAPIP());
      wifiStatus = WIFI_AP_READY;
      return true;
    }
  }
}

// ===== UDP INITIALIZATION =====
bool initUDP() {
  DEBUG_PRINTLN("[UDP] Initializing UDP socket...");

  initUDPQueues();  // no-op if already created
  if (udpSendQueue == NULL) {
    DEBUG_PRINTLN("[UDP] Failed to create UDP send queue!");
    return false;
  }

  if (!rtcmUdp.begin(RTCM_UDP_PORT)) {
    DEBUG_PRINTF("[RTCM] Failed to listen on UDP port %d!\n", RTCM_UDP_PORT);
    return false;
  }

  if (udp.begin(wifiRuntimeConfig.udpPort)) {
    udpSocketReady = true;
    DEBUG_PRINTF("[UDP] Listening on port %d\n", wifiRuntimeConfig.udpPort);

    // Start background tasks only after both UDP sockets are open.
    xTaskCreatePinnedToCore(
      udpSendTask,
      "udpSend",
      2048,
      NULL,
      10,  // Increased priority from 1 to 10 (higher = more priority)
      NULL,
      0  // Core 0 (WiFi stack uses Core 0)
    );

    DEBUG_PRINTF("[RTCM] Listening on UDP port %d, forwarding to Serial2\n", RTCM_UDP_PORT);
    xTaskCreatePinnedToCore(
      rtcmReceiveTask,
      "rtcmReceive",
      4096,
      NULL,
      1,
      NULL,
      0
    );
    return true;
  } else {
    rtcmUdp.stop();
    DEBUG_PRINTLN("[UDP] Failed to listen on UDP port!");
    return false;
  }
}

// ===== SEND DATA VIA UDP (NON-BLOCKING QUEUE) =====
bool sendUDP(const uint8_t* data, uint16_t length) {
  if (wifiStatus == WIFI_ERROR || udpSendQueue == NULL) {
    return false;
  }
  
  // Check buffer size
  if (length > 256) {
    DEBUG_PRINTF("[UDP] Data too large: %d bytes\n", length);
    return false;
  }
  
  // No client connected yet
  if (udpRemotePort == 0) {
    return false;
  }
  
  // Queue packet for async sending (non-blocking)
  UDPPacket packet;
  memcpy(packet.data, data, length);
  packet.length = length;

  return xQueueSend(udpSendQueue, &packet, 0) == pdTRUE;
}

void sendNMEA(const uint8_t* data, uint16_t length) {
#if ENABLE_UDP
  if (!sendUDP(data, length)) {
    DEBUG_PRINTLN("[SEND] ERROR: UDP queue full or no client");
  }
#else
  if (!sendSerial(data, length)) {
    DEBUG_PRINTLN("[SEND] ERROR: Serial queue full");
  }
#endif
}

// ===== RECEIVE UDP DATA (polls WiFiUDP, blocks until a packet arrives) ======
uint16_t receiveUDP(uint8_t* buffer, uint16_t maxLen) {
  int packetSize;
  while (!udpSocketReady) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  while ((packetSize = udp.parsePacket()) == 0) {
    vTaskDelay(pdMS_TO_TICKS(1));  // yield while polling for the next datagram
  }

  // Save sender info for responses (first connection only)
  if (udpRemotePort == 0) {
    udpRemoteIP = udp.remoteIP();
    udpRemotePort = udp.remotePort();
    DEBUG_PRINTF("[UDP] First packet from %s:%d\n", udpRemoteIP.toString().c_str(), udpRemotePort);
  }

  int len = udp.read(buffer, maxLen);
  return (len > 0) ? (uint16_t)len : 0;
}

// ===== GET WIFI STATUS =====
WiFiStatus getWiFiStatus() {
  return wifiStatus;
}

// ===== GET CONNECTED CLIENT COUNT =====
uint8_t getWiFiClientCount() {
  if (wifiRuntimeConfig.mode == 1) {
    return WiFi.softAPgetStationNum();
  }
  return (WiFi.status() == WL_CONNECTED) ? 1 : 0;
}

// ===== PRINT WIFI STATUS =====
void printWiFiStatus() {
  DEBUG_PRINTLN("\n===== WiFi Status =====");
  
  switch (wifiStatus) {
    case WIFI_INIT:
      DEBUG_PRINTLN("Status: Initializing");
      break;
    case WIFI_AP_READY:
      DEBUG_PRINTLN("Status: AP Ready");
      DEBUG_PRINT("SSID: ");
      DEBUG_PRINTLN(wifiRuntimeConfig.apSsid);
      DEBUG_PRINT("IP: ");
      DEBUG_PRINTLN(WiFi.softAPIP());
      DEBUG_PRINT("Clients: ");
      DEBUG_PRINTLN(getWiFiClientCount());
      break;
    case WIFI_STA_CONNECTING:
      DEBUG_PRINTLN("Status: Connecting to network");
      break;
    case WIFI_STA_CONNECTED:
      DEBUG_PRINTLN("Status: Connected");
      DEBUG_PRINT("IP: ");
      DEBUG_PRINTLN(WiFi.localIP());
      break;
    case WIFI_ERROR:
      DEBUG_PRINTLN("Status: ERROR");
      break;
  }
  
  DEBUG_PRINTF("UDP Port: %d\n", wifiRuntimeConfig.udpPort);
  DEBUG_PRINTLN("=======================\n");
}

// ===== UDP SEND TASK (Background) =====
void udpSendTask(void* params) {
  UDPPacket packet;
  
  while (1) {
    // Wait for packet in queue (1000ms timeout)
    if (xQueueReceive(udpSendQueue, &packet, pdMS_TO_TICKS(1000))) {
      // Only send if we have a connected client
      if (udpRemotePort != 0 && wifiStatus != WIFI_ERROR) {
        bool success = false;
        // Unicast to the IP:port the client actually sent its first packet
        // from (learned in receiveUDP). Previously this was a hardcoded 9999,
        // which dropped replies when the client sent from another port.
        if (udp.beginPacket(udpRemoteIP, udpRemotePort)) {
          udp.write(packet.data, packet.length);
          success = udp.endPacket();
        }
        if (!success) {
          DEBUG_PRINTF("[UDP] ERROR: Failed to send %d bytes to %s:%d\n", 
                       packet.length, udpRemoteIP.toString().c_str(), udpRemotePort);
        }
      }
    }
  }
  vTaskDelete(NULL);
}

// ===== RTCM RECEIVE TASK (UDP 2233 -> Serial2) =====
void rtcmReceiveTask(void* params) {
  uint8_t buffer[RTCM_PACKET_BUFFER_SIZE];

  while (true) {
    int packetSize = rtcmUdp.parsePacket();
    if (packetSize > 0) {
      while (packetSize > 0) {
        int bytesToRead = (packetSize > (int)sizeof(buffer)) ? sizeof(buffer) : packetSize;
        int bytesRead = rtcmUdp.read(buffer, bytesToRead);
        if (bytesRead <= 0) {
          break;
        }
        Serial2.write(buffer, bytesRead);
        packetSize -= bytesRead;
      }
    } else if (packetSize < 0) {
      // WiFiUDP can retain an invalid socket after a WiFi reconnect. Reopen it
      // instead of continuously reporting the same socket error.
      rtcmUdp.stop();
      vTaskDelay(pdMS_TO_TICKS(100));
      if (rtcmUdp.begin(RTCM_UDP_PORT)) {
        DEBUG_PRINTF("[RTCM] UDP socket reopened on port %d\n", RTCM_UDP_PORT);
      } else {
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}
