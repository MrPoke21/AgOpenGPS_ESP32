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
// Fixed remote port - AgOpenGPS always sends from this port, so replies
// always go to the same port. Only the client IP is learned dynamically.
extern const uint16_t udpRemotePort = 9999;
static volatile bool udpClientConnected = false;  // true once a client has sent a packet

static constexpr uint16_t RTCM_UDP_PORT = 2233;
static constexpr size_t RTCM_PACKET_BUFFER_SIZE = 1024;
static volatile bool udpSocketReady = false;

// UDP send queue
QueueHandle_t udpSendQueue = NULL;

// Forward declaration - defined at the end of this file
void wifiMonitorTask(void* params);

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
    WiFi.softAP(wifiRuntimeConfig.apSsid, wifiRuntimeConfig.apPass);

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
    // NO TIMEOUT: the attempt retries forever until connected (no AP fallback)
    DEBUG_PRINTLN("[UDP] Starting WiFi Station Mode (Higher Latency ~30-50ms, no timeout)");
    WiFi.mode(WIFI_STA);

    // Set TX power to maximum
    WiFi.setTxPower((wifi_power_t)WIFI_TX_POWER);
    wifi_config_t conf = {};
    esp_wifi_get_config(WIFI_IF_STA, &conf);
    conf.sta.listen_interval = 1;  // 1 beacon-intervallumonként ébredjen
    esp_wifi_set_config(WIFI_IF_STA, &conf);
    WiFi.begin(wifiRuntimeConfig.staSsid, wifiRuntimeConfig.staPass);

    wifiStatus = WIFI_STA_CONNECTING;

    // Végtelen csatlakozási ciklus - nincs időkorlát, amíg sikerül próbálkozik.
    // A WiFi driver disconnect eseményre magától újracsatlakozik; a ~30 mp-enkénti
    // WiFi.begin() újrakiadás a beragadt állapotokat is kezel (pl. a hálózat
    // eltűnt egy scan közben).
    uint32_t connectStartMs = millis();
    uint32_t lastBeginMs = millis();
    uint32_t lastProgressMs = millis();
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);

      if (millis() - lastBeginMs >= 30000) {
        lastBeginMs = millis();
        WiFi.begin(wifiRuntimeConfig.staSsid, wifiRuntimeConfig.staPass);
        DEBUG_PRINTLN("[UDP] Still connecting - re-issuing WiFi.begin()");
      }

      // A DEBUG uzenetek a webes /log oldalon jelennek meg (a debug engedelyezesevel).
      if (millis() - lastProgressMs >= 10000) {
        lastProgressMs = millis();
        DEBUG_PRINTF("[UDP] Connecting to \"%s\"... %lu s\n",
                      wifiRuntimeConfig.staSsid, (unsigned long)((millis() - connectStartMs) / 1000));
      }

      DEBUG_PRINT(".");
    }

    // A ciklus csak sikeres csatlakozáskor lép ki - többé nincs timeout / AP fallback
    DEBUG_PRINTLN("\n[UDP] WiFi Connected!");
    DEBUG_PRINT("[UDP] IP: ");
    DEBUG_PRINTLN(WiFi.localIP());
    wifiStatus = WIFI_STA_CONNECTED;
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);

    // Runtime watchdog for later drops (see wifiMonitorTask) - refreshes the
    // status and keeps reconnecting forever while the STA link is down
    xTaskCreatePinnedToCore(
      wifiMonitorTask,
      "wifiMon",
      2048,
      NULL,
      1,   // Low priority - monitoring only
      NULL,
      0    // Core 0 (WiFi stack)
    );

    return true;
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

  // STA link down: drop early instead of queueing - keeps the queue clear,
  // avoids the per-packet "queue full" error flood on Serial, and lets the
  // WiFi monitor task handle the reconnect. (In AP mode the link is always up.)
  if (wifiRuntimeConfig.mode == 0 && WiFi.status() != WL_CONNECTED) {
    return false;
  }

  // No client connected yet
  if (!udpClientConnected) {
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
    // Throttled: this fires per packet (up to 50 Hz) while the link is down
    static uint32_t lastQueueFullLog = 0;
    if (millis() - lastQueueFullLog >= 5000) {
      lastQueueFullLog = millis();
      DEBUG_PRINTLN("[SEND] ERROR: UDP queue full or no client");
    }
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

  // Learn the client IP for the replies. The remote port is FIXED
  // (udpRemotePort = 9999 - AgOpenGPS always sends from this port), so only
  // the IP is updated whenever the sender changes: after a WiFi outage (the
  // ESP can get a new IP) the previously learned address would be stale and
  // every reply would be silently lost. Only one AgOpenGPS client is
  // expected on this port, so "last sender wins" is safe here.
  IPAddress senderIP = udp.remoteIP();
  if (!udpClientConnected || senderIP != udpRemoteIP) {
    udpRemoteIP = senderIP;
    udpClientConnected = true;
    DEBUG_PRINTF("[UDP] Client: %s:%d\n", udpRemoteIP.toString().c_str(), udpRemotePort);
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
  static uint32_t lastFailLogMs = 0;  // throttle: max one failure log / 5 s

  while (1) {
    // Wait for packet in queue (1000ms timeout)
    if (xQueueReceive(udpSendQueue, &packet, pdMS_TO_TICKS(1000))) {
      // While the STA link is down, DISCARD the queued packets instead of
      // hammering lwIP: every failed sendto would print a lwIP error on Serial
      // (up to 50 packets/s), flooding the UART and stalling high-priority
      // tasks. Stale state is worthless for AgOpenGPS anyway - the first
      // packet after recovery carries fresh data.
      if (wifiRuntimeConfig.mode == 0 && WiFi.status() != WL_CONNECTED) {
        continue;
      }
      // Only send if we have a connected client
      if (udpClientConnected && wifiStatus != WIFI_ERROR) {
        bool success = false;
        // Unicast to the client IP with the fixed remote port
        // (udpRemotePort = 9999, AgOpenGPS always listens on this port).
        if (udp.beginPacket(udpRemoteIP, udpRemotePort)) {
          udp.write(packet.data, packet.length);
          success = udp.endPacket();
        }
        if (!success && millis() - lastFailLogMs >= 5000) {
          lastFailLogMs = millis();
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

// ===== WIFI MONITOR TASK (STA reconnect watchdog, Core 0) =====
// The WiFi driver auto-reconnects on most drop reasons (_autoReconnect is
// true by default), but not on all of them (e.g. authentication failures).
// This task keeps the connection state fresh for the rest of the firmware
// and re-issues the connect every 30 s while down - STA mode retries forever,
// there is no timeout and no automatic AP fallback.
void wifiMonitorTask(void* params) {
  bool wasConnected = false;
  uint32_t lastRetryMs = 0;

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(2000));

    if (wifiRuntimeConfig.mode != 0) {
      continue;  // AP mode - the link is ours, nothing to monitor
    }

    bool connected = (WiFi.status() == WL_CONNECTED);

    if (connected && !wasConnected) {
      wifiStatus = WIFI_STA_CONNECTED;
      DEBUG_PRINTF("[WIFI] STA %s, IP: %s\n",
                    wasConnected ? "ujracsatlakozott" : "csatlakozva",
                    WiFi.localIP().toString().c_str());
    } else if (!connected) {
      wifiStatus = WIFI_STA_CONNECTING;
      if (millis() - lastRetryMs >= 30000) {
        lastRetryMs = millis();
        WiFi.reconnect();
        DEBUG_PRINTLN("[WIFI] STA kapcsolat nincs meg - ujracsatlakozas...");
      }
    }

    wasConnected = connected;
  }
}
