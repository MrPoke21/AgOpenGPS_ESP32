#ifndef ZUDP_H
#define ZUDP_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "Configuration.h"

// UDP packet queue structure
struct UDPPacket {
  uint8_t data[256];
  uint16_t length;
};

// WiFi status enumeration
enum WiFiStatus {
  WIFI_INIT,
  WIFI_AP_READY,
  WIFI_STA_CONNECTING,
  WIFI_STA_CONNECTED,
  WIFI_ERROR
};

// ===== FUNCTION DECLARATIONS =====

/**
 * Initialize WiFi and UDP
 * @return true if successful
 */
bool initWiFi();

/**
 * Create the UDP send/receive queues - call once from setup() BEFORE
 * creating autoSteerPacketPerser (which blocks on the receive queue),
 * so it's never handed a NULL queue.
 */
void initUDPQueues();

/**
 * Initialize UDP socket
 * @return true if successful
 */
bool initUDP();

/**
 * Forward incoming RTCM UDP packets from port 2233 to Serial2.
 */
void rtcmReceiveTask(void* params);

/**
 * Send data via UDP (non-blocking queue-based)
 * @param data: pointer to data buffer
 * @param length: data length
 * @return true if queued successfully
 */
bool sendUDP(const uint8_t* data, uint16_t length);

/**
 * UDP send task - processes queued packets (run in separate task)
 */
void udpSendTask(void* params);

/**
 * Send an ASCII NMEA sentence to Serial or UDP depending on configuration
 * @param data: pointer to data buffer
 * @param length: data length
 */
void sendNMEA(const uint8_t* data, uint16_t length);

/**
 * Receive UDP data - blocks until a packet arrives
 * @param buffer: pointer to receive buffer
 * @param maxLen: max buffer length
 * @return number of bytes received
 */
uint16_t receiveUDP(uint8_t* buffer, uint16_t maxLen);

/**
 * Get WiFi status
 * @return WiFi status enum
 */
WiFiStatus getWiFiStatus();

/**
 * Get connected client count (for AP mode)
 * @return number of connected clients
 */
uint8_t getWiFiClientCount();

/**
 * Print WiFi status to Serial
 */
void printWiFiStatus();

extern WiFiStatus wifiStatus;
extern WiFiUDP udp;
extern IPAddress udpRemoteIP;
extern uint16_t udpRemotePort;

#endif
