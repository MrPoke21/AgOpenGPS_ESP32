
#include <Arduino.h>
#include <Configuration.h>
#include <zPackets.h>
#include <zUDP.h>
#include <zWebConfig.h>
#include <main.h>
#include <AutosteerPID.h>
#include <PgnBuilder.h>
#ifdef SPEED_IMPULSE_ENABLED
  #include <zSpeedImpulse.h>
#endif

// Buffer size definitions
#define BUFFER_SIZE 1024
#define MAX_PACKET_SIZE 1024
#define MAX_PACKET_DATA_SIZE (MAX_PACKET_SIZE - 6)  // Header + checksum

// Forward declaration
void processPacketBytes(byte* dataBuffer, uint16_t dataLen);

//uint8_t data[128];
byte buffer[BUFFER_SIZE];
byte packetBuffer[MAX_PACKET_SIZE];
int stateIndex = 0;
int totalHeaderByteCount = 5;
int count;


void autoSteerPacketPerser(void *pvParameters) {
  while (1) {
#if ENABLE_UDP == 1
    // Blocks until a packet arrives - event-driven, no polling delay
    uint16_t udpLen = receiveUDP((uint8_t*)buffer, BUFFER_SIZE);
    if (udpLen > 0) {
      DEBUG_PRINTF("[PKT] Received %d bytes from UDP\n", udpLen);
      processPacketBytes(buffer, udpLen);
    }
#else
    // Process Serial data only - non-blocking byte-by-byte read
    int aas = Serial.available();
    if (aas > 0) {
      int readCount = (aas > 64) ? 64 : aas;
      DEBUG_PRINT("[PKT] Received ");
      DEBUG_PRINT(readCount);
      DEBUG_PRINTLN(" bytes from Serial");
      for (int i = 0; i < readCount; i++) {
        byte b = Serial.read();
        buffer[i] = b;
      }
      processPacketBytes(buffer, readCount);
    }
     vTaskDelay(pdMS_TO_TICKS(1));  // Prevent watchdog reset, yield to other tasks
#endif
  }
}

// Helper function to process byte stream from either Serial or UDP
void processPacketBytes(byte* dataBuffer, uint16_t dataLen) {
  byte a;
  for (int i = 0; i < dataLen; i++) {
    a = dataBuffer[i];
    
    // Prevent buffer overflow
    if (stateIndex >= MAX_PACKET_SIZE - 1) {
      DEBUG_PRINTLN("ERROR: Packet buffer overflow");
      stateIndex = 0;
      continue;
    }

    switch (stateIndex) {
      case 0:  //find 0x80
        {
          if (a == 128) packetBuffer[stateIndex++] = a;
          else stateIndex = 0;
          break;
        }

      case 1:  //find 0x81
        {
          if (a == 129) packetBuffer[stateIndex++] = a;
          else {
            if (a == 181) {
              stateIndex = 0;
              packetBuffer[stateIndex++] = a;
            } else stateIndex = 0;
          }
          break;
        }
      case 2:  //Source Address (7F)
        {
          if (a < 128 && a > 120)
            packetBuffer[stateIndex++] = a;
          else stateIndex = 0;
          break;
        }
      case 3:  //PGN ID
      case 4:  //Num of data bytes
        {
          packetBuffer[stateIndex++] = a;
          break;
        }
      default:  //Data load and Checksum
        {
          if (stateIndex > 4) {
            int length = packetBuffer[4] + 6;
            packetBuffer[stateIndex++] = a;
            if (stateIndex < length) {
              break;
            } else {
              DEBUG_PRINT("[PKT] Complete packet: length=");
              DEBUG_PRINT(length);
              DEBUG_PRINT(", PGN=0x");
              DEBUG_PRINTLN(packetBuffer[3], HEX);
              parsePacket(packetBuffer, length);
              //clear out the current pgn
              stateIndex = 0;
              break;
            }
          }
          break;
        }
    }
  }
}

void parsePacket(byte* packet, int size) {
  // Validate packet size
  if (size < 6 || size > MAX_PACKET_SIZE || packet == NULL) {
    DEBUG_PRINT("[PKT] ERROR: Invalid packet size: ");
    DEBUG_PRINTLN(size);
    return;
  }
  
  DEBUG_PRINT("[PKT] Parsing packet: size=");
  DEBUG_PRINT(size);
  DEBUG_PRINT(", PGN=0x");
  DEBUG_PRINTLN((size > 3) ? packet[3] : 0, HEX);
  
  if (packet[0] == 128 && packet[1] == 129) {
    int length = packet[4] + 6;
    
    // Bounds check
    if (length < 6 || length > MAX_PACKET_SIZE || length != size) {
      DEBUG_PRINT("ERROR: Packet length mismatch: expected ");
      DEBUG_PRINT(length);
      DEBUG_PRINT(" got ");
      DEBUG_PRINTLN(size);
      return;
    }

    // Checksum verification via PgnBuilder (sum of bytes 2..length-2)
    if (!PgnBuilder::ValidateChecksum(packet, (size_t)length)) {
      DEBUG_PRINTLN("ERROR: Checksum mismatch");
      printLnByteArray(packet, size);
      return;
    }
  }

  if (packet[0] == 0x80 && packet[1] == 0x81 && packet[2] == 0x7F)  //Data
  {
    lastFEPacketTime = millis();
    int packetLength = packet[4] + 6;
    
    switch (packet[3]) {
      case 0xFE:
        {
          if (packetLength < 13) {
            break;
          }

          // Parse the incoming AutoSteer Data (PGN 254) via PgnBuilder
          AutoSteerData steerData;
          if (!PgnBuilder::TryParseAutoSteerData(packet, (size_t)packetLength, steerData)) {
            break;
          }

          gpsSpeed = ((float)steerData.speedX10) * 0.1;
          #ifdef SPEED_IMPULSE_ENABLED
            setSpeedKmh(gpsSpeed);
          #endif
          prevGuidanceStatus = guidanceStatus;
          guidanceStatus = steerData.status;
          guidanceStatusChanged = (guidanceStatus != prevGuidanceStatus);

          //Bit 8,9    set point steer angle * 100 is sent
          steerAngleSetPoint = ((float)steerData.steerAngleX100) * 0.01;  //high low bytes

          byte guidanceBit = bitRead(guidanceStatus, 0);
          
          steerEnable = (guidanceBit != 0);
          
          // Only update steerEnable if the condition actually changed (prevent rapid toggling)
          if (steerEnable != prevSteerEnableCondition) {
            prevSteerEnableCondition = steerEnable;
          }
          
          //Bit 10 Tram / cross-track error
          tram = (uint8_t)steerData.xte;
          //Bit 11: section bits 1-8 (classic: relay)
          relay = (uint8_t)(steerData.sections & 0xFF);
          //Bit 12: section bits 9-16 (classic: relayHi)
          relayHi = (uint8_t)(steerData.sections >> 8);
          //----------------------------------------------------------------------------
          //Serial Send to agopenGPS
          // Reply with the current steer state (PGN 253, CRC computed by the builder).
          // Heading/roll keep the placeholder values of the classic firmware
          // (999.9 deg / 888.8 deg) until real IMU values are wired in.
          sendData(PgnBuilder::BuildSteerDataPgn(
                       steerAngleActual,
                       PgnBuilder::PLACEHOLDER_HEADING_X10,
                       PgnBuilder::PLACEHOLDER_ROLL_X10,
                       switchByte, pwmDisplay),
                   PgnBuilder::STEER_DATA_PGN_SIZE);

          //Steer Data 2 -------------------------------------------------
          if (steerConfig.pressureSensor || steerConfig.currentSensor) {
            if (aog2Count++ > 2) {
              //Send fromAutosteer2 (PGN 250)
              sendData(PgnBuilder::BuildSensorDataPgn((uint8_t)sensorReading),
                       PgnBuilder::SENSOR_DATA_PGN_SIZE);
              aog2Count = 0;
            }
          }
          break;
        }
      //steer settings
      case 252:
        {  //0xFC
          // Parse the incoming Steer Settings (PGN 252) directly into the
          // runtime settings struct (steerSettings IS the parsed payload)
          if (!PgnBuilder::TryParseSteerSettings(packet, (size_t)packetLength, steerSettings)) {
            break;
          }

          // NOTE: kept from the original parser - lowPWM ends up as minPWM
          steerSettings.lowPwm = steerSettings.minPwm;

          //crc
          //autoSteerUdpData[13];

          //store in EEPROM
          EEPROM.put(10, steerSettings);
          EEPROM.commit();
          // for PWM High to Low interpolator
          highLowPerDeg = ((float)(steerSettings.highPwm - steerSettings.lowPwm)) / LOW_HIGH_DEGREES;
          break;
        }
      case 251:  //251 FB - SteerConfig
        {
          // Parse the incoming Steer Config (PGN 251) directly into the
          // runtime config struct (steerConfig IS the parsed payload)
          if (!PgnBuilder::TryParseSteerConfig(packet, (size_t)packetLength, steerConfig)) {
            break;
          }

          //crc
          //autoSteerUdpData[13];

          EEPROM.put(40, steerConfig);
          EEPROM.commit();
          // Re-Init
          break;
        }
      case 200:
        {  // Hello from AgIO
          // Parse the incoming hello (PGN 200) via PgnBuilder
          if (!PgnBuilder::TryParseHelloFromAgIo(packet, (size_t)packetLength)) {
            break;
          }

          // Reply with the hello from AutoSteer (PGN 126, CRC computed by the builder)
          sendData(PgnBuilder::BuildHelloFromAutoSteerPgn(steerAngleActual, helloSteerPosition, switchByte),
                   PgnBuilder::HELLO_AUTO_STEER_PGN_SIZE);

          // Send IMU hello immediately after (no delay)
          if (useBNO08x) {
            sendData(PgnBuilder::BuildHelloFromImuPgn(), PgnBuilder::HELLO_IMU_PGN_SIZE);
          }
          break;
        }
      case 202:
        {
          // Make really sure this is the scan request pgn (via PgnBuilder)
          if (PgnBuilder::TryParseScanRequest(packet, (size_t)packetLength)) {
#if ENABLE_UDP
            // Build scan reply with local IP and remote subnet
            IPAddress myIP;

            // Get local IP address based on WiFi mode
            myIP = (wifiRuntimeConfig.mode == 1) ? WiFi.softAPIP() : WiFi.localIP();

            // Local IP bytes (5-8)
            const uint8_t localIp[4] = { (uint8_t)myIP[0], (uint8_t)myIP[1],
                                         (uint8_t)myIP[2], (uint8_t)myIP[3] };

            // Remote subnet bytes (9-11 - first 3 octets)
            const uint8_t remoteSubnet[3] = { (uint8_t)udpRemoteIP[0], (uint8_t)udpRemoteIP[1],
                                              (uint8_t)udpRemoteIP[2] };

            // PGN 203 scan reply, CRC computed by the builder
            sendData(PgnBuilder::BuildScanReplyPgn(PgnBuilder::SOURCE_AUTO_STEER, localIp, remoteSubnet),
                     PgnBuilder::SCAN_REPLY_PGN_SIZE);
#else
            // Serial-only build: no IP addresses available
            sendData(PgnBuilder::BuildScanReplyPgn(PgnBuilder::SOURCE_AUTO_STEER, NULL, NULL),
                     PgnBuilder::SCAN_REPLY_PGN_SIZE);
#endif
            DEBUG_PRINTLN("[PKT] Response sent: scanReply (0xCB)");
          }
          break;
        }
    }
  } else {
    DEBUG_PRINTLN("Unknown packet!!! : ");
    printLnByteArray(packet, size);
  }
}
