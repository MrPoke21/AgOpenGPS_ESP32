#ifndef PGNBUILDER_H
#define PGNBUILDER_H

#include <Arduino.h>
#include <stddef.h>

// ===== Parsed packet payloads (RX) - declared before PgnBuilder =====

/** PGN 254 - AutoSteer Data from the host. */
struct AutoSteerData {
  uint16_t speedX10 = 0;        // byte 5-6 (LE): speed km/h * 10
  uint8_t status = 0;           // byte 7: STATUS_* bits
  int16_t steerAngleX100 = 0;   // byte 8-9 (LE): steer angle setpoint * 100
  int8_t xte = 0;               // byte 10: cross-track error cm (classic: tram)
  uint16_t sections = 0;        // byte 11-12 (LE): SC1to8/SC9to16 (classic: relay/relayHi)
};

/** PGN 239 - Machine Data from the host. */
struct MachineData {
  uint8_t uturn = 0;            // byte 5: U-turn state
  uint8_t speedX10 = 0;         // byte 6: speed km/h * 10 (single byte, max 25.5)
  uint8_t hydLift = 0;          // byte 7: hydraulic lift state
  uint8_t tram = 0;             // byte 8: tramline state
  uint8_t geoStop = 0;          // byte 9: geo-fence stop
  uint8_t reserved = 0;         // byte 10
  uint16_t sections = 0;        // byte 11-12 (LE): SC1to8/SC9to16
};

/** PGN 229 - 64 sections + L/R speed from the host. */
struct Sections64Data {
  uint8_t sectionBytes[8] = {0};  // byte 5-12: SC1to8 .. SC57to64 (section 1 = bit 0)
  uint64_t sectionStates = 0;     // all 64 section bits, little-endian assembly
  uint8_t lSpeedX10 = 0;          // byte 13: left speed km/h * 10 (clamped 0-255)
  uint8_t rSpeedX10 = 0;          // byte 14: right speed km/h * 10 (clamped 0-255)
};

/** PGN 252 - Steer Settings from the host.
 *  Defaults match the classic firmware's Storage defaults (active until the
 *  host sends PGN 252; the whole struct is persisted to EEPROM). */
struct SteerSettingsData {
  uint8_t gainP = 40;             // byte 5: proportional gain (1-100)
  uint8_t highPwm = 60;           // byte 6: max PWM
  uint8_t lowPwm = 10;            // byte 7: low PWM (highPWM / 3)
  uint8_t minPwm = 9;             // byte 8: minimum PWM to move
  uint8_t countsPerDegree = 30;   // byte 9: WAS counts per degree
  int16_t wasOffset = 0;          // byte 10-11 (LE): WAS zero offset
  uint8_t ackermannX100 = 100;    // byte 12: ackermann percent * 100 (1.00x)
};

/** PGN 251 - Steer Config from the host.
 *  Defaults match the classic firmware's Setup defaults (active until the
 *  host sends PGN 251; the whole struct is persisted to EEPROM). */
struct SteerConfigData {
  uint8_t set0 = 0;               // byte 5: raw setting byte 0
  uint8_t pulseCountMax = 5;      // byte 6
  uint8_t minSpeedX10 = 0;        // byte 7: min steer speed * 10
  uint8_t set1 = 0;               // byte 8: raw setting byte 1
  uint8_t angularVelocity = 0;    // byte 9
  // decoded set0 (same bit layout the classic firmware bitRead()s)
  bool invertWas = false;             // bit 0
  bool isRelayActiveHigh = false;     // bit 1
  bool motorDriveDirection = false;   // bit 2
  bool singleInputWas = true;         // bit 3 (classic default: AD converter WAS)
  bool cytronDriver = true;           // bit 4 (classic default: Cytron driver)
  bool steerSwitch = false;           // bit 5
  bool steerButton = false;           // bit 6
  bool shaftEncoder = false;          // bit 7
  // decoded set1
  bool isDanfoss = false;             // bit 0
  bool pressureSensor = false;        // bit 1
  bool currentSensor = false;         // bit 2
  bool isUseYAxis = false;            // bit 3
};

/** PGN 238 - Machine Config from the host. */
struct MachineConfigData {
  uint8_t raiseTime = 0;          // byte 5: raise time (seconds)
  uint8_t lowerTime = 0;          // byte 6: lower time (seconds)
  uint8_t hydraulicLiftEnabled = 0;  // byte 7: enable hydraulic (0/1)
  uint8_t set0 = 0;               // byte 8: bit0 InvertRelay, bit1 HydEnabled
  uint8_t user1 = 0;              // byte 9: user value 1 (0-255)
  uint8_t user2 = 0;              // byte 10: user value 2 (0-255)
  uint8_t user3 = 0;              // byte 11: user value 3 (0-255)
  uint8_t user4 = 0;              // byte 12: user value 4 (0-255)
};

/** PGN 236 - Machine Pin Config from the host. */
struct MachinePinsData {
  uint8_t pins[24] = {0};         // byte 5-28: pin 0-23 function assignments
};

/** PGN 201 - Set Subnet broadcast from the host. */
struct SubnetChange {
  uint8_t octet1 = 0;             // byte 7: new first IP octet
  uint8_t octet2 = 0;             // byte 8: new second IP octet
  uint8_t octet3 = 0;             // byte 9: new third IP octet
};

/**
 * Per-packet PGN build/parse functions for the AgOpenGPS binary protocol.
 *
 * C++ (module-side) port of the AgOpenWeb reference implementation:
 *   Shared/AgOpenWeb.Services/AutoSteer/PgnBuilder.cs
 *   (AgOpenGPS-Official/AgOpenWeb @ 11ed934)
 *
 * The build/parse directions are mirrored to THIS module's perspective:
 *   - Build* methods assemble the packets the board SENDS
 *     (PGN 253, 250, 126, 121, 203), each with a computed CRC.
 *   - TryParse* methods decode the packets the board RECEIVES
 *     (PGN 254, 239, 229, 252, 251, 238, 236, 200, 202, 201).
 *
 * Wire format follows the AgOpenGPS standard:
 *   [0x80, 0x81, Source, PGN, Length, Data..., CRC]
 * where CRC is the sum of bytes [2 .. len-2] (source through the last data
 * byte) - the same rule parsePacket() in zPackets.cpp verifies.
 *
 * The builders write into per-PGN static buffers (zero heap allocations,
 * same pattern as the reference's thread-local buffers). Call them from a
 * single task per packet type - they are not re-entrant.
 *
 * zPackets.cpp routes every packet through this class, and the runtime
 * settings globals in main.h (steerSettings / steerConfig) ARE the parsed
 * SteerSettingsData / SteerConfigData payloads - no duplicated copies.
 */
class PgnBuilder {
 public:
  // ===== Standard AgOpenGPS header =====
  static const uint8_t HEADER1 = 0x80;
  static const uint8_t HEADER2 = 0x81;
  static const uint8_t SOURCE = 0x7F;  // AgIO/AgOpenGPS source (packets we receive)

  // Module source addresses (packets we send)
  static const uint8_t SOURCE_AUTO_STEER = 126;  // 0x7E
  static const uint8_t SOURCE_IMU = 121;         // 0x79
  static const uint8_t SOURCE_GPS = 120;         // 0x78

  // ===== PGN identifiers =====
  static const uint8_t PGN_AUTOSTEER = 0xFE;        // 254 - AutoSteer Data (RX)
  static const uint8_t PGN_MACHINE = 0xEF;          // 239 - Machine Data (RX)
  static const uint8_t PGN_SECTIONS_64 = 0xE5;      // 229 - 64-section on/off + L/R speed (RX)
  static const uint8_t PGN_STEER_SETTINGS = 0xFC;   // 252 - Steer Settings (RX)
  static const uint8_t PGN_STEER_CONFIG = 0xFB;     // 251 - Steer Config (RX)
  static const uint8_t PGN_MACHINE_CONFIG = 0xEE;   // 238 - Machine Config (RX)
  static const uint8_t PGN_MACHINE_PINS = 0xEC;     // 236 - Machine Pin Config (RX)
  static const uint8_t PGN_STEER_DATA = 0xFD;       // 253 - Steer Data FROM Module (TX)
  static const uint8_t PGN_SENSOR_DATA = 0xFA;      // 250 - Sensor Data FROM Module (TX)
  static const uint8_t PGN_HELLO_FROM_AGIO = 200;   // 0xC8 - "hello" from the host (RX)
  static const uint8_t PGN_SET_SUBNET = 201;        // 0xC9 - set subnet broadcast (RX)
  static const uint8_t PGN_SCAN_REQUEST = 202;      // 0xCA - scan request broadcast (RX)
  static const uint8_t PGN_SCAN_REPLY = 203;        // 0xCB - scan reply with IP/subnet (TX)
  static const uint8_t PGN_HELLO_AUTO_STEER = 126;  // 0x7E - hello from AutoSteer module (TX)
  static const uint8_t PGN_HELLO_IMU = 121;         // 0x79 - hello from IMU module (TX)

  // Packet sizes: header(2) + source(1) + pgn(1) + length(1) + data(N) + crc(1)
  static const uint8_t STEER_DATA_PGN_SIZE = 14;       // 5 header + 8 data + 1 crc (253)
  static const uint8_t SENSOR_DATA_PGN_SIZE = 14;      // 5 header + 8 data + 1 crc (250)
  static const uint8_t HELLO_AUTO_STEER_PGN_SIZE = 11; // 5 header + 5 data + 1 crc (126)
  static const uint8_t HELLO_IMU_PGN_SIZE = 11;        // 5 header + 5 data + 1 crc (121)
  static const uint8_t SCAN_REPLY_PGN_SIZE = 13;       // 5 header + 7 data + 1 crc (203)
  static const uint8_t AUTOSTEER_PGN_SIZE = 14;        // 5 header + 8 data + 1 crc (254)
  static const uint8_t MACHINE_PGN_SIZE = 14;          // 5 header + 8 data + 1 crc (239)
  static const uint8_t SECTIONS_64_PGN_SIZE = 16;      // 5 header + 10 data + 1 crc (229)
  static const uint8_t STEER_SETTINGS_PGN_SIZE = 14;   // 5 header + 8 data + 1 crc (252)
  static const uint8_t STEER_CONFIG_PGN_SIZE = 11;     // 5 header + 5 data + 1 crc (251)
  static const uint8_t MACHINE_CONFIG_PGN_SIZE = 14;   // 5 header + 8 data + 1 crc (238)
  static const uint8_t MACHINE_PINS_PGN_SIZE = 30;     // 5 header + 24 data + 1 crc (236)
  static const uint8_t HELLO_AGIO_PGN_SIZE = 9;        // 5 header + 3 data + 1 crc (200)
  static const uint8_t SCAN_REQUEST_PGN_SIZE = 9;      // 5 header + 3 data + 1 crc (202)
  static const uint8_t SUBNET_CHANGE_PGN_SIZE = 11;    // 5 header + 5 data + 1 crc (201)

  // PGN 254 status byte (byte 7) bits - as built by the AgOpenWeb host
  static const uint8_t STATUS_STEER_SWITCH = 0x01;
  static const uint8_t STATUS_WORK_SWITCH = 0x02;
  static const uint8_t STATUS_AUTO_STEER_ENGAGED = 0x04;
  static const uint8_t STATUS_GPS_VALID = 0x08;
  static const uint8_t STATUS_GUIDANCE_VALID = 0x10;

  // PGN 253 switch byte (byte 11) bits - as parsed by the AgOpenWeb host.
  // NOTE: the classic firmware's switchByte uses bit0 = steer switch,
  // bit1 = work switch; reconcile during the packet migration.
  static const uint8_t SWITCH_WORK_ACTIVE = 0x01;    // inverted: 1 = work switch OFF
  static const uint8_t SWITCH_STEER_ENABLED = 0x02;
  static const uint8_t SWITCH_REMOTE_BUTTON = 0x04;
  static const uint8_t SWITCH_VWAS_FUSION = 0x20;

  // Placeholder IMU values the current firmware sends in PGN 253 bytes 7-10
  // (0x0F,0x27 -> 9999 -> 999.9 deg, 0xB8,0x22 -> 8888 -> 888.8 deg)
  static const int16_t PLACEHOLDER_HEADING_X10 = 9999;
  static const int16_t PLACEHOLDER_ROLL_X10 = 8888;

  // ===== TX builders (packets the board sends) =====

  /**
   * Build PGN 253 (Steer Data FROM Module).
   * Format: [0x80, 0x81, 126, 0xFD, 8, AngleLo, AngleHi, HeadingLo, HeadingHi,
   *          RollLo, RollHi, Switches, PWM, CRC]
   *
   * Byte 5-6:  actual steer angle * 100 (signed int16, little-endian)
   * Byte 7-8:  heading * 10 (signed int16, little-endian)
   * Byte 9-10: roll * 10 (signed int16, little-endian)
   * Byte 11:   switch status byte (see SWITCH_* bits)
   * Byte 12:   PWM display (0-255)
   * Byte 13:   CRC
   *
   * Returns a pointer to an internal static buffer of STEER_DATA_PGN_SIZE bytes.
   */
  static uint8_t* BuildSteerDataPgn(float steerAngleActual, float imuHeading,
                                    float imuRoll, uint8_t switches,
                                    uint8_t pwmDisplay);

  /**
   * Build PGN 250 (Sensor Data FROM Module).
   * Format: [0x80, 0x81, 126, 0xFA, 8, Sensor, 0, 0, 0, 0, 0, 0, 0, CRC]
   *
   * Byte 5: raw sensor reading (pressure/current sensor; interpretation
   *         depends on the hardware config). Bytes 6-12 reserved (0).
   *
   * Returns a pointer to an internal static buffer of SENSOR_DATA_PGN_SIZE bytes.
   */
  static uint8_t* BuildSensorDataPgn(uint8_t sensorValue);

  /**
   * Build PGN 126 - "hello" reply from the AutoSteer module. Sent whenever a
   * PGN 200 hello arrives from the host.
   * Format: [0x80, 0x81, 126, 126, 5, AngleLo, AngleHi, PosLo, PosHi, SwitchByte, CRC]
   *
   * Byte 5-6: actual steer angle * 100 (signed int16, little-endian)
   * Byte 7-8: steer position (WAS counts, signed int16, little-endian)
   * Byte 9:   switch byte (steer/work switch states)
   *
   * Returns a pointer to an internal static buffer of HELLO_AUTO_STEER_PGN_SIZE bytes.
   */
  static uint8_t* BuildHelloFromAutoSteerPgn(float steerAngleActual,
                                             int16_t steerPosition,
                                             uint8_t switchByte);

  /**
   * Build PGN 121 - "hello" reply from the IMU module. Sent right after the
   * PGN 126 reply when a BNO08x IMU is present.
   * Format: [0x80, 0x81, 121, 121, 5, 0, 0, 0, 0, 0, CRC]
   *
   * Returns a pointer to an internal static buffer of HELLO_IMU_PGN_SIZE bytes.
   */
  static uint8_t* BuildHelloFromImuPgn();

  /**
   * Build PGN 203 - scan reply. Answers a PGN 202 scan request with this
   * module's identity, IP address and subnet.
   * Format: [0x80, 0x81, ModuleId, 203, 7, IP0, IP1, IP2, IP3, Sub0, Sub1, Sub2, CRC]
   *
   * Byte 2:    module id (126 = AutoSteer module)
   * Byte 5-8:  local IPv4 address (4 octets)
   * Byte 9-11: subnet / remote host, first 3 octets
   *
   * ipAddress must point to 4 bytes, subnet to 3 bytes (NULL -> zeros).
   * Returns a pointer to an internal static buffer of SCAN_REPLY_PGN_SIZE bytes.
   */
  static uint8_t* BuildScanReplyPgn(uint8_t moduleId, const uint8_t* ipAddress,
                                    const uint8_t* subnet);

  // ===== RX parsers (packets the board receives) =====

  /**
   * Parse PGN 254 (AutoSteer Data) received from the host.
   * Format per PGN 5.6 spec:
   *   [0x80, 0x81, 0x7F, 0xFE, 8, Speed(2), Status, SteerAngle(2), XTE, SC1-8, SC9-16, CRC]
   *
   * Byte 5-6:  speed * 10 km/h (little-endian)
   * Byte 7:    status (see STATUS_* bits; the classic firmware reads bit 0 as steerEnable)
   * Byte 8-9:  steer angle setpoint * 100 (signed, little-endian)
   * Byte 10:   cross-track error in cm (-127..127); classic firmware: tram
   * Byte 11-12: section bits SC1to8 / SC9to16; classic firmware: relay / relayHi
   *
   * Returns true if the packet is a well-formed PGN 254.
   */
  static bool TryParseAutoSteerData(const uint8_t* data, size_t len, AutoSteerData& out);

  /**
   * Parse PGN 239 (Machine Data) received from the host.
   * Format: [0x80, 0x81, 0x7F, 0xEF, 8, uturn, speed*10, hydLift, Tram, GeoStop,
   *          reserved, SC1-8, SC9-16, CRC]
   *
   * Returns true if the packet is a well-formed PGN 239.
   */
  static bool TryParseMachineData(const uint8_t* data, size_t len, MachineData& out);

  /**
   * Parse PGN 229 (64 sections + L/R speed) received from the host. Sent in
   * addition to PGN 239 when more than 16 sections are configured.
   * Format: [0x80, 0x81, 0x7F, 0xE5, 10, SC1-8, SC9-16, SC17-24, SC25-32,
   *          SC33-40, SC41-48, SC49-56, SC57-64, Lspeed, Rspeed, CRC]
   *
   * Returns true if the packet is a well-formed PGN 229.
   */
  static bool TryParseSections64(const uint8_t* data, size_t len, Sections64Data& out);

  /**
   * Parse PGN 252 (Steer Settings) received from the host.
   * Format: [0x80, 0x81, 0x7F, 0xFC, 8, gainP, highPWM, lowPWM, minPWM,
   *          countsPerDeg, offsetLo, offsetHi, ackerman, CRC]
   *
   * Byte 5:  proportional gain (1-100)
   * Byte 6:  high PWM limit (max PWM)
   * Byte 7:  low PWM limit (highPWM / 3)
   * Byte 8:  minimum PWM to move
   * Byte 9:  counts per degree (1-255)
   * Byte 10-11: WAS offset (signed 16-bit, little-endian)
   * Byte 12: ackermann correction percent * 100 (0-200)
   *
   * Returns true if the packet is a well-formed PGN 252.
   */
  static bool TryParseSteerSettings(const uint8_t* data, size_t len, SteerSettingsData& out);

  /**
   * Parse PGN 251 (Steer Config) received from the host.
   * Format: [0x80, 0x81, 0x7F, 0xFB, 5, set0, pulseCount, minSpeed, set1, angVel, CRC]
   *
   * Byte 5 (set0): bit0 Invert WAS, bit1 Invert Relays (IsRelayActiveHigh),
   *                bit2 Invert Motor (MotorDriveDirection), bit3 AD converter
   *                (SingleInputWAS), bit4 Motor driver (Cytron), bit5 Steer
   *                switch, bit6 Steer button, bit7 Shaft/Turn sensor
   * Byte 6:  pulse count max
   * Byte 7:  min steer speed * 10
   * Byte 8 (set1): bit0 Danfoss, bit1 Pressure sensor, bit2 Current sensor,
   *                bit3 Use Y axis
   * Byte 9:  angular velocity
   *
   * Returns true if the packet is a well-formed PGN 251.
   */
  static bool TryParseSteerConfig(const uint8_t* data, size_t len, SteerConfigData& out);

  /**
   * Parse PGN 238 (Machine Config) received from the host.
   * Format: [0x80, 0x81, 0x7F, 0xEE, 8, raiseTime, lowerTime, hydEnable, set0,
   *          user1, user2, user3, user4, CRC]
   *
   * Returns true if the packet is a well-formed PGN 238.
   */
  static bool TryParseMachineConfig(const uint8_t* data, size_t len, MachineConfigData& out);

  /**
   * Parse PGN 236 (Machine Pin Config) received from the host.
   * Format: [0x80, 0x81, 0x7F, 0xEC, 24, Pin0..Pin23, CRC]
   *
   * Returns true if the packet is a well-formed PGN 236.
   */
  static bool TryParseMachinePins(const uint8_t* data, size_t len, MachinePinsData& out);

  /**
   * Parse PGN 200 - "hello" from AgIO. The host broadcasts this so the modules
   * can confirm they are alive (modules answer with PGN 126 / PGN 121).
   * Format: [0x80, 0x81, 0x7F, 200, 3, 56, 0, 0, CRC]
   *
   * Returns true if the packet is a well-formed PGN 200 hello (magic byte 56
   * at data[5], matching the host's BuildHelloPacket layout).
   */
  static bool TryParseHelloFromAgIo(const uint8_t* data, size_t len);

  /**
   * Parse PGN 202 - scan request broadcast. Modules answer with a PGN 203
   * scan reply carrying their IP/subnet.
   * Format: [0x80, 0x81, 0x7F, 202, 3, 202, 202, 5, CRC]
   *
   * Returns true if the packet is a well-formed PGN 202 scan request
   * (magic bytes 202, 202 at data[5]/data[6], same check the classic
   * firmware performs).
   */
  static bool TryParseScanRequest(const uint8_t* data, size_t len);

  /**
   * Parse PGN 201 - set subnet broadcast. Changes the first three IP octets
   * (the /24) on ALL modules at once; the host octet is preserved per module.
   * Format: [0x80, 0x81, 0x7F, 201, 5, 201, 201, octet1, octet2, octet3, CRC]
   *
   * Returns true if the packet is a well-formed PGN 201.
   */
  static bool TryParseSubnetChange(const uint8_t* data, size_t len, SubnetChange& out);

  // ===== Helpers =====

  /**
   * Validate a received packet checksum using the same rule the send path
   * uses: the trailing byte is the sum of bytes [2 .. len-2].
   *
   * Diagnostics only - do NOT gate the inbound path on this: some modules
   * ship packets carrying a placeholder CRC, so rejecting those packets
   * would take the whole link down (same caveat as in the reference).
   */
  static bool ValidateChecksum(const uint8_t* data, size_t len);

 private:
  PgnBuilder() {}  // static-only class

  /** Sum of data[start .. start+length-1] as uint8 (overflow wraps). */
  static uint8_t CalculateCrc(const uint8_t* data, int start, int length);

  /** Stamp the trailing CRC into a fully-populated packet, in place. */
  static uint8_t* StampCrc(uint8_t* packet, size_t length);
};

#endif  // PGNBUILDER_H
