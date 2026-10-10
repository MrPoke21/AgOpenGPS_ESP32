#include <PgnBuilder.h>
#include <string.h>

// ===== TX builders =====

uint8_t* PgnBuilder::BuildSteerDataPgn(float steerAngleActual, float imuHeading,
                                       float imuRoll, uint8_t switches,
                                       uint8_t pwmDisplay) {
  // Per-PGN static buffer, zero heap allocations (PGN 253 mirror of PgnBuilder)
  static uint8_t buf[STEER_DATA_PGN_SIZE];

  // Header
  buf[0] = HEADER1;
  buf[1] = HEADER2;
  buf[2] = SOURCE_AUTO_STEER;
  buf[3] = PGN_STEER_DATA;
  buf[4] = 8;  // data length

  // Actual steer angle * 100 (signed, little-endian: low byte first)
  int16_t angleInt = (int16_t)(steerAngleActual * 100.0f);
  buf[5] = (uint8_t)(angleInt & 0xFF);         // low byte
  buf[6] = (uint8_t)((angleInt >> 8) & 0xFF);  // high byte

  // Heading * 10 (signed, little-endian)
  int16_t headingInt = (int16_t)(imuHeading * 10.0f);
  buf[7] = (uint8_t)(headingInt & 0xFF);       // low byte
  buf[8] = (uint8_t)((headingInt >> 8) & 0xFF);  // high byte

  // Roll * 10 (signed, little-endian)
  int16_t rollInt = (int16_t)(imuRoll * 10.0f);
  buf[9] = (uint8_t)(rollInt & 0xFF);          // low byte
  buf[10] = (uint8_t)((rollInt >> 8) & 0xFF);  // high byte

  // Switch status byte + PWM display
  buf[11] = switches;
  buf[12] = pwmDisplay;

  return StampCrc(buf, STEER_DATA_PGN_SIZE);
}

uint8_t* PgnBuilder::BuildSensorDataPgn(uint8_t sensorValue) {
  static uint8_t buf[SENSOR_DATA_PGN_SIZE];

  // Header
  buf[0] = HEADER1;
  buf[1] = HEADER2;
  buf[2] = SOURCE_AUTO_STEER;
  buf[3] = PGN_SENSOR_DATA;
  buf[4] = 8;  // data length

  // Byte 5: sensor value, bytes 6-12: reserved (0)
  for (int i = 5; i <= 12; i++) {
    buf[i] = 0;
  }
  buf[5] = sensorValue;

  return StampCrc(buf, SENSOR_DATA_PGN_SIZE);
}

uint8_t* PgnBuilder::BuildHelloFromAutoSteerPgn(float steerAngleActual,
                                                int16_t steerPosition,
                                                uint8_t switchByte) {
  static uint8_t buf[HELLO_AUTO_STEER_PGN_SIZE];

  // Header
  buf[0] = HEADER1;
  buf[1] = HEADER2;
  buf[2] = SOURCE_AUTO_STEER;
  buf[3] = PGN_HELLO_AUTO_STEER;
  buf[4] = 5;  // data length

  // Actual steer angle * 100 (signed, little-endian)
  int16_t angleInt = (int16_t)(steerAngleActual*100.0f);
  buf[5] = (uint8_t)(angleInt & 0xFF);         // low byte
  buf[6] = (uint8_t)((angleInt >> 8) & 0xFF);  // high byte

  // Steer position (WAS counts, signed, little-endian)
  buf[7] = (uint8_t)(steerPosition & 0xFF);        // low byte
  buf[8] = (uint8_t)((steerPosition >> 8) & 0xFF); // high byte

  // Switch byte
  buf[9] = switchByte;

  return StampCrc(buf, HELLO_AUTO_STEER_PGN_SIZE);
}

uint8_t* PgnBuilder::BuildHelloFromImuPgn() {
  static uint8_t buf[HELLO_IMU_PGN_SIZE];

  // Header - data bytes all zero
  buf[0] = HEADER1;
  buf[1] = HEADER2;
  buf[2] = SOURCE_IMU;
  buf[3] = PGN_HELLO_IMU;
  buf[4] = 5;  // data length
  for (int i = 5; i <= 9; i++) {
    buf[i] = 0;
  }

  return StampCrc(buf, HELLO_IMU_PGN_SIZE);
}

uint8_t* PgnBuilder::BuildScanReplyPgn(uint8_t moduleId, const uint8_t* ipAddress,
                                       const uint8_t* subnet) {
  static uint8_t buf[SCAN_REPLY_PGN_SIZE];

  // Header
  buf[0] = HEADER1;
  buf[1] = HEADER2;
  buf[2] = moduleId;  // 126 = AutoSteer module
  buf[3] = PGN_SCAN_REPLY;
  buf[4] = 7;  // data length

  // Local IP bytes (5-8) and subnet bytes (9-11)
  for (int i = 0; i < 7; i++) {
    buf[5 + i] = 0;
  }
  if (ipAddress != NULL) {
    buf[5] = ipAddress[0];
    buf[6] = ipAddress[1];
    buf[7] = ipAddress[2];
    buf[8] = ipAddress[3];
  }
  if (subnet != NULL) {
    buf[9] = subnet[0];
    buf[10] = subnet[1];
    buf[11] = subnet[2];
  }

  return StampCrc(buf, SCAN_REPLY_PGN_SIZE);
}

// ===== RX parsers =====

bool PgnBuilder::TryParseAutoSteerData(const uint8_t* data, size_t len, AutoSteerData& out) {
  out = AutoSteerData();

  // Minimum length: header(2) + source(1) + pgn(1) + length(1) + data(8) + crc(1) = 14
  if (len < AUTOSTEER_PGN_SIZE) return false;

  // Validate header and PGN
  if (data[0] != HEADER1 || data[1] != HEADER2) return false;
  if (data[3] != PGN_AUTOSTEER) return false;

  // Speed * 10 km/h (little-endian)
  out.speedX10 = (uint16_t)(data[5] | (data[6] << 8));

  // Status byte (see STATUS_* bits)
  out.status = data[7];

  // Steer angle setpoint * 100 (signed, little-endian)
  out.steerAngleX100 = (int16_t)(data[8] | (((int16_t)(int8_t)data[9]) << 8));

  // Cross-track error, single signed byte (-127..127 cm)
  out.xte = (int8_t)data[10];

  // Section bits SC1to8 / SC9to16 (little-endian)
  out.sections = (uint16_t)(data[11] | (data[12] << 8));

  return true;
}

bool PgnBuilder::TryParseMachineData(const uint8_t* data, size_t len, MachineData& out) {
  out = MachineData();

  // Minimum length: header(2) + source(1) + pgn(1) + length(1) + data(8) + crc(1) = 14
  if (len < MACHINE_PGN_SIZE) return false;

  // Validate header and PGN
  if (data[0] != HEADER1 || data[1] != HEADER2) return false;
  if (data[3] != PGN_MACHINE) return false;

  out.uturn = data[5];      // U-turn state
  out.speedX10 = data[6];   // speed * 10 (single byte, max 25.5 km/h)
  out.hydLift = data[7];    // hydraulic lift state
  out.tram = data[8];       // tramline state
  out.geoStop = data[9];    // geo-fence stop
  out.reserved = data[10];  // reserved

  // Section bits SC1to8 / SC9to16 (little-endian)
  out.sections = (uint16_t)(data[11] | (data[12] << 8));

  return true;
}

bool PgnBuilder::TryParseSections64(const uint8_t* data, size_t len, Sections64Data& out) {
  out = Sections64Data();

  // Minimum length: header(2) + source(1) + pgn(1) + length(1) + data(10) + crc(1) = 16
  if (len < SECTIONS_64_PGN_SIZE) return false;

  // Validate header and PGN
  if (data[0] != HEADER1 || data[1] != HEADER2) return false;
  if (data[3] != PGN_SECTIONS_64) return false;

  // 64 section bits, little-endian (section 1 = bit 0 of byte 5)
  for (int i = 0; i < 8; i++) {
    out.sectionBytes[i] = data[5 + i];
    out.sectionStates |= ((uint64_t)data[5 + i]) << (8 * i);
  }

  // L/R speed, mirror of PGN 239's speed byte (speed * 10, clamped to a byte)
  out.lSpeedX10 = data[13];
  out.rSpeedX10 = data[14];

  return true;
}

bool PgnBuilder::TryParseSteerSettings(const uint8_t* data, size_t len,
                                       SteerSettingsData& out) {
  out = SteerSettingsData();

  // Minimum length: header(2) + source(1) + pgn(1) + length(1) + data(8) + crc(1) = 14
  if (len < STEER_SETTINGS_PGN_SIZE) return false;

  // Validate header and PGN
  if (data[0] != HEADER1 || data[1] != HEADER2) return false;
  if (data[3] != PGN_STEER_SETTINGS) return false;

  out.gainP = data[5];              // proportional gain (1-100)
  out.highPwm = data[6];            // high PWM (max PWM)
  out.lowPwm = data[7];             // low PWM (typically highPWM / 3)
  out.minPwm = data[8];             // min PWM to move
  out.countsPerDegree = data[9];    // counts per degree

  // WAS offset (signed 16-bit, little-endian)
  out.wasOffset = (int16_t)(data[10] | (data[11] << 8));

  // Ackermann correction percent * 100 (0-200)
  out.ackermannX100 = data[12];

  return true;
}

bool PgnBuilder::TryParseSteerConfig(const uint8_t* data, size_t len, SteerConfigData& out) {
  out = SteerConfigData();

  // Minimum length: header(2) + source(1) + pgn(1) + length(1) + data(5) + crc(1) = 11
  if (len < STEER_CONFIG_PGN_SIZE) return false;

  // Validate header and PGN
  if (data[0] != HEADER1 || data[1] != HEADER2) return false;
  if (data[3] != PGN_STEER_CONFIG) return false;

  out.set0 = data[5];             // raw setting byte 0
  out.pulseCountMax = data[6];    // pulse count max
  out.minSpeedX10 = data[7];      // min steer speed * 10
  out.set1 = data[8];             // raw setting byte 1
  out.angularVelocity = data[9];  // angular velocity

  // Decode set0 (same bit layout the classic firmware bitRead()s into steerConfig)
  out.invertWas = (out.set0 & 0x01) != 0;
  out.isRelayActiveHigh = (out.set0 & 0x02) != 0;
  out.motorDriveDirection = (out.set0 & 0x04) != 0;
  out.singleInputWas = (out.set0 & 0x08) != 0;
  out.cytronDriver = (out.set0 & 0x10) != 0;
  out.steerSwitch = (out.set0 & 0x20) != 0;
  out.steerButton = (out.set0 & 0x40) != 0;
  out.shaftEncoder = (out.set0 & 0x80) != 0;

  // Decode set1
  out.isDanfoss = (out.set1 & 0x01) != 0;
  out.pressureSensor = (out.set1 & 0x02) != 0;
  out.currentSensor = (out.set1 & 0x04) != 0;
  out.isUseYAxis = (out.set1 & 0x08) != 0;

  return true;
}

bool PgnBuilder::TryParseMachineConfig(const uint8_t* data, size_t len,
                                       MachineConfigData& out) {
  out = MachineConfigData();

  // Minimum length: header(2) + source(1) + pgn(1) + length(1) + data(8) + crc(1) = 14
  if (len < MACHINE_CONFIG_PGN_SIZE) return false;

  // Validate header and PGN
  if (data[0] != HEADER1 || data[1] != HEADER2) return false;
  if (data[3] != PGN_MACHINE_CONFIG) return false;

  out.raiseTime = data[5];              // raise time (seconds)
  out.lowerTime = data[6];              // lower time (seconds)
  out.hydraulicLiftEnabled = data[7];   // enable hydraulic (0/1)
  out.set0 = data[8];                   // bit0 InvertRelay, bit1 HydEnabled
  out.user1 = data[9];                  // user1 value (0-255)
  out.user2 = data[10];                 // user2 value (0-255)
  out.user3 = data[11];                 // user3 value (0-255)
  out.user4 = data[12];                 // user4 value (0-255)

  return true;
}

bool PgnBuilder::TryParseMachinePins(const uint8_t* data, size_t len, MachinePinsData& out) {
  out = MachinePinsData();

  // Minimum length: header(2) + source(1) + pgn(1) + length(1) + data(24) + crc(1) = 30
  if (len < MACHINE_PINS_PGN_SIZE) return false;

  // Validate header and PGN
  if (data[0] != HEADER1 || data[1] != HEADER2) return false;
  if (data[3] != PGN_MACHINE_PINS) return false;

  // 24 relay pin assignments (PinFunction enum value per byte)
  for (int i = 0; i < 24; i++) {
    out.pins[i] = data[5 + i];
  }

  return true;
}

bool PgnBuilder::TryParseHelloFromAgIo(const uint8_t* data, size_t len) {
  // Minimum length: header(2) + source(1) + pgn(1) + length(1) + data(3) + crc(1) = 9
  if (len < HELLO_AGIO_PGN_SIZE) return false;

  // Validate header and PGN
  if (data[0] != HEADER1 || data[1] != HEADER2) return false;
  if (data[3] != PGN_HELLO_FROM_AGIO) return false;

  // Magic byte 56 at data[5] (AgIO's UDP.designer layout, BuildHelloPacket)
  return data[5] == 56;
}

bool PgnBuilder::TryParseScanRequest(const uint8_t* data, size_t len) {
  // Minimum length: header(2) + source(1) + pgn(1) + length(1) + data(3) + crc(1) = 9
  if (len < SCAN_REQUEST_PGN_SIZE) return false;

  // Validate header and PGN
  if (data[0] != HEADER1 || data[1] != HEADER2) return false;
  if (data[3] != PGN_SCAN_REQUEST) return false;

  // Make really sure this is the reply pgn: magic bytes 202, 202 at data[5]/[6]
  // (same check the classic firmware performs before answering)
  return data[4] == 3 && data[5] == 202 && data[6] == 202;
}

bool PgnBuilder::TryParseSubnetChange(const uint8_t* data, size_t len, SubnetChange& out) {
  out = SubnetChange();

  // Minimum length: header(2) + source(1) + pgn(1) + length(1) + data(5) + crc(1) = 11
  if (len < SUBNET_CHANGE_PGN_SIZE) return false;

  // Validate header and PGN
  if (data[0] != HEADER1 || data[1] != HEADER2) return false;
  if (data[3] != PGN_SET_SUBNET) return false;

  // Magic bytes 201, 201 at data[5]/[6], then the three subnet octets
  if (data[5] != 201 || data[6] != 201) return false;

  out.octet1 = data[7];
  out.octet2 = data[8];
  out.octet3 = data[9];

  return true;
}

// ===== Helpers =====

bool PgnBuilder::ValidateChecksum(const uint8_t* data, size_t len) {
  // header(2) + source + pgn + len + crc
  if (len < 6) return false;

  int checksumPos = len - 1;
  uint8_t calculated = 0;
  for (int i = 2; i < checksumPos; i++) {
    calculated += data[i];
  }
  return calculated == data[checksumPos];
}

uint8_t PgnBuilder::CalculateCrc(const uint8_t* data, int start, int length) {
  uint8_t crc = 0;
  for (int i = start; i < start + length; i++) {
    crc += data[i];
  }
  return crc;
}

uint8_t* PgnBuilder::StampCrc(uint8_t* packet, size_t length) {
  // The last byte is the sum of bytes [2 .. len-2] (source through last data
  // byte), matching CalculateCrc and the parsePacket() verification rule.
  // Every builder returns through here, so the CRC offset and span are derived
  // from the buffer length instead of being hand-written per PGN. Operates in
  // place - safe for the per-PGN static buffers the builders reuse.
  packet[length - 1] = CalculateCrc(packet, 2, length - 3);
  return packet;
}
