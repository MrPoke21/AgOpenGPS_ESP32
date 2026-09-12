#include <Configuration.h>
#include <zHandlers.h>
#include <zUDP.h>
#ifdef SPEED_IMPULSE_ENABLED
  #include <zSpeedImpulse.h>
#endif

uint8_t error = 0;

// BNO08x address variables to check where it is
const uint8_t bno08xAddresses[] = { 0x4A, 0x4B };
const int16_t nrBNO08xAdresses = sizeof(bno08xAddresses) / sizeof(bno08xAddresses[0]);
uint8_t bno08xAddress;

euler_t ypr = euler_t();
IMUSample imuPrev = { 0, 0, 0, 0, 0, false };
IMUSample imuCurr = { 0, 0, 0, 0, 0, false };
Adafruit_BNO08x bno08x(-1);
/* A parser is declared with 3 handlers at most */
NMEAParser<2> parser;

sh2_SensorValue_t sensorValue;

char nmea[256] = {};

// GGA
char fixTime[12] = {};
char latitude[15] = {};
char latNS[3] = {};
char longitude[15] = {};
char lonEW[3] = {};
char fixQuality[2] = {};
char numSats[4] = {};
char HDOP[5] = {};
char altitude[12] = {};
char ageDGPS[10] = {};

// VTG

char speedKnots[10] = {};
char vtgHeading[12] = {};

// Conversion to Hexidecimal
const char *asciiHex = "0123456789ABCDEF";

// Adaptive outlier gate: reject a new quaternion only if the implied
// rotation rate exceeds what the tractor/IMU can plausibly produce. This
// replaces the old fixed dot-product threshold, which assumed a fixed
// sample spacing and either passed everything through or froze the heading
// solid whenever a report was flagged bad.
const float MAX_IMU_ANGULAR_RATE_RADPS = 5.0f;     // ~286 deg/s hard cap
const float IMU_OUTLIER_ANGLE_MARGIN_RAD = 0.05f;  // ~3 deg noise floor
// BNO08x report interval. Draining is now done every loop() iteration (see
// imuTask()/main.cpp), so this just needs to comfortably exceed the 10Hz
// GPS rate; it is not the reason for the old stuttering.
const uint32_t IMU_REPORT_INTERVAL_US = 10000;

void initHandler() {
  // the dash means wildcard
  parser.setErrorHandler(errorHandler);
  parser.addHandler("G-GGA", GGA_Handler);
  parser.addHandler("G-VTG", VTG_Handler);
}
// If odd characters showed up.
void errorHandler() {
  //nothing at the moment
}

void GGA_Handler()  //Rec'd GGA
{
  // fix time
  parser.getArg(0, fixTime);

  // latitude
  parser.getArg(1, latitude);
  parser.getArg(2, latNS);

  // longitude
  parser.getArg(3, longitude);
  parser.getArg(4, lonEW);

  // fix quality
  parser.getArg(5, fixQuality);

  // satellite #
  parser.getArg(6, numSats);

  // HDOP
  parser.getArg(7, HDOP);

  // altitude
  parser.getArg(8, altitude);

  // time of last DGPS update
  parser.getArg(12, ageDGPS);

  GGA_Available = true;

  if (useBNO08x) {
    // "Now" (micros(), same timebase as the BNO08x sample timestamps) is the
    // instant the GGA sentence finished parsing. This is what the IMU
    // orientation gets synchronised to below.
    calculateIMU((uint32_t)micros());
  }               //Get IMU data ready, time-aligned with this GPS fix
  BuildNmea();  //Build & send GPS data to AgIO
}

void gpsStream() {
  // Non-blocking GPS stream processing with character limit per loop
  int charCount = 0;
  int maxCharsPerLoop = 64;  // Process max 64 chars per loop to avoid blocking
  
  while (Serial2.available() && charCount < maxCharsPerLoop) {
    char x = (char)Serial2.read();
    parser << x;
    charCount++;
  }
}

// Spherical linear interpolation between two unit quaternions. Also works
// as a bounded extrapolation when u > 1 (used when the GPS fix instant is a
// few ms newer than the latest IMU sample).
static void slerpQuaternion(float r0, float i0, float j0, float k0,
                            float r1, float i1, float j1, float k1,
                            float u,
                            float &rOut, float &iOut, float &jOut, float &kOut) {
  float dot = r0 * r1 + i0 * i1 + j0 * j1 + k0 * k1;

  // Take the shortest path around the hypersphere.
  if (dot < 0) {
    r1 = -r1; i1 = -i1; j1 = -j1; k1 = -k1;
    dot = -dot;
  }
  dot = constrain(dot, -1.0f, 1.0f);

  if (dot > 0.9995f) {
    // Nearly identical orientations - plain lerp avoids dividing by ~0.
    rOut = r0 + u * (r1 - r0);
    iOut = i0 + u * (i1 - i0);
    jOut = j0 + u * (j1 - j0);
    kOut = k0 + u * (k1 - k0);
  } else {
    float theta0 = acos(dot);
    float theta = theta0 * u;
    float sinTheta0 = sin(theta0);
    float s0 = sin(theta0 - theta) / sinTheta0;
    float s1 = sin(theta) / sinTheta0;
    rOut = s0 * r0 + s1 * r1;
    iOut = s0 * i0 + s1 * i1;
    jOut = s0 * j0 + s1 * j1;
    kOut = s0 * k0 + s1 * k1;
  }

  float norm = sqrt(rOut * rOut + iOut * iOut + jOut * jOut + kOut * kOut);
  if (norm > 1e-6f) {
    rOut /= norm; iOut /= norm; jOut /= norm; kOut /= norm;
  }
}

// Estimates the orientation at targetTimeUs (typically "now", captured the
// moment the GGA sentence finished parsing) by interpolating/extrapolating
// between the two most recent raw IMU samples. This is what actually
// synchronises the 10Hz GPS fix with the faster IMU stream: instead of
// grabbing whatever quaternion happened to be sitting in memory (up to one
// whole IMU period stale/jittery), we compute the value the IMU would have
// reported at the exact GPS instant.
//
// All of the "expensive" vector math - the outlier/plausibility check
// (acos), the SLERP interpolation (acos/sin) and the quaternion->Euler
// conversion (atan2/asin) - lives here and only runs once per incoming NMEA
// sentence (~10Hz). imuTask() itself does no trig at all; it only saves the
// raw quaternion samples as they arrive from the BNO08x (~100Hz), which is
// what actually keeps the CPU load down.
void calculateIMU(uint32_t targetTimeUs) {
  if (!imuCurr.valid) {
    return;
  }

  float qr = imuCurr.qr, qi = imuCurr.qi, qj = imuCurr.qj, qk = imuCurr.qk;

  if (imuPrev.valid) {
    int32_t span = (int32_t)(imuCurr.time - imuPrev.time);
    if (span > 0) {
      // Plausibility check between the two latest raw samples: reject the
      // newest one if it implies a rotation rate the tractor/IMU can't
      // actually produce, and fall back to the older sample instead.
      float dt = (float)span / 1000000.0f;
      float dot = qr * imuPrev.qr + qi * imuPrev.qi + qj * imuPrev.qj + qk * imuPrev.qk;
      if (dot < 0) dot = -dot;
      dot = constrain(dot, -1.0f, 1.0f);
      float angle = 2.0f * acos(dot);
      float maxAngle = MAX_IMU_ANGULAR_RATE_RADPS * dt + IMU_OUTLIER_ANGLE_MARGIN_RAD;

      if (angle > maxAngle) {
        DEBUG_PRINT(millis());
        DEBUG_PRINT("\tIMU outlier! angle(deg)=");
        DEBUG_PRINTLN(angle * RAD_TO_DEG);
        qr = imuPrev.qr; qi = imuPrev.qi; qj = imuPrev.qj; qk = imuPrev.qk;
      } else {
        float u = (float)(int32_t)(targetTimeUs - imuPrev.time) / (float)span;
        // Small bounded extrapolation only - a stalled/disconnected IMU must
        // not be allowed to run away with the heading.
        u = constrain(u, 0.0f, 1.5f);
        slerpQuaternion(imuPrev.qr, imuPrev.qi, imuPrev.qj, imuPrev.qk,
                        imuCurr.qr, imuCurr.qi, imuCurr.qj, imuCurr.qk,
                        u, qr, qi, qj, qk);
      }
    }
  }

  quaternionToEuler(qr, qi, qj, qk);
}

void quaternionToEuler(float qr, float qi, float qj, float qk) {

  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

    // Normalizálás ellenőrzése
  float norm_sq = sqr + sqi + sqj + sqk;
  if (abs(norm_sq - 1.0f) > 0.1f) {
    DEBUG_PRINT(millis());
    DEBUG_PRINT("\tQuat not normalized! norm²=");
    DEBUG_PRINTLN(norm_sq);
    
    // Normalizálás
    float norm = sqrt(norm_sq);
    qr /= norm;
    qi /= norm;
    qj /= norm;
    qk /= norm;
    
    sqr = sq(qr);
    sqi = sq(qi);
    sqj = sq(qj);
    sqk = sq(qk);
  }

  ypr.yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  if (steerConfig.IsUseY_Axis) {
    ypr.pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr.roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
  } else {
    ypr.roll = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr.pitch = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
  }

  ypr.yaw *= -RAD_TO_DEG;
  if (ypr.yaw < 0) {
    ypr.yaw += 360;
  }
  ypr.pitch *= RAD_TO_DEG;
  ypr.roll *= RAD_TO_DEG;

  if (invertRoll) {
    ypr.roll *= -1;
  }
}

void setReports() {
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, IMU_REPORT_INTERVAL_US)) {
    DEBUG_PRINTLN("Could not enable stabilized remote vector");
    return;
  }
}

void imuTask() {

  if (!useBNO08x) {
    return;
  }
  if (bno08x.wasReset()) {
    DEBUG_PRINTLN("sensor was reset ");
    setReports();
  }

  // Drain every buffered report in one pass. imuTask() is now called from
  // every loop() iteration (see main.cpp) instead of a slow 50ms timer, so
  // this keeps the BNO08x's internal queue empty and guarantees imuCurr is
  // always the freshest report available - no backlog, no growing latency.
  //
  // Performance: this loop only *saves* the raw quaternion + timestamp for
  // every incoming report (~100Hz) - no trigonometry here at all. The
  // outlier check and all orientation math (SLERP, Euler conversion) is
  // deferred to calculateIMU(), which only runs once per incoming NMEA GGA
  // sentence (~10Hz). That is a ~10x reduction in expensive trig calls.
  while (bno08x.getSensorEvent(&sensorValue)) {

    if (sensorValue.sensorId != SH2_GAME_ROTATION_VECTOR) {
      continue;
    }

    // status: 0 = unreliable, 1..3 = low/medium/high confidence. This comes
    // straight from the chip's own fusion output, so checking it here is
    // free (no computation of our own).
    if (sensorValue.status == 0 && imuCurr.valid) {
      continue;
    }

    imuPrev = imuCurr;
    imuCurr.time = (uint32_t)sensorValue.timestamp;  // us, same timebase as micros()
    imuCurr.qr = sensorValue.un.gameRotationVector.real;
    imuCurr.qi = sensorValue.un.gameRotationVector.i;
    imuCurr.qj = sensorValue.un.gameRotationVector.j;
    imuCurr.qk = sensorValue.un.gameRotationVector.k;
    imuCurr.valid = true;
  }
}


void initIMU() {
  for (int16_t i = 0; i < nrBNO08xAdresses; i++) {
    bno08xAddress = bno08xAddresses[i];

    DEBUG_PRINT("Checking for BNO08X at address 0x");
    DEBUG_PRINTLN(bno08xAddress, HEX);
    
    Wire.beginTransmission(bno08xAddress);
    error = Wire.endTransmission();

    if (error == 0) {
      DEBUG_PRINT("0x");
      DEBUG_PRINT(bno08xAddress, HEX);
      DEBUG_PRINTLN(" BNO08X detected.");
      
      // Initialize BNO080 lib with proper address casting
      if (bno08x.begin_I2C(bno08xAddress)) {
        useBNO08x = true;
        DEBUG_PRINTLN("BNO08x initialized successfully");
        setReports();  // Enable sensor reports after init
        break;
      } else {
        DEBUG_PRINTLN("BNO080 initialization failed at given I2C address.");
      }
    } else {
      DEBUG_PRINT("0x");
      DEBUG_PRINT(bno08xAddress, HEX);
      DEBUG_PRINTLN(" BNO08X not found (error code: ");
      DEBUG_PRINT(error);
      DEBUG_PRINTLN(")");
    }
  }
  
  if (!useBNO08x) {
    DEBUG_PRINTLN("WARNING: BNO08x not found on any address - IMU disabled");
  }
}

void BuildNmea(void) {
  char imuHeading[8];
  char imuRoll[8];
  char imuPitch[8];
  const char *age = ageDGPS[0] ? ageDGPS : "0";
  const char *speed = speedKnots[0] ? speedKnots : "0";

  // AgOpenGPS expects IMU angles as integer tenths of a degree, not floats.
  snprintf(imuHeading, sizeof(imuHeading), "%ld", lroundf(ypr.yaw * 10.0f));
  snprintf(imuRoll, sizeof(imuRoll), "%ld", lroundf(ypr.roll * 10.0f));
  snprintf(imuPitch, sizeof(imuPitch), "%ld", lroundf(ypr.pitch * 10.0f));

  int length = snprintf(
    nmea, sizeof(nmea),
    "$PANDA,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,0*",
    fixTime,
    latitude,
    latNS,
    longitude,
    lonEW,
    fixQuality,
    numSats,
    HDOP,
    altitude,
    age,
    speed,
    imuHeading,
    imuRoll,
    imuPitch);

  if (length < 0 || length + 5 > (int)sizeof(nmea)) {
    return;
  }

  uint8_t checksum = 0;
  for (int i = 1; i < length - 1; i++) {
    checksum ^= (uint8_t)nmea[i];
  }

  nmea[length++] = asciiHex[(checksum >> 4) & 0x0F];
  nmea[length++] = asciiHex[checksum & 0x0F];
  nmea[length++] = '\r';
  nmea[length++] = '\n';
  nmea[length] = '\0';

  sendNMEA(reinterpret_cast<const uint8_t *>(nmea), (uint16_t)length);
}

double convertToDecimalDegrees(const char *latLon, const char *direction) {
  // Input validation
  if (!latLon || !direction || latLon[0] == '\0') {
    Serial.println("ERROR: Invalid lat/lon string");
    return 0.0;
  }
  
  char deg[4] = { 0 };
  char *dot, *min;
  int len;
  double dec = 0;

  if ((dot = strchr(latLon, '.'))) {   // decimal point was found
    min = dot - 2;                     // mark the start of minutes 2 chars back
    len = min - latLon;                // find the length of degrees
    
    if (len < 0 || len > 3) {
      Serial.println("ERROR: Invalid lat/lon format");
      return 0.0;
    }
    
    strncpy(deg, latLon, len);         // copy the degree string to allow conversion to float
    dec = atof(deg) + atof(min) / 60;  // convert to float
    
    if (strcmp(direction, "S") == 0 || strcmp(direction, "W") == 0)
      dec *= -1;
  }
  return dec;
}

void VTG_Handler() {
  // vtg heading
  parser.getArg(0, vtgHeading);
  // vtg Speed knots
  parser.getArg(4, speedKnots);
}