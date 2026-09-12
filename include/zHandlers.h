#ifndef ZHANDLERS_H
#define ZHANDLERS_H

#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include "zNMEAParser.h"
#include "main.h"
#define RAD_TO_DEG_X_10 57.295779513082320876798154814105



// A single BNO08x game-rotation-vector sample, timestamped on the sensor
// hub's own microsecond clock (same timebase as micros() on this MCU).
// Two consecutive accepted samples are kept so the orientation can be
// interpolated (SLERP) to the exact instant a GPS GGA sentence arrives,
// instead of using whatever stale snapshot happened to be sitting in
// memory.
struct IMUSample {
  uint32_t time;   // sensor timestamp [us], same timebase as micros()
  float qr;
  float qi;
  float qj;
  float qk;
  bool valid;
};
extern IMUSample imuPrev;
extern IMUSample imuCurr;


// booleans to see if we are using BNO08x

//extern uint8_t error;

// BNO08x address variables to check where it is
extern const uint8_t bno08xAddresses[2];
extern const int16_t nrBNO08xAdresses;
extern uint8_t bno08xAddress;

// the new PANDA sentence buffer
extern char nmea[256];

// GGA
extern char fixTime[12];
extern char latitude[15];
extern char latNS[3];
extern char longitude[15];
extern char lonEW[3];
extern char fixQuality[2];
extern char numSats[4];
extern char HDOP[5];
extern char altitude[12];
extern char ageDGPS[10];

// VTG
extern char speedKnots[10];

double convertToDecimalDegrees(const char *latLon, const char *direction);
void quaternionToEuler(float qr, float qi, float qj, float qk);
void BuildNmea(void);
// targetTimeUs: the instant (micros() timebase) the orientation should be
// evaluated at - normally "now", captured right when a GGA sentence has
// just finished parsing.
void calculateIMU(uint32_t targetTimeUs);

void errorHandler();
void GGA_Handler();
void VTG_Handler();

void initIMU();
void initHandler();
void imuTask();
void gpsStream();
#endif