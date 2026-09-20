#ifndef AUTOSTEERPID_H
#define AUTOSTEERPID_H

#include <Arduino.h>
#include "main.h"
//pwm variables - declared as extern (defined in AutosteerPID.cpp)
extern int16_t pwmDrive;
extern float pValue, errorAbs, highLowPerDeg;

void motorStateControl();

void motorDrive();
void calcSteeringPID(void);
void adaptKp(void);
#ifdef USE_AUTOTUNE_PID
void loadAutoTuneKd(void);
/** Currently learned (auto-tuned) D gain - exposed for the web Telemetria tab */
float getLearnedKd(void);
/** Current I-term accumulator state (pre-gain) - exposed for the web Telemetria tab */
float getIntegralError(void);
#endif

#endif