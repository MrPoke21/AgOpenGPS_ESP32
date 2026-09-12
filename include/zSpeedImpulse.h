#ifndef ZSPEEDIMPULSE_H
#define ZSPEEDIMPULSE_H

#include <Arduino.h>

// Initializes the speed impulse system (pin setup, timer init)
void initSpeedImpulse();

// Sets the speed in Kmh, generates impulses accordingly
void setSpeedKmh(float speed_Kmh);

// Timer interrupt handler for impulse generation
void IRAM_ATTR onTimer();

#endif // ZSPEEDIMPULSE_H
