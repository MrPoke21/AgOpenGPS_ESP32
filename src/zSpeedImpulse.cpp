#include <Arduino.h>
#include <Configuration.h>

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool running = false;


// Forward declaration for timer interrupt handler
void IRAM_ATTR onTimer();

void initSpeedImpulse() { // Pin beállítás
  pinMode(IMPULSE_PIN, OUTPUT);
  digitalWrite(IMPULSE_PIN, LOW);
  
  // Hardware timer inicializálás
  timer = timerBegin(0, 80, true);              // Timer 0, prescaler 80 (1 MHz)
  timerAttachInterrupt(timer, &onTimer, true);  // Interrupt hozzárendelés
  timerAlarmWrite(timer, 1000, true);           // Kezdeti érték
  timerAlarmEnable(timer);
}

// ========== TIMER INTERRUPT ==========
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  if (running) {
    digitalWrite(IMPULSE_PIN, !digitalRead(IMPULSE_PIN));
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

// ========== FŐ FÜGGVÉNY: SEBESSÉG BEÁLLÍTÁS ==========
void setSpeedKmh(float speed_kmh) {
  // Konverzió: km/h -> m/s
  float speed_ms = speed_kmh / 3.6;
  
  // Ha sebesség túl alacsony, stop
  if (speed_ms < 0.01) {
    running = false;
    digitalWrite(IMPULSE_PIN, LOW);
    return;
  }
  
  // Impulzus frekvencia (Hz) = sebesség (m/s) × impulzus/méter
  float pulses_per_second = speed_ms * PULSES_PER_METER;
  
  // Timer periódus (toggle-höz fél periódus kell)
  uint32_t pulse_interval_us = (uint32_t)((1000000.0 / pulses_per_second) / 2.0);
  
  // Timer újrakonfigurálás
  timerAlarmWrite(timer, pulse_interval_us, true);
  running = true;
}