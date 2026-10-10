#include <Arduino.h>
#include <main.h>
#include <zAutosteer.h>
#include <zInput.h>
#include <zUDP.h>
#include <AutosteerPID.h>

// Minimum continuous hold time (ms) required before the steer button toggle fires.
// 0.5 s rejects switch contact bounce and EMI blips, which previously caused the
// random on/off toggling of steerEnable in steerButton (momentary) mode.
#ifndef STEERBTN_HOLD_TOGGLE_MS
  #define STEERBTN_HOLD_TOGGLE_MS 500
#endif

void readInputSwitches() {
  // Button toggle state variables (static = memory persists between calls)
  static uint8_t currentState = 1;
  static uint8_t reading = 0;

  // read all the switches
  workSwitch = !gpio_get_level((gpio_num_t)WORKSW_PIN);

  if (steerConfig.steerSwitch) // steer switch on - off
  {
    steerSwitch = gpio_get_level((gpio_num_t)STEERSW_PIN); // read auto steer enable switch (inverted: 1 when shorted to GND)
  } else if (steerConfig.steerButton) // steer Button momentary
  {
    // Detect steerEnable state change from external sources
    static uint8_t lastSteerEnable = 0;
    static unsigned long holdStartMs = 0;   // 0.5 s long-press hold timer
    
    reading = !gpio_get_level((gpio_num_t)STEERSW_PIN);  // inverted: 1 when button shorted to GND
    
    if (steerEnable != lastSteerEnable) {
      steerSwitch = !steerEnable;  // Sync toggle state with external changes
      currentState = steerSwitch;  // Update toggle state to match switch
      lastSteerEnable = steerEnable;
    }
    
    // Debounced toggle: the button must be held continuously for
    // STEERBTN_HOLD_TOGGLE_MS (0.5 s) before the steer toggle fires. Any
    // bounce / EMI blip (<0.5 s) resets the hold timer, so it can no
    // longer cause the random on/off toggling of steerEnable seen before.
    if (reading == HIGH) {
      if (holdStartMs == 0) { holdStartMs = millis(); }
      else if (millis() - holdStartMs >= STEERBTN_HOLD_TOGGLE_MS) {
        currentState = currentState ? 0 : 1;
        steerSwitch = currentState;
        holdStartMs = 0;  // re-arm for the next press
      }
    } else {
      holdStartMs = 0;  // released -> reset hold timer
    }
  } else // No steer switch and no steer button - keep steerSwitch at default (1)
  {
    // When no physical switch is configured, steerSwitch remains 1
    // The guidance status is handled separately via guidanceBit in packet processing
    // This prevents steerSwitch from being affected by guidance packets
  }
  switchByte = 0;
  switchByte |= (steerSwitch << 1); // put steerswitch status in bit 1 position
  switchByte |= workSwitch;
}

void autosteerLoop() {
  
  // Safety: disable steering if no 0xFE packet received for 2 seconds
  if (steerEnable && (millis() - lastFEPacketTime > 1000)) {
    steerEnable = false;
  }

  inputHandler();

  calcSteerAngle();
  
  // Handle motor on/off state safely (only transitions once)
  motorStateControl();
  
  // Only calculate PID when steering is enabled to save resources
  if (steerEnable) {
    calcSteeringPID();
  }
}
