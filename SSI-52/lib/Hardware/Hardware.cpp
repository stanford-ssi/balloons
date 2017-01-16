/*
  Stanford Student Space Initiative
  Balloons | HABEES | January 2017
  Davy Ragland | dragland@stanford.edu

  File: Hardware.cpp
  --------------------------
  Implimentation of Hardware.h
*/

#include "Hardware.h"

/**********************************  SETUP  ***********************************/
/*
  function: init
  ---------------------------------
  This function initializes the PCB hardware.
*/
void Hardware::init() {
  mcp.begin();

  pinMode(WATCHDOG_PIN, OUTPUT);
  pinMode(FAULT_LED, OUTPUT);
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(CUTDOWN_PIN, OUTPUT);
  analogWrite(CUTDOWN_PIN, 0);

  for (uint8_t i = 0; i < 16; i++) mcp.pinMode(i,  OUTPUT);
  for (uint8_t i = 0; i < 8; i++) writeLED(i, false);
}

/********************************  FUNCTIONS  *********************************/
/*
  function: writeLED
  ---------------------------------
  This function sets a pin to green or red.
*/
int8_t Hardware::writeLED(uint8_t PIN, bool green) {
  if (green) {
    mcp.digitalWrite(PIN, HIGH);
    mcp.digitalWrite(15 - PIN, LOW);
  }
 else {
    mcp.digitalWrite(PIN, LOW);
    mcp.digitalWrite(15 - PIN, HIGH);
  }
  return 0;
}

/*
 * Function: faultLED
 * -------------------
 * This function alerts the user if there has been a fatal error.
 */
int8_t Hardware::faultLED() {
  digitalWrite(FAULT_LED, HIGH);
  delay(LOOP_RATE);
  digitalWrite(FAULT_LED, LOW);
  return 0;
}

/*
  function: cutDown
  ---------------------------------
  This function runs the PID heater within the board.
*/
int8_t Hardware::heater(double temp) {
  PIDTempPtr = temp;
  pid.Compute();
  if (PIDOutPtr != 0.0) analogWrite(HEATER_PIN, PIDOutPtr / 2 + 127.50);
  else analogWrite(HEATER_PIN, 0);
  return 0;
}

/*
  function: cutDown
  ---------------------------------
  This function triggers the mechanical cutdown of the payload.
*/
int8_t Hardware::cutDown(bool on) {
  if(on) analogWrite(CUTDOWN_PIN, 255);
  else   analogWrite(CUTDOWN_PIN, 0);
  return 0;
}

/*
  function: watchdog
  ---------------------------------
  This function pings the watchdog IC in order to prevent a hardware reboot.
*/
int8_t Hardware::watchdog() {
  digitalWrite(WATCHDOG_PIN, HIGH);
  delay(1);
  digitalWrite(WATCHDOG_PIN, LOW);
  return 0;
}
