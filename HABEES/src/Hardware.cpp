/*
  Stanford Student Space Initiative
  Balloons | HABEES | February 2017
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
  pinMode(WATCHDOG_PIN, OUTPUT);
  pinMode(FAULT_PIN, OUTPUT);
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(CUTDOWN_PIN, OUTPUT);
  mcp.begin();
  for (uint8_t i = 0; i < 16; i++) mcp.pinMode(i,  OUTPUT);
  for (uint8_t i = 0; i < 8; i++) writeLED(i, false);
  analogWriteResolution(ANALOG_RES);
  analogWrite(CUTDOWN_PIN, 0);
}

/********************************  FUNCTIONS  *********************************/
/*
  function: writeLED
  ---------------------------------
  This function sets a pin to green or red.
*/
void Hardware::writeLED(uint8_t PIN, bool green) {
  if (green) {
    mcp.digitalWrite(PIN, HIGH);
    mcp.digitalWrite(15 - PIN, LOW);
  }
 else {
    mcp.digitalWrite(PIN, LOW);
    mcp.digitalWrite(15 - PIN, HIGH);
  }
}

/*
 * Function: faultLED
 * -------------------
 * This function alerts the user if there has been a fatal error.
 */
void Hardware::faultLED() {
  digitalWrite(FAULT_PIN, HIGH);
  delay(LOOP_RATE);
  digitalWrite(FAULT_PIN, LOW);
}

/*
  function: heater
  ---------------------------------
  This function runs the PID heater within the board.
*/
void Hardware::heater(double temp) {
  PIDTempVar = temp;
  pid.Compute();
  if (PIDOutVar != 0.0) analogWrite(HEATER_PIN, PIDOutVar / 2 + (ANALOG_MAX / 2));
  else analogWrite(HEATER_PIN, 0);
}

/*
  function: cutDown
  ---------------------------------
  This function triggers the mechanical cutdown of the payload.
*/
void Hardware::cutDown(bool on) {
  if(on) analogWrite(CUTDOWN_PIN, ANALOG_MAX);
  else   analogWrite(CUTDOWN_PIN, 0);
}

/*
  function: watchdog
  ---------------------------------
  This function pings the watchdog IC in order to prevent a hardware reboot.
*/
void Hardware::watchdog() {
  digitalWrite(WATCHDOG_PIN, HIGH);
  delay(1);
  digitalWrite(WATCHDOG_PIN, LOW);
}
