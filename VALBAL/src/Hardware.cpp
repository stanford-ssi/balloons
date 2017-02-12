/*
  Stanford Student Space Initiative
  Balloons | VALBAL | February 2017
  Davy Ragland | dragland@stanford.edu
  Matthew Tan | mratan@stanford.edu

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
  pinMode(HEATER_INTERNAL_STRONG, OUTPUT);
  pinMode(HEATER_INTERNAL_WEAK, OUTPUT);
}

/********************************  FUNCTIONS  *********************************/
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
  if (PIDOutVar != 0.0) analogWrite(HEATER_INTERNAL_STRONG, PIDOutVar / 2 + (ANALOG_MAX / 2));
  else analogWrite(HEATER_INTERNAL_STRONG, 0);
}

/*
  function: valve
  ---------------------------------
  This function triggers the mechanical valve mechanism.
*/
void Hardware::valve(bool on) {
  // if(on) // engage valve
  // else //disengage valve
}

/*
  function: balast
  ---------------------------------
  This function triggers the mechanical balast mechanism.
*/
void Hardware::balast(bool on) {
  // if(on) // engage balast
  // else //disengage balast
}

/*
  function: cutDown
  ---------------------------------
  This function triggers the mechanical cutdown of the payload.
*/
void Hardware::cutDown(bool on) {
  // if(on) // engage cutdown
  // else //disengage cutdown
}
