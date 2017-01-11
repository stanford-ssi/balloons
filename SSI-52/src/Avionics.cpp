/*
  Stanford Student Space Initiative
  Balloons | HABEES | January 2017
  Davy Ragland | dragland@stanford.edu

  File: Avionics.cpp
  --------------------------
  Implimentation of Avionics.h
*/

#include "Avionics.h"

/**********************************  SETUP  ***********************************/
void Avionics::init() {
  Serial.begin(9600);
  pinMode(FAULT_LED, OUTPUT);
  if(!Serial) faultLED();
}

/********************************  FUNCTIONS  *********************************/
void Avionics::updateData() {
  if(readData()    < 0) logFatalError("failed to read Data");
  if(logData()     < 0) logFatalError("failed to log Data");
  if(printData()   < 0) logFatalError("failed to print Data");
}

void Avionics::evaluateState() {
  if(debug()       < 0) logFatalError("failed to debug state");
  if(runHeaters()  < 0) logFatalError("failed to run heaters");
  if(runCutdown()  < 0) logFatalError("failed to run cutdown");
}

void Avionics::sendComms() {
  if(sendSATCOMS() < 0) logFatalError("failed to communicate over SATCOMS");
  if(sendAPRS()    < 0) logFatalError("failed to communicate over APRS");
  if(sendCAN()     < 0) logFatalError("failed to communicate over CAN");
}

void Avionics::sleep() {
  delay(LOOP_RATE);
}

/*********************************  HELPERS  **********************************/
int8_t Avionics::readData() {
  return 0;
}

int8_t Avionics::debug() {
  return 0;
}

int8_t Avionics::logData() {
  Serial.print("test value");
  Serial.println();
  return 0;
}

int8_t Avionics::printData() {
  return 0;
}

int8_t Avionics::runHeaters() {
  return 0;
}

int8_t Avionics::runCutdown() {
  return 0;
}

int8_t Avionics::sendSATCOMS() {
  return 0;
}

int8_t Avionics::sendAPRS() {
  return 0;
}

int8_t Avionics::sendCAN() {
  return 0;
}

void Avionics::faultLED() {
  digitalWrite(FAULT_LED, HIGH);
  delay(100);
  digitalWrite(FAULT_LED, LOW);
}

void Avionics::logFatalError(const char* debug) {
  faultLED();
  Serial.print("FATAL ERROR: ");
  Serial.print(debug);
  Serial.println("...");
}
