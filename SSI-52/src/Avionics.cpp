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
/*
 * Function: init
 * -------------------
 * This function initializes the avionics flight controller.
 */
void Avionics::init() {
  Serial.begin(9600);
  pinMode(FAULT_LED, OUTPUT);
  if(!Serial) faultLED();
  sensors.init();
}

/********************************  FUNCTIONS  *********************************/
/*
 * Function: updateData
 * -------------------
 * This function handles basic flight data collection.
 */
void Avionics::updateData() {
  if(readData()    < 0) logFatalError("failed to read Data");
  if(logData()     < 0) logFatalError("failed to log Data");
  if(printData()   < 0) logFatalError("failed to print Data");
}

/*
 * Function: evaluateState
 * -------------------
 * This function intelligently reacts to the current data frame.
 */
void Avionics::evaluateState() {
  if(debug()       < 0) logFatalError("failed to debug state");
  if(runHeaters()  < 0) logFatalError("failed to run heaters");
  if(runCutdown()  < 0) logFatalError("failed to run cutdown");
}

/*
 * Function: sendComms
 * -------------------
 * This function sends the current data frame down.
 */
void Avionics::sendComms() {
  if(sendSATCOMS() < 0) logFatalError("failed to communicate over SATCOMS");
  if(sendAPRS()    < 0) logFatalError("failed to communicate over APRS");
  if(sendCAN()     < 0) logFatalError("failed to communicate over CAN");
}

/*
 * Function: sleep
 * -------------------
 * This function sleeps at the end of the loop.
 */
void Avionics::sleep() {
  delay(LOOP_RATE);
}

/*********************************  HELPERS  **********************************/
/*
 * Function: readData
 * -------------------
 * This function updates the current data frame.
 */
int8_t Avionics::readData() {
  data.ALTITUDE_LAST = data.ALTITUDE_BMP;
  data.TEMP_EXT      = sensors.getTempOut();
  data.TEMP_IN       = sensors.getTempIn();
  data.PRESS_BMP     = sensors.getPressure();
  data.ALTITUDE_BMP  = sensors.getAltitude();
  data.BLINK         = !data.BLINK;
  return 0;
}

/*
 * Function: debug
 * -------------------
 * This function error checks the current data frame.
 */
int8_t Avionics::debug() {
  return 0;
}

/*
 * Function: logData
 * -------------------
 * This function logs the current data frame.
 */
int8_t Avionics::logData() {
  return 0;
}

/*
 * Function: printData
 * -------------------
 * This function prints the current data frame.
 */
int8_t Avionics::printData() {
  if (data.ALTITUDE_BMP < DEBUG_ALT) {
    Serial.print(millis());
    Serial.print(",");
    Serial.print(data.ALTITUDE_BMP);
    Serial.print(",");
    Serial.print(data.TEMP_IN);
    Serial.print(",");
    Serial.print(data.TEMP_EXT);
    Serial.print(",");
    Serial.print(data.PRESS_BMP);
    Serial.println();
  }
  return 0;
}

/*
 * Function: runHeaters
 * -------------------
 * This function thermaly regulates the avionics.
 */
int8_t Avionics::runHeaters() {
  return 0;
}

/*
 * Function: runCutdown
 * -------------------
 * This function cutsdown if nessisary.
 */
int8_t Avionics::runCutdown() {
  return 0;
}

/*
 * Function: sendSATCOMS
 * -------------------
 * This function sends the current data frame over the ROCKBLOCK IO.
 */
int8_t Avionics::sendSATCOMS() {
  return 0;
}

/*
 * Function: sendAPRS
 * -------------------
 * This function sends the current data frame over the APRS RF IO.
 */
int8_t Avionics::sendAPRS() {
  return 0;
}

/*
 * Function: sendCAN
 * -------------------
 * This function sends the current data frame over the CAN BUS IO.
 */
int8_t Avionics::sendCAN() {
  return 0;
}

/*
 * Function: faultLED
 * -------------------
 * This function alerts the user if there has been a Fatal error.
 */
void Avionics::faultLED() {
  digitalWrite(FAULT_LED, HIGH);
  delay(100);
  digitalWrite(FAULT_LED, LOW);
}

/*
 * Function: logFatalError
 * -------------------
 * This function logs information if there has been a Fatal error.
 */
void Avionics::logFatalError(const char* debug) {
  faultLED();
  Serial.print("FATAL ERROR: ");
  Serial.print(debug);
  Serial.println("...");
}
