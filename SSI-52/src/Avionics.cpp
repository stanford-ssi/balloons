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
  if(!Serial) PCB.faultLED();
  sensors.init();
  PCB.init();
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
}

/*
 * Function: evaluateState
 * -------------------
 * This function intelligently reacts to the current data frame.
 */
void Avionics::evaluateState() {
  if(calcState()   < 0) logFatalError("failed to calculate state");
  if(runDebug()    < 0) logFatalError("failed to run debug");
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
  data.BLINK         = !data.BLINK;
  data.ALTITUDE_LAST = data.ALTITUDE_BMP;
  data.VOLTAGE       = sensors.getVoltage();
  data.TEMP_EXT      = sensors.getTempOut();
  data.TEMP_IN       = sensors.getTempIn();
  data.PRESS_BMP     = sensors.getPressure();
  data.ALTITUDE_BMP  = sensors.getAltitude();
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
 * Function: calcState
 * -------------------
 * This function sets the appropriate values and flags based on the current data frame.
 */
int8_t Avionics::calcState() {
  if(data.ENABLE_DEBUG && (data.ALTITUDE_LAST >= DEBUG_ALT) && (data.ALTITUDE_BMP >= DEBUG_ALT)) {
    data.ENABLE_DEBUG = false;
  }
  return 0;
}

/*
 * Function: runDebug
 * -------------------
 * This function provides debuging information.
 */
int8_t Avionics::runDebug() {
  if(data.ENABLE_DEBUG) {
    if(displayState() < 0) return -1;
    if(printState()   < 0) return -1;
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
 * This function cuts down the payload if nessisary.
 */
int8_t Avionics::runCutdown() {
  if(!data.CUTDOWN_STATE && (data.ALTITUDE_LAST >= CUTDOWN_ALT) && (data.ALTITUDE_BMP >= CUTDOWN_ALT)) {
    return PCB.cutDown();
  }
  return 0;
}

/*
 * Function: sendSATCOMS
 * -------------------
 * This function sends the current data frame over the ROCKBLOCK IO.
 */
int8_t Avionics::sendSATCOMS() {
  String messageToSend = "";
  return 0;
}

/*
 * Function: sendAPRS
 * -------------------
 * This function sends the current data frame over the APRS RF IO.
 */
int8_t Avionics::sendAPRS() {
  String messageToSend = "";
  return 0;
}

/*
 * Function: sendCAN
 * -------------------
 * This function sends the current data frame over the CAN BUS IO.
 */
int8_t Avionics::sendCAN() {
  String messageToSend = "";
  return 0;
}

/*
 * Function: printState
 * -------------------
 * This function prints the current avionics state.
 */
int8_t Avionics::printState() {
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
  return 0;
}

/*
 * Function: displayState
 * -------------------
 * This function displays the current avionics state.
 */
int8_t Avionics::displayState() {
    PCB.writeLED(BAT_GOOD,  (data.VOLTAGE >= 3.63));
    PCB.writeLED(I_GOOD,    (data.CURRENT > 0.0 && data.CURRENT <= 0.5));
    PCB.writeLED(P_GOOD,    (data.ALTITUDE_BMP > -50 && data.ALTITUDE_BMP < 200));
    PCB.writeLED(T_GOOD,    (data.TEMP_IN > 15 && data.TEMP_IN < 50));
    PCB.writeLED(CAN_GOOD,  (data.CAN_SET_SUCESS));
    PCB.writeLED(RB_GOOD,   (data.RB_SET_SUCESS));
    PCB.writeLED(GPS_GOOD,  (data.LAT != 1000.0 && data.LAT != 0.0 && data.LONG != 1000.0 && data.LONG != 0.0));
    PCB.writeLED(HEARTBEAT, (data.BLINK));
  return 0;
}

/*
 * Function: logFatalError
 * -------------------
 * This function logs information if there has been a Fatal error.
 */
void Avionics::logFatalError(const char* debug) {
  PCB.faultLED();
  Serial.print("FATAL ERROR: ");
  Serial.print(debug);
  Serial.println("...");
}
