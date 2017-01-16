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
  sensors.init();
  PCB.init();
  if(!Serial) PCB.faultLED();
  if(!SD.begin(SD_CS)) PCB.faultLED();

  Serial.println("Stanford Student Space Initiative Balloons Launch "+ MISSION_NUMBER + "\n" + CSV_DATA_HEADER);
  dataFile = SD.open("data.txt", FILE_WRITE);
  if(!dataFile) PCB.faultLED();
  dataFile.println("Stanford Student Space Initiative Balloons Launch " + MISSION_NUMBER + "\n" + CSV_DATA_HEADER);
  dataFile.close();

  gpsModule.init();
  RBModule.init();
  radioModule.init();
  canModule.init();

}

/********************************  FUNCTIONS  *********************************/
/*
 * Function: updateData
 * -------------------
 * This function handles basic flight data collection.
 */
void Avionics::updateData() {
  if(readData()    < 0) logFatalError("unable to read Data");
  if(logData()     < 0) logFatalError("unable to log Data");
}

/*
 * Function: evaluateState
 * -------------------
 * This function intelligently reacts to the current data frame.
 */
void Avionics::evaluateState() {
  if(calcState()   < 0) logFatalError("unable to calculate state");
  if(debugState()  < 0) logFatalError("unable to debug state");
  if(runHeaters()  < 0) logFatalError("unable to run heaters");
  if(runCutdown()  < 0) logFatalError("unable to run cutdown");
}

/*
 * Function: sendComms
 * -------------------
 * This function sends the current data frame down.
 */
void Avionics::sendComms() {
  if(sendSATCOMS() < 0) logFatalError("unable to communicate over SATCOMS");
  if(sendAPRS()    < 0) logFatalError("unable to communicate over APRS");
  if(sendCAN()     < 0) logFatalError("unable to communicate over CAN");
}

/*
 * Function: sleep
 * -------------------
 * This function sleeps at the end of the loop.
 */
void Avionics::sleep() {
  gpsModule.smartDelay(LOOP_RATE);
}

/*********************************  HELPERS  **********************************/
/*
 * Function: readData
 * -------------------
 * This function updates the current data frame.
 */
int8_t Avionics::readData() {
  data.HEARTBEAT_STATE = !data.HEARTBEAT_STATE;
  data.LOOP_RATE       = millis() - data.LOOP_START;
  data.LOOP_START      = millis();
  data.ALTITUDE_LAST   = data.ALTITUDE_BMP;
  data.VOLTAGE         = sensors.getVoltage();
  data.TEMP_EXT        = sensors.getTempOut();
  data.TEMP_IN         = sensors.getTempIn();
  data.PRESS_BMP       = sensors.getPressure();
  data.ALTITUDE_BMP    = sensors.getAltitude();
  data.LAT_GPS         = gpsModule.getLatitude();
  data.LONG_GPS        = gpsModule.getLongitude();
  data.ALTITUDE_GPS    = gpsModule.getAltitude();
  data.SPEED_GPS       = gpsModule.getSpeed();
  return 0;
}

/*
 * Function: logData
 * -------------------
 * This function logs the current data frame.
 */
int8_t Avionics::logData() {
  dataFile = SD.open("data.txt", FILE_WRITE);
  if(!dataFile) return -1;
  dataFile.print(data.TIME);
  dataFile.print(",");
  dataFile.print(data.LOOP_RATE);
  dataFile.print(",");
  dataFile.print(data.VOLTAGE);
  dataFile.print(",");
  dataFile.print(data.CURRENT);
  dataFile.print(",");
  dataFile.print(data.ALTITUDE_BMP);
  dataFile.print(",");
  dataFile.print(data.ASCENT_RATE);
  dataFile.print(",");
  dataFile.print(data.TEMP_IN);
  dataFile.print(",");
  dataFile.print(data.TEMP_EXT);
  dataFile.print(",");
  dataFile.print(String(data.LAT_GPS, 4));
  dataFile.print(",");
  dataFile.print(String(data.LONG_GPS, 4));
  dataFile.print(",");
  dataFile.print(data.SPEED_GPS);
  dataFile.print(",");
  dataFile.print(data.ALTITUDE_GPS);
  dataFile.print(",");
  dataFile.print(data.PRESS_BMP);
  dataFile.print(",");
  dataFile.print(data.RB_SENT_COMMS);
  dataFile.print(",");
  dataFile.print(data.CUTDOWN_STATE);
  dataFile.print("\n");
  dataFile.close();
  return 0;
}

/*
 * Function: calcState
 * -------------------
 * This function sets the appropriate values and flags based on the current data frame.
 */
int8_t Avionics::calcState() {
  data.BAT_GOOD_STATE  = (data.VOLTAGE >= 3.63);
  data.I_GOODD_STATE   = (data.CURRENT > 0.0 && data.CURRENT <= 0.5);
  data.P_GOOD_STATE    = (data.ALTITUDE_BMP > -50 && data.ALTITUDE_BMP < 200);
  data.T_GOOD_STATE    = (data.TEMP_IN > 15 && data.TEMP_IN < 50);
  data.GPS_GOOD_STATE  = (data.LAT_GPS != 1000.0 && data.LAT_GPS != 0.0 && data.LONG_GPS != 1000.0 && data.LONG_GPS != 0.0);
  if(data.DEBUG_STATE && (data.ALTITUDE_LAST >= DEBUG_ALT) && (data.ALTITUDE_BMP >= DEBUG_ALT)) {
    data.DEBUG_STATE = false;
  }
  return 0;
}

/*
 * Function: debugState
 * -------------------
 * This function provides debuging information.
 */
int8_t Avionics::debugState() {
  if(data.DEBUG_STATE) {
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
  //CUTDOWN_ENABLE
  // if(!data.CUTDOWN_STATE && (data.ALTITUDE_LAST >= CUTDOWN_ALT) && (data.ALTITUDE_BMP >= CUTDOWN_ALT)) {
    PCB.cutDown(true);
    // smartDelay(CUTDOWN_TIME);
    // PCB.cutDown(false);
    data.CUTDOWN_STATE = true;
  // }
  return 0;
}

/*
 * Function: sendSATCOMS
 * -------------------
 * This function sends the current data frame over the ROCKBLOCK IO.
 */
int8_t Avionics::sendSATCOMS() {
  data.RB_GOOD_STATE  = false;
  String messageToSend = writeState();
  data.RB_SENT_COMMS++;
  RBModule.write(data.COMMS_BUFFER, messageToSend.length());
  return 0;
}

/*
 * Function: sendAPRS
 * -------------------
 * This function sends the current data frame over the APRS RF IO.
 */
int8_t Avionics::sendAPRS() {
  String messageToSend = writeState();
  radioModule.write(data.COMMS_BUFFER, messageToSend.length());
  return 0;
}

/*
 * Function: sendCAN
 * -------------------
 * This function sends the current data frame over the CAN BUS IO.
 */
int8_t Avionics::sendCAN() {
  data.CAN_GOOD_STATE = false;
  String messageToSend = writeState();
  canModule.write(data.COMMS_BUFFER, messageToSend.length());
  return 0;
}

/*
 * Function: displayState
 * -------------------
 * This function displays the current avionics state.
 */
int8_t Avionics::displayState() {
    PCB.writeLED(BAT_GOOD_LED,  (data.VOLTAGE >= 3.63));
    PCB.writeLED(I_GOOD_LED,    (data.CURRENT > 0.0 && data.CURRENT <= 0.5));
    PCB.writeLED(P_GOOD_LED,    (data.ALTITUDE_BMP > -50 && data.ALTITUDE_BMP < 200));
    PCB.writeLED(T_GOOD_LED,    (data.TEMP_IN > 15 && data.TEMP_IN < 50));
    PCB.writeLED(CAN_GOOD_LED,  (data.CAN_GOOD_STATE));
    PCB.writeLED(RB_GOOD_LED,   (data.RB_GOOD_STATE));
    PCB.writeLED(GPS_GOOD_LED,  (data.LAT_GPS != 1000.0 && data.LAT_GPS != 0.0 && data.LONG_GPS != 1000.0 && data.LONG_GPS != 0.0));
    PCB.writeLED(HEARTBEAT_LED, (data.HEARTBEAT_STATE));
  return 0;
}

/*
 * Function: printState
 * -------------------
 * This function prints the current avionics state.
 */
int8_t Avionics::printState() {
  Serial.print(data.TIME);
  Serial.print(",");
  Serial.print(data.LOOP_RATE);
  Serial.print(",");
  Serial.print(data.VOLTAGE);
  Serial.print(",");
  Serial.print(data.CURRENT);
  Serial.print(",");
  Serial.print(data.ALTITUDE_BMP);
  Serial.print(",");
  Serial.print(data.ASCENT_RATE);
  Serial.print(",");
  Serial.print(data.TEMP_IN);
  Serial.print(",");
  Serial.print(data.TEMP_EXT);
  Serial.print(",");
  Serial.print(String(data.LAT_GPS, 4));
  Serial.print(",");
  Serial.print(String(data.LONG_GPS, 4));
  Serial.print(",");
  Serial.print(data.SPEED_GPS);
  Serial.print(",");
  Serial.print(data.ALTITUDE_GPS);
  Serial.print(",");
  Serial.print(data.PRESS_BMP);
  Serial.print(",");
  Serial.print(data.RB_SENT_COMMS);
  Serial.print(",");
  Serial.print(data.CUTDOWN_STATE);
  Serial.print("\n");
  return 0;
}

/*
 * Function: writeState
 * -------------------
 * This function compresses the data frame into a bit stream.
 */
String Avionics::writeState() {
  String messageToSend = "";
  messageToSend += data.TIME;
  messageToSend += ",";
  messageToSend += data.LOOP_RATE;
  messageToSend += ",";
  messageToSend += data.VOLTAGE;
  messageToSend += ",";
  messageToSend += data.CURRENT;
  messageToSend += ",";
  messageToSend += data.ALTITUDE_BMP;
  messageToSend += ",";
  messageToSend += data.ASCENT_RATE;
  messageToSend += ",";
  messageToSend += data.TEMP_IN;
  messageToSend += ",";
  messageToSend += data.TEMP_EXT;
  messageToSend += ",";
  messageToSend += String(data.LAT_GPS, 4);
  messageToSend += ",";
  messageToSend += String(data.LONG_GPS, 4);
  messageToSend += ",";
  messageToSend += data.SPEED_GPS;
  messageToSend += ",";
  messageToSend += data.ALTITUDE_GPS;
  messageToSend += ",";
  messageToSend += data.PRESS_BMP;
  messageToSend += ",";
  messageToSend += data.RB_SENT_COMMS;
  messageToSend += ",";
  messageToSend += data.CUTDOWN_STATE;
  messageToSend += "\n";
  return messageToSend;
}

/*
 * Function: logFatalError
 * -------------------
 * This function logs information if there has been a Fatal error.
 */
void Avionics::logFatalError(const char* debug) {
  PCB.faultLED();
  dataFile = SD.open("log.txt", FILE_WRITE);
  if(dataFile) {
    dataFile.print("FATAL ERROR: ");
    dataFile.print(debug);
    dataFile.print("...\n");
    dataFile.close();
  }
  if(data.DEBUG_STATE) {
    Serial.print("FATAL ERROR: ");
    Serial.print(debug);
    Serial.print("...\n");
  }
}
