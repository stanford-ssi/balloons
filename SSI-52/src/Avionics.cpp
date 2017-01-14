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

  data.GPS_SET_SUCESS = false;
  data.RB_SET_SUCESS = false;
  data.CAN_SET_SUCESS = false;

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
  if(data.DEBUG_STATE && (data.ALTITUDE_LAST >= DEBUG_ALT) && (data.ALTITUDE_BMP >= DEBUG_ALT)) {
    data.DEBUG_STATE = false;
  }
  // long    LOOP_START            =     0;
  // double  ALTITUDE_LAST         =     0;
  // float   AscentRateBuffer[BUFFER_SIZE];
  return 0;
}

/*
 * Function: runDebug
 * -------------------
 * This function provides debuging information.
 */
int8_t Avionics::runDebug() {
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
    PCB.writeLED(BAT_GOOD,  (data.VOLTAGE >= 3.63));
    PCB.writeLED(I_GOOD,    (data.CURRENT > 0.0 && data.CURRENT <= 0.5));
    PCB.writeLED(P_GOOD,    (data.ALTITUDE_BMP > -50 && data.ALTITUDE_BMP < 200));
    PCB.writeLED(T_GOOD,    (data.TEMP_IN > 15 && data.TEMP_IN < 50));
    PCB.writeLED(CAN_GOOD,  (data.CAN_SET_SUCESS));
    PCB.writeLED(RB_GOOD,   (data.RB_SET_SUCESS));
    PCB.writeLED(GPS_GOOD,  (data.LAT_GPS != 1000.0 && data.LAT_GPS != 0.0 && data.LONG_GPS != 1000.0 && data.LONG_GPS != 0.0));
    PCB.writeLED(HEARTBEAT, (data.BLINK));
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
  Serial.print("FATAL ERROR: ");
  Serial.print(debug);
  Serial.println("...");
}
