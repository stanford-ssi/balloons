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
  PCB.init();
  if(!Serial) PCB.faultLED();
  Serial.print("Stanford Student Space Initiative Balloons Launch ");
  Serial.print(MISSION_NUMBER);
  Serial.print('\n');
  Serial.print(CSV_DATA_HEADER);
  Serial.print('\n');

  if(!SD.begin(SD_CS)) PCB.faultLED();
  dataFile = SD.open("data.txt", FILE_WRITE);
  if(!dataFile) PCB.faultLED();
  dataFile.print("Stanford Student Space Initiative Balloons Launch ");
  dataFile.print(MISSION_NUMBER);
  dataFile.print('\n');
  dataFile.print(CSV_DATA_HEADER);
  dataFile.print('\n');

  dataFile.close();
  if(sensors.init()     < 0) logAlert("unable to initialize Sensors", true);
  if(gpsModule.init()   < 0) logAlert("unable to initialize GPS", true);
  // if(RBModule.init()    < 0) logAlert("unable to initialize RockBlock", true);
  if(radioModule.init() < 0) logAlert("unable to initialize radio", true);
  if(canModule.init()   < 0) logAlert("unable to initialize CAN BUS", true);
  watchdog();
  data.SETUP_STATE = false;
}

/********************************  FUNCTIONS  *********************************/
/*
 * Function: updateData
 * -------------------
 * This function handles basic flight data collection.
 */
void Avionics::updateData() {
  if(readData()    < 0) logAlert("unable to read Data", true);
  if(logData()     < 0) logAlert("unable to log Data", true);
  watchdog();
}

/*
 * Function: evaluateState
 * -------------------
 * This function intelligently reacts to the current data frame.
 */
void Avionics::evaluateState() {
  if(calcState()   < 0) logAlert("unable to calculate state", true);
  if(debugState()  < 0) logAlert("unable to debug state", true);
  if(runHeaters()  < 0) logAlert("unable to run heaters", true);
  if(runCutdown()  < 0) logAlert("unable to run cutdown", true);
  watchdog();
}

/*
 * Function: sendComms
 * -------------------
 * This function sends the current data frame down.
 */
void Avionics::sendComms() {
  if((millis() - data.COMMS_LAST) < COMMS_RATE) return;
  if(writeState()  < 0) logAlert("unable to write to COMMS buffer", true);
  if(sendSATCOMS() < 0) logAlert("unable to communicate over RB", true);
  if(sendAPRS()    < 0) logAlert("unable to communicate over APRS", true);
  if(sendCAN()     < 0) logAlert("unable to communicate over CAN", true);
  data.COMMS_LAST = millis();
  watchdog();
}

/*
 * Function: sleep
 * -------------------
 * This function sleeps at the end of the loop.
 */
void Avionics::sleep() {
  gpsModule.smartDelay(LOOP_RATE);
  watchdog();
}

/*
 * Function: finishedSetup
 * -------------------
 * This function returns true if the avionics has completed setup.
 */
bool Avionics::finishedSetup() {
  return !data.SETUP_STATE;
}

/*********************************  HELPERS  **********************************/
/*
 * Function: readData
 * -------------------
 * This function updates the current data frame.
 */
int8_t Avionics::readData() {
  data.LOOP_GOOD_STATE = !data.LOOP_GOOD_STATE;
  data.LOOP_RATE       = millis() - data.LOOP_START;
  data.LOOP_START      = millis();
  data.TIME            = sensors.getTime();
  data.ALTITUDE_LAST   = data.ALTITUDE_BMP;
  data.VOLTAGE         = sensors.getVoltage();
  data.CURRENT         = sensors.getCurrent();
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
  dataFile.print(',');
  dataFile.print(data.LOOP_RATE);
  dataFile.print(',');
  dataFile.print(data.VOLTAGE);
  dataFile.print(',');
  dataFile.print(data.CURRENT);
  dataFile.print(',');
  dataFile.print(data.ALTITUDE_BMP);
  dataFile.print(',');
  dataFile.print(data.ASCENT_RATE);
  dataFile.print(',');
  dataFile.print(data.TEMP_IN);
  dataFile.print(',');
  dataFile.print(data.TEMP_EXT);
  dataFile.print(',');
  dataFile.print(data.LAT_GPS, 4);
  dataFile.print(',');
  dataFile.print(data.LONG_GPS, 4);
  dataFile.print(',');
  dataFile.print(data.SPEED_GPS);
  dataFile.print(',');
  dataFile.print(data.ALTITUDE_GPS);
  dataFile.print(',');
  dataFile.print(data.PRESS_BMP);
  dataFile.print(',');
  dataFile.print(data.RB_SENT_COMMS);
  dataFile.print(',');
  dataFile.print(data.CUTDOWN_STATE);
  dataFile.print('\n');
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
  data.I_GOOD_STATE   = (data.CURRENT > 0.0 && data.CURRENT <= 0.5);
  data.P_GOOD_STATE    = (data.ALTITUDE_BMP > -50 && data.ALTITUDE_BMP < 200);
  data.T_GOOD_STATE    = (data.TEMP_IN > 15 && data.TEMP_IN < 50);
  data.GPS_GOOD_STATE  = (data.LAT_GPS != 1000.0 && data.LAT_GPS != 0.0 && data.LONG_GPS != 1000.0 && data.LONG_GPS != 0.0);

  if(data.DEBUG_STATE && (data.ALTITUDE_LAST >= DEBUG_ALT) && (data.ALTITUDE_BMP >= DEBUG_ALT)) data.DEBUG_STATE = false;

  if(CUTDOWN_GPS_ENABLE && data.GPS_GOOD_STATE &&
    ((GPS_FENCE_LAT_MIN < data.LAT_GPS) && (data.LAT_GPS < GPS_FENCE_LAT_MAX)) &&
    ((GPS_FENCE_LON_MIN < data.LONG_GPS) && (data.LONG_GPS < GPS_FENCE_LON_MAX))
  ) data.SHOULD_CUTDOWN = true;

  if(CUTDOWN_ALT_ENABLE && !data.CUTDOWN_STATE &&
    (data.ALTITUDE_LAST >= CUTDOWN_ALT) &&
    (data.ALTITUDE_BMP >= CUTDOWN_ALT)
  ) data.SHOULD_CUTDOWN = true;

  for (int i = 0; i < BUFFER_SIZE - 1; i++) data.ASCENT_BUFFER[i] = data.ASCENT_BUFFER[i + 1];
  data.ASCENT_BUFFER[BUFFER_SIZE - 1] = (data.ALTITUDE_BMP - data.ALTITUDE_LAST) / ((millis() - data.ASCENT_RATE_LAST) / 1000.0);
  data.ASCENT_RATE_LAST = millis();
  float ascentRateTotal = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) ascentRateTotal += data.ASCENT_BUFFER[i];
  data.ASCENT_RATE =  ascentRateTotal / BUFFER_SIZE;
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
  PCB.heater(data.TEMP_IN);
  return 0;
}

/*
 * Function: runCutdown
 * -------------------
 * This function cuts down the payload if nessisary.
 */
int8_t Avionics::runCutdown() {
  if(data.CUTDOWN_STATE) return 0;
  if(data.SHOULD_CUTDOWN) {
    PCB.cutDown(true);
    gpsModule.smartDelay(CUTDOWN_TIME);
    PCB.cutDown(false);
    data.CUTDOWN_STATE = true;
    logAlert("completed cutdown", false);
  }
  return 0;
}

/*
 * Function: sendSATCOMS
 * -------------------
 * This function sends the current data frame over the ROCKBLOCK IO.
 */
int8_t Avionics::sendSATCOMS() {
logAlert("sending Rockblock message", false);
  data.RB_GOOD_STATE  = false;
  data.RB_SENT_COMMS++;
  int8_t ret = RBModule.writeRead(data.COMMS_BUFFER, data.COMMS_LENGTH);
  if(ret < 0) return -1;
  if(ret > 0) parseCommand(ret);
  return 0;
}

/*
 * Function: sendAPRS
 * -------------------
 * This function sends the current data frame over the APRS RF IO.
 */
int8_t Avionics::sendAPRS() {
  radioModule.write(data.COMMS_BUFFER, data.COMMS_LENGTH);
  logAlert("sent APRS message", false);
  return 0;
}

/*
 * Function: sendCAN
 * -------------------
 * This function sends the current data frame over the CAN BUS IO.
 */
int8_t Avionics::sendCAN() {
  data.CAN_GOOD_STATE = false;
  canModule.write(data.COMMS_BUFFER, data.COMMS_LENGTH);
  logAlert("sent CAN message", false);
  return 0;
}

/*
 * Function: displayState
 * -------------------
 * This function displays the current avionics state.
 */
int8_t Avionics::displayState() {
    PCB.writeLED(BAT_GOOD_LED,  data.BAT_GOOD_STATE);
    PCB.writeLED(I_GOOD_LED,    data.I_GOOD_STATE);
    PCB.writeLED(P_GOOD_LED,    data.P_GOOD_STATE);
    PCB.writeLED(T_GOOD_LED,    data.T_GOOD_STATE);
    PCB.writeLED(CAN_GOOD_LED,  data.CAN_GOOD_STATE);
    PCB.writeLED(RB_GOOD_LED,   data.RB_GOOD_STATE);
    PCB.writeLED(GPS_GOOD_LED,  data.GPS_GOOD_STATE);
    PCB.writeLED(LOOP_GOOD_LED, data.LOOP_GOOD_STATE);
  return 0;
}

/*
 * Function: printState
 * -------------------
 * This function prints the current avionics state.
 */
int8_t Avionics::printState() {
  Serial.print(data.TIME);
  Serial.print(',');
  Serial.print(data.LOOP_RATE);
  Serial.print(',');
  Serial.print(data.VOLTAGE);
  Serial.print(',');
  Serial.print(data.CURRENT);
  Serial.print(',');
  Serial.print(data.ALTITUDE_BMP);
  Serial.print(',');
  Serial.print(data.ASCENT_RATE);
  Serial.print(',');
  Serial.print(data.TEMP_IN);
  Serial.print(',');
  Serial.print(data.TEMP_EXT);
  Serial.print(',');
  Serial.print(data.LAT_GPS, 4);
  Serial.print(',');
  Serial.print(data.LONG_GPS, 4);
  Serial.print(',');
  Serial.print(data.SPEED_GPS);
  Serial.print(',');
  Serial.print(data.ALTITUDE_GPS);
  Serial.print(',');
  Serial.print(data.PRESS_BMP);
  Serial.print(',');
  Serial.print(data.RB_SENT_COMMS);
  Serial.print(',');
  Serial.print(data.CUTDOWN_STATE);
  Serial.print('\n');
  return 0;
}

/*
 * Function: writeState
 * -------------------
 * This function compresses the data frame into a bit stream.
 */
int16_t Avionics::writeState() {
  int16_t length = 0;
  for(uint16_t i = 0; i < BUFFER_SIZE; i++) {
    data.COMMS_BUFFER[0] = '!';
    length++;
  }
  // double num = 123412341234.123456789;
  // char output[50];
  // snprintf(output, 50, "%.2f", num);
  return length;
}

/*
 * Function: logAlert
 * -------------------
 * This function logs important information whenever a specific event occurs
 */
void Avionics::logAlert(const char* debug, bool fatal) {
  PCB.faultLED();
  dataFile = SD.open("log.txt", FILE_WRITE);
  if(dataFile) {
    dataFile.print(data.TIME);
    dataFile.print(',');
    if(fatal) dataFile.print("FATAL ERROR!!!!!!!!!!: ");
    dataFile.print(debug);
    dataFile.print("...\n");
    dataFile.close();
  }
  if(data.DEBUG_STATE) {
    Serial.print(data.TIME);
    Serial.print(',');
    if(fatal) Serial.print("FATAL ERROR!!!!!!!!!!: ");
    Serial.print(debug);
    Serial.print("...\n");
  }
}

/*
 * Function: parseCommand
 * -------------------
 * This function parses the command received from the RockBLOCK.
 */
void Avionics::parseCommand(int8_t len) {
  if(strncmp(data.COMMS_BUFFER, CUTDOWN_COMAND, len)) {
    data.SHOULD_CUTDOWN = true;
  }
}

/*
 * Function: watchdog
 * -------------------
 * This function pulses the watchdog IC in order to ensure that avionics recovers from a fatal crash.
 */
void Avionics::watchdog() {
  if((millis() - data.WATCHDOG_LAST) < WATCHDOG_RATE) return;
  PCB.watchdog();
  data.WATCHDOG_LAST = millis();
}
