/*
  Stanford Student Space Initiative
  Balloons | HABEES | January 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | satedjarati@stanford.edu

  File: RockBlock.cpp
  --------------------------
  Implimentation of RockBlock.h
*/

#include "RockBLOCK.h"

IridiumSBD isbd(Serial3, RB_SLEEP);

/**********************************  SETUP  ***********************************/
/*
  function: init
  ---------------------------------
  This function initializes the RockBlock module.
*/
int8_t RockBLOCK::init() {
  isbd.attachConsole(Serial);
  isbd.attachDiags(Serial);
  isbd.setPowerProfile(1);
  Serial3.begin(RB_BAUD);
  delay(5000);
  // isbd.begin();
  return 0;
}

/********************************  FUNCTIONS  *********************************/
/*
  function: write
  ---------------------------------
  This function writes a bitstream across the communication interface.
*/
int8_t RockBLOCK::write(char* buff, uint8_t len) {
  return 0;
}

/*
   function: satelliteTransmission
   usage: satelliteTransmission();
   ---------------------------------
   This function sends messages through the Rock Block.
*/
int     messagesSent  = 0;
size_t bufferSize           = 0;
void satelliteTransmission(){
  Serial.println("Beginning to talk to the RockBLOCK...");
  messagesSent += 1;
  Serial.println("Transmitting message: ");
  Serial.println((String)messagesSent);
  Serial.println("Beginning to talk to the RockBLOCK...");
  Serial.println("Sending RB message");
  char outBuffer[200];
  String messageToSend = "HELLO WORLD";
  messageToSend += '!';

  for(uint8_t i = 0; i < messageToSend.length(); i++) {
    outBuffer[i] = messageToSend[i];
  }
  uint8_t rxBuffer[200] = {0};
  for(uint8_t i = 0; i < messageToSend.length(); i++){
    rxBuffer[i] = outBuffer[i];
  }
  bufferSize = sizeof(rxBuffer);
  isbd.sendReceiveSBDBinary(rxBuffer, messageToSend.length(), rxBuffer, bufferSize);
  Serial.println("Going to sleep mode");
  delay(2000);
}


// void checkRB() {
//   if ((powerStates[0] == 0 || powerStates[0] == 1 || powerStates[0] == 3) && (minutes >= RBRestartTimer)) {
//     powerStates[0] = 1;
//     EEPROM.write(10, 1);
//     digitalWrite(RB_GATE, HIGH);
//     delay(5000);
//     Serial.print("isbd begin return"); Serial.println(isbd.begin());
//     delay(2000);
//     powerStates[0] = 2;
//     EEPROM.write(10, 2);
//     RBRestartTimer = minutes + RB_RESTART_TIMER;
//   }
// }

// void satelliteTransmission() {
//   if ((minutes > commBeaconInterval)/* && isbd.isAsleep()*/) {
//     internalTemperatureAverage = internalTemperatureAverage / temperatureCount;
//     neckTemperatureAverage = neckTemperatureAverage /temperatureCount;
//     RBmsg = messagesSent;
//     RBvel = averageAscentRate;
//     RBaltB = DAlt;
//     RBaltG = alt;
//     RBlat = lati;
//     RBlong = longi;
//     RBtemp = internalTemperatureAverage;
//     RBbat = batteryCapacity / initialBatteryCapacity;
//     RBvoltP = batteryVoltage;
//     RBavgI = currentAvg / currentNum;
//     RBmaxI = currentMax * 1000.0;
//     RBminI = currentMin * 1000.0;
//
//     RBStrongHeat = strongHeaterOn;
//     RBMildHeat = weakHeaterOn;
//
//     RBreadback = doReadback;
//
//     RBcurrentGPSmax = currentGPSMax;
//     RBcurrentGPSavg = currentGPSAvg / currentNum;
//     RBcurrentRBmax = currentRBMax;
//     RBcurrentRBavg = currentRBAvg / currentNum;
//     RBcurrentValmax = currentValMax;
//     RBcurrentValavg = (currentValNum != 0) ? currentValAvg / currentValNum : 0;
//     RBcurrentBalmax = currentBalMax;
//     RBcurrentBalavg = (currentBalNum != 0) ? currentBalAvg / currentBalNum : 0;
//
//     RBVin = valveIncentive;
//     RBBin = ballastIncentive;
//     RBcutdown = cutdownReason; // umm look into that mebeh
//     RBBtime = ballastTime;
//     RBVtime = ventTime;
//     RBheat = mainDutyCycle;
//     RBs1delta = sensor1max - sensor1min;
//     RBs2delta = sensor2max - sensor2min;
//     RBs3delta = sensor3max - sensor3min;
//     RBs4delta = sensor4max - sensor4min;
//     RBmode = (reportMode << 1) + (controlMode << 0);
//     RBLEDon = LEDon;
//
//     RBpowerRB = powerStates[0];
//     RBpowerGPS = powerStates[1];
//     RBpowerHeaters = powerStates[2];
//     //RBpotReading = analogRead(VALVE_POT);
//     RBNeckTemp = neckTemperatureAverage;
//     RBsensors = activeSensors;
//     RBt0 = initAltitudeTime;
//
//
//
//     RBs1rej = log(sensor1rej + 1) / log(2);
//     RBs2rej = log(sensor2rej + 1) / log(2);
//     RBs3rej = log(sensor3rej + 1) / log(2);
//     RBs4rej = log(sensor4rej + 1) / log(2);
//
//     RBatt1time -= RBt0;
//     RBatt2time -= RBt0;
//     RBatt3time -= RBt0;
//     RBatt4time -= RBt0;
//
//     RBIthresh = I_thresh;
//     RBreArmConst = reArmConst;
//
//
//     RBCValveSpeed = SPEED_CONSTANT; RBCValveAltDiff = 1./ALTITUDE_DIFFERENCE_CONSTANT; RBCValveLast = 1./LAST_VALVE_OPENING_CONSTANT; RBCBallastSpeed = BALLASTER_SPEED_CONSTANT;
//     RBCBallastAltDiff = 1./BALLASTER_ALT_DIFF_CONSTANT; RBCBallastLast = 1./BALLASTER_LAST_DROP_CONSTANT; RBCValveSetpoint = ALTITUDE_SETPOINT; RBCBallastSetpoint = BALLAST_ALTITUDE_SETPOINT;
//     RBFValveLast = altitudeSinceLastVent; RBFDropLast = altitudeSinceLastDrop; RBCOpenValve = OPEN_VALVE_VALUE; RBCClosedValve = CLOSED_VALVE_VALUE; RBCCutdownValve = CUTDOWN_VALVE_VALUE;
//     RBCMaxEncoder = MAX_TIME_WITHOUT_ENCODER; RBCDoNothing = DO_NOTHING_TIMER; RBCValTime = MAX_VALVE_ON_MILLIS/1000.; RBCBalTime = MAX_BALLAST_ON_MILLIS/1000.; RBControlMode = controlMode;
//     RBReportMode = reportMode; RBPotMode = forceValveTimedMode; RBTemperatureSetpoint = INTERNAL_TEMP_SETPOINT; RBRBinterval = COMM_BEACON_INTERVAL; RBGPSinterval = GPS_BEACON_INTERVAL;
//     RBValveSpeed = VALVE_MOTOR_SPEED; RBOpeningTime = VALVE_OPEN_BACKUP_TIMER;
//
//     messagesSent += 1;
//     delay(200);
//     Serial.println("Beginning to talk to the RockBLOCK...");
//     Serial.println("Sending RB message");
//
//
//     uint8_t temporary[50] = {0};
//     int len = encodeTelemetry(temporary);
//
//     float var;
//     int adc;
//     uint8_t rxBuffer[200] = {0};
//     int byteidx = 0;
//     int bitidx = 7;
//     for (int i = 0; i < argc; i++) {
//       var = *vars[i];
//       Serial.println(var);
//       if (var > maxs[i]) var = maxs[i];
//       if (var < mins[i]) var = mins[i];
//       adc = round( ( pow(2, bits[i]) - 1 ) * (var - mins[i]) / (maxs[i] - mins[i]));
//       for (int j = bits[i] - 1; j >= 0; j--) {
//         bool bit = adc & (1 << j);
//         if (bit) rxBuffer[byteidx] |= (1 << bitidx);
//         bitidx -= 1;
//         if (bitidx < 0) {
//           bitidx = 7;
//           byteidx += 1;
//         }
//       }
//       if (!reportMode && i == debugThreshold) break; // stop if we're no longer in reduced-land.
//       if (vars[i] == &RBreadback && RBreadback == 0) break;
//     }
//     if (bitidx != 7) byteidx += 1; // Leave the rest of the byte empty between this and possibly callbacks.
//
//     if (len > 0) {
//       for (int k = 0; k < len; k++) {
//         rxBuffer[byteidx] = temporary[k];
//         byteidx += 1;
//       }
//     }
//
//     for (int i = 0; i < (byteidx+1); i++) {
//       uint8_t x = rxBuffer[i];
//       (x & 0x80 ? Serial.print('1') : Serial.print('0'));
//       (x & 0x40 ? Serial.print('1') : Serial.print('0'));
//       (x & 0x20 ? Serial.print('1') : Serial.print('0'));
//       (x & 0x10 ? Serial.print('1') : Serial.print('0'));
//       (x & 0x08 ? Serial.print('1') : Serial.print('0'));
//       (x & 0x04 ? Serial.print('1') : Serial.print('0'));
//       (x & 0x02 ? Serial.print('1') : Serial.print('0'));
//       (x & 0x01 ? Serial.print('1') : Serial.print('0'));
//
//     }
//     Serial.println();
//     // reset variables before transmitting!
//
//     temperatureCount = 0.0;
//     internalTemperatureAverage = 0.0;
//
//     sensor1max = -40000;
//     sensor2max = -40000;
//     sensor3max = -40000;
//     sensor4max = -40000;
//
//     sensor1min = 40000;
//     sensor2min = 40000;
//     sensor3min = 40000;
//     sensor4min = 40000;
//
//     sensor1rej = 0;
//     sensor2rej = 0;
//     sensor3rej = 0;
//     sensor4rej = 0;
//
//     RBVatt = 0;
//     RBBatt = 0;
//
//     RBatt1type = -2;
//     RBatt1time = 0;
//
//     RBatt2type = -2;
//     RBatt2time = 0;
//
//     RBatt3type = -2;
//     RBatt3time = 0;
//
//     RBatt4type = -2;
//     RBatt4time = 0;
//
//     bufferSize = sizeof(rxBuffer);
//     Serial.println("SENDING FORREAL");
//     int ret = isbd.sendReceiveSBDBinary(rxBuffer, byteidx, rxBuffer, bufferSize);
//     if (ret == ISBD_SUCCESS) {
//       currentMin = 10000;
//       currentMax = 0;
//       currentAvg = 0;
//       currentNum = 0;
//       currentGPSMax = 0;
//       currentGPSAvg = 0;
//       currentRBMax = 0;
//       currentRBAvg = 0;
//       currentValMax = 0;
//       currentValAvg = 0;
//       currentBalMax = 0;
//       currentBalAvg = 0;
//       initAltitudeTime += QcurrentDt*QmulFactor*(currentAltitude-1) + Qdt;
//       QcurrentDt = Qdt;
//       QmulFactor = 1;
//       currentAltitude = 0;
//       doReadback = false;
//     }
//     int number = bufferSize;
//     Serial.println("Not going to sleep mode, buddy.");
//     delay(2000);
//     for (int i = 0; i < 9; i++) {
//       returnValuee[i] = 99;
//     }
//     if (number > 0) {
//       iridiumParser(number, rxBuffer);
//     }
//     commBeaconInterval = minutes + COMM_BEACON_INTERVAL;
//   }
// }

// bool ISBDCallback() {
//   if (!inSetup) {
//     loopStartTime = millis();
//     iterateSensors();
//     if (powerStates[2] == 2) {
//       iteratePIDController();
//       getHeaterDutyCycles();
//     }
//     if (powerStates[1] == 2) acquireGPSLock();
//     printToSerialAndLog();
//     elapsedSeconds = (double)(((double)millis() - loopStartTime) / 1000.0);
//     getAscentRates();
//     altitudeController();
//     blinkLED();
//     int derray = (int)(50.0 - (double)((double)millis() - loopStartTime));
//     if (derray > 0) delay(derray);
//     overflowSeconds = (double)(((double)millis() - loopStartTime) / 1000.0) - elapsedSeconds;
//     minutes += (double)(((double)millis() - loopStartTime) / 1000.0 / 60.0);
//   }
//   return true;
// }

//
// void iridiumParser(int number, uint8_t rxBuffer[]) {
//   // First put everything in a character array
//   char fullMessage[200] = {0};
//   for (uint8_t i = 0; i < number; i++) {
//     fullMessage[i] = rxBuffer[i];
//   }
//
//   // Declare separate commands and indexies
//   char firstCommand[100] = {0}; int firstIndex = 0; String firstCommandStr = ""; char *p1_1;
//   char secondCommand[100] = {0}; int secondIndex = 0; String secondCommandStr = ""; char *p2_1;
//   char thirdCommand[100] = {0}; int thirdIndex = 0; String thirdCommandStr = ""; char *p3_1;
//   char fourthCommand[100] = {0}; int fourthIndex = 0; String fourthCommandStr = ""; char *p4_1;
//   char fifthCommand[100] = {0}; int fifthIndex = 0; String fifthCommandStr = ""; char *p5_1;
//   char sixthCommand[100] = {0}; int sixthIndex = 0; String sixthCommandStr = ""; char *p6_1;
//   char seventhCommand[100] = {0}; int seventhIndex = 0; String seventhCommandStr = ""; char *p7_1;
//   char eighthCommand[100] = {0}; int eighthIndex = 0; String eighthCommandStr = ""; char *p8_1;
//
//   // Read the message, scan it, and separate commands
//   int nscanned = sscanf(fullMessage, "%d,%s %d,%s %d,%s %d,%s %d,%s %d,%s %d,%s %d,%s", &firstIndex, firstCommand, &secondIndex, secondCommand,
//                         &thirdIndex, thirdCommand, &fourthIndex, fourthCommand, &fifthIndex, fifthCommand, &sixthIndex, sixthCommand, &seventhIndex,
//                         seventhCommand, &eighthIndex, eighthCommand);
//
//   for (int i = 0; i < 9; i++) {
//     returnValuee[i] = 99;
//   }
//
//   if (nscanned % 2 != 0) return;
//
//
//   // Convert commands to Arduino readable strings then act on commands
//   if (nscanned >= 2) {
//     for (uint8_t i = 0; i < strlen(firstCommand); i++) {
//       firstCommandStr += firstCommand[i];
//     }
//     if (firstIndex == CUTDOWN_INDEX && firstCommandStr.equals(CUTDOWN_COMMAND)) {
//       cutdownBalloon();
//     } else {
//       strtod(firstCommand, &p1_1);
//       if (firstIndex < 0 || firstIndex > 80 || *p1_1) return;
//       constantChanger(firstIndex, firstCommandStr.toFloat());
//     }
//   }
//   if (nscanned >= 4) {
//     for (uint8_t i = 0; i < strlen(secondCommand); i++) {
//       secondCommandStr += secondCommand[i];
//     }
//     if (secondIndex == CUTDOWN_INDEX && secondCommandStr.equals(CUTDOWN_COMMAND)) {
//       cutdownBalloon();
//     } else {
//       strtod(secondCommand, &p2_1);
//       if (secondIndex < 0 || secondIndex > 80 || *p2_1) return;
//       constantChanger(secondIndex, secondCommandStr.toFloat());
//     }
//   }
//   if (nscanned >= 6) {
//     for (uint8_t i = 0; i < strlen(thirdCommand); i++) {
//       thirdCommandStr += thirdCommand[i];
//     }
//     if (thirdIndex == CUTDOWN_INDEX && thirdCommandStr.equals(CUTDOWN_COMMAND)) {
//       cutdownBalloon();
//     } else {
//       strtod(thirdCommand, &p3_1);
//       if (thirdIndex < 0 || thirdIndex > 80 || *p3_1) return;
//       constantChanger(thirdIndex, thirdCommandStr.toFloat());
//     }
//   }
//   if (nscanned >= 8) {
//     for (uint8_t i = 0; i < strlen(fourthCommand); i++) {
//       fourthCommandStr += fourthCommand[i];
//     }
//     if (fourthIndex == CUTDOWN_INDEX && fourthCommandStr.equals(CUTDOWN_COMMAND)) {
//       cutdownBalloon();
//     } else {
//       strtod(fourthCommand, &p4_1);
//       if (fourthIndex < 0 || fourthIndex > 80 || *p4_1) return;
//       constantChanger(fourthIndex, fourthCommandStr.toFloat());
//     }
//   }
//   if (nscanned >= 10) {
//     for (uint8_t i = 0; i < strlen(fifthCommand); i++) {
//       fifthCommandStr += fifthCommand[i];
//     }
//     if (fifthIndex == CUTDOWN_INDEX && fifthCommandStr.equals(CUTDOWN_COMMAND)) {
//       cutdownBalloon();
//     } else {
//       strtod(fifthCommand, &p5_1);
//       if (fifthIndex < 0 || fifthIndex > 80 || *p5_1) return;
//       constantChanger(fifthIndex, fifthCommandStr.toFloat());
//     }
//   }
//   if (nscanned >= 12) {
//     for (uint8_t i = 0; i < strlen(sixthCommand); i++) {
//       sixthCommandStr += sixthCommand[i];
//     }
//     if (sixthIndex == CUTDOWN_INDEX && sixthCommandStr.equals(CUTDOWN_COMMAND)) {
//       cutdownBalloon();
//     } else {
//       strtod(sixthCommand, &p6_1);
//       if (sixthIndex < 0 || sixthIndex > 80 || *p6_1) return;
//       constantChanger(sixthIndex, sixthCommandStr.toFloat());
//     }
//   }
//   if (nscanned >= 14) {
//     for (uint8_t i = 0; i < strlen(seventhCommand); i++) {
//       seventhCommandStr += seventhCommand[i];
//     }
//     if (seventhIndex == CUTDOWN_INDEX && seventhCommandStr.equals(CUTDOWN_COMMAND)) {
//       cutdownBalloon();
//     } else {
//       strtod(seventhCommand, &p7_1);
//       if (seventhIndex < 0 || seventhIndex > 80 || *p7_1) return;
//       constantChanger(seventhIndex, seventhCommandStr.toFloat());
//     }
//   }
//   if (nscanned == 16) {
//     for (uint8_t i = 0; i < strlen(eighthCommand); i++) {
//       eighthCommandStr += eighthCommand[i];
//     }
//     if (eighthIndex == CUTDOWN_INDEX && eighthCommandStr.equals(CUTDOWN_COMMAND)) {
//       cutdownBalloon();
//     } else {
//       strtod(eighthCommand, &p8_1);
//       if (eighthIndex < 0 || eighthIndex > 80 || *p8_1) return;
//       constantChanger(eighthIndex, eighthCommandStr.toFloat());
//     }
//   }
//
//   if (nscanned >= 2) returnValuee[0] = firstIndex;
//   if (nscanned >= 4) returnValuee[1] = secondIndex;
//   if (nscanned >= 6) returnValuee[2] = thirdIndex;
//   if (nscanned >= 8) returnValuee[3] = fourthIndex;
//   if (nscanned >= 10)returnValuee[4] = fifthIndex;
//   if (nscanned >= 12)returnValuee[5] = sixthIndex;
//   if (nscanned >= 14)returnValuee[6] = seventhIndex;
//   if (nscanned == 16)returnValuee[7] = eighthIndex;
//
//   returnValuee[8] = nscanned / 2;
// }
