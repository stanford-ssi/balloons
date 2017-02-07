/*
  Stanford Student Space Initiative
  Balloons | HABEES | January 2017
  Davy Ragland | dragland@stanford.edu

  File: avionics.h
  --------------------------
  Primary avionics code.
*/

#ifndef AVIONICS_H
#define AVIONICS_H

#include "Config.h"
#include "Cutdown.h"
#include "Data.h"
#include <SD.h>
#include <Sensors.h>
#include <Hardware.h>
#include <GPS.h>
#include <RockBLOCK.h>
#include <APRS.h>
#include <CAN.h>

class Avionics {
public:
/**********************************  SETUP  ***********************************/
  Avionics() :
    PCB(WATCHDOG_PIN, FAULT_PIN, HEATER_PIN, CUTDOWN_PIN, PID_SETPOINT, ANALOG_RES, ANALOG_MAX, LOOP_RATE),
    sensors(VBAT_PIN, BMP_CS1, BMP_CS2, THERMOCPL_CS),
    gpsModule(GPS_ENABLE, GPS_BAUD, GPS_LOCK_TIME) {
  }
  void    init();
/********************************  FUNCTIONS  *********************************/
  void    updateData();
  void    evaluateState();
  void    sendComms();
  void    sleep();
  bool    finishedSetup();

private:
/*********************************  HELPERS  **********************************/
  bool    readData();
  bool    logData();

  bool    calcState();
  bool    debugState();
  bool    runHeaters();
  bool    runCutdown();

  bool    sendSATCOMS();
  bool    sendAPRS();
  bool    sendCAN();

  int16_t compressData();
  void    parseCommand(int16_t len);

  void    calcVitals();
  void    calcDebug();
  void    calcCutdown();
  void    calcAscent();

  void    displayState();
  void    printState();

  void    printHeader();
  void    logHeader();

  void    logAlert(const char*, bool fatal);
  void    watchdog();
/*********************************  OBJECTS  **********************************/
  char COMMS_BUFFER[BUFFER_SIZE];
  DataFrame data;
  File dataFile;
  Hardware PCB;
  Sensors sensors;
  GPS gpsModule;
  RockBLOCK RBModule;
  APRS radioModule;
  CAN canModule;
};

#endif
