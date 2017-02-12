/*
  Stanford Student Space Initiative
  Balloons | HABEES | February 2017
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
#include "Sensors.h"
#include "Hardware.h"
#include <SD.h>
#include <GPS.h>
#include <RockBLOCK.h>
#include <APRS.h>
#include <CAN.h>

class Avionics {
public:
/**********************************  SETUP  ***********************************/
  Avionics() :
    PCB(),
    sensors(),
    gpsModule(GPS_ENABLE, GPS_BAUD, GPS_LOCK_TIME),
    canModule(CAN_ENABLE, CAN_BAUD),
    RBModule(RB_SLEEP, RB_BAUD),
    radioModule() {
  }
  void    init();
/********************************  FUNCTIONS  *********************************/
  void    updateState();
  void    evaluateState();
  void    actuateState();
  void    logState();
  void    sendComms();
  void    sleep();
  bool    finishedSetup();

private:
/*********************************  HELPERS  **********************************/
  bool    readData();
  bool    calcState();
  bool    debugState();
  bool    runHeaters();
  bool    runCutdown();
  bool    sendCAN();
  bool    sendSATCOMS();
  bool    sendAPRS();
  void    parseCommand(int16_t len);
  void    calcVitals();
  void    calcDebug();
  void    calcCutdown();
  void    calcAscent();
  void    displayState();
  void    printHeader();
  void    logHeader();
  void    logAlert(const char*, bool fatal);
  void    watchdog();
  void    printState();
  bool    logData();
  int16_t compressData();
/*********************************  OBJECTS  **********************************/
  char COMMS_BUFFER[BUFFER_SIZE];
  DataFrame data;
  File dataFile;
  Hardware PCB;
  Sensors sensors;
  GPS gpsModule;
  CAN canModule;
  RockBLOCK RBModule;
  APRS radioModule;
};

#endif
