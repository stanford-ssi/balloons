/*
  Stanford Student Space Initiative
  Balloons | VALBAL | February 2017
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
#include "Controller.h"
#include <SD.h>
#include <GPS.h>
#include <RockBLOCK.h>

class Avionics {
public:
/**********************************  SETUP  ***********************************/
  Avionics() :
    PCB(),
    sensors(),
    gpsModule(GPS_ENABLE, GPS_BAUD, GPS_LOCK_TIME),
    RBModule(RB_SLEEP, RB_BAUD) {
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
  bool    runValve();
  bool    runBalast();
  bool    runCutdown();
  bool    sendSATCOMS();
  void    parseCommand(int16_t len);
  void    calcVitals();
  void    calcDebug();
  void    calcCutdown();
  void    calcAscent();
  void    printHeader();
  void    logHeader();
  void    logAlert(const char*, bool fatal);
  void    printState();
  bool    logData();
  int16_t compressData();
/*********************************  OBJECTS  **********************************/
  char COMMS_BUFFER[BUFFER_SIZE];
  DataFrame data;
  File dataFile;
  Hardware PCB;
  Sensors sensors;
  Controller computer;
  GPS gpsModule;
  RockBLOCK RBModule;
};

#endif
