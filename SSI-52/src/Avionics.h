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
#include "Data.h"

class Avionics {
public:
/**********************************  SETUP  ***********************************/
  void init();
/********************************  FUNCTIONS  *********************************/
  void updateData();
  void evaluateState();
  void sendComms();
  void sleep();

private:
/*********************************  HELPERS  **********************************/
  int8_t readData();
  int8_t logData();
  int8_t printData();
  int8_t debug();
  int8_t runHeaters();
  int8_t runCutdown();
  int8_t sendSATCOMS();
  int8_t sendAPRS();
  int8_t sendCAN();
  void   faultLED();
  void   logFatalError(const char*);

  DataFrame DATA;
};

#endif
