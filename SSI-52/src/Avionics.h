/*
  Stanford Student Space Initiative
  Balloons | HABEES | January 2017
  Davy Ragland | dragland@stanford.edu

  File: Avionics.h
  --------------------------
  Primary avionics code.
*/

#ifndef AVIONICS_H
#define AVIONICS_H

#include "config.h"

class Avionics {
public:
  void init();

  void updateData();
  void evaluateState();
  void sendComms();
  void sleep();

private:
  int8_t readData();
  int8_t logData();
  int8_t printData();

  int8_t debug();
  int8_t runHeaters();
  int8_t runCutdown();

  int8_t sendSATCOMS();
  int8_t sendAPRS();
  int8_t sendCAN();

  void faultLED();
  void logFatalError(const char*);
};

#endif
