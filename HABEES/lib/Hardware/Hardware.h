/*
  Stanford Student Space Initiative
  Balloons | HABEES | January 2017
  Davy Ragland | dragland@stanford.edu

  File: Hardware.h
  --------------------------
  Interface to PCB hardware.
*/

#ifndef HARDWARE_H
#define HARDWARE_H

#include <Adafruit_MCP23017.h>
#include <PID_v1.h>
#include "../../src/Config.h"

class Hardware {
public:
/**********************************  SETUP  ***********************************/
  Hardware() : pid(&PIDTempVar, &PIDOutVar, &PIDSetVar, 2, 5, 1, DIRECT) {}
  void init();
/********************************  FUNCTIONS  *********************************/
  void writeLED(uint8_t PIN, bool green);
  void faultLED();
  void heater(double temp);
  void cutDown(bool on);
  void watchdog();
private:
/*********************************  OBJECTS  **********************************/
  Adafruit_MCP23017 mcp;
  PID pid;
  double PIDOutVar;
  double PIDTempVar;
  double PIDSetVar = PID_SETPOINT;
};

#endif
