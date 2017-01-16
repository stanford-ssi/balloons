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
  Hardware() :
  pid(&PID_temp_ptr, &PID_out_ptr, &PID_set_ptr, 2, 5, 1, DIRECT) {}
  void init();
/********************************  FUNCTIONS  *********************************/
  void   logToSDCard();
  void   updateStatusLEDs();
  int8_t writeLED(uint8_t PIN, bool green);
  int8_t faultLED();
  int8_t heater(double temp);
  int8_t cutDown(bool on);
private:
/*********************************  HELPERS  **********************************/
  int8_t readData();
/*********************************  OBJECTS  **********************************/
  Adafruit_MCP23017 mcp;
  PID pid;
  double PID_out_ptr;
  double PID_temp_ptr;
  double PID_set_ptr = PID_SETPOINT;
};

#endif
