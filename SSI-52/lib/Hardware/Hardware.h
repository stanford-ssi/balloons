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
#include "../../src/Config.h"

class Hardware {
public:
/**********************************  SETUP  ***********************************/
  void init();
/********************************  FUNCTIONS  *********************************/
  void logToSDCard();
  void updateStatusLEDs();
  uint8_t writeLED(uint8_t PIN, bool green);
  uint8_t faultLED();
  uint8_t cutDown(bool on);
private:
/*********************************  HELPERS  **********************************/
  int8_t readData();
/*********************************  OBJECTS  **********************************/
  Adafruit_MCP23017 mcp;
};

#endif
