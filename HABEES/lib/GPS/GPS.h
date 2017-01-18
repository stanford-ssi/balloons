/*
  Stanford Student Space Initiative
  Balloons | HABEES | January 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | satedjarati@stanford.edu

  File: GPS.h
  --------------------------
  Interface to Ublox NEO-M8Q GPS module.
*/

#ifndef GPS_H
#define GPS_H

#include <TinyGPS++.h>
#include "../../src/Config.h"

class GPS {
public:
/**********************************  SETUP  ***********************************/
  int8_t init();
/********************************  FUNCTIONS  *********************************/
  float  getLatitude();
  float  getLongitude();
  float  getAltitude();
  float  getSpeed();
  void   smartDelay(unsigned long ms);
private:
/*********************************  HELPERS  **********************************/
  void   setFlightMode();
  void   sendUBX(uint8_t *MSG, uint8_t len);
  bool   getUBX_ACK(uint8_t *MSG);
/*********************************  OBJECTS  **********************************/
  TinyGPSPlus tinygps;
};

#endif