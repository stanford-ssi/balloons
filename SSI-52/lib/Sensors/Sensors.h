/*
  Stanford Student Space Initiative
  Balloons | HABEES | January 2017
  Davy Ragland | dragland@stanford.edu

  File: Sensors.h
  --------------------------
  Interface to sensors on avionics hardware.
*/

#ifndef Sensors_H
#define Sensors_H

#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_MAX31855.h>
#include "../../src/Config.h"

class Sensors {
public:
/**********************************  SETUP  ***********************************/
  void init();
/********************************  FUNCTIONS  *********************************/
  double getTempOut();
  double getTempIn();
  double getPressure();
  double getAltitude();
  Sensors() : val(2) { }
  int8_t val;
private:
/*********************************  HELPERS  **********************************/
  int8_t readData();

};

#endif
