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
  Sensors() :
    bme1(BMP_CS1),
    bme2(BMP_CS2),
    thermocouple(THERMOCPL_CS) {
  }
  void init();
/********************************  FUNCTIONS  *********************************/
  double getTempOut();
  double getTempIn();
  double getPressure();
  double getAltitude();
  int8_t val;
private:
/*********************************  HELPERS  **********************************/
  int8_t readData();

  Adafruit_BMP280 bme1;
  Adafruit_BMP280 bme2;
  Adafruit_MAX31855 thermocouple;
  Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
};

#endif
