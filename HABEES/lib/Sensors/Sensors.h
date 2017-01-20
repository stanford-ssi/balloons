/*
  Stanford Student Space Initiative
  Balloons | HABEES | January 2017
  Davy Ragland | dragland@stanford.edu

  File: Sensors.h
  --------------------------
  Interface to sensors on avionics hardware.
*/

#ifndef SENSORS_H
#define SENSORS_H

#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_MAX31855.h>
#include <Adafruit_INA219.h>
#include <TimeLib.h>
#include "../../src/Config.h"

class Sensors {
public:
/**********************************  SETUP  ***********************************/
  Sensors() :
    bme1(BMP_CS1),
    bme2(BMP_CS2),
    thermocouple(THERMOCPL_CS) {
  }
  bool        init();
/********************************  FUNCTIONS  *********************************/
  const char* getTime();
  double      getVoltage();
  double      getCurrent();
  double      getTempOut();
  double      getTempIn();
  double      getPressure();
  double      getAltitude();
private:
/*********************************  HELPERS  **********************************/
  void        convertDigits(uint8_t start, uint8_t digits);
  void        convertYear(uint8_t start, int year);
/*********************************  OBJECTS  **********************************/
  char buf[20] = {'\0'};
  Adafruit_BMP280 bme1;
  Adafruit_BMP280 bme2;
  Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
  Adafruit_MAX31855 thermocouple;
  Adafruit_INA219 inaRadio;
  Adafruit_INA219 inaCutdown;
  Adafruit_INA219 inaHeater;
};

#endif
