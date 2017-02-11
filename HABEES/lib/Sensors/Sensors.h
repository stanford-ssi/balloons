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

class Sensors {
public:
/**********************************  SETUP  ***********************************/
  Sensors(uint8_t VBAT_PIN_NUM, uint8_t BMP_CS1_NUM, uint8_t BMP_CS2_NUM, uint8_t THERMOCPL_CS_NUM, uint16_t BUFFER_SIZE) :
    VBAT_PIN(VBAT_PIN_NUM),
    bme1(BMP_CS1_NUM),
    bme2(BMP_CS2_NUM),
    thermocouple(THERMOCPL_CS_NUM),
    BUFFER_SIZE(BUFFER_SIZE) {
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
  double      getAscentRate();
private:
/*********************************  HELPERS  **********************************/
  void        convertDigits(uint8_t start, uint8_t digits);
  void        convertYear(uint8_t start, int year);
/*********************************  OBJECTS  **********************************/
  uint8_t VBAT_PIN;
  char buf[20] = {'\0'};
  Adafruit_BMP280 bme1;
  Adafruit_BMP280 bme2;
  Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
  Adafruit_MAX31855 thermocouple;
  Adafruit_INA219 inaRadio;
  Adafruit_INA219 inaCutdown;
  Adafruit_INA219 inaHeater;
  uint16_t BUFFER_SIZE;
  float    ASCENT_BUFFER[200];
  double   ALTITUDE_CURR;
  double   ALTITUDE_LAST;
};

#endif
