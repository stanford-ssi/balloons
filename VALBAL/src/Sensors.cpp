/*
  Stanford Student Space Initiative
  Balloons | VALBAL | February 2017
  Davy Ragland | dragland@stanford.edu
  Michal Adamkiewicz | mikadam@stanford.edu

  File: Sensors.cpp
  --------------------------
  Implimentation of Sensors.h
*/

#include "Sensors.h"

/**********************************  SETUP  ***********************************/
/*
  function: init
  ---------------------------------
  This function initializes the sensor hardware.
*/
bool Sensors::init() {
  bool sucess = true;
  if (!bme1.begin()) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    sucess = false;
  }
  if (!bme2.begin()) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    sucess = false;
  }
  if (!bme3.begin()) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    sucess = false;
  }
  if (!bme4.begin()) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    sucess = false;
  }
  return sucess;
}

/********************************  FUNCTIONS  *********************************/
/*
  function: getTime
  ---------------------------------
  This function returns the time as a fixed length string.
*/
uint32_t Sensors::getTime() {
  return millis();
}

/*
  function: getVoltage
  ---------------------------------
  This function gets the battery voltage.
*/
double Sensors::getVoltage() {
  return (analogRead(VBAT_PIN) / 310.0) * 4;
}

/*
  function: getCurrent
  ---------------------------------
  This function gets the total current draw.
*/
double Sensors::getCurrent() {
  return -1;
}

/*
  function: getTempIn
  ---------------------------------
  This function returns a sensor fused reading.
*/
double Sensors::getTemp() {
  double temp_1 = bme1.readTemperature();
  double temp_2 = bme2.readTemperature();
  double temp_3 = bme3.readTemperature();
  double temp_4 = bme4.readTemperature();
  return (temp_1 + temp_2 + temp_3 + temp_4) / 4;
}

/*
  function: getPressure
  ---------------------------------
  This function returns a sensor fused reading.
*/
double Sensors::getPressure() {
  double press_1 = bme1.readPressure();
  double press_2 = bme2.readPressure();
  double press_3 = bme3.readPressure();
  double press_4 = bme4.readPressure();
  return (press_1 + press_2 + press_3 + press_4) / 4;
}

/*
  function: getAltitude
  ---------------------------------
  This function returns a sensor fused reading.
*/
double Sensors::getAltitude() {
  ALTITUDE_LAST = ALTITUDE_CURR;
  double altitude_1 = bme1.readAltitude(1013.25);
  double altitude_2 = bme2.readAltitude(1013.25);
  double altitude_3 = bme3.readAltitude(1013.25);
  double altitude_4 = bme4.readAltitude(1013.25);
  ALTITUDE_CURR = (altitude_1 + altitude_2 + altitude_3 + altitude_4) / 4;
  return ALTITUDE_CURR;
}

/*
  function: getAscentRate
  ---------------------------------
  This function returns the current ascent rate.
*/
double Sensors::getAscentRate() {
  float ascentRateTotal = 0;
  for (int i = 0; i < BUFFER_SIZE - 1; i++) ASCENT_BUFFER[i] = ASCENT_BUFFER[i + 1];
  ASCENT_BUFFER[BUFFER_SIZE - 1] = (ALTITUDE_CURR - ALTITUDE_LAST) / ((millis() - ASCENT_RATE_LAST) / 1000.0);
  ASCENT_RATE_LAST = millis();
  for (int i = 0; i < BUFFER_SIZE; i++) ascentRateTotal += ASCENT_BUFFER[i];
  return  ascentRateTotal / BUFFER_SIZE;
}
