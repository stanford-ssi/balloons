/*
  Stanford Student Space Initiative
  Balloons | HABEES | January 2017
  Davy Ragland | dragland@stanford.edu

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
void Sensors::init() {
  if (!bme1.begin()) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
  }
  if (!bme2.begin()) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
  }
  if (! baro.begin()) {
    Serial.println("Could not find a valid MPL3115A2 sensor, check wiring!");
  }
}

/********************************  FUNCTIONS  *********************************/
/*
  function: getVoltage
  ---------------------------------
  This function gets the battery voltage.
*/
double Sensors::getVoltage() {
  return (analogRead(VBAT_PIN) / 310.0) * 4;
}

/*
  function: getTempOut
  ---------------------------------
  This function gets the external temperature.
*/
double Sensors::getTempOut() {
  if (!isnan(thermocouple.readCelsius())) return -1; //return TEMP_EXT;
  else return thermocouple.readCelsius();
}

/*
  function: getTempIn
  ---------------------------------
  This function returns a sensor fused reading.
*/
double Sensors::getTempIn() {
  double temp_1 = bme1.readTemperature();
  double temp_2 = bme2.readTemperature();
  if (temp_1 >= -20 && temp_2 >= -20) return (temp_1 + temp_2) / 2;
  if (temp_1 >= -20 && temp_2 < -20) return temp_1;
  if (temp_2 >= -20 && temp_1 < -20) return temp_2;
  else return baro.getTemperature();
}

/*
  function: getPressure
  ---------------------------------
  This function returns a sensor fused reading.
*/
double Sensors::getPressure() {
  double press_1 = bme1.readPressure();
  double press_2 = bme2.readPressure();
  if (press_1 >= 1000 && press_2 >= 1000) return (press_1 + press_2) / 2;
  if (press_1 >= 1000 && press_2 <= 0) return press_1;
  if (press_2 >= 1000 && press_1 <= 0) return press_2;
  else return baro.getPressure();
}

/*
  function: getAltitude
  ---------------------------------
  This function returns a sensor fused reading.
*/
double Sensors::getAltitude() {
  double altitude_1 = bme1.readAltitude(1013.25);
  double altitude_2 = bme2.readAltitude(1013.25);
  if (altitude_1 >= -75 && altitude_2 >= -75) return (altitude_1 + altitude_2) / 2;
  if (altitude_1 >= -75 && altitude_2 <= -75) return altitude_1;
  if (altitude_2 >= -75 && altitude_1 <= -75) return altitude_2;
  else return baro.getAltitude();
}
