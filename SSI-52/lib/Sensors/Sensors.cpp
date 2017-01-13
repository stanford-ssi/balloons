/*
  Stanford Student Space Initiative
  Balloons | HABEES | January 2017
  Davy Ragland | dragland@stanford.edu

  File: Sensors.cpp
  --------------------------
  Implimentation of Sensors.h
*/

#include "Sensors.h"
/******************************  GLOBAL OBJECTS  *******************************/
Adafruit_BMP280 bme1(BMP_CS1);
Adafruit_BMP280 bme2(BMP_CS2);
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
Adafruit_MAX31855 thermocouple(THERMOCPL_CS);

/**********************************  SETUP  ***********************************/
/*
  function: initSensors
  ---------------------------------
  This function initializes the sensor hardware.
*/
void Sensors::init() {
  if (!bme1.begin()) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
  if (!bme2.begin()) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
  if (! baro.begin()) {
    Serial.println("Could not find a valid MPL3115A2 sensor, check wiring!");
  }
}

/********************************  FUNCTIONS  *********************************/
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
  if (altitude_1 >= -50 && altitude_2 >= -50) return (altitude_1 + altitude_2) / 2;
  if (altitude_1 >= -50 && altitude_2 <= -50) return altitude_1;
  if (altitude_2 >= -50 && altitude_1 <= -50) return altitude_2;
  else return baro.getAltitude();
}

/*********************************  HELPERS  **********************************/
/*
 * Function: readData
 * -------------------
 * This function updates the current data frame.
 */
int8_t Sensors::readData() {
  return 0;
}
