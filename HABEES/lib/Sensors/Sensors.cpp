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
char buf[] = "##:##:## ## ## ####";
time_t getTeensy3Time() {return Teensy3Clock.get();}
int8_t Sensors::init() {
  int8_t sucess = 0;
  setSyncProvider(getTeensy3Time);
  if (timeStatus()!= timeSet) {
    Serial.println("Unable to sync with the RTC");
    sucess += -8;
  }
  if (!bme1.begin()) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    sucess += -1;
  }
  if (!bme2.begin()) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    sucess += -2;
  }
  if (! baro.begin()) {
    Serial.println("Could not find a valid MPL3115A2 sensor, check wiring!");
    sucess += -4;
  }
  inaHeater.begin(0x41);
  inaCutdown.begin(0x40);
  inaRadio.begin(0x45);
  return sucess;
}

/********************************  FUNCTIONS  *********************************/
/*
  function: getTime
  ---------------------------------
  This function returns the time as a fixed length string.
*/
const char* Sensors::getTime() {
  convertDigits(0, hour());
  buf[2] = ':';
  convertDigits(3, minute());
  buf[5] = ':';
  convertDigits(6, second());
  buf[8] = ' ';
  convertDigits(9, day());
  buf[11] = ' ';
  convertDigits(12, month());
  buf[14] = ' ';
  convertYear(15, year());
  return buf;
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
  double current = 0;
  current += inaHeater.getCurrent_mA();
  current += inaCutdown.getCurrent_mA();
  current += inaRadio.getCurrent_mA();
  return current;
}

/*
  function: getTempOut
  ---------------------------------
  This function gets the external temperature.
*/
double Sensors::getTempOut() {
  double temp = thermocouple.readCelsius();
  if (isnan(temp)) return -1;
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
/*********************************  HELPERS  **********************************/
/*
  function: convertDigits
  ---------------------------------
  This function populates a buffer with a digit as a string.
*/
void Sensors::convertDigits(uint8_t start, uint8_t digits){
  buf[start + 0] = (digits / 10) + '0';
  buf[start + 1] = (digits % 10) + '0';
}

/*
  function: convertYear
  ---------------------------------
  This function populates a buffer with a year as a string.
*/
void Sensors::convertYear(uint8_t start, int year) {
  for(uint8_t i = 0; i < 4; i++) {
    buf[start + (3 - i)] = (year % 10) + '0';
    year /= 10;
  }
}
