#include <TinyGPS.h>
#include <i2c_t3.h>
#include <Wire.h>
#include "MPL3115A2.h"
#include <Time.h>
#include <DS1307RTC.h>
#include <PID_v1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_INA219.h>
#include <utility/imumaths.h>

/*
 * Gen-1 Avionics
 *
 * Description
 *
 * @author
 * @date February 2016
 * @launch SSI-31
 */

#define GPS_SERIAL Serial1
#define COMMS_TEENSY Serial2 
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();
TinyGPS gps;
MPL3115A2 MPL_1;
Adafruit_INA219 CURRENT_1;

//#if defined(ARDUINO_ARCH_SAMD)
//   #define Serial SerialUSB
//#endif

float CUR_LAT = 0.0;
float CUR_LON = 0.0;
float CUR_GPS_ALT = 0.0;
float CUR_E_X = 0.0; 
float CUR_E_Y = 0.0; 
float CUR_E_Z = 0.0; 
float CUR_PRES_ALT = 0.0; 
float CUR_INTERIOR_TEMP = 0.0; 
long long LAST_SATELLITE_TRANSMISSION = 0; 
const long SATELLITE_TRANSMISSION_INTERVAL = 240000; // ms

void setup() {
  Serial.begin(115200);
  GPS_SERIAL.begin(9600);
  COMMS_TEENSY.begin(9600); 
  setupTime();
  setGPSFlightMode(); 
  setupBNO(); 
  setup3115(); 
  setupCurrent(); 
}

void loop() {
  runGPS();
  runBNO(); 
  logTime();
  satelliteTransmission(); 
  runHeaters(); 
}

void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

// TODO: change to SD
void logTime() {
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year());
  Serial.println();
}

void setupTime() {
  setSyncProvider(RTC.get);   // the function to get the time from the RTC
  if (timeStatus() != timeSet)
    Serial.println("Unable to sync with the RTC");
  else
    Serial.println("RTC has set the system time");
}

