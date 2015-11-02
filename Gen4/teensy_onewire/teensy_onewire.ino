#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS_PIN 4

/*
 * OneWire Temp sensors. 
 * © SSI 2015 
 */
 
OneWire oneWire(ONE_WIRE_BUS_PIN);
DallasTemperature sensors(&oneWire);

// see http://www.hacktronics.com/Tutorials/arduino-1-wire-address-finder.html
DeviceAddress Probe01 = { 0x28, 0x8A, 0xB1, 0x40, 0x04, 0x00, 0x00, 0xC7 };
DeviceAddress Probe02 = { 0x28, 0xCC, 0x92, 0x40, 0x04, 0x00, 0x00, 0xB6 };

/*
 * Setup each sensor. 
 */
void setup() {
  sensors.begin();

  // set the resolution to 10 bit (Can be 9 to 12 bits .. lower is faster)
  sensors.setResolution(Probe01, 10);
  sensors.setResolution(Probe02, 10);
}

// Each temperature 
float temp1 = -1, temp2 = -1; 

/* 
 *  Get each temperature 
 */
void loop() {
  sensors.requestTemperatures();

  temp1 = getTemp(Probe01);
  temp2 = getTemp(Probe02);
  delay(100); 
}

/*
 * Ping address for temperature. 
 */
float getTemp(DeviceAddress deviceAddress) {
  return sensors.getTempC(deviceAddress);
}
