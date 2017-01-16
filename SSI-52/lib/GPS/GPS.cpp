/*
  Stanford Student Space Initiative
  Balloons | HABEES | January 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | satedjarati@stanford.edu

  File: GPS.cpp
  --------------------------
  Implimentation of GPS.h
*/

#include "GPS.h"

/**********************************  SETUP  ***********************************/
/*
  function: init
  ---------------------------------
  This function initializes the Ublox NEO-M8Q GPS module.
*/
void GPS::init() {
  pinMode(GPS_ENABLE, OUTPUT);
  digitalWrite(GPS_ENABLE, LOW);
  Serial1.begin(GPS_BAUD);
  setFlightMode();
}

/********************************  FUNCTIONS  *********************************/
/*
  function: getLatitude
  ---------------------------------
  This function returns the current latitude.
*/
float GPS::getLatitude(){
  return tinygps.location.lat();
}

/*
  function: getLongitude
  ---------------------------------
  This function returns the current longitude.
*/
float GPS::getLongitude(){
  return tinygps.location.lng();
}

/*
  function: getAltitude
  ---------------------------------
  This function returns the current altitude.
*/
float GPS::getAltitude(){
  return tinygps.altitude.meters();
}

/*
  function: getSpeed
  ---------------------------------
  This function returns the current speed.
*/
float GPS::getSpeed(){
  return tinygps.speed.mph();
}

/*
 * Function: smartDelay
 * -------------------
 * This function pauses the main thread while still communicating with the cooms interface.
 */
void GPS::smartDelay(unsigned long ms) {
  unsigned long startt = millis();
  do {
    while (Serial1.available())
      tinygps.encode(Serial1.read());
  } while (millis() - startt < ms);
}
/*********************************  HELPERS  **********************************/
/*
  function: setFlightMode
  ---------------------------------
  This function sets the GPS module into flight mode.
*/
void GPS::setFlightMode(){
  Serial.println("Setting uBlox nav mode: ");
  uint8_t gps_set_sucess = 0 ;
  uint8_t setNav[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC };
  while(!gps_set_sucess) {
    sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
    gps_set_sucess = getUBX_ACK(setNav);
  }
  smartDelay(GPS_LOCK_TIME);
}

/*
  function: sendUBX
  ---------------------------------
  This function send a byte array of UBX protocol to the GPS.
*/
void GPS::sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    Serial1.write(MSG[i]);
    Serial.print(MSG[i], HEX);
  }
  Serial1.println();
}

/*
  function: getUBX_ACK
  ---------------------------------
  This function calculates the expected UBX ACK packet and parse UBX response from GPS.
*/
boolean GPS::getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  Serial.print(" * Reading ACK response: ");

  ackPacket[0] = 0xB5;	 // header
  ackPacket[1] = 0x62;	 // header
  ackPacket[2] = 0x05;	 // class
  ackPacket[3] = 0x01;	 // id
  ackPacket[4] = 0x02;	 // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2]; // ACK class
  ackPacket[7] = MSG[3]; // ACK id
  ackPacket[8] = 0;		   // CK_A
  ackPacket[9] = 0;		   // CK_B

  for (uint8_t i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (1) {
    if (ackByteID > 9) {
      Serial.println(" (SUCCESS!)");
      return true;
    }
    if (millis() - startTime > 3000) {
      Serial.println(" (FAILED!)");
      return false;
    }
    if (Serial1.available()) {
      b = Serial1.read();
      if (b == ackPacket[ackByteID]) {
        ackByteID++;
        Serial.print(b, HEX);
      }
      else {
        ackByteID = 0;
      }
    }
  }
}
