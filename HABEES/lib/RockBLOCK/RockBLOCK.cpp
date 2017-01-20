/*
  Stanford Student Space Initiative
  Balloons | HABEES | January 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | satedjarati@stanford.edu

  File: RockBlock.cpp
  --------------------------
  Implimentation of RockBlock.h
*/

#include "RockBLOCK.h"

/**********************************  SETUP  ***********************************/
/*
  function: init
  ---------------------------------
  This function initializes the RockBlock module.
*/
int8_t RockBLOCK::init() {
  isbd.attachConsole(Serial);
  isbd.attachDiags(Serial);
  isbd.setPowerProfile(1);
  Serial3.begin(RB_BAUD);
  delay(5000);
  isbd.begin();
  return 0;
}

/********************************  FUNCTIONS  *********************************/
/*
  function: writeRead
  ---------------------------------
  This function writes a bitstream across the communication interface.
  It returns the length of a read message.
*/
int8_t RockBLOCK::writeRead(char* buff, uint8_t len) {
  uint8_t rxBuffer[BUFFER_SIZE] = {0};
  size_t  bufferSize = sizeof(rxBuffer);
  for(size_t i = 0; i < len; i++) rxBuffer[i] = buff[i];
  Serial.println("Beginning to talk to the RockBLOCK...");
  delay(200);
  Serial.println("Sending RB message");
  if(isbd.sendReceiveSBDBinary(rxBuffer, len, rxBuffer, bufferSize) != ISBD_SUCCESS) return -1;
  Serial.println("Not going to sleep mode.");
  for(size_t i = 0; i < bufferSize; i++) buff[i] = rxBuffer[i];
  return bufferSize;
}
