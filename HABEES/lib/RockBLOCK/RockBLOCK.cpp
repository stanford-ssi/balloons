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

IridiumSBD isbd(Serial3, RB_SLEEP);

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
  // isbd.begin();
  return 0;
}

/********************************  FUNCTIONS  *********************************/
/*
  function: writeRead
  ---------------------------------
  This function writes and reads a bitstream across the communication interface.
*/

/*
 * TODO:
 * pass buffer by addrs
 * return message by addrs and len
*/
int8_t RockBLOCK::writeRead(char* buff, uint8_t len) {
  String messageToSend = "HELLO WORLD!";
  size_t  messageLen = messageToSend.length();

  size_t  bufferSize = 0;
  uint8_t rxBuffer[BUFFER_SIZE] = {0};
  uint8_t commands[BUFFER_SIZE] = {0};

  delay(200);
  Serial.println("Beginning to talk to the RockBLOCK...");
  Serial.println("Sending RB message");

  bufferSize = sizeof(rxBuffer);
  for(uint8_t i = 0; i < messageToSend.length(); i++){
    rxBuffer[i] = messageToSend[i];
  }

  Serial.println("SENDING FORREAL");
  isbd.sendReceiveSBDBinary(rxBuffer, messageLen, rxBuffer, bufferSize);
  Serial.println("Not going to sleep mode.");
  if (bufferSize > 0) {
    for (size_t i = 0; i < bufferSize; i++) {
      commands[i] = rxBuffer[i];
    }
  }
  return 0;
}

/*********************************  HELPERS  **********************************/
