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
  function: read
  ---------------------------------
  This function reads a bitstream across the communication interface.
*/
void read(uint8_t rxBuffer[]) {
  size_t bufferSize = sizeof(rxBuffer);
  char fullMessage[200] = {0};
  for (uint8_t i = 0; i < bufferSize; i++) {
    fullMessage[i] = rxBuffer[i];
  }
}

/*
  function: writeRead
  ---------------------------------
  This function writes and reads a bitstream across the communication interface.
*/
int8_t RockBLOCK::writeRead(char* buff, uint8_t len) {
  String messageToSend = "HELLO WORLD!";
  size_t bufferSize = 0;
  uint8_t rxBuffer[200] = {0};

  delay(200);
  Serial.println("Beginning to talk to the RockBLOCK...");
  Serial.println("Sending RB message");

  bufferSize = sizeof(rxBuffer);
  for(uint8_t i = 0; i < messageToSend.length(); i++){
    rxBuffer[i] = messageToSend[i];
  }

  Serial.println("SENDING FORREAL");
  int ret = isbd.sendReceiveSBDBinary(rxBuffer, messageToSend.length(), rxBuffer, bufferSize);
  if (ret == ISBD_SUCCESS) Serial.println("ISBD_SUCCESS");
  Serial.println("Not going to sleep mode, buddy.");
  delay(2000);
  Serial.println("Reading command buffer");
  read(rxBuffer);
  return 0;
}

/*
   function: ISBDCallback
   ---------------------------------
   This callback function is run whenever the a RockBLOCK function is being called.
*/
bool ISBDCallback() {
  bool inSetup = false;
  if (!inSetup) {
    //HOW DO I CALL BACK FROM ANOTHER CLASS SOS
  }
  return true;
}
