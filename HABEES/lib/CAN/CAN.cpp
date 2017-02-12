/*
  Stanford Student Space Initiative
  Balloons | HABEES | February 2017
  Davy Ragland | dragland@stanford.edu
  Zach Belateche | zachbela@stanford.edu
  Alex Mallery | amallery@stanford.edu

  File: CAN.cpp
  --------------------------
  Implimentation of CAN.h
*/

#include "CAN.h"

/**********************************  SETUP  ***********************************/
/*
  function: init
  ---------------------------------
  This function initializes the CAN bus.
*/
bool CAN::init() {
  pinMode(CAN_ENABLE, OUTPUT);
  analogWrite(CAN_ENABLE, 0);
  CANbus.begin();
  delay(1000);
  return true;
}

/********************************  FUNCTIONS  *********************************/
/*
  function: write
  ---------------------------------
  This function writes a bitstream across the communication interface.
*/
int16_t CAN::write(char* buff, uint16_t len) {
  if(len > BUFFER_SIZE) return -1;
  if(len < 0) return -1;
  static CAN_message_t msg;
  msg.len = 8;
  msg.id = 0x000;
  for(size_t i = 0; i < len; i+=8) {
    for(uint8_t j = 0; j < 8; j++){
      if(i+j < len){
        msg.buf[j] = buff[i+j];
      }
      else{ msg.buf[j] = 0; }
    }
    if( CANbus.write(msg) == 0){return -1;}
    msg.id++;
  }
  return 0;
}
