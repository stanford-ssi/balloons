/*
  Stanford Student Space Initiative
  Balloons | HABEES | January 2017
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
  return true;;
}

/********************************  FUNCTIONS  *********************************/
/*
  function: write
  ---------------------------------
  This function writes a bitstream across the communication interface.
*/
int16_t CAN::write(char* buff, uint16_t len) {
  for(size_t i = 0; i < len; i++) {
    rxBuffer[i] = buff[i];
  }
  return -1;
}
