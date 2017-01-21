/*
  Stanford Student Space Initiative
  Balloons | HABEES | January 2017
  Davy Ragland | dragland@stanford.edu
  Vinh Nguyen | vnguyen5@stanford.edu

  File: APRS.cpp
  --------------------------
  Implimentation of APRS.h
*/

#include "APRS.h"

/**********************************  SETUP  ***********************************/
/*
  function: init
  ---------------------------------
  This function initializes the APRS module.
*/
bool APRS::init() {
  return true;
}

/********************************  FUNCTIONS  *********************************/
/*
  function: write
  ---------------------------------
  This function writes a bitstream across the communication interface.
*/
int16_t APRS::write(char* buff, uint16_t len) {
  for(size_t i = 0; i < len; i++) {
    rxBuffer[i] = buff[i];
  }
  return -1;
}
