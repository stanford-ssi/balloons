/*
  Stanford Student Space Initiative
  Balloons | HABEES | January 2017
  Davy Ragland | dragland@stanford.edu
  Zach Belateche | zachbela@stanford.edu
  Alex Mallery | amallery@stanford.edu

  File: CAN.h
  --------------------------
  CAN BUS API.
*/

#ifndef CAN_H
#define CAN_H

#include "../../src/Config.h"

class CAN {
public:
/**********************************  SETUP  ***********************************/
  bool init();
/********************************  FUNCTIONS  *********************************/
  int16_t write(char* buff, uint16_t len);
private:
/*********************************  OBJECTS  **********************************/
  uint8_t rxBuffer[BUFFER_SIZE] = {0};
};

#endif
