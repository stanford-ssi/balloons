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
  int8_t init();
/********************************  FUNCTIONS  *********************************/
  int8_t write(char* buff, uint8_t len);
private:
/*********************************  HELPERS  **********************************/
};

#endif
