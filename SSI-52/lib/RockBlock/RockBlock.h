/*
  Stanford Student Space Initiative
  Balloons | HABEES | January 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | satedjarati@stanford.edu

  File: RockBlock.h
  --------------------------
  Interface to Iridium RockBlock satalite communications.
*/

#ifndef RockBlock_H
#define RockBlock_H

#include "../../src/Config.h"

/*
  Global variable specified in Config.h
  --------------------------
  static const uint16_t  RB_BAUD            = 19200;
  static const float     RB_COMM_RATE      =   2.0;
  static const uint8_t   RB_SLEEP          =     9;
*/

class RockBlock {
public:
/**********************************  SETUP  ***********************************/
  int8_t init();
/********************************  FUNCTIONS  *********************************/
  int8_t write(char* buff, uint8_t len);
private:
/*********************************  HELPERS  **********************************/
};

#endif
