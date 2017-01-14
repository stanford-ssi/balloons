/*
  Stanford Student Space Initiative
  Balloons | HABEES | January 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | satedjarati@stanford.edu

  File: RockBlock.h
  --------------------------
  RockBlock API.
*/

#ifndef RockBlock_H
#define RockBlock_H

#include "../../src/Config.h"

/*
  Global variable specified in Config.h
  --------------------------
  static const uint16_t  TRANSMIT_RATE     =  5000;
  static const float     RB_COMM_RATE      =   2.0;
  static const uint8_t   ROCKBLOCK_SLEEP   =     9;
*/

class RockBlock {
public:
/**********************************  SETUP  ***********************************/
  void init();
/********************************  FUNCTIONS  *********************************/
  uint8_t write(char* buff, uint8_t len);
private:
/*********************************  HELPERS  **********************************/
};

#endif
