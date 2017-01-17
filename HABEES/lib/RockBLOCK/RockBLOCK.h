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

#include <IridiumSBD.h>
#include "../../src/Config.h"

class RockBLOCK {
public:
/**********************************  SETUP  ***********************************/
  int8_t init();
/********************************  FUNCTIONS  *********************************/
  int8_t writeRead(char* buff, uint8_t len);
private:
/*********************************  HELPERS  **********************************/
  void read(uint8_t rxBuffer[], size_t len);
  void write(uint8_t rxBuffer[], size_t len);
};

#endif
