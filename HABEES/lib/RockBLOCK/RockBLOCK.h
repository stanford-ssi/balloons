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
  RockBLOCK() : isbd(Serial3, RB_SLEEP){}
  bool    init();
/********************************  FUNCTIONS  *********************************/
  int16_t writeRead(char* buff, uint16_t len);
private:
/*********************************  HELPERS  **********************************/
  void    write(char* buff, uint16_t len);
  void    read(char* buff, uint16_t len);
/*********************************  OBJECTS  **********************************/
  IridiumSBD isbd;
  uint8_t rxBuffer[BUFFER_SIZE] = {0};
};

#endif
