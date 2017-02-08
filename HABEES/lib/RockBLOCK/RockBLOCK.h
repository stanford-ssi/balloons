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

class RockBLOCK {
public:
/**********************************  SETUP  ***********************************/
  RockBLOCK(uint8_t RB_SLEEP_PIN, uint16_t RB_BAUD_VAL, uint16_t BUFFER_SIZE_VAL) :
    isbd(Serial3, RB_SLEEP_PIN),
    RB_BAUD(RB_BAUD_VAL),
    BUFFER_SIZE(BUFFER_SIZE_VAL) {
  }
  bool    init();
/********************************  FUNCTIONS  *********************************/
  int16_t writeRead(char* buff, uint16_t len);
private:
/*********************************  HELPERS  **********************************/
  void    write(char* buff, uint16_t len);
  void    read(char* buff, uint16_t len);
/*********************************  OBJECTS  **********************************/
  IridiumSBD isbd;
  uint16_t RB_BAUD;
  uint16_t BUFFER_SIZE;
  uint8_t  rxBuffer[200] = {0};
};

#endif
