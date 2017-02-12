/*
  Stanford Student Space Initiative
  Balloons | HABEES | February 2017
  Davy Ragland | dragland@stanford.edu
  Zach Belateche | zachbela@stanford.edu
  Alex Mallery | amallery@stanford.edu

  File: CAN.h
  --------------------------
  CAN BUS API.
*/

#ifndef CAN_H
#define CAN_H

#include <Metro.h>
#include <FlexCAN.h>

class CAN {
public:
/**********************************  SETUP  ***********************************/
  CAN(uint8_t CAN_ENABLE_PIN, uint32_t CAN_BAUD_VAL) :
    CAN_ENABLE(CAN_ENABLE_PIN),
    CANbus(CAN_BAUD_VAL) {
  }
  bool    init();
/********************************  FUNCTIONS  *********************************/
  int16_t write(char* buff, uint16_t len);
private:
/*********************************  OBJECTS  **********************************/
  static const uint16_t BUFFER_SIZE = 200;
  uint8_t crxBuffer[BUFFER_SIZE] = {0};
  uint8_t CAN_ENABLE;
  FlexCAN CANbus;
};

#endif
