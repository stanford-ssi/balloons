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

#include <Metro.h>
#include <FlexCAN.h>

class CAN {
public:
/**********************************  SETUP  ***********************************/
  CAN(uint8_t CAN_ENABLE_PIN, uint32_t CAN_BAUD_VAL, uint16_t BUFFER_SIZE_VAL) :
  CAN_ENABLE(CAN_ENABLE_PIN),
  BUFFER_SIZE(BUFFER_SIZE_VAL),
  CANbus(CAN_BAUD_VAL) {
  }
  bool    init();
/********************************  FUNCTIONS  *********************************/
  int16_t write(char* buff, uint16_t len);
private:
/*********************************  OBJECTS  **********************************/
  uint8_t  CAN_ENABLE;
  uint16_t BUFFER_SIZE;
  uint8_t crxBuffer[200] = {0};
  FlexCAN CANbus;
};

#endif
