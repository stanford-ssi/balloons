/*
  Stanford Student Space Initiative
  Balloons | HABEES | January 2017
  Davy Ragland | dragland@stanford.edu
  Vinh Nguyen | vnguyen5@stanford.edu

  File: APRS.h
  --------------------------
  APRS API.
*/

#ifndef APRS_H
#define APRS_H

#include "../../src/Config.h"

/*
  Global variable specified in Config.h
  --------------------------
  static const uint8_t   RADIO_ENABLE      =    A0;
  static const uint8_t   DRA_TX            =   A16;
  static const uint8_t   DRA_RX            =   A17;
  static const uint8_t   DRA_PTT           =    A2;
*/

class APRS {
public:
/**********************************  SETUP  ***********************************/
  void init();
/********************************  FUNCTIONS  *********************************/
  uint8_t write(char* buff, uint8_t len);
private:
/*********************************  HELPERS  **********************************/
};

#endif
