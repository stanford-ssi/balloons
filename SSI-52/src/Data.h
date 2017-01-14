/*
  Stanford Student Space Initiative
  Balloons | HABEES | January 2017
  Davy Ragland | dragland@stanford.edu

  File: data.h
  --------------------------
  Data frame structure.
*/

#ifndef data_H
#define data_H

#include "Config.h"

/**********************************  data  ************************************/
struct DataFrame {
  bool    ENABLE_DEBUG      =  true;
  float   AscentRateBuffer[ASCENT_RATE_SIZE];
  double  VOLTAGE             = 0;
  double  CURRENT             = 0;
  double  ALTITUDE_LAST       = 0;
  double  ALTITUDE_GPS        = 0;
  double  ALTITUDE_BMP        = 0;
  double  ASCENT_RATE         = 0;
  double  TEMP_IN             = 0;
  double  TEMP_EXT            = 0;
  float   LAT                 = 0;
  float   LONG                = 0;
  bool    CUTDOWN_STATE       = 0;
  int     messagesSent        = 0;
  double  SPEED_GPS           = 0;
  double  PRESS_BMP           = 0;
  bool    BLINK               = 0;
  String  TIMER               = "";

  bool CAN_SET_SUCESS         = false;
  bool RB_SET_SUCESS          = false;


} ;

#endif
