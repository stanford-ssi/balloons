/*
  Stanford Student Space Initiative
  Balloons | HABEES | January 2017
  Davy Ragland | dragland@stanford.edu

  File: data.h
  --------------------------
  Data frame structure.
*/

#ifndef DATA_H
#define DATA_H

#include "Config.h"

/**********************************  DATA  ************************************/
struct DataFrame {
  float   AscentRateBuffer[ASCENT_RATE_SIZE];
  double  VOLTAGE             = 0;
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
  double  CURRENT             = 0;
  bool    BLINK               = 0;
  String  TIMER               = "";
} ;

#endif
