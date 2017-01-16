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
  String  TIME               =    "";
  long    LOOP_RATE          =     0;
  double  VOLTAGE            =     0;
  double  CURRENT            =     0;
  double  ALTITUDE_BMP       =     0;
  double  ASCENT_RATE        =     0;
  double  TEMP_IN            =     0;
  double  TEMP_EXT           =     0;
  float   LAT_GPS            =     0;
  float   LONG_GPS           =     0;
  double  SPEED_GPS          =     0;
  double  ALTITUDE_GPS       =     0;
  double  PRESS_BMP          =     0;
  int     RB_SENT_COMMS      =     0;
  bool    CUTDOWN_STATE      = false;

  bool    BAT_GOOD_STATE     = false;
  bool    I_GOODD_STATE      = false;
  bool    P_GOOD_STATE       = false;
  bool    T_GOOD_STATE       = false;
  bool    CAN_GOOD_STATE     = false;
  bool    RB_GOOD_STATE      = false;
  bool    GPS_GOOD_STATE     = false;
  bool    HEARTBEAT_STATE    = false;

  bool    DEBUG_STATE        =  true;
  double  ALTITUDE_LAST      =     0;
  long    LOOP_START         =     0;
  float   Ascent_BUFFER[BUFFER_SIZE];
  char    COMMS_BUFFER[BUFFER_SIZE];
} ;

#endif
