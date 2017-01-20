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
  const char*  TIME               =    "##:##:## ## ## ####";
  char         time[20]           =    "##:##:## ## ## ####";
  uint16_t     LOOP_RATE          =     0;
  double       VOLTAGE            =     0;
  double       CURRENT            =     0;
  double       ALTITUDE_BMP       =     0;
  double       ASCENT_RATE        =     0;
  double       TEMP_IN            =     0;
  double       TEMP_EXT           =     0;
  float        LAT_GPS            =     0;
  float        LONG_GPS           =     0;
  double       SPEED_GPS          =     0;
  double       ALTITUDE_GPS       =     0;
  double       PRESS_BMP          =     0;
  uint16_t     RB_SENT_COMMS      =     0;
  bool         CUTDOWN_STATE      = false;

  bool         BAT_GOOD_STATE     = false;
  bool         I_GOOD_STATE       = false;
  bool         P_GOOD_STATE       = false;
  bool         T_GOOD_STATE       = false;
  bool         CAN_GOOD_STATE     = false;
  bool         RB_GOOD_STATE      = false;
  bool         GPS_GOOD_STATE     = false;
  bool         LOOP_GOOD_STATE    = false;

  bool         SETUP_STATE        =  true;
  bool         DEBUG_STATE        =  true;
  bool         SHOULD_CUTDOWN     = false;
  double       ALTITUDE_LAST      =     0;
  uint16_t     ASCENT_RATE_LAST   =     0;
  uint16_t     COMMS_LAST         =     0;
  uint16_t     WATCHDOG_LAST      =     0;
  uint16_t     LOOP_START         =     0;
  uint16_t     COMMS_LENGTH       =     0;
  float        ASCENT_BUFFER[BUFFER_SIZE];
  char         COMMS_BUFFER[BUFFER_SIZE];
} ;

#endif
