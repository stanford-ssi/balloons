/*
  Stanford Student Space Initiative
  Balloons | HABEES | January 2017
  Davy Ragland | dragland@stanford.edu

  File: Config.h
  --------------------------
  Global constants specific to each launch.
*/

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <stdint.h>

/****************************  EDITABLE CONSTANTS  ****************************/
static const String    MISSION_NUMBER     = "SSI-52";
static const String    CSV_DATA_HEADER    = "TIME,LOOP_RATE,VOLTAGE,CURRENT,ALTITUDE_BMP,ASCENT_RATE,TEMP_IN,TEMP_EXT,LAT_GPS,LONG_GPS,SPEED_GPS,ALTITUDE_GPS,PRESS_BMP,RB_SENT_COMMS,CUTDOWN_STATE";
static const uint16_t  GPS_BAUD           =  9600;
static const uint16_t  RB_BAUD            = 19200;
static const bool      CUTDOWN_ALT_ENABLE =  true;
static const bool      CUTDOWN_GPS_ENABLE =  true;
static const uint16_t  BUFFER_SIZE        =   200;
static const uint16_t  DEBUG_ALT          =   300;
static const uint16_t  CUTDOWN_ALT        = 20000;
static const uint16_t  CUTDOWN_TIME       =  5000;
static const uint16_t  GPS_LOCK_TIME      =   500;
static const uint16_t  COMMS_RATE         = 60000;
static const uint16_t  WATCHDOG_RATE      =  2000;
static const uint16_t  LOOP_RATE          =    50;
static const float     GPS_FENCE_LAT_MIN  = -9999;
static const float     GPS_FENCE_LAT_MAX  =  9999;
static const float     GPS_FENCE_LON_MIN  = -9999;
static const float     GPS_FENCE_LON_MAX  =  9999;
static const double    PID_SETPOINT       =     0;

/*****************************  TEENSY PIN OUTS  ******************************/
static const uint8_t   VBAT_PIN           =   A13;
static const uint8_t   SD_CS              =    21;
static const uint8_t   BMP_CS1            =    20;
static const uint8_t   BMP_CS2            =    15;
static const uint8_t   THERMOCPL_CS       =    22;
static const uint8_t   CUTDOWN_PIN        =    25;
static const uint8_t   HEATER_PIN         =    23;
static const uint8_t   WATCHDOG_PIN       =    33;
static const uint8_t   FAULT_LED          =    24;

static const uint8_t   GPS_ENABLE         =    26;
static const uint8_t   RB_SLEEP           =     9;
static const uint8_t   DRA_ENABLE         =    A0;
static const uint8_t   DRA_SLEEP          =   A18;
static const uint8_t   DRA_PWR            =    A3;
static const uint8_t   DRA_TX             =   A16;
static const uint8_t   DRA_RX             =   A17;
static const uint8_t   DRA_MIC            =   A14;
static const uint8_t   DRA_PTT            =    A2;

static const uint8_t   CAN_GOOD_LED       =     0;
static const uint8_t   RB_GOOD_LED        =     1;
static const uint8_t   GPS_GOOD_LED       =     2;
static const uint8_t   HEARTBEAT_LED      =     3;
static const uint8_t   BAT_GOOD_LED       =     4;
static const uint8_t   I_GOOD_LED         =     5;
static const uint8_t   P_GOOD_LED         =     6;
static const uint8_t   T_GOOD_LED         =     7;

#endif