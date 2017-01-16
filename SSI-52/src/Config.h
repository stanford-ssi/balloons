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

//ina219
//9dof??

// radio   1000101
// cutdown 1000000
//heaters  1000001

//24
//watchdog
// rf pwr A3
// radio sleep a18
// mic in a14



/****************************  EDITABLE CONSTANTS  ****************************/
static const String    MISSION_NUMBER    = "SSI-52";
static const String    CSV_DATA_HEADER   = "TIME,LOOP_RATE,VOLTAGE,CURRENT,ALTITUDE_BMP,ASCENT_RATE,TEMP_IN,TEMP_EXT,LAT_GPS,LONG_GPS,SPEED_GPS,ALTITUDE_GPS,PRESS_BMP,RB_SENT_COMMS,CUTDOWN_STATE";
static const uint16_t  GPS_BAUD          =  9600;
static const uint16_t  ROCKBLOCK_BAUD    = 19200;
static const bool      CUTDOWN_ENABLE    =  true;
static const uint16_t  BUFFER_SIZE       =   200;
static const uint16_t  DEBUG_ALT         =   300;
static const uint16_t  CUTDOWN_ALT       = 20000;
static const uint16_t  CUTDOWN_TIME      = 10000;
static const uint16_t  GPS_LOCK_TIME     =   500;
static const uint16_t  COMMS_RATE        = 60000;
static const uint16_t  LOOP_RATE         =    50;
static const double    PID_SETPOINT      =     0;
static const float     RB_COMM_RATE      =   2.0;

/*****************************  TEENSY PIN OUTS  ******************************/
static const uint8_t   VBAT_PIN          =   A13;
static const uint8_t   SD_CS             =    21;
static const uint8_t   BMP_CS1           =    20;
static const uint8_t   BMP_CS2           =    15;
static const uint8_t   THERMOCPL_CS      =    22;
static const uint8_t   CUTDOWN_PIN       =    25;
static const uint8_t   HEATER_PIN        =    23;
static const uint8_t   FAULT_LED         =    13;

static const uint8_t   ROCKBLOCK_SLEEP   =     9;
static const uint8_t   GPS_ENABLE        =    26;
static const uint8_t   RADIO_ENABLE      =    A0;
static const uint8_t   DRA_TX            =   A16;
static const uint8_t   DRA_RX            =   A17;
static const uint8_t   DRA_PTT           =    A2;

static const uint8_t   CAN_GOOD_LED      =     0;
static const uint8_t   RB_GOOD_LED       =     1;
static const uint8_t   GPS_GOOD_LED      =     2;
static const uint8_t   HEARTBEAT_LED     =     3;
static const uint8_t   BAT_GOOD_LED      =     4;
static const uint8_t   I_GOOD_LED        =     5;
static const uint8_t   P_GOOD_LED        =     6;
static const uint8_t   T_GOOD_LED        =     7;

#endif
