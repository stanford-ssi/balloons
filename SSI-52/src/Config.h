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
static const String    MISSION_NUMBER    = "SSI-52";
static const String    CSV_DATA_HEADER   = "TIME,MILLIS,LOOP,VOLTAGE,ALT_GPS,ALT_BMP,ASCENT_RATE,TEMP_IN,TEMP_EXT,LAT,LONG,SPEED_GPS,PRESS_BMP,CURRENT,CUTDOWN_STATE,MESSAGES SENT";
static const bool      ENABLE_CUTDOWN    =  true;
static const uint16_t  ASCENT_RATE_SIZE  =   200;
static const uint16_t  CUTDOWN_ALT       = 20000;
static const uint16_t  DEBUG_ALT         =   300;
static const double    PID_SETPOINT      =     0;
static const uint16_t  TRANSMIT_RATE     =  5000;
static const uint8_t   LOOP_RATE         =   100;

/*****************************  TEENSY PIN OUTS  ******************************/
static const uint8_t SD_CS               =    21;
static const uint8_t ROCKBLOCK_SLEEP     =     9;
static const uint8_t VMON                =   A13;
static const uint8_t BMP_CS1             =    20;
static const uint8_t BMP_CS2             =    15;
static const uint8_t THERMOCPL_CS        =    22;
static const uint8_t CUTDOWN_PIN         =    25;
static const uint8_t HEATER_PIN          =    23;
static const uint8_t FAULT_LED           =    13;

static const uint8_t GPS_ENABLE          =    26;

static const uint8_t DRA_TX              =   A16;
static const uint8_t DRA_RX              =   A17;
static const uint8_t PTT                 =    A2;
static const uint8_t RADIO_ENABLE        =    A0;

/***************************  MULTIPLEXER PIN OUTS  ***************************/
static const uint8_t CAN_GOOD           =     0;
static const uint8_t RB_GOOD            =     1;
static const uint8_t GPS_GOOD           =     2;
static const uint8_t HEARTBEAT          =     3;
static const uint8_t BAT_GOOD           =     4;
static const uint8_t I_GOOD             =     5;
static const uint8_t P_GOOD             =     6;
static const uint8_t T_GOOD             =     7;

#endif
