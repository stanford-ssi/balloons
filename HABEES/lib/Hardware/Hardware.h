/*
  Stanford Student Space Initiative
  Balloons | HABEES | January 2017
  Davy Ragland | dragland@stanford.edu

  File: Hardware.h
  --------------------------
  Interface to PCB hardware.
*/

#ifndef HARDWARE_H
#define HARDWARE_H

#include <Adafruit_MCP23017.h>
#include <PID_v1.h>

class Hardware {
public:
/**********************************  SETUP  ***********************************/
  Hardware(uint8_t WATCHDOG_PIN_NUM, uint8_t FAULT_PIN_NUM, uint8_t HEATER_PIN_NUM, uint8_t CUTDOWN_PIN_NUM, double PID_SETPOINT_VAL, uint16_t ANALOG_RES_VAL, uint16_t ANALOG_MAX_VAL, uint16_t LOOP_RATE_VAL) :
    WATCHDOG_PIN(WATCHDOG_PIN_NUM),
    FAULT_PIN(FAULT_PIN_NUM),
    HEATER_PIN(HEATER_PIN_NUM),
    CUTDOWN_PIN(CUTDOWN_PIN_NUM),
    PIDSetVar(PID_SETPOINT_VAL),
    ANALOG_RES(ANALOG_RES_VAL),
    ANALOG_MAX(ANALOG_MAX_VAL),
    LOOP_RATE(LOOP_RATE_VAL),
    pid(&PIDTempVar, &PIDOutVar, &PIDSetVar, 2, 5, 1, DIRECT) {
  }
  void init();
/********************************  FUNCTIONS  *********************************/
  void writeLED(uint8_t PIN, bool green);
  void faultLED();
  void heater(double temp);
  void cutDown(bool on);
  void watchdog();
private:
/*********************************  OBJECTS  **********************************/
  uint8_t  WATCHDOG_PIN;
  uint8_t  FAULT_PIN;
  uint8_t  HEATER_PIN;
  uint8_t  CUTDOWN_PIN;
  double   PIDSetVar;
  double   PIDOutVar;
  double   PIDTempVar;
  uint16_t ANALOG_RES;
  uint16_t ANALOG_MAX;
  uint16_t LOOP_RATE;
  PID pid;
  Adafruit_MCP23017 mcp;
};

#endif
