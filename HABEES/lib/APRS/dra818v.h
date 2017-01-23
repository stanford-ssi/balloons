#ifndef DRA818V_H
#define DRA818V_H

#include "APRS.h"

class SoftwareSerial;
class DRA818V
{

public:
    DRA818V();
    bool init();
    void setSquelch(uint8_t sq_level);
    SoftwareSerial *radioSerial;
private:
    void configSettings();
    uint8_t pttPin = 0;
    uint8_t micPin = 0;
    uint8_t audioOutPin = 0;
    uint8_t pwmPin = 0;
    uint8_t squelch;
    uint8_t sleepPin = 0;
    uint8_t radio_enable = 0;
    uint8_t radio_power = 0;
};

#endif // DRA818V_H
