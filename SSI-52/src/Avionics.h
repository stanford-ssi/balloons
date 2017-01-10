#ifndef AVIONICS_H
#define AVIONICS_H

#include "Arduino.h"
#include <stdint.h>

class Avionics {
public:
  void init();
  void run();
private:
  uint8_t value = 0x0;
};

#endif
