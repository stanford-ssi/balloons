/*
  Stanford Student Space Initiative
  Balloons | HABEES | January 2017
  Davy Ragland | dragland@stanford.edu
*/

/*
  File: Avionics.h
  --------------------------
  Primary avionics code.
*/

#ifndef AVIONICS_H
#define AVIONICS_H

#include "config.h"

class Avionics {
public:
  void init();
  void run();
private:
  uint8_t value = 0x0;
};

#endif
