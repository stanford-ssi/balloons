/*
  Stanford Student Space Initiative
  Balloons Launch 52 | HABEES | January 2017
  Davy Ragland   | dragland@stanford.edu
  Zach Belateche | zachbela@stanford.edu
  Alex Mallery   | amallery@stanford.edu
  Kirill Safin   | ksafin@stanford.edu
  Aria Tedjarati | satedjarati@stanford.edu
*/

/*********************************************************************
  File: main.cpp
  --------------------------
  Code for ELMO avionics
  Executes essential fight controler tasks.
*********************************************************************/

#include "Arduino.h"
#include "Avionics.h"

Avionics flightController;

void setup() {
  flightController.init();
}

void loop() {
  flightController.run();
  delay(1000);
}
