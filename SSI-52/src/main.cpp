/*
  Stanford Student Space Initiative
  Balloons | HABEES | January 2017
  Davy Ragland | dragland@stanford.edu
*/

/*
  File: main.cpp
  --------------------------
  Flight code for ELMO avionics.
*/

#include "Avionics.h"

/******************************  GLOBAL OBJECTS  ******************************/
Avionics flightController;

/***********************************  BOOT  ***********************************/
void setup() {
  flightController.init();
}

/***********************************  MAIN  ***********************************/
void loop() {
  flightController.run();
  delay(1000);
}
