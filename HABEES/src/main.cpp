/*
  Stanford Student Space Initiative
  Balloons | HABEES | January 2017
  Davy Ragland | dragland@stanford.edu

  File: main.cpp
  --------------------------
  Flight code for main HABEES avionics.
*/

#include "Avionics.h"

/***********************************  BOOT  ***********************************/
bool inSetup = true;
Avionics flightController;
int main(void) {
  flightController.init();
  inSetup = false;
/***********************************  MAIN  ***********************************/
  while(true) {
    flightController.updateData();
    flightController.evaluateState();
    flightController.sendComms();
    flightController.sleep();
  }
}
/*********************************  CALLBACK  *********************************/
bool ISBDCallback() {
  if (!inSetup) {
    flightController.updateData();
    flightController.evaluateState();
    flightController.sleep();
  }
  return true;
}
