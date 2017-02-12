/*
  Stanford Student Space Initiative
  Balloons | HABEES | February 2017
  Davy Ragland | dragland@stanford.edu
  Aria Tedjarati | satedjarati@stanford.edu
  Vinh Nguyen | vnguyen5@stanford.edu

  File: main.cpp
  --------------------------
  Flight code for main HABEES avionics.
*/

#include "Avionics.h"

/***********************************  BOOT  ***********************************/
Avionics HABEES;
int main(void) {
  HABEES.init();
/***********************************  MAIN  ***********************************/
  while(true) {
    HABEES.updateState();
    HABEES.evaluateState();
    HABEES.actuateState();
    HABEES.logState();
    HABEES.sendComms();
    HABEES.sleep();
  }
}
/*********************************  CALLBACK  *********************************/
bool ISBDCallback() {
  if(HABEES.finishedSetup()) {
    HABEES.updateState();
    HABEES.evaluateState();
    HABEES.actuateState();
    HABEES.logState();
    HABEES.sleep();
  }
  return true;
}
