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

void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.println("Hello World!");
  delay(1000);
}
