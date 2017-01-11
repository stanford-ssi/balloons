/*
  Stanford Student Space Initiative
  Balloons | HABEES | January 2017
  Davy Ragland | dragland@stanford.edu
*/

/*
  File: Avionics.cpp
  --------------------------
  Implimentation of Avionics.h
*/

#include "Avionics.h"

void Avionics::init() {
  Serial.begin(9600);
}

void Avionics::run() {
  Serial.print(MISSION_NUMBER);
  Serial.print(value++);
  Serial.println("");
}
