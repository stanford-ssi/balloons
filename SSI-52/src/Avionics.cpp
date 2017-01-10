#include "Avionics.h"

void Avionics::init() {
  Serial.begin(9600);
}

void Avionics::run() {
  Serial.println("Hello World!");
}
