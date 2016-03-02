void satelliteTransmission() {
  if (millis() - LAST_SATELLITE_TRANSMISSION > SATELLITE_TRANSMISSION_INTERVAL) {
     COMMS_TEENSY.print("hi");
     // TODO: update with flight parameters
     LAST_SATELLITE_TRANSMISSION = millis(); 
  }
}
