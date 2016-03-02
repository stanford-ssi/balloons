void setup3115() {
  MPL_1.begin(); 
  MPL_1.setModeAltimeter(); // Measure altitude above sea level in meters
  //myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
  MPL_1.setOversampleRate(7); // Set Oversample to the recommended 128
  MPL_1.enableEventFlags(); 
}

void run3115() {
  // set the current pressure altitude 
}

void setupBNO() {
  if(!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
}

void runBNO() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  CUR_E_X = euler.x(); 
  CUR_E_Y = euler.y(); 
  CUR_E_Z = euler.z(); 
}

