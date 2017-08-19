
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

/* based on the latitude and longitude of BEND, OR, the sun will have the following angles on aug 21, 2017 at 1000: 
 *  TODO: update with actual launchsite data
*/
#define BNO055_SAMPLERATE_DELAY_MS (100)
float AZIMUTH = 150; //Azimuth is measured clockwise from true north to the point on the horizon directly below the object.
int AIN1 = 14;
int AIN2 = 15;
int PULSE_LENGTH = 4094;
int PWM_CHANNEL = 3;
int PWM_CHANNEL_TEST = 1; 
int OUTPUT_ENABLE = 6;
int HBRIDGE_STANDBY = 13;
Adafruit_PWMServoDriver pwmdriver = Adafruit_PWMServoDriver();
Adafruit_BNO055 bno = Adafruit_BNO055(55);

int STOP_DRIVING = 5;
int DRIVE_CLOCKWISE = 6;
int DRIVE_COUNTERCLOCKWISE = 7;

void setup() {
  //define motor driver input pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(OUTPUT_ENABLE, OUTPUT);
  pinMode(HBRIDGE_STANDBY, OUTPUT);

  //sets up i2c between teensy and pwm chip
  pwmdriver.begin();
  pwmdriver.setPWMFreq(60);
  digitalWrite(OUTPUT_ENABLE, LOW);

  //set up i2c between teensy and bno055
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);
  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);
  
}

void loop() {
  if(Serial.available()) {
    char b = Serial.read();
    switch (b) {
      case 'l': 
      loadCalibrationConstants();
      break;
      case 's': saveCalibrationConstants();
      break;
    }
  }
  digitalWrite(HBRIDGE_STANDBY, HIGH);
  ControlMotor();
  delay(BNO055_SAMPLERATE_DELAY_MS);
  displayCalibration();
}

void displayCalibration () {
  uint8_t sys, gyro, accel, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  Serial.print(F("Calibration: "));
  Serial.print(sys, DEC);
  Serial.print(F(" "));
  Serial.print(gyro, DEC);
  Serial.print(F(" "));
  Serial.print(accel, DEC);
  Serial.print(F(" "));
  Serial.println(mag, DEC);
}

/*todo: adjust to rotate correctly*/

void ControlMotor() { //todo
  //get orientation relative to magnetic north - get heading: the BNO055 is pointing towards magnetic north when it's heading is 0
  sensors_event_t event;
  bno.getEvent(&event);

  //heading: 0° to 360° (turning clockwise increases values)
  float heading = (float)event.orientation.x;
  if (heading > AZIMUTH) { //BNO055 has rotated farther clockwise than the sun
    Serial.print((float)(event.orientation.x));
    Serial.println(F(""));
    Serial.println("azimuth is less than heading");
    driveCounterClockwise();
  } else if (AZIMUTH > heading) { //BNO055 is behind the sun
    Serial.print((float)(event.orientation.x));
    Serial.println(F(""));
    Serial.println("Azimuth is greater than heading"); 
    driveClockwise();
  } else { //gopro is pointing at the sun
    Serial.println("Azimuth is equal to heading");
    keepMotorStill();
  }
}

void driveClockwise() {
  pwmdriver.setPWM(PWM_CHANNEL, 0, PULSE_LENGTH);
  Serial.println("driving clockwise");
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
}

void driveCounterClockwise() {
  pwmdriver.setPWM(PWM_CHANNEL, 0, PULSE_LENGTH);
  Serial.println("driving counterclockwise");
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
}

void keepMotorStill() {
  pwmdriver.setPWM(PWM_CHANNEL, 0, 0);
  Serial.println("stopping");
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
}

/*automatically load in BNO055 calibration*/

void loadCalibrationConstants(){

  int eeAddress = 0;
  long bnoID;
  bool foundCalib = false;

  EEPROM.get(eeAddress, bnoID);

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  /*
  *  Look for the sensor's unique ID at the beginning oF EEPROM.
  *  This isn't foolproof, but it's better than nothing.
  */
  bno.getSensor(&sensor);
  if (bnoID != sensor.sensor_id)
  {
      Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
      delay(500);
  }
  else
  {
      Serial.println("\nFound Calibration for this sensor in EEPROM.");
      eeAddress += sizeof(long);
      EEPROM.get(eeAddress, calibrationData);

      Serial.println("\n\nRestoring Calibration data to the BNO055...");
      bno.setSensorOffsets(calibrationData);

      Serial.println("\n\nCalibration data loaded into BNO055");
      foundCalib = true;
  }

  delay(1000);
    
  sensors_event_t event;
  bno.getEvent(&event);
  if (foundCalib){
    Serial.println("Move sensor slightly to calibrate magnetometers");
//    while (!bno.isFullyCalibrated())
//    {
//        bno.getEvent(&event);
//        delay(BNO055_SAMPLERATE_DELAY_MS);
//    }
    
    Serial.println("\nFully calibrated!");
    Serial.println("--------------------------------");
    Serial.println("Calibration Results: ");
    adafruit_bno055_offsets_t newCalib;
    bno.getSensorOffsets(newCalib);
  }
}

void saveCalibrationConstants(){

  if(bno.isFullyCalibrated()){
    Serial.println("Calibration Results: ");
    adafruit_bno055_offsets_t newCalib;
    bno.getSensorOffsets(newCalib);

    Serial.println("\n\nStoring calibration data to EEPROM...");

    int eeAddress = 0;
    sensor_t sensor;
    bno.getSensor(&sensor);
    long bnoID = sensor.sensor_id;

    EEPROM.put(eeAddress, bnoID);

    eeAddress += sizeof(long);
    EEPROM.put(eeAddress, newCalib);
    Serial.println("Data stored to EEPROM.");

    Serial.println("\n--------------------------------\n");
  }
}
