
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* based on the latitude and longitude of BEND, OR, the sun will have the following angles on aug 21, 2017 at 1000:
*/

float AZIMUTH = 203.43;
int AIN1 = 14;
int AIN2 = 15;
int PULSE_LENGTH = 4094;
int PWM_CHANNEL = 3;
int PWM_CHANNEL_TEST = 1;
int OUTPUT_ENABLE = 6;
int HBRIDGE_STANDBY = 13;
Adafruit_PWMServoDriver pwmdriver = Adafruit_PWMServoDriver();
Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() {
  //define motor driver input pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, INPUT);
  pinMode(OUTPUT_ENABLE, OUTPUT);
  pinMode(HBRIDGE_STANDBY, OUTPUT);

  //sets up i2c between teensy and pwm chip
  pwmdriver.begin();
  pwmdriver.setPWMFreq(60);
  digitalWrite(OUTPUT_ENABLE, LOW);

  /*initialize the i2c devices*/
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);
}

void loop() {
  pwmdriver.setPWM(PWM_CHANNEL, 0, PULSE_LENGTH);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(HBRIDGE_STANDBY, HIGH);
  pwmdriver.setPWM(PWM_CHANNEL_TEST, 0, PULSE_LENGTH);

  //get heading 
  sensors_event_t event;
  bno.getEvent(&event);
  float = (float)event.orientation.z;
  

  //get orientation relative to magnetic north
  /*the BNO055 is pointing towards magnetic north when it's heading is 0*/
}
