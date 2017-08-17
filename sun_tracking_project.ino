
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* based on the latitude and longitude of BEND, OR, the sun will have the following angles on aug 21, 2017 at 1000: 
 *  TODO: update with actual launchsite data
*/

float AZIMUTH = 203.43; //Azimuth is measured clockwise from true north to the point on the horizon directly below the object.
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
  pinMode(AIN2, INPUT);
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
  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);
}

void loop() {
  digitalWrite(HBRIDGE_STANDBY, HIGH);
  ControlMotor();
}

/*todo: adjust to rotate correctly*/

void ControlMotor() { //todo
  //get orientation relative to magnetic north - get heading: the BNO055 is pointing towards magnetic north when it's heading is 0
  sensors_event_t event;
  bno.getEvent(&event);

  //heading: 0° to 360° (turning clockwise increases values)
  float heading = (float)event.orientation.z;
  if (heading > AZIMUTH) { //BNO055 has rotated farther clockwise than the sun
     driveCounterClockwise();
  } else if (AZIMUTH > heading) { //BNO055 is behind the sun 
    driveClockwise();
  } else { //gopro is pointing at the sun
    keepMotorStill();
  }
}

void driveClockwise() {
  pwmdriver.setPWM(PWM_CHANNEL, 0, PULSE_LENGTH);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
}

void driveCounterClockwise() {
  pwmdriver.setPWM(PWM_CHANNEL, 0, PULSE_LENGTH);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
}

void keepMotorStill() {
  pwmdriver.setPWM(PWM_CHANNEL, 0, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
}
