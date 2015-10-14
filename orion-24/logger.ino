#include <SPI.h>
#include <SD.h>
#include <SPI.h>
#include <Pixy.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include "DHT.h"

#define DHTPIN 9     // DHT Temp sensor pin 
#define DHTTYPE DHT22   // DHT sensor type

/*
 * Orion Balloon Launch - SSI-24
 * 
 * Records, processes, and stores data from sensors, including 
 * pixy camera, temperature sensor, accelerometer, and GPS. 
 * Processes temperature data and controls heater as well. 
 * 
 * A function is triggered in the loop every 2000 milliseconds 
 * to run the logger. 
 * 
 * Launch date: October 18, 2015? 
 * 
 * @version 1.0
 * @author SSI - © 2015
 * 
 */

 /*
  * TODO: 
  * 
  * * servo release at specified altitude 
  * * better data logging (make sure it works) 
  * * gps stuff? 
  * * wifi downlink 
  * 
  */

File logfile;
TinyGPS gps;
SoftwareSerial ss(3, 2);
DHT dht(DHTPIN, DHTTYPE);

const int xPin = 0;
const int yPin = 1;
const int zPin = 2;
const int chipSelect = 10;

int minVal = 265;
int maxVal = 402;

double x;
double y;
double z;

Pixy pixy;
float Temp = 100;

/*
 * Setup function to initialize sensors. 
 */
void setup() {
  Serial.begin(9600);
  dht.begin();
  ss.begin(9600);
  
  // Initialize the SD reader. 
  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  // Setup and initialize SD reader and file. 
  logfile = SD.open("logfile.txt", FILE_WRITE);
  if (!logfile) { //for debugging
    Serial.println("ERROR: COULD NOT MAKE FILE");
  }
  else {
    Serial.println("Logging into: ");
  }
  
  printLogfileHeader();

  pixy.init();
}

// Variable used to determine when to log data based on milliseconds
float logTime = 0;
int largestBlock[4] = {0, 0, 0, 0}; // x, y, width, height
float latitude = -1, longitude = -1, milesperhour = -1, altitude_1 = -1;

/*
 * Loops continuously while arduino is running. Collects  
 * and logs data from sensors. 
 */
void loop() {
  static int ii = 0;
  int jj;
  uint16_t blocks;
  char buf[32];
  blocks = pixy.getBlocks();

  if (blocks) {
    ii++;
    if (ii % 50 == 0) {
      sprintf(buf, "Detected %d:\n", blocks);
      Serial.print(buf);
      int x = -1, y = -1, w = -1, h = -1;
      for (jj = 0; jj < blocks; jj++)
      {
        x = int(pixy.blocks[jj].x);
        y = int(pixy.blocks[jj].y);
        w = int(pixy.blocks[jj].width);
        h = int(pixy.blocks[jj].height);

        if (w * h >= largestBlock[2]*largestBlock[3]) {
          largestBlock[0] = x;
          largestBlock[1] = y;
          largestBlock[2] = w;
          largestBlock[3] = h;
        }
      }
    }
  }
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;
  
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
    {
      char c = ss.read();
      
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }
  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    float falt = gps.f_altitude();
    float fmph = gps.f_speed_mph(); // speed in miles/hr
    float fmps = gps.f_speed_mps(); // speed in m/sec
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    latitude = flat;
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    longitude = flon;
    Serial.print(" ALT=");
    Serial.print(falt);
    altitude_1 = falt;
    Serial.print(" MPH=");
    Serial.print(fmph);
    milesperhour = fmph;
  }

  gps.stats(&chars, &sentences, &failed);
  Serial.println(failed);
  if (chars == 0)
    Serial.println("** No characters received from GPS: check wiring **");

  // This runs every 2000 milliseconds - every 2 seconds. 
  if (millis() - logTime > 2000) {
    Temp = dht.readTemperature();
    if (Temp < 10) {
      analogWrite(5, 255); //make sure we are writing at least 2.5V to keep the transistor on
      Serial.println("hi"); 
    } else {
      analogWrite(5, 0); // 5 is MOSFET Pin
      Serial.println("low"); 
    }
    logTime = int(millis() / 2000) * 2000;
    Serial.println("logging at " + String(logTime));
    logAcc();
    logFiler();
  }
}

/*
 * Prints header/labels to log. 
 */
void printLogfileHeader() {
  logfile.print("Time, ");
  logfile.print("AccX, ");
  logfile.print("AccY, ");
  logfile.print("AccZ, ");
  logfile.print("Temp");
  logfile.print("pixix");
  logfile.print("pixiy");
  logfile.print("pixiwidth");
  logfile.println("pixiheight");
  logfile.flush();
}

/*
 * Log acceleration data. 
 */
void logAcc() {
  int xRead = analogRead(xPin);
  int yRead = analogRead(yPin);
  int zRead = analogRead(zPin);

  int xAng = map(xRead, minVal, maxVal, -90, 90);
  int yAng = map(yRead, minVal, maxVal, -90, 90);
  int zAng = map(zRead, minVal, maxVal, -90, 90);

  x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
  y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
  z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);

  //Output the caculations
  Serial.print("x: ");
  Serial.print(x);
  Serial.print(" | y: ");
  Serial.print(y);
  Serial.print(" | z: ");
  Serial.println(z);
}

/*
 * Log every piece of sensor data. 
 */
void logFiler() {
  logfile.print(millis());
  logfile.print(", ");
  logfile.print(x);
  logfile.print(", ");
  logfile.print(y);
  logfile.print(", ");
  logfile.print(z);
  logfile.print(", ");
  //  logfile.print(temp);
  //  logfile.print(", ");
  logfile.print(largestBlock[0]);
  logfile.print(", ");
  logfile.print(largestBlock[1]);
  logfile.print(", ");
  logfile.print(largestBlock[2]);
  logfile.print(", ");
  logfile.print(largestBlock[3]);
  logfile.print(", ");
  logfile.print(latitude);
  logfile.print(", ");
  logfile.print(longitude);
  logfile.print(", ");
  logfile.print(altitude_1);
  logfile.print(", ");
  logfile.println(milesperhour);
  logfile.flush();
}

