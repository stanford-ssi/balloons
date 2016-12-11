/*
   Stanford Student Space Initiative
   Balloons Launch 49 | HABEES test | December 2016
   Davy Ragland   | dragland@stanford.edu
   Zach Belateche | zachbela@stanford.edu
   Alex Mallery   | amallery@stanford.edu
   Kirill Safin   | ksafin@stanford.edu
   Aria Tedjarati | satedjarati@stanford.edu
*/

/*********************************************************************
 * File: SSI-49_MEDUSA.ino
 * --------------------------
 * Code for MEDUSA payload
 * Executes scientific payload data collection.
*********************************************************************/

/*********************************************************************
                            SETUP
*********************************************************************/
/* ****************  LIBRARY IMPORTS  ****************  */
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

/* ****************  EDITABLE CONSTANTS  ****************  */
String    MISSION_NUMBER   = "SSI-49";
String    CSV_DATA_HEADER  = "MILLIS,LOOP,ALT_BMP,GAS_1,GAS_2,GAS_3,GAS_4,RADS";

/* ****************  TEENSY PIN OUTS  ****************  */
uint8_t  GAS_SENSE_1       =   9;
uint8_t  GAS_SENSE_2       =  10 ;
uint8_t  GAS_SENSE_3       =  11;
uint8_t  GAS_SENSE_4       =  12;
uint8_t  RAD_SENSE         =   5; // to 8
uint8_t SD_CS              =  15;

/* ****************  GLOBAL OBJECTS  ****************  */
File dataFile;
File logFile;

/* ****************  FUNCTION PROTOTYPES  ****************  */
static void   readData(void);
static void   setPinMode(void);
static void   printHeader(void);
static void   printToSerialAndLog(String text);
static void   printData(void);
static void   logData(void);
static void   updateTiming(void);
static void   readCan(void);
static double parseMessage();
static double countGeigers(void);
static void   measure_GAS(void);

/*********************************************************************
                             DATA
 *********************************************************************/
String  CANmessage          = "";
double  minutes             = 0.0;
float   loopStartTime       = 0.0;
float   elapsedSeconds      = 0.0;
float   overflowSeconds     = 0.0;
double  ALTITUDE_BMP        = 0;
double  GAS_1               = 0;
double  GAS_2               = 0;
double  GAS_3               = 0;
double  GAS_4               = 0;
double  RADs                = 0;

/*********************************************************************
                             BOOT
 *********************************************************************/
void setup() {
  Serial.begin(9600);
  delay(500);
  if(!SD.begin(SD_CS)){
    Serial.println("Could not find a valid SD Card, check wiring!");
  }
  printHeader();
  setPinMode();
}

/*********************************************************************
                             MAIN
 *********************************************************************/
void loop() {
  loopStartTime = millis();
  readCan();
  readData();
  logData();
  printData();
  updateTiming();
  delay(100);
}
/**********************************************************************
                           FUNCTIONS
 *********************************************************************/

/*
   function: readData
   usage: readData();
   ---------------------------------
   This function updates the data global values.
*/
static void readData(void){
  ALTITUDE_BMP   = parseMessage(); 
  RADs = countGeigers();
  measure_GAS();
}

/*********************************************************************
                           HELPERS
 *********************************************************************/
/*
   function: setPinMode
   usage: setPinMode(void);
   ---------------------------------
   This function initializes the pinmodes. 
*/
static void setPinMode(void){
  pinMode(GAS_SENSE_1,  INPUT);
  pinMode(GAS_SENSE_2,  INPUT);
  pinMode(GAS_SENSE_3,  INPUT);
  pinMode(GAS_SENSE_4,  INPUT);
  pinMode(RAD_SENSE,  INPUT);
}

/*
   function: printHeader
   usage: printHeader();
   ---------------------------------
   This function prints the header for the data file.
*/
static void printHeader(void){
  dataFile = SD.open("data.txt", FILE_WRITE);
  if(dataFile){
    dataFile.println("Stanford Student Space Initiative Balloons Launch " + MISSION_NUMBER + "\n" + CSV_DATA_HEADER);
    dataFile.close();
    printToSerialAndLog("Stanford Student Space Initiative Balloons Launch " + MISSION_NUMBER + "\n" + CSV_DATA_HEADER);
  }
  else{
    printToSerialAndLog("error opening \"data.txt\"");
  }  
}

/*
   function: printToSerialAndLog
   usage: printToSerialAndLog(String text);
   ---------------------------------
   This function logs logs onto Serial and the 3D card.
*/
static void printToSerialAndLog(String text){
  logFile = SD.open("log.txt", FILE_WRITE);
  if(logFile){
    logFile.println(text);
    logFile.close();
  }
  else {
    Serial.println("error opening \"log.txt\"");
  }
}

/*
   function: printData
   usage: printData();
   ---------------------------------
   This function logs data to the 3D card.
*/
static void printData(void) {
    Serial.print(millis());
    Serial.print(",");
    Serial.print(elapsedSeconds*1000);
    Serial.print(",");
    Serial.print(ALTITUDE_BMP);
    Serial.print(",");
    Serial.print(GAS_1);
    Serial.print(",");
    Serial.print(GAS_2);
    Serial.print(",");
    Serial.print(GAS_3);
    Serial.print(",");
    Serial.print(GAS_4);
    Serial.print(",");
    Serial.print(RADs);
}

/*
   function: logData
   usage: logData();
   ---------------------------------
   This function logs data to the 3D card.
*/
static void logData(void){
  dataFile = SD.open("data.txt", FILE_WRITE);
  if(dataFile){
    dataFile.print(millis());
    dataFile.print(",");
    dataFile.print(elapsedSeconds*1000);
    dataFile.print(",");
    dataFile.print(ALTITUDE_BMP);
    dataFile.print(",");
    dataFile.print(GAS_1);
    dataFile.print(",");
    dataFile.print(GAS_2);
    dataFile.print(",");
    dataFile.print(GAS_3);
    dataFile.print(",");
    dataFile.print(GAS_4);
    dataFile.print(",");
    dataFile.print(RADs);
    dataFile.close();
  }
  else {
    printToSerialAndLog("error opening \"data.txt\"");
  }
}

/*
   function: updateTiming
   usage: updateTiming();
   ---------------------------------
   This function updates the data global values.
*/
static void updateTiming(void){
  elapsedSeconds  =  (float)(((float)millis() - loopStartTime) / 1000.0);
  overflowSeconds =  (float)(((float)millis() - loopStartTime) / 1000.0) - elapsedSeconds;
  minutes         += (float)(((float)millis() - loopStartTime) / 1000.0 / 60.0);
}

/*
   function: readCan
   usage: readCan();
   ---------------------------------
   This function reads messages through the CAN Bus. 
*/
static void readCan() {
}


static double parseMessage() {
   return 331456.78;
}
static double countGeigers(void) {
  return 420;
}
static void   measure_GAS(void) {
  GAS_1 = 69;
  GAS_2 = 69;
  GAS_3 = 69;
  GAS_4 = 69;
}
