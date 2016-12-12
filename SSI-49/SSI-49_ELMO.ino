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
   File: SSI-49_ELMO.ino
   --------------------------
   Code for ELMO avionics
   Executes essential fight controler tasks.
*********************************************************************/


/*********************************************************************
                            SETUP
*********************************************************************/
/* ****************  LIBRARY IMPORTS  ****************  */
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <PID_v1.h>
#include <TinyGPS++.h>
#include <IridiumSBD.h>
#include <FlexCAN.h>
#include <RTClib.h>
#include "afsk.h"
#include "aprs.h"
#include "ax25.h"
#include "progmemSin.h"
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MAX31855.h>
#include <Adafruit_MCP23017.h>
#include <Adafruit_MPL3115A2.h>

/* ****************  EDITABLE CONSTANTS  ****************  */
String    MISSION_NUMBER   = "SSI-49";
String    CSV_DATA_HEADER  = "TIME,MILLIS,LOOP,VOLTAGE,ALT_GPS,ALT_BMP,ASCENT_RATE,TEMP_IN,TEMP_EXT,LAT,LONG,SPEED_GPS,PRESS_BMP,CURRENT,CUTDOWN_STATE,MESSAGES SENT";
bool      ENABLE_CUTDOWN   = true;
const int AscentRateSize   = 200;
uint16_t  CUTDOWN_ALT      = 20000;
uint16_t  DEBUG_ALT        = 300;
double    PID_setpoint     =  0;
long      TRANSMIT_RATE    =  5000;

/* ****************  TEENSY PIN OUTS  ****************  */
uint8_t SD_CS              =  21;
uint8_t ROCKBLOCK_SLEEP    =   9;
uint8_t VMON               = A13;
uint8_t BMP_CS1            =  20;
uint8_t BMP_CS2            =  15;
uint8_t THERMOCPL_CS       =  22;
uint8_t CUTDOWN_PIN        =  25;
uint8_t HEATER_PIN         =  23;
uint8_t GPS_ENABLE         =  26;

uint8_t draTX              = A16;
uint8_t draRX              = A17;
uint8_t PTT                = A2; 
uint8_t radioEn            = A0;

/* ****************  MULTIPLEXER PIN OUTS  ****************  */
uint8_t CAN_GOOD           =   0;
uint8_t RB_GOOD            =   1;
uint8_t GPS_GOOD           =   2;
uint8_t HEARTBEAT          =   3;
uint8_t BAT_GOOD           =   4;
uint8_t I_GOOD             =   5;
uint8_t P_GOOD             =   6;
uint8_t T_GOOD             =   7;

/* ****************  GLOBAL OBJECTS  ****************  */
#define HWSERIAL Serial1  
IridiumSBD isbd(Serial3, ROCKBLOCK_SLEEP);
TinyGPSPlus gps;
RTC_DS1307 rtc;
File dataFile;
File logFile;
FlexCAN CANbus(1000000);
SoftwareSerial radioSerial(draTX,draRX); 
Adafruit_BMP280 bme1(BMP_CS1);
Adafruit_BMP280 bme2(BMP_CS2);
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
Adafruit_MAX31855 thermocouple(THERMOCPL_CS);
Adafruit_MCP23017 mcp;
double PID_output;
double PID_temperature;
PID    pid(&PID_temperature, &PID_output, &PID_setpoint, 2, 5, 1, DIRECT);
PathAddress addresses[3] = {
  {(char*) "APRS", 0},
  {(char*) "KM6HBK",11},
  {(char*) "WIDE2-2",11}
};

/* ****************  FUNCTION PROTOTYPES  ****************  */
bool          ISBDCallback();
static void   readData(void);
static void   writeLEDS(void);
static void   writeLED(uint8_t PIN, bool green); 
static void   initSensors(void);
static void   setPinMode(void);
static void   runHeaters(void);
static void   runCutdown(void);
static void   printHeader(void);
static void   printToSerialAndLog(String text);
static void   printData(void);
static void   logData(void);
static void   updateTiming(void);
static void   smartdelay(unsigned long ms);
static double getTemp_EXT();
static double getTEMP_IN(void);
static double getPRESS(void);
static double getALTITUDE(void);
static String getTime(void);
static String printDigits(int digits);
static float  calcAscentRate(void);
static void radioSetup();
static void   CANTransmission();
static void   APRSTransmission();
void          satelliteTransmission();
void          set_GPS_flight_mode();

/*********************************************************************
                             DATA
 *********************************************************************/
float  commBeaconInterval   = 0.05;
float  COMM_BEACON_INTERVAL = 2.0;
double minutes              = 0.0;
float  loopStartTime        = 0.0;
float  elapsedSeconds       = 0.0;
float  overflowSeconds      = 0.0;
float  lastGPSCall          = 0.0;
float  lastLEDCall          = 0.0;
float  lastALTCall          = 0.0;
float  currALTCall          = 0.0;
double ALTITUDE_LAST        = 0;
byte   gps_set_sucess       = 0;
byte   RB_set_sucess        = 0;
bool   CAN_set_sucess       = 0;
size_t bufferSize           = 0;
boolean gpsStatus[] = {false, false, false, false, false, false, false};
byte settingsArray[] = {0x03, 0xFA, 0x00, 0x00, 0xE1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
float  AscentRateBuffer[AscentRateSize];
volatile long msAtLastTX    = 0;
double  VOLTAGE       = 0;
double  ALTITUDE_GPS  = 0;
double  ALTITUDE_BMP  = 0;
double  ASCENT_RATE   = 0;
double  TEMP_IN       = 0;
double  TEMP_EXT      = 0;
float   LAT           = 0;
float   LONG          = 0;
bool    CUTDOWN_STATE = 0;
int     messagesSent  = 0;
double  SPEED_GPS     = 0;
double  PRESS_BMP     = 0;
double  CURRENT       = 0;
String  TIMER         = "";
bool    BLINK         = 0;

/*********************************************************************
                             BOOT
 *********************************************************************/
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  delay(500);
  if (!SD.begin(SD_CS)) Serial.println("Could not find a valid SD Card, check wiring!");
  printHeader();
  initSensors();
  setPinMode();
//  set_GPS_flight_mode();
//  radioSetup();
//  isbd.attachConsole(Serial);
//  isbd.attachDiags(Serial);
//  isbd.setPowerProfile(1);
//  Serial3.begin(19200);
//  delay(5000);
//  isbd.begin();
}

/*********************************************************************
                             MAIN
 *********************************************************************/
void loop() {
//  loopStartTime = millis();
//  readData();
//  writeLEDS();
  logData();
//  printData();
//  runHeaters();
//  runCutdown();
//  satelliteTransmission();
//  CANTransmission();
//  APRSTransmission();
//  updateTiming();
//  smartdelay(100);
}
/**********************************************************************
                           FUNCTIONS
 *********************************************************************/
/*
  function: ISBDCallback
  usage: ISBDCallback();
  ---------------------------------
  This function loops back into the main
  call while waiting for Rock Block responces.
*/
bool ISBDCallback() {
  loopStartTime = millis();
  readData();
  writeLEDS();
  logData();
  printData();
  runHeaters();
  runCutdown();
  CANTransmission();
  APRSTransmission();
  updateTiming();
  smartdelay(100);
  RB_set_sucess = 1;
  return true;
}


/*
   function: getGPSValue
   usage: getGPSValue();
   ---------------------------------
   This function gets the GPS valaues
*/
void getGPSValue() {
  if (gps.location.isValid()) {
    LAT = gps.location.lat();
    LONG = gps.location.lng();
    SPEED_GPS = gps.speed.kmph();
    ALTITUDE_GPS = gps.altitude.meters();
    lastGPSCall  = millis();
  }
}

/*
   function: readData
   usage: readData();
   ---------------------------------
   This function updates the data global values.
*/
static void readData(void) {
  getGPSValue();
  TIMER         = getTime();
  printToSerialAndLog("Calling GPS");
  VOLTAGE       = (analogRead(VMON) / 310.0) * 4;
//  CURRENT       = 0.0
  ALTITUDE_LAST = ALTITUDE_BMP;
  TEMP_EXT      = getTemp_EXT();
  TEMP_IN       = getTEMP_IN();
  PRESS_BMP     = getPRESS();
  ALTITUDE_BMP  = getALTITUDE();
  ASCENT_RATE   = calcAscentRate();
  BLINK = !BLINK;
}

/*
   function: writeLEDS
   usage: writeLEDS(void);
   ---------------------------------
   This function visualy error checks the system.
*/
static void writeLEDS(void) {
  if (ALTITUDE_BMP < DEBUG_ALT) {
    if (CAN_set_sucess) writeLED(CAN_GOOD, true);
    else writeLED(CAN_GOOD, false);
    if (RB_set_sucess) writeLED(RB_GOOD, true);
    else writeLED(RB_GOOD, false);
    if (LAT != 1000.0 && LAT != 0.0 && LONG != 1000.0 && LONG != 0.0) writeLED(GPS_GOOD, true);
    else writeLED(GPS_GOOD, false);
    if (BLINK) writeLED(HEARTBEAT, true);
    else writeLED(HEARTBEAT, false);
    if (VOLTAGE >= 3.63) writeLED(BAT_GOOD, true);
    else writeLED(BAT_GOOD, false);
    if (CURRENT > 0.0 && CURRENT <= 0.5) writeLED(I_GOOD, true);
    else writeLED(I_GOOD, false);
    if (ALTITUDE_BMP > -50 && ALTITUDE_BMP < 200) writeLED(P_GOOD, true);
    else writeLED(P_GOOD, false);
    if (TEMP_IN > 15 && TEMP_IN < 50) writeLED(T_GOOD, true);
    else writeLED(T_GOOD, false);
    lastLEDCall = millis();
  }
}
/*********************************************************************
                           HELPERS
 *********************************************************************/
/*
   function: initSensors
   usage: initSensors();
   ---------------------------------
   This function initializes the BMP280, MS5803-1BA, DS1307 RTC, and MCP23017.
*/
static void initSensors(void) {
  if (!bme1.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    
    while (1);
  }
  if (!bme2.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
  if (!rtc.begin()) {
    printToSerialAndLog("Could not find a valid RTC, check wiring!");
  }
  if (! baro.begin()) {
    Serial.println("Couldnt find MPL3115A2");
  }
  mcp.begin();
  CANbus.begin();
}

/*
   function: setPinMode
   usage: setPinMode(void);
   ---------------------------------
   This function initializes the pinmodes.
*/
static void setPinMode(void) {
//  for (size_t i = 0; i < 16; i++) mcp.pinMode(i,  OUTPUT);
//  for (size_t i = 0; i < 8; i++) writeLED(i, false);
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(CUTDOWN_PIN,  OUTPUT);
  pinMode(GPS_ENABLE,  OUTPUT);
  digitalWrite(CUTDOWN_PIN, LOW);
  digitalWrite(GPS_ENABLE, LOW);
  pinMode(VMON,  INPUT);
}

/*
   function: writeLED
   usage: writeLED(PIN, green)
   ---------------------------------
   This function sets a pin to red or green.
*/
static void writeLED(uint8_t PIN, bool green) {
  if (green) {
    mcp.digitalWrite(PIN, HIGH);
    mcp.digitalWrite(15 - PIN, LOW); 
  }
 else {
    mcp.digitalWrite(PIN, LOW);
    mcp.digitalWrite(15- PIN, HIGH); 
  }
}

/*
   function: runHeaters
   usage: runHeaters(void);
   ---------------------------------
   This function powers the heaters.
*/
static void runHeaters(void) {
  PID_temperature = TEMP_IN;
  pid.Compute();
  if (PID_output != 0.0) {
    analogWrite(HEATER_PIN, PID_output / 2 + 127.50);
  }
  else {
    analogWrite(HEATER_PIN, 0);
  }
}

/*
   function: runCutdown
   usage: runCutdown(void);
   ---------------------------------
   This function cuts down at a set altitude.
*/
static void runCutdown(void) {
  if (!ENABLE_CUTDOWN) return;
  if (ALTITUDE_BMP > CUTDOWN_ALT && ALTITUDE_LAST > CUTDOWN_ALT) {
    analogWrite(CUTDOWN_PIN, 255);
    CUTDOWN_STATE = 1;
    smartdelay(30000);
    analogWrite(CUTDOWN_PIN, 0);
  }
}

/*
   function: printHeader
   usage: printHeader();
   ---------------------------------
   This function prints the header for the data file.
*/
static void printHeader(void) {
  dataFile = SD.open("data.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println("Stanford Student Space Initiative Balloons Launch " + MISSION_NUMBER + "\n" + CSV_DATA_HEADER);
    dataFile.close();
    printToSerialAndLog("Stanford Student Space Initiative Balloons Launch " + MISSION_NUMBER + "\n" + CSV_DATA_HEADER);
  }
  else {
    printToSerialAndLog("error opening \"data.txt\"");
  }
}

/*
   function: printToSerialAndLog
   usage: printToSerialAndLog(String text);
   ---------------------------------
   This function logs logs onto Serial and the 3D card.
*/
static void printToSerialAndLog(String text) {
  logFile = SD.open("log.txt", FILE_WRITE);
  if (logFile) {
    logFile.print(TIMER);
    logFile.print(",");
    logFile.println(text);
    logFile.close();
  }
  else {
    Serial.println("error opening \"log.txt\"");
  }
  if (ALTITUDE_BMP < DEBUG_ALT) {
    Serial.println(text);
  }
}

/*
   function: printData
   usage: printData();
   ---------------------------------
   This function logs data to the 3D card.
*/
static void printData(void) {
  if (ALTITUDE_BMP < DEBUG_ALT) {
    Serial.print(TIMER);
    Serial.print(",");
    Serial.print(millis());
    Serial.print(",");
    Serial.print(elapsedSeconds * 1000);
    Serial.print(",");
    Serial.print(VOLTAGE);
    Serial.print(",");
    Serial.print(ALTITUDE_GPS);
    Serial.print(",");
    Serial.print(ALTITUDE_BMP);
    Serial.print(",");
    Serial.print(ASCENT_RATE);
    Serial.print(",");
    Serial.print(TEMP_IN);
    Serial.print(",");
    Serial.print(TEMP_EXT);
    Serial.print(",");
    Serial.print(String(LAT, 4));
    Serial.print(",");
    Serial.print(String(LONG, 4));
    Serial.print(",");
    Serial.print(SPEED_GPS);
    Serial.print(",");
    Serial.print(PRESS_BMP);
    Serial.print(",");
    Serial.print(CURRENT);
    Serial.print(",");
    Serial.print(CUTDOWN_STATE);
    Serial.print(",");
    Serial.println(messagesSent);
  }
}

/*
   function: logData
   usage: logData();
   ---------------------------------
   This function logs data to the 3D card.
*/
static void logData(void) {
  
  String dataString = "";
  for (int analogPin = 0; analogPin < 3; analogPin++) {
    int sensor = analogRead(analogPin);
    dataString += String(sensor);
    if (analogPin < 2) {
      dataString += ","; 
    }
  }
  File dataFile = SD.open("data.txt", FILE_WRITE);
  if (dataFile) {
dataFile.print(TIMER);
    dataFile.print(",");
    dataFile.print(millis());
    dataFile.print(",");
    dataFile.print(elapsedSeconds * 1000);
    dataFile.print(",");
    dataFile.print(VOLTAGE);
    dataFile.print(",");
    dataFile.print(ALTITUDE_GPS);
    dataFile.print(",");
    dataFile.print(ALTITUDE_BMP);
    dataFile.print(",");
    dataFile.print(ASCENT_RATE);
    dataFile.print(",");
    dataFile.print(TEMP_IN);
    dataFile.print(",");
    dataFile.print(TEMP_EXT);
    dataFile.print(",");
    dataFile.print(String(LAT, 4));
    dataFile.print(",");
    dataFile.print(String(LONG, 4));
    dataFile.print(",");
    dataFile.print(SPEED_GPS);
    dataFile.print(",");
    dataFile.print(PRESS_BMP);
    dataFile.print(",");
    dataFile.print(CURRENT);
    dataFile.print(",");
    dataFile.print(CUTDOWN_STATE);
    dataFile.print(",");
    dataFile.println(messagesSent);
    dataFile.close();
    Serial.println("good");
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
static void updateTiming(void) {
  elapsedSeconds  =  (float)(((float)millis() - loopStartTime) / 1000.0);
  overflowSeconds =  (float)(((float)millis() - loopStartTime) / 1000.0) - elapsedSeconds;
  minutes         += (float)(((float)millis() - loopStartTime) / 1000.0 / 60.0);
}

/*
   function: getTemp_EXT
   usage: getTemp_EXT()
   ---------------------------------
   This function gets the external temperature.
*/
static double getTemp_EXT() {
  if (!isnan(thermocouple.readCelsius())) return TEMP_EXT;
  else return thermocouple.readCelsius();
}
/*
   function: getTEMP_IN
   usage: getTEMP_IN(void);
   ---------------------------------
   This function returns a sensor fused reading.
*/
static double getTEMP_IN() {
  double temp_1 = bme1.readTemperature();
  double temp_2 = bme2.readTemperature();
  if (temp_1 >= -20 && temp_2 >= -20) return (temp_1 + temp_2) / 2;
  printToSerialAndLog("error reading BMP280");
  if (temp_1 >= -20 && temp_2 < -20) return temp_1;
  if (temp_2 >= -20 && temp_1 < -20) return temp_2;
  else return baro.getTemperature();
}

/*
   function: getPRESS
   usage: getPRESS(void);
   ---------------------------------
   This function returns a sensor fused reading.
*/
static double getPRESS() {
  double press_1 = bme1.readPressure();
  double press_2 = bme2.readPressure();
  if (press_1 >= 1000 && press_2 >= 1000) return (press_1 + press_2) / 2;
  printToSerialAndLog("error reading BMP280");
  if (press_1 >= 1000 && press_2 <= 0) return press_1;
  if (press_2 >= 1000 && press_1 <= 0) return press_2;
  else return baro.getPressure();
}

/*
   function: getALTITUDE
   usage: getALTITUDE(void);
   ---------------------------------
   This function returns a sensor fused reading.
*/
static double getALTITUDE() {
  double altitude_1 = bme1.readAltitude(1013.25);
  double altitude_2 = bme2.readAltitude(1013.25);
  if (altitude_1 >= -50 && altitude_2 >= -50) return (altitude_1 + altitude_2) / 2;
  printToSerialAndLog("error reading MPL3115A2");
  if (altitude_1 >= -50 && altitude_2 <= -50) return altitude_1;
  if (altitude_2 >= -50 && altitude_1 <= -50) return altitude_2;
  else return baro.getAltitude();
}
  

/*
   function: getTime
   usage: getTime(void);
   ---------------------------------
   This function formats the time string.
*/
static String getTime(void) {
  String timer = "";
  DateTime now = rtc.now();
  timer += now.hour();
  timer += printDigits(now.minute());
  timer += printDigits(now.second());
  timer += '/';
  timer += now.year();
  timer += '/';
  timer += now.month();
  timer += '/';
  timer += now.day();
  return timer;
}

/*
   function: printDigits
   usage: printDigits(int digits);
   ---------------------------------
   This function formats digits for the time.
*/
static String printDigits(int digits) {
  String timer = "";
  timer += ":";
  if (digits < 10) {
    timer += '0';
  }
  timer += digits;
  return timer;
}

/*
   function: calcAscentRate
   usage: calcAscentRate();
   return: ascent rate from last two data points
   ---------------------------------
   This function calculates the linearized ascent rate from
   the last two altitude readings.
*/
static float calcAscentRate(void) {
  for (int i = 0; i < AscentRateSize - 1; i++) {
    AscentRateBuffer[i] = AscentRateBuffer[i + 1];
  }
  lastALTCall = currALTCall;
  currALTCall = millis();
  AscentRateBuffer[AscentRateSize - 1] = (ALTITUDE_BMP - ALTITUDE_LAST) / ((currALTCall - lastALTCall) / 1000.0);
  float AscentRateTotal = 0;
  for (int i = 0; i < AscentRateSize; i++) {
    AscentRateTotal += AscentRateBuffer[i];
  }
  return AscentRateTotal / AscentRateSize;
}

/*
   function: smartdelay
   usage: smartdelay(unsigned long ms);
   ---------------------------------
   This function sleeps while also reading GPS data.
*/
static void smartdelay(unsigned long ms) {
  unsigned long startt = millis();
  do {
    while (HWSERIAL.available())
      gps.encode(HWSERIAL.read());
  } while (millis() - startt < ms);
}

static void radioSetup(){
  pinMode(radioEn,OUTPUT);
  digitalWrite(radioEn,LOW); //Write low to power the radio on
  pinMode(draTX,INPUT);
  pinMode(draRX,OUTPUT);// set up software serial pins
  Serial.begin(9600);
  radioSerial.begin(9600);
  pinMode(PTT,OUTPUT);
  digitalWrite(PTT,LOW);
  delay(500);
  radioSerial.print("AT+DMOCONNECT\r\n");
  delay(200);
  digitalWrite(PTT,HIGH);
  delay(500);
  digitalWrite(PTT,LOW);
  radioSerial.print("AT+DMOSETGROUP=");      
  radioSerial.print(1,1);
  radioSerial.print(",");
  radioSerial.print(144.390,4); 
  radioSerial.print(",");
  radioSerial.print(144.390,4);
  radioSerial.print(",");
  radioSerial.print("0000");
  radioSerial.print(",");
  radioSerial.print(0);
  radioSerial.print(",");
  radioSerial.print("0000");
  radioSerial.print("\r\n"); 
  delay(500);
  digitalWrite(PTT,HIGH);
  delay(500);
  digitalWrite(PTT,LOW);
  radioSerial.print("AT+SETFILTER=1,0,0\r\n");
  aprs_setup(2,PTT,700,0,0);
  digitalWrite(PTT,HIGH);
}

/*
   function: CANTransmission
   usage: CANTransmission();
   ---------------------------------
   This function sends the altitude over the CAN bus.
*/
static void CANTransmission() {
  CAN_message_t msg;
  msg.id = 0x222;
  msg.len = 8;
  void* pointer =  &ALTITUDE_BMP;
  for(byte i=0; i<8; i++){
    msg.buf[i] = *((uint8_t*)(pointer)+i);
  }
  if (CANbus.write(msg) == 1) CAN_set_sucess = 1;
  else CAN_set_sucess = 0;
}

/*
   function: APRSTransmission
   usage: APRSTransmission();
   ---------------------------------
   This function sends messages through the APRS radio.
*/
static void APRSTransmission() {
    if(msAtLastTX == 0 || millis() - msAtLastTX > TRANSMIT_RATE) {
      printToSerialAndLog("Beginning to talk over APRS...");
      String messageToSend = "";
      messageToSend += VOLTAGE;
      messageToSend += ",";
      messageToSend += ALTITUDE_GPS;
      messageToSend += ",";
      messageToSend += ALTITUDE_BMP;
      messageToSend += ",";
      messageToSend += ASCENT_RATE;
      messageToSend += ",";
      messageToSend += TEMP_IN;
      messageToSend += ",";
      messageToSend += TEMP_EXT;
      messageToSend += ",";
      messageToSend += String(LAT, 4);
      messageToSend += ",";
      messageToSend += String(LONG, 4);
      messageToSend += ",";
      messageToSend += CURRENT;
      messageToSend += ",";
      messageToSend += CUTDOWN_STATE;
      messageToSend += ",";
      messageToSend += messagesSent;
      aprs_send(addresses,3,messageToSend);
      msAtLastTX = millis();
      Serial.println("sent");
  }
}


/*
   function: satelliteTransmission
   usage: satelliteTransmission();
   ---------------------------------
   This function sends messages through the Rock Block.
*/
void satelliteTransmission() {
  if ((minutes < commBeaconInterval)) {
    printToSerialAndLog("Beginning to talk to the RockBLOCK...");
    messagesSent += 1;
    printToSerialAndLog("Transmitting message: ");
    printToSerialAndLog((String)messagesSent);
    printToSerialAndLog("Beginning to talk to the RockBLOCK...");
    printToSerialAndLog("Sending RB message");
    char outBuffer[200];
    String messageToSend = "";
    messageToSend += VOLTAGE;
    messageToSend += ",";
    messageToSend += ALTITUDE_GPS;
    messageToSend += ",";
    messageToSend += ALTITUDE_BMP;
    messageToSend += ",";
    messageToSend += ASCENT_RATE;
    messageToSend += ",";
    messageToSend += TEMP_IN;
    messageToSend += ",";
    messageToSend += TEMP_EXT;
    messageToSend += ",";
    messageToSend += String(LAT, 4);
    messageToSend += ",";
    messageToSend += String(LONG, 4);
    messageToSend += ",";
    messageToSend += CURRENT;
    messageToSend += ",";
    messageToSend += CUTDOWN_STATE;
    messageToSend += ",";
    messageToSend += messagesSent;

    for (uint8_t i = 0; i < messageToSend.length(); i++) {
      outBuffer[i] = messageToSend[i];
    }
    uint8_t rxBuffer[200] = {0};
    for (int i = 0; i < messageToSend.length(); i++) {
      rxBuffer[i] = outBuffer[i];
    }
    bufferSize = sizeof(rxBuffer);
    isbd.sendReceiveSBDBinary(rxBuffer, messageToSend.length(), rxBuffer, bufferSize);
    int number = bufferSize;
    printToSerialAndLog("Going to sleep mode");
    delay(2000);
    commBeaconInterval = minutes + COMM_BEACON_INTERVAL;
  }
}

/*
   function: set_GPS_flight_mode
   usage: set_GPS_flight_mode();
   ---------------------------------
   This function sets the GPS so that it won't lock out
   at higher altitude.
*/
void set_GPS_flight_mode() {
  byte *settingsArrayPointer = settingsArray;
  byte gpsSetSuccess = 0;
  Serial.println("Configuring u-Blox GPS initial state...");
  //Generate the configuration string for Navigation Mode
  byte setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, *settingsArrayPointer, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  calcChecksum(&setNav[2], sizeof(setNav) - 4);
  //Generate the configuration string for Data Rate
  byte setDataRate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, settingsArrayPointer[1], settingsArrayPointer[2], 0x01, 0x00, 0x01, 0x00, 0x00, 0x00};
  calcChecksum(&setDataRate[2], sizeof(setDataRate) - 4);
  //Generate the configuration string for Baud Rate
  byte setPortRate[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, settingsArrayPointer[3], settingsArrayPointer[4], settingsArrayPointer[5], 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  calcChecksum(&setPortRate[2], sizeof(setPortRate) - 4);
  byte setGLL[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B};
  byte setGSA[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32};
  byte setGSV[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39};
  byte setRMC[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40};
  byte setVTG[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};
  delay(2500);
  while(gpsSetSuccess < 3) {
    Serial.print("Setting Navigation Mode... ");
    sendUBX(&setNav[0], sizeof(setNav));  //Send UBX Packet
    gpsSetSuccess += getUBX_ACK(&setNav[2]); //Passes Class ID and Message ID to the ACK Receive function
    if (gpsSetSuccess == 5) {
      gpsSetSuccess -= 4;
      setBaud(settingsArrayPointer[4]);
      delay(1500);
      byte lowerPortRate[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA2, 0xB5};
      sendUBX(lowerPortRate, sizeof(lowerPortRate));
      HWSERIAL.begin(9600);
      delay(2000);      
    }
    if(gpsSetSuccess == 6) gpsSetSuccess -= 4;
    if (gpsSetSuccess == 10) gpsStatus[0] = true;
  }
  if (gpsSetSuccess == 3) Serial.println("Navigation mode configuration failed.");
  gpsSetSuccess = 0;
  while(gpsSetSuccess < 3) {
    Serial.print("Setting Data Update Rate... ");
    sendUBX(&setDataRate[0], sizeof(setDataRate));  //Send UBX Packet
    gpsSetSuccess += getUBX_ACK(&setDataRate[2]); //Passes Class ID and Message ID to the ACK Receive function      
    if (gpsSetSuccess == 10) gpsStatus[1] = true;
    if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
  }
  if (gpsSetSuccess == 3) Serial.println("Data update mode configuration failed.");
  gpsSetSuccess = 0;
  while(gpsSetSuccess < 3 && settingsArrayPointer[6] == 0x00) {
    Serial.print("Deactivating NMEA GLL Messages ");
    sendUBX(setGLL, sizeof(setGLL));
    gpsSetSuccess += getUBX_ACK(&setGLL[2]);
    if (gpsSetSuccess == 10) gpsStatus[2] = true;
    if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
  }
  if (gpsSetSuccess == 3) Serial.println("NMEA GLL Message Deactivation Failed!");
  gpsSetSuccess = 0;
  while(gpsSetSuccess < 3 && settingsArrayPointer[7] == 0x00) {
    Serial.print("Deactivating NMEA GSA Messages ");
    sendUBX(setGSA, sizeof(setGSA));
    gpsSetSuccess += getUBX_ACK(&setGSA[2]);
    if (gpsSetSuccess == 10) gpsStatus[3] = true;
    if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
  }
  if (gpsSetSuccess == 3) Serial.println("NMEA GSA Message Deactivation Failed!");  
  gpsSetSuccess = 0;
  while(gpsSetSuccess < 3 && settingsArrayPointer[8] == 0x00) {
    Serial.print("Deactivating NMEA GSV Messages ");
    sendUBX(setGSV, sizeof(setGSV));
    gpsSetSuccess += getUBX_ACK(&setGSV[2]);
    if (gpsSetSuccess == 10) gpsStatus[4] = true;
    if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
  }
  if (gpsSetSuccess == 3) Serial.println("NMEA GSV Message Deactivation Failed!");  
  gpsSetSuccess = 0;
  while(gpsSetSuccess < 3 && settingsArrayPointer[9] == 0x00) {
    Serial.print("Deactivating NMEA RMC Messages ");
    sendUBX(setRMC, sizeof(setRMC));
    gpsSetSuccess += getUBX_ACK(&setRMC[2]);
    if (gpsSetSuccess == 10) gpsStatus[5] = true;
    if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
  }
  if (gpsSetSuccess == 3) Serial.println("NMEA RMC Message Deactivation Failed!");  
  gpsSetSuccess = 0;
  while(gpsSetSuccess < 3 && settingsArrayPointer[10] == 0x00) {
    Serial.print("Deactivating NMEA VTG Messages ");
    sendUBX(setVTG, sizeof(setVTG));
    gpsSetSuccess += getUBX_ACK(&setVTG[2]);
    if (gpsSetSuccess == 10) gpsStatus[6] = true;
    if (gpsSetSuccess == 5 | gpsSetSuccess == 6) gpsSetSuccess -= 4;
  }
  if (gpsSetSuccess == 3) Serial.println("NMEA VTG Message Deactivation Failed!");
  gpsSetSuccess = 0;
  if (settingsArrayPointer[4] != 0x25) {
    Serial.print("Setting Port Baud Rate... ");
    sendUBX(&setPortRate[0], sizeof(setPortRate));
    setBaud(settingsArrayPointer[4]);
    Serial.println("Success!");
    delay(500);
  }
}
void calcChecksum(byte *checksumPayload, byte payloadSize) {
  byte CK_A = 0, CK_B = 0;
  for (int i = 0; i < payloadSize ;i++) {
    CK_A = CK_A + *checksumPayload;
    CK_B = CK_B + CK_A;
    checksumPayload++;
  }
  *checksumPayload = CK_A;
  checksumPayload++;
  *checksumPayload = CK_B;
}
void sendUBX(byte *UBXmsg, byte msgLength) {
  for(int i = 0; i < msgLength; i++) {
    HWSERIAL.write(UBXmsg[i]);
    HWSERIAL.flush();
  }
  HWSERIAL.println();
  HWSERIAL.flush();
}
byte getUBX_ACK(byte *msgID) {
  byte CK_A = 0, CK_B = 0;
  byte incoming_char;
  boolean headerReceived = false;
  unsigned long ackWait = millis();
  byte ackPacket[10] = {0xB5, 0x62, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  int i = 0;
  while (1) {
    if (HWSERIAL.available()) {
      incoming_char = HWSERIAL.read();
      if (incoming_char == ackPacket[i]) {
        i++;
      }
      else if (i > 2) {
        ackPacket[i] = incoming_char;
        i++;
      }
    }
    if (i > 9) break;
    if ((millis() - ackWait) > 1500) {
      Serial.println("ACK Timeout");
      return 5;
    }
    if (i == 4 && ackPacket[3] == 0x00) {
      Serial.println("NAK Received");
      return 1;
    }
  }
  for (i = 2; i < 8 ;i++) {
  CK_A = CK_A + ackPacket[i];
  CK_B = CK_B + CK_A;
  }
  if (msgID[0] == ackPacket[6] && msgID[1] == ackPacket[7] && CK_A == ackPacket[8] && CK_B == ackPacket[9]) {
    Serial.println("Success!");
    Serial.print("ACK Received! ");
    printHex(ackPacket, sizeof(ackPacket));
    return 10;
  }
  else {
    Serial.print("ACK Checksum Failure: ");
    printHex(ackPacket, sizeof(ackPacket));
    delay(1000);
    return 1;
  }
}
void printHex(uint8_t *data, uint8_t length) // prints 8-bit data in hex
{
  char tmp[length*2+1];
  byte first ;
  int j=0;
  for (byte i = 0; i < length; i++) 
  {
    first = (data[i] >> 4) | 48;
    if (first > 57) tmp[j] = first + (byte)7;
    else tmp[j] = first ;
    j++;

    first = (data[i] & 0x0F) | 48;
    if (first > 57) tmp[j] = first + (byte)7; 
    else tmp[j] = first;
    j++;
  }
  tmp[length*2] = 0;
  for (byte i = 0, j = 0; i < sizeof(tmp); i++) {
    Serial.print(tmp[i]);
    if (j == 1) {
      Serial.print(" "); 
      j = 0;
    }
    else j++;
  }
  Serial.println();
}
void setBaud(byte baudSetting) {
  if (baudSetting == 0x12) HWSERIAL.begin(4800);
  if (baudSetting == 0x4B) HWSERIAL.begin(19200);
  if (baudSetting == 0x96) HWSERIAL.begin(38400);
  if (baudSetting == 0xE1) HWSERIAL.begin(57600);
  if (baudSetting == 0xC2) HWSERIAL.begin(115200);
  if (baudSetting == 0x84) HWSERIAL.begin(230400);
}
