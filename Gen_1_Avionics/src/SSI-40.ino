/*
   Stanford Student Space Initiative
   Balloons Avionics Launch 3 | April 2016
   Davy Ragland   | dragland@stanford.edu
   Kirill Safin   | ksafin@stanford.edu
   Aria Tedjarati | satedjarati@stanford.edu
*/

/*********************************************************************
                            SETUP
*********************************************************************/
/* ****************  LIBRARY IMPORTS  ****************  */
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <MS5803.h>
#include <TinyGPS.h>
#include <IridiumSBD.h>
#include <RTClib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MAX31855.h>
#include <Adafruit_MCP23017.h>

/* ****************  EDITABLE CONSTANTS  ****************  */
String   MISSION_NUMBER   = "SSI-40";
String   CSV_DATA_HEADER  = "TIME,MILLIS,LOOP,VOLTAGE,ALT_GPS,ALT_BMP,ASCENT_RATE,TEMP_IN,TEMP_EXT,LAT,LONG,SPEED_GPS,PRESS_BMP,CURRENT,PRESS_MS5803,TEMP_MS5803,MESSAGES SENT";
bool     ENABLE_CUTDOWN   = true;
const int AscentRateSize   = 200;
uint16_t CUTDOWN_ALT      = 30000;
uint16_t DEBUG_ALT        = 1000;
uint16_t HEATER_SETPOINT  = 0;

/* ****************  TEENSY PIN OUTS  ****************  */
uint8_t SD_CD             =   2;
uint8_t ROCKBLOCK_SLEEP   =   9;
uint8_t SD_CS             =  10;
uint8_t VMON              =  14;
uint8_t IMON              =  15;
uint8_t BMP280_CS         =  20;
uint8_t THERMOCPL_CS      =  22;

/* ****************  MULTIPLEXER PIN OUTS  ****************  */
uint8_t HEARTBEAT         =   0;
uint8_t I_GOOD            =   1;
uint8_t T_GOOD            =   2;
uint8_t P_GOOD            =   3;
uint8_t RB_GOOD           =   4;
uint8_t GPS_GOOD          =   5;
uint8_t SD_GOOD           =   6;
uint8_t BAT_GOOD          =   7;
uint8_t CUTDOWN           =   9;
uint8_t HEATER            =  10;

/* ****************  GLOBAL OBJECTS  ****************  */
#define HWSERIAL Serial1
IridiumSBD isbd(Serial3, ROCKBLOCK_SLEEP);
TinyGPS gps;
RTC_DS1307 rtc;
MS5803 MS_5803 = MS5803();
File dataFile;
File logFile;
Adafruit_BMP280 bme(BMP280_CS);
Adafruit_MAX31855 thermocouple(THERMOCPL_CS);
Adafruit_MCP23017 mcp;

/* ****************  FUNCTION PROTOTYPES  ****************  */
bool          ISBDCallback();
static void   readData(void);
static void   writeLEDS(void);
static void   initSensors(void);
static void   setPinMode(void);
static void   runHeaters(void);
static void   runCutdown(void);
static void   printHeader(void);
static void   printToSerialAndLog(String text);
static void   printData(void);
static void   logData(void);
static void   updateTiming(void);
static void   smartDelay(unsigned long ms);
static String getTime(void);
static String printDigits(int digits);
static float  calcAscentRate(void);
void          satelliteTransmission();
void          set_GPS_flight_mode();
void          sendUBX(uint8_t *MSG, uint8_t len);
boolean       getUBX_ACK(uint8_t *MSG);

/*********************************************************************
                             DATA
 *********************************************************************/
float  commBeaconInterval   = 2.0; 
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
size_t bufferSize           = 0;
float  AscentRateBuffer[AscentRateSize];
uint8_t setNav[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
  0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC
};
double  VOLTAGE       = 0;
double  ALTITUDE_GPS  = 0;
double  ALTITUDE_BMP  = 0;
double  ASCENT_RATE   = 0;
double  TEMP_IN       = 0;
double  TEMP_EXT      = 0;
float   LAT           = 0;
float   LONG          = 0;
int     messagesSent  = 0;
double  SPEED_GPS     = 0;
double  PRESS_BMP     = 0;   
double  CURRENT       = 0;
double  PRESS_MS5803  = 0;
double  TEMP_MS5803   = 0;
String  TIMER         = "";
bool    BLINK         = 0;

/*********************************************************************
                             BOOT
 *********************************************************************/
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  delay(500);
  if(!SD.begin(SD_CS)){
    Serial.println("Could not find a valid SD Card, check wiring!");
  }
  printHeader();
  initSensors();
  setPinMode();
  set_GPS_flight_mode();
  isbd.attachConsole(Serial);
  isbd.attachDiags(Serial);
  isbd.setPowerProfile(1);      
  Serial3.begin(19200);
  delay(5000);
  isbd.begin();
}

/*********************************************************************
                             MAIN
 *********************************************************************/
void loop() {
  loopStartTime = millis();
  readData();
  writeLEDS();
  logData();
  printData();
  runHeaters();
  runCutdown();
  satelliteTransmission();
  updateTiming();
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
bool ISBDCallback(){
  loopStartTime = millis();
  readData();
  writeLEDS();
  logData();
  printData();
  runHeaters();
  runCutdown();
  updateTiming();
  RB_set_sucess = 1;
  return true;
}

/*
   function: readData
   usage: readData();
   ---------------------------------
   This function updates the data global values.
*/
static void readData(void){
  unsigned long age;
  // Poll once/minute
  if(millis() - lastGPSCall > 60000){
    smartDelay(30);
    gps.f_get_position(&LAT, &LONG, &age);
    ALTITUDE_GPS = gps.f_altitude();
    SPEED_GPS    = gps.f_speed_kmph();
    lastGPSCall  = millis();
    printToSerialAndLog("Calling GPS");
  }
  // Poll every loop
  MS_5803.readSensor();
  VOLTAGE       = (analogRead(VMON) / 310.0) * 4;
  ALTITUDE_LAST = ALTITUDE_BMP;
  ALTITUDE_BMP  = bme.readAltitude(1013.25);
  ASCENT_RATE   = calcAscentRate();
  TEMP_IN       = bme.readTemperature();
  PRESS_BMP     = bme.readPressure();   
  CURRENT       = (analogRead(IMON) / 310.0) * 0.2;
  PRESS_MS5803  = MS_5803.pressure() * 10;
  TEMP_MS5803   = MS_5803.temperature();
  TIMER         = getTime();
  if(!isnan(thermocouple.readCelsius())) {
    TEMP_EXT =  thermocouple.readCelsius();
  }
  BLINK = !BLINK;
}

/*
   function: writeLEDS
   usage: writeLEDS(void);
   ---------------------------------
   This function visualy error checks the system.
*/
static void writeLEDS(void){
  if(ALTITUDE_BMP < DEBUG_ALT){
    mcp.digitalWrite(HEARTBEAT, LOW);
    mcp.digitalWrite(I_GOOD,    LOW);
    mcp.digitalWrite(T_GOOD,    LOW);
    mcp.digitalWrite(P_GOOD,    LOW);
    mcp.digitalWrite(RB_GOOD,   LOW);
    mcp.digitalWrite(GPS_GOOD,  LOW);
    mcp.digitalWrite(SD_GOOD,   LOW);
    mcp.digitalWrite(BAT_GOOD,  LOW);
    if(BLINK){
      mcp.digitalWrite(HEARTBEAT, HIGH);
    }
    if(CURRENT <= 0.5){
      mcp.digitalWrite(I_GOOD, HIGH);
    }
    if(TEMP_IN > 15 && TEMP_IN < 50){
      mcp.digitalWrite(T_GOOD, HIGH);
    }
    if(ALTITUDE_BMP > -20 && ALTITUDE_BMP < 200){
      mcp.digitalWrite(P_GOOD, HIGH);
    }
    if(RB_set_sucess){
      mcp.digitalWrite(RB_GOOD, HIGH);
    }
    if(LAT != 1000.0 && LAT != 0.0 && LONG != 1000.0 && LONG != 0.0){
      mcp.digitalWrite(GPS_GOOD, HIGH);
    }
    if(digitalRead(SD_CD) == 0){
      mcp.digitalWrite(SD_GOOD, HIGH);
    }
    if(VOLTAGE >= 3.63){
      mcp.digitalWrite(BAT_GOOD, HIGH);
    } 
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
static void initSensors(void){
  if(!bme.begin()) {
     printToSerialAndLog("Could not find a valid BMP280 sensor, check wiring!");
  } 
  if(!MS_5803.initalizeSensor()){
    printToSerialAndLog("Could not find a valid MS5803 sensor, check wiring!");
  } 
  if(!rtc.begin()){
    printToSerialAndLog("Could not find a valid RTC, check wiring!");
  }  
  mcp.begin();
}

/*
   function: setPinMode
   usage: setPinMode(void);
   ---------------------------------
   This function initializes the pinmodes. 
*/
static void setPinMode(void){
  mcp.pinMode(BLINK,    OUTPUT);
  mcp.pinMode(I_GOOD,   OUTPUT);
  mcp.pinMode(T_GOOD,   OUTPUT);
  mcp.pinMode(P_GOOD,   OUTPUT);
  mcp.pinMode(RB_GOOD,  OUTPUT);
  mcp.pinMode(GPS_GOOD, OUTPUT);
  mcp.pinMode(SD_GOOD,  OUTPUT);
  mcp.pinMode(BAT_GOOD, OUTPUT);
  mcp.pinMode(HEATER,   OUTPUT);
  mcp.pinMode(CUTDOWN,  OUTPUT);
  mcp.digitalWrite(CUTDOWN, LOW);
  pinMode(SD_CD, OUTPUT);
  pinMode(VMON,  INPUT);
  pinMode(IMON,  INPUT);
}

/*
   function: runHeaters
   usage: runHeaters(void);
   ---------------------------------
   This function powers the heaters. 
*/
static void runHeaters(void){
  if(TEMP_IN <= HEATER_SETPOINT){
    mcp.digitalWrite(HEATER, HIGH);
  }
  else{
    mcp.digitalWrite(HEATER, LOW);
  }
}

/*
   function: runCutdown
   usage: runCutdown(void);
   ---------------------------------
   This function cuts down at a set altitude. 
*/
static void runCutdown(void){
  if(!ENABLE_CUTDOWN) return;
  if(ALTITUDE_BMP > CUTDOWN_ALT && ALTITUDE_LAST > CUTDOWN_ALT){
    mcp.digitalWrite(CUTDOWN, HIGH);
    smartDelay(30000);
    mcp.digitalWrite(CUTDOWN, LOW);
  }
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
    printToSerialAndLog("Stanford Student Space Initiative Balloons Launch " + MISSION_NUMBER + "\n" + CSV_DATA_HEADER);
    dataFile.println("Stanford Student Space Initiative Balloons Launch " + MISSION_NUMBER + "\n" + CSV_DATA_HEADER);
    dataFile.close();
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
    logFile.print(TIMER);
    logFile.print(",");
    logFile.println(text);
    logFile.close();
  }
  else {
    Serial.println("error opening \"log.txt\"");
  }
  if(ALTITUDE_BMP < DEBUG_ALT){
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
  if(ALTITUDE_BMP < DEBUG_ALT){
    Serial.print(TIMER);
    Serial.print(",");
    Serial.print(millis());
    Serial.print(",");
    Serial.print(elapsedSeconds*1000);
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
    Serial.print(PRESS_MS5803);
    Serial.print(",");
    Serial.print(TEMP_MS5803);
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
static void logData(void){
  dataFile = SD.open("data.txt", FILE_WRITE);
  if(dataFile){
    dataFile.print(TIMER);
    dataFile.print(",");
    dataFile.print(millis());
    dataFile.print(",");
    dataFile.print(elapsedSeconds*1000);
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
    dataFile.print(PRESS_MS5803);
    dataFile.print(",");
    dataFile.print(TEMP_MS5803);
    dataFile.print(",");
    dataFile.println(messagesSent);
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
   function: smartDelay
   usage: smartdelay(unsigned long ms);
   @param: ms = time to delay for
   ---------------------------------
   This function sleeps while also reading GPS data.
*/
static void smartDelay(unsigned long ms){
  unsigned long start = millis();
  do{
    while(Serial1.available()){
      gps.encode(Serial1.read());
    }
  }while (millis() - start < ms);
}

/*
   function: getTime
   usage: getTime(void);
   ---------------------------------
   This function formats the time string. 
*/
static String getTime(void){
  String timer = "";
  DateTime now = rtc.now();
  timer+= now.hour();
  timer+= printDigits(now.minute());
  timer+= printDigits(now.second());
  timer+= '/';
  timer+= now.year();
  timer+= '/';
  timer+= now.month();
  timer+= '/';
  timer+= now.day();
  return timer;
}

/*
   function: printDigits
   usage: printDigits(int digits);
   ---------------------------------
   This function formats digits for the time.
*/
static String printDigits(int digits){
  String timer = "";
  timer += ":";
  if(digits < 10){
    timer += '0';
  }
  timer += digits;
  return timer;
}

/*
   function: calcAscentRate(void)
   usage: calcAscentRate();
   return: ascent rate from last two data points
   ---------------------------------
   This function calculates the linearized ascent rate from 
   the last two altitude readings.
*/
static float calcAscentRate(void){
  for(int i = 0; i < AscentRateSize - 1; i++){
    AscentRateBuffer[i] = AscentRateBuffer[i + 1];
  }
  lastALTCall = currALTCall;
  currALTCall = millis();
  AscentRateBuffer[AscentRateSize -1] = (ALTITUDE_BMP - ALTITUDE_LAST) / ((currALTCall - lastALTCall) / 1000.0);
  float AscentRateTotal = 0;
  for(int i = 0; i < AscentRateSize; i++){
    AscentRateTotal += AscentRateBuffer[i];
  }
  return AscentRateTotal / AscentRateSize;
}

/*
   function: satelliteTransmission
   usage: satelliteTransmission();
   ---------------------------------
   This function sends messages through the Rock Block. 
*/
void satelliteTransmission(){
  if((minutes < commBeaconInterval)){
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
    messageToSend += messagesSent;
    
    for(uint8_t i = 0; i < messageToSend.length(); i++) {
      outBuffer[i] = messageToSend[i];
    }
    uint8_t rxBuffer[200] = {0};
    for(int i = 0; i < messageToSend.length(); i++){
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
void set_GPS_flight_mode(){
  for (int i = 0; i < 6; i++){
    sendUBX(setNav, sizeof(setNav) / sizeof(uint8_t));
    gps_set_sucess = getUBX_ACK(setNav);
    if(gps_set_sucess){
      break;
    }
  }
  gps_set_sucess = 0;
}

/*
   function: sendUBX
   usage: sendUBX(uint8_t *MSG, uint8_t len);
   param: MSG = message
   param: len = length
   ---------------------------------
   This function sends end a byte array of UBX protocol to the GPS
   so that it doesn't lock it out at altitude.
*/
void sendUBX(uint8_t *MSG, uint8_t len){
  for(int i = 0; i < len; i++){
    HWSERIAL.write(MSG[i]);
  }
  HWSERIAL.println();
}

/*
   function: getUBX_ACK
   usage: getUBX_ACK(uint8_t *MSG);
   param: MSG = message
   return: success = checks message
   ---------------------------------
   This function calculates the expected UBX ACK 
   packet and parse UBX response from GPS.
*/
boolean getUBX_ACK(uint8_t *MSG){
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  ackPacket[0] = 0xB5; 
  ackPacket[1] = 0x62;  
  ackPacket[2] = 0x05;  
  ackPacket[3] = 0x01;  
  ackPacket[4] = 0x02; 
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];
  ackPacket[7] = MSG[3];  
  ackPacket[8] = 0;
  ackPacket[9] = 0;   
  for (uint8_t i = 2; i > 8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
  while(1){
    if(ackByteID > 9){
      // All packets in order!
      printToSerialAndLog("(SUCCESS!)");
      return true;
    }
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) {
      printToSerialAndLog("(FAILED!)");
      return false;
    }
    // Make sure data is available to read
    if(HWSERIAL.available()){
      b = HWSERIAL.read();
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) {
        ackByteID++;
        //Serial.print(b, HEX);
      }
      else{
        ackByteID = 0;  // Reset and look again, invalid order
      }
    }
  }
}
