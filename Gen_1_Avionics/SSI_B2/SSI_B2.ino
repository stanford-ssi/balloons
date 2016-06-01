/*
   Stanford Student Space Initiative
   Balloons Avionics Launch 2 | April 2016
   Davy Ragland   | dragland@stanford.edu
   Kirill Safin   | ksafin@stanford.edu
   Aria Tedjarati | atedjarati@stanford.edu
*/

/*********************************************************************
                            SETUP
*********************************************************************/
//Teensy physical pinout
uint8_t BLINK_PIN        =   0;
uint8_t I_GOOD_PIN       =   1;
uint8_t T_GOOD_PIN       =   2;
uint8_t P_GOOD_PIN       =   3;
uint8_t RB_GOOD_PIN      =   4;
uint8_t GPS_GOOD_PIN     =   5;
uint8_t SD_GOOD_PIN      =   6;
uint8_t BAT_GOOD_PIN     =   7;
uint8_t SD_CARD_PIN_ON   =   2;
uint8_t ROCKBLOCK_S_PIN  =   9;
uint8_t SD_CARD_PIN_CS   =  10;
uint8_t Voltage_PIN      =  14;
uint8_t Current_PIN      =  15;
uint8_t BMP280_PIN_CS    =  20;
uint8_t Thermo_PIN_CS    =  22;
uint8_t CUTTDOWN_PIN     =   9;
uint8_t HEATER_PIN       =  10;

//Import libraries
#include <SD.h>
#include <Wire.h>
#include <SPI.h>
#include <TinyGPS.h>
#include <MS5803.h>
#include <IridiumSBD.h>
#include <RTClib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MAX31855.h>
#include <Adafruit_MCP23017.h>

//Global objects
#define HWSERIAL Serial1
File dataFile;
TinyGPS gps;
RTC_DS1307 rtc;
MS5803 MS_803 = MS5803();
IridiumSBD isbd(Serial3, ROCKBLOCK_S_PIN);
Adafruit_BMP280 bme(BMP280_PIN_CS);
Adafruit_MAX31855 thermocouple(Thermo_PIN_CS);
Adafruit_MCP23017 mcp;
String HEADER = "Stanford Student Space Initiative Balloons Avionics Launch 2\nVOLTAGE,ALTITUDE_GPS,ALTITUDE_BMP,TEMP_IN,TEMP_OUT,LAT,LONG,MESSAGESSENT,SPEED_GPS,PRESS_BMP,CURRENT,PRESS_MS5803,TEMP_MS5803,TIMER,BLINK";

static void readData(void);
static String parseData(int COMMS);
static void   logData(String currentValues);
static void   writeLEDS(void);
static void   smartDelay(unsigned long ms);
static void   setPinMode(void);
static String getTime(void);
static String printDigits(int digits);
void satelliteTransmission();
bool ISBDCallback();
/*********************************************************************
                             DATA
 *********************************************************************/
float commBeaconInterval   = 0.05;
float COMM_BEACON_INTERVAL = 2.0;  
double minutes             = 0.0;
float loopStartTime        = 0.0;
float elapsedSeconds       = 0.0;
float overflowSeconds      = 0.0;
size_t bufferSize          = 0;
byte gps_set_sucess = 0;
uint8_t setNav[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
  0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC
};

double  VOLTAGE      = 0;
double  ALTITUDE_GPS = 0;
double  ALTITUDE_BMP = 0;
double  TEMP_IN      = 0;
double  TEMP_OUT     = 0;
float   LAT          = 0;
float   LONG         = 0;
int messagesSent     = 0;
double  SPEED_GPS    = 0;
double  PRESS_BMP    = 0;   
double  CURRENT      = 0;
double  PRESS_MS5803 = 0;
double  TEMP_MS5803  = 0;
String  TIMER        = "";
double  BLINK        = 0;
/*********************************************************************
                             BOOT
 *********************************************************************/
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  delay(500);
  if(!SD.begin(SD_CARD_PIN_CS)){Serial.println("Could not find a valid SD Card, check wiring!");return;}
  logData(HEADER);
  if(!bme.begin()){Serial.println("Could not find a valid BMP280 sensor, check wiring!");}
  if(!MS_803.initalizeSensor()){Serial.println("Could not find a valid MS5803 sensor, check wiring!");} 
  if(!rtc.begin()){Serial.println("Could not find a valid RTC, check wiring!");}
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  mcp.begin();
  setPinMode();
  isbd.attachConsole(Serial);
  isbd.attachDiags(Serial);
  isbd.setPowerProfile(1);  
  set_flight_mode();       
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
  String currentValues = parseData(0);
  logData(currentValues);
  satelliteTransmission();
  elapsedSeconds = (float)(((float)millis() - loopStartTime) / 1000.0);
  overflowSeconds = (float)(((float)millis() - loopStartTime) / 1000.0) - elapsedSeconds;
  minutes += (float)(((float)millis() - loopStartTime) / 1000.0 / 60.0);
  smartDelay(100);
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
  unsigned long age;
  gps.f_get_position(&LAT, &LONG, &age);
  MS_803.readSensor();
  VOLTAGE      = (analogRead(Voltage_PIN) / 310.0) * 4;
  ALTITUDE_GPS = gps.f_altitude();
  ALTITUDE_BMP = bme.readAltitude(1013.25);
  if(ALTITUDE_BMP > 15240){
    mcp.digitalWrite(CUTTDOWN_PIN, HIGH);
    delay(30000);
    mcp.digitalWrite(CUTTDOWN_PIN, LOW);
  }
  TEMP_IN      = bme.readTemperature();
  if(!isnan(thermocouple.readCelsius())){TEMP_OUT =  thermocouple.readCelsius();}
  SPEED_GPS    = gps.f_speed_kmph();
  PRESS_BMP    = bme.readPressure();   
  CURRENT      = (analogRead(Current_PIN) / 310.0) * 0.2;
  PRESS_MS5803 = MS_803.pressure() * 10;
  TEMP_MS5803  = MS_803.temperature();
  TIMER        = getTime();
  BLINK = !BLINK;
}

/*
   function: parseData
   usage: parseData();
   param: COMMS = cuttof for rockblock
   return: formatted CSV string
   ---------------------------------
   This function returns the data as a formated string.
*/
static String parseData(int COMMS){
  String dataString = "";
  dataString += VOLTAGE;
  dataString += ",";
  dataString += ALTITUDE_GPS;
  dataString += ",";
  dataString += ALTITUDE_BMP;
  dataString += ",";
  dataString += TEMP_IN;
  dataString += ",";
  dataString += TEMP_OUT;
  dataString += ",";
  dataString += String(LAT, 4);
  dataString += ",";
  dataString += String(LONG, 4);
  dataString += ",";
  dataString += messagesSent;
  if(COMMS){return dataString;}
  dataString += ",";
  dataString += SPEED_GPS;
  dataString += ",";
  dataString += PRESS_BMP; 
  dataString += ",";
  dataString += CURRENT;
  dataString += ",";
  dataString += PRESS_MS5803;
  dataString += ",";
  dataString += TEMP_MS5803;
  dataString += ",";
  dataString += TIMER;
  dataString += ",";
  dataString += BLINK;
  return dataString;
}
/*********************************************************************
                           HELPERS
 *********************************************************************/
/*
   function: logData
   usage: logData(String currentValues);
   @param: currentValues = data string to log
   ---------------------------------
   This function logs data to the 3D card.
*/
static void logData(String currentValues) {
  dataFile = SD.open("SSI-B2.txt", FILE_WRITE);
  if(dataFile){
    dataFile.println(currentValues);
    dataFile.close();
    Serial.println(currentValues);
  }
  else {Serial.println("error opening \"SSI-B2.txt\"");}
}

/*
   function: writeLEDS
   usage: writeLEDS(void);
   ---------------------------------
   This function visualy error checks the system.
*/
static void writeLEDS(void){
  mcp.digitalWrite(BLINK_PIN,    LOW);
  mcp.digitalWrite(I_GOOD_PIN,   LOW);
  mcp.digitalWrite(T_GOOD_PIN,   LOW);
  mcp.digitalWrite(P_GOOD_PIN,   LOW);
  mcp.digitalWrite(RB_GOOD_PIN,  LOW);
  mcp.digitalWrite(GPS_GOOD_PIN, LOW);
  mcp.digitalWrite(SD_GOOD_PIN,  LOW);
  mcp.digitalWrite(BAT_GOOD_PIN, LOW);
  if(BLINK){mcp.digitalWrite(BLINK_PIN, HIGH);}
  if(CURRENT <= 0.5){mcp.digitalWrite(I_GOOD_PIN, HIGH);}
  if(TEMP_IN > 15 && TEMP_IN < 50){mcp.digitalWrite(T_GOOD_PIN, HIGH);}
  if(ALTITUDE_BMP > -20 && ALTITUDE_BMP < 200){mcp.digitalWrite(P_GOOD_PIN, HIGH);}
  if(1){mcp.digitalWrite(RB_GOOD_PIN, HIGH);}
  if(LAT != 1000.0 && LONG != 1000.0){mcp.digitalWrite(GPS_GOOD_PIN, HIGH);}
  if(digitalRead(SD_CARD_PIN_ON) == 0){mcp.digitalWrite(SD_GOOD_PIN, HIGH);}
  if(VOLTAGE >= 3.63){mcp.digitalWrite(BAT_GOOD_PIN,HIGH);}
}

/*
   function: smartDelay
   usage: smartdelay(unsigned long ms);
   @param: ms = time to delay for
   ---------------------------------
   This function sleeps while also reading GPS data.
*/
static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do{
    while (Serial1.available()){gps.encode(Serial1.read());}
  }while (millis() - start < ms);
}

/*
   function: setPinMode
   usage: setPinMode(void);
   ---------------------------------
   This function initializes the pinmodes. 
*/
static void setPinMode(void){
  mcp.pinMode(BLINK_PIN,    OUTPUT);
  mcp.pinMode(I_GOOD_PIN,   OUTPUT);
  mcp.pinMode(T_GOOD_PIN,   OUTPUT);
  mcp.pinMode(P_GOOD_PIN,   OUTPUT);
  mcp.pinMode(RB_GOOD_PIN,  OUTPUT);
  mcp.pinMode(GPS_GOOD_PIN, OUTPUT);
  mcp.pinMode(SD_GOOD_PIN,  OUTPUT);
  mcp.pinMode(BAT_GOOD_PIN, OUTPUT);
  for(int i = 8; i < 13; i++){
    mcp.pinMode(i, OUTPUT);
    mcp.digitalWrite(i, LOW);
  }
  pinMode(SD_CARD_PIN_ON,   OUTPUT);
  pinMode(Voltage_PIN,      INPUT);
  pinMode(Current_PIN,      INPUT);
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
   This function formats digits for the time 
*/
static String printDigits(int digits){
  String timer = "";
  timer += ":";
  if(digits < 10)
    timer += '0';
  timer += digits;
  return timer;
}

/*
   function: satelliteTransmission
   usage: satelliteTransmission();
   ---------------------------------
   This function sends messages through the Rock Block. 
*/
void satelliteTransmission(){
  if((minutes < commBeaconInterval)){
    Serial.println("Beginning to talk to the RockBLOCK...");
    messagesSent += 1;
    Serial.print("Transmitting message: ");
    Serial.println((String)messagesSent);
    delay(200);
    Serial.println("Beginning to talk to the RockBLOCK...");
    Serial.println("Sending RB message");
    char outBuffer[200];
    String messageToSend = parseData(1); 
    for(uint8_t i = 0; i < messageToSend.length(); i++) {outBuffer[i] = messageToSend[i];}
    uint8_t rxBuffer[200] = {0};
    for(int i = 0; i < messageToSend.length(); i++){rxBuffer[i] = outBuffer[i];}
    bufferSize = sizeof(rxBuffer);
    isbd.sendReceiveSBDBinary(rxBuffer, messageToSend.length(), rxBuffer, bufferSize);
    int number = bufferSize;
    Serial.println("Going to sleep mode");
    delay(2000);
    commBeaconInterval = minutes + COMM_BEACON_INTERVAL;
  }
}

/*
   function: ISBDCallback
   usage: ISBDCallback();
   ---------------------------------
   This function loops back into the main 
   call while waiting for responces.
*/
bool ISBDCallback(){
  loopStartTime = millis();
  readData();
  writeLEDS();
  String currentValues = parseData(0);
  String SatCommValues = parseData(1);
  logData(currentValues);
  elapsedSeconds = (float)(((float)millis() - loopStartTime) / 1000.0);
  overflowSeconds = (float)(((float)millis() - loopStartTime) / 1000.0) - elapsedSeconds;
  minutes += (float)(((float)millis() - loopStartTime) / 1000.0 / 60.0);
  return true;
}

/*
   function: set_flight_mode
   usage: set_flight_mode();
   ---------------------------------
   This function sets the GPS so that it won't lock out. 
*/
void set_flight_mode() {
  for (int i = 0; i < 6; i++) {
    sendUBX(setNav, sizeof(setNav) / sizeof(uint8_t));
    gps_set_sucess = getUBX_ACK(setNav);
    if (gps_set_sucess) break;
  }
  gps_set_sucess = 0;
}

/*
   function: sendUBX
   usage: sendUBX(uint8_t *MSG, uint8_t len);
   param: MSG = message
   param: len = length
   ---------------------------------
   This function sets the GPS so that it won't lock out. 
*/
// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for (int i = 0; i < len; i++) {
    HWSERIAL.write(MSG[i]);
  }
  HWSERIAL.println();
}

/*
   function: getUBX_ACK
   usage: getUBX_ACK(uint8_t *MSG);
   param: MSG = message
   ---------------------------------
   This function communicates with the ROCKBLOCK
*/
// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
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
  while (1) {
    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      Serial.println("(SUCCESS!)");
      return true;
    }
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) {
      Serial.println("(FAILED!)");
      return false;
    }
    // Make sure data is available to read
    if (HWSERIAL.available()) {
      b = HWSERIAL.read();
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) {
        ackByteID++;
        //Serial.print(b, HEX);
      }
      else {
        ackByteID = 0;  // Reset and look again, invalid order
      }
    }
  }
}
