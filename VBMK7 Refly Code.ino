
/********************************************************
Stanford SSI High Altitude Balloon Group Arduino Code for AVIONICS GEN Mk 7
- This code is for the main avionics board.

Written by Aria Tedjarati and Joan Creus-Costa Winter 2017

**********************************/

/* As you set out for Ithaka
 * hope your road is a long one, */

#include <PID_v1.h>
#include <SD.h>
#include <math.h>
#include <TinyGPS++.h>
#include <stdlib.h>
#include <stdio.h>
#include <EEPROM.h>
#include <SPI.h>
#include <i2c_t3.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <IridiumSBD.h>
#include <LTC2991.h>
#include <SoftwareSerial.h>

// debug thingy to print bytes for me
#define bytestring "%c%c%c%c%c%c%c%c"
#define bytefunction(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/* full of adventure, full of discovery.
 * Laistrygonians, Cyclops,
 * angry Poseidon—don’t be afraid of them: */

// PINS LAYOUT
const int RB_SLEEP =                           14;  // RockBLOCK sleep pin
const int CHIP_SELECT =                        10;  // SD Card Chip Select Pin
const int BALLAST_REVERSE =                    5;   // Ballast Reverse Pin
const int BALLAST_FORWARD =                    21;   // Ballast Forward Pin
const int VALVE_REVERSE =                      6;
const int VALVE_FORWARD =                      20;
const int HEATER_INTERNAL_STRONG =             4;  // Heater PWM for internal payload
const int HEATER_INTERNAL_WEAK =               3;
const int V_BATT =                             A14; // Analog pin for reading voltage of battery
const int LED_PIN =                            26;
const int BMP_CS_ONE =                         9;
const int BMP_CS_TWO =                         15;
const int BMP_CS_THREE =                       25;
const int BMP_CS_FOUR =                        32;
const int GPS_ENABLE =                         17;
const int BATT_CURRENT =                       A10;
const int EXTERNAL_CURRENT =                   A11;
const int VALVE_POT =                          A2;
const int RB_GATE =                            28;
const int PAYLOAD_GATE =                       31;
const int FEMTO_RX =                           23;
const int FEMTO_TX =                           22;


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/* you’ll never find things like that on your way */

// Constants and variables that can be changed:
                                                               //LAT   LONG   TEMP     ALT     PRESSURE   CURRENT
const float     ERROR_MAX_VALUES                          [] = {  90.0,  180.0,  80.0,  40000.0,  125000,    4.0};
const float     ERROR_MIN_VALUES                          [] = { -90.0, -180.0, -80.0, -5000.0,   1,         0.0};
float           reArmConst =                              0.0;
float           I_thresh =                                0.75;
boolean         strongHeaterOn =                          true;           // Boolean to turn on strong heater
boolean         weakHeaterOn =                            false;          // Boolean to turn on weak heater (default heater is strong heater)
double          INTERNAL_TEMP_SETPOINT =                  0.0;            // Temperature Setpoint in degrees celsius for Internal Payload
const int       ROCKBLOCK_BAUD =                          19200;          // Rockblock TRANSMISSION BAUD rate
const int       GPS_BAUD =                                9600;           // uBLOX GPS BAUD rate
const int       CONSOLE_BAUD =                            115200;         // Console BAUD rate
float           SPEED_CONSTANT =                          1.0;            // Constant of proportionality for our vertical velocity for valve incentive calculation
float           ALTITUDE_DIFFERENCE_CONSTANT =            1.0 / 1000.0;   // Constant of proportionality for our altitude difference from setpoint for valve incentive calculation
float           LAST_VALVE_OPENING_CONSTANT =             1.0 / 1000.0;   // Constant of proportionality for our last valve opening for valve incentive calculation
float           BALLASTER_SPEED_CONSTANT =                1.0;            // Constant of proportionality for our vertical velocity for ballast incentive calculation
float           BALLASTER_ALT_DIFF_CONSTANT =             1.0 / 1000.0;   // Constant of proportionality for our altitude difference from setpoint for ballast incentive calculation
float           BALLASTER_LAST_DROP_CONSTANT =            1.0 / 1000.0;   // Constant of proportionality for our last ballast dropping for ballast incentive calculation
float           ALTITUDE_SETPOINT =                       13500.0;        // Altitude setpoint for our valve system 16000
float           BALLAST_ALTITUDE_SETPOINT =               13000.0;        // Altitude setpoint for our ballast system 15000 SHOULD BE 15000.00000
float           ASCENT_RATE_BUFFER_SIZE =                 1000.0;         // Size of the ascent rate buffer
float           PWM_HEATER_BUFFER_SIZE =                  50.0;           // Size of the PWM heater buffer
const float     GPS_MAX_VELOCITY =                        0.100;          // Max GPS velocity for error handling purposes in latitude/second
const float     GPS_NOISE_STANDARD_DEVIATION =            0.00015;        // Calculated gps standard deviation in latitude from previous logfiles
const float     BMP_ALTITUDE_NOISE_STANDARD_DEVIATION =   60;             // Calculated altitude standard deviation in meters from previous logfiles
const float     BMP_ALTITUDE_MAX_VELOCITY =               75.0;           // Max Altitude Vertical velocity for error handling purposes in meters/second
float           commBeaconInterval =                      0.20;           // Time for first satellite communication to be sent
float           COMM_BEACON_INTERVAL =                    2.0;            // Time between satellite transmissions in minutes; default is 4
float           gpsBeaconInterval =                       0.05;           // Time for first GPS lock to be acquired
float           GPS_BEACON_INTERVAL =                     0.2;            // Time between GPS lock acquisitions
int             GPS_ACQUISITION_TIME_2 =                  500;             // Time in milliseconds for GPS 2 acquisition time
float           altitudeBuffer1                           [1000] = {0};    // Buffer of differentials in altitude
float           altitudeBuffer2                           [1000] = {0};    // Buffer of differentials in altitude
float           altitudeBuffer3                           [1000] = {0};    // Buffer of differentials in altitude
float           altitudeBuffer4                           [1000] = {0};    // Buffer of differentials in altitude
float           altitudeBuffer5                           [1000] = {0};    // Buffer of differentials in altitude
double          timeBuffer                                [1000] = {0};    // Size of the time buffer for calculating ascent rate
float           mainHeaterBuffer                          [50] = {0};     // Size of the main Heater buffer for calculating pwm
float           velocityBuffer                            [400] = {0};    // Velocity buffer
int             VEL_BUFFER_SIZE =                         400;
int             velocityCounter =                         0;
float           currentMax =                              0;              // Max current in comms window
float           currentMin =                              10000;             // Min current in comms window
float           currentAvg =                              0;              // Avg current in comms window (without denominator)
int             currentNum =                              0;              // Number of current readings in window
int             currentValNum =                           0;              // Number of current readings in window
int             currentBalNum =                           0;              // Number of current readings in window
float           currentGPSMax =                           0;
float           currentGPSAvg =                           0;
float           currentRBMax  =                           0;
float           currentRBAvg  =                           0;
float           currentValMax =                           0;
float           currentValAvg =                           0;
float           currentBalMax =                           0;
float           currentBalAvg =                           0;
float           altitudeSinceLastVent =                   0.0;            // Altitude since last venting of the valve system
float           altitudeSinceLastDrop =                   -90000.0;       // Altitude since last ballast drop - -90,000
float           ALTITUDE_SINCE_LAST_DROP_FILLER =         13000.0;        // Altitude since last ballast drop
float           lastLati =                                37.561699;      // Latitude of launch location - 37.561699 37.427463
float           lastLongi =                               -121.150567;    // Longitude of launch location - -121.150567 122.170595
long            MAX_VALVE_ON_MILLIS =                     20000;          // Milliseconds for venting
long            MAX_BALLAST_ON_MILLIS =                   20000;          // Milliseconds for ballast dropping
float           ALTITUDE_CHANGE_BALLAST =                 13000;          // Altitude where we change the altitudeSinceLastVent
int             OPEN_VALVE_VALUE =                        2700;           // Servo position for closed valve 140
int             CLOSED_VALVE_VALUE =                      2250;           // Servo position for open valve 63
const int       CUTDOWN_VALVE_VALUE =                     3000;           // Servo position for cutdown 10
const int       VALVE_TIMEOUT =                           2500;           // Valve Timeout
const int       VALVE_CUTDOWN_TIMEOUT =                   10000;          // Valve Cutdown Timeout
const int       MAX_TIME_WITHOUT_ENCODER =                20000;          // Max time in milliseconds without encoder giving us proper command
const int       MAX_TEMP_DIFF =                           15;             // Max temperature difference between sensors
int             BALLAST_MOTOR_SPEED =                     255;            // Speed that we run the ballast motor at
int             VALVE_MOTOR_SPEED =                       255;            // Valve Motor Speed
double          batteryCapacity =                         172800.0;       // Initial battery capacity in mWh
double          initialBatteryCapacity =                  172800.0;       // Initial battery capacity in mWh
long            IridiumVentTime =                         20000;          // Initial time to vent
long            IridiumBallastTime =                      20000;          // Initial time to drop ballast for
double          RBRestartTimer =                          30.0;           // Minutes to wait before restarting rockblock if it decides to turn off
double          RB_RESTART_TIMER =                        30.0;
double          VALVE_OPEN_BACKUP_TIMER =                 1000;
double         neckTemperatureAverage =                   0.0;
double         neckTemperature =                          0.0;


float          sensor1max =                               -40000;
float          sensor2max =                               -40000;
float          sensor3max =                               -40000;
float          sensor4max =                               -40000;
float          sensor1min =                               40000;
float          sensor2min =                               40000;
float          sensor3min =                               40000;
float          sensor4min =                               40000;
int            sensor1rej =                               0;
int            sensor2rej =                               0;
int            sensor3rej =                               0;
int            sensor4rej =                               0;
bool           forceValve =                               false;
bool           forceBallast =                             false;
float          forceValveTime =                           0.0;
float          forceBallastTime =                         0.0;
uint16_t       DO_NOTHING_TIMER =                         30;        // # of seconds to wait before checking if valve is closed always
long           doNothingTimer =                           30000;     // initialize in milliseconds
double         internalSetpoint, internalOutput;                     // Define Variables we'll be connecting to for PID Internal
double         internalTemperature =                      33.0;      // Temperature of Internal compartment in Celsius
float          pressure_baseline =                        0.0;       // Launch Location Pressure
float          rawDAltOne =                               0.0;       // Raw Altitude Reading of First Pressure Sensor in meters
float          rawDAltTwo =                               0.0;       // Raw Altitude Reading of Second Pressure Sensor in meters
float          rawDAltThree =                             0.0;       // Raw Altitude Reading of Third Pressure Sensor in meters
float          rawDAltFour =                              0.0;       // Raw Altitude Reading of Forth Pressure Sensor in meters
float          rawDAltFive =                              0.0;
float          lastDAltOne =                              0.0;       // Raw Altitude Reading of First Pressure Sensor in meters
float          lastDAltTwo =                              0.0;       // Raw Altitude Reading of Second Pressure Sensor in meters
float          lastDAltThree =                            0.0;       // Raw Altitude Reading of Third Pressure Sensor in meters
float          lastDAltFour =                             0.0;       // Raw Altitude Reading of Forth Pressure Sensor in meters
float          lastDAltFive =                             0.0;
float          DAltOne =                                  0.0;       // Raw Altitude Reading of First Pressure Sensor in meters
float          DAltTwo =                                  0.0;       // Raw Altitude Reading of Second Pressure Sensor in meters
float          DAltThree =                                0.0;       // Raw Altitude Reading of Third Pressure Sensor in meters
float          DAltFour =                                 0.0;       // Raw Altitude Reading of Forth Pressure Sensor in meters
float          DAltFive =                                 0.0;
double         lastSensorOneSeconds =                    -7200.0;    // time we last accepted a sensor 1 reading
double         lastSensorTwoSeconds =                    -7200.0;    // time we last accepted a sensor 2 reading
double         lastSensorThreeSeconds =                  -7200.0;    // time we last accepted a sensor 3 reading
double         lastSensorFourSeconds =                   -7200.0;    // time we last accepted a sensor 4 reading
double         lastSensorFiveSeconds =                   -7200.0;
float          DAlt =                                     0.0;       // Current Altitude in meters
float          lastDAlt =                                 0.0;       // Previous Altitude in meters
float          DAltErrOne =                               0.0;       // Altitude Error of First Pressure Sensor
float          DAltErrTwo =                               0.0;       // Altitude Error of Second Pressure Sensor
float          DAltErrThree =                             0.0;       // Altitude Error of Third Pressure Sensor
float          DAltErrFour =                              0.0;       // Altitude Error of Fourth Pressure Sensor
float          DAltErrFive =                              0.0;
double         minutes =                                  0.0;       // Total time in minutes since startup
int            messagesSent =                             0;         // Number of messages sent by the RockBLOCK to us
int            messageReceived =                          0;         // Number of messages received by the RockBLOCK from us
float          averageAscentRate =                        0.0;       // Average ascent rate of our balloon in m/s
float          averageAscentRate1 =                       0.0;       // Average ascent rate of our balloon in m/s
float          averageAscentRate2 =                       0.0;       // Average ascent rate of our balloon in m/s
float          averageAscentRate3 =                       0.0;       // Average ascent rate of our balloon in m/s
float          averageAscentRate4 =                       0.0;       // Average ascent rate of our balloon in m/s
float          averageAscentRate5 =                       0.0;
float          tempAscentRate =                           0.0;       // Average ascent rate of our balloon in m/s
float          mainDutyCycle =                            0.0;
int            mainDutyCycleInt =                         0;
double         batteryVoltage =                           0.0;       // Voltage of the battery
double         currentMonitor =                           0.0;       // Current flowing out of battery
double         externalCurrentMonitor =                   0.0;
float          alt =                                      0.0;       // GPS Altitude (in meters)
float          lati =                                     lastLati;  // Actual useful latitude after error handling
float          longi =                                    lastLongi; // Actual useful longitude after error handling
float          gpsLatErr =                                0.0;       // GPS Latitude Error
float          gpsLongErr =                               0.0;       // GPS Longitude Error
float          rawGpsLat =                                0.0;       // Raw GPS Latitude
float          rawGpsLong =                               0.0;       // Raw GPS Longitude
double         elapsedSeconds =                           1.0;       // Elapsed Seconds
double         elapsedSecondsAltitudeErrorHandler =       1.0;       // Altitude Error Handler Timer
double         elapsedSecondsGPSErrorHandler2 =           100000.0;  // GPS Error Handler Timer. Used to be 1.0, changed to a big number so that we accept the first value
float          temperatureCount =                         0;         // #Times we ping temperature
float          internalTemperatureAverage =               0;         // Average Temperature Sent to RockBLOCK
float          valveIncentive =                           0.0;       // Valve Incentive to Vent
float          ballastIncentive =                         0.0;       // Ballast Incentive to Drop
int            ascentRateCounter =                        0;         // Iteration through ascent rate buffer
int            heaterPWMCounter =                         0;
int            isValveOpen =                              2;         // 0 for valve not open AND not closed = fucked, 1 for valve open, 2 for valve closed
int            ventCount =                                0;         // Number of total vents
int            ballastCount =                             0;         // Number of total ballast drops
double         cutdownReason =                            0.0;       // 1 for Satcomm cutdown, 2 for gps fence, 3 for altitude, 4 for failure to cutdown
boolean        bufferFull =                               false;     // Boolean for state of ascent rate buffer
boolean        heaterBufferFull =                         false;
boolean        ballastState =                             false;     // False = not on, True = on, for ballast state
double         overflowSeconds =                          0.0;       // Overflow seconds
boolean        firstBallastDrop =                         false;     // State of first ballast drop
float          encoderTimer =                             0;         // Encoder timer, resets to 0 when we read high from encoder
int            ballastDirection =                         2;         // 1 --> Forward // 2 --> Reverse
unsigned long  age1 =                                     0;         // Age of GPS lat/long
boolean        LED_STATE =                                true;
int            activeSensors =                            13;        // = 1111_2
int            returnValuee[] =                           {99, 99, 99, 99, 99, 99, 99, 99, 99};
int            powerStates[] =                            {0,    0,    0};//,    3};         //RB   GPS   HEATER  FEMTOLOON    // 0 = reset, 1 = trying to turn on, 2 = successfully turned on, 3 = skip.
const String   CUTDOWN_COMMAND =                          "Little Timmy Deserved It";
const int      CUTDOWN_INDEX =                            99;
bool           includeOne,
               includeTwo,
               includeThree,
               includeFour,
               includeFive,
               rawIncludeFive;
long           ventTime =                                 0;
long           ballastTime =                              0;
float          mins[150];
float          maxs[150];
float*         vars[150];
int            bits[150];
int            argc =                                     0;
int            debugThreshold =                           1000;
bool           reportMode =                               true;     // true for extended, false for reduced
float          deltaIncentive =                           0.0;      // increase in incentive due to fucked sensors
float          curcurbuf[100] =                           {0};
int            curcurnum =                                0;
bool           LEDon =                                    true;
float          RBjoules =                                 0.0;
float          RBheatJ =                                  0.0;
size_t         bufferSize =                               0;
boolean        inSetup =                                  true;
double         loopStartTime =                            0.0;
/******/
bool           controlMode =                              false;     // true for auto, false for manual
boolean        forceValveTimedMode =                      true;
bool           doReadback =                               false;
/******/

/* as long as you keep your thoughts raised high, */

float RBmsg, RBvel, RBaltB, RBaltG, RBlat, RBlong, RBtemp, RBbat, RBvoltP, RBavgI, RBmaxI, RBminI,
      RBVin, RBBin, RBcutdown, RBBalls, RBBtime, RBVtime, RBheat, RBs1delta, RBs2delta, RBs3delta, RBs4delta,
      RBs1max, RBs2max, RBs3max, RBs4max, RBs1min, RBs2min, RBs3min, RBs4min, RBs1rej, RBs2rej, RBs3rej, RBs4rej,
      RBatt1type, RBatt1time, RBatt2type, RBatt2time, RBatt3type, RBatt3time, RBatt4type, RBatt4time, RBVatt, RBBatt, RBt0, RBmode,
      RBheightOffset, RBheightScale, RBtimeFactor, RBtimeInterval, RBsensors, RBLEDon,
      RBpowerHeaters, RBpowerGPS, RBpowerRB, RBpotReading, RBreadback, RBaltDiff, RBmaxTime,
      RBcurrentGPSmax, RBcurrentGPSavg, RBcurrentRBmax, RBcurrentRBavg, RBcurrentValmax, RBcurrentValavg, RBcurrentBalmax, RBcurrentBalavg, 
      RBCValveSpeed, RBCValveAltDiff, RBCValveLast, RBCBallastSpeed, RBCBallastAltDiff, RBCBallastLast, RBCValveSetpoint, RBCBallastSetpoint, RBFValveLast, RBFDropLast, RBCOpenValve, RBCClosedValve, RBCCutdownValve,
      RBCMaxEncoder, RBCDoNothing, RBCValTime, RBCBalTime, RBControlMode, RBReportMode, RBPotMode, RBTemperatureSetpoint, RBRBinterval, RBGPSinterval, RBValveSpeed, RBOpeningTime,
      RBStrongHeat, RBMildHeat, RBIthresh, RBreArmConst, RBNeckTemp;

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/* as long as a rare excitement */

const int NUM_KALMANS = 200;
float kalmanAltitudes[NUM_KALMANS] = {};
int currentAltitude = 93;
double Qdt = 4.0;
double QcurrentDt = Qdt;
int QmulFactor = 1;
double initAltitudeTime = 10;
int Qbit = 7;
float Qmat[16] = {8, 16, 16, 32, 32, 32, 32, 32, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000};

const char *DCcoeffs[] = {"11100000", "11100001", "1110001", "111001", "11101", "110", "10", "0", "1111"};

const char *ACcoeffs[] = {"00101101101111100", "00101101101111110", "00101101110000010", "00101101110001000", "00101101110001110", "00101101110010100", "00101101110011011", "00101101110100010", "00100", "101110", "101111", "11100", "1010", "1111", "101101", "0010100", "01", "0011", "101100", "0010111", "00101100", "00101101100", "00101101101100", "00101101110100011", "00101101110101010", "00101101110110001", "00101101110111000", "00101101110111111", "00101101111000110", "00101101111001101", "0010110110110100", "00101101101110101", "110", "0010101", "001011011111", "001011011110101", "001011011011011", "00101101110010101", "00101101110011100", "00101101110100100", "00101101110101011", "00101101110110010", "00101101110111001", "00101101111000000", "00101101111000111", "00101101111001110", "0010110110110101", "00101101101110110", "100", "001011010", "00101101110000011", "00101101110001001", "00101101110001111", "00101101110010110", "00101101110011101", "00101101110100101", "00101101110101100", "00101101110110011", "00101101110111010", "00101101111000001", "00101101111001000", "00101101111001111", "00101101101110000", "00101101101110111", "000", "001011011010", "00101101110000100", "00101101110001010", "00101101110010000", "00101101110010111", "00101101110011110", "00101101110100110", "00101101110101101", "00101101110110100", "00101101110111011", "00101101111000010", "00101101111001001", "00101101111010000", "00101101101110001", "00101101101111000", "11101", "00101101101111111", "00101101110000101", "00101101110001011", "00101101110010001", "00101101110011000", "00101101110011111", "00101101110100111", "00101101110101110", "00101101110110101", "00101101110111100", "00101101111000011", "00101101111001010", "00101101111010001", "00101101101110010", "00101101101111001", "00101101111011", "00101101110000000", "00101101110000110", "00101101110001100", "00101101110010010", "00101101110011001", "00101101110100000", "00101101110101000", "00101101110101111", "00101101110110110", "00101101110111101", "00101101111000100", "00101101111001011", "00101101111010010", "00101101101110011", "00101101101111010", "00101101101111101", "00101101110000001", "00101101110000111", "00101101110001101", "00101101110010011", "00101101110011010", "00101101110100001", "00101101110101001", "00101101110110000", "00101101110110111", "00101101110111110", "00101101111000101", "00101101111001100", "00101101111010011", "00101101101110100", "00101101101111011"};

double p11 = 0.25, p12 = 0.25, p21 = 0.25, p22 = 0.25;
double qn = 0.01;
double h = -10000;
double v = 0;

double lastDAltSeconds;

/*** Declaration of objects  **/
Adafruit_BMP280           sensor_1(BMP_CS_ONE);                                                                       // Pressure Sensor 1
Adafruit_BMP280           sensor_2(BMP_CS_TWO);                                                                       // Pressure Sensor 2
Adafruit_BMP280           sensor_3(BMP_CS_THREE);                                                                     // Pressure Sensor 3
Adafruit_BMP280           sensor_4(BMP_CS_FOUR);                                                                      // Pressure Sensor 4
File                      logfile;                                                                                    // SD Card Logfile
PID                       internalPID(&internalTemperature, &internalOutput, &internalSetpoint, 2, 5, 0, DIRECT);     // Internal PID
#define hsGPS             Serial1                                                                                     // Hardware Serial 1 GPS
TinyGPSPlus               tinygps;    
IridiumSBD                isbd(Serial3, RB_SLEEP);
byte                      gps_set_sucess = 0 ;

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void setup() {
  pinMode(BALLAST_FORWARD, OUTPUT);
  pinMode(BALLAST_REVERSE, OUTPUT);                      /* Pinmodes */
  pinMode(VALVE_FORWARD, OUTPUT);
  pinMode(VALVE_REVERSE, OUTPUT);
  pinMode(HEATER_INTERNAL_STRONG, OUTPUT);
  pinMode(HEATER_INTERNAL_WEAK, OUTPUT);
  pinMode(GPS_ENABLE, OUTPUT);
  pinMode(PAYLOAD_GATE,OUTPUT);
  pinMode(VALVE_POT, INPUT);
  pinMode(RB_GATE, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  allPowerOff();
  delay(2000);
  Serial.begin(CONSOLE_BAUD);
  hsGPS.begin(GPS_BAUD);
  isbd.attachConsole(Serial);
  isbd.attachDiags(Serial);
  isbd.setPowerProfile(1);
  Serial3.begin(ROCKBLOCK_BAUD);
  init_log();
  pinMode(CHIP_SELECT, OUTPUT);
  if (!SD.begin(CHIP_SELECT)) {
    Serial.print("Error: Card failed, or not present");
  }
  setupSDCard();                                         /* Setup SD Card */
  printLogfileHeaders();
  analogReference(INTERNAL);                             /* Analog Settings */
  analogReadResolution(12);
  internalSetpoint = INTERNAL_TEMP_SETPOINT;
  internalPID.SetMode(AUTOMATIC);                        /* Initialize PID Heater Settings */
  init_sensors();                                        /* Initialize Sensors */
  readValveFromEEPROMandClear();                         /* Read Valve and Ballast Information from EEPROM */
  readBallastFromEEPROMandClear();
  readPowerStatesFromEEPROMandParse();
  startupGPS();
  startupRockblock();
  startupHeater();
  delay(3000);
  int8_t ack = 0;
  ack |= LTC2991_register_write(LTC2991_I2C_ADDRESS, LTC2991_CHANNEL_ENABLE_REG, LTC2991_ENABLE_ALL_CHANNELS);   //! Enables all channels
  ack |= LTC2991_register_write(LTC2991_I2C_ADDRESS, LTC2991_CONTROL_V1234_REG, 0x00);                           //! Sets registers to default starting values.
  ack |= LTC2991_register_write(LTC2991_I2C_ADDRESS, LTC2991_CONTROL_V5678_REG, 0x00);
  ack |= LTC2991_register_write(LTC2991_I2C_ADDRESS, LTC2991_CONTROL_PWM_Tinternal_REG, LTC2991_REPEAT_MODE);    //! Configures LTC2991 for Repeated Acquisition mode

  setGPSFlightMode();
  
//closeValve();
//valveOpen();
//closeValve();
//cutdownBalloon();
//    for (int i = 0; i < 5; i++) {
//      delay(1000);
//      valveOpen();
//      delay(1000);
//      closeValve();
//    }
//continuouslyDropBallast();
//delay(15000);
//reverseBallastDirection();continuouslyDropBallast();
//delay(15000);
//reverseBallastDirection();continuouslyDropBallast();
//delay(15000);
//reverseBallastDirection();continuouslyDropBallast();
//delay(15000);
//reverseBallastDirection();continuouslyDropBallast();
//delay(15000);
//reverseBallastDirection();continuouslyDropBallast();
//delay(15000);
//reverseBallastDirection();continuouslyDropBallast();
//delay(15000);
//reverseBallastDirection();continuouslyDropBallast();
//valveAndBallasterDoNothing();
//    cutdownBalloon();cutdownBalloon();cutdownBalloon();cutdownBalloon();cutdownBalloon();

  /* For Filling */
  //  while(1);
  //   delay(1000);
  //  closeValve();



  inSetup = false;
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


void loop() {
  loopStartTime = millis();
  iterateSensors();
  if (powerStates[2] == 2) {
    iteratePIDController();
    getHeaterDutyCycles();
  }
  if (powerStates[1] == 2) acquireGPSLock();
  printToSerialAndLog();
  if (powerStates[0] == 2) satelliteTransmission();
  elapsedSeconds = (double)(((double)millis() - loopStartTime) / 1000.0);
  getAscentRates();
  altitudeController();
  blinkLED();  
  checkRB();
  if (minutes > 7200.0) {
     logfile.close();
     setupSDCard();
     printLogfileHeaders();
  }
  int derray = (int) (50.0 - (double)((double)millis() - loopStartTime));
  if (derray > 0) delay(derray);
  overflowSeconds = (double)(((double)millis() - loopStartTime) / 1000.0) - elapsedSeconds;
  minutes += (double)(((double)millis() - loopStartTime) / 1000.0 / 60.0);
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


void setGPSFlightMode(){
  if (powerStates[1] == 2){
    Serial.println("Setting uBlox nav mode: ");
    uint8_t setNav[] = {
      0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 
      0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC };
    while(!gps_set_sucess) {
      sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
      gps_set_sucess=getUBX_ACK(setNav);
    }
    gps_set_sucess=0;
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void checkRB() {
  if ((powerStates[0] == 0 || powerStates[0] == 1 || powerStates[0] == 3) && (minutes >= RBRestartTimer)) {
    powerStates[0] = 1;
    EEPROM.write(10, 1);
    digitalWrite(RB_GATE, HIGH);
    delay(5000);
    Serial.print("isbd begin return"); Serial.println(isbd.begin());
    delay(2000);
    powerStates[0] = 2;
    EEPROM.write(10, 2);
    RBRestartTimer = minutes + RB_RESTART_TIMER;
  }
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


void blinkLED() {
  if (LEDon) {
    int minutess = millis() / 1000.0;
    if (minutess % 2 == 1) {
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, LOW);
    }
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void getAscentRates() {
  if (ascentRateCounter > ASCENT_RATE_BUFFER_SIZE - 1.0) {
    ascentRateCounter = 0;
    bufferFull = true;
  }
  if ((ascentRateCounter <= ASCENT_RATE_BUFFER_SIZE - 1.0)) { //  && (DAlt != lastDAlt)
    int c = ascentRateCounter;
    int prev = c - 1;
    if (prev == -1) {
      prev = ASCENT_RATE_BUFFER_SIZE - 1;
    }
    int pprev = c - 2;
    if (pprev == -1) {
      pprev = ASCENT_RATE_BUFFER_SIZE - 1;
    } else if (pprev == -2) {
      pprev = ASCENT_RATE_BUFFER_SIZE - 2;
    }
    altitudeBuffer1[ascentRateCounter] = includeOne ? rawDAltOne : altitudeBuffer1[prev]; //(2*altitudeBuffer1[prev] - altitudeBuffer1[pprev]);
    altitudeBuffer2[ascentRateCounter] = includeTwo ? rawDAltTwo : altitudeBuffer2[prev];//(2*altitudeBuffer2[prev] - altitudeBuffer2[pprev]);
    altitudeBuffer3[ascentRateCounter] = includeThree ? rawDAltThree : altitudeBuffer3[prev];//(2*altitudeBuffer3[prev] - altitudeBuffer3[pprev]);
    altitudeBuffer4[ascentRateCounter] = includeFour ? rawDAltFour : altitudeBuffer4[prev];//(2*altitudeBuffer4[prev] - altitudeBuffer4[pprev]);
    altitudeBuffer5[ascentRateCounter] = rawIncludeFive ? rawDAltFive : altitudeBuffer5[prev];//(2*altitudeBuffer4[prev] - altitudeBuffer4[pprev]);
    timeBuffer[ascentRateCounter] = ((double)millis()) / 1000.0;
    elapsedSecondsAltitudeErrorHandler = elapsedSeconds + overflowSeconds;
    ascentRateCounter++;
  }
  if (bufferFull) {
    float totalLastAltitude1 = 0.0;
    float totalCurrentAltitude1 = 0.0;
    float totalLastAltitude2 = 0.0;
    float totalCurrentAltitude2 = 0.0;
    float totalLastAltitude3 = 0.0;
    float totalCurrentAltitude3 = 0.0;
    float totalLastAltitude4 = 0.0;
    float totalCurrentAltitude4 = 0.0;
    float totalLastAltitude5 = 0.0;
    float totalCurrentAltitude5 = 0.0;
    float timeLast = 0.0;
    float timeCurrent = 0.0;
    int buffSize = (int)ASCENT_RATE_BUFFER_SIZE;
    int k1; int k2;
    for (int i = 0; i < buffSize / 2; i++) {
      k1 = (i + ascentRateCounter - 1) % buffSize;
      k2 = ((i + ascentRateCounter + buffSize / 2) - 1) % buffSize;
      totalCurrentAltitude1 += altitudeBuffer1[k1];
      totalLastAltitude1 += altitudeBuffer1[k2];
      totalCurrentAltitude2 += altitudeBuffer2[k1];
      totalLastAltitude2 += altitudeBuffer2[k2];
      totalCurrentAltitude3 += altitudeBuffer3[k1];
      totalLastAltitude3 += altitudeBuffer3[k2];
      totalCurrentAltitude4 += altitudeBuffer4[k1];
      totalLastAltitude4 += altitudeBuffer4[k2];
      totalCurrentAltitude5 += altitudeBuffer5[k1];
      totalLastAltitude5 += altitudeBuffer5[k2];
      timeCurrent += timeBuffer[(i + ascentRateCounter - 1) % buffSize];
      timeLast += timeBuffer[((i + ascentRateCounter + buffSize / 2) - 1) % buffSize];
    }
    totalLastAltitude1 = totalLastAltitude1 / (ASCENT_RATE_BUFFER_SIZE / 2);
    totalCurrentAltitude1 = totalCurrentAltitude1 / (ASCENT_RATE_BUFFER_SIZE / 2);
    totalLastAltitude2 = totalLastAltitude2 / (ASCENT_RATE_BUFFER_SIZE / 2);
    totalCurrentAltitude2 = totalCurrentAltitude2 / (ASCENT_RATE_BUFFER_SIZE / 2);
    totalLastAltitude3 = totalLastAltitude3 / (ASCENT_RATE_BUFFER_SIZE / 2);
    totalCurrentAltitude3 = totalCurrentAltitude3 / (ASCENT_RATE_BUFFER_SIZE / 2);
    totalLastAltitude4 = totalLastAltitude4 / (ASCENT_RATE_BUFFER_SIZE / 2);
    totalCurrentAltitude4 = totalCurrentAltitude4 / (ASCENT_RATE_BUFFER_SIZE / 2);
    totalLastAltitude5 = totalLastAltitude5 / (ASCENT_RATE_BUFFER_SIZE / 2);
    totalCurrentAltitude5 = totalCurrentAltitude5 / (ASCENT_RATE_BUFFER_SIZE / 2);
    timeLast = timeLast / (ASCENT_RATE_BUFFER_SIZE / 2);
    timeCurrent = timeCurrent / (ASCENT_RATE_BUFFER_SIZE / 2);
    averageAscentRate1 = (totalCurrentAltitude1 - totalLastAltitude1) / (timeCurrent - timeLast);
    averageAscentRate2 = (totalCurrentAltitude2 - totalLastAltitude2) / (timeCurrent - timeLast);
    averageAscentRate3 = (totalCurrentAltitude3 - totalLastAltitude3) / (timeCurrent - timeLast);
    averageAscentRate4 = (totalCurrentAltitude4 - totalLastAltitude4) / (timeCurrent - timeLast);
    averageAscentRate5 = (totalCurrentAltitude5 - totalLastAltitude5) / (timeCurrent - timeLast);
  }

  tempAscentRate = 0;
  int cnt = 0;
  if (includeOne) {
    tempAscentRate += averageAscentRate1;
    cnt += 1;
  }
  if (includeTwo) {
    tempAscentRate += averageAscentRate2;
    cnt += 1;
  }
  if (includeThree) {
    tempAscentRate += averageAscentRate3;
    cnt += 1;
  }
  if (includeFour) {
    tempAscentRate += averageAscentRate4;
    cnt += 1;
  }
  if (includeFive) {
    tempAscentRate += averageAscentRate5;
    cnt += 1;
  }
  tempAscentRate = tempAscentRate / cnt;

  velocityBuffer[velocityCounter % VEL_BUFFER_SIZE] = tempAscentRate;
  velocityCounter += 1;

  averageAscentRate = 0;
  for (int i = 0; i < VEL_BUFFER_SIZE; i++) {
    averageAscentRate += velocityBuffer[i];
  }
  averageAscentRate /= VEL_BUFFER_SIZE;

  if (fabs(averageAscentRate) > 10) {
    controlMode = false;
    reportMode = true;
  }

}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


  void altitudeController() {
  //  if (cutdownReason == 0.0) {
  if (!firstBallastDrop && DAlt >= ALTITUDE_CHANGE_BALLAST && altitudeSinceLastDrop == -90000.0) {
    altitudeSinceLastDrop = ALTITUDE_SINCE_LAST_DROP_FILLER;
    firstBallastDrop = true;
  }
  reArmConst = I_thresh/(BALLASTER_ALT_DIFF_CONSTANT + BALLASTER_LAST_DROP_CONSTANT);
  if (isValveOpen == 2) {
    altitudeSinceLastVent = min(altitudeSinceLastVent, DAlt + reArmConst);
    valveIncentive = SPEED_CONSTANT * averageAscentRate + ALTITUDE_DIFFERENCE_CONSTANT * (DAlt - ALTITUDE_SETPOINT) +
                     LAST_VALVE_OPENING_CONSTANT * (DAlt - altitudeSinceLastVent);
  }
  if (!ballastState) {
    if (firstBallastDrop){
      altitudeSinceLastDrop = max(altitudeSinceLastDrop, DAlt- reArmConst);
    }
    ballastIncentive = -1 * BALLASTER_SPEED_CONSTANT * averageAscentRate + BALLASTER_ALT_DIFF_CONSTANT * (BALLAST_ALTITUDE_SETPOINT - DAlt) +
                       BALLASTER_LAST_DROP_CONSTANT * (altitudeSinceLastDrop - DAlt);
  }

  if (isValveOpen == 2) MAX_VALVE_ON_MILLIS = IridiumVentTime;
  if (!ballastState) MAX_BALLAST_ON_MILLIS = IridiumBallastTime;

  if (forceValve) {
    valveIncentive = 10;
    MAX_VALVE_ON_MILLIS = forceValveTime;
  }
  if (forceBallast) {
    ballastIncentive = 10;
    MAX_BALLAST_ON_MILLIS = forceBallastTime;
  }

  bool somethingHappened = false;
  // Valve incentive is positive and the valve is closed.
  if (valveIncentive >= (1 + deltaIncentive) && isValveOpen == 2) {
    RBVatt += 1;
    if (controlMode || forceValve) {
      if (forceValve) forceValve = false;
      valveOpen();
      ventCount++;
      isValveOpen = 1;
      ventTime += MAX_VALVE_ON_MILLIS / 1000;
      somethingHappened = true;
    }

    if (RBatt1type < -1) {
      RBatt1type = 0;
      RBatt1time = (((double)millis()) / 1000.0);
    } else if (RBatt2type < -1) {
      RBatt2type = 0;
      RBatt2time = (((double)millis()) / 1000.0);
    } else if (RBatt3type < -1) {
      RBatt3type = 0;
      RBatt3time = (((double)millis()) / 1000.0);
    } else if (RBatt4type < -1) {
      RBatt4type = 0;
      RBatt4time = (((double)millis()) / 1000.0);
    }
  }

  // Valve incentive is positive and the valve is already open.
  if (valveIncentive >= (1 + deltaIncentive) && isValveOpen == 1) {
    MAX_VALVE_ON_MILLIS -= (long)(elapsedSeconds * 1000);
    MAX_VALVE_ON_MILLIS -= (long)(overflowSeconds * 1000);
    if (MAX_VALVE_ON_MILLIS <= 0) {
      altitudeSinceLastVent = DAlt;      
      closeValve();
      isValveOpen = 2;
      writeValveToEEPROM();
    } else {
      somethingHappened = true;
    }
  }

  // Ballast incenttive is positive and we're not ballasting.
  if (ballastIncentive >= (1 + deltaIncentive) && !ballastState) {
    RBBatt += 1;
    if (controlMode || forceBallast) {
      if (forceBallast) forceBallast = false;
      continuouslyDropBallast();
      ballastCount++;
      ballastState = true;
      ballastTime += MAX_BALLAST_ON_MILLIS / 1000;
      somethingHappened = true;
    }
    if (RBatt1type < -1) {
      RBatt1type = 1;
      RBatt1time = (((double)millis()) / 1000.0);
    } else if (RBatt2type < -1) {
      RBatt2type = 1;
      RBatt2time = (((double)millis()) / 1000.0);
    } else if (RBatt3type < -1) {
      RBatt3type = 1;
      RBatt3time = (((double)millis()) / 1000.0);
    } else if (RBatt4type < -1) {
      RBatt4type = 1;
      RBatt4time = (((double)millis()) / 1000.0);
    }
  }

  // Ballast incentive is positive and we're already ballasting.
  if (ballastIncentive >= (1 + deltaIncentive) && ballastState) {
    MAX_BALLAST_ON_MILLIS -= (long)(elapsedSeconds * 1000);
    MAX_BALLAST_ON_MILLIS -= (long)(overflowSeconds * 1000);
    encoderTimer += elapsedSeconds * 1000;
    encoderTimer += overflowSeconds * 1000;
    Serial.println("Encoder Timer:  " + (String)encoderTimer);
    Serial.println("MAX_BALLAST_ON_MILLIS: " + (String)MAX_BALLAST_ON_MILLIS);
    if (encoderTimer >= MAX_TIME_WITHOUT_ENCODER) {
      Serial.println("Reverse");
      reverseBallastDirection();
      encoderTimer = 0;
    }
    continuouslyDropBallast();
    if (MAX_BALLAST_ON_MILLIS <= 0) {
      altitudeSinceLastDrop = DAlt;
      valveAndBallasterDoNothing();
      writeBallastToEEPROM();
      ballastState = false;
    } else {
      somethingHappened = true;
    }
  }
  
  if (!somethingHappened) {
    Serial.println("Valve and Ballaster Do Nothing");
    if (doNothingTimer >= DO_NOTHING_TIMER * 1000) { //30000
      valveAndBallasterDoNothing();
      isValveOpen = 2;
      ballastState = false;
      doNothingTimer = 0;
      forceValve = false;
      forceBallast = false;
    } else {
      doNothingTimer += (elapsedSeconds * 1000 + overflowSeconds * 1000);
    }
  }
  // }
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void reverseBallastDirection() {
  (ballastDirection == 1) ? ballastDirection = 2 : ballastDirection = 1;
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void valveAndBallasterDoNothing() {
  analogWrite(BALLAST_FORWARD, LOW);
  analogWrite(BALLAST_REVERSE, LOW);
  closeValve(); // CHANGE THIS
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void valveOpen() {
  Serial.println("OPENING VALVE");
  analogWrite(HEATER_INTERNAL_STRONG, 0);
  analogWrite(HEATER_INTERNAL_WEAK, 0);
  delay(1000);
  double valveOpenTimer = millis();
  int valvePosition = 0;
  for (int i = 0; i < 5; i++) {
    valvePosition += analogRead(VALVE_POT);
    delay(5);
  }
  valvePosition /= 5;
  if (valvePosition > 200 && !forceValveTimedMode) {
    while (valvePosition < OPEN_VALVE_VALUE) {
      while (true) {
        int newPos = analogRead(VALVE_POT);
        if ((millis() - valveOpenTimer) > VALVE_TIMEOUT) break;
        if (abs(newPos - valvePosition) < 100) {
          valvePosition = newPos;
          break;
        }
        delay(5);
      }
      Serial.println(valvePosition);
      if ((millis() - valveOpenTimer) > VALVE_TIMEOUT) break;
      analogWrite(VALVE_FORWARD, LOW);
      analogWrite(VALVE_REVERSE, VALVE_MOTOR_SPEED);
      delay(5);
      float currentVal = readCurrent(4);
      if (currentVal > currentValMax) currentValMax = currentVal;
      currentValAvg += currentVal;
      currentValNum++;
      analogWrite(VALVE_FORWARD, LOW);
      analogWrite(VALVE_REVERSE, LOW);
    }
    analogWrite(VALVE_FORWARD, LOW);
    analogWrite(VALVE_REVERSE, LOW);
  } else if (forceValveTimedMode || valvePosition <= 200) {
    analogWrite(VALVE_FORWARD, LOW);
    analogWrite(VALVE_REVERSE, VALVE_MOTOR_SPEED);
    delay(VALVE_OPEN_BACKUP_TIMER);
    analogWrite(VALVE_FORWARD, LOW);
    analogWrite(VALVE_REVERSE, LOW);
  }
  Serial.println("FINISHED OPENING VALVE");
  delay(100);
  Serial.println(analogRead(VALVE_POT));
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void closeValve() {
  Serial.println("CLOSING VALVE");
  analogWrite(HEATER_INTERNAL_STRONG, 0);
  analogWrite(HEATER_INTERNAL_WEAK, 0);
  delay(1000);
  double valveOpenTimer = millis();
  int valvePosition = 0;
  for (int i = 0; i < 5; i++) {
    valvePosition += analogRead(VALVE_POT);
    delay(5);
  }
  valvePosition /= 5;
  if (valvePosition > 200 && !forceValveTimedMode) {
    while (valvePosition > CLOSED_VALVE_VALUE) {
      while (true) {
        int newPos = analogRead(VALVE_POT);
        if ((millis() - valveOpenTimer) > VALVE_TIMEOUT) break;
        if (abs(newPos - valvePosition) < 100) {
          valvePosition = newPos;
          break;
        }
        delay(5);
      }
      Serial.println(valvePosition);
      if ((millis() - valveOpenTimer) > VALVE_TIMEOUT) break;
      analogWrite(VALVE_FORWARD, VALVE_MOTOR_SPEED);
      analogWrite(VALVE_REVERSE, LOW);
      delay(5);
      float currentVal = readCurrent(4);
      if (currentVal > currentValMax) currentValMax = currentVal;
      currentValAvg += currentVal;
      currentValNum++;
      analogWrite(VALVE_FORWARD, LOW);
      analogWrite(VALVE_REVERSE, LOW);
    }
    analogWrite(VALVE_FORWARD, LOW);
    analogWrite(VALVE_REVERSE, LOW);
  } else if (forceValveTimedMode || valvePosition <= 200) {
    analogWrite(VALVE_FORWARD, VALVE_MOTOR_SPEED);
    analogWrite(VALVE_REVERSE, LOW);
    delay(VALVE_TIMEOUT);
    analogWrite(VALVE_FORWARD, LOW);
    analogWrite(VALVE_REVERSE, LOW);
  }
  Serial.println("FINISHED CLOSING VALVE");
  Serial.println(analogRead(VALVE_POT));
  delay(100);
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void continuouslyDropBallast() {
  if (ballastDirection == 1) {
    analogWrite(BALLAST_FORWARD, BALLAST_MOTOR_SPEED);
    analogWrite(BALLAST_REVERSE, LOW);
  } else if (ballastDirection == 2) {
    analogWrite(BALLAST_FORWARD, LOW);
    analogWrite(BALLAST_REVERSE, BALLAST_MOTOR_SPEED);
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


void cutdownBalloon() {
  for (int i = 0; i < 3; i++) {
    Serial.println("CUTTING DOWN BALLOON");
    analogWrite(HEATER_INTERNAL_STRONG, 0);
    analogWrite(HEATER_INTERNAL_WEAK, 0);
    delay(1000);
    double valveOpenTimer = millis();
    while ((millis() - valveOpenTimer) <= VALVE_CUTDOWN_TIMEOUT) {
      Serial.println(analogRead(VALVE_POT));
      analogWrite(VALVE_FORWARD, LOW);
      analogWrite(VALVE_REVERSE, VALVE_MOTOR_SPEED);
      delay(5);
    }
    analogWrite(VALVE_FORWARD, LOW);
    analogWrite(VALVE_REVERSE, LOW);
    Serial.println("FINISHED CUTTING DOWN");
    delay(100);
    cutdownReason = 1;
    closeValve();
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


void constantChanger(int index, float valuee) {
  doReadback = true;
  if (index == 0) altitudeSinceLastVent = valuee;
  else if (index == 1) altitudeSinceLastDrop = valuee;
  else if (index == 2) ALTITUDE_SETPOINT = valuee;
  else if (index == 3) BALLAST_ALTITUDE_SETPOINT = valuee;
  else if (index == 4) INTERNAL_TEMP_SETPOINT = valuee;
  else if (index == 5) ALTITUDE_CHANGE_BALLAST = valuee;
  else if (index == 6) SPEED_CONSTANT = valuee;
  else if (index == 7) ALTITUDE_DIFFERENCE_CONSTANT = 1.0 / valuee;
  else if (index == 8) LAST_VALVE_OPENING_CONSTANT = 1.0 / valuee;
  else if (index == 9) BALLASTER_SPEED_CONSTANT = valuee;
  else if (index == 10) BALLASTER_ALT_DIFF_CONSTANT = 1.0 / valuee;
  else if (index == 11) BALLASTER_LAST_DROP_CONSTANT = 1.0 / valuee;
  else if (index == 12) COMM_BEACON_INTERVAL = valuee;
  else if (index == 13) GPS_BEACON_INTERVAL = valuee;
  else if (index == 14) IridiumVentTime = valuee;
  else if (index == 15) IridiumBallastTime = valuee;
  else if (index == 16) {
    int tval = valuee;
    activeSensors = tval;
  }
  else if (index == 17) {
    int val = valuee;
    if (val == 0) { // what the fuck is wrong with you
      reportMode = false;
      controlMode = false;
    } else if (val == 1) { // that's chill flight mode
      reportMode = false;
      controlMode = true;
    } else if (val == 2) { // that's we're fucked mode
      reportMode = true;
      controlMode = false;
    } else if (val == 3) { // that's launch / data gathering mode
      reportMode = true;
      controlMode = true;
    } else { // what wat wat whattt
      reportMode = true;
      controlMode = false;
    }
  }
  else if (index == 30) {
    forceValve = true;
    forceValveTime = valuee;
  }
  else if (index == 31) {
    forceBallast = true;
    forceBallastTime = valuee;
  }
  else if (index == 33) {
    LEDon = (bool) valuee;
  }
  else if (index == 34) { // RB
    if (valuee == 2) {
      if (powerStates[0] == 0 || powerStates[0] == 1 || powerStates[0] == 3) {
        powerStates[0] = 1;
        EEPROM.write(10, 1);
        digitalWrite(RB_GATE, HIGH);
        delay(1000);
        isbd.begin();
        powerStates[0] = 2;
        EEPROM.write(10, 2);
      }
    } else if (valuee == 0 || valuee == 1 || valuee == 3) {
      digitalWrite(RB_GATE, LOW);
      powerStates[0] = valuee;
      EEPROM.write(10, valuee);
      isbd.begin();
    }
  }
  else if (index == 35) {
    if (valuee == 0 || valuee == 1) {
      powerStates[1] = valuee;
      digitalWrite(GPS_ENABLE, LOW);
      EEPROM.write(11, valuee);
    } else if (valuee == 2) {
      EEPROM.write(11, 1);
      digitalWrite(GPS_ENABLE, HIGH);
      delay(500);
      EEPROM.write(11, 2);
      powerStates[1] = 2;
      setGPSFlightMode();
    } else if (valuee == 3) {
      hsGPS.println("$PUBX,00*33");
      delay(1000);
      smartdelay2(GPS_ACQUISITION_TIME_2);
      age1 = 0;
      lati = tinygps.location.lat();
      longi = tinygps.location.lng();
      alt = tinygps.altitude.meters();
    }
  }
  else if (index == 36) {
    powerStates[2] = valuee;
    EEPROM.write(12, valuee);
    if (valuee == 0 || valuee == 1 || valuee == 3) {
      analogWrite(HEATER_INTERNAL_STRONG, 0);
      analogWrite(HEATER_INTERNAL_WEAK, 0);
    }
  }
  else if (index == 37) {
    pressure_baseline = valuee;
  }
  else if (index == 38) {
    DO_NOTHING_TIMER = valuee;
  }
  else if (index == 39) {
    //forceValveTimedMode = valuee;
  }
  else if (index == 40) {
    CLOSED_VALVE_VALUE = valuee;
  }
  else if (index == 41) {
    OPEN_VALVE_VALUE = valuee;
  }
  else if (index == 42) {
    VALVE_MOTOR_SPEED = valuee;
  }
  else if (index == 43) {
    VALVE_OPEN_BACKUP_TIMER = valuee;
  }
  else if (index == 44) {
    int vall = valuee;
    if (vall == 0) {
      strongHeaterOn = false;
      weakHeaterOn = false;
    } else if (vall == 1) {
      strongHeaterOn = false;
      weakHeaterOn = true;
    } else if (vall == 2) {
      strongHeaterOn = true;
      weakHeaterOn = false;
    } else if (vall == 3) {
      strongHeaterOn = true;
      weakHeaterOn = true;
    } 
  }
  else if (index == 45) {
    I_thresh = valuee; 
  }
  else if (index == 46) {

  }
  else if (index == 54) {
    doReadback = true;
  }
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


void iridiumParser(int number, uint8_t rxBuffer[]) {
  // First put everything in a character array
  char fullMessage[200] = {0};
  for (uint8_t i = 0; i < number; i++) {
    fullMessage[i] = rxBuffer[i];
  }

  // Declare separate commands and indexies
  char firstCommand[100] = {0}; int firstIndex = 0; String firstCommandStr = ""; char *p1_1;
  char secondCommand[100] = {0}; int secondIndex = 0; String secondCommandStr = ""; char *p2_1;
  char thirdCommand[100] = {0}; int thirdIndex = 0; String thirdCommandStr = ""; char *p3_1;
  char fourthCommand[100] = {0}; int fourthIndex = 0; String fourthCommandStr = ""; char *p4_1;
  char fifthCommand[100] = {0}; int fifthIndex = 0; String fifthCommandStr = ""; char *p5_1;
  char sixthCommand[100] = {0}; int sixthIndex = 0; String sixthCommandStr = ""; char *p6_1;
  char seventhCommand[100] = {0}; int seventhIndex = 0; String seventhCommandStr = ""; char *p7_1;
  char eighthCommand[100] = {0}; int eighthIndex = 0; String eighthCommandStr = ""; char *p8_1;

  // Read the message, scan it, and separate commands
  int nscanned = sscanf(fullMessage, "%d,%s %d,%s %d,%s %d,%s %d,%s %d,%s %d,%s %d,%s", &firstIndex, firstCommand, &secondIndex, secondCommand,
                        &thirdIndex, thirdCommand, &fourthIndex, fourthCommand, &fifthIndex, fifthCommand, &sixthIndex, sixthCommand, &seventhIndex,
                        seventhCommand, &eighthIndex, eighthCommand);

  for (int i = 0; i < 9; i++) {
    returnValuee[i] = 99;
  }

  if (nscanned % 2 != 0) return;


  // Convert commands to Arduino readable strings then act on commands
  if (nscanned >= 2) {
    for (uint8_t i = 0; i < strlen(firstCommand); i++) {
      firstCommandStr += firstCommand[i];
    }
    if (firstIndex == CUTDOWN_INDEX && firstCommandStr.equals(CUTDOWN_COMMAND)) {
      cutdownBalloon();
    } else {
      strtod(firstCommand, &p1_1);
      if (firstIndex < 0 || firstIndex > 80 || *p1_1) return;
      constantChanger(firstIndex, firstCommandStr.toFloat());
    }
  }
  if (nscanned >= 4) {
    for (uint8_t i = 0; i < strlen(secondCommand); i++) {
      secondCommandStr += secondCommand[i];
    }
    if (secondIndex == CUTDOWN_INDEX && secondCommandStr.equals(CUTDOWN_COMMAND)) {
      cutdownBalloon();
    } else {
      strtod(secondCommand, &p2_1);
      if (secondIndex < 0 || secondIndex > 80 || *p2_1) return;
      constantChanger(secondIndex, secondCommandStr.toFloat());
    }
  }
  if (nscanned >= 6) {
    for (uint8_t i = 0; i < strlen(thirdCommand); i++) {
      thirdCommandStr += thirdCommand[i];
    }
    if (thirdIndex == CUTDOWN_INDEX && thirdCommandStr.equals(CUTDOWN_COMMAND)) {
      cutdownBalloon();
    } else {
      strtod(thirdCommand, &p3_1);
      if (thirdIndex < 0 || thirdIndex > 80 || *p3_1) return;
      constantChanger(thirdIndex, thirdCommandStr.toFloat());
    }
  }
  if (nscanned >= 8) {
    for (uint8_t i = 0; i < strlen(fourthCommand); i++) {
      fourthCommandStr += fourthCommand[i];
    }
    if (fourthIndex == CUTDOWN_INDEX && fourthCommandStr.equals(CUTDOWN_COMMAND)) {
      cutdownBalloon();
    } else {
      strtod(fourthCommand, &p4_1);
      if (fourthIndex < 0 || fourthIndex > 80 || *p4_1) return;
      constantChanger(fourthIndex, fourthCommandStr.toFloat());
    }
  }
  if (nscanned >= 10) {
    for (uint8_t i = 0; i < strlen(fifthCommand); i++) {
      fifthCommandStr += fifthCommand[i];
    }
    if (fifthIndex == CUTDOWN_INDEX && fifthCommandStr.equals(CUTDOWN_COMMAND)) {
      cutdownBalloon();
    } else {
      strtod(fifthCommand, &p5_1);
      if (fifthIndex < 0 || fifthIndex > 80 || *p5_1) return;
      constantChanger(fifthIndex, fifthCommandStr.toFloat());
    }
  }
  if (nscanned >= 12) {
    for (uint8_t i = 0; i < strlen(sixthCommand); i++) {
      sixthCommandStr += sixthCommand[i];
    }
    if (sixthIndex == CUTDOWN_INDEX && sixthCommandStr.equals(CUTDOWN_COMMAND)) {
      cutdownBalloon();
    } else {
      strtod(sixthCommand, &p6_1);
      if (sixthIndex < 0 || sixthIndex > 80 || *p6_1) return;
      constantChanger(sixthIndex, sixthCommandStr.toFloat());
    }
  }
  if (nscanned >= 14) {
    for (uint8_t i = 0; i < strlen(seventhCommand); i++) {
      seventhCommandStr += seventhCommand[i];
    }
    if (seventhIndex == CUTDOWN_INDEX && seventhCommandStr.equals(CUTDOWN_COMMAND)) {
      cutdownBalloon();
    } else {
      strtod(seventhCommand, &p7_1);
      if (seventhIndex < 0 || seventhIndex > 80 || *p7_1) return;
      constantChanger(seventhIndex, seventhCommandStr.toFloat());
    }
  }
  if (nscanned == 16) {
    for (uint8_t i = 0; i < strlen(eighthCommand); i++) {
      eighthCommandStr += eighthCommand[i];
    }
    if (eighthIndex == CUTDOWN_INDEX && eighthCommandStr.equals(CUTDOWN_COMMAND)) {
      cutdownBalloon();
    } else {
      strtod(eighthCommand, &p8_1);
      if (eighthIndex < 0 || eighthIndex > 80 || *p8_1) return;
      constantChanger(eighthIndex, eighthCommandStr.toFloat());
    }
  }

  if (nscanned >= 2) returnValuee[0] = firstIndex;
  if (nscanned >= 4) returnValuee[1] = secondIndex;
  if (nscanned >= 6) returnValuee[2] = thirdIndex;
  if (nscanned >= 8) returnValuee[3] = fourthIndex;
  if (nscanned >= 10)returnValuee[4] = fifthIndex;
  if (nscanned >= 12)returnValuee[5] = sixthIndex;
  if (nscanned >= 14)returnValuee[6] = seventhIndex;
  if (nscanned == 16)returnValuee[7] = eighthIndex;

  returnValuee[8] = nscanned / 2;
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void log_variable(float* variable, float min, float max, int bit) {
  mins[argc] = min;
  maxs[argc] = max;//+(max-min)*pow(2,-bit);
  bits[argc] = bit;
  vars[argc] = variable;
  argc += 1;
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


void satelliteTransmission() {
  if ((minutes > commBeaconInterval)/* && isbd.isAsleep()*/) {
    internalTemperatureAverage = internalTemperatureAverage / temperatureCount;
    neckTemperatureAverage = neckTemperatureAverage /temperatureCount;
    RBmsg = messagesSent;
    RBvel = averageAscentRate;
    RBaltB = DAlt;
    RBaltG = alt;
    RBlat = lati;
    RBlong = longi;
    RBtemp = internalTemperatureAverage;
    RBbat = batteryCapacity / initialBatteryCapacity;
    RBvoltP = batteryVoltage;
    RBavgI = currentAvg / currentNum;
    RBmaxI = currentMax * 1000.0;
    RBminI = currentMin * 1000.0;

    RBStrongHeat = strongHeaterOn;
    RBMildHeat = weakHeaterOn;

    RBreadback = doReadback;

    RBcurrentGPSmax = currentGPSMax;
    RBcurrentGPSavg = currentGPSAvg / currentNum;
    RBcurrentRBmax = currentRBMax;
    RBcurrentRBavg = currentRBAvg / currentNum;
    RBcurrentValmax = currentValMax;
    RBcurrentValavg = (currentValNum != 0) ? currentValAvg / currentValNum : 0;
    RBcurrentBalmax = currentBalMax;
    RBcurrentBalavg = (currentBalNum != 0) ? currentBalAvg / currentBalNum : 0;

    RBVin = valveIncentive;
    RBBin = ballastIncentive;
    RBcutdown = cutdownReason; // umm look into that mebeh
    RBBtime = ballastTime;
    RBVtime = ventTime;
    RBheat = mainDutyCycle;
    RBs1delta = sensor1max - sensor1min;
    RBs2delta = sensor2max - sensor2min;
    RBs3delta = sensor3max - sensor3min;
    RBs4delta = sensor4max - sensor4min;
    RBmode = (reportMode << 1) + (controlMode << 0);
    RBLEDon = LEDon;

    RBpowerRB = powerStates[0];
    RBpowerGPS = powerStates[1];
    RBpowerHeaters = powerStates[2];
    //RBpotReading = analogRead(VALVE_POT);
    RBNeckTemp = neckTemperatureAverage;
    RBsensors = activeSensors;
    RBt0 = initAltitudeTime;
    
    

    RBs1rej = log(sensor1rej + 1) / log(2);
    RBs2rej = log(sensor2rej + 1) / log(2);
    RBs3rej = log(sensor3rej + 1) / log(2);
    RBs4rej = log(sensor4rej + 1) / log(2);

    RBatt1time -= RBt0;
    RBatt2time -= RBt0;
    RBatt3time -= RBt0;
    RBatt4time -= RBt0;

    RBIthresh = I_thresh;
    RBreArmConst = reArmConst;
    

    RBCValveSpeed = SPEED_CONSTANT; RBCValveAltDiff = 1./ALTITUDE_DIFFERENCE_CONSTANT; RBCValveLast = 1./LAST_VALVE_OPENING_CONSTANT; RBCBallastSpeed = BALLASTER_SPEED_CONSTANT;
    RBCBallastAltDiff = 1./BALLASTER_ALT_DIFF_CONSTANT; RBCBallastLast = 1./BALLASTER_LAST_DROP_CONSTANT; RBCValveSetpoint = ALTITUDE_SETPOINT; RBCBallastSetpoint = BALLAST_ALTITUDE_SETPOINT;
    RBFValveLast = altitudeSinceLastVent; RBFDropLast = altitudeSinceLastDrop; RBCOpenValve = OPEN_VALVE_VALUE; RBCClosedValve = CLOSED_VALVE_VALUE; RBCCutdownValve = CUTDOWN_VALVE_VALUE;
    RBCMaxEncoder = MAX_TIME_WITHOUT_ENCODER; RBCDoNothing = DO_NOTHING_TIMER; RBCValTime = MAX_VALVE_ON_MILLIS/1000.; RBCBalTime = MAX_BALLAST_ON_MILLIS/1000.; RBControlMode = controlMode;
    RBReportMode = reportMode; RBPotMode = forceValveTimedMode; RBTemperatureSetpoint = INTERNAL_TEMP_SETPOINT; RBRBinterval = COMM_BEACON_INTERVAL; RBGPSinterval = GPS_BEACON_INTERVAL;
    RBValveSpeed = VALVE_MOTOR_SPEED; RBOpeningTime = VALVE_OPEN_BACKUP_TIMER;

    messagesSent += 1;
    delay(200);
    Serial.println("Beginning to talk to the RockBLOCK...");
    Serial.println("Sending RB message");


    uint8_t temporary[50] = {0};
    int len = encodeTelemetry(temporary);

    float var;
    int adc;
    uint8_t rxBuffer[200] = {0};
    int byteidx = 0;
    int bitidx = 7;
    for (int i = 0; i < argc; i++) {
      var = *vars[i];
      Serial.println(var);
      if (var > maxs[i]) var = maxs[i];
      if (var < mins[i]) var = mins[i];
      adc = round( ( pow(2, bits[i]) - 1 ) * (var - mins[i]) / (maxs[i] - mins[i]));
      for (int j = bits[i] - 1; j >= 0; j--) {
        bool bit = adc & (1 << j);
        if (bit) rxBuffer[byteidx] |= (1 << bitidx);
        bitidx -= 1;
        if (bitidx < 0) {
          bitidx = 7;
          byteidx += 1;
        }
      }
      if (!reportMode && i == debugThreshold) break; // stop if we're no longer in reduced-land.
      if (vars[i] == &RBreadback && RBreadback == 0) break;
    }
    if (bitidx != 7) byteidx += 1; // Leave the rest of the byte empty between this and possibly callbacks.

    if (len > 0) {
      for (int k = 0; k < len; k++) {
        rxBuffer[byteidx] = temporary[k];
        byteidx += 1;
      }
    }
    
    for (int i = 0; i < (byteidx+1); i++) {
      uint8_t x = rxBuffer[i];
      (x & 0x80 ? Serial.print('1') : Serial.print('0'));
      (x & 0x40 ? Serial.print('1') : Serial.print('0'));
      (x & 0x20 ? Serial.print('1') : Serial.print('0'));
      (x & 0x10 ? Serial.print('1') : Serial.print('0'));
      (x & 0x08 ? Serial.print('1') : Serial.print('0'));
      (x & 0x04 ? Serial.print('1') : Serial.print('0'));
      (x & 0x02 ? Serial.print('1') : Serial.print('0'));
      (x & 0x01 ? Serial.print('1') : Serial.print('0'));

    }
    Serial.println();
    // reset variables before transmitting!

    temperatureCount = 0.0;
    internalTemperatureAverage = 0.0;

    sensor1max = -40000;
    sensor2max = -40000;
    sensor3max = -40000;
    sensor4max = -40000;

    sensor1min = 40000;
    sensor2min = 40000;
    sensor3min = 40000;
    sensor4min = 40000;

    sensor1rej = 0;
    sensor2rej = 0;
    sensor3rej = 0;
    sensor4rej = 0;

    RBVatt = 0;
    RBBatt = 0;

    RBatt1type = -2;
    RBatt1time = 0;

    RBatt2type = -2;
    RBatt2time = 0;

    RBatt3type = -2;
    RBatt3time = 0;

    RBatt4type = -2;
    RBatt4time = 0;

    bufferSize = sizeof(rxBuffer);
    Serial.println("SENDING FORREAL");
    int ret = isbd.sendReceiveSBDBinary(rxBuffer, byteidx, rxBuffer, bufferSize);
    if (ret == ISBD_SUCCESS) {
      currentMin = 10000;
      currentMax = 0;
      currentAvg = 0;
      currentNum = 0;
      currentGPSMax = 0;
      currentGPSAvg = 0;
      currentRBMax = 0;
      currentRBAvg = 0;
      currentValMax = 0;
      currentValAvg = 0;
      currentBalMax = 0;
      currentBalAvg = 0;
      initAltitudeTime += QcurrentDt*QmulFactor*(currentAltitude-1) + Qdt;
      QcurrentDt = Qdt;
      QmulFactor = 1;
      currentAltitude = 0;
      doReadback = false;
    }
    int number = bufferSize;
    Serial.println("Not going to sleep mode, buddy.");
    delay(2000);
    for (int i = 0; i < 9; i++) {
      returnValuee[i] = 99;
    }
    if (number > 0) {
      iridiumParser(number, rxBuffer);
    }
    commBeaconInterval = minutes + COMM_BEACON_INTERVAL;
  }
}




//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

double tempSensorPolling(double temp1, double temp2, double temp3, double temp4) {
  boolean iincludeOne = true;
  boolean iincludeTwo = true;
  boolean iincludeThree = true;
  boolean iincludeFour = true;
  double tempCount = 4;
  double temp = 0.0;
  if (temp1 < ERROR_MIN_VALUES[2] || temp1 > ERROR_MAX_VALUES[2]) {
    iincludeOne = false;
    tempCount--;
  }
  if (temp2 < ERROR_MIN_VALUES[2] || temp2 > ERROR_MAX_VALUES[2]) {
    iincludeTwo = false;
    tempCount--;
  }
  if (temp3 < ERROR_MIN_VALUES[2] || temp3 > ERROR_MAX_VALUES[2]) {
    iincludeThree = false;
    tempCount--;
  }
  if (temp4 < ERROR_MIN_VALUES[2] || temp4 > ERROR_MAX_VALUES[2]) {
    iincludeFour = false;
    tempCount--;
  }
  if (fabs(internalTemperature - temp1) > MAX_TEMP_DIFF && iincludeOne) {
    iincludeOne = false;
    tempCount--;
  }
  if (fabs(internalTemperature - temp2) > MAX_TEMP_DIFF && iincludeTwo) {
    iincludeTwo = false;
    tempCount--;
  }
  if (fabs(internalTemperature - temp3) > MAX_TEMP_DIFF && iincludeThree) {
    iincludeThree = false;
    tempCount--;
  }
  if (fabs(internalTemperature - temp4) > MAX_TEMP_DIFF && iincludeFour) {
    iincludeFour = false;
    tempCount--;
  }
  if (iincludeOne) {
    temp += temp1;
  }
  if (iincludeTwo) {
    temp += temp2;
  }
  if (iincludeThree) {
    temp += temp3;
  }
  if (iincludeFour) {
    temp += temp4;
  }
  if (tempCount == 0) return 99;
  return (temp / tempCount);
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

bool bounds_check(float pressure) { // hectopascals
  if (pressure > 1800) return false;
  if (pressure < 3) return false;
  return true;
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void copyBits(const char *code, uint8_t *buffer, int& byteidx, int& bitidx) {
  if (byteidx > 45) return;
  int len = strlen(code);
  for (int i = 0; i < len; i++) {
    bool bit = (code[i] == '1');
    if (bit) buffer[byteidx] |= (1 << bitidx);
    bitidx -= 1;
    if (bitidx < 0) {
      bitidx = 7;
      byteidx += 1;
    }
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


void copyNumber(int number, int size, uint8_t *buffer, int& byteidx, int& bitidx) {
  if (byteidx > 45) return;
  for (int j = size - 1; j >= 0; j--) {
    bool bit = number & (1 << j);
    if (bit) buffer[byteidx] |= (1 << bitidx);
    bitidx -= 1;
    if (bitidx < 0) {
      bitidx = 7;
      byteidx += 1;
    }
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// uint8_t temporary[50] = {0};
int encodeTelemetry(uint8_t *temporary) {
  int loopidx = 0;
  while (loopidx < 5) { // JPL RULEZ DON'T DO INFINITE LOOPS
    int bytesUsed = 0;

    float slope = (kalmanAltitudes[currentAltitude - 1] - kalmanAltitudes[0]) / ((QcurrentDt * QmulFactor) * (currentAltitude - 1));
    Serial.println("Slope");
    Serial.println(slope, 5);

    float newAltitudes[NUM_KALMANS] = {0};
    float mx = -50000;
    float mn = 50000;
    int k;
    for (k = 0; k < currentAltitude; k++) {
      float alt = kalmanAltitudes[k] - (kalmanAltitudes[0] + slope * k * QcurrentDt * QmulFactor);
      newAltitudes[k] = alt;
      if (alt > mx) mx = alt;
      if (alt < mn) mn = alt;
    }
    Serial.println(mn, 5);
    Serial.println(mx - mn, 5);
    Serial.println(kalmanAltitudes[0], 5);
    Serial.println(mx);
    Serial.println(mn);
    
    if (k % 16 != 0) {
      int top = ((k / 16) + 1) * 16;
      for (; k < top; k++) {
        newAltitudes[k] = newAltitudes[k - 1];
      }
    }

    RBheightOffset = kalmanAltitudes[0] + mn;
    RBheightScale = mx - mn;
    RBtimeFactor = log2(QmulFactor);
    RBtimeInterval = QcurrentDt;
    RBaltDiff = kalmanAltitudes[currentAltitude - 1] - kalmanAltitudes[0];
    RBmaxTime = currentAltitude;

    int byteidx = 0;
    int bitidx = 7;
    int prevDC = 0;
    float block[16] = {0};
    //int trans[16] = {0};
    for (int k = 0; k < currentAltitude; k += 16) {
      for (int j = 0; j < 16; j++) {
        float num = (newAltitudes[k + j] - mn) / (mx - mn) * (pow(2, Qbit) - 1) - pow(2, Qbit - 1);
        block[j] = num;
      }
      int nzeros = 0;
      for (int j = 0; j < 16; j++) {
        float s = 0;
        //Serial.println("yo");
        for (int n = 0; n < 16; n++) {
          s += block[n] * cos(M_PI / 16.*(n + 0.5) * j);
        }
        //Serial.println("raw");
        //Serial.println(s);
        int trans = round(s / Qmat[j]);
        if (j % 16 == 0) {
          //Serial.print("DC ");
          //Serial.println(trans);
          int diff = trans - prevDC;
          //Serial.println("Diff");
          //Serial.println(diff);
          int size = (diff == 0) ? 0 : floor(log2(abs(diff)) + 1);
          if (diff < 0) {
            diff = diff + ((1 << size) - 1);
          }
          if (size < 8) {
            const char *code = DCcoeffs[size];
            copyBits(code, temporary, byteidx, bitidx);
          }
          copyNumber(diff, size, temporary, byteidx, bitidx);

          prevDC = trans;
        } else { // it's AC
          //Serial.print("AC ");
          //Serial.println(trans);
          if (trans == 0) nzeros++;
          else {
            int size = floor(log2(abs(trans)) + 1);
            if ((size*16+nzeros) < 128) {
              const char *code = ACcoeffs[size * 16 + nzeros];
              //Serial.print("Encoded AC "); Serial.print(trans); Serial.print(" as "); Serial.println(code);
              copyBits(code, temporary, byteidx, bitidx);
            }
            if (trans < 0) trans += (1 << size) - 1;
            copyNumber(trans, size, temporary, byteidx, bitidx);
            nzeros = 0;
          }
        }
      }
      const char *code = ACcoeffs[0 * 16 + nzeros];
      copyBits(code, temporary, byteidx, bitidx);
    }

    Serial.println("Final string: ");
    for (int i = 0; i < (byteidx + 1); i++) {
      uint8_t x = temporary[i];
      (x & 0x80 ? Serial.print('1') : Serial.print('0'));
      (x & 0x40 ? Serial.print('1') : Serial.print('0'));
      (x & 0x20 ? Serial.print('1') : Serial.print('0'));
      (x & 0x10 ? Serial.print('1') : Serial.print('0'));
      (x & 0x08 ? Serial.print('1') : Serial.print('0'));
      (x & 0x04 ? Serial.print('1') : Serial.print('0'));
      (x & 0x02 ? Serial.print('1') : Serial.print('0'));
      (x & 0x01 ? Serial.print('1') : Serial.print('0'));

    }
    Serial.println();


    bytesUsed = byteidx+1;
    if (bytesUsed < 40) {
      return bytesUsed;
    } else {
      halveTemporalResolution();
    }
    loopidx++;
  }
  return -1;
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void halveTemporalResolution() {
  double current = QcurrentDt * QmulFactor;
  initAltitudeTime += current / 2.;

  int last = 0;
  for (int i = 0; i < currentAltitude; i += 2) {
    kalmanAltitudes[i / 2] = (kalmanAltitudes[i] + kalmanAltitudes[i + 1]) / 2.;
    last = i / 2;
  }
  currentAltitude = last + 1;
  QmulFactor = QmulFactor * 2;
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


double getNextAltitudeTime() {
  return initAltitudeTime + QcurrentDt * QmulFactor * currentAltitude;
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void doTheKalmanThing() {

  if (h < -5000) { // first altitude yo
    h = DAlt;
    return;
  }
  double current = (((double)millis()) / 1000.0);
  double dt = current - lastDAltSeconds;
  double sigma = pow(0.8, 2);
  double hmeas = DAlt;
  double hnew = dt * v + h + (-dt * v - h + hmeas) * (dt * dt * qn + dt * p21 + dt * (dt * p22 + p12) + p11) / (dt * dt * qn + dt * p21 + dt * (dt * p22 + p12) + p11 + sigma);
  double vnew = v + (dt * p22 + dt * qn + p21) * (-dt * v - h + hmeas) / (dt * dt * qn + dt * p21 + dt * (dt * p22 + p12) + p11 + sigma);
  double p11new = (-(dt * dt * qn + dt * p21 + dt * (dt * p22 + p12) + p11) / (dt * dt * qn + dt * p21 + dt * (dt * p22 + p12) + p11 + sigma) + 1) * (dt * dt * qn + dt * p21 + dt * (dt * p22 + p12) + p11);
  double p12new = (-(dt * dt * qn + dt * p21 + dt * (dt * p22 + p12) + p11) / (dt * dt * qn + dt * p21 + dt * (dt * p22 + p12) + p11 + sigma) + 1) * (dt * p22 + dt * qn + p12);
  double p21new = dt * p22 + dt * qn + p21 - (dt * p22 + dt * qn + p21) * (dt * dt * qn + dt * p21 + dt * (dt * p22 + p12) + p11) / (dt * dt * qn + dt * p21 + dt * (dt * p22 + p12) + p11 + sigma);
  double p22new = p22 + qn - (dt * p22 + dt * qn + p12) * (dt * p22 + dt * qn + p21) / (dt * dt * qn + dt * p21 + dt * (dt * p22 + p12) + p11 + sigma);
  p11 = p11new;
  p12 = p12new;
  p21 = p21new;
  p22 = p22new;
  h = hnew;
  v = vnew;

  int i = 0;
  while (current > getNextAltitudeTime() && i < 10) { // JPL RULEZ NEVER HAVE INFINITE LOOPS
    if (currentAltitude == NUM_KALMANS) {
      halveTemporalResolution();
    }
    double next = getNextAltitudeTime();
    double ddt = current - next;
    double hp = h - ddt * v;
    kalmanAltitudes[currentAltitude] = hp;
    currentAltitude++;
    i++;
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

double doTheFitThing(double r) {
   double x = log(r);
   
   // Joan, Jan 10, 2017. Wow, I used const. I'm such a clean coder.
   const double a = 4.00141132e+02, b=  -9.94189235e+01, c=   1.16421122e+01, d=  -9.42945754e-01,
    e = 4.40544424e-02, f=  -8.79058885e-04;
   
   return a+b*x+c*pow(x,2)+d*pow(x,3)+e*pow(x,4)+f*pow(x,5);
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

double doTheThermistorThing() {
  double vA = analogRead(VALVE_POT)*1.2/(pow(2.0,12.0));
  // vA = (1 volt)*(R/(100k+R))
  // va*100k + vA*R = 1 volt*R
  // va*100k = R (1 V - vA)
  // R = va*100k/(1-vA)
  double lmao = doTheFitThing(vA*100000.0/(1.-vA));
  Serial.println("lmao");
  Serial.println(lmao);
  Serial.println(vA,5);
  return lmao;
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


void iterateSensors() {
  //sensor_i2c.readSensor();
  
  /* GET READINGS FROM ALL 5 TEMP SENSORS */
  double temperature_c_1 = sensor_1.readTemperature();
  double temperature_c_2 = sensor_2.readTemperature();
  double temperature_c_3 = sensor_3.readTemperature();
  double temperature_c_4 = sensor_4.readTemperature();
  neckTemperature = doTheThermistorThing();
  neckTemperatureAverage += neckTemperature;
  //double temperature_c_5 = sensor_i2c.temperature();

  internalTemperature = tempSensorPolling(temperature_c_1, temperature_c_2, temperature_c_2, temperature_c_4);
  if (internalTemperature >98 && internalTemperature < 100) internalTemperature = (temperature_c_1 + temperature_c_2 + temperature_c_4)/3.0;
  //if (internalTemperature == 99) internalTemperature = temperature_c_5;
  internalTemperatureAverage += internalTemperature;
  temperatureCount += 1.0;

  /* PRESSURE READINGS */
  float pressure_abs_1 = sensor_1.readPressure() / 100.0;
  float pressure_abs_2 = sensor_2.readPressure() / 100.0;
  float pressure_abs_3 = sensor_3.readPressure() / 100.0;
  float pressure_abs_4 = sensor_4.readPressure() / 100.0;
  //float pressure_abs_5 = sensor_i2c.pressure();

  /*Serial.println(pressure_abs_1);
  Serial.println(pressure_abs_2);
  Serial.println(pressure_abs_3);
  Serial.println(pressure_abs_4);
  Serial.println(pressure_abs_5);*/

  // jesus take the wheel

  int mxnum = -1;
  float mxdiff = 9999;
  int num[16] = {0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4}; // number of ones in corresponding binary numbers lol
  int cnt;
  int mmax;
  int mmin;
  float diff;
  for (int i = 0; i < 16; i++) {
    if (((i & 8) <= (activeSensors & 8)) && ((i & 4) <= (activeSensors & 4)) && ((i & 2) <= (activeSensors & 2)) && ((i & 1) <= (activeSensors & 1))) { // this sensor combo is allowed
      cnt = num[i];
      mmax = -9999;
      mmin = 9999;
      if ((i & (1 << 3)) != 0 && bounds_check(pressure_abs_1)) { // sensor 1 allowed
        if (pressure_abs_1 > mmax) mmax = pressure_abs_1;
        if (pressure_abs_1 < mmin) mmin = pressure_abs_1;
      }
      if ((i & (1 << 2)) != 0 && bounds_check(pressure_abs_2)) { // sensor 2 allowed
        if (pressure_abs_2 > mmax) mmax = pressure_abs_2;
        if (pressure_abs_2 < mmin) mmin = pressure_abs_2;
      }
      if ((i & (1 << 1)) != 0 && bounds_check(pressure_abs_3)) { // sensor 3 allowed
        if (pressure_abs_3 > mmax) mmax = pressure_abs_3;
        if (pressure_abs_3 < mmin) mmin = pressure_abs_3;
      }
      if ((i & (1 << 0)) != 0 && bounds_check(pressure_abs_4)) { // sensor 4 allowed
        if (pressure_abs_4 > mmax) mmax = pressure_abs_4;
        if (pressure_abs_4 < mmin) mmin = pressure_abs_4;
      }
      diff = mmax - mmin;
      if (diff <= 7) { // all within 5 hectopascals of each other as a first check
        if (cnt > mxnum || (cnt == mxnum && diff < mxdiff)) {
          mxnum = cnt;
          mxdiff = diff;
          includeOne = (i & (1 << 3)) != 0;
          includeTwo = (i & (1 << 2)) != 0;
          includeThree = (i & (1 << 1)) != 0;
          includeFour = (i & (1 << 0)) != 0;
        }
      }
    }
  }

  // Move further below!! Or do it again later
  if (mxnum < 2) { // shit's fucked - let's only use what satcomms told us
    includeOne = false;
    includeTwo = false;
    includeThree = false;
    includeFour = false;
    includeFive = true;
  }

  /* ALTITUDE CALCULATIONS */
  rawDAltOne = altitude_calc(pressure_abs_1, pressure_baseline);
  rawDAltTwo = altitude_calc(pressure_abs_2, pressure_baseline);
  rawDAltThree = altitude_calc(pressure_abs_3, pressure_baseline);
  rawDAltFour = altitude_calc(pressure_abs_4, pressure_baseline);
  //rawDAltFive = altitude_calc(pressure_abs_5, pressure_baseline);


  /* ERROR HANDLING */
  if (includeOne) includeOne = errorHandleAltitude(rawDAltOne, lastDAltOne, DAltErrOne, lastSensorOneSeconds);
  if (includeTwo) includeTwo = errorHandleAltitude(rawDAltTwo, lastDAltTwo, DAltErrTwo, lastSensorTwoSeconds);
  if (includeThree) includeThree = errorHandleAltitude(rawDAltThree, lastDAltThree, DAltErrThree, lastSensorThreeSeconds);
  if (includeFour) includeFour = errorHandleAltitude(rawDAltFour, lastDAltFour, DAltErrFour, lastSensorFourSeconds);
//  rawIncludeFive = errorHandleAltitude(rawDAltFive, lastDAlt, DAltErrFive, lastSensorFiveSeconds);;
//  if (includeFive) includeFive = rawIncludeFive;


  if (!includeOne) sensor1rej++;
  if (!includeTwo) sensor2rej++;
  if (!includeThree) sensor3rej++;
  if (!includeFour) sensor4rej++;

  deltaIncentive = 0.0;

  float sataccepted = num[activeSensors];
  float actual = ((int) includeOne) + ((int) includeTwo) + ((int) includeThree) + ((int) includeFour);
  /*Serial.println("DELTA DELTA DELTA: ");
  Serial.println(sataccepted-actual);
  Serial.println(mxnum);
  Serial.println(mxdiff);
  Serial.println();*/
  deltaIncentive = max(0, sataccepted - actual);


  if (!includeOne && !includeTwo && !includeThree && !includeFour && !includeFive) { // last resort: do what satcomms tells us to
    includeOne = activeSensors & 8;
    includeTwo = activeSensors & 4;
    includeThree = activeSensors & 2;
    includeFour = activeSensors & 1;
  }

  float tempDAlt = 0.0;
  double altCnt = 0.0;
  if (includeOne) { // accepting one, woo!
    //Serial.println("Including one");
    altCnt++;
    tempDAlt += rawDAltOne;
    lastDAltOne = rawDAltOne;
    lastSensorOneSeconds = ((double)millis()) / 1000.;
  } else {
    Serial.print("rej1: ");
    Serial.println(rawDAltOne);
  }
  if (includeTwo) {
    //Serial.println("Including two");
    altCnt++;
    tempDAlt += rawDAltTwo;
    lastDAltTwo = rawDAltTwo;
    lastSensorTwoSeconds = ((double)millis()) / 1000.;
  } else {
    Serial.print("rej2: ");
    Serial.println(rawDAltTwo);
  }
  if (includeThree) {
    //Serial.println("Including three");
    altCnt++;
    tempDAlt += rawDAltThree;
    lastDAltThree = rawDAltThree;
    lastSensorThreeSeconds = ((double)millis()) / 1000.;
  } else {
    Serial.print("rej3: ");
    Serial.println(rawDAltThree);
  }
  if (includeFour) {
    //Serial.println("Including four");
    altCnt++;
    tempDAlt += rawDAltFour;
    lastDAltFour = rawDAltFour;
    lastSensorFourSeconds = ((double)millis()) / 1000.;
  } else {
    Serial.print("rej4: ");
    Serial.println(rawDAltFour);
  }

  if (includeFive) {
    Serial.println("Jesus has taken over the wheel.");
    altCnt++;
    tempDAlt += rawDAltFive;
  }
//  if (rawIncludeFive) {
//    lastDAltFive = rawDAltFive;
//    lastSensorFiveSeconds = ((double)millis()) / 1000.;
//  }
  lastDAlt = DAlt;
  DAlt = tempDAlt / altCnt;


  doTheKalmanThing();
  lastDAltSeconds = ((double)millis()) / 1000.;

  if (rawDAltOne > sensor1max) sensor1max = rawDAltOne;
  if (rawDAltOne < sensor1min) sensor1min = rawDAltOne;

  if (rawDAltTwo > sensor2max) sensor2max = rawDAltTwo;
  if (rawDAltTwo < sensor2min) sensor2min = rawDAltTwo;

  if (rawDAltThree > sensor3max) sensor3max = rawDAltThree;
  if (rawDAltThree < sensor3min) sensor3min = rawDAltThree;

  if (rawDAltFour > sensor4max) sensor4max = rawDAltFour;
  if (rawDAltFour < sensor4min) sensor4min = rawDAltFour;


  /* BATTERY VOLTAGE */
  batteryVoltage = (double)analogRead(V_BATT) * 1.2 * 4.0 / (double)pow(2, 12);

  /* CURRENT SENSOR */
  currentMonitor = (double)analogRead(BATT_CURRENT) / (double)pow(2, 12) * 1.2 * 4.0 / 0.496; // Full range is 5A, scales linearly
  externalCurrentMonitor = (double)analogRead(EXTERNAL_CURRENT) / (double)pow(2, 12) * 1.2 * 4.0 / 0.496;
  if (currentMonitor >= currentMax) currentMax = currentMonitor;
  if (currentMonitor <= currentMin) currentMin = currentMonitor;
  currentAvg += currentMonitor * 1000.0;
  currentNum += 1;

  float currentGPS = readCurrent(1);
  float currentRB = readCurrent(2);
  float currentBal = readCurrent(3);

  if (currentGPS > currentGPSMax) currentGPSMax = currentGPS;
  if (currentRB > currentRBMax) currentRBMax = currentRB;

  if (ballastState) {
    if (currentBal > currentBalMax) currentBalMax = currentBal;
    currentBalAvg += currentBal;
    currentBalNum++;
  }

  currentGPSAvg += currentGPS;
  currentRBAvg += currentRB;

  curcurbuf[curcurnum % 100] = currentMonitor * 1000;
  curcurnum += 1;

  /* BATTERY CAPACITY */
  batteryCapacity -= currentMonitor * 1000.0 * batteryVoltage * (elapsedSeconds + overflowSeconds) / 60.0 / 60.0;
  RBjoules += currentMonitor * batteryVoltage * (elapsedSeconds + overflowSeconds);


  if (!includeOne && !includeTwo && !includeThree && !includeFour && includeFive) {
    DAlt = rawDAltFive;
    lastSensorFiveSeconds = ((double)millis()) / 1000.0;
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


float altitude_calc(float P, float P0) {
  return (44330.0 * (1 - pow(P / P0, 1 / 5.255)));
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


boolean errorHandleAltitude(float rawDAlt, float lastalt, float& DAltErr, double lastTime) {
  double elapsedTimee = ((double)millis()) / 1000. - lastTime;
  DAltErr = (fabs(rawDAlt - lastalt) - 3 * BMP_ALTITUDE_NOISE_STANDARD_DEVIATION) / (BMP_ALTITUDE_MAX_VELOCITY * elapsedTimee);
  if (rawDAlt == 0.0 || rawDAlt > ERROR_MAX_VALUES[3] || rawDAlt < ERROR_MIN_VALUES[3] || DAltErr > 1.0) {
    //Serial.println("rejected.");
    return false;
  } else {
    //Serial.println("accepted.");
    return true;
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

static void smartdelay2(unsigned long ms) {
  unsigned long startt = millis();
  do
  {
    while (hsGPS.available())
      tinygps.encode(hsGPS.read());
  } while (millis() - startt < ms);
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void acquireGPSLock() {
  if (minutes > gpsBeaconInterval) {
    smartdelay2(GPS_ACQUISITION_TIME_2);
    age1 = 0;
    lati = tinygps.location.lat();
    longi = tinygps.location.lng();
    alt = tinygps.altitude.meters();
    gpsBeaconInterval += GPS_BEACON_INTERVAL;
  }
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void iteratePIDController() {
  internalSetpoint = INTERNAL_TEMP_SETPOINT;
  internalTemperature = round(internalTemperature);
  internalPID.Compute();
  if (internalOutput != 0) {
    if (!strongHeaterOn && !weakHeaterOn){
      analogWrite(HEATER_INTERNAL_STRONG, 0);
      analogWrite(HEATER_INTERNAL_WEAK, 0);
    } else if (!strongHeaterOn && weakHeaterOn){
      analogWrite(HEATER_INTERNAL_STRONG, 0);
      analogWrite(HEATER_INTERNAL_WEAK, internalOutput/2 + 127.5);
    } else if (strongHeaterOn && !weakHeaterOn) {
      analogWrite(HEATER_INTERNAL_STRONG, internalOutput/2 + 127.5);
      analogWrite(HEATER_INTERNAL_WEAK, 0);
    } else if (strongHeaterOn && weakHeaterOn){
      analogWrite(HEATER_INTERNAL_STRONG, internalOutput/2 + 127.5);
      analogWrite(HEATER_INTERNAL_WEAK, internalOutput/2 + 127.5);
    }
    if (strongHeaterOn) RBheatJ += batteryVoltage * batteryVoltage * (elapsedSeconds + overflowSeconds) / 5.; // V^2/R * dt
    if (weakHeaterOn) RBheatJ += batteryVoltage * batteryVoltage * (elapsedSeconds + overflowSeconds) / 10.; // V^2/R * dt
  } else {
    analogWrite(HEATER_INTERNAL_STRONG, 0);
    analogWrite(HEATER_INTERNAL_WEAK, 0);
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void getHeaterDutyCycles() {
  if (heaterPWMCounter > PWM_HEATER_BUFFER_SIZE - 1.0) {
    heaterPWMCounter = 0;
    heaterBufferFull = true;
  }
  if ((heaterPWMCounter <= PWM_HEATER_BUFFER_SIZE - 1.0)) {
    mainHeaterBuffer[heaterPWMCounter] = internalOutput;
    heaterPWMCounter++;
  }
  if (heaterBufferFull) {
    float mainn = 0.0;
    for (int i = 0; i < (int)PWM_HEATER_BUFFER_SIZE; i++) {
      mainn += mainHeaterBuffer[i];
    }
    mainDutyCycle = (mainn / PWM_HEATER_BUFFER_SIZE / 255.0);
  }
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void setupSDCard() {
  Serial.println("Card Initialitzed");
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i / 10 + '0';
    filename[7] = i % 10 + '0';
    if (! SD.exists(filename)) {   // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE);
      break;  // leave the loop!
    }
  }
  if (!logfile) {
    Serial.println ("ERROR: COULD NOT CREATE FILE");
  } else {
    Serial.print("Logging to: "); Serial.println(filename);
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void writeValveToEEPROM() {
  int b = altitudeSinceLastVent;
  for (int i = 4; i >= 0; i--) {
    int r = b % 10;
    b /= 10;
    EEPROM.write(i, r);
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void writeBallastToEEPROM() {
  int b = altitudeSinceLastDrop;
  for (int i = 9; i >= 5; i--) {
    int r = b % 10;
    b /= 10;
    EEPROM.write(i, r);
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void readValveFromEEPROMandClear() {
  int a = 0;
  for (int i = 0; i < 5; i++) {
    if (EEPROM.read(0) == 8) break;
    int r = EEPROM.read(i);
    a *= 10;
    a += r;
  }
  Serial.println();
  Serial.println(a);
  Serial.println();
  if (a != 0 && a != altitudeSinceLastVent) {
    altitudeSinceLastVent = a;
  }
  Serial.println();
  Serial.println(altitudeSinceLastVent);
  Serial.println();
  for (int i = 0; i < 5; i++) {
    EEPROM.write(i, 8);
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void readBallastFromEEPROMandClear() {
  int a = 0;
  for (int i = 5; i < 10; i++) {
    if (EEPROM.read(0) == 8) break;
    int r = EEPROM.read(i);
    a *= 10;
    a += r;
  }
  Serial.println();
  Serial.println(a);
  Serial.println();
  if (a != 0 && a != altitudeSinceLastDrop) {
    altitudeSinceLastDrop = a;
    firstBallastDrop = true;
  }
  Serial.println();
  Serial.println(altitudeSinceLastDrop);
  Serial.println();
  for (int i = 5; i < 10; i++) {
    EEPROM.write(i, 8);
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void init_sensors() {
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
  Wire.setDefaultTimeout(5 * 1000);
  sensor_1.begin();
  sensor_2.begin();
  sensor_3.begin();
  sensor_4.begin();
  //sensor_i2c.initializeMS_5803();
  //sensor_i2c.readSensor();
  Serial.println(sensor_1.readPressure());
  Serial.println(sensor_2.readPressure());
  Serial.println(sensor_3.readPressure());
  Serial.println(sensor_4.readPressure());
  //Serial.println(sensor_i2c.pressure());
  pressure_baseline = 1013.25; //(sensor_1.readPressure()/100.0 + sensor_2.readPressure()/100.0 + sensor_3.readPressure()/100.0 + sensor_4.readPressure()/100.0) / 4.0;
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void printLogfileHeaders() {
  logfile.print("Min, ");
  logfile.print("PID, ");
  logfile.print("NckTmp, ");
  logfile.print("IBatt, ");
  logfile.print("IExt, ");
  logfile.print("Temp, ");
  logfile.print("Raw DAlt1, ");
  logfile.print("Raw DAlt2, ");
  logfile.print("Raw DAlt3, ");
  logfile.print("Raw DAlt4, ");
  logfile.print("Raw DAlt5, ");
  logfile.print("AltErr1, ");
  logfile.print("AltErr2, ");
  logfile.print("AltErr3, ");
  logfile.print("AltErr4, ");
  logfile.print("AltErr5, ");
  logfile.print("Inc1, ");
  logfile.print("Inc2, ");
  logfile.print("Inc3, ");
  logfile.print("Inc4, ");
  logfile.print("DAlt, ");
  logfile.print("LnchP, ");
  logfile.print("AvgAscentRate, ");
  logfile.print("Vent#, ");
  logfile.print("Bal#, ");
  logfile.print("ValInct, ");
  logfile.print("BalInct, ");
  logfile.print("ValStat, ");
  logfile.print("BatV, ");
  logfile.print("RBRX, ");
  logfile.print("RBTX, ");
  logfile.print("GPS Alt, ");
  logfile.print("Lat, ");
  logfile.print("Lng, ");
  logfile.print("LatErr, ");
  logfile.print("LngErr, ");
  logfile.print("RawLat, ");
  logfile.print("RawLng, ");
  logfile.print("Cutdown, ");
  logfile.print("Min_I, ");
  logfile.print("Max_I, ");
  logfile.print("LstVnt, ");
  logfile.print("LstDrp, ");
  logfile.print("VlvStpt, ");
  logfile.print("BalStpt, ");
  logfile.print("TmpStpt, ");
  logfile.print("ArmAlt, ");
  logfile.print("VlvC1, ");
  logfile.print("VlvC2, ");
  logfile.print("VlvC3, ");
  logfile.print("BalC1, ");
  logfile.print("BalC2, ");
  logfile.print("BalC3, ");
  logfile.print("CommInt, ");
  logfile.print("GPSInt, ");
  logfile.print("IridVntTim, ");
  logfile.print("IridDrpTim, ");
  logfile.println("LoopTim");
  logfile.flush();
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void printToSerialAndLog() {
  logfile.print(minutes,6);
  logfile.print(",");
  logfile.print(internalOutput);
  logfile.print(",");
  logfile.print(neckTemperature);
  logfile.print(",");
  logfile.print(currentMonitor * 1000);
  logfile.print(",");
  logfile.print(externalCurrentMonitor * 1000);
  logfile.print(",");
  logfile.print(internalTemperature);
  logfile.print(",");
  logfile.print(rawDAltOne);
  logfile.print(",");
  logfile.print(rawDAltTwo);
  logfile.print(",");
  logfile.print(rawDAltThree);
  logfile.print(",");
  logfile.print(rawDAltFour);
  logfile.print(",");
  logfile.print(DAltErrOne);
  logfile.print(",");
  logfile.print(DAltErrTwo);
  logfile.print(",");
  logfile.print(DAltErrThree);
  logfile.print(",");
  logfile.print(DAltErrFour);
  logfile.print(",");
  logfile.print(DAltErrFive);
  logfile.print(",");
  logfile.print(includeOne);
  logfile.print(",");
  logfile.print(includeTwo);
  logfile.print(",");
  logfile.print(includeThree);
  logfile.print(",");
  logfile.print(includeFour);
  logfile.print(",");
  logfile.print(DAlt);
  logfile.print(",");
  logfile.print(pressure_baseline);
  logfile.print(",");
  logfile.print(averageAscentRate);
  logfile.print(",");
  logfile.print(ventCount);
  logfile.print(",");
  logfile.print(ballastCount);
  logfile.print(",");
  logfile.print(valveIncentive);
  logfile.print(",");
  logfile.print(ballastIncentive);
  logfile.print(",");
  logfile.print(isValveOpen);
  logfile.print(",");
  logfile.print(batteryVoltage);
  logfile.print(",");
  logfile.print(messageReceived);
  logfile.print(",");
  logfile.print(messagesSent);
  logfile.print(",");
  logfile.print(alt);
  logfile.print(",");
  logfile.print(lati);
  logfile.print(",");
  logfile.print(longi);
  logfile.print(",");
  logfile.print(gpsLatErr);
  logfile.print(",");
  logfile.print(gpsLongErr);
  logfile.print(",");
  logfile.print(rawGpsLat);
  logfile.print(",");
  logfile.print(rawGpsLong);
  logfile.print(",");
  logfile.print(cutdownReason);
  logfile.print(",");
  logfile.print(currentMin);
  logfile.print(",");
  logfile.print(currentMax);
  logfile.print(",");
  logfile.print(altitudeSinceLastVent);
  logfile.print(",");
  logfile.print(altitudeSinceLastDrop);
  logfile.print(",");
  logfile.print(ALTITUDE_SETPOINT);
  logfile.print(",");
  logfile.print(BALLAST_ALTITUDE_SETPOINT);
  logfile.print(",");
  logfile.print(INTERNAL_TEMP_SETPOINT);
  logfile.print(",");
  logfile.print(ALTITUDE_CHANGE_BALLAST);
  logfile.print(",");
  logfile.print(SPEED_CONSTANT);
  logfile.print(",");
  logfile.print(ALTITUDE_DIFFERENCE_CONSTANT);
  logfile.print(",");
  logfile.print(LAST_VALVE_OPENING_CONSTANT);
  logfile.print(",");
  logfile.print(BALLASTER_SPEED_CONSTANT);
  logfile.print(",");
  logfile.print(BALLASTER_ALT_DIFF_CONSTANT);
  logfile.print(",");
  logfile.print(BALLASTER_LAST_DROP_CONSTANT);
  logfile.print(",");
  logfile.print(COMM_BEACON_INTERVAL);
  logfile.print(",");
  logfile.print(GPS_BEACON_INTERVAL);
  logfile.print(",");
  logfile.print(IridiumVentTime);
  logfile.print(",");
  logfile.print(IridiumBallastTime);
  logfile.print(",");
  logfile.println((elapsedSeconds + overflowSeconds) * 1000.0);
  logfile.flush();
  
  Serial.print("External Temperature: ");
  Serial.print(neckTemperature);
  Serial.print("Primary: ");
  Serial.print(batteryVoltage);
  Serial.print(", Batt Current: ");
  Serial.print(currentMonitor * 1000);
  Serial.print(", Ext Current: ");
  Serial.print(externalCurrentMonitor * 1000);
  Serial.print(", I1: ");
  Serial.print(includeOne);
  Serial.print(", I2: ");
  Serial.print(includeTwo);
  Serial.print(", I3: ");
  Serial.print(includeThree);
  Serial.print(", I4: ");
  Serial.print(includeFour);
  Serial.print(", DAlt1: ");
  Serial.print(rawDAltOne);
  Serial.print(", DAlt2: ");
  Serial.print(rawDAltTwo);
  Serial.print(", DAlt3: ");
  Serial.print(rawDAltThree);
  Serial.print(", DAlt4: ");
  Serial.print(rawDAltFour);
  Serial.print(", AvgAscentRate: ");
  Serial.print(averageAscentRate);
  Serial.print(", TmpAscentRate: ");
  Serial.print(tempAscentRate);
  Serial.print(", A1: ");
  Serial.print(averageAscentRate1);
  Serial.print(", A2: ");
  Serial.print(averageAscentRate2);
  Serial.print(", A3: ");
  Serial.print(averageAscentRate3);
  Serial.print(", A4: ");
  Serial.print(averageAscentRate4);
  //Serial.println();
  Serial.print(", Loop time: ");
  Serial.println((elapsedSeconds + overflowSeconds) * 1000);
  Serial.println();
  Serial.println();

  Serial.print("[Current/mA: Avg (max)] ");
  Serial.print("GPS: ");
  Serial.print(currentGPSAvg);
  Serial.print(" (");
  Serial.print(currentGPSMax);
  Serial.print(" ) ");
  Serial.print("RB: ");
  Serial.print(currentRBAvg);
  Serial.print(" (");
  Serial.print(currentRBMax);
  Serial.print(" ) ");
  Serial.print("Val: ");
  Serial.print(currentValAvg);
  Serial.print(" (");
  Serial.print(currentValMax);
  Serial.print(" ) ");
  Serial.print("Bal: ");
  Serial.print(currentBalAvg);
  Serial.print(" (");
  Serial.print(currentBalMax);
  Serial.println(" )");

  Serial.print("Min: ");
  Serial.print(minutes);
  Serial.print(", Setpoint: ");
  Serial.print(ALTITUDE_SETPOINT);
  Serial.print(", PID_I: ");
  Serial.print(internalOutput);
  Serial.print(", SrvoPos: ");
  Serial.print("");
  Serial.print(", Curr: ");
  Serial.print(currentMonitor * 1000);
  Serial.print(", Curr avg:");
  Serial.print(currentAvg / currentNum);
  Serial.print(", I_Temp: ");
  Serial.print(internalTemperature);
  Serial.println();
  Serial.print(", DAlt: ");
  Serial.print(DAlt);
  Serial.print(", DAlt1: ");
  Serial.print(rawDAltOne);
  Serial.print(", DAlt2: ");
  Serial.print(rawDAltTwo);
  Serial.print(", DAlt3: ");
  Serial.print(rawDAltThree);
  Serial.print(", DAlt4: ");
  Serial.print(rawDAltFour);
  /*Serial.print(", AltErr1: ");
  Serial.print(DAltErrOne);
  Serial.print(", AltErr2: ");
  Serial.print(DAltErrTwo);
  Serial.print(", AltErr3: ");
  Serial.print(DAltErrThree);
  Serial.print(", AltErr4: ");
  Serial.print(DAltErrFour);*/
  //Serial.print(", DAlt: ");
  //Serial.print(DAlt);
  //Serial.print(", LnchLocPrs: ");
  //Serial.print(pressure_baseline);
  Serial.print(", AvgAscentRate: ");
  Serial.print(tempAscentRate);
  Serial.print(", A1: ");
  Serial.print(averageAscentRate1);
  Serial.print(", A2: ");
  Serial.print(averageAscentRate2);
  Serial.print(", A3: ");
  Serial.print(averageAscentRate3);
  Serial.print(", A4: ");
  Serial.print(averageAscentRate4);
  Serial.println();
  Serial.print(", Vent#: ");
  Serial.print(ventCount);
  Serial.print(", Bal#: ");
  Serial.print(ballastCount);
  Serial.print(", VIncent: ");
  Serial.print(valveIncentive);
  Serial.print(", BIncent: ");
  Serial.print(ballastIncentive);
  Serial.print(", Valve State: ");
  Serial.print(isValveOpen);
  Serial.print(", BatV: ");
  Serial.print(batteryVoltage);
  Serial.println();
  //Serial.print(", RBRX: ");
  //Serial.print(messageReceived);
  //Serial.print(", RBTX: ");
  //Serial.print(messagesSent);
  Serial.print(", GPS Alt: ");
  Serial.print(alt);
  Serial.print(", GPS Lat: ");
  Serial.println(lati, 6);
  Serial.print(", GPS Long: ");
  Serial.print(longi, 6);
  Serial.print(", LatErr: ");
  Serial.print(gpsLatErr);
  Serial.print(", LongErr: ");
  Serial.print(gpsLongErr);
  Serial.print(", Raw GPS Lat: ");
  Serial.print(rawGpsLat, 6);
  Serial.print(", Raw GPS Long: ");
  Serial.print(rawGpsLong, 6);
  Serial.print(", Cutdown: ");
  Serial.print(cutdownReason);
  Serial.print(", LstVnt: ");
  Serial.print(altitudeSinceLastVent);
  Serial.print(", LstDrp: ");
  Serial.print(altitudeSinceLastDrop);
  Serial.print(", ITempStp: ");
  Serial.println(INTERNAL_TEMP_SETPOINT);
  Serial.print(", Min Current: ");
  Serial.print(currentMin);
  Serial.print(", Max Current: ");
  Serial.print(currentMax);
  Serial.print(", Loop time: ");
  Serial.println((elapsedSeconds + overflowSeconds) * 1000);
  Serial.println("Power States");
  for (int j = 0; j < 3; j++) {
    Serial.println(powerStates[j]);
  } Serial.println();
  for (int i = 0; i < NUM_KALMANS; i++) {
    Serial.print(kalmanAltitudes[i]);
    Serial.print(" ");
  }
  Serial.println();
  Serial.println(initAltitudeTime);
  Serial.println(QcurrentDt);
  Serial.println(QmulFactor);
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


bool ISBDCallback() {
  if (!inSetup) {
    loopStartTime = millis();
    iterateSensors();
    if (powerStates[2] == 2) {
      iteratePIDController();
      getHeaterDutyCycles();
    }
    if (powerStates[1] == 2) acquireGPSLock();
    printToSerialAndLog();
    elapsedSeconds = (double)(((double)millis() - loopStartTime) / 1000.0);
    getAscentRates();
    altitudeController();
    blinkLED();
    int derray = (int)(50.0 - (double)((double)millis() - loopStartTime));
    if (derray > 0) delay(derray);
    overflowSeconds = (double)(((double)millis() - loopStartTime) / 1000.0) - elapsedSeconds;
    minutes += (double)(((double)millis() - loopStartTime) / 1000.0 / 60.0);
  }
  return true;
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


void init_log() {
  log_variable(&RBmsg, 0, 8191, 13); // message | 1
  log_variable(&RBvel, -10, 10, 11); // ascent rate
  log_variable(&RBaltB, -2000, 40000, 16); // altitude barometer | 2
  log_variable(&RBaltG, -2000, 40000, 16); // altitude gps | 2
  log_variable(&RBlat, -90, 90, 21); // latitude
  log_variable(&RBlong, -180, 180, 22); // longitude
  log_variable(&RBtemp, -50, 100, 9); // temperature | 1
  log_variable(&RBjoules, 0, 1500000, 18); // joules expended
  log_variable(&RBvoltP, 0, 5, 9); // primary voltage
  log_variable(&RBavgI, 0, 5000, 8); // average current
  log_variable(&RBmaxI, 0, 5000, 8); // max current
  log_variable(&RBminI, 0, 5000, 8); // min current
  log_variable(&RBVin, -50, 10, 12); // valve incentive
  log_variable(&RBBin, -50, 10, 12); // ballast incentive
  log_variable(&RBcutdown, 0, 1, 1); // cutdown state
  log_variable(&RBBtime, 0, 8191, 12); // ballast time
  log_variable(&RBVtime, 0, 8191, 12); // valve time
  log_variable(&RBheatJ, 0, 200000, 16); // heaters
  log_variable(&RBs1delta, 0, 2000, 9); // sensor 1 delta
  log_variable(&RBs2delta, 0, 2000, 9); // sensor 2 delta
  log_variable(&RBs3delta, 0, 2000, 9); // sensor 3 delta
  log_variable(&RBs4delta, 0, 2000, 9); // sensor 4 delta
  log_variable(&RBs1rej, 0, 6, 4); // sensor 1 logrejections
  log_variable(&RBs2rej, 0, 6, 4); // sensor 2 logrejections
  log_variable(&RBs3rej, 0, 6, 4); // sensor 3 logrejections
  log_variable(&RBs4rej, 0, 6, 4); // sensor 4 logrejections
  log_variable(&RBmode, 0, 3, 2); // flight mode
  log_variable(&RBt0, 0, 1000000, 19); // initial time
  log_variable(&RBsensors, 0, 15, 4); // active sensors
  log_variable(&RBLEDon, 0, 1, 1); // LED on
  log_variable(&RBpowerGPS, 0, 3, 2); // GPS power state
  log_variable(&RBpowerRB, 0, 3, 2); // RB power state
  log_variable(&RBpowerHeaters, 0, 3, 2); // Heaters power state
  log_variable(&RBcurrentGPSmax, 0, 250, 6); // GPS current max
  log_variable(&RBcurrentGPSavg, 0, 250, 6); // GPS current avg
  log_variable(&RBcurrentRBmax, 0, 1000, 6); // RB current max
  log_variable(&RBcurrentRBmax, 0, 1000, 6); // RB current avg
  log_variable(&RBcurrentValmax, 0, 500, 6); // Val current max
  log_variable(&RBcurrentValavg, 0, 500, 6); // Val current avg
  log_variable(&RBcurrentBalmax, 0, 500, 6); // Bal current max
  log_variable(&RBcurrentBalavg, 0, 500, 6); // Bal current avg
  log_variable(&RBheightOffset, -2000, 30000, 16); // Compressed height offset
  log_variable(&RBheightScale, 0, 1500, 12); // Compressed height scale
  log_variable(&RBtimeFactor, 0, 7, 3); // Compressed time factor
  log_variable(&RBtimeInterval, 0, 15, 4); // Compressed base interval
  log_variable(&RBStrongHeat, 0, 1, 1); // Super roasty toasty on
  log_variable(&RBMildHeat, 0, 1, 1); // Mildly roasty toasty on

  debugThreshold = argc;
  log_variable(&RBVatt, 0, 4095, 12); // valve attempts
  log_variable(&RBBatt, 0, 4095, 12); // ballast attempts
  log_variable(&RBatt1type, 0, 1, 1); // attempt 1 type
  log_variable(&RBatt1time, -5, 600, 9); // attempt 1 time
  log_variable(&RBatt2type, 0, 1, 1); // attempt 2 type
  log_variable(&RBatt2time, -5, 600, 9); // attempt 2 time
  log_variable(&RBatt3type, 0, 1, 1); // attempt 3 type
  log_variable(&RBatt3time, -5, 600, 9); // attempt 3 time
  log_variable(&RBatt4type, 0, 1, 1); // attempt 4 type
  log_variable(&RBatt4time, -5, 600, 9); // attempt 4 time
  //log_variable(&RBpotReading, 0, 4095, 12); // potentiometer reading
  log_variable(&RBNeckTemp, -90, 50, 12); // Balloon Temperature
  log_variable(&RBmaxTime, 0, 255, 8); // Compressed max time
  log_variable(&RBaltDiff, -2000, 2000, 13); // Compressed alt diff
  log_variable(&RBreadback, 0, 1, 1); // Have readback

  // WEEP
  log_variable(&RBCValveSpeed, 0, 5, 8); // Valve speed constant
  log_variable(&RBCValveAltDiff, 0, 4000, 8); // Valve alt diff constant
  log_variable(&RBCValveLast, 0, 4000, 8); // Valve last vent constant
  log_variable(&RBCBallastSpeed, 0, 5, 8); // Ballast speed constant
  log_variable(&RBCBallastAltDiff, 0, 4000, 8); // Ballast alt diff constant
  log_variable(&RBCBallastLast, 0, 4000, 8); // Ballast last ballast constant
  log_variable(&RBCValveSetpoint, -2000, 50000, 11); // Valve setpoint
  log_variable(&RBCBallastSetpoint, -2000, 50000, 11); // Ballast setpoint
  log_variable(&RBFValveLast, -2000, 50000, 11); // Last vent height
  log_variable(&RBFDropLast, -2000, 50000, 11); // Last drop height
  log_variable(&RBCOpenValve, 0, 4095, 12); // Open valve setting
  log_variable(&RBCClosedValve, 0, 4095, 12); // Closed valve setting
  log_variable(&RBCCutdownValve, 0, 4095, 12); // Cutdown valve setting
  log_variable(&RBCMaxEncoder, 0, 50, 5); // Max time without encoder
  log_variable(&RBCDoNothing, 0, 50, 5); // Do nothing timer
  log_variable(&RBCValTime, 0, 50, 6); // Time per vent
  log_variable(&RBCBalTime, 0, 50, 6); // Time per ballast
  log_variable(&RBControlMode, 0, 1, 1); // Control mode enabled
  log_variable(&RBReportMode, 0, 1, 1); // Report mode enabled
  log_variable(&RBPotMode, 0, 1, 1); // Pot mode type
  log_variable(&RBTemperatureSetpoint, -20, 40, 6); // Temperature setpoint
  log_variable(&RBRBinterval, 0, 20, 5); // RB interval
  log_variable(&RBGPSinterval, 0, 2, 7); // GPS interval
  log_variable(&RBValveSpeed, 0, 255, 8); // Valve speed setting
  log_variable(&RBOpeningTime, 0, 2000, 8); // Valve opening time setting
  log_variable(&RBIthresh, 0, 4, 8); // Controller I thresh
  log_variable(&RBreArmConst, 0, 4, 8); // Controller I thresh
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void allPowerOff() {
  digitalWrite(LED_PIN, LOW);
  analogWrite(HEATER_INTERNAL_STRONG, 0);
  analogWrite(HEATER_INTERNAL_WEAK, 0);
  digitalWrite(GPS_ENABLE, LOW);
  digitalWrite(RB_GATE, LOW);
  digitalWrite(PAYLOAD_GATE,LOW);
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


void startupRockblock() {
  if (powerStates[0] == 0) {
    powerStates[0] = 1;
    EEPROM.write(10, 1);
    digitalWrite(RB_GATE,HIGH);
    delay(1000);
    isbd.begin();
    powerStates[0] = 2;
    EEPROM.write(10, 2);
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void startupGPS() {
  if (powerStates[1] == 0) {
    powerStates[1] = 1;
    EEPROM.write(11, 1);
    digitalWrite(GPS_ENABLE, HIGH);
    powerStates[1] = 2;
    EEPROM.write(11, 2);
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void startupHeater() {
  if (powerStates[2] == 0) {
    powerStates[2] = 1;
    EEPROM.write(12, 1);
    analogWrite(HEATER_INTERNAL_STRONG, 255);
    analogWrite(HEATER_INTERNAL_WEAK, 255);
    delay(2000);
    analogWrite(HEATER_INTERNAL_STRONG, 0);
    analogWrite(HEATER_INTERNAL_WEAK, 0);
    powerStates[2] = 2;
    EEPROM.write(12, 2);
  }
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void readPowerStatesFromEEPROMandParse() {
  for (int i = 10; i < 13; i++) {
    powerStates[i - 10] = EEPROM.read(i);
    if (powerStates[i - 10] == 2) {
      powerStates[i - 10] = 0;
      EEPROM.write(i, 0);
    }
    if (powerStates[i - 10] == 1) {
      powerStates[i - 10] = 3;
      EEPROM.write(i, 3);
    }
  }
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    hsGPS.write(MSG[i]);
    Serial.print(MSG[i], HEX);
  }
  hsGPS.println();
}
 
 //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  Serial.print(" * Reading ACK response: ");
 
  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;	// header
  ackPacket[1] = 0x62;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];	// ACK class
  ackPacket[7] = MSG[3];	// ACK id
  ackPacket[8] = 0;		// CK_A
  ackPacket[9] = 0;		// CK_B
 
  // Calculate the checksums
  for (uint8_t i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
 
  while (1) {
 
    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      Serial.println(" (SUCCESS!)");
      return true;
    }
 
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      Serial.println(" (FAILED!)");
      return false;
    }
 
    // Make sure data is available to read
    if (hsGPS.available()) {
      b = hsGPS.read();
 
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
        Serial.print(b, HEX);
      } 
      else {
        ackByteID = 0;	// Reset and look again, invalid order
      }
 
    }
  }
}

