/*
  Stanford Student Space Initiative
  Balloons | HABEES | February 2017
  Davy Ragland | dragland@stanford.edu
  Vinh Nguyen | vnguyen5@stanford.edu

  File: APRS.h
  --------------------------
  APRS API.
*/

#ifndef APRS_H
#define APRS_H
#include <Arduino.h>
#include <stdint.h>
#include <string>
#include "afsk.h"
#include <SoftwareSerial.h>
/**********************************  TODO  ************************************/
#include "../../src/Config.h"
/**********************************  TODO  ************************************/

using namespace std;

class DRA818V;
struct SSID {
    const char* address;
    uint8_t ssid_designator;
};

class APRS
{
public:
/**********************************  SETUP  ***********************************/
    APRS();
    void    setSSIDs();
    bool    init();
/********************************  FUNCTIONS  *********************************/
    int16_t sendAdditionalData(char* extData, uint16_t len);
    int16_t sendPacket(char* time, float lat, float lon, float altitude, uint16_t heading, float speed, bool debug);
    void    sendPacketNoGPS(string data);
    void    sendPacketNoGPS(char* data);
    int     getPacketSize();
    void    clearPacket();
private:
/*********************************  HELPERS  **********************************/
    void    loadHeader();
    void    loadData(uint8_t *data_buffer, uint8_t length);
    void    loadFooter();
    void    loadTrailingBits(uint8_t bitIndex);
    void    loadByte(uint8_t byte);
    void    loadBit(uint8_t bit, bool bitStuff);
    void    loadString(const char* str);
    void    loadString(const char* str, uint16_t len);
    void    loadString(string str);
    void    loadHDLCFlag();
    void    update_crc(uint8_t bit);
/*********************************  OBJECTS  **********************************/
    static const uint16_t  BUFFER_SIZE = 200;

/**********************************  TODO  ************************************/
    // uint8_t   DRA_ENABLE         =       A0;
    // uint8_t   DRA_SLEEP          =      A18;
    // uint8_t   DRA_PWR            =       A3;
    // uint8_t   DRA_TX             =      A16;
    // uint8_t   DRA_RX             =      A17;
    // uint8_t   DRA_MIC            =      A14;
    // uint8_t   DRA_PTT            =       A2;

    // uint16_t  ANALOG_RES         =       12;
    // uint16_t  ANALOG_MAX         =     4095;
    // char      MISSION_NUMBER[50];
    // char      TARGET_CALLSIGN[50];
    // char      TX_CALLSIGN[50];
    // char      DEFAULT_PATH[50];
    // uint8_t   MAX_EXTRA_DATA     =      100;
    // uint8_t   TARGET_DESIG       =        0;
    // uint8_t   TX_DESIG           =       11;
    // uint8_t   PATH_DESIG         =        1;
/**********************************  TODO  ************************************/

    DRA818V* radio;
    SSID* ssids;
    uint8_t num_ssids;
    volatile uint8_t* packet_buffer;
    int packet_size;

    static const uint8_t HDLC_FLAG = 0x7E;
    uint8_t NUM_HDLC_FLAGS = 50; //default number
    uint8_t NUM_SSIDS = 3; //default number
    static const uint8_t MAX_SSIDS = 4;
    bool USE_WIDE2_2 = false;
    static const int MAX_BUFFER_SIZE = 512; //bytes
    static const uint8_t BIT_STUFF_THRESHOLD = 5; //APRS protocol

    uint16_t crc = 0;
    uint8_t consecutiveOnes = 0;
    uint8_t bitMask= B00000000;
    uint8_t bitPos = 8;

    char extraData[BUFFER_SIZE];
    int16_t extraLen = 0;
};
#endif // APRS_H
