/*
  Stanford Student Space Initiative
  Balloons | HABEES | January 2017
  Davy Ragland | dragland@stanford.edu
  Vinh Nguyen | vnguyen5@stanford.edu

  File: APRS.h
  --------------------------
  APRS API.
*/

#ifndef APRS_H
#define APRS_H
#include <string>
#include "../../src/Data.h"
#include "afsk.h"
#include <SoftwareSerial.h>

using namespace std;

class DRA818V;
struct SSID {
    const char* address;
    uint8_t ssid_designator;
};

class APRS
{
public:
    APRS();
    void setSSIDs();
    bool init();
    void sendAdditionalData(const char* extData, uint16_t len);
    void sendPacket(DataFrame &dataArr);
    void sendPacketNoGPS(string data);
    void sendPacketNoGPS(char* data);
    int getPacketSize();
    void clearPacket();
private:
    void loadHeader();
    void loadData(uint8_t *data_buffer, uint8_t length);
    void loadFooter();
    void loadTrailingBits(uint8_t bitIndex);
    void loadByte(uint8_t byte);
    void loadBit(uint8_t bit, bool bitStuff);
    void loadString(const char* str);
    void loadString(const char* str, uint16_t len);
    void loadString(string str);
    void loadHDLCFlag();
    void update_crc(uint8_t bit);
    DRA818V* radio;
    SSID* ssids;
    uint8_t num_ssids;
    volatile uint8_t* packet_buffer;
    int packet_size;
};
#endif // APRS_H
