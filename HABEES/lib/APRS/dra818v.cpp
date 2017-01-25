#include "dra818v.h"
static const char* TX_CTCSS = "0000";
static const char* RX_CTCSS = "0000";
static const float APRS_NA_FTX = 144.390; //transmission frequency in MHz (APRS 144.390 standard)
static const float APRS_NA_FRX = 144.390; //receiving frequency in MHz
static const int SOFT_SERIAL_BAUD = 9600;
static const int SERIAL_BAUD = 9600;
static const bool CHANNEL_SCAN_BW = true; //true: 25kHz, false: 12.5kHz

DRA818V::DRA818V()
{
    sleepPin = DRA_SLEEP;
    pttPin = DRA_PTT;
    radio_enable = DRA_ENABLE;
    radio_power = DRA_PWR;
    micPin = DRA_MIC;
    radioSerial = new SoftwareSerial(DRA_TX,DRA_RX);
}

bool DRA818V::init() {
    pinMode(pttPin,OUTPUT);
    pinMode(radio_enable,OUTPUT);
    pinMode(micPin,OUTPUT);
    digitalWrite(radio_enable,LOW);
    pinMode(sleepPin,OUTPUT);
    digitalWrite(DRA_SLEEP,HIGH);
    radioSerial->begin(SOFT_SERIAL_BAUD);
    if(!Serial) {
        Serial.begin(SOFT_SERIAL_BAUD);
    }
    digitalWrite(pttPin,LOW);
    delay(200);
    radioSerial->print("AT+DMOCONNECT\r\n");
    digitalWrite(pttPin,HIGH);
    delay(200);
    digitalWrite(pttPin,LOW);
    delay(200);
    configSettings();
    return true;
}

void DRA818V::configSettings() {
    digitalWrite(pttPin,LOW);
    radioSerial->print("AT+DMOSETGROUP=");
    radioSerial->print(CHANNEL_SCAN_BW,1);
    radioSerial->print(",");
    radioSerial->print(APRS_NA_FTX,4);
    radioSerial->print(",");
    radioSerial->print(APRS_NA_FRX,4);
    radioSerial->print(",");
    radioSerial->print(TX_CTCSS);
    radioSerial->print(",");
    radioSerial->print(squelch);
    radioSerial->print(",");
    radioSerial->print(RX_CTCSS);
    radioSerial->print("\r\n");
    delay(500);
    digitalWrite(pttPin,HIGH);
    delay(200);
}
