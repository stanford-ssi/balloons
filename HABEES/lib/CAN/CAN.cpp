/*
  Stanford Student Space Initiative
  Balloons | HABEES | January 2017
  Davy Ragland | dragland@stanford.edu
  Zach Belateche | zachbela@stanford.edu
  Alex Mallery | amallery@stanford.edu

  File: CAN.cpp
  --------------------------
  Implimentation of CAN.h
*/

#include "CAN.h"

/**********************************  SETUP  ***********************************/
/*
  function: init
  ---------------------------------
  This function initializes the CAN bus.
*/
bool CAN::init() {
  CANbus.begin();
  delay(1000);
  sysTimer.reset();
  return true;
}

/********************************  FUNCTIONS  *********************************/
/*
  function: write
  ---------------------------------
  This function writes a bitstream across the communication interface.
*/
int16_t CAN::write(char* buff, uint16_t len) {
  static CAN_message_t msg;

  for(size_t i = 0; i < len; i+=8) {
    for(uint8_t j = 0; j < 8; j++){
      if(i+j < len){
        msg.buf[j] = buff[i+j];
      }
      else{ msg.buf[j] = 0; }
    }
    if( CANbus.write(msg) == 0){return -1;}
  }
  return 0;
}

/*
  function: canTestLoop
  ---------------------------------
  This function calls a while(true) test loop for the CAN bus.
*/
void CAN::canTestLoop(void) {
  static CAN_message_t msg;
  static CAN_message_t rxmsg;
  int txCount;
  int rxCount;
  unsigned int txTimer;
  unsigned int rxTimer;
  while(true) {
    // service software timers based on Metro tick
    if ( sysTimer.check() ) {
      if ( txTimer ) --txTimer;
      if ( rxTimer ) --rxTimer;
    }
    // if not time-delayed, read CAN messages and print 1st byte
    if ( !rxTimer ) {
      while ( CANbus.read(rxmsg) ) {
        //hexDump( sizeof(rxmsg), (uint8_t *)&rxmsg );
        Serial.write(rxmsg.buf[0]);
        rxCount++;
      }
    }
    // insert a time delay between transmissions
    if ( !txTimer ) {
      // if frames were received, print the count
      if ( rxCount ) {
        Serial.write('=');
        Serial.print(rxCount);
        rxCount = 0;
      }
      txTimer = 100;//milliseconds
      msg.len = 8;
      msg.id = 0x222;
      for( int idx=0; idx<8; ++idx ) {
        msg.buf[idx] = '0'+idx;
      }
      // send 6 at a time to force tx buffering
      txCount = 6;
      Serial.println(".");
      while ( txCount-- ) {
        CANbus.write(msg);
        msg.buf[0]++;
      }
      // time delay to force some rx data queue use
      rxTimer = 3;//milliseconds
    }
  }
}

/*********************************  HELPERS  **********************************/
/*
  function: hexDump
  ---------------------------------
  This function dups the hex data from an input buffer.
*/
void CAN::hexDump(uint8_t dumpLen, uint8_t *bytePtr) {
  static uint8_t hex[17] = "0123456789abcdef";
  uint8_t working;
  while( dumpLen-- ) {
    working = *bytePtr++;
    Serial.write( hex[ working>>4 ] );
    Serial.write( hex[ working&15 ] );
  }
  Serial.write('\r');
  Serial.write('\n');
}
