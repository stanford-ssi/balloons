#include <TinyGPS.h>

HardwareSerial hsGPS(Serial3);

/*
 * uBlox GPS initialization into flight mode. 
 * Gathers data using TinyGPS library. 
 */

TinyGPS tinygps;

/*
 * Used to set flight mode. 
 */
byte gps_set_sucess = 0 ;
uint8_t setNav[] = {
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
  0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC
};

/*
 * important GPS data
 */
float flat = -1.00, flon = -1.00, falt = -1, fmps = -1;

// other crap 
bool newData = false;
unsigned long age; 

/*
 * Setup
 */
void setup() {
  hsGPS.begin(9600); // check 4800 too
  // set it up in FLIGHT MODE
  set_flight_mode();
}

/*
 * Loop
 */
void loop() {
  delay(100); 
}

/*
 * Runs flight mode stuff. 
 *  See flight_mode.ino or https://ukhas.org.uk/guides:ublox6
 */
void set_flight_mode() {
  while (!gps_set_sucess) {
    sendUBX(setNav, sizeof(setNav) / sizeof(uint8_t));
    gps_set_sucess = getUBX_ACK(setNav);
  }
  gps_set_sucess = 0;
}

// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for (int i = 0; i < len; i++) {
    hsGPS.write(MSG[i]);
  }
  hsGPS.println();
}

// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();

  // Construct the expected ACK packet
  ackPacket[0] = 0xB5;  // header
  ackPacket[1] = 0x62;  // header
  ackPacket[2] = 0x05;  // class
  ackPacket[3] = 0x01;  // id
  ackPacket[4] = 0x02;  // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];  // ACK class
  ackPacket[7] = MSG[3];  // ACK id
  ackPacket[8] = 0;   // CK_A
  ackPacket[9] = 0;   // CK_B

  // Calculate the checksums
  for (uint8_t i = 2; i < 8; i++) {
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
        //Serial.print(b, HEX);
      }
      else {
        ackByteID = 0;  // Reset and look again, invalid order
      }
    }
  }
}
