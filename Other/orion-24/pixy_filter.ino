#include <SPI.h>
#include <Pixy.h>
#include <EEPROMAnything.h>
#include <EEPROM.h>

/*
 * Orion PIXY Cam Logging
 *
 * @author SSI Orion
 * @date 10/06/2015
 * @version 0.1
 */

Pixy pixy;

// Printing booleans
boolean printSerial = true; // Change to false
boolean printEEPROM = true;
boolean printSD = false;
// Base EEPROM Address
unsigned int baseAddr = 0;


/*
 * Setup serial, pixy cam, variables,
 * logging.
 *
 * @author SSI Orion Team
 */
void setup() {
  Serial.begin(9600);
  delay(500);

  if (Serial.available() > 0) {
    printSerial = true;
    Serial.print("Starting pixy filtraton program.");
  }

  pixy.init();
}

/*
 * Arduino loop. Logs pixy blocks.
 *
 */
void loop() {
  static int ii = 0;
  int jj;
  uint16_t blocks;
  char buf[32];

  blocks = pixy.getBlocks();

  if (blocks) {
    ii++;
    int largestBlock[4] = {0, 0, 0, 0}; // x, y, width, height
    if (ii % 50 == 0) {
      if (printSerial) {
        Serial.println();
        sprintf(buf, "Detected %d:\n", blocks);
        Serial.print(buf);
      }
      int x = -1, y = -1, w = -1, h = -1;
      for (jj = 0; jj < blocks; jj++)
      {
        x = int(pixy.blocks[jj].x);
        y = int(pixy.blocks[jj].y);
        w = int(pixy.blocks[jj].width);
        h = int(pixy.blocks[jj].height);

        if (w * h >= largestBlock[2]*largestBlock[3]) {
          largestBlock[0] = x;
          largestBlock[1] = y;
          largestBlock[2] = w;
          largestBlock[3] = h;
        }
        if (printSerial) {
          sprintf(buf, "  block %d: ", jj); // From pixy example
          Serial.print(buf);
          pixy.blocks[jj].print(); // Only print if connected to computer
        }
      }
      // We now have a filtered largest block.
      if (printSD) {
        // Write to SD
      }
      if (printEEPROM) {
      }
      if (printSerial) {
        Serial.println("Largest block: " + String(largestBlock[0]) + ", " +
                       String(largestBlock[1]) + ", " + String(largestBlock[2]) + ", " + String(largestBlock[3]));
      }
    }
  }
  else {
    if (printSerial || printSD || printEEPROM) {
      // do nothing because this would flood console
    }
  }
}

