# SSI HABEES:
###A modular flight controller with payload interface for high altitude balloons

#Flight States:
The avionics uses an Event Driven Programming model in order to clearly transition between states of operation.

#### States
1. Pre-Launch  
 - 1a. Startup initialization
 - 1b. System verification
2. Launch
 - 2a. Liftoff
 - 2b. Flight mode
 - 2c. Apogee
 - 2d. Descent

#Code Architecture:
The avionics flight software operates on a read-eval loop in order to change states and respond to its environment.

#### Files
`main.cpp` - Start point of flight controller.

`config.h` - Mission specific configuration values.

`data.h` - Structure of current data frame.

#### Classes
`Avionics` - Implementation of flight controller.

`Sensors` - Interface to filtered data from hardware.

`Hardware` - Interface to PCB mechatronics.

#Implementation Details:
Here is the current status of the code:

####Flight Critical Systems
1. MicroSD logging of current data frame to data.txt.
2. MicroSD logging of errors and notable events to log.txt.
3. Altitude readings from filtered and error checked BMP280 pressure & temperature values and backup MPL3115A2 values.
4. External temperature from thermocouple.
5. Heating using PID on inboard heater trace.
6. Flight termination optionally based on altitude and GPS setpoints.
7. I2C current readings for heaters, Cutdown FET, and radio.
8. Debug LEDs for all appropriate states.
9. Fault LED if anything flight-critical is not fully functioning.
10. Debug mode disabled at altitude.
11. Timestamp from on-board Teensy RTC.
12. Integration of uBlox M8Q GPS.
13. GPS successful set to Flight Mode.
14. RockBlock data downlink.

####Useful Flight Features
1. Watchdog interrupts in case of main thread halt.
2. RockBlock command parsing for cutdown and transmit rate and hardware resets.
3. APRS downlink with Dorji.
4. CAN communication of current state through payload interface
