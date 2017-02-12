# SSI HABEES:
###A modular flight controller with payload interface for high altitude balloons

#Flight States:
The avionics uses an Event Driven model in order to clearly transition between states of operation.

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

The avionics flight software was written in compliance with NASA JPL's  Safety-Critical Code standards.

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
2. Altitude readings from filtered and error checked BMP280 and MPL3115A2.
3. Timestamp from on-board Teensy RTC.
4. PID Heating on inboard heater trace.
5. Integration of uBlox M8Q GPS.
6. RockBlock data downlink.
7. Flight termination optionally based on altitude and GPS setpoints.

####Useful Flight Features
1. MicroSD logging of errors and notable events to log.txt.
2. External temperature reading from thermocouple.
3. Ascent rate calculations from filtered and error checked data.
4. Current readings for heaters, cutdown FET, and radio.
5. Debug LEDs for all appropriate states.
6. Fault LED if anything flight-critical is not fully functioning.
7. Debug mode disabled at altitude.
8. GPS successful set to flight mode.
9. Compression of data frame into bitstream for comms.
10. RockBlock command parsing for satcomms uplink.
11. APRS downlink with Dorji.
12. CAN communication of current state through payload interface
13. Watchdog to prevent main thread halt.
