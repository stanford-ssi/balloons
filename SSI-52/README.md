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
1. MicroSD data logging of current data frame to data.txt.
2. MicroSD data logging of errors and notable events to log.txt.
3. Altitude data from filtered and error checked BMP280 pressure & temperature values and backup MPL3115A2.
4. External temperature from Thermocouple.
5. Heating using PID on inboard heater trace.
6. Flight termination optionally based on altitude and GPS setpoints.
7. Debug LEDs for all appropriate states.
8. Fault LED if anything flight-critical is not fully functioning.
9. Debug mode disabled at altitude.
10. Timestamp from on-board Teensy RTC.
11. Integration of uBlox M8Q GPS.
12. GPS successful set to Flight Mode.
13. RockBlock data downlink.

####Useful Flight Features
1. APRS downlink with Dorji.
2. CAN communication with of current state through payload interface
3. I2C current readings for Heater Trace, Cutdown FET, and Dorji.
3. 9DOF orientation data.
4. Set appropriate bounds for current draw for each, add a buffer, and set as current limits after which the device will be hard reset.
5. Watchdog interrupts (pulse pin high for 1ms, then low) every 2-10 seconds to disable hardware reset.
6. RockBlock command parsing for cutdown and transmit rate and hardware resets.
