# SSI HABEES:
###A modular flight controler with payload interface for high altitude balloons

#Flight States:
The avionics uses an Event Driven Programming model in order to clearly transition between states of opperation.

#### States
1. Pre-Launch  
 - 1a. Startup  initialization
 - 1b. System verification
2. Launch
 - 2a. Liftoff
 - 2b. Flight mode
 - 2c. Apogee
 - 2d. Descent

#Code Architecture:
The avionics flight software opperates on a read-eval loop in order to change states and respond to its environment.

#### Classes
`Avionics` - Implimentation of flight controller.

`Data` - Structure of current data frame.

`Sensors` - Interface to current, filtered data from hardware.

#### Files
`main.cpp` - Start point of flight controller.

`config.h` - Mission specific configuration values.

#Implimentation Details:
Here is the current status of the code:

####Flight Critical Systems
1. MicroSD data logging of both raw & calculated values. Log both data & operations (transmitting on RockBlock, transmitting on APRS, cutting down, feeding WD, etc).
2. BMP280 reading, filtering, error-checking of pressure & temperature values.
3. MPL3115A2; use as backup for BMP280's.
4. Thermocouple for external temperature sensing.
5. Heating using PID with adjustable setpoint by default 0C on inboard heater trace.
6. Flight Termination FET for Nichrome Wire -- having an enable bool, an *alt_termination* bool, a *gpsfence_termination* bool, as well as variables for GPS fence & altitude max. The former bool must be true for either of the latter bools to matter; if the former bool is true, then software should follow either, both, or neither of the latter bools for flight termination.
7. Indicator LEDs for all appropriate states -- Green for good, Red for bad.
8. Fault LED *ON* if anything flight-critical is not fully functioning -- you should make a list of every possible thing this could be. If this LED is on, you should NOT launch.
9. Disable LED's > 500 feet/meters.
10. Use of on-board Teensy RTC -- first column of data log in each row should be timestamp.
11. Integration of uBlox M8Q GPS -- this means gathering all of the possible useful data (including lat, long, alt, numSats, VDOP/PDOP if possible, others -- whatever you can).
12. GPS successful set to Flight Mode.
13. RockBlock data downlink.
####Useful Flight Features
1. APRS downlink with Dorji.
2. CAN communication with Medusa to get current state of system.
3. I2C current readings for Heater Trace, Cutdown FET, Dorji.
4. Set appropriate bounds for current draw for each, add a buffer, and set as current limits after which the device will be hard reset.
5. Watchdog interrupts (pulse pin high for 1ms, then low) every 2-10 seconds to disable hardware reset.
6.RockBlock command parsing for cutdown and transmit rate and hardware resets.
