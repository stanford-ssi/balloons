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

#Code architecture:
The avionics flight software opperates on a read-eval loop in order to change states and respond to its environment.

#### Classes
`Avionics` - Implimentation of flight controller.

#### Files
`main.cpp` - Start point of flight controller.
`config.h` - Mission specific configuration values.

#TODO:
1. microSD data logging of both raw & calculated values. Log both data & operations (transmitting on rockblock, transmitting on APRS, cutting down, feeding WD, etc)
2. Reading, Filtering, Error-Checking of Pressure & Temperature values for BMP280's
3. The above, for MPL3115A2; use as backup or systematic error of BMP280's.
4. Thermocouple for external temperature sensing.
5. Heating using PID with adjustable setpoint by default 0C. Inboard heater trace.
6. Flight Termination FET for Nichrome Wire -- having an enable bool, an *alt_termination* bool, a *gpsfence_termination* bool, as well as variables for GPS fence & altitude max. The former bool must be true for either of the latter bools to matter; if the former bool is true, then software should follow either, both, or neither of the latter bools for flight termination.
7. Indicator LEDs for all appropriate states -- Green for good, Red for bad.
8. Fault LED *ON* if anything flight-critical is not fully functioning -- you should make a list of every possible thing this could be. If this LED is on, you should NOT launch.
9. Disable LED's > 500 feet/meters
10. Use of on-board Teensy RTC -- first column of data log in each row should be timestamp.
11. Integration of uBlox M8Q GPS -- this means gathering all of the possible useful data (including lat, long, alt, numSats, VDOP/PDOP if possible, others -- whatever you can).
12. GPS Flight Mode
13. (last priority) Using USB Debug with GPS
14. Watchdog feeding -- feed the watchdog (pulse pin high for 1ms, then low) every 2-10 seconds to disable hardware reset.
15. RockBlock, 2 way comms. Alterable transmit time interval via global variable. Allow being able to send commands to do any number of useful things, at the least being altering key global variables (cutdown, cutdown vars, etc) as well as hardware resets of GPS & Dorji (via PFETs). Also, cutdown via passcode. Also, being able to send commands to avionics to send to medusa.
16. Bi-Directional CAN communication with Medusa; end-goal is to develop custom library for both Avionics & Medusa/other boards featuring methods such as *getAlt(), getIntTemp(), getExtTemp(), getLat(), getLong(), cutdown(), sendToAvionics()* etc, which should use some sort of identifier to identify which board is receiving the commands, and execute appropriately.
15. APRS Transmission using Dorji -- necessarily transmit lat/long/alt, and if appropriate and possible, transmit payload vitals.
16. Current readings for everything with a current sensor -- they are on I2C. Devices with current sensor include *Heater Trace, Cutdown FET, Dorji). I believe GPS doesn't have one. Should set appropriate bounds for current draw for each, add a buffer, and set as current limits after which the device will be hard reset. Include other safety mechanisms & make sure it doesn't allow entering a rogue state.
17. Determine what is, and log, key data to EEPROM. Examples could be cutdown settings/passcode among other things. Read from EEPROM on hard start -- this is useful in particular in the case of a HW reset by the watchdog, so we aren't left confused after the reboot. Should *DEFINITELY* keep track of commands sent over RB and re-load them on re-boot, because those will otherwise be lost (if they are, say, cutdown setting changes).
