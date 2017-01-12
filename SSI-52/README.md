# SSI HABEES:
###A modular flight controler with payload interface for high altitude balloons

#TODO:
1) microSD data logging of both raw & calculated values. Log both data & operations (transmitting on rockblock, transmitting on APRS, cutting down, feeding WD, etc)
2) Reading, Filtering, Error-Checking of Pressure & Temperature values for BMP280's
3) The above, for MPL3115A2; use as backup or systematic error of BMP280's.
4) Use of on-board Teensy RTC -- first column of data log in each row should be timestamp.
5) Thermocouple for external temperature sensing.
6) Integration of uBlox M8Q GPS -- this means gathering all of the possible useful data (including lat, long, alt, numSats, VDOP/PDOP if possible, others -- whatever you can).
7) GPS Flight Mode
8) (last priority) Using USB Debug with GPS
9) Flight Termination FET for Nichrome Wire -- having an enable bool, an *alt_termination* bool, a *gpsfence_termination* bool, as well as variables for GPS fence & altitude max. The former bool must be true for either of the latter bools to matter; if the former bool is true, then software should follow either, both, or neither of the latter bools for flight termination.
10) Heating using PID with adjustable setpoint by default 0C. Inboard heater trace.
11) Indicator LEDs for all appropriate states -- Green for good, Red for bad.
12) Fault LED *ON* if anything flight-critical is not fully functioning -- you should make a list of every possible thing this could be. If this LED is on, you should NOT launch.
13) Watchdog feeding -- feed the watchdog (pulse pin high for 1ms, then low) every 2-10 seconds to disable hardware reset.
14) APRS Transmission using Dorji -- necessarily transmit lat/long/alt, and if appropriate and possible, transmit payload vitals.
15) RockBlock, 2 way comms. Alterable transmit time interval via global variable. Allow being able to send commands to do any number of useful things, at the least being altering key global variables (cutdown, cutdown vars, etc) as well as hardware resets of GPS & Dorji (via PFETs). Also, cutdown via passcode. Also, being able to send commands to avionics to send to medusa.
16) Bi-Directional CAN communication with Medusa; end-goal is to develop custom library for both Avionics & Medusa/other boards featuring methods such as *getAlt(), getIntTemp(), getExtTemp(), getLat(), getLong(), cutdown(), sendToAvionics()* etc, which should use some sort of identifier to identify which board is receiving the commands, and execute appropriately.
17) Enable FET's in appropriate states; automated adjustment of these (maybe? maybe should just be manual command ACK).
18) Current readings for everything with a current sensor -- they are on I2C. Devices with current sensor include *Heater Trace, Cutdown FET, Dorji). I believe GPS doesn't have one. Should set appropriate bounds for current draw for each, add a buffer, and set as current limits after which the device will be hard reset. Include other safety mechanisms & make sure it doesn't allow entering a rogue state.
19) Determine what is, and log, key data to EEPROM. Examples could be cutdown settings/passcode among other things. Read from EEPROM on hard start -- this is useful in particular in the case of a HW reset by the watchdog, so we aren't left confused after the reboot. Should *DEFINITELY* keep track of commands sent over RB and re-load them on re-boot, because those will otherwise be lost (if they are, say, cutdown setting changes).
20) Disable LED's > 500 feet/meters/whatevers.
