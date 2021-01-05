# EV-Code
Arduino projects for the Boxster - http://www.electricboxster.com

LLTBMS07 runs 12 BMS modules on the Pacifica batteries, using multiplexers and isolators for safety.

JLD505 is a clone of the EVTV board of the same name; the code is available but in an old version. Handles coulomb counting using INA226 and CHAdeMO with MCP2515 and optoisolators. Includes temperature sensors. Will run ammeter, voltmeter, and temperature gauges eventually.

CANRelay is a Pro Mini (well a Pro Micro) with 2 CAN boards so it connects the 250kbps EVCC/charger to the 500kbps DC/DC, BMS and BMS controller. Eventually to the JLD505 and CHAdeMO as well, maybe the GEVCU and DMOC too.

FinalBMSCode is my current use, bleeding edge code running on a Teensy 3.5 reading and interpreting Leaf BMS cell data. Leaf Spy Pro is concurrently running.
