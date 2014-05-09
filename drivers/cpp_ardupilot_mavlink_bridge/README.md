Ardupilot MAVLINK Bridge
========================
Reads LCM MAVLINK messages and converts them to messages on many different channels, one channel for each data stream./

Dependencies
============
The bridge takes its input from the MAVCONN serial bridge.  You must install and run that bridge.  Start here: 

https://github.com/andybarry/flight/wiki/Install-mavconn---mavlink


Installation
============

You must have the LCM types compiled.  Run flight/LCM/build to do so.


Usage
=====
Example:

    ./ardupilot-mavlink-bridge MAVLINK attitude baro-airspeed gps battery-status deltawing_u servo_out stereo-control beep
