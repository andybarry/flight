Ardupilot MAVLINK Bridge
========================
Reads LCM MAVLINK messages and converts them to messages on many different channels, one channel for each data stream./

Dependencies
============
The bridge takes its input from the MAVCONN serial bridge.  You must install and run that bridge.  Start here: https://pixhawk.ethz.ch/software/mavconn/start

Note: you do not need all of MAVCONN, instead you just need the serial bridge.  When compiling, you can just run:

    make mavconn-bridge-serial



Installation
============

You must have the LCM types compiled.  Run flight/LCM/build to do so.


Usage
=====
Example:

    ./ardupilot-mavlink-bridge MAVLINK attitude baro-airspeed gps battery-status deltawing_u servo_out stereo-control beep
