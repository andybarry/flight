Ardupilot MAVLINK Bridge
========================

Reads messages from the serial port and publishes them to LCM.


Installation
============
You must have the LCM types compiled.  Run flight/LCM/build to do so.


Usage
=====
Example:
  ./ardupilot-mavlink-bridge MAVLINK attitude baro-airspeed gps battery-status deltawing_u servo_out stereo-control beep
