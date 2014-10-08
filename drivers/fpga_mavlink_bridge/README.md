FPGA MAVLINK Bridge
========================
Writes MAVLINK LCM to be written to Helen's FPGA board via the mavlink-lcm serial bridge.

Dependencies
============
The bridge uses the MAVCONN serial bridge.  You must install and run that bridge.  Start here: 

https://github.com/andybarry/flight/wiki/Install-mavconn---mavlink


Installation
============

You must have the LCM types compiled.  Run flight/LCM/build to do so.


Usage
=====
Example:

    ./fpga-mavlink-bridge
