#!/bin/bash

ifconfig wlan2 -multicast
ifconfig lo multicast
route add -net 224.0.0.0 netmask 240.0.0.0 dev lo
