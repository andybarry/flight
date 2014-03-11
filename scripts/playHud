#!/bin/bash

# play HUD given an argument for a logfile
# usage: ./playHud <lcm-log>

if [ -z "$1" -o "$1" == "--help" -o "$1" == "-h" ]; then
    echo "usage: playHud <lcm-log>"
    exit 1
fi

# check for an existing .jlp file, if one
# isn't found, then create one that maps
# stereo --> stereo_replay

if [ ! -f $1.jlp ];
then
    # no .jlp file, add one
    echo -e "ZOOMFRAC 0.1\nCHANNEL stereo stereo_replay true" > "$1.jlp"
fi


REALTIME=$HOME/realtime

lcm-logplayer-gui $1 &

$REALTIME/sensors/stereo/opencv-stereo -c $REALTIME/sensors/stereo/aaazzz.conf -i `dirname $1`/onboard -d -v

