#!/bin/bash

# plays back logs with a reasonable selection of channels

# check for an existing .jlp file

while getopts ":rh" opt; do
    case $opt in
        r)
            FORCE_RESET="1"
            ;;
        h)
            echo "Usage: log-playback [options] <lcm log file>"
            echo "Options:"
            echo "    -r reset .jlp file to default."
            exit 1
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            ;;
    esac
done

# get the files
shift $((OPTIND-1))

LOG_FILE=$@

JLP_TEMPLATE=$HOME/realtime/scripts/logs/playback-jlp-template.jlp

if [[ -e $LOG_FILE ]]
then
    if [[ -e $LOG_FILE.jlp && $FORCE_RESET -ne "1" ]]
    then
        # a .jlp file exists, just play the log
        : # pass
    else
        # a .jlp file doesn't exist, copy in a template
        cp $JLP_TEMPLATE $LOG_FILE.jlp
    fi
    lcm-logplayer-gui -p $LOG_FILE
else
    # log file does not exist
    echo "Log file does not exist"
fi
