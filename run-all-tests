#!/bin/bash

# make bash bail out if anything errors
set -e
set -o pipefail

# error out if the date is before the year 2014
# since that indicates the clock isn't set

if [ `date +%s` -lt 1430231759 ]
then
    echo "Error: time/date is not set correctly."
    echo "Current time/date: `date`"
    exit 1
fi

# read the file
FILE=tests.txt

ROOT=`pwd`

while read line; do
    # ignore comments
    case "$line" in \#*) continue ;; esac

    # ignore empty lines
    if [ -z $line ]
    then
        continue
    fi



    # get directory
    dir=$(dirname "$line")
    filename=$(basename "$line")

    echo "----------------------------------------"
    echo "   Running: $line"
    echo "----------------------------------------"

    cd $ROOT/$dir

    # run test
    ./$filename


done < $FILE


