#!/bin/bash

if [ -z "$1" ]; then
    echo "Usage: full-update <plane_number> [number of commits]"
    exit 1
fi

cd $HOME/realtime

scripts/sendGitBundle $1 $2

scripts/build-full $1
