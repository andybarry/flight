#!/bin/bash

if [ -z "$1" ]; then
    echo "Usage: sendGitBundleSingle <hostname> [number of commits]"
    exit 1
fi

if [ -z "$2" ]; then
    NUM_COMMITS=15
else
    NUM_COMMITS=$2
fi

cd $HOME/realtime

git bundle create commits.bundle HEAD~$NUM_COMMITS..HEAD

rsync --progress commits.bundle odroid@$1:/home/odroid/

ssh odroid@$1 "/home/odroid/realtime/scripts/unpackGitBundle /home/odroid/commits.bundle"

rm commits.bundle
