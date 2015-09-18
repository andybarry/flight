#!/bin/bash

if [ -z "$1" ]; then
    echo "Usage: getGitBundle <hostname>"
    exit 1
fi

if [ -z "$2" ]; then
    NUM_COMMITS=15
else
    NUM_COMMITS=$2
fi

cd $HOME/realtime

echo "Packing bundle on remote..."
ssh odroid@$1 "cd /home/odroid/realtime; git bundle create commits.bundle HEAD~$NUM_COMMITS..HEAD"

echo "Downloading bundle..."
rsync --progress odroid@$1:/home/odroid/realtime/commits.bundle /home/$USER/commits.bundle

$HOME/realtime/scripts/unpackGitBundle "$HOME/commits.bundle"

echo "Removing bundle from remote..."
ssh odroid@$1 "rm /home/odroid/realtime/commits.bundle"
