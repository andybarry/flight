#!/bin/bash

# download logs from remote

if [ -z "$3" ]; then
    echo "Usage: downloadLogs <REMOTE_DIR> <LOCAL_DIR> <RYSNC_OPTIONS> [HOSTNAME]"
    exit 1
fi

if [ -z "$4" ]
then
    echo "Please enter the hostname:"
    echo -n "odroid-"
    read HOST_END
else
    HOST_END=${4: -4}
    echo "computer name = $HOST_END"
fi

REMOTE_DIR="$1"
LOCAL_DIR="$2"

OPTIONS=$3

if [[ $HOST_END == *"cam"* ]]
then
    # this is a multi-hop download, figure out the number in the download
    ODROID_NUMBER=${HOST_END: -1}

    echo "Detected multihop through odroid-gps$ODROID_NUMBER"

    SSH_TRANSPORT="-e ssh -A odroid@odroid-gps$ODROID_NUMBER ssh odroid@odroid-$HOST_END"

else
    SSH_TRANSPORT="odroid@odroid-$HOST_END"
fi

mkdir -p $LOCAL_DIR/odroid-$HOST_END

echo "Downloading logs from: $SSH_TRANSPORT..."


if [[ $HOST_END == *"cam"* ]]
then
    echo rsync --progress --no-inc-recursive --ignore-existing -a -s $OPTIONS "$SSH_TRANSPORT" :$REMOTE_DIR $LOCAL_DIR/odroid-$HOST_END
    rsync --progress --no-inc-recursive --ignore-existing -a -s $OPTIONS "$SSH_TRANSPORT" :$REMOTE_DIR $LOCAL_DIR/odroid-$HOST_END
else
    echo rsync --progress --no-inc-recursive --ignore-existing -a -s $OPTIONS "$SSH_TRANSPORT":$REMOTE_DIR $LOCAL_DIR/odroid-$HOST_END
    rsync --progress --no-inc-recursive --ignore-existing -a -s $OPTIONS "$SSH_TRANSPORT":$REMOTE_DIR $LOCAL_DIR/odroid-$HOST_END
fi

