#!/bin/bash

# automatically create directories for a new log

VIDEODIR=$HOME/video/rlg/4-delta-wing-era

LOGDIR=$HOME/rlg/logs

AUTO_DOWNLOAD=false

OPTIND=1         # Reset in case getopts has been used previously in the shell.
# check for a flag that the user might want their own date
while getopts "dc:h" flag; do
    case $flag in
        d)
            echo "Enter the date in YYYY-MM-DD format:"
            read DATESTR
            ;;
        c)
            AUTO_DOWNLOAD=true
            PLANE_NUMBER=$OPTARG
            ;;
        h)
            echo ""
            echo "importLogs:"
            echo "    -d                request a custom date"
            echo "    -c=PLANE_NUMBER   automatic yes on all download questions"
            echo "    -h                show this help"
            echo ""
            exit
            ;;
    esac
done

if [ -z "$DATESTR" ]; then
    DATESTR=`date "+%Y-%m-%d"`
fi

counter=1
for dir in $LOGDIR/$DATESTR-*
do
    if [ -d $dir ]
    then
        if [ $counter -eq 1 ]
        then
            echo "[0]: <new directory>"
        fi

        echo "[$counter]: $dir"
        dir_array[$counter]=$dir
        counter=$((counter+1))
    fi
done

if [ $counter -gt 1 ]
then
    while [ -z $DIRNAME ]
    do
        echo -n "Selection: "
        read SELECTION

        if [ "$SELECTION" -eq 0 ]
        then
            # got something else
            echo "Please enter the name:"
            echo -n "$DATESTR-"
            read DIRNAME
        else
            selected_dir=${dir_array[$SELECTION]}

            DIRNAME=${selected_dir/"$LOGDIR/$DATESTR-"/}
        fi
    done
else
    # query the user for the name of the directory

    echo "Please enter the name:"
    echo -n "$DATESTR-"
    read DIRNAME
fi

echo "Creating directories..."
echo "$VIDEODIR/$DATESTR-$DIRNAME"
mkdir $VIDEODIR/$DATESTR-$DIRNAME

echo "$LOGDIR/$DATESTR-$DIRNAME/onboard-vids"
mkdir -p $LOGDIR/$DATESTR-$DIRNAME/onboard-vids

echo "$LOGDIR/$DATESTR-$DIRNAME/mat"
mkdir -p $LOGDIR/$DATESTR-$DIRNAME/mat

echo "$LOGDIR/$DATESTR-$DIRNAME/local-logs"
mkdir -p $LOGDIR/$DATESTR-$DIRNAME/local-logs

echo "Creating symlinks..."
echo "$VIDEODIR/$DATESTR-$DIRNAME -> $LOGDIR/$DATESTR-$DIRNAME/ground-vids"
ln -s $VIDEODIR/$DATESTR-$DIRNAME $LOGDIR/$DATESTR-$DIRNAME/ground-vids

echo "$LOGDIR/$DATESTR-$DIRNAME/onboard-vids -> $VIDEODIR/$DATESTR-$DIRNAME/onboard-vids"
ln -s $LOGDIR/$DATESTR-$DIRNAME/onboard-vids $VIDEODIR/$DATESTR-$DIRNAME/onboard-vids

# download logs
if [ "$AUTO_DOWNLOAD" = true ]
then
    ./downloadLocalLogs "/home/$USER/lcmlog-$DATESTR.*" $LOGDIR/$DATESTR-$DIRNAME/local-logs
else
    while true; do
        read -p "Do you want to copy local logs now? [Y/N] " yn
        case $yn in
            [Yy]* ) ./downloadLocalLogs "/home/$USER/lcmlog-$DATESTR.*" $LOGDIR/$DATESTR-$DIRNAME/local-logs; break;;
            [Nn]* ) break;;
            * ) echo "Please answer yes or no.";;
        esac
    done
fi

if [ "$AUTO_DOWNLOAD" = true ]
then
    ./downloadLogs "lcmlog-$DATESTR.*" $LOGDIR/$DATESTR-$DIRNAME "-z" "odroid-gps$PLANE_NUMBER"
else
    while true; do
        read -p "Do you want to download GPS logs now? [Y/N] " yn
        case $yn in
            [Yy]* ) ./downloadLogs "lcmlog-$DATESTR.*" $LOGDIR/$DATESTR-$DIRNAME "-z"; break;;
            [Nn]* ) break;;
            * ) echo "Please answer yes or no.";;
        esac
    done
fi

if [ "$AUTO_DOWNLOAD" = true ]
then
    ./downloadLogs "lcmlog-$DATESTR.*" $LOGDIR/$DATESTR-$DIRNAME "-z" "odroid-cam$PLANE_NUMBER"
else
    while true; do
        read -p "Do you want to download CAM logs now? [Y/N] " yn
        case $yn in
            [Yy]* ) ./downloadLogs "lcmlog-$DATESTR.*" $LOGDIR/$DATESTR-$DIRNAME "-z"; break;;
            [Nn]* ) break;;
            * ) echo "Please answer yes or no.";;
        esac
    done
fi

if [ "$AUTO_DOWNLOAD" = true ]
then
    ./downloadLogs "realtime/sensors/stereo/vids/mono-*$DATESTR*" $LOGDIR/$DATESTR-$DIRNAME/onboard-vids "-z" "odroid-cam$PLANE_NUMBER"
else
    while true; do
        read -p "Do you want to download mono videos now? [Y/N] " yn
        case $yn in
            [Yy]* ) ./downloadLogs "realtime/sensors/stereo/vids/mono-*$DATESTR*" $LOGDIR/$DATESTR-$DIRNAME/onboard-vids "-z"; break;;
            [Nn]* ) break;;
            * ) echo "Please answer yes or no.";;
        esac

    done
fi

if [ "$AUTO_DOWNLOAD" = true ]
then
    ./downloadLogs "realtime/sensors/stereo/vids/video*$DATESTR*" $LOGDIR/$DATESTR-$DIRNAME/onboard-vids "-r -z" "odroid-cam$PLANE_NUMBER"
else
    while true; do
        read -p "Do you want to download stereo videos now? [Y/N] " yn
        case $yn in
            [Yy]* ) ./downloadLogs "realtime/sensors/stereo/vids/video*$DATESTR*" $LOGDIR/$DATESTR-$DIRNAME/onboard-vids "-r -z"; break;;
            [Nn]* ) break;;
            * ) echo "Please answer yes or no.";;
        esac

    done
fi

echo "Log download complete."
