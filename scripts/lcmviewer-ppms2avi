#!/bin/bash

if (( $# < 3 ))
then
    echo "usage: `basename $0` <bitrate> <framerate> <input_file.ppms.gz>"
    exit 1
fi

bitrate=$1
framerate=$2
shift 2

# convert a .ppms.gz file (produced by viewer) into an mpeg4 file, using high quality.
# for "web" quality, use -b 2M
zcat $* | avconv -r $framerate -y -vcodec ppm -f image2pipe -i - -vcodec mpeg4 -b $bitrate -vtag XVID output.avi 
