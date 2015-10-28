#!/bin/bash
cd $1

FLIGHTS=((0))

for dir in `ls -d 2015*-field-test*`
do
    echo $dir

    seconddir="gps-logs"

    fulldir="$dir/$seconddir"
    for file in `ls $fulldir/lcmlog-*`
    do
        FILESIZE=`du -k "$file" | cut -f1`

        if [ "$FILESIZE" -gt "100000" ]
        then
            echo "$file => $FILESIZE"
            FLIGHTS=$((FLIGHTS+1))
        fi
    done

    seconddir="odroid-gps1"

    fulldir="$dir/$seconddir"
    for file in `ls $fulldir/lcmlog-*`
    do
        FILESIZE=`du -k "$file" | cut -f1`

        if [ "$FILESIZE" -gt "100000" ]
        then
            echo "$file => $FILESIZE"
            FLIGHTS=$((FLIGHTS+1))
        fi
    done

    seconddir="odroid-gps2"

    fulldir="$dir/$seconddir"
    for file in `ls $fulldir/lcmlog-*`
    do
        FILESIZE=`du -k "$file" | cut -f1`

        if [ "$FILESIZE" -gt "100000" ]
        then
            echo "$file => $FILESIZE"
            FLIGHTS=$((FLIGHTS+1))
        fi
    done

    seconddir="odroid-gps3"

    fulldir="$dir/$seconddir"
    for file in `ls $fulldir/lcmlog-*`
    do
        FILESIZE=`du -k "$file" | cut -f1`

        if [ "$FILESIZE" -gt "100000" ]
        then
            echo "$file => $FILESIZE"
            FLIGHTS=$((FLIGHTS+1))
        fi
    done

done
echo "flights = $FLIGHTS"
