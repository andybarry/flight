#!/bin/sh

export CLASSPATH="`pkg-config --variable=classpath lcm-java`:."

for FILE in `ls types/*.lcm`
do
    lcm-gen -j $FILE
    lcm-gen -p $FILE
 
    # replace "types" in path with "lcmtypes" and ".lcm" with ".java"
    FILE2=${FILE/types/lcmtypes}
    
    # % says "search from back"
    javac ${FILE2/%lcm/java}

    #lcm-gen -j types/lcmt_glider_vicon.lcm
    #lcm-gen -p types/lcmt_glider_vicon.lcm

    #javac lcmtypes/lcmt_glider_vicon.java
    
done

jar cf LCMtypes.jar lcmtypes/*.class



