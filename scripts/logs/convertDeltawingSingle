#!/bin/bash


TYPELIST=`find $HOME/realtime/LCM/*.py -printf ",%f" | cut -c  2- | sed s/.py//g`
TYPELIST2=`find $HOME/realtime/LCM/lcmt/*.py -printf ",%f" | cut -c  2- | sed s/.py//g`

TYPELIST="$TYPELIST,$TYPELIST2"

# for some reason the mavlink_msg_container_t causes issues
TYPELIST=${TYPELIST/,mavlink_msg_container_t/}
TYPELIST=${TYPELIST/,__init__/}

export PYTHONPATH=$PYTHONPATH:$HOME/realtime/LCM:$HOME/realtime/LCM/lcmt:$HOME/pronto-distro/build/lib/python2.7/dist-packages/mav:$HOME/pronto-distro/build/lib/python2.7/dist-packages/bot_core

python $HOME/realtime/scripts/logs/log_to_mat.py -m -f -l $TYPELIST,pose_t,pose_t,ins_t,gps_data_t,indexed_measurement_t,filter_state_t,image_sync_t $1
