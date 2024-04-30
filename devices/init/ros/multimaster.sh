#!/bin/bash
printf "waiting for GCS ...\n"

while ! ping -c 1 -n -w 1 10.42.0.2 &> /dev/null
do
	sleep 2
done

var=0

roslaunch onboard_dji multimaster-noetic.launch &

while  [ "$var" -eq 0 ];do
    sleep 20
    myvar=$(rosservice call /master_discovery/list_masters )
    echo -e  $myvar
    if [[ "$myvar" == *'10.42.0.2'* ]]; then
        echo "It's there."
        var=1
        break
    fi
    rosnode kill /master_discovery
    rosnode kill /master_sync
    sleep 20
    roslaunch onboard_dji multimaster-noetic.launch &
done
