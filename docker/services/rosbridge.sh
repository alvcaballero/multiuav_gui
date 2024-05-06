#!/bin/bash
sourceVar="source /opt/ros/noetic/setup.bash && source /home/user/catkin_ws/devel/setup.bash "

sleep 10
myflatvar=0
while  [ $myflatvar -lt 2 ];
do
    myvar=$(docker ps --filter "name=px4_noetic" --filter "status=running" )
    echo -e  $myvar
    if [[ "$myvar" == *'px4_noetic'* ]]; then
        echo "already running"
        ((myflatvar++))
    fi
    sleep 2
done
sleep 10


docker exec -i px4_noetic /bin/bash -c "${sourceVar} && roslaunch aerialcore_gui connect_uas.launch"



