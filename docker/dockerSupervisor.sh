#!/bin/bash
sourceVar="source /opt/ros/noetic/setup.bash && source /home/user/catkin_ws/devel/setup.bash "

echo " init service for px4_noetic"


myvar=$(docker ps --filter "name=px4_noetic" --filter "status=running" )
echo -e  $myvar
if [[ "$myvar" == *'px4_noetic'* ]]; then
    echo "already running"
    docker stop px4_noetic
fi


sleep 10

docker start px4_noetic

sleep 10
docker exec -d px4_noetic /bin/bash -c "${sourceVar} && roscore"

sleep 3
docker exec -d px4_noetic /bin/bash -c "${sourceVar} && roslaunch aerialcore_gui connect_uas.launch"

var=0

while ! ping -c 1 -n -w 1 10.42.0.2 &> /dev/null
do
    echo " ping failt waiting for exec ping ."
	sleep 10
    var=$((var+1))
done

sleep 2


if [ $var -ne 0 ]; then

    docker stop px4_noetic

    sleep 5
    
    docker start px4_noetic

    echo "var is different from zero,init docker again..."

    sleep 10
    docker exec -d px4_noetic /bin/bash -c "${sourceVar} && roscore"
    sleep 2 
    docker exec -d px4_noetic /bin/bash -c "${sourceVar} && roslaunch aerialcore_gui connect_uas.launch"

fi


if [ $var -eq 0 ]; then
    echo " find the ip address in first ping"
fi


