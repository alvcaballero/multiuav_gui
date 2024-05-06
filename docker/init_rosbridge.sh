#!/bin/bash
sourceVar="source /opt/ros/noetic/setup.bash && source /home/user/catkin_ws/devel/setup.bash "

sleep 10

docker start px4_noetic

sleep 10


docker exec -i px4_noetic /bin/bash -c "${sourceVar} && roslaunch aerialcore_gui connect_uas.launch"



