#!/bin/bash
#https://github.com/Taeyoung96/OctoMap-ROS2/blob/master/docker/ros_entrypoint.sh 
set -e

# Ros build
source "/opt/ros/noetic/setup.bash"
#source "/root/catkin_ws/devel/setup.bash"

echo "============== Server ROS Noetic Docker Env Ready================"

cd /root/catkin_ws

exec "$@"