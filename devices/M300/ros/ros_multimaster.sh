
sleep 5
while ! ping -c 1 -n -w 1 10.42.0.2 &> /dev/null
do
    echo " ping failt waiting for exec ping ."
    sleep 10
done

sleep 5
source /opt/ros/noetic/setup.bash && source /home/nvidia/programming/resisto_ws/devel/setup.bash && roslaunch onboard_dji multimaster-noetic.launch
