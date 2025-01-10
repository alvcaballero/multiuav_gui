# sudo echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
# sudo echo "source /home/${USER}/work/catkin_ws/devel/setup.bash" >> ~/.bashrc

cd ~/work/px4
git clone https://github.com/dji-sdk/Onboard-SDK.git
cd Onboard-SDK
mkdir build
cd build
cmake ..
sudo make -j7 install

cd ~/work/px4
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone --recurse-submodules -j8 --branch GimbalPitch https://github.com/alvcaballero/multiUAV_system.git
git clone https://github.com/CircusMonkey/ros_rtsp.git
git clone https://github.com/miggilcas/simple_vs.git
git clone --branch noetic https://github.com/grvcTeam/grvc-utils.git
git clone https://github.com/miggilcas/Onboard-SDK-ROS.git
cd ..
catkin_make

