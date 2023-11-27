# add alias of  config 
alias M210-real="/home/nvidia/programming/Onboard-SDK/utility/bin/armv8/64-bit/./MatriceSeries_ConfigTool --usb-port /dev/ttyACM0 --config-file /home/nvidia/programming/Onboard-SDK/build/bin/UserConfig.txt --power-supply on"
alias M210-sim-on="/home/nvidia/programming/Onboard-SDK/utility/bin/armv8/64-bit/./MatriceSeries_ConfigTool --usb-port /dev/ttyACM0 --config-file /home/nvidia/programming/Onboard-SDK/build/bin/UserConfig.txt --power-supply on --usb-connected-flight on --simulation on --latitude 37.4103824 --longitude -6.0020946"

# add ros sources

source /opt/ros/noetic/setup.bash
source /home/$USER/programming/resisto_ws/devel/setup.bash