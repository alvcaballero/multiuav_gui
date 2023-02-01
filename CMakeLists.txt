cmake_minimum_required(VERSION 2.8.3)
project(aerialcore_gui)

find_package(catkin REQUIRED COMPONENTS
 std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

#############
## Install ##
#############

install(PROGRAMS
   src/index.js
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

