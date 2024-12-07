# Author: TaeYoung Kim
# email: tyoung96@yonsei.ac.kr

FROM muavgcs:noetic

ARG ROS_DISTRO=noetic


RUN apt-get install  git 

SHELL ["/bin/bash", "-c"]

RUN cd ~ \
    && git clone https://github.com/dji-sdk/Onboard-SDK.git \
    && cd Onboard-SDK\
    && mkdir build \
    && cd build \
    && cmake .. \
    && sudo make -j7 install 



RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && mkdir -p ~/catkin_ws/src  \
    && cd ~/catkin_ws/src 
#    && git clone --recurse-submodules --branch GimbalPitch -j8 https://github.com/alvcaballero/multiUAV_system.git 
#    && git clone --branch dev https://github.com/miggilcas/Onboard-SDK-ROS.git
#    && git clone https://github.com/grvcTeam/grvc-utils.git

COPY ./multiUAV_system /root/catkin_ws/src/multiUAV_system
COPY ./Onboard-SDK-ROS /root/catkin_ws/src/Onboard-SDK-ROS
#COPY /home/grvc/work/px4/carkin_ws/src/multiUAV_system/grvc-utils /root/catkin_ws/src/grvc-utils

RUN source /opt/ros/$ROS_DISTRO/setup.bash  \
    && cd ~/catkin_ws \
    && catkin_make 

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc \
    && echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc 


CMD ["bash"]