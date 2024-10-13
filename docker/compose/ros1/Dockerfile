# Author: TaeYoung Kim
# email: tyoung96@yonsei.ac.kr

FROM muavgcs:noetic

ARG ROS_DISTRO=noetic


RUN apt-get install  git 

SHELL ["/bin/bash", "-c"]

RUN git clone https://github.com/dji-sdk/Onboard-SDK.git \
    && cd Onboard-SDK\
    && mkdir build \
    && cd build \
    && cmake.. \
    && sudo make -j7 install 


SHELL ["/bin/bash", "-c"]


RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && mkdir catkin_ws && cd carkin_ws \
    && mkdir src && cd src \
    && catkin_init_workspace \
    && git clone --recurse-submodules --branch GimbalPitch -j8 https://github.com/alvcaballero/multiUAV_system.git \
    && git clone https://github.com/miggilcas/Onboard-SDK-ROS.git \
    && cd .. \
    && catkin_make \
    && echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc \
    && source /root/catkin_ws/devel/setup.bash


WORKDIR /root/catkin_ws

CMD ["bash"]