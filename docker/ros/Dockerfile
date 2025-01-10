# Author: TaeYoung Kim
# email: tyoung96@yonsei.ac.kr

FROM px4io/px4-dev-ros-noetic

#add in /root/.bashrc 
# source /opt/ros/noetic/setup.bash
# source /home/user/catkin_ws/devel/setup.bash

# Install PCL & Eigen & essential libraries
RUN apt-get update 

RUN apt-get install -y nano 
RUN apt-get install -y ros-noetic-rviz


RUN apt-get install -y ros-noetic-rosbridge-server

# multimaster
RUN apt-get install -y ros-noetic-fkie-master-sync 
RUN apt-get install -y ros-noetic-fkie-master-discovery
# ROS SDK DJI
RUN apt-get install -y ros-noetic-nmea-comms
RUN apt-get install -y ffmpeg libavcodec-dev libswresample-dev
RUN apt-get install -y libsdl2-dev
# install opencv 3.3.X


# dependencies simple VS
RUN apt-get install -y ros-noetic-cv-bridge ros-noetic-image-transport ros-noetic-theora-image-transport 
RUN apt-get install -y ros-noetic-tf2-kdl
RUN apt-get install -y ros-noetic-geodesy
RUN apt-get install -y ros-noetic-tf2-geometry-msgs
RUN apt-get install -y ros-noetic-image-view
RUN apt-get install -y ros-noetic-teleop-twist-keyboard

# install Gstreamer
RUN apt-get install -y libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev libgstreamer-plugins-bad1.0-dev libgstrtspserver-1.0-dev gstreamer1.0-plugins-ugly gstreamer1.0-plugins-bad gstreamer1.0-rtsp
RUN apt install gstreamer1.0-rtsp

# libqt5websockets5 --- for flatbuffers
RUN apt-get -y install libqt5websockets5-dev
RUN ln -s /usr/bin/python3 /usr/bin/python

# permissions for user
RUN apt-get install -y ros-noetic-rqt-graph


# Update the package list, install sudo, create a non-root user, and grant password-less sudo permissions
ARG USER_NAME=nonroot
ARG USER_ID=1000
ARG GROUP_ID=1000

RUN apt update && \
    apt install -y sudo && \
    addgroup --gid $GROUP_ID $USER_NAME && \
    adduser --uid $USER_ID --gid $GROUP_ID --disabled-password --gecos "" $USER_NAME && \
    echo 'nonroot ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers


# Opcional: Cambia los permisos de la carpeta si necesitas que sea accesible por otros usuarios
RUN mkdir -p /home/$USER_NAME/work
RUN chmod 755 /home/$USER_NAME/work
RUN chown $USER_NAME:$USER_NAME /home/$USER_NAME/work
    
USER $USER_NAME
RUN mkdir -p /home/$USER_NAME/test
RUN /bin/bash -c 'echo -e "/opt/ros/noetic/setup.bash \n" >> ~/.bashrc ;'
RUN /bin/bash -c 'echo -e "source /home/${USER_NAME}/work/catkin_ws/devel/setup.bash \n" >> ~/.bashrc ;'

# WORKDIR /root/catkin_ws

# Load ROS environment at each run
# COPY ./ros_entrypoint.sh /
# RUN chmod 755 /ros_entrypoint.sh
# ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]