https://roboticseabass.com/2021/04/21/docker-and-ros/

# Build the Dockerfile

docker build -t nvidia_ros .

# Start a terminal

docker run -it --net=host --gpus all \
 --env="NVIDIA_DRIVER_CAPABILITIES=all" \
 --env="DISPLAY" \
 --env="QT_X11_NO_MITSHM=1" \
 --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
 nvidia_ros \
 bash
