xhost +local:docker;

# Set the project directory (PROJECT_DIR) as the parent directory of the current working directory
PROJECT_DIR="${HOME}/work/px4";

# Print the current working directory to verify the change
echo "Current working directory: $PROJECT_DIR"

# if dont exist container create one
if [ "$(docker ps -qaf name=ros2humble)" = "" ]; then
echo 'Container not found, creating it ...';
docker run -it \
--name ros2humble \
--privileged \
--mount type=bind,source=$PROJECT_DIR,destination=/home/user/ \
--env DISPLAY=$DISPLAY \
--network host \
--volume /tmp/.X11-unix:/tmp/.X11-unix \
-p 11311:11311    \
-p 11312:11312    \
-p 9090:9090      \
-p 14550:14550/udp  \
-p 14570:14570/udp  \
-p 14560:14560  \
-p 8080:8080  \
-p 8553:8553  \
ros:humble-ros-base bash\
;
else
    if [ "${1}" = "restart" ] && ! docker stop ros2humble > /dev/null; then
        echo 'Error while stopping the container, exiting now ...';
        return 1;
    fi;
    if ! docker start ros2humble > /dev/null; then
        echo 'Error while starting the container, exiting now ...';
        return 1;
    fi;
    echo 'px4_docker found and running, executing a shell ...';
    docker exec -it ros2humble bash --login;
fi;
