xhost +local:docker;

# Set the project directory (PROJECT_DIR) as the parent directory of the current working directory
PROJECT_DIR="${HOME}/work/px4";
PROJECT_DIST="/home/nonroot/work";

# Print the current working directory to verify the change
echo "Current working directory: $PROJECT_DIR"

# if dont exist container create one
if [ "$(docker ps -qaf name=ros2humble)" = "" ]; then
echo 'Container not found, creating it ...';
docker run -it \
--name ros2humble \
--privileged \
--env DISPLAY=$DISPLAY \
--network host \
--volume /tmp/.X11-unix:/tmp/.X11-unix \
--mount type=bind,source=$PROJECT_DIR,destination=$PROJECT_DIST \
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
