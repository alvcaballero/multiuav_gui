xhost +local:docker;

# Set the project directory (PROJECT_DIR) as the parent directory of the current working directory
PROJECT_DIR="${HOME}/work/px4";
PROJECT_DIST="/home/nonroot/work";

# Print the current working directory to verify the change
echo "Current working directory: $PROJECT_DIR"

# if dont exist container create one
if [ "$(docker ps -qaf name=px4_ros2_humble)" = "" ]; then
echo 'Container not found, creating it ...';
docker run -it \
--name px4_ros2_humble \
--privileged \
--env DISPLAY=$DISPLAY \
--gpus all \
--network host \
--pid=host \
--volume /tmp/.X11-unix:/tmp/.X11-unix \
--volume /mnt/wslg:/mnt/wslg \
--mount type=bind,source=$PROJECT_DIR,destination=$PROJECT_DIST \
mzahana/px4-dev-simulation-ubuntu22:latest bash\
;
else
    if [ "${1}" = "restart" ] && ! docker stop px4_ros2_humble > /dev/null; then
        echo 'Error while stopping the container, exiting now ...';
        return 1;
    fi;
    if ! docker start px4_ros2_humble > /dev/null; then
        echo 'Error while starting the container, exiting now ...';
        return 1;
    fi;
    echo 'px4_docker found and running, executing a shell ...';
    docker exec -it px4_ros2_humble bash --login;
fi;
