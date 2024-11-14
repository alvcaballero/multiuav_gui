xhost +local:docker;

# Set the project directory (PROJECT_DIR) as the parent directory of the current working directory
PROJECT_DIR="${HOME}/work/px4";
#PROJECT_DIST="/home/${USER}/work";
PROJECT_DIST="/home/nonroot/work";

# Print the current working directory to verify the change
echo "Current working directory: $PROJECT_DIR"

# if dont exist container create one
if [ "$(docker ps -qaf name=px4_noetic)" = "" ]; then
echo 'Container not found, creating it ...';
docker run -it \
--name px4_noetic \
--privileged \
--workdir $PROJECT_DIST \
--mount type=bind,source=$PROJECT_DIR,destination=$PROJECT_DIST \
--env DISPLAY=$DISPLAY \
--network host \
--volume /tmp/.X11-unix:/tmp/.X11-unix \
muavgcs:noetic bash\
;
else
    if [ "${1}" = "restart" ] && ! docker stop px4_noetic > /dev/null; then
        echo 'Error while stopping the container, exiting now ...';
        return 1;
    fi;
    if ! docker start px4_noetic > /dev/null; then
        echo 'Error while starting the container, exiting now ...';
        return 1;
    fi;
    echo 'px4_docker found and running, executing a shell ...';
    docker exec -it px4_noetic bash --login;
fi;
