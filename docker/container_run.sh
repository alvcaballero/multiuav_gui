xhost +local:docker;

# Set the project directory (PROJECT_DIR) as the parent directory of the current working directory
PROJECT_DIR="${HOME}/work/px4";

# Print the current working directory to verify the change
echo "Current working directory: $PROJECT_DIR"

# if dont exist container create one
if [ "$(docker ps -qaf name=px4_noetic)" = "" ]; then
echo 'Container not found, creating it ...';
docker run -it \
--name px4_noetic \
--privileged \
--mount type=bind,source=$PROJECT_DIR,destination=/home/user/ \
--env DISPLAY=$DISPLAY \
--network host \
--volume /tmp/.X11-unix:/tmp/.X11-unix \
-p 11311:11311    \
-p 9090:9090      \
-p 14550:14550/udp  \
-p 14570:14570/udp  \
-p 14560:14560  \
-p 8080:8080  \
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
