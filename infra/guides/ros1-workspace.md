# Workspace ROS 1 (Noetic) para el ROS bridge

Lo que va dentro de la imagen `muavgcs:ros1` (ver `infra/README.md` › ROS Bridge) o de un contenedor propio que
se arranca con su `container_run.sh` (lo que usa `infra/tmuxinator/gcs-full.yml`, `ROS_DOCKER_DIR`). El Dockerfile
vive en el repo del workspace ROS, no en este.

## DJI Onboard SDK

```bash
git clone https://github.com/dji-sdk/Onboard-SDK.git
cd Onboard-SDK
mkdir build && cd build
cmake ..
sudo make -j7 install
cd ../..
```

## Workspace catkin

```bash
mkdir -p catkin_ws/src && cd catkin_ws/src
catkin_init_workspace

git clone --recurse-submodules --branch GimbalPitch -j8 https://github.com/alvcaballero/multiUAV_system.git
git clone https://github.com/miggilcas/Onboard-SDK-ROS
git clone --branch noetic https://github.com/grvcTeam/grvc-utils.git
# opcionales:
# git clone https://github.com/CircusMonkey/ros_rtsp.git
# git clone https://github.com/miggilcas/simple_vs.git

cd ..
catkin_make
```

## Arrancar el bridge

Dentro del contenedor:

```bash
roslaunch aerialcore_gui connect_uas.launch
```

## WSL

Con WSL hay que mapear los puertos ([docs de Microsoft](https://learn.microsoft.com/en-us/windows/wsl/networking)):

```
netsh interface portproxy add v4tov4 listenport=4000 listenaddress=0.0.0.0 connectport=4000 connectaddress=<IP de WSL: hostname -I>
```
