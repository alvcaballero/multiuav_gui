# Stream the video

The stream of video use Mediamtx.yml in the drone and the server.
https://github.com/bluenviron/mediamtx

## Configuration of Server

For this server is important add the path with the data of de drone server

```
path:
  uav_2:
   source: rtsp://10.42.0.43:8554/uav2_fpv
```

For run de server

```
docker run --rm -it --network=host -v $PWD/mediamtx.yml:/mediamtx.yml bluenviron/mediamtx

```

For access to the server

```
<iframe src="http://mediamtx-ip:8889/mystream/publish" scrolling="no"></iframe>
or
http://localhost:8889/uav_2/ # in the browser
```

## Configuration of Drone

wake up the mediamts server

```
docker run --rm -it --network=host bluenviron/mediamtx:latest
```

in the case of px4 run the commando

```
roslaunch simple_vs video_streamer.launch
```

in the case of DJI

first run the device node of OSDK
and after set up the camera with the command

```
rosservice call /setup_camera_stream "cameraType: 0 start: 1"
```

after read the tocpic and send de image to the server

```
rosrun simple_vs gstramer-ros.py
```

For install the package simple_vs is necesary to follow the next instructions:

```
sudo apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-ugly gstreamer1.0-rtsp python3-dev python3-numpy
git clone --depth=1 -b 4.5.4 https://github.com/opencv/opencv
cd opencv
mkdir build && cd build
cmake -D CMAKE_INSTALL_PREFIX=/usr -D WITH_GSTREAMER=ON ..
make -j$(nproc)
sudo make install
```