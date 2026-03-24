# Gstreamer dependencies

//new link to check
https://github.com/daniilidis-group/ffmpeg_image_transport

```
apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio gstreamer1.0-rtsp
```

# Stream the video server and drone onli serve in drone

rosservice call /uav_2/setup_camera_stream "cameraType: 0
start: 1"

https://github.com/CircusMonkey/ros_rtsp/tree/master

# Stream the video server and drone server in all devices

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
rosservice call /uav_2/setup_camera_stream "cameraType: 0 start: 1"
rosservice call /uav_2/setup_camera_stream "{cameraType: 0, start: 1}"
```

UAV_2 -M210

```
docker run --rm -it --network=host bluenviron/mediamtx:latest
rosservice call /uav_2/setup_camera_stream "{cameraType: 1, start: 1}"
rosrun simple_vs gstreamer-ros.py


```

UAV_3 - PX4 -Vtool

```

docker run --rm -it --network=host  aler9/rtsp-simple-server
gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480 ! videoconvert ! x264enc bframes=0 tune=zerolatency bitrate=500 speed-preset=superfast ! h264parse  ! rtspclientsink location=rtsp://0.0.0.0:8554/video0 rtpjpegpay name=pj
```

UAV_4 -M300

```
rosservice call /uav_15/setup_camera_stream "{cameraType: 1, start: 1}"
roslaunch ros_rtsp rtsp_streams.launch
roslaunch simple_vs video_resize.launch
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

# websockets
