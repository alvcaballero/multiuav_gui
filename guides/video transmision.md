
docker run --rm -it --network=host aler9/rtsp-simple-server

gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw,width=640,height=480 ! videoconvert ! x264enc bframes=0 tune=zerolatency bitrate=500 speed-preset=superfast ! h264parse  ! rtspclientsink location=rtsp://0.0.0.0:8554/video0 rtpjpegpay name=pj