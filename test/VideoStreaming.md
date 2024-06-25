# Webrtc and webrtc-ext

The self computer, we add the next uav in devices init, and with gstreamer comand publish the video

```

  - name: 'uav_2'
    protocol: 'robofleet' #'robofleet'
    category: 'dji_M210_noetic'
    ip: '127.0.0.1'
    user: 'gridwatch'
    pwd: 'fooPass'
    camera:
      - type: 'WebRTC'
        source: 'main'


  - name: 'uav_20'
    category: 'fuvex'
    ip: '127.0.0.1' #'10.42.0.101'
    camera:
      - type: 'WebRTC'
        source: 'test'
```

publish the camera

```
gst-launch-1.0 v4l2src device=/dev/main ! video/x-raw,width=640,height=480 ! videoconvert ! x264enc bframes=0 tune=zerolatency bitrate=500 speed-preset=superfast ! h264parse  ! rtspclientsink location=rtsp://0.0.0.0:8554/video0 rtpjpegpay name=pj
```

publis a download video from youtube

```
cd ~/work/px4
gst-launch-1.0 filesrc location=test.mp4 ! decodebin ! x264enc tune=zerolatency ! h264parse ! rtspclientsink location=rtsp://0.0.0.0:8554/test rtpjpegpay
```

```
gst-launch-1.0 rtspsrc location=rtsp://10.42.0.230:8554/visible latency=100 ! rtph264depay ! avdec_h264 ! videoscale ! video/x-raw,width=640,height=480 ! x264enc tune=zerolatency ! h264parse  ! rtspclientsink location=rtsp://0.0.0.0:8554/visible2 rtpjpegpay name=pj

gst-launch-1.0 rtspsrc location=rtsp://10.42.0.230:8554/visible latency=100 ! rtph264depay ! avdec_h264 ! x264enc tune=zerolatency ! h264parse  ! rtspclientsink location=rtsp://0.0.0.0:8554/visible2 rtpjpegpay name=pj

gst-launch-1.0 rtspsrc location=rtsp://10.42.0.230:8554/thermal latency=100 ! rtph264depay! avdec_h264 ! autovideosink

```
