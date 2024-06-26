from threading import Thread
import cv2
import numpy as np

from datetime import datetime
from time import sleep, time


class RTSPVideoWriterObject(object):
    def __init__(self, src=0):
        # Create a VideoCapture object
        self.capture = cv2.VideoCapture(src)
        self.status, self.frame = None, None

        # Default resolutions of the frame are obtained (system dependent)
        self.frame_width = int(self.capture.get(3))
        self.frame_height = int(self.capture.get(4))

        self.frame = np.zeros(
            (self.frame_height, self.frame_width, 3), np.uint8)

        self.status = False

        # Set up codec and output video settings
        # self.codec = cv2.VideoWriter_fourcc(*'MJPG')
        # self.output_video = cv2.VideoWriter(
        #    'output.avi', self.codec, 30, (self.frame_width, self.frame_height))

        # Start the thread to read frames from the video stream
        self.thread = Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()

    def update(self):
        # Read the next frame from the stream in a different thread
        while True:
            if self.capture.isOpened():
                (self.status, self.frame) = self.capture.read()
            else:
                print("Camera is disconnected")

    def show_frame(self):
        # Display frames in main program
        if self.status:
            cv2.imshow('frame', self.frame)

        # Press Q on keyboard to stop recording
        key = cv2.waitKey(1)
        if key == ord('q'):
            self.capture.release()
            # self.output_video.release()
            cv2.destroyAllWindows()
            exit(1)

        # def save_frame(self):
        # Save obtained frame into video output file
        # self.output_video.write(self.frame)
    def getframe(self):
        return self.frame

    def getstatus(self):
        return self.status


class RTSPVideoPublishObject(object):
    def __init__(self, src="rtsp://0.0.0.0:8554/visible2"):
        self.fps = 15
        self.width = 640
        self.height = 480

        self.out = cv2.VideoWriter('appsrc ! videoconvert' +
                                   ' ! video/x-raw,format=I420' +
                                   ' ! x264enc speed-preset=ultrafast bitrate=600 key-int-max=' + str(self.fps * 2) +
                                   ' ! video/x-h264,profile=baseline' +
                                   ' ! rtspclientsink location='+src,
                                   cv2.CAP_GSTREAMER, 0, self.fps, (self.width, self.height), True)

        # if not self.out.isOpened():
        #     raise Exception("can't open video writer")

        self.start = time()
        self.frame = np.zeros((self.height, self.width, 3), np.uint8)

        # Start the thread to read frames from the video stream
        self.thread = Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()

    def update(self):
        # Read the next frame from the stream in a different thread
        while True:

            self.out.write(self.frame)
            print("%s frame written to the server" % datetime.now())

            now = time()
            diff = (1 / self.fps) - now - self.start
            if diff > 0:
                sleep(diff)
            self.start = now

    def update_frame(self, frame):
        resized_image = cv2.resize(frame, (self.width, self.height))
        self.frame = resized_image


if __name__ == '__main__':
    rtsp_link = "rtsp://127.0.0.1:8554/test"
    rtsp_link_output = "rtsp://0.0.0.0:8554/visible2"
    video_stream_widget = RTSPVideoWriterObject(rtsp_link)

    video_stream_widget_output = RTSPVideoPublishObject(rtsp_link_output)

    while True:
        try:
            video_stream_widget.show_frame()
            status = video_stream_widget.getstatus()
            if status:
                myframe = video_stream_widget.getframe()
                video_stream_widget_output.update_frame(myframe)
            if status is False:
                sleep(1)
                # manejar y reconectar
                print("Frame is read")

            # video_stream_widget.save_frame()
        except AttributeError:
            pass
