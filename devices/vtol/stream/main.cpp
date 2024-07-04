#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <chrono>
#include <thread>
#include <ctime>
#include <sstream>


//std::string  rtspInput = "rtsp://10.42.0.230:8554/visible";

using namespace cv;

class RTSPVideoWriterObject {
public:
    RTSPVideoWriterObject(std::string src = "rtsp://127.0.0.1:8554/test") {
        std ::string addr("rtspsrc location=" + src +" latency=300 caps = \"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96\" ! rtph264depay ! decodebin ! videoconvert ! appsink");
    //std ::string addr("rtspsrc location=" + src + " latency=0 buffer-mode=auto ! queue ! rtph264depay ! decodebin ! videoconvert !  queue ! appsink");

        // Create a VideoCapture object
        capture.open(addr, cv::CAP_GSTREAMER);

        status = false;

        // Get default resolutions (system dependent)
        if (capture.isOpened()) {
            frame_width = capture.get(cv::CAP_PROP_FRAME_WIDTH);
            frame_height = capture.get(cv::CAP_PROP_FRAME_HEIGHT);
            frame = cv::Mat::zeros(cv::Size(frame_height, frame_width), CV_64FC1);

        } else {
            std::cerr << "Error opening video capture" << std::endl;
        }

        // Start the thread to read frames from the video stream
        thread = std::thread(&RTSPVideoWriterObject::update, this);
        thread.detach();  // Detach the thread to avoid resource leaks
    }
private:
    void update() {
        // Read the next frame from the stream in a separate thread
        while (true) {
            std::cerr << "video capture" << std::endl;

            if (capture.isOpened()) {
                status = capture.read(frame);

                if (!status) {
                    std::cerr << "Error reading frame" << std::endl;
                }
            } else {
                std::cerr << "Camera is disconnected" << std::endl;
                break;
            }
        }
    }

public:
    void show_frame() {
        // Display frames in main program

        if (status) {
            cv::imshow("frame", frame);


            // Press Q on keyboard to stop recording
            int key = cv::waitKey(1);
            if (key == 'q') {
                capture.release();
                cv::destroyAllWindows();
                exit(1);
            }
        }
    }

    cv::Mat getFrame() {
        // Access the current frame with thread synchronization
        std::mutex frame_mutex;  // Mutex to protect frame access
        frame_mutex.lock();
        cv::Mat frame_copy = frame.clone();  // Create a copy to avoid race conditions
        frame_mutex.unlock();
        return frame_copy;
    }

    bool getStatus() {
        return status;
    }

private:
    cv::VideoCapture capture;
    bool status;
    cv::Mat frame;
    int frame_width, frame_height;
    std::thread thread;
};


class RTSPVideoPublishObject {
public:
    RTSPVideoPublishObject(std::string src = "rtsp://0.0.0.0:8554/visible2") {
        fps = 15;
        width = 640;
        height = 480;
        frame = cv::Mat::zeros(cv::Size(width, height), CV_64FC1);

        std::string gstpipeline = std::string("appsrc ! videoconvert") +
            " ! video/x-raw,format=I420" + 
            " ! x264enc speed-preset=ultrafast bitrate=600 key-int-max=" + std::to_string(fps * 2) +
            " ! video/x-h264,profile=baseline" + 
            " ! rtspclientsink location="+src;
    
        out.open(gstpipeline,
        cv::CAP_GSTREAMER,
            0, 
        fps, 
        cv::Size(width, height), 
        true);

        start = std::chrono::high_resolution_clock::now();
        now = std::chrono::high_resolution_clock::now();

        if( ! out.isOpened()){
            std::cerr << "Error opening video capture" << std::endl;
        }

        // Start the thread to read frames from the video stream
        thread = std::thread(&RTSPVideoPublishObject::update, this);
        thread.detach();  // Detach the thread to avoid resource leaks
    }
private:
    void update() {
        // Read the next frame from the stream in a separate thread

        double desired_frame_time,sleep_duration;

        while (true) {
            // Simulate frame processing (replace with your actual processing logic)
            std::this_thread::sleep_for(std::chrono::milliseconds(20)); // Placeholder delay
            
            if(  out.isOpened()){
                out.write(frame);
                std::cerr << "write in server" << std::endl;
            }else {
                    std::cerr << "can't write" << std::endl;
                    break;
                }
            // Print timestamp (consider using C++20 formatted time strings for efficiency)
            now = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = now - start;
            desired_frame_time = 1.0 / fps;

            // Calculate sleep duration to maintain target FPS
            sleep_duration = desired_frame_time - elapsed.count();
            if (sleep_duration > 0) {
                std::this_thread::sleep_for(std::chrono::duration<double>(sleep_duration));
            } else {
                // Handle potential frame drops (optional)
                std::cerr << "Warning: Frame dropped due to slow processing." << std::endl;
            }

            start = now;  // Update start time for next iteration

            // Print message (consider using a formatted string with time for efficiency)
            std::ostringstream message;
            message << std::chrono::system_clock::to_time_t(now) << " frame written to the server";
            std::cout << message.str() << std::endl;
        }

    }
public:
    void update_frame(cv::Mat input_frame) {
        // Display frames in main program
        cv::resize(input_frame, frame, cv::Size(width, height), 0, 0, cv::INTER_LINEAR);
    }
private:
    cv::VideoWriter out;
    bool status;
    cv::Mat frame;
    int width, height,fps;
    std::thread thread;
    std::chrono::high_resolution_clock::time_point start,now;
};

int main(int, char**)
{
    std::string  rtspInput = "rtsp://10.42.0.230:8554/visible";
    //std::string  rtspInput = "rtsp://127.0.0.1:8554/test";
    std::string  rtspOutput = "rtsp://0.0.0.0:8554/visible2";

    std::cout << "Hello World!\n";
    std::cout << "rstp input " << rtspInput <<  "\n";

    RTSPVideoWriterObject myObj(rtspInput);
    RTSPVideoPublishObject myObj2(rtspOutput);

    cv::Mat auxframe;
    bool status = false;

    while (true) {
        myObj.show_frame();
        status = myObj.getStatus();
        if (status) {
            auxframe = myObj.getFrame();
            myObj2.update_frame(auxframe);
        }
    }


}


