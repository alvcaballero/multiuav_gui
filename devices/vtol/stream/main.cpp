#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <chrono>
#include <thread>
#include <ctime>
#include <sstream>


//std::string  rtspInput = "rtsp://10.42.0.230:8554/visible";
std::string  rtspInput = "rtsp://127.0.0.1:8554/test";

std::string  rtspOutput = "rtsp://0.0.0.0:8554/visible2";

cv::Mat myframe = cv::Mat(640, 480, CV_8UC3);;

bool _stream = new bool(true);
const double fps = 15.0;  // Target frames per second


using namespace cv;


void GetRTPSImg()
{

    //cv::VideoCapture cap(_ssrc.toStdString());
    std ::string addr("rtspsrc location=" + rtspInput + " latency=300 caps = \"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96\" ! rtph264depay ! decodebin ! videoconvert ! appsink");
    //std ::string addr("rtspsrc location=" + rtspInput + "  ! rtph264depay ! avdec_h264 ! decodebin ! videoconvert ! appsink");
    //std ::string addr( rtspInput );
    
    std::cout << "gst input " << addr;

    cv::VideoCapture cap(addr, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
         printf("no can open video stream\n");
        _stream = false;
        return;
    }
    // we will use a QLabel to show the video
    cv::Mat frame;
    while(_stream) {
        cap >> frame;
        //cap.read(frame);
        // std::cout << frame.size() << std::endl; 
        if (frame.empty()) {
            printf("Empty frame \n");
            break;
        }        
        
        // resize the image to 640x480
        cv::resize(frame, myframe, cv::Size(640, 480));
        //cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
    }
    
    cap.release();
    _stream = false;
    printf("Stop read rtsp input \n");
}

void sendRSTP(){
    cv::VideoWriter out;

    std::string gstpipeline = std::string("appsrc ! videoconvert") +
    " ! video/x-raw,format=I420" + 
    " ! x264enc speed-preset=ultrafast bitrate=600 key-int-max=" + std::to_string(fps * 2) +
    " ! video/x-h264,profile=baseline" + 
    " ! rtspclientsink location="+rtspOutput;
    
    out.open(gstpipeline,
    cv::CAP_GSTREAMER,
     0, 
    fps, 
    cv::Size(640, 480), 
    true);

    if( ! out.isOpened()){
        printf("no can open video stream\n");
        _stream = false;
        return;
    
    }

    auto start = std::chrono::high_resolution_clock::now();
        auto now = std::chrono::high_resolution_clock::now();

    double desired_frame_time,sleep_duration;

    while (true) {
        // Simulate frame processing (replace with your actual processing logic)
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // Placeholder delay
        
        
        out.write(myframe);


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

int main(int, char**)
{
    std::thread thread_obj(GetRTPSImg);
    std::thread thread_obj1(sendRSTP);

    std::cout << "Hello World!\n";
    std::cout << "rstp input " << rtspInput <<  "\n";

    thread_obj.join();
    thread_obj1.join();

}


