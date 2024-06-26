cmake_minimum_required(VERSION 3.0.0)
project(forwardRTSPvideo VERSION 0.1.0 LANGUAGES C CXX)

find_package(OpenCV)
include_directories( ${OpenCV_INCLUDE_DIRS} )


add_executable(solid main.cpp)
target_link_libraries( solid ${OpenCV_LIBS} )
