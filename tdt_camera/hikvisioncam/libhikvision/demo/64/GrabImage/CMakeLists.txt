cmake_minimum_required(VERSION 3.12)
project(GrabImage)

set(CMAKE_CXX_STANDARD 14)
INCLUDE_DIRECTORIES("/home/greed/桌面/test/HikVision/ImageProcess")
LINK_DIRECTORIES("/home/greed/视觉/CODE_SHENZHEN/HERO/TDT_Soldier/HikVision/lib64")
include_directories(.)

add_executable(GrabImage
        GrabImage.cpp
        Makefile)
target_link_libraries(GrabImage -pthread ${OpenCV_LIBS} libMvCameraControl.so libMVGigEVisionSDK.so libMVRender.so  libMvUsb3vTL.so -lpthread -lm)
