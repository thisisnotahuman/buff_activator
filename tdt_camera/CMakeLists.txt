#cmake_minimum_required(VERSION 3.10)
project(tdtcamera)

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV REQUIRED)
add_subdirectory(hikvisioncam)
add_subdirectory(uvccam)
find_package(yaml-cpp REQUIRED)
include_directories(${yaml-cpp_INCLUDE_DIRS})
add_library(tdtcamera SHARED tdtcamera.cpp)
target_link_libraries(tdtcamera PRIVATE  tdtcommon ${OpenCV_LIBS} ${yaml-cpp_LIBS} hikvisioncam uvccam -lpthread -lm)