cmake_minimum_required(VERSION 2.8.3)
project(detectball)

find_package(OpenCV REQUIRED)
find_package(OpenCV COMPONENTS core highgui video imgproc REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_BINARY_DIR})

add_executable( detectball detectball.cpp )
target_link_libraries( detectball ${OpenCV_LIBS} ${OpenCV_LIBRARIES})