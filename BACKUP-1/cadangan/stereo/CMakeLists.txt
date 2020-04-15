cmake_minimum_required(VERSION 2.8.3)
project(stereo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_filters
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  visualization_msgs
)

find_package(OpenCV)
find_package(PCL REQUIRED)

catkin_package(
)



###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
include_directories(${OpenCV_INCLUDE_DIRS})


link_directories(
    ${PCL_LIBRARY_DIRS}
)

## Declare a C++ executable
add_executable(stereo_node src/stereo_node.cpp)
add_executable(calib_camera src/calib_camera.cpp)
add_executable(stereo_tf_broadcaster src/stereo_tf_broadcaster.cpp)
add_executable(draw_boxes src/draw_boxes.cpp)

add_dependencies(stereo_node stereo)
add_dependencies(draw_boxes stereo)
add_dependencies(stereo_tf_broadcaster stereo)
add_dependencies(calib_camera stereo)


## Specify libraries to link a library or executable target against
target_link_libraries(stereo_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_link_libraries(calib_camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_link_libraries(stereo_tf_broadcaster ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_link_libraries(draw_boxes ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

