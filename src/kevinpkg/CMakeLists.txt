cmake_minimum_required(VERSION 2.8.3)
project(kevinpkg)

set(OpenCV_DIR /usr/share/OpenCV/)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  image_transport
  OpenCV
  sensor_msgs
  cv_bridge
  pcl_ros
  pcl_conversions
)

find_package(OpenCV)
find_package(PCL REQUIRED)

catkin_package(
)

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
    ${PCL_LIBRARY_DIRS}
)

add_executable(stereoCapture src/stereoCapture.cpp)
add_dependencies(stereoCapture kevinpkg)
target_link_libraries(stereoCapture ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenCV_LIBS})

add_executable(stereoNode src/stereoNode.cpp)
add_dependencies(stereoNode kevinpkg)
target_link_libraries(stereoNode ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(img_writer src/img_writer.cpp)
add_dependencies(img_writer kevinpkg)
target_link_libraries(img_writer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(detection1 src/detection1.cpp)
add_dependencies(detection1 kevinpkg)
target_link_libraries(detection1 ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(show_fps src/show_fps.cpp)
add_dependencies(show_fps kevinpkg)
target_link_libraries(show_fps ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
