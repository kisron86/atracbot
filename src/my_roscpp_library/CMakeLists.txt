cmake_minimum_required(VERSION 2.8.3)
project(my_roscpp_library)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  OpenCV
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
#  DEPENDS other non-ROS libs
)
SET(OpenCV_DIR /home/kisron/installation/OpenCV-3.4/share/OpenCV/)
include_directories(include ${catkin_INCLUDE_DIRS}) 
include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/my_super_roscpp_library.cpp
  src/my_hog.cpp
  #src/stereograb.cpp
  src/stereofunction.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
#add_executable(${PROJECT_NAME} src/my_hog.cpp)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
