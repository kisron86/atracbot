cmake_minimum_required(VERSION 2.8.3)
project(minorucam)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(catkin REQUIRED COMPONENTS roscpp image_transport cv_bridge)
find_package(PCL REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )


# let cmake find OpenMP and set some variables
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
  set(OpenMP_LIBS gomp)
endif()

# list OPENCV libraries
set( OpenCV_LIBS opencv_calib3d opencv_contrib opencv_core opencv_features2d opencv_flann opencv_gpu opencv_highgui opencv_imgproc opencv_legacy opencv_ml opencv_objdetect opencv_ocl opencv_photo opencv_stitching opencv_superres opencv_ts opencv_video opencv_videostab )

set( CMAKE_CXX_FLAGS "" )   #"-Wall -O3"

## check c++11 / c++0x
#include(CheckCXXCompilerFlag)
#CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
#CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
#if(COMPILER_SUPPORTS_CXX11)
#    message(STATUS "Enabling c++11")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#elseif(COMPILER_SUPPORTS_CXX0X)
#    message(STATUS "Enabling c++0x")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#else()
#    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
#endif()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES minorucam
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS OpenMP
)

###########
## Build ##
###########

#message(STATUS "PCL include: ${PCL_INCLUDE_DIRS}")
#message(STATUS "PCL library: ${PCL_LIBRARY_DIRS}")
#message(STATUS "PCL definitions: ${PCL_DEFINITIONS}")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
   ${catkin_INCLUDE_DIRS}
   ${PCL_INCLUDE_DIRS}
   /usr/local/include/opencv2
   /usr/local/include/opencv
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


file(GLOB_RECURSE V4L2STEREO_SRC_FILES src/v4l2stereo_no_sse/src/*.cpp)
list(REMOVE_ITEM  V4L2STEREO_SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/v4l2stereo_no_sse/src/main.cpp)

## Declare a C++ library
# add_library(minorucam
#   src/${PROJECT_NAME}/minorucam.cpp
# )
#add_library(v4l2stereo
#   ${V4L2STEREO_SRC_FILES}
#)

## Declare a C++ executable
#add_executable(minorucam_node src/stereocam_broadcast.cpp
##   ${V4L2STEREO_SRC_FILES}
#)

#add_executable(minorucam_disp src/stereocam_disparity.cpp
#   ${V4L2STEREO_SRC_FILES}
#)

#add_executable(minorucam_cloud src/stereocam_pointcloud.cpp
##   ${V4L2STEREO_SRC_FILES}
#)

#add_executable(minorucam_laser src/stereocam_laserscan.cpp
#   ${V4L2STEREO_SRC_FILES}
#)

add_executable(minorucam src/stereocamera.cpp
   ${V4L2STEREO_SRC_FILES}
)

# compile flags and options
#target_compile_options(v4l2stereo PRIVATE ${OpenMP_FLAGS} -O3)
#target_compile_options(minorucam_node PRIVATE ${OpenMP_FLAGS} -O3)  #-Wall -O3
#target_compile_options(minorucam_disp PRIVATE ${OpenMP_FLAGS} -O3)
#target_compile_options(minorucam_cloud PRIVATE ${OpenMP_FLAGS} -O3)
#target_compile_options(minorucam_laser PRIVATE ${OpenMP_FLAGS} -O3)
target_compile_options(minorucam PRIVATE ${OpenMP_FLAGS} -O3)

# C++ standard
#set_property(TARGET v4l2stereo PROPERTY CXX_STANDARD 11)
#set_property(TARGET minorucam_node PROPERTY CXX_STANDARD 11)
#set_property(TARGET minorucam_disp PROPERTY CXX_STANDARD 11)
#set_property(TARGET minorucam_cloud PROPERTY CXX_STANDARD 11)
#set_property(TARGET minorucam_laser PROPERTY CXX_STANDARD 11)
set_property(TARGET minorucam PROPERTY CXX_STANDARD 11)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(v4l2stereo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(minorucam_disp v4l2stereo)
#add_dependencies(minorucam_disp v4l2stereo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(minorucam_cloud v4l2stereo)
#add_dependencies(minorucam_cloud v4l2stereo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(minorucam_laser v4l2stereo)

## Specify libraries to link a library or executable target against
#target_link_libraries(v4l2stereo
#   ${OpenMP_LIBS}
#   ${OpenCV_LIBS}
#)

#target_link_libraries(minorucam_node
#   ${catkin_LIBRARIES}
#   ${OpenMP_LIBS}
#   ${OpenCV_LIBS}
#   v4l2stereo
#)
#target_link_libraries(minorucam_disp
#   ${catkin_LIBRARIES}
#   ${OpenMP_LIBS}
#   ${OpenCV_LIBS}
##   v4l2stereo
#)
#target_link_libraries(minorucam_cloud
#   ${catkin_LIBRARIES}
#   ${OpenMP_LIBS}
#   ${OpenCV_LIBS}
#   ${PCL_LIBRARIES}
#   v4l2stereo
#)
#target_link_libraries(minorucam_laser
#   ${catkin_LIBRARIES}
#   ${OpenMP_LIBS}
#   ${OpenCV_LIBS}
##   v4l2stereo
#)
target_link_libraries(minorucam
   ${catkin_LIBRARIES}
   ${OpenMP_LIBS}
   ${OpenCV_LIBS}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS minorucam minorucam_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_minorucam.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

message(STATUS "CMAKE FLAGS: ${CMAKE_CXX_FLAGS}")
