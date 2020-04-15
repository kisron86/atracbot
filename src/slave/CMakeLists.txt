cmake_minimum_required(VERSION 2.8.3)
project(slave)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(prediction_system src/prediction_system.cpp)
add_dependencies(prediction_system slave)
target_link_libraries(prediction_system ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenCV_LIBS})

add_executable(publisher src/publisher.cpp)
add_dependencies(publisher slave)
target_link_libraries(publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenCV_LIBS})

add_executable(read_sensor src/read_sensor.cpp)
add_dependencies(read_sensor slave)
target_link_libraries(read_sensor ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenCV_LIBS})

add_executable(data_conversion src/data_conversion.cpp)
add_dependencies(data_conversion slave)
target_link_libraries(data_conversion ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenCV_LIBS})

add_executable(decision_making_system src/decision_making_system.cpp)
add_dependencies(decision_making_system slave)
target_link_libraries(decision_making_system ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenCV_LIBS})

#add_executable(talker_py src/talker_py.py)
#add_dependencies(talker_py slave)
#target_link_libraries(talker_py ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenCV_LIBS})
