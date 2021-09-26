cmake_minimum_required(VERSION 2.8.3)

project(commander)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  cv_bridge
  std_msgs
)

find_package(OpenCV 4 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  sensor_msgs
  cv_bridge
  std_msgs
)

#catkin_install_python(PROGRAMS 
#  scripts/shape_detector.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${src}
  ${OpenCV_INCLUDE_DIRS}
)

