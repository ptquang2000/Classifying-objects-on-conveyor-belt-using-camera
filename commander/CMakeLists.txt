cmake_minimum_required(VERSION 2.8.3)

project(commander)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  cv_bridge
  std_msgs
  message_generation
)

find_package(OpenCV 4 REQUIRED)

add_service_files(
  FILES
  image_capture_srv.srv
)

add_message_files(
  FILES
  image_details_msg.msg
  toy_score_msg.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  sensor_msgs
  cv_bridge
  std_msgs
  message_runtime
)

catkin_install_python(PROGRAMS 
  scripts/image_capture.py
  scripts/toy_detection.py
  scripts/image_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${src}
  ${OpenCV_INCLUDE_DIRS}
)

