cmake_minimum_required(VERSION 2.8.3)

project(sim_env)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  gazebo_ros
)

# Depending on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
)

#catkin_install_python(PROGRAMS 
#  scripts/object_spawner.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${src}
  ${GAZEBO_INCLUDE_DIRS}
)

# Gazebo set flags
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

#add_library(hello_world SHARED src/hello_world.cc)
#target_link_libraries(hello_world 
#  ${catkin_LIBRARIES}
#  ${GAZEBO_LIBRARIES}
#)
add_library(model_push SHARED src/model_push.cc)
target_link_libraries(model_push 
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
add_library(gazebo_ros_camera SHARED src/gazebo_ros_camera.cc)
target_link_libraries(gazebo_ros_camera 
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
#add_library(factory SHARED src/factory.cc)
#target_link_libraries(factory 
#  ${catkin_LIBRARIES}
#  ${GAZEBO_LIBRARIES}
#)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

