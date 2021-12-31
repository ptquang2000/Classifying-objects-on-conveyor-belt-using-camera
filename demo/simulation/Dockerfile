ARG ROS_IMG=ros:noetic

FROM gazebo:libgazebo11 as gz

FROM ${ROS_IMG}

ARG ROS_PKG=sim_env

WORKDIR /root
COPY ./${ROS_PKG} /root/src/${ROS_PKG}
COPY --from=gz /usr/bin/gazebo /usr/bin/gazebo

RUN apt-get update -y &&\
    apt-get install -y \
        ros-noetic-gazebo-ros-pkgs \
        ros-noetic-gazebo-ros-control \
        xvfb &&\
    chmod +x /root/src/${ROS_PKG}/scripts &&\
    . /opt/ros/noetic/setup.sh &&\
    catkin_make &&\
    rm -rf /var/lib/apt/lists/*

ENV PKG_NAME=${ROS_PKG}
CMD Xvfb -shmem -screen 0 1280x1024x24 &\
    . ./devel/setup.sh &&\
    (roslaunch --wait ${PKG_NAME} conveyor_belt.launch &\
    rosrun ${PKG_NAME} object_spawner.py toy)