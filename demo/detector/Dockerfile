ARG ROS_IMG=ros:noetic

FROM ${ROS_IMG}

ARG ROS_PKG=commander

WORKDIR /root
COPY ./${ROS_PKG} /root/src/${ROS_PKG}

RUN apt-get update -y &&\
    apt-get install -y \
        ros-noetic-rosbridge-server \
        ros-noetic-cv-bridge \
        curl &&\
    echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | tee /etc/apt/sources.list.d/coral-edgetpu.list &&\
    curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | apt-key add - &&\
    apt-get update -y &&\
    apt-get install -y \
        python3-tflite-runtime \
        python3-pycoral \ 
        python3-pip &&\
    pip3 install tensorflow keras &&\
    chmod +x /root/src/${ROS_PKG}/scripts &&\
    . /opt/ros/noetic/setup.sh &&\
    catkin_make &&\
    rm -rf /var/lib/apt/lists/*

ENV PKG_NAME=${ROS_PKG}
CMD . ./devel/setup.sh &&\
    (rosrun ${PKG_NAME} toy_detection.py camera keras/model.h5 tf/model.tflite keras/labels.txt &\
    roslaunch --wait rosbridge_server rosbridge_websocket.launch port:=8080)