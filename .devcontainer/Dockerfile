FROM shiukaheng/ros-base:latest

ENTRYPOINT [ "/usr/bin/env" ]

RUN apt-get update && sudo apt-get install -y \
    python3-catkin-tools\
    nano\
    curl\
    git\
    ros-noetic-rosserial\
    ros-noetic-rosserial-arduino\
    ros-noetic-rqt-ez-publisher\
    ros-noetic-ros-control\
    ros-noetic-ros-controllers\
    x11-apps\
    libeigen3-dev

# Astra Camera
RUN apt-get install -y ros-noetic-rgbd-launch ros-noetic-libuvc-camera ros-noetic-libuvc-ros libuvc-dev
RUN apt-get install -y libgoogle-glog-dev


WORKDIR /root
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
RUN /root/bin/arduino-cli core update-index
RUN /root/bin/arduino-cli core install arduino:avr
RUN /root/bin/arduino-cli board install arduino:avr:mega
RUN /root/bin/arduino-cli lib install "FireTimer"
RUN /root/bin/arduino-cli lib install "ArduPID"

# Copying source code over
COPY .. /root/robocock
COPY ./.bashrc /root/.bashrc
RUN mkdir -p /root/catkin_ws
WORKDIR /root/catkin_ws
RUN ln -s /root/robocock/ros src