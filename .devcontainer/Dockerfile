FROM shiukaheng/ros-base:latest

ENTRYPOINT [ "/usr/bin/env" ]

# Add ros as user, without password, and add to sudo group
RUN adduser --disabled-password --gecos '' ros && \
    adduser ros sudo && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

RUN chown -R ros:ros /home/ros
RUN adduser ros dialout
RUN adduser ros tty

USER ros

RUN sudo apt-get update && sudo apt-get install -y \
    python3-catkin-tools\
    nano\
    curl\
    git\
    ros-noetic-rosserial\
    ros-noetic-rosserial-arduino

# Should be redundant, but just in case
RUN mkdir -p /home/ros/catkin_ws/src
RUN mkdir -p /home/ros/catkin_ws/devel
RUN mkdir -p /home/ros/catkin_ws/build
RUN mkdir -p /home/ros/catkin_ws/install
RUN mkdir -p /home/ros/catkin_ws/logs
RUN mkdir -p /home/ros/catkin_ws/.catkin_tools
RUN mkdir -p /home/ros/robocock

# Go to user home
WORKDIR /home/ros
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
RUN /home/ros/bin/arduino-cli core update-index
RUN /home/ros/bin/arduino-cli core install arduino:avr
RUN /home/ros/bin/arduino-cli board install arduino:avr:mega
RUN /home/ros/bin/arduino-cli lib install "FireTimer"
RUN /home/ros/bin/arduino-cli lib install "ArduPID"

# Initialize rosserial
WORKDIR /home/ros/Arduino/libraries
RUN rosrun rosserial_arduino make_libraries.py .
