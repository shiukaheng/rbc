FROM shiukaheng/ros-base:latest

ENTRYPOINT [ "/usr/bin/env" ]

# Add ros as user, without password, and add to sudo group
RUN adduser --disabled-password --gecos '' ros && \
    adduser ros sudo && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER ros

RUN sudo apt-get update && sudo apt-get install -y \
    python3-catkin-tools\
    nano\
    curl\
    git

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