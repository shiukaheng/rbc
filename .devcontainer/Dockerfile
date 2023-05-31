FROM shiukaheng/ros-base:latest

ENTRYPOINT [ "/usr/bin/env" ]

RUN apt-get update && sudo apt-get install -y \
    python3-catkin-tools\
    nano\
    curl\
    git\
    ros-noetic-rosserial\
    ros-noetic-rosserial-arduino\
    ros-noetic-rqt-ez-publisher

RUN mkdir -p /root/catkin_ws/src

# Go to user home
WORKDIR /root
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
RUN /root/bin/arduino-cli core update-index
RUN /root/bin/arduino-cli core install arduino:avr
RUN /root/bin/arduino-cli board install arduino:avr:mega
RUN /root/bin/arduino-cli lib install "FireTimer"
RUN /root/bin/arduino-cli lib install "ArduPID"

# Initialize rosserial
WORKDIR /root/Arduino/libraries
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && rosrun rosserial_arduino make_libraries.py ."

RUN apt-get install x11-apps -y