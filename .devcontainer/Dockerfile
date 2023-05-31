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
    x11-apps

RUN mkdir -p /root/catkin_ws/src

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
RUN ln -s /root/robocock/ros /root/catkin_ws/src