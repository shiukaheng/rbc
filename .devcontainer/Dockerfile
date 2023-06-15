# This is an auto generated Dockerfile for ros:ros-core
# generated from docker_images/create_ros_core_image.Dockerfile.em
FROM ubuntu:focal

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO noetic

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-core=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-robot=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

### END ROS ###

ENTRYPOINT [ "/usr/bin/env" ]
SHELL [ "/bin/bash", "-c" ]

RUN apt-get update
RUN apt-get install -y python3-catkin-tools nano curl git x11-apps
RUN apt-get install -y ros-noetic-rosserial ros-noetic-rosserial-arduino ros-noetic-rqt-ez-publisher ros-noetic-ros-control ros-noetic-ros-controllers libeigen3-dev

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