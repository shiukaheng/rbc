FROM shiukaheng/ros-base:latest

ENTRYPOINT [ "/usr/bin/env" ]

RUN apt-get update
RUN apt-get install -y python3-catkin-tools nano curl git x11-apps
RUN apt-get install -y ros-noetic-rosserial ros-noetic-rosserial-arduino ros-noetic-rqt-ez-publisher ros-noetic-ros-control ros-noetic-ros-controllers libeigen3-dev

# Astra Camera
RUN apt-get install -y ros-noetic-rgbd-launch ros-noetic-libuvc-camera ros-noetic-libuvc-ros libuvc-dev
RUN apt-get install -y libgoogle-glog-dev

# LDS-01
RUN apt-get install -y ros-noetic-hls-lfcd-lds-driver

WORKDIR /root
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
RUN /root/bin/arduino-cli core update-index
RUN /root/bin/arduino-cli core install arduino:avr
RUN /root/bin/arduino-cli board install arduino:avr:mega
RUN /root/bin/arduino-cli lib install "FireTimer"

# Install slam_toolbox
RUN apt-get install -y ros-noetic-slam-toolbox

# Install move_base_flex
RUN apt-get install -y ros-noetic-move-base

# End of ROS dependencies

# Install OpenSSH for remote development
RUN apt-get install -y openssh-server
RUN service ssh start

# Copy in sshd_config file
COPY ./sshd_config /etc/ssh/sshd_config

# Install avahi-daemon for mDNS
RUN apt-get install -y avahi-daemon iputils-ping

# Copying source code over
COPY .. /root/rbc
COPY ./.bashrc /root/.bashrc
RUN mkdir -p /root/catkin_ws
WORKDIR /root/catkin_ws
RUN ln -s /root/rbc/ros src

# Set password for root to be roboc0ck
RUN echo 'root:roboc0ck' | chpasswd

# Remove ~/.dev
RUN rm -rf ~/.dev