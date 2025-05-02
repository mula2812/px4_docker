ARG BASE_IMAGE="ubuntu"
ARG TAG="22.04"
FROM ${BASE_IMAGE}:${TAG}

# Update package lists
RUN apt-get update

# Install essential packages
RUN apt-get install -y git curl gnupg lsb-release

# Add ROS 2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Update package lists again after adding ROS 2 repo
RUN apt-get update

# Install more dependencies
RUN apt-get install -y git-core unzip sudo wget 
# Instaltion of PX4
RUN git clone https://github.com/PX4/Firmware.git --recursive

WORKDIR /Firmware

RUN bash ./Tools/setup/ubuntu.sh

# Update package lists after install px4
RUN apt-get update

# Install required packages for gazebo simulation
RUN apt-get install -y aptitude gazebo libgazebo11 libgazebo-dev

# Add necessary dependencies for PX4 and ROS 2
RUN apt-get install -y python3-rosdep python3-pip python3-colcon-common-extensions

# instaltion for mavlink-router

# Install dependencies 
RUN apt-get install -y build-essential pkg-config git meson ninja-build systemd

# Clone and build mavlink-router
RUN git clone https://github.com/mavlink-router/mavlink-router.git
WORKDIR /Firmware/mavlink-router
RUN git submodule update --init --recursive && \
    meson setup build . && \
    ninja -C build && \
    ninja -C build install

# Ensure mavlink-router binaries are in PATH
ENV PATH="/Firmware/mavlink-router/build:${PATH}"

# Optional: Install screen if you want to run PX4 in a detachable session
# RUN apt-get install -y screen

# entry point dir for px4
WORKDIR /Firmware

# Compile PX4 SITL
RUN make px4_sitl all

RUN DONT_RUN=1 make px4_sitl gazebo-classic -j$(nproc)

# Set up environment variables
ENV PX4_HOME_LAT=47.397751
ENV PX4_HOME_LON=8.545607
ENV PX4_HOME_ALT=488.13123

# Set environment variables for UDP forwarding
ENV QGC_IP=10.0.0.16
ENV CONTROL_IP=10.0.0.16

ENV CONTROL_PORT=14540
ENV QGC_PORT=14550

ENV px4_start_command="make px4_sitl gazebo-classic"

CMD ["bash", "-c", "mavlink-routerd -e ${QGC_IP}:${QGC_PORT} -e ${CONTROL_IP}:${CONTROL_PORT} & export PX4_HOME_LAT=${PX4_HOME_LAT} && export PX4_HOME_LON=${PX4_HOME_LON} && export PX4_HOME_ALT=${PX4_HOME_ALT} && ${px4_start_command}"]