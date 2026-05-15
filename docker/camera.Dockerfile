FROM ros:humble

ENV DEBIAN_FRONTEND="noninteractive"
ENV TZ="UTC"
ENV ROS_DISTRO=humble
ENV DISPLAY=:0
ENV LIBGL_ALWAYS_INDIRECT=0

RUN apt-get update && apt-get install -yy \
    python3-pip \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-pcl-conversions \
    libpcl-dev \
    terminator \
    git \
    gedit \
  && rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade pip

RUN apt update
RUN apt install ros-${ROS_DISTRO}-rosidl-default-generators -y

WORKDIR /ros2_ws/src
RUN apt install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp -y

RUN apt-get update && apt-get install -y \
    libxinerama-dev libxcursor-dev libxrandr-dev libxi-dev libglfw3-dev \
    libglu1-mesa-dev freeglut3-dev mesa-common-dev

RUN pip install --upgrade pyrealsense2 numpy opencv-python
WORKDIR /
RUN git clone https://github.com/IntelRealSense/librealsense.git
WORKDIR /librealsense
RUN apt install v4l-utils -y
RUN ./scripts/setup_udev_rules.sh
RUN mkdir build && cd build && \
    cmake ../ -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=Release && \
    make -j4 && make install
    
WORKDIR /ros2_ws/src
RUN git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master

RUN apt-get install ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-diagnostic-updater ros-${ROS_DISTRO}-image-transport-plugins -y

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

RUN echo "export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH" >> /root/.bashrc

RUN sysctl net.ipv4.ipfrag_time=3
RUN sysctl net.ipv4.ipfrag_high_thresh=134217728

RUN echo "export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA?max_msg_size=1MB&soets_size=1MB&non_blocking=true&tcp_negotiation_timeout=50" >> ~/.bashrc

RUN echo "export ROS_DOMAIN_ID=1" >> ~/.bashrc


CMD ["bash"]
