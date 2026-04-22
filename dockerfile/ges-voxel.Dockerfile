# ros noetic
FROM ros:noetic

# ROS Install dependencies (libceres-dev removed — STD requires Ceres >= 2.1.0, built from source below)
RUN apt-get update && apt-get install -y --no-install-recommends \
    git wget nano unzip build-essential cmake python3-pip libomp-dev libmetis-dev \
    ros-noetic-pcl-ros ros-noetic-eigen-conversions \
    ros-noetic-tf ros-noetic-tf2-ros ros-noetic-tf2-eigen \
    ros-noetic-message-filters ros-noetic-nav-msgs ros-noetic-geometry-msgs \
    ros-noetic-rviz \
    ros-noetic-cv-bridge ros-noetic-tf-conversions ros-noetic-pcl-conversions \
    libsuitesparse-dev \
    libgoogle-glog-dev libgflags-dev \
    python3-catkin-tools \
    libopencv-dev \
    python3-tk \
    software-properties-common \
    && apt-get clean && rm -rf /var/lib/apt/lists/*
RUN apt install curl gnupg2 lsb-release -y

# Install GTSAM 4.0 (STD dependency — do NOT use develop branch)
RUN add-apt-repository ppa:borglab/gtsam-release-4.0 \
    && apt-get update \
    && apt-get install -y libgtsam-dev libgtsam-unstable-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Ceres 2.1.0 from source (STD requires >= 2.1.0)
RUN git clone https://ceres-solver.googlesource.com/ceres-solver /tmp/ceres \
    && cd /tmp/ceres && git checkout 2.1.0 \
    && mkdir build && cd build \
    && cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF \
    && make -j$(nproc) && make install \
    && rm -rf /tmp/ceres

# Install Livox-SDK
WORKDIR /root
RUN git clone https://github.com/Livox-SDK/Livox-SDK.git
WORKDIR /root/Livox-SDK/build
RUN cmake .. && make -j$(nproc) install

# Create catkin workspace
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_init_workspace'

# Clone Repositories
RUN git clone --recursive https://github.com/Livox-SDK/livox_ros_driver.git \
    && git clone --recursive https://github.com/hku-mars/FAST_LIO.git 

WORKDIR /root/catkin_ws/src/FAST_LIO
RUN rm -rf include/ikd-Tree \
    && git clone https://github.com/hku-mars/ikd-Tree.git include/ikd-Tree \
    && cp include/ikd-Tree/ikd-Tree/* include/ikd-Tree/ \
    && rm -rf include/ikd-Tree/ikd-Tree

# .bashrc setup
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

WORKDIR /root/catkin_ws/src/FAST_LIO
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
