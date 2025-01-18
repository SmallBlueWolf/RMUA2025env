FROM osrf/ros:noetic-desktop-full-focal

ADD modules/abseil-cpp /uniarc_main/modules/abseil-cpp/
ADD modules/ceres-solver /uniarc_main/modules/ceres-solver/

USER root
ENV ROS_DISTRO=noetic
# 更新系统并安装基础依赖和 ROS 依赖
RUN apt update && apt install -y --fix-missing \
    python3-catkin-tools \
    ros-noetic-geographic-msgs \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-image-transport \
    net-tools \
    build-essential \
    cmake \
    git \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    wget

# 安装absl库
WORKDIR /uniarc_main/modules/abseil-cpp
RUN cmake -DBUILD_SHARED_LIBS=ON -L CMakeLists.txt && make
RUN make install

# 安装 Ceres Solver，用clone好的库，2.0.0
WORKDIR /uniarc_main/modules/ceres-solver
RUN mkdir build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DGTEST_LIBRARY=/usr/local/lib/libgtest.a -DGTEST_MAIN_LIBRARY=/usr/local/lib/libgtest_main.a -DGMOCK_LIBRARY=/usr/local/lib/libgmock.a -DGMOCK_MAIN_LIBRARY=/usr/local/lib/libgmock_main.a && \ 
    make -j4 && \
    make install

ADD src /uniarc_main/src/
ADD configs /uniarc_main/configs
ADD setup.bash /

RUN chmod +x /setup.bash

WORKDIR /uniarc_main/
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && catkin_make --only-pkg-with-deps airsim_ros && \
    . devel/setup.sh && catkin_make --only-pkg-with-deps quadrotor_msgs && \
    . devel/setup.sh && catkin_make --only-pkg-with-deps uniarc_main && \
    . devel/setup.sh && catkin_make --only-pkg-with-deps vins && \
    . devel/setup.sh && catkin_make --only-pkg-with-deps ego_planner

ENTRYPOINT [ "/setup.bash" ]