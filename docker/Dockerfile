FROM osrf/ros:noetic-desktop-full-focal


# ENV ROS_PYTHON_VERSION=3

# Install libs
RUN apt-get update && apt-get install -y \
    libopencv-dev \
    liblua5.2-dev \
    virtualenv \
    screen \
    libpython3-dev \
    python3-rosdep \
    python3-rosinstall-generator \
    python3-vcstool \
    build-essential \
    python3-dev \
    git \
    ssh \
    ros-noetic-tf2-geometry-msgs \
    python3-tf2-sensor-msgs \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-tf2-msgs \
    python3-pcl \
    ros-noetic-navigation \
    ros-noetic-rviz \
    ros-noetic-pcl-ros \
    ros-noetic-gazebo-ros

WORKDIR /usr

# RUN apt-get update && \
#     apt-get install -y --no-install-recommends \
#     libpython3-dev \
#     python3-rosdep \
#     python3-rosinstall-generator \
#     python3-vcstool \
#     build-essential && \
#     rosdep init && \
#     rosdep update && 
#     rm -rf /var/lib/apt/lists/*

#Creating virtualenv for python3
RUN virtualenv /venv_p3 --python=python3
RUN /venv_p3/bin/pip install \
    pyyaml \
    rospkg \
    catkin_pkg \
    exception \
    numpy \
    # tensorflow=="1.13.1" \
    gym==0.15.7 \
    pyquaternion \ 
    mpi4py \
    matplotlib

# Add catkin repos
RUN mkdir -p /usr/catkin_ws/src/drl_local_planner_forks
RUN cd /usr/catkin_ws/src/drl_local_planner_forks && \
   git clone https://github.com/Steplerchik/flatland.git flatland --branch feature/noetic && \
   git clone https://github.com/Steplerchik/pedsim_ros.git pedsim --branch RGring-rl_features && \
   git clone https://github.com/RGring/navigation.git navigation --branch flatland_integration
#    git clone https://github.com/Steplerchik/stable-baselines.git stable-baselines --branch added_1d_conv_layer && \
#    /venv_p3/bin/pip install -e /usr/catkin_ws/src/drl_local_planner_forks/stable-baselines
