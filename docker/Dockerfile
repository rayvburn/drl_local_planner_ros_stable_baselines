FROM osrf/ros:melodic-desktop-full

# Install libs
RUN apt-get update && apt-get install -y \
    libqt4-dev \
    libopencv-dev \
    liblua5.2-dev \
    virtualenv \
    screen \
    python3-dev \
    git \
    ssh \
    ros-melodic-tf2-geometry-msgs \
    ros-melodic-navigation \
    ros-melodic-rviz \
    ros-melodic-pcl-ros \
    ros-melodic-gazebo-ros

WORKDIR /usr

#Creating virtualenv for python3
RUN virtualenv /venv_p3 --python=python3
RUN /venv_p3/bin/pip install \
    pyyaml \
    rospkg \
    catkin_pkg \
    exception \
    numpy \
    tensorflow=="1.13.1" \
    gym==0.15.7 \
    pyquaternion \ 
    mpi4py \
    matplotlib

# Add catkin repos
RUN mkdir -p /usr/catkin_ws/src/drl_local_planner_forks
RUN cd /usr/catkin_ws/src/drl_local_planner_forks && \
    git clone https://github.com/Steplerchik/flatland.git flatland --branch pedsim_integration && \
    git clone https://github.com/Steplerchik/pedsim_ros.git pedsim --branch RGring-rl_features && \
    git clone https://github.com/RGring/navigation.git navigation --branch flatland_integration && \
    git clone https://github.com/Steplerchik/stable-baselines.git stable-baselines --branch added_1d_conv_layer && \
    /venv_p3/bin/pip install -e /usr/catkin_ws/src/drl_local_planner_forks/stable-baselines