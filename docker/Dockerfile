FROM osrf/ros:melodic-desktop-full

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

# Building catkin_ws
RUN mkdir -p /usr/catkin_ws/src
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
