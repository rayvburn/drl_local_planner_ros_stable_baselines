FROM ros:kinetic-robot-xenial

RUN apt-get update && apt-get install -y \
  libqt4-dev \
  libopencv-dev \
  liblua5.2-dev \
  virtualenv \
  screen \
  python3-dev \
  git \
  ros-kinetic-tf2-geometry-msgs \
  ros-kinetic-navigation \
  ros-kinetic-rviz 

# libffi-dev -> ModuleNotFoundError: No module named '_ctypes'
# https://github.com/pypa/packaging-problems/issues/573 (when installing gym)
RUN apt-get update && apt-get install -y \
  make \
  build-essential \
  libssl-dev \
  zlib1g-dev \
  libbz2-dev \
  libreadline-dev \
  libsqlite3-dev \
  wget \
  curl \
  llvm \
  libncursesw5-dev \
  xz-utils \
  tk-dev \
  libxml2-dev \
  libxmlsec1-dev \
  libffi-dev \
  liblzma-dev

# Install newer version of Python (than available in apt) https://phoenixnap.com/kb/how-to-install-python-3-ubuntu
RUN apt-get install -y wget
RUN wget https://www.python.org/ftp/python/3.7.9/Python-3.7.9.tgz
RUN tar -xf Python-3.7.9.tgz
RUN cd Python-3.7.9 && ./configure --enable-optimizations
RUN cd Python-3.7.9 && make altinstall

# Building catkin_ws
RUN mkdir -p /usr/catkin_ws/src
WORKDIR /usr/catkin_ws/src
RUN git clone https://github.com/RGring/drl_local_planner_ros_stable_baselines
RUN cp drl_local_planner_ros_stable_baselines/.rosinstall .
RUN rosws update
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; cd /usr/catkin_ws; catkin_make'

#Creating virtualenv for python3
# can't use virtualenv here (installed version is incompatible with the P3.7)
RUN python3.7 -m venv /venv_p3

RUN /venv_p3/bin/pip install --upgrade pip
RUN /venv_p3/bin/pip install Cython
RUN /venv_p3/bin/pip install \
    pyyaml=="5.1" \
    rospkg=="1.1.7" \
    catkin_pkg \
    exception=="0.1.0" \
    numpy=="1.17.0" \
    tensorflow=="1.13.1" \
    protobuf=="3.7.1" \
    gym=="0.12.1" \
    pyquaternion=="0.9.5" \
    mpi4py=="3.0.1" \
    matplotlib=="3.0.3"
RUN /venv_p3/bin/pip install -e /usr/catkin_ws/src/drl_local_planner_forks/stable-baselines


RUN mv /usr/catkin_ws/src/drl_local_planner_ros_stable_baselines/rl_bringup/config/path_config_docker.ini \
/usr/catkin_ws/src/drl_local_planner_ros_stable_baselines/rl_bringup/config/path_config.ini

WORKDIR /usr/catkin_ws/src/drl_local_planner_ros_stable_baselines/start_scripts/

ENTRYPOINT ./entrypoint_ppo2.sh "$AGENT_NAME" "$NUM_SIM_ENVS"
