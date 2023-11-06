cmake_minimum_required(VERSION 2.8.3)
project(rl_bringup)

set(PACKAGE_DEPS
  roscpp
  rospy
  std_msgs
  nav_msgs
  tf
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPS})

# Create virtualenv
set(CREATE_PATH_CFG_CMD "${CMAKE_CURRENT_SOURCE_DIR}/../create_path_config.sh")
MESSAGE("Executing script creating the path configuration for rl_bringup package: '${CREATE_PATH_CFG_CMD}'")
execute_process(COMMAND ${CREATE_PATH_CFG_CMD})

catkin_package(
  CATKIN_DEPENDS ${PACKAGE_DEPS}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config/
)
