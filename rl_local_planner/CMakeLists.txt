cmake_minimum_required(VERSION 2.8.3)
project(rl_local_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  base_local_planner
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  nav_msgs
  nav_core
  laser_geometry
  geometry_msgs
  message_generation
  geometry_msgs
  rl_msgs
  pcl_ros
)

# see here: https://github.com/PointCloudLibrary/pcl/issues/3680
# when this is fixed, we can remove the following 3 lines.
if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
     set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES
    rlLocalPlanner
  CATKIN_DEPENDS
    base_local_planner
    costmap_2d
    nav_msgs
    nav_core
    roscpp
    std_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    laser_geometry
    geometry_msgs
    rl_msgs
    message_runtime
    nav_msgs
    pcl_ros
  DEPENDS
    EIGEN3
)



###########
## Build ##
###########

include_directories(
  include
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

add_library(rlLocalPlanner 	
  src/rl_local_planner.cpp
  src/utils.cpp
)

add_dependencies(rlLocalPlanner
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(rlLocalPlanner
   ${catkin_LIBRARIES}
   ${boost_LIBRARIES}
)

add_executable(wp_generator
  src/wp_generator.cpp
  src/utils.cpp
)

add_dependencies(wp_generator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(wp_generator
  ${catkin_LIBRARIES}
)

add_executable(image_generator
  src/image_generator.cpp
  src/utils.cpp
)

add_dependencies(image_generator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(image_generator
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)



#############
## Install ##
#############

install(
  TARGETS 
    rlLocalPlanner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  FILES
    blp_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS
    wp_generator
    image_generator
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
