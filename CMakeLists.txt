cmake_minimum_required(VERSION 2.8.3)
project(msg_conversion)

add_compile_options(-std=c++14 -O2 -g -Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)
find_package(PCL)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(pc_zxy_to_xyz src/pc_zxy_to_xyz.cpp)
target_link_libraries(pc_zxy_to_xyz
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(odometry_zxy_to_xyz src/odometry_zxy_to_xyz.cpp)
target_link_libraries(odometry_zxy_to_xyz
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############
