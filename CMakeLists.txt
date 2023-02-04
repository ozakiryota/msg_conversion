cmake_minimum_required(VERSION 2.8.3)
project(msg_conversion)

add_compile_options(-std=c++17 -O2 -g -Wall)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  tf
  rosbag
  cv_bridge
)
find_package(PCL)
find_package(OpenCV)

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

add_executable(pc_ned_to_nwu src/pc_ned_to_nwu.cpp)
target_link_libraries(pc_ned_to_nwu
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(odometry_to_pose src/odometry_to_pose.cpp)
target_link_libraries(odometry_to_pose
  ${catkin_LIBRARIES}
)

add_executable(odometry_zxy_to_xyz src/odometry_zxy_to_xyz.cpp)
target_link_libraries(odometry_zxy_to_xyz
  ${catkin_LIBRARIES}
)

add_executable(odometry_twist_to_pose src/odometry_twist_to_pose.cpp)
target_link_libraries(odometry_twist_to_pose
  ${catkin_LIBRARIES}
)

add_executable(pose_to_tf src/pose_to_tf.cpp)
target_link_libraries(pose_to_tf
  ${catkin_LIBRARIES}
)

add_executable(image_64fc1_to_16uc1_offline src/image_64fc1_to_16uc1_offline.cpp)
target_link_libraries(image_64fc1_to_16uc1_offline
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(frame_rename_offline src/frame_rename_offline.cpp)
target_link_libraries(frame_rename_offline
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############
