cmake_minimum_required(VERSION 2.8.3)
project(msg_conversion)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  PCL
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES msg_conversion
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(msg_conversion
#   src/${PROJECT_NAME}/msg_conversion.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(msg_conversion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(msg_conversion_node src/msg_conversion_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(msg_conversion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(msg_conversion_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS msg_conversion msg_conversion_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_msg_conversion.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
add_executable(odometry_reset_origin src/odometry_reset_origin.cpp)
target_link_libraries(odometry_reset_origin ${catkin_LIBRARIES})
add_executable(odometry_zxy_to_xyz src/odometry_zxy_to_xyz.cpp)
target_link_libraries(odometry_zxy_to_xyz ${catkin_LIBRARIES})
add_executable(posestamped_to_rpy src/posestamped_to_rpy.cpp)
target_link_libraries(posestamped_to_rpy ${catkin_LIBRARIES})
add_executable(odometry_to_rpy src/odometry_to_rpy.cpp)
target_link_libraries(odometry_to_rpy ${catkin_LIBRARIES})
add_executable(quatstamped_to_rpy src/quatstamped_to_rpy.cpp)
target_link_libraries(quatstamped_to_rpy ${catkin_LIBRARIES})
add_executable(posestamped_to_odometry src/posestamped_to_odometry.cpp)
target_link_libraries(posestamped_to_odometry ${catkin_LIBRARIES})
add_executable(odometry_3d_to_2d src/odometry_3d_to_2d.cpp)
target_link_libraries(odometry_3d_to_2d ${catkin_LIBRARIES})
add_executable(odometry_yaw_to_quaternion src/odometry_yaw_to_quaternion.cpp)
target_link_libraries(odometry_yaw_to_quaternion ${catkin_LIBRARIES})
add_executable(odometry_to_posestamped src/odometry_to_posestamped.cpp)
target_link_libraries(odometry_to_posestamped ${catkin_LIBRARIES})
add_executable(odometry_to_quaternionstamped src/odometry_to_quaternionstamped.cpp)
target_link_libraries(odometry_to_quaternionstamped ${catkin_LIBRARIES})
add_executable(infant_odometry_republish src/infant_odometry_republish.cpp)
target_link_libraries(infant_odometry_republish ${catkin_LIBRARIES})
add_executable(pc_rename_frameid src/pc_rename_frameid.cpp)
target_link_libraries(pc_rename_frameid ${catkin_LIBRARIES})
add_executable(sq2_odometry_republish src/sq2_odometry_republish.cpp)
target_link_libraries(sq2_odometry_republish ${catkin_LIBRARIES})
add_executable(odometry_input_linearvelocity src/odometry_input_linearvelocity.cpp)
target_link_libraries(odometry_input_linearvelocity ${catkin_LIBRARIES})
add_executable(combine_rp_y_linear src/combine_rp_y_linear.cpp)
target_link_libraries(combine_rp_y_linear ${catkin_LIBRARIES})
add_executable(combine_orientation_linear src/combine_orientation_linear.cpp)
target_link_libraries(combine_orientation_linear ${catkin_LIBRARIES})
add_executable(ned_to_neu src/ned_to_neu.cpp)
target_link_libraries(ned_to_neu ${catkin_LIBRARIES})
add_executable(pc_ned_to_neu src/pc_ned_to_neu.cpp)
target_link_libraries(pc_ned_to_neu ${catkin_LIBRARIES})
add_executable(odometry_linear_global_to_local src/odometry_linear_global_to_local.cpp)
target_link_libraries(odometry_linear_global_to_local ${catkin_LIBRARIES})
add_executable(imu_to_rpy src/imu_to_rpy.cpp)
target_link_libraries(imu_to_rpy ${catkin_LIBRARIES})
add_executable(imu_add_noise src/imu_add_noise.cpp)
target_link_libraries(imu_add_noise ${catkin_LIBRARIES})
add_executable(vector_to_arrow src/vector_to_arrow.cpp)
target_link_libraries(vector_to_arrow ${catkin_LIBRARIES})
add_executable(tf_to_posestamped src/tf_to_posestamped.cpp)
target_link_libraries(tf_to_posestamped ${catkin_LIBRARIES})
add_executable(posestamped_reset_origin src/posestamped_reset_origin.cpp)
target_link_libraries(posestamped_reset_origin ${catkin_LIBRARIES})
add_executable(gravity_to_rpy src/gravity_to_rpy.cpp)
target_link_libraries(gravity_to_rpy ${catkin_LIBRARIES})
