cmake_minimum_required(VERSION 2.8.3)
project(ubot_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  kdl_parser
  umass_math
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  ubot_msgs
  perception_msgs
  control_msgs
  umass_control_msgs
  ubot_spline_controller
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(orocos_kdl REQUIRED)
find_package(umass_control REQUIRED)


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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
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
#add_message_files(
#   FILES
#   UbotError.msg
#   Message2.msg
#)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  # ML 5/2017 I don't think these are actually used anywhere.
  #GazeTarget.srv
  #GazeControlFeedback.srv

  InitiateBracing.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  Bracing.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  geometry_msgs
  sensor_msgs
)

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
  #INCLUDE_DIRS include
  #LIBRARIES Controller
  CATKIN_DEPENDS actionlib geometry_msgs kdl_parser umass_math roscpp rospy sensor_msgs std_msgs ubot_msgs umass_control umass_control_msgs ubot_spline_controller
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${umass_control_INCLUDE_DIRS}

)
link_directories(${orocos_kdl_LIBRARY_DIRS})

## Declare a cpp library
# add_library(umass_control
   ## Add the Controller class and any objective functions to be used.
#   src/controller/Controller.cpp
#   src/objective_functions/EndpointCartesianPositionObj.cpp
#   src/objective_functions/PseudoForceObj.cpp
#   src/objective_functions/CosineFieldConditioningObj.cpp
# )


## Declare a cpp executable
#add_executable(bimanual_endpoint_pos_server src/nodes/bimanual_endpoint_pos_server.cpp)
#add_executable(bimanual_endpoint_pos_client src/nodes/bimanual_endpoint_pos_client.cpp)
#add_executable(bimanual_grasp_server src/nodes/bimanual_grasp_server.cpp)
#add_executable(bimanual_grasp_client src/nodes/bimanual_grasp_client.cpp)
#add_executable(bimanual_lift_server src/nodes/bimanual_lift_server.cpp)
#add_executable(bimanual_lift_client src/nodes/bimanual_lift_client.cpp)
#add_executable(force_position_controller_server src/nodes/force_position_controller_server.cpp)
#add_executable(force_position_controller_client src/nodes/force_position_controller_client.cpp)
#add_executable(grasp_server src/nodes/grasp_server.cpp)
#add_executable(grasp_client src/nodes/grasp_client.cpp)
#add_executable(drive_pose_server src/nodes/drive_pose_server.cpp)
#add_executable(force_position_rotation_controller_server src/nodes/force_position_rotation_controller_server.cpp )
#add_executable(force_position_rotation_controller_client src/nodes/force_position_rotation_controller_client.cpp )
#add_executable(force_rotation_controller_server src/nodes/force_rotation_controller_server.cpp )
#add_executable(force_rotation_controller_client src/nodes/force_rotation_controller_client.cpp )
#add_executable(rotate_client src/nodes/rotate_client.cpp )
#add_executable(rotate_server src/nodes/rotate_server.cpp )
add_executable(gaze_control_server src/nodes/gaze_control_server.cpp)
add_executable(gaze_control_client src/nodes/gaze_control_client.cpp)
add_executable(gaze_control_server_ubot7 src/nodes/gaze_control_server_ubot7.cpp)

# old gaze controller not using control library
#add_executable(gaze_controller src/nodes/gaze_controller.cpp)
#add_executable(bracing_control_server src/nodes/bracing_control_server.cpp)



## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(control_lib ubot_control_generate_messages_cpp ubot_msgs_gencpp)
#add_dependencies(bimanual_endpoint_pos_server ubot_msgs_gencpp umass_control_msgs_gencpp)
#add_dependencies(bimanual_endpoint_pos_client ubot_msgs_gencpp umass_control_msgs_gencpp)
#add_dependencies(bimanual_grasp_server ubot_msgs_gencpp umass_control_msgs_gencpp)
#add_dependencies(bimanual_grasp_client ubot_msgs_gencpp umass_control_msgs_gencpp)
#add_dependencies(bimanual_lift_server ubot_msgs_gencpp umass_control_msgs_gencpp)
#add_dependencies(bimanual_lift_client ubot_msgs_gencpp umass_control_msgs_gencpp)
#add_dependencies(drive_pose_server ubot_msgs_gencpp umass_control_msgs_gencpp)
#add_dependencies(force_position_controller_client ubot_msgs_gencpp umass_control_msgs_gencpp)
#add_dependencies(force_position_controller_server ubot_msgs_gencpp umass_control_msgs_gencpp)
#add_dependencies(grasp_server ubot_msgs_gencpp umass_control_msgs_gencpp)
#add_dependencies(grasp_client ubot_msgs_gencpp umass_control_msgs_gencpp)
#add_dependencies(rotate_server ubot_msgs_gencpp umass_control_msgs_gencpp)
#add_dependencies(rotate_client ubot_msgs_gencpp umass_control_msgs_gencpp)
#add_dependencies(force_position_rotation_controller_client ubot_msgs_gencpp umass_control_msgs_gencpp)
#add_dependencies(force_position_rotation_controller_server ubot_msgs_gencpp umass_control_msgs_gencpp)
#add_dependencies(force_rotation_controller_client ubot_msgs_gencpp umass_control_msgs_gencpp)
#add_dependencies(force_rotation_controller_server ubot_msgs_gencpp umass_control_msgs_gencpp)
add_dependencies(gaze_control_server ubot_msgs_gencpp umass_control_msgs_gencpp)
add_dependencies(gaze_control_client ubot_msgs_gencpp umass_control_msgs_gencpp)
add_dependencies(gaze_control_server_ubot7 ubot_msgs_gencpp umass_control_msgs_gencpp)

# old gaze controller
#add_dependencies(gaze_controller ubot_msgs_gencpp umass_control_msgs_gencpp)
#add_dependencies(bracing_control_server ubot_msgs_gencpp ubot_control_msgs_gencpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(ubot_control_node
#   ${catkin_LIBRARIES}
# )


target_link_libraries(gaze_control_server
  ${catkin_LIBRARIES}
  umass_control
  umass_matrix
)

target_link_libraries(gaze_control_client
  ${catkin_LIBRARIES}
  umass_control
  umass_matrix
)

target_link_libraries(gaze_control_server_ubot7
  ${catkin_LIBRARIES}
  umass_control
  umass_matrix
)

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
# install(TARGETS ubot_control ubot_control_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ubot_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
